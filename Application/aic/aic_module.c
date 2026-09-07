/**
  ******************************************************************************
  * @file    aic_module.c
  * @brief   Eight-channel current acquisition (see aic_module.h).
  ******************************************************************************
  */

#include "aic_module.h"

#include <math.h>
#include <stddef.h>

#include "ads1220.h"
#include "calstore.h"
#include "main.h"
#include "settings.h"
#include "stm32f4xx_hal.h"

/* Codes at or above this are a saturated input (over-range regardless of the
 * computed current). */
#define AIC_CODE_SATURATED      0x7FF000

/* Channel status LED blink half-period on fault, ms. */
#define AIC_FAULT_BLINK_MS      100u

typedef struct {
    GPIO_TypeDef* port;
    uint16_t      pin;
} gpio_ref_t;

static const gpio_ref_t s_stat[AIC_CHANNEL_COUNT] = {
    { AI0_STAT_GPIO_Port, AI0_STAT_Pin }, { AI1_STAT_GPIO_Port, AI1_STAT_Pin },
    { AI2_STAT_GPIO_Port, AI2_STAT_Pin }, { AI3_STAT_GPIO_Port, AI3_STAT_Pin },
    { AI4_STAT_GPIO_Port, AI4_STAT_Pin }, { AI5_STAT_GPIO_Port, AI5_STAT_Pin },
    { AI6_STAT_GPIO_Port, AI6_STAT_Pin }, { AI7_STAT_GPIO_Port, AI7_STAT_Pin },
};

/* Runtime, derived from settings on aic_module_apply_config(). */
static aic_channel_status_t s_status[AIC_CHANNEL_COUNT];
static bool     s_enabled[AIC_CHANNEL_COUNT];
static uint8_t  s_range[AIC_CHANNEL_COUNT];
static uint8_t  s_smooth[AIC_CHANNEL_COUNT];
static uint16_t s_scan_ms = SETTINGS_DEF_SCAN_MS;
static uint8_t  s_rate    = SETTINGS_DEF_ADC_RATE;

/* Software smoothing (EMA): y += alpha * (x - y), alpha = 1 / 2^(level+1).
 * Seeded with the first valid sample, reset on fault and on any
 * configuration change so it never drags a stale value. */
static float    s_ema[AIC_CHANNEL_COUNT];
static bool     s_ema_seeded[AIC_CHANNEL_COUNT];

/* LED blink state. */
static uint16_t s_blink_timer;
static uint8_t  s_blink_on;

static inline void stat_set(uint8_t ch, bool on)
{
    HAL_GPIO_WritePin(s_stat[ch].port, s_stat[ch].pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void load_settings(void)
{
    const settings_t* s = settings_get();

    s_scan_ms = s->scan_ms;
    if (s_scan_ms < SETTINGS_SCAN_MS_MIN) { s_scan_ms = SETTINGS_SCAN_MS_MIN; }
    if (s_scan_ms > SETTINGS_SCAN_MS_MAX) { s_scan_ms = SETTINGS_SCAN_MS_MAX; }

    s_rate = (s->adc_rate < ADS1220_RATE_COUNT) ? s->adc_rate : SETTINGS_DEF_ADC_RATE;

    for (uint8_t ch = 0; ch < AIC_CHANNEL_COUNT; ch++) {
        s_enabled[ch] = (s->ch_enabled[ch] != 0u);
        s_range[ch]   = (s->ch_range[ch] < AIC_RANGE_COUNT) ? s->ch_range[ch] : AIC_RANGE_4_20MA;

        uint8_t smooth = s->ch_smooth[ch];
        if (smooth > SETTINGS_SMOOTH_MAX) { smooth = SETTINGS_SMOOTH_OFF; }
        s_smooth[ch] = smooth;

        s_ema_seeded[ch] = false;
    }
}

void aic_module_init(void)
{
    for (uint8_t ch = 0; ch < AIC_CHANNEL_COUNT; ch++) {
        s_status[ch].i_raw_ma = NAN;
        s_status[ch].i_cal_ma = NAN;
        stat_set(ch, false);
    }
    load_settings();
    ads1220_init(s_rate);
}

void aic_module_apply_config(void)
{
    const uint8_t old_rate = s_rate;
    load_settings();
    if (s_rate != old_rate) {
        ads1220_set_rate(s_rate);
    }
}

static void set_fault(aic_channel_status_t* st, uint8_t ch, uint8_t code)
{
    st->fault        = true;
    st->fault_code   = code;
    st->valid        = false;
    st->i_raw_ma     = NAN;
    st->i_cal_ma     = NAN;
    s_ema_seeded[ch] = false;
}

static void set_disabled(aic_channel_status_t* st, uint8_t ch)
{
    st->fault        = false;
    st->fault_code   = AIC_FAULT_NONE;
    st->valid        = false;
    st->adc_code     = 0;
    st->i_raw_ma     = NAN;
    st->i_cal_ma     = NAN;
    s_ema_seeded[ch] = false;
}

void aic_module_tick(void)
{
    const settings_t* s = settings_get();
    const float ma_per_code = (s->vref_nominal / s->shunt_nominal) * 1000.0f / ADS1220_FULL_SCALE;

    for (uint8_t mux = 0; mux < ADS1220_MUX_COUNT; mux++) {
        /* Skip the conversion entirely if no channel on this mux input is
         * enabled on either chip — saves a full conversion slot. */
        bool any = false;
        for (uint8_t chip = 0; chip < ADS1220_CHIP_COUNT; chip++) {
            if (s_enabled[chip * ADS1220_MUX_COUNT + mux]) { any = true; }
        }

        int32_t codes[ADS1220_CHIP_COUNT] = { 0, 0 };
        bool    ok[ADS1220_CHIP_COUNT]    = { false, false };
        if (any) {
            ads1220_convert_mux(mux, codes, ok);
        }

        for (uint8_t chip = 0; chip < ADS1220_CHIP_COUNT; chip++) {
            const uint8_t ch = (uint8_t)(chip * ADS1220_MUX_COUNT + mux);
            aic_channel_status_t* st = &s_status[ch];

            st->enabled = s_enabled[ch];
            st->range   = s_range[ch];

            if (!st->enabled) {
                set_disabled(st, ch);
                continue;
            }

            st->adc_code = codes[chip];
            if (!ok[chip]) {
                set_fault(st, ch, AIC_FAULT_ADC);
                continue;
            }

            int32_t code = codes[chip];
            if (code < 0) { code = 0; }   /* noise around a genuine 0 mA input */

            const float i_raw = (float)code * ma_per_code;
            float       i_cal = calstore_gain(ch, 0u) * i_raw + calstore_offset(ch, 0u);

            if (codes[chip] >= AIC_CODE_SATURATED || i_cal > AIC_OVER_MA) {
                set_fault(st, ch, AIC_FAULT_OVER);
                continue;
            }
            if (s_range[ch] == AIC_RANGE_4_20MA && i_cal < AIC_OPEN_MA) {
                set_fault(st, ch, AIC_FAULT_OPEN);
                continue;
            }

            st->fault      = false;
            st->fault_code = AIC_FAULT_NONE;
            st->valid      = true;

            if (s_smooth[ch] != SETTINGS_SMOOTH_OFF) {
                if (!s_ema_seeded[ch]) {
                    s_ema[ch]        = i_cal;
                    s_ema_seeded[ch] = true;
                } else {
                    const float alpha = 1.0f / (float)(1u << (s_smooth[ch] + 1u));
                    s_ema[ch] += alpha * (i_cal - s_ema[ch]);
                }
                i_cal = s_ema[ch];
            }

            st->i_raw_ma = i_raw;
            st->i_cal_ma = i_cal;
        }
    }
}

static int16_t clamp_i16(float v, float lo)
{
    if (v > AIC_I16_FULL) { v = AIC_I16_FULL; }
    if (v < lo)           { v = lo; }
    return (int16_t)lroundf(v);
}

int16_t aic_module_int16_current(uint8_t ch)
{
    if (ch >= AIC_CHANNEL_COUNT) { return AIC_I16_DISABLED; }
    const aic_channel_status_t* st = &s_status[ch];
    if (!st->enabled)            { return AIC_I16_DISABLED; }
    if (st->fault || !st->valid) { return AIC_I16_FAULT; }
    return clamp_i16(st->i_cal_ma / AIC_I16_CURRENT_FS_MA * AIC_I16_FULL, 0.0f);
}

int16_t aic_module_int16_percent(uint8_t ch)
{
    if (ch >= AIC_CHANNEL_COUNT) { return AIC_I16_DISABLED; }
    const aic_channel_status_t* st = &s_status[ch];
    if (!st->enabled)            { return AIC_I16_DISABLED; }
    if (st->fault || !st->valid) { return AIC_I16_FAULT; }

    const float lo   = (st->range == AIC_RANGE_4_20MA) ? 4.0f : 0.0f;
    const float span = 20.0f - lo;
    return clamp_i16((st->i_cal_ma - lo) / span * AIC_I16_FULL, -AIC_I16_FULL);
}

void aic_module_led_tick(uint16_t period_ms)
{
    s_blink_timer = (uint16_t)(s_blink_timer + period_ms);
    if (s_blink_timer >= AIC_FAULT_BLINK_MS) {
        s_blink_timer = 0u;
        s_blink_on    = (uint8_t)(!s_blink_on);
    }

    for (uint8_t ch = 0; ch < AIC_CHANNEL_COUNT; ch++) {
        if (!s_enabled[ch]) {
            stat_set(ch, false);
        } else if (s_status[ch].fault) {
            stat_set(ch, s_blink_on != 0u);
        } else {
            stat_set(ch, true);
        }
    }
}

const aic_channel_status_t* aic_module_get_status(uint8_t ch)
{
    return (ch < AIC_CHANNEL_COUNT) ? &s_status[ch] : NULL;
}

uint16_t aic_module_scan_period_ms(void)
{
    return s_scan_ms;
}
