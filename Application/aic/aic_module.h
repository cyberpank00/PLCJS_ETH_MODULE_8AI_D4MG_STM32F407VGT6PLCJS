/**
  ******************************************************************************
  * @file    aic_module.h
  * @brief   Eight-channel 4–20 mA / 0–20 mA current acquisition on top of the
  *          ADS1220 driver (two converters × four single-ended inputs).
  *
  *  Each channel:
  *    - is converted single-shot through the ADS1220 multiplexer; a full scan
  *      is four conversions (both converters run in parallel),
  *    - converts the 24-bit code to an uncalibrated current using the nominal
  *      reference voltage and shunt:  I_raw = code/2^23 · V_REF / R_shunt,
  *    - applies the per-channel 2-point calibration  I = gain·I_raw + offset
  *      (removes shunt tolerance, reference error and the 1 kΩ / input-
  *      impedance divider),
  *    - optionally smooths the calibrated current with an EMA (level 0 = off,
  *      1..3 => alpha 1/4, 1/8, 1/16); the raw current and the code stay
  *      unfiltered,
  *    - flags faults from the value (the ADS1220 has no fault register):
  *      open loop (< AIC_OPEN_MA, 4–20 mA scale only), over-range
  *      (> AIC_OVER_MA or a saturated code), converter not responding,
  *    - drives its status LED (solid = active, off = inactive, fast blink =
  *      fault).
  ******************************************************************************
  */
#ifndef APPLICATION_AIC_MODULE_H
#define APPLICATION_AIC_MODULE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AIC_CHANNEL_COUNT           8u

/* Channel scale (settings.ch_range). */
#define AIC_RANGE_4_20MA            0u
#define AIC_RANGE_0_20MA            1u
#define AIC_RANGE_COUNT             2u

/* NAMUR NE43 style limits, mA (fixed). */
#define AIC_OPEN_MA                 3.6f    /* below: open loop (4–20 mA only) */
#define AIC_OVER_MA                 21.0f   /* above: over-range / short        */

/* Fault codes reported in aic_channel_status_t.fault_code. */
#define AIC_FAULT_NONE              0u
#define AIC_FAULT_OPEN              1u   /* open loop / below 3.6 mA          */
#define AIC_FAULT_OVER              2u   /* above 21 mA or saturated code     */
#define AIC_FAULT_ADC               3u   /* converter not responding (DRDY)   */

/* int16 views (holding registers 0..7 and 8..15). */
#define AIC_I16_FAULT               ((int16_t)-32768)
#define AIC_I16_DISABLED            ((int16_t)0)
#define AIC_I16_FULL                32767.0f
#define AIC_I16_CURRENT_FS_MA       20.0f   /* 32767 = 20.000 mA */

typedef struct {
    bool     enabled;
    bool     valid;         /* last conversion produced a usable value      */
    bool     fault;
    uint8_t  fault_code;    /* AIC_FAULT_*                                  */
    uint8_t  range;         /* AIC_RANGE_*                                  */
    int32_t  adc_code;      /* 24-bit signed conversion result              */
    float    i_raw_ma;      /* uncalibrated current, mA                     */
    float    i_cal_ma;      /* calibrated (and smoothed) current, mA        */
} aic_channel_status_t;

/** Initialise SPI, the ADS1220 converters and the runtime configuration. */
void aic_module_init(void);

/** Re-read settings into the runtime state (ranges, smoothing, ADC rate). */
void aic_module_apply_config(void);

/** Acquire all eight channels once. Blocks for four conversions (~210 ms at
 *  20 SPS); call from the dedicated acquisition task. */
void aic_module_tick(void);

/** Drive the channel status LEDs; call from a fast (e.g. 10 ms) tick. */
void aic_module_led_tick(uint16_t period_ms);

/** Read-only access to the latest per-channel status (NULL if out of range). */
const aic_channel_status_t* aic_module_get_status(uint8_t ch);

/** int16 current view: 0..32767 = 0..20 mA (clamped); 0 disabled, −32768 fault. */
int16_t aic_module_int16_current(uint8_t ch);

/** int16 percent-of-range view: 0..32767 = 0..100 % of the channel scale
 *  (4–20 or 0–20 mA), negative below the scale start; 0 disabled, −32768 fault. */
int16_t aic_module_int16_percent(uint8_t ch);

/** Configured scan period, ms (clamped; the scan itself may take longer). */
uint16_t aic_module_scan_period_ms(void);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_AIC_MODULE_H */
