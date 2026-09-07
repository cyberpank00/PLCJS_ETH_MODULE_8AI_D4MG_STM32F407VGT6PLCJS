/**
  ******************************************************************************
  * @file    ads1220.c
  * @brief   ADS1220 x2 driver implementation (see ads1220.h).
  ******************************************************************************
  */

#include "ads1220.h"

#include <stddef.h>

#include "cmsis_os.h"
#include "main.h"
#include "spi_bus.h"
#include "stm32f4xx_hal.h"

typedef struct {
    GPIO_TypeDef* port;
    uint16_t      pin;
} gpio_ref_t;

static const gpio_ref_t s_cs[ADS1220_CHIP_COUNT] = {
    { ADC0_CS_GPIO_Port, ADC0_CS_Pin },
    { ADC1_CS_GPIO_Port, ADC1_CS_Pin },
};

static const gpio_ref_t s_drdy[ADS1220_CHIP_COUNT] = {
    { ADC0_DRDY_GPIO_Port, ADC0_DRDY_Pin },
    { ADC1_DRDY_GPIO_Port, ADC1_DRDY_Pin },
};

/* Single-shot conversion time per rate (datasheet t_CONV, rounded up), ms. */
static const uint16_t s_conv_ms[ADS1220_RATE_COUNT] = { 52u, 12u, 4u };

static uint8_t s_rate = ADS1220_RATE_20SPS_FIR;
/* Last config register 0 written per chip — used as a liveness check. */
static uint8_t s_cfg0[ADS1220_CHIP_COUNT];

static inline void short_delay(uint32_t loops)
{
    for (volatile uint32_t i = 0; i < loops; i++) { __NOP(); }
}

static inline void cs_assert(uint8_t chip)
{
    HAL_GPIO_WritePin(s_cs[chip].port, s_cs[chip].pin, GPIO_PIN_RESET);
    short_delay(8u);   /* t_CSSC */
}

static inline void cs_release(uint8_t chip)
{
    short_delay(8u);   /* t_SCCS */
    HAL_GPIO_WritePin(s_cs[chip].port, s_cs[chip].pin, GPIO_PIN_SET);
    short_delay(8u);   /* t_CSH */
}

static void send_cmd(uint8_t chip, uint8_t cmd)
{
    cs_assert(chip);
    (void)spi_bus_transfer(cmd);
    cs_release(chip);
}

static void write_regs(uint8_t chip, uint8_t first, const uint8_t* vals, uint8_t count)
{
    cs_assert(chip);
    (void)spi_bus_transfer((uint8_t)(ADS1220_CMD_WREG | (first << 2) | (count - 1u)));
    for (uint8_t i = 0; i < count; i++) {
        (void)spi_bus_transfer(vals[i]);
    }
    cs_release(chip);
}

uint8_t ads1220_read_reg(uint8_t chip, uint8_t reg)
{
    if (chip >= ADS1220_CHIP_COUNT || reg > 3u) {
        return 0u;
    }
    cs_assert(chip);
    (void)spi_bus_transfer((uint8_t)(ADS1220_CMD_RREG | (reg << 2)));
    const uint8_t v = spi_bus_transfer(0x00u);
    cs_release(chip);
    return v;
}

static uint8_t cfg1_for_rate(uint8_t rate)
{
    uint8_t dr;
    switch (rate) {
    case ADS1220_RATE_90SPS:  dr = ADS1220_DR_90SPS;  break;
    case ADS1220_RATE_330SPS: dr = ADS1220_DR_330SPS; break;
    default:                  dr = ADS1220_DR_20SPS;  break;
    }
    return (uint8_t)(dr | ADS1220_MODE_NORMAL | ADS1220_CM_SINGLE);
}

static uint8_t cfg2_for_rate(uint8_t rate)
{
    /* The FIR notch is only valid together with the 20 SPS setting. */
    const uint8_t fir = (rate == ADS1220_RATE_20SPS_FIR) ? ADS1220_FIR_50_60HZ : ADS1220_FIR_OFF;
    return (uint8_t)(ADS1220_VREF_EXT_REF0 | fir | ADS1220_IDAC_OFF);
}

static void configure(uint8_t chip, uint8_t rate)
{
    const uint8_t regs[4] = {
        (uint8_t)(ADS1220_MUX_AINx_AVSS(0u) | ADS1220_GAIN_1 | ADS1220_PGA_BYPASS),
        cfg1_for_rate(rate),
        cfg2_for_rate(rate),
        ADS1220_CFG3_DEFAULT,
    };
    s_cfg0[chip] = regs[0];
    write_regs(chip, 0u, regs, 4u);
}

void ads1220_init(uint8_t rate)
{
    spi_bus_init();
    s_rate = (rate < ADS1220_RATE_COUNT) ? rate : ADS1220_RATE_20SPS_FIR;

    for (uint8_t c = 0; c < ADS1220_CHIP_COUNT; c++) {
        send_cmd(c, ADS1220_CMD_RESET);
    }
    /* t_RESET: >= 50 us + 32 modulator clocks before the next access. */
    short_delay(20000u);

    for (uint8_t c = 0; c < ADS1220_CHIP_COUNT; c++) {
        configure(c, s_rate);
    }
}

void ads1220_set_rate(uint8_t rate)
{
    if (rate >= ADS1220_RATE_COUNT) { return; }
    s_rate = rate;
    for (uint8_t c = 0; c < ADS1220_CHIP_COUNT; c++) {
        const uint8_t regs[2] = { cfg1_for_rate(rate), cfg2_for_rate(rate) };
        write_regs(c, 1u, regs, 2u);
    }
}

uint16_t ads1220_conv_time_ms(uint8_t rate)
{
    return (rate < ADS1220_RATE_COUNT) ? s_conv_ms[rate] : s_conv_ms[0];
}

static int32_t read_data(uint8_t chip)
{
    cs_assert(chip);
    (void)spi_bus_transfer(ADS1220_CMD_RDATA);
    const uint8_t b2 = spi_bus_transfer(0x00u);
    const uint8_t b1 = spi_bus_transfer(0x00u);
    const uint8_t b0 = spi_bus_transfer(0x00u);
    cs_release(chip);

    int32_t code = (int32_t)(((uint32_t)b2 << 16) | ((uint32_t)b1 << 8) | b0);
    if (code & 0x00800000) {
        code -= 0x01000000;   /* sign-extend the 24-bit two's complement */
    }
    return code;
}

static inline bool drdy_asserted(uint8_t chip)
{
    return HAL_GPIO_ReadPin(s_drdy[chip].port, s_drdy[chip].pin) == GPIO_PIN_RESET;
}

void ads1220_convert_mux(uint8_t mux, int32_t codes_out[ADS1220_CHIP_COUNT],
                         bool ok_out[ADS1220_CHIP_COUNT])
{
    mux &= 3u;

    /* Select the input on every chip, then start all conversions together so
     * the DRDY waits overlap instead of adding up. */
    for (uint8_t c = 0; c < ADS1220_CHIP_COUNT; c++) {
        s_cfg0[c] = (uint8_t)(ADS1220_MUX_AINx_AVSS(mux) | ADS1220_GAIN_1 | ADS1220_PGA_BYPASS);
        write_regs(c, 0u, &s_cfg0[c], 1u);
        codes_out[c] = 0;
        ok_out[c]    = false;
    }
    for (uint8_t c = 0; c < ADS1220_CHIP_COUNT; c++) {
        send_cmd(c, ADS1220_CMD_START);
    }

    /* Wait for DRDY on each chip: up to twice the conversion time plus margin.
     * The 1 ms tick granularity is negligible against the 20 SPS budget. */
    const uint32_t deadline = osKernelGetTickCount() + 2u * s_conv_ms[s_rate] + 10u;
    uint8_t pending = ADS1220_CHIP_COUNT;
    bool    done[ADS1220_CHIP_COUNT] = { false, false };

    while (pending > 0u) {
        for (uint8_t c = 0; c < ADS1220_CHIP_COUNT; c++) {
            if (!done[c] && drdy_asserted(c)) {
                done[c] = true;
                pending--;
                codes_out[c] = read_data(c);
                /* Liveness: an absent / unpowered converter never asserts
                 * DRDY through the isolator, but guard the readback as well. */
                ok_out[c] = (ads1220_read_reg(c, 0u) == s_cfg0[c]);
            }
        }
        if (pending == 0u) { break; }
        if ((int32_t)(osKernelGetTickCount() - deadline) >= 0) { break; }
        osDelay(1u);
    }
}
