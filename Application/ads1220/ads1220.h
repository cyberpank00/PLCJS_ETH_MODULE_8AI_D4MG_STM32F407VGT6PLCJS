/**
  ******************************************************************************
  * @file    ads1220.h
  * @brief   Driver for the two TI ADS1220 24-bit delta-sigma ADCs of the 8AIC
  *          board. Each converter serves four single-ended current inputs
  *          through its input multiplexer (ADC0: AI0..AI3, ADC1: AI4..AI7).
  *
  *  Board wiring per input (identical for all eight channels):
  *
  *    AIn+ ── fuse ── R_shunt (nominal 89.9 Ω, 91 Ω on the schematic) ── GND_ISO
  *                 └── 1 kΩ ── AINx  (100 nF to GND_ISO, BAV99 clamps to 3V3)
  *
  *  REFP0/REFN0 = REF5020 2.048 V precision reference / GND_ISO, so with the
  *  PGA bypassed at gain 1 the full scale is exactly 2.048 V:
  *
  *      V_in  = code / 2^23 * V_REF
  *      I_in  = V_in / R_shunt          (20 mA ≈ 1.8 V ≈ 88 % FS at 89.9 Ω)
  *
  *  Acquisition is single-shot per channel, synchronised with the DRDY pins
  *  (wired through the isolators): WREG MUX → START → wait DRDY → RDATA. Both
  *  converters are started together and read together, so a full 8-channel
  *  scan costs four conversions. The data rate is configurable:
  *    0 = 20 SPS + simultaneous 50/60 Hz FIR (~52 ms per conversion)
  *    1 = 90 SPS                              (~12 ms)
  *    2 = 330 SPS                             (~3.5 ms)
  ******************************************************************************
  */
#ifndef APPLICATION_ADS1220_H
#define APPLICATION_ADS1220_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADS1220_CHIP_COUNT          2u
#define ADS1220_MUX_COUNT           4u    /* single-ended inputs per chip */
#define ADS1220_CHANNEL_COUNT       (ADS1220_CHIP_COUNT * ADS1220_MUX_COUNT)

/* SPI commands (Table 14). */
#define ADS1220_CMD_RESET           0x06u
#define ADS1220_CMD_START           0x08u
#define ADS1220_CMD_POWERDOWN       0x02u
#define ADS1220_CMD_RDATA           0x10u
#define ADS1220_CMD_RREG            0x20u   /* | (reg << 2) | (count - 1) */
#define ADS1220_CMD_WREG            0x40u   /* | (reg << 2) | (count - 1) */

/* Config register 0: MUX[7:4] GAIN[3:1] PGA_BYPASS[0]. */
#define ADS1220_MUX_AINx_AVSS(x)    ((uint8_t)((0x8u | ((x) & 3u)) << 4))
#define ADS1220_GAIN_1              (0x0u << 1)
#define ADS1220_PGA_BYPASS          0x01u

/* Config register 1: DR[7:5] MODE[4:3] CM[2] TS[1] BCS[0]. */
#define ADS1220_DR_20SPS            (0x0u << 5)
#define ADS1220_DR_90SPS            (0x2u << 5)
#define ADS1220_DR_330SPS           (0x4u << 5)
#define ADS1220_MODE_NORMAL         (0x0u << 3)
#define ADS1220_CM_SINGLE           0x00u
#define ADS1220_TS_ENABLE           0x02u

/* Config register 2: VREF[7:6] 50/60[5:4] PSW[3] IDAC[2:0]. */
#define ADS1220_VREF_EXT_REF0       (0x1u << 6)   /* REFP0 / REFN0 (REF5020) */
#define ADS1220_FIR_OFF             (0x0u << 4)
#define ADS1220_FIR_50_60HZ         (0x1u << 4)
#define ADS1220_IDAC_OFF            0x0u

/* Config register 3: I1MUX[7:5] I2MUX[4:2] DRDYM[1] — all off. */
#define ADS1220_CFG3_DEFAULT        0x00u

/* Data-rate selector (Modbus HR / settings). */
#define ADS1220_RATE_20SPS_FIR      0u
#define ADS1220_RATE_90SPS          1u
#define ADS1220_RATE_330SPS         2u
#define ADS1220_RATE_COUNT          3u

/* Full-scale code of the 24-bit two's-complement result. */
#define ADS1220_FULL_SCALE          8388608.0f   /* 2^23 */
#define ADS1220_CODE_MAX            8388607     /* 0x7FFFFF */

/** Bring the SPI bus up, reset both converters and program the static
 *  configuration for the given data rate (ADS1220_RATE_*). */
void ads1220_init(uint8_t rate);

/** Change the data rate (rewrites config registers 1/2 on both chips). */
void ads1220_set_rate(uint8_t rate);

/** Conversion time for a rate, ms (used for DRDY timeouts / scan budget). */
uint16_t ads1220_conv_time_ms(uint8_t rate);

/**
 * Convert one multiplexer input on every chip simultaneously (single-shot):
 * select AIN<mux> on both chips, START, wait for both DRDY, RDATA.
 * @param mux        input 0..3 (channel = chip*4 + mux)
 * @param codes_out  receives ADS1220_CHIP_COUNT sign-extended 24-bit codes
 * @param ok_out     per chip: false if DRDY never asserted or the config
 *                   readback failed (converter absent / unpowered)
 */
void ads1220_convert_mux(uint8_t mux, int32_t codes_out[ADS1220_CHIP_COUNT],
                         bool ok_out[ADS1220_CHIP_COUNT]);

/** Read back config register @p reg (0..3) of a chip, for diagnostics. */
uint8_t ads1220_read_reg(uint8_t chip, uint8_t reg);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_ADS1220_H */
