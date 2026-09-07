/**
  ******************************************************************************
  * @file    settings.h
  * @brief   Persistent settings stored in internal Flash with CRC32 protection.
  *          8AIC variant: network + per-channel current-input configuration.
  *          Calibration lives in the write-once calstore, not here.
  ******************************************************************************
  */
#ifndef APPLICATION_SETTINGS_H
#define APPLICATION_SETTINGS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Magic and version --------------------------------------------------------- */
#define SETTINGS_MAGIC          0x08AC4A57u
#define SETTINGS_VERSION        1u

#define SETTINGS_AI_CHANNELS    8u

/* Defaults ------------------------------------------------------------------ */
#define SETTINGS_DEF_SCAN_MS       250u
#define SETTINGS_SCAN_MS_MIN       50u
#define SETTINGS_SCAN_MS_MAX       5000u

#define SETTINGS_DEF_LED_MODE       2u    /* STATE_MACHINE */

#define SETTINGS_DEF_SLAVE_ID       1u
#define SETTINGS_DEF_TCP_PORT       502u

/* Network mode stored in settings.use_dhcp (field name kept for parity with
 * the sibling modules): 0 = STATIC, 1 = DHCP, 2 = LINK-LOCAL (169.254/16). */
#define NET_MODE_STATIC            0u
#define NET_MODE_DHCP              1u
#define NET_MODE_LINKLOCAL         2u

/* Factory default: link-local / "unconfigured" (assign IP via discovery tool or
 * a plain Modbus client on the labelled 169.254.x.y address). */
#define SETTINGS_DEF_USE_DHCP       NET_MODE_LINKLOCAL

#define SETTINGS_DEF_IP0            192u
#define SETTINGS_DEF_IP1            168u
#define SETTINGS_DEF_IP2            1u
#define SETTINGS_DEF_IP3            10u

#define SETTINGS_DEF_MASK0          255u
#define SETTINGS_DEF_MASK1          255u
#define SETTINGS_DEF_MASK2          255u
#define SETTINGS_DEF_MASK3          0u

#define SETTINGS_DEF_GW0            192u
#define SETTINGS_DEF_GW1            168u
#define SETTINGS_DEF_GW2            1u
#define SETTINGS_DEF_GW3            1u

/* Per-channel defaults. */
#define SETTINGS_DEF_CH_RANGE       0u    /* AIC_RANGE_4_20MA */

/* Software smoothing (EMA) levels: 0 = off, 1..3 => alpha 1/4, 1/8, 1/16. */
#define SETTINGS_SMOOTH_OFF         0u
#define SETTINGS_SMOOTH_MAX         3u
#define SETTINGS_DEF_CH_SMOOTH      SETTINGS_SMOOTH_OFF

/* ADS1220 data rate: 0 = 20 SPS + 50/60 Hz FIR, 1 = 90 SPS, 2 = 330 SPS. */
#define SETTINGS_DEF_ADC_RATE       0u

/* Nominal shunt (Ω) and reference (V). Starting estimates only — the
 * per-channel calibration removes shunt tolerance and reference error. The
 * schematic carries 91 Ω; 89.9 Ω is the purchasable substitute. */
#define SETTINGS_DEF_SHUNT          89.9f
#define SETTINGS_DEF_VREF           2.048f

/* LED mode codes ------------------------------------------------------------ */
typedef enum {
    LED_MODE_ALW_OFF       = 0,
    LED_MODE_ALW_ON        = 1,
    LED_MODE_STATE_MACHINE = 2,
} led_mode_t;

/**
 * Persistent settings structure. Layout is fixed and naturally aligned. Do not
 * reorder without bumping SETTINGS_VERSION.
 */
typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved0;

    uint16_t scan_ms;               /* full 8-channel scan period, ms         */
    uint16_t led_mode;              /* led_mode_t                             */

    uint16_t modbus_tcp_port;       /* default 502                            */
    uint8_t  modbus_slave_id;       /* default 1                              */
    uint8_t  use_dhcp;              /* net mode: 0=static,1=DHCP,2=link-local */

    uint8_t  ip[4];
    uint8_t  netmask[4];
    uint8_t  gateway[4];

    /* Per-channel configuration. */
    uint8_t  ch_enabled[SETTINGS_AI_CHANNELS];
    uint8_t  ch_range[SETTINGS_AI_CHANNELS];       /* 0 = 4–20 mA, 1 = 0–20 mA */
    uint8_t  ch_smooth[SETTINGS_AI_CHANNELS];      /* EMA level 0..3          */

    uint8_t  adc_rate;              /* ADS1220 data-rate selector 0..2        */
    uint8_t  reserved_a[3];

    /* Nominal shunt (Ω) and reference (V). */
    float    shunt_nominal;
    float    vref_nominal;

    char     name[16];              /* device name, NUL-padded (discovery)    */

    uint32_t crc32;                 /* CRC32 over all preceding bytes         */
} settings_t;

#define SETTINGS_NAME_LEN   16u

/* API ----------------------------------------------------------------------- */

/**
 * Initialise the settings subsystem. Loads settings from Flash; if the stored
 * image is invalid the structure is filled with defaults.
 *
 * @return true if the stored image was valid, false if defaults were applied.
 */
bool settings_init(void);

/** Reload defaults into the in-memory settings (does not write to flash). */
void settings_reset_to_defaults(void);

/** Persist the current in-memory settings to internal Flash. */
bool settings_save(void);

/** Get a pointer to the live in-memory settings. */
settings_t* settings_get(void);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_SETTINGS_H */
