/* ========================================
 *
 * Copyright Hamish Johnson, 2026
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF 
 * Quantum Internet Systems Lab (QISL),
 * Department of Physics, SFU, Canada.
 *
 * Author: Hamish Johnson (2026)
 *
 * ========================================
 *
 * Representation of in-memory outbound telemetry.
 * Agnostic and useable for SBC comms as well as local debug.
*/

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include "app_ctx.h"

// Hot packet
// Critical telemetry at 10 Hz
typedef struct {
    uint32_t t_ms;              //  PSoC monotonic ms
    uint8_t  state;             //  app state enum (leaf)
    uint8_t  ctrl_mode_pan;     //  hamfly_control_mode_t
    uint8_t  ctrl_mode_tilt;
    uint8_t  ctrl_mode_roll;
    uint8_t  flags;             // State machine flags (fatsal, origin, nudging)
    uint8_t  kill;
    int16_t  input_pan;         //  ctl.pan * 100 (units depend on mode)
    int16_t  input_tilt;
    int16_t  input_roll;
    uint8_t  gimbal_status1;    //  QX287 raw status bytes
    uint8_t  gimbal_status2;
    int16_t  q_i, q_j, q_k, q_r;//  QX quaternion * 32767
} telem_hot_t;

// For the uint8 state machine flag positions
#define HOT_FLAG_FATAL_LATCHED 0x01u
#define HOT_FLAG_ORIGIN_SET    0x02u
#define HOT_FLAG_NUDGE_HOLD    0x04u

// Link (SBC comms) packet
// SBC comms quality at 1 Hz
typedef struct {
    uint32_t t_ms;
    uint16_t crc_fail;
    uint16_t uart_err;
    uint16_t unknown_magic;
    uint32_t rx_pkt_count;
    uint16_t last_centroid_dt_ms;  //  coarse stream
} telem_link_t;

// Power telemetry from all relevant ATTRs for validation
// TODO: Validate and trim
typedef struct {
    uint32_t t_ms;
    uint16_t vbat_left_mv;     //  QX287 battery_v_left * 1000
    uint16_t vbat_right_mv;    //  QX287 battery_v_right * 1000
    uint16_t sysstat_batt_mv;  //  sysstat_batt_v * 1000
    int16_t  sysstat_batt_ma;  //  sysstat_batt_a * 1000
    uint8_t  sysstat_cpu_pct;
    int8_t   mcu_temp_c;       //  PSoC DieTemp, or 0 if component absent
} telem_power_t;

// Environmental telemetry from sensors
typedef struct {
    uint32_t t_ms;
    int16_t  sysstat_temp_c10;   //  sysstat_temp_c * 10 (0.1 deg C)
    uint32_t sysstat_press_Pa;   //  sysstat_pressure_mb * 100 (Pa)
} telem_env_t;

typedef struct {
    uint32_t t_ms;
    uint8_t  flags;         //  bit 0 = gps_valid, bit 1 = gps_locked
    uint8_t  sats;          //  sysstat_gps_sats (free from SYSSTAT pump)
    int32_t  raw_lat;       //  /1e7 deg
    int32_t  raw_lon;       //  /1e7 deg
    int32_t  raw_alt;
    int16_t  raw_spd;       //  /100 m/s
    int16_t  raw_hdg;
    uint16_t hacc;          //  /100 m
    uint16_t vacc;          //  /100 m
    uint16_t sacc;          //  /100 m/s
} telem_gps_t;

//  BARO — altitude + climb rate. 1 Hz cadence. Needs HAMFLY_ATTR_BARO pumped.
typedef struct {
    uint32_t t_ms;
    uint8_t  flags;        //  bit 0 = baro_valid
    int32_t  alt_dm;       //  baro_alt_m * 10 (0.1 m)
    int16_t  roc_cms;      //  baro_roc_ms * 100 (0.01 m/s)
} telem_baro_t;

typedef struct {
    uint32_t t_ms;
    uint8_t  flags;        //  bit 0 = mag_valid
    int16_t  x_raw;        //  mag_x * 1000
    int16_t  y_raw;        //  mag_y * 1000
    int16_t  z_raw;        //  mag_z * 1000
    int16_t  decl_raw;     //  mag_declination * 10
} telem_mag_t;

// Helpers: Collect needed data into struct
void telemetry_collect_hot  (const app_ctx_t *ctx, telem_hot_t  *out);
void telemetry_collect_link (const app_ctx_t *ctx, telem_link_t *out);
void telemetry_collect_power(const app_ctx_t *ctx, telem_power_t *out);
void telemetry_collect_env  (const app_ctx_t *ctx, telem_env_t  *out);
void telemetry_collect_gps  (const app_ctx_t *ctx, telem_gps_t  *out);
void telemetry_collect_baro (const app_ctx_t *ctx, telem_baro_t *out);
void telemetry_collect_mag  (const app_ctx_t *ctx, telem_mag_t *out);

// Transport convenience function: collect + encode + sbc_send_frame
void telemetry_send_hot_sbc  (const app_ctx_t *ctx);
void telemetry_send_link_sbc (const app_ctx_t *ctx);
void telemetry_send_power_sbc(const app_ctx_t *ctx);
void telemetry_send_env_sbc  (const app_ctx_t *ctx);
void telemetry_send_gps_sbc  (const app_ctx_t *ctx);
void telemetry_send_baro_sbc (const app_ctx_t *ctx);
void telemetry_send_mag_sbc (const app_ctx_t *ctx);

// Uses s_attrs[] to schedule getting the requested attributes.
void telemetry_pump(app_ctx_t *ctx);

#endif
/* [] END OF FILE */
