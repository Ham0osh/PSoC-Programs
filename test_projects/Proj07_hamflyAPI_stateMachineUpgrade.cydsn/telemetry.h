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
    uint32_t t_ms;             //  PSoC monotonic ms
    uint8_t  state;            //  app state enum (leaf)
    uint8_t  ctrl_mode_pan;    //  hamfly_control_mode_t
    uint8_t  ctrl_mode_tilt;
    uint8_t  ctrl_mode_roll;
    uint8_t  enable;
    uint8_t  kill;
    int16_t  input_pan;        //  ctl.pan * 100 (units depend on mode)
    int16_t  input_tilt;
    int16_t  input_roll;
    uint8_t  gimbal_status1;   //  QX287 raw status bytes
    uint8_t  gimbal_status2;
    int16_t  q_i, q_j, q_k, q_r;  //  QX quaternion * 32767
} telem_hot_t;

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

// Helpers: Collect needed data into struct
void telemetry_collect_hot (const app_ctx_t *ctx, telem_hot_t  *out);
void telemetry_collect_link(const app_ctx_t *ctx, telem_link_t *out);
void telemetry_collect_power(const app_ctx_t *ctx, telem_power_t *out);
void telemetry_collect_env  (const app_ctx_t *ctx, telem_env_t   *out);

// Transport convenience function: collect + encode + sbc_send_frame
void telemetry_send_hot_sbc (const app_ctx_t *ctx);
void telemetry_send_link_sbc(const app_ctx_t *ctx);
void telemetry_send_power_sbc(const app_ctx_t *ctx);
void telemetry_send_env_sbc  (const app_ctx_t *ctx);

// Uses s_attrs[] to schedule getting the requested attributes.
void telemetry_pump(app_ctx_t *ctx);

#endif
/* [] END OF FILE */
