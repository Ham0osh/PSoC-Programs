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
 * Shared context + state enums for the HFSM.
*/

#ifndef APP_CTX_H
#define APP_CTX_H

#include "debug_usbcdc.h"           // For debug UART to go to micro USB.
#include <project.h>
#include "hamfly_core_control.h"    // for hamfly_control_mode_t
#include "hamfly_core_gimbal.h"     // For hamfly_gimbal_t 

// Channels for joystick ADC multiplexing. Depricated.
#define CH_X   0u
#define CH_Y   1u

// Timing constants
#define CONTROL_PERIOD_MS       100u   // Must be faster than 5 Hz else gimble times out.
#define DIAG_PERIOD_MS          5000u  // 0.2 Hz diagnostic print
#define QUERY_TIMEOUT_MS        2000u  // max wait for attr response in QUERY
#define AUTO_HOLD_TO_SPIRAL_MS    10000u  // AUTO_HOLD -> SPIRAL if no centroid
#define TRACKING_LOSS_TIMEOUT_MS   5000u  // TRACKING -> LOSS if no centroid
#define NUDGE_BURST_MS               80u  // velocity burst, tbd.
#define TELEM_STABLE_MS         5000u  // Consecutive gimbal telemetry before leaving defer.

// TEMP: Nudge constants
#define NUDGE_LSB_DEG     (180.0f/32767.0f)  /* ~0.00549°, one on-wire LSB */
#define NUDGE_FINE_DEG    0.5f
#define NUDGE_COARSE_DEG  1.0f
#define NUDGE_SETTLE_DEG  0.05f   /* "arrived" tolerance */
#define NUDGE_MIN_DWELL_MS 150u   /* hold long enough for the step to land */
#define NUDGE_TIMEOUT_MS  2000u   /* give up if it never settles */
    
// HFSM states %==============================================================%
// ROOT is a sentinel; STBY/MANU/AUTO/ERROR are parents (never set directly).
// All other entries are leaves and ARE the only legal values of ctx.state.
typedef enum {
    ROOT = 0,
    // Parents
    STBY, MANU, AUTO, ERROR,
    // STBY leaves
    STBY_DEFER, STBY_HOLD,
    // MANU leaves
    MANU_JOYSTICK,
    // AUTO leaves
    AUTO_HOME,                      // Go to software origin
    AUTO_HOLD,                      // Hold vel=0 and look for centroids
    AUTO_ACQ_GPS, AUTO_ACQ_SPIRAL,  // TODO: Open loop acquisition patterns
    AUTO_TRACKING,                  // TODO: Closed loop tracking
    AUTO_LOSS, AUTO_NO_LOCK,        // No lock states.
    // ERROR leaf (one leaf, severity carried on ctx)
    ERROR_ACTIVE,  // Has flag for error severity
    ERROR_KILL,  // Manual trigger kill
    STATE_COUNT  // THis is the value of the max state!
} state_t;

// Error severity %===========================================================%
// USER  : invalid input; print and reject, no state change.
// WARN  : transient issue; ERROR_ACTIVE, ack -> STBY_HOLD.
// SOFTWARE: internal bug / bad assumption; ERROR_ACTIVE, ack -> STBY_HOLD.
// FATAL : hardware/protocol break; ERROR_ACTIVE, ctl.kill latched, 
//         only Ctrl+R / power cycle clears.
typedef enum {
    SEV_USER = 0,
    SEV_WARN,
    SEV_SOFTWARE,
    SEV_FATAL
} err_sev_t;

// Shared context %===========================================================%
typedef struct {
    // Shared gimbal context for HamflyAPI. Initialized in main, never changes.
    hamfly_gimbal_t *gimbal;

    // HFSM
    state_t   state;  // current leaf (never a parent)
    state_t   prev_leaf;  // for return-to-last-state ergonomics
    // HFSM Flow
    uint8_t  auto_flow;  // Bool, if true then auto flow through states
    uint8_t  home_return_stby;  // Bool, if true then AUTO_HOME exits to STBY
    // Timer for how long in AUTO HOLD
    uint32_t auto_hold_entry_ms;
    // Timer for good telemetry from gimbal
    uint32_t telem_stable_since_ms;
    
    // Error
    err_sev_t   err_sev;
    const char *err_msg;  // string literal; lifetime = program
    uint8_t     fatal_latched;  // 1 once FATAL fires, cleared only by ctrl+R
    
    // Control mode toggle, used for joystick.
    // Depricated, joystick always rate now.
    // hamfly_control_mode_t ctrl_mode;   // DEFER, RATE, ABSOLUTE
    hamfly_control_mode_t hold_mode;
    
    // Absolute target (used by AUTO_HOME, and eventualy GPS pointing too.)
    float    abs_pan_target;  // units [-1, +1]
    float    abs_tilt_target;
    
    // GPS Pointing target
    // Written by app_sbc_tick any time.
    // Read only by app_sbc_tick and AUTO_ACQ_GPS.
    int32_t  gps_target_lat_raw;   // /1e7 deg
    int32_t  gps_target_lon_raw;   // /1e7 deg
    int32_t  gps_target_alt_mm;    // mm above MSL
    uint8_t  gps_target_set;       // 1 once any target has been received
    uint8_t  gps_new_target;       // 1 when a new target arrived during ACQ_GPS

    // Working copy of coordinates to point at so we dont overwrite mid point.
    int32_t  gps_work_lat_raw;
    int32_t  gps_work_lon_raw;
    int32_t  gps_work_alt_mm;

    // Settle state (valid only while in AUTO_ACQ_GPS)
    uint8_t  gps_settled;
    float    gps_last_pan_deg;
    float    gps_last_tilt_deg;
    uint32_t gps_last_sample_ms;

    // AUTO_TRACKING proportional control
    float    track_kp;             // P gain  (norm_rate per mrad)
    float    track_ki;             // I gain  (norm_rate per (mrad·s))
    float    track_kd;             // D gain  (norm_rate per (mrad/s))
    float    track_rate_max;

    // PID running state values
    float    track_i_pan;
    float    track_i_tilt;
    float    track_pan_cmd;   // Cached command
    float    track_tilt_cmd;

    // TODO: track_kd;
    int16_t  track_cx_last;
    int16_t  track_cy_last;
    
    // Origin (set by '[')
    // Defines relative origin for software attitude limits.
    float    origin_pan_deg;
    float    origin_tilt_deg;
    uint8_t  origin_set;
    
    // Telemetry / loop bookkeeping
    uint32_t last_tx_ms;
    uint32_t last_diag_ms;
    uint32_t state_entry_ms;  // wall clock at last transition; per-state timers
    uint8_t  diag_active;

    // TEMP: Manual control vars
    uint8_t  nudge_hold;          // 1 = streaming absolute target; 0 = joystick rate
    uint32_t nudge_start_ms;      // episode start, for the release timeout
    float    nudge_base_pan_deg;  // attitude captured at episode start
    float    nudge_base_tilt_deg;
    float    tgt_pan_deg;         // accumulated targets, MoVI frame degree
    float    tgt_tilt_deg;
    uint8_t  nudge_active;          // 1 = nudge burst in progress, suppresses state build
    uint32_t nudge_burst_start_ms;  // when the current burst started

} app_ctx_t;

void app_ctx_init(app_ctx_t *ctx);

// Manual control macros
#define DEG_TO_UNIT(d)   ((float)(d) / 180.0f)
#define UNIT_TO_DEG(u)   ((float)(u) * 180.0f)
#define DEG_TO_RAD            0.017453f
#define RAD_TO_DEG            57.2958f
#define CLAMP(v,lo,hi)   ((v)<(lo)?(lo):((v)>(hi)?(hi):(v)))
// Firmware limits relative to software origin
#define LIMIT_PAN_MAX_DEG   30.0f
#define LIMIT_PAN_MIN_DEG  -30.0f
#define LIMIT_TILT_MAX_DEG  20.0f
#define LIMIT_TILT_MIN_DEG -20.0f

#define TRACK_KP_DEFAULT      0.001f    // norm_rate per mrad, tune on hardware
#define TRACK_KI_DEFAULT        0.0f    // Off
#define TRACK_KD_DEFAULT        0.0f    // Off
#define TRACK_RATE_MAX_DEFAULT  0.20f   // Hard cut off for oscillations.

// Todo: Update with relevant params for pointing when in Vancouver and in Waterloo.
// Todo: Think about letting the GPS hand off early as long as speed is low enough, and centroids coming.
#define GPS_EARTH_R_M         6371000.0f
#define GPS_POINT_SETTLE_DEG  0.5f      // "arrived" tolerance for slew-done
#define GPS_RATE_SETTLE_DPS   1.0f      // "slow" threshold for slew-done
#define GPS_SETTLE_PERIOD_MS  50u       // settle-check / rate finite-diff cadence

#endif /* APP_CTX_H */

/* [] END OF FILE */