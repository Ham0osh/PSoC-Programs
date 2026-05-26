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

#include <project.h>
#include "hamfly_core_control.h"    // for hamfly_control_mode_t
#include "hamfly_core_gimbal.h"     // For hamfly_gimbal_t 
#include "joystick.h"               // for joy_cmd_t, joy_sensitivity_t

// Channels for joystick ADC multiplexing.
#define CH_X   0u
#define CH_Y   1u

// Timing constants
#define CONTROL_PERIOD_MS     100u   // 10 Hz control TX
#define DIAG_PERIOD_MS        1000u  // 1 Hz diagnostic print
#define QUERY_TIMEOUT_MS      2000u  // max wait for attr response in QUERY

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
    MANU_JOYSTICK, MANU_NUDGE,
    // AUTO leaves
    AUTO_HOME,                      // Go to software origin
    AUTO_ACQ_GPS, AUTO_ACQ_SPIRAL,  // TODO: Open loop acquisition patterns
    AUTO_TRACKING,                  // TODO: Closed loop tracking
    AUTO_LOSS, AUTO_NO_LOCK,        // No lock states.
    // ERROR leaf (one leaf, severity carried on ctx)
    ERROR_ACTIVE,
    STATE_COUNT
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
    
    // Error
    err_sev_t err_sev;
    const char *err_msg;  // string literal; lifetime = program
    uint8_t    fatal_latched;  // 1 once FATAL fires, cleared only by ctrl+R
    
    // Control mode toggle, used for joystick.
    hamfly_control_mode_t ctrl_mode;   // DEFER, RATE, ABSOLUTE
    
    // Absolute target (used by AUTO_HOME, future tracking integrators)
    float    abs_pan_target;  // units [-1, +1]
    float    abs_tilt_target;
    
    // Origin (set by '[')
    float    origin_pan_deg;
    float    origin_tilt_deg;
    uint8_t  origin_set;
    
    // Telemetry / loop bookkeeping
    uint32_t last_tx_ms;
    uint32_t last_diag_ms;
    uint32_t state_entry_ms;  // wall clock at last transition; per-state timers
    uint8_t  diag_active;
    
    // Joystick context
    joy_cmd_t         cmd;
    joy_sensitivity_t sense;
    uint32_t          invert_mask;

    // TEMP: Manual control vars
    uint8_t nudge_hold;          /* 1 = streaming absolute target; 0 = joystick rate */
    uint32_t nudge_start_ms;     /* episode start, for the release timeout */
    float   nudge_base_pan_deg;  /* attitude captured at episode start */
    float   nudge_base_tilt_deg;
    float   tgt_pan_deg;         /* accumulated targets, MoVI frame degrees */
    float   tgt_tilt_deg;

    // TODO: Closed loop tracking vars.
} app_ctx_t;

void app_ctx_init(app_ctx_t *ctx);

// Manual control macros
#define DEG_TO_UNIT(d)   ((float)(d) / 180.0f)
#define UNIT_TO_DEG(u)   ((float)(u) * 180.0f)
#define CLAMP(v,lo,hi)   ((v)<(lo)?(lo):((v)>(hi)?(hi):(v)))
#define LIMIT_PAN_MAX_DEG   30.0f
#define LIMIT_PAN_MIN_DEG  -30.0f
#define LIMIT_TILT_MAX_DEG  20.0f
#define LIMIT_TILT_MIN_DEG -20.0f


#endif /* APP_CTX_H */

/* [] END OF FILE */