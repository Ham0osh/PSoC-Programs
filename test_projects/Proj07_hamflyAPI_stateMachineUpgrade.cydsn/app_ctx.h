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
 * Shared context for state machine.
*/

#ifndef APP_CTX_H
#define APP_CTX_H

#include <project.h>
#include "hamfly_core_control.h"   /* for hamfly_control_mode_t */
#include "joystick.h"              /* for joy_cmd_t, joy_sensitivity_t */

    
    
typedef enum {
    STANDBY = 0,
    CONTROL,
    NUDGE_HOME,
    NUDGE,
    ERROR,
    PI_SERIAL_TEST
} app_mode_t;

typedef struct {
    // State machine
    app_mode_t            mode;
    hamfly_control_mode_t ctrl_mode;  // DEFER, RATE, ABSOLUTE
    const char           *error_msg;  // Message shown on ERROR state
    
    uint8_t  joystick_active;
    // Used for GPS slew after calculated
    float    abs_pan_target;          // Units [-1, +1
    float    abs_tilt_target;
 
    float    nudge_pan_rate;
    float    nudge_tilt_rate;
    uint8_t  nudge_time_short;
    uint8_t  nudge_time_long;
    uint8_t  home_entry_ms;  // Timer for homing
 
    float    origin_pan_deg;
    float    origin_tilt_deg;
    uint8_t  origin_set;
 
    // Telemetry context
    uint32_t last_tx_ms;
    uint32_t last_diag_ms;
    uint8_t  diag_active;
    uint8_t  show_platform_yaw;
    uint8_t  ctrl_attr_pending;
 
    // Joystick context
    joy_cmd_t         cmd;
    joy_sensitivity_t sense;
    uint32_t          invert_mask;
    uint8_t           last_button;
 
    /* TBD Closed loop tracking
     * uint8_t tracking_on;
     * float   track_pan_rate;   // computed by tracker each loop
     * float   track_tilt_rate;
     */
} app_ctx_t;  // Shared context between states

void app_ctx_init(app_ctx_t *ctx);

#endif /* APP_CTX_H */
/* [] END OF FILE */
