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
 * Finite state machine code.
*/

#include "app_statemachine.h"
#include <project.h>
#include <string.h>
#include "hamfly.h"
#include "joystick.h"

#define NUDGE_HOME_TIMEOUT_MS 5000u

// Helper functions %=========================================================%
#define DEG_TO_UNIT(d) ((float)(d) / 180.0f)
// Declared in main.c
extern uint8_t get_euler_deg(float *pan_deg_out, float *tilt_deg_out);
extern hamfly_gimbal_t g_movi;

// Common keypress helper %===================================================%
uint8_t handle_common_keys(app_ctx_t *ctx, char ch, uint32_t now)
{
    (void)now;
    switch (ch) {
        case 'x':  /* exit to STANDBY */
            transition(ctx, STANDBY);
            return 1u;
        case 'k':  /* immediate kill: defer + kill flag set in build_control */
            transition(ctx, STANDBY);
            UART_1_PutString("[KILL]\r\n");
            return 1u;
        case 'z':  /* zero target to (0,0) */
            ctx->abs_pan_target    = 0.0f;
            ctx->abs_tilt_target   = 0.0f;
            UART_1_PutString("[ZERO]\r\n");
            return 1u;
        case '[':
            if (get_euler_deg(&ctx->origin_pan_deg, &ctx->origin_tilt_deg)) {
                ctx->origin_set = 1u;
                UART_1_PutString("[ORIGIN] set to current attitude\r\n");
            } else {
                UART_1_PutString("[ORIGIN] Error, no telemetry.");
            }
            return 1u;
        case ']':  /* reset to hardware origin */
            ctx->origin_set      = 0u;
            ctx->origin_pan_deg  = 0.0f;
            ctx->origin_tilt_deg = 0.0f;
            UART_1_PutString("[ORIGIN reset]\r\n");
            return 1u;
        default:
            return 0u;
    }
}

// State transitions %========================================================%
void transition(app_ctx_t *ctx, app_mode_t next)
{
    if (ctx->mode == next) return;  // Dont do anything for same to same.
 
    // Exit actions
    switch (ctx->mode) {
        case CONTROL:
            ctx->joystick_active   = 0u;
            break;
        case NUDGE:
            break;
        default: break;
    }
 
    ctx->mode = next;
 
    // Entry actions
    switch (next) {
        case STANDBY:
            ctx->ctrl_mode = HAMFLY_DEFER;
            UART_1_PutString("\r\n[STATE] -> STANDBY\r\n"
                            "(j: Joystick control,"
                            " c: Joystick calibrate,"
                            " n: Nudge,"
                            " ?/h: Calibrate)\r\n");
            break;
        case CONTROL:
            ctx->ctrl_mode = HAMFLY_DEFER;  // Defer as default entry, user cycles
            ctx->joystick_active = 1u;
            UART_1_PutString("\r\n[STATE] -> CONTROL\r\n"
                            "(l: cycle DEFER/RATE/ABS, s: sensitivity,"
                            " n: nudge, x: standby)\r\n");
            break;
        case NUDGE_HOME:{
            ctx->ctrl_mode         = HAMFLY_ABSOLUTE;
            ctx->abs_pan_target  = DEG_TO_UNIT(ctx->origin_pan_deg);
            ctx->abs_tilt_target = DEG_TO_UNIT(ctx->origin_tilt_deg);
            ctx->joystick_active   = 0u;
            ctx->home_entry_ms = g_tick_ms;  // TODO IMPLEMENT PROPER TIMER
            UART_1_PutString("\r\n[STATE] Homing to [0,0]...");
            break;
        }
        case NUDGE:{
            // Set to rate mode
            ctx->ctrl_mode         = HAMFLY_RATE;
            ctx->joystick_active   = 0u;
            UART_1_PutString("\r\n[STATE] -> NUDGE\r\n"
                             "(wasd slow, 8462 fast, "
                             "x: standby)\r\n");
            break;
        }
        case ERROR:
            ctx->ctrl_mode = HAMFLY_DEFER;
            UART_1_PutString("\r\n[STATE] -> ERROR: ");
            UART_1_PutString(ctx->error_msg ? ctx->error_msg : "(unspecified)");
            UART_1_PutString("\r\n  press <Enter> to return to STANDBY\r\n");
            break;
        case PI_SERIAL_TEST:
            UART_1_PutString("Not implemented.\r\n");
            break;
    }
}

// Error handling %===========================================================%
void set_error(app_ctx_t *ctx, const char *msg)
{
    ctx->error_msg = msg;
    transition(ctx, ERROR);
}


/* =========================================================================
 * State Handlers
 * ========================================================================= */

void state_standby(app_ctx_t *ctx, char ch, uint32_t now)
{
    (void)now;
    switch (ch) {
        case 'c':
            joystick_on_key('c');
            UART_1_PutString("[STANDBY] joystick calibration started\r\n");
            break;
        case 'j': transition(ctx, CONTROL); break;   // Joystick control
        case 'n': transition(ctx, NUDGE_HOME); break;// Nudge control
        case '?':
            UART_1_PutString("[STANDBY] j=control n=nudge c=cal "
                             "z=zero [=origin ]=reset x=standby\r\n");
            break;
        case 'h':
            UART_1_PutString("[STANDBY] j=control n=nudge c=cal "
                             "z=zero [=origin ]=reset x=standby\r\n");
            break;
        default: break;
    }
}

void state_control(app_ctx_t *ctx, char ch, uint32_t now)
{
    (void)now;
    switch (ch) {
        case 'l':
            ctx->ctrl_mode = (ctx->ctrl_mode == HAMFLY_DEFER) ? HAMFLY_RATE
                           : (ctx->ctrl_mode == HAMFLY_RATE)  ? HAMFLY_ABSOLUTE
                           :                                    HAMFLY_DEFER;
            break;
        case 's':
            ctx->sense = (ctx->sense == SENSE_LOW)  ? SENSE_MED
                       : (ctx->sense == SENSE_MED)  ? SENSE_HIGH
                       :                              SENSE_LOW;
            joystick_set_sensitivity(ctx->sense);
            break;
        case 'n': transition(ctx, NUDGE_HOME); break;
        default: break;
    }
}

void state_nudge(app_ctx_t *ctx, char ch, uint32_t now)
{
    (void)ctx; (void)ch; (void)now;
    /* TODO: pulse logic */
}

void state_nudge_home(app_ctx_t *ctx, char ch, uint32_t now)
{
    (void)ch;
    ctx->ctrl_mode = HAMFLY_ABSOLUTE;  // NOT IMPLEMENTED
    ctx->abs_pan_target = DEG_TO_UNIT(ctx->origin_pan_deg);
    ctx->abs_tilt_target = DEG_TO_UNIT(ctx->origin_tilt_deg);
    
    if ( (now - ctx->home_entry_ms) > NUDGE_HOME_TIMEOUT_MS ) {
        UART_1_PutString("Hommed\r\n");
        transition(ctx, NUDGE);
    }
}

void state_error(app_ctx_t *ctx, char ch, uint32_t now)
{
    (void)now;
    if (ch == '\r' || ch == '\n') {
        ctx->error_msg = NULL;
        transition(ctx, STANDBY);
    }
}

/* [] END OF FILE */
