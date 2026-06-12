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
 * Parent STBY - For all standby states.
*/

#include "app_statemachine.h"  // app_transition
#include "app_state_stby.h"
#include "utils.h"  // gimble_pan_tilt
#include <project.h>  // For UART

// Timing
// TODO: Move into utils?
extern volatile uint32_t g_tick_ms;

// STBY_DEFER %===============================================================%
// On Enter: Print message, wait for user input.
void entry_stby_defer(app_ctx_t *ctx)
{
    ctx->telem_stable_since_ms  = 0u;
    UART_DEBUG_PutString("\r\n[STBY_DEFER] '1' hold, '2' manual, '3' auto,"
                         " '?' help\r\n> ");
}
// On Exit: None

// STBY_HOLD %================================================================%
// On Enter
void entry_stby_hold(app_ctx_t *ctx)
{
    (void)ctx;
    UART_DEBUG_PutString("\r\n[STBY_HOLD] rates zeroed\r\n> ");
}
// On Exit: None

// STBY Ticker %==============================================================%
void app_stby_tick(app_ctx_t *ctx)
{
    // %--- STBY_DEFER: auto flow advances to HOLD after 5 s good telemetry --%
    if (ctx->state == STBY_DEFER && ctx->auto_flow) {
        float p, t;
        if (gimbal_pan_tilt_deg(ctx, &p, &t)) {
            if (ctx->telem_stable_since_ms  == 0u)
                ctx->telem_stable_since_ms  = g_tick_ms;   // start the clock
            if (g_tick_ms - ctx->telem_stable_since_ms  >= TELEM_STABLE_MS) {
                UART_DEBUG_PutString("[STBY_DEFER] telemetry stable -> STBY_HOLD\r\n");
                app_transition(ctx, STBY_HOLD);
            }
        } else {
            ctx->telem_stable_since_ms  = 0u;  // bad frame resets the clock
        }
        return;
    }

    // Flow for STBY_HOLD: Advance to AUTO_ACQ_GPS if a target has been given.
    // %--- STBY_HOLD: auto flow advances to GPS slew if target is set ------%
    if (ctx->state == STBY_HOLD && ctx->auto_flow) {
        if (ctx->gps_target_set) {
            UART_DEBUG_PutString("[STBY_HOLD] GPS target set -> AUTO_ACQ_GPS\r\n");
            app_transition(ctx, AUTO_ACQ_GPS);
        }
        return;
    }
}


// STBY Key Handler %=========================================================%
uint8_t key_stby_hold(app_ctx_t *ctx, char k)
{
    if (handle_nudge_key(ctx, k)) return 1;
    switch (k) {
        default:  return 0;
    }
}

/* [] END OF FILE */
