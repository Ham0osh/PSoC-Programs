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
 * Parent MANU - For all manual states.
*/

#include "app_state_manu.h"
#include "app_statemachine.h"   // app_transition, app_raise_error
#include "utils.h"              // Euler angles calc and nudge def
#include <project.h>            // For UART
#include <math.h>               // fabsf

extern volatile uint32_t g_tick_ms;

// MANU_JOYSTICK %============================================================%
// Entry Guard: Must be in a stationary hold.
uint8_t guard_manu_joystick(const app_ctx_t *ctx)
{
    return (ctx->state == STBY_HOLD);
}

// On Enter: Enable joystick, print message.
void entry_manu_joystick(app_ctx_t *ctx)
{
    (void)ctx;
    UART_DEBUG_PutString("\r\n[MANU] nudge: numpad=fine  "
                         "ijkl=0.5 deg wasd=1 deg e=exit\r\n> ");
}
// On Exit: clear any nudge in progress.
void exit_manu_joystick(app_ctx_t *ctx)
{
    ctx->nudge_hold = 0u;
}

// Build packet from joystick inputs (or nudge absolute target).
void build_manu_joystick(const app_ctx_t *ctx, hamfly_control_t *out)
{
    out->roll_mode = HAMFLY_DEFER;
    out->roll      = 0.0f;
    out->kill      = 0u;
    if (ctx->nudge_hold) {
        // In nudge: pan is relative. TODO: Document why.
        out->pan_mode = out->tilt_mode = HAMFLY_ABSOLUTE;
        out->pan  = DEG_TO_UNIT(ctx->tgt_pan_deg - ctx->nudge_base_pan_deg);
        out->tilt = DEG_TO_UNIT(ctx->tgt_tilt_deg);
    } else {
        out->pan_mode = out->tilt_mode = HAMFLY_RATE;
        out->pan  = 0.0f;  // Joysticks removed so hold and let user nudge.
        out->tilt = 0.0f;
    }
}

uint8_t key_manu_joystick(app_ctx_t *ctx, char k)
{
    if (handle_nudge_key(ctx, k)) return 1;
    switch (k) {
        // Numpad for loswest possible abs increment
        case '8': nudge_apply(ctx, 0,    +NUDGE_LSB_DEG);  return 1;  // Numpad
        case '2': nudge_apply(ctx, 0,    -NUDGE_LSB_DEG);  return 1;  // Numpad
        case '4': nudge_apply(ctx, -NUDGE_LSB_DEG,    0);  return 1;  // Numpad
        case '6': nudge_apply(ctx, +NUDGE_LSB_DEG,    0);  return 1;  // Numpad
        case 'e': app_transition(ctx, STBY_HOLD);            return 1;  // Exit
        default:  return 0;
    }
}

// Auto releases nudge flag once stationary.
void app_manual_tick(app_ctx_t *ctx)
{
    if (!ctx->nudge_hold) return;
    uint32_t elapsed = g_tick_ms - ctx->nudge_start_ms;
    if (elapsed < NUDGE_MIN_DWELL_MS) return;  // Wait for the nudge to land

    float p, t;
    uint8_t arrived = 0u;
    if (gimbal_pan_tilt_deg(ctx, &p, &t))
        arrived = (fabsf(p - ctx->tgt_pan_deg)  < NUDGE_SETTLE_DEG) &&
                  (fabsf(t - ctx->tgt_tilt_deg) < NUDGE_SETTLE_DEG);
    if (arrived || elapsed >= NUDGE_TIMEOUT_MS)
        ctx->nudge_hold = 0u;  // Hold from here
}

/* [] END OF FILE */
