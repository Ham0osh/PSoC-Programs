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
#include "app_statemachine.h"   // app_transition, app_raise_error, gimbal_pan_tilt_deg
#include <project.h>            // For UART
#include <math.h>               // fabsf

extern volatile uint32_t g_tick_ms;

// MANU_JOYSTICK %============================================================%
// On Enter: Enable joystick, print message.
void entry_manu_joystick(app_ctx_t *ctx)
{
    (void)ctx;
    UART_DEBUG_PutString("\r\n[MANU] joystick=rate  nudge: numpad=fine  "
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
        out->pan  =  ctx->cmd.u[CH_X];
        out->tilt = -ctx->cmd.u[CH_Y];
    }
}

// TEMP: Nudge handler
static void nudge_apply(app_ctx_t *ctx, float dpan_deg, float dtilt_deg)
{
    if (!ctx->nudge_hold) {
        float p, t;  // Capture nudge baseline
        if (!gimbal_pan_tilt_deg(ctx, &p, &t)) {
            app_raise_error(ctx, SEV_USER, "no telemetry (nudge ignored).");
            return;
        }
        UART_DEBUG_PutString("Nudging!\r\n");  // TODO: Verbose nudge print
        ctx->nudge_base_pan_deg  = p;
        ctx->nudge_base_tilt_deg = t;
        ctx->tgt_pan_deg  = p;
        ctx->tgt_tilt_deg = t;
        ctx->nudge_hold = 1u;
    }
    ctx->tgt_pan_deg  = CLAMP(ctx->tgt_pan_deg  + dpan_deg,
                              ctx->origin_pan_deg  + LIMIT_PAN_MIN_DEG,
                              ctx->origin_pan_deg  + LIMIT_PAN_MAX_DEG);
    ctx->tgt_tilt_deg = CLAMP(ctx->tgt_tilt_deg + dtilt_deg,
                              ctx->origin_tilt_deg + LIMIT_TILT_MIN_DEG,
                              ctx->origin_tilt_deg + LIMIT_TILT_MAX_DEG);
    ctx->nudge_start_ms = g_tick_ms;  // Restart the dwell timer
}

uint8_t key_manu_joystick(app_ctx_t *ctx, char k)
{
    switch (k) {
        // Numpad for loswest possible abs increment
        case '8': nudge_apply(ctx, 0,    +NUDGE_LSB_DEG);  return 1;  // Numpad
        case '2': nudge_apply(ctx, 0,    -NUDGE_LSB_DEG);  return 1;  // Numpad
        case '4': nudge_apply(ctx, -NUDGE_LSB_DEG,    0);  return 1;  // Numpad
        case '6': nudge_apply(ctx, +NUDGE_LSB_DEG,    0);  return 1;  // Numpad
        // ijkl for 0.5 degree
        case 'i': nudge_apply(ctx, 0,    +NUDGE_FINE_DEG);   return 1;  // ijkl
        case 'k': nudge_apply(ctx, 0,    -NUDGE_FINE_DEG);   return 1;  // ijkl
        case 'j': nudge_apply(ctx, -NUDGE_FINE_DEG,   0);    return 1;  // ijkl
        case 'l': nudge_apply(ctx, +NUDGE_FINE_DEG,   0);    return 1;  // ijkl
        // wasd for 1 degree
        case 'w': nudge_apply(ctx, 0,    +NUDGE_COARSE_DEG); return 1;  // wasd
        case 's': nudge_apply(ctx, 0,    -NUDGE_COARSE_DEG); return 1;  // wasd
        case 'a': nudge_apply(ctx, -NUDGE_COARSE_DEG, 0);    return 1;  // wasd
        case 'd': nudge_apply(ctx, +NUDGE_COARSE_DEG, 0);    return 1;  // wasd
        case 'e': app_transition(ctx, STBY_HOLD);            return 1;  // Exit
        default:  return 0;
    }
}

// Auto releases nudge flag once stationary.
void app_manual_tick(app_ctx_t *ctx)
{
    if (ctx->state != MANU_JOYSTICK || !ctx->nudge_hold) return;
    uint32_t elapsed = g_tick_ms - ctx->nudge_start_ms;
    if (elapsed < NUDGE_MIN_DWELL_MS) return;       /* let the step physically land */

    float p, t; uint8_t arrived = 0u;
    if (gimbal_pan_tilt_deg(ctx, &p, &t))
        arrived = (fabsf(p - ctx->tgt_pan_deg)  < NUDGE_SETTLE_DEG) &&
                  (fabsf(t - ctx->tgt_tilt_deg) < NUDGE_SETTLE_DEG);
    if (arrived || elapsed >= NUDGE_TIMEOUT_MS)
        ctx->nudge_hold = 0u;                        /* rate=0 holds the new attitude */
}

/* [] END OF FILE */
