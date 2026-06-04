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
 * Shared utility functions.
*/

#include "utils.h"
#include "hamfly.h"
#include "app_statemachine.h"  // Raise error helper
//#include "debug_pins.h"  // Uncoment if we add pin macros here
#include <project.h>  // Pustring to UART_DEBUG

extern volatile uint32_t g_tick_ms;

uint8_t gimbal_pan_tilt_deg(const app_ctx_t *ctx, float *pan_deg, float *tilt_deg)
{
    if (!ctx->gimbal) return 0u;
    hamfly_telemetry_t st;
    hamfly_get_telemetry(ctx->gimbal, &st);
    if (!st.valid) return 0u;
    float roll_deg;
    return hamfly_telemetry_to_euler(&st, pan_deg, tilt_deg, &roll_deg);
}

// Nudge 1: Absolute from MANU Leaf
void nudge_apply(app_ctx_t *ctx, float dpan_deg, float dtilt_deg)
{
    if (!ctx->nudge_hold) {
        float p, t;
        // Ensure connected
        if (!gimbal_pan_tilt_deg(ctx, &p, &t)) {
            app_raise_error(ctx, SEV_USER, "no telemetry (nudge ignored).");
            return;
        }
        UART_DEBUG_PutString("Nudging!\r\n");
        // Save base and movement
        ctx->nudge_base_pan_deg  = p;
        ctx->nudge_base_tilt_deg = t;
        ctx->tgt_pan_deg  = p;
        ctx->tgt_tilt_deg = t;
        ctx->nudge_hold = 1u;
    }
    // Set targets after clamping to software limits
    ctx->tgt_pan_deg  = CLAMP(ctx->tgt_pan_deg  + dpan_deg,
                              ctx->origin_pan_deg + LIMIT_PAN_MIN_DEG,
                              ctx->origin_pan_deg + LIMIT_PAN_MAX_DEG);
    ctx->tgt_tilt_deg = CLAMP(ctx->tgt_tilt_deg + dtilt_deg,
                              ctx->origin_tilt_deg + LIMIT_TILT_MIN_DEG,
                              ctx->origin_tilt_deg + LIMIT_TILT_MAX_DEG);
    ctx->nudge_start_ms = g_tick_ms;
}

// NUDGE 2: Velocity from any state
// TODO: Is this a abs or a vel nudge?
// Is it safe and respectful to enter from any state?
void nudge_overlay_apply(app_ctx_t *ctx, float dpan_deg, float dtilt_deg)
{
    // Set rate targets for the burst, clamp to limits.
    ctx->tgt_pan_deg  = CLAMP(dpan_deg, LIMIT_PAN_MIN_DEG,  LIMIT_PAN_MAX_DEG);
    ctx->tgt_tilt_deg = CLAMP(dtilt_deg, LIMIT_TILT_MIN_DEG, LIMIT_TILT_MAX_DEG);
    ctx->nudge_active          = 1u;
    ctx->nudge_burst_start_ms  = g_tick_ms;
}

uint8_t handle_nudge_key(app_ctx_t *ctx, char k)
{
    switch (k) {
        case 'i': nudge_apply(ctx,  0,               +NUDGE_FINE_DEG);   return 1;
        case 'k': nudge_apply(ctx,  0,               -NUDGE_FINE_DEG);   return 1;
        case 'j': nudge_apply(ctx, -NUDGE_FINE_DEG,   0);                return 1;
        case 'l': nudge_apply(ctx, +NUDGE_FINE_DEG,   0);                return 1;
        case 'w': nudge_apply(ctx,  0,               +NUDGE_COARSE_DEG); return 1;
        case 's': nudge_apply(ctx,  0,               -NUDGE_COARSE_DEG); return 1;
        case 'a': nudge_apply(ctx, -NUDGE_COARSE_DEG, 0);                return 1;
        case 'd': nudge_apply(ctx, +NUDGE_COARSE_DEG, 0);                return 1;
        default:  return 0;
    }
}

/* [] END OF FILE */

