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
 * Struct init for state machine chared context.
*/

#include "app_ctx.h"
#include <string.h>

void app_ctx_init(app_ctx_t *ctx)
{
    memset(ctx, 0, sizeof *ctx);
    ctx->diag_active   = 1u;
    // State context
    ctx->state         = STBY_DEFER;  // boot to defer
    ctx->prev_leaf     = STBY_DEFER;  // boot to defer
    // Orientation context
    ctx->hold_mode = HAMFLY_RATE;
    ctx->origin_set      = 1u;  // Origin initialy 0,0.
    ctx->gps_target_set  = 0u;  // TODO: Refactor shared app_ctx and debloat gps params.
    ctx->gps_new_target  = 0u;
    ctx->gps_settled     = 0u;
    // Tracking context
    ctx->track_kp        = TRACK_KP_DEFAULT;
    ctx->track_cx_last   = 0;
    ctx->track_cy_last   = 0;
}

/* [] END OF FILE */