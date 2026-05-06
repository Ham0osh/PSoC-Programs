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

#include "app_ctx.h"
#include <string.h>

// Initializes the shared state
void app_ctx_init(app_ctx_t *ctx)
{
    memset(ctx, 0, sizeof *ctx);
    ctx->mode        = STANDBY;
    ctx->ctrl_mode   = HAMFLY_DEFER;
    ctx->sense       = SENSE_MED;
    ctx->invert_mask = (1u << CH_X);
    ctx->last_button = 1u;
    ctx->cmd.u[0]    = 0.0f;
    ctx->cmd.u[1]    = 0.0f;
    ctx->nudge_pan_rate   = 0.2f;
    ctx->nudge_tilt_rate  = 0.2f;
    ctx->nudge_time_short = 50u;
    ctx->nudge_time_long  =250u;
}

/* [] END OF FILE */
