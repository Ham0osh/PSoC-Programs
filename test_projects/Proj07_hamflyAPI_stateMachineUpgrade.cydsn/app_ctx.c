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
    ctx->state         = STBY_DEFER;  // boot to defer
    ctx->prev_leaf     = STBY_DEFER;  // boot to defer
    ctx->ctrl_mode     = HAMFLY_DEFER;
    ctx->origin_set    = 1u;          // Origin initialy 0,0.
    ctx->sense         = SENSE_MED;
    ctx->diag_active   = 1u;
    /* origin_set, fatal_latched, err_msg already 0/NULL from memset */
}

/* [] END OF FILE */