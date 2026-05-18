/* ========================================
 *  ... (same banner) ...
 * Shared context initializer.
*/

#include "app_ctx.h"
#include <string.h>

void app_ctx_init(app_ctx_t *ctx)
{
    memset(ctx, 0, sizeof *ctx);
    ctx->state         = STBY_DEFER;  // boot to defer
    ctx->prev_leaf     = STBY_DEFER;  // boot to defer
    ctx->ctrl_mode     = HAMFLY_DEFER;
    ctx->sense         = SENSE_MED;
    ctx->diag_active   = 1u;
    /* origin_set, fatal_latched, err_msg already 0/NULL from memset */
}

/* [] END OF FILE */