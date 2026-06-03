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
 * Parent ERROR - Save landing for errors.
*/

#include "app_state_error.h"
#include "app_statemachine.h"   // app_transition, build_defer
#include "hamfly.h"             // hamfly_kill
#include <project.h>            // For UART

// Debug print helper
static const char *sev_str(err_sev_t s)
{
    switch (s) {
        case SEV_WARN:     return "WARN";
        case SEV_SOFTWARE: return "SOFTWARE";
        case SEV_FATAL:    return "FATAL";
        default:           return "USER";
    }
}

// Print error and severity.
void entry_error_active(app_ctx_t *ctx)
{
    UART_DEBUG_PutString("\r\n[ERROR:");
    UART_DEBUG_PutString(sev_str(ctx->err_sev));
    UART_DEBUG_PutString("] ");
    UART_DEBUG_PutString(ctx->err_msg ? ctx->err_msg : "(no msg)");
    if (ctx->err_sev == SEV_FATAL && ctx->gimbal)
    {
        hamfly_kill(ctx->gimbal);
        UART_DEBUG_PutString("\r\nFATAL latched. Power cycle or Ctrl+R to clear.\r\n");
    }
    else    UART_DEBUG_PutString("\r\nAny key to acknowledge.\r\n");
}

// Fatal latch or allow return to previous state.
uint8_t key_error_active(app_ctx_t *ctx, char k)
{
    (void)k;
    if (ctx->err_sev != SEV_FATAL) {            /* WARN/SOFTWARE ack */
        state_t back = (ctx->prev_leaf == ERROR_ACTIVE) ? STBY_HOLD : ctx->prev_leaf;
        app_transition(ctx, back);
        return 1;
    }
    return 0;                                   /* FATAL: only Ctrl+R (handled in lockout) */
}

// Build packet for when in ERROR state.
// Defer on error, kill if fatal.
void build_error(const app_ctx_t *ctx, hamfly_control_t *out) {
    build_defer(ctx, out);  // Error
    if (ctx->err_sev == SEV_FATAL) out->kill = 1u;
}

/* [] END OF FILE */
