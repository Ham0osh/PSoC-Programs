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
 * HFSM public contract. main.c only sees these five calls plus the
 * types from app_ctx.h. Everything else (parent_of[], LCA walk,
 * per-leaf entry/exit/key/build tables) is private to the .c file.
*/

#ifndef APP_STATEMACHINE_H
#define APP_STATEMACHINE_H

#include "app_ctx.h"
#include "hamfly_core_control.h"  // hamfly_control_t struct

#define MAX_HFSM_DEPTH 4u  // For LCA walk, we have 2 levels + ROOT.

// State machine functions
void        app_start(app_ctx_t *ctx);
uint8_t     app_dispatch_key(app_ctx_t *ctx, char key);
void        app_build_control(const app_ctx_t *ctx, hamfly_control_t *out);
void        app_raise_error(app_ctx_t *ctx, err_sev_t sev, const char *msg);
const char *app_state_name(state_t s);

// State based on tick.
// TEMP: Ticker for manual nudge.
void app_manual_tick(app_ctx_t *ctx); // For nudge smoothing
void app_auto_tick(app_ctx_t *ctx);   // Temp for debug prints
void app_telem_tick(app_ctx_t *ctx);  // Handles Hot and Cold telemetry


#endif /* APP_STATEMACHINE_H */
/* [] END OF FILE */