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
 * Finite state machine code.
*/

#ifndef APP_STATEMACHINE_H
#define APP_STATEMACHINE_H
    
#include "app_ctx.h"
#include "hamfly.h"

// Transiton and helpers
void    transition         (app_ctx_t *ctx, app_mode_t next);
void    set_error          (app_ctx_t *ctx, const char *msg);
uint8_t handle_common_keys (app_ctx_t *ctx, char ch, uint32_t now);

// State handles
void state_standby (app_ctx_t *ctx, char ch, uint32_t now);
void state_control (app_ctx_t *ctx, char ch, uint32_t now);
void state_nudge   (app_ctx_t *ctx, char ch, uint32_t now);
void state_nudge_home(app_ctx_t *ctx, char ch, uint32_t now);
void state_error   (app_ctx_t *ctx, char ch, uint32_t now);

#endif


/* [] END OF FILE */
