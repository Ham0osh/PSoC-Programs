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
 * Parent STBY - For all standby states.
*/

#ifndef APP_STATE_STBY_H
#define APP_STATE_STBY_H

#include "app_ctx.h"

// STBY leaves. Passive states.
// Entry enforcers
void entry_stby_defer(app_ctx_t *ctx);   // DEFER: gimbal released
void entry_stby_hold (app_ctx_t *ctx);   // HOLD:  rates zeroed
// On key press
uint8_t key_stby_defer(app_ctx_t *ctx, char k);
uint8_t key_stby_hold (app_ctx_t *ctx, char k);

// Flow control handler
// This ticker handles auto-advancing through leaves.
// TODO: Can this be folded into the state machine?
// Or is this already agnostic enough.
void    app_stby_tick (app_ctx_t *ctx);


#endif /* APP_STATE_STBY_H */
/* [] END OF FILE */
