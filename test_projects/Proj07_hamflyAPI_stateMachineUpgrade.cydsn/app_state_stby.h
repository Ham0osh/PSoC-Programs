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
void entry_stby_defer(app_ctx_t *ctx);   // DEFER: gimbal released
void entry_stby_hold (app_ctx_t *ctx);   // HOLD:  rates zeroed

#endif /* APP_STATE_STBY_H */
/* [] END OF FILE */
