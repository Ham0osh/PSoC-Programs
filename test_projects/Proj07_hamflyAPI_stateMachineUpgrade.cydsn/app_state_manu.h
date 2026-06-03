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
 * Parent MANU - For all manual states.
*/

#ifndef APP_STATE_MANU_H
#define APP_STATE_MANU_H

#include "app_ctx.h"
#include "hamfly_core_control.h"  // hamfly_control_t

// MANU_JOYSTICK leaf. Rate joystick with folded-in keyboard nudge.
void    entry_manu_joystick(app_ctx_t *ctx);
void    exit_manu_joystick (app_ctx_t *ctx);
uint8_t key_manu_joystick  (app_ctx_t *ctx, char k);
void    build_manu_joystick(const app_ctx_t *ctx, hamfly_control_t *out);

// Per-loop nudge dwell/settle for the joystick leaf.
// Handles cracefull absolute pointing of small increments.
void    app_manual_tick    (app_ctx_t *ctx);

#endif /* APP_STATE_MANU_H */
/* [] END OF FILE */
