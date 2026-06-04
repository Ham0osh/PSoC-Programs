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

#ifndef APP_STATE_ERROR_H
#define APP_STATE_ERROR_H

#include "app_ctx.h"
#include "hamfly_core_control.h"  // hamfly_control_t

// ERROR_ACTIVE leaf. Prints severity, kills on FATAL, waits for ack.
void    entry_error_active(app_ctx_t *ctx);
uint8_t key_error_active  (app_ctx_t *ctx, char k);
void    build_error       (const app_ctx_t *ctx, hamfly_control_t *out);

// ERROR_KILL tester state, eventualy will be superceded by the FATAL status
void    entry_error_kill(app_ctx_t *ctx);
uint8_t key_error_kill  (app_ctx_t *ctx, char k);

#endif /* APP_STATE_ERROR_H */
/* [] END OF FILE */
