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
 * Parent AUTO - For all tracking states.
*/

#ifndef APP_STATE_AUTO_H
#define APP_STATE_AUTO_H

#include "app_ctx.h"
#include "hamfly_core_control.h"  // hamfly_control_t

// Entry guards. 1 = entry allowed, 0 = refuse (engine redirects to STBY_HOLD).
uint8_t guard_auto_home   (const app_ctx_t *ctx);  // needs origin + live telemetry
uint8_t guard_auto_acq_gps(const app_ctx_t *ctx);  // needs a GPS target

// AUTO sub-state handlers (HOME, ACQ_GPS, ACQ_SPIRAL, TRACKING, LOSS, NO_LOCK)
// AUTO_HOME
void    entry_auto_home      (app_ctx_t *ctx);
void    exit_auto_home       (app_ctx_t *ctx);
uint8_t key_auto_home        (app_ctx_t *ctx, char k);
void    build_auto_home      (const app_ctx_t *ctx, hamfly_control_t *out);

// AUTO_ACQ_GPS
void    entry_auto_acq_gps   (app_ctx_t *ctx);
void    exit_auto_acq_gps    (app_ctx_t *ctx);
uint8_t key_auto_acq_gps     (app_ctx_t *ctx, char k);
void    build_auto_acq_gps   (const app_ctx_t *ctx, hamfly_control_t *out);

// AUTO_ACQ_SPIRAL
void    entry_auto_acq_spiral(app_ctx_t *ctx);

// AUTO_TRACKING
void    entry_auto_tracking  (app_ctx_t *ctx);
uint8_t key_auto_tracking    (app_ctx_t *ctx, char k);
void    build_auto_tracking  (const app_ctx_t *ctx, hamfly_control_t *out);

// AUTO_LOSS
void    entry_auto_loss      (app_ctx_t *ctx);
uint8_t key_auto_loss        (app_ctx_t *ctx, char k);

// AUTO_NO_LOCK
void    entry_auto_no_lock   (app_ctx_t *ctx);
uint8_t key_auto_no_lock     (app_ctx_t *ctx, char k);

// Per-loop worker.
// Used for checking settling, GPS repoint if new target, and pulling centroid
// from packets from the SBC.
void    app_auto_tick        (app_ctx_t *ctx);

#endif /* APP_STATE_AUTO_H */
/* [] END OF FILE */
