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
 * Shared utility functions.
*/

#ifndef UTILS_H
#define UTILS_H

#include "app_ctx.h"

// Reads current gimbal pan/tilt as degrees from latest telemetry.
// Returns 1 on success, 0 if no gimbal or stale telemetry.
uint8_t gimbal_pan_tilt_deg(const app_ctx_t *ctx, float *pan_deg, float *tilt_deg);

// Applies a nudge in absolute degrees.
void nudge_apply(app_ctx_t *ctx, float dpan_deg, float dtilt_deg);
uint8_t handle_nudge_key(app_ctx_t *ctx, char k);

#endif /* UTILS_H */
/* [] END OF FILE */