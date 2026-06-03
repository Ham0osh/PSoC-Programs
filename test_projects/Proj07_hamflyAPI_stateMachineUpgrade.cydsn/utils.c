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

#include "utils.h"
#include "hamfly.h"

uint8_t gimbal_pan_tilt_deg(const app_ctx_t *ctx, float *pan_deg, float *tilt_deg)
{
    if (!ctx->gimbal) return 0u;
    hamfly_telemetry_t st;
    hamfly_get_telemetry(ctx->gimbal, &st);
    if (!st.valid) return 0u;
    float roll_deg;
    return hamfly_telemetry_to_euler(&st, pan_deg, tilt_deg, &roll_deg);
}

/* [] END OF FILE */

