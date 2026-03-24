/* hamfly_control.h
 * ============================================================
 * Copyright Hamish Johnson, 2026
 * Quantum Internet Systems Lab, SFU Physics
 * ============================================================
 *
 * Gimbal control types and send functions.
 * Control type enum matches FreeflyAPI ff_api_control_type_e
 * so existing QX_Protocol_App.c parser needs no changes.
 */

#ifndef HAMFLY_CONTROL_H
#define HAMFLY_CONTROL_H

#include <stdint.h>

/* Must match ff_api_control_type_e in qx_app.h */
typedef enum {
    HAMFLY_DEFER    = 0,
    HAMFLY_RATE     = 1,
    HAMFLY_ABSOLUTE = 2
} hamfly_control_mode_t;

typedef struct {
    float                 pan;
    float                 tilt;
    float                 roll;
    hamfly_control_mode_t pan_mode;
    hamfly_control_mode_t tilt_mode;
    hamfly_control_mode_t roll_mode;
    uint8_t               enable;
    uint8_t               kill;
} hamfly_control_t;

#endif /* HAMFLY_CONTROL_H */
