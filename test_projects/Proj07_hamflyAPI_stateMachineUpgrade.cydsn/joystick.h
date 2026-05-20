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
 * This code helps callibrate ADC inpit for
 * use as a joystick.
*/

#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <project.h>

// Define number of channels
#ifndef N_CH
#define N_CH (2u)
#endif

#if (N_CH < 1u)
#error "N_CH must be >= 1" // Cant have zero channels ofc
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Software tuning
#ifndef CAL_CENTER_SAMPLES
#define CAL_CENTER_SAMPLES (32u)  // Number of samples averaged to get center
#endif

#ifndef CAL_MIN_SPAN_COUNTS
#define CAL_MIN_SPAN_COUNTS (20)  // Minimum values with a range
#endif

#ifndef JOY_DEADBAND
#define JOY_DEADBAND (0.05f)  // Deadband percentage
#endif

#ifndef CMD_MAX
#define CMD_MAX (1.0f)  // Scaling of output range (+/-1 * CMD_MAX)
#endif

// Sensitivity state machine
typedef enum {
    SENSE_LOW = 0,
    SENSE_MED,
    SENSE_HIGH
} joy_sensitivity_t;
void joystick_set_sensitivity(joy_sensitivity_t level);

// Calibration data
typedef struct {
    int16 minv[N_CH];
    int16 maxv[N_CH];
    int16 center[N_CH];
    uint8 valid;
} joy_cal_t;

// State machine enum
typedef enum {
    JOY_CAL_OFF = 0,
    JOY_CAL_RANGE_CAPTURE,
    JOY_CAL_CENTER_CAPTURE,
    JOY_CAL_CONFIRM_SAVE
} joy_cal_state_t;

// Sctruct for joystick output
typedef struct {
    float u[N_CH];  // per channel
} joy_cmd_t;

// Initialize with defaults
void joystick_init(const joy_cal_t *defaults, uint32 invert_mask);

// Get frame of values
void joystick_on_sample(const int16 counts[N_CH]);

// Handles the CLI from main.c
void joystick_on_key(char key);

// Calibration state in use
joy_cal_state_t joystick_cal_state(void);

// Calibration state getter that also validates it
uint8 joystick_get_cal(joy_cal_t *out);

// Calibration state validator
uint8 joystick_validate(const joy_cal_t *c);

// Map ADC values to valid joystick range, saved to joy_cmd_t
void joystick_get_cmd(joy_cmd_t *out_cmd);

// Getter for raw data - unused
void joystick_get_raw(int16 out_counts[N_CH]);

uint16 joystick_center_samples_collected(void);

/* EEPROM persistence -- requires psoc_eeprom.h to be available.
 * Save/load joystick calibration to/from EEPROM slot JOY_CAL.
 * joystick.c includes psoc_eeprom.h directly for these functions. */
#include "psoc_eeprom.h"
uint8 joystick_save(void);
uint8 joystick_load(uint32 invert_mask);

#endif

/* [] END OF FILE */
