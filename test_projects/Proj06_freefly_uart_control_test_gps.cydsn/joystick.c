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

#include "joystick.h"

// Persistent states
static joy_cal_t s_default;
static joy_cal_t s_saved;
static joy_cal_t s_work;

static joy_cal_state_t s_state = JOY_CAL_OFF;

static int16 s_latest_counts[N_CH];
static uint32 s_invert_mask = 0u;

// Variables to capture joystick centered from average.
static int32  s_center_sum[N_CH];
static uint16 s_center_n = 0u;

static float s_sensitivity_multiplier = 1.0f;
void joystick_set_sensitivity(joy_sensitivity_t level) {
    switch(level) {
        case SENSE_LOW:  s_sensitivity_multiplier = 0.1f; break;
        case SENSE_MED:  s_sensitivity_multiplier = 0.5f; break;
        case SENSE_HIGH: s_sensitivity_multiplier = 1.0f; break;
    }
}

// Helper - Normalizes within +/-1
static float normalize_axis(int16 c, int16 cmin, int16 ccenter, int16 cmax)
{
    if (c < cmin) c = cmin;
    if (c > cmax) c = cmax;

    int16 span_pos = (int16)(cmax - ccenter);
    int16 span_neg = (int16)(ccenter - cmin);

    if (span_pos <= 0) span_pos = 1;
    if (span_neg <= 0) span_neg = 1;

    if (c >= ccenter)
        return (float)(c - ccenter) / (float)span_pos;  // 0..+1
    else
        return (float)(c - ccenter) / (float)span_neg;  // 0..-1
}

// Helper - Deadband central region to avoid fluctuations
static float apply_deadband(float u, float db)
{
    if (u > -db && u < db)
        return 0.0f;

    if (u > 0.0f)
        return (u - db) / (1.0f - db);
    else
        return (u + db) / (1.0f - db);
}

// Helper - Validate callibration data
uint8 joystick_validate(const joy_cal_t *c)
{
    // Validate over each channel
    for (uint8 i = 0u; i < (uint8)N_CH; i++)
    {
        // Calculate channel range
        int16 span   = (int16)(c->maxv[i] - c->minv[i]);
        // Ensure measured center is within this range
        uint8 center_ok = (c->minv[i] < c->center[i]) && (
                            c->center[i] < c->maxv[i]);
        // Unhappy cases
        if (span < (int16)CAL_MIN_SPAN_COUNTS){return 0u;}
        if (!center_ok){return 0u;}
    }
    return 1u;  // Happy
}

// Helper - Prepare for range calibration
static void cal_begin_range_capture(const int16 counts[N_CH])
{
    // Copy last calibration state in case we need to revert
    s_work = s_saved;

    // BEfore collecting data, initialize the state with current readings
    for (uint8 i = 0u; i < (uint8)N_CH; i++)
    {
        s_work.minv[i]   = counts[i];
        s_work.maxv[i]   = counts[i];
        s_work.center[i] = counts[i]; /* provisional */
    }

    s_state = JOY_CAL_RANGE_CAPTURE;  // Enter calibration mode
}

// Helper - Update max and min from callibration data
static void cal_update_ranges(const int16 counts[N_CH])
{   
    // Dont run if not calibrating
    if (s_state != JOY_CAL_RANGE_CAPTURE)
        return;
    
    // Loop over captured data and populate range limits.
    for (uint8 i = 0u; i < (uint8)N_CH; i++)
    {
        if (counts[i] < s_work.minv[i]) s_work.minv[i] = counts[i];  // min
        if (counts[i] > s_work.maxv[i]) s_work.maxv[i] = counts[i];  // max
    }
}

// Helper - Prepare for centered calibration
static void cal_start_center_capture(void)
{
    // Initialize center value
    for (uint8 i = 0u; i < (uint8)N_CH; i++)
        s_center_sum[i] = 0;

    s_center_n = 0u;
    s_state = JOY_CAL_CENTER_CAPTURE;  // Move to next state
}

// Helper - Collect and average center data, save to calibration
static void cal_center_capture_update(const int16 counts[N_CH])
{
    // Dont run if not calibrating
    if (s_state != JOY_CAL_CENTER_CAPTURE)
        return;

    // Collect data (User should not be touching joystick)
    for (uint8 i = 0u; i < (uint8)N_CH; i++)
        s_center_sum[i] += counts[i];

    s_center_n++;

    // Process and average
    if (s_center_n >= (uint16)CAL_CENTER_SAMPLES)
    {
        for (uint8 i = 0u; i < (uint8)N_CH; i++)
            s_work.center[i] = (int16)(s_center_sum[i] / (int32)CAL_CENTER_SAMPLES);

        s_state = JOY_CAL_CONFIRM_SAVE;  // Move to next state
    }
}

// Helper - Give user option to save, begin calibration again, or cancel
static void cal_save_or_restart(void)
{
    if (!joystick_validate(&s_work))
    {
        // Re-run from range capturing state (Caution: recursion-ish)
        cal_begin_range_capture(s_latest_counts);
        return;
    }

    // Save!
    s_saved = s_work;
    s_saved.valid = 1u;
    s_state = JOY_CAL_OFF;
}

/* ------------------------------------------------------------------
 * Public API (This is what you include in your code (; )
 * ------------------------------------------------------------------ */

// Initialize joystick with default values
void joystick_init(const joy_cal_t *defaults, uint32 invert_mask)
{
    s_default = *defaults;
    s_saved   = *defaults;
    s_saved.valid = 0u;
    s_work    = *defaults;

    s_invert_mask = invert_mask;

    for (uint8 i = 0u; i < (uint8)N_CH; i++)
        s_latest_counts[i] = defaults->center[i];

    s_state = JOY_CAL_OFF;
    s_center_n = 0u;
}

void joystick_on_sample(const int16 counts[N_CH])
{
    // Received sample from ADC over channels
    for (uint8 i = 0u; i < (uint8)N_CH; i++)
        s_latest_counts[i] = counts[i];

    // If calibration enabled these will accept counts, otherwise will pass
    cal_update_ranges(counts);
    cal_center_capture_update(counts);
}

// Terminal UI, this is not generalized
// Handles:
//  x - disable joystick
//  c - start calibration
void joystick_on_key(char key)
{
    if (key == 'x' || key == 'X')
    {
        s_state = JOY_CAL_OFF;
        return;
    }

    if (!(key == 'c' || key == 'C'))
        return;

    // c will advance through calibration stages
    if (s_state == JOY_CAL_OFF)
    {
        cal_begin_range_capture(s_latest_counts);
    }
    else if (s_state == JOY_CAL_RANGE_CAPTURE)
    {
        cal_start_center_capture();
    }
    else if (s_state == JOY_CAL_CENTER_CAPTURE)
    {
        // Pass
    }
    else if (s_state == JOY_CAL_CONFIRM_SAVE)
    {
        cal_save_or_restart();
    }
}

// Calibration state machine state getter
joy_cal_state_t joystick_cal_state(void)
{
    return s_state;
}

// Getter function for calibration - Unused
uint8 joystick_get_cal(joy_cal_t *out)
{
    if (joystick_validate(&s_saved))
    {
        *out = s_saved;
        return 1u;
    }
    return 0u;
}

// Getter function for uncalibrated samples - Unused
void joystick_get_raw(int16 out_counts[N_CH])
{
    for (uint8 i = 0u; i < (uint8)N_CH; i++)
        out_counts[i] = s_latest_counts[i];
}

// Getter function for how many samples have been captured during calibration
uint16 joystick_center_samples_collected(void)
{
    return s_center_n;
}

// Main API call
// Takes raw ADC and converts to normalized joystick output.
void joystick_get_cmd(joy_cmd_t *out_cmd)
{
    // Grab default cal, and use user calibration only if its valid.
    const joy_cal_t *cal = &s_default;
    if (joystick_validate(&s_saved))
    {
        cal = &s_saved;
    }

    // For each channel
    for (uint8 i = 0u; i < (uint8)N_CH; i++)
    {
        // Linear normalize between min, center, and max
        float u = normalize_axis(s_latest_counts[i], cal->minv[i], cal->center[i], cal->maxv[i]);
        // Zeroes anything within 5% of center
        u = apply_deadband(u, (float)JOY_DEADBAND);

        // Flips x axis for pan
        if ((s_invert_mask >> i) & 0x1u)
            u = -u;

        // Scale by global variable so user can tune output to different limits
        u *= s_sensitivity_multiplier;
        if (u > (float)CMD_MAX) u = (float)CMD_MAX;
        if (u < -(float)CMD_MAX) u = -(float)CMD_MAX;
        out_cmd->u[i] = u;
    }
}

/* [] END OF FILE */
