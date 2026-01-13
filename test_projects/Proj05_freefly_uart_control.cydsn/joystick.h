/* joystick.h
 *
 * Joystick calibration + mapping for N_CH channels.
 * Current intended use: N_CH=2 (X,Y) from adc_balanced.
 *
 * Calibration workflow (single combined):
 *   'c' : begin range capture (min/max for all channels)
 *   move stick over full range, release to center
 *   'c' : begin center capture (averaged over CAL_CENTER_SAMPLES fresh frames)
 *   'c' : validate + save
 *   'x' : cancel at any time
 *
 * Mapping:
 *   - normalize each channel to [-1..+1] using per-channel min/center/max
 *   - apply deadband with rescale
 *   - optional per-channel inversion
 *   - scale by CMD_MAX
 *
 * Notes:
 *   - This module is logic-only (no UART printing). main.c handles CLI + prompts.
 */

#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <project.h>

/* Must match adc_balanced N_CH */
#ifndef N_CH
#define N_CH (2u)
#endif

#if (N_CH < 1u)
#error "N_CH must be >= 1"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Tuning */
#ifndef CAL_CENTER_SAMPLES
#define CAL_CENTER_SAMPLES (32u)
#endif

#ifndef CAL_MIN_SPAN_COUNTS
#define CAL_MIN_SPAN_COUNTS (20)
#endif

#ifndef JOY_DEADBAND
#define JOY_DEADBAND (0.05f)
#endif

#ifndef CMD_MAX
#define CMD_MAX (1.0f)
#endif

typedef struct {
    int16 minv[N_CH];
    int16 maxv[N_CH];
    int16 center[N_CH];
    uint8 valid;
} joy_cal_t;

typedef enum {
    JOY_CAL_OFF = 0,
    JOY_CAL_RANGE_CAPTURE,
    JOY_CAL_CENTER_CAPTURE,
    JOY_CAL_CONFIRM_SAVE
} joy_cal_state_t;

typedef struct {
    float u[N_CH];  /* mapped outputs in [-CMD_MAX..+CMD_MAX] per channel */
} joy_cmd_t;

/* Initialization: provide defaults (best-guess) calibration and inversion mask.
 * invert_mask bit i: 1 => invert channel i after normalization.
 */
void joystick_init(const joy_cal_t *defaults, uint32 invert_mask);

/* Feed fresh raw samples (one complete frame of N_CH counts).
 * Call this once per adc_balanced frame.
 */
void joystick_on_sample(const int16 counts[N_CH]);

/* Handle CLI key (main keeps CLI; this just updates calibration state).
 * key: 'c'/'C' advance, 'x'/'X' cancel, others ignored.
 */
void joystick_on_key(char key);

/* Current calibration state */
joy_cal_state_t joystick_cal_state(void);

/* Returns 1 if saved calibration is sane/usable (also copies it to out). */
uint8 joystick_get_cal(joy_cal_t *out);

/* Returns 1 if c is sane/usable (shared validator). */
uint8 joystick_validate(const joy_cal_t *c);

/* Map latest counts to commands using best available calibration
 * (saved if valid else defaults), applying deadband/inversion/scaling.
 */
void joystick_get_cmd(joy_cmd_t *out_cmd);

/* Access latest raw counts (most recent frame). */
void joystick_get_raw(int16 out_counts[N_CH]);

/* Utility for main.c prompts: after center capture completes, module transitions
 * automatically to JOY_CAL_CONFIRM_SAVE; main can watch state changes.
 */
uint16 joystick_center_samples_collected(void);

#ifdef __cplusplus
}
#endif

#endif /* JOYSTICK_H */

/* [] END OF FILE */
