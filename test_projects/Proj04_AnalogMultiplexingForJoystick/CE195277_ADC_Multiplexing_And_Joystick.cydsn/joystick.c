/* joystick.c */

#include "joystick.h"

/* ---- Internal persistent state ---- */
static joy_cal_t s_default;
static joy_cal_t s_saved;
static joy_cal_t s_work;

static joy_cal_state_t s_state = JOY_CAL_OFF;

static int16 s_latest_counts[N_CH];
static uint32 s_invert_mask = 0u;

/* Center capture accumulators */
static int32  s_center_sum[N_CH];
static uint16 s_center_n = 0u;

/* ---- Math helpers ---- */
static float normalize_axis(int16 c, int16 cmin, int16 ccenter, int16 cmax)
{
    if (c < cmin) c = cmin;
    if (c > cmax) c = cmax;

    int16 span_pos = (int16)(cmax - ccenter);
    int16 span_neg = (int16)(ccenter - cmin);

    if (span_pos <= 0) span_pos = 1;
    if (span_neg <= 0) span_neg = 1;

    if (c >= ccenter)
        return (float)(c - ccenter) / (float)span_pos;  /* 0..+1 */
    else
        return (float)(c - ccenter) / (float)span_neg;  /* 0..-1 */
}

static float apply_deadband(float u, float db)
{
    if (u > -db && u < db)
        return 0.0f;

    if (u > 0.0f)
        return (u - db) / (1.0f - db);
    else
        return (u + db) / (1.0f - db);
}

/* ---- Calibration helpers ---- */
uint8 joystick_validate(const joy_cal_t *c)
{
    for (uint8 i = 0u; i < (uint8)N_CH; i++)
    {
        int16 span = (int16)(c->maxv[i] - c->minv[i]);
        if (span < (int16)CAL_MIN_SPAN_COUNTS) return 0u;

        if (!(c->minv[i] < c->center[i] && c->center[i] < c->maxv[i])) return 0u;
    }
    return 1u;
}

static void cal_begin_range_capture(const int16 counts[N_CH])
{
    /* start from saved so cancel is easy; but overwrite spans from current */
    s_work = s_saved;

    for (uint8 i = 0u; i < (uint8)N_CH; i++)
    {
        s_work.minv[i]   = counts[i];
        s_work.maxv[i]   = counts[i];
        s_work.center[i] = counts[i]; /* provisional */
    }

    s_state = JOY_CAL_RANGE_CAPTURE;
}

static void cal_update_ranges(const int16 counts[N_CH])
{
    if (s_state != JOY_CAL_RANGE_CAPTURE)
        return;

    for (uint8 i = 0u; i < (uint8)N_CH; i++)
    {
        if (counts[i] < s_work.minv[i]) s_work.minv[i] = counts[i];
        if (counts[i] > s_work.maxv[i]) s_work.maxv[i] = counts[i];
    }
}

static void cal_start_center_capture(void)
{
    for (uint8 i = 0u; i < (uint8)N_CH; i++)
        s_center_sum[i] = 0;

    s_center_n = 0u;
    s_state = JOY_CAL_CENTER_CAPTURE;
}

static void cal_center_capture_update(const int16 counts[N_CH])
{
    if (s_state != JOY_CAL_CENTER_CAPTURE)
        return;

    for (uint8 i = 0u; i < (uint8)N_CH; i++)
        s_center_sum[i] += counts[i];

    s_center_n++;

    if (s_center_n >= (uint16)CAL_CENTER_SAMPLES)
    {
        for (uint8 i = 0u; i < (uint8)N_CH; i++)
            s_work.center[i] = (int16)(s_center_sum[i] / (int32)CAL_CENTER_SAMPLES);

        s_state = JOY_CAL_CONFIRM_SAVE;
    }
}

static void cal_save_or_restart(void)
{
    if (!joystick_validate(&s_work))
    {
        /* restart range capture; keep s_work spans as-is? safest: restart clean */
        cal_begin_range_capture(s_latest_counts);
        return;
    }

    s_saved = s_work;
    s_saved.valid = 1u;
    s_state = JOY_CAL_OFF;
}

/* ---- Public API ---- */
void joystick_init(const joy_cal_t *defaults, uint32 invert_mask)
{
    s_default = *defaults;
    s_saved   = *defaults;
    s_saved.valid = 0u;        /* defaults are a fallback, not "user-calibrated" */
    s_work    = *defaults;

    s_invert_mask = invert_mask;

    for (uint8 i = 0u; i < (uint8)N_CH; i++)
        s_latest_counts[i] = defaults->center[i];

    s_state = JOY_CAL_OFF;
    s_center_n = 0u;
}

void joystick_on_sample(const int16 counts[N_CH])
{
    for (uint8 i = 0u; i < (uint8)N_CH; i++)
        s_latest_counts[i] = counts[i];

    /* Calibration state machine consumes samples */
    cal_update_ranges(counts);
    cal_center_capture_update(counts);
}

void joystick_on_key(char key)
{
    if (key == 'x' || key == 'X')
    {
        s_state = JOY_CAL_OFF;
        return;
    }

    if (!(key == 'c' || key == 'C'))
        return;

    /* 'c' advances through calibration */
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
        /* ignore: auto-completes after N samples */
    }
    else if (s_state == JOY_CAL_CONFIRM_SAVE)
    {
        cal_save_or_restart();
    }
}

joy_cal_state_t joystick_cal_state(void)
{
    return s_state;
}

uint8 joystick_get_cal(joy_cal_t *out)
{
    if (joystick_validate(&s_saved))
    {
        *out = s_saved;
        return 1u;
    }
    return 0u;
}

void joystick_get_raw(int16 out_counts[N_CH])
{
    for (uint8 i = 0u; i < (uint8)N_CH; i++)
        out_counts[i] = s_latest_counts[i];
}

uint16 joystick_center_samples_collected(void)
{
    return s_center_n;
}

void joystick_get_cmd(joy_cmd_t *out_cmd)
{
    const joy_cal_t *cal = &s_default;

    /* Promote saved calibration only if sane */
    if (joystick_validate(&s_saved))
    {
        cal = &s_saved;
    }


    for (uint8 i = 0u; i < (uint8)N_CH; i++)
    {
        float u = normalize_axis(s_latest_counts[i], cal->minv[i], cal->center[i], cal->maxv[i]);
        u = apply_deadband(u, (float)JOY_DEADBAND);

        if ((s_invert_mask >> i) & 0x1u)
            u = -u;

        u *= (float)CMD_MAX;
        out_cmd->u[i] = u;
    }
}


/* [] END OF FILE */
