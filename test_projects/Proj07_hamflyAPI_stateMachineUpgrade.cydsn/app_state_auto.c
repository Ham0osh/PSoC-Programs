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

#include "app_state_auto.h"
#include "app_statemachine.h"  // app_transitions and app_raise_error
#include "hamfly.h"            // telemetry helpers and gimble mode enums
#include "sbc_comms.h"         // centroid stream getters + counters
#include "utils.h"             // Euler angles calc
#include <project.h>           // UART_DEBUG_PutString
#include <math.h>              // sinf cosf atan2f sqrtf fabsf
#include <stdio.h>             // snprintf

extern volatile uint32_t g_tick_ms;

// GPS Pointing Math %========================================================%
// Forward declare here, defined below.
static float gps_bearing_deg (float lat1, float lon1, float lat2, float lon2);
static float gps_elev_deg    (float lat1, float lon1, float alt1_m,
                              float lat2, float lon2, float alt2_m);
static void  gps_start_slew     (app_ctx_t *ctx);
static void  gps_update_pointing(app_ctx_t *ctx);
static void  gps_check_settle   (app_ctx_t *ctx);

// State On-Entry Guards %====================================================%
uint8_t guard_auto_home(const app_ctx_t *ctx)
{
    // A home origin must be set, and attitude returns properly.
    float p, t;
    return ctx->origin_set && gimbal_pan_tilt_deg(ctx, &p, &t);
}

uint8_t guard_auto_acq_gps(const app_ctx_t *ctx)
{
    // A target GPS coordinate must be set.
    return ctx->gps_target_set;
}

// %==========================================================================%
// %                          AUTO Tick Handling                              %
// %==========================================================================%
// Handles the multi-tick actions of sub-modes such as:
//  - HOME: Reached within tolerance and steady.
//  - ACQ_GPS: Recheck for new coordinate at 1Hz, settle.
//  - TRACKING: Pull freshest centroid, print.

void app_auto_tick(app_ctx_t *ctx)
{
    // AUTO_HOME
    if (ctx->state == AUTO_HOME && ctx->nudge_hold) {
        uint32_t elapsed = g_tick_ms - ctx->nudge_start_ms;
        if (elapsed >= NUDGE_MIN_DWELL_MS) {
            float p, t; uint8_t arrived = 0u;
            if (gimbal_pan_tilt_deg(ctx, &p, &t))
                arrived = (fabsf(p - ctx->tgt_pan_deg)  < NUDGE_SETTLE_DEG) &&
                          (fabsf(t - ctx->tgt_tilt_deg) < NUDGE_SETTLE_DEG);
            if (arrived || elapsed >= NUDGE_TIMEOUT_MS) {
                UART_DEBUG_PutString("[AUTO_HOME] arrived -> STBY_HOLD\r\n");
                app_transition(ctx, STBY_HOLD);  // -> STBY_HOLD
            }
        }
        return;
    }
    
    // AUTO_ACQ_GPS
    // TODO: GPS should go to the set coordinate and settle and stay there,
    // waiting for centroid packets to arrive for N seconds (0 means wait forever).
    // If no lock in N seconds, spiral, if lock go to tracking.
    if (ctx->state == AUTO_ACQ_GPS) {
        static uint32_t last_pt_ms = 0u;
        if (g_tick_ms - last_pt_ms >= DIAG_PERIOD_MS) {
            last_pt_ms = g_tick_ms;
            if (ctx->gps_target_set) gps_update_pointing(ctx);
        }
        gps_check_settle(ctx);   // self-throttled to GPS_SETTLE_PERIOD_MS
        return;
    }

    // AUTO_TRACKING
    if (ctx->state == AUTO_TRACKING) {
        static const char *tag[STREAM_COUNTS] = { "C", "F" };
        for (uint8_t s = 0u; s < STREAM_COUNTS; s++) {
            payload_centroid_t c;
            if (!sbc_get_centroid(s, &c)) continue;
            if (s == STREAM_COARSE) {  // Save coarse tracking centroids
                ctx->track_cx_last = c.cx;
                ctx->track_cy_last = c.cy;
            }
            char b[128];
            snprintf(b, sizeof b,
                     "[SBC:%s] cx=%d cy=%d dt=%u rx=%lu crcErr=%u uartErr=%u\r\n",
                     tag[s], c.cx, c.cy,
                     sbc_last_centroid_dt_ms(s),
                     (unsigned long)sbc_rx_pkt_count(),
                     sbc_crc_errors(), sbc_uart_errors());
            UART_DEBUG_PutString(b);
        }
    }
}


// %==========================================================================%
// %                              AUTO_HOME                                   %
// %==========================================================================%
void entry_auto_home(app_ctx_t *ctx)
{
    UART_DEBUG_PutString("\r\n[AUTO_HOME] slewing to origin  e=exit\r\n> ");
    // TODO: Cross verify if this is handled by entry validator.
    // Error check: Do we have a software origin set?
    if (!ctx->origin_set) {
        app_raise_error(ctx, SEV_USER, "no origin set (press '[' first)");
        app_transition(ctx, STBY_HOLD);
        return;
    }
    // Error check: Do we have attitude telemetry from gimbal?
    float pan, tilt;
    if (!gimbal_pan_tilt_deg(ctx, &pan, &tilt)) {
        app_raise_error(ctx, SEV_USER, "no telemetry (home aborted)");
        app_transition(ctx, STBY_HOLD);
        return;
    }
    ctx->nudge_base_pan_deg  = pan;  // Init nudge reference
    ctx->nudge_base_tilt_deg = tilt;
    ctx->tgt_pan_deg         = ctx->origin_pan_deg;  // TODO: Do we actualy use these??
    ctx->tgt_tilt_deg        = ctx->origin_tilt_deg;
    ctx->nudge_hold          = 1u;  // Why are we triggering a nudge?
    ctx->nudge_start_ms      = g_tick_ms;
}

void exit_auto_home(app_ctx_t *ctx)
{
    // Dissable nudge TODO: Why was it on to begin with??
    ctx->nudge_hold = 0u;
}

uint8_t key_auto_home(app_ctx_t *ctx, char k)
{
    // Exit home with e
    switch (k) {
        case 'e': app_transition(ctx, STBY_HOLD); return 1;
        default:  return 0;
    }
}

void build_auto_home(const app_ctx_t *ctx, hamfly_control_t *out)
{
    // Use absolute mode to point to saftware origin.
    // Software origin defaults to gimbal origin.
    out->kill      = 0u;
    out->roll_mode = HAMFLY_DEFER;
    out->roll      = 0.0f;
    out->pan_mode  = out->tilt_mode = HAMFLY_ABSOLUTE;
    out->pan  = DEG_TO_UNIT(ctx->tgt_pan_deg - ctx->nudge_base_pan_deg);
    out->tilt = DEG_TO_UNIT(ctx->tgt_tilt_deg);
}

// %==========================================================================%
// %                        AUTO_ACQ_GPS + GPS math                           %
// %==========================================================================%
// Note: Keep in mind PSoC 5 LP has no FPU! This means we should avoid
// floating-point arithmatic as much as possible. But this is not a speed 
// concern yet so we will keep simulated floating point.

// TODO: The GPS math stub doesnt need to live here. HamflyAPI should introduce
// a point at coordinate like function which takes in a target coordinate, and
// uses its gps reading (if available) to point at another coordinate in user
// selectable abs or majestic mode. THis would offload the bulk here to the API
// at the expense of being blind to FP or intiger math.

// TODO: Currently we use bearing/elevation geometry, but we should move to
// ECEF Vector math, however we first need to validate IMU data from the gimble
// or get an external IMU.
static float gps_bearing_deg(float lat1, float lon1, float lat2, float lon2)
{
    // Calculates bearing between two points in degrees.
    float la1 = lat1 * DEG_TO_RAD, la2 = lat2 * DEG_TO_RAD;
    float dlo = (lon2 - lon1) * DEG_TO_RAD;
    float x   = sinf(dlo) * cosf(la2);
    float y   = cosf(la1) * sinf(la2) - sinf(la1) * cosf(la2) * cosf(dlo);
    float b   = atan2f(x, y) * RAD_TO_DEG;
    if (b < 0.0f) b += 360.0f;
    return b;  // 0 = North, clockwise
}
static float gps_elev_deg(float lat1, float lon1, float alt1_m,
                          float lat2, float lon2, float alt2_m)
{
    // COmputes elevation angle between two points in degrees.
    float la1 = lat1 * DEG_TO_RAD, la2 = lat2 * DEG_TO_RAD;
    float dlo = (lon2 - lon1) * DEG_TO_RAD;
    float dla = la2 - la1;
    float a   = sinf(dla * 0.5f) * sinf(dla * 0.5f)
              + cosf(la1) * cosf(la2) * sinf(dlo * 0.5f) * sinf(dlo * 0.5f);
    float dist_m = 2.0f * GPS_EARTH_R_M * atan2f(sqrtf(a), sqrtf(1.0f - a));
    if (dist_m < 1.0f) dist_m = 1.0f;  // guard div/0 at zero range
    return atan2f(alt2_m - alt1_m, dist_m) * RAD_TO_DEG;
}

// Copy the active target into the working copy and reset the slew.
static void gps_start_slew(app_ctx_t *ctx)
{
    ctx->gps_work_lat_raw = ctx->gps_target_lat_raw;
    ctx->gps_work_lon_raw = ctx->gps_target_lon_raw;
    ctx->gps_work_alt_mm  = ctx->gps_target_alt_mm;
    // Clear flags to say: We are not settled and we have a new target.
    ctx->gps_settled      = 0u;
    ctx->gps_new_target   = 0u;
}

// Recompute abs_pan/tilt setpoints from the working copy + live gimbal GPS.
// Overkill for now, but handles the gimble moving, this updates the target in
// case previous computed absolute move is out of date. Important to have now
// in case GPS quality degrades or improves during slew.
static void gps_update_pointing(app_ctx_t *ctx)
{
    // TODO: Only give update if moved by 1 degree or more
    // Get attitude and position
    hamfly_telemetry_t st;
    hamfly_get_telemetry(ctx->gimbal, &st);
    if (!st.gps_valid) return;
    // Load working target
    float tlat = (float)ctx->gps_work_lat_raw * 1e-7f;
    float tlon = (float)ctx->gps_work_lon_raw * 1e-7f;
    float talt = (float)ctx->gps_work_alt_mm  * 0.001f;
    // Update target
    ctx->abs_pan_target  = DEG_TO_UNIT(
        gps_bearing_deg(  st.gps_lat_deg, st.gps_lon_deg, tlat, tlon)
                        - st.gps_heading_deg);
    ctx->abs_tilt_target = DEG_TO_UNIT(
        gps_elev_deg(st.gps_lat_deg, st.gps_lon_deg, st.gps_alt_m, 
                     tlat, tlon, talt));
}

// Take finite difference of attitude to tell when arrived at destination.
// Checks for new target GPS coordinate while in progress, which triggers a 
// restart of AUTO_ACQ_GPS.
static void gps_check_settle(app_ctx_t *ctx)
{
    // Too fast? Dont judge.
    uint32_t now = g_tick_ms;
    if (now - ctx->gps_last_sample_ms < GPS_SETTLE_PERIOD_MS) return;

    // no telemetry -> can't judge
    float pan, tilt;
    if (!gimbal_pan_tilt_deg(ctx, &pan, &tilt)) return;

    float speed_dps = 1.0e9f;   // assume moving until we have a valid dt
    if (ctx->gps_last_sample_ms != 0u) {
        float dt = (float)(now - ctx->gps_last_sample_ms) * 0.001f;
        if (dt > 0.0f)
            speed_dps = (fabsf(pan  - ctx->gps_last_pan_deg) +
                         fabsf(tilt - ctx->gps_last_tilt_deg)) / dt;
    }
    ctx->gps_last_pan_deg   = pan;
    ctx->gps_last_tilt_deg  = tilt;
    ctx->gps_last_sample_ms = now;

    /* Frame assumption: measured euler pan/tilt share the gimbal's ABSOLUTE
       command reference. Validate on hardware; if frames differ, the low-speed
       gate still gives a usable settle proxy. */
    float cmd_pan_deg  = UNIT_TO_DEG(ctx->abs_pan_target);
    float cmd_tilt_deg = UNIT_TO_DEG(ctx->abs_tilt_target);
    uint8_t arrived = (fabsf(pan  - cmd_pan_deg)  < GPS_POINT_SETTLE_DEG) &&
                      (fabsf(tilt - cmd_tilt_deg) < GPS_POINT_SETTLE_DEG);
    uint8_t slow    = (speed_dps < GPS_RATE_SETTLE_DPS);
    
    // Catch new target flag
    if (ctx->gps_new_target && (ctx->gps_settled || (arrived && slow))) {
        UART_DEBUG_PutString("[AUTO_ACQ_GPS] new target -> re-slew\r\n");
        gps_start_slew(ctx);  // copies target->work, clears flag + settled
        gps_update_pointing(ctx);
        return;
    }
    // Exit condition for valid finishing of GPS slew.
    if (!ctx->gps_settled && arrived && slow) {
        ctx->gps_settled = 1u;
        UART_DEBUG_PutString("[AUTO_ACQ_GPS] settled\r\n");
    }
}

void entry_auto_acq_gps(app_ctx_t *ctx)
{
    // Check for GPS target. TODO: Is this hanlded seperately now?
    if (!ctx->gps_target_set) {
        UART_DEBUG_PutString("\r\n[AUTO_ACQ_GPS] no GPS target -> STBY_HOLD\r\n");
        app_transition(ctx, STBY_HOLD);
        return;
    }
    UART_DEBUG_PutString("\r\n[AUTO_ACQ_GPS] open-loop GPS pointing  e=exit\r\n> ");
    // Set start flag.
    ctx->gps_last_sample_ms = 0u;
    // Trigger flags that we are starting slew.
    gps_start_slew(ctx);        // snap target -> work, start fresh
    // Get and use bearing for pointing math.
    gps_update_pointing(ctx);   // initial bearing
}

void exit_auto_acq_gps(app_ctx_t *ctx)
{
    // Reset target variables
    ctx->abs_pan_target = ctx->abs_tilt_target = 0.0f;
}

uint8_t key_auto_acq_gps(app_ctx_t *ctx, char k)
{
    switch (k) {
        case 'e': app_transition(ctx, STBY_HOLD); return 1;
        default:  return 0;
    }
}

void build_auto_acq_gps(const app_ctx_t *ctx, hamfly_control_t *out)
{
    out->kill      = 0u;
    out->roll_mode = HAMFLY_DEFER;
    out->roll      = 0.0f;
    out->pan_mode  = out->tilt_mode = HAMFLY_ABSOLUTE;  // TODO: Majestic? Needs testing of pointing stability.
    out->pan  = CLAMP(ctx->abs_pan_target,  -1.0f, 1.0f);
    out->tilt = CLAMP(ctx->abs_tilt_target, -1.0f, 1.0f);
}


// %==========================================================================%
// %                           AUTO_ACQ_SPIRAL                                %
// %==========================================================================%
void entry_auto_acq_spiral(app_ctx_t *ctx)
{
    // Not implemented.
    (void)ctx;
    UART_DEBUG_PutString("\r\n[AUTO_ACQ_SPIRAL] placeholder (NI).\r\n> ");
}

// %==========================================================================%
// %                            AUTO_TRACKING                                 %
// %==========================================================================%
void entry_auto_tracking(app_ctx_t *ctx)
{
    // Clear any old values for clean handover.
    ctx->track_cx_last = 0;  // Clear last centroid value
    ctx->track_cy_last = 0;
    UART_DEBUG_PutString("\r\n[AUTO_TRACKING] closed-loop  e=exit\r\n> ");
}

uint8_t key_auto_tracking(app_ctx_t *ctx, char k)
{
    // TEMP: Debug exit to hold with e over serial.
    switch (k) {
        case 'e': app_transition(ctx, STBY_HOLD); return 1;
        default:  return 0;
    }
}

void build_auto_tracking(const app_ctx_t *ctx, hamfly_control_t *out)
{
    // Construct control packet from centroid error signal.
    out->kill      = 0u;
    out->roll_mode = HAMFLY_DEFER;  // No roll
    out->roll      = 0.0f;
    out->pan_mode  = out->tilt_mode = HAMFLY_RATE;  // Rate mode
    float cx_mrad = (float)ctx->track_cx_last * 0.1f;  // 0.1 mrad to mrad
    float cy_mrad = (float)ctx->track_cy_last * 0.1f;
    // Simple proportional gain linear feedback.
    out->pan  =  CLAMP(ctx->track_kp * cx_mrad, -1.0f, 1.0f);  // TODO: Confirm direction
    out->tilt = -CLAMP(ctx->track_kp * cy_mrad, -1.0f, 1.0f);  // +cy = up
}

// %==========================================================================%
// %                              AUTO_LOSS                                   %
// %==========================================================================%
void entry_auto_loss(app_ctx_t *ctx)
{
    // TODO: Loss should hold and wait. If packets re-commence go back to tracking,
    // TODO: Let user trigger spiral search, GPS re-acq, or exit to STBY later.
    (void)ctx;
    UART_DEBUG_PutString("\r\n[AUTO_LOSS] centroid lost  1=hold  3=retry\r\n> ");
}

uint8_t key_auto_loss(app_ctx_t *ctx, char k)
{
    switch (k) {  // Keep stationary, spiral search if requested.
        case '3': app_transition(ctx, AUTO_ACQ_SPIRAL); return 1;
        default:  return 0;
    }
}

// %==========================================================================%
// %                             AUTO_NO_LOCK                                 %
// %==========================================================================%
void entry_auto_no_lock(app_ctx_t *ctx)
{
    (void)ctx;
    UART_DEBUG_PutString("\r\n[AUTO_NO_LOCK] acquisition failed  1=hold  3=retry\r\n> ");
}

uint8_t key_auto_no_lock(app_ctx_t *ctx, char k)
{
    switch (k) {
        case '3': app_transition(ctx, AUTO_ACQ_SPIRAL); return 1;
        default:  return 0;
    }
}

/* [] END OF FILE */
