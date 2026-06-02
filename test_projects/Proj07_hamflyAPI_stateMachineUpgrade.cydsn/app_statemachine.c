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
 * HFSM engine: parent table, LCA walk, transition driver, error raiser.
 *
 *   ROOT
 *   ├─ STBY ── STBY_DEFER (boot)
 *   ├       ── STBY_HOLD
 *   ├─ MANU ── MANU_JOYSTICK
 *   ├─ AUTO ── AUTO_HOME*
 *   ├       ── ACQ_GPS*
 *   ├       ── ACQ_SPIRAL*
 *   ├       ── TRACKING*
 *   ├       ── LOSS*
 *   ├       ── NO_LOCK*
 *   └─ ERROR ─ ERROR_ACTIVE
 * 
 *      * = declared, not yet implemented
 *
 * KEYMAP (keep in sync with the key_* handlers in section 5):
 *   GLOBAL (any non-FATAL leaf):
 *     k       soft kill -> STBY_HOLD
 *     [       capture origin from telemetry
 *     ?       help
 *     Ctrl+R  clear FATAL latch -> STBY_HOLD
 *   NAV (suppressed where leaf_traps_nav()==1):
 *     1/2/3   STBY_HOLD / MANU_JOYSTICK / AUTO
 *   MANU_JOYSTICK leaf:
 *     r/a     RATE / ABSOLUTE      e  exit -> STBY_HOLD
 *     ]       -> AUTO_HOME
 *
 * DISPATCH PRECEDENCE (app_dispatch_key):
 *     FATAL-lockout  >  global  >  leaf  >  nav
*/

#include "app_statemachine.h"
#include "hamfly.h"
#include "sbc_comms.h"
#include "telemetry.h"
#include "sbc_comms.h"
#include <project.h>  // For UART
#include <math.h>
#include <string.h>
#include <stdio.h>

// For timing, PSoC incremented with looptimer ISR.
extern volatile uint32_t g_tick_ms;

// Hierarchy %================================================================%
// Heirarchy allows shared entry/exit behaviour.
// ROOT is the base of the walk.
static const state_t parent_of[STATE_COUNT] = {
    [ROOT]            = ROOT,  // Book keeping
    [STBY]            = ROOT,  // Parent states
    [MANU]            = ROOT,
    [AUTO]            = ROOT,
    [ERROR]           = ROOT,

    [STBY_DEFER]      = STBY,  // Two types of standby
    [STBY_HOLD]       = STBY,

    [MANU_JOYSTICK]   = MANU,

    [AUTO_HOME]       = AUTO,  // Go to 0, 0
    [AUTO_ACQ_GPS]    = AUTO,  // Go to GPS vector pointing
    [AUTO_ACQ_SPIRAL] = AUTO,  // Spiral search for lock
    [AUTO_TRACKING]   = AUTO,  // Closed loop tracking
    [AUTO_LOSS]       = AUTO,  // Loss of centeroiding packets
    [AUTO_NO_LOCK]    = AUTO,  // No lock after ACQ attempts

    [ERROR_ACTIVE]    = ERROR,  // Error state. TODO: Severity levels.
};

// Per-state hook tables %====================================================%
// Forward declaration of state handlers.
typedef void    (*state_fn_t)(app_ctx_t *);         // Entry and Exit handlers
typedef uint8_t (*key_fn_t)  (app_ctx_t *, char);   // Input handlers
typedef void    (*build_fn_t)(const app_ctx_t *, hamfly_control_t *); //Packet
typedef uint8_t (*guard_fn_t)(const app_ctx_t *);   // 1=valid, 0=refuse

// State change guarding
// TODO: Add data validation not just check if set.
static uint8_t guard_auto_home   (const app_ctx_t *);  // Is 'home' set? Valid?
static uint8_t guard_auto_acq_gps(const app_ctx_t *);  // Is a target set? Valid?

// Entry handlers
static void entry_stby_defer    (app_ctx_t *);
static void entry_stby_hold     (app_ctx_t *);
static void entry_manu_joystick (app_ctx_t *);
static void entry_auto_home     (app_ctx_t *);
static void entry_auto_acq_gps  (app_ctx_t *);
static void entry_auto_acq_spiral(app_ctx_t *);
static void entry_auto_tracking (app_ctx_t *);
static void entry_auto_loss     (app_ctx_t *);
static void entry_auto_no_lock  (app_ctx_t *);
static void entry_error_active  (app_ctx_t *);

// Exit handlers
static void exit_manu_joystick  (app_ctx_t *);
static void exit_auto_home      (app_ctx_t *);
static void exit_auto_acq_gps   (app_ctx_t *);

// Key handlers
static uint8_t key_manu_joystick(app_ctx_t *, char);
static uint8_t key_auto_home    (app_ctx_t *, char);
static uint8_t key_auto_acq_gps (app_ctx_t *, char);
static uint8_t key_auto_tracking(app_ctx_t *, char);
static uint8_t key_auto_loss    (app_ctx_t *, char);
static uint8_t key_auto_no_lock (app_ctx_t *, char);
static uint8_t key_error_active (app_ctx_t *, char);

// Packet building handlers
static void build_defer         (const app_ctx_t *, hamfly_control_t *);
static void build_hold          (const app_ctx_t *, hamfly_control_t *);
static void build_manu_joystick (const app_ctx_t *, hamfly_control_t *);
static void build_auto_home     (const app_ctx_t *, hamfly_control_t *);
static void build_auto_acq_gps  (const app_ctx_t *, hamfly_control_t *);
static void build_auto_tracking (const app_ctx_t *, hamfly_control_t *);
static void build_error         (const app_ctx_t *, hamfly_control_t *);

// GPS vector math + slew helpers (internal)
static float gps_bearing_deg (float lat1, float lon1, float lat2, float lon2);
static float gps_elev_deg    (float lat1, float lon1, float alt1_m,
                              float lat2, float lon2, float alt2_m);
static void  gps_start_slew    (app_ctx_t *);
static void  gps_update_pointing(app_ctx_t *);
static void  gps_check_settle  (app_ctx_t *);

// Telemetry Helpers

// Gets the current pan and tilt as azm alt in degrees from latest gimble
// status telemetry. Returns 1 on success, 0 on fail or stale. Read-only
// helper.
static uint8_t gimbal_pan_tilt_deg(const app_ctx_t *ctx, float *pan_deg, float *tilt_deg);

// Table of entry guards.
static const guard_fn_t can_enter[STATE_COUNT] = {
    [AUTO_HOME]    = guard_auto_home,     // needs origin set + live telemetry
    [AUTO_ACQ_GPS] = guard_auto_acq_gps,  // needs a GPS target
};

static const state_fn_t on_entry[STATE_COUNT] = {
    [STBY]            = NULL,   /* parent: shared STBY entry, if ever needed */
    [MANU]            = NULL,   /* parent */
    [AUTO]            = NULL,   /* parent */
    [ERROR]           = NULL,   /* parent */
    [STBY_DEFER]      = entry_stby_defer,
    [STBY_HOLD]       = entry_stby_hold,
    [MANU_JOYSTICK]   = entry_manu_joystick,
    [AUTO_HOME]       = entry_auto_home,
    [AUTO_ACQ_GPS]    = entry_auto_acq_gps,
    [AUTO_ACQ_SPIRAL] = entry_auto_acq_spiral,
    [AUTO_TRACKING]   = entry_auto_tracking,
    [AUTO_LOSS]       = entry_auto_loss,
    [AUTO_NO_LOCK]    = entry_auto_no_lock,
    [ERROR_ACTIVE]    = entry_error_active,
};

static const state_fn_t on_exit[STATE_COUNT] = {
    [STBY]            = NULL,   /* parent */
    [MANU]            = NULL,   /* parent */
    [AUTO]            = NULL,   /* parent */
    [ERROR]           = NULL,   /* parent */
    [STBY_DEFER]      = NULL,
    [STBY_HOLD]       = NULL,
    [MANU_JOYSTICK]   = exit_manu_joystick,
    [AUTO_HOME]       = exit_auto_home,
    [AUTO_ACQ_GPS]    = exit_auto_acq_gps,
    [AUTO_ACQ_SPIRAL] = NULL,   /* TODO */
    [AUTO_TRACKING]   = NULL,  
    [AUTO_LOSS]       = NULL,   /* TODO */
    [AUTO_NO_LOCK]    = NULL,   /* TODO */
    [ERROR_ACTIVE]    = NULL,
};

static const key_fn_t on_key[STATE_COUNT] = {
    [STBY_DEFER]      = NULL,
    [STBY_HOLD]       = NULL,
    [MANU_JOYSTICK]   = key_manu_joystick,
    [AUTO_HOME]       = key_auto_home,
    [AUTO_ACQ_GPS]    = key_auto_acq_gps,
    [AUTO_ACQ_SPIRAL] = NULL,   /* TODO */
    [AUTO_TRACKING]   = key_auto_tracking,
    [AUTO_LOSS]       = key_auto_loss,
    [AUTO_NO_LOCK]    = key_auto_no_lock,
    [ERROR_ACTIVE]    = key_error_active,
};

static const build_fn_t on_build[STATE_COUNT] = {
    [STBY_DEFER]      = build_defer,
    [STBY_HOLD]       = build_hold,
    [MANU_JOYSTICK]   = build_manu_joystick,  // NOTE: Nudge folded into joysticl leaf.
    [AUTO_HOME]       = build_auto_home,
    [AUTO_ACQ_GPS]    = build_auto_acq_gps,
    [AUTO_ACQ_SPIRAL] = build_hold,          /* placeholder: zero rates, hold */
    [AUTO_TRACKING]   = build_auto_tracking,
    [AUTO_LOSS]       = build_hold,          /* placeholder: hold position */
    [AUTO_NO_LOCK]    = build_hold,          /* placeholder: hold position */
    [ERROR_ACTIVE]    = build_error,
};

// Lowest common ancestor (LCA) walk %========================================%
// For a given state, walk up its parents to ROOT, then walk down to incoming
// state. Allows for heirarchical and shared entry/exit behaviour.
static uint8_t depth_of(state_t s)
{
    uint8_t d = 0;
    while (s != ROOT) { s = parent_of[s]; d++; }
    return d;
}

static state_t lca(state_t a, state_t b)
{
    uint8_t da = depth_of(a), db = depth_of(b);
    while (da > db) { a = parent_of[a]; da--; }
    while (db > da) { b = parent_of[b]; db--; }
    while (a != b)  { a = parent_of[a]; b = parent_of[b]; }
    return a;
}

// Transition driver %========================================================%
// Walk src up to LCA (running on_exit), then LCA down to target (running
// on_entry). Self-transitions are a no-op. ROOT is the absorbing root, so
// any pair has an LCA.

void app_transition(app_ctx_t *ctx, state_t target)
{
    // Check entry guard
    if (can_enter[target] && !can_enter[target](ctx)) {
        UART_DEBUG_PutString("[FSM] entry guard failed -> STBY_HOLD\r\n");
        target = STBY_HOLD;
    }
    
    // Self transition handle.
    state_t src = ctx->state;
    if (target == src) return;
    
    // LCA walk.
    state_t common = lca(src, target);

    // Shared parents require no duplicate on_exit calls.
    // Walk up path, running on_exit.
    for (state_t s = src; s != common; s = parent_of[s]) {
        if (on_exit[s]) on_exit[s](ctx);
    }
    
    ctx->prev_leaf = src;  // Return to last state book-keeping.

    // Walk down the path, running on_entry.
    state_t path[MAX_HFSM_DEPTH];
    uint8_t n = 0;
    for (state_t s = target; s != common; s = parent_of[s]) {
        path[n++] = s;
    }
    while (n--) {
        if (on_entry[path[n]]) on_entry[path[n]](ctx);
    }
    
    // Store new state.
    ctx->state          = target;
    ctx->state_entry_ms = g_tick_ms;
}

// On boot.
void app_start(app_ctx_t *ctx)
{
    // Walk entry conditions from ROOT to the init state,
    state_t target = ctx->state;
    ctx->state = ROOT;
    app_transition(ctx, target);
}

// On error: Handles severity.
void app_raise_error(app_ctx_t *ctx, err_sev_t sev, const char *msg)
{
    // User error: Invalid request, do not change and notify user.
    if (sev == SEV_USER) {
        UART_DEBUG_PutString("[USER] ");
        UART_DEBUG_PutString(msg);
        UART_DEBUG_PutString("\r\n");
        return;
    }
    
    // Fatal error flag.
    if (ctx->fatal_latched && sev != SEV_FATAL) return;
    
    ctx->err_sev = sev;
    ctx->err_msg = msg;
    if (sev == SEV_FATAL) ctx->fatal_latched = 1u;
    
    app_transition(ctx, ERROR_ACTIVE);  // Go to error state.
}

// Entry guards
static uint8_t guard_auto_home(const app_ctx_t *ctx)
{
    float p, t;
    return ctx->origin_set && gimbal_pan_tilt_deg(ctx, &p, &t);
}

static uint8_t guard_auto_acq_gps(const app_ctx_t *ctx)
{
    return ctx->gps_target_set;
}

// Lookup table for state names, used by serial monitor.
static const char *const state_names[STATE_COUNT] = {
    [ROOT] = "ROOT",
    [STBY] = "STBY", [MANU] = "MANU", [AUTO] = "AUTO", [ERROR] = "ERROR",
    [STBY_DEFER]      = "STBY_DEFER",
    [STBY_HOLD]       = "STBY_HOLD",
    [MANU_JOYSTICK]   = "MANU_JOYSTICK",
    [AUTO_HOME]       = "AUTO_HOME",
    [AUTO_ACQ_GPS]    = "AUTO_ACQ_GPS",
    [AUTO_ACQ_SPIRAL] = "AUTO_ACQ_SPIRAL",
    [AUTO_TRACKING]   = "AUTO_TRACKING",
    [AUTO_LOSS]       = "AUTO_LOSS",
    [AUTO_NO_LOCK]    = "AUTO_NO_LOCK",
    [ERROR_ACTIVE]    = "ERROR_ACTIVE",
};

// Helper for serial monitor.
const char *app_state_name(state_t s) { return state_names[s]; }

// Telemetry %===============================================================%
void app_telem_tick(app_ctx_t *ctx)
{
    // Do the telemetry grabs if gimble connected
    if (!ctx->gimbal) return;

    static uint32_t last_hot_ms  = 0u;  // Hot is the Movi status
    static uint32_t last_link_ms = 0u;  // Cold are all other sensors
    if (g_tick_ms - last_hot_ms  >= CONTROL_PERIOD_MS) {  //  10 Hz
        last_hot_ms = g_tick_ms;
        telemetry_send_hot_sbc(ctx);
    }
    if (g_tick_ms - last_link_ms >= DIAG_PERIOD_MS) {     //  1 Hz
        last_link_ms = g_tick_ms;
        telemetry_send_link_sbc(ctx);
        telemetry_send_power_sbc(ctx);
        telemetry_send_env_sbc(ctx);
        telemetry_send_gps_sbc(ctx);
        telemetry_send_baro_sbc(ctx);
        telemetry_send_mag_sbc(ctx);
    }
}

// %==========================================================================%
// %                               Standby                                    %
// %==========================================================================%

// STBY_DEFER %===============================================================%
// On Enter: Print message, wait for user input.
static void entry_stby_defer(app_ctx_t *ctx)
{
    (void)ctx;
    UART_DEBUG_PutString("\r\n[STBY_DEFER] '1' hold, '2' manual, '3' auto, '?' help\r\n> ");
}
// On Exit: None


// STBY_HOLD %================================================================%
// On Enter
static void entry_stby_hold(app_ctx_t *ctx)
{
    (void)ctx;
    UART_DEBUG_PutString("\r\n[STBY_HOLD] rates zeroed\r\n> ");
}
// On Exit: None


// %==========================================================================%
// %                                Manual                                    %
// %==========================================================================%

// MANU_JOYSTICK %============================================================%
// On Enter: Enable joystick, print message.
static void entry_manu_joystick(app_ctx_t *ctx)
{
    (void)ctx;
    UART_DEBUG_PutString("\r\n[MANU] joystick=rate  nudge: numpad=fine  ijkl=0.5 deg wasd=1 deg e=exit\r\n> ");
}
// On Exit: Disable joystick, zero control mode.
static void exit_manu_joystick(app_ctx_t *ctx)
{
    ctx->nudge_hold = 0u;  // Clean nudge if in progress.
}

// %==========================================================================%
// %                                 Auto                                     %
// %==========================================================================%
// Auto has sub-modes: HOME, GPS, SPIRAL, TRACKING, LOSS, NO LOCK
void app_auto_tick(app_ctx_t *ctx)
{
    // %--- AUTO_HOME: settle then drop to STBY_HOLD ------------------------%
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

    // %--- AUTO_ACQ_GPS: re-point at 1 Hz, settle check every loop ---------%
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

    // %--- AUTO_TRACKING: pull freshest centroid into P-control state ------%
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

// AUTO_HOME %================================================================%
static void entry_auto_home(app_ctx_t *ctx)
{
    UART_DEBUG_PutString("\r\n[AUTO_HOME] slewing to origin  e=exit\r\n> ");
    if (!ctx->origin_set) {
        app_raise_error(ctx, SEV_USER, "no origin set (press '[' first)");
        app_transition(ctx, STBY_HOLD);
        return;
    }
    float pan, tilt;
    if (!gimbal_pan_tilt_deg(ctx, &pan, &tilt)) {
        app_raise_error(ctx, SEV_USER, "no telemetry (home aborted)");
        app_transition(ctx, STBY_HOLD);
        return;
    }
    ctx->nudge_base_pan_deg  = pan;
    ctx->nudge_base_tilt_deg = tilt;
    ctx->tgt_pan_deg         = ctx->origin_pan_deg;
    ctx->tgt_tilt_deg        = ctx->origin_tilt_deg;
    ctx->nudge_hold          = 1u;
    ctx->nudge_start_ms      = g_tick_ms;
}
static void exit_auto_home(app_ctx_t *ctx) { ctx->nudge_hold = 0u; }

static uint8_t key_auto_home(app_ctx_t *ctx, char k)
{
    switch (k) {
        case 'e': app_transition(ctx, STBY_HOLD); return 1;
        default:  return 0;
    }
}

static void build_auto_home(const app_ctx_t *ctx, hamfly_control_t *out)
{
    out->kill      = 0u;
    out->roll_mode = HAMFLY_DEFER;
    out->roll      = 0.0f;
    out->pan_mode  = out->tilt_mode = HAMFLY_ABSOLUTE;
    out->pan  = DEG_TO_UNIT(ctx->tgt_pan_deg - ctx->nudge_base_pan_deg);
    out->tilt = DEG_TO_UNIT(ctx->tgt_tilt_deg);
}

// AUTO_GPS %=================================================================%
// All float trig; PSoC 5 LP has no HW FPU, so callers run these at most 1 Hz.
static float gps_bearing_deg(float lat1, float lon1, float lat2, float lon2)
{
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
    float la1 = lat1 * DEG_TO_RAD, la2 = lat2 * DEG_TO_RAD;
    float dlo = (lon2 - lon1) * DEG_TO_RAD;
    float dla = la2 - la1;
    float a   = sinf(dla * 0.5f) * sinf(dla * 0.5f)
              + cosf(la1) * cosf(la2) * sinf(dlo * 0.5f) * sinf(dlo * 0.5f);
    float dist_m = 2.0f * GPS_EARTH_R_M * atan2f(sqrtf(a), sqrtf(1.0f - a));
    if (dist_m < 1.0f) dist_m = 1.0f;  // guard div/0 at zero range
    return atan2f(alt2_m - alt1_m, dist_m) * RAD_TO_DEG;
}

// Snap the active target into the working copy and reset the slew.
static void gps_start_slew(app_ctx_t *ctx)
{
    ctx->gps_work_lat_raw = ctx->gps_target_lat_raw;
    ctx->gps_work_lon_raw = ctx->gps_target_lon_raw;
    ctx->gps_work_alt_mm  = ctx->gps_target_alt_mm;
    ctx->gps_settled      = 0u;
    ctx->gps_new_target   = 0u;
}

// Recompute abs_pan/tilt setpoints from the working copy + live gimbal GPS.
static void gps_update_pointing(app_ctx_t *ctx)
{
    hamfly_telemetry_t st;
    hamfly_get_telemetry(ctx->gimbal, &st);
    if (!st.gps_valid) return;
    float tlat = (float)ctx->gps_work_lat_raw * 1e-7f;
    float tlon = (float)ctx->gps_work_lon_raw * 1e-7f;
    float talt = (float)ctx->gps_work_alt_mm  * 0.001f;
    ctx->abs_pan_target  = DEG_TO_UNIT(
        gps_bearing_deg(st.gps_lat_deg, st.gps_lon_deg, tlat, tlon) - st.gps_heading_deg);
    ctx->abs_tilt_target = DEG_TO_UNIT(
        gps_elev_deg(st.gps_lat_deg, st.gps_lon_deg, st.gps_alt_m, tlat, tlon, talt));
}

static void gps_check_settle(app_ctx_t *ctx)
{
    uint32_t now = g_tick_ms;
    if (now - ctx->gps_last_sample_ms < GPS_SETTLE_PERIOD_MS) return;

    float pan, tilt;
    if (!gimbal_pan_tilt_deg(ctx, &pan, &tilt)) return;  // no telemetry -> can't judge

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

    if (ctx->gps_new_target && (ctx->gps_settled || (arrived && slow))) {
        UART_DEBUG_PutString("[AUTO_ACQ_GPS] new target -> re-slew\r\n");
        gps_start_slew(ctx);        // copies target->work, clears flag + settled
        gps_update_pointing(ctx);
        return;
    }
    if (!ctx->gps_settled && arrived && slow) {
        ctx->gps_settled = 1u;
        UART_DEBUG_PutString("[AUTO_ACQ_GPS] settled\r\n");
    }
}

static void entry_auto_acq_gps(app_ctx_t *ctx)
{
    if (!ctx->gps_target_set) {
        UART_DEBUG_PutString("\r\n[AUTO_ACQ_GPS] no GPS target -> STBY_HOLD\r\n");
        app_transition(ctx, STBY_HOLD);
        return;
    }
    UART_DEBUG_PutString("\r\n[AUTO_ACQ_GPS] open-loop GPS pointing  e=exit\r\n> ");
    ctx->gps_last_sample_ms = 0u;
    gps_start_slew(ctx);        // snap target -> work, start fresh
    gps_update_pointing(ctx);   // initial bearing
}
static void exit_auto_acq_gps(app_ctx_t *ctx)
{
    ctx->abs_pan_target = ctx->abs_tilt_target = 0.0f;
}
static uint8_t key_auto_acq_gps(app_ctx_t *ctx, char k)
{
    switch (k) {
        case 'e': app_transition(ctx, STBY_HOLD); return 1;
        default:  return 0;
    }
}
static void build_auto_acq_gps(const app_ctx_t *ctx, hamfly_control_t *out)
{
    out->kill      = 0u;
    out->roll_mode = HAMFLY_DEFER;
    out->roll      = 0.0f;
    out->pan_mode  = out->tilt_mode = HAMFLY_ABSOLUTE;
    out->pan  = CLAMP(ctx->abs_pan_target,  -1.0f, 1.0f);
    out->tilt = CLAMP(ctx->abs_tilt_target, -1.0f, 1.0f);
}

// AUTO_SPIRAL %=================================================================%
static void entry_auto_acq_spiral(app_ctx_t *ctx)
{
    (void)ctx;
    UART_DEBUG_PutString("\r\n[AUTO_ACQ_SPIRAL] placeholder (NI).\r\n> ");
}

// AUTO_TRACKING %=================================================================%
static void entry_auto_tracking(app_ctx_t *ctx)
{
    ctx->track_cx_last = 0;  // Clear last centroid value (TODO: Init wht the first centroid packet? Wait for centroid packet before we enter this state?)
    ctx->track_cy_last = 0;
    UART_DEBUG_PutString("\r\n[AUTO_TRACKING] closed-loop  e=exit\r\n> ");
}
static uint8_t key_auto_tracking(app_ctx_t *ctx, char k)
{
    // TEMP: Debug exit to hold with e over serial.
    switch (k) {
        case 'e': app_transition(ctx, STBY_HOLD); return 1;
        default:  return 0;
    }
}

static void build_auto_tracking(const app_ctx_t *ctx, hamfly_control_t *out)
{
    // Construct control packet from centroid error signal.
    out->kill      = 0u;
    out->roll_mode = HAMFLY_DEFER;  // No roll
    out->roll      = 0.0f;
    out->pan_mode  = out->tilt_mode = HAMFLY_RATE;
    float cx_mrad = (float)ctx->track_cx_last * 0.1f;  // 0.1 mrad to mrad
    float cy_mrad = (float)ctx->track_cy_last * 0.1f;
    // Simple proportional gain linear feedback.
    out->pan  =  CLAMP(ctx->track_kp * cx_mrad, -1.0f, 1.0f);  // TODO: Confirm direction with physical test
    out->tilt = -CLAMP(ctx->track_kp * cy_mrad, -1.0f, 1.0f);  // +cy = up
}

// AUTO_LOSS %=================================================================%
static void entry_auto_loss(app_ctx_t *ctx)
{
    // TODO: Loss should hold and wait. If packets re-commence we go back into tracking,
    // TODO: Let user trigger spiral search, GPS re-aqc, or exit to STBY later.
    (void)ctx;
    UART_DEBUG_PutString("\r\n[AUTO_LOSS] centroid lost  1=hold  3=retry\r\n> ");
}
static uint8_t key_auto_loss(app_ctx_t *ctx, char k)
{
    switch (k) {  // Keep stationary, spiral search if requested.
        case '3': app_transition(ctx, AUTO_ACQ_SPIRAL); return 1;
        default:  return 0;
    }
}

// AUTO_NO_LOCK %=================================================================%
static void entry_auto_no_lock(app_ctx_t *ctx)
{
    (void)ctx;
    UART_DEBUG_PutString("\r\n[AUTO_NO_LOCK] acquisition failed  1=hold  3=retry\r\n> ");
}
static uint8_t key_auto_no_lock(app_ctx_t *ctx, char k)
{
    switch (k) {
        case '3': app_transition(ctx, AUTO_ACQ_SPIRAL); return 1;
        default:  return 0;
    }
}
// %==========================================================================%
// %                                 Error                                    %
// %==========================================================================%
static const char *sev_str(err_sev_t s)
{
    switch (s) {
        case SEV_WARN:     return "WARN";
        case SEV_SOFTWARE: return "SOFTWARE";
        case SEV_FATAL:    return "FATAL";
        default:           return "USER";
    }
}
static void entry_error_active(app_ctx_t *ctx)
{
    UART_DEBUG_PutString("\r\n[ERROR:");
    UART_DEBUG_PutString(sev_str(ctx->err_sev));
    UART_DEBUG_PutString("] ");
    UART_DEBUG_PutString(ctx->err_msg ? ctx->err_msg : "(no msg)");
    if (ctx->err_sev == SEV_FATAL && ctx->gimbal)
    {
        hamfly_kill(ctx->gimbal);
        UART_DEBUG_PutString("\r\nFATAL latched. Power cycle or Ctrl+R to clear.\r\n");
    }
    else    UART_DEBUG_PutString("\r\nAny key to acknowledge.\r\n");
}

// %==========================================================================%
// %                             Key Handling                                 %
// %==========================================================================%
#define KEY_CTRL_R 0x12  // Reset ASCII code.

static uint8_t gimbal_pan_tilt_deg(const app_ctx_t *ctx, float *pan_deg, float *tilt_deg)
{
    if (!ctx->gimbal) return 0u;
    hamfly_telemetry_t st;
    hamfly_get_telemetry(ctx->gimbal, &st);
    if (!st.valid) return 0u;
    float roll_deg;                                   /* discarded */
    return hamfly_telemetry_to_euler(&st, pan_deg, tilt_deg, &roll_deg);
}

/* Capture current gimbal attitude as the origin (global '['). */
static void set_origin(app_ctx_t *ctx)
{
    if (!ctx->gimbal) { app_raise_error(ctx, SEV_USER, "no gimbal handle"); return; }

    float pan, tilt;
    if (!gimbal_pan_tilt_deg(ctx, &pan, &tilt)) {
        app_raise_error(ctx, SEV_USER, "no telemetry (origin not set)");
        return;
    }
    ctx->origin_pan_deg  = pan;
    ctx->origin_tilt_deg = tilt;
    ctx->origin_set = 1u;
    UART_DEBUG_PutString("\r\n[ORIGIN] captured.\r\n");
}

static void print_help(const app_ctx_t *ctx)
{
    (void)ctx;
    UART_DEBUG_PutString(
        "\r\n--- keys ---\r\n"
        " 1 hold   2 manual       3 auto\r\n"
        " x kill   [ set-origin   ? help\r\n"
        " e exit   ] home\r\n"
        " (in MANU) numpad/ijkl/wasd nudge\r\n");
}

/* ---- always-global keys (honored in every non-FATAL leaf) ------------- */
static uint8_t handle_global_key(app_ctx_t *ctx, char k)
{
    switch (k) {
        case KEY_CTRL_R:                       /* clear FATAL latch */
            if (ctx->fatal_latched) {
                ctx->fatal_latched = 0u;
                ctx->err_sev = SEV_USER;
                ctx->err_msg = NULL;
                UART_DEBUG_PutString("\r\nFATAL cleared.\r\n");
                app_transition(ctx, STBY_HOLD);
            }
            return 1;                          /* reserved key, always consumed */
        case 'x':                              /* soft kill */
            if (ctx->gimbal) hamfly_kill(ctx->gimbal);
            app_transition(ctx, STBY_HOLD);
            return 1;
        case '[': set_origin(ctx);  return 1;
            case '?': print_help(ctx);  return 1;
            default:  return 0;
    }
}

/* ---- navigation keys (suppressed in trapping leaves) ------------------ */
static uint8_t leaf_traps_nav(state_t s)
{
    /* Active control states swallow nav keys; loss/no-lock allow user escape. */
    return (s == MANU_JOYSTICK   ||
            s == AUTO_HOME       ||
            s == AUTO_ACQ_GPS    ||
            s == AUTO_ACQ_SPIRAL ||
            s == AUTO_TRACKING);
}
static uint8_t handle_nav_key(app_ctx_t *ctx, char k)
{
    switch (k) {
        case '1': app_transition(ctx, STBY_HOLD);     return 1;
        case '2': app_transition(ctx, MANU_JOYSTICK); return 1;
        case '3': app_transition(ctx, AUTO_TRACKING); return 1;
        default:  return 0;
    }
}

/* ---- leaf-specific keys ----------------------------------------------- */
static uint8_t key_error_active(app_ctx_t *ctx, char k)
{
    (void)k;
    if (ctx->err_sev != SEV_FATAL) {            /* WARN/SOFTWARE ack */
        state_t back = (ctx->prev_leaf == ERROR_ACTIVE) ? STBY_HOLD : ctx->prev_leaf;
        app_transition(ctx, back);
        return 1;
    }
    return 0;                                   /* FATAL: only Ctrl+R (handled in lockout) */
}

/* ---- public entry point ----------------------------------------------- */
uint8_t app_dispatch_key(app_ctx_t *ctx, char key)
{
    if (ctx->state >= STATE_COUNT) return 0;          /* Patch 12 guard */

    if (ctx->fatal_latched)                            /* FATAL lockout */
        return (key == KEY_CTRL_R) ? handle_global_key(ctx, key) : 0;

    if (handle_global_key(ctx, key)) return 1;         /* k, [, ?, Ctrl+R */

    key_fn_t f = on_key[ctx->state];
    if (f && f(ctx, key)) return 1;                    /* leaf keys */

    if (!leaf_traps_nav(ctx->state))                   /* nav, unless trapped */
        return handle_nav_key(ctx, key);

    return 0;
}

// %==========================================================================%
// %                       Gimbal Control Packet                              %
// %==========================================================================%

// Send a defer packet to the gimble. Defer releases control.
// Defer is really an empty packet! But lets set explicitly.
void build_defer(const app_ctx_t *ctx, hamfly_control_t *out)
{
    (void)ctx;
    memset(out, 0, sizeof *out);          /* DEFER == 0, so this releases */
    out->pan_mode = HAMFLY_DEFER;
    out->tilt_mode = HAMFLY_DEFER;
    out->roll_mode = HAMFLY_DEFER;
}
// Active hold at vel=0.
void build_hold(const app_ctx_t *ctx, hamfly_control_t *out)
{
    (void)ctx;
    memset(out, 0, sizeof *out);
    out->pan_mode  = HAMFLY_RATE;          /* RATE with value 0 == "stay put" */
    out->tilt_mode = HAMFLY_RATE;
    out->roll_mode = HAMFLY_DEFER;         /* roll has always been deferred here */
}

// Build packet from joystick inputs.
static void build_manu_joystick(const app_ctx_t *ctx, hamfly_control_t *out) {
    out->roll_mode = HAMFLY_DEFER;
    out->roll      = 0.0f;
    out->kill      = 0u;
    // Nudge vs joystick control mode
    if (ctx->nudge_hold) {
        out->pan_mode = out->tilt_mode = HAMFLY_ABSOLUTE;
        out->pan  = DEG_TO_UNIT(ctx->tgt_pan_deg - ctx->nudge_base_pan_deg); /* pan rebased -> relative delta */
        out->tilt = DEG_TO_UNIT(ctx->tgt_tilt_deg);                          /* tilt true-absolute */
    } else {
        out->pan_mode = out->tilt_mode = HAMFLY_RATE;
        out->pan  =  ctx->cmd.u[CH_X];
        out->tilt = -ctx->cmd.u[CH_Y];
    }

}

// Build packet for when in ERROR state
static void build_error(const app_ctx_t *ctx, hamfly_control_t *out) {
    build_defer(ctx, out);  // Error
    if (ctx->err_sev == SEV_FATAL) out->kill = 1u;
}


// Do build. For states without a build function returns zeroed packet.
void app_build_control(const app_ctx_t *ctx, hamfly_control_t *out)
{
    build_fn_t f = on_build[ctx->state];
    if (f)  f(ctx, out);
    else    build_defer(ctx, out);  // Safe default.
}

// TEMP: Nudge handler
static void nudge_apply(app_ctx_t *ctx, float dpan_deg, float dtilt_deg)
{
    if (!ctx->nudge_hold) {// Capture nudge baseline
        float p, t;
        if (!gimbal_pan_tilt_deg(ctx, &p, &t)) {
            app_raise_error(ctx, SEV_USER, "no telemetry (nudge ignored).");
            return;
        }
        UART_DEBUG_PutString("Nudging!\r\n");  // TODO: Verbose nudge print
        ctx->nudge_base_pan_deg  = p;
        ctx->nudge_base_tilt_deg = t;
        ctx->tgt_pan_deg  = p;
        ctx->tgt_tilt_deg = t;
        ctx->nudge_hold = 1u;
    }
    ctx->tgt_pan_deg  = CLAMP(ctx->tgt_pan_deg  + dpan_deg,
                              ctx->origin_pan_deg  + LIMIT_PAN_MIN_DEG,
                              ctx->origin_pan_deg  + LIMIT_PAN_MAX_DEG);
    ctx->tgt_tilt_deg = CLAMP(ctx->tgt_tilt_deg + dtilt_deg,
                              ctx->origin_tilt_deg + LIMIT_TILT_MIN_DEG,
                              ctx->origin_tilt_deg + LIMIT_TILT_MAX_DEG);
    ctx->nudge_start_ms = g_tick_ms;  // Restart the dwell timer
}

static uint8_t key_manu_joystick(app_ctx_t *ctx, char k)
{
    switch (k) {
        case '8': nudge_apply(ctx, 0,    +NUDGE_LSB_DEG);  return 1;  // Numpad
        case '2': nudge_apply(ctx, 0,    -NUDGE_LSB_DEG);  return 1;  // Numpad
        case '4': nudge_apply(ctx, -NUDGE_LSB_DEG,    0);  return 1;  // Numpad
        case '6': nudge_apply(ctx, +NUDGE_LSB_DEG,    0);  return 1;  // Numpad

        case 'i': nudge_apply(ctx, 0,    +NUDGE_FINE_DEG);   return 1;  // ijkl
        case 'k': nudge_apply(ctx, 0,    -NUDGE_FINE_DEG);   return 1;  // ijkl
        case 'j': nudge_apply(ctx, -NUDGE_FINE_DEG,   0);    return 1;  // ijkl
        case 'l': nudge_apply(ctx, +NUDGE_FINE_DEG,   0);    return 1;  // ijkl

        case 'w': nudge_apply(ctx, 0,    +NUDGE_COARSE_DEG); return 1;  // wasd
        case 's': nudge_apply(ctx, 0,    -NUDGE_COARSE_DEG); return 1;  // wasd
        case 'a': nudge_apply(ctx, -NUDGE_COARSE_DEG, 0);    return 1;  // wasd
        case 'd': nudge_apply(ctx, +NUDGE_COARSE_DEG, 0);    return 1;  // wasd
        case 'e': app_transition(ctx, STBY_HOLD);                return 1;  // Exit
        default:  return 0;
    }
}

// TEMP: Manual tick for nudge
void app_manual_tick(app_ctx_t *ctx)
{
    if (ctx->state != MANU_JOYSTICK || !ctx->nudge_hold) return;
    uint32_t elapsed = g_tick_ms - ctx->nudge_start_ms;
    if (elapsed < NUDGE_MIN_DWELL_MS) return;       /* let the step physically land */

    float p, t; uint8_t arrived = 0u;
    if (gimbal_pan_tilt_deg(ctx, &p, &t))
        arrived = (fabsf(p - ctx->tgt_pan_deg)  < NUDGE_SETTLE_DEG) &&
                  (fabsf(t - ctx->tgt_tilt_deg) < NUDGE_SETTLE_DEG);
    if (arrived || elapsed >= NUDGE_TIMEOUT_MS)
        ctx->nudge_hold = 0u;                        /* rate=0 holds the new attitude */
}

// Helper - Validate state change request
static uint8_t sbc_state_is_requestable(uint8_t s)
{
    // Not implemented.
    switch ((state_t)s) {
        case STBY_HOLD:
        case MANU_JOYSTICK:
        case AUTO_HOME:
        case AUTO_ACQ_GPS:
        case AUTO_ACQ_SPIRAL:
        case AUTO_TRACKING:
            return 1u;
        default:
            return 0u;
    }
}

void app_sbc_tick(app_ctx_t *ctx)
{
    // 1) GPS target — always commit to ctx + flag. Run BEFORE the state req so
    //    a "target then go" pair landing in one tick has the target ready when
    //    entry_auto_acq_gps checks gps_target_set.
    payload_gps_target_t tgt;
    if (sbc_get_gps_target(&tgt)) {
        ctx->gps_target_lat_raw = tgt.lat_raw;
        ctx->gps_target_lon_raw = tgt.lon_raw;
        ctx->gps_target_alt_mm  = tgt.alt_mm;
        ctx->gps_target_set     = 1u;
        ctx->gps_new_target     = 1u;   // ACQ_GPS picks this up on next settle
    }

    // 2) State change request
    payload_state_req_t req;
    if (!sbc_get_state_req(&req)) return;

    if (ctx->fatal_latched) {
        sbc_send_state_ack((uint8_t)ctx->state, STATE_ACK_REJECTED);
        return;
    }
    if (!sbc_state_is_requestable(req.requested_state)) {
        sbc_send_state_ack((uint8_t)ctx->state, STATE_ACK_INVALID);
        char b[64];
        snprintf(b, sizeof b, "[SBC] invalid state req=%u\r\n",
                 (unsigned)req.requested_state);
        UART_DEBUG_PutString(b);
        return;
    }

    state_t target = (state_t)req.requested_state;
    char b[64];
    snprintf(b, sizeof b, "[SBC] state req -> %s\r\n", app_state_name(target));
    UART_DEBUG_PutString(b);
    app_transition(ctx, target);
    sbc_send_state_ack((uint8_t)ctx->state, STATE_ACK_OK);
}