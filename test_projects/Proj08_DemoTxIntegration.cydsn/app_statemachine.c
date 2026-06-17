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

#include "hamfly.h"
#include "sbc_comms.h"
#include "telemetry.h"
#include "utils.h"    // nudge_apply
#include <project.h>  // For UART
#include <math.h> // TODO: Remove?
#include <string.h>
#include <stdio.h>

// Substates
#include "app_statemachine.h"
#include "app_state_stby.h"
#include "app_state_manu.h"
#include "app_state_auto.h"
#include "app_state_error.h"

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
    [STBY_HOLD]       = STBY,  // Hold and wait

    [MANU_JOYSTICK]   = MANU,

    [AUTO_HOME]       = AUTO,  // Go to 0, 0
    [AUTO_HOLD]       = AUTO,  // Hold but accept centroids
    [AUTO_ACQ_GPS]    = AUTO,  // Go to GPS vector pointing
    [AUTO_ACQ_SPIRAL] = AUTO,  // Spiral search for lock
    [AUTO_TRACKING]   = AUTO,  // Closed loop tracking
    [AUTO_LOSS]       = AUTO,  // Loss of centeroiding packets
    [AUTO_NO_LOCK]    = AUTO,  // No lock after ACQ attempts

    [ERROR_ACTIVE]    = ERROR,  // Error state. TODO: Severity levels.
    [ERROR_KILL]      = ERROR,  // Sends kill to gimbal.
};

// Hook table type forward declaration
typedef void    (*state_fn_t)(app_ctx_t *);         // Entry and Exit handlers
typedef uint8_t (*key_fn_t)  (app_ctx_t *, char);   // Input handlers
typedef void    (*build_fn_t)(const app_ctx_t *, hamfly_control_t *); //Packet
typedef uint8_t (*guard_fn_t)(const app_ctx_t *);   // 1=valid, 0=refuse

// %==========================================================================%
// %                          Dispatch tables                                 %
// %==========================================================================%
// All handler symbols are declared in their respective app_state_*.h headers.

// Table of entry guards.
static const guard_fn_t can_enter[STATE_COUNT] = {
    [MANU_JOYSTICK] = guard_manu_joystick,
    [AUTO]         = guard_auto_parent,   // need gimbal telemetry to enter
    [AUTO_HOME]    = guard_auto_home,     // needs origin set + live telemetry
    [AUTO_ACQ_GPS] = guard_auto_acq_gps,  // needs a GPS target
};

// Table of on-entry functions.
static const state_fn_t on_entry[STATE_COUNT] = {
    [STBY]            = NULL,   // parent: shared STBY entry, if ever needed
    [MANU]            = NULL,   // parent
    [AUTO]            = NULL,   // parent
    [ERROR]           = NULL,   // parent
    [STBY_DEFER]      = entry_stby_defer,
    [STBY_HOLD]       = entry_stby_hold,
    [MANU_JOYSTICK]   = entry_manu_joystick,
    [AUTO_HOME]       = entry_auto_home,
    [AUTO_HOLD]       = entry_auto_hold,
    [AUTO_ACQ_GPS]    = entry_auto_acq_gps,
    [AUTO_ACQ_SPIRAL] = entry_auto_acq_spiral,
    [AUTO_TRACKING]   = entry_auto_tracking,
    [AUTO_LOSS]       = entry_auto_loss,
    [AUTO_NO_LOCK]    = entry_auto_no_lock,
    [ERROR_ACTIVE]    = entry_error_active,
    [ERROR_KILL]      = entry_error_kill,
};

static const state_fn_t on_exit[STATE_COUNT] = {
    [STBY]            = NULL,   // parent
    [MANU]            = NULL,   // parent
    [AUTO]            = NULL,   // parent
    [ERROR]           = NULL,   // parent
    [STBY_DEFER]      = NULL,
    [STBY_HOLD]       = NULL,
    [MANU_JOYSTICK]   = exit_manu_joystick,
    [AUTO_HOME]       = exit_auto_home,
    [AUTO_HOLD]       = NULL,
    [AUTO_ACQ_GPS]    = exit_auto_acq_gps,
    [AUTO_ACQ_SPIRAL] = NULL,   // TODO
    [AUTO_TRACKING]   = NULL,   // TODO
    [AUTO_LOSS]       = NULL,   // TODO
    [AUTO_NO_LOCK]    = NULL,   // TODO
    [ERROR_ACTIVE]    = NULL,   // TODO
    [ERROR_KILL]      = NULL,
};

// For key handling over serial
// Will be superceded by auto flow conditions and SBC commands.
static const key_fn_t on_key[STATE_COUNT] = {
    [STBY_DEFER]      = NULL,  // TODO
    [STBY_HOLD]       = key_stby_hold ,  // TODO
    [MANU_JOYSTICK]   = key_manu_joystick,
    [AUTO_HOME]       = key_auto_home,
    [AUTO_HOLD]       = key_auto_hold,
    [AUTO_ACQ_GPS]    = key_auto_acq_gps,
    [AUTO_ACQ_SPIRAL] = NULL,  // TODO
    [AUTO_TRACKING]   = key_auto_tracking,
    [AUTO_LOSS]       = key_auto_loss,
    [AUTO_NO_LOCK]    = key_auto_no_lock,
    [ERROR_ACTIVE]    = key_error_active,
    [ERROR_KILL]      = key_error_kill,
};

static const build_fn_t on_build[STATE_COUNT] = {
    [STBY_DEFER]      = build_defer,
    [STBY_HOLD]       = build_hold,
    [MANU_JOYSTICK]   = build_manu_joystick,  // NOTE: Nudge folded into joysticl leaf.
    [AUTO_HOME]       = build_auto_home,
    [AUTO_HOLD]       = build_hold,
    [AUTO_ACQ_GPS]    = build_auto_acq_gps,
    [AUTO_ACQ_SPIRAL] = build_hold,          // placeholder: zero rates, hold
    [AUTO_TRACKING]   = build_auto_tracking,
    [AUTO_LOSS]       = build_hold,          // placeholder: hold position
    [AUTO_NO_LOCK]    = build_hold,          // placeholder: hold position
    [ERROR_ACTIVE]    = build_error,
    [ERROR_KILL]      = build_error,
};

// %==========================================================================%
// %                              LCA walk                                    %
// %==========================================================================%
// Lowest Common Ancestor (LCA) allows for the relevant exit and entry action
// cascate occurs as needed to get between heirarchecal leafs. For a given 
// state, walk up its parents to ROOT, then walk down to incoming state.
// Allows for heirarchical and shared entry/exit behaviour.

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
    // Check auto state flow pin
    ctx->auto_flow = (Pin_SwitchAuto_Read() == 1u) ? 1u : 0u;
    UART_DEBUG_PutString(ctx->auto_flow ? "[DBG] auto_flow=1\r\n" : "[DBG] auto_flow=0\r\n");
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

// %==========================================================================%
// %                            State names                                   %
// %==========================================================================%
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


// %==========================================================================%
// %                              Telemetry                                   %
// %==========================================================================%
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
        telemetry_send_link_sbc(ctx);  // These each only excecute if scheduled
        telemetry_send_power_sbc(ctx);
        telemetry_send_env_sbc(ctx);
        telemetry_send_gps_sbc(ctx);
        telemetry_send_baro_sbc(ctx);
        telemetry_send_mag_sbc(ctx);
    }
}

// %==========================================================================%
// %                        Gimbal control packets                            %
// %==========================================================================%
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

// Do build. For states without a build function returns zeroed packet.
void app_build_control(const app_ctx_t *ctx, hamfly_control_t *out)
{
    // Nudge overlay: Allow user triggered nudges to supress state building
    if (ctx->nudge_hold) {
        build_manu_joystick(ctx, out);  // Borrow from MANU
        // TODO: Ensure we return to the correct state.
        // TODO: Move to utils? Or a less biased name.
        return;
    }
    
    // No nudge, build from state
    build_fn_t f = on_build[ctx->state];
    if (f)  f(ctx, out);
    else    build_defer(ctx, out);  // Safe default.
}


// %==========================================================================%
// %                        Serial Key dispatch                               %
// %==========================================================================%
#define KEY_CTRL_R 0x12  // Reset ASCII code.


// Capture current attitude and save as origin
// zero_tilt is a bool which forces the tilt origin to be level
uint8_t set_origin(app_ctx_t *ctx, uint8_t zero_tilt)
{
    if (!ctx->gimbal) {
        app_raise_error(ctx, SEV_USER, "no gimbal handle");
        return 0u;
    }
    float pan, tilt;
    if (!gimbal_pan_tilt_deg(ctx, &pan, &tilt)) {
        app_raise_error(ctx, SEV_USER, "no telemetry (origin not set)");
        return 0u;
    }
    ctx->origin_pan_deg  = pan;
    ctx->origin_tilt_deg = zero_tilt ? 0.0f : tilt;
    ctx->origin_set = 1u;
    UART_DEBUG_PutString("\r\n[ORIGIN] captured.\r\n");
    return 1u;
}

// Serial key menu, for debugging but superceded soon by SBC comms.
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

// Handle global keys in any state.
static uint8_t handle_global_key(app_ctx_t *ctx, char k)
{
    switch (k) {
        case KEY_CTRL_R:  // Clear FATAL error
            if (ctx->fatal_latched) {
                ctx->fatal_latched = 0u;
                ctx->err_sev = SEV_USER;
                ctx->err_msg = NULL;
                UART_DEBUG_PutString("\r\nFATAL cleared.\r\n");
                app_transition(ctx, STBY_HOLD);
            } else {
                app_transition(ctx, STBY_DEFER);
            }
            return 1;
        case 'x':  // Return to HOLD as a safe state
            if (ctx->gimbal) hamfly_kill(ctx->gimbal);
            app_transition(ctx, STBY_HOLD);
            return 1;
        case '[':
            (void)set_origin(ctx, 0u);
            return 1;
        case '?':
            print_help(ctx); 
            return 1;
        default:  return 0;
    }
}

static uint8_t leaf_traps_nav(state_t s)
{
    // Tee following states need to be exited (e) before numerals change state.
    return (s == MANU_JOYSTICK   ||
            s == AUTO_HOME       ||
            s == AUTO_ACQ_GPS    ||
            s == AUTO_ACQ_SPIRAL ||
            s == AUTO_TRACKING);
}
static uint8_t handle_nav_key(app_ctx_t *ctx, char k)
{
    // TODO: Add remaining states
    switch (k) {
        case '1': app_transition(ctx, STBY_HOLD);     return 1;
        case '2': app_transition(ctx, MANU_JOYSTICK); return 1;
        case '3': app_transition(ctx, AUTO_TRACKING); return 1;
        default:  return 0;
    }
}


// % Public key handling entry point %----------------------------------------%
uint8_t app_dispatch_key(app_ctx_t *ctx, char key)
{
    if (ctx->state >= STATE_COUNT) return 0;  // Guard on real states

    if (ctx->fatal_latched)  // Lout of if in FATAL error.
        return (key == KEY_CTRL_R) ? handle_global_key(ctx, key) : 0;

    if (handle_global_key(ctx, key)) return 1;  // Handle global keys first

    // Handle per leaf key
    key_fn_t f = on_key[ctx->state];
    if (f && f(ctx, key)) return 1;
    
    // Handle state change numerals unless locked
    if (!leaf_traps_nav(ctx->state))
        return handle_nav_key(ctx, key);

    return 0;
}


// %==========================================================================%
// %                        SBC Commands and Control                          %
// %==========================================================================%
// Helper - Validate state change request
static uint8_t sbc_state_is_requestable(uint8_t s)
{
    // Not implemented. Should be a state-to-state look up table I think.
    switch ((state_t)s) {
        case STBY_HOLD:
        case MANU_JOYSTICK:
        case AUTO_HOME:
        case AUTO_HOLD:
        case AUTO_ACQ_GPS:
        case AUTO_ACQ_SPIRAL:
        case AUTO_TRACKING:
            return 1u;
        case ERROR_KILL:  // Serial only for now
        default:
            return 0u;
    }
}

// gate which states are allowed to receive manual nudges from SBC
static uint8_t nudge_allowed_in(state_t s)
{
    switch (s) {
        case STBY_HOLD:
        case MANU_JOYSTICK:
        case AUTO_HOLD:
        case AUTO_LOSS:
        case AUTO_NO_LOCK:
            return 1u;
        default:
            return 0u;
    }
}

// Lookup parameter by id.
// Return 1 = found and got it.
// Return 0 = unknown id.
uint8_t app_param_get(app_ctx_t *ctx, uint8_t id, float *out_value)
{
    switch (id) {
        case PARAM_TRACK_KP:
            *out_value = ctx->track_kp;
            return 1u;
        case PARAM_TRACK_KI:
            *out_value = ctx->track_ki;
            return 1u;
        case PARAM_TRACK_KD:
            *out_value = ctx->track_kd; 
            return 1u;
        case PARAM_TRACK_RATE_MAX:
            *out_value = ctx->track_rate_max;
            return 1u;
        default:
            return 0u;
    }
}

void app_sbc_tick(app_ctx_t *ctx)
{
    // TODO: Is this the right flow? The ctx should have a target coord,
    // when the ISR for the SBC comms triggers it should load a new target 
    // into the ctx.

    // 1 -> Check for GPS target
    payload_gps_target_t tgt;
    if (sbc_get_gps_target(&tgt))
    {
        ctx->gps_target_lat_raw = tgt.lat_raw;
        ctx->gps_target_lon_raw = tgt.lon_raw;
        ctx->gps_target_alt_mm  = tgt.alt_mm;
        ctx->gps_target_set     = 1u;
        ctx->gps_new_target     = 1u;   // ACQ_GPS picks this up on next settle
        sbc_send_cmd_ack(PKT_GPS_TARGET, CMD_ACK_OK);
    }

    // 2 -> Check for nudge (clamped to soft limits of the origin inside nudge_apply)
    payload_nudge_t nz;
    if (sbc_get_nudge(&nz)) {
        if (nudge_allowed_in(ctx->state)) {
            nudge_apply(ctx,
                        (float)nz.dpan_cdeg  / 100.0f,
                        (float)nz.dtilt_cdeg / 100.0f);
            sbc_send_cmd_ack(PKT_NUDGE, CMD_ACK_OK);
        } else {
            UART_DEBUG_PutString("[SBC] nudge ignored (not in MANU)\r\n");
            sbc_send_cmd_ack(PKT_NUDGE, CMD_ACK_REJECTED);
        }
    }

    // 3 -> Parameter tuning
    payload_param_t pp;
    if (sbc_get_param(&pp)) {
        uint8_t result = CMD_ACK_OK;
        switch (pp.id) {
            case PARAM_TRACK_KP:
                ctx->track_kp = pp.value;
                {
                    char b[64];
                    snprintf(b, sizeof b, "[SBC] track_kp = %.6f\r\n",
                             (double)ctx->track_kp);
                    UART_DEBUG_PutString(b);
                }
                break;
            case PARAM_TRACK_KI:
                ctx->track_ki = pp.value;
                // Reset integrator on Ki change so old accumulator doesn't bleed
                // through the new gain.
                ctx->track_i_pan  = 0.0f;
                ctx->track_i_tilt = 0.0f;
                {
                    char b[64];
                    snprintf(b, sizeof b, "[SBC] track_ki = %.6f\r\n",
                             (double)ctx->track_ki);
                    UART_DEBUG_PutString(b);
                }
                break;
            case PARAM_TRACK_KD:
                ctx->track_kd = pp.value;
                {
                    char b[64];
                    snprintf(b, sizeof b, "[SBC] track_kd = %.6f\r\n",
                             (double)ctx->track_kd);
                    UART_DEBUG_PutString(b);
                }
                break;
            case PARAM_TRACK_RATE_MAX:
                ctx->track_rate_max = CLAMP(pp.value, 0.0f, 1.0f);
                {
                    char b[64];
                    snprintf(b, sizeof b, "[SBC] track_rate_max = %.4f\r\n",
                             (double)ctx->track_rate_max);
                    UART_DEBUG_PutString(b);
                }
                break;
            default:
                {
                    char b[64];
                    snprintf(b, sizeof b, "[SBC] unknown param id=%u\r\n",
                             (unsigned)pp.id);
                    UART_DEBUG_PutString(b);
                }
                result = CMD_ACK_INVALID;
                break;
        }
        sbc_send_cmd_ack(PKT_PARAM_SET, result);
    }

    // 4 -> Capture software origin from current telemetry
    if (sbc_get_set_origin()) {
        uint8_t ok = set_origin(ctx, 0u);  // same path as the '[' key
        sbc_send_cmd_ack(PKT_SET_ORIGIN, ok 
                         ? CMD_ACK_OK 
                         : CMD_ACK_NO_TELEM);
    }
    
    // 5 -> Clear FATAL latch
    if (sbc_get_fatal_clear()) {
        // Mirrors the Ctrl+R path in app_dispatch_key.
        if (ctx->fatal_latched) {
            ctx->fatal_latched = 0u;
            ctx->err_sev = SEV_USER;
            ctx->err_msg = NULL;
            UART_DEBUG_PutString("\r\n[SBC] FATAL cleared.\r\n");
            app_transition(ctx, STBY_HOLD);
        }
        sbc_send_cmd_ack(PKT_FATAL_CLEAR, CMD_ACK_OK);
    }

    // 6 -> Parameter GET
    payload_param_get_t pg;
    if (sbc_get_param_read(&pg)) {  // Load parameter request into struct
        float v = 0.0f;
        if (app_param_get(ctx, pg.id, &v)) {  // Try to get and send param back
            sbc_send_param_value(pg.id, v);
        } else {
            char b[64];  // Failed to GET
            snprintf(b, sizeof b, "[SBC] GET unknown param id=%u\r\n",
                     (unsigned)pg.id);
            UART_DEBUG_PutString(b);
            sbc_send_cmd_ack(PKT_PARAM_GET, CMD_ACK_INVALID);
        }
    }
    
    // 7 -> State change request
    payload_state_req_t req;
    if (!sbc_get_state_req(&req)) return;
    
    // Lock out if in FATAL error until ctrl-R.
    // TODO: SBC needs a way to get out of error state too.
    if (ctx->fatal_latched)
    {
        sbc_send_state_ack((uint8_t)ctx->state, STATE_ACK_REJECTED);
        return;
    }
    // Validate state change request.
    if (!sbc_state_is_requestable(req.requested_state))
    {
        sbc_send_state_ack((uint8_t)ctx->state, STATE_ACK_INVALID);
        char b[64];
        snprintf(b, sizeof b, "[SBC] invalid state req=%u\r\n",
                 (unsigned)req.requested_state);
        UART_DEBUG_PutString(b);
        return;
    }

    state_t target = (state_t)req.requested_state;
    // Debug serial monitor print.
    char b[64];
    snprintf(b, sizeof b, "[SBC] state req -> %s\r\n", app_state_name(target));
    UART_DEBUG_PutString(b);
    // Trigger state change
    app_transition(ctx, target);
    // Acknowledge on success.
    sbc_send_state_ack((uint8_t)ctx->state, STATE_ACK_OK);
}
/* [] END OF FILE */