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
 *   ├─ STBY ── STBY_DEFER (boot)   STBY_HOLD
 *   ├─ MANU ── MANU_JOYSTICK       MANU_NUDGE*
 *   ├─ AUTO ── AUTO_HOME*  ACQ_GPS*  ACQ_SPIRAL*
 *   │          TRACKING*   LOSS*     NO_LOCK*
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
#include "pi_comms.h"
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

    [MANU_JOYSTICK]   = MANU,  // TODO: Fold into one manual state with both active.
    [MANU_NUDGE]      = MANU,  //       Currently nudge works during joystick.

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
typedef void    (*state_fn_t)(app_ctx_t *);  // Entry and Exit handlers
typedef uint8_t (*key_fn_t)  (app_ctx_t *, char);  // Input handlers
typedef void    (*build_fn_t)(const app_ctx_t *, hamfly_control_t *); //Packet

// Entry handlers
static void entry_stby_defer   (app_ctx_t *);
static void entry_stby_hold    (app_ctx_t *);
static void entry_manu_joystick(app_ctx_t *);
static void entry_error_active (app_ctx_t *);

static void entry_auto_test(app_ctx_t *ctx);

// Exit handlers
static void exit_manu_joystick (app_ctx_t *);

// Key handlers
static uint8_t key_manu_joystick(app_ctx_t *, char);
static uint8_t key_error_active (app_ctx_t *, char);

// Packet building handlers
static void build_zero         (const app_ctx_t *, hamfly_control_t *);
static void build_stby         (const app_ctx_t *, hamfly_control_t *);
static void build_manu_joystick(const app_ctx_t *, hamfly_control_t *);
static void build_error        (const app_ctx_t *, hamfly_control_t *);
// AUTO_* and MANU_NUDGE: NULL in the tables until implemented

static const state_fn_t on_entry[STATE_COUNT] = {
    [STBY]            = NULL,   /* parent: shared STBY entry, if ever needed */
    [MANU]            = NULL,   /* parent */
    [AUTO]            = NULL,   /* parent */
    [ERROR]           = NULL,   /* parent */
    [STBY_DEFER]      = entry_stby_defer,
    [STBY_HOLD]       = entry_stby_hold,
    [MANU_JOYSTICK]   = entry_manu_joystick,
    [MANU_NUDGE]      = NULL,   /* TODO */
    [AUTO_HOME]       = NULL,   /* TODO */
    [AUTO_ACQ_GPS]    = NULL,   /* TODO */
    [AUTO_ACQ_SPIRAL] = NULL,   /* TODO */
    [AUTO_TRACKING]   = entry_auto_test,  // Temp for testing
    [AUTO_LOSS]       = NULL,   /* TODO */
    [AUTO_NO_LOCK]    = NULL,   /* TODO */
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
    [MANU_NUDGE]      = NULL,   /* TODO */
    [AUTO_HOME]       = NULL,   /* TODO */
    [AUTO_ACQ_GPS]    = NULL,   /* TODO */
    [AUTO_ACQ_SPIRAL] = NULL,   /* TODO */
    [AUTO_TRACKING]   = NULL ,  
    [AUTO_LOSS]       = NULL,   /* TODO */
    [AUTO_NO_LOCK]    = NULL,   /* TODO */
    [ERROR_ACTIVE]    = NULL,
};

static const key_fn_t on_key[STATE_COUNT] = {
    [STBY_DEFER]      = NULL,
    [STBY_HOLD]       = NULL,
    [MANU_JOYSTICK]   = key_manu_joystick,
    [MANU_NUDGE]      = NULL,   /* TODO */
    [AUTO_HOME]       = NULL,   /* TODO */
    [AUTO_ACQ_GPS]    = NULL,   /* TODO */
    [AUTO_ACQ_SPIRAL] = NULL,   /* TODO */
    [AUTO_TRACKING]   = NULL,   /* TODO */
    [AUTO_LOSS]       = NULL,   /* TODO */
    [AUTO_NO_LOCK]    = NULL,   /* TODO */
    [ERROR_ACTIVE]    = key_error_active,
};

static const build_fn_t on_build[STATE_COUNT] = {
    [STBY_DEFER]      = build_stby,
    [STBY_HOLD]       = build_stby,
    [MANU_JOYSTICK]   = build_manu_joystick,
    [MANU_NUDGE]      = NULL,   /* TODO */
    [AUTO_HOME]       = NULL,   /* TODO */
    [AUTO_ACQ_GPS]    = NULL,   /* TODO */
    [AUTO_ACQ_SPIRAL] = NULL,   /* TODO */
    [AUTO_TRACKING]   = build_zero,  // Temp for testing.
    [AUTO_LOSS]       = NULL,   /* TODO */
    [AUTO_NO_LOCK]    = NULL,   /* TODO */
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

// Transition workhorse %=====================================================%
//
static void transition(app_ctx_t *ctx, state_t target)
{
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
    transition(ctx, target);
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
    
    transition(ctx, ERROR_ACTIVE);  // Go to error state.
}

// Lookup table for state names, used by serial monitor.
static const char *const state_names[STATE_COUNT] = {
    [ROOT] = "ROOT",
    [STBY] = "STBY", [MANU] = "MANU", [AUTO] = "AUTO", [ERROR] = "ERROR",
    [STBY_DEFER]      = "STBY_DEFER",
    [STBY_HOLD]       = "STBY_HOLD",
    [MANU_JOYSTICK]   = "MANU_JOYSTICK",
    [MANU_NUDGE]      = "MANU_NUDGE",
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
    // Rate default
    ctx->ctrl_mode = HAMFLY_RATE;
     UART_DEBUG_PutString("\r\n[MANU] joystick=rate  nudge: numpad=fine  ijkl=0.5  wasd=1  e=exit\r\n> ");
}
// On Exit: Disable joystick, zero control mode.
static void exit_manu_joystick(app_ctx_t *ctx)
{
    ctx->ctrl_mode = HAMFLY_DEFER;
}

// %==========================================================================%
// %                                 Auto                                     %
// %==========================================================================%
// TEMP: for initial inter-device comms testing
static void entry_auto_test(app_ctx_t *ctx)
{
    (void)ctx;
    UART_DEBUG_PutString("\r\n[AUTO] link loopback test: TX centroid @1Hz, echo on RX\r\n");
}

void app_auto_tick(app_ctx_t *ctx)
{
    if (ctx->state != AUTO_TRACKING) return;

    pi_centroid_t c;
    if (pi_get_centroid(&c)) {
        char b[96];
        snprintf(b, sizeof b, "[PI] cx=%d cy=%d ex=%u ey=%u ts=%lu crcErr=%u uartErr=%u\r\n",
                 c.cx, c.cy, c.ex, c.ey,
                 (unsigned long)(c.pi_time_us & 0xFFFFFFFFUL),
                 pi_crc_errors(), pi_uart_errors());
        UART_DEBUG_PutString(b);
    }

    // TEMP: build a fake centroid here and loop it back via the public framer
    static uint32_t last = 0u;
    if (g_tick_ms - last >= 1000u) {
        last = g_tick_ms;
        uint8_t pl[16];
        int16_t  cx = 1234, cy = -567;
        uint16_t ex = 10,   ey = 20;
        uint64_t ts = 0x0102030405060708ull;
        memcpy(pl + 0, &cx, 2); memcpy(pl + 2, &cy, 2);
        memcpy(pl + 4, &ex, 2); memcpy(pl + 6, &ey, 2);
        memcpy(pl + 8, &ts, 8);
        pi_send_frame(PI_MAGIC_CENTROID, pl, sizeof pl);
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

static uint8_t manu_pan_tilt_deg(const app_ctx_t *ctx, float *pan_deg, float *tilt_deg)
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
    if (!manu_pan_tilt_deg(ctx, &pan, &tilt)) {
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
                transition(ctx, STBY_HOLD);
            }
            return 1;                          /* reserved key, always consumed */
        case 'x':                              /* soft kill */
            if (ctx->gimbal) hamfly_kill(ctx->gimbal);
            transition(ctx, STBY_HOLD);
            return 1;
        case '[': set_origin(ctx);  return 1;
            case '?': print_help(ctx);  return 1;
            default:  return 0;
    }
}

/* ---- navigation keys (suppressed in trapping leaves) ------------------ */
static uint8_t leaf_traps_nav(state_t s) { return (s == MANU_JOYSTICK); }

static uint8_t handle_nav_key(app_ctx_t *ctx, char k)
{
    switch (k) {
        case '1': transition(ctx, STBY_HOLD);     return 1;
        case '2': transition(ctx, MANU_JOYSTICK); return 1;
        case '3': transition(ctx, AUTO_TRACKING); return 1;
        default:  return 0;
    }
}

/* ---- leaf-specific keys ----------------------------------------------- */
static uint8_t key_error_active(app_ctx_t *ctx, char k)
{
    (void)k;
    if (ctx->err_sev != SEV_FATAL) {            /* WARN/SOFTWARE ack */
        state_t back = (ctx->prev_leaf == ERROR_ACTIVE) ? STBY_HOLD : ctx->prev_leaf;
        transition(ctx, back);
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
// Initialize empty packet.
static void build_zero(const app_ctx_t *ctx, hamfly_control_t *out)
{
    (void)ctx;
    memset(out, 0, sizeof *out);
}
// Build packet for STBY states: zero rates, defer mode.
static void build_stby(const app_ctx_t *ctx, hamfly_control_t *out) {
    build_zero(ctx, out);  // DEFERed, and disabled.
}

// Build packet from joystick inputs.
static void build_manu_joystick(const app_ctx_t *ctx, hamfly_control_t *out) {
    out->enable    = 1u;
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
    build_zero(ctx, out);
    if (ctx->err_sev == SEV_FATAL) out->kill = 1u;
}


// Do build. For states without a build function returns zeroed packet.
void app_build_control(const app_ctx_t *ctx, hamfly_control_t *out)
{
    build_fn_t f = on_build[ctx->state];
    if (f)  f(ctx, out);
    else    build_zero(ctx, out);  // Safe default.
}

// TEMP: Nudge constants
#define NUDGE_LSB_DEG     (180.0f/32767.0f)  /* ~0.00549°, one on-wire LSB */
#define NUDGE_FINE_DEG    0.5f
#define NUDGE_COARSE_DEG  1.0f
#define NUDGE_SETTLE_DEG  0.05f   /* "arrived" tolerance */
#define NUDGE_MIN_DWELL_MS 150u   /* hold long enough for the step to land */
#define NUDGE_TIMEOUT_MS  2000u   /* give up if it never settles */
// TEMP: Nudge handler
static void nudge_apply(app_ctx_t *ctx, float dpan_deg, float dtilt_deg)
{
    if (!ctx->nudge_hold) {// Capture nudge baseline
        float p, t;
        if (!manu_pan_tilt_deg(ctx, &p, &t)) {
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
        case 'e': transition(ctx, STBY_HOLD);                return 1;  // Exit
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
    if (manu_pan_tilt_deg(ctx, &p, &t))
        arrived = (fabsf(p - ctx->tgt_pan_deg)  < NUDGE_SETTLE_DEG) &&
                  (fabsf(t - ctx->tgt_tilt_deg) < NUDGE_SETTLE_DEG);
    if (arrived || elapsed >= NUDGE_TIMEOUT_MS)
        ctx->nudge_hold = 0u;                        /* rate=0 holds the new attitude */
}