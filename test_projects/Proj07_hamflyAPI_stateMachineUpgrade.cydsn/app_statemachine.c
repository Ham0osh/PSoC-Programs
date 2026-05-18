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
*/

#include "app_statemachine.h"
#include "app_clock.h"  // TODO: Placehlder for PSoC timer.
#include <project.h>  // For UART

// Hierarchy %================================================================%
// Heirarchy allows shared entry/exit behaviour.
// ROOT is the base of the walk.
static const state_t parent_of[STATE_COUNT] = {
    [ROOT]            = ROOT,  // Book keeping
    [STBY]            = ROOT,  // Parent states
    [MANU]            = ROOT,
    [AUTO]            = ROOT,
    [ERROR]           = ROOT,

    [STBY_DEFER]      = STBY,
    [STBY_HOLD]       = STBY,

    [MANU_JOYSTICK]   = MANU,
    [MANU_NUDGE]      = MANU,

    [AUTO_HOME]       = AUTO,
    [AUTO_ACQ_GPS]    = AUTO,
    [AUTO_ACQ_SPIRAL] = AUTO,
    [AUTO_TRACKING]   = AUTO,
    [AUTO_LOSS]       = AUTO,
    [AUTO_NO_LOCK]    = AUTO,

    [ERROR_ACTIVE]    = ERROR,
};

// Per-state hook tables %====================================================%
// on_entry and on_exit are independent; a state may have none, one, or both.
typedef void (*state_fn_t)(app_ctx_t *);

static void entry_stby_defer   (app_ctx_t *);
static void entry_stby_hold    (app_ctx_t *);
static void entry_manu_joystick(app_ctx_t *);
static void exit_manu_joystick (app_ctx_t *);
static void entry_error_active (app_ctx_t *);
// AUTO_* and MANU_NUDGE: NULL until implemented

static const state_fn_t on_entry[STATE_COUNT] = {
    [STBY_DEFER]    = entry_stby_defer,
    [STBY_HOLD]     = entry_stby_hold,
    [MANU_JOYSTICK] = entry_manu_joystick,
    [ERROR_ACTIVE]  = entry_error_active,
};
static const state_fn_t on_exit[STATE_COUNT] = {
    [MANU_JOYSTICK] = exit_manu_joystick,
};

// Lowest common ancestor walk %==============================================%
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
    ctx->state_entry_ms = app_millis();  // TODO: Wire to PSoC timer.
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
        UART_PutString("[USER] ");
        UART_PutString(msg);
        UART_PutString("\r\n");
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
    UART_PutString("\r\n[STBY_DEFER] boot — '1' hold, '2' manual, '3' auto, '?' help\r\n> ");
}
// On Exit: None


// STBY_HOLD %================================================================%
// On Enter
static void entry_stby_hold(app_ctx_t *ctx)
{
    (void)ctx;
    UART_PutString("\r\n[STBY_HOLD] rates zeroed\r\n> ");
}
// On Exit: None


// %==========================================================================%
// %                                Manual                                    %
// %==========================================================================%

// MANU_JOYSTICK %============================================================%
// On Enter: Enable joystick, print message.
static void entry_manu_joystick(app_ctx_t *ctx)
{
    joystick_enable(&ctx->cmd);                  /* TODO confirm API name */
    UART_PutString("\r\n[MANU_JOYSTICK] r=rate a=abs [=origin ]=home\r\n> ");
}
// On Exit: Disable joystick, zero control mode.
static void exit_manu_joystick(app_ctx_t *ctx)
{
    joystick_disable();                          /* TODO confirm API name */
    ctx->ctrl_mode = HAMFLY_DEFER;
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
    UART_PutString("\r\n[ERROR:");
    UART_PutString(sev_str(ctx->err_sev));
    UART_PutString("] ");
    UART_PutString(ctx->err_msg ? ctx->err_msg : "(no msg)");
    if (ctx->err_sev == SEV_FATAL)
        UART_PutString("\r\nFATAL latched. Power cycle or Ctrl+R to clear.\r\n");
    else
        UART_PutString("\r\nAny key to acknowledge.\r\n");
}

// %==========================================================================%
// %                             Key Handling                                 %
// %==========================================================================%
#define KEY_CTRL_R 0x12  // Reset ASCII code.

// Handles global key commands over serial monitor.
// Returns 1 if consumed, 0 for leaf-specific handlers.
static uint8_t handle_global_key(app_ctx_t *ctx, char k)
{
    if (k == KEY_CTRL_R) {  // Reset fatal state.
        if (ctx->fatal_latched) {
            ctx->fatal_latched = 0u;
            ctx->err_sev       = SEV_USER;
            ctx->err_msg       = NULL;
            UART_PutString("\r\nFATAL cleared.\r\n");
            transition(ctx, STBY_HOLD);
        }
        return 1;
    }
    if (ctx->fatal_latched) return 0;
    
    // State switch statement for global keys.
    switch (k) {
        case '1': transition(ctx, STBY_HOLD);     return 1;
        case '2': transition(ctx, MANU_JOYSTICK); return 1;
        case '3': transition(ctx, AUTO_HOME);     return 1;
        case '?': /* print_help(); */             return 1;  // TODO.
        default:  return 0;
    }
}

typedef uint8_t (*key_fn_t)(app_ctx_t *, char);

// Manual controls during joystick mode.
static uint8_t key_manu_joystick(app_ctx_t *ctx, char k)
{
    switch (k) {
        case 'r': ctx->ctrl_mode = HAMFLY_RATE;     return 1;
        case 'a': ctx->ctrl_mode = HAMFLY_ABSOLUTE; return 1;
        case '[': /* TODO read encoders into ctx->origin_pan_deg/tilt_deg */
                  ctx->origin_set = 1u;             return 1;
        case ']': transition(ctx, AUTO_HOME);       return 1;
        default:  return 0;
    }
}

// Acknowledges error and returns to STBY_HOLD.
// Does nothing if FATAL.
static uint8_t key_error_active(app_ctx_t *ctx, char k)
{
    (void)k;
    if (ctx->err_sev != SEV_FATAL)
    {
        // Clear error and return to HOLD for safety.
        transition(ctx, STBY_HOLD);
        return 1;
    }
    return 0;  // Do nothing on ack if FATAL.
}

static const key_fn_t on_key[STATE_COUNT] = {
    [MANU_JOYSTICK] = key_manu_joystick,
    [ERROR_ACTIVE]  = key_error_active,
};

uint8_t app_dispatch_key(app_ctx_t *ctx, char k)
{
    if (handle_global_key(ctx, k)) return 1;
    key_fn_t f = on_key[ctx->state];
    return f ? f(ctx, k) : 0;
}

// %==========================================================================%
// %                       Gimbal Control Packet                              %
// %==========================================================================%
// TODO: Compare with Proj06 and HamflyAPI to refactor and unify cleanly.
typedef void (*build_fn_t)(const app_ctx_t *, hamfly_control_t *);

// Initialize empty packet.
static void build_zero(const app_ctx_t *ctx, hamfly_control_t *out)
{
    (void)ctx;
    memset(out, 0, sizeof *out);
}
// Build packet for STBY states: zero rates, defer mode.
static void build_stby(const app_ctx_t *ctx, hamfly_control_t *out)
{
    build_zero(ctx, out);
    out->mode = HAMFLY_DEFER;                    /* TODO confirm field */
}
static void build_manu_joystick(const app_ctx_t *ctx, hamfly_control_t *out)
{
    out->mode = ctx->ctrl_mode;
    if (ctx->ctrl_mode == HAMFLY_RATE) {
        out->pan_rate  = ctx->cmd.pan;           /* TODO confirm field names */
        out->tilt_rate = ctx->cmd.tilt;
    } else if (ctx->ctrl_mode == HAMFLY_ABSOLUTE) {
        out->pan_abs   = ctx->cmd.pan;
        out->tilt_abs  = ctx->cmd.tilt;
    }
    out->kill = 0u;
}
static void build_error(const app_ctx_t *ctx, hamfly_control_t *out)
{
    build_zero(ctx, out);
    if (ctx->err_sev == SEV_FATAL) out->kill = 1u;
}

static const build_fn_t on_build[STATE_COUNT] = {
    [STBY_DEFER]    = build_stby,
    [STBY_HOLD]     = build_stby,
    [MANU_JOYSTICK] = build_manu_joystick,
    [ERROR_ACTIVE]  = build_error,
};

void app_build_control(const app_ctx_t *ctx, hamfly_control_t *out)
{
    build_fn_t f = on_build[ctx->state];
    if (f) f(ctx, out);
    else   build_zero(ctx, out);                 /* safe default for AUTO_* */
}
