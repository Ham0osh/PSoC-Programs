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

