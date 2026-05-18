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
 * HFSM public contract. main.c only sees these five calls plus the
 * types from app_ctx.h. Everything else (parent_of[], LCA walk,
 * per-leaf entry/exit/key/build tables) is private to the .c file.
*/

#ifndef APP_STATEMACHINE_H
#define APP_STATEMACHINE_H

#include "app_ctx.h"
#include "hamfly_core_control.h"   /* hamfly_control_t */

#define MAX_HFSM_DEPTH 4u  // For LCA walk, we have 2 levels + ROOT.

void        app_start(app_ctx_t *ctx);
uint8_t     app_dispatch_key(app_ctx_t *ctx, char key);
void        app_build_control(const app_ctx_t *ctx, hamfly_control_t *out);
void        app_raise_error(app_ctx_t *ctx, err_sev_t sev, const char *msg);
const char *app_state_name(state_t s);

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


#endif /* APP_STATEMACHINE_H */
/* [] END OF FILE */