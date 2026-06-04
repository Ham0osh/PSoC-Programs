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
 * Uses HamflyAPI (hamfly.h) for all gimbal communication.
 *
 * Modes:
 *   STANDBY   passive; pump runs; 1 Hz status if telemetry valid.
 *             Set boresight origin here before entering other modes.
 *   JOYSTICK  10 Hz QX277 TX.  Joystick (rate) and numpad presets
 *             share the mode — preset executes, settles to within
 *             PRESET_SETTLE_DEG of target (or PRESET_SETTLE_MS
 *             timeout), then joystick resumes automatically.
 *   NUDGE     keyboard-driven absolute accumulation for beam-profiler
 *             calibration.  Each keypress adds a fixed step to the
 *             accumulated pan/tilt target, clamped to software limits.
 *             Current accumulated position prints after every step.
 *             No joystick input in this mode.
 *
 *   t  exit to STANDBY     
 *   x  kill + exit
 */

#include <project.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

// Gimbal stuff
#include "hamfly.h"
#include "hamfly_platform_psoc5.h"

// Implementation specific stuff
#include "adc_balanced.h"
#include "joystick.h"
#include "debug_pins.h"

// PSoC state machine stuff
#include "psoc_eeprom.h"
#include "app_ctx.h"
#include "app_statemachine.h"

// Rasp Pi communications stuff
#include "sbc_comms.h"
#include "telemetry.h"

#define TX_BUF_SIZE           160u

// Soft limits relative to origin (degrees)
// Origin set by pressing '[' in any mode
// Nudge config
#define NUDGE_RATE              0.5f
#define NUDGE_TIME_SMALL        50u   // ms
#define NUDGE_TIME_LARGE        200u  // ms

// Utilities
#define FALSE  0u
#define TRUE   1u

// State machine states %=====================================================%

// Gimbal config and HAL
static hamfly_gimbal_t g_movi;
volatile uint32_t      g_tick_ms = 0u;

static uint32_t psoc_get_tick(void *ctx)
{
    (void)ctx;
    return g_tick_ms;
}
static const hamfly_hal_t MOVI_HAL = {
    .ctx         = NULL,
    .uart_putc   = _hamfly_psoc5_putc,   // exposed by the header
    .get_tick_ms = psoc_get_tick,
};

// ISRs %=====================================================================%
CY_ISR(isr_rx_movi_Handler)
{
    uint8_t st;
    do {
        st = UART_MOVI_RXSTATUS_REG;
        if (st & (UART_MOVI_RX_STS_BREAK     |
                  UART_MOVI_RX_STS_PAR_ERROR |
                  UART_MOVI_RX_STS_STOP_ERROR|
                  UART_MOVI_RX_STS_OVERRUN))
            hamfly_on_uart_err_flags(&g_movi, st);
        if (st & UART_MOVI_RX_STS_FIFO_NOTEMPTY)
            hamfly_on_rx_byte(&g_movi, UART_MOVI_RXDATA_REG);
    } while (st & UART_MOVI_RX_STS_FIFO_NOTEMPTY);
}

CY_ISR(isr_Looptimer_Handler)
{
    g_tick_ms++;
    Looptimer_ReadStatusRegister();
}

// For Raspberry Pi communications
CY_ISR(isr_rx_sbc_Handler)
{
    uint8_t st;
    do {
        st = UART_SBC_RXSTATUS_REG;
        if (st & (UART_SBC_RX_STS_BREAK     |
                  UART_SBC_RX_STS_PAR_ERROR |
                  UART_SBC_RX_STS_STOP_ERROR|
                  UART_SBC_RX_STS_OVERRUN))
            sbc_on_uart_err_flags(st);
        if (st & UART_SBC_RX_STS_FIFO_NOTEMPTY)
            sbc_on_rx_byte(UART_SBC_RXDATA_REG);
    } while (st & UART_SBC_RX_STS_FIFO_NOTEMPTY);
}

// Helper Functions  %========================================================%
static void decomp(float val, int16_t *i_out, int16_t *f_out, char *sign)
{
    *sign  = (val < 0.0f) ? '-' : ' ';
    if (val < 0.0f) val = -val;
    *i_out = (int16_t)val;
    *f_out = (int16_t)((val - (float)*i_out) * 100.0f);
    if (*f_out < 0) *f_out = -*f_out;
}

// Joystick calibration printer
static void cal_prompt(joy_cal_state_t prev, joy_cal_state_t now)
{
    if (prev == now) return;
    if (now == JOY_CAL_RANGE_CAPTURE)
        UART_DEBUG_PutString("\r\n[CAL] Move joystick full range then press 'c'\r\n");
    else if (now == JOY_CAL_CENTER_CAPTURE)
        UART_DEBUG_PutString("\r\n[CAL] Hold center...\r\n");
    else if (now == JOY_CAL_CONFIRM_SAVE)
        UART_DEBUG_PutString("\r\n[CAL] Press 'c' to save, 'x' to cancel\r\n");
    else if (now == JOY_CAL_OFF && prev == JOY_CAL_CONFIRM_SAVE) {
        if (joystick_save())
            UART_DEBUG_PutString("[CAL] Saved to EEPROM\r\n");
        else
        {
            UART_DEBUG_PutString("[CAL] EEPROM save FAILED\r\n");
            char dbuf[64];
            sprintf(dbuf, "[CAL] sizeof(joy_cal_t)=%u\r\n",
                    (unsigned)sizeof(joy_cal_t));
            UART_DEBUG_PutString(dbuf);
        }
    }
    else if (now == JOY_CAL_OFF && prev != JOY_CAL_OFF)
        UART_DEBUG_PutString("[CAL] Cancelled\r\n");
}

 
// Enum translator for Movi 
static const char *mode_str(hamfly_control_mode_t m)
{
    switch (m) {
        case HAMFLY_RATE:     return "RATE";
        case HAMFLY_ABSOLUTE: return "ABS ";
        default:              return "DEFR";
    }
}

static void print_hex_dump(const uint8_t *p, uint8_t plen)
{
    char buf[TX_BUF_SIZE];
    uint8_t row, col;
    for (row = 0u; row < plen; row += 8u) {
        char *bp = buf;
        bp += sprintf(bp, "  off%02u:", (unsigned)row);
        for (col = 0u; col < 8u && (row+col) < plen; col++)
            bp += sprintf(bp, " %02X", (unsigned)p[row+col]);
        bp += sprintf(bp, "\r\n");
        UART_DEBUG_PutString(buf);
    }
}

/* =========================================================================
 * main
 * ========================================================================= */
int main(void)
{
    char tx[TX_BUF_SIZE];  // Tx buffer for serial monitor
    
    // Initialize states
    hamfly_control_t      ctl;  // Control packet object
    app_ctx_t             ctx;  // Shared context for state machine
    // Default values
    app_ctx_init(&ctx);
    memset(&ctl, 0, sizeof ctl);
    
    // Default Joystick calibration as fallback to eeprom
    joy_cal_t defaults;
    for (uint8_t i = 0u; i < (uint8_t)N_CH; i++) {
        defaults.minv[i]   = 0;
        defaults.maxv[i]   = 255;
        defaults.center[i] = 128;
    }
    defaults.valid = 0u;


    // Initialize hardware %==================================================%
    CyGlobalIntEnable;  // PSoC API enable.
    
    // Start serial monitor comms.
    UART_DEBUG_Start();
    UART_DEBUG_PutString("\r\n=== Gimbal Control  PSoC 5LP ===\r\n");
    
    // Start Raspberry Pi comms.
    UART_SBC_Start();
    UART_SBC_ClearRxBuffer();
    UART_SBC_ClearTxBuffer();
    sbc_init();
    isr_rx_sbc_StartEx(isr_rx_sbc_Handler);
    UART_DEBUG_PutString("[Init] Pi comms ready\r\n");
    
    // Start Movi comms
    UART_MOVI_Start();
    UART_MOVI_ClearRxBuffer();
    UART_MOVI_ClearTxBuffer();
    
    // Initialize movi object and give to context
    hamfly_init(&g_movi, &MOVI_HAL);
    ctx.gimbal = &g_movi;
    isr_rx_movi_StartEx(isr_rx_movi_Handler);  // Rx handler
    UART_DEBUG_PutString("[Init] Gimbal UART ready\r\n");
    
    // Load joystick calibration from eeprom
    psoc_eeprom_init();
    if (!joystick_load(ctx.invert_mask)) {
        UART_DEBUG_PutString("[Init] No saved cal, using defaults.\r\n");
        joystick_init(&defaults, ctx.invert_mask);
        joystick_set_sensitivity(ctx.sense);
    } else {
        UART_DEBUG_PutString("[Init] Loaded cal from EEPROM.\r\n");
    }
    
    // Initialize joystick ADC
    adc_balanced_init();
    UART_DEBUG_PutString("[Init] ADC + Joystick ready!\r\n");

    // Initialize looptimer
    Looptimer_Start();
    isr_Looptimer_StartEx(isr_Looptimer_Handler);
    UART_DEBUG_PutString("[Init] Ready  Mode: STANDBY.\r\n");

    // Fire state machine "on entry" function
    app_start(&ctx);
    
    // %======================================================================%
    // Main Loop %============================================================%
    // %======================================================================%
    for (;;)
    {
        DPIN_LOOP_TOGGLE();  // Toggle debug pin
        uint32_t  now = g_tick_ms;
        // Get serial monitor input.
        uint8_t ch = (uint8_t)UART_DEBUG_GetChar();
        // Handle joystick and common keys
        // 'app_dispatch_key' handles the state machine input logic
        if (ch) {
            if (joystick_cal_state() != JOY_CAL_OFF)
                joystick_on_key(ch);
            else
                app_dispatch_key(&ctx, ch);
        }
        
        // Always pump gimbal Rx
        hamfly_pump(&g_movi);
        telemetry_pump(&ctx);
        
        // Always read joystick
        int16_t counts[N_CH];
        if (adc_balanced_poll_frame(counts))
        {
            joystick_on_sample(counts);
            joystick_get_cmd(&ctx.cmd);
        }
        
        // Always run on tick definition
        // These are by state but only run when in the given state
        // Per state logiv before making acontrol packet or telem etc.
        app_stby_tick(&ctx);
        app_manual_tick(&ctx);  // For MANU substates
        app_auto_tick(&ctx);  // For AUTO substates
        
        app_telem_tick(&ctx);  // Telemetry getting and scheduling
        app_sbc_tick(&ctx);  // SBC communications processing
        
        // At 'CONTROL_PERIOD_MS' build and send control packet
        // app_build_control handles the state of the state machine
        if ((now - ctx.last_tx_ms) >= CONTROL_PERIOD_MS) {
            ctx.last_tx_ms = now;
            app_build_control(&ctx, &ctl);
            hamfly_send_control(&g_movi, &ctl);
        }
        
        // At 'DIAG_PERIOD_MS' share telemetry to monitor
        if (ctx.diag_active && (now - ctx.last_diag_ms) >= DIAG_PERIOD_MS) {
            ctx.last_diag_ms = now;
            int16_t pi, pf, ti, tf; char ps, ts;
            decomp(ctx.cmd.u[CH_X], &pi, &pf, &ps);
            decomp(ctx.cmd.u[CH_Y], &ti, &tf, &ts);
            snprintf(tx, sizeof tx,
                     "[%lu] state=%s pan=%c%d.%02d tilt=%c%d.%02d\r\n",
                     (unsigned long)now,
                     app_state_name(ctx.state),
                     ps, pi, pf, ts, ti, tf);
            UART_DEBUG_PutString(tx);
        }
    }
}

/* [] END OF FILE */