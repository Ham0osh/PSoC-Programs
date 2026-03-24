/* main.c
 * HamflyAPI example — PSoC 5LP
 * Joystick gimbal control + one-shot attr test queries.
 *
 * Modes:
 *   STANDBY      passive, pump runs, no TX
 *   CONTROL      QX277 at 10 Hz via joystick
 *   TEST         one-shot attr read, print response or timeout
 *
 * Keys — STANDBY:
 *   m  toggle STANDBY <-> CONTROL
 *   t  enter TEST mode
 *   l  cycle DEFER -> RATE -> ABSOLUTE -> DEFER
 *   k  kill + STANDBY
 *
 * Keys — TEST:
 *   a  attr 22  gimbal Euler
 *   g  attr  4  GPS
 *   b  attr  3  baro
 *   s  attr  1  system status
 *   n  attr 12  magnetometer
 *   t  exit TEST
 *   k  kill + exit
 */

#include <project.h>
#include <stdio.h>
#include <math.h>

#include "hamfly.h"
#include "adc_balanced.h"
#include "joystick.h"

/* =========================================================================
 * Constants
 * ========================================================================= */
#define TX_BUF_SIZE       160u
#define CONTROL_PERIOD_MS 100u    /* 10 Hz */
#define TEST_TIMEOUT_MS  2000u

#define CH_X  0u   /* pan  */
#define CH_Y  1u   /* tilt */

#define DEG_TO_UNIT(d)  ((float)(d) / 180.0f)

/* =========================================================================
 * App state
 * ========================================================================= */
typedef enum {
    MODE_STANDBY = 0,
    MODE_CONTROL,
    MODE_TEST
} app_mode_t;

/* =========================================================================
 * Globals
 * ========================================================================= */
static hamfly_gimbal_t g_gimbal;
volatile uint32_t      g_tick_ms = 0u;

/* =========================================================================
 * HAL — PSoC 5 UART
 * ========================================================================= */
static void psoc_uart_putc(void *ctx, uint8_t b)
{
    (void)ctx;
    UART_MOVI_PutChar((char)b);
}

static uint32_t psoc_get_tick(void *ctx)
{
    (void)ctx;
    return g_tick_ms;
}

static const hamfly_hal_t HAL = {
    .ctx         = NULL,
    .uart_putc   = psoc_uart_putc,
    .get_tick_ms = psoc_get_tick
};

/* =========================================================================
 * ISRs
 * ========================================================================= */
CY_ISR(isr_rx_movi_Handler)
{
    uint8_t st;
    do {
        st = UART_MOVI_RXSTATUS_REG;
        if (st & (UART_MOVI_RX_STS_BREAK     |
                  UART_MOVI_RX_STS_PAR_ERROR |
                  UART_MOVI_RX_STS_STOP_ERROR|
                  UART_MOVI_RX_STS_OVERRUN))
            hamfly_on_uart_err_flags(&g_gimbal, st);

        if (st & UART_MOVI_RX_STS_FIFO_NOTEMPTY)
            hamfly_on_rx_byte(&g_gimbal, UART_MOVI_RXDATA_REG);

    } while (st & UART_MOVI_RX_STS_FIFO_NOTEMPTY);
}

CY_ISR(isr_Looptimer_Handler)
{
    g_tick_ms++;
    Looptimer_ReadStatusRegister();
}

/* =========================================================================
 * Helpers
 * ========================================================================= */
static void decomp(float val, int16_t *i_out, int16_t *f_out, char *sign)
{
    *sign  = (val < 0.0f) ? '-' : ' ';
    if (val < 0.0f) val = -val;
    *i_out = (int16_t)val;
    *f_out = (int16_t)((val - (float)*i_out) * 100.0f);
}

static const char *mode_str(hamfly_control_mode_t m)
{
    switch (m) {
        case HAMFLY_RATE:     return "RATE";
        case HAMFLY_ABSOLUTE: return "ABS ";
        default:              return "DEFR";
    }
}

/* Print whatever attr just landed in pending_payload */
static void print_response(void)
{
    char tx[TX_BUF_SIZE];
    uint16_t attr = g_gimbal.pending_response_attr;
    const uint8_t *p = g_gimbal.pending_payload;
    uint8_t plen     = g_gimbal.pending_payload_len;
    uint8_t i;

    sprintf(tx, "\r\n[TEST] attr=%u  plen=%u\r\n", attr, plen);
    UART_1_PutString(tx);

    /* Hex dump */
    for (i = 0u; i < plen; i += 8u) {
        char *bp = tx;
        uint8_t col;
        bp += sprintf(bp, "  %02u: ", i);
        for (col = 0u; col < 8u && (i + col) < plen; col++)
            bp += sprintf(bp, "%02X ", p[i + col]);
        bp += sprintf(bp, "\r\n");
        UART_1_PutString(tx);
    }

    /* Decoded summary per known attr */
    switch (attr) {

        case 22u: { /* Gimbal Euler */
            if (plen < 6u) break;
            int16_t pitch_r, yaw_r, roll_r;
            int16_t vi; int16_t vf; char s;
            memcpy(&pitch_r, p + 1, 2);
            memcpy(&yaw_r,   p + 3, 2);
            memcpy(&roll_r,  p + 5, 2);
            decomp((float)pitch_r / 100.0f, &vi, &vf, &s);
            sprintf(tx, "  Pitch=%c%d.%02d  ", s, vi, vf);  UART_1_PutString(tx);
            decomp((float)yaw_r   / 100.0f, &vi, &vf, &s);
            sprintf(tx, "Yaw=%c%d.%02d  ",   s, vi, vf);    UART_1_PutString(tx);
            decomp((float)roll_r  / 100.0f, &vi, &vf, &s);
            sprintf(tx, "Roll=%c%d.%02d deg\r\n", s, vi, vf); UART_1_PutString(tx);
            break;
        }

        case 4u: { /* GPS */
            if (plen < 12u) break;
            int32_t lon = (int32_t)(((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3]);
            int32_t lat = (int32_t)(((uint32_t)p[4]<<24)|((uint32_t)p[5]<<16)|((uint32_t)p[6]<<8)|p[7]);
            sprintf(tx, "  Lat=%ld.%07ld  Lon=%ld.%07ld (raw/1e7)\r\n",
                    (long)(lat/10000000L), (long)(lat%10000000L < 0 ? -lat%10000000L : lat%10000000L),
                    (long)(lon/10000000L), (long)(lon%10000000L < 0 ? -lon%10000000L : lon%10000000L));
            UART_1_PutString(tx);
            break;
        }

        case 3u: { /* Baro */
            if (plen < 5u) break;
            int16_t alt, roc;
            int16_t vi; int16_t vf; char s;
            memcpy(&alt, p + 1, 2);
            memcpy(&roc, p + 3, 2);
            decomp((float)alt / 10.0f,  &vi, &vf, &s);
            sprintf(tx, "  Alt=%c%d.%02d m  ", s, vi, vf);   UART_1_PutString(tx);
            decomp((float)roc / 100.0f, &vi, &vf, &s);
            sprintf(tx, "ROC=%c%d.%02d m/s\r\n", s, vi, vf); UART_1_PutString(tx);
            break;
        }

        case 1u: { /* System status */
            if (plen < 4u) break;
            int16_t bv = (int16_t)(((uint16_t)p[0] << 8) | p[1]);
            int16_t vi; int16_t vf; char s;
            decomp((float)bv / 100.0f, &vi, &vf, &s);
            sprintf(tx, "  Batt=%c%d.%02d V  Sats=%u  Temp=%d C\r\n",
                    s, vi, vf, p[2], p[3]/3);
            UART_1_PutString(tx);
            break;
        }

        case 12u: { /* Magnetometer */
            if (plen < 6u) break;
            int16_t x = (int16_t)(((uint16_t)p[1]<<8)|p[2]);
            int16_t y = (int16_t)(((uint16_t)p[3]<<8)|p[4]);
            int16_t z = (int16_t)(((uint16_t)p[5]<<8)|p[6]);
            float hdg = atan2f((float)y, (float)x) * 57.2958f;
            if (hdg < 0.0f) hdg += 360.0f;
            int16_t hi, hf; char hs;
            decomp(hdg, &hi, &hf, &hs);
            sprintf(tx, "  X=%d  Y=%d  Z=%d  hdg~=%d.%02d deg\r\n",
                    x, y, z, hi, hf);
            UART_1_PutString(tx);
            break;
        }

        default:
            UART_1_PutString("  (no decoder for this attr)\r\n");
            break;
    }
}

/* =========================================================================
 * main
 * ========================================================================= */
int main(void)
{
    char tx[TX_BUF_SIZE];

    app_mode_t            app_mode  = MODE_STANDBY;
    hamfly_control_mode_t ctrl_mode = HAMFLY_DEFER;
    hamfly_control_t      ctl;

    uint8_t  joystick_active = 0u;
    uint32_t last_tx_ms      = 0u;

    int16_t   counts[N_CH];
    joy_cmd_t cmd;
    cmd.u[0] = cmd.u[1] = 0.0f;

    joy_cal_t defaults;
    uint8_t i;
    for (i = 0u; i < (uint8_t)N_CH; i++) {
        defaults.minv[i] = 0; defaults.maxv[i] = 255; defaults.center[i] = 128;
    }
    defaults.valid = 0u;

    /* ------------------------------------------------------------------
     * Hardware init
     * ------------------------------------------------------------------ */
    CyGlobalIntEnable;

    UART_1_Start();
    UART_1_PutString("\r\n=== HamflyAPI  PSoC 5LP ===\r\n");
    UART_1_PutString("m=ctrl  t=test  l=mode  j=joy  k=kill\r\n");
    UART_1_PutString("TEST: a=att  g=GPS  b=baro  s=stat  n=mag  t=exit\r\n");

    UART_MOVI_Start();
    UART_MOVI_ClearRxBuffer();
    UART_MOVI_ClearTxBuffer();

    hamfly_init(&g_gimbal, &HAL);
    isr_rx_movi_StartEx(isr_rx_movi_Handler);

    joystick_init(&defaults, (1u << CH_X));
    adc_balanced_init();

    Looptimer_Start();
    isr_Looptimer_StartEx(isr_Looptimer_Handler);

    UART_1_PutString("[Init] Ready\r\n");

    /* ------------------------------------------------------------------
     * Main loop
     * ------------------------------------------------------------------ */
    for (;;)
    {
        /* ----------------------------------------------------------------
         * CLI
         * ---------------------------------------------------------------- */
        uint8_t ch = (uint8_t)UART_1_GetChar();

        if (app_mode == MODE_STANDBY || app_mode == MODE_CONTROL)
        {
            if (ch == 'm' || ch == 'M') {
                if (app_mode == MODE_STANDBY) {
                    app_mode = MODE_CONTROL;
                    UART_1_PutString("[CLI] CONTROL\r\n");
                } else {
                    app_mode = MODE_STANDBY;
                    UART_1_PutString("[CLI] STANDBY\r\n");
                }
            }
            else if (ch == 't' || ch == 'T') {
                if (app_mode == MODE_STANDBY) {
                    app_mode = MODE_TEST;
                    UART_1_PutString("[CLI] TEST  a/g/b/s/n to query  t=exit\r\n");
                }
            }
            else if (ch == 'l' || ch == 'L') {
                if      (ctrl_mode == HAMFLY_DEFER)    ctrl_mode = HAMFLY_RATE;
                else if (ctrl_mode == HAMFLY_RATE)     ctrl_mode = HAMFLY_ABSOLUTE;
                else                                    ctrl_mode = HAMFLY_DEFER;
                sprintf(tx, "[CLI] Mode: %s\r\n", mode_str(ctrl_mode));
                UART_1_PutString(tx);
            }
            else if (ch == 'j' || ch == 'J') {
                joystick_active = !joystick_active;
                UART_1_PutString(joystick_active ? "[CLI] Joystick ON\r\n"
                                                 : "[CLI] Joystick OFF\r\n");
            }
            else if (ch == 'k' || ch == 'K') {
                hamfly_kill(&g_gimbal);
                app_mode       = MODE_STANDBY;
                joystick_active = 0u;
                ctrl_mode      = HAMFLY_DEFER;
                UART_1_PutString("[CLI] Kill sent -> STANDBY\r\n");
            }
        }
        else if (app_mode == MODE_TEST)
        {
            uint16_t req_attr = 0u;

            if      (ch == 'a' || ch == 'A') { req_attr = 22u;  UART_1_PutString("[TEST] attr 22 (Euler)...\r\n"); }
            else if (ch == 'g' || ch == 'G') { req_attr = 4u;   UART_1_PutString("[TEST] attr 4  (GPS)...\r\n"); }
            else if (ch == 'b' || ch == 'B') { req_attr = 3u;   UART_1_PutString("[TEST] attr 3  (baro)...\r\n"); }
            else if (ch == 's' || ch == 'S') { req_attr = 1u;   UART_1_PutString("[TEST] attr 1  (sysstat)...\r\n"); }
            else if (ch == 'n' || ch == 'N') { req_attr = 12u;  UART_1_PutString("[TEST] attr 12 (mag)...\r\n"); }
            else if (ch == 't' || ch == 'T') {
                g_gimbal.pending_attr  = 0u;
                g_gimbal.pending_ready = false;
                app_mode = MODE_STANDBY;
                UART_1_PutString("[TEST] -> STANDBY\r\n");
            }
            else if (ch == 'k' || ch == 'K') {
                hamfly_kill(&g_gimbal);
                app_mode = MODE_STANDBY;
                UART_1_PutString("[TEST] Kill -> STANDBY\r\n");
            }

            if (req_attr != 0u && g_gimbal.pending_attr == 0u) {
                hamfly_request_attr(&g_gimbal, req_attr);
                g_gimbal.pending_sent_ms = g_tick_ms;
            }
        }

        /* ----------------------------------------------------------------
         * Pump RX (always)
         * ---------------------------------------------------------------- */
        hamfly_pump(&g_gimbal);

        /* ----------------------------------------------------------------
         * TEST: response or timeout
         * ---------------------------------------------------------------- */
        if (app_mode == MODE_TEST)
        {
            if (g_gimbal.pending_ready) {
                uint32_t dt = g_tick_ms - g_gimbal.pending_sent_ms;
                sprintf(tx, "  dt=%lums\r\n", (unsigned long)dt);
                UART_1_PutString(tx);
                print_response();
                g_gimbal.pending_ready = false;
                g_gimbal.pending_attr  = 0u;
            }
            else if (g_gimbal.pending_attr != 0u &&
                     (g_tick_ms - g_gimbal.pending_sent_ms) >= TEST_TIMEOUT_MS) {
                sprintf(tx, "[TEST] TIMEOUT attr=%u\r\n",
                        (unsigned)g_gimbal.pending_attr);
                UART_1_PutString(tx);
                g_gimbal.pending_attr = 0u;
            }
        }

        /* ----------------------------------------------------------------
         * Joystick ADC
         * ---------------------------------------------------------------- */
        if (adc_balanced_poll_frame(counts)) {
            joystick_on_sample(counts);
            joystick_get_cmd(&cmd);
        }

        /* ----------------------------------------------------------------
         * Build control
         * ---------------------------------------------------------------- */
        if (joystick_active && joystick_cal_state() == JOY_CAL_OFF) {
            ctl.pan_mode  = ctrl_mode;
            ctl.tilt_mode = ctrl_mode;
            ctl.roll_mode = HAMFLY_DEFER;
            ctl.pan  =  cmd.u[CH_X];
            ctl.tilt = -cmd.u[CH_Y];
            ctl.roll =  0.0f;
        } else {
            ctl.pan_mode  = HAMFLY_DEFER;
            ctl.tilt_mode = HAMFLY_DEFER;
            ctl.roll_mode = HAMFLY_DEFER;
            ctl.pan = ctl.tilt = ctl.roll = 0.0f;
        }
        ctl.enable = 1u;
        ctl.kill   = 0u;

        /* ----------------------------------------------------------------
         * TX at 10 Hz (CONTROL mode only)
         * ---------------------------------------------------------------- */
        if (app_mode == MODE_CONTROL &&
            (g_tick_ms - last_tx_ms) >= CONTROL_PERIOD_MS)
        {
            last_tx_ms = g_tick_ms;
            hamfly_send_control(&g_gimbal, &ctl);
        }
    }
}