/* main.c
 * Author: Hamish Johnson
 * Org: Quantum Internet Systems Lab, SFU, Physics
 * Date: March 2026
 *
 * Uses HamflyAPI (hamfly.h) for all gimbal communication.
 *
 * Modes:
 *   STANDBY   passive; pump runs; 1 Hz status if telemetry valid.
 *             Set boresight origin here before entering other modes.
 *   CONTROL   10 Hz QX277 TX.  Joystick (rate) and numpad presets
 *             share the mode — preset executes, settles to within
 *             PRESET_SETTLE_DEG of target (or PRESET_SETTLE_MS
 *             timeout), then joystick resumes automatically.
 *   QUERY     one-shot attr reads with verbose print or 2 s timeout.
 *             No periodic TX; gimbal holds last command.
 *   NUDGE     keyboard-driven absolute accumulation for beam-profiler
 *             calibration.  Each keypress adds a fixed step to the
 *             accumulated pan/tilt target, clamped to software limits.
 *             Current accumulated position prints after every step.
 *             No joystick input in this mode.
 *
 * Keys — STANDBY:
 *   m  enter CONTROL      q  enter QUERY
 *   n  enter NUDGE        s  toggle ADC stream
 *   j  toggle joystick    l  cycle DEFER/RATE/ABS
 *   c  calibration        k  kill + STANDBY
 *   x  kill + quiet
 *   z  command (0, 0) absolute  (go to boresight)
 *   [  set limit origin at current Euler position
 *      (prints Euler first so you can confirm boresight)
 *
 * Keys — CONTROL (plus all STANDBY keys active):
 *   Numpad presets (require ABS mode via 'l'):
 *     8  tilt +25 deg      2  tilt -25 deg      5  tilt  0 deg
 *     4  pan  -45 deg      6  pan  +45 deg       0  pan   0 deg
 *   (preset executes, joystick resumes automatically after settle)
 *   p  toggle platform-yaw display in 1 Hz diag
 *
 * Keys — QUERY:
 *   a  attr 22 euler       g  attr  4 GPS
 *   b  attr  3 baro        s  attr  1 sysstat
 *   n  attr 12 mag         2  attr  2 platform att
 *   4  attr 48 motor       q  quat crosscheck
 *   t  exit to STANDBY     x  kill + exit
 *
 * Keys — NUDGE:
 *   j  pan  left  (NUDGE_SMALL_DEG)    l  pan  right (NUDGE_SMALL_DEG)
 *   i  tilt up    (NUDGE_SMALL_DEG)    k  tilt down  (NUDGE_SMALL_DEG)
 *   a  pan  left  (NUDGE_LARGE_DEG)    d  pan  right (NUDGE_LARGE_DEG)
 *   w  tilt up    (NUDGE_LARGE_DEG)    s  tilt down  (NUDGE_LARGE_DEG)
 *   (accumulated pan/tilt printed after every step)
 *   t  exit to STANDBY     x  kill + exit
 */

#include <project.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "hamfly.h"
#include "adc_balanced.h"
#include "joystick.h"
#include "psoc_eeprom.h"

/* =========================================================================
 * Timing constants
 * ========================================================================= */
#define TX_BUF_SIZE           160u
#define CONTROL_PERIOD_MS     100u    /* 10 Hz control TX                       */
#define DIAG_PERIOD_MS        1000u   /* 1 Hz diagnostic print                  */
#define QUERY_TIMEOUT_MS      2000u   /* max wait for attr response in QUERY     */
#define PRESET_SETTLE_MS      1500u   /* fallback timeout if threshold not met   */

/* =========================================================================
 * Software limits
 * Limits are relative to the boresight origin set with '[' in STANDBY.
 * All values in degrees.
 * ========================================================================= */
#define LIMIT_PAN_MAX_DEG      15.0f
#define LIMIT_PAN_MIN_DEG     -15.0f
#define LIMIT_TILT_MAX_DEG     15.0f
#define LIMIT_TILT_MIN_DEG    -15.0f

/* =========================================================================
 * Nudge step sizes (degrees)
 * ========================================================================= */
#define NUDGE_SMALL_DEG         0.05f  /* jkli — smallest reliable gimbal step  */
#define NUDGE_LARGE_DEG         0.5f   /* wasd — coarse step for beam scanning  */

/* =========================================================================
 * Preset settle threshold
 * Joystick resumes when |commanded - actual| < this value on both axes,
 * or after PRESET_SETTLE_MS, whichever comes first.
 * ========================================================================= */
#define PRESET_SETTLE_DEG       0.5f

/* =========================================================================
 * Utility macros
 * ========================================================================= */
#define DEG_TO_UNIT(d)   ((float)(d) / 180.0f)
#define UNIT_TO_DEG(u)   ((float)(u) * 180.0f)
#define CLAMP(v,lo,hi)   ((v)<(lo)?(lo):((v)>(hi)?(hi):(v)))
#define FALSE  0u
#define TRUE   1u
#define CH_X   0u
#define CH_Y   1u

/* =========================================================================
 * App state
 * ========================================================================= */
typedef enum {
    APP_MODE_STANDBY = 0,
    APP_MODE_CONTROL,      /* joystick rate + numpad presets                   */
    APP_MODE_QUERY,        /* one-shot attr reads, verbose print                */
    APP_MODE_NUDGE,        /* keyboard abs-accumulation for beam calibration    */
} app_mode_t;

/* =========================================================================
 * Globals
 * ========================================================================= */
static hamfly_gimbal_t g_movi;
volatile uint32_t      g_tick_ms = 0u;

/* =========================================================================
 * HAL
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

static const hamfly_hal_t MOVI_HAL = {
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

/* =========================================================================
 * Helpers
 * ========================================================================= */
static void decomp(float val, int16_t *i_out, int16_t *f_out, char *sign)
{
    *sign  = (val < 0.0f) ? '-' : ' ';
    if (val < 0.0f) val = -val;
    *i_out = (int16_t)val;
    *f_out = (int16_t)((val - (float)*i_out) * 100.0f);
    if (*f_out < 0) *f_out = -*f_out;
}

static void compute_euler(const hamfly_telemetry_t *st,
    char *pan_s,  int16_t *pan_i,  int16_t *pan_f,
    char *tilt_s, int16_t *tilt_i, int16_t *tilt_f,
    char *roll_s, int16_t *roll_i, int16_t *roll_f)
{
    float r  = st->gimbal_r, ii = st->gimbal_i;
    float j  = st->gimbal_j, k  = st->gimbal_k;
    float sinr = 2.0f*(r*ii+j*k), cosr = 1.0f-2.0f*(ii*ii+j*j);
    float roll_deg = atan2f(sinr, cosr) * 57.2958f;
    float sinp = 2.0f*(r*j-k*ii);
    float tilt_deg = (sinp >= 1.0f) ?  90.0f :
                     (sinp <=-1.0f) ? -90.0f :
                     asinf(sinp) * 57.2958f;
    float siny = 2.0f*(r*k+ii*j), cosy = 1.0f-2.0f*(j*j+k*k);
    float pan_deg = atan2f(siny, cosy) * 57.2958f;
    decomp(pan_deg,  pan_i,  pan_f,  pan_s);
    decomp(tilt_deg, tilt_i, tilt_f, tilt_s);
    decomp(roll_deg, roll_i, roll_f, roll_s);
}

/* Extract pan and tilt in degrees from current telemetry (no decomp). */
static uint8_t get_euler_deg(float *pan_deg_out, float *tilt_deg_out)
{
    hamfly_telemetry_t st;
    hamfly_get_telemetry(&g_movi, &st);
    if (!st.valid) return FALSE;
    float r  = st.gimbal_r, ii = st.gimbal_i;
    float j  = st.gimbal_j, k  = st.gimbal_k;
    float sinp = 2.0f*(r*j-k*ii);
    *tilt_deg_out = (sinp >= 1.0f) ?  90.0f :
                    (sinp <=-1.0f) ? -90.0f :
                    asinf(sinp) * 57.2958f;
    float siny = 2.0f*(r*k+ii*j), cosy = 1.0f-2.0f*(j*j+k*k);
    *pan_deg_out = atan2f(siny, cosy) * 57.2958f;
    return TRUE;
}

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
        UART_1_PutString(buf);
    }
}

/* =========================================================================
 * Snapshot macros
 * ========================================================================= */
#define SNAP16(p,plen,off) \
    (((off)+2u<=(plen))?({int16_t _v;memcpy(&_v,(p)+(off),2);_v;}):(int16_t)0)
#define SNAP16BE(p,plen,off) \
    (((off)+2u<=(plen)) \
        ?(int16_t)(((uint16_t)(p)[(off)]<<8)|(uint16_t)(p)[(off)+1u]) \
        :(int16_t)0)
#define SNAP32BE(p,plen,off) \
    (((off)+4u<=(plen)) \
        ?(int32_t)(((uint32_t)(p)[(off)]<<24)|((uint32_t)(p)[(off)+1u]<<16) \
                  |((uint32_t)(p)[(off)+2u]<<8)|(uint32_t)(p)[(off)+3u]) \
        :(int32_t)0)

/* =========================================================================
 * QUERY verbose printers  (unchanged from integration version)
 * ========================================================================= */
static void print_att(const uint8_t *p, uint8_t plen, uint32_t dt)
{
    char buf[TX_BUF_SIZE];
    sprintf(buf, "\r\n[QUERY][Att] dt=%lums snaplen=%u\r\n",
            (unsigned long)dt, plen);
    UART_1_PutString(buf);
    print_hex_dump(p, plen);
    if (plen < 6u) { UART_1_PutString("  short\r\n"); return; }
    char s; int16_t vi, vf;
    int16_t r1=SNAP16(p,plen,1), r3=SNAP16(p,plen,3), r5=SNAP16(p,plen,5);
    decomp((float)r1/100.0f, &vi, &vf, &s);
    sprintf(buf,"  off1 /100 %6d => %c%d.%02d [PITCH]\r\n",(int)r1,s,vi,vf);
    UART_1_PutString(buf);
    decomp((float)r3/100.0f, &vi, &vf, &s);
    sprintf(buf,"  off3 /100 %6d => %c%d.%02d [YAW]\r\n",(int)r3,s,vi,vf);
    UART_1_PutString(buf);
    decomp((float)r5/100.0f, &vi, &vf, &s);
    sprintf(buf,"  off5 /100 %6d => %c%d.%02d [ROLL]\r\n",(int)r5,s,vi,vf);
    UART_1_PutString(buf);
}

static void print_gps(const uint8_t *p, uint8_t plen, uint32_t dt)
{
    char buf[TX_BUF_SIZE];
    sprintf(buf, "\r\n[QUERY][GPS] dt=%lums snaplen=%u\r\n",
            (unsigned long)dt, plen);
    UART_1_PutString(buf);
    print_hex_dump(p, plen);
    if (plen < 12u) { UART_1_PutString("  short\r\n"); return; }
    int32_t lon=SNAP32BE(p,plen,0), lat=SNAP32BE(p,plen,4);
    int32_t alt=SNAP32BE(p,plen,8);
    sprintf(buf,"  Lat=%ld.%07ld Lon=%ld.%07ld Alt=%ld mm\r\n",
            (long)(lat/10000000L),(long)(lat%10000000L),
            (long)(lon/10000000L),(long)(lon%10000000L),(long)alt);
    UART_1_PutString(buf);
}

static void print_baro(const uint8_t *p, uint8_t plen, uint32_t dt)
{
    char buf[TX_BUF_SIZE];
    sprintf(buf, "\r\n[QUERY][Baro] dt=%lums snaplen=%u\r\n",
            (unsigned long)dt, plen);
    UART_1_PutString(buf);
    print_hex_dump(p, plen);
    if (plen < 5u) { UART_1_PutString("  short\r\n"); return; }
    char s; int16_t vi, vf;
    int16_t alt=SNAP16(p,plen,1), roc=SNAP16(p,plen,3);
    decomp((float)alt/10.0f, &vi, &vf, &s);
    sprintf(buf,"  Alt=%c%d.%02d m  ", s, vi, vf); UART_1_PutString(buf);
    decomp((float)roc/100.0f, &vi, &vf, &s);
    sprintf(buf,"ROC=%c%d.%02d m/s\r\n", s, vi, vf); UART_1_PutString(buf);
}

static void print_sysstat(const uint8_t *p, uint8_t plen, uint32_t dt)
{
    char buf[TX_BUF_SIZE];
    sprintf(buf, "\r\n[QUERY][Stat] dt=%lums snaplen=%u\r\n",
            (unsigned long)dt, plen);
    UART_1_PutString(buf);
    print_hex_dump(p, plen);
    if (plen < 4u) { UART_1_PutString("  short\r\n"); return; }
    char s; int16_t vi, vf;
    int16_t bv = SNAP16BE(p,plen,0);
    decomp((float)bv/100.0f, &vi, &vf, &s);
    sprintf(buf,"  Batt=%c%d.%02d V  Sats=%u  Temp=%d C\r\n",
            s, vi, vf, p[2], p[3]/3u);
    UART_1_PutString(buf);
    if (plen >= 11u) {
        int16_t pres = SNAP16BE(p,plen,9);
        decomp((float)pres/10.0f, &vi, &vf, &s);
        sprintf(buf,"  Pres=%c%d.%02d mb\r\n", s, vi, vf);
        UART_1_PutString(buf);
    }
}

static void print_mag(const uint8_t *p, uint8_t plen, uint32_t dt)
{
    char buf[TX_BUF_SIZE];
    sprintf(buf, "\r\n[QUERY][Mag] dt=%lums snaplen=%u\r\n",
            (unsigned long)dt, plen);
    UART_1_PutString(buf);
    print_hex_dump(p, plen);
    if (plen < 6u) { UART_1_PutString("  short\r\n"); return; }
    int16_t x=SNAP16BE(p,plen,0), y=SNAP16BE(p,plen,2), z=SNAP16BE(p,plen,4);
    float hdg = atan2f((float)y,(float)x)*57.2958f;
    if (hdg < 0.0f) hdg += 360.0f;
    char s; int16_t hi, hf;
    decomp(hdg, &hi, &hf, &s);
    sprintf(buf,"  X=%d Y=%d Z=%d  hdg~=%d.%02d deg\r\n",
            (int)x,(int)y,(int)z,(int)hi,(int)hf);
    UART_1_PutString(buf);
}

static void print_platform(const uint8_t *p, uint8_t plen, uint32_t dt)
{
    char buf[TX_BUF_SIZE];
    sprintf(buf, "\r\n[QUERY][Plat] dt=%lums snaplen=%u\r\n",
            (unsigned long)dt, plen);
    UART_1_PutString(buf);
    print_hex_dump(p, plen);
    if (plen < 7u) { UART_1_PutString("  short\r\n"); return; }
    char s; int16_t vi, vf;
    int16_t roll =SNAP16BE(p,plen,1);
    int16_t pitch=SNAP16BE(p,plen,3);
    int16_t yaw  =SNAP16BE(p,plen,5);
    decomp((float)roll /100.0f,&vi,&vf,&s);
    sprintf(buf,"  Roll =%c%d.%02d  ",s,vi,vf); UART_1_PutString(buf);
    decomp((float)pitch/100.0f,&vi,&vf,&s);
    sprintf(buf,"Pitch=%c%d.%02d  ",s,vi,vf); UART_1_PutString(buf);
    decomp((float)yaw  /100.0f,&vi,&vf,&s);
    sprintf(buf,"Yaw=%c%d.%02d deg\r\n",s,vi,vf); UART_1_PutString(buf);
    UART_1_PutString("  (GCU box, not camera)\r\n");
}

static void print_motor(const uint8_t *p, uint8_t plen, uint32_t dt)
{
    char buf[TX_BUF_SIZE];
    sprintf(buf, "\r\n[QUERY][Drv] dt=%lums snaplen=%u\r\n",
            (unsigned long)dt, plen);
    UART_1_PutString(buf);
    print_hex_dump(p, plen);
    if (plen < 9u) { UART_1_PutString("  short\r\n"); return; }
    sprintf(buf,"  off8=0x%02X (device flag)\r\n",(unsigned)p[8]);
    UART_1_PutString(buf);
    if (plen >= 14u) {
        uint8_t any = 0u, i;
        for (i=9u;i<=13u;i++) if(p[i]) { any=1u; break; }
        sprintf(buf,"  off9-13: %02X %02X %02X %02X %02X  %s\r\n",
                p[9],p[10],p[11],p[12],p[13],
                any ? "non-zero: active" : "all zero: idle");
        UART_1_PutString(buf);
    }
}

static void print_query_response(uint32_t dt)
{
    uint16_t attr = g_movi.pending_response_attr;
    const uint8_t *p = g_movi.pending_payload;
    uint8_t plen = g_movi.pending_payload_len;
    switch (attr) {
        case 22u: print_att(p, plen, dt);      break;
        case  4u: print_gps(p, plen, dt);      break;
        case  3u: print_baro(p, plen, dt);     break;
        case  1u: print_sysstat(p, plen, dt);  break;
        case 12u: print_mag(p, plen, dt);      break;
        case  2u: print_platform(p, plen, dt); break;
        case 48u: print_motor(p, plen, dt);    break;
        default: {
            char buf[64];
            sprintf(buf, "[QUERY] attr=%u -- no printer\r\n", (unsigned)attr);
            UART_1_PutString(buf);
        }
    }
}

/* =========================================================================
 * Kill helper
 * ========================================================================= */
static void do_kill(app_mode_t *mode, uint8_t *joy_active,
                    uint8_t *streaming, hamfly_control_mode_t *ctrl_mode)
{
    if (*mode == APP_MODE_CONTROL || *mode == APP_MODE_NUDGE) {
        hamfly_kill(&g_movi);
        UART_1_PutString("[CLI] Kill sent\r\n");
    }
    *joy_active = FALSE;
    *mode       = APP_MODE_STANDBY;
    *streaming  = FALSE;
    *ctrl_mode  = HAMFLY_DEFER;
    joystick_on_key('x');
    UART_1_PutString("[CLI] Stopped -> STANDBY\r\n");
}

/* =========================================================================
 * Quat crosscheck helper
 * ========================================================================= */
static void print_quat_crosscheck(void)
{
    char buf[TX_BUF_SIZE];
    hamfly_telemetry_t st;
    hamfly_get_telemetry(&g_movi, &st);

    UART_1_PutString("\r\n[QUERY] Quat vs Euler crosscheck\r\n");
    if (!st.valid) {
        UART_1_PutString("[QUERY] No valid telemetry -- enter CONTROL first\r\n");
        return;
    }
    char ps, ts, rs;
    int16_t pi, pf, ti, tf, ri, rf;
    compute_euler(&st, &ps,&pi,&pf, &ts,&ti,&tf, &rs,&ri,&rf);
    sprintf(buf,"  Q->Euler: Pan=%c%d.%02d Tilt=%c%d.%02d Roll=%c%d.%02d\r\n",
            ps,pi,pf, ts,ti,tf, rs,ri,rf);
    UART_1_PutString(buf);
    sprintf(buf,"  Raw: R=%.4f I=%.4f J=%.4f K=%.4f\r\n",
            st.gimbal_r, st.gimbal_i, st.gimbal_j, st.gimbal_k);
    UART_1_PutString(buf);
    UART_1_PutString("[QUERY] Requesting attr 22 for direct Euler...\r\n");
    hamfly_request_attr(&g_movi, 22u);
    g_movi.pending_sent_ms = g_tick_ms;
}

/* =========================================================================
 * Calibration prompt helper
 * ========================================================================= */
static void cal_prompt(joy_cal_state_t prev, joy_cal_state_t now)
{
    if (prev == now) return;
    if (now == JOY_CAL_RANGE_CAPTURE)
        UART_1_PutString("\r\n[CAL] Move joystick full range then press 'c'\r\n");
    else if (now == JOY_CAL_CENTER_CAPTURE)
        UART_1_PutString("\r\n[CAL] Hold center...\r\n");
    else if (now == JOY_CAL_CONFIRM_SAVE)
        UART_1_PutString("\r\n[CAL] Press 'c' to save, 'x' to cancel\r\n");
    else if (now == JOY_CAL_OFF && prev == JOY_CAL_CONFIRM_SAVE) {
        if (joystick_save())
            UART_1_PutString("[CAL] Saved to EEPROM\r\n");
        else
            UART_1_PutString("[CAL] EEPROM save FAILED\r\n");
            char dbuf[64];
            sprintf(dbuf, "[CAL] sizeof(joy_cal_t)=%u\r\n",
                    (unsigned)sizeof(joy_cal_t));
            UART_1_PutString(dbuf);
    }
    else if (now == JOY_CAL_OFF && prev != JOY_CAL_OFF)
        UART_1_PutString("[CAL] Cancelled\r\n");
}

/* =========================================================================
 * NUDGE: print accumulated position
 * ========================================================================= */
static void nudge_print_pos(float pan_deg, float tilt_deg)
{
    char buf[TX_BUF_SIZE];
    char ps, ts;
    int16_t pi, pf, ti, tf;
    decomp(pan_deg,  &pi, &pf, &ps);
    decomp(tilt_deg, &ti, &tf, &ts);
    sprintf(buf, "[NUDGE] Pan=%c%d.%02d  Tilt=%c%d.%02d  (deg from origin)\r\n",
            ps, pi, pf, ts, ti, tf);
    UART_1_PutString(buf);
}

/* =========================================================================
 * main
 * ========================================================================= */
int main(void)
{
    char tx[TX_BUF_SIZE];

    app_mode_t            app_mode  = APP_MODE_STANDBY;
    hamfly_control_mode_t ctrl_mode = HAMFLY_DEFER;
    hamfly_control_t      ctl;

    uint8_t joystick_active   = FALSE;
    uint8_t streaming_adc     = FALSE;
    uint8_t diag_active       = FALSE;
    uint8_t show_platform_yaw = FALSE;
    uint8_t ctrl_attr_pending = 0u;

    /* CONTROL: absolute preset state */
    float   abs_pan_target    = 0.0f;   /* in hamfly units [-1, +1]             */
    float   abs_tilt_target   = 0.0f;
    uint8_t abs_preset_active = FALSE;
    uint32_t preset_sent_ms   = 0u;

    /* NUDGE: accumulated position in degrees relative to limit origin */
    float   nudge_pan_deg     = 0.0f;
    float   nudge_tilt_deg    = 0.0f;

    /* Software limit origin: Euler degrees at the moment '[' was pressed.
     * All limit checks are relative to this origin. */
    float   origin_pan_deg    = 0.0f;
    float   origin_tilt_deg   = 0.0f;
    uint8_t origin_set        = FALSE;

    uint32_t last_tx_ms   = 0u;
    uint32_t last_diag_ms = 0u;

    int16_t   counts[N_CH];
    joy_cmd_t cmd;
    cmd.u[0] = cmd.u[1] = 0.0f;

    joy_cal_t defaults;
    uint8_t i;
    for (i=0u; i<(uint8_t)N_CH; i++) {
        defaults.minv[i]=0; defaults.maxv[i]=255; defaults.center[i]=128;
    }
    defaults.valid = 0u;

    joy_cal_state_t   cal_prev     = JOY_CAL_OFF;
    joy_sensitivity_t current_sense = SENSE_MED;
    uint8_t           last_button  = 1u;
    uint32_t          invert_mask  = (1u << CH_X);

    /* ------------------------------------------------------------------
     * Hardware init
     * ------------------------------------------------------------------ */
    CyGlobalIntEnable;

    UART_1_Start();
    UART_1_PutString("\r\n=== Gimbal Control  PSoC 5LP ===\r\n");
    UART_1_PutString("STANDBY: m=CONTROL q=QUERY n=NUDGE\r\n");
    UART_1_PutString("         j=joy l=mode s=adc c=cal k=kill x=quiet\r\n");
    UART_1_PutString("         z=goto(0,0)  [=set limit origin\r\n");
    UART_1_PutString("CONTROL: numpad 8/2/5=tilt  4/6/0=pan  p=plat-yaw\r\n");
    UART_1_PutString("QUERY:   a=euler g=GPS b=baro s=stat n=mag\r\n");
    UART_1_PutString("         2=plat 4=motor q=quat  t=exit\r\n");
    UART_1_PutString("NUDGE:   jl=pan(small) ik=tilt(small)\r\n");
    UART_1_PutString("         ad=pan(0.5deg) ws=tilt(0.5deg)  t=exit\r\n");

    UART_MOVI_Start();
    UART_MOVI_ClearRxBuffer();
    UART_MOVI_ClearTxBuffer();

    hamfly_init(&g_movi, &MOVI_HAL);
    isr_rx_movi_StartEx(isr_rx_movi_Handler);
    UART_1_PutString("[Init] Gimbal UART ready\r\n");
    psoc_eeprom_init();
    if (!joystick_load(invert_mask)) {
        UART_1_PutString("[Init] No saved cal -- using defaults\r\n");
        joystick_init(&defaults, invert_mask);
    } else {
        UART_1_PutString("[Init] Loaded cal from EEPROM\r\n");
    }
    adc_balanced_init();
    UART_1_PutString("[Init] ADC + Joystick ready\r\n");

    Looptimer_Start();
    isr_Looptimer_StartEx(isr_Looptimer_Handler);
    UART_1_PutString("[Init] Ready  Mode: STANDBY\r\n");

    /* ------------------------------------------------------------------
     * Main loop
     * ------------------------------------------------------------------ */
    for (;;)
    {
        /* ----------------------------------------------------------------
         * CLI
         * ---------------------------------------------------------------- */
        uint8_t ch = (uint8_t)UART_1_GetChar();

        /* ----------------------------------------------------------------
         * STANDBY / CONTROL shared keys
         * ---------------------------------------------------------------- */
        if (app_mode == APP_MODE_STANDBY ||
            app_mode == APP_MODE_CONTROL)
        {
            if (ch == 'm' || ch == 'M') {
                if (app_mode == APP_MODE_STANDBY) {
                    app_mode = APP_MODE_CONTROL;
                    diag_active = TRUE;
                    abs_preset_active = FALSE;
                    UART_1_PutString("[CLI] -> CONTROL\r\n");
                } else {
                    app_mode = APP_MODE_STANDBY;
                    UART_1_PutString("[CLI] -> STANDBY\r\n");
                }
            }
            else if ((ch=='q'||ch=='Q') && app_mode==APP_MODE_STANDBY) {
                app_mode = APP_MODE_QUERY;
                UART_1_PutString("[CLI] -> QUERY\r\n");
            }
            else if ((ch=='n'||ch=='N') && app_mode==APP_MODE_STANDBY) {
                nudge_pan_deg  = 0.0f;
                nudge_tilt_deg = 0.0f;
                app_mode = APP_MODE_NUDGE;
                UART_1_PutString("[CLI] -> NUDGE  (jl=pan ik=tilt ad=pan0.5 ws=tilt0.5 t=exit)\r\n");
                if (!origin_set)
                    UART_1_PutString("[NUDGE] Warning: limit origin not set. Use '[' in STANDBY.\r\n");
                nudge_print_pos(nudge_pan_deg, nudge_tilt_deg);
            }
            else if (ch=='j'||ch=='J') {
                joystick_active = !joystick_active;
                if (joystick_active) { diag_active=TRUE; abs_preset_active=FALSE; }
                UART_1_PutString(joystick_active
                    ? "[CLI] Joystick ON\r\n" : "[CLI] Joystick OFF\r\n");
            }
            else if (ch=='l'||ch=='L') {
                if      (ctrl_mode==HAMFLY_DEFER)    ctrl_mode=HAMFLY_RATE;
                else if (ctrl_mode==HAMFLY_RATE)     ctrl_mode=HAMFLY_ABSOLUTE;
                else                                  ctrl_mode=HAMFLY_DEFER;
                abs_preset_active = FALSE;
                sprintf(tx,"[CLI] Mode: %s\r\n",mode_str(ctrl_mode));
                UART_1_PutString(tx);
            }
            else if (ch=='k'||ch=='K') {
                abs_preset_active = FALSE;
                do_kill(&app_mode, &joystick_active,
                        &streaming_adc, &ctrl_mode);
            }
            else if (ch=='x'||ch=='X') {
                abs_preset_active = FALSE;
                do_kill(&app_mode, &joystick_active,
                        &streaming_adc, &ctrl_mode);
                diag_active = FALSE;
                UART_1_PutString("[CLI] Quiet. m or j to resume.\r\n");
            }
            else if (ch=='s'||ch=='S') {
                streaming_adc = !streaming_adc;
                UART_1_PutString(streaming_adc
                    ? "[CLI] ADC stream ON\r\n":"[CLI] ADC stream OFF\r\n");
            }
            else if (ch=='c'||ch=='C') {
                joystick_on_key('c');
            }
            /* Go to absolute (0,0) — optical boresight shortcut */
            else if (ch=='z'||ch=='Z') {
                abs_pan_target    = 0.0f;
                abs_tilt_target   = 0.0f;
                abs_preset_active = TRUE;
                joystick_active   = FALSE;
                preset_sent_ms    = g_tick_ms;
                /* Ensure ABS mode is active */
                ctrl_mode = HAMFLY_ABSOLUTE;
                if (app_mode == APP_MODE_STANDBY)
                    app_mode = APP_MODE_CONTROL;
                UART_1_PutString("[CLI] Commanding (0, 0)...\r\n");
            }
            /* Set limit origin at current Euler position */
            else if (ch=='[') {
                float pd = 0.0f, td = 0.0f;
                if (get_euler_deg(&pd, &td)) {
                    origin_pan_deg  = pd;
                    origin_tilt_deg = td;
                    origin_set = TRUE;
                    char ps, ts;
                    int16_t pi, pf, ti, tf;
                    decomp(pd, &pi, &pf, &ps);
                    decomp(td, &ti, &tf, &ts);
                    sprintf(tx,
                        "[CLI] Limit origin set: Pan=%c%d.%02d  Tilt=%c%d.%02d deg\r\n",
                        ps,pi,pf, ts,ti,tf);
                    UART_1_PutString(tx);
                } else {
                    UART_1_PutString("[CLI] No valid telemetry -- cannot set origin\r\n");
                }
            }

            /* ---- CONTROL-only keys ---- */
            if (app_mode == APP_MODE_CONTROL) {
                if (ch=='p'||ch=='P') {
                    show_platform_yaw = !show_platform_yaw;
                    UART_1_PutString(show_platform_yaw
                        ? "[CTRL] Platform yaw ON\r\n"
                        : "[CTRL] Platform yaw OFF\r\n");
                }
                /* Numpad presets: require ABS mode */
                else if (ch=='8'||ch=='2'||ch=='5'||
                         ch=='4'||ch=='6'||ch=='0') {
                    if (ctrl_mode != HAMFLY_ABSOLUTE) {
                        UART_1_PutString("[CTRL] Need ABS mode -- press 'l'\r\n");
                    } else {
                        switch (ch) {
                            /* Pan presets */
                            case '4':
                                abs_pan_target  = DEG_TO_UNIT(-45.0f);
                                abs_tilt_target = 0.0f;
                                UART_1_PutString("[CTRL] Pan -45 deg\r\n"); break;
                            case '0':
                                abs_pan_target  = 0.0f;
                                abs_tilt_target = 0.0f;
                                UART_1_PutString("[CTRL] Pan 0 (home)\r\n"); break;
                            case '6':
                                abs_pan_target  = DEG_TO_UNIT(45.0f);
                                abs_tilt_target = 0.0f;
                                UART_1_PutString("[CTRL] Pan +45 deg\r\n"); break;
                            /* Tilt presets */
                            case '8':
                                abs_pan_target  = 0.0f;
                                abs_tilt_target = DEG_TO_UNIT(25.0f);
                                UART_1_PutString("[CTRL] Tilt +25 deg\r\n"); break;
                            case '5':
                                abs_pan_target  = 0.0f;
                                abs_tilt_target = 0.0f;
                                UART_1_PutString("[CTRL] Tilt 0 deg\r\n"); break;
                            case '2':
                                abs_pan_target  = 0.0f;
                                abs_tilt_target = DEG_TO_UNIT(-25.0f);
                                UART_1_PutString("[CTRL] Tilt -25 deg\r\n"); break;
                            default: break;
                        }
                        abs_preset_active = TRUE;
                        joystick_active   = FALSE;
                        preset_sent_ms    = g_tick_ms;
                    }
                }
            }
        }

        /* ----------------------------------------------------------------
         * QUERY keys
         * ---------------------------------------------------------------- */
        else if (app_mode == APP_MODE_QUERY) {
            if (ch=='t'||ch=='T') {
                g_movi.pending_attr  = 0u;
                g_movi.pending_ready = false;
                app_mode = APP_MODE_STANDBY;
                UART_1_PutString("[QUERY] -> STANDBY\r\n");
            }
            else if (ch=='x'||ch=='X') {
                g_movi.pending_attr  = 0u;
                g_movi.pending_ready = false;
                do_kill(&app_mode,&joystick_active,&streaming_adc,&ctrl_mode);
            }
            else {
                uint16_t req = 0u;
                if      (ch=='a'||ch=='A') { req=22u; UART_1_PutString("[QUERY] attr 22 euler\r\n"); }
                else if (ch=='g'||ch=='G') { req=4u;  UART_1_PutString("[QUERY] attr 4 GPS\r\n"); }
                else if (ch=='b'||ch=='B') { req=3u;  UART_1_PutString("[QUERY] attr 3 baro\r\n"); }
                else if (ch=='s'||ch=='S') { req=1u;  UART_1_PutString("[QUERY] attr 1 sysstat\r\n"); }
                else if (ch=='n'||ch=='N') { req=12u; UART_1_PutString("[QUERY] attr 12 mag\r\n"); }
                else if (ch=='2')          { req=2u;  UART_1_PutString("[QUERY] attr 2 platform\r\n"); }
                else if (ch=='4')          { req=48u; UART_1_PutString("[QUERY] attr 48 motor\r\n"); }
                else if (ch=='q'||ch=='Q') { print_quat_crosscheck(); }
                if (req != 0u && g_movi.pending_attr == 0u) {
                    hamfly_request_attr(&g_movi, req);
                    g_movi.pending_sent_ms = g_tick_ms;
                }
            }
        }

        /* ----------------------------------------------------------------
         * NUDGE keys
         * ---------------------------------------------------------------- */
        else if (app_mode == APP_MODE_NUDGE) {
            if (ch=='t'||ch=='T') {
                app_mode = APP_MODE_STANDBY;
                UART_1_PutString("[NUDGE] -> STANDBY\r\n");
            }
            else if (ch=='x'||ch=='X') {
                do_kill(&app_mode,&joystick_active,&streaming_adc,&ctrl_mode);
            }
            else {
                float dp = 0.0f, dt = 0.0f;
                if      (ch=='j'||ch=='J') dp = -NUDGE_SMALL_DEG;
                else if (ch=='l'||ch=='L') dp =  NUDGE_SMALL_DEG;
                else if (ch=='i'||ch=='I') dt =  NUDGE_SMALL_DEG;
                else if (ch=='k'||ch=='K') dt = -NUDGE_SMALL_DEG;
                else if (ch=='a'||ch=='A') dp = -NUDGE_LARGE_DEG;
                else if (ch=='d'||ch=='D') dp =  NUDGE_LARGE_DEG;
                else if (ch=='w'||ch=='W') dt =  NUDGE_LARGE_DEG;
                else if (ch=='s'||ch=='S') dt = -NUDGE_LARGE_DEG;

                if (dp != 0.0f || dt != 0.0f) {
                    nudge_pan_deg  = CLAMP(nudge_pan_deg  + dp,
                                          LIMIT_PAN_MIN_DEG,  LIMIT_PAN_MAX_DEG);
                    nudge_tilt_deg = CLAMP(nudge_tilt_deg + dt,
                                          LIMIT_TILT_MIN_DEG, LIMIT_TILT_MAX_DEG);
                    nudge_print_pos(nudge_pan_deg, nudge_tilt_deg);
                }
            }
        }

        /* Calibration prompt */
        joy_cal_state_t cal_now = joystick_cal_state();
        cal_prompt(cal_prev, cal_now);
        cal_prev = cal_now;

        /* ----------------------------------------------------------------
         * Pump RX
         * ---------------------------------------------------------------- */
        hamfly_pump(&g_movi);

        /* ----------------------------------------------------------------
         * QUERY: response or timeout
         * ---------------------------------------------------------------- */
        if (app_mode == APP_MODE_QUERY) {
            if (g_movi.pending_ready) {
                uint32_t dt = g_tick_ms - g_movi.pending_sent_ms;
                print_query_response(dt);
                g_movi.pending_ready = false;
                g_movi.pending_attr  = 0u;
            }
            else if (g_movi.pending_attr != 0u &&
                     (g_tick_ms - g_movi.pending_sent_ms) >= QUERY_TIMEOUT_MS) {
                sprintf(tx,"[QUERY] TIMEOUT attr=%u\r\n",
                        (unsigned)g_movi.pending_attr);
                UART_1_PutString(tx);
                g_movi.pending_attr = 0u;
            }
        }

        /* ----------------------------------------------------------------
         * CONTROL: check for preset settle via Euler threshold + timeout
         * ---------------------------------------------------------------- */
        if (app_mode == APP_MODE_CONTROL && abs_preset_active) {
            uint8_t settled = FALSE;
            float pan_now = 0.0f, tilt_now = 0.0f;

            /* Try Euler threshold first */
            if (get_euler_deg(&pan_now, &tilt_now)) {
                float pan_err  = pan_now  - UNIT_TO_DEG(abs_pan_target);
                float tilt_err = tilt_now - UNIT_TO_DEG(abs_tilt_target);
                if (pan_err  < 0.0f) pan_err  = -pan_err;
                if (tilt_err < 0.0f) tilt_err = -tilt_err;
                if (pan_err < PRESET_SETTLE_DEG && tilt_err < PRESET_SETTLE_DEG)
                    settled = TRUE;
            }

            /* Fallback: timeout */
            if (!settled &&
                (g_tick_ms - preset_sent_ms) >= PRESET_SETTLE_MS)
                settled = TRUE;

            if (settled) {
                abs_preset_active = FALSE;
                joystick_active   = TRUE;
                UART_1_PutString("[CTRL] Preset settled -- joystick active\r\n");
            }
        }

        /* ----------------------------------------------------------------
         * Joystick ADC
         * ---------------------------------------------------------------- */
        uint8_t pin = Pin_Sensitivity_Read();
        if (last_button==1u && pin==0u) {
            CyDelay(50);
            if (Pin_Sensitivity_Read()==0u) {
                current_sense=(joy_sensitivity_t)((current_sense+1)%3);
                joystick_set_sensitivity(current_sense);
                UART_1_PutString("Sensitivity changed\r\n");
                while (Pin_Sensitivity_Read()==0u);
            }
        }
        last_button = pin;

        if (adc_balanced_poll_frame(counts)) {
            joystick_on_sample(counts);
            joystick_get_cmd(&cmd);
            if (streaming_adc) {
                int16_t pm = (int16_t)(cmd.u[CH_X]*1000.0f);
                int16_t tm = (int16_t)(cmd.u[CH_Y]*1000.0f);
                int32_t xmv = adc_balanced_counts_to_mv(counts[CH_X]);
                int32_t ymv = adc_balanced_counts_to_mv(counts[CH_Y]);
                sprintf(tx,"X:%ld mV Y:%ld mV  pan:%d tilt:%d /1000\r\n",
                        (long)xmv,(long)ymv,(int)pm,(int)tm);
                UART_1_PutString(tx);
            }
            if (joystick_cal_state()==JOY_CAL_CENTER_CAPTURE) {
                uint16_t n = joystick_center_samples_collected();
                if ((n%8u)==0u) {
                    sprintf(tx,"[CAL] %u/%u\r\n",(unsigned)n,
                            (unsigned)CAL_CENTER_SAMPLES);
                    UART_1_PutString(tx);
                }
            }
        }

        /* ----------------------------------------------------------------
         * Build control struct
         * ---------------------------------------------------------------- */
        ctl.enable = 1u;
        ctl.kill   = 0u;

        if (app_mode == APP_MODE_NUDGE) {
            /* NUDGE: send accumulated position as ABS command every loop */
            ctl.pan_mode  = HAMFLY_ABSOLUTE;
            ctl.tilt_mode = HAMFLY_ABSOLUTE;
            ctl.roll_mode = HAMFLY_DEFER;
            ctl.pan  = DEG_TO_UNIT(nudge_pan_deg);
            ctl.tilt = DEG_TO_UNIT(nudge_tilt_deg);
            ctl.roll = 0.0f;
        }
        else if (abs_preset_active && ctrl_mode==HAMFLY_ABSOLUTE) {
            ctl.pan_mode  = HAMFLY_ABSOLUTE;
            ctl.tilt_mode = HAMFLY_ABSOLUTE;
            ctl.roll_mode = HAMFLY_DEFER;
            ctl.pan  = abs_pan_target;
            ctl.tilt = abs_tilt_target;
            ctl.roll = 0.0f;
        }
        else if (joystick_active && joystick_cal_state()==JOY_CAL_OFF) {
            ctl.pan_mode  = ctrl_mode;
            ctl.tilt_mode = ctrl_mode;
            ctl.roll_mode = HAMFLY_DEFER;
            ctl.pan  =  cmd.u[CH_X];
            ctl.tilt = -cmd.u[CH_Y];
            ctl.roll = 0.0f;
        }
        else {
            ctl.pan_mode  = HAMFLY_DEFER;
            ctl.tilt_mode = HAMFLY_DEFER;
            ctl.roll_mode = HAMFLY_DEFER;
            ctl.pan = ctl.tilt = ctl.roll = 0.0f;
        }

        /* ----------------------------------------------------------------
         * TX at 10 Hz (CONTROL and NUDGE)
         * ---------------------------------------------------------------- */
        if ((app_mode==APP_MODE_CONTROL || app_mode==APP_MODE_NUDGE) &&
            joystick_cal_state()==JOY_CAL_OFF &&
            (g_tick_ms-last_tx_ms) >= CONTROL_PERIOD_MS) {
            last_tx_ms = g_tick_ms;
            hamfly_send_control(&g_movi, &ctl);
        }

        /* ----------------------------------------------------------------
         * 1 Hz diagnostic (CONTROL only)
         * ---------------------------------------------------------------- */
        if (diag_active && app_mode==APP_MODE_CONTROL &&
            joystick_cal_state()==JOY_CAL_OFF &&
            (g_tick_ms-last_diag_ms) >= DIAG_PERIOD_MS) {
            last_diag_ms = g_tick_ms;

            uint32_t ms = g_tick_ms;
            sprintf(tx,"  %02lu:%02lu:%02lu\r\n",
                    (unsigned long)(ms/3600000u),
                    (unsigned long)((ms/60000u)%60u),
                    (unsigned long)((ms/1000u)%60u));
            UART_1_PutString(tx);

            hamfly_statistics_t s;
            hamfly_get_statistics(&g_movi, &s);
            sprintf(tx,"[Serial] TX:%lu RX:%lu csum:%lu err:0x%02X\r\n",
                    (unsigned long)s.tx_packets,
                    (unsigned long)s.rx_packets,
                    (unsigned long)s.rx_bad_checksum,
                    (unsigned)s.uart_err_flags);
            UART_1_PutString(tx);

            hamfly_telemetry_t st;
            hamfly_get_telemetry(&g_movi, &st);
            if (st.valid) {
                char ps,ts,rs;
                int16_t pi,pf,ti,tf,ri,rf;
                compute_euler(&st,&ps,&pi,&pf,&ts,&ti,&tf,&rs,&ri,&rf);
                int16_t vli=(int16_t)st.battery_left_v;
                int16_t vlf=(int16_t)((st.battery_left_v-(float)vli)*10.0f);
                int16_t vri=(int16_t)st.battery_right_v;
                int16_t vrf=(int16_t)((st.battery_right_v-(float)vri)*10.0f);
                uint8_t imu_err=(st.gimbal_status1>>6)&0x01u;
                uint8_t drv_err=(st.gimbal_status1>>4)&0x01u;
                uint8_t gbl_err=(st.gimbal_status2>>4)&0x01u;
                if (abs_preset_active) {
                    char aps,ats; int16_t api,apf,ati,atf;
                    decomp(UNIT_TO_DEG(abs_pan_target), &api,&apf,&aps);
                    decomp(UNIT_TO_DEG(abs_tilt_target),&ati,&atf,&ats);
                    sprintf(tx,"[Target] Pan=%c%d.%02d Tilt=%c%d.%02d deg\r\n",
                            aps,api,apf,ats,ati,atf);
                    UART_1_PutString(tx);
                }
                sprintf(tx,
                    "[Gimbal] Pan=%c%d.%02d Tilt=%c%d.%02d Roll=%c%d.%02d"
                    " L=%d.%dV R=%d.%dV IMU:%d DRV:%d ERR:%d\r\n",
                    ps,pi,pf, ts,ti,tf, rs,ri,rf,
                    vli,vlf, vri,vrf,
                    imu_err, drv_err, gbl_err);
                UART_1_PutString(tx);
            } else {
                UART_1_PutString("[Gimbal] No valid frame\r\n");
            }

            if (show_platform_yaw && ctrl_attr_pending==0u) {
                hamfly_request_attr(&g_movi, 2u);
                g_movi.pending_sent_ms = g_tick_ms;
                ctrl_attr_pending = 2u;
            }

            /* Handle pending platform yaw response */
            if (ctrl_attr_pending != 0u && g_movi.pending_ready &&
                g_movi.pending_response_attr == ctrl_attr_pending) {
                uint32_t dt = g_tick_ms - g_movi.pending_sent_ms;
                print_query_response(dt);
                g_movi.pending_ready = false;
                g_movi.pending_attr  = 0u;
                ctrl_attr_pending    = 0u;
            }
            else if (ctrl_attr_pending != 0u &&
                     (g_tick_ms-g_movi.pending_sent_ms) >= QUERY_TIMEOUT_MS) {
                sprintf(tx,"[CTRL] attr %u timeout\r\n",
                        (unsigned)ctrl_attr_pending);
                UART_1_PutString(tx);
                g_movi.pending_attr = 0u;
                ctrl_attr_pending   = 0u;
            }

            char lrs,uds; int16_t lri,lrf,udi,udf;
            decomp(cmd.u[CH_X],&lri,&lrf,&lrs);
            decomp(cmd.u[CH_Y],&udi,&udf,&uds);
            sprintf(tx,"[Joy] L/R=%c%d.%02d U/D=%c%d.%02d (%s)\r\n",
                    lrs,lri,lrf, uds,udi,udf, mode_str(ctrl_mode));
            UART_1_PutString(tx);

            s.uart_err_flags = 0u;
        }
    }
}

/* [] END OF FILE */