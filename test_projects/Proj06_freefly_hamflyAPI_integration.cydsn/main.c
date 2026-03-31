/* main.c
 * Author: Hamish Johnson
 * Org: Quantum Internet Systems Lab, SFU, Physics
 * Date: March 2026
 *
 * Uses HamflyAPI (hamfly.h) for all gimbal communication.
 *
 * Modes:
 *   STANDBY      passive, pump runs, 1Hz status if valid
 *   CONTROL      QX277 at 10Hz, joystick or presets
 *   TEST         one-shot attr read, verbose print or timeout
 *   SCAN         exhaustive attr sweep
 *   RESET        attr 382 heading reset sequence
 *   SENSOR       continuous compact sensor stream
 *
 * Keys — STANDBY:
 *   m  toggle STANDBY <-> CONTROL
 *   t  enter TEST      w  enter SCAN
 *   r  enter RESET     v  enter SENSOR
 *   j  toggle joystick l  cycle DEFER/RATE/ABS
 *   k  kill + STANDBY  x  kill + quiet
 *   s  toggle ADC stream  c  calibration
 *
 * Keys — CONTROL (plus all STANDBY keys):
 *   f  attr 48 one-shot  p  platform yaw toggle
 *   0  home  1  pan+45   2  pan-45
 *   3  tilt+30  4  tilt-30  5  pan+90
 *
 * Keys — TEST:
 *   a  attr 22 euler    g  attr 4 GPS
 *   b  attr 3 baro      s  attr 1 sysstat
 *   n  attr 12 mag      2  attr 2 platform att
 *   4  attr 48 motor    q  quat crosscheck
 *   t  exit             k/x  kill + exit
 *
 * Keys — RESET:  x/k abort
 * Keys — SENSOR: any key exits
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
 * Constants
 * ========================================================================= */
#define TX_BUF_SIZE       160u
#define CONTROL_PERIOD_MS 100u
#define DIAG_PERIOD_MS    1000u
#define TEST_TIMEOUT_MS   2000u
#define SCAN_TIMEOUT_MS    500u
#define SCAN_BATCH_SIZE    200u
#define SCAN_ATTR_MIN      801u
#define SCAN_ATTR_MAX     2400u
#define SCAN_TX_BUF_SIZE   16u
#define RESET_POLL_MS     1000u
#define RESET_TIMEOUT_MS 15000u
#define SENSOR_POLL_MS     600u
#define SENSOR_TIMEOUT_MS 1500u
#define SENSOR_N_ATTRS      4u

#define DEG_TO_UNIT(d)  ((float)(d) / 180.0f)
#define FALSE  0u
#define TRUE   1u
#define CH_X   0u
#define CH_Y   1u

/* =========================================================================
 * App state
 * ========================================================================= */
typedef enum {
    APP_MODE_STANDBY = 0,
    APP_MODE_CONTROL,
    APP_MODE_TEST,
    APP_MODE_SCAN,
    APP_MODE_RESET,
    APP_MODE_SENSOR
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
 * Test verbose printers
 * ========================================================================= */
static void print_att(const uint8_t *p, uint8_t plen, uint32_t dt)
{
    char buf[TX_BUF_SIZE];
    sprintf(buf, "\r\n[TEST][Att] dt=%lums snaplen=%u\r\n",
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
    sprintf(buf, "\r\n[TEST][GPS] dt=%lums snaplen=%u\r\n",
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
    sprintf(buf, "\r\n[TEST][Baro] dt=%lums snaplen=%u\r\n",
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
    sprintf(buf, "\r\n[TEST][Stat] dt=%lums snaplen=%u\r\n",
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
    sprintf(buf, "\r\n[TEST][Mag] dt=%lums snaplen=%u\r\n",
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
    sprintf(buf, "\r\n[TEST][Plat] dt=%lums snaplen=%u\r\n",
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
    sprintf(buf, "\r\n[TEST][Drv] dt=%lums snaplen=%u\r\n",
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

static void print_test_response(uint32_t dt)
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
            sprintf(buf, "[TEST] attr=%u -- no printer\r\n", (unsigned)attr);
            UART_1_PutString(buf);
        }
    }
}

/* =========================================================================
 * Sensor mode compact printers
 * ========================================================================= */
static void print_sensor_response(void)
{
    uint16_t attr = g_movi.pending_response_attr;
    const uint8_t *p = g_movi.pending_payload;
    uint8_t plen = g_movi.pending_payload_len;
    char buf[TX_BUF_SIZE];
    char s; int16_t vi, vf;

    switch (attr) {
        case 12u: {
            if (plen < 6u) { UART_1_PutString("[MAG] short\r\n"); break; }
            int16_t x=SNAP16BE(p,plen,0);
            int16_t y=SNAP16BE(p,plen,2);
            int16_t z=SNAP16BE(p,plen,4);
            float hdg=atan2f((float)y,(float)x)*57.2958f;
            if (hdg<0.0f) hdg+=360.0f;
            decomp(hdg,&vi,&vf,&s);
            sprintf(buf,"[MAG] X=%6d Y=%6d Z=%6d hdg~=%d.%02d deg\r\n",
                    (int)x,(int)y,(int)z,(int)vi,(int)vf);
            UART_1_PutString(buf);
            break;
        }
        case 3u: {
            if (plen < 5u) { UART_1_PutString("[BARO] short\r\n"); break; }
            int16_t alt=SNAP16(p,plen,1), roc=SNAP16(p,plen,3);
            decomp((float)alt/10.0f,&vi,&vf,&s);
            sprintf(buf,"[BARO] alt=%c%d.%02d m  ",s,vi,vf);
            UART_1_PutString(buf);
            decomp((float)roc/100.0f,&vi,&vf,&s);
            sprintf(buf,"roc=%c%d.%02d m/s\r\n",s,vi,vf);
            UART_1_PutString(buf);
            break;
        }
        case 1u: {
            if (plen < 4u) { UART_1_PutString("[STAT] short\r\n"); break; }
            int16_t bv=SNAP16BE(p,plen,0);
            decomp((float)bv/100.0f,&vi,&vf,&s);
            sprintf(buf,"[STAT] batt=%c%d.%02d V  sats=%u  temp=%d C\r\n",
                    s,vi,vf,p[2],p[3]/3u);
            UART_1_PutString(buf);
            break;
        }
        case 2u: {
            if (plen < 7u) { UART_1_PutString("[PLAT] short\r\n"); break; }
            int16_t yaw=SNAP16BE(p,plen,5);
            decomp((float)yaw/100.0f,&vi,&vf,&s);
            sprintf(buf,"[PLAT] yaw=%c%d.%02d deg\r\n",s,vi,vf);
            UART_1_PutString(buf);
            break;
        }
        default: break;
    }
}

/* =========================================================================
 * Kill helper
 * ========================================================================= */
static void do_kill(app_mode_t *mode, uint8_t *joy_active,
                    uint8_t *streaming, hamfly_control_mode_t *ctrl_mode)
{
    if (*mode == APP_MODE_CONTROL) {
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

    UART_1_PutString("\r\n[TEST3] Quat vs Euler crosscheck\r\n");
    if (!st.valid) {
        UART_1_PutString("[TEST3] No valid 287 yet -- enter CONTROL first\r\n");
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
    UART_1_PutString("[TEST3] Requesting attr 22 for direct Euler...\r\n");
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
 * main
 * ========================================================================= */
int main(void)
{
    char tx[TX_BUF_SIZE];

    app_mode_t            app_mode   = APP_MODE_STANDBY;
    hamfly_control_mode_t ctrl_mode  = HAMFLY_DEFER;
    hamfly_control_t      ctl;

    uint8_t joystick_active   = FALSE;
    uint8_t streaming_adc     = FALSE;
    uint8_t diag_active       = FALSE;
    uint8_t show_platform_yaw = FALSE;
    uint8_t ctrl_attr_pending = 0u;

    float abs_pan_target    = 0.0f;
    float abs_tilt_target   = 0.0f;
    uint8_t abs_preset_active = FALSE;

    uint32_t last_tx_ms   = 0u;
    uint32_t last_diag_ms = 0u;

    /* RESET state */
    typedef enum {
        RESET_IDLE=0, RESET_SENT, RESET_ACTIVE, RESET_DONE
    } reset_state_t;
    reset_state_t reset_state   = RESET_IDLE;
    uint32_t      reset_poll_ms = 0u;
    uint32_t      reset_start_ms = 0u;
    uint8_t       reset_saw_01  = FALSE;

    /* SENSOR state */
    static const uint16_t sensor_attrs[SENSOR_N_ATTRS] = {12u,3u,1u,2u};
    uint8_t  sensor_idx        = 0u;
    uint32_t sensor_sent_ms    = 0u;
    uint8_t  sensor_tx_pending = FALSE;

    /* SCAN state */
    uint16_t scan_next_attr    = SCAN_ATTR_MIN;
    uint16_t scan_batch_start  = SCAN_ATTR_MIN;
    uint8_t  scan_waiting      = FALSE;
    uint8_t  scan_tx_pending   = FALSE;
    uint32_t scan_sent_ms      = 0u;
    uint8_t  scan_tx_buf[SCAN_TX_BUF_SIZE];
    uint8_t  scan_tx_len       = 0u;

    int16_t   counts[N_CH];
    joy_cmd_t cmd;
    cmd.u[0] = cmd.u[1] = 0.0f;

    joy_cal_t defaults;
    uint8_t i;
    for (i=0u; i<(uint8_t)N_CH; i++) {
        defaults.minv[i]=0; defaults.maxv[i]=255; defaults.center[i]=128;
    }
    defaults.valid = 0u;

    joy_cal_state_t cal_prev = JOY_CAL_OFF;
    joy_sensitivity_t current_sense = SENSE_MED;
    uint8_t last_button = 1u;
    uint32_t invert_mask = (1u << CH_X);

    /* ------------------------------------------------------------------
     * Hardware init
     * ------------------------------------------------------------------ */
    CyGlobalIntEnable;

    UART_1_Start();
    UART_1_PutString("\r\n=== HamflyAPI  PSoC 5LP ===\r\n");
    UART_1_PutString("STANDBY: m=ctrl t=test w=scan r=reset v=sensor\r\n");
    UART_1_PutString("         j=joy l=mode s=adc c=cal k=kill x=quiet\r\n");
    UART_1_PutString("CONTROL: f=attr48 p=plat-yaw 0-5=presets\r\n");
    UART_1_PutString("TEST:    a=att g=GPS b=baro s=stat n=mag\r\n");
    UART_1_PutString("         2=plat 4=motor q=quat t=exit\r\n");

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

        /* --- STANDBY / CONTROL shared keys --- */
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
            else if ((ch=='t'||ch=='T') && app_mode==APP_MODE_STANDBY) {
                app_mode = APP_MODE_TEST;
                UART_1_PutString("[CLI] -> TEST\r\n");
            }
            else if ((ch=='r'||ch=='R') && app_mode==APP_MODE_STANDBY) {
                app_mode       = APP_MODE_RESET;
                reset_state    = RESET_SENT;
                reset_saw_01   = FALSE;
                reset_start_ms = g_tick_ms;
                reset_poll_ms  = g_tick_ms;
                UART_1_PutString("[RESET] Writing attr 382=0x01\r\n");
                hamfly_write_attr_u8(&g_movi, 382u, 0x01u);
                hamfly_request_attr(&g_movi, 382u);
                g_movi.pending_sent_ms = g_tick_ms;
            }
            else if ((ch=='v'||ch=='V') && app_mode==APP_MODE_STANDBY) {
                app_mode = APP_MODE_SENSOR;
                sensor_idx = 0u; sensor_tx_pending = FALSE;
                UART_1_PutString("[SENSOR] MAG BARO STAT PLAT -- any key exits\r\n");
            }
            else if ((ch=='w'||ch=='W') && app_mode==APP_MODE_STANDBY) {
                app_mode = APP_MODE_SCAN;
                scan_next_attr = SCAN_ATTR_MIN;
                scan_batch_start = SCAN_ATTR_MIN;
                scan_waiting = FALSE; scan_tx_pending = FALSE;
                UART_1_PutString("[SCAN] attr,dt_ms,TX,RX -- any key=next batch\r\n");
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

            /* CONTROL-only keys */
            if (app_mode == APP_MODE_CONTROL) {
                if (ch=='f'||ch=='F') {
                    hamfly_request_attr(&g_movi, 48u);
                    g_movi.pending_sent_ms = g_tick_ms;
                    ctrl_attr_pending = 48u;
                    UART_1_PutString("[CTRL] Attr 48 requested\r\n");
                }
                else if (ch=='p'||ch=='P') {
                    show_platform_yaw = !show_platform_yaw;
                    UART_1_PutString(show_platform_yaw
                        ? "[CTRL] Platform yaw ON\r\n"
                        : "[CTRL] Platform yaw OFF\r\n");
                }
                else if (ch>='0' && ch<='5') {
                    if (ctrl_mode != HAMFLY_ABSOLUTE) {
                        UART_1_PutString("[CTRL] Need ABS mode (l)\r\n");
                    } else {
                        switch (ch) {
                            case '0':
                                abs_pan_target=0.0f; abs_tilt_target=0.0f;
                                UART_1_PutString("[CTRL] HOME\r\n"); break;
                            case '1':
                                abs_pan_target=DEG_TO_UNIT(45.0f);
                                abs_tilt_target=0.0f;
                                UART_1_PutString("[CTRL] Pan+45\r\n"); break;
                            case '2':
                                abs_pan_target=DEG_TO_UNIT(-45.0f);
                                abs_tilt_target=0.0f;
                                UART_1_PutString("[CTRL] Pan-45\r\n"); break;
                            case '3':
                                abs_pan_target=0.0f;
                                abs_tilt_target=DEG_TO_UNIT(30.0f);
                                UART_1_PutString("[CTRL] Tilt+30\r\n"); break;
                            case '4':
                                abs_pan_target=0.0f;
                                abs_tilt_target=DEG_TO_UNIT(-30.0f);
                                UART_1_PutString("[CTRL] Tilt-30\r\n"); break;
                            case '5':
                                abs_pan_target=DEG_TO_UNIT(90.0f);
                                abs_tilt_target=0.0f;
                                UART_1_PutString("[CTRL] Pan+90\r\n"); break;
                            default: break;
                        }
                        abs_preset_active = TRUE;
                        joystick_active   = FALSE;
                    }
                }
            }
        }

        /* --- SCAN keys --- */
        else if (app_mode == APP_MODE_SCAN) {
            if (ch != 0u) {
                if (ch=='x'||ch=='X'||ch=='k'||ch=='K') {
                    scan_tx_pending = FALSE; scan_waiting = FALSE;
                    g_movi.pending_attr  = 0u;
                    g_movi.pending_ready = false;
                    app_mode = APP_MODE_STANDBY;
                    UART_1_PutString("[SCAN] Aborted\r\n");
                } else if (scan_waiting) {
                    scan_waiting = FALSE;
                }
            }
        }

        /* --- TEST keys --- */
        else if (app_mode == APP_MODE_TEST) {
            if (ch=='t'||ch=='T') {
                g_movi.pending_attr  = 0u;
                g_movi.pending_ready = false;
                app_mode = APP_MODE_STANDBY;
                UART_1_PutString("[TEST] -> STANDBY\r\n");
            }
            else if (ch=='k'||ch=='K'||ch=='x'||ch=='X') {
                g_movi.pending_attr  = 0u;
                g_movi.pending_ready = false;
                do_kill(&app_mode,&joystick_active,&streaming_adc,&ctrl_mode);
            }
            else {
                uint16_t req = 0u;
                if      (ch=='a'||ch=='A') { req=22u; UART_1_PutString("[TEST] attr 22\r\n"); }
                else if (ch=='g'||ch=='G') { req=4u;  UART_1_PutString("[TEST] attr 4\r\n"); }
                else if (ch=='b'||ch=='B') { req=3u;  UART_1_PutString("[TEST] attr 3\r\n"); }
                else if (ch=='s'||ch=='S') { req=1u;  UART_1_PutString("[TEST] attr 1\r\n"); }
                else if (ch=='n'||ch=='N') { req=12u; UART_1_PutString("[TEST] attr 12\r\n");}
                else if (ch=='2')          { req=2u;  UART_1_PutString("[TEST] attr 2\r\n"); }
                else if (ch=='4')          { req=48u; UART_1_PutString("[TEST] attr 48\r\n");}
                else if (ch=='q'||ch=='Q') { print_quat_crosscheck(); }
                if (req != 0u && g_movi.pending_attr == 0u) {
                    hamfly_request_attr(&g_movi, req);
                    g_movi.pending_sent_ms = g_tick_ms;
                }
            }
        }

        /* --- RESET keys --- */
        else if (app_mode == APP_MODE_RESET) {
            if (ch=='x'||ch=='X'||ch=='k'||ch=='K') {
                g_movi.pending_attr  = 0u;
                g_movi.pending_ready = false;
                reset_state = RESET_IDLE;
                app_mode    = APP_MODE_STANDBY;
                UART_1_PutString("[RESET] Aborted\r\n");
            }
        }

        /* --- SENSOR keys --- */
        else if (app_mode == APP_MODE_SENSOR) {
            if (ch != 0u) {
                g_movi.pending_attr  = 0u;
                g_movi.pending_ready = false;
                sensor_tx_pending = FALSE;
                app_mode = APP_MODE_STANDBY;
                UART_1_PutString("[SENSOR] -> STANDBY\r\n");
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
         * TEST: response or timeout
         * ---------------------------------------------------------------- */
        if (app_mode == APP_MODE_TEST) {
            if (g_movi.pending_ready) {
                uint32_t dt = g_tick_ms - g_movi.pending_sent_ms;
                print_test_response(dt);
                g_movi.pending_ready = false;
                g_movi.pending_attr  = 0u;
            }
            else if (g_movi.pending_attr != 0u &&
                     (g_tick_ms - g_movi.pending_sent_ms) >= TEST_TIMEOUT_MS) {
                sprintf(tx,"[TEST] TIMEOUT attr=%u\r\n",
                        (unsigned)g_movi.pending_attr);
                UART_1_PutString(tx);
                g_movi.pending_attr = 0u;
            }
        }

        /* ----------------------------------------------------------------
         * SCAN
         * ---------------------------------------------------------------- */
        if (app_mode == APP_MODE_SCAN) {
            if (!scan_waiting) {
                if (scan_tx_pending && g_movi.pending_ready) {
                    uint32_t dt = g_tick_ms - scan_sent_ms;
                    char *bp = tx;
                    bp += sprintf(bp, "%u,%lu,",
                            (unsigned)g_movi.pending_response_attr,
                            (unsigned long)dt);
                    for (i=0u; i<scan_tx_len; i++)
                        bp += sprintf(bp,"%02X",(unsigned)scan_tx_buf[i]);
                    bp += sprintf(bp,",");
                    for (i=0u; i<g_movi.pending_payload_len; i++)
                        bp += sprintf(bp,"%02X",(unsigned)g_movi.pending_payload[i]);
                    bp += sprintf(bp,"\r\n");
                    UART_1_PutString(tx);
                    g_movi.pending_ready = false;
                    g_movi.pending_attr  = 0u;
                    scan_tx_pending = FALSE;
                    scan_next_attr++;
                    if (scan_next_attr==scan_batch_start+SCAN_BATCH_SIZE
                        || scan_next_attr>SCAN_ATTR_MAX) {
                        UART_1_PutString("=========\r\n");
                        if (scan_next_attr > SCAN_ATTR_MAX) {
                            UART_1_PutString("[SCAN] Complete\r\n");
                            app_mode = APP_MODE_STANDBY;
                        } else {
                            scan_batch_start = scan_next_attr;
                            scan_waiting = TRUE;
                            UART_1_PutString("[SCAN] any key for next batch\r\n");
                        }
                    }
                }
                else if (scan_tx_pending &&
                         (g_tick_ms-scan_sent_ms) >= SCAN_TIMEOUT_MS) {
                    char *bp = tx;
                    bp += sprintf(bp,"%u,%u,",
                            (unsigned)scan_next_attr,(unsigned)SCAN_TIMEOUT_MS);
                    for (i=0u; i<scan_tx_len; i++)
                        bp += sprintf(bp,"%02X",(unsigned)scan_tx_buf[i]);
                    bp += sprintf(bp,",\r\n");
                    UART_1_PutString(tx);
                    g_movi.pending_attr = 0u;
                    scan_tx_pending = FALSE;
                    scan_next_attr++;
                    if (scan_next_attr==scan_batch_start+SCAN_BATCH_SIZE
                        || scan_next_attr>SCAN_ATTR_MAX) {
                        UART_1_PutString("=========\r\n");
                        if (scan_next_attr > SCAN_ATTR_MAX) {
                            UART_1_PutString("[SCAN] Complete\r\n");
                            app_mode = APP_MODE_STANDBY;
                        } else {
                            scan_batch_start = scan_next_attr;
                            scan_waiting = TRUE;
                            UART_1_PutString("[SCAN] any key for next batch\r\n");
                        }
                    }
                }
                else if (!scan_tx_pending && scan_next_attr<=SCAN_ATTR_MAX) {
                    scan_tx_len = 0u;
                    hamfly_request_attr_capture(&g_movi, scan_next_attr,
                        scan_tx_buf, SCAN_TX_BUF_SIZE, &scan_tx_len);
                    scan_sent_ms = g_tick_ms;
                    g_movi.pending_sent_ms = g_tick_ms;
                    scan_tx_pending = TRUE;
                }
            }
        }

        /* ----------------------------------------------------------------
         * RESET state machine
         * ---------------------------------------------------------------- */
        if (app_mode == APP_MODE_RESET) {
            if (g_movi.pending_ready &&
                g_movi.pending_response_attr == 382u) {
                uint8_t echo = (g_movi.pending_payload_len > 0u)
                               ? g_movi.pending_payload[0] : 0xFFu;
                g_movi.pending_ready = false;
                g_movi.pending_attr  = 0u;
                if (echo == 0x01u) {
                    reset_saw_01 = TRUE;
                    UART_1_PutString("[RESET] 0x01 -- resetting...\r\n");
                    reset_state = RESET_ACTIVE;
                } else if (echo==0x00u && reset_saw_01) {
                    UART_1_PutString("[RESET] COMPLETE -- heading set\r\n");
                    reset_state = RESET_DONE;
                    app_mode    = APP_MODE_STANDBY;
                } else {
                    UART_1_PutString("[RESET] Waiting for 0x01...\r\n");
                }
            }
            if (reset_state != RESET_DONE &&
                (g_tick_ms-reset_poll_ms) >= RESET_POLL_MS &&
                g_movi.pending_attr == 0u) {
                reset_poll_ms = g_tick_ms;
                hamfly_request_attr(&g_movi, 382u);
                g_movi.pending_sent_ms = g_tick_ms;
            }
            if ((g_tick_ms-reset_start_ms) >= RESET_TIMEOUT_MS) {
                UART_1_PutString("[RESET] TIMEOUT\r\n");
                g_movi.pending_attr  = 0u;
                g_movi.pending_ready = false;
                reset_state = RESET_IDLE;
                app_mode    = APP_MODE_STANDBY;
            }
        }

        /* ----------------------------------------------------------------
         * SENSOR
         * ---------------------------------------------------------------- */
        if (app_mode == APP_MODE_SENSOR) {
            if (sensor_tx_pending && g_movi.pending_ready) {
                print_sensor_response();
                g_movi.pending_ready = false;
                g_movi.pending_attr  = 0u;
                sensor_tx_pending    = FALSE;
                sensor_idx = (uint8_t)((sensor_idx+1u) % SENSOR_N_ATTRS);
            }
            else if (sensor_tx_pending &&
                     (g_tick_ms-sensor_sent_ms) >= SENSOR_TIMEOUT_MS) {
                sprintf(tx,"[SENSOR] timeout attr=%u\r\n",
                        (unsigned)sensor_attrs[sensor_idx]);
                UART_1_PutString(tx);
                g_movi.pending_attr = 0u;
                sensor_tx_pending   = FALSE;
                sensor_idx = (uint8_t)((sensor_idx+1u) % SENSOR_N_ATTRS);
            }
            else if (!sensor_tx_pending &&
                     (g_tick_ms-sensor_sent_ms) >= SENSOR_POLL_MS) {
                hamfly_request_attr(&g_movi, sensor_attrs[sensor_idx]);
                g_movi.pending_sent_ms = g_tick_ms;
                sensor_sent_ms = g_tick_ms;
                sensor_tx_pending = TRUE;
            }
        }

        /* ----------------------------------------------------------------
         * CONTROL: on-demand attr reads (f, p keys)
         * ---------------------------------------------------------------- */
        if (app_mode == APP_MODE_CONTROL) {
            if (ctrl_attr_pending != 0u && g_movi.pending_ready &&
                g_movi.pending_response_attr == ctrl_attr_pending) {
                uint32_t dt = g_tick_ms - g_movi.pending_sent_ms;
                print_test_response(dt);
                g_movi.pending_ready = false;
                g_movi.pending_attr  = 0u;
                ctrl_attr_pending    = 0u;
            }
            else if (ctrl_attr_pending != 0u &&
                     (g_tick_ms-g_movi.pending_sent_ms) >= TEST_TIMEOUT_MS) {
                sprintf(tx,"[CTRL] attr %u timeout\r\n",
                        (unsigned)ctrl_attr_pending);
                UART_1_PutString(tx);
                g_movi.pending_attr = 0u;
                ctrl_attr_pending   = 0u;
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
         * Build control
         * ---------------------------------------------------------------- */
        ctl.enable = 1u;
        ctl.kill   = 0u;
        if (abs_preset_active && ctrl_mode==HAMFLY_ABSOLUTE) {
            ctl.pan_mode=HAMFLY_ABSOLUTE; ctl.tilt_mode=HAMFLY_ABSOLUTE;
            ctl.roll_mode=HAMFLY_DEFER;
            ctl.pan=abs_pan_target; ctl.tilt=abs_tilt_target; ctl.roll=0.0f;
        }
        else if (joystick_active && joystick_cal_state()==JOY_CAL_OFF) {
            ctl.pan_mode=ctrl_mode; ctl.tilt_mode=ctrl_mode;
            ctl.roll_mode=HAMFLY_DEFER;
            ctl.pan= cmd.u[CH_X]; ctl.tilt=-cmd.u[CH_Y]; ctl.roll=0.0f;
        }
        else {
            ctl.pan_mode=HAMFLY_DEFER; ctl.tilt_mode=HAMFLY_DEFER;
            ctl.roll_mode=HAMFLY_DEFER;
            ctl.pan=ctl.tilt=ctl.roll=0.0f;
        }

        /* ----------------------------------------------------------------
         * TX at 10 Hz (CONTROL only)
         * ---------------------------------------------------------------- */
        if (app_mode==APP_MODE_CONTROL &&
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
                    decomp(abs_pan_target*180.0f,&api,&apf,&aps);
                    decomp(abs_tilt_target*180.0f,&ati,&atf,&ats);
                    sprintf(tx,"[Target] Pan=%c%d.%02d Tilt=%c%d.%02d\r\n",
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