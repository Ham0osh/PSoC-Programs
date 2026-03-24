/* main.c
 * Author: Hamish Johnson
 * Org: Quantum Internet Systems Lab, SFU, Physics
 * Date: March 2026
 *
 * Uses:
 *  - adc_balanced.c/.h  : round-robin ADC sampling over AMux (N_CH channels)
 *  - joystick.c/.h      : calibration + mapping (deadband, inversion, scaling)
 *  - movi_comm.c/.h     : MōVI Pro QX protocol HAL
 *
 * Application modes (app_mode):
 *   STANDBY      : passive — pump() runs, diag prints [Status] at 1 Hz if valid
 *   MOVI_CONTROL : sends QX277 at 10 Hz, 1 Hz diag w/ [Serial][Status][Joystick]
 *   MOVI_TEST    : one-shot query mode — sends single attr read, prints verbose
 *                  response or 2-second timeout.  No periodic TX or diag block.
 *   MOVI_SCAN    : exhaustive attr sweep (existing)
 *   RESET        : attr 382 heading-reset sequence (test 1)
 *   SENSOR       : continuous compact sensor stream, no GPS needed
 *
 * CLI keys — STANDBY:
 *   m : toggle STANDBY <-> MOVI_CONTROL
 *   t : enter MOVI_TEST mode
 *   w : enter MOVI_SCAN mode
 *   r : enter RESET mode  (test 1 — attr 382 heading reset)
 *   v : enter SENSOR mode (indoor sensor validation)
 *   j : toggle joystick active
 *   l : cycle control mode DEFER -> RATE -> ABSOLUTE -> DEFER
 *   k : kill motion + STANDBY      x : kill + STANDBY + quiet
 *   s : toggle raw ADC stream      c : calibration advance
 *
 * CLI keys — MOVI_CONTROL:
 *   (all STANDBY keys, plus:)
 *   f : one-shot attr 48 read  (test 4 — motor drive status)
 *   p : toggle platform-yaw display in 1 Hz diag  (M2 — attr 2)
 *   0..5 : absolute angle presets  (test 2 — requires 'l' to select ABS first)
 *          0=home(0,0)  1=pan+45  2=pan-45  3=tilt+30  4=tilt-30  5=pan+90
 *
 * CLI keys — MOVI_TEST:
 *   a : attr 22  — gimbal Euler attitude
 *   g : attr  4  — GPS
 *   b : attr  3  — baro altitude
 *   s : attr  1  — system status
 *   n : attr 12  — magnetometer
 *   2 : attr  2  — platform body attitude  (M2)
 *   4 : attr 48  — motor drive status      (test 4)
 *   q : quaternion cross-check (attr 287 quat + attr 22 Euler side-by-side) (test 3)
 *   t : exit test   k/x : kill + exit
 *
 * CLI keys — RESET mode:
 *   (automatic: polls attr 382 at 1 Hz, exits when GCU returns 0x00)
 *   x/k : abort reset, return to STANDBY
 *
 * CLI keys — SENSOR mode:
 *   any key : exit to STANDBY
 */

#include <project.h>
#include <stdio.h>
#include <math.h>

#include "adc_balanced.h"
#include "joystick.h"
#include "movi_comm.h"

/* =========================================================================
 * Constants
 * ========================================================================= */
#define TRANSMIT_BUFFER_SIZE  160u
#define MOVI_TX_PERIOD_MS     100u    /* 10 Hz control TX                      */
#define DIAG_PERIOD_MS        1000u   /* 1 Hz diagnostic print (MOVI_CONTROL)  */
#define TEST_TIMEOUT_MS       2000u   /* test mode: max wait for response       */
#define SCAN_TIMEOUT_MS        500u   /* scan mode: max wait per attr           */
#define SCAN_BATCH_SIZE        200u   /* attrs per batch before pausing         */
#define SCAN_ATTR_MIN          801u   /* first attr to scan                     */
#define SCAN_ATTR_MAX         2400u   /* last attr to scan (inclusive)          */
#define SCAN_TX_BUF_SIZE        16u   /* enough for any QX read-request frame   */

#define RESET_POLL_MS         1000u   /* poll attr 382 echo at 1 Hz             */
#define RESET_TIMEOUT_MS     15000u   /* abort if still active after 15 s       */
#define SENSOR_POLL_MS         600u   /* ms between sensor attr polls           */
#define SENSOR_TIMEOUT_MS     1500u   /* per-attr timeout in sensor mode        */

/* Number of attrs in sensor rotation */
#define SENSOR_N_ATTRS          4u

/* Absolute angle preset scale: value passed to movi_control is pan/tilt in
 * [-1, +1] where 1.0 = 180 deg.  Presets defined in degrees. */
#define DEG_TO_UNIT(d)  ((float)(d) / 180.0f)

#define FALSE  0
#define TRUE   1

#define CH_X (0u)  /* pan  */
#define CH_Y (1u)  /* tilt */

/* =========================================================================
 * App mode state machine
 * ========================================================================= */
typedef enum {
    APP_MODE_STANDBY      = 0,
    APP_MODE_MOVI_CONTROL = 1,
    APP_MODE_MOVI_TEST    = 2,
    APP_MODE_MOVI_SCAN    = 3,
    APP_MODE_RESET        = 4,   /* attr 382 heading-reset sequence (test 1)  */
    APP_MODE_SENSOR       = 5,   /* continuous indoor sensor stream            */
} app_mode_t;

/* =========================================================================
 * Hardware / globals
 * ========================================================================= */
static movi_comm_t g_movi;

static void movi_uart_putc(void* ctx, uint8_t b)
{
    (void)ctx;
    UART_MOVI_PutChar((char)b);
}

static const movi_hal_t MOVI_HAL = {
    .ctx       = NULL,
    .uart_putc = movi_uart_putc
};

volatile uint32 g_tick_ms = 0u;

CY_ISR(isr_rx_movi_Handler)
{
    uint8 st;
    do {
        st = UART_MOVI_RXSTATUS_REG;
        if (st & (UART_MOVI_RX_STS_BREAK     |
                  UART_MOVI_RX_STS_PAR_ERROR |
                  UART_MOVI_RX_STS_STOP_ERROR|
                  UART_MOVI_RX_STS_OVERRUN)) {
            movi_comm_on_uart_error_flags(&g_movi,
                (uint8)(st & (UART_MOVI_RX_STS_BREAK     |
                              UART_MOVI_RX_STS_PAR_ERROR |
                              UART_MOVI_RX_STS_STOP_ERROR|
                              UART_MOVI_RX_STS_OVERRUN)));
        }
        if (st & UART_MOVI_RX_STS_FIFO_NOTEMPTY) {
            uint8 b = UART_MOVI_RXDATA_REG;
            movi_comm_on_rx_byte(&g_movi, b);
        }
    } while (st & UART_MOVI_RX_STS_FIFO_NOTEMPTY);
}

CY_ISR(isr_Looptimer_Handler)
{
    g_tick_ms++;
    Looptimer_ReadStatusRegister();
}

/* =========================================================================
 * Low-level write helper
 * Sends a QX WRITE_ABS frame for any attr with 1-byte value.
 * Bypasses movi_comm to avoid needing a new API entry point.
 * Supports both single-byte attrs (< 128) and two-byte varint attrs.
 * ========================================================================= */
static void send_write_attr_u8(uint16_t attr, uint8_t value)
{
    uint8_t pkt[13];   /* worst case: 2B magic + 1B len + 2B varint + 5B hdr + 1B val + 1B cs */
    uint8_t idx = 0u;
    uint8_t i;

    pkt[idx++] = 0x51u;  /* QX magic */
    pkt[idx++] = 0x58u;

    if (attr < 128u) {
        pkt[idx++] = 0x07u;          /* length = 7: 1B varint+opt+src+tgt+trid+rrid+val */
        pkt[idx++] = (uint8_t)attr;  /* single-byte varint */
    } else {
        pkt[idx++] = 0x08u;                              /* length = 8 */
        pkt[idx++] = (uint8_t)((attr & 0x7Fu) | 0x80u); /* varint byte 1 */
        pkt[idx++] = (uint8_t)(attr >> 7u);              /* varint byte 2 */
    }
    pkt[idx++] = 0x02u;  /* OPTIONS = WRITE_ABS */
    pkt[idx++] = 0x0Au;  /* SOURCE  = QX_DEV_ID_MOVI_API_CONTROLLER */
    pkt[idx++] = 0x02u;  /* TARGET  = QX_DEV_ID_GIMBAL */
    pkt[idx++] = 0x00u;  /* TRID */
    pkt[idx++] = 0x00u;  /* RRID */
    pkt[idx++] = value;

    /* Checksum = 255 - SUM(bytes[3..idx-1]) mod 256 */
    uint16_t sum = 0u;
    for (i = 3u; i < idx; i++) sum += pkt[i];
    pkt[idx++] = (uint8_t)(255u - (sum & 0xFFu));

    for (i = 0u; i < idx; i++) {
        UART_MOVI_PutChar((char)pkt[i]);
    }
}

/* =========================================================================
 * General helpers
 * ========================================================================= */
static void decomp(float val, int16* i_out, int16* f_out, char* sign_out)
{
    *sign_out = (val < 0.0f) ? '-' : ' ';
    if (val < 0.0f) val = -val;
    *i_out = (int16)val;
    *f_out = (int16)((val - (float)*i_out) * 100.0f);
    if (*f_out < 0) *f_out = -*f_out;
}

static void compute_euler_ints(const movi_status_t* st,
                                char* pan_s,  int16* pan_i,  int16* pan_f,
                                char* tilt_s, int16* tilt_i, int16* tilt_f,
                                char* roll_s, int16* roll_i, int16* roll_f)
{
    float r  = st->gimbal_r;
    float ii = st->gimbal_i;
    float j  = st->gimbal_j;
    float k  = st->gimbal_k;

    float sinr_val = 2.0f * (r*ii + j*k);
    float cosr_val = 1.0f - 2.0f * (ii*ii + j*j);
    float roll_deg = atan2f(sinr_val, cosr_val) * 57.2958f;

    float sinp = 2.0f * (r*j - k*ii);
    float tilt_deg = (sinp >=  1.0f) ?  90.0f :
                     (sinp <= -1.0f) ? -90.0f :
                     asinf(sinp) * 57.2958f;

    float siny_val = 2.0f * (r*k + ii*j);
    float cosy_val = 1.0f - 2.0f * (j*j + k*k);
    float pan_deg  = atan2f(siny_val, cosy_val) * 57.2958f;

    decomp(pan_deg,  pan_i,  pan_f,  pan_s);
    decomp(tilt_deg, tilt_i, tilt_f, tilt_s);
    decomp(roll_deg, roll_i, roll_f, roll_s);
}

static const char* mode_to_str(ff_api_control_type_e m)
{
    switch (m) {
        case RATE:     return "RATE";
        case ABSOLUTE: return "ABS ";
        default:       return "DEFR";
    }
}

static void print_cal_prompt_on_state_change(joy_cal_state_t prev, joy_cal_state_t now)
{
    if (prev == now) return;
    if (now == JOY_CAL_RANGE_CAPTURE) {
        UART_1_PutString("\r\n[CAL] Step 1: Move joystick through FULL range, then release to center.\r\n");
        UART_1_PutString("[CAL] Press 'c' to proceed to center capture, 'x' to cancel.\r\n");
    } else if (now == JOY_CAL_CENTER_CAPTURE) {
        UART_1_PutString("\r\n[CAL] Step 2: Hold center -- averaging...\r\n");
    } else if (now == JOY_CAL_CONFIRM_SAVE) {
        UART_1_PutString("\r\n[CAL] Step 3: Center captured. Press 'c' to SAVE, 'x' to cancel.\r\n");
    } else if (now == JOY_CAL_OFF && prev != JOY_CAL_OFF) {
        UART_1_PutString("[CAL] Exited.\r\n");
    }
}

/* =========================================================================
 * Kill helper
 * ========================================================================= */
static void do_kill(movi_comm_t* movi, app_mode_t* mode,
                    uint8* joystick_active, uint8* streaming_adc,
                    ff_api_control_type_e* ctrl_mode)
{
    if (*mode == APP_MODE_MOVI_CONTROL) {
        movi_comm_kill(movi);
        UART_1_PutString("[CLI] Kill packet sent\r\n");
    }
    *joystick_active = FALSE;
    *mode            = APP_MODE_STANDBY;
    *streaming_adc   = FALSE;
    *ctrl_mode       = DEFER;
    joystick_on_key('x');
    UART_1_PutString("[CLI] All stopped  Mode: DEFR  -> STANDBY\r\n");
}

/* =========================================================================
 * Snapshot read macros (shared by test printers)
 * ========================================================================= */
#define SNAP16(p, plen, off) \
    (((off) + 2u <= (plen)) ? ({ int16_t _v; memcpy(&_v, (p)+(off), 2); _v; }) : (int16_t)0)

#define SNAP32(p, plen, off) \
    (((off) + 4u <= (plen)) ? ({ int32_t _v; memcpy(&_v, (p)+(off), 4); _v; }) : (int32_t)0)

#define SNAP16BE(p, plen, off) \
    (((off) + 2u <= (plen)) \
        ? (int16_t)(((uint16_t)(p)[(off)] << 8) | (uint16_t)(p)[(off)+1u]) \
        : (int16_t)0)

#define SNAP32BE(p, plen, off) \
    (((off) + 4u <= (plen)) \
        ? (int32_t)(  ((uint32_t)(p)[(off)]   << 24) \
                    | ((uint32_t)(p)[(off)+1u] << 16) \
                    | ((uint32_t)(p)[(off)+2u] <<  8) \
                    |  (uint32_t)(p)[(off)+3u]) \
        : (int32_t)0)

/* =========================================================================
 * Hex dump helper (shared by verbose printers)
 * ========================================================================= */
static void print_hex_dump(const uint8_t* p, uint8_t plen)
{
    char buf[TRANSMIT_BUFFER_SIZE];
    uint8_t row, col;
    for (row = 0u; row < plen; row += 8u) {
        char* bp = buf;
        bp += sprintf(bp, "  off%02u:", (unsigned)row);
        for (col = 0u; col < 8u && (row + col) < plen; col++)
            bp += sprintf(bp, " %02X", (unsigned)p[row + col]);
        bp += sprintf(bp, "\r\n");
        UART_1_PutString(buf);
    }
}

/* =========================================================================
 * TEST MODE verbose print functions
 * ========================================================================= */

/* --- attr 22: Gimbal Euler attitude --- */
static void print_test_attitude(const movi_status_t* st, uint32_t dt,
                                 uint16_t rxlen, const uint8_t* p, uint8_t plen)
{
    char buf[TRANSMIT_BUFFER_SIZE];
    (void)st;
    sprintf(buf, "\r\n[TEST][Att] attr=22  dt=%lums  rxlen=%u  snaplen=%u\r\n",
            (unsigned long)dt, (unsigned)rxlen, (unsigned)plen);
    UART_1_PutString(buf);
    print_hex_dump(p, plen);

    if (plen < 6u) { UART_1_PutString("  ERROR: too short\r\n"); return; }

    char s; int16 vi, vf;
    int16_t r1 = SNAP16(p, plen, 1);
    int16_t r3 = SNAP16(p, plen, 3);
    int16_t r5 = SNAP16(p, plen, 5);

    decomp((float)r1 / 100.0f, &vi, &vf, &s);
    sprintf(buf, "  off  1  int16LE /100  raw=%6d  =>  %c%d.%02d deg  [PITCH]\r\n",
            (int)r1, s, (int)vi, (int)vf);
    UART_1_PutString(buf);

    decomp((float)r3 / 100.0f, &vi, &vf, &s);
    sprintf(buf, "  off  3  int16LE /100  raw=%6d  =>  %c%d.%02d deg  [YAW]\r\n",
            (int)r3, s, (int)vi, (int)vf);
    UART_1_PutString(buf);

    decomp((float)r5 / 100.0f, &vi, &vf, &s);
    sprintf(buf, "  off  5  int16LE /100  raw=%6d  =>  %c%d.%02d deg  [ROLL]\r\n",
            (int)r5, s, (int)vi, (int)vf);
    UART_1_PutString(buf);

    if (plen >= 19u) {
        UART_1_PutString("  -- rates (off 9-17) --\r\n");
        int16_t rp = SNAP16(p, plen,  9);
        int16_t ry = SNAP16(p, plen, 11);
        int16_t rr = SNAP16(p, plen, 13);
        decomp((float)rp / 100.0f, &vi, &vf, &s);
        sprintf(buf, "  off  9  int16LE /100  raw=%6d  =>  %c%d.%02d dps  [PITCH rate]\r\n",
                (int)rp, s, (int)vi, (int)vf);
        UART_1_PutString(buf);
        decomp((float)ry / 100.0f, &vi, &vf, &s);
        sprintf(buf, "  off 11  int16LE /100  raw=%6d  =>  %c%d.%02d dps  [YAW rate]\r\n",
                (int)ry, s, (int)vi, (int)vf);
        UART_1_PutString(buf);
        decomp((float)rr / 100.0f, &vi, &vf, &s);
        sprintf(buf, "  off 13  int16LE /100  raw=%6d  =>  %c%d.%02d dps  [ROLL rate]\r\n",
                (int)rr, s, (int)vi, (int)vf);
        UART_1_PutString(buf);
    }
}

/* --- attr 4: GPS --- */
static void print_test_gps(const movi_status_t* st, uint32_t dt,
                            uint16_t rxlen, const uint8_t* p, uint8_t plen)
{
    char buf[TRANSMIT_BUFFER_SIZE];
    (void)st;
    sprintf(buf, "\r\n[TEST][GPS] attr=4  dt=%lums  rxlen=%u  snaplen=%u\r\n",
            (unsigned long)dt, (unsigned)rxlen, (unsigned)plen);
    UART_1_PutString(buf);
    print_hex_dump(p, plen);
    if (plen < 12u) { UART_1_PutString("  ERROR: too short\r\n"); return; }

    char s; int16 vi, vf;
    long i_part, f_part;
    char sign;

    int32_t lon_r = SNAP32BE(p, plen,  0);
    int32_t lat_r = SNAP32BE(p, plen,  4);
    int32_t alt_r = SNAP32BE(p, plen,  8);

    /* split helper inline for GPS coords */
    {
        int32_t abs_v = lon_r < 0 ? -lon_r : lon_r;
        sign = lon_r < 0 ? '-' : '+';
        i_part = (long)(abs_v / 10000000L);
        f_part = (long)(abs_v % 10000000L);
        sprintf(buf, "  off  0  int32BE /1e7   raw=%ld  =>  %c%ld.%07ld deg  [LON]\r\n",
                (long)lon_r, sign, i_part, f_part);
        UART_1_PutString(buf);
    }
    {
        int32_t abs_v = lat_r < 0 ? -lat_r : lat_r;
        sign = lat_r < 0 ? '-' : '+';
        i_part = (long)(abs_v / 10000000L);
        f_part = (long)(abs_v % 10000000L);
        sprintf(buf, "  off  4  int32BE /1e7   raw=%ld  =>  %c%ld.%07ld deg  [LAT]\r\n",
                (long)lat_r, sign, i_part, f_part);
        UART_1_PutString(buf);
    }
    {
        int32_t abs_v = alt_r < 0 ? -alt_r : alt_r;
        sign = alt_r < 0 ? '-' : '+';
        i_part = (long)(abs_v / 1000L);
        f_part = (long)(abs_v % 1000L);
        sprintf(buf, "  off  8  int32BE /1000  raw=%ld  =>  %c%ld.%03ld m  [ALT MSL]\r\n",
                (long)alt_r, sign, i_part, f_part);
        UART_1_PutString(buf);
    }

    if (plen >= 27u) {
        sprintf(buf, "  off 26  uint8         raw=%3u  =>  %u sats  [SAT CNT]\r\n",
                (unsigned)p[26], (unsigned)p[26]);
        UART_1_PutString(buf);
        sprintf(buf, "  off 27  uint8         raw=%3u  =>  fix type %u  [FIX]\r\n",
                (unsigned)p[27], (unsigned)p[27]);
        UART_1_PutString(buf);
    }
    (void)vi; (void)vf; (void)s;
}

/* --- attr 3: Baro --- */
static void print_test_baro(const movi_status_t* st, uint32_t dt,
                             uint16_t rxlen, const uint8_t* p, uint8_t plen)
{
    char buf[TRANSMIT_BUFFER_SIZE];
    (void)st;
    sprintf(buf, "\r\n[TEST][Baro] attr=3  dt=%lums  rxlen=%u  snaplen=%u\r\n",
            (unsigned long)dt, (unsigned)rxlen, (unsigned)plen);
    UART_1_PutString(buf);
    print_hex_dump(p, plen);
    if (plen < 4u) { UART_1_PutString("  ERROR: too short\r\n"); return; }

    char s; int16 vi, vf;
    int16_t alt_le = SNAP16(p, plen, 1);
    int16_t roc_le = SNAP16(p, plen, 3);

    decomp((float)alt_le / 10.0f, &vi, &vf, &s);
    sprintf(buf, "  off  1  int16LE /10   raw=%6d  =>  %c%d.%02d m  [ALT rel home]\r\n",
            (int)alt_le, s, (int)vi, (int)vf);
    UART_1_PutString(buf);
    decomp((float)roc_le / 100.0f, &vi, &vf, &s);
    sprintf(buf, "  off  3  int16LE /100  raw=%6d  =>  %c%d.%02d m/s  [ROC]\r\n",
            (int)roc_le, s, (int)vi, (int)vf);
    UART_1_PutString(buf);
}

/* --- attr 1: System status --- */
static void print_test_sysstat(const movi_status_t* st, uint32_t dt,
                                uint16_t rxlen, const uint8_t* p, uint8_t plen)
{
    char buf[TRANSMIT_BUFFER_SIZE];
    (void)st;
    sprintf(buf, "\r\n[TEST][Stat] attr=1  dt=%lums  rxlen=%u  snaplen=%u\r\n",
            (unsigned long)dt, (unsigned)rxlen, (unsigned)plen);
    UART_1_PutString(buf);
    print_hex_dump(p, plen);
    if (plen < 5u) { UART_1_PutString("  ERROR: too short\r\n"); return; }

    char s; int16 vi, vf;

    int16_t bv_r   = SNAP16BE(p, plen, 0);
    decomp((float)bv_r / 100.0f, &vi, &vf, &s);
    sprintf(buf, "  off  0  int16BE /100  raw=%6d  =>  %c%d.%02d V  [BATT V] *\r\n",
            (int)bv_r, s, (int)vi, (int)vf);
    UART_1_PutString(buf);

    sprintf(buf, "  off  2  uint8   /1    raw=%3u  =>  %u sats  [GPS SATS] *\r\n",
            (unsigned)p[2], (unsigned)p[2]);
    UART_1_PutString(buf);

    sprintf(buf, "  off  3  uint8   /3    raw=%3u  =>  %d.%02d C  [TEMP] *\r\n",
            (unsigned)p[3], (int)(p[3]/3u), (int)((p[3]%3u)*33u));
    UART_1_PutString(buf);

    if (plen >= 5u) {
        sprintf(buf, "  off  4  uint8   /1    raw=%3u  =>  %u%%  [CPU]\r\n",
                (unsigned)p[4], (unsigned)p[4]);
        UART_1_PutString(buf);
    }
    if (plen >= 7u) {
        sprintf(buf, "  off  5  int16BE flags raw=0x%04X  [STATUS FLAGS]\r\n",
                (unsigned)(uint16_t)SNAP16BE(p, plen, 5));
        UART_1_PutString(buf);
    }
    if (plen >= 11u) {
        int16_t pres_r = SNAP16BE(p, plen, 9);
        decomp((float)pres_r / 10.0f, &vi, &vf, &s);
        sprintf(buf, "  off  9  int16BE /10   raw=%6d  =>  %c%d.%02d mb  [PRESSURE] *\r\n",
                (int)pres_r, s, (int)vi, (int)vf);
        UART_1_PutString(buf);
    }
    if (plen >= 13u) {
        sprintf(buf, "  off 11  int16BE /1    raw=%6d  [IMU RATE Hz]\r\n",
                (int)SNAP16BE(p, plen, 11));
        UART_1_PutString(buf);
    }
    UART_1_PutString("  (* = confirmed against iOS app)\r\n");
}

/* --- attr 12: Magnetometer --- */
static void print_test_mag(const movi_status_t* st, uint32_t dt,
                            uint16_t rxlen, const uint8_t* p, uint8_t plen)
{
    char buf[TRANSMIT_BUFFER_SIZE];
    (void)st;
    sprintf(buf, "\r\n[TEST][Mag] attr=12  dt=%lums  rxlen=%u  snaplen=%u\r\n",
            (unsigned long)dt, (unsigned)rxlen, (unsigned)plen);
    UART_1_PutString(buf);
    print_hex_dump(p, plen);
    if (plen < 6u) { UART_1_PutString("  ERROR: too short\r\n"); return; }

    char s; int16 vi, vf;
    int16_t x_r  = SNAP16BE(p, plen, 0);
    int16_t y_r  = SNAP16BE(p, plen, 2);
    int16_t z_r  = SNAP16BE(p, plen, 4);

    decomp((float)x_r, &vi, &vf, &s);
    sprintf(buf, "  off  0  int16BE raw  raw=%6d  [MAG X raw]\r\n", (int)x_r);
    UART_1_PutString(buf);
    sprintf(buf, "  off  2  int16BE raw  raw=%6d  [MAG Y raw]\r\n", (int)y_r);
    UART_1_PutString(buf);
    sprintf(buf, "  off  4  int16BE raw  raw=%6d  [MAG Z raw]\r\n", (int)z_r);
    UART_1_PutString(buf);

    if (plen >= 14u) {
        int16_t fc1b = SNAP16BE(p, plen, 12);
        decomp((float)fc1b / 100.0f, &vi, &vf, &s);
        sprintf(buf, "  off 12  int16BE /100  raw=%6d  =>  %c%d.%02d deg  [DECL?]\r\n",
                (int)fc1b, s, (int)vi, (int)vf);
        UART_1_PutString(buf);
    }
    (void)vi; (void)vf; (void)s;
}

/* --- attr 2: Platform body attitude (TEST verbose) --- */
static void print_test_platform_att(const movi_status_t* st, uint32_t dt,
                                     uint16_t rxlen, const uint8_t* p, uint8_t plen)
{
    char buf[TRANSMIT_BUFFER_SIZE];
    (void)st;
    sprintf(buf, "\r\n[TEST][Plat] attr=2  dt=%lums  rxlen=%u  snaplen=%u\r\n",
            (unsigned long)dt, (unsigned)rxlen, (unsigned)plen);
    UART_1_PutString(buf);
    print_hex_dump(p, plen);
    if (plen < 7u) { UART_1_PutString("  ERROR: too short\r\n"); return; }

    char s; int16 vi, vf;
    /* Confirmed layout: off 0 = sub-index 0x00, then int16 BE /100 for roll/pitch/yaw */
    int16_t roll_r  = SNAP16BE(p, plen, 1);
    int16_t pitch_r = SNAP16BE(p, plen, 3);
    int16_t yaw_r   = SNAP16BE(p, plen, 5);

    decomp((float)roll_r  / 100.0f, &vi, &vf, &s);
    sprintf(buf, "  off  1  int16BE /100  raw=%6d  =>  %c%d.%02d deg  [ROLL  platform body]\r\n",
            (int)roll_r, s, (int)vi, (int)vf);
    UART_1_PutString(buf);

    decomp((float)pitch_r / 100.0f, &vi, &vf, &s);
    sprintf(buf, "  off  3  int16BE /100  raw=%6d  =>  %c%d.%02d deg  [PITCH platform body]\r\n",
            (int)pitch_r, s, (int)vi, (int)vf);
    UART_1_PutString(buf);

    decomp((float)yaw_r   / 100.0f, &vi, &vf, &s);
    sprintf(buf, "  off  5  int16BE /100  raw=%6d  =>  %c%d.%02d deg  [YAW   platform body] *** geo-pointing reference ***\r\n",
            (int)yaw_r, s, (int)vi, (int)vf);
    UART_1_PutString(buf);

    UART_1_PutString("  NOTE: This is GCU box orientation, NOT stabilised camera.\r\n");
    UART_1_PutString("        Rotate the whole unit to watch yaw track physical heading.\r\n");
}

/* --- attr 48: Motor drive status (TEST verbose) --- */
static void print_test_motor_drive(const movi_status_t* st, uint32_t dt,
                                    uint16_t rxlen, const uint8_t* p, uint8_t plen)
{
    char buf[TRANSMIT_BUFFER_SIZE];
    (void)st;
    sprintf(buf, "\r\n[TEST][Drv] attr=48  dt=%lums  rxlen=%u  snaplen=%u\r\n",
            (unsigned long)dt, (unsigned)rxlen, (unsigned)plen);
    UART_1_PutString(buf);
    print_hex_dump(p, plen);
    if (plen < 9u) { UART_1_PutString("  ERROR: too short\r\n"); return; }

    sprintf(buf, "  off  0-7: zeros (always)\r\n");
    UART_1_PutString(buf);
    sprintf(buf, "  off  8:  0x%02X  (constant device flag -- confirmed 0xCB)\r\n",
            (unsigned)p[8]);
    UART_1_PutString(buf);

    if (plen >= 14u) {
        sprintf(buf,
            "  off  9: 0x%02X  off 10: 0x%02X  off 11: 0x%02X  off 12: 0x%02X  off 13: 0x%02X"
            "  [motor status -- non-zero only when QX277 active]\r\n",
            (unsigned)p[9], (unsigned)p[10], (unsigned)p[11],
            (unsigned)p[12], (unsigned)p[13]);
        UART_1_PutString(buf);

        uint8_t any_nz = 0u;
        uint8_t i;
        for (i = 9u; i <= 13u; i++) if (p[i]) { any_nz = 1u; break; }
        UART_1_PutString(any_nz
            ? "  -> non-zero bytes detected: QX277 was active\r\n"
            : "  -> all zero: gimbal idle (no QX277 in progress)\r\n");
    }
}

/* =========================================================================
 * Test 3 — Quaternion cross-check
 * Prints attr 287 quat → Euler, then requests attr 22 to compare direct Euler.
 * ========================================================================= */
static void print_quat_crosscheck(movi_comm_t* m)
{
    char buf[TRANSMIT_BUFFER_SIZE];
    movi_status_t st;
    movi_comm_get_status(m, &st);

    UART_1_PutString("\r\n[TEST3] Quaternion vs Euler cross-check\r\n");
    UART_1_PutString("[TEST3] Step 1: attr 287 auto-push (last received) -> Euler via PSoC math\r\n");

    if (!st.valid) {
        UART_1_PutString("[TEST3] ERROR: no valid attr 287 frame yet -- enter MOVI_CONTROL first\r\n");
        return;
    }

    char pan_s, tilt_s, roll_s;
    int16 pan_i, pan_f, tilt_i, tilt_f, roll_i, roll_f;
    compute_euler_ints(&st,
        &pan_s,  &pan_i,  &pan_f,
        &tilt_s, &tilt_i, &tilt_f,
        &roll_s, &roll_i, &roll_f);

    sprintf(buf, "  Q->Euler:  Pan=%c%3d.%02d  Tilt=%c%3d.%02d  Roll=%c%3d.%02d deg\r\n",
            pan_s,  (int)pan_i,  (int)pan_f,
            tilt_s, (int)tilt_i, (int)tilt_f,
            roll_s, (int)roll_i, (int)roll_f);
    UART_1_PutString(buf);
    sprintf(buf, "  Raw quat:  R=%+.4f  I=%+.4f  J=%+.4f  K=%+.4f\r\n",
            st.gimbal_r, st.gimbal_i, st.gimbal_j, st.gimbal_k);
    UART_1_PutString(buf);

    UART_1_PutString("[TEST3] Step 2: requesting attr 22 (direct Euler from GCU) ...\r\n");
    movi_comm_request_attr(m, 22u);
    m->test_sent_ms = g_tick_ms;
    /* The verbose response will auto-print via the test dispatcher when it arrives. */
    UART_1_PutString("[TEST3] When attr 22 arrives, compare yaw/pitch/roll values above.\r\n");
    UART_1_PutString("[TEST3] They should agree within ~0.5 deg if both decodes are correct.\r\n");
}

/* =========================================================================
 * Test dispatch for MOVI_TEST mode
 * ========================================================================= */
static void print_test_response(movi_comm_t* m, uint32_t dt)
{
    movi_status_t st;
    movi_comm_get_status(m, &st);
    uint16_t len     = m->test_rx_len;
    uint16_t attr    = m->test_response_attr;
    const uint8_t *p = m->test_rx_payload;
    uint8_t  plen    = m->test_rx_payload_len;

    switch (attr) {
        case 22u:  print_test_attitude(&st, dt, len, p, plen);     break;
        case  4u:  print_test_gps(&st, dt, len, p, plen);          break;
        case  3u:  print_test_baro(&st, dt, len, p, plen);         break;
        case  1u:  print_test_sysstat(&st, dt, len, p, plen);      break;
        case 12u:  print_test_mag(&st, dt, len, p, plen);          break;
        case  2u:  print_test_platform_att(&st, dt, len, p, plen); break;
        case 48u:  print_test_motor_drive(&st, dt, len, p, plen);  break;
        default: {
            char buf[64];
            sprintf(buf, "[TEST] attr=%u responded -- no printer defined\r\n",
                    (unsigned)attr);
            UART_1_PutString(buf);
            break;
        }
    }
}

/* =========================================================================
 * SENSOR MODE compact printers
 * One-line output per attr for continuous streaming.
 * ========================================================================= */
static void print_sensor_mag(const uint8_t* p, uint8_t plen)
{
    if (plen < 6u) { UART_1_PutString("[MAG] short\r\n"); return; }
    char buf[TRANSMIT_BUFFER_SIZE];
    int16_t x = SNAP16BE(p, plen, 0);
    int16_t y = SNAP16BE(p, plen, 2);
    int16_t z = SNAP16BE(p, plen, 4);
    /* Heading estimate from XY plane (magnetic north relative) */
    float hdg = atan2f((float)y, (float)x) * 57.2958f;
    if (hdg < 0.0f) hdg += 360.0f;
    int16 hi, hf; char hs;
    decomp(hdg, &hi, &hf, &hs);
    /* Decl field at off 12 if present */
    char decl_buf[24] = "";
    if (plen >= 14u) {
        int16_t decl = SNAP16BE(p, plen, 12);
        char ds; int16 di, df;
        decomp((float)decl/100.0f, &di, &df, &ds);
        sprintf(decl_buf, "  decl=%c%d.%02d", ds, (int)di, (int)df);
    }
    sprintf(buf, "[MAG]  X=%6d  Y=%6d  Z=%6d  hdg~=%3d.%02d deg%s\r\n",
            (int)x, (int)y, (int)z, (int)hi, (int)hf, decl_buf);
    UART_1_PutString(buf);
}

static void print_sensor_baro(const uint8_t* p, uint8_t plen)
{
    if (plen < 5u) { UART_1_PutString("[BARO] short\r\n"); return; }
    char buf[TRANSMIT_BUFFER_SIZE];
    int16_t alt = SNAP16(p, plen, 1);
    int16_t roc = SNAP16(p, plen, 3);
    char as, rs; int16 ai, af, ri, rf;
    decomp((float)alt/10.0f,  &ai, &af, &as);
    decomp((float)roc/100.0f, &ri, &rf, &rs);
    sprintf(buf, "[BARO] alt=%c%d.%02d m  roc=%c%d.%02d m/s\r\n",
            as, (int)ai, (int)af, rs, (int)ri, (int)rf);
    UART_1_PutString(buf);
}

static void print_sensor_sysstat(const uint8_t* p, uint8_t plen)
{
    if (plen < 4u) { UART_1_PutString("[STAT] short\r\n"); return; }
    char buf[TRANSMIT_BUFFER_SIZE];
    int16_t bv = SNAP16BE(p, plen, 0);
    uint8_t sats = p[2];
    uint8_t temp_raw = p[3];
    char vs; int16 vi, vf;
    decomp((float)bv/100.0f, &vi, &vf, &vs);
    /* temp: raw/3 = degrees C */
    int16_t tc = (int16_t)((uint16_t)temp_raw * 100u / 3u);  /* *100 for /3 with decimal */
    sprintf(buf, "[STAT] batt=%c%d.%02d V  sats=%u  temp=%d.%02d C",
            vs, (int)vi, (int)vf,
            (unsigned)sats,
            (int)(tc/100), (int)(tc%100));
    if (plen >= 11u) {
        int16_t pres = SNAP16BE(p, plen, 9);
        char ps; int16 pi, pf;
        decomp((float)pres/10.0f, &pi, &pf, &ps);
        char pres_buf[20];
        sprintf(pres_buf, "  pres=%c%d.%02d mb", ps, (int)pi, (int)pf);
        UART_1_PutString(buf);
        UART_1_PutString(pres_buf);
        UART_1_PutString("\r\n");
    } else {
        UART_1_PutString(buf);
        UART_1_PutString("\r\n");
    }
}

static void print_sensor_platform_att(const uint8_t* p, uint8_t plen)
{
    if (plen < 7u) { UART_1_PutString("[PLAT] short\r\n"); return; }
    char buf[TRANSMIT_BUFFER_SIZE];
    int16_t roll  = SNAP16BE(p, plen, 1);
    int16_t pitch = SNAP16BE(p, plen, 3);
    int16_t yaw   = SNAP16BE(p, plen, 5);
    char rs, ps, ys; int16 ri, rf, pi, pf, yi, yf;
    decomp((float)roll /100.0f, &ri, &rf, &rs);
    decomp((float)pitch/100.0f, &pi, &pf, &ps);
    decomp((float)yaw  /100.0f, &yi, &yf, &ys);
    sprintf(buf, "[PLAT] roll=%c%d.%02d  pitch=%c%d.%02d  yaw=%c%d.%02d deg  (platform body)\r\n",
            rs, (int)ri, (int)rf,
            ps, (int)pi, (int)pf,
            ys, (int)yi, (int)yf);
    UART_1_PutString(buf);
}

/* Dispatch compact sensor print based on attr */
static void print_sensor_response(uint16_t attr, const uint8_t* p, uint8_t plen)
{
    switch (attr) {
        case 12u: print_sensor_mag(p, plen);          break;
        case  3u: print_sensor_baro(p, plen);         break;
        case  1u: print_sensor_sysstat(p, plen);      break;
        case  2u: print_sensor_platform_att(p, plen); break;
        default: break;
    }
}

/* =========================================================================
 * main
 * ========================================================================= */
int main(void)
{
    char tx[TRANSMIT_BUFFER_SIZE];
    uint8 ch;

    app_mode_t app_mode = APP_MODE_STANDBY;

    uint8 joystick_active = FALSE;
    uint8 streaming_adc   = FALSE;
    uint8 diag_active     = FALSE;

    ff_api_control_type_e ctrl_mode = DEFER;

    static uint32 last_tx_ms   = 0u;
    static uint32 last_diag_ms = 0u;

    /* ---- Test 2: absolute angle preset state ---- */
    static float   abs_pan_target  = 0.0f;
    static float   abs_tilt_target = 0.0f;
    static uint8_t abs_preset_active = FALSE;

    /* ---- Test 4 / diag: on-demand attr reads in MOVI_CONTROL ---- */
    static uint8_t ctrl_attr_pending = 0u;   /* attr ID to read this loop */
    static uint8_t show_platform_yaw = FALSE; /* 'p' toggle */

    /* ---- RESET mode state ---- */
    typedef enum { RESET_IDLE=0, RESET_SENT, RESET_ACTIVE, RESET_DONE } reset_state_t;
    static reset_state_t reset_state  = RESET_IDLE;
    static uint32_t      reset_poll_ms = 0u;
    static uint32_t      reset_start_ms = 0u;
    static uint8_t       reset_saw_01   = FALSE;

    /* ---- SENSOR mode state ---- */
    static const uint16_t sensor_attrs[SENSOR_N_ATTRS] = {12u, 3u, 1u, 2u};
    static uint8_t  sensor_idx         = 0u;
    static uint32_t sensor_sent_ms     = 0u;
    static uint8_t  sensor_tx_pending  = FALSE;

    /* ---- Scan mode state ---- */
    static uint16_t scan_next_attr     = SCAN_ATTR_MIN;
    static uint16_t scan_batch_start   = SCAN_ATTR_MIN;
    static uint8_t  scan_waiting_space = FALSE;
    static uint8_t  scan_tx_pending    = FALSE;
    static uint32_t scan_sent_ms       = 0u;
    static uint8_t  scan_tx_buf[SCAN_TX_BUF_SIZE];
    static uint8_t  scan_tx_len        = 0u;

    int16     counts[N_CH];
    joy_cmd_t cmd;
    cmd.u[0] = 0.0f;
    cmd.u[1] = 0.0f;

    movi_control_t ctl;

    joy_cal_state_t cal_prev = JOY_CAL_OFF;
    joy_cal_t defaults;
    uint8 i;
    for (i = 0u; i < (uint8)N_CH; i++) {
        defaults.minv[i]   = 0;
        defaults.maxv[i]   = 255;
        defaults.center[i] = 128;
    }
    defaults.valid = 0u;
    joy_sensitivity_t current_sense = SENSE_MED;
    uint8 last_button_state = 1;

    uint32 invert_mask = (1u << CH_X);

    /* ------------------------------------------------------------------
     * Hardware init
     * ------------------------------------------------------------------ */
    CyGlobalIntEnable;

    UART_1_Start();
    UART_1_PutString("\r\n========================================\r\n");
    UART_1_PutString("  MoVI Pro Controller  PSoC 5LP\r\n");
    UART_1_PutString("========================================\r\n");
    UART_1_PutString("  m=ctrl  t=test  r=RESET  v=SENSOR  w=scan\r\n");
    UART_1_PutString("  j=joy  l=mode  s=adc  c=cal  k=kill  x=quiet\r\n");
    UART_1_PutString("  -- MOVI_CONTROL extra keys --\r\n");
    UART_1_PutString("  f=attr48  p=platform-yaw toggle\r\n");
    UART_1_PutString("  0=home  1=pan+45  2=pan-45  3=tilt+30  4=tilt-30  5=pan+90\r\n");
    UART_1_PutString("  (abs presets require 'l' to ABS mode first)\r\n");
    UART_1_PutString("  -- TEST mode keys --\r\n");
    UART_1_PutString("  a=att22  g=GPS  b=baro  s=stat  n=mag\r\n");
    UART_1_PutString("  2=platform-att  4=motor-drv  q=quat-crosscheck\r\n");
    UART_1_PutString("  t=exit  (2s timeout)  k/x=kill+exit\r\n");
    UART_1_PutString("========================================\r\n");

    UART_MOVI_Start();
    UART_MOVI_ClearRxBuffer();
    UART_MOVI_ClearTxBuffer();

    movi_comm_init(&g_movi, &MOVI_HAL);
    isr_rx_movi_StartEx(isr_rx_movi_Handler);
    UART_1_PutString("[Init] MoVI UART ready\r\n");

    joystick_init(&defaults, invert_mask);
    adc_balanced_init();
    UART_1_PutString("[Init] ADC + Joystick ready\r\n");

    Looptimer_Start();
    isr_Looptimer_StartEx(isr_Looptimer_Handler);
    UART_1_PutString("[Init] Loop timer ready\r\n");
    UART_1_PutString("[Init] Mode: STANDBY\r\n");

    /* ------------------------------------------------------------------
     * Main loop
     * ------------------------------------------------------------------ */
    for (;;)
    {
        /* ----------------------------------------------------------------
         * STEP 1  CLI
         * ---------------------------------------------------------------- */
        ch = (uint8)UART_1_GetChar();

        /* ==============================================================
         * Keys: STANDBY and MOVI_CONTROL
         * ============================================================== */
        if (app_mode == APP_MODE_STANDBY || app_mode == APP_MODE_MOVI_CONTROL)
        {
            if (ch == 'm' || ch == 'M')
            {
                if (app_mode == APP_MODE_STANDBY) {
                    app_mode    = APP_MODE_MOVI_CONTROL;
                    diag_active = TRUE;
                    abs_preset_active = FALSE;
                    UART_1_PutString("[CLI] -> MOVI_CONTROL  TX active\r\n");
                } else {
                    app_mode = APP_MODE_STANDBY;
                    UART_1_PutString("[CLI] -> STANDBY  TX stopped\r\n");
                }
            }
            else if (ch == 't' || ch == 'T')
            {
                if (app_mode == APP_MODE_STANDBY) {
                    app_mode = APP_MODE_MOVI_TEST;
                    UART_1_PutString("[CLI] -> MOVI_TEST\r\n");
                    UART_1_PutString("  a=att  g=GPS  b=baro  s=stat  n=mag\r\n");
                    UART_1_PutString("  2=platform  4=motor  q=quat-xcheck  t=exit\r\n");
                } else {
                    UART_1_PutString("[CLI] Enter STANDBY first (m to stop ctrl)\r\n");
                }
            }
            else if (ch == 'r' || ch == 'R')
            {
                /* ---- TEST 1: heading reset ---- */
                if (app_mode == APP_MODE_STANDBY) {
                    app_mode      = APP_MODE_RESET;
                    reset_state   = RESET_SENT;
                    reset_saw_01  = FALSE;
                    reset_start_ms = g_tick_ms;
                    reset_poll_ms  = g_tick_ms;
                    UART_1_PutString("[RESET] Sending attr 382 = 0x01 (heading reset)\r\n");
                    UART_1_PutString("[RESET] Polling echo at 1 Hz.  x=abort\r\n");
                    UART_1_PutString("[RESET] Expected: GCU echoes 0x01 for ~10s, then 0x00 = done\r\n");
                    send_write_attr_u8(382u, 0x01u);
                    /* Also arm a read so we get the first echo */
                    movi_comm_request_attr(&g_movi, 382u);
                    g_movi.test_sent_ms = g_tick_ms;
                } else {
                    UART_1_PutString("[CLI] Enter STANDBY first\r\n");
                }
            }
            else if (ch == 'v' || ch == 'V')
            {
                /* ---- Sensor validation stream ---- */
                if (app_mode == APP_MODE_STANDBY) {
                    app_mode          = APP_MODE_SENSOR;
                    sensor_idx        = 0u;
                    sensor_tx_pending = FALSE;
                    UART_1_PutString("[SENSOR] Streaming: MAG, BARO, STAT, PLAT-ATT\r\n");
                    UART_1_PutString("[SENSOR] Rotate unit to observe yaw tracking.\r\n");
                    UART_1_PutString("[SENSOR] Any key to exit.\r\n");
                } else {
                    UART_1_PutString("[CLI] Enter STANDBY first\r\n");
                }
            }
            else if (ch == 'w' || ch == 'W')
            {
                if (app_mode == APP_MODE_STANDBY) {
                    app_mode          = APP_MODE_MOVI_SCAN;
                    scan_next_attr    = SCAN_ATTR_MIN;
                    scan_batch_start  = SCAN_ATTR_MIN;
                    scan_waiting_space = FALSE;
                    scan_tx_pending   = FALSE;
                    UART_1_PutString("[SCAN] Attr sweep, 500ms timeout, 200/batch\r\n");
                    UART_1_PutString("[SCAN] Format: attr,dt_ms,TX_hex,RX_hex\r\n");
                    UART_1_PutString("[SCAN] any key=next batch  x=abort\r\n");
                } else {
                    UART_1_PutString("[CLI] Enter STANDBY first\r\n");
                }
            }
            else if (ch == 'j' || ch == 'J')
            {
                joystick_active = !joystick_active;
                if (joystick_active) { diag_active = TRUE; abs_preset_active = FALSE; }
                UART_1_PutString(joystick_active
                    ? "[CLI] Joystick ON  (presets cleared)\r\n"
                    : "[CLI] Joystick OFF\r\n");
            }
            else if (ch == 'l' || ch == 'L')
            {
                if      (ctrl_mode == DEFER)    ctrl_mode = RATE;
                else if (ctrl_mode == RATE)     ctrl_mode = ABSOLUTE;
                else                             ctrl_mode = DEFER;
                abs_preset_active = FALSE;
                sprintf(tx, "[CLI] Mode: %s\r\n", mode_to_str(ctrl_mode));
                UART_1_PutString(tx);
                if (ctrl_mode == ABSOLUTE)
                    UART_1_PutString("[CLI] ABS mode: keys 0-5 for angle presets\r\n");
            }
            else if (ch == 'k' || ch == 'K')
            {
                abs_preset_active = FALSE;
                do_kill(&g_movi, &app_mode, &joystick_active,
                        &streaming_adc, &ctrl_mode);
            }
            else if (ch == 'x' || ch == 'X')
            {
                abs_preset_active = FALSE;
                do_kill(&g_movi, &app_mode, &joystick_active,
                        &streaming_adc, &ctrl_mode);
                diag_active = FALSE;
                UART_1_PutString("[CLI] Output paused. Press m or j to resume.\r\n");
            }
            else if (ch == 's' || ch == 'S')
            {
                streaming_adc = !streaming_adc;
                UART_1_PutString(streaming_adc
                    ? "[CLI] ADC stream ON\r\n"
                    : "[CLI] ADC stream OFF\r\n");
            }
            else if (ch == 'c' || ch == 'C')
            {
                joystick_on_key('c');
            }

            /* ---- MOVI_CONTROL-only keys ---- */
            if (app_mode == APP_MODE_MOVI_CONTROL)
            {
                /* f: one-shot attr 48 read (test 4) */
                if (ch == 'f' || ch == 'F')
                {
                    movi_comm_request_attr(&g_movi, 48u);
                    g_movi.test_sent_ms = g_tick_ms;
                    ctrl_attr_pending   = 48u;
                    UART_1_PutString("[CTRL] Attr 48 requested (motor drive status)\r\n");
                }
                /* p: toggle platform-yaw in diag (M2) */
                else if (ch == 'p' || ch == 'P')
                {
                    show_platform_yaw = !show_platform_yaw;
                    UART_1_PutString(show_platform_yaw
                        ? "[CTRL] Platform yaw ON in diag (attr 2 requested each cycle)\r\n"
                        : "[CTRL] Platform yaw OFF\r\n");
                }
                /* Absolute angle presets (test 2) */
                else if (ch >= '0' && ch <= '5')
                {
                    if (ctrl_mode != ABSOLUTE) {
                        UART_1_PutString("[CTRL] Need ABS mode (press l)\r\n");
                    } else {
                        switch (ch) {
                            case '0': abs_pan_target=0.0f;                abs_tilt_target=0.0f;
                                      UART_1_PutString("[CTRL] Preset: HOME  pan=0  tilt=0\r\n"); break;
                            case '1': abs_pan_target=DEG_TO_UNIT(45.0f);  abs_tilt_target=0.0f;
                                      UART_1_PutString("[CTRL] Preset: pan=+45 deg\r\n"); break;
                            case '2': abs_pan_target=DEG_TO_UNIT(-45.0f); abs_tilt_target=0.0f;
                                      UART_1_PutString("[CTRL] Preset: pan=-45 deg\r\n"); break;
                            case '3': abs_pan_target=0.0f;                abs_tilt_target=DEG_TO_UNIT(30.0f);
                                      UART_1_PutString("[CTRL] Preset: tilt=+30 deg\r\n"); break;
                            case '4': abs_pan_target=0.0f;                abs_tilt_target=DEG_TO_UNIT(-30.0f);
                                      UART_1_PutString("[CTRL] Preset: tilt=-30 deg\r\n"); break;
                            case '5': abs_pan_target=DEG_TO_UNIT(90.0f);  abs_tilt_target=0.0f;
                                      UART_1_PutString("[CTRL] Preset: pan=+90 deg\r\n"); break;
                            default: break;
                        }
                        abs_preset_active = TRUE;
                        joystick_active   = FALSE;
                    }
                }
            }
        }

        /* ==============================================================
         * Keys: MOVI_SCAN
         * ============================================================== */
        else if (app_mode == APP_MODE_MOVI_SCAN)
        {
            if (ch != 0u) {
                if (ch == 'x' || ch == 'X' || ch == 'k' || ch == 'K') {
                    scan_tx_pending    = FALSE;
                    scan_waiting_space = FALSE;
                    g_movi.test_pending_attr   = 0u;
                    g_movi.test_response_ready = false;
                    app_mode = APP_MODE_STANDBY;
                    UART_1_PutString("[SCAN] Aborted -> STANDBY\r\n");
                } else if (scan_waiting_space) {
                    scan_waiting_space = FALSE;
                }
            }
        }

        /* ==============================================================
         * Keys: MOVI_TEST
         * ============================================================== */
        else if (app_mode == APP_MODE_MOVI_TEST)
        {
            if (ch == 't' || ch == 'T') {
                g_movi.test_pending_attr   = 0u;
                g_movi.test_response_ready = false;
                app_mode = APP_MODE_STANDBY;
                UART_1_PutString("[TEST] -> STANDBY\r\n");
            }
            else if (ch == 'k' || ch == 'K' || ch == 'x' || ch == 'X') {
                g_movi.test_pending_attr   = 0u;
                g_movi.test_response_ready = false;
                do_kill(&g_movi, &app_mode, &joystick_active,
                        &streaming_adc, &ctrl_mode);
                if (ch == 'x' || ch == 'X') {
                    diag_active = FALSE;
                    UART_1_PutString("[CLI] Output paused.\r\n");
                }
            }
            else if (ch == 'a' || ch == 'A') {
                movi_comm_request_attr(&g_movi, 22u);
                g_movi.test_sent_ms = g_tick_ms;
                UART_1_PutString("[TEST] Sent attr 22 (gimbal euler) ...\r\n");
            }
            else if (ch == 'g' || ch == 'G') {
                movi_comm_request_attr(&g_movi, 4u);
                g_movi.test_sent_ms = g_tick_ms;
                UART_1_PutString("[TEST] Sent attr 4 (GPS) ...\r\n");
            }
            else if (ch == 'b' || ch == 'B') {
                movi_comm_request_attr(&g_movi, 3u);
                g_movi.test_sent_ms = g_tick_ms;
                UART_1_PutString("[TEST] Sent attr 3 (baro) ...\r\n");
            }
            else if (ch == 's' || ch == 'S') {
                movi_comm_request_attr(&g_movi, 1u);
                g_movi.test_sent_ms = g_tick_ms;
                UART_1_PutString("[TEST] Sent attr 1 (system status) ...\r\n");
            }
            else if (ch == 'n' || ch == 'N') {
                movi_comm_request_attr(&g_movi, 12u);
                g_movi.test_sent_ms = g_tick_ms;
                UART_1_PutString("[TEST] Sent attr 12 (magnetometer) ...\r\n");
            }
            else if (ch == '2') {
                /* M2: platform body attitude */
                movi_comm_request_attr(&g_movi, 2u);
                g_movi.test_sent_ms = g_tick_ms;
                UART_1_PutString("[TEST] Sent attr 2 (platform attitude) ...\r\n");
                UART_1_PutString("[TEST] Rotate the whole unit to see yaw change.\r\n");
            }
            else if (ch == '4') {
                /* Test 4: motor drive status */
                movi_comm_request_attr(&g_movi, 48u);
                g_movi.test_sent_ms = g_tick_ms;
                UART_1_PutString("[TEST] Sent attr 48 (motor drive) ...\r\n");
                UART_1_PutString("[TEST] NOTE: off9-13 only non-zero if QX277 was active.\r\n");
                UART_1_PutString("[TEST] Enter CTRL (m) first, then test, then come back here.\r\n");
            }
            else if (ch == 'q' || ch == 'Q') {
                /* Test 3: quaternion cross-check */
                print_quat_crosscheck(&g_movi);
            }
        }

        /* ==============================================================
         * Keys: RESET mode
         * ============================================================== */
        else if (app_mode == APP_MODE_RESET)
        {
            if (ch == 'x' || ch == 'X' || ch == 'k' || ch == 'K') {
                g_movi.test_pending_attr   = 0u;
                g_movi.test_response_ready = false;
                reset_state = RESET_IDLE;
                app_mode    = APP_MODE_STANDBY;
                UART_1_PutString("[RESET] Aborted -> STANDBY\r\n");
            }
        }

        /* ==============================================================
         * Keys: SENSOR mode
         * ============================================================== */
        else if (app_mode == APP_MODE_SENSOR)
        {
            if (ch != 0u) {
                /* Any key exits */
                g_movi.test_pending_attr   = 0u;
                g_movi.test_response_ready = false;
                sensor_tx_pending = FALSE;
                app_mode = APP_MODE_STANDBY;
                UART_1_PutString("[SENSOR] -> STANDBY\r\n");
            }
        }

        /* Calibration state prompt */
        joy_cal_state_t cal_now = joystick_cal_state();
        print_cal_prompt_on_state_change(cal_prev, cal_now);
        cal_prev = cal_now;

        /* ----------------------------------------------------------------
         * STEP 2  Pump RX from MoVI (always)
         * ---------------------------------------------------------------- */
        movi_comm_pump(&g_movi);

        /* ----------------------------------------------------------------
         * STEP 2b  MOVI_TEST: response or timeout
         * ---------------------------------------------------------------- */
        if (app_mode == APP_MODE_MOVI_TEST)
        {
            if (g_movi.test_response_ready) {
                uint32_t dt = g_tick_ms - g_movi.test_sent_ms;
                print_test_response(&g_movi, dt);
                g_movi.test_response_ready = false;
                g_movi.test_pending_attr   = 0u;
            }
            else if (g_movi.test_pending_attr != 0u &&
                     (g_tick_ms - g_movi.test_sent_ms) >= TEST_TIMEOUT_MS) {
                sprintf(tx, "[TEST] TIMEOUT -- attr=%u\r\n",
                        (unsigned)g_movi.test_pending_attr);
                UART_1_PutString(tx);
                g_movi.test_pending_attr = 0u;
            }
        }

        /* ----------------------------------------------------------------
         * STEP 2c  MOVI_SCAN
         * ---------------------------------------------------------------- */
        if (app_mode == APP_MODE_MOVI_SCAN)
        {
            if (scan_waiting_space) {
                /* handled in CLI block above */
            }
            else if (scan_tx_pending && g_movi.test_response_ready) {
                uint32_t dt = g_tick_ms - scan_sent_ms;
                char* bp = tx;
                bp += sprintf(bp, "%u,%lu,", (unsigned)g_movi.test_response_attr,
                              (unsigned long)dt);
                uint8_t si;
                for (si = 0u; si < scan_tx_len; si++)
                    bp += sprintf(bp, "%02X", (unsigned)scan_tx_buf[si]);
                bp += sprintf(bp, ",");
                for (si = 0u; si < g_movi.test_rx_payload_len; si++)
                    bp += sprintf(bp, "%02X", (unsigned)g_movi.test_rx_payload[si]);
                bp += sprintf(bp, "\r\n");
                UART_1_PutString(tx);

                g_movi.test_response_ready = false;
                g_movi.test_pending_attr   = 0u;
                scan_tx_pending            = FALSE;
                scan_next_attr++;

                if (scan_next_attr == scan_batch_start + SCAN_BATCH_SIZE
                    || scan_next_attr > SCAN_ATTR_MAX) {
                    UART_1_PutString("=========\r\n");
                    if (scan_next_attr > SCAN_ATTR_MAX) {
                        UART_1_PutString("[SCAN] Complete -> STANDBY\r\n");
                        app_mode = APP_MODE_STANDBY;
                    } else {
                        scan_batch_start   = scan_next_attr;
                        scan_waiting_space = TRUE;
                        UART_1_PutString("[SCAN] any key for next batch\r\n");
                    }
                }
            }
            else if (scan_tx_pending &&
                     (g_tick_ms - scan_sent_ms) >= SCAN_TIMEOUT_MS) {
                char* bp = tx;
                bp += sprintf(bp, "%u,%u,", (unsigned)scan_next_attr,
                              (unsigned)SCAN_TIMEOUT_MS);
                uint8_t si;
                for (si = 0u; si < scan_tx_len; si++)
                    bp += sprintf(bp, "%02X", (unsigned)scan_tx_buf[si]);
                bp += sprintf(bp, ",\r\n");
                UART_1_PutString(tx);

                g_movi.test_pending_attr = 0u;
                scan_tx_pending          = FALSE;
                scan_next_attr++;

                if (scan_next_attr == scan_batch_start + SCAN_BATCH_SIZE
                    || scan_next_attr > SCAN_ATTR_MAX) {
                    UART_1_PutString("=========\r\n");
                    if (scan_next_attr > SCAN_ATTR_MAX) {
                        UART_1_PutString("[SCAN] Complete -> STANDBY\r\n");
                        app_mode = APP_MODE_STANDBY;
                    } else {
                        scan_batch_start   = scan_next_attr;
                        scan_waiting_space = TRUE;
                        UART_1_PutString("[SCAN] any key for next batch\r\n");
                    }
                }
            }
            else if (!scan_tx_pending && !scan_waiting_space
                     && scan_next_attr <= SCAN_ATTR_MAX) {
                scan_tx_len = 0u;
                movi_comm_request_attr_capture(&g_movi, scan_next_attr,
                                               scan_tx_buf, SCAN_TX_BUF_SIZE,
                                               &scan_tx_len);
                scan_sent_ms    = g_tick_ms;
                g_movi.test_sent_ms = scan_sent_ms;
                scan_tx_pending = TRUE;
            }
        }

        /* ----------------------------------------------------------------
         * STEP 2d  RESET mode state machine  (test 1)
         * ---------------------------------------------------------------- */
        if (app_mode == APP_MODE_RESET)
        {
            /* Handle incoming response to our attr 382 read poll */
            if (g_movi.test_response_ready &&
                g_movi.test_response_attr == 382u)
            {
                uint8_t echo = (g_movi.test_rx_payload_len > 0u)
                               ? g_movi.test_rx_payload[0] : 0xFFu;
                g_movi.test_response_ready = false;
                g_movi.test_pending_attr   = 0u;

                if (echo == 0x01u) {
                    reset_saw_01 = TRUE;
                    sprintf(tx, "[RESET] t=%lus  echo=0x01  Resetting...\r\n",
                            (unsigned long)((g_tick_ms - reset_start_ms) / 1000u));
                    UART_1_PutString(tx);
                    reset_state = RESET_ACTIVE;
                } else if (echo == 0x00u && reset_saw_01) {
                    /* Was active, now complete */
                    sprintf(tx, "[RESET] COMPLETE  t=%lus  echo=0x00\r\n",
                            (unsigned long)((g_tick_ms - reset_start_ms) / 1000u));
                    UART_1_PutString(tx);
                    UART_1_PutString("[RESET] Heading reference established.\r\n");
                    UART_1_PutString("[RESET] -> STANDBY.  Send QX277 ABSOLUTE to point.\r\n");
                    reset_state = RESET_DONE;
                    app_mode    = APP_MODE_STANDBY;
                } else {
                    sprintf(tx, "[RESET] t=%lus  echo=0x%02X  (waiting for 0x01...)\r\n",
                            (unsigned long)((g_tick_ms - reset_start_ms) / 1000u),
                            (unsigned)echo);
                    UART_1_PutString(tx);
                }
            }

            /* Poll at 1 Hz */
            if (reset_state != RESET_DONE &&
                (g_tick_ms - reset_poll_ms) >= RESET_POLL_MS &&
                g_movi.test_pending_attr == 0u)
            {
                reset_poll_ms = g_tick_ms;
                movi_comm_request_attr(&g_movi, 382u);
                g_movi.test_sent_ms = g_tick_ms;
            }

            /* Timeout guard */
            if ((g_tick_ms - reset_start_ms) >= RESET_TIMEOUT_MS) {
                UART_1_PutString("[RESET] TIMEOUT (15s) -- GCU did not complete.\r\n");
                UART_1_PutString("[RESET] Check gimbal status flags, try again.\r\n");
                g_movi.test_pending_attr   = 0u;
                g_movi.test_response_ready = false;
                reset_state = RESET_IDLE;
                app_mode    = APP_MODE_STANDBY;
            }
        }

        /* ----------------------------------------------------------------
         * STEP 2e  SENSOR mode  (indoor sensor validation)
         * ---------------------------------------------------------------- */
        if (app_mode == APP_MODE_SENSOR)
        {
            /* Response arrived */
            if (sensor_tx_pending && g_movi.test_response_ready) {
                print_sensor_response(g_movi.test_response_attr,
                                      g_movi.test_rx_payload,
                                      g_movi.test_rx_payload_len);
                g_movi.test_response_ready = false;
                g_movi.test_pending_attr   = 0u;
                sensor_tx_pending          = FALSE;
                sensor_idx = (uint8_t)((sensor_idx + 1u) % SENSOR_N_ATTRS);
            }
            /* Timeout */
            else if (sensor_tx_pending &&
                     (g_tick_ms - sensor_sent_ms) >= SENSOR_TIMEOUT_MS) {
                sprintf(tx, "[SENSOR] timeout attr=%u\r\n",
                        (unsigned)sensor_attrs[sensor_idx]);
                UART_1_PutString(tx);
                g_movi.test_pending_attr = 0u;
                sensor_tx_pending        = FALSE;
                sensor_idx = (uint8_t)((sensor_idx + 1u) % SENSOR_N_ATTRS);
            }
            /* Arm next request after interval */
            else if (!sensor_tx_pending &&
                     (g_tick_ms - sensor_sent_ms) >= SENSOR_POLL_MS) {
                uint16_t attr_to_req = sensor_attrs[sensor_idx];
                movi_comm_request_attr(&g_movi, attr_to_req);
                g_movi.test_sent_ms = g_tick_ms;
                sensor_sent_ms      = g_tick_ms;
                sensor_tx_pending   = TRUE;
            }
        }

        /* ----------------------------------------------------------------
         * STEP 2f  MOVI_CONTROL: handle on-demand attr reads (f and p keys)
         * ---------------------------------------------------------------- */
        if (app_mode == APP_MODE_MOVI_CONTROL)
        {
            /* Attr 48 one-shot response (test 4) */
            if (ctrl_attr_pending != 0u && g_movi.test_response_ready &&
                g_movi.test_response_attr == ctrl_attr_pending)
            {
                uint32_t dt = g_tick_ms - g_movi.test_sent_ms;
                print_test_response(&g_movi, dt);
                g_movi.test_response_ready = false;
                g_movi.test_pending_attr   = 0u;
                ctrl_attr_pending          = 0u;
            }
            /* Timeout for on-demand reads */
            else if (ctrl_attr_pending != 0u &&
                     (g_tick_ms - g_movi.test_sent_ms) >= TEST_TIMEOUT_MS)
            {
                sprintf(tx, "[CTRL] attr %u timeout\r\n", (unsigned)ctrl_attr_pending);
                UART_1_PutString(tx);
                g_movi.test_pending_attr = 0u;
                ctrl_attr_pending        = 0u;
            }
        }

        /* ----------------------------------------------------------------
         * STEP 3  Joystick ADC
         * ---------------------------------------------------------------- */
        uint8 current_pin_state = Pin_Sensitivity_Read();
        if (last_button_state == 1 && current_pin_state == 0)
        {
            CyDelay(50);
            if (Pin_Sensitivity_Read() == 0) {
                current_sense = (joy_sensitivity_t)((current_sense + 1) % 3);
                joystick_set_sensitivity(current_sense);
                UART_1_PutString("Mode Switched!\r\n");
                while(Pin_Sensitivity_Read() == 0);
            }
        }
        last_button_state = current_pin_state;

        if (adc_balanced_poll_frame(counts))
        {
            joystick_on_sample(counts);
            joystick_get_cmd(&cmd);

            if (streaming_adc) {
                int16 pan_milli  = (int16)(cmd.u[CH_X] * 1000.0f);
                int16 tilt_milli = (int16)(cmd.u[CH_Y] * 1000.0f);
                int32 x_mv = adc_balanced_counts_to_mv(counts[CH_X]);
                int32 y_mv = adc_balanced_counts_to_mv(counts[CH_Y]);
                sprintf(tx,
                    "X:%ld mV Y:%ld mV  pan:%d/1000  tilt:%d/1000\r\n",
                    (long)x_mv, (long)y_mv, pan_milli, tilt_milli);
                UART_1_PutString(tx);
            }

            if (joystick_cal_state() == JOY_CAL_CENTER_CAPTURE) {
                uint16 n = joystick_center_samples_collected();
                if ((n % 8u) == 0u) {
                    sprintf(tx, "[CAL] Center samples: %u/%u\r\n",
                            (unsigned)n, (unsigned)CAL_CENTER_SAMPLES);
                    UART_1_PutString(tx);
                }
            }
        }

        /* ----------------------------------------------------------------
         * STEP 4  Build control struct
         * ---------------------------------------------------------------- */
        ctl.enable = 1u;
        ctl.kill   = 0u;

        if (abs_preset_active && ctrl_mode == ABSOLUTE)
        {
            /* Test 2: preset absolute angles override joystick */
            ctl.pan_mode  = ABSOLUTE;
            ctl.tilt_mode = ABSOLUTE;
            ctl.roll_mode = DEFER;
            ctl.pan  = abs_pan_target;
            ctl.tilt = abs_tilt_target;
            ctl.roll = 0.0f;
        }
        else if (joystick_active && joystick_cal_state() == JOY_CAL_OFF)
        {
            ctl.pan_mode  = ctrl_mode;
            ctl.tilt_mode = ctrl_mode;
            ctl.roll_mode = DEFER;
            ctl.pan  =  cmd.u[CH_X];
            ctl.tilt = -cmd.u[CH_Y];
            ctl.roll =  0.0f;
        }
        else
        {
            ctl.pan_mode  = DEFER;
            ctl.tilt_mode = DEFER;
            ctl.roll_mode = DEFER;
            ctl.pan  = 0.0f;
            ctl.tilt = 0.0f;
            ctl.roll = 0.0f;
        }

        /* ----------------------------------------------------------------
         * STEP 5  Transmit QX277 (MOVI_CONTROL only)
         * ---------------------------------------------------------------- */
        if (app_mode == APP_MODE_MOVI_CONTROL
            && joystick_cal_state() == JOY_CAL_OFF
            && (g_tick_ms - last_tx_ms) >= MOVI_TX_PERIOD_MS)
        {
            last_tx_ms = g_tick_ms;
            movi_comm_set_control(&g_movi, &ctl);
            movi_comm_send_control(&g_movi);
        }

        /* ----------------------------------------------------------------
         * STEP 6  1 Hz diagnostic (MOVI_CONTROL only)
         * ---------------------------------------------------------------- */
        if (diag_active
            && app_mode == APP_MODE_MOVI_CONTROL
            && joystick_cal_state() == JOY_CAL_OFF
            && (g_tick_ms - last_diag_ms) >= DIAG_PERIOD_MS)
        {
            last_diag_ms = g_tick_ms;

            uint32 ms_total = g_tick_ms;
            uint32 sec_part = (ms_total / 1000u)   % 60u;
            uint32 min_part = (ms_total / 60000u)  % 60u;
            uint32 hr_part  =  ms_total / 3600000u;
            sprintf(tx, "  %02lu:%02lu:%02lu\r\n",
                    (unsigned long)hr_part,
                    (unsigned long)min_part,
                    (unsigned long)sec_part);
            UART_1_PutString(tx);

            movi_statistics_t s;
            movi_comm_get_statistics(&g_movi, &s);
            sprintf(tx,
                "[Serial]  TX:%lu RX:%lu | csum:%lu err:0x%02X | %s%s\r\n",
                (unsigned long)s.tx_packets,
                (unsigned long)s.rx_packets,
                (unsigned long)s.rx_bad_checksum,
                (unsigned)s.uart_err_flags,
                joystick_active ? "JOY " : "",
                abs_preset_active ? "PRESET" : "");
            UART_1_PutString(tx);

            movi_status_t st;
            movi_comm_get_status(&g_movi, &st);

            if (st.valid) {
                char pan_s, tilt_s, roll_s;
                int16 pan_i, pan_f, tilt_i, tilt_f, roll_i, roll_f;
                compute_euler_ints(&st,
                    &pan_s,  &pan_i,  &pan_f,
                    &tilt_s, &tilt_i, &tilt_f,
                    &roll_s, &roll_i, &roll_f);

                int16 vl_i = (int16)st.battery_left_v;
                int16 vl_f = (int16)((st.battery_left_v  - (float)vl_i) * 10.0f);
                int16 vr_i = (int16)st.battery_right_v;
                int16 vr_f = (int16)((st.battery_right_v - (float)vr_i) * 10.0f);

                uint8 imu_err = (st.gimbal_status1 >> 6) & 0x01u;
                uint8 drv_err = (st.gimbal_status1 >> 4) & 0x01u;
                uint8 gbl_err = (st.gimbal_status2 >> 4) & 0x01u;

                /* Show preset target in ABS mode */
                if (abs_preset_active) {
                    char ps; int16 pi, pf, ti, tf;
                    decomp(abs_pan_target  * 180.0f, &pi, &pf, &ps);
                    char ts2; decomp(abs_tilt_target * 180.0f, &ti, &tf, &ts2);
                    sprintf(tx, "[Target]  Pan=%c%3d.%02d  Tilt=%c%3d.%02d deg (ABS preset)\r\n",
                            ps, (int)pi, (int)pf, ts2, (int)ti, (int)tf);
                    UART_1_PutString(tx);
                }

                sprintf(tx,
                    "[Gimbal]  Pan=%c%3d.%02d  Tilt=%c%3d.%02d  Roll=%c%3d.%02d"
                    " | L=%2d.%1dV R=%2d.%1dV | IMU:%d DRV:%d ERR:%d\r\n",
                    pan_s,  (int)pan_i,  (int)pan_f,
                    tilt_s, (int)tilt_i, (int)tilt_f,
                    roll_s, (int)roll_i, (int)roll_f,
                    (int)vl_i, (int)vl_f,
                    (int)vr_i, (int)vr_f,
                    (int)imu_err, (int)drv_err, (int)gbl_err);
                UART_1_PutString(tx);

            } else {
                UART_1_PutString("[Gimbal]  No valid frame yet\r\n");
            }

            /* M2: platform yaw alongside gimbal yaw */
            if (show_platform_yaw && ctrl_attr_pending == 0u) {
                movi_comm_request_attr(&g_movi, 2u);
                g_movi.test_sent_ms = g_tick_ms;
                ctrl_attr_pending   = 2u;
                /* Result will print via STEP 2f when response arrives */
            }

            char lr_s, ud_s  ;
            int16 lr_i, lr_f, ud_i, ud_f;
            decomp(cmd.u[CH_X], &lr_i, &lr_f, &lr_s);
            decomp(cmd.u[CH_Y], &ud_i, &ud_f, &ud_s);
            sprintf(tx,
                "[Joystick] L/R=%c%d.%02d (%s)  U/D=%c%d.%02d (%s)\r\n",
                lr_s, (int)lr_i, (int)lr_f, mode_to_str(ctrl_mode),
                ud_s, (int)ud_i, (int)ud_f, mode_to_str(ctrl_mode));
            UART_1_PutString(tx);

            g_movi.statistics.uart_err_flags = 0u;
        }
    }
}

/* [] END OF FILE */