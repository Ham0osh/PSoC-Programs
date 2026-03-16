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
 *
 * CLI keys — STANDBY / MOVI_CONTROL:
 *   m : toggle STANDBY <-> MOVI_CONTROL
 *   j : toggle joystick active (feeds pan/tilt commands)
 *   l : cycle control mode DEFER -> RATE -> ABSOLUTE -> DEFER
 *   k : kill motion, return to STANDBY, keep printing
 *   x : kill motion, return to STANDBY, stop printing
 *   s : toggle raw ADC stream
 *   c : calibration advance
 *   t : enter MOVI_TEST mode (from STANDBY only)
 *
 * CLI keys — MOVI_TEST:
 *   a : request attr 22  — gimbal attitude (euler: roll/pitch/yaw + errors + rates)
 *   g : request attr  4  — GPS (lat/lon/alt/speed/heading/accuracy)
 *   b : request attr  3  — baro altitude + rate of climb
 *   s : request attr  1  — system status (battery, sats, temp, CPU, pressure)
 *   n : request attr 12  — magnetometer (XYZ, magnitude, freq, offsets, declination)
 *   t : exit MOVI_TEST, return to STANDBY
 *   k/x : kill + exit to STANDBY
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
#define MOVI_TX_PERIOD_MS     100u   /* 10 Hz control TX                     */
#define DIAG_PERIOD_MS        1000u  /* 1 Hz diagnostic print (MOVI_CONTROL) */
#define TEST_TIMEOUT_MS       2000u  /* test mode: max wait for response      */

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
 * Helpers
 * ========================================================================= */

/* Decompose signed float to sign char + integer + 2-decimal fraction. */
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
 * kill helper — common to 'k' and 'x'
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
 * Test mode verbose print functions
 * Each prints one header line then per-field lines showing:
 *   offset | type/scale annotation | raw value | => | scaled value + unit
 * This lets you cross-check the java ParameterStructure offsets against
 * what actually arrives in the packet.
 * ========================================================================= */

/* Helper: split int32 at a power-of-10 boundary for printing without %f.
 * Returns sign char, integer part, and fractional part as separate ints.
 * scale_pow10 must be 10, 100, 1000, or 10000000.                        */
static char split32(int32_t raw, int32_t scale_pow10,
                    long* i_out, long* f_out)
{
    char sign = (raw < 0) ? '-' : '+';
    int32_t abs = (raw < 0) ? -raw : raw;
    *i_out = (long)(abs / scale_pow10);
    *f_out = (long)(abs % scale_pow10);
    return sign;
}

/* -------------------------------------------------------------------------
 * Test print functions — all take (dt, rxlen, snapshot_payload, snap_plen).
 * Raw values are read directly from the snapshot bytes (p), not from
 * movi_status_t floats, to guarantee we see exactly what the GCU sent
 * before the next attr 287 push could overwrite RxMsg.
 * ------------------------------------------------------------------------- */

/* Macro: read int16 LE from snapshot at byte offset off.
 * Returns 0 if snapshot is too short.                    */
#define SNAP16(p, plen, off) \
    (((off) + 2u <= (plen)) ? ({ int16_t _v; memcpy(&_v, (p)+(off), 2); _v; }) : (int16_t)0)

/* Macro: read int32 LE from snapshot at byte offset off. */
#define SNAP32(p, plen, off) \
    (((off) + 4u <= (plen)) ? ({ int32_t _v; memcpy(&_v, (p)+(off), 4); _v; }) : (int32_t)0)

/* Macro: read int16 big-endian from snapshot. */
#define SNAP16BE(p, plen, off) \
    (((off) + 2u <= (plen)) \
        ? (int16_t)(((uint16_t)(p)[(off)] << 8) | (uint16_t)(p)[(off)+1u]) \
        : (int16_t)0)

/* Macro: read int32 big-endian from snapshot. */
#define SNAP32BE(p, plen, off) \
    (((off) + 4u <= (plen)) \
        ? (int32_t)(  ((uint32_t)(p)[(off)]   << 24) \
                    | ((uint32_t)(p)[(off)+1u] << 16) \
                    | ((uint32_t)(p)[(off)+2u] <<  8) \
                    |  (uint32_t)(p)[(off)+3u]) \
        : (int32_t)0)

/* --- attr 22: Gimbal attitude (euler) --- */
static void print_test_attitude(const movi_status_t* st, uint32_t dt,
                                 uint16_t rxlen, const uint8_t* p, uint8_t plen)
{
    char buf[TRANSMIT_BUFFER_SIZE];
    (void)st;  /* parsed floats not used — we read direct from snapshot */

    sprintf(buf, "\r\n[TEST][Att] attr=22  dt=%lums  rxlen=%u  snaplen=%u\r\n",
            (unsigned long)dt, (unsigned)rxlen, (unsigned)plen);
    UART_1_PutString(buf);

    /* Raw hex dump — all snapshot bytes, 8 per row.
     * Lets us verify exact wire bytes vs Java ParameterStructure offsets.
     * Format:  off00: XX XX XX XX  XX XX XX XX               */
    {
        uint8_t row, col;
        for (row = 0u; row < plen; row += 8u) {
            char* bp = buf;
            bp += sprintf(bp, "  off%02u:", (unsigned)row);
            for (col = 0u; col < 8u && (row + col) < plen; col++) {
                bp += sprintf(bp, " %02X", (unsigned)p[row + col]);
            }
            bp += sprintf(bp, "\r\n");
            UART_1_PutString(buf);
        }
    }

    if (plen < 6u) {
        sprintf(buf, "  ERROR: snapshot only %u bytes -- BLE bridge truncation?\r\n",
                (unsigned)plen);
        UART_1_PutString(buf);
        return;
    }

    char s; int16 vi, vf;
    int16_t r1 = SNAP16(p, plen,  1);
    int16_t r3 = SNAP16(p, plen,  3);
    int16_t r5 = SNAP16(p, plen,  5);

    /* Axis order confirmed from live GCU data: off1=Pitch off3=Yaw off5=Roll */
    decomp((float)r1 / 100.0f, &vi, &vf, &s);
    sprintf(buf, "  off  1  int16  /100   raw=%6d  =>  %c%d.%02d deg  [PITCH]\r\n",
            (int)r1, s, (int)vi, (int)vf);
    UART_1_PutString(buf);

    decomp((float)r3 / 100.0f, &vi, &vf, &s);
    sprintf(buf, "  off  3  int16  /100   raw=%6d  =>  %c%d.%02d deg  [YAW]\r\n",
            (int)r3, s, (int)vi, (int)vf);
    UART_1_PutString(buf);

    decomp((float)r5 / 100.0f, &vi, &vf, &s);
    sprintf(buf, "  off  5  int16  /100   raw=%6d  =>  %c%d.%02d deg  [ROLL]\r\n",
            (int)r5, s, (int)vi, (int)vf);
    UART_1_PutString(buf);

    if (plen < 19u) {
        sprintf(buf, "  (snap only %u bytes -- err/rate fields off 7-17 unavailable)\r\n",
                (unsigned)plen);
        UART_1_PutString(buf);
        return;
    }

    UART_1_PutString("  -- error + rate fields (off 7-17) --\r\n");
    int16_t r7  = SNAP16(p, plen,  7);
    int16_t r9  = SNAP16(p, plen,  9);
    int16_t r11 = SNAP16(p, plen, 11);
    int16_t r13 = SNAP16(p, plen, 13);
    int16_t r15 = SNAP16(p, plen, 15);
    int16_t r17 = SNAP16(p, plen, 17);

    decomp((float)r7  / 10.0f, &vi, &vf, &s);
    sprintf(buf, "  off  7  int16  /10    raw=%6d  =>  %c%d.%02d  [roll_err deg]\r\n",
            (int)r7, s, (int)vi, (int)vf);
    UART_1_PutString(buf);

    decomp((float)r9  / 10.0f, &vi, &vf, &s);
    sprintf(buf, "  off  9  int16  /10    raw=%6d  =>  %c%d.%02d  [pitch_err deg]\r\n",
            (int)r9, s, (int)vi, (int)vf);
    UART_1_PutString(buf);

    decomp((float)r11 / 10.0f, &vi, &vf, &s);
    sprintf(buf, "  off 11  int16  /10    raw=%6d  =>  %c%d.%02d  [yaw_err deg]\r\n",
            (int)r11, s, (int)vi, (int)vf);
    UART_1_PutString(buf);

    decomp((float)r13 / 10.0f, &vi, &vf, &s);
    sprintf(buf, "  off 13  int16  /10    raw=%6d  =>  %c%d.%02d  [roll_rate_cmd deg/s]\r\n",
            (int)r13, s, (int)vi, (int)vf);
    UART_1_PutString(buf);

    decomp((float)r15 / 10.0f, &vi, &vf, &s);
    sprintf(buf, "  off 15  int16  /10    raw=%6d  =>  %c%d.%02d  [pitch_rate_cmd deg/s]\r\n",
            (int)r15, s, (int)vi, (int)vf);
    UART_1_PutString(buf);

    decomp((float)r17 / 10.0f, &vi, &vf, &s);
    sprintf(buf, "  off 17  int16  /10    raw=%6d  =>  %c%d.%02d  [yaw_rate_cmd deg/s]\r\n",
            (int)r17, s, (int)vi, (int)vf);
    UART_1_PutString(buf);
}

/* --- attr 4: GPS ---
 * Big-endian, no sub-index. off 0=lon, off 4=lat (confirmed). */
static void print_test_gps(const movi_status_t* st, uint32_t dt,
                            uint16_t rxlen, const uint8_t* p, uint8_t plen)
{
    char buf[TRANSMIT_BUFFER_SIZE];
    (void)st;

    sprintf(buf, "\r\n[TEST][GPS] attr=4  dt=%lums  rxlen=%u  snaplen=%u\r\n",
            (unsigned long)dt, (unsigned)rxlen, (unsigned)plen);
    UART_1_PutString(buf);

    /* Raw hex dump */
    {
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

    if (plen < 12u) {
        sprintf(buf, "  ERROR: snapshot only %u bytes -- need 12 minimum\r\n",
                (unsigned)plen);
        UART_1_PutString(buf);
        return;
    }

    long i_part, f_part;
    char sign;
    char s; int16 vi, vf;

    /* int32 BE, no sub-index: lon at off 0, lat at off 4 */
    int32_t lon_r = SNAP32BE(p, plen,  0);
    int32_t lat_r = SNAP32BE(p, plen,  4);
    int32_t alt_r = SNAP32BE(p, plen,  8);

    sign = split32(lon_r, 10000000L, &i_part, &f_part);
    sprintf(buf, "  off  0  int32BE /1e7   raw=%ld  =>  %c%ld.%07ld deg  [LON]\r\n",
            (long)lon_r, sign, i_part, f_part);
    UART_1_PutString(buf);

    sign = split32(lat_r, 10000000L, &i_part, &f_part);
    sprintf(buf, "  off  4  int32BE /1e7   raw=%ld  =>  %c%ld.%07ld deg  [LAT]\r\n",
            (long)lat_r, sign, i_part, f_part);
    UART_1_PutString(buf);

    sign = split32(alt_r, 1000L, &i_part, &f_part);
    sprintf(buf, "  off  8  int32BE /1000  raw=%ld  =>  %c%ld.%03ld m MSL  [ALT]\r\n",
            (long)alt_r, sign, i_part, f_part);
    UART_1_PutString(buf);

    if (plen < 22u) {
        sprintf(buf, "  (snap only %u bytes -- speed/hdg/acc fields unavailable)\r\n",
                (unsigned)plen);
        UART_1_PutString(buf);
        return;
    }

    int16_t spd_r  = SNAP16BE(p, plen, 12);
    int16_t hdg_r  = SNAP16BE(p, plen, 14);
    int16_t hacc_r = SNAP16BE(p, plen, 16);
    int16_t vacc_r = SNAP16BE(p, plen, 18);
    int16_t sacc_r = SNAP16BE(p, plen, 20);

    decomp((float)spd_r  / 100.0f, &vi, &vf, &s);
    sprintf(buf, "  off 12  int16BE /100   raw=%6d  =>  %c%d.%02d m/s  [GND SPD]\r\n",
            (int)spd_r, s, (int)vi, (int)vf);
    UART_1_PutString(buf);

    decomp((float)hdg_r  / 10.0f,  &vi, &vf, &s);
    sprintf(buf, "  off 14  int16BE /10    raw=%6d  =>  %c%d.%02d deg  [HDG]\r\n",
            (int)hdg_r, s, (int)vi, (int)vf);
    UART_1_PutString(buf);

    decomp((float)hacc_r / 100.0f, &vi, &vf, &s);
    sprintf(buf, "  off 16  int16BE /100   raw=%6d  =>  %c%d.%02d m  [HACC]\r\n",
            (int)hacc_r, s, (int)vi, (int)vf);
    UART_1_PutString(buf);

    decomp((float)vacc_r / 100.0f, &vi, &vf, &s);
    sprintf(buf, "  off 18  int16BE /100   raw=%6d  =>  %c%d.%02d m  [VACC]\r\n",
            (int)vacc_r, s, (int)vi, (int)vf);
    UART_1_PutString(buf);

    decomp((float)sacc_r / 100.0f, &vi, &vf, &s);
    sprintf(buf, "  off 20  int16BE /100   raw=%6d  =>  %c%d.%02d m/s  [SACC]\r\n",
            (int)sacc_r, s, (int)vi, (int)vf);
    UART_1_PutString(buf);
}

/* --- attr 3: Baro ---
 * Endianness and sub-index unconfirmed — hex dump included for verification. */
static void print_test_baro(const movi_status_t* st, uint32_t dt,
                             uint16_t rxlen, const uint8_t* p, uint8_t plen)
{
    char buf[TRANSMIT_BUFFER_SIZE];
    (void)st;

    sprintf(buf, "\r\n[TEST][Baro] attr=3  dt=%lums  rxlen=%u  snaplen=%u\r\n",
            (unsigned long)dt, (unsigned)rxlen, (unsigned)plen);
    UART_1_PutString(buf);

    /* Hex dump — needed to determine correct endianness / sub-index */
    {
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

    if (plen < 4u) {
        sprintf(buf, "  ERROR: snapshot only %u bytes\r\n", (unsigned)plen);
        UART_1_PutString(buf);
        return;
    }

    char s; int16 vi, vf;

    /* Try both sub-index (off 1) and no-sub-index (off 0) interpretations.
     * GPS and sysstat have no sub-index (BE from off 0).
     * Attr 22 has sub-index (LE from off 1).
     * Compare against a known altitude reference to determine which is correct. */
    int16_t alt0_le = SNAP16(p, plen, 0);   /* LE from off 0 (no sub-ix) */
    int16_t alt0_be = SNAP16BE(p, plen, 0); /* BE from off 0 (no sub-ix) */
    int16_t alt1_le = SNAP16(p, plen, 1);   /* LE from off 1 (sub-ix)    */

    decomp((float)alt0_le / 10.0f, &vi, &vf, &s);
    sprintf(buf, "  off  0  int16LE /10  raw=%6d  =>  %c%d.%02d m  [ALT if no-subix LE]\r\n",
            (int)alt0_le, s, (int)vi, (int)vf);
    UART_1_PutString(buf);

    decomp((float)alt0_be / 10.0f, &vi, &vf, &s);
    sprintf(buf, "  off  0  int16BE /10  raw=%6d  =>  %c%d.%02d m  [ALT if no-subix BE]\r\n",
            (int)alt0_be, s, (int)vi, (int)vf);
    UART_1_PutString(buf);

    decomp((float)alt1_le / 10.0f, &vi, &vf, &s);
    sprintf(buf, "  off  1  int16LE /10  raw=%6d  =>  %c%d.%02d m  [ALT if sub-ix LE]\r\n",
            (int)alt1_le, s, (int)vi, (int)vf);
    UART_1_PutString(buf);

    UART_1_PutString("  -- ROC --\r\n");
    if (plen >= 6u) {
        int16_t roc0_le = SNAP16(p, plen, 2);
        int16_t roc0_be = SNAP16BE(p, plen, 2);
        int16_t roc1_le = SNAP16(p, plen, 3);
        decomp((float)roc0_le / 100.0f, &vi, &vf, &s);
        sprintf(buf, "  off  2  int16LE /100 raw=%6d  =>  %c%d.%02d m/s  [ROC no-subix LE]\r\n",
                (int)roc0_le, s, (int)vi, (int)vf);
        UART_1_PutString(buf);
        decomp((float)roc0_be / 100.0f, &vi, &vf, &s);
        sprintf(buf, "  off  2  int16BE /100 raw=%6d  =>  %c%d.%02d m/s  [ROC no-subix BE]\r\n",
                (int)roc0_be, s, (int)vi, (int)vf);
        UART_1_PutString(buf);
        decomp((float)roc1_le / 100.0f, &vi, &vf, &s);
        sprintf(buf, "  off  3  int16LE /100 raw=%6d  =>  %c%d.%02d m/s  [ROC sub-ix LE]\r\n",
                (int)roc1_le, s, (int)vi, (int)vf);
        UART_1_PutString(buf);
    }
    UART_1_PutString("  Compare altitude to known barometric reference to pick correct interpretation.\r\n");
}

/* --- attr 1: System status ---
 * Big-endian, no sub-index. Fields off 0-10 confirmed; off 11+ provisional. */
static void print_test_sysstat(const movi_status_t* st, uint32_t dt,
                                uint16_t rxlen, const uint8_t* p, uint8_t plen)
{
    char buf[TRANSMIT_BUFFER_SIZE];
    (void)st;

    sprintf(buf, "\r\n[TEST][Stat] attr=1  dt=%lums  rxlen=%u  snaplen=%u\r\n",
            (unsigned long)dt, (unsigned)rxlen, (unsigned)plen);
    UART_1_PutString(buf);

    /* Raw hex dump */
    {
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

    if (plen < 5u) {
        sprintf(buf, "  ERROR: snapshot only %u bytes\r\n", (unsigned)plen);
        UART_1_PutString(buf);
        return;
    }

    char s; int16 vi, vf;

    /* --- Confirmed fields --- */
    int16_t bv_r = SNAP16BE(p, plen, 0);
    decomp((float)bv_r / 100.0f, &vi, &vf, &s);
    sprintf(buf, "  off  0  int16BE /100   raw=%6d  =>  %c%d.%02d V  [BATT V] *\r\n",
            (int)bv_r, s, (int)vi, (int)vf);
    UART_1_PutString(buf);

    sprintf(buf, "  off  2  uint8   /1     raw=%3u  =>  %u  [GPS SATS] *\r\n",
            (unsigned)p[2], (unsigned)p[2]);
    UART_1_PutString(buf);

    sprintf(buf, "  off  3  uint8   /3     raw=%3u  =>  %d.%02d C  [TEMP] *\r\n",
            (unsigned)p[3], (int)(p[3]/3u), (int)((p[3]%3u)*33u));
    UART_1_PutString(buf);

    if (plen >= 5u) {
        sprintf(buf, "  off  4  uint8   /1     raw=%3u  =>  %u%%  [CPU]\r\n",
                (unsigned)p[4], (unsigned)p[4]);
        UART_1_PutString(buf);
    }
    if (plen >= 7u) {
        int16_t st_r = SNAP16BE(p, plen, 5);
        sprintf(buf, "  off  5  int16BE flags  raw=0x%04X  [STATUS FLAGS]\r\n",
                (unsigned)(uint16_t)st_r);
        UART_1_PutString(buf);
    }
    if (plen >= 9u) {
        int16_t time_r = SNAP16BE(p, plen, 7);
        sprintf(buf, "  off  7  int16BE /1     raw=%6d  =>  %d s  [TIME]\r\n",
                (int)time_r, (int)time_r);
        UART_1_PutString(buf);
    }
    if (plen >= 11u) {
        int16_t pres_r = SNAP16BE(p, plen, 9);
        decomp((float)pres_r / 10.0f, &vi, &vf, &s);
        sprintf(buf, "  off  9  int16BE /10    raw=%6d  =>  %c%d.%02d mb  [PRESSURE] *\r\n",
                (int)pres_r, s, (int)vi, (int)vf);
        UART_1_PutString(buf);
    }

    UART_1_PutString("  -- fields below provisional (not yet iOS-verified) --\r\n");

    if (plen >= 13u) {
        int16_t imu_r = SNAP16BE(p, plen, 11);
        sprintf(buf, "  off 11  int16BE /1     raw=%6d  [IMU RATE]\r\n", (int)imu_r);
        UART_1_PutString(buf);
    }
    if (plen >= 15u) {
        int16_t cam_r = SNAP16BE(p, plen, 13);
        sprintf(buf, "  off 13  int16BE flags  raw=0x%04X  [CAM STATUS]\r\n",
                (unsigned)(uint16_t)cam_r);
        UART_1_PutString(buf);
    }
    if (plen >= 18u) {
        int16_t lat_r = SNAP16BE(p, plen, 16);
        sprintf(buf, "  off 16  int16BE /1     raw=%6d  [IMU LATENCY]\r\n", (int)lat_r);
        UART_1_PutString(buf);
    }
    if (plen >= 20u) {
        int16_t ba_r = SNAP16BE(p, plen, 18);
        decomp((float)ba_r / 100.0f, &vi, &vf, &s);
        sprintf(buf, "  off 18  int16BE /100   raw=%6d  =>  %c%d.%02d A  [BATT A]\r\n",
                (int)ba_r, s, (int)vi, (int)vf);
        UART_1_PutString(buf);
    }
    if (plen >= 22u) {
        int16_t chg_r = SNAP16BE(p, plen, 20);
        sprintf(buf, "  off 20  int16BE /1     raw=%6d  =>  %d mAh  [CHARGE USED]\r\n",
                (int)chg_r, (int)chg_r);
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

    if (plen < 6u) {
        sprintf(buf, "  ERROR: snapshot only %u bytes\r\n", (unsigned)plen);
        UART_1_PutString(buf);
        return;
    }

    char s; int16 vi, vf;

    int16_t x_r  = SNAP16(p, plen,  1);
    int16_t y_r  = SNAP16(p, plen,  3);
    int16_t z_r  = SNAP16(p, plen,  5);

    decomp((float)x_r / 1000.0f, &vi, &vf, &s);
    sprintf(buf, "  off  1  int16  /1000  raw=%6d  =>  %c%d.%02d  [MAG X]\r\n",
            (int)x_r, s, (int)vi, (int)vf);
    UART_1_PutString(buf);

    decomp((float)y_r / 1000.0f, &vi, &vf, &s);
    sprintf(buf, "  off  3  int16  /1000  raw=%6d  =>  %c%d.%02d  [MAG Y]\r\n",
            (int)y_r, s, (int)vi, (int)vf);
    UART_1_PutString(buf);

    decomp((float)z_r / 1000.0f, &vi, &vf, &s);
    sprintf(buf, "  off  5  int16  /1000  raw=%6d  =>  %c%d.%02d  [MAG Z]\r\n",
            (int)z_r, s, (int)vi, (int)vf);
    UART_1_PutString(buf);

    if (plen >= 11u) {
        int16_t mag_r  = SNAP16(p, plen,  7);
        int16_t freq_r = SNAP16(p, plen,  9);
        decomp((float)mag_r  / 100.0f, &vi, &vf, &s);
        sprintf(buf, "  off  7  int16  /100   raw=%6d  =>  %c%d.%02d  [MAGNITUDE]\r\n",
                (int)mag_r, s, (int)vi, (int)vf);
        UART_1_PutString(buf);
        decomp((float)freq_r / 100.0f, &vi, &vf, &s);
        sprintf(buf, "  off  9  int16  /100   raw=%6d  =>  %c%d.%02d Hz  [FREQ]\r\n",
                (int)freq_r, s, (int)vi, (int)vf);
        UART_1_PutString(buf);
    }
    if (plen >= 17u) {
        int16_t ox_r = SNAP16(p, plen, 11);
        int16_t oy_r = SNAP16(p, plen, 13);
        int16_t oz_r = SNAP16(p, plen, 15);
        decomp((float)ox_r / 1000.0f, &vi, &vf, &s);
        sprintf(buf, "  off 11  int16  /1000  raw=%6d  =>  %c%d.%02d  [OFF X]\r\n",
                (int)ox_r, s, (int)vi, (int)vf);
        UART_1_PutString(buf);
        decomp((float)oy_r / 1000.0f, &vi, &vf, &s);
        sprintf(buf, "  off 13  int16  /1000  raw=%6d  =>  %c%d.%02d  [OFF Y]\r\n",
                (int)oy_r, s, (int)vi, (int)vf);
        UART_1_PutString(buf);
        decomp((float)oz_r / 1000.0f, &vi, &vf, &s);
        sprintf(buf, "  off 15  int16  /1000  raw=%6d  =>  %c%d.%02d  [OFF Z]\r\n",
                (int)oz_r, s, (int)vi, (int)vf);
        UART_1_PutString(buf);
    }
    if (plen >= 19u) {
        int16_t decl_r = SNAP16(p, plen, 17);
        decomp((float)decl_r / 10.0f, &vi, &vf, &s);
        sprintf(buf, "  off 17  int16  /10    raw=%6d  =>  %c%d.%02d deg  [DECLINATION]\r\n",
                (int)decl_r, s, (int)vi, (int)vf);
        UART_1_PutString(buf);
    }
}

/* Dispatcher: called from main loop when test_response_ready is set.
 * Passes the snapshotted payload bytes directly to each printer so
 * they read from the frozen copy, not from RxMsg (which may already
 * have been overwritten by the next attr 287 push).                  */
static void print_test_response(movi_comm_t* m, uint32_t dt)
{
    movi_status_t st;
    movi_comm_get_status(m, &st);
    uint16_t len      = m->test_rx_len;
    uint16_t attr     = m->test_response_attr;
    const uint8_t *p  = m->test_rx_payload;
    uint8_t  plen     = m->test_rx_payload_len;

    switch (attr) {
        case 22u: print_test_attitude(&st, dt, len, p, plen); break;
        case  4u: print_test_gps(&st, dt, len, p, plen);      break;
        case  3u: print_test_baro(&st, dt, len, p, plen);     break;
        case  1u: print_test_sysstat(&st, dt, len, p, plen);  break;
        case 12u: print_test_mag(&st, dt, len, p, plen);      break;
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
 * main
 * ========================================================================= */
int main(void)
{
    char tx[TRANSMIT_BUFFER_SIZE];
    uint8 ch;

    /* Application mode */
    app_mode_t app_mode = APP_MODE_STANDBY;

    /* Sub-flags (valid in any mode) */
    uint8 joystick_active = FALSE;
    uint8 streaming_adc   = FALSE;
    uint8 diag_active     = FALSE;

    ff_api_control_type_e ctrl_mode = DEFER;

    static uint32 last_tx_ms   = 0u;
    static uint32 last_diag_ms = 0u;

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
    UART_1_PutString("  m=movi ctrl  t=test mode  j=joystick\r\n");
    UART_1_PutString("  l=mode  s=adc  c=cal  k=kill  x=kill+quiet\r\n");
    UART_1_PutString("  -- In test mode --\r\n");
    UART_1_PutString("  a=attitude  g=GPS  b=baro  s=status  n=mag\r\n");
    UART_1_PutString("  t=exit test  (2s timeout per query)\r\n");
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

        /* ---- Keys active in STANDBY and MOVI_CONTROL ---- */
        if (app_mode != APP_MODE_MOVI_TEST)
        {
            if (ch == 'm' || ch == 'M')
            {
                if (app_mode == APP_MODE_STANDBY) {
                    app_mode   = APP_MODE_MOVI_CONTROL;
                    diag_active = TRUE;
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
                    UART_1_PutString("  a=attitude  g=GPS  b=baro  s=status  n=mag\r\n");
                    UART_1_PutString("  t=exit  k/x=kill+exit  (2s timeout per query)\r\n");
                } else {
                    UART_1_PutString("[CLI] Enter STANDBY first (m to stop ctrl)\r\n");
                }
            }
            else if (ch == 'j' || ch == 'J')
            {
                joystick_active = !joystick_active;
                if (joystick_active) diag_active = TRUE;
                UART_1_PutString(joystick_active
                    ? "[CLI] Joystick ON\r\n"
                    : "[CLI] Joystick OFF\r\n");
            }
            else if (ch == 'l' || ch == 'L')
            {
                if      (ctrl_mode == DEFER)    ctrl_mode = RATE;
                else if (ctrl_mode == RATE)     ctrl_mode = ABSOLUTE;
                else                             ctrl_mode = DEFER;
                sprintf(tx, "[CLI] Mode: %s\r\n", mode_to_str(ctrl_mode));
                UART_1_PutString(tx);
            }
            else if (ch == 'k' || ch == 'K')
            {
                do_kill(&g_movi, &app_mode, &joystick_active,
                        &streaming_adc, &ctrl_mode);
            }
            else if (ch == 'x' || ch == 'X')
            {
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
        }

        /* ---- Keys active only in MOVI_TEST ---- */
        else  /* app_mode == APP_MODE_MOVI_TEST */
        {
            if (ch == 't' || ch == 'T')
            {
                /* Abort any pending request */
                g_movi.test_pending_attr   = 0u;
                g_movi.test_response_ready = false;
                app_mode = APP_MODE_STANDBY;
                UART_1_PutString("[TEST] -> STANDBY\r\n");
            }
            else if (ch == 'k' || ch == 'K' || ch == 'x' || ch == 'X')
            {
                g_movi.test_pending_attr   = 0u;
                g_movi.test_response_ready = false;
                do_kill(&g_movi, &app_mode, &joystick_active,
                        &streaming_adc, &ctrl_mode);
                if (ch == 'x' || ch == 'X') {
                    diag_active = FALSE;
                    UART_1_PutString("[CLI] Output paused.\r\n");
                }
            }
            else if (ch == 'a' || ch == 'A')
            {
                movi_result_t r = movi_comm_request_attr(&g_movi, 22u);
                g_movi.test_sent_ms = g_tick_ms;   /* use PSoC timer, not QX internal */
                sprintf(tx, "[TEST] Sent attr 22 (attitude) at t=%lums  result=%s\r\n",
                        (unsigned long)g_movi.test_sent_ms,
                        r == MOVI_OK ? "OK" : "UART_ERR");
                UART_1_PutString(tx);
                UART_1_PutString("[TEST] Waiting up to 2s...\r\n");
            }
            else if (ch == 'g' || ch == 'G')
            {
                movi_result_t r = movi_comm_request_attr(&g_movi, 4u);
                g_movi.test_sent_ms = g_tick_ms;
                sprintf(tx, "[TEST] Sent attr 4 (GPS) at t=%lums  result=%s\r\n",
                        (unsigned long)g_movi.test_sent_ms,
                        r == MOVI_OK ? "OK" : "UART_ERR");
                UART_1_PutString(tx);
                UART_1_PutString("[TEST] Waiting up to 2s...\r\n");
            }
            else if (ch == 'b' || ch == 'B')
            {
                movi_result_t r = movi_comm_request_attr(&g_movi, 3u);
                g_movi.test_sent_ms = g_tick_ms;
                sprintf(tx, "[TEST] Sent attr 3 (baro) at t=%lums  result=%s\r\n",
                        (unsigned long)g_movi.test_sent_ms,
                        r == MOVI_OK ? "OK" : "UART_ERR");
                UART_1_PutString(tx);
                UART_1_PutString("[TEST] Waiting up to 2s...\r\n");
            }
            else if (ch == 's' || ch == 'S')
            {
                movi_result_t r = movi_comm_request_attr(&g_movi, 1u);
                g_movi.test_sent_ms = g_tick_ms;
                sprintf(tx, "[TEST] Sent attr 1 (system status) at t=%lums  result=%s\r\n",
                        (unsigned long)g_movi.test_sent_ms,
                        r == MOVI_OK ? "OK" : "UART_ERR");
                UART_1_PutString(tx);
                UART_1_PutString("[TEST] Waiting up to 2s...\r\n");
            }
            else if (ch == 'n' || ch == 'N')
            {
                movi_result_t r = movi_comm_request_attr(&g_movi, 12u);
                g_movi.test_sent_ms = g_tick_ms;
                sprintf(tx, "[TEST] Sent attr 12 (magnetometer) at t=%lums  result=%s\r\n",
                        (unsigned long)g_movi.test_sent_ms,
                        r == MOVI_OK ? "OK" : "UART_ERR");
                UART_1_PutString(tx);
                UART_1_PutString("[TEST] Waiting up to 2s...\r\n");
            }
        }

        /* Calibration state prompt (works in any mode) */
        joy_cal_state_t cal_now = joystick_cal_state();
        print_cal_prompt_on_state_change(cal_prev, cal_now);
        cal_prev = cal_now;

        /* ----------------------------------------------------------------
         * STEP 2  Pump RX from MoVI (always)
         * ---------------------------------------------------------------- */
        movi_comm_pump(&g_movi);

        /* ----------------------------------------------------------------
         * STEP 2b  Test mode: check for response or timeout
         * ---------------------------------------------------------------- */
        if (app_mode == APP_MODE_MOVI_TEST)
        {
            if (g_movi.test_response_ready)
            {
                uint32_t dt = g_tick_ms - g_movi.test_sent_ms;
                print_test_response(&g_movi, dt);
                g_movi.test_response_ready = false;
                g_movi.test_pending_attr   = 0u;
            }
            else if (g_movi.test_pending_attr != 0u &&
                     (g_tick_ms - g_movi.test_sent_ms) >= TEST_TIMEOUT_MS)
            {
                sprintf(tx, "[TEST] TIMEOUT -- attr=%u  no response in %lums\r\n",
                        (unsigned)g_movi.test_pending_attr,
                        (unsigned long)TEST_TIMEOUT_MS);
                UART_1_PutString(tx);
                UART_1_PutString("[TEST] Check: RX count still rising? BLE bridge alive?\r\n");
                g_movi.test_pending_attr = 0u;
            }
        }

        /* ----------------------------------------------------------------
         * STEP 3  Joystick ADC
         * ---------------------------------------------------------------- */
        uint8 current_pin_state = Pin_Sensitivity_Read();
        if (last_button_state == 1 && current_pin_state == 0) 
        {
            // 3. Debounce: Wait 50ms and check again to ignore electrical noise
            CyDelay(50); 
            if (Pin_Sensitivity_Read() == 0) 
            {
                // 4. Cycle the sensitivity
                current_sense = (joy_sensitivity_t)((current_sense + 1) % 3); 
                joystick_set_sensitivity(current_sense);
                
                // Optional: Print to confirm
                UART_1_PutString("Mode Switched!\r\n");
                
                // 5. Wait for release so it doesn't loop while holding the button
                while(Pin_Sensitivity_Read() == 0); 
            }
        }
        last_button_state = current_pin_state;
        if (adc_balanced_poll_frame(counts))
        {
            joystick_on_sample(counts);
            joystick_get_cmd(&cmd);

            if (streaming_adc)
            {
                int16 pan_milli  = (int16)(cmd.u[CH_X] * 1000.0f);
                int16 tilt_milli = (int16)(cmd.u[CH_Y] * 1000.0f);
                int32 x_mv = adc_balanced_counts_to_mv(counts[CH_X]);
                int32 y_mv = adc_balanced_counts_to_mv(counts[CH_Y]);
                sprintf(tx,
                    "X:%ld mV Y:%ld mV  pan:%d/1000  tilt:%d/1000\r\n",
                    (long)x_mv, (long)y_mv, pan_milli, tilt_milli);
                UART_1_PutString(tx);
            }

            if (joystick_cal_state() == JOY_CAL_CENTER_CAPTURE)
            {
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

        if (joystick_active && joystick_cal_state() == JOY_CAL_OFF)
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
         * STEP 5  Transmit QX277 to MoVI (MOVI_CONTROL only)
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
         * STEP 6  1 Hz diagnostic print (MOVI_CONTROL only)
         * ---------------------------------------------------------------- */
        if (diag_active
            && app_mode == APP_MODE_MOVI_CONTROL
            && joystick_cal_state() == JOY_CAL_OFF
            && (g_tick_ms - last_diag_ms) >= DIAG_PERIOD_MS)
        {
            last_diag_ms = g_tick_ms;

            /* Timestamp */
            uint32 ms_total = g_tick_ms;
            uint32 sec_part = (ms_total / 1000u)   % 60u;
            uint32 min_part = (ms_total / 60000u)  % 60u;
            uint32 hr_part  =  ms_total / 3600000u;
            sprintf(tx, "  %02lu:%02lu:%02lu\r\n",
                    (unsigned long)hr_part,
                    (unsigned long)min_part,
                    (unsigned long)sec_part);
            UART_1_PutString(tx);

            /* Serial stats */
            movi_statistics_t s;
            movi_comm_get_statistics(&g_movi, &s);
            sprintf(tx,
                "[Serial]    TX:%lu RX:%lu | csum:%lu err:0x%02X | COMMS:ON JOYSTICK:%s\r\n",
                (unsigned long)s.tx_packets,
                (unsigned long)s.rx_packets,
                (unsigned long)s.rx_bad_checksum,
                (unsigned)s.uart_err_flags,
                joystick_active ? "ON " : "OFF");
            UART_1_PutString(tx);

            /* Gimbal attitude from quaternion (attr 287 auto-push) */
            movi_status_t st;
            movi_comm_get_status(&g_movi, &st);

            if (st.valid)
            {
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

                sprintf(tx,
                    "[Status]    Pan=%c%3d.%02d  Tilt=%c%3d.%02d  Roll=%c%3d.%02d"
                    " | L=%2d.%1dV R=%2d.%1dV | IMU:%d DRV:%d ERR:%d\r\n",
                    pan_s,  (int)pan_i,  (int)pan_f,
                    tilt_s, (int)tilt_i, (int)tilt_f,
                    roll_s, (int)roll_i, (int)roll_f,
                    (int)vl_i, (int)vl_f,
                    (int)vr_i, (int)vr_f,
                    (int)imu_err, (int)drv_err, (int)gbl_err);
                UART_1_PutString(tx);
            }
            else
            {
                UART_1_PutString("[Status]    No valid frame yet\r\n");
            }

            /* Joystick */
            char lr_s, ud_s;
            int16 lr_i, lr_f, ud_i, ud_f;
            decomp(cmd.u[CH_X], &lr_i, &lr_f, &lr_s);
            decomp(cmd.u[CH_Y], &ud_i, &ud_f, &ud_s);
            sprintf(tx,
                "[Joystick]  L/R=%c%d.%02d (%s)  U/D=%c%d.%02d (%s)\r\n",
                lr_s, (int)lr_i, (int)lr_f, mode_to_str(ctrl_mode),
                ud_s, (int)ud_i, (int)ud_f, mode_to_str(ctrl_mode));
            UART_1_PutString(tx);

            /* Clear error flags */
            g_movi.statistics.uart_err_flags = 0u;
        }
    }
}

/* [] END OF FILE */