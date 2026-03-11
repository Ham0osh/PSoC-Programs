/* main.c
 * Author: Hamish Johnson
 * Org: Quantum Internet Systems Lab, SFU, Physics
 * Date: March 9 2026
 * 
 * Uses:
 *  - adc_balanced.c/.h  : round-robin ADC sampling over AMux (N_CH channels)
 *  - joystick.c/.h      : calibration + mapping (deadband, inversion, scaling)
 *  - movi_comm.c/.h     : MōVI Pro QX protocol HAL
 *
 * CLI:
 *   'j' : toggle joystick active (feeds pan/tilt commands)
 *   'm' : toggle movi comms active (sends QX packets)
 *   'l' : cycle control mode DEFER -> RATE -> ABSOLUTE -> DEFER (both axes)
 *   'k' : kill motion + reset mode to DEFER, keep printing
 *   'x' : kill motion + reset mode to DEFER + stop diagnostic printing
 *   's' : toggle raw ADC stream (debug)
 *   'c' : calibration advance
 */

// Standard imports
#include <project.h>
#include <stdio.h>
#include <math.h>

// Library and API imports
#include "adc_balanced.h"
#include "joystick.h"
#include "movi_comm.h"

// Constants
#define TRANSMIT_BUFFER_SIZE  160u   // enough for longest diagnostic line
#define MOVI_TX_PERIOD_MS     100u   // 10 Hz — safe over BLE bridge
#define DIAG_PERIOD_MS        1000u  // 1 Hz diagnostic print

#define FALSE  0
#define TRUE   1

// ADC Channels
#define CH_X (0u)  // pan
#define CH_Y (1u)  // tilt

// Helper: Decompose signed float into string (+,-) and unsigned float
//         for printing.
static void decomp(float val, int16* i_out, int16* f_out, char* sign_out)
{
    *sign_out = (val < 0.0f) ? '-' : ' ';
    if (val < 0.0f) val = -val;
    *i_out = (int16)val;
    *f_out = (int16)((val - (float)*i_out) * 100.0f);
    if (*f_out < 0) *f_out = -*f_out;
}

// Helper: Convert quaternion to euler angle.
static void compute_euler_ints(const movi_status_t* st,
                                char* pan_s,  int16* pan_i,  int16* pan_f,
                                char* tilt_s, int16* tilt_i, int16* tilt_f,
                                char* roll_s, int16* roll_i, int16* roll_f)
{
    float r  = st->gimbal_r;
    float ii = st->gimbal_i;
    float j  = st->gimbal_j;
    float k  = st->gimbal_k;

    // Roll axis
    float sinr_val = 2.0f * (r*ii + j*k);
    float cosr_val = 1.0f - 2.0f * (ii*ii + j*j);
    float roll_deg = atan2f(sinr_val, cosr_val) * 57.2958f;

    // Tilt axis
    float sinp = 2.0f * (r*j - k*ii);
    float tilt_deg = (sinp >=  1.0f) ?  90.0f :
                     (sinp <= -1.0f) ? -90.0f :
                     asinf(sinp) * 57.2958f;

    // Pan axis
    float siny_val = 2.0f * (r*k + ii*j);
    float cosy_val = 1.0f - 2.0f * (j*j + k*k);
    float pan_deg  = atan2f(siny_val, cosy_val) * 57.2958f;
    
    // Optional if youre copying this to other code, my sprintf was not
    // happy with floats...
    decomp(pan_deg,  pan_i,  pan_f,  pan_s);
    decomp(tilt_deg, tilt_i, tilt_f, tilt_s);
    decomp(roll_deg, roll_i, roll_f, roll_s);
}

// Helper: Control type to string for printing to CLI.
static const char* mode_to_str(ff_api_control_type_e m)
{
    switch (m) {
        case RATE:     return "RATE";
        case ABSOLUTE: return "ABS ";
        default:       return "DEFR";
    }
}

// Calibration mode prompter
// Handles printing prompts during calibration of the joystick voltage.
static void print_cal_prompt_on_state_change(joy_cal_state_t prev,
                                              joy_cal_state_t now)
{
    if (prev == now) return;

    if (now == JOY_CAL_RANGE_CAPTURE) {
        UART_1_PutString("\r\n[CAL] Step 1: Move joystick through FULL range, then release to center.\r\n");
        UART_1_PutString("[CAL] Press 'c' to proceed to center capture, 'x' to cancel.\r\n");
    } else if (now == JOY_CAL_CENTER_CAPTURE) {
        UART_1_PutString("\r\n[CAL] Step 2: Hold center — averaging...\r\n");
    } else if (now == JOY_CAL_CONFIRM_SAVE) {
        UART_1_PutString("\r\n[CAL] Step 3: Center captured. Press 'c' to SAVE, 'x' to cancel.\r\n");
    } else if (now == JOY_CAL_OFF && prev != JOY_CAL_OFF) {
        UART_1_PutString("[CAL] Exited.\r\n");
    }
}

/* =========================================================================
 * Initialize movi connection!
 * 
 * In order to handle the Movi gracefully in C, and allow multiple connections,
 * we need to create a hardware abstraction layer (HAL). Rather than the code 
 * calling the serial directly, we pass the code a pointer to the hardware!
 *
 * It also means its agnostic to comm type! We can give it a way to put to
 * UART just as well as CAN, Linux serial, or a loopback. The three parts are:
 * 1. movi_uart_putc function - The only piece that talks to the hardware.
 * 2. movi_hal_t struct - Holds pointer to the putc function
 * 3. ctx pointer - Handling multiple gimbals. If multiple are needed, the putc
 *    would need to take the ctx as a context for the UART handle to use in a 
 * specific command.
 * 
 * HAL only handles Tx to the Movi, the Rx is handled by an ISR which does not
 * differentiate between two devices, would need seperate UART UDB with its own
 * ISR.
 *
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

// ISRs
CY_ISR(isr_rx_movi_Handler)
{
    uint8 st;
    do {
        st = UART_MOVI_RXSTATUS_REG;
        
        // Check errors
        if (st & (UART_MOVI_RX_STS_BREAK     |
                  UART_MOVI_RX_STS_PAR_ERROR |
                  UART_MOVI_RX_STS_STOP_ERROR|
                  UART_MOVI_RX_STS_OVERRUN))
        {
            // Handle errors if found
            movi_comm_on_uart_error_flags(&g_movi,
                (uint8)(st & (UART_MOVI_RX_STS_BREAK     |
                              UART_MOVI_RX_STS_PAR_ERROR |
                              UART_MOVI_RX_STS_STOP_ERROR|
                              UART_MOVI_RX_STS_OVERRUN)));
        }
        
        // Check if buffer filled and read until empty
        if (st & UART_MOVI_RX_STS_FIFO_NOTEMPTY) {
            uint8 b = UART_MOVI_RXDATA_REG;
            movi_comm_on_rx_byte(&g_movi, b);
        }
    } while (st & UART_MOVI_RX_STS_FIFO_NOTEMPTY);
}

// Simple millisecond incrementing timer
volatile uint32 g_tick_ms    = 0u;
CY_ISR(isr_Looptimer_Handler)
{
    g_tick_ms++;
    Looptimer_ReadStatusRegister();
}

// Helper: Verbose kill toggle for this UI
static void do_kill(movi_comm_t* movi,
                    uint8* joystick_active,
                    uint8* movi_active,
                    uint8* streaming_adc,
                    ff_api_control_type_e* mode)
{
    if (*movi_active) {
        movi_comm_kill(movi);
        UART_1_PutString("[CLI] Kill packet sent\r\n");
    }
    // Set flags to false after kill
    *joystick_active = FALSE;
    *movi_active     = FALSE;
    *streaming_adc   = FALSE;
    *mode            = DEFER;
    joystick_on_key('x');
    UART_1_PutString("[CLI] All stopped  Mode: DEFR\r\n");
}

/* =========================================================================
 * main
 * ========================================================================= */
int main(void)
{
    char tx[TRANSMIT_BUFFER_SIZE];
    uint8 ch;

    // State Machine flags
    uint8 joystick_active = FALSE;  // 'j' — feeds joystick -> ctl
    uint8 movi_active     = FALSE;  // 'm' — sends QX packets to MoVI
    uint8 streaming_adc   = FALSE;  // 's' — raw ADC debug stream
    uint8 diag_active     = FALSE;  // printing suppressed until j or m ON

    // Control mode of both axis
    ff_api_control_type_e ctrl_mode = DEFER;

    // Timers
    static uint32 last_tx_ms = 0u;    // For movi comms.
    static uint32 last_diag_ms = 0u;  // For human monitor

    // Joystick & ADC
    int16     counts[N_CH];
    joy_cmd_t cmd;
    cmd.u[0] = 0.0f;
    cmd.u[1] = 0.0f;

    // Movi Pro - Control Packet Struct
    movi_control_t ctl;

    // Joystick calibration
    joy_cal_state_t cal_prev = JOY_CAL_OFF;
    joy_cal_t defaults;  // Defaults
    uint8 i;
    for (i = 0u; i < (uint8)N_CH; i++) {
        defaults.minv[i]   = 0;
        defaults.maxv[i]   = 255;
        defaults.center[i] = 128;
    }
    defaults.valid = 0u;

    // TODO: This is a hardcoded flip of the x axis due to my breadboard 
    // orientation
    uint32 invert_mask = (1u << CH_X);

    /* ------------------------------------------------------------------
     * Hardware init
     * ------------------------------------------------------------------ */
    CyGlobalIntEnable;

    UART_1_Start();
    UART_1_PutString("\r\n========================================\r\n");
    UART_1_PutString("  MoVI Pro Controller  PSoC 5LP\r\n");
    UART_1_PutString("========================================\r\n");
    UART_1_PutString("  j=joystick  m=movi  l=mode  s=adc\r\n");
    UART_1_PutString("  c=cal  k=kill  x=kill+quiet\r\n");
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

    /* ------------------------------------------------------------------
     * Main loop
     * ------------------------------------------------------------------ */
    for (;;)
    {
        // STEP 1 - CLI User Input
        ch = (uint8)UART_1_GetChar();

        if (ch == 'j' || ch == 'J')  // Toggle Joystick reading
        {
            joystick_active = !joystick_active;
            if (joystick_active) diag_active = TRUE;
            UART_1_PutString(joystick_active
                ? "[CLI] Joystick ON\r\n"  // If true
                : "[CLI] Joystick OFF\r\n");  // If false
        }
        else if (ch == 'm' || ch == 'M')  // Toggle Movi Comms
        {
            movi_active = !movi_active;
            if (movi_active) diag_active = TRUE;
            UART_1_PutString(movi_active
                ? "[CLI] MoVI comms ON\r\n"  // If true
                : "[CLI] MoVI comms OFF\r\n");  // If false
        }
        else if (ch == 'l' || ch == 'L')  // Iterate between control modes
        {
            // DEFER -> RATE -> ABSOLUTE -> DEFER
            if (ctrl_mode == DEFER){ctrl_mode = RATE;}
            else if (ctrl_mode == RATE){ctrl_mode = ABSOLUTE;}
            else {ctrl_mode = DEFER;}

            sprintf(tx, "[CLI] Mode: %s\r\n", mode_to_str(ctrl_mode));
            UART_1_PutString(tx);
        }
        else if (ch == 'k' || ch == 'K')  // Kill but keep printing
        {
            do_kill(&g_movi, &joystick_active, &movi_active,
                    &streaming_adc, &ctrl_mode);
        }
        else if (ch == 'x' || ch == 'X')  // Kill and stop printing
        {
            do_kill(&g_movi, &joystick_active, &movi_active,
                    &streaming_adc, &ctrl_mode);
            diag_active = FALSE;
            UART_1_PutString("[CLI] Output paused. Press j or m to resume.\r\n");
        }
        else if (ch == 's' || ch == 'S')  // Test stream ADC values and calibration
        {
            streaming_adc = !streaming_adc;
            UART_1_PutString(streaming_adc
                ? "[CLI] ADC stream ON\r\n"  // If true
                : "[CLI] ADC stream OFF\r\n");  // If false
        }
        else if (ch == 'c' || ch == 'C')  // Start ADC calibration sequence
        {
            joystick_on_key('c');
        }

        joy_cal_state_t cal_now = joystick_cal_state();
        print_cal_prompt_on_state_change(cal_prev, cal_now);
        cal_prev = cal_now;

        // STEP 2 -  Always pump Rx packets from Movi
        movi_comm_pump(&g_movi);

        // Step 3 - Always read joystick ADC
        if (adc_balanced_poll_frame(counts))
        {
            joystick_on_sample(counts);
            joystick_get_cmd(&cmd);

            // Print raw ADC (/1000) and calibrated voltage output.
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

            // Capture centered zero during calibration.
            if (joystick_cal_state() == JOY_CAL_CENTER_CAPTURE)
            {
                uint16 n = joystick_center_samples_collected();
                if ((n % 8u) == 0u)
                {
                    sprintf(tx, "[CAL] Center samples: %u/%u\r\n",
                            (unsigned)n, (unsigned)CAL_CENTER_SAMPLES);
                    UART_1_PutString(tx);
                }
            }
        }
        
        // Step 4 - Build control packet struct
        ctl.enable = 1u;
        ctl.kill   = 0u;

        if (joystick_active && joystick_cal_state() == JOY_CAL_OFF)
        {
            ctl.pan_mode  = ctrl_mode;
            ctl.tilt_mode = ctrl_mode;
            ctl.roll_mode = DEFER;
            ctl.pan  =  cmd.u[CH_X];
            ctl.tilt = -cmd.u[CH_Y];   // Negated!
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

        // STEP 5 - Transmit to Movi throug HAL
        if (movi_active
            && joystick_cal_state() == JOY_CAL_OFF
            && (g_tick_ms - last_tx_ms) >= MOVI_TX_PERIOD_MS)
        {
            last_tx_ms = g_tick_ms;
            movi_comm_set_control(&g_movi, &ctl);  // Set packet with API
            movi_comm_send_control(&g_movi);       // Send through HAL
        }

        // STEP 6 - Print diagnostics/stats and receive status packet
        if (diag_active
            && joystick_cal_state() == JOY_CAL_OFF
            && (g_tick_ms - last_diag_ms) >= DIAG_PERIOD_MS)
        {
            // Timer
            last_diag_ms = g_tick_ms;

            // Timestamp
            uint32 ms_total = g_tick_ms;
            uint32 sec_part = (ms_total / 1000u)   % 60u;
            uint32 min_part = (ms_total / 60000u)  % 60u;
            uint32 hr_part  =  ms_total / 3600000u;

            sprintf(tx, "  %02lu:%02lu:%02lu\r\n",
                    (unsigned long)hr_part,
                    (unsigned long)min_part,
                    (unsigned long)sec_part);
            UART_1_PutString(tx);

            // Serial comms stats
            movi_statistics_t s;
            movi_comm_get_statistics(&g_movi, &s);

            sprintf(tx,
                "[Serial]    TX:%lu RX:%lu | csum:%lu err:0x%02X | COMMS:%s JOYSTICK:%s\r\n",
                (unsigned long)s.tx_packets,
                (unsigned long)s.rx_packets,
                (unsigned long)s.rx_bad_checksum,
                (unsigned)s.uart_err_flags,
                movi_active     ? "ON " : "OFF",
                joystick_active ? "ON " : "OFF");
            UART_1_PutString(tx);

            // Gimble Status Packet
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

            // GPS telemetry
            if (st.gps_valid)
            {
                char lat_s, lon_s, spd_s, alt_s;
                int16 lat_i, lat_f, lon_i, lon_f;
                int16 alt_i, alt_f, spd_i, spd_f;
                int16 hacc_cm = (int16)(st.gps_hacc_m * 100.0f);

                decomp(st.gps_lat_deg,       &lat_i, &lat_f, &lat_s);
                decomp(st.gps_lon_deg,       &lon_i, &lon_f, &lon_s);
                decomp(st.gps_alt_m,         &alt_i, &alt_f, &alt_s);
                decomp(st.gps_ground_spd_ms, &spd_i, &spd_f, &spd_s);

                sprintf(tx,
                    "[GPS]       Lat=%c%d.%06d  Lon=%c%d.%06d"
                    "  Alt=%c%d.%02dm  Spd=%d.%02dm/s  Hacc=%dcm\r\n",
                    lat_s, (int)lat_i, (int)lat_f,
                    lon_s, (int)lon_i, (int)lon_f,
                    alt_s, (int)alt_i, (int)alt_f,
                    (int)spd_i, (int)spd_f,
                    (int)hacc_cm);
                UART_1_PutString(tx);
            }
            else
            {
                UART_1_PutString("[GPS]       No fix\r\n");
            }

            // Joystick Current Values
            char lr_s, ud_s;
            int16 lr_i, lr_f, ud_i, ud_f;
            decomp(cmd.u[CH_X], &lr_i, &lr_f, &lr_s);
            decomp(cmd.u[CH_Y], &ud_i, &ud_f, &ud_s);

            sprintf(tx,
                "[Joystick]  L/R=%c%d.%02d (%s)  U/D=%c%d.%02d (%s)\r\n",
                lr_s, (int)lr_i, (int)lr_f, mode_to_str(ctrl_mode),
                ud_s, (int)ud_i, (int)ud_f, mode_to_str(ctrl_mode));
            UART_1_PutString(tx);

            // Clear error flags for next loop
            g_movi.statistics.uart_err_flags = 0u;
        }
    }
}

/* [] END OF FILE */