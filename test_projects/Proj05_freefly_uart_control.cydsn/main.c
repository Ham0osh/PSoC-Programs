/* main.c
 *
 * Uses:
 *  - adc_balanced.c/.h  : round-robin ADC sampling over AMux (N_CH channels)
 *  - joystick.c/.h      : calibration + mapping (deadband, inversion, scaling)
 *
 * CLI stays here:
 *   'c' : calibration advance (start -> center capture -> save)
 *   'x' : stop streaming (and also cancels calibration if active)
 *   's' : start streaming
 */

#include <project.h>
#include <stdio.h>

// Modules
#include "adc_balanced.h"  // ADC + AMux handling
#include "joystick.h"      // Joystick processing of analog in + calibration
#include "movi_comm.h"     // Movi communication instances

// To not overload the movi or cause accidents
#define LOOP_DELAY_MS 20  // 50 Hz loop
#define TRANSMIT_BUFFER_SIZE 96  // for monitor

/* Project Defines */
#define FALSE  0
#define TRUE   1

/* Channel assignments */
#define CH_X (0u)
#define CH_Y (1u)

static void print_cal_prompt_on_state_change(joy_cal_state_t prev, joy_cal_state_t now)
{
    if (prev == now) return;

    if (now == JOY_CAL_RANGE_CAPTURE)
    {
        UART_1_PutString("\r\nCAL MODE\r\n");
        UART_1_PutString("Step 1: Move joystick through FULL range (all directions), then release to center.\r\n");
        UART_1_PutString("Press 'c' to capture CENTER (averaged), 'x' to cancel.\r\n");
    }
    else if (now == JOY_CAL_CENTER_CAPTURE)
    {
        UART_1_PutString("\r\nStep 2: Capturing CENTER (averaging)... keep joystick relaxed.\r\n");
        UART_1_PutString("Wait for capture to complete, then press 'c' to SAVE.\r\n");
    }
    else if (now == JOY_CAL_CONFIRM_SAVE)
    {
        UART_1_PutString("\r\nCenter captured.\r\n");
        UART_1_PutString("Step 3: Press 'c' to SAVE, 'x' to cancel.\r\n");
    }
    else if (now == JOY_CAL_OFF && prev != JOY_CAL_OFF)
    {
        /* Could have been cancel or save; joystick module stays print-free */
        UART_1_PutString("\r\nCAL exited.\r\n");
    }
}

// Requisites for Movi UART
static movi_comm_t g_movi;

// HAL Layer
static void movi_uart_putc(void* ctx, uint8_t b){
    (void)ctx;
    UART_MOVI_PutChar((char)b);
}
static const movi_hal_t MOVI_HAL = {
    .ctx = NULL,
    .uart_putc = movi_uart_putc
};

CY_ISR(isr_rx_movi_Handler)
{
    uint8 st;

    do
    {
        st = UART_MOVI_RXSTATUS_REG;

        /* Accumulate UART error flags into the instance (sticky) */
        if (st & (UART_MOVI_RX_STS_BREAK      |
                  UART_MOVI_RX_STS_PAR_ERROR  |
                  UART_MOVI_RX_STS_STOP_ERROR |
                  UART_MOVI_RX_STS_OVERRUN))
        {
            movi_comm_on_uart_error_flags(&g_movi,
                (uint8)(st & (UART_MOVI_RX_STS_BREAK      |
                              UART_MOVI_RX_STS_PAR_ERROR  |
                              UART_MOVI_RX_STS_STOP_ERROR |
                              UART_MOVI_RX_STS_OVERRUN)));
        }

        /* Drain FIFO and forward bytes to movi_comm */
        if (st & UART_MOVI_RX_STS_FIFO_NOTEMPTY)
        {
            uint8 b = UART_MOVI_RXDATA_REG;
            movi_comm_on_rx_byte(&g_movi, b);
        }

    } while (st & UART_MOVI_RX_STS_FIFO_NOTEMPTY);
}

// For testing movi comms
static uint32 last_movi_tx_ms = 0u;
static const uint32 MOVI_PERIOD_MS = 1000u; /* 1 Hz for test */
static uint32 last_rx_packets = 0u;
// Timer for state machine loop
volatile uint32 g_tick_ms = 0;
volatile uint32 last_diag_ms = 0;
CY_ISR(isr_Looptimer_Handler)
{
    g_tick_ms++;
    Looptimer_ReadStatusRegister();  // Clear interrupt
}


int main(void)
{
    char tx[TRANSMIT_BUFFER_SIZE]; // For monitor
    uint8 ch;
    uint8 streaming_adc = FALSE;
    uint8 streaming_movi = FALSE;

    /* Latest raw counts from ADC frame */
    int16 counts[N_CH];

    /* Telemetry */
    int32 x_mv = 0;
    int32 y_mv = 0;
    static uint32 last_movi_ms = 0u;


    /* Commands */
    joy_cmd_t cmd;

    /* Calibration state tracking for prompts */
    joy_cal_state_t cal_prev = JOY_CAL_OFF;

    /* Default calibration (best guess) */
    joy_cal_t defaults;
    for (uint8 i = 0u; i < (uint8)N_CH; i++)
    {
        defaults.minv[i]   = 0;
        defaults.maxv[i]   = 255;
        defaults.center[i] = 128;
    }
    defaults.valid = 0u;

    /* Inversion mask: flip X for “pan” direction, leave Y normal */
    uint32 invert_mask = (1u << CH_X);

    // Start monitor UART
    CyGlobalIntEnable;
    UART_1_Start();
    UART_1_PutString("\r\nCOM Port Open\r\n");
    UART_1_PutString("Keys: s=start stream, x=stop stream/cancel cal, c=cal advance\r\n");
    // Start the Movi UART connection
    UART_MOVI_Start();
    UART_MOVI_ClearRxBuffer();
    UART_MOVI_ClearTxBuffer();
    // Init movi comm from instances created
    // does not start UART; it sets internal state + RB
    movi_comm_init(&g_movi, &MOVI_HAL);
    isr_rx_movi_StartEx(isr_rx_movi_Handler);
    movi_control_t ctl;
    UART_1_PutString("\r\nMovi UART Initialized\r\n");

    // Inits for joystick and ADC
    joystick_init(&defaults, invert_mask);
    adc_balanced_init();
    UART_1_PutString("\r\nADC Initialized\r\n");
    
    // Start timer
    Looptimer_Start();
    isr_Looptimer_StartEx(isr_Looptimer_Handler);

    for (;;)
    {
       // Loop Step 1: Get CLI input
        ch = (uint8)UART_1_GetChar();

        if (ch == 's' || ch == 'S')
        {
            streaming_adc = TRUE;
            streaming_movi = FALSE;
        }
        else if (ch =='u' || ch == 'U')
        {
            streaming_adc = FALSE;
            streaming_movi = TRUE;
        }
        else if (ch == 'x' || ch == 'X')
        {
            streaming_adc = FALSE;
            streaming_movi = FALSE;
            joystick_on_key((char)ch); /* also cancels calibration if active */
        }
        else if (ch == 'c' || ch == 'C')
        {
            streaming_adc = FALSE;
            streaming_movi = FALSE;
            joystick_on_key((char)ch);
        }

        /* Provide prompts when cal state changes */
        {
            joy_cal_state_t cal_now = joystick_cal_state();
            print_cal_prompt_on_state_change(cal_prev, cal_now);
            cal_prev = cal_now;
        }

        // Loop Step 2: ALWAYS poll movi ISR
        // Otherwise it will overwhelm the ringbuffer.
        movi_comm_pump(&g_movi);
        
        // Loop Step 3: Poll ADC for Joystick
        if (adc_balanced_poll_frame(counts))
        {
            /* Feed joystick module the fresh frame */
            joystick_on_sample(counts);

            /* Convert to mV for telemetry (optional) */
            x_mv = adc_balanced_counts_to_mv(counts[CH_X]);
            y_mv = adc_balanced_counts_to_mv(counts[CH_Y]);

            /* Get mapped commands */
            joystick_get_cmd(&cmd);

            if (streaming_adc)
            {
                /* Avoid printf-float settings: print milli-units */
                int16 pan_milli  = (int16)(cmd.u[CH_X] * 1000.0f);
                int16 tilt_milli = (int16)(cmd.u[CH_Y] * 1000.0f);

                sprintf(tx,
                        "X:%ld mV Y:%ld mV  pan:%d/1000  tilt:%d/1000\r\n",
                        (long)x_mv, (long)y_mv, pan_milli, tilt_milli);
                UART_1_PutString(tx);
            }

            /* Optional: while center capture is running, show progress */
            if (joystick_cal_state() == JOY_CAL_CENTER_CAPTURE)
            {
                uint16 n = joystick_center_samples_collected();
                if ((n % 8u) == 0u) /* throttle prints */
                {
                    sprintf(tx, "Center samples: %u/%u\r\n", (unsigned)n, (unsigned)CAL_CENTER_SAMPLES);
                    UART_1_PutString(tx);
                }
            }
        }
    
        // Loop Step 4: Movi Tx section
        if (streaming_movi)
        {
            static uint32 last_tx_ms = 0;
            static uint32 last_print_pkts = 0;
            
            // Tx gated to every 20ms
            if ((g_tick_ms - last_tx_ms) >= 100u) {
                last_tx_ms = g_tick_ms;
                // read current control packet into variable
                movi_comm_get_control(&g_movi, &ctl);

                // Update with example packet
                ctl.enable = 1u;
                ctl.kill = 0u;
                ctl.pan_mode  = DEFER;
                ctl.tilt_mode = DEFER;
                ctl.roll_mode = DEFER;
                ctl.pan  = 0.0f;
                ctl.tilt = 0.0f;
                ctl.roll = 0.0f;

                movi_comm_set_control(&g_movi, &ctl);
                movi_result_t r = movi_comm_send_control(&g_movi);
            }
            
            // Get diagnostics throttled down to not overwhelm
            if ((g_tick_ms - last_diag_ms) >= 1000u)
            {
                last_diag_ms = g_tick_ms;
                movi_statistics_t s;
                movi_comm_get_statistics(&g_movi, &s);
                sprintf(tx, "[Movi] TX pkts=%lu bytes=%lu | RX pkts=%lu bytes=%lu csum_fail=%lu err=0x%02X\r\n",
                        (unsigned long)s.tx_packets,
                        (unsigned long)s.tx_bytes,
                        (unsigned long)s.rx_packets,
                        (unsigned long)s.rx_bytes,
                        (unsigned long)s.rx_bad_checksum,
                        (unsigned)s.uart_err_flags);
                UART_1_PutString(tx);
                movi_status_t st;
                movi_comm_get_status(&g_movi, &st);
                if (st.valid) {
                    // Battery
                    int16 vl = (int16)(st.battery_left_v  * 10.0f);
                    int16 vr = (int16)(st.battery_right_v * 10.0f);
                    sprintf(tx, "[Status] Batt L=%d.%dV R=%d.%dV\r\n",
                            vl/10, vl%10, vr/10, vr%10);
                    UART_1_PutString(tx);

                    // Quaternion → Euler conversion (degrees)
                    // Pan = k axis, Tilt = j axis, Roll = i axis
                    // For small angles: angle ≈ 2*asin(component) * (180/pi)
                    // Full conversion using atan2 for robustness:
                    float r = st.gimbal_r;
                    float i = st.gimbal_i;
                    float j = st.gimbal_j;
                    float k = st.gimbal_k;

                    // Roll (X/i axis)
                    float sinr = 2.0f * (r*i + j*k);
                    float cosr = 1.0f - 2.0f * (i*i + j*j);
                    int16 roll_deg = (int16)(atan2f(sinr, cosr) * 57.2958f);

                    // Tilt (Y/j axis)
                    float sinp = 2.0f * (r*j - k*i);
                    int16 tilt_deg;
                    if (sinp >= 1.0f)       tilt_deg =  90;
                    else if (sinp <= -1.0f) tilt_deg = -90;
                    else                    tilt_deg = (int16)(asinf(sinp) * 57.2958f);

                    // Pan (Z/k axis)
                    float siny = 2.0f * (r*k + i*j);
                    float cosy = 1.0f - 2.0f * (j*j + k*k);
                    int16 pan_deg = (int16)(atan2f(siny, cosy) * 57.2958f);

                    sprintf(tx, "[Status] Pan=%d Tilt=%d Roll=%d deg\r\n",
                            pan_deg, tilt_deg, roll_deg);
                    UART_1_PutString(tx);

                    // Status flags
                    sprintf(tx, "[Status] St1=0x%02X St2=0x%02X\r\n",
                            (unsigned)st.gimbal_status1,
                            (unsigned)st.gimbal_status2);
                    UART_1_PutString(tx);
                } else {
                    UART_1_PutString("[Status] waiting for valid frame...\r\n");
                }
                
                g_movi.statistics.uart_err_flags = 0;
            }
            
            // NEW: Clear error flags for next iteration so we see fresh errors
//                g_movi.statistics.uart_err_flags = 0;
//                if (r == MOVI_OK) {
//                    sprintf(tx, "[Tx] ok  pkts=%lu bytes=%lu\r\n",
//                            (unsigned long)s.tx_packets, (unsigned long)s.tx_bytes);
//                    UART_1_PutString(tx);
//                } else {
//                    UART_1_PutString("[Tx] FAIL\r\n");
//                }
//                if (s.rx_packets > last_rx_packets) {
//                    UART_1_PutString("[Rx] frame received : SUCCESS\r\n");
//                    last_rx_packets = s.rx_packets;
//
//                    movi_status_t st;
//                    movi_comm_get_status(&g_movi, &st);
//                    if (st.valid) {
//                        sprintf(tx, "V_L=%.2f  V_R=%.2f\r\n", (double)st.battery_left_v, (double)st.battery_right_v);
//                        UART_1_PutString(tx);
//                    }
//                } else {
//                    UART_1_PutString("[Rx] no new frame    : (waiting)\r\n");
//                }
//
//                if (s.uart_err_flags != 0u)
//                {
//                    char t[64];
//                    sprintf(t, "UART err flags: 0x%02X\r\n", (unsigned)s.uart_err_flags);
//                    UART_1_PutString(t);
//                }
//
//                if (s.rb_drops != 0u)
//                {
//                    char t[64];
//                    sprintf(t, "RB drops: %u\r\n", (unsigned)s.rb_drops);
//                    UART_1_PutString(t);
//                }
        }
    }
}   

/* [] END OF FILE */
