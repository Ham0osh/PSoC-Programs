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

#include "adc_balanced.h"
#include "joystick.h"

/* Project Defines */
#define FALSE  0
#define TRUE   1

#define TRANSMIT_BUFFER_SIZE  96

/* N_CH should be consistent across modules (defaults to 2 if not defined) */
/* #define N_CH (2u) */

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

int main(void)
{
    char tx[TRANSMIT_BUFFER_SIZE];

    uint8 ch;
    uint8 streaming = FALSE;

    /* Latest raw counts from ADC frame */
    int16 counts[N_CH];

    /* Telemetry */
    int32 x_mv = 0;
    int32 y_mv = 0;

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

    /* Start UART early for user feedback */
    CyGlobalIntEnable;
    UART_1_Start();
    UART_1_PutString("COM Port Open\r\n");
    UART_1_PutString("Keys: s=start stream, x=stop stream/cancel cal, c=cal advance\r\n");

    /* Init modules */
    joystick_init(&defaults, invert_mask);
    adc_balanced_init();

    for (;;)
    {
        /* ---------- CLI ---------- */
        ch = (uint8)UART_1_GetChar();

        if (ch == 's' || ch == 'S')
        {
            streaming = TRUE;
        }
        else if (ch == 'x' || ch == 'X')
        {
            streaming = FALSE;
            joystick_on_key((char)ch); /* also cancels calibration if active */
        }
        else if (ch == 'c' || ch == 'C')
        {
            joystick_on_key((char)ch);
        }

        /* Provide prompts when cal state changes */
        {
            joy_cal_state_t cal_now = joystick_cal_state();
            print_cal_prompt_on_state_change(cal_prev, cal_now);
            cal_prev = cal_now;
        }

        /* ---------- ADC sampling ---------- */
        if (adc_balanced_poll_frame(counts))
        {
            /* Feed joystick module the fresh frame */
            joystick_on_sample(counts);

            /* Convert to mV for telemetry (optional) */
            x_mv = adc_balanced_counts_to_mv(counts[CH_X]);
            y_mv = adc_balanced_counts_to_mv(counts[CH_Y]);

            /* Get mapped commands */
            joystick_get_cmd(&cmd);

            if (streaming)
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
    }
}

/* [] END OF FILE */
