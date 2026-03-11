/* adc_balanced.c
 * ========================================
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
 * This code helps collect data from a
 * multiplexed ADC Del Sig.
*/

#include "adc_balanced.h"

// ADC state
static volatile uint8 s_eoc_flag = 0;     // set by ISR on EOC
static volatile uint8 s_frame_ready = 0;  // set when we finish channel N_CH-1
static volatile uint8 s_ch_idx = 0;       // current mux channel index
static volatile int16 s_counts[N_CH];     // latest counts per channel

// ISR prototype, PSoC generates
CY_ISR_PROTO(isr_adc_Handler);

// ISR - Fires after ADC conversion finishes
CY_ISR(isr_adc_Handler)
{
    // Clear pending IRQ and set received flag
    isr_adc_ClearPending();
    s_eoc_flag = 1u;
}

// Initialzie the ADC and MUX
void adc_balanced_init(void)
{
    CyGlobalIntEnable;

    // Start UDBs - USER EDIT THESE, MUST MATCH TopDesign.cysch
    ADC_DelSig_1_Start();
    AMux_1_Start();

    // Start ISR handler
    isr_adc_StartEx(isr_adc_Handler);

    // Initialize state variables
    s_ch_idx = 0u;
    s_eoc_flag = 0u;
    s_frame_ready = 0u;

    // zero our buffer
    for (uint8 i = 0u; i < (uint8)N_CH; i++)
        s_counts[i] = 0;

    // Select first channel
    AMux_1_Select(0u);

    // Start ADC converting!
    ADC_DelSig_1_StartConvert();
}

// Function that advances on each "End of Conversion"
static void adc_balanced_advance_on_eoc(void)
{
    // Ensure conversion stopped before advancing
    ADC_DelSig_1_StopConvert();

    // Read last ADC result
    s_counts[s_ch_idx] = ADC_DelSig_1_GetResult16();

    // Mark state variable that we have received a frame
    if (s_ch_idx == (uint8)(N_CH - 1u))
    {
        s_frame_ready = 1u;
        s_ch_idx = 0u;
    }
    else
    {
        s_ch_idx++;
    }

    // Switch to next channel
    AMux_1_Select(s_ch_idx);

    // Advance done, continue converting!
    ADC_DelSig_1_StartConvert();
}

// Consume "End of Conversion" events
uint8 adc_balanced_poll_frame(int16 out_counts[N_CH])
{
    if (s_eoc_flag)  // If EoC triggered from interrupt
    {
        s_eoc_flag = 0u;                // Reset flag
        adc_balanced_advance_on_eoc();  // Advance MUX
    }

    if (s_frame_ready)  // If frame received from "adc_balanced_advance_on_eoc"
    {
        s_frame_ready = 0u;  // Reset flag
        
        // Copy results into out_counts
        for (uint8 i = 0u; i < (uint8)N_CH; i++)
            out_counts[i] = s_counts[i];

        return 1u;  // Return true that frame is available to poll
    }

    return 0u;  // Frame not ready to poll
}

// Convert counts to mV using PSoC API
int32 adc_balanced_counts_to_mv(int16 counts)
{
    return (int32)ADC_DelSig_1_CountsTo_mVolts(counts);
}

/* [] END OF FILE */
