/* adc_balanced.h
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

#ifndef ADC_BALANCED_H
#define ADC_BALANCED_H

#include <project.h>

// Number of channels
#ifndef N_CH
#define N_CH (2u)
#endif

#if (N_CH < 1u)
#error "N_CH must be >= 1"  // Cant have no channels!
#endif

// Initialized ADC Del Sig, MUX, and ISR
void adc_balanced_init(void);


// Polls for full ADC frame set by stet variable, 1 when available else 0.
// 'out_counts must' point to an array of at least N_CH elements.
uint8 adc_balanced_poll_frame(int16 out_counts[N_CH]);

// Convert counts to mV
int32 adc_balanced_counts_to_mv(int16 counts);

#endif

/* [] END OF FILE */
