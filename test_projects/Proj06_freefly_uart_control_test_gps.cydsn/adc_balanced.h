/* adc_balanced.h
 *
 * Balanced / round-robin ADC sampling over an analog mux (AMux) into a single ADC.
 * - Designed for PSoC Creator components:
 *     ADC_DelSig_1  (ADC_DelSig)
 *     AMux_1        (AMux)
 *     isr_adc       (ISR connected to ADC EOC)
 *
 * - Provides a "frame" of N_CH samples, each sample captured on EOC and mux switched safely.
 * - Current configuration: N_CH = 2 (X,Y), but designed to scale.
 */

#ifndef ADC_BALANCED_H
#define ADC_BALANCED_H

#include <project.h>

/* Number of mux channels to scan (0..N_CH-1). Set to 2 for X/Y joystick. */
#ifndef N_CH
#define N_CH (2u)
#endif

#if (N_CH < 1u)
#error "N_CH must be >= 1"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize ADC+AMux+ISR and start scanning from channel 0. */
void adc_balanced_init(void);

/* Poll for a complete scan frame (all N_CH channels updated once).
 * Returns 1 when a new frame is available and copies latest counts into out_counts[].
 * Returns 0 otherwise.
 *
 * out_counts must point to an array of at least N_CH elements.
 */
uint8 adc_balanced_poll_frame(int16 out_counts[N_CH]);


/* Convert ADC counts to millivolts using the ADC component conversion helper. */
int32 adc_balanced_counts_to_mv(int16 counts);

#endif /* ADC_BALANCED_H */


/* [] END OF FILE */
