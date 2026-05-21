/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

#ifndef PI_COMMS_H
#define PI_COMMS_H

#include <stdint.h>

// Temporary, will eventualy match Pi conventroin for all Tx and Rx
#define PI_MAGIC_CENTROID  0xA5u  // Pi   -> PSoC
#define PI_MAGIC_TELEM     0x5Au  // PSoC -> Pi

typedef struct {
    int16_t  cx, cy;      // centroid, 0.1 mrad
    uint16_t ex, ey;      // per-axis error, 0.1 mrad
    uint64_t pi_time_us;  // Pi uptime, microseconds
} pi_centroid_t;

void     pi_init(void);
void     pi_on_rx_byte(uint8_t b);             // fed by isr_rx_pi_Handler
void     pi_on_uart_err_flags(uint8_t flags);  // fed by isr_rx_pi_Handler
uint8_t  pi_get_centroid(pi_centroid_t *out);  // 1 if a new frame arrived
uint32_t pi_last_rx_ms(void);
uint16_t pi_crc_errors(void);
uint16_t pi_uart_errors(void);
void     pi_send_frame(uint8_t magic, const uint8_t *payload, uint8_t len);

#endif /* PI_COMMS_H */
/* [] END OF FILE */
