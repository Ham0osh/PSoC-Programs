/* ========================================
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
 * For inter-device communications with Raspberry Pi SBC.
*/

#ifndef PI_COMMS_H
#define PI_COMMS_H

#include <stdint.h>

// Index for both camera streams, accepts up to two inputs for now!
// ATTENTION: Hard coded definition of camera based centroid streams.
// Add or reduce according to needs, and sensor types!
// TODO: Is there a more generalizable way to handle N streams?
//       Or maybe stream-wise packet types for different sensors?
#define STREAM_COARSE  0u
#define STREAM_FINE    1u
#define STREAM_COUNTS  2u

#define SBC_MAGIC   0xAB  // Pi <-> PSoC both; signals packet start

#define PKT_CENTROID_C  0x01u  // Pi   -> PSoC Coarse
#define PKT_CENTROID_F  0x02u  // Pi   -> PSoC Fine
#define PKT_STATE_REQ   0x10u  // Pi   -> PSoC

#define PKT_TELEM_HOT   0x20u  // PSoC -> Pi
#define PKT_TELEM_POWER 0x21u  // PSoC -> Pi
#define PKT_TELEM_ENV   0x22u  // PSoC -> Pi
#define PKT_TELEM_LINK  0x23u  // PSoC -> Pi
#define PKT_STATE_ACK   0x30u  // PSoC -> Pi

typedef struct {
    uint32_t t_ms;          // Pi monotonic ms
    int16_t  cx, cy;        // 0.1 mrad
    uint16_t cxerr, cyerr;  // one sigma stdev, 0 = undef
} payload_centroid_t;

void     sbc_init(void);
void     sbc_on_rx_byte(uint8_t b);             // fed by isr_rx_sbc_Handler
void     sbc_on_uart_err_flags(uint8_t flags);  // fed by isr_rx_sbc_Handler
uint8_t  sbc_get_centroid(uint8_t stream, payload_centroid_t *out);  // 1 if a new frame arrived
uint32_t sbc_last_rx_ms(uint8_t stream);
uint16_t sbc_crc_errors(void);
uint16_t sbc_uart_errors(void);
uint16_t sbc_unknown_magic(void);
uint32_t sbc_rx_pkt_count(void);
uint16_t sbc_last_centroid_dt_ms(uint8_t stream);
void     sbc_send_frame(uint8_t type, const uint8_t *payload, uint8_t len);

#endif /* PI_COMMS_H */
/* [] END OF FILE */
