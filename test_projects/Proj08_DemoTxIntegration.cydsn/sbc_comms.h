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

#ifndef SBC_COMMS_H
#define SBC_COMMS_H

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

// Control
#define PKT_CENTROID_C  0x01u  // Pi   -> PSoC Coarse
#define PKT_CENTROID_F  0x02u  // Pi   -> PSoC Fine
#define PKT_STATE_REQ   0x10u  // Pi   -> PSoC
#define PKT_GPS_TARGET  0x0Fu  // Pi   -> PSoC GPS target
#define PKT_NUDGE       0x11u  // Pi   -> PSoC General nudge in deg
#define PKT_PARAM_SET   0x12u  // Pi   -> PSoC Param setter like Kp
#define PKT_SET_ORIGIN  0x13u  // Pi   -> PSoC Set software origin.
#define PKT_PARAM_GET   0x14u  // Pi   -> PSoC param read
#define PKT_FATAL_CLEAR 0x15u  // Pi   -> PSoC clear FATAL
    
// telemetry
#define PKT_TELEM_HOT   0x20u  // PSoC -> Pi
#define PKT_TELEM_POWER 0x21u  // PSoC -> Pi
#define PKT_TELEM_ENV   0x22u  // PSoC -> Pi
#define PKT_TELEM_LINK  0x23u  // PSoC -> Pi
#define PKT_TELEM_GPS   0x24u  // PSoC -> Pi
#define PKT_TELEM_BARO  0x25u  // PSoC -> Pi
#define PKT_TELEM_MAG   0x26u  // PSoC -> Pi
#define PKT_PARAM_VALUE 0x34u  // PSoC -> Pi
    
#define PKT_STATE_ACK   0x30u  // PSoC -> Pi
#define PKT_CMD_ACK     0x31u  // PSoC -> Pi
    
// Incoming packet types from SBC
typedef struct {
    uint32_t t_ms;          // Pi monotonic ms
    int16_t  cx, cy;        // 0.1 mrad
    uint16_t cxerr, cyerr;  // one sigma stdev, 0 = undef
} payload_centroid_t;

#define SBC_STATE_REQ_LEN   1u     // single byte: state_t value
typedef struct {
    uint8_t requested_state;       // state_t value — SBC must use the same enum
} payload_state_req_t;

#define SBC_GPS_TARGET_LEN  12u    // 3x int32
typedef struct {
    int32_t lat_raw;               // /1e7 deg
    int32_t lon_raw;
    int32_t alt_mm;                // mm above MSL
} payload_gps_target_t;

#define SBC_NUDGE_LEN   4u
typedef struct {
    int16_t dpan_cdeg;
    int16_t dtilt_cdeg;
} payload_nudge_t;

#define SBC_PARAM_LEN   5u
#define PARAM_TRACK_KP  0x01u
typedef struct {
    uint8_t id;
    float value;
} payload_param_t;

#define SBC_SET_ORIGIN_LEN  0u


#define SBC_PARAM_GET_LEN     1u
typedef struct {
    uint8_t id;
} payload_param_get_t;

#define SBC_FATAL_CLEAR_LEN   0u

#define SBC_CMD_ACK_LEN       2u
typedef struct {
    uint8_t cmd_type;
    uint8_t result;
} payload_cmd_ack_t;

#define SBC_PARAM_VALUE_LEN   5u
typedef struct {
    uint8_t id;
    float value;
} payload_param_value_t;

// TODO: All of these acks can be consolidated now that they speak the same language.
// Response codes for state change
#define STATE_ACK_OK        0x00u  // transition accepted
#define STATE_ACK_REJECTED  0x01u  // FATAL latch active
#define STATE_ACK_INVALID   0x02u  // state value not a requestable leaf
// Generic CMD_ACK result codes
#define CMD_ACK_OK            0x00u
#define CMD_ACK_REJECTED      0x01u  // state-disallowed (e.g. nudge in bad state)
#define CMD_ACK_INVALID       0x02u  // unknown id or malformed payload
#define CMD_ACK_NO_TELEM      0x03u  // operation needs telemetry it doesn't have
#define CMD_ACK_FATAL_LATCHED 0x04u  // refused while ERROR_KILL / fatal latch active


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
void     sbc_discard_centroid(uint8_t stream);

uint8_t sbc_get_state_req (payload_state_req_t  *out);  // 1 if a new request arrived
uint8_t sbc_get_gps_target(payload_gps_target_t *out);  // 1 if a new target arrived
void    sbc_send_state_ack(uint8_t actual_state, uint8_t result);

uint8_t sbc_get_nudge     (payload_nudge_t *out);
uint8_t sbc_get_param     (payload_param_t *out);
uint8_t sbc_get_set_origin(void);

uint8_t sbc_get_fatal_clear(void);
void    sbc_send_cmd_ack    (uint8_t cmd_type, uint8_t result);
void    sbc_send_param_value(uint8_t id, float value);

#endif /* SBC_COMMS_H */
/* [] END OF FILE */
