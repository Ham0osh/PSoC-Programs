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
 * This code is for inter-device communications with a Raspberry Pi over UART.
*/

#include "sbc_comms.h"
#include "debug_pins.h"
#include <string.h>
#include <project.h>

extern volatile uint32_t g_tick_ms;  // PSoC ms timer

// Copied for Pi software codebase
// We use a Cyclic Redundancy Check (CRC).
// The CRC takes the whole message as one binary number, divides it by 0x07
// using XOR operations to be left with the remainder. A table is used instead
// of computing each of the possible bit-wise division operations.
static const uint8_t crc8_table[256] = {
    0x00,0x07,0x0E,0x09,0x1C,0x1B,0x12,0x15,
    0x38,0x3F,0x36,0x31,0x24,0x23,0x2A,0x2D,
    0x70,0x77,0x7E,0x79,0x6C,0x6B,0x62,0x65,
    0x48,0x4F,0x46,0x41,0x54,0x53,0x5A,0x5D,
    0xE0,0xE7,0xEE,0xE9,0xFC,0xFB,0xF2,0xF5,
    0xD8,0xDF,0xD6,0xD1,0xC4,0xC3,0xCA,0xCD,
    0x90,0x97,0x9E,0x99,0x8C,0x8B,0x82,0x85,
    0xA8,0xAF,0xA6,0xA1,0xB4,0xB3,0xBA,0xBD,
    0xC7,0xC0,0xC9,0xCE,0xDB,0xDC,0xD5,0xD2,
    0xFF,0xF8,0xF1,0xF6,0xE3,0xE4,0xED,0xEA,
    0xB7,0xB0,0xB9,0xBE,0xAB,0xAC,0xA5,0xA2,
    0x8F,0x88,0x81,0x86,0x93,0x94,0x9D,0x9A,
    0x27,0x20,0x29,0x2E,0x3B,0x3C,0x35,0x32,
    0x1F,0x18,0x11,0x16,0x03,0x04,0x0D,0x0A,
    0x57,0x50,0x59,0x5E,0x4B,0x4C,0x45,0x42,
    0x6F,0x68,0x61,0x66,0x73,0x74,0x7D,0x7A,
    0x89,0x8E,0x87,0x80,0x95,0x92,0x9B,0x9C,
    0xB1,0xB6,0xBF,0xB8,0xAD,0xAA,0xA3,0xA4,
    0xF9,0xFE,0xF7,0xF0,0xE5,0xE2,0xEB,0xEC,
    0xC1,0xC6,0xCF,0xC8,0xDD,0xDA,0xD3,0xD4,
    0x69,0x6E,0x67,0x60,0x75,0x72,0x7B,0x7C,
    0x51,0x56,0x5F,0x58,0x4D,0x4A,0x43,0x44,
    0x19,0x1E,0x17,0x10,0x05,0x02,0x0B,0x0C,
    0x21,0x26,0x2F,0x28,0x3D,0x3A,0x33,0x34,
    0x4E,0x49,0x40,0x47,0x52,0x55,0x5C,0x5B,
    0x76,0x71,0x78,0x7F,0x6A,0x6D,0x64,0x63,
    0x3E,0x39,0x30,0x37,0x22,0x25,0x2C,0x2B,
    0x06,0x01,0x08,0x0F,0x1A,0x1D,0x14,0x13,
    0xAE,0xA9,0xA0,0xA7,0xB2,0xB5,0xBC,0xBB,
    0x96,0x91,0x98,0x9F,0x8A,0x8D,0x84,0x83,
    0xDE,0xD9,0xD0,0xD7,0xC2,0xC5,0xCC,0xCB,
    0xE6,0xE1,0xE8,0xEF,0xFA,0xFD,0xF4,0xF3,
};

#define SBC_CENTROID_LEN  12u   // uint32 t_ms + 2*int16 + 2*uint16
#define SBC_RX_MAX        64u

// Packet parser section enum, initialized to magic byte
// Expects: [MAGIC] [TYPE] [LEN] ... [PAYLOAD] ... [CRC]
static enum { S_MAGIC, S_TYPE, S_LEN, S_PAYLOAD, S_CRC } sbc_rx_state = S_MAGIC;

// Variables that packets are loaded into plus flag for "newer" write.
static uint8_t              s_type;  // Type loaded into here
static uint8_t              s_len;   // Length loaded into here
static uint8_t              s_idx;
static uint8_t              s_buf[SBC_RX_MAX];  // Payload parsed into here!
// Per centroid stream
static payload_centroid_t   s_centroid[STREAM_COUNTS];  // Payload per centroid
static volatile uint8_t     s_fresh_bits;  // bit 0 for coarse and bit 1 for fine
static uint32_t             s_last_rx_ms[STREAM_COUNTS];
static uint16_t             s_last_centroid_dt_ms[STREAM_COUNTS];
// Global telemetry for communications.
static uint16_t s_crc_err             = 0u;  // inter-device communications.
static uint16_t s_uart_err            = 0u;
static uint16_t s_unknown_magic       = 0u;
static uint32_t s_rx_pkt_count        = 0u;
// SBC -> PSoC State Requests
static payload_state_req_t  s_state_req;
static volatile uint8_t     s_state_req_fresh   = 0u;
// SBC -> PSoC GPS Target Sets
static payload_gps_target_t s_gps_target;
static volatile uint8_t     s_gps_target_fresh  = 0u;
// SBC -> PSoC Relative Nudge
static payload_nudge_t      s_nudge;
static volatile uint8_t     s_nudge_fresh       = 0u;
// SBC -> PSoC Parameter Set (kp, etc.)
static payload_param_t      s_param;
static volatile uint8_t     s_param_fresh       = 0u;
// SBC -> PSoC Software-origin capture request (zero-length payload)
static volatile uint8_t     s_set_origin_fresh  = 0u;
// TODO: Document the below three.
static payload_param_get_t s_param_get;
static volatile uint8_t s_param_get_fresh       = 0u;
static volatile uint8_t s_fatal_clear_fresh     = 0u;

void sbc_init(void)
{
    // Initialize parameters for SBC communications.
    sbc_rx_state    = S_MAGIC;
    s_idx          = 0u;
    s_fresh_bits   = 0u;
    s_crc_err      = 0u;
    s_uart_err     = 0u;
    s_unknown_magic = 0u;
    s_rx_pkt_count = 0u;
    // Init each of the N centroid streams
    for (uint8_t i = 0u; i < STREAM_COUNTS; i++) {
        s_last_rx_ms[i]          = 0u;
        s_last_centroid_dt_ms[i] = 0u;
    }
    // Init SBC Setters and getters
    s_state_req_fresh  = 0u;
    s_gps_target_fresh = 0u;
    s_nudge_fresh      = 0u;
    s_param_fresh      = 0u;
    s_set_origin_fresh = 0u;
    s_param_get_fresh   = 0u;
    s_fatal_clear_fresh = 0u;
}

static void decode_centroid(uint8_t stream, const uint8_t *p)
{
    // Store time since last centroid arrived.
    uint32_t now = g_tick_ms;
    if (s_last_rx_ms[stream] != 0u) {
        uint32_t dt = now - s_last_rx_ms[stream];
        s_last_centroid_dt_ms[stream] = (dt > 0xFFFFu)  // Clamp to uint16
                                      ? 0xFFFFu         // Max to 65 seconds
                                      : (uint16_t)dt;   // Cast down
    }
    // COpy into array of centroid structs
    memcpy(&s_centroid[stream].t_ms,  p +  0, 4);
    memcpy(&s_centroid[stream].cx,    p +  4, 2);
    memcpy(&s_centroid[stream].cy,    p +  6, 2);
    memcpy(&s_centroid[stream].cxerr, p +  8, 2);
    memcpy(&s_centroid[stream].cyerr, p + 10, 2);
    // Pip freshness into the streams byte
    s_fresh_bits |= (uint8_t)(1u << stream);
    s_last_rx_ms[stream] = now;  // Update last heard from
}

// Incoming byte parser!
// Parses each part of the packet structure,
// then calls the apropriate decoder!
void sbc_on_rx_byte(uint8_t b)
{
    // Parse incoming bytes according to expected packet structure.
    switch (sbc_rx_state) {
        case S_MAGIC:
            if (b == SBC_MAGIC){
                sbc_rx_state = S_TYPE;  // Move to type parsing
            } else {
                s_unknown_magic++;  // Measure of noise
            }
            break;
        case S_TYPE:
            s_type = b;  // Save bye as type
            sbc_rx_state = S_LEN;  // Move to length parsing
            break;
        case S_LEN:
            s_len = b;
            if (s_len > SBC_RX_MAX) {
                sbc_rx_state = S_MAGIC;  // Bad len -> resync
            } else if (s_len == 0u) {
                sbc_rx_state = S_CRC;    // 0-byte payload legal
            } else {
                s_idx = 0u;               // Reset to start of payload
                sbc_rx_state = S_PAYLOAD;  // Move to payload parsing
            }
            break;
        case S_PAYLOAD:
            s_buf[s_idx++] = b;  // Load bytes into buffer
            if (s_idx >= s_len){
                sbc_rx_state = S_CRC;  // Move to CRC when reached tot length
            }
            break;
        case S_CRC: {
            // CRC over TYPE + LEN + PAYLOAD
            // byte-wise XOR then CRC lookup.
            uint8_t calc = crc8_table[s_type];  // On type
            calc = crc8_table[calc ^ s_len];    // On length
            for (uint8_t i = 0u; i < s_len; i++)// Over payload
                calc = crc8_table[calc ^ s_buf[i]];

            // Verify checksum, if good then decode!
            if (calc == b) {
                s_rx_pkt_count++;
                // Process payload bytes according to type.
                // Add as we accumulate more things to Rx.
                switch (s_type) {
                    case PKT_CENTROID_C:
                        if (s_len == SBC_CENTROID_LEN){
                            decode_centroid(STREAM_COARSE, s_buf);
                        }
                        break;
                    case PKT_CENTROID_F:
                        if (s_len == SBC_CENTROID_LEN){
                            decode_centroid(STREAM_FINE,   s_buf);
                        }
                        break;
                    case PKT_STATE_REQ:
                        if (s_len == SBC_STATE_REQ_LEN) {
                            s_state_req.requested_state = s_buf[0];
                            s_state_req_fresh = 1u;
                        }
                        break;
                    case PKT_GPS_TARGET:
                        if (s_len == SBC_GPS_TARGET_LEN) {
                            memcpy(&s_gps_target.lat_raw, s_buf + 0, 4);
                            memcpy(&s_gps_target.lon_raw, s_buf + 4, 4);
                            memcpy(&s_gps_target.alt_mm,  s_buf + 8, 4);
                            s_gps_target_fresh = 1u;
                        }
                        break;
                    case PKT_NUDGE:
                        if (s_len == SBC_NUDGE_LEN) {
                            memcpy(&s_nudge.dpan_cdeg,  s_buf + 0, 2);
                            memcpy(&s_nudge.dtilt_cdeg, s_buf + 2, 2);
                            s_nudge_fresh = 1u;
                        }
                        break;
                    case PKT_PARAM_SET:
                        if (s_len == SBC_PARAM_LEN) {
                            s_param.id = s_buf[0];
                            memcpy(&s_param.value, s_buf + 1, 4);
                            s_param_fresh = 1u;
                        }
                        break;
                    case PKT_SET_ORIGIN:
                        if (s_len == SBC_SET_ORIGIN_LEN) {
                            s_set_origin_fresh = 1u;
                        }
                        break;
                    case PKT_PARAM_GET:
                        if (s_len == SBC_PARAM_GET_LEN) {
                            s_param_get.id = s_buf[0];
                            s_param_get_fresh = 1u;
                        }
                        break;
                    case PKT_FATAL_CLEAR:
                        if (s_len == SBC_FATAL_CLEAR_LEN) {
                            s_fatal_clear_fresh = 1u;
                        }
                        break;
                    default:
                        break;
                }
            } else {
                s_crc_err++;
            }
            sbc_rx_state = S_MAGIC;
            break;
        }
    }
}

void sbc_on_uart_err_flags(uint8_t flags)
{
    // If UART errors, accumulate counter and reset to magic (next frame)
    (void)flags;
    s_uart_err++;
    sbc_rx_state = S_MAGIC;
}

uint8_t sbc_get_centroid(uint8_t stream, payload_centroid_t *out)
{
    // Getter function for the rest of our codebase to use
    // Runs as atomic so it cannot be interrupted. If it were interrupted we
    // could start copying out the centroid, then rx a new centroid, and
    // continue reading out from the new centroid data.
    // This would corrupt the data, or at least confuse our control loop.
    if (stream >= STREAM_COUNTS) return 0u;
    uint8_t bit = (uint8_t)(1u << stream);
    uint8_t got = 0u;
    uint8_t ist = CyEnterCriticalSection();// Disables interruprs
    if (s_fresh_bits & bit) {
        *out = s_centroid[stream];  // Copy out the centroid
        s_fresh_bits &= (uint8_t)~bit;  // Set stale (already grabbed)
        got = 1u;  // Say we got em.
        if (stream == STREAM_COARSE) DPIN_CENTROID();
    }
    CyExitCriticalSection(ist);  // Re-enables interruprs;
    // Toggle relevant debug pin
    return got;  // Caller is told they got a new centroid!
}

void sbc_discard_centroid(uint8_t stream)
{
    // Discard centroid packets during slew process to avoid jumoy snaps on exit.
    if (stream >= STREAM_COUNTS) return;  // No centroids
    uint8_t bit = (uint8_t)(1u << stream);
    uint8_t ist = CyEnterCriticalSection();  // Atomic clear
    s_fresh_bits &= (uint8_t)~bit;
    CyExitCriticalSection(ist);
}

uint16_t sbc_unknown_magic(void)
{
    // When magic byte unknown
    return s_unknown_magic;
}

uint32_t sbc_rx_pkt_count(void)
{
    // Received packet count getter
    return s_rx_pkt_count;
}
uint16_t sbc_last_centroid_dt_ms(uint8_t stream)
{
    // Time between last centroid pair getter
    return (stream < STREAM_COUNTS) ? s_last_centroid_dt_ms[stream] : 0u;
}

uint32_t sbc_last_rx_ms(uint8_t stream)
{
    // Time of last received packet getter
    return (stream < STREAM_COUNTS) ? s_last_rx_ms[stream] : 0u;
}
uint16_t sbc_crc_errors(void)
{
    // CRC error getter
    return s_crc_err;
}
uint16_t sbc_uart_errors(void)
{
    // UART error getter
    return s_uart_err;
}

void sbc_send_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    // Encoder to send a frame to the Pi
    UART_SBC_PutChar(SBC_MAGIC);  // Magic byte
    UART_SBC_PutChar(type);      // Type byte
    UART_SBC_PutChar(len);       // Length byte
    uint8_t crc = crc8_table[type];
    crc = crc8_table[crc ^ len];// Start CRC
    for (uint8_t i = 0u; i < len; i++) {
        UART_SBC_PutChar(payload[i]);  // Put payload and compute CRC
        crc = crc8_table[crc ^ payload[i]];
    }
    UART_SBC_PutChar(crc);       // Put calculates CRC
}

uint8_t sbc_get_state_req(payload_state_req_t *out)
{
    uint8_t got = 0u;
    uint8_t ist = CyEnterCriticalSection();
    if (s_state_req_fresh) {
        *out = s_state_req;
        s_state_req_fresh = 0u;
        got = 1u;
    }
    CyExitCriticalSection(ist);
    return got;
}

uint8_t sbc_get_gps_target(payload_gps_target_t *out)
{
    uint8_t got = 0u;
    uint8_t ist = CyEnterCriticalSection();
    if (s_gps_target_fresh) {
        *out = s_gps_target;
        s_gps_target_fresh = 0u;
        got = 1u;
    }
    CyExitCriticalSection(ist);
    return got;
}

uint8_t sbc_get_nudge(payload_nudge_t *out)
{
    uint8_t got = 0u;
    uint8_t ist = CyEnterCriticalSection();
    if (s_nudge_fresh) {
        *out = s_nudge;
        s_nudge_fresh = 0u;
        got = 1u;
    }
    CyExitCriticalSection(ist);
    return got;
}

uint8_t sbc_get_param(payload_param_t *out)
{
    uint8_t got = 0u;
    uint8_t ist = CyEnterCriticalSection();
    if (s_param_fresh) {
        *out = s_param;
        s_param_fresh = 0u;
        got = 1u;
    }
    CyExitCriticalSection(ist);
    return got;
}

uint8_t sbc_get_param_read(payload_param_get_t *out)
{
    uint8_t got = 0u;
    uint8_t ist = CyEnterCriticalSection();
    if (s_param_get_fresh) {
        *out = s_param_get;
        s_param_get_fresh = 0u;
        got = 1u;
    }
    CyExitCriticalSection(ist);
    return got;
}

uint8_t sbc_get_fatal_clear(void)
{
    uint8_t got = 0u;
    uint8_t ist = CyEnterCriticalSection();
    if (s_fatal_clear_fresh)
    {
        s_fatal_clear_fresh = 0u;
        got = 1u;
    }
    CyExitCriticalSection(ist);
    return got;
}

void sbc_send_cmd_ack(uint8_t cmd_type, uint8_t result)
{
    uint8_t payload[2] = { cmd_type, result };
    sbc_send_frame(PKT_CMD_ACK, payload, sizeof payload);
}

void sbc_send_param_value(uint8_t id, float value)
{
    uint8_t payload[5];
    payload[0] = id;
    memcpy(payload + 1, &value, 4);
    sbc_send_frame(PKT_PARAM_VALUE, payload, sizeof payload);
}

uint8_t sbc_get_set_origin(void)
{
    uint8_t got = 0u;
    uint8_t ist = CyEnterCriticalSection();
    if (s_set_origin_fresh) {
        s_set_origin_fresh = 0u;
        got = 1u;
    }
    CyExitCriticalSection(ist);
    return got;
}

void sbc_send_state_ack(uint8_t actual_state, uint8_t result)
{
    uint8_t payload[2] = { actual_state, result };
    sbc_send_frame(PKT_STATE_ACK, payload, sizeof payload);
}

/* [] END OF FILE */
