/* hamfly_gimbal.h
 * ============================================================
 * Copyright Hamish Johnson, 2026
 * Quantum Internet Systems Lab, SFU Physics
 * ============================================================
 *
 * Top-level gimbal instance. One per physical MoVI Pro.
 * Owns the HAL, RX ring buffer, TX staging buffer,
 * telemetry, control state, statistics, and pending
 * transaction slot.
 *
 * Typical main-loop usage:
 *
 *   hamfly_on_rx_byte(&g, b);          // call from UART ISR
 *
 *   hamfly_pump(&g);                   // call every loop tick
 *   hamfly_send_control(&g, &ctl);     // call at TX rate (e.g. 10 Hz)
 *   hamfly_request_attr(&g, 22u);      // optional one-shot attr read
 *
 *   if (g.pending_ready) {             // response arrived
 *       // use g.pending_payload, g.pending_response_attr, etc.
 *       g.pending_ready = false;
 *       g.pending_attr  = 0u;
 *   }
 */

#ifndef HAMFLY_GIMBAL_H
#define HAMFLY_GIMBAL_H

#include <stdint.h>
#include <stdbool.h>

#include "hamfly_core_hal.h"
#include "hamfly_core_control.h"
#include "hamfly_core_telemetry.h"
#include "hamfly_comm_rb.h"
#include "hamfly_comm_txbuf.h"

typedef enum {
    HAMFLY_OK         = 0,
    HAMFLY_ERR_UART   = 1,
    HAMFLY_ERR_ENCODE = 2,
    HAMFLY_ERR_BUSY   = 3   /* another attr request is already pending */
} hamfly_result_t;

typedef struct {
    uint32_t tx_packets;
    uint32_t tx_bytes;
    uint32_t rx_packets;
    uint32_t rx_bytes;
    uint32_t rx_bad_checksum;
    uint16_t rb_drops;
    uint8_t  uart_err_flags;
} hamfly_statistics_t;

typedef struct {
    hamfly_hal_t        hal;
    hamfly_control_t    ctl;
    hamfly_telemetry_t  telemetry;
    hamfly_statistics_t statistics;
    hamfly_rb_t         rb;
    hamfly_txbuf_t      txbuf;

    /* GCU device address. Default = QX_DEV_ID_GIMBAL (2).
     * Future: set per-instance for multi-gimbal. */
    uint8_t gimbal_id;

    /* Pending transaction slot -- one outstanding request at a time.
     * Caller sets pending_attr and pending_sent_ms after hamfly_request_attr().
     * pump() sets pending_ready when the matching response arrives.
     * Caller clears pending_ready and pending_attr when done.          */
    uint16_t pending_attr;
    uint32_t pending_sent_ms;
    bool     pending_ready;
    uint16_t pending_response_attr;
    uint16_t pending_response_len;
    uint8_t  pending_payload[32];
    uint8_t  pending_payload_len;

} hamfly_gimbal_t;

/* Lifecycle */
void hamfly_init            (hamfly_gimbal_t *g, const hamfly_hal_t *hal);
void hamfly_reset           (hamfly_gimbal_t *g);
void hamfly_clear_statistics(hamfly_gimbal_t *g);

/* Called from ISR */
void hamfly_on_rx_byte        (hamfly_gimbal_t *g, uint8_t b);
void hamfly_on_uart_err_flags (hamfly_gimbal_t *g, uint8_t err_mask);

/* Called from main loop */
void           hamfly_pump          (hamfly_gimbal_t *g);
hamfly_result_t hamfly_send_control (hamfly_gimbal_t *g, const hamfly_control_t *ctl);
void           hamfly_kill          (hamfly_gimbal_t *g);

/* Attr read/write */
hamfly_result_t hamfly_request_attr (hamfly_gimbal_t *g, uint16_t attr_id);
hamfly_result_t hamfly_request_attr_capture(hamfly_gimbal_t *g, uint16_t attr_id,
                                             uint8_t *tx_buf, uint8_t tx_buf_max,
                                             uint8_t *tx_len_out);
hamfly_result_t hamfly_write_attr_u8(hamfly_gimbal_t *g, uint16_t attr_id, uint8_t value);

/* Accessors */
void hamfly_get_telemetry  (hamfly_gimbal_t *g, hamfly_telemetry_t  *out);
void hamfly_get_statistics (hamfly_gimbal_t *g, hamfly_statistics_t *out);

#endif /* HAMFLY_GIMBAL_H */
