/* hamfly_gimbal.c
 * ============================================================
 * Copyright Hamish Johnson, 2026
 * Quantum Internet Systems Lab, SFU Physics
 * ============================================================
 *
 * Top-level gimbal instance. Direct translation of movi_comm.c
 * with the following changes:
 *   - movi_comm_t  -> hamfly_gimbal_t
 *   - movi_hal_t   -> hamfly_hal_t
 *   - movi_rb_*    -> hamfly_rb_*
 *   - simple_buffer BufAdd/BufRemove -> hamfly_txbuf_*
 *   - test_* fields -> pending_* fields
 *   - send_write_attr_u8 (was in main.c) added here
 *   - qx_active_gimbal set at init
 *   - decode functions called by attr ID via hamfly_telemetry
 */

#include "hamfly_core_gimbal.h"
#include "hamfly_core_telemetry.h"
#include "hamfly_qx_protocol.h"
#include "hamfly_qx_app.h"
#include <string.h>

#define HAMFLY_QX_PORT  (QX_COMMS_PORT_UART)

/* ============================================================
 * Internal helpers: load/save between hamfly_control_t
 * and FreeflyAPI.control
 * ============================================================ */
static void load_freefly_control(const hamfly_control_t *ctl)
{
    FreeflyAPI.control.pan.value   = ctl->pan;
    FreeflyAPI.control.tilt.value  = ctl->tilt;
    FreeflyAPI.control.roll.value  = ctl->roll;
    FreeflyAPI.control.pan.type    = (ff_api_control_type_e)ctl->pan_mode;
    FreeflyAPI.control.tilt.type   = (ff_api_control_type_e)ctl->tilt_mode;
    FreeflyAPI.control.roll.type   = (ff_api_control_type_e)ctl->roll_mode;
    FreeflyAPI.control.gimbal_kill = ctl->kill;
}

static void copy_qx287_to_telemetry(hamfly_telemetry_t *tel)
{
    /* 287 is already parsed into FreeflyAPI.status by the QX callback.
     * hamfly_decode_qx287 mirrors it into hamfly_telemetry_t.          */
    hamfly_decode_qx287(NULL, 0u, tel);
}

/* ============================================================
 * Drain txbuf to UART via HAL
 * ============================================================ */
static uint32_t drain_txbuf(hamfly_gimbal_t *g)
{
    uint8_t  byte;
    uint32_t sent = 0u;
    while (hamfly_txbuf_remove(&g->txbuf, &byte)) {
        g->hal.uart_putc(g->hal.ctx, byte);
        sent++;
    }
    return sent;
}

/* ============================================================
 * Lifecycle
 * ============================================================ */
void hamfly_init(hamfly_gimbal_t *g, const hamfly_hal_t *hal)
{
    memset(g, 0, sizeof(*g));
    g->hal       = *hal;
    g->gimbal_id = QX_DEV_ID_GIMBAL;
    hamfly_rb_init(&g->rb);
    hamfly_txbuf_init(&g->txbuf);

    /* Register as the active instance for QX callbacks */
    qx_active_gimbal_ptr = (void *)g;

    FreeflyAPI.begin();
}

void hamfly_reset(hamfly_gimbal_t *g)
{
    if (!g) return;
    hamfly_rb_clear(&g->rb);
    hamfly_txbuf_clear(&g->txbuf);
    memset(&g->telemetry,  0, sizeof(g->telemetry));
    memset(&g->statistics, 0, sizeof(g->statistics));
    g->pending_attr         = 0u;
    g->pending_sent_ms      = 0u;
    g->pending_ready        = false;
    g->pending_response_attr = 0u;
    g->pending_response_len  = 0u;
    g->pending_payload_len   = 0u;
}

void hamfly_clear_statistics(hamfly_gimbal_t *g)
{
    if (!g) return;
    memset(&g->statistics, 0, sizeof(g->statistics));
}

/* ============================================================
 * ISR-facing functions
 * ============================================================ */
void hamfly_on_rx_byte(hamfly_gimbal_t *g, uint8_t b)
{
    if (!g) return;
    (void)hamfly_rb_push(&g->rb, b);
    g->statistics.rb_drops = g->rb.drops;
}

void hamfly_on_uart_err_flags(hamfly_gimbal_t *g, uint8_t err_mask)
{
    if (!g) return;
    g->statistics.uart_err_flags |= err_mask;
}

/* ============================================================
 * hamfly_pump
 * Drain RX ring buffer into QX state machine.
 * On each complete packet: update telemetry, check pending.
 * ============================================================ */
void hamfly_pump(hamfly_gimbal_t *g)
{
    if (!g) return;

    uint8_t  b;
    uint32_t chkfail_before = QX_CommsPorts[HAMFLY_QX_PORT].ChkSumFail_cnt;

    while (hamfly_rb_pop(&g->rb, &b)) {
        g->statistics.rx_bytes++;

        int got_packet = QX_StreamRxCharSM(HAMFLY_QX_PORT, (unsigned char)b);
        if (!got_packet) continue;

        g->statistics.rx_packets++;

        uint16_t rxattr = (uint16_t)QX_CommsPorts[HAMFLY_QX_PORT].RxMsg.Header.Attrib;
        uint16_t rxlen  = (uint16_t)QX_CommsPorts[HAMFLY_QX_PORT].RxMsg.Header.MsgLength;
        const uint8_t *pay = QX_CommsPorts[HAMFLY_QX_PORT].RxMsg.BufPayloadStart_p;

        /* Always sync attr 287 from FreeflyAPI -- it auto-decodes via QX callback */
        copy_qx287_to_telemetry(&g->telemetry);

        /* Per-attribute decoders -- each checks its own attr ID via plen guard */
        if (pay) {
            switch (rxattr) {
                case 4u:  hamfly_decode_gps     (pay, rxlen, &g->telemetry); break;
                case 3u:  hamfly_decode_baro     (pay, rxlen, &g->telemetry); break;
                case 22u: hamfly_decode_attitude (pay, rxlen, &g->telemetry); break;
                case 1u:  hamfly_decode_sysstat  (pay, rxlen, &g->telemetry); break;
                case 12u: hamfly_decode_mag      (pay, rxlen, &g->telemetry); break;
                default: break;
            }
        }

        /* Pending transaction: snapshot payload if this is the awaited response.
         * Must happen before the while loop continues -- the next iteration of
         * QX_StreamRxCharSM will overwrite RxMsg.                              */
        if (g->pending_attr != 0u && rxattr == g->pending_attr) {
            g->pending_response_attr = rxattr;
            g->pending_response_len  = rxlen;
            g->pending_ready         = true;

            uint8_t snap_len = (rxlen < (uint16_t)sizeof(g->pending_payload))
                               ? (uint8_t)rxlen
                               : (uint8_t)sizeof(g->pending_payload);
            if (pay) memcpy(g->pending_payload, pay, snap_len);
            else     memset(g->pending_payload, 0,   snap_len);
            g->pending_payload_len = snap_len;
        }
    }

    uint32_t chkfail_after = QX_CommsPorts[HAMFLY_QX_PORT].ChkSumFail_cnt;
    if (chkfail_after > chkfail_before)
        g->statistics.rx_bad_checksum += (chkfail_after - chkfail_before);
    g->statistics.rb_drops = g->rb.drops;
}

/* ============================================================
 * hamfly_send_control
 * ============================================================ */
hamfly_result_t hamfly_send_control(hamfly_gimbal_t *g,
                                     const hamfly_control_t *ctl)
{
    if (!g || !g->hal.uart_putc) return HAMFLY_ERR_UART;

    load_freefly_control(ctl);
    g->ctl = *ctl;
    FreeflyAPI.send();

    uint32_t sent = drain_txbuf(g);
    g->statistics.tx_packets++;
    g->statistics.tx_bytes += sent;

    return (g->statistics.uart_err_flags != 0u) ? HAMFLY_ERR_UART : HAMFLY_OK;
}

/* ============================================================
 * hamfly_kill
 * ============================================================ */
void hamfly_kill(hamfly_gimbal_t *g)
{
    if (!g) return;
    hamfly_control_t k = g->ctl;
    k.kill = 1u;
    (void)hamfly_send_control(g, &k);
}

/* ============================================================
 * hamfly_request_attr
 * Send a one-shot QX read request. Arms the pending slot.
 * ============================================================ */
hamfly_result_t hamfly_request_attr(hamfly_gimbal_t *g, uint16_t attr_id)
{
    if (!g || !g->hal.uart_putc) return HAMFLY_ERR_UART;

    QX_TxMsgOptions_t opts;
    QX_InitTxOptions(&opts);
    opts.Target_Addr = (QX_DevId_e)g->gimbal_id;
    QX_SendPacket_Cli_Read(&QX_Clients[0], (uint32_t)attr_id,
                            HAMFLY_QX_PORT, opts);

    uint32_t sent = drain_txbuf(g);

    g->pending_attr         = attr_id;
    g->pending_sent_ms      = 0u;   /* caller sets this from their tick */
    g->pending_ready        = false;

    g->statistics.tx_packets++;
    g->statistics.tx_bytes += sent;

    return (g->statistics.uart_err_flags != 0u) ? HAMFLY_ERR_UART : HAMFLY_OK;
}

/* ============================================================
 * hamfly_request_attr_capture
 * Like hamfly_request_attr but also records the raw TX bytes.
 * Used by SCAN mode to log the exact wire frame.
 * ============================================================ */
hamfly_result_t hamfly_request_attr_capture(hamfly_gimbal_t *g,
                                              uint16_t attr_id,
                                              uint8_t *tx_buf,
                                              uint8_t  tx_buf_max,
                                              uint8_t *tx_len_out)
{
    if (!g || !g->hal.uart_putc) return HAMFLY_ERR_UART;

    QX_TxMsgOptions_t opts;
    QX_InitTxOptions(&opts);
    opts.Target_Addr = (QX_DevId_e)g->gimbal_id;
    QX_SendPacket_Cli_Read(&QX_Clients[0], (uint32_t)attr_id,
                            HAMFLY_QX_PORT, opts);

    uint8_t  byte;
    uint32_t sent = 0u;
    uint8_t  cap  = 0u;
    while (hamfly_txbuf_remove(&g->txbuf, &byte)) {
        g->hal.uart_putc(g->hal.ctx, byte);
        if (tx_buf && cap < tx_buf_max) tx_buf[cap++] = byte;
        sent++;
    }
    if (tx_len_out) *tx_len_out = cap;

    g->pending_attr    = attr_id;
    g->pending_sent_ms = 0u;
    g->pending_ready   = false;

    g->statistics.tx_packets++;
    g->statistics.tx_bytes += sent;

    return (g->statistics.uart_err_flags != 0u) ? HAMFLY_ERR_UART : HAMFLY_OK;
}

/* ============================================================
 * hamfly_write_attr_u8
 * Send a WRITE_ABS for any attr with a single-byte value.
 * Handles both single-byte varints (attr < 128) and two-byte.
 * Migrated from send_write_attr_u8() in main.c.
 * ============================================================ */
hamfly_result_t hamfly_write_attr_u8(hamfly_gimbal_t *g,
                                      uint16_t attr_id,
                                      uint8_t  value)
{
    if (!g || !g->hal.uart_putc) return HAMFLY_ERR_UART;

    uint8_t pkt[13];
    uint8_t idx = 0u, i;

    pkt[idx++] = 0x51u;  /* 'Q' */
    pkt[idx++] = 0x58u;  /* 'X' */

    if (attr_id < 128u) {
        pkt[idx++] = 0x07u;
        pkt[idx++] = (uint8_t)attr_id;
    } else {
        pkt[idx++] = 0x08u;
        pkt[idx++] = (uint8_t)((attr_id & 0x7Fu) | 0x80u);
        pkt[idx++] = (uint8_t)(attr_id >> 7u);
    }
    pkt[idx++] = 0x02u;  /* OPTIONS = WRITE_ABS */
    pkt[idx++] = 0x0Au;  /* SOURCE  = QX_DEV_ID_MOVI_API_CONTROLLER */
    pkt[idx++] = (uint8_t)g->gimbal_id;
    pkt[idx++] = 0x00u;  /* TRID */
    pkt[idx++] = 0x00u;  /* RRID */
    pkt[idx++] = value;

    uint16_t sum = 0u;
    for (i = 3u; i < idx; i++) sum += pkt[i];
    pkt[idx++] = (uint8_t)(255u - (sum & 0xFFu));

    uint32_t sent = 0u;
    for (i = 0u; i < idx; i++) {
        g->hal.uart_putc(g->hal.ctx, pkt[i]);
        sent++;
    }

    g->statistics.tx_packets++;
    g->statistics.tx_bytes += sent;

    return (g->statistics.uart_err_flags != 0u) ? HAMFLY_ERR_UART : HAMFLY_OK;
}

/* ============================================================
 * Accessors
 * ============================================================ */
void hamfly_get_telemetry(hamfly_gimbal_t *g, hamfly_telemetry_t *out)
{
    if (!g || !out) return;
    *out = g->telemetry;
}

void hamfly_get_statistics(hamfly_gimbal_t *g, hamfly_statistics_t *out)
{
    if (!g || !out) return;
    g->statistics.rb_drops = g->rb.drops;
    *out = g->statistics;
}
