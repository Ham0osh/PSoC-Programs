#include "movi_comm.h"
#include <string.h>
#include <stdio.h>

/* Freefly/QX */
#include "QX_Protocol.h"
#include "simple_buffer.h"

#define MOVI_QX_PORT        (QX_COMMS_PORT_UART)
#define GPS_POLL_PERIOD_MS  200u   /* 5 Hz */

/* -----------------------------------------------------------------------
 * GPS parsing — no separate QX client needed.
 * QX_StreamRxCharSM populates QX_CommsPorts[port].RxMsg before returning.
 * We inspect it inline in movi_comm_pump after got_packet is true.
 * ----------------------------------------------------------------------- */
static void parse_gps_from_rxmsg(movi_status_t *st)
{
    const QX_Msg_t *msg = &QX_CommsPorts[MOVI_QX_PORT].RxMsg;

    if (msg->Header.Attrib != 4u) return;
    if (msg->Header.MsgLength < 23u) return;

    const uint8_t *p = msg->BufPayloadStart_p;
    if (!p) return;

    /* Offsets from Firmware6_2 ParameterStructure (1-based, byte 0 is
     * the QX sub-index byte).  All little-endian.                        */
    int32_t lon_raw, lat_raw, alt_raw;
    int16_t spd_raw, hdg_raw, hacc_raw, vacc_raw, sacc_raw;

    memcpy(&lon_raw,  p + 1,  4);   /* Longitude    scale 1e7   */
    memcpy(&lat_raw,  p + 5,  4);   /* Latitude     scale 1e7   */
    memcpy(&alt_raw,  p + 9,  4);   /* Altitude     scale 1000  */
    memcpy(&spd_raw,  p + 13, 2);   /* Ground speed scale 100   */
    memcpy(&hdg_raw,  p + 15, 2);   /* Heading      scale 10    */
    memcpy(&hacc_raw, p + 17, 2);   /* Hacc         scale 100   */
    memcpy(&vacc_raw, p + 19, 2);   /* Vacc         scale 100   */
    memcpy(&sacc_raw, p + 21, 2);   /* Sacc         scale 100   */

    st->gps_lat_deg       = (float)lat_raw  / 1e7f;
    st->gps_lon_deg       = (float)lon_raw  / 1e7f;
    st->gps_alt_m         = (float)alt_raw  / 1000.0f;
    st->gps_ground_spd_ms = (float)spd_raw  / 100.0f;
    st->gps_heading_deg   = (float)hdg_raw  / 10.0f;
    st->gps_hacc_m        = (float)hacc_raw / 100.0f;
    st->gps_vacc_m        = (float)vacc_raw / 100.0f;
    st->gps_sacc_ms       = (float)sacc_raw / 100.0f;
    st->gps_valid         = true;
}

static void load_freefly_control_from_movi(const movi_control_t* in)
{
    /* If you want enable to “gate” output, do it here */
    if (in->enable == 0u) {
        FreeflyAPI.control.pan.type  = DEFER;
        FreeflyAPI.control.tilt.type = DEFER;
        FreeflyAPI.control.roll.type = DEFER;
        FreeflyAPI.control.focus.type = DEFER;
        FreeflyAPI.control.iris.type  = DEFER;
        FreeflyAPI.control.zoom.type  = DEFER;

        FreeflyAPI.control.pan.value  = 0.0f;
        FreeflyAPI.control.tilt.value = 0.0f;
        FreeflyAPI.control.roll.value = 0.0f;
        FreeflyAPI.control.focus.value = 0.0f;
        FreeflyAPI.control.iris.value  = 0.0f;
        FreeflyAPI.control.zoom.value  = 0.0f;
    } else {
        FreeflyAPI.control.pan.type  = in->pan_mode;
        FreeflyAPI.control.tilt.type = in->tilt_mode;
        FreeflyAPI.control.roll.type = in->roll_mode;

        FreeflyAPI.control.pan.value  = in->pan;
        FreeflyAPI.control.tilt.value = in->tilt;
        FreeflyAPI.control.roll.value = in->roll;

        /* Leave lens DEFER for now */
        FreeflyAPI.control.focus.type = DEFER;
        FreeflyAPI.control.iris.type  = DEFER;
        FreeflyAPI.control.zoom.type  = DEFER;
        FreeflyAPI.control.focus.value = 0.0f;
        FreeflyAPI.control.iris.value  = 0.0f;
        FreeflyAPI.control.zoom.value  = 0.0f;
    }

    /* One-shot kill flag */
    FreeflyAPI.control.gimbal_kill = (in->kill ? 1u : 0u);

    /* Most setups use euler-style controls; leave quaternions off */
    FreeflyAPI.control.gimbal_position_type_quaternions = 0u;

    /* Leave FIZ flags alone unless you wire them */
    FreeflyAPI.control.fiz_clearFaults_all_flag  = 0u;
    FreeflyAPI.control.fiz_autoCalStart_all_flag = 0u;
    FreeflyAPI.control.fiz_record_button_flag    = 0u;
    FreeflyAPI.control.fiz_setSubRangeLim_F_flag = 0u;
    FreeflyAPI.control.fiz_setSubRangeLim_I_flag = 0u;
    FreeflyAPI.control.fiz_setSubRangeLim_Z_flag = 0u;
}

static void copy_status_from_freefly(movi_status_t* out)
{
    out->battery_left_v  = FreeflyAPI.status.battery_v_left;
    out->battery_right_v = FreeflyAPI.status.battery_v_right;
    out->gimbal_r        = FreeflyAPI.status.gimbal_r;
    out->gimbal_i        = FreeflyAPI.status.gimbal_i;
    out->gimbal_j        = FreeflyAPI.status.gimbal_j;
    out->gimbal_k        = FreeflyAPI.status.gimbal_k;
    out->gimbal_status1  = FreeflyAPI.status.gimbal_Status1;
    out->gimbal_status2  = FreeflyAPI.status.gimbal_Status2;
    out->valid = true;
}

void movi_comm_init(movi_comm_t* m, const movi_hal_t* hal)
{
    if (!m || !hal || !hal->uart_putc) return;

    memset(m, 0, sizeof(*m));
    m->hal = *hal;
    movi_rb_init(&m->rb);

    m->status.valid     = false;
    m->status.gps_valid = false;
    m->gps_poll_ms      = 0u;

    /* Initializes QX client and default API fields */
    FreeflyAPI.begin();
    /* No second QX_InitCli — we parse GPS inline from RxMsg in pump() */
}

void movi_comm_reset(movi_comm_t* m)
{
    if (!m) return;

    movi_rb_clear(&m->rb);
    memset(&m->statistics, 0, sizeof(m->statistics));
    m->status.valid = false;
}

void movi_comm_clear_statistics(movi_comm_t* m)
{
    if (!m) return;
    memset(&m->statistics, 0, sizeof(m->statistics));
}

void movi_comm_on_uart_error_flags(movi_comm_t* m, uint8_t err_mask)
{
    if (!m) return;
    m->statistics.uart_err_flags |= err_mask;
}

void movi_comm_on_rx_byte(movi_comm_t* m, uint8_t b)
{
    if (!m) return;

    (void)movi_rb_push(&m->rb, b);
    m->statistics.rb_drops = m->rb.drops;
}

void movi_comm_pump(movi_comm_t* m)
{
    if (!m) return;

    uint8_t b;

    /* Track checksum fails using QX port counter */
    uint32_t chkfail_before = QX_CommsPorts[MOVI_QX_PORT].ChkSumFail_cnt;

    while (movi_rb_pop(&m->rb, &b)) {
        m->statistics.rx_bytes++;

        int got_packet = QX_StreamRxCharSM(MOVI_QX_PORT, (unsigned char)b);
        if (got_packet) {
            m->statistics.rx_packets++;
            /* Always sync attitude/battery from FreeflyAPI */
            copy_status_from_freefly(&m->status);
            /* Also check if this packet carries GPS (attribute 4) */
            parse_gps_from_rxmsg(&m->status);
        }
    }

    uint32_t chkfail_after = QX_CommsPorts[MOVI_QX_PORT].ChkSumFail_cnt;
    if (chkfail_after > chkfail_before) {
        m->statistics.rx_bad_checksum += (chkfail_after - chkfail_before);
    }

    m->statistics.rb_drops = m->rb.drops;

    /* ------------------------------------------------------------------ *
     * GPS poll — send a QX read request for attribute 4 at 5 Hz.         *
     * Uses QX_Clients[0] which FreeflyAPI.begin() already registered.    *
     * Drain the send buffer immediately so bytes don't mix with the next  *
     * control packet.                                                     *
     * ------------------------------------------------------------------ */
    uint32_t now = QX_GetTicks_ms();
    if ((now - m->gps_poll_ms) >= GPS_POLL_PERIOD_MS) {
        m->gps_poll_ms = now;

        QX_TxMsgOptions_t opts;
        QX_InitTxOptions(&opts);
        opts.Target_Addr = QX_DEV_ID_GIMBAL;

        QX_SendPacket_Cli_Read(&QX_Clients[0], 4u, MOVI_QX_PORT, opts);

        uint8_t byte;
        while (BufRemove(SEND_BUF_IDX, (volatile uint8_t*)&byte) != 0) {
            m->hal.uart_putc(m->hal.ctx, byte);
        }
    }
}

void movi_comm_set_control(movi_comm_t* m, const movi_control_t* ctl)
{
    if (!m || !ctl) return;
    m->ctl = *ctl;
}

void movi_comm_get_control(movi_comm_t* m, movi_control_t* out)
{
    if (!m || !out) return;
    *out = m->ctl;
}

void movi_comm_kill(movi_comm_t* m)
{
    if (!m) return;
    m->ctl.kill = 1u;
    (void)movi_comm_send_control(m);
    m->ctl.kill = 0u;
}

/* Not meaningful in the Freefly “send() then drain queue” architecture */
movi_result_t movi_comm_build_control_frame(movi_comm_t* m,
                                           uint8_t* out, uint16_t out_max, uint16_t* out_len)
{
    (void)m; (void)out; (void)out_max;
    if (out_len) *out_len = 0u;
    return MOVI_ERR_ENCODE;
}

movi_result_t movi_comm_send_control(movi_comm_t* m)
{
    if (!m || !m->hal.uart_putc) return MOVI_ERR_UART;

    load_freefly_control_from_movi(&m->ctl);

    /* Build/enqueue a QX277 control packet into SEND_BUF_IDX */
    FreeflyAPI.send();

    /* Drain the queued bytes and put them on the wire */
    uint8_t byte;
    uint32_t sent = 0;

    while (BufRemove(SEND_BUF_IDX, (volatile uint8_t*)&byte) != 0) {
        m->hal.uart_putc(m->hal.ctx, byte);
        sent++;
    }

    m->statistics.tx_packets++;
    m->statistics.tx_bytes += sent;

    return (m->statistics.uart_err_flags != 0u) ? MOVI_ERR_UART : MOVI_OK;
}

void movi_comm_get_status(movi_comm_t* m, movi_status_t* out)
{
    if (!m || !out) return;
    *out = m->status;
}

void movi_comm_get_statistics(movi_comm_t* m, movi_statistics_t* out)
{
    if (!m || !out) return;
    m->statistics.rb_drops = m->rb.drops;
    *out = m->statistics;
}