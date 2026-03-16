/* movi_comm.c
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
 * Communications library to interface with
 * the FreeflyAPI as a HAL.
*/

#include "movi_comm.h"
#include <string.h>
#include <stdio.h>

#include "QX_Protocol.h"
#include "simple_buffer.h"

#define MOVI_QX_PORT  (QX_COMMS_PORT_UART)

/* -----------------------------------------------------------------------
 * Big-endian read helpers.
 * GPS (attr 4) and sysstat (attr 1) payload bytes are big-endian and have
 * NO sub-index byte — data starts at offset 0.  Confirmed by cross-checking
 * against iOS app ground-truth values (lon/lat/batt_v/temp/pressure).
 * Attr 22 (attitude) is little-endian with a 0x00 sub-index at offset 0.
 * ----------------------------------------------------------------------- */
static int32_t rd_be32(const uint8_t *p)
{
    return (int32_t)(  ((uint32_t)p[0] << 24)
                     | ((uint32_t)p[1] << 16)
                     | ((uint32_t)p[2] <<  8)
                     |  (uint32_t)p[3]);
}
static int16_t rd_be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

/* -----------------------------------------------------------------------
 * parse_gps_from_rxmsg  —  attr 4
 *
 * Payload is big-endian, no sub-index byte.  Layout confirmed from hex dump
 * against known coordinates (Burnaby, BC):
 *   off  0-3 BE int32: Longitude  scale 1e7   (-122.917 deg confirmed)
 *   off  4-7 BE int32: Latitude   scale 1e7   (+49.277  deg confirmed)
 *   off  8-11 BE int32: Altitude  scale 1000
 *   off 12-13 BE int16: Gnd speed scale 100
 *   off 14-15 BE int16: Heading   scale 10
 *   off 16-17 BE int16: HACC      scale 100
 *   off 18-19 BE int16: VACC      scale 100
 *   off 20-21 BE int16: SACC      scale 100
 * ----------------------------------------------------------------------- */
static void parse_gps_from_rxmsg(movi_status_t *st)
{
    const QX_Msg_t *msg = &QX_CommsPorts[MOVI_QX_PORT].RxMsg;
    if (msg->Header.Attrib != 4u)    return;
    if (msg->Header.MsgLength < 22u) return;
    const uint8_t *p = msg->BufPayloadStart_p;
    if (!p) return;

    int32_t lon_raw = rd_be32(p +  0);
    int32_t lat_raw = rd_be32(p +  4);
    int32_t alt_raw = rd_be32(p +  8);
    int16_t spd_raw = rd_be16(p + 12);
    int16_t hdg_raw = rd_be16(p + 14);
    int16_t hacc_raw = rd_be16(p + 16);
    int16_t vacc_raw = rd_be16(p + 18);
    int16_t sacc_raw = rd_be16(p + 20);

    st->gps_raw_lon  = lon_raw;
    st->gps_raw_lat  = lat_raw;
    st->gps_raw_alt  = alt_raw;
    st->gps_raw_spd  = spd_raw;
    st->gps_raw_hdg  = hdg_raw;
    st->gps_raw_hacc = hacc_raw;
    st->gps_raw_vacc = vacc_raw;
    st->gps_raw_sacc = sacc_raw;

    st->gps_lon_deg       = (float)lon_raw  / 1e7f;
    st->gps_lat_deg       = (float)lat_raw  / 1e7f;
    st->gps_alt_m         = (float)alt_raw  / 1000.0f;
    st->gps_ground_spd_ms = (float)spd_raw  / 100.0f;
    st->gps_heading_deg   = (float)hdg_raw  / 10.0f;
    st->gps_hacc_m        = (float)hacc_raw / 100.0f;
    st->gps_vacc_m        = (float)vacc_raw / 100.0f;
    st->gps_sacc_ms       = (float)sacc_raw / 100.0f;
    st->gps_valid         = true;
}

/* -----------------------------------------------------------------------
 * parse_baro_from_rxmsg  —  attr 3,  plen=5
 * ----------------------------------------------------------------------- */
static void parse_baro_from_rxmsg(movi_status_t *st)
{
    const QX_Msg_t *msg = &QX_CommsPorts[MOVI_QX_PORT].RxMsg;
    if (msg->Header.Attrib != 3u)  return;
    if (msg->Header.MsgLength < 5u) return;
    const uint8_t *p = msg->BufPayloadStart_p;
    if (!p) return;

    int16_t alt_raw, roc_raw;
    memcpy(&alt_raw, p + 1, 2);  /* Height  scale 10  -> m   */
    memcpy(&roc_raw, p + 3, 2);  /* ROC     scale 100 -> m/s */

    st->baro_alt_m  = (float)alt_raw / 10.0f;
    st->baro_roc_ms = (float)roc_raw / 100.0f;
    st->baro_valid  = true;
}

/* -----------------------------------------------------------------------
 * parse_attitude_from_rxmsg  —  attr 22,  plen=19
 *
 * All int16 LE.  Note: attr 287 gives quaternion from FreeflyAPI;
 * attr 22 gives euler angles directly and must be requested separately.
 * ----------------------------------------------------------------------- */
static void parse_attitude_from_rxmsg(movi_status_t *st)
{
    const QX_Msg_t *msg = &QX_CommsPorts[MOVI_QX_PORT].RxMsg;
    if (msg->Header.Attrib != 22u)  return;
    if (msg->Header.MsgLength < 19u) return;
    const uint8_t *p = msg->BufPayloadStart_p;
    if (!p) return;

    int16_t roll_r, pitch_r, yaw_r;
    int16_t roll_err_r, pitch_err_r, yaw_err_r;
    int16_t roll_rate_r, pitch_rate_r, yaw_rate_r;

    /* NOTE: Java ParameterStructure labels off 1 as "Roll", but observed data
     * shows off 3 matches Yaw and off 1 matches Pitch. The PN.* constants at
     * off 3 and off 5 appear to be Yaw and Roll respectively.
     * Layout confirmed against live GCU data: Pitch / Yaw / Roll @ 1 / 3 / 5. */
    memcpy(&roll_r,       p +  1, 2);  /* Pitch           scale 100 (Java: "Roll") */
    memcpy(&pitch_r,      p +  3, 2);  /* Yaw             scale 100 (Java: PN const) */
    memcpy(&yaw_r,        p +  5, 2);  /* Roll            scale 100 (Java: PN const) */
    memcpy(&roll_err_r,   p +  7, 2);  /* Roll Error      scale 10  -- may be stale in read response */
    memcpy(&pitch_err_r,  p +  9, 2);  /* Pitch Error     scale 10  -- may be stale */
    memcpy(&yaw_err_r,    p + 11, 2);  /* Yaw Error       scale 10  -- may be stale */
    memcpy(&roll_rate_r,  p + 13, 2);  /* Roll Rate Cmd   scale 10  -- may be stale */
    memcpy(&pitch_rate_r, p + 15, 2);  /* Pitch Rate Cmd  scale 10  -- may be stale */
    memcpy(&yaw_rate_r,   p + 17, 2);  /* Yaw Rate Cmd    scale 10  -- may be stale */

    st->att_pitch_deg      = (float)roll_r       / 100.0f;  /* off 1 = Pitch */
    st->att_yaw_deg        = (float)pitch_r      / 100.0f;  /* off 3 = Yaw   */
    st->att_roll_deg       = (float)yaw_r        / 100.0f;  /* off 5 = Roll  */
    st->att_roll_err_deg   = (float)roll_err_r   / 10.0f;
    st->att_pitch_err_deg  = (float)pitch_err_r  / 10.0f;
    st->att_yaw_err_deg    = (float)yaw_err_r    / 10.0f;
    st->att_roll_rate_dps  = (float)roll_rate_r  / 10.0f;
    st->att_pitch_rate_dps = (float)pitch_rate_r / 10.0f;
    st->att_yaw_rate_dps   = (float)yaw_rate_r   / 10.0f;
    st->att_valid          = true;
}

/* -----------------------------------------------------------------------
 * parse_sysstat_from_rxmsg  —  attr 1
 *
 * Payload is big-endian, no sub-index byte.  Layout confirmed from hex dump:
 *   off  0-1 BE int16: Battery voltage  scale 100  (23.22V confirmed)
 *   off  2   uint8:    GPS satellites              (0 confirmed)
 *   off  3   uint8:    Temperature       scale 3   (38.67C confirmed)
 *   off  4   uint8:    CPU %
 *   off  5-6 BE int16: Status flags
 *   off  7-8 BE int16: Time              scale 1   (s)
 *   off  9-10 BE int16: Pressure         scale 10  (984.0mb confirmed)
 *   off 11-12 BE int16: IMU rate
 *   off 13-14 BE int16: Cam status flags
 *   off 15    uint8:   (reserved / unknown)
 *   off 16-17 BE int16: IMU latency
 *   off 18-19 BE int16: Battery current  scale 100
 *   off 20-21 BE int16: Charge used      scale 1   (mAh)
 *
 * Note: fields off 11+ are shifted -1 from Firmware6_2 Java map (which had
 * a spurious sub-index byte at off 0).  Values for off 11+ need further
 * verification against iOS app — treat as provisional.
 * ----------------------------------------------------------------------- */
static void parse_sysstat_from_rxmsg(movi_status_t *st)
{
    const QX_Msg_t *msg = &QX_CommsPorts[MOVI_QX_PORT].RxMsg;
    if (msg->Header.Attrib != 1u)    return;
    if (msg->Header.MsgLength < 22u) return;
    const uint8_t *p = msg->BufPayloadStart_p;
    if (!p) return;

    st->sysstat_batt_v       = (float)rd_be16(p +  0) / 100.0f;
    st->sysstat_gps_sats     = p[2];
    st->sysstat_temp_c       = (float)p[3]            / 3.0f;
    st->sysstat_cpu_pct      = p[4];
    st->sysstat_status_flags = rd_be16(p +  5);
    st->sysstat_time_s       = rd_be16(p +  7);
    st->sysstat_pressure_mb  = (float)rd_be16(p +  9) / 10.0f;
    st->sysstat_imu_rate     = rd_be16(p + 11);
    st->sysstat_cam_status   = rd_be16(p + 13);
    /* off 15: reserved uint8 */
    st->sysstat_imu_latency  = rd_be16(p + 16);
    st->sysstat_batt_a       = (float)rd_be16(p + 18) / 100.0f;
    st->sysstat_charge_mah   = rd_be16(p + 20);
    st->sysstat_valid        = true;
}

/* -----------------------------------------------------------------------
 * parse_mag_from_rxmsg  —  attr 12,  plen=19
 * ----------------------------------------------------------------------- */
static void parse_mag_from_rxmsg(movi_status_t *st)
{
    const QX_Msg_t *msg = &QX_CommsPorts[MOVI_QX_PORT].RxMsg;
    if (msg->Header.Attrib != 12u)  return;
    if (msg->Header.MsgLength < 19u) return;
    const uint8_t *p = msg->BufPayloadStart_p;
    if (!p) return;

    int16_t x_r, y_r, z_r, mag_r, freq_r, ox_r, oy_r, oz_r, decl_r;

    memcpy(&x_r,    p +  1, 2);  /* Compass X      scale 1000 */
    memcpy(&y_r,    p +  3, 2);  /* Compass Y      scale 1000 */
    memcpy(&z_r,    p +  5, 2);  /* Compass Z      scale 1000 */
    memcpy(&mag_r,  p +  7, 2);  /* Magnitude      scale 100  */
    memcpy(&freq_r, p +  9, 2);  /* Freq           scale 100  */
    memcpy(&ox_r,   p + 11, 2);  /* Offset X       scale 1000 */
    memcpy(&oy_r,   p + 13, 2);  /* Offset Y       scale 1000 */
    memcpy(&oz_r,   p + 15, 2);  /* Offset Z       scale 1000 */
    memcpy(&decl_r, p + 17, 2);  /* Declination    scale 10   */

    st->mag_x           = (float)x_r    / 1000.0f;
    st->mag_y           = (float)y_r    / 1000.0f;
    st->mag_z           = (float)z_r    / 1000.0f;
    st->mag_magnitude   = (float)mag_r  / 100.0f;
    st->mag_freq        = (float)freq_r / 100.0f;
    st->mag_off_x       = (float)ox_r   / 1000.0f;
    st->mag_off_y       = (float)oy_r   / 1000.0f;
    st->mag_off_z       = (float)oz_r   / 1000.0f;
    st->mag_declination = (float)decl_r / 10.0f;
    st->mag_valid       = true;
}

/* -----------------------------------------------------------------------
 * Internal helpers — load/copy between API structs
 * ----------------------------------------------------------------------- */
static void load_freefly_control_from_movi(const movi_control_t* in)
{
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
        FreeflyAPI.control.focus.type = DEFER;
        FreeflyAPI.control.iris.type  = DEFER;
        FreeflyAPI.control.zoom.type  = DEFER;
        FreeflyAPI.control.focus.value = 0.0f;
        FreeflyAPI.control.iris.value  = 0.0f;
        FreeflyAPI.control.zoom.value  = 0.0f;
    }
    FreeflyAPI.control.gimbal_kill = (in->kill ? 1u : 0u);
    FreeflyAPI.control.gimbal_position_type_quaternions = 0u;
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

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */
void movi_comm_init(movi_comm_t* m, const movi_hal_t* hal)
{
    if (!m || !hal || !hal->uart_putc) return;
    memset(m, 0, sizeof(*m));
    m->hal = *hal;
    movi_rb_init(&m->rb);
    m->status.valid      = false;
    m->status.gps_valid  = false;
    m->status.baro_valid = false;
    m->status.att_valid  = false;
    m->status.sysstat_valid = false;
    m->status.mag_valid  = false;
    m->test_pending_attr  = 0u;
    m->test_sent_ms       = 0u;
    m->test_response_ready = false;
    FreeflyAPI.begin();
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
    uint32_t chkfail_before = QX_CommsPorts[MOVI_QX_PORT].ChkSumFail_cnt;

    while (movi_rb_pop(&m->rb, &b)) {
        m->statistics.rx_bytes++;

        int got_packet = QX_StreamRxCharSM(MOVI_QX_PORT, (unsigned char)b);
        if (got_packet) {
            m->statistics.rx_packets++;

            uint16_t rxattr = (uint16_t)QX_CommsPorts[MOVI_QX_PORT].RxMsg.Header.Attrib;
            uint16_t rxlen  = (uint16_t)QX_CommsPorts[MOVI_QX_PORT].RxMsg.Header.MsgLength;

            /* Always sync attitude quaternion + battery from attr 287 */
            copy_status_from_freefly(&m->status);

            /* Per-attribute parsers — each checks its own attr ID */
            parse_gps_from_rxmsg(&m->status);
            parse_baro_from_rxmsg(&m->status);
            parse_attitude_from_rxmsg(&m->status);
            parse_sysstat_from_rxmsg(&m->status);
            parse_mag_from_rxmsg(&m->status);

            /* Test mode: detect response to pending request.
             * CRITICAL: snapshot payload NOW before the while loop
             * continues draining the ring buffer and QX_StreamRxCharSM
             * overwrites RxMsg with the next attr 287 push.            */
            if (m->test_pending_attr != 0u && rxattr == m->test_pending_attr) {
                m->test_response_attr  = rxattr;
                m->test_rx_len         = rxlen;
                m->test_response_ready = true;

                /* Copy raw payload bytes into snapshot buffer */
                const uint8_t *pay = QX_CommsPorts[MOVI_QX_PORT].RxMsg.BufPayloadStart_p;
                uint8_t snap_len = (rxlen < 32u) ? (uint8_t)rxlen : 32u;
                if (pay) {
                    memcpy(m->test_rx_payload, pay, snap_len);
                } else {
                    memset(m->test_rx_payload, 0, snap_len);
                }
                m->test_rx_payload_len = snap_len;
            }
        }
    }

    uint32_t chkfail_after = QX_CommsPorts[MOVI_QX_PORT].ChkSumFail_cnt;
    if (chkfail_after > chkfail_before) {
        m->statistics.rx_bad_checksum += (chkfail_after - chkfail_before);
    }
    m->statistics.rb_drops = m->rb.drops;
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

movi_result_t movi_comm_build_control_frame(movi_comm_t* m,
                                            uint8_t* out, uint16_t out_max,
                                            uint16_t* out_len)
{
    (void)m; (void)out; (void)out_max;
    if (out_len) *out_len = 0u;
    return MOVI_ERR_ENCODE;
}

/* Send a one-shot QX read request for any attribute ID.
 * Arms test_pending_attr so pump() can match the response.
 * A new call silently replaces any previous pending request.          */
movi_result_t movi_comm_request_attr(movi_comm_t* m, uint16_t attr_id)
{
    if (!m || !m->hal.uart_putc) return MOVI_ERR_UART;

    QX_TxMsgOptions_t opts;
    QX_InitTxOptions(&opts);
    opts.Target_Addr = QX_DEV_ID_GIMBAL;

    QX_SendPacket_Cli_Read(&QX_Clients[0], (uint32_t)attr_id,
                           MOVI_QX_PORT, opts);

    uint8_t byte;
    uint32_t sent = 0u;
    while (BufRemove(SEND_BUF_IDX, (volatile uint8_t*)&byte) != 0) {
        m->hal.uart_putc(m->hal.ctx, byte);
        sent++;
    }

    /* Arm test pending state.
     * test_sent_ms is set by main.c using g_tick_ms immediately after this
     * returns — QX_GetTicks_ms() is not the same clock as g_tick_ms.       */
    m->test_pending_attr   = attr_id;
    m->test_sent_ms        = 0u;
    m->test_response_ready = false;

    m->statistics.tx_packets++;
    m->statistics.tx_bytes += sent;

    return (m->statistics.uart_err_flags != 0u) ? MOVI_ERR_UART : MOVI_OK;
}

movi_result_t movi_comm_send_control(movi_comm_t* m)
{
    if (!m || !m->hal.uart_putc) return MOVI_ERR_UART;

    load_freefly_control_from_movi(&m->ctl);
    FreeflyAPI.send();

    uint8_t byte;
    uint32_t sent = 0u;
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