#include "movi_comm.h"
#include <string.h>
#include <stdio.h>

/* Freefly/QX */
#include "QX_Protocol.h"
#include "simple_buffer.h"

#define MOVI_QX_PORT (QX_COMMS_PORT_UART)

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

    m->status.valid = false;

    /* Initializes QX client and default API fields */
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

    /* Track checksum fails using QX port counter */
    uint32_t chkfail_before = QX_CommsPorts[MOVI_QX_PORT].ChkSumFail_cnt;

    while (movi_rb_pop(&m->rb, &b)) {
        m->statistics.rx_bytes++;
        
        int got_packet = QX_StreamRxCharSM(MOVI_QX_PORT, (unsigned char)b);
        // Print if a valid QX packet was parsed
        if (got_packet) {
            // UART_1_PutString(" [QX OK]\r\n");
            m->statistics.rx_packets++;
            copy_status_from_freefly(&m->status);
        } else {
            // UART_1_PutString("\r\n");
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
