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
 * Representation of in-memory outbound telemetry.
 * Agnostic and useable for SBC comms as well as local debug.
*/

#include "telemetry.h"
#include "sbc_comms.h"
#include "hamfly.h"

#include <string.h>
#include <project.h>

extern volatile uint32_t g_tick_ms;

// Quaternion encoding helper
static int16_t sat_i16(float f)
{
    if (f >=  32767.0f) return  32767;
    if (f <= -32768.0f) return -32768;
    return (int16_t)f;
}

// Helpers: Collectors
void telemetry_collect_hot(const app_ctx_t *ctx, telem_hot_t *out)
{
    memset(out, 0, sizeof *out);
    out->t_ms  = g_tick_ms;
    out->state = (uint8_t)ctx->state;

    if (!ctx->gimbal) return;

    const hamfly_control_t *ctl = &ctx->gimbal->ctl;
    out->ctrl_mode_pan  = (uint8_t)ctl->pan_mode;
    out->ctrl_mode_tilt = (uint8_t)ctl->tilt_mode;
    out->ctrl_mode_roll = (uint8_t)ctl->roll_mode;
    out->enable         = ctl->enable;
    out->kill           = ctl->kill;
    out->input_pan      = sat_i16(ctl->pan  * 100.0f);
    out->input_tilt     = sat_i16(ctl->tilt * 100.0f);
    out->input_roll     = sat_i16(ctl->roll * 100.0f);

    hamfly_telemetry_t tel;
    hamfly_get_telemetry(ctx->gimbal, &tel);

    out->gimbal_status1 = tel.gimbal_status1;
    out->gimbal_status2 = tel.gimbal_status2;
    out->q_i = sat_i16(tel.gimbal_i * 32767.0f);
    out->q_j = sat_i16(tel.gimbal_j * 32767.0f);
    out->q_k = sat_i16(tel.gimbal_k * 32767.0f);
    out->q_r = sat_i16(tel.gimbal_r * 32767.0f);
}

void telemetry_collect_link(const app_ctx_t *ctx, telem_link_t *out)
{
    (void)ctx;
    memset(out, 0, sizeof *out);
    out->t_ms                = g_tick_ms;
    out->crc_fail            = sbc_crc_errors();
    out->uart_err            = sbc_uart_errors();
    out->unknown_magic       = sbc_unknown_magic();
    out->rx_pkt_count        = sbc_rx_pkt_count();
    out->last_centroid_dt_ms = sbc_last_centroid_dt_ms(STREAM_COARSE);
}

// Packet encoders on the SBC wire
#define TELEM_HOT_LEN   26u
#define TELEM_LINK_LEN  16u

static uint8_t encode_hot(const telem_hot_t *h, uint8_t *buf)
{
    uint8_t *p = buf;
    memcpy(p, &h->t_ms,         4); p += 4;
    *p++ = h->state;
    *p++ = h->ctrl_mode_pan;
    *p++ = h->ctrl_mode_tilt;
    *p++ = h->ctrl_mode_roll;
    *p++ = h->enable;
    *p++ = h->kill;
    memcpy(p, &h->input_pan,    2); p += 2;
    memcpy(p, &h->input_tilt,   2); p += 2;
    memcpy(p, &h->input_roll,   2); p += 2;
    *p++ = h->gimbal_status1;
    *p++ = h->gimbal_status2;
    memcpy(p, &h->q_i,          2); p += 2;
    memcpy(p, &h->q_j,          2); p += 2;
    memcpy(p, &h->q_k,          2); p += 2;
    memcpy(p, &h->q_r,          2); p += 2;
    return (uint8_t)(p - buf);  //  == TELEM_HOT_LEN
}

static uint8_t encode_link(const telem_link_t *l, uint8_t *buf)
{
    uint8_t *p = buf;
    memcpy(p, &l->t_ms,                4); p += 4;
    memcpy(p, &l->crc_fail,            2); p += 2;
    memcpy(p, &l->uart_err,            2); p += 2;
    memcpy(p, &l->unknown_magic,       2); p += 2;
    memcpy(p, &l->rx_pkt_count,        4); p += 4;
    memcpy(p, &l->last_centroid_dt_ms, 2); p += 2;
    return (uint8_t)(p - buf);  //  == TELEM_LINK_LEN
}


// Packet senders
void telemetry_send_hot_sbc(const app_ctx_t *ctx)
{
    telem_hot_t h;
    telemetry_collect_hot(ctx, &h);
    uint8_t buf[TELEM_HOT_LEN];
    uint8_t n = encode_hot(&h, buf);
    sbc_send_frame(PKT_TELEM_HOT, buf, n);
}

void telemetry_send_link_sbc(const app_ctx_t *ctx)
{
    telem_link_t l;
    telemetry_collect_link(ctx, &l);
    uint8_t buf[TELEM_LINK_LEN];
    uint8_t n = encode_link(&l, buf);
    sbc_send_frame(PKT_TELEM_LINK, buf, n);
}

// Gimbal sensor attribultes and request scheduling
typedef struct {
    uint16_t attr_id;           // HAMFLY_ATTR_SYSSTAT, etc.
    uint32_t period_ms;         // Should all be uniform
    uint32_t last_request_ms;   // Tibreaker
} attr_sched_t;

static attr_sched_t s_attrs[] = {
    //  { HAMFLY_ATTR_SYSSTAT, 5000u, 0u },   //  POWER + ENV cold
    //  { HAMFLY_ATTR_BARO,    5000u, 0u },   //  BARO cold
    //  { HAMFLY_ATTR_GPS,     5000u, 0u },   //  GPS cold
    //  ... examples
};
#define N_ATTRS (sizeof s_attrs / sizeof s_attrs[0])

void telemetry_pump(app_ctx_t *ctx)
{
    // Earcly check if gimbal exists, and there are attrs to grab
    if (!ctx->gimbal || N_ATTRS == 0u) return;
    // Yield to control packet being sent to Movi
    if (g_tick_ms - ctx->last_tx_ms < 5u) return;

    uint32_t now = g_tick_ms;
    // Pick the oldest telemetry getter.
    size_t   chosen = (size_t)-1;
    uint32_t worst_overdue = 0u;
    for (size_t i = 0u; i < N_ATTRS; i++) {
        uint32_t age = now - s_attrs[i].last_request_ms;
        if (age >= s_attrs[i].period_ms && age >= worst_overdue) {
            worst_overdue = age;
            chosen = i;
        }
    }
    if (chosen == (size_t)-1) return;  // No attr passed its period.
    
    // Try to request for the attribute
    if (hamfly_request_attr(ctx->gimbal, s_attrs[chosen].attr_id) == HAMFLY_OK){
        s_attrs[chosen].last_request_ms = now;
    }
}

/* [] END OF FILE */
