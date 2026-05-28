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
// Helper for unsigned ints and avoid rollovers.
static uint32_t sat_u32(float f)
{
    if (f <= 0.0f)          return 0u;
    if (f >= 4294967295.0f) return 0xFFFFFFFFu;
    return (uint32_t)f;
}
static uint16_t sat_u16(float f)
{
    if (f <= 0.0f)          return 0u;
    if (f >= 65535.0f)      return 0xFFFFu;
    return (uint16_t)f;
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
void telemetry_collect_power(const app_ctx_t *ctx, telem_power_t *out)
{
    memset(out, 0, sizeof *out);
    out->t_ms = g_tick_ms;

    if (!ctx->gimbal) return;
    hamfly_telemetry_t tel;
    hamfly_get_telemetry(ctx->gimbal, &tel);

    out->vbat_left_mv    = sat_u16(tel.battery_left_v  * 1000.0f);
    out->vbat_right_mv   = sat_u16(tel.battery_right_v * 1000.0f);
    out->sysstat_batt_mv = sat_u16(tel.sysstat_batt_v  * 1000.0f);
    out->sysstat_batt_ma = sat_i16(tel.sysstat_batt_a  * 1000.0f);
    out->sysstat_cpu_pct = tel.sysstat_cpu_pct;

    //  TODO: wire DieTemp component if/when added to TopDesign.
    out->mcu_temp_c = 0;
}

void telemetry_collect_env(const app_ctx_t *ctx, telem_env_t *out)
{
    memset(out, 0, sizeof *out);
    out->t_ms = g_tick_ms;

    if (!ctx->gimbal) return;
    hamfly_telemetry_t tel;
    hamfly_get_telemetry(ctx->gimbal, &tel);

    out->sysstat_temp_c10  = sat_i16(tel.sysstat_temp_c     * 10.0f);
    out->sysstat_press_Pa  = sat_u32(tel.sysstat_pressure_mb * 100.0f);
}

void telemetry_collect_gps(const app_ctx_t *ctx, telem_gps_t *out)
{
    memset(out, 0, sizeof *out);
    out->t_ms = g_tick_ms;

    if (!ctx->gimbal) return;
    hamfly_telemetry_t tel;
    hamfly_get_telemetry(ctx->gimbal, &tel);

    out->flags   = (uint8_t)((tel.gps_valid  ? 0x01u : 0u)
                           | (tel.gps_locked ? 0x02u : 0u));
    out->sats    = tel.sysstat_gps_sats;   //  comes from SYSSTAT, not GPS attr
    out->raw_lat = tel.gps_raw_lat;
    out->raw_lon = tel.gps_raw_lon;
    out->raw_alt = tel.gps_raw_alt;
    out->raw_spd = tel.gps_raw_spd;
    out->raw_hdg = tel.gps_raw_hdg;
    out->hacc    = tel.gps_raw_hacc;
    out->vacc    = tel.gps_raw_vacc;
    out->sacc    = tel.gps_raw_sacc;
}

void telemetry_collect_baro(const app_ctx_t *ctx, telem_baro_t *out)
{
    memset(out, 0, sizeof *out);
    out->t_ms = g_tick_ms;

    if (!ctx->gimbal) return;
    hamfly_telemetry_t tel;
    hamfly_get_telemetry(ctx->gimbal, &tel);

    out->flags   = (uint8_t)(tel.baro_valid ? 0x01u : 0u);
    out->alt_dm  = (int32_t)(tel.baro_alt_m * 10.0f);     //  saturate not needed: alt range fits int32 easily
    out->roc_cms = sat_i16(tel.baro_roc_ms * 100.0f);
}

void telemetry_collect_mag(const app_ctx_t *ctx, telem_mag_t *out)
{
    memset(out, 0, sizeof *out);
    out->t_ms = g_tick_ms;

    if (!ctx->gimbal) return;
    hamfly_telemetry_t tel;
    hamfly_get_telemetry(ctx->gimbal, &tel);

    out->flags    = (uint8_t)(tel.mag_valid ? 0x01u : 0u);
    out->x_raw    = sat_i16(tel.mag_x          * 1000.0f);
    out->y_raw    = sat_i16(tel.mag_y          * 1000.0f);
    out->z_raw    = sat_i16(tel.mag_z          * 1000.0f);
    out->decl_raw = sat_i16(tel.mag_declination * 10.0f);
}

// Packet encoders on the SBC wire
#define TELEM_HOT_LEN   26u
#define TELEM_LINK_LEN  16u
#define TELEM_POWER_LEN 14u
#define TELEM_ENV_LEN   10u
#define TELEM_GPS_LEN   28u
#define TELEM_BARO_LEN  11u
#define TELEM_MAG_LEN  13u

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

static uint8_t encode_power(const telem_power_t *p, uint8_t *buf)
{
    uint8_t *q = buf;
    memcpy(q, &p->t_ms,            4); q += 4;
    memcpy(q, &p->vbat_left_mv,    2); q += 2;
    memcpy(q, &p->vbat_right_mv,   2); q += 2;
    memcpy(q, &p->sysstat_batt_mv, 2); q += 2;
    memcpy(q, &p->sysstat_batt_ma, 2); q += 2;
    *q++ = p->sysstat_cpu_pct;
    *q++ = (uint8_t)p->mcu_temp_c;
    return (uint8_t)(q - buf);  //  == TELEM_POWER_LEN
}

static uint8_t encode_env(const telem_env_t *e, uint8_t *buf)
{
    uint8_t *q = buf;
    memcpy(q, &e->t_ms,             4); q += 4;
    memcpy(q, &e->sysstat_temp_c10, 2); q += 2;
    memcpy(q, &e->sysstat_press_Pa, 4); q += 4;
    return (uint8_t)(q - buf);  //  == TELEM_ENV_LEN
}

static uint8_t encode_gps(const telem_gps_t *g, uint8_t *buf)
{
    uint8_t *q = buf;
    memcpy(q, &g->t_ms,    4); q += 4;
    *q++ = g->flags;
    *q++ = g->sats;
    memcpy(q, &g->raw_lat, 4); q += 4;
    memcpy(q, &g->raw_lon, 4); q += 4;
    memcpy(q, &g->raw_alt, 4); q += 4;
    memcpy(q, &g->raw_spd, 2); q += 2;
    memcpy(q, &g->raw_hdg, 2); q += 2;
    memcpy(q, &g->hacc,    2); q += 2;
    memcpy(q, &g->vacc,    2); q += 2;
    memcpy(q, &g->sacc,    2); q += 2;
    return (uint8_t)(q - buf);  //  == TELEM_GPS_LEN
}

static uint8_t encode_baro(const telem_baro_t *b, uint8_t *buf)
{
    uint8_t *q = buf;
    memcpy(q, &b->t_ms,    4); q += 4;
    *q++ = b->flags;
    memcpy(q, &b->alt_dm,  4); q += 4;
    memcpy(q, &b->roc_cms, 2); q += 2;
    return (uint8_t)(q - buf);  //  == TELEM_BARO_LEN
}

static uint8_t encode_mag(const telem_mag_t *m, uint8_t *buf)
{
    uint8_t *q = buf;
    memcpy(q, &m->t_ms,     4); q += 4;
    *q++ = m->flags;
    memcpy(q, &m->x_raw,    2); q += 2;
    memcpy(q, &m->y_raw,    2); q += 2;
    memcpy(q, &m->z_raw,    2); q += 2;
    memcpy(q, &m->decl_raw, 2); q += 2;
    return (uint8_t)(q - buf);  //  == TELEM_MAG_LEN
}

// Packet senders -- These are what the code publicly calls!
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

void telemetry_send_power_sbc(const app_ctx_t *ctx)
{
    telem_power_t p;
    telemetry_collect_power(ctx, &p);
    uint8_t buf[TELEM_POWER_LEN];
    uint8_t n = encode_power(&p, buf);
    sbc_send_frame(PKT_TELEM_POWER, buf, n);
}

void telemetry_send_env_sbc(const app_ctx_t *ctx)
{
    telem_env_t e;
    telemetry_collect_env(ctx, &e);
    uint8_t buf[TELEM_ENV_LEN];
    uint8_t n = encode_env(&e, buf);
    sbc_send_frame(PKT_TELEM_ENV, buf, n);
}

void telemetry_send_gps_sbc(const app_ctx_t *ctx)
{
    telem_gps_t g;
    telemetry_collect_gps(ctx, &g);
    uint8_t buf[TELEM_GPS_LEN];
    uint8_t n = encode_gps(&g, buf);
    sbc_send_frame(PKT_TELEM_GPS, buf, n);
}

void telemetry_send_baro_sbc(const app_ctx_t *ctx)
{
    telem_baro_t b;
    telemetry_collect_baro(ctx, &b);
    uint8_t buf[TELEM_BARO_LEN];
    uint8_t n = encode_baro(&b, buf);
    sbc_send_frame(PKT_TELEM_BARO, buf, n);
}

void telemetry_send_mag_sbc(const app_ctx_t *ctx)
{
    telem_mag_t m;
    telemetry_collect_mag(ctx, &m);
    uint8_t buf[TELEM_MAG_LEN];
    uint8_t n = encode_mag(&m, buf);
    sbc_send_frame(PKT_TELEM_MAG, buf, n);
}

// Gimbal sensor attribultes and request scheduling
typedef struct {
    uint16_t attr_id;           // HAMFLY_ATTR_SYSSTAT, etc.
    uint32_t period_ms;         // Should all be uniform
    uint32_t last_request_ms;   // Tibreaker
} attr_sched_t;

static attr_sched_t s_attrs[] = {
    { HAMFLY_ATTR_SYSSTAT, 5000u, 0u },  //  POWER + ENV
    { HAMFLY_ATTR_GPS,     5000u, 0u },  //  GPS
    { HAMFLY_ATTR_BARO,    5000u, 0u },  //  BARO
    { HAMFLY_ATTR_MAG,     5000u, 0u },  //  MAG
    //  ... examples (magnetometer! IMU? etc).
};
#define N_ATTRS (sizeof s_attrs / sizeof s_attrs[0])

void telemetry_pump(app_ctx_t *ctx)
{
    // Do a HamflyAttr request if on schedule and no control in process.
    // Early check if gimbal exists, and there are attrs to grab
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
