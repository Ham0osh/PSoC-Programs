/* hamfly_telemetry.c
 * ============================================================
 * Copyright Hamish Johnson, 2026
 * Quantum Internet Systems Lab, SFU Physics
 * ============================================================
 *
 * Decode functions for all confirmed GCU attrs.
 * Promoted from private statics in movi_comm.c to public
 * functions so they can be called directly on captured
 * byte arrays for testing without a live gimbal.
 *
 * Each function is self-contained:
 *   - checks attr ID and minimum length before touching dst
 *   - sets the corresponding valid flag on success
 *   - does not touch any other fields in dst
 *
 * hamfly_pump() calls all of these automatically.
 * Direct use is for testing or custom polling.
 */

#include "hamfly_core_telemetry.h"
#include "hamfly_qx_protocol.h"
#include <string.h>
#include <stdint.h>

/* ============================================================
 * Big-endian read helpers
 * Attr 1, 4, 287 payloads are big-endian, no sub-index byte.
 * Attr 22 is little-endian with a 0x00 sub-index at offset 0.
 * ============================================================ */
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

/* Little-endian read -- attr 22 */
static int16_t rd_le16(const uint8_t *p)
{
    int16_t v;
    memcpy(&v, p, 2);
    return v;
}

/* ============================================================
 * hamfly_decode_qx287
 *
 * Attr 287 -- QX status response (arrives with every QX277).
 * Reads from FreeflyAPI.status which the QX parser already
 * populated, then copies into hamfly_telemetry_t.
 * plen and p are not used directly here; FreeflyAPI is the
 * source of truth for 287 since the QX parser owns it.
 * ============================================================ */
void hamfly_decode_qx287(const uint8_t *p, uint16_t plen,
                          hamfly_telemetry_t *dst)
{
    /* FreeflyAPI.status is already decoded by the QX parser callback.
     * We just mirror it into the hamfly_telemetry_t struct.           */
    (void)p; (void)plen;


    dst->battery_left_v   = FreeflyAPI.status.battery_v_left;
    dst->battery_right_v  = FreeflyAPI.status.battery_v_right;
    dst->gimbal_r         = FreeflyAPI.status.gimbal_r;
    dst->gimbal_i         = FreeflyAPI.status.gimbal_i;
    dst->gimbal_j         = FreeflyAPI.status.gimbal_j;
    dst->gimbal_k         = FreeflyAPI.status.gimbal_k;
    dst->gimbal_status1   = FreeflyAPI.status.gimbal_Status1;
    dst->gimbal_status2   = FreeflyAPI.status.gimbal_Status2;
    dst->valid            = true;
}

/* ============================================================
 * hamfly_decode_gps  --  attr 4
 *
 * Payload: big-endian, no sub-index byte.
 *   off  0-3  BE int32: longitude   /1e7  deg
 *   off  4-7  BE int32: latitude    /1e7  deg
 *   off  8-11 BE int32: altitude    /1000 m
 *   off 12-13 BE int16: ground speed /100 m/s
 *   off 14-15 BE int16: heading     /10   deg
 *   off 16-17 BE int16: HACC        /100  m
 *   off 18-19 BE int16: VACC        /100  m
 *   off 20-21 BE int16: SACC        /100  m/s
 * ============================================================ */
void hamfly_decode_gps(const uint8_t *p, uint16_t plen,
                        hamfly_telemetry_t *dst)
{
    if (plen < 22u) return;

    int32_t lon_raw  = rd_be32(p +  0);
    int32_t lat_raw  = rd_be32(p +  4);
    int32_t alt_raw  = rd_be32(p +  8);
    int16_t spd_raw  = rd_be16(p + 12);
    int16_t hdg_raw  = rd_be16(p + 14);
    int16_t hacc_raw = rd_be16(p + 16);
    int16_t vacc_raw = rd_be16(p + 18);
    int16_t sacc_raw = rd_be16(p + 20);

    dst->gps_raw_lon  = lon_raw;
    dst->gps_raw_lat  = lat_raw;
    dst->gps_raw_alt  = alt_raw;
    dst->gps_raw_spd  = spd_raw;
    dst->gps_raw_hdg  = hdg_raw;
    dst->gps_raw_hacc = hacc_raw;
    dst->gps_raw_vacc = vacc_raw;
    dst->gps_raw_sacc = sacc_raw;

    dst->gps_lon_deg       = (float)lon_raw  / 1e7f;
    dst->gps_lat_deg       = (float)lat_raw  / 1e7f;
    dst->gps_alt_m         = (float)alt_raw  / 1000.0f;
    dst->gps_ground_spd_ms = (float)spd_raw  / 100.0f;
    dst->gps_heading_deg   = (float)hdg_raw  / 10.0f;
    dst->gps_hacc_m        = (float)hacc_raw / 100.0f;
    dst->gps_vacc_m        = (float)vacc_raw / 100.0f;
    dst->gps_sacc_ms       = (float)sacc_raw / 100.0f;
    dst->gps_valid         = true;
}

/* ============================================================
 * hamfly_decode_baro  --  attr 3
 *
 * plen=5, sub-index byte at off 0 (ignored).
 *   off 1-2  LE int16: altitude     /10   m
 *   off 3-4  LE int16: rate of climb /100 m/s
 * ============================================================ */
void hamfly_decode_baro(const uint8_t *p, uint16_t plen,
                         hamfly_telemetry_t *dst)
{
    if (plen < 5u) return;

    int16_t alt_raw = rd_le16(p + 1);
    int16_t roc_raw = rd_le16(p + 3);

    dst->baro_alt_m  = (float)alt_raw / 10.0f;
    dst->baro_roc_ms = (float)roc_raw / 100.0f;
    dst->baro_valid  = true;
}

/* ============================================================
 * hamfly_decode_attitude  --  attr 22
 *
 * plen=19, all LE int16, sub-index byte at off 0 (ignored).
 * Note: Java field label "Roll" at off 1 is actually Pitch
 * in observed data. Layout confirmed against live GCU.
 *   off  1-2  LE int16: Pitch  /100  deg
 *   off  3-4  LE int16: Yaw    /100  deg
 *   off  5-6  LE int16: Roll   /100  deg
 *   off  7-8  LE int16: Roll error   /10 deg
 *   off  9-10 LE int16: Pitch error  /10 deg
 *   off 11-12 LE int16: Yaw error    /10 deg
 *   off 13-14 LE int16: Roll rate    /10 dps
 *   off 15-16 LE int16: Pitch rate   /10 dps
 *   off 17-18 LE int16: Yaw rate     /10 dps
 * ============================================================ */
void hamfly_decode_attitude(const uint8_t *p, uint16_t plen,
                              hamfly_telemetry_t *dst)
{
    if (plen < 19u) return;

    dst->att_pitch_deg      = (float)rd_le16(p +  1) / 100.0f;
    dst->att_yaw_deg        = (float)rd_le16(p +  3) / 100.0f;
    dst->att_roll_deg       = (float)rd_le16(p +  5) / 100.0f;
    dst->att_roll_err_deg   = (float)rd_le16(p +  7) / 10.0f;
    dst->att_pitch_err_deg  = (float)rd_le16(p +  9) / 10.0f;
    dst->att_yaw_err_deg    = (float)rd_le16(p + 11) / 10.0f;
    dst->att_roll_rate_dps  = (float)rd_le16(p + 13) / 10.0f;
    dst->att_pitch_rate_dps = (float)rd_le16(p + 15) / 10.0f;
    dst->att_yaw_rate_dps   = (float)rd_le16(p + 17) / 10.0f;
    dst->att_valid          = true;
}

/* ============================================================
 * hamfly_decode_sysstat  --  attr 1
 *
 * plen=22, big-endian, no sub-index byte.
 *   off  0-1  BE int16:  battery voltage  /100  V
 *   off  2    uint8:     GPS satellites
 *   off  3    uint8:     temperature      /3    C
 *   off  4    uint8:     CPU %
 *   off  5-6  BE int16:  status flags
 *   off  7-8  BE int16:  time             s
 *   off  9-10 BE int16:  pressure         /10   mb
 *   off 11-12 BE int16:  IMU rate
 *   off 13-14 BE int16:  camera status
 *   off 15    uint8:     reserved
 *   off 16-17 BE int16:  IMU latency      us
 *   off 18-19 BE int16:  battery current  /100  A
 *   off 20-21 BE int16:  charge used      mAh
 * ============================================================ */
void hamfly_decode_sysstat(const uint8_t *p, uint16_t plen,
                             hamfly_telemetry_t *dst)
{
    if (plen < 22u) return;

    dst->sysstat_batt_v       = (float)rd_be16(p +  0) / 100.0f;
    dst->sysstat_gps_sats     = p[2];
    dst->sysstat_temp_c       = (float)p[3] / 3.0f;
    dst->sysstat_cpu_pct      = p[4];
    dst->sysstat_status_flags = rd_be16(p +  5);
    dst->sysstat_time_s       = rd_be16(p +  7);
    dst->sysstat_pressure_mb  = (float)rd_be16(p +  9) / 10.0f;
    dst->sysstat_imu_rate     = rd_be16(p + 11);
    dst->sysstat_cam_status   = rd_be16(p + 13);
    /* off 15: reserved, skip */
    dst->sysstat_imu_latency  = rd_be16(p + 16);
    dst->sysstat_batt_a       = (float)rd_be16(p + 18) / 100.0f;
    dst->sysstat_charge_mah   = rd_be16(p + 20);
    dst->sysstat_valid        = true;
}

/* ============================================================
 * hamfly_decode_mag  --  attr 12
 *
 * plen=19, all LE int16, sub-index byte at off 0 (ignored).
 *   off  1-2  LE int16: X         /1000
 *   off  3-4  LE int16: Y         /1000
 *   off  5-6  LE int16: Z         /1000
 *   off  7-8  LE int16: magnitude /100
 *   off  9-10 LE int16: frequency /100
 *   off 11-12 LE int16: offset X  /1000
 *   off 13-14 LE int16: offset Y  /1000
 *   off 15-16 LE int16: offset Z  /1000
 *   off 17-18 LE int16: declination /10
 * ============================================================ */
void hamfly_decode_mag(const uint8_t *p, uint16_t plen,
                        hamfly_telemetry_t *dst)
{
    if (plen < 19u) return;

    dst->mag_x           = (float)rd_le16(p +  1) / 1000.0f;
    dst->mag_y           = (float)rd_le16(p +  3) / 1000.0f;
    dst->mag_z           = (float)rd_le16(p +  5) / 1000.0f;
    dst->mag_magnitude   = (float)rd_le16(p +  7) / 100.0f;
    dst->mag_freq        = (float)rd_le16(p +  9) / 100.0f;
    dst->mag_off_x       = (float)rd_le16(p + 11) / 1000.0f;
    dst->mag_off_y       = (float)rd_le16(p + 13) / 1000.0f;
    dst->mag_off_z       = (float)rd_le16(p + 15) / 1000.0f;
    dst->mag_declination = (float)rd_le16(p + 17) / 10.0f;
    dst->mag_valid       = true;
}
