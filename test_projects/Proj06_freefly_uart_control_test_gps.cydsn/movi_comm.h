/* movi_comm.h
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
    
#ifndef MOVI_COMM_H
#define MOVI_COMM_H
#include <stdint.h>
#include <stdbool.h>
#ifndef MOVI_MAX_FRAME
#define MOVI_MAX_FRAME 96u
#endif

typedef enum {
    MOVI_OK = 0,
    MOVI_ERR_UART,
    MOVI_ERR_ENCODE,
} movi_result_t;

typedef struct {
    void* ctx;
    void (*uart_putc)(void* ctx, uint8_t b);
} movi_hal_t;

#include "FreeflyAPI.h"  /* use RATE / ABSOLUTE / DEFER */

typedef struct {
    float pan;
    float tilt;
    float roll;
    ff_api_control_type_e pan_mode;
    ff_api_control_type_e tilt_mode;
    ff_api_control_type_e roll_mode;
    uint8_t enable;
    uint8_t kill;
} movi_control_t;

typedef struct {
    /* ---- Attr 287 — automatic GCU status push ---- */
    bool    valid;
    float   battery_left_v;
    float   battery_right_v;
    /* Gimbal attitude quaternion (i=Roll, j=Tilt, k=Pan) */
    float   gimbal_r;
    float   gimbal_i;
    float   gimbal_j;
    float   gimbal_k;
    uint8_t gimbal_status1;
    uint8_t gimbal_status2;

    /* ---- Attr 4 — GPS ---- */
    bool    gps_valid;
    float   gps_lat_deg;
    float   gps_lon_deg;
    float   gps_alt_m;
    float   gps_ground_spd_ms;
    float   gps_heading_deg;
    float   gps_hacc_m;
    float   gps_vacc_m;
    float   gps_sacc_ms;
    /* Raw int values kept separately — float loses precision at 1e7 scale */
    int32_t gps_raw_lat;
    int32_t gps_raw_lon;
    int32_t gps_raw_alt;
    int16_t gps_raw_spd;
    int16_t gps_raw_hdg;
    int16_t gps_raw_hacc;
    int16_t gps_raw_vacc;
    int16_t gps_raw_sacc;

    /* ---- Attr 3 — Barometric altitude ---- */
    bool    baro_valid;
    float   baro_alt_m;
    float   baro_roc_ms;

    /* ---- Attr 22 — Gimbal attitude (euler, direct from GCU) ---- *
     * plen=19, all int16 LE                                         *
     * Note: attr 287 gives quaternion; attr 22 gives euler angles.  *
     * These are separate — attr 22 must be requested.               */
    bool    att_valid;
    float   att_roll_deg;        /* off  1  scale 100 */
    float   att_pitch_deg;       /* off  3  scale 100 */
    float   att_yaw_deg;         /* off  5  scale 100 */
    float   att_roll_err_deg;    /* off  7  scale 10  */
    float   att_pitch_err_deg;   /* off  9  scale 10  */
    float   att_yaw_err_deg;     /* off 11  scale 10  */
    float   att_roll_rate_dps;   /* off 13  scale 10  */
    float   att_pitch_rate_dps;  /* off 15  scale 10  */
    float   att_yaw_rate_dps;    /* off 17  scale 10  */

    /* ---- Attr 1 — System status ---- *
     * plen=23                          */
    bool     sysstat_valid;
    float    sysstat_batt_v;         /* off  1  int16  scale 100  V   */
    uint8_t  sysstat_gps_sats;       /* off  3  uint8  scale 1        */
    float    sysstat_temp_c;         /* off  4  uint8  scale 3   °C   */
    uint8_t  sysstat_cpu_pct;        /* off  5  uint8  scale 1   %    */
    int16_t  sysstat_status_flags;   /* off  6  int16  scale 1        */
    int16_t  sysstat_time_s;         /* off  8  int16  scale 1   s    */
    float    sysstat_pressure_mb;    /* off 10  int16  scale 10  mb   */
    int16_t  sysstat_imu_rate;       /* off 12  int16  scale 1        */
    int16_t  sysstat_cam_status;     /* off 14  int16  scale 1        */
    int16_t  sysstat_imu_latency;    /* off 17  int16  scale 1        */
    float    sysstat_batt_a;         /* off 19  int16  scale 100  A   */
    int16_t  sysstat_charge_mah;     /* off 21  int16  scale 1   mAh  */

    /* ---- Attr 12 — Magnetometer ---- *
     * plen=19, all int16 LE             */
    bool    mag_valid;
    float   mag_x;              /* off  1  scale 1000 */
    float   mag_y;              /* off  3  scale 1000 */
    float   mag_z;              /* off  5  scale 1000 */
    float   mag_magnitude;      /* off  7  scale 100  */
    float   mag_freq;           /* off  9  scale 100  */
    float   mag_off_x;          /* off 11  scale 1000 */
    float   mag_off_y;          /* off 13  scale 1000 */
    float   mag_off_z;          /* off 15  scale 1000 */
    float   mag_declination;    /* off 17  scale 10   */
} movi_status_t;

typedef struct {
    uint32_t tx_packets;
    uint32_t tx_bytes;
    uint32_t rx_packets;
    uint32_t rx_bad_checksum;
    uint32_t rx_bytes;
    uint16_t rb_drops;
    uint8_t  uart_err_flags;
} movi_statistics_t;

#include "movi_rb.h"

typedef struct {
    movi_hal_t        hal;
    movi_control_t    ctl;
    movi_status_t     status;
    movi_statistics_t statistics;
    movi_rb_t         rb;

    /* Test mode — one pending request at a time.                      *
     * movi_comm_request_attr() arms these fields.                     *
     * pump() sets test_response_ready when the pending attr arrives.  *
     * main() checks test_response_ready, prints verbose, clears it.   */
    uint16_t test_pending_attr;   /* 0 = no request outstanding        */
    uint32_t test_sent_ms;        /* g_tick_ms at time of send         */
    bool     test_response_ready; /* set by pump(), cleared by main()  */
    uint16_t test_response_attr;  /* which attr just responded         */
    uint16_t test_rx_len;         /* MsgLength of the response packet  */

    /* Snapshot of raw payload bytes taken inside pump() at the instant *
     * the response is detected — before the ring buffer drains further *
     * and QX_StreamRxCharSM overwrites RxMsg with the next packet.     *
     * Size 32 covers the largest test attr (attr 1, plen=23).          */
    uint8_t  test_rx_payload[32];
    uint8_t  test_rx_payload_len; /* bytes actually copied             */
} movi_comm_t;

void          movi_comm_init(movi_comm_t* m, const movi_hal_t* hal);
void          movi_comm_reset(movi_comm_t* m);
void          movi_comm_clear_statistics(movi_comm_t* m);
void          movi_comm_on_rx_byte(movi_comm_t* m, uint8_t b);
void          movi_comm_on_uart_error_flags(movi_comm_t* m, uint8_t err_mask);
void          movi_comm_pump(movi_comm_t* m);
void          movi_comm_set_control(movi_comm_t* m, const movi_control_t* ctl);
void          movi_comm_get_control(movi_comm_t* m, movi_control_t* out);
void          movi_comm_kill(movi_comm_t* m);
movi_result_t movi_comm_build_control_frame(movi_comm_t* m,
                  uint8_t* out, uint16_t out_max, uint16_t* out_len);
movi_result_t movi_comm_send_control(movi_comm_t* m);

/* Send a one-shot QX read request for any attribute ID.
 * Also arms test_pending_attr / test_sent_ms so pump() can detect the
 * response.  A new call replaces any previous pending request.
 * Safe to call from main loop. NOT ISR-safe.                         */
movi_result_t movi_comm_request_attr(movi_comm_t* m, uint16_t attr_id);

void          movi_comm_get_status(movi_comm_t* m, movi_status_t* out);
void          movi_comm_get_statistics(movi_comm_t* m, movi_statistics_t* out);

#endif /* MOVI_COMM_H */