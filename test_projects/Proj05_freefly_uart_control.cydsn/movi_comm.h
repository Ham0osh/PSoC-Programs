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

#include "FreeflyAPI.h"  // use RATE / ABSOLUTE / DEFER

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
    bool  valid;
    float battery_left_v;
    float battery_right_v;
    // Gimbal attitude quaternion (i=Roll, j=Tilt, k=Pan)
    float gimbal_r;
    float gimbal_i;  // Roll
    float gimbal_j;  // Tilt
    float gimbal_k;  // Pan
    // Gimbal status flags
    uint8_t gimbal_status1;
    uint8_t gimbal_status2;
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
    movi_hal_t hal;

    movi_control_t    ctl;
    movi_status_t     status;
    movi_statistics_t statistics;

    movi_rb_t rb;
} movi_comm_t;

void movi_comm_init(movi_comm_t* m, const movi_hal_t* hal);
void movi_comm_reset(movi_comm_t* m);
void movi_comm_clear_statistics(movi_comm_t* m);

void movi_comm_on_rx_byte(movi_comm_t* m, uint8_t b);
void movi_comm_on_uart_error_flags(movi_comm_t* m, uint8_t err_mask);

void movi_comm_pump(movi_comm_t* m);

void movi_comm_set_control(movi_comm_t* m, const movi_control_t* ctl);
void movi_comm_get_control(movi_comm_t* m, movi_control_t* out);

void movi_comm_kill(movi_comm_t* m);

/* Not supported with the Freefly/QX “queue then drain” TX model */
movi_result_t movi_comm_build_control_frame(movi_comm_t* m,
                                           uint8_t* out, uint16_t out_max, uint16_t* out_len);

movi_result_t movi_comm_send_control(movi_comm_t* m);

void movi_comm_get_status(movi_comm_t* m, movi_status_t* out);
void movi_comm_get_statistics(movi_comm_t* m, movi_statistics_t* out);

#endif /* MOVI_COMM_H */
