/* hamfly_hal.h
 * ============================================================
 * Copyright Hamish Johnson, 2026
 * Quantum Internet Systems Lab, SFU Physics
 * ============================================================
 *
 * Hardware abstraction interface for HamflyAPI.
 * Implement uart_putc and ctx for your platform.
 * The ISR always calls hamfly_on_rx_byte() directly --
 * it is never owned by this layer.
 */

#ifndef HAMFLY_HAL_H
#define HAMFLY_HAL_H

#include <stdint.h>

typedef struct {
    void     *ctx;
    void     (*uart_putc)   (void *ctx, uint8_t b);
    uint32_t (*get_tick_ms) (void *ctx);   /* optional: return 0 if unused */
} hamfly_hal_t;

#endif /* HAMFLY_HAL_H */
