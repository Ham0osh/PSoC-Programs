/* hamfly_psoc5.h
 * ============================================================
 * Copyright Hamish Johnson, 2026
 * Quantum Internet Systems Lab, SFU Physics
 * ============================================================
 *
 * PSoC 5 LP concrete implementation of hamfly_hal_t.
 * Only file in HamflyAPI that includes <project.h>.
 *
 * Usage in main.c:
 *
 *   static hamfly_gimbal_t g_gimbal;
 *
 *   static const hamfly_hal_t HAL = hamfly_psoc5_hal(NULL);
 *
 *   // In UART ISR:
 *   CY_ISR(isr_rx_movi_Handler) {
 *       hamfly_on_rx_byte(&g_gimbal, UART_MOVI_RXDATA_REG);
 *   }
 *
 *   // In main:
 *   hamfly_init(&g_gimbal, &HAL);
 */

#ifndef HAMFLY_PSOC5_H
#define HAMFLY_PSOC5_H

#include "hamfly_core_hal.h"
#include "project.h"

static void _hamfly_psoc5_putc(void *ctx, uint8_t b)
{
    (void)ctx;
    UART_MOVI_PutChar((char)b);
}

/*
 * Returns a ready-to-use hamfly_hal_t for PSoC 5 UART_MOVI.
 * Pass ctx=NULL unless you need to distinguish multiple UARTs.
 */
static inline hamfly_hal_t hamfly_psoc5_hal(void *ctx)
{
    hamfly_hal_t h;
    h.ctx       = ctx;
    h.uart_putc = _hamfly_psoc5_putc;
    return h;
}

#endif /* HAMFLY_PSOC5_H */
