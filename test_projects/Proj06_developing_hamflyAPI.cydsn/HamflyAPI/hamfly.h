/* hamfly.h
 * ============================================================
 * Copyright Hamish Johnson, 2026
 * Quantum Internet Systems Lab, SFU Physics
 * ============================================================
 *
 * Single include for HamflyAPI users.
 *
 * Typical usage:
 *
 *   #include "hamfly.h"
 *
 *   static hamfly_gimbal_t g_gimbal;
 *   static const hamfly_hal_t HAL = hamfly_psoc5_hal(NULL);
 *
 *   hamfly_init(&g_gimbal, &HAL);
 *
 *   // ISR:
 *   hamfly_on_rx_byte(&g_gimbal, UART_MOVI_RXDATA_REG);
 *
 *   // Main loop:
 *   hamfly_pump(&g_gimbal);
 *   hamfly_send_control(&g_gimbal, &ctl);
 *
 *   // Optional attr read:
 *   hamfly_request_attr(&g_gimbal, 22u);
 *   g_gimbal.pending_sent_ms = now_ms;
 *   // ... later ...
 *   if (g_gimbal.pending_ready) { ... }
 */

#ifndef HAMFLY_H
#define HAMFLY_H

#include "hamfly_core_hal.h"
#include "hamfly_core_control.h"
#include "hamfly_core_telemetry.h"
#include "hamfly_core_gimbal.h"

#endif /* HAMFLY_H */
