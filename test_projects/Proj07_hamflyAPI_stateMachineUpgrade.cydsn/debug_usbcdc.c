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
 * Shim to map UART_DEBUG onto the USBFS.
*/

#include "debug_usbcdc.h"
#include <string.h>

#define USBFS_DEVICE   0u
#define USB_TX_CHUNK   64u  // Max input packet for USB CDC.

static uint8_t s_ready = 0u;

void dbg_init(void)
{
    USBFS_DEBUG_Start(USBFS_DEVICE, USBFS_DEBUG_3V_OPERATION);
}

void dbg_tick(void)
{
    // Re initialize CDC after re-enumeration
    if (USBFS_DEBUG_IsConfigurationChanged() != 0u) {
        if (USBFS_DEBUG_GetConfiguration() != 0u) {
            USBFS_DEBUG_CDC_Init();
            s_ready = 1u;
        } else {
            s_ready = 0u;
        }
    }
}

void dbg_put(const char *s)
{
    if (!s_ready || !s) return;

    size_t total = strlen(s);
    size_t off   = 0u;

    while (off < total) {
        // Drop on timeout so we do not block main loop if host unplugs.
        uint32_t spin = 0u;
        while (USBFS_DEBUG_CDCIsReady() == 0u) {
            if (++spin > 200000u) return;
        }
        uint16_t chunk = (uint16_t)((total - off) > USB_TX_CHUNK
                                    ? USB_TX_CHUNK
                                    : (total - off));
        USBFS_DEBUG_PutData((const uint8_t *)(s + off), chunk);
        off += chunk;
    }
}

uint8_t dbg_get(void)
{
    if (!s_ready) return 0u;
    if (USBFS_DEBUG_DataIsReady() == 0u) return 0u;
    uint8_t b = 0u;
    (void)USBFS_DEBUG_GetData(&b, 1u);
    return b;
}