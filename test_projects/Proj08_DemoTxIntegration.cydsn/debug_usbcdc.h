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

#ifndef DBG_CDC_H
#define DBG_CDC_H

#include <project.h>

#ifdef __cplusplus
extern "C" {
#endif

void    dbg_init(void);          // Init after CyGlobalIntEnable
void    dbg_tick(void);          // Call each superloop
void    dbg_put(const char *s);  // Non blocking, drop if no listener host
uint8_t dbg_get(void);           // Return 0 if no byte avail.

// Aliases to legacy debug print commands.
#define UART_DEBUG_Start()        dbg_init()
#define UART_DEBUG_PutString(s)   dbg_put((s))
#define UART_DEBUG_GetChar()      dbg_get()

#ifdef __cplusplus
}
#endif

#endif /* DBG_CDC_H */