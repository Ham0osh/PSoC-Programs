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
 * Parent STBY - For all standby states.
*/

#include "app_state_stby.h"
#include <project.h>  // For UART

// STBY_DEFER %===============================================================%
// On Enter: Print message, wait for user input.
void entry_stby_defer(app_ctx_t *ctx)
{
    (void)ctx;
    UART_DEBUG_PutString("\r\n[STBY_DEFER] '1' hold, '2' manual, '3' auto,"
                         " '?' help\r\n> ");
}
// On Exit: None

// STBY_HOLD %================================================================%
// On Enter
void entry_stby_hold(app_ctx_t *ctx)
{
    (void)ctx;
    UART_DEBUG_PutString("\r\n[STBY_HOLD] rates zeroed\r\n> ");
}
// On Exit: None

/* [] END OF FILE */
