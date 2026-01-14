/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
/* movi_qx.h */
#ifndef MOVI_QX_H
#define MOVI_QX_H

#include <project.h>

/* MOVI UART loopback / bring-up API (Milestone 1) */
void movi_qx_init(void);

/* Enter/exit the "u-mode" UART loopback state */
void movi_qx_set_uart_loopback_mode(uint8 enable);
uint8 movi_qx_uart_loopback_mode(void);

/* Call frequently from main loop */
void movi_qx_poll_rx(void);

/* Call frequently; internally sends every 50 ms when loopback mode is enabled */
void movi_qx_tick(void);

#endif /* MOVI_QX_H */

/* [] END OF FILE */
