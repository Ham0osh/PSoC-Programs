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
#include "project.h"

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    UART_Start();
    int rate_slow = 1;
    int rate_fast = 10;
    int active = 1;
    UART_PutString("Initialized!\r\n");

    for(;;)
    {
        // Depending on state machine, change requency of updates
        uint8 state_reg = state_register_Read() & 0x07;
        
        if(state_reg==0x01){
            if(active){
                UART_PutString("Entering standby...\r\n");
                active = 0;
            }
          }
        if(state_reg==0x02){
            active = 1;
            UART_PutString("Slow mode (:\r\n");
            CyDelay(1000 / rate_slow);
        }
        if(state_reg==0x04){
            active = 1;
            UART_PutString("Fast mode :O\r\n");
            CyDelay(1000 / rate_fast);
       }
    }
}

/* [] END OF FILE */
