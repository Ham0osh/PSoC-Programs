/* ========================================
 *
 * Copyright Hamish Johnson, 2025
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"
#include "Timer.h"
#include <cytypes.h>
#include <CyLib.h>

// Constants
uint32_t blink_period_ms = 500;  // default 2 Hz

// Timer Interrupt Service Routine
CY_ISR( Timer_Int_Handler )
{
    uint16 counter = Timer_ReadCapture();
    if(counter > 10000)
    {
        counter = 10000;
    }
    blink_period_ms = counter;
    
    //Timer_ClearInterrupt();
}

int main(void)
{
    CyGlobalIntEnable;  // enable global interrupts

    // Turn LED on by default
    Pin_LED_Write(1);
    Timer_Int_StartEx( Timer_Int_Handler );
    
    for(;;)
    {
        // Button active low: 0 when pressed
        if(Pin_SW1_Read() == 0)
        {
            Pin_LED_Write(1);  // turn LED off while held
        }
        else
        {
        }
        Pin_LED_Write(!Pin_LED_Read());
        CyDelay(blink_period_ms / 2);
    }
}



/* [] END OF FILE */
