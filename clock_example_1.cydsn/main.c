/*******************************************************************************
* File Name: main.c
*
* Version: 1.0
*
* Description:
*  This is the source code for the Clock example project of the cy_boot
*  component.
*
********************************************************************************
* Copyright 2015, Cypress Semiconductor Corporation. All rights reserved.
* This software is owned by Cypress Semiconductor Corporation and is protected
* by and subject to worldwide patent and copyright laws and treaties.
* Therefore, you may use this software only as provided in the license agreement
* accompanying the software package from which you obtained this software.
* CYPRESS AND ITS SUPPLIERS MAKE NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* WITH REGARD TO THIS SOFTWARE, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT,
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
*******************************************************************************/

#include <project.h>

#define CLK_IMO_MHZ                 (32u)
#define SYSCLK_DIVIDER              (2u)

#define HZ_IN_MHZ                   (1000000u)
#define BLINK_DELAY_MS              (1000u)
#define VISIBILITY_DELAY_MS         (1000u)

#define WAIT_IMO_TO_SETTLE          (1u)
#define WAIT_PLL_TO_SETTLE          (1u)
#define IMO_FREQ                    (CY_IMO_FREQ_3MHZ) /* CyIMO_SetFreq() parameter */


#define LED_OFF                     (0u)
#define LED_ON                      (1u)

#define PLL_Q                       (1u)    /* 1 MHz <= (Fin / Q) <= 3 MHz */
#define PLL_P                       (16u)   /* Fout = Fin * P/Q */
#define PLL_ICP                     (1u)    /* Valid for all configurations */

/* Conversion between CyIMO_SetFreq() parameter and frequency in 3-62 MHz range */
const uint8 CYCODE imoFreqParam2Frequency[6u] = {3u, 6u, 12u, 24u, 48, 62u};

/* Function prototype */
void Led_Blink(void);


/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Summary:
*  Reconfigures the clock tree from "XTAL -> Master Clock -> Bus Clock" to "IMO
*  -> PLL -> Master Clock -> Bus Clock". Reconfigures ILO 1 KHz to route it out
*  of the ILO block instead of ILO 100 KHz. Disables the unused clock source:
*  XTAL and ILO 100 KHz. The LCD displays the execution status. The LED blinks
*  twice for the same period of time before clock configuration changes and
*  after.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
int main(void)
{
    LCD_Start();
    LCD_Position(0u, 0u);
    LCD_PrintString("Clock Example");

    Led_Blink();

    /* Configure and start IMO */
    CyIMO_SetFreq(IMO_FREQ);

    /* PLLout = Fimo * P /Q */
    CyPLL_OUT_SetPQ(PLL_P, PLL_Q, PLL_ICP);
    CyIMO_Start(WAIT_IMO_TO_SETTLE);

    /* Configure and start PLL */
    CyPLL_OUT_SetSource(CY_PLL_SOURCE_IMO);
    if(CYRET_SUCCESS == CyPLL_OUT_Start(WAIT_PLL_TO_SETTLE))
    {
        /***********************************************************************
        * Set the number of clock cycles the cache waits before it samples
        * data coming back from the Flash. This function must be called before
        * increasing the CPU clock frequency. It can optionally be called after
        * lowering the CPU clock frequency to improve the CPU performance.
        ***********************************************************************/
        CyFlash_SetWaitCycles(imoFreqParam2Frequency[IMO_FREQ] * PLL_P / PLL_Q);


        /***********************************************************************
        * Adjustments to keep the CyDelay function accurate.
        * By default, the number of cycles to delay in CyDelay() and CyDelayUs()
        * is calculated based on the clock configuration entered in PSoC Creator.
        * If the clock configuration changes at the run-time, the CyDelayFreq()
        * function is used to indicate a new frequency.
        ***********************************************************************/
        CyDelayFreq(HZ_IN_MHZ * imoFreqParam2Frequency[IMO_FREQ] * PLL_P / PLL_Q);

        /* Change Master clock source to PLL */
        CyMasterClk_SetSource(CY_MASTER_SOURCE_PLL);

        /* Master clock is not sourced by XTAL - safe to stop it */
        CyXTAL_Stop();
    }
    else
    {
        LCD_Position(1u, 0u);
        LCD_PrintString("PLL - Failed.");
        CyDelay(VISIBILITY_DELAY_MS);
    }

    /* ILO 1 KHz is enabled by default, so route ILO 1 KHz and disable 100 KHz */
    CyILO_SetSource(CY_ILO_SOURCE_1K);
    CyILO_Stop100K();

    LCD_Position(1u, 0u);
    LCD_PrintString("Reconfigured");

    Led_Blink();

    for(;;)
    {
    }

}


/*******************************************************************************
* Function Name: Led_Blink
********************************************************************************
* Summary:
*  Blinks LED with the LED_Status pin.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void Led_Blink(void)
{
    LED_Status_Write(LED_ON);
    CyDelay(BLINK_DELAY_MS);
    LED_Status_Write(LED_OFF);
    CyDelay(BLINK_DELAY_MS);
}


/* [] END OF FILE */
