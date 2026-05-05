/*******************************************************************************
* File Name: SPIMaster_SDFlash_PM.c
* Version 2.50
*
* Description:
*  This file contains the setup, control and status commands to support
*  component operations in low power mode.
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "SPIMaster_SDFlash_PVT.h"

static SPIMaster_SDFlash_BACKUP_STRUCT SPIMaster_SDFlash_backup =
{
    SPIMaster_SDFlash_DISABLED,
    SPIMaster_SDFlash_BITCTR_INIT,
};


/*******************************************************************************
* Function Name: SPIMaster_SDFlash_SaveConfig
********************************************************************************
*
* Summary:
*  Empty function. Included for consistency with other components.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void SPIMaster_SDFlash_SaveConfig(void) 
{

}


/*******************************************************************************
* Function Name: SPIMaster_SDFlash_RestoreConfig
********************************************************************************
*
* Summary:
*  Empty function. Included for consistency with other components.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void SPIMaster_SDFlash_RestoreConfig(void) 
{

}


/*******************************************************************************
* Function Name: SPIMaster_SDFlash_Sleep
********************************************************************************
*
* Summary:
*  Prepare SPIM Component goes to sleep.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  SPIMaster_SDFlash_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void SPIMaster_SDFlash_Sleep(void) 
{
    /* Save components enable state */
    SPIMaster_SDFlash_backup.enableState = ((uint8) SPIMaster_SDFlash_IS_ENABLED);

    SPIMaster_SDFlash_Stop();
}


/*******************************************************************************
* Function Name: SPIMaster_SDFlash_Wakeup
********************************************************************************
*
* Summary:
*  Prepare SPIM Component to wake up.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  SPIMaster_SDFlash_backup - used when non-retention registers are restored.
*  SPIMaster_SDFlash_txBufferWrite - modified every function call - resets to
*  zero.
*  SPIMaster_SDFlash_txBufferRead - modified every function call - resets to
*  zero.
*  SPIMaster_SDFlash_rxBufferWrite - modified every function call - resets to
*  zero.
*  SPIMaster_SDFlash_rxBufferRead - modified every function call - resets to
*  zero.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void SPIMaster_SDFlash_Wakeup(void) 
{
    #if(SPIMaster_SDFlash_RX_SOFTWARE_BUF_ENABLED)
        SPIMaster_SDFlash_rxBufferFull  = 0u;
        SPIMaster_SDFlash_rxBufferRead  = 0u;
        SPIMaster_SDFlash_rxBufferWrite = 0u;
    #endif /* (SPIMaster_SDFlash_RX_SOFTWARE_BUF_ENABLED) */

    #if(SPIMaster_SDFlash_TX_SOFTWARE_BUF_ENABLED)
        SPIMaster_SDFlash_txBufferFull  = 0u;
        SPIMaster_SDFlash_txBufferRead  = 0u;
        SPIMaster_SDFlash_txBufferWrite = 0u;
    #endif /* (SPIMaster_SDFlash_TX_SOFTWARE_BUF_ENABLED) */

    /* Clear any data from the RX and TX FIFO */
    SPIMaster_SDFlash_ClearFIFO();

    /* Restore components block enable state */
    if(0u != SPIMaster_SDFlash_backup.enableState)
    {
        SPIMaster_SDFlash_Enable();
    }
}


/* [] END OF FILE */
