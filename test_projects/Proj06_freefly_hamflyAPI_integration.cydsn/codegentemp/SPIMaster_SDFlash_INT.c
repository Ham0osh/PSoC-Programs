/*******************************************************************************
* File Name: SPIMaster_SDFlash_INT.c
* Version 2.50
*
* Description:
*  This file provides all Interrupt Service Routine (ISR) for the SPI Master
*  component.
*
* Note:
*  None.
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "SPIMaster_SDFlash_PVT.h"
#include "cyapicallbacks.h"

/* User code required at start of ISR */
/* `#START SPIMaster_SDFlash_ISR_START_DEF` */

/* `#END` */


/*******************************************************************************
* Function Name: SPIMaster_SDFlash_TX_ISR
********************************************************************************
*
* Summary:
*  Interrupt Service Routine for TX portion of the SPI Master.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  SPIMaster_SDFlash_txBufferWrite - used for the account of the bytes which
*  have been written down in the TX software buffer.
*  SPIMaster_SDFlash_txBufferRead - used for the account of the bytes which
*  have been read from the TX software buffer, modified when exist data to
*  sending and FIFO Not Full.
*  SPIMaster_SDFlash_txBuffer[SPIMaster_SDFlash_TX_BUFFER_SIZE] - used to store
*  data to sending.
*  All described above Global variables are used when Software Buffer is used.
*
*******************************************************************************/
CY_ISR(SPIMaster_SDFlash_TX_ISR)
{
    #if(SPIMaster_SDFlash_TX_SOFTWARE_BUF_ENABLED)
        uint8 tmpStatus;
    #endif /* (SPIMaster_SDFlash_TX_SOFTWARE_BUF_ENABLED) */

    #ifdef SPIMaster_SDFlash_TX_ISR_ENTRY_CALLBACK
        SPIMaster_SDFlash_TX_ISR_EntryCallback();
    #endif /* SPIMaster_SDFlash_TX_ISR_ENTRY_CALLBACK */

    /* User code required at start of ISR */
    /* `#START SPIMaster_SDFlash_TX_ISR_START` */

    /* `#END` */
    
    #if(SPIMaster_SDFlash_TX_SOFTWARE_BUF_ENABLED)
        /* Check if TX data buffer is not empty and there is space in TX FIFO */
        while(SPIMaster_SDFlash_txBufferRead != SPIMaster_SDFlash_txBufferWrite)
        {
            tmpStatus = SPIMaster_SDFlash_GET_STATUS_TX(SPIMaster_SDFlash_swStatusTx);
            SPIMaster_SDFlash_swStatusTx = tmpStatus;

            if(0u != (SPIMaster_SDFlash_swStatusTx & SPIMaster_SDFlash_STS_TX_FIFO_NOT_FULL))
            {
                if(0u == SPIMaster_SDFlash_txBufferFull)
                {
                   SPIMaster_SDFlash_txBufferRead++;

                    if(SPIMaster_SDFlash_txBufferRead >= SPIMaster_SDFlash_TX_BUFFER_SIZE)
                    {
                        SPIMaster_SDFlash_txBufferRead = 0u;
                    }
                }
                else
                {
                    SPIMaster_SDFlash_txBufferFull = 0u;
                }

                /* Put data element into the TX FIFO */
                CY_SET_REG8(SPIMaster_SDFlash_TXDATA_PTR, 
                                             SPIMaster_SDFlash_txBuffer[SPIMaster_SDFlash_txBufferRead]);
            }
            else
            {
                break;
            }
        }

        if(SPIMaster_SDFlash_txBufferRead == SPIMaster_SDFlash_txBufferWrite)
        {
            /* TX Buffer is EMPTY: disable interrupt on TX NOT FULL */
            SPIMaster_SDFlash_TX_STATUS_MASK_REG &= ((uint8) ~SPIMaster_SDFlash_STS_TX_FIFO_NOT_FULL);
        }

    #endif /* (SPIMaster_SDFlash_TX_SOFTWARE_BUF_ENABLED) */

    /* User code required at end of ISR (Optional) */
    /* `#START SPIMaster_SDFlash_TX_ISR_END` */

    /* `#END` */
    
    #ifdef SPIMaster_SDFlash_TX_ISR_EXIT_CALLBACK
        SPIMaster_SDFlash_TX_ISR_ExitCallback();
    #endif /* SPIMaster_SDFlash_TX_ISR_EXIT_CALLBACK */
}


/*******************************************************************************
* Function Name: SPIMaster_SDFlash_RX_ISR
********************************************************************************
*
* Summary:
*  Interrupt Service Routine for RX portion of the SPI Master.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  SPIMaster_SDFlash_rxBufferWrite - used for the account of the bytes which
*  have been written down in the RX software buffer modified when FIFO contains
*  new data.
*  SPIMaster_SDFlash_rxBufferRead - used for the account of the bytes which
*  have been read from the RX software buffer, modified when overflow occurred.
*  SPIMaster_SDFlash_rxBuffer[SPIMaster_SDFlash_RX_BUFFER_SIZE] - used to store
*  received data, modified when FIFO contains new data.
*  All described above Global variables are used when Software Buffer is used.
*
*******************************************************************************/
CY_ISR(SPIMaster_SDFlash_RX_ISR)
{
    #if(SPIMaster_SDFlash_RX_SOFTWARE_BUF_ENABLED)
        uint8 tmpStatus;
        uint8 rxData;
    #endif /* (SPIMaster_SDFlash_RX_SOFTWARE_BUF_ENABLED) */

    #ifdef SPIMaster_SDFlash_RX_ISR_ENTRY_CALLBACK
        SPIMaster_SDFlash_RX_ISR_EntryCallback();
    #endif /* SPIMaster_SDFlash_RX_ISR_ENTRY_CALLBACK */

    /* User code required at start of ISR */
    /* `#START SPIMaster_SDFlash_RX_ISR_START` */

    /* `#END` */
    
    #if(SPIMaster_SDFlash_RX_SOFTWARE_BUF_ENABLED)

        tmpStatus = SPIMaster_SDFlash_GET_STATUS_RX(SPIMaster_SDFlash_swStatusRx);
        SPIMaster_SDFlash_swStatusRx = tmpStatus;

        /* Check if RX data FIFO has some data to be moved into the RX Buffer */
        while(0u != (SPIMaster_SDFlash_swStatusRx & SPIMaster_SDFlash_STS_RX_FIFO_NOT_EMPTY))
        {
            rxData = CY_GET_REG8(SPIMaster_SDFlash_RXDATA_PTR);

            /* Set next pointer. */
            SPIMaster_SDFlash_rxBufferWrite++;
            if(SPIMaster_SDFlash_rxBufferWrite >= SPIMaster_SDFlash_RX_BUFFER_SIZE)
            {
                SPIMaster_SDFlash_rxBufferWrite = 0u;
            }

            if(SPIMaster_SDFlash_rxBufferWrite == SPIMaster_SDFlash_rxBufferRead)
            {
                SPIMaster_SDFlash_rxBufferRead++;
                if(SPIMaster_SDFlash_rxBufferRead >= SPIMaster_SDFlash_RX_BUFFER_SIZE)
                {
                    SPIMaster_SDFlash_rxBufferRead = 0u;
                }

                SPIMaster_SDFlash_rxBufferFull = 1u;
            }

            /* Move data from the FIFO to the Buffer */
            SPIMaster_SDFlash_rxBuffer[SPIMaster_SDFlash_rxBufferWrite] = rxData;

            tmpStatus = SPIMaster_SDFlash_GET_STATUS_RX(SPIMaster_SDFlash_swStatusRx);
            SPIMaster_SDFlash_swStatusRx = tmpStatus;
        }

    #endif /* (SPIMaster_SDFlash_RX_SOFTWARE_BUF_ENABLED) */

    /* User code required at end of ISR (Optional) */
    /* `#START SPIMaster_SDFlash_RX_ISR_END` */

    /* `#END` */
    
    #ifdef SPIMaster_SDFlash_RX_ISR_EXIT_CALLBACK
        SPIMaster_SDFlash_RX_ISR_ExitCallback();
    #endif /* SPIMaster_SDFlash_RX_ISR_EXIT_CALLBACK */
}

/* [] END OF FILE */
