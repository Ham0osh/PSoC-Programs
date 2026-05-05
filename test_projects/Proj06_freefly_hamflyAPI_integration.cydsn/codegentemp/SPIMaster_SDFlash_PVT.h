/*******************************************************************************
* File Name: .h
* Version 2.50
*
* Description:
*  This private header file contains internal definitions for the SPIM
*  component. Do not use these definitions directly in your application.
*
* Note:
*
********************************************************************************
* Copyright 2012-2015, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SPIM_PVT_SPIMaster_SDFlash_H)
#define CY_SPIM_PVT_SPIMaster_SDFlash_H

#include "SPIMaster_SDFlash.h"


/**********************************
*   Functions with external linkage
**********************************/


/**********************************
*   Variables with external linkage
**********************************/

extern volatile uint8 SPIMaster_SDFlash_swStatusTx;
extern volatile uint8 SPIMaster_SDFlash_swStatusRx;

#if(SPIMaster_SDFlash_TX_SOFTWARE_BUF_ENABLED)
    extern volatile uint8 SPIMaster_SDFlash_txBuffer[SPIMaster_SDFlash_TX_BUFFER_SIZE];
    extern volatile uint8 SPIMaster_SDFlash_txBufferRead;
    extern volatile uint8 SPIMaster_SDFlash_txBufferWrite;
    extern volatile uint8 SPIMaster_SDFlash_txBufferFull;
#endif /* (SPIMaster_SDFlash_TX_SOFTWARE_BUF_ENABLED) */

#if(SPIMaster_SDFlash_RX_SOFTWARE_BUF_ENABLED)
    extern volatile uint8 SPIMaster_SDFlash_rxBuffer[SPIMaster_SDFlash_RX_BUFFER_SIZE];
    extern volatile uint8 SPIMaster_SDFlash_rxBufferRead;
    extern volatile uint8 SPIMaster_SDFlash_rxBufferWrite;
    extern volatile uint8 SPIMaster_SDFlash_rxBufferFull;
#endif /* (SPIMaster_SDFlash_RX_SOFTWARE_BUF_ENABLED) */

#endif /* CY_SPIM_PVT_SPIMaster_SDFlash_H */


/* [] END OF FILE */
