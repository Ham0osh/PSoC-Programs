/*******************************************************************************
* File Name: SPIMaster_SDFlash.c
* Version 2.50
*
* Description:
*  This file provides all API functionality of the SPI Master component.
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

#if(SPIMaster_SDFlash_TX_SOFTWARE_BUF_ENABLED)
    volatile uint8 SPIMaster_SDFlash_txBuffer[SPIMaster_SDFlash_TX_BUFFER_SIZE];
    volatile uint8 SPIMaster_SDFlash_txBufferFull;
    volatile uint8 SPIMaster_SDFlash_txBufferRead;
    volatile uint8 SPIMaster_SDFlash_txBufferWrite;
#endif /* (SPIMaster_SDFlash_TX_SOFTWARE_BUF_ENABLED) */

#if(SPIMaster_SDFlash_RX_SOFTWARE_BUF_ENABLED)
    volatile uint8 SPIMaster_SDFlash_rxBuffer[SPIMaster_SDFlash_RX_BUFFER_SIZE];
    volatile uint8 SPIMaster_SDFlash_rxBufferFull;
    volatile uint8 SPIMaster_SDFlash_rxBufferRead;
    volatile uint8 SPIMaster_SDFlash_rxBufferWrite;
#endif /* (SPIMaster_SDFlash_RX_SOFTWARE_BUF_ENABLED) */

uint8 SPIMaster_SDFlash_initVar = 0u;

volatile uint8 SPIMaster_SDFlash_swStatusTx;
volatile uint8 SPIMaster_SDFlash_swStatusRx;


/*******************************************************************************
* Function Name: SPIMaster_SDFlash_Init
********************************************************************************
*
* Summary:
*  Inits/Restores default SPIM configuration provided with customizer.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Side Effects:
*  When this function is called it initializes all of the necessary parameters
*  for execution. i.e. setting the initial interrupt mask, configuring the
*  interrupt service routine, configuring the bit-counter parameters and
*  clearing the FIFO and Status Register.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void SPIMaster_SDFlash_Init(void) 
{
    /* Initialize the Bit counter */
    SPIMaster_SDFlash_COUNTER_PERIOD_REG = SPIMaster_SDFlash_BITCTR_INIT;

    /* Init TX ISR  */
    #if(0u != SPIMaster_SDFlash_INTERNAL_TX_INT_ENABLED)
        CyIntDisable         (SPIMaster_SDFlash_TX_ISR_NUMBER);
        CyIntSetPriority     (SPIMaster_SDFlash_TX_ISR_NUMBER,  SPIMaster_SDFlash_TX_ISR_PRIORITY);
        (void) CyIntSetVector(SPIMaster_SDFlash_TX_ISR_NUMBER, &SPIMaster_SDFlash_TX_ISR);
    #endif /* (0u != SPIMaster_SDFlash_INTERNAL_TX_INT_ENABLED) */

    /* Init RX ISR  */
    #if(0u != SPIMaster_SDFlash_INTERNAL_RX_INT_ENABLED)
        CyIntDisable         (SPIMaster_SDFlash_RX_ISR_NUMBER);
        CyIntSetPriority     (SPIMaster_SDFlash_RX_ISR_NUMBER,  SPIMaster_SDFlash_RX_ISR_PRIORITY);
        (void) CyIntSetVector(SPIMaster_SDFlash_RX_ISR_NUMBER, &SPIMaster_SDFlash_RX_ISR);
    #endif /* (0u != SPIMaster_SDFlash_INTERNAL_RX_INT_ENABLED) */

    /* Clear any stray data from the RX and TX FIFO */
    SPIMaster_SDFlash_ClearFIFO();

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

    (void) SPIMaster_SDFlash_ReadTxStatus(); /* Clear Tx status and swStatusTx */
    (void) SPIMaster_SDFlash_ReadRxStatus(); /* Clear Rx status and swStatusRx */

    /* Configure TX and RX interrupt mask */
    SPIMaster_SDFlash_TX_STATUS_MASK_REG = SPIMaster_SDFlash_TX_INIT_INTERRUPTS_MASK;
    SPIMaster_SDFlash_RX_STATUS_MASK_REG = SPIMaster_SDFlash_RX_INIT_INTERRUPTS_MASK;
}


/*******************************************************************************
* Function Name: SPIMaster_SDFlash_Enable
********************************************************************************
*
* Summary:
*  Enable SPIM component.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void SPIMaster_SDFlash_Enable(void) 
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();
    SPIMaster_SDFlash_COUNTER_CONTROL_REG |= SPIMaster_SDFlash_CNTR_ENABLE;
    SPIMaster_SDFlash_TX_STATUS_ACTL_REG  |= SPIMaster_SDFlash_INT_ENABLE;
    SPIMaster_SDFlash_RX_STATUS_ACTL_REG  |= SPIMaster_SDFlash_INT_ENABLE;
    CyExitCriticalSection(enableInterrupts);

    #if(0u != SPIMaster_SDFlash_INTERNAL_CLOCK)
        SPIMaster_SDFlash_IntClock_Enable();
    #endif /* (0u != SPIMaster_SDFlash_INTERNAL_CLOCK) */

    SPIMaster_SDFlash_EnableTxInt();
    SPIMaster_SDFlash_EnableRxInt();
}


/*******************************************************************************
* Function Name: SPIMaster_SDFlash_Start
********************************************************************************
*
* Summary:
*  Initialize and Enable the SPI Master component.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  SPIMaster_SDFlash_initVar - used to check initial configuration, modified on
*  first function call.
*
* Theory:
*  Enable the clock input to enable operation.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void SPIMaster_SDFlash_Start(void) 
{
    if(0u == SPIMaster_SDFlash_initVar)
    {
        SPIMaster_SDFlash_Init();
        SPIMaster_SDFlash_initVar = 1u;
    }

    SPIMaster_SDFlash_Enable();
}


/*******************************************************************************
* Function Name: SPIMaster_SDFlash_Stop
********************************************************************************
*
* Summary:
*  Disable the SPI Master component.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Theory:
*  Disable the clock input to enable operation.
*
*******************************************************************************/
void SPIMaster_SDFlash_Stop(void) 
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();
    SPIMaster_SDFlash_TX_STATUS_ACTL_REG &= ((uint8) ~SPIMaster_SDFlash_INT_ENABLE);
    SPIMaster_SDFlash_RX_STATUS_ACTL_REG &= ((uint8) ~SPIMaster_SDFlash_INT_ENABLE);
    CyExitCriticalSection(enableInterrupts);

    #if(0u != SPIMaster_SDFlash_INTERNAL_CLOCK)
        SPIMaster_SDFlash_IntClock_Disable();
    #endif /* (0u != SPIMaster_SDFlash_INTERNAL_CLOCK) */

    SPIMaster_SDFlash_DisableTxInt();
    SPIMaster_SDFlash_DisableRxInt();
}


/*******************************************************************************
* Function Name: SPIMaster_SDFlash_EnableTxInt
********************************************************************************
*
* Summary:
*  Enable internal Tx interrupt generation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Theory:
*  Enable the internal Tx interrupt output -or- the interrupt component itself.
*
*******************************************************************************/
void SPIMaster_SDFlash_EnableTxInt(void) 
{
    #if(0u != SPIMaster_SDFlash_INTERNAL_TX_INT_ENABLED)
        CyIntEnable(SPIMaster_SDFlash_TX_ISR_NUMBER);
    #endif /* (0u != SPIMaster_SDFlash_INTERNAL_TX_INT_ENABLED) */
}


/*******************************************************************************
* Function Name: SPIMaster_SDFlash_EnableRxInt
********************************************************************************
*
* Summary:
*  Enable internal Rx interrupt generation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Theory:
*  Enable the internal Rx interrupt output -or- the interrupt component itself.
*
*******************************************************************************/
void SPIMaster_SDFlash_EnableRxInt(void) 
{
    #if(0u != SPIMaster_SDFlash_INTERNAL_RX_INT_ENABLED)
        CyIntEnable(SPIMaster_SDFlash_RX_ISR_NUMBER);
    #endif /* (0u != SPIMaster_SDFlash_INTERNAL_RX_INT_ENABLED) */
}


/*******************************************************************************
* Function Name: SPIMaster_SDFlash_DisableTxInt
********************************************************************************
*
* Summary:
*  Disable internal Tx interrupt generation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Theory:
*  Disable the internal Tx interrupt output -or- the interrupt component itself.
*
*******************************************************************************/
void SPIMaster_SDFlash_DisableTxInt(void) 
{
    #if(0u != SPIMaster_SDFlash_INTERNAL_TX_INT_ENABLED)
        CyIntDisable(SPIMaster_SDFlash_TX_ISR_NUMBER);
    #endif /* (0u != SPIMaster_SDFlash_INTERNAL_TX_INT_ENABLED) */
}


/*******************************************************************************
* Function Name: SPIMaster_SDFlash_DisableRxInt
********************************************************************************
*
* Summary:
*  Disable internal Rx interrupt generation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Theory:
*  Disable the internal Rx interrupt output -or- the interrupt component itself.
*
*******************************************************************************/
void SPIMaster_SDFlash_DisableRxInt(void) 
{
    #if(0u != SPIMaster_SDFlash_INTERNAL_RX_INT_ENABLED)
        CyIntDisable(SPIMaster_SDFlash_RX_ISR_NUMBER);
    #endif /* (0u != SPIMaster_SDFlash_INTERNAL_RX_INT_ENABLED) */
}


/*******************************************************************************
* Function Name: SPIMaster_SDFlash_SetTxInterruptMode
********************************************************************************
*
* Summary:
*  Configure which status bits trigger an interrupt event.
*
* Parameters:
*  intSrc: An or'd combination of the desired status bit masks (defined in the
*  header file).
*
* Return:
*  None.
*
* Theory:
*  Enables the output of specific status bits to the interrupt controller.
*
*******************************************************************************/
void SPIMaster_SDFlash_SetTxInterruptMode(uint8 intSrc) 
{
    SPIMaster_SDFlash_TX_STATUS_MASK_REG = intSrc;
}


/*******************************************************************************
* Function Name: SPIMaster_SDFlash_SetRxInterruptMode
********************************************************************************
*
* Summary:
*  Configure which status bits trigger an interrupt event.
*
* Parameters:
*  intSrc: An or'd combination of the desired status bit masks (defined in the
*  header file).
*
* Return:
*  None.
*
* Theory:
*  Enables the output of specific status bits to the interrupt controller.
*
*******************************************************************************/
void SPIMaster_SDFlash_SetRxInterruptMode(uint8 intSrc) 
{
    SPIMaster_SDFlash_RX_STATUS_MASK_REG  = intSrc;
}


/*******************************************************************************
* Function Name: SPIMaster_SDFlash_ReadTxStatus
********************************************************************************
*
* Summary:
*  Read the Tx status register for the component.
*
* Parameters:
*  None.
*
* Return:
*  Contents of the Tx status register.
*
* Global variables:
*  SPIMaster_SDFlash_swStatusTx - used to store in software status register,
*  modified every function call - resets to zero.
*
* Theory:
*  Allows the user and the API to read the Tx status register for error
*  detection and flow control.
*
* Side Effects:
*  Clear Tx status register of the component.
*
* Reentrant:
*  No.
*
*******************************************************************************/
uint8 SPIMaster_SDFlash_ReadTxStatus(void) 
{
    uint8 tmpStatus;

    #if(SPIMaster_SDFlash_TX_SOFTWARE_BUF_ENABLED)
        /* Disable TX interrupt to protect global veriables */
        SPIMaster_SDFlash_DisableTxInt();

        tmpStatus = SPIMaster_SDFlash_GET_STATUS_TX(SPIMaster_SDFlash_swStatusTx);
        SPIMaster_SDFlash_swStatusTx = 0u;

        SPIMaster_SDFlash_EnableTxInt();

    #else

        tmpStatus = SPIMaster_SDFlash_TX_STATUS_REG;

    #endif /* (SPIMaster_SDFlash_TX_SOFTWARE_BUF_ENABLED) */

    return(tmpStatus);
}


/*******************************************************************************
* Function Name: SPIMaster_SDFlash_ReadRxStatus
********************************************************************************
*
* Summary:
*  Read the Rx status register for the component.
*
* Parameters:
*  None.
*
* Return:
*  Contents of the Rx status register.
*
* Global variables:
*  SPIMaster_SDFlash_swStatusRx - used to store in software Rx status register,
*  modified every function call - resets to zero.
*
* Theory:
*  Allows the user and the API to read the Rx status register for error
*  detection and flow control.
*
* Side Effects:
*  Clear Rx status register of the component.
*
* Reentrant:
*  No.
*
*******************************************************************************/
uint8 SPIMaster_SDFlash_ReadRxStatus(void) 
{
    uint8 tmpStatus;

    #if(SPIMaster_SDFlash_RX_SOFTWARE_BUF_ENABLED)
        /* Disable RX interrupt to protect global veriables */
        SPIMaster_SDFlash_DisableRxInt();

        tmpStatus = SPIMaster_SDFlash_GET_STATUS_RX(SPIMaster_SDFlash_swStatusRx);
        SPIMaster_SDFlash_swStatusRx = 0u;

        SPIMaster_SDFlash_EnableRxInt();

    #else

        tmpStatus = SPIMaster_SDFlash_RX_STATUS_REG;

    #endif /* (SPIMaster_SDFlash_RX_SOFTWARE_BUF_ENABLED) */

    return(tmpStatus);
}


/*******************************************************************************
* Function Name: SPIMaster_SDFlash_WriteTxData
********************************************************************************
*
* Summary:
*  Write a byte of data to be sent across the SPI.
*
* Parameters:
*  txDataByte: The data value to send across the SPI.
*
* Return:
*  None.
*
* Global variables:
*  SPIMaster_SDFlash_txBufferWrite - used for the account of the bytes which
*  have been written down in the TX software buffer, modified every function
*  call if TX Software Buffer is used.
*  SPIMaster_SDFlash_txBufferRead - used for the account of the bytes which
*  have been read from the TX software buffer.
*  SPIMaster_SDFlash_txBuffer[SPIMaster_SDFlash_TX_BUFFER_SIZE] - used to store
*  data to sending, modified every function call if TX Software Buffer is used.
*
* Theory:
*  Allows the user to transmit any byte of data in a single transfer.
*
* Side Effects:
*  If this function is called again before the previous byte is finished then
*  the next byte will be appended to the transfer with no time between
*  the byte transfers. Clear Tx status register of the component.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void SPIMaster_SDFlash_WriteTxData(uint8 txData) 
{
    #if(SPIMaster_SDFlash_TX_SOFTWARE_BUF_ENABLED)

        uint8 tempStatus;
        uint8 tmpTxBufferRead;

        /* Block if TX buffer is FULL: don't overwrite */
        do
        {
            tmpTxBufferRead = SPIMaster_SDFlash_txBufferRead;
            if(0u == tmpTxBufferRead)
            {
                tmpTxBufferRead = (SPIMaster_SDFlash_TX_BUFFER_SIZE - 1u);
            }
            else
            {
                tmpTxBufferRead--;
            }

        }while(tmpTxBufferRead == SPIMaster_SDFlash_txBufferWrite);

        /* Disable TX interrupt to protect global veriables */
        SPIMaster_SDFlash_DisableTxInt();

        tempStatus = SPIMaster_SDFlash_GET_STATUS_TX(SPIMaster_SDFlash_swStatusTx);
        SPIMaster_SDFlash_swStatusTx = tempStatus;


        if((SPIMaster_SDFlash_txBufferRead == SPIMaster_SDFlash_txBufferWrite) &&
           (0u != (SPIMaster_SDFlash_swStatusTx & SPIMaster_SDFlash_STS_TX_FIFO_NOT_FULL)))
        {
            /* Put data element into the TX FIFO */
            CY_SET_REG8(SPIMaster_SDFlash_TXDATA_PTR, txData);
        }
        else
        {
            /* Add to the TX software buffer */
            SPIMaster_SDFlash_txBufferWrite++;
            if(SPIMaster_SDFlash_txBufferWrite >= SPIMaster_SDFlash_TX_BUFFER_SIZE)
            {
                SPIMaster_SDFlash_txBufferWrite = 0u;
            }

            if(SPIMaster_SDFlash_txBufferWrite == SPIMaster_SDFlash_txBufferRead)
            {
                SPIMaster_SDFlash_txBufferRead++;
                if(SPIMaster_SDFlash_txBufferRead >= SPIMaster_SDFlash_TX_BUFFER_SIZE)
                {
                    SPIMaster_SDFlash_txBufferRead = 0u;
                }
                SPIMaster_SDFlash_txBufferFull = 1u;
            }

            SPIMaster_SDFlash_txBuffer[SPIMaster_SDFlash_txBufferWrite] = txData;

            SPIMaster_SDFlash_TX_STATUS_MASK_REG |= SPIMaster_SDFlash_STS_TX_FIFO_NOT_FULL;
        }

        SPIMaster_SDFlash_EnableTxInt();

    #else
        /* Wait until TX FIFO has a place */
        while(0u == (SPIMaster_SDFlash_TX_STATUS_REG & SPIMaster_SDFlash_STS_TX_FIFO_NOT_FULL))
        {
        }

        /* Put data element into the TX FIFO */
        CY_SET_REG8(SPIMaster_SDFlash_TXDATA_PTR, txData);

    #endif /* (SPIMaster_SDFlash_TX_SOFTWARE_BUF_ENABLED) */
}


/*******************************************************************************
* Function Name: SPIMaster_SDFlash_ReadRxData
********************************************************************************
*
* Summary:
*  Read the next byte of data received across the SPI.
*
* Parameters:
*  None.
*
* Return:
*  The next byte of data read from the FIFO.
*
* Global variables:
*  SPIMaster_SDFlash_rxBufferWrite - used for the account of the bytes which
*  have been written down in the RX software buffer.
*  SPIMaster_SDFlash_rxBufferRead - used for the account of the bytes which
*  have been read from the RX software buffer, modified every function
*  call if RX Software Buffer is used.
*  SPIMaster_SDFlash_rxBuffer[SPIMaster_SDFlash_RX_BUFFER_SIZE] - used to store
*  received data.
*
* Theory:
*  Allows the user to read a byte of data received.
*
* Side Effects:
*  Will return invalid data if the FIFO is empty. The user should Call
*  GetRxBufferSize() and if it returns a non-zero value then it is safe to call
*  ReadByte() function.
*
* Reentrant:
*  No.
*
*******************************************************************************/
uint8 SPIMaster_SDFlash_ReadRxData(void) 
{
    uint8 rxData;

    #if(SPIMaster_SDFlash_RX_SOFTWARE_BUF_ENABLED)

        /* Disable RX interrupt to protect global veriables */
        SPIMaster_SDFlash_DisableRxInt();

        if(SPIMaster_SDFlash_rxBufferRead != SPIMaster_SDFlash_rxBufferWrite)
        {
            if(0u == SPIMaster_SDFlash_rxBufferFull)
            {
                SPIMaster_SDFlash_rxBufferRead++;
                if(SPIMaster_SDFlash_rxBufferRead >= SPIMaster_SDFlash_RX_BUFFER_SIZE)
                {
                    SPIMaster_SDFlash_rxBufferRead = 0u;
                }
            }
            else
            {
                SPIMaster_SDFlash_rxBufferFull = 0u;
            }
        }

        rxData = SPIMaster_SDFlash_rxBuffer[SPIMaster_SDFlash_rxBufferRead];

        SPIMaster_SDFlash_EnableRxInt();

    #else

        rxData = CY_GET_REG8(SPIMaster_SDFlash_RXDATA_PTR);

    #endif /* (SPIMaster_SDFlash_RX_SOFTWARE_BUF_ENABLED) */

    return(rxData);
}


/*******************************************************************************
* Function Name: SPIMaster_SDFlash_GetRxBufferSize
********************************************************************************
*
* Summary:
*  Returns the number of bytes/words of data currently held in the RX buffer.
*  If RX Software Buffer not used then function return 0 if FIFO empty or 1 if
*  FIFO not empty. In another case function return size of RX Software Buffer.
*
* Parameters:
*  None.
*
* Return:
*  Integer count of the number of bytes/words in the RX buffer.
*
* Global variables:
*  SPIMaster_SDFlash_rxBufferWrite - used for the account of the bytes which
*  have been written down in the RX software buffer.
*  SPIMaster_SDFlash_rxBufferRead - used for the account of the bytes which
*  have been read from the RX software buffer.
*
* Side Effects:
*  Clear status register of the component.
*
*******************************************************************************/
uint8 SPIMaster_SDFlash_GetRxBufferSize(void) 
{
    uint8 size;

    #if(SPIMaster_SDFlash_RX_SOFTWARE_BUF_ENABLED)

        /* Disable RX interrupt to protect global veriables */
        SPIMaster_SDFlash_DisableRxInt();

        if(SPIMaster_SDFlash_rxBufferRead == SPIMaster_SDFlash_rxBufferWrite)
        {
            size = 0u;
        }
        else if(SPIMaster_SDFlash_rxBufferRead < SPIMaster_SDFlash_rxBufferWrite)
        {
            size = (SPIMaster_SDFlash_rxBufferWrite - SPIMaster_SDFlash_rxBufferRead);
        }
        else
        {
            size = (SPIMaster_SDFlash_RX_BUFFER_SIZE - SPIMaster_SDFlash_rxBufferRead) + SPIMaster_SDFlash_rxBufferWrite;
        }

        SPIMaster_SDFlash_EnableRxInt();

    #else

        /* We can only know if there is data in the RX FIFO */
        size = (0u != (SPIMaster_SDFlash_RX_STATUS_REG & SPIMaster_SDFlash_STS_RX_FIFO_NOT_EMPTY)) ? 1u : 0u;

    #endif /* (SPIMaster_SDFlash_TX_SOFTWARE_BUF_ENABLED) */

    return(size);
}


/*******************************************************************************
* Function Name: SPIMaster_SDFlash_GetTxBufferSize
********************************************************************************
*
* Summary:
*  Returns the number of bytes/words of data currently held in the TX buffer.
*  If TX Software Buffer not used then function return 0 - if FIFO empty, 1 - if
*  FIFO not full, 4 - if FIFO full. In another case function return size of TX
*  Software Buffer.
*
* Parameters:
*  None.
*
* Return:
*  Integer count of the number of bytes/words in the TX buffer.
*
* Global variables:
*  SPIMaster_SDFlash_txBufferWrite - used for the account of the bytes which
*  have been written down in the TX software buffer.
*  SPIMaster_SDFlash_txBufferRead - used for the account of the bytes which
*  have been read from the TX software buffer.
*
* Side Effects:
*  Clear status register of the component.
*
*******************************************************************************/
uint8  SPIMaster_SDFlash_GetTxBufferSize(void) 
{
    uint8 size;

    #if(SPIMaster_SDFlash_TX_SOFTWARE_BUF_ENABLED)
        /* Disable TX interrupt to protect global veriables */
        SPIMaster_SDFlash_DisableTxInt();

        if(SPIMaster_SDFlash_txBufferRead == SPIMaster_SDFlash_txBufferWrite)
        {
            size = 0u;
        }
        else if(SPIMaster_SDFlash_txBufferRead < SPIMaster_SDFlash_txBufferWrite)
        {
            size = (SPIMaster_SDFlash_txBufferWrite - SPIMaster_SDFlash_txBufferRead);
        }
        else
        {
            size = (SPIMaster_SDFlash_TX_BUFFER_SIZE - SPIMaster_SDFlash_txBufferRead) + SPIMaster_SDFlash_txBufferWrite;
        }

        SPIMaster_SDFlash_EnableTxInt();

    #else

        size = SPIMaster_SDFlash_TX_STATUS_REG;

        if(0u != (size & SPIMaster_SDFlash_STS_TX_FIFO_EMPTY))
        {
            size = 0u;
        }
        else if(0u != (size & SPIMaster_SDFlash_STS_TX_FIFO_NOT_FULL))
        {
            size = 1u;
        }
        else
        {
            size = SPIMaster_SDFlash_FIFO_SIZE;
        }

    #endif /* (SPIMaster_SDFlash_TX_SOFTWARE_BUF_ENABLED) */

    return(size);
}


/*******************************************************************************
* Function Name: SPIMaster_SDFlash_ClearRxBuffer
********************************************************************************
*
* Summary:
*  Clear the RX RAM buffer by setting the read and write pointers both to zero.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  SPIMaster_SDFlash_rxBufferWrite - used for the account of the bytes which
*  have been written down in the RX software buffer, modified every function
*  call - resets to zero.
*  SPIMaster_SDFlash_rxBufferRead - used for the account of the bytes which
*  have been read from the RX software buffer, modified every function call -
*  resets to zero.
*
* Theory:
*  Setting the pointers to zero makes the system believe there is no data to
*  read and writing will resume at address 0 overwriting any data that may have
*  remained in the RAM.
*
* Side Effects:
*  Any received data not read from the RAM buffer will be lost when overwritten.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void SPIMaster_SDFlash_ClearRxBuffer(void) 
{
    /* Clear Hardware RX FIFO */
    while(0u !=(SPIMaster_SDFlash_RX_STATUS_REG & SPIMaster_SDFlash_STS_RX_FIFO_NOT_EMPTY))
    {
        (void) CY_GET_REG8(SPIMaster_SDFlash_RXDATA_PTR);
    }

    #if(SPIMaster_SDFlash_RX_SOFTWARE_BUF_ENABLED)
        /* Disable RX interrupt to protect global veriables */
        SPIMaster_SDFlash_DisableRxInt();

        SPIMaster_SDFlash_rxBufferFull  = 0u;
        SPIMaster_SDFlash_rxBufferRead  = 0u;
        SPIMaster_SDFlash_rxBufferWrite = 0u;

        SPIMaster_SDFlash_EnableRxInt();
    #endif /* (SPIMaster_SDFlash_RX_SOFTWARE_BUF_ENABLED) */
}


/*******************************************************************************
* Function Name: SPIMaster_SDFlash_ClearTxBuffer
********************************************************************************
*
* Summary:
*  Clear the TX RAM buffer by setting the read and write pointers both to zero.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  SPIMaster_SDFlash_txBufferWrite - used for the account of the bytes which
*  have been written down in the TX software buffer, modified every function
*  call - resets to zero.
*  SPIMaster_SDFlash_txBufferRead - used for the account of the bytes which
*  have been read from the TX software buffer, modified every function call -
*  resets to zero.
*
* Theory:
*  Setting the pointers to zero makes the system believe there is no data to
*  read and writing will resume at address 0 overwriting any data that may have
*  remained in the RAM.
*
* Side Effects:
*  Any data not yet transmitted from the RAM buffer will be lost when
*  overwritten.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void SPIMaster_SDFlash_ClearTxBuffer(void) 
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();
    /* Clear TX FIFO */
    SPIMaster_SDFlash_AUX_CONTROL_DP0_REG |= ((uint8)  SPIMaster_SDFlash_TX_FIFO_CLR);
    SPIMaster_SDFlash_AUX_CONTROL_DP0_REG &= ((uint8) ~SPIMaster_SDFlash_TX_FIFO_CLR);

    #if(SPIMaster_SDFlash_USE_SECOND_DATAPATH)
        /* Clear TX FIFO for 2nd Datapath */
        SPIMaster_SDFlash_AUX_CONTROL_DP1_REG |= ((uint8)  SPIMaster_SDFlash_TX_FIFO_CLR);
        SPIMaster_SDFlash_AUX_CONTROL_DP1_REG &= ((uint8) ~SPIMaster_SDFlash_TX_FIFO_CLR);
    #endif /* (SPIMaster_SDFlash_USE_SECOND_DATAPATH) */
    CyExitCriticalSection(enableInterrupts);

    #if(SPIMaster_SDFlash_TX_SOFTWARE_BUF_ENABLED)
        /* Disable TX interrupt to protect global veriables */
        SPIMaster_SDFlash_DisableTxInt();

        SPIMaster_SDFlash_txBufferFull  = 0u;
        SPIMaster_SDFlash_txBufferRead  = 0u;
        SPIMaster_SDFlash_txBufferWrite = 0u;

        /* Buffer is EMPTY: disable TX FIFO NOT FULL interrupt */
        SPIMaster_SDFlash_TX_STATUS_MASK_REG &= ((uint8) ~SPIMaster_SDFlash_STS_TX_FIFO_NOT_FULL);

        SPIMaster_SDFlash_EnableTxInt();
    #endif /* (SPIMaster_SDFlash_TX_SOFTWARE_BUF_ENABLED) */
}


#if(0u != SPIMaster_SDFlash_BIDIRECTIONAL_MODE)
    /*******************************************************************************
    * Function Name: SPIMaster_SDFlash_TxEnable
    ********************************************************************************
    *
    * Summary:
    *  If the SPI master is configured to use a single bi-directional pin then this
    *  will set the bi-directional pin to transmit.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    *******************************************************************************/
    void SPIMaster_SDFlash_TxEnable(void) 
    {
        SPIMaster_SDFlash_CONTROL_REG |= SPIMaster_SDFlash_CTRL_TX_SIGNAL_EN;
    }


    /*******************************************************************************
    * Function Name: SPIMaster_SDFlash_TxDisable
    ********************************************************************************
    *
    * Summary:
    *  If the SPI master is configured to use a single bi-directional pin then this
    *  will set the bi-directional pin to receive.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    *******************************************************************************/
    void SPIMaster_SDFlash_TxDisable(void) 
    {
        SPIMaster_SDFlash_CONTROL_REG &= ((uint8) ~SPIMaster_SDFlash_CTRL_TX_SIGNAL_EN);
    }

#endif /* (0u != SPIMaster_SDFlash_BIDIRECTIONAL_MODE) */


/*******************************************************************************
* Function Name: SPIMaster_SDFlash_PutArray
********************************************************************************
*
* Summary:
*  Write available data from ROM/RAM to the TX buffer while space is available
*  in the TX buffer. Keep trying until all data is passed to the TX buffer.
*
* Parameters:
*  *buffer: Pointer to the location in RAM containing the data to send
*  byteCount: The number of bytes to move to the transmit buffer.
*
* Return:
*  None.
*
* Side Effects:
*  Will stay in this routine until all data has been sent.  May get locked in
*  this loop if data is not being initiated by the master if there is not
*  enough room in the TX FIFO.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void SPIMaster_SDFlash_PutArray(const uint8 buffer[], uint8 byteCount)
                                                                          
{
    uint8 bufIndex;

    bufIndex = 0u;

    while(byteCount > 0u)
    {
        SPIMaster_SDFlash_WriteTxData(buffer[bufIndex]);
        bufIndex++;
        byteCount--;
    }
}


/*******************************************************************************
* Function Name: SPIMaster_SDFlash_ClearFIFO
********************************************************************************
*
* Summary:
*  Clear the RX and TX FIFO's of all data for a fresh start.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Side Effects:
*  Clear status register of the component.
*
*******************************************************************************/
void SPIMaster_SDFlash_ClearFIFO(void) 
{
    uint8 enableInterrupts;

    /* Clear Hardware RX FIFO */
    while(0u !=(SPIMaster_SDFlash_RX_STATUS_REG & SPIMaster_SDFlash_STS_RX_FIFO_NOT_EMPTY))
    {
        (void) CY_GET_REG8(SPIMaster_SDFlash_RXDATA_PTR);
    }

    enableInterrupts = CyEnterCriticalSection();
    /* Clear TX FIFO */
    SPIMaster_SDFlash_AUX_CONTROL_DP0_REG |= ((uint8)  SPIMaster_SDFlash_TX_FIFO_CLR);
    SPIMaster_SDFlash_AUX_CONTROL_DP0_REG &= ((uint8) ~SPIMaster_SDFlash_TX_FIFO_CLR);

    #if(SPIMaster_SDFlash_USE_SECOND_DATAPATH)
        /* Clear TX FIFO for 2nd Datapath */
        SPIMaster_SDFlash_AUX_CONTROL_DP1_REG |= ((uint8)  SPIMaster_SDFlash_TX_FIFO_CLR);
        SPIMaster_SDFlash_AUX_CONTROL_DP1_REG &= ((uint8) ~SPIMaster_SDFlash_TX_FIFO_CLR);
    #endif /* (SPIMaster_SDFlash_USE_SECOND_DATAPATH) */
    CyExitCriticalSection(enableInterrupts);
}


/* Following functions are for version Compatibility, they are obsolete.
*  Please do not use it in new projects.
*/


/*******************************************************************************
* Function Name: SPIMaster_SDFlash_EnableInt
********************************************************************************
*
* Summary:
*  Enable internal interrupt generation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Theory:
*  Enable the internal interrupt output -or- the interrupt component itself.
*
*******************************************************************************/
void SPIMaster_SDFlash_EnableInt(void) 
{
    SPIMaster_SDFlash_EnableRxInt();
    SPIMaster_SDFlash_EnableTxInt();
}


/*******************************************************************************
* Function Name: SPIMaster_SDFlash_DisableInt
********************************************************************************
*
* Summary:
*  Disable internal interrupt generation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Theory:
*  Disable the internal interrupt output -or- the interrupt component itself.
*
*******************************************************************************/
void SPIMaster_SDFlash_DisableInt(void) 
{
    SPIMaster_SDFlash_DisableTxInt();
    SPIMaster_SDFlash_DisableRxInt();
}


/*******************************************************************************
* Function Name: SPIMaster_SDFlash_SetInterruptMode
********************************************************************************
*
* Summary:
*  Configure which status bits trigger an interrupt event.
*
* Parameters:
*  intSrc: An or'd combination of the desired status bit masks (defined in the
*  header file).
*
* Return:
*  None.
*
* Theory:
*  Enables the output of specific status bits to the interrupt controller.
*
*******************************************************************************/
void SPIMaster_SDFlash_SetInterruptMode(uint8 intSrc) 
{
    SPIMaster_SDFlash_TX_STATUS_MASK_REG  = (intSrc & ((uint8) ~SPIMaster_SDFlash_STS_SPI_IDLE));
    SPIMaster_SDFlash_RX_STATUS_MASK_REG  =  intSrc;
}


/*******************************************************************************
* Function Name: SPIMaster_SDFlash_ReadStatus
********************************************************************************
*
* Summary:
*  Read the status register for the component.
*
* Parameters:
*  None.
*
* Return:
*  Contents of the status register.
*
* Global variables:
*  SPIMaster_SDFlash_swStatus - used to store in software status register,
*  modified every function call - resets to zero.
*
* Theory:
*  Allows the user and the API to read the status register for error detection
*  and flow control.
*
* Side Effects:
*  Clear status register of the component.
*
* Reentrant:
*  No.
*
*******************************************************************************/
uint8 SPIMaster_SDFlash_ReadStatus(void) 
{
    uint8 tmpStatus;

    #if(SPIMaster_SDFlash_TX_SOFTWARE_BUF_ENABLED || SPIMaster_SDFlash_RX_SOFTWARE_BUF_ENABLED)

        SPIMaster_SDFlash_DisableInt();

        tmpStatus  = SPIMaster_SDFlash_GET_STATUS_RX(SPIMaster_SDFlash_swStatusRx);
        tmpStatus |= SPIMaster_SDFlash_GET_STATUS_TX(SPIMaster_SDFlash_swStatusTx);
        tmpStatus &= ((uint8) ~SPIMaster_SDFlash_STS_SPI_IDLE);

        SPIMaster_SDFlash_swStatusTx = 0u;
        SPIMaster_SDFlash_swStatusRx = 0u;

        SPIMaster_SDFlash_EnableInt();

    #else

        tmpStatus  = SPIMaster_SDFlash_RX_STATUS_REG;
        tmpStatus |= SPIMaster_SDFlash_TX_STATUS_REG;
        tmpStatus &= ((uint8) ~SPIMaster_SDFlash_STS_SPI_IDLE);

    #endif /* (SPIMaster_SDFlash_TX_SOFTWARE_BUF_ENABLED || SPIMaster_SDFlash_RX_SOFTWARE_BUF_ENABLED) */

    return(tmpStatus);
}


/* [] END OF FILE */
