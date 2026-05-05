/*******************************************************************************
* File Name: SPIMaster_SDFlash.h
* Version 2.50
*
* Description:
*  Contains the function prototypes, constants and register definition
*  of the SPI Master Component.
*
* Note:
*  None
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SPIM_SPIMaster_SDFlash_H)
#define CY_SPIM_SPIMaster_SDFlash_H

#include "cyfitter.h"
#include "cytypes.h"
#include "CyLib.h" /* For CyEnterCriticalSection() and CyExitCriticalSection() functions */


/***************************************
*   Conditional Compilation Parameters
***************************************/

#define SPIMaster_SDFlash_INTERNAL_CLOCK             (0u)

#if(0u != SPIMaster_SDFlash_INTERNAL_CLOCK)
    #include "SPIMaster_SDFlash_IntClock.h"
#endif /* (0u != SPIMaster_SDFlash_INTERNAL_CLOCK) */

#define SPIMaster_SDFlash_MODE                       (1u)
#define SPIMaster_SDFlash_DATA_WIDTH                 (8u)
#define SPIMaster_SDFlash_MODE_USE_ZERO              (1u)
#define SPIMaster_SDFlash_BIDIRECTIONAL_MODE         (0u)

/* Internal interrupt handling */
#define SPIMaster_SDFlash_TX_BUFFER_SIZE             (4u)
#define SPIMaster_SDFlash_RX_BUFFER_SIZE             (4u)
#define SPIMaster_SDFlash_INTERNAL_TX_INT_ENABLED    (0u)
#define SPIMaster_SDFlash_INTERNAL_RX_INT_ENABLED    (0u)

#define SPIMaster_SDFlash_SINGLE_REG_SIZE            (8u)
#define SPIMaster_SDFlash_USE_SECOND_DATAPATH        (SPIMaster_SDFlash_DATA_WIDTH > SPIMaster_SDFlash_SINGLE_REG_SIZE)

#define SPIMaster_SDFlash_FIFO_SIZE                  (4u)
#define SPIMaster_SDFlash_TX_SOFTWARE_BUF_ENABLED    ((0u != SPIMaster_SDFlash_INTERNAL_TX_INT_ENABLED) && \
                                                     (SPIMaster_SDFlash_TX_BUFFER_SIZE > SPIMaster_SDFlash_FIFO_SIZE))

#define SPIMaster_SDFlash_RX_SOFTWARE_BUF_ENABLED    ((0u != SPIMaster_SDFlash_INTERNAL_RX_INT_ENABLED) && \
                                                     (SPIMaster_SDFlash_RX_BUFFER_SIZE > SPIMaster_SDFlash_FIFO_SIZE))


/***************************************
*        Data Struct Definition
***************************************/

/* Sleep Mode API Support */
typedef struct
{
    uint8 enableState;
    uint8 cntrPeriod;
} SPIMaster_SDFlash_BACKUP_STRUCT;


/***************************************
*        Function Prototypes
***************************************/

void  SPIMaster_SDFlash_Init(void)                           ;
void  SPIMaster_SDFlash_Enable(void)                         ;
void  SPIMaster_SDFlash_Start(void)                          ;
void  SPIMaster_SDFlash_Stop(void)                           ;

void  SPIMaster_SDFlash_EnableTxInt(void)                    ;
void  SPIMaster_SDFlash_EnableRxInt(void)                    ;
void  SPIMaster_SDFlash_DisableTxInt(void)                   ;
void  SPIMaster_SDFlash_DisableRxInt(void)                   ;

void  SPIMaster_SDFlash_Sleep(void)                          ;
void  SPIMaster_SDFlash_Wakeup(void)                         ;
void  SPIMaster_SDFlash_SaveConfig(void)                     ;
void  SPIMaster_SDFlash_RestoreConfig(void)                  ;

void  SPIMaster_SDFlash_SetTxInterruptMode(uint8 intSrc)     ;
void  SPIMaster_SDFlash_SetRxInterruptMode(uint8 intSrc)     ;
uint8 SPIMaster_SDFlash_ReadTxStatus(void)                   ;
uint8 SPIMaster_SDFlash_ReadRxStatus(void)                   ;
void  SPIMaster_SDFlash_WriteTxData(uint8 txData)  \
                                                            ;
uint8 SPIMaster_SDFlash_ReadRxData(void) \
                                                            ;
uint8 SPIMaster_SDFlash_GetRxBufferSize(void)                ;
uint8 SPIMaster_SDFlash_GetTxBufferSize(void)                ;
void  SPIMaster_SDFlash_ClearRxBuffer(void)                  ;
void  SPIMaster_SDFlash_ClearTxBuffer(void)                  ;
void  SPIMaster_SDFlash_ClearFIFO(void)                              ;
void  SPIMaster_SDFlash_PutArray(const uint8 buffer[], uint8 byteCount) \
                                                            ;

#if(0u != SPIMaster_SDFlash_BIDIRECTIONAL_MODE)
    void  SPIMaster_SDFlash_TxEnable(void)                   ;
    void  SPIMaster_SDFlash_TxDisable(void)                  ;
#endif /* (0u != SPIMaster_SDFlash_BIDIRECTIONAL_MODE) */

CY_ISR_PROTO(SPIMaster_SDFlash_TX_ISR);
CY_ISR_PROTO(SPIMaster_SDFlash_RX_ISR);


/***************************************
*   Variable with external linkage
***************************************/

extern uint8 SPIMaster_SDFlash_initVar;


/***************************************
*           API Constants
***************************************/

#define SPIMaster_SDFlash_TX_ISR_NUMBER     ((uint8) (SPIMaster_SDFlash_TxInternalInterrupt__INTC_NUMBER))
#define SPIMaster_SDFlash_RX_ISR_NUMBER     ((uint8) (SPIMaster_SDFlash_RxInternalInterrupt__INTC_NUMBER))

#define SPIMaster_SDFlash_TX_ISR_PRIORITY   ((uint8) (SPIMaster_SDFlash_TxInternalInterrupt__INTC_PRIOR_NUM))
#define SPIMaster_SDFlash_RX_ISR_PRIORITY   ((uint8) (SPIMaster_SDFlash_RxInternalInterrupt__INTC_PRIOR_NUM))


/***************************************
*    Initial Parameter Constants
***************************************/

#define SPIMaster_SDFlash_INT_ON_SPI_DONE    ((uint8) (0u   << SPIMaster_SDFlash_STS_SPI_DONE_SHIFT))
#define SPIMaster_SDFlash_INT_ON_TX_EMPTY    ((uint8) (0u   << SPIMaster_SDFlash_STS_TX_FIFO_EMPTY_SHIFT))
#define SPIMaster_SDFlash_INT_ON_TX_NOT_FULL ((uint8) (0u << \
                                                                           SPIMaster_SDFlash_STS_TX_FIFO_NOT_FULL_SHIFT))
#define SPIMaster_SDFlash_INT_ON_BYTE_COMP   ((uint8) (0u  << SPIMaster_SDFlash_STS_BYTE_COMPLETE_SHIFT))
#define SPIMaster_SDFlash_INT_ON_SPI_IDLE    ((uint8) (0u   << SPIMaster_SDFlash_STS_SPI_IDLE_SHIFT))

/* Disable TX_NOT_FULL if software buffer is used */
#define SPIMaster_SDFlash_INT_ON_TX_NOT_FULL_DEF ((SPIMaster_SDFlash_TX_SOFTWARE_BUF_ENABLED) ? \
                                                                        (0u) : (SPIMaster_SDFlash_INT_ON_TX_NOT_FULL))

/* TX interrupt mask */
#define SPIMaster_SDFlash_TX_INIT_INTERRUPTS_MASK    (SPIMaster_SDFlash_INT_ON_SPI_DONE  | \
                                                     SPIMaster_SDFlash_INT_ON_TX_EMPTY  | \
                                                     SPIMaster_SDFlash_INT_ON_TX_NOT_FULL_DEF | \
                                                     SPIMaster_SDFlash_INT_ON_BYTE_COMP | \
                                                     SPIMaster_SDFlash_INT_ON_SPI_IDLE)

#define SPIMaster_SDFlash_INT_ON_RX_FULL         ((uint8) (0u << \
                                                                          SPIMaster_SDFlash_STS_RX_FIFO_FULL_SHIFT))
#define SPIMaster_SDFlash_INT_ON_RX_NOT_EMPTY    ((uint8) (0u << \
                                                                          SPIMaster_SDFlash_STS_RX_FIFO_NOT_EMPTY_SHIFT))
#define SPIMaster_SDFlash_INT_ON_RX_OVER         ((uint8) (0u << \
                                                                          SPIMaster_SDFlash_STS_RX_FIFO_OVERRUN_SHIFT))

/* RX interrupt mask */
#define SPIMaster_SDFlash_RX_INIT_INTERRUPTS_MASK    (SPIMaster_SDFlash_INT_ON_RX_FULL      | \
                                                     SPIMaster_SDFlash_INT_ON_RX_NOT_EMPTY | \
                                                     SPIMaster_SDFlash_INT_ON_RX_OVER)
/* Nubmer of bits to receive/transmit */
#define SPIMaster_SDFlash_BITCTR_INIT            (((uint8) (SPIMaster_SDFlash_DATA_WIDTH << 1u)) - 1u)


/***************************************
*             Registers
***************************************/
#if(CY_PSOC3 || CY_PSOC5)
    #define SPIMaster_SDFlash_TXDATA_REG (* (reg8 *) \
                                                SPIMaster_SDFlash_BSPIM_sR8_Dp_u0__F0_REG)
    #define SPIMaster_SDFlash_TXDATA_PTR (  (reg8 *) \
                                                SPIMaster_SDFlash_BSPIM_sR8_Dp_u0__F0_REG)
    #define SPIMaster_SDFlash_RXDATA_REG (* (reg8 *) \
                                                SPIMaster_SDFlash_BSPIM_sR8_Dp_u0__F1_REG)
    #define SPIMaster_SDFlash_RXDATA_PTR (  (reg8 *) \
                                                SPIMaster_SDFlash_BSPIM_sR8_Dp_u0__F1_REG)
#else   /* PSOC4 */
    #if(SPIMaster_SDFlash_USE_SECOND_DATAPATH)
        #define SPIMaster_SDFlash_TXDATA_REG (* (reg16 *) \
                                          SPIMaster_SDFlash_BSPIM_sR8_Dp_u0__16BIT_F0_REG)
        #define SPIMaster_SDFlash_TXDATA_PTR (  (reg16 *) \
                                          SPIMaster_SDFlash_BSPIM_sR8_Dp_u0__16BIT_F0_REG)
        #define SPIMaster_SDFlash_RXDATA_REG (* (reg16 *) \
                                          SPIMaster_SDFlash_BSPIM_sR8_Dp_u0__16BIT_F1_REG)
        #define SPIMaster_SDFlash_RXDATA_PTR (  (reg16 *) \
                                          SPIMaster_SDFlash_BSPIM_sR8_Dp_u0__16BIT_F1_REG)
    #else
        #define SPIMaster_SDFlash_TXDATA_REG (* (reg8 *) \
                                                SPIMaster_SDFlash_BSPIM_sR8_Dp_u0__F0_REG)
        #define SPIMaster_SDFlash_TXDATA_PTR (  (reg8 *) \
                                                SPIMaster_SDFlash_BSPIM_sR8_Dp_u0__F0_REG)
        #define SPIMaster_SDFlash_RXDATA_REG (* (reg8 *) \
                                                SPIMaster_SDFlash_BSPIM_sR8_Dp_u0__F1_REG)
        #define SPIMaster_SDFlash_RXDATA_PTR (  (reg8 *) \
                                                SPIMaster_SDFlash_BSPIM_sR8_Dp_u0__F1_REG)
    #endif /* (SPIMaster_SDFlash_USE_SECOND_DATAPATH) */
#endif     /* (CY_PSOC3 || CY_PSOC5) */

#define SPIMaster_SDFlash_AUX_CONTROL_DP0_REG (* (reg8 *) \
                                        SPIMaster_SDFlash_BSPIM_sR8_Dp_u0__DP_AUX_CTL_REG)
#define SPIMaster_SDFlash_AUX_CONTROL_DP0_PTR (  (reg8 *) \
                                        SPIMaster_SDFlash_BSPIM_sR8_Dp_u0__DP_AUX_CTL_REG)

#if(SPIMaster_SDFlash_USE_SECOND_DATAPATH)
    #define SPIMaster_SDFlash_AUX_CONTROL_DP1_REG  (* (reg8 *) \
                                        SPIMaster_SDFlash_BSPIM_sR8_Dp_u1__DP_AUX_CTL_REG)
    #define SPIMaster_SDFlash_AUX_CONTROL_DP1_PTR  (  (reg8 *) \
                                        SPIMaster_SDFlash_BSPIM_sR8_Dp_u1__DP_AUX_CTL_REG)
#endif /* (SPIMaster_SDFlash_USE_SECOND_DATAPATH) */

#define SPIMaster_SDFlash_COUNTER_PERIOD_REG     (* (reg8 *) SPIMaster_SDFlash_BSPIM_BitCounter__PERIOD_REG)
#define SPIMaster_SDFlash_COUNTER_PERIOD_PTR     (  (reg8 *) SPIMaster_SDFlash_BSPIM_BitCounter__PERIOD_REG)
#define SPIMaster_SDFlash_COUNTER_CONTROL_REG    (* (reg8 *) SPIMaster_SDFlash_BSPIM_BitCounter__CONTROL_AUX_CTL_REG)
#define SPIMaster_SDFlash_COUNTER_CONTROL_PTR    (  (reg8 *) SPIMaster_SDFlash_BSPIM_BitCounter__CONTROL_AUX_CTL_REG)

#define SPIMaster_SDFlash_TX_STATUS_REG          (* (reg8 *) SPIMaster_SDFlash_BSPIM_TxStsReg__STATUS_REG)
#define SPIMaster_SDFlash_TX_STATUS_PTR          (  (reg8 *) SPIMaster_SDFlash_BSPIM_TxStsReg__STATUS_REG)
#define SPIMaster_SDFlash_RX_STATUS_REG          (* (reg8 *) SPIMaster_SDFlash_BSPIM_RxStsReg__STATUS_REG)
#define SPIMaster_SDFlash_RX_STATUS_PTR          (  (reg8 *) SPIMaster_SDFlash_BSPIM_RxStsReg__STATUS_REG)

#define SPIMaster_SDFlash_CONTROL_REG            (* (reg8 *) \
                                      SPIMaster_SDFlash_BSPIM_BidirMode_CtrlReg__CONTROL_REG)
#define SPIMaster_SDFlash_CONTROL_PTR            (  (reg8 *) \
                                      SPIMaster_SDFlash_BSPIM_BidirMode_CtrlReg__CONTROL_REG)

#define SPIMaster_SDFlash_TX_STATUS_MASK_REG     (* (reg8 *) SPIMaster_SDFlash_BSPIM_TxStsReg__MASK_REG)
#define SPIMaster_SDFlash_TX_STATUS_MASK_PTR     (  (reg8 *) SPIMaster_SDFlash_BSPIM_TxStsReg__MASK_REG)
#define SPIMaster_SDFlash_RX_STATUS_MASK_REG     (* (reg8 *) SPIMaster_SDFlash_BSPIM_RxStsReg__MASK_REG)
#define SPIMaster_SDFlash_RX_STATUS_MASK_PTR     (  (reg8 *) SPIMaster_SDFlash_BSPIM_RxStsReg__MASK_REG)

#define SPIMaster_SDFlash_TX_STATUS_ACTL_REG     (* (reg8 *) SPIMaster_SDFlash_BSPIM_TxStsReg__STATUS_AUX_CTL_REG)
#define SPIMaster_SDFlash_TX_STATUS_ACTL_PTR     (  (reg8 *) SPIMaster_SDFlash_BSPIM_TxStsReg__STATUS_AUX_CTL_REG)
#define SPIMaster_SDFlash_RX_STATUS_ACTL_REG     (* (reg8 *) SPIMaster_SDFlash_BSPIM_RxStsReg__STATUS_AUX_CTL_REG)
#define SPIMaster_SDFlash_RX_STATUS_ACTL_PTR     (  (reg8 *) SPIMaster_SDFlash_BSPIM_RxStsReg__STATUS_AUX_CTL_REG)

#if(SPIMaster_SDFlash_USE_SECOND_DATAPATH)
    #define SPIMaster_SDFlash_AUX_CONTROLDP1     (SPIMaster_SDFlash_AUX_CONTROL_DP1_REG)
#endif /* (SPIMaster_SDFlash_USE_SECOND_DATAPATH) */


/***************************************
*       Register Constants
***************************************/

/* Status Register Definitions */
#define SPIMaster_SDFlash_STS_SPI_DONE_SHIFT             (0x00u)
#define SPIMaster_SDFlash_STS_TX_FIFO_EMPTY_SHIFT        (0x01u)
#define SPIMaster_SDFlash_STS_TX_FIFO_NOT_FULL_SHIFT     (0x02u)
#define SPIMaster_SDFlash_STS_BYTE_COMPLETE_SHIFT        (0x03u)
#define SPIMaster_SDFlash_STS_SPI_IDLE_SHIFT             (0x04u)
#define SPIMaster_SDFlash_STS_RX_FIFO_FULL_SHIFT         (0x04u)
#define SPIMaster_SDFlash_STS_RX_FIFO_NOT_EMPTY_SHIFT    (0x05u)
#define SPIMaster_SDFlash_STS_RX_FIFO_OVERRUN_SHIFT      (0x06u)

#define SPIMaster_SDFlash_STS_SPI_DONE           ((uint8) (0x01u << SPIMaster_SDFlash_STS_SPI_DONE_SHIFT))
#define SPIMaster_SDFlash_STS_TX_FIFO_EMPTY      ((uint8) (0x01u << SPIMaster_SDFlash_STS_TX_FIFO_EMPTY_SHIFT))
#define SPIMaster_SDFlash_STS_TX_FIFO_NOT_FULL   ((uint8) (0x01u << SPIMaster_SDFlash_STS_TX_FIFO_NOT_FULL_SHIFT))
#define SPIMaster_SDFlash_STS_BYTE_COMPLETE      ((uint8) (0x01u << SPIMaster_SDFlash_STS_BYTE_COMPLETE_SHIFT))
#define SPIMaster_SDFlash_STS_SPI_IDLE           ((uint8) (0x01u << SPIMaster_SDFlash_STS_SPI_IDLE_SHIFT))
#define SPIMaster_SDFlash_STS_RX_FIFO_FULL       ((uint8) (0x01u << SPIMaster_SDFlash_STS_RX_FIFO_FULL_SHIFT))
#define SPIMaster_SDFlash_STS_RX_FIFO_NOT_EMPTY  ((uint8) (0x01u << SPIMaster_SDFlash_STS_RX_FIFO_NOT_EMPTY_SHIFT))
#define SPIMaster_SDFlash_STS_RX_FIFO_OVERRUN    ((uint8) (0x01u << SPIMaster_SDFlash_STS_RX_FIFO_OVERRUN_SHIFT))

/* TX and RX masks for clear on read bits */
#define SPIMaster_SDFlash_TX_STS_CLR_ON_RD_BYTES_MASK    (0x09u)
#define SPIMaster_SDFlash_RX_STS_CLR_ON_RD_BYTES_MASK    (0x40u)

/* StatusI Register Interrupt Enable Control Bits */
/* As defined by the Register map for the AUX Control Register */
#define SPIMaster_SDFlash_INT_ENABLE     (0x10u) /* Enable interrupt from statusi */
#define SPIMaster_SDFlash_TX_FIFO_CLR    (0x01u) /* F0 - TX FIFO */
#define SPIMaster_SDFlash_RX_FIFO_CLR    (0x02u) /* F1 - RX FIFO */
#define SPIMaster_SDFlash_FIFO_CLR       (SPIMaster_SDFlash_TX_FIFO_CLR | SPIMaster_SDFlash_RX_FIFO_CLR)

/* Bit Counter (7-bit) Control Register Bit Definitions */
/* As defined by the Register map for the AUX Control Register */
#define SPIMaster_SDFlash_CNTR_ENABLE    (0x20u) /* Enable CNT7 */

/* Bi-Directional mode control bit */
#define SPIMaster_SDFlash_CTRL_TX_SIGNAL_EN  (0x01u)

/* Datapath Auxillary Control Register definitions */
#define SPIMaster_SDFlash_AUX_CTRL_FIFO0_CLR         (0x01u)
#define SPIMaster_SDFlash_AUX_CTRL_FIFO1_CLR         (0x02u)
#define SPIMaster_SDFlash_AUX_CTRL_FIFO0_LVL         (0x04u)
#define SPIMaster_SDFlash_AUX_CTRL_FIFO1_LVL         (0x08u)
#define SPIMaster_SDFlash_STATUS_ACTL_INT_EN_MASK    (0x10u)

/* Component disabled */
#define SPIMaster_SDFlash_DISABLED   (0u)


/***************************************
*       Macros
***************************************/

/* Returns true if componentn enabled */
#define SPIMaster_SDFlash_IS_ENABLED (0u != (SPIMaster_SDFlash_TX_STATUS_ACTL_REG & SPIMaster_SDFlash_INT_ENABLE))

/* Retuns TX status register */
#define SPIMaster_SDFlash_GET_STATUS_TX(swTxSts) ( (uint8)(SPIMaster_SDFlash_TX_STATUS_REG | \
                                                          ((swTxSts) & SPIMaster_SDFlash_TX_STS_CLR_ON_RD_BYTES_MASK)) )
/* Retuns RX status register */
#define SPIMaster_SDFlash_GET_STATUS_RX(swRxSts) ( (uint8)(SPIMaster_SDFlash_RX_STATUS_REG | \
                                                          ((swRxSts) & SPIMaster_SDFlash_RX_STS_CLR_ON_RD_BYTES_MASK)) )


/***************************************
* The following code is DEPRECATED and 
* should not be used in new projects.
***************************************/

#define SPIMaster_SDFlash_WriteByte   SPIMaster_SDFlash_WriteTxData
#define SPIMaster_SDFlash_ReadByte    SPIMaster_SDFlash_ReadRxData
void  SPIMaster_SDFlash_SetInterruptMode(uint8 intSrc)       ;
uint8 SPIMaster_SDFlash_ReadStatus(void)                     ;
void  SPIMaster_SDFlash_EnableInt(void)                      ;
void  SPIMaster_SDFlash_DisableInt(void)                     ;

#define SPIMaster_SDFlash_TXDATA                 (SPIMaster_SDFlash_TXDATA_REG)
#define SPIMaster_SDFlash_RXDATA                 (SPIMaster_SDFlash_RXDATA_REG)
#define SPIMaster_SDFlash_AUX_CONTROLDP0         (SPIMaster_SDFlash_AUX_CONTROL_DP0_REG)
#define SPIMaster_SDFlash_TXBUFFERREAD           (SPIMaster_SDFlash_txBufferRead)
#define SPIMaster_SDFlash_TXBUFFERWRITE          (SPIMaster_SDFlash_txBufferWrite)
#define SPIMaster_SDFlash_RXBUFFERREAD           (SPIMaster_SDFlash_rxBufferRead)
#define SPIMaster_SDFlash_RXBUFFERWRITE          (SPIMaster_SDFlash_rxBufferWrite)

#define SPIMaster_SDFlash_COUNTER_PERIOD         (SPIMaster_SDFlash_COUNTER_PERIOD_REG)
#define SPIMaster_SDFlash_COUNTER_CONTROL        (SPIMaster_SDFlash_COUNTER_CONTROL_REG)
#define SPIMaster_SDFlash_STATUS                 (SPIMaster_SDFlash_TX_STATUS_REG)
#define SPIMaster_SDFlash_CONTROL                (SPIMaster_SDFlash_CONTROL_REG)
#define SPIMaster_SDFlash_STATUS_MASK            (SPIMaster_SDFlash_TX_STATUS_MASK_REG)
#define SPIMaster_SDFlash_STATUS_ACTL            (SPIMaster_SDFlash_TX_STATUS_ACTL_REG)

#define SPIMaster_SDFlash_INIT_INTERRUPTS_MASK  (SPIMaster_SDFlash_INT_ON_SPI_DONE     | \
                                                SPIMaster_SDFlash_INT_ON_TX_EMPTY     | \
                                                SPIMaster_SDFlash_INT_ON_TX_NOT_FULL_DEF  | \
                                                SPIMaster_SDFlash_INT_ON_RX_FULL      | \
                                                SPIMaster_SDFlash_INT_ON_RX_NOT_EMPTY | \
                                                SPIMaster_SDFlash_INT_ON_RX_OVER      | \
                                                SPIMaster_SDFlash_INT_ON_BYTE_COMP)
                                                
#define SPIMaster_SDFlash_DataWidth                  (SPIMaster_SDFlash_DATA_WIDTH)
#define SPIMaster_SDFlash_InternalClockUsed          (SPIMaster_SDFlash_INTERNAL_CLOCK)
#define SPIMaster_SDFlash_InternalTxInterruptEnabled (SPIMaster_SDFlash_INTERNAL_TX_INT_ENABLED)
#define SPIMaster_SDFlash_InternalRxInterruptEnabled (SPIMaster_SDFlash_INTERNAL_RX_INT_ENABLED)
#define SPIMaster_SDFlash_ModeUseZero                (SPIMaster_SDFlash_MODE_USE_ZERO)
#define SPIMaster_SDFlash_BidirectionalMode          (SPIMaster_SDFlash_BIDIRECTIONAL_MODE)
#define SPIMaster_SDFlash_Mode                       (SPIMaster_SDFlash_MODE)
#define SPIMaster_SDFlash_DATAWIDHT                  (SPIMaster_SDFlash_DATA_WIDTH)
#define SPIMaster_SDFlash_InternalInterruptEnabled   (0u)

#define SPIMaster_SDFlash_TXBUFFERSIZE   (SPIMaster_SDFlash_TX_BUFFER_SIZE)
#define SPIMaster_SDFlash_RXBUFFERSIZE   (SPIMaster_SDFlash_RX_BUFFER_SIZE)

#define SPIMaster_SDFlash_TXBUFFER       SPIMaster_SDFlash_txBuffer
#define SPIMaster_SDFlash_RXBUFFER       SPIMaster_SDFlash_rxBuffer

#endif /* (CY_SPIM_SPIMaster_SDFlash_H) */


/* [] END OF FILE */
