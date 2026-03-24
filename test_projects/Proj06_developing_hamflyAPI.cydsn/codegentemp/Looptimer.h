/*******************************************************************************
* File Name: Looptimer.h
* Version 2.80
*
*  Description:
*     Contains the function prototypes and constants available to the timer
*     user module.
*
*   Note:
*     None
*
********************************************************************************
* Copyright 2008-2017, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
********************************************************************************/

#if !defined(CY_TIMER_Looptimer_H)
#define CY_TIMER_Looptimer_H

#include "cytypes.h"
#include "cyfitter.h"
#include "CyLib.h" /* For CyEnterCriticalSection() and CyExitCriticalSection() functions */

extern uint8 Looptimer_initVar;

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component Timer_v2_80 requires cy_boot v3.0 or later
#endif /* (CY_ PSOC5LP) */


/**************************************
*           Parameter Defaults
**************************************/

#define Looptimer_Resolution                 16u
#define Looptimer_UsingFixedFunction         1u
#define Looptimer_UsingHWCaptureCounter      0u
#define Looptimer_SoftwareCaptureMode        0u
#define Looptimer_SoftwareTriggerMode        0u
#define Looptimer_UsingHWEnable              0u
#define Looptimer_EnableTriggerMode          0u
#define Looptimer_InterruptOnCaptureCount    0u
#define Looptimer_RunModeUsed                0u
#define Looptimer_ControlRegRemoved          0u

#if defined(Looptimer_TimerUDB_sCTRLReg_SyncCtl_ctrlreg__CONTROL_REG)
    #define Looptimer_UDB_CONTROL_REG_REMOVED            (0u)
#elif  (Looptimer_UsingFixedFunction)
    #define Looptimer_UDB_CONTROL_REG_REMOVED            (0u)
#else 
    #define Looptimer_UDB_CONTROL_REG_REMOVED            (1u)
#endif /* End Looptimer_TimerUDB_sCTRLReg_SyncCtl_ctrlreg__CONTROL_REG */


/***************************************
*       Type defines
***************************************/


/**************************************************************************
 * Sleep Wakeup Backup structure for Timer Component
 *************************************************************************/
typedef struct
{
    uint8 TimerEnableState;
    #if(!Looptimer_UsingFixedFunction)

        uint16 TimerUdb;
        uint8 InterruptMaskValue;
        #if (Looptimer_UsingHWCaptureCounter)
            uint8 TimerCaptureCounter;
        #endif /* variable declarations for backing up non retention registers in CY_UDB_V1 */

        #if (!Looptimer_UDB_CONTROL_REG_REMOVED)
            uint8 TimerControlRegister;
        #endif /* variable declaration for backing up enable state of the Timer */
    #endif /* define backup variables only for UDB implementation. Fixed function registers are all retention */

}Looptimer_backupStruct;


/***************************************
*       Function Prototypes
***************************************/

void    Looptimer_Start(void) ;
void    Looptimer_Stop(void) ;

void    Looptimer_SetInterruptMode(uint8 interruptMode) ;
uint8   Looptimer_ReadStatusRegister(void) ;
/* Deprecated function. Do not use this in future. Retained for backward compatibility */
#define Looptimer_GetInterruptSource() Looptimer_ReadStatusRegister()

#if(!Looptimer_UDB_CONTROL_REG_REMOVED)
    uint8   Looptimer_ReadControlRegister(void) ;
    void    Looptimer_WriteControlRegister(uint8 control) ;
#endif /* (!Looptimer_UDB_CONTROL_REG_REMOVED) */

uint16  Looptimer_ReadPeriod(void) ;
void    Looptimer_WritePeriod(uint16 period) ;
uint16  Looptimer_ReadCounter(void) ;
void    Looptimer_WriteCounter(uint16 counter) ;
uint16  Looptimer_ReadCapture(void) ;
void    Looptimer_SoftwareCapture(void) ;

#if(!Looptimer_UsingFixedFunction) /* UDB Prototypes */
    #if (Looptimer_SoftwareCaptureMode)
        void    Looptimer_SetCaptureMode(uint8 captureMode) ;
    #endif /* (!Looptimer_UsingFixedFunction) */

    #if (Looptimer_SoftwareTriggerMode)
        void    Looptimer_SetTriggerMode(uint8 triggerMode) ;
    #endif /* (Looptimer_SoftwareTriggerMode) */

    #if (Looptimer_EnableTriggerMode)
        void    Looptimer_EnableTrigger(void) ;
        void    Looptimer_DisableTrigger(void) ;
    #endif /* (Looptimer_EnableTriggerMode) */


    #if(Looptimer_InterruptOnCaptureCount)
        void    Looptimer_SetInterruptCount(uint8 interruptCount) ;
    #endif /* (Looptimer_InterruptOnCaptureCount) */

    #if (Looptimer_UsingHWCaptureCounter)
        void    Looptimer_SetCaptureCount(uint8 captureCount) ;
        uint8   Looptimer_ReadCaptureCount(void) ;
    #endif /* (Looptimer_UsingHWCaptureCounter) */

    void Looptimer_ClearFIFO(void) ;
#endif /* UDB Prototypes */

/* Sleep Retention APIs */
void Looptimer_Init(void)          ;
void Looptimer_Enable(void)        ;
void Looptimer_SaveConfig(void)    ;
void Looptimer_RestoreConfig(void) ;
void Looptimer_Sleep(void)         ;
void Looptimer_Wakeup(void)        ;


/***************************************
*   Enumerated Types and Parameters
***************************************/

/* Enumerated Type B_Timer__CaptureModes, Used in Capture Mode */
#define Looptimer__B_TIMER__CM_NONE 0
#define Looptimer__B_TIMER__CM_RISINGEDGE 1
#define Looptimer__B_TIMER__CM_FALLINGEDGE 2
#define Looptimer__B_TIMER__CM_EITHEREDGE 3
#define Looptimer__B_TIMER__CM_SOFTWARE 4



/* Enumerated Type B_Timer__TriggerModes, Used in Trigger Mode */
#define Looptimer__B_TIMER__TM_NONE 0x00u
#define Looptimer__B_TIMER__TM_RISINGEDGE 0x04u
#define Looptimer__B_TIMER__TM_FALLINGEDGE 0x08u
#define Looptimer__B_TIMER__TM_EITHEREDGE 0x0Cu
#define Looptimer__B_TIMER__TM_SOFTWARE 0x10u


/***************************************
*    Initialial Parameter Constants
***************************************/

#define Looptimer_INIT_PERIOD             23999u
#define Looptimer_INIT_CAPTURE_MODE       ((uint8)((uint8)1u << Looptimer_CTRL_CAP_MODE_SHIFT))
#define Looptimer_INIT_TRIGGER_MODE       ((uint8)((uint8)0u << Looptimer_CTRL_TRIG_MODE_SHIFT))
#if (Looptimer_UsingFixedFunction)
    #define Looptimer_INIT_INTERRUPT_MODE (((uint8)((uint8)1u << Looptimer_STATUS_TC_INT_MASK_SHIFT)) | \
                                                  ((uint8)((uint8)0 << Looptimer_STATUS_CAPTURE_INT_MASK_SHIFT)))
#else
    #define Looptimer_INIT_INTERRUPT_MODE (((uint8)((uint8)1u << Looptimer_STATUS_TC_INT_MASK_SHIFT)) | \
                                                 ((uint8)((uint8)0 << Looptimer_STATUS_CAPTURE_INT_MASK_SHIFT)) | \
                                                 ((uint8)((uint8)0 << Looptimer_STATUS_FIFOFULL_INT_MASK_SHIFT)))
#endif /* (Looptimer_UsingFixedFunction) */
#define Looptimer_INIT_CAPTURE_COUNT      (2u)
#define Looptimer_INIT_INT_CAPTURE_COUNT  ((uint8)((uint8)(1u - 1u) << Looptimer_CTRL_INTCNT_SHIFT))


/***************************************
*           Registers
***************************************/

#if (Looptimer_UsingFixedFunction) /* Implementation Specific Registers and Register Constants */


    /***************************************
    *    Fixed Function Registers
    ***************************************/

    #define Looptimer_STATUS         (*(reg8 *) Looptimer_TimerHW__SR0 )
    /* In Fixed Function Block Status and Mask are the same register */
    #define Looptimer_STATUS_MASK    (*(reg8 *) Looptimer_TimerHW__SR0 )
    #define Looptimer_CONTROL        (*(reg8 *) Looptimer_TimerHW__CFG0)
    #define Looptimer_CONTROL2       (*(reg8 *) Looptimer_TimerHW__CFG1)
    #define Looptimer_CONTROL2_PTR   ( (reg8 *) Looptimer_TimerHW__CFG1)
    #define Looptimer_RT1            (*(reg8 *) Looptimer_TimerHW__RT1)
    #define Looptimer_RT1_PTR        ( (reg8 *) Looptimer_TimerHW__RT1)

    #if (CY_PSOC3 || CY_PSOC5LP)
        #define Looptimer_CONTROL3       (*(reg8 *) Looptimer_TimerHW__CFG2)
        #define Looptimer_CONTROL3_PTR   ( (reg8 *) Looptimer_TimerHW__CFG2)
    #endif /* (CY_PSOC3 || CY_PSOC5LP) */
    #define Looptimer_GLOBAL_ENABLE  (*(reg8 *) Looptimer_TimerHW__PM_ACT_CFG)
    #define Looptimer_GLOBAL_STBY_ENABLE  (*(reg8 *) Looptimer_TimerHW__PM_STBY_CFG)

    #define Looptimer_CAPTURE_LSB         (* (reg16 *) Looptimer_TimerHW__CAP0 )
    #define Looptimer_CAPTURE_LSB_PTR       ((reg16 *) Looptimer_TimerHW__CAP0 )
    #define Looptimer_PERIOD_LSB          (* (reg16 *) Looptimer_TimerHW__PER0 )
    #define Looptimer_PERIOD_LSB_PTR        ((reg16 *) Looptimer_TimerHW__PER0 )
    #define Looptimer_COUNTER_LSB         (* (reg16 *) Looptimer_TimerHW__CNT_CMP0 )
    #define Looptimer_COUNTER_LSB_PTR       ((reg16 *) Looptimer_TimerHW__CNT_CMP0 )


    /***************************************
    *    Register Constants
    ***************************************/

    /* Fixed Function Block Chosen */
    #define Looptimer_BLOCK_EN_MASK                     Looptimer_TimerHW__PM_ACT_MSK
    #define Looptimer_BLOCK_STBY_EN_MASK                Looptimer_TimerHW__PM_STBY_MSK

    /* Control Register Bit Locations */
    /* Interrupt Count - Not valid for Fixed Function Block */
    #define Looptimer_CTRL_INTCNT_SHIFT                  0x00u
    /* Trigger Polarity - Not valid for Fixed Function Block */
    #define Looptimer_CTRL_TRIG_MODE_SHIFT               0x00u
    /* Trigger Enable - Not valid for Fixed Function Block */
    #define Looptimer_CTRL_TRIG_EN_SHIFT                 0x00u
    /* Capture Polarity - Not valid for Fixed Function Block */
    #define Looptimer_CTRL_CAP_MODE_SHIFT                0x00u
    /* Timer Enable - As defined in Register Map, part of TMRX_CFG0 register */
    #define Looptimer_CTRL_ENABLE_SHIFT                  0x00u

    /* Control Register Bit Masks */
    #define Looptimer_CTRL_ENABLE                        ((uint8)((uint8)0x01u << Looptimer_CTRL_ENABLE_SHIFT))

    /* Control2 Register Bit Masks */
    /* As defined in Register Map, Part of the TMRX_CFG1 register */
    #define Looptimer_CTRL2_IRQ_SEL_SHIFT                 0x00u
    #define Looptimer_CTRL2_IRQ_SEL                      ((uint8)((uint8)0x01u << Looptimer_CTRL2_IRQ_SEL_SHIFT))

    #if (CY_PSOC5A)
        /* Use CFG1 Mode bits to set run mode */
        /* As defined by Verilog Implementation */
        #define Looptimer_CTRL_MODE_SHIFT                 0x01u
        #define Looptimer_CTRL_MODE_MASK                 ((uint8)((uint8)0x07u << Looptimer_CTRL_MODE_SHIFT))
    #endif /* (CY_PSOC5A) */
    #if (CY_PSOC3 || CY_PSOC5LP)
        /* Control3 Register Bit Locations */
        #define Looptimer_CTRL_RCOD_SHIFT        0x02u
        #define Looptimer_CTRL_ENBL_SHIFT        0x00u
        #define Looptimer_CTRL_MODE_SHIFT        0x00u

        /* Control3 Register Bit Masks */
        #define Looptimer_CTRL_RCOD_MASK  ((uint8)((uint8)0x03u << Looptimer_CTRL_RCOD_SHIFT)) /* ROD and COD bit masks */
        #define Looptimer_CTRL_ENBL_MASK  ((uint8)((uint8)0x80u << Looptimer_CTRL_ENBL_SHIFT)) /* HW_EN bit mask */
        #define Looptimer_CTRL_MODE_MASK  ((uint8)((uint8)0x03u << Looptimer_CTRL_MODE_SHIFT)) /* Run mode bit mask */

        #define Looptimer_CTRL_RCOD       ((uint8)((uint8)0x03u << Looptimer_CTRL_RCOD_SHIFT))
        #define Looptimer_CTRL_ENBL       ((uint8)((uint8)0x80u << Looptimer_CTRL_ENBL_SHIFT))
    #endif /* (CY_PSOC3 || CY_PSOC5LP) */

    /*RT1 Synch Constants: Applicable for PSoC3 and PSoC5LP */
    #define Looptimer_RT1_SHIFT                       0x04u
    /* Sync TC and CMP bit masks */
    #define Looptimer_RT1_MASK                        ((uint8)((uint8)0x03u << Looptimer_RT1_SHIFT))
    #define Looptimer_SYNC                            ((uint8)((uint8)0x03u << Looptimer_RT1_SHIFT))
    #define Looptimer_SYNCDSI_SHIFT                   0x00u
    /* Sync all DSI inputs with Mask  */
    #define Looptimer_SYNCDSI_MASK                    ((uint8)((uint8)0x0Fu << Looptimer_SYNCDSI_SHIFT))
    /* Sync all DSI inputs */
    #define Looptimer_SYNCDSI_EN                      ((uint8)((uint8)0x0Fu << Looptimer_SYNCDSI_SHIFT))

    #define Looptimer_CTRL_MODE_PULSEWIDTH            ((uint8)((uint8)0x01u << Looptimer_CTRL_MODE_SHIFT))
    #define Looptimer_CTRL_MODE_PERIOD                ((uint8)((uint8)0x02u << Looptimer_CTRL_MODE_SHIFT))
    #define Looptimer_CTRL_MODE_CONTINUOUS            ((uint8)((uint8)0x00u << Looptimer_CTRL_MODE_SHIFT))

    /* Status Register Bit Locations */
    /* As defined in Register Map, part of TMRX_SR0 register */
    #define Looptimer_STATUS_TC_SHIFT                 0x07u
    /* As defined in Register Map, part of TMRX_SR0 register, Shared with Compare Status */
    #define Looptimer_STATUS_CAPTURE_SHIFT            0x06u
    /* As defined in Register Map, part of TMRX_SR0 register */
    #define Looptimer_STATUS_TC_INT_MASK_SHIFT        (Looptimer_STATUS_TC_SHIFT - 0x04u)
    /* As defined in Register Map, part of TMRX_SR0 register, Shared with Compare Status */
    #define Looptimer_STATUS_CAPTURE_INT_MASK_SHIFT   (Looptimer_STATUS_CAPTURE_SHIFT - 0x04u)

    /* Status Register Bit Masks */
    #define Looptimer_STATUS_TC                       ((uint8)((uint8)0x01u << Looptimer_STATUS_TC_SHIFT))
    #define Looptimer_STATUS_CAPTURE                  ((uint8)((uint8)0x01u << Looptimer_STATUS_CAPTURE_SHIFT))
    /* Interrupt Enable Bit-Mask for interrupt on TC */
    #define Looptimer_STATUS_TC_INT_MASK              ((uint8)((uint8)0x01u << Looptimer_STATUS_TC_INT_MASK_SHIFT))
    /* Interrupt Enable Bit-Mask for interrupt on Capture */
    #define Looptimer_STATUS_CAPTURE_INT_MASK         ((uint8)((uint8)0x01u << Looptimer_STATUS_CAPTURE_INT_MASK_SHIFT))

#else   /* UDB Registers and Register Constants */


    /***************************************
    *           UDB Registers
    ***************************************/

    #define Looptimer_STATUS              (* (reg8 *) Looptimer_TimerUDB_rstSts_stsreg__STATUS_REG )
    #define Looptimer_STATUS_MASK         (* (reg8 *) Looptimer_TimerUDB_rstSts_stsreg__MASK_REG)
    #define Looptimer_STATUS_AUX_CTRL     (* (reg8 *) Looptimer_TimerUDB_rstSts_stsreg__STATUS_AUX_CTL_REG)
    #define Looptimer_CONTROL             (* (reg8 *) Looptimer_TimerUDB_sCTRLReg_SyncCtl_ctrlreg__CONTROL_REG )
    
    #if(Looptimer_Resolution <= 8u) /* 8-bit Timer */
        #define Looptimer_CAPTURE_LSB         (* (reg8 *) Looptimer_TimerUDB_sT16_timerdp_u0__F0_REG )
        #define Looptimer_CAPTURE_LSB_PTR       ((reg8 *) Looptimer_TimerUDB_sT16_timerdp_u0__F0_REG )
        #define Looptimer_PERIOD_LSB          (* (reg8 *) Looptimer_TimerUDB_sT16_timerdp_u0__D0_REG )
        #define Looptimer_PERIOD_LSB_PTR        ((reg8 *) Looptimer_TimerUDB_sT16_timerdp_u0__D0_REG )
        #define Looptimer_COUNTER_LSB         (* (reg8 *) Looptimer_TimerUDB_sT16_timerdp_u0__A0_REG )
        #define Looptimer_COUNTER_LSB_PTR       ((reg8 *) Looptimer_TimerUDB_sT16_timerdp_u0__A0_REG )
    #elif(Looptimer_Resolution <= 16u) /* 8-bit Timer */
        #if(CY_PSOC3) /* 8-bit addres space */
            #define Looptimer_CAPTURE_LSB         (* (reg16 *) Looptimer_TimerUDB_sT16_timerdp_u0__F0_REG )
            #define Looptimer_CAPTURE_LSB_PTR       ((reg16 *) Looptimer_TimerUDB_sT16_timerdp_u0__F0_REG )
            #define Looptimer_PERIOD_LSB          (* (reg16 *) Looptimer_TimerUDB_sT16_timerdp_u0__D0_REG )
            #define Looptimer_PERIOD_LSB_PTR        ((reg16 *) Looptimer_TimerUDB_sT16_timerdp_u0__D0_REG )
            #define Looptimer_COUNTER_LSB         (* (reg16 *) Looptimer_TimerUDB_sT16_timerdp_u0__A0_REG )
            #define Looptimer_COUNTER_LSB_PTR       ((reg16 *) Looptimer_TimerUDB_sT16_timerdp_u0__A0_REG )
        #else /* 16-bit address space */
            #define Looptimer_CAPTURE_LSB         (* (reg16 *) Looptimer_TimerUDB_sT16_timerdp_u0__16BIT_F0_REG )
            #define Looptimer_CAPTURE_LSB_PTR       ((reg16 *) Looptimer_TimerUDB_sT16_timerdp_u0__16BIT_F0_REG )
            #define Looptimer_PERIOD_LSB          (* (reg16 *) Looptimer_TimerUDB_sT16_timerdp_u0__16BIT_D0_REG )
            #define Looptimer_PERIOD_LSB_PTR        ((reg16 *) Looptimer_TimerUDB_sT16_timerdp_u0__16BIT_D0_REG )
            #define Looptimer_COUNTER_LSB         (* (reg16 *) Looptimer_TimerUDB_sT16_timerdp_u0__16BIT_A0_REG )
            #define Looptimer_COUNTER_LSB_PTR       ((reg16 *) Looptimer_TimerUDB_sT16_timerdp_u0__16BIT_A0_REG )
        #endif /* CY_PSOC3 */
    #elif(Looptimer_Resolution <= 24u)/* 24-bit Timer */
        #define Looptimer_CAPTURE_LSB         (* (reg32 *) Looptimer_TimerUDB_sT16_timerdp_u0__F0_REG )
        #define Looptimer_CAPTURE_LSB_PTR       ((reg32 *) Looptimer_TimerUDB_sT16_timerdp_u0__F0_REG )
        #define Looptimer_PERIOD_LSB          (* (reg32 *) Looptimer_TimerUDB_sT16_timerdp_u0__D0_REG )
        #define Looptimer_PERIOD_LSB_PTR        ((reg32 *) Looptimer_TimerUDB_sT16_timerdp_u0__D0_REG )
        #define Looptimer_COUNTER_LSB         (* (reg32 *) Looptimer_TimerUDB_sT16_timerdp_u0__A0_REG )
        #define Looptimer_COUNTER_LSB_PTR       ((reg32 *) Looptimer_TimerUDB_sT16_timerdp_u0__A0_REG )
    #else /* 32-bit Timer */
        #if(CY_PSOC3 || CY_PSOC5) /* 8-bit address space */
            #define Looptimer_CAPTURE_LSB         (* (reg32 *) Looptimer_TimerUDB_sT16_timerdp_u0__F0_REG )
            #define Looptimer_CAPTURE_LSB_PTR       ((reg32 *) Looptimer_TimerUDB_sT16_timerdp_u0__F0_REG )
            #define Looptimer_PERIOD_LSB          (* (reg32 *) Looptimer_TimerUDB_sT16_timerdp_u0__D0_REG )
            #define Looptimer_PERIOD_LSB_PTR        ((reg32 *) Looptimer_TimerUDB_sT16_timerdp_u0__D0_REG )
            #define Looptimer_COUNTER_LSB         (* (reg32 *) Looptimer_TimerUDB_sT16_timerdp_u0__A0_REG )
            #define Looptimer_COUNTER_LSB_PTR       ((reg32 *) Looptimer_TimerUDB_sT16_timerdp_u0__A0_REG )
        #else /* 32-bit address space */
            #define Looptimer_CAPTURE_LSB         (* (reg32 *) Looptimer_TimerUDB_sT16_timerdp_u0__32BIT_F0_REG )
            #define Looptimer_CAPTURE_LSB_PTR       ((reg32 *) Looptimer_TimerUDB_sT16_timerdp_u0__32BIT_F0_REG )
            #define Looptimer_PERIOD_LSB          (* (reg32 *) Looptimer_TimerUDB_sT16_timerdp_u0__32BIT_D0_REG )
            #define Looptimer_PERIOD_LSB_PTR        ((reg32 *) Looptimer_TimerUDB_sT16_timerdp_u0__32BIT_D0_REG )
            #define Looptimer_COUNTER_LSB         (* (reg32 *) Looptimer_TimerUDB_sT16_timerdp_u0__32BIT_A0_REG )
            #define Looptimer_COUNTER_LSB_PTR       ((reg32 *) Looptimer_TimerUDB_sT16_timerdp_u0__32BIT_A0_REG )
        #endif /* CY_PSOC3 || CY_PSOC5 */ 
    #endif

    #define Looptimer_COUNTER_LSB_PTR_8BIT       ((reg8 *) Looptimer_TimerUDB_sT16_timerdp_u0__A0_REG )
    
    #if (Looptimer_UsingHWCaptureCounter)
        #define Looptimer_CAP_COUNT              (*(reg8 *) Looptimer_TimerUDB_sCapCount_counter__PERIOD_REG )
        #define Looptimer_CAP_COUNT_PTR          ( (reg8 *) Looptimer_TimerUDB_sCapCount_counter__PERIOD_REG )
        #define Looptimer_CAPTURE_COUNT_CTRL     (*(reg8 *) Looptimer_TimerUDB_sCapCount_counter__CONTROL_AUX_CTL_REG )
        #define Looptimer_CAPTURE_COUNT_CTRL_PTR ( (reg8 *) Looptimer_TimerUDB_sCapCount_counter__CONTROL_AUX_CTL_REG )
    #endif /* (Looptimer_UsingHWCaptureCounter) */


    /***************************************
    *       Register Constants
    ***************************************/

    /* Control Register Bit Locations */
    #define Looptimer_CTRL_INTCNT_SHIFT              0x00u       /* As defined by Verilog Implementation */
    #define Looptimer_CTRL_TRIG_MODE_SHIFT           0x02u       /* As defined by Verilog Implementation */
    #define Looptimer_CTRL_TRIG_EN_SHIFT             0x04u       /* As defined by Verilog Implementation */
    #define Looptimer_CTRL_CAP_MODE_SHIFT            0x05u       /* As defined by Verilog Implementation */
    #define Looptimer_CTRL_ENABLE_SHIFT              0x07u       /* As defined by Verilog Implementation */

    /* Control Register Bit Masks */
    #define Looptimer_CTRL_INTCNT_MASK               ((uint8)((uint8)0x03u << Looptimer_CTRL_INTCNT_SHIFT))
    #define Looptimer_CTRL_TRIG_MODE_MASK            ((uint8)((uint8)0x03u << Looptimer_CTRL_TRIG_MODE_SHIFT))
    #define Looptimer_CTRL_TRIG_EN                   ((uint8)((uint8)0x01u << Looptimer_CTRL_TRIG_EN_SHIFT))
    #define Looptimer_CTRL_CAP_MODE_MASK             ((uint8)((uint8)0x03u << Looptimer_CTRL_CAP_MODE_SHIFT))
    #define Looptimer_CTRL_ENABLE                    ((uint8)((uint8)0x01u << Looptimer_CTRL_ENABLE_SHIFT))

    /* Bit Counter (7-bit) Control Register Bit Definitions */
    /* As defined by the Register map for the AUX Control Register */
    #define Looptimer_CNTR_ENABLE                    0x20u

    /* Status Register Bit Locations */
    #define Looptimer_STATUS_TC_SHIFT                0x00u  /* As defined by Verilog Implementation */
    #define Looptimer_STATUS_CAPTURE_SHIFT           0x01u  /* As defined by Verilog Implementation */
    #define Looptimer_STATUS_TC_INT_MASK_SHIFT       Looptimer_STATUS_TC_SHIFT
    #define Looptimer_STATUS_CAPTURE_INT_MASK_SHIFT  Looptimer_STATUS_CAPTURE_SHIFT
    #define Looptimer_STATUS_FIFOFULL_SHIFT          0x02u  /* As defined by Verilog Implementation */
    #define Looptimer_STATUS_FIFONEMP_SHIFT          0x03u  /* As defined by Verilog Implementation */
    #define Looptimer_STATUS_FIFOFULL_INT_MASK_SHIFT Looptimer_STATUS_FIFOFULL_SHIFT

    /* Status Register Bit Masks */
    /* Sticky TC Event Bit-Mask */
    #define Looptimer_STATUS_TC                      ((uint8)((uint8)0x01u << Looptimer_STATUS_TC_SHIFT))
    /* Sticky Capture Event Bit-Mask */
    #define Looptimer_STATUS_CAPTURE                 ((uint8)((uint8)0x01u << Looptimer_STATUS_CAPTURE_SHIFT))
    /* Interrupt Enable Bit-Mask */
    #define Looptimer_STATUS_TC_INT_MASK             ((uint8)((uint8)0x01u << Looptimer_STATUS_TC_SHIFT))
    /* Interrupt Enable Bit-Mask */
    #define Looptimer_STATUS_CAPTURE_INT_MASK        ((uint8)((uint8)0x01u << Looptimer_STATUS_CAPTURE_SHIFT))
    /* NOT-Sticky FIFO Full Bit-Mask */
    #define Looptimer_STATUS_FIFOFULL                ((uint8)((uint8)0x01u << Looptimer_STATUS_FIFOFULL_SHIFT))
    /* NOT-Sticky FIFO Not Empty Bit-Mask */
    #define Looptimer_STATUS_FIFONEMP                ((uint8)((uint8)0x01u << Looptimer_STATUS_FIFONEMP_SHIFT))
    /* Interrupt Enable Bit-Mask */
    #define Looptimer_STATUS_FIFOFULL_INT_MASK       ((uint8)((uint8)0x01u << Looptimer_STATUS_FIFOFULL_SHIFT))

    #define Looptimer_STATUS_ACTL_INT_EN             0x10u   /* As defined for the ACTL Register */

    /* Datapath Auxillary Control Register definitions */
    #define Looptimer_AUX_CTRL_FIFO0_CLR             0x01u   /* As defined by Register map */
    #define Looptimer_AUX_CTRL_FIFO1_CLR             0x02u   /* As defined by Register map */
    #define Looptimer_AUX_CTRL_FIFO0_LVL             0x04u   /* As defined by Register map */
    #define Looptimer_AUX_CTRL_FIFO1_LVL             0x08u   /* As defined by Register map */
    #define Looptimer_STATUS_ACTL_INT_EN_MASK        0x10u   /* As defined for the ACTL Register */

#endif /* Implementation Specific Registers and Register Constants */

#endif  /* CY_TIMER_Looptimer_H */


/* [] END OF FILE */
