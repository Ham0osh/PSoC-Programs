/*******************************************************************************
* File Name: Looptimer_PM.c
* Version 2.80
*
*  Description:
*     This file provides the power management source code to API for the
*     Timer.
*
*   Note:
*     None
*
*******************************************************************************
* Copyright 2008-2017, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
********************************************************************************/

#include "Looptimer.h"

static Looptimer_backupStruct Looptimer_backup;


/*******************************************************************************
* Function Name: Looptimer_SaveConfig
********************************************************************************
*
* Summary:
*     Save the current user configuration
*
* Parameters:
*  void
*
* Return:
*  void
*
* Global variables:
*  Looptimer_backup:  Variables of this global structure are modified to
*  store the values of non retention configuration registers when Sleep() API is
*  called.
*
*******************************************************************************/
void Looptimer_SaveConfig(void) 
{
    #if (!Looptimer_UsingFixedFunction)
        Looptimer_backup.TimerUdb = Looptimer_ReadCounter();
        Looptimer_backup.InterruptMaskValue = Looptimer_STATUS_MASK;
        #if (Looptimer_UsingHWCaptureCounter)
            Looptimer_backup.TimerCaptureCounter = Looptimer_ReadCaptureCount();
        #endif /* Back Up capture counter register  */

        #if(!Looptimer_UDB_CONTROL_REG_REMOVED)
            Looptimer_backup.TimerControlRegister = Looptimer_ReadControlRegister();
        #endif /* Backup the enable state of the Timer component */
    #endif /* Backup non retention registers in UDB implementation. All fixed function registers are retention */
}


/*******************************************************************************
* Function Name: Looptimer_RestoreConfig
********************************************************************************
*
* Summary:
*  Restores the current user configuration.
*
* Parameters:
*  void
*
* Return:
*  void
*
* Global variables:
*  Looptimer_backup:  Variables of this global structure are used to
*  restore the values of non retention registers on wakeup from sleep mode.
*
*******************************************************************************/
void Looptimer_RestoreConfig(void) 
{   
    #if (!Looptimer_UsingFixedFunction)

        Looptimer_WriteCounter(Looptimer_backup.TimerUdb);
        Looptimer_STATUS_MASK =Looptimer_backup.InterruptMaskValue;
        #if (Looptimer_UsingHWCaptureCounter)
            Looptimer_SetCaptureCount(Looptimer_backup.TimerCaptureCounter);
        #endif /* Restore Capture counter register*/

        #if(!Looptimer_UDB_CONTROL_REG_REMOVED)
            Looptimer_WriteControlRegister(Looptimer_backup.TimerControlRegister);
        #endif /* Restore the enable state of the Timer component */
    #endif /* Restore non retention registers in the UDB implementation only */
}


/*******************************************************************************
* Function Name: Looptimer_Sleep
********************************************************************************
*
* Summary:
*     Stop and Save the user configuration
*
* Parameters:
*  void
*
* Return:
*  void
*
* Global variables:
*  Looptimer_backup.TimerEnableState:  Is modified depending on the
*  enable state of the block before entering sleep mode.
*
*******************************************************************************/
void Looptimer_Sleep(void) 
{
    #if(!Looptimer_UDB_CONTROL_REG_REMOVED)
        /* Save Counter's enable state */
        if(Looptimer_CTRL_ENABLE == (Looptimer_CONTROL & Looptimer_CTRL_ENABLE))
        {
            /* Timer is enabled */
            Looptimer_backup.TimerEnableState = 1u;
        }
        else
        {
            /* Timer is disabled */
            Looptimer_backup.TimerEnableState = 0u;
        }
    #endif /* Back up enable state from the Timer control register */
    Looptimer_Stop();
    Looptimer_SaveConfig();
}


/*******************************************************************************
* Function Name: Looptimer_Wakeup
********************************************************************************
*
* Summary:
*  Restores and enables the user configuration
*
* Parameters:
*  void
*
* Return:
*  void
*
* Global variables:
*  Looptimer_backup.enableState:  Is used to restore the enable state of
*  block on wakeup from sleep mode.
*
*******************************************************************************/
void Looptimer_Wakeup(void) 
{
    Looptimer_RestoreConfig();
    #if(!Looptimer_UDB_CONTROL_REG_REMOVED)
        if(Looptimer_backup.TimerEnableState == 1u)
        {     /* Enable Timer's operation */
                Looptimer_Enable();
        } /* Do nothing if Timer was disabled before */
    #endif /* Remove this code section if Control register is removed */
}


/* [] END OF FILE */
