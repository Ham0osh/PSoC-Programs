/***************************************************************************//**
* \file USBFS_DEBUG_cdc.c
* \version 3.20
*
* \brief
*  This file contains the USB MSC Class request handler and global API for MSC 
*  class.
*
* Related Document:
*  Universal Serial Bus Class Definitions for Communication Devices Version 1.1
*
********************************************************************************
* \copyright
* Copyright 2012-2016, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "USBFS_DEBUG_msc.h"
#include "USBFS_DEBUG_pvt.h"
#include "cyapicallbacks.h"

#if (USBFS_DEBUG_HANDLE_MSC_REQUESTS)

/***************************************
*          Internal variables
***************************************/

static uint8 USBFS_DEBUG_lunCount = USBFS_DEBUG_MSC_LUN_NUMBER;


/*******************************************************************************
* Function Name: USBFS_DEBUG_DispatchMSCClassRqst
****************************************************************************//**
*   
*  \internal 
*  This routine dispatches MSC class requests.
*
* \return
*  Status of request processing: handled or not handled.
*
* \globalvars
*  USBFS_DEBUG_lunCount - stores number of LUN (logical units).
*
* \reentrant
*  No.
*
*******************************************************************************/
uint8 USBFS_DEBUG_DispatchMSCClassRqst(void) 
{
    uint8 requestHandled = USBFS_DEBUG_FALSE;
    
    /* Get request data. */
    uint16 value  = USBFS_DEBUG_GET_UINT16(USBFS_DEBUG_wValueHiReg,  USBFS_DEBUG_wValueLoReg);
    uint16 dataLength = USBFS_DEBUG_GET_UINT16(USBFS_DEBUG_wLengthHiReg, USBFS_DEBUG_wLengthLoReg);
       
    /* Check request direction: D2H or H2D. */
    if (0u != (USBFS_DEBUG_bmRequestTypeReg & USBFS_DEBUG_RQST_DIR_D2H))
    {
        /* Handle direction from device to host. */
        
        if (USBFS_DEBUG_MSC_GET_MAX_LUN == USBFS_DEBUG_bRequestReg)
        {
            /* Check request fields. */
            if ((value  == USBFS_DEBUG_MSC_GET_MAX_LUN_WVALUE) &&
                (dataLength == USBFS_DEBUG_MSC_GET_MAX_LUN_WLENGTH))
            {
                /* Reply to Get Max LUN request: setup control read. */
                USBFS_DEBUG_currentTD.pData = &USBFS_DEBUG_lunCount;
                USBFS_DEBUG_currentTD.count =  USBFS_DEBUG_MSC_GET_MAX_LUN_WLENGTH;
                
                requestHandled  = USBFS_DEBUG_InitControlRead();
            }
        }
    }
    else
    {
        /* Handle direction from host to device. */
        
        if (USBFS_DEBUG_MSC_RESET == USBFS_DEBUG_bRequestReg)
        {
            /* Check request fields. */
            if ((value  == USBFS_DEBUG_MSC_RESET_WVALUE) &&
                (dataLength == USBFS_DEBUG_MSC_RESET_WLENGTH))
            {
                /* Handle to Bulk-Only Reset request: no data control transfer. */
                USBFS_DEBUG_currentTD.count = USBFS_DEBUG_MSC_RESET_WLENGTH;
                
            #ifdef USBFS_DEBUG_DISPATCH_MSC_CLASS_MSC_RESET_RQST_CALLBACK
                USBFS_DEBUG_DispatchMSCClass_MSC_RESET_RQST_Callback();
            #endif /* (USBFS_DEBUG_DISPATCH_MSC_CLASS_MSC_RESET_RQST_CALLBACK) */
                
                requestHandled = USBFS_DEBUG_InitNoDataControlTransfer();
            }
        }
    }
    
    return (requestHandled);
}


/*******************************************************************************
* Function Name: USBFS_DEBUG_MSC_SetLunCount
****************************************************************************//**
*
*  This function sets the number of logical units supported in the application. 
*  The default number of logical units is set in the component customizer.
*
*  \param lunCount: Count of the logical units. Valid range is between 1 and 16.
*
*
* \globalvars
*  USBFS_DEBUG_lunCount - stores number of LUN (logical units).
*
* \reentrant
*  No.
*
*******************************************************************************/
void USBFS_DEBUG_MSC_SetLunCount(uint8 lunCount) 
{
    USBFS_DEBUG_lunCount = (lunCount - 1u);
}


/*******************************************************************************
* Function Name: USBFS_DEBUG_MSC_GetLunCount
****************************************************************************//**
*
*  This function returns the number of logical units.
*
* \return
*   Number of the logical units.
*
* \globalvars
*  USBFS_DEBUG_lunCount - stores number of LUN (logical units).
*
* \reentrant
*  No.
*
*******************************************************************************/
uint8 USBFS_DEBUG_MSC_GetLunCount(void) 
{
    return (USBFS_DEBUG_lunCount + 1u);
}   

#endif /* (USBFS_DEBUG_HANDLE_MSC_REQUESTS) */


/* [] END OF FILE */
