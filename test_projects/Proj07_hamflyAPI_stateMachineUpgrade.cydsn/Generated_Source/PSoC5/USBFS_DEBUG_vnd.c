/***************************************************************************//**
* \file USBFS_DEBUG_vnd.c
* \version 3.20
*
* \brief
*  This file contains the  USB vendor request handler.
*
********************************************************************************
* \copyright
* Copyright 2008-2016, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "USBFS_DEBUG_pvt.h"
#include "cyapicallbacks.h"

#if(USBFS_DEBUG_EXTERN_VND == USBFS_DEBUG_FALSE)

/***************************************
* Vendor Specific Declarations
***************************************/

/* `#START VENDOR_SPECIFIC_DECLARATIONS` Place your declaration here */

/* `#END` */


/*******************************************************************************
* Function Name: USBFS_DEBUG_HandleVendorRqst
****************************************************************************//**
*
*  This routine provide users with a method to implement vendor specific
*  requests.
*
*  To implement vendor specific requests, add your code in this function to
*  decode and disposition the request.  If the request is handled, your code
*  must set the variable "requestHandled" to TRUE, indicating that the
*  request has been handled.
*
* \return
*  requestHandled.
*
* \reentrant
*  No.
*
*******************************************************************************/
uint8 USBFS_DEBUG_HandleVendorRqst(void) 
{
    uint8 requestHandled = USBFS_DEBUG_FALSE;

    /* Check request direction: D2H or H2D. */
    if (0u != (USBFS_DEBUG_bmRequestTypeReg & USBFS_DEBUG_RQST_DIR_D2H))
    {
        /* Handle direction from device to host. */
        
        switch (USBFS_DEBUG_bRequestReg)
        {
            case USBFS_DEBUG_GET_EXTENDED_CONFIG_DESCRIPTOR:
            #if defined(USBFS_DEBUG_ENABLE_MSOS_STRING)
                USBFS_DEBUG_currentTD.pData = (volatile uint8 *) &USBFS_DEBUG_MSOS_CONFIGURATION_DESCR[0u];
                USBFS_DEBUG_currentTD.count = USBFS_DEBUG_MSOS_CONFIGURATION_DESCR[0u];
                requestHandled  = USBFS_DEBUG_InitControlRead();
            #endif /* (USBFS_DEBUG_ENABLE_MSOS_STRING) */
                break;
            
            default:
                break;
        }
    }

    /* `#START VENDOR_SPECIFIC_CODE` Place your vendor specific request here */

    /* `#END` */

#ifdef USBFS_DEBUG_HANDLE_VENDOR_RQST_CALLBACK
    if (USBFS_DEBUG_FALSE == requestHandled)
    {
        requestHandled = USBFS_DEBUG_HandleVendorRqst_Callback();
    }
#endif /* (USBFS_DEBUG_HANDLE_VENDOR_RQST_CALLBACK) */

    return (requestHandled);
}


/*******************************************************************************
* Additional user functions supporting Vendor Specific Requests
********************************************************************************/

/* `#START VENDOR_SPECIFIC_FUNCTIONS` Place any additional functions here */

/* `#END` */


#endif /* USBFS_DEBUG_EXTERN_VND */


/* [] END OF FILE */
