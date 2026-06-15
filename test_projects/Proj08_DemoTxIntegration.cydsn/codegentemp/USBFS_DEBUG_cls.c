/***************************************************************************//**
* \file USBFS_DEBUG_cls.c
* \version 3.20
*
* \brief
*  This file contains the USB Class request handler.
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

#if(USBFS_DEBUG_EXTERN_CLS == USBFS_DEBUG_FALSE)

/***************************************
* User Implemented Class Driver Declarations.
***************************************/
/* `#START USER_DEFINED_CLASS_DECLARATIONS` Place your declaration here */

/* `#END` */


/*******************************************************************************
* Function Name: USBFS_DEBUG_DispatchClassRqst
****************************************************************************//**
*  This routine dispatches class specific requests depend on interface class.
*
* \return
*  requestHandled.
*
* \reentrant
*  No.
*
*******************************************************************************/
uint8 USBFS_DEBUG_DispatchClassRqst(void) 
{
    uint8 interfaceNumber;
    uint8 requestHandled = USBFS_DEBUG_FALSE;

    /* Get interface to which request is intended. */
    switch (USBFS_DEBUG_bmRequestTypeReg & USBFS_DEBUG_RQST_RCPT_MASK)
    {
        case USBFS_DEBUG_RQST_RCPT_IFC:
            /* Class-specific request directed to interface: wIndexLoReg 
            * contains interface number.
            */
            interfaceNumber = (uint8) USBFS_DEBUG_wIndexLoReg;
            break;
        
        case USBFS_DEBUG_RQST_RCPT_EP:
            /* Class-specific request directed to endpoint: wIndexLoReg contains 
            * endpoint number. Find interface related to endpoint. 
            */
            interfaceNumber = USBFS_DEBUG_EP[USBFS_DEBUG_wIndexLoReg & USBFS_DEBUG_DIR_UNUSED].interface;
            break;
            
        default:
            /* Default interface is zero. */
            interfaceNumber = 0u;
            break;
    }
    
    /* Check that interface is within acceptable range */
    if (interfaceNumber <= USBFS_DEBUG_MAX_INTERFACES_NUMBER)
    {
    #if (defined(USBFS_DEBUG_ENABLE_HID_CLASS)   || \
         defined(USBFS_DEBUG_ENABLE_AUDIO_CLASS) || \
         defined(USBFS_DEBUG_ENABLE_CDC_CLASS)   || \
         USBFS_DEBUG_ENABLE_MSC_CLASS)

        /* Handle class request depends on interface type. */
        switch (USBFS_DEBUG_interfaceClass[interfaceNumber])
        {
        #if defined(USBFS_DEBUG_ENABLE_HID_CLASS)
            case USBFS_DEBUG_CLASS_HID:
                requestHandled = USBFS_DEBUG_DispatchHIDClassRqst();
                break;
        #endif /* (USBFS_DEBUG_ENABLE_HID_CLASS) */
                
        #if defined(USBFS_DEBUG_ENABLE_AUDIO_CLASS)
            case USBFS_DEBUG_CLASS_AUDIO:
                requestHandled = USBFS_DEBUG_DispatchAUDIOClassRqst();
                break;
        #endif /* (USBFS_DEBUG_CLASS_AUDIO) */
                
        #if defined(USBFS_DEBUG_ENABLE_CDC_CLASS)
            case USBFS_DEBUG_CLASS_CDC:
                requestHandled = USBFS_DEBUG_DispatchCDCClassRqst();
                break;
        #endif /* (USBFS_DEBUG_ENABLE_CDC_CLASS) */
            
        #if (USBFS_DEBUG_ENABLE_MSC_CLASS)
            case USBFS_DEBUG_CLASS_MSD:
            #if (USBFS_DEBUG_HANDLE_MSC_REQUESTS)
                /* MSC requests are handled by the component. */
                requestHandled = USBFS_DEBUG_DispatchMSCClassRqst();
            #elif defined(USBFS_DEBUG_DISPATCH_MSC_CLASS_RQST_CALLBACK)
                /* MSC requests are handled by user defined callbcak. */
                requestHandled = USBFS_DEBUG_DispatchMSCClassRqst_Callback();
            #else
                /* MSC requests are not handled. */
                requestHandled = USBFS_DEBUG_FALSE;
            #endif /* (USBFS_DEBUG_HANDLE_MSC_REQUESTS) */
                break;
        #endif /* (USBFS_DEBUG_ENABLE_MSC_CLASS) */
            
            default:
                /* Request is not handled: unknown class request type. */
                requestHandled = USBFS_DEBUG_FALSE;
                break;
        }
    #endif /* Class support is enabled */
    }
    
    /* `#START USER_DEFINED_CLASS_CODE` Place your Class request here */

    /* `#END` */

#ifdef USBFS_DEBUG_DISPATCH_CLASS_RQST_CALLBACK
    if (USBFS_DEBUG_FALSE == requestHandled)
    {
        requestHandled = USBFS_DEBUG_DispatchClassRqst_Callback(interfaceNumber);
    }
#endif /* (USBFS_DEBUG_DISPATCH_CLASS_RQST_CALLBACK) */

    return (requestHandled);
}


/*******************************************************************************
* Additional user functions supporting Class Specific Requests
********************************************************************************/

/* `#START CLASS_SPECIFIC_FUNCTIONS` Place any additional functions here */

/* `#END` */

#endif /* USBFS_DEBUG_EXTERN_CLS */


/* [] END OF FILE */
