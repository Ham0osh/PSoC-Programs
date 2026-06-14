/***************************************************************************//**
* \file .h
* \version 3.20
*
* \brief
*  This file provides private function prototypes and constants for the 
*  USBFS component. It is not intended to be used in the user project.
*
********************************************************************************
* \copyright
* Copyright 2013-2016, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_USBFS_USBFS_DEBUG_pvt_H)
#define CY_USBFS_USBFS_DEBUG_pvt_H

#include "USBFS_DEBUG.h"
   
#ifdef USBFS_DEBUG_ENABLE_AUDIO_CLASS
    #include "USBFS_DEBUG_audio.h"
#endif /* USBFS_DEBUG_ENABLE_AUDIO_CLASS */

#ifdef USBFS_DEBUG_ENABLE_CDC_CLASS
    #include "USBFS_DEBUG_cdc.h"
#endif /* USBFS_DEBUG_ENABLE_CDC_CLASS */

#if (USBFS_DEBUG_ENABLE_MIDI_CLASS)
    #include "USBFS_DEBUG_midi.h"
#endif /* (USBFS_DEBUG_ENABLE_MIDI_CLASS) */

#if (USBFS_DEBUG_ENABLE_MSC_CLASS)
    #include "USBFS_DEBUG_msc.h"
#endif /* (USBFS_DEBUG_ENABLE_MSC_CLASS) */

#if (USBFS_DEBUG_EP_MANAGEMENT_DMA)
    #if (CY_PSOC4)
        #include <CyDMA.h>
    #else
        #include <CyDmac.h>
        #if ((USBFS_DEBUG_EP_MANAGEMENT_DMA_AUTO) && (USBFS_DEBUG_EP_DMA_AUTO_OPT == 0u))
            #include "USBFS_DEBUG_EP_DMA_Done_isr.h"
            #include "USBFS_DEBUG_EP8_DMA_Done_SR.h"
            #include "USBFS_DEBUG_EP17_DMA_Done_SR.h"
        #endif /* ((USBFS_DEBUG_EP_MANAGEMENT_DMA_AUTO) && (USBFS_DEBUG_EP_DMA_AUTO_OPT == 0u)) */
    #endif /* (CY_PSOC4) */
#endif /* (USBFS_DEBUG_EP_MANAGEMENT_DMA) */

#if (USBFS_DEBUG_DMA1_ACTIVE)
    #include "USBFS_DEBUG_ep1_dma.h"
    #define USBFS_DEBUG_EP1_DMA_CH     (USBFS_DEBUG_ep1_dma_CHANNEL)
#endif /* (USBFS_DEBUG_DMA1_ACTIVE) */

#if (USBFS_DEBUG_DMA2_ACTIVE)
    #include "USBFS_DEBUG_ep2_dma.h"
    #define USBFS_DEBUG_EP2_DMA_CH     (USBFS_DEBUG_ep2_dma_CHANNEL)
#endif /* (USBFS_DEBUG_DMA2_ACTIVE) */

#if (USBFS_DEBUG_DMA3_ACTIVE)
    #include "USBFS_DEBUG_ep3_dma.h"
    #define USBFS_DEBUG_EP3_DMA_CH     (USBFS_DEBUG_ep3_dma_CHANNEL)
#endif /* (USBFS_DEBUG_DMA3_ACTIVE) */

#if (USBFS_DEBUG_DMA4_ACTIVE)
    #include "USBFS_DEBUG_ep4_dma.h"
    #define USBFS_DEBUG_EP4_DMA_CH     (USBFS_DEBUG_ep4_dma_CHANNEL)
#endif /* (USBFS_DEBUG_DMA4_ACTIVE) */

#if (USBFS_DEBUG_DMA5_ACTIVE)
    #include "USBFS_DEBUG_ep5_dma.h"
    #define USBFS_DEBUG_EP5_DMA_CH     (USBFS_DEBUG_ep5_dma_CHANNEL)
#endif /* (USBFS_DEBUG_DMA5_ACTIVE) */

#if (USBFS_DEBUG_DMA6_ACTIVE)
    #include "USBFS_DEBUG_ep6_dma.h"
    #define USBFS_DEBUG_EP6_DMA_CH     (USBFS_DEBUG_ep6_dma_CHANNEL)
#endif /* (USBFS_DEBUG_DMA6_ACTIVE) */

#if (USBFS_DEBUG_DMA7_ACTIVE)
    #include "USBFS_DEBUG_ep7_dma.h"
    #define USBFS_DEBUG_EP7_DMA_CH     (USBFS_DEBUG_ep7_dma_CHANNEL)
#endif /* (USBFS_DEBUG_DMA7_ACTIVE) */

#if (USBFS_DEBUG_DMA8_ACTIVE)
    #include "USBFS_DEBUG_ep8_dma.h"
    #define USBFS_DEBUG_EP8_DMA_CH     (USBFS_DEBUG_ep8_dma_CHANNEL)
#endif /* (USBFS_DEBUG_DMA8_ACTIVE) */


/***************************************
*     Private Variables
***************************************/

/* Generated external references for descriptors. */
extern const uint8 CYCODE USBFS_DEBUG_DEVICE0_DESCR[18u];
extern const uint8 CYCODE USBFS_DEBUG_DEVICE0_CONFIGURATION0_DESCR[75u];
extern const T_USBFS_DEBUG_EP_SETTINGS_BLOCK CYCODE USBFS_DEBUG_DEVICE0_CONFIGURATION0_EP_SETTINGS_TABLE[3u];
extern const uint8 CYCODE USBFS_DEBUG_DEVICE0_CONFIGURATION0_INTERFACE_CLASS[2u];
extern const T_USBFS_DEBUG_LUT CYCODE USBFS_DEBUG_DEVICE0_CONFIGURATION0_TABLE[5u];
extern const T_USBFS_DEBUG_LUT CYCODE USBFS_DEBUG_DEVICE0_TABLE[3u];
extern const T_USBFS_DEBUG_LUT CYCODE USBFS_DEBUG_TABLE[1u];
extern const uint8 CYCODE USBFS_DEBUG_SN_STRING_DESCRIPTOR[2];
extern const uint8 CYCODE USBFS_DEBUG_STRING_DESCRIPTORS[319u];


extern const uint8 CYCODE USBFS_DEBUG_MSOS_DESCRIPTOR[USBFS_DEBUG_MSOS_DESCRIPTOR_LENGTH];
extern const uint8 CYCODE USBFS_DEBUG_MSOS_CONFIGURATION_DESCR[USBFS_DEBUG_MSOS_CONF_DESCR_LENGTH];
#if defined(USBFS_DEBUG_ENABLE_IDSN_STRING)
    extern uint8 USBFS_DEBUG_idSerialNumberStringDescriptor[USBFS_DEBUG_IDSN_DESCR_LENGTH];
#endif /* (USBFS_DEBUG_ENABLE_IDSN_STRING) */

extern volatile uint8 USBFS_DEBUG_interfaceNumber;
extern volatile uint8 USBFS_DEBUG_interfaceSetting[USBFS_DEBUG_MAX_INTERFACES_NUMBER];
extern volatile uint8 USBFS_DEBUG_interfaceSettingLast[USBFS_DEBUG_MAX_INTERFACES_NUMBER];
extern volatile uint8 USBFS_DEBUG_deviceAddress;
extern volatile uint8 USBFS_DEBUG_interfaceStatus[USBFS_DEBUG_MAX_INTERFACES_NUMBER];
extern const uint8 CYCODE *USBFS_DEBUG_interfaceClass;

extern volatile T_USBFS_DEBUG_EP_CTL_BLOCK USBFS_DEBUG_EP[USBFS_DEBUG_MAX_EP];
extern volatile T_USBFS_DEBUG_TD USBFS_DEBUG_currentTD;

#if (USBFS_DEBUG_EP_MANAGEMENT_DMA)
    #if (CY_PSOC4)
        extern const uint8 USBFS_DEBUG_DmaChan[USBFS_DEBUG_MAX_EP];
    #else
        extern uint8 USBFS_DEBUG_DmaChan[USBFS_DEBUG_MAX_EP];
        extern uint8 USBFS_DEBUG_DmaTd  [USBFS_DEBUG_MAX_EP];
    #endif /* (CY_PSOC4) */
#endif /* (USBFS_DEBUG_EP_MANAGEMENT_DMA) */

#if (USBFS_DEBUG_EP_MANAGEMENT_DMA_AUTO)
#if (CY_PSOC4)
    extern uint8  USBFS_DEBUG_DmaEpBurstCnt   [USBFS_DEBUG_MAX_EP];
    extern uint8  USBFS_DEBUG_DmaEpLastBurstEl[USBFS_DEBUG_MAX_EP];

    extern uint8  USBFS_DEBUG_DmaEpBurstCntBackup  [USBFS_DEBUG_MAX_EP];
    extern uint32 USBFS_DEBUG_DmaEpBufferAddrBackup[USBFS_DEBUG_MAX_EP];
    
    extern const uint8 USBFS_DEBUG_DmaReqOut     [USBFS_DEBUG_MAX_EP];    
    extern const uint8 USBFS_DEBUG_DmaBurstEndOut[USBFS_DEBUG_MAX_EP];
#else
    #if (USBFS_DEBUG_EP_DMA_AUTO_OPT == 0u)
        extern uint8 USBFS_DEBUG_DmaNextTd[USBFS_DEBUG_MAX_EP];
        extern volatile uint16 USBFS_DEBUG_inLength [USBFS_DEBUG_MAX_EP];
        extern volatile uint8  USBFS_DEBUG_inBufFull[USBFS_DEBUG_MAX_EP];
        extern const uint8 USBFS_DEBUG_epX_TD_TERMOUT_EN[USBFS_DEBUG_MAX_EP];
        extern const uint8 *USBFS_DEBUG_inDataPointer[USBFS_DEBUG_MAX_EP];
    #endif /* (USBFS_DEBUG_EP_DMA_AUTO_OPT == 0u) */
#endif /* CY_PSOC4 */
#endif /* (USBFS_DEBUG_EP_MANAGEMENT_DMA_AUTO) */

extern volatile uint8 USBFS_DEBUG_ep0Toggle;
extern volatile uint8 USBFS_DEBUG_lastPacketSize;
extern volatile uint8 USBFS_DEBUG_ep0Mode;
extern volatile uint8 USBFS_DEBUG_ep0Count;
extern volatile uint16 USBFS_DEBUG_transferByteCount;


/***************************************
*     Private Function Prototypes
***************************************/
void  USBFS_DEBUG_ReInitComponent(void)            ;
void  USBFS_DEBUG_HandleSetup(void)                ;
void  USBFS_DEBUG_HandleIN(void)                   ;
void  USBFS_DEBUG_HandleOUT(void)                  ;
void  USBFS_DEBUG_LoadEP0(void)                    ;
uint8 USBFS_DEBUG_InitControlRead(void)            ;
uint8 USBFS_DEBUG_InitControlWrite(void)           ;
void  USBFS_DEBUG_ControlReadDataStage(void)       ;
void  USBFS_DEBUG_ControlReadStatusStage(void)     ;
void  USBFS_DEBUG_ControlReadPrematureStatus(void) ;
uint8 USBFS_DEBUG_InitControlWrite(void)           ;
uint8 USBFS_DEBUG_InitZeroLengthControlTransfer(void) ;
void  USBFS_DEBUG_ControlWriteDataStage(void)      ;
void  USBFS_DEBUG_ControlWriteStatusStage(void)    ;
void  USBFS_DEBUG_ControlWritePrematureStatus(void);
uint8 USBFS_DEBUG_InitNoDataControlTransfer(void)  ;
void  USBFS_DEBUG_NoDataControlStatusStage(void)   ;
void  USBFS_DEBUG_InitializeStatusBlock(void)      ;
void  USBFS_DEBUG_UpdateStatusBlock(uint8 completionCode) ;
uint8 USBFS_DEBUG_DispatchClassRqst(void)          ;

void USBFS_DEBUG_Config(uint8 clearAltSetting) ;
void USBFS_DEBUG_ConfigAltChanged(void)        ;
void USBFS_DEBUG_ConfigReg(void)               ;
void USBFS_DEBUG_EpStateInit(void)             ;


const T_USBFS_DEBUG_LUT CYCODE *USBFS_DEBUG_GetConfigTablePtr(uint8 confIndex);
const T_USBFS_DEBUG_LUT CYCODE *USBFS_DEBUG_GetDeviceTablePtr(void)           ;
#if (USBFS_DEBUG_BOS_ENABLE)
    const T_USBFS_DEBUG_LUT CYCODE *USBFS_DEBUG_GetBOSPtr(void)               ;
#endif /* (USBFS_DEBUG_BOS_ENABLE) */
const uint8 CYCODE *USBFS_DEBUG_GetInterfaceClassTablePtr(void)                    ;
uint8 USBFS_DEBUG_ClearEndpointHalt(void)                                          ;
uint8 USBFS_DEBUG_SetEndpointHalt(void)                                            ;
uint8 USBFS_DEBUG_ValidateAlternateSetting(void)                                   ;

void USBFS_DEBUG_SaveConfig(void)      ;
void USBFS_DEBUG_RestoreConfig(void)   ;

#if (CY_PSOC3 || CY_PSOC5LP)
    #if (USBFS_DEBUG_EP_MANAGEMENT_DMA_AUTO && (USBFS_DEBUG_EP_DMA_AUTO_OPT == 0u))
        void USBFS_DEBUG_LoadNextInEP(uint8 epNumber, uint8 mode)  ;
    #endif /* (USBFS_DEBUG_EP_MANAGEMENT_DMA_AUTO && (USBFS_DEBUG_EP_DMA_AUTO_OPT == 0u)) */
#endif /* (CY_PSOC3 || CY_PSOC5LP) */

#if defined(USBFS_DEBUG_ENABLE_IDSN_STRING)
    void USBFS_DEBUG_ReadDieID(uint8 descr[])  ;
#endif /* USBFS_DEBUG_ENABLE_IDSN_STRING */

#if defined(USBFS_DEBUG_ENABLE_HID_CLASS)
    uint8 USBFS_DEBUG_DispatchHIDClassRqst(void) ;
#endif /* (USBFS_DEBUG_ENABLE_HID_CLASS) */

#if defined(USBFS_DEBUG_ENABLE_AUDIO_CLASS)
    uint8 USBFS_DEBUG_DispatchAUDIOClassRqst(void) ;
#endif /* (USBFS_DEBUG_ENABLE_AUDIO_CLASS) */

#if defined(USBFS_DEBUG_ENABLE_CDC_CLASS)
    uint8 USBFS_DEBUG_DispatchCDCClassRqst(void) ;
#endif /* (USBFS_DEBUG_ENABLE_CDC_CLASS) */

#if (USBFS_DEBUG_ENABLE_MSC_CLASS)
    #if (USBFS_DEBUG_HANDLE_MSC_REQUESTS)
        uint8 USBFS_DEBUG_DispatchMSCClassRqst(void) ;
    #endif /* (USBFS_DEBUG_HANDLE_MSC_REQUESTS) */
#endif /* (USBFS_DEBUG_ENABLE_MSC_CLASS */

CY_ISR_PROTO(USBFS_DEBUG_EP_0_ISR);
CY_ISR_PROTO(USBFS_DEBUG_BUS_RESET_ISR);

#if (USBFS_DEBUG_SOF_ISR_ACTIVE)
    CY_ISR_PROTO(USBFS_DEBUG_SOF_ISR);
#endif /* (USBFS_DEBUG_SOF_ISR_ACTIVE) */

#if (USBFS_DEBUG_EP1_ISR_ACTIVE)
    CY_ISR_PROTO(USBFS_DEBUG_EP_1_ISR);
#endif /* (USBFS_DEBUG_EP1_ISR_ACTIVE) */

#if (USBFS_DEBUG_EP2_ISR_ACTIVE)
    CY_ISR_PROTO(USBFS_DEBUG_EP_2_ISR);
#endif /* (USBFS_DEBUG_EP2_ISR_ACTIVE) */

#if (USBFS_DEBUG_EP3_ISR_ACTIVE)
    CY_ISR_PROTO(USBFS_DEBUG_EP_3_ISR);
#endif /* (USBFS_DEBUG_EP3_ISR_ACTIVE) */

#if (USBFS_DEBUG_EP4_ISR_ACTIVE)
    CY_ISR_PROTO(USBFS_DEBUG_EP_4_ISR);
#endif /* (USBFS_DEBUG_EP4_ISR_ACTIVE) */

#if (USBFS_DEBUG_EP5_ISR_ACTIVE)
    CY_ISR_PROTO(USBFS_DEBUG_EP_5_ISR);
#endif /* (USBFS_DEBUG_EP5_ISR_ACTIVE) */

#if (USBFS_DEBUG_EP6_ISR_ACTIVE)
    CY_ISR_PROTO(USBFS_DEBUG_EP_6_ISR);
#endif /* (USBFS_DEBUG_EP6_ISR_ACTIVE) */

#if (USBFS_DEBUG_EP7_ISR_ACTIVE)
    CY_ISR_PROTO(USBFS_DEBUG_EP_7_ISR);
#endif /* (USBFS_DEBUG_EP7_ISR_ACTIVE) */

#if (USBFS_DEBUG_EP8_ISR_ACTIVE)
    CY_ISR_PROTO(USBFS_DEBUG_EP_8_ISR);
#endif /* (USBFS_DEBUG_EP8_ISR_ACTIVE) */

#if (USBFS_DEBUG_EP_MANAGEMENT_DMA)
    CY_ISR_PROTO(USBFS_DEBUG_ARB_ISR);
#endif /* (USBFS_DEBUG_EP_MANAGEMENT_DMA) */

#if (USBFS_DEBUG_DP_ISR_ACTIVE)
    CY_ISR_PROTO(USBFS_DEBUG_DP_ISR);
#endif /* (USBFS_DEBUG_DP_ISR_ACTIVE) */

#if (CY_PSOC4)
    CY_ISR_PROTO(USBFS_DEBUG_INTR_HI_ISR);
    CY_ISR_PROTO(USBFS_DEBUG_INTR_MED_ISR);
    CY_ISR_PROTO(USBFS_DEBUG_INTR_LO_ISR);
    #if (USBFS_DEBUG_LPM_ACTIVE)
        CY_ISR_PROTO(USBFS_DEBUG_LPM_ISR);
    #endif /* (USBFS_DEBUG_LPM_ACTIVE) */
#endif /* (CY_PSOC4) */

#if (USBFS_DEBUG_EP_MANAGEMENT_DMA_AUTO)
#if (CY_PSOC4)
    #if (USBFS_DEBUG_DMA1_ACTIVE)
        void USBFS_DEBUG_EP1_DMA_DONE_ISR(void);
    #endif /* (USBFS_DEBUG_DMA1_ACTIVE) */

    #if (USBFS_DEBUG_DMA2_ACTIVE)
        void USBFS_DEBUG_EP2_DMA_DONE_ISR(void);
    #endif /* (USBFS_DEBUG_DMA2_ACTIVE) */

    #if (USBFS_DEBUG_DMA3_ACTIVE)
        void USBFS_DEBUG_EP3_DMA_DONE_ISR(void);
    #endif /* (USBFS_DEBUG_DMA3_ACTIVE) */

    #if (USBFS_DEBUG_DMA4_ACTIVE)
        void USBFS_DEBUG_EP4_DMA_DONE_ISR(void);
    #endif /* (USBFS_DEBUG_DMA4_ACTIVE) */

    #if (USBFS_DEBUG_DMA5_ACTIVE)
        void USBFS_DEBUG_EP5_DMA_DONE_ISR(void);
    #endif /* (USBFS_DEBUG_DMA5_ACTIVE) */

    #if (USBFS_DEBUG_DMA6_ACTIVE)
        void USBFS_DEBUG_EP6_DMA_DONE_ISR(void);
    #endif /* (USBFS_DEBUG_DMA6_ACTIVE) */

    #if (USBFS_DEBUG_DMA7_ACTIVE)
        void USBFS_DEBUG_EP7_DMA_DONE_ISR(void);
    #endif /* (USBFS_DEBUG_DMA7_ACTIVE) */

    #if (USBFS_DEBUG_DMA8_ACTIVE)
        void USBFS_DEBUG_EP8_DMA_DONE_ISR(void);
    #endif /* (USBFS_DEBUG_DMA8_ACTIVE) */

#else
    #if (USBFS_DEBUG_EP_DMA_AUTO_OPT == 0u)
        CY_ISR_PROTO(USBFS_DEBUG_EP_DMA_DONE_ISR);
    #endif /* (USBFS_DEBUG_EP_DMA_AUTO_OPT == 0u) */
#endif /* (CY_PSOC4) */
#endif /* (USBFS_DEBUG_EP_MANAGEMENT_DMA_AUTO) */


/***************************************
*         Request Handlers
***************************************/

uint8 USBFS_DEBUG_HandleStandardRqst(void) ;
uint8 USBFS_DEBUG_DispatchClassRqst(void)  ;
uint8 USBFS_DEBUG_HandleVendorRqst(void)   ;


/***************************************
*    HID Internal references
***************************************/

#if defined(USBFS_DEBUG_ENABLE_HID_CLASS)
    void USBFS_DEBUG_FindReport(void)            ;
    void USBFS_DEBUG_FindReportDescriptor(void)  ;
    void USBFS_DEBUG_FindHidClassDecriptor(void) ;
#endif /* USBFS_DEBUG_ENABLE_HID_CLASS */


/***************************************
*    MIDI Internal references
***************************************/

#if defined(USBFS_DEBUG_ENABLE_MIDI_STREAMING)
    void USBFS_DEBUG_MIDI_IN_EP_Service(void)  ;
#endif /* (USBFS_DEBUG_ENABLE_MIDI_STREAMING) */


/***************************************
*    CDC Internal references
***************************************/

#if defined(USBFS_DEBUG_ENABLE_CDC_CLASS)

    typedef struct
    {
        uint8  bRequestType;
        uint8  bNotification;
        uint8  wValue;
        uint8  wValueMSB;
        uint8  wIndex;
        uint8  wIndexMSB;
        uint8  wLength;
        uint8  wLengthMSB;
        uint8  wSerialState;
        uint8  wSerialStateMSB;
    } t_USBFS_DEBUG_cdc_notification;

    uint8 USBFS_DEBUG_GetInterfaceComPort(uint8 interface) ;
    uint8 USBFS_DEBUG_Cdc_EpInit( const T_USBFS_DEBUG_EP_SETTINGS_BLOCK CYCODE *pEP, uint8 epNum, uint8 cdcComNums) ;

    extern volatile uint8  USBFS_DEBUG_cdc_dataInEpList[USBFS_DEBUG_MAX_MULTI_COM_NUM];
    extern volatile uint8  USBFS_DEBUG_cdc_dataOutEpList[USBFS_DEBUG_MAX_MULTI_COM_NUM];
    extern volatile uint8  USBFS_DEBUG_cdc_commInEpList[USBFS_DEBUG_MAX_MULTI_COM_NUM];
#endif /* (USBFS_DEBUG_ENABLE_CDC_CLASS) */


#endif /* CY_USBFS_USBFS_DEBUG_pvt_H */


/* [] END OF FILE */
