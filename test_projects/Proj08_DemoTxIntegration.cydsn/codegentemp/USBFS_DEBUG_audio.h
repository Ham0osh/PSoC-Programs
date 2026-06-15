/***************************************************************************//**
* \file USBFS_DEBUG_audio.h
* \version 3.20
*
* \brief
*  This file provides function prototypes and constants for the USBFS component 
*  Audio class.
*
* Related Document:
*  Universal Serial Bus Device Class Definition for Audio Devices Release 1.0
*
********************************************************************************
* \copyright
* Copyright 2008-2016, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_USBFS_USBFS_DEBUG_audio_H)
#define CY_USBFS_USBFS_DEBUG_audio_H

#include "USBFS_DEBUG.h"


/***************************************
* Custom Declarations
***************************************/

/* `#START CUSTOM_CONSTANTS` Place your declaration here */

/* `#END` */


/***************************************
*  Constants for USBFS_DEBUG_audio API.
***************************************/

/* Audio Class-Specific Request Codes (AUDIO Table A-9) */
#define USBFS_DEBUG_REQUEST_CODE_UNDEFINED     (0x00u)
#define USBFS_DEBUG_SET_CUR                    (0x01u)
#define USBFS_DEBUG_GET_CUR                    (0x81u)
#define USBFS_DEBUG_SET_MIN                    (0x02u)
#define USBFS_DEBUG_GET_MIN                    (0x82u)
#define USBFS_DEBUG_SET_MAX                    (0x03u)
#define USBFS_DEBUG_GET_MAX                    (0x83u)
#define USBFS_DEBUG_SET_RES                    (0x04u)
#define USBFS_DEBUG_GET_RES                    (0x84u)
#define USBFS_DEBUG_SET_MEM                    (0x05u)
#define USBFS_DEBUG_GET_MEM                    (0x85u)
#define USBFS_DEBUG_GET_STAT                   (0xFFu)

/* point Control Selectors (AUDIO Table A-19) */
#define USBFS_DEBUG_EP_CONTROL_UNDEFINED       (0x00u)
#define USBFS_DEBUG_SAMPLING_FREQ_CONTROL      (0x01u)
#define USBFS_DEBUG_PITCH_CONTROL              (0x02u)

/* Feature Unit Control Selectors (AUDIO Table A-11) */
#define USBFS_DEBUG_FU_CONTROL_UNDEFINED       (0x00u)
#define USBFS_DEBUG_MUTE_CONTROL               (0x01u)
#define USBFS_DEBUG_VOLUME_CONTROL             (0x02u)
#define USBFS_DEBUG_BASS_CONTROL               (0x03u)
#define USBFS_DEBUG_MID_CONTROL                (0x04u)
#define USBFS_DEBUG_TREBLE_CONTROL             (0x05u)
#define USBFS_DEBUG_GRAPHIC_EQUALIZER_CONTROL  (0x06u)
#define USBFS_DEBUG_AUTOMATIC_GAIN_CONTROL     (0x07u)
#define USBFS_DEBUG_DELAY_CONTROL              (0x08u)
#define USBFS_DEBUG_BASS_BOOST_CONTROL         (0x09u)
#define USBFS_DEBUG_LOUDNESS_CONTROL           (0x0Au)

#define USBFS_DEBUG_SAMPLE_FREQ_LEN            (3u)
#define USBFS_DEBUG_VOLUME_LEN                 (2u)

#if !defined(USER_SUPPLIED_DEFAULT_VOLUME_VALUE)
    #define USBFS_DEBUG_VOL_MIN_MSB            (0x80u)
    #define USBFS_DEBUG_VOL_MIN_LSB            (0x01u)
    #define USBFS_DEBUG_VOL_MAX_MSB            (0x7Fu)
    #define USBFS_DEBUG_VOL_MAX_LSB            (0xFFu)
    #define USBFS_DEBUG_VOL_RES_MSB            (0x00u)
    #define USBFS_DEBUG_VOL_RES_LSB            (0x01u)
#endif /* USER_SUPPLIED_DEFAULT_VOLUME_VALUE */


/***************************************
* External data references
***************************************/
/**
* \addtogroup group_audio
* @{
*/
extern volatile uint8 USBFS_DEBUG_currentSampleFrequency[USBFS_DEBUG_MAX_EP][USBFS_DEBUG_SAMPLE_FREQ_LEN];
extern volatile uint8 USBFS_DEBUG_frequencyChanged;
extern volatile uint8 USBFS_DEBUG_currentMute;
extern volatile uint8 USBFS_DEBUG_currentVolume[USBFS_DEBUG_VOLUME_LEN];
/** @} audio */

extern volatile uint8 USBFS_DEBUG_minimumVolume[USBFS_DEBUG_VOLUME_LEN];
extern volatile uint8 USBFS_DEBUG_maximumVolume[USBFS_DEBUG_VOLUME_LEN];
extern volatile uint8 USBFS_DEBUG_resolutionVolume[USBFS_DEBUG_VOLUME_LEN];

#endif /*  CY_USBFS_USBFS_DEBUG_audio_H */


/* [] END OF FILE */
