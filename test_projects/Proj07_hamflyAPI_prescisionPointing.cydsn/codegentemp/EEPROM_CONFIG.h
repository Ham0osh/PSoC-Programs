/*******************************************************************************
* File Name: EEPROM_CONFIG.h
* Version 3.0
*
*  Description:
*   Provides the function definitions for the EEPROM APIs.
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_EEPROM_EEPROM_CONFIG_H)
#define CY_EEPROM_EEPROM_CONFIG_H

#include "cydevice_trm.h"
#include "CyFlash.h"

#if !defined(CY_PSOC5LP)
    #error Component EEPROM_v3_0 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */


/***************************************
*        Function Prototypes
***************************************/

void EEPROM_CONFIG_Enable(void) ;
void EEPROM_CONFIG_Start(void) ;
void EEPROM_CONFIG_Stop (void) ;
cystatus EEPROM_CONFIG_WriteByte(uint8 dataByte, uint16 address) \
                                            ;
uint8 EEPROM_CONFIG_ReadByte(uint16 address) ;
uint8 EEPROM_CONFIG_UpdateTemperature(void) ;
cystatus EEPROM_CONFIG_EraseSector(uint8 sectorNumber) ;
cystatus EEPROM_CONFIG_Write(const uint8 * rowData, uint8 rowNumber) ;
cystatus EEPROM_CONFIG_StartWrite(const uint8 * rowData, uint8 rowNumber) \
                                                ;
cystatus EEPROM_CONFIG_StartErase(uint8 sectorNumber) ;
cystatus EEPROM_CONFIG_Query(void) ;
cystatus EEPROM_CONFIG_ByteWritePos(uint8 dataByte, uint8 rowNumber, uint8 byteNumber) \
                                                ;


/****************************************
*           API Constants
****************************************/

#define EEPROM_CONFIG_EEPROM_SIZE            CYDEV_EE_SIZE
#define EEPROM_CONFIG_SPC_BYTE_WRITE_SIZE    (0x01u)

#define EEPROM_CONFIG_SECTORS_NUMBER         (CYDEV_EE_SIZE / CYDEV_EEPROM_SECTOR_SIZE)

#define EEPROM_CONFIG_AHB_REQ_SHIFT          (0x00u)
#define EEPROM_CONFIG_AHB_REQ                ((uint8)(0x01u << EEPROM_CONFIG_AHB_REQ_SHIFT))
#define EEPROM_CONFIG_AHB_ACK_SHIFT          (0x01u)
#define EEPROM_CONFIG_AHB_ACK_MASK           ((uint8)(0x01u << EEPROM_CONFIG_AHB_ACK_SHIFT))


/***************************************
* Registers
***************************************/
#define EEPROM_CONFIG_SPC_EE_SCR_REG                 (*(reg8 *) CYREG_SPC_EE_SCR)
#define EEPROM_CONFIG_SPC_EE_SCR_PTR                 ( (reg8 *) CYREG_SPC_EE_SCR)



/***************************************
* The following code is DEPRECATED and
* should not be used in new projects.
***************************************/
#define EEPROM_CONFIG_ByteWrite                  EEPROM_CONFIG_ByteWritePos
#define EEPROM_CONFIG_QueryWrite                 EEPROM_CONFIG_Query

#endif /* CY_EEPROM_EEPROM_CONFIG_H */

/* [] END OF FILE */
