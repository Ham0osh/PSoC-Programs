/*******************************************************************************
* File Name: ModeSwitch.h  
* Version 2.20
*
* Description:
*  This file contains Pin function prototypes and register defines
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_ModeSwitch_H) /* Pins ModeSwitch_H */
#define CY_PINS_ModeSwitch_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "ModeSwitch_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 ModeSwitch__PORT == 15 && ((ModeSwitch__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    ModeSwitch_Write(uint8 value);
void    ModeSwitch_SetDriveMode(uint8 mode);
uint8   ModeSwitch_ReadDataReg(void);
uint8   ModeSwitch_Read(void);
void    ModeSwitch_SetInterruptMode(uint16 position, uint16 mode);
uint8   ModeSwitch_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the ModeSwitch_SetDriveMode() function.
     *  @{
     */
        #define ModeSwitch_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define ModeSwitch_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define ModeSwitch_DM_RES_UP          PIN_DM_RES_UP
        #define ModeSwitch_DM_RES_DWN         PIN_DM_RES_DWN
        #define ModeSwitch_DM_OD_LO           PIN_DM_OD_LO
        #define ModeSwitch_DM_OD_HI           PIN_DM_OD_HI
        #define ModeSwitch_DM_STRONG          PIN_DM_STRONG
        #define ModeSwitch_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define ModeSwitch_MASK               ModeSwitch__MASK
#define ModeSwitch_SHIFT              ModeSwitch__SHIFT
#define ModeSwitch_WIDTH              1u

/* Interrupt constants */
#if defined(ModeSwitch__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in ModeSwitch_SetInterruptMode() function.
     *  @{
     */
        #define ModeSwitch_INTR_NONE      (uint16)(0x0000u)
        #define ModeSwitch_INTR_RISING    (uint16)(0x0001u)
        #define ModeSwitch_INTR_FALLING   (uint16)(0x0002u)
        #define ModeSwitch_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define ModeSwitch_INTR_MASK      (0x01u) 
#endif /* (ModeSwitch__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define ModeSwitch_PS                     (* (reg8 *) ModeSwitch__PS)
/* Data Register */
#define ModeSwitch_DR                     (* (reg8 *) ModeSwitch__DR)
/* Port Number */
#define ModeSwitch_PRT_NUM                (* (reg8 *) ModeSwitch__PRT) 
/* Connect to Analog Globals */                                                  
#define ModeSwitch_AG                     (* (reg8 *) ModeSwitch__AG)                       
/* Analog MUX bux enable */
#define ModeSwitch_AMUX                   (* (reg8 *) ModeSwitch__AMUX) 
/* Bidirectional Enable */                                                        
#define ModeSwitch_BIE                    (* (reg8 *) ModeSwitch__BIE)
/* Bit-mask for Aliased Register Access */
#define ModeSwitch_BIT_MASK               (* (reg8 *) ModeSwitch__BIT_MASK)
/* Bypass Enable */
#define ModeSwitch_BYP                    (* (reg8 *) ModeSwitch__BYP)
/* Port wide control signals */                                                   
#define ModeSwitch_CTL                    (* (reg8 *) ModeSwitch__CTL)
/* Drive Modes */
#define ModeSwitch_DM0                    (* (reg8 *) ModeSwitch__DM0) 
#define ModeSwitch_DM1                    (* (reg8 *) ModeSwitch__DM1)
#define ModeSwitch_DM2                    (* (reg8 *) ModeSwitch__DM2) 
/* Input Buffer Disable Override */
#define ModeSwitch_INP_DIS                (* (reg8 *) ModeSwitch__INP_DIS)
/* LCD Common or Segment Drive */
#define ModeSwitch_LCD_COM_SEG            (* (reg8 *) ModeSwitch__LCD_COM_SEG)
/* Enable Segment LCD */
#define ModeSwitch_LCD_EN                 (* (reg8 *) ModeSwitch__LCD_EN)
/* Slew Rate Control */
#define ModeSwitch_SLW                    (* (reg8 *) ModeSwitch__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define ModeSwitch_PRTDSI__CAPS_SEL       (* (reg8 *) ModeSwitch__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define ModeSwitch_PRTDSI__DBL_SYNC_IN    (* (reg8 *) ModeSwitch__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define ModeSwitch_PRTDSI__OE_SEL0        (* (reg8 *) ModeSwitch__PRTDSI__OE_SEL0) 
#define ModeSwitch_PRTDSI__OE_SEL1        (* (reg8 *) ModeSwitch__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define ModeSwitch_PRTDSI__OUT_SEL0       (* (reg8 *) ModeSwitch__PRTDSI__OUT_SEL0) 
#define ModeSwitch_PRTDSI__OUT_SEL1       (* (reg8 *) ModeSwitch__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define ModeSwitch_PRTDSI__SYNC_OUT       (* (reg8 *) ModeSwitch__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(ModeSwitch__SIO_CFG)
    #define ModeSwitch_SIO_HYST_EN        (* (reg8 *) ModeSwitch__SIO_HYST_EN)
    #define ModeSwitch_SIO_REG_HIFREQ     (* (reg8 *) ModeSwitch__SIO_REG_HIFREQ)
    #define ModeSwitch_SIO_CFG            (* (reg8 *) ModeSwitch__SIO_CFG)
    #define ModeSwitch_SIO_DIFF           (* (reg8 *) ModeSwitch__SIO_DIFF)
#endif /* (ModeSwitch__SIO_CFG) */

/* Interrupt Registers */
#if defined(ModeSwitch__INTSTAT)
    #define ModeSwitch_INTSTAT            (* (reg8 *) ModeSwitch__INTSTAT)
    #define ModeSwitch_SNAP               (* (reg8 *) ModeSwitch__SNAP)
    
	#define ModeSwitch_0_INTTYPE_REG 		(* (reg8 *) ModeSwitch__0__INTTYPE)
#endif /* (ModeSwitch__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_ModeSwitch_H */


/* [] END OF FILE */
