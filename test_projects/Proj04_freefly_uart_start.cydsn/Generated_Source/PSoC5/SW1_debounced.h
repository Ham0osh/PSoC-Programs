/*******************************************************************************
* File Name: SW1_debounced.h  
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

#if !defined(CY_PINS_SW1_debounced_H) /* Pins SW1_debounced_H */
#define CY_PINS_SW1_debounced_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "SW1_debounced_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 SW1_debounced__PORT == 15 && ((SW1_debounced__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    SW1_debounced_Write(uint8 value);
void    SW1_debounced_SetDriveMode(uint8 mode);
uint8   SW1_debounced_ReadDataReg(void);
uint8   SW1_debounced_Read(void);
void    SW1_debounced_SetInterruptMode(uint16 position, uint16 mode);
uint8   SW1_debounced_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the SW1_debounced_SetDriveMode() function.
     *  @{
     */
        #define SW1_debounced_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define SW1_debounced_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define SW1_debounced_DM_RES_UP          PIN_DM_RES_UP
        #define SW1_debounced_DM_RES_DWN         PIN_DM_RES_DWN
        #define SW1_debounced_DM_OD_LO           PIN_DM_OD_LO
        #define SW1_debounced_DM_OD_HI           PIN_DM_OD_HI
        #define SW1_debounced_DM_STRONG          PIN_DM_STRONG
        #define SW1_debounced_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define SW1_debounced_MASK               SW1_debounced__MASK
#define SW1_debounced_SHIFT              SW1_debounced__SHIFT
#define SW1_debounced_WIDTH              1u

/* Interrupt constants */
#if defined(SW1_debounced__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in SW1_debounced_SetInterruptMode() function.
     *  @{
     */
        #define SW1_debounced_INTR_NONE      (uint16)(0x0000u)
        #define SW1_debounced_INTR_RISING    (uint16)(0x0001u)
        #define SW1_debounced_INTR_FALLING   (uint16)(0x0002u)
        #define SW1_debounced_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define SW1_debounced_INTR_MASK      (0x01u) 
#endif /* (SW1_debounced__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define SW1_debounced_PS                     (* (reg8 *) SW1_debounced__PS)
/* Data Register */
#define SW1_debounced_DR                     (* (reg8 *) SW1_debounced__DR)
/* Port Number */
#define SW1_debounced_PRT_NUM                (* (reg8 *) SW1_debounced__PRT) 
/* Connect to Analog Globals */                                                  
#define SW1_debounced_AG                     (* (reg8 *) SW1_debounced__AG)                       
/* Analog MUX bux enable */
#define SW1_debounced_AMUX                   (* (reg8 *) SW1_debounced__AMUX) 
/* Bidirectional Enable */                                                        
#define SW1_debounced_BIE                    (* (reg8 *) SW1_debounced__BIE)
/* Bit-mask for Aliased Register Access */
#define SW1_debounced_BIT_MASK               (* (reg8 *) SW1_debounced__BIT_MASK)
/* Bypass Enable */
#define SW1_debounced_BYP                    (* (reg8 *) SW1_debounced__BYP)
/* Port wide control signals */                                                   
#define SW1_debounced_CTL                    (* (reg8 *) SW1_debounced__CTL)
/* Drive Modes */
#define SW1_debounced_DM0                    (* (reg8 *) SW1_debounced__DM0) 
#define SW1_debounced_DM1                    (* (reg8 *) SW1_debounced__DM1)
#define SW1_debounced_DM2                    (* (reg8 *) SW1_debounced__DM2) 
/* Input Buffer Disable Override */
#define SW1_debounced_INP_DIS                (* (reg8 *) SW1_debounced__INP_DIS)
/* LCD Common or Segment Drive */
#define SW1_debounced_LCD_COM_SEG            (* (reg8 *) SW1_debounced__LCD_COM_SEG)
/* Enable Segment LCD */
#define SW1_debounced_LCD_EN                 (* (reg8 *) SW1_debounced__LCD_EN)
/* Slew Rate Control */
#define SW1_debounced_SLW                    (* (reg8 *) SW1_debounced__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define SW1_debounced_PRTDSI__CAPS_SEL       (* (reg8 *) SW1_debounced__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define SW1_debounced_PRTDSI__DBL_SYNC_IN    (* (reg8 *) SW1_debounced__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define SW1_debounced_PRTDSI__OE_SEL0        (* (reg8 *) SW1_debounced__PRTDSI__OE_SEL0) 
#define SW1_debounced_PRTDSI__OE_SEL1        (* (reg8 *) SW1_debounced__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define SW1_debounced_PRTDSI__OUT_SEL0       (* (reg8 *) SW1_debounced__PRTDSI__OUT_SEL0) 
#define SW1_debounced_PRTDSI__OUT_SEL1       (* (reg8 *) SW1_debounced__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define SW1_debounced_PRTDSI__SYNC_OUT       (* (reg8 *) SW1_debounced__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(SW1_debounced__SIO_CFG)
    #define SW1_debounced_SIO_HYST_EN        (* (reg8 *) SW1_debounced__SIO_HYST_EN)
    #define SW1_debounced_SIO_REG_HIFREQ     (* (reg8 *) SW1_debounced__SIO_REG_HIFREQ)
    #define SW1_debounced_SIO_CFG            (* (reg8 *) SW1_debounced__SIO_CFG)
    #define SW1_debounced_SIO_DIFF           (* (reg8 *) SW1_debounced__SIO_DIFF)
#endif /* (SW1_debounced__SIO_CFG) */

/* Interrupt Registers */
#if defined(SW1_debounced__INTSTAT)
    #define SW1_debounced_INTSTAT            (* (reg8 *) SW1_debounced__INTSTAT)
    #define SW1_debounced_SNAP               (* (reg8 *) SW1_debounced__SNAP)
    
	#define SW1_debounced_0_INTTYPE_REG 		(* (reg8 *) SW1_debounced__0__INTTYPE)
#endif /* (SW1_debounced__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_SW1_debounced_H */


/* [] END OF FILE */
