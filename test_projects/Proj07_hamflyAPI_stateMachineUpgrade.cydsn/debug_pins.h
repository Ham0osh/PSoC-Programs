/* ========================================
 *
 * Copyright Hamish Johnson, 2026
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF 
 * Quantum Internet Systems Lab (QISL),
 * Department of Physics, SFU, Canada.
 *
 * Author: Hamish Johnson (2026)
 *
 * ========================================
 *
 * Pin definitions and macros for digital pin toggling.
 * This is used for scope debugging.
*/

#ifndef DEBUG_PINS_H
#define DEBUG_PINS_H

#include <project.h>

// Superloo, 5050 squarw wave at half the frequency.
#define DPIN_LOOP_TOGGLE() \
    Pin_Debug_SuperLoop_Write(Pin_Debug_SuperLoop_Read() ^ 1u)

// Pulse at each coarse frame received.
#define DPIN_CENTROID() \
    Pin_Debug_CoarseCentroid_Write(Pin_Debug_CoarseCentroid_Read() ^ 1u)

// Triggers on Rx of a Movi byte
#define DPIN_MOVI_RX_TOGGLE()\
    Pin_Debug_MOVI_RX_Write(Pin_Debug_MOVI_RX_Read() ^ 1u)
   
#endif /* DEBUG_PINS_H */
/* [] END OF FILE */