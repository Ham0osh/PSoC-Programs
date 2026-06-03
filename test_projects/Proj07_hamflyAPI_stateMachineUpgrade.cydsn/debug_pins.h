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
    do { Pin_Debug_CoarseCentroid_Write(1u); Pin_Debug_CoarseCentroid_Write(0u); } while (0)

#endif /* DEBUG_PINS_H */
/* [] END OF FILE */