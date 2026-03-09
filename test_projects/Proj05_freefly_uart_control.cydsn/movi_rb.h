/* ========================================
 *
 * Copyright Hamish Johnson 2026
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF QISL, Dept. of Phys, SFU.
 *
 * ========================================
*/

#ifndef MOVI_RB_H
#define MOVI_RB_H

#include <stdint.h>

// Allocate size for rb
#ifndef MOVI_RB_SIZE
#define MOVI_RB_SIZE 256u
#endif

typedef struct {
    volatile uint8_t  buf[MOVI_RB_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t drops;
} movi_rb_t;

// Primitives
void     movi_rb_init(movi_rb_t* rb);
void     movi_rb_clear(movi_rb_t* rb);
uint16_t movi_rb_count(const movi_rb_t* rb);

// ISR will produce bytes onto the buffer.
uint8_t  movi_rb_push(movi_rb_t* rb, uint8_t b);

// Within main, code will consume from buffer.
uint8_t  movi_rb_pop (movi_rb_t* rb, uint8_t* b);

#endif /* MOVI_RB_H */
/* [] END OF FILE */
