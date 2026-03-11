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
 * This is a simple instance of a ring buffer
 * to hold received Movi status packets.
 *
*/

#include "movi_rb.h"

static inline uint16_t rb_next(uint16_t i)
{
    i++;
    if (i >= MOVI_RB_SIZE) i = 0u;
    return i;
}

void movi_rb_init(movi_rb_t* rb)
{
    rb->head = 0u;
    rb->tail = 0u;
    rb->drops = 0u;
}

void movi_rb_clear(movi_rb_t* rb)
{
    rb->head = 0u;
    rb->tail = 0u;
}

uint16_t movi_rb_count(const movi_rb_t* rb)
{
    uint16_t h = rb->head;
    uint16_t t = rb->tail;
    return (h >= t) ? (h - t) : (uint16_t)(MOVI_RB_SIZE - (t - h));
}

// ISR uses this to empty FIFO from UART
uint8_t movi_rb_push(movi_rb_t* rb, uint8_t b)
{
    uint16_t next = rb_next(rb->head);
    if (next == rb->tail)
    {
        rb->drops++;
        return 0u;  // FULL!
    }
    rb->buf[rb->head] = b;
    rb->head = next;
    return 1u;
}

// Control loop uses this to get Rx from Movi
uint8_t movi_rb_pop(movi_rb_t* rb, uint8_t* b)
{
    if (rb->head == rb->tail) return 0u;

    *b = rb->buf[rb->tail];
    rb->tail = rb_next(rb->tail);
    return 1u;
}

/* [] END OF FILE */
