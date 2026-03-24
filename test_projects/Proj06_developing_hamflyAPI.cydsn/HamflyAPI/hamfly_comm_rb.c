/* hamfly_rb.c
 * ============================================================
 * Copyright Hamish Johnson, 2026
 * Quantum Internet Systems Lab, SFU Physics
 * ============================================================
 *
 * ISR-safe RX ring buffer. Renamed from movi_rb.c.
 * ISR pushes via hamfly_rb_push().
 * Main loop pops via hamfly_rb_pop().
 */

#include "hamfly_comm_rb.h"

static inline uint16_t rb_next(uint16_t i)
{
    i++;
    if (i >= HAMFLY_RB_SIZE) i = 0u;
    return i;
}

void hamfly_rb_init(hamfly_rb_t *rb)
{
    rb->head  = 0u;
    rb->tail  = 0u;
    rb->drops = 0u;
}

void hamfly_rb_clear(hamfly_rb_t *rb)
{
    rb->head = 0u;
    rb->tail = 0u;
}

uint16_t hamfly_rb_count(const hamfly_rb_t *rb)
{
    uint16_t h = rb->head;
    uint16_t t = rb->tail;
    return (h >= t) ? (h - t) : (uint16_t)(HAMFLY_RB_SIZE - (t - h));
}

uint8_t hamfly_rb_push(hamfly_rb_t *rb, uint8_t b)
{
    uint16_t next = rb_next(rb->head);
    if (next == rb->tail) {
        rb->drops++;
        return 0u;
    }
    rb->buf[rb->head] = b;
    rb->head = next;
    return 1u;
}

uint8_t hamfly_rb_pop(hamfly_rb_t *rb, uint8_t *b)
{
    if (rb->head == rb->tail) return 0u;
    *b       = rb->buf[rb->tail];
    rb->tail = rb_next(rb->tail);
    return 1u;
}
