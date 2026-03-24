/* hamfly_rb.h
 * ============================================================
 * Copyright Hamish Johnson, 2026
 * Quantum Internet Systems Lab, SFU Physics
 * ============================================================
 *
 * ISR-safe RX ring buffer.
 * ISR pushes bytes in via hamfly_rb_push().
 * hamfly_pump() pops bytes out to QX_StreamRxCharSM.
 */

#ifndef HAMFLY_RB_H
#define HAMFLY_RB_H

#include <stdint.h>

#ifndef HAMFLY_RB_SIZE
#define HAMFLY_RB_SIZE 256u
#endif

typedef struct {
    volatile uint8_t  buf[HAMFLY_RB_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t drops;
} hamfly_rb_t;

void     hamfly_rb_init (hamfly_rb_t *rb);
void     hamfly_rb_clear(hamfly_rb_t *rb);
uint16_t hamfly_rb_count(const hamfly_rb_t *rb);
uint8_t  hamfly_rb_push (hamfly_rb_t *rb, uint8_t b);  /* call from ISR */
uint8_t  hamfly_rb_pop  (hamfly_rb_t *rb, uint8_t *b); /* call from main */

#endif /* HAMFLY_RB_H */
