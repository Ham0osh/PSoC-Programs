/* hamfly_txbuf.c
 * ============================================================
 * Copyright Hamish Johnson, 2026
 * Quantum Internet Systems Lab, SFU Physics
 * ============================================================
 *
 * Instance-based TX staging buffer.
 * Replaces the global simple_buffer singleton so each
 * hamfly_gimbal_t owns its own TX buffer.
 * Main-loop only -- not ISR-safe.
 */

#include "hamfly_comm_txbuf.h"

void hamfly_txbuf_init(hamfly_txbuf_t *tb)
{
    tb->head = 0u;
    tb->tail = 0u;
}

void hamfly_txbuf_clear(hamfly_txbuf_t *tb)
{
    tb->head = 0u;
    tb->tail = 0u;
}

int32_t hamfly_txbuf_add(hamfly_txbuf_t *tb, uint8_t b)
{
    uint8_t next = (uint8_t)((tb->head + 1u) % HAMFLY_TXBUF_SIZE);
    if (next == tb->tail) return 0;   /* full */
    tb->buf[tb->head] = b;
    tb->head = next;
    return 1;
}

int32_t hamfly_txbuf_remove(hamfly_txbuf_t *tb, uint8_t *b)
{
    if (tb->head == tb->tail) return 0;   /* empty */
    *b       = tb->buf[tb->tail];
    tb->tail = (uint8_t)((tb->tail + 1u) % HAMFLY_TXBUF_SIZE);
    return 1;
}
