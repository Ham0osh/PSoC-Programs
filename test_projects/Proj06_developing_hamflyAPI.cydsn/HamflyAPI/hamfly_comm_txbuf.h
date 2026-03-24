/* hamfly_txbuf.h
 * ============================================================
 * Copyright Hamish Johnson, 2026
 * Quantum Internet Systems Lab, SFU Physics
 * ============================================================
 *
 * Instance-based TX staging buffer.
 * QX_SendMsg2CommsPort_CB writes bytes in via hamfly_txbuf_add().
 * Send functions drain via hamfly_txbuf_remove().
 * Main-loop only -- not ISR-safe.
 */

#ifndef HAMFLY_TXBUF_H
#define HAMFLY_TXBUF_H

#include <stdint.h>

#ifndef HAMFLY_TXBUF_SIZE
#define HAMFLY_TXBUF_SIZE 64u
#endif

typedef struct {
    uint8_t  buf[HAMFLY_TXBUF_SIZE];
    uint8_t  head;
    uint8_t  tail;
} hamfly_txbuf_t;

void    hamfly_txbuf_init  (hamfly_txbuf_t *tb);
void    hamfly_txbuf_clear (hamfly_txbuf_t *tb);
int32_t hamfly_txbuf_add   (hamfly_txbuf_t *tb, uint8_t b);
int32_t hamfly_txbuf_remove(hamfly_txbuf_t *tb, uint8_t *b);

#endif /* HAMFLY_TXBUF_H */
