// Simple Buffer

#ifndef _simple_buffer_h
#define _simple_buffer_h

#include <stdint.h>

#define RECV_BUF_IDX 0
#define SEND_BUF_IDX 1

signed long BufAdd(uint8_t buf_num, uint8_t c);
signed long BufRemove(uint8_t buf_num, volatile uint8_t * c);

#endif

