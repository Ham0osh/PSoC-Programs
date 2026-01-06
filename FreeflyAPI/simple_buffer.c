// Test Buffer

#include "simple_buffer.h"

// standard headers
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

#define NUM_OF_BUFS 2
#define BUF_LENGTH 64

volatile uint8_t Buf[NUM_OF_BUFS][BUF_LENGTH];	//circular buffers
volatile uint8_t head[NUM_OF_BUFS];
volatile uint8_t tail[NUM_OF_BUFS];

//----------------------------------------------------------------------------
// Interrupt safe add (only modifies head)
signed long BufAdd(uint8_t buf_num, uint8_t c)
{
    uint32_t next_head = (head[buf_num] + 1) % BUF_LENGTH;
    if(next_head != tail[buf_num])
    {
        Buf[buf_num][head[buf_num]] = c;
        head[buf_num] = next_head;
        if(((head[buf_num] + 1) % BUF_LENGTH) == tail[buf_num])
        {
            return 0;    // Now full
        }
        else
        {
            return 1;    // Still room
        }
    }
    else
    {
        return 0;        // Full
    }
}

//----------------------------------------------------------------------------
// Interrupt safe remove (only modifies tail)
signed long BufRemove(uint8_t buf_num, volatile uint8_t * c)
{
    if(head[buf_num] != tail[buf_num]) 
    {
        *c = Buf[buf_num][tail[buf_num]];
        tail[buf_num] = (tail[buf_num] + 1) % BUF_LENGTH;
        return 1;
    } else {
        return 0;
    }
}


