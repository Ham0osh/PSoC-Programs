/*
MIT License

Copyright (c) 2017 Freefly Systems

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Description: Test Buffer

*/



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


