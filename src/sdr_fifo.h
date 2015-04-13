/*
This file is part of rtl-dab
trl-dab is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Foobar is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with rtl-dab.  If not, see <http://www.gnu.org/licenses/>.


david may 2012
david.may.muc@googlemail.com

*/

#include <stdio.h>
#include <stdint.h>
#include <malloc.h>


typedef struct 
{
  uint32_t size;
  uint32_t start;
  uint32_t count;
  uint8_t *elems;
} CircularBuffer;


void cbInit(CircularBuffer *cb, uint32_t size);
void cbFree(CircularBuffer *cb);
int cbIsFull(CircularBuffer *cb);
int cbIsEmpty(CircularBuffer *cb);
void cbWrite(CircularBuffer *cb, uint8_t *elem);
void cbRead(CircularBuffer *cb, uint8_t *elem);

int32_t sdr_read_fifo(CircularBuffer * fifo,uint32_t bytes,int32_t shift,uint8_t * buffer);
