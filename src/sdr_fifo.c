#include "sdr_fifo.h"


/* http://en.wikipedia.org/wiki/Circular_buffer */



void cbInit(CircularBuffer *cb, uint32_t size) {
    cb->size  = size;
    cb->start = 0;
    cb->count = 0;
    cb->elems = (uint8_t *)calloc(cb->size, sizeof(uint8_t));
}
void cbFree(CircularBuffer *cb) {
    free(cb->elems);
	}

int cbIsFull(CircularBuffer *cb) {
    return cb->count == cb->size; 
	}
 
int cbIsEmpty(CircularBuffer *cb) {
    return cb->count == 0;
	 }
 
void cbWrite(CircularBuffer *cb, uint8_t *elem) {
    uint32_t end = (cb->start + cb->count) % cb->size;
    cb->elems[end] = *elem;
    if (cb->count == cb->size) {
        cb->start = (cb->start + 1) % cb->size; 
	fprintf(stderr,"fifo overflow!\n");
    }
    else
        ++ cb->count;
}
 
void cbRead(CircularBuffer *cb, uint8_t *elem) {
    *elem = cb->elems[cb->start];
    cb->start = (cb->start + 1) % cb->size;
    -- cb->count;
}

int32_t sdr_read_fifo(CircularBuffer * fifo,uint32_t bytes,int32_t shift,uint8_t * buffer)
{
  int32_t i=0;
  uint32_t j=0;
  if (shift>0)
    {
      for (i=0;i<shift;i++)
	if(!cbIsEmpty(fifo))
	  cbRead(fifo,&buffer[i]);
      for (j=0;j<bytes;j++)
	if(!cbIsEmpty(fifo))
	  cbRead(fifo,&buffer[j]);
    }
  else {
    for (j=0;j<bytes+shift;j++)
      cbRead(fifo,&buffer[j]);
  }
  return 1;
}
