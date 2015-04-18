#ifndef _VITERBI_SPIRAL_H
#define _VITERBI_SPIRAL_H

#define K 7
#define RATE 4
//#define POLYS { 91, 121, 101, 91 }
#define POLYS { 0x6d, 0x4f, 0x53, 0x6d }
#define NUMSTATES 64
#define FRAMEBITS 768
#define DECISIONTYPE unsigned char
#define DECISIONTYPE_BITSIZE 8
#define COMPUTETYPE unsigned char
#define EBN0 3
#define TRIALS 10000
#define __int32 int
#define FUNC FULL_SPIRAL
#define METRICSHIFT 2
#define PRECISIONSHIFT 2
#define RENORMALIZE_THRESHOLD 110


void *create_viterbi(int len);
void viterbi(void *p,COMPUTETYPE *symbols, unsigned char *data, int framebits);

#endif
