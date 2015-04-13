#ifndef _VITERBI_H
#define _VITERBI_H

extern int mettab[2][256];

int init_viterbi();
int k_viterbi(unsigned int *metric, unsigned char *data, unsigned char *symbols, unsigned int nbits, int mettab[][256], unsigned int startstate, unsigned int endstate);
int viterbi(unsigned  char *ibuf, int ilen, unsigned char *obuf);

#endif
