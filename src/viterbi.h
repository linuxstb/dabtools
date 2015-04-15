#ifndef _VITERBI_H
#define _VITERBI_H

extern int mettab[2][256];

int init_viterbi();
int viterbi(unsigned int *metric, unsigned char *data, unsigned char *symbols, unsigned int nbits, int mettab[][256], unsigned int startstate, unsigned int endstate);

#endif
