/* Generic Viterbi decoder,
 * Copyright Phil Karn, KA9Q, 
 * Code has been slightly modified for use with Spiral (www.spiral.net)
 * Karn's original code can be found here: http://www.ka9q.net/code/fec/
 * May be used under the terms of the GNU Lesser General Public License (LGPL)
 * see http://www.gnu.org/copyleft/lgpl.html
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <memory.h>
#include <sys/resource.h>
#include "viterbi_spiral.h"
#include <pmmintrin.h>
#include <emmintrin.h>
#include <xmmintrin.h>
#include <mmintrin.h>

#define OFFSET (127.5)
#define CLIP 255

//#define GENERICONLY

/* Determine parity of argument: 1 = odd, 0 = even */
#ifdef __i386__
static inline int parityb(unsigned char x){
  __asm__ __volatile__ ("test %1,%1;setpo %0" : "=g"(x) : "r" (x));
  return x;
}
#else
void partab_init();

static inline int parityb(unsigned char x){
  extern unsigned char Partab[256];
  extern int P_init;
  if(!P_init){
    partab_init();
  }
  return Partab[x];
}
#endif

static inline int parity(int x){
  /* Fold down to one byte */
  x ^= (x >> 16);
  x ^= (x >> 8);
  return parityb(x);
}

unsigned char Partab[256];
int P_init;

/* Create 256-entry odd-parity lookup table
 * Needed only on non-ia32 machines
 */
void partab_init(void){
  int i,cnt,ti;

  /* Initialize parity lookup table */
  for(i=0;i<256;i++){
    cnt = 0;
    ti = i;
    while(ti){
      if(ti & 1)
	cnt++;
      ti >>= 1;
    }
    Partab[i] = cnt & 1;
  }
  P_init=1;
}

/* Lookup table giving count of 1 bits for integers 0-255 */
int Bitcnt[] = {
 0, 1, 1, 2, 1, 2, 2, 3,
 1, 2, 2, 3, 2, 3, 3, 4,
 1, 2, 2, 3, 2, 3, 3, 4,
 2, 3, 3, 4, 3, 4, 4, 5,
 1, 2, 2, 3, 2, 3, 3, 4,
 2, 3, 3, 4, 3, 4, 4, 5,
 2, 3, 3, 4, 3, 4, 4, 5,
 3, 4, 4, 5, 4, 5, 5, 6,
 1, 2, 2, 3, 2, 3, 3, 4,
 2, 3, 3, 4, 3, 4, 4, 5,
 2, 3, 3, 4, 3, 4, 4, 5,
 3, 4, 4, 5, 4, 5, 5, 6,
 2, 3, 3, 4, 3, 4, 4, 5,
 3, 4, 4, 5, 4, 5, 5, 6,
 3, 4, 4, 5, 4, 5, 5, 6,
 4, 5, 5, 6, 5, 6, 6, 7,
 1, 2, 2, 3, 2, 3, 3, 4,
 2, 3, 3, 4, 3, 4, 4, 5,
 2, 3, 3, 4, 3, 4, 4, 5,
 3, 4, 4, 5, 4, 5, 5, 6,
 2, 3, 3, 4, 3, 4, 4, 5,
 3, 4, 4, 5, 4, 5, 5, 6,
 3, 4, 4, 5, 4, 5, 5, 6,
 4, 5, 5, 6, 5, 6, 6, 7,
 2, 3, 3, 4, 3, 4, 4, 5,
 3, 4, 4, 5, 4, 5, 5, 6,
 3, 4, 4, 5, 4, 5, 5, 6,
 4, 5, 5, 6, 5, 6, 6, 7,
 3, 4, 4, 5, 4, 5, 5, 6,
 4, 5, 5, 6, 5, 6, 6, 7,
 4, 5, 5, 6, 5, 6, 6, 7,
 5, 6, 6, 7, 6, 7, 7, 8,
};

extern int posix_memalign(void **memptr, size_t alignment, size_t size);
//decision_t is a BIT vector
typedef union {
  DECISIONTYPE t[NUMSTATES/DECISIONTYPE_BITSIZE];
  unsigned int w[NUMSTATES/32];
  unsigned short s[NUMSTATES/16];
  unsigned char c[NUMSTATES/8];
} decision_t __attribute__ ((aligned (16)));

typedef union {
  COMPUTETYPE t[NUMSTATES];
} metric_t __attribute__ ((aligned (16)));

inline void renormalize(COMPUTETYPE* X, COMPUTETYPE threshold){
      if (X[0]>threshold){
    COMPUTETYPE min=X[0];
    for(int i=0;i<NUMSTATES;i++)
    if (min>X[i])
      min=X[i];
    for(int i=0;i<NUMSTATES;i++)
      X[i]-=min;
      }
}

COMPUTETYPE Branchtab[NUMSTATES/2*RATE] __attribute__ ((aligned (16)));

/* State info for instance of Viterbi decoder
 */
struct v {
  __attribute__ ((aligned (16))) metric_t metrics1; /* path metric buffer 1 */
  __attribute__ ((aligned (16))) metric_t metrics2; /* path metric buffer 2 */
  metric_t *old_metrics,*new_metrics; /* Pointers to path metrics, swapped on every bit */
  decision_t *decisions;   /* decisions */
};

/* Initialize Viterbi decoder for start of new frame */
int init_viterbi(void *p,int starting_state){
  struct v *vp = p;
  int i;

  if(p == NULL)
    return -1;
  for(i=0;i<NUMSTATES;i++)
      vp->metrics1.t[i] = 63;

  vp->old_metrics = &vp->metrics1;
  vp->new_metrics = &vp->metrics2;
  vp->old_metrics->t[starting_state & (NUMSTATES-1)] = 0; /* Bias known start state */
  return 0;
}

/* Create a new instance of a Viterbi decoder */
void *create_viterbi(int len){
  void *p;
  struct v *vp;
  static int Init = 0;

  if(!Init){
    int state, i;
    int polys[RATE] = POLYS;
    for(state=0;state < NUMSTATES/2;state++){
      for (i=0; i<RATE; i++){
        Branchtab[i*NUMSTATES/2+state] = (polys[i] < 0) ^ parity((2*state) & abs(polys[i])) ? 255 : 0;
      }
    }
    Init++;
  }

  if(posix_memalign((void**)&p, 16,sizeof(struct v)))
    return NULL;

  vp = (struct v *)p;

  if(posix_memalign((void**)&vp->decisions, 16,(len+(K-1))*sizeof(decision_t))){
    free(vp);
    return NULL;
  }
  init_viterbi(vp,0);

  return vp;
}

/* Viterbi chainback */
int chainback_viterbi(
      void *p,
      unsigned char *data, /* Decoded output data */
      unsigned int nbits, /* Number of data bits */
      unsigned int endstate){ /* Terminal encoder state */
  struct v *vp = p;
  decision_t *d;

  /* ADDSHIFT and SUBSHIFT make sure that the thing returned is a byte. */
#if (K-1<8)
#define ADDSHIFT (8-(K-1))
#define SUBSHIFT 0
#elif (K-1>8)
#define ADDSHIFT 0
#define SUBSHIFT ((K-1)-8)
#else
#define ADDSHIFT 0
#define SUBSHIFT 0
#endif

  if(p == NULL)
    return -1;
  d = vp->decisions;
  /* Make room beyond the end of the encoder register so we can
   * accumulate a full byte of decoded data
   */

  endstate = (endstate%NUMSTATES) << ADDSHIFT;

  /* The store into data[] only needs to be done every 8 bits.
   * But this avoids a conditional branch, and the writes will
   * combine in the cache anyway
   */
  d += (K-1); /* Look past tail */
  while(nbits-- != 0){
    int k;
    k = (d[nbits].w[(endstate>>ADDSHIFT)/32] >> ((endstate>>ADDSHIFT)%32)) & 1;
    endstate = (endstate >> 1) | (k << (K-2+ADDSHIFT));
    data[nbits>>3] = endstate>>SUBSHIFT;
  }
  return 0;
}

/* Delete instance of a Viterbi decoder */
void delete_viterbi(void *p){
  struct v *vp = p;

  if(vp != NULL){
    free(vp->decisions);
    free(vp);
  }
}

/* C-language butterfly */
void BFLY(int i, int s, COMPUTETYPE * syms, struct v * vp, decision_t * d) {
  int j, decision0, decision1;
  COMPUTETYPE metric,m0,m1,m2,m3;

  metric =0;
  for (j=0;j<RATE;j++) metric += (Branchtab[i+j*NUMSTATES/2] ^ syms[s*RATE+j])>>METRICSHIFT ;
  metric=metric>>PRECISIONSHIFT;
  
  const COMPUTETYPE max = ((RATE*((256 -1)>>METRICSHIFT))>>PRECISIONSHIFT);
  
  m0 = vp->old_metrics->t[i] + metric;
  m1 = vp->old_metrics->t[i+NUMSTATES/2] + (max - metric);
  m2 = vp->old_metrics->t[i] + (max - metric);
  m3 = vp->old_metrics->t[i+NUMSTATES/2] + metric;
  
  decision0 = (signed int)(m0-m1) > 0;
  decision1 = (signed int)(m2-m3) > 0;
  
  vp->new_metrics->t[2*i] = decision0 ? m1 : m0;
  vp->new_metrics->t[2*i+1] =  decision1 ? m3 : m2;
  
  d->w[i/(sizeof(unsigned int)*8/2)+s*(sizeof(decision_t)/sizeof(unsigned int))] |= 
    (decision0|decision1<<1) << ((2*i)&(sizeof(unsigned int)*8-1));
}


/* Update decoder with a block of demodulated symbols
 * Note that nbits is the number of decoded data bits, not the number
 * of symbols!
 */
    COMPUTETYPE max_spread = 0;

int update_viterbi_blk_GENERIC(void *p, COMPUTETYPE *syms,int nbits){
  struct v *vp = p;

  decision_t *d;
  int s,i;
  if(p == NULL)
    return -1;
  d = (decision_t *)vp->decisions;

  for (s=0;s<nbits;s++)
    memset(d+s,0,sizeof(decision_t));

  for (s=0;s<nbits;s++){
    void *tmp;
    for(i=0;i<NUMSTATES/2;i++){
      BFLY(i, s, syms, vp, vp->decisions);
    }

#ifdef GENERICONLY
    COMPUTETYPE min=vp->new_metrics->t[0];
    COMPUTETYPE max=vp->new_metrics->t[0];
    
    /* Compute Spread */
    for(int i=0;i<NUMSTATES;i++)
      if (min>vp->new_metrics->t[i]) 
        min=vp->new_metrics->t[i];
      else if (max<vp->new_metrics->t[i])
        max=vp->new_metrics->t[i];
    if (max_spread<max-min)
      max_spread=max-min;
#endif

    renormalize(vp->new_metrics->t, RENORMALIZE_THRESHOLD);
    
    ///     Swap pointers to old and new metrics
    tmp = vp->old_metrics;
    vp->old_metrics = vp->new_metrics;
    vp->new_metrics = tmp;
  }

  return 0;
}

#ifndef GENERICONLY
extern void FULL_SPIRAL(COMPUTETYPE *Y, COMPUTETYPE *X, COMPUTETYPE *syms, DECISIONTYPE *dec, COMPUTETYPE *Branchtab, int bits);

int update_viterbi_blk_SPIRAL(void *p, COMPUTETYPE *syms,int nbits){
  struct v *vp = p;

  decision_t *d;
  int s,i;

  if(p == NULL)
    return -1;
  d = (decision_t *)vp->decisions;

  for (s=0;s<nbits;s++)
    memset(d+s,0,sizeof(decision_t));

  FULL_SPIRAL( vp->new_metrics->t, vp->old_metrics->t, syms, d->t, Branchtab, nbits);

  return 0;
}
#endif

void viterbi(void *p,COMPUTETYPE *symbols, unsigned char *data, int framebits)
{
  struct v *vp = p;

  /* Initialize Viterbi decoder */
  init_viterbi(vp,0);
    
  /* Decode block */
  update_viterbi_blk_SPIRAL(vp,symbols,framebits+(K-1));

  /* Do Viterbi chainback */
  chainback_viterbi(vp,data,framebits,0);
}
