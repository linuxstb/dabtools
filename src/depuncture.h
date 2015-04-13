#ifndef _DEPUNCTURE_H
#define _DEPUNCTURE_H

#include "dab.h"

int uep_depuncture(unsigned char *obuf, unsigned char *inbuf, struct subchannel_info_t *s, int* len);
int eep_depuncture(unsigned char *obuf, unsigned char *inbuf, struct subchannel_info_t *s, int* len);

#endif
