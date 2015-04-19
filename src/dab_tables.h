#ifndef _DAB_TABLES_H
#define _DAB_TABLES_H

#include <stdint.h>

struct uepprof {
  unsigned int bitrate;
  unsigned int subchsz;
  unsigned int protlvl;
  int l[4];
  int pi[4];
  int padbits;
};

struct eepprof {
  int sizemul;
  int ratemul;
  struct {
    int mul;
    int offset;
  } l[2];
  int pi[2];
};

extern const struct uepprof ueptable[];
extern const struct eepprof eeptable[];
extern const struct eepprof eep2a8kbps;
extern const char pvec[][32];

extern const uint16_t rev_freq_deint_tab[1536];

#endif


