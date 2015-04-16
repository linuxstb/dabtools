#ifndef _INPUT_WF_H
#define _INPUT_WF_H

#include <stdint.h>
#include "dab.h"

struct wavefinder_t {
  int fd;
  int sync_locked;
  unsigned char *symstr;
  int fic_read[3];
  int msc_read[72];
};

int wf_init(struct wavefinder_t* wf);
int wf_open(struct wavefinder_t* wf, char* filename);
void wf_tune(struct wavefinder_t* wf, int freq_khz);
int wf_read_frame(struct wavefinder_t* wf, struct demapped_transmission_frame_t *tf);

#endif
