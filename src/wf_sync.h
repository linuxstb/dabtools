#ifndef _WF_SYNC_H
#define _WF_SYNC_H

#include "input_wf.h"

int wfsyncinit(void);
int wf_prs_assemble(struct wavefinder_t* wf, unsigned char *rdbuf);

#endif
