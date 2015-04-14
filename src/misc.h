#ifndef _MISC_H
#define _MISC_H

#include "dab.h"

void merge_info(struct ens_info_t* ei, struct tf_info_t *info);
void create_eti(struct dab_state_t* dab);
void dump_ens_info(struct ens_info_t* info);

#endif

