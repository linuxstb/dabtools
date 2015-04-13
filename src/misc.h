#ifndef _MISC_H
#define _MISC_H

void merge_info(struct ens_info_t* ei, struct tf_info_t *info);
void create_eti(uint8_t* fibs, uint8_t* cifs_msc[], struct ens_info_t *info);
void dump_ens_info(struct ens_info_t* info);

#endif

