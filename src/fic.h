#ifndef _FIC_H
#define _FIC_H

int crc16(unsigned char *buf, int len, int width);
void fib_decode(struct tf_info_t *info, struct tf_fibs_t *fibs, int nfibs);
void fic_decode(struct dab_state_t *dab, struct demapped_transmission_frame_t *tf);
void dump_tf_info(struct tf_info_t* info);

#endif
