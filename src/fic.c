#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "dab.h"
#include "fic.h"
#include "depuncture.h"
#include "viterbi.h"
#include "misc.h"
#include "dab_tables.h"

static int dump_buffer(char *name, char *buf, int blen)
{
  int i;

  printf("%s = ",name);
  for(i=0; i<blen; i++)
    {
      printf("%02x ",(unsigned char)buf[i]);
    }
  printf("\n");
  return 0;
}

void dump_tf_info(struct tf_info_t* info)
{
  int i;
  
  fprintf(stderr,"EId=0x%04x, CIFCount = %d %d\n",info->EId,info->CIFCount_hi,info->CIFCount_lo);

  for (i=0;i<64;i++) {
    struct subchannel_info_t *sc = &info->subchans[i];
    if (sc->id >= 0) {
      fprintf(stderr,"SubChId=%d, slForm=%d, StartAddress=%d, size=%d, bitrate=%d, ASCTy=0x%02x\n",sc->id,sc->slForm,sc->start_cu,sc->size,sc->bitrate,sc->ASCTy);
    }
  }
}

/* Simple FIB/FIG parser to extract information from FIG 0/0
   (Ensemble Information - CIFCount) and FIG 0/1 (Sub-channel
   information) needed to create ETI stream */
void fib_parse(struct tf_info_t* info, uint8_t* fib)
{
  int i,j,k;

  i = 0;
  while ((fib[i] != 0xff) && (i < 30)) {
    int type = (fib[i] & 0xe0) >> 5;
    int len = fib[i] & 0x1f;
    //fprintf(stderr,"FIG: i=%d, type=%d, len=%d\n",i,type,len);
    i++;

    if (type == 0) {
      int ext = fib[i] & 0x1f;
      int PD = (fib[i] & 0x20) >> 5;
      int OE = (fib[i] & 0x40) >> 6;
      int CN = (fib[i] & 0x80) >> 7;
      //fprintf(stderr,"Type 0 - ext=%d\n",ext);

      if (ext == 0) {  // FIG 0/0
        info->EId = (fib[i+1] << 8) | fib[i+2];
        info->CIFCount_hi = fib[i+3] & 0x1f;
        info->CIFCount_lo = fib[i+4];
      } else if (ext == 1) { // FIG 0/1
        j = i + 1;
        while (j < i + len) {
          int id = (fib[j] & 0xfc) >> 2;
	  struct subchannel_info_t *sc = &info->subchans[id];

	  sc->id = id;
	  sc->start_cu =  ((fib[j] & 0x03) << 8) | fib[j+1];
          sc->slForm = (fib[j+2] & 0x80) >> 7;
	  sc->eepprot = sc->slForm;
          if (sc->slForm == 0) {
            sc->uep_index = fib[j+2] & 0x3f;
            sc->size = ueptable[sc->uep_index].subchsz;
            sc->bitrate = ueptable[sc->uep_index].bitrate;
            sc->protlev = ueptable[sc->uep_index].protlvl;
            j += 3;
          } else {
            int Option = (fib[j+2] & 0x70) >> 4;
            sc->protlev = (fib[j+2] & 0x0c) >> 2;
            sc->protlev |= Option << 2;
            sc->size = ((fib[j+2] & 0x03) << 8) | fib[j+3];
            sc->bitrate = (sc->size / eeptable[sc->protlev].sizemul) * eeptable[sc->protlev].ratemul;
            j += 4;
          }
        }
      } else if (ext == 2) { // FIG 0/2
        j = i + 1;
        while (j < i + len) {
          int sid;
          if (PD == 0) { // Audio stream
            sid = (fib[j] << 8) | fib[j+1];
            j += 2;
          } else { // Data stream
            sid = (fib[j] << 24) | (fib[j+1] << 16) | (fib[j+2] << 8) | fib[j+3];
            j += 4;
          }
          int n = fib[j++] & 0x0f;
          //fprintf(stderr,"service %d, ncomponents=%d\n",sid,n);
          for (k=0;k<n;k++) {
            int TMid = (fib[j] & 0xc0) >> 6;
            if (TMid == 0) {
              int id = (fib[j+1]&0xfc) >> 2;
              info->subchans[id].ASCTy = fib[j] & 0x3f;
              //fprintf(stderr,"Subchannel %d, ASCTy=0x%02x\n",id,info->subchans[id].ASCTy);
            } else if (TMid == 1) {
              int id = (fib[j+1]&0xfc) >> 2;
              fprintf(stderr,"Unhandled TMid %d for subchannel %d\n",TMid,id);
            } else if (TMid == 2) {
              int id = (fib[j+1]&0xfc) >> 2;
              fprintf(stderr,"Unhandled TMid %d for subchannel %d\n",TMid,id);
            } else if (TMid == 3) {
              int id = (fib[j+1] << 4) | (fib[j+2]&0xf0) >> 4;
              //fprintf(stderr,"Unhandled TMid %d for subchannel %d\n",TMid,id);
            }
            j += 2;
          }
        }
      }
    }
    i += len;
  }
}

void fib_decode(struct tf_info_t *info, struct tf_fibs_t *fibs, int nfibs)
{
  int i;
  
  /* Initialise the info struct */
  memset(info, 0, sizeof(struct tf_info_t));
  for (i=0;i<64;i++) { info->subchans[i].id = -1; info->subchans[i].ASCTy = -1; }

  /* Parse nfibs FIBs */
  for (i=0;i<nfibs;i++) {
    if (fibs->FIB_CRC_OK[i]) {
      //fprintf(stderr,"fib_parse(%d)\n",i);
      fib_parse(info, fibs->FIB[i]);
    }
  }
}

/* A NULL FIB with valid CRC */
static uint8_t null_fib[32] = {
  0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa8, 0xa8
};

/* Convert the 3 demapped FIC symbols (3 * 3072 bits) into 4 sets of 3
   32-byte FIBs, check the FIB CRCs, and parse ensemble and sub-channel information. */

void fic_decode(struct demapped_transmission_frame_t *tf)
{
  uint8_t tmp[3096];
  int i,j;
  unsigned int metric;

  tf->fibs.ok_count = 0;

  if (!tf->has_fic) {
    /* NO FIC in received data, replace with NULL FIBs */
    for (i=0;i<12;i++) {
      memcpy(tf->fibs.FIB[i], null_fib, 32);
      tf->fibs.FIB_CRC_OK[i] = 1;
    }
    tf->fibs.ok_count = 12;
    return;
  }

  int fib = 0;

  /* The 3 FIC symbols are 3*3072 = 9216 bits in total.
     We treat them as 4 sets of 2304 bits */
  for (i=0;i<4;i++) {
    /* depuncture, 2304->3096 */
    fic_depuncture(tmp, tf->fic_symbols_demapped[0]+(i*2304));

    /* viterbi, 3096 -> 768.  Output is converted to bytes */
    viterbi(&metric, tf->fibs.FIB[fib], tmp, 768, mettab, 0, 0);
  
    /* descramble (in-place), 768->768 */
    dab_descramble_bytes(tf->fibs.FIB[fib], 96);

    /* Now check the CRCs of the three FIBs. */
    for (j=0;j<3;j++) {
      tf->fibs.FIB_CRC_OK[fib] = check_fib_crc(tf->fibs.FIB[fib]);

      if (tf->fibs.FIB_CRC_OK[fib]) {
        tf->fibs.ok_count++;
        //fprintf(stderr,"CRC OK in fib %d\n",fib);
      } else {
        //fprintf(stderr,"CRC error in fib %d\n",fib);
      }
      fib++;
    }
  }

  return;
}
