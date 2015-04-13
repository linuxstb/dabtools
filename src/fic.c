#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "dab.h"
#include "fic.h"
#include "viterbi.h"
#include "dab_tables.h"

#define CRC_POLY    0x8408
#define CRC_GOOD    0xf0b8

int crc16(unsigned char *buf, int len, int width)
{
  unsigned short crc;
  int c15;
  int i,j;

  crc = 0xffff;
  for (i=0; i<len; i++) {
    for (j=0; j<width; j++) {
      c15 = (crc & 1) ^ ((buf[i]>>(width-1-j))&1);
      crc = crc >> 1;
      if (c15) crc ^= CRC_POLY;
     }
  }
  //fprintf(stderr,"crc=0x%04x\n",crc);

  return (crc == CRC_GOOD);
}

static void bit_to_byte(unsigned char *ibuf, int ilen, unsigned char *obuf)
{
  int i,j;

  j = 0;
  for (i=0; i<ilen; i+=8) {
    obuf[j] = (ibuf[i+0]<<7) + (ibuf[i+1]<<6) + (ibuf[i+2]<<5) + (ibuf[i+3]<<4) +       //be
      (ibuf[i+4]<<3) + (ibuf[i+5]<<2) + (ibuf[i+6]<<1) + (ibuf[i+7]<<0);
    j++;
  }

  return;
}

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

static void fic_depuncture(uint8_t *in, uint8_t *out)
{
    int i,j,offset;

    offset = 0;
    for (i=0; i<21*128; i+=32)
    {
        for (j=0; j<8; j++)
        {
            out[i+j*4+0] = in[offset+0];
            out[i+j*4+1] = in[offset+1];
            out[i+j*4+2] = in[offset+2];
            out[i+j*4+3] = 8;           
            offset+=3;
        }
    }
    for (i=21*128; i<24*128; i+=32)
    {
        for (j=0; j<7; j++)
        {
            out[i+j*4+0] = in[offset+0];
            out[i+j*4+1] = in[offset+1];
            out[i+j*4+2] = in[offset+2];
            out[i+j*4+3] = 8;
            offset+=3;
        }
        out[i+j*4+0] = in[offset+0];
        out[i+j*4+1] = in[offset+1];
        out[i+j*4+2] = 8;
        out[i+j*4+3] = 8;
        offset+=2;
    }
    for (j=0; j<6; j++)
    {
        out[i+j*4+0] = in[offset+0];
        out[i+j*4+1] = in[offset+1];
        out[i+j*4+2] = 8;
        out[i+j*4+3] = 8;
        offset+=2;
    }

   return;
}

static void fic_descramble(uint8_t *in, uint8_t * out, int32_t len)
{
  int32_t i;
  uint16_t p = 0x01FF;
  int32_t pp;
  for (i=0; i<len; i++)
    {
      p = p << 1;
      pp = ((p>>9)&1) ^ ((p>>5)&1);
      p |= pp;
      out[i] = in[i] ^ pp;
    }
  return;
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
  uint8_t tmp1[3096];
  uint8_t tmp2[768];
  int i,j;

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
    fic_depuncture(tf->fic_symbols_demapped[0]+(i*2304), tmp1);

    /* viterbi, 3096->768 */
    viterbi(tmp1, 3096, tmp2);
  
    /* descramble, 768->768 */
    fic_descramble(tmp2, tmp1, 768);

    // 768 bits = 3 FIBs - check CRC and convert to bytes
    for (j=0;j<3;j++) {
      tf->fibs.FIB_CRC_OK[fib] = crc16(tmp1+j*256, 256, 1);

      if (tf->fibs.FIB_CRC_OK[fib]) {
        bit_to_byte(tmp1+j*256, 256, tf->fibs.FIB[fib]);
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
