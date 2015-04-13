#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#include "dab.h"
#include "fic.h"
#include "wf_sync.h"
#include "dab_tables.h"
#include "../wavefinder-driver/wavefinder.h"

/* Select all symbols by default */
static unsigned char selstr[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

/* Use this before changing the symbol selection */
static unsigned char chgstr[] = {0x00, 0xf0, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11};

/* Convert the data read over USB into the common "demapped symbols"
   format */
static void wf_demap_symbol(uint8_t* dst, uint8_t* src)
{
  int i,j,k;
  uint8_t tmp[3072];  /* 384*8 */
  uint8_t* p = tmp;

  /* Convert 16-bit LE words to bits */
  for (i=0;i<192;i++) {
    k = (src[1]<<8) | src[0];
    src += 2;
    for (j=15;j>=0;j--) {
      *(p++) = (k >> j) & 1;
    }
  }

  /* Frequency deinterleaver and QPSK symbol demapper combined */
  for(i=0;i<1536;i++) {
    k = freq_deint_tab[i];

    dst[i] = tmp[(2*k)+0];
    dst[i+1536] = tmp[(2*k)+1];
  }
}

int wf_init(struct wavefinder_t* wf)
{
  wf->sync_locked = 0;
  wf->symstr = selstr;
  return wfsyncinit();
}

int wf_open(struct wavefinder_t* wf, char* filename)
{
  wf->fd = open(filename, O_RDWR);
  if (wf->fd < 0) {
    return wf->fd;
  }
  return 0;
}

void wf_tune(struct wavefinder_t* wf, int freq_khz)
{
  fprintf(stderr,"Tuning to %uKHz...",freq_khz);
  ioctl(wf->fd,IOCTL_WAVEFINDER_TUNE_KHZ,freq_khz);
  fprintf(stderr,"complete.\n");
}

int wf_read_frame(struct wavefinder_t* wf, struct demapped_transmission_frame_t *tf)
{
  int i;
  size_t n;
  uint8_t buf[524];
  
  memset(wf->fic_read, 0, sizeof(wf->fic_read));
  memset(wf->msc_read, 0, sizeof(wf->msc_read));

  while (1) {
    n = read(wf->fd, buf, 524);
    if (n < 524) {
      fprintf(stderr,"Read error: n=%d\n",(int)n);
      return 1;
    }

#if 0
    for (i=0;i<12;i++) {
      fprintf(stderr,"%02x ",buf[i]);
    }
    fprintf(stderr,"\n");
#endif
    int symbol = buf[2];
    
    if (symbol == 1) {
      /* PRS Symbol */
      wf_prs_assemble(wf, buf);
    } else if (wf->sync_locked) {
      /* Other symbols */
      if (symbol == 0) {
        /* NULL Symbol - process all that we have read so far */

	/* First process the FIC symbols ,if any */
	if ((wf->fic_read[0]) && (wf->fic_read[1]) && (wf->fic_read[2])) {
          tf->has_fic = 1;
	  for (i=0;i<3;i++) {
	    //dump_buffer("data2", fic_buffer+(i*384), 384);
            wf_demap_symbol(tf->fic_symbols_demapped[i], wf->fic_buffer+(i*384));
	  }
	} else {
	  tf->has_fic = 0;
	  /* ?? What do we write to the ETI stream in this case? */
	  /* At the very least we need to calculate CIFCount based on the previous FIC */
	}

	for (i=0;i<72;i++) {
	  if (wf->msc_read[i]) {
	    tf->msc_filter[i] = 1;
	    wf_demap_symbol(tf->msc_symbols_demapped[i], wf->msc_buffer+(i*384));
	  } else {
	    tf->msc_filter[i] = 0;
	  }
	}

        return 0;
      } else if (symbol <= 4) { /* Symbols 2, 3 and 4 are FIC symbols*/
	memcpy(wf->fic_buffer + (symbol-2)*384, buf+12, 384);
	wf->fic_read[symbol-2] = 1;
      } else if (symbol <= 76) { /* Symbols 5 to 76 are MSC symbols */
	memcpy(wf->msc_buffer + (symbol-5)*384, buf+12, 384);
	wf->msc_read[symbol-5] = 1;
      }
    }
  }
}
