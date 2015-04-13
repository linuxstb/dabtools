#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "dab.h"
#include "fic.h"
#include "viterbi.h"
#include "depuncture.h"
#include "misc.h"
#include "input_wf.h"
#include "input_sdr.h"

void usage(void)
{
  fprintf(stderr,"Usage: dab2eti freq_in_khz\n");
}

/* We need buffers for 5 tranmission frames - the four previous, plus the new */
static struct demapped_transmission_frame_t tfs[5];
static struct tf_info_t tf_info;
static struct ens_info_t ens_info;

int main(int argc, char* argv[])
{
  int freq_khz;
  int i;
  struct wavefinder_t  wf;
  int displayed_lock = 0;

  if (argc != 2) {
    usage();
    return 1;
  }

  freq_khz = atoi(argv[1]);

  init_viterbi();

  wf_init(&wf);

  // TODO: wf_list_devices() or similar

  if (wf_open(&wf,"/dev/wavefinder0") < 0) {
    fprintf(stderr,"Cannot open /dev/wavefinder0\n");
    return 1;
  }

  wf_tune(&wf, freq_khz);

  fprintf(stderr,"Waiting for sync...");

  /* Read (and discard) the first frame - we know it is missing the FIC symbols */
  wf_read_frame(&wf,&tfs[0]);
  if ((wf.sync_locked) && (!displayed_lock)) {
    fprintf(stderr,"LOCKED\n");
    displayed_lock = 1;
  }

  unsigned char* cifs_fibs[16];  /* Each CIF consists of 3072*18 bits */
  unsigned char* cifs_msc[16];  /* Each CIF consists of 3072*18 bits */
  int ncifs = 0;  /* Number of CIFs in buffer - we need 16 to start outputting them */
  int tfidx = 0;  /* Next tf buffer to read to. */
  int ens_info_shown = 0;

  for (i=0;i<64;i++) { ens_info.subchans[i].id = -1; }
  ens_info.CIFCount_hi = 0xff;
  ens_info.CIFCount_lo = 0xff;

  while (1) {
    wf_read_frame(&wf,&tfs[tfidx]);

    if (tfs[tfidx].has_fic) {
      fic_decode(&tfs[tfidx]);
      //fprintf(stderr,"Decoded FIBs - ok_count=%d\n",tfs[tfidx].fibs.ok_count);
      fib_decode(&tf_info,&tfs[tfidx].fibs,12);
      merge_info(&ens_info,&tf_info);
      //dump_tf_info(&tf_info);
    }

    /* We now have a demapped transmission frame, do something.... */
    fic_decode(&tfs[tfidx]);
    fib_decode(&tf_info, &tfs[tfidx].fibs,12);
    merge_info(&ens_info,&tf_info);
    //if (!ens_info_shown) { dump_tf_info(&tf_info); }

    if (ncifs < 16) {
      /* Initial buffer fill */
      cifs_fibs[ncifs]   = tfs[tfidx].fibs.FIB[0];
      cifs_fibs[ncifs+1] = tfs[tfidx].fibs.FIB[3];
      cifs_fibs[ncifs+2] = tfs[tfidx].fibs.FIB[6];
      cifs_fibs[ncifs+3] = tfs[tfidx].fibs.FIB[9];

      cifs_msc[ncifs]   = tfs[tfidx].msc_symbols_demapped[0];
      cifs_msc[ncifs+1] = tfs[tfidx].msc_symbols_demapped[18];
      cifs_msc[ncifs+2] = tfs[tfidx].msc_symbols_demapped[36];
      cifs_msc[ncifs+3] = tfs[tfidx].msc_symbols_demapped[54];

      ncifs += 4;
    } else {
      if (!ens_info_shown) {
        dump_ens_info(&ens_info);
        ens_info_shown = 1;
      }
      /* We have a full history of 16 CIFs, so we can output the
	 oldest TF, which we do one CIF at a time */
      for (i=0;i<4;i++) {
	create_eti(cifs_fibs[0], cifs_msc, &ens_info);

        /* Discard earliest CIF to make room for new one (note we are only copying 15 pointers, not the data) */
        memmove(cifs_fibs,cifs_fibs+1,sizeof(cifs_fibs[0])*15); 
        memmove(cifs_msc,cifs_msc+1,sizeof(cifs_msc[0])*15); 
        /* Add our new CIF to the end */
        cifs_fibs[15] = tfs[tfidx].fibs.FIB[i*3];
        cifs_msc[15] = tfs[tfidx].msc_symbols_demapped[i*18];
      }
    }
    tfidx = (tfidx + 1) % 5;
  }    

  return 0;
}
