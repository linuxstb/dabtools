#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dab.h"
#ifdef ENABLE_SPIRAL_VITERBI
#include "viterbi_spiral.h"
#else
#include "viterbi.h"
#endif
#include "fic.h"
#include "misc.h"

void init_dab_state(struct dab_state_t **dab, void* device_state, void (* eti_callback)(uint8_t *eti))
{
  int i;

  *dab = calloc(sizeof(struct dab_state_t),1);

  (*dab)->device_state = device_state;
  (*dab)->eti_callback = eti_callback;

  for (i=0;i<64;i++) { (*dab)->ens_info.subchans[i].id = -1; (*dab)->ens_info.subchans[i].ASCTy = -1; }
  (*dab)->ens_info.CIFCount_hi = 0xff;
  (*dab)->ens_info.CIFCount_lo = 0xff;

#ifdef ENABLE_SPIRAL_VITERBI
  /* TODO: What is the maximum size of a sub-channel? */
  (*dab)->v = create_viterbi(768*18);
#else
  init_viterbi();
#endif
}

void dab_process_frame(struct dab_state_t *dab)
{
  int i;

  fic_decode(dab, &dab->tfs[dab->tfidx]);
  if (dab->tfs[dab->tfidx].fibs.ok_count > 0) {
    //fprintf(stderr,"Decoded FIBs - ok_count=%d\n",dab->tfs[dab->tfidx].fibs.ok_count);
    fib_decode(&dab->tf_info,&dab->tfs[dab->tfidx].fibs,12);
    merge_info(&dab->ens_info,&dab->tf_info);
    //dump_tf_info(&dab->tf_info);
  }

  if (dab->tfs[dab->tfidx].fibs.ok_count == 12) {
    dab->okcount++;
    if ((dab->okcount >= 10) && (!dab->locked)) { // 10 successive 100% perfect sets of FICs, we are locked.
      dab->locked = 1;
      //fprintf(stderr,"Locked with center-frequency %dHz\n",sdr->frequency);
      fprintf(stderr,"Locked\n");
    }
  } else {
    dab->okcount = 0;
    if (dab->locked) {
      dab->locked = 0;
      fprintf(stderr,"Lock lost, resetting ringbuffer\n");
      dab->ncifs = 0;
      dab->tfidx = 0;
	  return;
    }
  }

  if (dab->locked) {
    if (dab->ncifs < 16) {
      /* Initial buffer fill */
	  //fprintf(stderr,"Initial buffer fill - dab->ncifs=%d, dab->tfidx=%d\n",dab->ncifs,dab->tfidx);
	  dab->cifs_fibs[dab->ncifs] = dab->tfs[dab->tfidx].fibs.FIB[0];
      dab->cifs_msc[dab->ncifs++] = dab->tfs[dab->tfidx].msc_symbols_demapped[0];
	  dab->cifs_fibs[dab->ncifs] = dab->tfs[dab->tfidx].fibs.FIB[3];
      dab->cifs_msc[dab->ncifs++] = dab->tfs[dab->tfidx].msc_symbols_demapped[18];
	  dab->cifs_fibs[dab->ncifs] = dab->tfs[dab->tfidx].fibs.FIB[6];
      dab->cifs_msc[dab->ncifs++] = dab->tfs[dab->tfidx].msc_symbols_demapped[36];
	  dab->cifs_fibs[dab->ncifs] = dab->tfs[dab->tfidx].fibs.FIB[9];
      dab->cifs_msc[dab->ncifs++] = dab->tfs[dab->tfidx].msc_symbols_demapped[54];
    } else {
      if (!dab->ens_info_shown) {
        dump_ens_info(&dab->ens_info);
	    //for (i=0;i<16;i++) { fprintf(stderr,"cifs_msc[%d]=%d\n",i,(int)cifs_msc[i]); }
        dab->ens_info_shown = 1;
      }
      /* We have a full history of 16 CIFs, so we can output the
         oldest TF, which we do one CIF at a time */
      for (i=0;i<4;i++) {
        create_eti(dab);

        /* Discard earliest CIF to make room for new one (note we are only copying 15 pointers, not the data) */
        memmove(dab->cifs_fibs,dab->cifs_fibs+1,sizeof(dab->cifs_fibs[0])*15);
        memmove(dab->cifs_msc,dab->cifs_msc+1,sizeof(dab->cifs_msc[0])*15);

        /* Add our new CIF to the end */
        dab->cifs_fibs[15] = dab->tfs[dab->tfidx].fibs.FIB[i*3];
        dab->cifs_msc[15] = dab->tfs[dab->tfidx].msc_symbols_demapped[i*18];
      }
    }
    dab->tfidx = (dab->tfidx + 1) % 5;
  }
}
