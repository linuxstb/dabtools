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

struct dab_state_t *dab;

static void eti_callback(uint8_t* eti)
{
  write(1, eti, 6144);
}

int main(int argc, char* argv[])
{
  int freq_khz;
  struct wavefinder_t  wf;
  int displayed_lock = 0;

  if (argc != 2) {
    usage();
    return 1;
  }

  freq_khz = atoi(argv[1]);

  init_dab_state(&dab,&wf);
  dab->device_type = DAB_DEVICE_WAVEFINDER;

  dab->eti_callback=eti_callback;

  wf_init(&wf);

  // TODO: wf_list_devices() or similar

  if (wf_open(&wf,"/dev/wavefinder0") < 0) {
    fprintf(stderr,"Cannot open /dev/wavefinder0\n");
    return 1;
  }

  wf_tune(&wf, freq_khz);

  fprintf(stderr,"Waiting for sync...");

  /* Read (and discard) the first frame - we know it is missing the FIC symbols */
  wf_read_frame(&wf,&dab->tfs[0]);
  if ((wf.sync_locked) && (!displayed_lock)) {
    fprintf(stderr,"LOCKED\n");
    displayed_lock = 1;
  }

  while (1) {
    wf_read_frame(&wf,&dab->tfs[dab->tfidx]);
    dab_process_frame(dab);
  }

  return 0;
}
