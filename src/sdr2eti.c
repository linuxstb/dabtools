/*
This file is part of rtl-dab
trl-dab is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Foobar is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with rtl-dab.  If not, see <http://www.gnu.org/licenses/>.


david may 2012
david.may.muc@googlemail.com

*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <semaphore.h>
#include <signal.h>
#include <rtl-sdr.h>
#include "dab.h"
#include "fic.h"
#include "misc.h"
#include "viterbi.h"
#include "input_sdr.h"

/* RTL Device */
static rtlsdr_dev_t *dev = NULL;

int do_exit = 0;

static pthread_t demod_thread;
static sem_t data_ready;

#define AUTO_GAIN -100
#define DEFAULT_ASYNC_BUF_NUMBER 32

uint32_t corr_counter;
uint32_t ccount=0;

static void sighandler(int signum)
{
  fprintf(stderr, "Signal caught, exiting!\n");
  do_exit = 1;
  rtlsdr_cancel_async(dev);
}

/* We need buffers for 5 tranmission frames - the four previous, plus the new */
static struct demapped_transmission_frame_t tfs[5];
static struct tf_info_t tf_info;
static struct ens_info_t ens_info;

static void *demod_thread_fn(void *arg)
{
  struct sdr_state_t *dab = arg;
  int i,j;
  unsigned char* cifs_msc[16];  /* Each CIF consists of 3072*18 bits */
  unsigned char* cifs_fibs[16];  /* Each CIF consists of 3072*18 bits */
  int ncifs = 0;  /* Number of CIFs in buffer - we need 16 to start outputting them */
  int tfidx = 0;  /* Next tf buffer to read to. */
  int ens_info_shown = 0;
  int locked = 0;
  int okcount = 0;

  for (i=0;i<64;i++) { ens_info.subchans[i].id = -1; ens_info.subchans[i].ASCTy = -1; }
  ens_info.CIFCount_hi = 0xff;
  ens_info.CIFCount_lo = 0xff;

  while (!do_exit) {
    sem_wait(&data_ready);
    int ok = sdr_demod(&tfs[tfidx], dab);
    if (ok) {
      fic_decode(&tfs[tfidx]);
      if (tfs[tfidx].fibs.ok_count > 0) {
        //fprintf(stderr,"Decoded FIBs - ok_count=%d\n",tfs[tfidx].fibs.ok_count);
        fib_decode(&tf_info,&tfs[tfidx].fibs,12);
        merge_info(&ens_info,&tf_info);
        //dump_tf_info(&tf_info);
      }

      if (tfs[tfidx].fibs.ok_count == 12) {
        okcount++;
        if ((okcount >= 10) && (!locked)) { // 10 successive 100% perfect sets of FICs, we are locked.
          locked = 1;
          fprintf(stderr,"Locked with center-frequency %dHz\n",dab->frequency);
        }
      } else {
        okcount = 0;
        if (locked) {
          locked = 0;
          fprintf(stderr,"Lock lost, resetting ringbuffer\n");
          ncifs = 0;
          tfidx = 0;
	  exit(1);
        }
      }

      if (locked) {
        if (ncifs < 16) {
          /* Initial buffer fill */
	  //fprintf(stderr,"Initial buffer fill - ncifs=%d, tfidx=%d\n",ncifs,tfidx);
	  cifs_fibs[ncifs] = tfs[tfidx].fibs.FIB[0];
          cifs_msc[ncifs++] = tfs[tfidx].msc_symbols_demapped[0];
	  cifs_fibs[ncifs] = tfs[tfidx].fibs.FIB[3];
          cifs_msc[ncifs++] = tfs[tfidx].msc_symbols_demapped[18];
	  cifs_fibs[ncifs] = tfs[tfidx].fibs.FIB[6];
          cifs_msc[ncifs++] = tfs[tfidx].msc_symbols_demapped[36];
	  cifs_fibs[ncifs] = tfs[tfidx].fibs.FIB[9];
          cifs_msc[ncifs++] = tfs[tfidx].msc_symbols_demapped[54];
        } else {
          if (!ens_info_shown) {
            dump_ens_info(&ens_info);
	    //for (i=0;i<16;i++) { fprintf(stderr,"cifs_msc[%d]=%d\n",i,(int)cifs_msc[i]); }
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
    }
    //dab_fic_parser(dab->fib,&sinfo,&ana);
    // calculate error rates
    //dab_analyzer_calculate_error_rates(&ana,dab);

    int prev_freq = dab->frequency;
    if (abs(dab->coarse_freq_shift)>1) {
      if (dab->coarse_freq_shift<0)
	dab->frequency = dab->frequency -1000;
      else
	dab->frequency = dab->frequency +1000;
      
      rtlsdr_set_center_freq(dev,dab->frequency);
      
    }
    
    if (abs(dab->coarse_freq_shift) ==1) {
      
      if (dab->coarse_freq_shift<0)
	dab->frequency = dab->frequency -rand() % 1000;
      else
	dab->frequency = dab->frequency +rand() % 1000;
      
      rtlsdr_set_center_freq(dev,dab->frequency);
      //fprintf(stderr,"new center freq : %i\n",rtlsdr_get_center_freq(dev));
      
    } 
    if (abs(dab->coarse_freq_shift)<1 && (abs(dab->fine_freq_shift) > 50)) {
      dab->frequency = dab->frequency + (dab->fine_freq_shift/3);
      rtlsdr_set_center_freq(dev,dab->frequency);
      //fprintf(stderr,"ffs : %f\n",dab->fine_freq_shift);

    }

    //if (dab->frequency != prev_freq) {
    //  fprintf(stderr,"Adjusting centre-frequency to %dHz\n",dab->frequency);
    //}    
    ccount += 1;
    if (ccount == 10) {
      ccount = 0;
      //print_status(dab);
    }
  }
  return 0;
}

static void rtlsdr_callback(uint8_t *buf, uint32_t len, void *ctx)
{
  struct sdr_state_t *dab = ctx;
  int dr_val;
  if (do_exit) {
    return;}
  if (!ctx) {
    return;}
  memcpy(dab->input_buffer,buf,len);
  dab->input_buffer_len = len;
  sem_getvalue(&data_ready, &dr_val);
  if (!dr_val) {
    sem_post(&data_ready);}
}


int main (int argc, char **argv)
{
  struct sigaction sigact;
  uint32_t dev_index = 0;
  int32_t device_count;
  int i,r;
  char vendor[256], product[256], serial[256];
  uint32_t samp_rate = 2048000;

  int gain = AUTO_GAIN;
  struct sdr_state_t dab;

  memset(&dab,0,sizeof(struct sdr_state_t));

  if (argc > 1) {
    dab.frequency = atoi(argv[1]);
  } else {
    dab.frequency = 220352000;
  }
  if (argc > 2) { gain = atoi(argv[2]); } else { gain = AUTO_GAIN; }
  //fprintf(stderr,"%i\n",dab.frequency);

  /*---------------------------------------------------
    Looking for device and open connection
    ----------------------------------------------------*/
  device_count = rtlsdr_get_device_count();
  if (!device_count) {
    fprintf(stderr, "No supported devices found.\n");
    exit(1);
  }

  init_viterbi();
  
  fprintf(stderr, "Found %d device(s):\n", device_count);
  for (i = 0; i < device_count; i++) {
    rtlsdr_get_device_usb_strings(i, vendor, product, serial);
    fprintf(stderr, "  %d:  %s, %s, SN: %s\n", i, vendor, product, serial);
  }
  fprintf(stderr, "\n");
  
  fprintf(stderr, "Using device %d: %s\n",dev_index, rtlsdr_get_device_name(dev_index));
  
  r = rtlsdr_open(&dev, dev_index);
  if (r < 0) {
    fprintf(stderr, "Failed to open rtlsdr device #%d.\n", dev_index);
    exit(1);
  }

  int gains[100];
  int count = rtlsdr_get_tuner_gains(dev, gains);
  fprintf(stderr, "Supported gain values (%d): ", count);
  for (i = 0; i < count; i++)
    fprintf(stderr, "%.1f ", gains[i] / 10.0);
  fprintf(stderr, "\n");

  /*-------------------------------------------------
    Set Frequency & Sample Rate
    --------------------------------------------------*/
  /* Set the sample rate */
  r = rtlsdr_set_sample_rate(dev, samp_rate);
  if (r < 0)
    fprintf(stderr, "WARNING: Failed to set sample rate.\n");
  
  /* Set the frequency */
  r = rtlsdr_set_center_freq(dev, dab.frequency);
  if (r < 0)
    fprintf(stderr, "WARNING: Failed to set center freq.\n");
  else
    fprintf(stderr, "Tuned to %u Hz.\n", dab.frequency);

  /*------------------------------------------------
    Setting gain  
    -------------------------------------------------*/
  if (gain == AUTO_GAIN) {
    r = rtlsdr_set_tuner_gain_mode(dev, 0);
  } else {
    r = rtlsdr_set_tuner_gain_mode(dev, 1);
    r = rtlsdr_set_tuner_gain(dev, gain);
  }
  if (r != 0) {
    fprintf(stderr, "WARNING: Failed to set tuner gain.\n");
  } else if (gain == AUTO_GAIN) {
    fprintf(stderr, "Tuner gain set to automatic.\n");
  } else {
    fprintf(stderr, "Tuner gain set to %0.2f dB.\n", gain/10.0);
  }
  /*-----------------------------------------------
  /  Reset endpoint (mandatory) 
  ------------------------------------------------*/
  r = rtlsdr_reset_buffer(dev);
  /*-----------------------------------------------
  / Signal handler
  ------------------------------------------------*/
  sigact.sa_handler = sighandler;
  sigemptyset(&sigact.sa_mask);
  sigact.sa_flags = 0;
  sigaction(SIGINT, &sigact, NULL);
  sigaction(SIGTERM, &sigact, NULL);
  sigaction(SIGQUIT, &sigact, NULL);
  sigaction(SIGPIPE, &sigact, NULL);
  /*-----------------------------------------------
  / start demod thread & rtl read 
  -----------------------------------------------*/

  fprintf(stderr,"Waiting for sync...\n");

  sdr_init(&dab);
  //dab_fic_parser_init(&sinfo);
  //dab_analyzer_init(&ana);
  pthread_create(&demod_thread, NULL, demod_thread_fn, (void *)(&dab));
  rtlsdr_read_async(dev, rtlsdr_callback, (void *)(&dab),
			      DEFAULT_ASYNC_BUF_NUMBER, DEFAULT_BUF_LENGTH);

  if (do_exit) {
    fprintf(stderr, "\nUser cancel, exiting...\n");}
  else {
    fprintf(stderr, "\nLibrary error %d, exiting...\n", r);}
  rtlsdr_cancel_async(dev);
  //dab_demod_close(&dab);
  rtlsdr_close(dev);
  return 1;
}
