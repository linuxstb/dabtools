#ifndef _INPUT_SDR_H
#define _INPUT_SDR_H

#include <stdint.h>
#include <fftw3.h>

#include "sdr_fifo.h"

#define DEFAULT_BUF_LENGTH (16 * 16384)
#define GAIN_SETTLE_TIME 0

struct sdr_state_t {
  uint32_t frequency;
  uint8_t input_buffer[DEFAULT_BUF_LENGTH];
  int input_buffer_len;
  uint8_t buffer[196608*2];
  int32_t coarse_timeshift;
  int32_t fine_timeshift;
  int32_t coarse_freq_shift;
  double fine_freq_shift;
  CircularBuffer fifo;
  int8_t real[196608];
  int8_t imag[196608];
  float filt[196608-2662];
  fftw_complex * dab_frame;
  fftw_complex * prs_ifft;
  fftw_complex * prs_conj_ifft;
  fftw_complex * prs_syms;
  /* raw symbols */
  fftw_complex symbols[76][2048];
  /* symbols d-qpsk-ed */
  fftw_complex * symbols_d;

  int32_t startup_delay;
  uint8_t force_timesync;

  /* fault injection */
  double p_e_prior_dep;
  double p_e_prior_vitdec;
  double p_e_after_vitdec;
};

int sdr_demod(struct demapped_transmission_frame_t *tf, struct sdr_state_t *sdr);
void sdr_init(struct sdr_state_t *sdr);

#endif
