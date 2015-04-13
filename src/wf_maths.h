#ifndef _WF_MATHS_H
#define _WF_MATHS_H

int fft_prs(fftw_complex *in, fftw_complex *out, int n);
int ifft_prs(fftw_complex *in, fftw_complex *out, int n);
int mpy3(fftw_complex *srca, fftw_complex *srcb, fftw_complex *dst, int n);
int mpy(fftw_complex *srca, fftw_complex *srcb, fftw_complex *dst, int n);
int mag(fftw_complex *in, double *out, int n);
double maxext(double *in, int n, int *index);
double mean(double *in, int n);
  
#endif
