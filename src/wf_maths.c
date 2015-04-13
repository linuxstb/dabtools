/*
  wfmaths.c

  Copyright (C) 2005 David Crawley

  This file is part of OpenDAB.

  OpenDAB is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  OpenDAB is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with OpenDAB.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <math.h>
#include <complex.h>
#include <fftw3.h>
#include "wf_maths.h"

int fft_prs(fftw_complex *in, fftw_complex *out, int n)
{
	static fftw_plan fp;

	fp = fftw_plan_dft_1d(n, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
	fftw_execute(fp);
	return 0;
}

int ifft_prs(fftw_complex *in, fftw_complex *out, int n)
{
	static fftw_plan bp;

	bp = fftw_plan_dft_1d(n, in, out, FFTW_BACKWARD, FFTW_ESTIMATE);
	fftw_execute(bp);
	return 0;
}

int mpy3(fftw_complex *srca, fftw_complex *srcb, fftw_complex *dst, int n)
{
	int k;

	for (k=0; k < n; k++) {
		*(dst + k) = (*(srca + k) * *(srcb + k));
		*(dst + k) = *(dst + k)/1024;
	} 
	return 0;

}

int mpy(fftw_complex *srca, fftw_complex *srcb, fftw_complex *dst, int n)
{
	int k;

	for (k=0; k < n; k++) {
		*(dst + k) = (*(srca + k) * *(srcb + k));
		*(dst + k) = *(dst + k)/32.0;
	} 
	return 0;

}

double maxext(double *in, int n, int *index)
{
	int i;
	double max;
	
	max = *in;
	*index = 0;

	for (i=1; i < n; i++)
		if (*(in+i) > max) {
			max = *(in+i);
			*index = i;
		}

	return(max);
}

int mag(fftw_complex *in, double *out, int n)
{
	int i;

	for (i=0; i < n; i++)
		*(out+i) = cabs(*(in+i))/n;

	return 0;
}

double mean(double *in, int n)
{
	int i;

	double out = 0.0L;

	for (i=0; i < n; i++)
		out += *(in+i);

	out = out/n;

	return(out);
}

