/*
    wfsync.c

    Copyright (C) 2005-2008 David Crawley

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
/*
** Use the received Phase Reference Signal from
** the WaveFinder to generate the necessary timing
** values sent back in the control messages with
** bRequest = 5
*/

#include <string.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <math.h>
#include <complex.h>
#include <time.h>

#include <fftw3.h>
#include "input_wf.h"
#include "wf_sync.h"
#include "wf_maths.h"
#include "../wavefinder-driver/wavefinder.h"

/* Lookup tables in wf_prstables.c */
extern const fftw_complex prs1[];
extern const fftw_complex prs2[];
extern const int cos_table[0x800];


static double vmean;
static fftw_complex *idata, *rdata, *mdata, *prslocal, *iprslocal;
static double *magdata;
static unsigned char* prsbuf;
static fftw_complex *cdata;

static int wf_usb_ctrl_msg(int fd, int requesttype, int request, int value, int index, unsigned char *bytes, int size)
{
	ctrl_transfer_t ctl;
	int ret;
	ctl.setup.bRequestType=requesttype;
	ctl.setup.bRequest=request;
	ctl.setup.wValue=value;
	ctl.setup.wIndex=index;
	ctl.size=size;
	/* TODO: this copy shouldn't really be needed */
	memcpy(ctl.data, bytes, size*sizeof(unsigned char));
 
	if ((ret=ioctl(fd,IOCTL_WAVEFINDER_CMSGW,&ctl)) == -1) {
		perror("wf_usb_ctrl_msg");
		exit(EXIT_FAILURE);
	}
	/* else printf("ioctl: ret = %d\n", ret); */
	return 0;
}

/*
** Write a single 16-bit word to a location
** in the WaveFinder's SL11R address space.
** Automatically fills in the data buffer with the values
** from the index and value fields.
**
*/
static int wf_mem_write(int fd, unsigned short addr, unsigned short val)
{
	unsigned char bytes[4];

	bytes[0] = (unsigned char)(addr & 0xff);
	bytes[1] = (unsigned char)((addr >> 8) & 0xff);
	bytes[2] = (unsigned char)(val & 0xff);
	bytes[3] = (unsigned char)((val >> 8) & 0xff);

	/* fprintf(stderr,"%x, %x, %x, %x\n", bytes[0], bytes[1], bytes[2], bytes[3]); */

	return(wf_usb_ctrl_msg(fd, USB_TYPE_VENDOR, SLMEM, addr, val, bytes, 4));
}

static int wf_timing_msg(int fd, unsigned char* bytes)
{
	return(wf_usb_ctrl_msg(fd, USB_TYPE_VENDOR, WFTIMING, 0, 0, bytes, 32));
}  

/*
** Send AFC value to control the VCXO DAC in the WaveFinder
*/

static int wf_afc(int fd, double afcv)
{
	static double offset = 3.25e-1;
	static long lmsec = 0;
	double a;
	int i;
	long cmsec, dt;
	short afc_val;
	struct timespec tp;

        clock_gettime(CLOCK_REALTIME, &tp);
	tp.tv_sec = tp.tv_sec % 1000000;
	cmsec = tp.tv_sec * 1000 + tp.tv_nsec/1000000;
	dt = cmsec - lmsec;

	/* Must be at least 250ms between AFC messages */
	if ((dt > 250L) && ((afcv > 75) || (afcv < -75))) {
		if ((afcv > 350) || (afcv < -350)) {
			a = afcv * -2.2e-5;
			a = a + offset;
			offset = a;
		} else {
			a = 1.0/4096.0;
			if (afcv > 0.0)
				a = -a;
			a = offset + a;
			offset = a;
		}

		i = (int)(a * 65535);
	
		if (i > 0xffff)
			i = 0xffff;
		
		afc_val = i & 0xfffc;
		
		wf_mem_write(fd, DACVALUE, afc_val);
		/* fprintf(stderr,"afcv=%e offset=%e AFC: %04hx\n",afcv,offset,afc_val);  */
	} /* else
		fprintf(stderr,"cmsec = %ld lmsec = %ld dt = %ld afcv=%e: no AFC message\n",cmsec,lmsec,dt,afcv);

	if (dt < 250L)
		fprintf(stderr,"Skipped AFC - interval\n"); */

	lmsec = cmsec;

	return 0;
}

static double wfimp(double irtime, fftw_complex *mdata)
{
	double ir, re_prs, im_prs, ri, rj;
	int jr;
	int a = 0x800, d, a1c, m;
	int i, cosa, cosd, p, q, r, s;
	double j = 0.0, k = 0.0;

	ir = irtime * 4096.0;
	jr = (int)ir & 0x7fffff;

	m = a;

	for (i=0; i < 0x800; i++) {
		a = m;
		m = m + jr;
		a = a >> 12;
		d = a & 0x7ff;
		a -= 512;
		a &= 0x7ff;
		/* Too slow! */
		/* cosa = (1 << 15)*cos((double)a * 2*M_PI/2048.0L); */
		/* cosd = (1 << 15)*cos((double)d * 2*M_PI/2048.0L); */
		cosa = cos_table[a];
		cosd = cos_table[d];
		re_prs = creal(mdata[i]);
		//fprintf(stderr,"HERE: i=%d\n",i);
		im_prs = cimag(mdata[i]);
		//fprintf(stderr,"HERE: im_prs=%d\n",im_prs);
		//fprintf(stderr,"HERE: cosd=%d\n",cosd);
		p = im_prs * cosd;
		q = im_prs * cosa;
		r = re_prs * cosa;
		s = re_prs * cosd;
		s = s - q;
		r = r + p;
		k += s;
		j += r;
	}
	ri = sqrt((k*k) + (j*j));
	a1c = 0x800 * 32767;

	rj = 1/(double)a1c * ri;
	/* printf("rj = %e\n",rj); */
	return(rj);
}

static int wfpk(double *magdata, int indx)
{
	double b, c, d, bmax = 0.0;
	int a, i, l;
	int pts = 0x800;
	int res = 0;

	a = indx - 0x1f8 + pts;
	b = 0x1f8 / 2;
	l = a + b - 1;

	for (i = 0; i < (2 * 0x1f8); i++) {
		c = *(magdata + ((pts - 0x1f8 / 2 + a) % 0x800));
		d = *(magdata + (l % 0x800));
		b = b + d - c;
		if (b > bmax) {
			bmax = b;
			res = a % 0x800;
		}
		l++;
		a++;
	}
  
	if (res > 0x400)
		res = res - 0x800;

	/* printf("wfpk: res=%d\n",res); */
	return (res);
}

static int wfref(int indx, int pts, fftw_complex* outp, const fftw_complex* inp)
{
	if (indx == 0) {
		memcpy(outp, inp, pts*sizeof(fftw_complex));
	} else {
		if (indx > 0) {
			memcpy(outp, inp + pts - indx, indx*sizeof(fftw_complex));
			memcpy(outp + indx, inp, (pts - indx)*sizeof(fftw_complex));
		} else {
			if (indx < 0) {
				indx = -indx;
				memcpy(outp + pts - indx , inp, indx*sizeof(fftw_complex));
				memcpy(outp, inp + indx, (pts - indx)*sizeof(fftw_complex));
			}
		}
	}
	return 0;
}

/*
** Rescales and removes offset from the PRS read from the WaveFinder
*/
static int prs_scale(unsigned char* prsdata, fftw_complex* idata)
{
	int k;
	unsigned char *dptr;

	dptr = prsdata;

	for (k = 0; k < 0x800; k++)  
		/* *(idata + k) = 0 + 0.732421875*(*(dptr + k) - 128)*I; */
		*(idata + k) = 0 + (*(dptr + k) - 128)*I;

	return 0;
}

static double raverage(double ir)
{
	static double prev_ir = 0, sa[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	static int j = 0, k = 0;
	int d, i;
	double t;

	/* printf("prev_ir = %e ir = %e k = %d\n",prev_ir,ir,k); */
	if (fabs(prev_ir) > 350) {
		k = 0;
		j = 0;
	}

	sa[k++] = ir;

	if (k == 8) {
		k = 0;
		j = 1;
	}

	if (j != 0)
		d = 8;
	else
		d = k;

	t = 0.0;

	if (d > 0) {
		for  (i=0; i < d; i++)
			t = t + sa[i];
	}

	prev_ir = t / d;
	/* printf("sa=%e %e %e %e %e %e %e %e\n",sa[0],sa[1],sa[2],sa[3],sa[4],sa[5],sa[6],sa[7]);
	   printf("Post raverage prev_ir = %e\n",prev_ir); */
	return (prev_ir);
}

static int wf_sync(struct wavefinder_t* wf, unsigned char* prsb)
{
	static int cnt = 0;
	static int lckcnt = 3;
	static long lms = 0L;
	long ems, dt;
	int i, j, indxv = 0, indx, indx_n = 0, lcnt;
	double t,u,v, vi, vs, max, maxv, stf, ir, c;
	short w1, w2;
	unsigned short cv;
	int pts = 0x800;
	unsigned char imsg[] = {0x7f, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
				0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00};

	fftw_complex tc;
	struct timespec tp;

	prs_scale(prsb, idata);
	ifft_prs(idata, rdata, pts);

	/* The real part of the ifft has reverse order (apart from
	   the DC term) using fftw compared with the Intel SPL functions
	   used in the w!nd*ws software. */
	for (i=1; i < pts/2; i++) {
		t = creal(*(rdata+i));
		*(rdata+i) = *(rdata+i) - t + creal(*(rdata+pts-i));
		*(rdata+pts-i) = *(rdata+pts-i) - creal(*(rdata+pts-i)) + t;
	}

	if (wf->sync_locked) {
		wfref(0x0, 0x800, prslocal, prs1);
		lcnt = 1;
	} else {
		wfref(0x0c, 0x800, prslocal, prs1);
		lcnt = 25;
	}
	/* Copy 0x18 complex points from start of data and append to the end */
	for (i=0; i < 0x18; i++)
		*(prslocal + 0x800 + i) = *(prslocal + i);

	maxv = 0;

	for (j=0; j < lcnt; j++) {
		mpy3(rdata, prslocal + j, cdata, pts);
		fft_prs(cdata, mdata, pts);
		/* The real part of the fft has reverse order using fftw compared
		   with the Intel SPL functions used in the w!nd*ws software */
		for (i=1; i < pts/2; i++) {
			tc = *(mdata+i);
			*(mdata+i) = *(mdata+pts-i);
			*(mdata+pts-i) = tc;
		}
		mag(mdata, magdata, pts);
		max = maxext(magdata, pts, &indx);
		vmean = mean(magdata, pts);
 
		if ((vmean * 12) > max)
			max = 0;

		if (wf->sync_locked) {
			indx_n = wfpk(magdata, indx);
			indx_n = indx_n/15;

			if (indx_n > 12)
				indx_n = 80;
			else
				if (indx_n < -12)
					indx_n = -80;
			
			indx_n = -indx_n;
		}

		if (max > maxv) {
			maxv = max;
			indxv = indx;
		}
	}
	if (indxv < 0x400)
		indxv = -indxv;
	else
		indxv = 0x800 - indxv;

	c = 4.8828125e-7;

	if (wf->sync_locked)
		c = c * indx_n;
	else
		c = c * indxv;

	wfref(-indxv, 0x800, iprslocal, prs2);

	mpy(idata, iprslocal, mdata, pts);

	fft_prs(mdata, rdata, pts);
	/* The real part of the fft has reverse order using fftw compared
	   with the Intel SPL functions used in the w!nd*ws software */
	for (i=1; i < pts/2; i++) {
		tc = *(rdata+i);
		*(rdata+i) = *(rdata+pts-i);
		*(rdata+pts-i) = tc;
	}

	mag(rdata, magdata, pts);
	max = maxext(magdata, pts, &indx);
	vmean = mean(magdata, pts);

	if ((14.0 * vmean) > max)
		max = 0.0;

	ir = indx;

	if (ir >= 0x400)
		ir -= 0x800;

	stf = 0.666666666;

	do {
		stf = stf/2;
		v = ir - stf;
    
		vi = wfimp(v, mdata);
		if (vi > max) {
			max = vi;
			ir = v;
		}
		v = ir + stf;
		vs = wfimp(v, mdata);
		if (vs > max) {
			max = vs;
			ir = v;
		}
	} while ((1000*stf) > 2.5e-2); 

	ir = ir * 1000.0;

	if ((fabs(c) < (2.4609375e-4/2)) && (fabs(ir) < 350)) {
		if (lckcnt == 0) {
			wf->sync_locked = 1;
		} else {
			lckcnt--;
			wf->sync_locked = 0;
		}
	} else {
		lckcnt = 3;
		wf->sync_locked = 0;
	}

	i = c * -8192000.0;

        clock_gettime(CLOCK_REALTIME, &tp);
	tp.tv_sec = tp.tv_sec % 1000000;
	ems = tp.tv_sec * 1000 + tp.tv_nsec/1000000;
	dt = ems - lms;
	/* Allow at least 60ms between these messages */
	if (dt > 60L) {
		if (i != 0) {
			i = i + 0x7f;
			if (i > 0) {
				cv = i;
				if (cv > 0xff)
					cv = 0xff;
			} else cv = 0;

			cv = 0x1000 | cv;
			wf_mem_write(wf->fd, OUTREG0, cv);
		}
	}
	lms = ems;

	ir = raverage(ir);
	wf_afc(wf->fd, ir);
	
	t = ir * 81.66400146484375;
	w1 = (int)t & 0xffff;
	u = ir * 1.024;
	w2 = (int)u & 0xffff;

	memcpy(imsg + 2, wf->symstr, 10 * sizeof(unsigned char));
	cnt++;
	imsg[24] = w1 & 0xff;
	imsg[25] = (w1 >> 8) & 0xff;
	imsg[26] = w2 & 0xff;  
	imsg[27] = (w2 >> 8) & 0xff;
	wf_timing_msg(wf->fd, imsg);

	return 0;
}

/*
** Assemble the appropriate four packets which form
** the Phase Reference Symbol and then call
** wf_sync() to do the synchronization
*/ 
int wf_prs_assemble(struct wavefinder_t* wf, unsigned char *rdbuf)
{
	static unsigned char seen_flags;
	int blk;

	blk = *(rdbuf+7); /* Block number: 0-3 */

	if (blk == 0x00) {
		seen_flags = 1;
		memcpy(prsbuf, rdbuf+12, 512);
	}  else {
		seen_flags |= 1 << blk;
		/* Copy block data, excluding header */ 
		memcpy(prsbuf+(blk*512), rdbuf+12, 512);
    
		/* fprintf(stderr,"blk = %d, seen_flags = %d tinit = %d\n",blk,seen_flags,tinit); */
		if (seen_flags == 15) {
			seen_flags = 0;
			wf_sync(wf, prsbuf);
		}
	}
	return 0;
}

int wfsyncinit(void)
{

	/* alloc storage for impulse response calculations */
	if ((idata = calloc(0x800, sizeof(fftw_complex))) == NULL) {
		fprintf(stderr,"wfsync: calloc failed for idata");
		return -1;
	}

	if ((cdata = calloc(0x800, sizeof(fftw_complex))) == NULL) {
		fprintf(stderr,"wfsync: calloc failed for cdata");
		return -1;
	}

	if ((rdata = calloc(0x800, sizeof(fftw_complex))) == NULL) {
		fprintf(stderr,"wfsync: calloc failed for rdata");
		return -1;
	}

	if ((mdata = calloc(0x800, sizeof(fftw_complex))) == NULL) {
		fprintf(stderr,"wfsync: calloc failed for mdata");
		return -1;
	}

	if ((prslocal = calloc(0x820, sizeof(fftw_complex))) == NULL) {
		fprintf(stderr,"wfsync: calloc failed for prslocal");
		return -1;
	}

	if ((iprslocal = calloc(0x820, sizeof(fftw_complex))) == NULL) {
		fprintf(stderr,"wfsync: calloc failed for iprslocal");
		return -1;
	}

	if ((magdata = calloc(0x800, sizeof(double))) == NULL) {
		fprintf(stderr,"wfsync: calloc failed for magdata");
		return -1;
	}

	if ((prsbuf = calloc(0x800, sizeof(unsigned char))) == NULL) {
		fprintf(stderr,"wfsync: calloc failed for prsbuf");
		return -1;
	}

	return 0;
}
