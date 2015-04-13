/*
 *      wavefinder.c  --  Psion WaveFinder driver for Linux 2.6 kernel
 *
 *      Copyright (C) 2005-2010 David Crawley <degressive@yahoo.co.uk>
 *
 *      Modified March/April 2015 by Dave Chapman <dave@dchapman.com> to
 *      incorporate some of David Crawley's opendab application code in the driver
 *      to present a slightly higher-level API.
 *
 *      This program is free software; you can redistribute it and/or modify
 *      it under the terms of the GNU General Public License as published by
 *      the Free Software Foundation; either version 3 of the License, or
 *      (at your option) any later version.
 *
 *      This program is distributed in the hope that it will be useful,
 *      but WITHOUT ANY WARRANTY; without even the implied warranty of
 *      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *      GNU General Public License for more details.
 *
 *      You should have received a copy of the GNU General Public License
 *      along with this program; if not, If not, see <http://www.gnu.org/licenses/>
 *      or write to the Free Software Foundation, Inc., 675 Mass Ave, Cambridge,
 *      MA 02139, USA.
 * *
 *      Based partly on dabusb.c by Deti Fliegl and audio.c by Thomas Sailer
 *
 */
/*****************************************************************************/

#include <linux/types.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/socket.h>
#include <linux/list.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <linux/delay.h>
#include <linux/usb.h>
#include <linux/firmware.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 38)
#include <linux/smp_lock.h>
#endif

#include "wavefinder.h"
#include "wfsl11r.h"

/*
 * Version Information
 */
#define DRIVER_VERSION "v2.0"
#define DRIVER_AUTHOR "David Crawley degressive@yahoo.co.uk"
#define DRIVER_DESC "WaveFinder Driver for Linux (c)2005-2015"

#define err(format, arg...) printk(KERN_ALERT format "\n", ## arg)
#define dbg(format, arg...) printk(KERN_ALERT format "\n", ## arg)

#define WAVEFINDER_MINOR 240
#define WAVEFINDER_VERSION 0x1000
#define WAVEFINDER_VENDOR 0x9cd
#define WAVEFINDER_PRODUCT 0x2001

static struct usb_driver wavefinder_driver;

static struct usb_device_id wavefinder_ids [] = {
	{ USB_DEVICE(0x09cd, 0x2001) },
	{ }						/* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, wavefinder_ids);

#define NURBS 2

typedef enum { _stopped=0, _started } driver_state_t;

typedef struct
{
        struct semaphore mutex;
	struct usb_device *usbdev;
	wait_queue_head_t wait;
	wait_queue_head_t remove_ok;
	spinlock_t lock;
	atomic_t pending_io;
	driver_state_t state;
	int remove_pending;
	unsigned int overruns;
	int readptr;
	int opened;
        int devnum;
        int count; 
        int pipesize;                     /* Calculate this once */
        struct urb *wfurb[NURBS];
 	unsigned int last_order;
        int allocated[NURBS];
        int running[NURBS];
	char *recbuf;
	unsigned char prev_h[12];         /* Stores previous 12-byte packet header */
} wavefinder_t, *pwavefinder_t;

typedef struct 
{
	pwavefinder_t s;
	struct urb *purb;
	struct list_head buff_list;
} buff_t,*pbuff_t;


#define _WAVEFINDER_IF 0
#define _WAVEFINDER_ISOPIPE 0x81
#define _ISOPIPESIZE	16768

#ifdef DEBUG 
static void dump_urb(struct  urb* purb)
{
	int i, j;

	//dbg("urb                   :%p", purb);
	//dbg("dev                   :%p", purb->dev);
	//dbg("pipe                  :%08X", purb->pipe);
	//dbg("status                :%d", purb->status);
	//dbg("transfer_flags        :%08X", purb->transfer_flags);
	//dbg("transfer_buffer       :%p", purb->transfer_buffer);
	//dbg("transfer_buffer_length:%d", purb->transfer_buffer_length);
	//dbg("actual_length         :%d", purb->actual_length);
	//dbg("setup_packet          :%p", purb->setup_packet);
	//dbg("start_frame           :%d", purb->start_frame);
	//dbg("number_of_packets     :%d", purb->number_of_packets);
	//dbg("interval              :%d", purb->interval);
	//dbg("error_count           :%d", purb->error_count);
	//dbg("context               :%p", purb->context);
	//dbg("complete              :%p", purb->complete);
	for (i=0; i < purb->number_of_packets; i++) {
		//dbg("iso offset %d = %d",i,purb->iso_frame_desc[i].offset);
		//dbg("iso length %d = %d",i,purb->iso_frame_desc[i].length);
		//dbg("iso status %d = %d",i,purb->iso_frame_desc[i].status);
		dbg("packet %d (off=%d,len=%d,alen=%d,st=%d) iso bytes %02X %02X %02X %02X",
		    i,
		    purb->iso_frame_desc[i].offset,
		    purb->iso_frame_desc[i].length,
		    purb->iso_frame_desc[i].actual_length,
		    purb->iso_frame_desc[i].status,
		    *((unsigned char*)purb->transfer_buffer+purb->iso_frame_desc[i].offset),
		    *((unsigned char*)purb->transfer_buffer+purb->iso_frame_desc[i].offset+1),
		    *((unsigned char*)purb->transfer_buffer+purb->iso_frame_desc[i].offset+2),
		    *((unsigned char*)purb->transfer_buffer+purb->iso_frame_desc[i].offset+3));
	}
}
#endif

static int wavefinder_prepare_urb(pwavefinder_t s, struct urb *purb)
{
	int j, offs;
	int packets = _ISOPIPESIZE / s->pipesize;

	//dbg("wavefinder_prepare_urb");

	for (j = offs = 0; j < packets; j++, offs += s->pipesize) {
		purb->iso_frame_desc[j].offset = offs;
		purb->iso_frame_desc[j].length = s->pipesize;
	}
	purb->interval = 1;
	return 0;
}

static void wavefinder_iso_complete(struct urb *purb, struct pt_regs *regs)
{
	pwavefinder_t s = purb->context;
	int i,j, subret, pkts = 0;
	int len;
	char *bptr, *uptr;
	const unsigned char vhdr[] = {0x0c, 0x62};  /* Valid packets start like this */ 

	//dump_urb(purb);

	j=0;
	while ((purb != s->wfurb[j]) && (j < NURBS))
		j++;

	if (j < NURBS)
		s->running[j] = 0;
	else
		dbg("wavefinder_iso_complete: invalid urb %d", j);

	/* process if URB was not killed */
	if (purb->status != -ENOENT) {
		//dbg("wavefinder_iso_complete urb %d",j);
		for (i = 0; i < purb->number_of_packets; i++) {
			uptr = purb->transfer_buffer+purb->iso_frame_desc[i].offset;
			if (!purb->iso_frame_desc[i].status) {
				len = purb->iso_frame_desc[i].actual_length;
				if (len <= s->pipesize) {
					/*  Won't copy bad/duplicate packets */
					if (!strncmp(vhdr, uptr, 2) && strncmp(uptr, s->prev_h, 12)) {
						bptr = s->recbuf + (s->count % (_ISOPIPESIZE * NURBS));
						memcpy(bptr, uptr, len);
						memcpy(s->prev_h, bptr, 12);
						s->count += len;
						pkts++;
					}
				} else
					dbg("wavefinder_iso_complete: invalid len %d", len);
			} else
				dbg("wavefinder_iso_complete: corrupted packet status: %d", purb->iso_frame_desc[i].status);
			
		}
	}
	
	wavefinder_prepare_urb(s, s->wfurb[j]);
	if ((subret = usb_submit_urb(s->wfurb[j], GFP_ATOMIC)) == 0) {
		s->running[j] = 1;
	} else
		dbg("wavefinder_iso_complete: usb_submit_urb failed %d\n", subret);
	
	if (atomic_dec_and_test(&s->pending_io) && !s->remove_pending && s->state != _stopped) {
		s->overruns++;
		err("overrun (%d)", s->overruns);
	}

	wake_up(&s->wait);
}


static int wavefinder_free_buffers(pwavefinder_t s)
{
	int i;

	dbg("wavefinder_free_buffers");

	for (i=0; i < NURBS; i++) {
		if (s->allocated[i]) {
			//dbg("kfree(s->wfurb[%d]->transfer_buffer)",i);
			kfree(s->wfurb[i]->transfer_buffer);
			//dbg("usb_free_urb(s->wfurb[%d])",i);
			usb_free_urb(s->wfurb[i]);
			s->allocated[i] = 0;
		}
	}

	if (!s->recbuf) {
		//dbg("kfree(s->recbuf)");
		kfree(s->recbuf);
	}

	return 0;
}

static int wavefinder_alloc_buffers(pwavefinder_t s)
{
	unsigned int pipe = usb_rcvisocpipe(s->usbdev, _WAVEFINDER_ISOPIPE);
	int pipesize = usb_maxpacket(s->usbdev, pipe, usb_pipeout(pipe));
	int packets;
	int transfer_buffer_length;
	int i,j;
	
	dbg("wavefinder_alloc_buffers");

	if (!pipesize)	{
		err("ISO-pipe has size 0!!");
		return -ENOMEM;
	}
	packets = _ISOPIPESIZE / pipesize;
	transfer_buffer_length = packets * pipesize;
	
	/*dbg("wavefinder_alloc_buffers pipesize:%d packets:%d transfer_buffer_len:%d",
	  pipesize, packets, transfer_buffer_length);*/

	s->pipesize = pipesize;

	for (i=0; i < NURBS; i++) {
		s->wfurb[i] = usb_alloc_urb(packets,GFP_KERNEL);
		if (!s->wfurb[i]) {
			err("usb_alloc_urb == NULL");
			goto err;
		}
		s->wfurb[i]->transfer_buffer = kmalloc(transfer_buffer_length, GFP_KERNEL);
		if (!s->wfurb[i]->transfer_buffer) {
			kfree(s->wfurb[i]->transfer_buffer);
			err("kmalloc(%d)==NULL", transfer_buffer_length);
			goto err;
		}

		s->wfurb[i]->transfer_buffer_length = transfer_buffer_length;
		s->wfurb[i]->number_of_packets = packets;
		s->wfurb[i]->interval = 1;
		s->wfurb[i]->complete = (usb_complete_t)wavefinder_iso_complete;
		s->wfurb[i]->context = s;
		s->wfurb[i]->dev = s->usbdev;
		s->wfurb[i]->pipe = pipe;
		s->wfurb[i]->transfer_flags = URB_ISO_ASAP;

		for (j = 0; j < packets; j++) {
			s->wfurb[i]->iso_frame_desc[j].offset = j * pipesize;
			s->wfurb[i]->iso_frame_desc[j].length = pipesize;
		}
		s->allocated[i] = 1;
		s->running[i] = 0;
	}
	
	s->recbuf = kmalloc(transfer_buffer_length*NURBS, GFP_KERNEL);
	
	if (!s->recbuf) {
		kfree(s->recbuf);
		err("kmalloc(%d)==NULL (recbuf)", transfer_buffer_length*NURBS);
		goto err;
	}

	return 0;

err:
	wavefinder_free_buffers(s);
	return -ENOMEM;
}


static int wavefinder_stop(pwavefinder_t s)
{
        int i;

	dbg("wavefinder_stop");

	s->state = _stopped;
	for (i=0; i < NURBS; i++) {
		if (s->running[i]) {
			usb_kill_urb(s->wfurb[i]);
			s->running[i] = 0;
		}
	}

	s->pending_io.counter = 0;
	return 0;
}


static int wavefinder_start_receive(pwavefinder_t s)
{
        int i, subret = 0;
        int allocated = 0;

	//dbg("wavefinder_start_receive");

        for (i=0; i < NURBS; i++)
                allocated += s->allocated[i];

	if (allocated != NURBS && s->state != _started) {
		if (wavefinder_alloc_buffers(s) < 0)
			return -ENOMEM;
		//wavefinder_stop(s);
		s->state = _started;
		s->readptr = 0;

		for (i=0; i < NURBS; i++) {
			wavefinder_prepare_urb(s, s->wfurb[i]);
			//dbg("wavefinder_start_receive: submitting urb %d",i);
			if (s->running[i] == 0 && (subret = usb_submit_urb(s->wfurb[i], GFP_KERNEL)) == 0)
				s->running[i] = 1;
			else {
				s->running[i] = 0;
				printk(KERN_DEBUG "wavefinder_start_receive: usb_submit_urb failed %d\n", subret);
			}
		}
	}

	return 0;
}


static ssize_t wavefinder_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
        pwavefinder_t s = (pwavefinder_t) file->private_data;
	DECLARE_WAITQUEUE(wait, current);
	ssize_t ret = 0;
	unsigned long flags;
	int cnt = 0, err, rem;

        /* dbg("wavefinder_read"); */

	if (*ppos)
		return -ESPIPE;

	if (!access_ok(VERIFY_WRITE, buf, count))
		return -EFAULT;

	add_wait_queue(&s->wait, &wait);

	while (count > 0) {
		spin_lock_irqsave(&s->lock, flags);
		rem = s->count - s->readptr;
		if (count >= rem)
			cnt = rem;
		else
			cnt = count;
		/* set task state early to avoid wakeup races */
		if (cnt <= 0)
			__set_current_state(TASK_INTERRUPTIBLE);
		spin_unlock_irqrestore(&s->lock, flags);

		if (cnt > count)
			cnt = count;
		if (cnt <= 0) {
			if (wavefinder_start_receive(s)) {
				if (!ret)
					ret = -ENODEV;
				break;
			}
			if (file->f_flags & O_NONBLOCK) {
				if (!ret)
					ret = -EAGAIN;
				break;
			}
			schedule();
			if (signal_pending(current)) {
				if (!ret)
					ret = -ERESTARTSYS;
				break;
			}
			continue;
		}

		//dbg("wavefinder_read: copy_to_user %d bytes s->readptr=%d s->count=%d",cnt,s->readptr,s->count);
		if ((err = copy_to_user(buf, s->recbuf + s->readptr, cnt))) {
			dbg("copy_to_user err = %d",err);
			if (!ret)
				ret = -EFAULT;
			goto err;
		}

		s->readptr += cnt;
		count -= cnt;
		buf += cnt;
		ret += cnt;

		if (s->readptr == s->count) {
			s->readptr = 0;
			spin_lock_irqsave(&s->lock, flags);	
			s->count = 0;
			spin_unlock_irqrestore(&s->lock, flags);
		}
	}
	__set_current_state(TASK_RUNNING);
	remove_wait_queue(&s->wait, &wait);

err:
	return ret;
}


static int wf_usb_ctrl_msg(pwavefinder_t s, int requesttype, int request, int value, int index, unsigned char *bytes, int size)
{
	return usb_control_msg(s->usbdev, usb_sndctrlpipe( s->usbdev, 0 ), 
			      request, requesttype, value, index, bytes, size, HZ);
}

/*
** Convenience function that calls wf_usb_ctrl_msg() with
** bmRequestType = USB_TYPE_VENDOR and bmRequest = SLMEM
*/
static int wf_sendmem(pwavefinder_t s, int value, int index, unsigned char *bytes, int size)
{
	/* int i;
	   printf("SLMEM Value=%#hx, Index=%d, Length=%d\n",value, index, size);
	   for (i=0; i < size; i++)
	   printf("%#0hhx ",*(bytes+i));
	   printf("\n");*/
	return(wf_usb_ctrl_msg(s, USB_TYPE_VENDOR, SLMEM, value, index, bytes, size));
}

/*
** Write a single 16-bit word to a location
** in the WaveFinder's SL11R address space.
** Automatically fills in the data buffer with the values
** from the index and value fields.
**
*/
static int wf_mem_write(pwavefinder_t s, unsigned short addr, unsigned short val)
{
	unsigned char bytes[4];

	bytes[0] = (unsigned char)(addr & 0xff);
	bytes[1] = (unsigned char)((addr >> 8) & 0xff);
	bytes[2] = (unsigned char)(val & 0xff);
	bytes[3] = (unsigned char)((val >> 8) & 0xff);

	/* fprintf(stderr,"%x, %x, %x, %x\n", bytes[0], bytes[1], bytes[2], bytes[3]); */

	return(wf_usb_ctrl_msg(s, USB_TYPE_VENDOR, SLMEM, addr, val, bytes, 4));
}

/*
** Send a tuning message
*/
static int wf_tune_msg(pwavefinder_t s, unsigned int reg, unsigned char bits, unsigned char pll, int lband)
{
#define TBUFSIZE 12
	unsigned char tbuf[TBUFSIZE];

	memset(tbuf, 0, TBUFSIZE);  /* zero buffer */
	tbuf[0] = reg & 0xff;
	tbuf[1] = (reg >> 8) & 0xff;
	tbuf[2] = (reg >> 16) & 0xff;
	tbuf[3] = (reg >> 24) & 0xff;  
	tbuf[4] = bits;     /* no. of reg. bits */
	tbuf[6] = pll;      /* 1 = select LMX1511, 0 = select LMX2331A */
	tbuf[8] = lband;
	tbuf[11] = 0x10;  /* TODO: Check what this byte does, if anything */
	return(wf_usb_ctrl_msg(s, USB_TYPE_VENDOR, WFTUNE, 0, 0, tbuf, TBUFSIZE));
	/* printf("WTUNE Length=%d\n",TBUFSIZE);
	   for (i=0; i < TBUFSIZE; i++)
	   printf("%#0hhx ",*(tbuf+i));
	   printf("\n"); */
}

/* 
** Send a 32 byte timing control/symbol selection message
*/
static int wf_timing_msg(pwavefinder_t s, unsigned char* bytes)
{
	return(wf_usb_ctrl_msg(s, USB_TYPE_VENDOR, WFTIMING, 0, 0, bytes, 32));
}  

/*
** Send a pair of 64 byte request=1, request=2 messages
** TODO: work out what these messages do etc.
*/
static int wf_r2_msg(pwavefinder_t s, unsigned char* bytes)
{
	return(wf_usb_ctrl_msg(s, USB_TYPE_VENDOR, 2, 0, 0x80, bytes, 64));
}

static int wf_r1_msg(pwavefinder_t s, unsigned char* bytes)
{
	return(wf_usb_ctrl_msg(s, USB_TYPE_VENDOR, 1, 0, 0x80, bytes, 64));
}

/*
** Boot the WaveFinder DSPs.
**
** Each of the two TMS320VC5402 DSPs are booted via their Host Port Interfaces (HPI-8)
** see chapter 4 of Vol.5 (Enhanced Peripherals) of TI's TMS320C54x DSP Reference Set
** (Document SPRU302) from www.ti.com.
**
** This requires the two files rsDSPa.bin and rsDSPb.bin which contain executable code.
** TODO: It would be interesting to attempt to write GPL'ed Open Source versions.
** TODO: Maybe these filenames should be defined in a configuration file.
**
** Code is loaded via the HPI and then the entry point address is
** loaded into address 0x007f - changing the contents of this address
** from zero causes execution to start at the address specified by the
** new contents (see TI document SPRA618A).
**
** Locations of the HPI registers in the SL11R address space are defined in wfsl11r.h
**
** Note that the TMS320VC5402 has an 8-bit wide HPI (HPI-8) so 16-bit words have to
** be transferred a byte at a time.  The WaveFinder firmware dumbly writes anything it
** receives to the HPI. So, in order to send one 16-bit word, we actually have to send
** two 16-bit words. If the word we want to send is 0xmmll, we need to send 0xmm00 0xll00
**
** Also: 1. The first word of the rsDSP[ab].bin files is the entry point.
**       2. Only bytes from offset 0x80 to 0x2000 are uploaded from each file.
*/
static int wf_boot_dsps(pwavefinder_t s)
{
	/* The other software uses this length (16 bit words) for the data transfer stage */ 
#define USBDATALEN 31

	/*  unsigned short ctrlreg[2] = {HPIC_B, HPIC_A}; */
	int fw_offsets[2] = { 32767, 0 };  /* We load rsDSPb.bin first, then rsDSPa.bin */
	unsigned short addrreg[2] = {HPIA_B, HPIA_A};
	unsigned short datareg[2] = {HPID_B, HPID_A};
	unsigned short entry_pt[2];
	unsigned short rbuf[3];
	unsigned char ubuf[USBDATALEN*2];
	const struct firmware *fw = NULL;
	const unsigned char *cbuf;
	int i,j,remain, left, frames;

	/* This loads 0x0000 into DSP B at address 0x00e0
	   But why ?  Not even the HPI control reg has been loaded yet
	   TODO: Check the reason for this */

	rbuf[0] = HPIA_B; /* Load HPI address register */
	rbuf[1] = 0x00e0;
	rbuf[2] = 0x0000;
	wf_sendmem(s, 0, 0, (unsigned char*)&rbuf, 6);

	rbuf[0] = HPID_B; /* Load HPI data register */
	rbuf[1] = 0x0000;
	rbuf[2] = 0x0000;
	wf_sendmem(s, 0, 0, (unsigned char*)&rbuf, 6);  

	rbuf[0] = HPIC_B; /* Load HPI control register */
	rbuf[1] = 0x0001;
	rbuf[2] = 0x0001;
	wf_sendmem(s, 0, 0, (unsigned char*)&rbuf, 6);

	rbuf[0] = HPIC_A; /* Load HPI control register */
	rbuf[1] = 0x0001;
	rbuf[2] = 0x0001;
	wf_sendmem(s, 0, 0, (unsigned char*)&rbuf, 6);

	if (request_firmware(&fw, "wavefinder.fw", &s->usbdev->dev)) {
		printk("wavefinder:%s: firmware wavefinder.fw not found\n", __func__);
		return -1; /* FIXME */
	}

	if (fw->size != 32767*2) {
		printk("wavefinder: Wrong firmware size %d (expected %d)\n",(int)fw->size,32767*2);
		return -1; /* FIXME */
	}

	for (i = 0; i < 2; i++) {
		cbuf = fw->data + fw_offsets[i];
		rbuf[0] = addrreg[i]; /* Load HPI address register - actually at 0x0080 because of inc. */
		rbuf[1] = 0x007f;
		rbuf[2] = 0x0000;
		wf_sendmem(s, 0, 0, (unsigned char*)&rbuf, 6);

		frames = 0;
		memset(ubuf, 0, USBDATALEN*2);  /* zero buffer - unnecessary but less confusing to debug */    
		ubuf[0] = datareg[i] & 0xff;
		ubuf[1] = (datareg[i] >> 8) & 0xff;
		remain = 0x2000 - 0x80 + 1;         /* bytes to transfer */
		while (remain > 0) {
			if (remain >= USBDATALEN) {
				for (j=2; j < USBDATALEN*2; j+=2)
					ubuf[j] = *(cbuf + 0x2001 - remain--);
				wf_sendmem(s, datareg[i], 0, (unsigned char*)&ubuf, USBDATALEN*2);
				frames++;
			} else {
				left = remain*2;
				for (j=0; j < left; j+=2)
					ubuf[j+2] = *(cbuf + 0x2000 - remain--);
				wf_sendmem(s, datareg[i], 0, (unsigned char*)&ubuf, left);
				frames++;
			}
		}
		/* printf("\n Frames = %d\n",frames); */
		entry_pt[i] = *cbuf | (*(cbuf + 1) << 8);
	}

	release_firmware(fw);

	/* With the code uploaded, load address 0x007f
	   of each DSP with the appropriate entry point to begin execution */

	rbuf[0] = HPIA_A; /* Load HPI address register for DSP A */
	rbuf[1] = 0x007e; /* = 0x007f after increment */
	rbuf[2] = 0x0000;
	wf_sendmem(s, 0, 0, (unsigned char*)&rbuf, 6);

	rbuf[0] = HPIA_B; /* Load HPI address register for DSP B */
	rbuf[1] = 0x007e; /* = 0x007f after increment */
	rbuf[2] = 0x0000;
	wf_sendmem(s, 0, 0, (unsigned char*)&rbuf, 6);

	rbuf[0] = HPID_A; /* Load HPI data register for DSP A */
	rbuf[1] = entry_pt[1] & 0xff;
	rbuf[2] = (entry_pt[1] & 0xff00) >> 8;
	wf_sendmem(s, 0, 0, (unsigned char*)&rbuf, 6);  

	rbuf[0] = HPID_B; /* Load HPI data register for DSP B */
	rbuf[1] = entry_pt[0] & 0xff;;
	rbuf[2] = (entry_pt[0] & 0xff00) >> 8;
	wf_sendmem(s, 0, 0, (unsigned char*)&rbuf, 6);

	/* This also gets sent. TODO: why ? */
	rbuf[0] = HPIA_B; /* Load HPI address register for DSP B */
	rbuf[1] = 0x00ff;
	rbuf[2] = 0x003e;
	wf_sendmem(s, 0, 0, (unsigned char*)&rbuf, 6);  
	rbuf[0] = HPID_B; /* Load HPI data register for DSP B */
	rbuf[1] = 0x00;
	rbuf[2] = 0x00;
	wf_sendmem(s, 0, 0, (unsigned char*)&rbuf, 6);
	rbuf[0] = HPID_B; /* Load HPI data register for DSP B */
	rbuf[1] = 0x00;
	rbuf[2] = 0x00;
	wf_sendmem(s, 0, 0, (unsigned char*)&rbuf, 6);
	rbuf[0] = HPIA_A; /* Load HPI address register for DSP A */
	rbuf[1] = 0x00ff;
	rbuf[2] = 0x001f;
	wf_sendmem(s, 0, 0, (unsigned char*)&rbuf, 6);  
	rbuf[0] = HPIA_B; /* Load HPI address register for DSP B */
	rbuf[1] = 0xff;
	rbuf[2] = 0x1f;
	wf_sendmem(s, 0, 0, (unsigned char*)&rbuf, 6);

	return 0;
}

/*
** TODO: what are these messages for ?
** This isn't well written, but needs to be replaced anyway.
** Incidentally, USBsnoop and Dabble don't seem to agree about
** what gets sent - does the w!nd*ws driver do something odd ?
** In fact, it doesn't seem to matter what data is sent in these
** 64 bytes - hence setting it to zero.
*/
static int wf_req1req2(pwavefinder_t s, int reqnum, int msgnum)
{
	/* unsigned char r0[] = {0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,
			      0x9d,0x3f,0x1b,0xff,0xd4,0xeb,0x7c,0xdd,
			      0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
			      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
			      0x80,0xd4,0x92,0xda,0xe0,0x80,0x92,0xda,
			      0x02,0x80,0x00,0x00,0x00,0x00,0x00,0x00,
			      0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
			      0x00,0x00,0x00,0x00,0x27,0x4a,0x1b,0xff};

	unsigned char r3[] = {0x93,0x02,0x00,0x00,0x32,0x15,0x03,0xc0,
			      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
			      0xb0,0x71,0x02,0xc0,0x00,0x00,0x00,0x00,
			      0x00,0x02,0x00,0x00,0x93,0x02,0x00,0x00,
			      0x87,0x15,0x03,0xc0,0x31,0x0e,0x03,0xc0,
			      0xfc,0x80,0x92,0xda,0x95,0xf1,0x02,0xc0,
			      0x2a,0xba,0x02,0xc0,0xfc,0x80,0x92,0xda,
			      0x05,0x37,0x1b,0xff,0x0d,0x37,0x1b,0xff}; */

        unsigned char r2[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
			      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
			      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
			      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
			      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
			      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
			      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
			      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

	/* unsigned char *p = r0; */
	unsigned char *p = r2;

	switch (msgnum) {
	case 0:
		/*p = r0;*/
		p = r2;
		break;
	case 1:
		/*p = r3;*/
		p = r2;
		break;
	}

	if (reqnum == 1)
		wf_r1_msg(s, p);
	else
		wf_r2_msg(s, p);

	return 0;
}

/*
** Initialize
*/
static int wf_init(pwavefinder_t s)
{
	/* Initialize various SL11R registers - see wfsl11r.h */
	/* Much of this is concerned with the PWM channels which control
	   the coloured LEDs. */
	wf_req1req2(s, 2, 0);
	wf_mem_write(s, PWMCTRLREG, 0);
	wf_mem_write(s, PWMMAXCNT, 0x03ff);

	wf_mem_write(s, PWMCH0STRT, 0);
	wf_mem_write(s, PWMCH0STOP, 0);

	wf_mem_write(s, PWMCH1STRT, 0);
	wf_mem_write(s, PWMCH1STOP, 0);

	wf_mem_write(s, PWMCYCCNT, 0x03ff);

	wf_mem_write(s, PWMCH2STRT, 0);
	wf_mem_write(s, PWMCH2STOP, 0);

	wf_mem_write(s, PWMCH3STRT, 0);
	wf_mem_write(s, PWMCH3STOP, 0);

	wf_mem_write(s, PWMCH0STRT, 0);
	wf_mem_write(s, PWMCH0STOP, 0x02ff);

	wf_mem_write(s, PWMCH1STOP, 0x02ff);

	wf_mem_write(s, PWMCTRLREG, 0x800f);
	wf_mem_write(s, IOCTRLREG1, 0x3de0);
	wf_mem_write(s, UNK0XC120, 0);        /* TODO: work out what's at 0xc120 */
	msleep(100);   // WAS: wf_sleep(100000);
	wf_mem_write(s, UNK0XC120, 0xffff);
	wf_mem_write(s, OUTREG1, 0x3800);     /* TODO: work out what each bit controls */
	wf_mem_write(s, OUTREG0, 0x0000);
	wf_mem_write(s, OUTREG1, 0x3000);
	wf_mem_write(s, OUTREG1, 0x3800);
	wf_boot_dsps(s);
	wf_mem_write(s, OUTREG0, 0x1000);     /* TODO: work out what each bit controls */
        return 0;
}

/*
** Generate the necessary bytes to tune the WaveFinder
**
** There are 2 National Semiconductor PLLs
**     - an LMX1511 used for Band III tuning
**     - an LMX2331A used for the L-Band converter.
** 
** In the other software the LMX1511 is always used to tune, with the L-band converter
** operating at a fixed frequency.
**
** When bRequest is 5, the USB data is:
**
** 1st 4 bytes = data to be shifted into the PLL register
** 5th and 6th byte: word = count of the number of bits to shift
** 7th and 8th byte: word = 1 to address the LMX1511, 0 to address the LMX2331A
** 9th and 10th byte: word = 1 selects L-band, 0 selects Band III (tentative)
** 11th and 12th byte apparently unused (tentative).
**
** So tuning the WaveFinder requires 6 messages, 2 to load the LMX1511 and
** 4 to load the LMX2331A.
**
** Messages to tune the LMX2331A contain constant data (currently).
**
** IF is 38.912MHz
** Reference frequency (f_osc) is 16.384MHz
** L-Band converter frequency is 1251.456MHz
** 
** In the other software:
** LMX1511 uses prescaler: 64/65 P=64, R=1024.
** LMX2331A uses RF prescaler: 128/129 P=128, RF R=256, +ve phase, RF N count: A=98, B=152
**     "     "   IF prescaler: 16/17 P=16, IF R=256, +ve phase, IF N count: A=0, B=40.
**
** Data is loaded into a shift register in each of the PLLs via a 3-line serial microwire
** interface by the WaveFinder firmware.
**
** The LMX1511 uses two words - one has 16 bits and the other 19 bits - the LSB determines
** how the shift register contents are interpreted.
**
** The LMX2331A uses four 22-bit words - the two LSBs determine how the shift register
** contents are interpreted.
**
** (I don't know what the IF output from the LMX2331A is used for but it isn't disabled)
**
** For both PLLs (and for the RF and IF sections of the LMX2331A): f_vco = (P*B+A)*f_osc/R
** For the LMX1511, we need to calculate A and B:
** A = ((f_vco/f_osc)*R) mod P
** B = ((f_vco/f_osc)*R) div P
** where f_vco = frequency we want to tune + IF.
**
** Since the LMX2331A is set to a fixed frequency we can just load it with the appropriate
** constant data or (maybe later) we could allow an alternative tuning strategy.
**
** See the National Semiconductor data sheets for more detail.
*/

/* Maximum Band III frequency (KHz) */
#define MAXFREQIII_KHZ 240000

/* For L Band, this (KHz) is subtracted from the input frequency */
#define LBANDOFFSET_KHZ 1251456

/* The receiver Intermediate Frequency - WaveFinder uses Hitachi HWSD231 SAW filter */
#define IF_KHZ 38912

/* Reference frequency */
#define F_OSC_KHZ 16384

/* LMX1511 R division constant */
#define R_1511 1024

/* LMX1511 Prescaler */
#define P_1511 64

/* LMX2331A IF and RF R Counter */
#define R_2331A 256

/* LMX2331A IF N Counter */
#define NIFA_2331A 0
#define NIFB_2331A 40

/* LMX2331A RF N Counter */
#define NRFA_2331A 98
#define NRFB_2331A 152

/* PLL Selection */
#define LMX1511 1
#define LMX2331A 0

/*
** Reverse the 'len' least sig bits in 'op'
*/ 
static unsigned int reverse_bits(unsigned int op, unsigned int len)
{
	int i;
	unsigned int j = 0;

	for (i=0; i < len; i++) {
		if (op & (1 << (len - i - 1)))
			j |= 1 << i;
	}

	return(j);
}

/*
** Tune the WaveFinder
*/
static void wf_tune(pwavefinder_t s, u32 freq_khz)
{
#define TBUFSIZE 12
	int lband;
	int f_vco;
	int a_1511, b_1511;
	unsigned int rc;
	unsigned int tmp;

	lband = 0;

	if (freq_khz > MAXFREQIII_KHZ) {
		lband = 1;
		freq_khz = freq_khz - LBANDOFFSET_KHZ;
	}

	/* *Don't* change the order in which these messages are sent */

	/* Load the RF R counter of the Band L PLL - constants */
	rc = 0x100000 | reverse_bits(R_2331A, 15) << 5 | 0x10;
	wf_tune_msg(s, rc, 22, LMX2331A, lband);

	/* Load the RF N counter of the Band L PLL - constants */
	rc = 0x300000 | reverse_bits(NRFA_2331A, 7) << 13 | reverse_bits(NRFB_2331A, 11) << 2 | 2;
	wf_tune_msg(s, rc, 22, LMX2331A, lband);

	/* Load the IF R counter of the Band L PLL - constants */ 
	rc = reverse_bits(R_2331A, 15) << 5 | 0x10;
	wf_tune_msg(s, rc, 22, LMX2331A, lband);

	/* Load the N counter of the Band III PLL - this does the tuning */
	tmp = (freq_khz + IF_KHZ);
	tmp *= R_1511;
	f_vco = tmp / F_OSC_KHZ;
	if (tmp % F_OSC_KHZ) f_vco++;

	/* Load the IF N counter of the Band L PLL - constants */
	rc = 0x200000 | reverse_bits(NIFA_2331A, 7) << 13 | reverse_bits(NIFB_2331A, 11) << 2 | 2;
	wf_tune_msg(s, rc, 22, LMX2331A, lband);

	b_1511 = f_vco / P_1511;
	a_1511 = f_vco % P_1511;
  
	/* Load the R counter and S latch of the Band III PLL - constants */
	rc = 0x8000 | (reverse_bits((int)R_1511, 14)) << 1 | 1;
	wf_tune_msg(s, rc, 16, LMX1511, lband);

	/* Load the N counter (as A and B counters) of the Band III PLL */
	rc = reverse_bits(a_1511,7) << 11 | reverse_bits(b_1511,11);
	wf_tune_msg(s, rc, 19, LMX1511, lband);
}

static int wf_timing(pwavefinder_t s, int msgnum)
{
	unsigned char m0[] = {0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff,
			      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00};

	unsigned char m1[] = {0x7f, 0x00, 0x00, 0xfe, 0x80, 0x07, 0xe0, 0x01,
			      0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff,
			      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00};

	unsigned char m2[] = {0x7f, 0x00, 0x00, 0xfe, 0x80, 0x07, 0xe0, 0x01,
			      0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00};

	unsigned char m3[] = {0x7f, 0x00, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xff,
			      0xf8, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff,
			      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00};

	unsigned char m4[] = {0x7f, 0x00, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xff,
			      0xf8, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00};

	unsigned char *p;

	switch (msgnum) {
	case 0:
		p = m0;
		break;
	case 1:
		p = m1;
		break;
	case 2:
		p = m2;
		break;
	case 3:
		p = m3;
		break;
	case 4:
		p = m4;
		break;
	default:
		p = m0;
		break;
	}

	wf_timing_msg(s, p);

	return 0;
}

static int wavefinder_leds(pwavefinder_t s, u32 arg)
{
	int red, green, blue;

	red = (arg >> 20) & 0x3ff;
	green = (arg >> 10) & 0x3ff;
	blue = arg & 0x3ff;

	dbg("wavefinder: Setting LEDS: red=0x%03x, green=0x%03x, blue=0x%03x\n",red,green,blue);

	wf_mem_write(s, PWMCH2STOP, red);
	wf_mem_write(s, PWMCH3STOP, green);
	wf_mem_write(s, PWMCH1STOP, blue);

	return 0;
}

static int wavefinder_tune(pwavefinder_t s, u32 freq_khz)
{
	dbg("wavefinder: Request to tune to %u HZ\n",freq_khz);

  	wavefinder_leds(s,(0x3ff << 20) | (0x180 << 10) | 0x3ff); /* Green LED on as simple indicator */
	wf_tune(s, freq_khz);
	msleep(400); // WAS: wf_sleep(400000);
	wf_timing(s, 0);
	usleep_range(4000,4000); // WAS: wf_sleep(4000);
	wf_timing(s, 1);
	usleep_range(4000,4000); // WAS: wf_sleep(4000);
	wf_timing(s, 1);
	usleep_range(4000,4000); // WAS: wf_sleep(4000);
	wf_timing(s, 2);
	msleep(50); // WAS: wf_sleep(50000);
	wf_mem_write(s, DACVALUE, 0x5330);
	wf_mem_write(s, DACVALUE, 0x5330);
	msleep(77); // WAS: wf_sleep(77000);
	/* The next control message causes the WaveFinder to start sending
	   isochronous data */
	wf_req1req2(s, 1, 1);
	/* wf_read(s, of, &pkts); */
	wf_mem_write(s, PWMCTRLREG, 0x800f);
	wf_timing(s, 1);
	wf_timing(s, 2);
	wf_timing(s, 1);
	wf_timing(s, 3);
	wf_tune(s, freq_khz);
	msleep(200); // WAS: wf_sleep(200000);
	wf_timing(s, 4);
	wf_tune(s, freq_khz);
	msleep(200); // WAS: wf_sleep(200000);
	wf_tune(s, freq_khz);
	msleep(200); // WAS: wf_sleep(200000);
	wf_mem_write(s, DACVALUE, 0x5330);
        return 0;
}

static int wavefinder_release(struct inode *inode, struct file *file)
{
	pwavefinder_t s = (pwavefinder_t) file->private_data;

	dbg("wavefinder_release");

	down(&s->mutex);
        /* The following two lines are from wf_close() in the opendab application code */
	wavefinder_leds(s, (0x3ff < 20) | (0x3ff < 10) | 0x3ff );  // Turn LEDs off
	wf_mem_write(s, OUTREG1, 0x2000);

	wavefinder_stop(s);
	wavefinder_free_buffers(s);
	up(&s->mutex);

	wake_up(&s->remove_ok);

	s->opened = 0;
	return 0;
}

static long wavefinder_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	pwavefinder_t s = (pwavefinder_t) file->private_data;
	ctrl_transfer_t *ctrl;
	int ret = 0;

	/* dbg("wavefinder_ioctl"); */

	if (s->remove_pending)
		return -EIO;

	down(&s->mutex);

	if (!s->usbdev) {
		up(&s->mutex);
		return -EIO;
	}

	switch (cmd) {

	case IOCTL_WAVEFINDER_CMSGW:
		ctrl = (ctrl_transfer_t*) kmalloc(sizeof(ctrl_transfer_t), GFP_KERNEL);

		if (!ctrl) {
			ret = -ENOMEM;
			break;
		}

		if (copy_from_user(ctrl, (void *) arg, sizeof(ctrl_transfer_t))) {
			ret = -EFAULT;
			kfree(ctrl);
			break;
		}

		ret=usb_control_msg(s->usbdev, usb_sndctrlpipe( s->usbdev, 0 ), 
				    ctrl->setup.bRequest, ctrl->setup.bRequestType,
				    ctrl->setup.wValue,ctrl->setup.wIndex,
				    ctrl->data, ctrl->size,HZ);
		kfree(ctrl);
		break;


	case IOCTL_WAVEFINDER_TUNE_KHZ:
		wavefinder_tune(s, arg);
		break;

	case IOCTL_WAVEFINDER_LEDS:
		wavefinder_leds(s, arg);
		break;


	default:
		ret = -ENOIOCTLCMD;
		break;
	}
	up(&s->mutex);
	return ret;
}

static int wavefinder_open(struct inode *inode, struct file *file)
{
	int i,  devnum;
	pwavefinder_t s;
	struct usb_interface *interface;

	dbg("wavefinder_open");

	devnum = iminor(inode);

	interface = usb_find_interface (&wavefinder_driver, devnum);
	dbg("wavefinder_open: devnum = %d", devnum);
	s = usb_get_intfdata(interface);

	down(&s->mutex);
	if (!s->usbdev)	{
		up(&s->mutex);
		return -ENODEV;		
	}
		
	if (s->opened) {
		up(&s->mutex);
		return -EBUSY;		
	}
	
	s->opened = 1;
	up(&s->mutex);

	s->count = 0;
	s->readptr = 0;

	for (i = 0; i < NURBS; i++)
		s->allocated[i] = 0;
	s->recbuf = 0;


	file->f_pos = 0;
	file->private_data = s;

	wf_init(s);
	return 0;
}


static struct file_operations wavefinder_fops =
{
	.owner =	  THIS_MODULE,
	.read =		  wavefinder_read,
	.unlocked_ioctl = wavefinder_ioctl,
	.open =	          wavefinder_open,
	.release =	  wavefinder_release,
};

static struct usb_class_driver wavefinder_class = {
	.name =		"usb/wavefinder%d",
	.fops =		&wavefinder_fops,
	.minor_base =	WAVEFINDER_MINOR,
};


static int wavefinder_probe(struct usb_interface *intf,
			    const struct usb_device_id *id)
{
        struct usb_device *usbdev = interface_to_usbdev(intf);
	int retval;
	pwavefinder_t s;

	/* intf->minor only becomes valid after usb_register_dev() */
	retval = usb_register_dev(intf, &wavefinder_class);

	s = (wavefinder_t*)kmalloc(sizeof(wavefinder_t), GFP_KERNEL);
	if (!s)
		return -ENOMEM;
	memset(s,0,sizeof(wavefinder_t));
	sema_init (&s->mutex,1);
	init_waitqueue_head (&s->wait);
	init_waitqueue_head (&s->remove_ok);
	spin_lock_init (&s->lock);
		
	s->remove_pending = 0;
	s->usbdev = usbdev;
	s->devnum = intf->minor;
	s->state = _stopped;

	dbg("wavefinder: probe: vendor id 0x%x, device id 0x%x ifnum:%d intf->minor:%d s=%ud wavefinder_t=%d",
	    usbdev->descriptor.idVendor, usbdev->descriptor.idProduct, intf->altsetting->desc.bInterfaceNumber,intf->minor,(unsigned int)s,(int)(sizeof(wavefinder_t)));

	if (intf->altsetting->desc.bInterfaceNumber != _WAVEFINDER_IF)
	        goto reject;
	
	down(&s->mutex);

	if (usbdev == (struct usb_device*)NULL)
		goto reject;

	if (usb_reset_configuration(usbdev) < 0) {
		err("reset_configuration failed");
		goto reject;
	} 
	dbg("bound to interface: %d", intf->altsetting->desc.bInterfaceNumber);
	usb_set_intfdata(intf, s);
	up(&s->mutex);

	dbg("wavefinder: probe: intf->minor:%d",intf->minor);

	if (retval) {
		usb_set_intfdata(intf, NULL);
		return -ENOMEM;
	}

	return 0;

reject:
	up(&s->mutex);
	s->usbdev = NULL;
	return -ENODEV;
}

static void wavefinder_disconnect(struct usb_interface *intf)
{
        wait_queue_t __wait;
	pwavefinder_t s = usb_get_intfdata(intf);

	dbg("wavefinder_disconnect");

	init_waitqueue_entry(&__wait, current);

	usb_set_intfdata(intf, NULL);
	if (s) {
		usb_deregister_dev(intf, &wavefinder_class);
		s->remove_pending = 1;
		wake_up(&s->wait);
		add_wait_queue(&s->remove_ok, &__wait);
		set_current_state(TASK_UNINTERRUPTIBLE);
		if (s->state == _started)
			schedule();
		set_current_state(TASK_RUNNING);
		remove_wait_queue(&s->remove_ok, &__wait);
		s->usbdev = NULL;
		s->overruns = 0;
	}
}


static struct usb_driver wavefinder_driver =
{
        .name =         "wavefinder",
	.probe =	wavefinder_probe,
	.disconnect =	wavefinder_disconnect,
	.id_table =	wavefinder_ids,
};


static int __init wavefinder_init(void)
{
        int retval;

	dbg("wavefinder_init");

	/* register misc device */
	retval = usb_register(&wavefinder_driver);
	if (retval)
		goto out;

	dbg("wavefinder_init: driver registered");
	dbg(DRIVER_VERSION ": " DRIVER_DESC);

out:
	return retval;
}

static void __exit wavefinder_cleanup(void)
{
	dbg("wavefinder_cleanup");

	usb_deregister(&wavefinder_driver);

	dbg("wavefinder_cleanup finished");
}


module_init(wavefinder_init);
module_exit(wavefinder_cleanup);

MODULE_AUTHOR( DRIVER_AUTHOR );
MODULE_DESCRIPTION( DRIVER_DESC );
MODULE_LICENSE("GPL");

