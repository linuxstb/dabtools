/* -*- linux-c -*- ***********************************************************/
/*
 *      wavefinder.h  --  part of the Psion WaveFinder driver for Linux 2.6 kernel
 *
 *      Copyright (C) 2005 David Crawley <degressive@yahoo.co.uk>
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
 *      along with this program. If not, see <http://www.gnu.org/licenses/>.
 */
/*****************************************************************************/

#ifndef _WAVEFINDER_H
#define _WAVEFINDER_H

#include <linux/usb/ch9.h>

/* The following shouldn't be exposed - need to create some more higher-level
   API functions instead */
#include "wfsl11r.h"

/* WaveFinder bmRequest values */
#define bmRequest1 1
#define bmRequest2 2
#define SLMEM 3
#define WFTUNE 4
#define WFTIMING 5

typedef struct {
	struct usb_ctrlrequest setup;
	unsigned int size;
	unsigned char data[64];
} ctrl_transfer_t;

#define IOCTL_WAVEFINDER_CMSGW           _IOWR('d', 0x30, ctrl_transfer_t*)
#define IOCTL_WAVEFINDER_TUNE_KHZ        _IOW('d', 0x31, int)
#define IOCTL_WAVEFINDER_LEDS            _IOW('d', 0x32, int)

#endif
