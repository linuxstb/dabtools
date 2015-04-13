# dabtools

## Introduction

dabtools is work-in-progress set of tools for reception, recording and
playback of DAB and DAB+ digital radio broadcasts. It currently
supports the Psion Wavefinder USB DAB tuner and any SDR tuner
supported by the RTL-SDR project.

It is heavily based on David Crawley's "OpenDAB" software for the
Psion Wavefinder and David May's "rtl-dab" SDR DAB demodulator, and
wouldn't have been possible without their work (and other contributors
to those projects).

The included Psion Wavefinder kernel driver is an extended version of
David Crawley's original driver, with some functionality David
implemented in his userland application moved inside the kernel driver
in order to provide a higher-level API and simplify the application
code.


dabtools currently consists of the following tools:

sdr2eti - receiver for RTL-SDR dongles

dab2eti - receiver for the Psion Wavefinder

(these two tools will be unified in the near future)

eti2mpa - extract an MPEG audio stream from an ETI stream.

ETI is the standard file format for the storage and transport of a DAB
ensemble.  It is defined in ETSI 300 799.

It consists of a set of fixed-size (6144 byte) frames, each containing
24ms of audio and other data.

## Hardware support

### Psion Wavefinder

The Psion Wavefinder is a USB DAB receiver sold between 2000 and 2002.
The COFDM demodulation is performed on two DSPs and the samples are
then transferred over USB for further software processing on the host
computer.

To use dabtools with the Wavefinder, you need to build and install the
driver in the wavefinder-driver directory.  The DSP firmware is
included in the wavefinder.fw file in that directory and must be
copied to /lib/firmware/wavefinder.fw

Note that this firmware file is simply the concatenation of the
rsDSPa.bin and rsDSPb.bin files included in the original Windows
driver.

This driver is an extended version of David Crawley's driver included
with OpenDAB, but with the low-level functionality from the OpenDAB
application moved into the driver in order to provide a higher-level
API.

dab2eti is used to receive an ETI stream, and the frequency is
specified in KHz.  e.g.

./dab2eti 218640 > dump.eti

to record a stream or

./dab2eti 218640 | eti2mpa 2 | madplay -v -

to play sub-channel 2 from the ensemble.


### RTL-SDR devices

I have tested sdr2eti with RTL-SDR dongles with both FC00013 and R820T
tuners with similar success.  Achieving a lock on a signal requires
manually setting the gain value, which is passed in 10ths of a dB.

e.g. to record an ensemble broadcasting at 218.640MHz with 9dB gain:

./sdr2eti 218640000 90 > dump.eti


## Building

dabtools requires librtlsdr and libfftw3.  The former can be found at
http://sdr.osmocom.org/trac/wiki/rtl-sdr and the latter should be
available via your distribution's package manage (e.g. libfftw3-dev).



## Copyright

dabtools is written by Dave Chapman <dave@dchapman.com> 

Large parts of the code are copied verbatim (or with trivial
modifications) from David Crawley's OpenDAB and hence retain his
copyright.

The RTL-SDR code is (C) David May.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at
your option) any later version.

This program is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
