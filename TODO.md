Things to do (in no particular order):

* Handle multiplex reconfigurations

* Allow the user to specify the device to use.

* Add the ability to record (and playback) raw data for
  testing/debugging usage.

* Check the details of the ETI stream (especially the sync/count/phase
  fields to ensure they are correct).

* Identify and deal with errors in the MSC symbols, better deal with
  errors in the FIC.

* Write an "eti2aac" filter to extract a DAB+ audio stream (probably
  merged with eti2mpa)

* Clarify the (C) of each file, including making sure each file has a
  (C)/licence header.

* Investigate alternative viterbi decoders - profiling shows that
  about 97% of dab2eti's time (when using a wavefinder) is spent in
  the viterbi decoder, so this is the obvious place for optimisation.
  One option is http://www.spiral.net/software/viterbi.html

* Implement sub-channel filters in dab2eti, so that dab2eti can save
  CPU time by not decoding data which will later be discarded.  This
  will greatly reduce CPU usage when the user is just playing a single
  service.

* Investigate the possibility of using multiple decoding threads to
  enable dab2eti to run on relatively low-speed but multi-core
  hardware such as the Raspberry Pi 2.

* General code cleaning and re-organisation.  

* Turn the dab2eti functionality into a library to allow full-featured
  DAB applications to be built on top of it.

