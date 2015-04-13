ifneq ($(KERNELRELEASE),)
	obj-m := wavefinder.o
else
KDIR        := /lib/modules/$(shell uname -r)/build
PWD         := $(shell pwd)
DEBUG	    := 1

default:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules
endif
