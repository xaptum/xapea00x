#
# Makefile for XAP-EA-00x
#

ifneq ($(KERNELRELEASE),)

include Kbuild

else

KDIR ?= /lib/modules/`uname -r`/build

default:
	$(MAKE) -C $(KDIR) M=$$PWD

clean:
	rm -rf *.o .depend .*.cmd *.ko *.mod.c \
	       modules.order  Module.symvers

.PHONY: default clean

endif
