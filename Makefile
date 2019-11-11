ifneq ($(KERNELRELEASE),)
# kbuild part of makefile
obj-m  := pps-gmtimer.o

else
# normal makefile
KDIR = /lib/modules/$(shell uname -r)/build

MY_CFLAGS += -g -DDEBUG
ccflags-y += ${MY_CFLAGS}
CC += ${MY_CFLAGS}
KVERSION := $(shell uname -r)

.PHONY: default clean

default:
	$(MAKE) -C $(KDIR) M=$$PWD ARCH=arm

debug:
	$(MAKE) -C $(KDIR) M=$$PWD ARCH=arm EXTRA_CFLAGS="$(MY_CFLAGS)"

clean:
	$(MAKE) -C $(KDIR) M=$$PWD ARCH=arm clean


DD-GPS-00A0.dtbo: DD-GPS-00A0.dts
	dtc -@ -I dts -O dtb -o $@ $<

DD-GPS-TCLKIN-00A0.dtbo: DD-GPS-TCLKIN-00A0.dts
	dtc -@ -I dts -O dtb -o $@ $<

.PHONY: install
install: DD-GPS-00A0.dtbo default
	install -D DD-GPS-00A0.dtbo $(DESTDIR)/lib/firmware/DD-GPS-00A0.dtbo
	install -D pps-gmtimer.ko /lib/modules/$(KVERSION)/extra/

endif
