# Makefile - Lumenera camera firmware loading driver build system
#
# Copyright (C) 2005-2009 Lumenera
#
# Requires GNU Make


#ifeq ($(KERNELRELEASE),)	# Test whether we were called by 2.6.x build system
# Called by user

# Get current directory
PWD := $(shell pwd)

# Get MODULE_DIR (where to install kernel module)
#ifndef MODULE_DIR
#ifndef LINUX_DIR
#MODULE_DIR := /lib/modules/$(shell uname -r)
#else
## Note: the brackets on line below contain a space and a _tab_!
#RELEASE := $(shell awk -F \" '/[ 	]*\#[ 	]*define[ 	]*UTS_RELEASE[ 	]*/ { print $$2 }' $(LINUX_DIR)/include/linux/version.h|tail -n 1)
#MODULE_DIR := /lib/modules/$(RELEASE)
#endif
#endif
#
# Get LINUX_DIR (where the kernel source is or at least headers)
#ifndef LINUX_DIR
#LINUX_DIR := $(MODULE_DIR)/build
#endif
#
ifndef USER_OPT
USER_OPT :=
endif

# Get VERSION_CODE (from version.h in kernel source directory)
#VERSION_CODE := $(shell awk '/[ 	]*\#[ 	]*define[ 	]*LINUX_VERSION_CODE[ 	]*/ { print $$3 }' $(LINUX_DIR)/include/linux/version.h|tail -n 1)

MODULE_NAME := $(LULDR_PREFIX)ldr$(LULDR_PID)

.PHONY: help
help:
	@echo '-=- Lumenera USB loader driver -=-'
	@echo
	@echo "Makefile target examples:"
	@echo "make all - Compile driver and utilities against current running kernel"
	@echo "make all USER_OPT=-DDEBUG - Compile with debugging code and messages"
	@echo "make all LINUX_DIR=/usr/src/linux - Compile against specified kernel source"
	@echo "make clean - Remove object files from the source directory"
	@echo
	@echo "Current configuration:"
	@echo "Driver source directory (PWD):         $(PWD)"
	@echo "Kernel source directory (LINUX_DIR):   $(LINUX_DIR)"
	@echo "Module install directory (MODULE_DIR): $(MODULE_DIR)"
	@echo "User options (USER_OPT):               $(USER_OPT)"
	@echo "Driver file name (use with insmod):    $(MODULE_NAME)"
	@echo "Kernel version code:                   $(VERSION_CODE)"
	@echo "This module        :                   $(THIS_MODULE)"
	@echo "Current directory  :                   $(CURDIR)"

all: modules

clean:
	rm -f *.o *~ .\#* .*.cmd *.mod.c *.ko
	rm -rf .tmp_versions

# Is it 2.6.0 or newer?
#ifeq ($(shell if [ $(VERSION_CODE) -ge 132608 ]; then echo y; fi),y)
# Yes, 2.6.0 or newer

ifeq ($(obj),)
obj := $(CURDIR)
endif

modules: $(MODULE_NAME).ko

$(MODULE_NAME).ko: ../luldr.c $(obj)/firms.c usb_ids.c ../luldr.h
	+make -C "$(LINUX_DIR)" SUBDIRS="$(PWD)" modules $(KERNEL_MAKE_ARGS) USER_OPT="$(USER_OPT)"

#else
## No, 2.4.x or older
#
#include $(LINUX_DIR)/Rules.make
#include $(LINUX_DIR)/.config
#
#MODULE_INC    := -I$(LINUX_DIR)/include -I..
#MODULE_DEF    := -DMODULE -D__KERNEL__ -DNOKERNEL
#MODULE_WARN   := -Wall -Wstrict-prototypes -Wno-trigraphs
#CFLAGS  += $(MODULE_INC) $(MODULE_DEF) $(MODULE_WARN) $(USER_OPT)
#
#obj-m := $(MODULE_NAME).o
#
#modules: $(obj-m)
#
#$(MODULE_NAME).o: luldr.o firms.o usb_ids.o
	#$(LD) $(LD_RFLAG) -r -o $@ $(filter %.o,$^)
#
#%.c: ../%.c
	#ln -sfn $< $@
#
#luldr.o: luldr.c ../luldr.h
#firms.o: firms.c ../luldr.h
#usb_ids.o: usb_ids.c ../luldr.h
#
#endif # for kernel version 2.6.x test


#else
# Called recursively by 2.6.x kernel build process

EXTRA_CFLAGS += -DNOKERNEL $(USER_OPT) -I$(src)/..

obj-m := $(LULDR_PREFIX)ldr$(LULDR_PID).o
$(LULDR_PREFIX)ldr$(LULDR_PID)-objs := luldr.o firms.o usb_ids.o

$(obj)/%.c: $(src)/../%.c
	ln -sfn $(abspath $<) $@

$(obj)/luldr.o: $(obj)/luldr.c $(src)/../luldr.h
$(obj)/firms.o: $(obj)/firms.c $(src)/../luldr.h
$(obj)/usb_ids.o: $(src)/usb_ids.c $(src)/../luldr.h

#endif

