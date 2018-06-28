# Makefile - driver build system
#
# Copyright (C) 2005 Lumenera
#
# Requires GNU Make

LUCAM_PREFIX := lt
LUCAM_PID := usb

ifeq ($(src),)
src := $(CURDIR)
endif 

include $(src)/../lucam.mk

EXTRA_CFLAGS += -DUSB3_ENDPOINT_CONFIGURATION=1 -DENHANCED_BUFFERING=1
