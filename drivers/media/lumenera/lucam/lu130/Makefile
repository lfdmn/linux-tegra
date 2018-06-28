# Makefile - driver build system
#
# Copyright (C) Lumenera
#
# Requires GNU Make

LUCAM_PREFIX := lu
LUCAM_PID := 09A

ifeq ($(src),)
src := $(CURDIR)
endif 

include $(src)/../lucam.mk

$(LUCAM_PREFIX)cam$(LUCAM_PID)-objs += custom.o

$(obj)/custom.o: $(obj)/custom.c $(src)/../lucam.h

EXTRA_CFLAGS += -DDRIVER_INITRESET -DDRIVER_SOFT_FORMAT_VALIDATION

