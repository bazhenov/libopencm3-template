PROJECT = firmware
BUILD_DIR = bin

OPENCM3_DIR=../libopencm3
SHARED_DIR = ../shared
FREERTOS_DIR = ../freertos

OBJS += $(FREERTOS_DIR)/heap_4.o \
	$(FREERTOS_DIR)/tasks.o \
	$(FREERTOS_DIR)/opencm3.o \
	$(FREERTOS_DIR)/list.o \
	$(FREERTOS_DIR)/port.o

CFILES += StrPrintf.c
CXXFILES += usbcdc.cxx main.cxx lis3dsh.cxx hd44780.cxx

# TODO - you will need to edit these two lines!
DEVICE=stm32f407vgt6
OOCD_FILE = board/stm32f4discovery.cfg

# You shouldn't have to edit anything below here.
include ../rules.mk