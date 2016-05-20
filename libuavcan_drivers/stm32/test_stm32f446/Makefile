#
# Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
#
#   Adjusted for STM32446 and Chopstx thread library by Kaz Kojima
#

PROJECT = uavcan_test_stm32f446

#
# Test application based on uavcan_test_stm32f107
#

MAIN ?= main.cpp

CPPSRC = src/$(MAIN)              \
         src/dummy.cpp            \
         src/board/board.cpp

#
# UAVCAN library
#

export LIBUAVCAN_REPO_ROOT := $(abspath ../../..)

UDEFS = -DUAVCAN_STM32_CHOPSTX=1         \
        -DUAVCAN_STM32_TIMER_NUMBER=7    \
        -DUAVCAN_TINY=1                  \
        -DUAVCAN_STM32_NUM_IFACES=2      \
        -DUAVCAN_MEM_POOL_BLOCK_SIZE=48

include $(LIBUAVCAN_REPO_ROOT)/libuavcan/include.mk
CPPSRC += $(LIBUAVCAN_SRC)
UINCDIR += $(LIBUAVCAN_INC)

include $(LIBUAVCAN_REPO_ROOT)/libuavcan_drivers/stm32/driver/include.mk
CPPSRC += $(LIBUAVCAN_STM32_SRC)
UINCDIR += $(LIBUAVCAN_STM32_INC)

$(info $(shell $(LIBUAVCAN_DSDLC) $(UAVCAN_DSDL_DIR)))
UINCDIR += dsdlc_generated

#
# Git commit hash
#

GIT_HASH := $(shell git rev-parse --short HEAD)
UDEFS += -DGIT_HASH=0x$(GIT_HASH)

#
# Platform
#

UINCDIR += src/sys

SERIAL_CLI_PORT_NUMBER = 2

CPPWARN := -Wundef -Wno-error=undef

RELEASE_OPT = -Os -fomit-frame-pointer
DEBUG_OPT = -Os -g3

#
# Chopstx
#

CHOPSTX = ./chopstx
LDSCRIPT= test_stm32f446.ld

CROSS = arm-none-eabi-
CC   = $(CROSS)gcc
CXX  = $(CROSS)g++
LD   = $(CROSS)g++
OBJCOPY   = $(CROSS)objcopy

MCU   = cortex-m4
CWARN = -Wall -Wextra -Wstrict-prototypes
DEFS  = -DHAVE_SYS_H -DFREE_STANDING -DMHZ=180 -D__ARM_ARCH_7M__
OPT   = -O3 -Os -g -mfloat-abi=hard -mfpu=fpv4-sp-d16 -ffast-math
USE_EVENTFLAG = yes

INCDIR += $(UINCDIR)
DEFS += $(UDEFS)
OBJS_ADD = $(addprefix build/, $(notdir $(CPPSRC:.cpp=.o)))

include $(CHOPSTX)/rules.mk

vpath %.cpp $(dir $(CPPSRC))

CXXWARN = -Wall -Wextra
CXXFLAGS = $(MCFLAGS) $(OPT) -std=c++11 $(CXXWARN) -Wa,-alms=$(BUILDDIR)/$(notdir $(<:.cpp=.lst)) $(DEFS)
CXXFLAGS += -mthumb -mno-thumb-interwork -DTHUMB
CXXFLAGS += -MD -MP -MF .dep/$(@F).d

$(OBJS_ADD) : $(BUILDDIR)/%.o : %.cpp Makefile
	@echo
	$(CXX) -c $(CXXFLAGS) -I. $(IINCDIR) $< -o $@
#
