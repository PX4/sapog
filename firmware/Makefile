#
#   Copyright (C) 2013-2016 PX4 Development Team. All rights reserved.
#   Author: Pavel Kirienko <pavel.kirienko@gmail.com>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

PROJECT = io.px4.sapog

HW_VERSION = 1
FW_VERSION_MAJOR = 1
FW_VERSION_MINOR = 0

export SAPOG_BASE := $(realpath $(dir $(lastword $(MAKEFILE_LIST))))
export SAPOG_SRC_DIR = $(abspath $(SAPOG_BASE)/src)

PYTHON := python

#
# Sources
#

CSRC = $(wildcard $(SAPOG_SRC_DIR)/*.c)     \
       $(wildcard $(SAPOG_SRC_DIR)/*/*.c)   \
       $(wildcard $(SAPOG_SRC_DIR)/*/*/*.c)

CPPSRC = $(wildcard $(SAPOG_SRC_DIR)/*.cpp)   \
         $(wildcard $(SAPOG_SRC_DIR)/*/*.cpp)

UINCDIR = src           \
          src/sys

UDEFS = -DFW_VERSION_MAJOR=$(FW_VERSION_MAJOR)           \
        -DFW_VERSION_MINOR=$(FW_VERSION_MINOR)           \
        -DHW_VERSION=$(HW_VERSION)

#
# UAVCAN library
#

UDEFS += -DUAVCAN_STM32_TIMER_NUMBER=7                   \
         -DUAVCAN_STM32_NUM_IFACES=2                     \
         -DUAVCAN_CPP_VERSION=UAVCAN_CPP11               \
         -DUAVCAN_STM32_CHIBIOS=1

include libuavcan/libuavcan/include.mk
CPPSRC += $(LIBUAVCAN_SRC)
UINCDIR += $(LIBUAVCAN_INC)

include libuavcan/libuavcan_drivers/stm32/driver/include.mk
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
# OS
#

SERIAL_CLI_PORT_NUMBER = 1

BOOTLOADER_SIZE = 8192

DDEFS += -DCORTEX_VTOR_INIT=$(BOOTLOADER_SIZE)

LDSCRIPT= linker.ld

RELEASE_OPT = -O2 -fomit-frame-pointer
DEBUG_OPT = -O2 -g3

# More warnings are enabled in the included makefile
CWARN = -Wshadow -Wpointer-arith -Wno-packed -Wno-attributes -Wno-error=undef -Wno-error=shadow
CPPWARN = $(CWARN)

include zubax_chibios/rules_stm32f105_107.mk

%.image: %.bin
	$(PYTHON) ../tools/make_can_boot_descriptor.py --verbose -g $< build/io.px4.sapog-1.0.00000000.uavcan.bin

