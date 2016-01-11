#
#   Copyright (C) 2013 PX4 Development Team. All rights reserved.
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

PROJECT = px4esc

UDEFS = -DFW_VERSION_MAJOR=0 -DFW_VERSION_MINOR=1

export PX4ESC_BASE := $(realpath $(dir $(lastword $(MAKEFILE_LIST))))
export PX4ESC_SRC_DIR = $(abspath $(PX4ESC_BASE)/src)

#
# Sources
#

CSRC = $(wildcard $(PX4ESC_SRC_DIR)/*.c)     \
       $(wildcard $(PX4ESC_SRC_DIR)/*/*.c)   \
       $(wildcard $(PX4ESC_SRC_DIR)/*/*/*.c)

CPPSRC = $(wildcard $(PX4ESC_SRC_DIR)/*.cpp)   \
         $(wildcard $(PX4ESC_SRC_DIR)/*/*.cpp)

UINCDIR = src           \
          src/sys       \
          spl/inc

#
# UAVCAN library
#

UDEFS += -DUAVCAN_STM32_TIMER_NUMBER=7    \
         -DUAVCAN_STM32_NUM_IFACES=2      \
         -DUAVCAN_MEM_POOL_BLOCK_SIZE=56  \
         -DUAVCAN_STM32_CHIBIOS=1

include uavcan/libuavcan/include.mk
CPPSRC += $(LIBUAVCAN_SRC)
UINCDIR += $(LIBUAVCAN_INC)

include uavcan/libuavcan_drivers/stm32/driver/include.mk
CPPSRC += $(LIBUAVCAN_STM32_SRC)
UINCDIR += $(LIBUAVCAN_STM32_INC)

$(info $(shell $(LIBUAVCAN_DSDLC) $(UAVCAN_DSDL_DIR)))
UINCDIR += dsdlc_generated

#
# OS configuration
#

UDEFS += -DCORTEX_ENABLE_WFI_IDLE=1 -DSTDOUT_SD=SD1 -DSTDIN_SD=STDOUT_SD -DCHPRINTF_USE_FLOAT=1 -DSHELL_MAX_ARGUMENTS=5

USE_LINK_GC = yes
USE_THUMB = yes
USE_VERBOSE_COMPILE ?= no
USE_FWLIB = no

CHIBIOS = chibios
include $(CHIBIOS)/os/hal/platforms/STM32F1xx/platform_f105_f107.mk
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/ports/GCC/ARMCMx/STM32F1xx/port.mk
include $(CHIBIOS)/os/kernel/kernel.mk
include $(CHIBIOS)/os/various/cpp_wrappers/kernel.mk

LDSCRIPT= $(PORTLD)/STM32F107xC.ld

VARIOUSSRC = $(CHIBIOS)/os/various/syscalls.c    \
             $(CHIBIOS)/os/various/chprintf.c    \
             $(CHIBIOS)/os/various/shell.c       \
             $(CHIBIOS)/os/various/memstreams.c

CSRC += $(PORTSRC) $(KERNSRC) $(HALSRC) $(PLATFORMSRC) $(VARIOUSSRC)

CPPSRC += $(CHCPPSRC)

ASMSRC = $(PORTASM)

INCDIR = $(PORTINC) $(KERNINC) $(HALINC) $(PLATFORMINC) $(CHCPPINC) $(CHIBIOS)/os/various

#
# Git commit hash
#

GIT_HASH := $(shell git rev-parse --short HEAD)
ifneq ($(words $(GIT_HASH)),1)
    GIT_HASH := "?"
endif
UDEFS += -DGIT_HASH=0x$(GIT_HASH)

#
# Build configuration
#

USE_OPT = -falign-functions=16 -U__STRICT_ANSI__ -fno-exceptions -fno-unwind-tables -fno-stack-protector
USE_OPT += -nodefaultlibs -lc -lgcc -lm
USE_OPT += -include src/global_header.h

USE_COPT = -std=c99
USE_CPPOPT = -std=c++11 -fno-rtti -fno-threadsafe-statics

RELEASE ?= 0
ifneq ($(RELEASE),0)
    DDEFS += -DRELEASE_BUILD -DNDEBUG
    USE_OPT += -O2 -fomit-frame-pointer
else
    DDEFS += -DDEBUG_BUILD
    USE_OPT += -O1 -g3
endif

#
# Compiler options
#

MCU  = cortex-m3

TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CPPC = $(TRGT)g++
LD   = $(TRGT)g++
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
OD   = $(TRGT)objdump
SZ   = $(TRGT)size
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary

# ARM-specific options here
AOPT =

# THUMB-specific options here
TOPT = -mthumb -DTHUMB

WARN = -Wall -Wextra -Werror -Wshadow -Wpointer-arith -Wpacked
CWARN = $(WARN) -Wstrict-prototypes
CPPWARN = $(WARN)

# asm statement fix
DDEFS += -Dasm=__asm

include $(CHIBIOS)/os/ports/GCC/ARMCMx/rules.mk
