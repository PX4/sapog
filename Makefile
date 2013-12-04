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

#
# Sources
#

MOTOR_CSRC = src/motor_lowlevel/motor_pwm.c    \
             src/motor_lowlevel/motor_timer.c  \
             src/motor_lowlevel/motor_adc.c    \
             src/motor_lowlevel/motor_test.c   \
             src/motor_lowlevel/motor.c

MOTORMGR_CSRC = src/motor_manager/motormgr.c   \
                src/motor_manager/rpmctl.c

CSRC = src/main.c                       \
       src/console.c                    \
       src/sys/board.c                  \
       src/sys/sys.c                    \
       src/config/config.c              \
       src/config/flash_storage.c       \
       $(MOTOR_CSRC) $(MOTORMGR_CSRC)

UINCDIR = src/sys src/config

UDEFS = -DHRT_TIMER_NUMBER=1

#
# OS configuration
#

UDEFS += -DCORTEX_ENABLE_WFI_IDLE=1 -DSTDOUT_SD=SD1 -DSTDIN_SD=STDOUT_SD -DCHPRINTF_USE_FLOAT=1

USE_LINK_GC = yes
USE_THUMB = yes
USE_VERBOSE_COMPILE = no
USE_FWLIB = no

CHIBIOS = chibios
include $(CHIBIOS)/os/hal/platforms/STM32F1xx/platform.mk
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/ports/GCC/ARMCMx/STM32F1xx/port.mk
include $(CHIBIOS)/os/kernel/kernel.mk

LDSCRIPT= $(PORTLD)/STM32F103xB.ld

VARIOUSSRC = $(CHIBIOS)/os/various/syscalls.c $(CHIBIOS)/os/various/chprintf.c $(CHIBIOS)/os/various/shell.c

CSRC += $(PORTSRC) $(KERNSRC) $(HALSRC) $(PLATFORMSRC) $(VARIOUSSRC)

ASMSRC = $(PORTASM)

INCDIR = $(PORTINC) $(KERNINC) $(HALINC) $(PLATFORMINC) $(CHIBIOS)/os/various

#
# Build configuration
#

USE_OPT = -falign-functions=16 --std=c99 -Wno-unused-parameter -U__STRICT_ANSI__
USE_COPT =
USE_CPPOPT = -fno-rtti

RELEASE ?= 0
ifneq ($(RELEASE),0)
    DDEFS += -DRELEASE
    USE_OPT += -O2 -fomit-frame-pointer
else
    DDEFS += -DDEBUG
    USE_OPT += -O0 -g3
endif

#
# Compiler options
#

MCU  = cortex-m3

TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CPPC = $(TRGT)g++
LD   = $(TRGT)gcc
#LD   = $(TRGT)g++
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
OD   = $(TRGT)objdump
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary

# ARM-specific options here
AOPT =

# THUMB-specific options here
TOPT = -mthumb -DTHUMB

CWARN = -Wall -Wextra -Werror -Wstrict-prototypes
CPPWARN = -Wall -Wextra -Werror

# asm statement fix
DDEFS += -Dasm=__asm

include $(CHIBIOS)/os/ports/GCC/ARMCMx/rules.mk
