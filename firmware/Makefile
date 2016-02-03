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
        -DHW_VERSION=$(HW_VERSION)                       \
        -DNODE_NAME=\"$(PROJECT)\"

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

BOOTLOADER_SIZE = 16384

DDEFS += -DCORTEX_VTOR_INIT=$(BOOTLOADER_SIZE)

LDSCRIPT= linker.ld

RELEASE_OPT = -O2 -fomit-frame-pointer
DEBUG_OPT = -O2 -g3

# More warnings are enabled in the included makefile
CWARN = -Wshadow -Wpointer-arith -Wno-packed -Wno-attributes -Wno-error=undef -Wno-error=shadow
CPPWARN = $(CWARN)

HW_VERSION_MAJOR_MINOR := $(HW_VERSION).0
FW_VERSION_MAJOR_MINOR_VCS_HASH := $(FW_VERSION_MAJOR).$(FW_VERSION_MINOR).$(GIT_HASH)
COMPOUND_IMAGE_FILE := $(PROJECT)-$(HW_VERSION_MAJOR_MINOR)-$(FW_VERSION_MAJOR_MINOR_VCS_HASH).compound.bin
BOOTLOADER_DIR := $(abspath $(SAPOG_BASE)/../bootloader)
BOOTLOADER_IMAGE := $(BOOTLOADER_DIR)/build/bootloader.bin

# This rule hook is defined in the ChibiOS build system
POST_MAKE_ALL_RULE_HOOK: build/$(PROJECT).bin build/$(PROJECT).elf
	# Building the bootloader
	+cd $(BOOTLOADER_DIR) && make
	
	# Removing previous build outputs that could use a different git hash
	rm -rf build/*.uavcan.bin build/*.compound.bin
	
	# Generating compound image with embedded bootloader
	cd build && dd if=/dev/zero bs=$(BOOTLOADER_SIZE) count=1 | tr "\000" "\377" >padded_bootloader.tmp.bin
	cd build && dd if=$(BOOTLOADER_IMAGE) of=padded_bootloader.tmp.bin conv=notrunc
	cd build && cat padded_bootloader.tmp.bin $(PROJECT).bin >$(COMPOUND_IMAGE_FILE)
	
	# Generating the signed image for the bootloader
	cd build && ../zubax_chibios/tools/make_boot_descriptor.py $(PROJECT).bin $(PROJECT) $(HW_VERSION_MAJOR_MINOR) \
	                                                           --also-patch-descriptor-in=$(PROJECT).elf           \
	                                                           --also-patch-descriptor-in=$(COMPOUND_IMAGE_FILE)
	
	# Injecting the bootloader into the final ELF
	cd build && $(TOOLCHAIN_PREFIX)-objcopy --add-section bootloader=$(BOOTLOADER_IMAGE)   \
	                                        --set-section-flags bootloader=load,alloc      \
	                                        --change-section-address bootloader=0x08000000 \
	                                        $(PROJECT).elf compound.elf
	
	# Removing temporary files
	cd build && rm -f $(PROJECT).bin $(PROJECT).elf *.hex *.tmp.bin

include zubax_chibios/rules_stm32f105_107.mk
