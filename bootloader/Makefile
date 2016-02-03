##############################################################################
# GNU GCC ARM Embeded Toolchain base directories and binaries
##############################################################################
# GCC_BASE = /usr/local/opt/gcc-arm-none-eabi/

CC_BIN := $(shell dirname `which arm-none-eabi-gcc`)/
CC_LIB  := $(CC_BIN)../arm-none-eabi/lib/
CC_INC  := $(CC_BIN)../arm-none-eabi/include/
AS       := $(CC_BIN)arm-none-eabi-as
CC       := $(CC_BIN)arm-none-eabi-gcc
CPP      := $(CC_BIN)arm-none-eabi-g++
LD       := $(CC_BIN)arm-none-eabi-g++
OBJCOPY  := $(CC_BIN)arm-none-eabi-objcopy
SIZE     := $(CC_BIN)arm-none-eabi-size

PYTHON   := python


##############################################################################
# Custom options for cortex-m and cortex-r processors
##############################################################################
CORTEX_M0PLUS_CC_FLAGS  = -mthumb -mcpu=cortex-m0plus
CORTEX_M0PLUS_LIB_PATH  = $(CC_LIB)armv6-m
CORTEX_M0_CC_FLAGS      = -mthumb -mcpu=cortex-m0
CORTEX_M0_LIB_PATH      = $(CC_LIB)armv6-m
CORTEX_M1_CC_FLAGS      = -mthumb -mcpu=cortex-m1
CORTEX_M1_LIB_PATH      = $(CC_LIB)armv6-m
CORTEX_M3_CC_FLAGS      = -mthumb -mcpu=cortex-m3
CORTEX_M3_LIB_PATH      = $(CC_LIB)armv7-m
CORTEX_M4_NOFP_CC_FLAGS = -mthumb -mcpu=cortex-m4
CORTEX_M4_NOFP_LIB_PATH = $(CC_LIB)armv7e-m
CORTEX_M4_SWFP_CC_FLAGS = -mthumb -mcpu=cortex-m4 -mfloat-abi=softfp -mfpu=fpv4-sp-d16
CORTEX_M4_SWFP_LIB_PATH = $(CC_LIB)armv7e-m/softfp
CORTEX_M4_HWFP_CC_FLAGS = -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16
CORTEX_M4_HWFP_LIB_PATH = $(CC_LIB)armv7e-m/fpu
CORTEX_R4_NOFP_CC_FLAGS = -mthumb -march=armv7-r
CORTEX_R4_NOFP_LIB_PATH = $(CC_LIB)armv7-r/thumb
CORTEX_R4_SWFP_CC_FLAGS = -mthumb -march=armv7-r -mfloat-abi=softfp -mfloat-abi=softfp -mfpu=vfpv3-d16
CORTEX_R4_SWFP_LIB_PATH = $(CC_LIB)armv7-r/thumb/softfp
CORTEX_R4_HWFP_CC_FLAGS = -mthumb -march=armv7-r -mfloat-abi=softfp -mfloat-abi=hard -mfpu=vfpv3-d16
CORTEX_R4_HWFP_LIB_PATH = $(CC_LIB)armv7-r/thumb/fpu
CORTEX_R5_NOFP_CC_FLAGS = -mthumb -march=armv7-r
CORTEX_R5_NOFP_LIB_PATH = $(CC_LIB)armv7-r/thumb
CORTEX_R5_SWFP_CC_FLAGS = -mthumb -march=armv7-r -mfloat-abi=softfp -mfloat-abi=softfp -mfpu=vfpv3-d16
CORTEX_R5_SWFP_LIB_PATH = $(CC_LIB)armv7-r/thumb/softfp
CORTEX_R5_HWFP_CC_FLAGS = -mthumb -march=armv7-r -mfloat-abi=softfp -mfloat-abi=hard -mfpu=vfpv3-d16
CORTEX_R5_HWFP_LIB_PATH = $(CC_LIB)armv7-r/thumb/fpu


##############################################################################
# Main makefile project configuration
#    PROJECT      = <name of the target to be built>
#    MCU_CC_FLAGS = <one of the CC_FLAGS above>
#    MCU_LIB_PATH = <one of the LIB_PATH above>
#    OPTIMIZE_FOR = < SIZE or nothing >
#    DEBUG_LEVEL  = < -g compiler option or nothing >
#    OPTIM_LEVEL  = < -O compiler option or nothing >
##############################################################################
PROJECT           = app
BUILD             = build/
PROJECT_SRC       = $(PROJECT)/src/
PROJECT_BUILD     = $(BUILD)$(PROJECT)/
MCU_CC_FLAGS      = $(CORTEX_M4_HWFP_CC_FLAGS)
MCU_LIB_PATH      = $(CORTEX_M4_HWFP_LIB_PATH)
DEBUG_LEVEL       =
OPTIM_FLAGS       = -Os -flto
LINKER_SCRIPT     = $(PROJECT)/$(PROJECT).ld
PROJECT_OBJECTS   = $(addprefix $(PROJECT_BUILD), \
					  main.o _cxx.o configuration.o can.o shared.o \
					  m4f_float_spec.o)
PROJECT_INC_PATHS = -I$(PROJECT)/include/
PROJECT_LIB_PATHS = -L.
PROJECT_LIBRARIES =
PROJECT_SYMBOLS   = -include config.h -DTOOLCHAIN_GCC_ARM -DNO_RELOC='0'


##############################################################################
# Bootloader build configuration
#    BL              = <name of the target to be built>
#    BL_OPTIMIZE_FOR = < SIZE or nothing >
#    BL_DEBUG_LEVEL  = < -g compiler option or nothing >
#    BL_OPTIM_LEVEL  = < -O compiler option or nothing >
##############################################################################
BL                = bootloader
BL_SRC            = $(BL)/src/
BL_BUILD          = $(BUILD)$(BL)/
BL_OPTIMIZE_FOR   = #SIZE
BL_DEBUG_LEVEL    = 3
BL_OPTIM_FLAGS    = -Os #-flto
BL_LINKER_SCRIPT  = $(BL)/bootloader.ld
BL_OBJECTS        = $(addprefix $(BL_BUILD), \
					  main.o can.o crc.o flash.o random.o shared.o timer.o \
					  uavcan.o)
BL_INC_PATHS      = -I$(BL)/include/
BL_LIB_PATHS      = -L.
BL_LIBRARIES      =
BL_SYMBOLS        = -include config.h -DTOOLCHAIN_GCC_ARM -DNO_RELOC='0'


##############################################################################
# Main makefile system configuration
##############################################################################
ARCH_SRC = arch/src/
SYS_OBJECTS = stm32_vectors.o up_fpu.o stm32_start.o up_systemreset.o \
			  stm32_flash.o up_modifyreg32.o stm32_rcc.o stm32_irq.o \
			  stm32_gpio.o stm32_timerisr.o irq_attach.o irq_dispatch.o \
			  irq_initialize.o irq_unexpectedisr.o up_doirq.o \
			  up_cxxinitialize.o libc.o
SYS_INC_PATHS = -Iarch/include -Iarch/src -Iarch/src/chip -I$(CC_INC)
SYS_LIB_PATHS = -L$(MCU_LIB_PATH)
SYS_LIBRARIES =
SYS_LD_FLAGS  = --specs=nano.specs


##############################################################################
# libuavcan make configuration
##############################################################################
UAVCAN = libuavcan
UAVCAN_SRC = $(UAVCAN)/src/
UAVCAN_BUILD = $(BUILD)libuavcan/
UAVCAN_OBJECTS = $(addprefix $(UAVCAN_BUILD), \
				   marshal/uc_bit_array_copy.o marshal/uc_bit_stream.o \
				   marshal/uc_float_spec.o marshal/uc_scalar_codec.o \
				   node/uc_global_data_type_registry.o node/uc_timer.o \
				   transport/uc_crc.o transport/uc_transfer_buffer.o \
				   uc_data_type.o uc_error.o)
UAVCAN_CONFIG = -DUAVCAN_TINY=1 -DUAVCAN_DEBUG=0 \
				-DUAVCAN_GENERAL_PURPOSE_PLATFORM=0 -DUAVCAN_NOEXCEPT=1 \
				-DUAVCAN_TOSTRING=0 -DUAVCAN_USE_EXTERNAL_SNPRINTF=1 \
				-DUAVCAN_NO_ASSERTIONS=1 -DUAVCAN_NO_GLOBAL_DATA_TYPE_REGISTRY=1 \
				-DUAVCAN_USE_EXTERNAL_FLOAT16_CONVERSION=1
UAVCAN_INC_PATHS = -I$(UAVCAN)/include -I$(UAVCAN)/include/dsdlc_generated -I$(CC_INC)
UAVCAN_LIB_PATHS = -L$(MCU_LIB_PATH)
UAVCAN_LIBRARIES =


###############################################################################
# Command line building
###############################################################################
ifdef DEBUG_LEVEL
CC_DEBUG_FLAGS = -g$(DEBUG_LEVEL)
CC_SYMBOLS = -DDEBUG $(PROJECT_SYMBOLS) $(UAVCAN_CONFIG)
else
CC_DEBUG_FLAGS =
CC_SYMBOLS = -DNODEBUG $(PROJECT_SYMBOLS) $(UAVCAN_CONFIG)
endif

ifdef BL_DEBUG_LEVEL
BL_CC_DEBUG_FLAGS = -g$(BL_DEBUG_LEVEL)
BL_CC_SYMBOLS = -DDEBUG $(BL_SYMBOLS)
else
BL_CC_DEBUG_FLAGS =
BL_CC_SYMBOLS = -DNODEBUG $(BL_SYMBOLS)
endif

INCLUDE_PATHS = $(PROJECT_INC_PATHS) $(SYS_INC_PATHS) $(UAVCAN_INC_PATHS)
LIBRARY_PATHS = $(PROJECT_LIB_PATHS) $(SYS_LIB_PATHS)
ARCH_CC_FLAGS = -ffreestanding -fno-common -fmessage-length=0 -Wall -fno-exceptions -ffunction-sections -fdata-sections
CFLAGS = $(MCU_CC_FLAGS) -c $(OPTIM_FLAGS) $(CC_DEBUG_FLAGS) $(ARCH_CC_FLAGS)
CXXFLAGS = $(CFLAGS) -fno-rtti -fno-threadsafe-statics
LD_FLAGS = $(MCU_CC_FLAGS) $(OPTIM_FLAGS) -Wl,--gc-sections $(SYS_LD_FLAGS) -Wl,-Map=$(PROJECT_BUILD)$(PROJECT).map -ffreestanding -nostartfiles
LD_SYS_LIBS = $(SYS_LIBRARIES)

BL_INCLUDE_PATHS = $(BL_INC_PATHS) $(SYS_INC_PATHS)
BL_LIBRARY_PATHS = $(BL_LIB_PATHS) $(SYS_LIB_PATHS)
BL_CFLAGS = $(MCU_CC_FLAGS) -c $(BL_OPTIM_FLAGS) $(BL_CC_DEBUG_FLAGS) $(ARCH_CC_FLAGS)
BL_LD_FLAGS = $(MCU_CC_FLAGS) $(BL_OPTIM_FLAGS) -Wl,--gc-sections $(SYS_LD_FLAGS) -Wl,-Map=$(BL_BUILD)$(BL).map -ffreestanding -nostartfiles -nostdlib -nodefaultlibs

UAVCAN_CXXFLAGS = $(MCU_CC_FLAGS) -c $(OPTIM_FLAGS) $(ARCH_CC_FLAGS) -fno-rtti -fno-threadsafe-statics

###############################################################################
# Makefile execution
###############################################################################

all: build/$(BL).bin

clean:
	rm -f $(BL_OBJECTS) \
		  $(addprefix $(BL_BUILD)arch/, $(SYS_OBJECTS) libc.o up_exit.o) \
		  $(PROJECT_OBJECTS) \
		  $(addprefix $(PROJECT_BUILD)arch/, $(SYS_OBJECTS)) \
		  $(UAVCAN_OBJECTS)
	rm -rf build

$(BL_BUILD)arch/%.o: $(ARCH_SRC)%.S
	@mkdir -p $(@D)
	$(CC) $(BL_CFLAGS) $(BL_CC_SYMBOLS) -D__ASSEMBLY__ $(SYS_INC_PATHS) -o $@ $<

$(BL_BUILD)arch/%.o: $(ARCH_SRC)%.c
	@mkdir -p $(@D)
	$(CC)  $(BL_CFLAGS) $(BL_CC_SYMBOLS) -std=gnu99 $(SYS_INC_PATHS) -o $@ $<

$(BL_BUILD)%.o: $(BL_SRC)%.c
	@mkdir -p $(@D)
	$(CC)  $(BL_CFLAGS) $(BL_CC_SYMBOLS) -std=gnu99 $(BL_INCLUDE_PATHS) -o $@ $<

$(UAVCAN_BUILD)%.o: $(UAVCAN_SRC)%.cpp
	@mkdir -p $(@D)
	$(CPP) $(UAVCAN_CXXFLAGS) $(CC_SYMBOLS) -std=c++03 $(SYS_INC_PATHS) $(UAVCAN_INC_PATHS) -o $@ $<

$(PROJECT_BUILD)arch/%.o: $(ARCH_SRC)%.S
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) $(CC_SYMBOLS) -D__ASSEMBLY__ $(SYS_INC_PATHS) -o $@ $<

$(PROJECT_BUILD)arch/%.o: $(ARCH_SRC)%.c
	@mkdir -p $(@D)
	$(CC)  $(CFLAGS) $(CC_SYMBOLS) -std=gnu99 $(SYS_INC_PATHS) -o $@ $<

build/$(BL).elf: $(BL_OBJECTS) $(addprefix $(BL_BUILD)arch/, $(SYS_OBJECTS) up_exit.o)
	@mkdir -p $(@D)
	$(LD) $(BL_LD_FLAGS) -T$(BL_LINKER_SCRIPT) $(BL_LIBRARY_PATHS) -o $@ $^ $(BL_LIBRARIES) $(SYS_LIBRARIES)

build/$(BL).bin: build/$(BL).elf
	@mkdir -p $(@D)
	$(SIZE) $<
	$(OBJCOPY) -O binary $< $@
