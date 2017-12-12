#
# Pavel Kirienko <pavel.kirienko@gmail.com>
#

SRC = ardubenchmark.c

MCU = atmega328p
DEF = -DF_CPU=16000000

FLAGS  = -O3 -mmcu=$(MCU) -Wl,-u,vfprintf -lprintf_flt -Wl,-u,vfscanf -lscanf_flt -lm
CFLAGS = $(FLAGS) -ffunction-sections -fdata-sections -Wall -Wextra -Werror -pedantic -Wno-unused-parameter -std=c99

LDFLAGS = $(FLAGS) -lgcc -lc

# ---------------

AVRDUDE_PORT ?= /dev/ttyUSB0

# ---------------

COBJ = $(SRC:.c=.o)
SOBJ = $(ASMFILES:.s=.o)
OBJ  = $(COBJ) $(SOBJ)

CROSS_COMPILE = avr-
CC   = $(CROSS_COMPILE)gcc
AS   = $(CROSS_COMPILE)as
LD   = $(CROSS_COMPILE)gcc
CP   = ${CROSS_COMPILE}objcopy
SIZE = ${CROSS_COMPILE}size

# ---------------

all: output.elf output.hex size

%.elf: $(OBJ)
	$(LD) $(LDFLAGS) $(OBJ) -o $@

%.hex: %.elf
	$(CP) -O ihex -R .eeprom $< $@

$(COBJ): %.o: %.c
	$(CC) -c $(DEF) $(INC) $(CFLAGS) $< -o $@

clean:
	rm -f output.elf output.hex $(OBJ)

size:
	@echo $(MAKEFILE_LIST)
	@if [ -f output.elf ]; then echo; $(SIZE) $(SOBJ) $(COBJ) -t; echo; fi;

sizex:
	$(SIZE) $(SOBJ) $(COBJ) -t
	@echo
	$(SIZE) output.elf -A

dude:
	avrdude -p$(MCU) -carduino -P$(AVRDUDE_PORT) -b57600 -Uflash:w:output.hex:i

.PHONY: all clean size sizex dude
