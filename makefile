TC     := $(HOME)/tools/arm/gcc-arm-none-eabi-10.3-2021.10/bin
PREFIX := arm-none-eabi-
CC     := $(TC)/$(PREFIX)gcc
AS     := $(TC)/$(PREFIX)as
DUMP   := $(TC)/$(PREFIX)objdump
SIZE   := $(TC)/$(PREFIX)size 
COPY   := $(TC)/$(PREFIX)objcopy

NRFX   := /media/hdd/dev/nrfx
CMSIS  := /media/hdd/dev/CMSIS_5

STARTUP := $(NRFX)/mdk/gcc_startup_nrf52.S
LSCRIPT := $(NRFX)/mdk/nrf52832_xxaa.ld 
PROGRAM := blinky

C_SRC := blinky.c \
		 $(NRFX)/mdk/system_nrf52.c

A_SRC := $(STARTUP) 

OBJ := $(C_SRC:%.c=%.o) $(A_SRC:%.s=%.o) 

C_INC := -I $(NRFX) \
		 -I $(NRFX)/hal \
		 -I $(NRFX)/mdk \
		 -I $(CMSIS)/CMSIS/Core/Include

L_LIB := -L $(NRFX)/mdk 

MFLAGS := -mcpu=cortex-m4 -mthumb -mabi=aapcs -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D NRF52
CFLAGS := $(MFLAGS) $(C_INC) -ffunction-sections -fdata-sections \
		  -O0 -g -Wall -std=gnu99 -c 
LFLAGS := $(MFLAGS) -T $(LSCRIPT) $(L_LIB) -nodefaultlibs -nolibc -nostdlib -fno-builtin -fno-strict-aliasing -static

.PHONY: all $(PROGRAM) dump flash clean format bdbg dbg attach

all: $(PROGRAM)

$(PROGRAM): $(OBJ) 
	@$(CC) $(OBJ) -o $@ $(LFLAGS)
	@$(SIZE) $(PROGRAM)
	@$(COPY) -Oihex $(PROGRAM) $(PROGRAM).hex

%.c.o: $(C_SRC)
	@$(CC) $(CFLAGS) $< -o $@

%.s.o: $(A_SRC)
	@$(CC) $(CFLAGS) $< -o $@

%.S.o: $(A_SRC)
	@$(AS) $(CFLAGS) $< -o $@

dump:
	@$(DUMP) -D $(PROGRAM)

clean:
	@rm *.o *.elf *.hex *.bin $(PROGRAM)

format:
	@clang-format -style=llvm -dump-config > .clang-format
	@clang-format -i *.c

flash:
	@nrfjprog -f NRF52 --program $(PROGRAM).hex --chiperase --verify
	@nrfjprog -f NRF52 --reset

attach:
	@openocd -f openocd.cfg

dbg:
	arm-none-eabi-gdb -x init.gdb --args blinky

bdbg:
	arm-none-eabi-gdb --batch -x init.gdb --args blinky