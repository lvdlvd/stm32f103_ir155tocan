# Tool names
PREFIX=arm-none-eabi-
CC          := $(PREFIX)gcc
OBJCOPY     := $(PREFIX)objcopy
SIZE        := $(PREFIX)size
AR          := $(PREFIX)ar

# reported in console and used by boot0
REVISION 	:= $(shell git log -1 --format="%h")

ARCH_FLAGS	 = -mthumb -mcpu=cortex-m3
LTO_FLAGS	 = -O3 -flto -fuse-linker-plugin -ffunction-sections -fdata-sections -fverbose-asm -ffat-lto-objects
WARN_FLAGS   = -Werror -Wfatal-errors -Wall -Wextra -Wunsafe-loop-optimizations -Wdouble-promotion -Wundef  -Wno-pedantic -Wno-enum-conversion
DEBUG_FLAGS	 = -ggdb3 -DNDEBUG -D__REVISION__='0x$(REVISION)'
CFLAGS 		 = -std=gnu99 -I../lib $(ARCH_FLAGS) $(LTO_FLAGS) $(WARN_FLAGS) $(DEBUG_FLAGS)
LDFLAGS		 = -nostartfiles -lnosys -static $(ARCH_FLAGS) $(LTO_FLAGS) $(WARN_FLAGS) $(DEBUG_FLAGS)
LDFLAGS 	+= -Wl,-gc-sections,-Map,main.map -Wl,--cref
LDFLAGS 	+= -Wl,--undefined=vector_table
LD_SCRIPT 	 = stm32f103_64k.ld

.DEFAULT_GOAL := main.hex

OBJS = \
        vectors.o \
        boot.o \
        clock.o \
        gpio2.o \
		serial.o \
        bxcan.o \
        main.o \

$(OBJS): Makefile

%.o: %.c
	$(CC) -c -o $@ $(CFLAGS) $<

main.elf: $(OBJS)
	$(CC) -o $@ $^ $(LDFLAGS) -T$(LD_SCRIPT) 
	$(SIZE) $@

%.hex: %.elf
	$(OBJCOPY) -O ihex $< $@

flash: main.hex
	st-flash --reset --format ihex write $<

clean:
	rm -f *~ *.o *.a *.hex *.bin *.elf *.map

depend:
	makedepend -w150 -Y  -- *.c
