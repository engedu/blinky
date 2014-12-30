# Arch
ARCH = arm-none-eabi

# Tools
CC      = $(ARCH)-gcc
LD      = $(ARCH)-ld
AR      = $(ARCH)-ar
AS      = $(ARCH)-as
CP      = $(ARCH)-objcopy
OD		= $(ARCH)-objdump
SIZE	= $(ARCH)-size

# Core
CORTEX_M=3
CORE=CM$(CORTEX_M)

# NXP Lib
INC_NXP = -I ..
INC_NXP += -I ../NXP/Core/CM3/CoreSupport
INC_NXP += -I ../NXP/Core/CM3/DeviceSupport/NXP/LPC17xx
INC_NXP += -I ../NXP/Drivers/include

# enable newlib-nano 
USE_NANO=--specs=nano.specs

# enable/disbale seimhosting
USE_SEMIHOST=--specs=rdimon.specs
#USE_NOHOST=--specs=nosys.specs

# Options for specific architecture
# -mthumb -mcpu=cortex-m3
ARCH_FLAGS= -mthumb -mcpu=cortex-m$(CORTEX_M) -mthumb -march=armv7-m

# Startup code
STARTUP=../startup/startup_LPC1768.S

# Link for code size
GC=-Wl,--gc-sections


STARTUP_DEFS=-D__NO_SYSTEM_INIT

# CFLAGS
CFLAGS=$(USE_NANO) $(USE_SEMIHOST) $(ARCH_FLAGS) $(STARTUP_DEFS) $(GC) $(MAP) -std=c11 -O0 -g3 -pedantic -Wall -Wextra -c -fmessage-length=0 -flto -ffunction-sections -fdata-sections $(INC_NXP)

# MAP
MAP=-Wl,-Map=blinky.map

# LFLAGS
LFLAGS=$(USE_NANO) $(USE_SEMIHOST) $(LDSCRIPTS) $(GC) $(MAP)
#LFLAGS=$(LDSCRIPTS) 

# Linker
LDSCRIPTS=-L ../ldscripts -T lpc1768.ld


blinky.o:
	$(CC) -c $(CFLAGS) -g blinky.c -o $@

startup.o:
	$(CC) -c $(CFLAGS) -g $(STARTUP) -o $@

funtions.o:
	$(CC) -c $(CFLAGS) -g LPC1768Function.c -o $@

debug_frmwrk.o:
	$(CC) -c $(CFLAGS) -g ../NXP/Drivers/source/debug_frmwrk.c -o $@

lpc17xx_uart.o:
	$(CC) -c $(CFLAGS) -g ../NXP/Drivers/source/lpc17xx_uart.c -o $@

core_cm3.o:
	$(CC) -c $(CFLAGS) -g ../NXP/Core/CM3/CoreSupport/core_cm3.c -o $@

system_LPC17xx.o:
	$(CC) -c $(CFLAGS) -g ../NXP/Core/CM3/DeviceSupport/NXP/LPC17xx/system_LPC17xx.c -o $@

lpc17xx_exti.o:
	$(CC) -c $(CFLAGS) -g ../NXP/Drivers/source/lpc17xx_exti.c -o $@

lpc17xx_libcfg_default.o:
	$(CC) -c $(CFLAGS) -g ../NXP/Drivers/source/lpc17xx_libcfg_default.c -o $@

lpc17xx_clkpwr.o:        
	$(CC) -c $(CFLAGS) -g ../NXP/Drivers/source/lpc17xx_clkpwr.c -o $@

lpc17xx_pinsel.o:        
	$(CC) -c $(CFLAGS) -g ../NXP/Drivers/source/lpc17xx_pinsel.c -o $@

all: lpc17xx_pinsel.o lpc17xx_clkpwr.o lpc17xx_libcfg_default.o lpc17xx_exti.o system_LPC17xx.o lpc17xx_uart.o debug_frmwrk.o blinky.o startup.o funtions.o 
	$(CC) $(LFLAGS) lpc17xx_pinsel.o lpc17xx_clkpwr.o lpc17xx_libcfg_default.o lpc17xx_exti.o system_LPC17xx.o lpc17xx_uart.o debug_frmwrk.o blinky.o funtions.o startup.o -o main.elf
	$(CP) -O binary main.elf main.bin
	../ext/checksum main.bin

clean:
	rm -f *.o
	rm -f *.elf
	rm -f *.bin
	rm -r *.map