#include gmd
include ../makefile.conf
NAME=blinky
STARTUP_DEFS=-D__NO_SYSTEM_INIT

LDSCRIPTS= -L ../ldscripts -T gcc.ld
LFLAGS=$(USE_NANO) $(USE_SEMIHOST) $(LDSCRIPTS) $(GC) $(MAP)

$(NAME)-$(CORE).axf: $(NAME).c $(STARTUP)
	$(CC) $^ $(CFLAGS) -fno-exceptions $(LFLAGS) -o $@

clean: 
	rm -f $(NAME)*.axf *.map
	
all:
 	$(__BREAKPOINT)