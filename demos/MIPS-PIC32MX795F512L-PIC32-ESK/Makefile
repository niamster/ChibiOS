##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

#USE_VERBOSE_COMPILE = yes

# Compiler options here.
ifeq ($(USE_OPT),)
  USE_OPT = -EL -O2 -ggdb -ffreestanding
  USE_OPT += -mtune=4kc
  # usually gives very good i-cache hit rate
  USE_OPT += -falign-functions=16
  USE_OPT += -D__32MX795F512L__
  USE_OPT += -DCHPRINTF_CR_INSERT=TRUE
  USE_OPT += -DCHPRINTF_USE_FLOAT=TRUE
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT =
endif

# LD specific options here (added to USE_OPT).
ifeq ($(USE_LDOPT),)
  USE_LDOPT = -EL -lc
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti -fno-exceptions
endif

# Enable this if you want to compile in mips16 ISA
ifeq ($(USE_MIPS16),)
  USE_MIPS16 = no
endif

# Enable this if you want the linker to remove unused code and data
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = no
endif

#
# Build global options
##############################################################################

##############################################################################
# Project, sources and paths
#

# Define project name here
PROJECT = ch

# Imported source files and paths
CHIBIOS = ../..
include $(CHIBIOS)/boards/MIPS-PIC32MX795F512L-PIC32-ESK/board.mk
include $(CHIBIOS)/os/hal/platforms/MIPS-PIC32MX/platform.mk
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/ports/GCC/MIPS/MIPS32r2/port.mk
include $(CHIBIOS)/os/kernel/kernel.mk
include $(CHIBIOS)/os/various/lwip_bindings/lwip.mk
include $(CHIBIOS)/test/test.mk

# Define linker script file here
LDSCRIPT= ld/mips_pic32mx795f512l.lds

# List C source files here
CSRC = $(PORTSRC) \
       $(KERNSRC) \
       $(TESTSRC) \
       $(HALSRC) \
       $(PLATFORMSRC) \
       $(BOARDSRC) \
       $(LWSRC) \
       $(CHIBIOS)/os/various/shell.c \
       $(CHIBIOS)/os/various/chprintf.c \
       $(CHIBIOS)/os/various/evtimer.c \
       $(CHIBIOS)/os/various/memstreams.c \
       usbcfg.c \
       stubs.c \
       ./web/web.c \
       main.c

# List C++ source files here
CPPSRC =
# List ASM source files here
ASMSRC  = $(PORTASM) $(PLATFORMASM)

INCDIR = $(PORTINC) $(KERNINC) $(TESTINC) $(TESTDMAINC) \
         $(HALINC) $(PLATFORMINC) $(BOARDINC) $(LWINC) \
         $(CHIBIOS)/os/various

#
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

MCU  = 

#TRGT ?= mips-sde-elf-
#TRGT ?= /usr/local/mips-2012.03/bin/mips-sde-elf-
TRGT ?= /usr/local/mips-gcc-4.7.2/bin/mips-elf-

CC   = $(TRGT)gcc
CPPC = $(TRGT)g++
# Enable loading with g++ only if you need C++ runtime support.
# NOTE: You can use C++ even without C++ support if you are careful. C++
#       runtime support makes code size explode.
LD   = $(TRGT)gcc
#LD   = $(TRGT)g++
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
OD   = $(TRGT)objdump
STRIP= $(TRGT)strip --strip-all
SREC = $(CP) -O srec
HEX  = $(CP) -O ihex --change-addresses=0x80000000 --change-section-address .boot=0x1FC00000 --change-section-address .devcfg=0x1FC02FF0

# Define C warning options here
CWARN = -Wall -Wextra -Wstrict-prototypes

# Define C++ warning options here
CPPWARN = -Wall -Wextra

#
# Compiler settings
##############################################################################

##############################################################################
# Start of default section
#

# List all default C defines here, like -D_DEBUG=1
DDEFS =

# List all default ASM defines here, like -D_DEBUG=1
DADEFS =

# List all default directories to look for include files here
DINCDIR =

# List the default directory to look for the libraries here
DLIBDIR =

# List all default libraries here
DLIBS =

#
# End of default section
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
UDEFS =

# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR =

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS =

#
# End of user defines
##############################################################################

OUTFILES = $(BUILDDIR)/$(PROJECT).srec \
    $(BUILDDIR)/$(PROJECT).hex

include $(CHIBIOS)/os/ports/GCC/MIPS/rules.mk

$(BUILDDIR)/%.srec: $(BUILDDIR)/%.elf $(LDSCRIPT)
ifeq ($(USE_VERBOSE_COMPILE),yes)
	$(SREC) $< $@
else
	@echo Creating $@
	@$(SREC) $< $@
endif

$(BUILDDIR)/%.hex: $(BUILDDIR)/%.elf $(LDSCRIPT)
ifeq ($(USE_VERBOSE_COMPILE),yes)
	$(HEX) $< $@
else
	@echo Creating $@
	@$(HEX) $< $@
endif
