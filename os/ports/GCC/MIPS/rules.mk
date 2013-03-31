# MIPS common makefile scripts and rules.

# Output directory and files
ifeq ($(BUILDDIR),)
  BUILDDIR = build
endif
ifeq ($(BUILDDIR),.)
  BUILDDIR = build
endif
OUTFILES += $(BUILDDIR)/$(PROJECT).elf \
            $(BUILDDIR)/$(PROJECT).elf.strip \
            $(BUILDDIR)/$(PROJECT).dmp

# Automatic compiler options
OPT       = $(USE_OPT)
COPT      = $(USE_COPT)
CPPOPT    = $(USE_CPPOPT)
LDOPT     = $(USE_LDOPT)
ifeq ($(USE_LINK_GC),yes)
  OPT += -ffunction-sections -fdata-sections
endif
ifeq ($(USE_MIPS16),yes)
  COPT    += -mips16 -minterlink-mips16 -mlong-calls
  CPPOPT  += -mips16 -minterlink-mips16 -mlong-calls
  OPT     += -DMIPS_USE_MIPS16_ISA
endif

# Source files path
SRCPATHS  = $(sort $(dir $(ASMSRC)) $(dir $(CSRC)) $(dir $(CPPSRC)))

# Various directories
OBJDIR    = $(BUILDDIR)/obj
LSTDIR    = $(BUILDDIR)/lst

# Object files groups
COBJS     = $(addprefix $(OBJDIR)/, $(notdir $(CSRC:.c=.o)))
CPPOBJS   = $(addprefix $(OBJDIR)/, $(notdir $(CPPSRC:.cpp=.o)))
ASMOBJS   = $(addprefix $(OBJDIR)/, $(notdir $(ASMSRC:.s=.o)))
OBJS      = $(ASMOBJS) $(COBJS) $(CPPOBJS)

# Paths
IINCDIR   = $(patsubst %,-I%,$(INCDIR) $(DINCDIR) $(UINCDIR))
LLIBDIR   = $(patsubst %,-L%,$(DLIBDIR) $(ULIBDIR))

# Macros
DEFS      = $(DDEFS) $(UDEFS)
ADEFS     = $(DADEFS) $(UADEFS)

# Libs
LIBS      = $(DLIBS) $(ULIBS)

# Various settings
MCFLAGS   = -mips32r2 -G0 -fno-pic -fno-PIC -mno-abicalls -msoft-float -fomit-frame-pointer
ODFLAGS   = -x --syms
ASFLAGS   = $(MCFLAGS) $(OPT) -Wa,-amhls=$(LSTDIR)/$(notdir $(<:.s=.lst)) $(ADEFS)
CFLAGS    = $(MCFLAGS) $(OPT) $(COPT) $(CWARN) -Wa,-alms=$(LSTDIR)/$(notdir $(<:.c=.lst)) $(DEFS)
CPPFLAGS  = $(MCFLAGS) $(OPT) $(CPPOPT) $(CPPWARN) -Wa,-alms=$(LSTDIR)/$(notdir $(<:.cpp=.lst)) $(DEFS)
ifeq ($(USE_LINK_GC),yes)
  LDFLAGS = $(MCFLAGS) -G0 -fno-pic -fno-PIC -static -n -nostdlib -nostartfiles $(LDOPT) -Wl,--script=$(LDSCRIPT) -Wl,-Map=$(BUILDDIR)/$(PROJECT).map,--cref,--no-warn-mismatch,--gc-sections $(LLIBDIR)
else
  LDFLAGS = $(MCFLAGS) -G0 -fno-pic -fno-PIC -static -n -nostdlib -nostartfiles $(LDOPT) -Wl,--script=$(LDSCRIPT) -Wl,-Map=$(BUILDDIR)/$(PROJECT).map,--cref,--no-warn-mismatch $(LLIBDIR)
endif

# Generate dependency information
CFLAGS   += -MD -MP -MF .dep/$(@F).d
ASFLAGS  += -MD -MP -MF .dep/$(@F).d
CPPFLAGS += -MD -MP -MF .dep/$(@F).d

ifeq ($(USE_VERBOSE_COMPILE),yes)
Q :=
else
Q := @
endif

# Paths where to search for sources
VPATH     = $(SRCPATHS)

#
# Makefile rules
#

all: $(OBJS) $(OUTFILES) MAKE_ALL_RULE_HOOK

MAKE_ALL_RULE_HOOK:

$(OBJS): | $(BUILDDIR)

$(BUILDDIR) $(OBJDIR) $(LSTDIR):
	@echo Compiler Options
	@echo $(CC) -c $(CFLAGS) -I. $(IINCDIR)
	@mkdir -p $(OBJDIR)
	@mkdir -p $(LSTDIR)

$(CPPOBJS) : $(OBJDIR)/%.o : %.cpp Makefile
	@echo Compiling $<
	$(Q)$(CPPC) -c $(CPPFLAGS) $(AOPT) -I. $(IINCDIR) $< -o $@

$(COBJS) : $(OBJDIR)/%.o : %.c Makefile
	@echo Compiling $<
	$(Q)$(CC) -c $(CFLAGS) $(AOPT) -I. $(IINCDIR) $< -o $@

$(ASMOBJS) : $(OBJDIR)/%.o : %.s Makefile
	@echo Compiling $<
	$(Q)$(AS) -c $(ASFLAGS) -I. $(IINCDIR) $< -o $@

$(BUILDDIR)/%.elf: $(OBJS) $(LDSCRIPT)
	@echo Linking $@
	$(Q)$(LD) $(OBJS) $(LDFLAGS) $(LIBS) -o $@

$(BUILDDIR)/%.elf.strip: $(BUILDDIR)/%.elf
	@echo Linking $@
	$(Q)$(STRIP) $< -o $@

$(BUILDDIR)/%.dmp: $(BUILDDIR)/%.elf
	@echo Creating $@
	$(Q)$(OD) $(ODFLAGS) $< > $@

clean:
	@echo Cleaning
	-rm -fR .dep $(BUILDDIR)

#
# Include the dependency files, should be the last of the makefile
#
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# *** EOF ***
