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
XCOPT     = $(USE_XCOPT)
CPPOPT    = $(USE_CPPOPT)
LDOPT     = $(USE_LDOPT)
ifeq ($(USE_LINK_GC),yes)
  OPT += -ffunction-sections -fdata-sections
endif

# Source files path
SRCPATHS  = $(sort $(dir $(ASMSRC)) $(dir $(XASMSRC)) $(dir $(CSRC)) $(dir $(CPPSRC)))

# Various directories
OBJDIR    = $(BUILDDIR)/obj
LSTDIR    = $(BUILDDIR)/lst

# Object files groups
COBJS     = $(addprefix $(OBJDIR)/, $(notdir $(CSRC:.c=.o)))
CPPOBJS   = $(addprefix $(OBJDIR)/, $(notdir $(CPPSRC:.cpp=.o)))
ASMOBJS   = $(addprefix $(OBJDIR)/, $(notdir $(ASMSRC:.s=.o)))
XASMOBJS  = $(addprefix $(OBJDIR)/, $(notdir $(XASMSRC:.S=.o)))
XCOBJS    = $(addprefix $(OBJDIR)/, $(notdir $(XCSRC:.xc=.o)))
OBJS      = $(ASMOBJS) $(XASMOBJS) $(COBJS) $(CPPOBJS) $(XCOBJS)

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
XCFLAGS   = $(MCFLAGS) $(OPT) -xc $(XCOPT) $(CWARN) -Wa,-alms=$(LSTDIR)/$(notdir $(<:.c=.lst)) $(DEFS)
CPPFLAGS  = $(MCFLAGS) $(OPT) $(CPPOPT) $(CPPWARN) -Wa,-alms=$(LSTDIR)/$(notdir $(<:.cpp=.lst)) $(DEFS)
ifeq ($(USE_LINK_GC),yes)
  LDFLAGS = $(MCFLAGS) -G0 -fno-pic -fno-PIC -static -n -nostdlib -nostartfiles $(LDOPT) -T$(LDSCRIPT) -Wl,-Map=$(BUILDDIR)/$(PROJECT).map,--cref,--no-warn-mismatch,--gc-sections $(LLIBDIR)
else
  LDFLAGS = $(MCFLAGS) -G0 -fno-pic -fno-PIC -static -n -nostdlib -nostartfiles $(LDOPT) -T$(LDSCRIPT) -Wl,-Map=$(BUILDDIR)/$(PROJECT).map,--cref,--no-warn-mismatch $(LLIBDIR)
endif

# Generate dependency information
CFLAGS   += -MD -MP -MF .dep/$(@F).d
ASFLAGS  += -MD -MP -MF .dep/$(@F).d
CPPFLAGS += -MD -MP -MF .dep/$(@F).d

# Paths where to search for sources
VPATH     = $(SRCPATHS)

#
# Makefile rules
#

all: $(OBJS) $(OUTFILES) MAKE_ALL_RULE_HOOK

MAKE_ALL_RULE_HOOK:

$(OBJS): | $(BUILDDIR)

$(BUILDDIR) $(OBJDIR) $(LSTDIR):
ifneq ($(USE_VERBOSE_COMPILE),yes)
	@echo Compiler Options
	@echo $(CC) -c $(CFLAGS) -I. $(IINCDIR) main.c -o main.o
	@echo
endif
	mkdir -p $(OBJDIR)
	mkdir -p $(LSTDIR)

$(CPPOBJS) : $(OBJDIR)/%.o : %.cpp Makefile
ifeq ($(USE_VERBOSE_COMPILE),yes)
	@echo
	$(CPPC) -c $(CPPFLAGS) $(AOPT) -I. $(IINCDIR) $< -o $@
else
	@echo Compiling $<
	@$(CPPC) -c $(CPPFLAGS) $(AOPT) -I. $(IINCDIR) $< -o $@
endif

$(COBJS) : $(OBJDIR)/%.o : %.c Makefile
ifeq ($(USE_VERBOSE_COMPILE),yes)
	@echo
	$(CC) -c $(CFLAGS) $(AOPT) -I. $(IINCDIR) $< -o $@
else
	@echo Compiling $<
	@$(CC) -c $(CFLAGS) $(AOPT) -I. $(IINCDIR) $< -o $@
endif

$(ASMOBJS) : $(OBJDIR)/%.o : %.s Makefile
ifeq ($(USE_VERBOSE_COMPILE),yes)
	@echo
	$(AS) -c $(ASFLAGS) -I. $(IINCDIR) $< -o $@
else
	@echo Compiling $<
	@$(AS) -c $(ASFLAGS) -I. $(IINCDIR) $< -o $@
endif

$(XASMOBJS) : $(OBJDIR)/%.o : %.S Makefile
ifeq ($(USE_VERBOSE_COMPILE),yes)
	@echo
	$(AS) -c $(ASFLAGS) -I. $(IINCDIR) $< -o $@
else
	@echo Compiling $<
	@$(AS) -c $(ASFLAGS) -I. $(IINCDIR) $< -o $@
endif

$(XCOBJS) : $(OBJDIR)/%.o : %.xc Makefile
ifeq ($(USE_VERBOSE_COMPILE),yes)
	@echo
	$(XCC) -c $(XCFLAGS) $(AOPT) -I. $(IINCDIR) $< -o $@
else
	@echo Compiling $<
	@$(XCC) -c $(XCFLAGS) $(AOPT) -I. $(IINCDIR) $< -o $@
endif

$(BUILDDIR)/%.elf: $(OBJS) $(LDSCRIPT)
ifeq ($(USE_VERBOSE_COMPILE),yes)
	@echo
	$(LD) $(OBJS) $(LDFLAGS) $(LIBS) -o $@
else
	@echo Linking $@
	@$(LD) $(OBJS) $(LDFLAGS) $(LIBS) -o $@
endif

$(BUILDDIR)/%.elf.strip: $(BUILDDIR)/%.elf
ifeq ($(USE_VERBOSE_COMPILE),yes)
	@echo
	$(STRIP) $< -o $@
else
	@echo Linking $@
	@$(STRIP) $< -o $@
endif

$(BUILDDIR)/%.dmp: $(BUILDDIR)/%.elf
ifeq ($(USE_VERBOSE_COMPILE),yes)
	$(OD) $(ODFLAGS) $< > $@
else
	@echo Creating $@
	@$(OD) $(ODFLAGS) $< > $@
	@echo Done
endif

clean:
	@echo Cleaning
	-rm -fR .dep $(BUILDDIR)
	@echo Done

#
# Include the dependency files, should be the last of the makefile
#
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# *** EOF ***
