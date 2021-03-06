#
#             LUFA Library
#     Copyright (C) Dean Camera, 2012.
#
#  dean [at] fourwalledcubicle [dot] com
#           www.lufa-lib.org
#

LUFA_BUILD_MODULES         += IAR
LUFA_BUILD_TARGETS         += all elf hex clean mostlyclean
LUFA_BUILD_MANDATORY_VARS  += TARGET ARCH MCU SRC F_USB LUFA_PATH
LUFA_BUILD_OPTIONAL_VARS   += BOARD OPTIMIZATION CPP_STANDARD F_CPU C_FLAGS CPP_FLAGS ASM_FLAGS CC_FLAGS LD_FLAGS OBJDIR OBJECT_FILES
LUFA_BUILD_PROVIDED_VARS   += 
LUFA_BUILD_PROVIDED_MACROS += 

# -----------------------------------------------------------------------------
#               LUFA IAR Compiler Buildsystem Makefile Module.
# -----------------------------------------------------------------------------
# DESCRIPTION:
#   Provides a set of targets to build a C, C++ and/or Assembly application
#   via the IAR compiler.
# -----------------------------------------------------------------------------
# TARGETS:
#
#    all                       - Build application and list size
#    elf                       - Build application ELF debug object file
#    hex                       - Build application HEX object files
#    clean                     - Remove output files
#    mostlyclean               - Remove intermediatary output files, but
#                                preserve binaries
#
# MANDATORY PARAMETERS:
#
#    TARGET                    - Application name
#    ARCH                      - Device architecture name
#    MCU                       - Microcontroller device model name
#    SRC                       - List of input source files (*.c, *.cpp, *.S)
#    F_USB                     - Speed of the input clock of the USB controller
#                                in Hz
#    LUFA_PATH                 - Path to the LUFA library core
#
# OPTIONAL PARAMETERS:
#
#    BOARD                     - LUFA board hardware
#    OPTIMIZATION              - Optimization level
#    CPP_STANDARD              - C++ Language Standard to use
#    F_CPU                     - Speed of the CPU, in Hz
#    C_FLAGS                   - Flags to pass to the C compiler only
#    CPP_FLAGS                 - Flags to pass to the C++ compiler only
#    ASM_FLAGS                 - Flags to pass to the assembler only
#    CC_FLAGS                  - Common flags to pass to the C/C++ compiler and
#                                assembler
#    LD_FLAGS                  - Flags to pass to the linker
#    OBJDIR                    - Directory for the output object and dependency
#                                files; if equal to ".", the output files will
#                                be generated in the same folder as the sources
#    OBJECT_FILES              - Extra object files to link in to the binaries
#
# PROVIDED VARIABLES:
#
#    (None)
#
# PROVIDED MACROS:
#
#    (None)
#
# -----------------------------------------------------------------------------

ERROR_IF_UNSET   = $(if $(filter undefined, $(origin $(strip $(1)))), $(error Makefile $(strip $(1)) value not set))
ERROR_IF_EMPTY   = $(if $(strip $($(strip $(1)))), , $(error Makefile $(strip $(1)) option cannot be blank))
ERROR_IF_NONBOOL = $(if $(filter Y N, $($(strip $(1)))), , $(error Makefile $(strip $(1)) option must be Y or N))

# Default values of optionally user-supplied variables
BOARD           ?= NONE
OPTIMIZATION    ?= s
F_CPU           ?=
CPP_STANDARD    ?= ec++
C_FLAGS         ?=
CPP_FLAGS       ?=
ASM_FLAGS       ?=
CC_FLAGS        ?=
OBJDIR          ?= .
OBJECT_FILES    ?=

# Sanity check user supplied values
$(foreach MANDATORY_VAR, $(LUFA_BUILD_MANDATORY_VARS), $(call ERROR_IF_UNSET, $(MANDATORY_VAR)))
$(call ERROR_IF_EMPTY, MCU)
$(call ERROR_IF_EMPTY, TARGET)
$(call ERROR_IF_EMPTY, ARCH)
$(call ERROR_IF_EMPTY, F_USB)
$(call ERROR_IF_EMPTY, LUFA_PATH)
$(call ERROR_IF_EMPTY, BOARD)
$(call ERROR_IF_EMPTY, OPTIMIZATION)
$(call ERROR_IF_EMPTY, CPP_STANDARD)
$(call ERROR_IF_EMPTY, OBJDIR)

# Determine the utility prefix to use for the selected architecture
ifeq ($(ARCH), AVR8)
   CROSS        := avr
else ifeq ($(ARCH), XMEGA)
   CROSS        := avr
   $(warning The XMEGA device support is currently EXPERIMENTAL (incomplete and/or non-functional), and is included for preview purposes only.)
else ifeq ($(ARCH), UC3)
   CROSS        := avr32
   $(warning The UC3 device support is currently EXPERIMENTAL (incomplete and/or non-functional), and is included for preview purposes only.)
else
   $(error Unsupported architecture "$(ARCH)")
endif

$(warning IAR compiler support is currently EXPERIMENTAL (incomplete and/or non-functional), and is included for preview purposes only.)

# Output Messages
MSG_BUILD_BEGIN  := Begin compilation of project \"$(TARGET)\"...
MSG_BUILD_END    := Finished building project \"$(TARGET)\".
MSG_COMPILE_CMD  := ' [ICC]     :'
MSG_ASSEMBLE_CMD := ' [IAS]     :'
MSG_LINKER_CMD   := ' [XLNK]    :'

# Convert input source file list to differentiate them by type
C_SOURCE   = $(filter %.c, $(SRC))
CPP_SOURCE = $(filter %.cpp, $(SRC))
ASM_SOURCE = $(filter %.S, $(SRC))

# Create a list of unknown source file types, if any are found throw an error
UNKNOWN_SOURCE = $(filter-out $(C_SOURCE) $(CPP_SOURCE) $(ASM_SOURCE), $(SRC))
ifneq ($(UNKNOWN_SOURCE),)
   $(error Unknown input source formats: $(UNKNOWN_SOURCE))
endif

# Convert input source filenames into a list of required output object files
OBJECT_FILES += $(addsuffix .o, $(basename $(SRC)))
ifneq ($(OBJDIR),.)
   $(shell mkdir $(OBJDIR) 2> /dev/null)   
   VPATH           += $(dir $(SRC))
   OBJECT_FILES    := $(addprefix $(patsubst %/,%,$(OBJDIR))/, $(notdir $(OBJECT_FILES)))
   
   # Check if any object file (without path) appears more than once in the object file list
   ifneq ($(words $(sort $(OBJECT_FILES))), $(words $(OBJECT_FILES)))
       $(error Cannot build with OBJDIR parameter set - one or more object file name is not unique)
   endif
endif

DEPENDENCY_FILES = $(OBJECT_FILES:%.o=%.d)

# Create a list of common flags to pass to the compiler/linker/assembler
BASE_CC_FLAGS    := 
ifeq ($(ARCH), AVR8)
   BASE_CC_FLAGS += --variable_enum_size enabled
else ifeq ($(ARCH), XMEGA)
   BASE_CC_FLAGS += --variable_enum_size enabled
else ifeq ($(ARCH), UC3)
   BASE_CC_FLAGS += --code_model=large --data_model=large 
endif
BASE_CC_FLAGS += --silent --debug --cpu=$(MCU)
BASE_CC_FLAGS += -I. -I$(patsubst %/,%,$(LUFA_PATH))/..
BASE_CC_FLAGS += -DARCH=ARCH_$(ARCH) -DBOARD=BOARD_$(BOARD) -DF_USB=$(F_USB)UL
ifneq ($(F_CPU),)
   BASE_CC_FLAGS += -DF_CPU=$(F_CPU)UL
endif

# Additional language specific compiler flags
BASE_C_FLAGS   := -e --diag_suppress Pe546,Pa082 --dlib_config DLib_Config_Normal.h --require_prototypes --vla --no_tbaa --char_is_unsigned -O$(OPTIMIZATION) 
BASE_CPP_FLAGS := $(BASE_C_FLAGS) --$(CPP_STANDARD)
BASE_ASM_FLAGS := 

# TODO: FIXME!
IAR_PATH := C:\Program\ Files\ \(x86\)\IAR\ Systems\Embedded\ Workbench\ 6.0\ Kickstart

# Create a list of flags to pass to the linker
BASE_LD_FLAGS := -l $(TARGET).map $(IAR_PATH)\avr32\lib\dlavr32allasn.r82 -f $(IAR_PATH)\avr32\config\lnk$(MCU:at32%=%).xcl -D_SSTACK_SIZE=1000 -D_CSTACK_SIZE=2000 -D_HEAP_SIZE=1000 -I$(IAR_PATH)\avr32\LIB\ -rt -s __program_start -g__init_all_ihandlers -g__handle_all_exceptions

build_begin:
	@echo ""
	@echo $(MSG_BUILD_BEGIN)
	@echo ""
	
build_end:
	@echo $(MSG_BUILD_END)
	@echo ""

check_source:
	@for f in $(SRC); do \
		if [ ! -f $$f ]; then \
			echo "Error: Source file not found: $$f"; \
			exit 1; \
		fi; \
	 done

clean:
	@echo $(MSG_REMOVE_CMD) Removing object files of \"$(TARGET)\"
	rm -f $(OBJECT_FILES)
	@echo $(MSG_REMOVE_CMD) Removing dependency files of \"$(TARGET)\"
	rm -f $(DEPENDENCY_FILES)
	@echo $(MSG_REMOVE_CMD) Removing output files of \"$(TARGET)\"
	rm -f $(TARGET).elf $(TARGET).hex $(TARGET).map

all: build_begin check_source elf hex build_end

elf: $(TARGET).elf
hex: $(TARGET).hex

$(OBJDIR)/%.o: %.c $(MAKEFILE_LIST)
	@echo $(MSG_COMPILE_CMD) Compiling C file \"$(notdir $<)\"
	icc$(CROSS) $(BASE_CC_FLAGS) $(BASE_C_FLAGS) $(CC_FLAGS) $(C_FLAGS) --dependencies=m $(@:%.o=%.d) $< -o $@

$(OBJDIR)/%.o: %.cpp $(MAKEFILE_LIST)
	@echo $(MSG_COMPILE_CMD) Compiling C++ file \"$(notdir $<)\"
	icc$(CROSS) $(BASE_CC_FLAGS) $(BASE_CPP_FLAGS) $(CC_FLAGS) $(CPP_FLAGS) --dependencies=m $(@:%.o=%.d) $< -o $@

$(OBJDIR)/%.o: %.S $(MAKEFILE_LIST)
	@echo $(MSG_ASSEMBLE_CMD) Assembling \"$(notdir $<)\"
	a$(CROSS) $(BASE_CC_FLAGS) $(BASE_ASM_FLAGS) $(CC_FLAGS) $(ASM_FLAGS) --dependencies=m $(@:%.o=%.d) $< -o $@

.PRECIOUS  : $(OBJECT_FILES)
.SECONDARY : %.elf
%.elf: $(OBJECT_FILES)
	@echo $(MSG_LINKER_CMD) Linking object files into \"$@\"
	xlink $(BASE_LD_FLAGS) $(LD_FLAGS) -Felf -yp -S $^ -o $@

%.hex : $(OBJECT_FILES)
	@echo $(MSG_LINKER_CMD) Linking object files into \"$@\"
	xlink $(BASE_LD_FLAGS) $(LD_FLAGS) -Fintel-extended $^ -o $@

# Include build dependency files
-include $(DEPENDENCY_FILES)

# Phony build targets for this module
.PHONY: build_begin build_end check_source clean elf hex
