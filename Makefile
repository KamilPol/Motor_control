#=============================================================================#
# toolchain configuration
#=============================================================================#

ARCH 	= arm-none-eabi
CC 		= $(ARCH)-gcc
CXX 	= $(ARCH)-g++
LD 		= $(ARCH)-g++
AS 		= $(ARCH)-as
OBJCPY 	= $(ARCH)-objcopy
OBJDMP 	= $(ARCH)-objdump
GDB 	= $(ARCH)-gdb
SZ 		= $(ARCH)-size

OPTFLAG = -O2 -flto -ffat-lto-objects ### optimalization changed to lvl 0 (from 2)

#=============================================================================#
# project configuration
#=============================================q================================#

DRIVER_DIR = drivers
MODULE_DIR = modules
LIB_DIR = lib
#LIBS = buffer logger format shell text_tools parser math_algo analog_scaler hash rand_tools stubs can_processor mosfet_io
LIBS = stubs buffer typeConverter states lcd PID motor motor_process motor_math

LD_SCRIPT := $(DRIVER_DIR)/linker_scripts/STM32G474RETX_FLASH.ld

BUILD_DIR = build
BINARY_NAME = BINARY

GLOBAL_DEFS = STM32G474xx
CORE = cortex-m4
CORE_FLAGS = -mcpu=$(CORE) -mfpu=fpv4-sp-d16 -mfloat-abi=hard

CXX_WARNINGS = -Wall -Wextra
C_WARNINGS = -Wall -Wextra

CXX_STD = c++20
C_STD = c17

#=============================================================================#
# source files configuration
#=============================================================================#

DRIVERS := $(patsubst %/., %, $(wildcard $(DRIVER_DIR)/*/.))
MODULES := $(patsubst %/., %, $(wildcard $(MODULE_DIR)/*/.))
LIBS := $(addprefix $(LIB_DIR)/, $(LIBS))

SRCS_DIRS = config $(DRIVERS) $(LIBS) $(MODULES)
INC_DIRS = config $(DRIVERS) $(LIBS) $(MODULES)

CPP_EXT = cpp
C_EXT = c
AS_EXT = s

CPP_SRCS = $(wildcard $(patsubst %, %/*.$(CPP_EXT), . $(SRCS_DIRS)))
C_SRCS = $(wildcard $(patsubst %, %/*.$(C_EXT), . $(SRCS_DIRS)))
AS_SRCS = $(DRIVER_DIR)/startup/startup_stm32g474retx.s

SOURCES = $(CPP_SRCS) $(C_SRCS) $(AS_SRCS)

INCLUDES = $(addprefix -I, $(INC_DIRS))

#=============================================================================#
# various compilation flags
#=============================================================================#

AFLAGS := $(CORE_FLAGS)

CFLAGS = -ggdb \
		 -mthumb \
		 -fno-common \
		 -ffast-math \
		 $(CORE_FLAGS) \
		 $(addprefix -D, $(GLOBAL_DEFS)) \
		 $(INCLUDES) \
		 -fdata-sections \
		 -ffunction-sections \
		 -fomit-frame-pointer \
		 -nostartfiles \
		 -ffreestanding \
		 -fno-strict-aliasing \
		 $(C_WARNINGS)

CXXFLAGS = $(CFLAGS) \
		-std=$(CXX_STD) \
		-fno-rtti \
		-fno-exceptions \
		-fno-unwind-tables \
		-ffreestanding \
		-fno-threadsafe-statics \
		-Werror=return-type \
		-Wdouble-promotion \
		-Wno-volatile \
		 $(EXTRACXXFLAGS) \
		 $(CXX_WARNINGS)

LFLAGS = -Wl,--gc-sections \
		 -Wl,-Map,$(BUILD_DIR)/$(BINARY_NAME).map,--cref \
		 $(CORE_FLAGS)  \
		 -T $(LD_SCRIPT) \
		 -nostdlib \
		 -nostartfiles \
		 -ffreestanding \
		 -ffunction-sections \
		 -fdata-sections \
		 -Wl,--no-warn-rwx-segment

DEPFLAGS = -MMD -MP -MF $(OBJ_DIR)/$(basename $<).d

#=============================================================================#
# do some formatting
#=============================================================================#

OBJ_DIR = $(BUILD_DIR)/obj

OBJECTS   = $(addprefix $(OBJ_DIR)/, $(addsuffix .o, $(basename $(SOURCES))))
DEPS   	  = $(addprefix $(OBJ_DIR)/, $(addsuffix .d, $(basename $(SOURCES))))

#contents of output directory
GENERATED = $(wildcard $(patsubst %, $(OUT_DIR_F)*.%, bin d dmp elf hex lss lst map o))

SZOPTS 	= -d

ELF 	= $(BUILD_DIR)/$(BINARY_NAME).elf
HEX 	= $(BUILD_DIR)/$(BINARY_NAME).hex
BIN 	= $(BUILD_DIR)/$(BINARY_NAME).bin
LSS		= $(BUILD_DIR)/$(BINARY_NAME).lss

#=============================================================================#
# build process
#=============================================================================#

all: Makefile $(ELF) $(LSS) $(BIN) print_size

$(OBJECTS) : Makefile

$(OBJ_DIR)/%.o: %.$(AS_EXT)
	@if not exist "$(dir $@)" mkdir "$(dir $@)"
	$(info AS: Building $< at $(OPTFLAG))
#$(info $(AS) $(AFLAGS) $< -o $@)
	@$(AS) $(AFLAGS) $< -o $@ 

$(OBJ_DIR)/%.o: %.$(C_EXT) $(OBJ_DIR)/%.d
	@if not exist "$(dir $@)" mkdir "$(dir $@)"
	$(info CC: Building $< at $(OPTFLAG))
#$(info $(CC) -c $(DEPFLAGS) $(OPTFLAG) $(CFLAGS) -Wstrict-prototypes $< -o $@)
	@$(CC) -c $(DEPFLAGS) $(OPTFLAG) $(CFLAGS) -Wstrict-prototypes $< -o $@

$(OBJ_DIR)/%.o: %.$(CPP_EXT) $(OBJ_DIR)/%.d
	@if not exist "$(dir $@)" mkdir "$(dir $@)"
	$(info CXX: Building $< at $(OPTFLAG))
#$(info $(CXX) -c $(DEPFLAGS) $(OPTFLAG) $(CXXFLAGS) $< -o $@)
	@$(CXX) -c $(DEPFLAGS) $(OPTFLAG) $(CXXFLAGS) $< -o $@

$(ELF): $(OBJECTS) $(LD_SCRIPT)
	$(info LD: Linking...)
	@$(LD) $(LFLAGS) -o $@ $(OBJECTS) -lgcc

$(BIN): $(ELF)
	$(info Creating binary: $(BIN))
	@$(OBJCPY) -O binary $< $@

$(HEX): $(ELF)
	$(info Creating HEX: $(HEX))
	@$(OBJCPY) --output-target=ihex $< $@
	@$(SZ) $(SZOPTS) $(ELF)

$(LSS) : $(ELF)
	$(info Creating listing: $(LSS))
	@$(OBJDMP) -S $< > $@

print_size :
	@echo Size of modules:
	@$(SZ) -B -t --common $(OBJECTS)
	@echo Size of target .elf file:
	@$(SZ) -B $(ELF)

%.d: ;

clean:
	rm -rf build

.PRECIOUS: $(DEPS) $(OBJECTS) $(ELF)
.PHONY: all clean install

-include $(DEPS)