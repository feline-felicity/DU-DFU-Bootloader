include config.mk
###### TARGET ######
TARGET=program
BUILD_DIR=build
# Provided by Tasks.json
MCU=$(error MCU not set)
ifdef CLOCK_OVERRIDE
CPU_CLOCK:=$(CLOCK_OVERRIDE)
$(info CPU clock overriden by CLOCK_OVERRIDE configuration: $(CPU_CLOCK))
else ifdef CLOCK
CPU_CLOCK:=$(CLOCK)
else
$(error CPU clock not specified)
endif

###### BUILD ######
OPTIM = $(CFG_OPTIM)
DEBUG = -g
SRCDIRS = ./src $(CFG_SRCDIRS)
INCDIRS = -I./src -I./link $(CFG_INCDIRS)
LIBDIRS = $(CFG_LIBDIRS)
CDEFS = $(CFG_CDEFS)
ADEFS = $(CFG_ADEFS)
CPPDEFS = $(CFG_CPPDEFS)
LIBS = -lm $(CFG_LIBS)
CUSTOM_LD_SCRIPT = $(CFG_LD_SCRIPT)

###### TOOLCHAIN ######
PREFIX=avr-
CC  = $(PREFIX)gcc
CXX = $(PREFIX)g++
AS  = $(PREFIX)g++ -x assembler-with-cpp
CP  = $(PREFIX)objcopy
OBJDUMP  = $(PREFIX)objdump
NM  = $(PREFIX)nm
SZ  = $(PREFIX)size

###### FLASHING ######
AVRDUDE = AVRDUDE
PROGRAMMER = usbasp
SERIALUPDIPORT = $(ENV_SERIALUPDIPORT)

###########################################################
# Option building
CFLAGS =  -funsigned-char \
          -funsigned-bitfields \
          -fpack-struct \
          -fshort-enums \
          -std=gnu23 \
          -Wall \
		  -Wno-array-bounds \
          -mstrict-X \
		  -mrelax \
          -mmcu=$(MCU) \
          -DF_CPU=$(CPU_CLOCK)UL \
          $(CDEFS) \
          $(INCDIRS) \
          $(OPTIM) \
          $(DEBUG) \
          -MMD -MP -MF"$(@:%.o=%.d)" \
          -Wa,-adhlns=$(@:%.o=%.lst) \
          -fdata-sections \
          -ffunction-sections\
          $(CFG_COPTS)
CPPFLAGS= -fpermissive \
          -funsigned-char \
          -funsigned-bitfields \
          -fpack-struct \
          -fshort-enums \
          -std=gnu++23 \
          -Wall \
		  -Wno-array-bounds \
          -mstrict-X \
		  -mrelax \
          -mmcu=$(MCU) \
          -DF_CPU=$(CPU_CLOCK)UL \
          $(CPPDEFS) \
          $(INCDIRS) \
          $(OPTIM) \
          $(DEBUG) \
          -MMD -MP -MF"$(@:%.o=%.d)" \
          -Wa,-adhlns=$(@:%.o=%.lst) \
          -fdata-sections \
          -ffunction-sections \
          $(CFG_CPPOPTS)
ASFLAGS = -mmcu=$(MCU) \
          -DF_CPU=$(CPU_CLOCK) \
          $(ADEFS) \
          $(INCDIRS) \
          $(OPTIM) \
          $(DEBUG) \
          -Wall \
          -MMD -MP -MF"$(@:%.o=%.d)" \
          -Wa,-adlns=$(@:%.o=%.lst) \
          -fdata-sections \
          -ffunction-sections \
          $(CFG_AOPTS)
LDFLAGS = -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref \
		  -mrelax \
          -mmcu=$(MCU) \
          $(LIBDIRS) \
          $(LIBS) \
          -Wl,--gc-sections \
          $(CUSTOM_LD_SCRIPT) \
          $(CFG_LDOPTS)
ifeq ($(PRINTF_SPEC), float)
LDFLAGS += -Wl,-u,vfprintf -lprintf_flt
$(info Using printf with float support)
else ifeq ($(PRINTF_SPEC), min)
LDFLAGS += -Wl,-u,vfprintf -lprintf_min
$(info Using minimal printf)
else
$(info Using default printf with no float support)
endif
ifeq ($(SCANF_SPEC), float)
LDFLAGS += -Wl,-u,vfscanf -lscanf_flt
$(info Using scanf with float support)
else ifeq ($(SCANF_SPEC), min)
LDFLAGS += -Wl,-u,vfscanf -lscanf_min
$(info Using minimal scanf)
else
$(info Using default scanf with no float support)
endif

$(info ------------------- BUILD -------------------)

C_SOURCES := $(foreach dir,$(SRCDIRS),$(shell (cd $(dir) && find . -name '*.c' -print)))
C_OBJECTS := $(addprefix $(BUILD_DIR)/objects/,$(C_SOURCES:.c=.c.o))
vpath %.c $(SRCDIRS)

CPP_SOURCES := $(foreach dir,$(SRCDIRS),$(shell (cd $(dir) && find . -name '*.cpp' -print)))
CPP_OBJECTS := $(addprefix $(BUILD_DIR)/objects/,$(CPP_SOURCES:.cpp=.cpp.o))
vpath %.cpp $(SRCDIRS)

ASM_SOURCES := $(foreach dir,$(SRCDIRS),$(shell (cd $(dir) && find . -name '*.S' -print)))
ASM_OBJECTS := $(addprefix $(BUILD_DIR)/objects/,$(ASM_SOURCES:.S=.s.o))
vpath %.S $(SRCDIRS)

OBJECTS := $(C_OBJECTS) $(CPP_OBJECTS) $(ASM_OBJECTS)

AVRDUDEFLAGS:= \
    -p $(MCU) \
    -c $(PROGRAMMER)
ifeq ($(PROGRAMMER), serialupdi)
AVRDUDEFLAGS += -P $(SERIALUPDIPORT)
endif

#################################################################
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).eep $(BUILD_DIR)/$(TARGET).usr $(BUILD_DIR)/$(TARGET).btr $(BUILD_DIR)/$(TARGET).fus $(BUILD_DIR)/$(TARGET).lss $(BUILD_DIR)/$(TARGET).sym size

$(C_OBJECTS): $(BUILD_DIR)/objects/%.c.o: %.c Makefile config.mk | $(BUILD_DIR)
	@mkdir --parents $(@D)
	$(CXX) -c $(CPPFLAGS) $< -o $@

$(CPP_OBJECTS): $(BUILD_DIR)/objects/%.cpp.o: %.cpp Makefile config.mk | $(BUILD_DIR)
	@mkdir --parents $(@D)
	$(CXX) -c $(CPPFLAGS) $< -o $@

$(ASM_OBJECTS): $(BUILD_DIR)/objects/%.s.o: %.S Makefile config.mk | $(BUILD_DIR)
	@mkdir --parents $(@D)
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile config.mk
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@$(CP) -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .userrow -R .bootrow $< $@

$(BUILD_DIR)/%.eep: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@$(CP) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--change-section-lma .eeprom=0 --no-change-warnings -O ihex $< $@

$(BUILD_DIR)/%.fus: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@$(CP) -j .fuse --set-section-flags=.fuse="alloc,load" \
	--change-section-lma .fuse=0 --no-change-warnings -O ihex $< $@

$(BUILD_DIR)/%.usr: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@$(CP) -j .userrow --set-section-flags=.userrow="alloc,load" \
	--change-section-lma .userrow=0 --no-change-warnings -O ihex $< $@

$(BUILD_DIR)/%.btr: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@$(CP) -j .bootrow --set-section-flags=.bootrow="alloc,load" \
	--change-section-lma .bootrow=0 --no-change-warnings -O ihex $< $@

$(BUILD_DIR)/%.lss: $(BUILD_DIR)/%.elf
	@$(OBJDUMP) -h -S -z $< > $@

$(BUILD_DIR)/%.sym: $(BUILD_DIR)/%.elf
	@$(NM) -n $< > $@

$(BUILD_DIR):
	@mkdir $@

.PHONY: clean all flash eeprom size fuse

size: $(BUILD_DIR)/$(TARGET).elf
	@$(SZ) $< -A > $(BUILD_DIR)/sz.txt
	@$(NM) $< > $(BUILD_DIR)/nm.txt
	@cat $(BUILD_DIR)/sz.txt $(BUILD_DIR)/nm.txt | awk '\
		/\\.data/{datasize=int($$2);} \
		/\\.rodata/{rodatasize=int($$2); rooffset=int($$3);} \
		/\\.text/{textsize=int($$2);}\
		/\\.bss/{bsssize=int($$2);}\
		/\\.noinit/{noinitsize=int($$2);}\
		/\\.eeprom/{eepromsize=int($$2);}\
		/\\.userrow/{userrowsize=int($$2);}\
		/\\.bootrow/{bootrowsize=int($$2);}\
		/__TEXT_REGION_LENGTH__/{devromsize=strtonum("0x" $$1);}\
		/__DATA_REGION_LENGTH__/{devramsize=strtonum("0x" $$1);}\
		END{ \
			printf("\n-------------------------------Sections--------------------------------\n"); \
			printf("  .text  .rodata    .data     .bss  .noinit  .eeprom .userrow .bootrow\n"); \
			printf("%7d  %7d  %7d  %7d  %7d  %7d  %7d  %7d\n", textsize, rodatasize, datasize, bsssize, noinitsize, eepromsize, userrowsize, bootrowsize); \
			printf("---------Flash--------- --------SRAM-------- -EEPROM- -USERRW- -BOOTRW-\n"); \
			flashsize = textsize+rodatasize+datasize; \
			ramsize = datasize+bsssize+noinitsize; \
			printf("  %7d / 0x%05x     %7d / 0x%04x       0x%03x    0x%03x    0x%03x\n\n", flashsize, flashsize, ramsize, ramsize, eepromsize, userrowsize, bootrowsize); \
			BARLEN=50; \
			for(i=0; i<int(flashsize*BARLEN/devromsize); i++) rombar=rombar "="; \
			for(i=0; i<int(ramsize*BARLEN/devramsize); i++) rambar=rambar "="; \
			if(devramsize%1024==0) ramsizestr=sprintf("%3d kiB", devramsize/1024); else ramsizestr=sprintf("%3d B", devramsize); \
			printf("Flash %5.1f%% [%-*s] %3d kiB\n", flashsize*100/devromsize, BARLEN, rombar, devromsize/1024); \
			printf("SRAM  %5.1f%% [%-*s] %s\n\n", ramsize*100/devramsize, BARLEN, rambar, ramsizestr); \
		}'
	@$(OBJDUMP) -h -w $< | grep "\\.rodata" | awk '{offset=strtonum("0x" $$5); block=int(offset/0x8000); printf(".rodata LMA: 0x%05x (FLMAP SECTION%d + 0x%04x)\n\n",offset, block, offset-block*0x8000)}'
	@rm $(BUILD_DIR)/sz.txt $(BUILD_DIR)/nm.txt

clean:
	-rm -fR $(BUILD_DIR)

flash: all
	$(AVRDUDE) $(AVRDUDEFLAGS) -U flash:w:"$(BUILD_DIR)/$(TARGET).hex"

eeprom: all
	$(AVRDUDE) $(AVRDUDEFLAGS) -U eeprom:w:"$(BUILD_DIR)/$(TARGET).eep"

userrow: all
	$(AVRDUDE) $(AVRDUDEFLAGS) -U userrow:w:"$(BUILD_DIR)/$(TARGET).usr"

bootrow: all
	$(AVRDUDE) $(AVRDUDEFLAGS) -U bootrow:w:"$(BUILD_DIR)/$(TARGET).btr"

# only available for AVRxt targets
fuse: all
	$(AVRDUDE) $(AVRDUDEFLAGS) -U fuses:w:"$(BUILD_DIR)/$(TARGET).fus"

-include $(shell (find $(BUILD_DIR)/objects/ -name '*.d' -print))