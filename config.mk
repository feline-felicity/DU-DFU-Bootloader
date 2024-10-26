########## OPTIONS ##########

# Uncomment if F_CPU is not equal to the max. speed of internal RC
# CLOCK_OVERRIDE := 16000000

# Choose from (default|float|min). No leading/trailing whitespaces allowed
PRINTF_SPEC:=default
SCANF_SPEC:=default

# Optimization. `-Os` will usually do.
CFG_OPTIM:= -Os

# Custom linker script.
# Use `-Tlink/nvm.ld` to enable memory-mapped special NVM variables (by nvmalloc.h).
# Note this is incompatible with AVR-LibC's EEPROM functions.
CFG_LD_SCRIPT:= -Tlink/nvm.ld -Tlink/novector.ld

# Defines for each of three languages. These are simply passed as options.
# Prefix with `-D` or `-U` like `-DMY_DEFINE=1` or `-UMY_UNDEFINE`.
CFG_CDEFS:=
CFG_CPPDEFS:=
CFG_ADEFS:=

# Any other options to be passed to gcc in each step
CFG_COPTS:=
CFG_CPPOPTS:=
CFG_AOPTS:=
CFG_LDOPTS:=

########## SEARCH PATHS ##########

# Source files are searched recursively by extension (.c/.cpp/.S)
# ./src will be automatically added, so usually nothing has to be specified here
CFG_SRCDIRS:=

# Additional include search directories shared by all three languages.
# Unlike source files, no fancy manipulation is applied on this.
# Prefix all directories with `-I` as options
CFG_INCDIRS:=

# Additional library search directories for linker
# Usually not needed. Prefix with `-L` as options
CFG_LIBDIRS:=

# Additional libraries to be linked
# Prefix with `-l`. `-lm` will be automatically added
CFG_LIBS:=