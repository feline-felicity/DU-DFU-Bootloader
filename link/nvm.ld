/* Fall back to 0x880000 if VMA (data memory-mapped address) is not provided */
ASSERT(DEFINED(_NVMALLOC_USERROW_VMA_) || DEFINED(_NVMALLOC_BOOTROW_VMA_) || DEFINED(_NVMALLOC_EEPROM_LENGTH_), "nvm.ld requires nvmalloc.h included at least once.");
_NVMALLOC_USERROW_VMA_ = DEFINED(_NVMALLOC_USERROW_VMA_) ? _NVMALLOC_USERROW_VMA_ : 0x880000;
_NVMALLOC_USERROW_LENGTH_ = DEFINED(_NVMALLOC_USERROW_LENGTH_) ? _NVMALLOC_USERROW_LENGTH_ : 32;
_NVMALLOC_BOOTROW_VMA_ = DEFINED(_NVMALLOC_BOOTROW_VMA_) ? _NVMALLOC_BOOTROW_VMA_ : 0x890000;
_NVMALLOC_BOOTROW_LENGTH_ = DEFINED(_NVMALLOC_BOOTROW_LENGTH_) ? _NVMALLOC_BOOTROW_LENGTH_ : 256;
_NVMALLOC_EEPROM_VMA_ = DEFINED(_NVMALLOC_EEPROM_VMA_) ? _NVMALLOC_EEPROM_VMA_ : 0x890000;
_NVMALLOC_EEPROM_LENGTH_ = DEFINED(_NVMALLOC_EEPROM_LENGTH_) ? _NVMALLOC_EEPROM_LENGTH_ : 256;
__DATA_SPACE_HEAD__ = 0x800000;

SECTIONS{
    /*
    Default linker script has 'user_signatures' at 0x850000,
    but we use our own nonstandard definition at 0x880000.
    After all, LMAs don't matter since we change it when dumping hex.
    */
    .userrow (_NVMALLOC_USERROW_VMA_ + __DATA_SPACE_HEAD__): AT (0x880000){
        KEEP(*(.userrow*))
        ASSERT(. <= _NVMALLOC_USERROW_LENGTH_, ".userrow too large.");
    }
    .bootrow (_NVMALLOC_BOOTROW_VMA_ + __DATA_SPACE_HEAD__): AT (0x890000){
        KEEP(*(.bootrow*))
        ASSERT(. <= _NVMALLOC_BOOTROW_LENGTH_, ".bootrow too large.");
    }
    /*
    We take over .eeprom section from default script.
    This makes our approach incompatible with AVR-LibC EEPROM operations,
    which assume LMA==VMA==0x81____, to C codes appearing as 0x____.
    */
    .eeprom (_NVMALLOC_EEPROM_VMA_ + __DATA_SPACE_HEAD__): AT (0x810000){
        KEEP(*(.eeprom*))
        ASSERT(. <= _NVMALLOC_EEPROM_LENGTH_, ".eeprom too large.");
    }
}
INSERT BEFORE .eeprom