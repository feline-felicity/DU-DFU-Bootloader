#pragma once
#include <avr/io.h>

#ifdef USERROW

// Memory-mapped USERROW variable.
#  define USERROW_VAR __attribute__((__section__(".userrow"), __used__))

// A do-nothing function just to enclose extended asm blocks, which is a hack to convert a #define to a symbol.
__attribute__((__used__)) static inline void _userrow_symdefs(void) {
    asm volatile(".global _NVMALLOC_USERROW_VMA_\n_NVMALLOC_USERROW_VMA_=%0"::"n"(&USERROW));
    asm volatile(".global _NVMALLOC_USERROW_LENGTH_\n_NVMALLOC_USERROW_LENGTH_=%0"::"n"(sizeof(USERROW)));
}

#endif

#ifdef BOOTROW

// Memory-mapped BOOTROW variable
#  define BOOTROW_VAR __attribute__((__section__(".bootrow"), __used__))
__attribute__((__used__)) static inline void _bootrow_symdefs(void) {
    asm volatile(".global _NVMALLOC_BOOTROW_VMA_\n_NVMALLOC_BOOTROW_VMA_=%0"::"n"(&BOOTROW));
    asm volatile(".global _NVMALLOC_BOOTROW_LENGTH_\n_NVMALLOC_BOOTROW_LENGTH_=%0"::"n"(sizeof(BOOTROW)));
}

#endif

#ifdef EEPROM_START
// Memory-mapped EEPROM variable
#   define EEPROM_VAR __attribute__((__section__(".eeprom"), __used__))
__attribute__((__used__)) static inline void _eeprom_symdefs(void) {
    asm volatile(".global _NVMALLOC_EEPROM_VMA_\n_NVMALLOC_EEPROM_VMA_=%0"::"n"(EEPROM_START));
    asm volatile(".global _NVMALLOC_EEPROM_LENGTH_\n_NVMALLOC_EEPROM_LENGTH_=%0"::"n"(EEPROM_SIZE));
}
#endif
