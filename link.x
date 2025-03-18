/* NOTE 1 K = 1 KiBi = 1024 bytes */
RAM_START = 0x20000000;
RAM_SIZE = 64K;

MEMORY
{
  FLASH  (rx)   : ORIGIN = 0x00008020, LENGTH = 256K
  RAM    (!rx)  : ORIGIN = RAM_START, LENGTH = RAM_SIZE
}

SECTIONS
{
    __code_start__ = .;
    .text 0x000 :  { *(.text.start) *(.text*) }
    __code_end__ = .;
    __data_start__ = .;
    .rodata :       { *(.rodata*) }
    .data :         { *(.data*) }
    __data_end__ = .;
    # https://blog.thea.codes/the-most-thoroughly-commented-linker-script/
    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        __bss_start__ = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
    } >RAM
}

/* Force link of _start and verify correct position */
ENTRY(_start)
ASSERT(_start == ADDR(.text), "_start symbol must be placed first in text section")
