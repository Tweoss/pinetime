MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  FLASH  (rx)   : ORIGIN = 0x00008020, LENGTH = 256K
  RAM    (!rx)  : ORIGIN = 0x20000008, LENGTH = 32760
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
    __bss_start__ = .;
    .bss :          { *(.bss*)  *(COMMON) } >RAM
    __bss_end__ = ALIGN(8);
}

/* Force link of _start and verify correct position */
ENTRY(_start)
ASSERT(_start == ADDR(.text), "_start symbol must be placed first in text section")
