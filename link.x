/* NOTE 1 K = 1 KiBi = 1024 bytes */
RAM_START = 0x20000000;
RAM_SIZE = 64K;

/* See nRF52832 memory map. (page 25) */
MEMORY
{
  FLASH  (rx)   : ORIGIN = 0x00000000, LENGTH = 512K
  RAM    (!rx)  : ORIGIN = RAM_START, LENGTH = RAM_SIZE
}

ENTRY(Reset);

/* Give overrideable defaults for the various handlers. */
/* perhaps unnecessary */
/*
PROVIDE(NonMaskableInt = DefaultHandler);
PROVIDE(MemoryManagement = DefaultHandler);
PROVIDE(BusFault = DefaultHandler);
PROVIDE(UsageFault = DefaultHandler);
PROVIDE(SecureFault = DefaultHandler);
PROVIDE(SVCall = DefaultHandler);
PROVIDE(DebugMonitor = DefaultHandler);
PROVIDE(PendSV = DefaultHandler);
PROVIDE(SysTick = DefaultHandler);
*/


SECTIONS
{
    PROVIDE(__ram_start__ = ORIGIN(RAM));
    PROVIDE(__ram_end__ = ORIGIN(RAM) + LENGTH(RAM));
    PROVIDE(__stack_start__ = __ram_end__);

    /*
    * See https://github.com/rust-embedded/cortex-m/blob/898b11c7783246fe0137537d20898b91e58ff6aa/cortex-m-rt/link.x.in#L35C1-L50C33.
    * and Armv7-M B1-525 for the definition of the vector table.
    * Vector table must start at 0x0.
    */

    .vector_table ORIGIN(FLASH) :
    {
        __vector_table__ = .;

        /* The vector table starts with the initial main stack pointer. */
        /* Per Armv7-M B1-535, the stack pointer must be 4-byte aligned. */
        /* cortex-m-rt crate uses 8-byte aligned which might be good. */
        LONG(__stack_start__ & 0xFFFFFFF8);

        /* The second entry in the vector table must be the reset vector. */
        *(.vector_table.reset_vector);
        /* Other exceptions */
        __exceptions_start__ = .;
        *(.vector_table.exceptions);
        __exceptions_end__ = .;

        /* Device specific interrupts */
        *(.vector_table.interrupts);
    } > FLASH

    PROVIDE(__start_text__ = ADDR(.vector_table) + SIZEOF(.vector_table));

    .text __start_text__ :
    {
        /* We put the reset/entry function first. */
        *(.Reset);
        /* Then HardFault. */
        *(.HardFault.*);
        /* The rest of the code after. */
        *(.text .text.*);
    } > FLASH
    PROVIDE(__end_text__ = ADDR(.text) + SIZEOF(.text));

    /* read only data */
    /* According to https://github.com/rust-embedded/cortex-m/blob/master/cortex-m-rt/link.x.in needs alignment. */
    .rodata : ALIGN(4)
    {
        . = ALIGN(4);
        *(.rodata .rodata.*);
        . = ALIGN(4);
    } > FLASH

    /*
     * Data is separate because it needs to be stored in flash memory but then
     * copied into RAM to be modified.
     */
    .data : ALIGN(4)
    {
        . = ALIGN(4);
        __data_start__ = .;
        *(.data .data.*);
        . = ALIGN(4);
    } > RAM AT>FLASH
    /* https://blog.thea.codes/the-most-thoroughly-commented-linker-script/#:~:text=%7D%20%3E%20rom-,Relocate,-The%20.relocate%20section */
    /* https://github.com/rust-embedded/cortex-m/blob/898b11c7783246fe0137537d20898b91e58ff6aa/cortex-m-rt/link.x.in#L133 */
    . = ALIGN(4);
    __data_end__ = .;

    /* Expose the actual RAM address that data should be loaded at. */
    __la_data_start__ = LOADADDR(.data);

    /* TODO: verify "trustzone-m veneers" doesn't need to exist */

    .bss (NOLOAD) : ALIGN(4)
    {
        . = ALIGN(4);
        __bss_start__ = .;
        *(.bss .bss.*);
        . = ALIGN(4);
    } > RAM
    . = ALIGN(4);
    __bss_end__ = .;

    /* Place end of stack at end of allocated RAM. */
    PROVIDE(__stack_end__ = __bss_end__);

    /*
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
    */
}



/* Force link of _start and verify correct position */
/*
RESET is the entry point, no longer 0x8000
ENTRY(_start)
ASSERT(_start == ADDR(.text), "_start symbol must be placed first in text section")
*/
