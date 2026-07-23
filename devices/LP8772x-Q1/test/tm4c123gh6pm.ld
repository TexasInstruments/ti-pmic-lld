/******************************************************************************
 * tm4c123gh6pm.ld - Linker script for TM4C123GH6PM with PMIC-LLD tests
 *
 * Memory layout for TM4C123GH6PM:
 * - Flash: 256KB at 0x00000000
 * - SRAM:  32KB at 0x20000000
 *****************************************************************************/

MEMORY
{
    FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 256K
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 32K
}

ENTRY(ResetISR)

SECTIONS
{
    /* Vector table and code */
    .text :
    {
        _text_start = .;
        KEEP(*(.isr_vector))    /* Interrupt vector table */
        *(.text*)               /* Code */
        *(.rodata*)             /* Read-only data */
        *(.glue_7)
        *(.glue_7t)
        *(.eh_frame)

        /* C++ constructors/destructors */
        . = ALIGN(4);
        KEEP(*(.init))
        KEEP(*(.fini))

        . = ALIGN(4);
        __preinit_array_start = .;
        KEEP (*(.preinit_array*))
        __preinit_array_end = .;

        . = ALIGN(4);
        __init_array_start = .;
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array*))
        __init_array_end = .;

        . = ALIGN(4);
        __fini_array_start = .;
        KEEP (*(SORT(.fini_array.*)))
        KEEP (*(.fini_array*))
        __fini_array_end = .;

        . = ALIGN(4);
        _text_end = .;
    } > FLASH

    /* ARM exception index */
    .ARM.exidx :
    {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } > FLASH

    /* Store the load address for .data section */
    _data_load_start = .;

    /* Initialized data (copied from FLASH to SRAM at startup) */
    .data : AT(_data_load_start)
    {
        _data_start = .;
        *(vtable)
        *(.data*)
        . = ALIGN(4);
        _data_end = .;
    } > SRAM

    /* Uninitialized data (zero-initialized at startup) */
    .bss :
    {
        _bss_start = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _bss_end = .;
    } > SRAM

    /* Heap (used by malloc/sbrk) */
    .heap :
    {
        . = ALIGN(4);
        _heap_start = .;
        _end = .;            /* Symbol required by newlib's sbrk */
        . = . + 6K;          /* Reserve 6K for heap (increased from 4K, max available ~12.5K total) */
        . = ALIGN(4);
        _heap_end = .;
    } > SRAM

    /* Stack (grows downward from end of SRAM) */
    .stack :
    {
        . = ALIGN(8);
        _stack_start = .;
        . = . + 6K;          /* Reserve 6K for stack (increased from 4K, max available ~12.5K total) */
        . = ALIGN(8);
        _stack_end = .;
    } > SRAM

    /* Symbols for startup code */
    _etext = _text_end;
    _data = _data_start;
    _edata = _data_end;
    _ldata = LOADADDR(.data);
    _bss = _bss_start;
    _ebss = _bss_end;
}
