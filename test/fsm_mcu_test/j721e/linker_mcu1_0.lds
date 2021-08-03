/* Linker Settings */
--retain="*(.bootCode)"
--retain="*(.startupCode)"
--retain="*(.startupData)"
--retain="*(.intvecs)"
--retain="*(.intc_text)"
--retain="*(.rstvectors)"
--retain="*(.irqStack)"
--retain="*(.fiqStack)"
--retain="*(.abortStack)"
--retain="*(.undStack)"
--retain="*(.svcStack)"
--fill_value=0
--stack_size=0x1000
--heap_size=0x1000
--entry_point=_resetvectors     /* Default C RTS boot.asm   */

-stack  0x1000                              /* SOFTWARE STACK SIZE           */
-heap   0x1000                              /* HEAP AREA SIZE                */

/* Stack Sizes for various modes */
__IRQ_STACK_SIZE = 0x500;
__FIQ_STACK_SIZE = 0x500;
__ABORT_STACK_SIZE = 0x500;
__UND_STACK_SIZE = 0x500;
__SVC_STACK_SIZE = 0x500;

/* Memory Map */
MEMORY
{
    /* MCU0_R5F_0 local view */
    MCU0_R5F_TCMA_SBL_RSVD (X)  : origin=0x0        length=0x100
    MCU0_R5F_TCMA (X)       : origin=0x100      length=0x8000 - 0x100
    MCU0_R5F_TCMB0 (RWIX)   : origin=0x41010000 length=0x8000

    /* MCU0_R5F_1 SoC view */
    MCU0_R5F1_ATCM (RWIX)   : origin=0x41400000 length=0x8000
    MCU0_R5F1_BTCM (RWIX)   : origin=0x41410000 length=0x8000

    /* Refer the user guide for details on persistence of these sections */
    OCMC_RAM_BOARD_CFG (RWIX)   : origin=0x41C80000 length=0x2000
    OCMC_RAM_SCISERVER (RWIX)   : origin=0x41C82000 length=0x60000
    VECTORS (X)                 : origin=0x41CE2000 length=0x1000
    RESET_VECTORS (X)           : origin=0x41CE3000 length=0x100
    OCMC_RAM (RWIX)             : origin=0x41CE3100 length=0x1CA00
    OCMC_RAM_X509_HEADER (RWIX) : origin=0x41CFFB00 length=0x500

    /* j721e MCMS3 locations */
    /* j721e Reserved Memory for ARM Trusted Firmware */
    MSMC3_ARM_FW   (RWIX)   : origin=0x70000000 length=0x40000         /* 256KB */
    MSMC3   (RWIX)          : origin=0x70040000 length=0x7B0000        /* 8MB - 320KB */
    /* j721e Reserved Memory for DMSC Firmware */
    MSMC3_DMSC_FW  (RWIX)   : origin=0x707F0000 length=0x10000         /* 64KB */

    DDR0    (RWIX)          : origin=0x80000000 length=0x80000000      /* 2GB */
}

/* Section Configuration */
SECTIONS
{
    /* 'intvecs' and 'intc_text' sections shall be placed within */
    /* a range of +\- 16 MB */
    .intvecs       : {} palign(8)      > VECTORS
    .intc_text     : {} palign(8)      > VECTORS
    .rstvectors    : {} palign(8)      > RESET_VECTORS
    .bootCode      : {} palign(8)      > OCMC_RAM
    .startupCode   : {} palign(8)      > OCMC_RAM
    .startupData   : {} palign(8)      > OCMC_RAM, type = NOINIT
    .text          : {} palign(8)      > OCMC_RAM_SCISERVER
    .const         : {} palign(8)      > OCMC_RAM
    .cinit         : {} palign(8)      > OCMC_RAM_SCISERVER
    .pinit         : {} palign(8)      > OCMC_RAM
    .bss           : {} align(4)       > OCMC_RAM
    .far           : {} align(4)       > OCMC_RAM
    .data          : {} palign(128)    > OCMC_RAM
    .sysmem        : {}                > OCMC_RAM
    .data_buffer   : {} palign(128)    > OCMC_RAM
    .bss.devgroup* : {} align(4)       > OCMC_RAM
    .const.devgroup*: {} align(4)      > OCMC_RAM_SCISERVER
    .boardcfg_data : {} align(4)       > OCMC_RAM_SCISERVER

    /* USB or any other LLD buffer for benchmarking */
    .benchmark_buffer (NOLOAD) {} ALIGN (8) > OCMC_RAM_SCISERVER

    .stack      : {} align(4)       > OCMC_RAM_SCISERVER  (HIGH)
    .irqStack   : {. = . + __IRQ_STACK_SIZE;} align(4)      > OCMC_RAM_SCISERVER  (HIGH)
    RUN_START(__IRQ_STACK_START)
    RUN_END(__IRQ_STACK_END)
    .fiqStack   : {. = . + __FIQ_STACK_SIZE;} align(4)      > OCMC_RAM_SCISERVER  (HIGH)
    RUN_START(__FIQ_STACK_START)
    RUN_END(__FIQ_STACK_END)
    .abortStack : {. = . + __ABORT_STACK_SIZE;} align(4)    > OCMC_RAM_SCISERVER  (HIGH)
    RUN_START(__ABORT_STACK_START)
    RUN_END(__ABORT_STACK_END)
    .undStack   : {. = . + __UND_STACK_SIZE;} align(4)      > OCMC_RAM_SCISERVER  (HIGH)
    RUN_START(__UND_STACK_START)
    RUN_END(__UND_STACK_END)
    .svcStack   : {. = . + __SVC_STACK_SIZE;} align(4)      > OCMC_RAM_SCISERVER  (HIGH)
    RUN_START(__SVC_STACK_START)
    RUN_END(__SVC_STACK_END)
}
