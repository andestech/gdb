. ${srcdir}/emulparams/elf32lriscv-defs.sh
ELFSIZE=64

SDATA_START_SYMBOLS=". = ALIGN ($ELFSIZE / 8);
    __global_pointer$ = . + 0x800;"
SDATA_START_SYMBOLS="${CREATE_SHLIB-${SDATA_START_SYMBOLS}}
    *(.srodata.cst16) *(.srodata.cst8) *(.srodata.cst4) *(.srodata.cst2) *(.srodata .srodata.*)"
