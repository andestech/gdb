. ${srcdir}/emulparams/elf32lriscv-defs.sh
ELFSIZE=64

SDATA_START_SYMBOLS=". = ALIGN ($ELFSIZE / 8);
    ${CREATE_SHLIB-__SDATA_BEGIN__ = .;}
    ${CREATE_SHLIB-__global_pointer$ = __SDATA_BEGIN__ + 0x800;}
    *(.srodata.cst16) *(.srodata.cst8) *(.srodata.cst4) *(.srodata.cst2) *(.srodata .srodata.*)"
