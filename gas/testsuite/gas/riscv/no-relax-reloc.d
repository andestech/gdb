#as: -march=rv32ic -mno-relax
#objdump: -r

.*:[ 	]+file format .*

RELOCATION RECORDS FOR .*
.*
.*R_RISCV_RELAX_ENTRY.*
0+0 R_RISCV_HI20.*
0+4 R_RISCV_LO12_I.*
0+8 R_RISCV_PCREL_HI20.*
0+c R_RISCV_PCREL_LO12_I.*
0+10 R_RISCV_CALL.*
