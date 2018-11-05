#as: -march=rv32if
#objdump: -dr

.*:[ 	]+file format .*


Disassembly of section .text:

0+000 <.text>:
[ 	]+0:[ 	]+002fd573[ 	]+fsrmi[ 	]+a0,31
.*R_RISCV_RELAX_ENTRY.*
.*R_RISCV_NO_RVC_REGION_BEGIN.*
.*R_RISCV_RELAX_REGION_BEGIN.*
[ 	]+4:[ 	]+002f5073[ 	]+fsrmi[ 	]+zero,30
[ 	]+8:[ 	]+001ed773[ 	]+fsflagsi[ 	]+a4,29
[ 	]+c:[ 	]+001e5073[ 	]+fsflagsi[ 	]+zero,28
