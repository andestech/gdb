#as: -march=rv32if
#objdump: -dr

.*:[ 	]+file format .*


Disassembly of section .text:

0+000 <.text>:
[ 	]+0:[ 	]+e00b8653[ 	]+fmv.x.w[ 	]+a2,fs7
.*R_RISCV_RELAX_ENTRY.*
.*R_RISCV_NO_RVC_REGION_BEGIN.*
.*R_RISCV_RELAX_REGION_BEGIN.*
[ 	]+4:[ 	]+e00b8653[ 	]+fmv.x.w[ 	]+a2,fs7
[ 	]+8:[ 	]+f00800d3[ 	]+fmv.w.x[ 	]+ft1,a6
[ 	]+c:[ 	]+f00800d3[ 	]+fmv.w.x[ 	]+ft1,a6
