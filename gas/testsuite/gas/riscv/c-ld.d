#as: -march=rv64ic
#objdump: -dr

.*:[ 	]+file format .*


Disassembly of section .text:

0+000 <target>:
[ 	]+0:[ 	]+6108[ 	]+c.ld[ 	]+a0,0\(a0\)
.*R_RISCV_RELAX_ENTRY.*
[ 	]+2:[ 	]+6108[ 	]+c.ld[ 	]+a0,0\(a0\)
[ 	]+4:[ 	]+e108[ 	]+c.sd[ 	]+a0,0\(a0\)
[ 	]+6:[ 	]+e108[ 	]+c.sd[ 	]+a0,0\(a0\)
[ 	]+8:[ 	]+6502[ 	]+c.ldsp[ 	]+a0,0\(sp\)
[ 	]+a:[ 	]+6502[ 	]+c.ldsp[ 	]+a0,0\(sp\)
[ 	]+c:[ 	]+e02a[ 	]+c.sdsp[ 	]+a0,0\(sp\)
[ 	]+e:[ 	]+e02a[ 	]+c.sdsp[ 	]+a0,0\(sp\)
