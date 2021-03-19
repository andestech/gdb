#as: -march=rv32ic
#objdump: -d

.*:[ 	]+file format .*


Disassembly of section .text:

0+000 <target>:
[ 	]+0:[ 	]+4108[ 	]+c.lw[ 	]+a0,0\(a0\)
[ 	]+2:[ 	]+4108[ 	]+c.lw[ 	]+a0,0\(a0\)
[ 	]+4:[ 	]+c108[ 	]+c.sw[ 	]+a0,0\(a0\)
[ 	]+6:[ 	]+c108[ 	]+c.sw[ 	]+a0,0\(a0\)
[ 	]+8:[ 	]+4502[ 	]+c.lwsp[ 	]+a0,0\(sp\)
[ 	]+a:[ 	]+4502[ 	]+c.lwsp[ 	]+a0,0\(sp\)
[ 	]+c:[ 	]+c02a[ 	]+c.swsp[ 	]+a0,0\(sp\)
[ 	]+e:[ 	]+c02a[ 	]+c.swsp[ 	]+a0,0\(sp\)
