#as: -march=rv32ic
#objdump: -d -M_no-prefer

.*:[ 	]+file format .*


Disassembly of section .text:

0+000 <target>:
[ 	]+0:[ 	]+4108[ 	]+lw[ 	]+a0,0\(a0\)
[ 	]+2:[ 	]+4108[ 	]+lw[ 	]+a0,0\(a0\)
[ 	]+4:[ 	]+c108[ 	]+sw[ 	]+a0,0\(a0\)
[ 	]+6:[ 	]+c108[ 	]+sw[ 	]+a0,0\(a0\)
[ 	]+8:[ 	]+4502[ 	]+lw[ 	]+a0,0\(sp\)
[ 	]+a:[ 	]+4502[ 	]+lw[ 	]+a0,0\(sp\)
[ 	]+c:[ 	]+c02a[ 	]+sw[ 	]+a0,0\(sp\)
[ 	]+e:[ 	]+c02a[ 	]+sw[ 	]+a0,0\(sp\)
