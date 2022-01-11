#as: -march=rv32ifv_xandes
#objdump: -d

.*:[ 	]+file format .*


Disassembly of section .text:

0+000 <.text>:
[ 	]+[0-9a-f]+:[ 	]+1020c05b[ 	]+vd4dots.vv[ 	]+v0,v1,v2,v0.t
[ 	]+[0-9a-f]+:[ 	]+1c20c05b[ 	]+vd4dotu.vv[ 	]+v0,v1,v2,v0.t
[ 	]+[0-9a-f]+:[ 	]+1420c05b[ 	]+vd4dotsu.vv[ 	]+v0,v1,v2,v0.t
[ 	]+[0-9a-f]+:[ 	]+1220c05b[ 	]+vd4dots.vv[ 	]+v0,v1,v2
[ 	]+[0-9a-f]+:[ 	]+1e20c05b[ 	]+vd4dotu.vv[ 	]+v0,v1,v2
[ 	]+[0-9a-f]+:[ 	]+1620c05b[ 	]+vd4dotsu.vv[ 	]+v0,v1,v2
