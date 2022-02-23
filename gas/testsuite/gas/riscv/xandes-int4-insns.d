#as: -march=rv32ifv_xandes
#objdump: -d

.*:[ 	]+file format .*


Disassembly of section .text:

0+000 <.text>:
[ 	]+[0-9a-f]+:[ 	]+0605c0db[ 	]+vle4.v[ 	]+v1,\(a1\)
[ 	]+[0-9a-f]+:[ 	]+0616415b[ 	]+vlnu.v[ 	]+v2,\(a2\)
[ 	]+[0-9a-f]+:[ 	]+0626c1db[ 	]+vln8.v[  	]+v3,\(a3\)
[ 	]+[0-9a-f]+:[ 	]+0637425b[ 	]+vlnu8.v[ 	]+v4,\(a4\)
[ 	]+[0-9a-f]+:[ 	]+0405c0db[ 	]+vln.v[ 	]+v1,\(a1\),v0.t
[ 	]+[0-9a-f]+:[ 	]+0416415b[ 	]+vlnu.v[ 	]+v2,\(a2\),v0.t
[ 	]+[0-9a-f]+:[ 	]+0426c1db[ 	]+vln8.v[  	]+v3,\(a3\),v0.t
[ 	]+[0-9a-f]+:[ 	]+0437425b[ 	]+vlnu8.v[ 	]+v4,\(a4\),v0.t
