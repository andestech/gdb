#as: -march=rv32ifv_xv5-1p0
#objdump: -d

.*:[ 	]+file format .*


Disassembly of section .text:

0+000 <.text>:
[ 	]+[0-9a-f]+:[ 	]+002040db[ 	]+vfwcvt.s.bf16[ 	]+v1,v2
[ 	]+[0-9a-f]+:[ 	]+0040c1db[ 	]+vfncvt.bf16.s[ 	]+v3,v4
[ 	]+[0-9a-f]+:[ 	]+006142db[ 	]+fcvt.s.bf16[  	]+ft5,ft6
[ 	]+[0-9a-f]+:[ 	]+0081c3db[ 	]+fcvt.bf16.s[  	]+ft7,fs0
