#as: -mno-relax
#source: mapping-04.s
#objdump: -d

.*:[ 	]+file format .*


Disassembly of section .text:

0+000 <.text>:
[ 	]+0:[ 	]+00001001[ 	]+.word[ 	]+0x00001001
[ 	]+4:[ 	]+00001001[ 	]+.word[ 	]+0x00001001
[ 	]+8:[ 	]+00000001[ 	]+.word[ 	]+0x00000001
[ 	]+c:[ 	]+00[ 	]+.byte[ 	]+0x00
[ 	]+d:[ 	]+00[ 	]+.byte[ 	]+0x00
[ 	]+e:[ 	]+0001[ 	]+.2byte[ 	]+0x1
[ 	]+10:[ 	]+00a50533[ 	]+add[ 	]+a0,a0,a0
[ 	]+14:[ 	]+20022002[ 	]+.word[ 	]+0x20022002
[ 	]+18:[ 	]+20022002[ 	]+.word[ 	]+0x20022002
[ 	]+1c:[ 	]+2002[ 	]+.short[ 	]+0x2002
[ 	]+1e:[ 	]+00b585b3[ 	]+add[ 	]+a1,a1,a1
[ 	]+22:[ 	]+0001[ 	]+.2byte[ 	]+0x1
