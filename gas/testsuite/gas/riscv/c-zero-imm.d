#as:
#objdump: -dr

.*:[ 	]+file format .*


Disassembly of section .text:

0+000 <.text>:
[ 	]+0:[ 	]+4501[ 	]+c.li[ 	]+a0,0
.*R_RISCV_RELAX_ENTRY.*
[ 	]+2:[ 	]+4581[ 	]+c.li[ 	]+a1,0
[ 	]+4:[ 	]+8a01[ 	]+c.andi[ 	]+a2,0
[ 	]+6:[ 	]+8a81[ 	]+c.andi[ 	]+a3,0
[ 	]+8:[ 	]+0001[ 	]+c.nop
[ 	]+a:[ 	]+00070713[ 	]+mv[ 	]+a4,a4
[ 	]+e:[ 	]+0781[ 	]+c.addi[ 	]+a5,0
[ 	]+10:[ 	]+00051513[ 	]+slli[ 	]+a0,a0,0x0
[ 	]+14:[ 	]+0005d593[ 	]+srli[ 	]+a1,a1,0x0
[ 	]+18:[ 	]+40065613[ 	]+srai[ 	]+a2,a2,0x0
[ 	]+1c:[ 	]+0682[ 	]+c.slli64[ 	]+a3
[ 	]+1e:[ 	]+8301[ 	]+c.srli64[ 	]+a4
[ 	]+20:[ 	]+8781[ 	]+c.srai64[ 	]+a5
#...
