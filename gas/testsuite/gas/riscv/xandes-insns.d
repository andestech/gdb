#as: -march=rv64ifcv_xandes
#objdump: -d

.*:[ 	]+file format .*


Disassembly of section .text:

0+000 <.*>:
[ 	]+0:[ 	]+4026f45b[ 	]+bbs[ 	]+a3,2,0x8 <.*>
			0: R_RISCV_RELAX_ENTRY	\*ABS\*
			0: R_RISCV_10_PCREL	\.L0 
[ 	]+4:[ 	]+0000006f[ 	]+j[ 	]+0x4 <.*>
			4: R_RISCV_JAL	\*ABS\*\+0x1
[ 	]+8:[ 	]+4002f45b[ 	]+bbs[ 	]+t0,0,0x10 <.*>
			8: R_RISCV_10_PCREL	\.L0 
[ 	]+c:[ 	]+0000006f[ 	]+j[ 	]+0xc <.*>
			c: R_RISCV_JAL	\*ABS\*\+0x6
[ 	]+10:[ 	]+001f745b[ 	]+bbc[ 	]+t5,1,0x18 <.*>
			10: R_RISCV_10_PCREL	\.L0 
[ 	]+14:[ 	]+0000006f[ 	]+j[ 	]+0x14 <.*>
			14: R_RISCV_JAL	\*ABS\*\+0x7
[ 	]+18:[ 	]+0038f45b[ 	]+bbc[ 	]+a7,3,0x20 <.*>
			18: R_RISCV_10_PCREL	\.L0 
[ 	]+1c:[ 	]+0000006f[ 	]+j[ 	]+0x1c <.*>
			1c: R_RISCV_JAL	\*ABS\*\+0x3
[ 	]+20:[ 	]+004e645b[ 	]+bnec[ 	]+t3,4,0x28 <.*>
			20: R_RISCV_10_PCREL	\.L0 
[ 	]+24:[ 	]+0000006f[ 	]+j[ 	]+0x24 <.*>
			24: R_RISCV_JAL	\*ABS\*\+0x1
[ 	]+28:[ 	]+003c545b[ 	]+beqc[ 	]+s8,3,0x30 <.*>
			28: R_RISCV_10_PCREL	\.L0 
[ 	]+2c:[ 	]+0000006f[ 	]+j[ 	]+0x2c <.*>
			2c: R_RISCV_JAL	\*ABS\*\+0x3
[ 	]+30:[ 	]+1455375b[ 	]+bfos[ 	]+a4,a0,5,5
[ 	]+34:[ 	]+184f215b[ 	]+bfoz[ 	]+sp,t5,6,4
[ 	]+38:[ 	]+0ae503db[ 	]+lea.h[ 	]+t2,a0,a4
[ 	]+3c:[ 	]+0cb503db[ 	]+lea.w[ 	]+t2,a0,a1
[ 	]+40:[ 	]+0ec90cdb[ 	]+lea.d[ 	]+s9,s2,a2
[ 	]+44:[ 	]+11a18e5b[ 	]+lea.b.ze[ 	]+t3,gp,s10
[ 	]+48:[ 	]+1371885b[ 	]+lea.h.ze[ 	]+a6,gp,s7
[ 	]+4c:[ 	]+15ab8f5b[ 	]+lea.w.ze[ 	]+t5,s7,s10
[ 	]+50:[ 	]+16ad0ddb[ 	]+lea.d.ze[ 	]+s11,s10,a0
[ 	]+54:[ 	]+00001d8b[ 	]+addigp[ 	]+s11,0
[ 	]+58:[ 	]+03000d0b[ 	]+lbgp[ 	]+s10,48
[ 	]+5c:[ 	]+05002a8b[ 	]+lbugp[ 	]+s5,80
[ 	]+60:[ 	]+008010ab[ 	]+lhgp[ 	]+ra,8
[ 	]+64:[ 	]+02805c2b[ 	]+lhugp[ 	]+s8,40
[ 	]+68:[ 	]+01802bab[ 	]+lwgp[ 	]+s7,24
[ 	]+6c:[ 	]+04806e2b[ 	]+lwugp[ 	]+t3,72
[ 	]+70:[ 	]+078034ab[ 	]+ldgp[ 	]+s1,120
[ 	]+74:[ 	]+0100300b[ 	]+sbgp[ 	]+a6,0
[ 	]+78:[ 	]+0080002b[ 	]+shgp[ 	]+s0,0
[ 	]+7c:[ 	]+05a0402b[ 	]+swgp[ 	]+s10,64
[ 	]+80:[ 	]+0460742b[ 	]+sdgp[ 	]+t1,72
[ 	]+84:[ 	]+21238a5b[ 	]+ffb[ 	]+s4,t2,s2
[ 	]+88:[ 	]+225d81db[ 	]+ffzmism[ 	]+gp,s11,t0
[ 	]+8c:[ 	]+24a98bdb[ 	]+ffmism[ 	]+s7,s3,a0
[ 	]+90:[ 	]+279e045b[ 	]+flmism[ 	]+s0,t3,s9
[ 	]+94:[ 	]+8034[ 	]+exec.it[ 	]+#25[ 	]+!.*
[ 	]+96:[ 	]+8c10[ 	]+exec.it[ 	]+#7[ 	]+!.*
