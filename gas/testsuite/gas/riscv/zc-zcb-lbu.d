#as: -march=rv32g_zca_zcb
#source: zc-zcb-lbu.s
#objdump: -dr -Mno-aliases

.*:[	 ]+file format .*


Disassembly of section .text:

0+000 <zcb_lbu>:
[	 ]*[0-9a-f]+:[	 ]+8120[	 ]+c.lbu[	 ]+s0,2\(a0\)
.*R_RISCV_RELAX_ENTRY.*
[	 ]*[0-9a-f]+:[	 ]+825c[	 ]+c.lbu[	 ]+a5,1\(a2\)
[	 ]*[0-9a-f]+:[	 ]+80e0[	 ]+c.lbu[	 ]+s0,3\(s1\)
[	 ]*[0-9a-f]+:[	 ]+8024[	 ]+c.lbu[	 ]+s1,2\(s0\)
[	 ]*[0-9a-f]+:[	 ]+8050[	 ]+c.lbu[	 ]+a2,1\(s0\)
[	 ]*[0-9a-f]+:[	 ]+83f4[	 ]+c.lbu[	 ]+a3,3\(a5\)
[	 ]*[0-9a-f]+:[	 ]+8398[	 ]+c.lbu[	 ]+a4,0\(a5\)
[	 ]*[0-9a-f]+:[	 ]+8044[	 ]+c.lbu[	 ]+s1,1\(s0\)
[	 ]*[0-9a-f]+:[	 ]+83bc[	 ]+c.lbu[  	 ]+a5,2\(a5\)
[	 ]*[0-9a-f]+:[	 ]+8060[	 ]+c.lbu[  	 ]+s0,3\(s0\)
[	 ]*[0-9a-f]+:[	 ]+80dc[	 ]+c.lbu[  	 ]+a5,1\(s1\)
[	 ]*[0-9a-f]+:[	 ]+823c[	 ]+c.lbu[  	 ]+a5,2\(a2\)
[	 ]*[0-9a-f]+:[	 ]+83d4[	 ]+c.lbu[  	 ]+a3,1\(a5\)
[	 ]*[0-9a-f]+:[	 ]+8398[	 ]+c.lbu[  	 ]+a4,0\(a5\)
[	 ]*[0-9a-f]+:[	 ]+8380[	 ]+c.lbu[  	 ]+s0,0\(a5\)
[	 ]*[0-9a-f]+:[	 ]+8004[	 ]+c.lbu[  	 ]+s1,0\(s0\)
