#as: -march=rv32g_zcmb
#source: zc-zcmb-lb.s
#objdump: -dr -Mno-aliases

.*:[	 ]+file format .*


Disassembly of section .text:

0+000 <zcmb_lb>:
[	 ]*[0-9a-f]+:[	 ]+2120[	 ]+cm.lb[	 ]+s0,2\(a0\)
.*R_RISCV_RELAX_ENTRY.*
[	 ]*[0-9a-f]+:[	 ]+2a1c[	 ]+cm.lb[	 ]+a5,1\(a2\)
[	 ]*[0-9a-f]+:[	 ]+2080[	 ]+cm.lb[	 ]+s0,0\(s1\)
[	 ]*[0-9a-f]+:[	 ]+2044[	 ]+cm.lb[	 ]+s1,4\(s0\)
[	 ]*[0-9a-f]+:[	 ]+2870[	 ]+cm.lb[	 ]+a2,7\(s0\)
[	 ]*[0-9a-f]+:[	 ]+2794[	 ]+cm.lb[	 ]+a3,8\(a5\)
[	 ]*[0-9a-f]+:[	 ]+2ff8[	 ]+cm.lb[	 ]+a4,15\(a5\)
[	 ]*[0-9a-f]+:[	 ]+2024[	 ]+cm.lb[	 ]+s1,2\(s0\)
[	 ]*[0-9a-f]+:[	 ]+2b9c[	 ]+cm.lb[	 ]+a5,1\(a5\)
[	 ]*[0-9a-f]+:[	 ]+2000[	 ]+cm.lb[	 ]+s0,0\(s0\)
[	 ]*[0-9a-f]+:[	 ]+20dc[	 ]+cm.lb[	 ]+a5,4\(s1\)
[	 ]*[0-9a-f]+:[	 ]+2a7c[	 ]+cm.lb[	 ]+a5,7\(a2\)
[	 ]*[0-9a-f]+:[	 ]+2794[	 ]+cm.lb[	 ]+a3,8\(a5\)
[	 ]*[0-9a-f]+:[	 ]+2ff8[	 ]+cm.lb[	 ]+a4,15\(a5\)
[	 ]*[0-9a-f]+:[	 ]+2380[	 ]+cm.lb[	 ]+s0,0\(a5\)
[	 ]*[0-9a-f]+:[	 ]+2004[	 ]+cm.lb[	 ]+s1,0\(s0\)
