#as: -march=rv32g_zca_zcb
#source: zc-zcb-sh.s
#objdump: -dr -Mno-aliases

.*:[	 ]+file format .*


Disassembly of section .text:

0+000 <zcb_sh>:
[	 ]*[0-9a-f]+:[	 ]+8c20[	 ]+c.sh[	 ]+s0,2\(s0\)
.*R_RISCV_RELAX_ENTRY.*
[	 ]*[0-9a-f]+:[	 ]+8f9c[	 ]+c.sh[	 ]+a5,0\(a5\)
[	 ]*[0-9a-f]+:[	 ]+8c20[	 ]+c.sh[	 ]+s0,2\(s0\)
[	 ]*[0-9a-f]+:[	 ]+8ca4[	 ]+c.sh[	 ]+s1,2\(s1\)
[	 ]*[0-9a-f]+:[	 ]+8e10[	 ]+c.sh[	 ]+a2,0\(a2\)
[	 ]*[0-9a-f]+:[	 ]+8eb4[	 ]+c.sh[	 ]+a3,2\(a3\)
[	 ]*[0-9a-f]+:[	 ]+8f18[	 ]+c.sh[	 ]+a4,0\(a4\)
[	 ]*[0-9a-f]+:[	 ]+8ca4[	 ]+c.sh[	 ]+s1,2\(s1\)
[	 ]*[0-9a-f]+:[	 ]+8fbc[	 ]+c.sh[  	 ]+a5,2\(a5\)
[	 ]*[0-9a-f]+:[	 ]+8c00[	 ]+c.sh[  	 ]+s0,0\(s0\)
[	 ]*[0-9a-f]+:[	 ]+8c84[	 ]+c.sh[  	 ]+s1,0\(s1\)
[	 ]*[0-9a-f]+:[	 ]+8e30[	 ]+c.sh[  	 ]+a2,2\(a2\)
[	 ]*[0-9a-f]+:[	 ]+8eb4[	 ]+c.sh[  	 ]+a3,2\(a3\)
[	 ]*[0-9a-f]+:[	 ]+8f18[	 ]+c.sh[  	 ]+a4,0\(a4\)
[	 ]*[0-9a-f]+:[	 ]+8c00[	 ]+c.sh[  	 ]+s0,0\(s0\)
[	 ]*[0-9a-f]+:[	 ]+8c84[	 ]+c.sh[  	 ]+s1,0\(s1\)
