#as: -march=rv64g_zca_zcb
#source: zc-zcb-test-operand-2.s
#objdump: -dr -Mno-aliases

.*:[	 ]+file format .*


Disassembly of section .text:

0+000 <target>:
[	 ]*[0-9a-f]+:[	 ]+fe840fa3+[	 ]+sb[	 ]+s0,-1\(s0\)
.*R_RISCV_RELAX_ENTRY.*
[	 ]*[0-9a-f]+:[	 ]+00840223+[	 ]+sb[	 ]+s0,4\(s0\)
[	 ]*[0-9a-f]+:[	 ]+fe840f23+[	 ]+sb[	 ]+s0,-2\(s0\)
[	 ]*[0-9a-f]+:[	 ]+fff44403+[	 ]+lbu[	 ]+s0,-1\(s0\)
[	 ]*[0-9a-f]+:[	 ]+00444403+[	 ]+lbu[	 ]+s0,4\(s0\)
[	 ]*[0-9a-f]+:[	 ]+ffe44403+[	 ]+lbu[	 ]+s0,-2\(s0\)
[	 ]*[0-9a-f]+:[	 ]+00841223+[	 ]+sh[	 ]+s0,4\(s0\)
[	 ]*[0-9a-f]+:[	 ]+008411a3+[	 ]+sh[	 ]+s0,3\(s0\)
[	 ]*[0-9a-f]+:[	 ]+fe841fa3+[	 ]+sh[	 ]+s0,-1\(s0\)
[	 ]*[0-9a-f]+:[	 ]+fe841f23+[	 ]+sh[	 ]+s0,-2\(s0\)
[	 ]*[0-9a-f]+:[	 ]+00441403+[	 ]+lh[	 ]+s0,4\(s0\)
[	 ]*[0-9a-f]+:[	 ]+00341403+[	 ]+lh[	 ]+s0,3\(s0\)
[	 ]*[0-9a-f]+:[	 ]+fff41403+[	 ]+lh[	 ]+s0,-1\(s0\)
[	 ]*[0-9a-f]+:[	 ]+ffd41403+[	 ]+lh[	 ]+s0,-3\(s0\)
[	 ]*[0-9a-f]+:[	 ]+00445403+[	 ]+lhu[	 ]+s0,4\(s0\)
[	 ]*[0-9a-f]+:[	 ]+00345403+[	 ]+lhu[	 ]+s0,3\(s0\)
[	 ]*[0-9a-f]+:[	 ]+fff45403+[	 ]+lhu[	 ]+s0,-1\(s0\)
[	 ]*[0-9a-f]+:[	 ]+00345403+[	 ]+lhu[	 ]+s0,3\(s0\)
