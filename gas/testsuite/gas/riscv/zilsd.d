#as: -march=rv32ic_zilsd
#source: zilsd.s
#objdump: -d

.*:[	 ]+file format .*

Disassembly of section .text:

0+000 <target>:
[	 ]+0:[	 ]+0080b003[	 ]+ld[	 ]+zero,8\(ra\)
[	 ]+4:[	 ]+6400[	 ]+c.ld[	 ]+s0,8\(s0\)
[	 ]+6:[	 ]+6122[	 ]+c.ldsp[	 ]+sp,8\(sp\)
[	 ]+8:[	 ]+0000b423[	 ]+sd[	 ]+zero,8\(ra\)
[	 ]+c:[	 ]+e400[	 ]+c.sd[	 ]+s0,8\(s0\)
[	 ]+e:[	 ]+e40a[	 ]+c.sdsp[	 ]+sp,8\(sp\)
[	 ]+10:[	 ]+e402[	 ]+c.sdsp[	 ]+zero,8\(sp\)
