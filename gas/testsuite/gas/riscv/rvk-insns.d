#as: -march=rv64ifd_zfh
#objdump: -d

.*:[ 	]+file format .*


Disassembly of section .text:

0+000 <.+>:
[ 	]+[0-9a-f]+:[ 	]+12351087[ 	]+flh[  	]+ft1,291\(a0\)
[ 	]+[0-9a-f]+:[ 	]+00000297[ 	]+auipc[  	]+t0,0x0
[ 	]+[0-9a-f]+:[ 	]+00029107[ 	]+flh[  	]+ft2,0\(t0\) # .+
[ 	]+[0-9a-f]+:[ 	]+44359b27[ 	]+fsh[  	]+ft3,1110\(a1\)
[ 	]+[0-9a-f]+:[ 	]+00000317[ 	]+auipc[  	]+t1,0x0
[ 	]+[0-9a-f]+:[ 	]+00431027[ 	]+fsh[  	]+ft4,0\(t1\) # .+
[ 	]+[0-9a-f]+:[ 	]+e40282d3[ 	]+fmv.x.h[ 	]+t0,ft5
[ 	]+[0-9a-f]+:[ 	]+f4030353[ 	]+fmv.h.x[ 	]+ft6,t1
[ 	]+[0-9a-f]+:[ 	]+248403d3[ 	]+fmv.h[ 	]+ft7,fs0
[ 	]+[0-9a-f]+:[ 	]+24a514d3[ 	]+fneg.h[ 	]+fs1,fa0
[ 	]+[0-9a-f]+:[ 	]+24c625d3[ 	]+fabs.h[ 	]+fa1,fa2
[ 	]+[0-9a-f]+:[ 	]+24f706d3[ 	]+fsgnj.h[ 	]+fa3,fa4,fa5
[ 	]+[0-9a-f]+:[ 	]+25289853[ 	]+fsgnjn.h[ 	]+fa6,fa7,fs2
[ 	]+[0-9a-f]+:[ 	]+255a29d3[ 	]+fsgnjx.h[ 	]+fs3,fs4,fs5
[ 	]+[0-9a-f]+:[ 	]+058bfb53[ 	]+fadd.h[ 	]+fs6,fs7,fs8
[ 	]+[0-9a-f]+:[ 	]+05bd0cd3[ 	]+fadd.h[ 	]+fs9,fs10,fs11,rne
[ 	]+[0-9a-f]+:[ 	]+0deefe53[ 	]+fsub.h[ 	]+ft8,ft9,ft10
[ 	]+[0-9a-f]+:[ 	]+0c101fd3[ 	]+fsub.h[ 	]+ft11,ft0,ft1,rtz
[ 	]+[0-9a-f]+:[ 	]+1441f153[ 	]+fmul.h[ 	]+ft2,ft3,ft4
[ 	]+[0-9a-f]+:[ 	]+147322d3[ 	]+fmul.h[ 	]+ft5,ft6,ft7,rdn
[ 	]+[0-9a-f]+:[ 	]+1ca4f453[ 	]+fdiv.h[ 	]+fs0,fs1,fa0
[ 	]+[0-9a-f]+:[ 	]+1cd635d3[ 	]+fdiv.h[ 	]+fa1,fa2,fa3,rup
[ 	]+[0-9a-f]+:[ 	]+5c07f753[ 	]+fsqrt.h[ 	]+fa4,fa5
[ 	]+[0-9a-f]+:[ 	]+5c0948d3[ 	]+fsqrt.h[ 	]+fa7,fs2,rmm
[ 	]+[0-9a-f]+:[ 	]+2d6a8a53[ 	]+fmin.h[ 	]+fs4,fs5,fs6
[ 	]+[0-9a-f]+:[ 	]+2d9c1bd3[ 	]+fmax.h[ 	]+fs7,fs8,fs9
[ 	]+[0-9a-f]+:[ 	]+edcdfd43[ 	]+fmadd.h[ 	]+fs10,fs11,ft8,ft9
[ 	]+[0-9a-f]+:[ 	]+05ff7ec3[ 	]+fmadd.h[ 	]+ft9,ft10,ft11,ft0
[ 	]+[0-9a-f]+:[ 	]+1c20f04f[ 	]+fnmadd.h[ 	]+ft0,ft1,ft2,ft3
[ 	]+[0-9a-f]+:[ 	]+345201cf[ 	]+fnmadd.h[ 	]+ft3,ft4,ft5,ft6,rne
[ 	]+[0-9a-f]+:[ 	]+4c83f347[ 	]+fmsub.h[ 	]+ft6,ft7,fs0,fs1
[ 	]+[0-9a-f]+:[ 	]+64b514c7[ 	]+fmsub.h[ 	]+fs1,fa0,fa1,fa2,rtz
[ 	]+[0-9a-f]+:[ 	]+7ce6f64b[ 	]+fnmsub.h[ 	]+fa2,fa3,fa4,fa5
[ 	]+[0-9a-f]+:[ 	]+951827cb[ 	]+fnmsub.h[ 	]+fa5,fa6,fa7,fs2,rdn
[ 	]+[0-9a-f]+:[ 	]+c400f0d3[ 	]+fcvt.w.h[ 	]+ra,ft1
[ 	]+[0-9a-f]+:[ 	]+c4013153[ 	]+fcvt.w.h[ 	]+sp,ft2,rup
[ 	]+[0-9a-f]+:[ 	]+c411f1d3[ 	]+fcvt.wu.h[ 	]+gp,ft3
[ 	]+[0-9a-f]+:[ 	]+c4124253[ 	]+fcvt.wu.h[ 	]+tp,ft4,rmm
[ 	]+[0-9a-f]+:[ 	]+d402f2d3[ 	]+fcvt.h.w[ 	]+ft5,t0
[ 	]+[0-9a-f]+:[ 	]+d4037353[ 	]+fcvt.h.w[ 	]+ft6,t1
[ 	]+[0-9a-f]+:[ 	]+d413f3d3[ 	]+fcvt.h.wu[ 	]+ft7,t2
[ 	]+[0-9a-f]+:[ 	]+d4140453[ 	]+fcvt.h.wu[ 	]+fs0,s0,rne
[ 	]+[0-9a-f]+:[ 	]+e40494d3[ 	]+fclass.h[ 	]+s1,fs1
[ 	]+[0-9a-f]+:[ 	]+a4b52553[ 	]+feq.h[ 	]+a0,fa0,fa1
[ 	]+[0-9a-f]+:[ 	]+a4d615d3[ 	]+flt.h[ 	]+a1,fa2,fa3
[ 	]+[0-9a-f]+:[ 	]+a4f70653[ 	]+fle.h[ 	]+a2,fa4,fa5
[ 	]+[0-9a-f]+:[ 	]+a50896d3[ 	]+flt.h[ 	]+a3,fa7,fa6
[ 	]+[0-9a-f]+:[ 	]+a5298753[ 	]+fle.h[ 	]+a4,fs3,fs2
[ 	]+[0-9a-f]+:[ 	]+c420f0d3[ 	]+fcvt.l.h[ 	]+ra,ft1
[ 	]+[0-9a-f]+:[ 	]+c4213153[ 	]+fcvt.l.h[ 	]+sp,ft2,rup
[ 	]+[0-9a-f]+:[ 	]+c431f1d3[ 	]+fcvt.lu.h[ 	]+gp,ft3
[ 	]+[0-9a-f]+:[ 	]+c4324253[ 	]+fcvt.lu.h[ 	]+tp,ft4,rmm
[ 	]+[0-9a-f]+:[ 	]+d422f2d3[ 	]+fcvt.h.l[ 	]+ft5,t0
[ 	]+[0-9a-f]+:[ 	]+d4237353[ 	]+fcvt.h.l[ 	]+ft6,t1
[ 	]+[0-9a-f]+:[ 	]+d433f3d3[ 	]+fcvt.h.lu[ 	]+ft7,t2
[ 	]+[0-9a-f]+:[ 	]+d4340453[ 	]+fcvt.h.lu[ 	]+fs0,s0,rne
[ 	]+[0-9a-f]+:[ 	]+40208053[ 	]+fcvt.s.h[ 	]+ft0,ft1
[ 	]+[0-9a-f]+:[ 	]+4401f153[ 	]+fcvt.h.s[ 	]+ft2,ft3
[ 	]+[0-9a-f]+:[ 	]+44029253[ 	]+fcvt.h.s[ 	]+ft4,ft5,rtz
[ 	]+[0-9a-f]+:[ 	]+42238353[ 	]+fcvt.d.h[ 	]+ft6,ft7
[ 	]+[0-9a-f]+:[ 	]+4414f453[ 	]+fcvt.h.d[ 	]+fs0,fs1
[ 	]+[0-9a-f]+:[ 	]+4415a553[ 	]+fcvt.h.d[ 	]+fa0,fa1,rdn
