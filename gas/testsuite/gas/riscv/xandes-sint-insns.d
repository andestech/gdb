#as: -march=rv32ifv_xandes
#objdump: -d

.*:[ 	]+file format .*


Disassembly of section .text:

0+000 <.text>:
[ 	]+[0-9a-f]+:[ 	]+0605425b[ 	]+vle4.v[ 	]+v4,\(a0\)
#[ 	]+[0-9a-f]+:[ 	]+0404445b[ 	]+vle4.v[ 	]+v8,\(s0\),v0.t
[ 	]+[0-9a-f]+:[ 	]+0605425b[ 	]+vle4.v[ 	]+v4,\(a0\)
[ 	]+[0-9a-f]+:[ 	]+0404445b[ 	]+vln.v[ 	]+v8,\(s0\),v0.t
[ 	]+[0-9a-f]+:[ 	]+0242405b[ 	]+vfwcvt.f.n.v[ 	]+v0,v4
[ 	]+[0-9a-f]+:[ 	]+0252c0db[ 	]+vfwcvt.f.nu.v[ 	]+v1,v5
[ 	]+[0-9a-f]+:[ 	]+0263415b[ 	]+vfwcvt.f.b.v[ 	]+v2,v6
[ 	]+[0-9a-f]+:[ 	]+0273c1db[ 	]+vfwcvt.f.bu.v[ 	]+v3,v7
[ 	]+[0-9a-f]+:[ 	]+0042405b[ 	]+vfwcvt.f.n.v[ 	]+v0,v4,v0.t
[ 	]+[0-9a-f]+:[ 	]+0052c0db[ 	]+vfwcvt.f.nu.v[ 	]+v1,v5,v0.t
[ 	]+[0-9a-f]+:[ 	]+0063415b[ 	]+vfwcvt.f.b.v[ 	]+v2,v6,v0.t
[ 	]+[0-9a-f]+:[ 	]+0073c1db[ 	]+vfwcvt.f.bu.v[ 	]+v3,v7,v0.t
[ 	]+[0-9a-f]+:[ 	]+0a60c25b[ 	]+vfpmadt.vf[ 	]+v4,ft1,v6
[ 	]+[0-9a-f]+:[ 	]+0861425b[ 	]+vfpmadt.vf[ 	]+v4,ft2,v6,v0.t
[ 	]+[0-9a-f]+:[ 	]+0e71c2db[ 	]+vfpmadb.vf[ 	]+v5,ft3,v7
[ 	]+[0-9a-f]+:[ 	]+0c7242db[ 	]+vfpmadb.vf[ 	]+v5,ft4,v7,v0.t
