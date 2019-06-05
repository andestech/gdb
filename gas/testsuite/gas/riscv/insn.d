#as: -march=rv32icv
#objdump: -dr

.*:[ 	]+file format .*


Disassembly of section .text:

0+000 <target>:
[ 	]+0:[ 	]+00c58533[ 	]+add[ 	]+a0,a1,a2
.*R_RISCV_RELAX_ENTRY.*
[ 	]+4:[ 	]+00d58513[ 	]+addi[ 	]+a0,a1,13
[ 	]+8:[ 	]+00a58567[ 	]+jalr[ 	]+a0,10\(a1\)
[ 	]+c:[ 	]+00458503[ 	]+lb[ 	]+a0,4\(a1\)
[ 	]+10:[ 	]+feb508e3[ 	]+beq[ 	]+a0,a1,0 \<target\>
[	]+10: R_RISCV_BRANCH[	]+target
[ 	]+14:[ 	]+00a58223[ 	]+sb[ 	]+a0,4\(a1\)
[ 	]+18:[ 	]+00fff537[ 	]+lui[ 	]+a0,0xfff
[ 	]+1c:[ 	]+fe5ff56f[ 	]+jal[ 	]+a0,0 \<target\>
[	]+1c: R_RISCV_JAL[	]+target
[ 	]+20:[ 	]+0511[ 	]+c.addi[ 	]+a0,4
[ 	]+22:[ 	]+852e[ 	]+c.mv[ 	]+a0,a1
[ 	]+24:[ 	]+002c[ 	]+c.addi4spn[ 	]+a1,sp,8
[ 	]+26:[ 	]+dde9[ 	]+c.beqz[ 	]+a1,0 \<target\>
[	]+26: R_RISCV_RVC_BRANCH[	]+target
[ 	]+28:[ 	]+bfe1[ 	]+c.j[ 	]+0 \<target\>
[	]+28: R_RISCV_RVC_JUMP[	]+target
[ 	]+2a:[ 	]+00c58533[ 	]+add[ 	]+a0,a1,a2
[ 	]+2e:[ 	]+00d58513[ 	]+addi[ 	]+a0,a1,13
[ 	]+32:[ 	]+00a58567[ 	]+jalr[ 	]+a0,10\(a1\)
[ 	]+36:[ 	]+00458503[ 	]+lb[ 	]+a0,4\(a1\)
[ 	]+3a:[ 	]+fcb503e3[ 	]+beq[ 	]+a0,a1,0 \<target\>
[	]+3a: R_RISCV_BRANCH[	]+target
[ 	]+3e:[ 	]+00a58223[ 	]+sb[ 	]+a0,4\(a1\)
[ 	]+42:[ 	]+00fff537[ 	]+lui[ 	]+a0,0xfff
[ 	]+46:[ 	]+fbbff56f[ 	]+jal[ 	]+a0,0 \<target\>
[	]+46: R_RISCV_JAL[	]+target
[ 	]+4a:[ 	]+0511[ 	]+c.addi[ 	]+a0,4
[ 	]+4c:[ 	]+852e[ 	]+c.mv[ 	]+a0,a1
[ 	]+4e:[ 	]+002c[ 	]+c.addi4spn[ 	]+a1,sp,8
[ 	]+50:[ 	]+8d6d[ 	]+c.and[ 	]+a0,a1
[ 	]+52:[ 	]+d5dd[ 	]+c.beqz[ 	]+a1,0 \<target\>
[	]+52: R_RISCV_RVC_BRANCH[	]+target
[ 	]+54:[ 	]+b775[ 	]+c.j[ 	]+0 \<target\>
[	]+54: R_RISCV_RVC_JUMP[	]+target
[ 	]+56:[ 	]+68c58543[ 	]+fmadd.s[ 	]+fa0,fa1,fa2,fa3,rne
[ 	]+5a:[ 	]+00c58533[ 	]+add[ 	]+a0,a1,a2
[ 	]+5e:[ 	]+00c58533[ 	]+add[ 	]+a0,a1,a2
[ 	]+62:[ 	]+00c58533[ 	]+add[ 	]+a0,a1,a2
[ 	]+66:[ 	]+00c58533[ 	]+add[ 	]+a0,a1,a2
[ 	]+6a:[ 	]+00c58533[ 	]+add[ 	]+a0,a1,a2
[ 	]+6e:[ 	]+00c58533[ 	]+add[ 	]+a0,a1,a2
[ 	]+72:[ 	]+00c58533[ 	]+add[ 	]+a0,a1,a2
[^:]+:[ 	]+00d665af[ 	]+vamoaddw.v[ 	]+zero,\(a2\),v13,v11,v0.t
[^:]+:[ 	]+00d665af[ 	]+vamoaddw.v[ 	]+zero,\(a2\),v13,v11,v0.t
[^:]+:[ 	]+00d665af[ 	]+vamoaddw.v[ 	]+zero,\(a2\),v13,v11,v0.t
[^:]+:[ 	]+00d665af[ 	]+vamoaddw.v[ 	]+zero,\(a2\),v13,v11,v0.t
[^:]+:[ 	]+00d665af[ 	]+vamoaddw.v[ 	]+zero,\(a2\),v13,v11,v0.t
[^:]+:[ 	]+00d665af[ 	]+vamoaddw.v[ 	]+zero,\(a2\),v13,v11,v0.t
[^:]+:[ 	]+00d665af[ 	]+vamoaddw.v[ 	]+zero,\(a2\),v13,v11,v0.t
[^:]+:[ 	]+00d665af[ 	]+vamoaddw.v[ 	]+zero,\(a2\),v13,v11,v0.t
[^:]+:[ 	]+00d665af[ 	]+vamoaddw.v[ 	]+zero,\(a2\),v13,v11,v0.t
[^:]+:[ 	]+00d665af[ 	]+vamoaddw.v[ 	]+zero,\(a2\),v13,v11,v0.t
[^:]+:[ 	]+00d665af[ 	]+vamoaddw.v[ 	]+zero,\(a2\),v13,v11,v0.t
[^:]+:[ 	]+00d665af[ 	]+vamoaddw.v[ 	]+zero,\(a2\),v13,v11,v0.t
[^:]+:[ 	]+00d665af[ 	]+vamoaddw.v[ 	]+zero,\(a2\),v13,v11,v0.t
[^:]+:[ 	]+00d665af[ 	]+vamoaddw.v[ 	]+zero,\(a2\),v13,v11,v0.t
[^:]+:[ 	]+00d665af[ 	]+vamoaddw.v[ 	]+zero,\(a2\),v13,v11,v0.t
[^:]+:[ 	]+00d665af[ 	]+vamoaddw.v[ 	]+zero,\(a2\),v13,v11,v0.t
[^:]+:[ 	]+00d665af[ 	]+vamoaddw.v[ 	]+zero,\(a2\),v13,v11,v0.t
[^:]+:[ 	]+00d665af[ 	]+vamoaddw.v[ 	]+zero,\(a2\),v13,v11,v0.t
[^:]+:[ 	]+00d665af[ 	]+vamoaddw.v[ 	]+zero,\(a2\),v13,v11,v0.t
[^:]+:[ 	]+08d60587[ 	]+vlsbu.v[ 	]+v11,\(a2\),a3,v0.t
[^:]+:[ 	]+08d60587[ 	]+vlsbu.v[ 	]+v11,\(a2\),a3,v0.t
[^:]+:[ 	]+08d60587[ 	]+vlsbu.v[ 	]+v11,\(a2\),a3,v0.t
[^:]+:[ 	]+08d60587[ 	]+vlsbu.v[ 	]+v11,\(a2\),a3,v0.t
[^:]+:[ 	]+08d60587[ 	]+vlsbu.v[ 	]+v11,\(a2\),a3,v0.t
[^:]+:[ 	]+0ad60587[ 	]+vlsbu.v[ 	]+v11,\(a2\),a3
[^:]+:[ 	]+0ad60587[ 	]+vlsbu.v[ 	]+v11,\(a2\),a3
[^:]+:[ 	]+0ad60587[ 	]+vlsbu.v[ 	]+v11,\(a2\),a3
[^:]+:[ 	]+0ad60587[ 	]+vlsbu.v[ 	]+v11,\(a2\),a3
[^:]+:[ 	]+0ad60587[ 	]+vlsbu.v[ 	]+v11,\(a2\),a3
[^:]+:[ 	]+08d605a7[ 	]+vssb.v[ 	]+v11,\(a2\),a3,v0.t
[^:]+:[ 	]+08d605a7[ 	]+vssb.v[ 	]+v11,\(a2\),a3,v0.t
[^:]+:[ 	]+08d605a7[ 	]+vssb.v[ 	]+v11,\(a2\),a3,v0.t
[^:]+:[ 	]+08d605a7[ 	]+vssb.v[ 	]+v11,\(a2\),a3,v0.t
[^:]+:[ 	]+08d605a7[ 	]+vssb.v[ 	]+v11,\(a2\),a3,v0.t
[^:]+:[ 	]+0ad605a7[ 	]+vssb.v[ 	]+v11,\(a2\),a3
[^:]+:[ 	]+0ad605a7[ 	]+vssb.v[ 	]+v11,\(a2\),a3
[^:]+:[ 	]+0ad605a7[ 	]+vssb.v[ 	]+v11,\(a2\),a3
[^:]+:[ 	]+0ad605a7[ 	]+vssb.v[ 	]+v11,\(a2\),a3
[^:]+:[ 	]+0ad605a7[ 	]+vssb.v[ 	]+v11,\(a2\),a3
[^:]+:[ 	]+00060587[ 	]+vlbu.v[ 	]+v11,\(a2\),v0.t
[^:]+:[ 	]+03060587[ 	]+vlbuff.v[ 	]+v11,\(a2\)
[^:]+:[ 	]+000605a7[ 	]+vsb.v[ 	]+v11,\(a2\),v0.t
[^:]+:[ 	]+00d605d7[ 	]+vadd.vv[ 	]+v11,v13,v12,v0.t
[^:]+:[ 	]+00d605d7[ 	]+vadd.vv[ 	]+v11,v13,v12,v0.t
[^:]+:[ 	]+00d605d7[ 	]+vadd.vv[ 	]+v11,v13,v12,v0.t
[^:]+:[ 	]+00d605d7[ 	]+vadd.vv[ 	]+v11,v13,v12,v0.t
[^:]+:[ 	]+00d605d7[ 	]+vadd.vv[ 	]+v11,v13,v12,v0.t
[^:]+:[ 	]+00d605d7[ 	]+vadd.vv[ 	]+v11,v13,v12,v0.t
[^:]+:[ 	]+00d605d7[ 	]+vadd.vv[ 	]+v11,v13,v12,v0.t
[^:]+:[ 	]+00d615d7[ 	]+vfadd.vv[ 	]+v11,v13,v12,v0.t
[^:]+:[ 	]+00d615d7[ 	]+vfadd.vv[ 	]+v11,v13,v12,v0.t
[^:]+:[ 	]+00d625d7[ 	]+vredsum.vs[ 	]+v11,v13,v12,v0.t
[^:]+:[ 	]+00d625d7[ 	]+vredsum.vs[ 	]+v11,v13,v12,v0.t
[^:]+:[ 	]+00d645d7[ 	]+vadd.vx[ 	]+v11,v13,a2,v0.t
[^:]+:[ 	]+00d645d7[ 	]+vadd.vx[ 	]+v11,v13,a2,v0.t
[^:]+:[ 	]+00d655d7[ 	]+vfadd.vf[ 	]+v11,v13,fa2,v0.t
[^:]+:[ 	]+00d655d7[ 	]+vfadd.vf[ 	]+v11,v13,fa2,v0.t
[^:]+:[ 	]+38d665d7[ 	]+vslide1up.vx[ 	]+v11,v13,a2,v0.t
[^:]+:[ 	]+3cd665d7[ 	]+vslide1down.vx[ 	]+v11,v13,a2,v0.t
[^:]+:[ 	]+00d675d7[ 	]+vsetvli[ 	]+a1,a2,e64,m2,d1
[^:]+:[ 	]+00d675d7[ 	]+vsetvli[ 	]+a1,a2,e64,m2,d1
[^:]+:[ 	]+00d035d7[ 	]+vadd.vi[ 	]+v11,v13,0,v0.t
[^:]+:[ 	]+00d0b5d7[ 	]+vadd.vi[ 	]+v11,v13,1,v0.t
[^:]+:[ 	]+00d7b5d7[ 	]+vadd.vi[ 	]+v11,v13,15,v0.t
[^:]+:[ 	]+00d835d7[ 	]+vadd.vi[ 	]+v11,v13,-16,v0.t
[^:]+:[ 	]+00df35d7[ 	]+vadd.vi[ 	]+v11,v13,-2,v0.t
[^:]+:[ 	]+00dfb5d7[ 	]+vadd.vi[ 	]+v11,v13,-1,v0.t
