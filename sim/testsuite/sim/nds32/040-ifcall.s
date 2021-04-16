# nds32 test basic ifcall and PSW.IFCON, expected to pass.
# mach:	 	all
# as:		-mbaseline=v3
# ld:		--defsym=_stack=0x3000000
# output:	pass\n

	.include "utils.inc"

	.text
	.global main
main:
	smw.adm $r6, [$sp], $r9, 10

	movi    $r8, 32768	! bit-15 for IFCON

	movi    $r7, 0

	ifcall  .Lcommon0

	! Return from ifret. Make sure ifcall is really done.
	addi    $r7, $r7, -1
	! check $r7 == 0
	beqz	$r7, .L2
	PUTS	.Lfstr2
.L2:
	! check IFCON is off
	mfsr	$r1, $psw
	and	$r1, $r1, $r8
	beqz	$r1, .L3
	PUTS	.Lfstr1
	EXIT	1
.L3:
	PUTS	.Lpstr
	EXIT	0

.Lcommon0:
	! check IFCON is set
	mfsr	$r1, $psw
	and	$r1, $r1, $r8
	beqz	$r1, .L1
	addi	$r7, $r7, 1
	ifret
	PUTS	.Lfstr3	! fail to ifret
	EXIT	1
.L1:
	PUTS	.Lfstr0	! FAIL: IFCON not set
	EXIT	1

.section	.rodata
	.align 2
.Lpstr:  .string "pass\n"
.Lfstr0: .string "fail: IFCON is not set\n"
.Lfstr1: .string "fail: IFCON is not off\n"
.Lfstr2: .string "fail: fail to ifcall\n"
.Lfstr3: .string "fail: fail to ifret\n"
