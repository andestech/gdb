# nds32 test chaining ifcall, expected to pass.
# mach:		all
# as:		-mbaseline=v3
# ld:		--defsym=_stack=0x3000000
# output:	pass\n

	.include "utils.inc"

	.text

!!! USE MACRO HERE, because jal may clear IFCON
.macro	check_ifcon	str
	! check IFCON
	movi    $r0, 32768	! bit-15 for IFCON
	mfsr	$r1, $psw
	and	$r1, $r1, $r0
	beqz	$r1, 1f
	PUTS	\str		! PSW.IFCON is not cleared
1:
	nop
.endm

	.global	main
main:
	smw.adm $r6, [$sp], $r9, 10

	! Check $lp after test.
	move	$r9, $lp

	ifcall	1f
	! Should return to here from ifret.
	beq	$lp, $r9, .LPASS
	PUTS	.Lfstr_lp
.LPASS:
	PUTS	.Lpstr
	EXIT	0

1:
	ifcall9	1f
	PUTS	.Lfstr0
	EXIT	1
1:
	ifcall	1f
	PUTS	.Lfstr0
	EXIT	1
1:
	ifcall9	1f
	PUTS	.Lfstr0
	EXIT	1
1:
	ifret
	PUTS	.Lfstr0
	EXIT	1

.section	.rodata
.Lpstr:    .string "pass\n"
.Lfstr0:   .string "fail: chaining ifcall\n"
.Lfstr_lp: .string "fail: $lp corrupted\n"
