# nds32 test J/J8/JR in ifcall, expected to pass.
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

	! ---- test J ----
	ifcall	2f
	nop
2:	j	3f
	PUTS	.Lfstr_j	! not jump
3:	check_ifcon	.Lfstr_ifcon_j


	!---- test J8 ----
	ifcall	2f
	nop
2:	j8	3f
	PUTS	.Lfstr_j8	! not jump
3:	check_ifcon	.Lfstr_ifcon_j8


	!---- test JR ----
	ifcall	2f
	nop
2:	la	$r0, 3f
	jr	$r0
	PUTS	.Lfstr_jr	! not jump
3:	check_ifcon	.Lfstr_ifcon_jr


	!---- test JR5 ----
	ifcall	2f
	nop
2:	la	$r0, 3f
	jr5	$r0
	PUTS	.Lfstr_jr5	! not jump
3:	check_ifcon	.Lfstr_ifcon_jr5


	!---- test JRNEZ ----
	ifcall	2f
	nop
2:	la	$r0, 3f
	jrnez	$r0
	PUTS	.Lfstr_jr5	! not jump
3:	check_ifcon	.Lfstr_ifcon_jr5

	beq	$lp, $r9, 3f
	PUTS	.Lfstr_lp

3:
	PUTS	.Lpstr
	EXIT	0

.section	.rodata
.Lpstr:       .string "pass\n"
.Lfstr_j:     .string "fail: j after ifcall\n"
.Lfstr_j8:    .string "fail: j8 after ifcall\n"
.Lfstr_jr:    .string "fail: jr after ifcall\n"
.Lfstr_jr5:   .string "fail: jr5 after ifcall\n"
.Lfstr_jrnez: .string "fail: jrnez after ifcall\n"
.Lfstr_ifcon_j:     .string "fail: PSW.IFCON is not cleared when j\n"
.Lfstr_ifcon_j8:    .string "fail: PSW.IFCON is not cleared when j8\n"
.Lfstr_ifcon_jr:    .string "fail: PSW.IFCON is not cleared when jr\n"
.Lfstr_ifcon_jr5:   .string "fail: PSW.IFCON is not cleared when jr5\n"
.Lfstr_ifcon_jrnez: .string "fail: PSW.IFCON is not cleared when jrnez\n"
.Lfstr_lp:    .string "fail: $lp is corrupted by j in ifcall\n"
