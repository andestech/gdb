# nds32 test J/J8/JR in ifcall, expected to pass.
# mach:		all
# as:		-mbaseline=v3
# ld:		--defsym=_stack=0x3000000
# output:	pass\n

	.include "utils.inc"

	.text

	! $r0: address for fail message
check_ifcon:
	! check IFCON
	movi    $r1, 32768	! bit-15 for IFCON
	mfsr	$r2, $psw
	and	$r2, $r2, $r1
	beqz	$r2, 1f
	jal	puts		! PSW.IFCON is not cleared
1:
	ret

	.global	main
main:
	smw.adm $r6, [$sp], $r9, 10

	! ---- test JAL ----
	ifcall	2f
	! should return from check_ifcon
	j	3f
2:	la	$r0, .Lfstr_ifcon_jal
	jal	check_ifcon
	PUTS	.Lfstr_jal		! return to the wrong address
3:	nop


	! ---- test JRAL ----
	ifcall	2f
	! should return from check_ifcon
	j	3f
2:	la	$r0, .Lfstr_ifcon_jral
	la	$r1, check_ifcon
	jral	$r1
	PUTS	.Lfstr_jral		! return to the wrong address
3:	nop


	! ---- test JRAL5 ----
	ifcall	2f
	! should return from check_ifcon
	j	3f
2:	la	$r0, .Lfstr_ifcon_jral5
	la	$r1, check_ifcon
	jral5	$r1
	PUTS	.Lfstr_jral5		! return to the wrong address
3:	nop


	! ---- test JRALNEZ ----
	ifcall	2f
	! should return from check_ifcon
	j	3f
2:	la	$r0, .Lfstr_ifcon_jralnez
	la	$r1, check_ifcon
	jralnez	$r1
	PUTS	.Lfstr_jralnez		! return to the wrong address
3:	nop

	PUTS	.Lpstr
	EXIT	0


.section	.rodata
.Lpstr:		.string "pass\n"
.Lfstr_jal:     .string "fail: return to wrong address (jal after ifcall)\n"
.Lfstr_jral:    .string "fail: return to wrong address (jral after ifcall)\n"
.Lfstr_jral5:   .string "fail: return to wrong address (jral5 after ifcall)\n"
.Lfstr_jralnez: .string "fail: return to wrong address (jralnez after ifcall)\n"
.Lfstr_ifcon_jal:      .string "fail: PSW.IFCON is not cleared when jal\n"
.Lfstr_ifcon_jral:     .string "fail: PSW.IFCON is not cleared when jral\n"
.Lfstr_ifcon_jral5:    .string "fail: PSW.IFCON is not cleared when jral5\n"
.Lfstr_ifcon_jralnez:  .string "fail: PSW.IFCON is not cleared when jralnez\n"
