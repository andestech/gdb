# nds32 beqc/bnec, expected to pass.
# mach:	 	all
# as:		-mbaseline=v3
# ld:		--defsym=_stack=0x3000000
# output:	pass\n

	.include "utils.inc"

	.text
	.global main
main:
	smw.adm $sp, [$sp], $sp, 10

	movi    $r7, 13
.L0:
	beqc	$r7, 13, .L1
	PUTS	.Lfstr0	! eq, but not take
.L1:
	beqc	$r7, 17, .L2
	bnec	$r7, 17, .L3
	PUTS	.Lfstr1	! ne, but not take

.L2:
	PUTS	.Lfstr2	! ne, but take

.L3:
	bnec	$r7, 13, .L4
	PUTS	.Lpstr
	EXIT	0

.L4:
	PUTS	.Lfstr3	! eq, but take

	lmw.bim $sp, [$sp], $sp, 10
	ret

.data
	.align 2
.Lpstr:	 .string "pass\n"
.Lfstr0: .string "fail: eq, but not take\n"
.Lfstr1: .string "fail: ne, but not take\n"
.Lfstr2: .string "fail: ne, but take\n"
.Lfstr3: .string "fail: eq, but take\n"
