# nds32 test FFBI, expected to pass.
# mach:		all
# as:		-mbaseline=v3
# ld:		--defsym=_stack=0x3000000
# output:	test be\ntest le\npass\n

	.include "utils.inc"

.section	.rodata
	.align	2
DATA:
	.byte	0x12,0x12,0x12,0x99

.text
	.global	main
main:
	movi	$r9, 0
	! big
	setend.b
	PUTS	.Lbe

.Lagain:
	! $6 = 0x12121299 in be
	! $6 = 0x99121212 in le

	l.w	$r6, DATA

	ffbi	$r0, $r6, #0x00
	beqz	$r0, 1f
	PUTS	.Lfstr0
1:
	ffbi	$r0, $r6, #0x99
	beqc	$r0, -1, 1f
	PUTS	.Lfstr1
1:


	! test it again for little-endian
	bnez	$r9, 1f
	movi	$r9, 1
	PUTS	.Lle
	setend.l
	j	.Lagain
1:
	PUTS	.Lpstr
	EXIT	0

.section	.rodata
.Lbe:	 .string "test be\n"
.Lle:	 .string "test le\n"
.Lpstr:  .string "pass\n"
.Lfstr0: .string "fail: ffbi no matching byte is found\n"
.Lfstr1: .string "fail: ffbi should return -1\n"
