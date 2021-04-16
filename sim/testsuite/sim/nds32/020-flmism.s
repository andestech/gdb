# nds32 test FLMISM, expected to pass.
# mach:	 all
# as:
# ld:		--defsym=_stack=0x3000000
# output:	test be\ntest le\npass\n

	.include "utils.inc"

.section	.rodata
	.align	2
DATA:
	.byte	0x11,0x22,0x33,0x44
	.byte	0x00,0x22,0x33,0x44
	.byte	0x00,0x00,0x33,0x44
	.byte	0x00,0x00,0x00,0x44
	.byte	0x00,0x00,0x00,0x00

.text
	.global	main
main:
	movi	$r9, 0
	! big
	setend.b
	PUTS	.Lbe

.Lagain:
	! match against 0x11223344
	l.w	$r6, DATA

	move	$r7, $r6
	flmism	$r0, $r6, $r7
	beqz	$r0, 1f
	PUTS	.Lfstr0
1:
	l.w	$r7, DATA + 4
	flmism	$r0, $r6, $r7
	beqc	$r0, -4, 1f
	PUTS	.Lfstr1
1:
	l.w	$r7, DATA + 8
	flmism	$r0, $r6, $r7
	beqc	$r0, -3, 1f
	PUTS	.Lfstr2
1:
	l.w	$r7, DATA + 12
	flmism	$r0, $r6, $r7
	beqc	$r0, -2, 1f
	PUTS	.Lfstr3
1:
	l.w	$r7, DATA + 16
	flmism  $r0, $r6, $r7
	beqc	$r0, -1, 1f
	PUTS	.Lfstr4
1:


	! match against 0x00003344
	l.w	$r6, DATA + 8

	l.w	$r7, DATA
	flmism	$r0, $r6, $r7
	beqc	$r0, -3, 1f
	PUTS	.Lfstr5
1:
	l.w	$r7, DATA + 4
	flmism $r0, $r6, $r7
	beqc	$r0, -3, 1f
	PUTS	.Lfstr6
1:
	l.w	$r7, DATA + 8
	flmism $r0, $r6, $r7
	beqc	$r0, 0, 1f
	PUTS	.Lfstr7
1:
	l.w	$r7, DATA + 12
	flmism $r0, $r6, $r7
	beqc	$r0, -2, 1f
	PUTS	.Lfstr8
1:
	l.w	$r7, DATA + 16
	flmism $r0, $r6, $r7
	beqc	$r0, -1, 1f
	PUTS	.Lfstr9
1:


	!! test it again for little-endian
	bnez	$r9, 1f
	movi	$r9, 1
	PUTS	.Lle
	setend.l
	j	.Lagain
1:
	PUTS	.Lpstr
	EXIT	0

.section	.rodata
.Lbe:    .string "test be\n"
.Lle:    .string "test le\n"
.Lpstr:  .string "pass\n"
.Lfstr0: .string "fail: flmism 11,22,33,44 11,22,33,44\n"
.Lfstr1: .string "fail: flmism 11,22,33,44 00,22,33,44\n"
.Lfstr2: .string "fail: flmism 11,22,33,44 00,00,33,44\n"
.Lfstr3: .string "fail: flmism 11,22,33,44 00,00,00,44\n"
.Lfstr4: .string "fail: flmism 11,22,33,44 00,00,00,00\n"

.Lfstr5: .string "fail: flmism 00,00,33,44 11,22,33,44\n"
.Lfstr6: .string "fail: flmism 00,00,33,44 00,22,33,44\n"
.Lfstr7: .string "fail: flmism 00,00,33,44 00,00,33,44\n"
.Lfstr8: .string "fail: flmism 00,00,33,44 00,00,00,44\n"
.Lfstr9: .string "fail: flmism 00,00,33,44 00,00,00,00\n"
