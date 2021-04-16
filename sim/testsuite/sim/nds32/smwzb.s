# nds32 test SMWZB, expected to pass.
# mach:	 all
# as:
# ld:		--defsym=_stack=0x3000000
# output:	pass\n

	.include "utils.inc"

.data
	.align 2
BASE:
	.word	0xffffffff ! $r6
	.word	0xffffffff ! $r7
	.word	0xffffffff ! $r8
	.word	0xffffffff ! $r9
	.word	0xffffffff ! $fp
WORD:
	.byte	0x77,0x00,0xff,0xff

.text
	.global	main
main:
	la	$r10, BASE	! load base
	addi	$r11, $r10, #20	! 5 words
	li	$r6, 0x11222211
	li	$r7, 0x55666655
	li	$r8, 0x77000077
	li	$r9, 0xaabbbbaa
	li	$fp, 0xccddddcc

	! expect:
	!	$r6: 0x11222211
	!	$r7: 0x55666655
	!	$r8: 0x77000077
	!	$r9: 0 (untouched)
	!	$fp: 0 (untouched)
	!	$r10 = BASE + 20
	smwzb.bm	$r6,[$r10],$r9,0x8

	beq	$r11, $r10, 1f
	PUTS	.Lfstr0

1:	l.w	$r0, BASE
	beq	$r6, $r0, 1f
	PUTS	.Lfstr1

1:	l.w	$r0, BASE + 4
	beq	$r7, $r0, 1f
	PUTS	.Lfstr2

1:	l.w	$r0, BASE + 8
	l.w	$r8, WORD
	beq	$r8, $r0, 1f
	PUTS	.Lfstr3

1:	l.w	$r0, BASE + 12
	li	$r9, 0xffffffff
	beq	$r9, $r0, 1f
	PUTS	.Lfstr4

1:	l.w	$r0, BASE + 16
	li	$fp, 0xffffffff
	beq	$fp, $r0, 1f
	PUTS	.Lfstr5

1:	PASS
	EXIT	0

.section	.rodata
.Lpstr:  .string "pass\n"
.Lfstr0: .string "fail: adjust $Ra + 20\n"
.Lfstr1: .string "fail: $r6 == 0x11222211\n"
.Lfstr2: .string "fail: $r7 == 0x55666655\n"
.Lfstr3: .string "fail: $r8 == 0x77000077\n"
.Lfstr4: .string "fail: $r9 == 0xffffffff != 0xaabbbbaa\n"
.Lfstr5: .string "fail: $fp == 0xffffffff != 0xaabbbbaa\n"
