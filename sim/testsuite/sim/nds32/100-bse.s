# nds32 bit stream extraction
# mach:	 	all
# as:		-mbaseline=v3 -mperf2-ext
# ld:		--defsym=_stack=0x3000000
# output:	pass\n

	.include "utils.inc"

.data
	.align 2

	.text
	.global main
main:
	smw.adm $r6, [$sp], $r25, 10

	movi	$r25, 0

	! case 1 - extract 12 bits from 12 bits
	! check rt and rb[4:0] (distance)
	li	$r7, 0x1234567a	! input
	li	$r8, 0xb0c	! rb
	bse	$r6, $r7, $r8	! 456

	li	$r9, 0x456
	beq	$r6, $r9, 1f	! check rt
	addi	$r25, $r25, 1
	PUTS	.Lfstr0a
1:
	andi	$r9, $r8, 0x1f
	beqc	$r9, 24, 1f	! check rb[4:0] updated distance
	addi	$r25, $r25, 1
	PUTS	.Lfstr0b
1:

	! case 2 - non-occupied untouched
	! check rt
	li	$r7, 0x87654321 ! input
	li	$r8, 0x40000b0c	! rb
	li	$r6, 0xabcdef12	! rt (non-occupied should be untouched)
	bse	$r6, $r7, $r8	! abcde543

	li	$r9, 0xabcde543
	beq	$r6, $r9, 1f	! check rt
	addi	$r25, $r25, 1
	PUTS	.Lfstr1
1:


	! case 3 - empty condition
	! check rt and refill-bit
	li	$r7, 0xabcd1234	! input
	li	$r8, 0x00000b14	! rb
	bse	$r6, $r7, $r8	! 234

	li	$r9, 0x234
	beq	$r6, $r9, 1f	! check rt
	addi	$r25, $r25, 1
	PUTS	.Lfstr2a
1:
	srli	$r9, $r7, 30
	beqc	$r9, 2, 1f	! check rb[31] refill-bit
	addi	$r25, $r25, 1
	PUTS	.Lfstr2b
1:

	! case 4 - underflow condition
	! check rt and rb
	li	$r7, 0x1a2b3c4d ! input
	li	$r8, 0xb18	! rb
	bse	$r6, $r7, $r8

	li	$r9, 0x4d0
	beq	$r6, $r9, 1f	! check rt
	addi	$r25, $r25, 1
	PUTS	.Lfstr3
1:

	! case 5 - underflow and refill
	li	$r7, 0x8a7b6c5d	! input
	li	$r8, 0xb18	! rb
	bse	$r6, $r7, $r8	! 5d0
	bse	$r6, $r7, $r8	! 5d8
	bse	$r7, $r7, $r8	! a7b

	li	$r9, 0x5d8
	beq	$r6, $r9, 1f	! check refilled rt
	addi	$r25, $r25, 1
	PUTS	.Lfstr4a
1:
	li	$r9, 0xa7b
	beq	$r7, $r9, 1f	! check next extract
	addi	$r25, $r25, 1
	PUTS	.Lfstr4b
1:


	bnez	$r25, 1f
	PUTS	.Lpstr
	movi	$r0, 0
1:
	lmw.bim $r6, [$sp], $r25, 10
	ret

.section .rodata
	.align 2
.Lpstr:	 .string "pass\n"
.Lfstr0a: .string "fail: bse normal condition.\n"
.Lfstr0b: .string "fail: bse normal condition. (update distance)\n"
.Lfstr1:  .string "fail: bse normal condition. (non-occupied untouched)\n"
.Lfstr2a: .string "fail: bse empty condition. \n"
.Lfstr2b: .string "fail: bse empty condition. (refill-bit)\n"
.Lfstr3:  .string "fail: bse underflow condition.\n"
.Lfstr4a: .string "fail: bse underflow condition refilling.\n"
.Lfstr4b: .string "fail: bse underflow condition next extraction.\n"
