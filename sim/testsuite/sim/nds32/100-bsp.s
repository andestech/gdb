# nds32 bit stream packing (normal)
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

	! case 1 - pack 12 bits from 12 bits
	! check rt and rb[4:0] (distance)
	li	$r6, 0x87654321	! rt
	li	$r7, 0xabcdefab	! ra
	li	$r8, 0xb0c	! rb
	bsp	$r6, $r7, $r8	! 0x876fab21

	li	$r9, 0x876fab21
	beq	$r6, $r9, 1f	! check rt
	addi	$r25, $r25, 1
	PUTS	.Lfstr0a
1:
	andi	$r9, $r8, 0x1f
	beqc	$r9, 24, 1f	! check rb[4:0] updated distance
	addi	$r25, $r25, 1
	PUTS	.Lfstr0b
1:


	! case 2 - empty condition
	! check rt and refill-bit
	li	$r6, 0x87654321 ! rt
	li	$r7, 0xabcdefab	! ra
	li	$r8, 0x00000b14	! rb
	bsp	$r6, $r7, $r8	! 0x87654fab

	li	$r9, 0x87654fab
	beq	$r6, $r9, 1f	! check rt
	addi	$r25, $r25, 1
	PUTS	.Lfstr1a
1:
	srli	$r9, $r7, 30
	beqc	$r9, 2, 1f	! check rb[31] refill-bit
	addi	$r25, $r25, 1
	PUTS	.Lfstr1b
1:

	! case 3 - underflow condition
	! check rt and refilling
	li	$r6, 0x87654321 ! rt
	li	$r7, 0xa8b7c6d5	! ra
	li	$r8, 0x00000b18	! rb

	bsp	$r6, $r7, $r8	! 0x8765436d
	li	$r9, 0x8765436d
	beq	$r6, $r9, 1f	! check rt
	addi	$r25, $r25, 1
	PUTS	.Lfstr2a
1:
	bsp	$r6, $r7, $r8	! 0x5765436d
	li	$r9, 0x5765436d
	beq	$r6, $r9, 1f	! check next filling
	addi	$r25, $r25, 1
	PUTS	.Lfstr2b
1:
	bsp	$r6, $r7, $r8	! 0x56d5436d
	li	$r9, 0x56d5436d
	beq	$r6, $r9, 1f	! check next filling
	addi	$r25, $r25, 1
	PUTS	.Lfstr2c
1:


	bnez	$r25, 1f
	PUTS	.Lpstr
	movi	$r0, 0
1:
	lmw.bim $r6, [$sp], $r25, 10
	ret

.section .rodata
	.align 2
.Lpstr:	  .string "pass\n"
.Lfstr0a: .string "fail: bsp normal condition.\n"
.Lfstr0b: .string "fail: bsp normal condition. (update distance)\n"
.Lfstr1a: .string "fail: bsp empty condition.\n"
.Lfstr1b: .string "fail: bsp empty condition. (update distance)\n"
.Lfstr2a: .string "fail: bsp underflow condition.\n"
.Lfstr2b: .string "fail: bsp underflow condition. (refilling)\n"
.Lfstr2c: .string "fail: bsp underflow condition. (next filling)\n"
