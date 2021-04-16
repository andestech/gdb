# nds32 beqc/bnec, expected to pass.
# mach:	 	all
# as:		-mbaseline=v3
# ld:		--defsym=_stack=0x3000000
# output:	pass\n*lignment*MWA*\n
# xerror:

	.include "utils.inc"

	.data
	.align 2
WORD:
	.byte	0x80
	.byte	0x81
HALF:
	.byte	0x82
	.byte	0x83
	.byte	0x84
	.byte	0x85
	.byte	0x86
	.byte	0x87

	.text
	.global main
main:
	smw.adm $sp, [$sp], $sp, 10

	! Set to big endian
	setend.b

	la	$r9, HALF
	lmw.bi	$r0, [$r9], $r0, 0
	li	$r1, 0x82838485
	beq	$r0, $r1, 1f
	PUTS	.Lfstr0

1:
	li	$r0, 0x12345678
	smw.bi	$r0, [$r9], $r0, 0

	! Set to little endian
	setend.l

	la	$r9, HALF
	lmw.bi	$r0, [$r9], $r0, 0
	li	$r1, 0x78563412
	beq	$r0, $r1, 1f
	FAIL	2
	PUTS	.Lfstr1
1:
	li	$r0, 0xaabbccdd
	smw.bi	$r0, [$r9], $r0, 0

	la	$r9, WORD
	lwi	$r0, [$r9 + 0]
	li	$r1, 0xccdd8180
	beq	$r0, $r1, 1f
	FAIL	3
	PUTS	.Lfstr2
1:

	la	$r0, LPASS_STR
	bal	puts

	la	$r9, HALF
	lmwa.bi	$r0, [$r9], $r0, 0	! Expected to fail.

	PUTS	.Lfstr3
	EXIT	1


.section	.rodata
.Lfstr0: .string "fail: Unaligned LMW.bi (big-endian)\n"
.Lfstr1: .string "fail: Unaligned LMW.bi (little-endian)\n"
.Lfstr2: .string "fail: Aligned LWI\n"
.Lfstr3: .string "fail: Unaligned LMWA.bi\n"
