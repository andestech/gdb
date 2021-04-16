# nds32 setend, expected to pass.
# mach:	 	all
# as:		-mbaseline=v3
# ld:		--defsym=_stack=0x3000000
# output:	pass\n

	.include "utils.inc"

.data
	.align 2
WORD:	.byte	0x11,0x22,0x33,0x44

	.text
	.global main
main:
	smw.adm $r6, [$sp], $r13, 10

	li	$r6, 0x11223344
	li	$r7, 0x44332211

	setend.b
	l.w	$r0, WORD
	beq	$r0, $r6, 1f
	PUTS	.Lfstr0		! setend.b fail
	beq	$r0, $r7, 1f
	PUTS	.Lfstr2		! $r0 is neither little-endian
1:
	setend.l
	l.w	$r0, WORD
	beq	$r0, $r7, 1f
	PUTS	.Lfstr1		! setend.b fail
	beq	$r0, $r6, 1f
	PUTS	.Lfstr2		! $r0 is neither big-endian
1:
	PUTS	.Lpstr
	movi	$r0, 0
	lmw.bim $r6, [$sp], $r13, 10
	ret

.section .rodata
	.align 2
.Lpstr:	 .string "pass\n"
.Lfstr0: .string "fail: setend.b\n"
.Lfstr1: .string "fail: setend.l\n"
.Lfstr2: .string "fail: l.w\n"
