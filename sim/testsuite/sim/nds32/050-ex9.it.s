# nds32 test J/JAL in ex9, expected to pass.
# mach:		all
# as:		-mbaseline=v3
# ld:		--defsym=_stack=0x3000000
# output:	pass\n

	.include "utils.inc"

.section	.ex9.itable, "a"
	.align	2
ITB:
	.space	32 * 4, 0	! pad 32 entries
.L32:
	addi	$r7, $r7, 13
.LITB_J:
	j	.LITB_J
.LITB_JAL:
	jal	.LITB_JAL
	jr	$r8


	.text
test_jal_call:
	addi	$r7, $r7, -1
	ret

	.global	main
main:
	smw.adm $r6, [$sp], $r9, 10

	! Test big-endian only.
	! The code for relocation is off topic.
	setend.b

	la	$r9, ITB
	mtusr	$r9, $ITB

	la	$r9, .L32	! address of ITB entry 32

	!	normal instruction >= 32
	movi	$r7, 17
	ex9.it	32
	addi	$r7, $r7, -30

	beqz	$r7, .Ltest_j32
	PUTS	.Lfstr_n32


.Ltest_j32:
	!	j > 32
	! fix the entry in table
	lwi	$r7, [$r9 + 4]
	la	$r0, .Ltest_jal	! fix this address in
	srli	$r0, $r0, 1
	or	$r0, $r0, $r7
	swi	$r0, [$r9 + 4]

	ex9.it	33
	PUTS	.Lfstr_j32

.Ltest_jal:
	!	jal > 32
	! fix the entry in table
	lwi	$r7, [$r9 + 8]
	la	$r0, test_jal_call	! fix this address in
	srli	$r0, $r0, 1
	or	$r0, $r0, $r7
	swi	$r0, [$r9 + 8]

	movi	$r7, 1
	ex9.it	34
	beqz	$r7, .Ltest_jr
	PUTS	.Lfstr_jal

.Ltest_jr:
	!	jr > 32
	la	$r8, .Ldone
	ex9.it	35
	PUTS	.Lfstr_jr

	!	jral > 32

.Ldone:
	PUTS	.Lpstr
	EXIT	0	! Because endian is chagned,
			! it cannot properly restore registers.
	.size	main, .-main

.section	.rodata
.Lpstr:     .string "pass\n"
.Lfstr_n32: .string "fall: addi in ex9.it (>=32)\n"
.Lfstr_j32: .string "fail: j in ex9.it (>=32)\n"
.Lfstr_jal: .string "fail: jal in ex9.it (>=32)\n"
.Lfstr_jr:  .string "fail: jr in ex9.it (>=32)\n"
