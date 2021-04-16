# nds32 test J/JAL in ex5.it (index < 32), expected to pass.
# mach:		all
# as:		-mbaseline=v3
# ld:		--defsym=_stack=0x3000000
# output:	pass\n

	.include "utils.inc"

.section	.ex9.itable, "a"
	.align	2
.LITB0:		addi	$r7, $r7, 13
.LITB_J:	j	.LITB_J
.LITB_JAL:	jal	.LITB_JAL
.LITB_JR:	jr	$r8
.LITB_BEQZ:	beqz	$r8, .LITB_BEQZ

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

	! Load ITB table
	la	$r9, .LITB0	! address of ITB entry 0
	mtusr	$r9, $ITB


	!	normal instruction >= 32
	movi	$r7, 17
	ex9.it	0
	addi	$r7, $r7, -30

	beqz	$r7, .Ltest_j32
	PUTS	.Lfstr_n32	! FAIL: addi in ex5.it

.Ltest_j32:
	! relocate the entry in table
	l.w	$r7, .LITB_J
	la	$r0, .Ltest_jal	! fix this address in
	srli	$r0, $r0, 1
	or	$r0, $r0, $r7
	s.w	$r0, .LITB_J

	ex9.it	0x1		! j  .Ltest_jal
	PUTS	.Lfstr_j32	! FAIL: j in ex5.it


.Ltest_jal:
	! relocate the entry in table
	l.w	$r7, .LITB_JAL
	la	$r0, test_jal_call	! fix this address in
	srli	$r0, $r0, 1
	or	$r0, $r0, $r7
	s.w	$r0, .LITB_JAL

	movi	$r7, 1
	ex9.it	0x2			! test_jal_call for $r7--
	beqz	$r7, .Ltest_jr
	PUTS	.Lfstr_jal		! jal .Ltest_jr

.Ltest_jr:
	la	$r8, .Ltest_beqz0
	ex9.it	0x3			! jr $r8 (.Ldone)
	PUTS	.Lfstr_jr
	EXIT	1

.Ltest_beqz0:
	! test 32-bit instruction fall-through in ex9
	movi	$r8, 13
	ex9.it	0x4	! beqz  $r8, .LITB_BEQZ
	addi45	$r8, 1	! If it fall-through incorrectly,
			! this instruction will be skipped.
			! ($pc + 4 instead of $pc + 2)
	beqc	$r8, 14, .Ltest_beqz1
	PUTS	.Lfstr_beq0

.Ltest_beqz1:
	! test 32-bit instruction branch in ex9

	! relocate the entry in table
	l.w	$r7, .LITB_BEQZ
	l.w	$r0, BR			! fix this address in
	srli	$r0, $r0, 1
	or	$r0, $r0, $r7
	s.w	$r0, .LITB_BEQZ

	movi	$r8, 0
.LBEQZ_S:
	ex9.it	0x4	! beqz  $r8, .LITB_BEQZ
	addi45	$r8, 1	! Padding for preventing incorrectly fall-through.
	PUTS	.Lfstr_beq1
.LBEQZ_D:
	nop

.Ldone:
	PUTS	.Lpstr
	EXIT	0	! Because endian is chagned,
			! it cannot properly restore registers.

	.size	main, .-main

.section	.rodata
            ! assume the range is very small and access as big-endian.
BR:         .byte   0x00,0x00,0x00,.LBEQZ_D - .LBEQZ_S
.Lpstr:     .string "pass\n"
.Lfstr_n32: .string "fall: addi in ex9.it (<32)\n"
.Lfstr_j32: .string "fail: j in ex9.it (<32)\n"
.Lfstr_jal: .string "fail: jal in ex9.it (<32)\n"
.Lfstr_jr:  .string "fail: jr in ex9.it (<32)\n"
.Lfstr_beq0:.string "fail: beqz in ex9.it - fall-through (<32)\n"
.Lfstr_beq1:.string "fail: beqz in ex9.it - branch (<32)\n"
