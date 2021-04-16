# nds32 test J/JAL in ex9, expected to pass.
# mach:	 	all
# as:		-mbaseline=v3
# ld:		--defsym=_stack=0x3000000
# output:	pass\n

	.include "utils.inc"

.data
	.align 2
WORD:
	.byte	0x81
	.byte	0x82
HALF:
	.byte	0x83
BYTE:
	.byte	0x84

	.text
	.global	main
main:

.Ldone:
	! $r5 is case counter for sanity check (current 20)
	movi	$r5, 0

	! Set to big endian
	mfsr	$r3, $psw
	ori	$r3, $r3, 32	!psw.be
	mtsr	$r3, $psw

	movi	$r4, 0

	! HALF
	la	$r3, HALF
	move	$r2, 0x8384
	addi	$r5, $r5, 1
	lhi	$r1, [$r3 + 0]
	beq	$r1, $r2, 1f
	FAIL	1
1:
	addi	$r5, $r5, 1
	lhi333	$r1, [$r3 + 0]
	beq	$r1, $r2, 1f
	FAIL	2
1:
	addi	$r5, $r5, 1
	lh	$r1, [$r3 + $r4]
	beq	$r1, $r2, 1f
	FAIL	3
1:

	move	$r2, 0xffff8384
	addi	$r5, $r5, 1
	lhsi	$r1, [$r3 + 0]
	beq	$r1, $r2, 1f
	FAIL	4
1:
	addi	$r5, $r5, 1
	lhs	$r1, [$r3 + $r4]
	beq	$r1, $r2, 1f
	FAIL	5
1:


	! BYTE
	move	$r2, 0x84
	la	$r3, BYTE
	addi	$r5, $r5, 1
	lbi	$r1, [$r3 + 0]
	beq	$r1, $r2, 1f
	FAIL	6
1:
	addi	$r5, $r5, 1
	lbi333	$r1, [$r3 + 0]
	beq	$r1, $r2, 1f
	FAIL	7
1:
	addi	$r5, $r5, 1
	lb	$r1, [$r3 + $r4]
	beq	$r1, $r2, 1f
	FAIL	8
1:

	move	$r2, 0xffffff84
	addi	$r5, $r5, 1
	lbsi	$r1, [$r3 + 0]
	beq	$r1, $r2, 1f
	FAIL	9
1:
	addi	$r5, $r5, 1
	lbs	$r1, [$r3 + $r4]
	beq	$r1, $r2, 1f
	FAIL	0xa
1:

	! Set to little endian
	mfsr	$r3, $psw
	li	$r2, ~32
	and	$r3, $r3, $r2	!psw.be
	mtsr	$r3, $psw

	! HALF
	move	$r2, 0x8483
	la	$r3, HALF
	addi	$r5, $r5, 1
	lhi	$r1, [$r3 + 0]
	beq	$r1, $r2, 1f
	FAIL	0xb
1:
	addi	$r5, $r5, 1
	lhi333	$r1, [$r3 + 0]
	beq	$r1, $r2, 1f
	FAIL	0xc
1:
	addi	$r5, $r5, 1
	lh	$r1, [$r3 + $r4]
	beq	$r1, $r2, 1f
	FAIL	0xd
1:

	move	$r2, 0xffff8483
	addi	$r5, $r5, 1
	lhsi	$r1, [$r3 + 0]
	beq	$r1, $r2, 1f
	FAIL	0xe
1:
	addi	$r5, $r5, 1
	lhs	$r1, [$r3 + $r4]
	beq	$r1, $r2, 1f
	FAIL	0xf
1:

	! BYTE
	move	$r2, 0x84
	la	$r3, BYTE
	addi	$r5, $r5, 1
	lbi	$r1, [$r3 + 0]
	beq	$r1, $r2, 1f
	FAIL	0x10
1:
	addi	$r5, $r5, 1
	lbi333	$r1, [$r3 + 0]
	beq	$r1, $r2, 1f
	FAIL	0x11
1:
	addi	$r5, $r5, 1
	lb	$r1, [$r3 + $r4]
	beq	$r1, $r2, 1f
	FAIL	0x12
1:

	move	$r2, 0xffffff84
	addi	$r5, $r5, 1
	lbsi	$r1, [$r3 + 0]
	beq	$r1, $r2, 1f
	FAIL	0x13
1:
	addi	$r5, $r5, 1
	lbs	$r1, [$r3 + $r4]
	beq	$r1, $r2, 1f
	FAIL	0x14
1:



	addi	$r5, $r5, -20
	beqz	$r5, 1f
	FAIL	0x15
1:
	PASS
	EXIT	0
