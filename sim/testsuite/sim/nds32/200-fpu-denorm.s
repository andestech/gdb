# nds32 test J/JAL in ex9, expected to pass.
# mach:	 	all
# as:		-mbaseline=v3 -mfpu-freg=3 -mfpu-sp-ext -mfpu-dp-ext
# ld:		--defsym=_stack=0x3000000
# output:	pass\n

	.include "utils.inc"

	.data
	.align	3
d64:
	.word 0x00000001
	.word 0x36900000
	.word 0x4f800000
	.word 0x00000000

	.text
	.global	main
main:
	la	$r6, d64
	fldi	$fd0, [$r6 + 0]
	flsi	$fs3, [$r6 + 8]
	flsi	$fs2, [$r6 + 12]

	#
	# DNZ off
	#
	movi	$r0, 0
	fmtcsr	$r0

	# 64 to 32
	fd2s	$fs4, $fd0	! $fs4 is denorm, 0x00000001
	fmfsr	$r0, $fs4
	beqc	$r0, 1, 1f
	FAIL	1

1:
	# Multiply denorm by 128
	fmuls	$fs4, $fs1, $fs3
	fmfsr	$r0, $fs4
	bnez	$r0, 1f
	FAIL	2

1:
	# denorm + norm == norm
	fadds	$fs4, $fs1, $fs3
	fmfsr	$r0, $fs4
	fmfsr	$r1, $fs3
	beq	$r0, $r1, 1f
	FAIL	3

1:
	# denorm + zero == denorm
	fadds	$fs4, $fs1, $fs2
	fmfsr	$r0, $fs4
	fmfsr	$r1, $fs1
	beq	$r0, $r1, 1f
	FAIL	4

1:
	#
	# DNZ on
	#
	movi	$r0, 0x1000
	fmtcsr	$r0

	# 64 to 32
	fd2s	$fs4, $fd0
	fmfsr	$r0, $fs4	! $fs4 flush-to-zero
	beqc	$r0, 0, 1f
	FAIL	5

1:
	# Multiply denorm by 128
	fmuls	$fs4, $fs1, $fs3
	fmfsr	$r0, $fs4
	beqz	$r0, 1f
	FAIL	6

1:
	# denorm + norm == norm
	fadds	$fs4, $fs1, $fs3
	fmfsr	$r0, $fs4
	fmfsr	$r1, $fs3
	beq	$r0, $r1, 1f
	FAIL	7

1:
	# denorm + zero == zero (flush-to-zero)
	fadds	$fs4, $fs1, $fs2
	fmfsr	$r0, $fs4
	fmfsr	$r1, $fs2
	beq	$r0, $r1, 1f
	FAIL	8

1:
	PASS
	EXIT	0
