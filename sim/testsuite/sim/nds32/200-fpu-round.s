# nds32 test J/JAL in ex9, expected to pass.
# mach:	 	all
# as:		-mbaseline=v3 -mfpu-freg=3 -mfpu-sp-ext -mfpu-dp-ext
# ld:		--defsym=_stack=0x3000000
# output:	pass\n

	.include "utils.inc"

	.data
three:	.word	0x40400000

dividends:
	.word	0x3f800000, 0xbf800000	! 1, -1
	.word	0x40a00000, 0xc0a00000	! 5, -5

quotients:
	#	nearest     pi          mi        zero
_333:	.word	0x3eaaaaab, 0x3eaaaaab, 0x3eaaaaaa, 0x3eaaaaaa
	.word	0xbeaaaaab, 0xbeaaaaaa, 0xbeaaaaab, 0xbeaaaaaa
_666:	.word	0x3fd55555, 0x3fd55556, 0x3fd55555, 0x3fd55555
	.word	0Xbfd55555, 0Xbfd55555, 0Xbfd55556, 0Xbfd55555

	.text
	.global	main
main:
	# $r6: iterator
	movi	$r6, 0
	# $r7: dividends[]
	la	$r7, dividends
	# $r8: quotients[]
	la	$r8, quotients
	# $fs0: 3.0
	la	$r0, three
	flsi	$fs0, [$r0 + 0]

	b	.Lend
.Loop:
	fls	$fs1, [$r7 + $r6 << 2]
	add_slli $r9, $r8, $r6, 4

	# Round towards Nearest Even
	fmfcsr	$r0
	movi	$r0, 0
	fmtcsr	$r0
	fdivs	$fs2, $fs1, $fs0
	fmfsr	$r1, $fs2
	lwi	$r0, [$r9 + 0]
	beq	$r0, $r1, 1f
	! fail
	slli	$r0, $r6, 2
	addi	$r0, $r0, 0
	bal	result

1:
	# Round towards Plus Infinity
	fmfcsr	$r0
	movi	$r0, 1
	fmtcsr	$r0
	fdivs	$fs2, $fs1, $fs0
	fmfsr	$r1, $fs2
	lwi	$r0, [$r9 + 4]
	beq	$r0, $r1, 1f
	! fail
	slli	$r0, $r6, 2
	addi	$r0, $r0, 1
	bal	result

1:
	# Round towards Minus Infinity
	fmfcsr	$r0
	movi	$r0, 2
	fmtcsr	$r0
	fdivs	$fs2, $fs1, $fs0
	fmfsr	$r1, $fs2
	lwi	$r0, [$r9 + 8]
	beq	$r0, $r1, 1f
	! fail
	slli	$r0, $r6, 2
	addi	$r0, $r0, 2
	bal	result

1:
	# Round towards Zero
	fmfcsr	$r0
	movi	$r0, 3
	fmtcsr	$r0
	fdivs	$fs2, $fs1, $fs0
	fmfsr	$r1, $fs2
	lwi	$r0, [$r9 + 12]
	beq	$r0, $r1, 1f
	! fail
	slli	$r0, $r6, 2
	addi	$r0, $r0, 3
	bal	result

1:
	addi	$r6, $r6, 1
.Lend:
	bnec	$r6, 4, .Loop

	PASS
	EXIT	0
