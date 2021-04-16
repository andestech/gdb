# nds32 test J/JAL in ex9, expected to pass.
# mach:	 	all
# as:		-mbaseline=v3
# ld:		--defsym=_stack=0x3000000
# output:	pass\n

	.include "utils.inc"

	.text
	.global	main
main:

.Ldone:
	# 0000 1111 1100 0011 1010 0101 1XXX XXXX
	li	$r8, 0x0fc3a580
	# 0000 1111 1100 0011 0101 1010 01XX XXXX
	li	$r9, 0x0fc35a70

	#
	# byte
	#
	seb	$r0, $r8
	li	$r1, 0xffffff80
	beq	$r0, $r1, 1f
	FAIL	1

1:
	zeb	$r0, $r8
	li	$r1, 0x00000080
	beq	$r0, $r1, 1f
	FAIL	2

1:
	seb	$r0, $r9
	li	$r1, 0x00000070
	beq	$r0, $r1, 1f
	FAIL	3

1:
	zeb	$r0, $r9
	li	$r1, 0x00000070
	beq	$r0, $r1, 1f
	FAIL	4

	#
	# half
	#
	seh	$r0, $r8
	li	$r1, 0xffffa580
	beq	$r0, $r1, 1f
	FAIL	1

1:
	zeh	$r0, $r8
	li	$r1, 0x0000a580
	beq	$r0, $r1, 1f
	FAIL	2

1:
	seh	$r0, $r9
	li	$r1, 0x00005a70
	beq	$r0, $r1, 1f
	FAIL	3

1:
	zeh	$r0, $r9
	li	$r1, 0x00005a70
	beq	$r0, $r1, 1f
	FAIL	4

1:
	PASS
	EXIT	0
