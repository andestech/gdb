# nds32 test sanity, expected to pass.
# mach:		all
# as:		-mbaseline=v3
# ld:		--defsym=_stack=0x3000000
# output:	pass\n

	.include "utils.inc"

	.text
	.global main
main:
	smw.adm $sp, [$sp], $sp, 10

	movi    $r7, 0
.L0:
	! test fall through
	addi    $r7, $r7, 1
	ifret
	addi    $r7, $r7, -1

	beqz	$r7, 1f
	la	$r0, .Lfstr
1:
	la	$r0, .Lpstr
	bal	puts

	movi	$r0, 0
	lmw.bim $sp, [$sp], $sp, 10
	ret

.data
	.align 2
.Lpstr:
	.string "pass\n"
.Lfstr:
	.string "fail to tall-through ifret\n"
