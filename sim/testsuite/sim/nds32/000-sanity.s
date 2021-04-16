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

	la	$r0, .Lstring
	bal	puts
	movi	$r0, 0

	lmw.bim $sp, [$sp], $sp, 10
	ret

.Lstring:
	.string "pass\n"
