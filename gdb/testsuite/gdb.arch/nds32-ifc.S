.text

	.align	2
	.global	foo
foo:
	smw.adm	$r6, [$sp], $r7, 0
.Lcommon0:
	movi	$r6, 0		! FAIL MARKER
	ifret
	lmw.bim $r6, [$sp], $r7, 0
	ret

	.align	2
	.global	main
main:
	smw.adm	$r6, [$sp], $r7, 0
	nop
	movi	$r6, 10
	ifcall	.Lcommon0
	mov55	$r0, $r6	! PASS MARKER
	lmw.bim $r6, [$sp], $r7, 0
	ret
