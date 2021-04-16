! _start: General entry potiner. It setups $sp.
!   swap: Swap a word in $r0 and return in $r0.
! result: Output "pass" if $r0 == 0; otherwise output "fail 0x##".

.macro	FAIL	code
	movi	$r0, \code
	bal	result
.endm

.macro	PASS
	movi	$r0, 0
	bal	result
.endm

.macro	EXIT	code
	li	$r0, \code
	syscall	1
.endm

.macro PUTS	label
	la	$r0, \label
	bal	puts
.endm

.section	.rodata
	.align 2
LPASS_STR:
	.string	"pass\n"

	.align 2
LFAIL_STR:
	.string "fail: 0x##\n"

.text
	.align	2
	.globl	swap
	.type	swap, @function
swap:
	wsbh    $r0, $r0
	rotri   $r0,$r0,#0x10
	ret
	.size	swap, .-swap

	! if $r0 = 0
	!     pass
	! else
	!     fail + id
	.align 2
	.global	result
	.type	result, @function
result:
	smw.adm	$r6,[$sp],$r14,0xe
	move	$r6, $r0
	bnez	$r0, 1f
	la	$r1, LPASS_STR
	b	2f
1:
	la	$r1, LFAIL_STR

	andi	$r2, $r0, 0xf
	slti	$r3, $r2, 10
	bnez	$r3, 3f
	addi	$r2, $r2, 55
	b	4f
3:
	addi	$r2, $r2, 48
4:
	sbi	$r2, [$r1 + 9]

	andi	$r2, $r0, 0xf0
	srli	$r2, $r2, 4
	slti	$r3, $r2, 10
	bnez	$r3, 3f
	addi	$r2, $r2, 55
	b	4f
3:
	addi	$r2, $r2, 48
4:
	sbi	$r2, [$r1 + 8]

2:
	move	$r0, $r1
	bal	puts
	move	$r0, $r6

	lmw.bim	$r6,[$sp],$r14,0xe
	ret

	.size	result, .-result

	.align 2
	.global	puts
	.type	puts, @function
puts:
	ori	$r1, $r0, 0
	movi	$r2, 0
1:
	lbi.bi	$r3, [$r0], 1
	beqz	$r3, 2f
	addi	$r2, $r2, 1
	b	1b
2:
	movi	$r0, 1
	syscall 5
	ret
	.size	puts, .-puts

	.align 2
	.global	_start
	.type	_start, @function
_start:
	la	$sp, _stack
	bal	main
	syscall	1
	.size	_start, .-_start
