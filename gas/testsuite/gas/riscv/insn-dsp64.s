dsp64:
	# Table 25. (RV64 Only) SIMD 32-bit Add/Subtract Instructions (30)
	add32     r1, r2, r3
	radd32    r1, r2, r3
	uradd32   r1, r2, r3
	kadd32    r1, r2, r3
	ukadd32   r1, r2, r3
	sub32     r1, r2, r3
	rsub32    r1, r2, r3
	ursub32   r1, r2, r3
	ksub32    r1, r2, r3
	uksub32   r1, r2, r3
	cras32    r1, r2, r3
	rcras32   r1, r2, r3
	urcras32  r1, r2, r3
	kcras32   r1, r2, r3
	ukcras32  r1, r2, r3
	crsa32    r1, r2, r3
	rcrsa32   r1, r2, r3
	urcrsa32  r1, r2, r3
	kcrsa32   r1, r2, r3
	ukcrsa32  r1, r2, r3
	stas32    r1, r2, r3
	rstas32   r1, r2, r3
	urstas32  r1, r2, r3
	kstas32   r1, r2, r3
	ukstas32  r1, r2, r3
	stsa32    r1, r2, r3
	rstsa32   r1, r2, r3
	urstsa32  r1, r2, r3
	kstsa32   r1, r2, r3
	ukstsa32  r1, r2, r3

	# Table 26. (RV64 Only) SIMD 32-bit Shift Instructions (14)
	sra32     r1, r2, r3
	srai32    r1, r2, 5
	sra32.u   r1, r2, r3
	srai32.u  r1, r2, 5
	srl32     r1, r2, r3
	srli32    r1, r2, 5
	srl32.u   r1, r2, r3
	srli32.u  r1, r2, 5
	sll32     r1, r2, r3
	slli32    r1, r2, 5
	ksll32    r1, r2, r3
	kslli32   r1, r2, 5
	kslra32   r1, r2, r3
	kslra32.u r1, r2, r3

	# Table 27. (RV64 Only) SIMD 32-bit Miscellaneous Instructions (5)
	smin32    r1, r2, r3
	umin32    r1, r2, r3
	smax32    r1, r2, r3
	umax32    r1, r2, r3
	kabs32    r1, r2

	# Table 28. (RV64 Only) SIMD Q15 saturating Multiply Instructions (9)
	khmbb16   r1, r2, r3
	khmbt16   r1, r2, r3
	khmtt16   r1, r2, r3
	kdmbb16   r1, r2, r3
	kdmbt16   r1, r2, r3
	kdmtt16   r1, r2, r3
	kdmabb16  r1, r2, r3
	kdmabt16  r1, r2, r3
	kdmatt16  r1, r2, r3

	# Table 29. (RV64 Only) 32-bit Multiply Instructions (3)
	smbb32    r1, r2, r3
	smbt32    r1, r2, r3
	smtt32    r1, r2, r3

	# Table 30. (RV64 Only) 32-bit Multiply & Add Instructions (3)
	kmabb32   r1, r2, r3
	kmabt32   r1, r2, r3
	kmatt32   r1, r2, r3

	# Table 31. (RV64 Only) 32-bit Parallel Multiply & Add Instructions (12)
	kmda32    r1, r2, r3
	kmxda32   r1, r2, r3
	kmada32   r1, r2, r3
	kmaxda32  r1, r2, r3
	kmads32   r1, r2, r3
	kmadrs32  r1, r2, r3
	kmaxds32  r1, r2, r3
	kmsda32   r1, r2, r3
	kmsxda32  r1, r2, r3
	smds32    r1, r2, r3
	smdrs32   r1, r2, r3
	smxds32   r1, r2, r3

	# Table 32. (RV64 Only) Non-SIMD 32-bit Shift Instructions (1)
	sraiw.u   r1, r2, 5

	# Table 33. 32-bit Packing Instructions (4)
	pkbb32    r1, r2, r3
	pkbt32    r1, r2, r3
	pktt32    r1, r2, r3
	pktb32    r1, r2, r3
