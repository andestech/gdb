dsp:
	# Table 1. SIMD 16-bit Add/Subtract Instructions (30)
	add16     r1, r2, r3
	radd16    r1, r2, r3
	uradd16   r1, r2, r3
	kadd16    r1, r2, r3
	ukadd16   r1, r2, r3
	sub16     r1, r2, r3
	rsub16    r1, r2, r3
	ursub16   r1, r2, r3
	ksub16    r1, r2, r3
	uksub16   r1, r2, r3
	cras16    r1, r2, r3
	rcras16   r1, r2, r3
	urcras16  r1, r2, r3
	kcras16   r1, r2, r3
	ukcras16  r1, r2, r3
	crsa16    r1, r2, r3
	rcrsa16   r1, r2, r3
	urcrsa16  r1, r2, r3
	kcrsa16   r1, r2, r3
	ukcrsa16  r1, r2, r3
	stas16    r1, r2, r3
	rstas16   r1, r2, r3
	urstas16  r1, r2, r3
	kstas16   r1, r2, r3
	ukstas16  r1, r2, r3
	stsa16    r1, r2, r3
	rstsa16   r1, r2, r3
	urstsa16  r1, r2, r3
	kstsa16   r1, r2, r3
	ukstsa16  r1, r2, r3

	# Table 2. SIMD 8-bit Add/Subtract Instructions (10)
	add8      r1, r2, r3
	radd8     r1, r2, r3
	uradd8    r1, r2, r3
	kadd8     r1, r2, r3
	ukadd8    r1, r2, r3
	sub8      r1, r2, r3
	rsub8     r1, r2, r3
	ursub8    r1, r2, r3
	ksub8     r1, r2, r3
	uksub8    r1, r2, r3

	# Table 3. SIMD 16-bit Shift Instructions (14)
	sra16     r1, r2, r3
	srai16    r1, r2, 4
	sra16.u   r1, r2, r3
	srai16.u  r1, r2, 4
	srl16     r1, r2, r3
	srli16    r1, r2, 4
	srl16.u   r1, r2, r3
	srli16.u  r1, r2, 4
	sll16     r1, r2, r3
	slli16    r1, r2, 4
	ksll16    r1, r2, r3
	kslli16   r1, r2, 4
	kslra16   r1, r2, r3
	kslra16.u r1, r2, r3

	# Table 4. SIMD 8-bit Shift Instructions (14)
	sra8      r1, r2, r3
	srai8     r1, r2, 3
	sra8.u    r1, r2, r3
	srai8.u   r1, r2, 4
	srl8      r1, r2, r3
	srli8     r1, r2, 3
	srl8.u    r1, r2, r3
	srli8.u   r1, r2, 4
	sll8      r1, r2, r3
	slli8     r1, r2, 3
	ksll8     r1, r2, r3
	kslli8    r1, r2, 3
	kslra8    r1, r2, r3
	kslra8.u  r1, r2, r3

	# Table 5. SIMD 16-bit Compare Instructions (5)
	cmpeq16   r1, r2, r3
	scmplt16  r1, r2, r3
	scmple16  r1, r2, r3
	ucmplt16  r1, r2, r3
	ucmple16  r1, r2, r3

	# Table 6. SIMD 8-bit Compare Instructions (5)
	cmpeq8    r1, r2, r3
	scmplt8   r1, r2, r3
	scmple8   r1, r2, r3
	ucmplt8   r1, r2, r3
	ucmple8   r1, r2, r3

	# Table 7. SIMD 16-bit Miscellaneous Instructions (12)
	smin16    r1, r2, r3
	umin16    r1, r2, r3
	smax16    r1, r2, r3
	umax16    r1, r2, r3
	sclip16   r1, r2, 4
	uclip16   r1, r2, 4
	khm16     r1, r2, r3
	khmx16    r1, r2, r3
	kabs16    r1, r2
	clrs16    r1, r2
	clz16     r1, r2
	clo16     r1, r2

	# Table 8. SIMD 8-bit Miscellaneous Instructions (12)
	smin8     r1, r2, r3
	umin8     r1, r2, r3
	smax8     r1, r2, r3
	umax8     r1, r2, r3
	khm8      r1, r2, r3
	khmx8     r1, r2, r3
	kabs8     r1, r2
	sclip8    r1, r2, 3
	uclip8    r1, r2, 3
	clrs8     r1, r2
	clz8      r1, r2
	clo8      r1, r2

	# Table 9. 8-bit Unpacking Instructions (10)
	sunpkd810 r1, r2
	sunpkd820 r1, r2
	sunpkd830 r1, r2
	sunpkd831 r1, r2
	sunpkd832 r1, r2
	zunpkd810 r1, r2
	zunpkd820 r1, r2
	zunpkd830 r1, r2
	zunpkd831 r1, r2
	zunpkd832 r1, r2

	# Table 10. 16-bit Packing Instructions (4)
	pkbb16    r1, r2, r3
	pkbt16    r1, r2, r3
	pktb16    r1, r2, r3
	pktt16    r1, r2, r3

	# Table 11. Signed MSW 32x32 Multiply and Add Instructions (8)
	smmul     r1, r2, r3
	smmul.u   r1, r2, r3
	kmmac     r1, r2, r3
	kmmac.u   r1, r2, r3
	kmmsb     r1, r2, r3
	kmmsb.u   r1, r2, r3
	kwmmul    r1, r2, r3
	kwmmul.u  r1, r2, r3

	# Table 12. Signed MSW 32x16 Multiply and Add Instructions (16)
	smmwb     r1, r2, r3
	smmwb.u   r1, r2, r3
	smmwt     r1, r2, r3
	smmwt.u   r1, r2, r3
	kmmawb    r1, r2, r3
	kmmawb.u  r1, r2, r3
	kmmawt    r1, r2, r3
	kmmawt.u  r1, r2, r3
	kmmwb2    r1, r2, r3
	kmmwb2.u  r1, r2, r3
	kmmwt2    r1, r2, r3
	kmmwt2.u  r1, r2, r3
	kmmawb2   r1, r2, r3
	kmmawb2.u r1, r2, r3
	kmmawt2   r1, r2, r3
	kmmawt2.u r1, r2, r3

	# Table 13. Signed 16-bit Multiply 32-bit Add/Subtract Instructions (18)
	smbb16    r1, r2, r3
	smbt16    r1, r2, r3
	smtt16    r1, r2, r3
	kmda      r1, r2, r3
	kmxda     r1, r2, r3
	smds      r1, r2, r3
	smdrs     r1, r2, r3
	smxds     r1, r2, r3
	kmabb     r1, r2, r3
	kmabt     r1, r2, r3
	kmatt     r1, r2, r3
	kmada     r1, r2, r3
	kmaxda    r1, r2, r3
	kmads     r1, r2, r3
	kmadrs    r1, r2, r3
	kmaxds    r1, r2, r3
	kmsda     r1, r2, r3
	kmsxda    r1, r2, r3

	# Table 14. Signed 16-bit Multiply 64-bit Add/Subtract Instructions (1)
	smal      r2, r4, r6

	# Table 15. Partial-SIMD Miscellaneous Instructions (7)
	sclip32   r1, r2, 5
	uclip32   r1, r2, 5
	clrs32    r1, r2
	clz32     r1, r2
	clo32     r1, r2
	pbsad     r1, r2, r3
	pbsada    r1, r2, r3

	# Table 16. 8-bit Multiply with 32-bit Add Instructions (3)
	smaqa     r1, r2, r3
	umaqa     r1, r2, r3
	smaqa.su  r1, r2, r3

	# Table 17. 64-bit Add/Subtract Instructions (10)
	add64     r2, r4, r6
	radd64    r2, r4, r6
	uradd64   r2, r4, r6
	kadd64    r2, r4, r6
	ukadd64   r2, r4, r6
	sub64     r2, r4, r6
	rsub64    r2, r4, r6
	ursub64   r2, r4, r6
	ksub64    r2, r4, r6
	uksub64   r2, r4, r6

	# Table 18. 32-bit Multiply 64-bit Add/Subtract Instructions (8)
	smar64    r2, r4, r6
	smsr64    r2, r4, r6
	umar64    r2, r4, r6
	umsr64    r2, r4, r6
	kmar64    r2, r4, r6
	kmsr64    r2, r4, r6
	ukmar64   r2, r4, r6
	ukmsr64   r2, r4, r6

	# Table 19. Signed 16-bit Multiply 64-bit Add/Subtract Instructions (10)
	smalbb    r2, r4, r6
	smalbt    r2, r4, r6
	smaltt    r2, r4, r6
	smalda    r2, r4, r6
	smalxda   r2, r4, r6
	smalds    r2, r4, r6
	smaldrs   r2, r4, r6
	smalxds   r2, r4, r6
	smslda    r2, r4, r6
	smslxda   r2, r4, r6

	# Table 20. Non-SIMD Q15 saturation ALU Instructions (7)
	kaddh     r1, r2, r3
	ksubh     r1, r2, r3
	khmbb     r1, r2, r3
	khmbt     r1, r2, r3
	khmtt     r1, r2, r3
	ukaddh    r1, r2, r3
	uksubh    r1, r2, r3

	# Table 21. Non-SIMD Q31 saturation ALU Instructions (15)
	kaddw     r1, r2, r3
	ukaddw    r1, r2, r3
	ksubw     r1, r2, r3
	uksubw    r1, r2, r3
	kdmbb     r1, r2, r3
	kdmbt     r1, r2, r3
	kdmtt     r1, r2, r3
	kslraw    r1, r2, r3
	kslraw.u  r1, r2, r3
	ksllw     r1, r2, r3
	kslliw    r1, r2, 5
	kdmabb    r1, r2, r3
	kdmabt    r1, r2, r3
	kdmatt    r1, r2, r3
	kabsw     r1, r2

	# Table 22. 32-bit Add/Sub Instructions (4)
	raddw     r1, r2, r3
	uraddw    r1, r2, r3
	rsubw     r1, r2, r3
	ursubw    r1, r2, r3

	# Table 23. OV (Overflow) flag Set/Clear Instructions (2)
	csrr      r1, satp	#rdov     r1
	csrrci    r1, satp, 1	#clrov

	# Table 24. Non-SIMD Miscellaneous Instructions (9)
	ave       r1, r2, r3
	sra.u     r1, r2, r3
	srai.u    r1, r2, 5
	bitrev    r1, r2, r3
	bitrevi   r1, r2, 5
	wext      r1, r2, r3
	wexti     r1, r2, 5
	bpick     r1, r2, r3, r4
	insb      r1, r2, 2

	# Table 34. ZOL Mechanism Instructions (2)
	mtlbi     16
	mtlei     16
