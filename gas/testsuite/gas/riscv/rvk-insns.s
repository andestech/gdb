    flh         f1, 0x123(a0)
    flh         f2, any, t0

    fsh         f3, 0x456(a1)
    fsh         f4, any, t1

    fmv.x.h     t0, f5
    fmv.h.x     f6, t1

    fmv.h       f7, f8
    fneg.h      f9, f10
    fabs.h      f11, f12
    fsgnj.h     f13, f14, f15
    fsgnjn.h    f16, f17, f18
    fsgnjx.h    f19, f20, f21

    fadd.h      f22, f23, f24
    fadd.h      f25, f26, f27, rne
    fsub.h      f28, f29, f30
    fsub.h      f31, f0, f1, rtz
    fmul.h      f2, f3, f4
    fmul.h      f5, f6, f7, rdn
    fdiv.h      f8, f9, f10
    fdiv.h      f11, f12, f13, rup
    fsqrt.h     f14, f15
    fsqrt.h     f17, f18, rmm
    fmin.h      f20, f21, f22
    fmax.h      f23, f24, f25

    fmadd.h     f26, f27, f28, f29
    fmadd.h     f29, f30, f31, f0, dyn
    fnmadd.h    f0, f1, f2, f3
    fnmadd.h    f3, f4, f5, f6, rne
    fmsub.h     f6, f7, f8, f9
    fmsub.h     f9, f10, f11, f12, rtz
    fnmsub.h    f12, f13, f14, f15
    fnmsub.h    f15, f16, f17, f18, rdn

    fcvt.w.h    r1, f1
    fcvt.w.h    r2, f2, rup
    fcvt.wu.h   r3, f3
    fcvt.wu.h   r4, f4, rmm
    fcvt.h.w    f5, r5
    fcvt.h.w    f6, r6, dyn
    fcvt.h.wu   f7, r7
    fcvt.h.wu   f8, r8, rne

    fclass.h    r9, f9

    feq.h       r10, f10, f11
    flt.h       r11, f12, f13
    fle.h       r12, f14, f15
    fgt.h       r13, f16, f17
    fge.h       r14, f18, f19

    fcvt.l.h    r1, f1
    fcvt.l.h    r2, f2, rup
    fcvt.lu.h   r3, f3
    fcvt.lu.h   r4, f4, rmm
    fcvt.h.l    f5, r5
    fcvt.h.l    f6, r6, dyn
    fcvt.h.lu   f7, r7
    fcvt.h.lu   f8, r8, rne

    fcvt.s.h    f0, f1
    fcvt.h.s    f2, f3
    fcvt.h.s    f4, f5, rtz

    fcvt.d.h    f6, f7
    fcvt.h.d    f8, f9
    fcvt.h.d    f10, f11, rdn

any:
