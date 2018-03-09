  vln.v   v1, (a1)
  vlnu.v  v2, (a2)
  vln8.v  v3, (a3)
  vlnu8.v v4, (a4)

  vln.v   v1, (a1), v0.t
  vlnu.v  v2, (a2), v0.t
  vln8.v  v3, (a3), v0.t
  vlnu8.v v4, (a4), v0.t

  vle4.v        v4, (a0)
# vle4.v        v8, (s0), v0.t

  vln.v         v4, (a0)
  vln.v         v8, (s0), v0.t

  vfwcvt.f.n.v  v0, v4
  vfwcvt.f.nu.v v1, v5
  vfwcvt.f.b.v  v2, v6
  vfwcvt.f.bu.v v3, v7

  vfwcvt.f.n.v  v0, v4, v0.t
  vfwcvt.f.nu.v v1, v5, v0.t
  vfwcvt.f.b.v  v2, v6, v0.t
  vfwcvt.f.bu.v v3, v7, v0.t

  vfpmadt.vf    v4, f1, v6
  vfpmadt.vf    v4, f2, v6, v0.t
  vfpmadb.vf    v5, f3, v7
  vfpmadb.vf    v5, f4, v7, v0.t
