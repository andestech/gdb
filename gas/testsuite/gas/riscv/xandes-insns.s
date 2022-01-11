# Branch Instructions

bbc x13, 2, 1
bbc x5, 0, 6

bbs x30, 1, 7
bbs x17, 3, 3

beqc x28, 4, 1
bnec x24, 3, 3

bfos x14, x10, 5, 5
bfoz x2, x30, 6, 4


# Load Effective Address Instructions

lea.h x7, x10, x14
lea.w x7, x10, x11
lea.d x25, x18, x12
lea.b.ze x28, x3, x26
lea.h.ze x16, x3, x23
lea.w.ze x30, x23, x26
lea.d.ze x27, x26, x10


# GP-Relative Instructions

addigp x27, 0
lbgp x26, 48
lbugp x21, 80
lhgp x1, 8
lhugp x24, 40
lwgp x23, 24
lwugp x28, 72
ldgp x9, 120
sbgp x16, 0
shgp x8, 0
swgp x26, 64
sdgp x6, 72


# String Processing Instructions

ffb x20, x7, x18
ffzmism x3, x27, x5
ffmism x23, x19, x10
flmism x8, x28, x25


# Code Dense Instructions

exec.it 25
ex9.it 7
