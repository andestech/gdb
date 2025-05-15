# SIMD 16-bit Add/Subtract Instructions
add16 x28, x27, x10
radd16 x1, x18, x27
uradd16 x14, x2, x4
kadd16 x2, x5, x31
ukadd16 x16, x20, x12
sub16 x22, x20, x8
rsub16 x19, x16, x18
ursub16 x8, x5, x23
ksub16 x13, x29, x19
uksub16 x21, x5, x9
cras16 x7, x16, x5
rcras16 x3, x14, x5
urcras16 x12, x18, x23
kcras16 x29, x15, x10
ukcras16 x7, x30, x16
crsa16 x3, x15, x15
rcrsa16 x23, x25, x15
urcrsa16 x5, x28, x24
kcrsa16 x15, x16, x14
ukcrsa16 x22, x16, x21
stas16 x7, x10, x21
rstas16 x5, x31, x9
urstas16 x12, x1, x24
kstas16 x3, x25, x16
ukstas16 x29, x16, x16
stsa16 x15, x18, x4
rstsa16 x16, x8, x1
urstsa16 x8, x28, x31
kstsa16 x26, x27, x11
ukstsa16 x31, x1, x22

# SIMD 8-bit Add/Subtract Instructions
add8 x11, x23, x26
radd8 x15, x3, x7
uradd8 x26, x1, x1
kadd8 x27, x11, x31
ukadd8 x13, x5, x28
sub8 x7, x30, x2
rsub8 x0, x27, x11
ursub8 x11, x0, x23
ksub8 x19, x8, x30
uksub8 x2, x7, x3

# SIMD 16-bit Shift Instructions
sra16 x3, x8, x25
srai16 x7, x7, 1
sra16.u x18, x7, x16
srai16.u x29, x26, 2
srl16 x1, x20, x20
srli16 x14, x17, 0
srl16.u x12, x22, x16
srli16.u x16, x20, 2
sll16 x14, x23, x1
slli16 x10, x24, 15
ksll16 x19, x17, x26
kslli16 x2, x3, 13
kslra16 x19, x24, x31
kslra16.u x14, x2, x26

# SIMD 8-bit Shift Instructions
sra8 x4, x9, x4
srai8 x23, x2, 6
sra8.u x19, x3, x1
srai8.u x15, x30, 1
srl8 x16, x21, x18
srli8 x8, x22, 1
srl8.u x12, x13, x0
srli8.u x30, x21, 4
sll8 x19, x8, x11
slli8 x14, x16, 5
ksll8 x18, x7, x1
kslli8 x23, x7, 5
kslra8 x30, x31, x17
kslra8.u x8, x1, x3

# SIMD 16-bit Compare Instructions
cmpeq16 x16, x18, x28
scmplt16 x1, x1, x6
scmple16 x30, x29, x8
ucmplt16 x10, x8, x2
ucmple16 x2, x4, x3

# SIMD 8-bit Compare Instructions
cmpeq8 x12, x3, x17
scmplt8 x14, x19, x11
scmple8 x27, x28, x24
ucmplt8 x6, x8, x6
ucmple8 x28, x16, x12

# SIMD 16-bit Multiply Instructions
smul16 x22, x14, x12
smulx16 x8, x28, x12
umul16 x20, x18, x24
umulx16 x14, x4, x20
khm16 x24, x19, x23
khmx16 x1, x5, x29

# SIMD 8-bit Multiply Instructions
smul8 x2, x9, x2
smulx8 x9, x8, x22
umul8 x14, x18, x6
umulx8 x27, x29, x14
khm8 x22, x6, x29
khmx8 x12, x23, x6

# SIMD 16-bit Miscellaneous Instructions
smin16 x15, x16, x5
umin16 x2, x15, x0
smax16 x6, x31, x31
umax16 x2, x4, x14
sclip16 x19, x17, 5
uclip16 x19, x11, 9
kabs16 x4, x18
clrs16 x22, x26
clz16 x26, x18
clo16 x21, x10
swap16 x5, x17

# SIMD 8-bit Miscellaneous Instructions
smin8 x12, x7, x10
umin8 x0, x24, x21
smax8 x24, x3, x30
umax8 x30, x19, x18
kabs8 x24, x28
sclip8 x18, x10, 4
uclip8 x8, x30, 3
clrs8 x23, x9
clz8 x24, x15
clo8 x3, x3
swap8 x3, x15

# 8-bit Unpacking Instructions
sunpkd810 x10, x19
sunpkd820 x8, x2
sunpkd830 x21, x24
sunpkd831 x19, x18
sunpkd832 x17, x11
zunpkd810 x22, x13
zunpkd820 x27, x16
zunpkd830 x6, x11
zunpkd831 x8, x30
zunpkd832 x7, x31

# 16-bit Packing Instructions
pkbb16 x12, x13, x11
pkbt16 x14, x21, x2
pktb16 x28, x5, x0
pktt16 x24, x30, x5

# Signed MSW 32x32 Multiply and Add Instructions
smmul x20, x1, x7
smmul.u x31, x13, x8
kmmac x30, x2, x25
kmmac.u x1, x26, x19
kmmsb x5, x9, x1
kmmsb.u x23, x17, x9
kwmmul x7, x13, x3
kwmmul.u x24, x19, x16

# Signed MSW 32x16 Multiply and Add Instructions
smmwb x23, x2, x30
smmwb.u x15, x0, x25
smmwt x14, x28, x23
smmwt.u x15, x8, x31
kmmawb x31, x11, x20
kmmawb.u x27, x2, x2
kmmawt x2, x14, x19
kmmawt.u x26, x30, x18
kmmwb2 x29, x1, x6
kmmwb2.u x22, x5, x24
kmmwt2 x10, x13, x31
kmmwt2.u x5, x26, x26
kmmawb2 x26, x7, x6
kmmawb2.u x23, x7, x27
kmmawt2 x1, x24, x30
kmmawt2.u x3, x13, x26

# Signed 16-bit Multiply 32-bit Add/Subtract Instructions
smbb16 x21, x27, x25
smbt16 x30, x13, x28
smtt16 x28, x1, x7
kmda x18, x23, x14
kmxda x4, x0, x31
smds x8, x15, x30
smdrs x7, x8, x28
smxds x13, x13, x5
kmabb x22, x8, x31
kmabt x14, x3, x13
kmatt x3, x18, x7
kmada x31, x25, x30
kmaxda x24, x25, x24
kmads x5, x29, x12
kmadrs x12, x22, x12
kmaxds x11, x28, x0
kmsda x8, x11, x11
kmsxda x31, x7, x20

# Signed 16-bit Multiply 64-bit Add/Subtract Instructions
smal x12, x30, x5

# Partial-SIMD Miscellaneous Instructions
sclip32 x30, x30, 29
uclip32 x8, x29, 10
clrs32 x5, x12
clz32 x7, x18
clo32 x6, x13
pbsad x5, x21, x22
pbsada x23, x26, x19

# 8-bit Multiply with 32-bit Add Instructions
smaqa x25, x11, x7
umaqa x30, x16, x8
smaqa.su x19, x22, x16

# 64-bit Add/Subtract Instructions
add64 x26, x15, x14
radd64 x20, x21, x24
uradd64 x18, x5, x24
kadd64 x5, x1, x5
ukadd64 x20, x8, x13
sub64 x0, x15, x4
rsub64 x19, x1, x15
ursub64 x22, x4, x24
ksub64 x3, x17, x1
uksub64 x15, x4, x0

# 32-bit Multiply 64-bit Add/Subtract Instructions
smar64 x1, x27, x6
smsr64 x3, x23, x2
umar64 x21, x2, x28
umsr64 x17, x8, x22
kmar64 x0, x19, x1
kmsr64 x1, x4, x30
ukmar64 x3, x1, x7
ukmsr64 x18, x4, x0

# Signed 16-bit Multiply 64-bit Add/Subtract Instructions
smalbb x11, x5, x1
smalbt x0, x23, x19
smaltt x5, x10, x13
smalda x19, x16, x12
smalxda x10, x29, x1
smalds x28, x27, x26
smaldrs x12, x15, x11
smalxds x24, x14, x25
smslda x18, x3, x12
smslxda x17, x9, x0

# Non-SIMD Q15 saturation ALU Instructions
kaddh x10, x18, x31
ksubh x16, x25, x20
khmbb x25, x28, x20
khmbt x13, x4, x19
khmtt x5, x22, x14
ukaddh x11, x26, x27
uksubh x10, x4, x26

# Non-SIMD Q31 saturation ALU Instructions
kaddw x6, x5, x7
ukaddw x30, x31, x29
ksubw x1, x9, x16
uksubw x1, x23, x15
kdmbb x24, x16, x30
kdmbt x23, x15, x10
kdmtt x1, x21, x31
kslraw x14, x25, x3
kslraw.u x4, x22, x7
ksllw x1, x1, x11
kslliw x13, x27, 21
kdmabb x13, x29, x7
kdmabt x30, x5, x28
kdmatt x14, x16, x24
kabsw x7, x16

# 32-bit ComputationInstructions
raddw x9, x12, x17
uraddw x16, x7, x23
rsubw x27, x30, x22
ursubw x0, x8, x26
maxw x11, x30, x28
minw x3, x4, x8
mulr64 x24, x25, x23
mulsr64 x10, x8, x12

# OV (Overflow) flag Set/Clear Instructions
rdov x16
clrov

# Non-SIMD Miscellaneous Instructions
ave x7, x2, x7
sra.u x15, x14, x24
srai.u x14, x23, 7
bitrev x14, x22, x5
bitrevi x28, x21, 30
wext x19, x14, x13
wexti x1, x24, 30
bpick x23, x24, x22, x11
insb x20, x2, 1
maddr32 x17, x30, x12
msubr32 x7, x27, x5

# SIMD 32-bit Add/Subtract Instructions
add32 x7, x5, x30
radd32 x4, x9, x15
uradd32 x19, x18, x24
kadd32 x16, x31, x4
ukadd32 x2, x1, x1
sub32 x31, x28, x28
rsub32 x18, x30, x21
ursub32 x2, x21, x24
ksub32 x27, x11, x26
uksub32 x10, x13, x26
cras32 x10, x31, x10
rcras32 x5, x6, x10
urcras32 x23, x3, x15
kcras32 x27, x31, x28
ukcras32 x20, x12, x29
crsa32 x5, x15, x28
rcrsa32 x1, x6, x4
urcrsa32 x13, x17, x6
kcrsa32 x22, x25, x29
ukcrsa32 x19, x0, x25
stas32 x9, x7, x27
rstas32 x13, x17, x18
urstas32 x28, x15, x25
kstas32 x7, x7, x11
ukstas32 x28, x3, x9
stsa32 x27, x20, x22
rstsa32 x8, x25, x8
urstsa32 x11, x23, x6
kstsa32 x9, x13, x22
ukstsa32 x18, x5, x6

# (RV64 Only) SIMD 32-bit Shift Instructions
sra32 x8, x16, x30
srai32 x10, x1, 28
sra32.u x20, x17, x20
srai32.u x13, x16, 27
srl32 x12, x23, x5
srli32 x25, x23, 12
srl32.u x24, x5, x3
srli32.u x5, x3, 27
sll32 x20, x11, x8
slli32 x27, x19, 10
ksll32 x14, x23, x16
kslli32 x24, x0, 0
kslra32 x6, x11, x16
kslra32.u x12, x28, x26

# (RV64 Only) SIMD 32-bit Miscellaneous Instructions
smin32 x3, x27, x10
umin32 x18, x2, x18
smax32 x9, x11, x7
umax32 x6, x5, x16
kabs32 x27, x11

# (RV64 Only) SIMD Q15 saturating Multiply Instructions
khmbb16 x14, x0, x13
khmbt16 x5, x1, x23
khmtt16 x25, x18, x4
kdmbb16 x12, x24, x19
kdmbt16 x22, x8, x7
kdmtt16 x31, x18, x17
kdmabb16 x5, x21, x13
kdmabt16 x8, x31, x5
kdmatt16 x10, x9, x24

# (RV64 Only) 32-bit Multiply Instructions
smbb32 x24, x3, x20
smbt32 x16, x2, x4
smtt32 x0, x9, x10

# (RV64 Only) 32-bit Multiply & Add Instructions
kmabb32 x18, x4, x14
kmabt32 x6, x23, x29
kmatt32 x26, x3, x17

# (RV64 Only) 32-bit Parallel Multiply & Add Instructions
kmda32 x1, x4, x16
kmxda32 x10, x15, x12
kmada32 x0, x11, x11
kmaxda32 x4, x19, x12
kmads32 x24, x5, x7
kmadrs32 x28, x4, x29
kmaxds32 x28, x27, x25
kmsda32 x0, x16, x12
kmsxda32 x23, x27, x28
smds32 x3, x30, x23
smdrs32 x20, x1, x31
smxds32 x27, x17, x19

# (RV64 Only) Non-SIMD 32-bitShift Instructions
sraiw.u x7, x5, 7

# 32-bit Packing Instructions
pkbb32 x16, x26, x2
pkbt32 x28, x22, x2
pktb32 x10, x15, x29
pktt32 x13, x6, x0
