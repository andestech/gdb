/* Bit operation for riscv crypto extension*/

#define zext32(x) ((uint64_t) (uint32_t) (x))
#define sext32_xlen(x) (uint32_t) (int32_t) (x)
#define sext_xlen(x) (((int64_t) (x) << (xlen - 32)) >> (xlen - 32))

/**
 * rol32 - rotate a 32-bit value left
 * @word: value to rotate
 * @shift: bits to roll
 */
static inline uint32_t
rol32 (uint32_t word, unsigned int shift)
{
  return (word << shift) | (word >> ((32 - shift) & 31));
}

/**
 * ror32 - rotate a 32-bit value right
 * @word: value to rotate
 * @shift: bits to roll
 */
static inline uint32_t
ror32 (uint32_t word, unsigned int shift)
{
  return (word >> shift) | (word << ((32 - shift) & 31));
}

/**
 * rol64 - rotate a 64-bit value left
 * @word: value to rotate
 * @shift: bits to roll
 */
static inline uint64_t
rol64 (uint64_t word, unsigned int shift)
{
  return (word << shift) | (word >> ((64 - shift) & 63));
}

/**
 * ror64 - rotate a 64-bit value right
 * @word: value to rotate
 * @shift: bits to roll
 */
static inline uint64_t
ror64 (uint64_t word, unsigned int shift)
{
  return (word >> shift) | (word << ((64 - shift) & 63));
}

static uint64_t
dup_const (unsigned vece, uint64_t c)
{
  switch (vece)
    {
    case 0:
      return 0x0101010101010101ull * (uint8_t) c;
    case 1:
      return 0x0001000100010001ull * (uint16_t) c;
    case 2:
      return 0x0000000100000001ull * (uint32_t) c;
    default:
      return c;
    }
}

#define dup_const(VECE, C)                                                     \
  (__builtin_constant_p (VECE)                                                 \
     ? ((VECE) == 0                                                            \
	  ? 0x0101010101010101ull * (uint8_t) (C)                              \
	  : (VECE) == 1 ? 0x0001000100010001ull * (uint16_t) (C)               \
			: (VECE) == 2 ? 0x0000000100000001ull * (uint32_t) (C) \
				      : (uint64_t) (C))                        \
     : dup_const (VECE, C))

static const uint64_t shuf_masks[]
  = {dup_const (0, 0x44), dup_const (0, 0x30), dup_const (1, 0x0F00),
     dup_const (2, 0xF0000), dup_const (3, 0xFFFF00000000)};

static inline uint32_t
do_shuf_stage (uint32_t src, uint64_t maskL, uint64_t maskR, int shift)
{
  uint32_t x = src & ~(maskL | maskR);
  x |= ((src << shift) & maskL) | ((src >> shift) & maskR);
  return x;
}

static inline uint32_t
do_swap32 (uint32_t x, uint64_t mask, int shift)
{
  return ((x & mask) << shift) | ((x & ~mask) >> shift);
}

static inline uint64_t
do_swap64 (uint64_t x, uint64_t mask, int shift)
{
  return ((x & mask) << shift) | ((x & ~mask) >> shift);
}

static inline uint32_t
do_xperm32 (uint32_t rs1, uint32_t rs2, uint32_t sz_log2)
{
  uint32_t r = 0;
  uint32_t sz = 1LL << sz_log2;
  uint32_t mask = (1LL << sz) - 1;
  uint32_t pos;

  for (int i = 0; i < 32; i += sz)
    {
      pos = ((rs2 >> i) & mask) << sz_log2;
      if (pos < sizeof (uint32_t) * 8)
	{
	  r |= ((rs1 >> pos) & mask) << i;
	}
    }
  return r;
}

static inline uint64_t
do_xperm64 (uint64_t rs1, uint64_t rs2, uint64_t sz_log2)
{
  uint64_t r = 0;
  uint64_t sz = 1LL << sz_log2;
  uint64_t mask = (1LL << sz) - 1;
  uint64_t pos;

  for (int i = 0; i < 64; i += sz)
    {
      pos = ((rs2 >> i) & mask) << sz_log2;
      if (pos < sizeof (uint64_t) * 8)
	{
	  r |= ((rs1 >> pos) & mask) << i;
	}
    }
  return r;
}
