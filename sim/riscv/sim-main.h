/* RISC-V simulator.

   Copyright (C) 2005-2022 Free Software Foundation, Inc.
   Contributed by Mike Frysinger.

   This file is part of simulators.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.  */

#ifndef SIM_MAIN_H
#define SIM_MAIN_H

#include "sim-basics.h"
#include "machs.h"
#include "sim-base.h"
#include "softfloat/softfloat_types.h"

typedef int64_t signed64;
typedef uint64_t unsigned64;
typedef int32_t signed32;
typedef uint32_t unsigned32;

#if (WITH_TARGET_WORD_BITSIZE == 64)
typedef union {
  uint64_t u;
  int64_t s;

  struct
    {
      int32_t i0;
      int32_t i1;
    } b32;
  struct
    {
      int16_t h0;
      int16_t h1;
      int16_t h2;
      int16_t h3;
    } b16;
  struct
    {
      int8_t b0;
      int8_t b1;
      int8_t b2;
      int8_t b3;
      int8_t b4;
      int8_t b5;
      int8_t b6;
      int8_t b7;
    } b8;
  struct
    {
      uint32_t i0;
      uint32_t i1;
    } ub32;
  struct
    {
      uint16_t h0;
      uint16_t h1;
      uint16_t h2;
      uint16_t h3;
    } ub16;
  struct
    {
      uint8_t b0;
      uint8_t b1;
      uint8_t b2;
      uint8_t b3;
      uint8_t b4;
      uint8_t b5;
      uint8_t b6;
      uint8_t b7;
    } ub8;
} reg_t;
#else
typedef union {
  uint32_t u;
  int32_t s;

  struct
    {
      int32_t i0;
    } b32;
  struct
    {
      int16_t h0;
      int16_t h1;
    } b16;
  struct
    {
      int8_t b0;
      int8_t b1;
      int8_t b2;
      int8_t b3;
    } b8;
  struct
    {
      uint32_t i0;
    } ub32;
  struct
    {
      uint16_t h0;
      uint16_t h1;
    } ub16;
  struct
    {
      uint8_t b0;
      uint8_t b1;
      uint8_t b2;
      uint8_t b3;
    } ub8;
} reg_t;
#endif

typedef union
{
  int64_t d0;

  struct
    {
      uint32_t w0;
      int32_t w1;
    } b32;
} union64_t;

typedef union
{
  int32_t  W;
  int16_t  H[2];

  uint32_t w;
  uint16_t h[2];

  float    S;

  float16_t hf[2];
  float32_t f;
} union32_t;

typedef union FRegisterValue
{
  uint64_t     v[2];
  uint32_t     w[4];
  uint16_t     h[8];

  int64_t      V[2];
  int32_t      W[4];
  int16_t      H[8];

  float        S[4];
  double       D[2];

  float32_t    f[4];
} FRegister;

struct _sim_cpu {
  union {
    reg_t regs[32];
    struct {
      /* These are the ABI names.  */
      reg_t zero, ra, sp, gp, tp;
      reg_t t0, t1, t2;
      reg_t s0, s1;
      reg_t a0, a1, a2, a3, a4, a5, a6, a7;
      reg_t s2, s3, s4, s5, s6, s7, s8, s9, s10, s11;
      reg_t t3, t4, t5, t6;
    };
  };
  union {
    FRegister fpregs[32];
    struct {
      /* These are the ABI names.  */
      unsigned_word ft0, ft1, ft2, ft3, ft4, ft5, ft6, ft7;
      unsigned_word fs0, fs1;
      unsigned_word fa0, fa1, fa2, fa3, fa4, fa5, fa6, fa7;
      unsigned_word fs2, fs3, fs4, fs5, fs6, fs7, fs8, fs9, fs10, fs11;
      unsigned_word ft8, ft9, ft10, ft11;
    };
  };

  /* System registers.  */
  reg_t reg_sr[8 * 16 * 8];
#define CCPU_SR         (cpu->reg_sr)

  sim_cia pc;
  sim_cia endbrk;
  unsigned long elf_flags;
#define CPU_ELF_FLAGS(cpu) ((cpu)->elf_flags)

  struct {
#define DECLARE_CSR(name, ...) unsigned_word name;
#include "opcode/riscv-opc.h"
#undef DECLARE_CSR
  } csr;

  sim_cpu_base base;
};

#define __TEST(VALUE,BIT)	(((VALUE) & (1 << (BIT))) ? 1 : 0)
#define __SET(VALUE,BIT)	do { (VALUE) |= (1 << (BIT)); } while (0)
#define __CLEAR(VALUE,BIT)	do { (VALUE) &= ~(1 << (BIT)); } while (0)
#define __GET(VALUE,BIT)	(((VALUE) >> (BIT)) & ((1 << (BIT##_N)) - 1))
#define __PUT(VALUE,BIT,V)	do { __put_field (&(VALUE), (BIT), (BIT##_N), (V)); } while (0)

#define CCPU_SR_TEST(SREG,BIT)	__TEST (cpu->reg_sr[SRIDX_##SREG].u, BIT)
#define CCPU_SR_SET(SREG,BIT)	__SET (cpu->reg_sr[SRIDX_##SREG].u, BIT)
#define CCPU_SR_CLEAR(SREG,BIT)	__CLEAR (cpu->reg_sr[SRIDX_##SREG].u, BIT)
#define CCPU_SR_GET(SREG,BIT)	__GET (cpu->reg_sr[SRIDX_##SREG].u, BIT)
#define CCPU_SR_PUT(SREG,BIT,V)	__PUT (cpu->reg_sr[SRIDX_##SREG].u, BIT, V)

#define CCPU_FPCFG_TEST(BIT)	__TEST (cpu->reg_fpcfg.u, FPCFG_##BIT)
#define CCPU_FPCFG_SET(BIT)	__SET (cpu->reg_fpcfg.u, FPCFG_##BIT)
#define CCPU_FPCFG_CLEAR(BIT)	__CLEAR (cpu->reg_fpcfg.u, FPCFG_##BIT)
#define CCPU_FPCFG_GET(BIT)	__GET (cpu->reg_fpcfg.u, FPCFG_##BIT)
#define CCPU_FPCFG_PUT(BIT,V)	__PUT (cpu->reg_fpcfg.u, FPCFG_##BIT, V)

#define CCPU_FPCSR_TEST(BIT)	__TEST (cpu->reg_fpcsr.u, FPCSR_##BIT)
#define CCPU_FPCSR_SET(BIT)	__SET (cpu->reg_fpcsr.u, FPCSR_##BIT)
#define CCPU_FPCSR_CLEAR(BIT)	__CLEAR (cpu->reg_fpcsr.u, FPCSR_##BIT)
#define CCPU_FPCSR_GET(BIT)	__GET (cpu->reg_fpcsr.u, FPCSR_##BIT)
#define CCPU_FPCSR_PUT(BIT,V)	__PUT (cpu->reg_fpcsr.u, FPCSR_##BIT, V)

#define SRIDX(M,m,e)  ((M << 7) | (m << 3) | e)

enum
{
  SRIDX_PSW	= SRIDX (1, 0, 0),
  SRIDX_IPSW	= SRIDX (1, 0, 1),
  SRIDX_P_IPSW	= SRIDX (1, 0, 2),
  PSW_GIE	= 0,
  PSW_BE	= 5,
  PSW_IFCON	= 15,
  PSW_OV	= 20,

  SRIDX_IVB	= SRIDX (1, 1, 1),
  IVB_EVIC	= 13,
  IVB_ESZ	= 14,
  IVB_ESZ_N	= 2,
  IVB_IVBASE	= 16,
  IVB_IVBASE_N	= 16,

  SRIDX_EVA	= SRIDX (1, 2, 1),
  SRIDX_P_EVA	= SRIDX (1, 2, 2),
  SRIDX_ITYPE	= SRIDX (1, 3, 1),
  SRIDX_P_ITYPE	= SRIDX (1, 3, 2),
  ITYPE_ETYPE	= 0,
  ITYPE_ETYPE_N	= 4,
  ITYPE_INST	= 4,
  ITYPE_SWID	= 16,
  ITYPE_SWID_N	= 15,

  SRIDX_MERR	= SRIDX (1, 4, 1),
  SRIDX_IPC	= SRIDX (1, 5, 1),
  SRIDX_P_IPC	= SRIDX (1, 5, 2),
  SRIDX_OIPC	= SRIDX (1, 5, 3),
  SRIDX_P_P0	= SRIDX (1, 6, 2),
  SRIDX_P_P1	= SRIDX (1, 7, 2),
  SRIDX_INT_MASK= SRIDX (1, 8, 0),
  SRIDX_INT_PEND= SRIDX (1, 9, 0),

  SRIDX_MSC_CFG	= SRIDX (0, 4, 0),
  MSC_CFG_PFM	= 2,
  MSC_CFG_DIV	= 5,
  MSC_CFG_MAC	= 6,
  MSC_CFG_IFC	= 19,
  MSC_CFG_EIT	= 24,

  SRIDX_PFMC0	= SRIDX (4, 0, 0),
  SRIDX_PFMC1	= SRIDX (4, 0, 1),
  SRIDX_PFMC2	= SRIDX (4, 0, 2),
  SRIDX_PFM_CTL	= SRIDX (4, 1, 0),
  PFM_CTL_EN	= 0,
  PFM_CTL_EN_N	= 3,
  PFM_CTL_IE	= 3,
  PFM_CTL_IE_N	= 3,
  PFM_CTL_OVF	= 6,
  PFM_CTL_OVF_N	= 3,
  PFM_CTL_KS	= 9,
  PFM_CTL_KS_N	= 3,
  PFM_CTL_KU	= 12,
  PFM_CTL_KU_N	= 3,
  PFM_CTL_SEL0	= 15,
  PFM_CTL_SEL0_N= 1,
  PFM_CTL_SEL1	= 16,
  PFM_CTL_SEL1_N= 6,
  PFM_CTL_SEL2	= 22,
  PFM_CTL_SEL2_N= 6,

  FPCFG_SP	= 0,
  FPCFG_DP	= 1,
  FPCFG_FREG	= 2,
  FPCFG_FREG_N	= 2,
  FPCFG_FMA	= 4,
  FPCFG_IMVER	= 22,
  FPCFG_IMVER_N	= 5,
  FPCFG_AVER	= 27,
  FPCFG_AVER_N	= 5,

  FPCSR_RM	= 0,
  FPCSR_RM_N	= 2,
  FPCSR_IVO	= 2,
  FPCSR_DBZ	= 3,
  FPCSR_OVF	= 4,
  FPCSR_UDF	= 5,
  FPCSR_IEX	= 6,
  FPCSR_IVOE	= 7,
  FPCSR_DBZE	= 8,
  FPCSR_OVFE	= 9,
  FPCSR_UDEF	= 10,
  FPCSR_IEXE	= 11,
  FPCSR_DNZ	= 12,
  FPCSR_IVOT	= 13,
  FPCSR_DBZT	= 14,
  FPCSR_OVFT	= 15,
  FPCSR_UDFT	= 16,
  FPCSR_IEXT	= 17,
  FPCSR_DNIT	= 18,
  FPCSR_RIT	= 19,
};

struct atomic_mem_reserved_list;
struct atomic_mem_reserved_list {
  struct atomic_mem_reserved_list *next;
  address_word addr;
};

struct riscv_sim_state {
  struct atomic_mem_reserved_list *amo_reserved_list;
};
#define RISCV_SIM_STATE(sd) ((struct riscv_sim_state *) STATE_ARCH_DATA (sd))

extern void step_once (SIM_CPU *);
extern void initialize_cpu (SIM_DESC, SIM_CPU *, int);
extern void initialize_env (SIM_DESC, const char * const *argv,
			    const char * const *env);
extern sim_cia riscv_decode (SIM_CPU *, unsigned_word, sim_cia, int);

#define DEFAULT_MEM_SIZE (64 * 1024 * 1024)

#define RISCV_XLEN(cpu) MACH_WORD_BITSIZE (CPU_MACH (cpu))
#define SIM_RV_X(x, s, n) \
  (((x) >> (unsigned_word)(s)) \
   & (((unsigned_word)1UL << (unsigned_word)(n)) - (unsigned_word)1UL))
#define SIM_RV_SEXT(x, bs) \
  ((((x) & (((unsigned_word)1UL << (unsigned_word)(bs)) \
	    - (unsigned_word)1UL)) \
    ^ ((unsigned_word)1UL << ((unsigned_word)(bs) - (unsigned_word)1UL))) \
   - ((unsigned_word)1UL << ((unsigned_word)(bs) - (unsigned_word)1UL)))

#endif
