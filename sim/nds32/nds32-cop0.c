/* Simulator for NDS32 COP0/FPU.

   Copyright (C) 2011-2013 Free Software Foundation, Inc.
   Contributed by Andes Technology Corporation.

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

#include "config.h"

#ifdef HAVE_STRING_H
#include <string.h>
#endif
#include "bfd.h"
#include "gdb/callback.h"
#include "gdb/signals.h"
#include "libiberty.h"
#include "gdb/remote-sim.h"
#include "dis-asm.h"
#include "sim-main.h"
#include "nds32-sim.h"
#include "sim-utils.h"
#include "sim-fpu.h"

#include "opcode/nds32.h"
#include "nds32-sim.h"

/* Concatenate the pair of FPRs specified by FD into a 64-bit value,
   so they can be written to memory.  */

static inline uint64_t
nds32_fd_to_64 (sim_cpu *cpu, int fd)
{
  fd <<= 1;
  return ((uint64_t) CCPU_FPR[fd].u << 32) | (uint64_t) CCPU_FPR[fd + 1].u;
}

/* Split the 64-bit value of a double-precision floating-point into
   high-part and low-part and write them into corresponding FPRs slots.  */

static inline void
nds32_fd_from_64 (sim_cpu *cpu, int fd, uint64_t u64)
{
  fd <<= 1;
  CCPU_FPR[fd + 1].u = u64;
  CCPU_FPR[fd].u = (u64 >> 32);
}

sim_cia
nds32_decode32_lwc (sim_cpu *cpu, const uint32_t insn, sim_cia cia)
{
  SIM_DESC sd = CPU_STATE (cpu);
  const int cop = __GF (insn, 13, 2);
  const int fst = N32_RT5 (insn);
  const int ra = N32_RA5 (insn);
  const int imm12s = N32_IMM12S (insn);

  SIM_ASSERT (cop == 0);

  if (__TEST (insn, 12))
    {
      CCPU_FPR[fst].u = nds32_ld_aligned (cpu, CCPU_GPR[ra].u, 4);
      CCPU_GPR[ra].u += (imm12s << 2);
    }
  else
    {
      CCPU_FPR[fst].u = nds32_ld_aligned (cpu, CCPU_GPR[ra].u + (imm12s << 2), 4);
    }

  return cia + 4;
}

sim_cia
nds32_decode32_swc (sim_cpu *cpu, const uint32_t insn, sim_cia cia)
{
  SIM_DESC sd = CPU_STATE (cpu);
  const int cop = __GF (insn, 13, 2);
  const int fst = N32_RT5 (insn);
  const int ra = N32_RA5 (insn);
  const int imm12s = N32_IMM12S (insn);

  SIM_ASSERT (cop == 0);

  if (__TEST (insn, 12))	/* fssi.bi */
    {
      nds32_st_aligned (cpu, CCPU_GPR[ra].u, 4, CCPU_FPR[fst].u);
      CCPU_GPR[ra].u += (imm12s << 2);
    }
  else				/* fssi */
    {
      nds32_st_aligned (cpu, CCPU_GPR[ra].u + (imm12s << 2), 4, CCPU_FPR[fst].u);
    }

  return cia + 4;
}

sim_cia
nds32_decode32_ldc (sim_cpu *cpu, const uint32_t insn, sim_cia cia)
{
  SIM_DESC sd = CPU_STATE (cpu);
  const int cop = __GF (insn, 13, 2);
  const int fdt = N32_RT5 (insn);
  const int ra = N32_RA5 (insn);
  const int imm12s = N32_IMM12S (insn);
  uint64_t u64;

  SIM_ASSERT (cop == 0);

  if (__TEST (insn, 12))	/* fldi.bi */
    {
      u64 = nds32_ld_aligned (cpu, CCPU_GPR[ra].u, 8);
      CCPU_GPR[ra].u += (imm12s << 2);
    }
  else				/* fldi */
    {
      u64 = nds32_ld_aligned (cpu, CCPU_GPR[ra].u + (imm12s << 2), 8);
    }

  nds32_fd_from_64 (cpu, fdt, u64);

  return cia + 4;
}

sim_cia
nds32_decode32_sdc (sim_cpu *cpu, const uint32_t insn, sim_cia cia)
{
  SIM_DESC sd = CPU_STATE (cpu);
  const int cop = __GF (insn, 13, 2);
  const int fdt = N32_RT5 (insn);
  const int ra = N32_RA5 (insn);
  const int imm12s = N32_IMM12S (insn);
  uint64_t u64;

  SIM_ASSERT (cop == 0);

  u64 = nds32_fd_to_64 (cpu, fdt);

  if (__TEST (insn, 12))
    {
      nds32_st_aligned (cpu, CCPU_GPR[ra].u, 8, u64);
      CCPU_GPR[ra].u += (imm12s << 2);
    }
  else
    {
      nds32_st_aligned (cpu, CCPU_GPR[ra].u + (imm12s << 2), 8, u64);
    }

  return cia + 4;
}

sim_cia
nds32_decode32_cop (sim_cpu *cpu, const uint32_t insn, sim_cia cia)
{
  SIM_DESC sd = CPU_STATE (cpu);
  static const int round_modes[] =
  {
      sim_fpu_round_near, sim_fpu_round_up,
      sim_fpu_round_down, sim_fpu_round_zero
  };
  const int cop = __GF (insn, 4, 2);
  const int sv = __GF (insn, 8, 2);
  const int fst = N32_RT5 (insn);
  const int fsa = N32_RA5 (insn);
  const int fsb = N32_RB5 (insn);
  const int rt = N32_RT5 (insn);
  const int ra = N32_RA5 (insn);
  const int rb = N32_RB5 (insn);
  const int fdt_ = N32_RT5 (insn) << 1;	/* I use fdX_ as shifted fdX. */
  const int fda_ = N32_RA5 (insn) << 1;
  const int fdb_ = N32_RB5 (insn) << 1;
  int fcmp = SIM_FPU_IS_SNAN;
  uint64_t u64;
  uint32_t u32;
  int32_t s32;
  sim_fpu sft, sft2;
  sim_fpu sfa;
  sim_fpu sfb;
  int denorm, rounding;

  /* Rounding and Denormalized flush-to-Zero modes in FPCSR.  */
  denorm = CCPU_FPCSR_TEST (DNZ) ? sim_fpu_denorm_zero
				 : sim_fpu_denorm_default;
  rounding = round_modes[CCPU_FPCSR_GET (RM)];


  SIM_ASSERT (cop == 0);

  /* Prepare operand for F[SD][12]. */
  if ((insn & 0xb) == 0)
    {
      /* FS1,  FS2 */
      sim_fpu_32to (&sfa, CCPU_FPR[fsa].u);
      sim_fpu_32to (&sfb, CCPU_FPR[fsb].u);

      /* MAC instructions use value in fst.  */
      switch (__GF (insn, 6, 4))
	{
	case 4: case 5: case 8: case 9:
	  sim_fpu_32to (&sft, CCPU_FPR[fst].u);
	  break;
	}
    }
  else if ((insn & 0xb) == 8)
    {
      /* FD1, FD2 */
      u64 = nds32_fd_to_64 (cpu, fda_ >> 1);
      sim_fpu_64to (&sfa, u64);
      u64 = nds32_fd_to_64 (cpu, fdb_ >> 1);
      sim_fpu_64to (&sfb, u64);

      /* MAC instructions use value in fst.  */
      switch (__GF (insn, 6, 4))
	{
	case 4: case 5: case 8: case 9:
	  u64 = nds32_fd_to_64 (cpu, fdt_ >> 1);
	  sim_fpu_64to (&sft, u64);
	  break;
	}
    }

  if (denorm == sim_fpu_denorm_zero)
    {
      if (sim_fpu_is_denorm (&sfa))
	sfa = sim_fpu_zero;
      if (sim_fpu_is_denorm (&sfb))
	sfb = sim_fpu_zero;
      if (sim_fpu_is_denorm (&sft))
	sft = sim_fpu_zero;
    }

  if ((insn & 0x7) == 0)	/* FS1 or FD1 */
    {
      int dp = (insn & 0x8) > 0;
      int sft_to_dp = dp;

      /* To simplify the operations, all the single-precision operations
	 are promoted to double-precision.  SFT_TO_DP determines whether
	 the final destination is single or double.  DP determines whether
	 the source operands are single or double.  */

      switch (__GF (insn, 6, 4))
	{
	case 0x0:		/* fadds */
	  sim_fpu_add (&sft, &sfa, &sfb);
	  break;
	case 0x1:		/* fsubs */
	  sim_fpu_sub (&sft, &sfa, &sfb);
	  break;
	case 0x2:		/* fcpynsd */
	  if (!dp)
	    {
	      /* fcpynss */
	      u32 = CCPU_FPR[fsa].u & 0x7fffffff;
	      u32 |= (CCPU_FPR[fsb].u & 0x80000000) ^ 0x80000000;
	      CCPU_FPR[fst].u = u32;
	    }
	  else
	    {
	      /* fcpynsd */
	      u32 = CCPU_FPR[fda_].u & 0x7fffffff;
	      u32 |= (CCPU_FPR[fdb_].u & 0x80000000) ^ 0x80000000;
	      CCPU_FPR[fdt_].u = u32;
	      CCPU_FPR[fdt_ + 1].u = CCPU_FPR[fda_ + 1].u;
	    }
	  goto done;
	case 0x3:
	  if (!dp)
	    {
	      /* fcpyss */
	      u32 = CCPU_FPR[fsa].u & 0x7fffffff;
	      u32 |= CCPU_FPR[fsb].u & 0x80000000;
	      CCPU_FPR[fst].u = u32;
	    }
	  else
	    {
	      /* fcpysd */
	      u32 = CCPU_FPR[fda_].u & 0x7fffffff;
	      u32 |= CCPU_FPR[fdb_].u & 0x80000000;
	      CCPU_FPR[fdt_].u = u32;
	      CCPU_FPR[fdt_ + 1].u = CCPU_FPR[fda_ + 1].u;
	    }
	  goto done; /* Just return.  */
	case 0x4:		/* fmaddd */
	  sim_fpu_mul (&sft2, &sfa, &sfb);
	  sim_fpu_add (&sft, &sft, &sft2);
	  break;
	case 0x5:		/* fmsubd */
	  sim_fpu_mul (&sft2, &sfa, &sfb);
	  sim_fpu_sub (&sft, &sft, &sft2);
	  break;
	case 0x6:		/* fcmovnX */
	case 0x7:		/* fcmovzX */
	  if (!dp)
	    {
	      /* fcmovzs */
	      if ((CCPU_FPR[fsb].u != 0) ^ (__TEST (insn, 6) != 0))
		CCPU_FPR[fst] = CCPU_FPR[fsa];
	    }
	  else
	    {
	      /* fcmovzd */
	      if ((CCPU_FPR[fsb].u != 0) ^ (__TEST (insn, 6) != 0))
		{
		  CCPU_FPR[fdt_] = CCPU_FPR[fda_];
		  CCPU_FPR[fdt_ + 1] = CCPU_FPR[fda_ + 1];
		}
	    }
	  goto done;
	case 0x8:		/* fnmaddd */
	  sim_fpu_mul (&sft2, &sfa, &sfb);
	  sim_fpu_add (&sft, &sft, &sft2);
	  sim_fpu_neg (&sft, &sft);
	  break;
	case 0x9:		/* fnmsubd */
	  sim_fpu_mul (&sft2, &sfa, &sfb);
	  sim_fpu_sub (&sft, &sft, &sft2);
	  sim_fpu_neg (&sft, &sft);
	  break;
	case 0xc:		/* fmuls */
	  sim_fpu_mul (&sft, &sfa, &sfb);
	  sim_fpu_round_64 (&sft, rounding, denorm);
	  break;
	case 0xd:		/* fdivs */
	  sim_fpu_div (&sft, &sfa, &sfb);
	  break;
	/* case 0xa:		reserved */
	/* case 0xb:		reserved */
	case 0xf:		/* F2OP */
	  switch (__GF (insn, 10, 5))
	    {
	    case 0x0:		/* fs2d, fd2s */
	      sft = sfa;
	      sft_to_dp = !dp;
	      break;
	    case 0x1:		/* sqrts, sqrtd */
	      /* Set IVO for EDOM.  */
	      if (sim_fpu_sqrt (&sft, &sfa) != 0)
		CCPU_FPCSR_SET(IVO);
	      break;
	    case 0x5:
	      if (!dp)
		{
		  /* fabss */
		  CCPU_FPR[fst].u = CCPU_FPR[fsa].u & 0x7fffffff;
		}
	      else
		{
		  /* fabsd */
		  CCPU_FPR[fdt_].u = CCPU_FPR[fda_].u & 0x7fffffff;
		  CCPU_FPR[fdt_ + 1].u = CCPU_FPR[fda_ + 1].u;
		}
	      goto done; /* Just return.  */
	    case 0x8:		/* fui2s, fui2d */
	      sim_fpu_u32to (&sft, CCPU_FPR[fsa].u, rounding);
	      break;
	    case 0xc:		/* fsi2s, fsi2d */
	      sim_fpu_i32to (&sft, CCPU_FPR[fsa].u, rounding);
	      break;
	    case 0x10:		/* fs2ui, fd2ui */
	    case 0x14:		/* fs2ui.z, fd2ui.z */
	      sim_fpu_to32u (&u32, &sfa, __TEST (insn, 12)
					 ? sim_fpu_round_zero : rounding);
	      CCPU_FPR[fst].u = u32;
	      goto done;	/* just return */
	    case 0x18:		/* fs2si, fd2si */
	    case 0x1c:		/* fs2si.z, fd2si.z */
	      sim_fpu_to32i (&s32, &sfa, __TEST (insn, 12)
					 ? sim_fpu_round_zero : rounding);
	      CCPU_FPR[fst].s = s32;
	      goto done; /* Just return.  */
	    default:
	      goto bad_op;
	    }
	  break;
	default:
	  goto bad_op;
	}

      if (!sft_to_dp)
	{
	  /* General epilogue for saving result to fst.  */
	  sim_fpu_round_32 (&sft, rounding, denorm);
	  sim_fpu_to32 ((unsigned32 *) (CCPU_FPR + fst), &sft);
	}
      else
	{
	  /* General epilogue for saving result to fdt.  */
	  sim_fpu_to64 (&u64, &sft);
	  nds32_fd_from_64 (cpu, fdt_ >> 1, u64);
	}
      goto done;
    }

  if ((insn & 0x7) == 4)	/* FS2 or FD2 */
    {
      /* fcmpxxd and fcmpxxs share this function. */
      switch (__GF (insn, 7, 3))
	{
	case 0x0:		/* fcmpeq[sd] */
	  CCPU_FPR[fst].u = sim_fpu_is_eq (&sfa, &sfb);
	  goto done;
	case 0x1:		/* fcmplt[sd] */
	  CCPU_FPR[fst].u = sim_fpu_is_lt (&sfa, &sfb);
	  goto done;
	case 0x2:		/* fcmple[sd] */
	  CCPU_FPR[fst].u = sim_fpu_is_le (&sfa, &sfb);
	  goto done;
	case 0x3:		/* fcmpun[sd] */
	  CCPU_FPR[fst].u = (sim_fpu_is_nan (&sfa) || sim_fpu_is_nan (&sfb))
			    ? 1 : 0;
	  goto done;
	default:
	  goto bad_op;
	}
      goto done;
    }

  if ((insn & 0x7) == 1)	/* MFCP or MTCP */
    {
      switch (insn & 0x3ff)
	{
	case 0x1:		/* fmfsr */
	  CCPU_GPR[rt].u = CCPU_FPR[fsa].u;
	  goto done;
	case 0x9:		/* fmtsr */
	  CCPU_FPR[fsa].u = CCPU_GPR[rt].u;
	  goto done;
	case 0x41:		/* fmfdr */
	    {
	      int rt_ = rt & ~1;

	      if (CCPU_SR_TEST (PSW, PSW_BE))
		{
		  CCPU_GPR[rt_] = CCPU_FPR[fda_];
		  CCPU_GPR[rt_ + 1] = CCPU_FPR[fda_ + 1];
		}
	      else
		{
		  CCPU_GPR[rt_] = CCPU_FPR[fda_ + 1];
		  CCPU_GPR[rt_ + 1] = CCPU_FPR[fda_];
		}
	    }
	  goto done;
	case 0x49:		/* fmtdr */
	    {
	      int rt_ = rt & ~1;

	      if (CCPU_SR_TEST (PSW, PSW_BE))
		{
		  CCPU_FPR[fda_ + 1] = CCPU_GPR[rt_ + 1];
		  CCPU_FPR[fda_] = CCPU_GPR[rt_];
		}
	      else
		{
		  CCPU_FPR[fda_ + 1] = CCPU_GPR[rt_];
		  CCPU_FPR[fda_] = CCPU_GPR[rt_ + 1];
		}
	    }
	  goto done;
	case 0x301:		/* FMFXR */
	  if (rb == 0)		/* fmfcfg */
	    CCPU_GPR[rt] = CCPU_FPCFG;
	  else if (rb == 1)	/* fmfcsr */
	    CCPU_GPR[rt] = CCPU_FPCSR;
	  else
	    goto bad_op;
	  goto done;
	case 0x309:		/* FMTXR */
	  if (rb == 1)		/* fmtcsr */
	    CCPU_FPCSR = CCPU_GPR[rt];
	  else
	    goto bad_op;
	  goto done;
	default:
	  goto bad_op;
	}
    }

  switch (insn & 0xff)
    {
    case 0x2:			/* fls */
      u32 = nds32_ld_aligned (cpu, CCPU_GPR[ra].u + (CCPU_GPR[rb].s << sv), 4);
      CCPU_FPR[fst].u = u32;
      goto done;
    case 0x3:			/* fld */
      u64 = nds32_ld_aligned (cpu, CCPU_GPR[ra].u + (CCPU_GPR[rb].s << sv), 8);
      nds32_fd_from_64 (cpu, fdt_ >> 1, u64);
      goto done;
    case 0xa:			/* fss */
      nds32_st_aligned (cpu, CCPU_GPR[ra].u + (CCPU_GPR[rb].s << sv), 4, CCPU_FPR[fst].u);
      goto done;
    case 0xb:			/* fsd */
      u64 = nds32_fd_to_64 (cpu, fdt_ >> 1);
      nds32_st_aligned (cpu, CCPU_GPR[ra].u + (CCPU_GPR[rb].s << sv), 8, u64);
      goto done;
    case 0x82:			/* fls.bi */
      u32 = nds32_ld_aligned (cpu, CCPU_GPR[ra].u, 4);
      CCPU_GPR[ra].u += (CCPU_GPR[rb].s << sv);
      CCPU_FPR[fst].u = u32;
      goto done;
    case 0x83:			/* fld.bi */
      u64 = nds32_ld_aligned (cpu, CCPU_GPR[ra].u, 8);
      CCPU_GPR[ra].u += (CCPU_GPR[rb].s << sv);
      nds32_fd_from_64 (cpu, fdt_ >> 1, u64);
      goto done;
    case 0x8a:			/* fss.bi */
      nds32_st_aligned (cpu, CCPU_GPR[ra].u, 4, CCPU_FPR[fst].u);
      CCPU_GPR[ra].u += (CCPU_GPR[rb].s << sv);
      goto done;
    case 0x8b:			/* fsd.bi */
      u64 = nds32_fd_to_64 (cpu, fdt_ >> 1);
      nds32_st_aligned (cpu, CCPU_GPR[ra].u, 8, u64);
      CCPU_GPR[ra].u += (CCPU_GPR[rb].s << sv);
      goto done;
    default:
      goto bad_op;
    }


done:
  return cia + 4;

bad_op:
  nds32_bad_op (cpu, cia, insn, "COP");
  return cia;
}
