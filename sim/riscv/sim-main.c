/* RISC-V simulator.

   Copyright (C) 2005-2014 Free Software Foundation, Inc.
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

/* This file contains the main simulator decoding logic.  i.e. everything that
   is architecture specific.  */

#include "config.h"

#include <inttypes.h>
#include <time.h>
#include <ctype.h>
#include <unistd.h>
#include <sys/time.h>

#include "sim-main.h"
#include "sim-fpu.h"
#include "sim-syscall.h"
#include "softfloat/softfloat.h"

#include "opcode/riscv.h"
#include "elf/riscv.h"

#include "gdb/sim-riscv.h"

#include "targ-vals.h"
#ifdef __MINGW32__
#include "windows.h"
#endif


#define TRACE_REG(cpu, reg) TRACE_REGISTER (cpu, "wrote %s = %#"PRIxTW, riscv_gpr_names_abi[reg], cpu->regs[reg].u)
#define TRACE_FREG(cpu, reg) TRACE_REGISTER (cpu, "wrote %s = %#" PRIx64, riscv_fpr_names_abi[reg], cpu->fpregs[reg].v[0])

#define HASH_TABLE_SZ (OP_MASK_OP + 1)
static const struct riscv_opcode *riscv_hash[HASH_TABLE_SZ];
static struct riscv_opcode *sim_riscv_opcodes = NULL;
#define OP_HASH_IDX(i) ((i) & (riscv_insn_length (i) == 2 ? 0x3 : 0x7f))

#define RISCV_ASSERT_RV32(cpu, fmt, args...) \
  do { \
    if (RISCV_XLEN (cpu) != 32) \
      { \
	SIM_DESC sd = CPU_STATE (cpu); \
	TRACE_INSN (cpu, "RV32I-only " fmt, ## args); \
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL); \
      } \
  } while (0)

#define RISCV_ASSERT_RV64(cpu, fmt, args...) \
  do { \
    if (RISCV_XLEN (cpu) != 64) \
      { \
	SIM_DESC sd = CPU_STATE (cpu); \
	TRACE_INSN (cpu, "RV64I-only " fmt, ## args); \
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL); \
      } \
  } while (0)

STATIC_INLINE void
store_rd (SIM_CPU *cpu, int rd, unsigned_word val)
{
  if (rd)
    {
      cpu->regs[rd].u = val;
      TRACE_REG (cpu, rd);
    }
}

STATIC_INLINE void
store_frd (SIM_CPU *cpu, int rd, unsigned32 val)
{
  cpu->fpregs[rd].w[0] = val;
  TRACE_FREG (cpu, rd);
}

STATIC_INLINE void
store_frd64 (SIM_CPU *cpu, int rd, unsigned64 val)
{
  cpu->fpregs[rd].v[0] = val;
  TRACE_FREG (cpu, rd);
}

STATIC_INLINE unsigned_word
fetch_csr (SIM_CPU *cpu, const char *name, int csr, unsigned_word *reg)
{
  /* Handle pseudo registers.  */
  switch (csr)
    {
    /* Allow certain registers only in respective modes.  */
    case CSR_CYCLEH:
    case CSR_INSTRETH:
    case CSR_TIMEH:
      RISCV_ASSERT_RV32 (cpu, "CSR: %s", name);
      break;
    }

  return *reg;
}

STATIC_INLINE void
store_csr (SIM_CPU *cpu, const char *name, int csr, unsigned_word *reg,
	   unsigned_word val)
{
  switch (csr)
    {
    /* These are pseudo registers that modify sub-fields of fcsr.  */
    case CSR_FRM:
      val &= 0x7;
      *reg = val;
      cpu->csr.fcsr = (cpu->csr.fcsr & ~0xe0) | (val << 5);
      break;
    case CSR_FFLAGS:
      val &= 0x1f;
      *reg = val;
      cpu->csr.fcsr = (cpu->csr.fcsr & ~0x1f) | val;
      break;
    /* Keep the sub-fields in sync.  */
    case CSR_FCSR:
      *reg = val;
      cpu->csr.frm = (val >> 5) & 0x7;
      cpu->csr.fflags = val & 0x1f;
      break;
    case CSR_UITB:
      cpu->csr.uitb = val;
      break;

    /* Allow certain registers only in respective modes.  */
    case CSR_CYCLEH:
    case CSR_INSTRETH:
    case CSR_TIMEH:
      RISCV_ASSERT_RV32 (cpu, "CSR: %s", name);

    /* All the rest are immutable.  */
    default:
      *reg = val;
      break;
    }

  TRACE_REGISTER (cpu, "wrote CSR %s = %#"PRIxTW, name, val);
}

STATIC_INLINE unsigned_word
ashiftrt (unsigned_word val, unsigned_word shift)
{
  unsigned32 sign = (val & 0x80000000) ? ~(0xfffffffful >> shift) : 0;
  return (val >> shift) | sign;
}

STATIC_INLINE unsigned_word
ashiftrt64 (unsigned_word val, unsigned_word shift)
{
  unsigned64 sign = (val & 0x8000000000000000ull) ? ~(0xffffffffffffffffull >> shift) : 0;
  return (val >> shift) | sign;
}

static int64_t
get_double (sim_cpu *cpu, int regnum)
{
  if (CCPU_SR_TEST (PSW, PSW_BE))
    return ((int64_t) cpu->regs[regnum].s << 32)
	    | ((int64_t) cpu->regs[regnum + 1].s & 0xFFFFFFFF);
  else
    return ((int64_t) cpu->regs[regnum + 1].s << 32)
	    | ((int64_t) cpu->regs[regnum].s & 0xFFFFFFFF);
}

static uint64_t
get_udouble (sim_cpu *cpu, int regnum)
{
  if (CCPU_SR_TEST (PSW, PSW_BE))
    return ((uint64_t) cpu->regs[regnum].u << 32)
	    | ((uint64_t) cpu->regs[regnum + 1].u & 0xFFFFFFFF);
  else
    return ((uint64_t) cpu->regs[regnum + 1].u << 32)
	    | ((uint64_t) cpu->regs[regnum].u & 0xFFFFFFFF);
}

static void
set_double (sim_cpu *cpu, int regnum, int64_t val)
{
  if (CCPU_SR_TEST (PSW, PSW_BE))
    {
      cpu->regs[regnum + 1].s = val & 0xFFFFFFFF;
      cpu->regs[regnum].s = (val >> 32) & 0xFFFFFFFF;
    }
  else
    {
      cpu->regs[regnum].s = val & 0xFFFFFFFF;
      cpu->regs[regnum + 1].s = (val >> 32) & 0xFFFFFFFF;
    }
  TRACE_REG (cpu, regnum);
  TRACE_REG (cpu, regnum + 1);
}

static void
set_udouble (sim_cpu *cpu, int regnum, uint64_t val)
{
  if (CCPU_SR_TEST (PSW, PSW_BE))
    {
      cpu->regs[regnum + 1].u = val & 0xFFFFFFFF;
      cpu->regs[regnum].u = (val >> 32) & 0xFFFFFFFF;
    }
  else
    {
      cpu->regs[regnum].u = val & 0xFFFFFFFF;
      cpu->regs[regnum + 1].u = (val >> 32) & 0xFFFFFFFF;
    }
}

static int64_t
insn_usat_helper (sim_cpu *cpu, int64_t res, const short range)
{
  const int64_t max = (1LL << range) - 1;

  if (res > max)
    {
      res = max;
      CCPU_UCODE_OV_SET();
    }
  else if (res < 0)
    {
      res = 0;
      CCPU_UCODE_OV_SET();
    }

  return res;
}

static int64_t
insn_sat_helper (sim_cpu *cpu, int64_t res, const short range)
{
  const int64_t max = (1LL << range) - 1;
  const int64_t min = -(1LL << range);

  if (res > max)
    {
      res = max;
      CCPU_UCODE_OV_SET();
    }
  else if (res < min)
    {
      res = min;
      CCPU_UCODE_OV_SET();
    }

  return res;
}

static int16_t
insn_sat_khm_helper (sim_cpu *cpu, int16_t aop, int16_t bop)
{
  int16_t res;
  if (((int16_t) 0x8000 != aop) || ((int16_t) 0x8000 != bop))
    res = (int16_t) (((int32_t) aop * bop) >> 15);
  else
    {
      res = 0x7fff;
      CCPU_UCODE_OV_SET();
    }
  return res;
}

static int8_t
insn_sat_khm8_helper (sim_cpu *cpu, int8_t aop, int8_t bop)
{
  int16_t res;
  if (((int8_t) 0x80 != aop) || ((int8_t) 0x80 != bop))
    res = (int8_t) (((int16_t) aop * bop) >> 7);
  else
    {
      res = 0x7f;
      CCPU_UCODE_OV_SET();
    }
  return res;
}

static sim_cia
execute_d (SIM_CPU *cpu, unsigned_word iw, const struct riscv_opcode *op, int ex9)
{
  SIM_DESC sd = CPU_STATE (cpu);
  unsigned int mask_arithmetic = MASK_FADD_D;
  unsigned int mask_mul_add = MASK_FMADD_S;
  unsigned int mask_convert = MASK_FCVT_S_W;

  static const int round_modes[] =
  {
      sim_fpu_round_near, sim_fpu_round_zero,
      sim_fpu_round_down, sim_fpu_round_up,
      sim_fpu_round_default, sim_fpu_round_default,
      sim_fpu_round_default
  };

  int rd = (iw >> OP_SH_RD) & OP_MASK_RD;
  int rs1 = (iw >> OP_SH_RS1) & OP_MASK_RS1;
  int rs2 = (iw >> OP_SH_RS2) & OP_MASK_RS2;
  int rs3 = (iw >> OP_SH_RS3) & OP_MASK_RS3;
  const char *frd_name = riscv_fpr_names_abi[rd];
  const char *frs1_name = riscv_fpr_names_abi[rs1];
  const char *frs2_name = riscv_fpr_names_abi[rs2];
  const char *frs3_name = riscv_fpr_names_abi[rs3];
  const char *rd_name = riscv_gpr_names_abi[rd];
  const char *rs1_name = riscv_gpr_names_abi[rs1];
  unsigned_word i_imm = EXTRACT_ITYPE_IMM (iw);
  unsigned_word s_imm = EXTRACT_STYPE_IMM (iw);
  uint32_t u32;
  int32_t i32;
  uint64_t u64;
  int64_t i64;
  sim_cia pc = cpu->pc + 4;
  /* Rounding mode.  */
  int rm = (iw >> OP_SH_RM) & OP_MASK_RM;
  int rounding = round_modes[rm];
  int zfh = 0;

  sim_fpu sft, sft2;
  sim_fpu sfa, sfb, sfc;

  if (ex9)
    pc -= 2;

  sim_fpu_64to (&sfa, cpu->fpregs[rs1].v[0]);
  sim_fpu_64to (&sfb, cpu->fpregs[rs2].v[0]);

  for (int i = 0; op->subset[i]; i++)
    {
      if (strncmp(op->subset[i], "ZFH", 3) == 0)
	zfh = 1;
    }

  if (zfh)
    {
    switch (op->match & mask_convert)
      {
      case MATCH_FCVT_D_H:
        TRACE_INSN (cpu, "fcvt.d.h %s, %s",
		    frd_name, frs1_name);
        cpu->fpregs[rd].d[0] = f16_to_f64(cpu->fpregs[rs1].hf[0]);
        TRACE_FREG (cpu, rd);
        goto done;
      case MATCH_FCVT_H_D:
        TRACE_INSN (cpu, "fcvt.h.d %s, %s",
		    frd_name, frs1_name);
        cpu->fpregs[rd].hf[0] = f64_to_f16(cpu->fpregs[rs1].d[0]);
        TRACE_FREG (cpu, rd);
        goto done;
      }
    }

  switch (op->match & mask_mul_add)
    {
    case MATCH_FMADD_D:
      TRACE_INSN (cpu, "fmadd.d %s, %s, %s, %s",
		  frd_name, frs1_name, frs2_name, frs3_name);
      sim_fpu_64to (&sfc, cpu->fpregs[rs3].v[0]);
      sim_fpu_mul (&sft2, &sfa, &sfb);
      sim_fpu_add (&sft, &sfc, &sft2);
      sim_fpu_round_64 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to64 (&cpu->fpregs[rd].v[0], &sft);
      TRACE_FREG (cpu, rd);
      goto done;
    case MATCH_FMSUB_D:
      TRACE_INSN (cpu, "fmsub.d %s, %s, %s, %s",
		  frd_name, frs1_name, frs2_name, frs3_name);
      sim_fpu_64to (&sfc, cpu->fpregs[rs3].v[0]);
      sim_fpu_mul (&sft2, &sfa, &sfb);
      sim_fpu_sub (&sft, &sft2, &sfc);
      sim_fpu_round_64 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to64 (&cpu->fpregs[rd].v[0], &sft);
      TRACE_FREG (cpu, rd);
      goto done;
    case MATCH_FNMADD_D:
      TRACE_INSN (cpu, "fnmadd.d %s, %s, %s, %s",
		  frd_name, frs1_name, frs2_name, frs3_name);
      sim_fpu_64to (&sfc, cpu->fpregs[rs3].v[0]);
      sim_fpu_neg (&sfa, &sfa);
      sim_fpu_mul (&sft2, &sfa, &sfb);
      sim_fpu_sub (&sft, &sft2, &sfc);
      sim_fpu_round_64 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to64 (&cpu->fpregs[rd].v[0], &sft);
      TRACE_FREG (cpu, rd);
      goto done;
    case MATCH_FNMSUB_D:
      TRACE_INSN (cpu, "fnmsub.d %s, %s, %s, %s",
		  frd_name, frs1_name, frs2_name, frs3_name);
      sim_fpu_64to (&sfc, cpu->fpregs[rs3].v[0]);
      sim_fpu_neg (&sfa, &sfa);
      sim_fpu_mul (&sft2, &sfa, &sfb);
      sim_fpu_add (&sft, &sft2, &sfc);
      sim_fpu_round_64 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to64 (&cpu->fpregs[rd].v[0], &sft);
      TRACE_FREG (cpu, rd);
      goto done;
    }

  switch (op->match & mask_arithmetic)
    {
    case MATCH_FADD_D:
      TRACE_INSN (cpu, "fadd.d %s, %s, %s",
		  frd_name, frs1_name, frs2_name);
      sim_fpu_add (&sft, &sfa, &sfb);
      sim_fpu_round_64 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to64 (&cpu->fpregs[rd].v[0], &sft);
      TRACE_FREG (cpu, rd);
      goto done;
    case MATCH_FSUB_D:
      TRACE_INSN (cpu, "fsub.d %s, %s, %s",
		  frd_name, frs1_name, frs2_name);
      sim_fpu_sub (&sft, &sfa, &sfb);
      sim_fpu_round_64 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to64 (&cpu->fpregs[rd].v[0], &sft);
      TRACE_FREG (cpu, rd);
      goto done;
    case MATCH_FMUL_D:
      TRACE_INSN (cpu, "fmul.d %s, %s, %s",
		  frd_name, frs1_name, frs2_name);
      sim_fpu_mul (&sft, &sfa, &sfb);
      sim_fpu_round_64 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to64 (&cpu->fpregs[rd].v[0], &sft);
      TRACE_FREG (cpu, rd);
      goto done;
    case MATCH_FDIV_D:
      TRACE_INSN (cpu, "fdiv.d %s, %s, %s",
		  frd_name, frs1_name, frs2_name);
      sim_fpu_div (&sft, &sfa, &sfb);
      sim_fpu_round_64 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to64 (&cpu->fpregs[rd].v[0], &sft);
      TRACE_FREG (cpu, rd);
      goto done;
    case MATCH_FSQRT_D:
      TRACE_INSN (cpu, "fsqrt.d %s, %s",
		  frd_name, frs1_name);
      sim_fpu_sqrt (&sft, &sfa);
      sim_fpu_round_64 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to64 (&cpu->fpregs[rd].v[0], &sft);
      TRACE_FREG (cpu, rd);
      goto done;
    }

  switch (op->match & mask_convert)
    {
    case MATCH_FCVT_W_D:
      TRACE_INSN (cpu, "fcvt.w.d %s, %s",
		  rd_name, frs1_name);
      sim_fpu_to32i (&i32, &sfa, rounding);
      cpu->regs[rd].u = i32;
      TRACE_REG (cpu, rd);
      goto done;
    case MATCH_FCVT_WU_D:
      TRACE_INSN (cpu, "fcvt.wu.d %s, %s",
		  rd_name, frs1_name);
      sim_fpu_to32u (&u32, &sfa, rounding);
      i32 = u32;
      cpu->regs[rd].u = i32;
      TRACE_REG (cpu, rd);
      goto done;
    case MATCH_FCVT_D_W:
      TRACE_INSN (cpu, "fcvt.d.w %s, %s",
		  frd_name, rs1_name);
      sim_fpu_i32to (&sft, cpu->regs[rs1].u, rounding);
      sim_fpu_to64 ((unsigned64 *) (cpu->fpregs + rd), &sft);
      TRACE_FREG (cpu, rd);
      goto done;
    case MATCH_FCVT_D_WU:
      TRACE_INSN (cpu, "fcvt.d.wu %s, %s",
		  frd_name, rs1_name);
      sim_fpu_u32to (&sft, cpu->regs[rs1].u, rounding);
      sim_fpu_to64 ((unsigned64 *) (cpu->fpregs + rd), &sft);
      TRACE_FREG (cpu, rd);
      goto done;
    case MATCH_FCVT_S_D:
      TRACE_INSN (cpu, "fcvt.s.d %s, %s",
		  frd_name, frs1_name);
      sft = sfa;
      sim_fpu_round_32 (&sft, sim_fpu_round_near, sim_fpu_denorm_default);
      sim_fpu_to32 ((unsigned32 *) (cpu -> fpregs + rd), &sft);
      TRACE_FREG (cpu, rd);
      goto done;
    case MATCH_FCVT_D_S:
      TRACE_INSN (cpu, "fcvt.d.s %s, %s",
		  frd_name, frs1_name);
      sim_fpu_32to (&sft, cpu->fpregs[rs1].w[0]);
      sim_fpu_to64 (&cpu->fpregs[rd].v[0], &sft);
      TRACE_FREG (cpu, rd);
      goto done;
    case MATCH_FCVT_L_D:
      TRACE_INSN (cpu, "fcvt.l.d %s, %s",
		  rd_name, frs1_name);
      cpu->regs[rd].u = (int64_t) cpu->fpregs[rs1].D[0];
      TRACE_REG (cpu, rd);
      goto done;
    case MATCH_FCVT_LU_D:
      TRACE_INSN (cpu, "fcvt.lu.d %s, %s",
		  rd_name, frs1_name);
      cpu->regs[rd].u = (uint64_t) cpu->fpregs[rs1].D[0];
      TRACE_REG (cpu, rd);
      goto done;
    case MATCH_FCVT_D_L:
      TRACE_INSN (cpu, "fcvt.d.l %s, %s",
		  frd_name, rs1_name);
      cpu->fpregs[rd].D[0] = (double) ((int64_t) cpu->regs[rs1].u);
      TRACE_FREG (cpu, rd);
      goto done;
    case MATCH_FCVT_D_LU:
      TRACE_INSN (cpu, "fcvt.d.lu %s, %s",
		  frd_name, rs1_name);
      cpu->fpregs[rd].D[0] = (double) cpu->regs[rs1].u;
      TRACE_FREG (cpu, rd);
      goto done;
    }

  switch (op->match)
    {
    case MATCH_FLD:
      TRACE_INSN (cpu, "fld %s, %" PRIiTW "(%s)",
		  frd_name, i_imm, rs1_name);
      store_frd64 (cpu, rd,
	sim_core_read_unaligned_8 (cpu, cpu->pc, read_map,
				   cpu->regs[rs1].u + i_imm));
      break;
    case MATCH_FSD:
      TRACE_INSN (cpu, "fsd %s, %" PRIiTW "(%s)",
		  frs2_name, s_imm, rs1_name);
      sim_core_write_unaligned_8 (cpu, cpu->pc, write_map,
				  cpu->regs[rs1].u + s_imm,
				  cpu->fpregs[rs2].v[0]);
      break;
    case MATCH_FSGNJ_D:
      TRACE_INSN (cpu, "fsgnj.d %s, %s, %s",
		  frd_name, frs1_name, frs2_name);
      u32 = cpu->fpregs[rs1].w[1] & 0x7fffffff;
      u32 |= cpu->fpregs[rs2].w[1] & 0x80000000;
      cpu->fpregs[rd].w[1] = u32;
      cpu->fpregs[rd].w[0] = cpu->fpregs[rs1].w[0];
      TRACE_FREG (cpu, rd);
      break;
    case MATCH_FSGNJN_D:
      TRACE_INSN (cpu, "fsgnjn.d %s, %s, %s",
		  frd_name, frs1_name, frs2_name);
      u32 = cpu->fpregs[rs1].w[1] & 0x7fffffff;
      u32 |= (cpu->fpregs[rs2].w[1] & 0x80000000) ^ 0x80000000;
      cpu->fpregs[rd].w[1] = u32;
      cpu->fpregs[rd].w[0] = cpu->fpregs[rs1].w[0];
      TRACE_FREG (cpu, rd);
      break;
    case MATCH_FSGNJX_D:
      TRACE_INSN (cpu, "fsgnjx.d %s, %s, %s",
		  frd_name, frs1_name, frs2_name);
      u32 = cpu->fpregs[rs1].w[1] & 0x7fffffff;
      u32 |= (cpu->fpregs[rs1].w[1] & 0x80000000) ^ (cpu->fpregs[rs2].w[1] & 0x80000000);
      cpu->fpregs[rd].w[1] = u32;
      cpu->fpregs[rd].w[0] = cpu->fpregs[rs1].w[0];
      TRACE_FREG (cpu, rd);
      break;
    case MATCH_FMIN_D:
      TRACE_INSN (cpu, "fmin.d %s, %s, %s",
		  frd_name, frs1_name, frs2_name);
      if (cpu->fpregs[rs1].D[0] < cpu->fpregs[rs2].D[0])
        cpu->fpregs[rd].D[0] = cpu->fpregs[rs1].D[0];
      else
        cpu->fpregs[rd].D[0] = cpu->fpregs[rs2].D[0];
      TRACE_FREG (cpu, rd);
      break;
    case MATCH_FMAX_D:
      TRACE_INSN (cpu, "fmax.d %s, %s, %s",
		  frd_name, frs1_name, frs2_name);
      if (cpu->fpregs[rs1].D[0] > cpu->fpregs[rs2].D[0])
        cpu->fpregs[rd].D[0] = cpu->fpregs[rs1].D[0];
      else
        cpu->fpregs[rd].D[0] = cpu->fpregs[rs2].D[0];
      TRACE_FREG (cpu, rd);
      break;
    case MATCH_FMV_X_D:
      TRACE_INSN (cpu, "fmv.x.d %s, %s",
		  rd_name, frs1_name);
      cpu->regs[rd].u = cpu->fpregs[rs1].v[0];
      TRACE_REG (cpu, rd);
      break;
    case MATCH_FMV_D_X:
      TRACE_INSN (cpu, "fmv.d.x %s, %s",
		  frd_name, frs1_name);
      cpu->fpregs[rd].v[0] = cpu->regs[rs1].u;
      TRACE_FREG (cpu, rd);
      break;
    case MATCH_FEQ_D:
      TRACE_INSN (cpu, "feq.d %s, %s, %s",
		  rd_name, frs1_name, frs2_name);
      cpu->regs[rd].u = sim_fpu_is_eq (&sfa, &sfb);
      TRACE_REG (cpu, rd);
      break;
    case MATCH_FLE_D:
      TRACE_INSN (cpu, "fle.d %s, %s, %s",
		  rd_name, frs1_name, frs2_name);
      cpu->regs[rd].u = sim_fpu_is_le (&sfa, &sfb);
      TRACE_REG (cpu, rd);
      break;
    case MATCH_FLT_D:
      TRACE_INSN (cpu, "flt.d %s, %s, %s",
		  rd_name, frs1_name, frs2_name);
      cpu->regs[rd].u = sim_fpu_is_lt (&sfa, &sfb);
      TRACE_REG (cpu, rd);
      break;
    case MATCH_FCLASS_D:
      TRACE_INSN (cpu, "fclass.d %s, %s",
		  rd_name, frs1_name);
      switch (sim_fpu_is (&sfa))
	{
	case SIM_FPU_IS_NINF:
	  cpu->regs[rd].u = 1;
	  break;
	case SIM_FPU_IS_NNUMBER:
	  cpu->regs[rd].u = 1 << 1;
	  break;
	case SIM_FPU_IS_NDENORM:
	  cpu->regs[rd].u = 1 << 2;
	  break;
	case SIM_FPU_IS_NZERO:
	  cpu->regs[rd].u = 1 << 3;
	  break;
	case SIM_FPU_IS_PZERO:
	  cpu->regs[rd].u = 1 << 4;
	  break;
	case SIM_FPU_IS_PDENORM:
	  cpu->regs[rd].u = 1 << 5;
	  break;
	case SIM_FPU_IS_PNUMBER:
	  cpu->regs[rd].u = 1 << 6;
	  break;
	case SIM_FPU_IS_PINF:
	  cpu->regs[rd].u = 1 << 7;
	  break;
	case SIM_FPU_IS_SNAN:
	  cpu->regs[rd].u = 1 << 8;
	  break;
	case SIM_FPU_IS_QNAN:
	  cpu->regs[rd].u = 1 << 9;
	  break;
	}
      TRACE_REG (cpu, rd);
      break;
    default:
      TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
      sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
    }

 done:
  return pc;

}

static sim_cia
execute_f (SIM_CPU *cpu, unsigned_word iw, const struct riscv_opcode *op, int ex9)
{
  SIM_DESC sd = CPU_STATE (cpu);
  unsigned int mask_arithmetic = MASK_FADD_S;
  unsigned int mask_mul_add = MASK_FMADD_S;
  unsigned int mask_convert = MASK_FCVT_S_W;

  static const int round_modes[] =
  {
      sim_fpu_round_near, sim_fpu_round_zero,
      sim_fpu_round_down, sim_fpu_round_up,
      sim_fpu_round_default, sim_fpu_round_default,
      sim_fpu_round_default
  };

  int rd = (iw >> OP_SH_RD) & OP_MASK_RD;
  int rs1 = (iw >> OP_SH_RS1) & OP_MASK_RS1;
  int rs2 = (iw >> OP_SH_RS2) & OP_MASK_RS2;
  int rs3 = (iw >> OP_SH_RS3) & OP_MASK_RS3;
  const char *frd_name = riscv_fpr_names_abi[rd];
  const char *frs1_name = riscv_fpr_names_abi[rs1];
  const char *frs2_name = riscv_fpr_names_abi[rs2];
  const char *frs3_name = riscv_fpr_names_abi[rs3];
  const char *rd_name = riscv_gpr_names_abi[rd];
  const char *rs1_name = riscv_gpr_names_abi[rs1];
  unsigned_word i_imm = EXTRACT_ITYPE_IMM (iw);
  unsigned_word s_imm = EXTRACT_STYPE_IMM (iw);
  uint32_t u32;
  int32_t i32;
  int64_t i64;
  uint64_t u64;
  sim_cia pc = cpu->pc + 4;
  /* Rounding mode.  */
  int rm = (iw >> OP_SH_RM) & OP_MASK_RM;
  int rounding = round_modes[rm];
  int zfh = 0;

  sim_fpu sft, sft2;
  sim_fpu sfa, sfb, sfc;
  sim_fpu_32to (&sfa, cpu->fpregs[rs1].w[0]);
  sim_fpu_32to (&sfb, cpu->fpregs[rs2].w[0]);

  if (ex9)
    pc -= 2;

  for (int i = 0; op->subset[i]; i++)
    {
      if (strncmp(op->subset[i], "ZFH", 3) == 0)
	zfh = 1;
    }

  if (zfh)
    {
    switch (op->match & mask_convert)
      {
      case MATCH_FCVT_S_H:
        TRACE_INSN (cpu, "fcvt.s.h %s, %s",
		    frd_name, frs1_name);
        cpu->fpregs[rd].f[0] = f16_to_f32(cpu->fpregs[rs1].hf[0]);
        TRACE_FREG (cpu, rd);
        goto done;
      case MATCH_FCVT_H_S:
        TRACE_INSN (cpu, "fcvt.h.s %s, %s",
		    frd_name, frs1_name);
        cpu->fpregs[rd].hf[0] = f32_to_f16(cpu->fpregs[rs1].f[0]);
        TRACE_FREG (cpu, rd);
        goto done;
      }
    }

  switch (op->match & mask_mul_add)
    {
    case MATCH_FMADD_S:
      TRACE_INSN (cpu, "fmadd.s %s, %s, %s, %s",
		  frd_name, frs1_name, frs2_name, frs3_name);
      sim_fpu_32to (&sfc, cpu->fpregs[rs3].w[0]);
      sim_fpu_mul (&sft2, &sfa, &sfb);
      sim_fpu_add (&sft, &sfc, &sft2);
      sim_fpu_round_32 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to32 (&cpu->fpregs[rd].w[0], &sft);
      TRACE_FREG (cpu, rd);
      goto done;
    case MATCH_FMSUB_S:
      TRACE_INSN (cpu, "fmsub.s %s, %s, %s, %s",
		  frd_name, frs1_name, frs2_name, frs3_name);
      sim_fpu_32to (&sfc, cpu->fpregs[rs3].w[0]);
      sim_fpu_mul (&sft2, &sfa, &sfb);
      sim_fpu_sub (&sft, &sft2, &sfc);
      sim_fpu_round_32 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to32 (&cpu->fpregs[rd].w[0], &sft);
      TRACE_FREG (cpu, rd);
      goto done;
    case MATCH_FNMADD_S:
      TRACE_INSN (cpu, "fnmadd.s %s, %s, %s, %s",
		  frd_name, frs1_name, frs2_name, frs3_name);
      sim_fpu_32to (&sfc, cpu->fpregs[rs3].w[0]);
      sim_fpu_neg (&sfa, &sfa);
      sim_fpu_mul (&sft2, &sfa, &sfb);
      sim_fpu_sub (&sft, &sft2, &sfc);
      sim_fpu_round_32 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to32 (&cpu->fpregs[rd].w[0], &sft);
      TRACE_FREG (cpu, rd);
      goto done;
    case MATCH_FNMSUB_S:
      TRACE_INSN (cpu, "fnmsub.s %s, %s, %s, %s",
		  frd_name, frs1_name, frs2_name, frs3_name);
      sim_fpu_32to (&sfc, cpu->fpregs[rs3].w[0]);
      sim_fpu_neg (&sfa, &sfa);
      sim_fpu_mul (&sft2, &sfa, &sfb);
      sim_fpu_add (&sft, &sft2, &sfc);
      sim_fpu_round_32 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to32 (&cpu->fpregs[rd].w[0], &sft);
      TRACE_FREG (cpu, rd);
      goto done;
    }

  switch (op->match & mask_arithmetic)
    {
    case MATCH_FADD_S:
      TRACE_INSN (cpu, "fadd.s %s, %s, %s",
		  frd_name, frs1_name, frs2_name);
      sim_fpu_add (&sft, &sfa, &sfb);
      sim_fpu_round_32 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to32 (&cpu->fpregs[rd].w[0], &sft);
      goto done;
    case MATCH_FSUB_S:
      TRACE_INSN (cpu, "fsub.s %s, %s, %s",
		  frd_name, frs1_name, frs2_name);
      sim_fpu_sub (&sft, &sfa, &sfb);
      sim_fpu_round_32 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to32 (&cpu->fpregs[rd].w[0], &sft);
      TRACE_FREG (cpu, rd);
      goto done;
    case MATCH_FMUL_S:
      TRACE_INSN (cpu, "fmul.s %s, %s, %s",
		  frd_name, frs1_name, frs2_name);
      sim_fpu_mul (&sft, &sfa, &sfb);
      sim_fpu_round_64 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_round_32 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to32 (&cpu->fpregs[rd].w[0], &sft);
      TRACE_FREG (cpu, rd);
      goto done;
    case MATCH_FDIV_S:
      TRACE_INSN (cpu, "fdiv.s %s, %s, %s",
		  frd_name, frs1_name, frs2_name);
      sim_fpu_div (&sft, &sfa, &sfb);
      sim_fpu_round_32 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to32 (&cpu->fpregs[rd].w[0], &sft);
      TRACE_FREG (cpu, rd);
      goto done;
    case MATCH_FSQRT_S:
      TRACE_INSN (cpu, "fsqrt.s %s, %s, %s",
		  frd_name, frs1_name, frs2_name);
      sim_fpu_sqrt (&sft, &sfa);
      sim_fpu_to32 (&cpu->fpregs[rd].w[0], &sft);
      TRACE_FREG (cpu, rd);
      goto done;
    }

  switch (op->match & mask_convert)
    {
    case MATCH_FCVT_W_S:
      TRACE_INSN (cpu, "fcvt.w.s %s, %s",
		  rd_name, frs1_name);
      sim_fpu_to32i (&i32, &sfa, rounding);
      cpu->regs[rd].u = i32;
      TRACE_REG (cpu, rd);
      goto done;
    case MATCH_FCVT_WU_S:
      TRACE_INSN (cpu, "fcvt.wu.s %s, %s",
		  rd_name, frs1_name);
      sim_fpu_to32u (&u32, &sfa, rounding);
      i32 = u32;
      cpu->regs[rd].u = i32;
      TRACE_REG (cpu, rd);
      goto done;
    case MATCH_FCVT_S_W:
      TRACE_INSN (cpu, "fcvt.s.w %s, %s",
		  frd_name, rs1_name);
      sim_fpu_i32to (&sft, cpu->regs[rs1].u, rounding);
      sim_fpu_round_32 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to32 ((unsigned32 *) (cpu->fpregs + rd), &sft);
      TRACE_FREG (cpu, rd);
      goto done;
    case MATCH_FCVT_S_WU:
      TRACE_INSN (cpu, "fcvt.s.wu %s, %s",
		  frd_name, rs1_name);
      sim_fpu_u32to (&sft, cpu->regs[rs1].u, rounding);
      sim_fpu_round_32 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to32 ((unsigned32 *) (cpu->fpregs + rd), &sft);
      TRACE_FREG (cpu, rd);
      goto done;
    case MATCH_FCVT_L_S:
      TRACE_INSN (cpu, "fcvt.l.s %s, %s",
		  rd_name, frs1_name);
      cpu->regs[rd].u = (int64_t) cpu->fpregs[rs1].S[0];
      TRACE_REG (cpu, rd);
      goto done;
    case MATCH_FCVT_LU_S:
      TRACE_INSN (cpu, "fcvt.lu.s %s, %s",
		  rd_name, frs1_name);
      cpu->regs[rd].u = (uint64_t) cpu->fpregs[rs1].S[0];
      TRACE_REG (cpu, rd);
      goto done;
    case MATCH_FCVT_S_L:
      TRACE_INSN (cpu, "fcvt.s.l %s, %s",
		  frd_name, rs1_name);
      cpu->fpregs[rd].S[0] = (float) ((int64_t) cpu->regs[rs1].u);
      TRACE_FREG (cpu, rd);
      goto done;
    case MATCH_FCVT_S_LU:
      TRACE_INSN (cpu, "fcvt.s.lu %s, %s",
		  frd_name, rs1_name);
      cpu->fpregs[rd].S[0] = (float) cpu->regs[rs1].u;
      TRACE_FREG (cpu, rd);
      goto done;
    }

  switch (op->match)
    {
    case MATCH_FLW:
      TRACE_INSN (cpu, "flw %s, %" PRIiTW "(%s)",
		  frd_name, i_imm, rs1_name);
      store_frd (cpu, rd, EXTEND32 (
	sim_core_read_unaligned_4 (cpu, cpu->pc, read_map,
				   cpu->regs[rs1].u + i_imm)));
      break;
    case MATCH_FSW:
      TRACE_INSN (cpu, "fsw %s, %" PRIiTW "(%s)",
		  frs2_name, s_imm, rs1_name);
      sim_core_write_unaligned_4 (cpu, cpu->pc, write_map,
				  cpu->regs[rs1].u + s_imm, cpu->fpregs[rs2].w[0]);
      break;
    case MATCH_FSGNJ_S:
      TRACE_INSN (cpu, "fsgnj.s %s, %s, %s",
		  frd_name, frs1_name, frs2_name);
      u32 = cpu->fpregs[rs1].w[0] & 0x7fffffff;
      u32 |= cpu->fpregs[rs2].w[0] & 0x80000000;
      cpu->fpregs[rd].w[0] = u32;
      TRACE_FREG (cpu, rd);
      break;
    case MATCH_FSGNJN_S:
      TRACE_INSN (cpu, "fsgnjn.s %s, %s, %s",
		  frd_name, frs1_name, frs2_name);
      u32 = cpu->fpregs[rs1].w[0] & 0x7fffffff;
      u32 |= (cpu->fpregs[rs2].w[0] & 0x80000000) ^ 0x80000000;
      cpu->fpregs[rd].w[0] = u32;
      TRACE_FREG (cpu, rd);
      break;
    case MATCH_FSGNJX_S:
      TRACE_INSN (cpu, "fsgnx.s %s, %s, %s",
		  frd_name, frs1_name, frs2_name);
      u32 = cpu->fpregs[rs1].w[0] & 0x7fffffff;
      u32 |= (cpu->fpregs[rs1].w[0] & 0x80000000) ^ (cpu->fpregs[rs2].w[0] & 0x80000000);
      cpu->fpregs[rd].w[0] = u32;
      TRACE_FREG (cpu, rd);
      break;
    case MATCH_FMIN_S:
      TRACE_INSN (cpu, "fmin.s %s, %s, %s",
		  frd_name, frs1_name, frs2_name);
      if (cpu->fpregs[rs1].S[0] < cpu->fpregs[rs2].S[0])
        cpu->fpregs[rd].S[0] = cpu->fpregs[rs1].S[0];
      else
        cpu->fpregs[rd].S[0] = cpu->fpregs[rs2].S[0];
      TRACE_FREG (cpu, rd);
      break;
    case MATCH_FMAX_S:
      TRACE_INSN (cpu, "fmax.s %s, %s, %s",
		  frd_name, frs1_name, frs2_name);
      if (cpu->fpregs[rs1].S[0] > cpu->fpregs[rs2].S[0])
        cpu->fpregs[rd].S[0] = cpu->fpregs[rs1].S[0];
      else
        cpu->fpregs[rd].S[0] = cpu->fpregs[rs2].S[0];
      TRACE_FREG (cpu, rd);
      break;
    case MATCH_FMV_X_S:
      TRACE_INSN (cpu, "fmv.x.s %s, %s",
		  rd_name, frs1_name);
      cpu->regs[rd].u = cpu->fpregs[rs1].W[0];
      TRACE_REG (cpu, rd);
      break;
    case MATCH_FMV_S_X:
      TRACE_INSN (cpu, "fmv.s.x %s, %s",
		  frd_name, rs1_name);
      cpu->fpregs[rd].w[0] = cpu->regs[rs1].u;
      TRACE_FREG (cpu, rd);
      break;
    case MATCH_FEQ_S:
      TRACE_INSN (cpu, "feq.s %s, %s, %s",
		  rd_name, frs1_name, frs2_name);
      cpu->regs[rd].u = sim_fpu_is_eq (&sfa, &sfb);
      TRACE_REG (cpu, rd);
      break;
    case MATCH_FLE_S:
      TRACE_INSN (cpu, "fle.s %s, %s, %s",
		  rd_name, frs1_name, frs2_name);
      cpu->regs[rd].u = sim_fpu_is_le (&sfa, &sfb);
      TRACE_REG (cpu, rd);
      break;
    case MATCH_FLT_S:
      TRACE_INSN (cpu, "flt.s %s, %s, %s",
		  rd_name, frs1_name, frs2_name);
      cpu->regs[rd].u = sim_fpu_is_lt (&sfa, &sfb);
      TRACE_REG (cpu, rd);
      break;
    case MATCH_FCLASS_S:
      TRACE_INSN (cpu, "fclass.s %s, %s",
		  rd_name, frs1_name);
      switch (sim_fpu_is (&sfa))
	{
	case SIM_FPU_IS_NINF:
	  cpu->regs[rd].u = 1;
	  break;
	case SIM_FPU_IS_NNUMBER:
	  cpu->regs[rd].u = 1 << 1;
	  break;
	case SIM_FPU_IS_NDENORM:
	  cpu->regs[rd].u = 1 << 2;
	  break;
	case SIM_FPU_IS_NZERO:
	  cpu->regs[rd].u = 1 << 3;
	  break;
	case SIM_FPU_IS_PZERO:
	  cpu->regs[rd].u = 1 << 4;
	  break;
	case SIM_FPU_IS_PDENORM:
	  cpu->regs[rd].u = 1 << 5;
	  break;
	case SIM_FPU_IS_PNUMBER:
	  cpu->regs[rd].u = 1 << 6;
	  break;
	case SIM_FPU_IS_PINF:
	  cpu->regs[rd].u = 1 << 7;
	  break;
	case SIM_FPU_IS_SNAN:
	  cpu->regs[rd].u = 1 << 8;
	  break;
	case SIM_FPU_IS_QNAN:
	  cpu->regs[rd].u = 1 << 9;
	  break;
	}
      TRACE_REG (cpu, rd);
      break;
    case MATCH_FRCSR:
      TRACE_INSN (cpu, "frcsr %s",
		  rd_name);
      store_rd (cpu, rd, fetch_csr (cpu, "fcsr", CSR_FCSR, &cpu->csr.fcsr));
      break;
    case MATCH_FSCSR:
      TRACE_INSN (cpu, "fscsr %s, %sf",
		  rd_name, rs1_name);
      store_rd (cpu, rd, fetch_csr (cpu, "fcsr", CSR_FCSR, &cpu->csr.fcsr));
      store_csr (cpu, "fcsr", CSR_FCSR, &cpu->csr.fcsr, cpu->regs[rs1].u);
      break;
    case MATCH_FRRM:
      TRACE_INSN (cpu, "frrm %s",
		  rd_name);
      store_rd (cpu, rd, fetch_csr (cpu, "frm", CSR_FRM, &cpu->csr.frm));
      break;
    case MATCH_FSRM:
      TRACE_INSN (cpu, "fsrm %s, %s",
		  rd_name, rs1_name);
      store_rd (cpu, rd, fetch_csr (cpu, "frm", CSR_FCSR, &cpu->csr.frm));
      store_csr (cpu, "frm", CSR_FCSR, &cpu->csr.frm, cpu->regs[rs1].u);
      break;
    case MATCH_FRFLAGS:
      TRACE_INSN (cpu, "frflags %s",
		  rd_name);
      store_rd (cpu, rd, fetch_csr (cpu, "fflags", CSR_FFLAGS, &cpu->csr.fflags));
      break;
    case MATCH_FSFLAGS:
      TRACE_INSN (cpu, "fsflags %s, %s",
		  rd_name, frs1_name);
      store_rd (cpu, rd, fetch_csr (cpu, "fflags", CSR_FFLAGS, &cpu->csr.fflags));
      store_csr (cpu, "fflags", CSR_FFLAGS, &cpu->csr.fflags, cpu->regs[rs1].u);
      break;
    case MATCH_FLHW:
      {
        union32_t val;
	TRACE_INSN (cpu, "flhw %s, %" PRIiTW "(%s)",
		    frd_name, i_imm, rs1_name);
	val.h[0] = sim_core_read_unaligned_2 (cpu, cpu->pc,
					      read_map,
					      cpu->regs[rs1].u + i_imm);
	/* Round towards Zero. */
	softfloat_roundingMode = softfloat_round_minMag;
	cpu->fpregs[rd].f[0] = f16_to_f32(val.hf[0]);
	TRACE_FREG (cpu, rd);
        break;
      }
    case MATCH_FSHW:
      {
        union32_t val;
        TRACE_INSN (cpu, "fshw %s, %" PRIiTW "(%s)",
		    frs2_name, s_imm, rs1_name);
	/* Round towards Zero. */
	softfloat_roundingMode = softfloat_round_minMag;
	val.hf[0] = f32_to_f16 (cpu->fpregs[rs2].f[0]);
	sim_core_write_unaligned_2 (cpu, cpu->pc, write_map,
				    cpu->regs[rs1].u + s_imm, val.h[0]);
        break;
      }
    default:
      TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
      sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
    }

 done:
  return pc;
}

static sim_cia
execute_c (SIM_CPU *cpu, unsigned_word iw, const struct riscv_opcode *op)
{
  SIM_DESC sd = CPU_STATE (cpu);
  const int mask_group_op = 0x3;
  const int mask_mv_jr = 0xf003;
  const int match_mv_jr = 0x8002;
  const int mask_ebk_jalr_add = 0xf003;
  const int match_ebk_jalr_add = 0x9002;

  int rd = (iw >> OP_SH_RD) & OP_MASK_RD;
  int crs2 = (iw >> OP_SH_CRS2) & OP_MASK_CRS2;
  int crs1s = ((iw >> OP_SH_CRS1S) & OP_MASK_CRS1S) | 0x8;
  int crs2s = ((iw >> OP_SH_CRS2S) & OP_MASK_CRS2S) | 0x8;
  int ciw_rd = crs2s;
  unsigned_word rvc_imm = EXTRACT_RVC_IMM (iw);
  unsigned_word tmp;
  int eh_rve_p = cpu->elf_flags & 0x8;
  sim_cia pc = cpu->pc + 2;

  const char *rd_name = riscv_gpr_names_abi[rd];
  const char *crs2_name = riscv_gpr_names_abi[crs2];
  const char *crs1s_name = riscv_gpr_names_abi[crs1s];
  const char *crs2s_name = riscv_gpr_names_abi[crs2s];
  const char *ciw_rd_name = crs2s_name;

  const char *frd_name = riscv_fpr_names_abi[rd];
  const char *fcrs2_name = riscv_fpr_names_abi[crs2];
  const char *fcrs1s_name = riscv_fpr_names_abi[crs1s];
  const char *fcrs2s_name = riscv_fpr_names_abi[crs2s];
  const char *fciw_rd_name = fcrs2s_name;

  /* Deal with c.mv, c.jr instructons.  */
  if ((op->match & mask_mv_jr) == match_mv_jr)
    {
      if (crs2 != 0)
	{
	  /* c.mv */
	  TRACE_INSN (cpu, "c.mv %s, %s // %s = %s",
		      rd_name, crs2_name, rd_name, crs2_name);
	  store_rd (cpu, rd, cpu->regs[crs2].u);
	}
      else
	{
	  /* c.jr */
	  TRACE_INSN (cpu, "c.jr %s", rd_name);
	  pc = cpu->regs[rd].u;
	  TRACE_BRANCH (cpu, "to %#"PRIxTW, pc);
	}
      return pc;
    }

  /* Deal with c.ebreak, c.jalr, c.add instructions.  */
  if ((op->match & mask_ebk_jalr_add) == match_ebk_jalr_add)
    {
      if (iw == MATCH_C_EBREAK)
	{
	  /* c.ebreak */
	  TRACE_INSN (cpu, "c.break");
	  sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_stopped, SIM_SIGTRAP);
	}
      else if (crs2 == 0)
	{
	  /* c.jalr */
	  TRACE_INSN (cpu, "c.jalr %s", rd_name);
	  pc = cpu->regs[rd].u;
	  store_rd (cpu, X_RA, cpu->pc + 2);
	  TRACE_BRANCH (cpu, "to %#"PRIxTW, pc);
	}
      else
	{
	  /* c.add */
	  TRACE_INSN (cpu, "c.add %s, %s // %s += %s",
		      rd_name, crs2_name, rd_name, crs2_name);
	  store_rd (cpu, rd, cpu->regs[rd].u + cpu->regs[crs2].u);
	}
      return pc;
    }

  switch (op->match & mask_group_op)
    {
    case 0:
      switch (op->match)
	{
	case MATCH_C_LW:
	  TRACE_INSN (cpu, "c.lw %s, %" PRIiTW "(%s);"
			   " // %s = *(%s + %" PRIiTW ")",
		      crs2s_name, EXTRACT_RVC_LW_IMM (iw), crs1s_name,
		      crs2s_name, crs1s_name, EXTRACT_RVC_LW_IMM (iw));
	  store_rd (cpu, crs2s, EXTEND32 (
	    sim_core_read_unaligned_4 (cpu, cpu->pc, read_map,
				       cpu->regs[crs1s].u
				       + EXTRACT_RVC_LW_IMM (iw))));
	  return pc;
	case MATCH_C_SW:
	  TRACE_INSN (cpu, "c.sw %s, %" PRIiTW "(%s);"
			   " // *(%s + %" PRIiTW ") = %s",
		      crs2s_name, EXTRACT_RVC_LW_IMM (iw), crs1s_name,
		      crs1s_name, EXTRACT_RVC_LW_IMM (iw), crs2s_name);
	  sim_core_write_unaligned_4 (cpu, cpu->pc, write_map,
				      (cpu->regs[crs1s].u
				       + EXTRACT_RVC_LW_IMM (iw)),
				      cpu->regs[crs2s].u);
	  return pc;
	case MATCH_C_ADDI4SPN:
	  TRACE_INSN (cpu, "c.addi4spn %s, %" PRIiTW
			   " // %s = sp + %" PRIiTW,
		      ciw_rd_name, EXTRACT_RVC_ADDI4SPN_IMM (iw),
		      ciw_rd_name, EXTRACT_RVC_ADDI4SPN_IMM (iw));
	  store_rd (cpu, ciw_rd, cpu->sp.u + EXTRACT_RVC_ADDI4SPN_IMM (iw));
	  return pc;
	case MATCH_C_FLD:
	  if (RISCV_XLEN (cpu) <= 64)
	    {

	      TRACE_INSN (cpu, "c.fld %s, %" PRIiTW "(%s);"
			       " // %s = *(%s + %" PRIiTW ")",
			  fcrs2s_name, EXTRACT_RVC_LD_IMM (iw), fcrs1s_name,
			  fcrs2s_name, fcrs1s_name, EXTRACT_RVC_LD_IMM (iw));
	      /* rv32/64, c.fld instruction.  */
	      store_frd64 (cpu, crs2s,
		sim_core_read_unaligned_8 (cpu, cpu->pc, read_map,
					   cpu->regs[crs1s].u
					   + EXTRACT_RVC_LD_IMM (iw)));
	      return pc;
	    }
	  else
	    {
	      /* rv128, c.lq instruction.  */
	      TRACE_INSN (cpu, "UNHANDLED RV128 INSN: %s", op->name);
	      sim_engine_halt (sd, cpu, NULL, cpu->pc,
			       sim_signalled, SIM_SIGILL);
	    }
	case MATCH_C_FLW:
	  /* rv32: c.flw, rv64: c.ld.  */
	  if (RISCV_XLEN (cpu) == 32)
	    {
	      TRACE_INSN (cpu, "c.flw %s, %" PRIiTW "(%s);"
			       " // *(%s + %" PRIiTW ") = %s",
			  fcrs2s_name, EXTRACT_RVC_LW_IMM (iw), crs1s_name,
			  crs1s_name, EXTRACT_RVC_LW_IMM (iw), fcrs2s_name);
	      store_frd (cpu, crs2s, EXTEND32 (
		sim_core_read_unaligned_4 (cpu, cpu->pc, read_map,
					   cpu->regs[crs1s].u
					   + EXTRACT_RVC_LW_IMM (iw))));
	    }
	  else
	    {
	      TRACE_INSN (cpu, "c.ld %s, %" PRIiTW "(%s);"
			       " // *(%s + %" PRIiTW ") = %s",
			  crs2s_name, EXTRACT_RVC_LD_IMM (iw), crs1s_name,
			  crs1s_name, EXTRACT_RVC_LD_IMM (iw), crs2s_name);
	      store_rd (cpu, crs2s,
		sim_core_read_unaligned_8 (cpu, cpu->pc, read_map,
					   cpu->regs[crs1s].u
					   + EXTRACT_RVC_LD_IMM (iw)));
	    }
	  return pc;
	case MATCH_C_FSD:
	  if (RISCV_XLEN (cpu) <= 64)
	    {
	      /* rv32/64, c.fsd instruction.  */
	      TRACE_INSN (cpu, "c.fsd %s, %" PRIiTW "(%s);"
			       " // *(%s + %" PRIiTW ") = %s",
			  fcrs2s_name, EXTRACT_RVC_LD_IMM (iw), crs1s_name,
			  crs1s_name, EXTRACT_RVC_LD_IMM (iw), fcrs2s_name);
	      sim_core_write_unaligned_8 (cpu, cpu->pc, write_map,
					  cpu->regs[crs1s].u
					  + EXTRACT_RVC_LD_IMM (iw),
					  cpu->fpregs[crs2s].v[0]);
	      return pc;
	    }
	  else
	    {
	      /* rv128, c.sq instruction.  */
	      TRACE_INSN (cpu, "UNHANDLED RV128 INSN: %s", op->name);
	      sim_engine_halt (sd, cpu, NULL, cpu->pc,
			       sim_signalled, SIM_SIGILL);
	    }
	case MATCH_C_FSW:
	  /* rv32: c.fsw, rv64: c.sd.  */
	  if (RISCV_XLEN (cpu) == 32)
	    {
	      TRACE_INSN (cpu, "c.fsw %s, %" PRIiTW "(%s);"
			       " // *(%s + %" PRIiTW ") = %s",
			  fcrs2s_name, EXTRACT_RVC_LW_IMM (iw), crs1s_name,
			  crs1s_name, EXTRACT_RVC_LW_IMM (iw), fcrs2s_name);
	      sim_core_write_unaligned_4 (cpu, cpu->pc, write_map,
					  cpu->regs[crs1s].u
					  + EXTRACT_RVC_LW_IMM (iw),
					  cpu->fpregs[crs2s].w[0]);
	    }
	  else
	    {
	      TRACE_INSN (cpu, "c.sd %s, %" PRIiTW "(%s);"
			       " // *(%s + %" PRIiTW ") = %s",
			  crs2s_name, EXTRACT_RVC_LD_IMM (iw), crs1s_name,
			  crs1s_name, EXTRACT_RVC_LD_IMM (iw), crs2s_name);
	      sim_core_write_unaligned_8 (cpu, cpu->pc, write_map,
					  cpu->regs[crs1s].u
					  + EXTRACT_RVC_LD_IMM (iw),
					  cpu->regs[crs2s].u);
	    }
	  return pc;
	case MATCH_C_EXECIT:
	  iw = sim_core_read_unaligned_4 (cpu, cpu->pc, exec_map,
					  cpu->csr.uitb + EXTRACT_RVC_EXECIT_IMM (iw));
	  pc = riscv_decode (cpu, iw, cpu->pc, 1);
	  return pc;
	default:
	  TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	  sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
	}
    case 1:
      switch (op->match)
	{
	case MATCH_C_ADDI:
	  if (rd != 0)
	    {
	      /* c.addi */
	      TRACE_INSN (cpu, "c.addi %s, %" PRIiTW " // %s += %" PRIiTW,
			  rd_name, rvc_imm, rd_name, rvc_imm);
              if (!eh_rve_p && rd == 2 && ((cpu->regs[rd].u + rvc_imm) & 0xf) != 0)
                {
                  fprintf (stderr, "Stack pointer is not aligned to 16-byte boundary.\n");
                  sim_engine_halt (sd, cpu, NULL, cpu->pc,
            	         	   sim_signalled, SIM_SIGILL);
                }
	      store_rd (cpu, rd, cpu->regs[rd].u + rvc_imm);
	      return pc;
	    }
	  else
	    {
	      /* c.nop */
	      TRACE_INSN (cpu, "c.nop");
	      return pc;
	    }
	case MATCH_C_JAL:
	  /* In rv32 is c.jal, rv64 c.addiw.  */
	  if (RISCV_XLEN (cpu) == 32)
	    {
	      TRACE_INSN (cpu, "c.jal %" PRIiTW, EXTRACT_RVC_J_IMM (iw));
	      store_rd (cpu, X_RA, cpu->pc + 2);
	      pc = cpu->pc + EXTRACT_RVC_J_IMM (iw);
	      TRACE_BRANCH (cpu, "to %#"PRIxTW, pc);
	    }
	  else
	    {
	      TRACE_INSN (cpu, "c.addiw %s, %" PRIiTW " // %s += %" PRIiTW,
			  rd_name, rvc_imm, rd_name, rvc_imm);
	      store_rd (cpu, rd, EXTEND32 (cpu->regs[rd].u + rvc_imm));
	    }
	  return pc;
	case MATCH_C_LI:
	  TRACE_INSN (cpu, "c.li %s, %" PRIiTW " // %s = %" PRIiTW,
		      rd_name, rvc_imm, rd_name, rvc_imm);
	  store_rd (cpu, rd, rvc_imm);
	  return pc;
	case MATCH_C_ADDI16SP:
	  TRACE_INSN (cpu, "c.addi16sp %s, %" PRIiTW,
		      rd_name, rvc_imm);
	  if (!eh_rve_p && (cpu->sp.u & 0xf) != 0)
	    {
	      fprintf (stderr, "Stack pointer is not aligned to 16-byte boundary.\n");
	      sim_engine_halt (sd, cpu, NULL, cpu->pc,
	      		       sim_signalled, SIM_SIGILL);
	    }
	  store_rd (cpu, rd, cpu->sp.u + EXTRACT_RVC_ADDI16SP_IMM (iw));
	  return pc;
	case MATCH_C_SRLI:
	  /* rv32: c.srli, rv128: c.srli64.  */
	  TRACE_INSN (cpu, "c.srli %s, %" PRIiTW,
		      crs1s_name, EXTRACT_RVC_IMM (iw));
	  if (RISCV_XLEN (cpu) == 32 && EXTRACT_RVC_IMM (iw) > 0x1f)
	    sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
	  store_rd (cpu, crs1s, cpu->regs[crs1s].u >> EXTRACT_RVC_IMM (iw));
	  return pc;
	case MATCH_C_SRAI:
	  /* rv32: c.srli, rv128: c.srli64.  */
	  TRACE_INSN (cpu, "c.srai %s, %" PRIiTW,
		      crs1s_name, EXTRACT_RVC_IMM (iw));
	  if (RISCV_XLEN (cpu) == 32)
	    {
	      if (EXTRACT_RVC_IMM (iw) > 0x1f)
		sim_engine_halt (sd, cpu, NULL, cpu->pc,
				 sim_signalled, SIM_SIGILL);
	      tmp = ashiftrt (cpu->regs[crs1s].u, EXTRACT_RVC_IMM (iw));
	    }
	  else
	    tmp = ashiftrt64 (cpu->regs[crs1s].u, EXTRACT_RVC_IMM (iw));
	  store_rd (cpu, crs1s, tmp);
	  return pc;
	case MATCH_C_ANDI:
	  TRACE_INSN (cpu, "c.andi %s, %" PRIiTW,
		      crs1s_name, EXTRACT_RVC_IMM (iw));
	  store_rd (cpu, crs1s, cpu->regs[crs1s].u & EXTRACT_RVC_IMM (iw));
	  return pc;
	case MATCH_C_SUB:
	  TRACE_INSN (cpu, "c.sub %s, %s",
		      crs1s_name, crs2s_name);
 	  store_rd (cpu, crs1s, cpu->regs[crs1s].u - cpu->regs[crs2s].u);
	  return pc;
	case MATCH_C_XOR:
	  TRACE_INSN (cpu, "c.xor %s, %s",
		      crs1s_name, crs2s_name);
	  store_rd (cpu, crs1s, cpu->regs[crs1s].u ^ cpu->regs[crs2s].u);
	  return pc;
	case MATCH_C_OR:
	  TRACE_INSN (cpu, "c.or %s, %s",
		      crs1s_name, crs2s_name);
	  store_rd (cpu, crs1s, cpu->regs[crs1s].u | cpu->regs[crs2s].u);
	  return pc;
	case MATCH_C_AND:
	  TRACE_INSN (cpu, "c.and %s, %s",
		      crs1s_name, crs2s_name);
	  store_rd (cpu, crs1s, cpu->regs[crs1s].u & cpu->regs[crs2s].u);
	  return pc;
	case MATCH_C_SUBW:
	  TRACE_INSN (cpu, "c.subw %s, %s",
		      crs1s_name, crs2s_name);
	  RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
	  store_rd (cpu, crs1s, EXTEND32 (cpu->regs[crs1s].u - cpu->regs[crs2s].u));
	  return pc;
	case MATCH_C_ADDW:
	  TRACE_INSN (cpu, "c.addw %s, %s",
		      crs1s_name, crs2s_name);
	  RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
	  store_rd (cpu, crs1s, EXTEND32 (cpu->regs[crs1s].u + cpu->regs[crs2s].u));
	  return pc;
	case MATCH_C_BEQZ:
	  TRACE_INSN (cpu, "c.beqz %s, %" PRIiTW,
		      crs1s_name, cpu->pc + EXTRACT_RVC_B_IMM (iw));
	  if (cpu->regs[crs1s].u == 0)
	    {
	      pc = cpu->pc + EXTRACT_RVC_B_IMM (iw);
	      TRACE_BRANCH (cpu, "to %#"PRIxTW, pc);
	    }
	  return pc;
	case MATCH_C_BNEZ:
	  TRACE_INSN (cpu, "c.bnez %s, %" PRIiTW,
		      crs1s_name, cpu->pc + EXTRACT_RVC_B_IMM (iw));
	  if (cpu->regs[crs1s].u != 0)
	    {
	      pc = cpu->pc + EXTRACT_RVC_B_IMM (iw);
	      TRACE_BRANCH (cpu, "to %#"PRIxTW, pc);
	    }
	  return pc;
	case MATCH_C_LUI:
	  TRACE_INSN (cpu, "c.lui %s, %" PRIiTW,
		      rd_name, EXTRACT_RVC_LUI_IMM (iw));
	  store_rd (cpu, rd, EXTRACT_RVC_LUI_IMM (iw));
	  return pc;
	case MATCH_C_J:
	  TRACE_INSN (cpu, "c.j %" PRIiTW,
		      cpu->pc + EXTRACT_RVC_B_IMM (iw));
	  pc = cpu->pc + EXTRACT_RVC_J_IMM (iw);
	  TRACE_BRANCH (cpu, "to %#"PRIxTW, pc);
	  return pc;
	default:
	  TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	  sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
	}
    case 2:
      switch (op->match)
	{
	case MATCH_C_SLLI:
	  TRACE_INSN (cpu, "c.slli %s, %" PRIiTW,
		      rd_name, rvc_imm);
	  /* rv32: c.slli, rv128: c.slli64.  */
	  if (RISCV_XLEN (cpu) == 32 && rvc_imm > 0x1f)
	    sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
	  store_rd (cpu, rd, cpu->regs[rd].u << rvc_imm);
	  return pc;
	case MATCH_C_LWSP:
	  TRACE_INSN (cpu, "c.lwsp %s, %" PRIiTW "(sp);"
			   " // %s = *(sp + %" PRIiTW ")",
		      rd_name, EXTRACT_RVC_LWSP_IMM (iw),
		      rd_name, EXTRACT_RVC_LWSP_IMM (iw));
	  store_rd (cpu, rd, EXTEND32 (
	    sim_core_read_unaligned_4 (cpu, cpu->pc, read_map,
				       cpu->sp.u
				       + EXTRACT_RVC_LWSP_IMM (iw))));
	  return pc;
	case MATCH_C_SWSP:
	  TRACE_INSN (cpu, "c.swsp %s, %" PRIiTW "(sp);"
			   " // *(sp + %" PRIiTW ") = %s",
		      crs2_name, EXTRACT_RVC_SWSP_IMM (iw),
		      EXTRACT_RVC_SWSP_IMM (iw), crs2_name);
	  sim_core_write_unaligned_4 (cpu, cpu->pc, write_map,
				      (cpu->sp.u + EXTRACT_RVC_SWSP_IMM (iw)),
				      cpu->regs[crs2].u);
	  return pc;
	case MATCH_C_ADD:
	  TRACE_INSN (cpu, "c.add %s, %s // %s += %s",
		      rd_name, crs2_name,
		      rd_name, crs2_name);
	  store_rd (cpu, rd, cpu->regs[rd].u + cpu->regs[crs2].u);
	  return pc;
	case MATCH_C_FLDSP:
	  /* rv32/64: c.fldsp, rv128: c.flqsp.  */
	  if (RISCV_XLEN (cpu) <= 64)
	    {
	      TRACE_INSN (cpu, "c.fldsp %s, %" PRIiTW "(sp);"
			       " // %s = *(sp + %" PRIiTW ")",
			  frd_name, EXTRACT_RVC_LDSP_IMM (iw),
			  frd_name, EXTRACT_RVC_LDSP_IMM (iw));
	      store_frd64 (cpu, rd,
		sim_core_read_unaligned_8 (cpu, cpu->pc, read_map,
					   cpu->sp.u
					   + EXTRACT_RVC_LDSP_IMM (iw)));
	      return pc;
	    }
	  else
	    {
	      TRACE_INSN (cpu, "UNHANDLED RV128 INSN: %s", op->name);
	      sim_engine_halt (sd, cpu, NULL, cpu->pc,
			       sim_signalled, SIM_SIGILL);
	    }
	case MATCH_C_FLWSP:
	  /* rv32: c.flwsp, rv64: c.ldsp.  */
	  if (RISCV_XLEN (cpu) == 32)
	    {
	      TRACE_INSN (cpu, "c.flwsp %s, %" PRIiTW "(sp);"
			       " // %s = *(sp + %" PRIiTW ")",
			  frd_name, EXTRACT_RVC_LWSP_IMM (iw),
			  frd_name, EXTRACT_RVC_LWSP_IMM (iw));
	      store_frd (cpu, rd, EXTEND32 (
		sim_core_read_unaligned_4 (cpu, cpu->pc, read_map,
					   cpu->sp.u
					   + EXTRACT_RVC_LWSP_IMM (iw))));
	    }
	  else
	    {
	      TRACE_INSN (cpu, "c.ldsp %s, %" PRIiTW "(sp);"
			       " // %s = *(sp + %" PRIiTW ")",
			  rd_name, EXTRACT_RVC_LDSP_IMM (iw),
			  rd_name, EXTRACT_RVC_LDSP_IMM (iw));
	      store_rd (cpu, rd,
		sim_core_read_unaligned_8 (cpu, cpu->pc, read_map,
					   cpu->sp.u
					   + EXTRACT_RVC_LDSP_IMM (iw)));
	    }
	  return pc;
	case MATCH_C_FSDSP:
	  /* rv32/64: c.fsdsp, rv128: c.fsqsp.  */
	  if (RISCV_XLEN (cpu) <= 64)
	    {
	      TRACE_INSN (cpu, "c.fsdsp %s, %" PRIiTW "(sp);"
			       " // *(sp + %" PRIiTW ") = %s",
			  fcrs2_name, EXTRACT_RVC_LDSP_IMM (iw),
			  EXTRACT_RVC_LDSP_IMM (iw), fcrs2_name);
	      sim_core_write_unaligned_8 (cpu, cpu->pc, write_map,
					  cpu->sp.u + EXTRACT_RVC_SDSP_IMM (iw),
					  cpu->fpregs[crs2].v[0]);
	      return pc;
	    }
	  else
	    {
	      TRACE_INSN (cpu, "UNHANDLED RV128 INSN: %s", op->name);
	      sim_engine_halt (sd, cpu, NULL, cpu->pc,
			       sim_signalled, SIM_SIGILL);
	    }
	case MATCH_C_FSWSP:
	  /* rv32: c.fswsp, rv64: c.sdsp.  */
	  if (RISCV_XLEN (cpu) == 32)
	    {
	      TRACE_INSN (cpu, "c.fswsp %s, %" PRIiTW "(sp);"
			       " // *(sp + %" PRIiTW ") = %s",
			  fcrs2_name, EXTRACT_RVC_SWSP_IMM (iw),
			  EXTRACT_RVC_SWSP_IMM (iw), fcrs2_name);
	      sim_core_write_unaligned_4 (cpu, cpu->pc, write_map,
					  cpu->sp.u
					  + EXTRACT_RVC_SWSP_IMM (iw),
					  cpu->fpregs[crs2].w[0]);
	    }
	  else
	    {
	      TRACE_INSN (cpu, "c.sdsp %s, %" PRIiTW "(sp);"
			       " // *(sp + %" PRIiTW ") = %s",
			  crs2_name, EXTRACT_RVC_SDSP_IMM (iw),
			  EXTRACT_RVC_SDSP_IMM (iw), crs2_name);
	      sim_core_write_unaligned_8 (cpu, cpu->pc, write_map,
					  cpu->sp.u + EXTRACT_RVC_SDSP_IMM (iw),
					  cpu->regs[crs2].u);
	    }
	  return pc;
	default:
	  TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	  sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
	}
    default:
      TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
      sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
    }

  return pc;
}

static sim_cia
execute_andes(SIM_CPU *cpu, unsigned_word iw, const struct riscv_opcode *op, int ex9)
{
  SIM_DESC sd = CPU_STATE (cpu);
  int rd = (iw >> OP_SH_RD) & OP_MASK_RD;
  int rs1 = (iw >> OP_SH_RS1) & OP_MASK_RS1;
  int rs2 = (iw >> OP_SH_RS2) & OP_MASK_RS2;
  const char *rd_name = riscv_gpr_names_abi[rd];
  const char *rs1_name = riscv_gpr_names_abi[rs1];
  const char *rs2_name = riscv_gpr_names_abi[rs2];

  unsigned_word cimm6 = EXTRACT_TYPE_CIMM6 (iw);
  unsigned_word simm7 = EXTRACT_STYPE_IMM7(iw);
  unsigned_word simm10 = EXTRACT_STYPE_IMM10(iw);

  unsigned_word msb = EXTRACT_ITYPE_IMM6H (iw);
  unsigned_word lsb = EXTRACT_ITYPE_IMM6L (iw);

  unsigned_word lbgp_imm = EXTRACT_GPTYPE_LB_IMM(iw);
  unsigned_word lhgp_imm = EXTRACT_GPTYPE_LH_IMM(iw);
  unsigned_word lwgp_imm = EXTRACT_GPTYPE_LW_IMM(iw);
  unsigned_word ldgp_imm = EXTRACT_GPTYPE_LD_IMM(iw);
  unsigned_word sbgp_imm = EXTRACT_GPTYPE_SB_IMM(iw);
  unsigned_word shgp_imm = EXTRACT_GPTYPE_SH_IMM(iw);
  unsigned_word swgp_imm = EXTRACT_GPTYPE_SW_IMM(iw);
  unsigned_word sdgp_imm = EXTRACT_GPTYPE_SD_IMM(iw);

  sim_cia pc = cpu->pc + 4;
  if (ex9)
    pc -= 2;

  switch (op->match)
    {
    case MATCH_ADDIGP: {
      unsigned_word gp_imm = EXTRACT_GPTYPE_LB_IMM(iw);
      TRACE_INSN (cpu, "addigp %s, gp, %#"PRIxTW";  // %s = gp + %#"PRIxTW,
		  rd_name, gp_imm, rd_name, gp_imm);
      store_rd (cpu, rd, cpu->regs[X_GP].u + gp_imm);
      break;
    }
    case MATCH_BBC:
      TRACE_INSN (cpu, "bbc %s, %d, %#"PRIxTW";  // if (!(%s & (1 << %d))) goto %#"PRIxTW,
		  rs1_name, cimm6, simm10, rs1_name, cimm6, cpu->pc + simm10);
      if (!(cpu->regs[rs1].u & (((uint64_t) 1) << cimm6)))
	{
	  pc = cpu->pc + simm10;
	  TRACE_BRANCH (cpu, "to %#"PRIxTW, pc);
	}
      break;
    case MATCH_BBS:
      TRACE_INSN (cpu, "bbs %s, %d, %#"PRIxTW";  // if (%s & (1 << %d)) goto %#"PRIxTW,
		  rs1_name, cimm6, simm10, rs1_name, cimm6, cpu->pc + simm10);
      if (cpu->regs[rs1].u & (((uint64_t) 1) << cimm6))
	{
	  pc = cpu->pc + simm10;
	  TRACE_BRANCH (cpu, "to %#"PRIxTW, pc);
	}
      break;
    case MATCH_BEQC:
      TRACE_INSN (cpu, "beqc %s, %d, %#"PRIxTW";  // if (%s == %d) goto %#"PRIxTW,
		  rs1_name, simm7, simm10, rs1_name, simm7, cpu->pc + simm10);
      if (cpu->regs[rs1].u == simm7)
	{
	  pc = cpu->pc + simm10;
	  TRACE_BRANCH (cpu, "to %#"PRIxTW, pc);
	}
      break;
    case MATCH_BNEC:
      TRACE_INSN (cpu, "bnec %s, %d, %#"PRIxTW";  // if (%s == %d) goto %#"PRIxTW,
		  rs1_name, simm7, simm10, rs1_name, simm7, cpu->pc + simm10);
      if (cpu->regs[rs1].u != simm7)
	{
	  pc = cpu->pc + simm10;
	  TRACE_BRANCH (cpu, "to %#"PRIxTW, pc);
	}
      break;
    case MATCH_BFOS:
      TRACE_INSN (cpu, "bfos %s, %s, %#"PRIxTW", %#"PRIxTW"; // ", rd_name, rs1_name,
		  msb, lsb);
      if (msb == 0)
	store_rd (cpu, rd, SIM_RV_SEXT ((cpu->regs[rs1].u & 0x1) << lsb, lsb + 1));
      else if (msb < lsb)
	store_rd (cpu, rd,
	          SIM_RV_SEXT (SIM_RV_X (cpu->regs[rs1].u, 0, lsb - msb + 1) << msb,
		               lsb + 1));
      else
	store_rd (cpu, rd,
	          SIM_RV_SEXT (SIM_RV_X (cpu->regs[rs1].u, lsb, msb - lsb + 1),
		               msb - lsb + 1));
      break;
    case MATCH_BFOZ:
      TRACE_INSN (cpu, "bfoz %s, %s, %#"PRIxTW", %#"PRIxTW"; // ", rd_name, rs1_name,
		  msb, lsb);
      if (msb == 0)
	store_rd (cpu, rd, (cpu->regs[rs1].u & 0x1) << lsb);
      else if (msb < lsb)
	store_rd (cpu, rd, SIM_RV_X (cpu->regs[rs1].u, 0, lsb - msb + 1) << msb);
      else
	store_rd (cpu, rd, SIM_RV_X (cpu->regs[rs1].u, lsb, msb - lsb + 1));
      break;
    case MATCH_LEA_H:
      TRACE_INSN (cpu, "lea.h %s, %s, %s;  // %s = %s + (%s << 1)",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      store_rd (cpu, rd, cpu->regs[rs1].u + (cpu->regs[rs2].u << 1));
      break;
    case MATCH_LEA_W:
      TRACE_INSN (cpu, "lea.h %s, %s, %s;  // %s = %s + (%s << 2)",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      store_rd (cpu, rd, cpu->regs[rs1].u + (cpu->regs[rs2].u << 2));
      break;
    case MATCH_LEA_D:
      TRACE_INSN (cpu, "lea.d %s, %s, %s;  // %s = %s + (%s << 3)",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      store_rd (cpu, rd, cpu->regs[rs1].u + (cpu->regs[rs2].u << 3));
      break;
#if (WITH_TARGET_WORD_BITSIZE == 64)
    case MATCH_LEA_B_ZE:
      TRACE_INSN (cpu, "lea.b.ze %s, %s, %s;  // %s = %s + (ZE(%s) << 0)",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      store_rd (cpu, rd, cpu->regs[rs1].u + (SIM_RV_X (cpu->regs[rs2].u, 0, 32) << 0));
      break;
    case MATCH_LEA_H_ZE:
      TRACE_INSN (cpu, "lea.h.ze %s, %s, %s;  // %s = %s + (ZE(%s) << 1)",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      store_rd (cpu, rd, cpu->regs[rs1].u + (SIM_RV_X (cpu->regs[rs2].u, 0, 32) << 1));
      break;
    case MATCH_LEA_W_ZE:
      TRACE_INSN (cpu, "lea.h.ze %s, %s, %s;  // %s = %s + (ZE(%s) << 2)",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      store_rd (cpu, rd, cpu->regs[rs1].u + (SIM_RV_X (cpu->regs[rs2].u, 0, 32) << 2));
      break;
    case MATCH_LEA_D_ZE:
      TRACE_INSN (cpu, "lea.d.ze %s, %s, %s;  // %s = %s + (ZE(%s) << 3)",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      store_rd (cpu, rd, cpu->regs[rs1].u + (SIM_RV_X (cpu->regs[rs2].u, 0, 32) << 3));
      break;
#endif
    case MATCH_LBGP:
      TRACE_INSN (cpu, "lbgp %s, %"PRIiTW"; // %s = *(gp + %"PRIiTW")",
		  rd_name, lbgp_imm, rd_name, lbgp_imm);
      store_rd (cpu, rd, EXTEND8 (
	sim_core_read_unaligned_1 (cpu, cpu->pc, read_map,
				   cpu->regs[X_GP].u + lbgp_imm)));
      break;
    case MATCH_LBUGP:
      TRACE_INSN (cpu, "lbugp %s, %"PRIiTW"; // %s = *(gp + %"PRIiTW")",
		  rd_name, lbgp_imm, rd_name, lbgp_imm);
      store_rd (cpu, rd,
	sim_core_read_unaligned_1 (cpu, cpu->pc, read_map,
				   cpu->regs[X_GP].u + lbgp_imm));
      break;
    case MATCH_LHGP:
      TRACE_INSN (cpu, "lhgp %s, %"PRIiTW"; // %s = *(gp + %"PRIiTW")",
		  rd_name, lbgp_imm, rd_name, lbgp_imm);
      store_rd (cpu, rd, EXTEND16 (
	sim_core_read_unaligned_2 (cpu, cpu->pc, read_map,
				   cpu->regs[X_GP].u + lhgp_imm)));
      break;
    case MATCH_LHUGP:
      TRACE_INSN (cpu, "lhugp %s, %"PRIiTW"; // %s = *(gp + %"PRIiTW")",
		  rd_name, lbgp_imm, rd_name, lbgp_imm);
      store_rd (cpu, rd,
	sim_core_read_unaligned_2 (cpu, cpu->pc, read_map,
				   cpu->regs[X_GP].u + lhgp_imm));
      break;
    case MATCH_LWGP:
      TRACE_INSN (cpu, "lwgp %s, %"PRIiTW"; // %s = *(gp + %"PRIiTW")",
		  rd_name, lbgp_imm, rd_name, lbgp_imm);
      store_rd (cpu, rd, EXTEND32 (
	sim_core_read_unaligned_4 (cpu, cpu->pc, read_map,
				   cpu->regs[X_GP].u + lwgp_imm)));
      break;
    case MATCH_LWUGP:
      TRACE_INSN (cpu, "lwugp %s, %"PRIiTW"; // %s = *(gp + %"PRIiTW")",
		  rd_name, lbgp_imm, rd_name, lbgp_imm);
      store_rd (cpu, rd,
	sim_core_read_unaligned_4 (cpu, cpu->pc, read_map,
				   cpu->regs[X_GP].u + lwgp_imm));
      break;
    case MATCH_LDGP:
      TRACE_INSN (cpu, "ldgp %s, %"PRIiTW"; // %s = *(gp + %"PRIiTW")",
		  rd_name, lbgp_imm, rd_name, lbgp_imm);
      RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
      store_rd (cpu, rd,
	sim_core_read_unaligned_8 (cpu, cpu->pc, read_map,
				   cpu->regs[X_GP].u + ldgp_imm));
      break;
    case MATCH_SBGP:
      TRACE_INSN (cpu, "sbgp %s, %"PRIiTW"; // *(gp + %"PRIiTW") = %s",
		  rs2_name, sbgp_imm, sbgp_imm, rs2_name);
      sim_core_write_unaligned_1 (cpu, cpu->pc, write_map,
				  cpu->regs[X_GP].u + sbgp_imm, cpu->regs[rs2].u);
      break;
    case MATCH_SHGP:
      TRACE_INSN (cpu, "shgp %s, %"PRIiTW"; // *(gp + %"PRIiTW") = %s",
		  rs2_name, sbgp_imm, sbgp_imm, rs2_name);
      sim_core_write_unaligned_2 (cpu, cpu->pc, write_map,
				  cpu->regs[X_GP].u + shgp_imm, cpu->regs[rs2].u);
      break;
    case MATCH_SWGP:
      TRACE_INSN (cpu, "swgp %s, %"PRIiTW"; // *(gp + %"PRIiTW") = %s",
		  rs2_name, sbgp_imm, sbgp_imm, rs2_name);
      sim_core_write_unaligned_4 (cpu, cpu->pc, write_map,
				  cpu->regs[X_GP].u + swgp_imm, cpu->regs[rs2].u);
      break;
    case MATCH_SDGP:
      TRACE_INSN (cpu, "sdgp %s, %"PRIiTW"; // *(gp + %"PRIiTW") = %s",
		  rs2_name, sbgp_imm, sbgp_imm, rs2_name);
      RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
      sim_core_write_unaligned_8 (cpu, cpu->pc, write_map,
				  cpu->regs[X_GP].u + sdgp_imm, cpu->regs[rs2].u);
      break;
    case MATCH_FFB:
    /* FIXME: Implement for little-endian only.  */
      {
	int j;
	TRACE_INSN (cpu, "ffb %s, %s, %s;", rd_name, rs1_name, rs2_name);
	for (j = 0; j < RISCV_XLEN (cpu); j+=8)
	  {
	    if (((cpu->regs[rs1].u >> j) & 0xff)
		== (cpu->regs[rs2].u & 0xff))
	      break;
	  }
	store_rd (cpu, rd, (int64_t) (j - RISCV_XLEN (cpu)) / 8);
      }
      break;
    case MATCH_FFZMISM:
    /* FIXME: Implement for little-endian only.  */
      {
	int j;
	TRACE_INSN (cpu, "ffzmism %s, %s, %s;", rd_name, rs1_name, rs2_name);
	for (j = 0; j < RISCV_XLEN (cpu); j+=8)
	  {
	    if ((cpu->regs[rs1].u & ((uint64_t) 0xff << j)) == 0
		|| ((cpu->regs[rs1].u & ((uint64_t) 0xff << j))
		    != (cpu->regs[rs2].u & ((uint64_t) 0xff << j))))
	      break;
	  }
	store_rd (cpu, rd, (int64_t) (j - RISCV_XLEN (cpu)) / 8);
      }
      break;
    case MATCH_FFMISM:
    /* FIXME: Implement for little-endian only.  */
      {
	int j;
	TRACE_INSN (cpu, "ffmism %s, %s, %s;", rd_name, rs1_name, rs2_name);
	for (j = 0; j < RISCV_XLEN (cpu); j+=8)
	  {
	    if ((cpu->regs[rs1].u & ((uint64_t) 0xff << j))
		!= (cpu->regs[rs2].u & ((uint64_t) 0xff << j)))
	      break;
	  }
	store_rd (cpu, rd, (int64_t) (j - RISCV_XLEN (cpu)) / 8);
      }
      break;
    case MATCH_FLMISM:
    /* FIXME: Implement for little-endian only.  */
      {
	int j;
	TRACE_INSN (cpu, "flmism %s, %s, %s;", rd_name, rs1_name, rs2_name);
	for (j = RISCV_XLEN (cpu) - 8; j >= 0; j-=8)
	  {
	    if ((cpu->regs[rs1].u & ((uint64_t) 0xff << j))
		!= (cpu->regs[rs2].u & ((uint64_t) 0xff << j)))
	      break;
	  }
	store_rd (cpu, rd, (int64_t) (j - RISCV_XLEN (cpu)) / 8);
      }
      break;
    default:
      TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
      sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
    }

  return pc;
}

static sim_cia
execute_p (SIM_CPU *cpu, unsigned_word iw, const struct riscv_opcode *op, int ex9)
{
  SIM_DESC sd = CPU_STATE (cpu);
  int rd = (iw >> OP_SH_RD) & OP_MASK_RD;
  int ra = (iw >> OP_SH_RS1) & OP_MASK_RS1;
  int rb = (iw >> OP_SH_RS2) & OP_MASK_RS2;
  int rc = (iw >> 25) & 0x1f;
  const char *rd_name = riscv_gpr_names_abi[rd];
  const char *ra_name = riscv_gpr_names_abi[ra];
  const char *rb_name = riscv_gpr_names_abi[rb];
  const int imm6u = (iw >> 20) & 0x3f;
  const int imm5u = rb;
  const int imm4u = rb & 0xf;
  const int imm3u = rb & 0x7;
  const int imm2u = (iw >> 20) & 0x3;

  reg_t result;
  int32_t res, i;
  int32_t vec32_num = RISCV_XLEN (cpu) == 64 ? 2 : 1;
  int32_t vec16_num = RISCV_XLEN (cpu) == 64 ? 4 : 2;
  int32_t vec8_num = RISCV_XLEN (cpu) == 64 ? 8 : 4;
  int32_t *ptr32 = (int32_t *) & result.b32;
  int32_t *ptr_a32 = (int32_t *) & cpu->regs[ra].b32;
  int32_t *ptr_b32 = (int32_t *) & cpu->regs[rb].b32;
  int32_t *ptr_d32 = (int32_t *) & cpu->regs[rd].b32;
  uint32_t *uptr32 = (uint32_t *) & result.b32;
  uint32_t *uptr_a32 = (uint32_t *) & cpu->regs[ra].b32;
  uint32_t *uptr_b32 = (uint32_t *) & cpu->regs[rb].b32;
  uint32_t *uptr_d32 = (uint32_t *) & cpu->regs[rd].b32;
  int16_t *ptr16 = (int16_t *) & result.b16;
  int16_t *ptr_a16 = (int16_t *) & cpu->regs[ra].b16;
  int16_t *ptr_b16 = (int16_t *) & cpu->regs[rb].b16;
  int16_t *ptr_d16 = (int16_t *) & cpu->regs[rd].b16;
  uint16_t *uptr16 = (uint16_t *) & result.ub16;
  uint16_t *uptr_a16 = (uint16_t *) & cpu->regs[ra].ub16;
  uint16_t *uptr_b16 = (uint16_t *) & cpu->regs[rb].ub16;
  int8_t *ptr8 = (int8_t *) & result.b8;
  int8_t *ptr_a8 = (int8_t *) & cpu->regs[ra].b8;
  int8_t *ptr_b8 = (int8_t *) & cpu->regs[rb].b8;
  uint8_t *uptr8 = (uint8_t *) & result.ub8;
  uint8_t *uptr_a8 = (uint8_t *) & cpu->regs[ra].ub8;
  uint8_t *uptr_b8 = (uint8_t *) & cpu->regs[rb].ub8;

  sim_cia pc = cpu->pc + 4;
  if (ex9)
    pc -= 2;

  switch (op->match)
    {
    case MATCH_ADD16:
      {
	for (i = 0; i < vec16_num; i++)
	  {
	    res = *(ptr_a16 + i) + *(ptr_b16 + i);
	    *(ptr16 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_RADD16:
      {
	for (i = 0; i < vec16_num; i++)
	  {
	    res = (int16_t) (((int32_t) *(ptr_a16 + i) + *(ptr_b16 + i)) >> 1);
	    *(ptr16 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_URADD16:
      {
	for (i = 0; i < vec16_num; i++)
	  {
	    res =
	      (uint16_t) (((uint32_t) *(uptr_a16 + i) + *(uptr_b16 + i)) >> 1);
	    *(uptr16 + i) = res;
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KADD16:
      {
	/* Rt[31:16] = Ra[31:16] + Rb[31:16]
	   Rt[15:0] = Ra[15:0] + Rb[15:0] */
	for (i = 0; i < vec16_num; i++)
	  {
	    res = *(ptr_a16 + i) + *(ptr_b16 + i);
	    res = insn_sat_helper (cpu, res, 15);
	    *(ptr16 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UKADD16:
      {
	/* Rt[31:16] = Ra[31:16] + Rb[31:16]
	   Rt[15:0] = Ra[15:0] + Rb[15:0] */
	for (i = 0; i < vec16_num; i++)
	  {
	    res = *(uptr_a16 + i) + *(uptr_b16 + i);
	    *(uptr16 + i) = insn_usat_helper (cpu, res, 16);
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_ADD32:
      {
	for (i = 0; i < vec32_num; i++)
	  {
	    res = *(ptr_a32 + i) + *(ptr_b32 + i);
	    *(ptr32 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_RADD32:
      {
	for (i = 0; i < vec32_num; i++)
	  {
	    res = (int32_t) (((int64_t) *(ptr_a32 + i) + *(ptr_b32 + i)) >> 1);
	    *(ptr32 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_URADD32:
      {
	for (i = 0; i < vec32_num; i++)
	  {
	    res =
	      (uint32_t) (((uint64_t) *(uptr_a32 + i) + *(uptr_b32 + i)) >> 1);
	    *(uptr32 + i) = res;
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KADD32:
      {
	for (i = 0; i < vec32_num; i++)
	  {
	    int64_t res = (int64_t) *(ptr_a32 + i) + *(ptr_b32 + i);
	    res = insn_sat_helper (cpu, res, 31);
	    *(ptr32 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UKADD32:
      {
	int64_t res1;
	for (i = 0; i < vec32_num; i++)
	  {
	    res1 = (int64_t) *(uptr_a32 + i) + *(uptr_b32 + i);
	    *(uptr32 + i) = insn_usat_helper (cpu, res1, 32);
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SUB16:
      {
	/* Rt[31:16] = Ra[31:16] - Rb[31:16]
	   Rt[15:0] = Ra[15:0] - Rb[15:0] */
	for (i = 0; i < vec16_num; i++)
	  {
	    res = *(ptr_a16 + i) - *(ptr_b16 + i);
	    *(ptr16 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_RSUB16:
      {
	/* Rt[31:16] = (Ra[31:16] - Rb[31:16]) >> 1
	   Rt[15:0] = (Ra[15:0] - Rb[15:0]) >> 1 */
	for (i = 0; i < vec16_num; i++)
	  {
	    res = (int16_t) (((int32_t) *(ptr_a16 + i) - *(ptr_b16 + i)) >> 1);
	    *(ptr16 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_URSUB16:
      {
	/* Rt[31:16] = Ra[31:16] - Rb[31:16]
	   Rt[15:0] = Ra[15:0] - Rb[15:0] */
	for (i = 0; i < vec16_num; i++)
	  {
	    res = (uint16_t) (((uint32_t) *(uptr_a16 + i)
			       - *(uptr_b16 + i)) >> 1);
	    *(uptr16 + i) = res;
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KSUB16:
      {
	/* Rt[31:16] = Ra[31:16] - Rb[31:16]
	   Rt[15:0] = Ra[15:0] - Rb[15:0] */
	for (i = 0; i < vec16_num; i++)
	  {
	    res = *(ptr_a16 + i) - *(ptr_b16 + i);
	    res = insn_sat_helper (cpu, res, 15);
	    *(ptr16 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UKSUB16:
      {
	/* Rt[31:16] = Ra[31:16] - Rb[31:16]
	   Rt[15:0] = Ra[15:0] - Rb[15:0] */
	for (i = 0; i < vec16_num; i++)
	  {
	    res = *(uptr_a16 + i) - *(uptr_b16 + i);
	    *(uptr16 + i) = insn_usat_helper (cpu, res, 16);
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SUB32:
      {
	for (i = 0; i < vec32_num; i++)
	  {
	    res = *(ptr_a32 + i) - *(ptr_b32 + i);
	    *(ptr32 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_RSUB32:
      {
	for (i = 0; i < vec32_num; i++)
	  {
	    res = (int32_t) (((int64_t) *(ptr_a32 + i) - *(ptr_b32 + i)) >> 1);
	    *(ptr32 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_URSUB32:
      {
	for (i = 0; i < vec32_num; i++)
	  {
	    res =
	      (uint32_t) (((uint64_t) *(uptr_a32 + i) - *(uptr_b32 + i)) >> 1);
	    *(uptr32 + i) = res;
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KSUB32:
      {
	for (i = 0; i < vec32_num; i++)
	  {
	    int64_t res = (int64_t) *(ptr_a32 + i) - *(ptr_b32 + i);
	    res = insn_sat_helper (cpu, res, 31);
	    *(ptr32 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UKSUB32:
      {
	int64_t res1;
	for (i = 0; i < vec32_num; i++)
	  {
	    res1 = (int64_t) *(uptr_a32 + i) - *(uptr_b32 + i);
	    *(uptr32 + i) = insn_usat_helper (cpu, res1, 32);
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_CRAS16:
      {
	/* Rt[31:16] = Ra[31:16] + Rb[15:0]
	   Rt[15:0] = Ra[15:0] - Rb[31:16] */
	for (i = 0; i < vec16_num; i+=2)
	  {
	    res = *(ptr_a16 + (i + 1)) + *(ptr_b16 + i);
	    *(ptr16 + (i + 1)) = res;
	    res = *(ptr_a16 + i) - *(ptr_b16 + (i + 1));
	    *(ptr16 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_RCRAS16:
      {
	/* Rt[31:16] = (Ra[31:16] + Rb[15:0]) >>1
	   Rt[15:0] = (Ra[15:0] - Rb[31:16]) >> 1 */
	for (i = 0; i < vec16_num; i+=2)
	  {
	    res = (int16_t) (((int32_t) *(ptr_a16 + (i + 1))
			      + *(ptr_b16 + i)) >> 1);
	    *(ptr16 + (i + 1)) = res;
	    res = (int16_t) (((int32_t) *(ptr_a16 + i)
			      - *(ptr_b16 + (i + 1))) >> 1);
	    *(ptr16 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_URCRAS16:
      {
	/* Rt[31:16] = (Ra[31:16] + Rb[15:0]) >>1
	   Rt[15:0] = (Ra[15:0] - Rb[31:16]) >> 1 */
	for (i = 0; i < vec16_num; i+=2)
	  {
	    res = (uint16_t) (((uint32_t) *(uptr_a16 + (i + 1))
			       + *(uptr_b16 + i)) >> 1);
	    *(uptr16 + (i + 1)) = res;

	    res = (uint16_t) (((uint32_t) *(uptr_a16 + i)
			       - *(uptr_b16 + (i + 1))) >> 1);
	    *(uptr16 + i) = res;
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KCRAS16:
      {
	/* Rt[31:16] = Ra[31:16] + Rb[15:0]
	   Rt[15:0] = Ra[15:0] - Rb[31:16] */
	for (i = 0; i < vec16_num; i+=2)
	  {
	    res = *(ptr_a16 + (i + 1)) + *(ptr_b16 + i);
	    *(ptr16 + (i + 1)) = insn_sat_helper (cpu, res, 15);
	    res = *(ptr_a16 + i) - *(ptr_b16 + (i + 1));
	    *(ptr16 + i) = insn_sat_helper (cpu, res, 15);
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UKCRAS16:
      {
	/* Rt[31:16] = Ra[31:16] + Rb[15:0]
	   Rt[15:0] = Ra[15:0] - Rb[31:16] */
	for (i = 0; i < vec16_num; i+=2)
	  {
	    res = *(uptr_a16 + (i + 1)) + *(uptr_b16 + i);
	    *(uptr16 + (i + 1)) = insn_usat_helper (cpu, res, 16);

	    res = *(uptr_a16 + i) - *(uptr_b16 + (i + 1));
	    *(uptr16 + i) = insn_usat_helper (cpu, res, 16);
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_CRAS32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	result.b32.i1 = cpu->regs[ra].b32.i1 + cpu->regs[rb].b32.i0;
	result.b32.i0 = cpu->regs[ra].b32.i0 - cpu->regs[rb].b32.i1;
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_RCRAS32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	result.b32.i1 = (int32_t) (((int64_t) cpu->regs[ra].b32.i1
				   + cpu->regs[rb].b32.i0) >> 1);
	result.b32.i0 = (int32_t) (((int64_t) cpu->regs[ra].b32.i0
				   - cpu->regs[rb].b32.i1) >> 1);
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_URCRAS32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	result.ub32.i1 = (uint32_t) (((uint64_t) cpu->regs[ra].ub32.i1
				     + cpu->regs[rb].ub32.i0) >> 1);
	result.ub32.i0 = (uint32_t) (((uint64_t) cpu->regs[ra].ub32.i0
				     - cpu->regs[rb].ub32.i1) >> 1);
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_KCRAS32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	int64_t res1 = (int64_t) cpu->regs[ra].b32.i1 + cpu->regs[rb].b32.i0;
	int64_t res2 = (int64_t) cpu->regs[ra].b32.i0 - cpu->regs[rb].b32.i1;
	cpu->regs[rd].b32.i0 = insn_sat_helper (cpu, res2, 31);
	cpu->regs[rd].b32.i1 = insn_sat_helper (cpu, res1, 31);
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_UKCRAS32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	int64_t res1 = (int64_t) cpu->regs[ra].ub32.i1 + cpu->regs[rb].ub32.i0;
	int64_t res2 = (int64_t) cpu->regs[ra].ub32.i0 - cpu->regs[rb].ub32.i1;
	cpu->regs[rd].ub32.i0 = insn_sat_helper (cpu, res2, 32);
	cpu->regs[rd].ub32.i1 = insn_sat_helper (cpu, res1, 32);
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_CRSA16:
      {
	/* Rt[31:16] = Ra[31:16] - Rb[15:0]
	   Rt[15:0] = Ra[15:0] + Rb[31:16] */
	for (i = 0; i < vec16_num; i+=2)
	  {
	    res = *(ptr_a16 + (i + 1)) - *(ptr_b16 + i);
	    *(ptr16 + (i + 1)) = res;
	    res = *(ptr_a16 + i) + *(ptr_b16 + (i + 1));
	    *(ptr16 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_RCRSA16:
      {
	/* Rt[31:16] = (Ra[31:16] - Rb[15:0]) >> 1
	   Rt[15:0] = (Ra[15:0] + Rb[31:16]) >> 1 */
	for (i = 0; i < vec16_num; i+=2)
	  {
	    res = (int16_t) (((int32_t) *(ptr_a16 + (i + 1))
			      - *(ptr_b16 + i)) >> 1);
	    *(ptr16 + (i + 1)) = res;
	    res = (int16_t) (((int32_t) *(ptr_a16 + i)
			      + *(ptr_b16 + (i + 1))) >> 1);
	    *(ptr16 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_URCRSA16:
      {
	/* Rt[31:16] = (Ra[31:16] - Rb[15:0]) >> 1
	   Rt[15:0] = (Ra[15:0] + Rb[31:16]) >> 1 */
	for (i = 0; i < vec16_num; i+=2)
	  {
	    res = (uint16_t) (((uint32_t) *(uptr_a16 + (i + 1))
			       - *(uptr_b16 + i)) >> 1);
	    *(uptr16 + (i + 1)) = res;

	    res = (uint16_t) (((uint32_t) *(uptr_a16 + i)
			       + *(uptr_b16 + (i + 1))) >> 1);
	    *(uptr16 + i) = res;
	}
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KCRSA16:
      {
	/* Rt[31:16] = Ra[31:16] - Rb[15:0]
	   Rt[15:0] = Ra[15:0] + Rb[31:16] */
	for (i = 0; i < vec16_num; i+=2)
	  {
	    res = *(ptr_a16 + (i + 1)) - *(ptr_b16 + i);
	    *(ptr16 + (i + 1)) = insn_sat_helper (cpu, res, 15);
	    res = *(ptr_a16 + i) + *(ptr_b16 + (i + 1));
	    *(ptr16 + i) = insn_sat_helper (cpu, res, 15);
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UKCRSA16:
      {
	/* Rt[31:16] = Ra[31:16] - Rb[15:0]
	   Rt[15:0] = Ra[15:0] + Rb[31:16] */
	for (i = 0; i < vec16_num; i+=2)
	  {
	    res = *(uptr_a16 + (i + 1)) - *(uptr_b16 + i);
	    *(uptr16 + (i + 1)) = insn_usat_helper (cpu, res, 16);

	    res = *(uptr_a16 + i) + *(uptr_b16 + (i + 1));
	    *(uptr16 + i) = insn_usat_helper (cpu, res, 16);
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_CRSA32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	result.b32.i1 = cpu->regs[ra].b32.i1 - cpu->regs[rb].b32.i0;
	result.b32.i0 = cpu->regs[ra].b32.i0 + cpu->regs[rb].b32.i1;
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_RCRSA32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	result.b32.i1 = (int32_t) (((int64_t) cpu->regs[ra].b32.i1
				   - cpu->regs[rb].b32.i0) >> 1);
	result.b32.i0 = (int32_t) (((int64_t) cpu->regs[ra].b32.i0
				   + cpu->regs[rb].b32.i1) >> 1);
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_URCRSA32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	result.ub32.i1 = (uint32_t) (((uint64_t) cpu->regs[ra].ub32.i1
				     - cpu->regs[rb].ub32.i0) >> 1);
	result.ub32.i0 = (uint32_t) (((uint64_t) cpu->regs[ra].ub32.i0
				     + cpu->regs[rb].ub32.i1) >> 1);
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_KCRSA32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	int64_t res1 = (int64_t) cpu->regs[ra].b32.i1 - cpu->regs[rb].b32.i0;
	int64_t res2 = (int64_t) cpu->regs[ra].b32.i0 + cpu->regs[rb].b32.i1;
	cpu->regs[rd].b32.i0 = insn_sat_helper (cpu, res2, 31);
	cpu->regs[rd].b32.i1 = insn_sat_helper (cpu, res1, 31);
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_UKCRSA32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	int64_t res1 = (int64_t) cpu->regs[ra].ub32.i1 - cpu->regs[rb].ub32.i0;
	int64_t res2 = (int64_t) cpu->regs[ra].ub32.i0 + cpu->regs[rb].ub32.i1;
	cpu->regs[rd].ub32.i0 = insn_sat_helper (cpu, res2, 32);
	cpu->regs[rd].ub32.i1 = insn_sat_helper (cpu, res1, 32);
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_STAS16:
      {
	/* Rt[31:16] = Ra[31:16] + Rb[31:16]
	   Rt[15:0] = Ra[15:0] - Rb[15:0] */
	for (i = 0; i < vec16_num; i+=2)
	  {
	    res = *(ptr_a16 + (i + 1)) + *(ptr_b16 + (i + 1));
	    *(ptr16 + (i + 1)) = res;
	    res = *(ptr_a16 + i) - *(ptr_b16 + i);
	    *(ptr16 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_RSTAS16:
      {
	/* Rt[31:16] = (Ra[31:16] + Rb[31:16]) >>1
	   Rt[15:0] = (Ra[15:0] - Rb[15:0]) >> 1 */
	for (i = 0; i < vec16_num; i+=2)
	  {
	    res = (int16_t) (((int32_t) *(ptr_a16 + (i + 1))
			      + *(ptr_b16 + (i + 1))) >> 1);
	    *(ptr16 + (i + 1)) = res;
	    res = (int16_t) (((int32_t) *(ptr_a16 + i)
			      - *(ptr_b16 + i)) >> 1);
	    *(ptr16 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_URSTAS16:
      {
	/* Rt[31:16] = (Ra[31:16] + Rb[31:16]) >>1
	   Rt[15:0] = (Ra[15:0] - Rb[15:0]) >> 1 */
	for (i = 0; i < vec16_num; i+=2)
	  {
	    res = (uint16_t) (((uint32_t) *(uptr_a16 + (i + 1))
			      + *(uptr_b16 + (i + 1))) >> 1);
	    *(uptr16 + (i + 1)) = res;
	    res = (uint16_t) (((uint32_t) *(uptr_a16 + i)
			      - *(uptr_b16 + i)) >> 1);
	    *(uptr16 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KSTAS16:
      {
	/* Rt[31:16] = Ra[31:16] + Rb[31:16]
	   Rt[15:0] = Ra[15:0] - Rb[15:0] */
	for (i = 0; i < vec16_num; i+=2)
	  {
	    res = *(ptr_a16 + (i + 1)) + *(ptr_b16 + (i + 1));
	    *(ptr16 + (i + 1)) = insn_sat_helper (cpu, res, 15);
	    res = *(ptr_a16 + i) - *(ptr_b16 + i);
	    *(ptr16 + i) = insn_sat_helper (cpu, res, 15);
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UKSTAS16:
      {
	/* Rt[31:16] = Ra[31:16] + Rb[31:16]
	   Rt[15:0] = Ra[15:0] - Rb[15:0] */
	for (i = 0; i < vec16_num; i+=2)
	  {
	    res = *(uptr_a16 + (i + 1)) + *(uptr_b16 + (i + 1));
	    *(uptr16 + (i + 1)) = insn_usat_helper (cpu, res, 16);
	    res = *(uptr_a16 + i) - *(uptr_b16 + i);
	    *(uptr16 + i) = insn_usat_helper (cpu, res, 16);
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_STAS32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	result.b32.i1 = cpu->regs[ra].b32.i1 + cpu->regs[rb].b32.i1;
	result.b32.i0 = cpu->regs[ra].b32.i0 - cpu->regs[rb].b32.i0;
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_RSTAS32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	result.b32.i1 = (int32_t) (((int64_t) cpu->regs[ra].b32.i1
				   + cpu->regs[rb].b32.i1) >> 1);
	result.b32.i0 = (int32_t) (((int64_t) cpu->regs[ra].b32.i0
				   - cpu->regs[rb].b32.i0) >> 1);
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_URSTAS32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	result.ub32.i1 = (uint32_t) (((uint64_t) cpu->regs[ra].ub32.i1
				     + cpu->regs[rb].ub32.i1) >> 1);
	result.ub32.i0 = (uint32_t) (((uint64_t) cpu->regs[ra].ub32.i0
				     - cpu->regs[rb].ub32.i0) >> 1);
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_KSTAS32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	int64_t res1 = (int64_t) cpu->regs[ra].b32.i1 + cpu->regs[rb].b32.i1;
	int64_t res2 = (int64_t) cpu->regs[ra].b32.i0 - cpu->regs[rb].b32.i0;
	cpu->regs[rd].b32.i0 = insn_sat_helper (cpu, res2, 31);
	cpu->regs[rd].b32.i1 = insn_sat_helper (cpu, res1, 31);
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_UKSTAS32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	int64_t res1 = (int64_t) cpu->regs[ra].ub32.i1 + cpu->regs[rb].ub32.i1;
	int64_t res2 = (int64_t) cpu->regs[ra].ub32.i0 - cpu->regs[rb].ub32.i0;
	cpu->regs[rd].ub32.i0 = insn_sat_helper (cpu, res2, 32);
	cpu->regs[rd].ub32.i1 = insn_sat_helper (cpu, res1, 32);
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_STSA16:
      {
	/* Rt[31:16] = Ra[31:16] - Rb[31:16]
	   Rt[15:0] = Ra[15:0] + Rb[15:0] */
	for (i = 0; i < vec16_num; i+=2)
	  {
	    res = *(ptr_a16 + (i + 1)) - *(ptr_b16 + (i + 1));
	    *(ptr16 + (i + 1)) = res;
	    res = *(ptr_a16 + i) + *(ptr_b16 + i);
	    *(ptr16 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_RSTSA16:
      {
	/* Rt[31:16] = (Ra[31:16] - Rb[31:16]) >>1
	   Rt[15:0] = (Ra[15:0] + Rb[15:0]) >> 1 */
	for (i = 0; i < vec16_num; i+=2)
	  {
	    res = (int16_t) (((int32_t) *(ptr_a16 + (i + 1))
			      - *(ptr_b16 + (i + 1))) >> 1);
	    *(ptr16 + (i + 1)) = res;
	    res = (int16_t) (((int32_t) *(ptr_a16 + i)
			      + *(ptr_b16 + i)) >> 1);
	    *(ptr16 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_URSTSA16:
      {
	/* Rt[31:16] = (Ra[31:16] - Rb[31:16]) >>1
	   Rt[15:0] = (Ra[15:0] + Rb[15:0]) >> 1 */
	for (i = 0; i < vec16_num; i+=2)
	  {
	    res = (uint16_t) (((uint32_t) *(uptr_a16 + (i + 1))
			      - *(uptr_b16 + (i + 1))) >> 1);
	    *(uptr16 + (i + 1)) = res;
	    res = (uint16_t) (((uint32_t) *(uptr_a16 + i)
			      + *(uptr_b16 + i)) >> 1);
	    *(uptr16 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KSTSA16:
      {
	/* Rt[31:16] = Ra[31:16] - Rb[31:16]
	   Rt[15:0] = Ra[15:0] + Rb[15:0] */
	for (i = 0; i < vec16_num; i+=2)
	  {
	    res = *(ptr_a16 + (i + 1)) - *(ptr_b16 + (i + 1));
	    *(ptr16 + (i + 1)) = insn_sat_helper (cpu, res, 15);
	    res = *(ptr_a16 + i) + *(ptr_b16 + i);
	    *(ptr16 + i) = insn_sat_helper (cpu, res, 15);
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UKSTSA16:
      {
	/* Rt[31:16] = Ra[31:16] - Rb[31:16]
	   Rt[15:0] = Ra[15:0] + Rb[15:0] */
	for (i = 0; i < vec16_num; i+=2)
	  {
	    res = *(uptr_a16 + (i + 1)) - *(uptr_b16 + (i + 1));
	    *(uptr16 + (i + 1)) = insn_usat_helper (cpu, res, 16);
	    res = *(uptr_a16 + i) + *(uptr_b16 + i);
	    *(uptr16 + i) = insn_usat_helper (cpu, res, 16);
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_STSA32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	result.b32.i1 = cpu->regs[ra].b32.i1 - cpu->regs[rb].b32.i1;
	result.b32.i0 = cpu->regs[ra].b32.i0 + cpu->regs[rb].b32.i0;
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_RSTSA32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	result.b32.i1 = (int32_t) (((int64_t) cpu->regs[ra].b32.i1
				   - cpu->regs[rb].b32.i1) >> 1);
	result.b32.i0 = (int32_t) (((int64_t) cpu->regs[ra].b32.i0
				   + cpu->regs[rb].b32.i0) >> 1);
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_URSTSA32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	result.ub32.i1 = (uint32_t) (((uint64_t) cpu->regs[ra].ub32.i1
				     - cpu->regs[rb].ub32.i1) >> 1);
	result.ub32.i0 = (uint32_t) (((uint64_t) cpu->regs[ra].ub32.i0
				     + cpu->regs[rb].ub32.i0) >> 1);
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_KSTSA32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	int64_t res1 = (int64_t) cpu->regs[ra].b32.i1 - cpu->regs[rb].b32.i1;
	int64_t res2 = (int64_t) cpu->regs[ra].b32.i0 + cpu->regs[rb].b32.i0;
	cpu->regs[rd].b32.i0 = insn_sat_helper (cpu, res2, 31);
	cpu->regs[rd].b32.i1 = insn_sat_helper (cpu, res1, 31);
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_UKSTSA32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	int64_t res1 = (int64_t) cpu->regs[ra].ub32.i1 - cpu->regs[rb].ub32.i1;
	int64_t res2 = (int64_t) cpu->regs[ra].ub32.i0 + cpu->regs[rb].ub32.i0;
	cpu->regs[rd].ub32.i0 = insn_sat_helper (cpu, res2, 32);
	cpu->regs[rd].ub32.i1 = insn_sat_helper (cpu, res1, 32);
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_ADD8:
      {
	/* Rt[31:24] = Ra[31:24] + Rb[31:24]
	   Rt[23:16] = Ra[23:16] + Rb[23:16]
	   Rt[15:8] = Ra[8:0] + Rb[8:0]
	   Rt[7:0] = Ra[7:0] + Rb[7:0] */
	for (i = 0; i < vec8_num; i++)
	  {
	    res = *(ptr_a8 + i) + *(ptr_b8 + i);
	    *(ptr8 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_RADD8:
      {
	/* Rt[31:24] = (Ra[31:24] + Rb[31:24]) >> 1
	   Rt[23:16] = (Ra[23:16] + Rb[23:16]) >> 1
	   Rt[15:8] = (Ra[8:0] + Rb[8:0]) >> 1
	   Rt[7:0] = (Ra[7:0] + Rb[7:0]) >> 1 */
	for (i = 0; i < vec8_num; i++)
	  {
	    res = (int8_t) (((int16_t) *(ptr_a8 + i) + *(ptr_b8 + i)) >> 1);
	    *(ptr8 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_URADD8:
      {
	/* Rt[31:24] = (Ra[31:24] + Rb[31:24]) >> 1
	   Rt[23:16] = (Ra[23:16] + Rb[23:16]) >> 1
	   Rt[15:8] = (Ra[8:0] + Rb[8:0]) >> 1
	   Rt[7:0] = (Ra[7:0] + Rb[7:0]) >> 1 */
	for (i = 0; i < vec8_num; i++)
	  {
	    res = (uint8_t) (((uint16_t) *(uptr_a8 + i) + *(uptr_b8 + i)) >> 1);
	    *(uptr8 + i) = res;
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KADD8:
      {
	/* Rt[31:24] = Ra[31:24] + Rb[31:24]
	   Rt[23:16] = Ra[23:16] + Rb[23:16]
	   Rt[15:8] = Ra[8:0] + Rb[8:0]
	   Rt[7:0] = Ra[7:0] + Rb[7:0] */
	for (i = 0; i < vec8_num; i++)
	  {
	    res = *(ptr_a8 + i) + *(ptr_b8 + i);
	    *(ptr8 + i) = insn_sat_helper (cpu, res, 7);
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UKADD8:
      {
	/* Rt[31:24] = Ra[31:24] + Rb[31:24]
	   Rt[23:16] = Ra[23:16] + Rb[23:16]
	   Rt[15:8] = Ra[8:0] + Rb[8:0]
	   Rt[7:0] = Ra[7:0] + Rb[7:0] */
	for (i = 0; i < vec8_num; i++)
	  {
	    res = *(uptr_a8 + i) + *(uptr_b8 + i);
	    *(uptr8 + i) = insn_usat_helper (cpu, res, 8);
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SUB8:
      {
	/* Rt[31:24] = Ra[31:24] - Rb[31:24]
	   Rt[23:16] = Ra[23:16] - Rb[23:16]
	   Rt[15:8] = Ra[8:0] - Rb[8:0]
	   Rt[7:0] = Ra[7:0] - Rb[7:0] */
	for (i = 0; i < vec8_num; i++)
	  {
	    res = *(ptr_a8 + i) - *(ptr_b8 + i);
	    *(ptr8 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_RSUB8:
      {
	/* Rt[31:24] = (Ra[31:24] - Rb[31:24]) >> 1
	   Rt[23:16] = (Ra[23:16] - Rb[23:16]) >> 1
	   Rt[15:8] = (Ra[8:0] - Rb[8:0]) >> 1
	   Rt[7:0] = (Ra[7:0] - Rb[7:0]) >> 1 */
	for (i = 0; i < vec8_num; i++)
	  {
	    res = (int8_t) (((int16_t) *(ptr_a8 + i) - *(ptr_b8 + i)) >> 1);
	    *(ptr8 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_URSUB8:
      {
	/* Rt[31:24] = (Ra[31:24] - Rb[31:24]) >> 1
	   Rt[23:16] = (Ra[23:16] - Rb[23:16]) >> 1
	   Rt[15:8] = (Ra[8:0] - Rb[8:0]) >> 1
	   Rt[7:0] = (Ra[7:0] - Rb[7:0]) >> 1 */
	for (i = 0; i < vec8_num; i++)
	  {
	    res = (uint8_t) (((uint16_t) *(uptr_a8 + i) - *(uptr_b8 + i)) >> 1);
	    *(uptr8 + i) = res;
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KSUB8:
      {
	/* Rt[31:24] = Ra[31:24] - Rb[31:24]
	   Rt[23:16] = Ra[23:16] - Rb[23:16]
	   Rt[15:8] = Ra[8:0] - Rb[8:0]
	   Rt[7:0] = Ra[7:0] - Rb[7:0] */
	for (i = 0; i < vec8_num; i++)
	  {
	    res = *(ptr_a8 + i) - *(ptr_b8 + i);
	    *(ptr8 + i) = insn_sat_helper (cpu, res, 7);
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UKSUB8:
      {
	/* Rt[31:24] = Ra[31:24] - Rb[31:24]
	   Rt[23:16] = Ra[23:16] - Rb[23:16]
	   Rt[15:8] = Ra[8:0] - Rb[8:0]
	   Rt[7:0] = Ra[7:0] - Rb[7:0] */
	for (i = 0; i < vec8_num; i++)
	  {
	    res = *(uptr_a8 + i) - *(uptr_b8 + i);
	    *(uptr8 + i) = insn_usat_helper (cpu, res, 8);
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SRA16:
      {
	for (i = 0; i < vec16_num; i++)
	  {
	    res = *(ptr_a16 + i) >> (cpu->regs[rb].u & 0xf);
	    *(ptr16 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SRAI16:
      {
	for (i = 0; i < vec16_num; i++)
	  {
	    res = *(ptr_a16 + i) >> imm4u;
	    *(ptr16 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SRA32:
      {
	for (i = 0; i < vec32_num; i++)
	  {
	    res = *(ptr_a32 + i) >> (cpu->regs[rb].u & 0x1f);
	    *(ptr32 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SRAI32:
      {
	for (i = 0; i < vec32_num; i++)
	  {
	    res = *(ptr_a32 + i) >> imm5u;
	    *(ptr32 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SRA8:
      {
	for (i = 0; i < vec8_num; i++)
	  {
	    res = *(ptr_a8 + i) >> (cpu->regs[rb].u & 0x7);
	    *(ptr8 + i) = res;
	  }

	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SRAI8:
      {
	for (i = 0; i < vec8_num; i++)
	  {
	    res = *(ptr_a8 + i) >> imm3u;
	    *(ptr8 + i) = res;
	  }

	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SRA16_U:
      {
	uint32_t rnd_mask = (1UL << (cpu->regs[rb].u - 1));
	int16_t rnd_val;
	for (i = 0; i < vec16_num; i++)
	  {
	    rnd_val = (*(ptr_a16 + i) & rnd_mask) ? 1 : 0;
	    res = (*(ptr_a16 + i) >> (cpu->regs[rb].u & 0xf)) + rnd_val;
	    *(ptr16 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SRA32_U:
      {
	uint32_t rnd_mask = (1UL << (cpu->regs[rb].u - 1));
	int16_t rnd_val;
	for (i = 0; i < vec32_num; i++)
	  {
	    rnd_val = (*(ptr_a32 + i) & rnd_mask) ? 1 : 0;
	    res = (*(ptr_a32 + i) >> (cpu->regs[rb].u & 0x1f)) + rnd_val;
	    *(ptr32 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SRA8_U:
      {
	uint32_t rnd_mask = (1UL << (cpu->regs[rb].u - 1));
	int8_t rnd_val;

	for (i = 0; i < vec8_num; i++)
	  {
	    rnd_val = (*(ptr_a8 + i) & rnd_mask) ? 1 : 0;
	    res = (*(ptr_a8 + i) >> (cpu->regs[rb].u & 0xf)) + rnd_val;
	    *(ptr8 + i) = res;
	  }

	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SRAI16_U:
      {
	uint32_t rnd_mask = (1UL << (imm4u - 1));
	int16_t rnd_val;
	for (i = 0; i < vec16_num; i++)
	  {
	    rnd_val = (*(ptr_a16 + i) & rnd_mask) ? 1 : 0;
	    res = (*(ptr_a16 + i) >> imm4u) + rnd_val;
	    *(ptr16 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SRAI32_U:
      {
	uint32_t rnd_mask = (1UL << (imm5u - 1));
	int16_t rnd_val;
	for (i = 0; i < vec32_num; i++)
	  {
	    rnd_val = (*(ptr_a32 + i) & rnd_mask) ? 1 : 0;
	    res = (*(ptr_a32 + i) >> imm4u) + rnd_val;
	    *(ptr32 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SRAIW_U:
      {
	uint32_t rnd_mask = (1UL << (imm5u - 1));
	int32_t rnd_val = (cpu->regs[ra].b32.i0 & rnd_mask) ? 1 : 0;

	cpu->regs[rd].s = (cpu->regs[ra].b32.i0 >> imm4u) + rnd_val;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SRAI8_U:
      {
	uint32_t rnd_mask = (1UL << (imm3u - 1));
	int8_t rnd_val;

	for (i = 0; i < vec8_num; i++)
	  {
	    rnd_val = (*(ptr_a8 + i) & rnd_mask) ? 1 : 0;
	    res = (*(ptr_a8 + i) >> imm3u) + rnd_val;
	    *(ptr8 + i) = res;
	  }

	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SRL16:
      {
	for (i = 0; i < vec16_num; i++)
	  {
	    res = *(uptr_a16 + i) >> (cpu->regs[rb].u & 0xf);
	    *(uptr16 + i) = res;
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SRLI16:
      {
	for (i = 0; i < vec16_num; i++)
	  {
	    res = *(uptr_a16 + i) >> imm4u;
	    *(uptr16 + i) = res;
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SRL32:
      {
	for (i = 0; i < vec32_num; i++)
	  {
	    res = *(uptr_a32 + i) >> (cpu->regs[rb].u & 0x1f);
	    *(uptr32 + i) = res;
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SRLI32:
      {
	for (i = 0; i < vec32_num; i++)
	  {
	    res = *(uptr_a32 + i) >> imm5u;
	    *(uptr32 + i) = res;
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SRL8:
      {
	for (i = 0; i < vec8_num; i++)
	  {
	    res = *(uptr_a8 + i) >> (cpu->regs[rb].u & 0x7);
	    *(uptr8 + i) = res;
	  }

	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SRLI8:
      {
	for (i = 0; i < vec8_num; i++)
	  {
	    res = *(uptr_a8 + i) >> imm3u;
	    *(uptr8 + i) = res;
	  }

	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SRL16_U:
      {
	uint32_t rnd_mask = (1UL << (cpu->regs[rb].u - 1));
	int16_t rnd_val;
	for (i = 0; i < vec16_num; i++)
	  {
	    rnd_val = (*(uptr_a16 + i) & rnd_mask) ? 1 : 0;
	    res = (*(uptr_a16 + i) >> (cpu->regs[rb].u & 0xf)) + rnd_val;
	    *(uptr16 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SRL32_U:
      {
	uint32_t rnd_mask = (1UL << (cpu->regs[rb].u - 1));
	int16_t rnd_val;
	for (i = 0; i < vec32_num; i++)
	  {
	    rnd_val = (*(uptr_a32 + i) & rnd_mask) ? 1 : 0;
	    res = (*(uptr_a32 + i) >> (cpu->regs[rb].u & 0x1f)) + rnd_val;
	    *(uptr32 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SRL8_U:
      {
	uint32_t rnd_mask = (1UL << (cpu->regs[rb].u - 1));
	int8_t rnd_val;

	for (i = 0; i < vec8_num; i++)
	  {
	    rnd_val = (*(uptr_a8 + i) & rnd_mask) ? 1 : 0;

	    res = (*(uptr_a8 + i) >> (cpu->regs[rb].u & 0xf)) + rnd_val;
	    *(uptr8 + i) = res;
	  }

	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SRLI16_U:
      {
	uint32_t rnd_mask = (1UL << (imm4u - 1));
	int16_t rnd_val;
	for (i = 0; i < vec16_num; i++)
	  {
	    rnd_val = (*(uptr_a16 + i) & rnd_mask) ? 1 : 0;
	    res = (*(uptr_a16 + i) >> imm4u) + rnd_val;
	    *(uptr16 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SRLI32_U:
      {
	uint32_t rnd_mask = (1UL << (imm5u - 1));
	int16_t rnd_val;
	for (i = 0; i < vec32_num; i++)
	  {
	    rnd_val = (*(uptr_a32 + i) & rnd_mask) ? 1 : 0;
	    res = (*(uptr_a32 + i) >> imm5u) + rnd_val;
	    *(uptr32 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SRLI8_U:
      {
	uint32_t rnd_mask = (1UL << (imm3u - 1));
	int8_t rnd_val;

	for (i = 0; i < vec8_num; i++)
	  {
	    rnd_val = (*(uptr_a8 + i) & rnd_mask) ? 1 : 0;

	    res = (*(uptr_a8 + i) >> imm3u) + rnd_val;
	    *(uptr8 + i) = res;
	  }

	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SLL16:
      {
	for (i = 0; i < vec16_num; i++)
	  {
	    res = *(ptr_a16 + i) << (cpu->regs[rb].u & 0xf);
	    *(ptr16 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SLLI16:
      {
	for (i = 0; i < vec16_num; i++)
	  {
	    res = *(ptr_a16 + i) << imm4u;
	    *(ptr16 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SLL32:
      {
	for (i = 0; i < vec32_num; i++)
	  {
	    res = *(ptr_a32 + i) << (cpu->regs[rb].u & 0x1f);
	    *(ptr32 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SLLI32:
      {
	for (i = 0; i < vec32_num; i++)
	  {
	    res = *(ptr_a32 + i) << imm5u;
	    *(ptr32 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SLL8:
      {
	for (i = 0; i < vec8_num; i++)
	  {
	    res = *(ptr_a8 + i) << (cpu->regs[rb].u & 0x7);
	    *(ptr8 + i) = res;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SLLI8:
      {
	for (i = 0; i < vec8_num; i++)
	  {
	    res = *(ptr_a8 + i) << imm3u;
	    *(ptr8 + i) = res;
	  }

	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KSLL8:
      {
	int32_t sa = cpu->regs[rb].u & 0x7;
	if (sa != 0)
	  {
	    for (i = 0; i < vec8_num; i++)
	      {
		res = (int32_t) *(ptr_a8 + i) << sa;
		*(ptr8 + i) = insn_sat_helper (cpu, res, 7);;
	      }
	    cpu->regs[rd].s = result.s;
	  }
	else
	  cpu->regs[rd].s = cpu->regs[ra].s;

	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KSLLI8:
      {
	if (imm3u != 0)
	  {
	    for (i = 0; i < vec8_num; i++)
	      {
		res = (int32_t) *(ptr_a8 + i) << imm3u;
		*(ptr8 + i) = insn_sat_helper (cpu, res, 7);;
	      }
	    cpu->regs[rd].s = result.s;
	  }
	else
	  cpu->regs[rd].s = cpu->regs[ra].s;

	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KSLL16:
      {
	int32_t res1, res2;
	int32_t sa = cpu->regs[rb].u & 0xf;
	if (sa != 0)
	  {
	    for (i = 0; i < vec16_num; i++)
	      {
		res = (int32_t) *(ptr_a16 + i) << sa;
		*(ptr16 + i) = insn_sat_helper (cpu, res, 15);
	      }
	    cpu->regs[rd].s = result.s;
	  }
	else
	  cpu->regs[rd].s = cpu->regs[ra].s;

	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KSLLI16:
      {
	int32_t res1, res2;

	if (imm3u != 0)
	  {
	    for (i = 0; i < vec16_num; i++)
	      {
		res = (int32_t) *(ptr_a16 + i) << imm3u;
		*(ptr16 + i) = insn_sat_helper (cpu, res, 15);
	      }
	    cpu->regs[rd].s = result.s;
	  }
	else
	  cpu->regs[rd].s = cpu->regs[ra].s;

	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KSLL32:
      {
	int32_t sa = cpu->regs[rb].u & 0x1f;
	if (sa != 0)
	  {
	    int64_t res;
	    for (i = 0; i < vec32_num; i++)
	      {
		res = (int64_t) *(ptr_a32 + i) << sa;
		*(ptr32 + i) = insn_sat_helper (cpu, res, 31);
	      }
	    cpu->regs[rd].s = result.s;
	  }
	else
	  cpu->regs[rd].s = cpu->regs[ra].s;

	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KSLLI32:
      {
	if (imm5u != 0)
	  {
	    int64_t res;
	    for (i = 0; i < vec32_num; i++)
	      {
		res = (int64_t) *(ptr_a32 + i) << imm5u;
		*(ptr32 + i) = insn_sat_helper (cpu, res, 31);
	      }
	    cpu->regs[rd].s = result.s;
	  }
	else
	  cpu->regs[rd].s = cpu->regs[ra].s;

	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KSLRA16:
      {
	if (cpu->regs[rb].b8.b0 < 0)
	  {
	    for (i = 0; i < vec16_num; i++)
	      {
		res = *(ptr_a16 + i) >> (-cpu->regs[rb].b8.b0);
		*(ptr16 + i) = res;
	      }
	    cpu->regs[rd].s = result.s;
	  }
	else
	  {
	    if (cpu->regs[rb].b8.b0 != 0)
	      {
		for (i = 0; i < vec16_num; i++)
		  {
		    res = (int32_t) *(ptr_a16 + i) << (cpu->regs[rb].b8.b0);
		    *(ptr16 + i) = insn_sat_helper (cpu, res, 15);
		  }
		cpu->regs[rd].s = result.s;
	      }
	    else
	      cpu->regs[rd].s = cpu->regs[ra].s;
	  }

	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KSLRA32:
      {
	uint8_t sh = (cpu->regs[rb].b8.b0 & 0x2f);

	if ((cpu->regs[rb].b8.b0 >> 5) & 1)
	  {
	    int new_sh = - (int8_t) (sh | 0xc0);
	    new_sh = (new_sh == 32) ? 31: new_sh;

	    for (i = 0; i < vec32_num; i++)
	      {
		res = *(ptr_a32 + i) >> new_sh;
		*(ptr32 + i) = res;
	      }
	    cpu->regs[rd].s = result.s;
	  }
	else
	  {
	    if ((cpu->regs[rb].b8.b0 & 0x2f) != 0)
	      {
		int64_t res1;
		for (i = 0; i < vec32_num; i++)
		  {
		    res1 = (int64_t) *(ptr_a32 + i) << sh;
		    *(ptr32 + i) = insn_sat_helper (cpu, res1, 31);
		  }
		cpu->regs[rd].s = result.s;
	      }
	    else
	      cpu->regs[rd].s = cpu->regs[ra].s;
	  }

	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KSLRA8:
      {
	if (cpu->regs[rb].b8.b0 < 0)
	  {
	    for (i = 0; i < vec8_num; i++)
	      {
		res = *(ptr_a8 + i) >> (-cpu->regs[rb].b8.b0);
		*(ptr8 + i) = res;
	      }
	    cpu->regs[rd].s = result.s;
	  }
	else
	  {
	    if (cpu->regs[rb].b8.b0 != 0)
	      {
		for (i = 0; i < vec8_num; i++)
		  {
		    res = (int32_t) *(ptr_a8 + i) << (cpu->regs[rb].b8.b0);
		    *(ptr8 + i) = insn_sat_helper (cpu, res, 7);
		  }
		cpu->regs[rd].s = result.s;

	      }
	    else
	      cpu->regs[rd].s = cpu->regs[ra].s;
	  }

	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KSLRA16_U:
      {
	if (cpu->regs[rb].b8.b0 < 0)
	  {
	    int rnd;
	    uint32_t mask_sh;
	    int sh = -cpu->regs[rb].b8.b0;
	    sh = (sh == 16) ? 15: sh;
	    mask_sh = 1UL << (sh - 1);

	    for (i = 0; i < vec16_num; i++)
	      {
		rnd = (*(ptr_a16 + i) & mask_sh) ? 1 : 0;
		res = *(ptr_a16 + i) >> sh;
		*(ptr16 + i) = res;
	      }
	    cpu->regs[rd].s = result.s;
	  }
	else
	  {
	    int32_t res1, res2;
	    if (cpu->regs[rb].b8.b0 != 0)
	      {
		for (i = 0; i < vec16_num; i++)
		  {
		    res = (int32_t) *(ptr_a16 + i) << cpu->regs[rb].b8.b0;
		    *(ptr16 + i) = insn_sat_helper (cpu, res, 15);
		  }
		cpu->regs[rd].s = result.s;
	      }
	    else
	      cpu->regs[rd].s = cpu->regs[ra].s;
	  }

	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KSLRA32_U:
      {
	uint8_t sh = (cpu->regs[rb].b8.b0 & 0x2f);

	if ((cpu->regs[rb].b8.b0 >> 5) & 1)
	  {
	    int rnd;
	    uint32_t mask_sh;
	    int new_sh = - (int8_t) (sh | 0xc0);
	    new_sh = (new_sh == 32) ? 31: new_sh;
	    mask_sh = 1UL << (new_sh - 1);

	    for (i = 0; i < vec32_num; i++)
	      {
		rnd = (*(ptr_a32 + i) & mask_sh) ? 1 : 0;
		res = *(ptr_a32 + i) >> new_sh;
		*(ptr16 + i) = res + rnd;
	      }
	    cpu->regs[rd].s = result.s;
	  }
	else
	  {
	    int64_t res1;
	    if (sh != 0)
	      {
		for (i = 0; i < vec32_num; i++)
		  {
		    res1 = (int64_t) *(ptr_a32 + i) << sh;
		    *(ptr32 + i) = insn_sat_helper (cpu, res1, 31);
		  }
		cpu->regs[rd].s = result.s;
	      }
	    else
	      cpu->regs[rd].s = cpu->regs[ra].s;
	  }

	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KSLRA8_U:
      {
	if (cpu->regs[rb].b8.b0 < 0)
	  {
	    int rnd;
	    uint32_t mask_sh;
	    int sh = -cpu->regs[rb].b8.b0;
	    sh = (sh == 8) ? 7: sh;
	    mask_sh = 1UL << (sh - 1);

	    for (i = 0; i < vec8_num; i++)
	      {
		rnd = (*(ptr_a8 + i) & mask_sh) ? 1 : 0;

		res = *(ptr_a8 + i) >> sh;
		*(ptr8 + i) = res + rnd;
	      }
	    cpu->regs[rd].s = result.s;
	  }
	else
	  {
	    int32_t res1, res2, res3, res4;
	    if (cpu->regs[rb].b8.b0 != 0)
	      {
		for (i = 0; i < vec8_num; i++)
		  {
		    res = (int32_t) *(ptr_a8 + i) << cpu->regs[rb].b8.b0;
		    *(ptr8 + i) = insn_sat_helper (cpu, res, 7);
		  }
		cpu->regs[rd].s = result.s;
	      }
	    else
	      cpu->regs[rd].s = cpu->regs[ra].s;
	  }
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_CMPEQ16:
      {
	for (i = 0; i < vec16_num; i++)
	  {
	    if (*(ptr_a16 + i) == *(ptr_b16 + i))
	      *(ptr16 + i) = 0xffff;
	    else
	      *(ptr16 + i) = 0;
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SCMPLT16:
      {
	for (i = 0; i < vec16_num; i++)
	  {
	    if (*(ptr_a16 + i) < *(ptr_b16 + i))
	      *(ptr16 + i) = 0xffff;
	    else
	      *(ptr16 + i) = 0;
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SCMPLE16:
      {
	for (i = 0; i < vec16_num; i++)
	  {
	    if (*(ptr_a16 + i) <= *(ptr_b16 + i))
	      *(ptr16 + i) = 0xffff;
	    else
	      *(ptr16 + i) = 0;
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SMIN16:
      {
	for (i = 0; i < vec16_num; i++)
	  {
	    if (*(ptr_a16 + i) < *(ptr_b16 + i))
	      *(ptr16 + i) = *(ptr_a16 + i);
	    else
	      *(ptr16 + i) = *(ptr_b16 + i);
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SMIN32:
      {
	for (i = 0; i < vec32_num; i++)
	  {
	    if (*(ptr_a32 + i) < *(ptr_b32 + i))
	      *(ptr32 + i) = *(ptr_a32 + i);
	    else
	      *(ptr32 + i) = *(ptr_b32 + i);
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_CMPEQ8:
      {
	for (i = 0; i < vec8_num; i++)
	  {
	    if (*(ptr_a8 + i) == *(ptr_b8 + i))
	      *(ptr8 + i) = 0xff;
	    else
	      *(ptr8 + i) = 0;
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SCMPLT8:
      {
	for (i = 0; i < vec8_num; i++)
	  {
	    if (*(ptr_a8 + i) < *(ptr_b8 + i))
	      *(ptr8 + i) = 0xff;
	    else
	      *(ptr8 + i) = 0;
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SCMPLE8:
      {
	for (i = 0; i < vec8_num; i++)
	  {
	    if (*(ptr_a8 + i) <= *(ptr_b8 + i))
	      *(ptr8 + i) = 0xff;
	    else
	      *(ptr8 + i) = 0;
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SMIN8:
      {
	for (i = 0; i < vec8_num; i++)
	  {
	    if (*(ptr_a8 + i) < *(ptr_b8 + i))
	      *(ptr8 + i) = *(ptr_a8 + i);
	    else
	      *(ptr8 + i) = *(ptr_b8 + i);
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UCMPLT16:
      {
	for (i = 0; i < vec16_num; i++)
	  {
	    if (*(uptr_a16 + i) < *(uptr_b16 + i))
	      *(uptr16 + i) = 0xffff;
	    else
	      *(uptr16 + i) = 0;
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UCMPLE16:
      {
	for (i = 0; i < vec16_num; i++)
	  {
	    if (*(uptr_a16 + i) <= *(uptr_b16 + i))
	      *(uptr16 + i) = 0xffff;
	    else
	      *(uptr16 + i) = 0;
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UMIN16:
      {
	for (i = 0; i < vec16_num; i++)
	  {
	    if (*(uptr_a16 + i) < *(uptr_b16 + i))
	      *(uptr16 + i) = *(uptr_a16 + i);
	    else
	      *(uptr16 + i) = *(uptr_b16 + i);
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UMIN32:
      {
	for (i = 0; i < vec32_num; i++)
	  {
	    if (*(uptr_a32 + i) < *(uptr_b32 + i))
	      *(uptr32 + i) = *(uptr_a32 + i);
	    else
	      *(uptr32 + i) = *(uptr_b32 + i);
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UCMPLT8:
      {
	for (i = 0; i < vec8_num; i++)
	  {
	    if (*(uptr_a8 + i) < *(uptr_b8 + i))
	      *(uptr8 + i) = 0xff;
	    else
	      *(uptr8 + i) = 0;
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UCMPLE8:
      {
	for (i = 0; i < vec8_num; i++)
	  {
	    if (*(uptr_a8 + i) <= *(uptr_b8 + i))
	      *(uptr8 + i) = 0xff;
	    else
	      *(uptr8 + i) = 0;
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UMIN8:
      {
	for (i = 0; i < vec8_num; i++)
	  {
	    if (*(uptr_a8 + i) < *(uptr_b8 + i))
	      *(uptr8 + i) = *(uptr_a8 + i);
	    else
	      *(uptr8 + i) = *(uptr_b8 + i);
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SCLIP16:
      {
	for (i = 0; i < vec16_num; i++)
	  *(ptr16 + i) = insn_sat_helper (cpu, *(ptr_a16 + i), imm4u);
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SMAX16:
      {
	/* Rt[31:16] = (Ra[31:16] > Rb[31:16])? Ra[31:16] : Rb[31:16]
	   Rt[15:0] = (Ra[15:0] > Rb[15:0]) ? Ra[15:0] : Rb[15:0] */
	for (i = 0; i < vec16_num; i++)
	  {
	    if (*(ptr_a16 + i) > *(ptr_b16 + i))
	      *(ptr16 + i) = *(ptr_a16 + i);
	    else
	      *(ptr16 + i) = *(ptr_b16 + i);
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SMAX32:
      {
	for (i = 0; i < vec32_num; i++)
	  {
	    if (*(ptr_a32 + i) > *(ptr_b32 + i))
	      *(ptr32 + i) = *(ptr_a32 + i);
	    else
	      *(ptr32 + i) = *(ptr_b32 + i);
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SMAX8:
      {
	/* Rt[31:24] = (Ra[31:24] > Rb[31:24])? Ra[31:24] : Rb[31:24]
	   Rt[23:16] = (Ra[23:16] > Rb[23:16])? Ra[23:16] : Rb[23:16]
	   Rt[15:8]  = (Ra[15:8]  > Rb[15:8]) ? Ra[15:8]  : Rb[15:8]
	   Rt[7:0]   = (Ra[7:0]   > Rb[7:0])  ? Ra[7:0]   : Rb[7:0]  */
	for (i = 0; i < vec8_num; i++)
	  {
	    if (*(ptr_a8 + i) > *(ptr_b8 + i))
	      *(ptr8 + i) = *(ptr_a8 + i);
	    else
	      *(ptr8 + i) = *(ptr_b8 + i);
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UCLIP16:
      {
	for (i = 0; i < vec16_num; i++)
	  *(ptr16 + i) = insn_usat_helper (cpu, *(ptr_a16 + i), imm4u);
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UMAX16:
      {
	/* Rt[31:16] = (Ra[31:16] > Rb[31:16])? Ra[31:16] : Rb[31:16]
	   Rt[15:0] = (Ra[15:0] > Rb[15:0]) ? Ra[15:0] : Rb[15:0] */
	for (i = 0; i < vec16_num; i++)
	  {
	    if (*(uptr_a16 + i) > *(uptr_b16 + i))
	      *(uptr16 + i) = *(uptr_a16 + i);
	    else
	      *(uptr16 + i) = *(uptr_b16 + i);
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UMAX32:
      {
	for (i = 0; i < vec32_num; i++)
	  {
	    if (*(uptr_a32 + i) > *(uptr_b32 + i))
	      *(uptr32 + i) = *(uptr_a32 + i);
	    else
	      *(uptr32 + i) = *(uptr_b32 + i);
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UMAX8:
      {
	/* Rt[31:24] = (Ra[31:24] > Rb[31:24])? Ra[31:24] : Rb[31:24]
	   Rt[23:16] = (Ra[23:16] > Rb[23:16])? Ra[23:16] : Rb[23:16]
	   Rt[15:8]  = (Ra[15:8]  > Rb[15:8]) ? Ra[15:8]  : Rb[15:8]
	   Rt[7:0]   = (Ra[7:0]   > Rb[7:0])  ? Ra[7:0]   : Rb[7:0]  */
	for (i = 0; i < vec8_num; i++)
	  {
	    if (*(uptr_a8 + i) > *(uptr_b8 + i))
	      *(uptr8 + i) = *(uptr_a8 + i);
	    else
	      *(uptr8 + i) = *(uptr_b8 + i);
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KHM16:
      {
	for (i = 0; i < vec16_num; i+=2)
	  {
	    *(ptr16 + i) = insn_sat_khm_helper (cpu, *(ptr_a16 + i),
						*(ptr_b16 + i));
	    *(ptr16 + (i + 1)) = insn_sat_khm_helper (cpu, *(ptr_a16 + (i + 1)),
						       *(ptr_b16 + (i + 1)));
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KHMX16:
      {
	for (i = 0; i < vec16_num; i+=2)
	  {
	    *(ptr16 + i) = insn_sat_khm_helper (cpu, *(ptr_a16 + i),
						*(ptr_b16 + (i + 1)));
	    *(ptr16 + (i + 1)) = insn_sat_khm_helper (cpu, *(ptr_a16 + (i + 1)),
						       *(ptr_b16 + i));
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KHM8:
      {
	for (i = 0; i < vec8_num; i+=2)
	  {
	    *(ptr8 + i) = insn_sat_khm8_helper (cpu, *(ptr_a8 + i),
						*(ptr_b8 + i));
	    *(ptr8 + (i + 1)) = insn_sat_khm8_helper (cpu, *(ptr_a8 + (i + 1)),
						      *(ptr_b8 + (i + 1)));
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KHMX8:
      {
	for (i = 0; i < vec8_num; i+=2)
	  {
	    *(ptr8 + i) = insn_sat_khm8_helper (cpu, *(ptr_a8 + i),
						*(ptr_b8 + (i + 1)));
	    *(ptr8 + (i + 1)) = insn_sat_khm8_helper (cpu, *(ptr_a8 + (i + 1)),
						      *(ptr_b8 + i));
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KABSW:
      {
	int64_t res;
	if (*ptr_a32 >= 0)
	  res = *ptr_a32;
	else if (*ptr_a32 == 0x80000000)
	  {
	    res = 0x7fffffff;
	    CCPU_UCODE_OV_SET();
	  }
	else
	  res = -*ptr_a32;

	cpu->regs[rd].s = res;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KABS16:
      {
	reg_t result;
	int16_t *ptr, i;
	result.u = cpu->regs[ra].u;
	for (i = 0; i < vec16_num; i++)
	  {
	    ptr = (int16_t *) &result.b16 + i;
	    if ((*ptr) == -0x8000)
	      {
		*ptr = 0x7fff;
	        CCPU_UCODE_OV_SET();
	      }
	    else if (*ptr & 0x8000)
	      *ptr = -(*ptr);
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KABS32:
      {
	reg_t result;
	int32_t *ptr, i;
	result.u = cpu->regs[ra].u;
	for (i = 0; i < vec32_num; i++)
	  {
	    ptr = (int32_t *) &result.b32 + i;
	    if ((*ptr) == -0x80000000)
	      {
		*ptr = 0x7fffffff;
	        CCPU_UCODE_OV_SET();
	      }
	    else if (*ptr & 0x80000000)
	      *ptr = -(*ptr);
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KABS8:
      {
	reg_t result;
	int8_t *ptr, i;
	result.u = cpu->regs[ra].u;
	for (i = 0; i < vec8_num; i++)
	  {
	    ptr = (int8_t *) &result.b8 + i;
	    if ((*ptr) == -0x80)
	      {
		*ptr = 0x7f;
	        CCPU_UCODE_OV_SET();
	      }
	    else if (*ptr & 0x80)
	      *ptr = -(*ptr);
	  }
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SUNPKD810:
      {
	reg_t result;
	result.b16.h1 = (int16_t) cpu->regs[ra].b8.b1;
	result.b16.h0 = (int16_t) cpu->regs[ra].b8.b0;

#if (WITH_TARGET_WORD_BITSIZE == 64)
	result.b16.h3 = (int16_t) cpu->regs[ra].b8.b5;
	result.b16.h2 = (int16_t) cpu->regs[ra].b8.b4;
#endif
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SUNPKD820:
      {
	reg_t result;
	result.b16.h1 = (int16_t) cpu->regs[ra].b8.b2;
	result.b16.h0 = (int16_t) cpu->regs[ra].b8.b0;

#if (WITH_TARGET_WORD_BITSIZE == 64)
	result.b16.h3 = (int16_t) cpu->regs[ra].b8.b6;
	result.b16.h2 = (int16_t) cpu->regs[ra].b8.b4;
#endif
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SUNPKD830:
      {
	reg_t result;
	result.b16.h1 = (int16_t) cpu->regs[ra].b8.b3;
	result.b16.h0 = (int16_t) cpu->regs[ra].b8.b0;

#if (WITH_TARGET_WORD_BITSIZE == 64)
	result.b16.h3 = (int16_t) cpu->regs[ra].b8.b7;
	result.b16.h2 = (int16_t) cpu->regs[ra].b8.b4;
#endif
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SUNPKD831:
      {
	reg_t result;
	result.b16.h1 = (int16_t) cpu->regs[ra].b8.b3;
	result.b16.h0 = (int16_t) cpu->regs[ra].b8.b1;

#if (WITH_TARGET_WORD_BITSIZE == 64)
	result.b16.h3 = (int16_t) cpu->regs[ra].b8.b7;
	result.b16.h2 = (int16_t) cpu->regs[ra].b8.b5;
#endif
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SUNPKD832:
      {
	reg_t result;
	result.b16.h1 = (int16_t) cpu->regs[ra].b8.b3;
	result.b16.h0 = (int16_t) cpu->regs[ra].b8.b2;

#if (WITH_TARGET_WORD_BITSIZE == 64)
	result.b16.h3 = (int16_t) cpu->regs[ra].b8.b7;
	result.b16.h2 = (int16_t) cpu->regs[ra].b8.b6;
#endif
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_ZUNPKD810:
      {
	reg_t result;
	result.ub16.h1 = (uint16_t) cpu->regs[ra].ub8.b1;
	result.ub16.h0 = (uint16_t) cpu->regs[ra].ub8.b0;

#if (WITH_TARGET_WORD_BITSIZE == 64)
	result.ub16.h3 = (uint16_t) cpu->regs[ra].ub8.b5;
	result.ub16.h2 = (uint16_t) cpu->regs[ra].ub8.b4;
#endif
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_ZUNPKD820:
      {
	reg_t result;
	result.ub16.h1 = (uint16_t) cpu->regs[ra].ub8.b2;
	result.ub16.h0 = (uint16_t) cpu->regs[ra].ub8.b0;

#if (WITH_TARGET_WORD_BITSIZE == 64)
	result.ub16.h3 = (uint16_t) cpu->regs[ra].ub8.b6;
	result.ub16.h2 = (uint16_t) cpu->regs[ra].ub8.b4;
#endif
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_ZUNPKD830:
      {
	reg_t result;
	result.ub16.h1 = (uint16_t) cpu->regs[ra].ub8.b3;
	result.ub16.h0 = (uint16_t) cpu->regs[ra].ub8.b0;

#if (WITH_TARGET_WORD_BITSIZE == 64)
	result.ub16.h3 = (uint16_t) cpu->regs[ra].ub8.b7;
	result.ub16.h2 = (uint16_t) cpu->regs[ra].ub8.b4;
#endif
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_ZUNPKD831:
      {
	reg_t result;
	result.ub16.h1 = (uint16_t) cpu->regs[ra].ub8.b3;
	result.ub16.h0 = (uint16_t) cpu->regs[ra].ub8.b1;

#if (WITH_TARGET_WORD_BITSIZE == 64)
	result.ub16.h3 = (uint16_t) cpu->regs[ra].ub8.b7;
	result.ub16.h2 = (uint16_t) cpu->regs[ra].ub8.b5;
#endif
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_ZUNPKD832:
      {
	reg_t result;
	result.ub16.h1 = (uint16_t) cpu->regs[ra].ub8.b3;
	result.ub16.h0 = (uint16_t) cpu->regs[ra].ub8.b2;

#if (WITH_TARGET_WORD_BITSIZE == 64)
	result.ub16.h3 = (uint16_t) cpu->regs[ra].ub8.b7;
	result.ub16.h2 = (uint16_t) cpu->regs[ra].ub8.b6;
#endif
	cpu->regs[rd].u = result.u;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_RADDW:
      {
	int32_t val1 = cpu->regs[ra].s;
	int32_t val2 = cpu->regs[rb].s;
	cpu->regs[rd].s = (int32_t) (((int64_t) val1 + (int64_t) val2) >> 1);
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_RSUBW:
      {
	int32_t val1 = cpu->regs[ra].s;
	int32_t val2 = cpu->regs[rb].s;
	cpu->regs[rd].s = (int32_t) (((int64_t) val1 - (int64_t) val2) >> 1);
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_URADDW:
      {
	uint32_t val1 = cpu->regs[ra].u;
	uint32_t val2 = cpu->regs[rb].u;
	cpu->regs[rd].u = (uint32_t) (((uint64_t) val1 + val2) >> 1);

	if (RISCV_XLEN (cpu) == 64)
	  cpu->regs[rd].u = ((int64_t)(int32_t) cpu->regs[rd].u);

	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_URSUBW:
      {
	uint32_t val1 = cpu->regs[ra].u;
	uint32_t val2 = cpu->regs[rb].u;
	cpu->regs[rd].u = (uint32_t) (((uint64_t) val1 - val2) >> 1);

	if (RISCV_XLEN (cpu) == 64)
	  cpu->regs[rd].u = ((int64_t)(int32_t) cpu->regs[rd].u);

	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SRA_U:
      {
	uint32_t rnd_mask = (1UL << (cpu->regs[rb].u - 1));
	int32_t rnd_val = (cpu->regs[ra].s & rnd_mask) ? 1 : 0;
	cpu->regs[rd].s = (cpu->regs[ra].s >> (cpu->regs[rb].u & 0x1f)) + rnd_val ;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SRAI_U:
      {
	uint32_t rnd_mask = (1UL << (imm5u - 1));
	int32_t rnd_val = (cpu->regs[ra].s & rnd_mask) ? 1 : 0;
	cpu->regs[rd].s = (cpu->regs[ra].s >> imm5u) + rnd_val ;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KSLLW:
      {
	int64_t res;
	int sa = cpu->regs[rb].u & 0xf;

	if (sa != 0)
	  {
	    res = (int64_t) cpu->regs[ra].s << sa;
	    res = insn_sat_helper (cpu, res, 31);
	    cpu->regs[rd].s = res;
	  }
	else
	  cpu->regs[rd].s = cpu->regs[ra].s;

	TRACE_REG (cpu, rd);
      }
      break;

    case MATCH_KSLLIW:
      {
	int64_t res;

	if (imm5u != 0)
	  {
	    res = (int64_t) cpu->regs[ra].s << imm5u;
	    res = insn_sat_helper (cpu, res, 31);
	    cpu->regs[rd].s = res;
	  }
	else
	  cpu->regs[rd].s = cpu->regs[ra].s;

	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KSLRAW_U:
      {
	int32_t ret;
	int sh = cpu->regs[rb].b8.b0;

	if (cpu->regs[rb].b8.b0 < 0)
	  {
	    int rnd;
	    uint32_t mask_sh;
	    sh = -cpu->regs[rb].b8.b0;
	    sh = sh > 31 ? 31 : sh;
	    mask_sh = (1UL << (sh - 1));
	    rnd = (cpu->regs[ra].s & mask_sh) ? 1 : 0;
	    ret = cpu->regs[ra].s >> sh;
	    ret += rnd;
	  }
	else
	  {
	    int64_t tmp;
	    sh = cpu->regs[rb].b8.b0 > 31 ? 31 : sh;
	    tmp = (int64_t) cpu->regs[ra].s << sh;
	    ret = insn_sat_helper (cpu, tmp, 31);
	  }
	cpu->regs[rd].s = ret;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_PKTT16:
      {
	/* Rt[31:0] = CONCAT(Ra[31:16], Rb[31:16]) */
#if (WITH_TARGET_WORD_BITSIZE == 32)
	cpu->regs[rd].s = (cpu->regs[ra].b16.h1 << 16)
			   | cpu->regs[rb].ub16.h1;
#else
	cpu->regs[rd].b32.i0 = (cpu->regs[ra].b16.h1 << 16)
				| cpu->regs[rb].ub16.h1;
	cpu->regs[rd].b32.i1 = (cpu->regs[ra].b16.h3 << 16)
				| cpu->regs[rb].ub16.h3;
#endif
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_PKTT32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	reg_t result;
	result.b32.i0 = cpu->regs[rb].b32.i1;
	result.b32.i1 = cpu->regs[ra].b32.i1;
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_PKTB16:
      {
	/* Rt[31:0] = CONCAT(Ra[31:16], Rb[15:0]) */
#if (WITH_TARGET_WORD_BITSIZE == 32)
	cpu->regs[rd].s = (cpu->regs[ra].b16.h1 << 16)
			   | cpu->regs[rb].ub16.h0;
#else
	cpu->regs[rd].b32.i0 = (cpu->regs[ra].b16.h1 << 16)
				| cpu->regs[rb].ub16.h0;
	cpu->regs[rd].b32.i1 = (cpu->regs[ra].b16.h3 << 16)
				| cpu->regs[rb].ub16.h2;
#endif
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_PKTB32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	reg_t result;
	result.b32.i0 = cpu->regs[rb].b32.i0;
	result.b32.i1 = cpu->regs[ra].b32.i1;
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
     case MATCH_PKBT16:
      {
	/* Rt[31:0] = CONCAT(Ra[15:0], Rb[31:16]) */
#if (WITH_TARGET_WORD_BITSIZE == 32)
	cpu->regs[rd].s = (cpu->regs[ra].b16.h0 << 16)
			   | cpu->regs[rb].ub16.h1;
#else
	cpu->regs[rd].b32.i0 = (cpu->regs[ra].b16.h0 << 16)
				| cpu->regs[rb].ub16.h1;
	cpu->regs[rd].b32.i1 = (cpu->regs[ra].b16.h2 << 16)
				| cpu->regs[rb].ub16.h3;
#endif
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_PKBT32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	reg_t result;
	result.b32.i0 = cpu->regs[rb].b32.i1;
	result.b32.i1 = cpu->regs[ra].b32.i0;
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_PKBB16:
      {
	/* Rt[31:0] = CONCAT(Ra[15:0], Rb[15:0]) */
#if (WITH_TARGET_WORD_BITSIZE == 32)
	cpu->regs[rd].s = (cpu->regs[ra].b16.h0 << 16)
			   | cpu->regs[rb].ub16.h0;
#else
	cpu->regs[rd].b32.i0 = (cpu->regs[ra].b16.h0 << 16)
				| cpu->regs[rb].ub16.h0;
	cpu->regs[rd].b32.i1 = (cpu->regs[ra].b16.h2 << 16)
				| cpu->regs[rb].ub16.h2;
#endif
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_PKBB32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	reg_t result;
	result.b32.i0 = cpu->regs[rb].b32.i0;
	result.b32.i1 = cpu->regs[ra].b32.i0;
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_SMMUL:
      {
#if (WITH_TARGET_WORD_BITSIZE == 32)
	cpu->regs[rd].s =
	  ((int64_t) cpu->regs[ra].s * (int64_t) cpu->regs[rb].s) >> 32;
#else
	cpu->regs[rd].b32.i0 = ((int64_t) cpu->regs[ra].b32.i0
				* (int64_t) cpu->regs[rb].b32.i0) >> 32;
	cpu->regs[rd].b32.i1 = ((int64_t) cpu->regs[ra].b32.i1
				* (int64_t) cpu->regs[rb].b32.i1) >> 32;
#endif
	TRACE_REG (cpu, rd);
      }
      break;
   case MATCH_SMMUL_U:
      {
	int32_t round_up;
	int64_t res;
	int vec = RISCV_XLEN (cpu) == 64 ? 1 : 0;

	for (i = 0; i <= vec; i++)
	  {
	    res = (int64_t) *(ptr_a32 + i) * (int64_t) *(ptr_b32 + i);
	    round_up = (res >> 31) & 0x1;
	    if (round_up != 0)
	      *(ptr32 + i) = (res >> 32) + 1;
	    else
	     *(ptr32 + i) = res >> 32;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KMMAC:
      {
	union64_t temp, res;

	for (i = 0; i < vec32_num; i++)
	  {
	    temp.d0 = (int64_t) *(ptr_a32 + i) * *(ptr_b32 + i);
	    res.d0 = (int64_t) cpu->regs[rd].s + temp.b32.w1;
	    res.d0 = insn_sat_helper (cpu, res.d0, 31);
	    *(ptr32 + i) = res.d0;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KMMAC_U:
      {
	union64_t temp, res;

	for (i = 0; i < vec32_num; i++)
	  {
	    temp.d0 = (int64_t) *(ptr_a32 + i) * *(ptr_b32 + i);
	    if ((temp.b32.w0 >> 31) != 0)
	      temp.b32.w1 += 1;

	    res.d0 = (int64_t) cpu->regs[rd].s + temp.b32.w1;
	    res.d0 = insn_sat_helper (cpu, res.d0, 31);
	    *(ptr32 + i) = res.d0;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KWMMUL:
      {
	union64_t temp;
	for (i = 0; i < vec32_num; i++)
	  {
	    if ((*(ptr_a32 + i) != 0x80000000)
		|| (*(ptr_b32 + i) != 0x80000000))
	      {
		temp.d0 = ((int64_t) *(ptr_a32 + i) * *(ptr_b32 + i)) << 1;
		*(ptr32 + i) = temp.b32.w1;
	      }
	    else
	      {
		*(ptr32 + i) = 0x7fffffff;
		CCPU_UCODE_OV_SET();
	      }
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KWMMUL_U:
      {
	union64_t temp;
	for (i = 0; i < vec32_num; i++)
	  {
	    if ((*(ptr_a32 + i) != 0x80000000)
		|| (*(ptr_b32 + i) != 0x80000000))
	      {
		temp.d0 = ((int64_t) *(ptr_a32 + i) * *(ptr_b32 + i));
		/* Let 30bit add 1 and left sh1ft 1.  */
		temp.d0 = (temp.d0 + (int32_t) (1 << 30)) << 1;
		*(ptr32 + i) = temp.b32.w1;
	      }
	    else
	      {
		*(ptr32 + i) = 0x7fffffff;
		CCPU_UCODE_OV_SET();
	      }
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SMMWB:
      {
	int b16_offset = 0;
	int vec = RISCV_XLEN (cpu) == 64 ? 1 : 0;

	for (i = 0; i <= vec; i++)
	  {
	    *(ptr32 + i) = ((int64_t) *(ptr_a32 + i)
			    * (int64_t) *(ptr_b16 + b16_offset)) >> 16;
	    b16_offset += 2;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KMMWB2:
      {
	int b16_offset = 0;
	for (i = 0; i < vec32_num; i++)
	  {
	    if ((*(ptr_a32 + i) == 0x80000000)
		&& (*(ptr_b16 + b16_offset) == (int16_t) 0x8000))
	      {
		*(ptr32 + i) = 0x7fffffff;
	        CCPU_UCODE_OV_SET();
	      }
	    else
	      {
		*(ptr32 + i) = ((int64_t) *(ptr_a32 + i)
				* (int64_t) *(ptr_b16 + b16_offset)) >> 15;
	      }
	    b16_offset += 2;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SMMWB_U:
      {
	int64_t res;
	int round_up, b16_offset = 0;
	int vec = RISCV_XLEN (cpu) == 64 ? 1 : 0;

	for (i = 0; i <= vec; i++)
	  {
	    res = ((int64_t) *(ptr_a32 + i)
		   * (int64_t) *(ptr_b16 + b16_offset));
	    round_up = (res >> 15) & 0x1;
	    if (round_up != 0)
	      *(ptr32 + i) = (res >> 16) + 1;
	    else
	      *(ptr32 + i) = res >> 16;
	    b16_offset += 2;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KMMWB2_U:
      {
	int64_t res;
	int round_up, b16_offset = 0;
	for (i = 0; i < vec32_num; i++)
	  {
	    if ((*(ptr_a32 + i) == 0x80000000)
		&& (*(ptr_b16 + b16_offset) == (int16_t) 0x8000))
	      {
		*(ptr32 + i) = 0x7fffffff;
	        CCPU_UCODE_OV_SET();
	      }
	    else
	      {
		res = ((int64_t) *(ptr_a32 + i)
		       * (int64_t) *(ptr_b16 + b16_offset));
		round_up = (res >> 14) & 0x1;
		if (round_up != 0)
		  *(ptr32 + i) = (res >> 15) + 1;
		else
		  *(ptr32 + i) = res >> 15;
	      }
	    b16_offset += 2;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KMMSB:
      {
	union64_t temp, res;

	for (i = 0; i < vec32_num; i++)
	  {
	    temp.d0 = (int64_t) *(ptr_a32 + i) * *(ptr_b32 + i);
	    res.d0 = (int64_t) cpu->regs[rd].s - temp.b32.w1;
	    res.d0 = insn_sat_helper (cpu, res.d0, 31);
	    *(ptr32 + i) = res.d0;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KMMSB_U:
      {
	union64_t temp, res;

	for (i = 0; i < vec32_num; i++)
	  {
	    temp.d0 = (int64_t) *(ptr_a32 + i) * *(ptr_b32 + i);
	    if ((temp.b32.w0 >> 31) != 0)
	      temp.b32.w1 += 1;

	    res.d0 = (int64_t) cpu->regs[rd].s - temp.b32.w1;
	    res.d0 = insn_sat_helper (cpu, res.d0, 31);
	    *(ptr32 + i) = res.d0;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SMMWT:
      {
	int b16_offset = 1;
	int vec = RISCV_XLEN (cpu) == 64 ? 1 : 0;

	for (i = 0; i <= vec; i++)
	  {
	    *(ptr32 + i) = ((int64_t) *(ptr_a32 + i)
			    * (int64_t) *(ptr_b16 + b16_offset)) >> 16;
	    b16_offset += 2;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KMMWT2:
      {
	int b16_offset = 1;
	for (i = 0; i < vec32_num; i++)
	  {
	    if ((*(ptr_a32 + i) == 0x80000000)
		&& (*(ptr_b16 + b16_offset) == (int16_t) 0x8000))
	      {
		*(ptr32 + i) = 0x7fffffff;
	        CCPU_UCODE_OV_SET();
	      }
	    else
	      {
		*(ptr32 + i) = ((int64_t) *(ptr_a32 + i)
				* (int64_t) *(ptr_b16 + b16_offset)) >> 15;
	      }
	    b16_offset += 2;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SMMWT_U:
      {
	int64_t res;
	int round_up, b16_offset = 1;
	int vec = RISCV_XLEN (cpu) == 64 ? 1 : 0;

	for (i = 0; i <= vec; i++)
	  {
	    res = ((int64_t) *(ptr_a32 + i)
		   * (int64_t) *(ptr_b16 + b16_offset));
	    round_up = (res >> 15) & 0x1;
	    if (round_up != 0)
	      *(ptr32 + i) = (res >> 16) + 1;
	    else
	      *(ptr32 + i) = res >> 16;
	    b16_offset += 2;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KMMWT2_U:
      {
	int64_t res;
	int round_up, b16_offset = 1;
	for (i = 0; i < vec32_num; i++)
	  {
	    if ((*(ptr_a32 + i) == 0x80000000)
		&& (*(ptr_b16 + b16_offset) == (int16_t) 0x8000))
	      {
		*(ptr32 + i) = 0x7fffffff;
	        CCPU_UCODE_OV_SET();
	      }
	    else
	      {
		res = ((int64_t) *(ptr_a32 + i)
		       * (int64_t) *(ptr_b16 + b16_offset));
		round_up = (res >> 14) & 0x1;
		if (round_up != 0)
		  *(ptr32 + i) = (res >> 15) + 1;
		else
		  *(ptr32 + i) = res >> 15;
	      }
	    b16_offset += 2;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KMMAWB:
      {
	union64_t temp;
	int64_t res;
	int b16_offset = 0;
	for (i = 0; i < vec32_num; i++)
	  {
	    temp.d0 = (int64_t) *(ptr_a32 + i) * *(ptr_b16 + b16_offset);
	    res = (int64_t) *(ptr_d32 + i) + (int32_t) (temp.d0 >> 16);
	    *(ptr32 + i) = insn_sat_helper (cpu, res, 31);
	    b16_offset += 2;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KMMAWB2:
      {
	int32_t addop;
	union64_t temp;
	int64_t res;
	int b16_offset = 0;
	for (i = 0; i < vec32_num; i++)
	  {
	    if (*(ptr_a32 + i) == 0x80000000
	        && (*(ptr_b16 + b16_offset)) == (int16_t) 0x8000)
	      {
	        addop = 0x7fffffff;
	        CCPU_UCODE_OV_SET();
	      }
	    else
	      {
	        temp.d0 = (int64_t) *(ptr_a32 + i) * *(ptr_b16 + b16_offset);
	        addop = temp.d0 >> 15;
	      }

	    res = (int64_t) *(ptr_d32 + i) + addop;
	    *(ptr32 + i) = insn_sat_helper (cpu, res, 31);
	    b16_offset += 2;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KMMAWB_U:
      {
	union64_t temp;
	int64_t res;
	int32_t rnd_val;
	int b16_offset = 0;
	for (i = 0; i < vec32_num; i++)
	  {
	    temp.d0 = (int64_t) *(ptr_a32 + i) * *(ptr_b16 + b16_offset);
	    rnd_val = (temp.b32.w0 & (1UL << 15)) ? (1L << 16) : 0;
	    temp.d0 += rnd_val;
	    res = (int64_t) *(ptr_d32 + i) + (int32_t) (temp.d0 >> 16);
	    *(ptr32 + i) = insn_sat_helper (cpu, res, 31);
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KMMAWB2_U:
      {
	int addop;
	union64_t temp;
	int64_t res;
	int32_t rnd_val;
	int b16_offset = 0;
	for (i = 0; i < vec32_num; i++)
	  {
	    if (*(ptr_a32 + i) == 0x80000000
	        && (*(ptr_b16 + b16_offset)) == (int16_t) 0x8000)
	      {
	        addop = 0x7fffffff;
	        CCPU_UCODE_OV_SET();
	      }
	    else
	      {
	        temp.d0 = (int64_t) *(ptr_a32 + i) * *(ptr_b16 + b16_offset);
	        rnd_val = (temp.b32.w0 & (1UL << 14)) ? (1L << 15) : 0;
	        temp.d0 += rnd_val;
	        addop = temp.d0 >> 15;
	      }

	    res = (int64_t) *(ptr_d32 + i) + addop;
	    *(ptr32 + i) = insn_sat_helper (cpu, res, 31);
	    b16_offset += 2;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KMMAWT:
      {
	union64_t temp;
	int64_t res;
	int b16_offset = 1;
	for (i = 0; i < vec32_num; i++)
	  {
	    temp.d0 = (int64_t) *(ptr_a32 + i) * *(ptr_b16 + b16_offset);
	    res = (int64_t) *(ptr_d32 + i) + (int32_t) (temp.d0 >> 16);
	    *(ptr32 + i) = insn_sat_helper (cpu, res, 31);
	    b16_offset += 2;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KMMAWT2:
      {
	int32_t addop;
	union64_t temp;
	int64_t res;
	int b16_offset = 1;
	for (i = 0; i < vec32_num; i++)
	  {
	    if (*(ptr_a32 + i) == 0x80000000
	        && (*(ptr_b16 + b16_offset)) == (int16_t) 0x8000)
	      {
	        addop = 0x7fffffff;
	        CCPU_UCODE_OV_SET();
	      }
	    else
	      {
	        temp.d0 = (int64_t) *(ptr_a32 + i) * *(ptr_b16 + b16_offset);
	        addop = temp.d0 >> 15;
	      }

	    res = (int64_t) *(ptr_d32 + i) + addop;
	    *(ptr32 + i) = insn_sat_helper (cpu, res, 31);
	    b16_offset += 2;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KMMAWT_U:
      {
	union64_t temp;
	int64_t res;
	int32_t rnd_val;
	int b16_offset = 1;
	for (i = 0; i < vec32_num; i++)
	  {
	    temp.d0 = (int64_t) *(ptr_a32 + i) * *(ptr_b16 + b16_offset);
	    rnd_val = (temp.b32.w0 & (1UL << 15)) ? (1L << 16) : 0;
	    temp.d0 += rnd_val;
	    res = (int64_t) *(ptr_d32 + i) + (int32_t) (temp.d0 >> 16);
	    *(ptr32 + i) = insn_sat_helper (cpu, res, 31);
	    b16_offset += 2;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KMMAWT2_U:
      {
	int addop;
	union64_t temp;
	int64_t res;
	int32_t rnd_val;
	int b16_offset = 1;
	for (i = 0; i < vec32_num; i++)
	  {
	    if (*(ptr_a32 + i) == 0x80000000
	        && (*(ptr_b16 + b16_offset)) == (int16_t) 0x8000)
	      {
	        addop = 0x7fffffff;
	        CCPU_UCODE_OV_SET();
	      }
	    else
	      {
	        temp.d0 = (int64_t) *(ptr_a32 + i) * *(ptr_b16 + b16_offset);
	        rnd_val = (temp.b32.w0 & (1UL << 14)) ? (1L << 15) : 0;
	        temp.d0 += rnd_val;
	        addop = temp.d0 >> 15;
	      }

	    res = (int64_t) *(ptr_d32 + i) + addop;
	    *(ptr32 + i) = insn_sat_helper (cpu, res, 31);
	    b16_offset += 2;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SMBB16:
      {
	/* Rt = Ra[15:0] * Rb[15:0] */
#if (WITH_TARGET_WORD_BITSIZE == 32)
	cpu->regs[rd].s = cpu->regs[ra].b16.h0 * cpu->regs[rb].b16.h0;
#else
	cpu->regs[rd].b32.i0 = cpu->regs[ra].b16.h0 * cpu->regs[rb].b16.h0;
	cpu->regs[rd].b32.i1 = cpu->regs[ra].b16.h2 * cpu->regs[rb].b16.h2;
#endif
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SMBB32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	cpu->regs[rd].s = (int64_t) cpu->regs[ra].b32.i0 * cpu->regs[rb].b32.i0;
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_SMBT16:
      {
	/* Rt = Ra[15:0] * Rb[31:16] */
#if (WITH_TARGET_WORD_BITSIZE == 32)
	cpu->regs[rd].s = cpu->regs[ra].b16.h0 * cpu->regs[rb].b16.h1;
#else
	cpu->regs[rd].b32.i0 = cpu->regs[ra].b16.h0 * cpu->regs[rb].b16.h1;
	cpu->regs[rd].b32.i1 = cpu->regs[ra].b16.h2 * cpu->regs[rb].b16.h3;
#endif
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SMBT32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	cpu->regs[rd].s = (int64_t) cpu->regs[ra].b32.i0 * cpu->regs[rb].b32.i1;
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_SMTT16:
      {
	/* Rt = Ra[31:16] * Rb[31:16] */
#if (WITH_TARGET_WORD_BITSIZE == 32)
	cpu->regs[rd].s = cpu->regs[ra].b16.h1 * cpu->regs[rb].b16.h1;
#else
	cpu->regs[rd].b32.i0 = cpu->regs[ra].b16.h1 * cpu->regs[rb].b16.h1;
	cpu->regs[rd].b32.i1 = cpu->regs[ra].b16.h3 * cpu->regs[rb].b16.h3;
#endif
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SMTT32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	cpu->regs[rd].s = (int64_t) cpu->regs[ra].b32.i1 * cpu->regs[rb].b32.i1;
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_SMDS32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	cpu->regs[rd].s
	  = ((int64_t) cpu->regs[ra].b32.i1 * cpu->regs[rb].b32.i1)
	    - ((int64_t) cpu->regs[ra].b32.i0 * cpu->regs[rb].b32.i0);
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_SMDRS32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	cpu->regs[rd].s
	  = ((int64_t) cpu->regs[ra].b32.i0 * cpu->regs[rb].b32.i0)
	    - ((int64_t) cpu->regs[ra].b32.i1 * cpu->regs[rb].b32.i1);
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_SMXDS32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	cpu->regs[rd].s
	  = ((int64_t) cpu->regs[ra].b32.i1 * cpu->regs[rb].b32.i0)
	    - ((int64_t) cpu->regs[ra].b32.i0 * cpu->regs[rb].b32.i1);
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_KMABB:
      {
	int32_t mul;
	int64_t res;
	for (i = 0; i <= vec32_num; i+=2)
	  {
	    mul = (int32_t) *(ptr_a16 + i) * *(ptr_b16 + i);
	    res = (int64_t) *(ptr_d32 + (i / 2)) + mul;
	    *(ptr32 + (i / 2)) = insn_sat_helper (cpu, res, 31);
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KMABB32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	int64_t acc = cpu->regs[rd].s;
	int64_t mul_val = (int64_t) cpu->regs[ra].b32.i0 * cpu->regs[rb].b32.i0;
	int64_t res = acc + mul_val;
	if ((acc > 0) && (mul_val > 0))
	  {
	    if (res <= 0)
	      {
		res = 0x7fffffffffffffffLL;
		CCPU_UCODE_OV_SET();
	      }
	  }
	else if ((acc < 0) && (mul_val < 0))
	  {
	    if (res >= 0)
	      {
		res = 0x8000000000000000LL;
		CCPU_UCODE_OV_SET();
	      }
	  }

	cpu->regs[rd].s = res;
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_KMABT:
      {
	int32_t mul;
	int64_t res;
	for (i = 0; i <= vec32_num; i+=2)
	  {
	    mul = (int32_t) *(ptr_a16 + i) * *(ptr_b16 + (i + 1));
	    res = (int64_t) *(ptr_d32 + (i / 2)) + mul;
	    *(ptr32 + (i / 2)) = insn_sat_helper (cpu, res, 31);
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KMABT32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	int64_t acc = cpu->regs[rd].s;
	int64_t mul_val = (int64_t) cpu->regs[ra].b32.i0 * cpu->regs[rb].b32.i1;
	int64_t res = acc + mul_val;
	if ((acc > 0) && (mul_val > 0))
	  {
	    if (res <= 0)
	      {
		res = 0x7fffffffffffffffLL;
		CCPU_UCODE_OV_SET();
	      }
	  }
	else if ((acc < 0) && (mul_val < 0))
	  {
	    if (res >= 0)
	      {
		res = 0x8000000000000000LL;
		CCPU_UCODE_OV_SET();
	      }
	  }

	cpu->regs[rd].s = res;
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_KMATT:
      {
	int32_t mul;
	int64_t res;
	for (i = 0; i <= vec32_num; i+=2)
	  {
	    mul = (int32_t) *(ptr_a16 + (i + 1)) * *(ptr_b16 + (i + 1));
	    res = (int64_t) *(ptr_d32 + (i / 2)) + mul;
	    *(ptr32 + (i / 2)) = insn_sat_helper (cpu, res, 31);
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KMATT32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	int64_t acc = cpu->regs[rd].s;
	int64_t mul_val = (int64_t) cpu->regs[ra].b32.i1 * cpu->regs[rb].b32.i1;
	int64_t res = acc + mul_val;
	if ((acc > 0) && (mul_val > 0))
	  {
	    if (res <= 0)
	      {
		res = 0x7fffffffffffffffLL;
		CCPU_UCODE_OV_SET();
	      }
	  }
	else if ((acc < 0) && (mul_val < 0))
	  {
	    if (res >= 0)
	      {
		res = 0x8000000000000000LL;
		CCPU_UCODE_OV_SET();
	      }
	  }

	cpu->regs[rd].s = res;
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_KMDA:
      {
	for (i = 0; i < vec32_num; i++)
	  {
	    if ((*(ptr_a32 + i) != 0x80008000)
		|| (*(ptr_b32 + i) != 0x80008000))
	      {
		 *(ptr32 + i) =
		    ((int32_t) *(ptr_a16 + (i + 1)) * *(ptr_b16 + (i + 1)))
		     + ((int32_t) *(ptr_a16 + i) * *(ptr_b16 + i));
	      }
	    else
	      {
		*(ptr32 + i) = 0x7fffffff;
		CCPU_UCODE_OV_SET();
	      }
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KMDA32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	if ((cpu->regs[ra].s != 0x8000000080000000LL)
	    || (cpu->regs[rb].s != 0x8000000080000000LL))
	  {
	    cpu->regs[rd].s
	      = ((int64_t) cpu->regs[ra].b32.i0 * cpu->regs[rb].b32.i0)
		+ ((int64_t) cpu->regs[ra].b32.i1 * cpu->regs[rb].b32.i1);
	  }
	else
	  {
	    cpu->regs[rd].s = 0x7fffffffffffffffLL;
	    CCPU_UCODE_OV_SET();
	  }
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_KMXDA:
      {
	for (i = 0; i < vec32_num; i++)
	  {
	    if ((*(ptr_a32 + i) != 0x80008000)
		|| (*(ptr_b32 + i) != 0x80008000))
	      {
		 *(ptr32 + i) =
		    ((int32_t) *(ptr_a16 + (i + 1)) * *(ptr_b16 + i))
		     + ((int32_t) *(ptr_a16 + i) * *(ptr_b16 + (i + 1)));
	      }
	    else
	      {
		*(ptr32 + i) = 0x7fffffff;
		CCPU_UCODE_OV_SET();
	      }
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KMXDA32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	if ((cpu->regs[ra].s != 0x8000000080000000LL)
	    || (cpu->regs[rb].s != 0x8000000080000000LL))
	  {
	    cpu->regs[rd].s
	      = ((int64_t) cpu->regs[ra].b32.i1 * cpu->regs[rb].b32.i0)
		+ ((int64_t) cpu->regs[ra].b32.i0 * cpu->regs[rb].b32.i1);
	  }
	else
	  {
	    cpu->regs[rd].s = 0x7fffffffffffffffLL;
	    CCPU_UCODE_OV_SET();
	  }
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_KMADA:
      {
	int64_t res;
	for (i = 0; i <= vec32_num; i+=2)
	  {
	    res = (int64_t) *(ptr_d32 + (i / 2))
		  + ((int32_t) *(ptr_a16 + (i + 1)) * *(ptr_b16 + (i + 1)))
		  + ((int32_t) *(ptr_a16 + i) * *(ptr_b16 + i));
	    *(ptr32 + (i / 2)) = insn_sat_helper (cpu, res, 31);
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KMAXDA:
      {
	int64_t res;
	for (i = 0; i <= vec32_num; i+=2)
	  {
	    res = (int64_t) *(ptr_d32 + (i / 2))
		  + ((int32_t) *(ptr_a16 + (i + 1)) * *(ptr_b16 + i))
		  + ((int32_t) *(ptr_a16 + i) * *(ptr_b16 + (i + 1)));
	    *(ptr32 + (i / 2)) = insn_sat_helper (cpu, res, 31);
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KMAXDA32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	int64_t acc = cpu->regs[rd].s;
	int64_t add_val =
	  (((int64_t) cpu->regs[ra].b32.i1 * cpu->regs[rb].b32.i0))
	  + ((int64_t) cpu->regs[ra].b32.i0 * cpu->regs[rb].b32.i1);
	int64_t res = acc + add_val;
	if ((acc > 0) && (add_val > 0))
	  {
	    if (res <= 0)
	      {
		res = 0x7fffffffffffffffLL;
		CCPU_UCODE_OV_SET();
	      }
	  }
	else if ((acc < 0) && (add_val < 0))
	  {
	    if (res >= 0)
	      {
		res = 0x8000000000000000LL;
		CCPU_UCODE_OV_SET();
	      }
	  }

	cpu->regs[rd].s = res;
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_KMSDA:
      {
	int64_t res;
	for (i = 0; i <= vec32_num; i+=2)
	  {
	    res = (int64_t) *(ptr_d32 + (i / 2))
		  - ((int32_t) *(ptr_a16 + (i + 1)) * *(ptr_b16 + (i + 1)))
		  - ((int32_t) *(ptr_a16 + i) * *(ptr_b16 + i));
	    *(ptr32 + (i / 2)) = insn_sat_helper (cpu, res, 31);
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KMSDA32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	int64_t acc = cpu->regs[rd].s;
	int64_t add_val =
	  (((int64_t) cpu->regs[ra].b32.i1 * cpu->regs[rb].b32.i1))
	  + ((int64_t) cpu->regs[ra].b32.i0 * cpu->regs[rb].b32.i0);
	int64_t res = acc - add_val;
	if ((acc > 0) && (add_val < 0))
	  {
	    if (res <= 0)
	      {
		res = 0x7fffffffffffffffLL;
		CCPU_UCODE_OV_SET();
	      }
	  }
	else if ((acc < 0) && (add_val > 0))
	  {
	    if (res >= 0)
	      {
		res = 0x8000000000000000LL;
		CCPU_UCODE_OV_SET();
	      }
	  }

	cpu->regs[rd].s = res;
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_KMSXDA:
      {
	int64_t res;
	for (i = 0; i <= vec32_num; i+=2)
	  {
	    res = (int64_t) *(ptr_d32 + (i / 2))
		  - ((int32_t) *(ptr_a16 + (i + 1)) * *(ptr_b16 + i))
		  - ((int32_t) *(ptr_a16 + i) * *(ptr_b16 + (i + 1)));
	    *(ptr32 + (i / 2)) = insn_sat_helper (cpu, res, 31);
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KMSXDA32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	int64_t acc = cpu->regs[rd].s;
	int64_t add_val =
	  (((int64_t) cpu->regs[ra].b32.i1 * cpu->regs[rb].b32.i0))
	  + ((int64_t) cpu->regs[ra].b32.i0 * cpu->regs[rb].b32.i1);
	int64_t res = acc - add_val;
	if ((acc > 0) && (add_val < 0))
	  {
	    if (res <= 0)
	      {
		res = 0x7fffffffffffffffLL;
		CCPU_UCODE_OV_SET();
	      }
	  }
	else if ((acc < 0) && (add_val > 0))
	  {
	    if (res >= 0)
	      {
		res = 0x8000000000000000LL;
		CCPU_UCODE_OV_SET();
	      }
	  }

	cpu->regs[rd].s = res;
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_SMDS:
      {
	/* Rt = (Ra[31:16] * Rb[31:16]) - (Ra[15:0] * Rb[15:0]) */
	for (i = 0; i <= vec32_num; i+=2)
	  {
	    *(ptr32 + (i / 2)) =
	      ((int32_t) *(ptr_a16 + (i + 1)) * *(ptr_b16 + (i + 1)))
	       - ((int32_t) *(ptr_a16 + i) * *(ptr_b16 + i));
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SMXDS:
      {
	/* Rt = (Ra[31:16] * Rb[15:0]) - (Ra[15:0] * Rb[31:16]) */
	for (i = 0; i <= vec32_num; i+=2)
	  {
	    *(ptr32 + (i / 2)) =
	      ((int32_t) *(ptr_a16 + (i + 1)) * *(ptr_b16 + i))
	       - ((int32_t) *(ptr_a16 + i) * *(ptr_b16 + (i + 1)));
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SMDRS:
      {
	/* Rt = (Ra[15:0] * Rb[15:0]) - (Ra[31:16] * Rb[31:16]) */
	for (i = 0; i <= vec32_num; i+=2)
	  {
	    *(ptr32 + (i / 2)) =
	       ((int32_t) *(ptr_a16 + i) * *(ptr_b16 + i))
		- ((int32_t) *(ptr_a16 + (i + 1)) * *(ptr_b16 + (i + 1)));
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KMADRS:
      {
	int64_t res;
	for (i = 0; i <= vec32_num; i+=2)
	  {
	    res = (int64_t) *(ptr_d32 + (i / 2))
		  - ((int32_t) *(ptr_a16 + (i + 1)) * *(ptr_b16 + (i + 1)))
		  + ((int32_t) *(ptr_a16 + i) * *(ptr_b16 + i));
	    *(ptr32 + (i / 2)) = insn_sat_helper (cpu, res, 31);
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KMADRS32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	int64_t acc = cpu->regs[rd].s;
	int64_t sub_val =
	  (((int64_t) cpu->regs[ra].b32.i0 * cpu->regs[rb].b32.i0))
	  - ((int64_t) cpu->regs[ra].b32.i1 * cpu->regs[rb].b32.i1);
	int64_t res = acc + sub_val;
	if ((acc > 0) && (sub_val > 0))
	  {
	    if (res <= 0)
	      {
		res = 0x7fffffffffffffffLL;
		CCPU_UCODE_OV_SET();
	      }
	  }
	else if ((acc < 0) && (sub_val < 0))
	  {
	    if (res >= 0)
	      {
		res = 0x8000000000000000LL;
		CCPU_UCODE_OV_SET();
	      }
	  }

	cpu->regs[rd].s = res;
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_KMADS:
      {
	int64_t res;
	for (i = 0; i <= vec32_num; i+=2)
	  {
	    res = (int64_t) *(ptr_d32 + (i / 2))
		  + ((int32_t) *(ptr_a16 + (i + 1)) * *(ptr_b16 + (i + 1)))
		  - ((int32_t) *(ptr_a16 + i) * *(ptr_b16 + i));
	    *(ptr32 + (i / 2)) = insn_sat_helper (cpu, res, 31);
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KMADS32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	int64_t acc = cpu->regs[rd].s;
	int64_t sub_val =
	  (((int64_t) cpu->regs[ra].b32.i1 * cpu->regs[rb].b32.i1))
	  - ((int64_t) cpu->regs[ra].b32.i0 * cpu->regs[rb].b32.i0);
	int64_t res = acc + sub_val;
	if ((acc > 0) && (sub_val > 0))
	  {
	    if (res <= 0)
	      {
		res = 0x7fffffffffffffffLL;
		CCPU_UCODE_OV_SET();
	      }
	  }
	else if ((acc < 0) && (sub_val < 0))
	  {
	    if (res >= 0)
	      {
		res = 0x8000000000000000LL;
		CCPU_UCODE_OV_SET();
	      }
	  }

	cpu->regs[rd].s = res;
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_KMAXDS:
      {
	int64_t res;
	for (i = 0; i <= vec32_num; i+=2)
	  {
	    res = (int64_t) *(ptr_d32 + (i / 2))
		  + ((int32_t) *(ptr_a16 + (i + 1)) * *(ptr_b16 + i))
		  - ((int32_t) *(ptr_a16 + i) * *(ptr_b16 + (i + 1)));
	    *(ptr32 + (i / 2)) = insn_sat_helper (cpu, res, 31);
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KMAXDS32:
      {
#if (WITH_TARGET_WORD_BITSIZE == 64)
	int64_t acc = cpu->regs[rd].s;
	int64_t sub_val =
	  (((int64_t) cpu->regs[ra].b32.i1 * cpu->regs[rb].b32.i0))
	  - ((int64_t) cpu->regs[ra].b32.i0 * cpu->regs[rb].b32.i1);
	int64_t res = acc + sub_val;
	if ((acc > 0) && (sub_val > 0))
	  {
	    if (res <= 0)
	      {
		res = 0x7fffffffffffffffLL;
		CCPU_UCODE_OV_SET();
	      }
	  }
	else if ((acc < 0) && (sub_val < 0))
	  {
	    if (res >= 0)
	      {
		res = 0x8000000000000000LL;
		CCPU_UCODE_OV_SET();
	      }
	  }

	cpu->regs[rd].s = res;
	TRACE_REG (cpu, rd);
#else
	TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      }
      break;
    case MATCH_SMAL:
      {
	/* Rt[63:0] = Ra[63:0] + Rb[31:16] * Rb[15:0] */
	int64_t mul_lo, mul_hi = 0;
	int64_t result;

#if (WITH_TARGET_WORD_BITSIZE == 64)
	mul_hi = (int64_t) cpu->regs[rb].b16.h3 * cpu->regs[rb].b16.h2;
#endif
	mul_lo = (int64_t) cpu->regs[rb].b16.h1 * cpu->regs[rb].b16.h0;

	if (RISCV_XLEN (cpu) == 32)
	  {
	    result = get_double (cpu, ra) + mul_lo + mul_hi;
	    set_double (cpu, rd, result);
	  }
	else
	  {
	    cpu->regs[rd].s = cpu->regs[ra].s + mul_lo + mul_hi;
	  }

	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_BITREV:
      {
	if (RISCV_XLEN (cpu) == 32)
	  {
	    uint32_t bits = cpu->regs[ra].u;
	    bits = (((bits & 0xaaaaaaaa) >> 1) | ((bits & 0x55555555) << 1));
	    bits = (((bits & 0xcccccccc) >> 2) | ((bits & 0x33333333) << 2));
	    bits = (((bits & 0xf0f0f0f0) >> 4) | ((bits & 0x0f0f0f0f) << 4));
	    bits = (((bits & 0xff00ff00) >> 8) | ((bits & 0x00ff00ff) << 8));
	    bits = ((bits >> 16) | (bits << 16));
	    cpu->regs[rd].u = bits >> (32 - (cpu->regs[rb].u + 1));
	  }
	else
	  {
	    uint64_t bits = cpu->regs[ra].u;
	    uint64_t r = 0;
	    int i;
	    for (i = 63; i >= 0; i--)
	      r |= ((bits >> i) & 0x1L) << (63 - i);

	    cpu->regs[rd].u = r >> (64 - (cpu->regs[rb].u + 1));
	  }
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_BITREVI:
      {
	if (RISCV_XLEN (cpu) == 32)
	  {
	    uint32_t bits = cpu->regs[ra].u;
	    bits = (((bits & 0xaaaaaaaa) >> 1) | ((bits & 0x55555555) << 1));
	    bits = (((bits & 0xcccccccc) >> 2) | ((bits & 0x33333333) << 2));
	    bits = (((bits & 0xf0f0f0f0) >> 4) | ((bits & 0x0f0f0f0f) << 4));
	    bits = (((bits & 0xff00ff00) >> 8) | ((bits & 0x00ff00ff) << 8));
	    bits = ((bits >> 16) | (bits << 16));
	    cpu->regs[rd].u = bits >> (32 - (imm5u + 1));
	  }
	else
	  {
	    uint64_t bits = cpu->regs[ra].u;
	    uint64_t r = 0;
	    int i;
	    for (i = 63; i >= 0; i--)
	      r |= ((bits >> i) & 0x1L) << (63 - i);

	    cpu->regs[rd].u = r >> (64 - (imm6u + 1));
	  }
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_WEXT:
      if (RISCV_XLEN (cpu) == 32)
	cpu->regs[rd].s = (int32_t) (get_udouble (cpu, ra)
				     >> (cpu->regs[rb].u & 0x1f));
      else
	cpu->regs[rd].s = (int32_t) (cpu->regs[ra].u
				     >> (cpu->regs[rb].u & 0x1f));
      TRACE_REG (cpu, rd);
      break;
    case MATCH_WEXTI:
      if (RISCV_XLEN (cpu) == 32)
	cpu->regs[rd].s = (int32_t) (get_udouble (cpu, ra) >> imm5u);
      else
	cpu->regs[rd].s = (int32_t) (cpu->regs[ra].u >> imm5u);
      TRACE_REG (cpu, rd);
      break;
    case MATCH_BPICK:
      {
	if (RISCV_XLEN (cpu) == 32)
	  {
	    uint32_t temp_ctl = ~cpu->regs[rc].u;
	    cpu->regs[rd].u = (uint32_t) ((cpu->regs[ra].u & ~temp_ctl)
					  | (cpu->regs[rb].u & temp_ctl));
	  }
	else
	  {
	    uint64_t temp_ctl = ~cpu->regs[rc].u;
	    cpu->regs[rd].u = (uint64_t) ((cpu->regs[ra].u & ~temp_ctl)
					  | (cpu->regs[rb].u & temp_ctl));
	  }
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_INSB:
      {
	if (RISCV_XLEN (cpu) == 32)
	  {
	    int temp = cpu->regs[ra].s;

	    /* default is byte 0 */
	    int32_t mask = 0xFFFFFF00;

	    if (imm2u == 1)
	      {
		mask = 0xFFFF00FF;
		temp <<= 8;
	      }
	    else if (imm2u == 2)
	      {
		mask = 0xFF00FFFF;
		temp <<= 16;
	      }
	    else if (imm2u == 3)
	      {
		mask = 0x00FFFFFF;
		temp <<= 24;
	      }
	    cpu->regs[rd].s = (cpu->regs[rd].s & mask) | (temp & (~mask));
	  }
	else
	  {
	    switch (imm3u)
	      {
	      case 0:
		cpu->regs[rd].b8.b0 = cpu->regs[ra].b8.b0;
		break;
	      case 1:
		cpu->regs[rd].b8.b1 = cpu->regs[ra].b8.b0;
		break;
	      case 2:
		cpu->regs[rd].b8.b2 = cpu->regs[ra].b8.b0;
		break;
	      case 3:
		cpu->regs[rd].b8.b3 = cpu->regs[ra].b8.b0;
		break;
#if (WITH_TARGET_WORD_BITSIZE == 64)
	      case 4:
		cpu->regs[rd].b8.b4 = cpu->regs[ra].b8.b0;
		break;
	      case 5:
		cpu->regs[rd].b8.b5 = cpu->regs[ra].b8.b0;
		break;
	      case 6:
		cpu->regs[rd].b8.b6 = cpu->regs[ra].b8.b0;
		break;
	      case 7:
		cpu->regs[rd].b8.b7 = cpu->regs[ra].b8.b0;
		break;
#endif
	      default:
		TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
		sim_engine_halt (sd, cpu, NULL, cpu->pc,
				 sim_signalled, SIM_SIGILL);
	      }
	  }
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_ADD64:
      {
	if (RISCV_XLEN (cpu) == 64)
	  cpu->regs[rd].s = cpu->regs[ra].s + cpu->regs[rb].s;
	else
	  {
	    int64_t result = get_double (cpu, ra) + get_double (cpu, rb);
	    set_double (cpu, rd, result);
	  }
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_RADD64:
      {
	/* 64 = (64 + 64) >> 1 */
	int64_t result;
	int64_t lsb_eq_1 = 1L;
	if (RISCV_XLEN (cpu) == 64)
	  {
	    lsb_eq_1 &= cpu->regs[ra].s;
	    lsb_eq_1 &= cpu->regs[rb].s;
	    cpu->regs[rd].s = (cpu->regs[ra].s >> 1)
			      + (cpu->regs[rb].s >> 1) + lsb_eq_1;
	  }
	else
	  {
	    lsb_eq_1 &= (int64_t) cpu->regs[ra].s;
	    lsb_eq_1 &= (int64_t) cpu->regs[rb].s;
	    result = (get_double (cpu, ra) >> 1)
		     + (get_double (cpu, rb) >> 1)
		     + lsb_eq_1;
	    set_double (cpu, rd, result);
	  }
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_URADD64:
      {
	/* 64 = (U64 + U64) >> 1 */
	uint64_t result;
	uint64_t lsb_eq_1 = 1UL;
	if (RISCV_XLEN (cpu) == 64)
	  {
	    lsb_eq_1 &= cpu->regs[ra].u;
	    lsb_eq_1 &= cpu->regs[rb].u;
	    cpu->regs[rd].u = (cpu->regs[ra].u >> 1)
			      + (cpu->regs[rb].u >> 1) + lsb_eq_1;
	  }
	else
	  {
	    lsb_eq_1 &= (uint64_t) cpu->regs[ra].u;
	    lsb_eq_1 &= (uint64_t) cpu->regs[rb].u;
	    result = (get_udouble (cpu, ra) >> 1)
		     + (get_udouble (cpu, rb) >> 1)
		     + lsb_eq_1;
	    set_udouble (cpu, rd, result);
	  }
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KADD64:
      {
	int64_t x, y, res;
	if (RISCV_XLEN (cpu) == 64)
	  {
	    x = cpu->regs[ra].s;
	    y = cpu->regs[rb].s;
	  }
	else
	  {
	    x = get_double (cpu, ra);
	    y = get_double (cpu, rb);
	  }

	res = x + y;
	if ((x > 0) && (y > 0))
	  {
	    if (res <= 0)
	      {
		res = 0x7fffffffffffffffLL;
	        CCPU_UCODE_OV_SET();
	      }
	  }
	else if ((x < 0) && (y < 0))
	  {
	    if (res >= 0)
	      {
		res = 0x8000000000000000LL;
	        CCPU_UCODE_OV_SET();
	      }
	  }

	if (RISCV_XLEN (cpu) == 64)
	  cpu->regs[rd].s = res;
	else
	  set_double (cpu, rd, res);
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UKADD64:
      {
	uint64_t x, y, res;
	if (RISCV_XLEN (cpu) == 64)
	  {
	    x = cpu->regs[ra].u;
	    y = cpu->regs[rb].u;
	  }
	else
	  {
	    x = get_udouble (cpu, ra);
	    y = get_udouble (cpu, rb);
	  }

	res = x + y;
	if (res < x)
	  {
	    res = 0xffffffffffffffffULL;
	    CCPU_UCODE_OV_SET();
	  }

	if (RISCV_XLEN (cpu) == 64)
	  cpu->regs[rd].u = res;
	else
	  set_udouble (cpu, rd, res);
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SUB64:
      {
	if (RISCV_XLEN (cpu) == 64)
	  cpu->regs[rd].s = cpu->regs[ra].s - cpu->regs[rb].s;
	else
	  {
	    int64_t result = get_double (cpu, ra) - get_double (cpu, rb);
	    set_double (cpu, rd, result);
	  }
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_RSUB64:
      {
	/* 64 = (64 - 64) >> 1 */
	int64_t result;
	int64_t lsb_ra, lsb_rb, signed_ra, signed_rb, sum_lsb;

	lsb_ra = cpu->regs[ra].s & 0x1;
	lsb_rb = cpu->regs[rb].s & 0x1;

	if (RISCV_XLEN (cpu) == 64)
	  {
	    signed_ra = (cpu->regs[ra].s >> 31) & 0x1;
	    signed_rb = (cpu->regs[rb].s >> 31) & 0x1;
	  }
	else
	  {
	    signed_ra = (cpu->regs[ra + 1].s >> 31) & 0x1;
	    signed_rb = (cpu->regs[rb + 1].s >> 31) & 0x1;
	  }
	if (lsb_ra == 1 && signed_ra)
	  lsb_ra = -1L;
	if (lsb_rb == 1 && signed_rb)
	  lsb_rb = -1L;

	if (lsb_ra == -1L && lsb_rb == 0)
	  sum_lsb = 0;
	else
	  sum_lsb = (lsb_ra - lsb_rb) >> 1;

	if (RISCV_XLEN (cpu) == 64)
	  cpu->regs[rd].s = (cpu->regs[ra].s >> 1)
			    - (cpu->regs[rb].s >> 1) + sum_lsb;
	else
	  {
	    result = (get_double (cpu, ra) >> 1)
		     - (get_double (cpu, rb) >> 1) + sum_lsb;
	    set_double (cpu, rd, result);
	  }
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_URSUB64:
      {
	/* 64 = (U64 - U64) >> 1 */
	uint64_t result;
	uint64_t lsb_ra, lsb_rb, sum_lsb;
	lsb_ra = cpu->regs[ra].u & 0x1;
	lsb_rb = cpu->regs[rb].u & 0x1;

	sum_lsb = (lsb_ra < lsb_rb) ? -1ULL : 0;

	if (RISCV_XLEN (cpu) == 64)
	  cpu->regs[rd].u = (cpu->regs[ra].u >> 1)
			    - (cpu->regs[rb].u >> 1) + sum_lsb;
	else
	  {
	    result = (get_udouble (cpu, ra) >> 1)
		     - (get_udouble (cpu, rb) >> 1) + sum_lsb;
	    set_udouble (cpu, rd, result);
	  }
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KSUB64:
      {
	int64_t x, y, res;
	if (RISCV_XLEN (cpu) == 64)
	  {
	    x = cpu->regs[ra].s;
	    y = cpu->regs[rb].s;
	  }
	else
	  {
	    x = get_double (cpu, ra);
	    y = get_double (cpu, rb);
	  }

	res = x - y;
	if ((x > 0) && (y < 0))
	  {
	    if (res <= 0)
	      {
		res = 0x7fffffffffffffffLL;
	        CCPU_UCODE_OV_SET();
	      }
	  }
	else if ((x < 0) && (y > 0))
	  {
	    if (res >= 0)
	      {
		res = 0x8000000000000000LL;
	        CCPU_UCODE_OV_SET();
	      }
	  }
	if (RISCV_XLEN (cpu) == 64)
	  cpu->regs[rd].s = res;
	else
	  set_double (cpu, rd, res);
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UKSUB64:
      {
	int64_t x, y, res;
	if (RISCV_XLEN (cpu) == 64)
	  {
	    x = cpu->regs[ra].s;
	    y = cpu->regs[rb].s;
	  }
	else
	  {
	    x = get_udouble (cpu, ra);
	    y = get_udouble (cpu, rb);
	  }

	res = x - y;
	if (x < y)
	  {
	    res = 0ULL;
	    CCPU_UCODE_OV_SET();
	  }

	if (RISCV_XLEN (cpu) == 64)
	  cpu->regs[rd].u = res;
	else
	  set_udouble (cpu, rd, res);
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SMAR64:
      {
#if (WITH_TARGET_WORD_BITSIZE == 32)
	int64_t result = get_double (cpu, rd)
			 + ((int64_t) cpu->regs[ra].s
			    * (int64_t) cpu->regs[rb].s);
	set_double (cpu, rd, result);
#else
	cpu->regs[rd].s = cpu->regs[rd].s
			  + ((int64_t) cpu->regs[ra].b32.i0
			     * (int64_t)cpu->regs[rb].b32.i0)
			  + ((int64_t) cpu->regs[ra].b32.i1
			     * (int64_t)cpu->regs[rb].b32.i1);
#endif
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UMAR64:
      {
#if (WITH_TARGET_WORD_BITSIZE == 32)
	uint64_t result = get_udouble (cpu, rd)
			  + ((uint64_t)cpu->regs[ra].u
			     * (uint64_t)cpu->regs[rb].u);
	set_udouble (cpu, rd, result);
#else
	cpu->regs[rd].u = cpu->regs[rd].u
			  + ((int64_t) cpu->regs[ra].ub32.i0
			     * (int64_t)cpu->regs[rb].ub32.i0)
			  + ((int64_t) cpu->regs[ra].ub32.i1
			     * (int64_t)cpu->regs[rb].ub32.i1);
#endif
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SMSR64:
      {
#if (WITH_TARGET_WORD_BITSIZE == 32)
	int64_t result = get_double (cpu, rd)
			 - ((int64_t)cpu->regs[ra].s
			    * (int64_t)cpu->regs[rb].s);
	set_double (cpu, rd, result);
#else
	cpu->regs[rd].s = cpu->regs[rd].s
			  - ((int64_t) cpu->regs[ra].b32.i0
			     * (int64_t)cpu->regs[rb].b32.i0)
			  - ((int64_t) cpu->regs[ra].b32.i1
			     * (int64_t)cpu->regs[rb].b32.i1);
#endif
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UMSR64:
      {
#if (WITH_TARGET_WORD_BITSIZE == 32)
	uint64_t result = get_udouble (cpu, rd)
			  - ((uint64_t)cpu->regs[ra].u
			     * (uint64_t)cpu->regs[rb].u);
	set_udouble (cpu, rd, result);
#else
	cpu->regs[rd].u = cpu->regs[rd].u
			  - ((int64_t) cpu->regs[ra].ub32.i0
			     * (int64_t)cpu->regs[rb].ub32.i0)
			  - ((int64_t) cpu->regs[ra].ub32.i1
			     * (int64_t)cpu->regs[rb].ub32.i1);
#endif
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KMAR64:
      {
	int64_t acc, mul_val, res;

#if (WITH_TARGET_WORD_BITSIZE == 32)
	acc = get_double (cpu, rd);
	mul_val = (int64_t) cpu->regs[ra].s * cpu->regs[rb].s;
#else
	acc = cpu->regs[rd].s;
	mul_val = ((int64_t) cpu->regs[ra].b32.i0 * cpu->regs[rb].b32.i0)
		  + ((int64_t) cpu->regs[ra].b32.i1 * cpu->regs[rb].b32.i1);
#endif
	res = acc + mul_val;
	if ((acc > 0) && (mul_val > 0))
	  {
	    if (res <= 0)
	      {
		res = 0x7fffffffffffffffLL;
		CCPU_UCODE_OV_SET();
	      }
	  }
	else if ((acc < 0) && (mul_val < 0))
	  {
	    if (res >= 0)
	      {
		res = 0x8000000000000000LL;
		CCPU_UCODE_OV_SET();
	      }
	  }

	if (RISCV_XLEN (cpu) == 32)
	  set_double (cpu, rd, res);
	else
	  cpu->regs[rd].s = res;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UKMAR64:
      {
	uint64_t acc, mul_val, res;

#if (WITH_TARGET_WORD_BITSIZE == 32)
	acc = get_udouble (cpu, rd);
	mul_val = (uint64_t) cpu->regs[ra].u * cpu->regs[rb].u;
#else
	acc = cpu->regs[rd].u;
	mul_val =
	  ((uint64_t) cpu->regs[ra].ub32.i0 * cpu->regs[rb].ub32.i0)
	  + ((uint64_t) cpu->regs[ra].ub32.i1 * cpu->regs[rb].ub32.i1);
#endif
	res = acc + mul_val;
	if (res < acc)
	  {
	    res = 0xffffffffffffffffULL;
	    CCPU_UCODE_OV_SET();
	  }

	if (RISCV_XLEN (cpu) == 32)
	  set_udouble (cpu, rd, res);
	else
	  cpu->regs[rd].u = res;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KMSR64:
      {
	int64_t acc, mul_val, res;

#if (WITH_TARGET_WORD_BITSIZE == 32)
	acc = get_double (cpu, rd);
	mul_val = (int64_t) cpu->regs[ra].s * cpu->regs[rb].s;
#else
	acc = cpu->regs[rd].s;
	mul_val = ((int64_t) cpu->regs[ra].b32.i0 * cpu->regs[rb].b32.i0)
		  + ((int64_t) cpu->regs[ra].b32.i1 * cpu->regs[rb].b32.i1);
#endif
	res = acc - mul_val;

	if ((acc > 0) && (mul_val < 0))
	  {
	    if (res <= 0)
	      {
		res = 0x7fffffffffffffffLL;
		CCPU_UCODE_OV_SET();
	      }
	  }
	else if ((acc < 0) && (mul_val > 0))
	  {
	    if (res >= 0)
	      {
		res = 0x8000000000000000LL;
		CCPU_UCODE_OV_SET();
	      }
	  }
	if (RISCV_XLEN (cpu) == 32)
	  set_double (cpu, rd, res);
	else
	  cpu->regs[rd].s = res;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UKMSR64:
      {
	uint64_t acc, mul_val, res;

#if (WITH_TARGET_WORD_BITSIZE == 32)
	acc = get_udouble (cpu, rd);
	mul_val = (uint64_t) cpu->regs[ra].s * cpu->regs[rb].s;
#else
	acc = cpu->regs[rd].u;
	mul_val =
	  ((uint64_t) cpu->regs[ra].ub32.i0 * cpu->regs[rb].ub32.i0)
	  + ((uint64_t) cpu->regs[ra].ub32.i1 * cpu->regs[rb].ub32.i1);
#endif
	res = acc - mul_val;

	if (acc < mul_val)
	  {
	    res = 0ULL;
	    CCPU_UCODE_OV_SET();
	  }

	if (RISCV_XLEN (cpu) == 32)
	  set_udouble (cpu, rd, res);
	else
	  cpu->regs[rd].u = res;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SMALDA:
      {
	/* 64 = 64 + (Ra[31:16] * Rb[31:16]) + (Ra[15:0] * Rb[15:0]) */
#if (WITH_TARGET_WORD_BITSIZE == 32)
	int64_t mul_h1 = cpu->regs[ra].b16.h1 * cpu->regs[rb].b16.h1;
	int64_t mul_h0 = cpu->regs[ra].b16.h0 * cpu->regs[rb].b16.h0;
	int64_t result = get_double (cpu, rd) + mul_h1 + mul_h0;
	set_double (cpu, rd, result);
#else
	cpu->regs[rd].s = cpu->regs[rd].s
			  + ((int64_t) cpu->regs[ra].b16.h0
			     * (int64_t)cpu->regs[rb].b16.h0)
			  + ((int64_t) cpu->regs[ra].b16.h1
			     * (int64_t)cpu->regs[rb].b16.h1)
			  + ((int64_t) cpu->regs[ra].b16.h2
			     * (int64_t)cpu->regs[rb].b16.h2)
			  + ((int64_t) cpu->regs[ra].b16.h3
			     * (int64_t)cpu->regs[rb].b16.h3);
#endif
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SMSLDA:
      {
	/* 64 = 64 - (Ra[31:16] * Rb[31:16]) + (Ra[15:0] * Rb[15:0]) */
#if (WITH_TARGET_WORD_BITSIZE == 32)
	int64_t mul_h1 = cpu->regs[ra].b16.h1 * cpu->regs[rb].b16.h1;
	int64_t mul_h0 = cpu->regs[ra].b16.h0 * cpu->regs[rb].b16.h0;
	int64_t result = get_double (cpu, rd) - mul_h1 - mul_h0;
	set_double (cpu, rd, result);
#else
	cpu->regs[rd].s = cpu->regs[rd].s
			  - ((int64_t) cpu->regs[ra].b16.h0
			     * (int64_t)cpu->regs[rb].b16.h0)
			  - ((int64_t) cpu->regs[ra].b16.h1
			     * (int64_t)cpu->regs[rb].b16.h1)
			  - ((int64_t) cpu->regs[ra].b16.h2
			     * (int64_t)cpu->regs[rb].b16.h2)
			  - ((int64_t) cpu->regs[ra].b16.h3
			     * (int64_t)cpu->regs[rb].b16.h3);
#endif
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SMALDS:
      {
	/* 64 = 64 + ((Ra[31:16] * Rb[31:16]) - (Ra[15:0] * Rb[15:0])) */
#if (WITH_TARGET_WORD_BITSIZE == 32)
	int64_t mul_h1 = cpu->regs[ra].b16.h1 * cpu->regs[rb].b16.h1;
	int64_t mul_h0 = cpu->regs[ra].b16.h0 * cpu->regs[rb].b16.h0;
	int64_t result = get_double (cpu, rd) + mul_h1 - mul_h0;
	set_double (cpu, rd, result);
#else
	cpu->regs[rd].s = cpu->regs[rd].s
			  + ((int64_t) cpu->regs[ra].b16.h1
			     * (int64_t)cpu->regs[rb].b16.h1)
			  - ((int64_t) cpu->regs[ra].b16.h0
			     * (int64_t)cpu->regs[rb].b16.h0)
			  + ((int64_t) cpu->regs[ra].b16.h3
			     * (int64_t)cpu->regs[rb].b16.h3)
			  - ((int64_t) cpu->regs[ra].b16.h2
			     * (int64_t)cpu->regs[rb].b16.h2);
#endif
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SMALBB:
      {
	/* 64 = 64 + Ra[15:0] * Rb[15:0] */
	int64_t mul_h0, res;

#if (WITH_TARGET_WORD_BITSIZE == 32)
	mul_h0 = cpu->regs[ra].b16.h0 * cpu->regs[rb].b16.h0;
	res = get_double (cpu, rd) + mul_h0;
	set_double (cpu, rd, res);
#else
	mul_h0 = cpu->regs[ra].b16.h0 * cpu->regs[rb].b16.h0
		 + cpu->regs[ra].b16.h2 * cpu->regs[rb].b16.h2;
	cpu->regs[rd].s = cpu->regs[rd].s + mul_h0;
#endif
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SMSLXDA:
      {
	/* 64 = 64 - (Ra[31:16] * Rb[15:0]) + (Ra[15:0] * Rb[31:16]) */
#if (WITH_TARGET_WORD_BITSIZE == 32)
	int64_t mul_h1 = cpu->regs[ra].b16.h1 * cpu->regs[rb].b16.h0;
	int64_t mul_h0 = cpu->regs[ra].b16.h0 * cpu->regs[rb].b16.h1;
	int64_t result = get_double (cpu, rd) - mul_h1 - mul_h0;
	set_double (cpu, rd, result);
#else
	cpu->regs[rd].s = cpu->regs[rd].s
			  - ((int64_t) cpu->regs[ra].b16.h1
			     * (int64_t)cpu->regs[rb].b16.h0)
			  - ((int64_t) cpu->regs[ra].b16.h0
			     * (int64_t)cpu->regs[rb].b16.h1)
			  - ((int64_t) cpu->regs[ra].b16.h3
			     * (int64_t)cpu->regs[rb].b16.h2)
			  - ((int64_t) cpu->regs[ra].b16.h2
			     * (int64_t)cpu->regs[rb].b16.h3);
#endif
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SMALXDS:
      {
	/* 64 = 64 + (Ra[31:16] * Rb[15:0]) - (Ra[15:0] * Rb[31:16]) */
#if (WITH_TARGET_WORD_BITSIZE == 32)
	int64_t mul_h1 = cpu->regs[ra].b16.h1 * cpu->regs[rb].b16.h0;
	int64_t mul_h0 = cpu->regs[ra].b16.h0 * cpu->regs[rb].b16.h1;
	int64_t result = get_double (cpu, rd) + mul_h1 - mul_h0;
	set_double (cpu, rd, result);
#else
	cpu->regs[rd].s = cpu->regs[rd].s
			  + ((int64_t) cpu->regs[ra].b16.h1
			     * (int64_t)cpu->regs[rb].b16.h0)
			  - ((int64_t) cpu->regs[ra].b16.h0
			     * (int64_t)cpu->regs[rb].b16.h1)
			  + ((int64_t) cpu->regs[ra].b16.h3
			     * (int64_t)cpu->regs[rb].b16.h2)
			  - ((int64_t) cpu->regs[ra].b16.h2
			     * (int64_t)cpu->regs[rb].b16.h3);
#endif
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SMALBT:
      {
	/* 64 = 64 + Ra[15:0] * Rb[31:16] */
	int64_t mul_h0, res;

#if (WITH_TARGET_WORD_BITSIZE == 32)
	mul_h0 = cpu->regs[ra].b16.h0 * cpu->regs[rb].b16.h1;
	res = get_double (cpu, rd) + mul_h0;
	set_double (cpu, rd, res);
#else
	mul_h0 = cpu->regs[ra].b16.h0 * cpu->regs[rb].b16.h1
		 + cpu->regs[ra].b16.h2 * cpu->regs[rb].b16.h3;
	cpu->regs[rd].s = cpu->regs[rd].s + mul_h0;
#endif
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SMALDRS:
      {
	/* 64 = 64 + (Ra[15:0] * Rb[15:0]) - (Ra[31:16] * Rb[31:16]) */
#if (WITH_TARGET_WORD_BITSIZE == 32)
	int64_t mul_h1 = cpu->regs[ra].b16.h1 * cpu->regs[rb].b16.h1;
	int64_t mul_h0 = cpu->regs[ra].b16.h0 * cpu->regs[rb].b16.h0;
	int64_t result = get_double (cpu, rd) - mul_h1 + mul_h0;
	set_double (cpu, rd, result);
#else
	cpu->regs[rd].s = cpu->regs[rd].s
			  - ((int64_t) cpu->regs[ra].b16.h1
			     * (int64_t)cpu->regs[rb].b16.h1)
			  + ((int64_t) cpu->regs[ra].b16.h0
			     * (int64_t)cpu->regs[rb].b16.h0)
			  - ((int64_t) cpu->regs[ra].b16.h3
			     * (int64_t)cpu->regs[rb].b16.h3)
			  + ((int64_t) cpu->regs[ra].b16.h2
			     * (int64_t)cpu->regs[rb].b16.h2);
#endif
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SMALTT:
      {
	/* 64 = 64 + Ra[31:16] * Rb[31:16] */
	int64_t mul_h0, res;

#if (WITH_TARGET_WORD_BITSIZE == 32)
	mul_h0 = cpu->regs[ra].b16.h1 * cpu->regs[rb].b16.h1;
	res = get_double (cpu, rd) + mul_h0;
	set_double (cpu, rd, res);
#else
	mul_h0 = cpu->regs[ra].b16.h1 * cpu->regs[rb].b16.h1
		 + cpu->regs[ra].b16.h3 * cpu->regs[rb].b16.h3;
	cpu->regs[rd].s = cpu->regs[rd].s + mul_h0;

#endif
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SMALXDA:
      {
	/* 64 = 64 + (Ra[31:16] * Rb[15:0]) + (Ra[15:0] * Rb[31:16]) */
#if (WITH_TARGET_WORD_BITSIZE == 32)
	int64_t mul_h1 = cpu->regs[ra].b16.h1 * cpu->regs[rb].b16.h0;
	int64_t mul_h0 = cpu->regs[ra].b16.h0 * cpu->regs[rb].b16.h1;
	int64_t result = get_double (cpu, rd) + mul_h1 + mul_h0;
	set_double (cpu, rd, result);
#else
	cpu->regs[rd].s = cpu->regs[rd].s
			  + ((int64_t) cpu->regs[ra].b16.h1
			     * (int64_t)cpu->regs[rb].b16.h0)
			  + ((int64_t) cpu->regs[ra].b16.h0
			     * (int64_t)cpu->regs[rb].b16.h1)
			  + ((int64_t) cpu->regs[ra].b16.h3
			     * (int64_t)cpu->regs[rb].b16.h2)
			  + ((int64_t) cpu->regs[ra].b16.h2
			     * (int64_t)cpu->regs[rb].b16.h3);
#endif
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KADDW:
      {
	int64_t tmp = (int64_t) cpu->regs[ra].b32.i0 +
	              (int64_t) cpu->regs[rb].b32.i0;
	cpu->regs[rd].s = EXTEND32 (insn_sat_helper (cpu, tmp, 31));
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KSUBW:
      {
	int64_t tmp = (int64_t) cpu->regs[ra].b32.i0 -
	              (int64_t) cpu->regs[rb].b32.i0;
	cpu->regs[rd].s = EXTEND32 (insn_sat_helper (cpu, tmp, 31));
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UKADDW:
      {
	uint64_t tmp = (uint64_t) cpu->regs[ra].ub32.i0 +
	               (uint64_t) cpu->regs[rb].ub32.i0;
	cpu->regs[rd].s = EXTEND32 (insn_usat_helper (cpu, tmp, 32));
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UKSUBW:
      {
	uint64_t tmp = (uint64_t) cpu->regs[ra].ub32.i0 -
	               (uint64_t) cpu->regs[rb].ub32.i0;
	cpu->regs[rd].s = EXTEND32 (insn_usat_helper (cpu, tmp, 32));
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KSLRAW:
      {
	if (cpu->regs[rb].b8.b0 < 0)
	  {
	    int sh = -cpu->regs[rb].b8.b0;
	    sh = sh > 31 ? 31 : sh;
	    cpu->regs[rd].s = cpu->regs[ra].s >> sh;
	  }
	else
	  {
	    int64_t ret = (int64_t) cpu->regs[ra].s << cpu->regs[rb].b8.b0;
	    cpu->regs[rd].s = insn_sat_helper (cpu, ret, 31);
	  }

	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KDMBB:
      {
	int16_t aop = cpu->regs[ra].b16.h0;
	int16_t bop = cpu->regs[rb].b16.h0;
	int32_t res;

	if (aop == (int16_t) 0x8000 && bop == (int16_t) 0x8000)
	  {
	    res = 0x7fffffff;
	    CCPU_UCODE_OV_SET();
	  }
	else
	  {
	    int32_t mul = (int32_t) aop * bop;
	    res = mul << 1;
	  }

	cpu->regs[rd].s = res;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KDMBB16:
      {
	for (i = 0; i <= vec32_num; i+=2)
	  {
	    if ((*(ptr_a16 + i) == (int16_t) 0x8000)
		&& (*(ptr_b16 + i) == (int16_t) 0x8000))
	      {
		*(ptr32 + (i / 2)) = 0x7fffffff;
		CCPU_UCODE_OV_SET();
	      }
	    else
	      {
		int32_t res;
		res = (int32_t) *(ptr_a16 + i) * *(ptr_b16 + i);
		*(ptr32 + (i / 2)) =  res << 1;
	      }
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KDMBT:
      {
	int16_t aop = cpu->regs[ra].b16.h0;
	int16_t bop = cpu->regs[rb].b16.h1;
	int32_t res;

	if (aop == (int16_t) 0x8000 && bop == (int16_t) 0x8000)
	  {
	    res = 0x7fffffff;
	    CCPU_UCODE_OV_SET();
	  }
	else
	  {
	    int32_t mul = (int32_t) aop * bop;
	    res = mul << 1;
	  }

	cpu->regs[rd].s = res;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KDMBT16:
      {
	for (i = 0; i <= vec32_num; i+=2)
	  {
	    if ((*(ptr_a16 + i) == (int16_t) 0x8000)
		&& (*(ptr_b16 + (i + 1)) == (int16_t) 0x8000))
	      {
		*(ptr32 + (i / 2)) = 0x7fffffff;
		CCPU_UCODE_OV_SET();
	      }
	    else
	      {
		int32_t res;
		res = (int32_t) *(ptr_a16 + i) * *(ptr_b16 + (i + 1));
		*(ptr32 + (i / 2)) =  res << 1;
	      }
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KDMTT:
      {
	int16_t aop = cpu->regs[ra].b16.h1;
	int16_t bop = cpu->regs[rb].b16.h1;
	int32_t res;

	if (aop == (int16_t) 0x8000 && bop == (int16_t) 0x8000)
	  {
	    res = 0x7fffffff;
	    CCPU_UCODE_OV_SET();
	  }
	else
	  {
	    int32_t mul = (int32_t) aop * bop;
	    res = mul << 1;
	  }

	cpu->regs[rd].s = res;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KDMTT16:
      {
	for (i = 0; i <= vec32_num; i+=2)
	  {
	    if ((*(ptr_a16 + (i + 1)) == (int16_t) 0x8000)
		&& (*(ptr_b16 + (i + 1)) == (int16_t) 0x8000))
	      {
		*(ptr32 + (i / 2)) = 0x7fffffff;
		CCPU_UCODE_OV_SET();
	      }
	    else
	      {
		int32_t res;
		res = (int32_t) *(ptr_a16 + (i + 1)) * *(ptr_b16 + (i + 1));
		*(ptr32 + (i / 2)) =  res << 1;
	      }
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KDMABB:
      {
	int16_t aop = cpu->regs[ra].b16.h0;
	int16_t bop = cpu->regs[rb].b16.h0;
	int32_t mul = (int32_t) aop * bop;
	int32_t res = mul << 1;
	int64_t resadd;

	if (mul != (res >> 1))
	  {
	    res = 0x7fffffff;
	    CCPU_UCODE_OV_SET();
	  }

	resadd = res + cpu->regs[rd].b32.i0;
	resadd = insn_sat_helper (cpu, resadd, 31);

	cpu->regs[rd].s = resadd;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KDMABB16:
      {
	for (i = 0; i <= vec32_num; i+=2)
	  {
	    int32_t res;
            int32_t resadd;
	    if ((*(ptr_a16 + i) == (int16_t) 0x8000)
		&& (*(ptr_b16 + i) == (int16_t) 0x8000))
	      {
		res = 0x7fffffff;
		CCPU_UCODE_OV_SET();
	      }
	    else
	      {
		res = (int32_t) *(ptr_a16 + i) * *(ptr_b16 + i);
		res = res << 1;
	      }

	    resadd = res + *(ptr_d32 + (i / 2));
	    resadd = insn_sat_helper (cpu, resadd, 31);
	    *(ptr32 + (i / 2)) = resadd;
	  }

	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KDMABT:
      {
	int16_t aop = cpu->regs[ra].b16.h0;
	int16_t bop = cpu->regs[rb].b16.h1;
	int32_t mul = (int32_t) aop * bop;
	int32_t res = mul << 1;
	int64_t resadd;

	if (mul != (res >> 1))
	  {
	    res = 0x7fffffff;
	    CCPU_UCODE_OV_SET();
	  }

	resadd = res + cpu->regs[rd].b32.i0;
	resadd = insn_sat_helper (cpu, resadd, 31);

	cpu->regs[rd].s = resadd;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KDMABT16:
      {
	for (i = 0; i <= vec32_num; i+=2)
	  {
	    int32_t res;
	    int32_t resadd;
	    if ((*(ptr_a16 + i) == (int16_t) 0x8000)
		&& (*(ptr_b16 + (i + 1)) == (int16_t) 0x8000))
	      {
		res = 0x7fffffff;
		CCPU_UCODE_OV_SET();
	      }
	    else
	      {
		res = (int32_t) *(ptr_a16 + i) * *(ptr_b16 + (i + 1));
		res = res << 1;
	      }

	    resadd = res + *(ptr_d32 + (i / 2));
	    resadd = insn_sat_helper (cpu, resadd, 31);
	    *(ptr32 + (i / 2)) = resadd;
	  }

	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KDMATT:
      {
	int16_t aop = cpu->regs[ra].b16.h1;
	int16_t bop = cpu->regs[rb].b16.h1;
	int32_t mul = (int32_t) aop * bop;
	int32_t res = mul << 1;
	int64_t resadd;

	if (mul != (res >> 1))
	  {
	    res = 0x7fffffff;
	    CCPU_UCODE_OV_SET();
	  }

	resadd = res + cpu->regs[rd].b32.i0;
	resadd = insn_sat_helper (cpu, resadd, 31);

	cpu->regs[rd].s = resadd;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KDMATT16:
      {
	for (i = 0; i <= vec32_num; i+=2)
	  {
	    int32_t res;
	    int32_t resadd;
	    if ((*(ptr_a16 + (i + 1)) == (int16_t) 0x8000)
		&& (*(ptr_b16 + (i + 1)) == (int16_t) 0x8000))
	      {
		res = 0x7fffffff;
		CCPU_UCODE_OV_SET();
	      }
	    else
	      {
		res = (int32_t) *(ptr_a16 + (i + 1)) * *(ptr_b16 + (i + 1));
		res = res << 1;
	      }

	    resadd = res + *(ptr_d32 + (i / 2));
	    resadd = insn_sat_helper (cpu, resadd, 31);
	    *(ptr32 + (i / 2)) = resadd;
	  }

	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KHMBB:
      {
	int16_t aop = cpu->regs[ra].b16.h0;
	int16_t bop = cpu->regs[rb].b16.h0;
	cpu->regs[rd].s = insn_sat_khm_helper (cpu, aop, bop);
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KHMBB16:
      {
	for (i = 0; i <= vec32_num; i+=2)
	  {
	    *(ptr32 + (i / 2))
	      = insn_sat_khm_helper (cpu, *(ptr_a16 + i), *(ptr_b16 + i));
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KHMBT:
      {
	int16_t aop = cpu->regs[ra].b16.h0;
	int16_t bop = cpu->regs[rb].b16.h1;
	cpu->regs[rd].s = insn_sat_khm_helper (cpu, aop, bop);
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KHMBT16:
      {
	for (i = 0; i <= vec32_num; i+=2)
	  {
	    *(ptr32 + (i / 2))
	      = insn_sat_khm_helper (cpu, *(ptr_a16 + i), *(ptr_b16 + (i + 1)));
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KHMTT:
      {
	int16_t aop = cpu->regs[ra].b16.h1;
	int16_t bop = cpu->regs[rb].b16.h1;
	cpu->regs[rd].s = insn_sat_khm_helper (cpu, aop, bop);
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KHMTT16:
      {
	for (i = 0; i <= vec32_num; i+=2)
	  {
	    *(ptr32 + (i / 2))
	      = insn_sat_khm_helper (cpu, *(ptr_a16 + (i + 1)),
				     *(ptr_b16 + (i + 1)));
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SMUL16:
      {
	/* Rt[63:32] = Ra[31:16] * Rb[31:16]
	   Rt[31:0] = Ra[15:0] * Rb[15:0] */
	union64_t temp;
	temp.b32.w0 = (int32_t) *ptr_a16 * *ptr_b16;
	temp.b32.w1 = (int32_t) *(ptr_a16 + 1) * *(ptr_b16 + 1);

	if (RISCV_XLEN (cpu) == 64)
	  cpu->regs[rd].s = temp.d0;
	else
	  set_double (cpu, rd, temp.d0);

	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SMULX16:
      {
	/* Rt[63:32] = Ra[31:16] * Rb[15:0]
	   Rt[31:0] = Ra[15:0] * Rb[31:16] */
	union64_t temp;
	temp.b32.w0 = (int32_t) *ptr_a16 * *(ptr_b16 + 1);
	temp.b32.w1 = (int32_t) *(ptr_a16 + 1) * *ptr_b16;

	if (RISCV_XLEN (cpu) == 64)
	  cpu->regs[rd].s = temp.d0;
	else
	  set_double (cpu, rd, temp.d0);

	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UMUL16:
      {
	/* Rt[63:32] = Ra[31:16] * Rb[31:16]
	   Rt[31:0] = Ra[15:0] * Rb[15:0] */
	union64_t temp;
	temp.ub32.w0 = (uint32_t) *uptr_a16 * *uptr_b16;
	temp.ub32.w1 = (uint32_t) *(uptr_a16 + 1) * *(uptr_b16 + 1);

	if (RISCV_XLEN (cpu) == 64)
	  cpu->regs[rd].s = temp.d0;
	else
	  set_double (cpu, rd, temp.d0);

	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UMULX16:
      {
	/* Rt[63:32] = Ra[31:16] * Rb[15:0]
	   Rt[31:0] = Ra[15:0] * Rb[31:16] */
	union64_t temp;
	temp.ub32.w0 = (uint32_t) *uptr_a16 * *(uptr_b16 + 1);
	temp.ub32.w1 = (uint32_t) *(uptr_a16 + 1) * *uptr_b16;

	if (RISCV_XLEN (cpu) == 64)
	  cpu->regs[rd].s = temp.d0;
	else
	  set_double (cpu, rd, temp.d0);

	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SMUL8:
      {
	/* Rt[63:48] = Ra[31:24] * Rb[31:24]
	 * Rt[47:32] = Ra[23:16] * Rb[23:16]
	 * Rt[31:16] = Ra[15:8] * Rb[15:8]
	   Rt[15:0] = Ra[7:0] * Rb[7:0] */
	union64_t temp;
	temp.b16.h0 = (int16_t) *ptr_a8 * *ptr_b8;
	temp.b16.h1 = (int16_t) *(ptr_a8 + 1) * *(ptr_b8 + 1);
	temp.b16.h2 = (int16_t) *(ptr_a8 + 2) * *(ptr_b8 + 2);
	temp.b16.h3 = (int16_t) *(ptr_a8 + 3) * *(ptr_b8 + 3);

	if (RISCV_XLEN (cpu) == 64)
	  cpu->regs[rd].s = temp.d0;
	else
	  set_double (cpu, rd, temp.d0);

	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SMULX8:
      {
	/* Rt[63:48] = Ra[31:24] * Rb[23:16]
	 * Rt[47:32] = Ra[23:16] * Rb[31:24]
	 * Rt[31:16] = Ra[15:8] * Rb[7:0]
	   Rt[15:0] = Ra[7:0] * Rb[15:8] */
	union64_t temp;
	temp.b16.h0 = (int16_t) *ptr_a8 * *(ptr_b8 + 1);
	temp.b16.h1 = (int16_t) *(ptr_a8 + 1) * *ptr_b8;
	temp.b16.h2 = (int16_t) *(ptr_a8 + 2) * *(ptr_b8 + 3);
	temp.b16.h3 = (int16_t) *(ptr_a8 + 3) * *(ptr_b8 + 2);

	if (RISCV_XLEN (cpu) == 64)
	  cpu->regs[rd].s = temp.d0;
	else
	  set_double (cpu, rd, temp.d0);

	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UMUL8:
      {
	/* Rt[63:48] = Ra[31:24] * Rb[31:24]
	 * Rt[47:32] = Ra[23:16] * Rb[23:16]
	 * Rt[31:16] = Ra[15:8] * Rb[15:8]
	   Rt[15:0] = Ra[7:0] * Rb[7:0] */
	union64_t temp;
	temp.ub16.h0 = (uint16_t) *uptr_a8 * *uptr_b8;
	temp.ub16.h1 = (uint16_t) *(uptr_a8 + 1) * *(uptr_b8 + 1);
	temp.ub16.h2 = (uint16_t) *(uptr_a8 + 2) * *(uptr_b8 + 2);
	temp.ub16.h3 = (uint16_t) *(uptr_a8 + 3) * *(uptr_b8 + 3);

	if (RISCV_XLEN (cpu) == 64)
	  cpu->regs[rd].s = temp.d0;
	else
	  set_double (cpu, rd, temp.d0);

	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UMULX8:
      {
	/* Rt[63:48] = Ra[31:24] * Rb[23:16]
	 * Rt[47:32] = Ra[23:16] * Rb[31:24]
	 * Rt[31:16] = Ra[15:8] * Rb[7:0]
	   Rt[15:0] = Ra[7:0] * Rb[15:8] */
	union64_t temp;
	temp.ub16.h0 = (int16_t) *uptr_a8 * *(uptr_b8 + 1);
	temp.ub16.h1 = (int16_t) *(uptr_a8 + 1) * *uptr_b8;
	temp.ub16.h2 = (int16_t) *(uptr_a8 + 2) * *(uptr_b8 + 3);
	temp.ub16.h3 = (int16_t) *(uptr_a8 + 3) * *(uptr_b8 + 2);

	if (RISCV_XLEN (cpu) == 64)
	  cpu->regs[rd].s = temp.d0;
	else
	  set_double (cpu, rd, temp.d0);

	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KADDH:
      {
	int64_t tmp = (int64_t) cpu->regs[ra].b32.i0 +
	              (int64_t) cpu->regs[rb].b32.i0;
	cpu->regs[rd].s = EXTEND16 (insn_sat_helper (cpu, tmp, 15));
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_KSUBH:
      {
	int64_t tmp = (int64_t) cpu->regs[ra].b32.i0 -
	              (int64_t) cpu->regs[rb].b32.i0;
	cpu->regs[rd].s = EXTEND16 (insn_sat_helper (cpu, tmp, 15));
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UKADDH:
      {
	uint64_t tmp = (uint64_t) cpu->regs[ra].ub32.i0 +
	               (uint64_t) cpu->regs[rb].ub32.i0;
	cpu->regs[rd].s = EXTEND16 (insn_usat_helper (cpu, tmp, 16));
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UKSUBH:
      {
	uint64_t tmp = (uint64_t) cpu->regs[ra].ub32.i0 -
	               (uint64_t) cpu->regs[rb].ub32.i0;
	cpu->regs[rd].s = EXTEND16 (insn_usat_helper (cpu, tmp, 16));
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_AVE:
      {
	int64_t r = ((int64_t) cpu->regs[ra].s)
		    + ((int64_t) cpu->regs[rb].s) + 1;
	cpu->regs[rd].u = (r >> 1);
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_MAXW:
      if (RISCV_XLEN (cpu) == 32)
	cpu->regs[rd].s = (cpu->regs[ra].s > cpu->regs[rb].s)
			   ? cpu->regs[ra].s : cpu->regs[rb].s;
      else
	cpu->regs[rd].s = (cpu->regs[ra].b32.i0 > cpu->regs[rb].b32.i0)
			   ? cpu->regs[ra].b32.i0 : cpu->regs[rb].b32.i0;
      TRACE_REG (cpu, rd);
      break;
    case MATCH_MINW:
      if (RISCV_XLEN (cpu) == 32)
	cpu->regs[rd].s = (cpu->regs[ra].s < cpu->regs[rb].s)
			   ? cpu->regs[ra].s : cpu->regs[rb].s;
      else
	cpu->regs[rd].s = (cpu->regs[ra].b32.i0 < cpu->regs[rb].b32.i0)
			   ? cpu->regs[ra].b32.i0 : cpu->regs[rb].b32.i0;
      TRACE_REG (cpu, rd);
      break;
    case MATCH_MULSR64:
      {
	if (RISCV_XLEN (cpu) == 32)
	  {
	    int64_t r = (int64_t) cpu->regs[ra].s
			 * (int64_t) cpu->regs[rb].s;
	    int d = rd & ~1;
	    cpu->regs[d + 1].u = (r >> 32);
	    cpu->regs[d].u = r;
	    TRACE_REG (cpu, rd);
	    TRACE_REG (cpu, rd + 1);
	  }
	else
	  {
	    cpu->regs[rd].s = (int64_t) cpu->regs[ra].b32.i0
			      * (int64_t) cpu->regs[rb].b32.i0;
	    TRACE_REG (cpu, rd);
	  }
      }
      break;
    case MATCH_MULR64:
      {
#if (WITH_TARGET_WORD_BITSIZE == 32)
	uint64_t r = (uint64_t) cpu->regs[ra].u
		      * (uint64_t) cpu->regs[rb].u;
	int d = rd & ~1;
	cpu->regs[d + 1].u = (r >> 32);
	cpu->regs[d].u = r;
	TRACE_REG (cpu, rd);
	TRACE_REG (cpu, rd + 1);

#else
	cpu->regs[rd].s = (uint64_t) cpu->regs[ra].ub32.i0
			  * (uint64_t) cpu->regs[rb].ub32.i0;
	TRACE_REG (cpu, rd);
#endif
      }
      break;
    case MATCH_PBSAD:
    case MATCH_PBSADA:
      {
	/* The four unsigned 8-bit elements of Ra are subtracted from the four
	   unsigned 8-bit elements of Rb.  */
        int i;
	int res = 0;
	for (i = 0; i < vec8_num; i++)
	  {
	    int diff = *(uptr_a8 + i) - *(uptr_b8 + i);
	    diff = (diff >= 0) ? diff : -diff;
	    res += diff;
	  }

	if (op->match == MATCH_PBSAD)
	  /* pbsad */
	  cpu->regs[rd].u = res;
	else
	  /* pbsada */
	  cpu->regs[rd].u = cpu->regs[rd].u + res;

	TRACE_REG (cpu, rd);
	break;
      }
    case MATCH_MADDR32:
      cpu->regs[rd].u = EXTEND32 (cpu->regs[rd].u + (cpu->regs[ra].u
				                     * cpu->regs[rb].u));
      TRACE_REG (cpu, rd);
      break;
    case MATCH_MSUBR32:
      cpu->regs[rd].u = EXTEND32 (cpu->regs[rd].u - (cpu->regs[ra].u
				                     * cpu->regs[rb].u));
      TRACE_REG (cpu, rd);
      break;
    case MATCH_SWAP8:
      cpu->regs[rd].ub32.i0 = ((cpu->regs[ra].ub32.i0 & 0xFF00FF00) >> 8)
	                      | ((cpu->regs[ra].ub32.i0 & 0x00FF00FF) << 8);
#if (WITH_TARGET_WORD_BITSIZE == 64)
      cpu->regs[rd].ub32.i1 = ((cpu->regs[ra].ub32.i1 & 0xFF00FF00) >> 8)
	                      | ((cpu->regs[ra].ub32.i1 & 0x00FF00FF) << 8);
#endif
      TRACE_REG (cpu, rd);
      break;
    case MATCH_SWAP16:
      cpu->regs[rd].ub32.i0 = ((cpu->regs[ra].ub32.i0 & 0xFFFF0000) >> 16)
	                      | ((cpu->regs[ra].ub32.i0 & 0x0000FFFF) << 16);
#if (WITH_TARGET_WORD_BITSIZE == 64)
      cpu->regs[rd].ub32.i1 = ((cpu->regs[ra].ub32.i1 & 0xFFFF0000) >> 16)
	                      | ((cpu->regs[ra].ub32.i1 & 0x0000FFFF) << 16);
#endif
      TRACE_REG (cpu, rd);
      break;
    case MATCH_SCLIP32:
      {
	int32_t val;
	for (i = 0; i < vec32_num; i++)
	  {
	    val = *(ptr_a32 + i);
	    if (val > ((1 << imm5u) - 1))
	      {
		*(ptr32 + i) = ((1 << imm5u) - 1);
		CCPU_UCODE_OV_SET();
	      }
	    else if (val < -(1 << imm5u))
	      {
		*(ptr32 + i) = -(1 << imm5u);
		CCPU_UCODE_OV_SET();
	      }
	    else
	      *(ptr32 + i) = val;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SCLIP8:
      for (i = 0; i < vec8_num; i++)
	*(ptr8 + i) = insn_sat_helper (cpu, *(ptr_a8 + i), imm3u);

      cpu->regs[rd].s = result.s;
      TRACE_REG (cpu, rd);
      break;
    case MATCH_UCLIP32:
      {
	int32_t val;
	for (i = 0; i < vec32_num; i++)
	  {
	    val = *(ptr_a32 + i);
	    if (val > ((1 << imm5u) - 1))
	      {
		*(ptr32 + i) = ((1 << imm5u) - 1);
		CCPU_UCODE_OV_SET();
	      }
	    else if (val < 0)
	      {
		*(ptr32 + i) = 0;
		CCPU_UCODE_OV_SET();
	      }
	    else
	      *(ptr32 + i) = val;
	  }
	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UCLIP8:
      for (i = 0; i < vec8_num; i++)
	*(ptr8 + i) = insn_usat_helper (cpu, *(ptr_a8 + i), imm3u);

      cpu->regs[rd].s = result.s;
      TRACE_REG (cpu, rd);
      break;
    case MATCH_CLO8:
      {
	int j, cnt = 0;
	for (i = 0; i < vec8_num; i++)
	  {
	    for (j = 7; j >= 0; j--)
	      {
		if (__TEST (*(ptr_a8 + i), j))
		  cnt++;
		else
		  break;
	      }
	    *(ptr8 + i) = cnt;
	    cnt = 0;
	  }

	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_CLO16:
      {
	int j, cnt = 0;
	for (i = 0; i < vec16_num; i++)
	  {
	    for (j = 15; j >= 0; j--)
	      {
		if (__TEST (*(ptr_a16 + i), j))
		  cnt++;
		else
		  break;
	      }
	    *(ptr16 + i) = cnt;
	    cnt = 0;
	  }

	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_CLO32:
      {
	int j, cnt = 0;
	int vec = RISCV_XLEN (cpu) == 64 ? 1 : 0;

	for (i = 0; i <= vec; i++)
	  {
	    for (j = 31; j >= 0; j--)
	      {
		if (__TEST (*(ptr_a32 + i), j))
		  cnt++;
		else
		  break;
	      }
	    *(ptr32 + i) = cnt;
	    cnt = 0;
	  }

	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_CLZ8:
      {
	int j, cnt = 0;
	for (i = 0; i < vec8_num; i++)
	  {
	    for (j = 7; j >= 0; j--)
	      {
		if (__TEST (*(ptr_a8 + i), j) == 0)
		  cnt++;
		else
		  break;
	      }
	    *(ptr8 + i) = cnt;
	    cnt = 0;
	  }

	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_CLZ16:
      {
	int j, cnt = 0;
	for (i = 0; i < vec16_num; i++)
	  {
	    for (j = 15; j >= 0; j--)
	      {
		if (__TEST (*(ptr_a16 + i), j) == 0)
		  cnt++;
		else
		  break;
	      }
	    *(ptr16 + i) = cnt;
	    cnt = 0;
	  }

	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_CLZ32:
      {
	int j, cnt = 0;
	int vec = RISCV_XLEN (cpu) == 64 ? 1 : 0;

	for (i = 0; i <= vec; i++)
	  {
	    for (j = 31; j >= 0; j--)
	      {
		if (__TEST (*(ptr_a32 + i), j) == 0)
		  cnt++;
		else
		  break;
	      }
	    *(ptr32 + i) = cnt;
	    cnt = 0;
	  }

	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_CLRS8:
      {
	int j, cnt = 0;
	for (i = 0; i < vec8_num; i++)
	  {
	    int leading = (*(ptr_a8 + i) >> 7) & 0x1;
	    for (j = 6; j >= 0; j--)
	      {
		if (__TEST (*(ptr_a8 + i), j) == leading)
		  cnt++;
		else
		  break;
	      }
	    *(ptr8 + i) = cnt;
	    cnt = 0;
	  }

	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_CLRS16:
      {
	int j, cnt = 0;
	for (i = 0; i < vec16_num; i++)
	  {
	    int leading = (*(ptr_a16 + i) >> 15) & 0x1;
	    for (j = 14; j >= 0; j--)
	      {
		if (__TEST (*(ptr_a16 + i), j) == leading)
		  cnt++;
		else
		  break;
	      }
	    *(ptr16 + i) = cnt;
	    cnt = 0;
	  }

	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_CLRS32:
      {
	int j, cnt = 0;
	int vec = RISCV_XLEN (cpu) == 64 ? 1 : 0;

	for (i = 0; i <= vec; i++)
	  {
	    int leading = (*(ptr_a32 + i) >> 31) & 0x1;
	    for (j = 30; j >= 0; j--)
	      {
		if (__TEST (*(ptr_a32 + i), j) == leading)
		  cnt++;
		else
		  break;
	      }
	    *(ptr32 + i) = cnt;
	    cnt = 0;
	  }

	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SMAQA:
      {
        result.b32.i0 = cpu->regs[rd].b32.i0 +
	                (int16_t) cpu->regs[ra].b8.b0 * cpu->regs[rb].b8.b0 +
	                (int16_t) cpu->regs[ra].b8.b1 * cpu->regs[rb].b8.b1 +
		        (int16_t) cpu->regs[ra].b8.b2 * cpu->regs[rb].b8.b2 +
		        (int16_t) cpu->regs[ra].b8.b3 * cpu->regs[rb].b8.b3;
#if (WITH_TARGET_WORD_BITSIZE == 64)
        result.b32.i1 = cpu->regs[rd].b32.i1 +
	                (int16_t) cpu->regs[ra].b8.b4 * cpu->regs[rb].b8.b4 +
	                (int16_t) cpu->regs[ra].b8.b5 * cpu->regs[rb].b8.b5 +
		        (int16_t) cpu->regs[ra].b8.b6 * cpu->regs[rb].b8.b6 +
		        (int16_t) cpu->regs[ra].b8.b7 * cpu->regs[rb].b8.b7;
#endif

	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_SMAQA_SU:
      {
        result.b32.i0 = cpu->regs[rd].b32.i0 +
	                (int16_t) cpu->regs[ra].b8.b0 * cpu->regs[rb].ub8.b0 +
	                (int16_t) cpu->regs[ra].b8.b1 * cpu->regs[rb].ub8.b1 +
		        (int16_t) cpu->regs[ra].b8.b2 * cpu->regs[rb].ub8.b2 +
		        (int16_t) cpu->regs[ra].b8.b3 * cpu->regs[rb].ub8.b3;
#if (WITH_TARGET_WORD_BITSIZE == 64)
        result.b32.i1 = cpu->regs[rd].b32.i1 +
	                (int16_t) cpu->regs[ra].b8.b4 * cpu->regs[rb].ub8.b4 +
	                (int16_t) cpu->regs[ra].b8.b5 * cpu->regs[rb].ub8.b5 +
		        (int16_t) cpu->regs[ra].b8.b6 * cpu->regs[rb].ub8.b6 +
		        (int16_t) cpu->regs[ra].b8.b7 * cpu->regs[rb].ub8.b7;
#endif

	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    case MATCH_UMAQA:
      {
        result.ub32.i0 = cpu->regs[rd].ub32.i0 +
	                (uint16_t) cpu->regs[ra].ub8.b0 * cpu->regs[rb].ub8.b0 +
	                (uint16_t) cpu->regs[ra].ub8.b1 * cpu->regs[rb].ub8.b1 +
		        (uint16_t) cpu->regs[ra].ub8.b2 * cpu->regs[rb].ub8.b2 +
		        (uint16_t) cpu->regs[ra].ub8.b3 * cpu->regs[rb].ub8.b3;
#if (WITH_TARGET_WORD_BITSIZE == 64)
        result.ub32.i1 = cpu->regs[rd].ub32.i1 +
	                (uint16_t) cpu->regs[ra].ub8.b4 * cpu->regs[rb].ub8.b4 +
	                (uint16_t) cpu->regs[ra].ub8.b5 * cpu->regs[rb].ub8.b5 +
		        (uint16_t) cpu->regs[ra].ub8.b6 * cpu->regs[rb].ub8.b6 +
		        (uint16_t) cpu->regs[ra].ub8.b7 * cpu->regs[rb].ub8.b7;
#endif

	cpu->regs[rd].s = result.s;
	TRACE_REG (cpu, rd);
      }
      break;
    default:
      TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
      sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
    }
  return pc;
}

/* The RV32 lsmw instruction access register order is
   tp, sp, gp, ra, s0-s11, a0-a7, t0-t6.  */
static const int lsmw_rv_order[] =
{
  0,  4,  2,  3,  1,  8,  9, 18,
  19, 20, 21, 22, 23, 24, 25, 26,
  27, 10, 11, 12, 13, 14, 15, 16,
  17,  5,  6,  7, 28, 29, 31
};

/* The RV32 lsmw instruction access register order is
   tp, sp, gp, ra, s0-s1, a0-a5, t0-t2.  */
static const int lsmw_rve_order[] =
{
  0,  4,  2,  3,  1, 8, 9, 10,
  11, 12, 13, 14, 15, 5, 6, 7
};

ATTRIBUTE_UNUSED static unsigned_word
extract_unsigned_integer (unsigned char *addr, int len)
{
  unsigned int retval;
  const unsigned char *p;
  const unsigned char *startaddr = addr;
  const unsigned char *endaddr = startaddr + len;

  retval = 0;
  for (p = endaddr - 1; p >= startaddr; --p)
    retval = (retval << 8) | *p;

  return retval;
}

ATTRIBUTE_UNUSED static void
store_unsigned_integer (unsigned char *addr, int len, unsigned_word val)
{
  unsigned char *p;
  unsigned char *startaddr = addr;
  unsigned char *endaddr = startaddr + len;

  /* Start at the least significant end of the integer,
     and work towards the most significant.  */
  for (p = startaddr; p < endaddr; ++p)
    {
      *p = val & 0xff;
      val >>= 8;
    }
}

ATTRIBUTE_UNUSED static void
execute_lsmw (SIM_CPU *cpu, unsigned_word iw,
	      const struct riscv_opcode *op, int load_p)
{
  SIM_DESC sd = CPU_STATE (cpu);
  int rd = (iw >> OP_SH_RD) & OP_MASK_RD;
  int rs = (iw >> OP_SH_RS1) & OP_MASK_RS1;
  int re = (iw >> OP_SH_RS2) & OP_MASK_RS2;
  const char *rd_name = riscv_gpr_names_abi[rd];
  const char *rs_name = riscv_gpr_names_abi[rs];
  const char *re_name = riscv_gpr_names_abi[re];
  SIM_ADDR base = cpu->regs[rd].u;
  int i, total_reg, start_reg = -1, end_reg = -1;
  int eh_rve_p = cpu->elf_flags & 0x8;
  int reg_cnt = 0;
  /* dec=-1 or inc=1 */
  int di = (iw & (1 << 30)) ? -1 : 1;
  int ret;
  /* The load/store bytes.  */
  int size = RISCV_XLEN (cpu) / 8;
  unsigned_word val = 0;
  char buf[4];
  int reg_table[NGPR];
  int zero_reg = 0;
  int m_bit  = iw & (1 << 29);
  int d_bit = iw & (1 << 30);
  int a_bit = iw & (1 << 31);
  int stack_align_mode = m_bit && rd == X_SP;
  int stack_align_adj = 0;

  TRACE_EXTRACT (cpu, "rd:%-2i:%-4s  rs:%-2i:%-4s %0*" PRIxTW "  re:%-2i:%-4s %0*" PRIxTW "  match:%#x mask:%#x",
		 rd, rd_name,
		 rs, rs_name, (int)sizeof (unsigned_word) * 2, cpu->regs[rs].u,
		 re, re_name, (int)sizeof (unsigned_word) * 2, cpu->regs[re].u,
		 (unsigned) op->match, (unsigned) op->mask);

  TRACE_INSN (cpu, "%cmw.%c%c%c %s, [%s], %s",
	      load_p ? 'l' : 's',
	      a_bit ? 'a' : 'b',
	      d_bit ? 'd' : 'i',
	      m_bit ? 'm' : ' ',
	      rs_name, rd_name, re_name);
  /* Do the alignment check. */
  if (base & (size - 1))
    {
      fprintf (stderr, "address is not aligned to %d-byte boundary.\n", size);
      sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
      return;
    }

  if ((rs == zero_reg) || (rd == zero_reg))
    {
      fprintf (stderr, "Illegal encoding for smw/lmw"
	       "(Rs or Rd is zero register) instruction.\n");
      sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
      return;
    }

  if (eh_rve_p)
    {
      memcpy (reg_table, lsmw_rve_order, sizeof (lsmw_rve_order));
      total_reg = NGPR - 16;
    }
  else
    {
      memcpy (reg_table, lsmw_rv_order, sizeof (lsmw_rv_order));
      total_reg = NGPR;
    }

  for (i = 0; i < total_reg; ++i)
    {
      if (rs == reg_table[i])
	start_reg = i;
      if (re == reg_table[i])
	end_reg = i;
    }

  if (start_reg > end_reg)
    {
      fprintf (stderr, "Illegal encoding for smw/lmw (Rb > Re) instruction.\n");
      sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
      return;
    }

  /* Sum up the registers count.  */
  reg_cnt = (end_reg - start_reg) + 1;

  if (stack_align_mode)
    {
      if (RISCV_XLEN (cpu) == 32)
	stack_align_adj = ((reg_cnt + 3) & ~3) - reg_cnt;
      else
	stack_align_adj = ((reg_cnt + 1) & ~1) - reg_cnt;
    }
  TRACE_INSN (cpu, "base = %" PRIxTW " stack_align_mode = %d stack_align_adj = %d", cpu->regs[rd].u, stack_align_mode, stack_align_adj);

  /* Generate the first memory address.  */
  if (a_bit)
    base += size * di;
  /* Adjust the first memory address
     due to operating from low address memory.  */
  if (d_bit)
    base -= (reg_cnt - 1 + stack_align_adj) * size;

  for (i = start_reg; i <= end_reg; ++i)
    {
      if (load_p)
	{
	  if (RISCV_XLEN (cpu) == 32)
	    store_rd (cpu, reg_table[i],
	      sim_core_read_unaligned_4 (cpu, cpu->pc, read_map,
					 base));
	  else
	    store_rd (cpu, reg_table[i],
	      sim_core_read_unaligned_8 (cpu, cpu->pc, read_map,
					 base));
	  TRACE_INSN (cpu, "[%" PRIxTW "]<- %s (%" PRIxTW ")", base, riscv_gpr_names_abi[reg_table[i]], cpu->regs[reg_table[i]].u);
	}
      else
	{
	  if (RISCV_XLEN (cpu) == 32)
	    sim_core_write_unaligned_4 (cpu, cpu->pc, write_map,
					base, cpu->regs[reg_table[i]].u);
	  else
	    sim_core_write_unaligned_8 (cpu, cpu->pc, write_map,
					base, cpu->regs[reg_table[i]].u);
	  TRACE_INSN (cpu, "%s(%" PRIxTW ") -> [%" PRIxTW "]", riscv_gpr_names_abi[reg_table[i]], cpu->regs[reg_table[i]].u, base);
	}
      base += size;
    }

  /* Update the base address register.  */
  if (m_bit)
    store_rd (cpu, rd, cpu->regs[rd].u + (reg_cnt + stack_align_adj) * size * di);

  if (!eh_rve_p && rd == X_SP && (cpu->regs[rd].u & 0xf) != 0)
    {
      fprintf (stderr, "Stack pointer is not aligned to 16-byte boundary.\n");
      sim_engine_halt (sd, cpu, NULL, cpu->pc,
		       sim_signalled, SIM_SIGILL);
    }


}

static sim_cia
execute_i (SIM_CPU *cpu, unsigned_word iw, const struct riscv_opcode *op, int ex9)
{
  SIM_DESC sd = CPU_STATE (cpu);
  int rd = (iw >> OP_SH_RD) & OP_MASK_RD;
  int rs1 = (iw >> OP_SH_RS1) & OP_MASK_RS1;
  int rs2 = (iw >> OP_SH_RS2) & OP_MASK_RS2;
  int sv = (iw >> OP_SH_SV) & OP_MASK_SV;
  const char *rd_name = riscv_gpr_names_abi[rd];
  const char *rs1_name = riscv_gpr_names_abi[rs1];
  const char *rs2_name = riscv_gpr_names_abi[rs2];
  unsigned int csr = (iw >> OP_SH_CSR) & OP_MASK_CSR;
  unsigned_word i_imm = EXTRACT_ITYPE_IMM (iw);
  unsigned_word u_imm = EXTRACT_UTYPE_IMM ((unsigned64) iw);
  unsigned_word s_imm = EXTRACT_STYPE_IMM (iw);
  unsigned_word sb_imm = EXTRACT_SBTYPE_IMM (iw);

  unsigned_word shamt_imm = ((iw >> OP_SH_SHAMT) & OP_MASK_SHAMT);
  unsigned_word tmp;
  unsigned_word sys_id;
  host_callback *cb;
  int eh_rve_p = cpu->elf_flags & 0x8;
  sim_cia pc = cpu->pc + 4;
  CB_SYSCALL sc;
  if (ex9)
    pc -= 2;

  cb = STATE_CALLBACK (sd);

  CB_SYSCALL_INIT (&sc);

  if (eh_rve_p)
    sc.func = cpu->t0.u;
  else
    sc.func = cpu->a7.u;

  sc.arg1 = cpu->a0.u;
  sc.arg2 = cpu->a1.u;
  sc.arg3 = cpu->a2.u;
  sc.arg4 = cpu->a3.u;

  sc.p1 = (PTR) sd;
  sc.p2 = (PTR) cpu;
  sc.read_mem = sim_syscall_read_mem;
  sc.write_mem = sim_syscall_write_mem;

  TRACE_EXTRACT (cpu, "rd:%-2i:%-4s  rs1:%-2i:%-4s %0*"PRIxTW"  rs2:%-2i:%-4s %0*"PRIxTW"  match:%#x mask:%#x",
		 rd, rd_name,
		 rs1, rs1_name, (int)sizeof (unsigned_word) * 2, cpu->regs[rs1].u,
		 rs2, rs2_name, (int)sizeof (unsigned_word) * 2, cpu->regs[rs2].u,
		 (unsigned) op->match, (unsigned) op->mask);

  switch (op->match)
    {
    case MATCH_ADD:
      TRACE_INSN (cpu, "add %s, %s, %s;  // %s = %s + %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      store_rd (cpu, rd, cpu->regs[rs1].u + cpu->regs[rs2].u);
      break;
    case MATCH_ADDW:
      TRACE_INSN (cpu, "addw %s, %s, %s;  // %s = %s + %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
      store_rd (cpu, rd, EXTEND32 (cpu->regs[rs1].u + cpu->regs[rs2].u));
      break;
    case MATCH_ADDI:
      if (!eh_rve_p && rd == 2 && ((cpu->regs[rs1].u + i_imm) & 0xf) != 0)
        {
          fprintf (stderr, "Stack pointer is not aligned to 16-byte boundary.\n");
          sim_engine_halt (sd, cpu, NULL, cpu->pc,
      		           sim_signalled, SIM_SIGILL);
        }

      TRACE_INSN (cpu, "addi %s, %s, %#"PRIxTW";  // %s = %s + %#"PRIxTW,
		  rd_name, rs1_name, i_imm, rd_name, rs1_name, i_imm);
      store_rd (cpu, rd, cpu->regs[rs1].u + i_imm);
      break;
    case MATCH_ADDIW:
      TRACE_INSN (cpu, "addiw %s, %s, %#"PRIxTW";  // %s = %s + %#"PRIxTW,
		  rd_name, rs1_name, i_imm, rd_name, rs1_name, i_imm);
      RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
      store_rd (cpu, rd, EXTEND32 (cpu->regs[rs1].u + i_imm));
      break;
    case MATCH_AND:
      TRACE_INSN (cpu, "and %s, %s, %s;  // %s = %s & %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      store_rd (cpu, rd, cpu->regs[rs1].u & cpu->regs[rs2].u);
      break;
    case MATCH_ANDI:
      TRACE_INSN (cpu, "andi %s, %s, %"PRIiTW";  // %s = %s & %#"PRIxTW,
		  rd_name, rs1_name, i_imm, rd_name, rs1_name, i_imm);
      store_rd (cpu, rd, cpu->regs[rs1].u & i_imm);
      break;
    case MATCH_OR:
      TRACE_INSN (cpu, "or %s, %s, %s;  // %s = %s | %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      store_rd (cpu, rd, cpu->regs[rs1].u | cpu->regs[rs2].u);
      break;
    case MATCH_ORI:
      TRACE_INSN (cpu, "ori %s, %s, %"PRIiTW";  // %s = %s | %#"PRIxTW,
		  rd_name, rs1_name, i_imm, rd_name, rs1_name, i_imm);
      store_rd (cpu, rd, cpu->regs[rs1].u | i_imm);
      break;
    case MATCH_XOR:
      TRACE_INSN (cpu, "xor %s, %s, %s;  // %s = %s ^ %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      store_rd (cpu, rd, cpu->regs[rs1].u ^ cpu->regs[rs2].u);
      break;
    case MATCH_XORI:
      TRACE_INSN (cpu, "xori %s, %s, %"PRIiTW";  // %s = %s ^ %#"PRIxTW,
		  rd_name, rs1_name, i_imm, rd_name, rs1_name, i_imm);
      store_rd (cpu, rd, cpu->regs[rs1].u ^ i_imm);
      break;
    case MATCH_SUB:
      TRACE_INSN (cpu, "sub %s, %s, %s;  // %s = %s - %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      store_rd (cpu, rd, cpu->regs[rs1].u - cpu->regs[rs2].u);
      break;
    case MATCH_SUBW:
      TRACE_INSN (cpu, "subw %s, %s, %s;  // %s = %s - %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
      store_rd (cpu, rd, EXTEND32 (cpu->regs[rs1].u - cpu->regs[rs2].u));
      break;
    case MATCH_LUI:
      TRACE_INSN (cpu, "lui %s, %#"PRIxTW";", rd_name, u_imm);
      store_rd (cpu, rd, u_imm);
      break;
    case MATCH_SLL:
      TRACE_INSN (cpu, "sll %s, %s, %s;  // %s = %s << %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      u_imm = RISCV_XLEN (cpu) == 32 ? 0x1f : 0x3f;
      store_rd (cpu, rd, cpu->regs[rs1].u << (cpu->regs[rs2].u & u_imm));
      break;
    case MATCH_SLLW:
      TRACE_INSN (cpu, "sllw %s, %s, %s;  // %s = %s << %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
      store_rd (cpu, rd, EXTEND32 ((unsigned32)cpu->regs[rs1].u << (cpu->regs[rs2].u & 0x1f)));
      break;
    case MATCH_SLLI:
      TRACE_INSN (cpu, "slli %s, %s, %"PRIiTW";  // %s = %s << %#"PRIxTW,
		  rd_name, rs1_name, shamt_imm, rd_name, rs1_name, shamt_imm);
      if (RISCV_XLEN (cpu) == 32 && shamt_imm > 0x1f)
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
      store_rd (cpu, rd, cpu->regs[rs1].u << shamt_imm);
      break;
    case MATCH_SLLIW:
      TRACE_INSN (cpu, "slliw %s, %s, %"PRIiTW";  // %s = %s << %#"PRIxTW,
		  rd_name, rs1_name, shamt_imm, rd_name, rs1_name, shamt_imm);
      RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
      store_rd (cpu, rd, EXTEND32 ((unsigned32)cpu->regs[rs1].u << shamt_imm));
      break;
    case MATCH_SRL:
      TRACE_INSN (cpu, "srl %s, %s, %s;  // %s = %s >> %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      u_imm = RISCV_XLEN (cpu) == 32 ? 0x1f : 0x3f;
      store_rd (cpu, rd, cpu->regs[rs1].u >> (cpu->regs[rs2].u & u_imm));
      break;
    case MATCH_SRLW:
      TRACE_INSN (cpu, "srlw %s, %s, %s;  // %s = %s >> %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
      store_rd (cpu, rd, EXTEND32 ((unsigned32)cpu->regs[rs1].u >> (cpu->regs[rs2].u & 0x1f)));
      break;
    case MATCH_SRLI:
      TRACE_INSN (cpu, "srli %s, %s, %"PRIiTW";  // %s = %s >> %#"PRIxTW,
		  rd_name, rs1_name, shamt_imm, rd_name, rs1_name, shamt_imm);
      if (RISCV_XLEN (cpu) == 32 && shamt_imm > 0x1f)
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
      store_rd (cpu, rd, cpu->regs[rs1].u >> shamt_imm);
      break;
    case MATCH_SRLIW:
      TRACE_INSN (cpu, "srliw %s, %s, %"PRIiTW";  // %s = %s >> %#"PRIxTW,
		  rd_name, rs1_name, shamt_imm, rd_name, rs1_name, shamt_imm);
      RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
      store_rd (cpu, rd, EXTEND32 ((unsigned32)cpu->regs[rs1].u >> shamt_imm));
      break;
    case MATCH_SRA:
      TRACE_INSN (cpu, "sra %s, %s, %s;  // %s = %s >>> %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      if (RISCV_XLEN (cpu) == 32)
	tmp = ashiftrt (cpu->regs[rs1].u, cpu->regs[rs2].u & 0x1f);
      else
	tmp = ashiftrt64 (cpu->regs[rs1].u, cpu->regs[rs2].u & 0x3f);
      store_rd (cpu, rd, tmp);
      break;
    case MATCH_SRAW:
      TRACE_INSN (cpu, "sraw %s, %s, %s;  // %s = %s >>> %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
      store_rd (cpu, rd, EXTEND32 (ashiftrt ((signed32)cpu->regs[rs1].u, cpu->regs[rs2].u & 0x1f)));
      break;
    case MATCH_SRAI:
      TRACE_INSN (cpu, "srai %s, %s, %" PRIiTW ";  // %s = %s >>> %#"PRIxTW,
		  rd_name, rs1_name, shamt_imm, rd_name, rs1_name, shamt_imm);
      if (RISCV_XLEN (cpu) == 32)
	{
	  if (shamt_imm > 0x1f)
	    sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
	  tmp = ashiftrt (cpu->regs[rs1].u, shamt_imm);
	}
      else
	tmp = ashiftrt64 (cpu->regs[rs1].u, shamt_imm);
      store_rd (cpu, rd, tmp);
      break;
    case MATCH_SRAIW:
      TRACE_INSN (cpu, "sraiw %s, %s, %"PRIiTW";  // %s = %s >>> %#"PRIxTW,
		  rd_name, rs1_name, shamt_imm, rd_name, rs1_name, shamt_imm);
      RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
      store_rd (cpu, rd,
		EXTEND32 (ashiftrt ((signed32)cpu->regs[rs1].u, shamt_imm)));
      break;
    case MATCH_SLT:
      TRACE_INSN (cpu, "slt %s %s %s", rd_name, rs1_name, rs2_name);
      store_rd (cpu, rd,
		!!((signed_word)cpu->regs[rs1].u < (signed_word)cpu->regs[rs2].u));
      break;
    case MATCH_SLTU:
      TRACE_INSN (cpu, "sltu %s %s %s", rd_name, rs1_name, rs2_name);
      store_rd (cpu, rd,
		!!((unsigned_word)cpu->regs[rs1].u
		   < (unsigned_word)cpu->regs[rs2].u));
      break;
    case MATCH_SLTI:
      TRACE_INSN (cpu, "slti %s %s %" PRIiTW,
		  rd_name, rs1_name, i_imm);
      store_rd (cpu, rd, !!((signed_word)cpu->regs[rs1].u < (signed_word)i_imm));
      break;
    case MATCH_SLTIU:
      TRACE_INSN (cpu, "sltiu %s %s %" PRIiTW,
		  rd_name, rs1_name, i_imm);
      store_rd (cpu, rd,
		!!((unsigned_word)cpu->regs[rs1].u < (unsigned_word)i_imm));
      break;
    case MATCH_AUIPC:
      TRACE_INSN (cpu, "auipc %s, %"PRIiTW";  // %s = pc + %"PRIiTW,
		  rd_name, u_imm, rd_name, u_imm);
      store_rd (cpu, rd, cpu->pc + u_imm);
      break;
    case MATCH_BEQ:
      TRACE_INSN (cpu, "beq %s, %s, %#"PRIxTW";  // if (%s == %s) goto %#"PRIxTW,
		  rs1_name, rs2_name, sb_imm, rs1_name, rs2_name, cpu->pc + sb_imm);
      if (cpu->regs[rs1].u == cpu->regs[rs2].u)
	{
	  pc = cpu->pc + sb_imm;
	  TRACE_BRANCH (cpu, "to %#"PRIxTW, pc);
	}
      break;
    case MATCH_BLT:
      TRACE_INSN (cpu, "blt %s, %s, %#"PRIxTW";  // if (%s < %s) goto %#"PRIxTW,
		  rs1_name, rs2_name, sb_imm, rs1_name, rs2_name, cpu->pc + sb_imm);
      if ((signed_word)cpu->regs[rs1].u < (signed_word)cpu->regs[rs2].u)
	{
	  pc = cpu->pc + sb_imm;
	  TRACE_BRANCH (cpu, "to %#"PRIxTW, pc);
	}
      break;
    case MATCH_BLTU:
      TRACE_INSN (cpu, "bltu %s, %s, %#"PRIxTW";  // if (%s < %s) goto %#"PRIxTW,
		  rs1_name, rs2_name, sb_imm, rs1_name, rs2_name, cpu->pc + sb_imm);
      if ((unsigned_word)cpu->regs[rs1].u < (unsigned_word)cpu->regs[rs2].u)
	{
	  pc = cpu->pc + sb_imm;
	  TRACE_BRANCH (cpu, "to %#"PRIxTW, pc);
	}
      break;
    case MATCH_BGE:
      TRACE_INSN (cpu, "bge %s, %s, %#"PRIxTW";  // if (%s >= %s) goto %#"PRIxTW,
		  rs1_name, rs2_name, sb_imm, rs1_name, rs2_name, cpu->pc + sb_imm);
      if ((signed_word)cpu->regs[rs1].u >= (signed_word)cpu->regs[rs2].u)
	{
	  pc = cpu->pc + sb_imm;
	  TRACE_BRANCH (cpu, "to %#"PRIxTW, pc);
	}
      break;
    case MATCH_BGEU:
      TRACE_INSN (cpu, "bgeu %s, %s, %#"PRIxTW";  // if (%s >= %s) goto %#"PRIxTW,
		  rs1_name, rs2_name, sb_imm, rs1_name, rs2_name, cpu->pc + sb_imm);
      if ((unsigned_word)cpu->regs[rs1].u >= (unsigned_word)cpu->regs[rs2].u)
	{
	  pc = cpu->pc + sb_imm;
	  TRACE_BRANCH (cpu, "to %#"PRIxTW, pc);
	}
      break;
    case MATCH_BNE:
      TRACE_INSN (cpu, "bne %s, %s, %#"PRIxTW";  // if (%s != %s) goto %#"PRIxTW,
		  rs1_name, rs2_name, sb_imm, rs1_name, rs2_name, cpu->pc + sb_imm);
      if (cpu->regs[rs1].u != cpu->regs[rs2].u)
	{
	  pc = cpu->pc + sb_imm;
	  TRACE_BRANCH (cpu, "to %#"PRIxTW, pc);
	}
      break;
    case MATCH_JAL:
      if (ex9 == 1)
	{
	  // case ex9.it
	  store_rd (cpu, rd, cpu->pc + 2);
	  pc = (cpu->pc & 0xffe00000) | EXTRACT_UJTYPE_IMM_EXECIT_TAB (iw);
	}
      else if (ex9 == 2) 
	{
	  // case ex9.cs
	  TRACE_INSN (cpu, "jal %s, %"PRIiTW";", rd_name, EXTRACT_UJTYPE_IMM (iw));
	  store_rd (cpu, rd, cpu->pc + 2);
	  pc = cpu->pc + EXTRACT_UJTYPE_IMM (iw);
	}
      else
	{
	  TRACE_INSN (cpu, "jal %s, %"PRIiTW";", rd_name, EXTRACT_UJTYPE_IMM (iw));
	  store_rd (cpu, rd, cpu->pc + 4);
	  pc = cpu->pc + EXTRACT_UJTYPE_IMM (iw);
	}
      TRACE_BRANCH (cpu, "to %#"PRIxTW, pc);
      break;
    case MATCH_JALR:
      TRACE_INSN (cpu, "jalr %s, %s, %"PRIiTW";", rd_name, rs1_name, i_imm);
      pc = cpu->regs[rs1].u + i_imm;
      if (ex9)
	store_rd (cpu, rd, cpu->pc + 2);
      else
	store_rd (cpu, rd, cpu->pc + 4);
      TRACE_BRANCH (cpu, "to %#"PRIxTW, pc);
      break;

    case MATCH_LD:
      TRACE_INSN (cpu, "ld %s, %"PRIiTW"(%s); // %s = *(%s + %"PRIiTW")",
		  rd_name, i_imm, rs1_name, rd_name, rs1_name, i_imm);
      RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
      store_rd (cpu, rd,
	sim_core_read_unaligned_8 (cpu, cpu->pc, read_map,
				   cpu->regs[rs1].u + i_imm));
      break;
    case MATCH_LW:
      TRACE_INSN (cpu, "lw %s, %"PRIiTW"(%s); // %s = *(%s + %"PRIiTW")",
		  rd_name, i_imm, rs1_name, rd_name, rs1_name, i_imm);
      store_rd (cpu, rd, EXTEND32 (
	sim_core_read_unaligned_4 (cpu, cpu->pc, read_map,
				   cpu->regs[rs1].u + i_imm)));
      break;
    case MATCH_LWU:
      TRACE_INSN (cpu, "lwu %s, %"PRIiTW"(%s); // %s = *(%s + %"PRIiTW")",
		  rd_name, i_imm, rs1_name, rd_name, rs1_name, i_imm);
      store_rd (cpu, rd,
	sim_core_read_unaligned_4 (cpu, cpu->pc, read_map,
				   cpu->regs[rs1].u + i_imm));
      break;
    case MATCH_LH:
      TRACE_INSN (cpu, "lh %s, %"PRIiTW"(%s); // %s = *(%s + %"PRIiTW")",
		  rd_name, i_imm, rs1_name, rd_name, rs1_name, i_imm);
      store_rd (cpu, rd, EXTEND16 (
	sim_core_read_unaligned_2 (cpu, cpu->pc, read_map,
				   cpu->regs[rs1].u + i_imm)));
      break;
    case MATCH_LHU:
      TRACE_INSN (cpu, "lhu %s, %"PRIiTW"(%s); // %s = *(%s + %"PRIiTW")",
		  rd_name, i_imm, rs1_name, rd_name, rs1_name, i_imm);
      store_rd (cpu, rd,
	sim_core_read_unaligned_2 (cpu, cpu->pc, read_map,
				   cpu->regs[rs1].u + i_imm));
      break;
    case MATCH_LB:
      TRACE_INSN (cpu, "lb %s, %"PRIiTW"(%s); // %s = *(%s + %"PRIiTW")",
		  rd_name, i_imm, rs1_name, rd_name, rs1_name, i_imm);
      store_rd (cpu, rd, EXTEND8 (
	sim_core_read_unaligned_1 (cpu, cpu->pc, read_map,
				   cpu->regs[rs1].u + i_imm)));
      break;
    case MATCH_LBU:
      TRACE_INSN (cpu, "lbu %s, %"PRIiTW"(%s); // %s = *(%s + %"PRIiTW")",
		  rd_name, i_imm, rs1_name, rd_name, rs1_name, i_imm);
      store_rd (cpu, rd,
	sim_core_read_unaligned_1 (cpu, cpu->pc, read_map,
				   cpu->regs[rs1].u + i_imm));
      break;
    case MATCH_SD:
      TRACE_INSN (cpu, "sd %s, %"PRIiTW"(%s); // *(%s + %"PRIiTW") = %s",
		  rs2_name, s_imm, rs1_name, rs1_name, s_imm, rs2_name);
      RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
      sim_core_write_unaligned_8 (cpu, cpu->pc, write_map,
				  cpu->regs[rs1].u + s_imm, cpu->regs[rs2].u);
      break;
    case MATCH_SW:
      TRACE_INSN (cpu, "sw %s, %"PRIiTW"(%s); // *(%s + %"PRIiTW") = %s",
		  rs2_name, s_imm, rs1_name, rs1_name, s_imm, rs2_name);
      sim_core_write_unaligned_4 (cpu, cpu->pc, write_map,
				  cpu->regs[rs1].u + s_imm, cpu->regs[rs2].u);
      break;
    case MATCH_SH:
      TRACE_INSN (cpu, "sh %s, %"PRIiTW"(%s); // *(%s + %"PRIiTW") = %s",
		  rs2_name, s_imm, rs1_name, rs1_name, s_imm, rs2_name);
      sim_core_write_unaligned_2 (cpu, cpu->pc, write_map,
				  cpu->regs[rs1].u + s_imm, cpu->regs[rs2].u);
      break;
    case MATCH_SB:
      TRACE_INSN (cpu, "sb %s, %"PRIiTW"(%s); // *(%s + %"PRIiTW") = %s",
		  rs2_name, s_imm, rs1_name, rs1_name, s_imm, rs2_name);
      sim_core_write_unaligned_1 (cpu, cpu->pc, write_map,
				  cpu->regs[rs1].u + s_imm, cpu->regs[rs2].u);
      break;
    case MATCH_CSRRC:
      TRACE_INSN (cpu, "csrrc");
      switch (csr)
	{
#define DECLARE_CSR(name, num, cls) \
	case num: \
	  store_rd (cpu, rd, fetch_csr (cpu, #name, num, &cpu->csr.name)); \
	  store_csr (cpu, #name, num, &cpu->csr.name, \
		     cpu->csr.name & !cpu->regs[rs1].u); \
	  break;
#include "opcode/riscv-opc.h"
#undef DECLARE_CSR
	}
      break;
    case MATCH_CSRRCI:
      TRACE_INSN (cpu, "csrrci");
      switch (csr)
	{
#define DECLARE_CSR(name, num, cls) \
	case num: \
	  store_rd (cpu, rd, fetch_csr (cpu, #name, num, &cpu->csr.name)); \
	  store_csr (cpu, #name, num, &cpu->csr.name, \
		     cpu->csr.name & !rs1); \
	  break;
#include "opcode/riscv-opc.h"
#undef DECLARE_CSR
	}
      break;
    case MATCH_CSRRS:
      TRACE_INSN (cpu, "csrrs");
      switch (csr)
	{
#define DECLARE_CSR(name, num, cls) \
	case num: \
	  store_rd (cpu, rd, fetch_csr (cpu, #name, num, &cpu->csr.name)); \
	  store_csr (cpu, #name, num, &cpu->csr.name, \
		     cpu->csr.name | cpu->regs[rs1].u); \
	  break;
#include "opcode/riscv-opc.h"
#undef DECLARE_CSR
	}
      break;
    case MATCH_CSRRSI:
      TRACE_INSN (cpu, "csrrsi");
      switch (csr)
	{
#define DECLARE_CSR(name, num, cls) \
	case num: \
	  store_rd (cpu, rd, fetch_csr (cpu, #name, num, &cpu->csr.name)); \
	  store_csr (cpu, #name, num, &cpu->csr.name, \
		     cpu->csr.name | rs1); \
	  break;
#include "opcode/riscv-opc.h"
#undef DECLARE_CSR
	}
      break;
    case MATCH_CSRRW:
      TRACE_INSN (cpu, "csrrw");
      switch (csr)
	{
#define DECLARE_CSR(name, num, cls) \
	case num: \
	  store_rd (cpu, rd, fetch_csr (cpu, #name, num, &cpu->csr.name)); \
	  store_csr (cpu, #name, num, &cpu->csr.name, cpu->regs[rs1].u); \
	  break;
#include "opcode/riscv-opc.h"
#undef DECLARE_CSR
	}
      break;
    case MATCH_CSRRWI:
      TRACE_INSN (cpu, "csrrwi");
      switch (csr)
	{
#define DECLARE_CSR(name, num, cls) \
	case num: \
	  store_rd (cpu, rd, fetch_csr (cpu, #name, num, &cpu->csr.name)); \
	  store_csr (cpu, #name, num, &cpu->csr.name, rs1); \
	  break;
#include "opcode/riscv-opc.h"
#undef DECLARE_CSR
	}
      break;

    case MATCH_RDCYCLE:
      TRACE_INSN (cpu, "rdcycle %s;", rd_name);
      store_rd (cpu, rd, fetch_csr (cpu, "cycle", CSR_CYCLE, &cpu->csr.cycle));
      break;
    case MATCH_RDCYCLEH:
      TRACE_INSN (cpu, "rdcycleh %s;", rd_name);
      RISCV_ASSERT_RV32 (cpu, "insn: %s", op->name);
      store_rd (cpu, rd, fetch_csr (cpu, "cycleh", CSR_CYCLEH, &cpu->csr.cycleh));
      break;
    case MATCH_RDINSTRET:
      TRACE_INSN (cpu, "rdinstret %s;", rd_name);
      store_rd (cpu, rd, fetch_csr (cpu, "instret", CSR_INSTRET, &cpu->csr.instret));
      break;
    case MATCH_RDINSTRETH:
      TRACE_INSN (cpu, "rdinstreth %s;", rd_name);
      RISCV_ASSERT_RV32 (cpu, "insn: %s", op->name);
      store_rd (cpu, rd, fetch_csr (cpu, "instreth", CSR_INSTRETH, &cpu->csr.instreth));
      break;
    case MATCH_RDTIME:
      TRACE_INSN (cpu, "rdtime %s;", rd_name);
      store_rd (cpu, rd, fetch_csr (cpu, "time", CSR_TIME, &cpu->csr.cycle));
      break;
    case MATCH_RDTIMEH:
      TRACE_INSN (cpu, "rdtimeh %s;", rd_name);
      RISCV_ASSERT_RV32 (cpu, "insn: %s", op->name);
      store_rd (cpu, rd, fetch_csr (cpu, "timeh", CSR_TIMEH, &cpu->csr.cycleh));
      break;

    case MATCH_FENCE:
      TRACE_INSN (cpu, "fence;");
      break;
    case MATCH_FENCE_I:
      TRACE_INSN (cpu, "fence.i;");
      break;
    case MATCH_SBREAK:
      TRACE_INSN (cpu, "sbreak;");
      /* GDB expects us to step over SBREAK.  */
      sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_stopped, SIM_SIGTRAP);
      break;
    case MATCH_ECALL:
      TRACE_INSN (cpu, "ecall;");
      if (eh_rve_p)
	sys_id = cpu->t0.u;
      else
	sys_id = cpu->a7.u;

      if (cb_target_to_host_syscall (STATE_CALLBACK (sd), sys_id) == -1)
	{
	  switch (sys_id)
	    {
#ifndef __MINGW32__
	    case TARGET_SYS_link:
	      {
		char oldpath[1024], newpath[1024];
		cb_get_string (cb, &sc, oldpath, sizeof (oldpath), sc.arg1);
		cb_get_string (cb, &sc, newpath, sizeof (newpath), sc.arg2);
#ifdef __MINGW32__
		cpu->a0.u = CreateHardLink(newpath, oldpath, NULL);
#else
		cpu->a0.u = link (oldpath, newpath);
#endif
		break;
	      }
#endif
	    case TARGET_SYS_brk:
	      {
		/* FIXME: Check the invalid access.  */
		if (cpu->a0.u == 0)
		  cpu->a0.u = cpu->endbrk;
		else
		  {
		    if (cpu->a0.u >= DEFAULT_MEM_SIZE)
		      cpu->a0.u = -1;
		    else
		      cpu->endbrk = cpu->a0.u;
		  }
		break;
	      }
	    case TARGET_SYS_gettimeofday:
	      {
		int rv;
		struct timeval tv;

		rv = gettimeofday (&tv, 0);
		if (RISCV_XLEN (cpu) == 32)
		  {
		    sim_core_write_unaligned_4 (cpu, cpu->pc, write_map,
						cpu->a0.u, tv.tv_sec);
		    sim_core_write_unaligned_4 (cpu, cpu->pc, write_map,
						cpu->a0.u + 4,
						tv.tv_usec);
		  }
		else
		  {
		    sim_core_write_unaligned_8 (cpu, cpu->pc, write_map,
						cpu->a0.u, tv.tv_sec);
		    sim_core_write_unaligned_8 (cpu, cpu->pc, write_map,
						cpu->a0.u + 8,
						tv.tv_usec);
		  }

		cpu->a0.u = rv;
		break;
	      }
	    default:
	      cpu->a0.u = sim_syscall (cpu, sys_id, cpu->a0.u,
				       cpu->a1.u, cpu->a2.u, cpu->a3.u);
	      break;
	    }
	}
      else
	cpu->a0.u = sim_syscall (cpu, sys_id, cpu->a0.u, cpu->a1.u, cpu->a2.u, cpu->a3.u);
      break;
    default:
      TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
      sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
    }

  return pc;
}

static unsigned64
mulhu (unsigned64 a, unsigned64 b)
{
  uint64_t t;
  uint32_t y1, y2, y3;
  uint64_t a0 = (uint32_t)a, a1 = a >> 32;
  uint64_t b0 = (uint32_t)b, b1 = b >> 32;

  t = a1*b0 + ((a0*b0) >> 32);
  y1 = t;
  y2 = t >> 32;

  t = a0*b1 + y1;
  y1 = t;

  t = a1*b1 + y2 + (t >> 32);
  y2 = t;
  y3 = t >> 32;

  return ((uint64_t)y3 << 32) | y2;
}

static unsigned64
mulh (signed64 a, signed64 b)
{
  int negate = (a < 0) != (b < 0);
  uint64_t res = mulhu (a < 0 ? -a : a, b < 0 ? -b : b);
  return negate ? ~res + (a * b == 0) : res;
}

static unsigned64
mulhsu (signed64 a, unsigned64 b)
{
  int negate = a < 0;
  uint64_t res = mulhu (a < 0 ? -a : a, b);
  return negate ? ~res + (a * b == 0) : res;
}

static sim_cia
execute_m (SIM_CPU *cpu, unsigned_word iw, const struct riscv_opcode *op, int ex9)
{
  SIM_DESC sd = CPU_STATE (cpu);
  int rd = (iw >> OP_SH_RD) & OP_MASK_RD;
  int rs1 = (iw >> OP_SH_RS1) & OP_MASK_RS1;
  int rs2 = (iw >> OP_SH_RS2) & OP_MASK_RS2;
  const char *rd_name = riscv_gpr_names_abi[rd];
  const char *rs1_name = riscv_gpr_names_abi[rs1];
  const char *rs2_name = riscv_gpr_names_abi[rs2];
  unsigned_word tmp, dividend_max;
  signed_word dividend32_max;
  sim_cia pc = cpu->pc + 4;
  if (ex9)
    pc -= 2;

  dividend_max = -((unsigned_word)1 << (WITH_TARGET_WORD_BITSIZE - 1));
  dividend32_max = INT32_MIN;

  switch (op->match)
    {
    case MATCH_DIV:
      TRACE_INSN (cpu, "div %s, %s, %s;  // %s = %s / %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      if (cpu->regs[rs1].u == dividend_max && cpu->regs[rs2].u == -1)
	tmp = dividend_max;
      else if (cpu->regs[rs2].u)
	tmp = (signed_word)cpu->regs[rs1].u / (signed_word)cpu->regs[rs2].u;
      else
	tmp = -1;
      store_rd (cpu, rd, tmp);
      break;
    case MATCH_DIVW:
      TRACE_INSN (cpu, "divw %s, %s, %s;  // %s = %s / %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
      if (EXTEND32 (cpu->regs[rs1].u) == dividend32_max
	  && EXTEND32 (cpu->regs[rs2].u) == -1)
	tmp = 1 << 31;
      else if (EXTEND32 (cpu->regs[rs2].u))
	tmp = EXTEND32 (cpu->regs[rs1].u) / EXTEND32 (cpu->regs[rs2].u);
      else
	tmp = -1;
      store_rd (cpu, rd, EXTEND32 (tmp));
      break;
    case MATCH_DIVU:
      TRACE_INSN (cpu, "divu %s, %s, %s;  // %s = %s / %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      if (cpu->regs[rs2].u)
	store_rd (cpu, rd, (unsigned_word)cpu->regs[rs1].u
			   / (unsigned_word)cpu->regs[rs2].u);
      else
	store_rd (cpu, rd, -1);
      break;
    case MATCH_DIVUW:
      TRACE_INSN (cpu, "divuw %s, %s, %s;  // %s = %s / %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
      if ((unsigned32)cpu->regs[rs2].u)
	tmp = (unsigned32)cpu->regs[rs1].u / (unsigned32)cpu->regs[rs2].u;
      else
	tmp = -1;
      store_rd (cpu, rd, EXTEND32 (tmp));
      break;
    case MATCH_MUL:
      TRACE_INSN (cpu, "mul %s, %s, %s;  // %s = %s * %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      store_rd (cpu, rd, cpu->regs[rs1].u * cpu->regs[rs2].u);
      break;
    case MATCH_MULW:
      TRACE_INSN (cpu, "mulw %s, %s, %s;  // %s = %s * %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
      store_rd (cpu, rd, EXTEND32 ((signed32)cpu->regs[rs1].u
				   * (signed32)cpu->regs[rs2].u));
      break;
    case MATCH_MULH:
      TRACE_INSN (cpu, "mulh %s, %s, %s;  // %s = %s * %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      if (RISCV_XLEN (cpu) == 32)
	store_rd (cpu, rd, ((signed64)(signed_word)cpu->regs[rs1].u
			    * (signed64)(signed_word)cpu->regs[rs2].u) >> 32);
      else
	store_rd (cpu, rd, mulh (cpu->regs[rs1].u, cpu->regs[rs2].u));
      break;
    case MATCH_MULHU:
      TRACE_INSN (cpu, "mulhu %s, %s, %s;  // %s = %s * %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      if (RISCV_XLEN (cpu) == 32)
	store_rd (cpu, rd, ((unsigned64)cpu->regs[rs1].u
			    * (unsigned64)cpu->regs[rs2].u) >> 32);
      else
	store_rd (cpu, rd, mulhu (cpu->regs[rs1].u, cpu->regs[rs2].u));
      break;
    case MATCH_MULHSU:
      TRACE_INSN (cpu, "mulhsu %s, %s, %s;  // %s = %s * %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      if (RISCV_XLEN (cpu) == 32)
	store_rd (cpu, rd, ((signed64)(signed_word)cpu->regs[rs1].u
			    * (unsigned64)cpu->regs[rs2].u) >> 32);
      else
	store_rd (cpu, rd, mulhsu (cpu->regs[rs1].u, cpu->regs[rs2].u));
      break;
    case MATCH_REM:
      TRACE_INSN (cpu, "rem %s, %s, %s;  // %s = %s %% %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      if (cpu->regs[rs1].u == dividend_max && cpu->regs[rs2].u == -1)
	tmp = 0;
      else if (cpu->regs[rs2].u)
	tmp = (signed_word)cpu->regs[rs1].u % (signed_word)cpu->regs[rs2].u;
      else
	tmp = cpu->regs[rs1].u;
      store_rd (cpu, rd, tmp);
      break;
    case MATCH_REMW:
      TRACE_INSN (cpu, "remw %s, %s, %s;  // %s = %s %% %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
      if (EXTEND32 (cpu->regs[rs1].u) == dividend32_max
	  && EXTEND32 (cpu->regs[rs2].u) == -1)
	tmp = 0;
      else if (EXTEND32 (cpu->regs[rs2].u))
	tmp = EXTEND32 (cpu->regs[rs1].u) % EXTEND32 (cpu->regs[rs2].u);
      else
	tmp = cpu->regs[rs1].u;
      store_rd (cpu, rd, EXTEND32 (tmp));
      break;
    case MATCH_REMU:
      TRACE_INSN (cpu, "remu %s, %s, %s;  // %s = %s %% %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      if (cpu->regs[rs2].u)
	store_rd (cpu, rd, cpu->regs[rs1].u % cpu->regs[rs2].u);
      else
	store_rd (cpu, rd, cpu->regs[rs1].u);
      break;
    case MATCH_REMUW:
      TRACE_INSN (cpu, "remuw %s, %s, %s;  // %s = %s %% %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
      if ((unsigned32)cpu->regs[rs2].u)
	tmp = (unsigned32)cpu->regs[rs1].u % (unsigned32)cpu->regs[rs2].u;
      else
	tmp = cpu->regs[rs1].u;
      store_rd (cpu, rd, EXTEND32 (tmp));
      break;
    default:
      TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
      sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
    }

  return pc;
}

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

static sim_cia
execute_a (SIM_CPU *cpu, unsigned_word iw, const struct riscv_opcode *op, int ex9)
{
  SIM_DESC sd = CPU_STATE (cpu);
  int rd = (iw >> OP_SH_RD) & OP_MASK_RD;
  int rs1 = (iw >> OP_SH_RS1) & OP_MASK_RS1;
  int rs2 = (iw >> OP_SH_RS2) & OP_MASK_RS2;
  const char *rd_name = riscv_gpr_names_abi[rd];
  const char *rs1_name = riscv_gpr_names_abi[rs1];
  const char *rs2_name = riscv_gpr_names_abi[rs2];
  struct atomic_mem_reserved_list *amo_prev, *amo_curr;
  insn_t aqrl_mask = (OP_MASK_AQ << OP_SH_AQ) | (OP_MASK_RL << OP_SH_RL);
  unsigned_word tmp;
  unsigned_word rs2_val = cpu->regs[rs2].u;
  sim_cia pc = cpu->pc + 4;
  if (ex9)
    pc -= 2;

  /* Handle these two load/store operations specifically.  */
  switch (op->match & ~aqrl_mask)
    {
    case MATCH_LR_W:
      TRACE_INSN (cpu, "%s %s, (%s);", op->name, rd_name, rs1_name);
      store_rd (cpu, rd,
	sim_core_read_unaligned_4 (cpu, cpu->pc, read_map, cpu->regs[rs1].u));

      /* Walk the reservation list to find an existing match.  */
      amo_curr = sd->amo_reserved_list;
      while (amo_curr)
	{
	  if (amo_curr->addr == cpu->regs[rs1].u)
	    goto done;
	  amo_curr = amo_curr->next;
	}

      /* No reservation exists, so add one.  */
      amo_curr = xmalloc (sizeof (*amo_curr));
      amo_curr->addr = cpu->regs[rs1].u;
      amo_curr->next = sd->amo_reserved_list;
      sd->amo_reserved_list = amo_curr;
      goto done;
    case MATCH_SC_W:
      TRACE_INSN (cpu, "%s %s, %s, (%s);", op->name, rd_name, rs2_name, rs1_name);

      /* Walk the reservation list to find a match.  */
      amo_curr = amo_prev = sd->amo_reserved_list;
      while (amo_curr)
	{
	  if (amo_curr->addr == cpu->regs[rs1].u)
	    {
	      /* We found a reservation, so operate it.  */
	      sim_core_write_unaligned_4 (cpu, cpu->pc, write_map,
					  cpu->regs[rs1].u, cpu->regs[rs2].u);
	      store_rd (cpu, rd, 0);
	      if (amo_curr == sd->amo_reserved_list)
		sd->amo_reserved_list = amo_curr->next;
	      else
		amo_prev->next = amo_curr->next;
	      free (amo_curr);
	      goto done;
	    }
	  amo_prev = amo_curr;
	  amo_curr = amo_curr->next;
	}

      /* If we're still here, then no reservation exists, so mark as failed.  */
      store_rd (cpu, rd, 1);
      goto done;
    }

  /* Handle the rest of the atomic insns with common code paths.  */
  TRACE_INSN (cpu, "%s %s, %s, (%s);",
	      op->name, rd_name, rs2_name, rs1_name);
  if (op->xlen_requirement == 64)
    tmp = sim_core_read_unaligned_8 (cpu, cpu->pc, read_map, cpu->regs[rs1].u);
  else
    tmp = EXTEND32 (sim_core_read_unaligned_4 (cpu, cpu->pc,
					       read_map, cpu->regs[rs1].u));

  store_rd (cpu, rd, tmp);

  switch (op->match & ~aqrl_mask)
    {
    case MATCH_AMOADD_D:
    case MATCH_AMOADD_W:
      tmp = tmp + cpu->regs[rs2].u;
      break;
    case MATCH_AMOAND_D:
    case MATCH_AMOAND_W:
      tmp = tmp & cpu->regs[rs2].u;
      break;
    case MATCH_AMOMAX_D:
    case MATCH_AMOMAX_W:
      tmp = MAX ((signed_word)tmp, (signed_word)cpu->regs[rs2].u);
      break;
    case MATCH_AMOMAXU_D:
    case MATCH_AMOMAXU_W:
      tmp = MAX ((unsigned_word)tmp, (unsigned_word)cpu->regs[rs2].u);
      break;
    case MATCH_AMOMIN_D:
    case MATCH_AMOMIN_W:
      tmp = MIN ((signed_word)tmp, (signed_word)cpu->regs[rs2].u);
      break;
    case MATCH_AMOMINU_D:
    case MATCH_AMOMINU_W:
      tmp = MIN ((unsigned_word)tmp, (unsigned_word)cpu->regs[rs2].u);
      break;
    case MATCH_AMOOR_D:
    case MATCH_AMOOR_W:
      tmp = tmp | cpu->regs[rs2].u;
      break;
    case MATCH_AMOSWAP_D:
    case MATCH_AMOSWAP_W:
      tmp = rs2_val;
      break;
    case MATCH_AMOXOR_D:
    case MATCH_AMOXOR_W:
      tmp = tmp ^ cpu->regs[rs2].u;
      break;
    default:
      TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
      sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
    }

  if (op->xlen_requirement == 64)
    sim_core_write_unaligned_8 (cpu, cpu->pc, write_map, cpu->regs[rs1].u, tmp);
  else
    sim_core_write_unaligned_4 (cpu, cpu->pc, write_map, cpu->regs[rs1].u, tmp);

 done:
  return pc;
}

static sim_cia
execute_zfh (SIM_CPU *cpu, unsigned_word iw, const struct riscv_opcode *op, int ex9)
{
  SIM_DESC sd = CPU_STATE (cpu);
  unsigned int mask_arithmetic = MASK_FADD_H;
  unsigned int mask_mul_add = MASK_FMADD_H;
  unsigned int mask_convert = MASK_FCVT_H_W;

  static const int round_modes[] =
  {
      sim_fpu_round_near, sim_fpu_round_zero,
      sim_fpu_round_down, sim_fpu_round_up,
      sim_fpu_round_default, sim_fpu_round_default,
      sim_fpu_round_default
  };

  int rd = (iw >> OP_SH_RD) & OP_MASK_RD;
  int rs1 = (iw >> OP_SH_RS1) & OP_MASK_RS1;
  int rs2 = (iw >> OP_SH_RS2) & OP_MASK_RS2;
  int rs3 = (iw >> OP_SH_RS3) & OP_MASK_RS3;
  const char *frd_name = riscv_fpr_names_abi[rd];
  const char *frs1_name = riscv_fpr_names_abi[rs1];
  const char *frs2_name = riscv_fpr_names_abi[rs2];
  const char *frs3_name = riscv_fpr_names_abi[rs3];
  const char *rd_name = riscv_gpr_names_abi[rd];
  const char *rs1_name = riscv_gpr_names_abi[rs1];
  unsigned_word i_imm = EXTRACT_ITYPE_IMM (iw);
  unsigned_word s_imm = EXTRACT_STYPE_IMM (iw);
  uint16_t u16;
  int16_t i16;
  float16_t tmp;
  float16_t zero = {0};
  sim_cia pc = cpu->pc + 4;
  /* Rounding mode.  */
  int rm = (iw >> OP_SH_RM) & OP_MASK_RM;
  int rounding = round_modes[rm];

  if (ex9)
    pc -= 2;

  switch (op->match & mask_mul_add)
    {
    case MATCH_FMADD_H:
      TRACE_INSN (cpu, "fmadd.h %s, %s, %s, %s",
		  frd_name, frs1_name, frs2_name, frs3_name);
      tmp = f16_mul (cpu->fpregs[rs1].hf[0], cpu->fpregs[rs2].hf[0]);
      cpu->fpregs[rd].hf[0] = f16_add (tmp, cpu->fpregs[rs3].hf[0]);
      TRACE_FREG (cpu, rd);
      goto done;
    case MATCH_FMSUB_H:
      TRACE_INSN (cpu, "fmsub.h %s, %s, %s, %s",
		  frd_name, frs1_name, frs2_name, frs3_name);
      tmp = f16_mul (cpu->fpregs[rs1].hf[0], cpu->fpregs[rs2].hf[0]);
      cpu->fpregs[rd].hf[0] = f16_sub (tmp, cpu->fpregs[rs3].hf[0]);
      TRACE_FREG (cpu, rd);
      goto done;
    case MATCH_FNMADD_H:
      TRACE_INSN (cpu, "fnmadd.h %s, %s, %s, %s",
		  frd_name, frs1_name, frs2_name, frs3_name);
      tmp = f16_sub (zero, cpu->fpregs[rs1].hf[0]);
      tmp = f16_mul (tmp, cpu->fpregs[rs2].hf[0]);
      cpu->fpregs[rd].hf[0] = f16_sub (tmp, cpu->fpregs[rs3].hf[0]);
      TRACE_FREG (cpu, rd);
      goto done;
    case MATCH_FNMSUB_H:
      TRACE_INSN (cpu, "fnmsub.h %s, %s, %s, %s",
		  frd_name, frs1_name, frs2_name, frs3_name);
      tmp = f16_sub (zero, cpu->fpregs[rs1].hf[0]);
      tmp = f16_mul (tmp, cpu->fpregs[rs2].hf[0]);
      cpu->fpregs[rd].hf[0] = f16_add (tmp, cpu->fpregs[rs3].hf[0]);
      TRACE_FREG (cpu, rd);
      goto done;
    }

  switch (op->match & mask_arithmetic)
    {
    case MATCH_FADD_H:
      TRACE_INSN (cpu, "fadd.h %s, %s, %s",
		  frd_name, frs1_name, frs2_name);
      cpu->fpregs[rd].hf[0] = f16_add (cpu->fpregs[rs1].hf[0], cpu->fpregs[rs2].hf[0]);
      goto done;
    case MATCH_FSUB_H:
      TRACE_INSN (cpu, "fsub.h %s, %s, %s",
		  frd_name, frs1_name, frs2_name);
      cpu->fpregs[rd].hf[0] = f16_sub (cpu->fpregs[rs1].hf[0], cpu->fpregs[rs2].hf[0]);
      goto done;
    case MATCH_FMUL_H:
      TRACE_INSN (cpu, "fmul.h %s, %s, %s",
		  frd_name, frs1_name, frs2_name);
      cpu->fpregs[rd].hf[0] = f16_mul (cpu->fpregs[rs1].hf[0], cpu->fpregs[rs2].hf[0]);
      goto done;
    case MATCH_FDIV_H:
      TRACE_INSN (cpu, "fdiv.h %s, %s, %s",
		  frd_name, frs1_name, frs2_name);
      cpu->fpregs[rd].hf[0] = f16_div (cpu->fpregs[rs1].hf[0], cpu->fpregs[rs2].hf[0]);
      goto done;
    case MATCH_FSQRT_H:
      TRACE_INSN (cpu, "fdiv.h %s, %s, %s",
		  frd_name, frs1_name, frs2_name);
      cpu->fpregs[rd].hf[0] = f16_sqrt (cpu->fpregs[rs1].hf[0]);
      goto done;
    }

  switch (op->match & mask_convert)
    {
    case MATCH_FCVT_W_H:
      TRACE_INSN (cpu, "fcvt.w.h %s, %s",
		  rd_name, frs1_name);
      cpu->regs[rd].s = f16_to_i32(cpu->fpregs[rs1].hf[0],
                                   softfloat_round_minMag,
				   true);
      TRACE_REG (cpu, rd);
      goto done;
    case MATCH_FCVT_H_W:
      TRACE_INSN (cpu, "fcvt.h.w %s, %s",
		  frd_name, rs1_name);
      cpu->fpregs[rd].hf[0] = i32_to_f16(cpu->regs[rd].s);
      TRACE_FREG (cpu, rd);
      goto done;
    case MATCH_FCVT_WU_H:
      TRACE_INSN (cpu, "fcvt.wu.h %s, %s",
		  rd_name, frs1_name);
      cpu->regs[rd].u = f16_to_ui32(cpu->fpregs[rs1].hf[0],
                                   softfloat_round_minMag,
				   true);
      TRACE_REG (cpu, rd);
      goto done;
    case MATCH_FCVT_H_WU:
      TRACE_INSN (cpu, "fcvt.h.wu %s, %s",
		  frd_name, rs1_name);
      cpu->fpregs[rd].hf[0] = ui32_to_f16(cpu->regs[rd].u);
      TRACE_FREG (cpu, rd);
      goto done;
    case MATCH_FCVT_L_H:
#if (WITH_TARGET_WORD_BITSIZE == 64)
      TRACE_INSN (cpu, "fcvt.l.h %s, %s",
		  rd_name, frs1_name);
      cpu->regs[rd].s = f16_to_i64(cpu->fpregs[rs1].hf[0],
                                   softfloat_round_minMag,
				   true);
      TRACE_REG (cpu, rd);
#else
      TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
      sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      goto done;
    case MATCH_FCVT_H_L:
#if (WITH_TARGET_WORD_BITSIZE == 64)
      TRACE_INSN (cpu, "fcvt.h.l %s, %s",
		  frd_name, rs1_name);
      cpu->fpregs[rd].hf[0] = i64_to_f16(cpu->regs[rd].s);
      TRACE_FREG (cpu, rd);
#else
      TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
      sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      goto done;
    case MATCH_FCVT_LU_H:
#if (WITH_TARGET_WORD_BITSIZE == 64)
      TRACE_INSN (cpu, "fcvt.lu.h %s, %s",
		  rd_name, frs1_name);
      cpu->regs[rd].u = f16_to_ui64(cpu->fpregs[rs1].hf[0],
                                   softfloat_round_minMag,
				   true);
      TRACE_REG (cpu, rd);
#else
      TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
      sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      goto done;
    case MATCH_FCVT_H_LU:
#if (WITH_TARGET_WORD_BITSIZE == 64)
      TRACE_INSN (cpu, "fcvt.h.lu %s, %s",
		  frd_name, rs1_name);
      cpu->fpregs[rd].hf[0] = ui64_to_f16(cpu->regs[rd].u);
      TRACE_FREG (cpu, rd);
#else
      TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
      sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
#endif
      goto done;
    }

  switch (op->match)
    {
    case MATCH_FLH:
      TRACE_INSN (cpu, "flh %s, %" PRIiTW "(%s)",
		  frd_name, i_imm, rs1_name);
      store_frd (cpu, rd, EXTEND16 (
	sim_core_read_unaligned_2 (cpu, cpu->pc, read_map,
				   cpu->regs[rs1].u + i_imm)));
      break;
    case MATCH_FSH:
      TRACE_INSN (cpu, "fsh %s, %" PRIiTW "(%s)",
		  frs2_name, s_imm, rs1_name);
      sim_core_write_unaligned_2 (cpu, cpu->pc, write_map,
				  cpu->regs[rs1].u + s_imm, cpu->fpregs[rs2].h[0]);
      break;
    case MATCH_FSGNJ_H:
      TRACE_INSN (cpu, "fsgnj.h %s, %s, %s",
		  frd_name, frs1_name, frs2_name);
      u16 = cpu->fpregs[rs1].h[0] & 0x7fff;
      u16 |= cpu->fpregs[rs2].h[0] & 0x8000;
      cpu->fpregs[rd].h[0] = u16;
      TRACE_FREG (cpu, rd);
      break;
    case MATCH_FSGNJN_H:
      TRACE_INSN (cpu, "fsgnjn.h %s, %s, %s",
		  frd_name, frs1_name, frs2_name);
      u16 = cpu->fpregs[rs1].h[0] & 0x7ffff;
      u16 |= (cpu->fpregs[rs2].h[0] & 0x80000) ^ 0x80000;
      cpu->fpregs[rd].h[0] = u16;
      TRACE_FREG (cpu, rd);
      break;
    case MATCH_FSGNJX_H:
      TRACE_INSN (cpu, "fsgnx.h %s, %s, %s",
		  frd_name, frs1_name, frs2_name);
      u16 = cpu->fpregs[rs1].h[0] & 0x7fff;
      u16 |= (cpu->fpregs[rs1].h[0] & 0x8000) ^ (cpu->fpregs[rs2].h[0] & 0x8000);
      cpu->fpregs[rd].h[0] = u16;
      TRACE_FREG (cpu, rd);
      break;
    case MATCH_FMIN_H:
      TRACE_INSN (cpu, "fmin.h %s, %s, %s",
		  frd_name, frs1_name, frs2_name);
      if (f16_lt (cpu->fpregs[rs1].hf[0], cpu->fpregs[rs2].hf[0]))
        cpu->fpregs[rd].hf[0] = cpu->fpregs[rs1].hf[0];
      else
        cpu->fpregs[rd].hf[0] = cpu->fpregs[rs2].hf[0];
      TRACE_FREG (cpu, rd);
      break;
    case MATCH_FMAX_H:
      TRACE_INSN (cpu, "fmax.h %s, %s, %s",
		  frd_name, frs1_name, frs2_name);
      if (f16_lt (cpu->fpregs[rs1].hf[0], cpu->fpregs[rs2].hf[0]))
        cpu->fpregs[rd].hf[0] = cpu->fpregs[rs2].hf[0];
      else
        cpu->fpregs[rd].hf[0] = cpu->fpregs[rs1].hf[0];
      TRACE_FREG (cpu, rd);
      break;
    case MATCH_FMV_X_H:
      TRACE_INSN (cpu, "fmv.x.h %s, %s",
		  rd_name, frs1_name);
      cpu->regs[rd].b16.h0 = cpu->fpregs[rs1].H[0];
      TRACE_REG (cpu, rd);
      break;
    case MATCH_FMV_H_X:
      TRACE_INSN (cpu, "fmv.h.x %s, %s",
		  frd_name, rs1_name);
      cpu->fpregs[rd].H[0] = cpu->regs[rs1].b16.h0;
      TRACE_FREG (cpu, rd);
      break;
    case MATCH_FEQ_H:
      TRACE_INSN (cpu, "feq.h %s, %s, %s",
		  rd_name, frs1_name, frs2_name);
      cpu->regs[rd].u = f16_eq (cpu->fpregs[rs1].hf[0], cpu->fpregs[rs2].hf[0]);
      TRACE_REG (cpu, rd);
      break;
    case MATCH_FLE_H:
      TRACE_INSN (cpu, "fle.h %s, %s, %s",
		  rd_name, frs1_name, frs2_name);
      cpu->regs[rd].u = f16_le (cpu->fpregs[rs1].hf[0], cpu->fpregs[rs2].hf[0]);
      TRACE_REG (cpu, rd);
      break;
    case MATCH_FLT_H:
      TRACE_INSN (cpu, "flt.h %s, %s, %s",
		  rd_name, frs1_name, frs2_name);
      cpu->regs[rd].u = f16_lt (cpu->fpregs[rs1].hf[0], cpu->fpregs[rs2].hf[0]);
      TRACE_REG (cpu, rd);
      break;
    case MATCH_FCLASS_S:
      TRACE_INSN (cpu, "UNIMPLEMENTED INSN: %s", op->name);
      sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
      break;
    default:
      TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
      sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
    }

 done:
  return pc;
}

static sim_cia
execute_xebfhw (SIM_CPU *cpu, unsigned_word iw, const struct riscv_opcode *op, int ex9)
{
  SIM_DESC sd = CPU_STATE (cpu);

  int rd = (iw >> OP_SH_RD) & OP_MASK_RD;
  int rs2 = (iw >> OP_SH_RS2) & OP_MASK_RS2;

  switch (op->match & MASK_FCVT_S_BF16)
    {
    case MATCH_FCVT_S_BF16:
      cpu->fpregs[rd].w[0] = cpu->fpregs[rs2].w[0] << 16;
      break;
    case MATCH_FCVT_BF16_S:
      cpu->fpregs[rd].w[0] = cpu->fpregs[rs2].w[0] >> 16;
      break;
    default:
      TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
      sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
    }

  return cpu->pc + 4;
}

static sim_cia
execute_one (SIM_CPU *cpu, unsigned_word iw, const struct riscv_opcode *op, int ex9)
{
  SIM_DESC sd = CPU_STATE (cpu);
  const char *subset = op->subset[0];
  int i;

  for (i = 0; op->subset[i]; i++)
    {
      /* Use subset 'C' if it exists.  */
      if (op->subset[i][0] == 'C')
	subset = op->subset[i];
    }

  if (op->xlen_requirement == 64)
    RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
  else if (op->xlen_requirement == 32)
    RISCV_ASSERT_RV32 (cpu, "insn: %s", op->name);

  switch (subset[0])
    {
    case 'A':
      return execute_a (cpu, iw, op, ex9);
    case 'C':
      return execute_c (cpu, iw, op);
    case 'D':
      return execute_d (cpu, iw, op, ex9);
    case 'F':
      return execute_f (cpu, iw, op, ex9);
    case 'I':
      return execute_i (cpu, iw, op, ex9);
    case 'M':
      return execute_m (cpu, iw, op, ex9);
    case 'P':
      return execute_p (cpu, iw, op, ex9);
    case 'X':
      subset++;
      if (strncmp(subset, "ANDES", 5) == 0)
        return execute_andes(cpu, iw, op, ex9);
      if (strncmp (subset, "EBFHW", 5) == 0)
        return execute_xebfhw(cpu, iw, op, ex9);
    case 'Z':
      subset++;
      if (strncmp(subset, "FH", 2) == 0)
        return execute_zfh(cpu, iw, op, ex9);
    default:
      TRACE_INSN (cpu, "UNHANDLED EXTENSION: %s", op->subset[0]);
      sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
    }

  return cpu->pc + riscv_insn_length (iw);
}

sim_cia
riscv_decode (SIM_CPU *cpu, unsigned_word iw, sim_cia pc, int ex9)
{
  SIM_DESC sd = CPU_STATE (cpu);
  const struct riscv_opcode *op;
  int xlen = RISCV_XLEN (cpu);

  op = riscv_hash[OP_HASH_IDX (iw)];
  if (!op)
    sim_engine_halt (sd, cpu, NULL, pc, sim_signalled, SIM_SIGILL);

  for (; op->name; op++)
    {
      /* Does the opcode match?  */
      if (!(op->match_func) (op, iw, 0, NULL))
	continue;

      /* It's a match.  */
      return execute_one (cpu, iw, op, ex9);
    }

  sim_engine_halt (sd, cpu, NULL, pc, sim_signalled, SIM_SIGILL);
}

/* Decode & execute a single instruction.  */
void step_once (SIM_CPU *cpu)
{
  SIM_DESC sd = CPU_STATE (cpu);
  unsigned_word iw;
  unsigned int len;
  sim_cia pc = cpu->pc;
  const struct riscv_opcode *op;
  int xlen = RISCV_XLEN (cpu);

  if (TRACE_ANY_P (cpu))
    trace_prefix (sd, cpu, NULL_CIA, pc, TRACE_LINENUM_P (cpu),
		  NULL, 0, " "); /* Use a space for gcc warnings.  */

  iw = sim_core_read_aligned_2 (cpu, pc, exec_map, pc);

  len = riscv_insn_length (iw);

  if (len == 4)
    iw |= ((unsigned_word)sim_core_read_aligned_2 (cpu, pc, exec_map, pc + 2) << 16);
  else
    iw |= ((unsigned_word)sim_core_read_aligned_2 (cpu, pc, exec_map, pc));

  TRACE_CORE (cpu, "0x%08"PRIxTW, iw);

  pc = riscv_decode (cpu, iw, pc, 0);

  /* TODO: Try to use a common counter and only update on demand (reads).  */
  if (RISCV_XLEN (cpu) == 32)
    {
      unsigned_word old_cycle = cpu->csr.cycle++;
      ++cpu->csr.mcycle;

      if (old_cycle > cpu->csr.cycle)
	{
	  /* Increase cycleh if cycle is overflowed.  */
	  cpu->csr.cycleh++;
	  cpu->csr.mcycleh++;
	}
    }
  else
    {
      ++cpu->csr.cycle;
      ++cpu->csr.mcycle;
    }
  ++cpu->csr.instret;

  /* Halt if jump to 0, it's almost always some thing wrong here.  */
  if (pc == 0)
    {
      fprintf (stderr, "pc == 0\n");
      sim_engine_halt (sd, cpu, NULL, cpu->pc,
		       sim_signalled, SIM_SIGILL);
    }

  cpu->pc = pc;
}

/* Return the program counter for this cpu. */
static sim_cia
pc_get (sim_cpu *cpu)
{
  return cpu->pc;
}

/* Set the program counter for this cpu to the new pc value. */
static void
pc_set (sim_cpu *cpu, sim_cia pc)
{
  cpu->pc = pc;
}

static int
reg_fetch (sim_cpu *cpu, int rn, unsigned char *buf, int len)
{
  if (len <= 0)
    return -1;

  if (rn >= SIM_RISCV_FIRST_FP_REGNUM && rn <= SIM_RISCV_LAST_FP_REGNUM)
    {
      int float_abi = CPU_ELF_FLAGS(cpu) & EF_RISCV_FLOAT_ABI;
      int n = float_abi >> 1;

      switch (float_abi)
	{
	case EF_RISCV_FLOAT_ABI_SOFT:
	  if (len > sizeof (unsigned_word))
	    return -1;
	  break;
	case EF_RISCV_FLOAT_ABI_SINGLE:
	case EF_RISCV_FLOAT_ABI_DOUBLE:
	case EF_RISCV_FLOAT_ABI_QUAD:
	  if (len > (sizeof (unsigned32) * n))
	    return -1;
	  break;
	default:
	  return -1;
	}
    }
  else if (len > sizeof (unsigned_word))
    return -1;

  switch (rn)
    {
    case SIM_RISCV_RA_REGNUM ... SIM_RISCV_T6_REGNUM:
      memcpy (buf, &cpu->regs[rn], len);
      return len;
    case SIM_RISCV_FIRST_FP_REGNUM ... SIM_RISCV_LAST_FP_REGNUM:
      rn -= SIM_RISCV_FIRST_FP_REGNUM;
      memcpy (buf, &cpu->fpregs[rn], len);
      return len;
    case SIM_RISCV_PC_REGNUM:
      memcpy (buf, &cpu->pc, len);
      return len;

#define DECLARE_CSR(name, num, cls) \
    case SIM_RISCV_ ## num ## _REGNUM: \
      memcpy (buf, &cpu->csr.name, len); \
      return len;
#include "opcode/riscv-opc.h"
#undef DECLARE_CSR

    default:
      return -1;
    }
}

static int
reg_store (sim_cpu *cpu, int rn, unsigned char *buf, int len)
{
  if (len <= 0)
    return -1;

  if (rn >= SIM_RISCV_FIRST_FP_REGNUM && rn <= SIM_RISCV_LAST_FP_REGNUM)
    {
      int float_abi = CPU_ELF_FLAGS(cpu) & EF_RISCV_FLOAT_ABI;
      int n = float_abi >> 1;

      switch (float_abi)
	{
	case EF_RISCV_FLOAT_ABI_SOFT:
	  if (len > sizeof (unsigned_word))
	    return -1;
	  break;
	case EF_RISCV_FLOAT_ABI_SINGLE:
	case EF_RISCV_FLOAT_ABI_DOUBLE:
	case EF_RISCV_FLOAT_ABI_QUAD:
	  if (len > (sizeof (unsigned32) * n))
	    return -1;
	  break;
	default:
	  return -1;
	}
    }
  else if (len > sizeof (unsigned_word))
    return -1;

  switch (rn)
    {
    case SIM_RISCV_ZERO_REGNUM:
      /* Always return len to avoid warning/error in gdbsim_store_register.  */
      return len;
    case SIM_RISCV_RA_REGNUM ... SIM_RISCV_T6_REGNUM:
      memcpy (&cpu->regs[rn], buf, len);
      return len;
    case SIM_RISCV_FIRST_FP_REGNUM ... SIM_RISCV_LAST_FP_REGNUM:
      rn -= SIM_RISCV_FIRST_FP_REGNUM;
      memcpy (&cpu->fpregs[rn], buf, len);
      return len;
    case SIM_RISCV_PC_REGNUM:
      memcpy (&cpu->pc, buf, len);
      return len;

#define DECLARE_CSR(name, num, cls) \
    case SIM_RISCV_ ## num ## _REGNUM: \
      memcpy (&cpu->csr.name, buf, len); \
      return len;
#include "opcode/riscv-opc.h"
#undef DECLARE_CSR

    default:
      return -1;
    }
}

/* Initialize the state for a single cpu.  Usuaully this involves clearing all
   registers back to their reset state.  Should also hook up the fetch/store
   helper functions too.  */
void initialize_cpu (SIM_DESC sd, SIM_CPU *cpu, int mhartid)
{
  const char *extensions;
  int i;
  unsigned n;
  const struct riscv_opcode *op;

  memset (cpu->regs, 0, sizeof (cpu->regs));

  CPU_PC_FETCH (cpu) = pc_get;
  CPU_PC_STORE (cpu) = pc_set;
  CPU_REG_FETCH (cpu) = reg_fetch;
  CPU_REG_STORE (cpu) = reg_store;

  if (sim_riscv_opcodes)
    free (sim_riscv_opcodes);

  /* Calculate how many entry we need for sim_riscv_opcodes.  */
  for (n = 0, op = riscv_opcodes; op->name; op++)
    {
      /* Skip all pseudo-instructions.  */
      if ((op->pinfo & INSN_ALIAS))
	continue;

      /* Skip all instructions which is not valid for current XLEN.  */
      if (op->xlen_requirement != 0 && op->xlen_requirement != RISCV_XLEN (cpu))
	continue;

      ++n;
    }

  /* +1 for sentinel.  */
  sim_riscv_opcodes = xmalloc (sizeof (struct riscv_opcode) * (n + 1));

  /* Copy riscv_opcodes into sim_riscv_opcodes.  */
  for (n = 0, op = riscv_opcodes; op->name; op++)
    {
      /* Skip all pseudo-instructions.  */
      if ((op->pinfo & INSN_ALIAS))
	continue;

      /* Skip all instructions which is not valid for current XLEN.  */
      if (op->xlen_requirement != 0 && op->xlen_requirement != RISCV_XLEN (cpu))
	continue;

      sim_riscv_opcodes[n++] = *op;
    }

  /* Setup sentinel. */
  memset (&sim_riscv_opcodes[n], 0, sizeof (struct riscv_opcode));

  /* Initialize for hash table.  */
  memset (riscv_hash, 0, sizeof (struct riscv_opcode *) * HASH_TABLE_SZ);

  for (op = sim_riscv_opcodes; op->name; op++)
    if (!riscv_hash[OP_HASH_IDX (op->match)])
      riscv_hash[OP_HASH_IDX (op->match)] = op;

  cpu->csr.misa = 0;
  /* RV32 sets this field to 0, and we don't really support RV128 yet.  */
  if (RISCV_XLEN (cpu) == 64)
    cpu->csr.misa |= (unsigned64)2 << 62;

  /* Skip the leading "rv" prefix and the two numbers.  */
  extensions = MODEL_NAME (CPU_MODEL (cpu)) + 4;
  for (i = 0; i < 26; ++i)
    {
      char ext = 'A' + i;

      if (ext == 'X')
	continue;
      else if (strchr (extensions, ext) != NULL)
	{
	  if (ext == 'G')
	    cpu->csr.misa |= 0x1129;  /* G = IMAFD.  */
	  else
	    cpu->csr.misa |= (1 << i);
	}
    }

  cpu->csr.mimpid = 0x8000;
  cpu->csr.mhartid = mhartid;
  cpu->csr.cycle = 0;
  cpu->csr.mcycle = 0;
  cpu->csr.instret = 0;
  /* dset = 256 set, dway = 4 way, dsz = 32 bytes, dc_ecc = ECC,
   * dlmb = One ILMB exists, dlmsz = 512 KiB, dlm_ecc = No parity/ECC */
  cpu->csr.mdcm_cfg = 0x518da;
  /* Enable execit */
  cpu->csr.mmsc_cfg = 0x8;
}

/* Some utils don't like having a NULL environ.  */
static const char * const simple_env[] = { "HOME=/", "PATH=/bin", NULL };

/* Count the number of arguments in an argv.  */
static int
count_argv (const char * const *argv)
{
  int i;

  if (!argv)
    return -1;

  for (i = 0; argv[i] != NULL; ++i)
    continue;
  return i;
}

void initialize_env (SIM_DESC sd, const char * const *argv,
		     const char * const *env)
{
  SIM_CPU *cpu = STATE_CPU (sd, 0);
  int i;
  int argc, argv_flat;
  int envc, env_flat;
  address_word sp, sp_flat;
  unsigned char null[8] = { 0, 0, 0, 0, 0, 0, 0, 0, };

  /* Figure out how many bytes the argv strings take up.  */
  argc = count_argv (argv);
  if (argc == -1)
    argc = 0;
  argv_flat = argc; /* NUL bytes.  */
  for (i = 0; i < argc; ++i)
    argv_flat += strlen (argv[i]);

  /* Figure out how many bytes the environ strings take up.  */
  if (!env)
    env = simple_env;
  envc = count_argv (env);
  env_flat = envc; /* NUL bytes.  */
  for (i = 0; i < envc; ++i)
    env_flat += strlen (env[i]);

  /* Make space for the strings themselves.  */
  sp_flat = (DEFAULT_MEM_SIZE - argv_flat - env_flat) & -sizeof (address_word);
  /* Then the pointers to the strings.  */
  sp = sp_flat - ((argc + 1 + envc + 1) * sizeof (address_word));
  /* Then the argc.  */
  sp -= sizeof (unsigned_word);
  /* Synchronize sp alignment with GCC's STACK_BOUNDARY.  */
  sp = ALIGN_16 (sp - 15);

  /* Set up the regs the libgloss crt0 expects.  */
  cpu->a0.u = argc;
  cpu->sp.u = sp;

  /* First push the argc value.  */
  sim_write (sd, sp, (void *)&argc, sizeof (unsigned_word));
  sp += sizeof (unsigned_word);

  /* Then the actual argv strings so we know where to point argv[].  */
  for (i = 0; i < argc; ++i)
    {
      unsigned len = strlen (argv[i]) + 1;
      sim_write (sd, sp_flat, (void *)argv[i], len);
      sim_write (sd, sp, (void *)&sp_flat, sizeof (address_word));
      sp_flat += len;
      sp += sizeof (address_word);
    }
  sim_write (sd, sp, null, sizeof (address_word));
  sp += sizeof (address_word);

  /* Then the actual env strings so we know where to point env[].  */
  for (i = 0; i < envc; ++i)
    {
      unsigned len = strlen (env[i]) + 1;
      sim_write (sd, sp_flat, (void *)env[i], len);
      sim_write (sd, sp, (void *)&sp_flat, sizeof (address_word));
      sp_flat += len;
      sp += sizeof (address_word);
    }
}
