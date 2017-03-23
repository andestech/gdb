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

/* This file contains the main simulator decoding logic.  i.e. everything that
   is architecture specific.  */

/* This must come before any other includes.  */
#include "defs.h"

#include <inttypes.h>
#include <time.h>
#include <unistd.h>
#include "sim/callback.h"
#include <sys/time.h>
#include "sim-main.h"
#include "sim-signal.h"
#include "sim-fpu.h"
#include "sim-syscall.h"

#include "opcode/riscv.h"

#include "gdb/sim-riscv.h"

#define TRACE_REG(cpu, reg) \
  TRACE_REGISTER (cpu, "wrote %s = %#" PRIxTW, riscv_gpr_names_abi[reg], \
		  cpu->regs[reg])

static const struct riscv_opcode *riscv_hash[OP_MASK_OP + 1];
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

static INLINE void
store_rd (SIM_CPU *cpu, int rd, unsigned_word val)
{
  if (rd)
    {
      cpu->regs[rd] = val;
      TRACE_REG (cpu, rd);
    }
}

static INLINE void
store_frd (SIM_CPU *cpu, int rd, unsigned_word val)
{
  cpu->fpregs[rd].w[0] = val;
}

static inline void
store_frd64 (SIM_CPU *cpu, int rd, uint64_t val)
{
  cpu->fpregs[rd].v[0] = val;
}

static INLINE unsigned_word
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

static INLINE void
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
      val = *reg;
      break;
    }

  TRACE_REGISTER (cpu, "wrote CSR %s = %#" PRIxTW, name, val);
}

static inline unsigned_word
ashiftrt (unsigned_word val, unsigned_word shift)
{
  uint32_t sign = (val & 0x80000000) ? ~(0xfffffffful >> shift) : 0;
  return (val >> shift) | sign;
}

static inline unsigned_word
ashiftrt64 (unsigned_word val, unsigned_word shift)
{
  uint64_t sign =
    (val & 0x8000000000000000ull) ? ~(0xffffffffffffffffull >> shift) : 0;
  return (val >> shift) | sign;
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
  if (ex9)
    pc -= 2;

  /* Rounding mode.  */
  int rm = (iw >> OP_SH_RM) & OP_MASK_RM;
  int rounding = round_modes[rm];

  sim_fpu sft, sft2;
  sim_fpu sfa, sfb, sfc;
  sim_fpu_64to (&sfa, cpu->fpregs[rs1].v[0]);
  sim_fpu_64to (&sfb, cpu->fpregs[rs2].v[0]);

  switch (op->match & mask_mul_add)
    {
    case MATCH_FMADD_D:
      sim_fpu_64to (&sfc, cpu->fpregs[rs3].v[0]);
      sim_fpu_mul (&sft2, &sfa, &sfb);
      sim_fpu_add (&sft, &sfc, &sft2);
      sim_fpu_round_64 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to64 (&cpu->fpregs[rd].v[0], &sft);
      goto done;
    case MATCH_FMSUB_D:
      sim_fpu_64to (&sfc, cpu->fpregs[rs3].v[0]);
      sim_fpu_mul (&sft2, &sfa, &sfb);
      sim_fpu_sub (&sft, &sft2, &sfc);
      sim_fpu_round_64 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to64 (&cpu->fpregs[rd].v[0], &sft);
      goto done;
    case MATCH_FNMADD_D:
      sim_fpu_64to (&sfc, cpu->fpregs[rs3].v[0]);
      sim_fpu_mul (&sft2, &sfa, &sfb);
      sim_fpu_add (&sft, &sfc, &sft2);
      sim_fpu_neg (&sft, &sft);
      sim_fpu_round_64 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to64 (&cpu->fpregs[rd].v[0], &sft);
      goto done;
    case MATCH_FNMSUB_D:
      sim_fpu_64to (&sfc, cpu->fpregs[rs3].v[0]);
      sim_fpu_mul (&sft2, &sfa, &sfb);
      sim_fpu_sub (&sft, &sft2, &sfc);
      sim_fpu_neg (&sft, &sft);
      sim_fpu_round_64 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to64 (&cpu->fpregs[rd].v[0], &sft);
      goto done;
    }

  switch (op->match & mask_arithmetic)
    {
    case MATCH_FADD_D:
      sim_fpu_add (&sft, &sfa, &sfb);
      sim_fpu_round_64 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to64 (&cpu->fpregs[rd].v[0], &sft);
      goto done;
    case MATCH_FSUB_D:
      sim_fpu_sub (&sft, &sfa, &sfb);
      sim_fpu_round_64 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to64 (&cpu->fpregs[rd].v[0], &sft);
      goto done;
    case MATCH_FMUL_D:
      sim_fpu_mul (&sft, &sfa, &sfb);
      sim_fpu_round_64 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to64 (&cpu->fpregs[rd].v[0], &sft);
      goto done;
    case MATCH_FDIV_D:
      sim_fpu_div (&sft, &sfa, &sfb);
      sim_fpu_round_64 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to64 (&cpu->fpregs[rd].v[0], &sft);
      goto done;
    case MATCH_FSQRT_D:
      sim_fpu_sqrt (&sft, &sfa);
      sim_fpu_round_64 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to64 (&cpu->fpregs[rd].v[0], &sft);
      goto done;
    }

  switch (op->match & mask_convert)
    {
    case MATCH_FCVT_W_D:
      sim_fpu_to32i (&i32, &sfa, rounding);
      cpu->regs[rd] = i32;
      goto done;
    case MATCH_FCVT_WU_D:
      sim_fpu_to32u (&u32, &sfa, rounding);
      i32 = u32;
      cpu->regs[rd] = i32;
      goto done;
    case MATCH_FCVT_D_W:
      sim_fpu_i32to (&sft, cpu->regs[rs1], rounding);
      sim_fpu_to64 ((unsigned64 *) (cpu->fpregs + rd), &sft);
      goto done;
    case MATCH_FCVT_D_WU:
      sim_fpu_u32to (&sft, cpu->regs[rs1], rounding);
      sim_fpu_to64 ((unsigned64 *) (cpu->fpregs + rd), &sft);
      goto done;
    case MATCH_FCVT_S_D:
      sft = sfa;
      sim_fpu_round_32 (&sft, sim_fpu_round_near, sim_fpu_denorm_default);
      sim_fpu_to32 ((unsigned32 *) (cpu -> fpregs + rd), &sft);
      goto done;
    case MATCH_FCVT_D_S:
      sim_fpu_32to (&sft, cpu->fpregs[rs1].w[0]);
      sim_fpu_to64 (&cpu->fpregs[rd].v[0], &sft);
      goto done;
    case MATCH_FCVT_L_D:
      cpu->regs[rd] = (int64_t) cpu->fpregs[rs1].D[0];
      goto done;
    case MATCH_FCVT_LU_D:
      cpu->regs[rd] = (uint64_t) cpu->fpregs[rs1].D[0];
      goto done;
    case MATCH_FCVT_D_L:
      cpu->fpregs[rd].D[0] = (double) ((int64_t) cpu->regs[rs1]);
      goto done;
    case MATCH_FCVT_D_LU:
      cpu->fpregs[rd].D[0] = (double) cpu->regs[rs1];
      goto done;
    }

  switch (op->match)
    {
    case MATCH_FLD:
      store_frd64 (cpu, rd,
	sim_core_read_unaligned_8 (cpu, cpu->pc, read_map,
				   cpu->regs[rs1] + i_imm));
      break;
    case MATCH_FSD:
      sim_core_write_unaligned_8 (cpu, cpu->pc, write_map,
				  cpu->regs[rs1] + s_imm,
				  cpu->fpregs[rs2].v[0]);
      break;
    case MATCH_FSGNJ_D:
      u32 = cpu->fpregs[rs1].w[1] & 0x7fffffff;
      u32 |= cpu->fpregs[rs2].w[1] & 0x80000000;
      cpu->fpregs[rd].w[1] = u32;
      cpu->fpregs[rd].w[0] = cpu->fpregs[rs1].w[0];
      break;
    case MATCH_FSGNJN_D:
      u32 = cpu->fpregs[rs1].w[1] & 0x7fffffff;
      u32 |= (cpu->fpregs[rs2].w[1] & 0x80000000) ^ 0x80000000;
      cpu->fpregs[rd].w[1] = u32;
      cpu->fpregs[rd].w[0] = cpu->fpregs[rs1].w[0];
      break;
    case MATCH_FSGNJX_D:
      u32 = cpu->fpregs[rs1].w[1] & 0x7fffffff;
      u32 |= (cpu->fpregs[rs1].w[1] & 0x80000000) ^ (cpu->fpregs[rs2].w[1] & 0x80000000);
      cpu->fpregs[rd].w[1] = u32;
      cpu->fpregs[rd].w[0] = cpu->fpregs[rs1].w[0];
      break;
    case MATCH_FMIN_D:
      if (cpu->fpregs[rs1].D[0] < cpu->fpregs[rs2].D[0])
        cpu->fpregs[rd].D[0] = cpu->fpregs[rs1].D[0];
      else
        cpu->fpregs[rd].D[0] = cpu->fpregs[rs2].D[0];
      break;
    case MATCH_FMAX_D:
      if (cpu->fpregs[rs1].D[0] > cpu->fpregs[rs2].D[0])
        cpu->fpregs[rd].D[0] = cpu->fpregs[rs1].D[0];
      else
        cpu->fpregs[rd].D[0] = cpu->fpregs[rs2].D[0];
      break;
    case MATCH_FMV_X_D:
      cpu->regs[rd] = cpu->fpregs[rs1].v[0];
      break;
    case MATCH_FMV_D_X:
      cpu->fpregs[rd].v[0] = cpu->regs[rs1];
      break;
    case MATCH_FEQ_D:
      cpu->regs[rd] = sim_fpu_is_eq (&sfa, &sfb);
      break;
    case MATCH_FLE_D:
      cpu->regs[rd] = sim_fpu_is_le (&sfa, &sfb);
      break;
    case MATCH_FLT_D:
      cpu->regs[rd] = sim_fpu_is_lt (&sfa, &sfb);
      break;
    case MATCH_FCLASS_D:
      switch (sim_fpu_is (&sfa))
	{
	case SIM_FPU_IS_NINF:
	  cpu->regs[rd] = 1;
	  break;
	case SIM_FPU_IS_NNUMBER:
	  cpu->regs[rd] = 1 << 1;
	  break;
	case SIM_FPU_IS_NDENORM:
	  cpu->regs[rd] = 1 << 2;
	  break;
	case SIM_FPU_IS_NZERO:
	  cpu->regs[rd] = 1 << 3;
	  break;
	case SIM_FPU_IS_PZERO:
	  cpu->regs[rd] = 1 << 4;
	  break;
	case SIM_FPU_IS_PDENORM:
	  cpu->regs[rd] = 1 << 5;
	  break;
	case SIM_FPU_IS_PNUMBER:
	  cpu->regs[rd] = 1 << 6;
	  break;
	case SIM_FPU_IS_PINF:
	  cpu->regs[rd] = 1 << 7;
	  break;
	case SIM_FPU_IS_SNAN:
	  cpu->regs[rd] = 1 << 8;
	  break;
	case SIM_FPU_IS_QNAN:
	  cpu->regs[rd] = 1 << 9;
	  break;
	}
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
  if (ex9)
    pc -= 2;

  /* Rounding mode.  */
  int rm = (iw >> OP_SH_RM) & OP_MASK_RM;
  int rounding = round_modes[rm];

  sim_fpu sft, sft2;
  sim_fpu sfa, sfb, sfc;
  sim_fpu_32to (&sfa, cpu->fpregs[rs1].w[0]);
  sim_fpu_32to (&sfb, cpu->fpregs[rs2].w[0]);

  switch (op->match & mask_mul_add)
    {
    case MATCH_FMADD_S:
      sim_fpu_32to (&sfc, cpu->fpregs[rs3].w[0]);
      sim_fpu_mul (&sft2, &sfa, &sfb);
      sim_fpu_add (&sft, &sfc, &sft2);
      sim_fpu_round_32 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to32 (&cpu->fpregs[rd].w[0], &sft);
      goto done;
    case MATCH_FMSUB_S:
      sim_fpu_32to (&sfc, cpu->fpregs[rs3].w[0]);
      sim_fpu_mul (&sft2, &sfa, &sfb);
      sim_fpu_sub (&sft, &sft2, &sfc);
      sim_fpu_round_32 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to32 (&cpu->fpregs[rd].w[0], &sft);
      goto done;
    case MATCH_FNMADD_S:
      sim_fpu_32to (&sfc, cpu->fpregs[rs3].w[0]);
      sim_fpu_mul (&sft2, &sfa, &sfb);
      sim_fpu_add (&sft, &sfc, &sft2);
      sim_fpu_neg (&sft, &sft);
      sim_fpu_round_32 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to32 (&cpu->fpregs[rd].w[0], &sft);
      goto done;
    case MATCH_FNMSUB_S:
      sim_fpu_32to (&sfc, cpu->fpregs[rs3].w[0]);
      sim_fpu_mul (&sft2, &sfa, &sfb);
      sim_fpu_sub (&sft, &sft2, &sfc);
      sim_fpu_neg (&sft, &sft);
      sim_fpu_round_32 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to32 (&cpu->fpregs[rd].w[0], &sft);
      goto done;
    }

  switch (op->match & mask_arithmetic)
    {
    case MATCH_FADD_S:
      sim_fpu_add (&sft, &sfa, &sfb);
      sim_fpu_round_32 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to32 (&cpu->fpregs[rd].w[0], &sft);
      goto done;
    case MATCH_FSUB_S:
      sim_fpu_sub (&sft, &sfa, &sfb);
      sim_fpu_round_32 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to32 (&cpu->fpregs[rd].w[0], &sft);
      goto done;
    case MATCH_FMUL_S:
      sim_fpu_mul (&sft, &sfa, &sfb);
      sim_fpu_round_64 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_round_32 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to32 (&cpu->fpregs[rd].w[0], &sft);
      goto done;
    case MATCH_FDIV_S:
      sim_fpu_div (&sft, &sfa, &sfb);
      sim_fpu_round_32 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to32 (&cpu->fpregs[rd].w[0], &sft);
      goto done;
    case MATCH_FSQRT_S:
      sim_fpu_sqrt (&sft, &sfa);
      sim_fpu_to32 (&cpu->fpregs[rd].w[0], &sft);
      goto done;
    }

  switch (op->match & mask_convert)
    {
    case MATCH_FCVT_W_S:
      sim_fpu_to32i (&i32, &sfa, rounding);
      cpu->regs[rd] = i32;
      goto done;
    case MATCH_FCVT_WU_S:
      sim_fpu_to32u (&u32, &sfa, rounding);
      i32 = u32;
      cpu->regs[rd] = i32;
      goto done;
    case MATCH_FCVT_S_W:
      sim_fpu_i32to (&sft, cpu->regs[rs1], rounding);
      sim_fpu_round_32 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to32 ((unsigned32 *) (cpu->fpregs + rd), &sft);
      goto done;
    case MATCH_FCVT_S_WU:
      sim_fpu_u32to (&sft, cpu->regs[rs1], rounding);
      sim_fpu_round_32 (&sft, rounding, sim_fpu_denorm_default);
      sim_fpu_to32 ((unsigned32 *) (cpu->fpregs + rd), &sft);
      goto done;
    case MATCH_FCVT_L_S:
      cpu->regs[rd] = (int64_t) cpu->fpregs[rs1].S[0];
      goto done;
    case MATCH_FCVT_LU_S:
      cpu->regs[rd] = (uint64_t) cpu->fpregs[rs1].S[0];
      goto done;
    case MATCH_FCVT_S_L:
      cpu->fpregs[rd].S[0] = (float) ((int64_t) cpu->regs[rs1]);
      goto done;
    case MATCH_FCVT_S_LU:
      cpu->fpregs[rd].S[0] = (float) cpu->regs[rs1];
      goto done;
    }

  switch (op->match)
    {
    case MATCH_FLW:
      store_frd (cpu, rd, EXTEND32 (
	sim_core_read_unaligned_4 (cpu, cpu->pc, read_map,
				   cpu->regs[rs1] + i_imm)));
      break;
    case MATCH_FSW:
      sim_core_write_unaligned_4 (cpu, cpu->pc, write_map,
				  cpu->regs[rs1] + s_imm, cpu->fpregs[rs2].w[0]);
      break;
    case MATCH_FSGNJ_S:
      u32 = cpu->fpregs[rs1].w[0] & 0x7fffffff;
      u32 |= cpu->fpregs[rs2].w[0] & 0x80000000;
      cpu->fpregs[rd].w[0] = u32;
      break;
    case MATCH_FSGNJN_S:
      u32 = cpu->fpregs[rs1].w[0] & 0x7fffffff;
      u32 |= (cpu->fpregs[rs2].w[0] & 0x80000000) ^ 0x80000000;
      cpu->fpregs[rd].w[0] = u32;
      break;
    case MATCH_FSGNJX_S:
      u32 = cpu->fpregs[rs1].w[0] & 0x7fffffff;
      u32 |= (cpu->fpregs[rs1].w[0] & 0x80000000) ^ (cpu->fpregs[rs2].w[0] & 0x80000000);
      cpu->fpregs[rd].w[0] = u32;
      break;
    case MATCH_FMIN_S:
      if (cpu->fpregs[rs1].S[0] < cpu->fpregs[rs2].S[0])
        cpu->fpregs[rd].S[0] = cpu->fpregs[rs1].S[0];
      else
        cpu->fpregs[rd].S[0] = cpu->fpregs[rs2].S[0];
      break;
    case MATCH_FMAX_S:
      if (cpu->fpregs[rs1].S[0] > cpu->fpregs[rs2].S[0])
        cpu->fpregs[rd].S[0] = cpu->fpregs[rs1].S[0];
      else
        cpu->fpregs[rd].S[0] = cpu->fpregs[rs2].S[0];
      break;
    case MATCH_FMV_X_S:
      cpu->regs[rd] = cpu->fpregs[rs1].W[0];
      break;
    case MATCH_FMV_S_X:
      cpu->fpregs[rd].w[0] = cpu->regs[rs1];
      break;
    case MATCH_FEQ_S:
      cpu->regs[rd] = sim_fpu_is_eq (&sfa, &sfb);
      break;
    case MATCH_FLE_S:
      cpu->regs[rd] = sim_fpu_is_le (&sfa, &sfb);
      break;
    case MATCH_FLT_S:
      cpu->regs[rd] = sim_fpu_is_lt (&sfa, &sfb);
      break;
    case MATCH_FCLASS_S:
      switch (sim_fpu_is (&sfa))
	{
	case SIM_FPU_IS_NINF:
	  cpu->regs[rd] = 1;
	  break;
	case SIM_FPU_IS_NNUMBER:
	  cpu->regs[rd] = 1 << 1;
	  break;
	case SIM_FPU_IS_NDENORM:
	  cpu->regs[rd] = 1 << 2;
	  break;
	case SIM_FPU_IS_NZERO:
	  cpu->regs[rd] = 1 << 3;
	  break;
	case SIM_FPU_IS_PZERO:
	  cpu->regs[rd] = 1 << 4;
	  break;
	case SIM_FPU_IS_PDENORM:
	  cpu->regs[rd] = 1 << 5;
	  break;
	case SIM_FPU_IS_PNUMBER:
	  cpu->regs[rd] = 1 << 6;
	  break;
	case SIM_FPU_IS_PINF:
	  cpu->regs[rd] = 1 << 7;
	  break;
	case SIM_FPU_IS_SNAN:
	  cpu->regs[rd] = 1 << 8;
	  break;
	case SIM_FPU_IS_QNAN:
	  cpu->regs[rd] = 1 << 9;
	  break;
	}
      break;
    case MATCH_FRCSR:
      store_rd (cpu, rd, fetch_csr (cpu, "fcsr", CSR_FCSR, &cpu->csr.fcsr));
      break;
    case MATCH_FSCSR:
      store_rd (cpu, rd, fetch_csr (cpu, "fcsr", CSR_FCSR, &cpu->csr.fcsr));
      store_csr (cpu, "fcsr", CSR_FCSR, &cpu->csr.fcsr, cpu->regs[rs1]);
      break;
    case MATCH_FRRM:
      store_rd (cpu, rd, fetch_csr (cpu, "frm", CSR_FRM, &cpu->csr.frm));
      break;
    case MATCH_FSRM:
      store_rd (cpu, rd, fetch_csr (cpu, "frm", CSR_FCSR, &cpu->csr.frm));
      store_csr (cpu, "frm", CSR_FCSR, &cpu->csr.frm, cpu->regs[rs1]);
      break;
    case MATCH_FRFLAGS:
      store_rd (cpu, rd, fetch_csr (cpu, "fflags", CSR_FFLAGS, &cpu->csr.fflags));
      break;
    case MATCH_FSFLAGS:
      store_rd (cpu, rd, fetch_csr (cpu, "fflags", CSR_FFLAGS, &cpu->csr.fflags));
      store_csr (cpu, "fflags", CSR_FFLAGS, &cpu->csr.fflags, cpu->regs[rs1]);
      break;
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
  unsigned_word rvc_imm = EXTRACT_CITYPE_IMM (iw);
  unsigned_word tmp;
  int eh_rve_p = cpu->elf_flags & 0x8;
  sim_cia pc = cpu->pc + 2;

  /* Deal with c.mv, c.jr instructons.  */
  if ((op->match & mask_mv_jr) == match_mv_jr)
    {
      if (crs2 != 0)
	{
	  /* c.mv */
	  cpu->regs[rd] = cpu->regs[crs2];
	}
      else
	{
	  /* c.jr */
	  pc = cpu->regs[rd];
	}
      return pc;
    }

  /* Deal with c.ebreak, c.jalr, c.add instructions.  */
  if ((op->match & mask_ebk_jalr_add) == match_ebk_jalr_add)
    {
      if (iw == MATCH_C_EBREAK)
	{
	  /* c.ebreak */
	  sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_stopped, SIM_SIGTRAP);
	}
      else if (crs2 == 0)
	{
	  /* c.jalr */
	  pc = cpu->regs[rd];
	  store_rd (cpu, X_RA, cpu->pc + 2);
	}
      else
	{
	  /* c.add */
	  store_rd (cpu, rd, cpu->regs[rd] + cpu->regs[crs2]);
	}
      return pc;
    }

  switch (op->match & mask_group_op)
    {
    case 0:
      switch (op->match)
	{
	case MATCH_C_LW:
	  store_rd (cpu, crs2s, EXTEND32 (
	    sim_core_read_unaligned_4 (cpu, cpu->pc, read_map,
				       cpu->regs[crs1s]
				       + EXTRACT_CLTYPE_LW_IMM (iw))));
	  return pc;
	case MATCH_C_SW:
	  sim_core_write_unaligned_4 (cpu, cpu->pc, write_map,
				      (cpu->regs[crs1s]
				       + EXTRACT_CLTYPE_LW_IMM (iw)),
				      cpu->regs[crs2s]);
	  return pc;
	case MATCH_C_ADDI4SPN:
	  store_rd (cpu, ciw_rd, cpu->sp + EXTRACT_CIWTYPE_ADDI4SPN_IMM (iw));
	  return pc;
	case MATCH_C_FLD:
	  if (RISCV_XLEN (cpu) <= 64)
	    {
	      /* rv32/64, c.fld instruction.  */
	      store_frd64 (cpu, crs2s,
		sim_core_read_unaligned_8 (cpu, cpu->pc, read_map,
					   cpu->regs[crs1s]
					   + EXTRACT_CLTYPE_LD_IMM (iw)));
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
	    store_frd (cpu, crs2s, EXTEND32 (
	      sim_core_read_unaligned_4 (cpu, cpu->pc, read_map,
					 cpu->regs[crs1s]
					 + EXTRACT_CLTYPE_LW_IMM (iw))));
	  else
	    store_rd (cpu, crs2s,
	      sim_core_read_unaligned_8 (cpu, cpu->pc, read_map,
					 cpu->regs[crs1s]
					 + EXTRACT_CLTYPE_LD_IMM (iw)));
	  return pc;
	case MATCH_C_FSD:
	  if (RISCV_XLEN (cpu) <= 64)
	    {
	      /* rv32/64, c.fsd instruction.  */
	      sim_core_write_unaligned_8 (cpu, cpu->pc, write_map,
					  cpu->regs[crs1s]
					  + EXTRACT_CLTYPE_LD_IMM (iw),
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
	    sim_core_write_unaligned_4 (cpu, cpu->pc, write_map,
					cpu->regs[crs1s]
					+ EXTRACT_CLTYPE_LW_IMM (iw),
					cpu->fpregs[crs2s].w[0]);
	  else
	    sim_core_write_unaligned_8 (cpu, cpu->pc, write_map,
					cpu->regs[crs1s]
					+ EXTRACT_CLTYPE_LD_IMM (iw),
					cpu->regs[crs2s]);
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
              if (!eh_rve_p && rd == 2 && ((cpu->regs[rd] + rvc_imm) & 0xf) != 0)
                {
                  fprintf (stderr, "Stack pointer is not aligned to 16-byte boundary.\n");
                  sim_engine_halt (sd, cpu, NULL, cpu->pc,
            	         	   sim_signalled, SIM_SIGILL);
                }
	      store_rd (cpu, rd, cpu->regs[rd] + rvc_imm);
	      return pc;
	    }
	  else
	    {
	      /* c.nop */
	      return pc;
	    }
	case MATCH_C_JAL:
	  /* In rv32 is c.jal, rv64 c.addiw.  */
	  if (RISCV_XLEN (cpu) == 32)
	    {
	      store_rd (cpu, X_RA, cpu->pc + 2);
	      pc = cpu->pc + EXTRACT_CJTYPE_IMM (iw);
	    }
	  else
	    store_rd (cpu, rd, EXTEND32 (cpu->regs[rd] + rvc_imm));
	  return pc;
	case MATCH_C_LI:
	  store_rd (cpu, rd, rvc_imm);
	  return pc;
	case MATCH_C_ADDI16SP:
	  {
	    if (!eh_rve_p &&(cpu->sp & 0xf) != 0)
	      {
		fprintf (stderr, "Stack point not align to 16 byte.\n");
		sim_engine_halt (sd, cpu, NULL, cpu->pc,
				 sim_signalled, SIM_SIGILL);
	      }
	  }
	  store_rd (cpu, rd, cpu->sp + EXTRACT_CITYPE_ADDI16SP_IMM (iw));
	  return pc;
	case MATCH_C_SRLI:
	  /* rv32: c.srli, rv128: c.srli64.  */
	  if (RISCV_XLEN (cpu) == 32 && EXTRACT_CITYPE_IMM (iw) > 0x1f)
	    sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
	  store_rd (cpu, crs1s, cpu->regs[crs1s] >> EXTRACT_CITYPE_IMM (iw));
	  return pc;
	case MATCH_C_SRAI:
	  /* rv32: c.srli, rv128: c.srli64.  */
	  if (RISCV_XLEN (cpu) == 32)
	    {
	      if (EXTRACT_CITYPE_IMM (iw) > 0x1f)
		sim_engine_halt (sd, cpu, NULL, cpu->pc,
				 sim_signalled, SIM_SIGILL);
	      tmp = ashiftrt (cpu->regs[crs1s], EXTRACT_CITYPE_IMM (iw));
	    }
	  else
	    tmp = ashiftrt64 (cpu->regs[crs1s], EXTRACT_CITYPE_IMM (iw));
	  store_rd (cpu, crs1s, tmp);
	  return pc;
	case MATCH_C_ANDI:
	  store_rd (cpu, crs1s, cpu->regs[crs1s] & EXTRACT_CITYPE_IMM (iw));
	  return pc;
	case MATCH_C_SUB:
 	  store_rd (cpu, crs1s, cpu->regs[crs1s] - cpu->regs[crs2s]);
	  return pc;
	case MATCH_C_XOR:
	  store_rd (cpu, crs1s, cpu->regs[crs1s] ^ cpu->regs[crs2s]);
	  return pc;
	case MATCH_C_OR:
	  store_rd (cpu, crs1s, cpu->regs[crs1s] | cpu->regs[crs2s]);
	  return pc;
	case MATCH_C_AND:
	  store_rd (cpu, crs1s, cpu->regs[crs1s] & cpu->regs[crs2s]);
	  return pc;
	case MATCH_C_SUBW:
	  RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
	  store_rd (cpu, crs1s, EXTEND32 (cpu->regs[crs1s] - cpu->regs[crs2s]));
	  return pc;
	case MATCH_C_ADDW:
	  RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
	  store_rd (cpu, crs1s, EXTEND32 (cpu->regs[crs1s] + cpu->regs[crs2s]));
	  return pc;
	case MATCH_C_BEQZ:
	  if (cpu->regs[crs1s] == 0)
	    pc = cpu->pc + EXTRACT_CBTYPE_IMM (iw);
	  return pc;
	case MATCH_C_BNEZ:
	  if (cpu->regs[crs1s] != 0)
	    pc = cpu->pc + EXTRACT_CBTYPE_IMM (iw);
	  return pc;
	case MATCH_C_LUI:
	  store_rd (cpu, rd, EXTRACT_CITYPE_LUI_IMM (iw));
	  return pc;
	case MATCH_C_J:
	  pc = cpu->pc + EXTRACT_CJTYPE_IMM (iw);
	  return pc;
	default:
	  TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
	  sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
	}
    case 2:
      switch (op->match)
	{
	case MATCH_C_SLLI:
	  /* rv32: c.slli, rv128: c.slli64.  */
	  if (RISCV_XLEN (cpu) == 32 && rvc_imm > 0x1f)
	    sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
	  store_rd (cpu, rd, cpu->regs[rd] << rvc_imm);
	  return pc;
	case MATCH_C_LWSP:
	  store_rd (cpu, rd, EXTEND32 (
	    sim_core_read_unaligned_4 (cpu, cpu->pc, read_map,
				       cpu->sp
				       + EXTRACT_CITYPE_LWSP_IMM (iw))));
	  return pc;
	case MATCH_C_SWSP:
	  sim_core_write_unaligned_4 (cpu, cpu->pc, write_map,
				      (cpu->sp + EXTRACT_CSSTYPE_SWSP_IMM (iw)),
				      cpu->regs[crs2]);
	  return pc;
	case MATCH_C_ADD:
	  store_rd (cpu, rd, cpu->regs[rd] + cpu->regs[crs2]);
	  return pc;
	case MATCH_C_FLDSP:
	  /* rv32/64: c.fldsp, rv128: c.flqsp.  */
	  if (RISCV_XLEN (cpu) <= 64)
	    {
	      store_frd64 (cpu, rd,
		sim_core_read_unaligned_8 (cpu, cpu->pc, read_map,
					   cpu->sp
					   + EXTRACT_CITYPE_LDSP_IMM (iw)));
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
	    store_frd (cpu, rd, EXTEND32 (
	      sim_core_read_unaligned_4 (cpu, cpu->pc, read_map,
					 cpu->sp
					 + EXTRACT_CITYPE_LWSP_IMM (iw))));
	  else
	    store_rd (cpu, rd,
	      sim_core_read_unaligned_8 (cpu, cpu->pc, read_map,
					 cpu->sp
					 + EXTRACT_CITYPE_LDSP_IMM (iw)));
	  return pc;
	case MATCH_C_FSDSP:
	  /* rv32/64: c.fsdsp, rv128: c.fsqsp.  */
	  if (RISCV_XLEN (cpu) <= 64)
	    {
	      sim_core_write_unaligned_8 (cpu, cpu->pc, write_map,
					  cpu->sp + EXTRACT_CSSTYPE_SDSP_IMM (iw),
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
	    sim_core_write_unaligned_4 (cpu, cpu->pc, write_map,
					cpu->sp
					+ EXTRACT_CSSTYPE_SWSP_IMM (iw),
					cpu->fpregs[crs2].w[0]);
	  else
	    sim_core_write_unaligned_8 (cpu, cpu->pc, write_map,
					cpu->sp + EXTRACT_CSSTYPE_SDSP_IMM (iw),
					cpu->regs[crs2]);
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
execute_i (SIM_CPU *cpu, unsigned_word iw, const struct riscv_opcode *op, int ex9)
{
  SIM_DESC sd = CPU_STATE (cpu);
  int rd = (iw >> OP_SH_RD) & OP_MASK_RD;
  int rs1 = (iw >> OP_SH_RS1) & OP_MASK_RS1;
  int rs2 = (iw >> OP_SH_RS2) & OP_MASK_RS2;
  const char *rd_name = riscv_gpr_names_abi[rd];
  const char *rs1_name = riscv_gpr_names_abi[rs1];
  const char *rs2_name = riscv_gpr_names_abi[rs2];
  unsigned int csr = (iw >> OP_SH_CSR) & OP_MASK_CSR;
  unsigned_word i_imm = EXTRACT_ITYPE_IMM (iw);
  unsigned_word u_imm = EXTRACT_UTYPE_IMM ((uint64_t) iw);
  unsigned_word s_imm = EXTRACT_STYPE_IMM (iw);
  unsigned_word sb_imm = EXTRACT_BTYPE_IMM (iw);
  unsigned_word shamt_imm = ((iw >> OP_SH_SHAMT) & OP_MASK_SHAMT);
  unsigned_word tmp;
  unsigned_word sys_id;
  int eh_rve_p = cpu->elf_flags & 0x8;
  sim_cia pc = cpu->pc + 4;
  if (ex9)
    pc -= 2;

  host_callback *cb = STATE_CALLBACK (sd);
  CB_SYSCALL sc;

  CB_SYSCALL_INIT (&sc);

  if (eh_rve_p)
    sc.func = cpu->t0;
  else
    sc.func = cpu->a7;

  sc.arg1 = cpu->a0;
  sc.arg2 = cpu->a1;
  sc.arg3 = cpu->a2;
  sc.arg4 = cpu->a3;

  sc.p1 = (PTR) sd;
  sc.p2 = (PTR) cpu;
  sc.read_mem = sim_syscall_read_mem;
  sc.write_mem = sim_syscall_write_mem;

  TRACE_EXTRACT (cpu,
		 "rd:%-2i:%-4s  "
		 "rs1:%-2i:%-4s %0*" PRIxTW "  "
		 "rs2:%-2i:%-4s %0*" PRIxTW "  "
		 "match:%#x mask:%#x",
		 rd, rd_name,
		 rs1, rs1_name, (int) sizeof (unsigned_word) * 2, cpu->regs[rs1],
		 rs2, rs2_name, (int) sizeof (unsigned_word) * 2, cpu->regs[rs2],
		 (unsigned) op->match, (unsigned) op->mask);

  switch (op->match)
    {
    case MATCH_ADD:
      TRACE_INSN (cpu, "add %s, %s, %s;  // %s = %s + %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      store_rd (cpu, rd, cpu->regs[rs1] + cpu->regs[rs2]);
      break;
    case MATCH_ADDW:
      TRACE_INSN (cpu, "addw %s, %s, %s;  // %s = %s + %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
      store_rd (cpu, rd, EXTEND32 (cpu->regs[rs1] + cpu->regs[rs2]));
      break;
    case MATCH_ADDI:
      if (!eh_rve_p && rd == 2 && ((cpu->regs[rs1] + i_imm) & 0xf) != 0)
        {
          fprintf (stderr, "Stack pointer is not aligned to 16-byte boundary.\n");
          sim_engine_halt (sd, cpu, NULL, cpu->pc,
      		           sim_signalled, SIM_SIGILL);
        }

      TRACE_INSN (cpu, "addi %s, %s, %#" PRIxTW ";  // %s = %s + %#"PRIxTW,
		  rd_name, rs1_name, i_imm, rd_name, rs1_name, i_imm);
      store_rd (cpu, rd, cpu->regs[rs1] + i_imm);
      break;
    case MATCH_ADDIW:
      TRACE_INSN (cpu, "addiw %s, %s, %#" PRIxTW ";  // %s = %s + %#" PRIxTW,
		  rd_name, rs1_name, i_imm, rd_name, rs1_name, i_imm);
      RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
      store_rd (cpu, rd, EXTEND32 (cpu->regs[rs1] + i_imm));
      break;
    case MATCH_AND:
      TRACE_INSN (cpu, "and %s, %s, %s;  // %s = %s & %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      store_rd (cpu, rd, cpu->regs[rs1] & cpu->regs[rs2]);
      break;
    case MATCH_ANDI:
      TRACE_INSN (cpu, "andi %s, %s, %" PRIiTW ";  // %s = %s & %#" PRIxTW,
		  rd_name, rs1_name, i_imm, rd_name, rs1_name, i_imm);
      store_rd (cpu, rd, cpu->regs[rs1] & i_imm);
      break;
    case MATCH_OR:
      TRACE_INSN (cpu, "or %s, %s, %s;  // %s = %s | %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      store_rd (cpu, rd, cpu->regs[rs1] | cpu->regs[rs2]);
      break;
    case MATCH_ORI:
      TRACE_INSN (cpu, "ori %s, %s, %" PRIiTW ";  // %s = %s | %#" PRIxTW,
		  rd_name, rs1_name, i_imm, rd_name, rs1_name, i_imm);
      store_rd (cpu, rd, cpu->regs[rs1] | i_imm);
      break;
    case MATCH_XOR:
      TRACE_INSN (cpu, "xor %s, %s, %s;  // %s = %s ^ %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      store_rd (cpu, rd, cpu->regs[rs1] ^ cpu->regs[rs2]);
      break;
    case MATCH_XORI:
      TRACE_INSN (cpu, "xori %s, %s, %" PRIiTW ";  // %s = %s ^ %#" PRIxTW,
		  rd_name, rs1_name, i_imm, rd_name, rs1_name, i_imm);
      store_rd (cpu, rd, cpu->regs[rs1] ^ i_imm);
      break;
    case MATCH_SUB:
      TRACE_INSN (cpu, "sub %s, %s, %s;  // %s = %s - %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      store_rd (cpu, rd, cpu->regs[rs1] - cpu->regs[rs2]);
      break;
    case MATCH_SUBW:
      TRACE_INSN (cpu, "subw %s, %s, %s;  // %s = %s - %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
      store_rd (cpu, rd, EXTEND32 (cpu->regs[rs1] - cpu->regs[rs2]));
      break;
    case MATCH_LUI:
      TRACE_INSN (cpu, "lui %s, %#" PRIxTW ";", rd_name, u_imm);
      store_rd (cpu, rd, u_imm);
      break;
    case MATCH_SLL:
      TRACE_INSN (cpu, "sll %s, %s, %s;  // %s = %s << %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      u_imm = RISCV_XLEN (cpu) == 32 ? 0x1f : 0x3f;
      store_rd (cpu, rd, cpu->regs[rs1] << (cpu->regs[rs2] & u_imm));
      break;
    case MATCH_SLLW:
      TRACE_INSN (cpu, "sllw %s, %s, %s;  // %s = %s << %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
      store_rd (cpu, rd, EXTEND32 (
	(uint32_t) cpu->regs[rs1] << (cpu->regs[rs2] & 0x1f)));
      break;
    case MATCH_SLLI:
      TRACE_INSN (cpu, "slli %s, %s, %" PRIiTW ";  // %s = %s << %#" PRIxTW,
		  rd_name, rs1_name, shamt_imm, rd_name, rs1_name, shamt_imm);
      if (RISCV_XLEN (cpu) == 32 && shamt_imm > 0x1f)
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
      store_rd (cpu, rd, cpu->regs[rs1] << shamt_imm);
      break;
    case MATCH_SLLIW:
      TRACE_INSN (cpu, "slliw %s, %s, %" PRIiTW ";  // %s = %s << %#" PRIxTW,
		  rd_name, rs1_name, shamt_imm, rd_name, rs1_name, shamt_imm);
      RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
      store_rd (cpu, rd, EXTEND32 ((uint32_t) cpu->regs[rs1] << shamt_imm));
      break;
    case MATCH_SRL:
      TRACE_INSN (cpu, "srl %s, %s, %s;  // %s = %s >> %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      u_imm = RISCV_XLEN (cpu) == 32 ? 0x1f : 0x3f;
      store_rd (cpu, rd, cpu->regs[rs1] >> (cpu->regs[rs2] & u_imm));
      break;
    case MATCH_SRLW:
      TRACE_INSN (cpu, "srlw %s, %s, %s;  // %s = %s >> %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
      store_rd (cpu, rd, EXTEND32 (
	(uint32_t) cpu->regs[rs1] >> (cpu->regs[rs2] & 0x1f)));
      break;
    case MATCH_SRLI:
      TRACE_INSN (cpu, "srli %s, %s, %" PRIiTW ";  // %s = %s >> %#" PRIxTW,
		  rd_name, rs1_name, shamt_imm, rd_name, rs1_name, shamt_imm);
      if (RISCV_XLEN (cpu) == 32 && shamt_imm > 0x1f)
	sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
      store_rd (cpu, rd, cpu->regs[rs1] >> shamt_imm);
      break;
    case MATCH_SRLIW:
      TRACE_INSN (cpu, "srliw %s, %s, %" PRIiTW ";  // %s = %s >> %#" PRIxTW,
		  rd_name, rs1_name, shamt_imm, rd_name, rs1_name, shamt_imm);
      RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
      store_rd (cpu, rd, EXTEND32 ((uint32_t) cpu->regs[rs1] >> shamt_imm));
      break;
    case MATCH_SRA:
      TRACE_INSN (cpu, "sra %s, %s, %s;  // %s = %s >>> %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      if (RISCV_XLEN (cpu) == 32)
	tmp = ashiftrt (cpu->regs[rs1], cpu->regs[rs2] & 0x1f);
      else
	tmp = ashiftrt64 (cpu->regs[rs1], cpu->regs[rs2] & 0x3f);
      store_rd (cpu, rd, tmp);
      break;
    case MATCH_SRAW:
      TRACE_INSN (cpu, "sraw %s, %s, %s;  // %s = %s >>> %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
      store_rd (cpu, rd, EXTEND32 (
	ashiftrt ((int32_t) cpu->regs[rs1], cpu->regs[rs2] & 0x1f)));
      break;
    case MATCH_SRAI:
      TRACE_INSN (cpu, "srai %s, %s, %" PRIiTW ";  // %s = %s >>> %#" PRIxTW,
		  rd_name, rs1_name, shamt_imm, rd_name, rs1_name, shamt_imm);
      if (RISCV_XLEN (cpu) == 32)
	{
	  if (shamt_imm > 0x1f)
	    sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
	  tmp = ashiftrt (cpu->regs[rs1], shamt_imm);
	}
      else
	tmp = ashiftrt64 (cpu->regs[rs1], shamt_imm);
      store_rd (cpu, rd, tmp);
      break;
    case MATCH_SRAIW:
      TRACE_INSN (cpu, "sraiw %s, %s, %" PRIiTW ";  // %s = %s >>> %#" PRIxTW,
		  rd_name, rs1_name, shamt_imm, rd_name, rs1_name, shamt_imm);
      RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
      store_rd (cpu, rd, EXTEND32 (
	ashiftrt ((int32_t) cpu->regs[rs1], shamt_imm)));
      break;
    case MATCH_SLT:
      TRACE_INSN (cpu, "slt");
      store_rd (cpu, rd,
		!!((signed_word) cpu->regs[rs1] < (signed_word) cpu->regs[rs2]));
      break;
    case MATCH_SLTU:
      TRACE_INSN (cpu, "sltu");
      store_rd (cpu, rd, !!((unsigned_word) cpu->regs[rs1] <
			    (unsigned_word) cpu->regs[rs2]));
      break;
    case MATCH_SLTI:
      TRACE_INSN (cpu, "slti");
      store_rd (cpu, rd, !!((signed_word) cpu->regs[rs1] <
			    (signed_word) i_imm));
      break;
    case MATCH_SLTIU:
      TRACE_INSN (cpu, "sltiu");
      store_rd (cpu, rd, !!((unsigned_word) cpu->regs[rs1] <
			    (unsigned_word) i_imm));
      break;
    case MATCH_AUIPC:
      TRACE_INSN (cpu, "auipc %s, %" PRIiTW ";  // %s = pc + %" PRIiTW,
		  rd_name, u_imm, rd_name, u_imm);
      store_rd (cpu, rd, cpu->pc + u_imm);
      break;
    case MATCH_BEQ:
      TRACE_INSN (cpu, "beq %s, %s, %#" PRIxTW ";  "
		       "// if (%s == %s) goto %#" PRIxTW,
		  rs1_name, rs2_name, sb_imm, rs1_name, rs2_name, sb_imm);
      if (cpu->regs[rs1] == cpu->regs[rs2])
	{
	  pc = cpu->pc + sb_imm;
	  TRACE_BRANCH (cpu, "to %#" PRIxTW, pc);
	}
      break;
    case MATCH_BLT:
      TRACE_INSN (cpu, "blt %s, %s, %#" PRIxTW ";  "
		       "// if (%s < %s) goto %#" PRIxTW,
		  rs1_name, rs2_name, sb_imm, rs1_name, rs2_name, sb_imm);
      if ((signed_word) cpu->regs[rs1] < (signed_word) cpu->regs[rs2])
	{
	  pc = cpu->pc + sb_imm;
	  TRACE_BRANCH (cpu, "to %#" PRIxTW, pc);
	}
      break;
    case MATCH_BLTU:
      TRACE_INSN (cpu, "bltu %s, %s, %#" PRIxTW ";  "
		       "// if (%s < %s) goto %#" PRIxTW,
		  rs1_name, rs2_name, sb_imm, rs1_name, rs2_name, sb_imm);
      if ((unsigned_word) cpu->regs[rs1] < (unsigned_word) cpu->regs[rs2])
	{
	  pc = cpu->pc + sb_imm;
	  TRACE_BRANCH (cpu, "to %#" PRIxTW, pc);
	}
      break;
    case MATCH_BGE:
      TRACE_INSN (cpu, "bge %s, %s, %#" PRIxTW ";  "
		       "// if (%s >= %s) goto %#" PRIxTW,
		  rs1_name, rs2_name, sb_imm, rs1_name, rs2_name, sb_imm);
      if ((signed_word) cpu->regs[rs1] >= (signed_word) cpu->regs[rs2])
	{
	  pc = cpu->pc + sb_imm;
	  TRACE_BRANCH (cpu, "to %#" PRIxTW, pc);
	}
      break;
    case MATCH_BGEU:
      TRACE_INSN (cpu, "bgeu %s, %s, %#" PRIxTW ";  "
		       "// if (%s >= %s) goto %#" PRIxTW,
		  rs1_name, rs2_name, sb_imm, rs1_name, rs2_name, sb_imm);
      if ((unsigned_word) cpu->regs[rs1] >= (unsigned_word) cpu->regs[rs2])
	{
	  pc = cpu->pc + sb_imm;
	  TRACE_BRANCH (cpu, "to %#" PRIxTW, pc);
	}
      break;
    case MATCH_BNE:
      TRACE_INSN (cpu, "bne %s, %s, %#" PRIxTW ";  "
		       "// if (%s != %s) goto %#" PRIxTW,
		  rs1_name, rs2_name, sb_imm, rs1_name, rs2_name, sb_imm);
      if (cpu->regs[rs1] != cpu->regs[rs2])
	{
	  pc = cpu->pc + sb_imm;
	  TRACE_BRANCH (cpu, "to %#" PRIxTW, pc);
	}
      break;
    case MATCH_JAL:
      if (ex9)
      {
          store_rd (cpu, rd, cpu->pc + 2);
          pc = (cpu->pc & 0xfff00000) | EXTRACT_UJTYPE_IMM_EXECIT_TAB (iw);
      }
      else
      {
          TRACE_INSN (cpu, "jal %s, %" PRIiTW ";", rd_name,
		      EXTRACT_JTYPE_IMM (iw));
          store_rd (cpu, rd, cpu->pc + 4);
          pc = cpu->pc + EXTRACT_JTYPE_IMM (iw);
          TRACE_BRANCH (cpu, "to %#" PRIxTW, pc);
      }
      break;
    case MATCH_JALR:
      TRACE_INSN (cpu, "jalr %s, %s, %" PRIiTW ";", rd_name, rs1_name, i_imm);
      pc = cpu->regs[rs1] + i_imm;
      if (ex9)
          store_rd (cpu, rd, cpu->pc + 2);
      else
          store_rd (cpu, rd, cpu->pc + 4);
      TRACE_BRANCH (cpu, "to %#" PRIxTW, pc);
      break;

    case MATCH_LD:
      TRACE_INSN (cpu, "ld %s, %" PRIiTW "(%s);",
		  rd_name, i_imm, rs1_name);
      RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
      store_rd (cpu, rd,
	sim_core_read_unaligned_8 (cpu, cpu->pc, read_map,
				   cpu->regs[rs1] + i_imm));
      break;
    case MATCH_LW:
      TRACE_INSN (cpu, "lw %s, %" PRIiTW "(%s);",
		  rd_name, i_imm, rs1_name);
      store_rd (cpu, rd, EXTEND32 (
	sim_core_read_unaligned_4 (cpu, cpu->pc, read_map,
				   cpu->regs[rs1] + i_imm)));
      break;
    case MATCH_LWU:
      TRACE_INSN (cpu, "lwu %s, %" PRIiTW "(%s);",
		  rd_name, i_imm, rs1_name);
      store_rd (cpu, rd,
	sim_core_read_unaligned_4 (cpu, cpu->pc, read_map,
				   cpu->regs[rs1] + i_imm));
      break;
    case MATCH_LH:
      TRACE_INSN (cpu, "lh %s, %" PRIiTW "(%s);",
		  rd_name, i_imm, rs1_name);
      store_rd (cpu, rd, EXTEND16 (
	sim_core_read_unaligned_2 (cpu, cpu->pc, read_map,
				   cpu->regs[rs1] + i_imm)));
      break;
    case MATCH_LHU:
      TRACE_INSN (cpu, "lhu %s, %" PRIiTW "(%s);",
		  rd_name, i_imm, rs1_name);
      store_rd (cpu, rd,
	sim_core_read_unaligned_2 (cpu, cpu->pc, read_map,
				   cpu->regs[rs1] + i_imm));
      break;
    case MATCH_LB:
      TRACE_INSN (cpu, "lb %s, %" PRIiTW "(%s);",
		  rd_name, i_imm, rs1_name);
      store_rd (cpu, rd, EXTEND8 (
	sim_core_read_unaligned_1 (cpu, cpu->pc, read_map,
				   cpu->regs[rs1] + i_imm)));
      break;
    case MATCH_LBU:
      TRACE_INSN (cpu, "lbu %s, %" PRIiTW "(%s);",
		  rd_name, i_imm, rs1_name);
      store_rd (cpu, rd,
	sim_core_read_unaligned_1 (cpu, cpu->pc, read_map,
				   cpu->regs[rs1] + i_imm));
      break;
    case MATCH_SD:
      TRACE_INSN (cpu, "sd %s, %" PRIiTW "(%s);",
		  rs2_name, s_imm, rs1_name);
      RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
      sim_core_write_unaligned_8 (cpu, cpu->pc, write_map,
				  cpu->regs[rs1] + s_imm, cpu->regs[rs2]);
      break;
    case MATCH_SW:
      TRACE_INSN (cpu, "sw %s, %" PRIiTW "(%s);",
		  rs2_name, s_imm, rs1_name);
      sim_core_write_unaligned_4 (cpu, cpu->pc, write_map,
				  cpu->regs[rs1] + s_imm, cpu->regs[rs2]);
      break;
    case MATCH_SH:
      TRACE_INSN (cpu, "sh %s, %" PRIiTW "(%s);",
		  rs2_name, s_imm, rs1_name);
      sim_core_write_unaligned_2 (cpu, cpu->pc, write_map,
				  cpu->regs[rs1] + s_imm, cpu->regs[rs2]);
      break;
    case MATCH_SB:
      TRACE_INSN (cpu, "sb %s, %" PRIiTW "(%s);",
		  rs2_name, s_imm, rs1_name);
      sim_core_write_unaligned_1 (cpu, cpu->pc, write_map,
				  cpu->regs[rs1] + s_imm, cpu->regs[rs2]);
      break;

    case MATCH_CSRRC:
      TRACE_INSN (cpu, "csrrc");
      switch (csr)
	{
#define DECLARE_CSR(name, num, ...) \
	case num: \
	  store_rd (cpu, rd, fetch_csr (cpu, #name, num, &cpu->csr.name)); \
	  store_csr (cpu, #name, num, &cpu->csr.name, \
		     cpu->csr.name & !cpu->regs[rs1]); \
	  break;
#include "opcode/riscv-opc.h"
#undef DECLARE_CSR
	}
      break;
    case MATCH_CSRRCI:
      TRACE_INSN (cpu, "csrrci");
      switch (csr)
	{
#define DECLARE_CSR(name, num, ...) \
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
#define DECLARE_CSR(name, num, ...) \
	case num: \
	  store_rd (cpu, rd, fetch_csr (cpu, #name, num, &cpu->csr.name)); \
	  store_csr (cpu, #name, num, &cpu->csr.name, \
		     cpu->csr.name | cpu->regs[rs1]); \
	  break;
#include "opcode/riscv-opc.h"
#undef DECLARE_CSR
	}
      break;
    case MATCH_CSRRSI:
      TRACE_INSN (cpu, "csrrsi");
      switch (csr)
	{
#define DECLARE_CSR(name, num, ...) \
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
#define DECLARE_CSR(name, num, ...) \
	case num: \
	  store_rd (cpu, rd, fetch_csr (cpu, #name, num, &cpu->csr.name)); \
	  store_csr (cpu, #name, num, &cpu->csr.name, cpu->regs[rs1]); \
	  break;
#include "opcode/riscv-opc.h"
#undef DECLARE_CSR
	}
      break;
    case MATCH_CSRRWI:
      TRACE_INSN (cpu, "csrrwi");
      switch (csr)
	{
#define DECLARE_CSR(name, num, ...) \
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
      store_rd (cpu, rd,
		fetch_csr (cpu, "cycleh", CSR_CYCLEH, &cpu->csr.cycleh));
      break;
    case MATCH_RDINSTRET:
      TRACE_INSN (cpu, "rdinstret %s;", rd_name);
      store_rd (cpu, rd,
		fetch_csr (cpu, "instret", CSR_INSTRET, &cpu->csr.instret));
      break;
    case MATCH_RDINSTRETH:
      TRACE_INSN (cpu, "rdinstreth %s;", rd_name);
      RISCV_ASSERT_RV32 (cpu, "insn: %s", op->name);
      store_rd (cpu, rd,
		fetch_csr (cpu, "instreth", CSR_INSTRETH, &cpu->csr.instreth));
      break;
    case MATCH_RDTIME:
      TRACE_INSN (cpu, "rdtime %s;", rd_name);
      store_rd (cpu, rd, fetch_csr (cpu, "time", CSR_TIME, &cpu->csr.time));
      break;
    case MATCH_RDTIMEH:
      TRACE_INSN (cpu, "rdtimeh %s;", rd_name);
      RISCV_ASSERT_RV32 (cpu, "insn: %s", op->name);
      store_rd (cpu, rd, fetch_csr (cpu, "timeh", CSR_TIMEH, &cpu->csr.timeh));
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
	sys_id = cpu->t0;
      else
	sys_id = cpu->a7;

      if (cb_target_to_host_syscall (STATE_CALLBACK (sd), sys_id) == -1)
	{
	  switch (sys_id)
	    {
	    case TARGET_SYS_link:
	      {
		char oldpath[1024], newpath[1024];
		cb_get_string (cb, &sc, oldpath, sizeof (oldpath), sc.arg1);
		cb_get_string (cb, &sc, newpath, sizeof (newpath), sc.arg2);
		cpu->a0 = link (oldpath, newpath);
		break;
	      }
	    case TARGET_SYS_brk:
	      {
		/* FIXME: Check the invalid access.  */
		if (cpu->a0 == 0)
		  cpu->a0 = cpu->endbrk;
		else
		  {
		    if (cpu->a0 >= DEFAULT_MEM_SIZE)
		      cpu->a0 = -1;
		    else
		      cpu->endbrk = cpu->a0;
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
						cpu->a0, tv.tv_sec);
		    sim_core_write_unaligned_4 (cpu, cpu->pc, write_map,
						cpu->a0 + 4,
						tv.tv_usec);
		  }
		else
		  {
		    sim_core_write_unaligned_8 (cpu, cpu->pc, write_map,
						cpu->a0, tv.tv_sec);
		    sim_core_write_unaligned_8 (cpu, cpu->pc, write_map,
						cpu->a0 + 8,
						tv.tv_usec);
		  }

		cpu->a0 = rv;
		break;
	      }
	    default:
	      cpu->a0 = sim_syscall (cpu, sys_id, cpu->a0,
				     cpu->a1, cpu->a2, cpu->a3);
	      break;
	    }
	}
      else
	cpu->a0 = sim_syscall (cpu, sys_id, cpu->a0, cpu->a1, cpu->a2, cpu->a3);
      break;
    default:
      TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
      sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
    }

  return pc;
}

static uint64_t
mulhu (uint64_t a, uint64_t b)
{
#ifdef HAVE___INT128
  return ((__int128)a * b) >> 64;
#else
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
#endif
}

static uint64_t
mulh (int64_t a, int64_t b)
{
  int negate = (a < 0) != (b < 0);
  uint64_t res = mulhu (a < 0 ? -a : a, b < 0 ? -b : b);
  return negate ? ~res + (a * b == 0) : res;
}

static uint64_t
mulhsu (int64_t a, uint64_t b)
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
  sim_cia pc = cpu->pc + 4;
  if (ex9)
    pc -= 2;

  dividend_max = -((unsigned_word) 1 << (WITH_TARGET_WORD_BITSIZE - 1));

  switch (op->match)
    {
    case MATCH_DIV:
      TRACE_INSN (cpu, "div %s, %s, %s;  // %s = %s / %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      if (cpu->regs[rs1] == dividend_max && cpu->regs[rs2] == -1)
	tmp = dividend_max;
      else if (cpu->regs[rs2])
	tmp = (signed_word) cpu->regs[rs1] / (signed_word) cpu->regs[rs2];
      else
	tmp = -1;
      store_rd (cpu, rd, tmp);
      break;
    case MATCH_DIVW:
      TRACE_INSN (cpu, "divw %s, %s, %s;  // %s = %s / %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
      if (EXTEND32 (cpu->regs[rs2]) == -1)
	tmp = 1 << 31;
      else if (EXTEND32 (cpu->regs[rs2]))
	tmp = EXTEND32 (cpu->regs[rs1]) / EXTEND32 (cpu->regs[rs2]);
      else
	tmp = -1;
      store_rd (cpu, rd, EXTEND32 (tmp));
      break;
    case MATCH_DIVU:
      TRACE_INSN (cpu, "divu %s, %s, %s;  // %s = %s / %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      if (cpu->regs[rs2])
	store_rd (cpu, rd, (unsigned_word) cpu->regs[rs1]
			   / (unsigned_word) cpu->regs[rs2]);
      else
	store_rd (cpu, rd, -1);
      break;
    case MATCH_DIVUW:
      TRACE_INSN (cpu, "divuw %s, %s, %s;  // %s = %s / %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
      if ((uint32_t) cpu->regs[rs2])
	tmp = (uint32_t) cpu->regs[rs1] / (uint32_t) cpu->regs[rs2];
      else
	tmp = -1;
      store_rd (cpu, rd, EXTEND32 (tmp));
      break;
    case MATCH_MUL:
      TRACE_INSN (cpu, "mul %s, %s, %s;  // %s = %s * %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      store_rd (cpu, rd, cpu->regs[rs1] * cpu->regs[rs2]);
      break;
    case MATCH_MULW:
      TRACE_INSN (cpu, "mulw %s, %s, %s;  // %s = %s * %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
      store_rd (cpu, rd, EXTEND32 ((int32_t) cpu->regs[rs1]
				   * (int32_t) cpu->regs[rs2]));
      break;
    case MATCH_MULH:
      TRACE_INSN (cpu, "mulh %s, %s, %s;  // %s = %s * %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      if (RISCV_XLEN (cpu) == 32)
	store_rd (cpu, rd, ((int64_t)(signed_word) cpu->regs[rs1]
			    * (int64_t)(signed_word) cpu->regs[rs2]) >> 32);
      else
	store_rd (cpu, rd, mulh (cpu->regs[rs1], cpu->regs[rs2]));
      break;
    case MATCH_MULHU:
      TRACE_INSN (cpu, "mulhu %s, %s, %s;  // %s = %s * %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      if (RISCV_XLEN (cpu) == 32)
	store_rd (cpu, rd, ((uint64_t)cpu->regs[rs1]
			    * (uint64_t)cpu->regs[rs2]) >> 32);
      else
	store_rd (cpu, rd, mulhu (cpu->regs[rs1], cpu->regs[rs2]));
      break;
    case MATCH_MULHSU:
      TRACE_INSN (cpu, "mulhsu %s, %s, %s;  // %s = %s * %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      if (RISCV_XLEN (cpu) == 32)
	store_rd (cpu, rd, ((int64_t)(signed_word) cpu->regs[rs1]
			    * (uint64_t)cpu->regs[rs2]) >> 32);
      else
	store_rd (cpu, rd, mulhsu (cpu->regs[rs1], cpu->regs[rs2]));
      break;
    case MATCH_REM:
      TRACE_INSN (cpu, "rem %s, %s, %s;  // %s = %s %% %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      if (cpu->regs[rs1] == dividend_max && cpu->regs[rs2] == -1)
	tmp = 0;
      else if (cpu->regs[rs2])
	tmp = (signed_word) cpu->regs[rs1] % (signed_word) cpu->regs[rs2];
      else
	tmp = cpu->regs[rs1];
      store_rd (cpu, rd, tmp);
      break;
    case MATCH_REMW:
      TRACE_INSN (cpu, "remw %s, %s, %s;  // %s = %s %% %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
      if (EXTEND32 (cpu->regs[rs2]) == -1)
	tmp = 0;
      else if (EXTEND32 (cpu->regs[rs2]))
	tmp = EXTEND32 (cpu->regs[rs1]) % EXTEND32 (cpu->regs[rs2]);
      else
	tmp = cpu->regs[rs1];
      store_rd (cpu, rd, EXTEND32 (tmp));
      break;
    case MATCH_REMU:
      TRACE_INSN (cpu, "remu %s, %s, %s;  // %s = %s %% %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      if (cpu->regs[rs2])
	store_rd (cpu, rd, cpu->regs[rs1] % cpu->regs[rs2]);
      else
	store_rd (cpu, rd, cpu->regs[rs1]);
      break;
    case MATCH_REMUW:
      TRACE_INSN (cpu, "remuw %s, %s, %s;  // %s = %s %% %s",
		  rd_name, rs1_name, rs2_name, rd_name, rs1_name, rs2_name);
      RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);
      if ((uint32_t) cpu->regs[rs2])
	tmp = (uint32_t) cpu->regs[rs1] % (uint32_t) cpu->regs[rs2];
      else
	tmp = cpu->regs[rs1];
      store_rd (cpu, rd, EXTEND32 (tmp));
      break;
    default:
      TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
      sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
    }

  return pc;
}

static sim_cia
execute_a (SIM_CPU *cpu, unsigned_word iw, const struct riscv_opcode *op, int ex9)
{
  SIM_DESC sd = CPU_STATE (cpu);
  struct riscv_sim_state *state = RISCV_SIM_STATE (sd);
  int rd = (iw >> OP_SH_RD) & OP_MASK_RD;
  int rs1 = (iw >> OP_SH_RS1) & OP_MASK_RS1;
  int rs2 = (iw >> OP_SH_RS2) & OP_MASK_RS2;
  const char *rd_name = riscv_gpr_names_abi[rd];
  const char *rs1_name = riscv_gpr_names_abi[rs1];
  const char *rs2_name = riscv_gpr_names_abi[rs2];
  struct atomic_mem_reserved_list *amo_prev, *amo_curr;
  insn_t aqrl_mask = (OP_MASK_AQ << OP_SH_AQ) | (OP_MASK_RL << OP_SH_RL);
  unsigned_word tmp;
  unsigned_word rs2_val = cpu->regs[rs2];
  sim_cia pc = cpu->pc + 4;
  if (ex9)
    pc -= 2;

  /* Handle these two load/store operations specifically.  */
  switch (op->match & ~aqrl_mask)
    {
    case MATCH_LR_W:
      TRACE_INSN (cpu, "%s %s, (%s);", op->name, rd_name, rs1_name);
      store_rd (cpu, rd,
	sim_core_read_unaligned_4 (cpu, cpu->pc, read_map, cpu->regs[rs1]));

      /* Walk the reservation list to find an existing match.  */
      amo_curr = state->amo_reserved_list;
      while (amo_curr)
	{
	  if (amo_curr->addr == cpu->regs[rs1])
	    goto done;
	  amo_curr = amo_curr->next;
	}

      /* No reservation exists, so add one.  */
      amo_curr = xmalloc (sizeof (*amo_curr));
      amo_curr->addr = cpu->regs[rs1];
      amo_curr->next = state->amo_reserved_list;
      state->amo_reserved_list = amo_curr;
      goto done;
    case MATCH_SC_W:
      TRACE_INSN (cpu, "%s %s, %s, (%s);", op->name, rd_name, rs2_name,
		  rs1_name);

      /* Walk the reservation list to find a match.  */
      amo_curr = amo_prev = state->amo_reserved_list;
      while (amo_curr)
	{
	  if (amo_curr->addr == cpu->regs[rs1])
	    {
	      /* We found a reservation, so operate it.  */
	      sim_core_write_unaligned_4 (cpu, cpu->pc, write_map,
					  cpu->regs[rs1], cpu->regs[rs2]);
	      store_rd (cpu, rd, 0);
	      if (amo_curr == state->amo_reserved_list)
		state->amo_reserved_list = amo_curr->next;
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
    tmp = sim_core_read_unaligned_8 (cpu, cpu->pc, read_map, cpu->regs[rs1]);
  else
    tmp = EXTEND32 (sim_core_read_unaligned_4 (cpu, cpu->pc, read_map,
					       cpu->regs[rs1]));
  store_rd (cpu, rd, tmp);

  switch (op->match & ~aqrl_mask)
    {
    case MATCH_AMOADD_D:
    case MATCH_AMOADD_W:
      tmp = tmp + cpu->regs[rs2];
      break;
    case MATCH_AMOAND_D:
    case MATCH_AMOAND_W:
      tmp = tmp & cpu->regs[rs2];
      break;
    case MATCH_AMOMAX_D:
    case MATCH_AMOMAX_W:
      tmp = max ((signed_word) cpu->regs[rd], (signed_word) cpu->regs[rs2]);
      break;
    case MATCH_AMOMAXU_D:
    case MATCH_AMOMAXU_W:
      tmp = max ((unsigned_word) cpu->regs[rd], (unsigned_word) cpu->regs[rs2]);
      break;
    case MATCH_AMOMIN_D:
    case MATCH_AMOMIN_W:
      tmp = min ((signed_word) cpu->regs[rd], (signed_word) cpu->regs[rs2]);
      break;
    case MATCH_AMOMINU_D:
    case MATCH_AMOMINU_W:
      tmp = min ((unsigned_word) cpu->regs[rd], (unsigned_word) cpu->regs[rs2]);
      break;
    case MATCH_AMOOR_D:
    case MATCH_AMOOR_W:
      tmp = tmp | cpu->regs[rs2];
      break;
    case MATCH_AMOSWAP_D:
    case MATCH_AMOSWAP_W:
      tmp = rs2_val;
      break;
    case MATCH_AMOXOR_D:
    case MATCH_AMOXOR_W:
      tmp = tmp ^ cpu->regs[rs2];
      break;
    default:
      TRACE_INSN (cpu, "UNHANDLED INSN: %s", op->name);
      sim_engine_halt (sd, cpu, NULL, cpu->pc, sim_signalled, SIM_SIGILL);
    }

  if (op->xlen_requirement == 64)
    sim_core_write_unaligned_8 (cpu, cpu->pc, write_map, cpu->regs[rs1], tmp);
  else
    sim_core_write_unaligned_4 (cpu, cpu->pc, write_map, cpu->regs[rs1], tmp);

 done:
  return pc;
}

static sim_cia
execute_one (SIM_CPU *cpu, unsigned_word iw, const struct riscv_opcode *op, int ex9)
{
  SIM_DESC sd = CPU_STATE (cpu);

  if (op->xlen_requirement == 32)
    RISCV_ASSERT_RV32 (cpu, "insn: %s", op->name);
  else if (op->xlen_requirement == 64)
    RISCV_ASSERT_RV64 (cpu, "insn: %s", op->name);

  switch (op->insn_class)
    {
    case INSN_CLASS_A:
      return execute_a (cpu, iw, op, ex9);
    case INSN_CLASS_C:
      return execute_c (cpu, iw, op);
    case INSN_CLASS_D:
      return execute_d (cpu, iw, op, ex9);
    case INSN_CLASS_F:
      return execute_f (cpu, iw, op, ex9);
    case INSN_CLASS_I:
      return execute_i (cpu, iw, op, ex9);
    case INSN_CLASS_M:
      return execute_m (cpu, iw, op, ex9);
    default:
      TRACE_INSN (cpu, "UNHANDLED EXTENSION: %d", op->insn_class);
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

  /* NB: Same loop logic as riscv_disassemble_insn.  */
  for (; op->name; op++)
    {
      /* Does the opcode match?  */
      if (! op->match_func (op, iw))
	continue;
      /* Is this a pseudo-instruction and may we print it as such?  */
      if (op->pinfo & INSN_ALIAS)
	continue;
      /* Is this instruction restricted to a certain value of XLEN?  */
      if (op->xlen_requirement != 0 && op->xlen_requirement != xlen)
	continue;

      /* It's a match.  */
      pc = execute_one (cpu, iw, op, ex9);
      break;
    }

  return pc;
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
  ++cpu->csr.cycle;
  ++cpu->csr.mcycle;
  if (RISCV_XLEN (cpu) == 32)
    {
      cpu->csr.cycleh = ((uint64_t)cpu->csr.cycle >> 32);
      cpu->csr.mcycleh = ((uint64_t)cpu->csr.mcycle >> 32);
    }
  ++cpu->csr.instret;

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
  if (len <= 0 || len > sizeof (unsigned_word))
    return -1;

  switch (rn)
    {
    case SIM_RISCV_ZERO_REGNUM:
      memset (buf, 0, len);
      return len;
    case SIM_RISCV_RA_REGNUM ... SIM_RISCV_T6_REGNUM:
      memcpy (buf, &cpu->regs[rn], len);
      return len;
    case SIM_RISCV_FIRST_FP_REGNUM ... SIM_RISCV_LAST_FP_REGNUM:
      memcpy (buf, &cpu->fpregs[rn - SIM_RISCV_FIRST_FP_REGNUM], len);
      return len;
    case SIM_RISCV_PC_REGNUM:
      memcpy (buf, &cpu->pc, len);
      return len;

#define DECLARE_CSR(name, num, ...) \
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
  if (len <= 0 || len > sizeof (unsigned_word))
    return -1;

  switch (rn)
    {
    case SIM_RISCV_ZERO_REGNUM:
      /* Ignore writes.  */
      return len;
    case SIM_RISCV_RA_REGNUM ... SIM_RISCV_T6_REGNUM:
      memcpy (&cpu->regs[rn], buf, len);
      return len;
    case SIM_RISCV_FIRST_FP_REGNUM ... SIM_RISCV_LAST_FP_REGNUM:
      memcpy (&cpu->fpregs[rn - SIM_RISCV_FIRST_FP_REGNUM], buf, len);
      return len;
    case SIM_RISCV_PC_REGNUM:
      memcpy (&cpu->pc, buf, len);
      return len;

#define DECLARE_CSR(name, num, ...) \
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
void
initialize_cpu (SIM_DESC sd, SIM_CPU *cpu, int mhartid)
{
  const char *extensions;
  int i;

  memset (cpu->regs, 0, sizeof (cpu->regs));

  CPU_PC_FETCH (cpu) = pc_get;
  CPU_PC_STORE (cpu) = pc_set;
  CPU_REG_FETCH (cpu) = reg_fetch;
  CPU_REG_STORE (cpu) = reg_store;

  if (!riscv_hash[0])
    {
      const struct riscv_opcode *op;

      for (op = riscv_opcodes; op->name; op++)
	if (!riscv_hash[OP_HASH_IDX (op->match)])
	  riscv_hash[OP_HASH_IDX (op->match)] = op;
    }

  cpu->csr.misa = 0;
  /* RV32 sets this field to 0, and we don't really support RV128 yet.  */
  if (RISCV_XLEN (cpu) == 64)
    cpu->csr.misa |= (uint64_t)2 << 62;

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

void
initialize_env (SIM_DESC sd, const char * const *argv, const char * const *env)
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
  sp = align_up ((sp - 15), 16);

  /* Set up the regs the libgloss crt0 expects.  */
  cpu->a0 = argc;
  cpu->sp = sp;

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
