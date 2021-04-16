/* Simulator for NDS32 processors.

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

#ifndef _NDS32_SIM_H_
#define _NDS32_SIM_H_

#include <stdbool.h>
#include <stdint.h>

#include <stdint.h>

#include "sim-main.h"

enum nds32_cpu_regnum
{
  GPR_TA = 15,
  GPR_FP = 28,
  GPR_GP = 29,
  GPR_LP = 30,
  GPR_SP = 31,

  USR0_D0LO = 0,
  USR0_D0HI = 1,
  USR0_D1LO = 2,
  USR0_D1HI = 3,
  USR0_LB = 25,
  USR0_LE = 26,
  USR0_LC = 27,
  USR0_ITB = 28,
  USR0_IFCLP = 29,
  USR0_PC = 31,
};

#define SRIDX(M,m,e)  ((M << 7) | (m << 3) | e)
#define UXIDX(g,u)    ((g << 5) | u)

enum nds32_exceptions
{
  EXP_RESET = 0,
  EXP_TLB_FILL = 1,
  EXP_NO_PTE = 2,
  EXP_TLB_MISC = 3,
  EXP_TLB_VLPT_MISS = 4,
  EXP_MACHINE_ERROR = 5,
  EXP_DEBUG = 6,
  EXP_GENERAL = 7,
  EXP_SYSCALL = 8,
  EXP_HW0 = 9,	/* HW0-5: 9-14 */
  EXP_VEP0 = 9,	/* VEP0-64: 9-72 */
  EXP_SW0 = 15,

  EXP_BADOP,
};

uint32_t nds32_raise_exception (sim_cpu *cpu, enum nds32_exceptions e, int sig, char *msg, ...);

/* Do not use thsi directly. */
uint64_t  __nds32_ld (sim_cpu *cpu, SIM_ADDR addr, int size, int aligned_p);
void __nds32_st (sim_cpu *cpu, SIM_ADDR addr, int size, uint64_t  val, int aligned_p);
/* Use these wrappers. */
#define nds32_ld_aligned(CPU, ADDR, SIZE)		__nds32_ld (CPU, ADDR, SIZE, 1)
#define nds32_st_aligned(CPU, ADDR, SIZE, VAL)		__nds32_st (CPU, ADDR, SIZE, VAL, 1)
#define nds32_ld_unaligned(CPU, ADDR, SIZE)		__nds32_ld (CPU, ADDR, SIZE, 0)
#define nds32_st_unaligned(CPU, ADDR, SIZE, VAL)	__nds32_st (CPU, ADDR, SIZE, VAL, 0)

void nds32_init_libgloss (SIM_DESC sd, struct bfd *abfd, char * const *argv, char * const *env);
void nds32_init_linux (SIM_DESC sd, struct bfd *abfd, char **argv, char **env);

sim_cia nds32_decode32_lwc (sim_cpu *cpu, const uint32_t insn, sim_cia cia);
sim_cia nds32_decode32_swc (sim_cpu *cpu, const uint32_t insn, sim_cia cia);
sim_cia nds32_decode32_ldc (sim_cpu *cpu, const uint32_t insn, sim_cia cia);
sim_cia nds32_decode32_sdc (sim_cpu *cpu, const uint32_t insn, sim_cia cia);
sim_cia nds32_decode32_cop (sim_cpu *cpu, const uint32_t insn, sim_cia cia);
void nds32_bad_op (sim_cpu *cpu, uint32_t cia, uint32_t insn, char *tag);

void nds32_bad_op (sim_cpu *cpu, uint32_t cia, uint32_t insn, char *tag);

#if 1
#define SIM_IO_DPRINTF(sd, fmt, args...)   sim_io_printf (sd, fmt, ## args)
#else
#define SIM_IO_DPRINTF(...)	do { } while (0)
#endif

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

enum PERFM_EVENT_ENUM
{
  PFM_CYCLE = 0,
  PFM_INST,

  PFM_COND_BRANCH = 64 + 2,
  PFM_TAKEN_COND,
  PFM_PREFETCH,
  PFM_RET,
  PFM_JR,
  PFM_JAL,
  PFM_NOP,
  PFM_SCW,
  PFM_IDSB,
  PFM_CCTL,
  PFM_TAKEN_INT,
  PFM_LOADS,

  PFM_COND_BRANCH_MISPREDICT = 128 + 2,
};

ATTRIBUTE_UNUSED static void
__put_field (uint32_t *src, int shift, int bs, uint32_t val)
{
  uint32_t mask = (1 << bs) - 1;

  val &= mask;
  *src = (*src & ~(mask << shift)) | (val << shift);
}

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

#endif
