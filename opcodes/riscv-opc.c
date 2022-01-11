/* RISC-V opcode list
   Copyright (C) 2011-2020 Free Software Foundation, Inc.

   Contributed by Andrew Waterman (andrew@sifive.com).
   Based on MIPS target.

   This file is part of the GNU opcodes library.

   This library is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3, or (at your option)
   any later version.

   It is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
   or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
   License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; see the file COPYING3. If not,
   see <http://www.gnu.org/licenses/>.  */

#include "sysdep.h"
#include "opcode/riscv.h"
#include <stdio.h>

/* Register names used by gas and objdump.  */

const char * const riscv_gpr_names_numeric[NGPR] =
{
  "x0",   "x1",   "x2",   "x3",   "x4",   "x5",   "x6",   "x7",
  "x8",   "x9",   "x10",  "x11",  "x12",  "x13",  "x14",  "x15",
  "x16",  "x17",  "x18",  "x19",  "x20",  "x21",  "x22",  "x23",
  "x24",  "x25",  "x26",  "x27",  "x28",  "x29",  "x30",  "x31"
};

const char * const riscv_gpr_names_abi[NGPR] = {
  "zero", "ra", "sp",  "gp",  "tp", "t0",  "t1",  "t2",
  "s0",   "s1", "a0",  "a1",  "a2", "a3",  "a4",  "a5",
  "a6",   "a7", "s2",  "s3",  "s4", "s5",  "s6",  "s7",
  "s8",   "s9", "s10", "s11", "t3", "t4",  "t5",  "t6"
};

const char * const riscv_gpr_names_standard[NGPR] = {
  "r0",   "r1",   "r2",   "r3",   "r4",   "r5",   "r6",   "r7",
  "r8",   "r9",   "r10",  "r11",  "r12",  "r13",  "r14",  "r15",
  "r16",  "r17",  "r18",  "r19",  "r20",  "r21",  "r22",  "r23",
  "r24",  "r25",  "r26",  "r27",  "r28",  "r29",  "r30",  "r31"
};

const char * const riscv_fpr_names_numeric[NFPR] =
{
  "f0",   "f1",   "f2",   "f3",   "f4",   "f5",   "f6",   "f7",
  "f8",   "f9",   "f10",  "f11",  "f12",  "f13",  "f14",  "f15",
  "f16",  "f17",  "f18",  "f19",  "f20",  "f21",  "f22",  "f23",
  "f24",  "f25",  "f26",  "f27",  "f28",  "f29",  "f30",  "f31"
};

const char * const riscv_fpr_names_abi[NFPR] = {
  "ft0", "ft1", "ft2",  "ft3",  "ft4", "ft5", "ft6",  "ft7",
  "fs0", "fs1", "fa0",  "fa1",  "fa2", "fa3", "fa4",  "fa5",
  "fa6", "fa7", "fs2",  "fs3",  "fs4", "fs5", "fs6",  "fs7",
  "fs8", "fs9", "fs10", "fs11", "ft8", "ft9", "ft10", "ft11"
};

const char * const riscv_vecr_names_numeric[NVECR] =
{
  "v0",   "v1",   "v2",   "v3",   "v4",   "v5",   "v6",   "v7",
  "v8",   "v9",   "v10",  "v11",  "v12",  "v13",  "v14",  "v15",
  "v16",  "v17",  "v18",  "v19",  "v20",  "v21",  "v22",  "v23",
  "v24",  "v25",  "v26",  "v27",  "v28",  "v29",  "v30",  "v31"
};

const char * const riscv_vecm_names_numeric[NVECM] =
{
  "v0.t"
};

/* The order of overloaded instructions matters.  Label arguments and
   register arguments look the same. Instructions that can have either
   for arguments must apear in the correct order in this table for the
   assembler to pick the right one. In other words, entries with
   immediate operands must apear after the same instruction with
   registers.

   Because of the lookup algorithm used, entries with the same opcode
   name must be contiguous.  */

#define MASK_RS1 (OP_MASK_RS1 << OP_SH_RS1)
#define MASK_RS2 (OP_MASK_RS2 << OP_SH_RS2)
#define MASK_RD (OP_MASK_RD << OP_SH_RD)
#define MASK_CRS2 (OP_MASK_CRS2 << OP_SH_CRS2)
#define MASK_IMM ENCODE_ITYPE_IMM (-1U)
#define MASK_RVC_IMM ENCODE_RVC_IMM (-1U)
#define MASK_UIMM ENCODE_UTYPE_IMM (-1U)
#define MASK_RM (OP_MASK_RM << OP_SH_RM)
#define MASK_PRED (OP_MASK_PRED << OP_SH_PRED)
#define MASK_SUCC (OP_MASK_SUCC << OP_SH_SUCC)
#define MASK_AQ (OP_MASK_AQ << OP_SH_AQ)
#define MASK_RL (OP_MASK_RL << OP_SH_RL)
#define MASK_AQRL (MASK_AQ | MASK_RL)
#define MASK_VD  (OP_MASK_VD << OP_SH_VD)
#define MASK_VS1 (OP_MASK_VS1 << OP_SH_VS1)
#define MASK_VS2 (OP_MASK_VS2 << OP_SH_VS2)
#define MASK_VMASK (OP_MASK_VMASK << OP_SH_VMASK)
/* the followings are moved to riscv-opc.h
#define MASK_SHAMT (OP_MASK_SHAMT << OP_SH_SHAMT)
#define MATCH_SHAMT_REV8_32 (0b11000 << OP_SH_SHAMT)
#define MATCH_SHAMT_REV8_64 (0b111000 << OP_SH_SHAMT)
#define MATCH_SHAMT_BREV8 (0b00111 << OP_SH_SHAMT)
#define MATCH_SHAMT_ZIP_32 (0b1111 << OP_SH_SHAMT)
#define MATCH_SHAMT_ORC_B (0b00111 << OP_SH_SHAMT)
*/

static struct opc_options
{
  int no_vic;
} opc_opts =
{
  0, /* no_vic */
};

int opc_set_no_vic (int is);
int opc_set_no_vic (int is)
{
  int previous = opc_opts.no_vic;
  opc_opts.no_vic = is;
  return previous;
}

static int
match_opcode (const struct riscv_opcode *op,
	      insn_t insn,
	      int constraints ATTRIBUTE_UNUSED,
	      const char **error ATTRIBUTE_UNUSED)
{
  return ((insn ^ op->match) & op->mask) == 0;
}

static int
match_never (const struct riscv_opcode *op ATTRIBUTE_UNUSED,
	     insn_t insn ATTRIBUTE_UNUSED,
	     int constraints ATTRIBUTE_UNUSED,
	     const char **error ATTRIBUTE_UNUSED)
{
  return 0;
}

static int
match_rs1_eq_rs2 (const struct riscv_opcode *op,
		  insn_t insn,
		  int constraints ATTRIBUTE_UNUSED,
		  const char **error ATTRIBUTE_UNUSED)
{
  int rs1 = (insn & MASK_RS1) >> OP_SH_RS1;
  int rs2 = (insn & MASK_RS2) >> OP_SH_RS2;
  return match_opcode (op, insn, 0, NULL) && rs1 == rs2;
}

static int
match_vs1_eq_vs2 (const struct riscv_opcode *op,
		  insn_t insn,
		  int constraints ATTRIBUTE_UNUSED,
		  const char **error ATTRIBUTE_UNUSED)
{
  int vs1 = (insn & MASK_VS1) >> OP_SH_VS1;
  int vs2 = (insn & MASK_VS2) >> OP_SH_VS2;

  return match_opcode (op, insn, 0, NULL) && vs1 == vs2;
}

static int
match_vs1_eq_vs2_neq_vm (const struct riscv_opcode *op,
			 insn_t insn,
			 int constraints,
			 const char **error)
{
  int vd = (insn & MASK_VD) >> OP_SH_VD;
  int vs1 = (insn & MASK_VS1) >> OP_SH_VS1;
  int vs2 = (insn & MASK_VS2) >> OP_SH_VS2;
  int vm = (insn & MASK_VMASK) >> OP_SH_VMASK;

  if (!constraints || error == NULL)
    return match_opcode (op, insn, 0, NULL) && vs1 == vs2;

  if (!vm && vm == vd)
    *error = "illegal operands vd cannot overlap vm";
  else
    return match_opcode (op, insn, 0, NULL) && vs1 == vs2;
  return 0;
}

static int
match_vd_eq_vs1_eq_vs2 (const struct riscv_opcode *op,
			insn_t insn,
			int constraints ATTRIBUTE_UNUSED,
			const char **error ATTRIBUTE_UNUSED)
{
  int vd =  (insn & MASK_VD) >> OP_SH_VD;
  int vs1 = (insn & MASK_VS1) >> OP_SH_VS1;
  int vs2 = (insn & MASK_VS2) >> OP_SH_VS2;

  return match_opcode (op, insn, 0, NULL) && vd == vs1 && vs1 == vs2;
}

static int
match_rd_nonzero (const struct riscv_opcode *op,
		  insn_t insn,
		  int constraints ATTRIBUTE_UNUSED,
		  const char **error ATTRIBUTE_UNUSED)
{
  return match_opcode (op, insn, 0, NULL) && ((insn & MASK_RD) != 0);
}

static int
match_c_add (const struct riscv_opcode *op,
	     insn_t insn,
	     int constraints ATTRIBUTE_UNUSED,
	     const char **error ATTRIBUTE_UNUSED)
{
  return match_rd_nonzero (op, insn, 0, NULL) && ((insn & MASK_CRS2) != 0);
}

/* We don't allow mv zero,X to become a c.mv hint, so we need a separate
   matching function for this.  */

static int
match_c_add_with_hint (const struct riscv_opcode *op,
		       insn_t insn,
		       int constraints ATTRIBUTE_UNUSED,
		       const char **error ATTRIBUTE_UNUSED)
{
  return match_opcode (op, insn, 0, NULL) && ((insn & MASK_CRS2) != 0);
}

static int
match_c_nop (const struct riscv_opcode *op,
	     insn_t insn,
	     int constraints ATTRIBUTE_UNUSED,
	     const char **error ATTRIBUTE_UNUSED)
{
  return (match_opcode (op, insn, 0, NULL)
	  && (((insn & MASK_RD) >> OP_SH_RD) == 0));
}

static int
match_c_addi16sp (const struct riscv_opcode *op,
		  insn_t insn,
		  int constraints ATTRIBUTE_UNUSED,
		  const char **error ATTRIBUTE_UNUSED)
{
  return (match_opcode (op, insn, 0, NULL)
	  && (((insn & MASK_RD) >> OP_SH_RD) == 2)
	  && EXTRACT_RVC_ADDI16SP_IMM (insn) != 0);
}

static int
match_c_lui (const struct riscv_opcode *op,
	     insn_t insn,
	     int constraints ATTRIBUTE_UNUSED,
	     const char **error ATTRIBUTE_UNUSED)
{
  return (match_rd_nonzero (op, insn, 0, NULL)
	  && (((insn & MASK_RD) >> OP_SH_RD) != 2)
	  && EXTRACT_RVC_LUI_IMM (insn) != 0);
}

/* We don't allow lui zero,X to become a c.lui hint, so we need a separate
   matching function for this.  */

static int
match_c_lui_with_hint (const struct riscv_opcode *op,
		       insn_t insn,
		       int constraints ATTRIBUTE_UNUSED,
		       const char **error ATTRIBUTE_UNUSED)
{
  return (match_opcode (op, insn, 0, NULL)
	  && (((insn & MASK_RD) >> OP_SH_RD) != 2)
	  && EXTRACT_RVC_LUI_IMM (insn) != 0);
}

static int
match_c_addi4spn (const struct riscv_opcode *op,
		  insn_t insn,
		  int constraints ATTRIBUTE_UNUSED,
		  const char **error ATTRIBUTE_UNUSED)
{
  return (match_opcode (op, insn, 0, NULL)
	  && EXTRACT_RVC_ADDI4SPN_IMM (insn) != 0);
}

/* This requires a non-zero shift.  A zero rd is a hint, so is allowed.  */

static int
match_c_slli (const struct riscv_opcode *op,
	      insn_t insn,
	      int constraints ATTRIBUTE_UNUSED,
	      const char **error ATTRIBUTE_UNUSED)
{
  return match_opcode (op, insn, 0, NULL) && EXTRACT_RVC_IMM (insn) != 0;
}

/* This requires a non-zero rd, and a non-zero shift.  */

static int
match_slli_as_c_slli (const struct riscv_opcode *op,
		      insn_t insn,
		      int constraints ATTRIBUTE_UNUSED,
		      const char **error ATTRIBUTE_UNUSED)
{
  return (match_rd_nonzero (op, insn, 0, NULL)
	  && EXTRACT_RVC_IMM (insn) != 0);
}

/* This requires a zero shift.  A zero rd is a hint, so is allowed.  */

static int
match_c_slli64 (const struct riscv_opcode *op,
		insn_t insn,
		int constraints ATTRIBUTE_UNUSED,
		const char **error ATTRIBUTE_UNUSED)
{
  return match_opcode (op, insn, 0, NULL) && EXTRACT_RVC_IMM (insn) == 0;
}

/* This is used for both srli and srai.  This requires a non-zero shift.
   A zero rd is not possible.  */

static int
match_srxi_as_c_srxi (const struct riscv_opcode *op,
		      insn_t insn,
		      int constraints ATTRIBUTE_UNUSED,
		      const char **error ATTRIBUTE_UNUSED)
{
  return match_opcode (op, insn, 0, NULL) && EXTRACT_RVC_IMM (insn) != 0;
}

/* These are used to check the vector constraints.  */

static int
match_widen_vd_neq_vs1_neq_vs2_neq_vm (const struct riscv_opcode *op,
				       insn_t insn,
				       int constraints,
				       const char **error)
{
  int vd = (insn & MASK_VD) >> OP_SH_VD;
  int vs1 = (insn & MASK_VS1) >> OP_SH_VS1;
  int vs2 = (insn & MASK_VS2) >> OP_SH_VS2;
  int vm = (insn & MASK_VMASK) >> OP_SH_VMASK;

  if (!constraints || error == NULL)
    return match_opcode (op, insn, 0, NULL);

  if ((vd % 2) != 0)
    *error = "illegal operands vd must be multiple of 2";
  else if (vs1 >= vd && vs1 <= (vd + 1))
    *error = "illegal operands vd cannot overlap vs1";
  else if (vs2 >= vd && vs2 <= (vd + 1))
    *error = "illegal operands vd cannot overlap vs2";
  else if (!vm && vm >= vd && vm <= (vd + 1))
    *error = "illegal operands vd cannot overlap vm";
  else
    return match_opcode (op, insn, 0, NULL);
  return 0;
}

static int
match_widen_vd_neq_vs1_neq_vm (const struct riscv_opcode *op,
			       insn_t insn,
			       int constraints,
			       const char **error)
{
  int vd = (insn & MASK_VD) >> OP_SH_VD;
  int vs1 = (insn & MASK_VS1) >> OP_SH_VS1;
  int vs2 = (insn & MASK_VS2) >> OP_SH_VS2;
  int vm = (insn & MASK_VMASK) >> OP_SH_VMASK;

  if (!constraints || error == NULL)
    return match_opcode (op, insn, 0, NULL);

  if ((vd % 2) != 0)
    *error = "illegal operands vd must be multiple of 2";
  else if ((vs2 % 2) != 0)
    *error = "illegal operands vs2 must be multiple of 2";
  else if (vs1 >= vd && vs1 <= (vd + 1))
    *error = "illegal operands vd cannot overlap vs1";
  else if (!vm && vm >= vd && vm <= (vd + 1))
    *error = "illegal operands vd cannot overlap vm";
  else
    return match_opcode (op, insn, 0, NULL);
  return 0;
}

static int
match_widen_vd_neq_vs2_neq_vm (const struct riscv_opcode *op,
			       insn_t insn,
			       int constraints,
			       const char **error)
{
  int vd = (insn & MASK_VD) >> OP_SH_VD;
  int vs2 = (insn & MASK_VS2) >> OP_SH_VS2;
  int vm = (insn & MASK_VMASK) >> OP_SH_VMASK;

  if (!constraints || error == NULL)
    return match_opcode (op, insn, 0, NULL);

  if ((vd % 2) != 0)
    *error = "illegal operands vd must be multiple of 2";
  else if (vs2 >= vd && vs2 <= (vd + 1))
    *error = "illegal operands vd cannot overlap vs2";
  else if (!vm && vm >= vd && vm <= (vd + 1))
    *error = "illegal operands vd cannot overlap vm";
  else
    return match_opcode (op, insn, 0, NULL);
  return 0;
}

static int
match_widen_vd_neq_vm (const struct riscv_opcode *op,
		       insn_t insn,
		       int constraints,
		       const char **error)
{
  int vd = (insn & MASK_VD) >> OP_SH_VD;
  int vs2 = (insn & MASK_VS2) >> OP_SH_VS2;
  int vm = (insn & MASK_VMASK) >> OP_SH_VMASK;

  if (!constraints || error == NULL)
    return match_opcode (op, insn, 0, NULL);

  if ((vd % 2) != 0)
    *error = "illegal operands vd must be multiple of 2";
  else if ((vs2 % 2) != 0)
    *error = "illegal operands vs2 must be multiple of 2";
  else if (!vm && vm >= vd && vm <= (vd + 1))
    *error = "illegal operands vd cannot overlap vm";
  else
    return match_opcode (op, insn, 0, NULL);
  return 0;
}

static int
match_narrow_vd_neq_vs2_neq_vm (const struct riscv_opcode *op,
				insn_t insn,
				int constraints,
				const char **error)
{
  int vd = (insn & MASK_VD) >> OP_SH_VD;
  int vs2 = (insn & MASK_VS2) >> OP_SH_VS2;
  int vm = (insn & MASK_VMASK) >> OP_SH_VMASK;

  if (!constraints || error == NULL)
    return match_opcode (op, insn, 0, NULL);

  if ((vs2 % 2) != 0)
    *error = "illegal operands vd must be multiple of 2";
  else if (vd >= vs2 && vd <= (vs2 + 1))
    *error = "illegal operands vd cannot overlap vs2";
  else if (!vm && vd >= vm && vd <= (vm + 1))
    *error = "illegal operands vd cannot overlap vm";
  else
    return match_opcode (op, insn, 0, NULL);
  return 0;
}

static int
match_quad_vd_neq_vs1_neq_vs2_neq_vm (const struct riscv_opcode *op,
				      insn_t insn,
				      int constraints,
				      const char **error)
{
  int vd = (insn & MASK_VD) >> OP_SH_VD;
  int vs1 = (insn & MASK_VS1) >> OP_SH_VS1;
  int vs2 = (insn & MASK_VS2) >> OP_SH_VS2;
  int vm = (insn & MASK_VMASK) >> OP_SH_VMASK;

  if (!constraints || error == NULL)
    return match_opcode (op, insn, 0, NULL);

  if ((vd % 4) != 0)
    *error = "illegal operands vd must be multiple of 4";
  else if (vs1 >= vd && vs1 <= (vd + 3))
    *error = "illegal operands vd cannot overlap vs1";
  else if (vs2 >= vd && vs2 <= (vd + 3))
    *error = "illegal operands vd cannot overlap vs2";
  else if (!vm && vm >= vd && vm <= (vd + 3))
    *error = "illegal operands vd cannot overlap vm";
  else
    return match_opcode (op, insn, 0, NULL);
  return 0;
}

static int
match_quad_vd_neq_vs2_neq_vm (const struct riscv_opcode *op,
			      insn_t insn,
			      int constraints,
			      const char **error)
{
  int vd = (insn & MASK_VD) >> OP_SH_VD;
  int vs2 = (insn & MASK_VS2) >> OP_SH_VS2;
  int vm = (insn & MASK_VMASK) >> OP_SH_VMASK;

  if (!constraints || error == NULL)
    return match_opcode (op, insn, 0, NULL);

  if ((vd % 4) != 0)
    *error = "illegal operands vd must be multiple of 4";
  else if (vs2 >= vd && vs2 <= (vd + 3))
    *error = "illegal operands vd cannot overlap vs2";
  else if (!vm && vm >= vd && vm <= (vd + 3))
    *error = "illegal operands vd cannot overlap vm";
  else
    return match_opcode (op, insn, 0, NULL);
  return 0;
}

static int
match_vd_neq_vs1_neq_vs2 (const struct riscv_opcode *op,
			  insn_t insn,
			  int constraints,
			  const char **error)
{
  int vd = (insn & MASK_VD) >> OP_SH_VD;
  int vs1 = (insn & MASK_VS1) >> OP_SH_VS1;
  int vs2 = (insn & MASK_VS2) >> OP_SH_VS2;

  if (!constraints || error == NULL)
    return match_opcode (op, insn, 0, NULL);

  if (vs1 == vd)
    *error = "illegal operands vd cannot overlap vs1";
  else if (vs2 == vd)
    *error = "illegal operands vd cannot overlap vs2";
  else
    return match_opcode (op, insn, 0, NULL);
  return 0;
}

static int
match_vd_neq_vs1_neq_vs2_neq_vm (const struct riscv_opcode *op,
				 insn_t insn,
				 int constraints,
				 const char **error)
{
  int vd = (insn & MASK_VD) >> OP_SH_VD;
  int vs1 = (insn & MASK_VS1) >> OP_SH_VS1;
  int vs2 = (insn & MASK_VS2) >> OP_SH_VS2;
  int vm = (insn & MASK_VMASK) >> OP_SH_VMASK;

  if (!constraints || error == NULL)
    return match_opcode (op, insn, 0, NULL);

  if (vs1 == vd)
    *error = "illegal operands vd cannot overlap vs1";
  else if (vs2 == vd)
    *error = "illegal operands vd cannot overlap vs2";
  else if (!vm && vm == vd)
    *error = "illegal operands vd cannot overlap vm";
  else
    return match_opcode (op, insn, 0, NULL);
  return 0;
}

static int
match_vd_neq_vs2_neq_vm (const struct riscv_opcode *op,
			 insn_t insn,
			 int constraints,
			 const char **error)
{
  int vd = (insn & MASK_VD) >> OP_SH_VD;
  int vs2 = (insn & MASK_VS2) >> OP_SH_VS2;
  int vm = (insn & MASK_VMASK) >> OP_SH_VMASK;

  if (!constraints || error == NULL)
    return match_opcode (op, insn, 0, NULL);

  if (vs2 == vd)
    *error = "illegal operands vd cannot overlap vs2";
  else if (!vm && vm == vd)
    *error = "illegal operands vd cannot overlap vm";
  else
    return match_opcode (op, insn, 0, NULL);
  return 0;
}

/* v[m]adc and v[m]sbc use the vm encoding to encode the
   carry-in v0 register.  The carry-in v0 register can not
   overlap with the vd, too.  Therefore, we use the same
   match_vd_neq_vm to check the overlap constraints.  */

static int
match_vd_neq_vm (const struct riscv_opcode *op,
		 insn_t insn,
		 int constraints,
		 const char **error)
{
  int vd = (insn & MASK_VD) >> OP_SH_VD;
  int vm = (insn & MASK_VMASK) >> OP_SH_VMASK;

  if (!constraints || error == NULL)
    return match_opcode (op, insn, 0, NULL);

  if (!vm && vm == vd)
    *error = "illegal operands vd cannot overlap vm";
  else
    return match_opcode (op, insn, 0, NULL);
  return 0;
}

static int
match_vls_nf_rv (const struct riscv_opcode *op,
		 insn_t insn,
		 int constraints,
		 const char **error)
{
  int vd = (insn & MASK_VD) >> OP_SH_VD;
  int nf = ((insn & (0x7 << 29) ) >> 29) + 1;

  if (!constraints || error == NULL)
    return match_opcode (op, insn, 0, NULL);

  if ((vd % nf) != 0)
    *error = "illegal operands vd must be multiple of nf";
  else
    return match_opcode (op, insn, 0, NULL);
  return 0;
}

static int
match_vmv_nf_rv (const struct riscv_opcode *op,
		 insn_t insn,
		 int constraints,
		 const char **error)
{
  int vd = (insn & MASK_VD) >> OP_SH_VD;
  int vs2 = (insn & MASK_VS2) >> OP_SH_VS2;
  int nf = ((insn & (0x7 << 15) ) >> 15) + 1;

  if (!constraints || error == NULL)
    return match_opcode (op, insn, 0, NULL);

  if ((vd % nf) != 0)
    *error = "illegal operands vd must be multiple of nf";
  else if ((vs2 % nf) != 0)
    *error = "illegal operands vs2 must be multiple of nf";
  else
    return match_opcode (op, insn, 0, NULL);
  return 0;
}

const struct riscv_opcode riscv_opcodes[] =
{
/* name,     xlen, isa,   operands, match, mask, match_func, pinfo.  */
{"unimp",       0, {"C", 0},   "",  0, 0xffffU,  match_opcode, INSN_ALIAS },
{"unimp",       0, {"I", 0},   "",  MATCH_CSRRW | (CSR_CYCLE << OP_SH_CSR), 0xffffffffU,  match_opcode, 0 }, /* csrw cycle, x0 */

/* Compressed instructions.  */
/* For bug-16573, move the compressed insns up can let objdump
   show the compressed insns with "c." suffix.  */
{"c.unimp",    0, {"C", 0},   "",  0, 0xffffU,  match_opcode, 0 },
{"c.ebreak",   0, {"C", 0},   "",  MATCH_C_EBREAK, MASK_C_EBREAK, match_opcode, 0 },
{"c.jr",       0, {"C", 0},   "d",  MATCH_C_JR, MASK_C_JR, match_rd_nonzero, INSN_BRANCH },
{"c.jalr",     0, {"C", 0},   "d",  MATCH_C_JALR, MASK_C_JALR, match_rd_nonzero, INSN_JSR },
{"c.j",        0, {"C", 0},   "Ca",  MATCH_C_J, MASK_C_J, match_opcode, INSN_BRANCH },
{"c.jal",     32, {"C", 0}, "Ca",  MATCH_C_JAL, MASK_C_JAL, match_opcode, INSN_JSR },
{"c.beqz",     0, {"C", 0},   "Cs,Cp",  MATCH_C_BEQZ, MASK_C_BEQZ, match_opcode, INSN_CONDBRANCH },
{"c.bnez",     0, {"C", 0},   "Cs,Cp",  MATCH_C_BNEZ, MASK_C_BNEZ, match_opcode, INSN_CONDBRANCH },
{"c.lwsp",     0, {"C", 0},   "d,Cm(Cc)",  MATCH_C_LWSP, MASK_C_LWSP, match_rd_nonzero, 0 },
{"c.lw",       0, {"C", 0},   "Ct,Ck(Cs)",  MATCH_C_LW, MASK_C_LW, match_opcode, INSN_DREF|INSN_4_BYTE },
{"c.swsp",     0, {"C", 0},   "CV,CM(Cc)",  MATCH_C_SWSP, MASK_C_SWSP, match_opcode, INSN_DREF|INSN_4_BYTE },
{"c.sw",       0, {"C", 0},   "Ct,Ck(Cs)",  MATCH_C_SW, MASK_C_SW, match_opcode, INSN_DREF|INSN_4_BYTE },
{"c.nop",      0, {"C", 0},   "",  MATCH_C_ADDI, 0xffff, match_opcode, INSN_ALIAS },
{"c.nop",      0, {"C", 0},   "Cj",  MATCH_C_ADDI, MASK_C_ADDI | MASK_RD, match_opcode, INSN_ALIAS },
{"c.mv",       0, {"C", 0},   "d,CV",  MATCH_C_MV, MASK_C_MV, match_c_add_with_hint, 0 },
{"c.lui",      0, {"C", 0},   "d,Cu",  MATCH_C_LUI, MASK_C_LUI, match_c_lui_with_hint, 0 },
{"c.li",       0, {"C", 0},   "d,Co",  MATCH_C_LI, MASK_C_LI, match_opcode, 0 },
{"c.addi4spn", 0, {"C", 0},   "Ct,Cc,CK", MATCH_C_ADDI4SPN, MASK_C_ADDI4SPN, match_c_addi4spn, 0 },
{"c.addi16sp", 0, {"C", 0},   "Cc,CL", MATCH_C_ADDI16SP, MASK_C_ADDI16SP, match_c_addi16sp, 0 },
{"c.addi",     0, {"C", 0},   "d,Co",  MATCH_C_ADDI, MASK_C_ADDI, match_opcode, 0 },
{"c.add",      0, {"C", 0},   "d,CV",  MATCH_C_ADD, MASK_C_ADD, match_c_add_with_hint, 0 },
{"c.sub",      0, {"C", 0},   "Cs,Ct",  MATCH_C_SUB, MASK_C_SUB, match_opcode, 0 },
{"c.and",      0, {"C", 0},   "Cs,Ct",  MATCH_C_AND, MASK_C_AND, match_opcode, 0 },
{"c.or",       0, {"C", 0},   "Cs,Ct",  MATCH_C_OR, MASK_C_OR, match_opcode, 0 },
{"c.xor",      0, {"C", 0},   "Cs,Ct",  MATCH_C_XOR, MASK_C_XOR, match_opcode, 0 },
{"c.slli",     0, {"C", 0},   "d,C>",  MATCH_C_SLLI, MASK_C_SLLI, match_c_slli, 0 },
{"c.srli",     0, {"C", 0},   "Cs,C>",  MATCH_C_SRLI, MASK_C_SRLI, match_c_slli, 0 },
{"c.srai",     0, {"C", 0},   "Cs,C>",  MATCH_C_SRAI, MASK_C_SRAI, match_c_slli, 0 },
{"c.slli64",   0, {"C", 0},   "d",  MATCH_C_SLLI64, MASK_C_SLLI64, match_c_slli64, 0 },
{"c.srli64",   0, {"C", 0},   "Cs",  MATCH_C_SRLI64, MASK_C_SRLI64, match_c_slli64, 0 },
{"c.srai64",   0, {"C", 0},   "Cs",  MATCH_C_SRAI64, MASK_C_SRAI64, match_c_slli64, 0 },
{"c.andi",     0, {"C", 0},   "Cs,Co",  MATCH_C_ANDI, MASK_C_ANDI, match_opcode, 0 },
{"c.addiw",   64, {"C", 0}, "d,Co",  MATCH_C_ADDIW, MASK_C_ADDIW, match_rd_nonzero, 0 },
{"c.addw",    64, {"C", 0}, "Cs,Ct",  MATCH_C_ADDW, MASK_C_ADDW, match_opcode, 0 },
{"c.subw",    64, {"C", 0}, "Cs,Ct",  MATCH_C_SUBW, MASK_C_SUBW, match_opcode, 0 },
{"c.ldsp",    64, {"C", 0}, "d,Cn(Cc)",  MATCH_C_LDSP, MASK_C_LDSP, match_rd_nonzero, INSN_DREF|INSN_8_BYTE },
{"c.ld",      64, {"C", 0}, "Ct,Cl(Cs)",  MATCH_C_LD, MASK_C_LD, match_opcode, INSN_DREF|INSN_8_BYTE },
{"c.sdsp",    64, {"C", 0}, "CV,CN(Cc)",  MATCH_C_SDSP, MASK_C_SDSP, match_opcode, INSN_DREF|INSN_8_BYTE },
{"c.sd",      64, {"C", 0}, "Ct,Cl(Cs)",  MATCH_C_SD, MASK_C_SD, match_opcode, INSN_DREF|INSN_8_BYTE },
{"c.fldsp",    0, {"D", "C", 0},   "D,Cn(Cc)",  MATCH_C_FLDSP, MASK_C_FLDSP, match_opcode, INSN_DREF|INSN_8_BYTE },
{"c.fld",      0, {"D", "C", 0},   "CD,Cl(Cs)",  MATCH_C_FLD, MASK_C_FLD, match_opcode, INSN_DREF|INSN_8_BYTE },
{"c.fsdsp",    0, {"D", "C", 0},   "CT,CN(Cc)",  MATCH_C_FSDSP, MASK_C_FSDSP, match_opcode, INSN_DREF|INSN_8_BYTE },
{"c.fsd",      0, {"D", "C", 0},   "CD,Cl(Cs)",  MATCH_C_FSD, MASK_C_FSD, match_opcode, INSN_DREF|INSN_8_BYTE },
{"c.flwsp",   32, {"F", "C", 0}, "D,Cm(Cc)",  MATCH_C_FLWSP, MASK_C_FLWSP, match_opcode, INSN_DREF|INSN_4_BYTE },
{"c.flw",     32, {"F", "C", 0}, "CD,Ck(Cs)",  MATCH_C_FLW, MASK_C_FLW, match_opcode, INSN_DREF|INSN_4_BYTE },
{"c.fswsp",   32, {"F", "C", 0}, "CT,CM(Cc)",  MATCH_C_FSWSP, MASK_C_FSWSP, match_opcode, INSN_DREF|INSN_4_BYTE },
{"c.fsw",     32, {"F", "C", 0}, "CD,Ck(Cs)",  MATCH_C_FSW, MASK_C_FSW, match_opcode, INSN_DREF|INSN_4_BYTE },

/* RVI instructions and RVC instructions without "c." suffix.  */
{"ebreak",      0, {"C", 0},   "",  MATCH_C_EBREAK, MASK_C_EBREAK, match_opcode, INSN_ALIAS },
{"ebreak",      0, {"I", 0},   "",    MATCH_EBREAK, MASK_EBREAK, match_opcode, 0 },
{"sbreak",      0, {"C", 0},   "",  MATCH_C_EBREAK, MASK_C_EBREAK, match_opcode, INSN_ALIAS },
{"sbreak",      0, {"I", 0},   "",    MATCH_EBREAK, MASK_EBREAK, match_opcode, INSN_ALIAS },
{"ret",         0, {"C", 0},   "",  MATCH_C_JR | (X_RA << OP_SH_RD), MASK_C_JR | MASK_RD, match_opcode, INSN_ALIAS|INSN_BRANCH },
{"ret",         0, {"I", 0},   "",  MATCH_JALR | (X_RA << OP_SH_RS1), MASK_JALR | MASK_RD | MASK_RS1 | MASK_IMM, match_opcode, INSN_ALIAS|INSN_BRANCH },
{"jr",          0, {"C", 0},   "d",  MATCH_C_JR, MASK_C_JR, match_rd_nonzero, INSN_ALIAS|INSN_BRANCH },
{"jr",          0, {"I", 0},   "s",  MATCH_JALR, MASK_JALR | MASK_RD | MASK_IMM, match_opcode, INSN_ALIAS|INSN_BRANCH },
{"jr",          0, {"I", 0},   "o(s)",  MATCH_JALR, MASK_JALR | MASK_RD, match_opcode, INSN_ALIAS|INSN_BRANCH },
{"jr",          0, {"I", 0},   "s,j",  MATCH_JALR, MASK_JALR | MASK_RD, match_opcode, INSN_ALIAS|INSN_BRANCH },
{"jalr",        0, {"C", 0},   "d",  MATCH_C_JALR, MASK_C_JALR, match_rd_nonzero, INSN_ALIAS|INSN_JSR },
{"jalr",        0, {"I", 0},   "s",  MATCH_JALR | (X_RA << OP_SH_RD), MASK_JALR | MASK_RD | MASK_IMM, match_opcode, INSN_ALIAS|INSN_JSR },
{"jalr",        0, {"I", 0},   "o(s)",  MATCH_JALR | (X_RA << OP_SH_RD), MASK_JALR | MASK_RD, match_opcode, INSN_ALIAS|INSN_JSR },
{"jalr",        0, {"I", 0},   "s,j",  MATCH_JALR | (X_RA << OP_SH_RD), MASK_JALR | MASK_RD, match_opcode, INSN_ALIAS|INSN_JSR },
{"jalr",        0, {"I", 0},   "d,s",  MATCH_JALR, MASK_JALR | MASK_IMM, match_opcode, INSN_ALIAS|INSN_JSR },
{"jalr",        0, {"I", 0},   "d,o(s)",  MATCH_JALR, MASK_JALR, match_opcode, INSN_JSR },
{"jalr",        0, {"I", 0},   "d,s,j",  MATCH_JALR, MASK_JALR, match_opcode, INSN_JSR },
{"j",           0, {"C", 0},   "Ca",  MATCH_C_J, MASK_C_J, match_opcode, INSN_ALIAS|INSN_BRANCH },
{"j",           0, {"I", 0},   "a",  MATCH_JAL, MASK_JAL | MASK_RD, match_opcode, INSN_ALIAS|INSN_BRANCH },
{"jal",         0, {"I", 0},   "d,a",  MATCH_JAL, MASK_JAL, match_opcode, INSN_JSR },
{"jal",        32, {"C", 0},   "Ca",  MATCH_C_JAL, MASK_C_JAL, match_opcode, INSN_ALIAS|INSN_JSR },
{"jal",         0, {"I", 0},   "a",  MATCH_JAL | (X_RA << OP_SH_RD), MASK_JAL | MASK_RD, match_opcode, INSN_ALIAS|INSN_JSR },
{"call",        0, {"I", 0},   "d,c", (X_T1 << OP_SH_RS1), (int) M_CALL,  match_never, INSN_MACRO },
{"call",        0, {"I", 0},   "c", (X_RA << OP_SH_RS1) | (X_RA << OP_SH_RD), (int) M_CALL,  match_never, INSN_MACRO },
{"tail",        0, {"I", 0},   "c", (X_T1 << OP_SH_RS1), (int) M_CALL,  match_never, INSN_MACRO },
{"jump",        0, {"I", 0},   "c,s", 0, (int) M_CALL,  match_never, INSN_MACRO },
{"nop",         0, {"C", 0},   "",  MATCH_C_ADDI, 0xffff, match_opcode, INSN_ALIAS },
{"nop",         0, {"I", 0},   "",         MATCH_ADDI, MASK_ADDI | MASK_RD | MASK_RS1 | MASK_IMM, match_opcode, INSN_ALIAS },
{"lui",         0, {"C", 0},   "d,Cu",  MATCH_C_LUI, MASK_C_LUI, match_c_lui, INSN_ALIAS },
{"lui",         0, {"I", 0},   "d,u",  MATCH_LUI, MASK_LUI, match_opcode, 0 },
{"li",          0, {"C", 0},   "d,Cv",  MATCH_C_LUI, MASK_C_LUI, match_c_lui, INSN_ALIAS },
{"li",          0, {"C", 0},   "d,Co",  MATCH_C_LI, MASK_C_LI, match_rd_nonzero, INSN_ALIAS },
{"li",          0, {"I", 0},   "d,j",      MATCH_ADDI, MASK_ADDI | MASK_RS1, match_opcode, INSN_ALIAS }, /* addi */
{"li",          0, {"I", 0},   "d,I",  0,    (int) M_LI,  match_never, INSN_MACRO },
{"mv",          0, {"C", 0},   "d,CV",  MATCH_C_MV, MASK_C_MV, match_c_add, INSN_ALIAS },
{"mv",          0, {"C", 0},   "d,Cz",  MATCH_C_LI, MASK_C_LI | MASK_RVC_IMM, match_rd_nonzero, INSN_ALIAS },
{"mv",          0, {"I", 0},   "d,s",  MATCH_ADDI, MASK_ADDI | MASK_IMM, match_opcode, INSN_ALIAS },
{"move",        0, {"C", 0},   "d,CV",  MATCH_C_MV, MASK_C_MV, match_c_add, INSN_ALIAS },
{"move",        0, {"C", 0},   "d,Cz",  MATCH_C_LI, MASK_C_LI | MASK_RVC_IMM, match_rd_nonzero, INSN_ALIAS },
{"move",        0, {"I", 0},   "d,s",  MATCH_ADDI, MASK_ADDI | MASK_IMM, match_opcode, INSN_ALIAS },
{"andi",        0, {"C", 0},   "Cs,Cw,Co",  MATCH_C_ANDI, MASK_C_ANDI, match_opcode, INSN_ALIAS },
{"andi",        0, {"I", 0},   "d,s,j",  MATCH_ANDI, MASK_ANDI, match_opcode, 0 },
{"and",         0, {"C", 0},   "Cs,Cw,Ct",  MATCH_C_AND, MASK_C_AND, match_opcode, INSN_ALIAS },
{"and",         0, {"C", 0},   "Cs,Ct,Cw",  MATCH_C_AND, MASK_C_AND, match_opcode, INSN_ALIAS },
{"and",         0, {"C", 0},   "Cs,Cw,Co",  MATCH_C_ANDI, MASK_C_ANDI, match_opcode, INSN_ALIAS },
{"and",         0, {"I", 0},   "d,s,t",  MATCH_AND, MASK_AND, match_opcode, 0 },
{"and",         0, {"I", 0},   "d,s,j",  MATCH_ANDI, MASK_ANDI, match_opcode, INSN_ALIAS },
{"beqz",        0, {"C", 0},   "Cs,Cp",  MATCH_C_BEQZ, MASK_C_BEQZ, match_opcode, INSN_ALIAS|INSN_CONDBRANCH },
{"beqz",        0, {"I", 0},   "s,p",  MATCH_BEQ, MASK_BEQ | MASK_RS2, match_opcode, INSN_ALIAS|INSN_CONDBRANCH },
{"beq",         0, {"C", 0},   "Cs,Cz,Cp",  MATCH_C_BEQZ, MASK_C_BEQZ, match_opcode, INSN_ALIAS|INSN_CONDBRANCH },
{"beq",         0, {"I", 0},   "s,t,p",  MATCH_BEQ, MASK_BEQ, match_opcode, INSN_CONDBRANCH },
{"blez",        0, {"I", 0},   "t,p",  MATCH_BGE, MASK_BGE | MASK_RS1, match_opcode, INSN_ALIAS|INSN_CONDBRANCH },
{"bgez",        0, {"I", 0},   "s,p",  MATCH_BGE, MASK_BGE | MASK_RS2, match_opcode, INSN_ALIAS|INSN_CONDBRANCH },
{"bge",         0, {"I", 0},   "s,t,p",  MATCH_BGE, MASK_BGE, match_opcode, INSN_CONDBRANCH },
{"bgeu",        0, {"I", 0},   "s,t,p",  MATCH_BGEU, MASK_BGEU, match_opcode, INSN_CONDBRANCH },
{"ble",         0, {"I", 0},   "t,s,p",  MATCH_BGE, MASK_BGE, match_opcode, INSN_ALIAS|INSN_CONDBRANCH },
{"bleu",        0, {"I", 0},   "t,s,p",  MATCH_BGEU, MASK_BGEU, match_opcode, INSN_ALIAS|INSN_CONDBRANCH },
{"bltz",        0, {"I", 0},   "s,p",  MATCH_BLT, MASK_BLT | MASK_RS2, match_opcode, INSN_ALIAS|INSN_CONDBRANCH },
{"bgtz",        0, {"I", 0},   "t,p",  MATCH_BLT, MASK_BLT | MASK_RS1, match_opcode, INSN_ALIAS|INSN_CONDBRANCH },
{"blt",         0, {"I", 0},   "s,t,p",  MATCH_BLT, MASK_BLT, match_opcode, INSN_CONDBRANCH },
{"bltu",        0, {"I", 0},   "s,t,p",  MATCH_BLTU, MASK_BLTU, match_opcode, INSN_CONDBRANCH },
{"bgt",         0, {"I", 0},   "t,s,p",  MATCH_BLT, MASK_BLT, match_opcode, INSN_ALIAS|INSN_CONDBRANCH },
{"bgtu",        0, {"I", 0},   "t,s,p",  MATCH_BLTU, MASK_BLTU, match_opcode, INSN_ALIAS|INSN_CONDBRANCH },
{"bnez",        0, {"C", 0},   "Cs,Cp",  MATCH_C_BNEZ, MASK_C_BNEZ, match_opcode, INSN_ALIAS|INSN_CONDBRANCH },
{"bnez",        0, {"I", 0},   "s,p",  MATCH_BNE, MASK_BNE | MASK_RS2, match_opcode, INSN_ALIAS|INSN_CONDBRANCH },
{"bne",         0, {"C", 0},   "Cs,Cz,Cp",  MATCH_C_BNEZ, MASK_C_BNEZ, match_opcode, INSN_ALIAS|INSN_CONDBRANCH },
{"bne",         0, {"I", 0},   "s,t,p",  MATCH_BNE, MASK_BNE, match_opcode, INSN_CONDBRANCH },
{"addi",        0, {"C", 0},   "Ct,Cc,CK", MATCH_C_ADDI4SPN, MASK_C_ADDI4SPN, match_c_addi4spn, INSN_ALIAS },
{"addi",        0, {"C", 0},   "d,CU,Cj",  MATCH_C_ADDI, MASK_C_ADDI, match_rd_nonzero, INSN_ALIAS },
{"addi",        0, {"C", 0},   "d,Cz,Co",  MATCH_C_LI, MASK_C_LI, match_rd_nonzero, INSN_ALIAS },
{"addi",        0, {"C", 0},   "d,CU,z",    MATCH_C_NOP, MASK_C_ADDI | MASK_RVC_IMM, match_c_nop, INSN_ALIAS },
{"addi",        0, {"C", 0},   "d,CV,z",    MATCH_C_MV, MASK_C_MV, match_c_add, INSN_ALIAS },
{"addi",        0, {"C", 0},   "Cc,Cc,CL", MATCH_C_ADDI16SP, MASK_C_ADDI16SP, match_c_addi16sp, INSN_ALIAS },
{"addi",        0, {"C", 0},   "d,Cz,Co",  MATCH_C_LI, MASK_C_LI, match_rd_nonzero, INSN_ALIAS },
{"addi",        0, {"I", 0},   "d,s,j",  MATCH_ADDI, MASK_ADDI, match_opcode, 0 },
{"add",         0, {"C", 0},   "d,CU,CV",  MATCH_C_ADD, MASK_C_ADD, match_c_add, INSN_ALIAS },
{"add",         0, {"C", 0},   "d,CV,CU",  MATCH_C_ADD, MASK_C_ADD, match_c_add, INSN_ALIAS },
{"add",         0, {"C", 0},   "d,CU,Co",  MATCH_C_ADDI, MASK_C_ADDI, match_rd_nonzero, INSN_ALIAS },
{"add",         0, {"C", 0},   "Ct,Cc,CK", MATCH_C_ADDI4SPN, MASK_C_ADDI4SPN, match_c_addi4spn, INSN_ALIAS },
{"add",         0, {"C", 0},   "Cc,Cc,CL", MATCH_C_ADDI16SP, MASK_C_ADDI16SP, match_c_addi16sp, INSN_ALIAS },
{"add",         0, {"C", 0},   "d,Cz,CV",  MATCH_C_MV, MASK_C_MV, match_c_add, INSN_ALIAS },
{"add",         0, {"C", 0},   "d,CV,Cz",  MATCH_C_MV, MASK_C_MV, match_c_add, INSN_ALIAS },
{"add",         0, {"I", 0},   "d,s,t",  MATCH_ADD, MASK_ADD, match_opcode, 0 },
/* This is used for TLS, where the fourth arg is %tprel_add, to get a reloc
   applied to an add instruction, for relaxation to use.  */
{"add",         0, {"I", 0},   "d,s,t,1",MATCH_ADD, MASK_ADD, match_opcode, 0 },
{"add",         0, {"I", 0},   "d,s,j",  MATCH_ADDI, MASK_ADDI, match_opcode, INSN_ALIAS },
{"la",          0, {"I", 0},   "d,B",  0,    (int) M_LA,  match_never, INSN_MACRO },
{"lla",         0, {"I", 0},   "d,B",  0,    (int) M_LLA,  match_never, INSN_MACRO },
{"la.tls.gd",   0, {"I", 0},   "d,A",  0,    (int) M_LA_TLS_GD,  match_never, INSN_MACRO },
{"la.tls.ie",   0, {"I", 0},   "d,A",  0,    (int) M_LA_TLS_IE,  match_never, INSN_MACRO },
{"neg",         0, {"I", 0},   "d,t",  MATCH_SUB, MASK_SUB | MASK_RS1, match_opcode, INSN_ALIAS }, /* sub 0 */
{"slli",        0, {"C", 0},   "d,CU,C>",  MATCH_C_SLLI, MASK_C_SLLI, match_slli_as_c_slli, INSN_ALIAS },
{"slli",        0, {"I", 0},   "d,s,>",   MATCH_SLLI, MASK_SLLI, match_opcode, 0 },
{"sll",         0, {"C", 0},   "d,CU,C>",  MATCH_C_SLLI, MASK_C_SLLI, match_slli_as_c_slli, INSN_ALIAS },
{"sll",         0, {"I", 0},   "d,s,t",   MATCH_SLL, MASK_SLL, match_opcode, 0 },
{"sll",         0, {"I", 0},   "d,s,>",   MATCH_SLLI, MASK_SLLI, match_opcode, INSN_ALIAS },
{"srli",        0, {"C", 0},   "Cs,Cw,C>",  MATCH_C_SRLI, MASK_C_SRLI, match_srxi_as_c_srxi, INSN_ALIAS },
{"srli",        0, {"I", 0},   "d,s,>",   MATCH_SRLI, MASK_SRLI, match_opcode, 0 },
{"srl",         0, {"C", 0},   "Cs,Cw,C>",  MATCH_C_SRLI, MASK_C_SRLI, match_srxi_as_c_srxi, INSN_ALIAS },
{"srl",         0, {"I", 0},   "d,s,t",   MATCH_SRL, MASK_SRL, match_opcode, 0 },
{"srl",         0, {"I", 0},   "d,s,>",   MATCH_SRLI, MASK_SRLI, match_opcode, INSN_ALIAS },
{"srai",        0, {"C", 0},   "Cs,Cw,C>",  MATCH_C_SRAI, MASK_C_SRAI, match_srxi_as_c_srxi, INSN_ALIAS },
{"srai",        0, {"I", 0},   "d,s,>",   MATCH_SRAI, MASK_SRAI, match_opcode, 0 },
{"sra",         0, {"C", 0},   "Cs,Cw,C>",  MATCH_C_SRAI, MASK_C_SRAI, match_srxi_as_c_srxi, INSN_ALIAS },
{"sra",         0, {"I", 0},   "d,s,t",   MATCH_SRA, MASK_SRA, match_opcode, 0 },
{"sra",         0, {"I", 0},   "d,s,>",   MATCH_SRAI, MASK_SRAI, match_opcode, INSN_ALIAS },
{"sub",         0, {"C", 0},   "Cs,Cw,Ct",  MATCH_C_SUB, MASK_C_SUB, match_opcode, INSN_ALIAS },
{"sub",         0, {"I", 0},   "d,s,t",  MATCH_SUB, MASK_SUB, match_opcode, 0 },
{"lb",          0, {"I", 0},   "d,o(s)",  MATCH_LB, MASK_LB, match_opcode, INSN_DREF|INSN_1_BYTE },
{"lb",          0, {"I", 0},   "d,A",  0, (int) M_LB, match_never, INSN_MACRO },
{"lbu",         0, {"I", 0},   "d,o(s)",  MATCH_LBU, MASK_LBU, match_opcode, INSN_DREF|INSN_1_BYTE },
{"lbu",         0, {"I", 0},   "d,A",  0, (int) M_LBU, match_never, INSN_MACRO },
{"lh",          0, {"I", 0},   "d,o(s)",  MATCH_LH, MASK_LH, match_opcode, INSN_DREF|INSN_2_BYTE },
{"lh",          0, {"I", 0},   "d,A",  0, (int) M_LH, match_never, INSN_MACRO },
{"lhu",         0, {"I", 0},   "d,o(s)",  MATCH_LHU, MASK_LHU, match_opcode, INSN_DREF|INSN_2_BYTE },
{"lhu",         0, {"I", 0},   "d,A",  0, (int) M_LHU, match_never, INSN_MACRO },
{"lw",          0, {"C", 0},   "d,Cm(Cc)",  MATCH_C_LWSP, MASK_C_LWSP, match_rd_nonzero, INSN_ALIAS|INSN_DREF|INSN_4_BYTE },
{"lw",          0, {"C", 0},   "Ct,Ck(Cs)",  MATCH_C_LW, MASK_C_LW, match_opcode, INSN_ALIAS|INSN_DREF|INSN_4_BYTE },
{"lw",          0, {"I", 0},   "d,o(s)",  MATCH_LW, MASK_LW, match_opcode, INSN_DREF|INSN_4_BYTE },
{"lw",          0, {"I", 0},   "d,A",  0, (int) M_LW, match_never, INSN_MACRO },
{"not",         0, {"I", 0},   "d,s",  MATCH_XORI | MASK_IMM, MASK_XORI | MASK_IMM, match_opcode, INSN_ALIAS },
{"ori",         0, {"I", 0},   "d,s,j",  MATCH_ORI, MASK_ORI, match_opcode, 0 },
{"or",          0, {"C", 0},   "Cs,Cw,Ct",  MATCH_C_OR, MASK_C_OR, match_opcode, INSN_ALIAS },
{"or",          0, {"C", 0},   "Cs,Ct,Cw",  MATCH_C_OR, MASK_C_OR, match_opcode, INSN_ALIAS },
{"or",          0, {"I", 0},   "d,s,t",  MATCH_OR, MASK_OR, match_opcode, 0 },
{"or",          0, {"I", 0},   "d,s,j",  MATCH_ORI, MASK_ORI, match_opcode, INSN_ALIAS },
{"auipc",       0, {"I", 0},   "d,u",  MATCH_AUIPC, MASK_AUIPC, match_opcode, 0 },
{"seqz",        0, {"I", 0},   "d,s",  MATCH_SLTIU | ENCODE_ITYPE_IMM (1), MASK_SLTIU | MASK_IMM, match_opcode, INSN_ALIAS },
{"snez",        0, {"I", 0},   "d,t",  MATCH_SLTU, MASK_SLTU | MASK_RS1, match_opcode, INSN_ALIAS },
{"sltz",        0, {"I", 0},   "d,s",  MATCH_SLT, MASK_SLT | MASK_RS2, match_opcode, INSN_ALIAS },
{"sgtz",        0, {"I", 0},   "d,t",  MATCH_SLT, MASK_SLT | MASK_RS1, match_opcode, INSN_ALIAS },
{"slti",        0, {"I", 0},   "d,s,j",  MATCH_SLTI, MASK_SLTI, match_opcode, 0 },
{"slt",         0, {"I", 0},   "d,s,t",  MATCH_SLT, MASK_SLT, match_opcode, 0 },
{"slt",         0, {"I", 0},   "d,s,j",  MATCH_SLTI, MASK_SLTI, match_opcode, INSN_ALIAS },
{"sltiu",       0, {"I", 0},   "d,s,j",  MATCH_SLTIU, MASK_SLTIU, match_opcode, 0 },
{"sltu",        0, {"I", 0},   "d,s,t",  MATCH_SLTU, MASK_SLTU, match_opcode, 0 },
{"sltu",        0, {"I", 0},   "d,s,j",  MATCH_SLTIU, MASK_SLTIU, match_opcode, INSN_ALIAS },
{"sgt",         0, {"I", 0},   "d,t,s",  MATCH_SLT, MASK_SLT, match_opcode, INSN_ALIAS },
{"sgtu",        0, {"I", 0},   "d,t,s",  MATCH_SLTU, MASK_SLTU, match_opcode, INSN_ALIAS },
{"sb",          0, {"I", 0},   "t,q(s)",  MATCH_SB, MASK_SB, match_opcode, INSN_DREF|INSN_1_BYTE },
{"sb",          0, {"I", 0},   "t,A,s",  0, (int) M_SB, match_never, INSN_MACRO },
{"sh",          0, {"I", 0},   "t,q(s)",  MATCH_SH, MASK_SH, match_opcode, INSN_DREF|INSN_2_BYTE },
{"sh",          0, {"I", 0},   "t,A,s",  0, (int) M_SH, match_never, INSN_MACRO },
{"sw",          0, {"C", 0},   "CV,CM(Cc)",  MATCH_C_SWSP, MASK_C_SWSP, match_opcode, INSN_ALIAS|INSN_DREF|INSN_4_BYTE },
{"sw",          0, {"C", 0},   "Ct,Ck(Cs)",  MATCH_C_SW, MASK_C_SW, match_opcode, INSN_ALIAS|INSN_DREF|INSN_4_BYTE },
{"sw",          0, {"I", 0},   "t,q(s)",  MATCH_SW, MASK_SW, match_opcode, INSN_DREF|INSN_4_BYTE },
{"sw",          0, {"I", 0},   "t,A,s",  0, (int) M_SW, match_never, INSN_MACRO },
{"fence",       0, {"I", 0},   "",  MATCH_FENCE | MASK_PRED | MASK_SUCC, MASK_FENCE | MASK_RD | MASK_RS1 | MASK_IMM, match_opcode, INSN_ALIAS },
{"fence",       0, {"I", 0},   "P,Q",  MATCH_FENCE, MASK_FENCE | MASK_RD | MASK_RS1 | (MASK_IMM & ~MASK_PRED & ~MASK_SUCC), match_opcode, 0 },
{"fence.i",     0, {"I", 0},   "",  MATCH_FENCE_I, MASK_FENCE | MASK_RD | MASK_RS1 | MASK_IMM, match_opcode, 0 },
{"fence.tso",   0, {"I", 0},   "",  MATCH_FENCE_TSO, MASK_FENCE_TSO | MASK_RD | MASK_RS1, match_opcode, INSN_ALIAS },
{"rdcycle",     0, {"I", 0},   "d",  MATCH_RDCYCLE, MASK_RDCYCLE, match_opcode, INSN_ALIAS },
{"rdinstret",   0, {"I", 0},   "d",  MATCH_RDINSTRET, MASK_RDINSTRET, match_opcode, INSN_ALIAS },
{"rdtime",      0, {"I", 0},   "d",  MATCH_RDTIME, MASK_RDTIME, match_opcode, INSN_ALIAS },
{"rdcycleh",   32, {"I", 0},   "d",  MATCH_RDCYCLEH, MASK_RDCYCLEH, match_opcode, INSN_ALIAS },
{"rdinstreth", 32, {"I", 0},   "d",  MATCH_RDINSTRETH, MASK_RDINSTRETH, match_opcode, INSN_ALIAS },
{"rdtimeh",    32, {"I", 0},   "d",  MATCH_RDTIMEH, MASK_RDTIMEH, match_opcode, INSN_ALIAS },
{"ecall",       0, {"I", 0},   "",    MATCH_SCALL, MASK_SCALL, match_opcode, 0 },
{"scall",       0, {"I", 0},   "",    MATCH_SCALL, MASK_SCALL, match_opcode, 0 },
{"xori",        0, {"I", 0},   "d,s,j",  MATCH_XORI, MASK_XORI, match_opcode, 0 },
{"xor",         0, {"C", 0},   "Cs,Cw,Ct",  MATCH_C_XOR, MASK_C_XOR, match_opcode, INSN_ALIAS },
{"xor",         0, {"C", 0},   "Cs,Ct,Cw",  MATCH_C_XOR, MASK_C_XOR, match_opcode, INSN_ALIAS },
{"xor",         0, {"I", 0},   "d,s,t",  MATCH_XOR, MASK_XOR, match_opcode, 0 },
{"xor",         0, {"I", 0},   "d,s,j",  MATCH_XORI, MASK_XORI, match_opcode, INSN_ALIAS },
{"lwu",        64, {"I", 0}, "d,o(s)",  MATCH_LWU, MASK_LWU, match_opcode, INSN_DREF|INSN_4_BYTE },
{"lwu",        64, {"I", 0}, "d,A",  0, (int) M_LWU, match_never, INSN_MACRO },
{"ld",         64, {"C", 0}, "d,Cn(Cc)",  MATCH_C_LDSP, MASK_C_LDSP, match_rd_nonzero, INSN_ALIAS|INSN_DREF|INSN_8_BYTE },
{"ld",         64, {"C", 0}, "Ct,Cl(Cs)",  MATCH_C_LD, MASK_C_LD, match_opcode, INSN_ALIAS|INSN_DREF|INSN_8_BYTE },
{"ld",         64, {"I", 0}, "d,o(s)", MATCH_LD, MASK_LD, match_opcode, INSN_DREF|INSN_8_BYTE },
{"ld",         64, {"I", 0}, "d,A",  0, (int) M_LD, match_never, INSN_MACRO },
{"sd",         64, {"C", 0}, "CV,CN(Cc)",  MATCH_C_SDSP, MASK_C_SDSP, match_opcode, INSN_ALIAS|INSN_DREF|INSN_8_BYTE },
{"sd",         64, {"C", 0}, "Ct,Cl(Cs)",  MATCH_C_SD, MASK_C_SD, match_opcode, INSN_ALIAS|INSN_DREF|INSN_8_BYTE },
{"sd",         64, {"I", 0}, "t,q(s)",  MATCH_SD, MASK_SD, match_opcode, INSN_DREF|INSN_8_BYTE },
{"sd",         64, {"I", 0}, "t,A,s",  0, (int) M_SD, match_never, INSN_MACRO },
{"sext.w",     64, {"C", 0}, "d,CU",  MATCH_C_ADDIW, MASK_C_ADDIW | MASK_RVC_IMM, match_rd_nonzero, INSN_ALIAS },
{"sext.w",     64, {"I", 0}, "d,s",  MATCH_ADDIW, MASK_ADDIW | MASK_IMM, match_opcode, INSN_ALIAS },
{"addiw",      64, {"C", 0}, "d,CU,Co",  MATCH_C_ADDIW, MASK_C_ADDIW, match_rd_nonzero, INSN_ALIAS },
{"addiw",      64, {"I", 0}, "d,s,j",  MATCH_ADDIW, MASK_ADDIW, match_opcode, 0 },
{"addw",       64, {"C", 0}, "Cs,Cw,Ct",  MATCH_C_ADDW, MASK_C_ADDW, match_opcode, INSN_ALIAS },
{"addw",       64, {"C", 0}, "Cs,Ct,Cw",  MATCH_C_ADDW, MASK_C_ADDW, match_opcode, INSN_ALIAS },
{"addw",       64, {"C", 0}, "d,CU,Co",  MATCH_C_ADDIW, MASK_C_ADDIW, match_rd_nonzero, INSN_ALIAS },
{"addw",       64, {"I", 0}, "d,s,t",  MATCH_ADDW, MASK_ADDW, match_opcode, 0 },
{"addw",       64, {"I", 0}, "d,s,j",  MATCH_ADDIW, MASK_ADDIW, match_opcode, INSN_ALIAS },
{"negw",       64, {"I", 0}, "d,t",  MATCH_SUBW, MASK_SUBW | MASK_RS1, match_opcode, INSN_ALIAS }, /* sub 0 */
{"slliw",      64, {"I", 0}, "d,s,<",   MATCH_SLLIW, MASK_SLLIW, match_opcode, 0 },
{"sllw",       64, {"I", 0}, "d,s,t",   MATCH_SLLW, MASK_SLLW, match_opcode, 0 },
{"sllw",       64, {"I", 0}, "d,s,<",   MATCH_SLLIW, MASK_SLLIW, match_opcode, INSN_ALIAS },
{"srliw",      64, {"I", 0}, "d,s,<",   MATCH_SRLIW, MASK_SRLIW, match_opcode, 0 },
{"srlw",       64, {"I", 0}, "d,s,t",   MATCH_SRLW, MASK_SRLW, match_opcode, 0 },
{"srlw",       64, {"I", 0}, "d,s,<",   MATCH_SRLIW, MASK_SRLIW, match_opcode, INSN_ALIAS },
{"sraiw",      64, {"I", 0}, "d,s,<",   MATCH_SRAIW, MASK_SRAIW, match_opcode, 0 },
{"sraw",       64, {"I", 0}, "d,s,t",   MATCH_SRAW, MASK_SRAW, match_opcode, 0 },
{"sraw",       64, {"I", 0}, "d,s,<",   MATCH_SRAIW, MASK_SRAIW, match_opcode, INSN_ALIAS },
{"subw",       64, {"C", 0}, "Cs,Cw,Ct",  MATCH_C_SUBW, MASK_C_SUBW, match_opcode, INSN_ALIAS },
{"subw",       64, {"I", 0}, "d,s,t",  MATCH_SUBW, MASK_SUBW, match_opcode, 0 },

/* NDS V5 Extension.  */
{"la.lo32",    64, {"XANDES", 0}, "d,A",  0, (int) M_LA_LO,  match_never, INSN_MACRO },
{"bfoz",        0, {"XANDES", 0}, "d,s,h,l",  MATCH_BFOZ, MASK_BFOZ, match_opcode, 0 },
{"bfos",        0, {"XANDES", 0}, "d,s,h,l",  MATCH_BFOS, MASK_BFOS, match_opcode, 0 },
{"beqc",        0, {"XANDES", 0}, "s,i,g",  MATCH_BEQC, MASK_BEQC, match_opcode, 0 },
{"bnec",        0, {"XANDES", 0}, "s,i,g",  MATCH_BNEC, MASK_BNEC, match_opcode, 0 },
{"bbc",         0, {"XANDES", 0}, "s,k,g",  MATCH_BBC, MASK_BBC, match_opcode, 0 },
{"bbs",         0, {"XANDES", 0}, "s,k,g",  MATCH_BBS, MASK_BBS, match_opcode, 0 },
{"lea.h",       0, {"XANDES", 0}, "d,s,t",  MATCH_LEA_H, MASK_LEA_H, match_opcode, 0 },
{"lea.w",       0, {"XANDES", 0}, "d,s,t",  MATCH_LEA_W, MASK_LEA_W, match_opcode, 0 },
{"lea.d",       0, {"XANDES", 0}, "d,s,t",  MATCH_LEA_D, MASK_LEA_D, match_opcode, 0 },
{"lea.b.ze",    0, {"XANDES", 0}, "d,s,t",  MATCH_LEA_B_ZE, MASK_LEA_B_ZE, match_opcode, 0 },
{"lea.h.ze",    0, {"XANDES", 0}, "d,s,t",  MATCH_LEA_H_ZE, MASK_LEA_H_ZE, match_opcode, 0 },
{"lea.w.ze",    0, {"XANDES", 0}, "d,s,t",  MATCH_LEA_W_ZE, MASK_LEA_W_ZE, match_opcode, 0 },
{"lea.d.ze",    0, {"XANDES", 0}, "d,s,t",  MATCH_LEA_D_ZE, MASK_LEA_D_ZE, match_opcode, 0 },
{"lbugp",       0, {"XANDES", 0}, "d,Gb", MATCH_LBUGP, MASK_LBUGP, match_opcode, 0},
{"lbgp",        0, {"XANDES", 0}, "d,Gb", MATCH_LBGP, MASK_LBGP, match_opcode, 0},
{"lhugp",       0, {"XANDES", 0}, "d,Gh", MATCH_LHUGP, MASK_LHUGP, match_opcode, 0},
{"lhgp",        0, {"XANDES", 0}, "d,Gh", MATCH_LHGP, MASK_LHGP, match_opcode, 0},
{"lwugp",      64, {"XANDES", 0}, "d,Gw", MATCH_LWUGP, MASK_LWUGP, match_opcode, 0},
{"lwgp",        0, {"XANDES", 0}, "d,Gw", MATCH_LWGP, MASK_LWGP, match_opcode, 0},
{"ldgp",       64, {"XANDES", 0}, "d,Gd", MATCH_LDGP, MASK_LDGP, match_opcode, 0},
{"sbgp",        0, {"XANDES", 0}, "t,Hb", MATCH_SBGP, MASK_SBGP, match_opcode, 0},
{"shgp",        0, {"XANDES", 0}, "t,Hh", MATCH_SHGP, MASK_SHGP, match_opcode, 0},
{"swgp",        0, {"XANDES", 0}, "t,Hw", MATCH_SWGP, MASK_SWGP, match_opcode, 0},
{"sdgp",       64, {"XANDES", 0}, "t,Hd", MATCH_SDGP, MASK_SDGP, match_opcode, 0},
{"addigp",      0, {"XANDES", 0}, "d,Gb", MATCH_ADDIGP, MASK_ADDIGP, match_opcode, 0},
{"ffb",         0, {"XANDES", 0}, "d,s,t",MATCH_FFB, MASK_FFB, match_opcode, 0 },
{"ffzmism",     0, {"XANDES", 0}, "d,s,t",MATCH_FFZMISM, MASK_FFZMISM, match_opcode, 0 },
{"ffmism",      0, {"XANDES", 0}, "d,s,t",MATCH_FFMISM, MASK_FFMISM, match_opcode, 0 },
{"flmism",      0, {"XANDES", 0}, "d,s,t",MATCH_FLMISM, MASK_FLMISM, match_opcode, 0 },
{"exec.it",     0, {"XANDES", "C", 0}, "Cet",  MATCH_C_EXECIT, MASK_C_EXECIT, match_opcode, 0 },
{"ex9.it",      0, {"XANDES", "C", 0}, "Cei",  MATCH_C_EX9IT, MASK_C_EX9IT, match_opcode, 0 },

{"fcvt.s.bf16", 0, {"XEBFHW", "XANDES", "F", 0}, "D,T",  MATCH_FCVT_S_BF16, MASK_FCVT_S_BF16, match_opcode, 0 },
{"fcvt.bf16.s", 0, {"XEBFHW", "XANDES", "F", 0}, "D,T",  MATCH_FCVT_BF16_S, MASK_FCVT_BF16_S, match_opcode, 0 },
{"vfwcvt.s.bf16",0,{"XANDES", "V", "F", 0}, "Vd,Vt", MATCH_VFWCVT_S_BF16, MASK_VFWCVT_S_BF16, match_opcode, 0},
{"vfncvt.bf16.s",0,{"XANDES", "V", "F", 0}, "Vd,Vt", MATCH_VFNCVT_BF16_S, MASK_VFNCVT_BF16_S, match_opcode, 0},

/* NDS V5 DSP Extension.  */
{"add8",        0, {"P", 0}, "d,s,t",     MATCH_ADD8, MASK_ADD8, match_opcode, 0 },
{"add16",       0, {"P", 0}, "d,s,t",     MATCH_ADD16, MASK_ADD16, match_opcode, 0 },
{"add64",       0, {"P", 0}, "nds_rdp,nds_rsp,nds_rtp", MATCH_ADD64, MASK_ADD64, match_opcode, 0 },
{"ave",         0, {"P", 0}, "d,s,t",     MATCH_AVE, MASK_AVE, match_opcode, 0 },
{"bitrev",      0, {"P", 0}, "d,s,t",     MATCH_BITREV, MASK_BITREV, match_opcode, 0 },
{"bitrevi",     0, {"P", 0}, "d,s,l",     MATCH_BITREVI, MASK_BITREVI, match_opcode, 0 },
{"bpick",       0, {"P", 0}, "d,s,t,nds_rc", MATCH_BPICK, MASK_BPICK, match_opcode, 0 },
{"clrs8",       0, {"P", 0}, "d,s",       MATCH_CLRS8, MASK_CLRS8, match_opcode, 0 },
{"clrs16",      0, {"P", 0}, "d,s",       MATCH_CLRS16, MASK_CLRS16, match_opcode, 0 },
{"clrs32",      0, {"P", 0}, "d,s",       MATCH_CLRS32, MASK_CLRS32, match_opcode, 0 },
{"clo8",        0, {"P", 0}, "d,s",       MATCH_CLO8, MASK_CLO8, match_opcode, 0 },
{"clo16",       0, {"P", 0}, "d,s",       MATCH_CLO16, MASK_CLO16, match_opcode, 0 },
{"clo32",       0, {"P", 0}, "d,s",       MATCH_CLO32, MASK_CLO32, match_opcode, 0 },
{"clz8",        0, {"P", 0}, "d,s",       MATCH_CLZ8, MASK_CLZ8, match_opcode, 0 },
{"clz16",       0, {"P", 0}, "d,s",       MATCH_CLZ16, MASK_CLZ16, match_opcode, 0 },
{"clz32",       0, {"P", 0}, "d,s",       MATCH_CLZ32, MASK_CLZ32, match_opcode, 0 },
{"cmpeq8",      0, {"P", 0}, "d,s,t",     MATCH_CMPEQ8, MASK_CMPEQ8, match_opcode, 0 },
{"cmpeq16",     0, {"P", 0}, "d,s,t",     MATCH_CMPEQ16, MASK_CMPEQ16, match_opcode, 0 },
{"cras16",      0, {"P", 0}, "d,s,t",     MATCH_CRAS16, MASK_CRAS16, match_opcode, 0 },
{"crsa16",      0, {"P", 0}, "d,s,t",     MATCH_CRSA16, MASK_CRSA16, match_opcode, 0 },
{"insb",        0, {"P", 0}, "d,s,nds_i3u", MATCH_INSB, MASK_INSB, match_opcode, 0 },
{"kabs8",       0, {"P", 0}, "d,s",       MATCH_KABS8, MASK_KABS8, match_opcode, 0 },
{"kabs16",      0, {"P", 0}, "d,s",       MATCH_KABS16, MASK_KABS16, match_opcode, 0 },
{"kabsw",       0, {"P", 0}, "d,s",       MATCH_KABSW, MASK_KABSW, match_opcode, 0 },
{"kadd8",       0, {"P", 0}, "d,s,t",     MATCH_KADD8, MASK_KADD8, match_opcode, 0 },
{"kadd16",      0, {"P", 0}, "d,s,t",     MATCH_KADD16, MASK_KADD16, match_opcode, 0 },
{"kadd64",      0, {"P", 0}, "nds_rdp,nds_rsp,nds_rtp", MATCH_KADD64, MASK_KADD64, match_opcode, 0 },
{"kaddh",       0, {"P", 0}, "d,s,t",     MATCH_KADDH, MASK_KADDH, match_opcode, 0 },
{"kaddw",       0, {"P", 0}, "d,s,t",     MATCH_KADDW, MASK_KADDW, match_opcode, 0 },
{"kcras16",     0, {"P", 0}, "d,s,t",     MATCH_KCRAS16, MASK_KCRAS16, match_opcode, 0 },
{"kcrsa16",     0, {"P", 0}, "d,s,t",     MATCH_KCRSA16, MASK_KCRSA16, match_opcode, 0 },
{"kdmbb",       0, {"P", 0}, "d,s,t",     MATCH_KDMBB, MASK_KDMBB, match_opcode, 0 },
{"kdmbt",       0, {"P", 0}, "d,s,t",     MATCH_KDMBT, MASK_KDMBT, match_opcode, 0 },
{"kdmtt",       0, {"P", 0}, "d,s,t",     MATCH_KDMTT, MASK_KDMTT, match_opcode, 0 },
{"kdmabb",      0, {"P", 0}, "d,s,t",     MATCH_KDMABB, MASK_KDMABB, match_opcode, 0 },
{"kdmabt",      0, {"P", 0}, "d,s,t",     MATCH_KDMABT, MASK_KDMABT, match_opcode, 0 },
{"kdmatt",      0, {"P", 0}, "d,s,t",     MATCH_KDMATT, MASK_KDMATT, match_opcode, 0 },
{"khm8",        0, {"P", 0}, "d,s,t",     MATCH_KHM8, MASK_KHM8, match_opcode, 0 },
{"khmx8",       0, {"P", 0}, "d,s,t",     MATCH_KHMX8, MASK_KHMX8, match_opcode, 0 },
{"khm16",       0, {"P", 0}, "d,s,t",     MATCH_KHM16, MASK_KHM16, match_opcode, 0 },
{"khmx16",      0, {"P", 0}, "d,s,t",     MATCH_KHMX16, MASK_KHMX16, match_opcode, 0 },
{"khmbb",       0, {"P", 0}, "d,s,t",     MATCH_KHMBB, MASK_KHMBB, match_opcode, 0 },
{"khmbt",       0, {"P", 0}, "d,s,t",     MATCH_KHMBT, MASK_KHMBT, match_opcode, 0 },
{"khmtt",       0, {"P", 0}, "d,s,t",     MATCH_KHMTT, MASK_KHMTT, match_opcode, 0 },
{"kmabb",       0, {"P", 0}, "d,s,t",     MATCH_KMABB, MASK_KMABB, match_opcode, 0 },
{"kmabt",       0, {"P", 0}, "d,s,t",     MATCH_KMABT, MASK_KMABT, match_opcode, 0 },
{"kmatt",       0, {"P", 0}, "d,s,t",     MATCH_KMATT, MASK_KMATT, match_opcode, 0 },
{"kmada",       0, {"P", 0}, "d,s,t",     MATCH_KMADA, MASK_KMADA, match_opcode, 0 },
{"kmaxda",      0, {"P", 0}, "d,s,t",     MATCH_KMAXDA, MASK_KMAXDA, match_opcode, 0 },
{"kmads",       0, {"P", 0}, "d,s,t",     MATCH_KMADS, MASK_KMADS, match_opcode, 0 },
{"kmadrs",      0, {"P", 0}, "d,s,t",     MATCH_KMADRS, MASK_KMADRS, match_opcode, 0 },
{"kmaxds",      0, {"P", 0}, "d,s,t",     MATCH_KMAXDS, MASK_KMAXDS, match_opcode, 0 },
{"kmar64",      0, {"P", 0}, "nds_rdp,s,t", MATCH_KMAR64, MASK_KMAR64, match_opcode, 0 },
{"kmda",        0, {"P", 0}, "d,s,t",     MATCH_KMDA, MASK_KMDA, match_opcode, 0 },
{"kmxda",       0, {"P", 0}, "d,s,t",     MATCH_KMXDA, MASK_KMXDA, match_opcode, 0 },
{"kmmac",       0, {"P", 0}, "d,s,t",     MATCH_KMMAC, MASK_KMMAC, match_opcode, 0 },
{"kmmac.u",     0, {"P", 0}, "d,s,t",     MATCH_KMMAC_U, MASK_KMMAC_U, match_opcode, 0 },
{"kmmawb",      0, {"P", 0}, "d,s,t",     MATCH_KMMAWB, MASK_KMMAWB, match_opcode, 0 },
{"kmmawb.u",    0, {"P", 0}, "d,s,t",     MATCH_KMMAWB_U, MASK_KMMAWB_U, match_opcode, 0 },
{"kmmawb2",     0, {"P", 0}, "d,s,t",     MATCH_KMMAWB2, MASK_KMMAWB2, match_opcode, 0 },
{"kmmawb2.u",   0, {"P", 0}, "d,s,t",     MATCH_KMMAWB2_U, MASK_KMMAWB2_U, match_opcode, 0 },
{"kmmawt",      0, {"P", 0}, "d,s,t",     MATCH_KMMAWT, MASK_KMMAWT, match_opcode, 0 },
{"kmmawt.u",    0, {"P", 0}, "d,s,t",     MATCH_KMMAWT_U, MASK_KMMAWT_U, match_opcode, 0 },
{"kmmawt2",     0, {"P", 0}, "d,s,t",     MATCH_KMMAWT2, MASK_KMMAWT2, match_opcode, 0 },
{"kmmawt2.u",   0, {"P", 0}, "d,s,t",     MATCH_KMMAWT2_U, MASK_KMMAWT2_U, match_opcode, 0 },
{"kmmsb",       0, {"P", 0}, "d,s,t",     MATCH_KMMSB, MASK_KMMSB, match_opcode, 0 },
{"kmmsb.u",     0, {"P", 0}, "d,s,t",     MATCH_KMMSB_U, MASK_KMMSB_U, match_opcode, 0 },
{"kmmwb2",      0, {"P", 0}, "d,s,t",     MATCH_KMMWB2, MASK_KMMWB2, match_opcode, 0 },
{"kmmwb2.u",    0, {"P", 0}, "d,s,t",     MATCH_KMMWB2_U, MASK_KMMWB2_U, match_opcode, 0 },
{"kmmwt2",      0, {"P", 0}, "d,s,t",     MATCH_KMMWT2, MASK_KMMWT2, match_opcode, 0 },
{"kmmwt2.u",    0, {"P", 0}, "d,s,t",     MATCH_KMMWT2_U, MASK_KMMWT2_U, match_opcode, 0 },
{"kmsda",       0, {"P", 0}, "d,s,t",     MATCH_KMSDA, MASK_KMSDA, match_opcode, 0 },
{"kmsxda",      0, {"P", 0}, "d,s,t",     MATCH_KMSXDA, MASK_KMSXDA, match_opcode, 0 },
{"kmsr64",      0, {"P", 0}, "nds_rdp,s,t", MATCH_KMSR64, MASK_KMSR64, match_opcode, 0 },
{"ksllw",       0, {"P", 0}, "d,s,t",     MATCH_KSLLW, MASK_KSLLW, match_opcode, 0 },
{"kslliw",      0, {"P", 0}, "d,s,nds_i5u", MATCH_KSLLIW, MASK_KSLLIW, match_opcode, 0 },
{"ksll8",       0, {"P", 0}, "d,s,t",     MATCH_KSLL8, MASK_KSLL8, match_opcode, 0 },
{"kslli8",      0, {"P", 0}, "d,s,nds_i3u", MATCH_KSLLI8, MASK_KSLLI8, match_opcode, 0 },
{"ksll16",      0, {"P", 0}, "d,s,t",     MATCH_KSLL16, MASK_KSLL16, match_opcode, 0 },
{"kslli16",     0, {"P", 0}, "d,s,nds_i4u", MATCH_KSLLI16, MASK_KSLLI16, match_opcode, 0 },
{"kslra8",      0, {"P", 0}, "d,s,t",     MATCH_KSLRA8, MASK_KSLRA8, match_opcode, 0 },
{"kslra8.u",    0, {"P", 0}, "d,s,t",     MATCH_KSLRA8_U, MASK_KSLRA8_U, match_opcode, 0 },
{"kslra16",     0, {"P", 0}, "d,s,t",     MATCH_KSLRA16, MASK_KSLRA16, match_opcode, 0 },
{"kslra16.u",   0, {"P", 0}, "d,s,t",     MATCH_KSLRA16_U, MASK_KSLRA16_U, match_opcode, 0 },
{"kslraw",      0, {"P", 0}, "d,s,t",     MATCH_KSLRAW, MASK_KSLRAW, match_opcode, 0 },
{"kslraw.u",    0, {"P", 0}, "d,s,t",     MATCH_KSLRAW_U, MASK_KSLRAW_U, match_opcode, 0 },
{"kstas16",     0, {"P", 0}, "d,s,t",     MATCH_KSTAS16, MASK_KSTAS16, match_opcode, 0 },
{"kstsa16",     0, {"P", 0}, "d,s,t",     MATCH_KSTSA16, MASK_KSTSA16, match_opcode, 0 },
{"ksub8",       0, {"P", 0}, "d,s,t",     MATCH_KSUB8, MASK_KSUB8, match_opcode, 0 },
{"ksub16",      0, {"P", 0}, "d,s,t",     MATCH_KSUB16, MASK_KSUB16, match_opcode, 0 },
{"ksub64",      0, {"P", 0}, "nds_rdp,nds_rsp,nds_rtp", MATCH_KSUB64, MASK_KSUB64, match_opcode, 0 },
{"ksubh",       0, {"P", 0}, "d,s,t",     MATCH_KSUBH, MASK_KSUBH, match_opcode, 0 },
{"ksubw",       0, {"P", 0}, "d,s,t",     MATCH_KSUBW, MASK_KSUBW, match_opcode, 0 },
{"kwmmul",      0, {"P", 0}, "d,s,t",     MATCH_KWMMUL, MASK_KWMMUL, match_opcode, 0 },
{"kwmmul.u",    0, {"P", 0}, "d,s,t",     MATCH_KWMMUL_U, MASK_KWMMUL_U, match_opcode, 0 },
{"mtlbi",       0, {"P", 0}, "nds_i15s",  MATCH_MTLBI, MASK_MTLBI, match_opcode, 0 },
{"mtlei",       0, {"P", 0}, "nds_i15s",  MATCH_MTLEI, MASK_MTLEI, match_opcode, 0 },
{"maddr32",     0, {"P", 0}, "d,s,t",     MATCH_MADDR32, MASK_MADDR32, match_opcode, 0 },
{"maxw",        0, {"P", 0}, "d,s,t",     MATCH_MAXW, MASK_MAXW, match_opcode, 0 },
{"minw",        0, {"P", 0}, "d,s,t",     MATCH_MINW, MASK_MINW, match_opcode, 0 },
{"msubr32",     0, {"P", 0}, "d,s,t",     MATCH_MSUBR32, MASK_MSUBR32, match_opcode, 0 },
{"mulr64",      0, {"P", 0}, "nds_rdp,s,t", MATCH_MULR64, MASK_MULR64, match_opcode, 0 },
{"mulsr64",     0, {"P", 0}, "nds_rdp,s,t", MATCH_MULSR64, MASK_MULSR64, match_opcode, 0 },
{"pbsad",       0, {"P", 0}, "d,s,t",     MATCH_PBSAD, MASK_PBSAD, match_opcode, 0 },
{"pbsada",      0, {"P", 0}, "d,s,t",     MATCH_PBSADA, MASK_PBSADA, match_opcode, 0 },
{"pkbb16",      0, {"P", 0}, "d,s,t",     MATCH_PKBB16, MASK_PKBB16, match_opcode, 0 },
{"pkbt16",      0, {"P", 0}, "d,s,t",     MATCH_PKBT16, MASK_PKBT16, match_opcode, 0 },
{"pktt16",      0, {"P", 0}, "d,s,t",     MATCH_PKTT16, MASK_PKTT16, match_opcode, 0 },
{"pktb16",      0, {"P", 0}, "d,s,t",     MATCH_PKTB16, MASK_PKTB16, match_opcode, 0 },
{"radd8",       0, {"P", 0}, "d,s,t",     MATCH_RADD8, MASK_RADD8, match_opcode, 0 },
{"radd16",      0, {"P", 0}, "d,s,t",     MATCH_RADD16, MASK_RADD16, match_opcode, 0 },
{"radd64",      0, {"P", 0}, "nds_rdp,nds_rsp,nds_rtp", MATCH_RADD64, MASK_RADD64, match_opcode, 0 },
{"raddw",       0, {"P", 0}, "d,s,t",     MATCH_RADDW, MASK_RADDW, match_opcode, 0 },
{"rcras16",     0, {"P", 0}, "d,s,t",     MATCH_RCRAS16, MASK_RCRAS16, match_opcode, 0 },
{"rcrsa16",     0, {"P", 0}, "d,s,t",     MATCH_RCRSA16, MASK_RCRSA16, match_opcode, 0 },
{"rstas16",     0, {"P", 0}, "d,s,t",     MATCH_RSTAS16, MASK_RSTAS16, match_opcode, 0 },
{"rstsa16",     0, {"P", 0}, "d,s,t",     MATCH_RSTSA16, MASK_RSTSA16, match_opcode, 0 },
{"rsub8",       0, {"P", 0}, "d,s,t",     MATCH_RSUB8, MASK_RSUB8, match_opcode, 0 },
{"rsub16",      0, {"P", 0}, "d,s,t",     MATCH_RSUB16, MASK_RSUB16, match_opcode, 0 },
{"rsub64",      0, {"P", 0}, "nds_rdp,nds_rsp,nds_rtp", MATCH_RSUB64, MASK_RSUB64, match_opcode, 0 },
{"rsubw",       0, {"P", 0}, "d,s,t",     MATCH_RSUBW, MASK_RSUBW, match_opcode, 0 },
{"sclip8",      0, {"P", 0}, "d,s,nds_i3u", MATCH_SCLIP8, MASK_SCLIP8, match_opcode, 0 },
{"sclip16",     0, {"P", 0}, "d,s,nds_i4u", MATCH_SCLIP16, MASK_SCLIP16, match_opcode, 0 },
{"sclip32",     0, {"P", 0}, "d,s,nds_i5u", MATCH_SCLIP32, MASK_SCLIP32, match_opcode, 0 },
{"scmple8",     0, {"P", 0}, "d,s,t",     MATCH_SCMPLE8, MASK_SCMPLE8, match_opcode, 0 },
{"scmple16",    0, {"P", 0}, "d,s,t",     MATCH_SCMPLE16, MASK_SCMPLE16, match_opcode, 0 },
{"scmplt8",     0, {"P", 0}, "d,s,t",     MATCH_SCMPLT8, MASK_SCMPLT8, match_opcode, 0 },
{"scmplt16",    0, {"P", 0}, "d,s,t",     MATCH_SCMPLT16, MASK_SCMPLT16, match_opcode, 0 },
{"sll8",        0, {"P", 0}, "d,s,t",     MATCH_SLL8, MASK_SLL8, match_opcode, 0 },
{"slli8",       0, {"P", 0}, "d,s,nds_i3u", MATCH_SLLI8, MASK_SLLI8, match_opcode, 0 },
{"sll16",       0, {"P", 0}, "d,s,t",     MATCH_SLL16, MASK_SLL16, match_opcode, 0 },
{"slli16",      0, {"P", 0}, "d,s,nds_i4u", MATCH_SLLI16, MASK_SLLI16, match_opcode, 0 },
{"smal",        0, {"P", 0}, "nds_rdp,nds_rsp,t", MATCH_SMAL, MASK_SMAL, match_opcode, 0 },
{"smalbb",      0, {"P", 0}, "nds_rdp,s,t", MATCH_SMALBB, MASK_SMALBB, match_opcode, 0 },
{"smalbt",      0, {"P", 0}, "nds_rdp,s,t", MATCH_SMALBT, MASK_SMALBT, match_opcode, 0 },
{"smaltt",      0, {"P", 0}, "nds_rdp,s,t", MATCH_SMALTT, MASK_SMALTT, match_opcode, 0 },
{"smalda",      0, {"P", 0}, "nds_rdp,s,t", MATCH_SMALDA, MASK_SMALDA, match_opcode, 0 },
{"smalxda",     0, {"P", 0}, "nds_rdp,s,t", MATCH_SMALXDA, MASK_SMALXDA, match_opcode, 0 },
{"smalds",      0, {"P", 0}, "nds_rdp,s,t", MATCH_SMALDS, MASK_SMALDS, match_opcode, 0 },
{"smaldrs",     0, {"P", 0}, "nds_rdp,s,t", MATCH_SMALDRS, MASK_SMALDRS, match_opcode, 0 },
{"smalxds",     0, {"P", 0}, "nds_rdp,s,t", MATCH_SMALXDS, MASK_SMALXDS, match_opcode, 0 },
{"smar64",      0, {"P", 0}, "nds_rdp,s,t", MATCH_SMAR64, MASK_SMAR64, match_opcode, 0 },
{"smaqa",       0, {"P", 0}, "d,s,t",     MATCH_SMAQA, MASK_SMAQA, match_opcode, 0 },
{"smaqa.su",    0, {"P", 0}, "d,s,t",     MATCH_SMAQA_SU, MASK_SMAQA_SU, match_opcode, 0 },
{"smax8",       0, {"P", 0}, "d,s,t",     MATCH_SMAX8, MASK_SMAX8, match_opcode, 0 },
{"smax16",      0, {"P", 0}, "d,s,t",     MATCH_SMAX16, MASK_SMAX16, match_opcode, 0 },
{"smbb16",      0, {"P", 0}, "d,s,t",     MATCH_SMBB16, MASK_SMBB16, match_opcode, 0 },
{"smbt16",      0, {"P", 0}, "d,s,t",     MATCH_SMBT16, MASK_SMBT16, match_opcode, 0 },
{"smtt16",      0, {"P", 0}, "d,s,t",     MATCH_SMTT16, MASK_SMTT16, match_opcode, 0 },
{"smds",        0, {"P", 0}, "d,s,t",     MATCH_SMDS, MASK_SMDS, match_opcode, 0 },
{"smdrs",       0, {"P", 0}, "d,s,t",     MATCH_SMDRS, MASK_SMDRS, match_opcode, 0 },
{"smxds",       0, {"P", 0}, "d,s,t",     MATCH_SMXDS, MASK_SMXDS, match_opcode, 0 },
{"smin8",       0, {"P", 0}, "d,s,t",     MATCH_SMIN8, MASK_SMIN8, match_opcode, 0 },
{"smin16",      0, {"P", 0}, "d,s,t",     MATCH_SMIN16, MASK_SMIN16, match_opcode, 0 },
{"smmul",       0, {"P", 0}, "d,s,t",     MATCH_SMMUL, MASK_SMMUL, match_opcode, 0 },
{"smmul.u",     0, {"P", 0}, "d,s,t",     MATCH_SMMUL_U, MASK_SMMUL_U, match_opcode, 0 },
{"smmwb",       0, {"P", 0}, "d,s,t",     MATCH_SMMWB, MASK_SMMWB, match_opcode, 0 },
{"smmwb.u",     0, {"P", 0}, "d,s,t",     MATCH_SMMWB_U, MASK_SMMWB_U, match_opcode, 0 },
{"smmwt",       0, {"P", 0}, "d,s,t",     MATCH_SMMWT, MASK_SMMWT, match_opcode, 0 },
{"smmwt.u",     0, {"P", 0}, "d,s,t",     MATCH_SMMWT_U, MASK_SMMWT_U, match_opcode, 0 },
{"smslda",      0, {"P", 0}, "nds_rdp,s,t", MATCH_SMSLDA, MASK_SMSLDA, match_opcode, 0 },
{"smslxda",     0, {"P", 0}, "nds_rdp,s,t", MATCH_SMSLXDA, MASK_SMSLXDA, match_opcode, 0 },
{"smsr64",      0, {"P", 0}, "nds_rdp,s,t", MATCH_SMSR64, MASK_SMSR64, match_opcode, 0 },
{"smul8",       0, {"P", 0}, "nds_rdp,s,t", MATCH_SMUL8, MASK_SMUL8, match_opcode, 0 },
{"smulx8",      0, {"P", 0}, "nds_rdp,s,t", MATCH_SMULX8, MASK_SMULX8, match_opcode, 0 },
{"smul16",      0, {"P", 0}, "nds_rdp,s,t", MATCH_SMUL16, MASK_SMUL16, match_opcode, 0 },
{"smulx16",     0, {"P", 0}, "nds_rdp,s,t", MATCH_SMULX16, MASK_SMULX16, match_opcode, 0 },
{"sra.u",       0, {"P", 0}, "d,s,t",     MATCH_SRA_U, MASK_SRA_U, match_opcode, 0 },
{"srai.u",      0, {"P", 0}, "d,s,nds_i6u", MATCH_SRAI_U, MASK_SRAI_U, match_opcode, 0 },
{"sra8",        0, {"P", 0}, "d,s,t",     MATCH_SRA8, MASK_SRA8, match_opcode, 0 },
{"sra8.u",      0, {"P", 0}, "d,s,t",     MATCH_SRA8_U, MASK_SRA8_U, match_opcode, 0 },
{"srai8",       0, {"P", 0}, "d,s,nds_i3u", MATCH_SRAI8, MASK_SRAI8, match_opcode, 0 },
{"srai8.u",     0, {"P", 0}, "d,s,nds_i3u", MATCH_SRAI8_U, MASK_SRAI8_U, match_opcode, 0 },
{"sra16",       0, {"P", 0}, "d,s,t",     MATCH_SRA16, MASK_SRA16, match_opcode, 0 },
{"sra16.u",     0, {"P", 0}, "d,s,t",     MATCH_SRA16_U, MASK_SRA16_U, match_opcode, 0 },
{"srai16",      0, {"P", 0}, "d,s,nds_i4u", MATCH_SRAI16, MASK_SRAI16, match_opcode, 0 },
{"srai16.u",    0, {"P", 0}, "d,s,nds_i4u", MATCH_SRAI16_U, MASK_SRAI16_U, match_opcode, 0 },
{"srl8",        0, {"P", 0}, "d,s,t",     MATCH_SRL8, MASK_SRL8, match_opcode, 0 },
{"srl8.u",      0, {"P", 0}, "d,s,t",     MATCH_SRL8_U, MASK_SRL8_U, match_opcode, 0 },
{"srli8",       0, {"P", 0}, "d,s,nds_i3u", MATCH_SRLI8, MASK_SRLI8, match_opcode, 0 },
{"srli8.u",     0, {"P", 0}, "d,s,nds_i3u", MATCH_SRLI8_U, MASK_SRLI8_U, match_opcode, 0 },
{"srl16",       0, {"P", 0}, "d,s,t",     MATCH_SRL16, MASK_SRL16, match_opcode, 0 },
{"srl16.u",     0, {"P", 0}, "d,s,t",     MATCH_SRL16_U, MASK_SRL16_U, match_opcode, 0 },
{"srli16",      0, {"P", 0}, "d,s,nds_i4u", MATCH_SRLI16, MASK_SRLI16, match_opcode, 0 },
{"srli16.u",    0, {"P", 0}, "d,s,nds_i4u", MATCH_SRLI16_U, MASK_SRLI16_U, match_opcode, 0 },
{"stas16",      0, {"P", 0}, "d,s,t",     MATCH_STAS16, MASK_STAS16, match_opcode, 0 },
{"stsa16",      0, {"P", 0}, "d,s,t",     MATCH_STSA16, MASK_STSA16, match_opcode, 0 },
{"sub8",        0, {"P", 0}, "d,s,t",     MATCH_SUB8, MASK_SUB8, match_opcode, 0 },
{"sub16",       0, {"P", 0}, "d,s,t",     MATCH_SUB16, MASK_SUB16, match_opcode, 0 },
{"sub64",       0, {"P", 0}, "nds_rdp,nds_rsp,nds_rtp", MATCH_SUB64, MASK_SUB64, match_opcode, 0 },
{"sunpkd810",   0, {"P", 0}, "d,s",       MATCH_SUNPKD810, MASK_SUNPKD810, match_opcode, 0 },
{"sunpkd820",   0, {"P", 0}, "d,s",       MATCH_SUNPKD820, MASK_SUNPKD820, match_opcode, 0 },
{"sunpkd830",   0, {"P", 0}, "d,s",       MATCH_SUNPKD830, MASK_SUNPKD830, match_opcode, 0 },
{"sunpkd831",   0, {"P", 0}, "d,s",       MATCH_SUNPKD831, MASK_SUNPKD831, match_opcode, 0 },
{"sunpkd832",   0, {"P", 0}, "d,s",       MATCH_SUNPKD832, MASK_SUNPKD832, match_opcode, 0 },
{"swap8",       0, {"P", 0}, "d,s",       MATCH_SWAP8, MASK_SWAP8, match_opcode, 0 },
{"swap16",      0, {"P", 0}, "d,s",       MATCH_SWAP16, MASK_SWAP16, match_opcode, 0 },
{"uclip8",      0, {"P", 0}, "d,s,nds_i3u", MATCH_UCLIP8, MASK_UCLIP8, match_opcode, 0 },
{"uclip16",     0, {"P", 0}, "d,s,nds_i4u", MATCH_UCLIP16, MASK_UCLIP16, match_opcode, 0 },
{"uclip32",     0, {"P", 0}, "d,s,nds_i5u", MATCH_UCLIP32, MASK_UCLIP32, match_opcode, 0 },
{"ucmple8",     0, {"P", 0}, "d,s,t",     MATCH_UCMPLE8, MASK_UCMPLE8, match_opcode, 0 },
{"ucmple16",    0, {"P", 0}, "d,s,t",     MATCH_UCMPLE16, MASK_UCMPLE16, match_opcode, 0 },
{"ucmplt8",     0, {"P", 0}, "d,s,t",     MATCH_UCMPLT8, MASK_UCMPLT8, match_opcode, 0 },
{"ucmplt16",    0, {"P", 0}, "d,s,t",     MATCH_UCMPLT16, MASK_UCMPLT16, match_opcode, 0 },
{"ukadd8",      0, {"P", 0}, "d,s,t",     MATCH_UKADD8, MASK_UKADD8, match_opcode, 0 },
{"ukadd16",     0, {"P", 0}, "d,s,t",     MATCH_UKADD16, MASK_UKADD16, match_opcode, 0 },
{"ukadd64",     0, {"P", 0}, "nds_rdp,nds_rsp,nds_rtp", MATCH_UKADD64, MASK_UKADD64, match_opcode, 0 },
{"ukaddh",      0, {"P", 0}, "d,s,t",     MATCH_UKADDH, MASK_UKADDH, match_opcode, 0 },
{"ukaddw",      0, {"P", 0}, "d,s,t",     MATCH_UKADDW, MASK_UKADDW, match_opcode, 0 },
{"ukcras16",    0, {"P", 0}, "d,s,t",     MATCH_UKCRAS16, MASK_UKCRAS16, match_opcode, 0 },
{"ukcrsa16",    0, {"P", 0}, "d,s,t",     MATCH_UKCRSA16, MASK_UKCRSA16, match_opcode, 0 },
{"ukmar64",     0, {"P", 0}, "nds_rdp,s,t", MATCH_UKMAR64, MASK_UKMAR64, match_opcode, 0 },
{"ukmsr64",     0, {"P", 0}, "nds_rdp,s,t", MATCH_UKMSR64, MASK_UKMSR64, match_opcode, 0 },
{"ukstas16",    0, {"P", 0}, "d,s,t",     MATCH_UKSTAS16, MASK_UKSTAS16, match_opcode, 0 },
{"ukstsa16",    0, {"P", 0}, "d,s,t",     MATCH_UKSTSA16, MASK_UKSTSA16, match_opcode, 0 },
{"uksub8",      0, {"P", 0}, "d,s,t",     MATCH_UKSUB8, MASK_UKSUB8, match_opcode, 0 },
{"uksub16",     0, {"P", 0}, "d,s,t",     MATCH_UKSUB16, MASK_UKSUB16, match_opcode, 0 },
{"uksub64",     0, {"P", 0}, "nds_rdp,nds_rsp,nds_rtp", MATCH_UKSUB64, MASK_UKSUB64, match_opcode, 0 },
{"uksubh",      0, {"P", 0}, "d,s,t",     MATCH_UKSUBH, MASK_UKSUBH, match_opcode, 0 },
{"uksubw",      0, {"P", 0}, "d,s,t",     MATCH_UKSUBW, MASK_UKSUBW, match_opcode, 0 },
{"umar64",      0, {"P", 0}, "nds_rdp,s,t", MATCH_UMAR64, MASK_UMAR64, match_opcode, 0 },
{"umaqa",       0, {"P", 0}, "d,s,t",     MATCH_UMAQA, MASK_UMAQA, match_opcode, 0 },
{"umax8",       0, {"P", 0}, "d,s,t",     MATCH_UMAX8, MASK_UMAX8, match_opcode, 0 },
{"umax16",      0, {"P", 0}, "d,s,t",     MATCH_UMAX16, MASK_UMAX16, match_opcode, 0 },
{"umin8",       0, {"P", 0}, "d,s,t",     MATCH_UMIN8, MASK_UMIN8, match_opcode, 0 },
{"umin16",      0, {"P", 0}, "d,s,t",     MATCH_UMIN16, MASK_UMIN16, match_opcode, 0 },
{"umsr64",      0, {"P", 0}, "nds_rdp,s,t", MATCH_UMSR64, MASK_UMSR64, match_opcode, 0 },
{"umul8",       0, {"P", 0}, "d,s,t",     MATCH_UMUL8, MASK_UMUL8, match_opcode, 0 },
{"umulx8",      0, {"P", 0}, "d,s,t",     MATCH_UMULX8, MASK_UMULX8, match_opcode, 0 },
{"umul16",      0, {"P", 0}, "nds_rdp,s,t", MATCH_UMUL16, MASK_UMUL16, match_opcode, 0 },
{"umulx16",     0, {"P", 0}, "nds_rdp,s,t", MATCH_UMULX16, MASK_UMULX16, match_opcode, 0 },
{"uradd8",      0, {"P", 0}, "d,s,t",     MATCH_URADD8, MASK_URADD8, match_opcode, 0 },
{"uradd16",     0, {"P", 0}, "d,s,t",     MATCH_URADD16, MASK_URADD16, match_opcode, 0 },
{"uradd64",     0, {"P", 0}, "nds_rdp,nds_rsp,nds_rtp", MATCH_URADD64, MASK_URADD64, match_opcode, 0 },
{"uraddw",      0, {"P", 0}, "d,s,t",     MATCH_URADDW, MASK_URADDW, match_opcode, 0 },
{"urcras16",    0, {"P", 0}, "d,s,t",     MATCH_URCRAS16, MASK_URCRAS16, match_opcode, 0 },
{"urcrsa16",    0, {"P", 0}, "d,s,t",     MATCH_URCRSA16, MASK_URCRSA16, match_opcode, 0 },
{"urstas16",    0, {"P", 0}, "d,s,t",     MATCH_URSTAS16, MASK_URSTAS16, match_opcode, 0 },
{"urstsa16",    0, {"P", 0}, "d,s,t",     MATCH_URSTSA16, MASK_URSTSA16, match_opcode, 0 },
{"ursub8",      0, {"P", 0}, "d,s,t",     MATCH_URSUB8, MASK_URSUB8, match_opcode, 0 },
{"ursub16",     0, {"P", 0}, "d,s,t",     MATCH_URSUB16, MASK_URSUB16, match_opcode, 0 },
{"ursub64",     0, {"P", 0}, "nds_rdp,nds_rsp,nds_rtp", MATCH_URSUB64, MASK_URSUB64, match_opcode, 0 },
{"ursubw",      0, {"P", 0}, "d,s,t",     MATCH_URSUBW, MASK_URSUBW, match_opcode, 0 },
{"wexti",       0, {"P", 0}, "d,nds_rsp,nds_i5u", MATCH_WEXTI, MASK_WEXTI, match_opcode, 0 },
{"wext",        0, {"P", 0}, "d,nds_rsp,t", MATCH_WEXT, MASK_WEXT, match_opcode, 0 },
{"zunpkd810",   0, {"P", 0}, "d,s",       MATCH_ZUNPKD810, MASK_ZUNPKD810, match_opcode, 0 },
{"zunpkd820",   0, {"P", 0}, "d,s",       MATCH_ZUNPKD820, MASK_ZUNPKD820, match_opcode, 0 },
{"zunpkd830",   0, {"P", 0}, "d,s",       MATCH_ZUNPKD830, MASK_ZUNPKD830, match_opcode, 0 },
{"zunpkd831",   0, {"P", 0}, "d,s",       MATCH_ZUNPKD831, MASK_ZUNPKD831, match_opcode, 0 },
{"zunpkd832",   0, {"P", 0}, "d,s",       MATCH_ZUNPKD832, MASK_ZUNPKD832, match_opcode, 0 },
{"rdov",        0, {"P", 0}, "d",         MATCH_RDOV, MASK_RDOV, match_opcode, INSN_ALIAS },
{"clrov",       0, {"P", 0}, "",          MATCH_CLROV, MASK_CLROV, match_opcode, INSN_ALIAS },

/* NDS V5 DSP Extension (RV64 only).  */
{"add32",      64, {"P", 0}, "d,s,t",     MATCH_ADD32, MASK_ADD32, match_opcode, 0 },
{"cras32",     64, {"P", 0}, "d,s,t",     MATCH_CRAS32, MASK_CRAS32, match_opcode, 0 },
{"crsa32",     64, {"P", 0}, "d,s,t",     MATCH_CRSA32, MASK_CRSA32, match_opcode, 0 },
{"kabs32",     64, {"P", 0}, "d,s",       MATCH_KABS32, MASK_KABS32, match_opcode, 0 },
{"kadd32",     64, {"P", 0}, "d,s,t",     MATCH_KADD32, MASK_KADD32, match_opcode, 0 },
{"kcras32",    64, {"P", 0}, "d,s,t",     MATCH_KCRAS32, MASK_KCRAS32, match_opcode, 0 },
{"kcrsa32",    64, {"P", 0}, "d,s,t",     MATCH_KCRSA32, MASK_KCRSA32, match_opcode, 0 },
{"kdmbb16",    64, {"P", 0}, "d,s,t",     MATCH_KDMBB16, MASK_KDMBB16, match_opcode, 0 },
{"kdmbt16",    64, {"P", 0}, "d,s,t",     MATCH_KDMBT16, MASK_KDMBT16, match_opcode, 0 },
{"kdmtt16",    64, {"P", 0}, "d,s,t",     MATCH_KDMTT16, MASK_KDMTT16, match_opcode, 0 },
{"kdmabb16",   64, {"P", 0}, "d,s,t",     MATCH_KDMABB16, MASK_KDMABB16, match_opcode, 0 },
{"kdmabt16",   64, {"P", 0}, "d,s,t",     MATCH_KDMABT16, MASK_KDMABT16, match_opcode, 0 },
{"kdmatt16",   64, {"P", 0}, "d,s,t",     MATCH_KDMATT16, MASK_KDMATT16, match_opcode, 0 },
{"khmbb16",    64, {"P", 0}, "d,s,t",     MATCH_KHMBB16, MASK_KHMBB16, match_opcode, 0 },
{"khmbt16",    64, {"P", 0}, "d,s,t",     MATCH_KHMBT16, MASK_KHMBT16, match_opcode, 0 },
{"khmtt16",    64, {"P", 0}, "d,s,t",     MATCH_KHMTT16, MASK_KHMTT16, match_opcode, 0 },
{"kmabb32",    64, {"P", 0}, "d,s,t",     MATCH_KMABB32, MASK_KMABB32, match_opcode, 0 },
{"kmabt32",    64, {"P", 0}, "d,s,t",     MATCH_KMABT32, MASK_KMABT32, match_opcode, 0 },
{"kmatt32",    64, {"P", 0}, "d,s,t",     MATCH_KMATT32, MASK_KMATT32, match_opcode, 0 },
{"kmada32",    64, {"P", 0}, "d,s,t",     MATCH_KMADA32, MASK_KMADA32, match_opcode, INSN_ALIAS },
{"kmaxda32",   64, {"P", 0}, "d,s,t",     MATCH_KMAXDA32, MASK_KMAXDA32, match_opcode, 0 },
{"kmda32",     64, {"P", 0}, "d,s,t",     MATCH_KMDA32, MASK_KMDA32, match_opcode, 0 },
{"kmxda32",    64, {"P", 0}, "d,s,t",     MATCH_KMXDA32, MASK_KMXDA32, match_opcode, 0 },
{"kmads32",    64, {"P", 0}, "d,s,t",     MATCH_KMADS32, MASK_KMADS32, match_opcode, 0 },
{"kmadrs32",   64, {"P", 0}, "d,s,t",     MATCH_KMADRS32, MASK_KMADRS32, match_opcode, 0 },
{"kmaxds32",   64, {"P", 0}, "d,s,t",     MATCH_KMAXDS32, MASK_KMAXDS32, match_opcode, 0 },
{"kmsda32",    64, {"P", 0}, "d,s,t",     MATCH_KMSDA32, MASK_KMSDA32, match_opcode, 0 },
{"kmsxda32",   64, {"P", 0}, "d,s,t",     MATCH_KMSXDA32, MASK_KMSXDA32, match_opcode, 0 },
{"ksll32",     64, {"P", 0}, "d,s,t",     MATCH_KSLL32, MASK_KSLL32, match_opcode, 0 },
{"kslli32",    64, {"P", 0}, "d,s,nds_i5u", MATCH_KSLLI32, MASK_KSLLI32, match_opcode, 0 },
{"kslra32",    64, {"P", 0}, "d,s,t",     MATCH_KSLRA32, MASK_KSLRA32, match_opcode, 0 },
{"kslra32.u",  64, {"P", 0}, "d,s,t",     MATCH_KSLRA32_U, MASK_KSLRA32_U, match_opcode, 0 },
{"kstas32",    64, {"P", 0}, "d,s,t",     MATCH_KSTAS32, MASK_KSTAS32, match_opcode, 0 },
{"kstsa32",    64, {"P", 0}, "d,s,t",     MATCH_KSTSA32, MASK_KSTSA32, match_opcode, 0 },
{"ksub32",     64, {"P", 0}, "d,s,t",     MATCH_KSUB32, MASK_KSUB32, match_opcode, 0 },
{"pkbb32",     64, {"P", 0}, "d,s,t",     MATCH_PKBB32, MASK_PKBB32, match_opcode, 0 },
{"pkbt32",     64, {"P", 0}, "d,s,t",     MATCH_PKBT32, MASK_PKBT32, match_opcode, 0 },
{"pktt32",     64, {"P", 0}, "d,s,t",     MATCH_PKTT32, MASK_PKTT32, match_opcode, 0 },
{"pktb32",     64, {"P", 0}, "d,s,t",     MATCH_PKTB32, MASK_PKTB32, match_opcode, 0 },
{"radd32",     64, {"P", 0}, "d,s,t",     MATCH_RADD32, MASK_RADD32, match_opcode, 0 },
{"rcras32",    64, {"P", 0}, "d,s,t",     MATCH_RCRAS32, MASK_RCRAS32, match_opcode, 0 },
{"rcrsa32",    64, {"P", 0}, "d,s,t",     MATCH_RCRSA32, MASK_RCRSA32, match_opcode, 0 },
{"rstas32",    64, {"P", 0}, "d,s,t",     MATCH_RSTAS32, MASK_RSTAS32, match_opcode, 0 },
{"rstsa32",    64, {"P", 0}, "d,s,t",     MATCH_RSTSA32, MASK_RSTSA32, match_opcode, 0 },
{"rsub32",     64, {"P", 0}, "d,s,t",     MATCH_RSUB32, MASK_RSUB32, match_opcode, 0 },
{"sll32",      64, {"P", 0}, "d,s,t",     MATCH_SLL32, MASK_SLL32, match_opcode, 0 },
{"slli32",     64, {"P", 0}, "d,s,nds_i5u", MATCH_SLLI32, MASK_SLLI32, match_opcode, 0 },
{"smax32",     64, {"P", 0}, "d,s,t",     MATCH_SMAX32, MASK_SMAX32, match_opcode, 0 },
{"smbb32",     64, {"P", 0}, "d,s,t",     MATCH_SMBB32, MASK_SMBB32, match_opcode, 0 },
{"smbt32",     64, {"P", 0}, "d,s,t",     MATCH_SMBT32, MASK_SMBT32, match_opcode, 0 },
{"smtt32",     64, {"P", 0}, "d,s,t",     MATCH_SMTT32, MASK_SMTT32, match_opcode, 0 },
{"smds32",     64, {"P", 0}, "d,s,t",     MATCH_SMDS32, MASK_SMDS32, match_opcode, 0 },
{"smdrs32",    64, {"P", 0}, "d,s,t",     MATCH_SMDRS32, MASK_SMDRS32, match_opcode, 0 },
{"smxds32",    64, {"P", 0}, "d,s,t",     MATCH_SMXDS32, MASK_SMXDS32, match_opcode, 0 },
{"smin32",     64, {"P", 0}, "d,s,t",     MATCH_SMIN32, MASK_SMIN32, match_opcode, 0 },
{"sra32",      64, {"P", 0}, "d,s,t",     MATCH_SRA32, MASK_SRA32, match_opcode, 0 },
{"sra32.u",    64, {"P", 0}, "d,s,t",     MATCH_SRA32_U, MASK_SRA32_U, match_opcode, 0 },
{"srai32",     64, {"P", 0}, "d,s,nds_i5u", MATCH_SRAI32, MASK_SRAI32, match_opcode, 0 },
{"srai32.u",   64, {"P", 0}, "d,s,nds_i5u", MATCH_SRAI32_U, MASK_SRAI32_U, match_opcode, 0 },
{"sraiw.u",    64, {"P", 0}, "d,s,nds_i5u", MATCH_SRAIW_U, MASK_SRAIW_U, match_opcode, 0 },
{"srl32",      64, {"P", 0}, "d,s,t",     MATCH_SRL32, MASK_SRL32, match_opcode, 0 },
{"srl32.u",    64, {"P", 0}, "d,s,t",     MATCH_SRL32_U, MASK_SRL32_U, match_opcode, 0 },
{"srli32",     64, {"P", 0}, "d,s,nds_i5u", MATCH_SRLI32, MASK_SRLI32, match_opcode, 0 },
{"srli32.u",   64, {"P", 0}, "d,s,nds_i5u", MATCH_SRLI32_U, MASK_SRLI32_U, match_opcode, 0 },
{"stas32",     64, {"P", 0}, "d,s,t",     MATCH_STAS32, MASK_STAS32, match_opcode, 0 },
{"stsa32",     64, {"P", 0}, "d,s,t",     MATCH_STSA32, MASK_STSA32, match_opcode, 0 },
{"sub32",      64, {"P", 0}, "d,s,t",     MATCH_SUB32, MASK_SUB32, match_opcode, 0 },
{"ukadd32",    64, {"P", 0}, "d,s,t",     MATCH_UKADD32, MASK_UKADD32, match_opcode, 0 },
{"ukcras32",   64, {"P", 0}, "d,s,t",     MATCH_UKCRAS32, MASK_UKCRAS32, match_opcode, 0 },
{"ukcrsa32",   64, {"P", 0}, "d,s,t",     MATCH_UKCRSA32, MASK_UKCRSA32, match_opcode, 0 },
{"ukstas32",   64, {"P", 0}, "d,s,t",     MATCH_UKSTAS32, MASK_UKSTAS32, match_opcode, 0 },
{"ukstsa32",   64, {"P", 0}, "d,s,t",     MATCH_UKSTSA32, MASK_UKSTSA32, match_opcode, 0 },
{"uksub32",    64, {"P", 0}, "d,s,t",     MATCH_UKSUB32, MASK_UKSUB32, match_opcode, 0 },
{"umax32",     64, {"P", 0}, "d,s,t",     MATCH_UMAX32, MASK_UMAX32, match_opcode, 0 },
{"umin32",     64, {"P", 0}, "d,s,t",     MATCH_UMIN32, MASK_UMIN32, match_opcode, 0 },
{"uradd32",    64, {"P", 0}, "d,s,t",     MATCH_URADD32, MASK_URADD32, match_opcode, 0 },
{"urcras32",   64, {"P", 0}, "d,s,t",     MATCH_URCRAS32, MASK_URCRAS32, match_opcode, 0 },
{"urcrsa32",   64, {"P", 0}, "d,s,t",     MATCH_URCRSA32, MASK_URCRSA32, match_opcode, 0 },
{"urstas32",   64, {"P", 0}, "d,s,t",     MATCH_URSTAS32, MASK_URSTAS32, match_opcode, 0 },
{"urstsa32",   64, {"P", 0}, "d,s,t",     MATCH_URSTSA32, MASK_URSTSA32, match_opcode, 0 },
{"ursub32",    64, {"P", 0}, "d,s,t",     MATCH_URSUB32, MASK_URSUB32, match_opcode, 0 },

/* Andes X_efhw extension */
{"flhw",        0, {"XEFHW", 0}, "D,o(s)", MATCH_FLHW, MASK_FLHW, match_opcode, 0 },
{"fshw",        0, {"XEFHW", 0}, "T,q(s)", MATCH_FSHW, MASK_FSHW, match_opcode, 0 },

/* Atomic memory operation instruction subset */
{"lr.w",         0, {"A", 0},   "d,0(s)",    MATCH_LR_W, MASK_LR_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"sc.w",         0, {"A", 0},   "d,t,0(s)",  MATCH_SC_W, MASK_SC_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amoadd.w",     0, {"A", 0},   "d,t,0(s)",  MATCH_AMOADD_W, MASK_AMOADD_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amoswap.w",    0, {"A", 0},   "d,t,0(s)",  MATCH_AMOSWAP_W, MASK_AMOSWAP_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amoand.w",     0, {"A", 0},   "d,t,0(s)",  MATCH_AMOAND_W, MASK_AMOAND_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amoor.w",      0, {"A", 0},   "d,t,0(s)",  MATCH_AMOOR_W, MASK_AMOOR_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amoxor.w",     0, {"A", 0},   "d,t,0(s)",  MATCH_AMOXOR_W, MASK_AMOXOR_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amomax.w",     0, {"A", 0},   "d,t,0(s)",  MATCH_AMOMAX_W, MASK_AMOMAX_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amomaxu.w",    0, {"A", 0},   "d,t,0(s)",  MATCH_AMOMAXU_W, MASK_AMOMAXU_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amomin.w",     0, {"A", 0},   "d,t,0(s)",  MATCH_AMOMIN_W, MASK_AMOMIN_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amominu.w",    0, {"A", 0},   "d,t,0(s)",  MATCH_AMOMINU_W, MASK_AMOMINU_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"lr.w.aq",      0, {"A", 0},   "d,0(s)",    MATCH_LR_W | MASK_AQ, MASK_LR_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"sc.w.aq",      0, {"A", 0},   "d,t,0(s)",  MATCH_SC_W | MASK_AQ, MASK_SC_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amoadd.w.aq",  0, {"A", 0},   "d,t,0(s)",  MATCH_AMOADD_W | MASK_AQ, MASK_AMOADD_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amoswap.w.aq", 0, {"A", 0},   "d,t,0(s)",  MATCH_AMOSWAP_W | MASK_AQ, MASK_AMOSWAP_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amoand.w.aq",  0, {"A", 0},   "d,t,0(s)",  MATCH_AMOAND_W | MASK_AQ, MASK_AMOAND_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amoor.w.aq",   0, {"A", 0},   "d,t,0(s)",  MATCH_AMOOR_W | MASK_AQ, MASK_AMOOR_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amoxor.w.aq",  0, {"A", 0},   "d,t,0(s)",  MATCH_AMOXOR_W | MASK_AQ, MASK_AMOXOR_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amomax.w.aq",  0, {"A", 0},   "d,t,0(s)",  MATCH_AMOMAX_W | MASK_AQ, MASK_AMOMAX_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amomaxu.w.aq", 0, {"A", 0},   "d,t,0(s)",  MATCH_AMOMAXU_W | MASK_AQ, MASK_AMOMAXU_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amomin.w.aq",  0, {"A", 0},   "d,t,0(s)",  MATCH_AMOMIN_W | MASK_AQ, MASK_AMOMIN_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amominu.w.aq", 0, {"A", 0},   "d,t,0(s)",  MATCH_AMOMINU_W | MASK_AQ, MASK_AMOMINU_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"lr.w.rl",      0, {"A", 0},   "d,0(s)",    MATCH_LR_W | MASK_RL, MASK_LR_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"sc.w.rl",      0, {"A", 0},   "d,t,0(s)",  MATCH_SC_W | MASK_RL, MASK_SC_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amoadd.w.rl",  0, {"A", 0},   "d,t,0(s)",  MATCH_AMOADD_W | MASK_RL, MASK_AMOADD_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amoswap.w.rl", 0, {"A", 0},   "d,t,0(s)",  MATCH_AMOSWAP_W | MASK_RL, MASK_AMOSWAP_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amoand.w.rl",  0, {"A", 0},   "d,t,0(s)",  MATCH_AMOAND_W | MASK_RL, MASK_AMOAND_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amoor.w.rl",   0, {"A", 0},   "d,t,0(s)",  MATCH_AMOOR_W | MASK_RL, MASK_AMOOR_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amoxor.w.rl",  0, {"A", 0},   "d,t,0(s)",  MATCH_AMOXOR_W | MASK_RL, MASK_AMOXOR_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amomax.w.rl",  0, {"A", 0},   "d,t,0(s)",  MATCH_AMOMAX_W | MASK_RL, MASK_AMOMAX_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amomaxu.w.rl", 0, {"A", 0},   "d,t,0(s)",  MATCH_AMOMAXU_W | MASK_RL, MASK_AMOMAXU_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amomin.w.rl",  0, {"A", 0},   "d,t,0(s)",  MATCH_AMOMIN_W | MASK_RL, MASK_AMOMIN_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amominu.w.rl", 0, {"A", 0},   "d,t,0(s)",  MATCH_AMOMINU_W | MASK_RL, MASK_AMOMINU_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"lr.w.aqrl",    0, {"A", 0},   "d,0(s)",    MATCH_LR_W | MASK_AQRL, MASK_LR_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"sc.w.aqrl",    0, {"A", 0},   "d,t,0(s)",  MATCH_SC_W | MASK_AQRL, MASK_SC_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amoadd.w.aqrl",  0, {"A", 0},   "d,t,0(s)",  MATCH_AMOADD_W | MASK_AQRL, MASK_AMOADD_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amoswap.w.aqrl", 0, {"A", 0},   "d,t,0(s)",  MATCH_AMOSWAP_W | MASK_AQRL, MASK_AMOSWAP_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amoand.w.aqrl",  0, {"A", 0},   "d,t,0(s)",  MATCH_AMOAND_W | MASK_AQRL, MASK_AMOAND_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amoor.w.aqrl",   0, {"A", 0},   "d,t,0(s)",  MATCH_AMOOR_W | MASK_AQRL, MASK_AMOOR_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amoxor.w.aqrl",  0, {"A", 0},   "d,t,0(s)",  MATCH_AMOXOR_W | MASK_AQRL, MASK_AMOXOR_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amomax.w.aqrl",  0, {"A", 0},   "d,t,0(s)",  MATCH_AMOMAX_W | MASK_AQRL, MASK_AMOMAX_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amomaxu.w.aqrl", 0, {"A", 0},   "d,t,0(s)",  MATCH_AMOMAXU_W | MASK_AQRL, MASK_AMOMAXU_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amomin.w.aqrl",  0, {"A", 0},   "d,t,0(s)",  MATCH_AMOMIN_W | MASK_AQRL, MASK_AMOMIN_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"amominu.w.aqrl", 0, {"A", 0},   "d,t,0(s)",  MATCH_AMOMINU_W | MASK_AQRL, MASK_AMOMINU_W | MASK_AQRL, match_opcode, INSN_DREF|INSN_4_BYTE },
{"lr.d",         64, {"A", 0} , "d,0(s)",    MATCH_LR_D, MASK_LR_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"sc.d",         64, {"A", 0} , "d,t,0(s)",  MATCH_SC_D, MASK_SC_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amoadd.d",     64, {"A", 0} , "d,t,0(s)",  MATCH_AMOADD_D, MASK_AMOADD_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amoswap.d",    64, {"A", 0} , "d,t,0(s)",  MATCH_AMOSWAP_D, MASK_AMOSWAP_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amoand.d",     64, {"A", 0} , "d,t,0(s)",  MATCH_AMOAND_D, MASK_AMOAND_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amoor.d",      64, {"A", 0} , "d,t,0(s)",  MATCH_AMOOR_D, MASK_AMOOR_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amoxor.d",     64, {"A", 0} , "d,t,0(s)",  MATCH_AMOXOR_D, MASK_AMOXOR_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amomax.d",     64, {"A", 0} , "d,t,0(s)",  MATCH_AMOMAX_D, MASK_AMOMAX_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amomaxu.d",    64, {"A", 0} , "d,t,0(s)",  MATCH_AMOMAXU_D, MASK_AMOMAXU_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amomin.d",     64, {"A", 0} , "d,t,0(s)",  MATCH_AMOMIN_D, MASK_AMOMIN_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amominu.d",    64, {"A", 0} , "d,t,0(s)",  MATCH_AMOMINU_D, MASK_AMOMINU_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"lr.d.aq",      64, {"A", 0} , "d,0(s)",    MATCH_LR_D | MASK_AQ, MASK_LR_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"sc.d.aq",      64, {"A", 0} , "d,t,0(s)",  MATCH_SC_D | MASK_AQ, MASK_SC_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amoadd.d.aq",  64, {"A", 0} , "d,t,0(s)",  MATCH_AMOADD_D | MASK_AQ, MASK_AMOADD_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amoswap.d.aq", 64, {"A", 0} , "d,t,0(s)",  MATCH_AMOSWAP_D | MASK_AQ, MASK_AMOSWAP_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amoand.d.aq",  64, {"A", 0} , "d,t,0(s)",  MATCH_AMOAND_D | MASK_AQ, MASK_AMOAND_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amoor.d.aq",   64, {"A", 0} , "d,t,0(s)",  MATCH_AMOOR_D | MASK_AQ, MASK_AMOOR_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amoxor.d.aq",  64, {"A", 0} , "d,t,0(s)",  MATCH_AMOXOR_D | MASK_AQ, MASK_AMOXOR_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amomax.d.aq",  64, {"A", 0} , "d,t,0(s)",  MATCH_AMOMAX_D | MASK_AQ, MASK_AMOMAX_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amomaxu.d.aq", 64, {"A", 0} , "d,t,0(s)",  MATCH_AMOMAXU_D | MASK_AQ, MASK_AMOMAXU_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amomin.d.aq",  64, {"A", 0} , "d,t,0(s)",  MATCH_AMOMIN_D | MASK_AQ, MASK_AMOMIN_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amominu.d.aq", 64, {"A", 0} , "d,t,0(s)",  MATCH_AMOMINU_D | MASK_AQ, MASK_AMOMINU_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"lr.d.rl",      64, {"A", 0} , "d,0(s)",    MATCH_LR_D | MASK_RL, MASK_LR_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"sc.d.rl",      64, {"A", 0} , "d,t,0(s)",  MATCH_SC_D | MASK_RL, MASK_SC_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amoadd.d.rl",  64, {"A", 0} , "d,t,0(s)",  MATCH_AMOADD_D | MASK_RL, MASK_AMOADD_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amoswap.d.rl", 64, {"A", 0} , "d,t,0(s)",  MATCH_AMOSWAP_D | MASK_RL, MASK_AMOSWAP_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amoand.d.rl",  64, {"A", 0} , "d,t,0(s)",  MATCH_AMOAND_D | MASK_RL, MASK_AMOAND_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amoor.d.rl",   64, {"A", 0} , "d,t,0(s)",  MATCH_AMOOR_D | MASK_RL, MASK_AMOOR_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amoxor.d.rl",  64, {"A", 0} , "d,t,0(s)",  MATCH_AMOXOR_D | MASK_RL, MASK_AMOXOR_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amomax.d.rl",  64, {"A", 0} , "d,t,0(s)",  MATCH_AMOMAX_D | MASK_RL, MASK_AMOMAX_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amomaxu.d.rl", 64, {"A", 0} , "d,t,0(s)",  MATCH_AMOMAXU_D | MASK_RL, MASK_AMOMAXU_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amomin.d.rl",  64, {"A", 0} , "d,t,0(s)",  MATCH_AMOMIN_D | MASK_RL, MASK_AMOMIN_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amominu.d.rl", 64, {"A", 0} , "d,t,0(s)",  MATCH_AMOMINU_D | MASK_RL, MASK_AMOMINU_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"lr.d.aqrl",    64, {"A", 0} , "d,0(s)",    MATCH_LR_D | MASK_AQRL, MASK_LR_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"sc.d.aqrl",    64, {"A", 0} , "d,t,0(s)",  MATCH_SC_D | MASK_AQRL, MASK_SC_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amoadd.d.aqrl",  64, {"A", 0} , "d,t,0(s)",  MATCH_AMOADD_D | MASK_AQRL, MASK_AMOADD_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amoswap.d.aqrl", 64, {"A", 0} , "d,t,0(s)",  MATCH_AMOSWAP_D | MASK_AQRL, MASK_AMOSWAP_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amoand.d.aqrl",  64, {"A", 0} , "d,t,0(s)",  MATCH_AMOAND_D | MASK_AQRL, MASK_AMOAND_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amoor.d.aqrl",   64, {"A", 0} , "d,t,0(s)",  MATCH_AMOOR_D | MASK_AQRL, MASK_AMOOR_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amoxor.d.aqrl",  64, {"A", 0} , "d,t,0(s)",  MATCH_AMOXOR_D | MASK_AQRL, MASK_AMOXOR_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amomax.d.aqrl",  64, {"A", 0} , "d,t,0(s)",  MATCH_AMOMAX_D | MASK_AQRL, MASK_AMOMAX_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amomaxu.d.aqrl", 64, {"A", 0} , "d,t,0(s)",  MATCH_AMOMAXU_D | MASK_AQRL, MASK_AMOMAXU_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amomin.d.aqrl",  64, {"A", 0} , "d,t,0(s)",  MATCH_AMOMIN_D | MASK_AQRL, MASK_AMOMIN_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },
{"amominu.d.aqrl", 64, {"A", 0} , "d,t,0(s)",  MATCH_AMOMINU_D | MASK_AQRL, MASK_AMOMINU_D | MASK_AQRL, match_opcode, INSN_DREF|INSN_8_BYTE },

/* Multiply/Divide instruction subset */
{"mul",       0, {"M", 0},   "d,s,t",  MATCH_MUL, MASK_MUL, match_opcode, 0 },
{"mulh",      0, {"M", 0},   "d,s,t",  MATCH_MULH, MASK_MULH, match_opcode, 0 },
{"mulhu",     0, {"M", 0},   "d,s,t",  MATCH_MULHU, MASK_MULHU, match_opcode, 0 },
{"mulhsu",    0, {"M", 0},   "d,s,t",  MATCH_MULHSU, MASK_MULHSU, match_opcode, 0 },
{"div",       0, {"M", 0},   "d,s,t",  MATCH_DIV, MASK_DIV, match_opcode, 0 },
{"divu",      0, {"M", 0},   "d,s,t",  MATCH_DIVU, MASK_DIVU, match_opcode, 0 },
{"rem",       0, {"M", 0},   "d,s,t",  MATCH_REM, MASK_REM, match_opcode, 0 },
{"remu",      0, {"M", 0},   "d,s,t",  MATCH_REMU, MASK_REMU, match_opcode, 0 },
{"mulw",     64, {"M", 0}, "d,s,t",  MATCH_MULW, MASK_MULW, match_opcode, 0 },
{"divw",     64, {"M", 0}, "d,s,t",  MATCH_DIVW, MASK_DIVW, match_opcode, 0 },
{"divuw",    64, {"M", 0}, "d,s,t",  MATCH_DIVUW, MASK_DIVUW, match_opcode, 0 },
{"remw",     64, {"M", 0}, "d,s,t",  MATCH_REMW, MASK_REMW, match_opcode, 0 },
{"remuw",    64, {"M", 0}, "d,s,t",  MATCH_REMUW, MASK_REMUW, match_opcode, 0 },

/* Half-precision floating-point instruction subset */
// {"flh",         32, {"ZFH", "C", 0},  "D,Cm(Cc)", MATCH_C_FLHSP, MASK_C_FLHSP, match_opcode, INSN_ALIAS|INSN_DREF|INSN_2_BYTE },
// {"flh",         32, {"ZFH", "C", 0},  "CD,Ck(Cs)",MATCH_C_FLH, MASK_C_FLH, match_opcode, INSN_ALIAS|INSN_DREF|INSN_2_BYTE },
{"flh",          0, {"ZFH", 0},       "D,o(s)",   MATCH_FLH, MASK_FLH, match_opcode, INSN_DREF|INSN_2_BYTE },
{"flh",          0, {"ZFH", 0},       "D,A,s",    0,  (int) M_FLH, match_never, INSN_MACRO },
// {"fsh",         32, {"ZFH", "C", 0},  "CT,CM(Cc)",MATCH_C_FSHSP, MASK_C_FSHSP, match_opcode, INSN_ALIAS|INSN_DREF|INSN_2_BYTE },
// {"fsh",         32, {"ZFH", "C", 0},  "CD,Ck(Cs)",MATCH_C_FSH, MASK_C_FSH, match_opcode, INSN_ALIAS|INSN_DREF|INSN_2_BYTE },
{"fsh",          0, {"ZFH", 0},       "T,q(s)",   MATCH_FSH, MASK_FSH, match_opcode, INSN_DREF|INSN_2_BYTE },
{"fsh",          0, {"ZFH", 0},       "T,A,s",    0, (int) M_FSH, match_never, INSN_MACRO },

{"fmv.x.h",      0, {"ZFH", 0},       "d,S",      MATCH_FMV_X_H, MASK_FMV_X_H, match_opcode, 0 },
{"fmv.h.x",      0, {"ZFH", 0},       "D,s",      MATCH_FMV_H_X, MASK_FMV_H_X, match_opcode, 0 },

{"fmv.h",        0, {"ZFH", 0},       "D,U",      MATCH_FSGNJ_H, MASK_FSGNJ_H, match_rs1_eq_rs2, INSN_ALIAS },
{"fneg.h",       0, {"ZFH", 0},       "D,U",      MATCH_FSGNJN_H, MASK_FSGNJN_H, match_rs1_eq_rs2, INSN_ALIAS },
{"fabs.h",       0, {"ZFH", 0},       "D,U",      MATCH_FSGNJX_H, MASK_FSGNJX_H, match_rs1_eq_rs2, INSN_ALIAS },
{"fsgnj.h",      0, {"ZFH", 0},       "D,S,T",    MATCH_FSGNJ_H, MASK_FSGNJ_H, match_opcode, 0 },
{"fsgnjn.h",     0, {"ZFH", 0},       "D,S,T",    MATCH_FSGNJN_H, MASK_FSGNJN_H, match_opcode, 0 },
{"fsgnjx.h",     0, {"ZFH", 0},       "D,S,T",    MATCH_FSGNJX_H, MASK_FSGNJX_H, match_opcode, 0 },
{"fadd.h",       0, {"ZFH", 0},       "D,S,T",    MATCH_FADD_H | MASK_RM, MASK_FADD_H | MASK_RM, match_opcode, 0 },
{"fadd.h",       0, {"ZFH", 0},       "D,S,T,m",  MATCH_FADD_H, MASK_FADD_H, match_opcode, 0 },
{"fsub.h",       0, {"ZFH", 0},       "D,S,T",    MATCH_FSUB_H | MASK_RM, MASK_FSUB_H | MASK_RM, match_opcode, 0 },
{"fsub.h",       0, {"ZFH", 0},       "D,S,T,m",  MATCH_FSUB_H, MASK_FSUB_H, match_opcode, 0 },
{"fmul.h",       0, {"ZFH", 0},       "D,S,T",    MATCH_FMUL_H | MASK_RM, MASK_FMUL_H | MASK_RM, match_opcode, 0 },
{"fmul.h",       0, {"ZFH", 0},       "D,S,T,m",  MATCH_FMUL_H, MASK_FMUL_H, match_opcode, 0 },
{"fdiv.h",       0, {"ZFH", 0},       "D,S,T",    MATCH_FDIV_H | MASK_RM, MASK_FDIV_H | MASK_RM, match_opcode, 0 },
{"fdiv.h",       0, {"ZFH", 0},       "D,S,T,m",  MATCH_FDIV_H, MASK_FDIV_H, match_opcode, 0 },
{"fsqrt.h",      0, {"ZFH", 0},       "D,S",      MATCH_FSQRT_H | MASK_RM, MASK_FSQRT_H | MASK_RM, match_opcode, 0 },
{"fsqrt.h",      0, {"ZFH", 0},       "D,S,m",    MATCH_FSQRT_H, MASK_FSQRT_H, match_opcode, 0 },
{"fmin.h",       0, {"ZFH", 0},       "D,S,T",    MATCH_FMIN_H, MASK_FMIN_H, match_opcode, 0 },
{"fmax.h",       0, {"ZFH", 0},       "D,S,T",    MATCH_FMAX_H, MASK_FMAX_H, match_opcode, 0 },
{"fmadd.h",      0, {"ZFH", 0},       "D,S,T,R",  MATCH_FMADD_H | MASK_RM, MASK_FMADD_H | MASK_RM, match_opcode, 0 },
{"fmadd.h",      0, {"ZFH", 0},       "D,S,T,R,m",MATCH_FMADD_H, MASK_FMADD_H, match_opcode, 0 },
{"fnmadd.h",     0, {"ZFH", 0},       "D,S,T,R",  MATCH_FNMADD_H | MASK_RM, MASK_FNMADD_H | MASK_RM, match_opcode, 0 },
{"fnmadd.h",     0, {"ZFH", 0},       "D,S,T,R,m",MATCH_FNMADD_H, MASK_FNMADD_H, match_opcode, 0 },
{"fmsub.h",      0, {"ZFH", 0},       "D,S,T,R",  MATCH_FMSUB_H | MASK_RM, MASK_FMSUB_H | MASK_RM, match_opcode, 0 },
{"fmsub.h",      0, {"ZFH", 0},       "D,S,T,R,m",MATCH_FMSUB_H, MASK_FMSUB_H, match_opcode, 0 },
{"fnmsub.h",     0, {"ZFH", 0},       "D,S,T,R",  MATCH_FNMSUB_H | MASK_RM, MASK_FNMSUB_H | MASK_RM, match_opcode, 0 },
{"fnmsub.h",     0, {"ZFH", 0},       "D,S,T,R,m",MATCH_FNMSUB_H, MASK_FNMSUB_H, match_opcode, 0 },
{"fcvt.w.h",     0, {"ZFH", 0},       "d,S",      MATCH_FCVT_W_H | MASK_RM, MASK_FCVT_W_H | MASK_RM, match_opcode, 0 },
{"fcvt.w.h",     0, {"ZFH", 0},       "d,S,m",    MATCH_FCVT_W_H, MASK_FCVT_W_H, match_opcode, 0 },
{"fcvt.wu.h",    0, {"ZFH", 0},       "d,S",      MATCH_FCVT_WU_H | MASK_RM, MASK_FCVT_WU_H | MASK_RM, match_opcode, 0 },
{"fcvt.wu.h",    0, {"ZFH", 0},       "d,S,m",    MATCH_FCVT_WU_H, MASK_FCVT_WU_H, match_opcode, 0 },
{"fcvt.h.w",     0, {"ZFH", 0},       "D,s",      MATCH_FCVT_H_W | MASK_RM, MASK_FCVT_H_W | MASK_RM, match_opcode, 0 },
{"fcvt.h.w",     0, {"ZFH", 0},       "D,s,m",    MATCH_FCVT_H_W, MASK_FCVT_H_W, match_opcode, 0 },
{"fcvt.h.wu",    0, {"ZFH", 0},       "D,s",      MATCH_FCVT_H_WU | MASK_RM, MASK_FCVT_H_W | MASK_RM, match_opcode, 0 },
{"fcvt.h.wu",    0, {"ZFH", 0},       "D,s,m",    MATCH_FCVT_H_WU, MASK_FCVT_H_WU, match_opcode, 0 },
{"fclass.h",     0, {"ZFH", 0},       "d,S",      MATCH_FCLASS_H, MASK_FCLASS_H, match_opcode, 0 },
{"feq.h",        0, {"ZFH", 0},       "d,S,T",    MATCH_FEQ_H, MASK_FEQ_H, match_opcode, 0 },
{"flt.h",        0, {"ZFH", 0},       "d,S,T",    MATCH_FLT_H, MASK_FLT_H, match_opcode, 0 },
{"fle.h",        0, {"ZFH", 0},       "d,S,T",    MATCH_FLE_H, MASK_FLE_H, match_opcode, 0 },
{"fgt.h",        0, {"ZFH", 0},       "d,T,S",    MATCH_FLT_H, MASK_FLT_H, match_opcode, 0 },
{"fge.h",        0, {"ZFH", 0},       "d,T,S",    MATCH_FLE_H, MASK_FLE_H, match_opcode, 0 },
{"fcvt.l.h",    64, {"ZFH", 0},       "d,S",      MATCH_FCVT_L_H | MASK_RM, MASK_FCVT_L_H | MASK_RM, match_opcode, 0 },
{"fcvt.l.h",    64, {"ZFH", 0},       "d,S,m",    MATCH_FCVT_L_H, MASK_FCVT_L_H, match_opcode, 0 },
{"fcvt.lu.h",   64, {"ZFH", 0},       "d,S",      MATCH_FCVT_LU_H | MASK_RM, MASK_FCVT_LU_H | MASK_RM, match_opcode, 0 },
{"fcvt.lu.h",   64, {"ZFH", 0},       "d,S,m",    MATCH_FCVT_LU_H, MASK_FCVT_LU_H, match_opcode, 0 },
{"fcvt.h.l",    64, {"ZFH", 0},       "D,s",      MATCH_FCVT_H_L | MASK_RM, MASK_FCVT_H_L | MASK_RM, match_opcode, 0 },
{"fcvt.h.l",    64, {"ZFH", 0},       "D,s,m",    MATCH_FCVT_H_L, MASK_FCVT_H_L, match_opcode, 0 },
{"fcvt.h.lu",   64, {"ZFH", 0},       "D,s",      MATCH_FCVT_H_LU | MASK_RM, MASK_FCVT_H_L | MASK_RM, match_opcode, 0 },
{"fcvt.h.lu",   64, {"ZFH", 0},       "D,s,m",    MATCH_FCVT_H_LU, MASK_FCVT_H_LU, match_opcode, 0 },

{"fcvt.s.h",     0, {"F", "ZFH", 0},  "D,S",      MATCH_FCVT_S_H, MASK_FCVT_S_H | MASK_RM, match_opcode, 0 },
{"fcvt.h.s",     0, {"F", "ZFH", 0},  "D,S",      MATCH_FCVT_H_S | MASK_RM, MASK_FCVT_H_S | MASK_RM, match_opcode, 0 },
{"fcvt.h.s",     0, {"F", "ZFH", 0},  "D,S,m",    MATCH_FCVT_H_S, MASK_FCVT_H_S, match_opcode, 0 },

{"fcvt.d.h",     0, {"D", "ZFH", 0},  "D,S",      MATCH_FCVT_D_H, MASK_FCVT_D_H | MASK_RM, match_opcode, 0 },
{"fcvt.h.d",     0, {"D", "ZFH", 0},  "D,S",      MATCH_FCVT_H_D | MASK_RM, MASK_FCVT_H_D | MASK_RM, match_opcode, 0 },
{"fcvt.h.d",     0, {"D", "ZFH", 0},  "D,S,m",    MATCH_FCVT_H_D, MASK_FCVT_H_D, match_opcode, 0 },

/* Single-precision floating-point instruction subset */
{"frsr",      0, {"F", 0},   "d",  MATCH_FRCSR, MASK_FRCSR, match_opcode, 0 },
{"fssr",      0, {"F", 0},   "s",  MATCH_FSCSR, MASK_FSCSR | MASK_RD, match_opcode, 0 },
{"fssr",      0, {"F", 0},   "d,s",  MATCH_FSCSR, MASK_FSCSR, match_opcode, 0 },
{"frcsr",     0, {"F", 0},   "d",  MATCH_FRCSR, MASK_FRCSR, match_opcode, 0 },
{"fscsr",     0, {"F", 0},   "s",  MATCH_FSCSR, MASK_FSCSR | MASK_RD, match_opcode, 0 },
{"fscsr",     0, {"F", 0},   "d,s",  MATCH_FSCSR, MASK_FSCSR, match_opcode, 0 },
{"frrm",      0, {"F", 0},   "d",  MATCH_FRRM, MASK_FRRM, match_opcode, 0 },
{"fsrm",      0, {"F", 0},   "s",  MATCH_FSRM, MASK_FSRM | MASK_RD, match_opcode, 0 },
{"fsrm",      0, {"F", 0},   "d,s",  MATCH_FSRM, MASK_FSRM, match_opcode, 0 },
{"fsrmi",     0, {"F", 0},   "d,Z",  MATCH_FSRMI, MASK_FSRMI, match_opcode, 0 },
{"fsrmi",     0, {"F", 0},   "Z",  MATCH_FSRMI, MASK_FSRMI | MASK_RD, match_opcode, 0 },
{"frflags",   0, {"F", 0},   "d",  MATCH_FRFLAGS, MASK_FRFLAGS, match_opcode, 0 },
{"fsflags",   0, {"F", 0},   "s",  MATCH_FSFLAGS, MASK_FSFLAGS | MASK_RD, match_opcode, 0 },
{"fsflags",   0, {"F", 0},   "d,s",  MATCH_FSFLAGS, MASK_FSFLAGS, match_opcode, 0 },
{"fsflagsi",  0, {"F", 0},   "d,Z",  MATCH_FSFLAGSI, MASK_FSFLAGSI, match_opcode, 0 },
{"fsflagsi",  0, {"F", 0},   "Z",  MATCH_FSFLAGSI, MASK_FSFLAGSI | MASK_RD, match_opcode, 0 },
{"flw",      32, {"F", "C", 0}, "D,Cm(Cc)",  MATCH_C_FLWSP, MASK_C_FLWSP, match_opcode, INSN_ALIAS|INSN_DREF|INSN_4_BYTE },
{"flw",      32, {"F", "C", 0}, "CD,Ck(Cs)",  MATCH_C_FLW, MASK_C_FLW, match_opcode, INSN_ALIAS|INSN_DREF|INSN_4_BYTE },
{"flw",       0, {"F", 0},   "D,o(s)",  MATCH_FLW, MASK_FLW, match_opcode, INSN_DREF|INSN_4_BYTE },
{"flw",       0, {"F", 0},   "D,A,s",  0, (int) M_FLW, match_never, INSN_MACRO },
{"fsw",      32, {"F", "C", 0}, "CT,CM(Cc)",  MATCH_C_FSWSP, MASK_C_FSWSP, match_opcode, INSN_ALIAS|INSN_DREF|INSN_4_BYTE },
{"fsw",      32, {"F", "C", 0}, "CD,Ck(Cs)",  MATCH_C_FSW, MASK_C_FSW, match_opcode, INSN_ALIAS|INSN_DREF|INSN_4_BYTE },
{"fsw",       0, {"F", 0},   "T,q(s)",  MATCH_FSW, MASK_FSW, match_opcode, INSN_DREF|INSN_4_BYTE },
{"fsw",       0, {"F", 0},   "T,A,s",  0, (int) M_FSW, match_never, INSN_MACRO },

{"fmv.x.w",    0, {"F", 0},   "d,S",  MATCH_FMV_X_S, MASK_FMV_X_S, match_opcode, 0 },
{"fmv.w.x",    0, {"F", 0},   "D,s",  MATCH_FMV_S_X, MASK_FMV_S_X, match_opcode, 0 },

{"fmv.x.s",    0, {"F", 0},   "d,S",  MATCH_FMV_X_S, MASK_FMV_X_S, match_opcode, 0 },
{"fmv.s.x",    0, {"F", 0},   "D,s",  MATCH_FMV_S_X, MASK_FMV_S_X, match_opcode, 0 },

{"fmv.s",      0, {"F", 0},   "D,U",  MATCH_FSGNJ_S, MASK_FSGNJ_S, match_rs1_eq_rs2, INSN_ALIAS },
{"fneg.s",     0, {"F", 0},   "D,U",  MATCH_FSGNJN_S, MASK_FSGNJN_S, match_rs1_eq_rs2, INSN_ALIAS },
{"fabs.s",     0, {"F", 0},   "D,U",  MATCH_FSGNJX_S, MASK_FSGNJX_S, match_rs1_eq_rs2, INSN_ALIAS },
{"fsgnj.s",    0, {"F", 0},   "D,S,T",  MATCH_FSGNJ_S, MASK_FSGNJ_S, match_opcode, 0 },
{"fsgnjn.s",   0, {"F", 0},   "D,S,T",  MATCH_FSGNJN_S, MASK_FSGNJN_S, match_opcode, 0 },
{"fsgnjx.s",   0, {"F", 0},   "D,S,T",  MATCH_FSGNJX_S, MASK_FSGNJX_S, match_opcode, 0 },
{"fadd.s",     0, {"F", 0},   "D,S,T",  MATCH_FADD_S | MASK_RM, MASK_FADD_S | MASK_RM, match_opcode, 0 },
{"fadd.s",     0, {"F", 0},   "D,S,T,m",  MATCH_FADD_S, MASK_FADD_S, match_opcode, 0 },
{"fsub.s",     0, {"F", 0},   "D,S,T",  MATCH_FSUB_S | MASK_RM, MASK_FSUB_S | MASK_RM, match_opcode, 0 },
{"fsub.s",     0, {"F", 0},   "D,S,T,m",  MATCH_FSUB_S, MASK_FSUB_S, match_opcode, 0 },
{"fmul.s",     0, {"F", 0},   "D,S,T",  MATCH_FMUL_S | MASK_RM, MASK_FMUL_S | MASK_RM, match_opcode, 0 },
{"fmul.s",     0, {"F", 0},   "D,S,T,m",  MATCH_FMUL_S, MASK_FMUL_S, match_opcode, 0 },
{"fdiv.s",     0, {"F", 0},   "D,S,T",  MATCH_FDIV_S | MASK_RM, MASK_FDIV_S | MASK_RM, match_opcode, 0 },
{"fdiv.s",     0, {"F", 0},   "D,S,T,m",  MATCH_FDIV_S, MASK_FDIV_S, match_opcode, 0 },
{"fsqrt.s",    0, {"F", 0},   "D,S",  MATCH_FSQRT_S | MASK_RM, MASK_FSQRT_S | MASK_RM, match_opcode, 0 },
{"fsqrt.s",    0, {"F", 0},   "D,S,m",  MATCH_FSQRT_S, MASK_FSQRT_S, match_opcode, 0 },
{"fmin.s",     0, {"F", 0},   "D,S,T",  MATCH_FMIN_S, MASK_FMIN_S, match_opcode, 0 },
{"fmax.s",     0, {"F", 0},   "D,S,T",  MATCH_FMAX_S, MASK_FMAX_S, match_opcode, 0 },
{"fmadd.s",    0, {"F", 0},   "D,S,T,R",  MATCH_FMADD_S | MASK_RM, MASK_FMADD_S | MASK_RM, match_opcode, 0 },
{"fmadd.s",    0, {"F", 0},   "D,S,T,R,m",  MATCH_FMADD_S, MASK_FMADD_S, match_opcode, 0 },
{"fnmadd.s",   0, {"F", 0},   "D,S,T,R",  MATCH_FNMADD_S | MASK_RM, MASK_FNMADD_S | MASK_RM, match_opcode, 0 },
{"fnmadd.s",   0, {"F", 0},   "D,S,T,R,m",  MATCH_FNMADD_S, MASK_FNMADD_S, match_opcode, 0 },
{"fmsub.s",    0, {"F", 0},   "D,S,T,R",  MATCH_FMSUB_S | MASK_RM, MASK_FMSUB_S | MASK_RM, match_opcode, 0 },
{"fmsub.s",    0, {"F", 0},   "D,S,T,R,m",  MATCH_FMSUB_S, MASK_FMSUB_S, match_opcode, 0 },
{"fnmsub.s",   0, {"F", 0},   "D,S,T,R",  MATCH_FNMSUB_S | MASK_RM, MASK_FNMSUB_S | MASK_RM, match_opcode, 0 },
{"fnmsub.s",   0, {"F", 0},   "D,S,T,R,m",  MATCH_FNMSUB_S, MASK_FNMSUB_S, match_opcode, 0 },
{"fcvt.w.s",   0, {"F", 0},   "d,S",  MATCH_FCVT_W_S | MASK_RM, MASK_FCVT_W_S | MASK_RM, match_opcode, 0 },
{"fcvt.w.s",   0, {"F", 0},   "d,S,m",  MATCH_FCVT_W_S, MASK_FCVT_W_S, match_opcode, 0 },
{"fcvt.wu.s",  0, {"F", 0},   "d,S",  MATCH_FCVT_WU_S | MASK_RM, MASK_FCVT_WU_S | MASK_RM, match_opcode, 0 },
{"fcvt.wu.s",  0, {"F", 0},   "d,S,m",  MATCH_FCVT_WU_S, MASK_FCVT_WU_S, match_opcode, 0 },
{"fcvt.s.w",   0, {"F", 0},   "D,s",  MATCH_FCVT_S_W | MASK_RM, MASK_FCVT_S_W | MASK_RM, match_opcode, 0 },
{"fcvt.s.w",   0, {"F", 0},   "D,s,m",  MATCH_FCVT_S_W, MASK_FCVT_S_W, match_opcode, 0 },
{"fcvt.s.wu",  0, {"F", 0},   "D,s",  MATCH_FCVT_S_WU | MASK_RM, MASK_FCVT_S_W | MASK_RM, match_opcode, 0 },
{"fcvt.s.wu",  0, {"F", 0},   "D,s,m",  MATCH_FCVT_S_WU, MASK_FCVT_S_WU, match_opcode, 0 },
{"fclass.s",   0, {"F", 0},   "d,S",  MATCH_FCLASS_S, MASK_FCLASS_S, match_opcode, 0 },
{"feq.s",      0, {"F", 0},   "d,S,T",    MATCH_FEQ_S, MASK_FEQ_S, match_opcode, 0 },
{"flt.s",      0, {"F", 0},   "d,S,T",    MATCH_FLT_S, MASK_FLT_S, match_opcode, 0 },
{"fle.s",      0, {"F", 0},   "d,S,T",    MATCH_FLE_S, MASK_FLE_S, match_opcode, 0 },
{"fgt.s",      0, {"F", 0},   "d,T,S",    MATCH_FLT_S, MASK_FLT_S, match_opcode, 0 },
{"fge.s",      0, {"F", 0},   "d,T,S",    MATCH_FLE_S, MASK_FLE_S, match_opcode, 0 },
{"fcvt.l.s",  64, {"F", 0}, "d,S",  MATCH_FCVT_L_S | MASK_RM, MASK_FCVT_L_S | MASK_RM, match_opcode, 0 },
{"fcvt.l.s",  64, {"F", 0}, "d,S,m",  MATCH_FCVT_L_S, MASK_FCVT_L_S, match_opcode, 0 },
{"fcvt.lu.s", 64, {"F", 0}, "d,S",  MATCH_FCVT_LU_S | MASK_RM, MASK_FCVT_LU_S | MASK_RM, match_opcode, 0 },
{"fcvt.lu.s", 64, {"F", 0}, "d,S,m",  MATCH_FCVT_LU_S, MASK_FCVT_LU_S, match_opcode, 0 },
{"fcvt.s.l",  64, {"F", 0}, "D,s",  MATCH_FCVT_S_L | MASK_RM, MASK_FCVT_S_L | MASK_RM, match_opcode, 0 },
{"fcvt.s.l",  64, {"F", 0}, "D,s,m",  MATCH_FCVT_S_L, MASK_FCVT_S_L, match_opcode, 0 },
{"fcvt.s.lu", 64, {"F", 0}, "D,s",  MATCH_FCVT_S_LU | MASK_RM, MASK_FCVT_S_L | MASK_RM, match_opcode, 0 },
{"fcvt.s.lu", 64, {"F", 0}, "D,s,m",  MATCH_FCVT_S_LU, MASK_FCVT_S_LU, match_opcode, 0 },

/* Double-precision floating-point instruction subset */
{"fld",        0, {"D", "C", 0},   "D,Cn(Cc)",  MATCH_C_FLDSP, MASK_C_FLDSP, match_opcode, INSN_ALIAS|INSN_DREF|INSN_8_BYTE },
{"fld",        0, {"D", "C", 0},   "CD,Cl(Cs)",  MATCH_C_FLD, MASK_C_FLD, match_opcode, INSN_ALIAS|INSN_DREF|INSN_8_BYTE },
{"fld",        0, {"D", 0},   "D,o(s)",  MATCH_FLD, MASK_FLD, match_opcode, INSN_DREF|INSN_8_BYTE },
{"fld",        0, {"D", 0},   "D,A,s",  0, (int) M_FLD, match_never, INSN_MACRO },
{"fsd",        0, {"D", "C", 0},   "CT,CN(Cc)",  MATCH_C_FSDSP, MASK_C_FSDSP, match_opcode, INSN_ALIAS|INSN_DREF|INSN_8_BYTE },
{"fsd",        0, {"D", "C", 0},   "CD,Cl(Cs)",  MATCH_C_FSD, MASK_C_FSD, match_opcode, INSN_ALIAS|INSN_DREF|INSN_8_BYTE },
{"fsd",        0, {"D", 0},   "T,q(s)",  MATCH_FSD, MASK_FSD, match_opcode, INSN_DREF|INSN_8_BYTE },
{"fsd",        0, {"D", 0},   "T,A,s",  0, (int) M_FSD, match_never, INSN_MACRO },
{"fmv.d",      0, {"D", 0},   "D,U",  MATCH_FSGNJ_D, MASK_FSGNJ_D, match_rs1_eq_rs2, INSN_ALIAS },
{"fneg.d",     0, {"D", 0},   "D,U",  MATCH_FSGNJN_D, MASK_FSGNJN_D, match_rs1_eq_rs2, INSN_ALIAS },
{"fabs.d",     0, {"D", 0},   "D,U",  MATCH_FSGNJX_D, MASK_FSGNJX_D, match_rs1_eq_rs2, INSN_ALIAS },
{"fsgnj.d",    0, {"D", 0},   "D,S,T",  MATCH_FSGNJ_D, MASK_FSGNJ_D, match_opcode, 0 },
{"fsgnjn.d",   0, {"D", 0},   "D,S,T",  MATCH_FSGNJN_D, MASK_FSGNJN_D, match_opcode, 0 },
{"fsgnjx.d",   0, {"D", 0},   "D,S,T",  MATCH_FSGNJX_D, MASK_FSGNJX_D, match_opcode, 0 },
{"fadd.d",     0, {"D", 0},   "D,S,T",  MATCH_FADD_D | MASK_RM, MASK_FADD_D | MASK_RM, match_opcode, 0 },
{"fadd.d",     0, {"D", 0},   "D,S,T,m",  MATCH_FADD_D, MASK_FADD_D, match_opcode, 0 },
{"fsub.d",     0, {"D", 0},   "D,S,T",  MATCH_FSUB_D | MASK_RM, MASK_FSUB_D | MASK_RM, match_opcode, 0 },
{"fsub.d",     0, {"D", 0},   "D,S,T,m",  MATCH_FSUB_D, MASK_FSUB_D, match_opcode, 0 },
{"fmul.d",     0, {"D", 0},   "D,S,T",  MATCH_FMUL_D | MASK_RM, MASK_FMUL_D | MASK_RM, match_opcode, 0 },
{"fmul.d",     0, {"D", 0},   "D,S,T,m",  MATCH_FMUL_D, MASK_FMUL_D, match_opcode, 0 },
{"fdiv.d",     0, {"D", 0},   "D,S,T",  MATCH_FDIV_D | MASK_RM, MASK_FDIV_D | MASK_RM, match_opcode, 0 },
{"fdiv.d",     0, {"D", 0},   "D,S,T,m",  MATCH_FDIV_D, MASK_FDIV_D, match_opcode, 0 },
{"fsqrt.d",    0, {"D", 0},   "D,S",  MATCH_FSQRT_D | MASK_RM, MASK_FSQRT_D | MASK_RM, match_opcode, 0 },
{"fsqrt.d",    0, {"D", 0},   "D,S,m",  MATCH_FSQRT_D, MASK_FSQRT_D, match_opcode, 0 },
{"fmin.d",     0, {"D", 0},   "D,S,T",  MATCH_FMIN_D, MASK_FMIN_D, match_opcode, 0 },
{"fmax.d",     0, {"D", 0},   "D,S,T",  MATCH_FMAX_D, MASK_FMAX_D, match_opcode, 0 },
{"fmadd.d",    0, {"D", 0},   "D,S,T,R",  MATCH_FMADD_D | MASK_RM, MASK_FMADD_D | MASK_RM, match_opcode, 0 },
{"fmadd.d",    0, {"D", 0},   "D,S,T,R,m",  MATCH_FMADD_D, MASK_FMADD_D, match_opcode, 0 },
{"fnmadd.d",   0, {"D", 0},   "D,S,T,R",  MATCH_FNMADD_D | MASK_RM, MASK_FNMADD_D | MASK_RM, match_opcode, 0 },
{"fnmadd.d",   0, {"D", 0},   "D,S,T,R,m",  MATCH_FNMADD_D, MASK_FNMADD_D, match_opcode, 0 },
{"fmsub.d",    0, {"D", 0},   "D,S,T,R",  MATCH_FMSUB_D | MASK_RM, MASK_FMSUB_D | MASK_RM, match_opcode, 0 },
{"fmsub.d",    0, {"D", 0},   "D,S,T,R,m",  MATCH_FMSUB_D, MASK_FMSUB_D, match_opcode, 0 },
{"fnmsub.d",   0, {"D", 0},   "D,S,T,R",  MATCH_FNMSUB_D | MASK_RM, MASK_FNMSUB_D | MASK_RM, match_opcode, 0 },
{"fnmsub.d",   0, {"D", 0},   "D,S,T,R,m",  MATCH_FNMSUB_D, MASK_FNMSUB_D, match_opcode, 0 },
{"fcvt.w.d",   0, {"D", 0},   "d,S",  MATCH_FCVT_W_D | MASK_RM, MASK_FCVT_W_D | MASK_RM, match_opcode, 0 },
{"fcvt.w.d",   0, {"D", 0},   "d,S,m",  MATCH_FCVT_W_D, MASK_FCVT_W_D, match_opcode, 0 },
{"fcvt.wu.d",  0, {"D", 0},   "d,S",  MATCH_FCVT_WU_D | MASK_RM, MASK_FCVT_WU_D | MASK_RM, match_opcode, 0 },
{"fcvt.wu.d",  0, {"D", 0},   "d,S,m",  MATCH_FCVT_WU_D, MASK_FCVT_WU_D, match_opcode, 0 },
{"fcvt.d.w",   0, {"D", 0},   "D,s",  MATCH_FCVT_D_W, MASK_FCVT_D_W | MASK_RM, match_opcode, 0 },
{"fcvt.d.wu",  0, {"D", 0},   "D,s",  MATCH_FCVT_D_WU, MASK_FCVT_D_WU | MASK_RM, match_opcode, 0 },
{"fcvt.d.s",   0, {"D", 0},   "D,S",  MATCH_FCVT_D_S, MASK_FCVT_D_S | MASK_RM, match_opcode, 0 },
{"fcvt.s.d",   0, {"D", 0},   "D,S",  MATCH_FCVT_S_D | MASK_RM, MASK_FCVT_S_D | MASK_RM, match_opcode, 0 },
{"fcvt.s.d",   0, {"D", 0},   "D,S,m",  MATCH_FCVT_S_D, MASK_FCVT_S_D, match_opcode, 0 },
{"fclass.d",   0, {"D", 0},   "d,S",  MATCH_FCLASS_D, MASK_FCLASS_D, match_opcode, 0 },
{"feq.d",      0, {"D", 0},   "d,S,T",    MATCH_FEQ_D, MASK_FEQ_D, match_opcode, 0 },
{"flt.d",      0, {"D", 0},   "d,S,T",    MATCH_FLT_D, MASK_FLT_D, match_opcode, 0 },
{"fle.d",      0, {"D", 0},   "d,S,T",    MATCH_FLE_D, MASK_FLE_D, match_opcode, 0 },
{"fgt.d",      0, {"D", 0},   "d,T,S",    MATCH_FLT_D, MASK_FLT_D, match_opcode, 0 },
{"fge.d",      0, {"D", 0},   "d,T,S",    MATCH_FLE_D, MASK_FLE_D, match_opcode, 0 },
{"fmv.x.d",   64, {"D", 0}, "d,S",  MATCH_FMV_X_D, MASK_FMV_X_D, match_opcode, 0 },
{"fmv.d.x",   64, {"D", 0}, "D,s",  MATCH_FMV_D_X, MASK_FMV_D_X, match_opcode, 0 },
{"fcvt.l.d",  64, {"D", 0}, "d,S",  MATCH_FCVT_L_D | MASK_RM, MASK_FCVT_L_D | MASK_RM, match_opcode, 0 },
{"fcvt.l.d",  64, {"D", 0}, "d,S,m",  MATCH_FCVT_L_D, MASK_FCVT_L_D, match_opcode, 0 },
{"fcvt.lu.d", 64, {"D", 0}, "d,S",  MATCH_FCVT_LU_D | MASK_RM, MASK_FCVT_LU_D | MASK_RM, match_opcode, 0 },
{"fcvt.lu.d", 64, {"D", 0}, "d,S,m",  MATCH_FCVT_LU_D, MASK_FCVT_LU_D, match_opcode, 0 },
{"fcvt.d.l",  64, {"D", 0}, "D,s",  MATCH_FCVT_D_L | MASK_RM, MASK_FCVT_D_L | MASK_RM, match_opcode, 0 },
{"fcvt.d.l",  64, {"D", 0}, "D,s,m",  MATCH_FCVT_D_L, MASK_FCVT_D_L, match_opcode, 0 },
{"fcvt.d.lu", 64, {"D", 0}, "D,s",  MATCH_FCVT_D_LU | MASK_RM, MASK_FCVT_D_L | MASK_RM, match_opcode, 0 },
{"fcvt.d.lu", 64, {"D", 0}, "D,s,m",  MATCH_FCVT_D_LU, MASK_FCVT_D_LU, match_opcode, 0 },

/* Quad-precision floating-point instruction subset */
{"flq",        0, {"Q", 0},   "D,o(s)",  MATCH_FLQ, MASK_FLQ, match_opcode, INSN_DREF|INSN_16_BYTE },
{"flq",        0, {"Q", 0},   "D,A,s",  0, (int) M_FLQ, match_never, INSN_MACRO },
{"fsq",        0, {"Q", 0},   "T,q(s)",  MATCH_FSQ, MASK_FSQ, match_opcode, INSN_DREF|INSN_16_BYTE },
{"fsq",        0, {"Q", 0},   "T,A,s",  0, (int) M_FSQ, match_never, INSN_MACRO },
{"fmv.q",      0, {"Q", 0},   "D,U",  MATCH_FSGNJ_Q, MASK_FSGNJ_Q, match_rs1_eq_rs2, INSN_ALIAS },
{"fneg.q",     0, {"Q", 0},   "D,U",  MATCH_FSGNJN_Q, MASK_FSGNJN_Q, match_rs1_eq_rs2, INSN_ALIAS },
{"fabs.q",     0, {"Q", 0},   "D,U",  MATCH_FSGNJX_Q, MASK_FSGNJX_Q, match_rs1_eq_rs2, INSN_ALIAS },
{"fsgnj.q",    0, {"Q", 0},   "D,S,T",  MATCH_FSGNJ_Q, MASK_FSGNJ_Q, match_opcode, 0 },
{"fsgnjn.q",   0, {"Q", 0},   "D,S,T",  MATCH_FSGNJN_Q, MASK_FSGNJN_Q, match_opcode, 0 },
{"fsgnjx.q",   0, {"Q", 0},   "D,S,T",  MATCH_FSGNJX_Q, MASK_FSGNJX_Q, match_opcode, 0 },
{"fadd.q",     0, {"Q", 0},   "D,S,T",  MATCH_FADD_Q | MASK_RM, MASK_FADD_Q | MASK_RM, match_opcode, 0 },
{"fadd.q",     0, {"Q", 0},   "D,S,T,m",  MATCH_FADD_Q, MASK_FADD_Q, match_opcode, 0 },
{"fsub.q",     0, {"Q", 0},   "D,S,T",  MATCH_FSUB_Q | MASK_RM, MASK_FSUB_Q | MASK_RM, match_opcode, 0 },
{"fsub.q",     0, {"Q", 0},   "D,S,T,m",  MATCH_FSUB_Q, MASK_FSUB_Q, match_opcode, 0 },
{"fmul.q",     0, {"Q", 0},   "D,S,T",  MATCH_FMUL_Q | MASK_RM, MASK_FMUL_Q | MASK_RM, match_opcode, 0 },
{"fmul.q",     0, {"Q", 0},   "D,S,T,m",  MATCH_FMUL_Q, MASK_FMUL_Q, match_opcode, 0 },
{"fdiv.q",     0, {"Q", 0},   "D,S,T",  MATCH_FDIV_Q | MASK_RM, MASK_FDIV_Q | MASK_RM, match_opcode, 0 },
{"fdiv.q",     0, {"Q", 0},   "D,S,T,m",  MATCH_FDIV_Q, MASK_FDIV_Q, match_opcode, 0 },
{"fsqrt.q",    0, {"Q", 0},   "D,S",  MATCH_FSQRT_Q | MASK_RM, MASK_FSQRT_Q | MASK_RM, match_opcode, 0 },
{"fsqrt.q",    0, {"Q", 0},   "D,S,m",  MATCH_FSQRT_Q, MASK_FSQRT_Q, match_opcode, 0 },
{"fmin.q",     0, {"Q", 0},   "D,S,T",  MATCH_FMIN_Q, MASK_FMIN_Q, match_opcode, 0 },
{"fmax.q",     0, {"Q", 0},   "D,S,T",  MATCH_FMAX_Q, MASK_FMAX_Q, match_opcode, 0 },
{"fmadd.q",    0, {"Q", 0},   "D,S,T,R",  MATCH_FMADD_Q | MASK_RM, MASK_FMADD_Q | MASK_RM, match_opcode, 0 },
{"fmadd.q",    0, {"Q", 0},   "D,S,T,R,m",  MATCH_FMADD_Q, MASK_FMADD_Q, match_opcode, 0 },
{"fnmadd.q",   0, {"Q", 0},   "D,S,T,R",  MATCH_FNMADD_Q | MASK_RM, MASK_FNMADD_Q | MASK_RM, match_opcode, 0 },
{"fnmadd.q",   0, {"Q", 0},   "D,S,T,R,m",  MATCH_FNMADD_Q, MASK_FNMADD_Q, match_opcode, 0 },
{"fmsub.q",    0, {"Q", 0},   "D,S,T,R",  MATCH_FMSUB_Q | MASK_RM, MASK_FMSUB_Q | MASK_RM, match_opcode, 0 },
{"fmsub.q",    0, {"Q", 0},   "D,S,T,R,m",  MATCH_FMSUB_Q, MASK_FMSUB_Q, match_opcode, 0 },
{"fnmsub.q",   0, {"Q", 0},   "D,S,T,R",  MATCH_FNMSUB_Q | MASK_RM, MASK_FNMSUB_Q | MASK_RM, match_opcode, 0 },
{"fnmsub.q",   0, {"Q", 0},   "D,S,T,R,m",  MATCH_FNMSUB_Q, MASK_FNMSUB_Q, match_opcode, 0 },
{"fcvt.w.q",   0, {"Q", 0},   "d,S",  MATCH_FCVT_W_Q | MASK_RM, MASK_FCVT_W_Q | MASK_RM, match_opcode, 0 },
{"fcvt.w.q",   0, {"Q", 0},   "d,S,m",  MATCH_FCVT_W_Q, MASK_FCVT_W_Q, match_opcode, 0 },
{"fcvt.wu.q",  0, {"Q", 0},   "d,S",  MATCH_FCVT_WU_Q | MASK_RM, MASK_FCVT_WU_Q | MASK_RM, match_opcode, 0 },
{"fcvt.wu.q",  0, {"Q", 0},   "d,S,m",  MATCH_FCVT_WU_Q, MASK_FCVT_WU_Q, match_opcode, 0 },
{"fcvt.q.w",   0, {"Q", 0},   "D,s",  MATCH_FCVT_Q_W, MASK_FCVT_Q_W | MASK_RM, match_opcode, 0 },
{"fcvt.q.wu",  0, {"Q", 0},   "D,s",  MATCH_FCVT_Q_WU, MASK_FCVT_Q_WU | MASK_RM, match_opcode, 0 },
{"fcvt.q.s",   0, {"Q", 0},   "D,S",  MATCH_FCVT_Q_S, MASK_FCVT_Q_S | MASK_RM, match_opcode, 0 },
{"fcvt.q.d",   0, {"Q", 0},   "D,S",  MATCH_FCVT_Q_D, MASK_FCVT_Q_D | MASK_RM, match_opcode, 0 },
{"fcvt.s.q",   0, {"Q", 0},   "D,S",  MATCH_FCVT_S_Q | MASK_RM, MASK_FCVT_S_Q | MASK_RM, match_opcode, 0 },
{"fcvt.s.q",   0, {"Q", 0},   "D,S,m",  MATCH_FCVT_S_Q, MASK_FCVT_S_Q, match_opcode, 0 },
{"fcvt.d.q",   0, {"Q", 0},   "D,S",  MATCH_FCVT_D_Q | MASK_RM, MASK_FCVT_D_Q | MASK_RM, match_opcode, 0 },
{"fcvt.d.q",   0, {"Q", 0},   "D,S,m",  MATCH_FCVT_D_Q, MASK_FCVT_D_Q, match_opcode, 0 },
{"fclass.q",   0, {"Q", 0},   "d,S",  MATCH_FCLASS_Q, MASK_FCLASS_Q, match_opcode, 0 },
{"feq.q",      0, {"Q", 0},   "d,S,T",    MATCH_FEQ_Q, MASK_FEQ_Q, match_opcode, 0 },
{"flt.q",      0, {"Q", 0},   "d,S,T",    MATCH_FLT_Q, MASK_FLT_Q, match_opcode, 0 },
{"fle.q",      0, {"Q", 0},   "d,S,T",    MATCH_FLE_Q, MASK_FLE_Q, match_opcode, 0 },
{"fgt.q",      0, {"Q", 0},   "d,T,S",    MATCH_FLT_Q, MASK_FLT_Q, match_opcode, 0 },
{"fge.q",      0, {"Q", 0},   "d,T,S",    MATCH_FLE_Q, MASK_FLE_Q, match_opcode, 0 },
{"fmv.x.q",   64, {"Q", 0}, "d,S",  MATCH_FMV_X_Q, MASK_FMV_X_Q, match_opcode, 0 },
{"fmv.q.x",   64, {"Q", 0}, "D,s",  MATCH_FMV_Q_X, MASK_FMV_Q_X, match_opcode, 0 },
{"fcvt.l.q",  64, {"Q", 0}, "d,S",  MATCH_FCVT_L_Q | MASK_RM, MASK_FCVT_L_Q | MASK_RM, match_opcode, 0 },
{"fcvt.l.q",  64, {"Q", 0}, "d,S,m",  MATCH_FCVT_L_Q, MASK_FCVT_L_Q, match_opcode, 0 },
{"fcvt.lu.q", 64, {"Q", 0}, "d,S",  MATCH_FCVT_LU_Q | MASK_RM, MASK_FCVT_LU_Q | MASK_RM, match_opcode, 0 },
{"fcvt.lu.q", 64, {"Q", 0}, "d,S,m",  MATCH_FCVT_LU_Q, MASK_FCVT_LU_Q, match_opcode, 0 },
{"fcvt.q.l",  64, {"Q", 0}, "D,s",  MATCH_FCVT_Q_L | MASK_RM, MASK_FCVT_Q_L | MASK_RM, match_opcode, 0 },
{"fcvt.q.l",  64, {"Q", 0}, "D,s,m",  MATCH_FCVT_Q_L, MASK_FCVT_Q_L, match_opcode, 0 },
{"fcvt.q.lu", 64, {"Q", 0}, "D,s",  MATCH_FCVT_Q_LU | MASK_RM, MASK_FCVT_Q_L | MASK_RM, match_opcode, 0 },
{"fcvt.q.lu", 64, {"Q", 0}, "D,s,m",  MATCH_FCVT_Q_LU, MASK_FCVT_Q_LU, match_opcode, 0 },

/* Supervisor instructions */
{"csrr",       0, {"I", 0},   "d,E",  MATCH_CSRRS, MASK_CSRRS | MASK_RS1, match_opcode, INSN_ALIAS },
{"csrwi",      0, {"I", 0},   "E,Z",  MATCH_CSRRWI, MASK_CSRRWI | MASK_RD, match_opcode, INSN_ALIAS },
{"csrsi",      0, {"I", 0},   "E,Z",  MATCH_CSRRSI, MASK_CSRRSI | MASK_RD, match_opcode, INSN_ALIAS },
{"csrci",      0, {"I", 0},   "E,Z",  MATCH_CSRRCI, MASK_CSRRCI | MASK_RD, match_opcode, INSN_ALIAS },
{"csrw",       0, {"I", 0},   "E,s",  MATCH_CSRRW, MASK_CSRRW | MASK_RD, match_opcode, INSN_ALIAS },
{"csrw",       0, {"I", 0},   "E,Z",  MATCH_CSRRWI, MASK_CSRRWI | MASK_RD, match_opcode, INSN_ALIAS },
{"csrs",       0, {"I", 0},   "E,s",  MATCH_CSRRS, MASK_CSRRS | MASK_RD, match_opcode, INSN_ALIAS },
{"csrs",       0, {"I", 0},   "E,Z",  MATCH_CSRRSI, MASK_CSRRSI | MASK_RD, match_opcode, INSN_ALIAS },
{"csrc",       0, {"I", 0},   "E,s",  MATCH_CSRRC, MASK_CSRRC | MASK_RD, match_opcode, INSN_ALIAS },
{"csrc",       0, {"I", 0},   "E,Z",  MATCH_CSRRCI, MASK_CSRRCI | MASK_RD, match_opcode, INSN_ALIAS },
{"csrrwi",     0, {"I", 0},   "d,E,Z",  MATCH_CSRRWI, MASK_CSRRWI, match_opcode, 0 },
{"csrrsi",     0, {"I", 0},   "d,E,Z",  MATCH_CSRRSI, MASK_CSRRSI, match_opcode, 0 },
{"csrrci",     0, {"I", 0},   "d,E,Z",  MATCH_CSRRCI, MASK_CSRRCI, match_opcode, 0 },
{"csrrw",      0, {"I", 0},   "d,E,s",  MATCH_CSRRW, MASK_CSRRW, match_opcode, 0 },
{"csrrw",      0, {"I", 0},   "d,E,Z",  MATCH_CSRRWI, MASK_CSRRWI, match_opcode, INSN_ALIAS },
{"csrrs",      0, {"I", 0},   "d,E,s",  MATCH_CSRRS, MASK_CSRRS, match_opcode, 0 },
{"csrrs",      0, {"I", 0},   "d,E,Z",  MATCH_CSRRSI, MASK_CSRRSI, match_opcode, INSN_ALIAS },
{"csrrc",      0, {"I", 0},   "d,E,s",  MATCH_CSRRC, MASK_CSRRC, match_opcode, 0 },
{"csrrc",      0, {"I", 0},   "d,E,Z",  MATCH_CSRRCI, MASK_CSRRCI, match_opcode, INSN_ALIAS },
{"uret",       0, {"I", 0},   "",     MATCH_URET, MASK_URET, match_opcode, 0 },
{"sret",       0, {"I", 0},   "",     MATCH_SRET, MASK_SRET, match_opcode, 0 },
{"hret",       0, {"I", 0},   "",     MATCH_HRET, MASK_HRET, match_opcode, 0 },
{"mret",       0, {"I", 0},   "",     MATCH_MRET, MASK_MRET, match_opcode, 0 },
{"dret",       0, {"I", 0},   "",     MATCH_DRET, MASK_DRET, match_opcode, 0 },
{"sfence.vm",  0, {"I", 0},   "",     MATCH_SFENCE_VM, MASK_SFENCE_VM | MASK_RS1, match_opcode, 0 },
{"sfence.vm",  0, {"I", 0},   "s",    MATCH_SFENCE_VM, MASK_SFENCE_VM, match_opcode, 0 },
{"sfence.vma", 0, {"I", 0},   "",     MATCH_SFENCE_VMA, MASK_SFENCE_VMA | MASK_RS1 | MASK_RS2, match_opcode, INSN_ALIAS },
{"sfence.vma", 0, {"I", 0},   "s",    MATCH_SFENCE_VMA, MASK_SFENCE_VMA | MASK_RS2, match_opcode, INSN_ALIAS },
{"sfence.vma", 0, {"I", 0},   "s,t",  MATCH_SFENCE_VMA, MASK_SFENCE_VMA, match_opcode, 0 },
{"wfi",        0, {"I", 0},   "",     MATCH_WFI, MASK_WFI, match_opcode, 0 },

/* Zbb or zbkb instructions  */
{"clz",        0, {"ZBB", 0},  "d,s",      MATCH_CLZ, MASK_CLZ, match_opcode, 0 },
{"ctz",        0, {"ZBB", 0},  "d,s",      MATCH_CTZ, MASK_CTZ, match_opcode, 0 },
{"cpop",       0, {"ZBB", 0},  "d,s",      MATCH_CPOP, MASK_CPOP, match_opcode, 0 },
{"min",        0, {"ZBB", 0},  "d,s,t",    MATCH_MIN, MASK_MIN, match_opcode, 0 },
{"max",        0, {"ZBB", 0},  "d,s,t",    MATCH_MAX, MASK_MAX, match_opcode, 0 },
{"minu",       0, {"ZBB", 0},  "d,s,t",    MATCH_MINU, MASK_MINU, match_opcode, 0 },
{"maxu",       0, {"ZBB", 0},  "d,s,t",    MATCH_MAXU, MASK_MAXU, match_opcode, 0 },
{"sext.b",     0, {"ZBB", 0},  "d,s",      MATCH_SEXT_B, MASK_SEXT_B, match_opcode, 0 },
{"sext.b",     0, {"I", 0},    "d,s",      0, (int) M_SEXTB, match_never, INSN_MACRO },
{"sext.h",     0, {"ZBB", 0},  "d,s",      MATCH_SEXT_H, MASK_SEXT_H, match_opcode, 0 },
{"sext.h",     0, {"I", 0},    "d,s",      0, (int) M_SEXTH, match_never, INSN_MACRO },
{"zext.h",    32, {"ZBB", 0},  "d,s",      MATCH_ZEXT_H_32, MASK_ZEXT_H | MASK_RS2, match_opcode, 0 },
{"zext.h",    64, {"ZBB", 0},  "d,s",      MATCH_ZEXT_H_64, MASK_ZEXT_H | MASK_RS2, match_opcode, 0 },
{"zext.h",     0, {"I", 0},    "d,s",      0, (int) M_ZEXTH, match_never, INSN_MACRO },
{"andn",       0, {"ZBB", 0},  "d,s,t",    MATCH_ANDN, MASK_ANDN, match_opcode, 0 },
{"orn",        0, {"ZBB", 0},  "d,s,t",    MATCH_ORN, MASK_ORN, match_opcode, 0 },
{"xnor",       0, {"ZBB", 0},  "d,s,t",    MATCH_XNOR, MASK_XNOR, match_opcode, 0 },
{"rol",        0, {"ZBB", 0},  "d,s,t",    MATCH_ROL, MASK_ROL, match_opcode, 0 },
{"rori",       0, {"ZBB", 0},  "d,s,>",    MATCH_RORI, MASK_RORI, match_opcode, 0 },
{"ror",        0, {"ZBB", 0},  "d,s,t",    MATCH_ROR, MASK_ROR, match_opcode, 0 },
{"ror",        0, {"ZBB", 0},  "d,s,>",    MATCH_RORI, MASK_RORI, match_opcode, INSN_ALIAS },
{"rev8",      32, {"ZBB", 0},  "d,s",      MATCH_GREVI|MATCH_SHAMT_REV8_32, MASK_GREVI|MASK_SHAMT, match_opcode, 0 },
{"rev8",      64, {"ZBB", 0},  "d,s",      MATCH_GREVI|MATCH_SHAMT_REV8_64, MASK_GREVI|MASK_SHAMT, match_opcode, 0 },
{"orc.b",      0, {"ZBB", 0},  "d,s",      MATCH_GORCI|MATCH_SHAMT_ORC_B, MASK_GORCI|MASK_SHAMT, match_opcode, 0 },
{"clzw",      64, {"ZBB", 0},  "d,s",      MATCH_CLZW, MASK_CLZW, match_opcode, 0 },
{"ctzw",      64, {"ZBB", 0},  "d,s",      MATCH_CTZW, MASK_CTZW, match_opcode, 0 },
{"cpopw",     64, {"ZBB", 0},  "d,s",      MATCH_CPOPW, MASK_CPOPW, match_opcode, 0 },
{"rolw",      64, {"ZBB", 0},  "d,s,t",    MATCH_ROLW, MASK_ROLW, match_opcode, 0 },
{"roriw",     64, {"ZBB", 0},  "d,s,<",    MATCH_RORIW, MASK_RORIW, match_opcode, 0 },
{"rorw",      64, {"ZBB", 0},  "d,s,t",    MATCH_RORW, MASK_RORW, match_opcode, 0 },
{"rorw",      64, {"ZBB", 0},  "d,s,<",    MATCH_RORIW, MASK_RORIW, match_opcode, 0 },
{"sh1add",     0, {"ZBA", 0},  "d,s,t",    MATCH_SH1ADD, MASK_SH1ADD, match_opcode, 0 },
{"sh2add",     0, {"ZBA", 0},  "d,s,t",    MATCH_SH2ADD, MASK_SH2ADD, match_opcode, 0 },
{"sh3add",     0, {"ZBA", 0},  "d,s,t",    MATCH_SH3ADD, MASK_SH3ADD, match_opcode, 0 },
{"sh1add.uw", 64, {"ZBA", 0},  "d,s,t",    MATCH_SH1ADD_UW, MASK_SH1ADD_UW, match_opcode, 0 },
{"sh2add.uw", 64, {"ZBA", 0},  "d,s,t",    MATCH_SH2ADD_UW, MASK_SH2ADD_UW, match_opcode, 0 },
{"sh3add.uw", 64, {"ZBA", 0},  "d,s,t",    MATCH_SH3ADD_UW, MASK_SH3ADD_UW, match_opcode, 0 },
{"zext.w",    64, {"ZBA", 0},  "d,s",      MATCH_ADD_UW, MASK_ADD_UW|MASK_RS2, match_opcode, INSN_ALIAS },
{"zext.w",    64, {"I", 0},    "d,s",      0, (int) M_ZEXTW, match_never, INSN_MACRO },
{"add.uw",    64, {"ZBA", 0},  "d,s,t",    MATCH_ADD_UW, MASK_ADD_UW, match_opcode, 0 },
{"slli.uw",   64, {"ZBA", 0},  "d,s,>",    MATCH_SLLI_UW, MASK_SLLI_UW, match_opcode, 0 },
{"clmul",      0, {"ZBC", 0},  "d,s,t",    MATCH_CLMUL, MASK_CLMUL, match_opcode, 0 },
{"clmulh",     0, {"ZBC", 0},  "d,s,t",    MATCH_CLMULH, MASK_CLMULH, match_opcode, 0 },
{"clmulr",     0, {"ZBC", 0},  "d,s,t",    MATCH_CLMULR, MASK_CLMULR, match_opcode, 0 },
{"bclr",       0, {"ZBS", 0},  "d,s,t",    MATCH_BCLR, MASK_BCLR, match_opcode, 0 },
{"bclri",      0, {"ZBS", 0},  "d,s,>",    MATCH_BCLRI, MASK_BCLRI, match_opcode, 0 },
{"bext",       0, {"ZBS", 0},  "d,s,t",    MATCH_BEXT, MASK_BEXT, match_opcode, 0 },
{"bexti",      0, {"ZBS", 0},  "d,s,>",    MATCH_BEXTI, MASK_BEXTI, match_opcode, 0 },
{"binv",       0, {"ZBS", 0},  "d,s,t",    MATCH_BINV, MASK_BINV, match_opcode, 0 },
{"binvi",      0, {"ZBS", 0},  "d,s,>",    MATCH_BINVI, MASK_BINVI, match_opcode, 0 },
{"bset",       0, {"ZBS", 0},  "d,s,t",    MATCH_BSET, MASK_BSET, match_opcode, 0 },
{"bseti",      0, {"ZBS", 0},  "d,s,>",    MATCH_BSETI, MASK_BSETI, match_opcode, 0 },

{"brev8",     32, {"ZBKB", 0},  "d,s",      MATCH_GREVI | MATCH_SHAMT_BREV8, MASK_GREVI | MASK_SHAMT, match_opcode, 0 },
{"brev8",     64, {"ZBKB", 0},  "d,s",      MATCH_GREVI | MATCH_SHAMT_BREV8, MASK_GREVI | MASK_SHAMT, match_opcode, 0 },
{"zip",       32, {"ZBKB", 0},  "d,s",      MATCH_SHFLI|MATCH_SHAMT_ZIP_32, MASK_SHFLI|MASK_SHAMT, match_opcode, INSN_ALIAS },
{"unzip",     32, {"ZBKB", 0},  "d,s",      MATCH_UNSHFLI|MATCH_SHAMT_ZIP_32, MASK_UNSHFLI|MASK_SHAMT, match_opcode, INSN_ALIAS },
{"pack",       0, {"ZBKB", 0},  "d,s,t",    MATCH_PACK, MASK_PACK, match_opcode, 0 },
{"packh",      0, {"ZBKB", 0},  "d,s,t",    MATCH_PACKH, MASK_PACKH, match_opcode, 0 },
{"packw",     64, {"ZBKB", 0},  "d,s,t",    MATCH_PACKW, MASK_PACKW, match_opcode, 0 },
// {"andn",       0, {"ZBB", "ZBKB", 0},  "d,s,t", MATCH_ANDN, MASK_ANDN, match_opcode, 0 },
// {"orn",        0, {"ZBB", "ZBKB", 0},  "d,s,t", MATCH_ORN, MASK_ORN, match_opcode, 0 },
// {"xnor",       0, {"ZBB", "ZBKB", 0},  "d,s,t", MATCH_XNOR, MASK_XNOR, match_opcode, 0 },
// {"rol",        0, {"ZBB", "ZBKB", 0},  "d,s,t", MATCH_ROL, MASK_ROL, match_opcode, 0 },
// {"rori",       0, {"ZBB", "ZBKB", 0},  "d,s,>", MATCH_RORI, MASK_RORI, match_opcode, 0 },
// {"ror",        0, {"ZBB", "ZBKB", 0},  "d,s,t", MATCH_ROR, MASK_ROR, match_opcode, 0 },
// {"ror",        0, {"ZBB", "ZBKB", 0},  "d,s,>", MATCH_RORI, MASK_RORI, match_opcode, INSN_ALIAS },
// {"rev8",      32, {"ZBB", "ZBKB", 0},  "d,s",   MATCH_GREVI | MATCH_SHAMT_REV8_32, MASK_GREVI | MASK_SHAMT, match_opcode, 0 },
// {"rev8",      64, {"ZBB", "ZBKB", 0},  "d,s",   MATCH_GREVI | MATCH_SHAMT_REV8_64, MASK_GREVI | MASK_SHAMT, match_opcode, 0 },
// {"rolw",      64, {"ZBB", "ZBKB", 0},  "d,s,t", MATCH_ROLW, MASK_ROLW, match_opcode, 0 },
// {"roriw",     64, {"ZBB", "ZBKB", 0},  "d,s,<", MATCH_RORIW, MASK_RORIW, match_opcode, 0 },
// {"rorw",      64, {"ZBB", "ZBKB", 0},  "d,s,t", MATCH_RORW, MASK_RORW, match_opcode, 0 },
// {"rorw",      64, {"ZBB", "ZBKB", 0},  "d,s,<", MATCH_RORIW, MASK_RORIW, match_opcode, 0 },

/* Zbc or zbkc instructions  */
// {"clmul",      0, {"ZBC", "ZBKC", 0},  "d,s,t", MATCH_CLMUL, MASK_CLMUL, match_opcode, 0 },
// {"clmulh",     0, {"ZBC", "ZBKC", 0},  "d,s,t", MATCH_CLMULH, MASK_CLMULH, match_opcode, 0 },

/* RVX Andes V5 Vector Dot Product Extension  */
{"vd4dots.vv",  0, {"XANDES", "V", 0},  "Vd,Vs,VtVm", MATCH_VD4DOTS_VV, MASK_VD4DOTS_VV, match_opcode, 0},
{"vd4dotu.vv",  0, {"XANDES", "V", 0},  "Vd,Vs,VtVm", MATCH_VD4DOTU_VV, MASK_VD4DOTU_VV, match_opcode, 0},
{"vd4dotsu.vv", 0, {"XANDES", "V", 0},  "Vd,Vs,VtVm", MATCH_VD4DOTSU_VV, MASK_VD4DOTSU_VV, match_opcode, 0},

/* RVV */
/* Andes V5 vector small INT handling extension  */
{"vle4.v",        0, {"XANDES", "V", 0},  "Vd,0(s)", MATCH_VLE4_V, MASK_VLE4_V, match_opcode, INSN_DREF },
{"vfwcvt.f.n.v",  0, {"XANDES", "V", "F", 0}, "Vd,VtVm", MATCH_VFWCVT_F_N_V, MASK_VFWCVT_F_N_V, match_widen_vd_neq_vs2_neq_vm, 0},
{"vfwcvt.f.nu.v", 0, {"XANDES", "V", "F", 0}, "Vd,VtVm", MATCH_VFWCVT_F_NU_V, MASK_VFWCVT_F_NU_V, match_widen_vd_neq_vs2_neq_vm, 0},
{"vfwcvt.f.b.v",  0, {"XANDES", "V", "F", 0}, "Vd,VtVm", MATCH_VFWCVT_F_B_V, MASK_VFWCVT_F_B_V, match_widen_vd_neq_vs2_neq_vm, 0},
{"vfwcvt.f.bu.v", 0, {"XANDES", "V", "F", 0}, "Vd,VtVm", MATCH_VFWCVT_F_BU_V, MASK_VFWCVT_F_BU_V, match_widen_vd_neq_vs2_neq_vm, 0},
{"vfpmadt.vf",    0, {"XANDES", "V", "F", 0}, "Vd,S,VtVm", MATCH_VFPMADT_VF, MASK_VFPMADT_VF, match_opcode, 0},
{"vfpmadb.vf",    0, {"XANDES", "V", "F", 0}, "Vd,S,VtVm", MATCH_VFPMADB_VF, MASK_VFPMADB_VF, match_opcode, 0},

/* Andes Xv5 INT4 Load (vln.v => vle4.v)  */
{"vln.v",       0, {"XANDES", "V", 0}, "Vd,0(s)Vm",  MATCH_VLN_V, MASK_VLN_V, match_opcode, INSN_DREF | INSN_ALIAS},
{"vlnu.v",      0, {"XANDES", "V", 0}, "Vd,0(s)Vm",  MATCH_VLNU_V, MASK_VLNU_V, match_opcode, INSN_DREF },
{"vln8.v",      0, {"XANDES", "V", 0}, "Vd,0(s)Vm", MATCH_VLN8_V, MASK_VLN8_V, match_opcode, INSN_DREF},
{"vlnu8.v",     0, {"XANDES", "V", 0}, "Vd,0(s)Vm", MATCH_VLNU8_V, MASK_VLNU8_V, match_opcode, INSN_DREF},

/* RVV standard exetension  */
{"vsetvl",     0, {"V", 0},  "d,s,t",  MATCH_VSETVL, MASK_VSETVL, match_opcode, 0},
{"vsetvli",    0, {"V", 0},  "d,s,Vc", MATCH_VSETVLI, MASK_VSETVLI, match_opcode, 0},
{"vsetivli",   0, {"V", 0},  "d,Z,Vb", MATCH_VSETIVLI, MASK_VSETIVLI, match_opcode, 0},

{"vle1.v",     0, {"V", 0},  "Vd,0(s)", MATCH_VLE1_V, MASK_VLE1_V, match_opcode, INSN_DREF },
{"vse1.v",     0, {"V", 0},  "Vd,0(s)", MATCH_VSE1_V, MASK_VSE1_V, match_opcode, INSN_DREF },

{"vle8.v",     0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLE8_V, MASK_VLE8_V, match_vd_neq_vm, INSN_DREF },
{"vle16.v",    0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLE16_V, MASK_VLE16_V, match_vd_neq_vm, INSN_DREF },
{"vle32.v",    0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLE32_V, MASK_VLE32_V, match_vd_neq_vm, INSN_DREF },
{"vle64.v",    0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLE64_V, MASK_VLE64_V, match_vd_neq_vm, INSN_DREF },

{"vse8.v",     0, {"V", 0},  "Vd,0(s)Vm", MATCH_VSE8_V, MASK_VSE8_V, match_vd_neq_vm, INSN_DREF },
{"vse16.v",    0, {"V", 0},  "Vd,0(s)Vm", MATCH_VSE16_V, MASK_VSE16_V, match_vd_neq_vm, INSN_DREF },
{"vse32.v",    0, {"V", 0},  "Vd,0(s)Vm", MATCH_VSE32_V, MASK_VSE32_V, match_vd_neq_vm, INSN_DREF },
{"vse64.v",    0, {"V", 0},  "Vd,0(s)Vm", MATCH_VSE64_V, MASK_VSE64_V, match_vd_neq_vm, INSN_DREF },

{"vlse8.v",    0, {"V", 0},  "Vd,0(s),tVm", MATCH_VLSE8_V, MASK_VLSE8_V, match_vd_neq_vm, INSN_DREF },
{"vlse16.v",   0, {"V", 0},  "Vd,0(s),tVm", MATCH_VLSE16_V, MASK_VLSE16_V, match_vd_neq_vm, INSN_DREF },
{"vlse32.v",   0, {"V", 0},  "Vd,0(s),tVm", MATCH_VLSE32_V, MASK_VLSE32_V, match_vd_neq_vm, INSN_DREF },
{"vlse64.v",   0, {"V", 0},  "Vd,0(s),tVm", MATCH_VLSE64_V, MASK_VLSE64_V, match_vd_neq_vm, INSN_DREF },

{"vsse8.v",    0, {"V", 0},  "Vd,0(s),tVm", MATCH_VSSE8_V, MASK_VSSE8_V, match_vd_neq_vm, INSN_DREF },
{"vsse16.v",   0, {"V", 0},  "Vd,0(s),tVm", MATCH_VSSE16_V, MASK_VSSE16_V, match_vd_neq_vm, INSN_DREF },
{"vsse32.v",   0, {"V", 0},  "Vd,0(s),tVm", MATCH_VSSE32_V, MASK_VSSE32_V, match_vd_neq_vm, INSN_DREF },
{"vsse64.v",   0, {"V", 0},  "Vd,0(s),tVm", MATCH_VSSE64_V, MASK_VSSE64_V, match_vd_neq_vm, INSN_DREF },

{"vloxei8.v",   0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLOXEI8_V, MASK_VLOXEI8_V, match_vd_neq_vm, INSN_DREF },
{"vloxei16.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLOXEI16_V, MASK_VLOXEI16_V, match_vd_neq_vm, INSN_DREF },
{"vloxei32.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLOXEI32_V, MASK_VLOXEI32_V, match_vd_neq_vm, INSN_DREF },
{"vloxei64.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLOXEI64_V, MASK_VLOXEI64_V, match_vd_neq_vm, INSN_DREF },

{"vsoxei8.v",   0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSOXEI8_V, MASK_VSOXEI8_V, match_vd_neq_vm, INSN_DREF },
{"vsoxei16.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSOXEI16_V, MASK_VSOXEI16_V, match_vd_neq_vm, INSN_DREF },
{"vsoxei32.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSOXEI32_V, MASK_VSOXEI32_V, match_vd_neq_vm, INSN_DREF },
{"vsoxei64.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSOXEI64_V, MASK_VSOXEI64_V, match_vd_neq_vm, INSN_DREF },

{"vluxei8.v",   0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLUXEI8_V, MASK_VLUXEI8_V, match_vd_neq_vm, INSN_DREF },
{"vluxei16.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLUXEI16_V, MASK_VLUXEI16_V, match_vd_neq_vm, INSN_DREF },
{"vluxei32.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLUXEI32_V, MASK_VLUXEI32_V, match_vd_neq_vm, INSN_DREF },
{"vluxei64.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLUXEI64_V, MASK_VLUXEI64_V, match_vd_neq_vm, INSN_DREF },

{"vsuxei8.v",   0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSUXEI8_V, MASK_VSUXEI8_V, match_vd_neq_vm, INSN_DREF },
{"vsuxei16.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSUXEI16_V, MASK_VSUXEI16_V, match_vd_neq_vm, INSN_DREF },
{"vsuxei32.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSUXEI32_V, MASK_VSUXEI32_V, match_vd_neq_vm, INSN_DREF },
{"vsuxei64.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSUXEI64_V, MASK_VSUXEI64_V, match_vd_neq_vm, INSN_DREF },

{"vle8ff.v",    0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLE8FF_V, MASK_VLE8FF_V, match_vd_neq_vm, INSN_DREF },
{"vle16ff.v",   0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLE16FF_V, MASK_VLE16FF_V, match_vd_neq_vm, INSN_DREF },
{"vle32ff.v",   0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLE32FF_V, MASK_VLE32FF_V, match_vd_neq_vm, INSN_DREF },
{"vle64ff.v",   0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLE64FF_V, MASK_VLE64FF_V, match_vd_neq_vm, INSN_DREF },

{"vlseg2e8.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG2E8_V, MASK_VLSEG2E8_V, match_vd_neq_vm, INSN_DREF },
{"vsseg2e8.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VSSEG2E8_V, MASK_VSSEG2E8_V, match_vd_neq_vm, INSN_DREF },
{"vlseg3e8.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG3E8_V, MASK_VLSEG3E8_V, match_vd_neq_vm, INSN_DREF },
{"vsseg3e8.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VSSEG3E8_V, MASK_VSSEG3E8_V, match_vd_neq_vm, INSN_DREF },
{"vlseg4e8.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG4E8_V, MASK_VLSEG4E8_V, match_vd_neq_vm, INSN_DREF },
{"vsseg4e8.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VSSEG4E8_V, MASK_VSSEG4E8_V, match_vd_neq_vm, INSN_DREF },
{"vlseg5e8.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG5E8_V, MASK_VLSEG5E8_V, match_vd_neq_vm, INSN_DREF },
{"vsseg5e8.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VSSEG5E8_V, MASK_VSSEG5E8_V, match_vd_neq_vm, INSN_DREF },
{"vlseg6e8.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG6E8_V, MASK_VLSEG6E8_V, match_vd_neq_vm, INSN_DREF },
{"vsseg6e8.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VSSEG6E8_V, MASK_VSSEG6E8_V, match_vd_neq_vm, INSN_DREF },
{"vlseg7e8.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG7E8_V, MASK_VLSEG7E8_V, match_vd_neq_vm, INSN_DREF },
{"vsseg7e8.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VSSEG7E8_V, MASK_VSSEG7E8_V, match_vd_neq_vm, INSN_DREF },
{"vlseg8e8.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG8E8_V, MASK_VLSEG8E8_V, match_vd_neq_vm, INSN_DREF },
{"vsseg8e8.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VSSEG8E8_V, MASK_VSSEG8E8_V, match_vd_neq_vm, INSN_DREF },

{"vlseg2e16.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG2E16_V, MASK_VLSEG2E16_V, match_vd_neq_vm, INSN_DREF },
{"vsseg2e16.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VSSEG2E16_V, MASK_VSSEG2E16_V, match_vd_neq_vm, INSN_DREF },
{"vlseg3e16.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG3E16_V, MASK_VLSEG3E16_V, match_vd_neq_vm, INSN_DREF },
{"vsseg3e16.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VSSEG3E16_V, MASK_VSSEG3E16_V, match_vd_neq_vm, INSN_DREF },
{"vlseg4e16.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG4E16_V, MASK_VLSEG4E16_V, match_vd_neq_vm, INSN_DREF },
{"vsseg4e16.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VSSEG4E16_V, MASK_VSSEG4E16_V, match_vd_neq_vm, INSN_DREF },
{"vlseg5e16.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG5E16_V, MASK_VLSEG5E16_V, match_vd_neq_vm, INSN_DREF },
{"vsseg5e16.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VSSEG5E16_V, MASK_VSSEG5E16_V, match_vd_neq_vm, INSN_DREF },
{"vlseg6e16.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG6E16_V, MASK_VLSEG6E16_V, match_vd_neq_vm, INSN_DREF },
{"vsseg6e16.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VSSEG6E16_V, MASK_VSSEG6E16_V, match_vd_neq_vm, INSN_DREF },
{"vlseg7e16.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG7E16_V, MASK_VLSEG7E16_V, match_vd_neq_vm, INSN_DREF },
{"vsseg7e16.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VSSEG7E16_V, MASK_VSSEG7E16_V, match_vd_neq_vm, INSN_DREF },
{"vlseg8e16.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG8E16_V, MASK_VLSEG8E16_V, match_vd_neq_vm, INSN_DREF },
{"vsseg8e16.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VSSEG8E16_V, MASK_VSSEG8E16_V, match_vd_neq_vm, INSN_DREF },

{"vlseg2e32.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG2E32_V, MASK_VLSEG2E32_V, match_vd_neq_vm, INSN_DREF },
{"vsseg2e32.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VSSEG2E32_V, MASK_VSSEG2E32_V, match_vd_neq_vm, INSN_DREF },
{"vlseg3e32.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG3E32_V, MASK_VLSEG3E32_V, match_vd_neq_vm, INSN_DREF },
{"vsseg3e32.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VSSEG3E32_V, MASK_VSSEG3E32_V, match_vd_neq_vm, INSN_DREF },
{"vlseg4e32.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG4E32_V, MASK_VLSEG4E32_V, match_vd_neq_vm, INSN_DREF },
{"vsseg4e32.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VSSEG4E32_V, MASK_VSSEG4E32_V, match_vd_neq_vm, INSN_DREF },
{"vlseg5e32.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG5E32_V, MASK_VLSEG5E32_V, match_vd_neq_vm, INSN_DREF },
{"vsseg5e32.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VSSEG5E32_V, MASK_VSSEG5E32_V, match_vd_neq_vm, INSN_DREF },
{"vlseg6e32.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG6E32_V, MASK_VLSEG6E32_V, match_vd_neq_vm, INSN_DREF },
{"vsseg6e32.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VSSEG6E32_V, MASK_VSSEG6E32_V, match_vd_neq_vm, INSN_DREF },
{"vlseg7e32.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG7E32_V, MASK_VLSEG7E32_V, match_vd_neq_vm, INSN_DREF },
{"vsseg7e32.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VSSEG7E32_V, MASK_VSSEG7E32_V, match_vd_neq_vm, INSN_DREF },
{"vlseg8e32.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG8E32_V, MASK_VLSEG8E32_V, match_vd_neq_vm, INSN_DREF },
{"vsseg8e32.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VSSEG8E32_V, MASK_VSSEG8E32_V, match_vd_neq_vm, INSN_DREF },

{"vlseg2e64.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG2E64_V, MASK_VLSEG2E64_V, match_vd_neq_vm, INSN_DREF },
{"vsseg2e64.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VSSEG2E64_V, MASK_VSSEG2E64_V, match_vd_neq_vm, INSN_DREF },
{"vlseg3e64.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG3E64_V, MASK_VLSEG3E64_V, match_vd_neq_vm, INSN_DREF },
{"vsseg3e64.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VSSEG3E64_V, MASK_VSSEG3E64_V, match_vd_neq_vm, INSN_DREF },
{"vlseg4e64.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG4E64_V, MASK_VLSEG4E64_V, match_vd_neq_vm, INSN_DREF },
{"vsseg4e64.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VSSEG4E64_V, MASK_VSSEG4E64_V, match_vd_neq_vm, INSN_DREF },
{"vlseg5e64.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG5E64_V, MASK_VLSEG5E64_V, match_vd_neq_vm, INSN_DREF },
{"vsseg5e64.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VSSEG5E64_V, MASK_VSSEG5E64_V, match_vd_neq_vm, INSN_DREF },
{"vlseg6e64.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG6E64_V, MASK_VLSEG6E64_V, match_vd_neq_vm, INSN_DREF },
{"vsseg6e64.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VSSEG6E64_V, MASK_VSSEG6E64_V, match_vd_neq_vm, INSN_DREF },
{"vlseg7e64.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG7E64_V, MASK_VLSEG7E64_V, match_vd_neq_vm, INSN_DREF },
{"vsseg7e64.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VSSEG7E64_V, MASK_VSSEG7E64_V, match_vd_neq_vm, INSN_DREF },
{"vlseg8e64.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG8E64_V, MASK_VLSEG8E64_V, match_vd_neq_vm, INSN_DREF },
{"vsseg8e64.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VSSEG8E64_V, MASK_VSSEG8E64_V, match_vd_neq_vm, INSN_DREF },

{"vlsseg2e8.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VLSSEG2E8_V, MASK_VLSSEG2E8_V, match_vd_neq_vm, INSN_DREF },
{"vssseg2e8.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VSSSEG2E8_V, MASK_VSSSEG2E8_V, match_vd_neq_vm, INSN_DREF },
{"vlsseg3e8.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VLSSEG3E8_V, MASK_VLSSEG3E8_V, match_vd_neq_vm, INSN_DREF },
{"vssseg3e8.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VSSSEG3E8_V, MASK_VSSSEG3E8_V, match_vd_neq_vm, INSN_DREF },
{"vlsseg4e8.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VLSSEG4E8_V, MASK_VLSSEG4E8_V, match_vd_neq_vm, INSN_DREF },
{"vssseg4e8.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VSSSEG4E8_V, MASK_VSSSEG4E8_V, match_vd_neq_vm, INSN_DREF },
{"vlsseg5e8.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VLSSEG5E8_V, MASK_VLSSEG5E8_V, match_vd_neq_vm, INSN_DREF },
{"vssseg5e8.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VSSSEG5E8_V, MASK_VSSSEG5E8_V, match_vd_neq_vm, INSN_DREF },
{"vlsseg6e8.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VLSSEG6E8_V, MASK_VLSSEG6E8_V, match_vd_neq_vm, INSN_DREF },
{"vssseg6e8.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VSSSEG6E8_V, MASK_VSSSEG6E8_V, match_vd_neq_vm, INSN_DREF },
{"vlsseg7e8.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VLSSEG7E8_V, MASK_VLSSEG7E8_V, match_vd_neq_vm, INSN_DREF },
{"vssseg7e8.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VSSSEG7E8_V, MASK_VSSSEG7E8_V, match_vd_neq_vm, INSN_DREF },
{"vlsseg8e8.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VLSSEG8E8_V, MASK_VLSSEG8E8_V, match_vd_neq_vm, INSN_DREF },
{"vssseg8e8.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VSSSEG8E8_V, MASK_VSSSEG8E8_V, match_vd_neq_vm, INSN_DREF },

{"vlsseg2e16.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VLSSEG2E16_V, MASK_VLSSEG2E16_V, match_vd_neq_vm, INSN_DREF },
{"vssseg2e16.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VSSSEG2E16_V, MASK_VSSSEG2E16_V, match_vd_neq_vm, INSN_DREF },
{"vlsseg3e16.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VLSSEG3E16_V, MASK_VLSSEG3E16_V, match_vd_neq_vm, INSN_DREF },
{"vssseg3e16.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VSSSEG3E16_V, MASK_VSSSEG3E16_V, match_vd_neq_vm, INSN_DREF },
{"vlsseg4e16.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VLSSEG4E16_V, MASK_VLSSEG4E16_V, match_vd_neq_vm, INSN_DREF },
{"vssseg4e16.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VSSSEG4E16_V, MASK_VSSSEG4E16_V, match_vd_neq_vm, INSN_DREF },
{"vlsseg5e16.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VLSSEG5E16_V, MASK_VLSSEG5E16_V, match_vd_neq_vm, INSN_DREF },
{"vssseg5e16.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VSSSEG5E16_V, MASK_VSSSEG5E16_V, match_vd_neq_vm, INSN_DREF },
{"vlsseg6e16.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VLSSEG6E16_V, MASK_VLSSEG6E16_V, match_vd_neq_vm, INSN_DREF },
{"vssseg6e16.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VSSSEG6E16_V, MASK_VSSSEG6E16_V, match_vd_neq_vm, INSN_DREF },
{"vlsseg7e16.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VLSSEG7E16_V, MASK_VLSSEG7E16_V, match_vd_neq_vm, INSN_DREF },
{"vssseg7e16.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VSSSEG7E16_V, MASK_VSSSEG7E16_V, match_vd_neq_vm, INSN_DREF },
{"vlsseg8e16.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VLSSEG8E16_V, MASK_VLSSEG8E16_V, match_vd_neq_vm, INSN_DREF },
{"vssseg8e16.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VSSSEG8E16_V, MASK_VSSSEG8E16_V, match_vd_neq_vm, INSN_DREF },

{"vlsseg2e32.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VLSSEG2E32_V, MASK_VLSSEG2E32_V, match_vd_neq_vm, INSN_DREF },
{"vssseg2e32.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VSSSEG2E32_V, MASK_VSSSEG2E32_V, match_vd_neq_vm, INSN_DREF },
{"vlsseg3e32.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VLSSEG3E32_V, MASK_VLSSEG3E32_V, match_vd_neq_vm, INSN_DREF },
{"vssseg3e32.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VSSSEG3E32_V, MASK_VSSSEG3E32_V, match_vd_neq_vm, INSN_DREF },
{"vlsseg4e32.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VLSSEG4E32_V, MASK_VLSSEG4E32_V, match_vd_neq_vm, INSN_DREF },
{"vssseg4e32.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VSSSEG4E32_V, MASK_VSSSEG4E32_V, match_vd_neq_vm, INSN_DREF },
{"vlsseg5e32.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VLSSEG5E32_V, MASK_VLSSEG5E32_V, match_vd_neq_vm, INSN_DREF },
{"vssseg5e32.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VSSSEG5E32_V, MASK_VSSSEG5E32_V, match_vd_neq_vm, INSN_DREF },
{"vlsseg6e32.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VLSSEG6E32_V, MASK_VLSSEG6E32_V, match_vd_neq_vm, INSN_DREF },
{"vssseg6e32.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VSSSEG6E32_V, MASK_VSSSEG6E32_V, match_vd_neq_vm, INSN_DREF },
{"vlsseg7e32.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VLSSEG7E32_V, MASK_VLSSEG7E32_V, match_vd_neq_vm, INSN_DREF },
{"vssseg7e32.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VSSSEG7E32_V, MASK_VSSSEG7E32_V, match_vd_neq_vm, INSN_DREF },
{"vlsseg8e32.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VLSSEG8E32_V, MASK_VLSSEG8E32_V, match_vd_neq_vm, INSN_DREF },
{"vssseg8e32.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VSSSEG8E32_V, MASK_VSSSEG8E32_V, match_vd_neq_vm, INSN_DREF },

{"vlsseg2e64.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VLSSEG2E64_V, MASK_VLSSEG2E64_V, match_vd_neq_vm, INSN_DREF },
{"vssseg2e64.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VSSSEG2E64_V, MASK_VSSSEG2E64_V, match_vd_neq_vm, INSN_DREF },
{"vlsseg3e64.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VLSSEG3E64_V, MASK_VLSSEG3E64_V, match_vd_neq_vm, INSN_DREF },
{"vssseg3e64.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VSSSEG3E64_V, MASK_VSSSEG3E64_V, match_vd_neq_vm, INSN_DREF },
{"vlsseg4e64.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VLSSEG4E64_V, MASK_VLSSEG4E64_V, match_vd_neq_vm, INSN_DREF },
{"vssseg4e64.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VSSSEG4E64_V, MASK_VSSSEG4E64_V, match_vd_neq_vm, INSN_DREF },
{"vlsseg5e64.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VLSSEG5E64_V, MASK_VLSSEG5E64_V, match_vd_neq_vm, INSN_DREF },
{"vssseg5e64.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VSSSEG5E64_V, MASK_VSSSEG5E64_V, match_vd_neq_vm, INSN_DREF },
{"vlsseg6e64.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VLSSEG6E64_V, MASK_VLSSEG6E64_V, match_vd_neq_vm, INSN_DREF },
{"vssseg6e64.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VSSSEG6E64_V, MASK_VSSSEG6E64_V, match_vd_neq_vm, INSN_DREF },
{"vlsseg7e64.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VLSSEG7E64_V, MASK_VLSSEG7E64_V, match_vd_neq_vm, INSN_DREF },
{"vssseg7e64.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VSSSEG7E64_V, MASK_VSSSEG7E64_V, match_vd_neq_vm, INSN_DREF },
{"vlsseg8e64.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VLSSEG8E64_V, MASK_VLSSEG8E64_V, match_vd_neq_vm, INSN_DREF },
{"vssseg8e64.v",  0, {"V", 0},  "Vd,0(s),tVm", MATCH_VSSSEG8E64_V, MASK_VSSSEG8E64_V, match_vd_neq_vm, INSN_DREF },

{"vloxseg2ei8.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLOXSEG2EI8_V, MASK_VLOXSEG2EI8_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsoxseg2ei8.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSOXSEG2EI8_V, MASK_VSOXSEG2EI8_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vloxseg3ei8.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLOXSEG3EI8_V, MASK_VLOXSEG3EI8_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsoxseg3ei8.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSOXSEG3EI8_V, MASK_VSOXSEG3EI8_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vloxseg4ei8.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLOXSEG4EI8_V, MASK_VLOXSEG4EI8_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsoxseg4ei8.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSOXSEG4EI8_V, MASK_VSOXSEG4EI8_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vloxseg5ei8.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLOXSEG5EI8_V, MASK_VLOXSEG5EI8_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsoxseg5ei8.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSOXSEG5EI8_V, MASK_VSOXSEG5EI8_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vloxseg6ei8.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLOXSEG6EI8_V, MASK_VLOXSEG6EI8_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsoxseg6ei8.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSOXSEG6EI8_V, MASK_VSOXSEG6EI8_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vloxseg7ei8.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLOXSEG7EI8_V, MASK_VLOXSEG7EI8_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsoxseg7ei8.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSOXSEG7EI8_V, MASK_VSOXSEG7EI8_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vloxseg8ei8.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLOXSEG8EI8_V, MASK_VLOXSEG8EI8_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsoxseg8ei8.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSOXSEG8EI8_V, MASK_VSOXSEG8EI8_V, match_vd_neq_vs2_neq_vm, INSN_DREF },

{"vloxseg2ei16.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLOXSEG2EI16_V, MASK_VLOXSEG2EI16_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsoxseg2ei16.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSOXSEG2EI16_V, MASK_VSOXSEG2EI16_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vloxseg3ei16.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLOXSEG3EI16_V, MASK_VLOXSEG3EI16_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsoxseg3ei16.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSOXSEG3EI16_V, MASK_VSOXSEG3EI16_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vloxseg4ei16.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLOXSEG4EI16_V, MASK_VLOXSEG4EI16_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsoxseg4ei16.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSOXSEG4EI16_V, MASK_VSOXSEG4EI16_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vloxseg5ei16.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLOXSEG5EI16_V, MASK_VLOXSEG5EI16_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsoxseg5ei16.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSOXSEG5EI16_V, MASK_VSOXSEG5EI16_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vloxseg6ei16.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLOXSEG6EI16_V, MASK_VLOXSEG6EI16_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsoxseg6ei16.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSOXSEG6EI16_V, MASK_VSOXSEG6EI16_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vloxseg7ei16.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLOXSEG7EI16_V, MASK_VLOXSEG7EI16_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsoxseg7ei16.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSOXSEG7EI16_V, MASK_VSOXSEG7EI16_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vloxseg8ei16.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLOXSEG8EI16_V, MASK_VLOXSEG8EI16_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsoxseg8ei16.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSOXSEG8EI16_V, MASK_VSOXSEG8EI16_V, match_vd_neq_vs2_neq_vm, INSN_DREF },

{"vloxseg2ei32.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLOXSEG2EI32_V, MASK_VLOXSEG2EI32_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsoxseg2ei32.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSOXSEG2EI32_V, MASK_VSOXSEG2EI32_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vloxseg3ei32.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLOXSEG3EI32_V, MASK_VLOXSEG3EI32_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsoxseg3ei32.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSOXSEG3EI32_V, MASK_VSOXSEG3EI32_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vloxseg4ei32.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLOXSEG4EI32_V, MASK_VLOXSEG4EI32_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsoxseg4ei32.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSOXSEG4EI32_V, MASK_VSOXSEG4EI32_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vloxseg5ei32.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLOXSEG5EI32_V, MASK_VLOXSEG5EI32_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsoxseg5ei32.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSOXSEG5EI32_V, MASK_VSOXSEG5EI32_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vloxseg6ei32.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLOXSEG6EI32_V, MASK_VLOXSEG6EI32_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsoxseg6ei32.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSOXSEG6EI32_V, MASK_VSOXSEG6EI32_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vloxseg7ei32.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLOXSEG7EI32_V, MASK_VLOXSEG7EI32_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsoxseg7ei32.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSOXSEG7EI32_V, MASK_VSOXSEG7EI32_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vloxseg8ei32.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLOXSEG8EI32_V, MASK_VLOXSEG8EI32_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsoxseg8ei32.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSOXSEG8EI32_V, MASK_VSOXSEG8EI32_V, match_vd_neq_vs2_neq_vm, INSN_DREF },

{"vloxseg2ei64.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLOXSEG2EI64_V, MASK_VLOXSEG2EI64_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsoxseg2ei64.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSOXSEG2EI64_V, MASK_VSOXSEG2EI64_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vloxseg3ei64.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLOXSEG3EI64_V, MASK_VLOXSEG3EI64_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsoxseg3ei64.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSOXSEG3EI64_V, MASK_VSOXSEG3EI64_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vloxseg4ei64.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLOXSEG4EI64_V, MASK_VLOXSEG4EI64_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsoxseg4ei64.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSOXSEG4EI64_V, MASK_VSOXSEG4EI64_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vloxseg5ei64.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLOXSEG5EI64_V, MASK_VLOXSEG5EI64_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsoxseg5ei64.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSOXSEG5EI64_V, MASK_VSOXSEG5EI64_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vloxseg6ei64.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLOXSEG6EI64_V, MASK_VLOXSEG6EI64_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsoxseg6ei64.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSOXSEG6EI64_V, MASK_VSOXSEG6EI64_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vloxseg7ei64.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLOXSEG7EI64_V, MASK_VLOXSEG7EI64_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsoxseg7ei64.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSOXSEG7EI64_V, MASK_VSOXSEG7EI64_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vloxseg8ei64.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLOXSEG8EI64_V, MASK_VLOXSEG8EI64_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsoxseg8ei64.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSOXSEG8EI64_V, MASK_VSOXSEG8EI64_V, match_vd_neq_vs2_neq_vm, INSN_DREF },

{"vluxseg2ei8.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLUXSEG2EI8_V, MASK_VLUXSEG2EI8_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsuxseg2ei8.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSUXSEG2EI8_V, MASK_VSUXSEG2EI8_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vluxseg3ei8.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLUXSEG3EI8_V, MASK_VLUXSEG3EI8_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsuxseg3ei8.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSUXSEG3EI8_V, MASK_VSUXSEG3EI8_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vluxseg4ei8.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLUXSEG4EI8_V, MASK_VLUXSEG4EI8_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsuxseg4ei8.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSUXSEG4EI8_V, MASK_VSUXSEG4EI8_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vluxseg5ei8.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLUXSEG5EI8_V, MASK_VLUXSEG5EI8_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsuxseg5ei8.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSUXSEG5EI8_V, MASK_VSUXSEG5EI8_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vluxseg6ei8.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLUXSEG6EI8_V, MASK_VLUXSEG6EI8_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsuxseg6ei8.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSUXSEG6EI8_V, MASK_VSUXSEG6EI8_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vluxseg7ei8.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLUXSEG7EI8_V, MASK_VLUXSEG7EI8_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsuxseg7ei8.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSUXSEG7EI8_V, MASK_VSUXSEG7EI8_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vluxseg8ei8.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLUXSEG8EI8_V, MASK_VLUXSEG8EI8_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsuxseg8ei8.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSUXSEG8EI8_V, MASK_VSUXSEG8EI8_V, match_vd_neq_vs2_neq_vm, INSN_DREF },

{"vluxseg2ei16.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLUXSEG2EI16_V, MASK_VLUXSEG2EI16_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsuxseg2ei16.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSUXSEG2EI16_V, MASK_VSUXSEG2EI16_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vluxseg3ei16.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLUXSEG3EI16_V, MASK_VLUXSEG3EI16_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsuxseg3ei16.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSUXSEG3EI16_V, MASK_VSUXSEG3EI16_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vluxseg4ei16.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLUXSEG4EI16_V, MASK_VLUXSEG4EI16_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsuxseg4ei16.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSUXSEG4EI16_V, MASK_VSUXSEG4EI16_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vluxseg5ei16.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLUXSEG5EI16_V, MASK_VLUXSEG5EI16_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsuxseg5ei16.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSUXSEG5EI16_V, MASK_VSUXSEG5EI16_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vluxseg6ei16.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLUXSEG6EI16_V, MASK_VLUXSEG6EI16_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsuxseg6ei16.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSUXSEG6EI16_V, MASK_VSUXSEG6EI16_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vluxseg7ei16.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLUXSEG7EI16_V, MASK_VLUXSEG7EI16_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsuxseg7ei16.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSUXSEG7EI16_V, MASK_VSUXSEG7EI16_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vluxseg8ei16.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLUXSEG8EI16_V, MASK_VLUXSEG8EI16_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsuxseg8ei16.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSUXSEG8EI16_V, MASK_VSUXSEG8EI16_V, match_vd_neq_vs2_neq_vm, INSN_DREF },

{"vluxseg2ei32.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLUXSEG2EI32_V, MASK_VLUXSEG2EI32_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsuxseg2ei32.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSUXSEG2EI32_V, MASK_VSUXSEG2EI32_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vluxseg3ei32.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLUXSEG3EI32_V, MASK_VLUXSEG3EI32_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsuxseg3ei32.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSUXSEG3EI32_V, MASK_VSUXSEG3EI32_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vluxseg4ei32.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLUXSEG4EI32_V, MASK_VLUXSEG4EI32_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsuxseg4ei32.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSUXSEG4EI32_V, MASK_VSUXSEG4EI32_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vluxseg5ei32.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLUXSEG5EI32_V, MASK_VLUXSEG5EI32_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsuxseg5ei32.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSUXSEG5EI32_V, MASK_VSUXSEG5EI32_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vluxseg6ei32.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLUXSEG6EI32_V, MASK_VLUXSEG6EI32_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsuxseg6ei32.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSUXSEG6EI32_V, MASK_VSUXSEG6EI32_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vluxseg7ei32.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLUXSEG7EI32_V, MASK_VLUXSEG7EI32_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsuxseg7ei32.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSUXSEG7EI32_V, MASK_VSUXSEG7EI32_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vluxseg8ei32.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLUXSEG8EI32_V, MASK_VLUXSEG8EI32_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsuxseg8ei32.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSUXSEG8EI32_V, MASK_VSUXSEG8EI32_V, match_vd_neq_vs2_neq_vm, INSN_DREF },

{"vluxseg2ei64.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLUXSEG2EI64_V, MASK_VLUXSEG2EI64_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsuxseg2ei64.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSUXSEG2EI64_V, MASK_VSUXSEG2EI64_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vluxseg3ei64.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLUXSEG3EI64_V, MASK_VLUXSEG3EI64_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsuxseg3ei64.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSUXSEG3EI64_V, MASK_VSUXSEG3EI64_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vluxseg4ei64.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLUXSEG4EI64_V, MASK_VLUXSEG4EI64_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsuxseg4ei64.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSUXSEG4EI64_V, MASK_VSUXSEG4EI64_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vluxseg5ei64.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLUXSEG5EI64_V, MASK_VLUXSEG5EI64_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsuxseg5ei64.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSUXSEG5EI64_V, MASK_VSUXSEG5EI64_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vluxseg6ei64.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLUXSEG6EI64_V, MASK_VLUXSEG6EI64_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsuxseg6ei64.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSUXSEG6EI64_V, MASK_VSUXSEG6EI64_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vluxseg7ei64.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLUXSEG7EI64_V, MASK_VLUXSEG7EI64_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsuxseg7ei64.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSUXSEG7EI64_V, MASK_VSUXSEG7EI64_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vluxseg8ei64.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VLUXSEG8EI64_V, MASK_VLUXSEG8EI64_V, match_vd_neq_vs2_neq_vm, INSN_DREF },
{"vsuxseg8ei64.v",  0, {"V", 0},  "Vd,0(s),VtVm", MATCH_VSUXSEG8EI64_V, MASK_VSUXSEG8EI64_V, match_vd_neq_vs2_neq_vm, INSN_DREF },

{"vlseg2e8ff.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG2E8FF_V, MASK_VLSEG2E8FF_V, match_vd_neq_vm, INSN_DREF },
{"vlseg3e8ff.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG3E8FF_V, MASK_VLSEG3E8FF_V, match_vd_neq_vm, INSN_DREF },
{"vlseg4e8ff.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG4E8FF_V, MASK_VLSEG4E8FF_V, match_vd_neq_vm, INSN_DREF },
{"vlseg5e8ff.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG5E8FF_V, MASK_VLSEG5E8FF_V, match_vd_neq_vm, INSN_DREF },
{"vlseg6e8ff.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG6E8FF_V, MASK_VLSEG6E8FF_V, match_vd_neq_vm, INSN_DREF },
{"vlseg7e8ff.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG7E8FF_V, MASK_VLSEG7E8FF_V, match_vd_neq_vm, INSN_DREF },
{"vlseg8e8ff.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG8E8FF_V, MASK_VLSEG8E8FF_V, match_vd_neq_vm, INSN_DREF },

{"vlseg2e16ff.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG2E16FF_V, MASK_VLSEG2E16FF_V, match_vd_neq_vm, INSN_DREF },
{"vlseg3e16ff.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG3E16FF_V, MASK_VLSEG3E16FF_V, match_vd_neq_vm, INSN_DREF },
{"vlseg4e16ff.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG4E16FF_V, MASK_VLSEG4E16FF_V, match_vd_neq_vm, INSN_DREF },
{"vlseg5e16ff.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG5E16FF_V, MASK_VLSEG5E16FF_V, match_vd_neq_vm, INSN_DREF },
{"vlseg6e16ff.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG6E16FF_V, MASK_VLSEG6E16FF_V, match_vd_neq_vm, INSN_DREF },
{"vlseg7e16ff.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG7E16FF_V, MASK_VLSEG7E16FF_V, match_vd_neq_vm, INSN_DREF },
{"vlseg8e16ff.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG8E16FF_V, MASK_VLSEG8E16FF_V, match_vd_neq_vm, INSN_DREF },

{"vlseg2e32ff.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG2E32FF_V, MASK_VLSEG2E32FF_V, match_vd_neq_vm, INSN_DREF },
{"vlseg3e32ff.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG3E32FF_V, MASK_VLSEG3E32FF_V, match_vd_neq_vm, INSN_DREF },
{"vlseg4e32ff.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG4E32FF_V, MASK_VLSEG4E32FF_V, match_vd_neq_vm, INSN_DREF },
{"vlseg5e32ff.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG5E32FF_V, MASK_VLSEG5E32FF_V, match_vd_neq_vm, INSN_DREF },
{"vlseg6e32ff.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG6E32FF_V, MASK_VLSEG6E32FF_V, match_vd_neq_vm, INSN_DREF },
{"vlseg7e32ff.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG7E32FF_V, MASK_VLSEG7E32FF_V, match_vd_neq_vm, INSN_DREF },
{"vlseg8e32ff.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG8E32FF_V, MASK_VLSEG8E32FF_V, match_vd_neq_vm, INSN_DREF },

{"vlseg2e64ff.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG2E64FF_V, MASK_VLSEG2E64FF_V, match_vd_neq_vm, INSN_DREF },
{"vlseg3e64ff.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG3E64FF_V, MASK_VLSEG3E64FF_V, match_vd_neq_vm, INSN_DREF },
{"vlseg4e64ff.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG4E64FF_V, MASK_VLSEG4E64FF_V, match_vd_neq_vm, INSN_DREF },
{"vlseg5e64ff.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG5E64FF_V, MASK_VLSEG5E64FF_V, match_vd_neq_vm, INSN_DREF },
{"vlseg6e64ff.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG6E64FF_V, MASK_VLSEG6E64FF_V, match_vd_neq_vm, INSN_DREF },
{"vlseg7e64ff.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG7E64FF_V, MASK_VLSEG7E64FF_V, match_vd_neq_vm, INSN_DREF },
{"vlseg8e64ff.v",  0, {"V", 0},  "Vd,0(s)Vm", MATCH_VLSEG8E64FF_V, MASK_VLSEG8E64FF_V, match_vd_neq_vm, INSN_DREF },

{"vl1r.v",      0, {"V", 0},  "Vd,0(s)", MATCH_VL1RE8_V,  MASK_VL1RE8_V,  match_vls_nf_rv, INSN_DREF|INSN_ALIAS },
{"vl1re8.v",    0, {"V", 0},  "Vd,0(s)", MATCH_VL1RE8_V,  MASK_VL1RE8_V,  match_vls_nf_rv, INSN_DREF },
{"vl1re16.v",   0, {"V", 0},  "Vd,0(s)", MATCH_VL1RE16_V, MASK_VL1RE16_V, match_vls_nf_rv, INSN_DREF },
{"vl1re32.v",   0, {"V", 0},  "Vd,0(s)", MATCH_VL1RE32_V, MASK_VL1RE32_V, match_vls_nf_rv, INSN_DREF },
{"vl1re64.v",   0, {"V", 0},  "Vd,0(s)", MATCH_VL1RE64_V, MASK_VL1RE64_V, match_vls_nf_rv, INSN_DREF },

{"vl2r.v",      0, {"V", 0},  "Vd,0(s)", MATCH_VL2RE8_V,  MASK_VL2RE8_V,  match_vls_nf_rv, INSN_DREF|INSN_ALIAS },
{"vl2re8.v",    0, {"V", 0},  "Vd,0(s)", MATCH_VL2RE8_V,  MASK_VL2RE8_V,  match_vls_nf_rv, INSN_DREF },
{"vl2re16.v",   0, {"V", 0},  "Vd,0(s)", MATCH_VL2RE16_V, MASK_VL2RE16_V, match_vls_nf_rv, INSN_DREF },
{"vl2re32.v",   0, {"V", 0},  "Vd,0(s)", MATCH_VL2RE32_V, MASK_VL2RE32_V, match_vls_nf_rv, INSN_DREF },
{"vl2re64.v",   0, {"V", 0},  "Vd,0(s)", MATCH_VL2RE64_V, MASK_VL2RE64_V, match_vls_nf_rv, INSN_DREF },

{"vl4r.v",      0, {"V", 0},  "Vd,0(s)", MATCH_VL4RE8_V,  MASK_VL4RE8_V,  match_vls_nf_rv, INSN_DREF|INSN_ALIAS },
{"vl4re8.v",    0, {"V", 0},  "Vd,0(s)", MATCH_VL4RE8_V,  MASK_VL4RE8_V,  match_vls_nf_rv, INSN_DREF },
{"vl4re16.v",   0, {"V", 0},  "Vd,0(s)", MATCH_VL4RE16_V, MASK_VL4RE16_V, match_vls_nf_rv, INSN_DREF },
{"vl4re32.v",   0, {"V", 0},  "Vd,0(s)", MATCH_VL4RE32_V, MASK_VL4RE32_V, match_vls_nf_rv, INSN_DREF },
{"vl4re64.v",   0, {"V", 0},  "Vd,0(s)", MATCH_VL4RE64_V, MASK_VL4RE64_V, match_vls_nf_rv, INSN_DREF },

{"vl8r.v",      0, {"V", 0},  "Vd,0(s)", MATCH_VL8RE8_V,  MASK_VL8RE8_V,  match_vls_nf_rv, INSN_DREF|INSN_ALIAS },
{"vl8re8.v",    0, {"V", 0},  "Vd,0(s)", MATCH_VL8RE8_V,  MASK_VL8RE8_V,  match_vls_nf_rv, INSN_DREF },
{"vl8re16.v",   0, {"V", 0},  "Vd,0(s)", MATCH_VL8RE16_V, MASK_VL8RE16_V, match_vls_nf_rv, INSN_DREF },
{"vl8re32.v",   0, {"V", 0},  "Vd,0(s)", MATCH_VL8RE32_V, MASK_VL8RE32_V, match_vls_nf_rv, INSN_DREF },
{"vl8re64.v",   0, {"V", 0},  "Vd,0(s)", MATCH_VL8RE64_V, MASK_VL8RE64_V, match_vls_nf_rv, INSN_DREF },

{"vs1r.v",  0, {"V", 0},  "Vd,0(s)", MATCH_VS1R_V, MASK_VS1R_V, match_vls_nf_rv, INSN_DREF },
{"vs2r.v",  0, {"V", 0},  "Vd,0(s)", MATCH_VS2R_V, MASK_VS2R_V, match_vls_nf_rv, INSN_DREF },
{"vs4r.v",  0, {"V", 0},  "Vd,0(s)", MATCH_VS4R_V, MASK_VS4R_V, match_vls_nf_rv, INSN_DREF },
{"vs8r.v",  0, {"V", 0},  "Vd,0(s)", MATCH_VS8R_V, MASK_VS8R_V, match_vls_nf_rv, INSN_DREF },

{"vamoaddei8.v",   0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOADDEI8_V, MASK_VAMOADDEI8_V, match_vd_neq_vm, INSN_DREF},
{"vamoswapei8.v",  0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOSWAPEI8_V, MASK_VAMOSWAPEI8_V, match_vd_neq_vm, INSN_DREF},
{"vamoxorei8.v",   0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOXOREI8_V, MASK_VAMOXOREI8_V, match_vd_neq_vm, INSN_DREF},
{"vamoandei8.v",   0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOANDEI8_V, MASK_VAMOANDEI8_V, match_vd_neq_vm, INSN_DREF},
{"vamoorei8.v",    0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOOREI8_V, MASK_VAMOOREI8_V, match_vd_neq_vm, INSN_DREF},
{"vamominei8.v",   0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOMINEI8_V, MASK_VAMOMINEI8_V, match_vd_neq_vm, INSN_DREF},
{"vamomaxei8.v",   0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOMAXEI8_V, MASK_VAMOMAXEI8_V, match_vd_neq_vm, INSN_DREF},
{"vamominuei8.v",  0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOMINUEI8_V, MASK_VAMOMINUEI8_V, match_vd_neq_vm, INSN_DREF},
{"vamomaxuei8.v",  0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOMAXUEI8_V, MASK_VAMOMAXUEI8_V, match_vd_neq_vm, INSN_DREF},

{"vamoaddei16.v",   0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOADDEI16_V, MASK_VAMOADDEI16_V, match_vd_neq_vm, INSN_DREF},
{"vamoswapei16.v",  0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOSWAPEI16_V, MASK_VAMOSWAPEI16_V, match_vd_neq_vm, INSN_DREF},
{"vamoxorei16.v",   0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOXOREI16_V, MASK_VAMOXOREI16_V, match_vd_neq_vm, INSN_DREF},
{"vamoandei16.v",   0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOANDEI16_V, MASK_VAMOANDEI16_V, match_vd_neq_vm, INSN_DREF},
{"vamoorei16.v",    0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOOREI16_V, MASK_VAMOOREI16_V, match_vd_neq_vm, INSN_DREF},
{"vamominei16.v",   0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOMINEI16_V, MASK_VAMOMINEI16_V, match_vd_neq_vm, INSN_DREF},
{"vamomaxei16.v",   0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOMAXEI16_V, MASK_VAMOMAXEI16_V, match_vd_neq_vm, INSN_DREF},
{"vamominuei16.v",  0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOMINUEI16_V, MASK_VAMOMINUEI16_V, match_vd_neq_vm, INSN_DREF},
{"vamomaxuei16.v",  0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOMAXUEI16_V, MASK_VAMOMAXUEI16_V, match_vd_neq_vm, INSN_DREF},

{"vamoaddei32.v",   0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOADDEI32_V, MASK_VAMOADDEI32_V, match_vd_neq_vm, INSN_DREF},
{"vamoswapei32.v",  0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOSWAPEI32_V, MASK_VAMOSWAPEI32_V, match_vd_neq_vm, INSN_DREF},
{"vamoxorei32.v",   0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOXOREI32_V, MASK_VAMOXOREI32_V, match_vd_neq_vm, INSN_DREF},
{"vamoandei32.v",   0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOANDEI32_V, MASK_VAMOANDEI32_V, match_vd_neq_vm, INSN_DREF},
{"vamoorei32.v",    0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOOREI32_V, MASK_VAMOOREI32_V, match_vd_neq_vm, INSN_DREF},
{"vamominei32.v",   0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOMINEI32_V, MASK_VAMOMINEI32_V, match_vd_neq_vm, INSN_DREF},
{"vamomaxei32.v",   0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOMAXEI32_V, MASK_VAMOMAXEI32_V, match_vd_neq_vm, INSN_DREF},
{"vamominuei32.v",  0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOMINUEI32_V, MASK_VAMOMINUEI32_V, match_vd_neq_vm, INSN_DREF},
{"vamomaxuei32.v",  0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOMAXUEI32_V, MASK_VAMOMAXUEI32_V, match_vd_neq_vm, INSN_DREF},

{"vamoaddei64.v",   0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOADDEI64_V, MASK_VAMOADDEI64_V, match_vd_neq_vm, INSN_DREF},
{"vamoswapei64.v",  0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOSWAPEI64_V, MASK_VAMOSWAPEI64_V, match_vd_neq_vm, INSN_DREF},
{"vamoxorei64.v",   0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOXOREI64_V, MASK_VAMOXOREI64_V, match_vd_neq_vm, INSN_DREF},
{"vamoandei64.v",   0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOANDEI64_V, MASK_VAMOANDEI64_V, match_vd_neq_vm, INSN_DREF},
{"vamoorei64.v",    0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOOREI64_V, MASK_VAMOOREI64_V, match_vd_neq_vm, INSN_DREF},
{"vamominei64.v",   0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOMINEI64_V, MASK_VAMOMINEI64_V, match_vd_neq_vm, INSN_DREF},
{"vamomaxei64.v",   0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOMAXEI64_V, MASK_VAMOMAXEI64_V, match_vd_neq_vm, INSN_DREF},
{"vamominuei64.v",  0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOMINUEI64_V, MASK_VAMOMINUEI64_V, match_vd_neq_vm, INSN_DREF},
{"vamomaxuei64.v",  0, {"V", 0},  "Ve,0(s),Vt,VfVm", MATCH_VAMOMAXUEI64_V, MASK_VAMOMAXUEI64_V, match_vd_neq_vm, INSN_DREF},

{"vneg.v",     0, {"V", 0},  "Vd,VtVm",  MATCH_VRSUB_VX, MASK_VRSUB_VX | MASK_RS1, match_vd_neq_vm, INSN_ALIAS },

{"vadd.vv",    0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VADD_VV,  MASK_VADD_VV, match_vd_neq_vm, 0 },
{"vadd.vx",    0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VADD_VX,  MASK_VADD_VX, match_vd_neq_vm, 0 },
{"vadd.vi",    0, {"V", 0},  "Vd,Vt,ViVm", MATCH_VADD_VI,  MASK_VADD_VI, match_vd_neq_vm, 0 },
{"vsub.vv",    0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VSUB_VV,  MASK_VSUB_VV, match_vd_neq_vm, 0 },
{"vsub.vx",    0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VSUB_VX,  MASK_VSUB_VX, match_vd_neq_vm, 0 },
{"vrsub.vx",   0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VRSUB_VX, MASK_VRSUB_VX, match_vd_neq_vm, 0 },
{"vrsub.vi",   0, {"V", 0},  "Vd,Vt,ViVm", MATCH_VRSUB_VI, MASK_VRSUB_VI, match_vd_neq_vm, 0 },

{"vwcvt.x.x.v",  0, {"V", 0},  "Vd,VtVm", MATCH_VWCVT_X_X_V,  MASK_VWCVT_X_X_V,  match_widen_vd_neq_vs2_neq_vm, INSN_ALIAS },
{"vwcvtu.x.x.v", 0, {"V", 0},  "Vd,VtVm", MATCH_VWCVTU_X_X_V, MASK_VWCVTU_X_X_V, match_widen_vd_neq_vs2_neq_vm, INSN_ALIAS },

{"vwaddu.vv",  0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VWADDU_VV, MASK_VWADDU_VV, match_widen_vd_neq_vs1_neq_vs2_neq_vm, 0 },
{"vwaddu.vx",  0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VWADDU_VX, MASK_VWADDU_VX, match_widen_vd_neq_vs2_neq_vm, 0 },
{"vwsubu.vv",  0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VWSUBU_VV, MASK_VWSUBU_VV, match_widen_vd_neq_vs1_neq_vs2_neq_vm, 0 },
{"vwsubu.vx",  0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VWSUBU_VX, MASK_VWSUBU_VX, match_widen_vd_neq_vs2_neq_vm, 0 },
{"vwadd.vv",   0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VWADD_VV, MASK_VWADD_VV, match_widen_vd_neq_vs1_neq_vs2_neq_vm, 0 },
{"vwadd.vx",   0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VWADD_VX, MASK_VWADD_VX, match_widen_vd_neq_vs2_neq_vm, 0 },
{"vwsub.vv",   0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VWSUB_VV, MASK_VWSUB_VV, match_widen_vd_neq_vs1_neq_vs2_neq_vm, 0 },
{"vwsub.vx",   0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VWSUB_VX, MASK_VWSUB_VX, match_widen_vd_neq_vs2_neq_vm, 0 },
{"vwaddu.wv",  0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VWADDU_WV, MASK_VWADDU_WV, match_widen_vd_neq_vs1_neq_vm, 0 },
{"vwaddu.wx",  0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VWADDU_WX, MASK_VWADDU_WX, match_widen_vd_neq_vm, 0 },
{"vwsubu.wv",  0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VWSUBU_WV, MASK_VWSUBU_WV, match_widen_vd_neq_vs1_neq_vm, 0 },
{"vwsubu.wx",  0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VWSUBU_WX, MASK_VWSUBU_WX, match_widen_vd_neq_vm, 0 },
{"vwadd.wv",   0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VWADD_WV, MASK_VWADD_WV, match_widen_vd_neq_vs1_neq_vm, 0 },
{"vwadd.wx",   0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VWADD_WX, MASK_VWADD_WX, match_widen_vd_neq_vm, 0 },
{"vwsub.wv",   0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VWSUB_WV, MASK_VWSUB_WV, match_widen_vd_neq_vs1_neq_vm, 0 },
{"vwsub.wx",   0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VWSUB_WX, MASK_VWSUB_WX, match_widen_vd_neq_vm, 0 },

{"vzext.vf2",  0, {"V", 0},  "Vd,VtVm", MATCH_VZEXT_VF2, MASK_VZEXT_VF2, match_vd_neq_vm, 0 },
{"vsext.vf2",  0, {"V", 0},  "Vd,VtVm", MATCH_VSEXT_VF2, MASK_VSEXT_VF2, match_vd_neq_vm, 0 },
{"vzext.vf4",  0, {"V", 0},  "Vd,VtVm", MATCH_VZEXT_VF4, MASK_VZEXT_VF4, match_vd_neq_vm, 0 },
{"vsext.vf4",  0, {"V", 0},  "Vd,VtVm", MATCH_VSEXT_VF4, MASK_VSEXT_VF4, match_vd_neq_vm, 0 },
{"vzext.vf8",  0, {"V", 0},  "Vd,VtVm", MATCH_VZEXT_VF8, MASK_VZEXT_VF8, match_vd_neq_vm, 0 },
{"vsext.vf8",  0, {"V", 0},  "Vd,VtVm", MATCH_VSEXT_VF8, MASK_VSEXT_VF8, match_vd_neq_vm, 0 },

{"vadc.vvm",   0, {"V", 0},  "Vd,Vt,Vs,V0", MATCH_VADC_VVM,  MASK_VADC_VVM,  match_vd_neq_vm, 0 },
{"vadc.vxm",   0, {"V", 0},  "Vd,Vt,s,V0",  MATCH_VADC_VXM,  MASK_VADC_VXM,  match_vd_neq_vm, 0 },
{"vadc.vim",   0, {"V", 0},  "Vd,Vt,Vi,V0", MATCH_VADC_VIM,  MASK_VADC_VIM,  match_vd_neq_vm, 0 },
{"vmadc.vvm",  0, {"V", 0},  "Vd,Vt,Vs,V0", MATCH_VMADC_VVM, MASK_VMADC_VVM, match_opcode, 0 },
{"vmadc.vxm",  0, {"V", 0},  "Vd,Vt,s,V0",  MATCH_VMADC_VXM, MASK_VMADC_VXM, match_opcode, 0 },
{"vmadc.vim",  0, {"V", 0},  "Vd,Vt,Vi,V0", MATCH_VMADC_VIM, MASK_VMADC_VIM, match_opcode, 0 },
{"vmadc.vv",   0, {"V", 0},  "Vd,Vt,Vs",    MATCH_VMADC_VV,  MASK_VMADC_VV,  match_opcode, 0 },
{"vmadc.vx",   0, {"V", 0},  "Vd,Vt,s",     MATCH_VMADC_VX,  MASK_VMADC_VX,  match_opcode, 0 },
{"vmadc.vi",   0, {"V", 0},  "Vd,Vt,Vi",    MATCH_VMADC_VI,  MASK_VMADC_VI,  match_opcode, 0 },
{"vsbc.vvm",   0, {"V", 0},  "Vd,Vt,Vs,V0", MATCH_VSBC_VVM,  MASK_VSBC_VVM,  match_vd_neq_vm, 0 },
{"vsbc.vxm",   0, {"V", 0},  "Vd,Vt,s,V0",  MATCH_VSBC_VXM,  MASK_VSBC_VXM,  match_vd_neq_vm, 0 },
{"vmsbc.vvm",  0, {"V", 0},  "Vd,Vt,Vs,V0", MATCH_VMSBC_VVM, MASK_VMSBC_VVM, match_opcode, 0 },
{"vmsbc.vxm",  0, {"V", 0},  "Vd,Vt,s,V0",  MATCH_VMSBC_VXM, MASK_VMSBC_VXM, match_opcode, 0 },
{"vmsbc.vv",   0, {"V", 0},  "Vd,Vt,Vs",    MATCH_VMSBC_VV,  MASK_VMSBC_VV,  match_opcode, 0 },
{"vmsbc.vx",   0, {"V", 0},  "Vd,Vt,s",     MATCH_VMSBC_VX,  MASK_VMSBC_VX,  match_opcode, 0 },

{"vnot.v",     0, {"V", 0},  "Vd,VtVm", MATCH_VNOT_V, MASK_VNOT_V, match_vd_neq_vm, INSN_ALIAS },

{"vand.vv",    0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VAND_VV, MASK_VAND_VV, match_vd_neq_vm, 0 },
{"vand.vx",    0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VAND_VX, MASK_VAND_VX, match_vd_neq_vm, 0 },
{"vand.vi",    0, {"V", 0},  "Vd,Vt,ViVm", MATCH_VAND_VI, MASK_VAND_VI, match_vd_neq_vm, 0 },
{"vor.vv",     0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VOR_VV,  MASK_VOR_VV,  match_vd_neq_vm, 0 },
{"vor.vx",     0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VOR_VX,  MASK_VOR_VX,  match_vd_neq_vm, 0 },
{"vor.vi",     0, {"V", 0},  "Vd,Vt,ViVm", MATCH_VOR_VI,  MASK_VOR_VI,  match_vd_neq_vm, 0 },
{"vxor.vv",    0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VXOR_VV, MASK_VXOR_VV, match_vd_neq_vm, 0 },
{"vxor.vx",    0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VXOR_VX, MASK_VXOR_VX, match_vd_neq_vm, 0 },
{"vxor.vi",    0, {"V", 0},  "Vd,Vt,ViVm", MATCH_VXOR_VI, MASK_VXOR_VI, match_vd_neq_vm, 0 },

{"vsll.vv",    0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VSLL_VV, MASK_VSLL_VV, match_vd_neq_vm, 0 },
{"vsll.vx",    0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VSLL_VX, MASK_VSLL_VX, match_vd_neq_vm, 0 },
{"vsll.vi",    0, {"V", 0},  "Vd,Vt,VjVm", MATCH_VSLL_VI, MASK_VSLL_VI, match_vd_neq_vm, 0 },
{"vsrl.vv",    0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VSRL_VV, MASK_VSRL_VV, match_vd_neq_vm, 0 },
{"vsrl.vx",    0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VSRL_VX, MASK_VSRL_VX, match_vd_neq_vm, 0 },
{"vsrl.vi",    0, {"V", 0},  "Vd,Vt,VjVm", MATCH_VSRL_VI, MASK_VSRL_VI, match_vd_neq_vm, 0 },
{"vsra.vv",    0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VSRA_VV, MASK_VSRA_VV, match_vd_neq_vm, 0 },
{"vsra.vx",    0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VSRA_VX, MASK_VSRA_VX, match_vd_neq_vm, 0 },
{"vsra.vi",    0, {"V", 0},  "Vd,Vt,VjVm", MATCH_VSRA_VI, MASK_VSRA_VI, match_vd_neq_vm, 0 },

{"vncvt.x.x.w",0, {"V", 0},  "Vd,VtVm", MATCH_VNCVT_X_X_W, MASK_VNCVT_X_X_W, match_narrow_vd_neq_vs2_neq_vm, INSN_ALIAS },

{"vnsrl.wv",   0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VNSRL_WV, MASK_VNSRL_WV, match_narrow_vd_neq_vs2_neq_vm, 0 },
{"vnsrl.wx",   0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VNSRL_WX, MASK_VNSRL_WX, match_narrow_vd_neq_vs2_neq_vm, 0 },
{"vnsrl.wi",   0, {"V", 0},  "Vd,Vt,VjVm", MATCH_VNSRL_WI, MASK_VNSRL_WI, match_narrow_vd_neq_vs2_neq_vm, 0 },
{"vnsra.wv",   0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VNSRA_WV, MASK_VNSRA_WV, match_narrow_vd_neq_vs2_neq_vm, 0 },
{"vnsra.wx",   0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VNSRA_WX, MASK_VNSRA_WX, match_narrow_vd_neq_vs2_neq_vm, 0 },
{"vnsra.wi",   0, {"V", 0},  "Vd,Vt,VjVm", MATCH_VNSRA_WI, MASK_VNSRA_WI, match_narrow_vd_neq_vs2_neq_vm, 0 },

{"vmseq.vv",   0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VMSEQ_VV,  MASK_VMSEQ_VV,  match_opcode, 0 },
{"vmseq.vx",   0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VMSEQ_VX,  MASK_VMSEQ_VX,  match_opcode, 0 },
{"vmseq.vi",   0, {"V", 0},  "Vd,Vt,ViVm", MATCH_VMSEQ_VI,  MASK_VMSEQ_VI,  match_opcode, 0 },
{"vmsne.vv",   0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VMSNE_VV,  MASK_VMSNE_VV,  match_opcode, 0 },
{"vmsne.vx",   0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VMSNE_VX,  MASK_VMSNE_VX,  match_opcode, 0 },
{"vmsne.vi",   0, {"V", 0},  "Vd,Vt,ViVm", MATCH_VMSNE_VI,  MASK_VMSNE_VI,  match_opcode, 0 },
{"vmsltu.vv",  0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VMSLTU_VV, MASK_VMSLTU_VV, match_opcode, 0 },
{"vmsltu.vx",  0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VMSLTU_VX, MASK_VMSLTU_VX, match_opcode, 0 },
{"vmslt.vv",   0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VMSLT_VV,  MASK_VMSLT_VV,  match_opcode, 0 },
{"vmslt.vx",   0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VMSLT_VX,  MASK_VMSLT_VX,  match_opcode, 0 },
{"vmsleu.vv",  0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VMSLEU_VV, MASK_VMSLEU_VV, match_opcode, 0 },
{"vmsleu.vx",  0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VMSLEU_VX, MASK_VMSLEU_VX, match_opcode, 0 },
{"vmsleu.vi",  0, {"V", 0},  "Vd,Vt,ViVm", MATCH_VMSLEU_VI, MASK_VMSLEU_VI, match_opcode, 0 },
{"vmsle.vv",   0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VMSLE_VV,  MASK_VMSLE_VV,  match_opcode, 0 },
{"vmsle.vx",   0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VMSLE_VX,  MASK_VMSLE_VX,  match_opcode, 0 },
{"vmsle.vi",   0, {"V", 0},  "Vd,Vt,ViVm", MATCH_VMSLE_VI,  MASK_VMSLE_VI,  match_opcode, 0 },
{"vmsgtu.vx",  0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VMSGTU_VX, MASK_VMSGTU_VX, match_opcode, 0 },
{"vmsgtu.vi",  0, {"V", 0},  "Vd,Vt,ViVm", MATCH_VMSGTU_VI, MASK_VMSGTU_VI, match_opcode, 0 },
{"vmsgt.vx",   0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VMSGT_VX,  MASK_VMSGT_VX,  match_opcode, 0 },
{"vmsgt.vi",   0, {"V", 0},  "Vd,Vt,ViVm", MATCH_VMSGT_VI,  MASK_VMSGT_VI,  match_opcode, 0 },

/* These aliases are for assembly but not disassembly.  */
{"vmsgt.vv",   0, {"V", 0},  "Vd,Vs,VtVm", MATCH_VMSLT_VV,  MASK_VMSLT_VV,  match_opcode, INSN_ALIAS },
{"vmsgtu.vv",  0, {"V", 0},  "Vd,Vs,VtVm", MATCH_VMSLTU_VV, MASK_VMSLTU_VV, match_opcode, INSN_ALIAS },
{"vmsge.vv",   0, {"V", 0},  "Vd,Vs,VtVm", MATCH_VMSLE_VV,  MASK_VMSLE_VV,  match_opcode, INSN_ALIAS },
{"vmsgeu.vv",  0, {"V", 0},  "Vd,Vs,VtVm", MATCH_VMSLEU_VV, MASK_VMSLEU_VV, match_opcode, INSN_ALIAS },
{"vmslt.vi",   0, {"V", 0},  "Vd,Vt,VkVm", MATCH_VMSLE_VI,  MASK_VMSLE_VI,  match_opcode, INSN_ALIAS },
{"vmsltu.vi",  0, {"V", 0},  "Vd,Vu,0Vm",  MATCH_VMSNE_VV,  MASK_VMSNE_VV,  match_opcode, INSN_ALIAS },
{"vmsltu.vi",  0, {"V", 0},  "Vd,Vt,VkVm", MATCH_VMSLEU_VI, MASK_VMSLEU_VI, match_opcode, INSN_ALIAS },
{"vmsge.vi",   0, {"V", 0},  "Vd,Vt,VkVm", MATCH_VMSGT_VI,  MASK_VMSGT_VI,  match_opcode, INSN_ALIAS },
{"vmsgeu.vi",  0, {"V", 0},  "Vd,Vu,0Vm",  MATCH_VMSEQ_VV,  MASK_VMSEQ_VV,  match_opcode, INSN_ALIAS },
{"vmsgeu.vi",  0, {"V", 0},  "Vd,Vt,VkVm", MATCH_VMSGTU_VI, MASK_VMSGTU_VI, match_opcode, INSN_ALIAS },

{"vmsge.vx",   0, {"V", 0}, "Vd,Vt,sVm", 0, (int) M_VMSGE, match_never, INSN_MACRO },
{"vmsge.vx",   0, {"V", 0}, "Vd,Vt,s,VM,VT", 0, (int) M_VMSGE, match_never, INSN_MACRO },
{"vmsgeu.vx",  0, {"V", 0}, "Vd,Vt,sVm", 0, (int) M_VMSGEU, match_never, INSN_MACRO },
{"vmsgeu.vx",  0, {"V", 0}, "Vd,Vt,s,VM,VT", 0, (int) M_VMSGEU, match_never, INSN_MACRO },

{"vminu.vv",   0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VMINU_VV, MASK_VMINU_VV, match_vd_neq_vm, 0},
{"vminu.vx",   0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VMINU_VX, MASK_VMINU_VX, match_vd_neq_vm, 0},
{"vmin.vv",    0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VMIN_VV,  MASK_VMIN_VV,  match_vd_neq_vm, 0},
{"vmin.vx",    0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VMIN_VX,  MASK_VMIN_VX,  match_vd_neq_vm, 0},
{"vmaxu.vv",   0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VMAXU_VV, MASK_VMAXU_VV, match_vd_neq_vm, 0},
{"vmaxu.vx",   0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VMAXU_VX, MASK_VMAXU_VX, match_vd_neq_vm, 0},
{"vmax.vv",    0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VMAX_VV,  MASK_VMAX_VV,  match_vd_neq_vm, 0},
{"vmax.vx",    0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VMAX_VX,  MASK_VMAX_VX,  match_vd_neq_vm, 0},

{"vmul.vv",    0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VMUL_VV,    MASK_VMUL_VV,    match_vd_neq_vm, 0 },
{"vmul.vx",    0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VMUL_VX,    MASK_VMUL_VX,    match_vd_neq_vm, 0 },
{"vmulh.vv",   0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VMULH_VV,   MASK_VMULH_VV,   match_vd_neq_vm, 0 },
{"vmulh.vx",   0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VMULH_VX,   MASK_VMULH_VX,   match_vd_neq_vm, 0 },
{"vmulhu.vv",  0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VMULHU_VV,  MASK_VMULHU_VV,  match_vd_neq_vm, 0 },
{"vmulhu.vx",  0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VMULHU_VX,  MASK_VMULHU_VX,  match_vd_neq_vm, 0 },
{"vmulhsu.vv", 0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VMULHSU_VV, MASK_VMULHSU_VV, match_vd_neq_vm, 0 },
{"vmulhsu.vx", 0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VMULHSU_VX, MASK_VMULHSU_VX, match_vd_neq_vm, 0 },

{"vwmul.vv",   0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VWMUL_VV,   MASK_VWMUL_VV,   match_widen_vd_neq_vs1_neq_vs2_neq_vm, 0 },
{"vwmul.vx",   0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VWMUL_VX,   MASK_VWMUL_VX,   match_widen_vd_neq_vs2_neq_vm, 0 },
{"vwmulu.vv",  0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VWMULU_VV,  MASK_VWMULU_VV,  match_widen_vd_neq_vs1_neq_vs2_neq_vm, 0 },
{"vwmulu.vx",  0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VWMULU_VX,  MASK_VWMULU_VX,  match_widen_vd_neq_vs2_neq_vm, 0 },
{"vwmulsu.vv", 0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VWMULSU_VV, MASK_VWMULSU_VV, match_widen_vd_neq_vs1_neq_vs2_neq_vm, 0 },
{"vwmulsu.vx", 0, {"V", 0},  "Vd,Vt,sVm",  MATCH_VWMULSU_VX, MASK_VWMULSU_VX, match_widen_vd_neq_vs2_neq_vm, 0 },

{"vmacc.vv",   0, {"V", 0},  "Vd,Vs,VtVm", MATCH_VMACC_VV,  MASK_VMACC_VV,  match_vd_neq_vm, 0},
{"vmacc.vx",   0, {"V", 0},  "Vd,s,VtVm",  MATCH_VMACC_VX,  MASK_VMACC_VX,  match_vd_neq_vm, 0},
{"vnmsac.vv",  0, {"V", 0},  "Vd,Vs,VtVm", MATCH_VNMSAC_VV, MASK_VNMSAC_VV, match_vd_neq_vm, 0},
{"vnmsac.vx",  0, {"V", 0},  "Vd,s,VtVm",  MATCH_VNMSAC_VX, MASK_VNMSAC_VX, match_vd_neq_vm, 0},
{"vmadd.vv",   0, {"V", 0},  "Vd,Vs,VtVm", MATCH_VMADD_VV,  MASK_VMADD_VV,  match_vd_neq_vm, 0},
{"vmadd.vx",   0, {"V", 0},  "Vd,s,VtVm",  MATCH_VMADD_VX,  MASK_VMADD_VX,  match_vd_neq_vm, 0},
{"vnmsub.vv",  0, {"V", 0},  "Vd,Vs,VtVm", MATCH_VNMSUB_VV, MASK_VNMSUB_VV, match_vd_neq_vm, 0},
{"vnmsub.vx",  0, {"V", 0},  "Vd,s,VtVm",  MATCH_VNMSUB_VX, MASK_VNMSUB_VX, match_vd_neq_vm, 0},

{"vwmaccu.vv",  0, {"V", 0},  "Vd,Vs,VtVm", MATCH_VWMACCU_VV,  MASK_VWMACCU_VV,  match_widen_vd_neq_vs1_neq_vs2_neq_vm, 0},
{"vwmaccu.vx",  0, {"V", 0},  "Vd,s,VtVm",  MATCH_VWMACCU_VX,  MASK_VWMACCU_VX,  match_widen_vd_neq_vs2_neq_vm, 0},
{"vwmacc.vv",   0, {"V", 0},  "Vd,Vs,VtVm", MATCH_VWMACC_VV,   MASK_VWMACC_VV,   match_widen_vd_neq_vs1_neq_vs2_neq_vm, 0},
{"vwmacc.vx",   0, {"V", 0},  "Vd,s,VtVm",  MATCH_VWMACC_VX,   MASK_VWMACC_VX,   match_widen_vd_neq_vs2_neq_vm, 0},
{"vwmaccsu.vv", 0, {"V", 0},  "Vd,Vs,VtVm", MATCH_VWMACCSU_VV, MASK_VWMACCSU_VV, match_widen_vd_neq_vs1_neq_vs2_neq_vm, 0},
{"vwmaccsu.vx", 0, {"V", 0},  "Vd,s,VtVm",  MATCH_VWMACCSU_VX, MASK_VWMACCSU_VX, match_widen_vd_neq_vs2_neq_vm, 0},
{"vwmaccus.vx", 0, {"V", 0},  "Vd,s,VtVm",  MATCH_VWMACCUS_VX, MASK_VWMACCUS_VX, match_widen_vd_neq_vs2_neq_vm, 0},

{"vqmaccu.vv",  0, {"V", 0},  "Vd,Vs,VtVm", MATCH_VQMACCU_VV,  MASK_VQMACCU_VV,  match_quad_vd_neq_vs1_neq_vs2_neq_vm, 0},
{"vqmaccu.vx",  0, {"V", 0},  "Vd,s,VtVm",  MATCH_VQMACCU_VX,  MASK_VQMACCU_VX,  match_quad_vd_neq_vs2_neq_vm, 0},
{"vqmacc.vv",   0, {"V", 0},  "Vd,Vs,VtVm", MATCH_VQMACC_VV,   MASK_VQMACC_VV,   match_quad_vd_neq_vs1_neq_vs2_neq_vm, 0},
{"vqmacc.vx",   0, {"V", 0},  "Vd,s,VtVm",  MATCH_VQMACC_VX,   MASK_VQMACC_VX,   match_quad_vd_neq_vs2_neq_vm, 0},
{"vqmaccsu.vv", 0, {"V", 0},  "Vd,Vs,VtVm", MATCH_VQMACCSU_VV, MASK_VQMACCSU_VV, match_quad_vd_neq_vs1_neq_vs2_neq_vm, 0},
{"vqmaccsu.vx", 0, {"V", 0},  "Vd,s,VtVm",  MATCH_VQMACCSU_VX, MASK_VQMACCSU_VX, match_quad_vd_neq_vs2_neq_vm, 0},
{"vqmaccus.vx", 0, {"V", 0},  "Vd,s,VtVm",  MATCH_VQMACCUS_VX, MASK_VQMACCUS_VX, match_quad_vd_neq_vs2_neq_vm, 0},

{"vdivu.vv",   0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VDIVU_VV, MASK_VDIVU_VV, match_vd_neq_vm, 0 },
{"vdivu.vx",   0, {"V", 0},  "Vd,Vt,sVm", MATCH_VDIVU_VX, MASK_VDIVU_VX, match_vd_neq_vm, 0 },
{"vdiv.vv",    0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VDIV_VV, MASK_VDIV_VV, match_vd_neq_vm, 0 },
{"vdiv.vx",    0, {"V", 0},  "Vd,Vt,sVm", MATCH_VDIV_VX, MASK_VDIV_VX, match_vd_neq_vm, 0 },
{"vremu.vv",   0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VREMU_VV, MASK_VREMU_VV, match_vd_neq_vm, 0 },
{"vremu.vx",   0, {"V", 0},  "Vd,Vt,sVm", MATCH_VREMU_VX, MASK_VREMU_VX, match_vd_neq_vm, 0 },
{"vrem.vv",    0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VREM_VV, MASK_VREM_VV, match_vd_neq_vm, 0 },
{"vrem.vx",    0, {"V", 0},  "Vd,Vt,sVm", MATCH_VREM_VX, MASK_VREM_VX, match_vd_neq_vm, 0 },

{"vmerge.vvm", 0, {"V", 0},  "Vd,Vt,Vs,V0", MATCH_VMERGE_VVM, MASK_VMERGE_VVM, match_opcode, 0 },
{"vmerge.vxm", 0, {"V", 0},  "Vd,Vt,s,V0", MATCH_VMERGE_VXM, MASK_VMERGE_VXM, match_opcode, 0 },
{"vmerge.vim", 0, {"V", 0},  "Vd,Vt,Vi,V0", MATCH_VMERGE_VIM, MASK_VMERGE_VIM, match_opcode, 0 },

{"vmv.v.v",    0, {"V", 0},  "Vd,Vs", MATCH_VMV_V_V, MASK_VMV_V_V, match_opcode, 0 },
{"vmv.v.x",    0, {"V", 0},  "Vd,s", MATCH_VMV_V_X, MASK_VMV_V_X, match_opcode, 0 },
{"vmv.v.i",    0, {"V", 0},  "Vd,Vi", MATCH_VMV_V_I, MASK_VMV_V_I, match_opcode, 0 },

{"vsaddu.vv",  0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VSADDU_VV, MASK_VSADDU_VV, match_vd_neq_vm, 0 },
{"vsaddu.vx",  0, {"V", 0},  "Vd,Vt,sVm", MATCH_VSADDU_VX, MASK_VSADDU_VX, match_vd_neq_vm, 0 },
{"vsaddu.vi",  0, {"V", 0},  "Vd,Vt,ViVm", MATCH_VSADDU_VI, MASK_VSADDU_VI, match_vd_neq_vm, 0 },
{"vsadd.vv",   0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VSADD_VV, MASK_VSADD_VV, match_vd_neq_vm, 0 },
{"vsadd.vx",   0, {"V", 0},  "Vd,Vt,sVm", MATCH_VSADD_VX, MASK_VSADD_VX, match_vd_neq_vm, 0 },
{"vsadd.vi",   0, {"V", 0},  "Vd,Vt,ViVm", MATCH_VSADD_VI, MASK_VSADD_VI, match_vd_neq_vm, 0 },
{"vssubu.vv",  0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VSSUBU_VV, MASK_VSSUBU_VV, match_vd_neq_vm, 0 },
{"vssubu.vx",  0, {"V", 0},  "Vd,Vt,sVm", MATCH_VSSUBU_VX, MASK_VSSUBU_VX, match_vd_neq_vm, 0 },
{"vssub.vv",   0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VSSUB_VV, MASK_VSSUB_VV, match_vd_neq_vm, 0 },
{"vssub.vx",   0, {"V", 0},  "Vd,Vt,sVm", MATCH_VSSUB_VX, MASK_VSSUB_VX, match_vd_neq_vm, 0 },

{"vaaddu.vv",  0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VAADDU_VV, MASK_VAADDU_VV, match_vd_neq_vm, 0 },
{"vaaddu.vx",  0, {"V", 0},  "Vd,Vt,sVm", MATCH_VAADDU_VX, MASK_VAADDU_VX, match_vd_neq_vm, 0 },
{"vaadd.vv",   0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VAADD_VV, MASK_VAADD_VV, match_vd_neq_vm, 0 },
{"vaadd.vx",   0, {"V", 0},  "Vd,Vt,sVm", MATCH_VAADD_VX, MASK_VAADD_VX, match_vd_neq_vm, 0 },
{"vasubu.vv",  0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VASUBU_VV, MASK_VASUBU_VV, match_vd_neq_vm, 0 },
{"vasubu.vx",  0, {"V", 0},  "Vd,Vt,sVm", MATCH_VASUBU_VX, MASK_VASUBU_VX, match_vd_neq_vm, 0 },
{"vasub.vv",   0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VASUB_VV, MASK_VASUB_VV, match_vd_neq_vm, 0 },
{"vasub.vx",   0, {"V", 0},  "Vd,Vt,sVm", MATCH_VASUB_VX, MASK_VASUB_VX, match_vd_neq_vm, 0 },

{"vsmul.vv",   0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VSMUL_VV, MASK_VSMUL_VV, match_vd_neq_vm, 0 },
{"vsmul.vx",   0, {"V", 0},  "Vd,Vt,sVm", MATCH_VSMUL_VX, MASK_VSMUL_VX, match_vd_neq_vm, 0 },

{"vssrl.vv",    0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VSSRL_VV, MASK_VSSRL_VV, match_vd_neq_vm, 0 },
{"vssrl.vx",    0, {"V", 0},  "Vd,Vt,sVm", MATCH_VSSRL_VX, MASK_VSSRL_VX, match_vd_neq_vm, 0 },
{"vssrl.vi",    0, {"V", 0},  "Vd,Vt,VjVm", MATCH_VSSRL_VI, MASK_VSSRL_VI, match_vd_neq_vm, 0 },
{"vssra.vv",    0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VSSRA_VV, MASK_VSSRA_VV, match_vd_neq_vm, 0 },
{"vssra.vx",    0, {"V", 0},  "Vd,Vt,sVm", MATCH_VSSRA_VX, MASK_VSSRA_VX, match_vd_neq_vm, 0 },
{"vssra.vi",    0, {"V", 0},  "Vd,Vt,VjVm", MATCH_VSSRA_VI, MASK_VSSRA_VI, match_vd_neq_vm, 0 },

{"vnclipu.wv",   0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VNCLIPU_WV, MASK_VNCLIPU_WV, match_narrow_vd_neq_vs2_neq_vm, 0 },
{"vnclipu.wx",   0, {"V", 0},  "Vd,Vt,sVm", MATCH_VNCLIPU_WX, MASK_VNCLIPU_WX, match_narrow_vd_neq_vs2_neq_vm, 0 },
{"vnclipu.wi",   0, {"V", 0},  "Vd,Vt,VjVm", MATCH_VNCLIPU_WI, MASK_VNCLIPU_WI, match_narrow_vd_neq_vs2_neq_vm, 0 },
{"vnclip.wv",   0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VNCLIP_WV, MASK_VNCLIP_WV, match_narrow_vd_neq_vs2_neq_vm, 0 },
{"vnclip.wx",   0, {"V", 0},  "Vd,Vt,sVm", MATCH_VNCLIP_WX, MASK_VNCLIP_WX, match_narrow_vd_neq_vs2_neq_vm, 0 },
{"vnclip.wi",   0, {"V", 0},  "Vd,Vt,VjVm", MATCH_VNCLIP_WI, MASK_VNCLIP_WI, match_narrow_vd_neq_vs2_neq_vm, 0 },

{"vfadd.vv",   0, {"V", "F", 0}, "Vd,Vt,VsVm", MATCH_VFADD_VV, MASK_VFADD_VV, match_vd_neq_vm, 0},
{"vfadd.vf",   0, {"V", "F", 0}, "Vd,Vt,SVm", MATCH_VFADD_VF, MASK_VFADD_VF, match_vd_neq_vm, 0},
{"vfsub.vv",   0, {"V", "F", 0}, "Vd,Vt,VsVm", MATCH_VFSUB_VV, MASK_VFSUB_VV, match_vd_neq_vm, 0},
{"vfsub.vf",   0, {"V", "F", 0}, "Vd,Vt,SVm", MATCH_VFSUB_VF, MASK_VFSUB_VF, match_vd_neq_vm, 0},
{"vfrsub.vf",  0, {"V", "F", 0}, "Vd,Vt,SVm", MATCH_VFRSUB_VF, MASK_VFRSUB_VF, match_vd_neq_vm, 0},

{"vfwadd.vv",   0, {"V", "F", 0}, "Vd,Vt,VsVm", MATCH_VFWADD_VV, MASK_VFWADD_VV, match_widen_vd_neq_vs1_neq_vs2_neq_vm, 0},
{"vfwadd.vf",   0, {"V", "F", 0}, "Vd,Vt,SVm", MATCH_VFWADD_VF, MASK_VFWADD_VF, match_widen_vd_neq_vs2_neq_vm, 0},
{"vfwsub.vv",   0, {"V", "F", 0}, "Vd,Vt,VsVm", MATCH_VFWSUB_VV, MASK_VFWSUB_VV, match_widen_vd_neq_vs1_neq_vs2_neq_vm, 0},
{"vfwsub.vf",   0, {"V", "F", 0}, "Vd,Vt,SVm", MATCH_VFWSUB_VF, MASK_VFWSUB_VF, match_widen_vd_neq_vs2_neq_vm, 0},
{"vfwadd.wv",   0, {"V", "F", 0}, "Vd,Vt,VsVm", MATCH_VFWADD_WV, MASK_VFWADD_WV, match_widen_vd_neq_vs1_neq_vm, 0},
{"vfwadd.wf",   0, {"V", "F", 0}, "Vd,Vt,SVm", MATCH_VFWADD_WF, MASK_VFWADD_WF, match_widen_vd_neq_vm, 0},
{"vfwsub.wv",   0, {"V", "F", 0}, "Vd,Vt,VsVm", MATCH_VFWSUB_WV, MASK_VFWSUB_WV, match_widen_vd_neq_vs1_neq_vm, 0},
{"vfwsub.wf",   0, {"V", "F", 0}, "Vd,Vt,SVm", MATCH_VFWSUB_WF, MASK_VFWSUB_WF, match_widen_vd_neq_vm, 0},

{"vfmul.vv",   0, {"V", "F", 0}, "Vd,Vt,VsVm", MATCH_VFMUL_VV, MASK_VFMUL_VV, match_vd_neq_vm, 0},
{"vfmul.vf",   0, {"V", "F", 0}, "Vd,Vt,SVm", MATCH_VFMUL_VF, MASK_VFMUL_VF, match_vd_neq_vm, 0},
{"vfdiv.vv",   0, {"V", "F", 0}, "Vd,Vt,VsVm", MATCH_VFDIV_VV, MASK_VFDIV_VV, match_vd_neq_vm, 0},
{"vfdiv.vf",   0, {"V", "F", 0}, "Vd,Vt,SVm", MATCH_VFDIV_VF, MASK_VFDIV_VF, match_vd_neq_vm, 0},
{"vfrdiv.vf",  0, {"V", "F", 0}, "Vd,Vt,SVm", MATCH_VFRDIV_VF, MASK_VFRDIV_VF, match_vd_neq_vm, 0},

{"vfwmul.vv",  0, {"V", "F", 0}, "Vd,Vt,VsVm", MATCH_VFWMUL_VV, MASK_VFWMUL_VV, match_widen_vd_neq_vs1_neq_vs2_neq_vm, 0},
{"vfwmul.vf",  0, {"V", "F", 0}, "Vd,Vt,SVm", MATCH_VFWMUL_VF, MASK_VFWMUL_VF, match_widen_vd_neq_vs2_neq_vm, 0},

{"vfmadd.vv",  0, {"V", "F", 0}, "Vd,Vs,VtVm", MATCH_VFMADD_VV, MASK_VFMADD_VV, match_vd_neq_vm, 0},
{"vfmadd.vf",  0, {"V", "F", 0}, "Vd,S,VtVm", MATCH_VFMADD_VF, MASK_VFMADD_VF, match_vd_neq_vm, 0},
{"vfnmadd.vv", 0, {"V", "F", 0}, "Vd,Vs,VtVm", MATCH_VFNMADD_VV, MASK_VFNMADD_VV, match_vd_neq_vm, 0},
{"vfnmadd.vf", 0, {"V", "F", 0}, "Vd,S,VtVm", MATCH_VFNMADD_VF, MASK_VFNMADD_VF, match_vd_neq_vm, 0},
{"vfmsub.vv",  0, {"V", "F", 0}, "Vd,Vs,VtVm", MATCH_VFMSUB_VV, MASK_VFMSUB_VV, match_vd_neq_vm, 0},
{"vfmsub.vf",  0, {"V", "F", 0}, "Vd,S,VtVm", MATCH_VFMSUB_VF, MASK_VFMSUB_VF, match_vd_neq_vm, 0},
{"vfnmsub.vv", 0, {"V", "F", 0}, "Vd,Vs,VtVm", MATCH_VFNMSUB_VV, MASK_VFNMSUB_VV, match_vd_neq_vm, 0},
{"vfnmsub.vf", 0, {"V", "F", 0}, "Vd,S,VtVm", MATCH_VFNMSUB_VF, MASK_VFNMSUB_VF, match_vd_neq_vm, 0},
{"vfmacc.vv",  0, {"V", "F", 0}, "Vd,Vs,VtVm", MATCH_VFMACC_VV, MASK_VFMACC_VV, match_vd_neq_vm, 0},
{"vfmacc.vf",  0, {"V", "F", 0}, "Vd,S,VtVm", MATCH_VFMACC_VF, MASK_VFMACC_VF, match_vd_neq_vm, 0},
{"vfnmacc.vv", 0, {"V", "F", 0}, "Vd,Vs,VtVm", MATCH_VFNMACC_VV, MASK_VFNMACC_VV, match_vd_neq_vm, 0},
{"vfnmacc.vf", 0, {"V", "F", 0}, "Vd,S,VtVm", MATCH_VFNMACC_VF, MASK_VFNMACC_VF, match_vd_neq_vm, 0},
{"vfmsac.vv",  0, {"V", "F", 0}, "Vd,Vs,VtVm", MATCH_VFMSAC_VV, MASK_VFMSAC_VV, match_vd_neq_vm, 0},
{"vfmsac.vf",  0, {"V", "F", 0}, "Vd,S,VtVm", MATCH_VFMSAC_VF, MASK_VFMSAC_VF, match_vd_neq_vm, 0},
{"vfnmsac.vv", 0, {"V", "F", 0}, "Vd,Vs,VtVm", MATCH_VFNMSAC_VV, MASK_VFNMSAC_VV, match_vd_neq_vm, 0},
{"vfnmsac.vf", 0, {"V", "F", 0}, "Vd,S,VtVm", MATCH_VFNMSAC_VF, MASK_VFNMSAC_VF, match_vd_neq_vm, 0},

{"vfwmacc.vv",  0, {"V", "F", 0}, "Vd,Vs,VtVm", MATCH_VFWMACC_VV, MASK_VFWMACC_VV, match_widen_vd_neq_vs1_neq_vs2_neq_vm, 0},
{"vfwmacc.vf",  0, {"V", "F", 0}, "Vd,S,VtVm", MATCH_VFWMACC_VF, MASK_VFWMACC_VF, match_widen_vd_neq_vs2_neq_vm, 0},
{"vfwnmacc.vv", 0, {"V", "F", 0}, "Vd,Vs,VtVm", MATCH_VFWNMACC_VV, MASK_VFWNMACC_VV, match_widen_vd_neq_vs1_neq_vs2_neq_vm, 0},
{"vfwnmacc.vf", 0, {"V", "F", 0}, "Vd,S,VtVm", MATCH_VFWNMACC_VF, MASK_VFWNMACC_VF, match_widen_vd_neq_vs2_neq_vm, 0},
{"vfwmsac.vv",  0, {"V", "F", 0}, "Vd,Vs,VtVm", MATCH_VFWMSAC_VV, MASK_VFWMSAC_VV, match_widen_vd_neq_vs1_neq_vs2_neq_vm, 0},
{"vfwmsac.vf",  0, {"V", "F", 0}, "Vd,S,VtVm", MATCH_VFWMSAC_VF, MASK_VFWMSAC_VF, match_widen_vd_neq_vs2_neq_vm, 0},
{"vfwnmsac.vv", 0, {"V", "F", 0}, "Vd,Vs,VtVm", MATCH_VFWNMSAC_VV, MASK_VFWNMSAC_VV, match_widen_vd_neq_vs1_neq_vs2_neq_vm, 0},
{"vfwnmsac.vf", 0, {"V", "F", 0}, "Vd,S,VtVm", MATCH_VFWNMSAC_VF, MASK_VFWNMSAC_VF, match_widen_vd_neq_vs2_neq_vm, 0},

{"vfsqrt.v",   0, {"V", "F", 0}, "Vd,VtVm", MATCH_VFSQRT_V, MASK_VFSQRT_V, match_vd_neq_vm, 0},
{"vfrsqrt7.v", 0, {"V", "F", 0}, "Vd,VtVm", MATCH_VFRSQRT7_V, MASK_VFRSQRT7_V, match_vd_neq_vm, 0},
{"vfrsqrte7.v",0, {"V", "F", 0}, "Vd,VtVm", MATCH_VFRSQRT7_V, MASK_VFRSQRT7_V, match_vd_neq_vm, 0},
{"vfrec7.v",   0, {"V", "F", 0}, "Vd,VtVm", MATCH_VFREC7_V, MASK_VFREC7_V, match_vd_neq_vm, 0},
{"vfrece7.v",  0, {"V", "F", 0}, "Vd,VtVm", MATCH_VFREC7_V, MASK_VFREC7_V, match_vd_neq_vm, 0},
{"vfclass.v",  0, {"V", "F", 0}, "Vd,VtVm", MATCH_VFCLASS_V, MASK_VFCLASS_V, match_vd_neq_vm, 0},

{"vfmin.vv",   0, {"V", "F", 0}, "Vd,Vt,VsVm", MATCH_VFMIN_VV, MASK_VFMIN_VV, match_vd_neq_vm, 0},
{"vfmin.vf",   0, {"V", "F", 0}, "Vd,Vt,SVm", MATCH_VFMIN_VF, MASK_VFMIN_VF, match_vd_neq_vm, 0},
{"vfmax.vv",   0, {"V", "F", 0}, "Vd,Vt,VsVm", MATCH_VFMAX_VV, MASK_VFMAX_VV, match_vd_neq_vm, 0},
{"vfmax.vf",   0, {"V", "F", 0}, "Vd,Vt,SVm", MATCH_VFMAX_VF, MASK_VFMAX_VF, match_vd_neq_vm, 0},

{"vfneg.v",    0, {"V", "F", 0}, "Vd,VuVm", MATCH_VFSGNJN_VV, MASK_VFSGNJN_VV, match_vs1_eq_vs2_neq_vm, INSN_ALIAS },
{"vfabs.v",    0, {"V", "F", 0}, "Vd,VuVm", MATCH_VFSGNJX_VV, MASK_VFSGNJX_VV, match_vs1_eq_vs2_neq_vm, INSN_ALIAS },

{"vfsgnj.vv",  0, {"V", "F", 0}, "Vd,Vt,VsVm", MATCH_VFSGNJ_VV, MASK_VFSGNJ_VV, match_vd_neq_vm, 0},
{"vfsgnj.vf",  0, {"V", "F", 0}, "Vd,Vt,SVm", MATCH_VFSGNJ_VF, MASK_VFSGNJ_VF, match_vd_neq_vm, 0},
{"vfsgnjn.vv", 0, {"V", "F", 0}, "Vd,Vt,VsVm", MATCH_VFSGNJN_VV, MASK_VFSGNJN_VV, match_vd_neq_vm, 0},
{"vfsgnjn.vf", 0, {"V", "F", 0}, "Vd,Vt,SVm", MATCH_VFSGNJN_VF, MASK_VFSGNJN_VF, match_vd_neq_vm, 0},
{"vfsgnjx.vv", 0, {"V", "F", 0}, "Vd,Vt,VsVm", MATCH_VFSGNJX_VV, MASK_VFSGNJX_VV, match_vd_neq_vm, 0},
{"vfsgnjx.vf", 0, {"V", "F", 0}, "Vd,Vt,SVm", MATCH_VFSGNJX_VF, MASK_VFSGNJX_VF, match_vd_neq_vm, 0},

{"vmfeq.vv",   0, {"V", "F", 0}, "Vd,Vt,VsVm", MATCH_VMFEQ_VV, MASK_VMFEQ_VV, match_opcode, 0},
{"vmfeq.vf",   0, {"V", "F", 0}, "Vd,Vt,SVm", MATCH_VMFEQ_VF, MASK_VMFEQ_VF, match_opcode, 0},
{"vmfne.vv",   0, {"V", "F", 0}, "Vd,Vt,VsVm", MATCH_VMFNE_VV, MASK_VMFNE_VV, match_opcode, 0},
{"vmfne.vf",   0, {"V", "F", 0}, "Vd,Vt,SVm", MATCH_VMFNE_VF, MASK_VMFNE_VF, match_opcode, 0},
{"vmflt.vv",   0, {"V", "F", 0}, "Vd,Vt,VsVm", MATCH_VMFLT_VV, MASK_VMFLT_VV, match_opcode, 0},
{"vmflt.vf",   0, {"V", "F", 0}, "Vd,Vt,SVm", MATCH_VMFLT_VF, MASK_VMFLT_VF, match_opcode, 0},
{"vmfle.vv",   0, {"V", "F", 0}, "Vd,Vt,VsVm", MATCH_VMFLE_VV, MASK_VMFLE_VV, match_opcode, 0},
{"vmfle.vf",   0, {"V", "F", 0}, "Vd,Vt,SVm", MATCH_VMFLE_VF, MASK_VMFLE_VF, match_opcode, 0},
{"vmfgt.vf",   0, {"V", "F", 0}, "Vd,Vt,SVm", MATCH_VMFGT_VF, MASK_VMFGT_VF, match_opcode, 0},
{"vmfge.vf",   0, {"V", "F", 0}, "Vd,Vt,SVm", MATCH_VMFGE_VF, MASK_VMFGE_VF, match_opcode, 0},

/* These aliases are for assembly but not disassembly.  */
{"vmfgt.vv",   0, {"V", "F", 0}, "Vd,Vs,VtVm", MATCH_VMFLT_VV, MASK_VMFLT_VV, match_opcode, INSN_ALIAS},
{"vmfge.vv",   0, {"V", "F", 0}, "Vd,Vs,VtVm", MATCH_VMFLE_VV, MASK_VMFLE_VV, match_opcode, INSN_ALIAS},

{"vfmerge.vfm",0, {"V", "F", 0}, "Vd,Vt,S,V0", MATCH_VFMERGE_VFM, MASK_VFMERGE_VFM, match_opcode, 0},
{"vfmv.v.f",   0, {"V", "F", 0}, "Vd,S", MATCH_VFMV_V_F, MASK_VFMV_V_F, match_opcode, 0 },

{"vfcvt.xu.f.v",     0, {"V", "F", 0}, "Vd,VtVm", MATCH_VFCVT_XU_F_V, MASK_VFCVT_XU_F_V, match_vd_neq_vm, 0},
{"vfcvt.x.f.v",      0, {"V", "F", 0}, "Vd,VtVm", MATCH_VFCVT_X_F_V, MASK_VFCVT_X_F_V, match_vd_neq_vm, 0},
{"vfcvt.rtz.xu.f.v", 0, {"V", "F", 0}, "Vd,VtVm", MATCH_VFCVT_RTZ_XU_F_V, MASK_VFCVT_RTZ_XU_F_V, match_vd_neq_vm, 0},
{"vfcvt.rtz.x.f.v",  0, {"V", "F", 0}, "Vd,VtVm", MATCH_VFCVT_RTZ_X_F_V, MASK_VFCVT_RTZ_X_F_V, match_vd_neq_vm, 0},
{"vfcvt.f.xu.v",     0, {"V", "F", 0}, "Vd,VtVm", MATCH_VFCVT_F_XU_V, MASK_VFCVT_F_XU_V, match_vd_neq_vm, 0},
{"vfcvt.f.x.v",      0, {"V", "F", 0}, "Vd,VtVm", MATCH_VFCVT_F_X_V, MASK_VFCVT_F_X_V, match_vd_neq_vm, 0},

{"vfwcvt.xu.f.v",     0, {"V", "F", 0}, "Vd,VtVm", MATCH_VFWCVT_XU_F_V, MASK_VFWCVT_XU_F_V, match_widen_vd_neq_vs2_neq_vm, 0},
{"vfwcvt.x.f.v",      0, {"V", "F", 0}, "Vd,VtVm", MATCH_VFWCVT_X_F_V, MASK_VFWCVT_X_F_V, match_widen_vd_neq_vs2_neq_vm, 0},
{"vfwcvt.rtz.xu.f.v", 0, {"V", "F", 0}, "Vd,VtVm", MATCH_VFWCVT_RTZ_XU_F_V, MASK_VFWCVT_RTZ_XU_F_V, match_widen_vd_neq_vs2_neq_vm, 0},
{"vfwcvt.rtz.x.f.v",  0, {"V", "F", 0}, "Vd,VtVm", MATCH_VFWCVT_RTZ_X_F_V, MASK_VFWCVT_RTZ_X_F_V, match_widen_vd_neq_vs2_neq_vm, 0},
{"vfwcvt.f.xu.v",     0, {"V", "F", 0}, "Vd,VtVm", MATCH_VFWCVT_F_XU_V, MASK_VFWCVT_F_XU_V, match_widen_vd_neq_vs2_neq_vm, 0},
{"vfwcvt.f.x.v",      0, {"V", "F", 0}, "Vd,VtVm", MATCH_VFWCVT_F_X_V, MASK_VFWCVT_F_X_V, match_widen_vd_neq_vs2_neq_vm, 0},
{"vfwcvt.f.f.v",      0, {"V", "F", 0}, "Vd,VtVm", MATCH_VFWCVT_F_F_V, MASK_VFWCVT_F_F_V, match_widen_vd_neq_vs2_neq_vm, 0},

{"vfncvt.xu.f.w",     0, {"V", "F", 0}, "Vd,VtVm", MATCH_VFNCVT_XU_F_W, MASK_VFNCVT_XU_F_W, match_narrow_vd_neq_vs2_neq_vm, 0},
{"vfncvt.x.f.w",      0, {"V", "F", 0}, "Vd,VtVm", MATCH_VFNCVT_X_F_W, MASK_VFNCVT_X_F_W, match_narrow_vd_neq_vs2_neq_vm, 0},
{"vfncvt.rtz.xu.f.w", 0, {"V", "F", 0}, "Vd,VtVm", MATCH_VFNCVT_RTZ_XU_F_W, MASK_VFNCVT_RTZ_XU_F_W, match_narrow_vd_neq_vs2_neq_vm, 0},
{"vfncvt.rtz.x.f.w",  0, {"V", "F", 0}, "Vd,VtVm", MATCH_VFNCVT_RTZ_X_F_W, MASK_VFNCVT_RTZ_X_F_W, match_narrow_vd_neq_vs2_neq_vm, 0},
{"vfncvt.f.xu.w",     0, {"V", "F", 0}, "Vd,VtVm", MATCH_VFNCVT_F_XU_W, MASK_VFNCVT_F_XU_W, match_narrow_vd_neq_vs2_neq_vm, 0},
{"vfncvt.f.x.w",      0, {"V", "F", 0}, "Vd,VtVm", MATCH_VFNCVT_F_X_W, MASK_VFNCVT_F_X_W, match_narrow_vd_neq_vs2_neq_vm, 0},
{"vfncvt.f.f.w",      0, {"V", "F", 0}, "Vd,VtVm", MATCH_VFNCVT_F_F_W, MASK_VFNCVT_F_F_W, match_narrow_vd_neq_vs2_neq_vm, 0},
{"vfncvt.rod.f.f.w",  0, {"V", "F", 0}, "Vd,VtVm", MATCH_VFNCVT_ROD_F_F_W, MASK_VFNCVT_ROD_F_F_W, match_narrow_vd_neq_vs2_neq_vm, 0},

{"vredsum.vs", 0, {"V", 0}, "Vd,Vt,VsVm", MATCH_VREDSUM_VS, MASK_VREDSUM_VS, match_opcode, 0},
{"vredmaxu.vs",0, {"V", 0}, "Vd,Vt,VsVm", MATCH_VREDMAXU_VS, MASK_VREDMAXU_VS, match_opcode, 0},
{"vredmax.vs", 0, {"V", 0}, "Vd,Vt,VsVm", MATCH_VREDMAX_VS, MASK_VREDMAX_VS, match_opcode, 0},
{"vredminu.vs",0, {"V", 0}, "Vd,Vt,VsVm", MATCH_VREDMINU_VS, MASK_VREDMINU_VS, match_opcode, 0},
{"vredmin.vs", 0, {"V", 0}, "Vd,Vt,VsVm", MATCH_VREDMIN_VS, MASK_VREDMIN_VS, match_opcode, 0},
{"vredand.vs", 0, {"V", 0}, "Vd,Vt,VsVm", MATCH_VREDAND_VS, MASK_VREDAND_VS, match_opcode, 0},
{"vredor.vs",  0, {"V", 0}, "Vd,Vt,VsVm", MATCH_VREDOR_VS, MASK_VREDOR_VS, match_opcode, 0},
{"vredxor.vs", 0, {"V", 0}, "Vd,Vt,VsVm", MATCH_VREDXOR_VS, MASK_VREDXOR_VS, match_opcode, 0},

{"vwredsumu.vs",0, {"V", 0}, "Vd,Vt,VsVm", MATCH_VWREDSUMU_VS, MASK_VWREDSUMU_VS, match_opcode, 0},
{"vwredsum.vs",0, {"V", 0}, "Vd,Vt,VsVm", MATCH_VWREDSUM_VS, MASK_VWREDSUM_VS, match_opcode, 0},

{"vfredosum.vs",0, {"V", "F", 0}, "Vd,Vt,VsVm", MATCH_VFREDOSUM_VS, MASK_VFREDOSUM_VS, match_opcode, 0},
{"vfredsum.vs", 0, {"V", "F", 0}, "Vd,Vt,VsVm", MATCH_VFREDSUM_VS, MASK_VFREDSUM_VS, match_opcode, 0},
{"vfredmax.vs", 0, {"V", "F", 0}, "Vd,Vt,VsVm", MATCH_VFREDMAX_VS, MASK_VFREDMAX_VS, match_opcode, 0},
{"vfredmin.vs", 0, {"V", "F", 0}, "Vd,Vt,VsVm", MATCH_VFREDMIN_VS, MASK_VFREDMIN_VS, match_opcode, 0},

{"vfwredosum.vs",0, {"V", "F", 0}, "Vd,Vt,VsVm", MATCH_VFWREDOSUM_VS, MASK_VFWREDOSUM_VS, match_opcode, 0},
{"vfwredsum.vs", 0, {"V", "F", 0}, "Vd,Vt,VsVm", MATCH_VFWREDSUM_VS, MASK_VFWREDSUM_VS, match_opcode, 0},

{"vmmv.m",     0, {"V", 0}, "Vd,Vu", MATCH_VMAND_MM, MASK_VMAND_MM, match_vs1_eq_vs2, INSN_ALIAS},
{"vmcpy.m",    0, {"V", 0}, "Vd,Vu", MATCH_VMAND_MM, MASK_VMAND_MM, match_vs1_eq_vs2, INSN_ALIAS},
{"vmclr.m",    0, {"V", 0}, "Vv", MATCH_VMXOR_MM, MASK_VMXOR_MM, match_vd_eq_vs1_eq_vs2, INSN_ALIAS},
{"vmset.m",    0, {"V", 0}, "Vv", MATCH_VMXNOR_MM, MASK_VMXNOR_MM, match_vd_eq_vs1_eq_vs2, INSN_ALIAS},
{"vmnot.m",    0, {"V", 0}, "Vd,Vu", MATCH_VMNAND_MM, MASK_VMNAND_MM, match_vs1_eq_vs2, INSN_ALIAS},

{"vmand.mm",   0, {"V", 0}, "Vd,Vt,Vs", MATCH_VMAND_MM, MASK_VMAND_MM, match_opcode, 0},
{"vmnand.mm",  0, {"V", 0}, "Vd,Vt,Vs", MATCH_VMNAND_MM, MASK_VMNAND_MM, match_opcode, 0},
{"vmandnot.mm",0, {"V", 0}, "Vd,Vt,Vs", MATCH_VMANDNOT_MM, MASK_VMANDNOT_MM, match_opcode, 0},
{"vmxor.mm",   0, {"V", 0}, "Vd,Vt,Vs", MATCH_VMXOR_MM, MASK_VMXOR_MM, match_opcode, 0},
{"vmor.mm",    0, {"V", 0}, "Vd,Vt,Vs", MATCH_VMOR_MM, MASK_VMOR_MM, match_opcode, 0},
{"vmnor.mm",   0, {"V", 0}, "Vd,Vt,Vs", MATCH_VMNOR_MM, MASK_VMNOR_MM, match_opcode, 0},
{"vmornot.mm", 0, {"V", 0}, "Vd,Vt,Vs", MATCH_VMORNOT_MM, MASK_VMORNOT_MM, match_opcode, 0},
{"vmxnor.mm",  0, {"V", 0}, "Vd,Vt,Vs", MATCH_VMXNOR_MM, MASK_VMXNOR_MM, match_opcode, 0},

{"vpopc.m",    0, {"V", 0}, "d,VtVm", MATCH_VPOPC_M, MASK_VPOPC_M, match_opcode, 0},
{"vfirst.m",   0, {"V", 0}, "d,VtVm", MATCH_VFIRST_M, MASK_VFIRST_M, match_opcode, 0},
{"vmsbf.m",    0, {"V", 0}, "Vd,VtVm", MATCH_VMSBF_M, MASK_VMSBF_M, match_vd_neq_vs2_neq_vm, 0},
{"vmsif.m",    0, {"V", 0}, "Vd,VtVm", MATCH_VMSIF_M, MASK_VMSIF_M, match_vd_neq_vs2_neq_vm, 0},
{"vmsof.m",    0, {"V", 0}, "Vd,VtVm", MATCH_VMSOF_M, MASK_VMSOF_M, match_vd_neq_vs2_neq_vm, 0},
{"viota.m",    0, {"V", 0}, "Vd,VtVm", MATCH_VIOTA_M, MASK_VIOTA_M, match_vd_neq_vs2_neq_vm, 0},
{"vid.v",      0, {"V", 0}, "VdVm", MATCH_VID_V, MASK_VID_V, match_vd_neq_vm, 0},

{"vmv.x.s",    0, {"V", 0}, "d,Vt", MATCH_VMV_X_S, MASK_VMV_X_S, match_opcode, 0},
{"vmv.s.x",    0, {"V", 0}, "Vd,s", MATCH_VMV_S_X, MASK_VMV_S_X, match_opcode, 0},

{"vfmv.f.s",   0, {"V", "F", 0}, "D,Vt", MATCH_VFMV_F_S, MASK_VFMV_F_S, match_opcode, 0},
{"vfmv.s.f",   0, {"V", "F", 0}, "Vd,S", MATCH_VFMV_S_F, MASK_VFMV_S_F, match_opcode, 0},

{"vslideup.vx",0, {"V", 0}, "Vd,Vt,sVm", MATCH_VSLIDEUP_VX, MASK_VSLIDEUP_VX, match_vd_neq_vs2_neq_vm, 0},
{"vslideup.vi",0, {"V", 0}, "Vd,Vt,VjVm", MATCH_VSLIDEUP_VI, MASK_VSLIDEUP_VI, match_vd_neq_vs2_neq_vm, 0},
{"vslidedown.vx",0,{"V", 0}, "Vd,Vt,sVm", MATCH_VSLIDEDOWN_VX, MASK_VSLIDEDOWN_VX, match_vd_neq_vm, 0},
{"vslidedown.vi",0,{"V", 0}, "Vd,Vt,VjVm", MATCH_VSLIDEDOWN_VI, MASK_VSLIDEDOWN_VI, match_vd_neq_vm, 0},

{"vslide1up.vx",    0, {"V", 0}, "Vd,Vt,sVm", MATCH_VSLIDE1UP_VX, MASK_VSLIDE1UP_VX, match_vd_neq_vs2_neq_vm, 0},
{"vslide1down.vx",  0, {"V", 0}, "Vd,Vt,sVm", MATCH_VSLIDE1DOWN_VX, MASK_VSLIDE1DOWN_VX, match_vd_neq_vm, 0},
{"vfslide1up.vf",   0, {"V", "F", 0}, "Vd,Vt,SVm", MATCH_VFSLIDE1UP_VF, MASK_VFSLIDE1UP_VF, match_vd_neq_vs2_neq_vm, 0},
{"vfslide1down.vf", 0, {"V", "F", 0}, "Vd,Vt,SVm", MATCH_VFSLIDE1DOWN_VF, MASK_VFSLIDE1DOWN_VF, match_vd_neq_vm, 0},

{"vrgather.vv",    0, {"V", 0}, "Vd,Vt,VsVm", MATCH_VRGATHER_VV, MASK_VRGATHER_VV, match_vd_neq_vs1_neq_vs2_neq_vm, 0},
{"vrgather.vx",    0, {"V", 0}, "Vd,Vt,sVm", MATCH_VRGATHER_VX, MASK_VRGATHER_VX, match_vd_neq_vs2_neq_vm, 0},
{"vrgather.vi",    0, {"V", 0}, "Vd,Vt,VjVm", MATCH_VRGATHER_VI, MASK_VRGATHER_VI, match_vd_neq_vs2_neq_vm, 0},
{"vrgatherei16.vv",0, {"V", 0}, "Vd,Vt,VsVm", MATCH_VRGATHEREI16_VV, MASK_VRGATHEREI16_VV, match_vd_neq_vs1_neq_vs2_neq_vm, 0},

{"vcompress.vm",0, {"V", 0}, "Vd,Vt,Vs", MATCH_VCOMPRESS_VM, MASK_VCOMPRESS_VM, match_vd_neq_vs1_neq_vs2, 0},

{"vmv1r.v",    0, {"V", 0}, "Vd,Vt", MATCH_VMV1R_V, MASK_VMV1R_V, match_vmv_nf_rv, 0},
{"vmv2r.v",    0, {"V", 0}, "Vd,Vt", MATCH_VMV2R_V, MASK_VMV2R_V, match_vmv_nf_rv, 0},
{"vmv4r.v",    0, {"V", 0}, "Vd,Vt", MATCH_VMV4R_V, MASK_VMV4R_V, match_vmv_nf_rv, 0},
{"vmv8r.v",    0, {"V", 0}, "Vd,Vt", MATCH_VMV8R_V, MASK_VMV8R_V, match_vmv_nf_rv, 0},

{"vdot.vv",    0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VDOT_VV, MASK_VDOT_VV, match_opcode, 0},
{"vdotu.vv",   0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VDOTU_VV, MASK_VDOTU_VV, match_opcode, 0},
{"vfdot.vv",   0, {"V", 0},  "Vd,Vt,VsVm", MATCH_VFDOT_VV, MASK_VFDOT_VV, match_opcode, 0},
/* END RVV */

/* Zbkx instructions  */
{"xperm4",     0, {"ZBKX", 0},  "d,s,t",  MATCH_XPERM4, MASK_XPERM4, match_opcode, 0 },
{"xperm8",     0, {"ZBKX", 0},  "d,s,t",  MATCH_XPERM8, MASK_XPERM8, match_opcode, 0 },

/* Zknd instructions  */
{"aes32dsi",  32, {"ZKND", 0},  "d,s,t,y",  MATCH_AES32DSI, MASK_AES32DSI, match_opcode, 0 },
{"aes32dsmi", 32, {"ZKND", 0},  "d,s,t,y",  MATCH_AES32DSMI, MASK_AES32DSMI, match_opcode, 0 },
{"aes64ds",   64, {"ZKND", 0},  "d,s,t",    MATCH_AES64DS, MASK_AES64DS, match_opcode, 0 },
{"aes64dsm",  64, {"ZKND", 0},  "d,s,t",    MATCH_AES64DSM, MASK_AES64DSM, match_opcode, 0 },
{"aes64im",   64, {"ZKND", 0},  "d,s",      MATCH_AES64IM, MASK_AES64IM, match_opcode, 0 },
{"aes64ks1i", 64, {"ZKNE", 0}, "d,s,Y",    MATCH_AES64KS1I, MASK_AES64KS1I, match_opcode, 0 },
{"aes64ks2",  64, {"ZKNE", 0}, "d,s,t",    MATCH_AES64KS2, MASK_AES64KS2, match_opcode, 0 },

/* Zkne instructions  */
{"aes32esi",  32, {"ZKNE", 0},  "d,s,t,y",  MATCH_AES32ESI, MASK_AES32ESI, match_opcode, 0 },
{"aes32esmi", 32, {"ZKNE", 0},  "d,s,t,y",  MATCH_AES32ESMI, MASK_AES32ESMI, match_opcode, 0 },
{"aes64es",   64, {"ZKNE", 0},  "d,s,t",    MATCH_AES64ES, MASK_AES64ES, match_opcode, 0 },
{"aes64esm",  64, {"ZKNE", 0},  "d,s,t",    MATCH_AES64ESM, MASK_AES64ESM, match_opcode, 0 },

/* Zknh instructions  */
{"sha256sig0",   0, {"ZKNH", 0},    "d,s",    MATCH_SHA256SIG0, MASK_SHA256SIG0, match_opcode, 0 },
{"sha256sig1",   0, {"ZKNH", 0},    "d,s",    MATCH_SHA256SIG1, MASK_SHA256SIG1, match_opcode, 0 },
{"sha256sum0",   0, {"ZKNH", 0},    "d,s",    MATCH_SHA256SUM0, MASK_SHA256SUM0, match_opcode, 0 },
{"sha256sum1",   0, {"ZKNH", 0},    "d,s",    MATCH_SHA256SUM1, MASK_SHA256SUM1, match_opcode, 0 },
{"sha512sig0h", 32, {"ZKNH", 0},    "d,s,t",  MATCH_SHA512SIG0H, MASK_SHA512SIG0H, match_opcode, 0 },
{"sha512sig0l", 32, {"ZKNH", 0},    "d,s,t",  MATCH_SHA512SIG0L, MASK_SHA512SIG0L, match_opcode, 0 },
{"sha512sig1h", 32, {"ZKNH", 0},    "d,s,t",  MATCH_SHA512SIG1H, MASK_SHA512SIG1H, match_opcode, 0 },
{"sha512sig1l", 32, {"ZKNH", 0},    "d,s,t",  MATCH_SHA512SIG1L, MASK_SHA512SIG1L, match_opcode, 0 },
{"sha512sum0r", 32, {"ZKNH", 0},    "d,s,t",  MATCH_SHA512SUM0R, MASK_SHA512SUM0R, match_opcode, 0 },
{"sha512sum1r", 32, {"ZKNH", 0},    "d,s,t",  MATCH_SHA512SUM1R, MASK_SHA512SUM1R, match_opcode, 0 },
{"sha512sig0",  64, {"ZKNH", 0},    "d,s",    MATCH_SHA512SIG0, MASK_SHA512SIG0, match_opcode, 0 },
{"sha512sig1",  64, {"ZKNH", 0},    "d,s",    MATCH_SHA512SIG1, MASK_SHA512SIG1, match_opcode, 0 },
{"sha512sum0",  64, {"ZKNH", 0},    "d,s",    MATCH_SHA512SUM0, MASK_SHA512SUM0, match_opcode, 0 },
{"sha512sum1",  64, {"ZKNH", 0},    "d,s",    MATCH_SHA512SUM1, MASK_SHA512SUM1, match_opcode, 0 },

/* Zksed instructions  */
{"sm4ed",    0, {"ZKSED", 0},   "d,s,t,y",  MATCH_SM4ED, MASK_SM4ED, match_opcode, 0 },
{"sm4ks",    0, {"ZKSED", 0},   "d,s,t,y",  MATCH_SM4KS, MASK_SM4KS, match_opcode, 0 },

/* Zksh instructions  */
{"sm3p0",    0, {"ZKSH", 0},    "d,s",    MATCH_SM3P0, MASK_SM3P0, match_opcode, 0 },
{"sm3p1",    0, {"ZKSH", 0},    "d,s",    MATCH_SM3P1, MASK_SM3P1, match_opcode, 0 },

/* Terminate the list.  */
{0, 0, {0}, 0, 0, 0, 0, 0}
};

/* Instruction format for .insn directive.  */
const struct riscv_opcode riscv_insn_types[] =
{
/* name, xlen, isa,          operands, match, mask,    match_func, pinfo.  */
{"r",       0, {"I", 0},  "O4,F3,F7,d,s,t",     0,    0,  match_opcode, 0 },
{"r",       0, {"F", 0},  "O4,F3,F7,D,s,t",     0,    0,  match_opcode, 0 },
{"r",       0, {"F", 0},  "O4,F3,F7,d,S,t",     0,    0,  match_opcode, 0 },
{"r",       0, {"F", 0},  "O4,F3,F7,D,S,t",     0,    0,  match_opcode, 0 },
{"r",       0, {"F", 0},  "O4,F3,F7,d,s,T",     0,    0,  match_opcode, 0 },
{"r",       0, {"F", 0},  "O4,F3,F7,D,s,T",     0,    0,  match_opcode, 0 },
{"r",       0, {"F", 0},  "O4,F3,F7,d,S,T",     0,    0,  match_opcode, 0 },
{"r",       0, {"F", 0},  "O4,F3,F7,D,S,T",     0,    0,  match_opcode, 0 },

{"r",       0, {"V", 0},       "O4,F3,F7,Vd,s,t",    0,    0,  match_opcode, 0 },
{"r",       0, {"V", "F", 0},  "O4,F3,F7,Vd,S,t",    0,    0,  match_opcode, 0 },
{"r",       0, {"V", "F", 0},  "O4,F3,F7,Vd,s,T",    0,    0,  match_opcode, 0 },
{"r",       0, {"V", "F", 0},  "O4,F3,F7,Vd,S,T",    0,    0,  match_opcode, 0 },
{"r",       0, {"V", 0},       "O4,F3,F7,d,Vs,t",    0,    0,  match_opcode, 0 },
{"r",       0, {"V", "F", 0},  "O4,F3,F7,D,Vs,t",    0,    0,  match_opcode, 0 },
{"r",       0, {"V", "F", 0},  "O4,F3,F7,d,Vs,T",    0,    0,  match_opcode, 0 },
{"r",       0, {"V", "F", 0},  "O4,F3,F7,D,Vs,T",    0,    0,  match_opcode, 0 },
{"r",       0, {"V", 0},       "O4,F3,F7,d,s,Vt",    0,    0,  match_opcode, 0 },
{"r",       0, {"V", "F", 0},  "O4,F3,F7,D,s,Vt",    0,    0,  match_opcode, 0 },
{"r",       0, {"V", "F", 0},  "O4,F3,F7,d,S,Vt",    0,    0,  match_opcode, 0 },
{"r",       0, {"V", "F", 0},  "O4,F3,F7,D,S,Vt",    0,    0,  match_opcode, 0 },
{"r",       0, {"V", 0},       "O4,F3,F7,Vd,Vs,t",   0,    0,  match_opcode, 0 },
{"r",       0, {"V", "F", 0},  "O4,F3,F7,Vd,Vs,T",   0,    0,  match_opcode, 0 },
{"r",       0, {"V", 0},       "O4,F3,F7,Vd,s,Vt",   0,    0,  match_opcode, 0 },
{"r",       0, {"V", "F", 0},  "O4,F3,F7,Vd,S,Vt",   0,    0,  match_opcode, 0 },
{"r",       0, {"V", 0},       "O4,F3,F7,d,Vs,Vt",   0,    0,  match_opcode, 0 },
{"r",       0, {"V", "F", 0},  "O4,F3,F7,D,Vs,Vt",   0,    0,  match_opcode, 0 },
{"r",       0, {"V", 0},       "O4,F3,F7,Vd,Vs,Vt",  0,    0,  match_opcode, 0 },

{"r",       0, {"I", 0},  "O4,F3,F2,d,s,t,r",   0,    0,  match_opcode, 0 },
{"r",       0, {"F", 0},  "O4,F3,F2,D,s,t,r",   0,    0,  match_opcode, 0 },
{"r",       0, {"F", 0},  "O4,F3,F2,d,S,t,r",   0,    0,  match_opcode, 0 },
{"r",       0, {"F", 0},  "O4,F3,F2,D,S,t,r",   0,    0,  match_opcode, 0 },
{"r",       0, {"F", 0},  "O4,F3,F2,d,s,T,r",   0,    0,  match_opcode, 0 },
{"r",       0, {"F", 0},  "O4,F3,F2,D,s,T,r",   0,    0,  match_opcode, 0 },
{"r",       0, {"F", 0},  "O4,F3,F2,d,S,T,r",   0,    0,  match_opcode, 0 },
{"r",       0, {"F", 0},  "O4,F3,F2,D,S,T,r",   0,    0,  match_opcode, 0 },
{"r",       0, {"F", 0},  "O4,F3,F2,d,s,t,R",   0,    0,  match_opcode, 0 },
{"r",       0, {"F", 0},  "O4,F3,F2,D,s,t,R",   0,    0,  match_opcode, 0 },
{"r",       0, {"F", 0},  "O4,F3,F2,d,S,t,R",   0,    0,  match_opcode, 0 },
{"r",       0, {"F", 0},  "O4,F3,F2,D,S,t,R",   0,    0,  match_opcode, 0 },
{"r",       0, {"F", 0},  "O4,F3,F2,d,s,T,R",   0,    0,  match_opcode, 0 },
{"r",       0, {"F", 0},  "O4,F3,F2,D,s,T,R",   0,    0,  match_opcode, 0 },
{"r",       0, {"F", 0},  "O4,F3,F2,d,S,T,R",   0,    0,  match_opcode, 0 },
{"r",       0, {"F", 0},  "O4,F3,F2,D,S,T,R",   0,    0,  match_opcode, 0 },

{"r4",      0, {"I", 0},  "O4,F3,F2,d,s,t,r",   0,    0,  match_opcode, 0 },
{"r4",      0, {"F", 0},  "O4,F3,F2,D,s,t,r",   0,    0,  match_opcode, 0 },
{"r4",      0, {"F", 0},  "O4,F3,F2,d,S,t,r",   0,    0,  match_opcode, 0 },
{"r4",      0, {"F", 0},  "O4,F3,F2,D,S,t,r",   0,    0,  match_opcode, 0 },
{"r4",      0, {"F", 0},  "O4,F3,F2,d,s,T,r",   0,    0,  match_opcode, 0 },
{"r4",      0, {"F", 0},  "O4,F3,F2,D,s,T,r",   0,    0,  match_opcode, 0 },
{"r4",      0, {"F", 0},  "O4,F3,F2,d,S,T,r",   0,    0,  match_opcode, 0 },
{"r4",      0, {"F", 0},  "O4,F3,F2,D,S,T,r",   0,    0,  match_opcode, 0 },
{"r4",      0, {"F", 0},  "O4,F3,F2,d,s,t,R",   0,    0,  match_opcode, 0 },
{"r4",      0, {"F", 0},  "O4,F3,F2,D,s,t,R",   0,    0,  match_opcode, 0 },
{"r4",      0, {"F", 0},  "O4,F3,F2,d,S,t,R",   0,    0,  match_opcode, 0 },
{"r4",      0, {"F", 0},  "O4,F3,F2,D,S,t,R",   0,    0,  match_opcode, 0 },
{"r4",      0, {"F", 0},  "O4,F3,F2,d,s,T,R",   0,    0,  match_opcode, 0 },
{"r4",      0, {"F", 0},  "O4,F3,F2,D,s,T,R",   0,    0,  match_opcode, 0 },
{"r4",      0, {"F", 0},  "O4,F3,F2,d,S,T,R",   0,    0,  match_opcode, 0 },
{"r4",      0, {"F", 0},  "O4,F3,F2,D,S,T,R",   0,    0,  match_opcode, 0 },

{"i",       0, {"I", 0},  "O4,F3,d,s,j",        0,    0,  match_opcode, 0 },
{"i",       0, {"F", 0},  "O4,F3,D,s,j",        0,    0,  match_opcode, 0 },
{"i",       0, {"F", 0},  "O4,F3,d,S,j",        0,    0,  match_opcode, 0 },
{"i",       0, {"F", 0},  "O4,F3,D,S,j",        0,    0,  match_opcode, 0 },

{"i",       0, {"I", 0},  "O4,F3,d,o(s)",       0,    0,  match_opcode, 0 },
{"i",       0, {"F", 0},  "O4,F3,D,o(s)",       0,    0,  match_opcode, 0 },

{"s",       0, {"I", 0},  "O4,F3,t,q(s)",       0,    0,  match_opcode, 0 },
{"s",       0, {"F", 0},  "O4,F3,T,q(s)",       0,    0,  match_opcode, 0 },

{"sb",      0, {"I", 0},  "O4,F3,s,t,p",        0,    0,  match_opcode, 0 },
{"sb",      0, {"F", 0},  "O4,F3,S,t,p",        0,    0,  match_opcode, 0 },
{"sb",      0, {"F", 0},  "O4,F3,s,T,p",        0,    0,  match_opcode, 0 },
{"sb",      0, {"F", 0},  "O4,F3,S,T,p",        0,    0,  match_opcode, 0 },

{"b",       0, {"I", 0},  "O4,F3,s,t,p",        0,    0,  match_opcode, 0 },
{"b",       0, {"F", 0},  "O4,F3,S,t,p",        0,    0,  match_opcode, 0 },
{"b",       0, {"F", 0},  "O4,F3,s,T,p",        0,    0,  match_opcode, 0 },
{"b",       0, {"F", 0},  "O4,F3,S,T,p",        0,    0,  match_opcode, 0 },

{"u",       0, {"I", 0},  "O4,d,u",             0,    0,  match_opcode, 0 },
{"u",       0, {"F", 0},  "O4,D,u",             0,    0,  match_opcode, 0 },

{"uj",      0, {"I", 0},  "O4,d,a",             0,    0,  match_opcode, 0 },
{"uj",      0, {"F", 0},  "O4,D,a",             0,    0,  match_opcode, 0 },

{"j",       0, {"I", 0},  "O4,d,a",             0,    0,  match_opcode, 0 },
{"j",       0, {"F", 0},  "O4,D,a",             0,    0,  match_opcode, 0 },

{"cr",      0, {"C", 0},       "O2,CF4,d,CV",        0,    0,  match_opcode, 0 },
{"cr",      0, {"F", "C", 0},  "O2,CF4,D,CV",        0,    0,  match_opcode, 0 },
{"cr",      0, {"F", "C", 0},  "O2,CF4,d,CT",        0,    0,  match_opcode, 0 },
{"cr",      0, {"F", "C", 0},  "O2,CF4,D,CT",        0,    0,  match_opcode, 0 },

{"ci",      0, {"C", 0},       "O2,CF3,d,Co",        0,    0,  match_opcode, 0 },
{"ci",      0, {"F", "C", 0},  "O2,CF3,D,Co",        0,    0,  match_opcode, 0 },

{"ciw",     0, {"C", 0},       "O2,CF3,Ct,C8",       0,    0,  match_opcode, 0 },
{"ciw",     0, {"F", "C", 0},  "O2,CF3,CD,C8",       0,    0,  match_opcode, 0 },

{"ca",      0, {"C", 0},       "O2,CF6,CF2,Cs,Ct",   0,    0,  match_opcode, 0 },
{"ca",      0, {"F", "C", 0},  "O2,CF6,CF2,CS,Ct",   0,    0,  match_opcode, 0 },
{"ca",      0, {"F", "C", 0},  "O2,CF6,CF2,Cs,CD",   0,    0,  match_opcode, 0 },
{"ca",      0, {"F", "C", 0},  "O2,CF6,CF2,CS,CD",   0,    0,  match_opcode, 0 },

{"cb",      0, {"C", 0},       "O2,CF3,Cs,Cp",       0,    0,  match_opcode, 0 },
{"cb",      0, {"F", "C", 0},  "O2,CF3,CS,Cp",       0,    0,  match_opcode, 0 },

{"cj",      0, {"C", 0},  "O2,CF3,Ca",          0,    0,  match_opcode, 0 },
/* Terminate the list.  */
{0, 0, {0}, 0, 0, 0, 0, 0}
};

/* All standard extensions defined in all supported ISA spec.  */
const struct riscv_ext_version riscv_ext_version_table[] =
{
/* name, ISA spec, major version, minor_version.  */
{"e", ISA_SPEC_CLASS_20191213, 1, 9},
{"e", ISA_SPEC_CLASS_20190608, 1, 9},
{"e", ISA_SPEC_CLASS_ANDES,    1, 9},
{"e", ISA_SPEC_CLASS_2P2,      1, 9},

{"i", ISA_SPEC_CLASS_20191213, 2, 1},
{"i", ISA_SPEC_CLASS_20190608, 2, 1},
{"i", ISA_SPEC_CLASS_ANDES,    2, 0},
{"i", ISA_SPEC_CLASS_2P2,      2, 0},

{"m", ISA_SPEC_CLASS_20191213, 2, 0},
{"m", ISA_SPEC_CLASS_20190608, 2, 0},
{"m", ISA_SPEC_CLASS_ANDES,    2, 0},
{"m", ISA_SPEC_CLASS_2P2,      2, 0},

{"a", ISA_SPEC_CLASS_20191213, 2, 1},
{"a", ISA_SPEC_CLASS_20190608, 2, 0},
{"a", ISA_SPEC_CLASS_ANDES,    2, 0},
{"a", ISA_SPEC_CLASS_2P2,      2, 0},

{"f", ISA_SPEC_CLASS_20191213, 2, 2},
{"f", ISA_SPEC_CLASS_20190608, 2, 2},
{"f", ISA_SPEC_CLASS_ANDES,    2, 0},
{"f", ISA_SPEC_CLASS_2P2,      2, 0},

{"d", ISA_SPEC_CLASS_20191213, 2, 2},
{"d", ISA_SPEC_CLASS_20190608, 2, 2},
{"d", ISA_SPEC_CLASS_ANDES,    2, 0},
{"d", ISA_SPEC_CLASS_2P2,      2, 0},

{"q", ISA_SPEC_CLASS_20191213, 2, 2},
{"q", ISA_SPEC_CLASS_20190608, 2, 2},
{"q", ISA_SPEC_CLASS_ANDES,    2, 0},
{"q", ISA_SPEC_CLASS_2P2,      2, 0},

{"c", ISA_SPEC_CLASS_20191213, 2, 0},
{"c", ISA_SPEC_CLASS_20190608, 2, 0},
{"c", ISA_SPEC_CLASS_ANDES,    2, 0},
{"c", ISA_SPEC_CLASS_2P2,      2, 0},

{"p", ISA_SPEC_CLASS_20191213, 0, 2},
{"p", ISA_SPEC_CLASS_20190608, 0, 2},
{"p", ISA_SPEC_CLASS_ANDES,    0, 5},
{"p", ISA_SPEC_CLASS_2P2,      0, 1},

{"v", ISA_SPEC_CLASS_ANDES,    1, 0},
{"v", ISA_SPEC_CLASS_NONE,     1, 0},

{"n", ISA_SPEC_CLASS_20190608, 1, 1},
{"n", ISA_SPEC_CLASS_ANDES,    1, 1},
{"n", ISA_SPEC_CLASS_2P2,      1, 1},

{"zfh", ISA_SPEC_CLASS_ANDES,  0, 1},
{"zfh", ISA_SPEC_CLASS_2P2,    0, 1},

// {"zicsr", ISA_SPEC_CLASS_ANDES, 2, 0},
{"zicsr", ISA_SPEC_CLASS_20191213, 2, 0},
{"zicsr", ISA_SPEC_CLASS_20190608, 2, 0},

// {"zifencei", ISA_SPEC_CLASS_ANDES, 2, 0},

{"zvamo",   ISA_SPEC_CLASS_ANDES, 1, 0},
{"zvamo",   ISA_SPEC_CLASS_NONE,  1, 0},

{"zvlsseg", ISA_SPEC_CLASS_ANDES, 1, 0},
{"zvlsseg", ISA_SPEC_CLASS_NONE,  1, 0},

/* Andes  */
{"xandes", ISA_SPEC_CLASS_ANDES, 5, 0},
{"xandes", ISA_SPEC_CLASS_2P2,   5, 0},

{"xefhw", ISA_SPEC_CLASS_ANDES,  1, 0},
{"xefhw", ISA_SPEC_CLASS_2P2,    1, 0},

/* RVB  */
{"b",     ISA_SPEC_CLASS_ANDES, 1, 0},
{"b",     ISA_SPEC_CLASS_DRAFT, 1, 0},
{"zba",   ISA_SPEC_CLASS_ANDES, 1, 0},
{"zba",   ISA_SPEC_CLASS_DRAFT, 1, 0},
{"zbb",   ISA_SPEC_CLASS_ANDES, 1, 0},
{"zbb",   ISA_SPEC_CLASS_DRAFT, 1, 0},
{"zbc",   ISA_SPEC_CLASS_ANDES, 1, 0},
{"zbc",   ISA_SPEC_CLASS_DRAFT, 1, 0},
{"zbs",   ISA_SPEC_CLASS_ANDES, 1, 0},
{"zbs",   ISA_SPEC_CLASS_DRAFT, 1, 0},

/* RVK  */
{"zbkb",   ISA_SPEC_CLASS_ANDES, 1, 0},
{"zbkb",   ISA_SPEC_CLASS_DRAFT, 1, 0},
{"zbkc",   ISA_SPEC_CLASS_ANDES, 1, 0},
{"zbkc",   ISA_SPEC_CLASS_DRAFT, 1, 0},
{"zbkx",   ISA_SPEC_CLASS_ANDES, 1, 0},
{"zbkx",   ISA_SPEC_CLASS_DRAFT, 1, 0},
{"zknd",   ISA_SPEC_CLASS_ANDES, 1, 0},
{"zknd",   ISA_SPEC_CLASS_DRAFT, 1, 0},
{"zkne",   ISA_SPEC_CLASS_ANDES, 1, 0},
{"zkne",   ISA_SPEC_CLASS_DRAFT, 1, 0},
{"zknh",   ISA_SPEC_CLASS_ANDES, 1, 0},
{"zknh",   ISA_SPEC_CLASS_DRAFT, 1, 0},
{"zksed",  ISA_SPEC_CLASS_ANDES, 1, 0},
{"zksed",  ISA_SPEC_CLASS_DRAFT, 1, 0},
{"zksh",   ISA_SPEC_CLASS_ANDES, 1, 0},
{"zksh",   ISA_SPEC_CLASS_DRAFT, 1, 0},
{"zkr",    ISA_SPEC_CLASS_ANDES, 1, 0},
{"zkr",    ISA_SPEC_CLASS_DRAFT, 1, 0},
{"zkn",    ISA_SPEC_CLASS_ANDES, 1, 0},
{"zkn",    ISA_SPEC_CLASS_DRAFT, 1, 0},
{"zks",    ISA_SPEC_CLASS_ANDES, 1, 0},
{"zks",    ISA_SPEC_CLASS_DRAFT, 1, 0},
{"zkt",    ISA_SPEC_CLASS_ANDES, 1, 0},
{"zkt",    ISA_SPEC_CLASS_DRAFT, 1, 0},
{"zk",     ISA_SPEC_CLASS_ANDES, 1, 0},
{"zk",     ISA_SPEC_CLASS_DRAFT, 1, 0},

/* Terminate the list.  */
{NULL, 0, 0, 0}
};

struct isa_spec_t
{
  const char *name;
  enum riscv_isa_spec_class class;
};

/* List for all supported ISA spec versions.  */
static const struct isa_spec_t isa_specs[] =
{
  {"andes",    ISA_SPEC_CLASS_ANDES},
  {"2.2",      ISA_SPEC_CLASS_2P2},
  {"20190608", ISA_SPEC_CLASS_20190608},
  {"20191213", ISA_SPEC_CLASS_20191213},

/* Terminate the list.  */
  {NULL, 0}
};

/* Get the corresponding ISA spec class by giving a ISA spec string.  */

int
riscv_get_isa_spec_class (const char *s,
                         enum riscv_isa_spec_class *class)
{
  const struct isa_spec_t *version;

  if (s == NULL)
    return 0;

  for (version = &isa_specs[0]; version->name != NULL; ++version)
    if (strcmp (version->name, s) == 0)
      {
       *class = version->class;
       return 1;
      }

  /* Can not find the supported ISA spec.  */
  return 0;
}
