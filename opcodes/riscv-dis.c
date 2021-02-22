/* RISC-V disassembler
   Copyright (C) 2011-2019 Free Software Foundation, Inc.

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
#include "disassemble.h"
#include "libiberty.h"
#include "opcode/riscv.h"
#include "opintl.h"
#include "elf-bfd.h"
#include "elf/riscv.h"

#include "bfd_stdint.h"
#include <ctype.h>
#ifndef __MINGW32__
#include <dlfcn.h>
#endif

struct riscv_private_data
{
  bfd_vma gp;
  bfd_vma print_addr;
  bfd_vma hi_addr[OP_MASK_RD + 1];
};

static const char * const *riscv_gpr_names;
static const char * const *riscv_fpr_names;
static const char * const *riscv_vecr_names;

static int
riscv_parse_opcode (bfd_vma, insn_t, disassemble_info *,
		    struct riscv_private_data *, uint32_t);

#define RISCV_PARSE_EXECIT      0x04
#define RISCV_PARSE_EXECIT_TAB  0x08

static void
riscv_execit_info (bfd_vma pc ATTRIBUTE_UNUSED,
		   disassemble_info *info, uint32_t execit_index)
{
  uint32_t insn;
  static asection *section = NULL;
  bfd_byte buffer[4];
  int insnlen;
  struct riscv_private_data *pd = info->private_data;

  /* If no section info can be related to this exec.it insn, this may be just
     a uninitial memory content, so not to decode it.  */
  if (info->section == NULL)
    return;

  /* Lookup section in which itb is located.  */
  if (!section)
    {
      section = bfd_get_section_by_name (info->section->owner, ".exec.itable");

      /* Lookup it only once, in case .exec.itable doesn't exist at all.  */
      if (section == NULL)
	section = (void *) -1;
    }

  if (section == (void *) -1)
    return;

  if (!section->owner)
    return;

  bfd_get_section_contents (section->owner, section, buffer,
			    execit_index * 4, 4);
  insn = bfd_get_32 (section->owner, buffer);
  insnlen = riscv_insn_length (insn);

  /* 16-bit instructions in .exec.itable.  */
  if (insnlen == 2)
    riscv_parse_opcode (pc, (insn & 0x0000FFFF), info, pd, RISCV_PARSE_EXECIT);
  /* 32-bit instructions in .exec.itable.  */
  else
    riscv_parse_opcode (pc, insn, info, pd, RISCV_PARSE_EXECIT);
}

/* Data structures used by ACE */
typedef struct ace_operand
{
  const char *name;  /* operand name */
  int bitpos;  /* operand start position */
  int bitsize;  /* operand width */
  int shift;  /* operand shift amount */
  int hw_res;  /* hardware resource */
  const char *hw_name;  /* hardware/register name */
} ace_op_t;

enum
{
  HW_GPR,
  HW_UINT,
  HW_INT,
  HW_ACR,
  HW_FPR,
  HW_VR
};

/* Pointers for storing symbols from ACE shared library */
struct riscv_opcode *ace_opcs;
ace_op_t *ace_ops;
/* Represent whether ACE shared library is loaded successfully */
bfd_boolean ace_lib_load_success = FALSE;

/* Other options.  */
static int no_aliases;	/* If set disassemble as most general inst.  */
/* Debugging mode:
 * Display ex9 table with ID.
 * Show the ACE insn even if the ACE library is loaded fail.  */
static int debugging;

static void
set_default_riscv_dis_options (void)
{
  riscv_gpr_names = riscv_gpr_names_abi;
  riscv_fpr_names = riscv_fpr_names_abi;
  riscv_vecr_names = riscv_vecr_names_numeric;
  no_aliases = 0;
  debugging = 0;
}

static void
parse_riscv_dis_option (const char *option)
{
  if (strcmp (option, "debugging") == 0)
    debugging = 1;
  else if (strcmp (option, "no-aliases") == 0)
    no_aliases = 1;
  else if (strcmp (option, "numeric") == 0)
    {
      riscv_gpr_names = riscv_gpr_names_numeric;
      riscv_fpr_names = riscv_fpr_names_numeric;
    }
  else if (strcmp (option, "standard") == 0)
    {
      riscv_gpr_names = riscv_gpr_names_standard;
      riscv_fpr_names = riscv_fpr_names_abi;
    }
  /* Load ACE shared library if ACE option is enable */
  else if (strncmp (option, "ace=", 4) == 0)
    {
#ifndef __MINGW32__
      char *ace_lib_path = malloc (strlen (option) - 4);
      strcpy (ace_lib_path, option + 4);

      void *dlc = dlopen (ace_lib_path, RTLD_NOW | RTLD_LOCAL);
      char *err;

      if (dlc == NULL)
	err = (char *) dlerror ();
      else
	{
	  ace_ops = (ace_op_t *) dlsym (dlc, "ace_operands");
	  err = (char *) dlerror ();
	  if (err == NULL)
	    {
	      ace_opcs = (struct riscv_opcode *) dlsym (dlc, "ace_opcodes_2");
	      err = (char *) dlerror ();
	    }
	}

      if (err == NULL)
	ace_lib_load_success = TRUE;
      else
	fprintf (stderr, _("Fault to load ACE shared library: %s\n"), err);
#endif
    }
  else
    {
      /* xgettext:c-format */
      opcodes_error_handler (_("unrecognized disassembler option: %s"), option);
    }
}

static void
parse_riscv_dis_options (const char *opts_in)
{
  char *opts = xstrdup (opts_in), *opt = opts, *opt_end = opts;

  set_default_riscv_dis_options ();

  for ( ; opt_end != NULL; opt = opt_end + 1)
    {
      if ((opt_end = strchr (opt, ',')) != NULL)
	*opt_end = 0;
      parse_riscv_dis_option (opt);
    }

  free (opts);
}

/* Print one argument from an array.  */

static void
arg_print (struct disassemble_info *info, unsigned long val,
	   const char* const* array, size_t size)
{
  const char *s = val >= size || array[val] == NULL ? "unknown" : array[val];
  (*info->fprintf_func) (info->stream, "%s", s);
}

static void
maybe_print_address (struct riscv_private_data *pd, int base_reg, int offset)
{
  if (pd->hi_addr[base_reg] != (bfd_vma)-1)
    {
      pd->print_addr = (base_reg != 0 ? pd->hi_addr[base_reg] : 0) + offset;
      pd->hi_addr[base_reg] = -1;
    }
  else if (base_reg == X_GP && pd->gp != (bfd_vma)-1)
    pd->print_addr = pd->gp + offset;
  else if (base_reg == X_TP || base_reg == 0)
    pd->print_addr = offset;
}

static unsigned int
ace_get_discrete_bit_value(unsigned int bit_value, char *op_name_discrete, const char *op)
{
  bfd_boolean found_or_token = TRUE;
  unsigned val, ret = 0;
  char *psep, *pval = op_name_discrete + strlen(op);
  unsigned msb = 0, width = 0, width_acc = 0;

  while (found_or_token)
    {
      /* Extract msb from string */
      psep = strchr (pval, '_');
      *psep = '\0';
      msb = strtoul (pval, (char **) NULL, 10);
      /* Extract width from string */
      pval = psep + 1;
      psep = strchr (pval, '|');
      if (psep)
	*psep = '\0';
      else
	found_or_token = FALSE;
      width = strtoul (pval, (char **) NULL, 10);

      /* Perform mask to truncate oversize value */
      val = bit_value << (32 - msb - 1);
      val >>= 32 - width;
      val <<= width_acc;
      ret |= val;
      width_acc += width;

      /* Prepare condition for next iteration */
      pval = psep + 1;
    }
  return ret;
}

/* Print out ACE instruction assembly code */

static void
print_ace_args (const char **args, insn_t l, disassemble_info * info)
{
  fprintf_ftype print = info->fprintf_func;

  /* Extract field attribute name from opcode description (ace_ops) and
     store the extracted result to var of op_name for finding the
     field attribute information from ace_field_hash */
  bfd_boolean found_op_str_end = FALSE;
  char *pch = strchr (*args, ',');
  if (pch == NULL)
    {
      pch = strchr (*args, '\0');
      found_op_str_end = TRUE;
    }
  if (pch == NULL)
    return;

  unsigned int op_name_size = pch - (*args + 1);
  char *op_name = malloc (op_name_size + 1);
  memcpy (op_name, *args + 1, op_name_size);
  /* Cat null character to the end of op_name to avoid gash */
  memcpy (op_name + op_name_size, "\0", 1);

  /* With rGPR encoding format, operand bit-field may be discrete.
     There is an "|" token in discrete format */
  bfd_boolean is_discrete = FALSE;
  char *por = strchr(op_name, '|');
  char *op_name_discrete;
  if (por != NULL)
    {
      is_discrete = TRUE;
      op_name_discrete = malloc(op_name_size + 1);
      strcpy(op_name_discrete, op_name);
      *por = '\0';
    }

  /*  Find the field attribute from ace_field_hash and encode instruction */
  ace_op_t *ace_op = NULL;
  unsigned int i = 0;
  while (ace_ops[i].name)
    {
      if (strcmp (ace_ops[i].name, op_name) == 0)
	{
	  ace_op = &ace_ops[i];
	  break;
	}
      i++;
    }

  if (ace_op != NULL)
    {
      /* Extract the value from defined location */
      unsigned int bit_value = l;
      bit_value <<= 32 - (ace_op->bitpos + 1);
      bit_value >>= 32 - ace_op->bitsize;

      switch (ace_op->hw_res)
	{
	case HW_GPR:
	  print (info->stream, "%s", riscv_gpr_names[bit_value]);
	  break;

	case HW_FPR:
	  print (info->stream, "%s", riscv_fpr_names[bit_value]);
	  break;

	case HW_VR:
	  print (info->stream, "%s", riscv_vecr_names[bit_value]);
	  break;

	case HW_UINT:
	  if (is_discrete)
	    bit_value = ace_get_discrete_bit_value(l, op_name_discrete, "imm");
	  print (info->stream, "%d", bit_value);
	  break;

	case HW_ACR:
	  if (is_discrete)
	    bit_value = ace_get_discrete_bit_value(l, op_name_discrete, ace_op->hw_name);
	  print (info->stream, "%s_%d", ace_op->hw_name, bit_value);
	  break;
	}
    }
  else
    {
      fprintf (stderr, _("ace_op is NULL\n"));
      return;
    }

  /* Update the address of pointer of the field attribute (*args) */
  if (found_op_str_end == TRUE)
    *args = pch - 1;
  else
    {
      *args = pch;
      print (info->stream, ",");
    }
}

#define MAX_KEYWORD_LEN 32

/* Parse the field defined for nds v5 extension.  */

static bfd_boolean
parse_nds_v5_field (const char **str, char name[MAX_KEYWORD_LEN])
{
  char *p = name;
  const char *str_t;

  str_t = *str;
  str_t--;
  while (isalnum (*str_t) || *str_t == '.' || *str_t == '_')
    *p++ = *str_t++;
  *p = '\0';

  if (strncmp (name, "nds_", 4) == 0)
    {
      *str = str_t;
      return TRUE;
    }
  else
    return FALSE;
}

/* Print insn arguments for 32/64-bit code.  */

static void
print_insn_args (const char *d, insn_t l, bfd_vma pc,
		 disassemble_info *info, uint32_t parse_mode)
{
  struct riscv_private_data *pd = info->private_data;
  int rs1 = (l >> OP_SH_RS1) & OP_MASK_RS1;
  int rd = (l >> OP_SH_RD) & OP_MASK_RD;
  fprintf_ftype print = info->fprintf_func;

  if (*d != '\0')
    print (info->stream, "\t");

  for (; *d != '\0'; d++)
    {
      switch (*d)
	{
	case 'C': /* RVC */
	  switch (*++d)
	    {
	    case 's': /* RS1 x8-x15 */
	    case 'w': /* RS1 x8-x15 */
	      print (info->stream, "%s",
		     riscv_gpr_names[EXTRACT_OPERAND (CRS1S, l) + 8]);
	      break;
	    case 't': /* RS2 x8-x15 */
	    case 'x': /* RS2 x8-x15 */
	      print (info->stream, "%s",
		     riscv_gpr_names[EXTRACT_OPERAND (CRS2S, l) + 8]);
	      break;
	    case 'U': /* RS1, constrained to equal RD */
	      print (info->stream, "%s", riscv_gpr_names[rd]);
	      break;
	    case 'c': /* RS1, constrained to equal sp */
	      print (info->stream, "%s", riscv_gpr_names[X_SP]);
	      break;
	    case 'V': /* RS2 */
	      print (info->stream, "%s",
		     riscv_gpr_names[EXTRACT_OPERAND (CRS2, l)]);
	      break;
	    case 'i':
	      print (info->stream, "%d", (int)EXTRACT_RVC_SIMM3 (l));
	      break;
	    case 'o':
	    case 'j':
	      print (info->stream, "%d", (int)EXTRACT_RVC_IMM (l));
	      break;
	    case 'e':
	      switch (*++d)
		{
		case 'i':
		  print (info->stream, "#%d	!", (int)EXTRACT_RVC_EX9IT_IMM (l) >> 2);
		  riscv_execit_info (pc, info, (int)EXTRACT_RVC_EX9IT_IMM (l) >> 2);
		  break;
		case 't':
		  print (info->stream, "#%d     !", (int)EXTRACT_RVC_EXECIT_IMM (l) >> 2);
		  riscv_execit_info (pc, info, (int)EXTRACT_RVC_EXECIT_IMM (l) >> 2);
		  break;
		}
	      break;
	    case 'k':
	      print (info->stream, "%d", (int)EXTRACT_RVC_LW_IMM (l));
	      break;
	    case 'l':
	      print (info->stream, "%d", (int)EXTRACT_RVC_LD_IMM (l));
	      break;
	    case 'm':
	      print (info->stream, "%d", (int)EXTRACT_RVC_LWSP_IMM (l));
	      break;
	    case 'n':
	      print (info->stream, "%d", (int)EXTRACT_RVC_LDSP_IMM (l));
	      break;
	    case 'K':
	      print (info->stream, "%d", (int)EXTRACT_RVC_ADDI4SPN_IMM (l));
	      break;
	    case 'L':
	      print (info->stream, "%d", (int)EXTRACT_RVC_ADDI16SP_IMM (l));
	      break;
	    case 'M':
	      print (info->stream, "%d", (int)EXTRACT_RVC_SWSP_IMM (l));
	      break;
	    case 'N':
	      print (info->stream, "%d", (int)EXTRACT_RVC_SDSP_IMM (l));
	      break;
	    case 'p':
	      info->target = EXTRACT_RVC_B_IMM (l) + pc;
	      (*info->print_address_func) (info->target, info);
	      break;
	    case 'a':
	      info->target = EXTRACT_RVC_J_IMM (l) + pc;
	      (*info->print_address_func) (info->target, info);
	      break;
	    case 'u':
	      print (info->stream, "0x%x",
		     (int)(EXTRACT_RVC_IMM (l) & (RISCV_BIGIMM_REACH-1)));
	      break;
	    case '>':
	      print (info->stream, "0x%x", (int)EXTRACT_RVC_IMM (l) & 0x3f);
	      break;
	    case '<':
	      print (info->stream, "0x%x", (int)EXTRACT_RVC_IMM (l) & 0x1f);
	      break;
	    case 'T': /* floating-point RS2 */
	      print (info->stream, "%s",
		     riscv_fpr_names[EXTRACT_OPERAND (CRS2, l)]);
	      break;
	    case 'D': /* floating-point RS2 x8-x15 */
	      print (info->stream, "%s",
		     riscv_fpr_names[EXTRACT_OPERAND (CRS2S, l) + 8]);
	      break;
	    }
	  break;

	case ',':
	case '(':
	case ')':
	case '[':
	case ']':
	case '+':
	  print (info->stream, "%c", *d);
	  break;

	case '0':
	  /* Only print constant 0 if it is the last argument */
	  if (!d[1])
	    print (info->stream, "0");
	  break;

	case 'b':
	case 's':
	  if ((l & MASK_JALR) == MATCH_JALR)
	    maybe_print_address (pd, rs1, 0);
	  print (info->stream, "%s", riscv_gpr_names[rs1]);
	  break;

	case 't':
	case 'e':
	  print (info->stream, "%s",
		 riscv_gpr_names[EXTRACT_OPERAND (RS2, l)]);
	  break;

	case 'u':
	  print (info->stream, "0x%x",
		 (unsigned)EXTRACT_UTYPE_IMM (l) >> RISCV_IMM_BITS);
	  break;

	case 'm':
	  arg_print (info, EXTRACT_OPERAND (RM, l),
		     riscv_rm, ARRAY_SIZE (riscv_rm));
	  break;

	case 'P':
	  arg_print (info, EXTRACT_OPERAND (PRED, l),
		     riscv_pred_succ, ARRAY_SIZE (riscv_pred_succ));
	  break;

	case 'Q':
	  arg_print (info, EXTRACT_OPERAND (SUCC, l),
		     riscv_pred_succ, ARRAY_SIZE (riscv_pred_succ));
	  break;

	case 'o':
	  maybe_print_address (pd, rs1, EXTRACT_ITYPE_IMM (l));
	  /* Fall through.  */
	case 'j':
	  if (((l & MASK_ADDI) == MATCH_ADDI && rs1 != 0)
	      || (l & MASK_JALR) == MATCH_JALR)
	    maybe_print_address (pd, rs1, EXTRACT_ITYPE_IMM (l));
	  print (info->stream, "%d", (int)EXTRACT_ITYPE_IMM (l));
	  break;

	case 'q':
	  maybe_print_address (pd, rs1, EXTRACT_STYPE_IMM (l));
	  print (info->stream, "%d", (int)EXTRACT_STYPE_IMM (l));
	  break;

	case 'a':
	  if (parse_mode & RISCV_PARSE_EXECIT)
	    {
	      /* Check instruction in .exec.itable.  */
	      info->target = EXTRACT_UJTYPE_IMM_EXECIT_TAB (l);
	      info->target |= (pc & 0xffe00000);
	      (*info->print_address_func) (info->target, info);
	    }
	  else if (parse_mode & RISCV_PARSE_EXECIT_TAB)
	    {
	      /* Check if decode .exec.itable.  */
	      info->target = EXTRACT_UJTYPE_IMM_EXECIT_TAB (l);
	      print (info->stream, "PC(31,21)|#0x%lx", (long) info->target);
	    }
	  else
	    {
	      info->target = EXTRACT_UJTYPE_IMM (l) + pc;
	      (*info->print_address_func) (info->target, info);
	    }
	  break;

	case 'p':
	  info->target = EXTRACT_SBTYPE_IMM (l) + pc;
	  (*info->print_address_func) (info->target, info);
	  break;

	case 'd':
	  if ((l & MASK_AUIPC) == MATCH_AUIPC)
	    pd->hi_addr[rd] = pc + EXTRACT_UTYPE_IMM (l);
	  else if ((l & MASK_LUI) == MATCH_LUI)
	    pd->hi_addr[rd] = EXTRACT_UTYPE_IMM (l);
	  else if ((l & MASK_C_LUI) == MATCH_C_LUI)
	    pd->hi_addr[rd] = EXTRACT_RVC_LUI_IMM (l);
	  print (info->stream, "%s", riscv_gpr_names[rd]);
	  break;

	case 'z':
	  print (info->stream, "%s", riscv_gpr_names[0]);
	  break;

	case '>':
	  print (info->stream, "0x%x", (int)EXTRACT_OPERAND (SHAMT, l));
	  break;

	case '<':
	  print (info->stream, "0x%x", (int)EXTRACT_OPERAND (SHAMTW, l));
	  break;

	case 'S':
	case 'U':
	  print (info->stream, "%s", riscv_fpr_names[rs1]);
	  break;

	case 'T':
	  print (info->stream, "%s", riscv_fpr_names[EXTRACT_OPERAND (RS2, l)]);
	  break;

	case 'D':
	  print (info->stream, "%s", riscv_fpr_names[rd]);
	  break;

	case 'R':
	  print (info->stream, "%s", riscv_fpr_names[EXTRACT_OPERAND (RS3, l)]);
	  break;

	case 'E':
	  {
	    const char* csr_name = NULL;
	    unsigned int csr = EXTRACT_OPERAND (CSR, l);
	    switch (csr)
	      {
#define DECLARE_CSR(name, num, class) case num: csr_name = #name; break;
#include "opcode/riscv-opc.h"
#undef DECLARE_CSR
	      }
	    if (csr_name)
	      print (info->stream, "%s", csr_name);
	    else
	      print (info->stream, "0x%x", csr);
	    break;
	  }

	case 'Z':
	  print (info->stream, "%d", rs1);
	  break;

	case 'V': /* RVV */
	  switch (*++d)
	    {
	    case 'd':
	    case 'f':
	      print (info->stream, "%s",
		      riscv_vecr_names[EXTRACT_OPERAND (VD, l)]);
	      break;

	    case 'e':
	      if (!EXTRACT_OPERAND (VWD, l))
		print (info->stream, "%s", riscv_gpr_names[0]);
	      else
		print (info->stream, "%s",
		       riscv_vecr_names[EXTRACT_OPERAND (VD, l)]);
	      break;

	    case 's':
	      print (info->stream, "%s",
		      riscv_vecr_names[EXTRACT_OPERAND (VS1, l)]);
	      break;

	    case 't':
	    case 'u': /* VS1 == VS2 already verified at this point.  */
	    case 'v': /* VD == VS1 == VS2 already verified at this point.  */
	      print (info->stream, "%s",
		      riscv_vecr_names[EXTRACT_OPERAND (VS2, l)]);
	      break;

	    case '0':
	      print (info->stream, "%s", riscv_vecr_names[0]);
	      break;

	    case 'b':
	    case 'c':
	      {
		int imm = (*d == 'b') ? EXTRACT_RVV_VB_IMM (l)
				      : EXTRACT_RVV_VC_IMM (l);
		unsigned int imm_vlmul = EXTRACT_OPERAND (VLMUL, imm);
		unsigned int imm_vsew = EXTRACT_OPERAND (VSEW, imm);
		unsigned int imm_vediv = EXTRACT_OPERAND (VEDIV, imm);
		unsigned int imm_vta = EXTRACT_OPERAND (VTA, imm);
		unsigned int imm_vma = EXTRACT_OPERAND (VMA, imm);
		unsigned int imm_vtype_res = EXTRACT_OPERAND (VTYPE_RES, imm);

		if (imm_vsew < ARRAY_SIZE (riscv_vsew)
		    && imm_vlmul < ARRAY_SIZE (riscv_vlmul)
		    && imm_vediv < ARRAY_SIZE (riscv_vediv)
		    && imm_vta < ARRAY_SIZE (riscv_vta)
		    && imm_vma < ARRAY_SIZE (riscv_vma)
		    && ! imm_vtype_res)
		  print (info->stream, "%s,%s,%s,%s,%s", riscv_vsew[imm_vsew],
			 riscv_vlmul[imm_vlmul], riscv_vta[imm_vta],
			 riscv_vma[imm_vma], riscv_vediv[imm_vediv]);
		else
		  print (info->stream, "%d", imm);
	      }
	      break;

	    case 'i':
	      print (info->stream, "%d", (int)EXTRACT_RVV_VI_IMM (l));
	      break;

	    case 'j':
	      print (info->stream, "%d", (int)EXTRACT_RVV_VI_UIMM (l));
	      break;

	    case 'k':
	      print (info->stream, "%d", (int)EXTRACT_RVV_OFFSET (l));
	      break;

	    case 'm':
	      if (! EXTRACT_OPERAND (VMASK, l))
		print (info->stream, ",%s", riscv_vecm_names_numeric[0]);
	      break;

	    default:
	      /* xgettext:c-format */
	      print (info->stream, _("# internal error, undefined modifier (V%c)"),
		     *d);
	      return;
	    }
	  break;

	/* Handle ACE operand field */
	case 'X':
	  if (ace_lib_load_success)
	    {
	      print_ace_args (&d, l, info);
	      break;
	    }
	  else
	    {
	      print (info->stream,
		     _("# ACE shared library is not loaded successfully"));
	      return;
	    }

	case 'h':
	  print (info->stream, "%d", (int)EXTRACT_ITYPE_IMM6H (l));
	  break;

	case 'l':
	  print (info->stream, "%d", (int)EXTRACT_ITYPE_IMM6L (l));
	  break;

	case 'i':
	  print (info->stream, "%d", (int)EXTRACT_STYPE_IMM7 (l));
	  break;

	case 'k':
	  print (info->stream, "%d", (int)EXTRACT_TYPE_CIMM6 (l));
	  break;

	case 'f':
	  print (info->stream, "%d", (int)EXTRACT_TYPE_IMM8 (l));
	  break;

	case 'r':
	  print (info->stream, "%d", (int)EXTRACT_TYPE_SIMM8 (l));
	  break;

	case 'g':
	  info->target = EXTRACT_STYPE_IMM10 (l) + pc;
	  (*info->print_address_func) (info->target, info);
	  break;

	case 'v':
	  print (info->stream, "<<0x%x", (int)EXTRACT_OPERAND (SV, l));
	  break;

	case 'G':
	  switch (*++d)
	    {
	    case 'b':
	      maybe_print_address (pd, X_GP, EXTRACT_GPTYPE_LB_IMM (l));
	      print (info->stream, "%d", (int)EXTRACT_GPTYPE_LB_IMM (l));
	      break;
	    case 'h':
	      maybe_print_address (pd, X_GP, EXTRACT_GPTYPE_LH_IMM (l));
	      print (info->stream, "%d", (int)EXTRACT_GPTYPE_LH_IMM (l));
	      break;
	    case 'w':
	      maybe_print_address (pd, X_GP, EXTRACT_GPTYPE_LW_IMM (l));
	      print (info->stream, "%d", (int)EXTRACT_GPTYPE_LW_IMM (l));
	      break;
	    case 'd':
	      maybe_print_address (pd, X_GP, EXTRACT_GPTYPE_LD_IMM (l));
	      print (info->stream, "%d", (int)EXTRACT_GPTYPE_LD_IMM (l));
	      break;
	    }
	  break;

	case 'H':
	  switch (*++d)
	    {
	    case 'b':
	      maybe_print_address (pd, X_GP, EXTRACT_GPTYPE_SB_IMM (l));
	      print (info->stream, "%d", (int)EXTRACT_GPTYPE_SB_IMM (l));
	      break;
	    case 'h':
	      maybe_print_address (pd, X_GP, EXTRACT_GPTYPE_SH_IMM (l));
	      print (info->stream, "%d", (int)EXTRACT_GPTYPE_SH_IMM (l));
	      break;
	    case 'w':
	      maybe_print_address (pd, X_GP, EXTRACT_GPTYPE_SW_IMM (l));
	      print (info->stream, "%d", (int)EXTRACT_GPTYPE_SW_IMM (l));
	      break;
	    case 'd':
	      maybe_print_address (pd, X_GP, EXTRACT_GPTYPE_SD_IMM (l));
	      print (info->stream, "%d", (int)EXTRACT_GPTYPE_SD_IMM (l));
	      break;
	    }
	  break;

	/* Handle operand fields of V5 extension.  */
	case 'n':
	  {
	    d++;
	    char field_name[MAX_KEYWORD_LEN];
	    if (parse_nds_v5_field (&d, field_name))
	      {
		if (strcmp (field_name, "nds_rc") == 0)
		  print (info->stream, "%s",
			 riscv_gpr_names[EXTRACT_OPERAND (RC, l)]);
		else if (strcmp (field_name, "nds_rdp") == 0)
		  print (info->stream, "%s", riscv_gpr_names[rd]);
		else if (strcmp (field_name, "nds_rsp") == 0)
		  print (info->stream, "%s", riscv_gpr_names[rs1]);
		else if (strcmp (field_name, "nds_rtp") == 0)
		  print (info->stream, "%s",
			 riscv_gpr_names[EXTRACT_OPERAND (RS2, l)]);
		else if (strcmp (field_name, "nds_i3u") == 0)
		  print (info->stream, "%d", (int)EXTRACT_PTYPE_IMM3U (l));
		else if (strcmp (field_name, "nds_i4u") == 0)
		  print (info->stream, "%d", (int)EXTRACT_PTYPE_IMM4U (l));
		else if (strcmp (field_name, "nds_i5u") == 0)
		  print (info->stream, "%d", (int)EXTRACT_PTYPE_IMM5U (l));
		else if (strcmp (field_name, "nds_i6u") == 0)
		  print (info->stream, "%d", (int)EXTRACT_PTYPE_IMM6U (l));
		else if (strcmp (field_name, "nds_i15s") == 0)
		  print (info->stream, "%d", (int)EXTRACT_PTYPE_IMM15S (l));
		else
		  print (info->stream,
			 _("# internal error, undefined nds v5 field (%s)"),
			 field_name);
	      }
	    d--;
	  }
	  break;

	default:
	  /* xgettext:c-format */
	  print (info->stream, _("# internal error, undefined modifier (%c)"),
		 *d);
	  return;
	}
    }
}

static const struct riscv_opcode *riscv_hash[OP_MASK_OP + 1];

static int
riscv_parse_opcode (bfd_vma memaddr, insn_t word, disassemble_info *info,
		    struct riscv_private_data *pd, uint32_t parse_mode)
{
  const struct riscv_opcode *op;

#define OP_HASH_IDX(i) ((i) & (riscv_insn_length (i) == 2 ? 0x3 : OP_MASK_OP))

  op = riscv_hash[OP_HASH_IDX (word)];
  if (op != NULL)
    {
      int xlen = 0;

      /* If XLEN is not known, get its value from the ELF class.  */
      if (info->mach == bfd_mach_riscv64)
	xlen = 64;
      else if (info->mach == bfd_mach_riscv32)
	xlen = 32;
      else if (info->section != NULL)
	{
	  Elf_Internal_Ehdr *ehdr = elf_elfheader (info->section->owner);
	  xlen = ehdr->e_ident[EI_CLASS] == ELFCLASS64 ? 64 : 32;
	}

      for (; op->name; op++)
	{
	  /* Does the opcode match?  */
	  if (! (op->match_func) (op, word, 0, NULL))
	    continue;
	  /* Is this a pseudo-instruction and may we print it as such?  */
	  if (no_aliases && (op->pinfo & INSN_ALIAS))
	    continue;
	  /* Is this instruction restricted to a certain value of XLEN?  */
	  if (isdigit (op->subset[0][0]) && atoi (op->subset[0]) != xlen)
	    continue;

	  /* It's a match.  */
	  (*info->fprintf_func) (info->stream, "%s", op->name);
	  print_insn_args (op->args, word, memaddr, info, parse_mode);

	  /* Try to disassemble multi-instruction addressing sequences.  */
	  if (pd->print_addr != (bfd_vma)-1)
	    {
	      info->target = pd->print_addr;
	      (*info->fprintf_func) (info->stream, " # ");
	      (*info->print_address_func) (info->target, info);
	      pd->print_addr = -1;
	    }

	  return 1;
	}
    }
  return 0;
}

/* get architecture attributes from input BFD to test if V + XV5  */

static bfd_boolean
has_extension (const char *ext, disassemble_info *info)
{
  bfd_boolean has = FALSE;
  obj_attribute *attr = NULL;

  if (info && info->section && info->section->owner)
    attr = &elf_known_obj_attributes (info->section->owner)[OBJ_ATTR_PROC][Tag_RISCV_arch];

  if (attr && attr->s)
    {
      int len = strlen (ext);
      const char *p = attr->s;
      if ((tolower(p[0]) == 'r') &&
	  (tolower(p[1]) == 'v') &&
	  (isdigit(p[2])) &&
	  (isdigit(p[3])))
	{
	  p += 4;
	  while (*p)
	    {
	      if (strncasecmp(p, ext, len) == 0)
		{
		  has = TRUE;
		  break;
		}
	      ++p;;
	    }
	}
    }

  return has;
}

/* Print the RISC-V instruction at address MEMADDR in debugged memory,
   on using INFO.  Returns length of the instruction, in bytes.
   BIGENDIAN must be 1 if this is big-endian code, 0 if
   this is little-endian code.  */

static int
riscv_disassemble_insn (bfd_vma memaddr, insn_t word, disassemble_info *info)
{
  const struct riscv_opcode *op;
  static bfd_boolean init = 0;
  struct riscv_private_data *pd;
  int insnlen;
  int match;
  static int execit_id = 0;
  static struct riscv_opcode reordered_op[] =
    {
      {0, 0, {0}, 0, 0, 1, 0, 0},
      {0, 0, {0}, 0, 0, 1, 0, 0},
      {0, 0, {0}, 0, 0, 1, 0, 0},
    };

#define OP_HASH_IDX(i) ((i) & (riscv_insn_length (i) == 2 ? 0x3 : OP_MASK_OP))

  /* Build a hash table to shorten the search time.  */
  if (!init)
    {
      bfd_boolean has_xefhw = has_extension ("xefhw", info);
      bfd_boolean has_v = has_extension ("v", info);
      for (op = riscv_opcodes; op->name; op++)
	if (!riscv_hash[OP_HASH_IDX (op->match)])
	  {
	    /* favor V extension than Xefhw one.  */
	    if ((has_v || !has_xefhw) && (op->mask == 0x707f))
	      if (!strcmp(op->name, "flhw") || !strcmp(op->name, "fshw"))
		{
		  reordered_op[init++] = *op;
		  continue;
		}
	    riscv_hash[OP_HASH_IDX (op->match)] = op;
	  }

      /* Insert ACE opcode attributes into hash table if exist */
      if (ace_lib_load_success && ace_opcs != NULL && ace_ops != NULL)
	{
	  for (op = ace_opcs; op->name; op++)
	    if (!riscv_hash[OP_HASH_IDX (op->match)])
	      riscv_hash[OP_HASH_IDX (op->match)] = op;
	}

      init = 1;
    }

  if (info->private_data == NULL)
    {
      int i;

      pd = info->private_data = xcalloc (1, sizeof (struct riscv_private_data));
      pd->gp = -1;
      pd->print_addr = -1;
      for (i = 0; i < (int)ARRAY_SIZE (pd->hi_addr); i++)
	pd->hi_addr[i] = -1;

      for (i = 0; i < info->symtab_size; i++)
	if (strcmp (bfd_asymbol_name (info->symtab[i]), RISCV_GP_SYMBOL) == 0)
	  pd->gp = bfd_asymbol_value (info->symtab[i]);
    }
  else
    pd = info->private_data;

  insnlen = riscv_insn_length (word);

  info->bytes_per_chunk = insnlen % 4 == 0 ? 4 : 2;
  info->bytes_per_line = 8;
  info->display_endian = info->endian;
  info->insn_info_valid = 1;
  info->branch_delay_insns = 0;
  info->data_size = 0;
  info->insn_type = dis_nonbranch;
  info->target = 0;
  info->target2 = 0;

  op = riscv_hash[OP_HASH_IDX (word)];
  if (op != NULL)
    {
      unsigned xlen = 0;

      /* If XLEN is not known, get its value from the ELF class.  */
      if (info->mach == bfd_mach_riscv64)
	xlen = 64;
      else if (info->mach == bfd_mach_riscv32)
	xlen = 32;
      else if (info->section != NULL)
	{
	  Elf_Internal_Ehdr *ehdr = elf_elfheader (info->section->owner);
	  xlen = ehdr->e_ident[EI_CLASS] == ELFCLASS64 ? 64 : 32;
	}

      for (; op->name; op++)
	{
	  /* Does the opcode match?  */
	  if (! (op->match_func) (op, word, 0, NULL))
	    {
	      if (!op[1].name && !op[1].mask && reordered_op[0].name)
	        op = reordered_op - 1;
	      continue;
	    }
	  /* Is this a pseudo-instruction and may we print it as such?  */
	  if (no_aliases && (op->pinfo & INSN_ALIAS))
	    continue;
	  /* Is this instruction restricted to a certain value of XLEN?  */
	  if ((op->xlen_requirement != 0) && (op->xlen_requirement != xlen))
	    continue;

	  /* It's a match.  */
	  (*info->fprintf_func) (info->stream, "%s", op->name);
	  print_insn_args (op->args, word, memaddr, info, 0);

	  /* Try to disassemble multi-instruction addressing sequences.  */
	  if (pd->print_addr != (bfd_vma)-1)
	    {
	      info->target = pd->print_addr;
	      (*info->fprintf_func) (info->stream, " # ");
	      (*info->print_address_func) (info->target, info);
	      pd->print_addr = -1;
	    }

	  /* Finish filling out insn_info fields.  */
	  switch (op->pinfo & INSN_TYPE)
	    {
	    case INSN_BRANCH:
	      info->insn_type = dis_branch;
	      break;
	    case INSN_CONDBRANCH:
	      info->insn_type = dis_condbranch;
	      break;
	    case INSN_JSR:
	      info->insn_type = dis_jsr;
	      break;
	    case INSN_DREF:
	      info->insn_type = dis_dref;
	      break;
	    default:
	      break;
	    }

	  if (op->pinfo & INSN_DATA_SIZE)
	    {
	      int size = ((op->pinfo & INSN_DATA_SIZE)
			  >> INSN_DATA_SIZE_SHIFT);
	      info->data_size = 1 << (size - 1);
	    }

	  return insnlen;
	}
    }

  if (info->section
      && strstr (info->section->name, ".exec.itable") != NULL)
    {
      match = riscv_parse_opcode (memaddr, word, info, pd, RISCV_PARSE_EXECIT_TAB);
      if (debugging)
	(*info->fprintf_func) (info->stream, "\t/* %d */", execit_id++);
    }
  else
    match = riscv_parse_opcode (memaddr, word, info, pd, 0);

  if (!match)
    {
      /* We did not find a match above. */
      /* It may be ACE insn but the ACE shared library is
	 failed to load.  */
      if (debugging
	  && !ace_lib_load_success
	  && (word & 0x7f) == 0x7b)
	{
	  info->insn_type = dis_noninsn;
	  (*info->fprintf_func) (info->stream,
				 "ACE insn (0x%llx)",
				 (unsigned long long)word);
	}
      else
	{
	  /* Just print the instruction bits.  */
	  info->insn_type = dis_noninsn;
	  (*info->fprintf_func) (info->stream, "0x%llx", (unsigned long long)word);
	}
    }

  return insnlen;
}

int
print_insn_riscv (bfd_vma memaddr, struct disassemble_info *info)
{
  bfd_byte packet[2];
  insn_t insn = 0;
  bfd_vma n;
  int status;

  if (info->disassembler_options != NULL)
    {
      parse_riscv_dis_options (info->disassembler_options);
      /* Avoid repeatedly parsing the options.  */
      info->disassembler_options = NULL;
    }
  else if (riscv_gpr_names == NULL)
    set_default_riscv_dis_options ();

  /* Instructions are a sequence of 2-byte packets in little-endian order.  */
  for (n = 0; n < sizeof (insn) && n < riscv_insn_length (insn); n += 2)
    {
      status = (*info->read_memory_func) (memaddr + n, packet, 2, info);
      if (status != 0)
	{
	  /* Don't fail just because we fell off the end.  */
	  if (n > 0)
	    break;
	  (*info->memory_error_func) (status, memaddr, info);
	  return status;
	}

      insn |= ((insn_t) bfd_getl16 (packet)) << (8 * n);
    }

  return riscv_disassemble_insn (memaddr, insn, info);
}

/* Prevent use of the fake labels that are generated as part of the DWARF
   and for relaxable relocations in the assembler.  */

bfd_boolean
riscv_symbol_is_valid (asymbol * sym,
                       struct disassemble_info * info ATTRIBUTE_UNUSED)
{
  const char * name;

  if (sym == NULL)
    return FALSE;

  name = bfd_asymbol_name (sym);

  return (strcmp (name, RISCV_FAKE_LABEL_NAME) != 0);
}

void
print_riscv_disassembler_options (FILE *stream)
{
  fprintf (stream, _("\n\
The following RISC-V-specific disassembler options are supported for use\n\
with the -M switch (multiple options should be separated by commas):\n"));

  fprintf (stream, _("\n\
  numeric       Print numeric register names, rather than ABI names.\n"));

  fprintf (stream, _("\n\
  standard      Print standard reigster names.\n"));

  fprintf (stream, _("\n\
  no-aliases    Disassemble only into canonical instructions, rather\n\
                than into pseudoinstructions.\n"));

  fprintf (stream, _("\n"));
}
