/* tc-riscv.c -- RISC-V assembler
   Copyright (C) 2011-2019 Free Software Foundation, Inc.

   Contributed by Andrew Waterman (andrew@sifive.com).
   Based on MIPS target.

   This file is part of GAS.

   GAS is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3, or (at your option)
   any later version.

   GAS is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; see the file COPYING3. If not,
   see <http://www.gnu.org/licenses/>.  */

#include "as.h"
#include "config.h"
#include "subsegs.h"
#include "safe-ctype.h"

#include "itbl-ops.h"
#include "dwarf2dbg.h"
#include "dw2gencfi.h"

#include "bfd/elfxx-riscv.h"
#include "elf/riscv.h"
#include "opcode/riscv.h"

#include <stdint.h>

#ifndef __MINGW32__
#include <dlfcn.h>
#endif

/* workaround of gas/hash changes  */
#define str_htab_create hash_new
#define str_hash_insert(htab, key, value, replace) hash_insert(htab, key, value)
#define str_hash_find  hash_find
typedef struct hash_control * htab_t;

static void arch_sanity_check (int is_final);
extern int opc_set_no_vic (int is);

/* Information about an instruction, including its format, operands
   and fixups.  */
struct riscv_cl_insn
{
  /* The opcode's entry in riscv_opcodes.  */
  const struct riscv_opcode *insn_mo;

  /* The encoded instruction bits.  */
  insn_t insn_opcode;

  /* The frag that contains the instruction.  */
  struct frag *frag;

  /* The offset into FRAG of the first instruction byte.  */
  long where;

  /* The relocs associated with the instruction, if any.  */
  fixS *fixp;

  /* The cmodel parameters.  */
  struct
    {
      int method;
      int state;
      int type;
      int index;
      int offset;
    } cmodel;
};

#ifndef DEFAULT_ARCH
#define DEFAULT_ARCH "riscv64"
#endif

#ifndef DEFAULT_RISCV_ATTR
#define DEFAULT_RISCV_ATTR 0
#endif

/* Let riscv_after_parse_args set the default value according to xlen.  */

#ifndef DEFAULT_RISCV_ARCH_WITH_EXT
#define DEFAULT_RISCV_ARCH_WITH_EXT NULL
#endif

/* The default ISA spec is set to 2.2 rather than the lastest version.
   The reason is that compiler generates the ISA string with fixed 2p0
   verisons only for the RISCV ELF architecture attributes, but not for
   the -march option.  Therefore, we should update the compiler or linker
   to resolve this problem.  */

#ifndef DEFAULT_RISCV_ISA_SPEC
#define DEFAULT_RISCV_ISA_SPEC "andes"
#endif

#ifndef DEFAULT_RISCV_PRIV_SPEC
#define DEFAULT_RISCV_PRIV_SPEC "1.11"
#endif

enum riscv_cl_insn_method
{
  METHOD_DEFAULT,
  METHOD_VARIABLE,
};

enum cmodel_subtype_index
{
  CSI_INDIRECT_SYMBOL = 0,
  CSI_REFERENCE_SYMBOL = 1,
  CSI_LARGE_CODE = 2,
  CSI_DEFAULT_CODE = 3,
  CSI_B22827,
  CSI_B22827_1,
};

static const char default_arch[] = DEFAULT_ARCH;
static const char *default_arch_with_ext = DEFAULT_RISCV_ARCH_WITH_EXT;
static enum riscv_isa_spec_class default_isa_spec = ISA_SPEC_CLASS_NONE;
static enum riscv_priv_spec_class default_priv_spec = PRIV_SPEC_CLASS_NONE;

static unsigned xlen = 0; /* width of an x-register */
static unsigned abi_xlen = 0; /* width of a pointer in the ABI */
static bfd_boolean rve_abi = FALSE;
enum float_abi {
  FLOAT_ABI_DEFAULT = -1,
  FLOAT_ABI_SOFT,
  FLOAT_ABI_SINGLE,
  FLOAT_ABI_DOUBLE,
  FLOAT_ABI_QUAD
};
static enum float_abi float_abi = FLOAT_ABI_DEFAULT;

#define LOAD_ADDRESS_INSN (abi_xlen == 64 ? "ld" : "lw")
#define ADD32_INSN (xlen == 64 ? "addiw" : "addi")

/* Record this attribute is set explicitly by .attribute directive.  */
static int attributes_set_explicitly[NUM_KNOWN_OBJ_ATTRIBUTES + NUM_KNOWN_OBJ_ATTRIBUTES_V5];

static unsigned elf_flags = 0;
/* Save option -O1 for perfomance.  */
static int optimize = 0;
/* Save option -Os for code size.  */
static int optimize_for_space = 0;
/* Save option -mict-model for ICT model setting.  */
static const char *m_ict_model = NULL;
static bfd_boolean pre_insn_is_a_cond_br = 0;

/* { # Andes  */
static void
macro_build (expressionS *ep, const char *name, const char *fmt, ...);

#define MASK_RM (0x7u << 12)

static bfd_boolean
is_insn_fdiv_or_fsqrt (const struct riscv_opcode *insn)
{
  static insn_t insns[] = {
    MATCH_FDIV_D, MATCH_FSQRT_D,
    MATCH_FDIV_S, MATCH_FSQRT_S,
    0
  };
  bfd_boolean rz = FALSE;
  if (insn)
    {
      insn_t x;
      insn_t match = insn->match & ~MASK_RM; /* all rounding mode  */
      int i = 0;
      while ((x = insns[i++]))
	{
	  if (x == match)
	    {
	      rz = TRUE;
	      break;
	    }
	}
    }
  return rz;
}

static inline int
insn_fp_rd (insn_t insn)
{
  int rd = 0x1fu & (insn >> 7);
  return rd;
}

static bfd_boolean
is_insn_in_b22827_list (const struct riscv_opcode *insn,
			insn_t prev, insn_t curr)
{
  static insn_t insns[] = {
    /* unconditional  */
    MATCH_FSUB_S, MATCH_FADD_S, MATCH_FMUL_S, MATCH_FMADD_S,
    MATCH_FSQRT_S, MATCH_FDIV_S,
    MATCH_FSUB_D, MATCH_FADD_D, MATCH_FMUL_D, MATCH_FMADD_D,
    MATCH_FSQRT_D, MATCH_FDIV_D,
    MATCH_FSUB_Q, MATCH_FADD_Q, MATCH_FMUL_Q, MATCH_FMADD_Q,
    MATCH_FSQRT_Q, MATCH_FDIV_Q,
    0
  };

  bfd_boolean rz = FALSE;
  insn_t x;
  insn_t match = insn->match & ~MASK_RM; /* all rounding mode  */
  int i = 0;
  while ((x = insns[i++]))
    {
      if (x == match)
	{
	  rz = TRUE;
	  return !rz;
	}
    }

  /* RD(FDIV/FSQRT) has to be in fa0-7 to exclude jal/ret  */
  if ((((insn->match & MASK_JAL) == MATCH_JAL)
       && (curr & (0x1fu << 7))) // JAL RD != 0 (not J)
      || (insn->match == (MATCH_JAL|(0x1u << 7)))   // JAL RA
      || ((insn->match & MASK_JALR) == MATCH_JALR)  // JALR|JR
      || (insn->match == MATCH_C_JAL)               // C.JAL
      || ((insn->match & MASK_C_JR) == MATCH_C_JR)  // C.JR
      || (insn->match == MATCH_C_JALR))             // C.JALR
    {
      int rd = insn_fp_rd (prev);
      rz = (10 <= rd) && (rd <= 17); /* rd in fa0 .. fa7  */
    }

  return !rz;
}

static inline bfd_boolean
is_insn_fmt_s (insn_t insn)
{
  int fmt = 0x3u & (insn >> 25);
  return fmt == 0; /* 0:s, 1:d, 2:h, 3:q  */
}

static inline bfd_boolean
is_insn_fshw (const struct riscv_opcode *insn)
{
  return insn && (insn->match == MATCH_FSHW);
}

static bfd_boolean
is_insn_of_std_type (const struct riscv_opcode *insn, const char *type)
{
  bfd_boolean rz = FALSE;
  int idx = 0;
  const char *sub;
  const char *p;
  while ((sub = insn->subset[idx++]) && !rz) /* iter subsets */
    {
      p = type;
      while (*p) /* iter type */
	{
	  if (*p++ == sub[0])
	    {
	      rz = TRUE;
	      break;
	    }
	}
    }
  return rz;
}

static bfd_boolean
is_insn_of_fp_types (const struct riscv_opcode *insn)
{
  bfd_boolean rz = FALSE;
  if (insn)
    {
      if (insn->match == MATCH_FSHW
	  || insn->match == MATCH_FLHW)
	rz = TRUE;
      else
	rz = is_insn_of_std_type (insn, "FDQ");
    }
  return rz;
}

#if TO_REMOVE
static bfd_boolean
is_insn_of_i_type (const struct riscv_opcode *insn)
{
  bfd_boolean rz = FALSE;
  if (insn)
    rz = is_insn_of_std_type (insn, "I");
  return rz;
}
#endif

/* } # Andes  */

/* Set the default_isa_spec.  Return 0 if the input spec string isn't
   supported.  Otherwise, return 1.  */

static int
riscv_set_default_isa_spec (const char *s)
{
  enum riscv_isa_spec_class class;
  if (!riscv_get_isa_spec_class (s, &class))
    {
      as_bad ("Unknown default ISA spec `%s' set by "
             "-misa-spec or --with-isa-spec", s);
      return 0;
    }
  else
    default_isa_spec = class;
  return 1;
}

/* Set the default_priv_spec, assembler will find the suitable CSR address
   according to default_priv_spec.  We will try to check priv attributes if
   the input string is NULL.  Return 0 if the input priv spec string isn't
   supported.  Otherwise, return 1.  */

static int
riscv_set_default_priv_spec (const char *s)
{
  enum riscv_priv_spec_class class;
  unsigned major, minor, revision;
  obj_attribute *attr;

  /* Find the corresponding priv spec class.  */
  if (riscv_get_priv_spec_class (s, &class))
    {
      default_priv_spec = class;
      return 1;
    }

  if (s != NULL)
    {
      as_bad (_("Unknown default privilege spec `%s' set by "
               "-mpriv-spec or --with-priv-spec"), s);
      return 0;
    }

  /* Try to set the default_priv_spec according to the priv attributes.  */
  attr = elf_known_obj_attributes_proc (stdoutput);
  major = (unsigned) attr[Tag_RISCV_priv_spec].i;
  minor = (unsigned) attr[Tag_RISCV_priv_spec_minor].i;
  revision = (unsigned) attr[Tag_RISCV_priv_spec_revision].i;

  if (riscv_get_priv_spec_class_from_numbers (major,
					      minor,
					      revision,
					      &class))
    {
      /* The priv attributes setting 0.0.0 is meaningless.  We should have set
	 the default_priv_spec by md_parse_option and riscv_after_parse_args,
	 so just skip the following setting.  */
      if (class == PRIV_SPEC_CLASS_NONE)
	return 1;

      default_priv_spec = class;
      return 1;
    }

  /* Still can not find the priv spec class.  */
  as_bad (_("Unknown default privilege spec `%d.%d.%d' set by "
           "privilege attributes"),  major, minor, revision);
  return 0;
}

/* This is the set of options which the .option pseudo-op may modify.  */

struct riscv_set_options
{
  int pic; /* Generate position-independent code.  */
  int rvc; /* Generate RVC code.  */
  int rve; /* Generate RVE code.  */
  int relax; /* Emit relocs the linker is allowed to relax.  */
  int arch_attr; /* Emit arch attribute.  */
  int check_constraints; /* Enable/disable the match_func checking.  */
  int csr_check; /* Enable the CSR checking.  */
  int no_16_bit; /* Do not emit any 16 bit instructions.  */
  int execit; /* Enable EXECIT relaxation.  */
  int atomic; /* A-ext */
  int verbatim; /* Code is generated by compiler.  */
  int dsp; /* P-ext */
  int efhw; /* Xefhw-ext (flhw/fshw)  */
  int vector; /* V-ext */
  int cmodel; /* cmodel type  */
  int no_vic; /* no vector instruction constraints */
  int update_count; /* virtual option, state of this object.  */
  /* bug{ID} workaround */
  int b19758;
  int b20282;
  int b22827;
  int b22827_1;
  int workaround;
};

enum CMODEL_TYPES {
  CMODEL_DEFAULT,
  CMODEL_LARGE,
};

static struct riscv_set_options riscv_opts =
{
  0,	/* pic */
  0,	/* rvc */
  0,	/* rve */
  1,	/* relax */
  DEFAULT_RISCV_ATTR, /* arch_attr */
  0,	/* check_constraints */
  0, 	/* csr_check */
  0,	/* no_16_bit */
  0,	/* execit */
  0,	/* atomic */
  0,	/* verbatim */
  0,	/* dsp */
  0,	/* efhw */
  0,	/* vector */
  CMODEL_DEFAULT,	/* cmodel */
  0,	/* no_vic */
  0,	/* update_count */
  1,	/* b19758 */
  0,	/* b20282 */
  0,	/* b22827 */
  0,	/* b22827_1 */
  1,	/* workaround */
};

/* The priority: `-mno-16-bit' option
   > `.option rvc/norvc' directive
   > attribute directive > `-march' option
   > default_arch

   Only the higher priority mode can change the rvc setting.
   mode 1: -march option and default_arch
   mode 2: attribute directive
   mode 3: .option directive.  */

static void
riscv_set_rvc (bfd_boolean rvc_value)
{
  /* Always close the rvc when -mno-16-bit option is set.  */
  if (riscv_opts.no_16_bit)
    return;

  if (rvc_value)
    elf_flags |= EF_RISCV_RVC;

  riscv_opts.rvc = rvc_value;
}

static void
riscv_set_rve (bfd_boolean rve_value)
{
  if (rve_value)
    elf_flags |= EF_RISCV_RVE;

  riscv_opts.rve = rve_value;
}

struct riscv_subset
{
  const char *name;

  struct riscv_subset *next;
};

static riscv_subset_list_t riscv_subsets;

static bfd_boolean
riscv_subset_supports (const char *feature)
{
  riscv_subset_t *subset = NULL;
  if (riscv_opts.rvc && (strcasecmp (feature, "c") == 0))
    return TRUE;

  return riscv_lookup_subset (&riscv_subsets, feature, &subset);
}

static bfd_boolean
riscv_multi_subset_supports (const char *features[])
{
  unsigned i = 0;
  bfd_boolean supported = TRUE;

  for (;features[i]; ++i)
    supported = supported && riscv_subset_supports (features[i]);

  return supported;
}

/* Handle of the extension with version hash table.  */
static htab_t ext_version_hash = NULL;

static htab_t ATTRIBUTE_UNUSED
init_ext_version_hash (const struct riscv_ext_version *table)
{
  int i = 0;
  htab_t hash = str_htab_create ();

  while (table[i].name)
    {
      const char *name = table[i].name;
      if (str_hash_insert (hash, name, (void*)&table[i], 0) != NULL)
	as_fatal (_("duplicate %s"), name);

      i++;
      while (table[i].name
            && strcmp (table[i].name, name) == 0)
       i++;
    }

  return hash;
}

static void
riscv_get_default_ext_version (const char *name,
			       int *major_version,
			       int *minor_version)
{
  struct riscv_ext_version *ext;

  if (name == NULL || default_isa_spec == ISA_SPEC_CLASS_NONE)
    return;

  ext = (struct riscv_ext_version *) str_hash_find (ext_version_hash, name);
  while (ext
	 && ext->name
	 && strcmp (ext->name, name) == 0)
    {
      if (ext->isa_spec_class == ISA_SPEC_CLASS_DRAFT
	  || ext->isa_spec_class == default_isa_spec)
	{
	  *major_version = ext->major_version;
	  *minor_version = ext->minor_version;
	  return;
	}
      ext++;
    }
}

/* Set which ISA and extensions are available.  */

static void
riscv_set_arch (const char *s)
{
  riscv_parse_subset_t rps;
  rps.subset_list = &riscv_subsets;
  rps.error_handler = as_bad;
  rps.xlen = &xlen;
  rps.get_default_version = riscv_get_default_ext_version;

  if (s == NULL)
    return;

  riscv_opts.update_count += 1;
  riscv_release_subset_list (&riscv_subsets);
  riscv_parse_subset (&rps, s);

  if (1)
    {
      riscv_subset_t *subset = NULL;
      if (riscv_lookup_subset (rps.subset_list, "e", &subset))
	riscv_set_rve (TRUE);
    }
}

/* Indicate -mabi= option is explictly set.  */
static bfd_boolean explicit_mabi = FALSE;

static void
riscv_set_abi (unsigned new_xlen, enum float_abi new_float_abi, bfd_boolean rve)
{
  abi_xlen = new_xlen;
  float_abi = new_float_abi;
  rve_abi = rve;
}

/* If the -mabi option isn't set, then we set the abi according to the arch
   string.  Otherwise, check if there are conflicts between architecture
   and abi setting.  */

static void
riscv_set_abi_by_arch (void)
{
  if (!explicit_mabi)
    {
      if (riscv_subset_supports ("q"))
	riscv_set_abi (xlen, FLOAT_ABI_QUAD, FALSE);
      else if (riscv_subset_supports ("d"))
	riscv_set_abi (xlen, FLOAT_ABI_DOUBLE, FALSE);
      else
	riscv_set_abi (xlen, FLOAT_ABI_SOFT, FALSE);
    }
  else
    {
      gas_assert (abi_xlen != 0 && xlen != 0 && float_abi != FLOAT_ABI_DEFAULT);
      if (abi_xlen > xlen)
	as_bad ("can't have %d-bit ABI on %d-bit ISA", abi_xlen, xlen);
      else if (abi_xlen < xlen)
	as_bad ("%d-bit ABI not yet supported on %d-bit ISA", abi_xlen, xlen);
    }

  /* Update the EF_RISCV_FLOAT_ABI field of elf_flags.  */
  elf_flags &= ~EF_RISCV_FLOAT_ABI;
  elf_flags |= float_abi << 1;

  if (rve_abi)
    elf_flags |= EF_RISCV_RVE;
}

/* Handle of the OPCODE hash table.  */
static struct hash_control *op_hash = NULL;

/* Handle of the type of .insn hash table.  */
static struct hash_control *insn_type_hash = NULL;

/* This array holds the chars that always start a comment.  If the
    pre-processor is disabled, these aren't very useful */
const char comment_chars[] = "#";

/* This array holds the chars that only start a comment at the beginning of
   a line.  If the line seems to have the form '# 123 filename'
   .line and .file directives will appear in the pre-processed output */
/* Note that input_file.c hand checks for '#' at the beginning of the
   first line of the input file.  This is because the compiler outputs
   #NO_APP at the beginning of its output.  */
/* Also note that C style comments are always supported.  */
const char line_comment_chars[] = "#";

/* This array holds machine specific line separator characters.  */
const char line_separator_chars[] = ";";

/* Chars that can be used to separate mant from exp in floating point nums */
const char EXP_CHARS[] = "eE";

/* Chars that mean this number is a floating point constant */
/* As in 0f12.456 */
/* or    0d1.2345e12 */
const char FLT_CHARS[] = "rRsSfFdDxXpP";

/* Indicate we are already assemble any instructions or not.  */
static bfd_boolean start_assemble = FALSE;

/* Indicate arch attribute is explicitly set.  */
static bfd_boolean explicit_arch_attr = FALSE;

/* Macros for encoding relaxation state for RVC branches and far jumps.  */
#define RELAX_BRANCH_ENCODE(uncond, rvc, length, range)	\
  ((relax_substateT) 					\
   (0xc0000000						\
    | ((uncond) ? 1 : 0)				\
    | ((rvc) ? 2 : 0)					\
    | ((length) << 2)					\
    | ((range) << 6)))
#define RELAX_BRANCH_P(i) (((i) & 0xf0000000) == 0xc0000000)
#define RELAX_BRANCH_LENGTH(i) (((i) >> 2) & 0xF)
#define RELAX_BRANCH_RVC(i) (((i) & 2) != 0)
#define RELAX_BRANCH_UNCOND(i) (((i) & 1) != 0)
#define RELAX_BRANCH_RANGE(i) (((i) >> 6) & 0xF)

#define RELAX_CMODEL_ENCODE(type, length, index)	\
  ((relax_substateT) 					\
   (0xd0000000						\
    | ((type) << 0)					\
    | ((length) << 8)					\
    | ((index) << 16)))
#define RELAX_CMODEL_P(i) (((i) & 0xf0000000) == 0xd0000000)
#define RELAX_CMODEL_TYPE(i) ((i) & 0xff)
#define RELAX_CMODEL_LENGTH(i) (((i) >> 8) & 0xff)
#define RELAX_CMODEL_INDEX(i) (((i) >> 16) & 0xff)

enum cmodel_type
{
  TYPE_JX = 0,
  TYPE_LA,
  TYPE_LD,
  TYPE_ST,
  TYPE_IS, /* indirect symbol  */
  TYPE_B22827,
  TYPE_B22827_1,
};

enum branch_range
{
  RANGE_JMP = 1,
  RANGE_BRANCH = 2,
  RANGE_10_PCREL = 3
};

#define ENUM_BRANCH_RANGE(reloc)					\
  ((reloc == BFD_RELOC_RISCV_JMP) ? RANGE_JMP				\
    : ((reloc == BFD_RELOC_12_PCREL) ? RANGE_BRANCH			\
      : (reloc == BFD_RELOC_RISCV_10_PCREL) ? RANGE_10_PCREL : 0))

/* Is the given value a sign-extended 32-bit value?  */
#define IS_SEXT_32BIT_NUM(x)						\
  (((x) &~ (offsetT) 0x7fffffff) == 0					\
   || (((x) &~ (offsetT) 0x7fffffff) == ~ (offsetT) 0x7fffffff))

/* Is the given value a zero-extended 32-bit value?  Or a negated one?  */
#define IS_ZEXT_32BIT_NUM(x)						\
  (((x) &~ (offsetT) 0xffffffff) == 0					\
   || (((x) &~ (offsetT) 0xffffffff) == ~ (offsetT) 0xffffffff))

/* Change INSN's opcode so that the operand given by FIELD has value VALUE.
   INSN is a riscv_cl_insn structure and VALUE is evaluated exactly once.  */
#define INSERT_OPERAND(FIELD, INSN, VALUE) \
  INSERT_BITS ((INSN).insn_opcode, VALUE, OP_MASK_##FIELD, OP_SH_##FIELD)

/* Determine if an instruction matches an opcode.  */
#define OPCODE_MATCHES(OPCODE, OP) \
  (((OPCODE) & MASK_##OP) == MATCH_##OP)

static char *expr_end;

static struct
{
  /* riscv_opts.b22827  */
  struct riscv_cl_insn prev_insn;
  fragS *frag_b22827;
} nds;

/* The default target format to use.  */

const char *
riscv_target_format (void)
{
  return xlen == 64 ? "elf64-littleriscv" : "elf32-littleriscv";
}

/* Return the length of instruction INSN.  */

static inline unsigned int
insn_length (const struct riscv_cl_insn *insn)
{
  return riscv_insn_length (insn->insn_opcode);
}

/* Initialise INSN from opcode entry MO.  Leave its position unspecified.  */

static void
create_insn (struct riscv_cl_insn *insn, const struct riscv_opcode *mo)
{
  insn->insn_mo = mo;
  insn->insn_opcode = mo->match;
  insn->frag = NULL;
  insn->where = 0;
  insn->fixp = NULL;
}

/* Install INSN at the location specified by its "frag" and "where" fields.  */

static void
install_insn (const struct riscv_cl_insn *insn)
{
  char *f = insn->frag->fr_literal + insn->where;
  md_number_to_chars (f, insn->insn_opcode, insn_length (insn));
}

/* Move INSN to offset WHERE in FRAG.  Adjust the fixups accordingly
   and install the opcode in the new location.  */

static void
move_insn (struct riscv_cl_insn *insn, fragS *frag, long where)
{
  insn->frag = frag;
  insn->where = where;
  if (insn->fixp != NULL)
    {
      insn->fixp->fx_frag = frag;
      insn->fixp->fx_where = where;
    }
  install_insn (insn);
}

/* Add INSN to the end of the output.  */

static void
add_fixed_insn (struct riscv_cl_insn *insn)
{
  char *f = frag_more (insn_length (insn));
  move_insn (insn, frag_now, f - frag_now->fr_literal);
}

static void
add_relaxed_insn (struct riscv_cl_insn *insn, int max_chars, int var,
      relax_substateT subtype, symbolS *symbol, offsetT offset)
{
  frag_grow (max_chars);
  move_insn (insn, frag_now, frag_more (0) - frag_now->fr_literal);
  frag_var (rs_machine_dependent, max_chars, var,
	    subtype, symbol, offset, NULL);
}

static void
add_insn_grow (struct riscv_cl_insn *insn)
{
  frag_grow (insn_length (insn));
  move_insn (insn, frag_now, frag_more (0) - frag_now->fr_literal + insn->cmodel.offset);
}

static inline void
add_insn_grow_done (struct riscv_cl_insn *insn ATTRIBUTE_UNUSED, int max_chars, int var,
      relax_substateT subtype, symbolS *symbol, offsetT offset)
{
  frag_var (rs_machine_dependent, max_chars, var,
	    subtype, symbol, offset, NULL);
}

static inline
bfd_boolean is_cmodel_large (void)
{
  return riscv_opts.cmodel == CMODEL_LARGE;
}

static inline
bfd_boolean is_same_section_symbol (symbolS *sym, asection *sec)
{
  return (sym != NULL
	  && S_IS_DEFINED (sym)
	  && !S_IS_WEAK (sym)
	  && sec == S_GET_SEGMENT (sym));
}

static inline
bfd_boolean is_cmodel_relaxable (symbolS *sym, asection *sec)
{
  return (abi_xlen >= 64
	  && is_cmodel_large ()
	  && !is_same_section_symbol (sym, sec));
}

/* Compute the length of a branch sequence, and adjust the stored length
   accordingly.  If FRAGP is NULL, the worst-case length is returned.  */

static unsigned
relaxed_branch_length (fragS *fragp, asection *sec, int update)
{
  int jump, rvc, length = 8;
  enum branch_range range;

  if (!fragp)
    return length;

  jump = RELAX_BRANCH_UNCOND (fragp->fr_subtype);
  rvc = RELAX_BRANCH_RVC (fragp->fr_subtype);
  length = RELAX_BRANCH_LENGTH (fragp->fr_subtype);
  range = RELAX_BRANCH_RANGE (fragp->fr_subtype);

  /* Assume jumps are in range; the linker will catch any that aren't.  */
  length = jump ? 4 : 8;

  if (fragp->fr_symbol != NULL
      && S_IS_DEFINED (fragp->fr_symbol)
      && !S_IS_WEAK (fragp->fr_symbol)
      && sec == S_GET_SEGMENT (fragp->fr_symbol))
    {
      offsetT val = S_GET_VALUE (fragp->fr_symbol) + fragp->fr_offset;
      bfd_vma rvc_range = jump ? RVC_JUMP_REACH : RVC_BRANCH_REACH;
      val -= fragp->fr_address + fragp->fr_fix;

      if (rvc && (bfd_vma)(val + rvc_range/2) < rvc_range)
	length = 2;
      else if (range == RANGE_BRANCH
	       && (bfd_vma)(val + RISCV_BRANCH_REACH/2) < RISCV_BRANCH_REACH)
	length = 4;
      else if  ((bfd_vma)(val + RISCV_10_PCREL_REACH/2) < RISCV_10_PCREL_REACH)
	length = 4;
      else if (!jump && rvc)
	length = 6;
    }

  if (update)
    fragp->fr_subtype = RELAX_BRANCH_ENCODE (jump, rvc, length, range);

  return length;
}

/* Compute the length of a CModel sequence, and adjust the stored length
   accordingly.  */
static unsigned
relaxed_cmodel_length (fragS *fragp, asection *sec)
{
  int type = RELAX_CMODEL_TYPE (fragp->fr_subtype);
  int length = RELAX_CMODEL_LENGTH (fragp->fr_subtype);
  int index = RELAX_CMODEL_INDEX (fragp->fr_subtype);
  int is_same_sec = is_same_section_symbol (fragp->fr_symbol, sec);

  gas_assert (fragp);

  switch (index)
    {
    case CSI_INDIRECT_SYMBOL:
      switch (type)
	{
	case TYPE_JX ... TYPE_ST:
	  length = 0;
	  break;
	case TYPE_IS:
	  if (is_same_sec)
	    length = 0;
	  else
	    gas_assert (length == 8);
	  break;
	default:
	  as_fatal (_("internal error: invalid CModel type!"));
	}
      break;
    case CSI_REFERENCE_SYMBOL:
      switch (type)
	{
	case TYPE_JX ... TYPE_ST:
	  length = 0;
	  break;
	default:
	  as_fatal (_("internal error: invalid CModel type!"));
	}
      break;
    case CSI_LARGE_CODE:
      switch (type)
	{
	case TYPE_JX:
	  if (is_same_sec)
	    length = 0;
	  break;
	case TYPE_LA:
	    length = 8;
	    break;
	case TYPE_LD... TYPE_ST:
	  if (is_same_sec)
	    length = 8; /* -= 4 is NG, might do more than time.  */
	  break;
	default:
	  as_fatal (_("internal error: invalid CModel type!"));
	}
      break;
    case CSI_DEFAULT_CODE:
      switch (type)
	{
	case TYPE_JX:
	  if (!is_same_sec)
	    length = 0;
	  break;
	default:
	  as_fatal (_("internal error: invalid CModel type!"));
	}
      break;
    case CSI_B22827:
    case CSI_B22827_1:
      if (fragp->fr_var == 0) length = 0;
      else if (fragp->fr_var == 1) length = 4;
      break;
    default:
      as_fatal (_("internal error: invalid CModel index!"));
    }

  fragp->fr_subtype = RELAX_CMODEL_ENCODE (type, length, index);
  return length;
}

/* Information about an opcode name, mnemonics and its value.  */
struct opcode_name_t
{
  const char *name;
  unsigned int val;
};

/* List for all supported opcode name.  */
static const struct opcode_name_t opcode_name_list[] =
{
  {"C0",        0x0},
  {"C1",        0x1},
  {"C2",        0x2},

  {"LOAD",      0x03},
  {"LOAD_FP",   0x07},
  {"CUSTOM_0",  0x0b},
  {"MISC_MEM",  0x0f},
  {"OP_IMM",    0x13},
  {"AUIPC",     0x17},
  {"OP_IMM_32", 0x1b},
  /* 48b        0x1f.  */

  {"STORE",     0x23},
  {"STORE_FP",  0x27},
  {"CUSTOM_1",  0x2b},
  {"AMO",       0x2f},
  {"OP",        0x33},
  {"LUI",       0x37},
  {"OP_32",     0x3b},
  /* 64b        0x3f.  */

  {"MADD",      0x43},
  {"MSUB",      0x47},
  {"NMADD",     0x4f},
  {"NMSUB",     0x4b},
  {"OP_FP",     0x53},
  /*reserved    0x57.  */
  {"CUSTOM_2",  0x5b},
  /* 48b        0x5f.  */

  {"BRANCH",    0x63},
  {"JALR",      0x67},
  /*reserved    0x5b.  */
  {"JAL",       0x6f},
  {"SYSTEM",    0x73},
  /*reserved    0x77.  */
  {"CUSTOM_3",  0x7b},
  /* >80b       0x7f.  */

  {NULL, 0}
};

/* Hash table for lookup opcode name.  */
static struct hash_control *opcode_names_hash = NULL;

/* Initialization for hash table of opcode name.  */
static void
init_opcode_names_hash (void)
{
  const char *retval;
  const struct opcode_name_t *opcode;

  for (opcode = &opcode_name_list[0]; opcode->name != NULL; ++opcode)
    {
      retval = hash_insert (opcode_names_hash, opcode->name, (void *)opcode);

      if (retval != NULL)
	as_fatal (_("internal error: can't hash `%s': %s"),
		  opcode->name, retval);
    }
}

/* Find `s` is a valid opcode name or not,
   return the opcode name info if found.  */
static const struct opcode_name_t *
opcode_name_lookup (char **s)
{
  char *e;
  char save_c;
  struct opcode_name_t *o;

  /* Find end of name.  */
  e = *s;
  if (is_name_beginner (*e))
    ++e;
  while (is_part_of_name (*e))
    ++e;

  /* Terminate name.  */
  save_c = *e;
  *e = '\0';

  o = (struct opcode_name_t *) hash_find (opcode_names_hash, *s);

  /* Advance to next token if one was recognized.  */
  if (o)
    *s = e;

  *e = save_c;
  expr_end = e;

  return o;
}

struct regname
{
  const char *name;
  unsigned int num;
};

enum reg_class
{
  RCLASS_GPR,
  RCLASS_FPR,
  RCLASS_CSR,
  RCLASS_VECR,
  RCLASS_VECM,
  RCLASS_MAX
};

static struct hash_control *reg_names_hash = NULL;
static struct hash_control *csr_extra_hash = NULL;

#define ENCODE_REG_HASH(cls, n) \
  ((void *)(uintptr_t)((n) * RCLASS_MAX + (cls) + 1))
#define DECODE_REG_CLASS(hash) (((uintptr_t)(hash) - 1) % RCLASS_MAX)
#define DECODE_REG_NUM(hash) (((uintptr_t)(hash) - 1) / RCLASS_MAX)

static void
hash_reg_name (enum reg_class class, const char *name, unsigned n)
{
  void *hash = ENCODE_REG_HASH (class, n);
  const char *retval = hash_insert (reg_names_hash, name, hash);

  if (retval != NULL)
    as_fatal (_("internal error: can't hash `%s': %s"), name, retval);
}

static void
hash_reg_names (enum reg_class class, const char * const names[], unsigned n)
{
  unsigned i;

  for (i = 0; i < n; i++)
    hash_reg_name (class, names[i], i);
}

/* Init two hashes, csr_extra_hash and reg_names_hash, for CSR.  */

static void
riscv_init_csr_hashes (const char *name,
		       unsigned address,
		       enum riscv_csr_class class)
{
  struct riscv_csr_extra *entry = XNEW (struct riscv_csr_extra);
  entry->csr_class = class;

  const char *hash_error =
    hash_insert (csr_extra_hash, name, (void *) entry);
  if (hash_error != NULL)
    {
      fprintf (stderr, _("internal error: can't hash `%s': %s\n"),
		      name, hash_error);
      /* Probably a memory allocation problem?  Give up now.  */
	as_fatal (_("Broken assembler.  No assembly attempted."));
    }

  hash_reg_name (RCLASS_CSR, name, address);
}

/* Check wether the CSR is valid according to the ISA.  */

static bfd_boolean
riscv_csr_class_check (enum riscv_csr_class csr_class)
{
  switch (csr_class)
    {
    case CSR_CLASS_I: return riscv_subset_supports ("i");
    case CSR_CLASS_F: return riscv_subset_supports ("f");
    case CSR_CLASS_V:
      return (riscv_subset_supports ("v")
	      || riscv_subset_supports ("zvamo")
	      || riscv_subset_supports ("zvlsseg"));
    case CSR_CLASS_I_32:
      return (xlen == 32 && riscv_subset_supports ("i"));
    case CSR_CLASS_ZKR:
      return riscv_subset_supports ("zkr");
    case CSR_CLASS_SSCOFPMF:
      return riscv_subset_supports ("sscofpmf");
    case CSR_CLASS_DEBUG:
      return TRUE;

    default:
      return FALSE;
    }
}

/* If the CSR is defined, then we call `riscv_csr_class_check` to do the
   further checking.  Return FALSE if the CSR is not defined.  Otherwise,
   return TRUE.  */

static bfd_boolean
reg_csr_lookup_internal (const char *s)
{
  struct riscv_csr_extra *r =
    (struct riscv_csr_extra *) hash_find (csr_extra_hash, s);

  if (r == NULL)
    return FALSE;

  /* We just report the warning when the CSR is invalid.  */
  if (!riscv_csr_class_check (r->csr_class))
    as_warn (_("Invalid CSR `%s' for the current ISA"), s);

  return TRUE;
}

static unsigned int
reg_lookup_internal (const char *s, enum reg_class class)
{
  struct regname *r = (struct regname *) hash_find (reg_names_hash, s);

  if (r == NULL || DECODE_REG_CLASS (r) != class)
    return -1;

  if (riscv_opts.rve && class == RCLASS_GPR && DECODE_REG_NUM (r) > 15)
    return -1;

  if (class == RCLASS_CSR
      && riscv_opts.csr_check
      && !reg_csr_lookup_internal (s))
    return -1;

  return DECODE_REG_NUM (r);
}

static bfd_boolean
reg_lookup (char **s, enum reg_class class, unsigned int *regnop)
{
  char *e;
  char save_c;
  int reg = -1;

  /* Find end of name.  */
  e = *s;
  if (is_name_beginner (*e))
    ++e;
  while (is_part_of_name (*e))
    ++e;

  /* Terminate name.  */
  save_c = *e;
  *e = '\0';

  /* Look for the register.  Advance to next token if one was recognized.  */
  if ((reg = reg_lookup_internal (*s, class)) >= 0)
    *s = e;

  *e = save_c;
  if (regnop)
    *regnop = reg;
  return reg >= 0;
}

static bfd_boolean
arg_lookup (char **s, const char *const *array, size_t size, unsigned *regnop)
{
  const char *p = strchr (*s, ',');
  size_t i, len = p ? (size_t)(p - *s) : strlen (*s);

  if (len == 0)
    return FALSE;

  for (i = 0; i < size; i++)
    if (array[i] != NULL && strncmp (array[i], *s, len) == 0)
      {
	*regnop = i;
	*s += len;
	return TRUE;
      }

  return FALSE;
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
  while (ISALNUM (*str_t) || *str_t == '.' || *str_t == '_')
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

/* For consistency checking, verify that all bits are specified either
   by the match/mask part of the instruction definition, or by the
   operand list.

   `length` could be 0, 4 or 8, 0 for auto detection.  */
static bfd_boolean
validate_riscv_insn (const struct riscv_opcode *opc, int length)
{
  const char *p = opc->args;
  char c;
  insn_t used_bits = opc->mask;
  int insn_width;
  insn_t required_bits;

  if (length == 0)
    insn_width = 8 * riscv_insn_length (opc->match);
  else
    insn_width = 8 * length;

  required_bits = ~0ULL >> (64 - insn_width);

  if ((used_bits & opc->match) != (opc->match & required_bits))
    {
      as_bad (_("internal: bad RISC-V opcode (mask error): %s %s"),
	      opc->name, opc->args);
      return FALSE;
    }

#define USE_BITS(mask,shift)	(used_bits |= ((insn_t)(mask) << (shift)))
  while (*p)
    switch (c = *p++)
      {
      case 'C': /* RVC */
	switch (c = *p++)
	  {
	  case 'a': used_bits |= ENCODE_RVC_J_IMM (-1U); break;
	  case 'c': break; /* RS1, constrained to equal sp */
	  case 'i': used_bits |= ENCODE_RVC_SIMM3(-1U); break;
	  case 'j': used_bits |= ENCODE_RVC_IMM (-1U); break;
	  case 'e':
	    switch (c = *p++)
	      {
	      case 'i':
		used_bits |= ENCODE_RVC_EX9IT_IMM (-1U); break;
	      case 't':
		used_bits |= ENCODE_RVC_EXECIT_IMM (-1U); break;
	      default:
		break;
	      }
	  case 'o': used_bits |= ENCODE_RVC_IMM (-1U); break;
	  case 'k': used_bits |= ENCODE_RVC_LW_IMM (-1U); break;
	  case 'l': used_bits |= ENCODE_RVC_LD_IMM (-1U); break;
	  case 'm': used_bits |= ENCODE_RVC_LWSP_IMM (-1U); break;
	  case 'n': used_bits |= ENCODE_RVC_LDSP_IMM (-1U); break;
	  case 'p': used_bits |= ENCODE_RVC_B_IMM (-1U); break;
	  case 's': USE_BITS (OP_MASK_CRS1S, OP_SH_CRS1S); break;
	  case 't': USE_BITS (OP_MASK_CRS2S, OP_SH_CRS2S); break;
	  case 'u': used_bits |= ENCODE_RVC_IMM (-1U); break;
	  case 'v': used_bits |= ENCODE_RVC_IMM (-1U); break;
	  case 'w': break; /* RS1S, constrained to equal RD */
	  case 'x': break; /* RS2S, constrained to equal RD */
	  case 'z': break; /* RS2S, contrained to be x0 */
	  case 'K': used_bits |= ENCODE_RVC_ADDI4SPN_IMM (-1U); break;
	  case 'L': used_bits |= ENCODE_RVC_ADDI16SP_IMM (-1U); break;
	  case 'M': used_bits |= ENCODE_RVC_SWSP_IMM (-1U); break;
	  case 'N': used_bits |= ENCODE_RVC_SDSP_IMM (-1U); break;
	  case 'U': break; /* RS1, constrained to equal RD */
	  case 'V': USE_BITS (OP_MASK_CRS2, OP_SH_CRS2); break;
	  case '<': used_bits |= ENCODE_RVC_IMM (-1U); break;
	  case '>': used_bits |= ENCODE_RVC_IMM (-1U); break;
	  case '8': used_bits |= ENCODE_RVC_UIMM8 (-1U); break;
	  case 'S': USE_BITS (OP_MASK_CRS1S, OP_SH_CRS1S); break;
	  case 'T': USE_BITS (OP_MASK_CRS2, OP_SH_CRS2); break;
	  case 'D': USE_BITS (OP_MASK_CRS2S, OP_SH_CRS2S); break;
	  case 'F': /* funct */
	    switch (c = *p++)
	      {
		case '6': USE_BITS (OP_MASK_CFUNCT6, OP_SH_CFUNCT6); break;
		case '4': USE_BITS (OP_MASK_CFUNCT4, OP_SH_CFUNCT4); break;
		case '3': USE_BITS (OP_MASK_CFUNCT3, OP_SH_CFUNCT3); break;
		case '2': USE_BITS (OP_MASK_CFUNCT2, OP_SH_CFUNCT2); break;
		default:
		  as_bad (_("internal: bad RISC-V opcode"
			    " (unknown operand type `CF%c'): %s %s"),
			  c, opc->name, opc->args);
		  return FALSE;
	      }
	    break;
	  default:
	    as_bad (_("internal: bad RISC-V opcode (unknown operand type `C%c'): %s %s"),
		    c, opc->name, opc->args);
	    return FALSE;
	  }
	break;
      case ',': break;
      case '+': break;
      case '(': break;
      case ')': break;
      case '<': USE_BITS (OP_MASK_SHAMTW,	OP_SH_SHAMTW);	break;
      case '>':	USE_BITS (OP_MASK_SHAMT,	OP_SH_SHAMT);	break;
      case 'A': break;
      case 'D':	USE_BITS (OP_MASK_RD,		OP_SH_RD);	break;
      case 'Z':	USE_BITS (OP_MASK_RS1,		OP_SH_RS1);	break;
      case 'E':	USE_BITS (OP_MASK_CSR,		OP_SH_CSR);	break;
      case 'I': break;
      case 'R':	USE_BITS (OP_MASK_RS3,		OP_SH_RS3);	break;
      case 'S':	USE_BITS (OP_MASK_RS1,		OP_SH_RS1);	break;
      case 'e': USE_BITS (OP_MASK_RS2,   	OP_SH_RS2);	break;
      case 'U':	USE_BITS (OP_MASK_RS1,		OP_SH_RS1);	/* fallthru */
      case 'T':	USE_BITS (OP_MASK_RS2,		OP_SH_RS2);	break;
      case 'd':	USE_BITS (OP_MASK_RD,		OP_SH_RD);	break;
      case 'm':	USE_BITS (OP_MASK_RM,		OP_SH_RM);	break;
      case 's':	USE_BITS (OP_MASK_RS1,		OP_SH_RS1);	break;
      case 't':	USE_BITS (OP_MASK_RS2,		OP_SH_RS2);	break;
      case 'r':	USE_BITS (OP_MASK_RS3,		OP_SH_RS3);	break;
      case 'P':	USE_BITS (OP_MASK_PRED,		OP_SH_PRED);	break;
      case 'Q':	USE_BITS (OP_MASK_SUCC,		OP_SH_SUCC);	break;
      case 'v': USE_BITS (OP_MASK_SV,		OP_SH_SV);	break;
      case 'y': USE_BITS (OP_MASK_BS,	OP_SH_BS); break;
      case 'Y': USE_BITS (OP_MASK_RNUM, OP_SH_RNUM); break;
      case 'o':
      case 'j': used_bits |= ENCODE_ITYPE_IMM (-1U); break;
      case 'a':	used_bits |= ENCODE_UJTYPE_IMM (-1U); break;
      case 'p':	used_bits |= ENCODE_SBTYPE_IMM (-1U); break;
      case 'q':	used_bits |= ENCODE_STYPE_IMM (-1U); break;
      case 'f': used_bits |= ENCODE_STYPE_IMM (-1U); break;
      case 'u':	used_bits |= ENCODE_UTYPE_IMM (-1U); break;
      case 'h': used_bits |= ENCODE_ITYPE_IMM (-1U); break;
      case 'l': used_bits |= ENCODE_ITYPE_IMM (-1U); break;
      case 'i': used_bits |= ENCODE_STYPE_IMM7 (-1U); break;
      case 'g': used_bits |= ENCODE_STYPE_IMM10 (-1U); break;
      case 'k': used_bits |= ENCODE_TYPE_CIMM6 (-1U); break;
      case 'z': break;
      case '[': break;
      case ']': break;
      case '0': break;
      case '1': break;
      case 'F': /* funct */
	switch (c = *p++)
	  {
	    case '7': USE_BITS (OP_MASK_FUNCT7, OP_SH_FUNCT7); break;
	    case '3': USE_BITS (OP_MASK_FUNCT3, OP_SH_FUNCT3); break;
	    case '2': USE_BITS (OP_MASK_FUNCT2, OP_SH_FUNCT2); break;
	    default:
	      as_bad (_("internal: bad RISC-V opcode"
			" (unknown operand type `F%c'): %s %s"),
		      c, opc->name, opc->args);
	    return FALSE;
	  }
	break;
      case 'O': /* opcode */
	switch (c = *p++)
	  {
	    case '4': USE_BITS (OP_MASK_OP, OP_SH_OP); break;
	    case '2': USE_BITS (OP_MASK_OP2, OP_SH_OP2); break;
	    default:
	      as_bad (_("internal: bad RISC-V opcode"
			" (unknown operand type `F%c'): %s %s"),
		      c, opc->name, opc->args);
	     return FALSE;
	  }
	break;

      case 'V': /* RVV */
	switch (c = *p++)
	  {
	  case 'd':
	  case 'f': USE_BITS (OP_MASK_VD, OP_SH_VD); break;
	  case 'e': USE_BITS (OP_MASK_VWD, OP_SH_VWD); break;
	  case 's': USE_BITS (OP_MASK_VS1, OP_SH_VS1); break;
	  case 't': USE_BITS (OP_MASK_VS2, OP_SH_VS2); break;
	  case 'u': USE_BITS (OP_MASK_VS1, OP_SH_VS1);
		    USE_BITS (OP_MASK_VS2, OP_SH_VS2); break;
	  case 'v': USE_BITS (OP_MASK_VD, OP_SH_VD);
		    USE_BITS (OP_MASK_VS1, OP_SH_VS1);
		    USE_BITS (OP_MASK_VS2, OP_SH_VS2); break;
	  case '0': break;
	  case 'b': used_bits |= ENCODE_RVV_VB_IMM (-1U); break;
	  case 'c': used_bits |= ENCODE_RVV_VC_IMM (-1U); break;
	  case 'i':
	  case 'j':
	  case 'k': USE_BITS (OP_MASK_VIMM, OP_SH_VIMM); break;
	  case 'm': USE_BITS (OP_MASK_VMASK, OP_SH_VMASK); break;
	  default:
	    as_bad (_("internal: bad RISC-V opcode (unknown operand type `V%c'): %s %s"),
		    c, opc->name, opc->args);
	  }
	break;

      case 'H':
	switch (c = *p++)
	  {
	    case 'b': used_bits |= ENCODE_GPTYPE_SB_IMM (-1U); break;
	    case 'h': used_bits |= ENCODE_GPTYPE_SH_IMM (-1U); break;
	    case 'w': used_bits |= ENCODE_GPTYPE_SW_IMM (-1U); break;
	    case 'd': used_bits |= ENCODE_GPTYPE_SD_IMM (-1U); break;
	    default: break;
	  }
	break;
      case 'G':
	switch (c = *p++)
	  {
	    case 'b': used_bits |= ENCODE_GPTYPE_LB_IMM (-1U); break;
	    case 'h': used_bits |= ENCODE_GPTYPE_LH_IMM (-1U); break;
	    case 'w': used_bits |= ENCODE_GPTYPE_LW_IMM (-1U); break;
	    case 'd': used_bits |= ENCODE_GPTYPE_LD_IMM (-1U); break;
	    default: break;
	  }
	break;
      /* Handle operand fields of V5 extension.  */
      case 'n':
	{
	  char field_name[MAX_KEYWORD_LEN];
	  if (parse_nds_v5_field (&p, field_name))
	    {
	      /* TODO: build hash table to store nds-defined operand fields.  */
	      if (strcmp (field_name, "nds_rc") == 0)
		USE_BITS (OP_MASK_RC, OP_SH_RC);
	      else if (strcmp (field_name, "nds_rdp") == 0)
		USE_BITS (OP_MASK_RD, OP_SH_RD);
	      else if (strcmp (field_name, "nds_rsp") == 0)
		USE_BITS (OP_MASK_RD, OP_SH_RS1);
	      else if (strcmp (field_name, "nds_rtp") == 0)
		USE_BITS (OP_MASK_RD, OP_SH_RS2);
	      else if (strcmp (field_name, "nds_i3u") == 0)
		used_bits |= ENCODE_PTYPE_IMM3U (-1U);
	      else if (strcmp (field_name, "nds_i4u") == 0)
		used_bits |= ENCODE_PTYPE_IMM4U (-1U);
	      else if (strcmp (field_name, "nds_i5u") == 0)
		used_bits |= ENCODE_PTYPE_IMM5U (-1U);
	      else if (strcmp (field_name, "nds_i6u") == 0)
		used_bits |= ENCODE_PTYPE_IMM6U (-1U);
	      else if (strcmp (field_name, "nds_i15s") == 0)
		used_bits |= ENCODE_PTYPE_IMM15S (-1U);
	      else
		as_bad (_("internal: bad RISC-V opcode "
			  "(unknown operand type `%s'): %s %s"),
			field_name, opc->name, opc->args);
	    }
	  else
	    as_bad (_("internal: bad RISC-V opcode "
		      "(unknown operand type `%c'): %s %s"),
		    c, opc->name, opc->args);
	}
	break;

      case 'N': /* Andes extensions */
	{
	  switch (c = *p++)
	    {
	    case 'f': used_bits |= ENCODE_TYPE_IMM8 (-1U); break;
	    default: break;
	    }
	}
	break;

      default:
	as_bad (_("internal: bad RISC-V opcode "
		  "(unknown operand type `%c'): %s %s"),
		c, opc->name, opc->args);
	return FALSE;
      }
#undef USE_BITS
  if (used_bits != required_bits)
    {
      as_bad (_("internal: bad RISC-V opcode (bits 0x%lx undefined): %s %s"),
	      ~(unsigned long)(used_bits & required_bits),
	      opc->name, opc->args);
      return FALSE;
    }
  return TRUE;
}

struct percent_op_match
{
  const char *str;
  bfd_reloc_code_real_type reloc;
};

/* Common hash table initialization function for
   instruction and .insn directive.  */
static struct hash_control *
init_opcode_hash (const struct riscv_opcode *opcodes,
		  bfd_boolean insn_directive_p)
{
  int i = 0;
  int length;
  struct hash_control *hash = hash_new ();
  while (opcodes[i].name)
    {
      const char *name = opcodes[i].name;
      const char *hash_error =
	hash_insert (hash, name, (void *) &opcodes[i]);

      if (hash_error != NULL)
	{
	  fprintf (stderr, _("internal error: can't hash `%s': %s\n"),
		   opcodes[i].name, hash_error);
	  /* Probably a memory allocation problem?  Give up now.  */
	  as_fatal (_("Broken assembler.  No assembly attempted."));
	}

      do
	{
	  if (opcodes[i].pinfo != INSN_MACRO)
	    {
	      if (insn_directive_p)
		length = ((name[0] == 'c') ? 2 : 4);
	      else
		length = 0; /* Let assembler determine the length. */
	      if (!validate_riscv_insn (&opcodes[i], length))
		as_fatal (_("Broken assembler.  No assembly attempted."));
	    }
	  else
	    gas_assert (!insn_directive_p);
	  ++i;
	}
      while (opcodes[i].name && !strcmp (opcodes[i].name, name));
    }

  return hash;
}

/* Data structures used by ACE */
typedef struct ace_keyword
{
  const char *name;  /* register name */
  int value;  /* register index */
  uint64_t attr;  /* register attribute */
} ace_keyword_t;

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

/* Hash table for storing symbols from shared library */
static struct hash_control *ace_keyword_hash = NULL;
static struct hash_control *ace_op_hash = NULL;
/* Pointers for storing symbols from ACE shared library */
struct riscv_opcode *ace_opcs;
ace_keyword_t *ace_keys;
ace_op_t *ace_ops;
/* Represent whether ACE shared library is loaded successfully */
bfd_boolean ace_lib_load_success = FALSE;

static void
ace_encode_insn (unsigned int v, ace_op_t * ace_op, struct riscv_cl_insn *ip)
{
  unsigned int bit_value = v;

  /* Perform mask to truncate oversize value */
  bit_value <<= 32 - ace_op->bitsize;
  bit_value >>= 32 - ace_op->bitsize;

  /* Shift value to specified position */
  bit_value <<= ace_op->bitpos - ace_op->bitsize + 1;

  ip->insn_opcode = (unsigned int) ip->insn_opcode | bit_value;
}

static void
ace_encode_insn_discrete (unsigned int v, char *op_name_discrete, const char *op, struct riscv_cl_insn *ip)
{
  bfd_boolean found_or_token = TRUE;
  unsigned bit_value = 0;
  char *psep, *pval = op_name_discrete + strlen(op);
  unsigned msb = 0, width = 0, width_acc = 0;

  while (found_or_token) {
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
	bit_value = v >> width_acc;
	bit_value <<= 32 - width;
	bit_value >>= 32 - width;

	/* Shift value to specified position */
	bit_value <<= msb - width + 1;

	ip->insn_opcode = (unsigned int) ip->insn_opcode | bit_value;
	width_acc += width;

	/* Prepare condition for next iteration */
	pval = psep + 1;
  }

}

/* Assemble ACE instruction string to binary */

static void
ace_ip (char **args, char **str, struct riscv_cl_insn *ip)
{
  /* Extract field attribute name from opcode description (ace_ops) and
     store the extracted result to var of op_name for finding the
     field attribute information from ace_op_hash */
  bfd_boolean found_op_str_end = FALSE;
  char *pch = strchr (*args, ',');
  if (pch == NULL)
    {
      pch = strchr (*args, '\0');
      found_op_str_end = TRUE;
    }
  if (pch == NULL)
    as_fatal (_("Broken assembler.  No assembly attempted."));

  unsigned int op_name_size = pch - (*args + 1);
  char *op_name = malloc (op_name_size + 1);
  memcpy (op_name, *args + 1, op_name_size);
  /* Cat null character to the end of op_name to avoid gash */
  memcpy (op_name + op_name_size, "\0", 1);

  /* Check whether encounter the end of line in assembly code */
  bfd_boolean found_asm_end = FALSE;
  if (strchr (*str, ',') == NULL && strchr (*str, '\0') != NULL)
    found_asm_end = TRUE;

  /* With rGPR encoding format, operand bit-field may be discrete.
     There is an "|" token in discrete format */
  bfd_boolean is_discrete = FALSE;
  char *por = strchr (op_name, '|');
  char *op_name_discrete = NULL;
  if (por != NULL)
    {
      is_discrete = TRUE;
      op_name_discrete = malloc (op_name_size + 1);
      strcpy (op_name_discrete, op_name);
      *por = '\0';
    }

  /*  Find the field attribute from ace_op_hash and encode instruction */
  ace_op_t *ace_op = (ace_op_t *) hash_find (ace_op_hash, op_name);
  switch (ace_op->hw_res)
    {
    case HW_GPR:
	{
	  unsigned int regno;
	  /* Extract the GPR index string from assembly code (*str) */
	  if (reg_lookup (str, RCLASS_GPR, &regno))
	    /* Encode instruction */
	    ace_encode_insn (regno, ace_op, ip);

	  /* Update the address of pointer of assembly code (*str) */
	  if (!found_asm_end)
	    *str += 1;
	}
      break;

    case HW_FPR:
	{
	  unsigned int regno;
	  /* Extract the FPR index string from assembly code (*str) */
	  if (reg_lookup (str, RCLASS_FPR, &regno))
	    /* Encode instruction */
	    ace_encode_insn (regno, ace_op, ip);

	  /* Update the address of pointer of assembly code (*str) */
	  if (!found_asm_end)
	    *str += 1;
	}
      break;

    case HW_VR:
	{
	  unsigned int regno;
	  /* Extract the VR index string from assembly code (*str) */
	  if (reg_lookup (str, RCLASS_VECR, &regno))
	    /* Encode instruction */
	    ace_encode_insn (regno, ace_op, ip);

	  /* Update the address of pointer of assembly code (*str) */
	  if (!found_asm_end)
	    *str += 1;
	}
      break;

    case HW_UINT:
	{
	  /* Extract the IMM value string from assembly code (*str) */
	  char *p = strchr (*str, ',');
	  if (p == NULL)
	    p = strchr (*str, '\0');
	  if (p == NULL)
	    as_fatal (_("Broken assembler.  No IMM value is given."));
	  unsigned int imm_size = p - *str;
	  char *imm = malloc (imm_size + 1);
	  memcpy (imm, *str, imm_size);
	  memcpy (imm + imm_size, "\0", 1);
	  unsigned int imm_value = strtoul (imm, (char **) NULL, 0);

	  /* Encode instruction */
	  if (is_discrete)
	    ace_encode_insn_discrete (imm_value, op_name_discrete, "imm",  ip);
	  else
	    ace_encode_insn (imm_value, ace_op, ip);

	  /* Update the address of pointer of assembly code (*str) */
	  if (found_asm_end)
	    *str += imm_size;
	  else
	    *str += imm_size + 1;
	}
      break;

    case HW_ACR:
	{
	  /* Extract the ACR register index string from assembly code (*str) */
	  char *p = strchr (*str, ',');
	  if (p == NULL)
	    p = strchr (*str, '\0');
	  if (p == NULL)
	    as_fatal (_("No ACR register index is given."));
	  unsigned int reg_idx_size = p - *str;
	  char *reg_idx = malloc (reg_idx_size + 1);
	  memcpy (reg_idx, *str, reg_idx_size);
	  // always append EOL at ACR index string
	  memcpy (reg_idx + reg_idx_size, "\0", 1);

	  /* Find the digit number of ACR register index string */
	  ace_keyword_t *ace_reg =
	    (ace_keyword_t *) hash_find (ace_keyword_hash, reg_idx);
	  if (ace_reg != NULL)
	    /* Encode instruction */
	    if (is_discrete)
	      ace_encode_insn_discrete (ace_reg->value, op_name_discrete, ace_op->hw_name,  ip);
	    else
	      ace_encode_insn (ace_reg->value, ace_op, ip);
	  else
	    as_fatal (_("Wrong ACR register index (%s)) is given."), reg_idx);

	  /* Update the address of pointer of assembly code (*str) */
	  if (found_asm_end)
	    *str += reg_idx_size;
	  else
	    *str += reg_idx_size + 1;
	}
      break;

    default:
      as_fatal (_("Broken assembler.  Cannot find field attribute."));
    }

  /* Update the address of pointer of the field attribute (*args) */
  if (found_op_str_end == TRUE)
    *args = pch - 1;
  else
    *args = pch;
}

/* The information of architecture attribute.  */
struct arch_info
{
  const char *name;
  const char *v_major;
  const char *v_minor;
  int valid;
};

struct arch_info arch_info[] =
{
/* Standard arch info.  */
{"e", "1", "9", 0}, {"i", "2", "0", 0}, {"m", "2", "0", 0},
{"a", "2", "0", 0}, {"f", "2", "0", 0}, {"d", "2", "0", 0},
{"q", "2", "0", 0}, {"c", "2", "0", 0}, {"p", "2", "0", 0},

/* Terminate the list.  */
{0, 0, 0, 0}
};

#ifdef TO_REMOVE
/* Generally, we do not allow the name of non-standard
   arch includes numbers, but you can define your own
   special arch name here.  */
const char *non_standard_arch_name[] = {"xv5"};

static struct hash_control *arch_info_hash = NULL;
#define DEFAULT_PRIV_SPEC 1
#define DEFAULT_PRIV_SPEC_MINOR 10
#define DEFAULT_PRIV_SPEC_REVISION 0
#define DEFAULT_STRICT_ALIGN 0
#define DEFAULT_STACK_ALIGN 0
#endif
#define DEFAULT_ICT_VERSION 1

#if 0
static void
arch_info_hash_init (void)
{
  int i = 0;
  arch_info_hash = hash_new ();

  for (; arch_info[i].name; i++)
    {
      const char *name = arch_info[i].name;
      const char *hash_error =
	hash_insert (arch_info_hash, name, (void *) &arch_info[i]);
      if (hash_error)
	{
	  fprintf (stderr, _("internal error: can't hash `%s': %s\n"),
		   arch_info[i].name, hash_error);
	  /* Probably a memory allocation problem?  Give up now.  */
	  as_fatal (_("Broken assembler.  No assembly attempted."));
	}
    }
}
#endif

/* This function is called once, at assembler startup time.  It should set up
   all the tables, etc. that the MD part of the assembler will need.  */

void
md_begin (void)
{
  unsigned long mach = xlen == 64 ? bfd_mach_riscv64 : bfd_mach_riscv32;

  if (! bfd_set_arch_mach (stdoutput, bfd_arch_riscv, mach))
    as_warn (_("Could not set architecture and machine"));

  op_hash = init_opcode_hash (riscv_opcodes, FALSE);
  insn_type_hash = init_opcode_hash (riscv_insn_types, TRUE);

  reg_names_hash = hash_new ();
  hash_reg_names (RCLASS_GPR, riscv_gpr_names_numeric, NGPR);
  hash_reg_names (RCLASS_GPR, riscv_gpr_names_abi, NGPR);
  hash_reg_names (RCLASS_GPR, riscv_gpr_names_standard, NGPR);
  hash_reg_names (RCLASS_FPR, riscv_fpr_names_numeric, NFPR);
  hash_reg_names (RCLASS_FPR, riscv_fpr_names_abi, NFPR);
  hash_reg_names (RCLASS_VECR, riscv_vecr_names_numeric, NVECR);
  hash_reg_names (RCLASS_VECM, riscv_vecm_names_numeric, NVECM);
  /* Add "fp" as an alias for "s0".  */
  hash_reg_name (RCLASS_GPR, "fp", 8);

  /* Create and insert CSR hash tables.  */
  csr_extra_hash = hash_new ();
#define DECLARE_CSR(name, num, class) riscv_init_csr_hashes (#name, num, class);
#define DECLARE_CSR_ALIAS(name, num, class) DECLARE_CSR(name, num, class);
#include "opcode/riscv-opc.h"
#undef DECLARE_CSR

  opcode_names_hash = hash_new ();
  init_opcode_names_hash ();

  /* Set the default alignment for the text section.  */
  record_alignment (text_section, riscv_opts.rvc ? 1 : 2);

  /* Load symbols from ACE shared library if exists */
  if (ace_lib_load_success)
    {
      int i;

      /* Insert instruction information in a hash table */
      i = 0;
      while (ace_opcs[i].name)
	{
	  const char *name = ace_opcs[i].name;
	  const char *hash_error =
	    hash_insert (op_hash, name, (void *) &ace_opcs[i]);
	  if (hash_error)
	    {
	      fprintf (stderr, _("internal error: can't hash `%s': %s\n"),
		       name, hash_error);
	      as_fatal (_("Broken assembler.  No assembly attempted."));
	    }
	  i++;
	}

      /* Insert ACR index name in a hash table */
      ace_keyword_hash = hash_new ();
      i = 0;
      while (ace_keys[i].name)
	{
	  const char *name = ace_keys[i].name;
	  const char *hash_error =
	    hash_insert (ace_keyword_hash, name, (void *) &ace_keys[i]);
	  if (hash_error)
	    {
	      fprintf (stderr, _("internal error: can't hash `%s': %s\n"),
		       name, hash_error);
	      as_fatal (_("Broken assembler.  No assembly attempted."));
	    }
	  i++;
	}

      /* Insert operand field information in a hash table */
      ace_op_hash = hash_new ();
      i = 0;
      while (ace_ops[i].name)
	{
	  const char *name = ace_ops[i].name;
	  const char *hash_error =
	    hash_insert (ace_op_hash, name, (void *) &ace_ops[i]);
	  if (hash_error)
	    {
	      fprintf (stderr, _("internal error: can't hash `%s': %s\n"),
		       name, hash_error);
	      as_fatal (_("Broken assembler.  No assembly attempted."));
	    }
	  i++;
	}
    }
}

static insn_t
riscv_apply_const_reloc (bfd_reloc_code_real_type reloc_type, bfd_vma value)
{
  switch (reloc_type)
    {
    case BFD_RELOC_32:
      return value;

    case BFD_RELOC_RISCV_HI20:
    case BFD_RELOC_RISCV_LALO_HI20:
    case BFD_RELOC_RISCV_ICT_HI20:
      return ENCODE_UTYPE_IMM (RISCV_CONST_HIGH_PART (value));

    case BFD_RELOC_RISCV_LO12_S:
      return ENCODE_STYPE_IMM (value);

    case BFD_RELOC_RISCV_LO12_I:
    case BFD_RELOC_RISCV_LALO_LO12_I:
    case BFD_RELOC_RISCV_ICT_LO12_I:
      return ENCODE_ITYPE_IMM (value);

    default:
      abort ();
    }
}

static void
riscv_make_nops (char *buf, bfd_vma bytes)
{
  bfd_vma i = 0;

  /* RISC-V instructions cannot begin or end on odd addresses, so this case
     means we are not within a valid instruction sequence.  It is thus safe
     to use a zero byte, even though that is not a valid instruction.  */
  if (bytes % 2 == 1)
    buf[i++] = 0;

  /* Use at most one 2-byte NOP.  */
  if ((bytes - i) % 4 == 2)
    {
      md_number_to_chars (buf + i, RVC_NOP, 2);
      i += 2;
    }

  /* Fill the remainder with 4-byte NOPs.  */
  for ( ; i < bytes; i += 4)
    md_number_to_chars (buf + i, RISCV_NOP, 4);
}

static int align_call = 0;

/* Output an instruction.  IP is the instruction information.
   ADDRESS_EXPR is an operand of the instruction to be used with
   RELOC_TYPE.  */

static void
append_insn (struct riscv_cl_insn *ip, expressionS *address_expr,
	     bfd_reloc_code_real_type reloc_type)
{
  dwarf2_emit_insn (0);

  if (reloc_type != BFD_RELOC_UNUSED)
    {
      reloc_howto_type *howto;

      gas_assert (address_expr);
      if (reloc_type == BFD_RELOC_12_PCREL
	  || reloc_type == BFD_RELOC_RISCV_JMP
	  || reloc_type == BFD_RELOC_RISCV_10_PCREL)
	{
	  int j = reloc_type == BFD_RELOC_RISCV_JMP;
	  int best_case = riscv_insn_length (ip->insn_opcode);
	  int range = ENUM_BRANCH_RANGE (reloc_type);
	  unsigned worst_case = relaxed_branch_length (NULL, NULL, 0);
	  add_relaxed_insn (ip, worst_case, best_case,
			    RELAX_BRANCH_ENCODE (j, best_case == 2,
						 worst_case, range),
			    address_expr->X_add_symbol,
			    address_expr->X_add_number);
	  return;
	}
      else if (ip->cmodel.method == METHOD_DEFAULT)
	{
	  howto = bfd_reloc_type_lookup (stdoutput, reloc_type);
	  if (howto == NULL)
	    as_bad (_("Unsupported RISC-V relocation number %d"), reloc_type);

	  ip->fixp = fix_new_exp (ip->frag, ip->where,
				  bfd_get_reloc_size (howto),
				  address_expr, FALSE, reloc_type);

	  ip->fixp->fx_tcbit = riscv_opts.relax;
	}
    }

  if (ip->cmodel.method == METHOD_DEFAULT)
    {
      add_fixed_insn (ip);
      if (riscv_opts.workaround)
	{
	  if ((riscv_opts.b22827 || riscv_opts.b22827_1)
	      && !riscv_subset_supports ("v"))
	    {
	      const struct riscv_opcode *insn = ip->insn_mo;

	      /* insert “fclass.x x0, RD(FDIV/FSQRT)” after FDIV/FSQRT unless
	       * the next immediate instruction is
	       * “fsub/fadd/fmul/fmadd/fsqrt/fdiv/jal/ret and their 16bit variants”
	       * NOTE: by jal I mean jal and jral. Ret includes jr.
	       * If you can accept more complex conditions, RD(FDIV/FSQRT) has to be
	       * in fa0-7 to exclude jal/ret.
	       */
	      if (riscv_opts.b22827 && is_insn_fdiv_or_fsqrt (insn))
		{
		  const char *mne = "fclass.d";
		  if (is_insn_fmt_s (ip->insn_opcode))
		    mne = "fclass.s";
		  nds.frag_b22827 = frag_now;
		  macro_build(NULL, mne, "d,s,C", 0, insn_fp_rd(ip->insn_opcode),
			      0, TYPE_B22827, CSI_B22827, 0);
		}

	      /* to provide a separate flag to turn it off, with the following rule:
	       * If FSHW is followed by any floating-point instructions (including
	       * FSHW and FLHW), insert a NOP after it.
	       */
	      else if (riscv_opts.b22827_1 && is_insn_fshw (insn))
		{
		  nds.frag_b22827 = frag_now;
		  macro_build(NULL, "nop", "C",
			      0, TYPE_B22827_1, CSI_B22827_1, 0);
		}
	    }
	}
    }
  else if  (ip->cmodel.method == METHOD_VARIABLE)
    {
      add_insn_grow (ip);
      if (ip->cmodel.state == 0)
	{
	  int length = ip->cmodel.offset + 4;
	  symbolS *symbol = address_expr ? address_expr->X_add_symbol : NULL;
	  offsetT offset = address_expr ? address_expr->X_add_number : 0;
	  add_insn_grow_done (ip, length, 0,
			      RELAX_CMODEL_ENCODE (ip->cmodel.type, length, ip->cmodel.index),
			      symbol, offset);
	}
      return;
    }
  else
    as_fatal (_("internal error: invalid append_insn method!"));

  /* We need to start a new frag after any instruction that can be
     optimized away or compressed by the linker during relaxation, to prevent
     the assembler from computing static offsets across such an instruction.
     This is necessary to get correct EH info.  */
  if (reloc_type == BFD_RELOC_RISCV_CALL
      || reloc_type == BFD_RELOC_RISCV_CALL_PLT
      || reloc_type == BFD_RELOC_RISCV_HI20
      || reloc_type == BFD_RELOC_RISCV_PCREL_HI20
      || reloc_type == BFD_RELOC_RISCV_TPREL_HI20
      || reloc_type == BFD_RELOC_RISCV_TPREL_ADD)
    {
      frag_wane (frag_now);
      frag_new (0);
    }

  /* TODO: The following code is used for doing target aligned
     and avoiding BTB miss.  Should it be here?  */
  /* Do not do target aligned and avoid BTB miss when Os and
     .option norelax.  */
  /* We do target aligned and avoid BTB miss only when RVC is enabled.  */
  if (optimize && riscv_opts.verbatim && riscv_opts.relax && riscv_opts.rvc
      && ((align_call && ((ip->insn_opcode >> 7 & 0x1f)) != 0)
	  || (!align_call && ((ip->insn_opcode & MASK_JALR) == MATCH_JALR))))
    {
      char *nops = frag_more (4);
      expressionS ex;
      ex.X_op = O_constant;
      ex.X_add_number = 4;

      riscv_make_nops (nops, 4);
      fix_new_exp (frag_now, nops - frag_now->fr_literal, 0,
		   &ex, FALSE, BFD_RELOC_RISCV_ALIGN_BTB);
    }
  align_call = 0;

  if (reloc_type == BFD_RELOC_RISCV_CALL)
    align_call = 1;
}

/* Build an instruction created by a macro expansion.  This is passed
   a pointer to the count of instructions created so far, an
   expression, the name of the instruction to build, an operand format
   string, and corresponding arguments.  */

static void
macro_build (expressionS *ep, const char *name, const char *fmt, ...)
{
  const struct riscv_opcode *mo;
  struct riscv_cl_insn insn;
  bfd_reloc_code_real_type r;
  va_list args;

  insn.cmodel.method = METHOD_DEFAULT;
  va_start (args, fmt);

  r = BFD_RELOC_UNUSED;
  mo = (struct riscv_opcode *) hash_find (op_hash, name);
  gas_assert (mo);

  /* Find a non-RVC variant of the instruction.  append_insn will compress
     it if possible.  */
  while (riscv_insn_length (mo->match) < 4)
    mo++;
  gas_assert (strcmp (name, mo->name) == 0);

  create_insn (&insn, mo);
  for (;;)
    {
      switch (*fmt++)
	{
	case 'd':
	  INSERT_OPERAND (RD, insn, va_arg (args, int));
	  continue;

	case 's':
	  INSERT_OPERAND (RS1, insn, va_arg (args, int));
	  continue;

	case 't':
	  INSERT_OPERAND (RS2, insn, va_arg (args, int));
	  continue;

	case '>':
	  INSERT_OPERAND (SHAMT, insn, va_arg (args, int));
	  continue;

	case 'j':
	case 'u':
	case 'q':
	  gas_assert (ep != NULL);
	  r = va_arg (args, int);
	  continue;

	case '\0':
	  break;
	case ',':
	  continue;

	case 'V': /* RVV */
	  {
	    switch (*fmt++)
	      {
	      case 'd':
		INSERT_OPERAND (VD, insn, va_arg (args, int));
		continue;

	      case 's':
		INSERT_OPERAND (VS1, insn, va_arg (args, int));
		continue;

	      case 't':
		INSERT_OPERAND (VS2, insn, va_arg (args, int));
		continue;

	      case 'm':
		{
		  int reg = va_arg (args, int);
		  if (reg == -1)
		    {
		      INSERT_OPERAND (VMASK, insn, 1);
		      continue;
		    }
		  else if (reg == 0)
		    {
		      INSERT_OPERAND (VMASK, insn, 0);
		      continue;
		    }
		}
		/* fallthru */
	      }
	  }
	  /* fallthru */

	case 'C':
	  insn.cmodel.method = METHOD_VARIABLE;
	  insn.cmodel.state = va_arg (args, int);
	  insn.cmodel.type = va_arg (args, int);
	  insn.cmodel.index = va_arg (args, int);
	  insn.cmodel.offset = va_arg (args, int);
	  continue;

	case 'P':
	  INSERT_OPERAND (PRED, insn, va_arg (args, int));
	  continue;
	case 'Q':
	  INSERT_OPERAND (SUCC, insn, va_arg (args, int));
	  continue;

	default:
	  as_fatal (_("internal error: invalid macro"));
	}
      break;
    }
  va_end (args);
  gas_assert (r == BFD_RELOC_UNUSED ? ep == NULL : ep != NULL);

  append_insn (&insn, ep, r);
}

/* Sign-extend 32-bit mode constants that have bit 31 set and all higher bits
   unset.  */
static void
normalize_constant_expr (expressionS *ex)
{
  if (xlen > 32)
    return;
  if ((ex->X_op == O_constant || ex->X_op == O_symbol)
      && IS_ZEXT_32BIT_NUM (ex->X_add_number))
    ex->X_add_number = (((ex->X_add_number & 0xffffffff) ^ 0x80000000)
			- 0x80000000);
}

/* Fail if an expression EX is not a constant.  IP is the instruction using EX.
   MAYBE_CSR is true if the symbol may be an unrecognized CSR name.  */

static void
check_absolute_expr (struct riscv_cl_insn *ip, expressionS *ex,
		     bfd_boolean maybe_csr)
{
  if (ex->X_op == O_big)
    as_bad (_("unsupported large constant"));
  else if (maybe_csr && ex->X_op == O_symbol)
    as_bad (_("unknown CSR `%s'"),
	    S_GET_NAME (ex->X_add_symbol));
  else if (ex->X_op != O_constant)
    as_bad (_("Instruction %s requires absolute expression"),
	    ip->insn_mo->name);
  normalize_constant_expr (ex);
}

static symbolS *
make_internal_label (void)
{
  return (symbolS *) local_symbol_make (FAKE_LABEL_NAME, now_seg,
					(valueT) frag_now_fix (), frag_now);
}

#define CMODEL_SUBSECTION 8100
#define CMODEL_SYMBOL_PREFIX ".Laddr"
#define CMODEL_SECTION_ALIGN 3
#define CMODEL_SECTION_ENTRY_SIZE (1u << CMODEL_SECTION_ALIGN)

static
void  make_indirect_symbol (expressionS *ep, expressionS *ep_ind)
{
  char buf[0x100];
  char isym_name[0x100];
  symbolS *isym;
  const char *seg_name = segment_name (now_seg);
  const char *sym_name = S_GET_NAME (ep->X_add_symbol);
  valueT sym_addend = ep->X_add_number;

  /* make indirect symbol once  */
  sprintf (isym_name, "%s_%s_%s_%lx", CMODEL_SYMBOL_PREFIX, seg_name,
	   sym_name, (unsigned long)sym_addend);
  isym = symbol_find (isym_name);
  if (isym == NULL)
    {
      /* create indirect symbol:
       *   # .pushsection subsection
       *   # make indirect symbol
       *   # new variable fragment (grow frag, pend reloc till md_convert)
       *   # .popsection
       */
      const char *sec_name = segment_name (now_seg);
      char *save_in;
      /* #  */
      sprintf (buf, "%s, %d", sec_name, CMODEL_SUBSECTION);
      save_in = input_line_pointer;
      input_line_pointer = buf;
      obj_elf_section (1); /* .pushsection  */
      input_line_pointer = save_in;
      /* #  */
      isym = colon (isym_name);
      /* #  */
      frag_var (rs_machine_dependent, CMODEL_SECTION_ENTRY_SIZE, 0,
		RELAX_CMODEL_ENCODE (TYPE_IS, CMODEL_SECTION_ENTRY_SIZE, 0),
		ep->X_add_symbol, ep->X_add_number, NULL);
      /* #  */
      obj_elf_popsection (0); /* .popsection  */
    }

  ep_ind->X_op = O_symbol;
  ep_ind->X_add_symbol = isym;
  ep_ind->X_add_number = 0;
  ep_ind->X_md = 0; /* for ICT logic within fix_new_exp */
}

/* Load an entry from the GOT.  */
static void
pcrel_access (int destreg, int tempreg, expressionS *ep,
	      const char *lo_insn, const char *lo_pattern,
	      bfd_reloc_code_real_type hi_reloc,
	      bfd_reloc_code_real_type lo_reloc)
{
  /* only L[BHWD]/S[BHWD support cmodel large  */
  if (hi_reloc == BFD_RELOC_RISCV_PCREL_HI20
      && is_cmodel_relaxable (ep->X_add_symbol, now_seg))
    {
      gas_assert (ep->X_op == O_symbol);
      char lo_pattern_ex[0x100];
      int index, type;
      expressionS ep_ind, ep_ref;
      bfd_boolean is_la = strcmp (lo_insn, "addi") == 0;
      bfd_boolean is_st = lo_reloc == BFD_RELOC_RISCV_PCREL_LO12_S;
      make_indirect_symbol (ep, &ep_ind);
      ep_ref.X_op = O_symbol;
      ep_ref.X_add_symbol = make_internal_label ();
      ep_ref.X_add_number = 0;
      ep_ref.X_md = 0; /* for ICT logic within fix_new_exp */
      type = is_st ? TYPE_ST : is_la ? TYPE_LA : TYPE_LD;

      strcpy (lo_pattern_ex, lo_pattern);
      strcat (lo_pattern_ex, ",C");

      /* ensure folloiwng instructions continously within the same frag.  */
      frag_grow(4 * 5);

      /* index 0: argument, C: state, type, index, offset.  */
      index = CSI_INDIRECT_SYMBOL;
      macro_build (&ep_ind, "nop", "j,C", hi_reloc, 0, type, index, 0);

      /* index 1: argument, C: state, type, index, offset.  */
      index++; /* CSI_REFERENCE_SYMBOL  */
      macro_build (&ep_ref, "nop", "j,C", hi_reloc, 0, type, index, 0);

      /* index 2: generic form, C: state, type, index, offset.  */
      index++; /* CSI_LARGE_CODE  */
      macro_build (&ep_ind, "auipc", "d,u,C", tempreg, hi_reloc, 1, type, index, 0);
      macro_build (&ep_ref, "ld", "d,s,j,C", tempreg, tempreg, hi_reloc, 1, type, index, 4);
      macro_build (ep, lo_insn, lo_pattern_ex, destreg, tempreg, lo_reloc, 0, type, index, 8);

      /* CSI_DEFAULT_CODE can be extracted from CSI_LARGE_CODE  */
    }
  else
    {
      expressionS ep2;
      ep2.X_op = O_symbol;
      ep2.X_add_symbol = make_internal_label ();
      ep2.X_add_number = 0;
      ep2.X_md = 0; /* for ICT logic within fix_new_exp */

      macro_build (ep, "auipc", "d,u", tempreg, hi_reloc);
      macro_build (&ep2, lo_insn, lo_pattern, destreg, tempreg, lo_reloc);
    }
}

static void
pcrel_load (int destreg, int tempreg, expressionS *ep, const char *lo_insn,
	    bfd_reloc_code_real_type hi_reloc,
	    bfd_reloc_code_real_type lo_reloc)
{
  gas_assert (lo_reloc == BFD_RELOC_RISCV_PCREL_LO12_I);
  pcrel_access (destreg, tempreg, ep, lo_insn, "d,s,j", hi_reloc, lo_reloc);
}

static void
pcrel_store (int srcreg, int tempreg, expressionS *ep, const char *lo_insn,
	     bfd_reloc_code_real_type hi_reloc,
	     bfd_reloc_code_real_type lo_reloc)
{
  gas_assert (lo_reloc == BFD_RELOC_RISCV_PCREL_LO12_S);
  pcrel_access (srcreg, tempreg, ep, lo_insn, "t,s,q", hi_reloc, lo_reloc);
}

/* PC-relative function call using AUIPC/JALR, relaxed to JAL.  */
static void
 riscv_call (int destreg, int tempreg, expressionS *ep,
	    bfd_reloc_code_real_type reloc)
{
  if (reloc == BFD_RELOC_RISCV_CALL
      && is_cmodel_relaxable (ep->X_add_symbol, now_seg))
    {
      gas_assert (ep->X_op == O_symbol);
      int index;
      expressionS ep_ind, ep_ref;
      make_indirect_symbol (ep, &ep_ind);
      ep_ref.X_op = O_symbol;
      ep_ref.X_add_symbol = make_internal_label ();
      ep_ref.X_add_number = 0;
      ep_ref.X_md = 0; /* for ICT logic within fix_new_exp */

      /* ensure folloiwng instructions continously within the same frag.  */
      frag_grow(4 * 7);

      /* index 0: argument, C: state, type, index, offset.  */
      index = CSI_INDIRECT_SYMBOL;
      macro_build (&ep_ind, "nop", "j,C", reloc, 0, TYPE_JX, index, 0);

      /* index 1: argument, C: state, type, index, offset.  */
      index++; /* CSI_REFERENCE_SYMBOL  */
      macro_build (&ep_ref, "nop", "j,C", reloc, 0, TYPE_JX, index, 0);

      /* index 2: generic form, C: state, type, index, offset.  */
      index++; /* CSI_LARGE_CODE  */
      macro_build (&ep_ind, "auipc", "d,u,C", tempreg, reloc, 1, TYPE_JX, index, 0);
      macro_build (&ep_ref, "ld", "d,s,j,C", tempreg, tempreg, reloc, 1, TYPE_JX, index, 4);
      macro_build (ep, "jalr", "d,s,j,C", destreg, tempreg, reloc, 0, TYPE_JX, index, 8);

      /* index 3: relaxed form  */
      index++; /* CSI_DEFAULT_CODE  */
      macro_build (ep, "auipc", "d,u,C", tempreg, reloc, 1, TYPE_JX, index, 0);
      macro_build (ep, "jalr", "d,s,j,C", destreg, tempreg, reloc, 0, TYPE_JX, index, 4);
    }
  else
    {
      macro_build (ep, "auipc", "d,u", tempreg, reloc);
      macro_build (NULL, "jalr", "d,s", destreg, tempreg);
    }
}

/* Build an instruction created by a macro expansion.  Like md_assemble but
   accept a printf-style format string and arguments.  */

static void
md_assemblef (const char *format, ...)
{
  char *buf = NULL;
  va_list ap;
  int r;

  va_start (ap, format);

  r = vasprintf (&buf, format, ap);

  if (r < 0)
    as_fatal (_("internal: vasprintf failed"));

  md_assemble (buf);
  free(buf);

  va_end (ap);
}

/* Zero extend and sign extend byte/half-word/word.  */

static void
riscv_ext (int destreg, int srcreg, unsigned shift, bfd_boolean sign)
{
  if (sign)
    {
      md_assemblef ("slli x%d, x%d, 0x%x", destreg, srcreg, shift);
      md_assemblef ("srai x%d, x%d, 0x%x", destreg, destreg, shift);
    }
  else
    {
      md_assemblef ("slli x%d, x%d, 0x%x", destreg, srcreg, shift);
      md_assemblef ("srli x%d, x%d, 0x%x", destreg, destreg, shift);
    }
}

/* Load an integer constant into a register.  */

static void
load_const (int reg, expressionS *ep)
{
  int shift = RISCV_IMM_BITS;
  expressionS upper = *ep, lower = *ep;
  lower.X_add_number = (int32_t) ep->X_add_number << (32-shift) >> (32-shift);
  upper.X_add_number -= lower.X_add_number;

  if (ep->X_op != O_constant)
    {
      as_bad (_("unsupported large constant"));
      return;
    }

  if (xlen > 32 && !IS_SEXT_32BIT_NUM (ep->X_add_number))
    {
      /* Reduce to a signed 32-bit constant using SLLI and ADDI.  */
      while (((upper.X_add_number >> shift) & 1) == 0)
	shift++;

      upper.X_add_number = (int64_t) upper.X_add_number >> shift;
      load_const (reg, &upper);

      macro_build (NULL, "slli", "d,s,>", reg, reg, shift);
      if (lower.X_add_number != 0)
	macro_build (&lower, "addi", "d,s,j", reg, reg, BFD_RELOC_RISCV_LO12_I);
    }
  else
    {
      /* Simply emit LUI and/or ADDI to build a 32-bit signed constant.  */
      int hi_reg = 0;

      if (upper.X_add_number != 0)
	{
	  macro_build (ep, "lui", "d,u", reg, BFD_RELOC_RISCV_HI20);
	  hi_reg = reg;
	}

      if (lower.X_add_number != 0 || hi_reg == 0)
	macro_build (ep, ADD32_INSN, "d,s,j", reg, hi_reg,
		     BFD_RELOC_RISCV_LO12_I);
    }
}

/* Expand RISC-V Vector macros into one of more instructions.  */

static void
vector_macro (struct riscv_cl_insn *ip)
{
  int vd = (ip->insn_opcode >> OP_SH_VD) & OP_MASK_VD;
  int vs1 = (ip->insn_opcode >> OP_SH_VS1) & OP_MASK_VS1;
  int vs2 = (ip->insn_opcode >> OP_SH_VS2) & OP_MASK_VS2;
  int vm = (ip->insn_opcode >> OP_SH_VMASK) & OP_MASK_VMASK;
  int vtemp = (ip->insn_opcode >> OP_SH_VFUNCT6) & OP_MASK_VFUNCT6;
  int mask = ip->insn_mo->mask;

  switch (mask)
    {
    case M_VMSGE:
      if (vm)
	{
	  /* Unmasked.  */
	  macro_build (NULL, "vmslt.vx", "Vd,Vt,sVm", vd, vs2, vs1, -1);
	  macro_build (NULL, "vmnand.mm", "Vd,Vt,Vs", vd, vd, vd);
	  break;
	}
      if (vtemp != 0)
	{
	  /* Masked.  Have vtemp to avoid overlap constraints.  */
	  if (vd == vm)
	    {
	      macro_build (NULL, "vmslt.vx", "Vd,Vt,s", vtemp, vs2, vs1);
	      macro_build (NULL, "vmandnot.mm", "Vd,Vt,Vs", vd, vm, vtemp);
	    }
	  else
	    {
	      /* Preserve the value of vd if not updating by vm.  */
	      macro_build (NULL, "vmslt.vx", "Vd,Vt,s", vtemp, vs2, vs1);
	      macro_build (NULL, "vmandnot.mm", "Vd,Vt,Vs", vtemp, vm, vtemp);
	      macro_build (NULL, "vmandnot.mm", "Vd,Vt,Vs", vd, vd, vm);
	      macro_build (NULL, "vmor.mm", "Vd,Vt,Vs", vd, vtemp, vd);
	    }
	}
      else if (vd != vm)
	{
	  /* Masked.  This may cause the vd overlaps vs2, when LMUL > 1.  */
	  macro_build (NULL, "vmslt.vx", "Vd,Vt,sVm", vd, vs2, vs1, vm);
	  macro_build (NULL, "vmxor.mm", "Vd,Vt,Vs", vd, vd, vm);
	}
      else
	as_bad (_("must provide temp if destination overlaps mask"));
      break;

    case M_VMSGEU:
      if (vm)
	{
	  /* Unmasked.  */
	  macro_build (NULL, "vmsltu.vx", "Vd,Vt,sVm", vd, vs2, vs1, -1);
	  macro_build (NULL, "vmnand.mm", "Vd,Vt,Vs", vd, vd, vd);
	  break;
	}
      if (vtemp != 0)
	{
	  /* Masked.  Have vtemp to avoid overlap constraints.  */
	  if (vd == vm)
	    {
	      macro_build (NULL, "vmsltu.vx", "Vd,Vt,s", vtemp, vs2, vs1);
	      macro_build (NULL, "vmandnot.mm", "Vd,Vt,Vs", vd, vm, vtemp);
	    }
	  else
	    {
	      /* Preserve the value of vd if not updating by vm.  */
	      macro_build (NULL, "vmsltu.vx", "Vd,Vt,s", vtemp, vs2, vs1);
	      macro_build (NULL, "vmandnot.mm", "Vd,Vt,Vs", vtemp, vm, vtemp);
	      macro_build (NULL, "vmandnot.mm", "Vd,Vt,Vs", vd, vd, vm);
	      macro_build (NULL, "vmor.mm", "Vd,Vt,Vs", vd, vtemp, vd);
	    }
	}
      else if (vd != vm)
	{
	  /* Masked.  This may cause the vd overlaps vs2, when LMUL > 1.  */
	  macro_build (NULL, "vmsltu.vx", "Vd,Vt,sVm", vd, vs2, vs1, vm);
	  macro_build (NULL, "vmxor.mm", "Vd,Vt,Vs", vd, vd, vm);
	}
      else
	as_bad (_("must provide temp if destination overlaps mask"));
      break;

    default:
      as_bad (_("Macro %s not implemented"), ip->insn_mo->name);
      break;
    }
}

/* Expand RISC-V assembly macros into one or more instructions.  */
static void
macro (struct riscv_cl_insn *ip, expressionS *imm_expr,
       bfd_reloc_code_real_type *imm_reloc)
{
  int rd = (ip->insn_opcode >> OP_SH_RD) & OP_MASK_RD;
  int rs1 = (ip->insn_opcode >> OP_SH_RS1) & OP_MASK_RS1;
  int rs2 = (ip->insn_opcode >> OP_SH_RS2) & OP_MASK_RS2;
  int mask = ip->insn_mo->mask;

  switch (mask)
    {
    case M_LI:
      load_const (rd, imm_expr);
      break;

    case M_LA:
    case M_LLA:
      /* Load the address of a symbol into a register.  */
      if (!IS_SEXT_32BIT_NUM (imm_expr->X_add_number))
	as_bad (_("offset too large"));

      /* For bug 14220.  */
      if (rd == 2)
	as_warn (_("Load address into sp directly may let sp "
		   "unaligned for auipc insn.\n"
		   "Please load address into a temp register!\n"
		   "\"la/lla a0, value; mv sp, a0\""));

      if (imm_expr->X_op == O_constant)
	load_const (rd, imm_expr);
      else if (riscv_opts.pic && mask == M_LA) /* Global PIC symbol */
	pcrel_load (rd, rd, imm_expr, LOAD_ADDRESS_INSN,
		    BFD_RELOC_RISCV_GOT_HI20, BFD_RELOC_RISCV_PCREL_LO12_I);
      else /* Local PIC symbol, or any non-PIC symbol */
	pcrel_load (rd, rd, imm_expr, "addi",
		    BFD_RELOC_RISCV_PCREL_HI20, BFD_RELOC_RISCV_PCREL_LO12_I);
      break;

    case M_LA_LO:
      /* Load the low 32-bit address of a 64-bit symbol into a register.  */
      macro_build (imm_expr, "lui", "d,u", rd, BFD_RELOC_RISCV_LALO_HI20);
      macro_build (imm_expr, ADD32_INSN, "d,s,j", rd, rd, BFD_RELOC_RISCV_LALO_LO12_I);
      break;

    case M_LA_TLS_GD:
      pcrel_load (rd, rd, imm_expr, "addi",
		  BFD_RELOC_RISCV_TLS_GD_HI20, BFD_RELOC_RISCV_PCREL_LO12_I);
      break;

    case M_LA_TLS_IE:
      pcrel_load (rd, rd, imm_expr, LOAD_ADDRESS_INSN,
		  BFD_RELOC_RISCV_TLS_GOT_HI20, BFD_RELOC_RISCV_PCREL_LO12_I);
      break;

    case M_LB:
      pcrel_load (rd, rd, imm_expr, "lb",
		  BFD_RELOC_RISCV_PCREL_HI20, BFD_RELOC_RISCV_PCREL_LO12_I);
      break;

    case M_LBU:
      pcrel_load (rd, rd, imm_expr, "lbu",
		  BFD_RELOC_RISCV_PCREL_HI20, BFD_RELOC_RISCV_PCREL_LO12_I);
      break;

    case M_LH:
      pcrel_load (rd, rd, imm_expr, "lh",
		  BFD_RELOC_RISCV_PCREL_HI20, BFD_RELOC_RISCV_PCREL_LO12_I);
      break;

    case M_LHU:
      pcrel_load (rd, rd, imm_expr, "lhu",
		  BFD_RELOC_RISCV_PCREL_HI20, BFD_RELOC_RISCV_PCREL_LO12_I);
      break;

    case M_LW:
      pcrel_load (rd, rd, imm_expr, "lw",
		  BFD_RELOC_RISCV_PCREL_HI20, BFD_RELOC_RISCV_PCREL_LO12_I);
      break;

    case M_LWU:
      pcrel_load (rd, rd, imm_expr, "lwu",
		  BFD_RELOC_RISCV_PCREL_HI20, BFD_RELOC_RISCV_PCREL_LO12_I);
      break;

    case M_LD:
      pcrel_load (rd, rd, imm_expr, "ld",
		  BFD_RELOC_RISCV_PCREL_HI20, BFD_RELOC_RISCV_PCREL_LO12_I);
      break;

    case M_FLH:
      pcrel_load (rd, rs1, imm_expr, "flh",
		  BFD_RELOC_RISCV_PCREL_HI20, BFD_RELOC_RISCV_PCREL_LO12_I);
      break;

    case M_FLW:
      pcrel_load (rd, rs1, imm_expr, "flw",
		  BFD_RELOC_RISCV_PCREL_HI20, BFD_RELOC_RISCV_PCREL_LO12_I);
      break;

    case M_FLD:
      pcrel_load (rd, rs1, imm_expr, "fld",
		  BFD_RELOC_RISCV_PCREL_HI20, BFD_RELOC_RISCV_PCREL_LO12_I);
      break;

    case M_SB:
      pcrel_store (rs2, rs1, imm_expr, "sb",
		   BFD_RELOC_RISCV_PCREL_HI20, BFD_RELOC_RISCV_PCREL_LO12_S);
      break;

    case M_SH:
      pcrel_store (rs2, rs1, imm_expr, "sh",
		   BFD_RELOC_RISCV_PCREL_HI20, BFD_RELOC_RISCV_PCREL_LO12_S);
      break;

    case M_SW:
      pcrel_store (rs2, rs1, imm_expr, "sw",
		   BFD_RELOC_RISCV_PCREL_HI20, BFD_RELOC_RISCV_PCREL_LO12_S);
      break;

    case M_SD:
      pcrel_store (rs2, rs1, imm_expr, "sd",
		   BFD_RELOC_RISCV_PCREL_HI20, BFD_RELOC_RISCV_PCREL_LO12_S);
      break;

    case M_FSH:
      pcrel_store (rs2, rs1, imm_expr, "fsh",
		   BFD_RELOC_RISCV_PCREL_HI20, BFD_RELOC_RISCV_PCREL_LO12_S);
      break;

    case M_FSW:
      pcrel_store (rs2, rs1, imm_expr, "fsw",
		   BFD_RELOC_RISCV_PCREL_HI20, BFD_RELOC_RISCV_PCREL_LO12_S);
      break;

    case M_FSD:
      pcrel_store (rs2, rs1, imm_expr, "fsd",
		   BFD_RELOC_RISCV_PCREL_HI20, BFD_RELOC_RISCV_PCREL_LO12_S);
      break;

    case M_CALL:
      riscv_call (rd, rs1, imm_expr, *imm_reloc);
      break;

    case M_VMSGE:
    case M_VMSGEU:
      vector_macro (ip);
      break;

    case M_ZEXTH:
      riscv_ext (rd, rs1, xlen - 16, FALSE);
      break;

    default:
      as_bad (_("Macro %s not implemented"), ip->insn_mo->name);
      break;
    }
}

static const struct percent_op_match percent_op_utype[] =
{
  {"%tprel_hi", BFD_RELOC_RISCV_TPREL_HI20},
  {"%pcrel_hi", BFD_RELOC_RISCV_PCREL_HI20},
  {"%tls_ie_pcrel_hi", BFD_RELOC_RISCV_TLS_GOT_HI20},
  {"%tls_gd_pcrel_hi", BFD_RELOC_RISCV_TLS_GD_HI20},
  {"%hi", BFD_RELOC_RISCV_HI20},
  {"%got_hi", BFD_RELOC_RISCV_GOT_HI20},
  {"%got_pcrel_hi", BFD_RELOC_RISCV_GOT_HI20},
  {0, 0}
};

static const struct percent_op_match percent_op_itype[] =
{
  {"%lo", BFD_RELOC_RISCV_LO12_I},
  {"%tprel_lo", BFD_RELOC_RISCV_TPREL_LO12_I},
  {"%pcrel_lo", BFD_RELOC_RISCV_PCREL_LO12_I},
  {0, 0}
};

static const struct percent_op_match percent_op_stype[] =
{
  {"%lo", BFD_RELOC_RISCV_LO12_S},
  {"%tprel_lo", BFD_RELOC_RISCV_TPREL_LO12_S},
  {"%pcrel_lo", BFD_RELOC_RISCV_PCREL_LO12_S},
  {0, 0}
};

static const struct percent_op_match percent_op_rtype[] =
{
  {"%tprel_add", BFD_RELOC_RISCV_TPREL_ADD},
  {0, 0}
};

static const struct percent_op_match percent_op_null[] =
{
  {0, 0}
};

/* Return true if *STR points to a relocation operator.  When returning true,
   move *STR over the operator and store its relocation code in *RELOC.
   Leave both *STR and *RELOC alone when returning false.  */

static bfd_boolean
parse_relocation (char **str, bfd_reloc_code_real_type *reloc,
		  const struct percent_op_match *percent_op)
{
  for ( ; percent_op->str; percent_op++)
    if (strncasecmp (*str, percent_op->str, strlen (percent_op->str)) == 0)
      {
	int len = strlen (percent_op->str);

	if (!ISSPACE ((*str)[len]) && (*str)[len] != '(')
	  continue;

	*str += strlen (percent_op->str);
	*reloc = percent_op->reloc;

	/* Check whether the output BFD supports this relocation.
	   If not, issue an error and fall back on something safe.  */
	if (*reloc != BFD_RELOC_UNUSED
	    && !bfd_reloc_type_lookup (stdoutput, *reloc))
	  {
	    as_bad ("relocation %s isn't supported by the current ABI",
		    percent_op->str);
	    *reloc = BFD_RELOC_UNUSED;
	  }
	return TRUE;
      }
  return FALSE;
}

static void
my_getExpression (expressionS *ep, char *str)
{
  char *save_in;

  save_in = input_line_pointer;
  input_line_pointer = str;
  expression (ep);
  expr_end = input_line_pointer;
  input_line_pointer = save_in;
}

/* Parse string STR as a 16-bit relocatable operand.  Store the
   expression in *EP and the relocation, if any, in RELOC.
   Return the number of relocation operators used (0 or 1).

   On exit, EXPR_END points to the first character after the expression.  */

static size_t
my_getSmallExpression (expressionS *ep, bfd_reloc_code_real_type *reloc,
		       char *str, const struct percent_op_match *percent_op)
{
  size_t reloc_index;
  unsigned crux_depth, str_depth, regno;
  char *crux;

  /* First, check for integer registers.  No callers can accept a reg, but
     we need to avoid accidentally creating a useless undefined symbol below,
     if this is an instruction pattern that can't match.  A glibc build fails
     if this is removed.  */
  if (reg_lookup (&str, RCLASS_GPR, &regno))
    {
      ep->X_op = O_register;
      ep->X_add_number = regno;
      expr_end = str;
      return 0;
    }

  /* Search for the start of the main expression.
     End the loop with CRUX pointing to the start
     of the main expression and with CRUX_DEPTH containing the number
     of open brackets at that point.  */
  reloc_index = -1;
  str_depth = 0;
  do
    {
      reloc_index++;
      crux = str;
      crux_depth = str_depth;

      /* Skip over whitespace and brackets, keeping count of the number
	 of brackets.  */
      while (*str == ' ' || *str == '\t' || *str == '(')
	if (*str++ == '(')
	  str_depth++;
    }
  while (*str == '%'
	 && reloc_index < 1
	 && parse_relocation (&str, reloc, percent_op));

  my_getExpression (ep, crux);
  str = expr_end;

  /* Match every open bracket.  */
  while (crux_depth > 0 && (*str == ')' || *str == ' ' || *str == '\t'))
    if (*str++ == ')')
      crux_depth--;

  if (crux_depth > 0)
    as_bad ("unclosed '('");

  expr_end = str;

  return reloc_index;
}

/* Parse string STR as a vsetvli operand.  Store the expression in *EP.
   On exit, EXPR_END points to the first character after the expression.  */

static void
my_getVsetvliExpression (expressionS *ep, char *str)
{
  unsigned int vsew_value = 0, vlmul_value = 0, vediv_value = 0;
  unsigned int vta_value = 0, vma_value = 0;
  bfd_boolean vsew_found = FALSE, vlmul_found = FALSE, vediv_found = FALSE;
  bfd_boolean vta_found = FALSE, vma_found = FALSE;

  if (arg_lookup (&str, riscv_vsew, ARRAY_SIZE (riscv_vsew), &vsew_value))
    {
      if (*str == ',')
	++str;
      if (vsew_found)
	as_bad (_("multiple vsew constants"));
      vsew_found = TRUE;
    }
  if (arg_lookup (&str, riscv_vlmul, ARRAY_SIZE (riscv_vlmul), &vlmul_value))
    {
      if (*str == ',')
	++str;
      if (vlmul_found)
	as_bad (_("multiple vlmul constants"));
      vlmul_found = TRUE;
    }
  if (arg_lookup (&str, riscv_vta, ARRAY_SIZE (riscv_vta), &vta_value))
    {
      if (*str == ',')
	++str;
      if (vta_found)
	as_bad (_("multiple vta constants"));
      vta_found = TRUE;
    }
  if (arg_lookup (&str, riscv_vma, ARRAY_SIZE (riscv_vma), &vma_value))
    {
      if (*str == ',')
	++str;
      if (vma_found)
	as_bad (_("multiple vma constants"));
      vma_found = TRUE;
    }
  if (arg_lookup (&str, riscv_vediv, ARRAY_SIZE (riscv_vediv), &vediv_value))
    {
      if (*str == ',')
	++str;
      if (vediv_found)
	as_bad (_("multiple vediv constants"));
      vediv_found = TRUE;
    }

  if (vsew_found || vlmul_found || vediv_found || vta_found || vma_found)
    {
      ep->X_op = O_constant;
      ep->X_add_number = (vlmul_value << OP_SH_VLMUL)
			 | (vsew_value << OP_SH_VSEW)
			 | (vta_value << OP_SH_VTA)
			 | (vma_value << OP_SH_VMA)
			 | (vediv_value << OP_SH_VEDIV);
      expr_end = str;
    }
  else
    {
      my_getExpression (ep, str);
      str = expr_end;
    }
}

/* Parse opcode name, could be an mnemonics or number.  */
static size_t
my_getOpcodeExpression (expressionS *ep, bfd_reloc_code_real_type *reloc,
			char *str, const struct percent_op_match *percent_op)
{
  const struct opcode_name_t *o = opcode_name_lookup (&str);

  if (o != NULL)
    {
      ep->X_op = O_constant;
      ep->X_add_number = o->val;
      return 0;
    }

  return my_getSmallExpression (ep, reloc, str, percent_op);
}

/* Detect and handle implicitly zero load-store offsets.  For example,
   "lw t0, (t1)" is shorthand for "lw t0, 0(t1)".  Return TRUE iff such
   an implicit offset was detected.  */

static bfd_boolean
riscv_handle_implicit_zero_offset (expressionS *ep, const char *s)
{
  /* Check whether there is only a single bracketed expression left.
     If so, it must be the base register and the constant must be zero.  */
  if (*s == '(' && strchr (s + 1, '(') == 0)
    {
      ep->X_op = O_constant;
      ep->X_add_number = 0;
      return TRUE;
    }

  return FALSE;
}

/* All RISC-V CSR instructions belong to one of these classes.  */

enum csr_insn_type
{
  INSN_NOT_CSR,
  INSN_CSRRW,
  INSN_CSRRS,
  INSN_CSRRC
};

/* Return which CSR instruction is checking.  */

static enum csr_insn_type
riscv_csr_insn_type (insn_t insn)
{
  if (((insn ^ MATCH_CSRRW) & MASK_CSRRW) == 0
      || ((insn ^ MATCH_CSRRWI) & MASK_CSRRWI) == 0)
    return INSN_CSRRW;
  else if (((insn ^ MATCH_CSRRS) & MASK_CSRRS) == 0
	   || ((insn ^ MATCH_CSRRSI) & MASK_CSRRSI) == 0)
    return INSN_CSRRS;
  else if (((insn ^ MATCH_CSRRC) & MASK_CSRRC) == 0
	   || ((insn ^ MATCH_CSRRCI) & MASK_CSRRCI) == 0)
    return INSN_CSRRC;
  else
    return INSN_NOT_CSR;
}

/* CSRRW and CSRRWI always write CSR.  CSRRS, CSRRC, CSRRSI and CSRRCI write
   CSR when RS1 isn't zero.  The CSR is read only if the [11:10] bits of
   CSR address is 0x3.  */

static bfd_boolean
riscv_csr_read_only_check (insn_t insn)
{
  int csr = (insn & (OP_MASK_CSR << OP_SH_CSR)) >> OP_SH_CSR;
  int rs1 = (insn & (OP_MASK_RS1 << OP_SH_RS1)) >> OP_SH_RS1;
  int readonly = (((csr & (0x3 << 10)) >> 10) == 0x3);
  enum csr_insn_type csr_insn = riscv_csr_insn_type (insn);

  if (readonly
      && (((csr_insn == INSN_CSRRS
	    || csr_insn == INSN_CSRRC)
	   && rs1 != 0)
	  || csr_insn == INSN_CSRRW))
    return FALSE;

  return TRUE;
}

#ifndef MASK_AQRL
  #define MASK_AQRL (0x3u << 25)
#endif

static bfd_boolean
is_b19758_associated_insn (struct riscv_opcode *insn)
{
  static insn_t lr_insns[] = {
    MATCH_LR_W,
    #if 0
    MATCH_LR_D,
    #endif
    0
  };
  static insn_t amo_insns[] = {
    MATCH_AMOSWAP_W,
    MATCH_AMOADD_W,
    MATCH_AMOAND_W,
    MATCH_AMOOR_W,
    MATCH_AMOXOR_W,
    MATCH_AMOMAX_W,
    MATCH_AMOMAXU_W,
    MATCH_AMOMIN_W,
    MATCH_AMOMINU_W,
    #if 0
    MATCH_AMOSWAP_D,
    MATCH_AMOADD_D,
    MATCH_AMOAND_D,
    MATCH_AMOOR_D,
    MATCH_AMOXOR_D,
    MATCH_AMOMAX_D,
    MATCH_AMOMAXU_D,
    MATCH_AMOMIN_D,
    MATCH_AMOMINU_D,
    #endif
    0
  };
  const insn_t lr_mask = MASK_LR_W | MASK_AQRL;
  const insn_t amo_mask = MASK_AMOSWAP_W | MASK_AQRL;
  bfd_boolean rz = FALSE;
  int i = 0;

  if (insn->mask == lr_mask) {
    insn_t match = insn->match & ~(MASK_AQRL|0x1000);
    while (lr_insns[i]) {
      if (match == lr_insns[i++]) {
        rz = TRUE;
        break;
      }
    }
  } else if (insn->mask == amo_mask) {
    insn_t match = insn->match & ~(MASK_AQRL|0x1000);
    while (amo_insns[i]) {
      if (match == amo_insns[i++]) {
        rz = TRUE;
        break;
      }
    }
  }
  return rz;
}

static bfd_boolean
is_indirect_jump (struct riscv_opcode *insn)
{
  static insn_t insns[] = {
    MATCH_JALR, MASK_JALR,
    MATCH_C_JALR, MASK_C_JALR,
    MATCH_C_JR, MASK_C_JR,
    0
  };
  bfd_boolean rz = FALSE;
  int i = 0;
  while (insns[i]) {
    insn_t match = insns[i++];
    insn_t mask = insns[i++];
    if ((insn->match == match) && (insn->mask == mask)) {
      rz = TRUE;
      break;
    }
  }
  return rz;
}

static bfd_boolean
is_conditional_branch (struct riscv_opcode *insn)
{
  static insn_t insns[] = {
    MATCH_BEQ, MATCH_BEQC, MATCH_C_BEQZ,
    MATCH_BNE, MATCH_BNEC, MATCH_C_BNEZ,
    MATCH_BLT, MATCH_BLTU,
    MATCH_BGE, MATCH_BGEU,
    MATCH_BBC, MATCH_BBS,
    0
  };
  bfd_boolean rz = FALSE;
  int i = 0;
  insn_t x;
  while ((x = insns[i++])) {
    if (x == insn->match) {
      rz = TRUE;
      break;
    }
  }
  return rz;
}

/* This routine assembles an instruction into its binary format.  As a
   side effect, it sets the global variable imm_reloc to the type of
   relocation to do if one of the operands is an address expression.  */

static const char *
riscv_ip (char *str, struct riscv_cl_insn *ip, expressionS *imm_expr,
	  bfd_reloc_code_real_type *imm_reloc, struct hash_control *hash)
{
  char *s;
  const char *args;
  char c = 0;
  struct riscv_opcode *insn;
  char *argsStart;
  unsigned int regno, save_regno = 0;
  char save_c = 0;
  int argnum;
  const struct percent_op_match *p;
  const char *error = "unrecognized opcode";
  const char *default_error = error;
  /* Indicate we are assembling instruction with CSR.  */
  bfd_boolean insn_with_csr = FALSE;

  /* Parse the name of the instruction.  Terminate the string if whitespace
     is found so that hash_find only sees the name part of the string.  */
  for (s = str; *s != '\0'; ++s)
    if (ISSPACE (*s))
      {
	save_c = *s;
	*s++ = '\0';
	break;
      }

  insn = (struct riscv_opcode *) hash_find (hash, str);

  argsStart = s;
  for ( ; insn && insn->name && strcmp (insn->name, str) == 0; insn++)
    {
      if ((insn->xlen_requirement != 0) && (xlen != insn->xlen_requirement))
	continue;

      if (ace_lib_load_success && !strcasecmp(insn->subset[0], "X"))
	;
      else if (!riscv_multi_subset_supports (insn->subset))
	continue;

      create_insn (ip, insn);
      argnum = 1;

      imm_expr->X_op = O_absent;
      *imm_reloc = BFD_RELOC_UNUSED;
      p = percent_op_itype;

      for (args = insn->args;; ++args)
	{
	  s += strspn (s, " \t");
	  switch (*args)
	    {
	    case '\0': 	/* End of args.  */
	      if (insn->pinfo != INSN_MACRO)
		{
		  if (!insn->match_func (insn, ip->insn_opcode,
					 riscv_opts.check_constraints,
					 &error))
		    break;

		  /* For .insn, insn->match and insn->mask are 0.  */
		  if (riscv_insn_length ((insn->match == 0 && insn->mask == 0)
					 ? ip->insn_opcode
					 : insn->match) == 2
		      && !riscv_opts.rvc)
		    break;

		  /* Check if we write a read-only CSR by the CSR
		     instruction.  */
		  if (insn_with_csr
		      && riscv_opts.csr_check
		      && !riscv_csr_read_only_check (ip->insn_opcode))
		    {
		      /* Restore the character in advance, since we want to
			 report the detailed warning message here.  */
		      if (save_c)
			*(argsStart - 1) = save_c;
		      as_warn (_("Read-only CSR is written `%s'"), str);
		      insn_with_csr = FALSE;
		    }
		}
	      if (*s != '\0')
		break;
	      /* Successful assembly.  */
	      error = NULL;
	      insn_with_csr = FALSE;
	      goto out;

	    case 'C': /* RVC */
	      switch (*++args)
		{
		case 's': /* RS1 x8-x15 */
		  if (!reg_lookup (&s, RCLASS_GPR, &regno)
		      || !(regno >= 8 && regno <= 15))
		    break;
		  INSERT_OPERAND (CRS1S, *ip, regno % 8);
		  continue;
		case 'w': /* RS1 x8-x15, constrained to equal RD x8-x15.  */
		  if (!reg_lookup (&s, RCLASS_GPR, &regno)
		      || EXTRACT_OPERAND (CRS1S, ip->insn_opcode) + 8 != regno)
		    break;
		  continue;
		case 't': /* RS2 x8-x15 */
		  if (!reg_lookup (&s, RCLASS_GPR, &regno)
		      || !(regno >= 8 && regno <= 15))
		    break;
		  INSERT_OPERAND (CRS2S, *ip, regno % 8);
		  continue;
		case 'x': /* RS2 x8-x15, constrained to equal RD x8-x15.  */
		  if (!reg_lookup (&s, RCLASS_GPR, &regno)
		      || EXTRACT_OPERAND (CRS2S, ip->insn_opcode) + 8 != regno)
		    break;
		  continue;
		case 'U': /* RS1, constrained to equal RD.  */
		  if (!reg_lookup (&s, RCLASS_GPR, &regno)
		      || EXTRACT_OPERAND (RD, ip->insn_opcode) != regno)
		    break;
		  continue;
		case 'V': /* RS2 */
		  if (!reg_lookup (&s, RCLASS_GPR, &regno))
		    break;
		  INSERT_OPERAND (CRS2, *ip, regno);
		  continue;
		case 'c': /* RS1, constrained to equal sp.  */
		  if (!reg_lookup (&s, RCLASS_GPR, &regno)
		      || regno != X_SP)
		    break;
		  continue;
		case 'z': /* RS2, contrained to equal x0.  */
		  if (!reg_lookup (&s, RCLASS_GPR, &regno)
		      || regno != 0)
		    break;
		  continue;
		case '>':
		  if (my_getSmallExpression (imm_expr, imm_reloc, s, p)
		      || imm_expr->X_op != O_constant
		      || imm_expr->X_add_number <= 0
		      || imm_expr->X_add_number >= 64)
		    break;
		  ip->insn_opcode |= ENCODE_RVC_IMM (imm_expr->X_add_number);
rvc_imm_done:
		  s = expr_end;
		  imm_expr->X_op = O_absent;
		  continue;
		case '<':
		  if (my_getSmallExpression (imm_expr, imm_reloc, s, p)
		      || imm_expr->X_op != O_constant
		      || !VALID_RVC_IMM (imm_expr->X_add_number)
		      || imm_expr->X_add_number <= 0
		      || imm_expr->X_add_number >= 32)
		    break;
		  ip->insn_opcode |= ENCODE_RVC_IMM (imm_expr->X_add_number);
		  goto rvc_imm_done;
		case '8':
		  if (my_getSmallExpression (imm_expr, imm_reloc, s, p)
		      || imm_expr->X_op != O_constant
		      || !VALID_RVC_UIMM8 (imm_expr->X_add_number)
		      || imm_expr->X_add_number < 0
		      || imm_expr->X_add_number >= 256)
		    break;
		  ip->insn_opcode |= ENCODE_RVC_UIMM8 (imm_expr->X_add_number);
		  goto rvc_imm_done;
		case 'i':
		  if (my_getSmallExpression (imm_expr, imm_reloc, s, p)
		      || imm_expr->X_op != O_constant
		      || imm_expr->X_add_number == 0
		      || !VALID_RVC_SIMM3 (imm_expr->X_add_number))
		    break;
		  ip->insn_opcode |= ENCODE_RVC_SIMM3 (imm_expr->X_add_number);
		  goto rvc_imm_done;
		case 'j':
		  if (my_getSmallExpression (imm_expr, imm_reloc, s, p)
		      || imm_expr->X_op != O_constant
		      || imm_expr->X_add_number == 0
		      || !VALID_RVC_IMM (imm_expr->X_add_number))
		    break;
		  ip->insn_opcode |= ENCODE_RVC_IMM (imm_expr->X_add_number);
		  goto rvc_imm_done;
		case 'e': /* exec.it imm  */
		  switch (*++args)
		    {
		    case 'i':
		      if (my_getSmallExpression (imm_expr, imm_reloc, s, p)
			  || imm_expr->X_op != O_constant
			  || !VALID_RVC_EX9IT_IMM (imm_expr->X_add_number << 2))
			break;
		      ip->insn_opcode |= ENCODE_RVC_EX9IT_IMM (imm_expr->X_add_number << 2);
		      goto rvc_imm_done;
		    case 't':
		      if (my_getSmallExpression (imm_expr, imm_reloc, s, p)
			  || imm_expr->X_op != O_constant
			  || !VALID_RVC_EXECIT_IMM (imm_expr->X_add_number << 2))
			break;
		      ip->insn_opcode |= ENCODE_RVC_EXECIT_IMM (imm_expr->X_add_number << 2);
		      goto rvc_imm_done;
		    default:
		      break;
		    }
		  break;
		case 'k':
		  if (riscv_handle_implicit_zero_offset (imm_expr, s))
		    continue;
		  if (my_getSmallExpression (imm_expr, imm_reloc, s, p)
		      || imm_expr->X_op != O_constant
		      || !VALID_RVC_LW_IMM (imm_expr->X_add_number))
		    break;
		  ip->insn_opcode |= ENCODE_RVC_LW_IMM (imm_expr->X_add_number);
		  goto rvc_imm_done;
		case 'l':
		  if (riscv_handle_implicit_zero_offset (imm_expr, s))
		    continue;
		  if (my_getSmallExpression (imm_expr, imm_reloc, s, p)
		      || imm_expr->X_op != O_constant
		      || !VALID_RVC_LD_IMM (imm_expr->X_add_number))
		    break;
		  ip->insn_opcode |= ENCODE_RVC_LD_IMM (imm_expr->X_add_number);
		  goto rvc_imm_done;
		case 'm':
		  if (riscv_handle_implicit_zero_offset (imm_expr, s))
		    continue;
		  if (my_getSmallExpression (imm_expr, imm_reloc, s, p)
		      || imm_expr->X_op != O_constant
		      || !VALID_RVC_LWSP_IMM (imm_expr->X_add_number))
		    break;
		  ip->insn_opcode |=
		    ENCODE_RVC_LWSP_IMM (imm_expr->X_add_number);
		  goto rvc_imm_done;
		case 'n':
		  if (riscv_handle_implicit_zero_offset (imm_expr, s))
		    continue;
		  if (my_getSmallExpression (imm_expr, imm_reloc, s, p)
		      || imm_expr->X_op != O_constant
		      || !VALID_RVC_LDSP_IMM (imm_expr->X_add_number))
		    break;
		  ip->insn_opcode |=
		    ENCODE_RVC_LDSP_IMM (imm_expr->X_add_number);
		  goto rvc_imm_done;
		case 'o':
		  if (my_getSmallExpression (imm_expr, imm_reloc, s, p)
		      || imm_expr->X_op != O_constant
		      /* C.addiw, c.li, and c.andi allow zero immediate.
			 C.addi allows zero immediate as hint.  Otherwise this
			 is same as 'j'.  */
		      || !VALID_RVC_IMM (imm_expr->X_add_number))
		    break;
		  ip->insn_opcode |= ENCODE_RVC_IMM (imm_expr->X_add_number);
		  goto rvc_imm_done;
		case 'K':
		  if (my_getSmallExpression (imm_expr, imm_reloc, s, p)
		      || imm_expr->X_op != O_constant
		      || !VALID_RVC_ADDI4SPN_IMM (imm_expr->X_add_number)
		      || imm_expr->X_add_number == 0)
		    break;
		  ip->insn_opcode |=
		    ENCODE_RVC_ADDI4SPN_IMM (imm_expr->X_add_number);
		  goto rvc_imm_done;
		case 'L':
		  if (my_getSmallExpression (imm_expr, imm_reloc, s, p)
		      || imm_expr->X_op != O_constant
		      || !VALID_RVC_ADDI16SP_IMM (imm_expr->X_add_number)
		      || imm_expr->X_add_number == 0)
		    break;
		  ip->insn_opcode |=
		    ENCODE_RVC_ADDI16SP_IMM (imm_expr->X_add_number);
		  goto rvc_imm_done;
		case 'M':
		  if (riscv_handle_implicit_zero_offset (imm_expr, s))
		    continue;
		  if (my_getSmallExpression (imm_expr, imm_reloc, s, p)
		      || imm_expr->X_op != O_constant
		      || !VALID_RVC_SWSP_IMM (imm_expr->X_add_number))
		    break;
		  ip->insn_opcode |=
		    ENCODE_RVC_SWSP_IMM (imm_expr->X_add_number);
		  goto rvc_imm_done;
		case 'N':
		  if (riscv_handle_implicit_zero_offset (imm_expr, s))
		    continue;
		  if (my_getSmallExpression (imm_expr, imm_reloc, s, p)
		      || imm_expr->X_op != O_constant
		      || !VALID_RVC_SDSP_IMM (imm_expr->X_add_number))
		    break;
		  ip->insn_opcode |=
		    ENCODE_RVC_SDSP_IMM (imm_expr->X_add_number);
		  goto rvc_imm_done;
		case 'u':
		  p = percent_op_utype;
		  if (my_getSmallExpression (imm_expr, imm_reloc, s, p))
		    break;
rvc_lui:
		  if (imm_expr->X_op != O_constant
		      || imm_expr->X_add_number <= 0
		      || imm_expr->X_add_number >= RISCV_BIGIMM_REACH
		      || (imm_expr->X_add_number >= RISCV_RVC_IMM_REACH / 2
			  && (imm_expr->X_add_number <
			      RISCV_BIGIMM_REACH - RISCV_RVC_IMM_REACH / 2)))
		    break;
		  ip->insn_opcode |= ENCODE_RVC_IMM (imm_expr->X_add_number);
		  goto rvc_imm_done;
		case 'v':
		  if (my_getSmallExpression (imm_expr, imm_reloc, s, p)
		      || (imm_expr->X_add_number & (RISCV_IMM_REACH - 1))
		      || ((int32_t)imm_expr->X_add_number
			  != imm_expr->X_add_number))
		    break;
		  imm_expr->X_add_number =
		    ((uint32_t) imm_expr->X_add_number) >> RISCV_IMM_BITS;
		  goto rvc_lui;
		case 'p':
		  goto branch;
		case 'a':
		  goto jump;
		case 'S': /* Floating-point RS1 x8-x15.  */
		  if (!reg_lookup (&s, RCLASS_FPR, &regno)
		      || !(regno >= 8 && regno <= 15))
		    break;
		  INSERT_OPERAND (CRS1S, *ip, regno % 8);
		  continue;
		case 'D': /* Floating-point RS2 x8-x15.  */
		  if (!reg_lookup (&s, RCLASS_FPR, &regno)
		      || !(regno >= 8 && regno <= 15))
		    break;
		  INSERT_OPERAND (CRS2S, *ip, regno % 8);
		  continue;
		case 'T': /* Floating-point RS2.  */
		  if (!reg_lookup (&s, RCLASS_FPR, &regno))
		    break;
		  INSERT_OPERAND (CRS2, *ip, regno);
		  continue;
		case 'F':
		  switch (*++args)
		    {
		      case '6':
		        if (my_getSmallExpression (imm_expr, imm_reloc, s, p)
			    || imm_expr->X_op != O_constant
			    || imm_expr->X_add_number < 0
			    || imm_expr->X_add_number >= 64)
			  {
			    as_bad (_("bad value for funct6 field, "
				      "value must be 0...64"));
			    break;
			  }

			INSERT_OPERAND (CFUNCT6, *ip, imm_expr->X_add_number);
			imm_expr->X_op = O_absent;
			s = expr_end;
			continue;
		      case '4':
		        if (my_getSmallExpression (imm_expr, imm_reloc, s, p)
			    || imm_expr->X_op != O_constant
			    || imm_expr->X_add_number < 0
			    || imm_expr->X_add_number >= 16)
			  {
			    as_bad (_("bad value for funct4 field, "
				      "value must be 0...15"));
			    break;
			  }

			INSERT_OPERAND (CFUNCT4, *ip, imm_expr->X_add_number);
			imm_expr->X_op = O_absent;
			s = expr_end;
			continue;
		      case '3':
			if (my_getSmallExpression (imm_expr, imm_reloc, s, p)
			    || imm_expr->X_op != O_constant
			    || imm_expr->X_add_number < 0
			    || imm_expr->X_add_number >= 8)
			  {
			    as_bad (_("bad value for funct3 field, "
				      "value must be 0...7"));
			    break;
			  }
			INSERT_OPERAND (CFUNCT3, *ip, imm_expr->X_add_number);
			imm_expr->X_op = O_absent;
			s = expr_end;
			continue;
		      case '2':
			if (my_getSmallExpression (imm_expr, imm_reloc, s, p)
			    || imm_expr->X_op != O_constant
			    || imm_expr->X_add_number < 0
			    || imm_expr->X_add_number >= 4)
			  {
			    as_bad (_("bad value for funct2 field, "
				      "value must be 0...3"));
			    break;
			  }
			INSERT_OPERAND (CFUNCT2, *ip, imm_expr->X_add_number);
			imm_expr->X_op = O_absent;
			s = expr_end;
			continue;
		      default:
			as_bad (_("bad compressed FUNCT field"
				  " specifier 'CF%c'\n"),
				*args);
		    }
		  break;

		default:
		  as_bad (_("bad RVC field specifier 'C%c'\n"), *args);
		}
	      break;

	    case ',':
	    case '+':
	      ++argnum;
	      if (*s++ == *args)
		continue;
	      s--;
	      break;

	    case '(':
	    case ')':
	    case '[':
	    case ']':
	      if (*s++ == *args)
		continue;
	      break;

	    case '<':		/* Shift amount, 0 - 31.  */
	      my_getExpression (imm_expr, s);
	      check_absolute_expr (ip, imm_expr, FALSE);
	      if ((unsigned long) imm_expr->X_add_number > 31)
		as_bad (_("Improper shift amount (%lu)"),
			(unsigned long) imm_expr->X_add_number);
	      INSERT_OPERAND (SHAMTW, *ip, imm_expr->X_add_number);
	      imm_expr->X_op = O_absent;
	      s = expr_end;
	      continue;

	    case '>':		/* Shift amount, 0 - (XLEN-1).  */
	      my_getExpression (imm_expr, s);
	      check_absolute_expr (ip, imm_expr, FALSE);
	      if ((unsigned long) imm_expr->X_add_number >= xlen)
		as_bad (_("Improper shift amount (%lu)"),
			(unsigned long) imm_expr->X_add_number);
	      INSERT_OPERAND (SHAMT, *ip, imm_expr->X_add_number);
	      imm_expr->X_op = O_absent;
	      s = expr_end;
	      continue;

	    case 'Z':		/* CSRRxI immediate.  */
	      my_getExpression (imm_expr, s);
	      check_absolute_expr (ip, imm_expr, FALSE);
	      if ((unsigned long) imm_expr->X_add_number > 31)
		as_bad (_("Improper CSRxI immediate (%lu)"),
			(unsigned long) imm_expr->X_add_number);
	      INSERT_OPERAND (RS1, *ip, imm_expr->X_add_number);
	      imm_expr->X_op = O_absent;
	      s = expr_end;
	      continue;

	    case 'E':		/* Control register.  */
	      insn_with_csr = TRUE;
	      if (reg_lookup (&s, RCLASS_CSR, &regno))
		INSERT_OPERAND (CSR, *ip, regno);
	      else
		{
		  my_getExpression (imm_expr, s);
		  check_absolute_expr (ip, imm_expr, TRUE);
		  if ((unsigned long) imm_expr->X_add_number > 0xfff)
		    as_bad (_("Improper CSR address (%lu)"),
			    (unsigned long) imm_expr->X_add_number);
		  INSERT_OPERAND (CSR, *ip, imm_expr->X_add_number);
		  imm_expr->X_op = O_absent;
		  s = expr_end;
		}
	      continue;

	    case 'm':		/* Rounding mode.  */
	      if (arg_lookup (&s, riscv_rm, ARRAY_SIZE (riscv_rm), &regno))
		{
		  INSERT_OPERAND (RM, *ip, regno);
		  continue;
		}
	      break;

	    case 'P':
	    case 'Q':		/* Fence predecessor/successor.  */
	      if (arg_lookup (&s, riscv_pred_succ, ARRAY_SIZE (riscv_pred_succ),
			      &regno))
		{
		  if (*args == 'P')
		    INSERT_OPERAND (PRED, *ip, regno);
		  else
		    INSERT_OPERAND (SUCC, *ip, regno);
		  continue;
		}
	      break;

	    case 'd':		/* Destination register.  */
	    case 's':		/* Source register.  */
	    case 't':		/* Target register.  */
	    case 'r':		/* rs3.  */
	    case 'e':		/* Target register with check.  */
	      if (reg_lookup (&s, RCLASS_GPR, &regno))
		{
		  c = *args;
		  if (*s == ' ')
		    ++s;

		  /* Now that we have assembled one operand, we use the args
		     string to figure out where it goes in the instruction.  */
		  switch (c)
		    {
		    case 's':
		      save_regno = regno;
		      INSERT_OPERAND (RS1, *ip, regno);
		      break;
		    case 'd':
		      INSERT_OPERAND (RD, *ip, regno);
		      break;
		    case 'e':
		    case 't':
		      INSERT_OPERAND (RS2, *ip, regno);
		      break;
		    case 'r':
		      INSERT_OPERAND (RS3, *ip, regno);
		      break;
		    }
		  if (c == 'e' && regno < save_regno)
		    break;
		  continue;
		}
	      break;

	    case 'D':		/* Floating point rd.  */
	    case 'S':		/* Floating point rs1.  */
	    case 'T':		/* Floating point rs2.  */
	    case 'U':		/* Floating point rs1 and rs2.  */
	    case 'R':		/* Floating point rs3.  */
	      if (reg_lookup (&s, RCLASS_FPR, &regno))
		{
		  c = *args;
		  if (*s == ' ')
		    ++s;
		  switch (c)
		    {
		    case 'D':
		      INSERT_OPERAND (RD, *ip, regno);
		      break;
		    case 'S':
		      INSERT_OPERAND (RS1, *ip, regno);
		      break;
		    case 'U':
		      INSERT_OPERAND (RS1, *ip, regno);
		      /* fallthru */
		    case 'T':
		      INSERT_OPERAND (RS2, *ip, regno);
		      break;
		    case 'R':
		      INSERT_OPERAND (RS3, *ip, regno);
		      break;
		    }
		  continue;
		}

	      break;

	    case 'I':
	      my_getExpression (imm_expr, s);
	      if (imm_expr->X_op != O_big
		  && imm_expr->X_op != O_constant)
		break;
	      normalize_constant_expr (imm_expr);
	      s = expr_end;
	      continue;

	    case 'A':
	      my_getExpression (imm_expr, s);
	      normalize_constant_expr (imm_expr);
	      /* The 'A' format specifier must be a symbol.  */
	      if (imm_expr->X_op != O_symbol)
	        break;
	      *imm_reloc = BFD_RELOC_32;
	      s = expr_end;
	      continue;

	    case 'B':
	      my_getExpression (imm_expr, s);
	      normalize_constant_expr (imm_expr);
	      /* The 'B' format specifier must be a symbol or a constant.  */
	      if (imm_expr->X_op != O_symbol && imm_expr->X_op != O_constant)
	        break;
	      if (imm_expr->X_op == O_symbol)
	        *imm_reloc = BFD_RELOC_32;
	      s = expr_end;
	      continue;

	    case 'j': /* Sign-extended immediate.  */
	      p = percent_op_itype;
	      *imm_reloc = BFD_RELOC_RISCV_LO12_I;
	      goto alu_op;
	    case 'q': /* Store displacement.  */
	      p = percent_op_stype;
	      *imm_reloc = BFD_RELOC_RISCV_LO12_S;
	      goto load_store;
	    case 'o': /* Load displacement.  */
	      p = percent_op_itype;
	      *imm_reloc = BFD_RELOC_RISCV_LO12_I;
	      goto load_store;
	    case '1': /* 4-operand add, must be %tprel_add.  */
	      p = percent_op_rtype;
	      goto alu_op;
	    case '0': /* AMO "displacement," which must be zero.  */
	      p = percent_op_null;
load_store:
	      if (riscv_handle_implicit_zero_offset (imm_expr, s))
		continue;
alu_op:
	      /* If this value won't fit into a 16 bit offset, then go
		 find a macro that will generate the 32 bit offset
		 code pattern.  */
	      if (!my_getSmallExpression (imm_expr, imm_reloc, s, p))
		{
		  normalize_constant_expr (imm_expr);
		  if (imm_expr->X_op != O_constant
		      || (*args == '0' && imm_expr->X_add_number != 0)
		      || (*args == '1')
		      || imm_expr->X_add_number >= (signed)RISCV_IMM_REACH/2
		      || imm_expr->X_add_number < -(signed)RISCV_IMM_REACH/2)
		    break;
		}

	      s = expr_end;
	      continue;

	    case 'p':		/* PC-relative offset.  */
branch:
	      *imm_reloc = BFD_RELOC_12_PCREL;
	      my_getExpression (imm_expr, s);
	      s = expr_end;
	      continue;

	    case 'u':		/* Upper 20 bits.  */
	      p = percent_op_utype;
	      if (!my_getSmallExpression (imm_expr, imm_reloc, s, p))
		{
		  if (imm_expr->X_op != O_constant)
		    break;

		  if (imm_expr->X_add_number < 0
		      || imm_expr->X_add_number >= (signed)RISCV_BIGIMM_REACH)
		    as_bad (_("lui expression not in range 0..1048575"));

		  *imm_reloc = BFD_RELOC_RISCV_HI20;
		  imm_expr->X_add_number <<= RISCV_IMM_BITS;
		}
	      s = expr_end;
	      continue;

	    case 'a':		/* 20-bit PC-relative offset.  */
jump:
	      my_getExpression (imm_expr, s);
	      s = expr_end;
	      *imm_reloc = BFD_RELOC_RISCV_JMP;
	      continue;

	    case 'c':
	      my_getExpression (imm_expr, s);
	      s = expr_end;
	      if (strcmp (s, "@plt") == 0 || strcmp(s, "@PLT") == 0)
		{
		  *imm_reloc = BFD_RELOC_RISCV_CALL_PLT;
		  s += 4;
		}
	      else
		*imm_reloc = BFD_RELOC_RISCV_CALL;
	      continue;
	    case 'O':
	      switch (*++args)
		{
		case '4':
		  if (my_getOpcodeExpression (imm_expr, imm_reloc, s, p)
		      || imm_expr->X_op != O_constant
		      || imm_expr->X_add_number < 0
		      || imm_expr->X_add_number >= 128
		      || (imm_expr->X_add_number & 0x3) != 3)
		    {
		      as_bad (_("bad value for opcode field, "
				"value must be 0...127 and "
				"lower 2 bits must be 0x3"));
		      break;
		    }

		  INSERT_OPERAND (OP, *ip, imm_expr->X_add_number);
		  imm_expr->X_op = O_absent;
		  s = expr_end;
		  continue;
		case '2':
		  if (my_getOpcodeExpression (imm_expr, imm_reloc, s, p)
		      || imm_expr->X_op != O_constant
		      || imm_expr->X_add_number < 0
		      || imm_expr->X_add_number >= 3)
		    {
		      as_bad (_("bad value for opcode field, "
				"value must be 0...2"));
		      break;
		    }

		  INSERT_OPERAND (OP2, *ip, imm_expr->X_add_number);
		  imm_expr->X_op = O_absent;
		  s = expr_end;
		  continue;
		default:
		  as_bad (_("bad Opcode field specifier 'O%c'\n"), *args);
		}
	      break;

	    case 'F':
	      switch (*++args)
		{
		case '7':
		  if (my_getSmallExpression (imm_expr, imm_reloc, s, p)
		      || imm_expr->X_op != O_constant
		      || imm_expr->X_add_number < 0
		      || imm_expr->X_add_number >= 128)
		    {
		      as_bad (_("bad value for funct7 field, "
				"value must be 0...127"));
		      break;
		    }

		  INSERT_OPERAND (FUNCT7, *ip, imm_expr->X_add_number);
		  imm_expr->X_op = O_absent;
		  s = expr_end;
		  continue;
		case '3':
		  if (my_getSmallExpression (imm_expr, imm_reloc, s, p)
		      || imm_expr->X_op != O_constant
		      || imm_expr->X_add_number < 0
		      || imm_expr->X_add_number >= 8)
		    {
		      as_bad (_("bad value for funct3 field, "
			        "value must be 0...7"));
		      break;
		    }

		  INSERT_OPERAND (FUNCT3, *ip, imm_expr->X_add_number);
		  imm_expr->X_op = O_absent;
		  s = expr_end;
		  continue;
		case '2':
		  if (my_getSmallExpression (imm_expr, imm_reloc, s, p)
		      || imm_expr->X_op != O_constant
		      || imm_expr->X_add_number < 0
		      || imm_expr->X_add_number >= 4)
		    {
		      as_bad (_("bad value for funct2 field, "
			        "value must be 0...3"));
		      break;
		    }

		  INSERT_OPERAND (FUNCT2, *ip, imm_expr->X_add_number);
		  imm_expr->X_op = O_absent;
		  s = expr_end;
		  continue;

		default:
		  as_bad (_("bad FUNCT field specifier 'F%c'\n"), *args);
		}
	      break;

	    case 'y':        /* bs immediate */
	      my_getExpression (imm_expr, s);
	      check_absolute_expr (ip, imm_expr, FALSE);
	      if ((unsigned long)imm_expr->X_add_number > 3)
		      as_bad(_("Improper bs immediate (%lu)"),
		        (unsigned long)imm_expr->X_add_number);
	      INSERT_OPERAND(BS, *ip, imm_expr->X_add_number);
	      imm_expr->X_op = O_absent;
	      s = expr_end;
	      continue;
            
	    case 'Y':        /* rnum immediate */
	      my_getExpression (imm_expr, s);
	      check_absolute_expr (ip, imm_expr, FALSE);
	      if ((unsigned long)imm_expr->X_add_number > 10)
		      as_bad(_("Improper rnum immediate (%lu)"),
		        (unsigned long)imm_expr->X_add_number);
	      INSERT_OPERAND(RNUM, *ip, imm_expr->X_add_number);
	      imm_expr->X_op = O_absent;
	      s = expr_end;
	      continue;

	    case 'z':
	      if (my_getSmallExpression (imm_expr, imm_reloc, s, p)
		  || imm_expr->X_op != O_constant
		  || imm_expr->X_add_number != 0)
		break;
	      s = expr_end;
	      imm_expr->X_op = O_absent;
	      continue;

	    case 'V': /* RVV */
	      switch (*++args)
		{
		case 'd': /* VD */
		  if (!reg_lookup (&s, RCLASS_VECR, &regno))
		    break;
		  INSERT_OPERAND (VD, *ip, regno);
		  continue;

		case 'e': /* AMO VD */
		  if (reg_lookup (&s, RCLASS_GPR, &regno) && regno == 0)
		    INSERT_OPERAND (VWD, *ip, 0);
		  else if (reg_lookup (&s, RCLASS_VECR, &regno))
		    {
		      INSERT_OPERAND (VWD, *ip, 1);
		      INSERT_OPERAND (VD, *ip, regno);
		    }
		  else
		    break;
		  continue;

		case 'f': /* AMO VS3 */
		  if (!reg_lookup (&s, RCLASS_VECR, &regno))
		    break;
		  if (!EXTRACT_OPERAND (VWD, ip->insn_opcode))
		    INSERT_OPERAND (VD, *ip, regno);
		  else
		    {
		      /* VS3 must match VD.  */
		      if (EXTRACT_OPERAND (VD, ip->insn_opcode) != regno)
			break;
		    }
		  continue;

		case 's': /* VS1 */
		  if (!reg_lookup (&s, RCLASS_VECR, &regno))
		    break;
		  INSERT_OPERAND (VS1, *ip, regno);
		  continue;

		case 't': /* VS2 */
		  if (!reg_lookup (&s, RCLASS_VECR, &regno))
		    break;
		  INSERT_OPERAND (VS2, *ip, regno);
		  continue;

		case 'u': /* VS1 == VS2 */
		  if (!reg_lookup (&s, RCLASS_VECR, &regno))
		    break;
		  INSERT_OPERAND (VS1, *ip, regno);
		  INSERT_OPERAND (VS2, *ip, regno);
		  continue;

		case 'v': /* VD == VS1 == VS2 */
		  if (!reg_lookup (&s, RCLASS_VECR, &regno))
		    break;
		  INSERT_OPERAND (VD, *ip, regno);
		  INSERT_OPERAND (VS1, *ip, regno);
		  INSERT_OPERAND (VS2, *ip, regno);
		  continue;

		/* The `V0` is carry-in register for v[m]adc and v[m]sbc,
		   and is used to choose vs1/rs1/frs1/imm or vs2 for
		   v[f]merge.  It use the same encoding as the vector mask
		   register.  */
		case '0':
		  if (reg_lookup (&s, RCLASS_VECR, &regno) && regno == 0)
		    continue;
		  break;

		case 'b': /* vtypei for vsetivli */
		  my_getVsetvliExpression (imm_expr, s);
		  check_absolute_expr (ip, imm_expr, FALSE);
		  if (!VALID_RVV_VB_IMM (imm_expr->X_add_number))
		    as_bad (_("bad value for vsetivli immediate field, "
			      "value must be 0..1023"));
		  ip->insn_opcode
		    |= ENCODE_RVV_VB_IMM (imm_expr->X_add_number);
		  imm_expr->X_op = O_absent;
		  s = expr_end;
		  continue;

		case 'c': /* vtypei for vsetvli */
		  my_getVsetvliExpression (imm_expr, s);
		  check_absolute_expr (ip, imm_expr, FALSE);
		  if (!VALID_RVV_VC_IMM (imm_expr->X_add_number))
		    as_bad (_("bad value for vsetvli immediate field, "
			      "value must be 0..2047"));
		  ip->insn_opcode
		    |= ENCODE_RVV_VC_IMM (imm_expr->X_add_number);
		  imm_expr->X_op = O_absent;
		  s = expr_end;
		  continue;

		case 'i': /* vector arith signed immediate */
		  my_getExpression (imm_expr, s);
		  check_absolute_expr (ip, imm_expr, FALSE);
		  if (imm_expr->X_add_number > 15
		      || imm_expr->X_add_number < -16)
		    as_bad (_("bad value for vector immediate field, "
			      "value must be -16...15"));
		  INSERT_OPERAND (VIMM, *ip, imm_expr->X_add_number);
		  imm_expr->X_op = O_absent;
		  s = expr_end;
		  continue;

		case 'j': /* vector arith unsigned immediate */
		  my_getExpression (imm_expr, s);
		  check_absolute_expr (ip, imm_expr, FALSE);
		  if (imm_expr->X_add_number < 0
		      || imm_expr->X_add_number >= 32)
		    as_bad (_("bad value for vector immediate field, "
			      "value must be 0...31"));
		  INSERT_OPERAND (VIMM, *ip, imm_expr->X_add_number);
		  imm_expr->X_op = O_absent;
		  s = expr_end;
		  continue;

		case 'k': /* vector arith signed immediate, minus 1 */
		  my_getExpression (imm_expr, s);
		  check_absolute_expr (ip, imm_expr, FALSE);
		  if (imm_expr->X_add_number > 16
		      || imm_expr->X_add_number < -15)
		    as_bad (_("bad value for vector immediate field, "
			      "value must be -15...16"));
		  INSERT_OPERAND (VIMM, *ip, imm_expr->X_add_number - 1);
		  imm_expr->X_op = O_absent;
		  s = expr_end;
		  continue;

		case 'm': /* optional vector mask */
		  if (*s == '\0')
		    {
		      INSERT_OPERAND (VMASK, *ip, 1);
		      continue;
		    }
		  else if (*s == ',' && s++
			   && reg_lookup (&s, RCLASS_VECM, &regno)
			   && regno == 0)
		    {
		      INSERT_OPERAND (VMASK, *ip, 0);
		      continue;
		    }
		  break;

		  /* The following ones are only used in macros.  */
		case 'M': /* required vector mask */
		  if (reg_lookup (&s, RCLASS_VECM, &regno) && regno == 0)
		    {
		      INSERT_OPERAND (VMASK, *ip, 0);
		      continue;
		    }
		  break;

		case 'T': /* vector macro temporary register */
		  if (!reg_lookup (&s, RCLASS_VECR, &regno) || regno == 0)
		    break;
		  /* Store it in the FUNCT6 field as we don't have anyplace
		     else to store it.  */
		  INSERT_OPERAND (VFUNCT6, *ip, regno);
		  continue;
		}
	      break;

	    case 'f': /* Prefetch offset, pseudo S-type but lower 5-bits zero.  */
	      if (riscv_handle_implicit_zero_offset (imm_expr, s))
		continue;
	      my_getExpression (imm_expr, s);
	      check_absolute_expr (ip, imm_expr, FALSE);
	      if (((unsigned) (imm_expr->X_add_number) & 0x1f)
		  || imm_expr->X_add_number >= (signed)RISCV_IMM_REACH/2
		  || imm_expr->X_add_number < -(signed)RISCV_IMM_REACH/2)
		as_bad (_("improper prefetch offset (%ld)"),
			(long) imm_expr->X_add_number);
	      ip->insn_opcode |= ENCODE_STYPE_IMM (imm_expr->X_add_number);
	      ip->insn_opcode &= ~ ENCODE_STYPE_IMM (0x1fU);
	      imm_expr->X_op = O_absent;
	      s = expr_end;
	      continue;

	    case 'h':		/* Upper unsigned 6-bit immediate.  */
	      my_getExpression (imm_expr, s);
	      if (imm_expr->X_op != O_constant
		  || imm_expr->X_add_number >= xlen
		  || imm_expr->X_add_number < 0)
		break;
	      s = expr_end;
	      ip->insn_opcode |= ENCODE_SBTYPE_IMM6H (imm_expr->X_add_number);
	      imm_expr->X_op = O_absent;
	      continue;

	    case 'l':		/* Lower unsigned 6-bit immediate.  */
	      my_getExpression (imm_expr, s);
	      if (imm_expr->X_op != O_constant
		  || imm_expr->X_add_number >= xlen
		  || imm_expr->X_add_number < 0)
		break;
	      ip->insn_opcode |= ENCODE_SBTYPE_IMM6L (imm_expr->X_add_number);
	      s = expr_end;
	      imm_expr->X_op = O_absent;
	      continue;

	    case 'i':           /* Signed 7-bit immediate in [31:25].  */
	      my_getExpression (imm_expr, s);
	      if (imm_expr->X_op != O_constant
		  || imm_expr->X_add_number >= (signed) RISCV_IMM7_REACH
		  || imm_expr->X_add_number < 0)
		break;
	      ip->insn_opcode |= ENCODE_STYPE_IMM7 (imm_expr->X_add_number);
	      s = expr_end;
	      imm_expr->X_op = O_absent;
	      continue;

	    case 'g':		/* 10bits PC-relative offset.  */
	      *imm_reloc = BFD_RELOC_RISCV_10_PCREL;
	      my_getExpression (imm_expr, s);
	      s = expr_end;
	      continue;

	    case 'k':		/* Cimm unsigned 6-bit immediate.  */
	      my_getExpression (imm_expr, s);
	      if (imm_expr->X_op != O_constant
		  || imm_expr->X_add_number >= xlen
		  || imm_expr->X_add_number < 0)
		break;
	      s = expr_end;
	      ip->insn_opcode |= ENCODE_TYPE_CIMM6 (imm_expr->X_add_number);
	      imm_expr->X_op = O_absent;
	      continue;

	    case 'v':		/* 2-bits shift value.  */
	      if (*s != '<' || *(s + 1) != '<')
		break;
	      s = s + 2;
	      my_getExpression (imm_expr, s);
	      if (imm_expr->X_op != O_constant
		  || imm_expr->X_add_number > 3
		  || imm_expr->X_add_number < 0)
		break;
	      INSERT_OPERAND (SV, *ip, imm_expr->X_add_number);
	      imm_expr->X_op = O_absent;
	      s = expr_end;
	      continue;

	    case 'H':
	    case 'G':
	      {
		bfd_boolean store = FALSE;
		if (*args == 'H')
		  store = TRUE;

		my_getExpression (imm_expr, s);
		switch (*++args)
		  {
		  case 'b':
		    if (imm_expr->X_op == O_constant
			&& imm_expr->X_add_number < (signed) RISCV_IMM18_REACH/2
			&& imm_expr->X_add_number >= -(signed) RISCV_IMM18_REACH/2)
		      {
			if (store)
			  ip->insn_opcode |= ENCODE_GPTYPE_SB_IMM (imm_expr->X_add_number);
			else
			  ip->insn_opcode |= ENCODE_GPTYPE_LB_IMM (imm_expr->X_add_number);
			s = expr_end;
			continue;
		      }
		    else if (imm_expr->X_op == O_symbol)
		      {
			if (store)
			  *imm_reloc = BFD_RELOC_RISCV_SGP18S0;
			else
			  *imm_reloc = BFD_RELOC_RISCV_LGP18S0;
			s = expr_end;
			continue;
		      }
		    break;

		  case 'h':
		    if (imm_expr->X_op == O_constant
			&& imm_expr->X_add_number < (signed) RISCV_IMM18_REACH/2
			&& imm_expr->X_add_number >= -(signed) RISCV_IMM18_REACH/2
			&& (imm_expr->X_add_number & 0x1) == 0)
		      {
			if (store)
			  ip->insn_opcode |= ENCODE_GPTYPE_SH_IMM (imm_expr->X_add_number);
			else
			  ip->insn_opcode |= ENCODE_GPTYPE_LH_IMM (imm_expr->X_add_number);
			s = expr_end;
			continue;
		      }
		    else if (imm_expr->X_op == O_symbol)
		      {
			if (store)
			  *imm_reloc = BFD_RELOC_RISCV_SGP17S1;
			else
			  *imm_reloc = BFD_RELOC_RISCV_LGP17S1;
			s = expr_end;
			continue;
		      }
		    break;

		  case 'w':
		    if (imm_expr->X_op == O_constant
			&& imm_expr->X_add_number < (signed) RISCV_IMM19_REACH/2
			&& imm_expr->X_add_number >= -(signed) RISCV_IMM19_REACH/2
			&& (imm_expr->X_add_number & 0x3) == 0)
		      {
			if (store)
			  ip->insn_opcode |= ENCODE_GPTYPE_SW_IMM (imm_expr->X_add_number);
			else
			  ip->insn_opcode |= ENCODE_GPTYPE_LW_IMM (imm_expr->X_add_number);
			s = expr_end;
			continue;
		      }
		    else if (imm_expr->X_op == O_symbol)
		      {
			if (store)
			  *imm_reloc = BFD_RELOC_RISCV_SGP17S2;
			else
			  *imm_reloc = BFD_RELOC_RISCV_LGP17S2;
			s = expr_end;
			continue;
		      }
		    break;

		  case 'd':
		    if (imm_expr->X_op == O_constant
			&& imm_expr->X_add_number < (signed) RISCV_IMM20_REACH/2
			&& imm_expr->X_add_number >= -(signed) RISCV_IMM20_REACH/2
			&& (imm_expr->X_add_number & 0x7) == 0)
		      {
			if (store)
			  ip->insn_opcode |= ENCODE_GPTYPE_SD_IMM (imm_expr->X_add_number);
			else
			  ip->insn_opcode |= ENCODE_GPTYPE_LD_IMM (imm_expr->X_add_number);
			s = expr_end;
			continue;
		      }
		    else if (imm_expr->X_op == O_symbol)
		      {
			if (store)
			  *imm_reloc = BFD_RELOC_RISCV_SGP17S3;
			else
			  *imm_reloc = BFD_RELOC_RISCV_LGP17S3;
			s = expr_end;
			continue;
		      }
		    break;

		  default:
		    break;
		  }
		break;
	      }

	    /* Handle operand fields of V5 extension.  */
	    case 'n':
	      {
		char field_name[MAX_KEYWORD_LEN];
		args++;
		/* TODO: build hash table to store the nds-defined fields.  */
		if (parse_nds_v5_field (&args, field_name))
		  {
		    if (strcmp (field_name, "nds_rc") == 0
			&& reg_lookup (&s, RCLASS_GPR, &regno))
		      {
			INSERT_OPERAND (RC, *ip, regno);
			args--;
			continue;
		      }
		    else if (strcmp (field_name, "nds_rdp") == 0
			     && reg_lookup (&s, RCLASS_GPR, &regno))
		      {
			if (xlen == 32 && (regno % 2) != 0)
			  {
			    as_bad (_("The number of Rd must be even "
				      "(limitation of register pair)."));
			    break;
			  }
			INSERT_OPERAND (RD, *ip, regno);
			args--;
			continue;
		      }
		    else if (strcmp (field_name, "nds_rsp") == 0
			     && reg_lookup (&s, RCLASS_GPR, &regno))
		      {
			if (xlen == 32 && (regno % 2) != 0)
			  {
			    as_bad (_("The number of Rs1 must be even "
				      "(limitation of register pair)."));
			    break;
			  }
			INSERT_OPERAND (RS1, *ip, regno);
			args--;
			continue;
		      }
		    else if (strcmp (field_name, "nds_rtp") == 0
			     && reg_lookup (&s, RCLASS_GPR, &regno))
		      {
			if (xlen == 32 && (regno % 2) != 0)
			  {
			    as_bad (_("The number of Rs2 must be even "
				      "(limitation of register pair)."));
			    break;
			  }
			INSERT_OPERAND (RS2, *ip, regno);
			args--;
			continue;
		      }

		    my_getExpression (imm_expr, s);
		    if (imm_expr->X_op != O_constant
			|| imm_expr->X_add_number >= xlen
			|| imm_expr->X_add_number < 0)
		      break;

		    if (strcmp (field_name, "nds_i3u") == 0
			&& VALID_PTYPE_IMM3U (imm_expr->X_add_number))
		      ip->insn_opcode |= ENCODE_PTYPE_IMM3U (imm_expr->X_add_number);
		    else if (strcmp (field_name, "nds_i4u") == 0
			     && VALID_PTYPE_IMM4U (imm_expr->X_add_number))
		      ip->insn_opcode |= ENCODE_PTYPE_IMM4U (imm_expr->X_add_number);
		    else if (strcmp (field_name, "nds_i5u") == 0
			     && VALID_PTYPE_IMM5U (imm_expr->X_add_number))
		      ip->insn_opcode |= ENCODE_PTYPE_IMM5U (imm_expr->X_add_number);
		    else if (strcmp (field_name, "nds_i6u") == 0
			     && VALID_PTYPE_IMM6U (imm_expr->X_add_number))
		      ip->insn_opcode |= ENCODE_PTYPE_IMM6U (imm_expr->X_add_number);
		    else if (strcmp (field_name, "nds_i15s") == 0
			     && VALID_PTYPE_IMM15S (imm_expr->X_add_number))
		      ip->insn_opcode |= ENCODE_PTYPE_IMM15S (imm_expr->X_add_number);
		    else
		      break;

		    s = expr_end;
		    imm_expr->X_op = O_absent;
		    args--;
		    continue;
		  }
		break;
	      }

	    /* Handle operand fields of ACE.  */
	    case 'X':
	      if (ace_lib_load_success)
		{
		  ace_ip ((char **) &args, &s, ip);
		  continue;
		}
	      else
		break;

	    case 'N': /* Andes extensions */
	      {
		switch (*++args)
		  {
		  case 'f': /* TO REMOVE */
		    my_getExpression (imm_expr, s);
		    if (imm_expr->X_op != O_constant
			|| imm_expr->X_add_number >= (signed) RISCV_IMM8_REACH/2
			|| imm_expr->X_add_number < -(signed) RISCV_IMM8_REACH/2)
		      break;
		    ip->insn_opcode |= ENCODE_TYPE_IMM8 (imm_expr->X_add_number);
		    s = expr_end;
		    imm_expr->X_op = O_absent;
		    continue;
		  default:
		    as_bad (_("Unknown Anndes extension operand 'N%c'"), *args);
		    break;
		  }
	      }
	      break;

	    default:
	      as_fatal (_("internal error: bad argument type %c"), *args);
	    }
	  break;
	}
      s = argsStart;
      if (error == default_error)
	error = _("illegal operands");
      insn_with_csr = FALSE;
    }

out:
  /* Restore the character we might have clobbered above.  */
  if (save_c)
    *(argsStart - 1) = save_c;

  if ((error == NULL) && riscv_opts.workaround)
    {
      if (riscv_opts.b19758)
	{
	  if (is_b19758_associated_insn(insn))
	    {
	      s = (char*)"iorw";
	      arg_lookup (&s, riscv_pred_succ, ARRAY_SIZE (riscv_pred_succ), &regno);
              macro_build(NULL, "fence", "P,Q", regno, regno);
	    }
	}

      if (riscv_opts.b20282)
	{
	  if (pre_insn_is_a_cond_br && is_indirect_jump(insn))
	    {
	      macro_build(NULL, "nop", "");
	      pre_insn_is_a_cond_br = FALSE;
	    }
	  else
	    {
	      pre_insn_is_a_cond_br = is_conditional_branch(insn);
	    }
	}

      if ((riscv_opts.b22827 || riscv_opts.b22827_1)
	  && !riscv_subset_supports ("v"))
	{
	  const struct riscv_opcode *prev_insn = nds.prev_insn.insn_mo;
	  insn_t prev_insn_co = nds.prev_insn.insn_opcode;
	  insn_t curr_insn_co = ip->insn_opcode;

	/* insert “fclass.x x0, RD(FDIV/FSQRT)” after FDIV/FSQRT unless 
	 * the next immediate instruction is 
	 * “fsub/fadd/fmul/fmadd/fsqrt/fdiv/jal/ret and their 16bit variants”
	 * NOTE: by jal I mean jal and jral. Ret includes jr. 
	 * If you can accept more complex conditions, RD(FDIV/FSQRT) has to be
	 * in fa0-7 to exclude jal/ret.
	 */
	  if (riscv_opts.b22827
	      && is_insn_fdiv_or_fsqrt (prev_insn)
	      && is_insn_in_b22827_list (insn, prev_insn_co, curr_insn_co))
	    {
	      nds.frag_b22827->fr_var = 1;
	    }

	/* to provide a separate flag to turn it off, with the following rule:
	 * If FSHW is followed by any floating-point instructions (including 
	 * FSHW and FLHW), insert a NOP after it.
	 */
	  else if (riscv_opts.b22827_1
	      && is_insn_fshw (prev_insn)
	      && is_insn_of_fp_types (insn))
	    {
	      nds.frag_b22827->fr_var = 1;
	    }

	  /* update previous insns  */
	  nds.prev_insn = *ip;
	}
    }

  return error;
}

#ifdef TO_REMOVE
/* Parse the version of ISA in .attribute directive.  */

static int
riscv_parse_arch_version (const char **in_ver)
{
  int version, num, major_set, minor_set;
  const char *string = *in_ver;

  version = 0;
  num = 0;
  major_set = 0;
  minor_set = 0;
  /* Major version.  */
  while (string[0] != '\0'
	 && string[0] != 'p'
	 && (string[0] - 48) >= 0
	 && (string[0] - 48) <= 9)
    {
      num = num * 10 + (string[0] - 48);
      string++;
      major_set = 1;
    }
  version = num * 10000;
  /* Minor verison.  */
  if (major_set && string[0] == 'p')
    {
      num = 0;
      string++;
      while (string[0] != '\0'
	     && (string[0] - 48) >= 0
	     && (string[0] - 48) <= 9)
	{
	  num = num * 10 + (string[0] - 48);
	  string++;
	  if (num >= 10000)
	    as_fatal (".attribute: minor version can not "
		      "be larger than 9999.");
	  minor_set = 1;
	}
      version += num;

      /* Major and Minor versions must be set or unset both.  */
      if (major_set ^ minor_set)
	as_fatal (".attribute: major and minor versions must be "
		  "set when 'p' is used.");
    }
  *in_ver = string;

  if (version > 0 || major_set)
    return version;
  else
    /* Use default version.  */
    return -1;
}

/* Parse the name of ISA in .attribute directive.  */

static void
riscv_parse_arch_name (const char **in_arch, int len, char **name)
{
  /* Parse the non-standard version name.  */
  const char *string = *in_arch;
  if (!len)
    {
      int i = 0, j = 0;
      for (; non_standard_arch_name[j]; j++)
	{
	  if (strncmp (string, non_standard_arch_name[j],
		       strlen (non_standard_arch_name[j])) == 0)
	    {
	      i += strlen (non_standard_arch_name[j]);
	      break;
	    }
	}
      /* No match.  */
      if (i == 0)
	while (string[i] != '\0'
	       && string[i] != '_'
	       && ((string[i] - 48) < 0
		   || (string[i] - 48) > 9))
	  i++;

      /* The first char 'x' is a keyword.  */
      if (i == 1)
	as_fatal (".attribute %s: empty non standard ISA extension?",
		  *in_arch);
      else
	len = i;
    }

  *name = (char *) malloc ((len + 1) * sizeof (char));
  memcpy (*name, *in_arch, len);
  memcpy (*name + len, "\0", 1);
  *in_arch = string + len;
}

/* Convert the version from integer to string.  */

static void
riscv_arch_version_int2str (int version, char *str, int minor)
{
  if (minor)
    sprintf (str, "%d", version % 10000);
  else
    sprintf (str, "%d", version / 10000);
}

static void
riscv_update_arch_info_hash (const char *arch, int version,
			     bfd_boolean update)
{
  const char *key;
  struct arch_info *info;
  char str[32];

  if (!update)
    return;

  info = (struct arch_info *) hash_find (arch_info_hash, arch);
  if (info)
    {
      if (version != -1)
	{
	  riscv_arch_version_int2str (version, str, 0);
	  info->v_major = xstrdup (str);
	  riscv_arch_version_int2str (version, str, 1);
	  info->v_minor = xstrdup (str);
	}
      info->valid = 1;
    }
  else
    {
      if (version == -1)
	version = 0;
      struct arch_info *new = malloc (sizeof (struct arch_info));
      key = xstrdup (arch);
      new->name = key;
      riscv_arch_version_int2str (version, str, 0);
      new->v_major = xstrdup (str);
      riscv_arch_version_int2str (version, str, 1);
      new->v_minor = xstrdup (str);
      new->valid = 1;

      const char *hash_error =
	hash_insert (arch_info_hash, key, (void *) new);
      if (hash_error)
	{
	  fprintf (stderr, _("internal error: can't hash `%s': %s\n"),
		   new->name, hash_error);
	}
    }
}

static int
strincmp(char const *a, char const *b, size_t sz)
{
  size_t i;
  if (!a || !b)
    return -1;

  for (i = 0; i < sz; a++, b++, i++)
    {
      int d = TOLOWER((unsigned char)*a) - TOLOWER((unsigned char)*b);
      if (d != 0 || !*a || !*b)
        return d;
    }

  return 0;
}

static bfd_boolean
riscv_parse_arch_attribute (const char *in_arch, bfd_boolean update)
{
  const char *all_subsets = "imafdqcp";
  char *name;
  int version;
  int parse_non_standard = 0;
  const char *in_arch_p = in_arch;

  /* Clear the subsets set by -march option.  */
  /* riscv_release_subset_list (&riscv_subsets); */

  /* Clear the riscv_opts.rvc if priority is higher.  */
  riscv_set_rvc (FALSE);

  if (strincmp (in_arch_p, "rv32", 4) == 0)
    {
      xlen = 32;
      in_arch_p += 4;
    }
  else if (strincmp (in_arch_p, "rv64", 4) == 0)
    {
      xlen = 64;
      in_arch_p += 4;
    }
  else
    as_fatal (".attribute %s: ISA string must begin with rv32/rv64",
	      in_arch_p);

  switch (TOLOWER(*in_arch_p))
    {
    case 'e':
      /* FIXME: the extensions after 'e' only can be M, A and C.  */
      in_arch_p++;
      version = riscv_parse_arch_version (&in_arch_p);
      riscv_update_arch_info_hash ("e", version, update);
      riscv_add_subset (&riscv_subsets, "e", 0, 0);
      riscv_add_subset (&riscv_subsets, "i", 0, 0);

      riscv_set_rve (TRUE);
      break;
    case 'g':
      in_arch_p++;
      version = riscv_parse_arch_version (&in_arch_p);
      for ( ; *all_subsets != 'q'; all_subsets++)
	{
	  const char subset[] = {*all_subsets, '\0'};
	  riscv_update_arch_info_hash (subset, version, update);
	  riscv_add_subset (&riscv_subsets, subset, 0, 0);
	}
      /* Addition.  */
      if (!riscv_opts.no_16_bit)
	{
	  riscv_update_arch_info_hash ("c", version, update);
	  riscv_set_rvc (TRUE);
	}
      break;
    case 'i':
      break;
    default:
      as_fatal (".attribute %s: first ISA subset must be i/g/e",
		in_arch_p);
    }

  while (*in_arch_p)
    {
      switch (TOLOWER(*in_arch_p))
	{
	case 'c':
	  if (riscv_opts.no_16_bit)
	    {
	      /* We still need to parse the `c' ext here, but
		 don't update the arch hash table.  */
	      name = NULL;
	      riscv_parse_arch_name (&in_arch_p, 1, &name);
	      version = riscv_parse_arch_version (&in_arch_p);
	      free ((char *) name);
	      break;
	    }
	  riscv_set_rvc (TRUE);
	  /* Fall through.  */
	case 'i':
	case 'm':
	case 'a':
	case 'f':
	case 'd':
	case 'q':
	case 'p':
	  /* Standard extensions.  */
	  if (!parse_non_standard)
	    {
	      name = NULL;
	      riscv_parse_arch_name (&in_arch_p, 1, &name);
	      version = riscv_parse_arch_version (&in_arch_p);
	      riscv_update_arch_info_hash (name, version, update);
	      riscv_add_subset (&riscv_subsets, name, 0, 0);

	      free ((char *) name);
	    }
	  else
	    as_fatal (".attribute %c: Standard ISA subset can not "
		      "be set after Non-standard ISA subset.",
		      *in_arch_p);
	  break;
	case 'x':
	  /* Non-standard extensions.  */
	  parse_non_standard = 1;
	  name = NULL;
	  riscv_parse_arch_name (&in_arch_p, 0, &name);
	  version = riscv_parse_arch_version (&in_arch_p);
	  if (strcmp (name, "xv5") == 0)
	    {
	      if (version == 0)
		riscv_opts.efhw = TRUE; /* 0p0 imply X_efhw */
	      if (version == -1)
		version = 10001;        /* default version: 1p1 */
	      riscv_update_arch_info_hash ("xv5-", version, update);
	      riscv_add_subset (&riscv_subsets, "xv5-", version/10000, version%10000);
	    }
	  else
	    {
	      riscv_update_arch_info_hash (name, version, update);
	      riscv_add_subset (&riscv_subsets, name, version/10000, version%10000);
	    }

	  if (*in_arch_p == '_')
	    in_arch_p++;

	  free ((char *) name);
	  break;
	default:
	  as_fatal (".attribute %s: ISA subset is unsupported",
		    in_arch_p);
	}
    }

  /* We must keep the extension set by the options if needed.  */
  if (riscv_opts.atomic)
    riscv_add_subset (&riscv_subsets, "a", 2, 0);

  if (riscv_opts.dsp)
    riscv_add_subset (&riscv_subsets, "p", 0, 5);

  /* default version of "xefhw": 1p0  */
  if (riscv_opts.efhw)
    riscv_add_subset (&riscv_subsets, "xefhw", 1, 0);

  if (riscv_opts.vector)
    riscv_add_subset (&riscv_subsets, "v", 1, 0);

  /* Always add `c' into `all_subsets' for the `riscv_opcodes' table.  */
  riscv_add_subset (&riscv_subsets, "c", 2, 0);

  return TRUE;
}

/* The priority of arch setting (attribute and subset extension):
   1. Ext options: -mext-dsp, -mno-16-bit.
   2. .option rvc.
   3. Attribute directive: Only use the last one.
   4. -march option.
   5. default_arch.

   First, call the riscv_set_arch_attributes when encountering the
   `-march' option, otherwise, call it with the `default_arch' in the
   riscv_after_parse_args. If attribute directive is set, we need
   to clean the previous setting, and then reset the subsets again
   according to the attribute directive.  */

static void
riscv_set_arch_attributes (const char *name)
{
  obj_attribute *attr;
  const char *string;
  bfd_boolean update;

  /* We can not update the arch hash table when parsing architectures
     for the `-march' option and `default_arch' since the hash table has
     not been initialized yet. I prefer to keep the initialization in
     the md_begin, and then use a boolean variable `update' to avoid
     the segment fault. If there is no attribute directive, then we will
     update the hash table in the riscv_write_out_arch_attr according to
     the `all_subsets' set by `-march' or `default_arch'.  */
  if (name)
    {
      string = name;
      update = TRUE;
    }
  else
    {
      attr = elf_known_obj_attributes_proc (stdoutput);
      string = attr[Tag_arch].s;
      update = TRUE;
    }

  if (string && !riscv_parse_arch_attribute (string, update))
    as_fatal ("internal error: cannot parse .attribute %s",
	      string);
}
#endif

void
md_assemble (char *str)
{
  struct riscv_cl_insn insn;
  expressionS imm_expr;
  bfd_reloc_code_real_type imm_reloc = BFD_RELOC_UNUSED;
  imm_expr.X_md = 0;
  insn.cmodel.method = METHOD_DEFAULT;

  /* Set the first rvc info for the the current fragmant.  */
  if (!frag_now->tc_frag_data.rvc)
    frag_now->tc_frag_data.rvc = riscv_opts.rvc ? 1 : -1;

  /* The arch and priv attributes should be set before assembling.  */
  if (!start_assemble)
    {
      start_assemble = TRUE;
      riscv_set_abi_by_arch ();

      /* Set the default_priv_spec according to the priv attributes.  */
      if (!riscv_set_default_priv_spec (NULL))
       return;

      arch_sanity_check(TRUE); /* is_final = TRUE  */
      memset (&nds, 0, sizeof (nds));
    }

  const char *error = riscv_ip (str, &insn, &imm_expr, &imm_reloc, op_hash);

  if (error)
    {
      as_bad ("%s `%s'", error, str);
      return;
    }

  if (insn.insn_mo->pinfo == INSN_MACRO)
    macro (&insn, &imm_expr, &imm_reloc);
  else
    append_insn (&insn, &imm_expr, imm_reloc);
}

const char *
md_atof (int type, char *litP, int *sizeP)
{
  return ieee_md_atof (type, litP, sizeP, TARGET_BYTES_BIG_ENDIAN);
}

void
md_number_to_chars (char *buf, valueT val, int n)
{
  number_to_chars_littleendian (buf, val, n);
}

const char *md_shortopts = "O::g::G:";

enum options
{
  OPTION_MARCH = OPTION_MD_BASE,
  OPTION_PIC,
  OPTION_NO_PIC,
  OPTION_MABI,
  OPTION_RELAX,
  OPTION_NO_RELAX,
  OPTION_ARCH_ATTR,
  OPTION_NO_ARCH_ATTR,
  OPTION_CSR_CHECK,
  OPTION_NO_CSR_CHECK,
  OPTION_MISA_SPEC,
  OPTION_MPRIV_SPEC,
  OPTION_BIG_ENDIAN,
  OPTION_LITTLE_ENDIAN,
  /* RVV  */
  OPTION_CHECK_CONSTRAINTS,
  OPTION_NO_CHECK_CONSTRAINTS,
  /* Andes  */
  OPTION_NO_16_BIT,
  OPTION_MATOMIC,
  OPTION_ACE,
  OPTION_OPTIMIZE,
  OPTION_OPTIMIZE_SPACE,
  OPTION_MEXT_DSP,
  OPTION_MEXT_EFHW,
  OPTION_MEXT_VECTOR,
  OPTION_MICT_MODEL,
  OPTION_MCMODEL,
  OPTION_MNO_VIC,
  OPTION_MNO_B19758,
  OPTION_MB20282,
  OPTION_MB22827,
  OPTION_MB22827_1,
  OPTION_MNO_WORKAROUND,
  OPTION_END_OF_ENUM
};

struct option md_longopts[] =
{
  {"march", required_argument, NULL, OPTION_MARCH},
  {"fPIC", no_argument, NULL, OPTION_PIC},
  {"fpic", no_argument, NULL, OPTION_PIC},
  {"fno-pic", no_argument, NULL, OPTION_NO_PIC},
  {"mabi", required_argument, NULL, OPTION_MABI},
  {"mrelax", no_argument, NULL, OPTION_RELAX},
  {"mno-relax", no_argument, NULL, OPTION_NO_RELAX},
  {"march-attr", no_argument, NULL, OPTION_ARCH_ATTR},
  {"mno-arch-attr", no_argument, NULL, OPTION_NO_ARCH_ATTR},
  {"mcsr-check", no_argument, NULL, OPTION_CSR_CHECK},
  {"mno-csr-check", no_argument, NULL, OPTION_NO_CSR_CHECK},
  {"misa-spec", required_argument, NULL, OPTION_MISA_SPEC},
  {"mpriv-spec", required_argument, NULL, OPTION_MPRIV_SPEC},
  {"mbig-endian", no_argument, NULL, OPTION_BIG_ENDIAN},
  {"mlittle-endian", no_argument, NULL, OPTION_LITTLE_ENDIAN},
  /* RVV  */
  {"mcheck-constraints", no_argument, NULL, OPTION_CHECK_CONSTRAINTS},
  {"mno-check-constraints", no_argument, NULL, OPTION_NO_CHECK_CONSTRAINTS},
  /* Andes  */
  {"mno-16-bit", no_argument, NULL, OPTION_NO_16_BIT},
  {"matomic", no_argument, NULL, OPTION_MATOMIC},
  {"mace", required_argument, NULL, OPTION_ACE},
  {"O1", no_argument, NULL, OPTION_OPTIMIZE},
  {"Os", no_argument, NULL, OPTION_OPTIMIZE_SPACE},
  {"mext-dsp", no_argument, NULL, OPTION_MEXT_DSP},
  {"mext-efhw", no_argument, NULL, OPTION_MEXT_EFHW},
  {"mext-vector", no_argument, NULL, OPTION_MEXT_VECTOR},
  {"mict-model", required_argument, NULL, OPTION_MICT_MODEL},
  {"mcmodel", required_argument, NULL, OPTION_MCMODEL},
  {"mno-check-constraints", no_argument, NULL, OPTION_MNO_VIC},
  {"mno-b19758", no_argument, NULL, OPTION_MNO_B19758},
  {"mb20282", no_argument, NULL, OPTION_MB20282},
  {"mb22827", no_argument, NULL, OPTION_MB22827},
  {"mb22827.1", no_argument, NULL, OPTION_MB22827_1},
  {"mno-workaround", no_argument, NULL, OPTION_MNO_WORKAROUND},
  
  {NULL, no_argument, NULL, 0}
};
size_t md_longopts_size = sizeof (md_longopts);

int
md_parse_option (int c, const char *arg)
{
  switch (c)
    {
    case OPTION_MARCH:
      /* riscv_after_parse_args will call riscv_set_arch to parse
        the architecture.  */
      default_arch_with_ext = arg;
      break;

    case OPTION_NO_PIC:
      riscv_opts.pic = FALSE;
      break;

    case OPTION_PIC:
      riscv_opts.pic = TRUE;
      break;

    case OPTION_MABI:
      if (strcmp (arg, "ilp32") == 0)
	riscv_set_abi (32, FLOAT_ABI_SOFT, FALSE);
      else if (strcmp (arg, "ilp32e") == 0)
	riscv_set_abi (32, FLOAT_ABI_SOFT, TRUE);
      else if (strcmp (arg, "ilp32f") == 0)
	riscv_set_abi (32, FLOAT_ABI_SINGLE, FALSE);
      else if (strcmp (arg, "ilp32d") == 0)
	riscv_set_abi (32, FLOAT_ABI_DOUBLE, FALSE);
      else if (strcmp (arg, "ilp32q") == 0)
	riscv_set_abi (32, FLOAT_ABI_QUAD, FALSE);
      else if (strcmp (arg, "lp64") == 0)
	riscv_set_abi (64, FLOAT_ABI_SOFT, FALSE);
      else if (strcmp (arg, "lp64f") == 0)
	riscv_set_abi (64, FLOAT_ABI_SINGLE, FALSE);
      else if (strcmp (arg, "lp64d") == 0)
	riscv_set_abi (64, FLOAT_ABI_DOUBLE, FALSE);
      else if (strcmp (arg, "lp64q") == 0)
	riscv_set_abi (64, FLOAT_ABI_QUAD, FALSE);
      else
	return 0;
      explicit_mabi = TRUE;
      break;

    case OPTION_RELAX:
      riscv_opts.relax = TRUE;
      break;

    case OPTION_NO_RELAX:
      riscv_opts.relax = FALSE;
      break;

    case OPTION_ARCH_ATTR:
      riscv_opts.arch_attr = TRUE;
      break;

    case OPTION_NO_ARCH_ATTR:
      riscv_opts.arch_attr = FALSE;
      break;

    case OPTION_CSR_CHECK:
      riscv_opts.csr_check = TRUE;
      break;

    case OPTION_NO_CSR_CHECK:
      riscv_opts.csr_check = FALSE;
      break;

    case OPTION_MISA_SPEC:
      return riscv_set_default_isa_spec (arg);

    case OPTION_MPRIV_SPEC:
      return riscv_set_default_priv_spec (arg);

    case OPTION_BIG_ENDIAN:
      target_big_endian = 1;
      break;

    case OPTION_LITTLE_ENDIAN:
      target_big_endian = 0;
      break;

    case OPTION_CHECK_CONSTRAINTS:
      riscv_opts.check_constraints = TRUE;
      break;

    case OPTION_NO_CHECK_CONSTRAINTS:
      riscv_opts.check_constraints = FALSE;
      break;

    /* Load ACE shared library if ACE option is enable */
    case OPTION_ACE:
      {
#ifndef __MINGW32__
	void *dlc = dlopen (arg, RTLD_NOW | RTLD_LOCAL);
	char *err;

	if (dlc == NULL)
	  err = (char *) dlerror ();
	else
	  {
	    ace_opcs = (struct riscv_opcode *) dlsym (dlc, "ace_opcodes_2");
	    err = (char *) dlerror ();
	    if (err == NULL)
	      {
		ace_ops = (ace_op_t *) dlsym (dlc, "ace_operands");
		err = (char *) dlerror ();
	    }
	    if (err == NULL)
	      {
		ace_keys = (ace_keyword_t *) dlsym (dlc, "ace_keywords");
		err = (char *) dlerror ();
	      }
	  }

	if (err == NULL)
	  {
	    ace_lib_load_success = TRUE;
	    return 1;
	  }
	else
	  as_bad ("Fault to load ACE shared library: %s\n", err);
#endif
      }
      break;


    case OPTION_NO_16_BIT:
      riscv_opts.no_16_bit = TRUE;
      break;

    case OPTION_MATOMIC:
      riscv_opts.atomic = TRUE;
      break;

    case OPTION_OPTIMIZE:
      optimize = 1;
      optimize_for_space = 0;
      break;

    case OPTION_OPTIMIZE_SPACE:
      optimize = 0;
      optimize_for_space = 1;
      break;

    case OPTION_MEXT_DSP:
      riscv_opts.dsp = TRUE;
      break;

    case OPTION_MEXT_EFHW:
      riscv_opts.efhw = TRUE;
      break;

    case OPTION_MEXT_VECTOR:
      riscv_opts.vector = TRUE;
      break;

    case OPTION_MICT_MODEL:
      if (strcmp ("tiny", arg) == 0
	  || strcmp ("small", arg) == 0
	  || strcmp ("large", arg) == 0)
	m_ict_model = arg;
      else
	as_bad (_("invalid ICT model setting -mict-model=%s"), arg);
      break;

    case OPTION_MCMODEL:
      if (strcmp (arg, "large") == 0)
	riscv_opts.cmodel = CMODEL_LARGE;
      else if (strcmp (arg, "medany") == 0)
	riscv_opts.cmodel = CMODEL_DEFAULT;
      else if (strcmp (arg, "medlow") == 0)
	riscv_opts.cmodel = CMODEL_DEFAULT;
      else
	as_bad (_("invalid cmodel setting -mcmodel=%s"), arg);
      break;

    case OPTION_MNO_VIC:
      riscv_opts.no_vic = TRUE;
      opc_set_no_vic (riscv_opts.no_vic);
      break;

    case OPTION_MNO_B19758:
	riscv_opts.b19758 = 0;
      break;

    case OPTION_MB20282:
	riscv_opts.b20282 = 1;
      break;

    case OPTION_MB22827:
	riscv_opts.b22827 = 1;
      break;
    case OPTION_MB22827_1:
	riscv_opts.b22827_1 = 1;
      break;

    case OPTION_MNO_WORKAROUND:
	riscv_opts.workaround = 0;
      break;

    default:
      return 0;
    }

  return 1;
}

static void
riscv_add_subset_if_not_found (riscv_subset_list_t *subset_list,
				const char *subset,
				int major,
				int minor)
{
  int majorN, minorN;

  if (riscv_subset_supports (subset))
    return;

  majorN = minorN = 0;
  if ((major == RISCV_UNKNOWN_VERSION) || (minor == RISCV_UNKNOWN_VERSION))
    riscv_get_default_ext_version (subset, &majorN, &minorN);

  if (major != RISCV_UNKNOWN_VERSION)
    majorN = major;
  
  if (minor != RISCV_UNKNOWN_VERSION)
    minorN = minor;

  riscv_add_subset (subset_list, subset, majorN, minorN);
}

static void
arch_sanity_check (int is_final)
{
  if (is_final && riscv_opts.update_count == 0)
    return;
  riscv_opts.update_count = 0;

  if (is_final)
    { /* update riscv_opts  */
      if (riscv_opts.efhw == 0)
	riscv_opts.efhw = riscv_subset_supports ("xefhw");
      if (riscv_opts.vector == 0)
	riscv_opts.vector = riscv_subset_supports ("v");
    }

  /* We must keep the extension set by the options if needed.  */
  if (riscv_opts.atomic)
    riscv_add_subset_if_not_found (&riscv_subsets, "a",
      RISCV_UNKNOWN_VERSION, RISCV_UNKNOWN_VERSION);

  if (riscv_opts.dsp)
    riscv_add_subset_if_not_found (&riscv_subsets, "p",
      RISCV_UNKNOWN_VERSION, RISCV_UNKNOWN_VERSION);

  /* default version of "xefhw": 1p0  */
  if (riscv_opts.efhw)
    riscv_add_subset_if_not_found (&riscv_subsets, "xefhw", 1, 0);

  if (riscv_opts.vector)
    riscv_add_subset_if_not_found (&riscv_subsets, "v",
      RISCV_UNKNOWN_VERSION, RISCV_UNKNOWN_VERSION);

  /* Infer ABI from ISA if not specified on command line.  */
  if (abi_xlen == 0)
    abi_xlen = xlen;
  else if (abi_xlen > xlen)
    as_bad ("can't have %d-bit ABI on %d-bit ISA", abi_xlen, xlen);
  else if (abi_xlen < xlen)
    as_bad ("%d-bit ABI not yet supported on %d-bit ISA", abi_xlen, xlen);

  if (float_abi == FLOAT_ABI_DEFAULT)
    {
      riscv_subset_t *subset;

      /* Assume soft-float unless D extension is present.  */
      float_abi = FLOAT_ABI_SOFT;

      for (subset = riscv_subsets.head; subset != NULL; subset = subset->next)
	{
	  if (strcasecmp (subset->name, "D") == 0)
	    float_abi = FLOAT_ABI_DOUBLE;
	  if (strcasecmp (subset->name, "Q") == 0)
	    float_abi = FLOAT_ABI_QUAD;
	}
    }

  if (rve_abi)
    elf_flags |= EF_RISCV_RVE;

  /* Insert float_abi into the EF_RISCV_FLOAT_ABI field of elf_flags.  */
  elf_flags |= float_abi * (EF_RISCV_FLOAT_ABI & ~(EF_RISCV_FLOAT_ABI << 1));

  /* disable --cmodel=large if RV32  */
  if (riscv_opts.cmodel == CMODEL_LARGE && xlen <= 32)
	riscv_opts.cmodel = CMODEL_DEFAULT;
}

void
riscv_after_parse_args (void)
{
  /* The --with-arch is optional for now, so we have to set the xlen
     according to the default_arch, which is set by the --targte, first.
     Then, we use the xlen to set the default_arch_with_ext if the
     -march and --with-arch are not set.  */
  if (xlen == 0)
    {
      if (strncmp (default_arch, "rv32", 4) == 0)
	xlen = 32;
      else if (strncmp (default_arch, "rv64", 4) == 0)
	xlen = 64;
      else if (strcmp (default_arch, "riscv32") == 0)
	xlen = 32;
      else if (strcmp (default_arch, "riscv64") == 0)
	xlen = 64;
      else
	as_bad ("unknown default architecture `%s'", default_arch);
    }
  if (default_arch_with_ext == NULL)
    {
      if (strncmp (default_arch, "rv", 2) == 0)
	default_arch_with_ext = default_arch;
      else
	default_arch_with_ext = xlen == 64 ? "rv64g" : "rv32g";
    }

  /* Initialize the hash table for extensions with default version.  */
  ext_version_hash = init_ext_version_hash (riscv_ext_version_table);

  /* If the -misa-spec isn't set, then we set the default ISA spec according
     to DEFAULT_RISCV_ISA_SPEC.  */
  if (default_isa_spec == ISA_SPEC_CLASS_NONE)
    riscv_set_default_isa_spec (DEFAULT_RISCV_ISA_SPEC);

  /* Set the architecture according to -march or or --with-arch.  */
  riscv_set_arch (default_arch_with_ext);

  /* Add the RVC extension, regardless of -march, to support .option rvc.  */
  riscv_set_rvc (FALSE);
  if (riscv_subset_supports ("c"))
    riscv_set_rvc (TRUE);

  /* Enable RVE if specified by the -march option.  */
  riscv_set_rve (FALSE);
  if (riscv_subset_supports ("e"))
    riscv_set_rve (TRUE);

  /* If the -mpriv-spec isn't set, then we set the default privilege spec
     according to DEFAULT_PRIV_SPEC.  */
  if (default_priv_spec == PRIV_SPEC_CLASS_NONE)
    riscv_set_default_priv_spec (DEFAULT_RISCV_PRIV_SPEC);

  /* If the CIE to be produced has not been overridden on the command line,
     then produce version 3 by default.  This allows us to use the full
     range of registers in a .cfi_return_column directive.  */
  if (flag_dwarf_cie_version == -1)
    flag_dwarf_cie_version = 3;

  arch_sanity_check(FALSE); /* is_final = FALSE  */
}

long
md_pcrel_from (fixS *fixP)
{
  return fixP->fx_where + fixP->fx_frag->fr_address;
}

static void
riscv_convert_ict_relocs (fixS ** fix)
{
  switch ((*fix)->fx_r_type)
    {
    case BFD_RELOC_RISCV_HI20:
      if ((*fix)->tc_fix_data.ict == BFD_RELOC_RISCV_ICT_HI20)
        (*fix)->fx_r_type = BFD_RELOC_RISCV_ICT_HI20;
      break;
    case BFD_RELOC_RISCV_LO12_I:
      if ((*fix)->tc_fix_data.ict == BFD_RELOC_RISCV_ICT_HI20)
        (*fix)->fx_r_type = BFD_RELOC_RISCV_ICT_LO12_I;
      break;
    case BFD_RELOC_RISCV_PCREL_HI20:
      if ((*fix)->tc_fix_data.ict == BFD_RELOC_RISCV_ICT_HI20)
        (*fix)->fx_r_type = BFD_RELOC_RISCV_PCREL_ICT_HI20;
      break;
    case BFD_RELOC_RISCV_CALL:
      if ((*fix)->tc_fix_data.ict == BFD_RELOC_RISCV_ICT_HI20)
        (*fix)->fx_r_type = BFD_RELOC_RISCV_CALL_ICT;
      break;
    case BFD_RELOC_64:
      if ((*fix)->tc_fix_data.ict == BFD_RELOC_RISCV_ICT_HI20)
        (*fix)->fx_r_type = BFD_RELOC_RISCV_ICT_64;
      break;
    default:
      break;
    }
}

/* Apply a fixup to the object file.  */

void
md_apply_fix (fixS *fixP, valueT *valP, segT seg ATTRIBUTE_UNUSED)
{
  unsigned int subtype;
  bfd_byte *buf = (bfd_byte *) (fixP->fx_frag->fr_literal + fixP->fx_where);
  bfd_boolean relaxable = FALSE;
  offsetT loc;
  segT sub_segment;

  /* Remember value for tc_gen_reloc.  */
  fixP->fx_addnumber = *valP;

  /* Convert the correct ICT relocs according to the ict
     flag in the fixup.  */
  riscv_convert_ict_relocs (&fixP);

  switch (fixP->fx_r_type)
    {
    case BFD_RELOC_RISCV_HI20:
    case BFD_RELOC_RISCV_LO12_I:
    case BFD_RELOC_RISCV_LO12_S:
    case BFD_RELOC_RISCV_LALO_HI20:
    case BFD_RELOC_RISCV_LALO_LO12_I:
    case BFD_RELOC_RISCV_ICT_HI20:
    case BFD_RELOC_RISCV_ICT_LO12_I:
      bfd_putl32 (riscv_apply_const_reloc (fixP->fx_r_type, *valP)
		  | bfd_getl32 (buf), buf);
      if (fixP->fx_addsy == NULL)
	fixP->fx_done = TRUE;
      relaxable = TRUE;
      break;

    case BFD_RELOC_RISCV_GOT_HI20:
    case BFD_RELOC_RISCV_ADD8:
    case BFD_RELOC_RISCV_ADD16:
    case BFD_RELOC_RISCV_ADD32:
    case BFD_RELOC_RISCV_ADD64:
    case BFD_RELOC_RISCV_SUB6:
    case BFD_RELOC_RISCV_SUB8:
    case BFD_RELOC_RISCV_SUB16:
    case BFD_RELOC_RISCV_SUB32:
    case BFD_RELOC_RISCV_SUB64:
    case BFD_RELOC_RISCV_RELAX:
      break;

    case BFD_RELOC_RISCV_TPREL_HI20:
    case BFD_RELOC_RISCV_TPREL_LO12_I:
    case BFD_RELOC_RISCV_TPREL_LO12_S:
    case BFD_RELOC_RISCV_TPREL_ADD:
      relaxable = TRUE;
      /* Fall through.  */

    case BFD_RELOC_RISCV_TLS_GOT_HI20:
    case BFD_RELOC_RISCV_TLS_GD_HI20:
    case BFD_RELOC_RISCV_TLS_DTPREL32:
    case BFD_RELOC_RISCV_TLS_DTPREL64:
      if (fixP->fx_addsy != NULL)
	S_SET_THREAD_LOCAL (fixP->fx_addsy);
      else
	as_bad_where (fixP->fx_file, fixP->fx_line,
		      _("TLS relocation against a constant"));
      break;

    case BFD_RELOC_32:
      /* Use pc-relative relocation for FDE initial location.
	 The symbol address in .eh_frame may be adjusted in
	 _bfd_elf_discard_section_eh_frame, and the content of
	 .eh_frame will be adjusted in _bfd_elf_write_section_eh_frame.
	 Therefore, we cannot insert a relocation whose addend symbol is
	 in .eh_frame. Othrewise, the value may be adjusted twice.*/
      if (fixP->fx_addsy && fixP->fx_subsy
	  && (sub_segment = S_GET_SEGMENT (fixP->fx_subsy))
	  && strcmp (sub_segment->name, ".eh_frame") == 0
	  && S_GET_VALUE (fixP->fx_subsy)
	     == fixP->fx_frag->fr_address + fixP->fx_where)
	{
	  fixP->fx_r_type = BFD_RELOC_RISCV_32_PCREL;
	  fixP->fx_subsy = NULL;
	  break;
	}
      /* Fall through.  */
    case BFD_RELOC_64:
    case BFD_RELOC_16:
    case BFD_RELOC_8:
    case BFD_RELOC_RISCV_CFA:
      if (fixP->fx_addsy && fixP->fx_subsy)
	{
	  fixP->fx_next = xmemdup (fixP, sizeof (*fixP), sizeof (*fixP));
	  fixP->fx_next->fx_addsy = fixP->fx_subsy;
	  fixP->fx_next->fx_subsy = NULL;
	  fixP->fx_next->fx_offset = 0;
	  fixP->fx_subsy = NULL;

	  switch (fixP->fx_r_type)
	    {
	    case BFD_RELOC_64:
	      fixP->fx_r_type = BFD_RELOC_RISCV_ADD64;
	      fixP->fx_next->fx_r_type = BFD_RELOC_RISCV_SUB64;
	      break;

	    case BFD_RELOC_32:
	      fixP->fx_r_type = BFD_RELOC_RISCV_ADD32;
	      fixP->fx_next->fx_r_type = BFD_RELOC_RISCV_SUB32;
	      break;

	    case BFD_RELOC_16:
	      fixP->fx_r_type = BFD_RELOC_RISCV_ADD16;
	      fixP->fx_next->fx_r_type = BFD_RELOC_RISCV_SUB16;
	      break;

	    case BFD_RELOC_8:
	      fixP->fx_r_type = BFD_RELOC_RISCV_ADD8;
	      fixP->fx_next->fx_r_type = BFD_RELOC_RISCV_SUB8;
	      break;

	    case BFD_RELOC_RISCV_CFA:
	      /* Load the byte to get the subtype.  */
	      subtype = bfd_get_8 (NULL, &((fragS *) (fixP->fx_frag->fr_opcode))->fr_literal[fixP->fx_where]);
	      loc = fixP->fx_frag->fr_fix - (subtype & 7);
	      switch (subtype)
		{
		case DW_CFA_advance_loc1:
		  fixP->fx_where = loc + 1;
		  fixP->fx_next->fx_where = loc + 1;
		  fixP->fx_r_type = BFD_RELOC_RISCV_SET8;
		  fixP->fx_next->fx_r_type = BFD_RELOC_RISCV_SUB8;
		  break;

		case DW_CFA_advance_loc2:
		  fixP->fx_size = 2;
		  fixP->fx_next->fx_size = 2;
		  fixP->fx_where = loc + 1;
		  fixP->fx_next->fx_where = loc + 1;
		  fixP->fx_r_type = BFD_RELOC_RISCV_SET16;
		  fixP->fx_next->fx_r_type = BFD_RELOC_RISCV_SUB16;
		  break;

		case DW_CFA_advance_loc4:
		  fixP->fx_size = 4;
		  fixP->fx_next->fx_size = 4;
		  fixP->fx_where = loc;
		  fixP->fx_next->fx_where = loc;
		  fixP->fx_r_type = BFD_RELOC_RISCV_SET32;
		  fixP->fx_next->fx_r_type = BFD_RELOC_RISCV_SUB32;
		  break;

		default:
		  if (subtype < 0x80 && (subtype & 0x40))
		    {
		      /* DW_CFA_advance_loc */
		      fixP->fx_frag = (fragS *) fixP->fx_frag->fr_opcode;
		      fixP->fx_next->fx_frag = fixP->fx_frag;
		      fixP->fx_r_type = BFD_RELOC_RISCV_SET6;
		      fixP->fx_next->fx_r_type = BFD_RELOC_RISCV_SUB6;
		    }
		  else
		    as_fatal (_("internal error: bad CFA value #%d"), subtype);
		  break;
		}
	      break;

	    default:
	      /* This case is unreachable.  */
	      abort ();
	    }
	}
      /* Fall through.  */

    case BFD_RELOC_RVA:
      /* If we are deleting this reloc entry, we must fill in the
	 value now.  This can happen if we have a .word which is not
	 resolved when it appears but is later defined.  */
      if (fixP->fx_addsy == NULL)
	{
	  gas_assert (fixP->fx_size <= sizeof (valueT));
	  md_number_to_chars ((char *) buf, *valP, fixP->fx_size);
	  fixP->fx_done = 1;
	}
      break;

    case BFD_RELOC_RISCV_JMP:
      if (fixP->fx_addsy)
	{
	  /* Fill in a tentative value to improve objdump readability.  */
	  bfd_vma target = S_GET_VALUE (fixP->fx_addsy) + *valP;
	  bfd_vma delta = target - md_pcrel_from (fixP);
	  bfd_putl32 (bfd_getl32 (buf) | ENCODE_UJTYPE_IMM (delta), buf);
	}
      break;

    case BFD_RELOC_12_PCREL:
      if (fixP->fx_addsy)
	{
	  /* Fill in a tentative value to improve objdump readability.  */
	  bfd_vma target = S_GET_VALUE (fixP->fx_addsy) + *valP;
	  bfd_vma delta = target - md_pcrel_from (fixP);
	  bfd_putl32 (bfd_getl32 (buf) | ENCODE_SBTYPE_IMM (delta), buf);
	}
      break;

    case BFD_RELOC_RISCV_RVC_BRANCH:
      if (fixP->fx_addsy)
	{
	  /* Fill in a tentative value to improve objdump readability.  */
	  bfd_vma target = S_GET_VALUE (fixP->fx_addsy) + *valP;
	  bfd_vma delta = target - md_pcrel_from (fixP);
	  bfd_putl16 (bfd_getl16 (buf) | ENCODE_RVC_B_IMM (delta), buf);
	}
      break;

    case BFD_RELOC_RISCV_RVC_JUMP:
      if (fixP->fx_addsy)
	{
	  /* Fill in a tentative value to improve objdump readability.  */
	  bfd_vma target = S_GET_VALUE (fixP->fx_addsy) + *valP;
	  bfd_vma delta = target - md_pcrel_from (fixP);
	  bfd_putl16 (bfd_getl16 (buf) | ENCODE_RVC_J_IMM (delta), buf);
	}
      break;

    case BFD_RELOC_RISCV_CALL:
    case BFD_RELOC_RISCV_CALL_PLT:
    case BFD_RELOC_RISCV_CALL_ICT:
      relaxable = TRUE;
      break;

    case BFD_RELOC_RISCV_PCREL_HI20:
    case BFD_RELOC_RISCV_PCREL_LO12_S:
    case BFD_RELOC_RISCV_PCREL_LO12_I:
    case BFD_RELOC_RISCV_PCREL_ICT_HI20:
      relaxable = riscv_opts.relax;
      break;

    case BFD_RELOC_RISCV_10_PCREL:
      if (fixP->fx_addsy)
	{
	  /* Fill in a tentative value to improve objdump readability.  */
	  bfd_vma target = S_GET_VALUE (fixP->fx_addsy) + *valP;
	  bfd_vma delta = target - md_pcrel_from (fixP);
	  bfd_putl32 (bfd_getl32 (buf) | ENCODE_STYPE_IMM10 (delta), buf);
	}
      break;

    case BFD_RELOC_RISCV_ALIGN:
      if (fixP->fx_frag->fr_var >= 2)
	fixP->fx_addnumber += 2;
      break;
    case BFD_RELOC_RISCV_ALIGN_BTB:
    case BFD_RELOC_RISCV_DATA:
    case BFD_RELOC_RISCV_ICT_64:
    case BFD_RELOC_RISCV_LGP18S0:
    case BFD_RELOC_RISCV_LGP17S1:
    case BFD_RELOC_RISCV_LGP17S2:
    case BFD_RELOC_RISCV_LGP17S3:
    case BFD_RELOC_RISCV_SGP18S0:
    case BFD_RELOC_RISCV_SGP17S1:
    case BFD_RELOC_RISCV_SGP17S2:
    case BFD_RELOC_RISCV_SGP17S3:
    case BFD_RELOC_RISCV_RELAX_REGION_BEGIN:
    case BFD_RELOC_RISCV_RELAX_REGION_END:
    case BFD_RELOC_RISCV_NO_RVC_REGION_BEGIN:
    case BFD_RELOC_RISCV_NO_RVC_REGION_END:
    case BFD_RELOC_RISCV_RELAX_ENTRY:
      break;

    default:
      /* We ignore generic BFD relocations we don't know about.  */
      if (bfd_reloc_type_lookup (stdoutput, fixP->fx_r_type) != NULL)
	as_fatal (_("internal error: bad relocation #%d"), fixP->fx_r_type);
    }

  if (fixP->fx_subsy != NULL)
    as_bad_where (fixP->fx_file, fixP->fx_line,
		  _("unsupported symbol subtraction"));

  /* Add an R_RISCV_RELAX reloc if the reloc is relaxable.  */
  if (relaxable && fixP->fx_tcbit && fixP->fx_addsy != NULL)
    {
      fixP->fx_next = xmemdup (fixP, sizeof (*fixP), sizeof (*fixP));
      fixP->fx_next->fx_addsy = fixP->fx_next->fx_subsy = NULL;
      fixP->fx_next->fx_r_type = BFD_RELOC_RISCV_RELAX;
      /* Clean up the ict flag for R_RISCV_RELAX.  */
      fixP->fx_next->tc_fix_data.ict = 0;
    }
}

/* Because the value of .cfi_remember_state may changed after relaxation,
   we insert a fix to relocate it again in link-time.  */

void
riscv_pre_output_hook (void)
{
  const frchainS *frch;
  const asection *s;

  for (s = stdoutput->sections; s; s = s->next)
    for (frch = seg_info (s)->frchainP; frch; frch = frch->frch_next)
      {
	fragS *frag;

	for (frag = frch->frch_root; frag; frag = frag->fr_next)
	  {
	    if (frag->fr_type == rs_cfa)
	      {
		expressionS exp;
		expressionS *symval;

		symval = symbol_get_value_expression (frag->fr_symbol);
		exp.X_op = O_subtract;
		exp.X_add_symbol = symval->X_add_symbol;
		exp.X_add_number = 0;
		exp.X_op_symbol = symval->X_op_symbol;

		fix_new_exp (frag, (int) frag->fr_offset, 1, &exp, 0,
			     BFD_RELOC_RISCV_CFA);
	      }
	  }
      }
}

static void
riscv_rvc_reloc_setting (int mode)
{
  /* We skip the rvc/norvc options which are set before the first
     instruction.  It is no necessary to insert the NO_RVC_REGION relocs
     according to these options since the first rvc information is
     stored in the fragment's tc_frag_data.rvc.  */
  /* RVC is disable entirely when -mno-16-bit is enabled  */
  if (!start_assemble || riscv_opts.no_16_bit)
    return;

  if (mode)
    /* RVC.  */
    fix_new (frag_now, frag_now_fix (), 0, abs_section_sym,
	     0x1, 0, BFD_RELOC_RISCV_NO_RVC_REGION_BEGIN);
  else
    /* No RVC.  */
    fix_new (frag_now, frag_now_fix (), 0, abs_section_sym,
	     0x0, 0, BFD_RELOC_RISCV_NO_RVC_REGION_BEGIN);
}

/* This structure is used to hold a stack of .option values.  */

struct riscv_option_stack
{
  struct riscv_option_stack *next;
  struct riscv_set_options options;
};

static struct riscv_option_stack *riscv_opts_stack;

/* Handle the .option pseudo-op.  */

static void
s_riscv_option (int x ATTRIBUTE_UNUSED)
{
  char *name = input_line_pointer, ch;

  while (!is_end_of_line[(unsigned char) *input_line_pointer])
    ++input_line_pointer;
  ch = *input_line_pointer;
  *input_line_pointer = '\0';

  if (strcmp (name, "rvc") == 0)
    {
      riscv_set_rvc (TRUE);
      riscv_rvc_reloc_setting (1);
    }
  else if (strcmp (name, "norvc") == 0)
    {
      /* Force to set the 4-byte aligned when converting
	 rvc to norvc.  The repeated alignment setting is
	 fine since linker will remove the redundant nops.  */
      if (riscv_opts.rvc && !riscv_opts.no_16_bit && start_assemble)
	riscv_frag_align_code (2);
      riscv_set_rvc (FALSE);
      riscv_rvc_reloc_setting (0);
    }
  else if (strcmp (name, "pic") == 0)
    riscv_opts.pic = TRUE;
  else if (strcmp (name, "nopic") == 0)
    riscv_opts.pic = FALSE;
  else if (strcmp (name, "relax") == 0)
    riscv_opts.relax = TRUE;
  else if (strcmp (name, "norelax") == 0)
    riscv_opts.relax = FALSE;
  else if (strcmp (name, "checkconstraints") == 0)
    riscv_opts.check_constraints = TRUE;
  else if (strcmp (name, "nocheckconstraints") == 0)
    riscv_opts.check_constraints = FALSE;
  else if (strcmp (name, "csr-check") == 0)
    riscv_opts.csr_check = TRUE;
  else if (strcmp (name, "no-csr-check") == 0)
    riscv_opts.csr_check = FALSE;
  else if (strcmp (name, "push") == 0)
    {
      struct riscv_option_stack *s;

      s = (struct riscv_option_stack *) xmalloc (sizeof *s);
      s->next = riscv_opts_stack;
      s->options = riscv_opts;
      riscv_opts_stack = s;
    }
  else if (strcmp (name, "pop") == 0)
    {
      struct riscv_option_stack *s;
      int pre_rvc;

      s = riscv_opts_stack;
      pre_rvc = riscv_opts.rvc;
      if (s == NULL)
	as_bad (_(".option pop with no .option push"));
      else
	{
	  riscv_opts = s->options;
	  riscv_opts_stack = s->next;
	  free (s);
	}

      /* Deal with the rvc setting.  */
      if (riscv_opts.rvc && !pre_rvc)
	/* norvc to rvc.  */
	riscv_rvc_reloc_setting (1);
      else if (!riscv_opts.rvc && pre_rvc)
	{
	  /* rvc to norvc.  */
	  if (!riscv_opts.no_16_bit)
	    {
	      riscv_opts.rvc = 1;
	      riscv_frag_align_code (2);
	    }
	  riscv_opts.rvc = 0;
	  riscv_rvc_reloc_setting (0);
	}
    }
  else if (strcmp (name, "execit") == 0
	   || strcmp (name, "ex9") == 0)
    riscv_opts.execit = TRUE;
  else if (strcmp (name, "verbatim") == 0)
    riscv_opts.verbatim = TRUE;
  else if (strncmp (name, "cmodel_[large|medany]", 6) == 0)
    {
      if (strcmp (name+6, "_large") == 0 && xlen > 32)
	riscv_opts.cmodel = CMODEL_LARGE;
      else if (strcmp (name+6, "_medany") == 0)
	riscv_opts.cmodel = CMODEL_DEFAULT;
      else if (strcmp (name+6, "_medlow") == 0)
	riscv_opts.cmodel = CMODEL_DEFAULT;
#ifdef DEBUG_CMODEL
      printf("%s: cmodel = %d\n", __func__, riscv_opts.cmodel);
#endif
    }
  else
    {
      as_warn (_("Unrecognized .option directive: %s\n"), name);
    }
  *input_line_pointer = ch;
  demand_empty_rest_of_line ();
}

/* Handle the .dtprelword and .dtpreldword pseudo-ops.  They generate
   a 32-bit or 64-bit DTP-relative relocation (BYTES says which) for
   use in DWARF debug information.  */

static void
s_dtprel (int bytes)
{
  expressionS ex;
  char *p;

  expression (&ex);

  if (ex.X_op != O_symbol)
    {
      as_bad (_("Unsupported use of %s"), (bytes == 8
					   ? ".dtpreldword"
					   : ".dtprelword"));
      ignore_rest_of_line ();
    }

  p = frag_more (bytes);
  md_number_to_chars (p, 0, bytes);
  fix_new_exp (frag_now, p - frag_now->fr_literal, bytes, &ex, FALSE,
	       (bytes == 8
		? BFD_RELOC_RISCV_TLS_DTPREL64
		: BFD_RELOC_RISCV_TLS_DTPREL32));

  demand_empty_rest_of_line ();
}

/* Handle the .bss pseudo-op.  */

static void
s_bss (int ignore ATTRIBUTE_UNUSED)
{
  subseg_set (bss_section, 0);
  demand_empty_rest_of_line ();
}

/* Called from md_do_align.  Used to create an alignment frag in a
   code section by emitting a worst-case NOP sequence that the linker
   will later relax to the correct number of NOPs.  We can't compute
   the correct alignment now because of other linker relaxations.  */

bfd_boolean
riscv_frag_align_code (int n)
{
  /* TODO: Review this.  */
  expressionS exp;
  bfd_vma alignment_power = riscv_opts.rvc ? 1 : 2;
  bfd_vma bytes = (bfd_vma) 1 << n;
  bfd_vma insn_alignment = riscv_opts.rvc ? 2 : 4;
  bfd_vma worst_case_bytes = bytes - insn_alignment;
  fragS* fragP = frag_now;
  char *p;

  /* Set the address at the optimizable begining.  */
  unsigned fragP_fix = (frag_now_fix() + 1) >> 1 << 1;

  /* When not relaxing, riscv_handle_align handles code alignment.  */
  if (!riscv_opts.relax)
    return FALSE;

  /* If we are moving to a smaller alignment than the instruction size,
     riscv_handle_align handles code alignment.  */
  if (bytes <= insn_alignment)
    return FALSE;

  /* Make sure the current alignment is align to insntruction alignment.  */
  frag_align_code (alignment_power, 0);

  /* Insert a ALIGN relocation for linker to remove the redandunt nops.
     Locate the relocation in the rs_align_code frag instead of frag_now,
     because we want linker to know the whole size of the alignment.  */
  exp.X_op = O_constant;
  /* Just set the worst value temporarily.  */
  exp.X_add_number = worst_case_bytes;
  exp.X_md = 0;
  fix_new_exp (fragP, fragP_fix, 0, &exp, 0, BFD_RELOC_RISCV_ALIGN);
  frag_grow (worst_case_bytes);
  p = frag_more (worst_case_bytes);
  /* zero contents for Andes bug20178.  */
  md_number_to_chars (p, 0, worst_case_bytes);

  return TRUE;
}

/* Implement HANDLE_ALIGN.  */

void
riscv_handle_align (fragS *fragP)
{
  bfd_signed_vma bytes ;

  if (fragP->fr_type != rs_align_code)
    return;

  bytes = fragP->fr_next->fr_address - fragP->fr_address - fragP->fr_fix;
  /* We have 4 byte uncompressed nops.  */
  bfd_signed_vma size = 4;
  bfd_signed_vma excess = bytes % size;
  char *p = fragP->fr_literal + fragP->fr_fix;

  if (bytes <= 0)
    return;

  /* Insert zeros or compressed nops to get 4 byte alignment.  */
  if (excess)
    {
      riscv_make_nops (p, excess);
      fragP->fr_fix += excess;
      p += excess;
    }

  if (bytes >= size)
    {
      /* After this function, the frag will be set to fr_fill.  We only
	 insert one 4 byte nop here.  The reset space will be filled in
	 write_contents.  */
      riscv_make_nops (p, size);
      fragP->fr_var = size;
    }
}

int
md_estimate_size_before_relax (fragS *fragp, asection *segtype)
{
  if (RELAX_BRANCH_P (fragp->fr_subtype))
    fragp->fr_var = relaxed_branch_length (fragp, segtype, FALSE);
  else if (RELAX_CMODEL_P (fragp->fr_subtype))
    fragp->fr_var = relaxed_cmodel_length (fragp, segtype);
  else
    gas_assert (0);

  return fragp->fr_var;
}

/* Translate internal representation of relocation info to BFD target
   format.  */

arelent *
tc_gen_reloc (asection *section ATTRIBUTE_UNUSED, fixS *fixp)
{
  arelent *reloc = (arelent *) xmalloc (sizeof (arelent));

  reloc->sym_ptr_ptr = (asymbol **) xmalloc (sizeof (asymbol *));
  *reloc->sym_ptr_ptr = symbol_get_bfdsym (fixp->fx_addsy);
  reloc->address = fixp->fx_frag->fr_address + fixp->fx_where;
  reloc->addend = fixp->fx_addnumber;

  reloc->howto = bfd_reloc_type_lookup (stdoutput, fixp->fx_r_type);
  if (reloc->howto == NULL)
    {
      if ((fixp->fx_r_type == BFD_RELOC_16 || fixp->fx_r_type == BFD_RELOC_8)
	  && fixp->fx_addsy != NULL && fixp->fx_subsy != NULL)
	{
	  /* We don't have R_RISCV_8/16, but for this special case,
	     we can use R_RISCV_ADD8/16 with R_RISCV_SUB8/16.  */
	  return reloc;
	}

      as_bad_where (fixp->fx_file, fixp->fx_line,
		    _("cannot represent %s relocation in object file"),
		    bfd_get_reloc_code_name (fixp->fx_r_type));
      return NULL;
    }

  switch (fixp->fx_r_type)
    {
    default:
      /* In general, addend of a relocation is the offset to the
	 associated symbol.  */
      reloc->addend = fixp->fx_addnumber;
      break;

    case BFD_RELOC_RISCV_DATA:
      /* Prevent linker from optimizing data in text sections.
	 For example, jump table.  */
      reloc->addend = fixp->fx_size;
      break;
    }

  return reloc;
}

int
riscv_relax_frag (asection *sec, fragS *fragp, long stretch ATTRIBUTE_UNUSED)
{
  if (RELAX_BRANCH_P (fragp->fr_subtype))
    {
      offsetT old_var = fragp->fr_var;
      fragp->fr_var = relaxed_branch_length (fragp, sec, TRUE);
      return fragp->fr_var - old_var;
    }

  return 0;
}

/* Expand far branches to multi-instruction sequences.  */

static void
md_convert_frag_branch (fragS *fragp, segT sec)
{
  bfd_byte *buf;
  expressionS exp;
  fixS *fixp;
  insn_t insn;
  int rs1, reloc;

  buf = (bfd_byte *)fragp->fr_literal + fragp->fr_fix;

  exp.X_op = O_symbol;
  exp.X_add_symbol = fragp->fr_symbol;
  exp.X_add_number = fragp->fr_offset;
  exp.X_md = 0;

  gas_assert (fragp->fr_var == RELAX_BRANCH_LENGTH (fragp->fr_subtype));

  /* We have to keep the relocation for the branch over jump
     since linker optimizations, inlcuding target aligned and EXECIT,
     may change the immediate field of the branch.  */
  if (RELAX_BRANCH_RVC (fragp->fr_subtype))
    {
      switch (RELAX_BRANCH_LENGTH (fragp->fr_subtype))
	{
	  case 8:
	  case 4:
	    /* Expand the RVC branch into a RISC-V one.  */
	    insn = bfd_getl16 (buf);
	    rs1 = 8 + ((insn >> OP_SH_CRS1S) & OP_MASK_CRS1S);
	    if ((insn & MASK_C_J) == MATCH_C_J)
	      insn = MATCH_JAL;
	    else if ((insn & MASK_C_JAL) == MATCH_C_JAL)
	      insn = MATCH_JAL | (X_RA << OP_SH_RD);
	    else if ((insn & MASK_C_BEQZ) == MATCH_C_BEQZ)
	      insn = MATCH_BEQ | (rs1 << OP_SH_RS1);
	    else if ((insn & MASK_C_BNEZ) == MATCH_C_BNEZ)
	      insn = MATCH_BNE | (rs1 << OP_SH_RS1);
	    else
	      abort ();
	    bfd_putl32 (insn, buf);
	    break;

	  case 6:
	    /* Invert the branch condition.  Branch over the jump.  */
	    insn = bfd_getl16 (buf);
	    insn ^= MATCH_C_BEQZ ^ MATCH_C_BNEZ;
	    insn |= ENCODE_RVC_B_IMM (6);
	    bfd_putl16 (insn, buf);
	    /* Keep the relocation for the RVC branch.  */
	    exp.X_add_symbol = symbol_temp_new (sec, 0, fragp->fr_next);
	    exp.X_add_number = 0;
	    reloc = BFD_RELOC_RISCV_RVC_BRANCH;
	    fixp = fix_new_exp (fragp, buf - (bfd_byte *)fragp->fr_literal,
				2, &exp, FALSE, reloc);
	    buf += 2;
	    goto jump;

	  case 2:
	    /* Just keep the RVC branch.  */
	    reloc = RELAX_BRANCH_UNCOND (fragp->fr_subtype)
		    ? BFD_RELOC_RISCV_RVC_JUMP : BFD_RELOC_RISCV_RVC_BRANCH;
	    fixp = fix_new_exp (fragp, buf - (bfd_byte *)fragp->fr_literal,
				2, &exp, FALSE, reloc);
	    buf += 2;
	    goto done;

	  default:
	    abort ();
	}
    }

  switch (RELAX_BRANCH_LENGTH (fragp->fr_subtype))
    {
    case 8:
      gas_assert (!RELAX_BRANCH_UNCOND (fragp->fr_subtype));

      /* Invert the branch condition.  Branch over the jump.  */
      insn = bfd_getl32 (buf);
      if (((insn & MASK_BEQC) == MATCH_BEQC)
	  || ((insn & MASK_BNEC) == MATCH_BNEC))
	{
	  insn ^= MATCH_BEQC ^ MATCH_BNEC;
	  insn |= ENCODE_STYPE_IMM10 (8);
	  reloc = BFD_RELOC_RISCV_10_PCREL;
	}
      else if (((insn & MASK_BBC) == MATCH_BBC)
	       || ((insn & MASK_BBS) == MATCH_BBS))
	{
	  insn ^= MATCH_BBC ^ MATCH_BBS;
	  insn |= ENCODE_STYPE_IMM10 (8);
	  reloc = BFD_RELOC_RISCV_10_PCREL;
	}
      else
	{
	  insn ^= MATCH_BEQ ^ MATCH_BNE;
	  insn |= ENCODE_SBTYPE_IMM (8);
	  reloc = BFD_RELOC_12_PCREL;
	}
      md_number_to_chars ((char *) buf, insn, 4);
      /* Keep the relocation for the branch.  */
      exp.X_add_symbol = symbol_temp_new (sec, 0, fragp->fr_next);
      exp.X_add_number = 0;
      fixp = fix_new_exp (fragp, buf - (bfd_byte *)fragp->fr_literal,
			  4, &exp, FALSE, reloc);
      buf += 4;

jump:
      /* Jump to the target.  */
      exp.X_add_symbol = fragp->fr_symbol;
      exp.X_add_number = fragp->fr_offset;
      fixp = fix_new_exp (fragp, buf - (bfd_byte *)fragp->fr_literal,
			  4, &exp, FALSE, BFD_RELOC_RISCV_JMP);
      md_number_to_chars ((char *) buf, MATCH_JAL, 4);
      buf += 4;
      break;

    case 4:
      switch (RELAX_BRANCH_RANGE (fragp->fr_subtype))
	{
	case RANGE_JMP:
	  reloc = BFD_RELOC_RISCV_JMP;
	  break;
	case RANGE_BRANCH:
	    reloc = BFD_RELOC_12_PCREL;
	    break;
	case RANGE_10_PCREL:
	    reloc = BFD_RELOC_RISCV_10_PCREL;
	    break;
	default:
	    reloc = RELAX_BRANCH_UNCOND (fragp->fr_subtype)
	      ? BFD_RELOC_RISCV_JMP : BFD_RELOC_12_PCREL;
	    break;
	}
      fixp = fix_new_exp (fragp, buf - (bfd_byte *)fragp->fr_literal,
			  4, &exp, FALSE, reloc);
      buf += 4;
      break;

    default:
      abort ();
    }

done:
  fixp->fx_file = fragp->fr_file;
  fixp->fx_line = fragp->fr_line;

  gas_assert (buf == (bfd_byte *)fragp->fr_literal
	      + fragp->fr_fix + fragp->fr_var);

  fragp->fr_fix += fragp->fr_var;
}

static void
md_convert_frag_cmodel (fragS *fragp, segT sec)
{
  static expressionS exp_ind, exp_ref;
  bfd_byte *buf;
  expressionS exp;
  fixS *fixp = NULL;
  int reloc;
  int type = RELAX_CMODEL_TYPE (fragp->fr_subtype);
  int length = RELAX_CMODEL_LENGTH (fragp->fr_subtype);
  int index = RELAX_CMODEL_INDEX (fragp->fr_subtype);
  int is_same_sec = is_same_section_symbol (fragp->fr_symbol, sec);

  gas_assert (fragp->fr_var == RELAX_CMODEL_LENGTH (fragp->fr_subtype));

  buf = (bfd_byte *)fragp->fr_literal + fragp->fr_fix;

  exp.X_op = O_symbol;
  exp.X_add_symbol = fragp->fr_symbol;
  exp.X_add_number = fragp->fr_offset;

  switch (index)
  {
  case CSI_INDIRECT_SYMBOL:
    exp_ind = exp;
    switch (type)
    {
    case TYPE_JX ... TYPE_ST:
      break;
    case TYPE_IS:
      if (is_same_sec)
	gas_assert (length == 0);
      else
	{
	  gas_assert (length == 8);
	  reloc = BFD_RELOC_64;
	  fix_new_exp (fragp, buf - (bfd_byte *)fragp->fr_literal,
		       CMODEL_SECTION_ENTRY_SIZE, &exp, FALSE, reloc);
	}
      break;
    default:
      as_fatal (_("internal error: invalid CModel type!"));
    }
    break;
  case CSI_REFERENCE_SYMBOL:
    exp_ref = exp;
    break;
  case CSI_LARGE_CODE:
    reloc = 0;
    switch (type)
    {
    case TYPE_JX:
      if (is_same_sec)
        gas_assert (length == 0);
      else
	{
	  reloc = BFD_RELOC_RISCV_PCREL_HI20;
	  fix_new_exp (fragp, buf - (bfd_byte *)fragp->fr_literal,
		       4, &exp_ind, FALSE, reloc);
	  reloc = BFD_RELOC_RISCV_RELAX;
	  fix_new (fragp, buf - (bfd_byte *)fragp->fr_literal,
		   0, abs_section_sym, 0, FALSE, reloc);

	  reloc = BFD_RELOC_RISCV_PCREL_LO12_I;
	  fix_new_exp (fragp, buf - (bfd_byte *)fragp->fr_literal + 4,
		       4, &exp_ref, FALSE, reloc);
	  reloc = BFD_RELOC_RISCV_RELAX;
	  fix_new (fragp, buf - (bfd_byte *)fragp->fr_literal +4,
		   0, abs_section_sym, 0, FALSE, reloc);

	  /* TODO: relax jalr to c.jalr  */
	}
      break;
    case TYPE_LA:
      gas_assert (length == 8);
      if (is_same_sec)
	{
	  int32_t *bin = (int32_t *) buf;
	  bin[1] = bin[2];

	  reloc = BFD_RELOC_RISCV_PCREL_HI20;
	  fix_new_exp (fragp, buf - (bfd_byte *)fragp->fr_literal,
		       4, &exp, FALSE, reloc);
	  reloc = BFD_RELOC_RISCV_RELAX;
	  fix_new (fragp, buf - (bfd_byte *)fragp->fr_literal,
		   0, abs_section_sym, 0, FALSE, reloc);

	  reloc = BFD_RELOC_RISCV_PCREL_LO12_I;
	  fix_new_exp (fragp, buf - (bfd_byte *)fragp->fr_literal + 4,
		       4, &exp_ref, FALSE, reloc);
	  reloc = BFD_RELOC_RISCV_RELAX;
	  fix_new (fragp, buf - (bfd_byte *)fragp->fr_literal +4,
		   0, abs_section_sym, 0, FALSE, reloc);
	}
      else
	{
	  reloc = BFD_RELOC_RISCV_PCREL_HI20;
	  fix_new_exp (fragp, buf - (bfd_byte *)fragp->fr_literal,
		       4, &exp_ind, FALSE, reloc);
	  reloc = BFD_RELOC_RISCV_RELAX;
	  fix_new (fragp, buf - (bfd_byte *)fragp->fr_literal,
		   0, abs_section_sym, 0, FALSE, reloc);

	  reloc = BFD_RELOC_RISCV_PCREL_LO12_I;
	  fix_new_exp (fragp, buf - (bfd_byte *)fragp->fr_literal + 4,
		       4, &exp_ref, FALSE, reloc);
	  reloc = BFD_RELOC_RISCV_RELAX;
	  fix_new (fragp, buf - (bfd_byte *)fragp->fr_literal +4,
		   0, abs_section_sym, 0, FALSE, reloc);
	}
      break;
    case TYPE_LD:
      if (is_same_sec)
	{
	  gas_assert (length == 8);
	  int32_t *bin = (int32_t *) buf;
	  bin[1] = bin[2];

	  reloc = BFD_RELOC_RISCV_PCREL_HI20;
	  fix_new_exp (fragp, buf - (bfd_byte *)fragp->fr_literal,
		       4, &exp, FALSE, reloc);
	  reloc = BFD_RELOC_RISCV_RELAX;
	  fix_new (fragp, buf - (bfd_byte *)fragp->fr_literal,
		   0, abs_section_sym, 0, FALSE, reloc);

	  reloc = BFD_RELOC_RISCV_PCREL_LO12_I;
	  fix_new_exp (fragp, buf - (bfd_byte *)fragp->fr_literal + 4,
		       4, &exp_ref, FALSE, reloc);
	  reloc = BFD_RELOC_RISCV_RELAX;
	  fix_new (fragp, buf - (bfd_byte *)fragp->fr_literal +4,
		   0, abs_section_sym, 0, FALSE, reloc);
	}
      else
	{
	  gas_assert (length == 12);
	  reloc = BFD_RELOC_RISCV_PCREL_HI20;
	  fix_new_exp (fragp, buf - (bfd_byte *)fragp->fr_literal,
		       4, &exp_ind, FALSE, reloc);
	  reloc = BFD_RELOC_RISCV_RELAX;
	  fix_new (fragp, buf - (bfd_byte *)fragp->fr_literal,
		   0, abs_section_sym, 0, FALSE, reloc);

	  reloc = BFD_RELOC_RISCV_PCREL_LO12_I;
	  fix_new_exp (fragp, buf - (bfd_byte *)fragp->fr_literal + 4,
		       4, &exp_ref, FALSE, reloc);
	  reloc = BFD_RELOC_RISCV_RELAX;
	  fix_new (fragp, buf - (bfd_byte *)fragp->fr_literal +4,
		   0, abs_section_sym, 0, FALSE, reloc);
	}
      break;
    case TYPE_ST:
      if (is_same_sec)
	{
	  gas_assert (length == 8);
	  int32_t *bin = (int32_t *) buf;
	  bin[1] = bin[2];

	  reloc = BFD_RELOC_RISCV_PCREL_HI20;
	  fix_new_exp (fragp, buf - (bfd_byte *)fragp->fr_literal,
		       4, &exp, FALSE, reloc);
	  reloc = BFD_RELOC_RISCV_RELAX;
	  fix_new (fragp, buf - (bfd_byte *)fragp->fr_literal,
		   0, abs_section_sym, 0, FALSE, reloc);

	  reloc = BFD_RELOC_RISCV_PCREL_LO12_S;
	  fix_new_exp (fragp, buf - (bfd_byte *)fragp->fr_literal + 4,
		       4, &exp_ref, FALSE, reloc);
	  reloc = BFD_RELOC_RISCV_RELAX;
	  fix_new (fragp, buf - (bfd_byte *)fragp->fr_literal +4,
		   0, abs_section_sym, 0, FALSE, reloc);
	}
      else
	{
	  gas_assert (length == 12);
	  reloc = BFD_RELOC_RISCV_PCREL_HI20;
	  fix_new_exp (fragp, buf - (bfd_byte *)fragp->fr_literal,
		       4, &exp_ind, FALSE, reloc);
	  reloc = BFD_RELOC_RISCV_RELAX;
	  fix_new (fragp, buf - (bfd_byte *)fragp->fr_literal,
		   0, abs_section_sym, 0, FALSE, reloc);

	  reloc = BFD_RELOC_RISCV_PCREL_LO12_I;
	  fix_new_exp (fragp, buf - (bfd_byte *)fragp->fr_literal + 4,
		       4, &exp_ref, FALSE, reloc);
	  reloc = BFD_RELOC_RISCV_RELAX;
	  fix_new (fragp, buf - (bfd_byte *)fragp->fr_literal +4,
		   0, abs_section_sym, 0, FALSE, reloc);
	}
      break;
    default:
      as_fatal (_("internal error: invalid CModel type!"));
    }
    break;
  case CSI_DEFAULT_CODE:
    reloc = 0;
    switch (type)
    {
    case TYPE_JX:
      if (!is_same_sec)
        gas_assert (length == 0);
      else
	{
	  reloc = BFD_RELOC_RISCV_CALL;
	  fix_new_exp (fragp, buf - (bfd_byte *)fragp->fr_literal,
		       4, &exp, FALSE, reloc);
	  reloc = BFD_RELOC_RISCV_RELAX;
	  fix_new (fragp, buf - (bfd_byte *)fragp->fr_literal,
		   0, abs_section_sym, 0, FALSE, reloc);
	}
      break;
    default:
      as_fatal (_("internal error: invalid CModel type!"));
    }
    break;
  case CSI_B22827:
  case CSI_B22827_1: /* TODO: relax nop to c.nop if has RVC  */
    /* blank  */
    break;
  default:
    as_fatal (_("internal error: invalid CModel index!"));
  }

  buf += length;

  if (fixp)
    {
      fixp->fx_file = fragp->fr_file;
      fixp->fx_line = fragp->fr_line;
    }

  gas_assert (buf == (bfd_byte *)fragp->fr_literal
	      + fragp->fr_fix + fragp->fr_var);

  fragp->fr_fix += fragp->fr_var;
}

/* Relax a machine dependent frag.  This returns the amount by which
   the current size of the frag should change.  */

void
md_convert_frag (bfd *abfd ATTRIBUTE_UNUSED, segT asec, fragS *fragp)
{
  gas_assert (RELAX_BRANCH_P (fragp->fr_subtype)
	      || RELAX_CMODEL_P (fragp->fr_subtype));
  if (RELAX_BRANCH_P (fragp->fr_subtype))
    md_convert_frag_branch (fragp, asec);
  else if (RELAX_CMODEL_P (fragp->fr_subtype))
    md_convert_frag_cmodel (fragp, asec);
}

void
md_show_usage (FILE *stream)
{
  fprintf (stream, _("\
RISC-V options:\n\
  -fpic           generate position-independent code\n\
  -fPIC           same as -fpic\n\
  -fno-pic        don't generate position-independent code (default)\n\
  -march=ISA      set the RISC-V architecture\n\
  -mabi=ABI       set the RISC-V ABI\n\
  -mrelax         enable relax (default)\n\
  -mno-relax      disable relax\n\
  -march-attr     generate RISC-V arch attribute\n\
  -mno-arch-attr  don't generate RISC-V arch attribute\n\
\nNDS specific command line options:\n\
  -mno-16-bit     don't generate rvc instructions\n\
  -matomic        enable atomic extension\n\
  -mace           support user defined instruction extension\n\
  -O1             optimize for performance\n\
  -Os             optimize for space\n\
  -mext-dsp       enable dsp extension\n\
  -mext-efhw      enable efhw extension\n\
  -mext-vector    enable vector extension\n\
  -mexecit-noji   disable execit relaxation for jump instructions\n\
  -mexecit-nols   disable execit relaxation for load/store instructions\n\
  -mexecit-norel  disable execit relaxation for instructions with reloaction\n\
  -mcmodel=TYPE   set cmodel type\n\
  -mno-workaround disable all workarounds\n\
"));
}

/* Standard calling conventions leave the CFA at SP on entry.  */
void
riscv_cfi_frame_initial_instructions (void)
{
  cfi_add_CFA_def_cfa_register (X_SP);
}

int
tc_riscv_regname_to_dw2regnum (char *regname)
{
  int reg;

  if ((reg = reg_lookup_internal (regname, RCLASS_GPR)) >= 0)
    return reg;

  if ((reg = reg_lookup_internal (regname, RCLASS_FPR)) >= 0)
    return reg + 32;

  /* CSRs are numbered 4096 -> 8191.  */
  if ((reg = reg_lookup_internal (regname, RCLASS_CSR)) >= 0)
    return reg + 4096;

  as_bad (_("unknown register `%s'"), regname);
  return -1;
}

/* We don't allow the overlapped NO_RVC_REGION_BEGIN and NO_RVC_REGION_END.
   Therefore, we choose the last NO_RVC_REGION as the effective setting
   at the same address.  */

static void
riscv_find_next_effective_rvc_region (fixS **fixp)
{
  fixS *effective_fixp = NULL;

  while (*fixp && (*fixp)->fx_r_type != BFD_RELOC_RISCV_NO_RVC_REGION_BEGIN)
    *fixp = (*fixp)->fx_next;
  effective_fixp = *fixp;
  if (!effective_fixp)
    return;

  *fixp = (*fixp)->fx_next;
  while (*fixp
	 && (*fixp)->fx_frag == effective_fixp->fx_frag
	 && (*fixp)->fx_where == effective_fixp->fx_where)
    {
      if ((*fixp)->fx_r_type == BFD_RELOC_RISCV_NO_RVC_REGION_BEGIN)
	{
	  effective_fixp->fx_offset = 2;
	  effective_fixp->fx_done = 1;
	  effective_fixp = *fixp;
	}
      *fixp = (*fixp)->fx_next;
    }
  *fixp = effective_fixp;
}

/* The Addend of BFD_RELOC_RISCV_NO_RVC_REGION_BEGIN means,
   0: norvc, unchecked
   1: rvc, unchecked
   2: Already checked.  */

static void
riscv_final_no_rvc_region (bfd *abfd ATTRIBUTE_UNUSED, asection *sec,
			   void *xxx ATTRIBUTE_UNUSED)
{
  segment_info_type *seginfo;
  frchainS *frch;
  fixS *fixp, *pre_fixp_begin;
  int current_rvc = 0;

  seginfo = seg_info (sec);
  if (!seginfo || !symbol_rootP || !subseg_text_p (sec) || sec->size == 0)
    return;

  subseg_change (sec, 0);

  frch = seginfo->frchainP;
  pre_fixp_begin = NULL;
  current_rvc = frch->frch_root->tc_frag_data.rvc;
  if (current_rvc == -1)
    {
      fixp = fix_at_start (frch->frch_root, 0, abs_section_sym, 0x2,
			   0, BFD_RELOC_RISCV_NO_RVC_REGION_BEGIN);
      pre_fixp_begin = fixp;
      fixp = seginfo->fix_root->fx_next;
    }
  else
    fixp = seginfo->fix_root;

  /* Assume the offset of the NO_RVC_REGION are in order.  */
  for (; fixp; fixp = fixp->fx_next)
    {
      /* Find the next effective NO_RVC_REGION relocations.  */
      riscv_find_next_effective_rvc_region (&fixp);
      if (!fixp)
	break;

      /* Remove the redundant NO_RVC_REGION relocations.  */
      if (fixp->fx_offset == 0)
	{
	  /* norvc to norvc.  */
	  if (current_rvc == -1)
	    fixp->fx_done = 1;
	  else
	    {
	      /* rvc to norvc.  */
	      current_rvc = -1;
	      pre_fixp_begin = fixp;
	    }
	}
      else if (fixp->fx_offset == 1)
	{
	  /* Cannot find the corresponding NO_RVC_REGION_BEGIN
	     or rvc to rvc.  */
	  if (!pre_fixp_begin
	      || current_rvc == 1)
	    fixp->fx_done = 1;
	  else
	    {
	      /* norvc to rvc.  */
	      current_rvc = 1;
	      pre_fixp_begin = NULL;
	      fixp->fx_r_type = BFD_RELOC_RISCV_NO_RVC_REGION_END;
	    }
	}
      fixp->fx_offset = 2;
    }
}

/* final both .no_execit_[begin|end]
 *        and .innermost_loop_[begin|end]
 */
static void
riscv_final_no_execit_region (bfd *abfd ATTRIBUTE_UNUSED, asection *sec,
			      void *xxx ATTRIBUTE_UNUSED)
{
  segment_info_type *seginfo;
  fixS *fixp;
  int no_execit_count;
  int innermost_loop_count;

  seginfo = seg_info (sec);
  if (!seginfo || !symbol_rootP || !subseg_text_p (sec) || sec->size == 0)
    return;

  subseg_change (sec, 0);

  /* First, we also need to insert noexecit directives according to
     the NO_RVC_REGION directives set at riscv_final_no_rvc_region.  */
  fixp = seginfo->fix_root;
  for (; fixp; fixp = fixp->fx_next)
    {
      /* skip redundant fixes  */
      if (fixp->fx_done)
	continue;

      if (fixp->fx_r_type == BFD_RELOC_RISCV_NO_RVC_REGION_BEGIN)
	fix_new (fixp->fx_frag, fixp->fx_where, 0, abs_section_sym,
		 R_RISCV_RELAX_REGION_NO_EXECIT_FLAG, 0,
		 BFD_RELOC_RISCV_RELAX_REGION_BEGIN);
      else if (fixp->fx_r_type == BFD_RELOC_RISCV_NO_RVC_REGION_END)
	fix_new (fixp->fx_frag, fixp->fx_where, 0, abs_section_sym,
		 R_RISCV_RELAX_REGION_NO_EXECIT_FLAG, 0,
		 BFD_RELOC_RISCV_RELAX_REGION_END);
    }

  /* Assume the offset of the BFD_RELOC_RISCV_RELAX_REGION_BEGIN/END
     are in order.  */
  no_execit_count = 0;
  innermost_loop_count = 0;
  fixp = seginfo->fix_root;
  for (; fixp; fixp = fixp->fx_next)
    {
      if (fixp->fx_r_type == BFD_RELOC_RISCV_RELAX_REGION_BEGIN)
	{
	  if (fixp->fx_offset == R_RISCV_RELAX_REGION_NO_EXECIT_FLAG)
	    {
	      /* We must find the corresponding REGION_END later.  */
	      if (no_execit_count > 0)
		fixp->fx_done = 1;
	      no_execit_count++;
	    }
	  else if (fixp->fx_offset == R_RISCV_RELAX_REGION_INNERMOST_LOOP_FLAG)
	    {
	      /* eliminate nested ".innermost_loop_begin".  */
	      if (innermost_loop_count > 0)
		fixp->fx_done = 1;
	      innermost_loop_count++;
	    }
	}
      else if (fixp->fx_r_type == BFD_RELOC_RISCV_RELAX_REGION_END)
        {
	  if (fixp->fx_offset == R_RISCV_RELAX_REGION_NO_EXECIT_FLAG)
	    {
	      no_execit_count--;
	      if (no_execit_count > 0)
		fixp->fx_done = 1;
	      else if (no_execit_count < 0)
		{
		  /* Find the unmatched REGION_END, ignore it.  */
		  no_execit_count++;
		  fixp->fx_done = 1;
		}
	    }
	  else if (fixp->fx_offset == R_RISCV_RELAX_REGION_INNERMOST_LOOP_FLAG)
	    {
	      innermost_loop_count--;
	      /* eliminate nested ".innermost_loop_end".  */
	      if (innermost_loop_count > 0)
		fixp->fx_done = 1;
	      else if (innermost_loop_count < 0)
		{
		  /* unmatched ".innermost_loop_end", ignore it.  */
		  innermost_loop_count++;
		  fixp->fx_done = 1;
		}
	    }
	}
    }

  /* We have handle the negative no_execit_count cases above.  */
  if (no_execit_count > 0)
    {
      /* Find the unmatched REGION_BEGIN, we should print
	 warning/error msg here.  */
    }
}

static void
riscv_insert_relax_entry (bfd *abfd ATTRIBUTE_UNUSED, asection *sec,
			  void *xxx ATTRIBUTE_UNUSED)
{
  segment_info_type *seginfo;
  frchainS *frch;
  fixS *fixp;
  offsetT X_add_number;

  seginfo = seg_info (sec);
  if (!seginfo || !symbol_rootP || !subseg_text_p (sec) || sec->size == 0)
    return;

  subseg_change (sec, 0);

  /* The original code says that it is not necessary to insert the
     R_RISCV_RELAX_ENTRY when there is no relocation and the execit
     is disabled. It is weird for me now, I think always insert the
     R_RISCV_RELAX_ENTRY is fine and do no harm.  */

  /* Set RELAX_ENTRY flags for linker.  */
  frch = seginfo->frchainP;
  X_add_number = 0;

  if (!riscv_opts.relax)
    X_add_number |= R_RISCV_RELAX_ENTRY_DISABLE_RELAX_FLAG;
  if (riscv_opts.execit)
    X_add_number |= R_RISCV_RELAX_ENTRY_EXECIT_FLAG;

  fixp = fix_at_start (frch->frch_root, 0, abs_section_sym, X_add_number,
		       0, BFD_RELOC_RISCV_RELAX_ENTRY);
  fixp->fx_no_overflow = 1;
}

void
riscv_post_relax_hook (void)
{
  /* TODO: Maybe we can sort the relocations here to reduce the burden
     of linker.  */
  bfd_map_over_sections (stdoutput, riscv_final_no_rvc_region, NULL);
  bfd_map_over_sections (stdoutput, riscv_final_no_execit_region, NULL);
  bfd_map_over_sections (stdoutput, riscv_insert_relax_entry, NULL);
}

void
riscv_elf_final_processing (void)
{
  riscv_set_abi_by_arch ();
  elf_elfheader (stdoutput)->e_flags |= elf_flags;
}

static void
riscv_aligned_cons (int idx)
{
  /* Call default handler.  */
  cons (1 << idx);
  if (now_seg->flags & SEC_CODE
      && now_seg->flags & SEC_ALLOC && now_seg->flags & SEC_RELOC)
    {
      /* Use BFD_RELOC_RISCV_DATA to avoid EXECIT optimization replacing data.  */
      expressionS exp;

      exp.X_add_number = 0;
      exp.X_op = O_constant;
      fix_new_exp (frag_now, frag_now_fix () - (1 << idx), 1 << idx,
		   &exp, 0, BFD_RELOC_RISCV_DATA);
    }
}

/* Parse the .sleb128 and .uleb128 pseudos.  Only allow constant expressions,
   since these directives break relaxation when used with symbol deltas.  */

static void
s_riscv_leb128 (int sign)
{
  expressionS exp;
  char *save_in = input_line_pointer;

  expression (&exp);
  if (exp.X_op != O_constant)
    as_bad (_("non-constant .%cleb128 is not supported"), sign ? 's' : 'u');
  demand_empty_rest_of_line ();

  input_line_pointer = save_in;
  return s_leb128 (sign);
}

/* Parse the .insn directive.  */

static void
s_riscv_insn (int x ATTRIBUTE_UNUSED)
{
  char *str = input_line_pointer;
  struct riscv_cl_insn insn;
  expressionS imm_expr;
  bfd_reloc_code_real_type imm_reloc = BFD_RELOC_UNUSED;
  char save_c;

  insn.cmodel.method = METHOD_DEFAULT;
  imm_expr.X_md = 0;

  while (!is_end_of_line[(unsigned char) *input_line_pointer])
    ++input_line_pointer;

  save_c = *input_line_pointer;
  *input_line_pointer = '\0';

  const char *error = riscv_ip (str, &insn, &imm_expr,
				&imm_reloc, insn_type_hash);

  if (error)
    {
      as_bad ("%s `%s'", error, str);
    }
  else
    {
      gas_assert (insn.insn_mo->pinfo != INSN_MACRO);
      append_insn (&insn, &imm_expr, imm_reloc);
    }

  *input_line_pointer = save_c;
  demand_empty_rest_of_line ();
}

/* Update arch attributes.  */

static void
riscv_write_out_arch_attr (void)
{
  const char *arch_str = riscv_arch_str (xlen, &riscv_subsets);

  bfd_elf_add_proc_attr_string (stdoutput, Tag_RISCV_arch, arch_str);

  xfree ((void *)arch_str);
}

/* Add the default contents for the .riscv.attributes section.  */
static void
andes_riscv_set_public_attributes (void);

#if 0
static void
riscv_set_public_attributes (void)
{
  if (riscv_opts.arch_attr || explicit_arch_attr)
    /* Re-write arch attribute to normalize the arch string.  */
    riscv_write_out_arch_attr ();
}
#endif

/* Called after all assembly has been done.  */

void
riscv_md_end (void)
{
/*riscv_set_public_attributes (); */
  andes_riscv_set_public_attributes ();
}

/* Given a symbolic attribute NAME, return the proper integer value.
   Returns -1 if the attribute is not known.  */

int
riscv_convert_symbolic_attribute (const char *name)
{
  static const struct
  {
    const char * name;
    const int    tag;
  }
  attribute_table[] =
    {
      /* When you modify this table you should
	 also modify the list in doc/c-riscv.texi.  */
#define T(tag) {#tag, Tag_RISCV_##tag},  {"Tag_RISCV_" #tag, Tag_RISCV_##tag}
      T(arch),
      T(priv_spec),
      T(priv_spec_minor),
      T(priv_spec_revision),
      T(unaligned_access),
      T(stack_align),
      /* Andes backward compatible */
      T(strict_align),
      T(ict_version),
      T(ict_model),
#undef T
    };

  unsigned int i;

  if (name == NULL)
    return -1;

  for (i = 0; i < ARRAY_SIZE (attribute_table); i++)
    if (strcmp (name, attribute_table[i].name) == 0)
      return attribute_table[i].tag;

  return -1;
}

/* Parse a .attribute directive.  */
static void andes_pre_s_riscv_attribute (void);
static void andes_post_s_riscv_attribute (int tag);

static void
s_riscv_attribute (int ignored ATTRIBUTE_UNUSED)
{
  andes_pre_s_riscv_attribute();

  int tag = obj_elf_vendor_attribute (OBJ_ATTR_PROC);

  if (tag == Tag_RISCV_arch)
    {
      unsigned old_xlen = xlen;

      explicit_arch_attr = TRUE;
      obj_attribute *attr;
      attr = elf_known_obj_attributes_proc (stdoutput);
      if (!start_assemble)
	riscv_set_arch (attr[Tag_RISCV_arch].s);
      else
	as_fatal (_(".attribute arch must set before any instructions"));

      if (old_xlen != xlen)
	{
	  /* We must re-init bfd again if xlen is changed.  */
	  unsigned long mach = xlen == 64 ? bfd_mach_riscv64 : bfd_mach_riscv32;
	  bfd_find_target (riscv_target_format (), stdoutput);

	  if (! bfd_set_arch_mach (stdoutput, bfd_arch_riscv, mach))
	    as_warn (_("Could not set architecture and machine"));
	}
    }

  andes_post_s_riscv_attribute(tag);
}

/* Insert relocations to mark the region that can not do EXECIT relaxation.  */

static void
riscv_no_execit (int mode)
{
  expressionS exp;

  exp.X_op = O_symbol;
  exp.X_add_symbol = abs_section_sym;
  if (mode == 1)
    {
      /* The begin of the region without EXECIT.  */
      exp.X_add_number = R_RISCV_RELAX_REGION_NO_EXECIT_FLAG;
      fix_new_exp (frag_now, frag_now_fix (), 0, &exp, 0,
		   BFD_RELOC_RISCV_RELAX_REGION_BEGIN);
    }
  else
    {
      /* The end of the region without EXECIT.  */
      exp.X_add_number = R_RISCV_RELAX_REGION_NO_EXECIT_FLAG;
      fix_new_exp (frag_now, frag_now_fix (), 0, &exp, 0,
		   BFD_RELOC_RISCV_RELAX_REGION_END);
    }
}

/* Insert relocations to mark the innermost loop region.  */

static void
riscv_innermost_loop (int mode)
{
  /* Insert loop region relocation here.  */
  expressionS exp;

  exp.X_op = O_symbol;
  exp.X_add_symbol = abs_section_sym;
  if (mode == 1)
    {
      exp.X_add_number = R_RISCV_RELAX_REGION_INNERMOST_LOOP_FLAG;
      fix_new_exp (frag_now, frag_now_fix (), 0, &exp, 0,
		   BFD_RELOC_RISCV_RELAX_REGION_BEGIN);
    }
  else
    {
      exp.X_add_number = R_RISCV_RELAX_REGION_INNERMOST_LOOP_FLAG;
      fix_new_exp (frag_now, frag_now_fix (), 0, &exp, 0,
		   BFD_RELOC_RISCV_RELAX_REGION_END);
    }
}

#ifdef DEBUG_ARCH_INFO_HASH
static void
riscv_print_arch_info_hash (const char *key, void *value)
{
  struct arch_info *data = (struct arch_info *) value;
  printf ("(key, name, v_major, v_minor, valid): (%s, %s, %s, %s, %d)\n",
	  key, data->name, data->v_major, data->v_minor, data->valid);
}
#endif

#if 0
static void
riscv_count_arch_attr_strlen (const char *key ATTRIBUTE_UNUSED,
			      void *value)
{
  struct arch_info *data = (struct arch_info *) value;
  if (data->valid)
    {
      arch_attr_strlen += strlen (data->name)
	+ strlen (data->v_major)
	+ strlen (data->v_minor)
	+ 1; /* for 'p'  */
      if (*(data->name) == 'x')
	{
	  first_X_arch = 0;
	  arch_attr_strlen++; /* for '_'  */
	}
    }
}
#endif

/* Update standard arch attributes.  */

#if 0
static void
riscv_update_non_standard_arch_attr (const char *key ATTRIBUTE_UNUSED,
				     void *value)
{
  struct arch_info *data = (struct arch_info *) value;
  if (data->valid)
    {
      if (first_X_arch)
	first_X_arch = 0;
      else
	strncat (out_arch, "_", 1);

      strncat (out_arch, data->name, strlen (data->name));
      strncat (out_arch, data->v_major, strlen (data->v_major));
      strncat (out_arch, "p", 1);
      strncat (out_arch, data->v_minor, strlen (data->v_minor));
      data->valid = 0;
    }
}
#endif
#if 0
static void
andes_riscv_write_out_arch_attr (void)
{
  unsigned int i;
  obj_attribute *attr;
  bfd_boolean e_ext;

  /* If we can't find any attribute directive, update the arch hash
     table according to the `all_subsets' set by the `-march' option
     or `default_arch'.  */
  attr = elf_known_obj_attributes_proc (stdoutput);
  if (!attr[Tag_arch].s)
    {
      while (riscv_subsets != NULL)
	{
	  struct riscv_subset *next = riscv_subsets->next;
	  /* Since riscv_subsets does not save the version so far,
	     1. Standard arch version: default setting
	     2. Non standard arch version: zero.  */
	  for (i = 0; arch_info[i].name; i++)
	    if (strcmp (riscv_subsets->name, arch_info[i].name) == 0)
	      {
		if (strcmp (arch_info[i].name, "c") != 0)
		  riscv_update_arch_info_hash (riscv_subsets->name, -1, TRUE);
		break;
	      }
	  /* Can not find the matched standard arch, regard it as
	     non-standard one.  */
	  if (!arch_info[i].name)
	    riscv_update_arch_info_hash (riscv_subsets->name, -1, TRUE);
	  riscv_subsets = next;
	}
    }

  /* Check the `elf_flags' rather than `riscv_subsets' and `riscv_opts'
     since .option rvc/norvc may occur at any place (after parsing
     attribute and -march).  */
  if (elf_flags & EF_RISCV_RVC)
    riscv_update_arch_info_hash ("c", -1, TRUE);

  arch_attr_strlen = 0;
  hash_traverse (arch_info_hash, riscv_count_arch_attr_strlen);
  /* Arch attribute is not set up.  */
  if (arch_attr_strlen == 0)
    return;
  if (!first_X_arch)
    arch_attr_strlen--; /* first x do not need '_'.  */
  first_X_arch = 1;
  arch_attr_strlen += 4; /* rv32/rv64.  */

  out_arch = (char *) malloc
    ((arch_attr_strlen + 1) * sizeof (char));
  out_arch[0] = '\0';
  if (attr[Tag_arch].s)
    strncat (out_arch, attr[Tag_arch].s, 4);
  else if (xlen == 32)
    strncat (out_arch, "rv32", 4);
  else
    strncat (out_arch, "rv64", 4);

  /* Update standard arch attributes.  */
  e_ext = FALSE;
  for (i = 0; arch_info[i].name; i++)
    {
      struct arch_info *info;

      info = (struct arch_info *) hash_find
	(arch_info_hash, arch_info[i].name);
      if (info && info->valid)
	{
	  if (strcmp(info->name, "e") == 0)
	    e_ext = TRUE;

	  if ((strcmp(info->name, "i") == 0) && e_ext)
	    {
	      info->valid = 0;
	      continue;
	    }

	  strncat(out_arch, info->name, strlen (info->name));
	  strncat(out_arch, info->v_major, strlen (info->v_major));
	  strncat(out_arch, "p", 1);
	  strncat(out_arch, info->v_minor, strlen (info->v_minor));
	  info->valid = 0;
	}
    }

  /* Update non-standard arch attributes.  */
  hash_traverse (arch_info_hash, riscv_update_non_standard_arch_attr);

  bfd_elf_add_proc_attr_string (stdoutput, Tag_arch, out_arch);

  if (strlen (out_arch) > arch_attr_strlen)
    as_fatal ("Not enough spaces for the architecture attribute name");

  /* Clean the unused items.  */
  first_X_arch = 1;
  free (out_arch);
  out_arch = NULL;
}
#endif

static void
andes_riscv_set_public_attributes (void)
{
  if (!riscv_opts.arch_attr && !explicit_arch_attr)
    return;

  /* Write out arch attribute according to the arch_info_hash.  */
#ifdef DEBUG_ARCH_INFO_HASH
  printf ("===== Contents of arch attribute hash table =====\n");
  hash_traverse (arch_info_hash, riscv_print_arch_info_hash);
  printf ("\n");
#endif

#ifdef TO_REMOVE
  /* The assembly dose not contain instructions.  */
  if (!start_assemble)
    riscv_set_arch_attributes (NULL);
#endif

  riscv_write_out_arch_attr ();

#if 0
  if (!attributes_set_explicitly[Tag_RISCV_priv_spec])
    bfd_elf_add_proc_attr_int (stdoutput, Tag_RISCV_priv_spec,
			       DEFAULT_PRIV_SPEC);
  if (!attributes_set_explicitly[Tag_RISCV_priv_spec_minor])
    bfd_elf_add_proc_attr_int (stdoutput, Tag_RISCV_priv_spec_minor,
			       DEFAULT_PRIV_SPEC_MINOR);
  if (!attributes_set_explicitly[Tag_RISCV_priv_spec_revision])
    bfd_elf_add_proc_attr_int (stdoutput, Tag_RISCV_priv_spec_revision,
			       DEFAULT_PRIV_SPEC_REVISION);
  if (!attributes_set_explicitly[Tag_RISCV_strict_align])
    bfd_elf_add_proc_attr_int (stdoutput, Tag_RISCV_strict_align,
			       DEFAULT_STRICT_ALIGN);
  if (!attributes_set_explicitly[Tag_RISCV_stack_align])
    bfd_elf_add_proc_attr_int (stdoutput, Tag_RISCV_stack_align,
			       DEFAULT_STACK_ALIGN);
#endif

  if (m_ict_model
      && !attributes_set_explicitly[Tag_RISCV_ict_version
      + NUM_KNOWN_OBJ_ATTRIBUTES
      - TAG_VALUE_BEGIN_V5])
    bfd_elf_add_proc_attr_int (stdoutput, Tag_RISCV_ict_version,
			       DEFAULT_ICT_VERSION);
  if (m_ict_model
      && !attributes_set_explicitly[Tag_RISCV_ict_model
      + NUM_KNOWN_OBJ_ATTRIBUTES
      - TAG_VALUE_BEGIN_V5])
    bfd_elf_add_proc_attr_string (stdoutput, Tag_RISCV_ict_model,
				  m_ict_model);
}

/* Add the default contents for the .riscv.attributes section.  */

/* Andes .attribute directive extensions.  */

static void
andes_pre_s_riscv_attribute (void)
{
  /* patch .attribute strict_align, X
   *   to  .attribute unaligned_access, !X
   */
  char *s = input_line_pointer;
  if (strncmp(s, "strict_align,", 13) != 0)
    return;

  /* FEED ME! what if X is an expression? */
  s = input_line_pointer + 13;
  *s ^= 1; /* '0' <-> '1' */
}

static void
andes_post_s_riscv_attribute (int tag)
{
  if (tag < NUM_KNOWN_OBJ_ATTRIBUTES)
    attributes_set_explicitly[tag] = 1;
  else if (tag >= TAG_VALUE_BEGIN_V5 &&
	   tag < (TAG_VALUE_BEGIN_V5 + NUM_KNOWN_OBJ_ATTRIBUTES_V5))
    attributes_set_explicitly[tag + NUM_KNOWN_OBJ_ATTRIBUTES -
			      TAG_VALUE_BEGIN_V5] = 1;
}

static void
andes_ignore_input (int mode ATTRIBUTE_UNUSED)
{
  while (!is_end_of_line[(unsigned char) *input_line_pointer])
    ++input_line_pointer;
}

/* Pseudo-op table.  */

static const pseudo_typeS riscv_pseudo_table[] =
{
  /* RISC-V-specific pseudo-ops.  */
  {"option", s_riscv_option, 0},
  {"byte", riscv_aligned_cons, 0},
  {"short", riscv_aligned_cons, 1},
  {"half", riscv_aligned_cons, 1},
  {"hword", riscv_aligned_cons, 1},
  {"int", riscv_aligned_cons, 2},
  {"long", riscv_aligned_cons, 2},
  {"word", riscv_aligned_cons, 2},
  {"dword", riscv_aligned_cons, 3},
  {"quad", riscv_aligned_cons, 3},
  {"octa", riscv_aligned_cons, 4},
  {"qword", riscv_aligned_cons, 4},
  {"dtprelword", s_dtprel, 4},
  {"dtpreldword", s_dtprel, 8},
  {"bss", s_bss, 0},
  {"sleb128", s_riscv_leb128, 1},
  {"insn", s_riscv_insn, 0},
  {"attribute", s_riscv_attribute, 0},
  {"no_ex9_begin", riscv_no_execit, 1},
  {"no_ex9_end", riscv_no_execit, 0},
  {"no_execit_begin", riscv_no_execit, 1},
  {"no_execit_end", riscv_no_execit, 0},
  {"innermost_loop_begin", riscv_innermost_loop, 1},
  {"innermost_loop_end", riscv_innermost_loop, 0},
  {"addrsig", andes_ignore_input, 0},
  {"addrsig_sym", andes_ignore_input, 0},

  { NULL, NULL, 0 },
};

void
riscv_pop_insert (void)
{
  extern void pop_insert (const pseudo_typeS *);

  pop_insert (riscv_pseudo_table);
}

/* Implement md_parse_name for parsing SYNBOL@ICT.  */

int
riscv_parse_name (char const *name, expressionS *exprP,
                  enum expr_mode mode ATTRIBUTE_UNUSED,
                  char *nextcharP)
{
  segT segment;
  char *next;

  /* We only deal with the case that includes `@ICT'.  */
  if (*nextcharP != '@'
      || (strncasecmp (input_line_pointer + 1, "ICT", 3) != 0
	  && strncasecmp (input_line_pointer + 1, "ict", 3) != 0))
    return 0;

  exprP->X_op_symbol = NULL;
  exprP->X_md = BFD_RELOC_UNUSED;

  exprP->X_add_symbol = symbol_find_or_make (name);
  exprP->X_op = O_symbol;
  exprP->X_add_number = 0;

  /* Check the specail name if a symbol.  */
  segment = S_GET_SEGMENT (exprP->X_add_symbol);
  if ((segment != undefined_section) && (*nextcharP != '@'))
    return 0;

  next = input_line_pointer + 1 + 3;	/* strlen (ict/ICT) is 3.  */

  if (!is_part_of_name (*next))
    {
      exprP->X_md = BFD_RELOC_RISCV_ICT_HI20;
      *input_line_pointer = *nextcharP;
      input_line_pointer = next;
      *nextcharP = *input_line_pointer;
      *input_line_pointer = '\0';
    }

  return 1;
}

/* insert cmodel=large indirect symbols */

void riscv_andes_md_cleanup (void)
{
}

// md_convert_frag
// add_relaxed_insn
