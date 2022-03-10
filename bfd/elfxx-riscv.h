/* RISC-V ELF specific backend routines.
   Copyright (C) 2011-2022 Free Software Foundation, Inc.

   Contributed by Andrew Waterman (andrew@sifive.com).
   Based on MIPS target.

   This file is part of BFD, the Binary File Descriptor library.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; see the file COPYING3. If not,
   see <http://www.gnu.org/licenses/>.  */

#include "elf/common.h"
#include "elf/internal.h"
#include "opcode/riscv.h"
#include "cpu-riscv.h"

#define RISCV_UNKNOWN_VERSION -1

extern reloc_howto_type *
riscv_reloc_name_lookup (bfd *, const char *);

extern reloc_howto_type *
riscv_reloc_type_lookup (bfd *, bfd_reloc_code_real_type);

extern reloc_howto_type *
riscv_elf_rtype_to_howto (bfd *, unsigned int r_type);

/* The information of architecture attribute.  */
struct riscv_subset_t
{
  const char *name;
  int major_version;
  int minor_version;
  struct riscv_subset_t *next;
};

typedef struct riscv_subset_t riscv_subset_t;

typedef struct
{
  riscv_subset_t *head;
  riscv_subset_t *tail;
} riscv_subset_list_t;

extern void
riscv_release_subset_list (riscv_subset_list_t *);

extern void
riscv_add_subset (riscv_subset_list_t *,
		  const char *,
		  int, int);

extern bool
riscv_lookup_subset (const riscv_subset_list_t *,
		     const char *,
		     riscv_subset_t **);

typedef struct
{
  riscv_subset_list_t *subset_list;
  void (*error_handler) (const char *,
			 ...) ATTRIBUTE_PRINTF_1;
  unsigned *xlen;
  enum riscv_spec_class *isa_spec;
  bool check_unknown_prefixed_ext;
} riscv_parse_subset_t;

extern bool
riscv_parse_subset (riscv_parse_subset_t *,
		    const char *);

extern void
riscv_release_subset_list (riscv_subset_list_t *);

extern char *
riscv_arch_str (unsigned, const riscv_subset_list_t *);

extern size_t
riscv_estimate_digit (unsigned);

extern int
riscv_compare_subsets (const char *, const char *);

extern riscv_subset_list_t *
riscv_copy_subset_list (riscv_subset_list_t *);

extern bool
riscv_update_subset (riscv_parse_subset_t *, const char *);

extern bool
riscv_subset_supports (riscv_parse_subset_t *, const char *);

extern bool
riscv_multi_subset_supports (riscv_parse_subset_t *, enum riscv_insn_class);

extern bool
riscv_disassemble_subset_tweak (riscv_parse_subset_t *,
				const struct riscv_opcode *op,
				insn_t insn);

extern void
bfd_elf32_riscv_set_data_segment_info (struct bfd_link_info *, int *);
extern void
bfd_elf64_riscv_set_data_segment_info (struct bfd_link_info *, int *);

/* { Andes  */
/* Get the RISC-V ELF linker hash table from a link_info structure.  */
#define riscv_elf_hash_table(p) \
  ((is_elf_hash_table ((p)->hash)					\
    && elf_hash_table_id (elf_hash_table (p)) == RISCV_ELF_DATA)	\
   ? (struct riscv_elf_link_hash_table *) (p)->hash : NULL)

#include "hashtab.h"
typedef struct andes_ld_options
{
  /* Export global symbols into linker script.  */
  FILE *sym_ld_script;
  /* Defalut do relax align.  */
  int set_relax_align;
  /* Defalut do target aligned.  */
  int target_aligned;
  /* Support gp relative insn relaxation.  */
  int gp_relative_insn;
  /* Default avoid BTB miss.  */
  int avoid_btb_miss;
  /* Defalut do relax lui.  */
  int set_relax_lui;
  /* Defalut do relax pc.  */
  int set_relax_pc;
  /* Defalut do relax call.  */
  int set_relax_call;
  /* Defalut do relax tls le.  */
  int set_relax_tls_le;
  /* Defalut do relax cross section call.  */
  int set_relax_cross_section_call;
  /* Defalut do workaround.  */
  int set_workaround;
  /* exec.it options  */
  int target_optimization;
} andes_ld_options_t;

struct riscv_elf_link_hash_table
{
  struct elf_link_hash_table elf;

  /* Short-cuts to get to dynamic linker sections.  */
  asection *sdyntdata;

  /* The max alignment of output sections.  */
  bfd_vma max_alignment;

  /* Used by local STT_GNU_IFUNC symbols.  */
  htab_t loc_hash_table;
  void * loc_hash_memory;

  /* The index of the last unused .rel.iplt slot.  */
  bfd_vma last_iplt_index;

  /* The data segment phase, don't relax the section
     when it is exp_seg_relro_adjust.  */
  int *data_segment_phase;

  /* Relocations for variant CC symbols may be present.  */
  int variant_cc;

  /* { Andes  */
  andes_ld_options_t andes;
  /* } Andes  */
};

/* EXECIT extention.  */

/* Optimization turn on mask.  */
#define RISCV_RELAX_EXECIT_ON	(1 << 0)

/* Optimization status mask.  */
#define RISCV_RELAX_EXECIT_DONE	(1 << 1)
/* } Andes  */
