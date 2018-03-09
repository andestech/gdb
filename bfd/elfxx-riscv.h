/* RISC-V ELF specific backend routines.
   Copyright (C) 2011-2019 Free Software Foundation, Inc.

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

extern reloc_howto_type *
riscv_reloc_name_lookup (bfd *, const char *);

extern reloc_howto_type *
riscv_reloc_type_lookup (bfd *, bfd_reloc_code_real_type);

extern reloc_howto_type *
riscv_elf_rtype_to_howto (bfd *, unsigned int r_type);

#define RISCV_DONT_CARE_VERSION -1

/* The information of architecture attribute.  */
struct riscv_subset_t
{
  const char *name;
  int major_version;
  int minor_version;
  struct riscv_subset_t *next;
};

typedef struct riscv_subset_t riscv_subset_t;

typedef struct {
  riscv_subset_t *head;
  riscv_subset_t *tail;
} riscv_subset_list_t;

extern void
riscv_release_subset_list (riscv_subset_list_t *);

extern void
riscv_add_subset (riscv_subset_list_t *,
		  const char *,
		  int, int);

bfd_boolean
riscv_lookup_subset (const riscv_subset_list_t *subset_list,
		     const char *subset,
		     riscv_subset_t **current);

extern riscv_subset_t *
riscv_lookup_subset_version (const riscv_subset_list_t *,
			     const char *,
			     int, int);

typedef struct {
  riscv_subset_list_t *subset_list;
  void (*error_handler) (const char *,
			 ...) ATTRIBUTE_PRINTF_1;
  unsigned *xlen;
  void (*get_default_version) (const char *,
			       int *,
			       int *);
} riscv_parse_subset_t;

extern bfd_boolean
riscv_parse_subset (riscv_parse_subset_t *,
		    const char *);

extern const char *
riscv_supported_std_ext (void);

extern void
riscv_release_subset_list (riscv_subset_list_t *);

extern char *
riscv_arch_str (unsigned, const riscv_subset_list_t *);

/* ISA extension name class. E.g. "zbb" corresponds to RV_ISA_CLASS_Z,
   "xargs" corresponds to RV_ISA_CLASS_X, etc.  */

typedef enum riscv_isa_ext_class
{
  RV_ISA_CLASS_S,
  RV_ISA_CLASS_H,
  RV_ISA_CLASS_Z,
  RV_ISA_CLASS_X,
  RV_ISA_CLASS_UNKNOWN
} riscv_isa_ext_class_t;

/* Classify the argument 'ext' into one of riscv_isa_ext_class_t.  */

riscv_isa_ext_class_t
riscv_get_prefix_class (const char *);

extern int
riscv_get_priv_spec_class (const char *, enum riscv_priv_spec_class *);

extern int
riscv_get_priv_spec_class_from_numbers (unsigned int,
					unsigned int,
					unsigned int,
					enum riscv_priv_spec_class *);

extern const char *
riscv_get_priv_spec_name (enum riscv_priv_spec_class);

extern int
riscv_compare_subsets (const char *, const char *);

struct riscv_elf_link_hash_table
{
  struct elf_link_hash_table elf;

  /* Short-cuts to get to dynamic linker sections.  */
  asection *sdyntdata;

  /* Small local sym to section mapping cache.  */
  struct sym_cache sym_cache;

  /* The max alignment of output sections.  */
  bfd_vma max_alignment;

  /* Target dependent options.  */
  FILE *sym_ld_script;
  /* For internal usage.  */
  int gp_relative_insn;
  int set_relax_align;
  int target_aligned;
  int avoid_btb_miss;
  int set_relax_lui;
  int set_relax_pc;
  int set_relax_call;
  int set_relax_tls_le;
  int set_relax_cross_section_call;
  int set_workaround;
  int set_relax_aggressive;
  int set_relax_page_size;
  /* For EXECIT.  */
  int target_optimize;
  int relax_status;
  char *execit_export_file;
  FILE *execit_import_file;
  int keep_import_execit;
  int update_execit_table;
  int execit_limit;
  int execit_loop_aware;
  bfd_boolean execit_noji; /* Forbid JI insn convert to execit.  */
  bfd_boolean execit_nols; /* Forbid load-store insn convert to execit.  */
  struct
  {
    int rvv:1;
    int rvp:1;
    int fls:1;
    int xdsp:1;
  } execit;
};

/* Get the RISC-V ELF linker hash table from a link_info structure.  */
#define riscv_elf_hash_table(p) \
  (elf_hash_table_id ((struct elf_link_hash_table *) ((p)->hash)) \
  == RISCV_ELF_DATA ? ((struct riscv_elf_link_hash_table *) ((p)->hash)) : NULL)

/* EXECIT extention.  */

/* Optimization status mask.  */
#define RISCV_RELAX_EXECIT_DONE	(1 << 1)

/* Optimization turn on mask.  */
#define RISCV_RELAX_EXECIT_ON	(1 << 1)

/* Relocation flags for R_RISCV_ERLAX_ENTRY.  */

/* Set if relax on this section is done or disabled.  */
#define R_RISCV_RELAX_ENTRY_DISABLE_RELAX_FLAG	(1 << 31)
/* EXECIT must be explicitly enabled, so we won't mess up handcraft assembly code.
   Enable EXECIT optimization for this section.  */
#define R_RISCV_RELAX_ENTRY_EXECIT_FLAG		(1 << 2)

/* Relocation flags for R_RISCV_RELAX_REGION_BEGIN/END.  */

/* Suppress EXECIT optimization in the region.  */
#define R_RISCV_RELAX_REGION_NO_EXECIT_FLAG	(1 << 2)
/* A Innermost loop region.  Some optimizations is suppressed in this region
   due to performance drop.  */
#define R_RISCV_RELAX_REGION_LOOP_FLAG		(1 << 4)

extern unsigned int number_of_howto_table;

extern unsigned int ict_table_entries;
extern unsigned int ict_model;
extern bfd_boolean find_imported_ict_table;

/* ICT stuff  */
typedef struct ict_sym_list
{
  struct ict_sym_list *next;
  struct elf_link_hash_entry *h;
  char *name;
  bfd_vma vma;
  int index;
} ict_sym_list_t;

extern ict_sym_list_t *get_ict_sym_list_head (void);
extern int get_ict_sym_list_len (void);
extern ict_sym_list_t *andes_ict_sym_list_add (int index, const char *name,
					       bfd_vma vma);
ict_sym_list_t *andes_ict_sym_list_insert (struct elf_link_hash_entry *h);