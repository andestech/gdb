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

extern riscv_subset_t *
riscv_lookup_subset (const riscv_subset_list_t *,
		     const char *);

extern riscv_subset_t *
riscv_lookup_subset_version (const riscv_subset_list_t *,
			     const char *,
			     int, int);

typedef struct {
  riscv_subset_list_t *subset_list;
  void (*error_handler) (const char *,
			 ...) ATTRIBUTE_PRINTF_1;
  unsigned *xlen;
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
  int target_aligned;
  int gp_relative_insn;
  FILE *sym_ld_script;
  int avoid_btb_miss;
  /* For EX9.  */
  int target_optimize;
  int relax_status;
  char *ex9_export_file;
  FILE *ex9_import_file;
  int keep_import_ex9;
  int update_ex9_table;
  int ex9_limit;
  int ex9_loop_aware;
};

/* Get the RISC-V ELF linker hash table from a link_info structure.  */
#define riscv_elf_hash_table(p) \
  (elf_hash_table_id ((struct elf_link_hash_table *) ((p)->hash)) \
  == RISCV_ELF_DATA ? ((struct riscv_elf_link_hash_table *) ((p)->hash)) : NULL)

/* EX9 extention .  */

/* Optimization status mask.  */
#define RISCV_RELAX_EX9_DONE	(1 << 1)

/* Optimization turn on mask.  */
#define RISCV_RELAX_EX9_ON	(1 << 1)

/* Relocation flags for R_RISCV_ERLAX_ENTRY.  */

/* Set if relax on this section is done or disabled.  */
#define R_RISCV_RELAX_ENTRY_DISABLE_RELAX_FLAG	(1 << 31)
/* EX9 must be explicitly enabled, so we won't mess up handcraft assembly code.
   Enable EX9 optimization for this section.  */
#define R_RISCV_RELAX_ENTRY_EX9_FLAG		(1 << 2)

/* Relocation flags for R_RISCV_RELAX_REGION_BEGIN/END.  */

/* Suppress EX9 optimization in the region.  */
#define R_RISCV_RELAX_REGION_NO_EX9_FLAG	(1 << 2)
/* A Innermost loop region.  Some optimizations is suppressed in this region
   due to performance drop.  */
#define R_RISCV_RELAX_REGION_LOOP_FLAG		(1 << 4)

extern unsigned int number_of_howto_table;
