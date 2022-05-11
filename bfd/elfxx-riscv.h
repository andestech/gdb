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
extern unsigned int ict_model;
extern unsigned int ict_table_entries;
extern bool find_imported_ict_table;
extern const unsigned int number_of_howto_table;

#define EXECIT_INSN 0x8000u
#define EXECIT_SECTION ".exec.itable"
#define EXECIT_HASH_OK (0)
#define EXECIT_HASH_NG (1)
#define MASK_2M ((1u << 21) - 1)
#define EXECIT_HW_ENTRY_MAX (1024)

#define RISCV_RELAX_EXECIT_ON	(1u << 0)
#define RISCV_RELAX_EXECIT_DONE	(1u << 1)

/* Relocation flags for R_RISCV_RELAX_ENTRY.  */
/* Set if relax on this section is done or disabled.  */
#define R_RISCV_RELAX_ENTRY_DISABLE_RELAX_FLAG	(1u << 31)
/* EXECIT must be explicitly enabled, so we won't mess up handcraft assembly code.
   Enable EXECIT optimization for this section.  */
#define R_RISCV_RELAX_ENTRY_EXECIT_FLAG		(1u << 2)

/* Relocation flags for R_RISCV_RELAX_REGION_BEGIN/END.  */
/* Suppress EXECIT optimization in the region.  */
#define R_RISCV_RELAX_REGION_NO_EXECIT_FLAG	(1u << 2)
/* A Innermost loop region.  Some optimizations is suppressed in this region
   due to performance drop.  */
#define R_RISCV_RELAX_REGION_IMLOOP_FLAG		(1u << 4)

/* Get the RISC-V ELF linker hash table from a link_info structure.  */
#define riscv_elf_hash_table(p) \
  ((is_elf_hash_table ((p)->hash)					\
    && elf_hash_table_id (elf_hash_table (p)) == RISCV_ELF_DATA)	\
   ? (struct riscv_elf_link_hash_table *) (p)->hash : NULL)

#define LIST_ITER(list_pp, obj, each_cb, final_cb) \
      list_iterate((list_entry_t **)list_pp, (void *)obj, \
		   (list_iter_cb_t)each_cb, (list_iter_cb_t)final_cb);

#define LIST_ITER(list_pp, obj, each_cb, final_cb) \
      list_iterate((list_entry_t **)list_pp, (void *)obj, \
		   (list_iter_cb_t)each_cb, (list_iter_cb_t)final_cb);
#define LIST_EACH(list_pp, func) LIST_ITER(list_pp, NULL, func, NULL)
#define LIST_EACH1(list_pp, func, obj) LIST_ITER(list_pp, obj, func, NULL)
#define LIST_APPEND(list_pp, obj) LIST_ITER(list_pp, obj, NULL, append_final_cb)
#define LIST_LEN(list_pp) LIST_ITER(list_pp, NULL, NULL, NULL)

/* Used for riscv_relocation_check.  */
enum
{
  DATA_EXIST = 1,
  /* For checking EXECIT with alignment.  */
  ALIGN_CLEAN_PRE = 1 << 1,
  ALIGN_PUSH_PRE = 1 << 2,
  RELAX_REGION_END = 1 << 3,
  SYMBOL_RELOCATION = 1 << 4,
  DUMMY
};

typedef struct { void *next; } list_entry_t;
typedef int (*list_iter_cb_t)(void *list_pp, void *obj, void *a, void *b);

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
  /* Default page size  */
  int set_relax_page_size;
  /* For EXECIT.  */
  /* exec.it options  */
  FILE *execit_import_file;
  char *execit_export_file;
  int target_optimization;
  int execit_limit;
  struct
  {
    int noji; /* Forbid JI insn convert to execit.  */
    int nols; /* Forbid load-store insn convert to execit.  */
    int rvv:1;
    int rvp:1;
    int fls:1;
    int xdsp:1;
  } execit_flags;
  int update_execit_table:1;
  int keep_import_execit:1;
  int execit_loop_aware:1;
} andes_ld_options_t;

/* exec.it */
typedef struct execit_itable_entry
{
  Elf_Internal_Rela *irel;      /* with relocation  */
  Elf_Internal_Sym *isym;       /* for local symbol  */
  struct elf_link_hash_entry *h;/* for global symbol  */
  asection *sec;                /* section of insn  */
  asection *isec;               /* section of local symbol  */
  bfd_vma pc;                   /* insn vma  */
  bfd_vma relocation;           /* might keep host-addr of irel instead  */
  bfd_vma addend;               /* relocation addend  */
  Elf_Internal_Rela irel_copy;
  Elf_Internal_Sym isym_copy;
  int est_count;        /* when hashing  */
  int ref_count;        /* when replacing  */
  int rank_order;       /* when building itable  */
  int itable_index;	/* when replacing/relocating  */
  int entries;          /* 0 as default 1  */
  uint32_t insn;        /* raw insn  */
  uint32_t fixed;       /* fixed parts of insn  */
} execit_itable_t;

typedef struct execit_irel_entry
{
  struct execit_irel_entry *next;
  execit_itable_t ie;
} execit_irel_t;

typedef struct execit_vma_entry
{
  struct execit_vma_entry *next;
  bfd_vma vma;
} execit_vma_t;

typedef struct execit_hash_entry
{
  struct bfd_hash_entry root;
  execit_itable_t ie;
  execit_irel_t *irels;
  execit_vma_t *vmas;
  int next; /* next itable index associated  */
  int id; /* for determined itable entries  */
  unsigned int is_worthy:1;
  unsigned int is_chosen:1;
  unsigned int is_final:1;
  unsigned int is_relocated:1;
  unsigned int is_imported:1;
} execit_hash_t;

typedef struct execit_rank_entry
{
  struct execit_rank_entry *next;
  execit_hash_t *he;
} execit_rank_t;

/* It used to record the blank information for EXECIT replacement.  */
typedef struct execit_blank_unit
{
  struct execit_blank_unit *next;
  bfd_vma offset;
  bfd_vma size;
} execit_blank_unit_t;

typedef struct execit_blank_section
{
  struct execit_blank_section *next;
  asection *sec;
  execit_blank_unit_t *unit;
} execit_blank_section_t;

typedef struct execit_blank_abfd
{
  struct execit_blank_abfd *next;
  bfd *abfd;
  execit_blank_section_t *sec;
} execit_blank_abfd_t;

typedef struct execit_irelx
{
  struct execit_irelx *next;
  Elf_Internal_Rela saved_irel;
  bfd_vma annotation;
} execit_irelx_t;

typedef struct execit_state
{
  /* EXECIT hash table, used to store all patterns of code.  */
  asection *final_sec;
  asection *itable_section;
  execit_rank_t *rank_list;
  execit_blank_abfd_t *blank_list;
  execit_irelx_t *irelx_list;
  execit_hash_t **itable_array;
  struct riscv_elf_link_hash_table *htab;
  struct bfd_hash_table code_hash;
  bfd_vma jal_window_end;
  int raw_itable_entries;
  int next_itable_index;
  int import_number;
  unsigned int is_init:1;
  unsigned int is_built:1;
  unsigned int is_replaced:1;
  unsigned int relocate_itable_done:1;
  unsigned int is_itable_finalized:1;
  unsigned int is_determining_lui:1;
  unsigned int is_itb_base_set:1;
  unsigned int is_replace_again:1;
  unsigned int is_import_ranked:1;
} execit_state_t;

/* execit processing context  */
typedef struct execit_context
{
  execit_itable_t ie;

  /* parameters  */
  bfd *abfd;
  asection *sec;
  struct bfd_link_info *info;
  Elf_Internal_Rela *irel;
  bfd_byte *contents;
  bfd_vma off;
  char buf[0x400];
} execit_context_t;

typedef struct ict_state
{
  unsigned int is_init:1;
} ict_state_t;

typedef struct andes_state
{
  unsigned int check_start_export_sym:1;
} andes_state_t;

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

/* ICT stuff  */
#define ANDES_ICT_SECTION ".nds.ict"

typedef struct andes_ict_entry
{
  struct bfd_hash_entry root;
  struct andes_ict_entry *next;
  struct elf_link_hash_entry *h;
  char *name;
  bfd_vma vma;
  int index;
} andes_ict_entry_t;

extern andes_ict_entry_t *get_ict_entry_list_head (void);
extern int get_ict_size (void);
extern
andes_ict_entry_t *andes_ict_entry_list_add (int index, const char *name, bfd_vma vma);
andes_ict_entry_t *andes_ict_entry_list_insert (struct elf_link_hash_entry *h);

/* ACE stuff  */
enum hw_res_type
{
  HW_GPR,
  HW_UINT,
  HW_INT,
  HW_ACR,
  HW_FPR,
  HW_VR
};

/* Data structures used by ACE */
typedef struct ace_keyword
{
  const char *name;     /* register name */
  int value;            /* register index */
  uint64_t attr;        /* register attribute */
} ace_keyword_t;

typedef struct ace_operand
{
  const char *name;     /* operand name */
  int bitpos;           /* operand start position */
  int bitsize;          /* operand width */
  int shift;            /* operand shift amount */
  int hw_res;           /* hardware resource */
  const char *hw_name;  /* hardware/register name */
} ace_op_t;
/* } Andes  */
