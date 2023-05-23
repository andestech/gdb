/* RISC-V-specific support for NN-bit ELF.
   Copyright (C) 2011-2022 Free Software Foundation, Inc.

   Contributed by Andrew Waterman (andrew@sifive.com).
   Based on TILE-Gx and MIPS targets.

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

/* This file handles RISC-V ELF targets.  */

#include "sysdep.h"
#include "bfd.h"
#include "libbfd.h"
#include "bfdlink.h"
#include "genlink.h"
#include "elf-bfd.h"
#include "elfxx-riscv.h"
#include "elf/riscv.h"
#include "opcode/riscv.h"
#include "objalloc.h"

#include <limits.h>
#ifndef CHAR_BIT
#define CHAR_BIT 8
#endif

/* Internal relocations used exclusively by the relaxation pass.  */
/* bug!
ELFNN_R_TYPE (R_RISCV_DELETE) != R_RISCV_DELETE for 32-bit toolchains
#define R_RISCV_DELETE (R_RISCV_max + 1)
*/

#define ARCH_SIZE NN

#define MINUS_ONE ((bfd_vma)0 - 1)

#define RISCV_ELF_LOG_WORD_BYTES (ARCH_SIZE == 32 ? 2 : 3)

#define RISCV_ELF_WORD_BYTES (1 << RISCV_ELF_LOG_WORD_BYTES)

/* The name of the dynamic interpreter.  This is put in the .interp
   section.  */

#define ELF64_DYNAMIC_INTERPRETER "/lib/ld.so.1"
#define ELF32_DYNAMIC_INTERPRETER "/lib32/ld.so.1"

#define ELF_ARCH			bfd_arch_riscv
#define ELF_TARGET_ID			RISCV_ELF_DATA
#define ELF_MACHINE_CODE		EM_RISCV
#define ELF_MAXPAGESIZE			0x1000
#define ELF_COMMONPAGESIZE		0x1000

#define RISCV_ATTRIBUTES_SECTION_NAME ".riscv.attributes"

/* Helper functions for Rom Patch and ICT.  */
static void riscv_elf_ict_init (void);
static void andes_relocate_ict_table (struct bfd_link_info *, bfd *);
static void andes_complement_ict_symbols (struct bfd_link_info *info);

#define ARCH_SIZE NN

#define MINUS_ONE ((bfd_vma)0 - 1)

#define RISCV_ELF_LOG_WORD_BYTES (ARCH_SIZE == 32 ? 2 : 3)

#define RISCV_ELF_WORD_BYTES (1 << RISCV_ELF_LOG_WORD_BYTES)

/* The name of the dynamic interpreter.  This is put in the .interp
   section.  */

#define ELF64_DYNAMIC_INTERPRETER "/lib/ld.so.1"
#define ELF32_DYNAMIC_INTERPRETER "/lib32/ld.so.1"

#define ELF_ARCH			bfd_arch_riscv
#define ELF_TARGET_ID			RISCV_ELF_DATA
#define ELF_MACHINE_CODE		EM_RISCV
#define ELF_MAXPAGESIZE			0x1000
#define ELF_COMMONPAGESIZE		0x1000

/* RISC-V ELF linker hash entry.  */

struct riscv_elf_link_hash_entry
{
  struct elf_link_hash_entry elf;

#define GOT_UNKNOWN	0
#define GOT_NORMAL	1
#define GOT_TLS_GD	2
#define GOT_TLS_IE	4
#define GOT_TLS_LE	8
  char tls_type;

  /* { Andes */
  bool indirect_call;
  /* } Andes */
};

#define riscv_elf_hash_entry(ent) \
  ((struct riscv_elf_link_hash_entry *) (ent))

struct _bfd_riscv_elf_obj_tdata
{
  struct elf_obj_tdata root;

  /* tls_type for each local got entry.  */
  char *local_got_tls_type;
};

#define _bfd_riscv_elf_tdata(abfd) \
  ((struct _bfd_riscv_elf_obj_tdata *) (abfd)->tdata.any)

#define _bfd_riscv_elf_local_got_tls_type(abfd) \
  (_bfd_riscv_elf_tdata (abfd)->local_got_tls_type)

#define _bfd_riscv_elf_tls_type(abfd, h, symndx)		\
  (*((h) != NULL ? &riscv_elf_hash_entry (h)->tls_type		\
     : &_bfd_riscv_elf_local_got_tls_type (abfd) [symndx]))

#define is_riscv_elf(bfd)				\
  (bfd_get_flavour (bfd) == bfd_target_elf_flavour	\
   && elf_tdata (bfd) != NULL				\
   && elf_object_id (bfd) == RISCV_ELF_DATA)

static bool
elfNN_riscv_mkobject (bfd *abfd)
{
  return bfd_elf_allocate_object (abfd,
				  sizeof (struct _bfd_riscv_elf_obj_tdata),
				  RISCV_ELF_DATA);
}

#include "elf/common.h"
#include "elf/internal.h"

#if 0 /* moved to elfxx-riscv.h  */
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

  riscv_table_jump_htab_t *table_jump_htab;
};
#endif

/* debug use */
#define ZCMT_PRINT_TABLE_JUMP_ENTRIES 0

/* Instruction access functions. */
#define riscv_get_insn(bits, ptr)		\
  ((bits) == 16 ? bfd_getl16 (ptr)		\
   : (bits) == 32 ? bfd_getl32 (ptr)		\
   : (bits) == 64 ? bfd_getl64 (ptr)		\
   : (abort (), (bfd_vma) - 1))
#define riscv_put_insn(bits, val, ptr)		\
  ((bits) == 16 ? bfd_putl16 (val, ptr)		\
   : (bits) == 32 ? bfd_putl32 (val, ptr)	\
   : (bits) == 64 ? bfd_putl64 (val, ptr)	\
   : (abort (), (void) 0))

/* Get the RISC-V ELF linker hash table from a link_info structure.  */
#define riscv_elf_hash_table(p) \
  ((is_elf_hash_table ((p)->hash)					\
    && elf_hash_table_id (elf_hash_table (p)) == RISCV_ELF_DATA)	\
   ? (struct riscv_elf_link_hash_table *) (p)->hash : NULL)

/* { Andes */
#define SZ_4K (0x1000)

typedef struct riscv_pcgp_hi_reloc riscv_pcgp_hi_reloc_t;

static bool
execit_set_itb_base (struct bfd_link_info *link_info);
static void
andes_execit_relocate_itable (struct bfd_link_info *link_info);
static bfd_vma
riscv_elf_execit_reloc_insn (execit_itable_t *ptr,
			     struct bfd_link_info *link_info);
static bfd_vma
riscv_elf_encode_relocation (bfd *abfd,
			     Elf_Internal_Rela *irel, bfd_vma relocation);
static void
riscv_insertion_sort (void *base, size_t nmemb, size_t size,
		      int (*compar) (const void *lhs, const void *rhs));
static int
compar_reloc (const void *lhs, const void *rhs);
static int
andes_relax_gp_insn (uint32_t *insn, Elf_Internal_Rela *rel,
		     riscv_pcgp_hi_reloc_t *hi, asection *sym_sec,
		     bfd_vma symval, bfd_vma gp, bfd_vma data_start);
static bool
riscv_init_global_pointer (bfd *output_bfd, struct bfd_link_info *info);
static bool
andes_relax_execit_ite (
  bfd *abfd,
  asection *sec,
  asection *sym_sec ATTRIBUTE_UNUSED,
  struct bfd_link_info *info ATTRIBUTE_UNUSED,
  Elf_Internal_Rela *rel,
  bfd_vma symval,
  bfd_vma max_alignment,
  bfd_vma reserve_size ATTRIBUTE_UNUSED,
  bool *again ATTRIBUTE_UNUSED,
  //riscv_pcgp_relocs *pcgp_relocs ATTRIBUTE_UNUSED,
  void *pcgp_relocs ATTRIBUTE_UNUSED,
  bool undefined_weak ATTRIBUTE_UNUSED);
static bool
andes_relax_fls_gp (
  bfd *abfd,
  asection *sec,
  asection *sym_sec ATTRIBUTE_UNUSED,
  struct bfd_link_info *info,
  Elf_Internal_Rela *rel,
  bfd_vma symval,
  bfd_vma max_alignment ATTRIBUTE_UNUSED,
  bfd_vma reserve_size ATTRIBUTE_UNUSED,
  bool *again ATTRIBUTE_UNUSED,
  //riscv_pcgp_relocs *pcgp_relocs ATTRIBUTE_UNUSED,
  void *pcgp_relocs ATTRIBUTE_UNUSED,
  bool undefined_weak ATTRIBUTE_UNUSED);

/* Record the symbol info for relaxing gp in relax_lui.  */
typedef struct relax_gp_sym_info
{
  Elf_Internal_Sym *lsym;
  struct elf_link_hash_entry *h;
  asection *sec;
  struct relax_gp_sym_info *next;
} relax_gp_sym_info_t;

static relax_gp_sym_info_t *relax_gp_sym_info_head = NULL;
static execit_state_t execit;
static ict_state_t ict;
static andes_linker_state_t nsta = {.opt = NULL};
extern andes_ict_state_t nds_ict_sta;
/* } Andes */

static bool
riscv_info_to_howto_rela (bfd *abfd,
			  arelent *cache_ptr,
			  Elf_Internal_Rela *dst)
{
  cache_ptr->howto = riscv_elf_rtype_to_howto (abfd, ELFNN_R_TYPE (dst->r_info));
  return cache_ptr->howto != NULL;
}

static void
riscv_elf_append_rela (bfd *abfd, asection *s, Elf_Internal_Rela *rel)
{
  const struct elf_backend_data *bed;
  bfd_byte *loc;

  bed = get_elf_backend_data (abfd);
  loc = s->contents + (s->reloc_count++ * bed->s->sizeof_rela);
  bed->s->swap_reloca_out (abfd, rel, loc);
}

/* Return true if a relocation is modifying an instruction. */

static bool
riscv_is_insn_reloc (const reloc_howto_type *howto)
{
  /* Heuristic: A multibyte destination with a nontrivial mask
     is an instruction */
  return (howto->bitsize > 8
	  && howto->dst_mask != 0
	  && ~(howto->dst_mask | (howto->bitsize < sizeof(bfd_vma) * CHAR_BIT
	       ? (MINUS_ONE << howto->bitsize) : (bfd_vma)0)) != 0);
}

/* PLT/GOT stuff.  */
#define PLT_HEADER_INSNS 8
#define PLT_ENTRY_INSNS 4
#define PLT_HEADER_SIZE (PLT_HEADER_INSNS * 4)
#define PLT_ENTRY_SIZE (PLT_ENTRY_INSNS * 4)
#define GOT_ENTRY_SIZE RISCV_ELF_WORD_BYTES
/* Reserve two entries of GOTPLT for ld.so, one is used for PLT resolver,
   the other is used for link map.  Other targets also reserve one more
   entry used for runtime profile?  */
#define GOTPLT_HEADER_SIZE (2 * GOT_ENTRY_SIZE)

#define sec_addr(sec) ((sec)->output_section->vma + (sec)->output_offset)

#if ARCH_SIZE == 32
# define MATCH_LREG MATCH_LW
#else
# define MATCH_LREG MATCH_LD
#endif

/* Generate a PLT header.  */

static bool
riscv_make_plt_header (bfd *output_bfd, bfd_vma gotplt_addr, bfd_vma addr,
		       uint32_t *entry)
{
  bfd_vma gotplt_offset_high = RISCV_PCREL_HIGH_PART (gotplt_addr, addr);
  bfd_vma gotplt_offset_low = RISCV_PCREL_LOW_PART (gotplt_addr, addr);

  /* RVE has no t3 register, so this won't work, and is not supported.  */
  if (elf_elfheader (output_bfd)->e_flags & EF_RISCV_RVE)
    {
      _bfd_error_handler (_("%pB: warning: RVE PLT generation not supported"),
			  output_bfd);
      return false;
    }

  /* auipc  t2, %hi(.got.plt)
     sub    t1, t1, t3		     # shifted .got.plt offset + hdr size + 12
     l[w|d] t3, %lo(.got.plt)(t2)    # _dl_runtime_resolve
     addi   t1, t1, -(hdr size + 12) # shifted .got.plt offset
     addi   t0, t2, %lo(.got.plt)    # &.got.plt
     srli   t1, t1, log2(16/PTRSIZE) # .got.plt offset
     l[w|d] t0, PTRSIZE(t0)	     # link map
     jr	    t3  */

  entry[0] = RISCV_UTYPE (AUIPC, X_T2, gotplt_offset_high);
  entry[1] = RISCV_RTYPE (SUB, X_T1, X_T1, X_T3);
  entry[2] = RISCV_ITYPE (LREG, X_T3, X_T2, gotplt_offset_low);
  entry[3] = RISCV_ITYPE (ADDI, X_T1, X_T1, (uint32_t) -(PLT_HEADER_SIZE + 12));
  entry[4] = RISCV_ITYPE (ADDI, X_T0, X_T2, gotplt_offset_low);
  entry[5] = RISCV_ITYPE (SRLI, X_T1, X_T1, 4 - RISCV_ELF_LOG_WORD_BYTES);
  entry[6] = RISCV_ITYPE (LREG, X_T0, X_T0, RISCV_ELF_WORD_BYTES);
  entry[7] = RISCV_ITYPE (JALR, 0, X_T3, 0);

  return true;
}

/* Generate a PLT entry.  */

static bool
riscv_make_plt_entry (bfd *output_bfd, bfd_vma got, bfd_vma addr,
		      uint32_t *entry)
{
  /* RVE has no t3 register, so this won't work, and is not supported.  */
  if (elf_elfheader (output_bfd)->e_flags & EF_RISCV_RVE)
    {
      _bfd_error_handler (_("%pB: warning: RVE PLT generation not supported"),
			  output_bfd);
      return false;
    }

  /* auipc  t3, %hi(.got.plt entry)
     l[w|d] t3, %lo(.got.plt entry)(t3)
     jalr   t1, t3
     nop  */

  entry[0] = RISCV_UTYPE (AUIPC, X_T3, RISCV_PCREL_HIGH_PART (got, addr));
  entry[1] = RISCV_ITYPE (LREG,  X_T3, X_T3, RISCV_PCREL_LOW_PART (got, addr));
  entry[2] = RISCV_ITYPE (JALR, X_T1, X_T3, 0);
  entry[3] = RISCV_NOP;

  return true;
}

/* Create an entry in an RISC-V ELF linker hash table.  */

static struct bfd_hash_entry *
link_hash_newfunc (struct bfd_hash_entry *entry,
		   struct bfd_hash_table *table, const char *string)
{
  /* Allocate the structure if it has not already been allocated by a
     subclass.  */
  if (entry == NULL)
    {
      entry =
	bfd_hash_allocate (table,
			   sizeof (struct riscv_elf_link_hash_entry));
      if (entry == NULL)
	return entry;
    }

  /* Call the allocation method of the superclass.  */
  entry = _bfd_elf_link_hash_newfunc (entry, table, string);
  if (entry != NULL)
    {
      struct riscv_elf_link_hash_entry *eh;

      eh = (struct riscv_elf_link_hash_entry *) entry;
      eh->tls_type = GOT_UNKNOWN;
      eh->indirect_call = false;
    }

  return entry;
}

static hashval_t
riscv_table_jump_htab_hash (const void *entry)
{
  const riscv_table_jump_htab_entry *e = entry;
  return (hashval_t)(e->address >> 2);
}

static int
riscv_table_jump_htab_entry_eq (const void *entry1, const void *entry2)
{
  const riscv_table_jump_htab_entry *e1 = entry1, *e2 = entry2;
  return e1->address == e2->address;
}

static bool
riscv_init_table_jump_htab (riscv_table_jump_htab_t *htab)
{
  htab->names = bfd_zmalloc (sizeof (const char *) * 256);
  htab->savings = bfd_zmalloc (sizeof (unsigned int) * 256);
  htab->tbj_indexes = bfd_zmalloc (sizeof (bfd_vma) * 256);
  htab->end_idx = 0;
  htab->total_saving = 0;

  htab->tbljt_htab = htab_create (50, riscv_table_jump_htab_hash,
			      riscv_table_jump_htab_entry_eq, free);
  if (htab->tbljt_htab == NULL)
    return false;

  htab->tbljalt_htab = htab_create (50, riscv_table_jump_htab_hash,
			      riscv_table_jump_htab_entry_eq, free);
  return htab->tbljalt_htab != NULL;
}

static void
riscv_free_table_jump_htab (riscv_table_jump_htab_t *htab)
{
  free (htab->names);
  free (htab->savings);
  free (htab->tbj_indexes);
  htab_delete (htab->tbljt_htab);
  htab_delete (htab->tbljalt_htab);
}

static bool
riscv_update_table_jump_entry (htab_t htab,
			       bfd_vma addr,
			       unsigned int benefit,
			       const char *name)
{
  riscv_table_jump_htab_entry search = {addr, 0, NULL, 0};
  riscv_table_jump_htab_entry *entry = htab_find (htab, &search);

  if (entry == NULL)
    {
      riscv_table_jump_htab_entry **slot =
	(riscv_table_jump_htab_entry **) htab_find_slot (
	  htab, &search, INSERT);

      BFD_ASSERT (*slot == NULL);

      *slot = (riscv_table_jump_htab_entry *) bfd_zmalloc (
	    sizeof (riscv_table_jump_htab_entry));

      if (*slot == NULL)
	return false;

      (*slot)->address = addr;
      (*slot)->benefit = benefit;
      (*slot)->name = name;
    }
  else
    entry->benefit += benefit;

  return true;
}

/* Compute a hash of a local hash entry.  We use elf_link_hash_entry
   for local symbol so that we can handle local STT_GNU_IFUNC symbols
   as global symbol.  We reuse indx and dynstr_index for local symbol
   hash since they aren't used by global symbols in this backend.  */

static hashval_t
riscv_elf_local_htab_hash (const void *ptr)
{
  struct elf_link_hash_entry *h = (struct elf_link_hash_entry *) ptr;
  return ELF_LOCAL_SYMBOL_HASH (h->indx, h->dynstr_index);
}

/* Compare local hash entries.  */

static int
riscv_elf_local_htab_eq (const void *ptr1, const void *ptr2)
{
  struct elf_link_hash_entry *h1 = (struct elf_link_hash_entry *) ptr1;
  struct elf_link_hash_entry *h2 = (struct elf_link_hash_entry *) ptr2;

  return h1->indx == h2->indx && h1->dynstr_index == h2->dynstr_index;
}

/* Find and/or create a hash entry for local symbol.  */

static struct elf_link_hash_entry *
riscv_elf_get_local_sym_hash (struct riscv_elf_link_hash_table *htab,
			      bfd *abfd, const Elf_Internal_Rela *rel,
			      bool create)
{
  struct riscv_elf_link_hash_entry eh, *ret;
  asection *sec = abfd->sections;
  hashval_t h = ELF_LOCAL_SYMBOL_HASH (sec->id,
				       ELFNN_R_SYM (rel->r_info));
  void **slot;

  eh.elf.indx = sec->id;
  eh.elf.dynstr_index = ELFNN_R_SYM (rel->r_info);
  slot = htab_find_slot_with_hash (htab->loc_hash_table, &eh, h,
				   create ? INSERT : NO_INSERT);

  if (!slot)
    return NULL;

  if (*slot)
    {
      ret = (struct riscv_elf_link_hash_entry *) *slot;
      return &ret->elf;
    }

  ret = (struct riscv_elf_link_hash_entry *)
	objalloc_alloc ((struct objalloc *) htab->loc_hash_memory,
			sizeof (struct riscv_elf_link_hash_entry));
  if (ret)
    {
      memset (ret, 0, sizeof (*ret));
      ret->elf.indx = sec->id;
      ret->elf.dynstr_index = ELFNN_R_SYM (rel->r_info);
      ret->elf.dynindx = -1;
      *slot = ret;
    }
  return &ret->elf;
}

#if ZCMT_PRINT_TABLE_JUMP_ENTRIES
static void
print_tablejump_entries(riscv_table_jump_htab_t *table_jump_htab)
{
  if (table_jump_htab->tbj_indexes[0])
    printf("cm.jt:\n");
  for (unsigned int z = 0; z < 64 && table_jump_htab->tbj_indexes[z] != 0; z ++)
    printf ("\tindex=%d, sym name=%s, address=0x%08lx, savings=%u\n",
	z, table_jump_htab->names[z], table_jump_htab->tbj_indexes[z], table_jump_htab->savings[z]);

  if (table_jump_htab->tbj_indexes[64])
    printf("cm.jalt:\n");
  for (unsigned int z = 64; z < 256 && table_jump_htab->tbj_indexes[z] != 0; z ++)
    printf ("\tindex=%d, sym name=%s, address=0x%08lx, savings=%u\n",
	z, table_jump_htab->names[z], table_jump_htab->tbj_indexes[z], table_jump_htab->savings[z]);
}
#endif

/* Destroy a RISC-V elf linker hash table.  */

static void
riscv_elf_link_hash_table_free (bfd *obfd)
{
  struct riscv_elf_link_hash_table *ret
    = (struct riscv_elf_link_hash_table *) obfd->link.hash;

  if (ret->loc_hash_table)
    htab_delete (ret->loc_hash_table);
  if (ret->loc_hash_memory)
    objalloc_free ((struct objalloc *) ret->loc_hash_memory);

  if (ret->table_jump_htab)
    {
#if ZCMT_PRINT_TABLE_JUMP_ENTRIES
      print_tablejump_entries(ret->table_jump_htab);
#endif
      riscv_free_table_jump_htab (ret->table_jump_htab);
      free (ret->table_jump_htab);
    }

  _bfd_elf_link_hash_table_free (obfd);
}

/* Create a RISC-V ELF linker hash table.  */

static struct bfd_link_hash_table *
riscv_elf_link_hash_table_create (bfd *abfd)
{
  struct riscv_elf_link_hash_table *ret;
  size_t amt = sizeof (struct riscv_elf_link_hash_table);

  ret = (struct riscv_elf_link_hash_table *) bfd_zmalloc (amt);
  if (ret == NULL)
    return NULL;

  if (!_bfd_elf_link_hash_table_init (&ret->elf, abfd, link_hash_newfunc,
				      sizeof (struct riscv_elf_link_hash_entry),
				      RISCV_ELF_DATA))
    {
      free (ret);
      return NULL;
    }

  ret->table_jump_htab = (riscv_table_jump_htab_t *) bfd_zmalloc (
	  sizeof (riscv_table_jump_htab_t));

  if (ret->table_jump_htab == NULL
	|| !riscv_init_table_jump_htab(ret->table_jump_htab))
    {
      riscv_elf_link_hash_table_free (abfd);
      return NULL;
    }

  ret->max_alignment = (bfd_vma) -1;

  /* Create hash table for local ifunc.  */
  ret->loc_hash_table = htab_try_create (1024,
					 riscv_elf_local_htab_hash,
					 riscv_elf_local_htab_eq,
					 NULL);
  ret->loc_hash_memory = objalloc_create ();
  if (!ret->loc_hash_table || !ret->loc_hash_memory)
    {
      riscv_elf_link_hash_table_free (abfd);
      return NULL;
    }
  ret->elf.root.hash_table_free = riscv_elf_link_hash_table_free;

  return &ret->elf.root;
}

/* Create the .got section.  */

static bool
riscv_elf_create_got_section (bfd *abfd, struct bfd_link_info *info)
{
  flagword flags;
  asection *s, *s_got;
  struct elf_link_hash_entry *h;
  const struct elf_backend_data *bed = get_elf_backend_data (abfd);
  struct elf_link_hash_table *htab = elf_hash_table (info);

  /* This function may be called more than once.  */
  if (htab->sgot != NULL)
    return true;

  flags = bed->dynamic_sec_flags;

  s = bfd_make_section_anyway_with_flags (abfd,
					  (bed->rela_plts_and_copies_p
					   ? ".rela.got" : ".rel.got"),
					  (bed->dynamic_sec_flags
					   | SEC_READONLY));
  if (s == NULL
      || !bfd_set_section_alignment (s, bed->s->log_file_align))
    return false;
  htab->srelgot = s;

  s = s_got = bfd_make_section_anyway_with_flags (abfd, ".got", flags);
  if (s == NULL
      || !bfd_set_section_alignment (s, bed->s->log_file_align))
    return false;
  htab->sgot = s;

  /* The first bit of the global offset table is the header.  */
  s->size += bed->got_header_size;

  if (bed->want_got_plt)
    {
      s = bfd_make_section_anyway_with_flags (abfd, ".got.plt", flags);
      if (s == NULL
	  || !bfd_set_section_alignment (s, bed->s->log_file_align))
	return false;
      htab->sgotplt = s;

      /* Reserve room for the header.  */
      s->size += GOTPLT_HEADER_SIZE;
    }

  if (bed->want_got_sym)
    {
      /* Define the symbol _GLOBAL_OFFSET_TABLE_ at the start of the .got
	 section.  We don't do this in the linker script because we don't want
	 to define the symbol if we are not creating a global offset
	 table.  */
      h = _bfd_elf_define_linkage_sym (abfd, info, s_got,
				       "_GLOBAL_OFFSET_TABLE_");
      elf_hash_table (info)->hgot = h;
      if (h == NULL)
	return false;
    }

  return true;
}

/* Create .plt, .rela.plt, .got, .got.plt, .rela.got, .dynbss, and
   .rela.bss sections in DYNOBJ, and set up shortcuts to them in our
   hash table.  */

static bool
riscv_elf_create_dynamic_sections (bfd *dynobj,
				   struct bfd_link_info *info)
{
  struct riscv_elf_link_hash_table *htab;

  htab = riscv_elf_hash_table (info);
  BFD_ASSERT (htab != NULL);

  if (!riscv_elf_create_got_section (dynobj, info))
    return false;

  if (!_bfd_elf_create_dynamic_sections (dynobj, info))
    return false;

  if (!bfd_link_pic (info))
    {
      /* Technically, this section doesn't have contents.  It is used as the
	 target of TLS copy relocs, to copy TLS data from shared libraries into
	 the executable.  However, if we don't mark it as loadable, then it
	 matches the IS_TBSS test in ldlang.c, and there is no run-time address
	 space allocated for it even though it has SEC_ALLOC.  That test is
	 correct for .tbss, but not correct for this section.  There is also
	 a second problem that having a section with no contents can only work
	 if it comes after all sections with contents in the same segment,
	 but the linker script does not guarantee that.  This is just mixed in
	 with other .tdata.* sections.  We can fix both problems by lying and
	 saying that there are contents.  This section is expected to be small
	 so this should not cause a significant extra program startup cost.  */
      htab->sdyntdata =
	bfd_make_section_anyway_with_flags (dynobj, ".tdata.dyn",
					    (SEC_ALLOC | SEC_THREAD_LOCAL
					     | SEC_LOAD | SEC_DATA
					     | SEC_HAS_CONTENTS
					     | SEC_LINKER_CREATED));
    }

  if (!htab->elf.splt || !htab->elf.srelplt || !htab->elf.sdynbss
      || (!bfd_link_pic (info) && (!htab->elf.srelbss || !htab->sdyntdata)))
    abort ();

  return true;
}

/* Copy the extra info we tack onto an elf_link_hash_entry.  */

static void
riscv_elf_copy_indirect_symbol (struct bfd_link_info *info,
				struct elf_link_hash_entry *dir,
				struct elf_link_hash_entry *ind)
{
  struct riscv_elf_link_hash_entry *edir, *eind;

  edir = (struct riscv_elf_link_hash_entry *) dir;
  eind = (struct riscv_elf_link_hash_entry *) ind;

  if (ind->root.type == bfd_link_hash_indirect
      && dir->got.refcount <= 0)
    {
      edir->tls_type = eind->tls_type;
      eind->tls_type = GOT_UNKNOWN;
    }
  _bfd_elf_link_hash_copy_indirect (info, dir, ind);
}

static bool
riscv_elf_record_tls_type (bfd *abfd, struct elf_link_hash_entry *h,
			   unsigned long symndx, char tls_type)
{
  char *new_tls_type = &_bfd_riscv_elf_tls_type (abfd, h, symndx);

  *new_tls_type |= tls_type;
  if ((*new_tls_type & GOT_NORMAL) && (*new_tls_type & ~GOT_NORMAL))
    {
      (*_bfd_error_handler)
	(_("%pB: `%s' accessed both as normal and thread local symbol"),
	 abfd, h ? h->root.root.string : "<local>");
      return false;
    }
  return true;
}

static bool
riscv_elf_record_got_reference (bfd *abfd, struct bfd_link_info *info,
				struct elf_link_hash_entry *h, long symndx)
{
  struct riscv_elf_link_hash_table *htab = riscv_elf_hash_table (info);
  Elf_Internal_Shdr *symtab_hdr = &elf_tdata (abfd)->symtab_hdr;

  if (htab->elf.sgot == NULL)
    {
      if (!riscv_elf_create_got_section (htab->elf.dynobj, info))
	return false;
    }

  if (h != NULL)
    {
      h->got.refcount += 1;
      return true;
    }

  /* This is a global offset table entry for a local symbol.  */
  if (elf_local_got_refcounts (abfd) == NULL)
    {
      bfd_size_type size = symtab_hdr->sh_info * (sizeof (bfd_vma) + 1);
      if (!(elf_local_got_refcounts (abfd) = bfd_zalloc (abfd, size)))
	return false;
      _bfd_riscv_elf_local_got_tls_type (abfd)
	= (char *) (elf_local_got_refcounts (abfd) + symtab_hdr->sh_info);
    }
  elf_local_got_refcounts (abfd) [symndx] += 1;

  return true;
}

static bool
bad_static_reloc (bfd *abfd, unsigned r_type, struct elf_link_hash_entry *h)
{
  reloc_howto_type * r = riscv_elf_rtype_to_howto (abfd, r_type);

  /* We propably can improve the information to tell users that they
     should be recompile the code with -fPIC or -fPIE, just like what
     x86 does.  */
  (*_bfd_error_handler)
    (_("%pB: relocation %s against `%s' can not be used when making a shared "
       "object; recompile with -fPIC"),
     abfd, r ? r->name : _("<unknown>"),
     h != NULL ? h->root.root.string : "a local symbol");
  bfd_set_error (bfd_error_bad_value);
  return false;
}

static bool
riscv_elf_update_ict_hash_table (bfd *abfd, asection *sec,
				 struct elf_link_hash_entry *h,
				 const Elf_Internal_Rela *rel)
{
  /* I am not sure why the addend isn't allowed.  */
  if (rel->r_addend != 0)
    {
      (*_bfd_error_handler)
	(_("%pB %s: Error: Rom-patch relocation offset: 0x%lx "
	   "with addend 0x%lx\n"),
	 abfd, sec->name, rel->r_offset, rel->r_addend);
      return false;
    }

  if (h)
    {
      andes_ict_hash_t *entry;
      /* First, just check whether the ICT symbol is the hash table.  */
      entry = (andes_ict_hash_t *)
	bfd_hash_lookup (&nds_ict_sta.indirect_call_table, h->root.root.string,
			 false, false);
      if (entry == NULL)
	{
	  /* Create new hash entry.  */
	  entry = (andes_ict_hash_t *)
	    bfd_hash_lookup (&nds_ict_sta.indirect_call_table, h->root.root.string,
			     true, true);
	  if (entry == NULL)
	    {
	      (*_bfd_error_handler)
		(_("%pB: failed to create indirect call %s hash table\n"),
		 abfd, h->root.root.string);
	      return false;
	    }

	  riscv_elf_hash_entry (h)->indirect_call = true;
	  andes_ict_entry_t *it = andes_ict_list_update_symbol (h);
	  entry->entry = it;
	  nds_ict_sta.hash_entries++;
	}
    }
  else
    {
      /* Rom-patch functions cannot be local.  */
      (*_bfd_error_handler)
	(_("%pB: indirect call relocation with local symbol.\n"), abfd);
      return false;
    }

  return true;
}

static bool
riscv_has_subset (struct bfd_link_info *info, const char *subset)
{
  unsigned xlen = ARCH_SIZE;
  riscv_subset_list_t subsets;
  bool ret;

  /* If relax is disabled by user, table jump insn
    will not be generated.  */
  if (info->disable_target_specific_optimizations >= 1)
    return false;

  if (!bfd_link_executable (info))
    return false;

  bfd *obfd = info->output_bfd;
  obj_attribute *out_attr = elf_known_obj_attributes_proc (obfd);

  subsets.head = NULL;
  subsets.tail = NULL;

  riscv_parse_subset_t riscv_rps_ld_out =
	{&subsets, _bfd_error_handler, _bfd_error_handler, &xlen, NULL, false,
	 STATE_DEFAULT, false};

  if (!riscv_parse_subset (&riscv_rps_ld_out, out_attr[Tag_RISCV_arch].s))
    return false;

  ret = riscv_subset_supports (&riscv_rps_ld_out, subset);
  riscv_release_subset_list (&subsets);

  return ret;
}

static bool
riscv_use_table_jump (struct bfd_link_info *info)
{
  unsigned xlen = ARCH_SIZE;
  riscv_subset_list_t subsets;
  bool ret;

  /* If relax is disabled by user, table jump insn
    will not be generated.  */
  if (info->disable_target_specific_optimizations >= 1)
    return false;

  if (!bfd_link_executable (info))
    return false;

  bfd *obfd = info->output_bfd;
  obj_attribute *out_attr = elf_known_obj_attributes_proc (obfd);

  subsets.head = NULL;
  subsets.tail = NULL;

  riscv_parse_subset_t riscv_rps_ld_out =
	{&subsets, _bfd_error_handler, _bfd_error_handler, &xlen, NULL, false,
	 STATE_DEFAULT, false};

  if (!riscv_parse_subset (&riscv_rps_ld_out, out_attr[Tag_RISCV_arch].s))
    return false;

  ret = riscv_subset_supports (&riscv_rps_ld_out, "zcmt");
  riscv_release_subset_list (&subsets);

  return ret;
}

#if 0
static bool
bfd_elf_riscv_make_tablejump_section (bfd *abfd, struct bfd_link_info *info)
{
  asection *sec;
  struct riscv_elf_link_hash_table *htab;

  /* Skip if no Zcmt.  */
  if (!riscv_use_table_jump (info))
    return true;

  htab = riscv_elf_hash_table (info);
  sec = bfd_get_linker_section (abfd, TABLE_JUMP_SEC_NAME);

  if (sec != NULL)
    return true;

  if (htab->table_jump_htab->tablejump_sec == NULL)
    {
      flagword flags = (SEC_CODE | SEC_ALLOC | SEC_LOAD
			| SEC_HAS_CONTENTS | SEC_READONLY
			| SEC_IN_MEMORY | SEC_KEEP
			| SEC_RELOC
			| SEC_LINKER_CREATED /* to skip output of LTO pass.  */
		       );
      sec = bfd_make_section_anyway_with_flags (abfd, TABLE_JUMP_SEC_NAME,
		flags);

      if (sec == NULL
	  || !bfd_set_section_alignment (sec, 6) /* CSR JVT aligns on 2^6.  */
	  || !bfd_set_section_size (sec, 256 * RISCV_ELF_WORD_BYTES))
	return false;
      sec->gc_mark = 1;

      htab->table_jump_htab->tablejump_sec = sec;
      htab->table_jump_htab->tablejump_sec_owner = abfd;
    }

  return true;
}
#endif

/* Look through the relocs for a section during the first phase, and
   allocate space in the global offset table or procedure linkage
   table.  */

static bool
riscv_elf_check_relocs (bfd *abfd, struct bfd_link_info *info,
			asection *sec, const Elf_Internal_Rela *relocs)
{
  struct riscv_elf_link_hash_table *htab;
  Elf_Internal_Shdr *symtab_hdr;
  struct elf_link_hash_entry **sym_hashes;
  const Elf_Internal_Rela *rel;
  asection *sreloc = NULL;
  bool update_ict_hash_collect = false;
  bool update_ict_hash_patch = false; /* TO REVIEW: should be removed.  */

  htab = riscv_elf_hash_table (info);

  static int is_inited = false;
  if (! is_inited) { /*Andes */
    is_inited = true;

    andes_ld_options_t *andes = &htab->andes;
    /* determine table jump enable:
	if not explicitly from CLI, then by the extension "zcmt".  */
    bool ena_zcmt = riscv_use_table_jump (info);
    bool determine = ena_zcmt;
    if (andes->set_table_jump_cli)
      {
	determine = andes->set_table_jump && ena_zcmt;
	if (determine != andes->set_table_jump)
	  {
	    andes->set_table_jump = determine;
	    (*_bfd_error_handler) (
		_("warning: failed to enable table jump! either without zcmt"
		  " or not linking executable.\n"));
	  }
      }
    andes->set_table_jump = (uint) determine;
    /* create section at "riscv_elf_after_check_relocs".  */
  } /* Andes */

  if (bfd_link_relocatable (info))
    return true;

  symtab_hdr = &elf_tdata (abfd)->symtab_hdr;
  sym_hashes = elf_sym_hashes (abfd);

  if (htab->elf.dynobj == NULL)
    htab->elf.dynobj = abfd;

  /* collect R_RISCV_XXX_ICT_XXX relocations to update ict list.  */
  /* special handling for these relocations? 
     R_RISCV_JAL, R_RISCV_CALL, and R_RISCV_64.  */
  //if (!(nds_ict_sta.list_head && nds_ict_sta.list_head->flags & ICT_FLG_LD))
    update_ict_hash_collect = true;
#if 0
  else
    update_ict_hash_patch = true;
#endif

  for (rel = relocs; rel < relocs + sec->reloc_count; rel++)
    {
      unsigned int r_type;
      unsigned int r_symndx;
      struct elf_link_hash_entry *h;

      ((Elf_Internal_Rela *)rel)->r_user = REL_R_USER_FLG_NONE;

      r_symndx = ELFNN_R_SYM (rel->r_info);
      r_type = ELFNN_R_TYPE (rel->r_info);

      if (r_symndx >= NUM_SHDR_ENTRIES (symtab_hdr))
	{
	  (*_bfd_error_handler) (_("%pB: bad symbol index: %d"),
				 abfd, r_symndx);
	  return false;
	}

      if (r_symndx < symtab_hdr->sh_info)
	{
	  /* A local symbol.  */
	  Elf_Internal_Sym *isym = bfd_sym_from_r_symndx (&htab->elf.sym_cache,
							  abfd, r_symndx);
	  if (isym == NULL)
	    return false;

	  /* Check relocation against local STT_GNU_IFUNC symbol.  */
	  if (ELF_ST_TYPE (isym->st_info) == STT_GNU_IFUNC)
	    {
	      h = riscv_elf_get_local_sym_hash (htab, abfd, rel, true);
	      if (h == NULL)
		return false;

	      /* Fake STT_GNU_IFUNC global symbol.  */
	      h->root.root.string = bfd_elf_sym_name (abfd, symtab_hdr,
						      isym, NULL);
	      h->type = STT_GNU_IFUNC;
	      h->def_regular = 1;
	      h->ref_regular = 1;
	      h->forced_local = 1;
	      h->root.type = bfd_link_hash_defined;
	    }
	  else
	    h = NULL;
	}
      else
	{
	  h = sym_hashes[r_symndx - symtab_hdr->sh_info];
	  while (h->root.type == bfd_link_hash_indirect
		 || h->root.type == bfd_link_hash_warning)
	    h = (struct elf_link_hash_entry *) h->root.u.i.link;
	}

      if (h != NULL)
	{
	  switch (r_type)
	    {
	    case R_RISCV_32:
	    case R_RISCV_64:
	    case R_RISCV_CALL:
	    case R_RISCV_CALL_PLT:
	    case R_RISCV_HI20:
	    case R_RISCV_GOT_HI20:
	    case R_RISCV_PCREL_HI20:
	      /* Create the ifunc sections, iplt and ipltgot, for static
		 executables.  */
	      if (h->type == STT_GNU_IFUNC
		  && !_bfd_elf_create_ifunc_sections (htab->elf.dynobj, info))
		return false;
	      break;

	    default:
	      break;
	    }

	  /* It is referenced by a non-shared object.  */
	  h->ref_regular = 1;
	}

      switch (r_type)
	{
	case R_RISCV_TLS_GD_HI20:
	  if (!riscv_elf_record_got_reference (abfd, info, h, r_symndx)
	      || !riscv_elf_record_tls_type (abfd, h, r_symndx, GOT_TLS_GD))
	    return false;
	  break;

	case R_RISCV_TLS_GOT_HI20:
	  if (bfd_link_pic (info))
	    info->flags |= DF_STATIC_TLS;
	  if (!riscv_elf_record_got_reference (abfd, info, h, r_symndx)
	      || !riscv_elf_record_tls_type (abfd, h, r_symndx, GOT_TLS_IE))
	    return false;
	  break;

	case R_RISCV_GOT_HI20:
	  if (!riscv_elf_record_got_reference (abfd, info, h, r_symndx)
	      || !riscv_elf_record_tls_type (abfd, h, r_symndx, GOT_NORMAL))
	    return false;
	  break;

	case R_RISCV_CALL:
	case R_RISCV_CALL_PLT:
	  /* These symbol requires a procedure linkage table entry.
	     We actually build the entry in adjust_dynamic_symbol,
	     because these might be a case of linking PIC code without
	     linking in any dynamic objects, in which case we don't
	     need to generate a procedure linkage table after all.  */

	  /* If it is a local symbol, then we resolve it directly
	     without creating a PLT entry.  */
	  if (h == NULL)
	    continue;

	  h->needs_plt = 1;
	  h->plt.refcount += 1;
	  break;

	case R_RISCV_PCREL_HI20:
	  if (h != NULL
	      && h->type == STT_GNU_IFUNC)
	    {
	      h->non_got_ref = 1;
	      h->pointer_equality_needed = 1;

	      /* We don't use the PCREL_HI20 in the data section,
		 so we always need the plt when it refers to
		 ifunc symbol.  */
	      h->plt.refcount += 1;
	    }
	  /* Fall through.  */

	case R_RISCV_JAL:
	  if (update_ict_hash_patch
	      && !riscv_elf_update_ict_hash_table (abfd, sec, h, rel))
	    return false;
	  /* Fall through.  */

	case R_RISCV_BRANCH:
	case R_RISCV_RVC_BRANCH:
	case R_RISCV_RVC_JUMP:
	  /* In shared libraries and pie, these relocs are known
	     to bind locally.  */
	  ((Elf_Internal_Rela *)rel)->r_user = REL_R_USER_FLG_RAW;
	  if (bfd_link_pic (info))
	    break;
	  goto static_reloc;

	case R_RISCV_TPREL_HI20:
	  if (!bfd_link_executable (info))
	    return bad_static_reloc (abfd, r_type, h);
	  if (h != NULL)
	    riscv_elf_record_tls_type (abfd, h, r_symndx, GOT_TLS_LE);
	  goto static_reloc;

	case R_RISCV_HI20:
	  if (bfd_link_pic (info))
	    return bad_static_reloc (abfd, r_type, h);
	  /* Fall through.  */

	case R_RISCV_64:
	  if (r_type == R_RISCV_64 && update_ict_hash_patch
	      && !riscv_elf_update_ict_hash_table (abfd, sec, h, rel))
	    return false;
	  /* Fall through.  */

	case R_RISCV_COPY:
	case R_RISCV_JUMP_SLOT:
	case R_RISCV_RELATIVE:
	case R_RISCV_32:
	  /* Fall through.  */

	static_reloc:

	  if (h != NULL
	      && (!bfd_link_pic (info)
		  || h->type == STT_GNU_IFUNC))
	    {
	      /* This reloc might not bind locally.  */
	      h->non_got_ref = 1;
	      h->pointer_equality_needed = 1;

	      if (!h->def_regular
		  || (sec->flags & (SEC_CODE | SEC_READONLY)) != 0)
		{
		  /* We may need a .plt entry if the symbol is a function
		     defined in a shared lib or is a function referenced
		     from the code or read-only section.  */
		  h->plt.refcount += 1;
		}
	    }

	  /* If we are creating a shared library, and this is a reloc
	     against a global symbol, or a non PC relative reloc
	     against a local symbol, then we need to copy the reloc
	     into the shared library.  However, if we are linking with
	     -Bsymbolic, we do not need to copy a reloc against a
	     global symbol which is defined in an object we are
	     including in the link (i.e., DEF_REGULAR is set).  At
	     this point we have not seen all the input files, so it is
	     possible that DEF_REGULAR is not set now but will be set
	     later (it is never cleared).  In case of a weak definition,
	     DEF_REGULAR may be cleared later by a strong definition in
	     a shared library.  We account for that possibility below by
	     storing information in the relocs_copied field of the hash
	     table entry.  A similar situation occurs when creating
	     shared libraries and symbol visibility changes render the
	     symbol local.

	     If on the other hand, we are creating an executable, we
	     may need to keep relocations for symbols satisfied by a
	     dynamic library if we manage to avoid copy relocs for the
	     symbol.

	     Generate dynamic pointer relocation against STT_GNU_IFUNC
	     symbol in the non-code section (R_RISCV_32/R_RISCV_64).  */
	  reloc_howto_type * r = riscv_elf_rtype_to_howto (abfd, r_type);

	  if ((bfd_link_pic (info)
	       && (sec->flags & SEC_ALLOC) != 0
	       && ((r != NULL && !r->pc_relative)
		   || (h != NULL
		       && (!info->symbolic
			   || h->root.type == bfd_link_hash_defweak
			   || !h->def_regular))))
	      || (!bfd_link_pic (info)
		  && (sec->flags & SEC_ALLOC) != 0
		  && h != NULL
		  && (h->root.type == bfd_link_hash_defweak
		      || !h->def_regular))
	      || (!bfd_link_pic (info)
		  && h != NULL
		  && h->type == STT_GNU_IFUNC
		  && (sec->flags & SEC_CODE) == 0))
	    {
	      struct elf_dyn_relocs *p;
	      struct elf_dyn_relocs **head;

	      /* When creating a shared object, we must copy these
		 relocs into the output file.  We create a reloc
		 section in dynobj and make room for the reloc.  */
	      if (sreloc == NULL)
		{
		  sreloc = _bfd_elf_make_dynamic_reloc_section
		    (sec, htab->elf.dynobj, RISCV_ELF_LOG_WORD_BYTES,
		    abfd, /*rela?*/ true);

		  if (sreloc == NULL)
		    return false;
		}

	      /* If this is a global symbol, we count the number of
		 relocations we need for this symbol.  */
	      if (h != NULL)
		head = &h->dyn_relocs;
	      else
		{
		  /* Track dynamic relocs needed for local syms too.
		     We really need local syms available to do this
		     easily.  Oh well.  */

		  asection *s;
		  void *vpp;
		  Elf_Internal_Sym *isym;

		  isym = bfd_sym_from_r_symndx (&htab->elf.sym_cache,
						abfd, r_symndx);
		  if (isym == NULL)
		    return false;

		  s = bfd_section_from_elf_index (abfd, isym->st_shndx);
		  if (s == NULL)
		    s = sec;

		  vpp = &elf_section_data (s)->local_dynrel;
		  head = (struct elf_dyn_relocs **) vpp;
		}

	      p = *head;
	      if (p == NULL || p->sec != sec)
		{
		  size_t amt = sizeof *p;
		  p = ((struct elf_dyn_relocs *)
		       bfd_alloc (htab->elf.dynobj, amt));
		  if (p == NULL)
		    return false;
		  p->next = *head;
		  *head = p;
		  p->sec = sec;
		  p->count = 0;
		  p->pc_count = 0;
		}

	      p->count += 1;
	      p->pc_count += r == NULL ? 0 : r->pc_relative;
	    }

	  break;

	case R_RISCV_GNU_VTINHERIT:
	  if (!bfd_elf_gc_record_vtinherit (abfd, sec, h, rel->r_offset))
	    return false;
	  break;

	case R_RISCV_GNU_VTENTRY:
	  if (!bfd_elf_gc_record_vtentry (abfd, sec, h, rel->r_addend))
	    return false;
	  break;

	case R_RISCV_ICT_HI20:
	case R_RISCV_ICT_LO12_I:
	case R_RISCV_PCREL_ICT_HI20:
	case R_RISCV_CALL_ICT:
	case R_RISCV_ICT_64:
	  if (update_ict_hash_collect
	      && !riscv_elf_update_ict_hash_table (abfd, sec, h, rel))
	    return false;
	  break;

	default:
	  break;
	}
    }

  return true;
}

static asection *
riscv_elf_gc_mark_hook (asection *sec,
			struct bfd_link_info *info,
			Elf_Internal_Rela *rel,
			struct elf_link_hash_entry *h,
			Elf_Internal_Sym *sym)
{
  if (h != NULL)
    switch (ELFNN_R_TYPE (rel->r_info))
      {
      case R_RISCV_GNU_VTINHERIT:
      case R_RISCV_GNU_VTENTRY:
	return NULL;
      }

  return _bfd_elf_gc_mark_hook (sec, info, rel, h, sym);
}

/* Adjust a symbol defined by a dynamic object and referenced by a
   regular object.  The current definition is in some section of the
   dynamic object, but we're not including those sections.  We have to
   change the definition to something the rest of the link can
   understand.  */

static bool
riscv_elf_adjust_dynamic_symbol (struct bfd_link_info *info,
				 struct elf_link_hash_entry *h)
{
  struct riscv_elf_link_hash_table *htab;
  struct riscv_elf_link_hash_entry * eh;
  bfd *dynobj;
  asection *s, *srel;

  htab = riscv_elf_hash_table (info);
  BFD_ASSERT (htab != NULL);

  dynobj = htab->elf.dynobj;

  /* Make sure we know what is going on here.  */
  BFD_ASSERT (dynobj != NULL
	      && (h->needs_plt
		  || h->type == STT_GNU_IFUNC
		  || h->is_weakalias
		  || (h->def_dynamic
		      && h->ref_regular
		      && !h->def_regular)));

  /* If this is a function, put it in the procedure linkage table.  We
     will fill in the contents of the procedure linkage table later
     (although we could actually do it here).  */
  if (h->type == STT_FUNC || h->type == STT_GNU_IFUNC || h->needs_plt)
    {
      if (h->plt.refcount <= 0
	  || (h->type != STT_GNU_IFUNC
	      && (SYMBOL_CALLS_LOCAL (info, h)
		  || (ELF_ST_VISIBILITY (h->other) != STV_DEFAULT
		      && h->root.type == bfd_link_hash_undefweak))))
	{
	  /* This case can occur if we saw a R_RISCV_CALL_PLT reloc in an
	     input file, but the symbol was never referred to by a dynamic
	     object, or if all references were garbage collected.  In such
	     a case, we don't actually need to build a PLT entry.  */
	  h->plt.offset = (bfd_vma) -1;
	  h->needs_plt = 0;
	}

      return true;
    }
  else
    h->plt.offset = (bfd_vma) -1;

  /* If this is a weak symbol, and there is a real definition, the
     processor independent code will have arranged for us to see the
     real definition first, and we can just use the same value.  */
  if (h->is_weakalias)
    {
      struct elf_link_hash_entry *def = weakdef (h);
      BFD_ASSERT (def->root.type == bfd_link_hash_defined);
      h->root.u.def.section = def->root.u.def.section;
      h->root.u.def.value = def->root.u.def.value;
      return true;
    }

  /* This is a reference to a symbol defined by a dynamic object which
     is not a function.  */

  /* If we are creating a shared library, we must presume that the
     only references to the symbol are via the global offset table.
     For such cases we need not do anything here; the relocations will
     be handled correctly by relocate_section.  */
  if (bfd_link_pic (info))
    return true;

  /* If there are no references to this symbol that do not use the
     GOT, we don't need to generate a copy reloc.  */
  if (!h->non_got_ref)
    return true;

  /* If -z nocopyreloc was given, we won't generate them either.  */
  if (info->nocopyreloc)
    {
      h->non_got_ref = 0;
      return true;
    }

  /* If we don't find any dynamic relocs in read-only sections, then
     we'll be keeping the dynamic relocs and avoiding the copy reloc.  */
  if (!_bfd_elf_readonly_dynrelocs (h))
    {
      h->non_got_ref = 0;
      return true;
    }

  /* We must allocate the symbol in our .dynbss section, which will
     become part of the .bss section of the executable.  There will be
     an entry for this symbol in the .dynsym section.  The dynamic
     object will contain position independent code, so all references
     from the dynamic object to this symbol will go through the global
     offset table.  The dynamic linker will use the .dynsym entry to
     determine the address it must put in the global offset table, so
     both the dynamic object and the regular object will refer to the
     same memory location for the variable.  */

  /* We must generate a R_RISCV_COPY reloc to tell the dynamic linker
     to copy the initial value out of the dynamic object and into the
     runtime process image.  We need to remember the offset into the
     .rel.bss section we are going to use.  */
  eh = (struct riscv_elf_link_hash_entry *) h;
  if (eh->tls_type & ~GOT_NORMAL)
    {
      s = htab->sdyntdata;
      srel = htab->elf.srelbss;
    }
  else if ((h->root.u.def.section->flags & SEC_READONLY) != 0)
    {
      s = htab->elf.sdynrelro;
      srel = htab->elf.sreldynrelro;
    }
  else
    {
      s = htab->elf.sdynbss;
      srel = htab->elf.srelbss;
    }
  if ((h->root.u.def.section->flags & SEC_ALLOC) != 0 && h->size != 0)
    {
      srel->size += sizeof (ElfNN_External_Rela);
      h->needs_copy = 1;
    }

  return _bfd_elf_adjust_dynamic_copy (info, h, s);
}

/* Allocate space in .plt, .got and associated reloc sections for
   dynamic relocs.  */

static bool
allocate_dynrelocs (struct elf_link_hash_entry *h, void *inf)
{
  struct bfd_link_info *info;
  struct riscv_elf_link_hash_table *htab;
  struct elf_dyn_relocs *p;

  if (h->root.type == bfd_link_hash_indirect)
    return true;

  info = (struct bfd_link_info *) inf;
  htab = riscv_elf_hash_table (info);
  BFD_ASSERT (htab != NULL);

  /* When we are generating pde, make sure gp symbol is output as a
     dynamic symbol.  Then ld.so can set the gp register earlier, before
     resolving the ifunc.  */
  if (!bfd_link_pic (info)
      && htab->elf.dynamic_sections_created
      && strcmp (h->root.root.string, RISCV_GP_SYMBOL) == 0
      && !bfd_elf_link_record_dynamic_symbol (info, h))
    return false;

  /* Since STT_GNU_IFUNC symbols must go through PLT, we handle them
     in the allocate_ifunc_dynrelocs and allocate_local_ifunc_dynrelocs,
     if they are defined and referenced in a non-shared object.  */
  if (h->type == STT_GNU_IFUNC
      && h->def_regular)
    return true;
  else if (htab->elf.dynamic_sections_created
	   && h->plt.refcount > 0)
    {
      /* Make sure this symbol is output as a dynamic symbol.
	 Undefined weak syms won't yet be marked as dynamic.  */
      if (h->dynindx == -1
	  && !h->forced_local)
	{
	  if (! bfd_elf_link_record_dynamic_symbol (info, h))
	    return false;
	}

      if (WILL_CALL_FINISH_DYNAMIC_SYMBOL (1, bfd_link_pic (info), h))
	{
	  asection *s = htab->elf.splt;

	  if (s->size == 0)
	    s->size = PLT_HEADER_SIZE;

	  h->plt.offset = s->size;

	  /* Make room for this entry.  */
	  s->size += PLT_ENTRY_SIZE;

	  /* We also need to make an entry in the .got.plt section.  */
	  htab->elf.sgotplt->size += GOT_ENTRY_SIZE;

	  /* We also need to make an entry in the .rela.plt section.  */
	  htab->elf.srelplt->size += sizeof (ElfNN_External_Rela);

	  /* If this symbol is not defined in a regular file, and we are
	     not generating a shared library, then set the symbol to this
	     location in the .plt.  This is required to make function
	     pointers compare as equal between the normal executable and
	     the shared library.  */
	  if (! bfd_link_pic (info)
	      && !h->def_regular)
	    {
	      h->root.u.def.section = s;
	      h->root.u.def.value = h->plt.offset;
	    }

	  /* If the symbol has STO_RISCV_VARIANT_CC flag, then raise the
	     variant_cc flag of riscv_elf_link_hash_table.  */
	  if (h->other & STO_RISCV_VARIANT_CC)
	    htab->variant_cc = 1;
	}
      else
	{
	  h->plt.offset = (bfd_vma) -1;
	  h->needs_plt = 0;
	}
    }
  else
    {
      h->plt.offset = (bfd_vma) -1;
      h->needs_plt = 0;
    }

  if (h->got.refcount > 0)
    {
      asection *s;
      bool dyn;
      int tls_type = riscv_elf_hash_entry (h)->tls_type;

      /* Make sure this symbol is output as a dynamic symbol.
	 Undefined weak syms won't yet be marked as dynamic.  */
      if (h->dynindx == -1
	  && !h->forced_local)
	{
	  if (! bfd_elf_link_record_dynamic_symbol (info, h))
	    return false;
	}

      s = htab->elf.sgot;
      h->got.offset = s->size;
      dyn = htab->elf.dynamic_sections_created;
      if (tls_type & (GOT_TLS_GD | GOT_TLS_IE))
	{
	  /* TLS_GD needs two dynamic relocs and two GOT slots.  */
	  if (tls_type & GOT_TLS_GD)
	    {
	      s->size += 2 * RISCV_ELF_WORD_BYTES;
	      htab->elf.srelgot->size += 2 * sizeof (ElfNN_External_Rela);
	    }

	  /* TLS_IE needs one dynamic reloc and one GOT slot.  */
	  if (tls_type & GOT_TLS_IE)
	    {
	      s->size += RISCV_ELF_WORD_BYTES;
	      htab->elf.srelgot->size += sizeof (ElfNN_External_Rela);
	    }
	}
      else
	{
	  s->size += RISCV_ELF_WORD_BYTES;
	  if (WILL_CALL_FINISH_DYNAMIC_SYMBOL (dyn, bfd_link_pic (info), h)
	      && ! UNDEFWEAK_NO_DYNAMIC_RELOC (info, h))
	    htab->elf.srelgot->size += sizeof (ElfNN_External_Rela);
	}
    }
  else
    h->got.offset = (bfd_vma) -1;

  if (h->dyn_relocs == NULL)
    return true;

  /* In the shared -Bsymbolic case, discard space allocated for
     dynamic pc-relative relocs against symbols which turn out to be
     defined in regular objects.  For the normal shared case, discard
     space for pc-relative relocs that have become local due to symbol
     visibility changes.  */

  if (bfd_link_pic (info))
    {
      if (SYMBOL_CALLS_LOCAL (info, h))
	{
	  struct elf_dyn_relocs **pp;

	  for (pp = &h->dyn_relocs; (p = *pp) != NULL; )
	    {
	      p->count -= p->pc_count;
	      p->pc_count = 0;
	      if (p->count == 0)
		*pp = p->next;
	      else
		pp = &p->next;
	    }
	}

      /* Also discard relocs on undefined weak syms with non-default
	 visibility.  */
      if (h->dyn_relocs != NULL
	  && h->root.type == bfd_link_hash_undefweak)
	{
	  if (ELF_ST_VISIBILITY (h->other) != STV_DEFAULT
	      || UNDEFWEAK_NO_DYNAMIC_RELOC (info, h))
	    h->dyn_relocs = NULL;

	  /* Make sure undefined weak symbols are output as a dynamic
	     symbol in PIEs.  */
	  else if (h->dynindx == -1
		   && !h->forced_local)
	    {
	      if (! bfd_elf_link_record_dynamic_symbol (info, h))
		return false;
	    }
	}
    }
  else
    {
      /* For the non-shared case, discard space for relocs against
	 symbols which turn out to need copy relocs or are not
	 dynamic.  */

      if (!h->non_got_ref
	  && ((h->def_dynamic
	       && !h->def_regular)
	      || (htab->elf.dynamic_sections_created
		  && (h->root.type == bfd_link_hash_undefweak
		      || h->root.type == bfd_link_hash_undefined))))
	{
	  /* Make sure this symbol is output as a dynamic symbol.
	     Undefined weak syms won't yet be marked as dynamic.  */
	  if (h->dynindx == -1
	      && !h->forced_local)
	    {
	      if (! bfd_elf_link_record_dynamic_symbol (info, h))
		return false;
	    }

	  /* If that succeeded, we know we'll be keeping all the
	     relocs.  */
	  if (h->dynindx != -1)
	    goto keep;
	}

      h->dyn_relocs = NULL;

    keep: ;
    }

  /* Finally, allocate space.  */
  for (p = h->dyn_relocs; p != NULL; p = p->next)
    {
      asection *sreloc = elf_section_data (p->sec)->sreloc;
      sreloc->size += p->count * sizeof (ElfNN_External_Rela);
    }

  return true;
}

/* Allocate space in .plt, .got and associated reloc sections for
   ifunc dynamic relocs.  */

static bool
allocate_ifunc_dynrelocs (struct elf_link_hash_entry *h,
			  void *inf)
{
  struct bfd_link_info *info;

  if (h->root.type == bfd_link_hash_indirect)
    return true;

  if (h->root.type == bfd_link_hash_warning)
    h = (struct elf_link_hash_entry *) h->root.u.i.link;

  info = (struct bfd_link_info *) inf;

  /* Since STT_GNU_IFUNC symbol must go through PLT, we handle it
     here if it is defined and referenced in a non-shared object.  */
  if (h->type == STT_GNU_IFUNC
      && h->def_regular)
    return _bfd_elf_allocate_ifunc_dyn_relocs (info, h,
					       &h->dyn_relocs,
					       PLT_ENTRY_SIZE,
					       PLT_HEADER_SIZE,
					       GOT_ENTRY_SIZE,
					       true);
  return true;
}

/* Allocate space in .plt, .got and associated reloc sections for
   local ifunc dynamic relocs.  */

static int
allocate_local_ifunc_dynrelocs (void **slot, void *inf)
{
  struct elf_link_hash_entry *h
    = (struct elf_link_hash_entry *) *slot;

  if (h->type != STT_GNU_IFUNC
      || !h->def_regular
      || !h->ref_regular
      || !h->forced_local
      || h->root.type != bfd_link_hash_defined)
    abort ();

  return allocate_ifunc_dynrelocs (h, inf);
}

static bool
riscv_elf_size_dynamic_sections (bfd *output_bfd, struct bfd_link_info *info)
{
  struct riscv_elf_link_hash_table *htab;
  bfd *dynobj;
  asection *s;
  bfd *ibfd;

  htab = riscv_elf_hash_table (info);
  BFD_ASSERT (htab != NULL);
  dynobj = htab->elf.dynobj;
  BFD_ASSERT (dynobj != NULL);

  if (elf_hash_table (info)->dynamic_sections_created)
    {
      /* Set the contents of the .interp section to the interpreter.  */
      if (bfd_link_executable (info) && !info->nointerp)
	{
	  s = bfd_get_linker_section (dynobj, ".interp");
	  BFD_ASSERT (s != NULL);
	  s->size = strlen (ELFNN_DYNAMIC_INTERPRETER) + 1;
	  s->contents = (unsigned char *) ELFNN_DYNAMIC_INTERPRETER;
	}
    }

  struct bfd_link_hash_entry *bh = NULL;

  if (riscv_use_table_jump (info)
      && htab->table_jump_htab->tablejump_sec)
    {
      s = htab->table_jump_htab->tablejump_sec;

      BFD_ASSERT (s != NULL);
      BFD_ASSERT (s->contents);

      if (s->contents == NULL)
	s->contents = (bfd_byte *) bfd_zalloc (
	  htab->table_jump_htab->tablejump_sec_owner, s->size);

      if (s->contents == NULL)
	return false;

      if (s->output_section == NULL)
	return false;
    }
  else
    s = bfd_abs_section_ptr;

  if (!_bfd_generic_link_add_one_symbol (info,
		  output_bfd,
		  RISCV_TABLE_JUMP_BASE_SYMBOL, BSF_GLOBAL,
		  s, (bfd_vma) 0, (const char *) NULL, true,
		  get_elf_backend_data (output_bfd)->collect, &bh))
    return false;

  /* Set up .got offsets for local syms, and space for local dynamic
     relocs.  */
  for (ibfd = info->input_bfds; ibfd != NULL; ibfd = ibfd->link.next)
    {
      bfd_signed_vma *local_got;
      bfd_signed_vma *end_local_got;
      char *local_tls_type;
      bfd_size_type locsymcount;
      Elf_Internal_Shdr *symtab_hdr;
      asection *srel;

      if (! is_riscv_elf (ibfd))
	continue;

      for (s = ibfd->sections; s != NULL; s = s->next)
	{
	  struct elf_dyn_relocs *p;

	  for (p = elf_section_data (s)->local_dynrel; p != NULL; p = p->next)
	    {
	      if (!bfd_is_abs_section (p->sec)
		  && bfd_is_abs_section (p->sec->output_section))
		{
		  /* Input section has been discarded, either because
		     it is a copy of a linkonce section or due to
		     linker script /DISCARD/, so we'll be discarding
		     the relocs too.  */
		}
	      else if (p->count != 0)
		{
		  srel = elf_section_data (p->sec)->sreloc;
		  srel->size += p->count * sizeof (ElfNN_External_Rela);
		  if ((p->sec->output_section->flags & SEC_READONLY) != 0)
		    info->flags |= DF_TEXTREL;
		}
	    }
	}

      local_got = elf_local_got_refcounts (ibfd);
      if (!local_got)
	continue;

      symtab_hdr = &elf_symtab_hdr (ibfd);
      locsymcount = symtab_hdr->sh_info;
      end_local_got = local_got + locsymcount;
      local_tls_type = _bfd_riscv_elf_local_got_tls_type (ibfd);
      s = htab->elf.sgot;
      srel = htab->elf.srelgot;
      for (; local_got < end_local_got; ++local_got, ++local_tls_type)
	{
	  if (*local_got > 0)
	    {
	      *local_got = s->size;
	      s->size += RISCV_ELF_WORD_BYTES;
	      if (*local_tls_type & GOT_TLS_GD)
		s->size += RISCV_ELF_WORD_BYTES;
	      if (bfd_link_pic (info)
		  || (*local_tls_type & (GOT_TLS_GD | GOT_TLS_IE)))
		srel->size += sizeof (ElfNN_External_Rela);
	    }
	  else
	    *local_got = (bfd_vma) -1;
	}
    }

  /* Allocate .plt and .got entries and space dynamic relocs for
     global symbols.  */
  elf_link_hash_traverse (&htab->elf, allocate_dynrelocs, info);

  /* Allocate .plt and .got entries and space dynamic relocs for
     global ifunc symbols.  */
  elf_link_hash_traverse (&htab->elf, allocate_ifunc_dynrelocs, info);

  /* Allocate .plt and .got entries and space dynamic relocs for
     local ifunc symbols.  */
  htab_traverse (htab->loc_hash_table, allocate_local_ifunc_dynrelocs, info);

  /* Used to resolve the dynamic relocs overwite problems when
     generating static executable.  */
  if (htab->elf.irelplt)
    htab->last_iplt_index = htab->elf.irelplt->reloc_count - 1;

  if (htab->elf.sgotplt)
    {
      struct elf_link_hash_entry *got;
      got = elf_link_hash_lookup (elf_hash_table (info),
				  "_GLOBAL_OFFSET_TABLE_",
				  false, false, false);

      /* Don't allocate .got.plt section if there are no GOT nor PLT
	 entries and there is no refeence to _GLOBAL_OFFSET_TABLE_.  */
      if ((got == NULL
	   || !got->ref_regular_nonweak)
	  && (htab->elf.sgotplt->size == GOTPLT_HEADER_SIZE)
	  && (htab->elf.splt == NULL
	      || htab->elf.splt->size == 0)
	  && (htab->elf.sgot == NULL
	      || (htab->elf.sgot->size
		  == get_elf_backend_data (output_bfd)->got_header_size)))
	htab->elf.sgotplt->size = 0;
    }

  /* The check_relocs and adjust_dynamic_symbol entry points have
     determined the sizes of the various dynamic sections.  Allocate
     memory for them.  */
  for (s = dynobj->sections; s != NULL; s = s->next)
    {
      if ((s->flags & SEC_LINKER_CREATED) == 0)
	continue;

      if (s == htab->elf.splt
	  || s == htab->elf.sgot
	  || s == htab->elf.sgotplt
	  || s == htab->elf.iplt
	  || s == htab->elf.igotplt
	  || s == htab->elf.sdynbss
	  || s == htab->elf.sdynrelro
	  || s == htab->sdyntdata)
	{
	  /* Strip this section if we don't need it; see the
	     comment below.  */
	}
      else if (startswith (s->name, ".rela"))
	{
	  if (s->size != 0)
	    {
	      /* We use the reloc_count field as a counter if we need
		 to copy relocs into the output file.  */
	      s->reloc_count = 0;
	    }
	}
      else
	{
	  /* It's not one of our sections.  */
	  continue;
	}

      if (s->size == 0)
	{
	  /* If we don't need this section, strip it from the
	     output file.  This is mostly to handle .rela.bss and
	     .rela.plt.  We must create both sections in
	     create_dynamic_sections, because they must be created
	     before the linker maps input sections to output
	     sections.  The linker does that before
	     adjust_dynamic_symbol is called, and it is that
	     function which decides whether anything needs to go
	     into these sections.  */
	  s->flags |= SEC_EXCLUDE;
	  continue;
	}

      if ((s->flags & SEC_HAS_CONTENTS) == 0)
	continue;

      /* Allocate memory for the section contents.  Zero the memory
	 for the benefit of .rela.plt, which has 4 unused entries
	 at the beginning, and we don't want garbage.  */
      s->contents = (bfd_byte *) bfd_zalloc (dynobj, s->size);
      if (s->contents == NULL)
	return false;
    }

  /* Add dynamic entries.  */
  if (elf_hash_table (info)->dynamic_sections_created)
    {
      if (!_bfd_elf_add_dynamic_tags (output_bfd, info, true))
	return false;

      if (htab->variant_cc
	  && !_bfd_elf_add_dynamic_entry (info, DT_RISCV_VARIANT_CC, 0))
       return false;
    }

  return true;
}

#define TP_OFFSET 0
#define DTP_OFFSET 0x800

/* Return the relocation value for a TLS dtp-relative reloc.  */

static bfd_vma
dtpoff (struct bfd_link_info *info, bfd_vma address)
{
  /* If tls_sec is NULL, we should have signalled an error already.  */
  if (elf_hash_table (info)->tls_sec == NULL)
    return 0;
  return address - elf_hash_table (info)->tls_sec->vma - DTP_OFFSET;
}

/* Return the relocation value for a static TLS tp-relative relocation.  */

static bfd_vma
tpoff (struct bfd_link_info *info, bfd_vma address)
{
  /* If tls_sec is NULL, we should have signalled an error already.  */
  if (elf_hash_table (info)->tls_sec == NULL)
    return 0;
  return address - elf_hash_table (info)->tls_sec->vma - TP_OFFSET;
}

/* Return the global pointer's value, or 0 if it is not in use.  */

static bfd_vma
riscv_global_pointer_value (struct bfd_link_info *info)
{
  struct bfd_link_hash_entry *h;

  h = bfd_link_hash_lookup (info->hash, RISCV_GP_SYMBOL, false, false, true);
  if (h == NULL || h->type != bfd_link_hash_defined)
    return 0;

  return h->u.def.value + sec_addr (h->u.def.section);
}

/* Emplace a static relocation.  */

static bfd_reloc_status_type
perform_relocation (const reloc_howto_type *howto,
		    const Elf_Internal_Rela *rel,
		    bfd_vma value,
		    asection *input_section,
		    bfd *input_bfd,
		    bfd_byte *contents)
{
  if (howto->pc_relative)
    value -= sec_addr (input_section) + rel->r_offset;
  value += rel->r_addend;

  switch (ELFNN_R_TYPE (rel->r_info))
    {
    case R_RISCV_HI20:
    case R_RISCV_TPREL_HI20:
    case R_RISCV_PCREL_HI20:
    case R_RISCV_GOT_HI20:
    case R_RISCV_TLS_GOT_HI20:
    case R_RISCV_TLS_GD_HI20:
    case R_RISCV_ICT_HI20:
    case R_RISCV_PCREL_ICT_HI20:
      if (ARCH_SIZE > 32 && !VALID_UTYPE_IMM (RISCV_CONST_HIGH_PART (value)))
	return bfd_reloc_overflow;
      value = ENCODE_UTYPE_IMM (RISCV_CONST_HIGH_PART (value));
      break;

    case R_RISCV_LO12_I:
    case R_RISCV_GPREL_I:
    case R_RISCV_TPREL_LO12_I:
    case R_RISCV_TPREL_I:
    case R_RISCV_PCREL_LO12_I:
    case R_RISCV_ICT_LO12_I:
      value = ENCODE_ITYPE_IMM (value);
      break;

    case R_RISCV_LO12_S:
    case R_RISCV_GPREL_S:
    case R_RISCV_TPREL_LO12_S:
    case R_RISCV_TPREL_S:
    case R_RISCV_PCREL_LO12_S:
      value = ENCODE_STYPE_IMM (value);
      break;

    case R_RISCV_CALL:
    case R_RISCV_CALL_PLT:
    case R_RISCV_CALL_ICT: /* Andes */
      if (ARCH_SIZE > 32 && !VALID_UTYPE_IMM (RISCV_CONST_HIGH_PART (value)))
	return bfd_reloc_overflow;
      value = ENCODE_UTYPE_IMM (RISCV_CONST_HIGH_PART (value))
	      | (ENCODE_ITYPE_IMM (value) << 32);
      break;

    case R_RISCV_JAL:
      if (!VALID_JTYPE_IMM (value))
	return bfd_reloc_overflow;
      value = ENCODE_JTYPE_IMM (value);
      break;

    case R_RISCV_BRANCH:
      if (!VALID_BTYPE_IMM (value))
	return bfd_reloc_overflow;
      value = ENCODE_BTYPE_IMM (value);
      break;

    case R_RISCV_RVC_BRANCH:
      if (!VALID_CBTYPE_IMM (value))
	return bfd_reloc_overflow;
      value = ENCODE_CBTYPE_IMM (value);
      break;

    case R_RISCV_TABLE_JUMP:
      return bfd_reloc_ok;

    case R_RISCV_RVC_JUMP:
      if (!VALID_CJTYPE_IMM (value))
	return bfd_reloc_overflow;
      value = ENCODE_CJTYPE_IMM (value);
      break;

    case R_RISCV_RVC_LUI:
      if (RISCV_CONST_HIGH_PART (value) == 0)
	{
	  /* Linker relaxation can convert an address equal to or greater than
	     0x800 to slightly below 0x800.  C.LUI does not accept zero as a
	     valid immediate.  We can fix this by converting it to a C.LI.  */
	  bfd_vma insn = riscv_get_insn (howto->bitsize,
					 contents + rel->r_offset);
	  insn = (insn & ~MATCH_C_LUI) | MATCH_C_LI;
	  riscv_put_insn (howto->bitsize, insn, contents + rel->r_offset);
	  value = ENCODE_CITYPE_IMM (0);
	}
      else if (!VALID_CITYPE_LUI_IMM (RISCV_CONST_HIGH_PART (value)))
	return bfd_reloc_overflow;
      else
	value = ENCODE_CITYPE_LUI_IMM (RISCV_CONST_HIGH_PART (value));
      break;

    case R_RISCV_32:
    case R_RISCV_64:
    case R_RISCV_ADD8:
    case R_RISCV_ADD16:
    case R_RISCV_ADD32:
    case R_RISCV_ADD64:
    case R_RISCV_SUB6:
    case R_RISCV_SUB8:
    case R_RISCV_SUB16:
    case R_RISCV_SUB32:
    case R_RISCV_SUB64:
    case R_RISCV_SET6:
    case R_RISCV_SET8:
    case R_RISCV_SET16:
    case R_RISCV_SET32:
    case R_RISCV_32_PCREL:
    case R_RISCV_TLS_DTPREL32:
    case R_RISCV_TLS_DTPREL64:
      break;

    case R_RISCV_DELETE:
      return bfd_reloc_ok;

    /* { Andes  */
    case R_RISCV_10_PCREL:
      if (!VALID_SBTYPE_IMM (value))
	return bfd_reloc_overflow;
      value = ENCODE_STYPE_IMM10 (value);
      break;

    case R_RISCV_LGP18S0:
      if (!VALID_GPTYPE_LB_IMM (value))
	return bfd_reloc_overflow;
      value = ENCODE_GPTYPE_LB_IMM (value);
      break;

    case R_RISCV_LGP17S1:
      if (!VALID_GPTYPE_LH_IMM (value))
	return bfd_reloc_overflow;
      value = ENCODE_GPTYPE_LH_IMM (value);
      break;

    case R_RISCV_LGP17S2:
      if (!VALID_GPTYPE_LW_IMM (value))
	return bfd_reloc_overflow;
      value = ENCODE_GPTYPE_LW_IMM (value);
      break;

    case R_RISCV_LGP17S3:
      if (!VALID_GPTYPE_LD_IMM (value))
	return bfd_reloc_overflow;
      value = ENCODE_GPTYPE_LD_IMM (value);
      break;

    case R_RISCV_SGP18S0:
      if (!VALID_GPTYPE_SB_IMM (value))
	return bfd_reloc_overflow;
      value = ENCODE_GPTYPE_SB_IMM (value);
      break;

    case R_RISCV_SGP17S1:
      if (!VALID_GPTYPE_SH_IMM (value))
	return bfd_reloc_overflow;
      value = ENCODE_GPTYPE_SH_IMM (value);
      break;

    case R_RISCV_SGP17S2:
      if (!VALID_GPTYPE_SW_IMM (value))
	return bfd_reloc_overflow;
      value = ENCODE_GPTYPE_SW_IMM (value);
      break;

    case R_RISCV_SGP17S3:
      if (!VALID_GPTYPE_SD_IMM (value))
	return bfd_reloc_overflow;
      value = ENCODE_GPTYPE_SD_IMM (value);
      break;

    case R_RISCV_ANDES_TAG:
    case R_RISCV_EXECIT_ITE:
      BFD_ASSERT (0); /* shall be all cleared in relaxation phases  */
      break;

    case R_RISCV_ICT_64:
      break;
    /* } Andes  */

    default:
      return bfd_reloc_notsupported;
    }

  bfd_vma word;
  if (riscv_is_insn_reloc (howto))
    word = riscv_get_insn (howto->bitsize, contents + rel->r_offset);
  else
    word = bfd_get (howto->bitsize, input_bfd, contents + rel->r_offset);
  word = (word & ~howto->dst_mask) | (value & howto->dst_mask);
  if (riscv_is_insn_reloc (howto))
    riscv_put_insn (howto->bitsize, word, contents + rel->r_offset);
  else
    bfd_put (howto->bitsize, input_bfd, word, contents + rel->r_offset);

  return bfd_reloc_ok;
}

/* Remember all PC-relative high-part relocs we've encountered to help us
   later resolve the corresponding low-part relocs.  */

typedef struct
{
  /* PC value.  */
  bfd_vma address;
  /* Relocation value with addend.  */
  bfd_vma value;
  /* Original reloc type.  */
  int type;
} riscv_pcrel_hi_reloc;

typedef struct riscv_pcrel_lo_reloc
{
  /* PC value of auipc.  */
  bfd_vma address;
  /* Internal relocation.  */
  const Elf_Internal_Rela *reloc;
  /* Record the following information helps to resolve the %pcrel
     which cross different input section.  For now we build a hash
     for pcrel at the start of riscv_elf_relocate_section, and then
     free the hash at the end.  But riscv_elf_relocate_section only
     handles an input section at a time, so that means we can only
     resolve the %pcrel_hi and %pcrel_lo which are in the same input
     section.  Otherwise, we will report dangerous relocation errors
     for those %pcrel which are not in the same input section.  */
  asection *input_section;
  struct bfd_link_info *info;
  reloc_howto_type *howto;
  bfd_byte *contents;
  /* The next riscv_pcrel_lo_reloc.  */
  struct riscv_pcrel_lo_reloc *next;
} riscv_pcrel_lo_reloc;

typedef struct
{
  /* Hash table for riscv_pcrel_hi_reloc.  */
  htab_t hi_relocs;
  /* Linked list for riscv_pcrel_lo_reloc.  */
  riscv_pcrel_lo_reloc *lo_relocs;
} riscv_pcrel_relocs;

static hashval_t
riscv_pcrel_reloc_hash (const void *entry)
{
  const riscv_pcrel_hi_reloc *e = entry;
  return (hashval_t)(e->address >> 2);
}

static int
riscv_pcrel_reloc_eq (const void *entry1, const void *entry2)
{
  const riscv_pcrel_hi_reloc *e1 = entry1, *e2 = entry2;
  return e1->address == e2->address;
}

static bool
riscv_init_pcrel_relocs (riscv_pcrel_relocs *p)
{
  p->lo_relocs = NULL;
  p->hi_relocs = htab_create (1024, riscv_pcrel_reloc_hash,
			      riscv_pcrel_reloc_eq, free);
  return p->hi_relocs != NULL;
}

static void
riscv_free_pcrel_relocs (riscv_pcrel_relocs *p)
{
  riscv_pcrel_lo_reloc *cur = p->lo_relocs;

  while (cur != NULL)
    {
      riscv_pcrel_lo_reloc *next = cur->next;
      free (cur);
      cur = next;
    }

  htab_delete (p->hi_relocs);
}

static bool
riscv_zero_pcrel_hi_reloc (Elf_Internal_Rela *rel,
			   struct bfd_link_info *info,
			   bfd_vma pc,
			   bfd_vma addr,
			   bfd_byte *contents,
			   const reloc_howto_type *howto)
{
  /* We may need to reference low addreses in PC-relative modes even when the
     PC is far away from these addresses.  For example, undefweak references
     need to produce the address 0 when linked.  As 0 is far from the arbitrary
     addresses that we can link PC-relative programs at, the linker can't
     actually relocate references to those symbols.  In order to allow these
     programs to work we simply convert the PC-relative auipc sequences to
     0-relative lui sequences.  */
  if (bfd_link_pic (info))
    return false;

  /* If it's possible to reference the symbol using auipc we do so, as that's
     more in the spirit of the PC-relative relocations we're processing.  */
  bfd_vma offset = addr - pc;
  if (ARCH_SIZE == 32 || VALID_UTYPE_IMM (RISCV_CONST_HIGH_PART (offset)))
    return false;

  /* If it's impossible to reference this with a LUI-based offset then don't
     bother to convert it at all so users still see the PC-relative relocation
     in the truncation message.  */
  if (ARCH_SIZE > 32 && !VALID_UTYPE_IMM (RISCV_CONST_HIGH_PART (addr)))
    return false;

  rel->r_info = ELFNN_R_INFO (addr, R_RISCV_HI20);

  bfd_vma insn = riscv_get_insn (howto->bitsize, contents + rel->r_offset);
  insn = (insn & ~MASK_AUIPC) | MATCH_LUI;
  riscv_put_insn (howto->bitsize, insn, contents + rel->r_offset);
  return true;
}

static bool
riscv_record_pcrel_hi_reloc (riscv_pcrel_relocs *p,
			     bfd_vma addr,
			     bfd_vma value,
			     int type,
			     bool absolute)
{
  bfd_vma offset = absolute ? value : value - addr;
  riscv_pcrel_hi_reloc entry = {addr, offset, type};
  riscv_pcrel_hi_reloc **slot =
    (riscv_pcrel_hi_reloc **) htab_find_slot (p->hi_relocs, &entry, INSERT);

  BFD_ASSERT (*slot == NULL);
  *slot = (riscv_pcrel_hi_reloc *) bfd_malloc (sizeof (riscv_pcrel_hi_reloc));
  if (*slot == NULL)
    return false;
  **slot = entry;
  return true;
}

static bool
riscv_record_pcrel_lo_reloc (riscv_pcrel_relocs *p,
			     bfd_vma addr,
			     const Elf_Internal_Rela *reloc,
			     asection *input_section,
			     struct bfd_link_info *info,
			     reloc_howto_type *howto,
			     bfd_byte *contents)
{
  riscv_pcrel_lo_reloc *entry;
  entry = (riscv_pcrel_lo_reloc *) bfd_malloc (sizeof (riscv_pcrel_lo_reloc));
  if (entry == NULL)
    return false;
  *entry = (riscv_pcrel_lo_reloc) {addr, reloc, input_section, info,
				   howto, contents, p->lo_relocs};
  p->lo_relocs = entry;
  return true;
}

static bool
riscv_resolve_pcrel_lo_relocs (riscv_pcrel_relocs *p)
{
  riscv_pcrel_lo_reloc *r;

  for (r = p->lo_relocs; r != NULL; r = r->next)
    {
      bfd *input_bfd = r->input_section->owner;

      riscv_pcrel_hi_reloc search = {r->address, 0, 0};
      riscv_pcrel_hi_reloc *entry = htab_find (p->hi_relocs, &search);
      /* There may be a risk if the %pcrel_lo with addend refers to
	 an IFUNC symbol.  The %pcrel_hi has been relocated to plt,
	 so the corresponding %pcrel_lo with addend looks wrong.  */
      char *string = NULL;
      if (entry == NULL)
	string = _("%pcrel_lo missing matching %pcrel_hi");
      else if (entry->type == R_RISCV_GOT_HI20
	       && r->reloc->r_addend != 0)
	string = _("%pcrel_lo with addend isn't allowed for R_RISCV_GOT_HI20");
      else if (RISCV_CONST_HIGH_PART (entry->value)
	       != RISCV_CONST_HIGH_PART (entry->value + r->reloc->r_addend))
	{
	  /* Check the overflow when adding reloc addend.  */
	  if (asprintf (&string,
			_("%%pcrel_lo overflow with an addend, the "
			  "value of %%pcrel_hi is 0x%" PRIx64 " without "
			  "any addend, but may be 0x%" PRIx64 " after "
			  "adding the %%pcrel_lo addend"),
			(int64_t) RISCV_CONST_HIGH_PART (entry->value),
			(int64_t) RISCV_CONST_HIGH_PART
				(entry->value + r->reloc->r_addend)) == -1)
	    string = _("%pcrel_lo overflow with an addend");
	}

      if (string != NULL)
	{
	  (*r->info->callbacks->reloc_dangerous)
	    (r->info, string, input_bfd, r->input_section, r->reloc->r_offset);
	  return true;
	}

      perform_relocation (r->howto, r->reloc, entry->value, r->input_section,
			  input_bfd, r->contents);
    }

  return true;
}

/* Relocate a RISC-V ELF section.

   The RELOCATE_SECTION function is called by the new ELF backend linker
   to handle the relocations for a section.

   The relocs are always passed as Rela structures.

   This function is responsible for adjusting the section contents as
   necessary, and (if generating a relocatable output file) adjusting
   the reloc addend as necessary.

   This function does not have to worry about setting the reloc
   address or the reloc symbol index.

   LOCAL_SYMS is a pointer to the swapped in local symbols.

   LOCAL_SECTIONS is an array giving the section in the input file
   corresponding to the st_shndx field of each local symbol.

   The global hash table entry for the global symbols can be found
   via elf_sym_hashes (input_bfd).

   When generating relocatable output, this function must handle
   STB_LOCAL/STT_SECTION symbols specially.  The output symbol is
   going to be the section symbol corresponding to the output
   section, which means that the addend must be adjusted
   accordingly.  */

static int
riscv_elf_relocate_section (bfd *output_bfd,
			    struct bfd_link_info *info,
			    bfd *input_bfd,
			    asection *input_section,
			    bfd_byte *contents,
			    Elf_Internal_Rela *relocs,
			    Elf_Internal_Sym *local_syms,
			    asection **local_sections)
{
  static bool is_ict_fini = false;
  Elf_Internal_Rela *rel;
  Elf_Internal_Rela *relend;
  riscv_pcrel_relocs pcrel_relocs;
  bool ret = false;
  struct riscv_elf_link_hash_table *htab = riscv_elf_hash_table (info);
  Elf_Internal_Shdr *symtab_hdr = &elf_symtab_hdr (input_bfd);
  struct elf_link_hash_entry **sym_hashes = elf_sym_hashes (input_bfd);
  bfd_vma *local_got_offsets = elf_local_got_offsets (input_bfd);
  bool absolute;

  if (!riscv_init_pcrel_relocs (&pcrel_relocs))
    return false;

  /* { Andes */
  andes_ld_options_t *andes = &htab->andes;
  if (execit.is_itb_base_set == 0)
    { /* Set the _ITB_BASE_.  */
      if (! execit_set_itb_base (info))
	{
	  (*_bfd_error_handler) (_("%pB: error: Cannot set _ITB_BASE_"),
				 output_bfd);
	  bfd_set_error (bfd_error_bad_value);
	}

    }

  if (execit.is_itable_relocated == 0)
    { /* init itable before relocating ITE.  */
      if (andes->target_optimization & RISCV_RELAX_EXECIT_ON)
	andes_execit_relocate_itable (info);
      execit.is_itable_relocated = 1;
    }

  /* if ict table existed, then assuming all jobs have done.  */
  if (!is_ict_fini && !find_imported_ict_table && nds_ict_sta.list_len)
    {
      andes_complement_ict_symbols (info);
      andes_relocate_ict_table (info, output_bfd);
    }
  is_ict_fini = true;
  /* } Andes */

  relend = relocs + input_section->reloc_count;
  for (rel = relocs; rel < relend; rel++)
    {
      unsigned long r_symndx;
      struct elf_link_hash_entry *h;
      Elf_Internal_Sym *sym;
      asection *sec;
      bfd_vma relocation;
      bfd_reloc_status_type r = bfd_reloc_ok;
      const char *name = NULL;
      bfd_vma off, ie_off;
      bool unresolved_reloc, is_ie = false;
      bfd_vma pc = sec_addr (input_section) + rel->r_offset;
      int r_type = ELFNN_R_TYPE (rel->r_info), tls_type;
      reloc_howto_type *howto = riscv_elf_rtype_to_howto (input_bfd, r_type);
      const char *msg = NULL;
      char *msg_buf = NULL;
      bool resolved_to_zero;
      bool is_execited = 0;
      andes_irelx_t *relx = NULL;
      // Elf_Internal_Rela relt;

      if (howto == NULL
	  /* { Andes */
	  || r_type == R_RISCV_RELAX_ENTRY
	  || r_type == R_RISCV_DATA
	  //|| r_type == R_RISCV_ANDES_TAG
	  || r_type == R_RISCV_RELAX_REGION_BEGIN
	  || r_type == R_RISCV_RELAX_REGION_END
	  || r_type == R_RISCV_NO_RVC_REGION_BEGIN
	  || r_type == R_RISCV_NO_RVC_REGION_END
	  /* } Andes */
	  || r_type == R_RISCV_GNU_VTINHERIT || r_type == R_RISCV_GNU_VTENTRY)
	continue;

      /* { Andes */
      if (r_type == R_RISCV_ANDES_TAG)
	{ /* restore rel for relocation solving. */
	  relx = (andes_irelx_t *) rel->r_user;
	  //relt.r_info = rel->r_info;
	  rel->r_info = relx->saved_irel.r_info;
	  r_type = ELFNN_R_TYPE (rel->r_info);
	  howto = riscv_elf_rtype_to_howto (input_bfd, r_type);
	}
      /* } Andes */

      /* This is a final link.  */
      r_symndx = ELFNN_R_SYM (rel->r_info);
      h = NULL;
      sym = NULL;
      sec = NULL;
      unresolved_reloc = false;
      if (r_symndx < symtab_hdr->sh_info)
	{
	  sym = local_syms + r_symndx;
	  sec = local_sections[r_symndx];
	  relocation = _bfd_elf_rela_local_sym (output_bfd, sym, &sec, rel);

	  /* Relocate against local STT_GNU_IFUNC symbol.  */
	  if (!bfd_link_relocatable (info)
	      && ELF_ST_TYPE (sym->st_info) == STT_GNU_IFUNC)
	    {
	      h = riscv_elf_get_local_sym_hash (htab, input_bfd, rel, false);
	      if (h == NULL)
		abort ();

	      /* Set STT_GNU_IFUNC symbol value.  */
	      h->root.u.def.value = sym->st_value;
	      h->root.u.def.section = sec;
	    }
	}
      else
	{
	  bool warned, ignored;

	  RELOC_FOR_GLOBAL_SYMBOL (info, input_bfd, input_section, rel,
				   r_symndx, symtab_hdr, sym_hashes,
				   h, sec, relocation,
				   unresolved_reloc, warned, ignored);
	  if (warned)
	    {
	      /* To avoid generating warning messages about truncated
		 relocations, set the relocation's address to be the same as
		 the start of this section.  */
	      if (input_section->output_section != NULL)
		relocation = input_section->output_section->vma;
	      else
		relocation = 0;
	    }
	}

      if (sec != NULL && discarded_section (sec))
	RELOC_AGAINST_DISCARDED_SECTION (info, input_bfd, input_section,
					 rel, 1, relend, howto, 0, contents);

      if (bfd_link_relocatable (info))
	continue;

      /* Since STT_GNU_IFUNC symbol must go through PLT, we handle
	 it here if it is defined in a non-shared object.  */
      if (h != NULL
	  && h->type == STT_GNU_IFUNC
	  && h->def_regular)
	{
	  asection *plt, *base_got;

	  if ((input_section->flags & SEC_ALLOC) == 0)
	    {
	      /* If this is a SHT_NOTE section without SHF_ALLOC, treat
		 STT_GNU_IFUNC symbol as STT_FUNC.  */
	      if (elf_section_type (input_section) == SHT_NOTE)
		goto skip_ifunc;

	      /* Dynamic relocs are not propagated for SEC_DEBUGGING
		 sections because such sections are not SEC_ALLOC and
		 thus ld.so will not process them.  */
	      if ((input_section->flags & SEC_DEBUGGING) != 0)
		continue;

	      abort ();
	    }
	  else if (h->plt.offset == (bfd_vma) -1
		   /* The following relocation may not need the .plt entries
		      when all references to a STT_GNU_IFUNC symbols are done
		      via GOT or static function pointers.  */
		   && r_type != R_RISCV_32
		   && r_type != R_RISCV_64
		   && r_type != R_RISCV_HI20
		   && r_type != R_RISCV_GOT_HI20
		   && r_type != R_RISCV_LO12_I
		   && r_type != R_RISCV_LO12_S)
	    goto bad_ifunc_reloc;

	  /* STT_GNU_IFUNC symbol must go through PLT.  */
	  plt = htab->elf.splt ? htab->elf.splt : htab->elf.iplt;
	  relocation = plt->output_section->vma
		       + plt->output_offset
		       + h->plt.offset;

	  switch (r_type)
	    {
	    case R_RISCV_32:
	    case R_RISCV_64:
	      if (rel->r_addend != 0)
		{
		  if (h->root.root.string)
		    name = h->root.root.string;
		  else
		    name = bfd_elf_sym_name (input_bfd, symtab_hdr, sym, NULL);

		  _bfd_error_handler
		    /* xgettext:c-format */
		    (_("%pB: relocation %s against STT_GNU_IFUNC "
		       "symbol `%s' has non-zero addend: %" PRId64),
		     input_bfd, howto->name, name, (int64_t) rel->r_addend);
		  bfd_set_error (bfd_error_bad_value);
		  return false;
		}

		/* Generate dynamic relocation only when there is a non-GOT
		   reference in a shared object or there is no PLT.  */
		if ((bfd_link_pic (info) && h->non_got_ref)
		    || h->plt.offset == (bfd_vma) -1)
		  {
		    Elf_Internal_Rela outrel;
		    asection *sreloc;

		    /* Need a dynamic relocation to get the real function
		       address.  */
		    outrel.r_offset = _bfd_elf_section_offset (output_bfd,
							       info,
							       input_section,
							       rel->r_offset);
		    if (outrel.r_offset == (bfd_vma) -1
			|| outrel.r_offset == (bfd_vma) -2)
		      abort ();

		    outrel.r_offset += input_section->output_section->vma
				       + input_section->output_offset;

		    if (h->dynindx == -1
			|| h->forced_local
			|| bfd_link_executable (info))
		      {
			info->callbacks->minfo
			  (_("Local IFUNC function `%s' in %pB\n"),
			   h->root.root.string,
			   h->root.u.def.section->owner);

			/* This symbol is resolved locally.  */
			outrel.r_info = ELFNN_R_INFO (0, R_RISCV_IRELATIVE);
			outrel.r_addend = h->root.u.def.value
			  + h->root.u.def.section->output_section->vma
			  + h->root.u.def.section->output_offset;
		      }
		    else
		      {
			outrel.r_info = ELFNN_R_INFO (h->dynindx, r_type);
			outrel.r_addend = 0;
		      }

		    /* Dynamic relocations are stored in
		       1. .rela.ifunc section in PIC object.
		       2. .rela.got section in dynamic executable.
		       3. .rela.iplt section in static executable.  */
		    if (bfd_link_pic (info))
		      sreloc = htab->elf.irelifunc;
		    else if (htab->elf.splt != NULL)
		      sreloc = htab->elf.srelgot;
		    else
		      sreloc = htab->elf.irelplt;

		    riscv_elf_append_rela (output_bfd, sreloc, &outrel);

		    /* If this reloc is against an external symbol, we
		       do not want to fiddle with the addend.  Otherwise,
		       we need to include the symbol value so that it
		       becomes an addend for the dynamic reloc.  For an
		       internal symbol, we have updated addend.  */
		    continue;
		  }
		goto do_relocation;

	      case R_RISCV_GOT_HI20:
		base_got = htab->elf.sgot;
		off = h->got.offset;

		if (base_got == NULL)
		  abort ();

		if (off == (bfd_vma) -1)
		  {
		    bfd_vma plt_idx;

		    /* We can't use h->got.offset here to save state, or
		       even just remember the offset, as finish_dynamic_symbol
		       would use that as offset into .got.  */

		    if (htab->elf.splt != NULL)
		      {
			plt_idx = (h->plt.offset - PLT_HEADER_SIZE)
				  / PLT_ENTRY_SIZE;
			off = GOTPLT_HEADER_SIZE + (plt_idx * GOT_ENTRY_SIZE);
			base_got = htab->elf.sgotplt;
		      }
		    else
		      {
			plt_idx = h->plt.offset / PLT_ENTRY_SIZE;
			off = plt_idx * GOT_ENTRY_SIZE;
			base_got = htab->elf.igotplt;
		      }

		    if (h->dynindx == -1
			|| h->forced_local
			|| info->symbolic)
		      {
			/* This references the local definition.  We must
			   initialize this entry in the global offset table.
			   Since the offset must always be a multiple of 8,
			   we use the least significant bit to record
			   whether we have initialized it already.

			   When doing a dynamic link, we create a .rela.got
			   relocation entry to initialize the value.  This
			   is done in the finish_dynamic_symbol routine.   */
			if ((off & 1) != 0)
			  off &= ~1;
			else
			  {
			    bfd_put_NN (output_bfd, relocation,
					base_got->contents + off);
			    /* Note that this is harmless for the case,
			       as -1 | 1 still is -1.  */
			    h->got.offset |= 1;
			  }
		      }
		  }

		relocation = base_got->output_section->vma
			     + base_got->output_offset + off;

		if (!riscv_record_pcrel_hi_reloc (&pcrel_relocs, pc,
						  relocation, r_type,
						  false))
		  r = bfd_reloc_overflow;
		goto do_relocation;

	      case R_RISCV_CALL:
	      case R_RISCV_CALL_PLT:
	      case R_RISCV_HI20:
	      case R_RISCV_LO12_I:
	      case R_RISCV_LO12_S:
		goto do_relocation;

	      case R_RISCV_PCREL_HI20:
		if (!riscv_record_pcrel_hi_reloc (&pcrel_relocs, pc,
						  relocation, r_type,
						  false))
		  r = bfd_reloc_overflow;
		goto do_relocation;

	    default:
	    bad_ifunc_reloc:
	      if (h->root.root.string)
		name = h->root.root.string;
	      else
		/* The entry of local ifunc is fake in global hash table,
		   we should find the name by the original local symbol.  */
		name = bfd_elf_sym_name (input_bfd, symtab_hdr, sym, NULL);

	      _bfd_error_handler
	      /* xgettext:c-format */
	      (_("%pB: relocation %s against STT_GNU_IFUNC "
		 "symbol `%s' isn't supported"), input_bfd,
	       howto->name, name);
	      bfd_set_error (bfd_error_bad_value);
	      return false;
	    }
	}

    skip_ifunc:
      if (h != NULL)
	name = h->root.root.string;
      else
	{
	  name = (bfd_elf_string_from_elf_section
		  (input_bfd, symtab_hdr->sh_link, sym->st_name));
	  if (name == NULL || *name == '\0')
	    name = bfd_section_name (sec);
	}

      resolved_to_zero = (h != NULL
			  && UNDEFWEAK_NO_DYNAMIC_RELOC (info, h));

      /* We don't allow any mixed indirect call function.  */
      if (find_imported_ict_table
	  && input_section == bfd_get_section_by_name (input_bfd, ANDES_ICT_SECTION))
	{
	  /* Don't need to check the mixed ict cases for the
	     imported ict table at the second link-time.  */
	}
      else if (h && riscv_elf_hash_entry (h)->indirect_call
	       && r_type != R_RISCV_ICT_HI20
	       && r_type != R_RISCV_ICT_LO12_I
	       && r_type != R_RISCV_PCREL_ICT_HI20
	       && r_type != R_RISCV_CALL_ICT
	       && r_type != R_RISCV_ICT_64)
	{
	#if 0
	  (*_bfd_error_handler)
	    (_("%pB: Error: a mixed indirect call function \'%s\' "
	       "in the ICT model\n"),
	     input_bfd, h->root.root.string);
	  return false;
	#else
	  (*_bfd_error_handler)
	    (_("%pB: Warning: a mixed indirect call function \'%s\' "
	       "in the ICT model\n"),
	     input_bfd, h->root.root.string);
	#endif
	}

      /* { Andes */
      if (relx != NULL)
	r_type = R_RISCV_ANDES_TAG;
      /* } Andes */

      switch (r_type)
	{
	case R_RISCV_NONE:
	case R_RISCV_RELAX:
	case R_RISCV_TPREL_ADD:
	case R_RISCV_COPY:
	case R_RISCV_JUMP_SLOT:
	case R_RISCV_RELATIVE:
	  /* These require nothing of us at all.  */
	  continue;

	case R_RISCV_HI20:
	case R_RISCV_BRANCH:
	case R_RISCV_RVC_BRANCH:
	case R_RISCV_RVC_LUI:
	case R_RISCV_LO12_I:
	case R_RISCV_LO12_S:
	case R_RISCV_SET6:
	case R_RISCV_SET8:
	case R_RISCV_SET16:
	case R_RISCV_SET32:
	case R_RISCV_32_PCREL:
	case R_RISCV_DELETE:
	case R_RISCV_10_PCREL: /* Andes */
	  /* These require no special handling beyond perform_relocation.  */
	  break;

	case R_RISCV_GOT_HI20:
	  if (h != NULL)
	    {
	      bool dyn, pic;

	      off = h->got.offset;
	      BFD_ASSERT (off != (bfd_vma) -1);
	      dyn = elf_hash_table (info)->dynamic_sections_created;
	      pic = bfd_link_pic (info);

	      if (! WILL_CALL_FINISH_DYNAMIC_SYMBOL (dyn, pic, h)
		  || (pic && SYMBOL_REFERENCES_LOCAL (info, h)))
		{
		  /* This is actually a static link, or it is a
		     -Bsymbolic link and the symbol is defined
		     locally, or the symbol was forced to be local
		     because of a version file.  We must initialize
		     this entry in the global offset table.  Since the
		     offset must always be a multiple of the word size,
		     we use the least significant bit to record whether
		     we have initialized it already.

		     When doing a dynamic link, we create a .rela.got
		     relocation entry to initialize the value.  This
		     is done in the finish_dynamic_symbol routine.  */
		  if ((off & 1) != 0)
		    off &= ~1;
		  else
		    {
		      bfd_put_NN (output_bfd, relocation,
				  htab->elf.sgot->contents + off);
		      h->got.offset |= 1;
		    }
		}
	      else
		unresolved_reloc = false;
	    }
	  else
	    {
	      BFD_ASSERT (local_got_offsets != NULL
			  && local_got_offsets[r_symndx] != (bfd_vma) -1);

	      off = local_got_offsets[r_symndx];

	      /* The offset must always be a multiple of the word size.
		 So, we can use the least significant bit to record
		 whether we have already processed this entry.  */
	      if ((off & 1) != 0)
		off &= ~1;
	      else
		{
		  if (bfd_link_pic (info))
		    {
		      asection *s;
		      Elf_Internal_Rela outrel;

		      /* We need to generate a R_RISCV_RELATIVE reloc
			 for the dynamic linker.  */
		      s = htab->elf.srelgot;
		      BFD_ASSERT (s != NULL);

		      outrel.r_offset = sec_addr (htab->elf.sgot) + off;
		      outrel.r_info =
			ELFNN_R_INFO (0, R_RISCV_RELATIVE);
		      outrel.r_addend = relocation;
		      relocation = 0;
		      riscv_elf_append_rela (output_bfd, s, &outrel);
		    }

		  bfd_put_NN (output_bfd, relocation,
			      htab->elf.sgot->contents + off);
		  local_got_offsets[r_symndx] |= 1;
		}
	    }

	  if (rel->r_addend != 0)
	    {
	      msg = _("The addend isn't allowed for R_RISCV_GOT_HI20");
	      r = bfd_reloc_dangerous;
	    }
	  else
	    {
	      /* Address of got entry.  */
	      relocation = sec_addr (htab->elf.sgot) + off;
	      absolute = riscv_zero_pcrel_hi_reloc (rel, info, pc,
						    relocation, contents,
						    howto);
	      /* Update howto if relocation is changed.  */
	      howto = riscv_elf_rtype_to_howto (input_bfd,
						ELFNN_R_TYPE (rel->r_info));
	      if (howto == NULL)
		r = bfd_reloc_notsupported;
	      else if (!riscv_record_pcrel_hi_reloc (&pcrel_relocs, pc,
						     relocation, r_type,
						     absolute))
		r = bfd_reloc_overflow;
	    }
	  break;

	case R_RISCV_ADD8:
	case R_RISCV_ADD16:
	case R_RISCV_ADD32:
	case R_RISCV_ADD64:
	  {
	    bfd_vma old_value = bfd_get (howto->bitsize, input_bfd,
					 contents + rel->r_offset);
	    relocation = old_value + relocation;
	  }
	  break;

	case R_RISCV_SUB6:
	case R_RISCV_SUB8:
	case R_RISCV_SUB16:
	case R_RISCV_SUB32:
	case R_RISCV_SUB64:
	  {
	    bfd_vma old_value = bfd_get (howto->bitsize, input_bfd,
					 contents + rel->r_offset);
	    relocation = old_value - relocation;
	  }
	  break;

	case R_RISCV_TABLE_JUMP:
	  {
	    bfd_vma insn = bfd_getl16 (contents + rel->r_offset);
	    unsigned int tbl_index = EXTRACT_ZCMP_TABLE_JUMP_INDEX (insn);
	    htab->table_jump_htab->tbj_indexes[tbl_index] = relocation + rel->r_addend;
	    htab->table_jump_htab->tbj_indexes[tbl_index] &= ~ (bfd_vma) 1;
	  }
	  break;

	case R_RISCV_CALL:
	case R_RISCV_CALL_PLT:
	  /* Handle a call to an undefined weak function.  This won't be
	     relaxed, so we have to handle it here.  */
	  if (h != NULL && h->root.type == bfd_link_hash_undefweak
	      && (!bfd_link_pic (info) || h->plt.offset == MINUS_ONE))
	    {
	      /* We can use x0 as the base register.  */
	      bfd_vma insn = bfd_getl32 (contents + rel->r_offset + 4);
	      insn &= ~(OP_MASK_RS1 << OP_SH_RS1);
	      bfd_putl32 (insn, contents + rel->r_offset + 4);
	      /* Set the relocation value so that we get 0 after the pc
		 relative adjustment.  */
	      relocation = sec_addr (input_section) + rel->r_offset;
	    }
	  /* Fall through.  */

	case R_RISCV_RVC_JUMP:
	case R_RISCV_JAL:
	  /* This line has to match the check in _bfd_riscv_relax_section.  */
	  if (bfd_link_pic (info) && h != NULL && h->plt.offset != MINUS_ONE
	      && rel->r_user != REL_R_USER_FLG_RAW)
	    {
	      /* Refer to the PLT entry.  */
	      relocation = sec_addr (htab->elf.splt) + h->plt.offset;
	      unresolved_reloc = false;
	    }
	  break;

	case R_RISCV_TPREL_HI20:
	  relocation = tpoff (info, relocation);
	  break;

	case R_RISCV_TPREL_LO12_I:
	case R_RISCV_TPREL_LO12_S:
	  relocation = tpoff (info, relocation);
	  break;

	case R_RISCV_TPREL_I:
	case R_RISCV_TPREL_S:
	  relocation = tpoff (info, relocation);
	  if (VALID_ITYPE_IMM (relocation + rel->r_addend))
	    {
	      /* We can use tp as the base register.  */
	      bfd_vma insn = bfd_getl32 (contents + rel->r_offset);
	      insn &= ~(OP_MASK_RS1 << OP_SH_RS1);
	      insn |= X_TP << OP_SH_RS1;
	      bfd_putl32 (insn, contents + rel->r_offset);
	    }
	  else
	    r = bfd_reloc_overflow;
	  break;

	case R_RISCV_GPREL_I:
	case R_RISCV_GPREL_S:
	  {
	    bfd_vma gp = riscv_global_pointer_value (info);
	    bool x0_base = VALID_ITYPE_IMM (relocation + rel->r_addend);
	    if (x0_base || VALID_ITYPE_IMM (relocation + rel->r_addend - gp))
	      {
		/* We can use x0 or gp as the base register.  */
		bfd_vma insn = bfd_getl32 (contents + rel->r_offset);
		insn &= ~(OP_MASK_RS1 << OP_SH_RS1);
		if (!x0_base)
		  {
		    rel->r_addend -= gp;
		    insn |= X_GP << OP_SH_RS1;
		  }
		bfd_putl32 (insn, contents + rel->r_offset);
	      }
	    else
	      r = bfd_reloc_overflow;
	    break;
	  }

	case R_RISCV_PCREL_HI20:
	  /* if annotated within andes_relax_execit_ite  */
	  is_execited = rel->r_offset & 1;
	  if (is_execited)
	    { /* un-annotation for processing.  */
	      rel->r_offset ^= 1;
	      pc ^= 1;
	    }
	  absolute = riscv_zero_pcrel_hi_reloc (rel, info, pc, relocation,
						contents, howto);
	  /* Update howto if relocation is changed.  */
	  howto = riscv_elf_rtype_to_howto (input_bfd,
					    ELFNN_R_TYPE (rel->r_info));
	  if (howto == NULL)
	    r = bfd_reloc_notsupported;
	  else if (!riscv_record_pcrel_hi_reloc (&pcrel_relocs, pc,
						 relocation + rel->r_addend,
						 r_type, absolute))
	    r = bfd_reloc_overflow;
	  if (is_execited)
	    rel->r_offset ^= 1; /* restore the annotation.  */
	  break;

	case R_RISCV_PCREL_LO12_I:
	case R_RISCV_PCREL_LO12_S:
	  /* We don't allow section symbols plus addends as the auipc address,
	     because then riscv_relax_delete_bytes would have to search through
	     all relocs to update these addends.  This is also ambiguous, as
	     we do allow offsets to be added to the target address, which are
	     not to be used to find the auipc address.  */
	  if (((sym != NULL && (ELF_ST_TYPE (sym->st_info) == STT_SECTION))
	       || (h != NULL && h->type == STT_SECTION))
	      && rel->r_addend)
	    {
	      msg = _("%pcrel_lo section symbol with an addend");
	      r = bfd_reloc_dangerous;
	      break;
	    }

	  if (riscv_record_pcrel_lo_reloc (&pcrel_relocs, relocation, rel,
					   input_section, info, howto,
					   contents))
	    continue;
	  r = bfd_reloc_overflow;
	  break;

	case R_RISCV_TLS_DTPREL32:
	case R_RISCV_TLS_DTPREL64:
	  relocation = dtpoff (info, relocation);
	  break;

	case R_RISCV_32:
	case R_RISCV_64:
	  if ((input_section->flags & SEC_ALLOC) == 0)
	    break;

	  if ((bfd_link_pic (info)
	       && (h == NULL
		   || (ELF_ST_VISIBILITY (h->other) == STV_DEFAULT
		       && !resolved_to_zero)
		   || h->root.type != bfd_link_hash_undefweak)
	       && (!howto->pc_relative
		   || !SYMBOL_CALLS_LOCAL (info, h)))
	      || (!bfd_link_pic (info)
		  && h != NULL
		  && h->dynindx != -1
		  && !h->non_got_ref
		  && ((h->def_dynamic
		       && !h->def_regular)
		      || h->root.type == bfd_link_hash_undefweak
		      || h->root.type == bfd_link_hash_undefined)))
	    {
	      Elf_Internal_Rela outrel;
	      asection *sreloc;
	      bool skip_static_relocation, skip_dynamic_relocation;

	      /* When generating a shared object, these relocations
		 are copied into the output file to be resolved at run
		 time.  */

	      outrel.r_offset =
		_bfd_elf_section_offset (output_bfd, info, input_section,
					 rel->r_offset);
	      skip_static_relocation = outrel.r_offset != (bfd_vma) -2;
	      skip_dynamic_relocation = outrel.r_offset >= (bfd_vma) -2;
	      outrel.r_offset += sec_addr (input_section);

	      if (skip_dynamic_relocation)
		memset (&outrel, 0, sizeof outrel);
	      else if (h != NULL && h->dynindx != -1
		       && !(bfd_link_pic (info)
			    && SYMBOLIC_BIND (info, h)
			    && h->def_regular))
		{
		  outrel.r_info = ELFNN_R_INFO (h->dynindx, r_type);
		  outrel.r_addend = rel->r_addend;
		}
	      else
		{
		  outrel.r_info = ELFNN_R_INFO (0, R_RISCV_RELATIVE);
		  outrel.r_addend = relocation + rel->r_addend;
		}

	      sreloc = elf_section_data (input_section)->sreloc;
	      riscv_elf_append_rela (output_bfd, sreloc, &outrel);
	      if (skip_static_relocation)
		continue;
	    }
	  break;

	case R_RISCV_TLS_GOT_HI20:
	  is_ie = true;
	  /* Fall through.  */

	case R_RISCV_TLS_GD_HI20:
	  if (h != NULL)
	    {
	      off = h->got.offset;
	      h->got.offset |= 1;
	    }
	  else
	    {
	      off = local_got_offsets[r_symndx];
	      local_got_offsets[r_symndx] |= 1;
	    }

	  tls_type = _bfd_riscv_elf_tls_type (input_bfd, h, r_symndx);
	  BFD_ASSERT (tls_type & (GOT_TLS_IE | GOT_TLS_GD));
	  /* If this symbol is referenced by both GD and IE TLS, the IE
	     reference's GOT slot follows the GD reference's slots.  */
	  ie_off = 0;
	  if ((tls_type & GOT_TLS_GD) && (tls_type & GOT_TLS_IE))
	    ie_off = 2 * GOT_ENTRY_SIZE;

	  if ((off & 1) != 0)
	    off &= ~1;
	  else
	    {
	      Elf_Internal_Rela outrel;
	      int indx = 0;
	      bool need_relocs = false;

	      if (htab->elf.srelgot == NULL)
		abort ();

	      if (h != NULL)
		{
		  bool dyn, pic;
		  dyn = htab->elf.dynamic_sections_created;
		  pic = bfd_link_pic (info);

		  if (WILL_CALL_FINISH_DYNAMIC_SYMBOL (dyn, pic, h)
		      && (!pic || !SYMBOL_REFERENCES_LOCAL (info, h)))
		    indx = h->dynindx;
		}

	      /* The GOT entries have not been initialized yet.  Do it
		 now, and emit any relocations.  */
	      if ((bfd_link_pic (info) || indx != 0)
		  && (h == NULL
		      || ELF_ST_VISIBILITY (h->other) == STV_DEFAULT
		      || h->root.type != bfd_link_hash_undefweak))
		    need_relocs = true;

	      if (tls_type & GOT_TLS_GD)
		{
		  if (need_relocs)
		    {
		      outrel.r_offset = sec_addr (htab->elf.sgot) + off;
		      outrel.r_addend = 0;
		      outrel.r_info = ELFNN_R_INFO (indx, R_RISCV_TLS_DTPMODNN);
		      bfd_put_NN (output_bfd, 0,
				  htab->elf.sgot->contents + off);
		      riscv_elf_append_rela (output_bfd, htab->elf.srelgot, &outrel);
		      if (indx == 0)
			{
			  BFD_ASSERT (! unresolved_reloc);
			  bfd_put_NN (output_bfd,
				      dtpoff (info, relocation),
				      (htab->elf.sgot->contents
				       + off + RISCV_ELF_WORD_BYTES));
			}
		      else
			{
			  bfd_put_NN (output_bfd, 0,
				      (htab->elf.sgot->contents
				       + off + RISCV_ELF_WORD_BYTES));
			  outrel.r_info = ELFNN_R_INFO (indx, R_RISCV_TLS_DTPRELNN);
			  outrel.r_offset += RISCV_ELF_WORD_BYTES;
			  riscv_elf_append_rela (output_bfd, htab->elf.srelgot, &outrel);
			}
		    }
		  else
		    {
		      /* If we are not emitting relocations for a
			 general dynamic reference, then we must be in a
			 static link or an executable link with the
			 symbol binding locally.  Mark it as belonging
			 to module 1, the executable.  */
		      bfd_put_NN (output_bfd, 1,
				  htab->elf.sgot->contents + off);
		      bfd_put_NN (output_bfd,
				  dtpoff (info, relocation),
				  (htab->elf.sgot->contents
				   + off + RISCV_ELF_WORD_BYTES));
		   }
		}

	      if (tls_type & GOT_TLS_IE)
		{
		  if (need_relocs)
		    {
		      bfd_put_NN (output_bfd, 0,
				  htab->elf.sgot->contents + off + ie_off);
		      outrel.r_offset = sec_addr (htab->elf.sgot)
					+ off + ie_off;
		      outrel.r_addend = 0;
		      if (indx == 0)
			outrel.r_addend = tpoff (info, relocation);
		      outrel.r_info = ELFNN_R_INFO (indx, R_RISCV_TLS_TPRELNN);
		      riscv_elf_append_rela (output_bfd, htab->elf.srelgot, &outrel);
		    }
		  else
		    {
		      bfd_put_NN (output_bfd, tpoff (info, relocation),
				  htab->elf.sgot->contents + off + ie_off);
		    }
		}
	    }

	  BFD_ASSERT (off < (bfd_vma) -2);
	  relocation = sec_addr (htab->elf.sgot) + off + (is_ie ? ie_off : 0);
	  if (!riscv_record_pcrel_hi_reloc (&pcrel_relocs, pc,
					    relocation, r_type,
					    false))
	    r = bfd_reloc_overflow;
	  unresolved_reloc = false;
	  break;

	/* { Andes  */
	case R_RISCV_LGP18S0:
	case R_RISCV_LGP17S1:
	case R_RISCV_LGP17S2:
	case R_RISCV_LGP17S3:
	case R_RISCV_SGP18S0:
	case R_RISCV_SGP17S1:
	case R_RISCV_SGP17S2:
	case R_RISCV_SGP17S3:
	  {
	    bfd_vma gp = riscv_global_pointer_value (info);
	    relocation -= gp;	
	    unresolved_reloc = false;
	    break;
	  }

	case R_RISCV_ICT_HI20:
	case R_RISCV_ICT_LO12_I:
	case R_RISCV_PCREL_ICT_HI20:
	case R_RISCV_CALL_ICT:
	case R_RISCV_ICT_64:
	  {
	    andes_ict_hash_t *hash;
	    struct bfd_link_hash_entry *ict_base;
	    int ict_entry_size;

	    hash = (andes_ict_hash_t*)
	      bfd_hash_lookup (&nds_ict_sta.indirect_call_table, h->root.root.string,
			       false, false);
	    if (hash == NULL)
	      {
		(*_bfd_error_handler)
		  (_("%pB %pA: internal error indirect call relocation "
		     "0x%lx without hash.\n"),
		     input_bfd, sec, rel->r_offset);
		bfd_set_error (bfd_error_bad_value);
		return false;
	      }

	    ict_base = bfd_link_hash_lookup (info->hash,
					     "_INDIRECT_CALL_TABLE_BASE_",
					     false, false, false);

	    if (ict_model == 1 || ict_model == 2)
	      /* Small model, use `call' in the ict table.  */
	      /* Large model, use `.dword' in the ict table.  */
	      ict_entry_size = 8;
	    else
	      /* Tiny model, use `jal' in the ict table.  */
	      ict_entry_size = 4;

	    relocation = ((ict_base->u.def.value
			   + ict_base->u.def.section->output_section->vma
			   + ict_base->u.def.section->output_offset)
			  + (hash->entry->index * ict_entry_size));

	    if (r_type == R_RISCV_PCREL_ICT_HI20)
	      {
		if (!riscv_record_pcrel_hi_reloc (&pcrel_relocs, pc,
						  relocation + rel->r_addend,
						  r_type, false))
		  r = bfd_reloc_overflow;
	      }

	    break;
	  }

	case R_RISCV_ANDES_TAG:
	  {
	    int tag = relx->flags;
	    relocation += rel->r_addend;
	    if (tag == R_RISCV_EXECIT_ITE)
	      { /* convert relocation for execit_ite.  */
		bfd_vma index = relx->tag;
		relx->saved_irel.r_offset = rel->r_offset;
		relx->saved_irel.r_addend = rel->r_addend;
		andes_relax_execit_ite (input_bfd, input_section, NULL, NULL,
		  &relx->saved_irel, relocation, index, 0, NULL, NULL, false);

		/* record R_RISCV_PCREL_HI20 for pals.  */
		execit_hash_t *he = execit.itable_array[index].he;
		int rtype = ELFNN_R_TYPE (he->ie.irel_copy.r_info);
		if (rtype == R_RISCV_CALL || rtype == R_RISCV_PCREL_HI20)
		  rtype = ELFNN_R_TYPE (rel->r_info);
		if (rtype == R_RISCV_PCREL_HI20)
		  riscv_record_pcrel_hi_reloc (&pcrel_relocs, pc, relocation,
					       rtype, false);
	      }
	    else if (tag == TAG_GPREL_SUBTYPE_FLX
		     || tag == TAG_GPREL_SUBTYPE_FSX)
	      { /* F[LS]X rd, *sym(gp) */
		relx->saved_irel.r_offset = rel->r_offset;
		relx->saved_irel.r_addend = rel->r_addend;
		andes_relax_fls_gp (input_bfd, input_section, NULL, info,
		  &relx->saved_irel, relocation, 0, 0, NULL, NULL, false);
	      }
	    else
	      BFD_ASSERT (0);

	    rel->r_info = ELFNN_R_INFO (0, R_RISCV_NONE);
	    continue;
	  }
	/* } Andes  */

	default:
	  r = bfd_reloc_notsupported;
	}

      /* Dynamic relocs are not propagated for SEC_DEBUGGING sections
	 because such sections are not SEC_ALLOC and thus ld.so will
	 not process them.  */
      if (unresolved_reloc
	  && !((input_section->flags & SEC_DEBUGGING) != 0
	       && h->def_dynamic)
	  && _bfd_elf_section_offset (output_bfd, info, input_section,
				      rel->r_offset) != (bfd_vma) -1)
	{
	  switch (r_type)
	    {
	    case R_RISCV_JAL:
	    case R_RISCV_RVC_JUMP:
	      if (asprintf (&msg_buf,
			    _("%%X%%P: relocation %s against `%s' can "
			      "not be used when making a shared object; "
			      "recompile with -fPIC\n"),
			    howto->name,
			    h->root.root.string) == -1)
		msg_buf = NULL;
	      break;

	    default:
	      if (asprintf (&msg_buf,
			    _("%%X%%P: unresolvable %s relocation against "
			      "symbol `%s'\n"),
			    howto->name,
			    h->root.root.string) == -1)
		msg_buf = NULL;
	      break;
	    }

	  msg = msg_buf;
	  r = bfd_reloc_notsupported;
	}

 do_relocation:
      if (is_execited)
	is_execited = 0;
      else if (r == bfd_reloc_ok)
	r = perform_relocation (howto, rel, relocation, input_section,
				input_bfd, contents);

      /* We should have already detected the error and set message before.
	 If the error message isn't set since the linker runs out of memory
	 or we don't set it before, then we should set the default message
	 with the "internal error" string here.  */
      switch (r)
	{
	case bfd_reloc_ok:
	  continue;

	case bfd_reloc_overflow:
	  info->callbacks->reloc_overflow
	    (info, (h ? &h->root : NULL), name, howto->name,
	     (bfd_vma) 0, input_bfd, input_section, rel->r_offset);
	  break;

	case bfd_reloc_undefined:
	  info->callbacks->undefined_symbol
	    (info, name, input_bfd, input_section, rel->r_offset,
	     true);
	  break;

	case bfd_reloc_outofrange:
	  if (msg == NULL)
	    msg = _("%X%P: internal error: out of range error\n");
	  break;

	case bfd_reloc_notsupported:
	  if (msg == NULL)
	    msg = _("%X%P: internal error: unsupported relocation error\n");
	  break;

	case bfd_reloc_dangerous:
	  /* The error message should already be set.  */
	  if (msg == NULL)
	    msg = _("dangerous relocation error");
	  info->callbacks->reloc_dangerous
	    (info, msg, input_bfd, input_section, rel->r_offset);
	  break;

	default:
	  msg = _("%X%P: internal error: unknown error\n");
	  break;
	}

      /* Do not report error message for the dangerous relocation again.  */
      if (msg && r != bfd_reloc_dangerous)
	info->callbacks->einfo (msg);

      /* Free the unused `msg_buf`.  */
      free (msg_buf);

      /* We already reported the error via a callback, so don't try to report
	 it again by returning false.  That leads to spurious errors.  */
      ret = true;
      goto out;
    }

  ret = riscv_resolve_pcrel_lo_relocs (&pcrel_relocs);
 out:
  riscv_free_pcrel_relocs (&pcrel_relocs);

  /* { Andes */
  /* again here to deal with newly allocated entries.  */
  if (andes->target_optimization & RISCV_RELAX_EXECIT_ON)
    andes_execit_relocate_itable (info);
  /* } Andes */

  return ret;
}

/* Finish up dynamic symbol handling.  We set the contents of various
   dynamic sections here.  */

static bool
riscv_elf_finish_dynamic_symbol (bfd *output_bfd,
				 struct bfd_link_info *info,
				 struct elf_link_hash_entry *h,
				 Elf_Internal_Sym *sym)
{
  struct riscv_elf_link_hash_table *htab = riscv_elf_hash_table (info);
  const struct elf_backend_data *bed = get_elf_backend_data (output_bfd);

  if (h->plt.offset != (bfd_vma) -1)
    {
      /* We've decided to create a PLT entry for this symbol.  */
      bfd_byte *loc;
      bfd_vma i, header_address, plt_idx, got_offset, got_address;
      uint32_t plt_entry[PLT_ENTRY_INSNS];
      Elf_Internal_Rela rela;
      asection *plt, *gotplt, *relplt;

      /* When building a static executable, use .iplt, .igot.plt and
	 .rela.iplt sections for STT_GNU_IFUNC symbols.  */
      if (htab->elf.splt != NULL)
        {
          plt = htab->elf.splt;
          gotplt = htab->elf.sgotplt;
          relplt = htab->elf.srelplt;
        }
      else
        {
          plt = htab->elf.iplt;
          gotplt = htab->elf.igotplt;
          relplt = htab->elf.irelplt;
        }

      /* This symbol has an entry in the procedure linkage table.  Set
         it up.  */
      if ((h->dynindx == -1
	   && !((h->forced_local || bfd_link_executable (info))
		&& h->def_regular
		&& h->type == STT_GNU_IFUNC))
	  || plt == NULL
	  || gotplt == NULL
	  || relplt == NULL)
	return false;

      /* Calculate the address of the PLT header.  */
      header_address = sec_addr (plt);

      /* Calculate the index of the entry and the offset of .got.plt entry.
	 For static executables, we don't reserve anything.  */
      if (plt == htab->elf.splt)
	{
	  plt_idx = (h->plt.offset - PLT_HEADER_SIZE) / PLT_ENTRY_SIZE;
	  got_offset = GOTPLT_HEADER_SIZE + (plt_idx * GOT_ENTRY_SIZE);
	}
      else
	{
	  plt_idx = h->plt.offset / PLT_ENTRY_SIZE;
	  got_offset = plt_idx * GOT_ENTRY_SIZE;
	}

      /* Calculate the address of the .got.plt entry.  */
      got_address = sec_addr (gotplt) + got_offset;

      /* Find out where the .plt entry should go.  */
      loc = plt->contents + h->plt.offset;

      /* Fill in the PLT entry itself.  */
      if (! riscv_make_plt_entry (output_bfd, got_address,
				  header_address + h->plt.offset,
				  plt_entry))
	return false;

      for (i = 0; i < PLT_ENTRY_INSNS; i++)
	bfd_putl32 (plt_entry[i], loc + 4*i);

      /* Fill in the initial value of the .got.plt entry.  */
      loc = gotplt->contents + (got_address - sec_addr (gotplt));
      bfd_put_NN (output_bfd, sec_addr (plt), loc);

      rela.r_offset = got_address;

      if (h->dynindx == -1
	  || ((bfd_link_executable (info)
	       || ELF_ST_VISIBILITY (h->other) != STV_DEFAULT)
	      && h->def_regular
	      && h->type == STT_GNU_IFUNC))
	{
	  info->callbacks->minfo (_("Local IFUNC function `%s' in %pB\n"),
				  h->root.root.string,
				  h->root.u.def.section->owner);

	  /* If an STT_GNU_IFUNC symbol is locally defined, generate
	     R_RISCV_IRELATIVE instead of R_RISCV_JUMP_SLOT.  */
	  asection *sec = h->root.u.def.section;
	  rela.r_info = ELFNN_R_INFO (0, R_RISCV_IRELATIVE);
	  rela.r_addend = h->root.u.def.value
			  + sec->output_section->vma
			  + sec->output_offset;
	}
      else
	{
	  /* Fill in the entry in the .rela.plt section.  */
	  rela.r_info = ELFNN_R_INFO (h->dynindx, R_RISCV_JUMP_SLOT);
	  rela.r_addend = 0;
	}

      loc = relplt->contents + plt_idx * sizeof (ElfNN_External_Rela);
      bed->s->swap_reloca_out (output_bfd, &rela, loc);

      if (!h->def_regular)
	{
	  /* Mark the symbol as undefined, rather than as defined in
	     the .plt section.  Leave the value alone.  */
	  sym->st_shndx = SHN_UNDEF;
	  /* If the symbol is weak, we do need to clear the value.
	     Otherwise, the PLT entry would provide a definition for
	     the symbol even if the symbol wasn't defined anywhere,
	     and so the symbol would never be NULL.  */
	  if (!h->ref_regular_nonweak)
	    sym->st_value = 0;
	}
    }

  if (h->got.offset != (bfd_vma) -1
      && !(riscv_elf_hash_entry (h)->tls_type & (GOT_TLS_GD | GOT_TLS_IE))
      && !UNDEFWEAK_NO_DYNAMIC_RELOC (info, h))
    {
      asection *sgot;
      asection *srela;
      Elf_Internal_Rela rela;
      bool use_elf_append_rela = true;

      /* This symbol has an entry in the GOT.  Set it up.  */

      sgot = htab->elf.sgot;
      srela = htab->elf.srelgot;
      BFD_ASSERT (sgot != NULL && srela != NULL);

      rela.r_offset = sec_addr (sgot) + (h->got.offset &~ (bfd_vma) 1);

      /* Handle the ifunc symbol in GOT entry.  */
      if (h->def_regular
	  && h->type == STT_GNU_IFUNC)
	{
	  if (h->plt.offset == (bfd_vma) -1)
	    {
	      /* STT_GNU_IFUNC is referenced without PLT.  */

	      if (htab->elf.splt == NULL)
		{
		  /* Use .rela.iplt section to store .got relocations
		     in static executable.  */
		  srela = htab->elf.irelplt;

		  /* Do not use riscv_elf_append_rela to add dynamic
		     relocs.  */
		  use_elf_append_rela = false;
		}

	      if (SYMBOL_REFERENCES_LOCAL (info, h))
		{
		  info->callbacks->minfo (_("Local IFUNC function `%s' in %pB\n"),
					  h->root.root.string,
					  h->root.u.def.section->owner);

		  rela.r_info = ELFNN_R_INFO (0, R_RISCV_IRELATIVE);
		  rela.r_addend = (h->root.u.def.value
				   + h->root.u.def.section->output_section->vma
				   + h->root.u.def.section->output_offset);
		}
	      else
		{
		  /* Generate R_RISCV_NN.  */
		  BFD_ASSERT ((h->got.offset & 1) == 0);
		  BFD_ASSERT (h->dynindx != -1);
		  rela.r_info = ELFNN_R_INFO (h->dynindx, R_RISCV_NN);
		  rela.r_addend = 0;
		}
	    }
	  else if (bfd_link_pic (info))
	    {
	      /* Generate R_RISCV_NN.  */
	      BFD_ASSERT ((h->got.offset & 1) == 0);
	      BFD_ASSERT (h->dynindx != -1);
	      rela.r_info = ELFNN_R_INFO (h->dynindx, R_RISCV_NN);
	      rela.r_addend = 0;
	    }
	  else
	    {
	      asection *plt;

	      if (!h->pointer_equality_needed)
		abort ();

	      /* For non-shared object, we can't use .got.plt, which
		 contains the real function address if we need pointer
		 equality.  We load the GOT entry with the PLT entry.  */
	      plt = htab->elf.splt ? htab->elf.splt : htab->elf.iplt;
	      bfd_put_NN (output_bfd, (plt->output_section->vma
				       + plt->output_offset
				       + h->plt.offset),
			  htab->elf.sgot->contents
			  + (h->got.offset & ~(bfd_vma) 1));
	      return true;
	    }
	}
      else if (bfd_link_pic (info)
	       && SYMBOL_REFERENCES_LOCAL (info, h))
	{
	  /* If this is a local symbol reference, we just want to emit
	     a RELATIVE reloc.  This can happen if it is a -Bsymbolic link,
	     or a pie link, or the symbol was forced to be local because
	     of a version file.  The entry in the global offset table will
	     already have been initialized in the relocate_section function.  */
	  BFD_ASSERT ((h->got.offset & 1) != 0);
	  asection *sec = h->root.u.def.section;
	  rela.r_info = ELFNN_R_INFO (0, R_RISCV_RELATIVE);
	  rela.r_addend = (h->root.u.def.value
			   + sec->output_section->vma
			   + sec->output_offset);
	}
      else
	{
	  BFD_ASSERT ((h->got.offset & 1) == 0);
	  BFD_ASSERT (h->dynindx != -1);
	  rela.r_info = ELFNN_R_INFO (h->dynindx, R_RISCV_NN);
	  rela.r_addend = 0;
	}

      bfd_put_NN (output_bfd, 0,
		  sgot->contents + (h->got.offset & ~(bfd_vma) 1));

      if (use_elf_append_rela)
	riscv_elf_append_rela (output_bfd, srela, &rela);
      else
	{
	  /* Use riscv_elf_append_rela to add the dynamic relocs into
	     .rela.iplt may cause the overwrite problems.  Since we insert
	     the relocs for PLT didn't handle the reloc_index of .rela.iplt,
	     but the riscv_elf_append_rela adds the relocs to the place
	     that are calculated from the reloc_index (in seqential).

	     One solution is that add these dynamic relocs (GOT IFUNC)
	     from the last of .rela.iplt section.  */
	  bfd_vma iplt_idx = htab->last_iplt_index--;
	  bfd_byte *loc = srela->contents
			  + iplt_idx * sizeof (ElfNN_External_Rela);
	  bed->s->swap_reloca_out (output_bfd, &rela, loc);
	}
    }

  if (h->needs_copy)
    {
      Elf_Internal_Rela rela;
      asection *s;

      /* This symbols needs a copy reloc.  Set it up.  */
      BFD_ASSERT (h->dynindx != -1);

      rela.r_offset = sec_addr (h->root.u.def.section) + h->root.u.def.value;
      rela.r_info = ELFNN_R_INFO (h->dynindx, R_RISCV_COPY);
      rela.r_addend = 0;
      if (h->root.u.def.section == htab->elf.sdynrelro)
	s = htab->elf.sreldynrelro;
      else
	s = htab->elf.srelbss;
      riscv_elf_append_rela (output_bfd, s, &rela);
    }

  /* Mark some specially defined symbols as absolute.  */
  if (h == htab->elf.hdynamic
      || (h == htab->elf.hgot || h == htab->elf.hplt))
    sym->st_shndx = SHN_ABS;

  return true;
}

/* Finish up local dynamic symbol handling.  We set the contents of
   various dynamic sections here.  */

static int
riscv_elf_finish_local_dynamic_symbol (void **slot, void *inf)
{
  struct elf_link_hash_entry *h = (struct elf_link_hash_entry *) *slot;
  struct bfd_link_info *info = (struct bfd_link_info *) inf;

  return riscv_elf_finish_dynamic_symbol (info->output_bfd, info, h, NULL);
}

/* Finish up the dynamic sections.  */

static bool
riscv_finish_dyn (bfd *output_bfd, struct bfd_link_info *info,
		  bfd *dynobj, asection *sdyn)
{
  struct riscv_elf_link_hash_table *htab = riscv_elf_hash_table (info);
  const struct elf_backend_data *bed = get_elf_backend_data (output_bfd);
  size_t dynsize = bed->s->sizeof_dyn;
  bfd_byte *dyncon, *dynconend;

  dynconend = sdyn->contents + sdyn->size;
  for (dyncon = sdyn->contents; dyncon < dynconend; dyncon += dynsize)
    {
      Elf_Internal_Dyn dyn;
      asection *s;

      bed->s->swap_dyn_in (dynobj, dyncon, &dyn);

      switch (dyn.d_tag)
	{
	case DT_PLTGOT:
	  s = htab->elf.sgotplt;
	  dyn.d_un.d_ptr = s->output_section->vma + s->output_offset;
	  break;
	case DT_JMPREL:
	  s = htab->elf.srelplt;
	  dyn.d_un.d_ptr = s->output_section->vma + s->output_offset;
	  break;
	case DT_PLTRELSZ:
	  s = htab->elf.srelplt;
	  dyn.d_un.d_val = s->size;
	  break;
	default:
	  continue;
	}

      bed->s->swap_dyn_out (output_bfd, &dyn, dyncon);
    }
  return true;
}

static bool
riscv_elf_finish_dynamic_sections (bfd *output_bfd,
				   struct bfd_link_info *info)
{
  bfd *dynobj;
  asection *sdyn;
  struct riscv_elf_link_hash_table *htab;

  htab = riscv_elf_hash_table (info);
  BFD_ASSERT (htab != NULL);
  dynobj = htab->elf.dynobj;

  sdyn = bfd_get_linker_section (dynobj, ".dynamic");

  if (elf_hash_table (info)->dynamic_sections_created)
    {
      asection *splt;
      bool ret;

      splt = htab->elf.splt;
      BFD_ASSERT (splt != NULL && sdyn != NULL);

      ret = riscv_finish_dyn (output_bfd, info, dynobj, sdyn);

      if (!ret)
	return ret;

      /* Fill in the head and tail entries in the procedure linkage table.  */
      if (splt->size > 0)
	{
	  int i;
	  uint32_t plt_header[PLT_HEADER_INSNS];
	  ret = riscv_make_plt_header (output_bfd,
				       sec_addr (htab->elf.sgotplt),
				       sec_addr (splt), plt_header);
	  if (!ret)
	    return ret;

	  for (i = 0; i < PLT_HEADER_INSNS; i++)
	    bfd_putl32 (plt_header[i], splt->contents + 4*i);

	  elf_section_data (splt->output_section)->this_hdr.sh_entsize
	    = PLT_ENTRY_SIZE;
	}
    }

  if (htab->elf.sgotplt)
    {
      asection *output_section = htab->elf.sgotplt->output_section;

      if (bfd_is_abs_section (output_section))
	{
	  (*_bfd_error_handler)
	    (_("discarded output section: `%pA'"), htab->elf.sgotplt);
	  return false;
	}

      if (htab->elf.sgotplt->size > 0)
	{
	  /* Write the first two entries in .got.plt, needed for the dynamic
	     linker.  */
	  bfd_put_NN (output_bfd, (bfd_vma) -1, htab->elf.sgotplt->contents);
	  bfd_put_NN (output_bfd, (bfd_vma) 0,
		      htab->elf.sgotplt->contents + GOT_ENTRY_SIZE);
	}

      elf_section_data (output_section)->this_hdr.sh_entsize = GOT_ENTRY_SIZE;
    }

  if (htab->elf.sgot)
    {
      asection *output_section = htab->elf.sgot->output_section;

      if (htab->elf.sgot->size > 0)
	{
	  /* Set the first entry in the global offset table to the address of
	     the dynamic section.  */
	  bfd_vma val = sdyn ? sec_addr (sdyn) : 0;
	  bfd_put_NN (output_bfd, val, htab->elf.sgot->contents);
	}

      elf_section_data (output_section)->this_hdr.sh_entsize = GOT_ENTRY_SIZE;
    }

  /* Fill PLT and GOT entries for local STT_GNU_IFUNC symbols.  */
  htab_traverse (htab->loc_hash_table,
		 riscv_elf_finish_local_dynamic_symbol,
		 info);

  return true;
}

/* Return address for Ith PLT stub in section PLT, for relocation REL
   or (bfd_vma) -1 if it should not be included.  */

static bfd_vma
riscv_elf_plt_sym_val (bfd_vma i, const asection *plt,
		       const arelent *rel ATTRIBUTE_UNUSED)
{
  return plt->vma + PLT_HEADER_SIZE + i * PLT_ENTRY_SIZE;
}

static enum elf_reloc_type_class
riscv_reloc_type_class (const struct bfd_link_info *info ATTRIBUTE_UNUSED,
			const asection *rel_sec ATTRIBUTE_UNUSED,
			const Elf_Internal_Rela *rela)
{
  switch (ELFNN_R_TYPE (rela->r_info))
    {
    case R_RISCV_RELATIVE:
      return reloc_class_relative;
    case R_RISCV_JUMP_SLOT:
      return reloc_class_plt;
    case R_RISCV_COPY:
      return reloc_class_copy;
    default:
      return reloc_class_normal;
    }
}

/* Given the ELF header flags in FLAGS, it returns a string that describes the
   float ABI.  */

static const char *
riscv_float_abi_string (flagword flags)
{
  switch (flags & EF_RISCV_FLOAT_ABI)
    {
    case EF_RISCV_FLOAT_ABI_SOFT:
      return "soft-float";
      break;
    case EF_RISCV_FLOAT_ABI_SINGLE:
      return "single-float";
      break;
    case EF_RISCV_FLOAT_ABI_DOUBLE:
      return "double-float";
      break;
    case EF_RISCV_FLOAT_ABI_QUAD:
      return "quad-float";
      break;
    default:
      abort ();
    }
}

/* The information of architecture elf attributes.  */
static riscv_subset_list_t in_subsets;
static riscv_subset_list_t out_subsets;
static riscv_subset_list_t merged_subsets;

/* Predicator for standard extension.  */

static bool
riscv_std_ext_p (const char *name)
{
  return (strlen (name) == 1) && (name[0] != 'x') && (name[0] != 's');
}

/* Update the output subset's version to match the input when the input
   subset's version is newer.  */

static void
riscv_update_subset_version (struct riscv_subset_t *in,
			     struct riscv_subset_t *out)
{
  if (in == NULL || out == NULL)
    return;

  /* Update the output ISA versions to the newest ones, but otherwise don't
     provide any errors or warnings about mis-matched ISA versions as it's
     generally too tricky to check for these at link time. */
  if ((in->major_version > out->major_version)
      || (in->major_version == out->major_version
	  && in->minor_version > out->minor_version)
      || (out->major_version == RISCV_UNKNOWN_VERSION))
    {
      out->major_version = in->major_version;
      out->minor_version = in->minor_version;
      out->is_implicit &= in->is_implicit;
    }
}

/* Return true if subset is 'i' or 'e'.  */

static bool
riscv_i_or_e_p (bfd *ibfd,
		const char *arch,
		struct riscv_subset_t *subset)
{
  if ((strcasecmp (subset->name, "e") != 0)
      && (strcasecmp (subset->name, "i") != 0))
    {
      _bfd_error_handler
	(_("error: %pB: corrupted ISA string '%s'.  "
	   "First letter should be 'i' or 'e' but got '%s'"),
	   ibfd, arch, subset->name);
      return false;
    }
  return true;
}

/* Merge standard extensions.

   Return Value:
     Return false if failed to merge.

   Arguments:
     `bfd`: bfd handler.
     `in_arch`: Raw ISA string for input object.
     `out_arch`: Raw ISA string for output object.
     `pin`: Subset list for input object.
     `pout`: Subset list for output object.  */

static bool
riscv_merge_std_ext (bfd *ibfd,
		     const char *in_arch,
		     const char *out_arch,
		     struct riscv_subset_t **pin,
		     struct riscv_subset_t **pout)
{
  const char *standard_exts = "mafdqlcbjtpvn";
  const char *p;
  struct riscv_subset_t *in = *pin;
  struct riscv_subset_t *out = *pout;

  /* First letter should be 'i' or 'e'.  */
  if (!riscv_i_or_e_p (ibfd, in_arch, in))
    return false;

  if (!riscv_i_or_e_p (ibfd, out_arch, out))
    return false;

  if (strcasecmp (in->name, out->name) != 0)
    {
      /* TODO: We might allow merge 'i' with 'e'.  */
      _bfd_error_handler
	(_("error: %pB: mis-matched ISA string to merge '%s' and '%s'"),
	 ibfd, in->name, out->name);
      return false;
    }

  riscv_update_subset_version(in, out);
  riscv_add_subset_ext (&merged_subsets,
			out->name, out->major_version, out->minor_version,
			out->is_implicit);

  in = in->next;
  out = out->next;

  /* Handle standard extension first.  */
  for (p = standard_exts; *p; ++p)
    {
      struct riscv_subset_t *ext_in, *ext_out, *ext_merged;
      char find_ext[2] = {*p, '\0'};
      bool find_in, find_out;

      find_in = riscv_lookup_subset (&in_subsets, find_ext, &ext_in);
      find_out = riscv_lookup_subset (&out_subsets, find_ext, &ext_out);

      if (!find_in && !find_out)
	continue;

      if (find_in && find_out)
	riscv_update_subset_version(ext_in, ext_out);

      ext_merged = find_out ? ext_out : ext_in;
      riscv_add_subset_ext (&merged_subsets, ext_merged->name,
			    ext_merged->major_version,
			    ext_merged->minor_version,
			    ext_merged->is_implicit);
    }

  /* Skip all standard extensions.  */
  while ((in != NULL) && riscv_std_ext_p (in->name)) in = in->next;
  while ((out != NULL) && riscv_std_ext_p (out->name)) out = out->next;

  *pin = in;
  *pout = out;

  return true;
}

/* Merge multi letter extensions.  PIN is a pointer to the head of the input
   object subset list.  Likewise for POUT and the output object.  Return true
   on success and false when a conflict is found.  */

static bool
riscv_merge_multi_letter_ext (riscv_subset_t **pin,
			      riscv_subset_t **pout)
{
  riscv_subset_t *in = *pin;
  riscv_subset_t *out = *pout;
  riscv_subset_t *tail;

  int cmp;

  while (in && out)
    {
      cmp = riscv_compare_subsets (in->name, out->name);

      if (cmp < 0)
	{
	  /* `in' comes before `out', append `in' and increment.  */
	  riscv_add_subset_ext (&merged_subsets, in->name,
				in->major_version, in->minor_version,
				in->is_implicit);
	  in = in->next;
	}
      else if (cmp > 0)
	{
	  /* `out' comes before `in', append `out' and increment.  */
	  riscv_add_subset_ext (&merged_subsets, out->name,
				out->major_version, out->minor_version,
				out->is_implicit);
	  out = out->next;
	}
      else
	{
	  /* Both present, check version and increment both.  */
	  riscv_update_subset_version (in, out);

	  riscv_add_subset_ext (&merged_subsets, out->name,
				out->major_version, out->minor_version,
				out->is_implicit);
	  out = out->next;
	  in = in->next;
	}
    }

  if (in || out)
    {
      /* If we're here, either `in' or `out' is running longer than
	 the other. So, we need to append the corresponding tail.  */
      tail = in ? in : out;
      while (tail)
	{
	  riscv_add_subset_ext (&merged_subsets, tail->name,
				tail->major_version, tail->minor_version,
				tail->is_implicit);
	  tail = tail->next;
	}
    }

  return true;
}

#if 0
/* add a subset to Tag_RISCV_arch attribute.  */

static char *
riscv_add_arch_attr_subset (char *arch, char *subset, uint state)
{
  char *merged_arch;
  unsigned xlen;

  riscv_parse_subset_t riscv_rps =
    {&out_subsets, _bfd_error_handler, _bfd_error_handler, &xlen, NULL, false,
     state, false};

  if (arch == NULL || subset == NULL)
    return NULL;

  /* Parse arch string.  */
  if (!riscv_parse_subset (&riscv_rps, arch))
    return NULL;

  riscv_parse_add_subset (&riscv_rps, subset, RISCV_UNKNOWN_VERSION,
			  RISCV_UNKNOWN_VERSION, false);

  merged_arch = riscv_arch_str (xlen, &out_subsets);

  /* Release the subset lists.  */
  riscv_release_subset_list (&out_subsets);

  return merged_arch;
}
#endif

/* Merge Tag_RISCV_arch attribute.  */

static char *
riscv_merge_arch_attr_info (bfd *ibfd, char *in_arch, char *out_arch)
{
  riscv_subset_t *in, *out;
  char *merged_arch_str;
  unsigned xlen_in, xlen_out;

  /* clean data.  */
  if (in_subsets.head)
    riscv_release_subset_list (&in_subsets);
  if (out_subsets.head)
    riscv_release_subset_list (&out_subsets);
  if (merged_subsets.head)
    riscv_release_subset_list (&merged_subsets);
  BFD_ASSERT (merged_subsets.tail == NULL);

  BFD_ASSERT (nsta.opt);
  bool enabled_execit = nsta.opt->target_optimization & RISCV_RELAX_EXECIT_ON;
  riscv_parse_subset_t riscv_rps_ld_in =
    {&in_subsets, _bfd_error_handler, _bfd_error_handler, &xlen_in, NULL, false,
     STATE_LINK, enabled_execit};
  riscv_parse_subset_t riscv_rps_ld_out =
    {&out_subsets, _bfd_error_handler, _bfd_error_handler, &xlen_out, NULL, false,
     STATE_LINK, enabled_execit};

  if (in_arch == NULL && out_arch == NULL)
    return NULL;
  if (in_arch == NULL && out_arch != NULL)
    return out_arch;
  if (in_arch != NULL && out_arch == NULL)
    return in_arch;

  /* Parse subset from ISA string.  */
  if (!riscv_parse_subset (&riscv_rps_ld_in, in_arch))
    return NULL;
  if (!riscv_parse_subset (&riscv_rps_ld_out, out_arch))
    return NULL;

  /* Checking XLEN.  */
  if (xlen_out != xlen_in)
    {
      _bfd_error_handler
	(_("error: %pB: ISA string of input (%s) doesn't match "
	   "output (%s)"), ibfd, in_arch, out_arch);
      return NULL;
    }

  /* Merge subset list.  */
  in = in_subsets.head;
  out = out_subsets.head;

  /* Merge standard extension.  */
  if (!riscv_merge_std_ext (ibfd, in_arch, out_arch, &in, &out))
    return NULL;

  /* Merge all non-single letter extensions with single call.  */
  if (!riscv_merge_multi_letter_ext (&in, &out))
    return NULL;

  if (xlen_in != xlen_out)
    {
      _bfd_error_handler
	(_("error: %pB: XLEN of input (%u) doesn't match "
	   "output (%u)"), ibfd, xlen_in, xlen_out);
      return NULL;
    }

  if (xlen_in != ARCH_SIZE)
    {
      _bfd_error_handler
	(_("error: %pB: unsupported XLEN (%u), you might be "
	   "using wrong emulation"), ibfd, xlen_in);
      return NULL;
    }

#if 0
  /* zcb + xandes + mexecit => xnexecit  */
  if (enabled_execit)
    {
      riscv_rps_ld_out.subset_list = &merged_subsets;
      if (riscv_subset_supports (&riscv_rps_ld_out, "zcb")
	  && riscv_subset_supports (&riscv_rps_ld_out, "xandes"))
	riscv_parse_add_subset (&riscv_rps_ld_out, "xnexecit", 1, 0, false);
    }
#endif

  merged_arch_str = riscv_arch_str_ext (ARCH_SIZE, &merged_subsets,
					false, ISA_SPEC_CLASS_2P2);

  /* Release the subset lists.  */
  riscv_release_subset_list (&in_subsets);
  riscv_release_subset_list (&out_subsets);
  riscv_release_subset_list (&merged_subsets);

  return merged_arch_str;
}

/* Merge object attributes from IBFD into output_bfd of INFO.
   Raise an error if there are conflicting attributes.  */

static bool
riscv_merge_attributes (bfd *ibfd, struct bfd_link_info *info)
{
  bfd *obfd = info->output_bfd;
  obj_attribute *in_attr;
  obj_attribute *out_attr;
  obj_attribute_list *in_attr_list;
  obj_attribute_list *out_attr_list;
  bool result = true;
  bool priv_attrs_merged = false;
  const char *sec_name = get_elf_backend_data (ibfd)->obj_attrs_section;
  unsigned int i;

  /* Skip linker created files.  */
  if (ibfd->flags & BFD_LINKER_CREATED)
    return true;

  /* Skip any input that doesn't have an attribute section.
     This enables to link object files without attribute section with
     any others.  */
  if (bfd_get_section_by_name (ibfd, sec_name) == NULL)
    return true;

  if (!elf_known_obj_attributes_proc (obfd)[0].i)
    {
      /* This is the first object.  Copy the attributes.  */
      _bfd_elf_copy_obj_attributes (ibfd, obfd);

      out_attr = elf_known_obj_attributes_proc (obfd);

      /* Use the Tag_null value to indicate the attributes have been
	 initialized.  */
      out_attr[0].i = 1;

      /* The function _bfd_elf_copy_obj_attributes only copy the attrs
	 from input bfd to output bfd. If we only have an input bfd,
	 we need to store the information of ict_model here.  */
      for (out_attr_list = elf_other_obj_attributes (obfd)[OBJ_ATTR_PROC];
	   out_attr_list;
	   out_attr_list = out_attr_list->next)
	{
	  if (out_attr_list->tag == Tag_RISCV_ict_model)
	    {
	      if (strcmp (out_attr_list->attr.s, "tiny") == 0)
		ict_model = 0;
	      else if (strcmp (out_attr_list->attr.s, "small") == 0)
		ict_model = 1;
	      else if (strcmp (out_attr_list->attr.s, "large") == 0)
		ict_model = 2;
	    }
	}

      /* in case single one file linkage, merge self arch to
	 apply implict rules.  */
      if (out_attr[Tag_RISCV_arch].s)
	{
	  char buf[0x100];
	  char *merged_arch = out_attr[Tag_RISCV_arch].s;
	  if (nsta.opt->execit_flags.nexecit_op)
	    {
	      char *us = strstr (merged_arch, "_");
	      if (us)
		{
		  int len = us - merged_arch;
		  strncpy (buf, merged_arch, len);
		  sprintf (&buf[len], "_xnexecit");
		  merged_arch = buf;
		}
	    }

	  merged_arch =
		riscv_merge_arch_attr_info (ibfd,
					    out_attr[Tag_RISCV_arch].s,
					    merged_arch);
	    if (merged_arch == NULL)
	      {
		result = false;
		out_attr[Tag_RISCV_arch].s = "";
	      }
	    else /* TODO: free old arch string?  */
	      out_attr[Tag_RISCV_arch].s = merged_arch;
	}

      return result;
    }

  in_attr = elf_known_obj_attributes_proc (ibfd);
  out_attr = elf_known_obj_attributes_proc (obfd);

  for (i = LEAST_KNOWN_OBJ_ATTRIBUTE; i < NUM_KNOWN_OBJ_ATTRIBUTES; i++)
    {
    switch (i)
      {
      case Tag_RISCV_arch:
	if (!out_attr[Tag_RISCV_arch].s)
	  out_attr[Tag_RISCV_arch].s = in_attr[Tag_RISCV_arch].s;
	else if (in_attr[Tag_RISCV_arch].s
		 && out_attr[Tag_RISCV_arch].s)
	  {
	    /* Check compatible.  */
	    char *merged_arch =
		riscv_merge_arch_attr_info (ibfd,
					    in_attr[Tag_RISCV_arch].s,
					    out_attr[Tag_RISCV_arch].s);
	    if (merged_arch == NULL)
	      {
		result = false;
		out_attr[Tag_RISCV_arch].s = "";
	      }
	    else
	      out_attr[Tag_RISCV_arch].s = merged_arch;
	  }
	break;

      case Tag_RISCV_priv_spec:
      case Tag_RISCV_priv_spec_minor:
      case Tag_RISCV_priv_spec_revision:
	/* If we have handled the privileged elf attributes, then skip it.  */
	if (!priv_attrs_merged)
	  {
	    unsigned int Tag_a = Tag_RISCV_priv_spec;
	    unsigned int Tag_b = Tag_RISCV_priv_spec_minor;
	    unsigned int Tag_c = Tag_RISCV_priv_spec_revision;
	    enum riscv_spec_class in_priv_spec = PRIV_SPEC_CLASS_NONE;
	    enum riscv_spec_class out_priv_spec = PRIV_SPEC_CLASS_NONE;

	    /* Get the privileged spec class from elf attributes.  */
	    riscv_get_priv_spec_class_from_numbers (in_attr[Tag_a].i,
						    in_attr[Tag_b].i,
						    in_attr[Tag_c].i,
						    &in_priv_spec);
	    riscv_get_priv_spec_class_from_numbers (out_attr[Tag_a].i,
						    out_attr[Tag_b].i,
						    out_attr[Tag_c].i,
						    &out_priv_spec);

	    /* Allow to link the object without the privileged specs.  */
	    if (out_priv_spec == PRIV_SPEC_CLASS_NONE)
	      {
		out_attr[Tag_a].i = in_attr[Tag_a].i;
		out_attr[Tag_b].i = in_attr[Tag_b].i;
		out_attr[Tag_c].i = in_attr[Tag_c].i;
	      }
	    else if (in_priv_spec != PRIV_SPEC_CLASS_NONE
		     && in_priv_spec != out_priv_spec)
	      {
		_bfd_error_handler
		  (_("warning: %pB use privileged spec version %u.%u.%u but "
		     "the output use version %u.%u.%u"),
		   ibfd,
		   in_attr[Tag_a].i,
		   in_attr[Tag_b].i,
		   in_attr[Tag_c].i,
		   out_attr[Tag_a].i,
		   out_attr[Tag_b].i,
		   out_attr[Tag_c].i);

		/* The privileged spec v1.9.1 can not be linked with others
		   since the conflicts, so we plan to drop it in a year or
		   two.  */
		if (in_priv_spec == PRIV_SPEC_CLASS_1P9P1
		    || out_priv_spec == PRIV_SPEC_CLASS_1P9P1)
		  {
		    _bfd_error_handler
		      (_("warning: privileged spec version 1.9.1 can not be "
			 "linked with other spec versions"));
		  }

		/* Update the output privileged spec to the newest one.  */
		if (in_priv_spec > out_priv_spec)
		  {
		    out_attr[Tag_a].i = in_attr[Tag_a].i;
		    out_attr[Tag_b].i = in_attr[Tag_b].i;
		    out_attr[Tag_c].i = in_attr[Tag_c].i;
		  }
	      }
	    priv_attrs_merged = true;
	  }
	break;

      case Tag_RISCV_unaligned_access:
	out_attr[i].i |= in_attr[i].i;
	break;

      case Tag_RISCV_stack_align:
	if (out_attr[i].i == 0)
	  out_attr[i].i = in_attr[i].i;
	else if (in_attr[i].i != 0
		 && out_attr[i].i != 0
		 && out_attr[i].i != in_attr[i].i)
	  {
	    _bfd_error_handler
	      (_("error: %pB use %u-byte stack aligned but the output "
		 "use %u-byte stack aligned"),
	       ibfd, in_attr[i].i, out_attr[i].i);
	    result = false;
	  }
	break;

      default:
	result &= _bfd_elf_merge_unknown_attribute_low (ibfd, obfd, i);
      }

      /* If out_attr was copied from in_attr then it won't have a type yet.  */
      if (i >= LEAST_KNOWN_OBJ_ATTRIBUTE
	  && i < NUM_KNOWN_OBJ_ATTRIBUTES
	  && in_attr[i].type
	  && !out_attr[i].type)
	out_attr[i].type = in_attr[i].type;
    }

  /* Handle our v5 attributes (unknown attributes).  */
  /* I recommend that don't use the riscv_elf_obj_attrs_handle_unknown
     for checking our v5 attributes, since the function only pass the
     information of input bfd.  */
  for (in_attr_list = elf_other_obj_attributes (ibfd)[OBJ_ATTR_PROC];
       in_attr_list;
       in_attr_list = in_attr_list->next)
    {
      for (out_attr_list = elf_other_obj_attributes (obfd)[OBJ_ATTR_PROC];
	   out_attr_list;
	   out_attr_list = out_attr_list->next)
	{
	  if (in_attr_list->tag == out_attr_list->tag)
	    {
	      /* Find the attr both in the input and output bfds.  */
	      switch (in_attr_list->tag)
		{
		case Tag_RISCV_ict_version:
		  if (in_attr_list->attr.i
		      != out_attr_list->attr.i)
		    {
		      _bfd_error_handler
			(_("error: %pB: conflicting ict version %d, "
			   "the output ict version is %d."),
			 ibfd, in_attr_list->attr.i,
			 out_attr_list->attr.i);
		      result = false;
		    }
		  break;
		case Tag_RISCV_ict_model:
		  if (strcmp (in_attr_list->attr.s,
			      out_attr_list->attr.s) != 0)
		    {
		      _bfd_error_handler
			(_("error: %pB: conflicting ict model %s, "
			   "the output ict model is %s."),
			 ibfd, in_attr_list->attr.s,
			 out_attr_list->attr.s);
		      result = false;
		    }
		  /* The information of ict_model is recorded when linking
		     the first input bfd.  */
		  break;
		default:
		  _bfd_error_handler
		    (_("Warning: %pB: Unknown RISC-V object attribute %d"),
		     ibfd, in_attr_list->tag);
		  result = false;
		  break;
		}
	      break;
	    }
	}
      if (out_attr_list == NULL)
	{
	  /* Can not find the input attr in the output, so insert it
	     into the output bfd.  */
	  switch (in_attr_list->tag)
	    {
	    case Tag_RISCV_ict_version:
	      bfd_elf_add_obj_attr_int (obfd, OBJ_ATTR_PROC,
					in_attr_list->tag,
					in_attr_list->attr.i);
	      break;
	    case Tag_RISCV_ict_model:
	      bfd_elf_add_obj_attr_string (obfd, OBJ_ATTR_PROC,
					   in_attr_list->tag,
					   in_attr_list->attr.s);

	      /* Also store the information of ict_model here.  */
	      if (strcmp (in_attr_list->attr.s, "tiny") == 0)
		ict_model = 0;
	      else if (strcmp (in_attr_list->attr.s, "small") == 0)
		ict_model = 1;
	      else if (strcmp (in_attr_list->attr.s, "large") == 0)
		ict_model = 2;
	      break;
	    default:
	      _bfd_error_handler
		(_("Warning: %pB: Unknown RISC-V object attribute %d"),
		 ibfd, in_attr_list->tag);
	      result = false;
	      break;
	    }
	}
    }

  /* Merge Tag_compatibility attributes and any common GNU ones.  */
  if (!_bfd_elf_merge_object_attributes (ibfd, info))
    return false;

  return result;
}

/* Merge backend specific data from an object file to the output
   object file when linking.  */

static bool
_bfd_riscv_elf_merge_private_bfd_data (bfd *ibfd, struct bfd_link_info *info)
{
  bfd *obfd = info->output_bfd;
  flagword new_flags, old_flags;

  /* { Andes  */
  /* TODO: init `andes' ASAP in better timing. */
  if (nsta.opt == NULL)
    {
      struct riscv_elf_link_hash_table *htab = riscv_elf_hash_table (info);
      memset (&nsta, 0, sizeof (nsta));
      nsta.opt = &htab->andes;
    }
  /* } Andes  */

  if (!is_riscv_elf (ibfd) || !is_riscv_elf (obfd))
    return true;

  if (strcmp (bfd_get_target (ibfd), bfd_get_target (obfd)) != 0)
    {
      (*_bfd_error_handler)
	(_("%pB: ABI is incompatible with that of the selected emulation:\n"
	   "  target emulation `%s' does not match `%s'"),
	 ibfd, bfd_get_target (ibfd), bfd_get_target (obfd));
      return false;
    }

  if (!_bfd_elf_merge_object_attributes (ibfd, info))
    return false;

  if (!riscv_merge_attributes (ibfd, info))
    return false;

  /* Check to see if the input BFD actually contains any sections.  If not,
     its flags may not have been initialized either, but it cannot actually
     cause any incompatibility.  Do not short-circuit dynamic objects; their
     section list may be emptied by elf_link_add_object_symbols.

     Also check to see if there are no code sections in the input.  In this
     case, there is no need to check for code specific flags.  */
  if (!(ibfd->flags & DYNAMIC))
    {
      bool null_input_bfd = true;
      bool only_data_sections = true;
      asection *sec;

      for (sec = ibfd->sections; sec != NULL; sec = sec->next)
	{
	  null_input_bfd = false;

	  if ((bfd_section_flags (sec)
	       & (SEC_LOAD | SEC_CODE | SEC_HAS_CONTENTS))
	      == (SEC_LOAD | SEC_CODE | SEC_HAS_CONTENTS))
	    {
	      only_data_sections = false;
	      break;
	    }
	}

      if (null_input_bfd || only_data_sections)
	return true;
    }

  new_flags = elf_elfheader (ibfd)->e_flags;
  old_flags = elf_elfheader (obfd)->e_flags;

  if (!elf_flags_init (obfd))
    {
      elf_flags_init (obfd) = true;
      elf_elfheader (obfd)->e_flags = new_flags;
      return true;
    }

  /* Disallow linking different float ABIs.  */
  if ((old_flags ^ new_flags) & EF_RISCV_FLOAT_ABI)
    {
      (*_bfd_error_handler)
	(_("%pB: can't link %s modules with %s modules"), ibfd,
	 riscv_float_abi_string (new_flags),
	 riscv_float_abi_string (old_flags));
      goto fail;
    }

  /* Disallow linking RVE and non-RVE.  */
  if ((old_flags ^ new_flags) & EF_RISCV_RVE)
    {
      (*_bfd_error_handler)
       (_("%pB: can't link RVE with other target"), ibfd);
      goto fail;
    }

  /* Allow linking RVC and non-RVC, and keep the RVC flag.  */
  elf_elfheader (obfd)->e_flags |= new_flags & EF_RISCV_RVC;

  return true;

 fail:
  bfd_set_error (bfd_error_bad_value);
  return false;
}

/* A second format for recording PC-relative hi relocations.  This stores the
   information required to relax them to GP-relative addresses.  */

typedef struct riscv_pcgp_hi_reloc riscv_pcgp_hi_reloc;
struct riscv_pcgp_hi_reloc
{
  bfd_vma hi_sec_off;
  bfd_vma hi_addend;
  bfd_vma hi_addr;
  unsigned hi_sym;
  asection *sym_sec;
  bool undefined_weak;
  riscv_pcgp_hi_reloc *next;
  /* { Andes */
  Elf_Internal_Rela *rel;
  int is_deleted:1;
  int is_marked:1;  /* by the lo12 pal  */
  /* } Andes */
};

typedef struct riscv_pcgp_lo_reloc riscv_pcgp_lo_reloc;
struct riscv_pcgp_lo_reloc
{
  bfd_vma hi_sec_off;
  riscv_pcgp_lo_reloc *next;
};

typedef struct
{
  riscv_pcgp_hi_reloc *hi;
  riscv_pcgp_lo_reloc *lo;
} riscv_pcgp_relocs;

/* { Andes  */
#define IS_RVC_INSN(x) (((x) & 0x3) != 0x3)
static int
andes_try_target_align (bfd *abfd, asection *sec,
			asection *sym_sec ATTRIBUTE_UNUSED,
			struct bfd_link_info *link_info,
			Elf_Internal_Rela *rel,
			bfd_vma offset, bfd_vma end);
static int
riscv_convert_16_to_32 (uint16_t insn16, uint32_t *insn32);
static bool
riscv_convert_16_to_32_reloc (Elf_Internal_Rela **irel);
static bool
target_align_check_branch_range (bfd *abfd, asection *sec, bfd_vma insn16_off,
				 bfd_vma nops_off, size_t count,
				 struct bfd_link_info *link_info);
static bool
riscv_relax_shift_bytes (bfd *abfd, asection *sec, bfd_vma insn16_off,
			 bfd_vma nops_off, size_t count, uint32_t insn32);
static bool
riscv_relax_avoid_BTB_miss (bfd *abfd, asection *sec, Elf_Internal_Rela *rel);
static bool
btb_miss_occur (bfd_vma return_address, bfd_vma branch_end);
static bool
riscv_relax_check_BTB_miss (bfd *abfd, asection *sec, Elf_Internal_Rela *rel);
static bool
_bfd_riscv_relax_align_btb (bfd *abfd, asection *sec,
			    asection *sym_sec,
			    struct bfd_link_info *link_info,
			    Elf_Internal_Rela *rel,
			    bfd_vma symval,
			    bfd_vma max_alignment ATTRIBUTE_UNUSED,
			    bfd_vma reserve_size ATTRIBUTE_UNUSED,
			    bool *again ATTRIBUTE_UNUSED,
			    riscv_pcgp_relocs *pcgp_relocs ATTRIBUTE_UNUSED,
			    bool undefined_weak ATTRIBUTE_UNUSED);
static bool
_bfd_riscv_relax_lui_gp_insn (bfd *abfd, asection *sec, asection *sym_sec,
			      struct bfd_link_info *link_info,
			      Elf_Internal_Rela *rel,
			      bfd_vma symval,
			      bfd_vma max_alignment,
			      bfd_vma reserve_size,
			      bool *again ATTRIBUTE_UNUSED,
			      riscv_pcgp_relocs *pcgp_relocs ATTRIBUTE_UNUSED,
			      bool undefined_weak ATTRIBUTE_UNUSED);
static int
andes_execit_render_hash (execit_context_t *ctx);
static void
riscv_elf_get_insn_with_reg (const bfd* abfd, const Elf_Internal_Rela *irel,
			     uint32_t insn, uint32_t *insn_with_reg);
static int
riscv_get_local_syms (const bfd *abfd, asection *sec ATTRIBUTE_UNUSED,
		      Elf_Internal_Sym **isymbuf_p);
static bool
execit_check_pchi_for_jal (bfd_vma relocation, bfd_vma insn_pc);
static bfd_vma
riscv_data_start_value (const struct bfd_link_info *info);
static bool
andes_execit_hash_insn (bfd *abfd, asection *sec,
			struct bfd_link_info *link_info);
static void
andes_execit_traverse_insn_hash (int (*func) (execit_hash_t*));
static int
andes_execit_rank_insn (execit_hash_t *he);
static int
andes_execit_rank_imported_insn (execit_hash_t *he);
static void
andes_execit_build_itable (struct bfd_link_info *info);
static bool
andes_execit_replace_insn (struct bfd_link_info *link_info,
			   bfd *abfd, asection *sec);
static void
andes_execit_delete_blank (struct bfd_link_info *info);
static asection*
andes_execit_get_itable_section (bfd *input_bfds);
#if 0
static asection*
andes_table_jump_get_section (struct bfd_link_info *info);
#endif
static int
riscv_get_section_contents (bfd *abfd, asection *sec,
			    bfd_byte **contents_p, bool cache);
static Elf_Internal_Rela *
find_relocs_at_address (Elf_Internal_Rela *reloc,
			Elf_Internal_Rela *relocs,
			Elf_Internal_Rela *irelend,
			enum elf_riscv_reloc_type reloc_type);
static int
riscv_relocation_check (struct bfd_link_info *info,
			Elf_Internal_Rela **irel,
			Elf_Internal_Rela *irelend,
			asection *sec, bfd_vma *off,
			bfd_byte *contents, int optimize);
static bool
riscv_elf_execit_check_insn_available (uint32_t insn,
				       struct riscv_elf_link_hash_table *htab);
static int
list_iterate (list_entry_t **lst, void *obj,
	      list_iter_cb_t each, list_iter_cb_t final);
static int
append_final_cb (list_entry_t **lst, list_entry_t *j,
		 list_entry_t *p, list_entry_t *q);
#if 0 //KILLME
static int 
free_each_cb (void *l ATTRIBUTE_UNUSED, void *j ATTRIBUTE_UNUSED,
	      void *p ATTRIBUTE_UNUSED, execit_vma_t *q);
static void
andes_execit_estimate_lui (execit_hash_t *he, execit_vma_t **lst_pp);
#endif
static void
andes_execit_estimate_hi20 (execit_hash_t *he);
static int
rank_each_cb (void *l ATTRIBUTE_UNUSED, execit_rank_t *j, execit_rank_t *p,
	      void *q ATTRIBUTE_UNUSED);
static bool
andes_execit_push_insn (execit_context_t *ctx, execit_hash_t* h);
#if 0 //KILLME
static int 
andes_execit_estimate_lui_each_cb (void *l ATTRIBUTE_UNUSED,
				   void *j ATTRIBUTE_UNUSED,
				   execit_irel_t *p,
				   void *q ATTRIBUTE_UNUSED);
static int 
collect_lui_vma_each_cb (void *l, void *j_pp, execit_irel_t *p,
			 void *q ATTRIBUTE_UNUSED);
static int 
insert_vma_each_cb (void *l ATTRIBUTE_UNUSED, execit_vma_t *j,
		    execit_vma_t *p, void *q ATTRIBUTE_UNUSED);
static int 
insert_vma_final_cb (void *l, execit_vma_t *j, execit_vma_t *p, void *q);
#endif
static execit_itable_t *
andes_execit_itable_lookup (execit_context_t *ctx,
			    execit_hash_t* h);
static bool
execit_push_blank (execit_context_t *ctx, bfd_vma delta, bfd_vma size);
static bool
andes_execit_mark_irel (Elf_Internal_Rela *irel, int index);
static andes_irelx_t*
andes_extend_irel (Elf_Internal_Rela *irel, int subtype,
		   andes_irelx_t **list);
static execit_blank_abfd_t*
execit_lookup_blank_abfd (execit_context_t *ctx);
static execit_blank_section_t*
execit_lookup_blank_section (execit_context_t *ctx,
			     execit_blank_abfd_t *blank_abfd);
static bool
andes_relax_pc_gp_insn (
  bfd *abfd,
  asection *sec,
  asection *sym_sec,
  struct bfd_link_info *info,
  Elf_Internal_Rela *rel,
  bfd_vma symval,
  bfd_vma max_alignment,
  bfd_vma reserve_size,
  bool *again ATTRIBUTE_UNUSED,
  riscv_pcgp_relocs *pcgp_relocs,
  bool undefined_weak);
static bool
riscv_mark_pcgp_hi_reloc (riscv_pcgp_relocs *p, riscv_pcgp_hi_reloc *hi);
static void
andes_relax_pc_gp_insn_final (riscv_pcgp_relocs *p);

/* Exec.it hash function.  */

static struct bfd_hash_entry *
riscv_elf_code_hash_newfunc (struct bfd_hash_entry *entry,
			     struct bfd_hash_table *table,
			     const char *string ATTRIBUTE_UNUSED)
{
  static int id = 0;
  const size_t sz_entry = sizeof (execit_hash_t);
  const size_t sz_head = sizeof (struct bfd_hash_entry);
  const size_t sz_body = sz_entry - sz_head;

  /* Allocate the structure if it has not already been
     allocated by a subclass.  */
  if (entry == NULL)
    {
      entry = (void *)
	bfd_hash_allocate (table, sz_entry);

      if (entry == NULL)
	return entry;
    }

  memset ((void *) entry + sz_head, 0, sz_body);
  ((execit_hash_t*) entry)->id = id++;

  return entry;
}

/* Initialize EXECIT hash table.  */

static int
andes_execit_init (struct bfd_link_info *info)
{
  /* init execit code hash  */
  if (!bfd_hash_table_init_n (&execit.code_hash, riscv_elf_code_hash_newfunc,
			      sizeof (execit_hash_t),
			      1023))
    {
      (*_bfd_error_handler) (_("Linker: cannot init EXECIT hash table error \n"));
      return false;
    }

  /* get the first 2M-windown base for JAL  */
  /* Traverse all output sections and return the min SHF_EXECINSTR addr.
     the sh_flags of output bfd by now is not finalized,
     check input bfd's instead.  */
  if (true)
    {
      bfd_vma min_execinstr_addr = -1u;
      bfd *ibfd;
      for (ibfd = info->input_bfds; ibfd ; ibfd = ibfd->link.next)
	{
	  asection *isec, *osec;
	  bfd_vma base;
	  for (isec = ibfd->sections; isec != NULL; isec = isec->next)
	    {
	      Elf_Internal_Shdr *shdr = &(elf_section_data(isec)->this_hdr);
	      if ((shdr->sh_flags & SHF_EXECINSTR) == 0)
		continue;
	      /* osec->output_offset is 0 by now, while
	       * isec->output_offset is the addr before relaxation.
	       */
	      osec = isec->output_section;
	      base = osec->vma + isec->output_offset;
	      if (min_execinstr_addr > base)
		min_execinstr_addr = base;
	    }
	}
	execit.jal_window_end = MASK_2M | min_execinstr_addr;
    }

  /* sanity check  */
  BFD_ASSERT (execit.jal_window_end);
  if (!execit.jal_window_end)
    execit.jal_window_end = MASK_2M;

  return true;
}

/* Import .exec.itable and then build list.  */

static void
riscv_elf_execit_import_table (bfd *abfd ATTRIBUTE_UNUSED, struct bfd_link_info *info ATTRIBUTE_UNUSED)
{
  int num = 0;
  bfd_byte buf[0x10];
  bfd_byte *contents;
  unsigned long insn;
  FILE *execit_import_file;
  struct riscv_elf_link_hash_table *htab;
  andes_ld_options_t *andes;

  htab = riscv_elf_hash_table (info);
  andes = &htab->andes;
  execit_import_file = andes->execit_import_file;
  rewind (andes->execit_import_file);

  contents = &buf[0];

  /* Read instructions from the input file, and then build the list.  */
  while (!feof (execit_import_file))
    {
      /* insert exec.it entry to hash  */
      execit_context_t ctx;
      execit_hash_t *he;
      size_t nread;

      const char *hash = ctx.buf;
      memset (&ctx, 0, sizeof (ctx));

      nread = fread (contents, sizeof (bfd_byte) * 4, 1, execit_import_file);
      /* Ignore the final byte 0x0a.  */
      if (nread < 1)
	break;
      insn = bfd_getl32 (contents);

      ctx.ie.insn = insn;
      if (andes_execit_render_hash (&ctx) != EXECIT_HASH_OK)
	{
	  BFD_ASSERT (0);
	  continue;
	}

      /* add hash entry.  */
      he = (execit_hash_t*)
	bfd_hash_lookup (&execit.code_hash, hash, true, true);
      if (he == NULL)
	{
	  (*_bfd_error_handler)
	    (_("Linker: failed import exec.it %s hash table\n"), hash);
	  continue;
	}
      else
	{
	  he->is_imported = 1;
	  he->is_chosen = 1;
	  he->ie.entries = 1;
	  he->ie.itable_index = execit.next_itable_index++;
	  /* to pass andes_execit_itable_lookup  */
	  he->ie.fixed = ctx.ie.fixed;
	}

      num++;
    }
  fclose (execit_import_file);

  /* Default set the maximun number of the EXECIT entries to 1024.
     There are still 1024 entries in .exec.itable even though the
     EXECIT limit setting exceeds the remaining entries.  */
  execit.import_number = num;
  if (andes->update_execit_table
      && andes->execit_limit != -1
      && (execit.import_number + andes->execit_limit) > EXECIT_HW_ENTRY_MAX)
    (*_bfd_error_handler)
      (_("Warning: There are only %d entries of .exec.itable left for this time."),
       (EXECIT_HW_ENTRY_MAX - execit.import_number));
}

/* Generate EXECIT hash from insn and its relocation.
 * key: "{code_pattern:x}|{rel_section:x}|{rel_offset:x}|{symbol:s}"
 *   code_pattern: opcode|registers
 *   rel_section:  vma of symbol
 *   rel_offset:   offset of symbol, or offset/(lui, auipc)
 *   symbol:       # SYM/global, LAB/local, ABS/constant
 * relocation is separated into section and offset instead of VMA
 * to merge aliases.
 */
static int
andes_execit_render_hash (execit_context_t *ctx)
{
  const bfd_vma off = ctx->off;
  bfd *abfd = ctx->abfd;
  asection *sec = ctx->sec;
  const struct bfd_link_info *info = ctx->info;
  const Elf_Internal_Rela *irel = ctx->irel;
  const uint32_t insn = ctx->ie.insn;
  bfd_vma relocation_section = 0;
  bfd_vma relocation_offset = 0;

  const char *symbol ="ABS";
  int rz = EXECIT_HASH_NG;
  BFD_ASSERT (ctx->ie.fixed || ctx->ie.relocation == 0);
  ctx->buf[0] = 0;
  ctx->ie.pc = off + (sec ? sec_addr(sec) : 0);
  ctx->ie.sec = sec;

  if (irel)
    {
      Elf_Internal_Shdr *symtab_hdr = &elf_tdata (abfd)->symtab_hdr;
      andes_ld_options_t *andes = &execit.htab->andes;
      asection *sym_sec;
      bfd_vma symval;
      char symtype;
      int rtype = ELFNN_R_TYPE (irel->r_info);
      int stype = TAG_NONE; /* subtype of ANDES_TAG.  */
      if (rtype == R_RISCV_ANDES_TAG)
	{
	  andes_irelx_t *relx = (andes_irelx_t *) irel->r_user;
	  int tag = relx->flags;
	  if (tag == TAG_GPREL_SUBTYPE_FLX || tag == TAG_GPREL_SUBTYPE_FSX)
	    stype = tag;
	}
      riscv_elf_get_insn_with_reg (abfd, irel, insn, &ctx->ie.fixed);
      if ((!andes->execit_flags.noji && rtype == R_RISCV_JAL)
	  || (!andes->execit_flags.nols
	      && (rtype == R_RISCV_HI20 || rtype == R_RISCV_LO12_I
		  || rtype == R_RISCV_LO12_S || rtype == R_RISCV_GPREL_I
		  || rtype == R_RISCV_GPREL_S
		  || (rtype >= R_RISCV_LGP18S0 && rtype <= R_RISCV_SGP17S3)
		  || stype != TAG_NONE))
	  || (!andes->execit_flags.no_auipc
	      && (rtype == R_RISCV_CALL || rtype == R_RISCV_PCREL_HI20)))
	{
	  unsigned long r_symndx = ELFNN_R_SYM (irel->r_info);
	  if (r_symndx < symtab_hdr->sh_info)
	    { /* Local symbol.  */
	      Elf_Internal_Sym *isym = NULL;
	      symbol ="LAB";
	      if (!riscv_get_local_syms (abfd, sec, &isym))
		{
		  BFD_ASSERT(0);
		  return rz;
		}
	      asection *isec;
	      unsigned int shndx = isym[r_symndx].st_shndx;
	      bfd_vma st_value = isym[r_symndx].st_value;
	      isec = elf_elfsections (abfd)[shndx]->bfd_section;

	      ctx->ie.addend = st_value + irel->r_addend;
	      ctx->ie.isym = isym + r_symndx;
	      ctx->ie.isym_copy = *ctx->ie.isym;
	      ctx->ie.isec = isec;

	      if (shndx == SHN_UNDEF)
		sym_sec = sec, symval = irel->r_offset;
	      else
		{
		  BFD_ASSERT (shndx < elf_numsections (abfd));
		  sym_sec = isec;
		  symval = st_value;
		}
	      symtype = ELF_ST_TYPE (isym[r_symndx].st_info);
	    }
	  else
	    { /* Global symbol.  */
	      struct elf_link_hash_entry *h;
	      struct elf_link_hash_entry **sym_hashes;
	      unsigned long indx;
	      symbol ="SYM";
	      sym_hashes = elf_sym_hashes (abfd);
	      indx = ELFNN_R_SYM (irel->r_info) - symtab_hdr->sh_info;
	      h = sym_hashes[indx];

	      while (h->root.type == bfd_link_hash_indirect
		     || h->root.type == bfd_link_hash_warning)
		h = (struct elf_link_hash_entry *) h->root.u.i.link;

	      /* TODO: the global symbol _FP_BASE_ should be skipped, too.  */
	      if (h->root.u.def.section->output_section == NULL
		  || (h->root.type != bfd_link_hash_defined
		      && h->root.type != bfd_link_hash_defweak
		      && h->root.type != bfd_link_hash_undefined
		      && h->root.type != bfd_link_hash_undefweak
		      ))
		{
		#ifdef DEBUG_EXECIT
		  printf("%s: skip global symbol.\n", __FUNCTION__);
		#endif
		  return rz;
		}

	      ctx->ie.h = h;
	      if ((h->root.type == bfd_link_hash_defined) ||
		  (h->root.type == bfd_link_hash_defweak))
		{
		  ctx->ie.isec = h->root.u.def.section; /* TODO: rename isec  */
		  ctx->ie.addend = h->root.u.def.value + irel->r_addend;
		}
	      else
		{
		  ctx->ie.isec = (asection*) h->root.u.undef.abfd;
		  ctx->ie.addend = irel->r_addend;
		}

	      if (h->plt.offset != MINUS_ONE)
		{
		  sym_sec = execit.htab->elf.splt;
		  symval = h->plt.offset;
		}
	      else if (h->root.u.def.section->output_section == NULL
		       || (h->root.type != bfd_link_hash_defined
			   && h->root.type != bfd_link_hash_defweak))
		return rz;
	      else
		{
		  symval = h->root.u.def.value;
		  sym_sec = h->root.u.def.section;
		}
	      symtype = h->type;
	    }

	  if (sym_sec->sec_info_type == SEC_INFO_TYPE_MERGE
	      && (sym_sec->flags & SEC_MERGE))
	    {
	      if (symtype == STT_SECTION)
		symval += irel->r_addend;

	      symval = _bfd_merged_section_offset (abfd, &sym_sec,
				elf_section_data (sym_sec)->sec_info,
				symval);

	      if (symtype != STT_SECTION)
		symval += irel->r_addend;
	    }
	  else
	    symval += irel->r_addend;

	  /* section start address might be changed before execit replace stage.
	   * use HOST_ADDR(section)/offset as hashing elements.
	   * Why not use relocation/VMA as it?
	   * the same relocations are not necessary aliases but of the same
	   * section and offset are (almost?) (b23753)  */
	  relocation_section = (intptr_t) ctx->ie.isec;
	  relocation_offset = ctx->ie.addend;

	  symval += sec_addr (sym_sec);
	  ctx->ie.relocation = symval;

	  /* special treaments for certain types of relocations.  */
	  if (rtype == R_RISCV_JAL &&
	      !execit_check_pchi_for_jal (ctx->ie.relocation, ctx->ie.pc))
	    return rz;
	  else if (rtype == R_RISCV_HI20 || rtype == R_RISCV_CALL
		   || rtype == R_RISCV_PCREL_HI20)
	    { /* LUI/AUIPC symbols having the same HI20 part can be exec.ited.
	       * # spliting them into 2 groups by __DATA_BEGIN__ to avoid to
	       * the DATA_SEGMENT_ALIGN issue.  */
	      bfd_vma data_start = riscv_data_start_value (info);
	      relocation_section = 0;
	      relocation_offset = ((ctx->ie.relocation - irel->r_addend)
				   >= data_start) ? 1 : 0;
	      if (rtype == R_RISCV_CALL || rtype == R_RISCV_PCREL_HI20)
		ctx->ie.bias = ctx->ie.relocation - ctx->ie.pc;
	      else if (rtype == R_RISCV_HI20)
		ctx->ie.bias = ctx->ie.relocation;

	      if (ARCH_SIZE > 32 &&
		  !VALID_UTYPE_IMM (RISCV_CONST_HIGH_PART (ctx->ie.bias)))
		return rz;
	    }

	  ctx->ie.irel = ctx->irel;
	  ctx->ie.irel_copy = *ctx->irel;
	  rz = EXECIT_HASH_OK;
	}
    }
  else
    { /* has no reloc  */
      ctx->ie.fixed = ctx->ie.insn;
      rz = EXECIT_HASH_OK;
    }

  if (rz == EXECIT_HASH_OK)
    snprintf (ctx->buf, sizeof(ctx->buf), "%x|%llx|%llx|%s",
	      ctx->ie.fixed,
	      (long long unsigned int)relocation_section,
	      (long long unsigned int)relocation_offset,
	      symbol);
  else
    BFD_ASSERT (irel);

  return rz;
}

/* Get insn with registers according to relocation type.  */

static void
riscv_elf_get_insn_with_reg (const bfd* abfd, const Elf_Internal_Rela *irel,
			     uint32_t insn, uint32_t *insn_with_reg)
{
  reloc_howto_type *howto = NULL;

  if (irel == NULL
      || (ELFNN_R_TYPE (irel->r_info) >= number_of_howto_table))
    {
      *insn_with_reg = insn;
      return;
    }

  howto = riscv_elf_rtype_to_howto ((bfd*) abfd, ELFNN_R_TYPE (irel->r_info));
  *insn_with_reg = insn & (~howto->dst_mask);
}

/* Get the contents of the internal symbol of abfd.  */

static int
riscv_get_local_syms (const bfd *abfd, asection *sec ATTRIBUTE_UNUSED,
		      Elf_Internal_Sym **isymbuf_p)
{
  Elf_Internal_Shdr *symtab_hdr;
  symtab_hdr = &elf_tdata (abfd)->symtab_hdr;

  /* Read this BFD's local symbols if we haven't done so already.  */
  if (*isymbuf_p == NULL && symtab_hdr->sh_info != 0)
    {
      *isymbuf_p = (Elf_Internal_Sym *) symtab_hdr->contents;
      if (*isymbuf_p == NULL)
	{
	  *isymbuf_p = bfd_elf_get_elf_syms ((bfd*)abfd, symtab_hdr,
					     symtab_hdr->sh_info, 0,
					     NULL, NULL, NULL);
	  if (*isymbuf_p == NULL)
	    return false;
	}
    }
  symtab_hdr->contents = (bfd_byte *) (*isymbuf_p);

  return true;
}

/* Check whether the high 11 bits of pc is different from
   the high 11 bits of relocation after EXECIT relaxation.
   Return True if the jal can be replaced with exec.it safely.  */

static bool
execit_check_pchi_for_jal (bfd_vma relocation, bfd_vma insn_pc)
{
  bool result = true;

  if (nsta.opt->execit_jal_over_2m)
    {
      result = ((insn_pc >> 21) == (relocation >> 21));
    }
  else
    {
      /* after relocation, EXECIT_JALs might be distributed across 2M window,
       * which would fail the execit relaxation.
       * so far, only the first 2M window are accepted.
       */
      if ((relocation > execit.jal_window_end) ||
	  (insn_pc > execit.jal_window_end))
	result = false;
    }

  return result;
}

/* } Andes  */

/* Initialize the pcgp reloc info in P.  */

static bool
riscv_init_pcgp_relocs (riscv_pcgp_relocs *p)
{
  p->hi = NULL;
  p->lo = NULL;
  return true;
}

/* Free the pcgp reloc info in P.  */

static void
riscv_free_pcgp_relocs (riscv_pcgp_relocs *p,
			bfd *abfd ATTRIBUTE_UNUSED,
			asection *sec ATTRIBUTE_UNUSED)
{
  riscv_pcgp_hi_reloc *c;
  riscv_pcgp_lo_reloc *l;

  for (c = p->hi; c != NULL; )
    {
      riscv_pcgp_hi_reloc *next = c->next;
      free (c);
      c = next;
    }

  for (l = p->lo; l != NULL; )
    {
      riscv_pcgp_lo_reloc *next = l->next;
      free (l);
      l = next;
    }
}

/* Record pcgp hi part reloc info in P, using HI_SEC_OFF as the lookup index.
   The HI_ADDEND, HI_ADDR, HI_SYM, and SYM_SEC args contain info required to
   relax the corresponding lo part reloc.  */

static bool
riscv_record_pcgp_hi_reloc (riscv_pcgp_relocs *p, bfd_vma hi_sec_off,
			    bfd_vma hi_addend, bfd_vma hi_addr,
			    unsigned hi_sym, asection *sym_sec,
			    bool undefined_weak)
{
  riscv_pcgp_hi_reloc *new = bfd_malloc (sizeof (*new));
  if (!new)
    return false;
  new->hi_sec_off = hi_sec_off;
  new->hi_addend = hi_addend;
  new->hi_addr = hi_addr;
  new->hi_sym = hi_sym;
  new->sym_sec = sym_sec;
  new->undefined_weak = undefined_weak;
  new->next = p->hi;
  p->hi = new;
  return true;
}

static bool
riscv_record_pcgp_hi_reloc_ext (
	riscv_pcgp_relocs *p, bfd_vma hi_sec_off,
	bfd_vma hi_addend, bfd_vma hi_addr,
	unsigned hi_sym, asection *sym_sec,
	bool undefined_weak,
	Elf_Internal_Rela *rel)
{
  bool rz = riscv_record_pcgp_hi_reloc (p, hi_sec_off, hi_addend,
	hi_addr, hi_sym, sym_sec, undefined_weak);
  if (rz)
     p->hi->rel = rel;
  return rz;
}

/* Look up hi part pcgp reloc info in P, using HI_SEC_OFF as the lookup index.
   This is used by a lo part reloc to find the corresponding hi part reloc.  */

static riscv_pcgp_hi_reloc *
riscv_find_pcgp_hi_reloc (riscv_pcgp_relocs *p, bfd_vma hi_sec_off)
{
  riscv_pcgp_hi_reloc *c;

  for (c = p->hi; c != NULL; c = c->next)
    if (c->hi_sec_off == hi_sec_off)
      return c;
  return NULL;
}

/* Record pcgp lo part reloc info in P, using HI_SEC_OFF as the lookup info.
   This is used to record relocs that can't be relaxed.  */

static bool
riscv_record_pcgp_lo_reloc (riscv_pcgp_relocs *p, bfd_vma hi_sec_off)
{
  riscv_pcgp_lo_reloc *new = bfd_malloc (sizeof (*new));
  if (!new)
    return false;
  new->hi_sec_off = hi_sec_off;
  new->next = p->lo;
  p->lo = new;
  return true;
}

static bool
riscv_use_pcgp_hi_reloc(riscv_pcgp_relocs *p, bfd_vma hi_sec_off)
{
  bool out = false;
  riscv_pcgp_hi_reloc *c;

  for (c = p->hi; c != NULL; c = c->next)
    if (c->hi_sec_off == hi_sec_off)
      out = true;

  return out;
}

/* Look up lo part pcgp reloc info in P, using HI_SEC_OFF as the lookup index.
   This is used by a hi part reloc to find the corresponding lo part reloc.  */

static bool
riscv_find_pcgp_lo_reloc (riscv_pcgp_relocs *p, bfd_vma hi_sec_off)
{
  riscv_pcgp_lo_reloc *c;

  for (c = p->lo; c != NULL; c = c->next)
    if (c->hi_sec_off == hi_sec_off)
      return true;
  return false;
}

static void
riscv_update_pcgp_relocs (riscv_pcgp_relocs *p, asection *deleted_sec,
			  bfd_vma deleted_addr, size_t deleted_count)
{
  /* Bytes have already been deleted and toaddr should match the old section
     size for our checks, so adjust it here.  */
  bfd_vma toaddr = deleted_sec->size + deleted_count;
  riscv_pcgp_lo_reloc *l;
  riscv_pcgp_hi_reloc *h;

  /* Update section offsets of corresponding pcrel_hi relocs for the pcrel_lo
     entries where they occur after the deleted bytes.  */
  for (l = p->lo; l != NULL; l = l->next)
    if (l->hi_sec_off > deleted_addr
	&& l->hi_sec_off < toaddr)
      l->hi_sec_off -= deleted_count;

  /* Update both section offsets, and symbol values of pcrel_hi relocs where
     these values occur after the deleted bytes.  */
  for (h = p->hi; h != NULL; h = h->next)
    {
      if (h->hi_sec_off > deleted_addr
	  && h->hi_sec_off < toaddr)
	h->hi_sec_off -= deleted_count;
      if (h->sym_sec == deleted_sec
	  && h->hi_addr > deleted_addr
	  && h->hi_addr < toaddr)
      h->hi_addr -= deleted_count;
    }
}

/* Delete some bytes from a section while relaxing.  */

static bool
riscv_relax_delete_bytes (bfd *abfd,
			  asection *sec,
			  bfd_vma addr,
			  size_t count,
			  struct bfd_link_info *link_info,
			  riscv_pcgp_relocs *p)
{
  unsigned int i, symcount;
  bfd_vma toaddr = sec->size;
  struct elf_link_hash_entry **sym_hashes = elf_sym_hashes (abfd);
  Elf_Internal_Shdr *symtab_hdr = &elf_tdata (abfd)->symtab_hdr;
  unsigned int sec_shndx = _bfd_elf_section_from_bfd_section (abfd, sec);
  struct bfd_elf_section_data *data = elf_section_data (sec);
  bfd_byte *contents = data->this_hdr.contents;

  /* Actually delete the bytes.  */
  sec->size -= count;
  memmove (contents + addr, contents + addr + count, toaddr - addr - count);

  /* Adjust the location of all of the relocs.  Note that we need not
     adjust the addends, since all PC-relative references must be against
     symbols, which we will adjust below.  */
  for (i = 0; i < sec->reloc_count; i++)
    if (data->relocs[i].r_offset > addr && data->relocs[i].r_offset < toaddr)
      data->relocs[i].r_offset -= count;

  /* Adjust the hi_sec_off, and the hi_addr of any entries in the pcgp relocs
     table for which these values occur after the deleted bytes.  */
  if (p)
    riscv_update_pcgp_relocs (p, sec, addr, count);

  /* Adjust the local symbols defined in this section.  */
  for (i = 0; i < symtab_hdr->sh_info; i++)
    {
      Elf_Internal_Sym *sym = (Elf_Internal_Sym *) symtab_hdr->contents + i;
      if (sym->st_shndx == sec_shndx)
	{
	  /* If the symbol is in the range of memory we just moved, we
	     have to adjust its value.  */
	  if (sym->st_value > addr && sym->st_value <= toaddr)
	    sym->st_value -= count;

	  /* If the symbol *spans* the bytes we just deleted (i.e. its
	     *end* is in the moved bytes but its *start* isn't), then we
	     must adjust its size.

	     This test needs to use the original value of st_value, otherwise
	     we might accidentally decrease size when deleting bytes right
	     before the symbol.  But since deleted relocs can't span across
	     symbols, we can't have both a st_value and a st_size decrease,
	     so it is simpler to just use an else.  */
	  else if (sym->st_value <= addr
		   && sym->st_value + sym->st_size > addr
		   && sym->st_value + sym->st_size <= toaddr)
	    sym->st_size -= count;
	}
    }

  /* Now adjust the global symbols defined in this section.  */
  symcount = ((symtab_hdr->sh_size / sizeof (ElfNN_External_Sym))
	      - symtab_hdr->sh_info);

  for (i = 0; i < symcount; i++)
    {
      struct elf_link_hash_entry *sym_hash = sym_hashes[i];

      /* The '--wrap SYMBOL' option is causing a pain when the object file,
	 containing the definition of __wrap_SYMBOL, includes a direct
	 call to SYMBOL as well. Since both __wrap_SYMBOL and SYMBOL reference
	 the same symbol (which is __wrap_SYMBOL), but still exist as two
	 different symbols in 'sym_hashes', we don't want to adjust
	 the global symbol __wrap_SYMBOL twice.

	 The same problem occurs with symbols that are versioned_hidden, as
	 foo becomes an alias for foo@BAR, and hence they need the same
	 treatment.  */
      if (link_info->wrap_hash != NULL
	  || sym_hash->versioned != unversioned)
	{
	  struct elf_link_hash_entry **cur_sym_hashes;

	  /* Loop only over the symbols which have already been checked.  */
	  for (cur_sym_hashes = sym_hashes; cur_sym_hashes < &sym_hashes[i];
	       cur_sym_hashes++)
	    {
	      /* If the current symbol is identical to 'sym_hash', that means
		 the symbol was already adjusted (or at least checked).  */
	      if (*cur_sym_hashes == sym_hash)
		break;
	    }
	  /* Don't adjust the symbol again.  */
	  if (cur_sym_hashes < &sym_hashes[i])
	    continue;
	}

      if ((sym_hash->root.type == bfd_link_hash_defined
	   || sym_hash->root.type == bfd_link_hash_defweak)
	  && sym_hash->root.u.def.section == sec)
	{
	  /* As above, adjust the value if needed.  */
	  if (sym_hash->root.u.def.value > addr
	      && sym_hash->root.u.def.value <= toaddr)
	    sym_hash->root.u.def.value -= count;

	  /* As above, adjust the size if needed.  */
	  else if (sym_hash->root.u.def.value <= addr
		   && sym_hash->root.u.def.value + sym_hash->size > addr
		   && sym_hash->root.u.def.value + sym_hash->size <= toaddr)
	    sym_hash->size -= count;
	}
    }

  return true;
}

typedef bool (*relax_func_t) (bfd *, asection *, asection *,
			      struct bfd_link_info *,
			      Elf_Internal_Rela *,
			      bfd_vma, bfd_vma, bfd_vma, bool *,
			      riscv_pcgp_relocs *,
			      bool undefined_weak);


static htab_t
riscv_get_table_jump_htab (struct bfd_link_info *info, unsigned int link_reg)
{
  struct riscv_elf_link_hash_table *htab = riscv_elf_hash_table (info);
  riscv_table_jump_htab_t *tbj_htab = htab->table_jump_htab;

  BFD_ASSERT (tbj_htab != NULL);

  if (link_reg == 0)
    return tbj_htab->tbljt_htab;
  if (link_reg == X_RA)
    return tbj_htab->tbljalt_htab;

  return NULL;
}

static const char*
riscv_get_symbol_name (bfd *abfd, Elf_Internal_Rela *rel)
{
  unsigned long r_symndx = ELFNN_R_SYM (rel->r_info);
  Elf_Internal_Shdr *symtab_hdr = &elf_symtab_hdr (abfd);
  const char *name;

  if (!symtab_hdr->contents)
    return NULL;

  if (ELFNN_R_SYM (rel->r_info) < symtab_hdr->sh_info)
    {
      /* A local symbol.  */
      Elf_Internal_Sym *sym = ((Elf_Internal_Sym *) symtab_hdr->contents
                                 + r_symndx);
      name = bfd_elf_sym_name (abfd, symtab_hdr, sym, NULL);
    }
  else
    {
      struct elf_link_hash_entry *h;
      unsigned indx = r_symndx - symtab_hdr->sh_info;
      h = elf_sym_hashes (abfd)[indx];
      while (h->root.type == bfd_link_hash_indirect
	  || h->root.type == bfd_link_hash_warning)
        h = (struct elf_link_hash_entry *) h->root.u.i.link;
      if (h != NULL && h->type != STT_GNU_IFUNC)
        name = h->root.root.string;
      else
        /* We do not handle STT_GNU_IFUNC currently. */
        return NULL;
    }

  return name;
}

static bool
_bfd_riscv_table_jump_mark (bfd *abfd ATTRIBUTE_UNUSED, asection *sec,
		       asection *sym_sec ATTRIBUTE_UNUSED,
		       struct bfd_link_info *link_info,
		       Elf_Internal_Rela *rel,
		       bfd_vma symval,
		       bfd_vma max_alignment ATTRIBUTE_UNUSED,
		       bfd_vma reserve_size ATTRIBUTE_UNUSED,
		       bool *again ATTRIBUTE_UNUSED,
		       riscv_pcgp_relocs *pcgp_relocs ATTRIBUTE_UNUSED,
		       bool undefined_weak ATTRIBUTE_UNUSED)
{
  bfd_byte *contents = elf_section_data (sec)->this_hdr.contents;
  bfd_vma location = ELFNN_R_TYPE (rel->r_info) == R_RISCV_JAL ?
				    (bfd_vma) (contents + rel->r_offset)
				  : (bfd_vma) (contents + rel->r_offset + 4);
  bfd_vma target = bfd_getl32 ((void*) location);
  int rd = (target >> OP_SH_RD) & OP_MASK_RD;
  //const char *name = riscv_get_symbol_name (abfd, rel);
  htab_t tbljal_htab = riscv_get_table_jump_htab (link_info, rd);

  int type = ELFNN_R_TYPE (rel->r_info);
  BFD_ASSERT (type == R_RISCV_JAL || type == R_RISCV_CALL || type == R_RISCV_CALL_PLT);

  /* Check if it uses a valid link register. */
  if (tbljal_htab == NULL)
    return true;

  riscv_table_jump_htab_entry search = {symval, 0, NULL, 0};
  riscv_table_jump_htab_entry *entry = htab_find (tbljal_htab, &search);

  /* entry->index == 0 when the entry is not used as a table jump entry. */
  if (entry != NULL && entry->index > 0)
    {
      target = MATCH_TABLE_JUMP | ENCODE_ZCMP_TABLE_JUMP_INDEX (entry->index-1);
      bfd_putl32 (target, contents + rel->r_offset);
    }
  return true;
}

/* Relax AUIPC + JALR into JAL.  */

static bool
_bfd_riscv_relax_call (bfd *abfd, asection *sec, asection *sym_sec,
		       struct bfd_link_info *link_info,
		       Elf_Internal_Rela *rel,
		       bfd_vma symval,
		       bfd_vma max_alignment,
		       bfd_vma reserve_size ATTRIBUTE_UNUSED,
		       bool *again,
		       riscv_pcgp_relocs *pcgp_relocs,
		       bool undefined_weak ATTRIBUTE_UNUSED)
{
  bfd_byte *contents = elf_section_data (sec)->this_hdr.contents;
  bfd_vma foff = symval - (sec_addr (sec) + rel->r_offset);
  bool near_zero = (symval + RISCV_IMM_REACH / 2) < RISCV_IMM_REACH;
  int rd, r_type, len = 4, rvc = elf_elfheader (abfd)->e_flags & EF_RISCV_RVC;
  bool is_table_jump = (link_info->relax_pass == PASS_ZCE_TABLE_JUMP_COLLECT
			|| link_info->relax_pass == PASS_ZCE_TABLE_JUMP_APPLY);

  bfd_vma auipc = bfd_getl32 (contents + rel->r_offset);
  bfd_vma jalr = bfd_getl32 (contents + rel->r_offset + 4);
  rd = (jalr >> OP_SH_RD) & OP_MASK_RD;
  rvc = rvc && VALID_CJTYPE_IMM (foff);

  if (!is_table_jump)
    {
  /* If the call crosses section boundaries, fixed sections or alignment
     directive could casue the PC-relative offset to later increase.  */
  /* table jump has no such limitation.  */
  struct riscv_elf_link_hash_table *htab = riscv_elf_hash_table (link_info);
  andes_ld_options_t *andes = &htab->andes;
  if ((!andes->set_relax_cross_section_call
       && sym_sec->output_section != sec->output_section))
    return true;

  /* If the call crosses section boundaries, an alignment directive could
     cause the PC-relative offset to later increase, so we need to add in the
     max alignment of any section inclusive from the call to the target.
     Otherwise, we only need to use the alignment of the current section.  */
  if (VALID_JTYPE_IMM (foff))
    {
      if (sym_sec->output_section == sec->output_section
	  && sym_sec->output_section != bfd_abs_section_ptr)
	max_alignment = (bfd_vma) 1 << sym_sec->output_section->alignment_power;
      foff += ((bfd_signed_vma) foff < 0 ? -max_alignment : max_alignment);
    }

  /* See if this function call can be shortened.  */
  if (!VALID_JTYPE_IMM (foff) && !(!bfd_link_pic (link_info) && near_zero))
    return true;

  /* Shorten the function call.  */
  BFD_ASSERT (rel->r_offset + 8 <= sec->size);
    }
  else
    {
  /* Table jump profiling stage. It will be moved out of the relax_call function. */
  if (link_info->relax_pass == PASS_ZCE_TABLE_JUMP_COLLECT)
    {
      /* Early stop to prevent _bfd_riscv_relax_call to delete bytes in pass 0.  */
      if (link_info->relax_trip != 0)
	return true;

      htab_t tbljal_htab = riscv_get_table_jump_htab (link_info, rd);
      const char *name = riscv_get_symbol_name (abfd, rel);
      unsigned int benefit = len - 2;

      if (tbljal_htab == NULL
	  || name == NULL
	  || benefit == 0)
	return true;

      return riscv_update_table_jump_entry (tbljal_htab, symval, benefit, name);
    }

  /* Relax a table jump instruction that is marked. */
  if (link_info->relax_pass == PASS_ZCE_TABLE_JUMP_APPLY)
    {
      if (((auipc ^ MATCH_TABLE_JUMP) & MASK_CM_JALT) == 0)
	{
	  rel->r_info = ELFNN_R_INFO (ELFNN_R_SYM (rel->r_info), R_RISCV_TABLE_JUMP);
	  *again = true;
	  return riscv_relax_delete_bytes (abfd, sec, rel->r_offset + 2, 6, link_info, pcgp_relocs);
	}
      return true;
    }
    }

  /* C.J exists on RV32 and RV64, but C.JAL is RV32-only.  */
  rvc = rvc && (rd == 0 || (rd == X_RA && ARCH_SIZE == 32));

  if (rvc)
    {
      /* Relax to C.J[AL] rd, addr.  */
      r_type = R_RISCV_RVC_JUMP;
      auipc = rd == 0 ? MATCH_C_J : MATCH_C_JAL;
      len = 2;
    }
  else if (VALID_JTYPE_IMM (foff))
    {
      /* Relax to JAL rd, addr.  */
      r_type = R_RISCV_JAL;
      auipc = MATCH_JAL | (rd << OP_SH_RD);
    }
  else
    {
      /* Near zero, relax to JALR rd, x0, addr.  */
      BFD_ASSERT (VALID_ITYPE_IMM (symval));
      r_type = R_RISCV_LO12_I;
      auipc = MATCH_JALR | (rd << OP_SH_RD);
    }

  /* Replace the R_RISCV_CALL reloc.  */
  rel->r_info = ELFNN_R_INFO (ELFNN_R_SYM (rel->r_info), r_type);
  /* Replace the AUIPC.  */
  riscv_put_insn (8 * len, auipc, contents + rel->r_offset);

  /* Delete unnecessary JALR.  */
  *again = true;
  return riscv_relax_delete_bytes (abfd, sec, rel->r_offset + len, 8 - len,
				   link_info, pcgp_relocs);
}

/* Traverse all output sections and return the max alignment.  */

static bfd_vma
_bfd_riscv_get_max_alignment (asection *sec)
{
  unsigned int max_alignment_power = 0;
  asection *o;

  for (o = sec->output_section->owner->sections; o != NULL; o = o->next)
    {
      if (o->alignment_power > max_alignment_power)
	max_alignment_power = o->alignment_power;
    }

  return (bfd_vma) 1 << max_alignment_power;
}

/* Relax non-PIC global variable references to GP-relative references.  */

static bool
_bfd_riscv_relax_lui (bfd *abfd,
		      asection *sec,
		      asection *sym_sec,
		      struct bfd_link_info *link_info,
		      Elf_Internal_Rela *rel,
		      bfd_vma symval,
		      bfd_vma max_alignment,
		      bfd_vma reserve_size,
		      bool *again,
		      riscv_pcgp_relocs *pcgp_relocs,
		      bool undefined_weak)
{
  bfd_byte *contents = elf_section_data (sec)->this_hdr.contents;
  bfd_vma gp = riscv_global_pointer_value (link_info);
  int use_rvc = elf_elfheader (abfd)->e_flags & EF_RISCV_RVC;

  BFD_ASSERT (rel->r_offset + 4 <= sec->size);

  if (gp)
    {
      /* If gp and the symbol are in the same output section, which is not the
	 abs section, then consider only that output section's alignment.  */
      struct bfd_link_hash_entry *h =
	bfd_link_hash_lookup (link_info->hash, RISCV_GP_SYMBOL, false, false,
			      true);
      if (h->u.def.section->output_section == sym_sec->output_section
	  && sym_sec->output_section != bfd_abs_section_ptr)
	max_alignment = (bfd_vma) 1 << sym_sec->output_section->alignment_power;
    }

  /* Is the reference in range of x0 or gp?
     Valid gp range conservatively because of alignment issue.  */
  if (undefined_weak
      || (VALID_ITYPE_IMM (symval)
	  || (symval >= gp
	      && VALID_ITYPE_IMM (symval - gp + max_alignment + reserve_size))
	  || (symval < gp
	      && VALID_ITYPE_IMM (symval - gp - max_alignment - reserve_size))))
    {
      unsigned sym = ELFNN_R_SYM (rel->r_info);
      switch (ELFNN_R_TYPE (rel->r_info))
	{
	case R_RISCV_LO12_I:
	  if (undefined_weak)
	    {
	      /* Change the RS1 to zero.  */
	      bfd_vma insn = bfd_getl32 (contents + rel->r_offset);
	      insn &= ~(OP_MASK_RS1 << OP_SH_RS1);
	      bfd_putl32 (insn, contents + rel->r_offset);
	    }
	  else
	    rel->r_info = ELFNN_R_INFO (sym, R_RISCV_GPREL_I);
	  return true;

	case R_RISCV_LO12_S:
	  if (undefined_weak)
	    {
	      /* Change the RS1 to zero.  */
	      bfd_vma insn = bfd_getl32 (contents + rel->r_offset);
	      insn &= ~(OP_MASK_RS1 << OP_SH_RS1);
	      bfd_putl32 (insn, contents + rel->r_offset);
	    }
	  else
	    rel->r_info = ELFNN_R_INFO (sym, R_RISCV_GPREL_S);
	  return true;

	case R_RISCV_HI20:
	  /* We can delete the unnecessary LUI and reloc.  */
	  rel->r_info = ELFNN_R_INFO (0, R_RISCV_NONE);
	  *again = true;
	  return riscv_relax_delete_bytes (abfd, sec, rel->r_offset, 4,
					   link_info, pcgp_relocs);

	default:
	  abort ();
	}
    }

  /* Can we relax LUI to C.LUI?  Alignment might move the section forward;
     account for this assuming page alignment at worst. In the presence of 
     RELRO segment the linker aligns it by one page size, therefore sections
     after the segment can be moved more than one page. */

  if (use_rvc
      && ELFNN_R_TYPE (rel->r_info) == R_RISCV_HI20
      && VALID_CITYPE_LUI_IMM (RISCV_CONST_HIGH_PART (symval))
      && VALID_CITYPE_LUI_IMM (RISCV_CONST_HIGH_PART (symval)
			    + (link_info->relro ? 2 * ELF_MAXPAGESIZE
			       : ELF_MAXPAGESIZE)))
    {
      /* Replace LUI with C.LUI if legal (i.e., rd != x0 and rd != x2/sp).  */
      bfd_vma lui = bfd_getl32 (contents + rel->r_offset);
      unsigned rd = ((unsigned)lui >> OP_SH_RD) & OP_MASK_RD;
      if (rd == 0 || rd == X_SP)
	return true;

      lui = (lui & (OP_MASK_RD << OP_SH_RD)) | MATCH_C_LUI;
      bfd_putl32 (lui, contents + rel->r_offset);

      /* Replace the R_RISCV_HI20 reloc.  */
      rel->r_info = ELFNN_R_INFO (ELFNN_R_SYM (rel->r_info), R_RISCV_RVC_LUI);

      *again = true;
      return riscv_relax_delete_bytes (abfd, sec, rel->r_offset + 2, 2,
				       link_info, pcgp_relocs);
    }

  return true;
}

/* Relax non-PIC TLS references to TP-relative references.  */

static bool
_bfd_riscv_relax_tls_le (bfd *abfd,
			 asection *sec,
			 asection *sym_sec ATTRIBUTE_UNUSED,
			 struct bfd_link_info *link_info,
			 Elf_Internal_Rela *rel,
			 bfd_vma symval,
			 bfd_vma max_alignment ATTRIBUTE_UNUSED,
			 bfd_vma reserve_size ATTRIBUTE_UNUSED,
			 bool *again,
			 riscv_pcgp_relocs *pcgp_relocs,
			 bool undefined_weak ATTRIBUTE_UNUSED)
{
  /* See if this symbol is in range of tp.  */
  if (RISCV_CONST_HIGH_PART (tpoff (link_info, symval)) != 0)
    return true;

  BFD_ASSERT (rel->r_offset + 4 <= sec->size);
  switch (ELFNN_R_TYPE (rel->r_info))
    {
    case R_RISCV_TPREL_LO12_I:
      rel->r_info = ELFNN_R_INFO (ELFNN_R_SYM (rel->r_info), R_RISCV_TPREL_I);
      return true;

    case R_RISCV_TPREL_LO12_S:
      rel->r_info = ELFNN_R_INFO (ELFNN_R_SYM (rel->r_info), R_RISCV_TPREL_S);
      return true;

    case R_RISCV_TPREL_HI20:
    case R_RISCV_TPREL_ADD:
      /* We can delete the unnecessary instruction and reloc.  */
      rel->r_info = ELFNN_R_INFO (0, R_RISCV_NONE);
      *again = true;
      return riscv_relax_delete_bytes (abfd, sec, rel->r_offset, 4, link_info,
				       pcgp_relocs);

    default:
      abort ();
    }
}

/* Implement R_RISCV_ALIGN by deleting excess alignment NOPs.
   Once we've handled an R_RISCV_ALIGN, we can't relax anything else.  */

static bool
_bfd_riscv_relax_align (bfd *abfd, asection *sec,
			asection *sym_sec,
			struct bfd_link_info *link_info,
			Elf_Internal_Rela *rel,
			bfd_vma symval,
			bfd_vma max_alignment ATTRIBUTE_UNUSED,
			bfd_vma reserve_size ATTRIBUTE_UNUSED,
			bool *again ATTRIBUTE_UNUSED,
			riscv_pcgp_relocs *pcgp_relocs ATTRIBUTE_UNUSED,
			bool undefined_weak ATTRIBUTE_UNUSED)
{
  bfd_byte *contents = elf_section_data (sec)->this_hdr.contents;
  bfd_vma alignment = 1, pos;
  bfd_vma filled = 0;
  while (alignment <= rel->r_addend)
    alignment *= 2;

  symval -= rel->r_addend;
  bfd_vma aligned_addr = ((symval - 1) & ~(alignment - 1)) + alignment;
  bfd_vma nop_bytes = aligned_addr - symval;

  /* Once we've handled an R_RISCV_ALIGN, we can't relax anything else.  */
  sec->sec_flg0 = true;

  /* Make sure there are enough NOPs to actually achieve the alignment.  */
  if (rel->r_addend < nop_bytes)
    {
      _bfd_error_handler
	(_("%pB(%pA+%#" PRIx64 "): %" PRId64 " bytes required for alignment "
	   "to %" PRId64 "-byte boundary, but only %" PRId64 " present"),
	 abfd, sym_sec, (uint64_t) rel->r_offset,
	 (int64_t) nop_bytes, (int64_t) alignment, (int64_t) rel->r_addend);
      bfd_set_error (bfd_error_bad_value);
      return false;
    }

  /* { Andes */
  /*       r_offset         aligned_offset   + r_addend
   *  -----+----------------+----------------+--------------------
   *       | filled? refill |     delete     |
   *  -----+----------------+----------------+--------------------
   *       <--  nop_bytes --^ alignment * n
   */
  if (nop_bytes & 3)
    {
      BFD_ASSERT ((nop_bytes & 3) == 2);
      bfd_vma offset = nsta.prev_aligned_offset;
      bfd_vma end = symval - sec_addr (sec);
      filled = andes_try_target_align (abfd, sec, sym_sec, link_info,
				       rel, offset, end);
      /* rel->offset might be changed!  */
      nop_bytes -= filled;
      rel->r_addend -= filled;
    }
  nsta.prev_aligned_offset = rel->r_offset + nop_bytes;
  /* } Andes */

  /* Delete the reloc.  */
  rel->r_info = ELFNN_R_INFO (0, R_RISCV_NONE);

  /* Write as many RISC-V NOPs as we need.  */
  for (pos = 0; pos < (nop_bytes & -4); pos += 4)
    bfd_putl32 (RISCV_NOP, contents + rel->r_offset + pos);

  /* Write a final RVC NOP if need be.  */
  if (nop_bytes % 4 != 0)
    bfd_putl16 (RVC_NOP, contents + rel->r_offset + pos);

  /* If the number of NOPs is already correct, there's nothing to do.  */
  if (nop_bytes == rel->r_addend)
    return true;

  /* Delete the excess bytes.  */
  return riscv_relax_delete_bytes (abfd, sec, rel->r_offset + nop_bytes,
				   rel->r_addend - nop_bytes, link_info,
				   NULL);
}

/* Relax PC-relative references to GP-relative references.  */

#define MAX(a, b) (a) > (b) ? (a) : (b)

static bool
_bfd_riscv_relax_pc (bfd *abfd ATTRIBUTE_UNUSED,
		     asection *sec,
		     asection *sym_sec,
		     struct bfd_link_info *link_info,
		     Elf_Internal_Rela *rel,
		     bfd_vma symval,
		     bfd_vma max_alignment,
		     bfd_vma reserve_size,
		     bool *again ATTRIBUTE_UNUSED,
		     riscv_pcgp_relocs *pcgp_relocs,
		     bool undefined_weak)
{
  bfd_byte *contents = elf_section_data (sec)->this_hdr.contents;
  bfd_vma gp = riscv_global_pointer_value (link_info);

  BFD_ASSERT (rel->r_offset + 4 <= sec->size);

  /* Chain the _LO relocs to their cooresponding _HI reloc to compute the
     actual target address.  */
  riscv_pcgp_hi_reloc *hi = NULL;
  riscv_pcgp_hi_reloc hi_reloc;
  memset (&hi_reloc, 0, sizeof (hi_reloc));
  switch (ELFNN_R_TYPE (rel->r_info))
    {
    case R_RISCV_PCREL_LO12_I:
    case R_RISCV_PCREL_LO12_S:
      {
	/* If the %lo has an addend, it isn't for the label pointing at the
	   hi part instruction, but rather for the symbol pointed at by the
	   hi part instruction.  So we must subtract it here for the lookup.
	   It is still used below in the final symbol address.  */
	bfd_vma hi_sec_off = symval - sec_addr (sym_sec) - rel->r_addend;
	hi = riscv_find_pcgp_hi_reloc (pcgp_relocs, hi_sec_off);
	if (hi == NULL)
	  {
	    riscv_record_pcgp_lo_reloc (pcgp_relocs, hi_sec_off);
	    return true;
	  }

	hi_reloc = *hi;
	symval = hi_reloc.hi_addr;
	sym_sec = hi_reloc.sym_sec;

	/* We can not know whether the undefined weak symbol is referenced
	   according to the information of R_RISCV_PCREL_LO12_I/S.  Therefore,
	   we have to record the 'undefined_weak' flag when handling the
	   corresponding R_RISCV_HI20 reloc in riscv_record_pcgp_hi_reloc.  */
	undefined_weak = hi_reloc.undefined_weak;
      }
      break;

    case R_RISCV_PCREL_HI20:
      /* Mergeable symbols and code might later move out of range.  */
      if (! undefined_weak
	  && sym_sec->flags & (SEC_MERGE | SEC_CODE))
	return true;

      /* If the cooresponding lo relocation has already been seen then it's not
         safe to relax this relocation.  */
      if (riscv_find_pcgp_lo_reloc (pcgp_relocs, rel->r_offset))
	return true;

      break;

    default:
      abort ();
    }

  if (gp)
    {
      /* If gp and the symbol are in the same output section, which is not the
	 abs section, then consider only that output section's alignment.  */
      struct bfd_link_hash_entry *h =
	bfd_link_hash_lookup (link_info->hash, RISCV_GP_SYMBOL, false, false,
			      true);
      if (h->u.def.section->output_section == sym_sec->output_section
	  && sym_sec->output_section != bfd_abs_section_ptr)
	max_alignment = (bfd_vma) 1 << sym_sec->output_section->alignment_power;

      /* alignment might enlarge the gap between symbol and GP when relaxing.
         guard it with the delta of both alignments.  */
      {
	unsigned sym_align = 1u << sym_sec->alignment_power;
	unsigned gp_align = 1u << h->u.def.section->alignment_power;
	if (sym_align > gp_align)
	  max_alignment = MAX (max_alignment, sym_align - gp_align);
      }
    }

  /* Is the reference in range of x0 or gp?
     Valid gp range conservatively because of alignment issue.  */
  if (undefined_weak
      || (VALID_ITYPE_IMM (symval)
	  || (symval >= gp
	      && VALID_ITYPE_IMM (symval - gp + max_alignment + reserve_size))
	  || (symval < gp
	      && VALID_ITYPE_IMM (symval - gp - max_alignment - reserve_size))))
    {
      unsigned sym = hi_reloc.hi_sym;
      switch (ELFNN_R_TYPE (rel->r_info))
	{
	case R_RISCV_PCREL_LO12_I:
	  if (undefined_weak)
	    {
	      /* Change the RS1 to zero, and then modify the relocation
		 type to R_RISCV_LO12_I.  */
	      bfd_vma insn = bfd_getl32 (contents + rel->r_offset);
	      insn &= ~(OP_MASK_RS1 << OP_SH_RS1);
	      bfd_putl32 (insn, contents + rel->r_offset);
	      rel->r_info = ELFNN_R_INFO (sym, R_RISCV_LO12_I);
	      rel->r_addend = hi_reloc.hi_addend;
	    }
	  else
	    {
	      rel->r_info = ELFNN_R_INFO (sym, R_RISCV_GPREL_I);
	      rel->r_addend += hi_reloc.hi_addend;
	    }
	  return riscv_mark_pcgp_hi_reloc (pcgp_relocs, hi);

	case R_RISCV_PCREL_LO12_S:
	  if (undefined_weak)
	    {
	      /* Change the RS1 to zero, and then modify the relocation
		 type to R_RISCV_LO12_S.  */
	      bfd_vma insn = bfd_getl32 (contents + rel->r_offset);
	      insn &= ~(OP_MASK_RS1 << OP_SH_RS1);
	      bfd_putl32 (insn, contents + rel->r_offset);
	      rel->r_info = ELFNN_R_INFO (sym, R_RISCV_LO12_S);
	      rel->r_addend = hi_reloc.hi_addend;
	    }
	  else
	    {
	      rel->r_info = ELFNN_R_INFO (sym, R_RISCV_GPREL_S);
	      rel->r_addend += hi_reloc.hi_addend;
	    }
	  return riscv_mark_pcgp_hi_reloc (pcgp_relocs, hi);

	case R_RISCV_PCREL_HI20:
	  riscv_record_pcgp_hi_reloc (pcgp_relocs,
				      rel->r_offset,
				      rel->r_addend,
				      symval,
				      ELFNN_R_SYM(rel->r_info),
				      sym_sec,
				      undefined_weak);
	  /* We can delete the unnecessary AUIPC and reloc.  */
	  rel->r_info = ELFNN_R_INFO (0, R_RISCV_DELETE);
	  rel->r_addend = 4;
	  return true;

	default:
	  abort ();
	}
    }

  return true;
}

static bool
_bfd_riscv_record_jal (bfd *abfd,
			 asection *sec ATTRIBUTE_UNUSED,
			 asection *sym_sec ATTRIBUTE_UNUSED,
			 struct bfd_link_info *link_info,
			 Elf_Internal_Rela *rel,
			 bfd_vma symval,
			 bfd_vma max_alignment ATTRIBUTE_UNUSED,
			 bfd_vma reserve_size ATTRIBUTE_UNUSED,
			 bool *again ATTRIBUTE_UNUSED,
			 riscv_pcgp_relocs *pcgp_relocs ATTRIBUTE_UNUSED,
			 bool undefined_weak ATTRIBUTE_UNUSED)
{
  bfd_byte *contents = elf_section_data (sec)->this_hdr.contents;
  bfd_vma jal = bfd_getl32 (contents + rel->r_offset);
  unsigned int rd = (jal >> OP_SH_RD) & OP_MASK_RD;
  htab_t tbljal_htab = riscv_get_table_jump_htab (link_info, rd);
  const char *name = riscv_get_symbol_name (abfd, rel);
  //struct riscv_elf_link_hash_table *htab = riscv_elf_hash_table (link_info);
  //riscv_table_jump_htab_t *tbj_htab = htab->table_jump_htab;

  if (link_info->relax_pass == PASS_ZCE_TABLE_JUMP_APPLY)
    {
      if (((jal ^ MATCH_TABLE_JUMP) & MASK_CM_JALT) == 0)
	{
	  rel->r_info = ELFNN_R_INFO (ELFNN_R_SYM (rel->r_info), R_RISCV_TABLE_JUMP);
	  *again = true;
	  return riscv_relax_delete_bytes (abfd, sec,
		  rel->r_offset + 2, 2, link_info, pcgp_relocs);
	}
      return true;
    }

  BFD_ASSERT (link_info->relax_pass == PASS_ZCE_TABLE_JUMP_COLLECT);

  if (tbljal_htab == NULL
      || name == NULL
      || (link_info->relax_pass == PASS_ZCE_TABLE_JUMP_COLLECT && link_info->relax_trip > 0)
      || link_info->relax_pass > PASS_ZCE_TABLE_JUMP_COLLECT)
    return true;

  return riscv_update_table_jump_entry (tbljal_htab, symval, 2, name);
}

typedef struct
{
  riscv_table_jump_htab_t *htab;
  unsigned int start;
  unsigned int end;
} riscv_table_jump_args;

static int
riscv_ranking_table_jump (void **entry_ptr, void *_arg)
{
  const riscv_table_jump_htab_entry *entry;
  riscv_table_jump_args *arg;
  riscv_table_jump_htab_t *htab;
  unsigned int *savings;
  const char **names;
  bfd_vma *tbj_indexes;

  entry = (const riscv_table_jump_htab_entry *) *entry_ptr;
  arg = (riscv_table_jump_args*) _arg;
  htab = (riscv_table_jump_htab_t *) arg->htab;

  savings = htab->savings;
  names = htab->names;
  tbj_indexes = htab->tbj_indexes;

  /* search insert position and rank. */
  unsigned int left = arg->start;
  unsigned int right = arg->end + 1;

  while (left < right)
    {
      unsigned int mid = (left + right) / 2;
      if (savings[mid] == entry->benefit)
	{
	  left = mid;
	  break;
	}
      else if (savings[mid] == 0
	  || savings[mid] < entry->benefit)
        right = mid;
      else
        left = mid + 1;
    }

  for (unsigned int idx = arg->end; idx > left; idx--)
    {
      tbj_indexes[idx] = tbj_indexes[idx-1];
      savings[idx] = savings[idx-1];
      names[idx] = names[idx-1];
    }

  if (left <= arg->end)
    {
      tbj_indexes[left] = entry->address;
      savings[left] = entry->benefit;
      names[left] = entry->name;
    }

  return true;
}

static bool
riscv_record_table_jump_index (htab_t htab, riscv_table_jump_args *args)
{
  unsigned int idx;
  riscv_table_jump_htab_t *tbj_htab = args->htab;
  riscv_table_jump_htab_entry search;
  riscv_table_jump_htab_entry *entry = NULL;

  for (idx = args->start; idx <= args->end && tbj_htab->tbj_indexes[idx]; idx++)
    {
      search = (riscv_table_jump_htab_entry)
	  {tbj_htab->tbj_indexes[idx], 0, NULL, 0};
      entry = htab_find (htab, &search);

      BFD_ASSERT (entry != NULL);
      entry->index = idx + 1;
      tbj_htab->total_saving += tbj_htab->savings[idx];
    }

  /* True if there is at least one entry in table jump section.  */
  if (entry && entry->index)
    tbj_htab->end_idx = entry->index;

  return true;
}

static bool
riscv_table_jump_profiling (riscv_table_jump_htab_t *table_jump_htab,
    riscv_table_jump_args *args)
{
  args->start = 0, args->end = 31; /* zc v1.0.0 rc5.7  */
  /* Do a ranking. */
  htab_traverse (table_jump_htab->tbljt_htab,
	riscv_ranking_table_jump,
	args);
  riscv_record_table_jump_index (
	  table_jump_htab->tbljt_htab,
	  args);

  args->start = 32, args->end = 255;
  htab_traverse (table_jump_htab->tbljalt_htab,
	riscv_ranking_table_jump,
	args);
  riscv_record_table_jump_index (
	  table_jump_htab->tbljalt_htab,
	  args);
  return true;
}

/* Delete the bytes for R_RISCV_DELETE.  */

static bool
_bfd_riscv_relax_delete (bfd *abfd,
			 asection *sec,
			 asection *sym_sec ATTRIBUTE_UNUSED,
			 struct bfd_link_info *link_info,
			 Elf_Internal_Rela *rel,
			 bfd_vma symval ATTRIBUTE_UNUSED,
			 bfd_vma max_alignment ATTRIBUTE_UNUSED,
			 bfd_vma reserve_size ATTRIBUTE_UNUSED,
			 bool *again ATTRIBUTE_UNUSED,
			 riscv_pcgp_relocs *pcgp_relocs ATTRIBUTE_UNUSED,
			 bool undefined_weak ATTRIBUTE_UNUSED)
{
  if (!riscv_relax_delete_bytes (abfd, sec, rel->r_offset, rel->r_addend,
				 link_info, NULL))
    return false;
  rel->r_info = ELFNN_R_INFO (0, R_RISCV_NONE);
  return true;
}

/* Called by after_allocation to set the information of data segment
   before relaxing.  */

void
bfd_elfNN_riscv_set_data_segment_info (struct bfd_link_info *info,
                                       int *data_segment_phase)
{
  struct riscv_elf_link_hash_table *htab = riscv_elf_hash_table (info);
  htab->data_segment_phase = data_segment_phase;
}

/* Relax a section.

   Pass 0: Shortens code sequences for LUI/CALL/TPREL/PCREL relocs.
   Pass 1: Deletes the bytes that PCREL relaxation in pass 0 made obsolete.
   Pass 2: Which cannot be disabled, handles code alignment directives.  */

/* Extended relax passes

   Lazy initializations: option handling (relax/exec.it), internal stuff.
   * denotes mandatory.

  *Pass ini: init stuff
   Pass gp0: GP instruction relaxation: pcrel
   Pass gp1:                          : low part
   Pass gp2:                          : high part
   Pass   0: Shortens code sequences for LUI/CALL/TPREL/PCREL relocs.
   Pass tj0: Table jump collect (after jal => c.jal)
   Pass tj1:            apply
  *Pass   1: Deletes the bytes that PCREL relaxation in pass 0 made obsolete.
   Pass ex1: Exec.it #1 collection
   Pass ex2:         #2 replacement
  *Pass   2: Which cannot be disabled, handles code alignment directives.
  *Pass res: Reslove special relocations (exec.it, gp-relaxation)
   Pass red: Reduce .exec.itable section iff secure
*/

static bool
_bfd_riscv_relax_section (bfd *abfd, asection *sec,
			  struct bfd_link_info *info,
			  bool *again)
{
  Elf_Internal_Shdr *symtab_hdr = &elf_symtab_hdr (abfd);
  struct riscv_elf_link_hash_table *htab = riscv_elf_hash_table (info);
  struct bfd_elf_section_data *data = elf_section_data (sec);
  Elf_Internal_Rela *relocs;
  bool ret = false;
  unsigned int i;
  bfd_vma max_alignment, reserve_size = 0, used_bytes, trimmed_bytes;
  riscv_pcgp_relocs pcgp_relocs;
  riscv_table_jump_htab_t *table_jump_htab = htab->table_jump_htab;
  struct elf_link_hash_entry *jvt_sym;

  /* { Andes  */
  andes_ld_options_t *andes = &htab->andes;
  /* initialize stuff here  */
  static int is_init = 0;
  if (is_init == 0)
    { /* init states  */
      if (nsta.opt == NULL)
	{
	  memset (&nsta, 0, sizeof (nsta));
	  nsta.opt = andes;
	}
      memset (&ict, 0, sizeof (ict));
      /* init execit state here  */
      memset (&execit, 0, sizeof (execit));
      execit.htab = riscv_elf_hash_table (info);
      /* exec.it or nexec.it  */
      if (nsta.opt->execit_flags.nexecit_op != 0 ||
	  riscv_has_subset (info, "xnexecit"))
	execit.execit_op = NEXECIT_INSN;
      else
	execit.execit_op = EXECIT_INSN;
      is_init = 1;
      /* init page size if not yet  */
      if (andes->set_relax_page_size == 0)
	andes->set_relax_page_size = ELF_MAXPAGESIZE;
      /* init '__global_pointer$' if not given.  */
      if (!bfd_link_pic (info))
	{
	  if (!riscv_init_global_pointer (sec->output_section->owner, info))
	    {
	      (*_bfd_error_handler)
		(_("\nWarning: Init __global_pointer$ failed. "
		   "Can not find __global_pointer$ and .sdata section.\n"));
	    }
	}
      /* For the gp relative insns, gp must be 4/8 bytes aligned (b14634).  */
      if (andes->gp_relative_insn)
	{
	  int align = (ARCH_SIZE == 64) ? 8 : 4;
	  bfd_vma gp = riscv_global_pointer_value (info);
	  if (gp % align)
	    (*_bfd_error_handler) (_("error: Please set gp to %x-byte aligned "
				"or turn off the gp relative instructions "
				"(--mno-gp-insn).\n"), align);
	}
      /* allocate content of jump table section. (why?)
       * riscv_relax_delete_bytes get contents from "sec->used_by_bfd->this_hdr.contents"
       * but allocated another copy in "sec->contents".
       */
      if (andes->set_table_jump)
	{
	  bfd_byte *contents = NULL;
	  Elf_Internal_Sym *isym = NULL;
	  bfd *owner = htab->table_jump_htab->tablejump_sec_owner;
	  asection *tjsec = htab->table_jump_htab->tablejump_sec;
	  riscv_get_section_contents (owner, tjsec, &contents, true);
	  riscv_get_local_syms (owner, tjsec, &isym);
	}
    }

  /* Reset aligned offset each input section.  */
  nsta.prev_aligned_offset = 0;
  /* } Andes  */

  *again = false;

  /* if relax disabled  */
  if (bfd_link_relocatable (info)
      || (sec->sec_flg0 && info->relax_pass <= PASS_ALIGN_ORG)
      || (sec->flags & SEC_RELOC) == 0
      || sec->reloc_count == 0
      || (info->disable_target_specific_optimizations
	  && info->relax_pass != PASS_ANDES_INIT
	  && info->relax_pass != PASS_DELETE_ORG
	  && info->relax_pass <= PASS_DELETE_ORG)
      /* The exp_seg_relro_adjust is enum phase_enum (0x4),
	 and defined in ld/ldexp.h.  */
      || *(htab->data_segment_phase) == 4)
    return true;

  /* TODO: if zcmt is not enabled.  */
  if (!andes->set_table_jump
      && info->relax_pass >= PASS_ZCE_TABLE_JUMP_COLLECT
      && info->relax_pass <= PASS_ZCE_TABLE_JUMP_APPLY)
    return true;

  /* { Andes  */
  /* if gp-insn-relax disabled  */
  if (andes->gp_relative_insn == 0
      && info->relax_pass >= PASS_ANDES_GP_PCREL
      && info->relax_pass <= PASS_ANDES_GP_2)
    return true;

  /* if execit disabled  */
  if ((andes->target_optimization & RISCV_RELAX_EXECIT_ON) == 0
      && info->relax_pass >= PASS_EXECIT_1
      && info->relax_pass <= PASS_EXECIT_2)
    return true;

  /* exec.it initializatoin if enabled.  */
  if (!execit.is_init && info->relax_pass == PASS_EXECIT_1)
    {
      bfd *output_bfd = info->output_bfd;
      if ((andes->target_optimization & RISCV_RELAX_EXECIT_ON) &&
	  (output_bfd) &&
	  (elf_elfheader (output_bfd)->e_flags & EF_RISCV_RVC))
	{
	  andes_execit_init (info);
	  /* For EXECIT update, we replace execit candiadtes to exec.it
	     according to the imported table first. After that,
	     we build the EXECIT hash table for the remaining patterns
	     to do EXECIT replacement again.  */
	  if (andes->execit_import_file)
	    {
	      execit.is_built = 1;
	      riscv_elf_execit_import_table (abfd, info);
	    }
	}
      else
	{
	  execit.is_built = 1;
	  execit.is_replaced = 1;
	}

      execit.is_init = 1;
    }
  /* } Andes  */

  riscv_init_pcgp_relocs (&pcgp_relocs);

  /* Read this BFD's relocs if we haven't done so already.  */
  if (data->relocs)
    relocs = data->relocs;
  else if (!(relocs = _bfd_elf_link_read_relocs (abfd, sec, NULL, NULL,
						 info->keep_memory)))
    goto fail;

  /* { Andes  */
  /* Sort relocation by r_offset.  */
  riscv_insertion_sort (relocs, sec->reloc_count,
			sizeof (Elf_Internal_Rela), compar_reloc);

  /*  exec.it processes:
   *    * pass 1 (now)
   *      ** hash insn
   *      ** rank hash (last)
   *      ** mark rank (last)
   *    * pass 2
   *      ** replace insn
   */
  switch (info->relax_pass)
    {
    case PASS_ANDES_INIT:
      if (execit.is_built && execit.is_replaced)
	return true;
      execit.final_sec = sec;
      return true;
    /* Here is the entrance of EXECIT relaxation. There are two pass of
	EXECIT relaxation. The one is to traverse all instructions and build
	the hash table. The other one is to compare instructions and replace
	it by exec.it.  */
    case PASS_EXECIT_1:
      if (andes->execit_import_file && andes->keep_import_execit
	  && !andes->update_execit_table && sec == execit.final_sec)
	{ /* special case that itable needs to be built explicitly.  */
	  andes_execit_traverse_insn_hash (andes_execit_rank_imported_insn);
	  andes_execit_build_itable (info);
	}
      if (execit.is_built)
	return true;
      if (!andes_execit_hash_insn (abfd, sec, info))
	return false;
      if (sec == execit.final_sec)
	{ /* rank instruction patterns.  */
	  andes_execit_traverse_insn_hash (andes_execit_rank_insn);
	  if (andes->execit_import_file)
	    andes_execit_traverse_insn_hash (andes_execit_rank_imported_insn);
	  andes_execit_build_itable (info);
	  execit.is_built = 1;
	}
      return true;
    case PASS_EXECIT_2:
      if (execit.is_replaced)
	return true;
      if (!andes_execit_replace_insn (info, abfd, sec))
	return false;
      if (sec == execit.final_sec)
	{
	  execit.is_replaced = 1;
	  andes_execit_delete_blank (info);
	  if (andes->update_execit_table && !execit.is_replace_again)
	    {
	      execit.is_replace_again = 1;
	      execit.is_built = 0;
	      execit.is_replaced = 0;
	      info->relax_pass = PASS_EXECIT_1;
	      *again = true;
	    }
	}
      return true;
    case PASS_REDUCE: /* after PASS_RESLOVE */
      if (!execit.is_itable_finalized)
	{ /* finalize itable size  */
	  execit.is_itable_finalized = 1;
	  if ((andes->execit_import_file == NULL) ||
	      andes->keep_import_execit ||
	      andes->update_execit_table)
	    {
	      asection *table_sec;
	      table_sec = andes_execit_get_itable_section (info->input_bfds);
	      BFD_ASSERT (table_sec != NULL);
	      #ifdef ITABLE_IS_SAFE_TO_REDUCE
	      table_sec->size = execit.next_itable_index << 2;
	      #endif
	    }
	}
      return true;
    case PASS_ANDES_GP_PCREL ... PASS_ANDES_GP_2:
    //case PASS_ZCE_TABLE_JUMP_COLLECT ... PASS_ZCE_TABLE_JUMP_APPLY:
    case PASS_SHORTEN_ORG ... PASS_DELETE_ORG:
    case PASS_ALIGN_ORG ... PASS_RESLOVE:
      break;
    default:
      (*_bfd_error_handler) (_("error: Unknow relax pass."));
      break;
    }
  /* } Andes  */

  if (htab)
    {
      max_alignment = htab->max_alignment;
      if (max_alignment == (bfd_vma) -1)
	{
	  max_alignment = _bfd_riscv_get_max_alignment (sec);
	  htab->max_alignment = max_alignment;
	}
    }
  else
    max_alignment = _bfd_riscv_get_max_alignment (sec);

  /* relax_trip 0:
      Record symbol address and expected size saving of each relocation
      that can be replaced by table jump instructions.

     relax_trip 1:
      Rank the best 64 relocations to replace for cm.jt and the best 192
      relocations for cm.jalt in terms of the total size saved.

     relax_trip 2:
      Check if table jump can reduce the size, and delete the whole table
      jump section if the size will not be reduced.

      If table jump can save size, and then we replace all targeted
      instructions/instruction pairs(e.g. auipc+jalr) to table jump
      instructions with the index encoded.

     relax_trip 3: Trim unused slots in the table jump section.  */

  if (info->relax_pass == PASS_ZCE_TABLE_JUMP_COLLECT
      && riscv_use_table_jump (info))
    {
      /* Avoid size savings of relocations to be recoreded multiple times.  */
      if (info->relax_trip == 0 && *(htab->data_segment_phase) != 0)
	return true;
      /* Rank the entries, and calculate the expected total saving.  */
      else if (info->relax_trip == 1)
	{
	  *again = true;
	  /* Profiling stage finished.  */
	  if (table_jump_htab->end_idx != 0)
	    return true;

	  riscv_table_jump_args args = {table_jump_htab, 0, 0};
	  /* Estimate size savings if table jump is used.  */
	  riscv_table_jump_profiling (table_jump_htab, &args);
	  return true;
	}
      /* Skip generating table jump instructions if they do not help reduce code size.   */
      else if (info->relax_trip == 2)
	{
	  bfd *owner = table_jump_htab->tablejump_sec_owner;
	  asection *tjsec = table_jump_htab->tablejump_sec;
	  /* Check if table jump can save size. Skip generating table
	    jump instruction if not.  */
	  if ((signed) table_jump_htab->total_saving <=
			  table_jump_htab->end_idx * RISCV_ELF_WORD_BYTES
	      && tjsec->size > 0)
	    {
	      jvt_sym = elf_link_hash_lookup (elf_hash_table (info),
			RISCV_TABLE_JUMP_BASE_SYMBOL,
			false, false, true);
	      jvt_sym->root.u.def.section = bfd_abs_section_ptr;
	      tjsec->alignment_power = 0; /* b27458 */
	      tjsec->output_section->alignment_power = 0; /* b27458 */
	      return riscv_relax_delete_bytes (owner, tjsec, 0, tjsec->size, info, NULL);
	    }
	  else if (tjsec->size == 0)
	    return true;
	  else if (tjsec->size > 0)
	    *again = true;
	}
      /* Trim the unused slot at the table jump section.
          TODO: skip generating entries if its saving is less than RISCV_ELF_WORD_BYTES.
	  We should skip those insns at the relax trip 2 without deleting bytes.  */
      else if (info->relax_trip == 3)
	{
	  /* Table jump entry section is trimmed.  */
	  if (table_jump_htab->end_idx < 0)
	    return true;

	  used_bytes = table_jump_htab->end_idx * RISCV_ELF_WORD_BYTES;
	  trimmed_bytes = (256 - table_jump_htab->end_idx) * RISCV_ELF_WORD_BYTES;
	  /* Trim unused slots.  */
	  if ((trimmed_bytes > 0) &&
	      !riscv_relax_delete_bytes (table_jump_htab->tablejump_sec_owner,
				table_jump_htab->tablejump_sec,
				used_bytes, trimmed_bytes, info, NULL))
	    return false;
	  /* Mark table jump profiling stage as completed.  */
	  table_jump_htab->end_idx = -1;
	  return true;
	}
    }

  /* Examine and consider relaxing each reloc.  */
  for (i = 0; i < sec->reloc_count; i++)
    {
      asection *sym_sec;
      Elf_Internal_Rela *rel = relocs + i;
      relax_func_t relax_func;
      int type = ELFNN_R_TYPE (rel->r_info);
      bfd_vma symval;
      char symtype;
      bool undefined_weak = false;

      relax_func = NULL;
      if (info->relax_pass == PASS_SHORTEN_ORG)
	{
	  if (andes->set_relax_call &&
	      (type == R_RISCV_CALL
	       || type == R_RISCV_CALL_PLT))
	    relax_func = _bfd_riscv_relax_call;
	  else if (andes->set_relax_lui &&
		   (type == R_RISCV_HI20
		    || type == R_RISCV_LO12_I
		    || type == R_RISCV_LO12_S))
	    relax_func = _bfd_riscv_relax_lui;
	  else if (andes->set_relax_tls_le &&
		   (type == R_RISCV_TPREL_HI20
		    || type == R_RISCV_TPREL_ADD
		    || type == R_RISCV_TPREL_LO12_I
		    || type == R_RISCV_TPREL_LO12_S))
	    relax_func = _bfd_riscv_relax_tls_le;
	  else if (!bfd_link_pic (info)
		   && andes->set_relax_pc
		   && (type == R_RISCV_PCREL_HI20
		       || type == R_RISCV_PCREL_LO12_I
		       || type == R_RISCV_PCREL_LO12_S))
	    relax_func = _bfd_riscv_relax_pc;
	  else
	    continue;

	  /* Only relax this reloc if it is paired with R_RISCV_RELAX.  */
	  if (type != R_RISCV_JAL && (i == sec->reloc_count - 1
		|| ELFNN_R_TYPE ((rel + 1)->r_info) != R_RISCV_RELAX
		|| rel->r_offset != (rel + 1)->r_offset))
	    continue;

	  /* Skip over the R_RISCV_RELAX.  */
	  if (type != R_RISCV_JAL)
	    i++;
	}
      else if (info->relax_pass == PASS_ZCE_TABLE_JUMP_COLLECT)
	{
	  if (!riscv_use_table_jump (info))
	    return true;

	  if (info->relax_trip == 0 || info->relax_trip == 2)
	    {
	      if (type == R_RISCV_CALL
		  || type == R_RISCV_CALL_PLT)
		relax_func = _bfd_riscv_relax_call;
	      else if (type == R_RISCV_JAL)
		relax_func = _bfd_riscv_record_jal;
	      else
		continue;

	      /* Only relax this reloc if it is paired with R_RISCV_RELAX.  */
	      if (type != R_RISCV_JAL && (i == sec->reloc_count - 1
		    || ELFNN_R_TYPE ((rel + 1)->r_info) != R_RISCV_RELAX
		    || rel->r_offset != (rel + 1)->r_offset))
		continue;

	      /* Skip over the R_RISCV_RELAX.  */
	      if (type != R_RISCV_JAL)
		i++;

	      *again = true;

	      if (info->relax_trip == 2)
		relax_func = _bfd_riscv_table_jump_mark;
	    }
	}
      else if (info->relax_pass == PASS_ZCE_TABLE_JUMP_APPLY)
	{
	  if (!riscv_use_table_jump (info))
	    return true;

	  if (andes->set_relax_call &&
	      (type == R_RISCV_CALL
	       || type == R_RISCV_CALL_PLT))
	    relax_func = _bfd_riscv_relax_call;
	  else if (type == R_RISCV_JAL)
	    relax_func = _bfd_riscv_record_jal;
	  else
	    continue;

	  /* Only relax this reloc if it is paired with R_RISCV_RELAX.  */
	  if (type != R_RISCV_JAL && (i == sec->reloc_count - 1
		|| ELFNN_R_TYPE ((rel + 1)->r_info) != R_RISCV_RELAX
		|| rel->r_offset != (rel + 1)->r_offset))
	    continue;

	  /* Skip over the R_RISCV_RELAX.  */
	  if (type != R_RISCV_JAL)
	    i++;
	}
      else if (info->relax_pass == PASS_ANDES_GP_PCREL)
	{
	  if (!bfd_link_pic (info)
	      && andes->set_relax_pc
	      && (type == R_RISCV_PCREL_HI20
		  || type == R_RISCV_PCREL_LO12_I
		  || type == R_RISCV_PCREL_LO12_S))
	    relax_func = andes_relax_pc_gp_insn;
	  else
	    continue;

	  /* Only relax this reloc if it is paired with R_RISCV_RELAX.  */
	  if (i == sec->reloc_count - 1
	      || ELFNN_R_TYPE ((rel + 1)->r_info) != R_RISCV_RELAX
	      || rel->r_offset != (rel + 1)->r_offset)
	    continue;

	  /* Skip over the R_RISCV_RELAX.  */
	  i++;
	}
      else if (info->relax_pass == PASS_ANDES_GP_1)
	{
	  if (andes->set_relax_lui &&
	      (type == R_RISCV_LO12_I || type == R_RISCV_LO12_S))
	    relax_func = _bfd_riscv_relax_lui_gp_insn;
	  else
	    continue;

	  /* Only relax this reloc if it is paired with R_RISCV_RELAX.  */
	  if (i == sec->reloc_count - 1
	      || ELFNN_R_TYPE ((rel + 1)->r_info) != R_RISCV_RELAX
	      || rel->r_offset != (rel + 1)->r_offset)
	    continue;

	  /* Skip over the R_RISCV_RELAX.  */
	  i++;
	}
      else if (info->relax_pass == PASS_ANDES_GP_2)
	{
	  if (andes->set_relax_lui && (type == R_RISCV_HI20))
	    relax_func = _bfd_riscv_relax_lui_gp_insn;
	  else
	    continue;

	  /* Only relax this reloc if it is paired with R_RISCV_RELAX.  */
	  if (i == sec->reloc_count - 1
	      || ELFNN_R_TYPE ((rel + 1)->r_info) != R_RISCV_RELAX
	      || rel->r_offset != (rel + 1)->r_offset)
	    continue;

	  /* Skip over the R_RISCV_RELAX.  */
	  i++;
	}
      else if (info->relax_pass == PASS_DELETE_ORG && type == R_RISCV_DELETE)
	relax_func = _bfd_riscv_relax_delete;
      else if (info->relax_pass == PASS_ALIGN_ORG
	       && andes->set_relax_align
	       && (type == R_RISCV_ALIGN
		   || type == R_RISCV_ALIGN_BTB)) /* Andes */
	relax_func = (type == R_RISCV_ALIGN_BTB) ?
	  _bfd_riscv_relax_align_btb : _bfd_riscv_relax_align;
      else if (info->relax_pass == PASS_RESLOVE && type == R_RISCV_ANDES_TAG)
	{
	  #if 0 /* it's not safe to do relocation here!
		   as VMA has not determined yet.  */
	  int tag = ELFNN_R_SYM (rel->r_info);
	  if (tag == R_RISCV_EXECIT_ITE)
	    { /* convert relocation for execit_ite.  */
	      andes_irelx_t *irel_ext = (andes_irelx_t *) rel->r_addend;
	      max_alignment = irel_ext->annotation; /* execit_index  */
	      /* rel->r_offset might be changed!  */
	      rel->r_info = irel_ext->saved_irel.r_info;
	      /* TODO: assert addend unchanged?  */
	      rel->r_addend = irel_ext->saved_irel.r_addend;
	      relax_func = andes_relax_execit_ite;
	    }
	  else if (ELFNN_R_SYM (rel->r_info) == TAG_GPREL_SUBTYPE_FLX
		   || ELFNN_R_SYM (rel->r_info) == TAG_GPREL_SUBTYPE_FSX)
	    { /* F[LS]X rd, *sym(gp) */
	      andes_irelx_t *irel_ext =
	        (andes_irelx_t *) rel->r_addend;
	      rel->r_info = irel_ext->saved_irel.r_info;
	      rel->r_addend = irel_ext->saved_irel.r_addend;
	      relax_func = andes_relax_fls_gp;
	    }
	  else
	    BFD_ASSERT (0);
	  #else
	  continue;
	  #endif
	}
      else
	continue;

      data->relocs = relocs;

      /* Read this BFD's contents if we haven't done so already.  */
      if (!data->this_hdr.contents
	  && !bfd_malloc_and_get_section (abfd, sec, &data->this_hdr.contents))
	goto fail;

      /* Read this BFD's symbols if we haven't done so already.  */
      if (symtab_hdr->sh_info != 0
	  && !symtab_hdr->contents
	  && !(symtab_hdr->contents =
	       (unsigned char *) bfd_elf_get_elf_syms (abfd, symtab_hdr,
						       symtab_hdr->sh_info,
						       0, NULL, NULL, NULL)))
	goto fail;

      /* Get the value of the symbol referred to by the reloc.  */
      if (ELFNN_R_SYM (rel->r_info) < symtab_hdr->sh_info)
	{
	  /* A local symbol.  */
	  Elf_Internal_Sym *isym = ((Elf_Internal_Sym *) symtab_hdr->contents
				    + ELFNN_R_SYM (rel->r_info));
	  reserve_size = (isym->st_size - rel->r_addend) > isym->st_size
	    ? 0 : isym->st_size - rel->r_addend;

	  /* Relocate against local STT_GNU_IFUNC symbol.  we have created
	     a fake global symbol entry for this, so deal with the local ifunc
	     as a global.  */
	  if (ELF_ST_TYPE (isym->st_info) == STT_GNU_IFUNC)
	    continue;

	  if (isym->st_shndx == SHN_UNDEF)
	    sym_sec = sec, symval = rel->r_offset;
	  else
	    {
	      BFD_ASSERT (isym->st_shndx < elf_numsections (abfd));
	      sym_sec = elf_elfsections (abfd)[isym->st_shndx]->bfd_section;
#if 0
	      /* The purpose of this code is unknown.  It breaks linker scripts
		 for embedded development that place sections at address zero.
		 This code is believed to be unnecessary.  Disabling it but not
		 yet removing it, in case something breaks.  */
	      if (sec_addr (sym_sec) == 0)
		continue;
#endif
	      symval = isym->st_value;
	    }
	  symtype = ELF_ST_TYPE (isym->st_info);
	}
      else
	{
	  unsigned long indx;
	  struct elf_link_hash_entry *h;

	  indx = ELFNN_R_SYM (rel->r_info) - symtab_hdr->sh_info;
	  h = elf_sym_hashes (abfd)[indx];

	  while (h->root.type == bfd_link_hash_indirect
		 || h->root.type == bfd_link_hash_warning)
	    h = (struct elf_link_hash_entry *) h->root.u.i.link;

	  /* Disable the relaxation for ifunc.  */
	  if (h != NULL && h->type == STT_GNU_IFUNC)
	    continue;

	  if (h->root.type == bfd_link_hash_undefweak
	      && (relax_func == _bfd_riscv_relax_lui
		  || relax_func == _bfd_riscv_relax_pc))
	    {
	      /* For the lui and auipc relaxations, since the symbol
		 value of an undefined weak symbol is always be zero,
		 we can optimize the patterns into a single LI/MV/ADDI
		 instruction.

		 Note that, creating shared libraries and pie output may
		 break the rule above.  Fortunately, since we do not relax
		 pc relocs when creating shared libraries and pie output,
		 and the absolute address access for R_RISCV_HI20 isn't
		 allowed when "-fPIC" is set, the problem of creating shared
		 libraries can not happen currently.  Once we support the
		 auipc relaxations when creating shared libraries, then we will
		 need the more rigorous checking for this optimization.  */
	      undefined_weak = true;
	    }

	  /* This line has to match the check in riscv_elf_relocate_section
	     in the R_RISCV_CALL[_PLT] case.  */
	  if (bfd_link_pic (info) && h->plt.offset != MINUS_ONE)
	    {
	      sym_sec = htab->elf.splt;
	      symval = h->plt.offset;
	    }
	  else if (undefined_weak)
	    {
	      symval = 0;
	      sym_sec = bfd_und_section_ptr;
	    }
	  else if ((h->root.type == bfd_link_hash_defined
		    || h->root.type == bfd_link_hash_defweak)
		   && h->root.u.def.section != NULL
		   && h->root.u.def.section->output_section != NULL)
	    {
	      symval = h->root.u.def.value;
	      sym_sec = h->root.u.def.section;
	    }
	  else
	    continue;

	  if (h->type != STT_FUNC)
	    reserve_size =
	      (h->size - rel->r_addend) > h->size ? 0 : h->size - rel->r_addend;
	  symtype = h->type;
	}

      if (sym_sec->sec_info_type == SEC_INFO_TYPE_MERGE
          && (sym_sec->flags & SEC_MERGE))
	{
	  /* At this stage in linking, no SEC_MERGE symbol has been
	     adjusted, so all references to such symbols need to be
	     passed through _bfd_merged_section_offset.  (Later, in
	     relocate_section, all SEC_MERGE symbols *except* for
	     section symbols have been adjusted.)

	     gas may reduce relocations against symbols in SEC_MERGE
	     sections to a relocation against the section symbol when
	     the original addend was zero.  When the reloc is against
	     a section symbol we should include the addend in the
	     offset passed to _bfd_merged_section_offset, since the
	     location of interest is the original symbol.  On the
	     other hand, an access to "sym+addend" where "sym" is not
	     a section symbol should not include the addend;  Such an
	     access is presumed to be an offset from "sym";  The
	     location of interest is just "sym".  */
	   if (symtype == STT_SECTION)
	     symval += rel->r_addend;

	   symval = _bfd_merged_section_offset (abfd, &sym_sec,
						elf_section_data (sym_sec)->sec_info,
						symval);

	   if (symtype != STT_SECTION)
	     symval += rel->r_addend;
	}
      else
	symval += rel->r_addend;

      symval += sec_addr (sym_sec);

      if (!relax_func (abfd, sec, sym_sec, info, rel, symval,
		       max_alignment, reserve_size, again,
		       &pcgp_relocs, undefined_weak))
	goto fail;
    }

  ret = true;

 fail:
  if (relocs != data->relocs)
    free (relocs);
  if (info->relax_pass == PASS_ANDES_GP_PCREL)
    andes_relax_pc_gp_insn_final(&pcgp_relocs);
  riscv_free_pcgp_relocs (&pcgp_relocs, abfd, sec);


  /* { Andes */
  /* Free the unused info for relax_lui_gp_insn.  */
  struct relax_gp_sym_info *temp;
  if (info->relax_pass == 7)
    while (relax_gp_sym_info_head != NULL)
      {
	temp = relax_gp_sym_info_head;
	relax_gp_sym_info_head = relax_gp_sym_info_head->next;
	free (temp);
      }
  /* } Andes */

  return ret;
}

#if ARCH_SIZE == 32
# define PRSTATUS_SIZE			204
# define PRSTATUS_OFFSET_PR_CURSIG	12
# define PRSTATUS_OFFSET_PR_PID		24
# define PRSTATUS_OFFSET_PR_REG		72
# define ELF_GREGSET_T_SIZE		128
# define PRPSINFO_SIZE			128
# define PRPSINFO_OFFSET_PR_PID		16
# define PRPSINFO_OFFSET_PR_FNAME	32
# define PRPSINFO_OFFSET_PR_PSARGS	48
# define PRPSINFO_PR_FNAME_LENGTH	16
# define PRPSINFO_PR_PSARGS_LENGTH	80
#else
# define PRSTATUS_SIZE			376
# define PRSTATUS_OFFSET_PR_CURSIG	12
# define PRSTATUS_OFFSET_PR_PID		32
# define PRSTATUS_OFFSET_PR_REG		112
# define ELF_GREGSET_T_SIZE		256
# define PRPSINFO_SIZE			136
# define PRPSINFO_OFFSET_PR_PID		24
# define PRPSINFO_OFFSET_PR_FNAME	40
# define PRPSINFO_OFFSET_PR_PSARGS	56
# define PRPSINFO_PR_FNAME_LENGTH	16
# define PRPSINFO_PR_PSARGS_LENGTH	80
#endif

/* Write PRSTATUS and PRPSINFO note into core file.  This will be called
   before the generic code in elf.c.  By checking the compiler defines we
   only perform any action here if the generic code would otherwise not be
   able to help us.  The intention is that bare metal core dumps (where the
   prstatus_t and/or prpsinfo_t might not be available) will use this code,
   while non bare metal tools will use the generic elf code.  */

static char *
riscv_write_core_note (bfd *abfd ATTRIBUTE_UNUSED,
                       char *buf ATTRIBUTE_UNUSED,
                       int *bufsiz ATTRIBUTE_UNUSED,
                       int note_type ATTRIBUTE_UNUSED, ...)
{
  switch (note_type)
    {
    default:
      return NULL;

#if !defined (HAVE_PRPSINFO_T)
    case NT_PRPSINFO:
      {
	char data[PRPSINFO_SIZE] ATTRIBUTE_NONSTRING;
	va_list ap;

	va_start (ap, note_type);
	memset (data, 0, sizeof (data));
	strncpy (data + PRPSINFO_OFFSET_PR_FNAME, va_arg (ap, const char *),
                 PRPSINFO_PR_FNAME_LENGTH);
#if GCC_VERSION == 8000 || GCC_VERSION == 8001
	DIAGNOSTIC_PUSH;
	/* GCC 8.0 and 8.1 warn about 80 equals destination size with
	   -Wstringop-truncation:
	   https://gcc.gnu.org/bugzilla/show_bug.cgi?id=85643
	 */
	DIAGNOSTIC_IGNORE_STRINGOP_TRUNCATION;
#endif
	strncpy (data + PRPSINFO_OFFSET_PR_PSARGS, va_arg (ap, const char *),
                 PRPSINFO_PR_PSARGS_LENGTH);
#if GCC_VERSION == 8000 || GCC_VERSION == 8001
	DIAGNOSTIC_POP;
#endif
	va_end (ap);
	return elfcore_write_note (abfd, buf, bufsiz,
				   "CORE", note_type, data, sizeof (data));
      }
#endif /* !HAVE_PRPSINFO_T */

#if !defined (HAVE_PRSTATUS_T)
    case NT_PRSTATUS:
      {
        char data[PRSTATUS_SIZE];
        va_list ap;
        long pid;
        int cursig;
        const void *greg;

        va_start (ap, note_type);
        memset (data, 0, sizeof(data));
        pid = va_arg (ap, long);
        bfd_put_32 (abfd, pid, data + PRSTATUS_OFFSET_PR_PID);
        cursig = va_arg (ap, int);
        bfd_put_16 (abfd, cursig, data + PRSTATUS_OFFSET_PR_CURSIG);
        greg = va_arg (ap, const void *);
        memcpy (data + PRSTATUS_OFFSET_PR_REG, greg,
                PRSTATUS_SIZE - PRSTATUS_OFFSET_PR_REG - ARCH_SIZE / 8);
        va_end (ap);
        return elfcore_write_note (abfd, buf, bufsiz,
                                   "CORE", note_type, data, sizeof (data));
      }
#endif /* !HAVE_PRSTATUS_T */
    }
}

/* Support for core dump NOTE sections.  */

static bool
riscv_elf_grok_prstatus (bfd *abfd, Elf_Internal_Note *note)
{
  switch (note->descsz)
    {
      default:
	return false;

      case PRSTATUS_SIZE: /* sizeof(struct elf_prstatus) on Linux/RISC-V.  */
	/* pr_cursig */
	elf_tdata (abfd)->core->signal
	  = bfd_get_16 (abfd, note->descdata + PRSTATUS_OFFSET_PR_CURSIG);

	/* pr_pid */
	elf_tdata (abfd)->core->lwpid
	  = bfd_get_32 (abfd, note->descdata + PRSTATUS_OFFSET_PR_PID);
	break;
    }

  /* Make a ".reg/999" section.  */
  return _bfd_elfcore_make_pseudosection (abfd, ".reg", ELF_GREGSET_T_SIZE,
					  note->descpos + PRSTATUS_OFFSET_PR_REG);
}

static bool
riscv_elf_grok_psinfo (bfd *abfd, Elf_Internal_Note *note)
{
  switch (note->descsz)
    {
      default:
	return false;

      case PRPSINFO_SIZE: /* sizeof(struct elf_prpsinfo) on Linux/RISC-V.  */
	/* pr_pid */
	elf_tdata (abfd)->core->pid
	  = bfd_get_32 (abfd, note->descdata + PRPSINFO_OFFSET_PR_PID);

	/* pr_fname */
	elf_tdata (abfd)->core->program = _bfd_elfcore_strndup
	  (abfd, note->descdata + PRPSINFO_OFFSET_PR_FNAME,
           PRPSINFO_PR_FNAME_LENGTH);

	/* pr_psargs */
	elf_tdata (abfd)->core->command = _bfd_elfcore_strndup
	  (abfd, note->descdata + PRPSINFO_OFFSET_PR_PSARGS,
           PRPSINFO_PR_PSARGS_LENGTH);
	break;
    }

  /* Note that for some reason, a spurious space is tacked
     onto the end of the args in some (at least one anyway)
     implementations, so strip it off if it exists.  */

  {
    char *command = elf_tdata (abfd)->core->command;
    int n = strlen (command);

    if (0 < n && command[n - 1] == ' ')
      command[n - 1] = '\0';
  }

  return true;
}

/* Set the right mach type.  */

static bool
riscv_elf_object_p (bfd *abfd)
{
  /* There are only two mach types in RISCV currently.  */
  if (strcmp (abfd->xvec->name, "elf32-littleriscv") == 0
      || strcmp (abfd->xvec->name, "elf32-bigriscv") == 0)
    bfd_default_set_arch_mach (abfd, bfd_arch_riscv, bfd_mach_riscv32);
  else
    bfd_default_set_arch_mach (abfd, bfd_arch_riscv, bfd_mach_riscv64);

  /* { Andes */
  /* Build the ict hash table to store all global symbols attached
     with ICT suffix.  */
  if (!ict.is_init)
    {
      riscv_elf_ict_init ();
      ict.is_init = 1;
    }
  /* } Andes */

  return true;
}

/* Determine whether an object attribute tag takes an integer, a
   string or both.  */

static int
riscv_elf_obj_attrs_arg_type (int tag)
{
  return (tag & 1) != 0 ? ATTR_TYPE_FLAG_STR_VAL : ATTR_TYPE_FLAG_INT_VAL;
}

static bool
riscv_final_link (bfd * abfd, struct bfd_link_info * info)
{
  struct riscv_elf_link_hash_table *htab = riscv_elf_hash_table (info);

  if (!bfd_elf_final_link (abfd, info))
    return false;

  /* Andes: set exec.it table contents.  */
  while (true) /* once */
    {
      asection *itable_sec = andes_execit_get_itable_section (info->input_bfds);
      bfd_byte *contents = NULL;
      if (itable_sec == NULL)
	break;
      contents = elf_section_data (itable_sec)->this_hdr.contents;
      if (contents == NULL)
	break;
      if (!bfd_set_section_contents (abfd,
		itable_sec->output_section,
		contents,
		(file_ptr) itable_sec->output_offset,
		itable_sec->size))
	return false;
      break; /* once */
    }

  if (riscv_use_table_jump (info)
      /* tablejump_sec is not created if no relocation happened, so we
        need to check section pointer here.  */
      && htab->table_jump_htab->tablejump_sec)
    {
      asection *sec = htab->table_jump_htab->tablejump_sec;
      asection *out_sec = sec->output_section;
      uintNN_t buf[256];
      uintNN_t *data;

      if (ARCH_SIZE == 32)
	{
	  data = buf;
	  for (int i=0; i < 256; ++i)
	    data[i] = (uintNN_t) htab->table_jump_htab->tbj_indexes[i];
	}
      else
	data = (uintNN_t*) htab->table_jump_htab->tbj_indexes;

      if (!bfd_set_section_contents (abfd,
		out_sec,
		data,
		(file_ptr) sec->output_offset,
		sec->size))
	return false;
    }

  return true;
}

/* Do not choose mapping symbols as a function name.  */

static bfd_size_type
riscv_maybe_function_sym (const asymbol *sym,
			  asection *sec,
			  bfd_vma *code_off)
{
  if (sym->flags & BSF_LOCAL
      && riscv_elf_is_mapping_symbols (sym->name))
    return 0;

  return _bfd_elf_maybe_function_sym (sym, sec, code_off);
}

/* Treat the following cases as target special symbols, they are
   usually omitted.  */

static bool
riscv_elf_is_target_special_symbol (bfd *abfd, asymbol *sym)
{
  /* PR27584, local and empty symbols.  Since they are usually
     generated for pcrel relocations.  */
  return (!strcmp (sym->name, "")
	  || _bfd_elf_is_local_label_name (abfd, sym->name)
	  /* PR27916, mapping symbols.  */
	  || riscv_elf_is_mapping_symbols (sym->name));
}

static int
riscv_elf_additional_program_headers (bfd *abfd,
				      struct bfd_link_info *info ATTRIBUTE_UNUSED)
{
  int ret = 0;

  /* See if we need a PT_RISCV_ATTRIBUTES segment.  */
  if (bfd_get_section_by_name (abfd, RISCV_ATTRIBUTES_SECTION_NAME))
    ++ret;

  return ret;
}

static bool
riscv_elf_modify_segment_map (bfd *abfd,
			      struct bfd_link_info *info ATTRIBUTE_UNUSED)
{
  asection *s;
  struct elf_segment_map *m, **pm;
  size_t amt;

  /* If there is a .riscv.attributes section, we need a PT_RISCV_ATTRIBUTES
     segment.  */
  s = bfd_get_section_by_name (abfd, RISCV_ATTRIBUTES_SECTION_NAME);
  if (s != NULL)
    {
      for (m = elf_seg_map (abfd); m != NULL; m = m->next)
	if (m->p_type == PT_RISCV_ATTRIBUTES)
	  break;
      /* If there is already a PT_RISCV_ATTRIBUTES header, avoid adding
	 another.  */
      if (m == NULL)
	{
	  amt = sizeof (*m);
	  m = bfd_zalloc (abfd, amt);
	  if (m == NULL)
	    return false;

	  m->p_type = PT_RISCV_ATTRIBUTES;
	  m->count = 1;
	  m->sections[0] = s;

	  /* We want to put it after the PHDR and INTERP segments.  */
	  pm = &elf_seg_map (abfd);
	  while (*pm != NULL
		 && ((*pm)->p_type == PT_PHDR
		     || (*pm)->p_type == PT_INTERP))
	    pm = &(*pm)->next;

	  m->next = *pm;
	  *pm = m;
	}
    }

  return true;
}

/* Merge non-visibility st_other attributes.  */

static void
riscv_elf_merge_symbol_attribute (struct elf_link_hash_entry *h,
				  unsigned int st_other,
				  bool definition ATTRIBUTE_UNUSED,
				  bool dynamic ATTRIBUTE_UNUSED)
{
  unsigned int isym_sto = st_other & ~ELF_ST_VISIBILITY (-1);
  unsigned int h_sto = h->other & ~ELF_ST_VISIBILITY (-1);

  if (isym_sto == h_sto)
    return;

  if (isym_sto & ~STO_RISCV_VARIANT_CC)
    _bfd_error_handler (_("unknown attribute for symbol `%s': 0x%02x"),
			h->root.root.string, isym_sto);

  if (isym_sto & STO_RISCV_VARIANT_CC)
    h->other |= STO_RISCV_VARIANT_CC;
}

/* { Andes  */
/* Return the symbol DATA_START_SYMBOLS value, or 0 if it is not in use.  */

static bfd_vma
riscv_data_start_value (const struct bfd_link_info *info)
{
  struct bfd_link_hash_entry *h;

  h = bfd_link_hash_lookup (info->hash, "__DATA_BEGIN__", false, false, true);
  if (h == NULL || h->type != bfd_link_hash_defined)
    return 0;

  return h->u.def.value + sec_addr (h->u.def.section);
}

static relax_gp_sym_info_t*
record_and_find_relax_gp_syms (asection *sec,
			       Elf_Internal_Sym *lsym,
			       struct elf_link_hash_entry *h,
			       int record)
{
  struct relax_gp_sym_info *ptr, *pre_ptr;
  ptr = relax_gp_sym_info_head;
  pre_ptr = ptr;

  /* Check whether the symbol is recorded.  */
  while (ptr)
    {
      if ((h && h == ptr->h && sec == ptr->sec)
	  || (lsym && lsym == ptr->lsym && sec == ptr->sec))
	return ptr;
      pre_ptr = ptr;
      ptr = ptr->next;
    }

  if (!ptr && record)
    {
      ptr = bfd_malloc (sizeof (struct relax_gp_sym_info));
      ptr->sec = sec;
      ptr->lsym = lsym;
      ptr->h = h;
      ptr->next = NULL;

      if (!relax_gp_sym_info_head)
	relax_gp_sym_info_head = ptr;
      else
	pre_ptr->next = ptr;
    }

  return NULL;
}

/* Relax non-PIC global variable references to gp relative instructions.  */
/* Relax pass 1: only low part insns
   Relax pass 2: only hi part isns.  */

static bool
_bfd_riscv_relax_lui_gp_insn (bfd *abfd, asection *sec, asection *sym_sec,
			      struct bfd_link_info *info,
			      Elf_Internal_Rela *rel,
			      bfd_vma symval,
			      bfd_vma max_alignment,
			      bfd_vma reserve_size,
			      bool *again ATTRIBUTE_UNUSED,
			      riscv_pcgp_relocs *pcgp_relocs ATTRIBUTE_UNUSED,
			      bool undefined_weak)
{
  bfd_byte *contents = elf_section_data (sec)->this_hdr.contents;
  bfd_vma gp = riscv_global_pointer_value (info);
  bfd_vma data_start = riscv_data_start_value (info);
  Elf_Internal_Shdr *symtab_hdr = &elf_symtab_hdr (abfd);
  Elf_Internal_Sym *isym = NULL;
  struct elf_link_hash_entry *h = NULL;
  struct riscv_elf_link_hash_table *htab = riscv_elf_hash_table (info);
  andes_ld_options_t *andes = &htab->andes;
  bfd_vma guard_size = 0;
  bfd_signed_vma effect_range;

#if 0
  bfd_vma data_start = riscv_data_start_value (info);
  /* Mergeable symbols and code might later move out of range.  */
  /* For bug-14274, symbols defined in the .rodata (the sections
     before .data, may also later move out of range.  */
  if (sym_sec->flags & (SEC_MERGE | SEC_CODE)
      || (data_start && sec_addr (sym_sec) < data_start))
    return true;
#endif

  BFD_ASSERT (rel->r_offset + 4 <= sec->size);

  if (undefined_weak)
    return true; /* bypass to original handling  */

  /* For bug-14274, symbols defined in the .rodata (the sections
     before .data, may also later move out of range.  */
  /* reserved one page size in worst case
     or two (refer to riscv_relax_lui_to_rvc)  */
  if ((data_start == 0) || (sec_addr (sym_sec) < data_start))
    guard_size += info->relro ? andes->set_relax_page_size * 2
				   : andes->set_relax_page_size;
  if (gp)
    {
      /* If gp and the symbol are in the same output section, which is not the
	 abs section, then consider only that output section's alignment.  */
      struct bfd_link_hash_entry *hh =
	bfd_link_hash_lookup (info->hash, RISCV_GP_SYMBOL, false, false, true);
      if (hh->u.def.section->output_section == sym_sec->output_section
	  && sym_sec->output_section != bfd_abs_section_ptr)
	max_alignment = (bfd_vma) 1 << sym_sec->output_section->alignment_power;
    }

  /* FIXME: Since we do not have grouping mechanism like v3, we may
     delete the high part insn that the corresponding low part insn
     isn't converted to gp relative one. Therefore, we record the
     symbols referenced by the non-relaxed low part insns in the
     relax Pass 1 round, and then do not delete the high part insns,
     which reference these symbols in the relax Pass 2 round.  */

  if (symtab_hdr->sh_info != 0
      && !symtab_hdr->contents
      && !(symtab_hdr->contents =
	   (unsigned char *) bfd_elf_get_elf_syms (abfd, symtab_hdr,
						   symtab_hdr->sh_info,
						   0, NULL, NULL, NULL)))
    return false;

  if (ELFNN_R_SYM (rel->r_info) < symtab_hdr->sh_info)
    /* A local symbol.  */
    isym = ((Elf_Internal_Sym *) symtab_hdr->contents
	    + ELFNN_R_SYM (rel->r_info));
  else
    {
      /* A global symbol.  */
      unsigned long indx;
      indx = ELFNN_R_SYM (rel->r_info) - symtab_hdr->sh_info;
      h = elf_sym_hashes (abfd)[indx];

      while (h->root.type == bfd_link_hash_indirect
	     || h->root.type == bfd_link_hash_warning)
	h = (struct elf_link_hash_entry *) h->root.u.i.link;
    }

  /* Enable nds v5 gp relative insns.  */
  int do_replace = 0;
  uint32_t insn = bfd_get_32 (abfd, contents + rel->r_offset);
  /* For Bug-16488, check if gp-relative offset is in range.  */
  const int max_range = 0x20000;
  guard_size += max_alignment + reserve_size;
  effect_range = max_range - guard_size;
  if (effect_range < 0)
    return true; /* out of range */
  if (((symval >= gp) && ((symval - gp) < (bfd_vma) effect_range)) ||
      ((symval < gp) && ((gp - symval) <= (bfd_vma) effect_range)))
    {
      riscv_pcgp_hi_reloc_t hi;
      do_replace = 1;
      hi.hi_sym = ELFNN_R_SYM (rel->r_info);
      hi.hi_addend = 0;
      if (ELFNN_R_TYPE (rel->r_info) == R_RISCV_HI20
	  && !record_and_find_relax_gp_syms (sym_sec, isym, h, 0))
	{
	  /* The HI20 can be deleted safely.  */
	  rel->r_info = ELFNN_R_INFO (0, R_RISCV_DELETE);
	  rel->r_addend = 4;
	  return true;
	}
      else
	do_replace = andes_relax_gp_insn (&insn, rel, &hi, sym_sec,
					  symval, gp, data_start);

      if (do_replace)
	bfd_put_32 (abfd, insn, contents + rel->r_offset);
      else
	/* The low insn can not be relaxed to v5 gp-relative insn.
	   Record the referenced symbol.  */
	record_and_find_relax_gp_syms (sym_sec, isym, h, 1);
    }

  /* Do not relax lui to c.lui here since the dangerous delete
     behavior.  */
  return true;
}

/* Generate EXECIT hash table.  */

static bool
andes_execit_hash_insn (bfd *abfd, asection *sec,
			struct bfd_link_info *link_info)
{
  bfd_byte *contents = NULL;
  Elf_Internal_Sym *isym = NULL;
  struct riscv_elf_link_hash_table *htab = riscv_elf_hash_table (link_info);
  bfd_vma off = 0;
  Elf_Internal_Rela *internal_relocs;
  Elf_Internal_Rela *irelend;
  Elf_Internal_Rela *irel;
  execit_context_t ctx;
  uint32_t insn;
  int data_flag;
  int is_on_relocation;
  int rtype;
  const char *hash = ctx.buf;

  ctx.abfd = abfd;
  ctx.sec = sec;
  ctx.info = link_info;

  /* Load section instructions, relocations, and symbol table.  */
  if (!riscv_get_section_contents (abfd, sec, &contents, true)
      || !riscv_get_local_syms (abfd, sec, &isym))
    return false;

  internal_relocs = _bfd_elf_link_read_relocs (abfd, sec, NULL, NULL,
					       true /* keep_memory  */);
  irelend = internal_relocs + sec->reloc_count;

  /* Check the input section enable EXECIT?  */
  irel = find_relocs_at_address (internal_relocs, internal_relocs, irelend,
				 R_RISCV_RELAX_ENTRY);

  /* Check this input section trigger EXECIT relaxation.  */
  rtype = ELFNN_R_TYPE (irel->r_info);
  if (irel == NULL || irel >= irelend || rtype != R_RISCV_RELAX_ENTRY
      || (rtype == R_RISCV_RELAX_ENTRY
	  && !(irel->r_addend & R_RISCV_RELAX_ENTRY_EXECIT_FLAG)))
    return true;

  /* hash insn. in andes_gen_execit_hash()  */
  while (off < sec->size)
    {
      execit_hash_t *he;

      /* locate next nearby relocation  */
      while (irel != NULL && irel < irelend && irel->r_offset < off)
	irel++;

      data_flag = riscv_relocation_check (link_info, &irel, irelend, sec,
					  &off, contents, 1);

      if (data_flag & DATA_EXIST)
	{
	  off += (data_flag >> 24);
	  continue;
	}
      else if (data_flag & RELAX_REGION_END)
	{
	  continue;
	}

      /* skip 16-bit instruction.  */
      if ((*(contents + off) & 0x3) != 0x3)
	{
	  off += 2;
	  continue;
	}

      /* filter out some sorts of pattern unsafe or hard to exec.it  */
      insn = bfd_get_32 (abfd, contents + off);
      if (!riscv_elf_execit_check_insn_available (insn, htab))
	{
	  off += 4;
	  continue;
	}

      is_on_relocation =
	(irel != NULL &&
         irel < irelend &&
	 irel->r_offset == off &&
	 data_flag & SYMBOL_RELOCATION) ?
	true : false;

      memset (&ctx.ie, 0, sizeof (ctx.ie));
      ctx.ie.insn = insn;
      ctx.irel = is_on_relocation ? irel : NULL;
      ctx.off = off;
      if (andes_execit_render_hash (&ctx) != EXECIT_HASH_OK)
	{
	  off += 4;
	  continue;
	}

      /* add hash entry.  */
      he = (execit_hash_t*)
	bfd_hash_lookup (&execit.code_hash, hash, true, true);
      if (he == NULL)
	{
	  (*_bfd_error_handler)
	    (_("Linker: failed creating exec.it %s hash table\n"), hash);
	  return false;
	}

      /* special handlings:
       *  LUI/AUIPC: log addresses to calcute itable entries to reserve.
       */
      rtype = ctx.irel ? ELFNN_R_TYPE (ctx.irel->r_info) : 0;
      if (ctx.irel
	  && (rtype == R_RISCV_HI20 || rtype == R_RISCV_CALL
	      || rtype == R_RISCV_PCREL_HI20))
	{
	  execit_irel_t *e = bfd_zmalloc (sizeof (execit_irel_t));
	  e->ie = ctx.ie;
	  LIST_APPEND (&he->irels, e);
	}

      if (he->ie.est_count == 0)
	he->ie = ctx.ie;

      he->ie.est_count++;
      off += 4;
    }
  return true;
}

/* Hash table traverse function.  */

static void
andes_execit_traverse_insn_hash (int (*func) (execit_hash_t*))
{
  unsigned int i;

  execit.code_hash.frozen = 1;
  for (i = 0; i < execit.code_hash.size; i++)
    {
      struct bfd_hash_entry *p;

      for (p = execit.code_hash.table[i]; p != NULL; p = p->next)
	if (!func ((execit_hash_t *) p))
	  goto out;
    }
out:
  execit.code_hash.frozen = 0;
}

/* Examine each insn times in hash table.
   Handle multi-link hash entry.

   NOTE: always return true to continue traversing
   TODO: This function doesn't assign so much info since it is fake.  */

#define EXECIT_GRADE(c,e) (int) (200.0 * (c - 2) / e)

static int
andes_execit_rank_insn (execit_hash_t *he)
{
  execit_itable_t *ie = &he->ie;
  Elf_Internal_Rela *irel = ie->irel;
  int rtype = irel ? ELFNN_R_TYPE (irel->r_info) : 0;

  if ((rtype == R_RISCV_HI20)
      || (rtype == R_RISCV_CALL || rtype == R_RISCV_PCREL_HI20))
    { /* HI20: LUI or AUIPC */
      andes_execit_estimate_hi20 (he);
      execit_irel_t *p = he->irels;
      int i, j;
      while (p)
	{
	  if (p->ie.est_count > (p->ie.entries * 2))
	    {
	      execit_rank_t *re = bfd_zmalloc (sizeof (execit_rank_t));
	      re->grade =  EXECIT_GRADE (p->ie.est_count, p->ie.entries);
	      re->type = ET_RK_TYPE_HI20;
	      re->data = (void*) p; /* group representative.  */
	      re->he = he;
	      he->is_worthy = true;
	      LIST_ITER (&execit.rank_list, re, rank_each_cb, append_final_cb);
	      /* skip the same ID.  */
	      for (i = 1, j = p->ie.est_count; i < j; ++i)
		p = p->next;
	    }
	  p = p->next;
	}
      return true;
    }
  else if (ie->est_count > 2)
    ie->entries = 1;
  else
    return true;

  execit_rank_t *re = bfd_zmalloc (sizeof (execit_rank_t));
  re->grade =  EXECIT_GRADE (ie->est_count, ie->entries);
  re->he = he;
  he->is_worthy = true;
  LIST_ITER (&execit.rank_list, re, rank_each_cb, append_final_cb);

  return true;
}

/* Examine each insn hash entry for imported exec.itable instructions
   NOTE: always return true to continue traversing
*/

static int
andes_execit_rank_imported_insn (execit_hash_t *he)
{
  execit_rank_t *re, *p, *pp;

  if (! he->is_imported)
    return true;

  re = bfd_zmalloc (sizeof (execit_rank_t));
  re->he = he;
  he->is_worthy = true;

  /* insert imported exec.it entries  */
  pp = NULL;
  p = execit.rank_list;
  while (p)
    {
      if ((! p->he->is_imported) ||
	  (p->he->ie.itable_index > he->ie.itable_index))
	break;
      pp = p;
      p = p->next;
    }

  re->next = p;
  if (pp)
    pp->next = re;
  else
    execit.rank_list = re;

  return true;
}

/* Build .exec.itable section.  */

static void
andes_execit_build_itable (struct bfd_link_info *info)
{
  bfd *abfd;
  asection *table_sec;
  execit_rank_t *p;
  bfd_byte *contents = NULL;
  struct riscv_elf_link_hash_table *table;
  andes_ld_options_t *andes;
  int limit; /* hardware available entries  */
  int total; /* software used entries  */
  int count; /* total insns to be replaced  */
  int order; /* rank order of (raw) hash  */
  int index; /* next entry index  */
  int has_entry = false;

  while (true)
    {
      /* Find the section .exec.itable, and put all entries into it.  */
      table_sec = andes_execit_get_itable_section (info->input_bfds);
      if (table_sec == NULL)
	break;

      table = riscv_elf_hash_table (info);
      andes = &table->andes;
      abfd = table_sec->owner;
      riscv_get_section_contents (abfd, table_sec, &contents, true);
      if (contents == NULL)
	break;

      /* skip ITB checking if there is no candidate. bug#23317  */
      if (execit.rank_list == NULL)
	break;

      /* Check ITB register if set.  */
      if (!andes->execit_import_file
	  && !bfd_link_hash_lookup (info->hash, "_ITB_BASE_",
				    false, false, true))
	{
	  (*_bfd_error_handler) (_(
	    "\nError: Instruction Table(IT) is used, but Instruction "
	    "Table Base($ITB) isn't set.\nPlease add the following "
	    "instructions in _start of the startup code"
	    "(crt0.S or start.S):\n"
	    "\"la a0, _ITB_BASE_; csrrw x0, uitb, a0\""));
	  exit (1);
	}

      /* skip if itable is imported and not to keep or to update  */
      if (andes->execit_import_file &&
	  ! andes->keep_import_execit &&
	  ! andes->update_execit_table)
	break;

      has_entry = true;
      break;
    };

  if (has_entry == false)
    {
      if (table_sec)
	table_sec->size = 0;
      return;
    }

  /* TODO: change the e_flag for EXECIT.  */

  limit = andes->execit_limit;
  /* odd old definition (v5_toolmisc_test)  */
  if (andes->execit_import_file &&
      andes->update_execit_table &&
      andes->execit_limit >= 0)
    limit += execit.next_itable_index;
  if ((limit < 0) || (limit > EXECIT_HW_ENTRY_MAX))
    limit = EXECIT_HW_ENTRY_MAX;

  /* Write EXECIT candidates into .exec.itable. We will
     relocate the patterns with relocations later
     in andes_execit_relocate_itable.  */

  /* might have imported some  */
  total = count = order = index = 0;
  for (p = execit.rank_list;
       p && index < limit;
       p = p->next)
    {
      execit_hash_t *he = p->he;
      execit_itable_t *ie = &he->ie;
      execit_irel_t *irel = p->data;

      if (p->type == ET_RK_TYPE_HI20)
	{
	  BFD_ASSERT (irel);
	  ie = &irel->ie;
	}

      if ((total + ie->entries) > limit)
	continue;

      bfd_put_32 (abfd, (bfd_vma) ie->fixed, (char *) contents + (index << 2));

      p->is_chosen = true;
      if (p->type == ET_RK_TYPE_HI20)
	{
	  for (int i = 0; i < ie->est_count; ++i)
	    {
	      BFD_ASSERT (irel);
	      irel->is_chosen = true;
	      irel->ie.rank_order = order;
	      irel->ie.itable_index = index;
	      irel = irel->next;
	    }
	}
      he->is_chosen = true;
      he->type = p->type;
      ie->rank_order = order++;
      ie->itable_index = index++;
      /* reserve one here, to allocate others on demand (R_RISCV_EXECIT_ITE)  */
      total += ie->entries;
      count += ie->est_count;

#if 0 //KILLME
      /* patch R_RISCV_PCREL_HI20 reloaction for later comparison.  */
      if (ie->irel)
	{
	  int rtype = ELFNN_R_TYPE (ie->irel_copy.r_info);
	  if (rtype == R_RISCV_PCREL_HI20)
	    {
	      bfd_vma hi20 = riscv_elf_encode_relocation (abfd, &ie->irel_copy, ie->relocation);
	      ie->relocation = hi20;
	    }
	}
#endif
    }

  table_sec->size = total << 2;

  /* build itable[0..size] = [item, ...]  */
  execit.itable_array = bfd_zmalloc (sizeof (execit_item_t) * total);
  index = 0;
  for (p = execit.rank_list;
       p && index < limit;
       p = p->next)
    {
      execit_hash_t *he = p->he;
      if (!p->is_chosen)
	continue;

      execit_item_t *item = &execit.itable_array[index];
      item->he = he;
      if (p->data) /* group representative.  */
	item->grp_hd = p->data;
      index++;
    }

  execit.raw_itable_entries = index;
  execit.next_itable_index = index;
}

/* Replace input file instruction which is in the .exec.itable.  */

static bool
andes_execit_replace_insn (struct bfd_link_info *link_info,
			   bfd *abfd, asection *sec)
{
  bfd_byte *contents = NULL;
  Elf_Internal_Sym *isym = NULL;
  struct riscv_elf_link_hash_table *htab = riscv_elf_hash_table (link_info);
  bfd_vma off = 0;
  Elf_Internal_Rela *internal_relocs;
  Elf_Internal_Rela *irelend;
  Elf_Internal_Rela *irel;
  uint32_t insn;
  int data_flag;
  int is_on_relocation;
  execit_context_t ctx;
  bool is_replace;
  
  memset (&ctx.ie, 0, sizeof (ctx.ie));
  ctx.abfd = abfd;
  ctx.sec = sec;
  ctx.info = link_info;

  /* Load section instructions, relocations, and symbol table.  */
  if (!riscv_get_section_contents (abfd, sec, &contents, true)
      || !riscv_get_local_syms (abfd, sec, &isym))
    return false;

  internal_relocs = _bfd_elf_link_read_relocs (abfd, sec, NULL, NULL,
					       true /* keep_memory  */);
  irelend = internal_relocs + sec->reloc_count;

  /* Check the input section enable EXECIT?  */
  irel = find_relocs_at_address (internal_relocs, internal_relocs, irelend,
				 R_RISCV_RELAX_ENTRY);

  /* Check this input section trigger EXECIT relaxation.  */
  if (irel == NULL
      || irel >= irelend
      || ELFNN_R_TYPE (irel->r_info) != R_RISCV_RELAX_ENTRY
      || (ELFNN_R_TYPE (irel->r_info) == R_RISCV_RELAX_ENTRY
	  && !(irel->r_addend & R_RISCV_RELAX_ENTRY_EXECIT_FLAG)))
    return true;

  /* hash insn. in andes_gen_execit_hash()  */
  char *hash = ctx.buf;
  while (off < sec->size)
    {
      execit_hash_t* entry;

      /* locate next nearby relocation  */
      while (irel != NULL && irel < irelend && irel->r_offset < off)
	irel++;

      data_flag = riscv_relocation_check (link_info, &irel, irelend, sec,
					  &off, contents, 1);

      if (data_flag & DATA_EXIST)
	{
	  off += (data_flag >> 24);
	  continue;
	}
      else if (data_flag & RELAX_REGION_END)
	{
	  continue;
	}

      /* skip 16-bit instruction.  */
      if ((*(contents + off) & 0x3) != 0x3)
	{
	  off += 2;
	  continue;
	}

      /* filter out some sorts of pattern unsafe or hard to exec.it  */
      insn = bfd_get_32 (abfd, contents + off);
      if (!riscv_elf_execit_check_insn_available (insn, htab))
	{
	  off += 4;
	  continue;
	}

      is_on_relocation =
	(irel != NULL &&
         irel < irelend &&
	 irel->r_offset == off &&
	 data_flag & SYMBOL_RELOCATION) ?
	true : false;

      ctx.irel = is_on_relocation ? irel : NULL;
      ctx.off = off;
      memset (&ctx.ie, 0, sizeof (ctx.ie));
      ctx.ie.insn = insn;
      if (andes_execit_render_hash (&ctx) != EXECIT_HASH_OK)
	{
	  off += 4;
	  continue;
	}

      /* lookup hash table.  */
      entry = (execit_hash_t*)
	bfd_hash_lookup (&execit.code_hash, hash, false, false);

      /* HI20 type might be choosen partially for exec.it.  */
      is_replace = entry && entry->is_chosen;
      if (is_replace && entry->type == ET_RK_TYPE_HI20)
	{
	  execit_irel_t *p = entry->irels;
	  while (p)
	    {
	      if (p->ie.irel == irel)
		break;
	      p = p->next;
	    }
	  if (p)
	    is_replace = p->is_chosen;
	}
      if (!is_replace)
	{
	  off += 4;
	  continue;
	}

      /* replace insn now.  */
      ctx.contents = contents;
      if (!andes_execit_push_insn (&ctx, entry))
	BFD_ASSERT (0);

      off += 4;
    } /* while off  */

  return true;
}

/* Delete blanks according to blank_list.  */

static void
andes_execit_delete_blank (struct bfd_link_info *info)
{
  execit_blank_abfd_t *pabfd = execit.blank_list;
  execit_blank_abfd_t *qabfd = NULL;

  while (pabfd)
    {
      qabfd = pabfd;
      pabfd = pabfd->next;

      execit_blank_section_t *psec = qabfd->sec;
      execit_blank_section_t *qsec = NULL;
      while (psec)
	{
	  qsec = psec;
	  psec = psec->next;

	  execit_blank_unit_t *p = qsec->unit;
	  execit_blank_unit_t *q = NULL;
	  size_t total_deleted_size = 0;
	  bfd_vma offset;
	  while (p)
	    {
	      q = p;
	      p = p->next;

	      offset = q->offset - total_deleted_size;
	      riscv_relax_delete_bytes (qabfd->abfd, qsec->sec, offset,
					q->size, info, NULL);
	      total_deleted_size += q->size;
	      free (q);
	    }
	  free (qsec);
	}
	free (qabfd);
    }
  execit.blank_list = NULL;
}

#if 0
static asection*
andes_table_jump_get_section (struct bfd_link_info *info)
{
  asection *sec = NULL;
  bfd *abfd;
  struct riscv_elf_link_hash_table *htab = riscv_elf_hash_table (info);

  if (htab->table_jump_htab->tablejump_sec != NULL)
    return htab->table_jump_htab->tablejump_sec;

  for (abfd = info->input_bfds; abfd != NULL; abfd = abfd->link.next)
    {
      sec = bfd_get_section_by_name (abfd, TABLE_JUMP_SEC_NAME);
      if (sec != NULL)
	break;
    }

  htab->table_jump_htab->tablejump_sec = sec;
  return sec;
}
#endif

/* Get section .exec.itable.  */

static asection*
andes_execit_get_itable_section (bfd *input_bfds)
{
  asection *sec = NULL;
  bfd *abfd;

  if (execit.itable_section != NULL)
    return execit.itable_section;

  for (abfd = input_bfds; abfd != NULL; abfd = abfd->link.next)
    {
      sec = bfd_get_section_by_name (abfd, EXECIT_SECTION);
      if (sec != NULL)
	break;
    }

  execit.itable_section = sec;
  return sec;
}

/* Get the contents of a section.  */

static int
riscv_get_section_contents (bfd *abfd, asection *sec,
			    bfd_byte **contents_p, bool cache)
{
  /* Get the section contents.  */
  if (elf_section_data (sec)->this_hdr.contents != NULL)
    *contents_p = elf_section_data (sec)->this_hdr.contents;
  else
    {
      if (!bfd_malloc_and_get_section (abfd, sec, contents_p))
	return false;
      if (cache)
	elf_section_data (sec)->this_hdr.contents = *contents_p;
    }

  return true;
}

/* Find a relocation of type specified by reloc_type
   of the same r_offset with reloc. If not found, return irelend.

   Note that relocations must be sorted by r_offset,
   we find the relocation from "reloc" backward untill relocs,
   or find it from "reloc" forward untill irelend.  */

static Elf_Internal_Rela *
find_relocs_at_address (Elf_Internal_Rela *reloc,
			Elf_Internal_Rela *relocs,
			Elf_Internal_Rela *irelend,
			enum elf_riscv_reloc_type reloc_type)
{
  Elf_Internal_Rela *rel_t;

  /* Find backward.  */
  for (rel_t = reloc;
       rel_t >= relocs && rel_t->r_offset == reloc->r_offset;
       rel_t--)
    if (ELFNN_R_TYPE (rel_t->r_info) == reloc_type)
      return rel_t;

  /* We didn't find it backward.
     Try find it forward.  */
  for (rel_t = reloc;
       rel_t < irelend && rel_t->r_offset == reloc->r_offset;
       rel_t++)
    if (ELFNN_R_TYPE (rel_t->r_info) == reloc_type)
      return rel_t;

  return irelend;
}

/* For target aligned, optimize is zero.
   For EXECIT, optimize is one.  */

static int
riscv_relocation_check (struct bfd_link_info *info,
			Elf_Internal_Rela **irel,
			Elf_Internal_Rela *irelend,
			asection *sec, bfd_vma *off,
			bfd_byte *contents, int optimize)
{
  /* to track R_RISCV_CALL pals  */
  static Elf_Internal_Rela *last_rel = NULL;
  static asection *last_sec = NULL;
  /* We use the highest 1 byte of result to record
     how many bytes location counter has to move.  */
  int result = 0;
  Elf_Internal_Rela *irel_save = NULL;
  int nested_execit_depth;
  bool execit_loop_aware;
  bool is_no_execit, is_inner_loop;
  struct riscv_elf_link_hash_table *table;
  andes_ld_options_t *andes;

  table = riscv_elf_hash_table (info);
  andes = &table->andes;
  execit_loop_aware = andes->execit_loop_aware;

  /* if last insn is tagged with R_RISCV_CALL(_PLT), skip the jarl  */
  while (last_rel)
    {
      int rtype = ELFNN_R_TYPE (last_rel->r_info);
      if (!(rtype == R_RISCV_CALL || rtype == R_RISCV_CALL_PLT))
	break;

      BFD_ASSERT (sec == last_sec);
      BFD_ASSERT (*off == (last_rel->r_offset + 4));
      BFD_ASSERT ((*irel == NULL) || (*irel == irelend)
		  || ((*irel)->r_offset > *off));
      last_rel = NULL;
      last_sec = NULL;
      result |= (4 << 24);
      result |= DATA_EXIST;

      break; /* once */
    }

  while ((*irel) != NULL && (*irel) < irelend && (*off) == (*irel)->r_offset)
    {
      switch (ELFNN_R_TYPE ((*irel)->r_info))
	{
	case R_RISCV_RELAX_REGION_BEGIN:
	  result = 0;
	  irel_save = *irel; /* in case multiple relocations.  */
	  /* to ignore code block.  */
	  is_no_execit = (*irel)->r_addend & R_RISCV_RELAX_REGION_NO_EXECIT_FLAG;
	  is_inner_loop = execit_loop_aware
			  && ((*irel)->r_addend & R_RISCV_RELAX_REGION_IMLOOP_FLAG);
	  nested_execit_depth = 0;
	  if (optimize /* for execit */
	      && (is_no_execit || is_inner_loop))
	    {
	      /* Check the region if is_inner_loop. if true and execit_loop_aware,
	         ignore the region till region end.  */
	      /* To save the status for in .no_execit_X region and
		 loop region to confirm the block can do EXECIT relaxation.  */
	      while ((*irel) && (*irel) < irelend && (is_no_execit || is_inner_loop))
		{
		  (*irel)++;
		  if (ELFNN_R_TYPE ((*irel)->r_info) == R_RISCV_RELAX_REGION_BEGIN)
		    { /* nested region detection.  */
		      bool next_is_no_execit, next_is_inner_loop;
		      nested_execit_depth++;
		      if (nested_execit_depth > 0)
			(*_bfd_error_handler)(_("%pB(%pA+0x%lx): Nested relax regions!"),
					      sec->owner, sec, (*irel)->r_offset);
		      /* since outter setting is still valid  */
		      next_is_no_execit = (*irel)->r_addend & R_RISCV_RELAX_REGION_NO_EXECIT_FLAG;
		      if (execit_loop_aware)
			next_is_inner_loop = (*irel)->r_addend & R_RISCV_RELAX_REGION_IMLOOP_FLAG;
		      if ((next_is_no_execit != is_no_execit) ||
			  ((execit_loop_aware) &&
			   (next_is_inner_loop != is_inner_loop)))
			(*_bfd_error_handler)(_("%pB(%pA+0x%lx): Conflict nested relax regions!"),
					      sec->owner, sec, (*irel)->r_offset);
		    }
		  else if (ELFNN_R_TYPE ((*irel)->r_info) == R_RISCV_RELAX_REGION_END)
		    { /* nested regions are parsed but ignored!  */
		      if (nested_execit_depth)
			nested_execit_depth--;
		      else
			{
			  bfd_vma end_off = (*irel)->r_offset;
			  if (end_off != irel_save->r_offset)
			    { /* rollback to the first relocation with the same end_off.  */
			      *irel = irel_save;
			      while (((*irel)->r_offset < end_off) && (*irel) < irelend)
				(*irel)++;
			      result |= RELAX_REGION_END;
			    }
			  else /* empty region, but might with other relocations.  */
			    *irel = irel_save + 1; /* skip region begin  */
			  break;
			}
		    }
		}

	      if ((*irel) >= irelend)
		*off = sec->size;
	      else
		*off = (*irel)->r_offset;
	      irel_save = NULL;
	    }
	  break;
	case R_RISCV_DATA:
	  /* Data in text section.  */
	  result |= ((*irel)->r_addend << 24);
	  result |= DATA_EXIST;
	  break;
	  /* Here we regard unsupported relocations as data relocations
	     and then skip them.   */
	  /* Since a pair of R_RISCV_ADD (R_RISCV_SET) and R_RISCV_SUB point
	     to the same address, we only need to skip them once.  */
	case R_RISCV_SUB6:
	case R_RISCV_SUB8:
	  result |= (1 << 24);
	  result |= DATA_EXIST;
	  break;
	case R_RISCV_SUB16:
	  result |= (2 << 24);
	  result |= DATA_EXIST;
	  break;
	case R_RISCV_RVC_LUI:
	case R_RISCV_RVC_JUMP:
	case R_RISCV_RVC_BRANCH:
	  if (!optimize)
	    irel_save = *irel;
	  else
	    {
	      result |= (2 << 24);
	      result |= DATA_EXIST;
	    }
	  break;
	case R_RISCV_32:
	case R_RISCV_SUB32:
	case R_RISCV_PCREL_LO12_I:
	case R_RISCV_PCREL_LO12_S:
	  result |= (4 << 24);
	  result |= DATA_EXIST;
	  break;
	case R_RISCV_PCREL_HI20:
	case R_RISCV_10_PCREL:
	case R_RISCV_BRANCH:
	  irel_save = *irel;
	  if (!optimize)
	    {
	      result |= (4 << 24);
	      result |= DATA_EXIST;
	    }
	  break;
	  /* These relocation is supported EXECIT relaxation currently.  */
	  /* We have to save the relocation for using later, since we have
	     to check there is any alignment in the same address.  */
	case R_RISCV_JAL:
	  irel_save = *irel;
	  if (!optimize)
	    {
	      result |= (4 << 24);
	      result |= DATA_EXIST;
	    }
	  break;
	case R_RISCV_HI20:
	case R_RISCV_LO12_I:
	case R_RISCV_LO12_S:
	case R_RISCV_GPREL_I:
	case R_RISCV_GPREL_S:
	case R_RISCV_LGP18S0:
	case R_RISCV_LGP17S1:
	case R_RISCV_LGP17S2:
	case R_RISCV_LGP17S3:
	case R_RISCV_SGP18S0:
	case R_RISCV_SGP17S1:
	case R_RISCV_SGP17S2:
	case R_RISCV_SGP17S3:
	  if (optimize)
	    irel_save = *irel;
	  else
	    {
	      result |= (4 << 24);
	      result |= DATA_EXIST;
	    }
	  break;
	case R_RISCV_64:
	case R_RISCV_SUB64:
	  result |= (8 << 24);
	  result |= DATA_EXIST;
	  break;
	case R_RISCV_CALL:
	case R_RISCV_CALL_PLT:
	  if (optimize)
	    {
	      irel_save = *irel;
	      last_rel = *irel;
	      last_sec = sec;
	    }
	  else
	    {
	      result |= (8 << 24);
	      result |= DATA_EXIST;
	    }
	  break;
	case R_RISCV_CALL_ICT:
	  /* exclude ICT call by now.  */
	  result |= (8 << 24);
	  result |= DATA_EXIST;
	  break;
	case R_RISCV_RELATIVE:
	case R_RISCV_COPY:
	case R_RISCV_JUMP_SLOT:
	case R_RISCV_TLS_DTPMOD32:
	case R_RISCV_TLS_DTPMOD64:
	case R_RISCV_TLS_DTPREL32:
	case R_RISCV_TLS_DTPREL64:
	case R_RISCV_TLS_TPREL32:
	case R_RISCV_TLS_TPREL64:
	  /* These relocations are used by dynamic linker. In general, we
	     should not see them here.  */
	  (*_bfd_error_handler)
	    (_("Linker: find dynamic relocation when doing relaxation\n"));
	  break;
	case R_RISCV_ANDES_TAG:
	  { /* */
	    andes_irelx_t *relx = (andes_irelx_t *) (*irel)->r_user;
	    int tag = relx->flags;
	    if (tag != R_RISCV_EXECIT_ITE)
	      irel_save = *irel; /* to skip dropped relocation (NONE).  */
	    /* EXECIT_ITE is tagged on exec.it (RVC) and would be skip.  */
	  }
	  break;
	default:
	  /* Relocation not supported.  */
	  if (ELFNN_R_TYPE ((*irel)->r_info) != R_RISCV_RELAX
	      && ELFNN_R_TYPE ((*irel)->r_info) != R_RISCV_NONE
	      && ELFNN_R_TYPE ((*irel)->r_info) != R_RISCV_RELAX_ENTRY
	      && ELFNN_R_TYPE ((*irel)->r_info) != R_RISCV_ADD8
	      && ELFNN_R_TYPE ((*irel)->r_info) != R_RISCV_ADD16
	      && ELFNN_R_TYPE ((*irel)->r_info) != R_RISCV_ADD32
	      && ELFNN_R_TYPE ((*irel)->r_info) != R_RISCV_ADD64
	      && ELFNN_R_TYPE ((*irel)->r_info) != R_RISCV_SET6
	      && ELFNN_R_TYPE ((*irel)->r_info) != R_RISCV_SET8
	      && ELFNN_R_TYPE ((*irel)->r_info) != R_RISCV_SET16
	      && ELFNN_R_TYPE ((*irel)->r_info) != R_RISCV_SET32
	      && ELFNN_R_TYPE ((*irel)->r_info) != R_RISCV_NO_RVC_REGION_BEGIN
	      && ELFNN_R_TYPE ((*irel)->r_info) != R_RISCV_NO_RVC_REGION_END
	      && ELFNN_R_TYPE ((*irel)->r_info) != R_RISCV_RELAX_REGION_END)
	    {
	      /* Since we already consider all relocations above, this won't
		 happen unless supporting new relocations. */
	      /* TODO: Maybe we should show warning message here.  */
	      result |= DATA_EXIST;
	      if ((*(contents + (*off)) & 0x3) != 0x3)
		/* 16-bit instruction.  */
		result |= (2 << 24);
	      else
		/* 32-bit instruction.  */
		result |= (4 << 24);
	      break;
	    }
	}
      if (result & RELAX_REGION_END)
	break;
      (*irel)++;
    }
  if (irel_save)
    {
      *irel = irel_save;
      result |= SYMBOL_RELOCATION;
    }
  return result;
}

#define MASK_IMM ENCODE_ITYPE_IMM (-1U)
#define MASK_RS1 (OP_MASK_RS1 << OP_SH_RS1)
#define MASK_RD (OP_MASK_RD << OP_SH_RD)
#define MASK_MAJOR_OP OP_MASK_OP
#define MATCH_OP_V (0x57)
#define MATCH_OP_P (0x77)
#define MATCH_OP_XDSP (0x7f)
#define MATCH_OP_AMO (0x2f)
#define MATCH_OP_LOAD_FP (0x07)
#define MATCH_OP_STORE_FP (0x27)
#define MASK_OP_XDSP_A  (0xfff0007f)
#define MATCH_OP_XDSP_A (0x80100073)

static bool
riscv_elf_execit_check_insn_available (uint32_t insn,
				       struct riscv_elf_link_hash_table *htab)
{
  andes_ld_options_t *andes = &htab->andes;
  /* For bug-11621, system call should not be replaced by exec.it.  */
  /* According to spec, SCALL and SBREAK have been renamed to
     ECALL and EBREAK. Their encoding and functionality are unchanged.  */
  /* Invalid insns: ecall, ebreak, ACE, ret.  */
  if ((insn & MASK_ECALL) == MATCH_ECALL
      || (insn) == MATCH_ADDI /* NOP (not c.nop)  */
      || (insn & MASK_EBREAK) == MATCH_EBREAK
      || (insn & 0x7f) == 0x7b /* ACE  */
      || ((insn & (MASK_JALR | MASK_RD | MASK_RS1 | MASK_IMM))
	  == (MATCH_JALR | (X_RA << OP_SH_RS1)))) /* ret  */
    return false;

  /* configurable sets  */
  uint32_t major = insn & MASK_MAJOR_OP;
  uint32_t width = (insn >> 12) & 0x7;
  if (!andes->execit_flags.rvv)
    { /* RVV is excluded.  */
      if (major == MATCH_OP_V)
	return false;
      else
	{
	  /* Zvamo is removed, TODO: review this  */
	  if ((major == MATCH_OP_AMO) && (width > 5))
	    return false;
	  /* partial FLS  */
	  if (((major == MATCH_OP_LOAD_FP) ||
	       (major == MATCH_OP_STORE_FP)) &&
	      ((width == 0) || (width > 4)))
	    return false;
	}
    }

  if (!andes->execit_flags.rvp)
    { /* RVP is excluded.  */
      if (major == MATCH_OP_P)
	return false;
    }

  if (!andes->execit_flags.fls)
    { /* Float Load/Store. is excluded.  */
      /* Standard scalar FP  */
      if (((major == MATCH_OP_LOAD_FP) ||
	   (major == MATCH_OP_STORE_FP)) &&
	  ((width > 0) && (width < 5)))
	return false;
    }

  if (!andes->execit_flags.xdsp)
    { /* Andes Xdsp is excluded.  */
      if ((major == MATCH_OP_XDSP) ||
          ((insn & MASK_OP_XDSP_A) == MATCH_OP_XDSP_A))
	return false;
    }

  /* others  */
  return true;
}

static int
list_iterate (list_entry_t **lst, void *obj,
	      list_iter_cb_t each, list_iter_cb_t final)
{
  list_entry_t *p, *q, *pp;
  int count = 0;

  p = *lst;
  q = NULL;
  while (p)
    {
      count++;

      /* p might be freed within following call.  */
      pp = p->next;
      if (each && each(lst, obj, p, q))
	break;

      q = p;
      p = pp;

    }

  if (final)
    final(lst, obj, p, q);

  return count;
}

static int
append_final_cb (list_entry_t **lst, list_entry_t *j,
		 list_entry_t *p, list_entry_t *q)
{
  if (q)
    q->next = j;
  else
    *lst = j;

  j->next = p;

  return 0;
}

#if 0 //KILLME
static int 
free_each_cb (void *l ATTRIBUTE_UNUSED, void *j ATTRIBUTE_UNUSED,
	      void *p ATTRIBUTE_UNUSED, execit_vma_t *q)
{
  if (q)
    free (q);
  return false; /* to the end  */
}
#endif

/* exec.it hi20 (lui/auipc) helpers  */
typedef struct hi20_context
{
  execit_irel_t *p, *q;
  int id;

} hi20_context_t;

#if 0 //KILLME
static void
andes_execit_estimate_lui (execit_hash_t *he, execit_vma_t **lst_pp)
{
  LIST_EACH (&he->irels, andes_execit_estimate_lui_each_cb);
  /* count itable entries are required  */
  collect_lui_vma_each_cb (NULL, NULL, NULL, NULL); /* reset cache  */
  LIST_EACH1 (&he->irels, collect_lui_vma_each_cb, lst_pp);
}
#endif

static int andes_execit_hi20_collect_one (hi20_context_t *ctx)
{
  BFD_ASSERT (ctx->p);
  const int entries = 1;
  execit_irel_t *p = ctx->p;
  execit_irel_t *q;
  int groups = 0;
  while (p) /* iter all items.  */
    { /* find each group.  */
      execit_itable_t *a = &p->ie;
      int count = 1; /* 1 for *p self.  */
      bool found;
      for (q = p->next; q; q = q->next)
	{
	  execit_itable_t *b = &q->ie;
	  found = false;
	  if (a->addend == b->addend)
	    {
	      if (a->h) /* global symbol */
		{
		  if (a->h == b->h)
		    found = true;
		}
	      else if (a->isym) /* local symbol */
		{
		  if (a->isym == b->isym)
		    found = true;
		}
	      else
		BFD_ASSERT (0);
	    }

	  if (found)
	    count++;
	  else
	    break;
	}

      /* record grade.  */
      if (count > (entries * 2))
	{ /* fill group info.  */
	  execit_irel_t *r = p;
	  int id = ctx->id++;
	  groups++;
	  while(r != q)
	    {
	      BFD_ASSERT (r->ie.est_count == 0 && r->ie.entries == 0);
	      r->ie.est_count = count;
	      r->ie.entries = entries;
	      r->ie.group_id = id;
	      r = r->next;
	    }
	}

      p = q;
    }

  return groups;
}

/* score:   count/entry
 * compare: countB/entryB > countA/entryA
 *          => countB*entryA - countA*entryB > 0
 */
#define AUIPC_COMPARE(cb,eb,ca,ea) ((cb*ea - ca*eb) > 0)

static int andes_execit_hi20_collect_multiple (hi20_context_t *ctx)
{
  BFD_ASSERT (ctx->p);
  bfd_signed_vma window = SZ_4K;
  execit_itable_t *a = &ctx->p->ie; /* data of node A. */
  execit_irel_t *p; /* node B. */
  execit_irel_t *pp; /* backword ref.  */
  ctx->q = ctx->p->next; /* if no new group found.  */
  int count = 1; /* 1 for *p self.  */
  int group_count = 0;
  bool found;
  for (p = ctx->p->next; p; pp = p, p = p->next)
    {
      execit_irel_t *q;
      execit_itable_t *b;
      int entries;
      /* count nodes within the window.  */
      b = &p->ie;
      found = (b->bias - a->bias) < window;
      if (found)
	{
	  count++;
	  if (p->next)
	    continue;
	  else
	    { /* for fall through.  */
	      found = false;
	      p = p->next;
	    }
	}

      /* reached a mismatched or the end.  */
      BFD_ASSERT (found == false);

      /* transaction: all new group members will have better score?  */
      entries = 1 + (window >> 12);
      if (count > (entries * 2))
	{
	  found = true;
	  q = ctx->p;
	  while(q != p) /* iter all nodes of the new gorup.  */
	    {
	      b = &q->ie;
	      if (q->ie.entries
		  && !AUIPC_COMPARE (count, entries, b->est_count, b->entries))
		{ /* no, stop.  */
		  found = false;
		  break;
		}
	      q = q->next;
	    }
	}

      /* update new group info.  */
      if (found)
	{
	  int id = ctx->id++;
	  q = ctx->p;
	  while(q != p)
	    {
	      q->ie.est_count = count;
	      q->ie.entries = entries;
	      q->ie.group_id = id;
	      q = q->next;
	    }
	  ctx->q = q;
	  group_count = count;
	}

      /* try larger window.  */
      if (found && p)
	{
	  p = pp; /* continue from p.  */
	  window += SZ_4K;
	  continue;
	}

      break; /* try next node by the caller.  */
    }

  return group_count;
}

static void
andes_execit_sort_irel_list (execit_irel_t **list)
{
  execit_irel_t *p, *q, *r, *s, *t;
  p = *list;
  if (p == NULL)
    return;
  q = p;
  p = p->next;
  q->next = NULL;
  while (p)
    { /* find the node, t, to insert.  */
      s = p->next;
      r = q;
      t = NULL;
      while (r)
	{
	  if (r->ie.bias > p->ie.bias)
	    break;
	  t = r;
	  r = r->next;
	}
      if (t == NULL)
	{
	  p->next = q;
	  q = p;
	}
      else
	{
	  p->next = t->next;
	  t->next = p;
	}
      p = s;
    }

  *list = q;
}

static void
andes_execit_estimate_hi20 (execit_hash_t *he)
{
  static int group_id = 1; /* 0 for non-grouped.  */
  hi20_context_t ctx = {.id = group_id};
  int type;

  BFD_ASSERT (he);
  if (he == NULL)
    return;

  type = ELFNN_R_TYPE (he->ie.irel_copy.r_info);
  BFD_ASSERT (type == R_RISCV_HI20
	      || type == R_RISCV_CALL || type == R_RISCV_PCREL_HI20);

  /* grade each element of the list.  */
  andes_execit_sort_irel_list (&he->irels);
  ctx.p = he->irels;

  /* LUIs with the same target can be treated as the same instructions.
   * but AUIPCs cannot, as the relative offset are not the same.
   */
  if (type == R_RISCV_HI20)
    {
      andes_execit_hi20_collect_one (&ctx);
      BFD_ASSERT (ctx.p == he->irels);
    }

  while (ctx.p)
    {
      andes_execit_hi20_collect_multiple (&ctx);
      ctx.p = ctx.q;
    }

  group_id = ctx.id;
}

/*  EXECIT rank list helpers  */

static int
rank_each_cb (void *l ATTRIBUTE_UNUSED, execit_rank_t *j, execit_rank_t *p,
	      void *q ATTRIBUTE_UNUSED)
{
  if (!p)
    return -1;

  if (j->grade != p->grade)
    return j->grade > p->grade;
  else if (j->he->ie.fixed != p->he->ie.fixed)
    return j->he->ie.fixed < p->he->ie.fixed; /* smaller op/reg first */

  /* earliy id first  */
  return (j->he->id < p->he->id);
}

/* Replace with exec.it instruction.  */

static bool
andes_execit_push_insn (execit_context_t *ctx, execit_hash_t* h)
{
  uint16_t insn16;
  execit_itable_t *e = andes_execit_itable_lookup (ctx, h);
  if (e == NULL)
    return (h->type == ET_RK_TYPE_HI20) ? true : false;

  /* replace code.  */
  insn16 = execit.execit_op | ((execit.execit_op == EXECIT_INSN)
	     ? ENCODE_RVC_EXECIT_IMM (e->itable_index << 2)
	     : ENCODE_RVC_NEXECIT_IMM (e->itable_index << 2));
  bfd_put_16 (ctx->abfd, insn16, ctx->contents + ctx->off);

  if (!execit_push_blank (ctx, 2, 2))
    return false;

  /* NOT necessary the one in hash  */
  if (ctx->irel && !andes_execit_mark_irel (ctx->irel, e->itable_index))
    return false;

  return true;
}

#if 0 //KILLME!
static int 
andes_execit_estimate_lui_each_cb(void *l ATTRIBUTE_UNUSED,
				  void *j ATTRIBUTE_UNUSED,
				  execit_irel_t *p,
				  void *q ATTRIBUTE_UNUSED)
{
  bfd_vma addr, hi20;
  BFD_ASSERT (p);
  addr = p->ie.relocation;
  if (ARCH_SIZE > 32)
    BFD_ASSERT (VALID_UTYPE_IMM (RISCV_CONST_HIGH_PART (addr)));
  hi20 = ENCODE_UTYPE_IMM (RISCV_CONST_HIGH_PART (addr));
  p->ie.relocation = hi20;
  return false; /* to iter to the end  */
}

static int 
collect_lui_vma_each_cb(void *l, void *j_pp, execit_irel_t *p, void *q ATTRIBUTE_UNUSED)
{
  static bfd_vma last = -1u;

  if (l == NULL) /* reset cache  */
    {
      last = -1u;
      return last;
    }

  BFD_ASSERT (p);

  if (p->ie.relocation != last)
    {
      execit_vma_t e;
      last = p->ie.relocation; /* chache for speed  */

      /* reserve one more entry in case crossing range.
       * NOT doing so when determining final relocations. 
       */
      if (execit.is_determining_auipc
	  || (p->ie.relocation > SIZE_4K))
	{
	  e.vma = p->ie.relocation - SIZE_4K;
	  LIST_ITER(j_pp, &e, insert_vma_each_cb, insert_vma_final_cb);
	}
      e.vma = p->ie.relocation;
      LIST_ITER(j_pp, &e, insert_vma_each_cb, insert_vma_final_cb);
    }

  return false; /* to iter to the end  */
}

static int 
insert_vma_each_cb(void *l ATTRIBUTE_UNUSED, execit_vma_t *j, execit_vma_t *p, void *q ATTRIBUTE_UNUSED)
{
  BFD_ASSERT (p);
  return (j->vma <= p->vma); /* ascending set */
}

static int 
insert_vma_final_cb(void *l, execit_vma_t *j, execit_vma_t *p, void *q)
{
  if (p && (p->vma == j->vma))
    return true;

  execit_vma_t *e = bfd_zmalloc(sizeof(execit_vma_t));
  e->vma = j->vma;
  return append_final_cb(l, (void*) e, (void*) p, q);
}
#endif

/*  Lookup EXECIT times entry.  */
static execit_itable_t *
andes_execit_itable_lookup (execit_context_t *ctx,
			    execit_hash_t* h)
{
  execit_itable_t *a = &ctx->ie;
  execit_itable_t *b = &h->ie;

  /* skip relocation check, as it might have been changed.  */

  while (true)
    {
      if (h->type == ET_RK_TYPE_HI20)
	{
	  execit_irel_t *p = h->irels;
	  while (p)
	    {
	      if (p->ie.irel == a->irel)
		return p->is_chosen ? &p->ie : NULL;
	      p = p->next;
	    }
	  break;
	}

      if (a->fixed != b->fixed)
	break;
      if ((a->irel == NULL) ^ (b->irel == NULL)) /* both NULL or SOME */
        break;
      if (a->irel)
	{ /* skip future check for some relocs  */
	  int rta = ELFNN_R_TYPE (a->irel_copy.r_info);
	  int rtb = ELFNN_R_TYPE (b->irel_copy.r_info);
	  if ((rta == R_RISCV_HI20 && rtb == R_RISCV_HI20)
	      || ((rta == R_RISCV_CALL || rta == R_RISCV_PCREL_HI20)
		  && (rtb == R_RISCV_CALL || rtb == R_RISCV_PCREL_HI20)))
	    return b;
	  if ((ELFNN_R_SYM (a->irel_copy.r_info)
	       != ELFNN_R_SYM (b->irel_copy.r_info))
	      && a->isec != b->isec)
	    break;
	}
      return b; /* Pass  */
    }

  BFD_ASSERT (0);
  return NULL;
}

static bool
execit_push_blank (execit_context_t *ctx, bfd_vma delta, bfd_vma size)
{
  /* TODO: abfd can be found from sec->owner  */
  execit_blank_abfd_t *pabfd = execit_lookup_blank_abfd (ctx);
  if (pabfd == NULL)
    return false;

  execit_blank_section_t *psec = execit_lookup_blank_section (ctx, pabfd);
  if (psec == NULL)
    return false;

  execit_blank_unit_t *p, *q;
  bfd_vma offset = ctx->off + delta;

  /* TODO: merge overlapped units  */
  p = q = psec->unit;
  while (p)
    {
      if ((p->offset == offset) &&
	  (p->size == size))
	break;
      q = p;
      p = p->next;
    }

  if (p == NULL)
    {
      p = bfd_zmalloc(sizeof (execit_blank_unit_t));
      if (q)
	q->next = p;
      else
	psec->unit = p;
      p->offset = offset;
      p->size = size;
    }

  return true;
}

static bool
andes_execit_mark_irel (Elf_Internal_Rela *irel, int index)
{
  int rtype = ELFNN_R_TYPE (irel->r_info);
  if (rtype == R_RISCV_HI20 || rtype == R_RISCV_CALL
      || rtype == R_RISCV_PCREL_HI20 || rtype == R_RISCV_JAL)
    {
      andes_irelx_t *irel_ext =
	andes_extend_irel(irel, R_RISCV_EXECIT_ITE, &execit.irelx_list);
      irel_ext->tag = index;
    }
  else
    { /* replaced insns has no neeed to relocate  */
      irel->r_info = ELFNN_R_INFO (ELFNN_R_SYM (irel->r_info), R_RISCV_NONE);
    }
  return true;
}

/* TODO: free allocated memory at a proper timing  */
/* rel {offset, info*, addend, user*}
 *                             /
 *                            relx {rel_copy, tag, flags}
 */
static andes_irelx_t*
andes_extend_irel (Elf_Internal_Rela *irel, int subtype,
		   andes_irelx_t **list)
{  /* new one  */
  andes_irelx_t *one = bfd_zmalloc (sizeof (andes_irelx_t));
  BFD_ASSERT (one);
  /* init  */
  one->saved_irel = *irel;
  one->saved_irel.r_user = (bfd_vma) irel; /* keep origin reference.  */
  one->flags = subtype;
  irel->r_user = (bfd_vma) one;
  irel->r_info = ELFNN_R_INFO (ELFNN_R_SYM (irel->r_info), R_RISCV_ANDES_TAG);
  /* link reversely  */
  one->next = *list;
  (*list) = one;
  return one;
}

static execit_blank_abfd_t*
execit_lookup_blank_abfd (execit_context_t *ctx)
{
  execit_blank_abfd_t *p, *q;
  p = q = execit.blank_list;

  while (p)
    {
      if (p->abfd == ctx->abfd)
	break;
      q = p;
      p = p->next;
    }

  if (p == NULL)
    {
      p = bfd_zmalloc(sizeof (execit_blank_abfd_t));
      if (q)
	q->next = p;
      else
	execit.blank_list = p;
      p->abfd = ctx->abfd;
    }

  return p;
}

static execit_blank_section_t*
execit_lookup_blank_section (execit_context_t *ctx, execit_blank_abfd_t *blank_abfd)
{
  execit_blank_section_t *p, *q;
  p = q = blank_abfd->sec;

  while (p)
    {
      if (p->sec == ctx->sec)
	break;
      q = p;
      p = p->next;
    }

  if (p == NULL)
    {
      p = bfd_zmalloc(sizeof (execit_blank_section_t));
      if (q)
	q->next = p;
      else
	blank_abfd->sec = p;
      p->sec = ctx->sec;
    }

  return p;
}

/* Set the _ITB_BASE_, and point it to .exec.itable.  */

static bool
execit_set_itb_base (struct bfd_link_info *link_info)
{
  asection *sec;
  bfd *output_bfd = NULL;
  struct bfd_link_hash_entry *bh = NULL;

  if (execit.is_itb_base_set == 1 || link_info->type == type_relocatable)
    return true;

  execit.is_itb_base_set = 1;

  sec = andes_execit_get_itable_section (link_info->input_bfds);
  if (sec != NULL)
    output_bfd = sec->output_section->owner;

  if (output_bfd == NULL)
    {
      output_bfd = link_info->output_bfd;
      if (output_bfd->sections == NULL)
	return true;
      else
	sec = bfd_abs_section_ptr;
    }

  /* Do not define _ITB_BASE_ if it is not used.
     And remain user to set it if needed.  */

  bh = bfd_link_hash_lookup (link_info->hash, "_ITB_BASE_",
			     false, false, true);
  if (!bh)
    return true;

  return (_bfd_generic_link_add_one_symbol
	  (link_info, output_bfd, "_ITB_BASE_", BSF_GLOBAL | BSF_WEAK | BSF_SECTION_SYM_USED,
	   sec, 0, (const char *) NULL, false,
	   get_elf_backend_data (output_bfd)->collect, &bh));
}

/* Adjust relocations in the .exec.itable, and then
   export it if needed.  */

static void
andes_execit_relocate_itable (struct bfd_link_info *info)
{
  bfd *abfd;
  asection *itable_sec = NULL;
  execit_item_t *itable = execit.itable_array;
  uint32_t insn, insn_with_reg;
  bfd_byte *contents = NULL;
  int size = 0;
  Elf_Internal_Rela rel_backup;
  struct riscv_elf_link_hash_table *table;
  andes_ld_options_t *andes;
  bfd_vma gp;

  /* Only need to be done once.  */
  if (execit.relocate_itable_done)
    return;
  execit.relocate_itable_done = true;

  table = riscv_elf_hash_table (info);
  andes = &table->andes;

  FILE *export_file = NULL;
  if (andes->execit_export_file != NULL)
    {
      export_file = fopen (andes->execit_export_file, "wb");
      if (export_file == NULL)
	{
	  (*_bfd_error_handler)
	    (_("Warning: cannot open the exported .exec.itable %s."),
	     andes->execit_export_file);
	}
    }

  /* TODO: Maybe we should close the export file here, too.  */
  if (andes->execit_import_file && !andes->update_execit_table)
    return;

  itable_sec = andes_execit_get_itable_section (info->input_bfds);
  if (itable_sec == NULL)
    {
      (*_bfd_error_handler) (_("ld: error cannot find .exec.itable section.\n"));
      return;
    }

  gp = riscv_global_pointer_value (info);
  if (itable_sec->size == 0)
    return;

  abfd = itable_sec->owner;
  /* TODO: hacky! try to do with SEC_LINKER_CREATED  */
  flagword keep = bfd_section_flags (itable_sec);
  itable_sec->flags |= SEC_LINKER_CREATED;
  riscv_get_section_contents (abfd, itable_sec, &contents, true);
  itable_sec->flags = keep;
  if (contents == NULL)
   return;

  /* Relocate instruction.  */
  /* TODO: mark relocated entries to avoid redundancy calculations  */
  for (int index = 0; index < execit.next_itable_index; ++index)
    {
      execit_item_t *item = &itable[index];
      execit_hash_t *he = item->he;
      bfd_vma relocation;
      int rtype = ELFNN_R_TYPE (he->ie.irel_copy.r_info);

      BFD_ASSERT (he->is_chosen);

      if (item->is_relocated
	  && !(rtype == R_RISCV_HI20
	       || rtype == R_RISCV_PCREL_HI20
	       || rtype == R_RISCV_CALL))
	continue;

      insn = he->ie.insn;
      if (he->ie.irel)
	{
	  rel_backup = he->ie.irel_copy;
	  insn_with_reg = he->ie.fixed;
	  rtype = ELFNN_R_TYPE (rel_backup.r_info); /* reuse variable  */
	  if (rtype == R_RISCV_JAL)
	    {
	      /* TODO: check est/ref counts for JAL window crossing.  */
	      bfd_vma insn_pc = sec_addr(he->ie.sec) + he->ie.irel->r_offset;
	      relocation = riscv_elf_execit_reloc_insn (&he->ie, info);
	      he->ie.relocation = relocation; /* keep for later sanity check  */
	      BFD_ASSERT ((relocation & 0xffe00000) == (insn_pc & 0xffe00000));
	      relocation &= 0x001fffffu;
	      insn = insn_with_reg
		| riscv_elf_encode_relocation (abfd, &rel_backup, relocation);
	      bfd_put_32 (abfd, insn, contents + (he->ie.itable_index) * 4);
	    }
	  else if (rtype == R_RISCV_LO12_I || rtype == R_RISCV_LO12_S)
	    {
	      relocation = riscv_elf_execit_reloc_insn (&he->ie, info);
	      insn = insn_with_reg
		| riscv_elf_encode_relocation (abfd, &rel_backup, relocation);
	      bfd_put_32 (abfd, insn, contents + (he->ie.itable_index) * 4);
	    }
	  else if (rtype == R_RISCV_GPREL_I)
	    {
	      insn = insn_with_reg & ~(OP_MASK_RS1 << OP_SH_RS1);
	      relocation = riscv_elf_execit_reloc_insn (&he->ie, info);
	      if (VALID_ITYPE_IMM (relocation))
		{ /* x0 REL */
		}
	      else if (VALID_ITYPE_IMM (relocation - gp))
		{ /* gp REL */
		  relocation -= gp;
		  insn |= X_GP << OP_SH_RS1;
		}
	      else BFD_ASSERT (0);
	      insn |= riscv_elf_encode_relocation (abfd, &rel_backup, relocation);
	      bfd_put_32 (abfd, insn, contents + (he->ie.itable_index) * 4);
	    }
	  else if (rtype == R_RISCV_GPREL_S)
	    {
	      insn = insn_with_reg & ~(OP_MASK_RS1 << OP_SH_RS1);
	      relocation = riscv_elf_execit_reloc_insn (&he->ie, info);
	      if (VALID_STYPE_IMM (relocation))
		{ /* x0 REL */
		}
	      else if (VALID_STYPE_IMM (relocation - gp))
		{ /* gp REL */
		  relocation -= gp;
		  insn |= X_GP << OP_SH_RS1;
		}
	      else BFD_ASSERT (0);
	      insn |= riscv_elf_encode_relocation (abfd, &rel_backup, relocation);
	      bfd_put_32 (abfd, insn, contents + (he->ie.itable_index) * 4);
	    }
	  else if (rtype >= R_RISCV_LGP18S0 && rtype <= R_RISCV_SGP17S3)
	    {
	      relocation = riscv_elf_execit_reloc_insn (&he->ie, info) - gp;
	      insn = insn_with_reg
		| riscv_elf_encode_relocation (abfd, &rel_backup, relocation);
	      bfd_put_32 (abfd, insn, contents + (he->ie.itable_index) * 4);
	    }
	  else if ((rtype == R_RISCV_HI20)
		   || (rtype == R_RISCV_CALL || rtype == R_RISCV_PCREL_HI20))
	    { /* estimate HI20 (auipc/lui) relocation again (final).  */
	      BFD_ASSERT (he->type == ET_RK_TYPE_HI20 && item->grp_hd);
	      execit_itable_t *ie = item->inf_ie ? item->inf_ie : &item->grp_hd->ie;
	      int entries = ie->entries;
	      for (int i = 0; i < entries; ++i)
		{
		  if (item->is_final == false)
		    break;
		  relocation = item->relocation;
		  insn = insn_with_reg
		    | riscv_elf_encode_relocation (abfd, &rel_backup, relocation);
		  bfd_put_32 (abfd, insn, contents + (ie->itable_index << 2));
		  item->is_relocated = true;
		  if (item->next == 0)
		    break;
		  item = &itable[item->next];
		  he = item->he;
		  ie = item->inf_ie;
		}
	    }
	  else if (rtype == R_RISCV_ANDES_TAG)
	    {
	      andes_irelx_t *relx = (andes_irelx_t *) rel_backup.r_user;
	      int tag = relx->flags;
	      if (tag == TAG_GPREL_SUBTYPE_FLX || tag == TAG_GPREL_SUBTYPE_FSX)
		{
		  int type = ELFNN_R_TYPE (relx->saved_irel.r_info);
		  relocation = riscv_elf_execit_reloc_insn (&he->ie, info) - gp;
		  if (type == R_RISCV_LO12_I || type == R_RISCV_PCREL_LO12_I)
		    insn = insn_with_reg | ENCODE_ITYPE_IMM(relocation);
		  else if (type == R_RISCV_LO12_S || type == R_RISCV_PCREL_LO12_S)
		    insn = insn_with_reg | ENCODE_STYPE_IMM(relocation);
		  else
		    BFD_ASSERT (0);
		  bfd_put_32 (abfd, insn, contents + (he->ie.itable_index) * 4);
		}
	    }

	  if (!(rtype == R_RISCV_HI20
		|| rtype == R_RISCV_CALL
		|| rtype == R_RISCV_PCREL_HI20))
	    item->is_final = item->is_relocated = true;
	}
      else
	{
	  /* No need to do relocation for insn without relocation.*/
	}
      size += 4;
    }

  size = execit.next_itable_index << 2;

  if (!andes->update_execit_table)
    size = itable_sec->size;

  if (export_file != NULL)
    {
      fwrite (contents, sizeof (bfd_byte), size, export_file);
      fclose (export_file);
    }
}

/* Relocate the entries in .exec.itable.  */

static bfd_vma
riscv_elf_execit_reloc_insn (execit_itable_t *ptr,
			     struct bfd_link_info *info)
{
  Elf_Internal_Sym *isym = NULL;
  bfd_vma relocation = -1;
  struct elf_link_hash_entry *h;
  if (ptr->h)
    { /* global symbol.  */
      h = ptr->h;
      if ((h->root.type == bfd_link_hash_defined
	   || h->root.type == bfd_link_hash_defweak)
	  && h->root.u.def.section != NULL
	  && h->root.u.def.section->output_section != NULL)
	{

	  relocation = h->root.u.def.value +
	    h->root.u.def.section->output_section->vma +
	    h->root.u.def.section->output_offset;
	  relocation += ptr->irel_copy.r_addend;
	}
      else
	relocation = 0;
    }
  else if (ptr->isym)
    {
      /* Local symbol.  */
      bfd *abfd = ptr->isec->owner;
      Elf_Internal_Rela irel = ptr->irel_copy;
      asection *sec = ptr->isec;
      bfd_vma value_backup;

      if (!riscv_get_local_syms (abfd, sec, &isym))
	return false;

      isym = isym + ELFNN_R_SYM (irel.r_info);
      BFD_ASSERT (isym == ptr->isym);

      value_backup = isym->st_value;
      /* According to elf_link_input_bfd, linker had called
	 _bfd_merged_section_offset to adjust the address of
	 symbols in the SEC_MERGE sections, and get the merged
	 sections. Since ptr->isec is the section before merging
	 (linker hasn't found the correct merge section in relax
	 time), we must call _bfd_merged_section_offset to find
	 the correct symbol address here, too.  */
      /* Note that we have to store the local symbol value for the
	 last relaxation before, since the symbol value here had
	 been modified in elf_link_input_bfd.  */
      if (sec->sec_info_type == SEC_INFO_TYPE_MERGE
	  && ELF_ST_TYPE (isym->st_info) != STT_SECTION)
	isym->st_value =
	  _bfd_merged_section_offset (info->output_bfd, &sec,
				      elf_section_data (sec)->sec_info,
				      ptr->isym_copy.st_value); /* copied one  */

      relocation = _bfd_elf_rela_local_sym (info->output_bfd, isym,
					    &sec,
					    &irel);
      relocation += irel.r_addend;

      /* Restore origin value.  */
      isym->st_value = value_backup;
    }

  return relocation;
}

/* Encode relocation into Imm field.  */

static bfd_vma
riscv_elf_encode_relocation (bfd *abfd, Elf_Internal_Rela *irel,
			     bfd_vma relocation)
{
  reloc_howto_type *howto = NULL;

  if (irel == NULL
      || (ELFNN_R_TYPE (irel->r_info) >= number_of_howto_table))
    return 0;

  howto = riscv_elf_rtype_to_howto (abfd, ELFNN_R_TYPE (irel->r_info));
  switch (ELFNN_R_TYPE (irel->r_info))
    {
    case R_RISCV_HI20:
    case R_RISCV_CALL:
      if (ARCH_SIZE > 32 && !VALID_UTYPE_IMM (RISCV_CONST_HIGH_PART (relocation)))
	return 0;
      relocation = ENCODE_UTYPE_IMM (RISCV_CONST_HIGH_PART (relocation));
      break;
    case R_RISCV_PCREL_HI20:
      {
	bfd_vma a = RISCV_CONST_HIGH_PART (relocation);
	bfd_vma d = HI_32B (a);
	BFD_ASSERT (d == 0 || d == MASK_LSB_32);
	relocation = ENCODE_UTYPE_IMM (a);
      }
      break;
    case R_RISCV_LO12_I:
    case R_RISCV_PCREL_LO12_I:
    case R_RISCV_GPREL_I:
      relocation = ENCODE_ITYPE_IMM (relocation);
      break;
    case R_RISCV_LO12_S:
    case R_RISCV_PCREL_LO12_S:
    case R_RISCV_GPREL_S:
      relocation = ENCODE_STYPE_IMM (relocation);
      break;
    case R_RISCV_JAL:
      relocation = ENCODE_UJTYPE_IMM (relocation);
      break;
    case R_RISCV_LGP18S0:
      relocation = ENCODE_GPTYPE_LB_IMM (relocation);
      break;
    case R_RISCV_LGP17S1:
      relocation = ENCODE_GPTYPE_LH_IMM (relocation);
      break;
    case R_RISCV_LGP17S2:
      relocation = ENCODE_GPTYPE_LW_IMM (relocation);
      break;
    case R_RISCV_LGP17S3:
      relocation= ENCODE_GPTYPE_LD_IMM (relocation);
      break;
    case R_RISCV_SGP18S0:
      relocation = ENCODE_GPTYPE_SB_IMM (relocation);
      break;
    case R_RISCV_SGP17S1:
      relocation = ENCODE_GPTYPE_SH_IMM (relocation);
      break;
    case R_RISCV_SGP17S2:
      relocation = ENCODE_GPTYPE_SW_IMM (relocation);
      break;
    case R_RISCV_SGP17S3:
      relocation = ENCODE_GPTYPE_SD_IMM (relocation);
      break;
    default:
      return 0;
    }

  return (relocation & howto->dst_mask);
}

/* ROM Patch with Indirect Call Table (ICT).  */

/* Indirect call hash function.  */

static struct bfd_hash_entry *
riscv_elf_ict_hash_newfunc (struct bfd_hash_entry *entry,
			    struct bfd_hash_table *table,
			    const char *string ATTRIBUTE_UNUSED)
{
  const size_t sz_entry = sizeof (andes_ict_hash_t);
  const size_t sz_head = sizeof (struct bfd_hash_entry);
  const size_t sz_body = sz_entry - sz_head;

  /* Allocate the structure if it has not already been allocated by a
     subclass.  */
  if (entry == NULL)
    {
      entry = (void *)
	bfd_hash_allocate (table, sz_entry);

      if (entry == NULL)
	return entry;
    }

  memset ((void *) entry + sz_head, 0, sz_body);

  return entry;
}

/* Initialize indirect call hash table.  */

static void
riscv_elf_ict_init (void)
{
  if (!bfd_hash_table_init_n (&nds_ict_sta.indirect_call_table, riscv_elf_ict_hash_newfunc,
			      sizeof (andes_ict_hash_t),
			      1023))
    (*_bfd_error_handler)
      (_("Linker: cannot init indirect call hash table.\n"));
  return;
}

/* non-invoked ict symbols are not collected in ict list.
   complement ict table for thest symbols.
*/
static void
andes_complement_ict_symbols (struct bfd_link_info *info)
{
  bfd *abfd;
  asection *sec;

  /* find ict table section.  */
  for (abfd = info->input_bfds; abfd != NULL; abfd = abfd->link.next)
    {
      if (abfd->link.next == NULL)
	sec = bfd_get_section_by_name (abfd, ANDES_ICT_SECTION);
    }
  if (sec == NULL)
    return;

  /* check if any ict entry patched.  */
  andes_ict_entry_t *s = nds_ict_sta.list_head;
  while (s)
    {
      struct elf_link_hash_entry *h;
      h = elf_link_hash_lookup (elf_hash_table (info),
				s->name, false, false, false);
      BFD_ASSERT (h);
      if (s->h)
	{
	  BFD_ASSERT (s->h == h
		      && riscv_elf_hash_entry (h)->indirect_call == true);
	}
      else
	{
	  s->h = h;
	  riscv_elf_hash_entry (h)->indirect_call = true;		
	}

      s = s->next;
    }
}

/* any ict table entry should be with a symbol `h',
   if the symbol is from linker script, then the `vma' is the VMA of symbol.
   otherwise the VMA shall be resolved from symbol `h'.  */

static void
andes_relocate_ict_table (struct bfd_link_info *info, bfd *output_bfd)
{
  bfd *input_bfd;
  asection *sec;
  bfd_byte *contents = NULL;
  struct elf_link_hash_entry *h;
  struct bfd_link_hash_entry *ict_sym;
  bfd_vma relocation, ict_base;
  bfd_vma insn;
  unsigned int ict_entry_size;
  andes_ict_entry_t *head;

  /* get ict table section.  */
  for (input_bfd = info->input_bfds;
       input_bfd != NULL;
       input_bfd = input_bfd->link.next)
    {
      sec = bfd_get_section_by_name (input_bfd, ANDES_ICT_SECTION);
      if (sec != NULL)
        break;
    }

  if (sec == NULL
      || !riscv_get_section_contents (sec->owner, sec, &contents, true))
    {
      (*_bfd_error_handler)
	(_("Linker: Can not find .nds.ict table or it's contents.\n"));
      return;
    }

  ict_sym = bfd_link_hash_lookup (info->hash, "_INDIRECT_CALL_TABLE_BASE_",
				  false, false, false);
  ict_base = ict_sym->u.def.value + sec_addr (ict_sym->u.def.section);

  if (ict_model == 1 || ict_model == 2)
    ict_entry_size = 8;
  else
    ict_entry_size = 4;

  head = nds_ict_sta.list_head;
  while (head)
    {
      int index, ict_table_reloc = R_RISCV_NONE;

      h = head->h;
      index = head->index;
      if (((h->root.type == bfd_link_hash_defined
	    || h->root.type == bfd_link_hash_defweak)
	   && h->root.u.def.section != NULL
	   && h->root.u.def.section->output_section != NULL)
	  || (h->root.type == bfd_link_hash_undefined))
	{
	  bfd_vma secaddr = 0;
	  if (h->root.u.def.section)
	    secaddr = sec_addr (h->root.u.def.section);

	  if (h->root.ldscript_def == 1 && !(head->flags & ICT_FLG_VMA))
	    {
	      h->root.u.def.value = head->vma - secaddr;
	      head->flags |= ICT_FLG_VMA;
	    }

	  relocation = h->root.u.def.value;
	  if (h->root.type != bfd_link_hash_undefined)
	    relocation += secaddr;

	  if (ict_model == 0)
	    {
	      /* Tiny model: jal ra, 0x0.  */
	      bfd_put_32 (output_bfd, RISCV_JTYPE (JAL, X_T1, 0x0),
			  contents + (index * ict_entry_size));
	      ict_table_reloc = R_RISCV_JAL;

	      /* PC is the entry of ICT table.  */
	      relocation -= ict_base + (index * ict_entry_size);
	      if (!VALID_JTYPE_IMM (relocation))
		{
		  (*_bfd_error_handler)
		    (_("Linker: relocate ICT table failed with tiny model.\n"));
		  return;
		}
	      relocation = ENCODE_JTYPE_IMM (relocation);
	    }
	  else if (ict_model == 1)
	    {
	      /* Small model: tail t1, 0x0.  */
	      bfd_put_32 (output_bfd, RISCV_UTYPE (AUIPC, X_T1, 0x0),
			  contents + (index * ict_entry_size));
	      bfd_put_32 (output_bfd, RISCV_ITYPE (JALR, X_T1, X_T1, 0),
			  contents + (index * ict_entry_size) + 4);
	      ict_table_reloc = R_RISCV_CALL;

	      /* PC is the entry of ICT table.  */
	      relocation -= ict_base + (index * ict_entry_size);
	      if (ARCH_SIZE > 32
		  && !VALID_UTYPE_IMM (RISCV_CONST_HIGH_PART (relocation)))
		{
		  (*_bfd_error_handler)
		    (_("Linker: relocate ICT table failed with small model.\n"));
		  return;
		}
	      relocation =
		ENCODE_UTYPE_IMM (RISCV_CONST_HIGH_PART (relocation))
		| (ENCODE_ITYPE_IMM (relocation) << 32);
	    }
	  else if (ict_model == 2)
	    {
	      /* Large model: .dword 0x0.  */
	      ict_table_reloc =  R_RISCV_64;
	    }
	  else
	    {
	      (*_bfd_error_handler)
		(_("Linker: Unknown ICT model.\n"));
	      return;
	    }

	  BFD_ASSERT (ict_table_reloc != R_RISCV_NONE);
	  reloc_howto_type *howto =
	    riscv_elf_rtype_to_howto (input_bfd, ict_table_reloc);
	  insn = bfd_get (howto->bitsize, output_bfd,
			  contents + (index * ict_entry_size));
	  insn = (insn & ~howto->dst_mask)
	    | (relocation & howto->dst_mask);
	  bfd_put (howto->bitsize, output_bfd, insn,
		   contents + (index * ict_entry_size));
	}
      else
	{
	  /* Should we allow the case that the ict symbol is undefined?  */
	  const char *name = h ? h->root.root.string : NULL;
	  (*_bfd_error_handler)
	    (_("Linker: Undefined ICT symbol `%s'!\n"), name);
	  BFD_ASSERT (0);
	}

      head = head->next;
    }
}

/* End of ROM Patch with Indirect Call Table (ICT).  */

/* Sort relocation by r_offset.
   We didn't use qsort () in stdlib, because quick-sort is not a stable sorting
   algorithm.  Relocations at the same r_offset must keep their order.

   Currently, this function implements insertion-sort.

   FIXME: If we already sort them in assembler, why bother sort them
   here again?  */

static void
riscv_insertion_sort (void *base, size_t nmemb, size_t size,
		      int (*compar) (const void *lhs, const void *rhs))
{
  char *ptr = (char *) base;
  int i, j;
  char *tmp = malloc (size);

  /* If i is less than j, i is inserted before j.

     |---- j ----- i --------------|
     \	      / \		  /
     sorted		unsorted
  */

  for (i = 1; i < (int) nmemb; i++)
    {
      for (j = (i - 1); j >= 0; j--)
	if (compar (ptr + i * size, ptr + j * size) >= 0)
	  break;
      j++;

      if (i == j)
	continue; /* i is in order.  */

      memcpy (tmp, ptr + i * size, size);
      memmove (ptr + (j + 1) * size, ptr + j * size, (i - j) * size);
      memcpy (ptr + j * size, tmp, size);
    }
}

static int
compar_reloc (const void *lhs, const void *rhs)
{
  const Elf_Internal_Rela *l = (const Elf_Internal_Rela *) lhs;
  const Elf_Internal_Rela *r = (const Elf_Internal_Rela *) rhs;

  if (l->r_offset > r->r_offset)
    return 1;
  else if (l->r_offset == r->r_offset)
    return 0;
  else
    return -1;
}

static bool
andes_relax_execit_ite (
  bfd *abfd,
  asection *sec,
  asection *sym_sec  ATTRIBUTE_UNUSED,
  struct bfd_link_info *info ATTRIBUTE_UNUSED,
  Elf_Internal_Rela *rel,
  bfd_vma symval,
  bfd_vma max_alignment,
  bfd_vma reserve_size ATTRIBUTE_UNUSED,
  bool *again ATTRIBUTE_UNUSED,
  //riscv_pcgp_relocs *pcgp_relocs ATTRIBUTE_UNUSED,
  void *pcgp_relocs ATTRIBUTE_UNUSED,
  bool undefined_weak ATTRIBUTE_UNUSED)
{
  int execit_index = (int) max_alignment;
  bfd_vma relocation = symval;
  bfd_vma pc = sec_addr (sec) + rel->r_offset;
  execit_item_t *item = &execit.itable_array[execit_index];
  execit_hash_t *he = item->he;
  execit_itable_t *ie = &he->ie;
  Elf_Internal_Rela reduction = *rel;
  int rtype = 0;

  if (he->type == ET_RK_TYPE_HI20)
    {
      execit_irel_t *p = item->grp_hd;
      bool found = false;
      while (p)
	{
	  if (p->ie.irel == (void*) rel->r_user)
	    {
	      found = true;
	      ie = &p->ie;
	      break;
	    }
	  p = p->next;
	}
      BFD_ASSERT (found);
      BFD_ASSERT (p->is_chosen);
    }

  if (ie->irel)
    {
      rtype = ELFNN_R_TYPE (ie->irel_copy.r_info);
      if (rtype == R_RISCV_CALL || rtype == R_RISCV_PCREL_HI20)
	rtype = ELFNN_R_TYPE (rel->r_info);
      else
	BFD_ASSERT (rtype == (int) ELFNN_R_TYPE (rel->r_info));
    }

  if (rtype == R_RISCV_HI20 || rtype == R_RISCV_CALL
      || rtype == R_RISCV_PCREL_HI20)
    { /* handle multiple reloction LUI/AUIPCs  */
      int i;
      if (rtype == R_RISCV_CALL || rtype == R_RISCV_PCREL_HI20)
	relocation -= pc;
      reduction.r_info = ELFNN_R_INFO (ELFNN_R_SYM (rel->r_info), rtype);
      bfd_vma hi20 = riscv_elf_encode_relocation (abfd, &reduction, relocation);
      if (item->is_final == false)
        {
	  item->is_final = true;
	  item->inf_ie = ie;
	  item->relocation = hi20;
	  execit.relocate_itable_done = false;
        }
      else
        {
	  int is_found = false;
	  BFD_ASSERT (item->grp_hd->ie.group_id == ie->group_id);
	  int entries = ie->entries;
	  for (i = 0; i < entries; ++i)
	    {
	      if (item->is_final)
		{
		  is_found = (item->relocation == hi20);
		  if (is_found)
		    {
		      ie = item->inf_ie;
		      break;
		    }
		  else if (item->next)
		    { /* try next  */
		      item = &execit.itable_array[item->next];
		      BFD_ASSERT (item->he == he);
		      continue;
		    }
		}
	      break;
	    }

	  if (!is_found)
	    { /* try allocate one  */
	      if ((i + 1) >= entries)
		{
		  BFD_ASSERT (0);
		  /* TODO: fatal handling
		   *   not enough entry reverved.
		   */
		  return false;
		}
	      else
		{
		  /* allocate index  */
		  int index = execit.next_itable_index++;
		  execit_item_t *t = &execit.itable_array[index];
		  *t = execit.itable_array[execit_index];
		  t->next = 0;
		  t->relocation = hi20;
		  t->inf_ie = ie;
		  ie->itable_index = index;

		  item->next = index;
		  item = t;

		  execit.relocate_itable_done = false;
		}
	    }
	}
      /* apply relocation  */
      bfd_byte *contents = elf_section_data (sec)->this_hdr.contents;
      uint16_t insn16 = execit.execit_op | ((execit.execit_op == EXECIT_INSN)
			  ? ENCODE_RVC_EXECIT_IMM (ie->itable_index << 2)
			  : ENCODE_RVC_NEXECIT_IMM (ie->itable_index << 2));
      bfd_put_16 (abfd, insn16, contents + rel->r_offset);
      if (rtype == R_RISCV_CALL)
	{ /* relocate the following JALR  */
	  Elf_Internal_Rela irel = {.r_info = ELFNN_R_INFO (0, R_RISCV_LO12_I)};
	  bfd_vma insn = bfd_get_32 (abfd, contents + rel->r_offset + 2);
	  bfd_vma value = riscv_elf_encode_relocation (abfd, &irel, relocation);
	  insn |= value;
	  bfd_put_32 (abfd, insn, contents + rel->r_offset + 2);
	}
    }
  else if (rtype == R_RISCV_JAL)
    { /* sanity check only  */
      BFD_ASSERT ((pc >> 21) == (ie->relocation >> 21));
      if (nsta.opt->execit_jal_over_2m
	  && ((pc >> 21) != (ie->relocation >> 21)))
	{
	  reloc_howto_type * r = riscv_elf_rtype_to_howto (abfd, rtype);
	  struct elf_link_hash_entry *h = ie->h;
	  (*_bfd_error_handler)
	    (_("%pB: relocation %s against `%s' crosses the 2M window of PC."
	       "Disable JAL over 2M to workaround."),
	     abfd, r ? r->name : _("<unknown>"),
	     h != NULL ? h->root.root.string : "a local symbol");
	  bfd_set_error (bfd_error_bad_value);
	  return false;
	}
    }
  else
    {
      BFD_ASSERT ((pc >> 21) == (ie->relocation >> 21));
    }

  /* record R_RISCV_PCREL_HI20 for pals.  */
  if (rtype == R_RISCV_JAL && nsta.opt->execit_jal_over_2m)
    { /* TODO: keep reloc to check in final relocation phase.  */
      rel->r_info = ELFNN_R_INFO (0, R_RISCV_NONE);
    }
  else
    {
      rel->r_info = ELFNN_R_INFO (0, R_RISCV_NONE);
    }

  return true;
}

static bool
riscv_mark_pcgp_hi_reloc (riscv_pcgp_relocs *p,riscv_pcgp_hi_reloc *hi)
{
  bool out = false;
  bfd_vma hi_sec_off = hi->hi_sec_off;
  riscv_pcgp_hi_reloc *c;

  for (c = p->hi; c != NULL; c = c->next)
    if (c->hi_sec_off == hi_sec_off)
      {
	out = true;
	c->is_marked = 1;
      }

  return out;
}

static bool
andes_relax_pc_gp_insn (
  bfd *abfd,
  asection *sec,
  asection *sym_sec,
  struct bfd_link_info *info,
  Elf_Internal_Rela *rel,
  bfd_vma symval,
  bfd_vma max_alignment,
  bfd_vma reserve_size,
  bool *again ATTRIBUTE_UNUSED,
  riscv_pcgp_relocs *pcgp_relocs,
  bool undefined_weak)
{
  bfd_byte *contents = elf_section_data (sec)->this_hdr.contents;
  bfd_vma gp = riscv_global_pointer_value (info);
  bfd_vma data_start = riscv_data_start_value (info);
  struct riscv_elf_link_hash_table *htab = riscv_elf_hash_table (info);
  andes_ld_options_t *andes = &htab->andes;
  bfd_vma guard_size = 0;
  bfd_signed_vma effect_range;

  BFD_ASSERT (rel->r_offset + 4 <= sec->size);

  /* Chain the _LO relocs to their cooresponding _HI reloc to compute the
   * actual target address.  */
  riscv_pcgp_hi_reloc *hi;
  riscv_pcgp_hi_reloc hi_reloc;
  memset (&hi_reloc, 0, sizeof (hi_reloc));
  switch (ELFNN_R_TYPE (rel->r_info))
    {
    case R_RISCV_PCREL_LO12_I:
    case R_RISCV_PCREL_LO12_S:
      {
	/* If the %lo has an addend, it isn't for the label pointing at the
	   hi part instruction, but rather for the symbol pointed at by the
	   hi part instruction.  So we must subtract it here for the lookup.
	   It is still used below in the final symbol address.  */
	bfd_vma hi_sec_off = symval - sec_addr (sym_sec) - rel->r_addend;
	hi = riscv_find_pcgp_hi_reloc (pcgp_relocs, hi_sec_off);
	if (hi == NULL)
	  {
	    riscv_record_pcgp_lo_reloc (pcgp_relocs, hi_sec_off);
	    return true;
	  }

	hi_reloc = *hi;
	symval = hi_reloc.hi_addr;
	sym_sec = hi_reloc.sym_sec;
	if (!riscv_use_pcgp_hi_reloc(pcgp_relocs, hi->hi_sec_off))
	  (*_bfd_error_handler)
	   (_("%pB(%pA+0x%lx): Unable to clear RISCV_PCREL_HI20 reloc"
	      "for cooresponding RISCV_PCREL_LO12 reloc"),
	    abfd, sec, rel->r_offset);

	/* We can not know whether the undefined weak symbol is referenced
	   according to the information of R_RISCV_PCREL_LO12_I/S.  Therefore,
	   we have to record the 'undefined_weak' flag when handling the
	   corresponding R_RISCV_HI20 reloc in riscv_record_pcgp_hi_reloc.  */
	undefined_weak = hi_reloc.undefined_weak;
      }
      break;

    case R_RISCV_PCREL_HI20:
      #if 0 /* working on merged address instead  */
      /* Mergeable symbols and code might later move out of range.  */
      if (! undefined_weak
	  && sym_sec->flags & (SEC_MERGE | SEC_CODE))
	return true;
      #endif
      /* If the cooresponding lo relocation has already been seen then it's not
       * safe to relax this relocation.  */
      if (riscv_find_pcgp_lo_reloc (pcgp_relocs, rel->r_offset))
	return true;

      break;

    default:
      abort ();
    }

  if (undefined_weak)
    return true; /* bypass to original handling  */

  /* For bug-14274, symbols defined in the .rodata (the sections
     before .data, may also later move out of range.  */
  /* reserved one page size in worst case
     or two (refer to riscv_relax_lui_to_rvc)  */
  if ((data_start == 0) || (sec_addr (sym_sec) < data_start))
    guard_size += info->relro ? andes->set_relax_page_size * 2
				   : andes->set_relax_page_size;

  if (gp)
    {
      /* If gp and the symbol are in the same output section, which is not the
	 abs section, then consider only that output section's alignment.  */
      struct bfd_link_hash_entry *h =
	bfd_link_hash_lookup (info->hash, RISCV_GP_SYMBOL, false, false, true);
      if (h->u.def.section->output_section == sym_sec->output_section
	  && sym_sec->output_section != bfd_abs_section_ptr)
	max_alignment = (bfd_vma) 1 << sym_sec->output_section->alignment_power;
    }

  /* Enable nds v5 gp relative insns.  */
  int do_replace = 0;
  uint32_t insn = bfd_get_32 (abfd, contents + rel->r_offset);
  /* For Bug-16488, check if gp-relative offset is in range.  */
  const int max_range = 0x20000;
  guard_size += max_alignment + reserve_size;
  effect_range = max_range - guard_size;
  if (effect_range < 0)
    return true; /* out of range */
  if (((symval >= gp) && ((symval - gp) < (bfd_vma) effect_range)) ||
      ((symval < gp) && ((gp - symval) <= (bfd_vma) effect_range)))
    {
      do_replace = 1;
      if (ELFNN_R_TYPE (rel->r_info) == R_RISCV_PCREL_HI20)
	{ /* here record only, defer relaxation to final  */
	    riscv_record_pcgp_hi_reloc_ext (
	      pcgp_relocs, rel->r_offset, rel->r_addend, symval,
	      ELFNN_R_SYM(rel->r_info), sym_sec, undefined_weak, rel);
	    return true;
	}
      else
	do_replace = andes_relax_gp_insn (&insn, rel, hi, sym_sec,
					  symval, gp, data_start);

      if (do_replace)
	{
	  bfd_put_32 (abfd, insn, contents + rel->r_offset);
	  return riscv_mark_pcgp_hi_reloc (pcgp_relocs, hi);
	}
      else
	{
	  BFD_ASSERT (hi);
	  hi->rel = NULL; /* mark (no relax it)  */
	}
    }

  /* Do not relax lui to c.lui here since the dangerous delete behavior.  */
  return true;
}

#define MIN(a, b) (a) < (b) ? (a) : (b)
static int
andes_min_p2align (Elf_Internal_Rela *rel, asection *sec)
{
  int limit = MIN (3, sec->alignment_power);
  int i;
  for (i = 0; i < limit; ++i)
    {
      uint m = 1u << i;
      if (rel && rel->r_addend & m) break;
      if (sec)
	{
	  if (sec->output_section->vma & m) break;
	  if (sec->output_section->output_offset & m) break;
	}
    }

  return i;
}

static int
andes_relax_gp_insn (uint32_t *insn, Elf_Internal_Rela *rel,
		     riscv_pcgp_hi_reloc_t *hi, asection *sym_sec,
		     bfd_vma symval, bfd_vma gp, bfd_vma data_start)
{
  /* symbols are not necessary aligned to data lenght.
     worst alignment is picked.  */
  bfd_signed_vma bias = symval - gp;
  int worst_p2alig = andes_min_p2align (rel, sym_sec);
  int type = ELFNN_R_TYPE (rel->r_info);
  int sym = hi->hi_sym;

  /* For Bug-16488, we don not need to consider max_alignment and
  reserve_size here, since they may cause the alignment checking
  fail.  */
  if ((*insn & MASK_ADDI) == MATCH_ADDI && VALID_GPTYPE_LB_IMM (bias))
    {
      rel->r_info = ELFNN_R_INFO (sym, R_RISCV_LGP18S0);
      if (type == R_RISCV_PCREL_LO12_I)
	rel->r_addend = hi->hi_addend;
      *insn = (*insn & (OP_MASK_RD << OP_SH_RD)) | MATCH_ADDIGP;
    }
  else if ((*insn & MASK_LB) == MATCH_LB && VALID_GPTYPE_LB_IMM (bias))
    {
      rel->r_info = ELFNN_R_INFO (sym, R_RISCV_LGP18S0);
      if (type == R_RISCV_PCREL_LO12_I)
	rel->r_addend = hi->hi_addend;
      *insn = (*insn & (OP_MASK_RD << OP_SH_RD)) | MATCH_LBGP;
    }
  else if ((*insn & MASK_LBU) == MATCH_LBU && VALID_GPTYPE_LB_IMM (bias))
    {
      rel->r_info = ELFNN_R_INFO (sym, R_RISCV_LGP18S0);
      if (type == R_RISCV_PCREL_LO12_I)
	rel->r_addend = hi->hi_addend;
      *insn = (*insn & (OP_MASK_RD << OP_SH_RD)) | MATCH_LBUGP;
    }
  else if ((*insn & MASK_LH) == MATCH_LH && VALID_GPTYPE_LH_IMM (bias)
	   && worst_p2alig > 0)
    {
      rel->r_info = ELFNN_R_INFO (sym, R_RISCV_LGP17S1);
      if (type == R_RISCV_PCREL_LO12_I)
	rel->r_addend = hi->hi_addend;
      *insn = (*insn & (OP_MASK_RD << OP_SH_RD)) | MATCH_LHGP;
    }
  else if ((*insn & MASK_LHU) == MATCH_LHU && VALID_GPTYPE_LH_IMM (bias)
	   && worst_p2alig > 0)
    {
      rel->r_info = ELFNN_R_INFO (sym, R_RISCV_LGP17S1);
      if (type == R_RISCV_PCREL_LO12_I)
	rel->r_addend = hi->hi_addend;
      *insn = (*insn & (OP_MASK_RD << OP_SH_RD)) | MATCH_LHUGP;
    }
  else if ((*insn & MASK_LW) == MATCH_LW && VALID_GPTYPE_LW_IMM (bias)
	   && worst_p2alig > 1)
    {
      rel->r_info = ELFNN_R_INFO (sym, R_RISCV_LGP17S2);
      if (type == R_RISCV_PCREL_LO12_I)
	rel->r_addend = hi->hi_addend;
      *insn = (*insn & (OP_MASK_RD << OP_SH_RD)) | MATCH_LWGP;
    }
  else if ((*insn & MASK_LWU) == MATCH_LWU && VALID_GPTYPE_LW_IMM (bias)
	   && worst_p2alig > 1)
    {
      rel->r_info = ELFNN_R_INFO (sym, R_RISCV_LGP17S2);
      if (type == R_RISCV_PCREL_LO12_I)
	rel->r_addend = hi->hi_addend;
      *insn = (*insn & (OP_MASK_RD << OP_SH_RD)) | MATCH_LWUGP;
    }
  else if ((*insn & MASK_LD) == MATCH_LD && VALID_GPTYPE_LD_IMM (bias)
	   && worst_p2alig > 2)
    {
      rel->r_info = ELFNN_R_INFO (sym, R_RISCV_LGP17S3);
      if (type == R_RISCV_PCREL_LO12_I)
	rel->r_addend = hi->hi_addend;
      *insn = (*insn & (OP_MASK_RD << OP_SH_RD)) | MATCH_LDGP;
    }
  else if ((*insn & MASK_SB) == MATCH_SB && VALID_GPTYPE_SB_IMM (bias))
    {
      rel->r_info = ELFNN_R_INFO (sym, R_RISCV_SGP18S0);
      if (type == R_RISCV_PCREL_LO12_S)
	rel->r_addend = hi->hi_addend;
      *insn = (*insn & (OP_MASK_RS2 << OP_SH_RS2)) | MATCH_SBGP;
    }
  else if ((*insn & MASK_SH) == MATCH_SH && VALID_GPTYPE_SH_IMM (bias)
	   && worst_p2alig > 0)
    {
      rel->r_info = ELFNN_R_INFO (sym, R_RISCV_SGP17S1);
      if (type == R_RISCV_PCREL_LO12_S)
	rel->r_addend = hi->hi_addend;
      *insn = (*insn & (OP_MASK_RS2 << OP_SH_RS2)) | MATCH_SHGP;
    }
  else if ((*insn & MASK_SW) == MATCH_SW && VALID_GPTYPE_SW_IMM (bias)
	   && worst_p2alig > 1)
    {
      rel->r_info = ELFNN_R_INFO (sym, R_RISCV_SGP17S2);
      if (type == R_RISCV_PCREL_LO12_S)
	rel->r_addend = hi->hi_addend;
      *insn = (*insn & (OP_MASK_RS2 << OP_SH_RS2)) | MATCH_SWGP;
    }
  else if ((*insn & MASK_SD) == MATCH_SD && VALID_GPTYPE_SD_IMM (bias)
	   && worst_p2alig > 2)
    {
      rel->r_info = ELFNN_R_INFO (sym, R_RISCV_SGP17S3);
      if (type == R_RISCV_PCREL_LO12_S)
	rel->r_addend = hi->hi_addend;
      *insn = (*insn & (OP_MASK_RS2 << OP_SH_RS2)) | MATCH_SDGP;
    }
  else if (type == R_RISCV_LO12_I || type == R_RISCV_LO12_S
	   || type == R_RISCV_PCREL_LO12_I || type == R_RISCV_PCREL_LO12_S)
    {
      bool is_out = false;
      while (true)
	{
	  /* GP value might reduce by 4K during resizing sections,
	     filter out symbols before DATA_START for safety.  */
	  if ((type == R_RISCV_PCREL_LO12_I) || (type == R_RISCV_PCREL_LO12_S))
	    is_out = (symval - hi->hi_addend) < data_start;
	  else
	    is_out = (symval - rel->r_addend) < data_start;
	  if (is_out) break;

	  /* alignment would increase gap between gp and symbol. (b26938)
	     guard with the alignment of the one with larger VMA.  */
	  /* TODO: replace alignment with gp's if VMA(gp) is larger.  */
	  {
	    int range = 0x1000 >> 1;
	    /* presume gp is XLEN/8 aligned. so mostly guard = 0.  */
	    unsigned xlen = ARCH_SIZE;
	    unsigned align = 1u << sym_sec->alignment_power;
	    bfd_vma guard = (align > (xlen >> 3)) ? (align - (xlen >> 3)) : 0;
	    bfd_signed_vma effect = range - guard;
	    if (effect < 0
		|| !(((symval >= gp) && ((signed)(symval - gp) < effect))
		     || ((symval < gp) && ((signed)(gp - symval) <= effect))))
	      {
		is_out = true;
		break;
	      }
	  }

	  if (((*insn & MASK_FLH) == MATCH_FLH
	       || (*insn & MASK_FLW) == MATCH_FLW
	       || (*insn & MASK_FLD) == MATCH_FLD)
	      && VALID_ITYPE_IMM (bias)
	      && worst_p2alig > 1)
	    {
	      if (type == R_RISCV_PCREL_LO12_I)
		{
		  rel->r_info = ELFNN_R_INFO (sym, type);
		  rel->r_addend = hi->hi_addend;
		}
	      andes_extend_irel(rel, TAG_GPREL_SUBTYPE_FLX, &nsta.ext_irel_list);
	      *insn = (*insn & ~(OP_MASK_RS1 << OP_SH_RS1)) | (GPR_ABI_GP << OP_SH_RS1);
	    }
	  else if (((*insn & MASK_FSH) == MATCH_FSH
		    || (*insn & MASK_FSW) == MATCH_FSW
		    || (*insn & MASK_FSD) == MATCH_FSD)
		   && VALID_STYPE_IMM (bias)
		   && worst_p2alig > 1)
	    {
	      if (type == R_RISCV_PCREL_LO12_S)
		{
		  rel->r_info = ELFNN_R_INFO (sym, type);
		  rel->r_addend = hi->hi_addend;
		}
	      andes_extend_irel(rel, TAG_GPREL_SUBTYPE_FSX, &nsta.ext_irel_list);
	      *insn = (*insn & ~(OP_MASK_RS1 << OP_SH_RS1)) | (GPR_ABI_GP << OP_SH_RS1);
	    }
	  else
	    is_out = true;
	  break; /* once */
	}

      if (is_out)
	return false;
    }
  else
    return false;

  return true;
}

static void
andes_relax_pc_gp_insn_final (riscv_pcgp_relocs *p)
{
  riscv_pcgp_hi_reloc *c;

  for (c = p->hi; c != NULL; c = c->next)
    {
      if (c->rel && c->is_marked)
	{ /* We can delete the unnecessary AUIPC and reloc.  */
	  c->rel->r_info = ELFNN_R_INFO (0, R_RISCV_DELETE);
	  c->rel->r_addend = 4;
	}
    }
}

static int
riscv_elf_output_symbol_hook (struct bfd_link_info *info,
			      const char *name,
			      Elf_Internal_Sym *elfsym ATTRIBUTE_UNUSED,
			      asection *input_sec,
			      struct elf_link_hash_entry *h ATTRIBUTE_UNUSED)
{
  const char *source;
  FILE *sym_ld_script = NULL;
  struct riscv_elf_link_hash_table *table;
  andes_ld_options_t *andes;

  table = riscv_elf_hash_table (info);
  andes = &table->andes;
  sym_ld_script = andes->sym_ld_script;
  if (!sym_ld_script)
    return true;

  if (!h || !name || *name == '\0')
    return true;

  if (input_sec->flags & SEC_EXCLUDE)
    return true;

  if (!nsta.check_start_export_sym)
    {
      fprintf (sym_ld_script, "SECTIONS\n{\n");
      nsta.check_start_export_sym = 1;
      /* dump ICT table if necessary  */
      andes_ict_entry_t *p = nds_ict_sta.list_head;
      if (p)
	{
	  int i = 0;
	  struct bfd_link_hash_entry *ict_base = bfd_link_hash_lookup (
	    info->hash, "_INDIRECT_CALL_TABLE_BASE_", false, false, false);
	  bfd_vma ict_base_vma = ict_base->u.def.value
				 + sec_addr (ict_base->u.def.section);
	  fprintf (sym_ld_script, "\t.nds.ict 0x%08lx : {\n", ict_base_vma);
	  while (p)
	    {
	      BFD_ASSERT (riscv_elf_hash_entry (p->h)->indirect_call);
	      bfd_vma sym_value = p->h->root.u.def.value
				  + sec_addr (p->h->root.u.def.section);
	      fprintf (sym_ld_script, "\t\tICT_ENTRY(%d, %s, 0x%08lx);\n",
		       i, p->h->root.root.string, sym_value);
	      p = p->next;
	      i++;
	    }
	  fprintf (sym_ld_script, "\t}\n");
	}
    }

  if (h->root.type == bfd_link_hash_defined
      || h->root.type == bfd_link_hash_defweak)
    {
      if (!h->root.u.def.section->output_section)
	return true;

      if (bfd_is_const_section (input_sec))
	source = input_sec->name;
      else
	source = input_sec->owner->filename;

      bfd_vma sym_value = h->root.u.def.value
	+ h->root.u.def.section->output_section->vma
	+ h->root.u.def.section->output_offset;

      if (!riscv_elf_hash_entry (h)->indirect_call)
	fprintf (sym_ld_script, "\t%s = 0x%08lx;\t /* %s  */\n",
		 h->root.root.string, sym_value, source);
    }

  return true;
}

static bool
riscv_elf_output_arch_syms (bfd *output_bfd ATTRIBUTE_UNUSED,
			    struct bfd_link_info *info,
			    void *finfo ATTRIBUTE_UNUSED,
			    int (*func) (void *, const char *,
					 Elf_Internal_Sym *,
					 asection *,
					 struct elf_link_hash_entry *)
			      ATTRIBUTE_UNUSED)
{
  FILE *sym_ld_script = NULL;
  struct riscv_elf_link_hash_table *table;
  andes_ld_options_t *andes;

  table = riscv_elf_hash_table (info);
  andes = &table->andes;
  sym_ld_script = andes->sym_ld_script;

  if (nsta.check_start_export_sym)
    fprintf (sym_ld_script, "}\n");

  return true;
}

/* Store the machine number in the flags field.  */

static bool
riscv_elf_final_write_processing (bfd *abfd ATTRIBUTE_UNUSED)
{
#ifdef OLD_ICT_OUTPUT
  ict_entry_t *head;

  /* Export the ICT table if needed.  */
  /* TODO: should we support new linker option to let user
     can define their own ict table name.  */
  if (exported_ict_table_head)
    {
      FILE *ict_table_file = fopen ("nds_ict.s", FOPEN_WT);
      if(ict_table_file == NULL)
	{
	  (*_bfd_error_handler) (_("Error: Fail to genertare nds_ict.s."));
	  return false;
	}

      fprintf (ict_table_file, "\t.section " ANDES_ICT_SECTION ", \"ax\"\n");
      /* The exported ict table can not be linked with the patch code
	 that use the different ict model.  */
      if (ict_model == 0)
	fprintf (ict_table_file, "\t.attribute ict_model, \"tiny\"\n");
      else if (ict_model == 1)
	fprintf (ict_table_file, "\t.attribute ict_model, \"small\"\n");
      else
	fprintf (ict_table_file, "\t.attribute ict_model, \"large\"\n");
      fprintf (ict_table_file, ".global _INDIRECT_CALL_TABLE_BASE_\n"
	       "_INDIRECT_CALL_TABLE_BASE_:\n");
      fprintf (ict_table_file, "\t.option push\n\t.option norelax\n");

      /* Output each entry of ict table according to different
	 ict models.  */
      head = exported_ict_table_head;
      while (head)
	{
	  if (ict_model == 2)
	    fprintf (ict_table_file, "\t.dword\t%s\n",
		     head->h->root.root.string);
	  else if (ict_model == 1)
	    fprintf (ict_table_file, "\ttail\t%s\n",
		     head->h->root.root.string);
	  else
	    fprintf (ict_table_file, "\tjal\tt1, %s\n",
		     head->h->root.root.string);
	  head = head->next;
	}

      fprintf (ict_table_file, "\t.option pop\n");

      /* Finish exporting the ict table, close it
	 and free the unused data.  */
      while (exported_ict_table_head)
	{
	  head = exported_ict_table_head;
	  exported_ict_table_head = exported_ict_table_head->next;
	  free (head);
	}
      fclose (ict_table_file);
    }
#endif /* OLD_ICT_OUTPUT */

  return true;
}

/* Find the symbol '__global_pointer$' in the output bfd.
   If not found, set it's value to (sdata + 0x800) by default.
   TODO: figure out the best SDA_BASE/GP value.
*/

static bool
riscv_init_global_pointer (bfd *output_bfd, struct bfd_link_info *info)
{
  struct bfd_link_hash_entry *h;
  asection *section = NULL;
  bfd_vma gp_value = 0x800;

  h = bfd_link_hash_lookup (info->hash, RISCV_GP_SYMBOL, false, false, true);
  /* 1. no GP_SYMBOL found, disable gp relaxation  */
  if (!h)
    {
      struct riscv_elf_link_hash_table *table;
      table = riscv_elf_hash_table (info);
      andes_ld_options_t *andes = &table->andes;
      andes->gp_relative_insn = 0;
    }
  /* 2. GP_SYMBOL referenced but not defined, generate one  */
  else if (h->type == bfd_link_hash_undefined)
    {
      /* find a suitable section to insert symbol.  */
      const char *sections[] = {".sdata", ".sbss", ".data", ".bss", NULL};
      int index = 0;
      while (sections[index])
	{
	  section = bfd_get_section_by_name (output_bfd, sections[index]);
	  if (section)
	    break;
	  index++;
	}
      /* if none, just insert it at COMMON (.bss) section blindly.  */
      if (!section)
	{
	  section = bfd_abs_section_ptr;
	  gp_value = 0;
	}

      if (!_bfd_generic_link_add_one_symbol
	  (info, output_bfd, RISCV_GP_SYMBOL, BSF_GLOBAL, section,
	   gp_value, (const char *) NULL, false,
	   get_elf_backend_data (output_bfd)->collect, &h))
	return false;
    }
  return true;
}

/* adapt from _bfd_riscv_relax_align
   Implement R_RISCV_ALIGN_BTB by deleting excess alignment NOPs.
   Once we've handled an R_RISCV_ALIGNBTB, we can't relax anything else.  */

static bool
_bfd_riscv_relax_align_btb (bfd *abfd, asection *sec,
			    asection *sym_sec ATTRIBUTE_UNUSED,
			    struct bfd_link_info *info,
			    Elf_Internal_Rela *rel,
			    bfd_vma symval,
			    bfd_vma max_alignment ATTRIBUTE_UNUSED,
			    bfd_vma reserve_size ATTRIBUTE_UNUSED,
			    bool *again ATTRIBUTE_UNUSED,
			    riscv_pcgp_relocs *pcgp_relocs ATTRIBUTE_UNUSED,
			    bool undefined_weak ATTRIBUTE_UNUSED)
{
  link_hash_table_t *htab = riscv_elf_hash_table (info);
  andes_ld_options_t *nopt = &htab->andes;
  bfd_vma filled;

  BFD_ASSERT (rel->r_addend == 4);

  /* Once we've handled an R_RISCV_ALIGN_BTB, we can't relax anything else.  */
  sec->sec_flg0 = true;

  /*       r_offset                          symval (address)
   *  -----+----------------+----------------+--------------------
   *       |    filled      /    delete      |
   *  -----+----------------+----------------+--------------------
   *       <    rel->r_addend == 4 (NOP)     >
   */

  /* try BTB target align  */
  if (nopt->target_aligned)
    {
      if (symval & 3)
	{
	  BFD_ASSERT ((symval & 3) == 2);
	  bfd_vma offset = nsta.prev_aligned_offset;
	  bfd_vma end = rel->r_offset;
	  filled = andes_try_target_align (abfd, sec, sym_sec, info, rel,
					   offset, end);
	  /* rel->offset might be changed!  */
	  rel->r_addend -= filled;

	}
      nsta.prev_aligned_offset = rel->r_offset;
    }

  /* Delete the reloc.  */
  rel->r_info = ELFNN_R_INFO (0, R_RISCV_NONE);

  /* check if BTB miss
   *   pc: (A)    r_offset (B)   symval (C)    (D)
   *   +----------+--------------+-------------+-----------------------+
   *   | jal/jalr | fill? / nop  | 16-bit insn | 16-bit jump or branch |
   *   +----------+------+-------+-------------+-----------------------+
   *                     ^ filled (4 *n )
   */

  /* as rel->r_offset must be sorted. so after BTB applied, the rel->r_offset
   * point to RVI instead of NOP. keep the missing offset in filled.
   */
  filled = 0;
  if (nopt->avoid_btb_miss
      && riscv_relax_check_BTB_miss (abfd, sec, rel))
    filled = riscv_relax_avoid_BTB_miss (abfd, sec, rel) ? 4 : 0;

  /* If the number of NOPs is already correct, there's nothing to do.  */
  if (rel->r_addend == 0)
    return true;

  /* Delete the unwated bytes.  */
  return riscv_relax_delete_bytes (abfd, sec, rel->r_offset + filled,
				   rel->r_addend, info, NULL);
}

/* Convert a RVC instruction to RVI one.  Return 1 if done successfully,
   otherwise, return 0.  */
/* Just consider RV32C/RV64C. Skip RV128C instructions.  */

static int
riscv_convert_16_to_32 (uint16_t insn16, uint32_t *insn32)
{
  bfd_vma imm;
  /* Stack-Pointer-Based Loads and Stores.  */
  /* TODO: C.LQSP, C.FLWSP, C.FLDSP, C.SQSP, C.FSWSP, C.FSDSP.  */
  if ((insn16 & MASK_C_LWSP) == MATCH_C_LWSP)
    {
      /* CI format to I-TYPE.  */
      int rd = (insn16 >> 7) & 0x1f;
      if (rd == 0)
	return 0;
      imm = EXTRACT_CITYPE_LWSP_IMM (insn16);
      *insn32 = RISCV_ITYPE (LW, rd, X_SP, imm);  /* lw rd, imm(x2)  */
    }
  else if ((ARCH_SIZE == 64)
	   && (insn16 & MASK_C_LDSP) == MATCH_C_LDSP)
    {
      /* CI format to I-TYPE.  */
      int rd = (insn16 >> 7) & 0x1f;
      if (rd == 0)
	return 0;
      imm = EXTRACT_CITYPE_LDSP_IMM (insn16);
      *insn32 = RISCV_ITYPE (LD, rd, X_SP, imm); /* ld rd, imm(x2)  */
    }
  else if ((insn16 & MASK_C_SWSP) == MATCH_C_SWSP)
    {
      /* CSS format to S-TYPE.  */
      int rs2 = (insn16 >> OP_SH_CRS2) & OP_MASK_CRS2;
      imm = EXTRACT_CSSTYPE_SWSP_IMM (insn16);
      *insn32 = RISCV_STYPE (SW, X_SP, rs2, imm); /* sw rs2, imm(x2)  */
    }
  else if ((ARCH_SIZE == 64)
	   && (insn16 & MASK_C_SDSP) == MATCH_C_SDSP)
    {
      /* CSS format to S-TYPE.  */
      int rs2 = (insn16 >> OP_SH_CRS2) & OP_MASK_CRS2;
      imm = EXTRACT_CSSTYPE_SDSP_IMM (insn16);
      *insn32 = RISCV_STYPE (SD, X_SP, rs2, imm); /* sw rs2, imm(x2)  */
    }

  /* Register-Based Loads and Stores.  */
  /* TODO: C.LQ, C.FLW, C.FLD, C.SQ, C.FSW, C.FSD.  */
  else if ((insn16 & MASK_C_LW) == MATCH_C_LW)
    {
      /* CL format to I-TYPE.  */
      int rs1 = 8 + ((insn16 >> OP_SH_CRS1S) & OP_MASK_CRS1S);
      int rd = 8 + ((insn16 >> OP_SH_CRS2S) & OP_MASK_CRS2S);
      imm = EXTRACT_CLTYPE_LW_IMM (insn16);
      *insn32 = RISCV_ITYPE (LW, rd, rs1, imm); /* lw rd, imm(rs1)  */
    }
  else if ((ARCH_SIZE == 64)
	   && (insn16 & MASK_C_LD) == MATCH_C_LD)
    {
      /* CL format to I-TYPE.  */
      int rs1 = 8 + ((insn16 >> OP_SH_CRS1S) & OP_MASK_CRS1S);
      int rd = 8 + ((insn16 >> OP_SH_CRS2S) & OP_MASK_CRS2S);
      imm = EXTRACT_CLTYPE_LD_IMM (insn16);
      *insn32 = RISCV_ITYPE (LD, rd, rs1, imm); /* ld rd, imm(rs1)  */
    }
  else if ((insn16 & MASK_C_SW) == MATCH_C_SW)
    {
      /* CS format to S-TYPE.  */
      int rs1 = 8 + ((insn16 >> OP_SH_CRS1S) & OP_MASK_CRS1S);
      int rs2 = 8 + ((insn16 >> OP_SH_CRS2S) & OP_MASK_CRS2S);
      imm = EXTRACT_CLTYPE_LW_IMM (insn16);
      *insn32 = RISCV_STYPE (SW, rs1, rs2, imm); /* sw rs2, imm(rs1)  */
    }
  else if ((ARCH_SIZE == 64)
	   && (insn16 & MASK_C_SD) == MATCH_C_SD)
    {
      /* CS format to S-TYPE.  */
      int rs1 = 8 + ((insn16 >> OP_SH_CRS1S) & OP_MASK_CRS1S);
      int rs2 = 8 + ((insn16 >> OP_SH_CRS2S) & OP_MASK_CRS2S);
      imm = EXTRACT_CLTYPE_LD_IMM (insn16);
      *insn32 = RISCV_STYPE (SD, rs1, rs2, imm); /* sd rs2, imm(rs1)  */
    }

  /* Control Transfer Instructions.  */
  else if ((insn16 & MASK_C_J) == MATCH_C_J)
    {
      /* CJ format to UJ-TYPE.  */
      imm = EXTRACT_CJTYPE_IMM (insn16);
      *insn32 = RISCV_JTYPE (JAL, 0, imm);  /* jal x0, imm  */
    }
  else if ((ARCH_SIZE == 32)
	   && (insn16 & MASK_C_JAL) == MATCH_C_JAL)
    {
      /* CJ format to UJ-TYPE.  */
      imm = EXTRACT_CJTYPE_IMM (insn16);
      *insn32 = RISCV_JTYPE (JAL, X_RA, imm);  /* jal x0, imm  */
    }
  else if ((insn16 & MASK_C_JR) == MATCH_C_JR)
    {
      /* CR format to I-TYPE.  */
      int rs1 = (insn16 >> 7) & 0x1f;
      int rs2 = (insn16 >> OP_SH_CRS2) & OP_MASK_CRS2;
      if (rs1 == 0 || rs2 != 0)
	return 0;
      *insn32 = RISCV_ITYPE (JALR, 0, rs1, 0);  /* jalr x0, rs1, 0  */
    }
  else if ((insn16 & MASK_C_JALR) == MATCH_C_JALR)
    {
      /* CR format to I-TYPE.  */
      int rs1 = (insn16 >> 7) & 0x1f;
      int rs2 = (insn16 >> OP_SH_CRS2) & OP_MASK_CRS2;
      if (rs1 == 0 || rs2 != 0)
	return 0;
      *insn32 = RISCV_ITYPE (JALR, X_RA, rs1, 0); /* jalr x1, rs1, 0  */
    }
  else if ((insn16 & MASK_C_BEQZ) == MATCH_C_BEQZ)
    {
      /* CB format to SB-TYPE.  */
      int rs1 = 8 + ((insn16 >> OP_SH_CRS1S) & OP_MASK_CRS1S);
      imm = EXTRACT_CBTYPE_IMM (insn16);
      *insn32 = RISCV_BTYPE (BEQ, rs1, 0, imm);  /* beq rs1, x0, imm  */
    }
  else if ((insn16 & MASK_C_BNEZ) == MATCH_C_BNEZ)
    {
      /* CB format to SB-TYPE.  */
      int rs1 = 8 + ((insn16 >> OP_SH_CRS1S) & OP_MASK_CRS1S);
      imm = EXTRACT_CBTYPE_IMM (insn16);
      *insn32 = RISCV_BTYPE (BNE, rs1, 0, imm);  /* bne rs1, x0, imm  */
    }

  /* Integer Register-Immediate Operations.  */
  /* TODO: C.ADDIW.  */
  else if ((insn16 & MASK_C_ADDI) == MATCH_C_ADDI)
    {
      /* CI format to I-TYPE.  */
      int rd = (insn16 >> 7) & 0x1f;
      imm = EXTRACT_CITYPE_IMM (insn16);
      *insn32 = RISCV_ITYPE (ADDI, rd, rd, imm); /* addi rd, rd, nzimm  */
    }
  else if ((insn16 & MASK_C_ADDI16SP) == MATCH_C_ADDI16SP)
    {
      /* CI format to I-TYPE.  */
      /* c.addi16sp shares the opcode with c.lui.  */
      int rd = (insn16 >> 7) & 0x1f;
      if (rd != X_SP)
	return 0;

      imm = EXTRACT_CITYPE_ADDI16SP_IMM (insn16);
      *insn32 = RISCV_ITYPE (ADDI, X_SP, X_SP, imm); /* addi x2, x2, nzimm  */
    }
  else if ((insn16 & MASK_C_LUI) == MATCH_C_LUI)
    {
      /* CI format to I-TYPE.  */
      /* c.addi16sp shares the opcode with c.lui.  */
      int rd = (insn16 >> 7) & 0x1f;
      if (rd == 0 || rd == X_SP)
	return 0;

      imm = EXTRACT_CITYPE_LUI_IMM (insn16);
      *insn32 = RISCV_UTYPE (LUI, rd, imm); /* lui rd, nzimm  */
    }
  else if ((insn16 & MASK_C_ADDI4SPN) == MATCH_C_ADDI4SPN)
    {
      /* CIW format to I-TYPE.  */
      int rd = 8 + ((insn16 >> OP_SH_CRS2S) & OP_MASK_CRS2S);
      imm = EXTRACT_CIWTYPE_ADDI4SPN_IMM (insn16);
      *insn32 = RISCV_ITYPE (ADDI, rd, X_SP, imm); /* addi rd, x2, zimm  */
    }
  else if ((insn16 & MASK_C_SLLI) == MATCH_C_SLLI)
    {
      /* CI format to I-TYPE.  */
      int rd = (insn16 >> 7) & 0x1f;
      if (rd == 0)
	return 0;

      imm = RV_X(insn16, 2, 5) | (RV_X(insn16, 12, 1) << 5);
      *insn32 = RISCV_ITYPE (SLLI, rd, rd, imm); /* slli rd, rd, imm  */
    }
  else if ((insn16 & MASK_C_SRLI) == MATCH_C_SRLI)
    {
      /* CB format to I-TYPE.  */
      int rd = 8 + ((insn16 >> OP_SH_CRS1S) & OP_MASK_CRS1S);
      imm = RV_X(insn16, 2, 5) | (RV_X(insn16, 12, 1) << 5);
      *insn32 = RISCV_ITYPE (SRLI, rd, rd, imm); /* srli rd, rd, imm  */
    }
  else if ((insn16 & MASK_C_SRAI) == MATCH_C_SRAI)
    {
      /* CB format to I-TYPE.  */
      int rd = 8 + ((insn16 >> OP_SH_CRS1S) & OP_MASK_CRS1S);
      imm = RV_X(insn16, 2, 5) | (RV_X(insn16, 12, 1) << 5);
      *insn32 = RISCV_ITYPE (SRAI, rd, rd, imm); /* srai rd, rd, imm  */
    }
  else if ((insn16 & MASK_C_ANDI) == MATCH_C_ANDI)
    {
      /* CB format to I-TYPE.  */
      int rd = 8 + ((insn16 >> OP_SH_CRS1S) & OP_MASK_CRS1S);
      imm = EXTRACT_CITYPE_IMM (insn16);
      *insn32 = RISCV_ITYPE (ANDI, rd, rd, imm); /* andi rd, rd, imm  */
    }

  /* Integer Constant-Generation Instructions.  */
  else if ((insn16 & MASK_C_LI) == MATCH_C_LI)
    {
      /* CI format to I-TYPE.  */
      int rd = (insn16 >> 7) & 0x1f;
      if (rd == 0)
	return 0;
      imm = EXTRACT_CITYPE_IMM (insn16);
      *insn32 = RISCV_ITYPE (ADDI, rd, 0, imm); /* addi rd, x0, imm  */
    }

  /* Integer Register-Register Operations.  */
  /* TODO: C.ADDW, C.SUBW.  */
  else if ((insn16 & MASK_C_MV) == MATCH_C_MV)
    {
      /* CR format to R-TYPE.  */
      int rd = (insn16 >> 7) & 0x1f;
      int rs2 = (insn16 >> 2) & 0x1f;
      if (rd == 0 || rs2 == 0)
	return 0;
      /* b24252 */
      *insn32 = RISCV_RTYPE (ADDI, rd, rs2, 0); /* addi rd, rs2, 0  */
    }
  else if ((insn16 & MASK_C_ADD) == MATCH_C_ADD)
    {
      /* CR format to R-TYPE.  */
      int rd = (insn16 >> 7) & 0x1f;
      int rs2 = (insn16 >> 2) & 0x1f;
      if (rd == 0 || rs2 == 0)
	return 0;
      *insn32 = RISCV_RTYPE (ADD, rd, rd, rs2); /* add rd, rd, rs2  */
    }
  else if ((insn16 & MASK_C_AND) == MATCH_C_AND)
    {
      /* CS format to R-TYPE.  */
      int rd = 8 + ((insn16 >> OP_SH_CRS1S) & OP_MASK_CRS1S);
      int rs2 = 8 + ((insn16 >> OP_SH_CRS2S) & OP_MASK_CRS2S);
      *insn32 = RISCV_RTYPE (AND, rd, rd, rs2); /* and rd, rd, rs2  */
    }
  else if ((insn16 & MASK_C_OR) == MATCH_C_OR)
    {
      /* CS format to R-TYPE.  */
      int rd = 8 + ((insn16 >> OP_SH_CRS1S) & OP_MASK_CRS1S);
      int rs2 = 8 + ((insn16 >> OP_SH_CRS2S) & OP_MASK_CRS2S);
      *insn32 = RISCV_RTYPE (OR, rd, rd, rs2); /* or rd, rd, rs2  */
    }
  else if ((insn16 & MASK_C_XOR) == MATCH_C_XOR)
    {
      /* CS format to R-TYPE.  */
      int rd = 8 + ((insn16 >> OP_SH_CRS1S) & OP_MASK_CRS1S);
      int rs2 = 8 + ((insn16 >> OP_SH_CRS2S) & OP_MASK_CRS2S);
      *insn32 = RISCV_RTYPE (XOR, rd, rd, rs2); /* xor rd, rd, rs2  */
    }
  else if ((insn16 & MASK_C_SUB) == MATCH_C_SUB)
    {
      /* CS format to R-TYPE.  */
      int rd = 8 + ((insn16 >> OP_SH_CRS1S) & OP_MASK_CRS1S);
      int rs2 = 8 + ((insn16 >> OP_SH_CRS2S) & OP_MASK_CRS2S);
      *insn32 = RISCV_RTYPE (SUB, rd, rd, rs2); /* sub rd, rd, rs2  */
    }
  else
    {
      /* This RVC can not be converted to RVI.  */
      return 0;
    }

  return 1;
}

static bool
riscv_convert_16_to_32_reloc (Elf_Internal_Rela **irel)
{
  if (*irel)
    {
      unsigned sym = ELFNN_R_SYM ((*irel)->r_info);
      unsigned type = ELFNN_R_TYPE ((*irel)->r_info);
      if (type == R_RISCV_RVC_BRANCH)
	(*irel)->r_info = ELFNN_R_INFO (sym, R_RISCV_BRANCH);
      else if (type == R_RISCV_RVC_JUMP)
	(*irel)->r_info = ELFNN_R_INFO (sym, R_RISCV_JAL);
      else if (type == R_RISCV_RVC_LUI)
	(*irel)->r_info = ELFNN_R_INFO (sym, R_RISCV_HI20);
      else
	/* Unsupported reloc converting.  */
	return false;
    }
  return true;
}

/* Check whether the ranges of 32-bit branch and jal is valid between
   the rvc candidate and alignment point after doing target aligned.  */

static bool
target_align_check_branch_range (bfd *abfd, asection *sec, bfd_vma insn16_off,
				 bfd_vma nops_off, size_t count,
				 struct bfd_link_info *link_info)
{
  struct elf_link_hash_entry **sym_hashes = elf_sym_hashes (abfd);
  Elf_Internal_Shdr *symtab_hdr = &elf_tdata (abfd)->symtab_hdr;
  Elf_Internal_Sym *sym = (Elf_Internal_Sym *) symtab_hdr->contents;
  struct bfd_elf_section_data *data = elf_section_data (sec);
  bfd_byte *contents = data->this_hdr.contents;
  Elf_Internal_Rela *irel, *irelend;
  bfd_vma relocation = 0, pc = 0;
  bfd_vma where = insn16_off + 2;

  irel = data->relocs;
  irelend = data->relocs + sec->reloc_count;
  while (where < nops_off)
    {
      if ((*(contents + where) & 0x3) != 0x3)
	/* 16-bits insn, skip it.  */
	where += 2;
      else
	{
	  /* Find the relocation that it's r_offset is same as where.  */
	  while (irel != NULL && irel < irelend && irel->r_offset < where)
	    irel++;

	  /* Only check branch and jal range. If overflow, return 0.  */
	  riscv_relocation_check (link_info, &irel, irelend, sec, &where, contents, 0);

	  if (ELFNN_R_TYPE (irel->r_info) == R_RISCV_BRANCH
	      || ELFNN_R_TYPE (irel->r_info) == R_RISCV_JAL)
	    {
	      unsigned long r_symndx = ELFNN_R_SYM (irel->r_info);
	      if (r_symndx < symtab_hdr->sh_info)
		{
		  /* Local symbol.  */
		  int shndx = sym[r_symndx].st_shndx;
		  bfd_vma st_value = (sym + r_symndx)->st_value;
		  asection *sym_sec = elf_elfsections (abfd)[shndx]->bfd_section;
		  relocation = sym_sec->output_section->vma
		    + sym_sec->output_offset
		    + st_value;
		  if ((sym + r_symndx)->st_value > insn16_off
		      && (sym + r_symndx)->st_value <= nops_off)
		    relocation += count;
		}
	      else
		{
		  /* External symbol.  */
		  bool warned ATTRIBUTE_UNUSED;
		  bool ignored ATTRIBUTE_UNUSED;
		  bool unresolved_reloc ATTRIBUTE_UNUSED;
		  struct elf_link_hash_entry *h;
		  asection *sym_sec;

		  RELOC_FOR_GLOBAL_SYMBOL (link_info, abfd, sec, irel,
					   r_symndx, symtab_hdr, sym_hashes,
					   h, sym_sec, relocation,
					   unresolved_reloc, warned, ignored);

		  if (h->root.u.def.value > insn16_off
		      && h->root.u.def.value <= nops_off)
		    relocation += count;
		}

	      pc = sec_addr (sec) + irel->r_offset;
	      if (irel->r_offset > insn16_off
		  && irel->r_offset < nops_off)
		pc += count;

	      reloc_howto_type *howto;
	      howto = riscv_elf_rtype_to_howto (abfd, ELFNN_R_TYPE (irel->r_info));
	      if (howto->pc_relative)
		relocation -= pc;
	      relocation += irel->r_addend;

	      if ((ELFNN_R_TYPE (irel->r_info) == R_RISCV_BRANCH
		   && !VALID_SBTYPE_IMM (relocation))
		  || (ELFNN_R_TYPE (irel->r_info) == R_RISCV_JAL
		      && !VALID_JTYPE_IMM (relocation)))
		return false;
	    }
	  where += 4;
	}
    }
  return true;
}

/* Shift a field of section content while doing target aligned.
   Like riscv_relax_delete_bytes, we need to adjust relocations
   and symbols in the field.  */

static bool
riscv_relax_shift_bytes (bfd *abfd, asection *sec, bfd_vma insn16_off,
			 bfd_vma nops_off, size_t count, uint32_t insn32)
{
  unsigned int i, symcount;
  struct elf_link_hash_entry **sym_hashes = elf_sym_hashes (abfd);
  Elf_Internal_Shdr *symtab_hdr = &elf_tdata (abfd)->symtab_hdr;
  unsigned int sec_shndx = _bfd_elf_section_from_bfd_section (abfd, sec);
  struct bfd_elf_section_data *data = elf_section_data (sec);
  bfd_byte *contents = data->this_hdr.contents;

  /* Shift the code, and then convert a 16-bit instruction into 32-bit.  */
  memmove (contents + insn16_off + 4, contents + insn16_off + 2, nops_off - insn16_off - 2);
  bfd_put_32 (abfd, insn32, contents + insn16_off);

  /* Adjust the location of all of the relocs.  */
  for (i = 0; i < sec->reloc_count; i++)
    if (data->relocs[i].r_offset > insn16_off
	&& data->relocs[i].r_offset <= nops_off)
      data->relocs[i].r_offset += count;

  /* Adjust the local symbols defined in this section.  */
  for (i = 0; i < symtab_hdr->sh_info; i++)
    {
      Elf_Internal_Sym *sym = (Elf_Internal_Sym *) symtab_hdr->contents + i;
      if (sym->st_shndx == sec_shndx)
	{
	  /* Adjust the symbol size if needed.  */
	  if (sym->st_value > insn16_off
	      && sym->st_value <= nops_off
	      && sym->st_value + sym->st_size > nops_off)
	    sym->st_size -= count;
	  else if (sym->st_value <= insn16_off
		   && sym->st_value + sym->st_size > insn16_off
		   && sym->st_value + sym->st_size <= nops_off)
	    sym->st_size += count;

	  /* If the symbol is in the range of memory we just shifted,
	     we have to adjust it's value.  */
	  if (sym->st_value > insn16_off && sym->st_value <= nops_off)
	    sym->st_value += count;
	}
    }

  /* Adjust the global symbols defined in this section.  */
  symcount = ((symtab_hdr->sh_size / sizeof (ElfNN_External_Sym))
	      - symtab_hdr->sh_info);

  for (i = 0; i < symcount; i++)
    {
      struct elf_link_hash_entry *sym_hash = sym_hashes[i];

      if ((sym_hash->root.type == bfd_link_hash_defined
	   || sym_hash->root.type == bfd_link_hash_defweak)
	  && sym_hash->root.u.def.section == sec)
	{
	  /* As above, adjust the size if needed.  */
	  if (sym_hash->root.u.def.value > insn16_off
	      && sym_hash->root.u.def.value <= nops_off
	      && sym_hash->root.u.def.value + sym_hash->size > nops_off)
	    sym_hash->size -= count;
	  else if (sym_hash->root.u.def.value <= insn16_off
		   && sym_hash->root.u.def.value + sym_hash->size > insn16_off
		   && sym_hash->root.u.def.value + sym_hash->size <= nops_off)
	    sym_hash->size += count;

	  /* As above, adjust the value if needed.  */
	  if (sym_hash->root.u.def.value > insn16_off
	      && sym_hash->root.u.def.value <= nops_off)
	    sym_hash->root.u.def.value += count;
	}
    }

  return true;
}

/* For avioding BTB miss, we need to convert a 16-bit insn to
   32-bit one (this insn is located between JAL and branch), and
   then adjust symbols for the insn.  */

static bool
riscv_relax_avoid_BTB_miss (bfd *abfd, asection *sec, Elf_Internal_Rela *rel)
{
  unsigned int i, symcount;
  struct elf_link_hash_entry **sym_hashes = elf_sym_hashes (abfd);
  Elf_Internal_Shdr *symtab_hdr = &elf_tdata (abfd)->symtab_hdr;
  unsigned int sec_shndx = _bfd_elf_section_from_bfd_section (abfd, sec);
  struct bfd_elf_section_data *data = elf_section_data (sec);
  bfd_byte *contents = data->this_hdr.contents;
  /* convert middle RVC into RVI.  */
  bfd_vma rvc_offset = rel->r_offset + rel->r_addend;
  bfd_vma rvi_offset = rel->r_offset;
  bfd_vma nop_bytes = rel->r_addend;

  BFD_ASSERT (IS_RVC_INSN (*(contents + rvc_offset)));

  while (true)
    {
      uint32_t insn;
      uint16_t insn16 = bfd_get_16 (abfd, contents + rvc_offset);
      /* Convert a 16-bit branch to 32-bit one doesn't help
	 to solved BTB miss.  */
      if (!(((ARCH_SIZE == 32) && ((insn16 & MASK_C_JAL) == MATCH_C_JAL))
	    || (insn16 & MASK_C_J) == MATCH_C_J
	    || (insn16 & MASK_C_BNEZ) == MATCH_C_BNEZ
	    || (insn16 & MASK_C_BEQZ) == MATCH_C_BEQZ)
	  && riscv_convert_16_to_32 (insn16, &insn))
	{
	  bfd_put_32 (abfd, insn, contents + rvi_offset);
	  /* Adjust the location of all of the relocs.  */
	  /* Maybe we should enhance the error msg here.  */
	  for (i = 0; i < sec->reloc_count; i++)
	    if (data->relocs[i].r_offset == rvc_offset)
	      {
		Elf_Internal_Rela *reloc = &(data->relocs[i]);
		riscv_convert_16_to_32_reloc (&reloc);
		data->relocs[i].r_offset -= nop_bytes;
	      }

	  for (i = 0; i < symtab_hdr->sh_info; i++)
	    {
	      Elf_Internal_Sym *sym = (Elf_Internal_Sym *) symtab_hdr->contents + i;
	      if (sym->st_shndx == sec_shndx)
		{ /* Adjust the symbol size if needed.  */
		  if (sym->st_value == rvc_offset)
		    sym->st_value -= nop_bytes;
		}
	    }

	  symcount = ((symtab_hdr->sh_size / sizeof (ElfNN_External_Sym))
		      - symtab_hdr->sh_info);
	  for (i = 0; i < symcount; i++)
	    {
	      struct elf_link_hash_entry *sym_hash = sym_hashes[i];

	      if ((sym_hash->root.type == bfd_link_hash_defined
		   || sym_hash->root.type == bfd_link_hash_defweak)
		  && sym_hash->root.u.def.section == sec)
		{
		  /* As above, adjust the value if needed.  */
		  if (sym_hash->root.u.def.value == rvc_offset)
		    sym_hash->root.u.def.value -= nop_bytes;
		}
	    }

	  /* rel->r_offset += 4; danger! relocations must be sorted!.  */
	  rel->r_addend -= 2;
	  return true;
	}
      break; /* once */
    }

  /* Can not avoid BTB miss, return false.  */
  return false;
}

static bool
btb_miss_occur (bfd_vma return_address, bfd_vma branch_end)
{
  if ((int)(return_address/4) == (int)(branch_end/4))
    return true;
  else
    return false;
}

/* offset:    to the begin of nop-bytes remained.
 * nop_bytes: number of nop bytes left.  */

static bool
riscv_relax_check_BTB_miss (bfd *abfd, asection *sec, Elf_Internal_Rela *rel)
{
  struct bfd_elf_section_data *data = elf_section_data (sec);
  bfd_byte *contents = data->this_hdr.contents;
  bfd_vma return_address, check_offset;
  unsigned int i;

  return_address = sec_addr (sec) + rel->r_offset;
  check_offset = rel->r_offset + rel->r_addend;

  /* The checking will overflow.  */
  if (check_offset + 4 >= sec->size)
    return false;

  /* The case ALIGN_BTB + ALIGN is hard to check BTB miss, skip it.  */
  /* nop  @ ALIGN_BTB  <-- return_address
   * next @ ALIGN      <-- check_offset
   */
  for (i = 0; i < sec->reloc_count; i++)
    if (data->relocs[i].r_offset == check_offset
	&& (ELFNN_R_TYPE (data->relocs[i].r_info) == R_RISCV_ALIGN
	    || ELFNN_R_TYPE (data->relocs[i].r_info) == R_RISCV_ALIGN_BTB))
      return false;

  /* check the next 2 insns.  */
  if (IS_RVC_INSN (*(contents + check_offset)))
    {
      bfd_vma branch_end = return_address + 2;
      if (IS_RVC_INSN (*(contents + check_offset + 2)))
	{
	  uint16_t insn16 = bfd_get_16 (abfd, contents + check_offset + 2);
	  if ((((ARCH_SIZE == 32) && (insn16 & MASK_C_JAL) == MATCH_C_JAL)
	       || (insn16 & MASK_C_JR) == MATCH_C_JR
	       || (insn16 & MASK_C_JALR) == MATCH_C_JALR
	       || (insn16 & MASK_C_J) == MATCH_C_J
	       || (insn16 & MASK_C_BEQZ) == MATCH_C_BEQZ
	       || (insn16 & MASK_C_BNEZ) == MATCH_C_BNEZ)
	      && btb_miss_occur (return_address, branch_end))
	    return true;
	}
      else
	{ /* 32-bit insn.  */

	  /* The checking will overflow.  */
	  if (check_offset + 6 >= sec->size)
	    return false;
	  uint32_t insn = bfd_get_32 (abfd, contents + check_offset + 2);
	  if (((insn & MASK_JAL) == MATCH_JAL
	       || (insn & MASK_JALR) == MATCH_JALR
	       || (insn & MASK_BEQ) == MATCH_BEQ
	       || (insn & MASK_BNE) == MATCH_BNE
	       || (insn & MASK_BLT) == MATCH_BLT
	       || (insn & MASK_BGE) == MATCH_BGE
	       || (insn & MASK_BLTU) == MATCH_BLTU
	       || (insn & MASK_BGEU) == MATCH_BGEU)
	      && btb_miss_occur (return_address, branch_end))
	    return true;
	}
    }

  return false;
}

static int
andes_try_target_align (bfd *abfd, asection *sec, asection *sym_sec,
			struct bfd_link_info *info, Elf_Internal_Rela *rel,
			bfd_vma offset, bfd_vma end)
{
  link_hash_table_t *htab = riscv_elf_hash_table (info);
  andes_ld_options_t *nopt = &htab->andes;
  bfd_byte *contents = elf_section_data (sec)->this_hdr.contents;
  bool rvc = elf_elfheader (abfd)->e_flags & EF_RISCV_RVC;

  if (!nopt->target_aligned || !rvc)
    return 0;

  {
    Elf_Internal_Rela *irel_save = NULL;
    Elf_Internal_Rela *relocs, *irelend, *irel;
    uint32_t insn = -1u;
    bfd_vma insn16_off = -1u;
    bfd_vma where = offset;
    int data_flag;

    if (elf_section_data (sec)->relocs)
      relocs = elf_section_data (sec)->relocs;
    else
      relocs = _bfd_elf_link_read_relocs (abfd, sec, NULL, NULL, true);

    irelend = relocs + sec->reloc_count;
    irel = relocs;
    while (where < end)
      { /* find the RCV candidate nearest by end.  */
	while (irel != NULL && irel < irelend && irel->r_offset < where)
	  irel++;

	data_flag = riscv_relocation_check (info, &irel, irelend,
					    sec, &where, contents, 0);
	if (data_flag & DATA_EXIST)
	  {
	    where += (data_flag >> 24);
	    continue;
	  }

	if (IS_RVC_INSN (*(contents + where)))
	  {
	    uint16_t insn16 = bfd_get_16 (abfd, contents + where);
	    if (riscv_convert_16_to_32 (insn16, &insn))
	      { /* keep the found candidate.  */
		insn16_off = where;
		irel_save = (irel->r_offset == where) ? irel : NULL;
	      }
	    where += 2; /* continue */
	  }
	else
	  where += 4;
      }

    /* convert the RVC candidate to RVI one if there is.  */
    if (insn != -1u && insn16_off != -1u
	&& target_align_check_branch_range (abfd, sec, insn16_off, offset,
					    2, info))
      { /* great, the candidate is qualified.
	 * don't forget the accompanying relocation it has.  */
	if (!riscv_convert_16_to_32_reloc (&irel_save))
	  {
	    (*_bfd_error_handler)
	      (_("%pB(%pA+0x%lx): Unsupported reloc %ld when converting "
		 "insn from 16-bit to 32-bit for target aligned"),
	       abfd, sym_sec, irel_save->r_offset,
	       ELFNN_R_TYPE (irel_save->r_info));
	    bfd_set_error (bfd_error_bad_value);
	    /* TODO: try next candidate.  */
	    return 0;
	  }
      }
    else
      return 0;

    /* shift and expand the candidate in place.  */
    riscv_relax_shift_bytes (abfd, sec, insn16_off, rel->r_offset, 2, insn);
  }

  return 2; /* bytes shifted */
}

static bool
andes_relax_fls_gp (
  bfd *abfd,
  asection *sec,
  asection *sym_sec ATTRIBUTE_UNUSED,
  struct bfd_link_info *info,
  Elf_Internal_Rela *rel,
  bfd_vma symval,
  bfd_vma max_alignment ATTRIBUTE_UNUSED,
  bfd_vma reserve_size ATTRIBUTE_UNUSED,
  bool *again ATTRIBUTE_UNUSED,
  //riscv_pcgp_relocs *pcgp_relocs ATTRIBUTE_UNUSED,
  void *pcgp_relocs ATTRIBUTE_UNUSED,
  bool undefined_weak ATTRIBUTE_UNUSED)
{
  bfd_vma gp = riscv_global_pointer_value (info);
  bfd_vma relocation = symval - gp;
  int type = ELFNN_R_TYPE (rel->r_info);
  if (type == R_RISCV_LO12_I || type == R_RISCV_PCREL_LO12_I)
    {
      BFD_ASSERT (VALID_ITYPE_IMM (relocation));
      bfd_byte *contents = elf_section_data (sec)->this_hdr.contents
			   + rel->r_offset;
      uint32_t insn32 = bfd_get_32 (abfd, contents);
      insn32 = (insn32 & ~ENCODE_ITYPE_IMM (-1)) | ENCODE_ITYPE_IMM(relocation);
      bfd_put_32 (abfd, insn32, contents);
    }
  else if (type == R_RISCV_LO12_S || type == R_RISCV_PCREL_LO12_S)
    {
      BFD_ASSERT (VALID_STYPE_IMM (relocation));
      bfd_byte *contents = elf_section_data (sec)->this_hdr.contents
			   + rel->r_offset;
      uint32_t insn32 = bfd_get_32 (abfd, contents);
      insn32 = (insn32 & ~ENCODE_STYPE_IMM (-1)) | ENCODE_STYPE_IMM(relocation);
      bfd_put_32 (abfd, insn32, contents);
    }
  else
    BFD_ASSERT (0);

  rel->r_info = ELFNN_R_INFO (0, R_RISCV_NONE);
  return true;
}
/* } Andes  */

#define TARGET_LITTLE_SYM			riscv_elfNN_vec
#define TARGET_LITTLE_NAME			"elfNN-littleriscv"
#define TARGET_BIG_SYM				riscv_elfNN_be_vec
#define TARGET_BIG_NAME				"elfNN-bigriscv"

#define elf_backend_reloc_type_class		riscv_reloc_type_class

#define bfd_elfNN_bfd_reloc_name_lookup		riscv_reloc_name_lookup
#define bfd_elfNN_bfd_link_hash_table_create	riscv_elf_link_hash_table_create
#define bfd_elfNN_bfd_reloc_type_lookup		riscv_reloc_type_lookup
#define bfd_elfNN_bfd_merge_private_bfd_data \
  _bfd_riscv_elf_merge_private_bfd_data
#define bfd_elfNN_bfd_is_target_special_symbol	riscv_elf_is_target_special_symbol

#define elf_backend_copy_indirect_symbol	riscv_elf_copy_indirect_symbol
#define elf_backend_create_dynamic_sections	riscv_elf_create_dynamic_sections
#define elf_backend_check_relocs		riscv_elf_check_relocs
#define elf_backend_adjust_dynamic_symbol	riscv_elf_adjust_dynamic_symbol
#define elf_backend_size_dynamic_sections	riscv_elf_size_dynamic_sections
#define elf_backend_relocate_section		riscv_elf_relocate_section
#define elf_backend_finish_dynamic_symbol	riscv_elf_finish_dynamic_symbol
#define elf_backend_finish_dynamic_sections	riscv_elf_finish_dynamic_sections
#define elf_backend_gc_mark_hook		riscv_elf_gc_mark_hook
#define elf_backend_plt_sym_val			riscv_elf_plt_sym_val
#define elf_backend_grok_prstatus		riscv_elf_grok_prstatus
#define elf_backend_grok_psinfo			riscv_elf_grok_psinfo
#define elf_backend_object_p			riscv_elf_object_p
#define elf_backend_write_core_note		riscv_write_core_note
#define elf_backend_maybe_function_sym		riscv_maybe_function_sym
#define elf_info_to_howto_rel			NULL
#define elf_info_to_howto			riscv_info_to_howto_rela
#define bfd_elfNN_bfd_relax_section		_bfd_riscv_relax_section
#define bfd_elfNN_mkobject			elfNN_riscv_mkobject
#define bfd_elfNN_bfd_final_link		riscv_final_link
#define elf_backend_additional_program_headers \
  riscv_elf_additional_program_headers
#define elf_backend_modify_segment_map		riscv_elf_modify_segment_map
#define elf_backend_merge_symbol_attribute	riscv_elf_merge_symbol_attribute

#define elf_backend_init_index_section		_bfd_elf_init_1_index_section

/* { Andes */
#define elf_backend_link_output_symbol_hook  riscv_elf_output_symbol_hook
#define elf_backend_output_arch_syms	     riscv_elf_output_arch_syms
#define elf_backend_final_write_processing   riscv_elf_final_write_processing
/* } Andes */

#define elf_backend_can_gc_sections		1
#define elf_backend_can_refcount		1
#define elf_backend_want_got_plt		1
#define elf_backend_plt_readonly		1
#define elf_backend_plt_alignment		4
#define elf_backend_want_plt_sym		1
#define elf_backend_got_header_size		(ARCH_SIZE / 8)
#define elf_backend_want_dynrelro		1
#define elf_backend_rela_normal			1
#define elf_backend_default_execstack		0

#undef  elf_backend_obj_attrs_vendor
#define elf_backend_obj_attrs_vendor		"riscv"
#undef  elf_backend_obj_attrs_arg_type
#define elf_backend_obj_attrs_arg_type		riscv_elf_obj_attrs_arg_type
#undef  elf_backend_obj_attrs_section_type
#define elf_backend_obj_attrs_section_type	SHT_RISCV_ATTRIBUTES
#undef  elf_backend_obj_attrs_section
#define elf_backend_obj_attrs_section		RISCV_ATTRIBUTES_SECTION_NAME

#include "elfNN-target.h"
