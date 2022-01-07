/* RISC-V-specific support for ELF.
   Copyright (C) 2011-2019 Free Software Foundation, Inc.

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

#include "sysdep.h"
#include "bfd.h"
#include "libbfd.h"
#include "elf-bfd.h"
#include "elf/riscv.h"
#include "opcode/riscv.h"
#include "libiberty.h"
#include "elfxx-riscv.h"
#include "safe-ctype.h"

#define MINUS_ONE ((bfd_vma)0 - 1)

/* Special handler for ADD/SUB relocations that allows them to be filled out
   both in the pre-linked and post-linked file.  This is necessary to make
   pre-linked debug info work, as due to linker relaxations we need to emit
   relocations for the debug info.  */
static bfd_reloc_status_type riscv_elf_add_sub_reloc
  (bfd *, arelent *, asymbol *, void *, asection *, bfd *, char **);

/* The relocation table used for SHT_RELA sections.  */

static reloc_howto_type howto_table[] =
{
  /* No relocation.  */
  HOWTO (R_RISCV_NONE,			/* type */
	 0,				/* rightshift */
	 3,				/* size */
	 0,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_NONE",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 0,				/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* 32 bit relocation.  */
  HOWTO (R_RISCV_32,			/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_32",			/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 MINUS_ONE,			/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* 64 bit relocation.  */
  HOWTO (R_RISCV_64,			/* type */
	 0,				/* rightshift */
	 4,				/* size */
	 64,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_64",			/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 MINUS_ONE,			/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* Relocation against a local symbol in a shared object.  */
  HOWTO (R_RISCV_RELATIVE,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_RELATIVE",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 MINUS_ONE,			/* dst_mask */
	 FALSE),			/* pcrel_offset */

  HOWTO (R_RISCV_COPY,			/* type */
	 0,				/* rightshift */
	 0,				/* this one is variable size */
	 0,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_bitfield,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_COPY",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 0,				/* dst_mask */
	 FALSE),			/* pcrel_offset */

  HOWTO (R_RISCV_JUMP_SLOT,		/* type */
	 0,				/* rightshift */
	 4,				/* size */
	 64,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_bitfield,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_JUMP_SLOT",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 0,				/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* Dynamic TLS relocations.  */
  HOWTO (R_RISCV_TLS_DTPMOD32,		/* type */
	 0,				/* rightshift */
	 4,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_TLS_DTPMOD32",	/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 MINUS_ONE,			/* dst_mask */
	 FALSE),			/* pcrel_offset */

  HOWTO (R_RISCV_TLS_DTPMOD64,		/* type */
	 0,				/* rightshift */
	 4,				/* size */
	 64,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_TLS_DTPMOD64",	/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 MINUS_ONE,			/* dst_mask */
	 FALSE),			/* pcrel_offset */

  HOWTO (R_RISCV_TLS_DTPREL32,		/* type */
	 0,				/* rightshift */
	 4,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_TLS_DTPREL32",	/* name */
	 TRUE,				/* partial_inplace */
	 0,				/* src_mask */
	 MINUS_ONE,			/* dst_mask */
	 FALSE),			/* pcrel_offset */

  HOWTO (R_RISCV_TLS_DTPREL64,		/* type */
	 0,				/* rightshift */
	 4,				/* size */
	 64,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_TLS_DTPREL64",	/* name */
	 TRUE,				/* partial_inplace */
	 0,				/* src_mask */
	 MINUS_ONE,			/* dst_mask */
	 FALSE),			/* pcrel_offset */

  HOWTO (R_RISCV_TLS_TPREL32,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_TLS_TPREL32",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 MINUS_ONE,			/* dst_mask */
	 FALSE),			/* pcrel_offset */

  HOWTO (R_RISCV_TLS_TPREL64,		/* type */
	 0,				/* rightshift */
	 4,				/* size */
	 64,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_TLS_TPREL64",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 MINUS_ONE,			/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* Reserved for future relocs that the dynamic linker must understand.  */
  EMPTY_HOWTO (12),
  EMPTY_HOWTO (13),
  EMPTY_HOWTO (14),
  EMPTY_HOWTO (15),

  /* 12-bit PC-relative branch offset.  */
  HOWTO (R_RISCV_BRANCH,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 TRUE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_signed,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_BRANCH",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_SBTYPE_IMM (-1U),	/* dst_mask */
	 TRUE),				/* pcrel_offset */

  /* 20-bit PC-relative jump offset.  */
  HOWTO (R_RISCV_JAL,			/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 TRUE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_JAL",			/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_UJTYPE_IMM (-1U),	/* dst_mask */
	 TRUE),				/* pcrel_offset */

  /* 32-bit PC-relative function call (AUIPC/JALR).  */
  HOWTO (R_RISCV_CALL,			/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 64,				/* bitsize */
	 TRUE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_CALL",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_UTYPE_IMM (-1U) | ((bfd_vma) ENCODE_ITYPE_IMM (-1U) << 32),
					/* dst_mask */
	 TRUE),				/* pcrel_offset */

  /* Like R_RISCV_CALL, but not locally binding.  */
  HOWTO (R_RISCV_CALL_PLT,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 64,				/* bitsize */
	 TRUE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_CALL_PLT",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_UTYPE_IMM (-1U) | ((bfd_vma) ENCODE_ITYPE_IMM (-1U) << 32),
					/* dst_mask */
	 TRUE),				/* pcrel_offset */

  /* High 20 bits of 32-bit PC-relative GOT access.  */
  HOWTO (R_RISCV_GOT_HI20,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 TRUE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_GOT_HI20",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_UTYPE_IMM (-1U),	/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* High 20 bits of 32-bit PC-relative TLS IE GOT access.  */
  HOWTO (R_RISCV_TLS_GOT_HI20,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 TRUE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_TLS_GOT_HI20",	/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_UTYPE_IMM (-1U),	/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* High 20 bits of 32-bit PC-relative TLS GD GOT reference.  */
  HOWTO (R_RISCV_TLS_GD_HI20,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 TRUE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_TLS_GD_HI20",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_UTYPE_IMM (-1U),	/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* High 20 bits of 32-bit PC-relative reference.  */
  HOWTO (R_RISCV_PCREL_HI20,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 TRUE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_PCREL_HI20",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_UTYPE_IMM (-1U),	/* dst_mask */
	 TRUE),				/* pcrel_offset */

  /* Low 12 bits of a 32-bit PC-relative load or add.  */
  HOWTO (R_RISCV_PCREL_LO12_I,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_PCREL_LO12_I",	/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_ITYPE_IMM (-1U),	/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* Low 12 bits of a 32-bit PC-relative store.  */
  HOWTO (R_RISCV_PCREL_LO12_S,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_PCREL_LO12_S",	/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_STYPE_IMM (-1U),	/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* High 20 bits of 32-bit absolute address.  */
  HOWTO (R_RISCV_HI20,			/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_HI20",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_UTYPE_IMM (-1U),	/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* High 12 bits of 32-bit load or add.  */
  HOWTO (R_RISCV_LO12_I,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_LO12_I",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_ITYPE_IMM (-1U),	/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* High 12 bits of 32-bit store.  */
  HOWTO (R_RISCV_LO12_S,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_LO12_S",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_STYPE_IMM (-1U),	/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* High 20 bits of TLS LE thread pointer offset.  */
  HOWTO (R_RISCV_TPREL_HI20,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_signed,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_TPREL_HI20",		/* name */
	 TRUE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_UTYPE_IMM (-1U),	/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* Low 12 bits of TLS LE thread pointer offset for loads and adds.  */
  HOWTO (R_RISCV_TPREL_LO12_I,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_signed,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_TPREL_LO12_I",	/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_ITYPE_IMM (-1U),	/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* Low 12 bits of TLS LE thread pointer offset for stores.  */
  HOWTO (R_RISCV_TPREL_LO12_S,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_signed,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_TPREL_LO12_S",	/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_STYPE_IMM (-1U),	/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* TLS LE thread pointer usage.  May be relaxed.  */
  HOWTO (R_RISCV_TPREL_ADD,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_TPREL_ADD",		/* name */
	 TRUE,				/* partial_inplace */
	 0,				/* src_mask */
	 0,				/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* 8-bit in-place addition, for local label subtraction.  */
  HOWTO (R_RISCV_ADD8,			/* type */
	 0,				/* rightshift */
	 0,				/* size */
	 8,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 riscv_elf_add_sub_reloc,	/* special_function */
	 "R_RISCV_ADD8",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 MINUS_ONE,			/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* 16-bit in-place addition, for local label subtraction.  */
  HOWTO (R_RISCV_ADD16,			/* type */
	 0,				/* rightshift */
	 1,				/* size */
	 16,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 riscv_elf_add_sub_reloc,	/* special_function */
	 "R_RISCV_ADD16",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 MINUS_ONE,			/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* 32-bit in-place addition, for local label subtraction.  */
  HOWTO (R_RISCV_ADD32,			/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 riscv_elf_add_sub_reloc,	/* special_function */
	 "R_RISCV_ADD32",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 MINUS_ONE,			/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* 64-bit in-place addition, for local label subtraction.  */
  HOWTO (R_RISCV_ADD64,			/* type */
	 0,				/* rightshift */
	 4,				/* size */
	 64,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 riscv_elf_add_sub_reloc,	/* special_function */
	 "R_RISCV_ADD64",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 MINUS_ONE,			/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* 8-bit in-place addition, for local label subtraction.  */
  HOWTO (R_RISCV_SUB8,			/* type */
	 0,				/* rightshift */
	 0,				/* size */
	 8,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 riscv_elf_add_sub_reloc,	/* special_function */
	 "R_RISCV_SUB8",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 MINUS_ONE,			/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* 16-bit in-place addition, for local label subtraction.  */
  HOWTO (R_RISCV_SUB16,			/* type */
	 0,				/* rightshift */
	 1,				/* size */
	 16,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 riscv_elf_add_sub_reloc,	/* special_function */
	 "R_RISCV_SUB16",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 MINUS_ONE,			/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* 32-bit in-place addition, for local label subtraction.  */
  HOWTO (R_RISCV_SUB32,			/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 riscv_elf_add_sub_reloc,	/* special_function */
	 "R_RISCV_SUB32",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 MINUS_ONE,			/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* 64-bit in-place addition, for local label subtraction.  */
  HOWTO (R_RISCV_SUB64,			/* type */
	 0,				/* rightshift */
	 4,				/* size */
	 64,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 riscv_elf_add_sub_reloc,	/* special_function */
	 "R_RISCV_SUB64",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 MINUS_ONE,			/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* GNU extension to record C++ vtable hierarchy */
  HOWTO (R_RISCV_GNU_VTINHERIT,		/* type */
	 0,				/* rightshift */
	 4,				/* size */
	 0,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 NULL,				/* special_function */
	 "R_RISCV_GNU_VTINHERIT",	/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 0,				/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* GNU extension to record C++ vtable member usage */
  HOWTO (R_RISCV_GNU_VTENTRY,		/* type */
	 0,				/* rightshift */
	 4,				/* size */
	 0,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 _bfd_elf_rel_vtable_reloc_fn,	/* special_function */
	 "R_RISCV_GNU_VTENTRY",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 0,				/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* Indicates an alignment statement.  The addend field encodes how many
     bytes of NOPs follow the statement.  The desired alignment is the
     addend rounded up to the next power of two.  */
  HOWTO (R_RISCV_ALIGN,			/* type */
	 0,				/* rightshift */
	 3,				/* size */
	 0,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_ALIGN",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 0,				/* dst_mask */
	 TRUE),				/* pcrel_offset */

  /* 8-bit PC-relative branch offset.  */
  HOWTO (R_RISCV_RVC_BRANCH,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 TRUE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_signed,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_RVC_BRANCH",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_RVC_B_IMM (-1U),	/* dst_mask */
	 TRUE),				/* pcrel_offset */

  /* 11-bit PC-relative jump offset.  */
  HOWTO (R_RISCV_RVC_JUMP,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 TRUE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_RVC_JUMP",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_RVC_J_IMM (-1U),	/* dst_mask */
	 TRUE),				/* pcrel_offset */

  /* High 6 bits of 18-bit absolute address.  */
  HOWTO (R_RISCV_RVC_LUI,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_RVC_LUI",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_RVC_IMM (-1U),		/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* GP-relative load.  */
  HOWTO (R_RISCV_GPREL_I,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_GPREL_I",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_ITYPE_IMM (-1U),	/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* GP-relative store.  */
  HOWTO (R_RISCV_GPREL_S,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_GPREL_S",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_STYPE_IMM (-1U),	/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* TP-relative TLS LE load.  */
  HOWTO (R_RISCV_TPREL_I,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_signed,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_TPREL_I",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_ITYPE_IMM (-1U),	/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* TP-relative TLS LE store.  */
  HOWTO (R_RISCV_TPREL_S,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_signed,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_TPREL_S",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_STYPE_IMM (-1U),	/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* The paired relocation may be relaxed.  */
  HOWTO (R_RISCV_RELAX,			/* type */
	 0,				/* rightshift */
	 3,				/* size */
	 0,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_RELAX",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 0,				/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* 6-bit in-place addition, for local label subtraction.  */
  HOWTO (R_RISCV_SUB6,			/* type */
	 0,				/* rightshift */
	 0,				/* size */
	 8,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 riscv_elf_add_sub_reloc,	/* special_function */
	 "R_RISCV_SUB6",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 0x3f,				/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* 6-bit in-place setting, for local label subtraction.  */
  HOWTO (R_RISCV_SET6,			/* type */
	 0,				/* rightshift */
	 0,				/* size */
	 8,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_SET6",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 0x3f,				/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* 8-bit in-place setting, for local label subtraction.  */
  HOWTO (R_RISCV_SET8,			/* type */
	 0,				/* rightshift */
	 0,				/* size */
	 8,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_SET8",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 MINUS_ONE,			/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* 16-bit in-place setting, for local label subtraction.  */
  HOWTO (R_RISCV_SET16,			/* type */
	 0,				/* rightshift */
	 1,				/* size */
	 16,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_SET16",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 MINUS_ONE,			/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* 32-bit in-place setting, for local label subtraction.  */
  HOWTO (R_RISCV_SET32,			/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_SET32",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 MINUS_ONE,			/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* 32-bit PC relative.  */
  HOWTO (R_RISCV_32_PCREL,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 TRUE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_32_PCREL",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 MINUS_ONE,			/* dst_mask */
	 FALSE),			/* pcrel_offset */

  EMPTY_HOWTO (58), EMPTY_HOWTO (59), EMPTY_HOWTO (60), EMPTY_HOWTO (61),
  EMPTY_HOWTO (62), EMPTY_HOWTO (63), EMPTY_HOWTO (64), EMPTY_HOWTO (65),
  EMPTY_HOWTO (66), EMPTY_HOWTO (67), EMPTY_HOWTO (68), EMPTY_HOWTO (69),
  EMPTY_HOWTO (70), EMPTY_HOWTO (71), EMPTY_HOWTO (72), EMPTY_HOWTO (73),
  EMPTY_HOWTO (74), EMPTY_HOWTO (75), EMPTY_HOWTO (76), EMPTY_HOWTO (77),
  EMPTY_HOWTO (78), EMPTY_HOWTO (79), EMPTY_HOWTO (80), EMPTY_HOWTO (81),
  EMPTY_HOWTO (82), EMPTY_HOWTO (83), EMPTY_HOWTO (84), EMPTY_HOWTO (85),
  EMPTY_HOWTO (86), EMPTY_HOWTO (87), EMPTY_HOWTO (88), EMPTY_HOWTO (89),
  EMPTY_HOWTO (90), EMPTY_HOWTO (91), EMPTY_HOWTO (92), EMPTY_HOWTO (93),
  EMPTY_HOWTO (94), EMPTY_HOWTO (95), EMPTY_HOWTO (96), EMPTY_HOWTO (97),
  EMPTY_HOWTO (98), EMPTY_HOWTO (99), EMPTY_HOWTO (100), EMPTY_HOWTO (101),
  EMPTY_HOWTO (102), EMPTY_HOWTO (103), EMPTY_HOWTO (104), EMPTY_HOWTO (105),
  EMPTY_HOWTO (106), EMPTY_HOWTO (107), EMPTY_HOWTO (108), EMPTY_HOWTO (109),
  EMPTY_HOWTO (110), EMPTY_HOWTO (111), EMPTY_HOWTO (112), EMPTY_HOWTO (113),
  EMPTY_HOWTO (114), EMPTY_HOWTO (115), EMPTY_HOWTO (116), EMPTY_HOWTO (117),
  EMPTY_HOWTO (118), EMPTY_HOWTO (119), EMPTY_HOWTO (120), EMPTY_HOWTO (121),
  EMPTY_HOWTO (122), EMPTY_HOWTO (123), EMPTY_HOWTO (124), EMPTY_HOWTO (125),
  EMPTY_HOWTO (126), EMPTY_HOWTO (127), EMPTY_HOWTO (128), EMPTY_HOWTO (129),
  EMPTY_HOWTO (130), EMPTY_HOWTO (131), EMPTY_HOWTO (132), EMPTY_HOWTO (133),
  EMPTY_HOWTO (134), EMPTY_HOWTO (135), EMPTY_HOWTO (136), EMPTY_HOWTO (137),
  EMPTY_HOWTO (138), EMPTY_HOWTO (139), EMPTY_HOWTO (140), EMPTY_HOWTO (141),
  EMPTY_HOWTO (142), EMPTY_HOWTO (143), EMPTY_HOWTO (144), EMPTY_HOWTO (145),
  EMPTY_HOWTO (146), EMPTY_HOWTO (147), EMPTY_HOWTO (148), EMPTY_HOWTO (149),
  EMPTY_HOWTO (150), EMPTY_HOWTO (151), EMPTY_HOWTO (152), EMPTY_HOWTO (153),
  EMPTY_HOWTO (154), EMPTY_HOWTO (155), EMPTY_HOWTO (156), EMPTY_HOWTO (157),
  EMPTY_HOWTO (158), EMPTY_HOWTO (159), EMPTY_HOWTO (160), EMPTY_HOWTO (161),
  EMPTY_HOWTO (162), EMPTY_HOWTO (163), EMPTY_HOWTO (164), EMPTY_HOWTO (165),
  EMPTY_HOWTO (166), EMPTY_HOWTO (167), EMPTY_HOWTO (168), EMPTY_HOWTO (169),
  EMPTY_HOWTO (170), EMPTY_HOWTO (171), EMPTY_HOWTO (172), EMPTY_HOWTO (173),
  EMPTY_HOWTO (174), EMPTY_HOWTO (175), EMPTY_HOWTO (176), EMPTY_HOWTO (177),
  EMPTY_HOWTO (178), EMPTY_HOWTO (179), EMPTY_HOWTO (180), EMPTY_HOWTO (181),
  EMPTY_HOWTO (182), EMPTY_HOWTO (183), EMPTY_HOWTO (184), EMPTY_HOWTO (185),
  EMPTY_HOWTO (186), EMPTY_HOWTO (187), EMPTY_HOWTO (188), EMPTY_HOWTO (189),
  EMPTY_HOWTO (190), EMPTY_HOWTO (191), EMPTY_HOWTO (192), EMPTY_HOWTO (193),
  EMPTY_HOWTO (194), EMPTY_HOWTO (195), EMPTY_HOWTO (196), EMPTY_HOWTO (197),
  EMPTY_HOWTO (198), EMPTY_HOWTO (199), EMPTY_HOWTO (200), EMPTY_HOWTO (201),
  EMPTY_HOWTO (202), EMPTY_HOWTO (203), EMPTY_HOWTO (204), EMPTY_HOWTO (205),
  EMPTY_HOWTO (206), EMPTY_HOWTO (207), EMPTY_HOWTO (208), EMPTY_HOWTO (209),
  EMPTY_HOWTO (210), EMPTY_HOWTO (211), EMPTY_HOWTO (212), EMPTY_HOWTO (213),
  EMPTY_HOWTO (214), EMPTY_HOWTO (215), EMPTY_HOWTO (216), EMPTY_HOWTO (217),
  EMPTY_HOWTO (218), EMPTY_HOWTO (219), EMPTY_HOWTO (220), EMPTY_HOWTO (221),
  EMPTY_HOWTO (222), EMPTY_HOWTO (223), EMPTY_HOWTO (224), EMPTY_HOWTO (225),
  EMPTY_HOWTO (226), EMPTY_HOWTO (227), EMPTY_HOWTO (228), EMPTY_HOWTO (229),

  /* Relocations for NDS V5.  */
  /* Andes misc. info. (alignment info so far)  */
  HOWTO (R_RISCV_ANDES_MISC,		/* type */
         0,				/* rightshift */
         0,				/* size */
         0,				/* bitsize */
         FALSE,				/* pc_relative */
         0,				/* bitpos */
         complain_overflow_dont,	/* complain_on_overflow */
         bfd_elf_generic_reloc,		/* special_function */
         "R_RISCV_ANDES_MISC",		/* name */
         FALSE,				/* partial_inplace */
         0,				/* src_mask */
         0,				/* dst_mask */
         FALSE),			/* pcrel_offset */

  /* EXEC.IT itable entry index.  */
  HOWTO (R_RISCV_EXECIT_ITE,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 16,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_EXECIT_ITE",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_RVC_EXECIT_IMM (-1U),	/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* Jump-patch table relocations.  */
  /* High 20 bits of 32-bit 32-bit absolute address for jump-patch table.  */
  HOWTO (R_RISCV_ICT_HI20,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_ICT_HI20",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_UTYPE_IMM (-1U),	/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* High 12 bits of 32-bit load or add for jump-patch table.  */
  HOWTO (R_RISCV_ICT_LO12_I,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_ICT_LO12_I",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_ITYPE_IMM (-1U),	/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* High 20 bits of 32-bit PC-relative reference for jump-patch table.  */
  HOWTO (R_RISCV_PCREL_ICT_HI20,	/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 TRUE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_PCREL_ICT_HI20",	/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_UTYPE_IMM (-1U),	/* dst_mask */
	 TRUE),				/* pcrel_offset */

  /* 32-bit PC-relative function call (AUIPC/JALR) for jump-patch table.  */
  HOWTO (R_RISCV_CALL_ICT,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 64,				/* bitsize */
	 TRUE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_CALL_ICT",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_UTYPE_IMM (-1U) | ((bfd_vma) ENCODE_ITYPE_IMM (-1U) << 32),
	 /* dst_mask */
	 TRUE),				/* pcrel_offset */

  /* 64 bit relocation for jump-patch table.  */
  HOWTO (R_RISCV_ICT_64,		/* type */
	 0,				/* rightshift */
	 4,				/* size */
	 64,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_ICT_64",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 MINUS_ONE,			/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* Mark the begin of the region that can not do RVC relaxations.  */
  HOWTO (R_RISCV_NO_RVC_REGION_BEGIN,	/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_NO_RVC_REGION_BEGIN",	/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 MINUS_ONE,			/* dst_mask */
	 FALSE),			/* pcrel_offset */
  /* Mark the end of the region that can not do RVC relaxations.  */
  HOWTO (R_RISCV_NO_RVC_REGION_END,	/* type */
         0,				/* rightshift */
         2,				/* size */
         32,				/* bitsize */
         FALSE,				/* pc_relative */
         0,				/* bitpos */
         complain_overflow_dont,	/* complain_on_overflow */
         bfd_elf_generic_reloc,		/* special_function */
         "R_RISCV_NO_RVC_REGION_END",	/* name */
         FALSE,				/* partial_inplace */
         0,				/* src_mask */
         MINUS_ONE,			/* dst_mask */
         FALSE),			/* pcrel_offset */

  /* Deleting the unused insn for pc to gp relaxation.
     This is defined to 256 (R_RISCV_max + 1) originally in the elfnn-riscv.c
     for internal relocations used exclusively by the relaxation pass.
     Unfortunately, the macro ELFNN_R_TYPE will get the unexpected value for
     R_RISCV_DELETE. I redefine it here to solve the problem.  */
  HOWTO (R_RISCV_DELETE,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 0,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_DELETE",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 0,				/* dst_mask */
	 TRUE),				/* pcrel_offset */

  /* For handling alignment and BTB miss */
  HOWTO (R_RISCV_ALIGN_BTB,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 0,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_ALIGN_BTB",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 0,				/* dst_mask */
	 TRUE),				/* pcrel_offset */

  /* 10-bit PC-relative branch offset.  */
  HOWTO (R_RISCV_10_PCREL,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 TRUE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_signed,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_10_PCREL",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_STYPE_IMM10 (-1U),	/* dst_mask */
	 TRUE),				/* pcrel_offset */

  /* Avoid linker optimizations replacing data in text.  */
  HOWTO (R_RISCV_DATA,			/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_DATA",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 MINUS_ONE,			/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* High 20 bits of low 32-bit absolute address for 64-bit symbol.  */
  HOWTO (R_RISCV_LALO_HI20,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_LALO_HI20",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_UTYPE_IMM (-1U),	/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /*  Low 12 bits of low 32-bit absolute address for 64-bit symbol. */
  HOWTO (R_RISCV_LALO_LO12_I,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_LALO_LO12_I",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_ITYPE_IMM (-1U),	/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* Mark which section can do extra linker optimization (like EXECIT).  */
  HOWTO (R_RISCV_RELAX_ENTRY,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_dont,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_RELAX_ENTRY",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 MINUS_ONE,			/* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* 18-bit gp-relative load offset.  */
  HOWTO (R_RISCV_LGP18S0,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_signed,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_LGP18S0",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_GPTYPE_LB_IMM (-1U),    /* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* 17-bit and right shift 1bit gp-relative load offset.  */
  HOWTO (R_RISCV_LGP17S1,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_signed,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_LGP17S1",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_GPTYPE_LH_IMM (-1U),    /* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* 16-bit and right shift 2bit gp-relative load offset.  */
  HOWTO (R_RISCV_LGP17S2,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_signed,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_LGP17S2",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_GPTYPE_LW_IMM (-1U),    /* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* 15-bit and right shift 3bit gp-relative load offset.  */
  HOWTO (R_RISCV_LGP17S3,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_signed,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_LGP17S3",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_GPTYPE_LD_IMM (-1U),    /* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* 18-bit gp-relative store offset.  */
  HOWTO (R_RISCV_SGP18S0,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_signed,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_SGP18S0",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_GPTYPE_SB_IMM (-1U),    /* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* 17-bit and right shift 1bit gp-relative store offset.  */
  HOWTO (R_RISCV_SGP17S1,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_signed,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_SGP17S1",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_GPTYPE_SH_IMM (-1U),    /* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* 16-bit and right shift 2bit gp-relative store offset.  */
  HOWTO (R_RISCV_SGP17S2,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_signed,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_SGP17S2",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_GPTYPE_SW_IMM (-1U),    /* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* 15-bit and right shift 3bit gp-relative store offset.  */
  HOWTO (R_RISCV_SGP17S3,		/* type */
	 0,				/* rightshift */
	 2,				/* size */
	 32,				/* bitsize */
	 FALSE,				/* pc_relative */
	 0,				/* bitpos */
	 complain_overflow_signed,	/* complain_on_overflow */
	 bfd_elf_generic_reloc,		/* special_function */
	 "R_RISCV_SGP17S3",		/* name */
	 FALSE,				/* partial_inplace */
	 0,				/* src_mask */
	 ENCODE_GPTYPE_SD_IMM (-1U),    /* dst_mask */
	 FALSE),			/* pcrel_offset */

  /* Mark the begin of the region that can not do some linker relaxations.  */
  HOWTO (R_RISCV_RELAX_REGION_BEGIN,	/* type */
         0,				/* rightshift */
         2,				/* size */
         32,				/* bitsize */
         FALSE,				/* pc_relative */
         0,				/* bitpos */
         complain_overflow_dont,	/* complain_on_overflow */
         bfd_elf_generic_reloc,		/* special_function */
         "R_RISCV_RELAX_REGION_BEGIN",	/* name */
         FALSE,				/* partial_inplace */
         0,				/* src_mask */
         MINUS_ONE,			/* dst_mask */
         FALSE),			/* pcrel_offset */

  /* Mark the end of the region that can not do some linker relaxations.  */
  HOWTO (R_RISCV_RELAX_REGION_END,	/* type */
         0,				/* rightshift */
         2,				/* size */
         32,				/* bitsize */
         FALSE,				/* pc_relative */
         0,				/* bitpos */
         complain_overflow_dont,	/* complain_on_overflow */
         bfd_elf_generic_reloc,		/* special_function */
         "R_RISCV_RELAX_REGION_END",	/* name */
         FALSE,				/* partial_inplace */
         0,				/* src_mask */
         MINUS_ONE,			/* dst_mask */
         FALSE),			/* pcrel_offset */
};

/* A mapping from BFD reloc types to RISC-V ELF reloc types.  */

struct elf_reloc_map
{
  bfd_reloc_code_real_type bfd_val;
  enum elf_riscv_reloc_type elf_val;
};

static const struct elf_reloc_map riscv_reloc_map[] =
{
  { BFD_RELOC_NONE, R_RISCV_NONE },
  { BFD_RELOC_32, R_RISCV_32 },
  { BFD_RELOC_64, R_RISCV_64 },
  { BFD_RELOC_RISCV_ADD8, R_RISCV_ADD8 },
  { BFD_RELOC_RISCV_ADD16, R_RISCV_ADD16 },
  { BFD_RELOC_RISCV_ADD32, R_RISCV_ADD32 },
  { BFD_RELOC_RISCV_ADD64, R_RISCV_ADD64 },
  { BFD_RELOC_RISCV_SUB8, R_RISCV_SUB8 },
  { BFD_RELOC_RISCV_SUB16, R_RISCV_SUB16 },
  { BFD_RELOC_RISCV_SUB32, R_RISCV_SUB32 },
  { BFD_RELOC_RISCV_SUB64, R_RISCV_SUB64 },
  { BFD_RELOC_CTOR, R_RISCV_64 },
  { BFD_RELOC_12_PCREL, R_RISCV_BRANCH },
  { BFD_RELOC_RISCV_HI20, R_RISCV_HI20 },
  { BFD_RELOC_RISCV_LO12_I, R_RISCV_LO12_I },
  { BFD_RELOC_RISCV_LO12_S, R_RISCV_LO12_S },
  { BFD_RELOC_RISCV_PCREL_LO12_I, R_RISCV_PCREL_LO12_I },
  { BFD_RELOC_RISCV_PCREL_LO12_S, R_RISCV_PCREL_LO12_S },
  { BFD_RELOC_RISCV_CALL, R_RISCV_CALL },
  { BFD_RELOC_RISCV_CALL_PLT, R_RISCV_CALL_PLT },
  { BFD_RELOC_RISCV_PCREL_HI20, R_RISCV_PCREL_HI20 },
  { BFD_RELOC_RISCV_JMP, R_RISCV_JAL },
  { BFD_RELOC_RISCV_GOT_HI20, R_RISCV_GOT_HI20 },
  { BFD_RELOC_RISCV_TLS_DTPMOD32, R_RISCV_TLS_DTPMOD32 },
  { BFD_RELOC_RISCV_TLS_DTPREL32, R_RISCV_TLS_DTPREL32 },
  { BFD_RELOC_RISCV_TLS_DTPMOD64, R_RISCV_TLS_DTPMOD64 },
  { BFD_RELOC_RISCV_TLS_DTPREL64, R_RISCV_TLS_DTPREL64 },
  { BFD_RELOC_RISCV_TLS_TPREL32, R_RISCV_TLS_TPREL32 },
  { BFD_RELOC_RISCV_TLS_TPREL64, R_RISCV_TLS_TPREL64 },
  { BFD_RELOC_RISCV_TPREL_HI20, R_RISCV_TPREL_HI20 },
  { BFD_RELOC_RISCV_TPREL_ADD, R_RISCV_TPREL_ADD },
  { BFD_RELOC_RISCV_TPREL_LO12_S, R_RISCV_TPREL_LO12_S },
  { BFD_RELOC_RISCV_TPREL_LO12_I, R_RISCV_TPREL_LO12_I },
  { BFD_RELOC_RISCV_TLS_GOT_HI20, R_RISCV_TLS_GOT_HI20 },
  { BFD_RELOC_RISCV_TLS_GD_HI20, R_RISCV_TLS_GD_HI20 },
  { BFD_RELOC_RISCV_ALIGN, R_RISCV_ALIGN },
  { BFD_RELOC_RISCV_RVC_BRANCH, R_RISCV_RVC_BRANCH },
  { BFD_RELOC_RISCV_RVC_JUMP, R_RISCV_RVC_JUMP },
  { BFD_RELOC_RISCV_RVC_LUI, R_RISCV_RVC_LUI },
  { BFD_RELOC_RISCV_GPREL_I, R_RISCV_GPREL_I },
  { BFD_RELOC_RISCV_GPREL_S, R_RISCV_GPREL_S },
  { BFD_RELOC_RISCV_TPREL_I, R_RISCV_TPREL_I },
  { BFD_RELOC_RISCV_TPREL_S, R_RISCV_TPREL_S },
  { BFD_RELOC_RISCV_RELAX, R_RISCV_RELAX },
  { BFD_RELOC_RISCV_SUB6, R_RISCV_SUB6 },
  { BFD_RELOC_RISCV_SET6, R_RISCV_SET6 },
  { BFD_RELOC_RISCV_SET8, R_RISCV_SET8 },
  { BFD_RELOC_RISCV_SET16, R_RISCV_SET16 },
  { BFD_RELOC_RISCV_SET32, R_RISCV_SET32 },
  { BFD_RELOC_RISCV_32_PCREL, R_RISCV_32_PCREL },
  { BFD_RELOC_RISCV_NDS_MISC, R_RISCV_ANDES_MISC },
  { BFD_RELOC_RISCV_EXECIT_ITE, R_RISCV_EXECIT_ITE },
  { BFD_RELOC_RISCV_ICT_HI20, R_RISCV_ICT_HI20 },
  { BFD_RELOC_RISCV_ICT_LO12_I, R_RISCV_ICT_LO12_I },
  { BFD_RELOC_RISCV_PCREL_ICT_HI20, R_RISCV_PCREL_ICT_HI20 },
  { BFD_RELOC_RISCV_CALL_ICT, R_RISCV_CALL_ICT },
  { BFD_RELOC_RISCV_ICT_64, R_RISCV_ICT_64 },
  { BFD_RELOC_RISCV_NO_RVC_REGION_BEGIN, R_RISCV_NO_RVC_REGION_BEGIN },
  { BFD_RELOC_RISCV_NO_RVC_REGION_END, R_RISCV_NO_RVC_REGION_END },
  { BFD_RELOC_RISCV_DELETE, R_RISCV_DELETE },
  { BFD_RELOC_RISCV_ALIGN_BTB, R_RISCV_ALIGN_BTB },
  { BFD_RELOC_RISCV_10_PCREL, R_RISCV_10_PCREL },
  { BFD_RELOC_RISCV_DATA, R_RISCV_DATA },
  { BFD_RELOC_RISCV_LALO_HI20, R_RISCV_LALO_HI20 },
  { BFD_RELOC_RISCV_LALO_LO12_I, R_RISCV_LALO_LO12_I },
  { BFD_RELOC_RISCV_RELAX_ENTRY, R_RISCV_RELAX_ENTRY },
  { BFD_RELOC_RISCV_LGP18S0, R_RISCV_LGP18S0 },
  { BFD_RELOC_RISCV_LGP17S1, R_RISCV_LGP17S1 },
  { BFD_RELOC_RISCV_LGP17S2, R_RISCV_LGP17S2 },
  { BFD_RELOC_RISCV_LGP17S3, R_RISCV_LGP17S3 },
  { BFD_RELOC_RISCV_SGP18S0, R_RISCV_SGP18S0 },
  { BFD_RELOC_RISCV_SGP17S1, R_RISCV_SGP17S1 },
  { BFD_RELOC_RISCV_SGP17S2, R_RISCV_SGP17S2 },
  { BFD_RELOC_RISCV_SGP17S3, R_RISCV_SGP17S3 },
  { BFD_RELOC_RISCV_RELAX_REGION_BEGIN, R_RISCV_RELAX_REGION_BEGIN },
  { BFD_RELOC_RISCV_RELAX_REGION_END, R_RISCV_RELAX_REGION_END },
};

unsigned int number_of_howto_table = (unsigned int) ARRAY_SIZE (howto_table);
unsigned int ict_table_entries = 0;
unsigned int ict_model = 0;	/* Default set ict to tiny model.  */
bfd_boolean find_imported_ict_table = FALSE;

/* Given a BFD reloc type, return a howto structure.  */

reloc_howto_type *
riscv_reloc_type_lookup (bfd *abfd ATTRIBUTE_UNUSED,
			 bfd_reloc_code_real_type code)
{
  unsigned int i;

  for (i = 0; i < ARRAY_SIZE (riscv_reloc_map); i++)
    if (riscv_reloc_map[i].bfd_val == code)
      return &howto_table[(int) riscv_reloc_map[i].elf_val];

  bfd_set_error (bfd_error_bad_value);
  return NULL;
}

reloc_howto_type *
riscv_reloc_name_lookup (bfd *abfd ATTRIBUTE_UNUSED, const char *r_name)
{
  unsigned int i;

  for (i = 0; i < ARRAY_SIZE (howto_table); i++)
    if (howto_table[i].name && strcasecmp (howto_table[i].name, r_name) == 0)
      return &howto_table[i];

  return NULL;
}

reloc_howto_type *
riscv_elf_rtype_to_howto (bfd *abfd, unsigned int r_type)
{
  if (r_type >= ARRAY_SIZE (howto_table))
    {
      (*_bfd_error_handler) (_("%pB: unsupported relocation type %#x"),
			     abfd, r_type);
      bfd_set_error (bfd_error_bad_value);
      return NULL;
    }
  return &howto_table[r_type];
}

/* Special_function of RISCV_ADD and RISCV_SUB relocations.  */

static bfd_reloc_status_type
riscv_elf_add_sub_reloc (bfd *abfd,
			 arelent *reloc_entry,
			 asymbol *symbol,
			 void *data,
			 asection *input_section,
			 bfd *output_bfd,
			 char **error_message ATTRIBUTE_UNUSED)
{
  reloc_howto_type *howto = reloc_entry->howto;
  bfd_vma relocation;

  if (output_bfd != NULL
      && (symbol->flags & BSF_SECTION_SYM) == 0
      && (!reloc_entry->howto->partial_inplace || reloc_entry->addend == 0))
    {
      reloc_entry->address += input_section->output_offset;
      return bfd_reloc_ok;
    }

  if (output_bfd != NULL)
    return bfd_reloc_continue;

  relocation = symbol->value + symbol->section->output_section->vma
    + symbol->section->output_offset + reloc_entry->addend;
  bfd_vma old_value = bfd_get (howto->bitsize, abfd,
			       data + reloc_entry->address);

  switch (howto->type)
    {
    case R_RISCV_ADD8:
    case R_RISCV_ADD16:
    case R_RISCV_ADD32:
    case R_RISCV_ADD64:
      relocation = old_value + relocation;
      break;
    case R_RISCV_SUB6:
    case R_RISCV_SUB8:
    case R_RISCV_SUB16:
    case R_RISCV_SUB32:
    case R_RISCV_SUB64:
      relocation = old_value - relocation;
      break;
    }
  bfd_put (howto->bitsize, abfd, relocation, data + reloc_entry->address);

  return bfd_reloc_ok;
}

/* Array is used to compare the all extensions' order quickly.

   Zero     number means it is a preserved keyword.
   Negative number means it is a prefix keyword (s, h, x, z).
   Positive number means it is a standard extension.  */

static int riscv_ext_order[26] = {0};

/* Similar to the strcmp.  It returns an integer less than, equal to,
   or greater than zero if `subset2` is found, respectively, to be less
   than, to match, or be greater than `subset1`.  */

int
riscv_compare_subsets (const char *subset1x, const char *subset2x)
{
  char buf1[0x100], buf2[0x100];
  const char *subset1 = buf1;
  const char *subset2 = buf2;
  const char *p;
  char *q;
  p = subset1x, q = buf1;
  while (*p) *q++ = TOLOWER(*p++);
  *q = 0;
  p = subset2x, q = buf2;
  while (*p) *q++ = TOLOWER(*p++);
  *q = 0;

  int order1 = riscv_ext_order[(*subset1 - 'a')];
  int order2 = riscv_ext_order[(*subset2 - 'a')];

  /* Compare the standard extension first.  */
  if (order1 > 0 && order2 > 0)
    return order1 - order2;

  if (order1 == order2 && order1 < 0)
    {
      /* Compare the standard addition z extensions.  */
      if (*subset1 == 'z')
	{
	  order1 = riscv_ext_order[(*++subset1 - 'a')];
	  order2 = riscv_ext_order[(*++subset2 - 'a')];
	  if (order1 != order2)
	    return order1 - order2;
	}
      return strcasecmp (++subset1, ++subset2);
    }

  return order2 - order1;
}

/* Find subset in the list.  Return TRUE and set `current` to the subset
   if it is found.  Otherwise, return FALSE and set `current` to the place
   where we should insert the subset.  However, return FALSE with the NULL
   `current` means we should insert the subset at the head of subset list,
   if needed.  */

bfd_boolean
riscv_lookup_subset (const riscv_subset_list_t *subset_list,
		     const char *subset,
		     riscv_subset_t **current)
{
  riscv_subset_t *s, *pre_s = NULL;

  for (s = subset_list->head;
       s != NULL;
       pre_s = s, s = s->next)
    {
      int cmp = riscv_compare_subsets (s->name, subset);
      if (cmp == 0)
	{
	  *current = s;
	  return TRUE;
	}
      else if (cmp > 0)
	break;
    }
  *current = pre_s;
  return FALSE;
}

/* Add arch string extension to the last of the subset list.  */

void
riscv_add_subset (riscv_subset_list_t *subset_list,
		  const char *subset,
		  int major,
		  int minor)
{
  /* de-duplication */
  riscv_subset_t *ss;
  if (riscv_lookup_subset (subset_list, subset, &ss))
    { /* merge higher versions  */
      if (major > ss->major_version)
	ss->major_version = major;
      if (minor > ss->minor_version)
	ss->minor_version = minor;
      return;
    }

  riscv_subset_t *s = xmalloc (sizeof *s);

  if (subset_list->head == NULL)
    subset_list->head = s;

  s->name = xstrdup (subset);
  s->major_version = major;
  s->minor_version = minor;
  s->next = NULL;

  if (ss && ss != subset_list->tail)
    { /* insert  */
      s->next = ss->next;
      ss->next = s;
      return;
    }

  /* append  */
  if (subset_list->tail != NULL)
    subset_list->tail->next = s;

  subset_list->tail = s;
}

/* Add the implicit extension to the subset list.  Search the
   list first, and then find the right place to add.  */

static void
riscv_add_implicit_subset (riscv_subset_list_t *subset_list,
			   const char *subset,
			   int major,
			   int minor)
{
  riscv_subset_t *current, *new;

  if (riscv_lookup_subset (subset_list, subset, &current))
    return;

  new = xmalloc (sizeof *new);
  new->name = xstrdup (subset);
  new->major_version = major;
  new->minor_version = minor;
  new->next = NULL;

  if (current != NULL)
    {
      new->next = current->next;
      current->next = new;
    }
  else
    {
      new->next = subset_list->head;
      subset_list->head = new;
    }
}

/* These extensions are added to the subset list for special purposes,
   with the explicit versions or the RISCV_UNKNOWN_VERSION versions.
   Therefore, we won't output them to the output arch string in the
   riscv_arch_str1, if the versions are unknown.  */

static bfd_boolean
riscv_ext_dont_care_version (const char *subset)
{
  if (strcmp (subset, "g") == 0
      || strcmp (subset, "zicsr") == 0
      || strcmp (subset, "zifencei") == 0)
    return TRUE;
  return FALSE;
}

/* We have to add all arch string extensions first, and then start to
   add their implicit extensions.  The arch string extensions must be
   set in order, so we can add them to the last of the subset list
   directly, without searching.

   Find the default versions for the extension before adding them to
   the subset list, if their versions are RISCV_UNKNOWN_VERSION.
   Afterwards, report errors if we can not find their default versions.  */

static void
riscv_parse_add_subset (riscv_parse_subset_t *rps,
			const char *subset,
			int major,
			int minor,
			bfd_boolean implicit)
{
  int major_version = major;
  int minor_version = minor;

  if ((major_version == RISCV_UNKNOWN_VERSION
       || minor_version == RISCV_UNKNOWN_VERSION)
      && rps->get_default_version != NULL)
    rps->get_default_version (subset, &major_version, &minor_version);

  if (!riscv_ext_dont_care_version (subset)
      && (major_version == RISCV_UNKNOWN_VERSION
	  || minor_version == RISCV_UNKNOWN_VERSION))
    {
      /* We only add the implicit extension if it is supported in the
	 chosen ISA spec.  */
      if (implicit)
	return;

      if (subset[0] == 'x')
	rps->error_handler
	  (_("x ISA extension `%s' must be set with the versions"),
	   subset);
      else
	rps->error_handler
	  (_("cannot find default versions of the ISA extension `%s'"),
	   subset);
      return;
    }

  if (!implicit)
    riscv_add_subset (rps->subset_list, subset,
		      major_version, minor_version);
  else
    riscv_add_implicit_subset (rps->subset_list, subset,
			       major_version, minor_version);
}

 static void
riscv_del_subset (riscv_parse_subset_t *rps,
		  const char *subset)
{
  const riscv_subset_list_t *subset_list = rps->subset_list;
  riscv_subset_t *s, *pre_s = NULL;

  for (s = subset_list->head;
       s != NULL;
       pre_s = s, s = s->next)
    {
      int cmp = riscv_compare_subsets (s->name, subset);
      if (cmp) continue;
      pre_s->next = s->next;
      free ((void*)s->name);
      free (s);
    }
}

/* Release subset list.  */

void
riscv_release_subset_list (riscv_subset_list_t *subset_list)
{
   while (subset_list->head != NULL)
    {
      riscv_subset_t *next = subset_list->head->next;
      free ((void *)subset_list->head->name);
      free (subset_list->head);
      subset_list->head = next;
    }

  subset_list->tail = NULL;
}

/* Parsing extension version.

   Return Value:
     Points to the end of version

   Arguments:
     `rps`: Hooks and status for parsing extensions.
     `march`: Full arch string.
     `p`: Curent parsing position.
     `major_version`: Parsed major version.
     `minor_version`: Parsed minor version.
     `std_ext_p`: True if parsing standard extension.  */

static const char *
riscv_parsing_subset_version (riscv_parse_subset_t *rps,
			      const char *march,
			      const char *p,
			      int *major_version,
			      int *minor_version,
			      bfd_boolean std_ext_p)
{
  bfd_boolean major_p = TRUE;
  int version = 0;
  char np;

  *major_version = 0;
  *minor_version = 0;
  for (; *p; ++p)
    {
      if (*p == 'p')
	{
	  np = *(p + 1);

	  if (!ISDIGIT (np))
	    {
	      /* Might be beginning of `p` extension.  */
	      if (std_ext_p)
		{
		  *major_version = version;
		  *minor_version = 0;
		  return p;
		}
	      else
		{
		  rps->error_handler
		    (_("-march=%s: expect number after `%dp'"),
		     march, version);
		  return NULL;
		}
	    }

	  *major_version = version;
	  major_p = FALSE;
	  version = 0;
	}
      else if (ISDIGIT (*p))
	version = (version * 10) + (*p - '0');
      else
	break;
    }

  if (major_p)
    *major_version = version;
  else
    *minor_version = version;

  /* We can not find any version in string.  */
  if (*major_version == 0 && *minor_version == 0)
    {
      *major_version = RISCV_UNKNOWN_VERSION;
      *minor_version = RISCV_UNKNOWN_VERSION;
    }

  return p;
}

/* Return string which contain all supported standard extensions in
   canonical order.  */

const char *
riscv_supported_std_ext (void)
{
  return "mafdqlcbkjtpvn";
}

/* Parsing function for standard extensions.

   Return Value:
     Points to the end of extensions.

   Arguments:
     `rps`: Hooks and status for parsing subset.
     `march`: Full arch string.
     `p`: Curent parsing position.  */

static const char *
riscv_parse_std_ext (riscv_parse_subset_t *rps,
		     const char *march,
		     const char *p)
{
  const char *all_std_exts = riscv_supported_std_ext ();
  const char *std_exts = all_std_exts;
  int major_version;
  int minor_version;
  char subset[2] = {0, 0};

  /* First letter must start with i, e or g.  */
  switch (*p)
    {
      case 'i':
	p = riscv_parsing_subset_version (rps, march, ++p,
					  &major_version,
					  &minor_version, TRUE);
	riscv_parse_add_subset (rps, "i",
				major_version,
				minor_version, FALSE);
	break;

      case 'e':
	p = riscv_parsing_subset_version (rps, march, ++p,
					  &major_version,
					  &minor_version, TRUE);
	riscv_parse_add_subset (rps, "e",
				major_version,
				minor_version, FALSE);
	/* i-ext must be enabled.  */
	if (rps->get_default_version == NULL)
	  riscv_parse_add_subset (rps, "i", 2, 0, TRUE);
	else
	  riscv_parse_add_subset (rps, "i",
				RISCV_UNKNOWN_VERSION,
				RISCV_UNKNOWN_VERSION, FALSE);

	if (*rps->xlen > 32)
	  {
	    rps->error_handler
	      (_("-march=%s: rv%de is not a valid base ISA"),
	       march, *rps->xlen);
	    return NULL;
	  }
	break;

      case 'g':
	p = riscv_parsing_subset_version (rps, march, ++p,
					  &major_version,
					  &minor_version, TRUE);
	/* i-ext must be enabled.  */
	riscv_parse_add_subset (rps, "i",
				RISCV_UNKNOWN_VERSION,
				RISCV_UNKNOWN_VERSION, FALSE);
	/* g-ext is used to add the implicit extensions, but will
	   not be output to the arch string.  */
	riscv_parse_add_subset (rps, "g",
				major_version,
				minor_version, FALSE);
	for ( ; *std_exts != 'q'; std_exts++)
	  {
	    subset[0] = *std_exts;
	    riscv_parse_add_subset (rps, subset,
				    RISCV_UNKNOWN_VERSION,
				    RISCV_UNKNOWN_VERSION, FALSE);
	  }
	break;

      default:
	rps->error_handler
	  (_("-march=%s: first ISA extension must be `e', `i' or `g'"),
	   march);
	return NULL;
    }

  while (p != NULL && *p != '\0')
    {
      if (*p == 'x' || *p == 's' || *p == 'h' || *p == 'z')
	break;

      if (*p == '_')
	{
	  p++;
	  continue;
	}

      /* Checking canonical order.  */
      char std_ext = *p;
      while (*std_exts && std_ext != *std_exts)
	std_exts++;

      if (std_ext != *std_exts)
	{
	  if (strchr (all_std_exts, std_ext) == NULL)
	    rps->error_handler
	      (_("-march=%s: unknown standard ISA extension `%c'"),
	       march, std_ext);
	  else
	    rps->error_handler
	      (_("-march=%s: standard ISA extension `%c' is not "
		 "in canonical order"), march, std_ext);
	  return NULL;
	}

      std_exts++;
      subset[0] = std_ext;
      p = riscv_parsing_subset_version (rps, march, ++p,
					&major_version,
					&minor_version, TRUE);
      riscv_parse_add_subset (rps, subset,
			      major_version,
			      minor_version, FALSE);
    }

  return p;
}

typedef struct {
  const char *name;
  const int major;
  const int minor;
} default_version_t;

riscv_isa_ext_class_t
riscv_get_prefix_class (const char *arch)
{
  switch (*arch)
    {
    case 's': case 'S': return RV_ISA_CLASS_S;
    case 'x': case 'X': return RV_ISA_CLASS_X;
    case 'z': case 'Z': return RV_ISA_CLASS_Z;
    default: return RV_ISA_CLASS_UNKNOWN;
    }
}

/* Structure describing parameters to use when parsing a particular
   riscv_isa_ext_class_t. One of these should be provided for each
   possible class, except RV_ISA_CLASS_UNKNOWN.  */

typedef struct riscv_parse_config
{
  /* Class of the extension. */
  riscv_isa_ext_class_t class;

  /* Lower-case prefix string for error printing
     and internal parser usage, e.g. "z", "x".  */
  const char *prefix;

  /* Predicate which is used for checking whether
     this is a "known" extension. For 'x',
     it always returns true (since they are by
     definition non-standard and cannot be known.  */
  bfd_boolean (*ext_valid_p) (const char *);
} riscv_parse_config_t;

/* Parsing function for prefixed extensions.

   Return Value:
     Points to the end of extension.

   Arguments:
     `rps`: Hooks and status for parsing extensions.
     `march`: Full architecture string.
     `p`: Curent parsing position.
     `config`: What class and predicate function to use for the
     extension.  */

static const char *
riscv_parse_prefixed_ext (riscv_parse_subset_t *rps,
			  const char *march,
			  const char *p,
			  const riscv_parse_config_t *config)
{
  int major_version;
  int minor_version;
  const char *last_name;
  riscv_isa_ext_class_t class;

  while (*p)
    {
      if (*p == '_')
	{
	  p++;
	  continue;
	}

      /* Assert that the current extension specifier matches our parsing
	 class.  */
      class = riscv_get_prefix_class (p);
      if (class != config->class
	  || class == RV_ISA_CLASS_UNKNOWN)
	break;

      char *subset = xstrdup (p);
      char *q = subset;
      const char *end_of_version;

      while (*++q != '\0' && *q != '_' && !ISDIGIT (*q))
	;

      end_of_version =
	riscv_parsing_subset_version (rps, march, q,
				      &major_version,
				      &minor_version, FALSE);
      *q = '\0';

      if (end_of_version == NULL)
	{
	  free (subset);
	  return NULL;
	}

      /* Check that the prefix extension is known.
	 For 'x', anything goes but it cannot simply be 'x'.
	 For 's', it must be known from a list and cannot simply be 's'.
	 For 'h', it must be known from a list and cannot simply be 'h'.
	 For 'z', it must be known from a list and cannot simply be 'z'.  */

      /* Check that the extension name is well-formed.  */
      if (!config->ext_valid_p (subset))
	{
	  rps->error_handler
	    (_("-march=%s: unknown %s ISA extension `%s'"),
	     march, config->prefix, subset);
	  free (subset);
	  return NULL;
	}

      /* Check that the extension isn't duplicate.  */
      last_name = rps->subset_list->tail->name;
      if (!strcasecmp (last_name, subset))
	{
	  rps->error_handler
	    (_("-march=%s: duplicate %s ISA extension `%s'"),
	     march, config->prefix, subset);
	  free (subset);
	  return NULL;
	}

      /* Check that the extension is in alphabetical order.  */
      if (riscv_compare_subsets (last_name, subset) > 0)
	{
	  rps->error_handler
	    (_("-march=%s: %s ISA extension `%s' is not in alphabetical "
	       "order.  It must come before `%s'"),
	     march, config->prefix, subset, last_name);
	  free (subset);
	  return NULL;
	}

      riscv_parse_add_subset (rps, subset,
			      major_version,
			      minor_version, FALSE);
      p += end_of_version - subset;
      free (subset);

      if (*p != '\0' && *p != '_')
	{
	  rps->error_handler
	    (_("-march=%s: %s ISA extension must separate with _"),
	     march, config->prefix);
	  return NULL;
	}
    }

  return p;
}

/* List of Z-class extensions that binutils should know about.
   Whether or not a particular entry is in this list will
   dictate if gas/ld will accept its presence in the -march
   string.

   Example: To add an extension called "Zbb" (bitmanip base extension),
   add "zbb" string to the list (all lowercase).

   Keep this list alphabetically ordered.  */

static const char * const riscv_std_z_ext_strtab[] =
  {
    "zefhw", "zfh", /* Andes Extensions  */
    "zvamo", "zvlsseg", /* RVV  */
    "zicsr", "zifencei", "zihintpause",
    "zba", "zbb", "zbc", "zbs",
    "zbkb", "zbkc", "zbkx", "zknd", "zkne", "zknh", "zksed", "zksh", "zkr",
    "zkn",  "zks", "zk", "zkt",
    NULL
  };

/* Same as `riscv_std_z_ext_strtab', but for S-class extensions.  */

static const char * const riscv_std_s_ext_strtab[] =
  {
    NULL
  };

static const char * const riscv_std_h_ext_strtab[] =
{
  NULL
};

/* For the extension `ext`, search through the list of known extensions
   `known_exts` for a match, and return TRUE if found.  */

static bfd_boolean
riscv_multi_letter_ext_valid_p (const char *ext,
				const char *const *known_exts)
{
  size_t i;

  for (i = 0; known_exts[i]; ++i)
    if (!strcmp (ext, known_exts[i]))
      return TRUE;

  return FALSE;
}

/* Predicator function for x-prefixed extensions.
   Anything goes, except the literal 'x'.  */

static bfd_boolean
riscv_ext_x_valid_p (const char *arg)
{
  if (!strcasecmp (arg, "x"))
    return FALSE;

  return TRUE;
}

/* Predicator functions for z-prefixed extensions.
   Only known z-extensions are permitted.  */

static bfd_boolean
riscv_ext_z_valid_p (const char *arg)
{
  return riscv_multi_letter_ext_valid_p (arg, riscv_std_z_ext_strtab);
}

/* Predicator function for 's' prefixed extensions.
   Only known s-extensions are permitted.  */

static bfd_boolean
riscv_ext_s_valid_p (const char *arg)
{
  return riscv_multi_letter_ext_valid_p (arg, riscv_std_s_ext_strtab);
}

/* Predicator function for 'h' prefixed extensions.
   Only known h-extensions are permitted.  */

static bfd_boolean
riscv_ext_h_valid_p (const char *arg)
{
  return riscv_multi_letter_ext_valid_p (arg, riscv_std_h_ext_strtab);
}

/* Parsing order of the prefixed extensions that is specified by
   the ISA spec.  */

static const riscv_parse_config_t parse_config[] =
{
  {RV_ISA_CLASS_S, "s", riscv_ext_s_valid_p},
  {RV_ISA_CLASS_H, "h", riscv_ext_h_valid_p},
  {RV_ISA_CLASS_Z, "z", riscv_ext_z_valid_p},
  {RV_ISA_CLASS_X, "x", riscv_ext_x_valid_p},
  {RV_ISA_CLASS_UNKNOWN, NULL, NULL}
};

/* Init the riscv_ext_order array.  */

static void
riscv_init_ext_order (void)
{
  static bfd_boolean inited = FALSE;
  const char *std_base_exts = "eig";
  const char *std_remain_exts = riscv_supported_std_ext ();
  const char *ext;
  unsigned int i;
  int order;

  if (inited)
    return;

  /* All standard extensions' orders are positive numbers.  */
  order = 1;

  /* Init the standard base extensions first.  */
  for (ext = std_base_exts; *ext; ext++)
    riscv_ext_order[(*ext - 'a')] = order++;

  /* Init the standard remaining extensions.  */
  for (ext = std_remain_exts; *ext; ext++)
    riscv_ext_order[(*ext - 'a')] = order++;

  /* Init the order for prefixed keywords.  The orders are
     negative numbers.  */
  order = -1;
  for (i = 0; parse_config[i].class != RV_ISA_CLASS_UNKNOWN; i++)
    {
      ext = parse_config[i].prefix;
      riscv_ext_order[(*ext - 'a')] = order--;
    }

  inited = TRUE;
}

/* Add the implicit extensions according to the arch string extensions.  */

static char* implicit_isa_table[] =
{/* isa, flags, implicit_isa  */
  "zbkb", "a", "zbb", 0,
  "zbkc", "a", "zbc", 0,
  "zknd", "a", "zkne", 0,
  "zkne", "a", "zknd", 0,
  "zbk3", "r", "zbkb", "zbkc", "zbkx", 0, /* virtual */
  "zkn", "r", "zbk3", "zkne", "zknd", "zknh", 0,
  "zks", "r", "zbk3", "zksed", "zksh", 0,
  "zk", "r", "zkn", "zkr", "zkt", 0,
  0
};

static void
riscv_parse_add_implicit_subsets (riscv_parse_subset_t *rps)
{
  riscv_subset_t *subset = NULL;

  /* Add the zicsr and zifencei only when the i's version less than 2.1.  */
  if ((riscv_lookup_subset (rps->subset_list, "i", &subset))
      && (subset->major_version < 2
	  || (subset->major_version == 2
	      && subset->minor_version < 1)))
    {
      riscv_parse_add_subset (rps, "zicsr",
			      RISCV_UNKNOWN_VERSION,
			      RISCV_UNKNOWN_VERSION, TRUE);
      riscv_parse_add_subset (rps, "zifencei",
			      RISCV_UNKNOWN_VERSION,
			      RISCV_UNKNOWN_VERSION, TRUE);
    }

  if ((riscv_lookup_subset (rps->subset_list, "q", &subset)))
    {
      riscv_parse_add_subset (rps, "d",
			      RISCV_UNKNOWN_VERSION,
			      RISCV_UNKNOWN_VERSION, TRUE);
      riscv_parse_add_subset (rps, "f",
			      RISCV_UNKNOWN_VERSION,
			      RISCV_UNKNOWN_VERSION, TRUE);
      riscv_parse_add_subset (rps, "zicsr",
			      RISCV_UNKNOWN_VERSION,
			      RISCV_UNKNOWN_VERSION, TRUE);
    }
  else if ((riscv_lookup_subset (rps->subset_list, "d", &subset)))
    {
      riscv_parse_add_subset (rps, "f",
			      RISCV_UNKNOWN_VERSION,
			      RISCV_UNKNOWN_VERSION, TRUE);
      riscv_parse_add_subset (rps, "zicsr",
			      RISCV_UNKNOWN_VERSION,
			      RISCV_UNKNOWN_VERSION, TRUE);
    }
  else if ((riscv_lookup_subset (rps->subset_list, "f", &subset)))
    riscv_parse_add_subset (rps, "zicsr",
			    RISCV_UNKNOWN_VERSION,
			    RISCV_UNKNOWN_VERSION, TRUE);

  if ((riscv_lookup_subset (rps->subset_list, "g", &subset)))
    {
      riscv_parse_add_subset (rps, "zicsr",
			      RISCV_UNKNOWN_VERSION,
			      RISCV_UNKNOWN_VERSION, TRUE);
      riscv_parse_add_subset (rps, "zifencei",
			      RISCV_UNKNOWN_VERSION,
			      RISCV_UNKNOWN_VERSION, TRUE);
    }

  { /* { handle implicit isa  */
    int i = 0;
    while (implicit_isa_table[i])
      {
	char *pisa = implicit_isa_table[i++];
	if ((riscv_lookup_subset (rps->subset_list, pisa, &subset)))
	  {
	    i++; /* skip flags for now  */
	    while (implicit_isa_table[i])
	      {
		pisa = implicit_isa_table[i++];
		riscv_parse_add_subset (rps, pisa,
					RISCV_UNKNOWN_VERSION,
					RISCV_UNKNOWN_VERSION, TRUE);
	      }
	  }
	i++;
      }
  } /* } handle implicit isa  */
}

/* Add the implicit extensions according to the arch string extensions.
 * Andes Extended.
 */

static void
andes_parse_add_implicit_subsets (riscv_parse_subset_t *rps)
{
  riscv_subset_t *subset = NULL;

  /* replace old extension names  */
  /* # xv5 is replaced with xandes, and xv5 / xv5-0p0 implies xefhw  */
  if (riscv_lookup_subset (rps->subset_list, "xv", &subset))
    {
      if (((subset->major_version == 5) || (subset->major_version == 50))
	  && (subset->minor_version == 0))
	{
	  riscv_parse_add_subset (rps, "xefhw",
				  RISCV_UNKNOWN_VERSION,
				  RISCV_UNKNOWN_VERSION, FALSE);
	}
      riscv_parse_add_subset (rps, "xandes",
			      RISCV_UNKNOWN_VERSION,
			      RISCV_UNKNOWN_VERSION, FALSE);
      riscv_del_subset (rps, "xv");
    }

  /* # xdsp is replaced with standard p  */
  if ((riscv_lookup_subset (rps->subset_list, "xdsp", &subset))
      && (subset->major_version >= 5))
    {
      riscv_del_subset (rps, "xdsp");
      riscv_parse_add_subset (rps, "p",
			      RISCV_UNKNOWN_VERSION,
			      RISCV_UNKNOWN_VERSION, FALSE);
    }

  /* add implicit extensions  */
  /* # zfh is implied by both v and f
   *   default version 0p0 is not working by current logic
   */
  if (riscv_lookup_subset (rps->subset_list, "v", &subset) &&
      riscv_lookup_subset (rps->subset_list, "f", &subset))
    {
      riscv_parse_add_subset (rps, "zfh",
			      RISCV_UNKNOWN_VERSION,
			      RISCV_UNKNOWN_VERSION, TRUE);
    }
}


static bfd_boolean
andes_parse_check_conflict_subsets (riscv_parse_subset_t *rps, const char *arch)
{
  bfd_boolean no_conflict = TRUE;
  riscv_subset_t *subset = NULL;
  if (riscv_lookup_subset (rps->subset_list, "v", &subset)
      && riscv_lookup_subset (rps->subset_list, "xefhw", &subset))
    {
      rps->error_handler
	(_("-march=%s: 'V' and 'XEFHW' are exclusive!"),
	 arch);
      no_conflict = FALSE;
    }
  return no_conflict;
}

static void
andes_replace_old_name_xv5 (const char *arch, char *buf)
{
  /* convert "xv5-" to "xv5"  */
  char tmp[0x200];
  strncpy (tmp, arch, sizeof(tmp));
  const char *p = tmp;
  char *q = buf;
  char *r = strstr (p, "xv5");
  while (r)
    {
      int sz = (intptr_t)r - (intptr_t)p + 3;
      strncpy (q, p, sz);
      p += sz;
      q += sz;
      if (*p == '-') p++;
      r = strstr (p, "xv5");
    }

  while (*p) *q++ = *p++;
  *q = 0;
}

static void
andes_replace_old_name_xdsp (const char *arch, char *buf)
{
  /* convert "xdsp" to "xdsp5"  */
  char tmp[0x200];
  strncpy (tmp, arch, sizeof(tmp));
  const char *p = tmp;
  char *q = buf;
  char *r = strstr (p, "xdsp");
  while (r)
    {
      int sz = (intptr_t)r - (intptr_t)p + 4;
      strncpy (q, p, sz);
      p += sz;
      q += sz;
      if (*p == '-') p++;
      *q++ = '5';
      r = strstr (p, "xdsp");
    }

  while (*p) *q++ = *p++;
  *q = 0;
}

/* Function for parsing arch string.

   Return Value:
     Return TRUE on success.

   Arguments:
     `rps`: Hooks and status for parsing extensions.
     `arch`: Full arch string.  */

bfd_boolean
riscv_parse_subset (riscv_parse_subset_t *rps,
		    const char *archx)
{
  riscv_subset_t *subset = NULL;
  const char *p;
  size_t i;
  bfd_boolean no_conflict = TRUE;

  /* Andes compatibility  */
  char buf[0x200];
  andes_replace_old_name_xv5 (archx, buf);
  andes_replace_old_name_xdsp (buf, buf);
  const char *arch = buf;

  for (p = arch; *p != '\0'; p++)
    {
      if (ISUPPER (*p))
	{
	  rps->error_handler
	    (_("-march=%s: ISA string cannot contain uppercase letters"),
	     arch);
	  return FALSE;
	}
    }

  p = arch;
  if (strncmp (p, "rv32", 4) == 0)
    {
      *rps->xlen = 32;
      p += 4;
    }
  else if (strncmp (p, "rv64", 4) == 0)
    {
      *rps->xlen = 64;
      p += 4;
    }
  else
    {
      /* Arch string shouldn't be NULL or empty here.  However,
	 it might be empty only when we failed to merge the arch
	 string in the riscv_merge_attributes.  We have already
	 issued the correct error message in another side, so do
	 not issue this error when the arch string is empty.  */
      if (strlen (arch))
	rps->error_handler (
	  _("-march=%s: ISA string must begin with rv32 or rv64"),
	  arch);
      return FALSE;
    }

  /* Init the riscv_ext_order array to compare the order of extensions
     quickly.  */
  riscv_init_ext_order ();

  /* Parsing standard extension.  */
  p = riscv_parse_std_ext (rps, arch, p);

  if (p == NULL)
    return FALSE;

  /* Parse the different classes of extensions in the specified order.  */
  for (i = 0; i < ARRAY_SIZE (parse_config); ++i)
    {
      p = riscv_parse_prefixed_ext (rps, arch, p, &parse_config[i]);

      if (p == NULL)
	return FALSE;
    }

  if (*p != '\0')
    {
      rps->error_handler (_("-march=%s: unexpected ISA string at end: %s"),
			  arch, p);
      return FALSE;
    }

  /* Finally add implicit extensions according to the current
     extensions.  */
  riscv_parse_add_implicit_subsets (rps);
  andes_parse_add_implicit_subsets (rps);

  /* Check the conflicts.  */
  if (riscv_lookup_subset (rps->subset_list, "e", &subset)
      && riscv_lookup_subset (rps->subset_list, "f", &subset))
    {
      rps->error_handler
	(_("-march=%s: rv32e does not support the `f' extension"),
	 arch);
      no_conflict = FALSE;
    }
  if (riscv_lookup_subset (rps->subset_list, "q", &subset)
      && *rps->xlen < 64)
    {
      rps->error_handler
	(_("-march=%s: rv32 does not support the `q' extension"),
	 arch);
      no_conflict = FALSE;
    }

  no_conflict &= andes_parse_check_conflict_subsets (rps, arch);

  return no_conflict;
}

/* Add new subset to list.  */
static const char extension_subsets_order[] = "iemafdgqlcbjtpvnzshzx";

static int ATTRIBUTE_UNUSED
riscv_subset_order(const char *subset)
{
  int order = -1;
  char *p = strchr(extension_subsets_order, TOLOWER(subset[0]));
  if (p)
    order = (int)(p - extension_subsets_order);

  return order;
}

/* Find subset in list with version checking, return NULL if not found.  */

riscv_subset_t *
riscv_lookup_subset_version (const riscv_subset_list_t *subset_list,
			     const char *subset,
			     int major, int minor)
{
  riscv_subset_t *s;

  for (s = subset_list->head; s != NULL; s = s->next)
    if (strcasecmp (s->name, subset) == 0)
      {
	if ((major != RISCV_DONT_CARE_VERSION)
	    && (s->major_version != major))
	  return NULL;

	if ((minor != RISCV_DONT_CARE_VERSION)
	    && (s->minor_version != minor))
	  return NULL;

	return s;
      }

  return NULL;
}

/* Return the number of digits for the input.  */

static size_t
riscv_estimate_digit (unsigned num)
{
  size_t digit = 0;
  if (num == 0)
    return 1;

  for (digit = 0; num ; num /= 10)
    digit++;

  return digit;
}

/* Auxiliary function to estimate string length of subset list.  */

static size_t
riscv_estimate_arch_strlen1 (const riscv_subset_t *subset)
{
  if (subset == NULL)
    return 6; /* For rv32/rv64/rv128 and string terminator.  */

  return riscv_estimate_arch_strlen1 (subset->next)
	 + strlen (subset->name)
	 + riscv_estimate_digit (subset->major_version)
	 + 1 /* For version seperator: 'p'.  */
	 + riscv_estimate_digit (subset->minor_version)
	 + 1 /* For underscore.  */;
}

/* Estimate the string length of this subset list.  */

static size_t
riscv_estimate_arch_strlen (const riscv_subset_list_t *subset_list)
{
  return riscv_estimate_arch_strlen1 (subset_list->head);
}

/* Auxiliary function to convert subset info to string.  */

/* Auxiliary function to convert subset info to string.  */

static void
riscv_arch_str1 (riscv_subset_t *subset,
		 char *attr_str, char *buf, size_t bufsz)
{
  const char *underline = "_";
  riscv_subset_t *subset_t = subset;

  if (subset_t == NULL)
    return;

  /* No underline between rvXX and i/e.   */
  if ((strcasecmp (subset_t->name, "i") == 0)
      || (strcasecmp (subset_t->name, "e") == 0))
    underline = "";

  snprintf (buf, bufsz, "%s%s%dp%d",
	    underline,
	    subset_t->name,
	    subset_t->major_version,
	    subset_t->minor_version);

  strncat (attr_str, buf, bufsz);

  /* Skip 'i' extension after 'e', or skip extensions which
     versions are unknown.  */
  while (subset_t->next
	 && ((strcmp (subset_t->name, "e") == 0
	      && strcmp (subset_t->next->name, "i") == 0)
	     || subset_t->next->major_version == RISCV_UNKNOWN_VERSION
	     || subset_t->next->minor_version == RISCV_UNKNOWN_VERSION))
    subset_t = subset_t->next;

  riscv_arch_str1 (subset_t->next, attr_str, buf, bufsz);
}

/* Convert subset info to string with explicit version info.  */

char *
riscv_arch_str (unsigned xlen, const riscv_subset_list_t *subset)
{
  size_t arch_str_len = riscv_estimate_arch_strlen (subset);
  char *attr_str = xmalloc (arch_str_len);
  char *buf = xmalloc (arch_str_len);

  snprintf (attr_str, arch_str_len, "rv%u", xlen);

  riscv_arch_str1 (subset->head, attr_str, buf, arch_str_len);
  free (buf);

  return attr_str;
}
