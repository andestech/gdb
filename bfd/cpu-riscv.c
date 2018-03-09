/* BFD backend for RISC-V
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

#include "sysdep.h"
#include "bfd.h"
#include "libbfd.h"
/* { #include "elfxx-riscv.h" # workaround  */
#include "opcode/riscv.h"

extern int
riscv_get_priv_spec_class (const char *, enum riscv_priv_spec_class *);

extern int
riscv_get_priv_spec_class_from_numbers (unsigned int,
					unsigned int,
					unsigned int,
					enum riscv_priv_spec_class *);

extern const char *
riscv_get_priv_spec_name (enum riscv_priv_spec_class);
/* } #include "elfxx-riscv.h" # workaround  */

/* Record the priv spec version string and the corresponding class.  */

struct priv_spec_t
{
  const char *name;
  enum riscv_priv_spec_class class;
};

/* List for all supported privilege versions.  */

static const struct priv_spec_t priv_specs[] =
{
  {"1.9.1", PRIV_SPEC_CLASS_1P9P1},
  {"1.10",  PRIV_SPEC_CLASS_1P10},
  {"1.11",  PRIV_SPEC_CLASS_1P11},

/* Terminate the list.  */
  {NULL, 0}
};

/* Get the corresponding CSR version class by giving a privilege
   version string.  */

int
riscv_get_priv_spec_class (const char *s,
			   enum riscv_priv_spec_class *class)
{
  const struct priv_spec_t *version;

  if (s == NULL)
    return 0;

  for (version = &priv_specs[0]; version->name != NULL; ++version)
    if (strcmp (version->name, s) == 0)
      {
	*class = version->class;
	return 1;
      }

  /* Can not find the supported privilege version.  */
  return 0;
}

/* Get the corresponding CSR version class by giving privilege
   version numbers.  It is usually used to convert the priv
   attribute numbers into the corresponding class.  */

int
riscv_get_priv_spec_class_from_numbers (unsigned int major,
					unsigned int minor,
					unsigned int revision,
					enum riscv_priv_spec_class *class)
{
  char buf[36];

  if (major == 0 && minor == 0 && revision == 0)
    {
      *class = PRIV_SPEC_CLASS_NONE;
      return 1;
    }

  if (revision != 0)
    snprintf (buf, sizeof (buf), "%u.%u.%u", major, minor, revision);
  else
    snprintf (buf, sizeof (buf), "%u.%u", major, minor);

  return riscv_get_priv_spec_class (buf, class);
}

/* Get the corresponding privilege version string by giving a CSR
   version class.  */

const char *
riscv_get_priv_spec_name (enum riscv_priv_spec_class class)
{
  /* The first enum is PRIV_SPEC_CLASS_NONE.  */
  return priv_specs[class - 1].name;
}

/* This routine is provided two arch_infos and returns an arch_info
   that is compatible with both, or NULL if none exists.  */

static const bfd_arch_info_type *
riscv_compatible (const bfd_arch_info_type *a, const bfd_arch_info_type *b)
{
  if (a->arch != b->arch)
    return NULL;

  /* Machine compatibility is checked in
     _bfd_riscv_elf_merge_private_bfd_data.  */

  return a;
}

#define N(BITS_WORD, BITS_ADDR, NUMBER, PRINT, DEFAULT, NEXT)	\
  {								\
    BITS_WORD, /*  bits in a word */				\
    BITS_ADDR, /* bits in an address */				\
    8,	/* 8 bits in a byte */					\
    bfd_arch_riscv,						\
    NUMBER,							\
    "riscv",							\
    PRINT,							\
    3,								\
    DEFAULT,							\
    riscv_compatible,						\
    bfd_default_scan,						\
    bfd_arch_default_fill,					\
    NEXT,							\
  }

/* This enum must be kept in the same order as arch_info_struct.  */
enum
{
  I_riscv64,
  I_riscv32
};

#define NN(index) (&arch_info_struct[(index) + 1])

/* This array must be kept in the same order as the anonymous enum above,
   and each entry except the last should end with NN (my enum value).  */
static const bfd_arch_info_type arch_info_struct[] =
{
  N (64, 64, bfd_mach_riscv64, "riscv:rv64", FALSE, NN (I_riscv64)),
  N (32, 32, bfd_mach_riscv32, "riscv:rv32", FALSE, 0)
};

/* The default architecture is riscv:rv64.  */

const bfd_arch_info_type bfd_riscv_arch =
  N (64, 64, 0, "riscv", TRUE, &arch_info_struct[0]);
