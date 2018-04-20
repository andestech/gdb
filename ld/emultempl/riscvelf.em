# This shell script emits a C file. -*- C -*-
#   Copyright (C) 2004-2019 Free Software Foundation, Inc.
#
# This file is part of the GNU Binutils.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street - Fifth Floor, Boston,
# MA 02110-1301, USA.

fragment <<EOF

#include "ldmain.h"
#include "ldctor.h"
#include "elf/riscv.h"
#include "elfxx-riscv.h"
#include "elf-bfd.h"
#include "libbfd.h"

static int target_aligned = 1;		/* Defalut do target aligned.  */
static int gp_relative_insn = 0;	/* Support gp relative insn.  */
static FILE *sym_ld_script = NULL;	/* Export global symbols into linker
					   script.  */
static int avoid_btb_miss = 1;		/* Default avoid BTB miss.  */

#define RISCV_EX9_EXT
static int target_optimize = 0;		/* Switch optimization.  */
static int relax_status = 0;		/* Finish optimization.  */
static char *ex9_export_file = NULL;	/* Export the ex9 table.  */
static FILE *ex9_import_file = NULL;	/* Do ex9 according to the imported
					   ex9 table.  */
static int keep_import_ex9 = 0;		/* Keep the imported ex9 table.  */
static int update_ex9_table = 0;
static int ex9_limit = -1;		/* Default set it to 512 entries.  */
static int ex9_loop_aware = 0;		/* --mex9-loop-aware.  */

/* Put target dependent option into info hash table.  */

static void
riscv_elf_set_target_option (struct bfd_link_info *info)
{
  struct riscv_elf_link_hash_table *table;

  table = riscv_elf_hash_table (info);
  if (table == NULL)
    return;

  table->target_aligned = target_aligned;
  table->gp_relative_insn = gp_relative_insn;
  table->sym_ld_script = sym_ld_script;
  table->avoid_btb_miss = avoid_btb_miss;
  table->target_optimize = target_optimize;
  table->relax_status = relax_status;
  table->ex9_export_file = ex9_export_file;
  table->ex9_import_file = ex9_import_file;
  table->keep_import_ex9 = keep_import_ex9;
  table->update_ex9_table = update_ex9_table;
  table->ex9_limit = ex9_limit;
  table->ex9_loop_aware = ex9_loop_aware;
}

static void
riscv_elf_append_section (struct bfd_link_info *info, bfd *abfd)
{
  asection *itable;
  struct bfd_link_hash_entry *h;

  if (target_optimize & RISCV_RELAX_EX9_ON
      && (ex9_import_file == NULL
	  || keep_import_ex9
	  || update_ex9_table))
    {
      /* Create section ".ex9.itable".  */
      itable = bfd_make_section_with_flags (abfd, ".ex9.itable",
					    SEC_CODE | SEC_ALLOC | SEC_LOAD
					    | SEC_HAS_CONTENTS | SEC_READONLY
					    | SEC_IN_MEMORY | SEC_KEEP
					    | SEC_RELOC);
      if (itable)
	{
	  itable->gc_mark = 1;
	  itable->alignment_power = 2;
	  /* Default ex9 table size can not be zero, so we can not set
	     it according to ex9_limit. Since we will adjust the table size
	     in riscv_elf_ex9_build_itable, it is okay to set the size to
	     the maximum value here.  */
	  itable->size = 0x800;
	  itable->contents = bfd_zalloc (abfd, itable->size);

	  /* Add a symbol in the head of ex9.itable to objdump clearly.  */
	  h = bfd_link_hash_lookup (info->hash, "_EX9_BASE_",
				    FALSE, FALSE, FALSE);
	  _bfd_generic_link_add_one_symbol
	    (info, info->output_bfd, "_EX9_BASE_",
	     BSF_GLOBAL | BSF_WEAK, itable, 0, (const char *) NULL, FALSE,
	     get_elf_backend_data (info->output_bfd)->collect, &h);
	}
    }
}

/* Save the target options into output bfd to avoid using to many global
   variables. Do this after the output has been created, but before
   inputs are read.  */

static void
riscv_elf_create_output_section_statements (void)
{
  if (strstr (bfd_get_target (link_info.output_bfd), "riscv") == NULL)
    {
      /* Check the output target is riscv.  */
      einfo ("%F%X%P: error: Cannot change output format whilst linking riscv binaries.\n");
      return;
    }

  riscv_elf_set_target_option (&link_info);
}

static void
riscv_elf_before_allocation (void)
{
  gld${EMULATION_NAME}_before_allocation ();

  if (link_info.discard == discard_sec_merge)
    link_info.discard = discard_l;

  if (!bfd_link_relocatable (&link_info))
    {
      /* We always need at least some relaxation to handle code alignment.  */
      if (RELAXATION_DISABLED_BY_USER)
	TARGET_ENABLE_RELAXATION;
      else
	ENABLE_RELAXATION;
    }

  link_info.relax_pass = 7;
}

static void
gld${EMULATION_NAME}_after_allocation (void)
{
  int need_layout = 0;

  /* Don't attempt to discard unused .eh_frame sections until the final link,
     as we can't reliably tell if they're used until after relaxation.  */
  if (!bfd_link_relocatable (&link_info))
    {
      need_layout = bfd_elf_discard_info (link_info.output_bfd, &link_info);
      if (need_layout < 0)
	{
	  einfo (_("%X%P: .eh_frame/.stab edit: %E\n"));
	  return;
	}
    }

  gld${EMULATION_NAME}_map_segments (need_layout);

  /* Add a symbol for linker script check the max size.  */
  if (link_info.output_bfd->sections)
    {
      struct bfd_link_hash_entry *h;
      h = bfd_link_hash_lookup (link_info.hash, "_RELAX_END_",
				FALSE, FALSE, FALSE);
      if (!h)
	_bfd_generic_link_add_one_symbol
	  (&link_info, link_info.output_bfd, "_RELAX_END_",
	   BSF_GLOBAL | BSF_WEAK, link_info.output_bfd->sections,
	   0, (const char *) NULL, FALSE,
	   get_elf_backend_data (link_info.output_bfd)->collect, &h);
    }
}

/* This is a convenient point to tell BFD about target specific flags.
   After the output has been created, but before inputs are read.  */

static void
riscv_create_output_section_statements (void)
{
  /* See PR 22920 for an example of why this is necessary.  */
  if (strstr (bfd_get_target (link_info.output_bfd), "riscv") == NULL)
    {
      /* The RISC-V backend needs special fields in the output hash structure.
	 These will only be created if the output format is a RISC-V format,
	 hence we do not support linking and changing output formats at the
	 same time.  Use a link followed by objcopy to change output formats.  */
      einfo (_("%F%P: error: cannot change output format"
	       " whilst linking %s binaries\n"), "RISC-V");
      return;
    }
}

static void
riscv_elf_after_open (void)
{
  bfd *abfd;
  for (abfd = link_info.input_bfds; abfd != NULL; abfd = abfd->link.next)
    {
      /* Append target needed section in the last input object file.  */
      if (abfd->link.next == NULL)
	riscv_elf_append_section (&link_info, abfd);
    }

  /* Call the standard elf routine.  */
  gld${EMULATION_NAME}_after_open ();
}
EOF
# Define some shell vars to insert bits of code into the standard elf
# parse_args and list_options functions.
#
PARSE_AND_LIST_PROLOGUE='
#define OPTION_BASELINE			301
#define OPTION_NO_TARGET_ALIGNED	(OPTION_BASELINE + 1)
#define OPTION_GP_RELATIVE_INSN		(OPTION_BASELINE + 2)
#define OPTION_NO_GP_RELATIVE_INSN	(OPTION_BASELINE + 3)
#define OPTION_EXPORT_SYMBOLS		(OPTION_BASELINE + 4)
#define OPTION_AVOID_BTB_MISS		(OPTION_BASELINE + 5)
#define OPTION_NO_AVOID_BTB_MISS	(OPTION_BASELINE + 6)

/* These are only available to ex9.  */
#if defined RISCV_EX9_EXT
#define OPTION_EX9_BASELINE		320
#define OPTION_EX9_TABLE		(OPTION_EX9_BASELINE + 1)
#define OPTION_NO_EX9_TABLE		(OPTION_EX9_BASELINE + 2)
#define OPTION_EXPORT_EX9		(OPTION_EX9_BASELINE + 3)
#define OPTION_IMPORT_EX9		(OPTION_EX9_BASELINE + 4)
#define OPTION_KEEP_IMPORT_EX9		(OPTION_EX9_BASELINE + 5)
#define OPTION_UPDATE_EX9		(OPTION_EX9_BASELINE + 6)
#define OPTION_EX9_LIMIT		(OPTION_EX9_BASELINE + 7)
#define OPTION_EX9_LOOP			(OPTION_EX9_BASELINE + 8)
#endif
'
PARSE_AND_LIST_LONGOPTS='
  { "mno-target-aligned", no_argument, NULL, OPTION_NO_TARGET_ALIGNED},
  { "mgp-insn", no_argument, NULL, OPTION_GP_RELATIVE_INSN},
  { "mno-gp-insn", no_argument, NULL, OPTION_NO_GP_RELATIVE_INSN},
  { "mexport-symbols", required_argument, NULL, OPTION_EXPORT_SYMBOLS},
  { "mavoid-btb-miss", no_argument, NULL, OPTION_AVOID_BTB_MISS},
  { "mno-avoid-btb-miss", no_argument, NULL, OPTION_NO_AVOID_BTB_MISS},

/* These are specific optioins for ex9-ext support.  */
#if defined RISCV_EX9_EXT
  { "mex9", no_argument, NULL, OPTION_EX9_TABLE},
  { "mno-ex9", no_argument, NULL, OPTION_NO_EX9_TABLE},
  { "mexport-ex9", required_argument, NULL, OPTION_EXPORT_EX9},
  { "mimport-ex9", required_argument, NULL, OPTION_IMPORT_EX9},
  { "mkeep-import-ex9", no_argument, NULL, OPTION_KEEP_IMPORT_EX9},
  { "mupdate-ex9", no_argument, NULL, OPTION_UPDATE_EX9},
  { "mex9-limit", required_argument, NULL, OPTION_EX9_LIMIT},
  { "mex9-loop-aware", no_argument, NULL, OPTION_EX9_LOOP},
#endif
'
PARSE_AND_LIST_OPTIONS='
fprintf (file, _("\
    --mno-target-aligned        Disable target aligned\n\
    --m[no-]gp-insn             Support gp relative instructions\n\
    --mexport-symbols=FILE      Exporting global symbols into linker script\n\
    --m[no-]avoid-btb-miss      Avoid btb miss \n\
"));

#if defined RISCV_EX9_EXT
  fprintf (file, _("\
    --m[no-]ex9                 Disable/enable link-time EX9 relaxation\n\
    --mexport-ex9=FILE          Export EX9 table after linking\n\
    --mimport-ex9=FILE          Import Ex9 table for EX9 relaxation\n\
    --mkeep-import-ex9          Keep import Ex9 table\n\
    --mupdate-ex9               Update existing EX9 table\n\
    --mex9-limit=NUM            Set maximum number of entries in ex9 table for this times\n\
    --mex9-loop-aware           Avoid generate EX9 instruction inside loop\n\
"));
#endif
'
PARSE_AND_LIST_ARGS_CASES='
  case OPTION_NO_TARGET_ALIGNED:
    target_aligned = 0;
    break;
  case OPTION_GP_RELATIVE_INSN:
    gp_relative_insn = 1;
    break;
  case OPTION_NO_GP_RELATIVE_INSN:
    gp_relative_insn = 0;
    break;
  case OPTION_EXPORT_SYMBOLS:
    if (!optarg)
      einfo (_("Missing file for --mexport-symbols.\n"), optarg);

    if(strcmp (optarg, "-") == 0)
      sym_ld_script = stdout;
    else
      {
	sym_ld_script = fopen (optarg, FOPEN_WT);
	if(sym_ld_script == NULL)
	  einfo (_("%P%F: cannot open map file %s: %E.\n"), optarg);
      }
    break;
  case OPTION_AVOID_BTB_MISS:
    avoid_btb_miss = 1;
    break;
  case OPTION_NO_AVOID_BTB_MISS:
    avoid_btb_miss = 0;
    break;

#if defined RISCV_EX9_EXT
  case OPTION_EX9_TABLE:
    target_optimize |= RISCV_RELAX_EX9_ON;
    break;
  case OPTION_NO_EX9_TABLE:
    target_optimize &= ~RISCV_RELAX_EX9_ON;
    break;
  case OPTION_EXPORT_EX9:
    if (!optarg)
      einfo (_("Missing file for --mexport-ex9=<file>.\n"));

      ex9_export_file = optarg;
      /* Open file in the riscv_elf_relocate_ex9_table.  */
      break;
  case OPTION_IMPORT_EX9:
    if (!optarg)
      einfo (_("Missing file for --mimport-ex9=<file>.\n"));

    ex9_import_file = fopen (optarg, "rb+");
    if(ex9_import_file == NULL)
      einfo (_("ERROR %P%F: cannot open ex9 import file %s.\n"), optarg);
    break;
  case OPTION_KEEP_IMPORT_EX9:
    keep_import_ex9 = 1;
    break;
  case OPTION_UPDATE_EX9:
    update_ex9_table = 1;
    break;
  case OPTION_EX9_LIMIT:
    if (optarg)
      {
	ex9_limit = atoi (optarg);
	if (ex9_limit > 512 || ex9_limit < 0)
	  {
	    einfo (_("ERROR: the range of ex9_limit must between 0 and 512 (default 512)\n"));
	    exit (1);
	  }
      }
    break;
  case OPTION_EX9_LOOP:
    ex9_loop_aware = 1;
    break;
#endif
'

LDEMUL_BEFORE_ALLOCATION=riscv_elf_before_allocation
LDEMUL_AFTER_ALLOCATION=gld${EMULATION_NAME}_after_allocation
LDEMUL_AFTER_OPEN=riscv_elf_after_open
LDEMUL_CREATE_OUTPUT_SECTION_STATEMENTS=riscv_create_output_section_statements
