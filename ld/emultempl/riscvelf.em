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

#define RISCV_EXECIT_EXT
static int target_optimize = 0;		/* Switch optimization.  */
static int relax_status = 0;		/* Finish optimization.  */
static char *execit_export_file = NULL;	/* Export the .exec.itable.  */
static FILE *execit_import_file = NULL;	/* Do EXECIT according to the imported
					   .exec.itable.  */
static int keep_import_execit = 0;	/* Keep the imported .exec.itable.  */
static int update_execit_table = 0;
static int execit_limit = -1;		/* Default set it to 1024 entries.  */
static int execit_loop_aware = 0;	/* --mexecit-loop-aware.  */

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
  table->execit_export_file = execit_export_file;
  table->execit_import_file = execit_import_file;
  table->keep_import_execit = keep_import_execit;
  table->update_execit_table = update_execit_table;
  table->execit_limit = execit_limit;
  table->execit_loop_aware = execit_loop_aware;
}

static void
riscv_elf_append_section (struct bfd_link_info *info, bfd *abfd)
{
  asection *itable;
  struct bfd_link_hash_entry *h;

  if (target_optimize & RISCV_RELAX_EXECIT_ON
      && (execit_import_file == NULL
	  || keep_import_execit
	  || update_execit_table))
    {
      /* Create section ".exec.itable".  */
      itable = bfd_make_section_with_flags (abfd, ".exec.itable",
					    SEC_CODE | SEC_ALLOC | SEC_LOAD
					    | SEC_HAS_CONTENTS | SEC_READONLY
					    | SEC_IN_MEMORY | SEC_KEEP
					    | SEC_RELOC);
      if (itable)
	{
	  itable->gc_mark = 1;
	  itable->alignment_power = 2;
	  /* Default size of .exec.itable can not be zero, so we can not set
	     it according to execit_limit. Since we will adjust the table size
	     in riscv_elf_execit_build_itable, it is okay to set the size to
	     the maximum value here.  */
	  itable->size = 0x1000;
	  itable->contents = bfd_zalloc (abfd, itable->size);

	  /* Add a symbol in the head of .exec.itable to objdump clearly.  */
	  h = bfd_link_hash_lookup (info->hash, "_EXECIT_BASE_",
				    FALSE, FALSE, FALSE);
	  _bfd_generic_link_add_one_symbol
	    (info, info->output_bfd, "_EXECIT_BASE_",
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

/* These are only available to EXECIT.  */
#if defined RISCV_EXECIT_EXT
#define OPTION_EXECIT_BASELINE		320
#define OPTION_EXECIT_TABLE		(OPTION_EXECIT_BASELINE + 1)
#define OPTION_EX9_TABLE		(OPTION_EXECIT_BASELINE + 2)
#define OPTION_NO_EXECIT_TABLE		(OPTION_EXECIT_BASELINE + 3)
#define OPTION_EXPORT_EXECIT		(OPTION_EXECIT_BASELINE + 4)
#define OPTION_IMPORT_EXECIT		(OPTION_EXECIT_BASELINE + 5)
#define OPTION_KEEP_IMPORT_EXECIT	(OPTION_EXECIT_BASELINE + 6)
#define OPTION_UPDATE_EXECIT		(OPTION_EXECIT_BASELINE + 7)
#define OPTION_EXECIT_LIMIT		(OPTION_EXECIT_BASELINE + 8)
#define OPTION_EXECIT_LOOP		(OPTION_EXECIT_BASELINE + 9)
#endif

/* These are only for lld internal usage and not affected for bfd.  */
#define OPTION_LLD_COMPATIBLE_BASELINE	340
#define OPTION_RELAX_GP_TO_RODATA	(OPTION_LLD_COMPATIBLE_BASELINE + 1)
'
PARSE_AND_LIST_LONGOPTS='
  { "mno-target-aligned", no_argument, NULL, OPTION_NO_TARGET_ALIGNED},
  { "mgp-insn-relax", no_argument, NULL, OPTION_GP_RELATIVE_INSN},
  { "mno-gp-insn-relax", no_argument, NULL, OPTION_NO_GP_RELATIVE_INSN},
  { "mexport-symbols", required_argument, NULL, OPTION_EXPORT_SYMBOLS},
  { "mavoid-btb-miss", no_argument, NULL, OPTION_AVOID_BTB_MISS},
  { "mno-avoid-btb-miss", no_argument, NULL, OPTION_NO_AVOID_BTB_MISS},

/* These are specific optioins for EXECIT support.  */
#if defined RISCV_EXECIT_EXT
  { "mexecit", no_argument, NULL, OPTION_EXECIT_TABLE},
  { "mno-execit", no_argument, NULL, OPTION_NO_EXECIT_TABLE},
  { "mexport-execit", required_argument, NULL, OPTION_EXPORT_EXECIT},
  { "mimport-execit", required_argument, NULL, OPTION_IMPORT_EXECIT},
  { "mkeep-import-execit", no_argument, NULL, OPTION_KEEP_IMPORT_EXECIT},
  { "mupdate-execit", no_argument, NULL, OPTION_UPDATE_EXECIT},
  { "mexecit-limit", required_argument, NULL, OPTION_EXECIT_LIMIT},
  { "mexecit-loop-aware", no_argument, NULL, OPTION_EXECIT_LOOP},
  /* Obsolete options for EXECIT.  */
  { "mex9", no_argument, NULL, OPTION_EX9_TABLE},
  { "mno-ex9", no_argument, NULL, OPTION_NO_EXECIT_TABLE},
  { "mexport-ex9", required_argument, NULL, OPTION_EXPORT_EXECIT},
  { "mimport-ex9", required_argument, NULL, OPTION_IMPORT_EXECIT},
  { "mkeep-import-ex9", no_argument, NULL, OPTION_KEEP_IMPORT_EXECIT},
  { "mupdate-ex9", no_argument, NULL, OPTION_UPDATE_EXECIT},
  { "mex9-limit", required_argument, NULL, OPTION_EXECIT_LIMIT},
  { "mex9-loop-aware", no_argument, NULL, OPTION_EXECIT_LOOP},
#endif

/* These are only for lld internal usage and not affected for bfd.  */
  {"mrelax-gp-to-rodata", no_argument, NULL, OPTION_RELAX_GP_TO_RODATA},
'
PARSE_AND_LIST_OPTIONS='
fprintf (file, _("\
    --mno-target-aligned        Disable target aligned\n\
    --m[no-]gp-insn             Support gp relative instructions\n\
    --mexport-symbols=FILE      Exporting global symbols into linker script\n\
    --m[no-]avoid-btb-miss      Avoid btb miss \n\
"));

#if defined RISCV_EXECIT_EXT
  fprintf (file, _("\
    --m[no-]execit              Disable/enable link-time EXECIT relaxation\n\
    --mexport-execit=FILE       Export .exec.itable after linking\n\
    --mimport-execit=FILE       Import .exec.itable for EXECIT relaxation\n\
    --mkeep-import-execit       Keep imported .exec.itable\n\
    --mupdate-execit            Update existing .exec.itable\n\
    --mexecit-limit=NUM         Set maximum number of entries in .exec.itable for this times\n\
    --mexecit-loop-aware        Avoid generate exec.it instruction inside loop\n\
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

#if defined RISCV_EXECIT_EXT
  case OPTION_EX9_TABLE:
    if (execit_limit == -1
	|| execit_limit > 512)
    execit_limit = 512;
    /* FALL THROUGH.  */
  case OPTION_EXECIT_TABLE:
    target_optimize |= RISCV_RELAX_EXECIT_ON;
    break;
  case OPTION_NO_EXECIT_TABLE:
    target_optimize &= ~RISCV_RELAX_EXECIT_ON;
    break;
  case OPTION_EXPORT_EXECIT:
    if (!optarg)
      einfo (_("Missing file for --mexport-execit=<file>.\n"));

      execit_export_file = optarg;
      /* Open file in the riscv_elf_relocate_execit_table.  */
      break;
  case OPTION_IMPORT_EXECIT:
    if (!optarg)
      einfo (_("Missing file for --mimport-execit=<file>.\n"));

    execit_import_file = fopen (optarg, "rb+");
    if(execit_import_file == NULL)
      einfo (_("ERROR %P%F: cannot open execit import file %s.\n"), optarg);
    break;
  case OPTION_KEEP_IMPORT_EXECIT:
    keep_import_execit = 1;
    break;
  case OPTION_UPDATE_EXECIT:
    update_execit_table = 1;
    break;
  case OPTION_EXECIT_LIMIT:
    if (optarg)
      {
	if (execit_limit != -1
	    && atoi (optarg) > execit_limit)
	  einfo (_("Warning: the value of execit_limit (%d) is larger "
		   "than the current setting (%d)\n"),
		 atoi (optarg), execit_limit);
	else
	  execit_limit = atoi (optarg);

	if (execit_limit > 1024 || execit_limit < 0)
	  {
	    einfo (_("ERROR: the range of execit_limit must between "
		     "0 and 1024 (default 1024)\n"));
	    exit (1);
	  }
      }
    break;
  case OPTION_EXECIT_LOOP:
    execit_loop_aware = 1;
    break;
#endif
  case OPTION_RELAX_GP_TO_RODATA:
    /* Do nothing.  */
    break;
'

LDEMUL_BEFORE_ALLOCATION=riscv_elf_before_allocation
LDEMUL_AFTER_ALLOCATION=gld${EMULATION_NAME}_after_allocation
LDEMUL_AFTER_OPEN=riscv_elf_after_open
LDEMUL_CREATE_OUTPUT_SECTION_STATEMENTS=riscv_create_output_section_statements
