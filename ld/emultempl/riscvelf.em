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

# default value for set_relax_cross_section_call: 0/elf, 1/linux
# hack: by referring GENERATE_SHLIB_SCRIPT within
#       ld/emulparams/elf32lriscv-defs.sh
D4_RCSC=$(test -n "$GENERATE_SHLIB_SCRIPT" && echo 1 || echo 0)

fragment <<EOF

#include "ldmain.h"
#include "ldctor.h"
#include "elf/riscv.h"
#include "elfxx-riscv.h"
#include "elf-bfd.h"
#include "libbfd.h"

static FILE *sym_ld_script = NULL;      /* Export global symbols into linker
					   script.  */

static int set_relax_align = 1;		/* Defalut do relax align.  */
static int target_aligned = 1;		/* Defalut do target aligned.  */
static int gp_relative_insn = 0;	/* Support gp relative insn.  */
static int avoid_btb_miss = 1;		/* Default avoid BTB miss.  */
static int set_relax_lui = 1;		/* Defalut do relax lui.  */
static int set_relax_pc = 1;		/* Defalut do relax pc.  */
static int set_relax_call = 1;		/* Defalut do relax call.  */
static int set_relax_tls_le = 1;	/* Defalut do relax tls le.  */
static int set_relax_cross_section_call = $D4_RCSC;	/* Defalut do relax cross section call.  */

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
static bfd_boolean execit_noji = 0;
static bfd_boolean execit_nols = 0;

/* Put target dependent option into info hash table.  */

static void
riscv_elf_set_target_option (struct bfd_link_info *info)
{
  struct riscv_elf_link_hash_table *table;

  table = riscv_elf_hash_table (info);
  if (table == NULL)
    return;

  table->sym_ld_script = sym_ld_script;

  table->gp_relative_insn = gp_relative_insn;
  table->set_relax_align = set_relax_align;
  table->target_aligned = target_aligned;
  table->avoid_btb_miss = avoid_btb_miss;
  table->set_relax_lui = set_relax_lui;
  table->set_relax_pc = set_relax_pc;
  table->set_relax_call = set_relax_call;
  table->set_relax_tls_le = set_relax_tls_le;
  table->set_relax_cross_section_call = set_relax_cross_section_call;

  table->target_optimize = target_optimize;
  table->relax_status = relax_status;
  table->execit_export_file = execit_export_file;
  table->execit_import_file = execit_import_file;
  table->keep_import_execit = keep_import_execit;
  table->update_execit_table = update_execit_table;
  table->execit_limit = execit_limit;
  table->execit_loop_aware = execit_loop_aware;
  table->execit_noji = execit_noji;
  table->execit_nols = execit_nols;
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

  link_info.relax_pass = 8;
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

  riscv_elf_create_output_section_statements ();
}

/* Create the target usage section for RISCV.  */

static void
riscv_elf_create_target_section (struct bfd_link_info *info, bfd *abfd,
				 char *sec_name, char *sym_name,
				 bfd_size_type sec_size,
				 unsigned int sec_aligned_power,
				 flagword flags)
{
  asection *itable;
  struct bfd_link_hash_entry *h;

  /* Create section.  */
  itable = bfd_make_section_with_flags (abfd, sec_name, flags);
  if (itable)
    {
      itable->size = sec_size;
      itable->alignment_power = sec_aligned_power;
      itable->contents = bfd_zalloc (abfd, itable->size);
      itable->gc_mark = 1;

      /* Add a symbol in the head of target section to objdump clearly.  */
      h = bfd_link_hash_lookup (info->hash, sym_name,
				FALSE, FALSE, FALSE);
      _bfd_generic_link_add_one_symbol
	(info, info->output_bfd, sym_name,
	 BSF_GLOBAL | BSF_WEAK, itable, 0, (const char *) NULL, FALSE,
	 get_elf_backend_data (info->output_bfd)->collect, &h);
    }
}

static void
riscv_elf_after_open (void)
{
  bfd *abfd;
  flagword flags;

  flags = (SEC_CODE | SEC_ALLOC | SEC_LOAD
	   | SEC_HAS_CONTENTS | SEC_READONLY
	   | SEC_IN_MEMORY | SEC_KEEP
	   | SEC_RELOC);

  if ((target_optimize & RISCV_RELAX_EXECIT_ON)
      && (execit_import_file == NULL
	  || keep_import_execit
	  || update_execit_table))
    {
      for (abfd = link_info.input_bfds; abfd != NULL; abfd = abfd->link.next)
	{
	  /* Create execit section in the last input object file.  */
	  /* Default size of .exec.itable can not be zero, so we can not set
	     it according to execit_limit. Since we will adjust the table size
	     in riscv_elf_execit_build_itable, it is okay to set the size to
	     the maximum value 0x1000 here.  */
	  if (abfd->link.next == NULL)
	    riscv_elf_create_target_section (&link_info, abfd, ".exec.itable",
					     "_EXECIT_BASE_", 0x1000, 2, flags);
	}
    }

  /* The ict table is imported in this link time.  */
  asection *sec;
  for (abfd = link_info.input_bfds; abfd != NULL; abfd = abfd->link.next)
    {
      sec = bfd_get_section_by_name (abfd, ".nds.ict");
      if (sec)
	{
	  find_imported_ict_table = TRUE;
	  break;
	}
    }

  /* Call the standard elf routine.  */
  gld${EMULATION_NAME}_after_open ();
}

static void
riscv_elf_after_check_relocs (void)
{
  bfd *abfd;
  flagword flags;

  if (ict_model == 2)
    flags = (SEC_DATA | SEC_ALLOC | SEC_LOAD
	     | SEC_HAS_CONTENTS | SEC_READONLY
	     | SEC_IN_MEMORY | SEC_KEEP
	     | SEC_RELOC);
  else
    flags = (SEC_CODE | SEC_ALLOC | SEC_LOAD
	     | SEC_HAS_CONTENTS | SEC_READONLY
	     | SEC_IN_MEMORY | SEC_KEEP
	     | SEC_RELOC);

  /* We only create the ict table and _INDIRECT_CALL_TABLE_BASE_ symbol
     when we compiling the main project at the first link-time.  */
  if (!find_imported_ict_table
      && ict_table_entries > 0)
    {
      for (abfd = link_info.input_bfds; abfd != NULL; abfd = abfd->link.next)
	{
	  /* Create ict table section in the last input object file.  */
	  /* The ict_table_entries has been set in the check_relocs.  */
	  if (abfd->link.next == NULL)
	    riscv_elf_create_target_section (&link_info, abfd, ".nds.ict",
					     "_INDIRECT_CALL_TABLE_BASE_",
					     ict_table_entries * 4 * 2, 2,
					     flags);
	}
    }
}
EOF
# Define some shell vars to insert bits of code into the standard elf
# parse_args and list_options functions.
#
PARSE_AND_LIST_PROLOGUE='
#define OPTION_BASELINE			300
#define OPTION_EXPORT_SYMBOLS		(OPTION_BASELINE + 1)

/* These are only for internal usage and debugging.  */
#define OPTION_INTERNAL_BASELINE	310
/* Relax lui to nds v5 gp insn.  */
#define OPTION_GP_RELATIVE_INSN		(OPTION_INTERNAL_BASELINE + 1)
#define OPTION_NO_GP_RELATIVE_INSN	(OPTION_INTERNAL_BASELINE + 2)
/* Alignment relaxations.  */
#define OPTION_NO_RELAX_ALIGN		(OPTION_INTERNAL_BASELINE + 3)
#define OPTION_NO_TARGET_ALIGNED	(OPTION_INTERNAL_BASELINE + 4)
#define OPTION_AVOID_BTB_MISS		(OPTION_INTERNAL_BASELINE + 5)
#define OPTION_NO_AVOID_BTB_MISS	(OPTION_INTERNAL_BASELINE + 6)
/* Disbale specific relaxation.  */
#define OPTION_NO_RELAX_LUI		(OPTION_INTERNAL_BASELINE + 7)
#define OPTION_NO_RELAX_PC		(OPTION_INTERNAL_BASELINE + 8)
#define OPTION_NO_RELAX_CALL		(OPTION_INTERNAL_BASELINE + 9)
#define OPTION_NO_RELAX_TLS_LE		(OPTION_INTERNAL_BASELINE + 10)
#define OPTION_RELAX_CROSS_SECTION_CALL		(OPTION_INTERNAL_BASELINE + 11)
#define OPTION_NO_RELAX_CROSS_SECTION_CALL		(OPTION_INTERNAL_BASELINE + 12)

/* These are only available to EXECIT.  */
#if defined RISCV_EXECIT_EXT
#define OPTION_EXECIT_BASELINE		340
#define OPTION_EXECIT_TABLE		(OPTION_EXECIT_BASELINE + 1)
#define OPTION_EX9_TABLE		(OPTION_EXECIT_BASELINE + 2)
#define OPTION_NO_EXECIT_TABLE		(OPTION_EXECIT_BASELINE + 3)
#define OPTION_EXPORT_EXECIT		(OPTION_EXECIT_BASELINE + 4)
#define OPTION_IMPORT_EXECIT		(OPTION_EXECIT_BASELINE + 5)
#define OPTION_KEEP_IMPORT_EXECIT	(OPTION_EXECIT_BASELINE + 6)
#define OPTION_UPDATE_EXECIT		(OPTION_EXECIT_BASELINE + 7)
#define OPTION_EXECIT_LIMIT		(OPTION_EXECIT_BASELINE + 8)
#define OPTION_EXECIT_LOOP		(OPTION_EXECIT_BASELINE + 9)
#define OPTION_EXECIT_NO_JI		(OPTION_EXECIT_BASELINE + 10)
#define OPTION_EXECIT_NO_LS		(OPTION_EXECIT_BASELINE + 11)
#define OPTION_EXECIT_NO_REL		(OPTION_EXECIT_BASELINE + 12)
#endif

/* These are only for lld internal usage and not affected for bfd.  */
#define OPTION_LLD_COMPATIBLE_BASELINE	360
#define OPTION_LLD_COMPATIBLE		360
#define OPTION_BEST_GP			(OPTION_LLD_COMPATIBLE_BASELINE + 1)
#define OPTION_EXECIT_OPT_DATA		(OPTION_LLD_COMPATIBLE_BASELINE + 2)
#define OPTION_EXECIT_OPT_RODATA	(OPTION_LLD_COMPATIBLE_BASELINE + 3)
#define OPTION_EXECIT_SEPERATE_CALL	(OPTION_LLD_COMPATIBLE_BASELINE + 4)
#define OPTION_RELAX_GP_TO_RODATA	(OPTION_LLD_COMPATIBLE_BASELINE + 5)
/*#define OPTION_RELAX_CROSS_SECTION_CALL	(OPTION_LLD_COMPATIBLE_BASELINE + 6) implemented */
#define OPTION_DEBUG_EXECIT_LIMIT	(OPTION_LLD_COMPATIBLE_BASELINE + 7)
#define OPTION_NO_EXECIT_AUIPC		(OPTION_LLD_COMPATIBLE_BASELINE + 8)
#define OPTION_NO_EXECIT_LUI		(OPTION_LLD_COMPATIBLE_BASELINE + 9)
#define OPTION_ALLOW_INCOMPATIBLE_ATTR	(OPTION_LLD_COMPATIBLE_BASELINE + 10)
#define OPTION_FULL_SHUTDOWN		(OPTION_LLD_COMPATIBLE_BASELINE + 11)
'
PARSE_AND_LIST_LONGOPTS='
  { "mexport-symbols", required_argument, NULL, OPTION_EXPORT_SYMBOLS},
  { "no-integrated-as", no_argument, NULL, OPTION_LLD_COMPATIBLE},
  { "as-opt", required_argument, NULL, OPTION_LLD_COMPATIBLE},

/* Generally, user does not need to set these options by themselves.  */
  { "mgp-insn-relax", no_argument, NULL, OPTION_GP_RELATIVE_INSN},
  { "mno-gp-insn-relax", no_argument, NULL, OPTION_NO_GP_RELATIVE_INSN},
  { "mno-relax-align", no_argument, NULL, OPTION_NO_RELAX_ALIGN},
  { "mno-target-aligned", no_argument, NULL, OPTION_NO_TARGET_ALIGNED},
  { "mavoid-btb-miss", no_argument, NULL, OPTION_AVOID_BTB_MISS},
  { "mno-avoid-btb-miss", no_argument, NULL, OPTION_NO_AVOID_BTB_MISS},
  { "mno-relax-lui", no_argument, NULL, OPTION_NO_RELAX_LUI},
  { "mno-relax-pcrel", no_argument, NULL, OPTION_NO_RELAX_PC},
  { "mno-relax-call", no_argument, NULL, OPTION_NO_RELAX_CALL},
  { "mno-relax-tls", no_argument, NULL, OPTION_NO_RELAX_TLS_LE},
  { "mrelax-cross-section-call", no_argument, NULL, OPTION_RELAX_CROSS_SECTION_CALL},
  { "mno-relax-cross-section-call", no_argument, NULL, OPTION_NO_RELAX_CROSS_SECTION_CALL},

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
  { "mexecit-noji", no_argument, NULL, OPTION_EXECIT_NO_JI},
  { "mexecit-nols", no_argument, NULL, OPTION_EXECIT_NO_LS},
  { "mexecit-norel", no_argument, NULL, OPTION_EXECIT_NO_REL},
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
  { "mbest-gp", no_argument, NULL, OPTION_BEST_GP},
  { "mexecit_opt_data", no_argument, NULL, OPTION_EXECIT_OPT_DATA},
  { "mexecit_opt_rodata", no_argument, NULL, OPTION_EXECIT_OPT_RODATA},
  { "mexecit_opt_seperate_call", no_argument, NULL, OPTION_EXECIT_SEPERATE_CALL},
  { "mrelax-gp-to-rodata", no_argument, NULL, OPTION_RELAX_GP_TO_RODATA},
  { "mdebug-execit-limit", required_argument, NULL, OPTION_DEBUG_EXECIT_LIMIT},
  { "mno-execit-auipc", no_argument, NULL, OPTION_NO_EXECIT_AUIPC},
  { "mno-execit-lui", no_argument, NULL, OPTION_NO_EXECIT_LUI},
  { "mallow-incompatible-attributes", no_argument, NULL, OPTION_ALLOW_INCOMPATIBLE_ATTR},
  {"full-shutdown", no_argument, NULL, OPTION_FULL_SHUTDOWN},
'
PARSE_AND_LIST_OPTIONS='
fprintf (file, _("\
    --mexport-symbols=FILE      Exporting global symbols into linker script\n\
    --m[no-]relax-cross-section-call Disable/enable cross-section relaxations\n\
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

  case OPTION_GP_RELATIVE_INSN:
    gp_relative_insn = 1;
    break;
  case OPTION_NO_GP_RELATIVE_INSN:
    gp_relative_insn = 0;
    break;
  case OPTION_NO_RELAX_ALIGN:
    set_relax_align = 0;
    break;
  case OPTION_NO_TARGET_ALIGNED:
    target_aligned = 0;
    break;
  case OPTION_AVOID_BTB_MISS:
    avoid_btb_miss = 1;
    break;
  case OPTION_NO_AVOID_BTB_MISS:
    avoid_btb_miss = 0;
    break;
  case OPTION_NO_RELAX_LUI:
    set_relax_lui = 0;
    break;
  case OPTION_NO_RELAX_PC:
    set_relax_pc = 0;
    break;
  case OPTION_NO_RELAX_CALL:
    set_relax_call = 0;
    break;
  case OPTION_NO_RELAX_TLS_LE:
    set_relax_tls_le = 0;
    break;
  case OPTION_RELAX_CROSS_SECTION_CALL:
    set_relax_cross_section_call = 1;
    break;
  case OPTION_NO_RELAX_CROSS_SECTION_CALL:
    set_relax_cross_section_call = 0;
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
  case OPTION_EXECIT_NO_JI:
    execit_noji = 1;
    break;
  case OPTION_EXECIT_NO_LS:
    execit_nols = 1;
    break;
  case OPTION_EXECIT_NO_REL:
    execit_noji = 1;
    execit_nols = 1;
    break;
#endif
  case OPTION_DEBUG_EXECIT_LIMIT:
    if (optarg)
      {
	/* Do nothing.  */
      }
    break;
  case OPTION_LLD_COMPATIBLE:
  case OPTION_BEST_GP:
  case OPTION_EXECIT_OPT_DATA:
  case OPTION_EXECIT_OPT_RODATA:
  case OPTION_EXECIT_SEPERATE_CALL:
  case OPTION_RELAX_GP_TO_RODATA:
  case OPTION_NO_EXECIT_AUIPC:
  case OPTION_NO_EXECIT_LUI:
  case OPTION_ALLOW_INCOMPATIBLE_ATTR:
  case OPTION_FULL_SHUTDOWN:
    /* Do nothing.  */
    break;
'

LDEMUL_BEFORE_ALLOCATION=riscv_elf_before_allocation
LDEMUL_AFTER_ALLOCATION=gld${EMULATION_NAME}_after_allocation
LDEMUL_AFTER_OPEN=riscv_elf_after_open
LDEMUL_CREATE_OUTPUT_SECTION_STATEMENTS=riscv_create_output_section_statements
LDEMUL_AFTER_CHECK_RELOCS=riscv_elf_after_check_relocs
