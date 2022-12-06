# This shell script emits a C file. -*- C -*-
#   Copyright (C) 2004-2022 Free Software Foundation, Inc.
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
# D4_RCSC=$(test -n "$GENERATE_SHLIB_SCRIPT" && echo 1 || echo 0)
D4_RCSC=0 # default off for both elf/linux tool-chain

fragment <<EOF

#include "ldmain.h"
#include "ldctor.h"
#include "elf/riscv.h"
#include "elfxx-riscv.h"
/* { Andes */
#include "libbfd.h"
/* } Andes */

/* { Andes  */
#define RISCV_ANDES_INTERNAL_OPTIONS
#define RISCV_EXECIT_EXT
static andes_ld_options_t andes =
{
  .sym_ld_script = NULL,
  .set_relax_align = 1,
  .target_aligned = 1,
  .gp_relative_insn = 0,
  .avoid_btb_miss = 1,
  .set_relax_lui = 1,
  .set_relax_pc = 1,
  .set_relax_call = 1,
  .set_relax_tls_le = 1,
  .set_relax_cross_section_call = $D4_RCSC,
  .set_workaround = 1,
  /* exec.it options  */
  .execit_import_file = NULL,
  .execit_export_file = NULL,
  .target_optimization = 0,
  .execit_limit = -1, /* default */
  .execit_flags = {0},
  .update_execit_table = 0,
  .keep_import_execit = 0,
  .execit_loop_aware = 0,
  .execit_jal_over_2m = 0,
  /* andes internal options.  */
  .set_table_jump = 0,
};
/* } Andes  */

/* { Andes  */
static void
riscv_elf_set_target_option (struct bfd_link_info *info)
{
  struct riscv_elf_link_hash_table *htab;

  htab = riscv_elf_hash_table (info);
  if (htab == NULL)
    return;

  htab->andes = andes;
}
/* { Andes  */

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

  link_info.relax_pass = 12;
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

  /* PR 27566, if the phase of data segment is exp_seg_relro_adjust,
     that means we are still adjusting the relro, and shouldn't do the
     relaxations at this stage.  Otherwise, we will get the symbol
     values beofore handling the relro, and may cause truncated fails
     when the relax range crossing the data segment.  One of the solution
     is to monitor the data segment phase while relaxing, to know whether
     the relro has been handled or not.

     I think we probably need to record more information about data
     segment or alignments in the future, to make sure it is safe
     to doing relaxations.  */
  enum phase_enum *phase = &(expld.dataseg.phase);
  bfd_elf${ELFSIZE}_riscv_set_data_segment_info (&link_info, (int *) phase);

  ldelf_map_segments (need_layout);

  /* { Andes */
  /* Add a symbol for linker script check the max size.  */
  if (link_info.output_bfd->sections)
    {
      struct bfd_link_hash_entry *h;
      h = bfd_link_hash_lookup (link_info.hash, "_RELAX_END_",
				false, false, false);
      if (!h)
	_bfd_generic_link_add_one_symbol
	  (&link_info, link_info.output_bfd, "_RELAX_END_",
	   BSF_GLOBAL | BSF_WEAK, link_info.output_bfd->sections,
	   0, (const char *) NULL, false,
	   get_elf_backend_data (link_info.output_bfd)->collect, &h);
    }
  /* } Andes */
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

  riscv_elf_set_target_option (&link_info);
}

/* { Andes */
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
				false, false, false);
      _bfd_generic_link_add_one_symbol
	(info, info->output_bfd, sym_name,
	 BSF_GLOBAL | BSF_WEAK, itable, 0, (const char *) NULL, false,
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

  if ((andes.target_optimization & RISCV_RELAX_EXECIT_ON)
      && (andes.execit_import_file == NULL
	  || andes.keep_import_execit
	  || andes.update_execit_table))
    {
      for (abfd = link_info.input_bfds; abfd != NULL; abfd = abfd->link.next)
	{
	  /* Create execit section in the last input object file.  */
	  /* Default size of .exec.itable can not be zero, so we can not set
	     it according to execit_limit. Since we will adjust the table size
	     in riscv_elf_execit_build_itable, it is okay to set the size to
	     the maximum value 0x1000 here.  */
	  if (abfd->link.next == NULL)
	    riscv_elf_create_target_section (&link_info, abfd, EXECIT_SECTION,
					     "_EXECIT_BASE_", 0x1000, 2, flags);
	}
    }

  /* The ict table is imported in this link time.  */
  asection *sec;
  for (abfd = link_info.input_bfds; abfd != NULL; abfd = abfd->link.next)
    {
      sec = bfd_get_section_by_name (abfd, ANDES_ICT_SECTION);
      if (sec)
	{
	  find_imported_ict_table = true;
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
  int size = get_ict_size ();
  if (!find_imported_ict_table
      && (ict_table_entries || size))
    {
      for (abfd = link_info.input_bfds; abfd != NULL; abfd = abfd->link.next)
	{
	  /* Create ict table section in the last input object file.  */
	  /* The ict_table_entries has been set in the check_relocs.  */
	  if (abfd->link.next == NULL)
	    riscv_elf_create_target_section (&link_info, abfd,
					     ANDES_ICT_SECTION,
					     "_INDIRECT_CALL_TABLE_BASE_",
					     size, 2, flags);
	}
    }
}

/* } Andes */

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
#define OPTION_RELAX_CROSS_SECTION_CALL	(OPTION_INTERNAL_BASELINE + 11)
#define OPTION_NO_RELAX_CROSS_SECTION_CALL	(OPTION_INTERNAL_BASELINE + 12)
#define OPTION_NO_WORKAROUND		(OPTION_INTERNAL_BASELINE + 13)

/* These are only available for Andes internal options.  */
#if defined RISCV_ANDES_INTERNAL_OPTIONS
#define OPTION_NO_OPT_TABLE_JUMP	(OPTION_INTERNAL_BASELINE + 28)
#define OPTION_OPT_TABLE_JUMP		(OPTION_INTERNAL_BASELINE + 29)
#endif

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
#define OPTION_EXECIT_AUIPC		(OPTION_EXECIT_BASELINE + 13)
#define OPTION_NO_EXECIT_AUIPC		(OPTION_EXECIT_BASELINE + 14)
#define OPTION_EXECIT_JAL		(OPTION_EXECIT_BASELINE + 15)
#define OPTION_NO_EXECIT_JAL		(OPTION_EXECIT_BASELINE + 16)
#define OPTION_EXECIT_JAL_OVER_2M	(OPTION_EXECIT_BASELINE + 17)
#define OPTION_NO_EXECIT_JAL_OVER_2M	(OPTION_EXECIT_BASELINE + 18)
#define OPTION_EXECIT_FLS		(OPTION_EXECIT_BASELINE + 19)
#define OPTION_NO_EXECIT_FLS		(OPTION_EXECIT_BASELINE + 20)
#define OPTION_EXECIT_RVV		(OPTION_EXECIT_BASELINE + 21)
#define OPTION_NO_EXECIT_RVV		(OPTION_EXECIT_BASELINE + 22)
#define OPTION_EXECIT_XDSP		(OPTION_EXECIT_BASELINE + 23)
#define OPTION_NO_EXECIT_XDSP		(OPTION_EXECIT_BASELINE + 24)
#define OPTION_EXECIT_RVP		(OPTION_EXECIT_BASELINE + 25)
#define OPTION_NO_EXECIT_RVP		(OPTION_EXECIT_BASELINE + 26)
#define OPTION_NEXECIT_OP		(OPTION_EXECIT_BASELINE + 27)
#endif

/* These are only for lld internal usage and not affected for bfd.  */
#define OPTION_LLD_COMPATIBLE_BASELINE	370
#define OPTION_LLD_COMPATIBLE		370
#define OPTION_BEST_GP			(OPTION_LLD_COMPATIBLE_BASELINE + 1)
#define OPTION_EXECIT_OPT_DATA		(OPTION_LLD_COMPATIBLE_BASELINE + 2)
#define OPTION_EXECIT_OPT_RODATA	(OPTION_LLD_COMPATIBLE_BASELINE + 3)
#define OPTION_EXECIT_SEPERATE_CALL	(OPTION_LLD_COMPATIBLE_BASELINE + 4)
#define OPTION_RELAX_GP_TO_RODATA	(OPTION_LLD_COMPATIBLE_BASELINE + 5)
#define OPTION_DEBUG_EXECIT_LIMIT	(OPTION_LLD_COMPATIBLE_BASELINE + 6)
#define OPTION_NO_EXECIT_LUI		(OPTION_LLD_COMPATIBLE_BASELINE + 7)
#define OPTION_ALLOW_INCOMPATIBLE_ATTR	(OPTION_LLD_COMPATIBLE_BASELINE + 8)
#define OPTION_FULL_SHUTDOWN		(OPTION_LLD_COMPATIBLE_BASELINE + 9)
'

PARSE_AND_LIST_LONGOPTS='
  { "mexport-symbols", required_argument, NULL, OPTION_EXPORT_SYMBOLS},

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
  { "mno-workaround", no_argument, NULL, OPTION_NO_WORKAROUND},

/* These are specific options for EXECIT support.  */
#if defined RISCV_ANDES_INTERNAL_OPTIONS
  { "mopt-table-jump", no_argument, NULL, OPTION_OPT_TABLE_JUMP},
  { "mno-opt-table-jump", no_argument, NULL, OPTION_NO_OPT_TABLE_JUMP},
#endif

/* These are specific options for EXECIT support.  */
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
  { "mexecit-jal", no_argument, NULL, OPTION_EXECIT_JAL},
  { "mno-execit-jal", no_argument, NULL, OPTION_EXECIT_NO_JI},
  { "mexecit-auipc", no_argument, NULL, OPTION_EXECIT_AUIPC},
  { "mno-execit-auipc", no_argument, NULL, OPTION_NO_EXECIT_AUIPC},
  { "mexecit-jal-over-2mib", no_argument, NULL, OPTION_EXECIT_JAL_OVER_2M},
  { "mno-execit-jal-over-2mib", no_argument, NULL, OPTION_NO_EXECIT_JAL_OVER_2M},
  { "mexecit-rvv", no_argument, NULL, OPTION_EXECIT_RVV},
  { "mno-execit-rvv", no_argument, NULL, OPTION_NO_EXECIT_RVV},
  { "mexecit-fls", no_argument, NULL, OPTION_EXECIT_FLS},
  { "mno-execit-fls", no_argument, NULL, OPTION_NO_EXECIT_FLS},
  { "mexecit-rvp", no_argument, NULL, OPTION_EXECIT_RVP},
  { "mno-execit-rvp", no_argument, NULL, OPTION_NO_EXECIT_RVP},
  { "mexecit-xdsp", no_argument, NULL, OPTION_EXECIT_XDSP},
  { "mno-execit-xdsp", no_argument, NULL, OPTION_NO_EXECIT_XDSP},
  { "mnexecitop", no_argument, NULL, OPTION_NEXECIT_OP},
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

/* These are only for lld compatibility, and take no affected for bfd.  */
  { "no-integrated-as", no_argument, NULL, OPTION_LLD_COMPATIBLE},
  { "as-opt", required_argument, NULL, OPTION_LLD_COMPATIBLE},
  { "mbest-gp", no_argument, NULL, OPTION_BEST_GP},
  { "mexecit_opt_data", no_argument, NULL, OPTION_EXECIT_OPT_DATA},
  { "mexecit_opt_rodata", no_argument, NULL, OPTION_EXECIT_OPT_RODATA},
  { "mexecit_opt_seperate_call", no_argument, NULL, OPTION_EXECIT_SEPERATE_CALL},
  { "mrelax-gp-to-rodata", no_argument, NULL, OPTION_RELAX_GP_TO_RODATA},
  { "mdebug-execit-limit", required_argument, NULL, OPTION_DEBUG_EXECIT_LIMIT},
  { "mno-execit-lui", no_argument, NULL, OPTION_NO_EXECIT_LUI},
  { "mallow-incompatible-attributes", no_argument, NULL, OPTION_ALLOW_INCOMPATIBLE_ATTR},
  {"full-shutdown", no_argument, NULL, OPTION_FULL_SHUTDOWN},
'

PARSE_AND_LIST_OPTIONS='
fprintf (file, _("\
    --mexport-symbols=FILE      Exporting global symbols into linker script\n\
    --m[no-]relax-cross-section-call Disable/enable cross-section relaxations\n\
    --mno-workaround            Disable all workarounds\n\
"));

#if defined RISCV_EXECIT_EXT
  fprintf (file, _("\
    --m[no-]execit              Disable/enable link-time EXECIT relaxation\n\
    --mexport-execit=FILE       Export .exec.itable after linking\n\
    --mimport-execit=FILE       Import .exec.itable for EXECIT relaxation\n\
    --mkeep-import-execit       Keep imported .exec.itable\n\
    --mupdate-execit            Update existing .exec.itable\n\
    --mexecit-limit=NUM         Set maximum number of entries in .exec.itable for this times\n\
    --mexecit-loop-aware        Avoid generate EXEC.IT instruction inside loop\n\
    --m[no-]execit-fls          Enable/Disable EXEC.IT for floating load/store instructions\n\
    --m[no-]execit-rvv          Enable/Disable EXEC.IT of RVV instructions\n\
    --m[no-]execit-xdsp         Enable/Disable EXEC.IT of XDSP instructions\n\
    --m[no-]execit-auipc        Enable/Disable EXEC.IT conversion for auipc instructions\n\
    --m[no-]execit-jal          Enable/Disable EXEC.IT conversion for jal instructions\n\
    --m[no-]execit-jal-over-2mib Disable/enable EXEC.IT conversion for jal instruction over the first 2MiB page of text section\n\
"));

  char *var = getenv("ANDES_HELP");
  if (var)
    {
      fprintf (file, _("\
    --mexecit-rvp               Enable exec.it of RVP\n\
"));
    }
#endif
'

PARSE_AND_LIST_ARGS_CASES='
  case OPTION_EXPORT_SYMBOLS:
    if (!optarg)
      einfo (_("Missing file for --mexport-symbols.\n"), optarg);

    if(strcmp (optarg, "-") == 0)
      andes.sym_ld_script = stdout;
    else
      {
	andes.sym_ld_script = fopen (optarg, FOPEN_WT);
	if(andes.sym_ld_script == NULL)
	  einfo (_("%P%F: cannot open map file %s: %E.\n"), optarg);
      }
    break;

  case OPTION_GP_RELATIVE_INSN:
    andes.gp_relative_insn = 1;
    break;
  case OPTION_NO_GP_RELATIVE_INSN:
    andes.gp_relative_insn = 0;
    break;
  case OPTION_NO_RELAX_ALIGN:
    andes.set_relax_align = 0;
    break;
  case OPTION_NO_TARGET_ALIGNED:
    andes.target_aligned = 0;
    break;
  case OPTION_AVOID_BTB_MISS:
    andes.avoid_btb_miss = 1;
    break;
  case OPTION_NO_AVOID_BTB_MISS:
    andes.avoid_btb_miss = 0;
    break;
  case OPTION_NO_RELAX_LUI:
    andes.set_relax_lui = 0;
    break;
  case OPTION_NO_RELAX_PC:
    andes.set_relax_pc = 0;
    break;
  case OPTION_NO_RELAX_CALL:
    andes.set_relax_call = 0;
    break;
  case OPTION_NO_RELAX_TLS_LE:
    andes.set_relax_tls_le = 0;
    break;
  case OPTION_RELAX_CROSS_SECTION_CALL:
    andes.set_relax_cross_section_call = 1;
    break;
  case OPTION_NO_RELAX_CROSS_SECTION_CALL:
    andes.set_relax_cross_section_call = 0;
    break;
  case OPTION_NO_WORKAROUND:
    andes.set_workaround = 0;
    break;

#if defined RISCV_ANDES_INTERNAL_OPTIONS
  case OPTION_OPT_TABLE_JUMP:
    andes.set_table_jump = 1;
    break;
  case OPTION_NO_OPT_TABLE_JUMP:
    andes.set_table_jump = 0;
    break;
#endif

#if defined RISCV_EXECIT_EXT
  case OPTION_EX9_TABLE:
    if (andes.execit_limit == -1
	|| andes.execit_limit > 512)
    andes.execit_limit = 512;
    /* FALL THROUGH.  */
  case OPTION_EXECIT_TABLE:
    andes.target_optimization |= RISCV_RELAX_EXECIT_ON;
    break;
  case OPTION_NO_EXECIT_TABLE:
    andes.target_optimization &= ~RISCV_RELAX_EXECIT_ON;
    break;
  case OPTION_EXPORT_EXECIT:
    if (!optarg)
      einfo (_("Missing file for --mexport-execit=<file>.\n"));

      andes.execit_export_file = optarg;
      /* Open file in the riscv_elf_relocate_execit_table.  */
      break;
  case OPTION_IMPORT_EXECIT:
    if (!optarg)
      einfo (_("Missing file for --mimport-execit=<file>.\n"));

    andes.execit_import_file = fopen (optarg, "rb+");
    if(andes.execit_import_file == NULL)
      einfo (_("ERROR %P%F: cannot open execit import file %s.\n"), optarg);
    break;
  case OPTION_KEEP_IMPORT_EXECIT:
    andes.keep_import_execit = 1;
    break;
  case OPTION_UPDATE_EXECIT:
    andes.update_execit_table = 1;
    break;
  case OPTION_EXECIT_LIMIT:
    if (optarg)
      {
	if (andes.execit_limit != -1
	    && atoi (optarg) > andes.execit_limit)
	  einfo (_("Warning: the value of execit_limit (%d) is larger "
		   "than the current setting (%d)\n"),
		 atoi (optarg), andes.execit_limit);
	else
	  andes.execit_limit = atoi (optarg);

	if (andes.execit_limit > 1024 || andes.execit_limit < 0)
	  {
	    einfo (_("ERROR: the range of execit_limit must between "
		     "0 and 1024 (default 1024)\n"));
	    exit (1);
	  }
      }
    break;
  case OPTION_EXECIT_LOOP:
    andes.execit_loop_aware = 1;
    break;
  case OPTION_EXECIT_JAL:
    andes.execit_flags.noji = 0;
    break;
  case OPTION_NO_EXECIT_JAL:
  case OPTION_EXECIT_NO_JI:
    andes.execit_flags.noji = 1;
    break;
  case OPTION_EXECIT_NO_LS:
    andes.execit_flags.nols = 1;
    break;
  case OPTION_EXECIT_NO_REL:
    andes.execit_flags.noji = 1;
    andes.execit_flags.nols = 1;
    break;
  case OPTION_EXECIT_AUIPC:
    andes.execit_flags.no_auipc = 0;
    break;
  case OPTION_NO_EXECIT_AUIPC:
    andes.execit_flags.no_auipc = 1;
    break;
  case OPTION_EXECIT_JAL_OVER_2M:
    andes.execit_jal_over_2m = 1;
    break;
  case OPTION_NO_EXECIT_JAL_OVER_2M:
    andes.execit_jal_over_2m = 0;
    break;
  case OPTION_EXECIT_RVV:
    andes.execit_flags.rvv = 1;
    break;
  case OPTION_EXECIT_RVP:
    andes.execit_flags.rvp = 1;
    break;
  case OPTION_EXECIT_FLS:
    andes.execit_flags.fls = 1;
    break;
  case OPTION_EXECIT_XDSP:
    andes.execit_flags.xdsp = 1;
    break;
  case OPTION_NO_EXECIT_RVV:
    andes.execit_flags.rvv = 0;
    break;
  case OPTION_NO_EXECIT_RVP:
    andes.execit_flags.rvp = 0;
    break;
  case OPTION_NO_EXECIT_FLS:
    andes.execit_flags.fls = 0;
    break;
  case OPTION_NO_EXECIT_XDSP:
    andes.execit_flags.xdsp = 0;
    break;
  case OPTION_NEXECIT_OP:
    andes.execit_flags.nexecit_op = 1;
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
  case OPTION_NO_EXECIT_LUI:
  case OPTION_ALLOW_INCOMPATIBLE_ATTR:
  case OPTION_FULL_SHUTDOWN:
    /* Do nothing.  */
    break;
'

LDEMUL_BEFORE_ALLOCATION=riscv_elf_before_allocation
LDEMUL_AFTER_ALLOCATION=gld${EMULATION_NAME}_after_allocation
LDEMUL_CREATE_OUTPUT_SECTION_STATEMENTS=riscv_create_output_section_statements
# /* { Andes */
LDEMUL_AFTER_OPEN=riscv_elf_after_open
LDEMUL_AFTER_CHECK_RELOCS=riscv_elf_after_check_relocs
# /* } Andes */
