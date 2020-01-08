/* Commands for communication with ANDES remote target.

   Copyright (C) 2006-2013 Free Software Foundation, Inc.
   Contributed by Andes Technology Corporation.

   This file is part of GDB.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.  */

#include "defs.h"
#include <string.h>
#include <sys/stat.h>
#include <sys/utsname.h>
#include <unistd.h>
#include "gdbcore.h"
#include "gdbcmd.h"
#include "gdbtypes.h"
#include "cli/cli-decode.h"
#include "remote.h"
#include "regcache.h"
#include "user-regs.h"
#include "inferior.h"		/* get_inferior_args () */
#include "top.h"		/* set_prompt () */
#include "ui-out.h"		/* current_uiout */
#include "exceptions.h"
#include <ctype.h>
#include <dlfcn.h>

#include "elf-bfd.h"		/* elf_elfheader () */
#include "observer.h"		/* observer_attach_inferior_created () */
#include "objfiles.h"
#include "dis-asm.h"

void nds_init_remote_cmds (void);

enum nds_remote_type
{
  nds_rt_unknown = 0,
  nds_rt_sid,
  nds_rt_ocd,
};

static struct
{
  enum nds_remote_type type;
  char cpuid[16];
  enum bfd_endian endian;
} nds_remote_info;

/* UI buffer for output redirection.  */

struct ui_file_buffer
{
  unsigned char *buf;
  long buf_size;
};

/* ui_file_put_method_ftype.

   This is used with mem_file_put to get the content of
   the internal stream buffer.  */

static void
do_ui_file_put_memcpy (void *object, const char *buffer, long length_buffer)
{
  struct ui_file_buffer *ui_buf;

  ui_buf = (struct ui_file_buffer *) object;
  if (ui_buf->buf_size < length_buffer)
    {
      /* Double the buffer when running out of space.
	 If it is too large, expand 1KB a time.  */
      if (length_buffer < 256 * 1024)
	ui_buf->buf_size = length_buffer += 1024;
      else
	ui_buf->buf_size = length_buffer * 2;
      ui_buf->buf = (unsigned char*) xrealloc (ui_buf->buf, ui_buf->buf_size);
    }

  memcpy (ui_buf->buf, buffer, length_buffer);
}


/* Callback for "nds query" command.  */

static void
nds_query_command (const char *args, int from_tty)
{
  error (_("Usage: nds query (profiling|perf-meter) [cpu] [human|ide]"));
}

/* Callback for "nds reset" command.  */

static void
nds_reset_command (const char *args, int from_tty)
{
  error (_("Usage: nds reset (profiling|perf-meter) [cpu]"));
}

/* Callback for "nds pipeline" command.  */

static void
nds_pipeline_command (const char *args, int from_tty)
{
  error (_("Usage: nds pipeline (on|off) [cpu]"));
}


/* Pretty-print for profiling data.  */

static void
nds_print_human_table (int col, int row, const char *scsv)
{
  int i;
  struct cleanup *table_cleanup = NULL;
  struct cleanup *row_cleanup = NULL;
  char *buf = NULL;
  char **col_fldname;
  char **col_hdrtext;
  int *col_width;
  enum ui_align *col_align;
  struct bound_minimal_symbol msymbol;
  CORE_ADDR addr;
  char symbol_text[256];

  buf = xstrdup (scsv);
  make_cleanup (xfree, buf);

  /* Allocate header structures.  */
  col_fldname = (char **) xmalloc (sizeof (col_fldname[0]) * col);
  col_hdrtext = (char **) xmalloc (sizeof (col_hdrtext[0]) * col);
  col_width = (int *) xmalloc (sizeof (col_width[0]) * col);
  col_align = (enum ui_align *) xmalloc (sizeof (col_align[0]) * col);

  make_cleanup (xfree, col_fldname);
  make_cleanup (xfree, col_hdrtext);
  make_cleanup (xfree, col_width);
  make_cleanup (xfree, col_align);

  /* Parsing column header.  */
  i = 0;
  while (*buf != '\0' && i < col)
    {
      CORE_ADDR addr = 0;
      char *sc = strchr (buf, ';');

      *sc = '\0';
      col_fldname[i] = buf;
      col_hdrtext[i] = col_fldname[i];
      if (col_fldname[i][0] == '%')
	col_width[i] = 6;
      else
	col_width[i] = strlen (col_hdrtext[i]) + 1;

      col_align[i] = ui_right;

      i++;
      buf = sc + 1;
    }

  gdb_assert (col == i);

  /* Output table.  */
  table_cleanup = make_cleanup_ui_out_table_begin_end
    (current_uiout, col, row - 1, "ProfilingTable");
  for (i = 0; i < col; i++)
    current_uiout->table_header (col_width[i], col_align[i],
				 col_fldname[i], col_hdrtext[i]);

  current_uiout->table_body ();

  /* Parse buf into col/row.  */
  i = 0;
  row_cleanup = make_cleanup_ui_out_tuple_begin_end (current_uiout, "row");
  while (*buf != '\0')
    {
      char *sc = strchr (buf, ';');
      int offset;

      *sc = '\0';
      switch (i)
	{
	case 0:
	  current_uiout->field_string (col_fldname[i], buf);

	  /* Assume first column is address.  */
	  strcpy (symbol_text, "\n");
	  addr = strtol (buf, NULL, 16);
	  msymbol = lookup_minimal_symbol_by_pc (addr);
	  if (!msymbol.minsym)
	    break;

	  offset = addr - BMSYMBOL_VALUE_ADDRESS (msymbol);
	  if (offset)
	    xsnprintf (symbol_text, sizeof (symbol_text), "%s + 0x%x\n",
		       MSYMBOL_PRINT_NAME (msymbol.minsym), offset);
	  else
	    xsnprintf (symbol_text, sizeof (symbol_text), "%s\n",
		       MSYMBOL_PRINT_NAME (msymbol.minsym));
	  break;
	case 1: case 2: case 3: case 4: case 5: case 6:
	  current_uiout->field_string (col_fldname[i], buf);
	  break;
	}

      i++;
      buf = sc + 1;
      if (i == col)
	{
	  current_uiout->text (symbol_text);
	  do_cleanups (row_cleanup);
	  i = 0;
	  row_cleanup = make_cleanup_ui_out_tuple_begin_end
	    (current_uiout, "row");
	}
    }

  do_cleanups (table_cleanup);
}

/* Callback for "nds query profiling" command.  */

static void
nds_query_profiling_command (const char *args, int from_tty)
{
  /* For profiling, there will be multiple responses.  */
  char cmd[256];
  int row, col;
  struct ui_file *res;
  int i;
  long int pkt_size;
  char *pkt_buf = NULL;
  struct ui_file_buffer ui_buf;
  char *arg_cpu = "cpu";
  int arg_human = 1;
  struct cleanup *back_to = NULL;
  char *p;

  /* Initial size. It may be resized by getpkt.  */
  pkt_size = 1024;

  res = mem_fileopen ();
  back_to = make_cleanup_ui_file_delete (res);

  ui_buf.buf_size = 2048;
  ui_buf.buf = (unsigned char *) xmalloc (ui_buf.buf_size);
  pkt_buf = (char *) xmalloc (pkt_size);

  make_cleanup (free_current_contents, &ui_buf.buf);
  make_cleanup (free_current_contents, &pkt_buf);
  scoped_restore save_stdtarg = make_scoped_restore (&gdb_stdtarg, res);

  gdb_argv argv (args);

  /* operator!= is overloading, so it can be used to check if args is NULL.  */
  if (argv != NULL)
    {
      if (argv[0] != NULL && *argv[0] != '\0')
	arg_cpu = argv[0];

      if (argv[1] != NULL && strcmp (argv[1], "ide") == 0)
	arg_human = FALSE;
    }

  xsnprintf (cmd, sizeof (cmd), "set %s profiling ide-query", arg_cpu);
  target_rcmd (cmd, res);
  memset (ui_buf.buf, 0, ui_buf.buf_size);
  ui_file_put (res, do_ui_file_put_memcpy, &ui_buf);

  if (arg_human == 0)
    {
      fprintf_unfiltered (gdb_stdtarg,
			  "=profiling,reason=\"fast_l1_profiling\",data=\"%s\"\n",
			  ui_buf.buf);
      goto bye;
    }

  /* The first response is Row=%d;Column=%d;
     and then comes 'Row' rows, including head row */
  i = sscanf ((char *) ui_buf.buf, "Row=%d;Column=%d;", &row, &col);
  if (i != 2)
    error (_("Failed to query profiling data"));

  p = (char *) ui_buf.buf;

  /* Skip "Row=r;Column=c;".  */
  for (i = 0; i < 2 && p; i++)
    p = strchr (p + 1, ';');
  p++;

  /* Print human-mode table here.  */
  nds_print_human_table (col, row, p);

bye:
  do_cleanups (back_to);
}

/* Callback for "nds query perfmeter" command.  */

static void
nds_query_perfmeter_command (const char *args, int from_tty)
{
  /* For perfmeter, there will be only one response.  */
  char cmd[256];

  xsnprintf (cmd, sizeof (cmd), "set %s perf-meter query",
	     args == NULL ? "cpu" : args);
  target_rcmd (cmd, gdb_stdtarg);
}

/* Callback for "nds reset profiling" command.  */

static void
nds_reset_profiling_command (const char *args, int from_tty)
{
  char cmd[256];

  xsnprintf (cmd, sizeof (cmd), "set %s profiling reset",
	     args == NULL ? "cpu" : args);
  target_rcmd (cmd, gdb_stdtarg);
}

/* Callback for "nds reset perfmeter" command.  */

static void
nds_reset_perfmeter_command (const char *args, int from_tty)
{
  char cmd[256];

  xsnprintf (cmd, sizeof (cmd), "set %s perf-meter reset",
	     args == NULL ? "cpu" : args);
  target_rcmd (cmd, gdb_stdtarg);
}

/* Callback for "nds pipeline on" command.  */

static void
nds_pipeline_on_command (const char *args, int from_tty)
{
  char cmd[256];

  xsnprintf (cmd, sizeof (cmd), "set %s pipeline-on 1",
	     args == NULL ? "cpu" : args);
  target_rcmd (cmd, gdb_stdtarg);
}

/* Callback for "nds pipeline off" command.  */

static void
nds_pipeline_off_command (const char *args, int from_tty)
{
  char cmd[256];

  xsnprintf (cmd, sizeof (cmd), "set %s pipeline-on 0",
	     args == NULL ? "cpu" : args);
  target_rcmd (cmd, gdb_stdtarg);
}


static void
nds_remote_info_init (void)
{
  nds_remote_info.type = nds_rt_unknown;
  nds_remote_info.endian = BFD_ENDIAN_UNKNOWN;
  nds_remote_info.cpuid[0] = '\0';
}

/* Query target information.  */

static struct value *
nds_target_type_make_value (struct gdbarch *gdbarch, struct internalvar *var,
			    void *ignore)
{
  int val = 0;

  if (strcmp (target_shortname, "remote") == 0
      || strcmp (target_shortname, "extended-remote") == 0)
    val = target_has_registers ? nds_remote_info.type
			       : nds_rt_unknown;

  return value_from_longest (builtin_type (gdbarch)->builtin_int,
			     val);
}

static int
nds_issue_qrcmd (const char *cmd, string_file &str)
{
  std::string whitespaces (" \t\f\v\n\r");

  /* make_cleanup outside TRY_CACHE,
     because it save and reset cleanup-chain.  */
  scoped_restore save_stdtarg = make_scoped_restore (&gdb_stdtarg, &str);
  /* Supress error messages from gdbserver
     if gdbserver doesn't support the monitor command.  */

  str.clear ();
  TRY
    {
      target_rcmd (cmd, &str);
    }
  CATCH (except, RETURN_MASK_ERROR)
    {
      return -1;
    }
  END_CATCH

  /* Trim trailing newline characters.  */
  std::size_t found = str.string ().find_last_not_of (whitespaces);
  if (found != std::string::npos)
    str.string ().erase (found + 1);
  else
    // all whitespace
    str.string ().clear ();

  return 0;
}

static int
nds_query_target_using_qrcmd (void)
{
  string_file str;
  const char *buf;
  const char *sstr = NULL;

  if (nds_issue_qrcmd ("nds query target", str) == -1)
    return -1;

  buf = str.c_str ();
  if (strcmp (buf, "SID") == 0)
    nds_remote_info.type = nds_rt_sid;
  else if (strcmp (buf, "OCD") == 0)
    nds_remote_info.type = nds_rt_ocd;
  else
    {
      printf_unfiltered (_("Unknown remote target %s\n"), buf);
      return -1;
    }

  if (nds_issue_qrcmd ("nds query endian", str) == -1)
    return -1;

  buf = str.c_str ();
  /* to match target_name: LE or target_name: BE.  */
  sstr = strstr (buf, ":");
  if (sstr == NULL)
    nds_remote_info.endian = BFD_ENDIAN_LITTLE;
  else
    {
      sstr += 2;
      if (strcmp (sstr , "LE") == 0)
	nds_remote_info.endian = BFD_ENDIAN_LITTLE;
      else if (strcmp (sstr, "BE") == 0)
	nds_remote_info.endian = BFD_ENDIAN_BIG;
    }

  if (nds_issue_qrcmd ("nds query cpuid", str) == -1)
    return -1;

  buf = str.c_str ();
  strncpy (nds_remote_info.cpuid, buf, sizeof (nds_remote_info.cpuid) - 1);

  return 0;
}

static void
nds_query_target_command (const char *args, int from_tty)
{
  char buf[64];

  nds_remote_info_init ();

  if (strcmp (target_shortname, "remote") != 0
      && strcmp (target_shortname, "extended-remote") != 0)
    return;

  /* Try to find out the type of target - SID or OCD.  */
  nds_query_target_using_qrcmd ();

  /* Prepend anything target return to prompt.  */
  xsnprintf (buf, sizeof (buf), "%s(gdb) ", nds_remote_info.cpuid);
  set_prompt (buf);
}

static void
nds_endian_check_command (const char *args, int from_tty)
{
  enum bfd_endian elf_endian = BFD_ENDIAN_UNKNOWN;

  /* ELF file is necessary for endian comparison.  */
  if (exec_bfd == NULL)
    return;

  /* The comparison is only for remote debugging.  */
  if (strcmp (target_shortname, "remote") != 0
      && strcmp (target_shortname, "extended-remote") != 0)
    return;

  if (nds_remote_info.type == nds_rt_unknown)
    return;

  elf_endian = exec_bfd->xvec->byteorder;

  if (nds_remote_info.endian != elf_endian)
    warning ("Target and elf have different endian");
}

/* This is only used for SID.  Set command-line string.  */

static void
nds_set_gloss_command (const char *args, int from_tty)
{
  int i;
  struct ui_file *out;
  const char *arg0;
  const char *inferior_args;
  const char *f;
  char cmdline[0x1000];		/* 4K for max command line.  */
  struct cleanup *back_to;
  asection *s = NULL;
  const char *sectnames[] = { ".text", "code", ".bss", "bss" };

  /* set gloss for SID only. */
  if (nds_remote_info.type != nds_rt_sid)
    return;

  back_to = make_cleanup (null_cleanup, 0);
  if (exec_bfd == NULL)
    error (_("Cannot set gloss without executable.\n"
	     "Use the \"file\" or \"exec-file\" command."));

  /* ui_file for target_rcmd.  */
  out = stdio_fileopen (stdout);
  make_cleanup_ui_file_delete (out);

  /* start_code, end_code, start_bss, end_bss,
     brk, command-line.  */
  for (s = exec_bfd->sections; s; s = s->next)
    {
      bfd_vma start, size;
      const char *attr;

      for (i = 0; i < ARRAY_SIZE (sectnames); i += 2)
	if (strcmp (bfd_get_section_name (exec_bfd, s), sectnames[i]) == 0)
	  break;

      if (i >= ARRAY_SIZE (sectnames))
	continue;

      start = bfd_get_section_vma (exec_bfd, s);
      size = bfd_section_size (exec_bfd, s);

      /* Set gloss (start|end)_XXX.  */
      xsnprintf (cmdline, sizeof (cmdline), "set gloss start_%s %u",
		 sectnames[i + 1], (unsigned int) start);
      target_rcmd (cmdline, out);
      xsnprintf (cmdline, sizeof (cmdline), "set gloss end_%s %u",
		 sectnames[i + 1], (unsigned int) (start + size));
      target_rcmd (cmdline, out);
    }

  /* Set gloss command-line for "set args".  */
  arg0 = bfd_get_filename(exec_bfd);
  inferior_args = get_inferior_args ();

  f = strrchr (arg0, '/');
  if (f == NULL)
    f = strrchr (arg0, '\\');

  if (f == NULL)
    f = "a.out";
  else
    f++; /* skip separator.  */

  xsnprintf (cmdline, sizeof (cmdline),
	     "set gloss command-line \"%s %s\"", f, inferior_args);
  target_rcmd (cmdline, out);

  do_cleanups (back_to);
}


/* Data structures used by ACE */
typedef struct ace_operand
{
  const char *name;  /* operand name */
  int bitpos;  /* operand start position */
  int bitsize;  /* operand width */
  int shift;  /* operand shift amount */
  int hw_res;  /* hardware resource */
  const char *hw_name;  /* hardware/register name */
} ace_op_t;

extern struct riscv_opcode *ace_opcs;
extern ace_op_t *ace_ops;
/* Represent whether ACE shared library is loaded successfully */
extern bfd_boolean ace_lib_load_success;

static void
nds_handle_ace(const char *ace_lib_path)
{
  chmod(ace_lib_path, S_IRWXU);
  void *dlc = dlopen (ace_lib_path, RTLD_NOW | RTLD_LOCAL);
  char *err;

  if (dlc == NULL)
    err = (char *) dlerror ();
  else
    {
      ace_ops = (ace_op_t *) dlsym (dlc, "ace_operands");
      err = (char *) dlerror ();
      if (err == NULL)
	{
	  ace_opcs = (struct riscv_opcode *) dlsym (dlc, "ace_opcodes");
	  err = (char *) dlerror ();
	}
    }

  if (err == NULL)
    ace_lib_load_success = TRUE;
  else
    fprintf (stderr, _("Fault to load ACE shared library: %s\n"), err);
}

/* Callback for "nds read_acedesc" command.  */

static void
nds_read_ace_desc_command (const char *args, int from_tty)
{
  struct utsname os;

  /* What kind of platform am I running at?  */
  if (uname (&os) == 0)
    {
      string_file str;
      char qrcmd[80];
      const char *filename = NULL;

      sprintf (qrcmd, "nds ace %s", os.sysname);
      if (nds_issue_qrcmd (qrcmd, str) == -1)
	return;

      filename = str.c_str ();
      if (strlen (filename) != 0)
	{
	  const char *lib = "./libace.so";

	  remote_file_get (filename, lib, from_tty);
	  nds_handle_ace (lib);
	  unlink (lib);
	}
    }
}


static int
nds_get_acr_info (struct gdbarch *gdbarch, const char *name,
		  int *regnum, int *len)
{
  int regno;

  if (name[0] == '$')
    name++;
  regno = user_reg_map_name_to_regnum (gdbarch, name, strlen (name));
  if (regno == -1)
    return -1;

  /* Get the size of register in byte.  */
  *len = register_size (gdbarch, regno);
  *regnum = regno;

  return 0;
}

#define MAX_ACR_BIT		1024
#define MAX_ACR_HEX_DIGIT	(MAX_ACR_BIT/4)

static char tohex[] = "0123456789abcdef";

/* Callback for "nds print" command, which is used to construct a
   hex string from the content of ACR.  */

static void
nds_print_acr_command (const char *args, int from_tty)
{
  struct regcache *regcache = get_current_regcache ();
  struct gdbarch *gdbarch = regcache->arch ();
  enum bfd_endian byte_order = gdbarch_byte_order (gdbarch);
  int regnum;
  char *name = NULL;
  int len, i;
  /* Flag used to trim leading-zero.  */
  int flag_lz;
  /* +1 for null terminating char.  */
  char val_str[MAX_ACR_HEX_DIGIT + 1];
  char *str_p;
  gdb_byte *acr_content;

  /* Parse arguments.  */
  gdb_argv argv (args);

  /* operator== is overloading, so it can be used to check if args is NULL.  */
  if (argv == NULL || argv[0] == NULL)
    {
      fprintf_unfiltered (gdb_stdout, "<usage>: nds print <acr_name>\n");
      return;
    }

  name = argv[0];
  if (nds_get_acr_info (gdbarch, name, &regnum, &len) == -1)
    return;

  /* Allocate space for ACR.  */
  acr_content = (gdb_byte *) xcalloc (1, len);

  get_frame_register (get_selected_frame (NULL), regnum, acr_content);

  /* Construct val_str from ACR byte buffer, and start from the MSB of
     val_str, so that the leading zero case can be handled more easily.  */
  flag_lz = 1;
  str_p = val_str;
  if (byte_order == BFD_ENDIAN_BIG)
    {
      for (i = 0; i < len; i++)
	{
	  gdb_byte b;

	  b = acr_content[i];
	  if (flag_lz == 1 && b == 0)
	    continue;
	  else
	    flag_lz = 0;

	  str_p[0] = tohex[(b >> 4) & 0xf];
	  str_p[1] = tohex[(b & 0xf)];
	  str_p += 2;
	}
    }
  else
    {
      for (i = len - 1; i >= 0; i--)
	{
	  gdb_byte b;

	  b = acr_content[i];
	  if (flag_lz == 1 && b == 0)
	    continue;
	  else
	    flag_lz = 0;

	  str_p[0] = tohex[(b >> 4) & 0xf];
	  str_p[1] = tohex[(b & 0xf)];
	  str_p += 2;
	}
    }
  str_p[0] = '\0';

  /* Handle null string specially.  */
  str_p = val_str;
  while (*str_p == '0')
    str_p++;
  if (*str_p == '\0')
    {
      str_p[0] = '0';
      str_p[1] = '\0';
    }

  fprintf_filtered (gdb_stdout, "The value of %s is 0x%s\n", name, str_p);

  xfree (acr_content);
}

/* Convert hex digit A to a number.  */

static int
fromhex (int a)
{
  if (a >= '0' && a <= '9')
    return a - '0';
  else if (a >= 'a' && a <= 'f')
    return a - 'a' + 10;
  else if (a >= 'A' && a <= 'F')
    return a - 'A' + 10;
  else
    error (_("Given value contains invalid hex digit %d"), a);
}

/* Callback for "nds set" command, which is used to construct
   the content of ACR from the given hex string.  */

static void
nds_set_acr_command (const char *args, int from_tty)
{
  struct regcache *regcache = get_current_regcache ();
  struct gdbarch *gdbarch = regcache->arch ();
  enum bfd_endian byte_order = gdbarch_byte_order (gdbarch);
  int regnum;
  char *name = NULL;
  int len, i;
  const char *val_str, *str_p;
  gdb_byte *acr_content = NULL;

  /* Parse arguments.  */
  gdb_argv argv (args);

  /* operator== is overloading, so it can be used to check if args is NULL.  */
  if (argv == NULL || argv[0] == NULL || argv[1] == NULL)
    {
      fprintf_unfiltered (gdb_stdout,
			  "<usage>: nds set <acr_name> <hex_str>\n");
      return;
    }

  name = argv[0];
  if (nds_get_acr_info (gdbarch, name, &regnum, &len) == -1)
    return;

  val_str = argv[1];
  if (val_str[0] == '0' && (val_str[1] == 'x' || val_str[1] == 'X'))
    val_str += 2;

  acr_content = (gdb_byte *) xcalloc (1, len);

  /* Construct ACR byte buffer from val_str, and start from the LSB of
     val_str, so that the leading zero case can be handled more easily.  */
  str_p = val_str + strlen (val_str);
  if (byte_order == BFD_ENDIAN_BIG)
    {
      for (i = len - 1; i >= 0; i--)
	{
	  str_p -= 2;
	  if (str_p >= val_str)
	    acr_content[i] = (fromhex (str_p[0]) << 4) + fromhex (str_p[1]);
	  else if (str_p == val_str - 1)
	    acr_content[i] = fromhex (str_p[1]);
	  else
	    break;
	}
    }
  else
    {
      for (i = 0; i < len; i++)
	{
	  str_p -= 2;
	  if (str_p >= val_str)
	    acr_content[i] = (fromhex (str_p[0]) << 4) + fromhex (str_p[1]);
	  else if (str_p == val_str - 1)
	    acr_content[i] = fromhex (str_p[1]);
	  else
	    break;
	}
    }

  put_frame_register (get_selected_frame (NULL), regnum, acr_content);

  /* The accurate bitsize info is necessary to do the truncation in GDB.
     Currently, the truncation is actually done at target side, so the
     regcache invalidation is necessary.  */
  regcache->invalidate (regnum);

  xfree (acr_content);
}

/* Bug 6654 - Multiple watchpoints was be hit, GDB only shows one of them.

   FIXME: This is a dirty hacking for hooking remote->to_stopped_data_address,
   in order to handling multiple hit. This is not a bug at all.  */

/* remote_stopped_data_address_p is used to record the to_stopped_data_address
   callback of remote_ops and extended_remote_ops, and that should be
   remote_stopped_data_address.

   One variable is enough, because remote_ops and extended_remote_ops have
   the same to_stopped_data_address callback.  */
static int (*remote_stopped_data_address_p) (struct target_ops*, CORE_ADDR*);

/* watchpoints check flow:
   watchpoints_triggered -> target_stopped_data_address
   -> delegate_stopped_data_address -> nds_remote_stopped_data_address.  */

static int
nds_remote_stopped_data_address (struct target_ops *target, CORE_ADDR *addr_p)
{
  /* When target is stopped by watchpoint, remote_stopped_data_address will
     return 1 and *add_p will be updated to be watch data address.  */
  if (!remote_stopped_data_address_p (target, addr_p))
    return 0;

  /* If the addr is 0x0, we assume SMW multiple hits.
     Pretent data_address is unknown and let GDB figure it out.  */
  return (*addr_p) != 0;
}

static void
nds_remote_inferior_created_observer (struct target_ops *target, int from_tty)
{
  /* Hack the member to_stopped_data_address in remote_ops/extend_remote_ops
     without modifying the generic code.

     @TARGET is always &current_target, use beneath to get the value
     of target_stack.  (See update_current_target @ target.c.)  */
  struct target_ops *t = target->beneath;

  /* remote_ops and extended_remote_ops have process_stratum, which is higher
     than file_stratum, so they can be found via target_stack first.  */
  if ((strcmp (t->to_shortname, "remote") == 0
       || strcmp (t->to_shortname, "extended-remote") == 0)
      && t->to_stopped_data_address != nds_remote_stopped_data_address)
    {
      remote_stopped_data_address_p = t->to_stopped_data_address;
      t->to_stopped_data_address = nds_remote_stopped_data_address;

      /* current_target is updated before inferior_created event, so the member
	 to_stopped_data_address may be the original one at the first time.
	 This issue can be avoided by always replacing this member.  */
      if ((strcmp (target->to_shortname, "remote") == 0
	   || strcmp (target->to_shortname, "extended-remote") == 0))
	{
	  target->to_stopped_data_address = nds_remote_stopped_data_address;
	}
    }
}

/* nds_insertion_sort sorts an array with nmemb elements of size size.
   This prototype is the same as qsort ().  */

static void
nds_insertion_sort (void *base, size_t nmemb, size_t size,
		    int (*compar) (const void *lhs, const void *rhs))
{
  char *ptr = (char *) base;
  int i, j;
  char *tmp = (char *) xmalloc (size);

  /* If i is less than j, i is inserted before j.

     |---- j ----- i --------------|
      \		 / \		  /
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
  free (tmp);
}

void
qsort (void *base, size_t nmemb, size_t size,
       int (*compar) (const void *lhs, const void *rhs))
{
  nds_insertion_sort (base, nmemb, size, compar);
}

static struct cmd_list_element *nds_pipeline_cmdlist;
static struct cmd_list_element *nds_query_cmdlist;
static struct cmd_list_element *nds_reset_cmdlist;

static const struct internalvar_funcs nds_target_type_funcs =
{
  nds_target_type_make_value,
  NULL,
  NULL
};

extern struct cmd_list_element *nds_cmdlist;

void
nds_init_remote_cmds (void)
{
  /* Hook for query remote target information.  */
  observer_attach_inferior_created (nds_remote_inferior_created_observer);

  nds_remote_info_init ();

  add_cmd ("endian-check", class_files, nds_endian_check_command,
	   _("Check endian consistency between elf and target. "
	     "Throwing warning if failed."),
	   &nds_cmdlist);

  /* nds set-gloss */
  add_cmd ("set-gloss", class_files, nds_set_gloss_command,
	   _("Set gloss related environment."), &nds_cmdlist);

  /* nds query (profiling|perf-meter|target)  */
  add_prefix_cmd ("query", no_class, nds_query_command,
		  _("Query remote data."), &nds_query_cmdlist, "query ",
		  0, &nds_cmdlist);
  add_cmd ("profiling", no_class, nds_query_profiling_command,
	   _("Query profiling results."), &nds_query_cmdlist);
  add_cmd ("perf-meter", no_class, nds_query_perfmeter_command,
	   _("Query perf-meter results."), &nds_query_cmdlist);
  add_cmd ("target", no_class, nds_query_target_command,
	   _("Query target information."), &nds_query_cmdlist);

  /* nds reset (profiling|perf-meter)  */
  add_prefix_cmd ("reset", no_class, nds_reset_command,
		  _("Reset profiling."), &nds_reset_cmdlist, "reset ",
		  0, &nds_cmdlist);
  add_cmd ("profiling", no_class, nds_reset_profiling_command,
	   _("Query profiling results."), &nds_reset_cmdlist);
  add_cmd ("perf-meter", no_class, nds_reset_perfmeter_command,
	   _("Query perf-meter results."), &nds_reset_cmdlist);

  /* nds pipeline (on|off) */
  add_prefix_cmd ("pipeline", no_class, nds_pipeline_command,
		  _("nds-sid profiling commands."),
		  &nds_pipeline_cmdlist, "pipeline ", 0, &nds_cmdlist);
  add_cmd ("on", no_class, nds_pipeline_on_command,
	   _("Turn on pipeline for profiling."), &nds_pipeline_cmdlist);
  add_cmd ("off", no_class, nds_pipeline_off_command,
	   _("Turn off pipeline for profiling."), &nds_pipeline_cmdlist);

  /* nds read_acedesc  */
  add_cmd ("read_acedesc", no_class, nds_read_ace_desc_command,
	   _("Request the ACE or coprocessor description file from remote."),
	     &nds_cmdlist);

  /* nds print  */
  add_cmd ("print", no_class, nds_print_acr_command,
	   _("Print the value of ACR in hex format."), &nds_cmdlist);

  /* nds set  */
  add_cmd ("set", no_class, nds_set_acr_command,
	   _("Set the value of ACR in hex format."), &nds_cmdlist);

  create_internalvar_type_lazy ("_nds_target_type", &nds_target_type_funcs,
				NULL);
}
