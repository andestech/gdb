/* Commands for communication with NDS32 remote target.

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
#ifndef __MINGW32__
#include <sys/utsname.h>
#endif
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

#include <elf/external.h>	/* Elf32_External_Ehdr */
#include <elf/internal.h>	/* Elf_Internal_Ehdr */
#include "elf-bfd.h"		/* elf_elfheader () */
#include "observer.h"		/* observer_attach_inferior_created () */
#include "objfiles.h"

#include "dis-asm.h"
#include "nds32-elf.h"
#include "opcodes/nds32-asm.h"
#include "bfd/elf32-nds32.h"
void nds32_init_remote_cmds (void);
static void nds32_read_ace_desc_command (char *args, int from_tty);


char *nds32_qparts [] =
{
  "qPart:nds32:ask:de",
  "qPart:nds32:ask:mach",
  "qPart:nds32:ask:base16",
  "qPart:nds32:ask:pex1",
  "qPart:nds32:ask:pex2",
  "qPart:nds32:ask:div",
  "qPart:nds32:ask:abi",
  "qPart:nds32:ask:mfusr_pc",
  "qPart:nds32:ask:fpu",
  "qPart:nds32:ask:audio",
  "qPart:nds32:ask:string",
  "qPart:nds32:ask:reduced_regs",
  "qPart:nds32:ask:video",
  "qPart:nds32:ask:ifc",
  "qPart:nds32:ask:elf_ver",
  "qPart:nds32:ask:l2c",
  "qPart:nds32:ask:mac",
  "qPart:nds32:ask:cpu", /* core0, cpu, etc */
  "qPart:nds32:ask:target", /* SID, ICE */

  "qPart:nds32:request:InvalidateCache",
  "qPart:nds32:request:MemAccBus",
  "qPart:nds32:request:MemAccCPU"
};

enum nds32_qparts_enum
{
  NDS32_Q_ENDIAN,
  NDS32_Q_MACH,
  NDS32_Q_BASE16,
  NDS32_Q_PEX1,
  NDS32_Q_PEX2,
  NDS32_Q_DIV,
  NDS32_Q_ABI,
  NDS32_Q_MFUSR_PC,
  NDS32_Q_FPU,
  NDS32_Q_AUDIO,
  NDS32_Q_STRING,
  NDS32_Q_REDUCED_REGS,
  NDS32_Q_VIDEO,
  NDS32_Q_IFC,
  NDS32_Q_ELF_VER,
  NDS32_Q_L2C,
  NDS32_Q_MAC,
  NDS32_Q_CPU,
  NDS32_Q_TARGET,

  NDS32_Q_INVALIDATE_CACHE,
  NDS32_Q_ACC_BUS,
  NDS32_Q_ACC_CPU,
  NDS32_Q_END
};

enum nds32_remote_type
{
  nds32_rt_unknown = 0,
  nds32_rt_sid,
  nds32_rt_ice,
  nds32_rt_ocd,
};

static struct
{
  enum nds32_remote_type type;
  char cpu[16];
  enum bfd_endian endian;
} nds32_remote_info;

extern keyword_t **nds32_keyword_table[NDS32_CORE_COUNT];

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


/* Wrapper for execute a GDB CLI command.  */

static void
nds32_execute_command (char *cmd, char *arg, int from_tty)
{
  int len;
  char *line;

  if (arg == NULL)
    arg = "";
  len = strlen (arg) + strlen (cmd) + 2;

  gdb_assert (len < 1024);

  line = (char *) alloca (len);
  memset (line, 0, len);
  if (arg != NULL)
    xsnprintf (line, len, "%s %s", cmd, arg);
  else
    xsnprintf (line, len, "%s", cmd);
  execute_command (line, from_tty);
}

static void
nds32_restore_remote_timeout (void *p)
{
  int value = *(int *) p;

  remote_timeout = value;
}

/* Reset-target.
   Set remote_timeout to 1000 sec to avoid timeout.  */

static void
nds32_reset_target_command (char *args, int from_tty)
{
  int saved_remote_timeout = remote_timeout;
  struct cleanup *back_to;

  back_to = make_cleanup (nds32_restore_remote_timeout, &saved_remote_timeout);
  remote_timeout = 1000;
  nds32_execute_command ("monitor reset target", NULL, from_tty);
  registers_changed ();
  do_cleanups (back_to);
}

/* Callback for "nds32 reset-hold" command.  */

static void
nds32_reset_hold_command (char *args, int from_tty)
{
  int saved_remote_timeout = remote_timeout;
  struct cleanup *back_to;

  back_to =
    make_cleanup (nds32_restore_remote_timeout, &saved_remote_timeout);
  remote_timeout = 1000;
  nds32_execute_command ("monitor reset hold", NULL, from_tty);
  registers_changed ();
  do_cleanups (back_to);
}

/* Callback for "nds32 pipeline on" command.  */

static void
nds32_pipeline_on_command (char *args, int from_tty)
{
  char cmd[256];

  xsnprintf (cmd, sizeof (cmd), "monitor set %s pipeline-on 1",
	     args == NULL ? "cpu" : args);
  nds32_execute_command (cmd, NULL, from_tty);
}

/* Callback for "nds32 pipeline off" command.  */

static void
nds32_pipeline_off_command (char *args, int from_tty)
{
  char cmd[256];

  xsnprintf (cmd, sizeof (cmd), "monitor set %s pipeline-on 0",
	     args == NULL ? "cpu" : args);
  nds32_execute_command (cmd, NULL, from_tty);
}

/* Callback for "nds32 pipeline" command.  */

static void
nds32_pipeline_command (char *args, int from_tty)
{
  error (_("Usage: nds32 pipeline (on|off) [cpu]"));
}

/* Callback for "nds32 query" command.  */

static void
nds32_query_command (char *args, int from_tty)
{
  error (_("Usage: nds32 query (profiling|perf-meter) [cpu] [human|ide]"));
}

/* Callback for "nds32 reset" command.  */

static void
nds32_reset_command (char *args, int from_tty)
{
  error (_("Usage: nds32 reset (profiling|perf-meter) [cpu]"));
}

/* Pretty-print for profiling data.  */

static void
nds32_print_human_table (int col, int row, const char *scsv)
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
    ui_out_table_header (current_uiout, col_width[i], col_align[i],
			 col_fldname[i], col_hdrtext[i]);

  ui_out_table_body (current_uiout);

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
	  ui_out_field_string (current_uiout, col_fldname[i], buf);

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
	  ui_out_field_string (current_uiout, col_fldname[i], buf);
	  break;
	}

      i++;
      buf = sc + 1;
      if (i == col)
	{
	  ui_out_text (current_uiout, symbol_text);
	  do_cleanups (row_cleanup);
	  i = 0;
	  row_cleanup = make_cleanup_ui_out_tuple_begin_end
	    (current_uiout, "row");
	}
    }

  do_cleanups (table_cleanup);
}

/* Callback for "nds32 query profiling" command.  */

static void
nds32_query_profiling_command (char *args, int from_tty)
{
  /* For profiling, there will be multiple responses.  */
  int row, col;
  struct ui_file *res;
  int i;
  long int pkt_size;
  char *pkt_buf = NULL;
  struct ui_file_buffer ui_buf;
  char *arg_cpu = "cpu";
  int arg_human = TRUE;
  struct cleanup *back_to = NULL;
  char **argv = NULL;
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
  make_cleanup_restore_ui_file (&gdb_stdtarg);

  gdb_stdtarg = res;

  if (args != NULL)
    {
      /* Parse arguments.  */
      argv = gdb_buildargv (args);
      make_cleanup_freeargv (argv);
    }

  for (i = 0; argv && argv[i]; i++)
    {
      switch (i)
	{
	case 0:
	  arg_cpu = argv[i];
	  break;
	case 1:
	  arg_human = strcmp (argv[i], "ide");	/* default human */
	  break;
	}
    }

  /* Fill BUF with monitor command. */
  xsnprintf ((char *) ui_buf.buf, ui_buf.buf_size,
	     "set %s profiling ide-query", args == NULL ? "cpu" : arg_cpu);
  target_rcmd ((char *) ui_buf.buf, res);
  memset (ui_buf.buf, 0, ui_buf.buf_size);
  ui_file_put (res, do_ui_file_put_memcpy, &ui_buf);

  if (!arg_human)
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
  nds32_print_human_table (col, row, p);

bye:
  do_cleanups (back_to);
}

/* Callback for "nds32 query perfmeter" command.  */

static void
nds32_query_perfmeter_command (char *args, int from_tty)
{
  /* For perfmeter, there will be only one response.  */
  char cmd[128];

  xsnprintf (cmd, sizeof (cmd), "set %s perf-meter query",
	     args == NULL ? "cpu" : args);
  target_rcmd (cmd, gdb_stdtarg);
}

/* Callback for "nds32 reset profiling" command.  */

static void
nds32_reset_profiling_command (char *args, int from_tty)
{
  char cmd[256];

  xsnprintf (cmd, sizeof (cmd), "set %s profiling reset",
	     args == NULL ? "cpu" : args);
  target_rcmd (cmd, gdb_stdtarg);
}

/* Callback for "nds32 reset perfmeter" command.  */

static void
nds32_reset_perfmeter_command (char *args, int from_tty)
{
  char cmd[256];

  xsnprintf (cmd, sizeof (cmd), "set %s perf-meter reset",
	     args == NULL ? "cpu" : args);
  target_rcmd (cmd, gdb_stdtarg);
}

static void
nds32_handle_ace(char *args, FILE *fptr, int cpid)
{
  char str[20];
  unsigned k;
  int copid;

  /* Initialize main core data if not done yet. */
  if (nds32_keyword_table[0] == NULL)
    {
      struct disassemble_info di;

      di.disassembler_options = NULL;
      disassemble_init_for_nds32 (&di);
    }

  /* ACE_ID is 4.  */
  copid = (cpid == 4) ? NDS32_ACE : NDS32_COP0 + cpid;

  str[0] = '.';
  str[1] = '/';
  for (k = 2; k < 16 && fread (&str[k], sizeof (char), 1, fptr) ==  1; k++)
    {
      if (str[k] == ',')
	{
	  str[k] = '\0';
	  break;
	}
    }
  if (k != 16)
    {
      char *buf;

      strcat (str, ".so");
      sscanf (&str[3], "%u", &k);
      if (k != 0)
	{
	  buf = (char*) malloc (k + 1);
	  if (buf != NULL && fread (buf, k, 1, fptr) == 1)
	    {
	      buf[k] = '\0';
	      if (str[2] == 'B')
		{
		  FILE *fd;

		  /* Binary share library.  */
		  fd = fopen (str, "wb");
		  if (fd != NULL)
		    {
		      size_t n = fwrite (buf, k, 1, fd);
		      fclose (fd);
		      chmod (str, S_IRWXU);
		      if (n == 1)
			{
			  switch (copid)
			    {
			    case NDS32_ACE:
			      n = nds32_parse_udi (str);
			      break;
			    case NDS32_COP0:
			      n = nds32_parse_cop0 (str);
			      break;
			    case NDS32_COP1:
			      n = nds32_parse_cop1 (str);
			      break;
			    case NDS32_COP2:
			      n = nds32_parse_cop2 (str);
			      break;
			    case NDS32_COP3:
			      n = nds32_parse_cop3 (str);
			      break;
			    default:
			      n = 0;
			      break;
			    }
			  if (n != 0)
			    {
			      /* Add op-codes.  */
			      nds32_add_opcode_hash_table (copid);
			    }
			}
		    }
		}
	      else // if (str[0] == 'X')
		{
		  /* FIXME: XML text.  */
		}

	      unlink (str);
	    }
	}
    }
  else
    {
      /* File is corrupted or what?  Nothing can be done. */
    }
}

/* Callback for "nds32 read_acedesc" command.  */

static void
nds32_read_ace_desc_command (char *args, int from_tty)
{
#ifndef __MINGW32__
  struct utsname os;

  /* What kind of platform am I running at?  */
  if (uname (&os) == 0)
    {
      struct cleanup *back_to;
      struct ui_file *res;
      struct ui_file_buffer ui_buf;
      volatile struct gdb_exception except;
      int len, cpid;

      /* ui_file for qRcmd.  */
      res = mem_fileopen ();
      back_to = make_cleanup_ui_file_delete (res);

      /* ui_file_buffer for reading ui_file.  */
      ui_buf.buf_size = 2048;
      ui_buf.buf = (unsigned char *) xmalloc (ui_buf.buf_size);
      make_cleanup (free_current_contents, &ui_buf.buf);

      for (cpid = 0; cpid < 5; cpid++)
	{
	  char qrcmd[80];

	  if (cpid == 4)
	    sprintf (qrcmd, "nds ace %s", os.sysname);
	  else
	    sprintf (qrcmd, "nds cop%d %s", cpid, os.sysname);

	  /* make_cleanup outside TRY_CACHE,
	     because it save and reset cleanup-chain.  */
	  make_cleanup_restore_ui_file (&gdb_stdtarg);
	  /* Supress error messages from gdbserver
	     if gdbserver doesn't support the monitor command.  */
	  gdb_stdtarg = res;

	  TRY
	    {
	      target_rcmd (qrcmd, res);
	    }
	  CATCH (except, RETURN_MASK_ERROR)
	    {
	      goto out;
	    }
	  END_CATCH

	  /* Read data in ui_file.  */
	  memset (ui_buf.buf, 0, ui_buf.buf_size);
	  ui_file_put (res, do_ui_file_put_memcpy, &ui_buf);
	  ui_file_rewind (res);

	  /* Trim trailing newline characters.  */
	  len = strlen ((char *) ui_buf.buf);
	  while (len != 0 && isspace (ui_buf.buf[len - 1]))
	    len--;
	  ui_buf.buf[len] = '\0';

	  if (len != 0)
	    {
	      FILE *fptr = NULL;

	      if (cpid == 4)
		strcpy (qrcmd, "target.ace");
	      else
		sprintf (qrcmd, "target.cop%d", cpid);
	      remote_file_get ((char *) ui_buf.buf, qrcmd, from_tty);
	      fptr = fopen (qrcmd, "r");
	      nds32_handle_ace (args, fptr, cpid);
	      fclose(fptr);
	      unlink (qrcmd);
	    }
	}
out:
      do_cleanups (back_to);
    }
#else
    abort ();
#endif
}


static void
nds32_remote_info_init (void)
{
  nds32_remote_info.type = nds32_rt_unknown;
  nds32_remote_info.endian = BFD_ENDIAN_UNKNOWN;
  strcpy (nds32_remote_info.cpu, "cpu");
}

/* Query target information.  */

static struct value *
nds32_target_type_make_value (struct gdbarch *gdbarch, struct internalvar *var,
			      void *ignore)
{
  int val = 0;

  if (strcmp (target_shortname, "remote") == 0
      || strcmp (target_shortname, "extended-remote") == 0)
    val = target_has_registers ? nds32_remote_info.type
			       : nds32_rt_unknown;

  return value_from_longest (builtin_type (gdbarch)->builtin_int,
			     val);
}

static int
nds32_query_target_using_qpart (void)
{
  char *buf;
  long size = 64;
  struct cleanup *back_to;
  int ret = FALSE;

  /* The buffer passed to getpkt must be allocated using xmalloc,
     because it might be xrealloc by read_frame.
     See remote.c for details.  `buf' must be freed before return.  */
  buf = (char *) xmalloc (size);

  /* Let caller clean it up.  */
  back_to = make_cleanup (free_current_contents, &buf);

  /* qPart:nds32:ask:target - SID or ICE.  */
  nds32_remote_info.type = nds32_rt_unknown;
  putpkt (nds32_qparts[NDS32_Q_TARGET]);
  getpkt (&buf, &size, 0);
  if (strcmp (buf, "SID") == 0)
    nds32_remote_info.type = nds32_rt_sid;
  else if (strcmp (buf, "ICE") == 0)
    nds32_remote_info.type = nds32_rt_ice;
  else
    goto out;

  /* qPart:nds32:ask:cpu - prompt, e.g., "core0(gdb) ".  */
  putpkt (nds32_qparts[NDS32_Q_CPU]);
  getpkt (&buf, &size, 0);
  if (strlen (buf) > 0 && buf[0] != 'E')
    {
      const int csize = sizeof (nds32_remote_info.cpu);
      memset (nds32_remote_info.cpu, 0, csize);
      strncpy (nds32_remote_info.cpu, buf, csize - 1);
    }

  /* qPart:nds32:ask:de - endian, e.g., LE or BE.  */
  putpkt (nds32_qparts[NDS32_Q_ENDIAN]);
  getpkt (&buf, &size, 0);
  if (strcmp (buf, "LE") == 0)
    nds32_remote_info.endian = BFD_ENDIAN_LITTLE;
  else if (strcmp (buf, "BE") == 0)
    nds32_remote_info.endian = BFD_ENDIAN_BIG;
  else
    nds32_remote_info.endian = BFD_ENDIAN_UNKNOWN;
  ret = TRUE;

out:
  do_cleanups (back_to);
  return ret;
}

static int
nds32_issue_qrcmd (char *cmd, struct ui_file *res,
		   struct ui_file_buffer *ui_buf)
{
  int len;
  volatile struct gdb_exception except;

  ui_file_rewind (res);

  TRY
    {
      target_rcmd (cmd, res);
    }
  CATCH (except, RETURN_MASK_ERROR)
    {
      return -1;
    }
  END_CATCH

  /* Read data in ui_file.  */
  memset (ui_buf->buf, 0, ui_buf->buf_size);
  ui_file_put (res, do_ui_file_put_memcpy, ui_buf);

  /* Trim trailing newline characters.  */
  len = strlen ((char *) ui_buf->buf);
  while (isspace (ui_buf->buf[len - 1]) && len > 0)
    len--;
  ui_buf->buf[len] = '\0';

  return 0;
}

static int
nds32_query_target_using_qrcmd (void)
{
  struct cleanup *back_to;
  struct ui_file *res;
  struct ui_file_buffer ui_buf;
  char buf[64];
  int ret = FALSE;
  volatile struct gdb_exception except;
  int len;
  const char *str = NULL;

  /* ui_file for qRcmd.  */
  res = mem_fileopen ();
  back_to = make_cleanup_ui_file_delete (res);

  /* ui_file_buffer for reading ui_file.  */
  ui_buf.buf_size = 64;
  ui_buf.buf = (unsigned char *) xmalloc (ui_buf.buf_size);
  make_cleanup (free_current_contents, &ui_buf.buf);

  /* make_cleanup outside TRY_CACHE,
     because it save and reset cleanup-chain.  */
  make_cleanup_restore_ui_file (&gdb_stdtarg);
  /* Supress error messages from gdbserver
     if gdbserver doesn't support the monitor command.  */
  gdb_stdtarg = res;

  if (nds32_issue_qrcmd ("nds query target", res, &ui_buf) == -1)
    goto out;

  if (strcmp ((char *) ui_buf.buf, "OCD") == 0)
    nds32_remote_info.type = nds32_rt_ocd;
  else
    {
      printf_unfiltered (_("Unknown remote target %s\n"),
			 ui_buf.buf);
      goto out;
    }

  if (nds32_issue_qrcmd ("nds query endian", res, &ui_buf) == -1)
    goto out;

  /* to match target_name: LE or target_name: BE.  */
  str = strstr ((char *) ui_buf.buf, ":");
  str += 2;
  if (strcmp (str , "LE") == 0)
    nds32_remote_info.endian = BFD_ENDIAN_LITTLE;
  else if (strcmp (str, "BE") == 0)
    nds32_remote_info.endian = BFD_ENDIAN_BIG;

  ret = TRUE;
out:
  do_cleanups (back_to);
  return ret;
}

static void
nds32_query_target_command (char *arg, int from_tty)
{
  nds32_remote_info_init ();

  if (strcmp (target_shortname, "remote") != 0
      && strcmp (target_shortname, "extended-remote") != 0)
    return;
  /* FIXME if we don't know, use ELF. */

  /* Try to find out the type of target - SID, ICE or OCD.  */
  if (!nds32_query_target_using_qpart ())
    nds32_query_target_using_qrcmd ();

end_query:
  /* Set cpu name if ICE and CPU!="cpu".  */
  if (strcmp ("cpu", nds32_remote_info.cpu) != 0)
    {
      char buf[64];

      xsnprintf (buf, sizeof (buf), "%s(gdb) ", nds32_remote_info.cpu);
      set_prompt (buf);
    }
  else
    {
      /* Restore to DEFAULT_PROMPT.  */
      set_prompt ("(gdb) ");
    }
}

/* Callback for elf-check.  */

static unsigned int
nds32_elf_check_get_register (unsigned int regno)
{
  ULONGEST regval;
  struct regcache *regcache = get_current_regcache ();
  struct gdbarch *gdbarch = get_regcache_arch (regcache);
  enum bfd_endian byte_order;
  int regnum = -1;
  gdb_byte regbuf[4] = { 0 };

  if (nds32_remote_info.endian == BFD_ENDIAN_UNKNOWN)
    byte_order = gdbarch_byte_order (gdbarch);
  else
    byte_order = nds32_remote_info.endian;

  switch ((INDEX_HW_MASK & regno))
    {
    case INDEX_HW_CPU:
      switch ((regno & SR_INDEX_MASK))
	{
	case CPU_SR_INDEX (0, 0, 0):	/* cr0 */
	  regnum = user_reg_map_name_to_regnum (gdbarch, "cr0", -1);
	  break;
	case CPU_SR_INDEX (0, 1, 0):	/* cr1 */
	  regnum = user_reg_map_name_to_regnum (gdbarch, "cr1", -1);
	  break;
	case CPU_SR_INDEX (0, 2, 0):	/* cr2 */
	  regnum = user_reg_map_name_to_regnum (gdbarch, "cr2", -1);
	  break;
	case CPU_SR_INDEX (0, 3, 0):	/* cr3 */
	  regnum = user_reg_map_name_to_regnum (gdbarch, "cr3", -1);
	  break;
	case CPU_SR_INDEX (0, 4, 0):	/* cr4 */
	  regnum = user_reg_map_name_to_regnum (gdbarch, "cr4", -1);
	  break;
	case CPU_SR_INDEX (0, 0, 1):	/* cr5 */
	  regnum = user_reg_map_name_to_regnum (gdbarch, "cr5", -1);
	  break;
	case CPU_SR_INDEX (0, 5, 0):	/* cr6 */
	  regnum = user_reg_map_name_to_regnum (gdbarch, "cr6", -1);
	  break;
	default:
	  return -1;
	}

      if (regnum == -1)
	error ("Fail to access system registers for elf-check.");

      /* Use target-endian instead of gdbarch-endian.  */
      if (regcache_cooked_read (regcache, regnum, regbuf) != REG_VALID)
	return -1;
      regval = extract_unsigned_integer (regbuf, 4, byte_order);

      return regval;
    }

  return -1;
}

/* Translate an ELF file header in internal format into an ELF file header in
   external format.

   elf_swap_ehdr_out () is copied from bfd/elfcode.h.  */

#define Elf_External_Ehdr	Elf32_External_Ehdr
#define H_PUT_WORD		H_PUT_32
#define H_PUT_SIGNED_WORD	H_PUT_S32
#define H_GET_WORD		H_GET_32
#define H_GET_SIGNED_WORD	H_GET_S32

static void
elf_swap_ehdr_out (bfd *abfd, const Elf_Internal_Ehdr *src,
		   Elf_External_Ehdr *dst)
{
  unsigned int tmp;
  int signed_vma = get_elf_backend_data (abfd)->sign_extend_vma;

  memcpy (dst->e_ident, src->e_ident, EI_NIDENT);
  /* Note that all elements of dst are *arrays of unsigned char* already...  */
  H_PUT_16 (abfd, src->e_type, dst->e_type);
  H_PUT_16 (abfd, src->e_machine, dst->e_machine);
  H_PUT_32 (abfd, src->e_version, dst->e_version);
  if (signed_vma)
    H_PUT_SIGNED_WORD (abfd, src->e_entry, dst->e_entry);
  else
    H_PUT_WORD (abfd, src->e_entry, dst->e_entry);
  H_PUT_WORD (abfd, src->e_phoff, dst->e_phoff);
  H_PUT_WORD (abfd, src->e_shoff, dst->e_shoff);
  H_PUT_32 (abfd, src->e_flags, dst->e_flags);
  H_PUT_16 (abfd, src->e_ehsize, dst->e_ehsize);
  H_PUT_16 (abfd, src->e_phentsize, dst->e_phentsize);
  tmp = src->e_phnum;
  if (tmp > PN_XNUM)
    tmp = PN_XNUM;
  H_PUT_16 (abfd, tmp, dst->e_phnum);
  H_PUT_16 (abfd, src->e_shentsize, dst->e_shentsize);
  tmp = src->e_shnum;
  if (tmp >= (SHN_LORESERVE & 0xffff))
    tmp = SHN_UNDEF;
  H_PUT_16 (abfd, tmp, dst->e_shnum);
  tmp = src->e_shstrndx;
  if (tmp >= (SHN_LORESERVE & 0xffff))
    tmp = SHN_XINDEX & 0xffff;
  H_PUT_16 (abfd, tmp, dst->e_shstrndx);
}

/* Callback for "nds32 elf-check" command.  */

static void
nds32_elf_check_command (char *arg, int from_tty)
{
  Elf_External_Ehdr x_ehdr;
  Elf_Internal_Ehdr *i_ehdrp;
  char check_msg[0x1000];
  unsigned int buf_status = 0;
  int err;

  if (exec_bfd == NULL)
    error (_("Cannot check ELF without executable.\n"
	     "Use the \"file\" or \"exec-file\" command."));

  /* elf-check with SID/ICE only. */
  if (nds32_remote_info.type == nds32_rt_unknown)
    error (_("Cannot check ELF without SID/ICE.\n"));

  i_ehdrp = elf_elfheader (exec_bfd);
  elf_swap_ehdr_out (exec_bfd, i_ehdrp, &x_ehdr);

  err = elf_check ((unsigned char *) &x_ehdr, nds32_elf_check_get_register,
		   check_msg, sizeof (check_msg), &buf_status);

  if (err)
    {
      /* internal test */
      if ((arg != NULL) && (strcmp (arg, "test") == 0))
	warning ("%s", check_msg);
      else
	error ("%s", check_msg);
    }
}

static void
nds32_endian_check_command (char *arg, int from_tty)
{
  enum bfd_endian elf_endian = BFD_ENDIAN_UNKNOWN;

  /* ELF file is necessary for endian comparison.  */
  if (exec_bfd == NULL)
    return;

  /* The comparison is only for remote debugging.  */
  if (strcmp (target_shortname, "remote") != 0
      && strcmp (target_shortname, "extended-remote") != 0)
    return;

  if (nds32_remote_info.type == nds32_rt_unknown)
    return;

  elf_endian = exec_bfd->xvec->byteorder;

  if (nds32_remote_info.endian != elf_endian)
    warning ("Target and elf have different endian");
}

/* This is only used for SID.  Set command-line string.  */

static void
nds32_set_gloss_command (char *arg, int from_tty)
{
  int i;
  struct ui_file *out;
  char *arg0;
  char *args;
  char *f;
  char cmdline[0x1000];		/* 4K for max command line.  */
  struct cleanup *back_to;
  asection *s = NULL;
  const char *sectnames[] = { ".text", "code", ".bss", "bss" };

  /* set gloss for SID only. */
  if (nds32_remote_info.type != nds32_rt_sid)
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
  args = get_inferior_args ();

  f = strrchr (arg0, '/');
  if (f == NULL)
    f = strrchr (arg0, '\\');

  if (f == NULL)
    f = "a.out";
  else
    f++; /* skip separator.  */

  xsnprintf (cmdline, sizeof (cmdline),
	     "set gloss command-line \"%s %s\"", f, args);
  target_rcmd (cmdline, out);

  do_cleanups (back_to);
}

static int
nds32_get_acr_info (struct gdbarch *gdbarch, const char *name,
		    int *regnum, int *len)
{
  struct type *type;
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

/* Callback for "nds32 print" command, which is used to construct a
   hex string from the content of ACR.  */

static void
nds32_print_acr_command (char *args, int from_tty)
{
  struct regcache *regcache = get_current_regcache ();
  struct gdbarch *gdbarch = get_regcache_arch (regcache);
  enum bfd_endian byte_order = gdbarch_byte_order (gdbarch);
  struct cleanup *back_to;
  int regnum;
  char **argv = NULL;
  char *name = NULL;
  int len, i;
  /* Flag used to trim leading-zero.  */
  int flag_lz;
  /* +1 for null terminating char.  */
  char val_str[MAX_ACR_HEX_DIGIT + 1];
  char *str_p;
  gdb_byte *acr_content;

  if (args == NULL)
    {
      fprintf_unfiltered (gdb_stdout, "<usage>: nds32 print <acr_name>\n");
      return;
    }

  /* Parse arguments.  */
  argv = gdb_buildargv (args);
  back_to = make_cleanup_freeargv (argv);

  if (argv[0] == NULL)
    {
      fprintf_unfiltered (gdb_stdout, "<usage>: nds32 print <acr_name>\n");
      goto out;
    }

  name = argv[0];
  if (nds32_get_acr_info (gdbarch, name, &regnum, &len) == -1)
    goto out;

  /* Allocate space for ACR.  */
  acr_content = (gdb_byte *) xcalloc (1, len);
  make_cleanup (xfree, acr_content);

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

out:
  do_cleanups (back_to);
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

/* Callback for "nds32 set" command, which is used to construct
   the content of ACR from the given hex string.  */

static void
nds32_set_acr_command (char *args, int from_tty)
{
  struct regcache *regcache = get_current_regcache ();
  struct gdbarch *gdbarch = get_regcache_arch (regcache);
  enum bfd_endian byte_order = gdbarch_byte_order (gdbarch);
  struct cleanup *back_to;
  int regnum;
  char **argv = NULL;
  char *name = NULL;
  int len, i;
  const char *val_str, *str_p;
  gdb_byte *acr_content = NULL;

  if (args == NULL)
    {
      fprintf_unfiltered (gdb_stdout,
			  "<usage>: nds32 set <acr_name> <hex_str>\n");
      return;
    }

  /* Parse arguments.  */
  argv = gdb_buildargv (args);
  back_to = make_cleanup_freeargv (argv);

  if (argv[0] == NULL || argv[1] == NULL)
    {
      fprintf_unfiltered (gdb_stdout,
			  "<usage>: nds32 set <acr_name> <hex_str>\n");
      goto out;
    }

  name = argv[0];
  if (nds32_get_acr_info (gdbarch, name, &regnum, &len) == -1)
    goto out;

  val_str = argv[1];
  if (val_str[0] == '0' && (val_str[1] == 'x' || val_str[1] == 'X'))
    val_str += 2;

  acr_content = (gdb_byte *) xcalloc (1, len);
  make_cleanup (xfree, acr_content);

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
  regcache_invalidate (regcache, regnum);

out:
  do_cleanups (back_to);
}

/* Bug 6654 - Multiple watchpoints was be hit, GDB only shows one of them.

   FIXME: This is a dirty hacking for hooking remote->to_stopped_data_address,
   in order to handling multiple hit. This is not a bug at all.  */

/* remote_ops and extended_remote_ops have the same to_stopped_data_address
   handler, so just a variable is enough to record the original one.  */
static int (*remote_stopped_data_address_p) (struct target_ops*, CORE_ADDR*);

static int
nds32_remote_stopped_data_address (struct target_ops *target, CORE_ADDR *addr_p)
{
  if (!remote_stopped_data_address_p (target, addr_p))
    return 0;

  /* If the addr is 0x0, we assume SMW multiple hits.
     Pretent data_address is unknown and let GDB figure it out.  */
  return (*addr_p) != 0;
}

static void
nds32_remote_inferior_created_observer (struct target_ops *target, int from_tty)
{
  /* nds32_query_target_command (NULL, 0); */

  /* Hack the member to_stopped_data_address in remote_ops/extend_remote_ops
     without modifying the generic code.

     @TARGET is always &current_target, use beneath to get the value
     of target_stack.  (See update_current_target @ target.c.)  */
  struct target_ops *t = target->beneath;

  /* remote_ops and extended_remote_ops have process_stratum, which is higher
     than file_stratum, so they can be found via target_stack first.  */
  if ((strcmp (t->to_shortname, "remote") == 0
       || strcmp (t->to_shortname, "extended-remote") == 0)
      && t->to_stopped_data_address != nds32_remote_stopped_data_address)
    {
      remote_stopped_data_address_p = t->to_stopped_data_address;
      t->to_stopped_data_address = nds32_remote_stopped_data_address;

      /* current_target is updated before inferior_created event, so the member
	 to_stopped_data_address may be the original one at the first time.
	 This issue can be avoided by always replacing this member.  */
      if ((strcmp (target->to_shortname, "remote") == 0
	   || strcmp (target->to_shortname, "extended-remote") == 0))
	{
	  target->to_stopped_data_address = nds32_remote_stopped_data_address;
	}
    }
}

void
qsort (void *base, size_t nmemb, size_t size,
       int (*compar) (const void *lhs, const void *rhs))
{
  nds32_insertion_sort (base, nmemb, size, compar);
}

static struct cmd_list_element *nds32_pipeline_cmdlist;
static struct cmd_list_element *nds32_query_cmdlist;
static struct cmd_list_element *nds32_reset_cmdlist;
static struct cmd_list_element *nds32_maint_cmdlist;

static const struct internalvar_funcs nds32_target_type_funcs =
{
  nds32_target_type_make_value,
  NULL,
  NULL
};

extern struct cmd_list_element *nds32_cmdlist;

void
nds32_init_remote_cmds (void)
{
  /* Hook for query remote target information.  */
  observer_attach_inferior_created (nds32_remote_inferior_created_observer);

  nds32_remote_info_init ();

  /* nds32 elf-check */
  add_cmd ("elf-check", class_files, nds32_elf_check_command,
	   _("Check elf/target compatibility before loading. "
	     "Throwing error if failed."),
	   &nds32_cmdlist);

  add_cmd ("endian-check", class_files, nds32_endian_check_command,
	   _("Check endian consistency between elf and target. "
	     "Throwing warning if failed."),
	   &nds32_cmdlist);

  /* nds32 set-gloss COMMAND_LINE */
  add_cmd ("set-gloss", class_files, nds32_set_gloss_command,
	   _("Check elf/target compatibility before loading. "
	     "Throwing error if failed."),
	   &nds32_cmdlist);

  /* nds32 reset-target */
  add_cmd ("reset-target", no_class, nds32_reset_target_command,
	   _("Turn on pipeline for profiling."), &nds32_cmdlist);
  /* nds32 reset-hold */
  add_cmd ("reset-hold", no_class, nds32_reset_hold_command,
	   _("Turn off pipeline for profiling."), &nds32_cmdlist);

  /* nds32 pipeline (on|off) */
  add_prefix_cmd ("pipeline", no_class, nds32_pipeline_command,
		  _("nds32-sid profiling commands."),
		  &nds32_pipeline_cmdlist, "pipeline ", 0, &nds32_cmdlist);
  add_cmd ("on", no_class, nds32_pipeline_on_command,
	   _("Turn on pipeline for profiling."), &nds32_pipeline_cmdlist);
  add_cmd ("off", no_class, nds32_pipeline_off_command,
	   _("Turn off pipeline for profiling."), &nds32_pipeline_cmdlist);

  /* nds32 query (profiling|perf-meter|target)  */
  add_prefix_cmd ("query", no_class, nds32_query_command,
		  _("Query remote data."), &nds32_query_cmdlist, "query ",
		  0, &nds32_cmdlist);
  add_cmd ("profiling", no_class, nds32_query_profiling_command,
	   _("Query profiling results."), &nds32_query_cmdlist);
  add_cmd ("perf-meter", no_class, nds32_query_perfmeter_command,
	   _("Query perf-meter results."), &nds32_query_cmdlist);
  add_cmd ("target", no_class, nds32_query_target_command,
	   _("Query target information."), &nds32_query_cmdlist);

  /* nds32 reset (profiling|perf-meter)  */
  add_prefix_cmd ("reset", no_class, nds32_reset_command,
		  _("Reset profiling."), &nds32_reset_cmdlist, "reset ",
		  0, &nds32_cmdlist);
  add_cmd ("profiling", no_class, nds32_reset_profiling_command,
	   _("Query profiling results."), &nds32_reset_cmdlist);
  add_cmd ("perf-meter", no_class, nds32_reset_perfmeter_command,
	   _("Query perf-meter results."), &nds32_reset_cmdlist);

  /* nds32 read_acedesc  */
  add_cmd ("read_acedesc", no_class, nds32_read_ace_desc_command,
	   _("Request the ACE or coprocessor description file from remote."),
	   &nds32_cmdlist);

  /* nds32 print  */
  add_cmd ("print", no_class, nds32_print_acr_command,
	   _("Print the value of ACR in hex format."), &nds32_cmdlist);

  /* nds32 set  */
  add_cmd ("set", no_class, nds32_set_acr_command,
	   _("Set the value of ACR in hex format."), &nds32_cmdlist);

  create_internalvar_type_lazy ("_nds32_target_type", &nds32_target_type_funcs,
				NULL);
}
