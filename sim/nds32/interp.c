/* Simulator for NDS32 processors.

   Copyright (C) 2011-2013 Free Software Foundation, Inc.
   Contributed by Andes Technology Corporation.

   This file is part of simulators.

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

#include "config.h"

#include <stdlib.h>
#include "bfd.h"
#include "elf-bfd.h"
#include "gdb/callback.h"
#include "gdb/signals.h"
#include "libiberty.h"
#include "gdb/remote-sim.h"
#include "gdb/sim-nds32.h"
#include "dis-asm.h"
#include "sim-main.h"
#include "nds32-sim.h"
#include "sim-utils.h"
#include "sim-fpu.h"
#include "sim-trace.h"
#include "sim-options.h"

#include "opcode/nds32.h"
#include "nds32-sim.h"
#include "nds32-syscall.h"

#include <sys/types.h>
#include <unistd.h>
#include <time.h>

/* Some hosts don't define LITTLE_ENDIAN or BIG_ENDIAN, help them out */

#ifndef LITTLE_ENDIAN
#define LITTLE_ENDIAN 1234
#endif
#ifndef BIG_ENDIAN
#define BIG_ENDIAN 4321
#endif

struct disassemble_info dis_info; /* For print insn.  */

static void nds32_set_nia (sim_cpu *cpu, sim_cia nia);

/* Recent $pc, for debugging.  */
#define RECENT_CIA_MASK	0xf
static sim_cia recent_cia[RECENT_CIA_MASK + 1];
static int recent_cia_idx = 0;

static uint64_t
extract_unsigned_integer (unsigned char *addr, int len, int byte_order)
{
  uint64_t retval;
  const unsigned char *p;
  const unsigned char *startaddr = addr;
  const unsigned char *endaddr = startaddr + len;

  retval = 0;
  if (byte_order == BIG_ENDIAN)
    {
      for (p = startaddr; p < endaddr; ++p)
	retval = (retval << 8) | *p;
    }
  else
    {
      for (p = endaddr - 1; p >= startaddr; --p)
	retval = (retval << 8) | *p;
    }
  return retval;
}

static void
store_unsigned_integer (unsigned char *addr, int len,
			int byte_order, uint64_t val)
{
  unsigned char *p;
  unsigned char *startaddr = addr;
  unsigned char *endaddr = startaddr + len;

  /* Start at the least significant end of the integer,
     and work towards the most significant.  */
  if (byte_order == BIG_ENDIAN)
    {
      for (p = endaddr - 1; p >= startaddr; --p)
	{
	  *p = val & 0xff;
	  val >>= 8;
	}
    }
  else
    {
      for (p = startaddr; p < endaddr; ++p)
	{
	  *p = val & 0xff;
	  val >>= 8;
	}
    }
}

static void
nds32_dump_registers (SIM_DESC sd)
{
  static char *reg2names[] = {
	"r0", "r1", "r2", "r3", "r4", "r5",
	"r6", "r7", "r8", "r9", "10", "11",
	"12", "13", "14", "ta", "16", "17",
	"18", "19", "20", "21", "22", "23",
	"24", "25", "p0", "p1", "fp", "gp",
	"lp", "sp"
	};
  int i;
  int j;

  for (i = 0; i < MAX_NR_PROCESSORS; ++i)
    {
      sim_cpu *cpu = STATE_CPU (sd, i);
      sim_io_eprintf (sd, "pc  %08x\n", CCPU_USR[USR0_PC].u);

      for (j = 0; j < 32; j++)
	{
	  sim_io_eprintf (sd, "%s  %08x  ", reg2names[j], CCPU_GPR[j].u);
	  if (j % 6 == 5)
	    sim_io_eprintf (sd, "\n");
	}
      sim_io_eprintf (sd, "\n");

      sim_io_eprintf (sd, "itb %08x  ", CCPU_USR[USR0_ITB].u);
      sim_io_eprintf (sd, "ifc %08x  ", CCPU_USR[USR0_IFCLP].u);
      sim_io_eprintf (sd, "d0  %08x  ", CCPU_USR[USR0_D0LO].u);
      sim_io_eprintf (sd, "hi  %08x  ", CCPU_USR[USR0_D0HI].u);
      sim_io_eprintf (sd, "d1  %08x  ", CCPU_USR[USR0_D1LO].u);
      sim_io_eprintf (sd, "hi  %08x  ", CCPU_USR[USR0_D1HI].u);
      sim_io_eprintf (sd, "\n");

      sim_io_eprintf (sd, "psw %08x  ", CCPU_SR[SRIDX_PSW].u);
      sim_io_eprintf (sd, "\n");
    }

  sim_io_eprintf (sd, "Recent $pc:\n");
  for (i = 0; i <= RECENT_CIA_MASK; i++)
    {
      sim_io_eprintf (sd, "  0x%x",
		      recent_cia[(i + recent_cia_idx) & RECENT_CIA_MASK]);
      if (i % 6 == 5)
	sim_io_eprintf (sd, "\n");
    }

  sim_io_eprintf (sd, "\n");
}

uint32_t
nds32_raise_exception (sim_cpu *cpu, enum nds32_exceptions e, int sig,
		       char *msg, ...)
{
  SIM_DESC sd = CPU_STATE (cpu);
  uint32_t cia = CCPU_USR[USR0_PC].u;
  int i;

  if (msg)
    {
      va_list va;
      va_start (va, msg);
      sim_io_evprintf (sd, msg, va);
      va_end (va);
    }

  /* Dump registers before halt.  */
  if (STATE_OPEN_KIND (sd) != SIM_OPEN_DEBUG)
    {
      fprintf (stderr, "  ");
      print_insn_nds32 (cia, &dis_info);
      fprintf (stderr, "\n");
      nds32_dump_registers (sd);
    }

  sim_engine_halt (CPU_STATE (cpu), cpu, NULL, cia, sim_stopped, sig);

  return cia;
}

void
nds32_bad_op (sim_cpu *cpu, uint32_t cia, uint32_t insn, char *tag)
{
  if (tag == NULL)
    tag = "";

  nds32_raise_exception (cpu, EXP_GENERAL, SIM_SIGILL,
			 "Illegal/Unhandled %s instruction (%08x)\n", tag, insn);
}

/* Load an integer in endian specified in PSW.BE flag.  */

uint64_t
__nds32_ld (sim_cpu *cpu, SIM_ADDR addr, int size, int aligned_p)
{
  int r, order;
  uint64_t val = 0;
  SIM_DESC sd = CPU_STATE (cpu);

  SIM_ASSERT (size <= sizeof (uint64_t));

  if (aligned_p && (addr & (size - 1)) != 0)
    nds32_raise_exception (cpu, EXP_GENERAL, SIM_SIGSEGV,
			   "Alignment check exception. "
			   "Read of address 0x%08x in size of %d.\n",
			   addr, size);

  r = sim_read (sd, addr, (unsigned char *) &val, size);
  order = CCPU_SR_TEST (PSW, PSW_BE) ? BIG_ENDIAN : LITTLE_ENDIAN;
  val = extract_unsigned_integer ((unsigned char *) &val, size, order);

  if (r == size)
    return val;

  nds32_raise_exception (cpu, EXP_GENERAL, SIM_SIGSEGV,
			 "Access violation. Read of address 0x%08x.\n", addr);

  return val;
}

/* Store an integer in endian specified in PSW.BE flag.  */

void
__nds32_st (sim_cpu *cpu, SIM_ADDR addr, int size, uint64_t val,
	    int aligned_p)
{
  int r, order;
  SIM_DESC sd = CPU_STATE (cpu);

  SIM_ASSERT (size <= sizeof (uint64_t));

  if (aligned_p && (addr & (size - 1)) != 0)
    nds32_raise_exception (cpu, EXP_GENERAL, SIM_SIGSEGV,
			   "Alignment check exception. "
			   "Write of address 0x%08x in size of %d.\n",
			   addr, size);

  order = CCPU_SR_TEST (PSW, PSW_BE) ? BIG_ENDIAN : LITTLE_ENDIAN;
  store_unsigned_integer ((unsigned char *) &val, size, order, val);
  r = sim_write (sd, addr, (unsigned char *) &val, size);

  if (r == size)
    return;

  nds32_raise_exception (cpu, EXP_GENERAL, SIM_SIGSEGV,
			 "Access violation. Write of address 0x%08x\n", addr);

  return;
}

/* Set next-instructoin-address, so sim_engine_run () fetches `nia'
   instead of ($pc + 4) or ($pc + 2) for next instruction base on
   currenly instruction size. */

static void
nds32_set_nia (sim_cpu *cpu, sim_cia nia)
{
  cpu->iflags |= NIF_BRANCH;
  cpu->baddr = nia;
}

static int64_t
get_double (sim_cpu *cpu, int regnum)
{
  if (CCPU_SR_TEST (PSW, PSW_BE))
    return ((int64_t) CCPU_GPR[regnum].s << 32)
	    | ((int64_t) CCPU_GPR[regnum + 1].s & 0xFFFFFFFF);
  else
    return ((int64_t) CCPU_GPR[regnum + 1].s << 32)
	    | ((int64_t) CCPU_GPR[regnum].s & 0xFFFFFFFF);
}

static uint64_t
get_udouble (sim_cpu *cpu, int regnum)
{
  if (CCPU_SR_TEST (PSW, PSW_BE))
    return ((uint64_t) CCPU_GPR[regnum].u << 32)
	    | ((uint64_t) CCPU_GPR[regnum + 1].u & 0xFFFFFFFF);
  else
    return ((uint64_t) CCPU_GPR[regnum + 1].u << 32)
	    | ((uint64_t) CCPU_GPR[regnum].u & 0xFFFFFFFF);
}

static void
set_double (sim_cpu *cpu, int regnum, int64_t val)
{
  if (CCPU_SR_TEST (PSW, PSW_BE))
    {
      CCPU_GPR[regnum + 1].s = val & 0xFFFFFFFF;
      CCPU_GPR[regnum].s = (val >> 32) & 0xFFFFFFFF;
    }
  else
    {
      CCPU_GPR[regnum].s = val & 0xFFFFFFFF;
      CCPU_GPR[regnum + 1].s = (val >> 32) & 0xFFFFFFFF;
    }
}

static void
set_udouble (sim_cpu *cpu, int regnum, uint64_t val)
{
  if (CCPU_SR_TEST (PSW, PSW_BE))
    {
      CCPU_GPR[regnum + 1].u = val & 0xFFFFFFFF;
      CCPU_GPR[regnum].u = (val >> 32) & 0xFFFFFFFF;
    }
  else
    {
      CCPU_GPR[regnum].u = val & 0xFFFFFFFF;
      CCPU_GPR[regnum + 1].u = (val >> 32) & 0xFFFFFFFF;
    }
}

static int32_t
insn_sat_helper (sim_cpu *cpu, int64_t res, const short range)
{
  const int max = (1 << range) - 1;
  const int min = -(1 << range);

  if (res > max)
    {
      res = max;
      CCPU_SR_SET (PSW, PSW_OV);
    }
  else if (res < min)
    {
      res = min;
      CCPU_SR_SET (PSW, PSW_OV);
    }

  return res;
}

static int32_t
insn_usat_helper (sim_cpu *cpu, int64_t res, const short range)
{
  const uint32_t max = (1 << range) - 1;

  if (res > max)
    {
      res = max;
      CCPU_SR_SET (PSW, PSW_OV);
    }
  else if (res < 0)
    {
      res = 0;
      CCPU_SR_SET (PSW, PSW_OV);
    }

  return res;
}

static int16_t
insn_sat_khm_helper (sim_cpu *cpu, int16_t aop, int16_t bop)
{
  int16_t res;
  if (((int16_t) 0x8000 != aop) || ((int16_t) 0x8000 != bop))
    res = (int16_t) (((int32_t) aop * bop) >> 15);
  else
    {
      res = 0x7fff;
      CCPU_SR_SET (PSW, PSW_OV);
    }
  return res;
}

static int32_t
get_abs (sim_cpu *cpu, int32_t val, const short range)
{
  const int max = (1 << range) - 1;
  const int min = -(1 << range);

  if (val == min)
    {
      val = max;
      CCPU_SR_SET (PSW, PSW_OV);
    }
  else if (val < 0)
    val = -val;

  return val;
}

/* Find first zero byte or mis-match in sequential memory address.
   If no such byte is found, return 0.  */

static uint32_t
find_null_mism (unsigned char *b1, unsigned char *b2)
{
  int i;

  for (i = 0; i < 4; i++)
    {
      if ((b1[i] == '\0') || (b1[i] != b2[i]))
	return -4 + i;
    }
  return 0;
}

/* Find first mis-match in sequential memory address.
   The 3rd argument inc: 1 means incremental memory address.
			-1 means decremental memory address.
   If no such byte is found, return 0.  */

static uint32_t
find_mism (unsigned char *b1, unsigned char *b2, int inc)
{
  int i, end;

  i = (inc == 1) ? 0 : 3;
  end = (inc == 1) ? 3 : 0;

  while (1)
    {
      if ((b1[i] != b2[i]))
	return -4 + i;
      if (i == end)
	return 0;
      i += inc;
    }
}

static void
nds32_decode32_mem (sim_cpu *cpu, const uint32_t insn, sim_cia cia)
{
  const int rt = N32_RT5 (insn);
  const int ra = N32_RA5 (insn);
  const int rb = N32_RB5 (insn);
  const int sv = __GF (insn, 8, 2);
  const int op = insn & 0xFF;
  uint32_t addr;
  uint32_t shift;

  switch (op)
    {
    case 0x0:			/* lb */
    case 0x1:			/* lh */
    case 0x2:			/* lw */
    case 0x3:			/* ld */
      addr = CCPU_GPR[ra].u + (CCPU_GPR[rb].u << sv);
      CCPU_GPR[rt].u = nds32_ld_aligned (cpu, addr, (1 << (op)));
      break;
    case 0x4:			/* lb.bi */
    case 0x5:			/* lh.bi */
    case 0x6:			/* lw.bi */
    /* case 0x7: */		/* ld.bi */
      /* UNPREDICTABLE if rt is equal to ra.
	 Compute address before load, because rb could be rt.  */
      addr = CCPU_GPR[ra].u + (CCPU_GPR[rb].u << sv);
      CCPU_GPR[rt].u = nds32_ld_aligned (cpu, CCPU_GPR[ra].u, (1 << (op & 0x3)));
      CCPU_GPR[ra].u = addr;
      break;
    case 0x8:			/* sb */
    case 0x9:			/* sh */
    case 0xa:			/* sw */
    /* case 0xb: */		/* sd */
      addr = CCPU_GPR[ra].u + (CCPU_GPR[rb].u << sv);
      nds32_st_aligned (cpu, addr, (1 << (op & 0x3)), CCPU_GPR[rt].u);
      break;
    case 0xc:			/* sb.bi */
    case 0xd:			/* sh.bi */
    case 0xe:			/* sw.bi */
    /* case 0xf: */		/* sd.bi */
      nds32_st_aligned (cpu, CCPU_GPR[ra].u, (1 << (op & 0x3)),
			CCPU_GPR[rt].u);
      CCPU_GPR[ra].u += (CCPU_GPR[rb].u << sv);
      break;
    case 0x10:			/* lbs */
    case 0x11:			/* lhs */
    /* case 0x12: */		/* lws */
      addr = CCPU_GPR[ra].u + (CCPU_GPR[rb].u << sv);
      CCPU_GPR[rt].u =
	nds32_ld_aligned (cpu, addr, (1 << (op & 0x3)));
      CCPU_GPR[rt].u = __SEXT (CCPU_GPR[rt].u, (1 << (op & 0x3)) * 8);
      break;
    case 0x13:			/* dpref */
      /* do nothing */
      break;
    case 0x14:			/* lbs.bi */
    case 0x15:			/* lhs.bi */
    /* case 0x16: */		/* lws.bi */
      /* UNPREDICTABLE if rt is equal to ra.
	Compute address before load, because rb could be rt.  */
      addr = CCPU_GPR[ra].u + (CCPU_GPR[rb].u << sv);
      CCPU_GPR[rt].u = nds32_ld_aligned (cpu, CCPU_GPR[ra].u, (1 << (op & 0x3)));
      CCPU_GPR[rt].u = __SEXT (CCPU_GPR[rt].u, (1 << (op & 0x3)) * 8);
      CCPU_GPR[ra].u = addr;
      break;
    case 0x18:			/* llw */
      CCPU_GPR[rt].u =
	nds32_ld_aligned (cpu, CCPU_GPR[ra].u + (CCPU_GPR[rb].u << sv), 4);
      break;
    case 0x19:			/* scw */
      /* SCW always successes.  */
      nds32_st_aligned (cpu, CCPU_GPR[ra].u + (CCPU_GPR[rb].u << sv), 4,
			CCPU_GPR[rt].u);
      CCPU_GPR[rt].u = 1;
      break;
    case 0x20:			/* lbup */
    case 0x22:			/* lwup */
    case 0x28:			/* sbup */
    case 0x2a:			/* swup */
    default:
      nds32_bad_op (cpu, cia, insn, "MEM");
      return;
    }
}

static void
nds32_decode32_lsmw (sim_cpu *cpu, const uint32_t insn, sim_cia cia)
{
  SIM_DESC sd = CPU_STATE (cpu);
  int rb, re, ra, enable4, i, ret;
  char buf[4];
  int wac;			/* With alignment-check?  */
  int reg_cnt = 0;		/* Total number of registers count.  */
  int di;			/* dec=-1 or inc=1  */
  int size = 4;			/* The load/store bytes.  */
  int len = 4;			/* The length of a fixed-size string.  */
  int order = CCPU_SR_TEST (PSW, PSW_BE) ? BIG_ENDIAN : LITTLE_ENDIAN;
  char enb4map[2][4] = { {3, 2, 1, 0}, {0, 1, 2, 3} };
  uint32_t val = 0;
  SIM_ADDR base = -1;

  /* Filter out undefined opcode.  */
  if ((insn & 0x3) == 0x3)
    {
      nds32_bad_op (cpu, cia, insn, "LSMW");
      return;
    }

  /* Filter out invalid opcode.  */
  if ((insn & 0xB) == 0xA)
    {
      nds32_bad_op (cpu, cia, insn, "LSMW");
      return;
    }

  /* Decode instruction.  */
  rb = N32_RT5 (insn);
  ra = N32_RA5 (insn);
  re = N32_RB5 (insn);
  enable4 = (insn >> 6) & 0x0F;
  wac = (insn & 1) ? 1 : 0;
  di = __TEST (insn, 3) ? -1 : 1;

  /* Get the first memory address  */
  base = CCPU_GPR[ra].u;

  /* Do the alignment check. */
  if (wac && (base & 0x3))
    {
      nds32_raise_exception (cpu, EXP_GENERAL, SIM_SIGSEGV,
			     (insn & 0x20)
			     ? "Alignment check exception (SMWA). "
			       "Write of address 0x%08x.\n"
			     : "Alignment check exception (LMWA). "
			       "Read of address 0x%08x.\n",
			     base);
      return;
    }

  /* Sum up the registers count.  */
  reg_cnt += (enable4 & 0x1) ? 1 : 0;
  reg_cnt += (enable4 & 0x2) ? 1 : 0;
  reg_cnt += (enable4 & 0x4) ? 1 : 0;
  reg_cnt += (enable4 & 0x8) ? 1 : 0;
  if (rb < GPR_FP && re < GPR_FP)
    {
      reg_cnt += (re - rb) + 1;
    }

  if (rb > re)
    {
      nds32_raise_exception (
	cpu, EXP_GENERAL, SIM_SIGILL,
	"Illegal encoding for smw/lmw (Rb > Re) instruction (%08x)\n",
	insn);
    }

  if (rb >= GPR_FP && (rb != GPR_SP || re != GPR_SP))
    {
      nds32_raise_exception (
	cpu, EXP_GENERAL, SIM_SIGILL,
	"Illegal encoding for smw/lmw (Rb > $fp, only"
	" exception is Rb = Re = $sp) instruction (%08x)\n",
	insn);
    }

  /* Generate the first memory address.  */
  if (__TEST (insn, 4))
    base += 4 * di;
  /* Set base to the lowest memory address we are going to access.
     Because we may load/store in increasing or decreasing order,
     always access the memory from the lowest address simplify the
     opertions.  */
  if (__TEST (insn, 3))
    base -= (reg_cnt - 1) * 4;

  for (i = rb; i <= re && rb < GPR_FP; i++)
    {
      if (insn & 0x20)
	{
	  /* SMW */

	  val = CCPU_GPR[i].u;
	  store_unsigned_integer ((unsigned char *) buf, 4, order, val);
	  if ((insn & 0x3) == 0x2)
	    {
	      /* Until zero byte case.  */
	      len = strnlen (buf, 4);
	      size = (len == 4) ? 4 : len + 1;	/* Include zero byte.  */
	    }
	  ret = sim_write (sd, base, (unsigned char *) buf, size);
	  if (ret != size)
	    {
	      nds32_raise_exception (cpu, EXP_GENERAL, SIM_SIGSEGV,
				     "Access violation. Write of address %#x\n",
				     base);
	    }
	  if (len < 4)
	    goto zero_byte_exist;
	}
      else
	{
	  /* LMW */

	  ret = sim_read (sd, base, (unsigned char *) buf, 4);
	  if (ret != 4)
	    {
	      nds32_raise_exception (cpu, EXP_GENERAL, SIM_SIGSEGV,
				     "Access violation. Write of address %#x\n",
				     base);
	    }
	  val = extract_unsigned_integer ((unsigned char *) buf, 4, order);
	  CCPU_GPR[i].u = val;
	  if ((insn & 0x3) == 0x2)
	    {
	      /* Until zero byte case.  */
	      len = strnlen (buf, 4);
	      if (len < 4)
		goto zero_byte_exist;
	    }
	}
      base += 4;
    }

  /* Load/store the 4 individual registers from low address memory
     to high address memory. */
  for (i = 0; i < 4; i++)
    {
      if (__TEST (enable4, enb4map[wac][i]))
	{
	  if (insn & 0x20)
	    {
	      /* SMW */

	      val = CCPU_GPR[GPR_SP - (enb4map[wac][i])].u;
	      store_unsigned_integer ((unsigned char *) buf, 4, order, val);
	      if ((insn & 0x3) == 0x2)	/* Until zero byte case.  */
		{
		  len = strnlen (buf, 4);
		  size = (len == 4) ? 4 : len + 1;	/* Include zero byte.  */
		}
	      ret = sim_write (sd, base, (unsigned char *) buf, size);
	      if (ret != size)
		{
		  nds32_raise_exception (cpu, EXP_GENERAL, SIM_SIGSEGV,
					 "Access violation. Write of address %#x\n",
					 base);
		}
	      if (len < 4)
		goto zero_byte_exist;
	    }
	  else
	    {
	      /* LMW */

	      ret = sim_read (sd, base, (unsigned char *) buf, 4);
	      if (ret != 4)
		{
		  nds32_raise_exception (cpu, EXP_GENERAL, SIM_SIGSEGV,
					 "Access violation. Write of address %#x\n",
					 base);
		}
	      val =
		extract_unsigned_integer ((unsigned char *) buf, 4, order);
	      CCPU_GPR[GPR_SP - (enb4map[wac][i])].u = val;
	      if ((insn & 0x3) == 0x2)	/* until zero byte ? */
		{
		  len = strnlen (buf, 4);
		  if (len < 4)
		    goto zero_byte_exist;
		}
	    }
	  base += 4;
	}
    }

zero_byte_exist:
  /* Update the base address register.  */
  if (__TEST (insn, 2))
    CCPU_GPR[ra].u += reg_cnt * 4 * di;

  return;
}

static void
nds32_decode32_alu1 (sim_cpu *cpu, const uint32_t insn, sim_cia cia)
{
  const int rt = N32_RT5 (insn);
  const int ra = N32_RA5 (insn);
  const int rb = N32_RB5 (insn);
  const int rd = N32_RD5 (insn);
  const int imm5u = rb;
  const int sh5 = N32_SH5 (insn);

  switch (N32_SUB5 (insn))
    {
    case 0x0:			/* add, add_slli */
      CCPU_GPR[rt].u = CCPU_GPR[ra].u + (CCPU_GPR[rb].u << sh5);
      break;
    case 0x1:			/* sub, sub_slli */
      CCPU_GPR[rt].u = CCPU_GPR[ra].u - (CCPU_GPR[rb].u << sh5);
      break;
    case 0x2:			/* and, add_slli */
      CCPU_GPR[rt].u = CCPU_GPR[ra].u & (CCPU_GPR[rb].u << sh5);
      break;
    case 0x3:			/* xor, xor_slli */
      CCPU_GPR[rt].u = CCPU_GPR[ra].u ^ (CCPU_GPR[rb].u << sh5);
      break;
    case 0x4:			/* or, or_slli */
      CCPU_GPR[rt].u = CCPU_GPR[ra].u | (CCPU_GPR[rb].u << sh5);
      break;
    case 0x5:			/* nor */
      CCPU_GPR[rt].u = ~(CCPU_GPR[ra].u | CCPU_GPR[rb].u);
      break;
    case 0x6:			/* slt */
      CCPU_GPR[rt].u = CCPU_GPR[ra].u < CCPU_GPR[rb].u ? 1 : 0;
      break;
    case 0x7:			/* slts */
      CCPU_GPR[rt].u = CCPU_GPR[ra].s < CCPU_GPR[rb].s ? 1 : 0;
      break;

    case 0x8:			/* slli */
      CCPU_GPR[rt].u = CCPU_GPR[ra].u << imm5u;
      break;
    case 0x9:			/* srli */
      CCPU_GPR[rt].u = CCPU_GPR[ra].u >> imm5u;
      break;
    case 0xa:			/* srai */
      CCPU_GPR[rt].s = CCPU_GPR[ra].s >> imm5u;
      break;
    case 0xc:			/* sll */
      CCPU_GPR[rt].u = CCPU_GPR[ra].u << (CCPU_GPR[rb].u & 0x1f);
      break;
    case 0xd:			/* srl */
      CCPU_GPR[rt].u = CCPU_GPR[ra].u >> CCPU_GPR[rb].u;
      break;
    case 0xe:			/* sra */
      CCPU_GPR[rt].s = CCPU_GPR[ra].s >> CCPU_GPR[rb].u;
      break;
    case 0xb:			/* rotri */
    case 0xf:			/* rotr */
      {
	uint32_t shift = (N32_SUB5 (insn) == 0xb) ? imm5u : CCPU_GPR[rb].u;
	uint32_t m = CCPU_GPR[ra].u & __MASK (shift);
	CCPU_GPR[rt].u = CCPU_GPR[ra].u >> shift;
	CCPU_GPR[rt].u |= m << (32 - shift);
      }
      break;

    case 0x10:			/* seb */
      CCPU_GPR[rt].s = __SEXT (CCPU_GPR[ra].s, 8);
      break;
    case 0x11:			/* seh */
      CCPU_GPR[rt].s = __SEXT (CCPU_GPR[ra].s, 16);
      break;
    case 0x12:			/* bitc */
      CCPU_GPR[rt].u = CCPU_GPR[ra].u & ~(CCPU_GPR[rb].u);
      break;
    case 0x13:			/* zeh */
      CCPU_GPR[rt].u = CCPU_GPR[ra].u & 0xffff;
      break;
    case 0x14:			/* wsbh */
      CCPU_GPR[rt].u = ((CCPU_GPR[ra].u & 0xFF00FF00) >> 8)
		       | ((CCPU_GPR[ra].u & 0x00FF00FF) << 8);
      break;
    case 0x15:			/* or_srli */
      CCPU_GPR[rt].u = CCPU_GPR[ra].u | (CCPU_GPR[rb].u >> sh5);
      break;
    case 0x16:			/* divsr */
      {
	/* TODO: Generate positive qoutient exception.  */
	int64_t q = 0;
	int64_t r = 0;

	if (CCPU_GPR[rb].u != 0)
	  {
	    q = CCPU_GPR[ra].s / CCPU_GPR[rb].s;
	    r = CCPU_GPR[ra].s % CCPU_GPR[rb].s;
	  }

	CCPU_GPR[rt].s = q;
	if (rt != rd)
	  CCPU_GPR[rd].s = r;
      }
      break;
    case 0x17:			/* divr */
      {
	uint64_t q = 0;
	uint64_t r = 0;

	if (CCPU_GPR[rb].u != 0)
	  {
	    q = CCPU_GPR[ra].u / CCPU_GPR[rb].u;
	    r = CCPU_GPR[ra].u % CCPU_GPR[rb].u;
	  }

	CCPU_GPR[rt].u = q;
	if (rt != rd)
	  CCPU_GPR[rd].u = r;
      }
      break;
    case 0x18:                  /* sva */
    case 0x19:                  /* svs */
      {
	int c1, c2;
	unsigned int ua, ub;
	int sub;

	ua = CCPU_GPR[ra].u;
	ub = CCPU_GPR[rb].u;
	sub = N32_SUB5 (insn) == 0x19 ? 1 : 0;

	if (sub)
	  ub = ~ub;

	c1 = ((ua & 0x7fffffff) + (ub & 0x7fffffff) + sub) >> 31;
	c2 = ((ua >> 31) + (ub >> 31) + c1) >> 1;

	CCPU_GPR[rt].u = c1 ^ c2;
      }
      break;
    case 0x1a:			/* cmovz */
      if (CCPU_GPR[rb].u == 0)
	CCPU_GPR[rt].u = CCPU_GPR[ra].u;
      break;
    case 0x1b:			/* cmovn */
      if (CCPU_GPR[rb].u != 0)
	CCPU_GPR[rt].u = CCPU_GPR[ra].u;
      break;
    case 0x1c:			/* add_srli */
      CCPU_GPR[rt].u = CCPU_GPR[ra].u + (CCPU_GPR[rb].u >> sh5);
      break;
    case 0x1d:			/* sub_srli */
      CCPU_GPR[rt].u = CCPU_GPR[ra].u - (CCPU_GPR[rb].u >> sh5);
      break;
    case 0x1e:			/* and_srli */
      CCPU_GPR[rt].u = CCPU_GPR[ra].u & (CCPU_GPR[rb].u >> sh5);
      break;
    case 0x1f:			/* xor_srli */
      CCPU_GPR[rt].u = CCPU_GPR[ra].u ^ (CCPU_GPR[rb].u >> sh5);
      break;
    default:
      nds32_bad_op (cpu, cia, insn, "ALU1");
      return;
    }

  return;
}

static void
nds32_decode32_oneop (sim_cpu *cpu, const uint32_t insn, sim_cia cia)
{
  int rt = N32_RT5 (insn);
  int ra = N32_RA5 (insn);
  const int imm2u = (insn >> 10) & 0x3;
  int i;

  switch ((insn >> 10) & 0x1f)
    {
    case 0x0:			/* sunpkd810 */
      {
	reg_t result;
	result.b16.hi = (int16_t) CCPU_GPR[ra].b8.b1;
	result.b16.lo = (int16_t) CCPU_GPR[ra].b8.b0;
	CCPU_GPR[rt].s = result.s;
      }
      break;
    case 0x1:			/* sunpkd820 */
      {
	reg_t result;
	result.b16.hi = (int16_t) CCPU_GPR[ra].b8.b2;
	result.b16.lo = (int16_t) CCPU_GPR[ra].b8.b0;
	CCPU_GPR[rt].s = result.s;
      }
      break;
    case 0x2:			/* sunpkd830 */
      {
	reg_t result;
	result.b16.hi = (int16_t) CCPU_GPR[ra].b8.b3;
	result.b16.lo = (int16_t) CCPU_GPR[ra].b8.b0;
	CCPU_GPR[rt].s = result.s;
      }
      break;
    case 0x3:			/* sunpkd831 */
      {
	reg_t result;
	result.b16.hi = (int16_t) CCPU_GPR[ra].b8.b3;
	result.b16.lo = (int16_t) CCPU_GPR[ra].b8.b1;
	CCPU_GPR[rt].s = result.s;
      }
      break;
    case 0x4:			/* zunpkd810 */
      {
	reg_t result;
	result.ub16.hi = (uint16_t) CCPU_GPR[ra].ub8.b1;
	result.ub16.lo = (uint16_t) CCPU_GPR[ra].ub8.b0;
	CCPU_GPR[rt].u = result.u;
      }
      break;
    case 0x5:			/* zunpkd820 */
      {
	reg_t result;
	result.ub16.hi = (uint16_t) CCPU_GPR[ra].ub8.b2;
	result.ub16.lo = (uint16_t) CCPU_GPR[ra].ub8.b0;
	CCPU_GPR[rt].u = result.u;
      }
      break;
    case 0x6:			/* zunpkd830 */
      {
	reg_t result;
	result.ub16.hi = (uint16_t) CCPU_GPR[ra].ub8.b3;
	result.ub16.lo = (uint16_t) CCPU_GPR[ra].ub8.b0;
	CCPU_GPR[rt].u = result.u;
      }
      break;
    case 0x7:			/* zunpkd831 */
      {
	reg_t result;
	result.ub16.hi = (uint16_t) CCPU_GPR[ra].ub8.b3;
	result.ub16.lo = (uint16_t) CCPU_GPR[ra].ub8.b1;
	CCPU_GPR[rt].u = result.u;
      }
      break;
    case 0x8:			/* kabs16 */
      {
	reg_t result;
	int16_t *ptr;
	result.u = CCPU_GPR[ra].u;
	for (i = 0; i < 2; i++)
	  {
	    ptr = (int16_t *) &result.b16 + i;
	    if ((*ptr) == -0x8000)
	      {
		*ptr = 0x7fff;
		CCPU_SR_SET (PSW, PSW_OV);
	      }
	    else if (*ptr & 0x8000)
	      *ptr = -(*ptr);
	  }
	CCPU_GPR[rt].u = result.u;
      }
      break;
    case 0xc:			/* kabs8 */
      {
	CCPU_GPR[rt].b8.b0 = get_abs (cpu, CCPU_GPR[ra].b8.b0, 7);
	CCPU_GPR[rt].b8.b1 = get_abs (cpu, CCPU_GPR[ra].b8.b1, 7);
	CCPU_GPR[rt].b8.b2 = get_abs (cpu, CCPU_GPR[ra].b8.b2, 7);
	CCPU_GPR[rt].b8.b3 = get_abs (cpu, CCPU_GPR[ra].b8.b3, 7);
      }
      break;
    case 0x10:			/* insb */
    case 0x11:
    case 0x12:
    case 0x13:
      {
	int temp = CCPU_GPR[ra].s;

	/* default is byte 0 */
	int32_t mask = 0xFFFFFF00;

	if (imm2u == 1)
	  {
	    mask = 0xFFFF00FF;
	    temp <<= 8;
	  }
	else if (imm2u == 2)
	  {
	    mask = 0xFF00FFFF;
	    temp <<= 16;
	  }
	else if (imm2u == 3)
	  {
	    mask = 0x00FFFFFF;
	    temp <<= 24;
	  }

	CCPU_GPR[rt].s = (CCPU_GPR[rt].s & mask) | (temp & (~mask));
      }
      break;
    default:
      nds32_bad_op (cpu, cia, insn, "ALU2");
      return;
    }

  return;
}

static void
nds32_decode32_kmxy (sim_cpu *cpu, const uint32_t insn, sim_cia cia)
{
  int rt = N32_RT5 (insn);
  int ra = N32_RA5 (insn);
  int rb = N32_RB5 (insn);

  switch ((insn >> 6) & 0xf)
    {
    case 0x0:		/* kdmbb */
      {
	int16_t aop = CCPU_GPR[ra].b16.lo;
	int16_t bop = CCPU_GPR[rb].b16.lo;
	int32_t mul = (int32_t) aop * bop;
	int32_t res = mul << 1;

	if (mul != (res >> 1))
	  {
	    res = 0x7fffffff;
	    CCPU_SR_SET (PSW, PSW_OV);
	  }

	CCPU_GPR[rt].s = res;
      }
      break;
    case 0x1:		/* kdmbt */
      {
	int16_t aop = CCPU_GPR[ra].b16.lo;
	int16_t bop = CCPU_GPR[rb].b16.hi;
	int32_t mul = (int32_t) aop * bop;
	int32_t res = mul << 1;

	if (mul != (res >> 1))
	  {
	    res = 0x7fffffff;
	    CCPU_SR_SET (PSW, PSW_OV);
	  }

	CCPU_GPR[rt].s = res;
      }
      break;
    case 0x2:		/* kdmtb */
      {
	int16_t aop = CCPU_GPR[ra].b16.hi;
	int16_t bop = CCPU_GPR[rb].b16.lo;
	int32_t mul = (int32_t) aop * bop;
	int32_t res = mul << 1;

	if (mul != (res >> 1))
	  {
	    res = 0x7fffffff;
	    CCPU_SR_SET (PSW, PSW_OV);
	  }

	CCPU_GPR[rt].s = res;
      }
      break;
    case 0x3:		/* kdmtt */
      {
	int16_t aop = CCPU_GPR[ra].b16.hi;
	int16_t bop = CCPU_GPR[rb].b16.hi;
	int32_t mul = (int32_t) aop * bop;
	int32_t res = mul << 1;

	if (mul != (res >> 1))
	  {
	    res = 0x7fffffff;
	    CCPU_SR_SET (PSW, PSW_OV);
	  }

	CCPU_GPR[rt].s = res;
      }
      break;
    case 0x4:		/* khmbb */
      {
	int16_t aop = CCPU_GPR[ra].b16.lo;
	int16_t bop = CCPU_GPR[rb].b16.lo;
	CCPU_GPR[rt].s = insn_sat_khm_helper (cpu, aop, bop);
      }
      break;
    case 0x5:		/* khmbt */
      {
	int16_t aop = CCPU_GPR[ra].b16.lo;
	int16_t bop = CCPU_GPR[rb].b16.hi;
	CCPU_GPR[rt].s = insn_sat_khm_helper (cpu, aop, bop);
      }
      break;
    case 0x6:		/* khmtb */
      {
	int16_t aop = CCPU_GPR[ra].b16.hi;
	int16_t bop = CCPU_GPR[rb].b16.lo;
	CCPU_GPR[rt].s = insn_sat_khm_helper (cpu, aop, bop);
      }
      break;
    case 0x7:		/* khmtt */
      {
	int16_t aop = CCPU_GPR[ra].b16.hi;
	int16_t bop = CCPU_GPR[rb].b16.hi;
	CCPU_GPR[rt].s = insn_sat_khm_helper (cpu, aop, bop);
      }
      break;
    case 0x8:		/* smul16 */
      {
	/* Rt[31:16] = Ra[31:16] * Rb[31:16]
	   Rt[15:0] = Ra[15:0] * Rb[15:0] */
	int64_t result = ((int64_t) (CCPU_GPR[ra].b16.hi
				     * CCPU_GPR[rb].b16.hi) << 32)
			  | ((int64_t) (CCPU_GPR[ra].b16.lo
					* CCPU_GPR[rb].b16.lo) & 0xFFFFFFFF);
	set_double (cpu, rt, result);
      }
      break;
    case 0x9:		/* smulx16 */
      {
	/* Rt[31:16] = Ra[31:16] * Rb[15:0]
	   Rt[15:0] = Ra[15:0] * Rb[31:16] */
	int64_t result = ((int64_t) (CCPU_GPR[ra].b16.hi
				     * CCPU_GPR[rb].b16.lo) << 32)
			  | ((int64_t) (CCPU_GPR[ra].b16.lo
					* CCPU_GPR[rb].b16.hi) & 0xFFFFFFFF);
	set_double (cpu, rt, result);
      }
      break;
    case 0xa:		/* umul16 */
      {
	/* Rt[31:16] = Ra[31:16] * Rb[31:16]
	   Rt[15:0] = Ra[15:0] * Rb[15:0] */
	uint64_t result = ((uint64_t) (CCPU_GPR[ra].ub16.hi
				       * CCPU_GPR[rb].ub16.hi) << 32)
			   | ((uint64_t) (CCPU_GPR[ra].ub16.lo
					  * CCPU_GPR[rb].ub16.lo) & 0xFFFFFFFF);
	set_udouble (cpu, rt, result);
      }
      break;
    case 0xb:		/* umulx16 */
      {
	/* Rt[31:16] = Ra[31:16] * Rb[15:0]
	   Rt[15:0] = Ra[15:0] * Rb[31:16] */
	uint64_t result = ((uint64_t) (CCPU_GPR[ra].ub16.hi
				       * CCPU_GPR[rb].ub16.lo) << 32)
			   | ((uint64_t) (CCPU_GPR[ra].ub16.lo
					  * CCPU_GPR[rb].ub16.hi) & 0xFFFFFFFF);
	set_udouble (cpu, rt, result);
      }
      break;
    case 0xc:		/* khm16 */
      {
	int16_t aop1 = CCPU_GPR[ra].b16.lo;
	int16_t bop1 = CCPU_GPR[rb].b16.lo;
	int16_t aop2 = CCPU_GPR[ra].b16.hi;
	int16_t bop2 = CCPU_GPR[rb].b16.hi;
	CCPU_GPR[rt].b16.lo = insn_sat_khm_helper (cpu, aop1, bop1);
	CCPU_GPR[rt].b16.hi = insn_sat_khm_helper (cpu, aop2, bop2);
      }
      break;
    case 0xd:		/* khmx16 */
      {
	int16_t aop1 = CCPU_GPR[ra].b16.hi;
	int16_t bop1 = CCPU_GPR[rb].b16.lo;
	int16_t aop2 = CCPU_GPR[ra].b16.lo;
	int16_t bop2 = CCPU_GPR[rb].b16.hi;
	CCPU_GPR[rt].b16.lo = insn_sat_khm_helper (cpu, aop1, bop1);
	CCPU_GPR[rt].b16.hi = insn_sat_khm_helper (cpu, aop2, bop2);
      }
      break;
    default:
      nds32_bad_op (cpu, cia, insn, "ALU2");
      return;
    }
}

static void
nds32_decode32_alu2 (sim_cpu *cpu, const uint32_t insn, sim_cia cia)
{
  const int rt = N32_RT5 (insn);
  const int ra = N32_RA5 (insn);
  const int rb = N32_RB5 (insn);
  const int imm5u = rb;
  const int imm4u = rb & 0xf;
  const int dt = __TEST (insn, 21) ? USR0_D1LO : USR0_D0LO;
  int i;

  if ((insn & 0x7f) == 0x4e)	/* ffbi */
    {
      unsigned char buff[4];
      int order = CCPU_SR_TEST (PSW, PSW_BE) ? BIG_ENDIAN : LITTLE_ENDIAN;
      int imm8 = ((insn >> 7) & 0xff);
      unsigned char *ret;

      store_unsigned_integer (buff, 4, order, CCPU_GPR[ra].u);
      ret = memchr (buff, imm8, 4);
      if (NULL == ret)
	CCPU_GPR[rt].u = 0;
      else
	CCPU_GPR[rt].u = ret - buff - 4;
      return;
    }

  if ((insn & 0x3f) == 0x14)	/* kmxy */
    {
      nds32_decode32_kmxy (cpu, insn, cia);
      return;
    }

  if ((insn >> 8) & 0x1)
    {
      switch (insn & 0x7ff)
	{
	case 0x100:			/* kmmwb2 */
	  {
	    if ((CCPU_GPR[ra].s == 0x80000000)
		&& (CCPU_GPR[rb].b16.lo == 0x8000))
	      {
		CCPU_GPR[rt].s = 0x7FFFFFFF;
		CCPU_SR_SET (PSW, PSW_OV);
	      }
	    else
	      CCPU_GPR[rt].s
		= ((int64_t) CCPU_GPR[ra].s * (int64_t) CCPU_GPR[rb].b16.lo) >> 15;

	    return;
	  }
	case 0x101:			/* kmmwb2.u */
	  {
	    if ((CCPU_GPR[ra].s == 0x80000000)
		&& (CCPU_GPR[rb].b16.lo == 0x8000))
	      {
		CCPU_GPR[rt].s = 0x7FFFFFFF;
		CCPU_SR_SET (PSW, PSW_OV);
	      }
	    else
	      {
		int64_t result =
		  (int64_t) CCPU_GPR[ra].s * (int64_t) CCPU_GPR[rb].b16.lo;
		int32_t round_up = (result >> 14) & 0x1;

		/* Round up */
		if (round_up != 0)
		  CCPU_GPR[rt].s = (result >> 15) + 1;
		else
		  CCPU_GPR[rt].s = result >> 15;
	      }
	    return;
	  }
	case 0x102:			/* kmmwt2 */
	  {
	    if ((CCPU_GPR[ra].s == 0x80000000)
		&& (CCPU_GPR[rb].b16.hi == 0x8000))
	      {
		CCPU_GPR[rt].s = 0x7FFFFFFF;
		CCPU_SR_SET (PSW, PSW_OV);
	      }
	    else
	      CCPU_GPR[rt].s
		= ((int64_t) CCPU_GPR[ra].s * (int64_t) CCPU_GPR[rb].b16.hi) >> 15;

	    return;
	  }
	case 0x103:			/* kmmwb2.u */
	  {
	    if ((CCPU_GPR[ra].s == 0x80000000)
		&& (CCPU_GPR[rb].b16.hi == 0x8000))
	      {
		CCPU_GPR[rt].s = 0x7FFFFFFF;
		CCPU_SR_SET (PSW, PSW_OV);
	      }
	    else
	      {
		int64_t result =
		(int64_t) CCPU_GPR[ra].s * (int64_t) CCPU_GPR[rb].b16.hi;
		int32_t round_up = (result >> 14) & 0x1;

		/* Round up */
		if (round_up != 0)
		  CCPU_GPR[rt].s = (result >> 15) + 1;
		else
		  CCPU_GPR[rt].s = result >> 15;
	      }
	    return;
	  }
	case 0x104:			/* kmmawb2 */
	  {
	    union64_t temp;
	    int64_t res;
	    int32_t addop;
	    if ((CCPU_GPR[ra].s == 0x80000000)
		&& (CCPU_GPR[rb].b16.lo == 0x8000))
	      {
		addop = 0x7FFFFFFF;
		CCPU_SR_SET (PSW, PSW_OV);
	      }
	    else
	      {
		temp.d0 = (int64_t) CCPU_GPR[ra].s * CCPU_GPR[rb].b16.lo;
		addop = (int32_t) (temp.d0 >> 15);
	      }

	    res = (int64_t) CCPU_GPR[rt].s + addop;
	    res = insn_sat_helper (cpu, res, 31);
	    CCPU_GPR[rt].s = res;
	    return;
	  }
	case 0x105:			/* kmmawb2.u */
	  {
	    union64_t temp;
	    int64_t res;
	    int32_t rnd_val, addop;

	    if ((CCPU_GPR[ra].s == 0x80000000)
		&& (CCPU_GPR[rb].b16.lo == 0x8000))
	      {
		addop = 0x7FFFFFFF;
		CCPU_SR_SET (PSW, PSW_OV);
	      }
	    else
	      {
		temp.d0 = (int64_t) CCPU_GPR[ra].s * CCPU_GPR[rb].b16.lo;
		temp.d0 = (temp.d0 >> 14) + 1;
		addop = (int32_t) (temp.d0 >> 15);
	      }

	    res = (int64_t) CCPU_GPR[rt].s + addop;
	    res = insn_sat_helper (cpu, res, 31);
	    CCPU_GPR[rt].s = res;
	    return;
	  }
	case 0x106:			/* kmmawt2 */
	  {
	    union64_t temp;
	    int64_t res;
	    int32_t addop;
	    if ((CCPU_GPR[ra].s == 0x80000000)
		&& (CCPU_GPR[rb].b16.hi == 0x8000))
	      {
		addop = 0x7FFFFFFF;
		CCPU_SR_SET (PSW, PSW_OV);
	      }
	    else
	      {
		temp.d0 = (int64_t) CCPU_GPR[ra].s * CCPU_GPR[rb].b16.hi;
		addop = (int32_t) (temp.d0 >> 15);
	      }

	    res = (int64_t) CCPU_GPR[rt].s + addop;
	    res = insn_sat_helper (cpu, res, 31);
	    CCPU_GPR[rt].s = res;
	    return;
	  }
	case 0x107:			/* kmmawt2.u */
	  {
	    union64_t temp;
	    int64_t res;
	    int32_t rnd_val, addop;

	    if ((CCPU_GPR[ra].s == 0x80000000)
		&& (CCPU_GPR[rb].b16.hi == 0x8000))
	      {
		addop = 0x7FFFFFFF;
		CCPU_SR_SET (PSW, PSW_OV);
	      }
	    else
	      {
		temp.d0 = (int64_t) CCPU_GPR[ra].s * CCPU_GPR[rb].b16.hi;
		temp.d0 = (temp.d0 >> 14) + 1;
		addop = (int32_t) (temp.d0 >> 15);
	      }

	    res = (int64_t) CCPU_GPR[rt].s + addop;
	    res = insn_sat_helper (cpu, res, 31);
	    CCPU_GPR[rt].s = res;
	    return;
	  }
	}
    }

  switch (insn & 0x3ff)
    {
    case 0x0:			/* max */
      CCPU_GPR[rt].s = (CCPU_GPR[ra].s > CCPU_GPR[rb].s)
		       ? CCPU_GPR[ra].s : CCPU_GPR[rb].s;
      break;
    case 0x1:			/* min */
      CCPU_GPR[rt].s = (CCPU_GPR[ra].s < CCPU_GPR[rb].s)
		       ? CCPU_GPR[ra].s : CCPU_GPR[rb].s;
      break;
    case 0x2:			/* ave */
      {
	int64_t r = ((int64_t) CCPU_GPR[ra].s)
		    + ((int64_t) CCPU_GPR[rb].s) + 1;
	CCPU_GPR[rt].u = (r >> 1);
      }
      break;
    case 0x3:			/* abs */
      if (CCPU_GPR[ra].s >= 0)
	CCPU_GPR[rt].s = CCPU_GPR[ra].s;
      else if (CCPU_GPR[ra].u == 0x80000000)
	CCPU_GPR[rt].u = 0x7fffffff;
      else
	CCPU_GPR[rt].s = -CCPU_GPR[ra].s;
      break;
    case 0x4:			/* clips */
      if (CCPU_GPR[ra].s > ((1 << imm5u) - 1))
	CCPU_GPR[rt].s = ((1 << imm5u) - 1);
      else if (CCPU_GPR[ra].s < -(1 << imm5u))
	CCPU_GPR[rt].s = -(1 << imm5u);
      else
	CCPU_GPR[rt].s = CCPU_GPR[ra].s;
      break;
    case 0x5:			/* clip */
      if (CCPU_GPR[ra].s > ((1 << imm5u) - 1))
	CCPU_GPR[rt].s = ((1 << imm5u) - 1);
      else if (CCPU_GPR[ra].s < 0)
	CCPU_GPR[rt].s = 0;
      else
	CCPU_GPR[rt].s = CCPU_GPR[ra].s;
      break;
    case 0x6:			/* clo */
      {
	int i, cnt = 0;

	for (i = 31; i >= 0; i--)
	  {
	    if (__TEST (CCPU_GPR[ra].u, i))
	      cnt++;
	    else
	      break;
	  }
	CCPU_GPR[rt].u = cnt;
      }
      break;
    case 0x7:			/* clz */
      {
	int i, cnt = 0;

	for (i = 31; i >= 0; i--)
	  {
	    if (__TEST (CCPU_GPR[ra].u, i) == 0)
	      cnt++;
	    else
	      break;
	  }
	CCPU_GPR[rt].u = cnt;
      }
      break;
    case 0x8:			/* bset */
      CCPU_GPR[rt].u = CCPU_GPR[ra].u | __BIT (imm5u);
      break;
    case 0x9:			/* bclr */
      CCPU_GPR[rt].u = CCPU_GPR[ra].u & ~__BIT (imm5u);
      break;
    case 0xa:			/* btgl */
      CCPU_GPR[rt].u = CCPU_GPR[ra].u ^ __BIT (imm5u);
      break;
    case 0xb:			/* btst */
      CCPU_GPR[rt].u = __TEST (CCPU_GPR[ra].u, imm5u) ? 1 : 0;
      break;
    case 0xc:			/* bse */
      {
	int n = __GF (CCPU_GPR[rb].u, 0, 5);
	int m = __GF (CCPU_GPR[rb].u, 8, 5);
	int underflow = __TEST (CCPU_GPR[rb].u, 30);
	int refill = __TEST (CCPU_GPR[rb].u, 31);
	int len = m + 1;
	int dist = 32 - len - n;	/* From LSB.  */
	int val;
	int d = n + m;
	uint32_t ora = CCPU_GPR[ra].u;

	/* Clear non-occupied.  */
	if (!underflow)
	  CCPU_GPR[rt].u = __GF (CCPU_GPR[rt].u, 0, len);

	/* Normal condition.  */
	if (31 > d)
	  {
	    __put_field (&CCPU_GPR[rb].u, 0, 5, d + 1);
	    val = __GF (ora, dist, len);

	    __put_field (&CCPU_GPR[rt].u, 0, len, val);

	    if (underflow)
	      {
		/* Restore old length.  */
		__put_field (&CCPU_GPR[rb].u, 8, 5, __GF (CCPU_GPR[rb].u, 16, 5));
		/* Why?  */
		__put_field (&CCPU_GPR[rb].u, 13, 3, 0);
	      }

	    CCPU_GPR[rb].u &= ~__BIT (30);
	    CCPU_GPR[rb].u &= ~__BIT (31);
	  }
	/* Empty condition.  */
	else if (31 == d)
	  {
	    CCPU_GPR[rb].u &= ~0x1f;
	    val = __GF (ora, dist, len);

	    __put_field (&CCPU_GPR[rt].u, 0, len, val);

	    CCPU_GPR[rb].u &= ~__BIT (30);
	    CCPU_GPR[rb].u |= __BIT (31);
	  }
	/* Undeflow condition.  */
	else /* 31 < d */
	  {
	    __put_field (&CCPU_GPR[rb].u, 16, 5, m);
	    __put_field (&CCPU_GPR[rb].u, 8, 5, d - 32);
	    CCPU_GPR[rb].u &= ~0x1f;
	    CCPU_GPR[rb].u |= __BIT (30);
	    CCPU_GPR[rb].u |= __BIT (31);
	    val = __GF (ora, 0, 32 - n);
	    __put_field (&CCPU_GPR[rt].u, 0, len, val << (d - 31));
	  }
      }
      break;
    case 0xd:			/* bsp */
      {
	int n = __GF (CCPU_GPR[rb].u, 0, 5);
	int m = __GF (CCPU_GPR[rb].u, 8, 5);
	int underflow = __TEST (CCPU_GPR[rb].u, 30);
	int refill = __TEST (CCPU_GPR[rb].u, 31);
	int len = m + 1;
	int dist = 32 - len - n;	/* From LSB.  */
	int val;
	int d = n + m;
	uint32_t ora = CCPU_GPR[ra].u;

	/* Normal condition.  */
	if (31 > d)
	  {
	    __put_field (&CCPU_GPR[rb].u, 0, 5, d + 1);
	    val = __GF (ora, 0, len);

	    __put_field (&CCPU_GPR[rt].u, dist, len, val);

	    if (underflow)
	      {
		/* Restore old length.  */
		__put_field (&CCPU_GPR[rb].u, 8, 5, __GF (CCPU_GPR[rb].u, 16, 5));
		/* Why?  */
		__put_field (&CCPU_GPR[rb].u, 13, 3, 0);
	      }

	    CCPU_GPR[rb].u &= ~__BIT (30);
	    CCPU_GPR[rb].u &= ~__BIT (31);
	  }
	/* Empty condition.  */
	else if (31 == d)
	  {
	    CCPU_GPR[rb].u &= ~0x1f;
	    val = __GF (ora, 0, len);

	    __put_field (&CCPU_GPR[rt].u, dist, len, val);

	    CCPU_GPR[rb].u &= ~__BIT (30);
	    CCPU_GPR[rb].u |= __BIT (31);
	  }
	/* Undeflow condition.  */
	else /* 31 < d */
	  {
	    __put_field (&CCPU_GPR[rb].u, 16, 5, m);
	    __put_field (&CCPU_GPR[rb].u, 8, 5, d - 32);
	    CCPU_GPR[rb].u &= ~0x1f;
	    CCPU_GPR[rb].u |= __BIT (30);
	    CCPU_GPR[rb].u |= __BIT (31);
	    val = __GF (ora, 0, len) >> (d - 31);
	    __put_field (&CCPU_GPR[rt].u, 0, 32 - n, val);
	  }
      }
      break;
    case 0xe:			/* ffb */
      {
	char buff[4];
	int order = CCPU_SR_TEST (PSW, PSW_BE) ? BIG_ENDIAN : LITTLE_ENDIAN;
	void *ret;

	store_unsigned_integer ((unsigned char *) &buff, 4, order, CCPU_GPR[ra].u);
	ret = memchr (buff, CCPU_GPR[rb].u, 4);
	if (NULL == ret)
	  CCPU_GPR[rt].u = 0;
	else
	  CCPU_GPR[rt].u = (char *) ret - (char *) buff - 4;
      }
      break;
    case 0xf:			/* ffmism */
      {
	char a[4];
	char b[4];
	int order = CCPU_SR_TEST (PSW, PSW_BE) ? BIG_ENDIAN : LITTLE_ENDIAN;
	int ret;

	store_unsigned_integer ((unsigned char *) &a, 4, order, CCPU_GPR[ra].u);
	store_unsigned_integer ((unsigned char *) &b, 4, order, CCPU_GPR[rb].u);
	ret = find_mism ((unsigned char *) &a, (unsigned char *) &b, 1);
	CCPU_GPR[rt].u = ret;
      }
      break;
    case 0x17:			/* ffzmism */
      {
	char a[4];
	char b[4];
	int order = CCPU_SR_TEST (PSW, PSW_BE) ? BIG_ENDIAN : LITTLE_ENDIAN;
	int ret;

	store_unsigned_integer ((unsigned char *) &a, 4, order, CCPU_GPR[ra].u);
	store_unsigned_integer ((unsigned char *) &b, 4, order, CCPU_GPR[rb].u);
	ret = find_null_mism ((unsigned char *) &a, (unsigned char *) &b);
	CCPU_GPR[rt].u = ret;
      }
      break;
    case 0x18:			/* kaddw */
      {
	int64_t tmp = (int64_t) CCPU_GPR[ra].s + (int64_t) CCPU_GPR[rb].s;
	CCPU_GPR[rt].s = insn_sat_helper (cpu, tmp, 31);
      }
      break;
    case 0x19:			/* ksubw */
      {
	int64_t tmp = (int64_t) CCPU_GPR[ra].s - (int64_t) CCPU_GPR[rb].s;
	CCPU_GPR[rt].s = insn_sat_helper (cpu, tmp, 31);
      }
      break;
    case 0x1a:			/* kslraw */
      {
	if (CCPU_GPR[rb].b8.b0 < 0)
	  {
	    int sh = -CCPU_GPR[rb].b8.b0;
	    sh = sh > 31 ? 31 : sh;
	    CCPU_GPR[rt].s = CCPU_GPR[ra].s >> sh;
	  }
	else
	  {
	    int64_t ret = (int64_t) CCPU_GPR[ra].s << CCPU_GPR[rb].b8.b0;
	    CCPU_GPR[rt].s = insn_sat_helper (cpu, ret, 31);
	  }
      }
      break;
    case 0x1b:			/* kslraw.u */
      {
	int32_t ret;
	int sh = CCPU_GPR[rb].b8.b0;

	if (CCPU_GPR[rb].b8.b0 < 0)
	  {
	    int rnd;
	    uint32_t mask_sh;
	    sh = -CCPU_GPR[rb].b8.b0;
	    sh = sh > 31 ? 31 : sh;
	    mask_sh = (1UL << (sh - 1));
	    rnd = (CCPU_GPR[ra].s & mask_sh) ? 1 : 0;
	    ret = CCPU_GPR[ra].s >> sh;
	    ret += rnd;
	  }
	else
	  {
	    int64_t tmp;
            sh = CCPU_GPR[rb].b8.b0 > 31 ? 31 : sh;
	    tmp = (int64_t) CCPU_GPR[ra].s << sh;
	    ret = insn_sat_helper (cpu, tmp, 31);
	  }
	CCPU_GPR[rt].s = ret;
      }
      break;
    case 0x24:			/* mul */
      CCPU_GPR[rt].u = CCPU_GPR[ra].u * CCPU_GPR[rb].u;
      break;
    case 0x20:			/* mfusr */
      CCPU_GPR[rt].u = CCPU_USR[rb << 5 | ra].u;
      if (((rb << 5) | ra) == 31)	/* PC */
	CCPU_GPR[rt].u = cia;
      break;
    case 0x21:			/* mtusr */
      CCPU_USR[(rb << 5) | ra].u = CCPU_GPR[rt].u;
      break;
    case 0x28:			/* mults64 */
      {
	int64_t d = (int64_t) CCPU_GPR[ra].s * (int64_t) CCPU_GPR[rb].s;

	CCPU_USR[dt].s = d;
	CCPU_USR[dt + 1].s = (d >> 32);
      }
      break;
    case 0x29:			/* mult64 */
      {
	uint64_t d = (uint64_t) CCPU_GPR[ra].u * (uint64_t) CCPU_GPR[rb].u;

	CCPU_USR[dt].u = d;
	CCPU_USR[dt + 1].u = (d >> 32);
      }
      break;
    case 0x2a:			/* madds64 */
      {
	int64_t mr = (int64_t) CCPU_GPR[ra].s * (int64_t) CCPU_GPR[rb].s;
	int64_t d = ((int64_t) CCPU_USR[dt + 1].s << 32)
		    | ((int64_t) CCPU_USR[dt].s & 0xFFFFFFFF);

	d += mr;
	CCPU_USR[dt].u = d;
	CCPU_USR[dt + 1].u = (d >> 32);
      }
      break;
    case 0x2b:			/* madd64 */
      {
	uint64_t mr = (uint64_t) CCPU_GPR[ra].u * (uint64_t) CCPU_GPR[rb].u;
	uint64_t d = ((uint64_t) CCPU_USR[dt + 1].u << 32)
		     | ((uint64_t) CCPU_USR[dt].u & 0xFFFFFFFF);

	d += mr;
	CCPU_USR[dt].u = d;
	CCPU_USR[dt + 1].u = (d >> 32);
      }
      break;
    case 0x2c:			/* msubs64 */
      {
	int64_t mr = (int64_t) CCPU_GPR[ra].s * (int64_t) CCPU_GPR[rb].s;
	int64_t d = ((int64_t) CCPU_USR[dt + 1].s << 32)
		    | ((int64_t) CCPU_USR[dt].s & 0xFFFFFFFF);

	d -= mr;
	CCPU_USR[dt].u = d;
	CCPU_USR[dt + 1].u = (d >> 32);
      }
      break;
    case 0x2d:			/* msub64 */
      {
	uint64_t mr = (uint64_t) CCPU_GPR[ra].u * (uint64_t) CCPU_GPR[rb].u;
	uint64_t d = ((uint64_t) CCPU_USR[dt + 1].u << 32)
		     | ((uint64_t) CCPU_USR[dt].u & 0xFFFFFFFF);

	d -= mr;
	CCPU_USR[dt].u = d;
	CCPU_USR[dt + 1].u = (d >> 32);
      }
      break;
    case 0x2e:			/* divs */
      {
	int32_t q;
	int32_t r;

	q = CCPU_GPR[ra].s / CCPU_GPR[rb].s;
	r = CCPU_GPR[ra].s % CCPU_GPR[rb].s;
	CCPU_USR[dt].s = q;
	CCPU_USR[dt + 1].s = r;
      }
      break;
    case 0x2f:			/* div */
      {
	uint32_t q;
	uint32_t r;

	q = CCPU_GPR[ra].u / CCPU_GPR[rb].u;
	r = CCPU_GPR[ra].u % CCPU_GPR[rb].u;
	CCPU_USR[dt].u = q;
	CCPU_USR[dt + 1].u = r;
      }
      break;
    case 0x30:			/* add64 */
      {
	int64_t result = get_double (cpu, ra) + get_double (cpu, rb);
	set_double (cpu, rt, result);
      }
      break;
    case 0x31:			/* mult32 */
      CCPU_USR[dt].s = CCPU_GPR[ra].s * CCPU_GPR[rb].s;
      break;
    case 0x32:			/* smal */
      {
	/* Rt[63:0] = Ra[63:0] + Rb[31:16] * Rb[15:0] */
	int64_t mul_rb = CCPU_GPR[rb].b16.hi * CCPU_GPR[rb].b16.lo;
	int64_t result = get_double (cpu, ra) + mul_rb;
	set_double (cpu, rt, result);
      }
      break;
    case 0x33:			/* madd32 */
      CCPU_USR[dt].s += CCPU_GPR[ra].s * CCPU_GPR[rb].s;
      break;
    case 0x34:			/* sub64 */
      {
	int64_t result = get_double (cpu, ra) - get_double (cpu, rb);
	set_double (cpu, rt, result);
      }
      break;
    case 0x35:			/* msub32 */
      CCPU_USR[dt].s -= CCPU_GPR[ra].s * CCPU_GPR[rb].s;
      break;
    case 0x38:			/* radd64 */
      {
	/* 64 = (64 + 64) >> 1 */
	int64_t result;
	int64_t lsb_eq_1 = 1L;
	lsb_eq_1 &= (int64_t) CCPU_GPR[ra].s;
	lsb_eq_1 &= (int64_t) CCPU_GPR[rb].s;
	result = (get_double (cpu, ra) >> 1)
		 + (get_double (cpu, rb) >> 1)
		 + lsb_eq_1;

	set_double (cpu, rt, result);
      }
      break;
    case 0x39:			/* uradd64 */
      {
	/* 64 = (U64 + U64) >> 1 */
	uint64_t result;
	uint64_t lsb_eq_1 = 1UL;
	lsb_eq_1 &= (uint64_t) CCPU_GPR[ra].u;
	lsb_eq_1 &= (uint64_t) CCPU_GPR[rb].u;
	result = (get_udouble (cpu, ra) >> 1)
		 + (get_udouble (cpu, rb) >> 1)
		 + lsb_eq_1;

	set_udouble (cpu, rt, result);
      }
      break;
    case 0x3a:			/* kadd64 */
      {
	int64_t x = get_double (cpu, ra);
	int64_t y = get_double (cpu, rb);
	int64_t res = x + y;
	if ((x > 0) && (y > 0))
	  {
	    if (res <= 0)
	      {
		res = 0x7fffffffffffffffLL;
		CCPU_SR_SET (PSW, PSW_OV);
	      }
	  }
	else if ((x < 0) && (y < 0))
	  {
	    if (res >= 0)
	      {
		res = 0x8000000000000000LL;
		CCPU_SR_SET (PSW, PSW_OV);
	      }
	  }
	set_double (cpu, rt, res);
      }
      break;
    case 0x3b:			/* ukadd64 */
      {
	uint64_t x = get_udouble (cpu, ra);
	uint64_t y = get_udouble (cpu, rb);
	uint64_t res = x + y;
	if (res < x)
	  {
	    res = 0xffffffffffffffffULL;
	    CCPU_SR_SET (PSW, PSW_OV);
	  }
	set_double (cpu, rt, res);
      }
      break;
    case 0x3c:			/* rsub64 */
      {
	/* 64 = (64 - 64) >> 1 */
	int64_t result;
	int64_t lsb_ra, lsb_rb, signed_ra, signed_rb, sum_lsb;

	if (CCPU_SR_TEST (PSW, PSW_BE))
	  {
	    lsb_ra = CCPU_GPR[ra + 1].s & 0x1;
	    lsb_rb = CCPU_GPR[rb + 1].s & 0x1;
	    signed_ra = (CCPU_GPR[ra].s >> 31) & 0x1;
	    signed_rb = (CCPU_GPR[ra].s >> 31) & 0x1;
	  }
	else
	  {
	    lsb_ra = CCPU_GPR[ra].s & 0x1;
	    lsb_rb = CCPU_GPR[rb].s & 0x1;
	    signed_ra = (CCPU_GPR[ra + 1].s >> 31) & 0x1;
	    signed_rb = (CCPU_GPR[ra + 1].s >> 31) & 0x1;
	  }

	if (lsb_ra == 1 && signed_ra)
	  lsb_ra = -1L;
	if (lsb_rb == 1 && signed_rb)
	  lsb_rb = -1L;

	if (lsb_ra == -1L && lsb_rb == 0)
	  sum_lsb = 0;
	else
	  sum_lsb = (lsb_ra - lsb_rb) >> 1;

	result = (get_double (cpu, ra) >> 1)
		  - (get_double (cpu, rb) >> 1) + sum_lsb;
	set_double (cpu, rt, result);
      }
      break;
    case 0x3d:			/* ursub64 */
      {
	/* 64 = (U64 - U64) >> 1 */
	uint64_t result;
	uint64_t lsb_ra, lsb_rb, sum_lsb;

	if (CCPU_SR_TEST (PSW, PSW_BE))
	  {
	    lsb_ra = CCPU_GPR[ra + 1].u & 0x1;
	    lsb_rb = CCPU_GPR[rb + 1].u & 0x1;
	  }
	else
	  {
	    lsb_ra = CCPU_GPR[ra].u & 0x1;
	    lsb_rb = CCPU_GPR[rb].u & 0x1;
	  }

	sum_lsb = (lsb_ra < lsb_rb) ? -1ULL : 0;
	result = (get_udouble (cpu, ra) >> 1)
		  - (get_udouble (cpu, rb) >> 1) + sum_lsb;
	set_udouble (cpu, rt, result);
      }
      break;
    case 0x3e:			/* ksub64 */
      {
	int64_t x = get_double (cpu, ra);
	int64_t y = get_double (cpu, rb);
	int64_t res = x - y;
	if ((x > 0) && (y < 0))
	  {
	    if (res <= 0)
	      {
		res = 0x7fffffffffffffffLL;
		CCPU_SR_SET (PSW, PSW_OV);
	      }
	  }
	else if ((x < 0) && (y > 0))
	  {
	    if (res >= 0)
	      {
		res = 0x8000000000000000LL;
		CCPU_SR_SET (PSW, PSW_OV);
	      }
	  }
	set_double (cpu, rt, res);
      }
      break;
    case 0x3f:			/* uksub64 */
      {
	uint64_t x = get_udouble (cpu, ra);
	uint64_t y = get_udouble (cpu, rb);
	uint64_t res = x - y;
	if (x < y)
	  {
	    res = 0ULL;
	    CCPU_SR_SET (PSW, PSW_OV);
	  }
	set_double (cpu, rt, res);
      }
      break;
    case 0x40:			/* smar64 */
      {
	int64_t result = get_double (cpu, rt)
			 + ((int64_t)CCPU_GPR[ra].s
			    * (int64_t)CCPU_GPR[rb].s);
	set_double (cpu, rt, result);
      }
      break;
    case 0x41:			/* umar64 */
      {
	uint64_t result = get_udouble (cpu, rt)
			  + ((uint64_t)CCPU_GPR[ra].u
			     * (uint64_t)CCPU_GPR[rb].u);
	set_udouble (cpu, rt, result);
      }
      break;
    case 0x42:			/* smsr64 */
      {
	int64_t result = get_double (cpu, rt)
			 - ((int64_t)CCPU_GPR[ra].s
			    * (int64_t)CCPU_GPR[rb].s);
	set_double (cpu, rt, result);
      }
      break;
    case 0x43:			/* umsr64 */
      {
	uint64_t result = get_udouble (cpu, rt)
			  - ((uint64_t)CCPU_GPR[ra].u
			     * (uint64_t)CCPU_GPR[rb].u);
	set_udouble (cpu, rt, result);
      }
      break;
    case 0x44:			/* kmar64 */
      {
	int64_t acc = get_double (cpu, rt);
	int64_t mul_val = (int64_t) CCPU_GPR[ra].s * CCPU_GPR[rb].s;
	int64_t res = acc + mul_val;

	if ((acc > 0) && (mul_val > 0))
	  {
	    if (res <= 0)
	      {
		res = 0x7fffffffffffffffLL;
		CCPU_SR_SET (PSW, PSW_OV);
	      }
	  }
	else if ((acc < 0) && (mul_val < 0))
	  {
	    if (res >= 0)
	      {
		res = 0x8000000000000000LL;
		CCPU_SR_SET (PSW, PSW_OV);
	      }
	  }
	set_double (cpu, rt, res);
      }
      break;
    case 0x45:			/* ukmar64 */
      {
	uint64_t acc = get_udouble (cpu, rt);
	uint64_t mul_val = (uint64_t) CCPU_GPR[ra].u * CCPU_GPR[rb].u;
	uint64_t res = acc + mul_val;
	if (res < acc)
	  {
	    res = 0xffffffffffffffffULL;
	    CCPU_SR_SET (PSW, PSW_OV);
	  }
	set_double (cpu, rt, res);
      }
      break;
    case 0x46:			/* kmsr64 */
      {
	int64_t acc = get_double (cpu, rt);
	int64_t mul_val = (int64_t) CCPU_GPR[ra].s * CCPU_GPR[rb].s;
	int64_t res = acc - mul_val;
	if ((acc > 0) && (mul_val < 0))
	  {
	    if (res <= 0)
	      {
		res = 0x7fffffffffffffffLL;
		CCPU_SR_SET (PSW, PSW_OV);
	      }
	  }
	else if ((acc < 0) && (mul_val > 0))
	  {
	    if (res >= 0)
	      {
		res = 0x8000000000000000LL;
		CCPU_SR_SET (PSW, PSW_OV);
	      }
	  }
	set_double (cpu, rt, res);
      }
      break;
    case 0x47:			/* ukmsr64 */
      {
	uint64_t acc = get_udouble (cpu, rt);
	uint64_t mul_val = (uint64_t) CCPU_GPR[ra].u * CCPU_GPR[rb].u;
	uint64_t res = acc - mul_val;
	if (acc < mul_val)
	  {
	    res = 0ULL;
	    CCPU_SR_SET (PSW, PSW_OV);
	  }
	set_double (cpu, rt, res);
      }
      break;
    case 0x48:			/* smalda */
      {
	/* 64 = 64 + (Ra[31:16] * Rb[31:16]) + (Ra[15:0] * Rb[15:0]) */
	int64_t mul_hi = CCPU_GPR[ra].b16.hi * CCPU_GPR[rb].b16.hi;
	int64_t mul_lo = CCPU_GPR[ra].b16.lo * CCPU_GPR[rb].b16.lo;
	int64_t result = get_double (cpu, rt) + mul_hi + mul_lo;
	set_double (cpu, rt, result);
      }
      break;
    case 0x49:			/* smslda */
      {
	/* 64 = 64 - (Ra[31:16] * Rb[31:16]) + (Ra[15:0] * Rb[15:0]) */
	int64_t mul_hi = CCPU_GPR[ra].b16.hi * CCPU_GPR[rb].b16.hi;
	int64_t mul_lo = CCPU_GPR[ra].b16.lo * CCPU_GPR[rb].b16.lo;
	int64_t result = get_double (cpu, rt) - mul_hi - mul_lo;
	set_double (cpu, rt, result);
      }
      break;
    case 0x4a:			/* smalds */
      {
	/* 64 = 64 + ((Ra[31:16] * Rb[31:16]) - (Ra[15:0] * Rb[15:0])) */
	int64_t mul_hi = CCPU_GPR[ra].b16.hi * CCPU_GPR[rb].b16.hi;
	int64_t mul_lo = CCPU_GPR[ra].b16.lo * CCPU_GPR[rb].b16.lo;
	int64_t result = get_double (cpu, rt) + mul_hi - mul_lo;
	set_double (cpu, rt, result);
      }
      break;
    case 0x4b:			/* smalbb */
      {
	/* 64 = 64 + Ra[15:0] * Rb[15:0] */
	int64_t mul_lo = CCPU_GPR[ra].b16.lo * CCPU_GPR[rb].b16.lo;
	int64_t result = get_double (cpu, rt) + mul_lo;
	set_double (cpu, rt, result);
      }
      break;
    case 0x4f:			/* flmism */
      {
	char a[4];
	char b[4];
	int order = CCPU_SR_TEST (PSW, PSW_BE) ? BIG_ENDIAN : LITTLE_ENDIAN;
	int ret;

	store_unsigned_integer ((unsigned char *) &a, 4, order, CCPU_GPR[ra].u);
	store_unsigned_integer ((unsigned char *) &b, 4, order, CCPU_GPR[rb].u);
	ret = find_mism ((unsigned char *) &a, (unsigned char *) &b, -1);
	CCPU_GPR[rt].u = ret;
      }
      break;
    case 0x50:			/* smalxda */
      {
	/* 64 = 64 + (Ra[31:16] * Rb[15:0]) + (Ra[15:0] * Rb[31:16]) */
	int64_t mul_hi_lo = CCPU_GPR[ra].b16.hi * CCPU_GPR[rb].b16.lo;
	int64_t mul_lo_hi = CCPU_GPR[ra].b16.lo * CCPU_GPR[rb].b16.hi;
	int64_t result = get_double (cpu, rt) + mul_hi_lo + mul_lo_hi;
	set_double (cpu, rt, result);
      }
      break;
    case 0x51:			/* smslxda */
      {
	/* 64 = 64 - (Ra[31:16] * Rb[15:0]) + (Ra[15:0] * Rb[31:16]) */
	int64_t mul_hi_lo = CCPU_GPR[ra].b16.hi * CCPU_GPR[rb].b16.lo;
	int64_t mul_lo_hi = CCPU_GPR[ra].b16.lo * CCPU_GPR[rb].b16.hi;
	int64_t result = get_double (cpu, rt) - mul_hi_lo - mul_lo_hi;
	set_double (cpu, rt, result);
      }
      break;
    case 0x52:			/* smalxds */
      {
	/* 64 = 64 + (Ra[31:16] * Rb[15:0]) - (Ra[15:0] * Rb[31:16]) */
	int64_t mul_hi_lo = CCPU_GPR[ra].b16.hi * CCPU_GPR[rb].b16.lo;
	int64_t mul_lo_hi = CCPU_GPR[ra].b16.lo * CCPU_GPR[rb].b16.hi;
	int64_t result = get_double (cpu, rt) + (mul_hi_lo - mul_lo_hi);
	set_double (cpu, rt, result);
      }
      break;
    case 0x53:			/* smalbt */
      {
	/* 64 = 64 + Ra[15:0] * Rb[31:16] */
	int64_t mul = CCPU_GPR[ra].b16.lo * CCPU_GPR[rb].b16.hi;
	int64_t result = get_double (cpu, rt) + mul;
	set_double (cpu, rt, result);
      }
      break;
    case 0x58:			/* kaddh */
      {
	int32_t tmp = CCPU_GPR[ra].s + CCPU_GPR[rb].s;
	CCPU_GPR[rt].s = insn_sat_helper (cpu, tmp, 15);
      }
      break;
    case 0x59:			/* ksubh */
      {
	int32_t tmp = CCPU_GPR[ra].s - CCPU_GPR[rb].s;
	CCPU_GPR[rt].s = insn_sat_helper (cpu, tmp, 15);
      }
      break;
    case 0x5a:			/* smaldrs */
      {
	/* 64 = 64 + (Ra[15:0] * Rb[15:0]) - (Ra[31:16] * Rb[31:16]) */
	int64_t mul_lo = CCPU_GPR[ra].b16.lo * CCPU_GPR[rb].b16.lo;
	int64_t mul_hi = CCPU_GPR[ra].b16.hi * CCPU_GPR[rb].b16.hi;
	int64_t result = get_double (cpu, rt) + (mul_lo - mul_hi);
	set_double (cpu, rt, result);
      }
      break;
    case 0x5b:			/* smaltt */
      {
	/* 64 = 64 + Ra[31:16] * Rb[31:16] */
	int64_t mul = CCPU_GPR[ra].b16.hi * CCPU_GPR[rb].b16.hi;
	int64_t result = get_double (cpu, rt) + mul;
	set_double (cpu, rt, result);
      }
      break;
    case 0x60:			/* rdov */
      CCPU_GPR[rt].s = CCPU_SR_TEST (PSW, PSW_OV);
      break;
    case 0x61:			/* clrov */
      CCPU_SR_CLEAR (PSW, PSW_OV);
      break;
    case 0x68:			/* mulsr64 */
      {
	int64_t r = (int64_t) CCPU_GPR[ra].s * (int64_t) CCPU_GPR[rb].s;
	int d = rt & ~1;

	if (CCPU_SR_TEST (PSW, PSW_BE))
	  {
	    CCPU_GPR[d].u = (r >> 32);
	    CCPU_GPR[d + 1].u = r;
	  }
	else
	  {
	    CCPU_GPR[d + 1].u = (r >> 32);
	    CCPU_GPR[d].u = r;
	  }
      }
      break;
    case 0x69:			/* mulr64 */
      {
	uint64_t r = (uint64_t) CCPU_GPR[ra].u * (uint64_t) CCPU_GPR[rb].u;
	int d = rt & ~1;

	if (CCPU_SR_TEST (PSW, PSW_BE))
	  {
	    CCPU_GPR[d].u = (r >> 32);
	    CCPU_GPR[d + 1].u = r;
	  }
	else
	  {
	    CCPU_GPR[d + 1].u = (r >> 32);
	    CCPU_GPR[d].u = r;
	  }
      }
      break;
    case 0x70:			/* smds */
      {
	/* Rt = (Ra[31:16] * Rb[31:16]) - (Ra[15:0] * Rb[15:0]) */
	int32_t mul_hi = CCPU_GPR[ra].b16.hi * CCPU_GPR[rb].b16.hi;
	int32_t mul_lo = CCPU_GPR[ra].b16.lo * CCPU_GPR[rb].b16.lo;
	CCPU_GPR[rt].s = mul_hi - mul_lo;
      }
      break;
    case 0x71:			/* smxds */
      {
	/* Rt = (Ra[31:16] * Rb[15:0]) - (Ra[15:0] * Rb[31:16]) */
	int32_t mul_hi_lo = CCPU_GPR[ra].b16.hi * CCPU_GPR[rb].b16.lo;
	int32_t mul_lo_hi = CCPU_GPR[ra].b16.lo * CCPU_GPR[rb].b16.hi;
	CCPU_GPR[rt].s = mul_hi_lo - mul_lo_hi;
      }
      break;
    case 0x72:			/* smdrs */
      {
	/* Rt = (Ra[15:0] * Rb[15:0]) - (Ra[31:16] * Rb[31:16]) */
	int32_t mul_hi = CCPU_GPR[ra].b16.hi * CCPU_GPR[rb].b16.hi;
	int32_t mul_lo = CCPU_GPR[ra].b16.lo * CCPU_GPR[rb].b16.lo;
	CCPU_GPR[rt].s = mul_lo - mul_hi;
      }
      break;
    case 0x73:			/* maddr32 */
      CCPU_GPR[rt].u += (CCPU_GPR[ra].u * CCPU_GPR[rb].u);
      break;
    case 0x74:			/* kmadrs */
      {
	int32_t mul_hi = (int32_t) CCPU_GPR[ra].b16.hi * CCPU_GPR[rb].b16.hi;
	int32_t mul_lo = (int32_t) CCPU_GPR[ra].b16.lo * CCPU_GPR[rb].b16.lo;
	int64_t res = (int64_t) CCPU_GPR[rt].s + (mul_lo - mul_hi);
	CCPU_GPR[rt].s = insn_sat_helper (cpu, res, 31);
      }
      break;
    case 0x75:			/* msubr32 */
      CCPU_GPR[rt].u -= (CCPU_GPR[ra].u * CCPU_GPR[rb].u);
      break;
    case 0x76:			/* kmads */
      {
	int32_t mul_hi = (int32_t) CCPU_GPR[ra].b16.hi * CCPU_GPR[rb].b16.hi;
	int32_t mul_lo = (int32_t) CCPU_GPR[ra].b16.lo * CCPU_GPR[rb].b16.lo;
	int64_t res = (int64_t) CCPU_GPR[rt].s + (mul_hi - mul_lo);
	CCPU_GPR[rt].s = insn_sat_helper (cpu, res, 31);
      }
      break;
    case 0x77:			/* kmaxds */
      {
	int32_t mul_hi_lo = (int32_t) CCPU_GPR[ra].b16.hi * CCPU_GPR[rb].b16.lo;
	int32_t mul_lo_hi = (int32_t) CCPU_GPR[ra].b16.lo * CCPU_GPR[rb].b16.hi;
	int64_t res = (int64_t) CCPU_GPR[rt].s + (mul_hi_lo - mul_lo_hi);
	CCPU_GPR[rt].s = insn_sat_helper (cpu, res, 31);
      }
      break;
    case 0x80:			/* kadd16 */
      {
	/* Rt[31:16] = Ra[31:16] + Rb[31:16]
	   Rt[15:0] = Ra[15:0] + Rb[15:0] */
	reg_t result;
	int32_t res;
	int16_t *ptr, *ptr_a, *ptr_b;
	ptr = (int16_t *) & result.b16;
	ptr_a = (int16_t *) & CCPU_GPR[ra].b16;
	ptr_b = (int16_t *) & CCPU_GPR[rb].b16;
	for (i = 0; i < 2; i++)
	  {
	    res = *(ptr_a + i) + *(ptr_b + i);
	    if (res > 0x7fff)
	      {
		res = 0x7fff;
		CCPU_SR_SET (PSW, PSW_OV);
	      }
	    else if (res < -0x8000)
	      {
		res = -0x8000;
		CCPU_SR_SET (PSW, PSW_OV);
	      }
	    *(ptr + i) = res;
	  }
	CCPU_GPR[rt].u = result.u;
      }
      break;
    case 0x81:			/* ksub16 */
      {
	/* Rt[31:16] = Ra[31:16] - Rb[31:16]
	   Rt[15:0] = Ra[15:0] - Rb[15:0] */
	int32_t res1 = CCPU_GPR[ra].b16.hi - CCPU_GPR[rb].b16.hi;
	int32_t res2 = CCPU_GPR[ra].b16.lo - CCPU_GPR[rb].b16.lo;
	CCPU_GPR[rt].b16.hi = insn_sat_helper (cpu, res1, 15);
	CCPU_GPR[rt].b16.lo = insn_sat_helper (cpu, res2, 15);
      }
      break;
    case 0x82:			/* kcras16 */
      {
	/* Rt[31:16] = Ra[31:16] + Rb[15:0]
	   Rt[15:0] = Ra[15:0] - Rb[31:16] */
	int32_t res1 = CCPU_GPR[ra].b16.hi + CCPU_GPR[rb].b16.lo;
	int32_t res2 = CCPU_GPR[ra].b16.lo - CCPU_GPR[rb].b16.hi;
	CCPU_GPR[rt].b16.hi = insn_sat_helper (cpu, res1, 15);
	CCPU_GPR[rt].b16.lo = insn_sat_helper (cpu, res2, 15);
      }
      break;
    case 0x83:			/* kcrsa16 */
      {
	/* Rt[31:16] = Ra[31:16] - Rb[15:0]
	   Rt[15:0] = Ra[15:0] + Rb[31:16] */
	int32_t res1 = CCPU_GPR[ra].b16.hi - CCPU_GPR[rb].b16.lo;
	int32_t res2 = CCPU_GPR[ra].b16.lo + CCPU_GPR[rb].b16.hi;
	CCPU_GPR[rt].b16.hi = insn_sat_helper (cpu, res1, 15);
	CCPU_GPR[rt].b16.lo = insn_sat_helper (cpu, res2, 15);
      }
      break;
    case 0x84:			/* kadd8 */
      {
	/* Rt[31:24] = Ra[31:24] + Rb[31:24]
	   Rt[23:16] = Ra[23:16] + Rb[23:16]
	   Rt[15:8] = Ra[8:0] + Rb[8:0]
	   Rt[7:0] = Ra[7:0] + Rb[7:0] */
	int32_t res1 = CCPU_GPR[ra].b8.b0 + CCPU_GPR[rb].b8.b0;
	int32_t res2 = CCPU_GPR[ra].b8.b1 + CCPU_GPR[rb].b8.b1;
	int32_t res3 = CCPU_GPR[ra].b8.b2 + CCPU_GPR[rb].b8.b2;
	int32_t res4 = CCPU_GPR[ra].b8.b3 + CCPU_GPR[rb].b8.b3;
	CCPU_GPR[rt].b8.b0 = insn_sat_helper (cpu, res1, 7);
	CCPU_GPR[rt].b8.b1 = insn_sat_helper (cpu, res2, 7);
	CCPU_GPR[rt].b8.b2 = insn_sat_helper (cpu, res3, 7);
	CCPU_GPR[rt].b8.b3 = insn_sat_helper (cpu, res4, 7);
      }
      break;
    case 0x85:			/* ksub8 */
      {
	/* Rt[31:24] = Ra[31:24] - Rb[31:24]
	   Rt[23:16] = Ra[23:16] - Rb[23:16]
	   Rt[15:8] = Ra[8:0] - Rb[8:0]
	   Rt[7:0] = Ra[7:0] - Rb[7:0] */
	int32_t res1 = CCPU_GPR[ra].b8.b0 - CCPU_GPR[rb].b8.b0;
	int32_t res2 = CCPU_GPR[ra].b8.b1 - CCPU_GPR[rb].b8.b1;
	int32_t res3 = CCPU_GPR[ra].b8.b2 - CCPU_GPR[rb].b8.b2;
	int32_t res4 = CCPU_GPR[ra].b8.b3 - CCPU_GPR[rb].b8.b3;
	CCPU_GPR[rt].b8.b0 = insn_sat_helper (cpu, res1, 7);
	CCPU_GPR[rt].b8.b1 = insn_sat_helper (cpu, res2, 7);
	CCPU_GPR[rt].b8.b2 = insn_sat_helper (cpu, res3, 7);
	CCPU_GPR[rt].b8.b3 = insn_sat_helper (cpu, res4, 7);
      }
      break;
    case 0x86:			/* wext */
      CCPU_GPR[rt].s = (int32_t) (get_udouble (cpu, ra) >> (CCPU_GPR[rb].u & 0x1f));
      break;
    case 0x87:			/* wexti */
      CCPU_GPR[rt].s = (int32_t) (get_udouble (cpu, ra) >> imm5u);
      break;
    case 0x88:			/* ukadd16 */
      {
	/* Rt[31:16] = Ra[31:16] + Rb[31:16]
	   Rt[15:0] = Ra[15:0] + Rb[15:0] */
	int32_t res1 = CCPU_GPR[ra].ub16.hi + CCPU_GPR[rb].ub16.hi;
	int32_t res2 = CCPU_GPR[ra].ub16.lo + CCPU_GPR[rb].ub16.lo;
	res1 = insn_usat_helper (cpu, res1, 16);
	res2 = insn_usat_helper (cpu, res2, 16);
	CCPU_GPR[rt].ub16.hi = res1;
	CCPU_GPR[rt].ub16.lo = res2;
      }
      break;
    case 0x89:			/* uksub16 */
      {
	/* Rt[31:16] = Ra[31:16] - Rb[31:16]
	   Rt[15:0] = Ra[15:0] - Rb[15:0] */
	int32_t res1 = CCPU_GPR[ra].ub16.hi - CCPU_GPR[rb].ub16.hi;
	int32_t res2 = CCPU_GPR[ra].ub16.lo - CCPU_GPR[rb].ub16.lo;
	res1 = insn_usat_helper (cpu, res1, 16);
	res2 = insn_usat_helper (cpu, res2, 16);
	CCPU_GPR[rt].ub16.hi = res1;
	CCPU_GPR[rt].ub16.lo = res2;
      }
      break;
    case 0x8a:			/* ukcras16 */
      {
	/* Rt[31:16] = Ra[31:16] + Rb[15:0]
	   Rt[15:0] = Ra[15:0] - Rb[31:16] */
	int32_t res1 = CCPU_GPR[ra].ub16.hi + CCPU_GPR[rb].ub16.lo;
	int32_t res2 = CCPU_GPR[ra].ub16.lo - CCPU_GPR[rb].ub16.hi;
	res1 = insn_usat_helper (cpu, res1, 16);
	res2 = insn_usat_helper (cpu, res2, 16);
	CCPU_GPR[rt].ub16.hi = res1;
	CCPU_GPR[rt].ub16.lo = res2;
      }
      break;
    case 0x8b:			/* ukcrsa16 */
      {
	/* Rt[31:16] = Ra[31:16] - Rb[15:0]
	   Rt[15:0] = Ra[15:0] + Rb[31:16] */
	int32_t res1 = CCPU_GPR[ra].ub16.hi - CCPU_GPR[rb].ub16.lo;
	int32_t res2 = CCPU_GPR[ra].ub16.lo + CCPU_GPR[rb].ub16.hi;
	res1 = insn_usat_helper (cpu, res1, 16);
	res2 = insn_usat_helper (cpu, res2, 16);
	CCPU_GPR[rt].ub16.hi = res1;
	CCPU_GPR[rt].ub16.lo = res2;
      }
      break;
    case 0x8c:			/* ukadd8 */
      {
	/* Rt[31:24] = Ra[31:24] + Rb[31:24]
	   Rt[23:16] = Ra[23:16] + Rb[23:16]
	   Rt[15:8] = Ra[8:0] + Rb[8:0]
	   Rt[7:0] = Ra[7:0] + Rb[7:0] */
	int32_t res1 = CCPU_GPR[ra].ub8.b0 + CCPU_GPR[rb].ub8.b0;
	int32_t res2 = CCPU_GPR[ra].ub8.b1 + CCPU_GPR[rb].ub8.b1;
	int32_t res3 = CCPU_GPR[ra].ub8.b2 + CCPU_GPR[rb].ub8.b2;
	int32_t res4 = CCPU_GPR[ra].ub8.b3 + CCPU_GPR[rb].ub8.b3;
	res1 = insn_usat_helper (cpu, res1, 8);
	res2 = insn_usat_helper (cpu, res2, 8);
	res3 = insn_usat_helper (cpu, res3, 8);
	res4 = insn_usat_helper (cpu, res4, 8);
	CCPU_GPR[rt].ub8.b0 = res1;
	CCPU_GPR[rt].ub8.b1 = res2;
	CCPU_GPR[rt].ub8.b2 = res3;
	CCPU_GPR[rt].ub8.b3 = res4;
      }
      break;
    case 0x8d:			/* uksub8 */
      {
	/* Rt[31:24] = Ra[31:24] - Rb[31:24]
	   Rt[23:16] = Ra[23:16] - Rb[23:16]
	   Rt[15:8] = Ra[8:0] - Rb[8:0]
	   Rt[7:0] = Ra[7:0] - Rb[7:0] */
	int32_t res1 = CCPU_GPR[ra].ub8.b0 - CCPU_GPR[rb].ub8.b0;
	int32_t res2 = CCPU_GPR[ra].ub8.b1 - CCPU_GPR[rb].ub8.b1;
	int32_t res3 = CCPU_GPR[ra].ub8.b2 - CCPU_GPR[rb].ub8.b2;
	int32_t res4 = CCPU_GPR[ra].ub8.b3 - CCPU_GPR[rb].ub8.b3;
	res1 = insn_usat_helper (cpu, res1, 8);
	res2 = insn_usat_helper (cpu, res2, 8);
	res3 = insn_usat_helper (cpu, res3, 8);
	res4 = insn_usat_helper (cpu, res4, 8);
	CCPU_GPR[rt].ub8.b0 = res1;
	CCPU_GPR[rt].ub8.b1 = res2;
	CCPU_GPR[rt].ub8.b2 = res3;
	CCPU_GPR[rt].ub8.b3 = res4;
      }
      break;
    case 0x8f:			/* ONEOP */
      nds32_decode32_oneop (cpu, insn, cia);
      break;
    case 0x90:			/* smbb */
      {
	/* Rt = Ra[15:0] * Rb[15:0] */
	CCPU_GPR[rt].s = CCPU_GPR[ra].b16.lo * CCPU_GPR[rb].b16.lo;
      }
      break;
    case 0x91:			/* smbt */
      {
	/* Rt = Ra[15:0] * Rb[31:16] */
	CCPU_GPR[rt].s = CCPU_GPR[ra].b16.lo * CCPU_GPR[rb].b16.hi;
      }
      break;
    case 0x92:			/* smtt */
      {
	/* Rt = Ra[31:16] * Rb[31:16] */
	CCPU_GPR[rt].s = CCPU_GPR[ra].b16.hi * CCPU_GPR[rb].b16.hi;
      }
      break;
    case 0x95:			/* kmabb */
      {
	int32_t mul = (int32_t) CCPU_GPR[ra].b16.lo * CCPU_GPR[rb].b16.lo;
	int64_t res = (int64_t) CCPU_GPR[rt].s + mul;
	CCPU_GPR[rt].s = insn_sat_helper (cpu, res, 31);
      }
      break;
    case 0x96:			/* kmabt */
      {
	int32_t mul = (int32_t) CCPU_GPR[ra].b16.lo * CCPU_GPR[rb].b16.hi;
	int64_t res = (int64_t) CCPU_GPR[rt].s + mul;
	CCPU_GPR[rt].s = insn_sat_helper (cpu, res, 31);
      }
      break;
    case 0x97:			/* kmatt */
      {
	int32_t mul = (int32_t) CCPU_GPR[ra].b16.hi * CCPU_GPR[rb].b16.hi;
	int64_t res = (int64_t) CCPU_GPR[rt].s + mul;
	CCPU_GPR[rt].s = insn_sat_helper (cpu, res, 31);
      }
      break;
    case 0x98:			/* kmda */
      {
	int32_t res;
	if ((CCPU_GPR[ra].s != 0x80008000) || (CCPU_GPR[rb].s != 0x80008000))
	  {
	    res = ((int32_t) CCPU_GPR[ra].b16.hi * CCPU_GPR[rb].b16.hi)
		   + ((int32_t) CCPU_GPR[ra].b16.lo * CCPU_GPR[rb].b16.lo);
	  }
	else
	  {
	    res = 0x7fffffff;
	    CCPU_SR_SET (PSW, PSW_OV);
	  }
	CCPU_GPR[rt].s = res;
      }
      break;
    case 0x99:			/* kmxda */
      {
	int32_t res;
	if ((CCPU_GPR[ra].s != 0x80008000) || (CCPU_GPR[rb].s != 0x80008000))
	  {
	    res = ((int32_t) CCPU_GPR[ra].b16.hi * CCPU_GPR[rb].b16.lo)
		   + ((int32_t) CCPU_GPR[ra].b16.lo * CCPU_GPR[rb].b16.hi);
	  }
	else
	  {
	    res = 0x7fffffff;
	    CCPU_SR_SET (PSW, PSW_OV);
	  }
	CCPU_GPR[rt].s = res;
      }
      break;
    case 0x9a:			/* kmada */
      {
	int64_t res = (int64_t) CCPU_GPR[rt].s
		      + ((int32_t) CCPU_GPR[ra].b16.hi * CCPU_GPR[rb].b16.hi)
		      + ((int32_t) CCPU_GPR[ra].b16.lo * CCPU_GPR[rb].b16.lo);
	CCPU_GPR[rt].s = insn_sat_helper (cpu, res, 31);
      }
      break;
    case 0x9b:			/* kmaxda */
      {
	int64_t res = (int64_t) CCPU_GPR[rt].s
		      + ((int32_t) CCPU_GPR[ra].b16.hi * CCPU_GPR[rb].b16.lo)
		      + ((int32_t) CCPU_GPR[ra].b16.lo * CCPU_GPR[rb].b16.hi);
	CCPU_GPR[rt].s = insn_sat_helper (cpu, res, 31);
      }
      break;
    case 0x9c:			/* kmsda */
      {
	int32_t mul_hi = (int32_t) CCPU_GPR[ra].b16.hi * CCPU_GPR[rb].b16.hi;
	int32_t mul_lo = (int32_t) CCPU_GPR[ra].b16.lo * CCPU_GPR[rb].b16.lo;
	int64_t res = (int64_t) CCPU_GPR[rt].s - (mul_hi + mul_lo);
	CCPU_GPR[rt].s = insn_sat_helper (cpu, res, 31);
      }
      break;
    case 0x9d:			/* kmsxda */
      {
	int32_t mul_hi_lo = (int32_t) CCPU_GPR[ra].b16.hi * CCPU_GPR[rb].b16.lo;
	int32_t mul_lo_hi = (int32_t) CCPU_GPR[ra].b16.lo * CCPU_GPR[rb].b16.hi;
	int64_t res = (int64_t) CCPU_GPR[rt].s - (mul_hi_lo + mul_lo_hi);
	CCPU_GPR[rt].s = insn_sat_helper (cpu, res, 31);
      }
      break;
    case 0xa0:			/* radd16 */
      {
	/* Rt[31:16] = (Ra[31:16] + Rb[31:16]) >> 1
	   Rt[15:0] = (Ra[15:0] + Rb[15:0]) >> 1 */
	reg_t result;
	result.b16.hi = (int16_t) (((int32_t) CCPU_GPR[ra].b16.hi
				     + CCPU_GPR[rb].b16.hi) >> 1);
	result.b16.lo = (int16_t) (((int32_t) CCPU_GPR[ra].b16.lo
				     + CCPU_GPR[rb].b16.lo) >> 1);
	CCPU_GPR[rt].s = result.s;
      }
      break;
    case 0xa1:			/* rsub16 */
      {
	/* Rt[31:16] = (Ra[31:16] - Rb[31:16]) >> 1
	   Rt[15:0] = (Ra[15:0] - Rb[15:0]) >> 1 */
	reg_t result;
	result.b16.hi = (int16_t) (((int32_t) CCPU_GPR[ra].b16.hi
				     - CCPU_GPR[rb].b16.hi) >> 1);
	result.b16.lo = (int16_t) (((int32_t) CCPU_GPR[ra].b16.lo
				     - CCPU_GPR[rb].b16.lo) >> 1);
	CCPU_GPR[rt].s = result.s;
      }
      break;
    case 0xa2:			/* rcras16 */
      {
	/* Rt[31:16] = (Ra[31:16] + Rb[15:0]) >>1
	   Rt[15:0] = (Ra[15:0] - Rb[31:16]) >> 1 */
	reg_t result;
	result.b16.hi = (int16_t) (((int32_t) CCPU_GPR[ra].b16.hi
				     + CCPU_GPR[rb].b16.lo) >> 1);
	result.b16.lo = (int16_t) (((int32_t) CCPU_GPR[ra].b16.lo
				     - CCPU_GPR[rb].b16.hi) >> 1);
	CCPU_GPR[rt].s = result.s;
      }
      break;
    case 0xa3:			/* rcrsa16 */
      {
	/* Rt[31:16] = (Ra[31:16] - Rb[15:0]) >> 1
	   Rt[15:0] = (Ra[15:0] + Rb[31:16]) >> 1 */
	reg_t result;
	result.b16.hi = (int16_t) (((int32_t) CCPU_GPR[ra].b16.hi
				     - CCPU_GPR[rb].b16.lo) >> 1);
	result.b16.lo = (int16_t) (((int32_t) CCPU_GPR[ra].b16.lo
				     + CCPU_GPR[rb].b16.hi) >> 1);
	CCPU_GPR[rt].s = result.s;
      }
      break;
    case 0xa4:			/* radd8 */
      {
	/* Rt[31:24] = (Ra[31:24] + Rb[31:24]) >> 1
	   Rt[23:16] = (Ra[23:16] + Rb[23:16]) >> 1
	   Rt[15:8] = (Ra[8:0] + Rb[8:0]) >> 1
	   Rt[7:0] = (Ra[7:0] + Rb[7:0]) >> 1 */
	reg_t result;
	result.b8.b3 = (int8_t) (((int16_t) CCPU_GPR[ra].b8.b3
				   + CCPU_GPR[rb].b8.b3) >> 1);
	result.b8.b2 = (int8_t) (((int16_t) CCPU_GPR[ra].b8.b2
				   + CCPU_GPR[rb].b8.b2) >> 1);
	result.b8.b1 = (int8_t) (((int16_t) CCPU_GPR[ra].b8.b1
				   + CCPU_GPR[rb].b8.b1) >> 1);
	result.b8.b0 = (int8_t) (((int16_t) CCPU_GPR[ra].b8.b0
				   + CCPU_GPR[rb].b8.b0) >> 1);
	CCPU_GPR[rt].s = result.s;
      }
      break;
    case 0xa5:			/* rsub8 */
      {
	/* Rt[31:24] = (Ra[31:24] - Rb[31:24]) >> 1
	   Rt[23:16] = (Ra[23:16] - Rb[23:16]) >> 1
	   Rt[15:8] = (Ra[8:0] - Rb[8:0]) >> 1
	   Rt[7:0] = (Ra[7:0] - Rb[7:0]) >> 1 */
	reg_t result;
	result.b8.b3 = (int8_t) (((int16_t) CCPU_GPR[ra].b8.b3
				   - CCPU_GPR[rb].b8.b3) >> 1);
	result.b8.b2 = (int8_t) (((int16_t) CCPU_GPR[ra].b8.b2
				   - CCPU_GPR[rb].b8.b2) >> 1);
	result.b8.b1 = (int8_t) (((int16_t) CCPU_GPR[ra].b8.b1
				   - CCPU_GPR[rb].b8.b1) >> 1);
	result.b8.b0 = (int8_t) (((int16_t) CCPU_GPR[ra].b8.b0
				   - CCPU_GPR[rb].b8.b0) >> 1);
	CCPU_GPR[rt].s = result.s;
      }
      break;
    case 0xa6:			/* raddw */
      {
	CCPU_GPR[rt].s = (int32_t) (((int64_t) CCPU_GPR[ra].s
				     + (int64_t) CCPU_GPR[rb].s) >> 1);
      }
      break;
    case 0xa7:			/* rsubw */
      {
	CCPU_GPR[rt].s = (int32_t) (((int64_t) CCPU_GPR[ra].s
				     - (int64_t) CCPU_GPR[rb].s) >> 1);
      }
      break;
    case 0xa8:			/* uradd16 */
      {
	/* Rt[31:16] = Ra[31:16] + Rb[31:16]
	   Rt[15:0] = Ra[15:0] + Rb[15:0] */
	reg_t result;
	result.ub16.hi = (uint16_t) (((uint32_t) CCPU_GPR[ra].ub16.hi
				       + CCPU_GPR[rb].ub16.hi) >> 1);
	result.ub16.lo = (uint16_t) (((uint32_t) CCPU_GPR[ra].ub16.lo
				       + CCPU_GPR[rb].ub16.lo) >> 1);
	CCPU_GPR[rt].u = result.u;
      }
      break;
    case 0xa9:			/* ursub16 */
      {
	/* Rt[31:16] = Ra[31:16] - Rb[31:16]
	   Rt[15:0] = Ra[15:0] - Rb[15:0] */
	reg_t result;
	result.ub16.hi = (uint16_t) (((uint32_t) CCPU_GPR[ra].ub16.hi
				       - CCPU_GPR[rb].ub16.hi) >> 1);
	result.ub16.lo = (uint16_t) (((uint32_t) CCPU_GPR[ra].ub16.lo
				       - CCPU_GPR[rb].ub16.lo) >> 1);
	CCPU_GPR[rt].u = result.u;
      }
      break;
    case 0xaa:			/* urcras16 */
      {
	/* Rt[31:16] = (Ra[31:16] + Rb[15:0]) >>1
	   Rt[15:0] = (Ra[15:0] - Rb[31:16]) >> 1 */
	reg_t result;
	result.ub16.hi = (uint16_t) (((uint32_t) CCPU_GPR[ra].ub16.hi
				       + CCPU_GPR[rb].ub16.lo) >> 1);
	result.ub16.lo = (uint16_t) (((uint32_t) CCPU_GPR[ra].ub16.lo
				       - CCPU_GPR[rb].ub16.hi) >> 1);
	CCPU_GPR[rt].u = result.u;
      }
      break;
    case 0xab:			/* urcrsa16 */
      {
	/* Rt[31:16] = (Ra[31:16] - Rb[15:0]) >> 1
	   Rt[15:0] = (Ra[15:0] + Rb[31:16]) >> 1 */
	reg_t result;
	result.ub16.hi = (uint16_t) (((uint32_t) CCPU_GPR[ra].ub16.hi
				       - CCPU_GPR[rb].ub16.lo) >> 1);
	result.ub16.lo = (uint16_t) (((uint32_t) CCPU_GPR[ra].ub16.lo
				       + CCPU_GPR[rb].ub16.hi) >> 1);
	CCPU_GPR[rt].u = result.u;
      }
      break;
    case 0xac:			/* uradd8 */
      {
	/* Rt[31:24] = (Ra[31:24] + Rb[31:24]) >> 1
	   Rt[23:16] = (Ra[23:16] + Rb[23:16]) >> 1
	   Rt[15:8] = (Ra[8:0] + Rb[8:0]) >> 1
	   Rt[7:0] = (Ra[7:0] + Rb[7:0]) >> 1 */
	reg_t result;
	result.ub8.b3 = (uint8_t) (((uint16_t) CCPU_GPR[ra].ub8.b3
				     + CCPU_GPR[rb].ub8.b3) >> 1);
	result.ub8.b2 = (uint8_t) (((uint16_t) CCPU_GPR[ra].ub8.b2
				     + CCPU_GPR[rb].ub8.b2) >> 1);
	result.ub8.b1 = (uint8_t) (((uint16_t) CCPU_GPR[ra].ub8.b1
				     + CCPU_GPR[rb].ub8.b1) >> 1);
	result.ub8.b0 = (uint8_t) (((uint16_t) CCPU_GPR[ra].ub8.b0
				     + CCPU_GPR[rb].ub8.b0) >> 1);
	CCPU_GPR[rt].u = result.u;
      }
      break;
    case 0xad:			/* ursub8 */
      {
	/* Rt[31:24] = (Ra[31:24] - Rb[31:24]) >> 1
	   Rt[23:16] = (Ra[23:16] - Rb[23:16]) >> 1
	   Rt[15:8] = (Ra[8:0] - Rb[8:0]) >> 1
	   Rt[7:0] = (Ra[7:0] - Rb[7:0]) >> 1 */
	reg_t result;
	result.ub8.b3 = (uint8_t) (((uint16_t) CCPU_GPR[ra].ub8.b3
				     - CCPU_GPR[rb].ub8.b3) >> 1);
	result.ub8.b2 = (uint8_t) (((uint16_t) CCPU_GPR[ra].ub8.b2
				     - CCPU_GPR[rb].ub8.b2) >> 1);
	result.ub8.b1 = (uint8_t) (((uint16_t) CCPU_GPR[ra].ub8.b1
				     - CCPU_GPR[rb].ub8.b1) >> 1);
	result.ub8.b0 = (uint8_t) (((uint16_t) CCPU_GPR[ra].ub8.b0
				     - CCPU_GPR[rb].ub8.b0) >> 1);
	CCPU_GPR[rt].u = result.u;
      }
      break;
    case 0xae:			/* uraddw */
      {
	CCPU_GPR[rt].u =
	  (uint32_t) (((uint64_t)CCPU_GPR[ra].u + CCPU_GPR[rb].u) >> 1);
      }
      break;
    case 0xaf:			/* ursubw */
      {
	CCPU_GPR[rt].u =
	  (uint32_t) (((uint64_t)CCPU_GPR[ra].u - CCPU_GPR[rb].u) >> 1);
      }
      break;
    case 0xb0:			/* add16 */
      {
	/* Rt[31:16] = Ra[31:16] + Rb[31:16]
	   Rt[15:0] = Ra[15:0] + Rb[15:0] */
	reg_t result;
	result.b16.hi = CCPU_GPR[ra].b16.hi + CCPU_GPR[rb].b16.hi;
	result.b16.lo = CCPU_GPR[ra].b16.lo + CCPU_GPR[rb].b16.lo;
	CCPU_GPR[rt].s = result.s;
      }
      break;
    case 0xb1:			/* sub16 */
      {
	/* Rt[31:16] = Ra[31:16] - Rb[31:16]
	   Rt[15:0] = Ra[15:0] - Rb[15:0] */
	reg_t result;
	result.b16.hi = CCPU_GPR[ra].b16.hi - CCPU_GPR[rb].b16.hi;
	result.b16.lo = CCPU_GPR[ra].b16.lo - CCPU_GPR[rb].b16.lo;
	CCPU_GPR[rt].s = result.s;
      }
      break;
    case 0xb2:			/* cras16 */
      {
	/* Rt[31:16] = Ra[31:16] + Rb[15:0]
	   Rt[15:0] = Ra[15:0] - Rb[31:16] */
	reg_t result;
	result.b16.hi = CCPU_GPR[ra].b16.hi + CCPU_GPR[rb].b16.lo;
	result.b16.lo = CCPU_GPR[ra].b16.lo - CCPU_GPR[rb].b16.hi;
	CCPU_GPR[rt].s = result.s;
      }
      break;
    case 0xb3:			/* crsa16 */
      {
	/* Rt[31:16] = Ra[31:16] - Rb[15:0]
	   Rt[15:0] = Ra[15:0] + Rb[31:16] */
	reg_t result;
	result.b16.hi = CCPU_GPR[ra].b16.hi - CCPU_GPR[rb].b16.lo;
	result.b16.lo = CCPU_GPR[ra].b16.lo + CCPU_GPR[rb].b16.hi;
	CCPU_GPR[rt].s = result.s;
      }
      break;
    case 0xb4:			/* add8 */
      {
	/* Rt[31:24] = Ra[31:24] + Rb[31:24]
	   Rt[23:16] = Ra[23:16] + Rb[23:16]
	   Rt[15:8] = Ra[8:0] + Rb[8:0]
	   Rt[7:0] = Ra[7:0] + Rb[7:0] */
	reg_t result;
	result.b8.b3 = CCPU_GPR[ra].b8.b3 + CCPU_GPR[rb].b8.b3;
	result.b8.b2 = CCPU_GPR[ra].b8.b2 + CCPU_GPR[rb].b8.b2;
	result.b8.b1 = CCPU_GPR[ra].b8.b1 + CCPU_GPR[rb].b8.b1;
	result.b8.b0 = CCPU_GPR[ra].b8.b0 + CCPU_GPR[rb].b8.b0;
	CCPU_GPR[rt].s = result.s;
      }
      break;
    case 0xb5:			/* sub8 */
      {
	/* Rt[31:24] = Ra[31:24] - Rb[31:24]
	   Rt[23:16] = Ra[23:16] - Rb[23:16]
	   Rt[15:8] = Ra[8:0] - Rb[8:0]
	   Rt[7:0] = Ra[7:0] - Rb[7:0] */
	reg_t result;
	result.b8.b3 = CCPU_GPR[ra].b8.b3 - CCPU_GPR[rb].b8.b3;
	result.b8.b2 = CCPU_GPR[ra].b8.b2 - CCPU_GPR[rb].b8.b2;
	result.b8.b1 = CCPU_GPR[ra].b8.b1 - CCPU_GPR[rb].b8.b1;
	result.b8.b0 = CCPU_GPR[ra].b8.b0 - CCPU_GPR[rb].b8.b0;
	CCPU_GPR[rt].s = result.s;
      }
      break;
    case 0xb6:			/* bitrev */
      {
	uint32_t bits = CCPU_GPR[ra].u;
	bits = (((bits & 0xaaaaaaaa) >> 1) | ((bits & 0x55555555) << 1));
	bits = (((bits & 0xcccccccc) >> 2) | ((bits & 0x33333333) << 2));
	bits = (((bits & 0xf0f0f0f0) >> 4) | ((bits & 0x0f0f0f0f) << 4));
	bits = (((bits & 0xff00ff00) >> 8) | ((bits & 0x00ff00ff) << 8));
	bits = ((bits >> 16) | (bits << 16));

	CCPU_GPR[rt].u = bits >> (32 - (CCPU_GPR[rb].u + 1));
      }
      break;
    case 0xb7:			/* bitrevi */
      {
	uint32_t bits = CCPU_GPR[ra].u;
	bits = (((bits & 0xaaaaaaaa) >> 1) | ((bits & 0x55555555) << 1));
	bits = (((bits & 0xcccccccc) >> 2) | ((bits & 0x33333333) << 2));
	bits = (((bits & 0xf0f0f0f0) >> 4) | ((bits & 0x0f0f0f0f) << 4));
	bits = (((bits & 0xff00ff00) >> 8) | ((bits & 0x00ff00ff) << 8));
	bits = ((bits >> 16) | (bits << 16));

	CCPU_GPR[rt].u = bits >> (32 - (imm5u + 1));
      }
      break;
    case 0xb8:			/* smmul */
      {
	CCPU_GPR[rt].s =
	  ((int64_t) CCPU_GPR[ra].s * (int64_t) CCPU_GPR[rb].s) >> 32;
      }
      break;
    case 0xb9:			/* smmul.u */
      {
	int64_t result = (int64_t) CCPU_GPR[ra].s * (int64_t) CCPU_GPR[rb].s;
	int32_t round_up = (result >> 31) & 0x1;

	/* Round up */
        if (round_up != 0)
	  CCPU_GPR[rt].s = (result >> 32) + 1;
	else
	  CCPU_GPR[rt].s = result >> 32;
      }
      break;
    case 0xba:			/* kmmac */
      {
	union64_t temp, res;
	temp.d0 = (int64_t) CCPU_GPR[ra].s * CCPU_GPR[rb].s;
	res.d0 = (int64_t) CCPU_GPR[rt].s + temp.b32.w1;
	res.d0 = insn_sat_helper (cpu, res.d0, 31);
	CCPU_GPR[rt].s = res.d0;
      }
      break;
    case 0xbb:			/* kmmac.u */
      {
	union64_t temp, res;
	temp.d0 = (int64_t) CCPU_GPR[ra].s * CCPU_GPR[rb].s;

	if ((temp.b32.w0 >> 31) != 0)
	  temp.b32.w1 += 1;

	res.d0 = (int64_t) CCPU_GPR[rt].s + temp.b32.w1;
	res.d0 = insn_sat_helper (cpu, res.d0, 31);
	CCPU_GPR[rt].s = res.d0;
      }
      break;
    case 0xbc:			/* kmmsb */
      {
	union64_t temp, res;
	temp.d0 = (int64_t) CCPU_GPR[ra].s * CCPU_GPR[rb].s;
	res.d0 = (int64_t) CCPU_GPR[rt].s - temp.b32.w1;
	res.d0 = insn_sat_helper (cpu, res.d0, 31);
	CCPU_GPR[rt].s = res.d0;
      }
      break;
    case 0xbd:			/* kmmsb.u */
      {
	union64_t temp, res;
	temp.d0 = (int64_t) CCPU_GPR[ra].s * CCPU_GPR[rb].s;

	if ((temp.b32.w0 >> 31) != 0)
	  temp.b32.w1 += 1;

	res.d0 = (int64_t) CCPU_GPR[rt].s - temp.b32.w1;
	res.d0 = insn_sat_helper (cpu, res.d0, 31);
	CCPU_GPR[rt].s = res.d0;
      }
      break;
    case 0xbe:			/* kwmmul */
      {
	union64_t temp;

	if ((CCPU_GPR[ra].s != 0x80000000) || (CCPU_GPR[rb].s != 0x80000000))
	  {
	    temp.d0 = ((int64_t) CCPU_GPR[ra].s * CCPU_GPR[rb].s) << 1;
	    CCPU_GPR[rt].s = temp.b32.w1;
	  }
	else
	  {
	    CCPU_GPR[rt].s = 0x7fffffff;
	    CCPU_SR_SET (PSW, PSW_OV);
	  }
      }
      break;
    case 0xbf:			/* kwmmul.u */
      {
	union64_t temp;

	if ((CCPU_GPR[ra].s != 0x80000000) || (CCPU_GPR[rb].s != 0x80000000))
	  {
	    temp.d0 = (int64_t) CCPU_GPR[ra].s * CCPU_GPR[rb].s;
	    /* Let 30bit add 1 and left shift 1.  */
	    temp.d0 = (temp.d0 + (int32_t) (1 << 30)) << 1;
	    CCPU_GPR[rt].s = temp.b32.w1;
	  }
	else
	  {
	    CCPU_GPR[rt].s = 0x7fffffff;
	    CCPU_SR_SET (PSW, PSW_OV);
	  }
      }
      break;
    case 0xc0:			/* smmwb */
      {
	CCPU_GPR[rt].s =
	  ((int64_t) CCPU_GPR[ra].s * (int64_t) CCPU_GPR[rb].b16.lo) >> 16;
      }
      break;
    case 0xc1:			/* smmwb.u */
      {
	int64_t result =
	  (int64_t) CCPU_GPR[ra].s * (int64_t) CCPU_GPR[rb].b16.lo;
	int32_t round_up = (result >> 15) & 0x1;

	/* Round up */
        if (round_up != 0)
	  CCPU_GPR[rt].s = (result >> 16) + 1;
	else
	  CCPU_GPR[rt].s = result >> 16;
      }
      break;
    case 0xc2:			/* smmwt */
      {
	CCPU_GPR[rt].s =
	  ((int64_t) CCPU_GPR[ra].s * (int64_t) CCPU_GPR[rb].b16.hi) >> 16;
      }
      break;
    case 0xc3:			/* smmwt.u */
      {
	int64_t result =
	  (int64_t) CCPU_GPR[ra].s * (int64_t) CCPU_GPR[rb].b16.hi;
	int32_t round_up = (result >> 15) & 0x1;

	/* Round up */
        if (round_up != 0)
	  CCPU_GPR[rt].s = (result >> 16) + 1;
	else
	  CCPU_GPR[rt].s = result >> 16;
      }
      break;
    case 0xc4:			/* kmmawb */
      {
	union64_t temp;
	int64_t res;

	temp.d0 = (int64_t) CCPU_GPR[ra].s * CCPU_GPR[rb].b16.lo;
	res = (int64_t) CCPU_GPR[rt].s + (int32_t) (temp.d0 >> 16);
	res = insn_sat_helper (cpu, res, 31);
	CCPU_GPR[rt].s = res;
      }
      break;
    case 0xc5:			/* kmmawb.u */
      {
	union64_t temp;
	int64_t res;
	int32_t rnd_val;

	temp.d0 = (int64_t) CCPU_GPR[ra].s * CCPU_GPR[rb].b16.lo;
	rnd_val = (temp.b32.w0 & (1UL << 15)) ? (1L << 16) : 0;
	temp.d0 += rnd_val;
	res = (int64_t) CCPU_GPR[rt].s + (int32_t) (temp.d0 >> 16);
	res = insn_sat_helper (cpu, res, 31);
	CCPU_GPR[rt].s = res;
      }
      break;
    case 0xc6:			/* kmmawt */
      {
	union64_t temp;
	int64_t res;

	temp.d0 = (int64_t) CCPU_GPR[ra].s * CCPU_GPR[rb].b16.hi;
	res = (int64_t) CCPU_GPR[rt].s + (int32_t) (temp.d0 >> 16);
	res = insn_sat_helper (cpu, res, 31);
	CCPU_GPR[rt].s = res;
      }
      break;
    case 0xc7:			/* kmmawt.u */
      {
	union64_t temp;
	int64_t res;
	int32_t rnd_val;

	temp.d0 = (int64_t) CCPU_GPR[ra].s * CCPU_GPR[rb].b16.hi;
	rnd_val = (temp.b32.w0 & (1UL << 15)) ? (1L << 16) : 0;
	temp.d0 += rnd_val;
	res = (int64_t) CCPU_GPR[rt].s + (int32_t) (temp.d0 >> 16);
	res = insn_sat_helper (cpu, res, 31);
	CCPU_GPR[rt].s = res;
      }
      break;
    case 0xc8:			/* pktt16 */
      {
	/* Rt[31:0] = CONCAT(Ra[31:16], Rb[31:16]) */
	CCPU_GPR[rt].s = (CCPU_GPR[ra].b16.hi << 16) | CCPU_GPR[rb].ub16.hi;
      }
      break;
    case 0xc9:			/* pktb16 */
      {
	/* Rt[31:0] = CONCAT(Ra[31:16], Rb[15:0]) */
	CCPU_GPR[rt].s = (CCPU_GPR[ra].b16.hi << 16) | CCPU_GPR[rb].ub16.lo;
      }
      break;
    case 0xca:			/* pkbt16 */
      {
	/* Rt[31:0] = CONCAT(Ra[15:0], Rb[31:16]) */
	CCPU_GPR[rt].s = (CCPU_GPR[ra].b16.lo << 16) | CCPU_GPR[rb].ub16.hi;
      }
      break;
    case 0xcb:			/* pkbb16 */
      {
	/* Rt[31:0] = CONCAT(Ra[15:0], Rb[15:0]) */
	CCPU_GPR[rt].s = (CCPU_GPR[ra].b16.lo << 16) | CCPU_GPR[rb].ub16.lo;
      }
      break;
    case 0xcc:			/* kdmabb */
      {
	int32_t mul = (int32_t) (CCPU_GPR[ra].b16.lo * CCPU_GPR[rb].b16.lo) << 1;
	int64_t res = (int64_t) CCPU_GPR[rt].s + mul;
	CCPU_GPR[rt].s = insn_sat_helper (cpu, res, 31);
      }
      break;
    case 0xcd:			/* kdmabt */
      {
	int32_t mul = (int32_t) (CCPU_GPR[ra].b16.lo * CCPU_GPR[rb].b16.hi) << 1;
	int64_t res = (int64_t) CCPU_GPR[rt].s + mul;
	CCPU_GPR[rt].s = insn_sat_helper (cpu, res, 31);
      }
      break;
    case 0xce:			/* kdmatt */
      {
	int32_t mul = (int32_t) (CCPU_GPR[ra].b16.hi * CCPU_GPR[rb].b16.hi) << 1;
	int64_t res = (int64_t) CCPU_GPR[rt].s + mul;
	CCPU_GPR[rt].s = insn_sat_helper (cpu, res, 31);
      }
      break;
    case 0xd1:			/* sclip16 */
      CCPU_GPR[rt].b16.hi = insn_sat_helper (cpu, CCPU_GPR[ra].b16.hi, imm4u);
      CCPU_GPR[rt].b16.lo = insn_sat_helper (cpu, CCPU_GPR[ra].b16.lo, imm4u);
      break;
    case 0xd3:			/* smax16 */
      {
	/* Rt[31:16] = (Ra[31:16] > Rb[31:16])? Ra[31:16] : Rb[31:16]
	   Rt[15:0] = (Ra[15:0] > Rb[15:0]) ? Ra[15:0] : Rb[15:0] */
	reg_t result;
	result.b16.hi = (CCPU_GPR[ra].b16.hi > CCPU_GPR[rb].b16.hi)
			 ? CCPU_GPR[ra].b16.hi : CCPU_GPR[rb].b16.hi;
	result.b16.lo = (CCPU_GPR[ra].b16.lo > CCPU_GPR[rb].b16.lo)
			 ? CCPU_GPR[ra].b16.lo : CCPU_GPR[rb].b16.lo;
	CCPU_GPR[rt].s = result.s;
      }
      break;
    case 0xd7:			/* smax8 */
      {
	/* Rt[31:24] = (Ra[31:24] > Rb[31:24])? Ra[31:24] : Rb[31:24]
	   Rt[23:16] = (Ra[23:16] > Rb[23:16])? Ra[23:16] : Rb[23:16]
	   Rt[15:8]  = (Ra[15:8]  > Rb[15:8]) ? Ra[15:8]  : Rb[15:8]
	   Rt[7:0]   = (Ra[7:0]   > Rb[7:0])  ? Ra[7:0]   : Rb[7:0]  */
	reg_t result;
	result.b8.b3 = (CCPU_GPR[ra].b8.b3 > CCPU_GPR[rb].b8.b3)
			? CCPU_GPR[ra].b8.b3 : CCPU_GPR[rb].b8.b3;
	result.b8.b2 = (CCPU_GPR[ra].b8.b2 > CCPU_GPR[rb].b8.b2)
			? CCPU_GPR[ra].b8.b2 : CCPU_GPR[rb].b8.b2;
	result.b8.b1 = (CCPU_GPR[ra].b8.b1 > CCPU_GPR[rb].b8.b1)
			? CCPU_GPR[ra].b8.b1 : CCPU_GPR[rb].b8.b1;
	result.b8.b0 = (CCPU_GPR[ra].b8.b0 > CCPU_GPR[rb].b8.b0)
			? CCPU_GPR[ra].b8.b0 : CCPU_GPR[rb].b8.b0;
	CCPU_GPR[rt].s = result.s;
      }
      break;
    case 0xd9:			/* uclip16 */
      CCPU_GPR[rt].b16.hi = insn_usat_helper (cpu, CCPU_GPR[ra].b16.hi, imm4u);
      CCPU_GPR[rt].b16.lo = insn_usat_helper (cpu, CCPU_GPR[ra].b16.lo, imm4u);
      break;
    case 0xdb:			/* umax16 */
      {
	/* Rt[31:16] = (Ra[31:16] > Rb[31:16])? Ra[31:16] : Rb[31:16]
	   Rt[15:0] = (Ra[15:0] > Rb[15:0]) ? Ra[15:0] : Rb[15:0] */
	reg_t result;
	result.ub16.hi = (CCPU_GPR[ra].ub16.hi > CCPU_GPR[rb].ub16.hi)
			  ? CCPU_GPR[ra].ub16.hi : CCPU_GPR[rb].ub16.hi;
	result.ub16.lo = (CCPU_GPR[ra].ub16.lo > CCPU_GPR[rb].ub16.lo)
			  ? CCPU_GPR[ra].ub16.lo : CCPU_GPR[rb].ub16.lo;
	CCPU_GPR[rt].u = result.u;
      }
      break;
    case 0xdf:			/* umax8 */
      {
	/* Rt[31:24] = (Ra[31:24] > Rb[31:24])? Ra[31:24] : Rb[31:24]
	   Rt[23:16] = (Ra[23:16] > Rb[23:16])? Ra[23:16] : Rb[23:16]
	   Rt[15:8]  = (Ra[15:8]  > Rb[15:8]) ? Ra[15:8]  : Rb[15:8]
	   Rt[7:0]   = (Ra[7:0]   > Rb[7:0])  ? Ra[7:0]   : Rb[7:0]  */
	reg_t result;
	result.ub8.b3 = (CCPU_GPR[ra].ub8.b3 > CCPU_GPR[rb].ub8.b3)
			 ? CCPU_GPR[ra].ub8.b3 : CCPU_GPR[rb].ub8.b3;
	result.ub8.b2 = (CCPU_GPR[ra].ub8.b2 > CCPU_GPR[rb].ub8.b2)
			 ? CCPU_GPR[ra].ub8.b2 : CCPU_GPR[rb].ub8.b2;
	result.ub8.b1 = (CCPU_GPR[ra].ub8.b1 > CCPU_GPR[rb].ub8.b1)
			 ? CCPU_GPR[ra].ub8.b1 : CCPU_GPR[rb].ub8.b1;
	result.ub8.b0 = (CCPU_GPR[ra].ub8.b0 > CCPU_GPR[rb].ub8.b0)
			 ? CCPU_GPR[ra].ub8.b0 : CCPU_GPR[rb].ub8.b0;
	CCPU_GPR[rt].u = result.u;
      }
      break;
    case 0xe0:			/* sra16 */
      {
	reg_t result;
	result.b16.hi = CCPU_GPR[ra].b16.hi >> (CCPU_GPR[rb].u & 0xf);
	result.b16.lo = CCPU_GPR[ra].b16.lo >> (CCPU_GPR[rb].u & 0xf);
	CCPU_GPR[rt].s = result.s;
      }
      break;
    case 0xe1:			/* sra16.u */
      {
	reg_t result;
	uint32_t rnd_mask = (1UL << (CCPU_GPR[rb].u - 1));
	int16_t rnd_val;

	rnd_val = (CCPU_GPR[ra].b16.hi & rnd_mask) ? 1 : 0;
	result.b16.hi = (CCPU_GPR[ra].b16.hi >> (CCPU_GPR[rb].u & 0xf))
			+ rnd_val;

	rnd_val = (CCPU_GPR[ra].b16.lo & rnd_mask) ? 1 : 0;
	result.b16.lo = (CCPU_GPR[ra].b16.lo >> (CCPU_GPR[rb].u & 0xf))
			+ rnd_val;

	CCPU_GPR[rt].s = result.s;
      }
      break;
    case 0xe2:			/* srl16 */
      {
	reg_t result;
	result.ub16.hi = CCPU_GPR[ra].ub16.hi >> (CCPU_GPR[rb].u & 0xf);
	result.ub16.lo = CCPU_GPR[ra].ub16.lo >> (CCPU_GPR[rb].u & 0xf);
	CCPU_GPR[rt].s = result.s;
      }
      break;
    case 0xe3:			/* srl16.u */
      {
	reg_t result;
	uint32_t rnd_mask = (1UL << (CCPU_GPR[rb].u - 1));
	int32_t rnd_val;

	rnd_val = (CCPU_GPR[ra].ub16.hi & rnd_mask) ? 1 : 0;
	result.ub16.hi = (CCPU_GPR[ra].ub16.hi >> (CCPU_GPR[rb].u & 0xf))
			 + rnd_val;

	rnd_val = (CCPU_GPR[ra].ub16.lo & rnd_mask) ? 1 : 0;
	result.ub16.lo = (CCPU_GPR[ra].ub16.lo >> (CCPU_GPR[rb].u & 0xf))
			 + rnd_val;

	CCPU_GPR[rt].s = result.s;
      }
      break;
    case 0xe4:			/* sll16 */
      {
	reg_t result;
	result.b16.hi = CCPU_GPR[ra].b16.hi << (CCPU_GPR[rb].u & 0xf);
	result.b16.lo = CCPU_GPR[ra].b16.lo << (CCPU_GPR[rb].u & 0xf);
	CCPU_GPR[rt].s = result.s;
      }
      break;
    case 0xe5:			/* kslra16 */
      {
	if (CCPU_GPR[rb].b8.b0 < 0)
	  {
	    CCPU_GPR[rt].b16.hi = CCPU_GPR[ra].b16.hi >> (-CCPU_GPR[rb].b8.b0);
	    CCPU_GPR[rt].b16.lo = CCPU_GPR[ra].b16.lo >> (-CCPU_GPR[rb].b8.b0);
	  }
	else
	  {
	    int32_t res1, res2;
	    if (CCPU_GPR[rb].b8.b0 != 0)
	      {
		res1 = (int32_t) CCPU_GPR[ra].b16.hi << CCPU_GPR[rb].b8.b0;
		res2 = (int32_t) CCPU_GPR[ra].b16.lo << CCPU_GPR[rb].b8.b0;
		res1 = insn_sat_helper (cpu, res1, 15);
		res2 = insn_sat_helper (cpu, res2, 15);
		CCPU_GPR[rt].b16.hi = res1;
		CCPU_GPR[rt].b16.lo = res2;
	      }
	    else
	      CCPU_GPR[rt].s = CCPU_GPR[ra].s;
	  }
      }
      break;
    case 0xe6:			/* kslra16.u */
      {
	if (CCPU_GPR[rb].b8.b0 < 0)
	  {
	    int rnd;
	    uint32_t mask_sh;
	    int sh = -CCPU_GPR[rb].b8.b0;
	    sh = (sh == 16) ? 15: sh;
	    mask_sh = 1UL << (sh - 1);

	    rnd = (CCPU_GPR[ra].b16.hi & mask_sh) ? 1 : 0;
	    CCPU_GPR[rt].b16.hi = CCPU_GPR[ra].b16.hi >> sh;
	    CCPU_GPR[rt].b16.hi += rnd;
	    rnd = (CCPU_GPR[ra].b16.lo & mask_sh) ? 1 : 0;
	    CCPU_GPR[rt].b16.lo = CCPU_GPR[ra].b16.lo >> sh;
	    CCPU_GPR[rt].b16.lo += rnd;
	  }
	else
	  {
	    int32_t res1, res2;
	    if (CCPU_GPR[rb].b8.b0 != 0)
	      {
		res1 = (int32_t) CCPU_GPR[ra].b16.hi << CCPU_GPR[rb].b8.b0;
		res2 = (int32_t) CCPU_GPR[ra].b16.lo << CCPU_GPR[rb].b8.b0;
		res1 = insn_sat_helper (cpu, res1, 15);
		res2 = insn_sat_helper (cpu, res2, 15);
		CCPU_GPR[rt].b16.hi = res1;
		CCPU_GPR[rt].b16.lo = res2;
	      }
	    else
	      CCPU_GPR[rt].s = CCPU_GPR[ra].s;
	  }
      }
      break;
    case 0xe7:			/* sra.u */
      {
	uint32_t rnd_mask = (1UL << (CCPU_GPR[rb].u - 1));
	int32_t rnd_val = (CCPU_GPR[ra].s & rnd_mask) ? 1 : 0;
	CCPU_GPR[rt].s = (CCPU_GPR[ra].s >> (CCPU_GPR[rb].u & 0x1f)) + rnd_val ;
      }
      break;
    case 0xe8:			/* srai16 */
      {
	reg_t result;
	result.b16.hi = CCPU_GPR[ra].b16.hi >> imm4u;
	result.b16.lo = CCPU_GPR[ra].b16.lo >> imm4u;
	CCPU_GPR[rt].s = result.s;
      }
      break;
    case 0xe9:			/* srai16.u */
      {
	reg_t result;
	uint32_t rnd_mask = (1UL << (imm4u - 1));
	int16_t rnd_val;

	rnd_val = (CCPU_GPR[ra].b16.hi & rnd_mask) ? 1 : 0;
	result.b16.hi = (CCPU_GPR[ra].b16.hi >> imm4u) + rnd_val;

	rnd_val = (CCPU_GPR[ra].b16.lo & rnd_mask) ? 1 : 0;
	result.b16.lo = (CCPU_GPR[ra].b16.lo >> imm4u) + rnd_val;

	CCPU_GPR[rt].s = result.s;
      }
      break;
    case 0xea:			/* srli16 */
      {
	reg_t result;
	result.ub16.hi = CCPU_GPR[ra].ub16.hi >> imm4u;
	result.ub16.lo = CCPU_GPR[ra].ub16.lo >> imm4u;
	CCPU_GPR[rt].s = result.s;
      }
      break;
    case 0xeb:			/* srli16.u */
      {
	reg_t result;
	uint32_t rnd_mask = (1UL << (imm4u - 1));
	int32_t rnd_val;

	rnd_val = (CCPU_GPR[ra].ub16.hi & rnd_mask) ? 1 : 0;
	result.ub16.hi = (CCPU_GPR[ra].ub16.hi >> imm4u) + rnd_val;

	rnd_val = (CCPU_GPR[ra].ub16.lo & rnd_mask) ? 1 : 0;
	result.ub16.lo = (CCPU_GPR[ra].ub16.lo >> imm4u) + rnd_val;

	CCPU_GPR[rt].s = result.s;
      }
      break;
    case 0xec:			/* slli16 */
      {
	reg_t result;
	result.b16.hi = CCPU_GPR[ra].b16.hi << imm4u;
	result.b16.lo = CCPU_GPR[ra].b16.lo << imm4u;
	CCPU_GPR[rt].s = result.s;
      }
      break;
    case 0xed:			/* kslli16 */
      {
	int32_t res1, res2;

	if (imm4u != 0)
	  {
	    res1 = (int32_t) CCPU_GPR[ra].b16.hi << imm4u;
	    res2 = (int32_t) CCPU_GPR[ra].b16.lo << imm4u;
	    res1 = insn_sat_helper (cpu, res1, 15);
	    res2 = insn_sat_helper (cpu, res2, 15);
	    CCPU_GPR[rt].b16.hi = res1;
	    CCPU_GPR[rt].b16.lo = res2;
	  }
	else
	  CCPU_GPR[rt].s = CCPU_GPR[ra].s;
      }
      break;
    case 0xee:			/* kslli */
      {
	int64_t res;

	if (imm5u != 0)
	  {
	    res = (int64_t) CCPU_GPR[ra].s << imm5u;
	    res = insn_sat_helper (cpu, res, 31);
	    CCPU_GPR[rt].s = res;
	  }
	else
	  CCPU_GPR[rt].s = CCPU_GPR[ra].s;
      }
      break;
    case 0xef:			/* srai.u */
      {
	uint32_t rnd_mask = (1UL << (imm5u - 1));
	int32_t rnd_val = (CCPU_GPR[ra].s & rnd_mask) ? 1 : 0;
	CCPU_GPR[rt].s = (CCPU_GPR[ra].s >> imm5u) + rnd_val ;
      }
      break;
    case 0xf0:			/* cmpeq16 */
      {
	reg_t result;

	result.u = 0;
	if (CCPU_GPR[ra].b16.hi == CCPU_GPR[rb].b16.hi)
	  result.ub16.hi = 0xffff;
	if (CCPU_GPR[ra].b16.lo == CCPU_GPR[rb].b16.lo)
	  result.ub16.lo = 0xffff;

	CCPU_GPR[rt].s = result.u;
      }
      break;
    case 0xf1:			/* scmplt16 */
      {
	reg_t result;

	result.u = 0;
	if (CCPU_GPR[ra].b16.hi < CCPU_GPR[rb].b16.hi)
	  result.ub16.hi = 0xffff;
	if (CCPU_GPR[ra].b16.lo < CCPU_GPR[rb].b16.lo)
	  result.ub16.lo = 0xffff;

	CCPU_GPR[rt].s = result.u;
      }
      break;
    case 0xf2:			/* scmple16 */
      {
	reg_t result;

	result.u = 0;
	if (CCPU_GPR[ra].b16.hi <= CCPU_GPR[rb].b16.hi)
	  result.ub16.hi = 0xffff;
	if (CCPU_GPR[ra].b16.lo <= CCPU_GPR[rb].b16.lo)
	  result.ub16.lo = 0xffff;

	CCPU_GPR[rt].s = result.u;
      }
      break;
    case 0xf3:			/* smin16 */
      {
	reg_t result;
	result.b16.hi = (CCPU_GPR[ra].b16.hi < CCPU_GPR[rb].b16.hi)
			 ? CCPU_GPR[ra].b16.hi : CCPU_GPR[rb].b16.hi;
	result.b16.lo = (CCPU_GPR[ra].b16.lo < CCPU_GPR[rb].b16.lo)
			 ? CCPU_GPR[ra].b16.lo : CCPU_GPR[rb].b16.lo;
	CCPU_GPR[rt].s = result.s;
      }
      break;
    case 0xf4:			/* cmpeq8 */
      {
	reg_t result;

	result.u = 0;
	if (CCPU_GPR[ra].b8.b3 == CCPU_GPR[rb].b8.b3)
	  result.ub8.b3 = 0xff;
	if (CCPU_GPR[ra].b8.b2 == CCPU_GPR[rb].b8.b2)
	  result.ub8.b2 = 0xff;
	if (CCPU_GPR[ra].b8.b1 == CCPU_GPR[rb].b8.b1)
	  result.ub8.b1 = 0xff;
	if (CCPU_GPR[ra].b8.b0 == CCPU_GPR[rb].b8.b0)
	  result.ub8.b0 = 0xff;

	CCPU_GPR[rt].s = result.u;
      }
      break;
    case 0xf5:			/* scmplt8 */
      {
	reg_t result;

	result.u = 0;
	if (CCPU_GPR[ra].b8.b3 < CCPU_GPR[rb].b8.b3)
	  result.ub8.b3 = 0xff;
	if (CCPU_GPR[ra].b8.b2 < CCPU_GPR[rb].b8.b2)
	  result.ub8.b2 = 0xff;
	if (CCPU_GPR[ra].b8.b1 < CCPU_GPR[rb].b8.b1)
	  result.ub8.b1 = 0xff;
	if (CCPU_GPR[ra].b8.b0 < CCPU_GPR[rb].b8.b0)
	  result.ub8.b0 = 0xff;

	CCPU_GPR[rt].s = result.u;
      }
      break;
    case 0xf6:			/* scmple8 */
      {
	reg_t result;

	result.u = 0;
	if (CCPU_GPR[ra].b8.b3 <= CCPU_GPR[rb].b8.b3)
	  result.ub8.b3 = 0xff;
	if (CCPU_GPR[ra].b8.b2 <= CCPU_GPR[rb].b8.b2)
	  result.ub8.b2 = 0xff;
	if (CCPU_GPR[ra].b8.b1 <= CCPU_GPR[rb].b8.b1)
	  result.ub8.b1 = 0xff;
	if (CCPU_GPR[ra].b8.b0 <= CCPU_GPR[rb].b8.b0)
	  result.ub8.b0 = 0xff;

	CCPU_GPR[rt].s = result.u;
      }
      break;
    case 0xf7:			/* smin8 */
      {
	reg_t result;
	result.b8.b3 = (CCPU_GPR[ra].b8.b3 < CCPU_GPR[rb].b8.b3)
			? CCPU_GPR[ra].b8.b3 : CCPU_GPR[rb].b8.b3;
	result.b8.b2 = (CCPU_GPR[ra].b8.b2 < CCPU_GPR[rb].b8.b2)
			? CCPU_GPR[ra].b8.b2 : CCPU_GPR[rb].b8.b2;
	result.b8.b1 = (CCPU_GPR[ra].b8.b1 < CCPU_GPR[rb].b8.b1)
			? CCPU_GPR[ra].b8.b1 : CCPU_GPR[rb].b8.b1;
	result.b8.b0 = (CCPU_GPR[ra].b8.b0 < CCPU_GPR[rb].b8.b0)
			? CCPU_GPR[ra].b8.b0 : CCPU_GPR[rb].b8.b0;
	CCPU_GPR[rt].s = result.s;
      }
      break;
    case 0xf9:			/* ucmplt16 */
      {
	reg_t result;

	result.u = 0;
	if (CCPU_GPR[ra].ub16.hi < CCPU_GPR[rb].ub16.hi)
	  result.ub16.hi = 0xffff;
	if (CCPU_GPR[ra].ub16.lo < CCPU_GPR[rb].ub16.lo)
	  result.ub16.lo = 0xffff;

	CCPU_GPR[rt].u = result.u;
      }
      break;
    case 0xfa:			/* ucmple16 */
      {
	reg_t result;

	result.u = 0;
	if (CCPU_GPR[ra].ub16.hi < CCPU_GPR[rb].ub16.hi)
	  result.ub16.hi = 0xffff;
	if (CCPU_GPR[ra].ub16.lo < CCPU_GPR[rb].ub16.lo)
	  result.ub16.lo = 0xffff;

	CCPU_GPR[rt].u = result.u;
      }
      break;
    case 0xfb:			/* umin16 */
      {
	reg_t result;
	result.ub16.hi = (CCPU_GPR[ra].ub16.hi < CCPU_GPR[rb].ub16.hi)
			  ? CCPU_GPR[ra].ub16.hi : CCPU_GPR[rb].ub16.hi;
	result.ub16.lo = (CCPU_GPR[ra].ub16.lo < CCPU_GPR[rb].ub16.lo)
			  ? CCPU_GPR[ra].ub16.lo : CCPU_GPR[rb].ub16.lo;
	CCPU_GPR[rt].u = result.u;
      }
      break;
    case 0xfd:			/* ucmplt8 */
      {
	reg_t result;

	result.u = 0;
	if (CCPU_GPR[ra].ub8.b3 < CCPU_GPR[rb].ub8.b3)
	  result.ub8.b3 = 0xff;
	if (CCPU_GPR[ra].ub8.b2 < CCPU_GPR[rb].ub8.b2)
	  result.ub8.b2 = 0xff;
	if (CCPU_GPR[ra].ub8.b1 < CCPU_GPR[rb].ub8.b1)
	  result.ub8.b1 = 0xff;
	if (CCPU_GPR[ra].ub8.b0 < CCPU_GPR[rb].ub8.b0)
	  result.ub8.b0 = 0xff;

	CCPU_GPR[rt].u = result.u;
      }
      break;
    case 0xfe:			/* ucmple8 */
      {
	reg_t result;

	result.u = 0;
	if (CCPU_GPR[ra].ub8.b3 <= CCPU_GPR[rb].ub8.b3)
	  result.ub8.b3 = 0xff;
	if (CCPU_GPR[ra].ub8.b2 <= CCPU_GPR[rb].ub8.b2)
	  result.ub8.b2 = 0xff;
	if (CCPU_GPR[ra].ub8.b1 <= CCPU_GPR[rb].ub8.b1)
	  result.ub8.b1 = 0xff;
	if (CCPU_GPR[ra].ub8.b0 <= CCPU_GPR[rb].ub8.b0)
	  result.ub8.b0 = 0xff;

	CCPU_GPR[rt].u = result.u;
      }
      break;
    case 0xff:			/* umin8 */
      {
	reg_t result;
	result.ub8.b3 = (CCPU_GPR[ra].ub8.b3 < CCPU_GPR[rb].ub8.b3)
			 ? CCPU_GPR[ra].ub8.b3 : CCPU_GPR[rb].ub8.b3;
	result.ub8.b2 = (CCPU_GPR[ra].ub8.b2 < CCPU_GPR[rb].ub8.b2)
			 ? CCPU_GPR[ra].ub8.b2 : CCPU_GPR[rb].ub8.b2;
	result.ub8.b1 = (CCPU_GPR[ra].ub8.b1 < CCPU_GPR[rb].ub8.b1)
			 ? CCPU_GPR[ra].ub8.b1 : CCPU_GPR[rb].ub8.b1;
	result.ub8.b0 = (CCPU_GPR[ra].ub8.b0 < CCPU_GPR[rb].ub8.b0)
			 ? CCPU_GPR[ra].ub8.b0 : CCPU_GPR[rb].ub8.b0;
	CCPU_GPR[rt].u = result.u;
      }
      break;
    default:
      nds32_bad_op (cpu, cia, insn, "ALU2");
      return;
    }

  return;
}

static void
nds32_decode32_jreg (sim_cpu *cpu, const uint32_t insn, sim_cia cia)
{
  const SIM_DESC sd = CPU_STATE (cpu);
  const int rt = N32_RT5 (insn);
  const int ra = N32_RA5 (insn);
  const int rb = N32_RB5 (insn);
  sim_cia nia;

  if (ra != 0)
    sim_io_error (sd, "JREG RA == %d at pc=0x%x, code=0x%08x\n",
		  ra, cia, insn);

  if (__GF (insn, 8, 2) != 0)
    sim_io_error (sd, "JREG DT/IT not supported at pc=0x%x, code=0x%08x\n",
		  cia, insn);

  switch (N32_SUB5 (insn))
    {
    case 0:			/* jr, ifret, ret */
      if (__GF (insn, 5, 2) == 0x3)
	{
	  /* ifret. IFC + RET */
	  if (CCPU_SR_TEST (PSW, PSW_IFCON))
	    cia = CCPU_USR[USR0_IFCLP].u;
	  else
	    return;		/* Do nothing. (ifret) */
	}
      else
	/* jr or ret */
	cia = CCPU_GPR[rb].u;

      CCPU_SR_CLEAR (PSW, PSW_IFCON);
      nds32_set_nia (cpu, cia);
      return;

    case 1:			/* jral */
      if (cpu->iflags & NIF_EX9)
	CCPU_GPR[rt].u = cia + 2;
      else
	CCPU_GPR[rt].u = cia + 4;

      cia = CCPU_GPR[rb].u;
      /* If PSW.IFCON, it returns to $ifc_lp instead.  */
      if (CCPU_SR_TEST (PSW, PSW_IFCON))
	CCPU_GPR[rt] = CCPU_USR[USR0_IFCLP];

      CCPU_SR_CLEAR (PSW, PSW_IFCON);
      nds32_set_nia (cpu, cia);
      return;

    case 2:			/* jrnez */
      if (CCPU_GPR[rb].u == 0)
	return;			/* NOT taken */

      /* PSW.IFCON is only cleared when taken.  */
      CCPU_SR_CLEAR (PSW, PSW_IFCON);
      nds32_set_nia (cpu, CCPU_GPR[rb].u);
      return;

    case 3:			/* jralnez */
      /* Prevent early clobbing of rb (rt == rb).  */
      nia = CCPU_GPR[rb].u;

      /* Rt is always set according to spec.  */
      if (cpu->iflags & NIF_EX9)
	CCPU_GPR[rt].u = cia + 2;
      else
	CCPU_GPR[rt].u = cia + 4;

      /* By spec, PSW.IFCON is always cleared no matter it takes or not.  */
      if (CCPU_SR_TEST (PSW, PSW_IFCON))
	CCPU_GPR[rt] = CCPU_USR[USR0_IFCLP];
      CCPU_SR_CLEAR (PSW, PSW_IFCON);

      if (nia != 0)		/* taken branch */
	nds32_set_nia (cpu, nia);

      return;

    default:
      nds32_bad_op (cpu, cia, insn, "JREG");
      return;
    }

  return;
}

static void
nds32_decode32_br1 (sim_cpu *cpu, const uint32_t insn, sim_cia cia)
{
  const int rt = N32_RT5 (insn);
  const int ra = N32_RA5 (insn);
  const int imm14s = N32_IMM14S (insn);

  switch ((insn >> 14) & 1)
    {
    case 0:			/* beq */
      if (CCPU_GPR[rt].u == CCPU_GPR[ra].u)
	{
	  CCPU_SR_CLEAR (PSW, PSW_IFCON);
	  nds32_set_nia (cpu, cia + (imm14s << 1));
	}
      break;
    case 1:			/* bne */
      if (CCPU_GPR[rt].u != CCPU_GPR[ra].u)
	{
	  CCPU_SR_CLEAR (PSW, PSW_IFCON);
	  nds32_set_nia (cpu, cia + (imm14s << 1));
	}
      break;
    }
}

static void
nds32_decode32_br2 (sim_cpu *cpu, const uint32_t insn, sim_cia cia)
{
  const SIM_DESC sd = CPU_STATE (cpu);
  const int rt = N32_RT5 (insn);
  const int imm16s1 = N32_IMM16S (insn) << 1;

  switch (__GF (insn, 16, 4))
    {
    case 0x0:			/* mtlbi, mtlei, ifcall */
      if (__GF (insn, 20, 2) == 1)
	{
	  /* mtlbi */
          CCPU_USR[USR0_LB].u = cia + imm16s1;
	}
      else if (__GF (insn, 20, 2) == 2)
	{
	  /* mtlei */
          CCPU_USR[USR0_LE].u = cia + imm16s1;
	}
      else
	{
	  /* Do not set $ifc_lp when chaining ifcall.  */
	  if (!CCPU_SR_TEST (PSW, PSW_IFCON))
	    {
	      if (cpu->iflags & NIF_EX9)
	        CCPU_USR[USR0_IFCLP].u = cia + 2;
	      else
	        CCPU_USR[USR0_IFCLP].u = cia + 4;
	    }
	  nds32_set_nia (cpu, cia + imm16s1);
	  CCPU_SR_SET (PSW, PSW_IFCON);
	}
      break;
    case 0x2:			/* beqz */
      if (CCPU_GPR[rt].s == 0)
	{
	  CCPU_SR_CLEAR (PSW, PSW_IFCON);
	  nds32_set_nia (cpu, cia + imm16s1);
	}
      break;
    case 0x3:			/* bnez */
      if (CCPU_GPR[rt].s != 0)
	{
	  CCPU_SR_CLEAR (PSW, PSW_IFCON);
	  nds32_set_nia (cpu, cia + imm16s1);
	}
      break;
    case 0x4:			/* bgez */
      if (CCPU_GPR[rt].s >= 0)
	{
	  CCPU_SR_CLEAR (PSW, PSW_IFCON);
	  nds32_set_nia (cpu, cia + imm16s1);
	}
      break;
    case 0x5:			/* bltz */
      if (CCPU_GPR[rt].s < 0)
	{
	  CCPU_SR_CLEAR (PSW, PSW_IFCON);
	  nds32_set_nia (cpu, cia + imm16s1);
	}
      break;
    case 0x6:			/* bgtz */
      if (CCPU_GPR[rt].s > 0)
	{
	  CCPU_SR_CLEAR (PSW, PSW_IFCON);
	  nds32_set_nia (cpu, cia + imm16s1);
	}
      break;
    case 0x7:			/* blez */
      if (CCPU_GPR[rt].s <= 0)
	{
	  CCPU_SR_CLEAR (PSW, PSW_IFCON);
	  nds32_set_nia (cpu, cia + imm16s1);
	}
      break;
    case 0xc:			/* bgezal */
      /* Always clob $lp.  */
      if (cpu->iflags & NIF_EX9)
	CCPU_GPR[GPR_LP].u = cia + 2;
      else
	CCPU_GPR[GPR_LP].u = cia + 4;

      /* Always set $lp = $ifc_lp no matter it takes no not.  */
      if (CCPU_SR_TEST (PSW, PSW_IFCON))
	CCPU_GPR[GPR_LP].u = CCPU_USR[USR0_IFCLP].u;

      /* PSW.IFCON is only cleared when the branch is taken.  */
      if (!(CCPU_GPR[rt].s >= 0))
	return;

      CCPU_SR_CLEAR (PSW, PSW_IFCON);
      nds32_set_nia (cpu, cia + imm16s1);
      return;
    case 0xd:			/* bltzal */
      /* Always clob $lp.  */
      if (cpu->iflags & NIF_EX9)
	CCPU_GPR[GPR_LP].u = cia + 2;
      else
	CCPU_GPR[GPR_LP].u = cia + 4;

      /* Always set $lp = $ifc_lp no matter it takes no not.  */
      if (CCPU_SR_TEST (PSW, PSW_IFCON))
	CCPU_GPR[GPR_LP].u = CCPU_USR[USR0_IFCLP].u;

      /* PSW.IFCON is only cleared when the branch is taken.  */
      if (!(CCPU_GPR[rt].s < 0))
	return;

      CCPU_SR_CLEAR (PSW, PSW_IFCON);
      nds32_set_nia (cpu, cia + imm16s1);
      break;
    default:
      nds32_bad_op (cpu, cia, insn, "BR2");
      break;
    }
}

static void
nds32_pfm_ctl (sim_cpu *cpu)
{
  int en, ie, ovf, ks, ku;
  int sel0, sel1, sel2;

  en = CCPU_SR_GET (PFM_CTL, PFM_CTL_EN);
  ie = CCPU_SR_GET (PFM_CTL, PFM_CTL_IE);
  ovf = CCPU_SR_GET (PFM_CTL, PFM_CTL_OVF);
  ks = CCPU_SR_GET (PFM_CTL, PFM_CTL_KS);
  ku = CCPU_SR_GET (PFM_CTL, PFM_CTL_KU);
  sel0 = CCPU_SR_GET (PFM_CTL, PFM_CTL_SEL0);
  sel1 = CCPU_SR_GET (PFM_CTL, PFM_CTL_SEL1);
  sel2 = CCPU_SR_GET (PFM_CTL, PFM_CTL_SEL2);
}

static void
nds32_pfm_event (sim_cpu *cpu, int pfm_event)
{
  int sel[3];
  int en, ovf;
  int i;

  en = CCPU_SR_GET (PFM_CTL, PFM_CTL_EN);
  ovf = CCPU_SR_GET (PFM_CTL, PFM_CTL_OVF);

  sel[0] = CCPU_SR_GET (PFM_CTL, PFM_CTL_SEL0);
  sel[1] = CCPU_SR_GET (PFM_CTL, PFM_CTL_SEL1);
  sel[2] = CCPU_SR_GET (PFM_CTL, PFM_CTL_SEL2);

  switch (pfm_event)
    {
    case PFM_CYCLE:
    case PFM_INST:
      for (i = 0; i < 3; i++)
	{
	  if (sel[i] == pfm_event && __TEST (en, i))
	    {
	      CCPU_SR[SRIDX_PFMC0 + i].u++;
	      if (CCPU_SR[SRIDX_PFMC0 + i].u == 0)
		ovf |= (1 << i);
	    }
	}
      break;
    }

  CCPU_SR_PUT (PFM_CTL, PFM_CTL_OVF, ovf);
}

static void
nds32_decode32_misc (sim_cpu *cpu, const uint32_t insn, sim_cia cia)
{
  const int rt = N32_RT5 (insn);
  const int ra = N32_RA5 (insn);
  const int rb = N32_RB5 (insn);
  const int rc = N32_RD5 (insn);

  switch (insn & 0x1F)
    {
    case 0x0:			/* standby */
    case 0x1:			/* cctl */
    case 0x8:			/* dsb */
    case 0x9:			/* isb */
    case 0xd:			/* isync */
    case 0xc:			/* msync */
      break;
    case 0x5:			/* trap */
    case 0xa:			/* break */
      nds32_raise_exception (cpu, EXP_DEBUG, SIM_SIGTRAP, NULL);
      return;
    case 0x2:			/* mfsr */
      CCPU_GPR[rt] = CCPU_SR[__GF (insn, 10, 10)];
      break;
    case 0x3:			/* mtsr */
      {
	int sridx = __GF (insn, 10, 10);

	switch (__GF (insn, 5, 5))
	  {
	  case 0:		/* mtsr */
	    CCPU_SR[sridx] = CCPU_GPR[rt];
	    switch (sridx)
	      {
	      case SRIDX_PFM_CTL:
		nds32_pfm_ctl (cpu);
		break;
	      }
	    break;
	  case 1:		/* setend */
	    if (sridx != 0x80)
	      nds32_bad_op (cpu, cia, insn, "SETEND (sridx)");

	    if (rt == 1)
	      CCPU_SR_SET (PSW, PSW_BE);
	    else if (rt == 0)
	      CCPU_SR_CLEAR (PSW, PSW_BE);
	    else
	      nds32_bad_op (cpu, cia, insn, "SETEND (BE/LE)");
	    break;
	  case 2:		/* setgie */
	    if (sridx != 0x80)
	      nds32_bad_op (cpu, cia, insn, "SETGIE (sridx)");

	    if (rt == 1)
	      CCPU_SR_SET (PSW, PSW_GIE);
	    else if (rt == 0)
	      CCPU_SR_CLEAR (PSW, PSW_GIE);
	    else
	      nds32_bad_op (cpu, cia, insn, "SETEND (BE/LE)");
	    break;
	  }
      }
      break;
    case 0xb:			/* syscall */
      nds32_syscall (cpu, __GF (insn, 5, 15), cia);
      break;
    case 0x4:			/* iret */
      nds32_bad_op (cpu, cia, insn, "iret (MISC)");
      break;
    case 0x6:			/* teqz */
      nds32_bad_op (cpu, cia, insn, "teqz (MISC)");
      break;
    case 0x7:			/* tnez */
      nds32_bad_op (cpu, cia, insn, "tnez (MISC)");
      break;
    case 0xe:			/* tlbop */
      nds32_bad_op (cpu, cia, insn, "tlbop (MISC)");
      break;
    case 0x10:			/* bpick */
      {
	uint32_t temp_ctl = ~CCPU_GPR[rc].u;
	CCPU_GPR[rt].u = (uint32_t) ((CCPU_GPR[ra].u & ~temp_ctl)
				     | (CCPU_GPR[rb].u & temp_ctl));
      }
      break;
    default:
      nds32_bad_op (cpu, cia, insn, "MISC");
      break;
    }
}

static void
nds32_decode32_simd (sim_cpu *cpu, const uint32_t insn, sim_cia cia)
{
  int rt = N32_RT5 (insn);
  int ra = N32_RA5 (insn);
  int rb = N32_RB5 (insn);
  int a, b, c, d;

  switch (insn & 0x3ff)
    {
    case 0x0:			/* pbsad */
    case 0x1:			/* pbsada */
      /* The four unsigned 8-bit elements of Ra are subtracted from the four
	 unsigned 8-bit elements of Rb.  */
      a = (CCPU_GPR[ra].u & 0xff) - (CCPU_GPR[rb].u & 0xff);
      b = ((CCPU_GPR[ra].u >> 8) & 0xff) - ((CCPU_GPR[rb].u >> 8) & 0xff);
      c = ((CCPU_GPR[ra].u >> 16) & 0xff) - ((CCPU_GPR[rb].u >> 16) & 0xff);
      d = ((CCPU_GPR[ra].u >> 24) & 0xff) - ((CCPU_GPR[rb].u >> 24) & 0xff);

      /* Absolute difference of four unsigned 8-bit data elements.  */
      a = (a >= 0) ? a : -a;
      b = (b >= 0) ? b : -b;
      c = (c >= 0) ? c : -c;
      d = (d >= 0) ? d : -d;

      if ((insn & 0x3ff) == 0x0)
	/* pbsad */
	CCPU_GPR[rt].u = a + b + c + d;
      else
	/* pbsada */
        CCPU_GPR[rt].u = CCPU_GPR[rt].u + a + b + c + d;
      break;
    default:
      nds32_bad_op (cpu, cia, insn, "MISC");
      break;
    }
}

static void
nds32_decode32 (sim_cpu *cpu, const uint32_t insn, sim_cia cia)
{
  const SIM_DESC sd = CPU_STATE (cpu);
  const int op = N32_OP6 (insn);
  const int rt = N32_RT5 (insn);
  const int ra = N32_RA5 (insn);
  const int imm15s = N32_IMM15S (insn);
  const int imm15u = N32_IMM15U (insn);
  uint32_t shift;
  uint32_t addr;
  sim_cia next_cia;

  switch (op)
    {
    case 0x0:			/* lbi */
    case 0x1:			/* lhi */
    case 0x2:			/* lwi */
    /* case 0x3: */		/* ldi */
      {
	shift = (op - 0x0);
	addr = CCPU_GPR[ra].u + (imm15s << shift);
	CCPU_GPR[rt].u = nds32_ld_aligned (cpu, addr, 1 << shift);
      }
      break;

    case 0x4:			/* lbi.bi */
    case 0x5:			/* lhi.bi */
    case 0x6:			/* lwi.bi */
    /* case 0x7: */		/* ldi.bi */
      {
	shift = (op - 0x4);
	CCPU_GPR[rt].u = nds32_ld_aligned (cpu, CCPU_GPR[ra].u, 1 << shift);
	CCPU_GPR[ra].u += (imm15s << shift);
      }
      break;

    case 0x8:			/* sbi */
    case 0x9:			/* shi */
    case 0xa:			/* swi */
    /* case 0xb: */		/* sdi */
      {
	shift = (op - 0x8);
	addr = CCPU_GPR[ra].u + (imm15s << shift);
	nds32_st_aligned (cpu, addr, 1 << shift, CCPU_GPR[rt].u);
      }
      break;

    case 0xc:			/* sbi.bi */
    case 0xd:			/* shi.bi */
    case 0xe:			/* swi.bi */
    /* case 0xf: */		/* sdi.bi */
      {
	shift = (op - 0xc);
	nds32_st_aligned (cpu, CCPU_GPR[ra].u, 1 << shift, CCPU_GPR[rt].u);
	CCPU_GPR[ra].u += (imm15s << shift);
      }
      break;

    case 0x10:			/* lbsi */
    case 0x11:			/* lhsi */
    /* case 0x12: */		/* lwsi */
      {
	shift = (op - 0x10);
	addr = CCPU_GPR[ra].u + (imm15s << shift);
	CCPU_GPR[rt].u = nds32_ld_aligned (cpu, addr, 1 << shift);
	CCPU_GPR[rt].u = __SEXT (CCPU_GPR[rt].u, (1 << shift) * 8);
      }
      break;
    case 0x13:			/* dprefi */
      /* do nothing */
      break;
    case 0x14:			/* lbsi.bi */
    case 0x15:			/* lhsi.bi */
    /* case 0x16: */		/* lwsi.bi */
      {
	shift = (op - 0x14);
	CCPU_GPR[rt].u = nds32_ld_aligned (cpu, CCPU_GPR[ra].u, 1 << shift);
	CCPU_GPR[rt].u = __SEXT (CCPU_GPR[rt].u, (1 << shift) * 8);
	CCPU_GPR[ra].u += (imm15s << shift);
      }
      break;
    case 0x17:			/* LBGP */
      if (__TEST (insn, 19))	/* lbsi.gp */
	{
	  addr = CCPU_GPR[GPR_GP].u + N32_IMMS (insn, 19);
	  CCPU_GPR[rt].u = nds32_ld_aligned (cpu, addr, 1);
	  CCPU_GPR[rt].u = __SEXT (CCPU_GPR[rt].u, 1 * 8);
	}
      else			/* lbi.gp */
	CCPU_GPR[rt].u =
	  nds32_ld_aligned (cpu, CCPU_GPR[GPR_GP].u + N32_IMMS (insn, 19), 1);
      break;
    case 0x18:			/* LWC */
      nds32_decode32_lwc (cpu, insn, cia);
      return;
    case 0x19:			/* SWC */
      nds32_decode32_swc (cpu, insn, cia);
      return;
    case 0x1a:			/* LDC */
      nds32_decode32_ldc (cpu, insn, cia);
      return;
    case 0x1b:			/* SDC */
      nds32_decode32_sdc (cpu, insn, cia);
      return;
    case 0x1c:			/* MEM */
      nds32_decode32_mem (cpu, insn, cia);
      return;
    case 0x1d:			/* LSMW */
      nds32_decode32_lsmw (cpu, insn, cia);
      return;
    case 0x1e:			/* HWGP */
      switch (__GF (insn, 17, 3))
	{
	case 0: case 1:		/* lhi.gp */
	  addr = CCPU_GPR[GPR_GP].u + (N32_IMMS (insn, 18) << 1);
	  CCPU_GPR[rt].u = nds32_ld_aligned (cpu, addr, 2);
	  break;
	case 2: case 3:		/* lhsi.gp */
	  addr = CCPU_GPR[GPR_GP].u + (N32_IMMS (insn, 18) << 1);
	  CCPU_GPR[rt].u = nds32_ld_aligned (cpu, addr, 2);
	  CCPU_GPR[rt].u = __SEXT (CCPU_GPR[rt].u, 2 * 8);
	  break;
	case 4: case 5:		/* shi.gp */
	  nds32_st_aligned (cpu, CCPU_GPR[GPR_GP].u + (N32_IMMS (insn, 18) << 1), 2,
			    CCPU_GPR[rt].u);
	  break;
	case 6:			/* lwi.gp */
	  addr= CCPU_GPR[GPR_GP].u + (N32_IMMS (insn, 17) << 2);
	  CCPU_GPR[rt].u = nds32_ld_aligned (cpu, addr, 4);
	  break;
	case 7:			/* swi.gp */
	  nds32_st_aligned (cpu, CCPU_GPR[GPR_GP].u + (N32_IMMS (insn, 17) << 2),
			    4, CCPU_GPR[rt].u);
	  break;
	}
      break;
    case 0x1f:			/* SBGP */
      if (__TEST (insn, 19))	/* addi.gp */
	CCPU_GPR[rt].s = CCPU_GPR[GPR_GP].u + N32_IMMS (insn, 19);
      else			/* sbi.gp */
	nds32_st_aligned (cpu, CCPU_GPR[GPR_GP].u + N32_IMMS (insn, 19), 1,
			  CCPU_GPR[rt].u & 0xFF);
      break;
    case 0x20:			/* ALU_1 */
      nds32_decode32_alu1 (cpu, insn, cia);
      return;
    case 0x21:			/* ALU_2 */
      nds32_decode32_alu2 (cpu, insn, cia);
      return;
    case 0x22:			/* movi */
      CCPU_GPR[rt].s = N32_IMM20S (insn);
      break;
    case 0x23:			/* sethi */
      CCPU_GPR[rt].u = N32_IMM20U (insn) << 12;
      break;
    case 0x24:			/* ji, jal */
      if (cpu->iflags & NIF_EX9)
	{
	  /* Address in ji/jal is treated as absolute address in ex9.  */
	  if (__TEST (insn, 24))	/* jal in ex9 */
	    CCPU_GPR[GPR_LP].u = cia + 2;
	  next_cia = (cia & 0xff000000) | (N32_IMMU (insn, 24) << 1);
	}
      else
	{
	  if (__TEST (insn, 24))	/* jal */
	    CCPU_GPR[GPR_LP].u = cia + 4;
	  next_cia = cia + (N32_IMMS (insn, 24) << 1);
	}

      if (CCPU_SR_TEST (PSW, PSW_IFCON))
	{
	  if (__TEST (insn, 24))	/* jal */
	    CCPU_GPR[GPR_LP] = CCPU_USR[USR0_IFCLP];
	}

      CCPU_SR_CLEAR (PSW, PSW_IFCON);
      nds32_set_nia (cpu, next_cia);
      return;
    case 0x25:			/* jreg */
      nds32_decode32_jreg (cpu, insn, cia);
      return;
    case 0x26:			/* br1 */
      nds32_decode32_br1 (cpu, insn, cia);
      return;
    case 0x27:			/* br2 */
      nds32_decode32_br2 (cpu, insn, cia);
      return;
    case 0x28:			/* addi rt, ra, imm15s */
      CCPU_GPR[rt].s = CCPU_GPR[ra].s + imm15s;
      break;
    case 0x29:			/* subri */
      CCPU_GPR[rt].s = imm15s - CCPU_GPR[ra].s;
      break;
    case 0x2a:			/* andi */
      CCPU_GPR[rt].u = CCPU_GPR[ra].u & imm15u;
      break;
    case 0x2b:			/* xori */
      CCPU_GPR[rt].u = CCPU_GPR[ra].u ^ imm15u;
      break;
    case 0x2c:			/* ori */
      CCPU_GPR[rt].u = CCPU_GPR[ra].u | imm15u;
      break;
    case 0x2d:			/* br3, beqc, bnec */
      {
	int imm11s = __SEXT (__GF (insn, 8, 11), 11);

	if ((__TEST (insn, 19) == 0) ^ (CCPU_GPR[rt].s != imm11s))
	  {
	    CCPU_SR_CLEAR (PSW, PSW_IFCON);
	    nds32_set_nia (cpu, cia + (N32_IMMS (insn, 8) << 1));
	  }
	return;
      }
      break;
    case 0x2e:			/* slti */
      CCPU_GPR[rt].u = (CCPU_GPR[ra].u < (uint32_t) imm15s) ? 1 : 0;
      break;
    case 0x2f:			/* sltsi */
      CCPU_GPR[rt].u = (CCPU_GPR[ra].s < imm15s) ? 1 : 0;
      break;
    case 0x32:			/* misc */
      nds32_decode32_misc (cpu, insn, cia);
      return;
    case 0x33:			/* bitci */
      CCPU_GPR[rt].u = CCPU_GPR[ra].u & ~imm15u;
      break;
    case 0x35:			/* COP */
      nds32_decode32_cop (cpu, insn, cia);
      return;
    case 0x38:			/* SIMD */
      nds32_decode32_simd (cpu, insn, cia);
      return;
    default:
      nds32_bad_op (cpu, cia, insn, "32-bit");
    }
}

static void
nds32_decode16_ex9 (sim_cpu *cpu, uint32_t insn, sim_cia cia)
{
  /* Set NIF_EX9 so to change how JI/JAL interpreting address.  */

  cpu->iflags |= NIF_EX9;
  nds32_decode32 (cpu, insn, cia);
  cpu->iflags &= ~NIF_EX9;
}

static void
nds32_decode16 (sim_cpu *cpu, uint32_t insn, sim_cia cia)
{
  const SIM_DESC sd = CPU_STATE (cpu);
  const int rt5 = N16_RT5 (insn);
  const int ra5 = N16_RA5 (insn);
  const int rt4 = N16_RT4 (insn);
  const int imm5u = N16_IMM5U (insn);
  const int imm5s = N16_IMM5S (insn);
  const int imm9u = N16_IMM9U (insn);
  const int rt3 = N16_RT3 (insn);
  const int ra3 = N16_RA3 (insn);
  const int rb3 = N16_RB3 (insn);
  const int rt38 = N16_RT38 (insn);
  const int imm3u = rb3;
  uint32_t shift;
  uint32_t addr;
  int tmp;

  switch (__GF (insn, 7, 8))
    {
    case 0xf8:			/* push25 */
      {
	uint32_t smw_adm = 0x3A6F83BC;
	uint32_t res[] = { 6, 8, 10, 14 };
	uint32_t re = __GF (insn, 5, 2);

	smw_adm |= res[re] << 10;
	nds32_decode32_lsmw (cpu, smw_adm, cia);
	CCPU_GPR[GPR_SP].u -= (imm5u << 3);
	if (re >= 1)
	  CCPU_GPR[8].u = cia & 0xFFFFFFFC;
      }
      return;
    case 0xf9:			/* pop25 */
      {
	uint32_t lmw_bim = 0x3A6F8384;
	uint32_t res[] = { 6, 8, 10, 14 };
	uint32_t re = __GF (insn, 5, 2);

	lmw_bim |= res[re] << 10;
	CCPU_GPR[GPR_SP].u += (imm5u << 3);
	nds32_decode32_lsmw (cpu, lmw_bim, cia);
	CCPU_SR_CLEAR (PSW, PSW_IFCON);
	nds32_set_nia (cpu, CCPU_GPR[GPR_LP].u);
      }
      return;
    }

  if (__GF (insn, 8, 7) == 0x7d)	/* movd44 */
    {
      int rt5e = __GF (insn, 4, 4) << 1;
      int ra5e = __GF (insn, 0, 4) << 1;

      CCPU_GPR[rt5e] = CCPU_GPR[ra5e];
      CCPU_GPR[rt5e + 1] = CCPU_GPR[ra5e + 1];
      return;
    }

  switch (__GF (insn, 9, 6))
    {
    case 0x4:			/* add45 */
      CCPU_GPR[rt4].u += CCPU_GPR[ra5].u;
      return;
    case 0x5:			/* sub45 */
      CCPU_GPR[rt4].u -= CCPU_GPR[ra5].u;
      return;
    case 0x6:			/* addi45 */
      CCPU_GPR[rt4].u += imm5u;
      return;
    case 0x7:			/* subi45 */
      CCPU_GPR[rt4].u -= imm5u;
      return;
    case 0x8:			/* srai45 */
      CCPU_GPR[rt4].u = CCPU_GPR[rt4].s >> imm5u;
      return;
    case 0x9:			/* srli45 */
      CCPU_GPR[rt4].u = CCPU_GPR[rt4].u >> imm5u;
      return;
    case 0xa:			/* slli333 */
      CCPU_GPR[rt3].u = CCPU_GPR[ra3].u << imm3u;
      return;
    case 0xc:			/* add333 */
      CCPU_GPR[rt3].u = CCPU_GPR[ra3].u + CCPU_GPR[rb3].u;
      return;
    case 0xd:			/* sub333 */
      CCPU_GPR[rt3].u = CCPU_GPR[ra3].u - CCPU_GPR[rb3].u;
      return;
    case 0xe:			/* addi333 */
      CCPU_GPR[rt3].u = CCPU_GPR[ra3].u + imm3u;
      return;
    case 0xf:			/* subi333 */
      CCPU_GPR[rt3].u = CCPU_GPR[ra3].u - imm3u;
      return;
    case 0x10:			/* lwi333 */
    case 0x12:			/* lhi333 */
    case 0x13:			/* lbi333 */
      {
	int shtbl[] = { 2, -1, 1, 0 };

	shift = shtbl[(__GF (insn, 9, 6) - 0x10)];
	addr = CCPU_GPR[ra3].u + (imm3u << shift);
	CCPU_GPR[rt3].u = nds32_ld_aligned (cpu, addr, 1 << shift);
      }
      return;
    case 0x11:			/* lwi333.bi */
      CCPU_GPR[rt3].u = nds32_ld_aligned (cpu, CCPU_GPR[ra3].u, 4);
      CCPU_GPR[ra3].u += imm3u << 2;
      return;
    case 0x14:			/* swi333 */
    case 0x16:			/* shi333 */
    case 0x17:			/* sbi333 */
      {
	int shtbl[] = { 2, -1, 1, 0 };

	shift = shtbl[(__GF (insn, 9, 6) - 0x14)];
	nds32_st_aligned (cpu, CCPU_GPR[ra3].u + (imm3u << shift),
			  1 << shift, CCPU_GPR[rt3].u);
      }
      return;
    case 0x15:			/* swi333.bi */
      nds32_st_aligned (cpu, CCPU_GPR[ra3].u, 4, CCPU_GPR[rt3].u);
      CCPU_GPR[ra3].u += imm3u << 2;
      return;
    case 0x18:			/* addri36.sp */
      CCPU_GPR[rt3].u = CCPU_GPR[GPR_SP].u + (N16_IMM6U (insn) << 2);
      return;
    case 0x19:			/* lwi45.fe */
      tmp = -((32 - imm5u) << 2); /* imm7n */
      CCPU_GPR[rt4].u = nds32_ld_aligned (cpu, CCPU_GPR[8].u + tmp, 4);
      return;
    case 0x1a:			/* lwi450 */
      CCPU_GPR[rt4].u = nds32_ld_aligned (cpu, CCPU_GPR[ra5].u, 4);
      return;
    case 0x1b:			/* swi450 */
      nds32_st_aligned (cpu, CCPU_GPR[ra5].u, 4, CCPU_GPR[rt4].u);
      return;
    case 0x30:			/* slts45 */
      CCPU_GPR[GPR_TA].u = (CCPU_GPR[rt4].s < CCPU_GPR[ra5].s) ? 1 : 0;
      return;
    case 0x31:			/* slt45 */
      CCPU_GPR[GPR_TA].u = (CCPU_GPR[rt4].u < CCPU_GPR[ra5].u) ? 1 : 0;
      return;
    case 0x32:			/* sltsi45 */
      CCPU_GPR[GPR_TA].u = (CCPU_GPR[rt4].s < imm5u) ? 1 : 0;
      return;
    case 0x33:			/* slti45 */
      CCPU_GPR[GPR_TA].u = (CCPU_GPR[rt4].u < imm5u) ? 1 : 0;
      return;

    case 0x34:			/* beqzs8, bnezs8 */
      if ((__TEST (insn, 8) == 0) ^ (CCPU_GPR[GPR_TA].u != 0))
	{
	  CCPU_SR_CLEAR (PSW, PSW_IFCON);
	  nds32_set_nia (cpu, cia + (N16_IMM8S (insn) << 1));
	}
      return;
    case 0x35:			/* break16, ex9.it */
      if (imm9u < 32)		/* break16 */
	{
	  nds32_raise_exception (cpu, EXP_DEBUG, SIM_SIGTRAP, NULL);
	  return;
	}

      /* ex9.it */
      sim_read (sd, (CCPU_USR[USR0_ITB].u & ~3U) + (imm9u << 2),
		(unsigned char *) &insn, 4);
      insn = extract_unsigned_integer ((unsigned char *) &insn, 4, BIG_ENDIAN);
      nds32_decode16_ex9 (cpu, insn, cia);
      return;
    case 0x3c:			/* ifcall9 */
      if (!CCPU_SR_TEST (PSW, PSW_IFCON))
	CCPU_USR[USR0_IFCLP].u = cia + 2;

      nds32_set_nia (cpu, cia + (N16_IMM9U (insn) << 1));
      CCPU_SR_SET (PSW, PSW_IFCON);
      return;
    case 0x3d:			/* movpi45 */
      CCPU_GPR[rt4].u = imm5u + 16;
      return;
    case 0x3f:			/* MISC33 */
      switch (insn & 0x7)
	{
	case 2:			/* neg33 */
	  CCPU_GPR[rt3].s = -CCPU_GPR[ra3].u;
	  return;
	case 3:			/* not33 */
	  CCPU_GPR[rt3].u = ~CCPU_GPR[ra3].u;
	  return;
	case 4:			/* mul33 */
	  CCPU_GPR[rt3].u = CCPU_GPR[rt3].u * CCPU_GPR[ra3].u;
	  return;
	case 5:			/* xor33 */
	  CCPU_GPR[rt3].u = CCPU_GPR[rt3].u ^ CCPU_GPR[ra3].u;
	  return;
	case 6:			/* and33 */
	  CCPU_GPR[rt3].u = CCPU_GPR[rt3].u & CCPU_GPR[ra3].u;
	  return;
	case 7:			/* or33 */
	  CCPU_GPR[rt3].u = CCPU_GPR[rt3].u | CCPU_GPR[ra3].u;
	  return;
	default:
	  goto bad_op;
	}
      return;
    case 0xb:			/* ... */
      switch (insn & 0x7)
	{
	case 0:			/* zeb33 */
	  CCPU_GPR[rt3].u = CCPU_GPR[ra3].u & 0xff;
	  break;
	case 1:			/* zeh33 */
	  CCPU_GPR[rt3].u = CCPU_GPR[ra3].u & 0xffff;
	  break;
	case 2:			/* seb33 */
	  CCPU_GPR[rt3].s = __SEXT (CCPU_GPR[ra3].s, 8);
	  break;
	case 3:			/* seh33 */
	  CCPU_GPR[rt3].s = __SEXT (CCPU_GPR[ra3].s, 16);
	  break;
	case 4:			/* xlsb33 */
	  CCPU_GPR[rt3].u = CCPU_GPR[ra3].u & 0x1;
	  break;
	case 5:			/* x11b33 */
	  CCPU_GPR[rt3].u = CCPU_GPR[ra3].u & 0x7FF;
	  break;
	case 6:			/* bmski33 */
	  CCPU_GPR[rt3].u = CCPU_GPR[rt3].u & (1 << __GF (insn, 3, 3));
	  break;
	case 7:			/* fexti33 */
	  CCPU_GPR[rt3].u = CCPU_GPR[rt3].u & ((1 << (__GF (insn, 3, 3) + 1)) - 1);
	  break;
	}
      return;
    }

  switch (__GF (insn, 10, 5))
    {
    case 0x0:			/* mov55 or ifret16 */
      /* It's ok to do assignment even if it's ifret16.  */
      CCPU_GPR[rt5].u = CCPU_GPR[ra5].u;

      if (rt5 == ra5 && rt5 == 31 && CCPU_SR_TEST (PSW, PSW_IFCON))
	{
	  /* ifret */
	  CCPU_SR_CLEAR (PSW, PSW_IFCON);
	  nds32_set_nia (cpu, CCPU_USR[USR0_IFCLP].u);
	}
      return;
    case 0x1:			/* movi55 */
      CCPU_GPR[rt5].s = imm5s;
      return;
    case 0x1b:			/* addi10s (V2) */
      CCPU_GPR[GPR_SP].u += N16_IMM10S (insn);
      return;
    }

  switch (__GF (insn, 11, 4))
    {
    case 0x7:			/* lwi37.fp/swi37.fp */
      addr = CCPU_GPR[GPR_FP].u + (N16_IMM7U (insn) << 2);
      if (__TEST (insn, 7))	/* swi37.fp */
	nds32_st_aligned (cpu, addr, 4, CCPU_GPR[rt38].u);
      else			/* lwi37.fp */
	CCPU_GPR[rt38].u = nds32_ld_aligned (cpu, addr, 4);
      return;
    case 0x8:			/* beqz38 */
      if (CCPU_GPR[rt38].u == 0)
	{
	  CCPU_SR_CLEAR (PSW, PSW_IFCON);
	  nds32_set_nia (cpu, cia + (N16_IMM8S (insn) << 1));
	}
      return;
    case 0x9:			/* bnez38 */
      if (CCPU_GPR[rt38].u != 0)
	{
	  CCPU_SR_CLEAR (PSW, PSW_IFCON);
	  nds32_set_nia (cpu, cia + (N16_IMM8S (insn) << 1));
	}
      return;
    case 0xa:			/* beqs38/j8, implied r5 */
      if (CCPU_GPR[rt38].u == CCPU_GPR[5].u)	/* rt38 == 5 means j8 */
	{
	  CCPU_SR_CLEAR (PSW, PSW_IFCON);
	  nds32_set_nia (cpu, cia + (N16_IMM8S (insn) << 1));
	}
      return;
    case 0xb:			/* bnes38 and others */
      if (rt38 == 5)
	{
	  switch (__GF (insn, 5, 3))
	    {
	    case 0:		/* jr5 */
	    case 4:		/* ret5 */
	      CCPU_SR_CLEAR (PSW, PSW_IFCON);
	      nds32_set_nia (cpu, CCPU_GPR[ra5].u);
	      return;
	    case 1:		/* jral5 */
	      CCPU_GPR[GPR_LP].u = cia + 2;
	      if (CCPU_SR_TEST (PSW, PSW_IFCON))
		CCPU_GPR[GPR_LP] = CCPU_USR[USR0_IFCLP];
	      CCPU_SR_CLEAR (PSW, PSW_IFCON);
	      nds32_set_nia (cpu, CCPU_GPR[ra5].u);
	      return;
	    case 2:		/* ex9.it imm5 */
	      sim_read (sd, (CCPU_USR[USR0_ITB].u & ~3U) + (imm5u << 2),
			(unsigned char *) &insn, 4);
	      insn = extract_unsigned_integer ((unsigned char *) &insn, 4,
					       BIG_ENDIAN);
	      nds32_decode16_ex9 (cpu, insn, cia);
	      return;
	    case 5:		/* add5.pc */
	      CCPU_GPR[ra5].u += cia;
	      return;
	    default:
	      goto bad_op;
	    }
	  return;
	}
      else if (CCPU_GPR[rt38].u != CCPU_GPR[5].u)
	{
	  /* bnes38 */
	  CCPU_SR_CLEAR (PSW, PSW_IFCON);
	  nds32_set_nia (cpu, cia + (N16_IMM8S (insn) << 1));
	  return;
	}
      return;
    case 0xe:			/* lwi37/swi37 */
      addr = CCPU_GPR[GPR_SP].u + (N16_IMM7U (insn) << 2);
      if (__TEST (insn, 7))	/* swi37.sp */
	nds32_st_aligned (cpu, addr, 4, CCPU_GPR[rt38].u);
      else			/* lwi37.sp */
	CCPU_GPR[rt38].u = nds32_ld_aligned (cpu, addr, 4);
      return;
    }

bad_op:
  nds32_bad_op (cpu, cia, insn, "16-bit");
}

void
sim_engine_run (SIM_DESC sd, int next_cpu_nr, int nr_cpus, int siggnal)
{
  int r;
  sim_cia cia;
  sim_cpu *cpu;
  SIM_ASSERT (STATE_MAGIC (sd) == SIM_MAGIC_NUMBER);
  cpu = STATE_CPU (sd, 0);
  cia = CPU_PC_GET (cpu);

  if (siggnal != 0)
    {
      sim_engine_halt (CPU_STATE (cpu), cpu, NULL, cia, sim_exited,
		       128 + siggnal);
      return;
    }

  while (1)
    {
      uint32_t insn;

      recent_cia[recent_cia_idx] = cia;
      recent_cia_idx = (recent_cia_idx + 1) & RECENT_CIA_MASK;

      nds32_pfm_event (cpu, PFM_CYCLE);
      nds32_pfm_event (cpu, PFM_INST);

      r = sim_read (sd, cia, (unsigned char *) &insn, 4);
      insn = extract_unsigned_integer ((unsigned char *) &insn, 4,
				       BIG_ENDIAN);

      if (r != 4)
	nds32_dump_registers (sd);
      SIM_ASSERT (r == 4);

      if (TRACE_LINENUM_P (cpu))
	{
	  trace_prefix (sd, cpu, NULL_CIA, cia, TRACE_LINENUM_P (cpu),
			NULL, 0, " "); /* Use a space for gcc warnings.  */
	}

      cpu->iflags &= ~NIF_BRANCH;
      if ((insn & 0x80000000) == 0)
	nds32_decode32 (cpu, insn, cia);
      else
	nds32_decode16 (cpu, insn >> 16, cia);

      /* Handle zero overhead loop.  */
      if (cpu->reg_usr[USR0_LC].u > 1 && (cia == cpu->reg_usr[USR0_LE].u))
	{
	  cpu->reg_usr[USR0_LC].u--;
	  cia = cpu->reg_usr[USR0_LB].u;
	}
      else if ((insn & 0x80000000) == 0)
	cia += 4;
      else
	cia += 2;

      if (cpu->iflags & NIF_BRANCH)
	{
	  if (cpu->baddr & 1)
	    nds32_raise_exception (cpu, EXP_GENERAL, SIM_SIGSEGV,
				   "Alignment check exception. "
				   "Unaligned instruction address 0x%08x\n",
				   cia);
	  cia = cpu->baddr;
	}

      if (TRACE_LINENUM_P (cpu))
	{
	  trace_result_addr1 (sd, cpu, TRACE_INSN_IDX, cia);
	}

      /* Sync registers.  */
      CPU_PC_SET (cpu, cia);

      /* process any events */
      if (sim_events_tick (sd))
	{
	  CPU_PC_SET (cpu, cia);
	  sim_events_process (sd);
	}
    }
}

/* This function is mainly used for fetch general purpose registers.
   GDB remote-sim calls this too, so it will be used for fetch some
   USR (PC, D0, D1), FLOAT, SR (PSW).  */

static int
nds32_fetch_register (sim_cpu *cpu, int rn, unsigned char *memory, int length)
{
  uint64_t val = 0;

  /* General purpose registers.  */
  if (rn < 32)
    {
      val = cpu->reg_gpr[rn].u;
      goto do_fetch;
    }

  /* Special user registers.  */
  switch (rn)
    {
    case SIM_NDS32_PC_REGNUM:
      val = cpu->reg_usr[USR0_PC].u;
      goto do_fetch;
    case SIM_NDS32_ITB_REGNUM:
      val = cpu->reg_usr[USR0_ITB].u;
      goto do_fetch;
    case SIM_NDS32_IFCLP_REGNUM:
      val = cpu->reg_usr[USR0_IFCLP].u;
      goto do_fetch;
    case SIM_NDS32_LB_REGNUM:
      val = cpu->reg_usr[USR0_LB].u;
      goto do_fetch;
    case SIM_NDS32_LE_REGNUM:
      val = cpu->reg_usr[USR0_LE].u;
      goto do_fetch;
    case SIM_NDS32_LC_REGNUM:
      val = cpu->reg_usr[USR0_LC].u;
      goto do_fetch;
    }

  if (rn >= SIM_NDS32_FD0_REGNUM && rn < SIM_NDS32_FD0_REGNUM + 32)
    {
      int fr = (rn - SIM_NDS32_FD0_REGNUM) << 1;

      val = ((uint64_t) cpu->reg_fpr[fr].u << 32)
	    | (uint64_t) cpu->reg_fpr[fr + 1].u;
      goto do_fetch;
    }

  /* System registers.  */
  switch (rn)
    {
    case SIM_NDS32_PSW_REGNUM:
      val = cpu->reg_sr[SRIDX (1, 0, 0)].u;
      goto do_fetch;
    }

  return 0;

do_fetch:
  store_unsigned_integer (memory, length,
			  CCPU_SR_TEST (PSW, PSW_BE)
			  ? BIG_ENDIAN : LITTLE_ENDIAN,
			  val);
  return length;
}

static int
nds32_store_register (sim_cpu *cpu, int rn, unsigned char *memory, int length)
{
  uint64_t val;

  val = extract_unsigned_integer (memory, length,
				  CCPU_SR_TEST (PSW, PSW_BE)
				  ? BIG_ENDIAN : LITTLE_ENDIAN);

  /* General purpose registers.  */
  if (rn < 32)
    {
      cpu->reg_gpr[rn].u = val;
      return 4;
    }

  /* Special user registers.  */
  switch (rn)
    {
    case SIM_NDS32_PC_REGNUM:
      cpu->reg_usr[USR0_PC].u = val;
      return 4;
    case SIM_NDS32_ITB_REGNUM:
      cpu->reg_usr[USR0_ITB].u = val;
      return 4;
    case SIM_NDS32_IFCLP_REGNUM:
      cpu->reg_usr[USR0_IFCLP].u = val;
      return 4;
    case SIM_NDS32_LB_REGNUM:
      cpu->reg_usr[USR0_LB].u = val;
      return 4;
    case SIM_NDS32_LE_REGNUM:
      cpu->reg_usr[USR0_LE].u = val;
      return 4;
    case SIM_NDS32_LC_REGNUM:
      cpu->reg_usr[USR0_LC].u = val;
      return 4;
    }

  if (rn >= SIM_NDS32_FD0_REGNUM && rn < SIM_NDS32_FD0_REGNUM + 32)
    {
      int fr = (rn - SIM_NDS32_FD0_REGNUM) << 1;

      cpu->reg_fpr[fr + 1].u = val & 0xffffffff;
      cpu->reg_fpr[fr].u = (val >> 32) & 0xffffffff;
      return 8;
    }

  /* System registers.  */
  switch (rn)
    {
    case SIM_NDS32_PSW_REGNUM:
      cpu->reg_sr[SRIDX (1, 0, 0)].u = val;
      return 4;
    }

  return 0;
}

static sim_cia
nds32_pc_get (sim_cpu *cpu)
{
  return cpu->reg_usr[USR0_PC].u;
}

static void
nds32_pc_set (sim_cpu *cpu, sim_cia cia)
{
  cpu->reg_usr[USR0_PC].u = cia;
}

static void
nds32_initialize_cpu (SIM_DESC sd, sim_cpu *cpu, struct bfd *abfd)
{
  memset (cpu->reg_gpr, 0, sizeof (cpu->reg_gpr));
  memset (cpu->reg_usr, 0, sizeof (cpu->reg_usr));
  memset (cpu->reg_sr, 0, sizeof (cpu->reg_sr));
  memset (cpu->reg_fpr, 0, sizeof (cpu->reg_fpr));

  /* Common operations defined in sim-cpu.h */
  CPU_REG_FETCH (cpu) = nds32_fetch_register;
  CPU_REG_STORE (cpu) = nds32_store_register;
  CPU_PC_FETCH (cpu) = nds32_pc_get;
  CPU_PC_STORE (cpu) = nds32_pc_set;

  /* CPU_VER: N12 + COP/FPU */
  CCPU_SR[SRIDX (0, 0, 0)].u = (0xc << 24) | 3;

  /* MSC_CFG */
  /* User code may need this for specialized code. e.g., set $ITB.  */
  CCPU_SR_SET (MSC_CFG, MSC_CFG_PFM);
  CCPU_SR_SET (MSC_CFG, MSC_CFG_DIV);
  CCPU_SR_SET (MSC_CFG, MSC_CFG_MAC);
  CCPU_SR_SET (MSC_CFG, MSC_CFG_IFC);
  CCPU_SR_SET (MSC_CFG, MSC_CFG_EIT);

  CCPU_SR_CLEAR (IVB, IVB_EVIC);	/* (IM) */
  CCPU_SR_PUT (IVB, IVB_ESZ, 1);	/* 16-byte */
  CCPU_SR_PUT (IVB, IVB_IVBASE, 0);	/* (IM) */

  /* Floating-Point Configuration Register.  */
  CCPU_FPCFG_SET (SP);
  CCPU_FPCFG_SET (DP);
  CCPU_FPCFG_SET (FMA);
  CCPU_FPCFG_PUT (FREG, 3);
  /* Floating-Point Control Status Register.  */
  CCPU_FPCSR.u = 0;
}

static void
nds32_free_state (SIM_DESC sd)
{
  if (STATE_MODULES (sd) != NULL)
    sim_module_uninstall (sd);
  sim_cpu_free_all (sd);
  sim_state_free (sd);
}

SIM_DESC
sim_open (SIM_OPEN_KIND kind, host_callback *callback,
	  struct bfd *abfd, char * const *argv)
{
  int i;
  SIM_DESC sd = sim_state_alloc (kind, callback);

  /* The cpu data is kept in a separately allocated chunk of memory.  */
  if (sim_cpu_alloc_all (sd, 1, 0) != SIM_RC_OK)
    {
      nds32_free_state (sd);
      return 0;
    }

  if (sim_pre_argv_init (sd, argv[0]) != SIM_RC_OK)
    {
      nds32_free_state (sd);
      return 0;
    }

  /* The parser will print an error message for us, so we silently return.  */
  if (sim_parse_args (sd, argv) != SIM_RC_OK)
    {
      nds32_free_state (sd);
      return 0;
    }

  /* Check for/establish the a reference program image.  */
  if (sim_analyze_program (sd,
			   (STATE_PROG_ARGV (sd) != NULL
			    ? *STATE_PROG_ARGV (sd)
			    : NULL), abfd) != SIM_RC_OK)
    {
      nds32_free_state (sd);
      return 0;
    }

  /* Establish any remaining configuration options.  */
  if (sim_config (sd) != SIM_RC_OK)
    {
      nds32_free_state (sd);
      return 0;
    }

  if (sim_post_argv_init (sd) != SIM_RC_OK)
    {
      nds32_free_state (sd);
      return 0;
    }

  /* Allocate 64MB memory if none is set up by user.  */
  if (STATE_MEMOPT (sd) == NULL)
    sim_do_command (sd, "memory region 0,0x4000000");	/* 64 MB */

  /* CPU specific initialization.  */
  for (i = 0; i < MAX_NR_PROCESSORS; ++i)
    {
      sim_cpu *cpu = STATE_CPU (sd, i);
      nds32_initialize_cpu (sd, cpu, abfd);
    }

  return sd;
}

static int
sim_dis_read (bfd_vma memaddr, bfd_byte *myaddr, unsigned int length,
	      struct disassemble_info *dinfo)
{
  SIM_DESC sd = (SIM_DESC) dinfo->application_data;

  return sim_read (sd, memaddr, (unsigned char *) myaddr, length)
	 != length;
}

void
nds32_init_libgloss (SIM_DESC sd, struct bfd *abfd,
		     char * const *argv, char * const *env)
{
  int len, mlen, i;

  STATE_CALLBACK (sd)->syscall_map = cb_nds32_libgloss_syscall_map;

  /* Save argv for -mcrt-arg hacking.  */
  memset (sd->cmdline, 0, sizeof (sd->cmdline));
  mlen = sizeof (sd->cmdline) - 1;
  len = 0;
  for (i = 0; argv && argv[i]; i++)
    {
      int l = strlen (argv[i]) + 1;

      if (l + len >= mlen)
	break;

      len += sprintf (sd->cmdline + len, "%s ", argv[i]);
    }

  if (len > 0)
    sd->cmdline[len - 1] = '\0';	/* Trim the last space. */

  return;
}

SIM_RC
sim_create_inferior (SIM_DESC sd, struct bfd *prog_bfd,
		     char * const *argv, char * const *env)
{
  SIM_CPU *cpu = STATE_CPU (sd, 0);

  /* Set the initial register set.  */
  if (prog_bfd == NULL)
    return SIM_RC_OK;

  memset (&dis_info, 0, sizeof (dis_info));
  /* See opcode/dis-init.c and dis-asm.h for details.  */
  INIT_DISASSEMBLE_INFO (dis_info, stderr, fprintf);
  dis_info.application_data = (void *) sd;
  dis_info.read_memory_func = sim_dis_read;
  dis_info.arch = bfd_get_arch (prog_bfd);
  dis_info.mach = bfd_get_mach (prog_bfd);
  disassemble_init_for_target (&dis_info);

  /* Set PC to entry point address.  */
  (*CPU_PC_STORE (cpu)) (cpu, bfd_get_start_address (prog_bfd));

  /* Set default endian.  */
  if (bfd_big_endian (prog_bfd))
    {
      CCPU_SR_SET (PSW, PSW_BE);
      STATE_CALLBACK (sd)->target_endian = BFD_ENDIAN_BIG;
    }
  else
    {
      CCPU_SR_CLEAR (PSW, PSW_BE);
      STATE_CALLBACK (sd)->target_endian = BFD_ENDIAN_LITTLE;
    }

  STATE_CALLBACK (sd)->syscall_map = cb_nds32_libgloss_syscall_map;
  nds32_init_libgloss (sd, prog_bfd, argv, env);

  return SIM_RC_OK;
}
