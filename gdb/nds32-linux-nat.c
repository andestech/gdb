/* Common target dependent code for GDB on nds32 systems.

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
#include "inferior.h"
#include "elf/common.h"
#include "gdbcore.h"
#include "regcache.h"
#include "target.h"
#include "linux-nat.h"
#include "target-descriptions.h"

#include <sys/ptrace.h>

/* Prototypes for supply_gregset etc. */
#include "gregset.h"

/* Defines ps_err_e, struct ps_prochandle.  */
#include "gdb_proc_service.h"

#include "nds32-tdep.h"
#include "nds32-linux-tdep.h"

/* Fetch a general register of the process and store into register
   cache REGCACHE.  See: arch/nds32/include/asm/ptrace.h */

static void
fetch_register (struct regcache *regcache, int regno)
{
  int ret, tid;
  elf_gregset_t regs;

  /* Check whether the size of gregset is consistent with kernel.  */
  gdb_assert (sizeof (elf_gregset_t) >= (sizeof (long) * 50));

  /* Get the thread id for the ptrace call.  */
  tid = ptid_get_lwp (inferior_ptid);

  ret = ptrace (PTRACE_GETREGS, tid, 0, &regs);
  if (ret < 0)
    perror_with_name (_("Unable to fetch general register."));

  if (regno >= NDS32_R0_REGNUM && regno < NDS32_LINUX_NUM_GPRS
      && nds32_ptreg_map[regno] != -1)
    regcache_raw_supply (regcache, regno,
			 (char *) &regs[nds32_ptreg_map[regno]]);
}

/* Fetch all general registers of the process and store into register
   cache REGCACHE.  */

static void
fetch_regs (struct regcache *regcache)
{
  int ret, regno, tid;
  elf_gregset_t regs;

  /* Get the thread id for the ptrace call.  */
  tid = ptid_get_lwp (inferior_ptid);

  ret = ptrace (PTRACE_GETREGS, tid, 0, &regs);
  if (ret < 0)
    perror_with_name (_("Unable to fetch general registers."));

  for (regno = NDS32_R0_REGNUM; regno < NDS32_LINUX_NUM_GPRS; regno++)
    {
      if (nds32_ptreg_map[regno] == -1)
	continue;
      regcache_raw_supply (regcache, regno,
			   (char *) &regs[nds32_ptreg_map[regno]]);
    }
}

/* Store all general registers of the process from the values in
   register cache REGCACHE.  */

static void
store_register (const struct regcache *regcache, int regno)
{
  int ret, tid;
  elf_gregset_t regs;

  if (REG_VALID != regcache_register_status (regcache, regno))
    return;

  /* Get the thread id for the ptrace call.  */
  tid = ptid_get_lwp (inferior_ptid);

  /* Get the general registers from the process.  */
  ret = ptrace (PTRACE_GETREGS, tid, 0, &regs);
  if (ret < 0)
    perror_with_name (_("Unable to fetch general register."));

  if (regno >= NDS32_R0_REGNUM && regno < NDS32_LINUX_NUM_GPRS
      && nds32_ptreg_map[regno] != -1)
    regcache_raw_collect (regcache, regno,
			  (char *) &regs[nds32_ptreg_map[regno]]);

  ret = ptrace (PTRACE_SETREGS, tid, 0, &regs);
  if (ret < 0)
    perror_with_name (_("Unable to store general register."));
}

static void
store_regs (const struct regcache *regcache)
{
  int ret, regno, tid;
  elf_gregset_t regs;

  /* Get the thread id for the ptrace call.  */
  tid = ptid_get_lwp (inferior_ptid);

  /* Fetch the general registers.  */
  ret = ptrace (PTRACE_GETREGS, tid, 0, &regs);
  if (ret < 0)
    perror_with_name (_("Unable to fetch general registers."));

  for (regno = 0; regno < NDS32_LINUX_NUM_GPRS; regno++)
    {
      if (nds32_ptreg_map[regno] == -1)
	continue;
      if (REG_VALID != regcache_register_status (regcache, regno))
	regcache_raw_collect (regcache, regno,
			      (char *) &regs[nds32_ptreg_map[regno]]);
    }

  ret = ptrace (PTRACE_SETREGS, tid, 0, &regs);

  if (ret < 0)
    perror_with_name (_("Unable to store general registers."));
}

/* Fetch registers from the child process.  Fetch all registers if
   REGNO == -1, otherwise fetch all general registers or all floating
   point registers depending upon the value of REGNO.  */

static void
nds32_linux_fetch_inferior_registers (struct target_ops *ops,
				      struct regcache *regcache,
				      int regno)
{
  /* TODO: Handle AUDIO and FPU registers.  */

  if (-1 == regno)
    fetch_regs (regcache);
  else
    fetch_register (regcache, regno);
}

/* Store registers back into the inferior.  Store all registers if
   REGNO == -1, otherwise store all general registers or all floating
   point registers depending upon the value of REGNO.  */

static void
nds32_linux_store_inferior_registers (struct target_ops *ops,
				      struct regcache *regcache,
				      int regno)
{
  /* TODO: Handle AUDIO and FPU registers.  */

  if (-1 == regno)
    store_regs (regcache);
  else
    store_register (regcache, regno);
}

/* Wrapper functions for the standard regset handling, used by
   thread debugging.  */

void
fill_gregset (const struct regcache *regcache,
	      gdb_gregset_t *gregsetp, int regno)
{
  nds32_linux_collect_gregset (NULL, regcache, regno, gregsetp, 0);
}

void
supply_gregset (struct regcache *regcache, const gdb_gregset_t *gregsetp)
{
  nds32_linux_supply_gregset (NULL, regcache, -1, gregsetp, 0);
}

void
fill_fpregset (const struct regcache *regcache,
	       gdb_fpregset_t *fpregsetp, int regno)
{
  warning (_("fill_fpregset not implemented"));
}

/* Fill GDB's register array with the floating-point register values
   in *fpregsetp.  */

void
supply_fpregset (struct regcache *regcache, const gdb_fpregset_t *fpregsetp)
{
  warning (_("supply_fpregset not implemented"));
}

/* Fetch the thread-local storage pointer for libthread_db.  */

#if 0
/* TODO: TLS is not supported, yet.  */

ps_err_e
ps_get_thread_area (const struct ps_prochandle *ph,
		    lwpid_t lwpid, int idx, void **base)
{
}
#endif

static const struct target_desc *
nds32_linux_read_description (struct target_ops *ops)
{
  CORE_ADDR nds32_hwcap = 0;

  /* TODO: FPU registers.  */

  return tdesc_nds32_linux;
}

/* Provide a prototype to silence -Wmissing-prototypes.  */
extern initialize_file_ftype _initialize_nds32_linux_nat;

void
_initialize_nds32_linux_nat (void)
{
  struct target_ops *t;

  /* Fill in the generic GNU/Linux methods.  */
  t = linux_target ();

  /* Add our register access methods.  */
  t->to_fetch_registers = nds32_linux_fetch_inferior_registers;
  t->to_store_registers = nds32_linux_store_inferior_registers;

  t->to_read_description = nds32_linux_read_description;

  /* Register the target.  */
  linux_nat_add_target (t);
}
