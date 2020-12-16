/* GNU/Linux/RISC-V specific low level interface, GDBserver.

   Copyright (C) 2012-2017 Free Software Foundation, Inc.

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

#include "server.h"
#include "linux-low.h"

#include "nat/gdb_ptrace.h"
#include <asm/ptrace.h>
#include <elf.h>

#if _LP64 != 1
# ifndef __riscv_flen
/* Defined in auto-generated file riscv32-linux.c.  */
void init_registers_riscv32_linux (void);
extern const struct target_desc *tdesc_riscv32_linux;
# else
#  if __riscv_flen == 64
/* Defined in auto-generated file riscv32d-linux.c.  */
void init_registers_riscv32d_linux (void);
extern const struct target_desc *tdesc_riscv32d_linux;
#  elif __riscv_flen == 32
/* Defined in auto-generated file riscv32f-linux.c.  */
void init_registers_riscv32f_linux (void);
extern const struct target_desc *tdesc_riscv32f_linux;
#  endif
# endif
#else
# ifndef __riscv_flen
/* Defined in auto-generated file riscv64-linux.c.  */
void init_registers_riscv64_linux (void);
extern const struct target_desc *tdesc_riscv64_linux;
# else
#  if __riscv_flen == 64
/* Defined in auto-generated file riscv64d-linux.c.  */
void init_registers_riscv64d_linux (void);
extern const struct target_desc *tdesc_riscv64d_linux;
#  elif __riscv_flen == 32
/* Defined in auto-generated file riscv64f-linux.c.  */
void init_registers_riscv64f_linux (void);
extern const struct target_desc *tdesc_riscv64f_linux;
#  endif
# endif
#endif

#define riscv_num_regs 32

#define RISCV_ZERO_REGNUM	0
#define RISCV_RA_REGNUM		1
#define RISCV_PC_REGNUM		32
#define RISCV_F0_REGNUM		33
#define RISCV_FPR_NUM		32

static int riscv_regmap[] =
{
  -1,  1,  2,  3,  4,  5,  6,  7,
   8,  9, 10, 11, 12, 13, 14, 15,
  16, 17, 18, 19, 20, 21, 22, 23,
  24, 25, 26, 27, 28, 29, 30, 31,

  /* pc */
  0,
};

static int
riscv_cannot_fetch_register (int regno)
{
  if (regno >= 0 && regno < 63)
    return 0;
  else
    return 1;
}

static int
riscv_cannot_store_register (int regno)
{
  if (regno >= 0 && regno < 63)
    return 0;
  else
    return 1;
}

/* Implementation of linux_target_ops method "sw_breakpoint_from_kind".  */

static const gdb_byte ebreak[] = { 0x73, 0x00, 0x10, 0x00, };
static const gdb_byte c_ebreak[] = { 0x02, 0x90 };

static const gdb_byte *
riscv_sw_breakpoint_from_kind (int kind, int *size)
{
  *size = kind;
  switch (kind)
    {
    case 2:
      return c_ebreak;
    case 4:
      return ebreak;
    default:
      gdb_assert(0);
    }
}

static int
riscv_breakpoint_at (CORE_ADDR where)
{
  uint8_t insn[4];

  (*the_target->read_memory) (where, (unsigned char *) &insn, 4);
  if (insn[0] == ebreak[0] && insn[1] == ebreak[1]
      && insn[2] == ebreak[2] && insn[3] == ebreak[3])
    return 1;
  if (insn[0] == c_ebreak[0] && insn[1] == c_ebreak[1])
    return 1;

  /* If necessary, recognize more trap instructions here.  GDB only uses the
     one.  */
  return 0;
}

static void
riscv_fill_gregset (struct regcache *regcache, void *buf)
{
  int i;

  for (i = RISCV_ZERO_REGNUM; i <= RISCV_PC_REGNUM; i++)
    if (riscv_regmap[i] != -1)
      collect_register (regcache, i, ((elf_greg_t *) buf) + riscv_regmap[i]);
}

static void
riscv_store_gregset (struct regcache *regcache, const void *buf)
{
  int i;

  for (i = RISCV_ZERO_REGNUM; i <= RISCV_PC_REGNUM; i++)
    if (riscv_regmap[i] != -1)
      supply_register (regcache, i, ((elf_greg_t *) buf) + riscv_regmap[i]);
}

#if __riscv_flen == 64
static void
riscv_fill_fpregset (struct regcache *regcache, void *buf)
{
  struct __riscv_d_ext_state *regset = (struct __riscv_d_ext_state*) buf;
  int i;

  for (i = 0; i < RISCV_FPR_NUM; i++)
    collect_register (regcache, RISCV_F0_REGNUM + i, &regset->f[i]);
  collect_register_by_name (regcache, "fcsr", &regset->fcsr);
}

static void
riscv_store_fpregset (struct regcache *regcache, const void *buf)
{
  struct __riscv_d_ext_state *regset = (struct __riscv_d_ext_state*) buf;
  int i;

  for (i = 0; i < RISCV_FPR_NUM; i++)
    supply_register (regcache, RISCV_F0_REGNUM + i, &regset->f[i]);
  supply_register_by_name (regcache, "fcsr", &regset->fcsr);
}
#endif

static struct regset_info riscv_regsets[] =
{
  { PTRACE_GETREGSET, PTRACE_SETREGSET, NT_PRSTATUS,
    sizeof (struct user_regs_struct), GENERAL_REGS,
    riscv_fill_gregset, riscv_store_gregset },
#if __riscv_flen == 64
  { PTRACE_GETREGSET, PTRACE_SETREGSET, NT_PRFPREG,
    sizeof (struct __riscv_d_ext_state), FP_REGS,
    riscv_fill_fpregset, riscv_store_fpregset },
#endif
  NULL_REGSET
};

static struct regsets_info riscv_regsets_info =
  {
    riscv_regsets, /* regsets */
    0, /* num_regsets */
    NULL, /* disabled_regsets */
  };

static struct regs_info regs_info =
  {
    NULL, /* regset_bitmap */
    NULL, /* usrregs */
    &riscv_regsets_info,
  };

static const struct regs_info *
riscv_regs_info (void)
{
  return &regs_info;
}

static void
riscv_arch_setup (void)
{
  int pid = pid_of (current_thread);
  unsigned int machine;
  int is_elf64 = linux_pid_exe_is_elf_64_file (pid, &machine);

  if (sizeof (void *) == 4)
    if (is_elf64 > 0)
      error (_("Can't debug 64-bit process with 32-bit GDBserver"));

#if _LP64 != 1
# ifndef __riscv_flen
  current_process ()->tdesc = tdesc_riscv32_linux;
# else
#  if __riscv_flen == 64
  current_process ()->tdesc = tdesc_riscv32d_linux;
#  elif __riscv_flen == 32
  current_process ()->tdesc = tdesc_riscv32f_linux;
#  endif
# endif
#else
# ifndef __riscv_flen
  current_process ()->tdesc = tdesc_riscv64_linux;
# else
#  if __riscv_flen == 64
  current_process ()->tdesc = tdesc_riscv64d_linux;
#  elif __riscv_flen == 32
  current_process ()->tdesc = tdesc_riscv64f_linux;
#  endif
# endif
#endif
}

/* Support for hardware single step.  */

static int
riscv_supports_hardware_single_step (void)
{
  return 1;
}


struct linux_target_ops the_low_target =
{
  riscv_arch_setup,
  riscv_regs_info,
  riscv_cannot_fetch_register,
  riscv_cannot_store_register,
  NULL,
  linux_get_pc_64bit,
  linux_set_pc_64bit,
  NULL, /* breakpoint_kind_from_pc */
  riscv_sw_breakpoint_from_kind,
  NULL,
  0,
  riscv_breakpoint_at,
  NULL, /* supports_z_point_type */
  NULL, /* insert_point */
  NULL, /* remove_point */
  NULL, /* stopped_by_watchpoint */
  NULL, /* stopped_data_address */
  NULL, /* collect_ptrace_register */
  NULL, /* supply_ptrace_register */
  NULL, /* siginfo_fixup */
  NULL, /* new_process */
  NULL, /* delete_process */
  NULL, /* new_thread */
  NULL, /* delete_thread */
  NULL, /* new_fork */
  NULL, /* prepare_to_resume */
  NULL, /* process_qsupported */
  NULL, /* supports_tracepoints */
  NULL, /* get_thread_area */
  NULL, /* install_fast_tracepoint_jump_pad */
  NULL, /* emit_ops */
  NULL, /* get_min_fast_tracepoint_insn_len */
  NULL, /* supports_range_stepping */
  NULL, /* breakpoint_kind_from_current_state */
  riscv_supports_hardware_single_step,
  NULL, /* get_syscall_trapinfo */
  NULL, /* get_ipa_tdesc_idx */
};

void
initialize_low_arch (void)
{
#if _LP64 != 1
# ifndef __riscv_flen
  init_registers_riscv32_linux ();
# else
#  if __riscv_flen == 64
  init_registers_riscv32d_linux ();
#  elif __riscv_flen == 32
  init_registers_riscv32f_linux ();
#  endif
# endif
#else
# ifndef __riscv_flen
  init_registers_riscv64_linux ();
# else
#  if __riscv_flen == 64
  init_registers_riscv64d_linux ();
#  elif __riscv_flen == 32
  init_registers_riscv64f_linux ();
#  endif
# endif
#endif

  initialize_regsets_info (&riscv_regsets_info);
}
