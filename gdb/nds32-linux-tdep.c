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
#include "gdbcore.h"
#include "regcache.h"
#include "osabi.h"
#include "regset.h"
#include "glibc-tdep.h"
#include "solib-svr4.h"
#include "tramp-frame.h"
#include "trad-frame.h"
#include "frame-unwind.h"
#include "linux-tdep.h"
#include "user-regs.h"
#include "xml-syscall.h"

#include "nds32-tdep.h"
#include "nds32-linux-tdep.h"

#include "features/nds32-linux.c"

extern struct nds32_gdb_config nds32_config;

/* Recognizing signal handler frames.  */

/* GNU/Linux has two flavors of signals.  Normal signal handlers, and
   "realtime" (RT) signals.  The RT signals can provide additional
   information to the signal handler if the SA_SIGINFO flag is set
   when establishing a signal handler using `sigaction'.  It is not
   unlikely that future versions of GNU/Linux will support SA_SIGINFO
   for normal signals too.  */

/* When the NDS32 Linux kernel calls a signal handler and the
   SA_RESTORER flag isn't set, the return address points to a bit of
   code on HIGH VECTOR.  This function returns whether the PC appears to
   be within this bit of code.

   The instructions for normal and realtime signals are
       syscall   #__NR_sigreturn ( 0x64 0x0A 0x0E 0xEB)
       or
       syscall   #__NR_rt_sigreturn ( 0x64 0x0A 0x15 0xAB)

   Checking for the code sequence should be somewhat reliable, because
   the effect is to call the system call sigreturn.  This is unlikely
   to occur anywhere other than in a signal trampoline.

   It kind of sucks that we have to read memory from the process in
   order to identify a signal trampoline, but there doesn't seem to be
   any other way.  Therefore we only do the memory reads if no
   function name could be identified, which should be the case since
   the code is on the stack.

   Detection of signal trampolines for handlers that set the
   SA_RESTORER flag is in general not possible.  Unfortunately this is
   what the GNU C Library has been doing for quite some time now.
   However, as of version 2.1.2, the GNU C Library uses signal
   trampolines (named __restore and __restore_rt) that are identical
   to the ones used by the kernel.  Therefore, these trampolines are
   supported too.  */

static void
nds32_linux_sigtramp_cache (struct frame_info *this_frame,
			    struct trad_frame_cache *this_cache,
			    CORE_ADDR func, int regs_offset)
{
  CORE_ADDR sp = get_frame_sp (this_frame);
  CORE_ADDR base = sp + regs_offset;
  int i;

  /* r0 ~ r25 */
  for (i = 0; i < 26; i++)
    trad_frame_set_reg_addr (this_cache, i, base + i * 4);

  /* FP, GP, LP, SP, PC */
  for (i = NDS32_FP_REGNUM; i <=NDS32_PC_REGNUM; i++)
    /* P0/P1 are not in sigcontext, so two elements are skipped.  */
    trad_frame_set_reg_addr (this_cache, i, base + (i - 2) * 4);

  /* Save a frame ID.  */
  trad_frame_set_id (this_cache, frame_id_build (sp, func));
}

#define NDS32_LINUX_SIGRETURN_INSTR	0xeb0e0a64 /* syscall #0x5077 */
#define NDS32_LINUX_RT_SIGRETURN_INSTR	0xab150a64 /* syscall #0x50ad */

/* There are three words (trap_no, error_code, oldmask) in
   struct sigcontext before r0.  */
#define OFFSETOF_SIGCONTEXT_R0 12

/* There are five words (uc_flags, uc_link, and three for uc_stack)
   in the ucontext_t and one padding word (sigcontext within which contains
   struct fpu_struct must be 8 bytes-aligned) before the sigcontext.  */
#define OFFSETOF_UCONTEXT_SIGCONTEXT 24

/* There is one element in an rt_sigframe before the ucontext:
   info, which is a struct siginfo, with size 128 bytes.
   Skip the whole thing to reach the base of ucontext.  */
#define SIZEOF_SIGINFO 128

static void
nds32_linux_sigreturn_init (const struct tramp_frame *self,
			    struct frame_info *this_frame,
			    struct trad_frame_cache *this_cache,
			    CORE_ADDR func)
{
  struct gdbarch *gdbarch = get_frame_arch (this_frame);

  nds32_linux_sigtramp_cache (this_frame, this_cache, func,
			      OFFSETOF_UCONTEXT_SIGCONTEXT
			      + OFFSETOF_SIGCONTEXT_R0);
}

static void
nds32_linux_rt_sigreturn_init (const struct tramp_frame *self,
			       struct frame_info *this_frame,
			       struct trad_frame_cache *this_cache,
			       CORE_ADDR func)
{
  struct gdbarch *gdbarch = get_frame_arch (this_frame);

  nds32_linux_sigtramp_cache (this_frame, this_cache, func,
			      SIZEOF_SIGINFO
			      + OFFSETOF_UCONTEXT_SIGCONTEXT
			      + OFFSETOF_SIGCONTEXT_R0);
}

static struct tramp_frame nds32_linux_sigreturn_tramp_frame = {
  SIGTRAMP_FRAME,
  4,
  {
    { NDS32_LINUX_SIGRETURN_INSTR, -1 },
    { TRAMP_SENTINEL_INSN }
  },
  nds32_linux_sigreturn_init
};

static struct tramp_frame nds32_linux_rt_sigreturn_tramp_frame = {
  SIGTRAMP_FRAME,
  4,
  {
    { NDS32_LINUX_RT_SIGRETURN_INSTR, -1 },
    { TRAMP_SENTINEL_INSN }
  },
  nds32_linux_rt_sigreturn_init
};

/* Supply register REGNUM from the buffer specified by GREGS and LEN
   in the general-purpose register set REGSET to register cache
   REGCACHE.  If REGNUM is -1, do this for all registers in REGSET.  */

void
nds32_linux_supply_gregset (const struct regset *regset,
			    struct regcache *regcache, int regnum,
			    const void *gregs, size_t len)
{
  const gdb_byte *regp = (const gdb_byte *) gregs;
  int i;

  for (i = NDS32_R0_REGNUM; i < NDS32_LINUX_NUM_GPRS; i++)
    {
      /* FIXME: Review me after <linux/user.h>, <asm/ptrace.h>, and SR regs
	 spec clear. [Harry@Mar.14.2006] */
      if ((regnum == i || regnum == -1)
	  && nds32_ptreg_map[i] != -1)
	regcache_raw_supply (regcache, i, regp + nds32_ptreg_map[i] * 4);
    }
}

/* Collect register REGNUM from the register cache REGCACHE and store
   it in the buffer specified by GREGS and LEN as described by the
   general-purpose register set REGSET.  If REGNUM is -1, do this for
   all registers in REGSET.  */

void
nds32_linux_collect_gregset (const struct regset *regset,
			     const struct regcache *regcache,
			     int regnum, void *gregs, size_t len)
{
  gdb_byte *regp = (gdb_byte *) gregs;
  int i;

  for (i = NDS32_R0_REGNUM; i < NDS32_LINUX_NUM_GPRS; i++)
    {
      if ((regnum == i || regnum == -1)
	  && nds32_ptreg_map[i] != -1)
	regcache_raw_collect (regcache, i, regp + nds32_ptreg_map[i] * 4);
    }
}

static const struct regset nds32_linux_gregset =
  {
    NULL, nds32_linux_supply_gregset, nds32_linux_collect_gregset
  };

/* Implement gdbarch_iterate_over_regset_sections method.  */

static void
nds32_linux_iterate_over_regset_sections (struct gdbarch *gdbarch,
					  iterate_over_regset_sections_cb *cb,
					  void *cb_data,
					  const struct regcache *regcache)
{
  cb (".reg", NDS32_LINUX_SIZEOF_GREGSET, &nds32_linux_gregset,
      NULL, cb_data);
  /* TODO: fpreg ".reg2" */
}

/* Get target description from core file.  */

static const struct target_desc *
nds32_linux_core_read_description (struct gdbarch *gdbarch,
				   struct target_ops *target,
				   bfd *abfd)
{
  return tdesc_nds32_linux;
}

/* Implement the "get_syscall_number" gdbarch method.  */

static LONGEST
nds32_linux_get_syscall_number (struct gdbarch *gdbarch,
				ptid_t ptid)
{
  struct regcache *regs = get_thread_regcache (ptid);
  ULONGEST sysno;

  /* Getting the system call number from orig_r0.  */
  regcache_cooked_read_unsigned (regs, NDS32_ORIG_R0_REGNUM, &sysno);

  return sysno;
}

static void
nds32_linux_init_abi (struct gdbarch_info info, struct gdbarch *gdbarch)
{
  linux_init_abi (info, gdbarch);

  /* Set the sigtramp frame sniffer.  */
  tramp_frame_prepend_unwinder (gdbarch,
				&nds32_linux_sigreturn_tramp_frame);
  tramp_frame_prepend_unwinder (gdbarch,
				&nds32_linux_rt_sigreturn_tramp_frame);

  /* GNU/Linux uses SVR4-style shared libraries.  */
  set_solib_svr4_fetch_link_map_offsets (gdbarch,
					 svr4_ilp32_fetch_link_map_offsets);

  /* Enable TLS support.  */
  set_gdbarch_fetch_tls_load_module_address (gdbarch,
					     svr4_fetch_objfile_link_map);

  /* Core file support.  */
  set_gdbarch_iterate_over_regset_sections
    (gdbarch, nds32_linux_iterate_over_regset_sections);
  set_gdbarch_core_read_description (gdbarch,
				     nds32_linux_core_read_description);

  set_gdbarch_skip_solib_resolver (gdbarch, glibc_skip_solib_resolver);

  /* `catch syscall' */
  set_xml_syscall_file_name (gdbarch, "syscalls/nds32-linux.xml");
  set_gdbarch_get_syscall_number (gdbarch, nds32_linux_get_syscall_number);
}

/* Provide a prototype to silence -Wmissing-prototypes.  */
extern initialize_file_ftype _initialize_nds32_linux_tdep;

void
_initialize_nds32_linux_tdep (void)
{
  gdbarch_register_osabi (bfd_arch_nds32, 0, GDB_OSABI_LINUX,
			  nds32_linux_init_abi);

  /* Initialize the Linux target description.  */
  initialize_tdesc_nds32_linux ();
}
