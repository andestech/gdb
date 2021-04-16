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

#ifndef _NDS32_SYSCALL_H_
#define _NDS32_SYSCALL_H_
#include "gdb/callback.h"

/* Check
	   gdb: include/gdb/callback.h
	kernel: arch/nds32/include/asm/unistd.h
	newlib: libgloss/nds32/syscall.h
   for details.  */

#define CB_SYS_BASE		0x1000
#define CB_SYS_link		(CB_SYS_BASE + 9)
#define CB_SYS_access		(CB_SYS_BASE + 33)
#define CB_SYS_times		(CB_SYS_BASE + 43)
#define CB_SYS_brk		(CB_SYS_BASE + 45)
#define CB_SYS_ioctl		(CB_SYS_BASE + 54)
#define CB_SYS_setrlimit	(CB_SYS_BASE + 75)
#define CB_SYS_getrlimit	(CB_SYS_BASE + 76)
#define CB_SYS_gettimeofday	(CB_SYS_BASE + 78)
#define CB_SYS_settimeofday	(CB_SYS_BASE + 79)
#define CB_SYS_mmap		(CB_SYS_BASE + 90)
#define CB_SYS_munmap		(CB_SYS_BASE + 91)
#define CB_SYS_uname		(CB_SYS_BASE + 122)
#define CB_SYS_mprotect		(CB_SYS_BASE + 125)
#define CB_SYS_llseek		(CB_SYS_BASE + 140)
#define CB_SYS_readv		(CB_SYS_BASE + 145)
#define CB_SYS_writev		(CB_SYS_BASE + 146)
#define CB_SYS_nanosleep	(CB_SYS_BASE + 162)
#define CB_SYS_getpagesize	(CB_SYS_BASE + 166)
#define CB_SYS_sigaction	(CB_SYS_BASE + 174)
#define CB_SYS_ugetrlimit	(CB_SYS_BASE + 191)
#define CB_SYS_mmap2		(CB_SYS_BASE + 192)
#define CB_SYS_stat64		(CB_SYS_BASE + 195)
#define CB_SYS_lstat64		(CB_SYS_BASE + 196)
#define CB_SYS_fstat64		(CB_SYS_BASE + 197)
#define CB_SYS_getuid32		(CB_SYS_BASE + 199)
#define CB_SYS_getgid32		(CB_SYS_BASE + 200)
#define CB_SYS_geteuid32	(CB_SYS_BASE + 201)
#define CB_SYS_getegid32	(CB_SYS_BASE + 202)
#define CB_SYS_setuid32		(CB_SYS_BASE + 213)
#define CB_SYS_setgid32		(CB_SYS_BASE + 214)
#define CB_SYS_fcntl64		(CB_SYS_BASE + 221)
#define CB_SYS_exit_group	(CB_SYS_BASE + 248)

#define CB_SYS_NDS32_isatty	(CB_SYS_BASE + 0x202)
#define CB_SYS_NDS32_errno	(CB_SYS_BASE + 0x203)
#define CB_SYS_NDS32_getcmdline	(CB_SYS_BASE + 0x204)

#if 0
  /* More standard syscalls.  */
  {CB_SYS_lstat,	19},
  {CB_SYS_truncate,	21},
  {CB_SYS_ftruncate,	22},
  {CB_SYS_pipe,		23},
#endif

#define LINUX_SYS_BASE		0x5000
#define TARGET_LINUX_SYS_write	(LINUX_SYS_BASE + 4)
#define TARGET_LINUX_SYS_lseek	(LINUX_SYS_BASE + 19)
#define TARGET_LINUX_SYS_stat	(LINUX_SYS_BASE + 106)
#define TARGET_LINUX_SYS_lstat	(LINUX_SYS_BASE + 107)
#define TARGET_LINUX_SYS_fstat	(LINUX_SYS_BASE + 108)

void nds32_syscall (sim_cpu *cpu, int swid, sim_cia cia);

extern CB_TARGET_DEFS_MAP cb_nds32_libgloss_syscall_map[];
extern CB_TARGET_DEFS_MAP cb_nds32_linux_syscall_map[];
#endif
