/* RISC-V simulator.

   Copyright (C) 2005-2022 Free Software Foundation, Inc.
   Contributed by Mike Frysinger.

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

#ifndef SIM_MAIN_H
#define SIM_MAIN_H

#include "sim-basics.h"
#include "machs.h"
#include "sim-base.h"

typedef int64_t signed64;
typedef uint64_t unsigned64;
typedef int32_t signed32;
typedef uint32_t unsigned32;

/* The values are from ./build/sim/riscv/targ-vals.h.  (generated from older-binutils gentmap.c /sim/common/nltvals.def) */
/* syscall values */
#define TARGET_SYS_access 1033
#define TARGET_SYS_brk 214
#define TARGET_SYS_chdir 49
#define TARGET_SYS_close 57
#define TARGET_SYS_dup 23
#define TARGET_SYS_exit 93
#define TARGET_SYS_exit_group 94
#define TARGET_SYS_faccessat 48
#define TARGET_SYS_fcntl 25
#define TARGET_SYS_fstat 80
#define TARGET_SYS_fstatat 79
#define TARGET_SYS_getcwd 17
#define TARGET_SYS_getdents 61
#define TARGET_SYS_getegid 177
#define TARGET_SYS_geteuid 175
#define TARGET_SYS_getgid 176
#define TARGET_SYS_getmainvars 2011
#define TARGET_SYS_getpid 172
#define TARGET_SYS_gettimeofday 169
#define TARGET_SYS_getuid 174
#define TARGET_SYS_kill 129
#define TARGET_SYS_link 1025
#define TARGET_SYS_lseek 62
#define TARGET_SYS_lstat 1039
#define TARGET_SYS_mkdir 1030
#define TARGET_SYS_mmap 222
#define TARGET_SYS_mremap 216
#define TARGET_SYS_munmap 215
#define TARGET_SYS_open 1024
#define TARGET_SYS_openat 56
#define TARGET_SYS_pread 67
#define TARGET_SYS_pwrite 68
#define TARGET_SYS_read 63
#define TARGET_SYS_rt_sigaction 134
#define TARGET_SYS_stat 1038
#define TARGET_SYS_time 1062
#define TARGET_SYS_times 153
#define TARGET_SYS_uname 160
#define TARGET_SYS_unlink 1026
#define TARGET_SYS_write 64
#define TARGET_SYS_writev 66
#define TARGET_SYS_rename 1034

typedef union FRegisterValue
{
  uint64_t     v[2];
  uint32_t     w[4];

  int64_t      V[2];
  int32_t      W[4];

  float        S[4];
  double       D[2];

} FRegister;

struct _sim_cpu {
  union {
    unsigned_word regs[32];
    struct {
      /* These are the ABI names.  */
      unsigned_word zero, ra, sp, gp, tp;
      unsigned_word t0, t1, t2;
      unsigned_word s0, s1;
      unsigned_word a0, a1, a2, a3, a4, a5, a6, a7;
      unsigned_word s2, s3, s4, s5, s6, s7, s8, s9, s10, s11;
      unsigned_word t3, t4, t5, t6;
    };
  };
  union {
    FRegister fpregs[32];
    struct {
      /* These are the ABI names.  */
      unsigned_word ft0, ft1, ft2, ft3, ft4, ft5, ft6, ft7;
      unsigned_word fs0, fs1;
      unsigned_word fa0, fa1, fa2, fa3, fa4, fa5, fa6, fa7;
      unsigned_word fs2, fs3, fs4, fs5, fs6, fs7, fs8, fs9, fs10, fs11;
      unsigned_word ft8, ft9, ft10, ft11;
    };
  };
  sim_cia pc;
  sim_cia endbrk;

  struct {
#define DECLARE_CSR(name, ...) unsigned_word name;
#include "opcode/riscv-opc.h"
#undef DECLARE_CSR
  } csr;

  sim_cpu_base base;
};

struct atomic_mem_reserved_list;
struct atomic_mem_reserved_list {
  struct atomic_mem_reserved_list *next;
  address_word addr;
};

struct riscv_sim_state {
  struct atomic_mem_reserved_list *amo_reserved_list;
};
#define RISCV_SIM_STATE(sd) ((struct riscv_sim_state *) STATE_ARCH_DATA (sd))

extern void step_once (SIM_CPU *);
extern void initialize_cpu (SIM_DESC, SIM_CPU *, int);
extern void initialize_env (SIM_DESC, const char * const *argv,
			    const char * const *env);

#define DEFAULT_MEM_SIZE (64 * 1024 * 1024)

#define RISCV_XLEN(cpu) MACH_WORD_BITSIZE (CPU_MACH (cpu))

#endif
