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

#ifndef _NDS32_MAIN_SIM_H_
#define _NDS32_MAIN_SIM_H_

#include <stdint.h>

#include "sim-basics.h"
#include "sim-signal.h"
#include "sim-base.h"

typedef union {
  uint32_t u;
  int32_t s;

  struct
    {
      int16_t lo;
      int16_t hi;
    } b16;
  struct
    {
      int8_t b0;
      int8_t b1;
      int8_t b2;
      int8_t b3;
    } b8;
  struct
    {
      uint16_t lo;
      uint16_t hi;
    } ub16;
  struct
    {
      uint8_t b0;
      uint8_t b1;
      uint8_t b2;
      uint8_t b3;
    } ub8;
} reg_t;

typedef union
{
  int64_t d0;

  struct
    {
      uint32_t w0;
      int32_t w1;
    } b32;
} union64_t;

enum nds32_internal_flags
{
  /* Set NIF_EX9 to indicate the instructions is executed in ITB.
     JAL and J work differently in ITB.  */
  NIF_EX9 = 1,
  /* Set NIF_BRANCH to indicate an branch is taken.  */
  NIF_BRANCH = 2,
};

struct _sim_cpu {
  /* 32 general purpose registers. */
  reg_t reg_gpr[32];
#define CCPU_GPR	(cpu->reg_gpr)

  /* User registers. 32 group x 32 USR */
  reg_t reg_usr[32 * 32];
#define CCPU_USR	(cpu->reg_usr)

  /* System registers.  Major x Minor x Ext */
  reg_t reg_sr[8 * 16 * 8];
#define CCPU_SR		(cpu->reg_sr)

  /* Floating-point registers. 32 single union 32 double. FIXME */
  reg_t reg_fpr[64];
#define CCPU_FPR	(cpu->reg_fpr)
  reg_t reg_fpcfg;
#define CCPU_FPCFG	(cpu->reg_fpcfg)
  reg_t reg_fpcsr;
#define CCPU_FPCSR	(cpu->reg_fpcsr)

  enum nds32_internal_flags iflags;
  /* If NIF_BRANCH, this is the destination address.  */
  SIM_ADDR	baddr;

  sim_cpu_base base;
};

struct sim_state {
  sim_cpu *cpu[MAX_NR_PROCESSORS];
#define STATE_BOARD_DATA(sd) (&(sd)->board)

  char cmdline[256];		/* cmdline buffer for -mcrt-arg hacking. */

  struct bfd *interp_bfd;	/* For Linux dynamic linker.  */
  uint32_t interp_base;		/* Base address of where interp is loaded. */
  uint32_t exec_base;		/* Base address of where executable is loaded. */

  sim_state_base base;
};

#include "sim-engine.h"
#include "sim-options.h"

#endif
