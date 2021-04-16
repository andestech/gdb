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

#ifndef NDS32_LINUX_TDEP_H
#define NDS32_LINUX_TDEP_H

void nds32_linux_supply_gregset (const struct regset *regset,
				 struct regcache *regcache, int regnum,
				 const void *gregs, size_t size);
void nds32_linux_collect_gregset (const struct regset *regset,
				  const struct regcache *regcache, int regnum,
				  void *gregs_buf, size_t len);

/* Mapping between the general-purpose registers in `struct user'
   format and GDB's register array layout.

   FIXME: fix me after <linux/user.h>, <asm/ptrace.h>,
   and SR regs spec clear. [Harry@Mar.14.2006]

   Current remap layout depend on Tom's implementation in kernel header,
   in ptrace.h and IR spec 0.1 (Jan.20.2006 version)
   [Harry@Mar.16.2006]

   Note: -1 means unable to get from ptrace syscall

   Renumber according to arch/nds32/include/asm/ptrace.h
   Not sure whether NDS32_r0 or NDS32_ORIG_r0 represents real $r0.
   (current use NDS32_ORIG_r0)
   Registers not mapped: NDS32_FUCOP_CTL, NDS32_osp. (42 & 43)
   [Rudolph@Aug.18.2010]  */

/* Map gdb regnum to pt_regs index.  */
static int nds32_ptreg_map[] =
{
    /* nds32-core */

    /* r0 - r25 */
    13, 14, 15, 16, 17, 18, 19, 20, 21, 22,
    23, 24, 25, 26, 27, 28, 29, 30, 31, 32,
    33, 34, 35, 36, 37, 38,
    /* r26 and r27 are reserved for kernel */
    -1, -1,
    /* fp, gp, lp, sp */
    39, 40, 41, 3,
    /* pc */
    2,

    /* nds32-linux */

    /* orig_r0 */
    4
};

#define NDS32_LINUX_NUM_GPRS (ARRAY_SIZE(nds32_ptreg_map))
#define NDS32_LINUX_SIZEOF_GREGSET (44 * 4)

/* Linux target description.  */
extern struct target_desc *tdesc_nds32_linux;

#endif
