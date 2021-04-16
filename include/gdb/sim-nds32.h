/* This file defines the interface between the NDS32 simulator and GDB.

   Copyright 2009-2013 Free Software Foundation, Inc.

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

#ifndef SIM_NDS32_H
#define SIM_NDS32_H

enum sim_nds32_regs
{
  SIM_NDS32_TA_REGNUM = 15,
  SIM_NDS32_FP_REGNUM = 28,
  SIM_NDS32_GP_REGNUM = 29,
  SIM_NDS32_LP_REGNUM = 30,
  SIM_NDS32_SP_REGNUM = 31,

  SIM_NDS32_PC_REGNUM = 32,

  SIM_NDS32_FD0_REGNUM = 34,

  SIM_NDS32_PSW_REGNUM = 66,
  SIM_NDS32_ITB_REGNUM = 67,
  SIM_NDS32_IFCLP_REGNUM = 68,
  SIM_NDS32_LB_REGNUM = 69,
  SIM_NDS32_LE_REGNUM = 70,
  SIM_NDS32_LC_REGNUM = 71,
};

#endif
