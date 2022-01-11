/* This testcase is part of GDB, the GNU debugger.

   Copyright 2009-2022 Free Software Foundation, Inc.

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

int
main (void)
{
  //register int lala_s0 asm("s0") = 0x08;
  //register int lala_s1 asm("s1") = 0x09;
  register int lala_a0 asm("a0") = 0x0A;
  register int lala_a1 asm("a1") = 0x0B;
  register int lala_a2 asm("a2") = 0x0C;
  register int lala_a3 asm("a3") = 0x0D;
  register int lala_a4 asm("a4") = 0x0E;
  register int lala_a5 asm("a5") = 0x0F;
  register int lala_a6 asm("a6") = 0x10;
  register int lala_a7 asm("a7") = 0x11;
  register int lala_s2 asm("s2") = 0x12;
  register int lala_s3 asm("s3") = 0x13;
  register int lala_s4 asm("s4") = 0x14;
  register int lala_s5 asm("s5") = 0x15;
  register int lala_s6 asm("s6") = 0x16;
  register int lala_s7 asm("s7") = 0x17;
  register int lala_s8 asm("s8") = 0x18;
  register int lala_s9 asm("s9") = 0x19;
  register int lala_s10 asm("s10") = 0x1A;
  register int lala_s11 asm("s11") = 0x1B;
  register int lala_t3 asm("t3") = 0x1C;
  register int lala_t4 asm("t4") = 0x1D;
  register int lala_t5 asm("t5") = 0x1E;
  register int lala_t6 asm("t6") = 0x1F;

  return 0; /* break-at-exit */
}
