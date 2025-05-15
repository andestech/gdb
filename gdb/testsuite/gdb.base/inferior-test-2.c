/* This testcase is part of GDB, the GNU debugger.

   Copyright 2021-2022 Free Software Foundation, Inc.

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
#include<stdio.h>
int
bar2 (void)
{
  puts("bar2");/* set bar2 breakpoint here */
  return 0;
}

int
foo2 (void)
{
	puts("foo2");/* set foo2 breakpoint here */
	bar2 ();
  return 0;
}

int
main (void)
{
	puts("inferior-test2 main");/* set inferior-test2 main breakpoint here */
	foo2 ();
  return 0;
}
