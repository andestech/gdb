dnl Process this file with autoconf to produce a configure script.
AC_PREREQ(2.64)dnl
AC_INIT(Makefile.in)
sinclude(../common/acinclude.m4)

SIM_AC_COMMON

SIM_AC_OPTION_ENDIAN
SIM_AC_OPTION_ALIGNMENT(NONSTRICT_ALIGNMENT)
SIM_AC_OPTION_WARNINGS

AC_CHECK_FUNCS([ \
	getuid getgid geteuid getegid setuid setgid \
	access fcntl link \
	gettimeofday times \
	getrlimit setrlimit \
	nanosleep \
])

AC_CHECK_HEADERS([ \
	sys/ioctl.h \
	sys/times.h \
])

SIM_AC_OUTPUT
