/* Common target dependent code for GDB on nds32 systems.

   Copyright (C) 2006-2015 Free Software Foundation, Inc.
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


/* This file is used for elc-check when downloading an ELF file to
   target board, and it is synced from nds32-sid.  */

#ifndef _NDS_ELF_CHECK
#define _NDS_ELF_CHECK

//#define TEST_ELF_CHECK_FUNC
#ifdef TEST_ELF_CHECK_FUNC
#include <stdio.h>
#include <stdlib.h>
#endif

#include <stdio.h>
#include <stdarg.h>

#ifdef __cplusplus
extern "C"
{
#else
#include <stdbool.h>
#endif //#ifdef __cplusplus
#define __STDC_FORMAT_MACROS 1
#include <ctype.h>
#include <errno.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <elf.h>
#include <fcntl.h>
#include <strings.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

//#define DEBUG_NEC

#ifndef EM_RISCV
#define EM_RISCV		243
#endif

#ifndef SHT_RISCV_ATTRIBUTES
#define SHT_RISCV_ATTRIBUTES	0x70000003
#endif

#define Tag_File		1
#define Tag_arch		4
#define Tag_priv_spec		5
#define Tag_priv_spec_minor	6
#define Tag_priv_spec_revision	7
#define Tag_strict_align	8
#define Tag_stack_align		9

/* type from gdb for both rv32 and rv64. */
typedef unsigned long long reg_t;

typedef reg_t (*CALLBACK_FUNC) (unsigned int index);

typedef enum ELF_Fail_Type
{
  EFT_NONE,
  EFT_WARNING,
  EFT_ERROR
} ELF_Fail_Type;

static char *nec_buf;
static unsigned int nec_buf_len;

static void
NEC_buf_init (char *buf, unsigned int len)
{
  nec_buf = buf;
  nec_buf_len = len;
  buf[0] = '\0';
}

#define OUTPUT_MSG_MAX_LEN			512
#define NEC_output(fmt, ...) \
do \
{ \
  char _temp[OUTPUT_MSG_MAX_LEN]; \
  NEC_snprintf (_temp, sizeof (_temp), fmt, ##__VA_ARGS__); \
  NEC_strcat_safety (nec_buf, nec_buf_len, _temp); \
} \
while (0)

#define die(fmt, ...) \
do \
{ \
  NEC_output(fmt, ##__VA_ARGS__); \
  return -1; \
} \
while (0)

	//NDS32 strcat for avoiding buf overflow
static inline void
NEC_strcat_safety (char *destination, unsigned int destination_size,
		   char *source)
{
  strncat (destination, source, destination_size - strlen (destination) - 1);
}

static inline void
NEC_itoa (unsigned int value, char *buf, const unsigned int base)
{
  char temp[10] = "\0", ch;
  int len = 1, index;

  while (value > 0)
    {
      ch = value % base;
      value = value / base;
      if (ch >= 10)
	ch = ch + 'a' - 10;
      else
	ch = ch + '0';
      temp[len++] = ch;
    }
  len--;

  index = len;
  while (index >= 0)
    {
      buf[index] = temp[len - index];
      index--;
    }
}

static inline void
NEC_format (char *buf, unsigned int width)
{
  unsigned int len = strlen (buf);
  memmove (buf + (width - len), buf, len + 1);
  memset (buf, ' ', (width - len));
}

static void
NEC_snprintf (char *buf, size_t size, const char *str, ...)
{
#define TEMP_SZ	100
  int width;
  size_t len = 0 ;
  va_list ap;
  char token, temp[TEMP_SZ];
  buf[0] = '\0';


  va_start (ap, str);
  while (*str != '\0')
    {
      if (*str != '%')
	buf[len++] = *str;
      else			//*str == '%'
	{
	  token = *(++str);

	  width = 0;
	  while (token >= '0' && token <= '9')
	    {
	      width *= 10;
	      width += token - '0';
	      token = *(++str);
	    }

	  switch (token)
	    {
	    case 'd':
	      NEC_itoa (va_arg (ap, unsigned int), temp, 10);
	      break;
	    case 'x':
	      NEC_itoa (va_arg (ap, unsigned int), temp, 16);
	      break;
	    case 's':
	      strncpy (temp, va_arg (ap, char *), sizeof (temp) - 1);
	      temp[TEMP_SZ - 1] = '\0';
	      break;
	    }

	  if (width != 0)
	    NEC_format (temp, width);

	  buf[len++] = '\0';
	  NEC_strcat_safety (buf, size, temp);
	  len = strlen (buf);

	  if (len > size)
	    /* The buffer is full.  */
	    return;
	}

      str++;
    }
  buf[len] = '\0';
#undef TEMP_SZ
}

	//NDS32 Elf Check print
static inline void
NEC_print (ELF_Fail_Type type, const char *name,
	   const char *cpu, const char *elf, const char *error_message)
{
  switch (type)
    {
    case EFT_NONE:
      NEC_output ("\t | %9s | %9s | %14s\n", cpu, elf, name);
      break;
    case EFT_WARNING:
      NEC_output ("\t?| %9s | %9s | %14s Warning: %s\n", cpu, elf,
		  name, error_message);
      break;
    case EFT_ERROR:
      NEC_output ("\t!| %9s | %9s | %14s Error: %s\n", cpu, elf, name,
		  error_message);
      break;
    }
}

static inline bool
NEC_check_bool (ELF_Fail_Type type, const char *isa, bool cpu, bool elf)
{
  bool code;
  const char *NEC_MSG_ISA[2] = { "OFF", "ON" };
  if (!cpu && elf)
    code = 1;
  else
    {
      code = 0;
      type = EFT_NONE;
    }
  NEC_print (type, isa, NEC_MSG_ISA[cpu], NEC_MSG_ISA[elf],
	     "Not supported by CPU");
  return code;
}

#ifdef DEBUG_NEC
#define Debug_printf(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define Debug_printf(fmt, ...)
#endif

enum BASE_ISA
{
  BASE_ISA_RV32E,
  BASE_ISA_RV32I,
  BASE_ISA_RV64I,
  BASE_ISA_RV128I,
  BASE_ISA_COUNT
};

// The ISA string must begin with one of these four.
static const char *base_isas[BASE_ISA_COUNT] =
  { "RV32E", "RV32I", "RV64I", "RV128I" };

// RISC-V extensions must appear in this order.
static const char *riscv_extensions = "MAFDQLCBJTPVNX";

// RISC-V Non-standard extensions
static const char *riscv_x_extensions[] = { "Xv5-" };

enum RISCV_EXT
{
  EXT_X = 13,
  EXT_COUNT
};

enum RISCV_X_EXT
{
  X_EXT_V5,
  X_EXT_COUNT
};

typedef struct
{
  int base_isa_index;
  int base_isa_major;
  int base_isa_minor;
  bool ext_use[EXT_COUNT];
  int ext_major[EXT_COUNT];
  int ext_minor[EXT_COUNT];
  bool x_ext_use[X_EXT_COUNT];
  int x_ext_major[X_EXT_COUNT];
  int x_ext_minor[X_EXT_COUNT];
} riscv_elf_info;

static riscv_elf_info nds_info;

static void
init_elf_info (void)
{
  memset (&nds_info, 0, sizeof (nds_info));
}

static void
set_riscv_base_isa_info (int index, int major, int minor)
{
  nds_info.base_isa_index = index;
  nds_info.base_isa_major = major;
  nds_info.base_isa_minor = minor;
}

static void
get_riscv_base_isa_info (int *index, int *major, int *minor)
{
  *index = nds_info.base_isa_index;
  *major = nds_info.base_isa_major;
  *minor = nds_info.base_isa_minor;
}

static void
set_riscv_ext_info (int index, int major, int minor)
{
  if (index >= EXT_COUNT)
    return;

  nds_info.ext_use[index] = true;
  nds_info.ext_major[index] = major;
  nds_info.ext_minor[index] = minor;
}

/* Get standard extension info by index. */
static int
get_riscv_ext_info_i (int i, bool * use, int *major, int *minor)
{
  if (i >= EXT_COUNT)
    return -1;

  *use = nds_info.ext_use[i];
  *major = nds_info.ext_major[i];
  *minor = nds_info.ext_minor[i];
  return 0;
}

static void
set_riscv_x_ext_info (const char *ext, int major, int minor)
{
  nds_info.ext_use[EXT_X] = true;

  if (strncmp (ext, riscv_x_extensions[X_EXT_V5], 4) == 0)
    {
      nds_info.x_ext_use[X_EXT_V5] = true;
      nds_info.x_ext_major[X_EXT_V5] = major;
      nds_info.x_ext_minor[X_EXT_V5] = minor;
    }
}

/* Get non-standard extension info by index. */
static int
get_riscv_x_ext_info_i (int i, bool * use, int *major, int *minor)
{
  if (i >= X_EXT_COUNT)
    return -1;

  *use = nds_info.x_ext_use[i];
  *major = nds_info.x_ext_major[i];
  *minor = nds_info.x_ext_minor[i];
  return 0;
}

/* Does ELF use the extension located by index. */
static bool
elf_use_ext_i (int i, bool std)
{
  bool use;
  int major, minor;

  if (std)
    {
      if (get_riscv_ext_info_i (i, &use, &major, &minor) == -1)
	return false;
    }
  else
    {
      if (get_riscv_x_ext_info_i (i, &use, &major, &minor) == -1)
	return false;
    }

  return use;
}

static bool
cpu_support_std_ext (reg_t misa, char ext)
{
  // I-Extension allow E-Extension
  if (ext == 'E')
    {
      return ((misa & (1 << ('E' - 'A'))) != 0) || ((misa & (1 << ('I' - 'A'))) != 0);
    }

  int i = ext - 'A';

  return (misa & (1 << i)) != 0;
}

static bool
cpu_support_v5_ext (reg_t mmsc_cfg)
{
  bool CPU_ECD = (mmsc_cfg & (1 << 3)) != 0;
  bool CPU_EV5PE = (mmsc_cfg & (1 << 13)) != 0;

  return CPU_ECD && CPU_EV5PE;
}

static void
print_riscv_isa_version (void)
{
#ifdef DEBUG_NEC
  int elf_base_isa = 0, base_isa_major = 0, base_isa_minor = 0;
  get_riscv_base_isa_info (&elf_base_isa, &base_isa_major, &base_isa_minor);
  printf ("Base ISA: %s v%d.%d\n", base_isas[elf_base_isa],
	  base_isa_major, base_isa_minor);
  printf ("Extensions:\n");
  bool use;
  int major, minor;
  int i;
  for (i = 0; i < EXT_X; i++)
    {
      if (get_riscv_ext_info_i (i, &use, &major, &minor) == -1)
	break;
      if (use)
	printf ("%c v%d.%d\n", riscv_extensions[i], major, minor);
    }

  /* Non standard (X) Extension. */
  if (get_riscv_ext_info_i (EXT_X, &use, &major, &minor) != -1 && use)
    if (get_riscv_x_ext_info_i (X_EXT_V5, &use, &major, &minor) != -1 && use)
      printf ("%s v%d.%d\n", riscv_x_extensions[X_EXT_V5], major, minor);
#endif
}

static int indent = 0;

void
print_indent (const char *fmt, ...)
{
#ifdef DEBUG_NEC
  printf ("%*s", indent * 2, "");
  va_list args;
  va_start (args, fmt);
  vprintf (fmt, args);
  va_end (args);
#endif
}

typedef struct
{
  uint8_t *data;
  size_t size;
} buf_t;

static bool
in_buffer (const buf_t buf, const size_t off, const size_t len)
{
  return off < buf.size && buf.size - off >= len;
}

#define define_elfnn_get_riscv_attribute_section \
static int elfnn_get_riscv_attribute_section(buf_t file, buf_t *ret) \
{ \
  if (file.size < sizeof (Elf_Ehdr)) \
    die ("Not an ELF file.\n"); \
  const Elf_Ehdr *ehdr = (const Elf_Ehdr*) file.data; \
\
  if (ehdr->e_machine != EM_RISCV) \
    die ("e_machine is not EM_RISCV.\n"); \
\
  if (!in_buffer (file, ehdr->e_shoff, ehdr->e_shnum * sizeof (Elf_Shdr))) \
    die ("Truncated ELF sections header.\n"); \
\
  for (size_t i = 0; i < ehdr->e_shnum; ++i) \
    { \
      const Elf_Shdr* sec = (const Elf_Shdr*) (file.data + ehdr->e_shoff) + i; \
      if (sec->sh_type == SHT_RISCV_ATTRIBUTES) \
	{ \
	  if (!in_buffer (file, sec->sh_offset, sec->sh_size)) \
	    die ("Truncated SHT_RISCV_ATTRIBUTE section.\n"); \
	  ret->data = file.data + sec->sh_offset; \
	  ret->size = sec->sh_size; \
	  return 0; \
	} \
    } \
\
  /* No SHT_RISCV_ATTRIBUTES sections found.  */ \
  return 1; \
}

#define Elf_Ehdr Elf32_Ehdr
#define Elf_Shdr Elf32_Shdr
#define elfnn_get_riscv_attribute_section elf32_get_riscv_attribute_section
define_elfnn_get_riscv_attribute_section
#undef Elf_Ehdr
#undef Elf_Shdr
#undef elfnn_get_riscv_attribute_section
#define Elf_Ehdr Elf64_Ehdr
#define Elf_Shdr Elf64_Shdr
#define elfnn_get_riscv_attribute_section elf64_get_riscv_attribute_section
define_elfnn_get_riscv_attribute_section
#undef Elf_Ehdr
#undef Elf_Shdr
#undef elfnn_get_riscv_attribute_section

// Case-insensitive strchr
static char *
strcasechr (const char *s, int c)
{
  for (; *s != '\0'; ++s)
    if (tolower (*s) == tolower (c))
      return (char *) s;

  return NULL;
}

static int
parse_byte (const uint8_t ** buf, const uint8_t * end, uint8_t * ret)
{
  if (end - *buf < 1)
    die ("Truncated ELF attribute section");

  const uint8_t x = **buf;
  *buf = *buf + 1;
  *ret = x;
  return 0;
}

static int
parse_uint32le (const uint8_t ** buf, const uint8_t * end, uint32_t * ret)
{
  if (end - *buf < 4)
    die ("Truncated ELF attribute section");

  const uint32_t x0 = *(*buf + 0);
  const uint32_t x1 = *(*buf + 1);
  const uint32_t x2 = *(*buf + 2);
  const uint32_t x3 = *(*buf + 3);

  *buf = *buf + 4;
  *ret = x0 | x1 << 8 | x2 << 16 | x3 << 24;
  return 0;
}

static int
parse_ntbs (const uint8_t ** buf, const uint8_t * end, const char **ret)
{
  const char *s = (const char *) *buf;
  while (*buf < end && **buf != '\0')
    *buf = *buf + 1;

  if (*buf == end)
    die ("Truncated ELF attribute section");

  *buf = *buf + 1;		// Skip over '\0'
  *ret = s;
  return 0;
}

// See: https://en.wikipedia.org/wiki/LEB128
static int
parse_uleb128 (const uint8_t ** buf, const uint8_t * end, uint64_t * ret)
{
  uint64_t shift = 0;
  uint64_t value = 0;
  while (*buf < end)
    {
      value |= (**buf & 0x7f) << shift;
      if ((**buf & 0x80) == 0)
	break;
      *buf += 1;
      shift += 7;
    }

  if (*buf == end)
    die ("Truncated ELF attribute section");

  *buf += 1;

  *ret = value;
  return 0;
}

static int
parse_decimal (const char **s, int *ret)
{
  if (!isdigit (**s))
    die ("Invalid decimal number `%c'", **s);

  int n = 0;
  while (isdigit (**s))
    {
      n = n * 10 + (**s - '0');
      *s += 1;
    }

  *ret = n;
  return 0;
}

static int
parse_riscv_isa_version (const char **s, int *p_major, int *p_minor)
{
  // ISA version is in the format of "<major>p<minor>" where major and minor
  // numbers are decimal. We don't use strtol() here to avoid accepting
  // whitespaces or plus/minus signs.
  int major;
  if (parse_decimal (s, &major) == -1)
    return -1;

  if (**s != 'p')
    die ("Major version number is not followed by `p'");
  *s += 1;

  int minor;
  if (parse_decimal (s, &minor) == -1)
    return -1;

  Debug_printf ("v%d.%d", major, minor);
  *p_major = major;
  *p_minor = minor;
  return 0;
}

static int
parse_riscv_base_isa (const char **s)
{
  int major = 0, minor = 0;
  for (int i = 0; i < BASE_ISA_COUNT; ++i)
    {
      if (strncasecmp (*s, base_isas[i], strlen (base_isas[i])) == 0)
	{
	  Debug_printf ("%.*s ", (int) strlen (base_isas[i]), *s);
	  *s += strlen (base_isas[i]);
	  // Followed by the version.
	  if (parse_riscv_isa_version (s, &major, &minor) == -1)
	    return -1;

	  set_riscv_base_isa_info (i, major, minor);
	  return 0;
	}
    }

  die ("Invalid base ISA `%s'", *s);
}

static int
parse_riscv_isa_string (const char *s)
{
  print_indent ("Base ISA: ");
  // The ISA string starts with the base ISA.
  if (parse_riscv_base_isa (&s) == -1)
    return -1;
  Debug_printf ("\n");

  // Skip over underscores
  while (*s == '_')
    ++s;

  // Followed by multiple extensions.
  print_indent ("Extensions:\n");
  ++indent;
  const char *ext = NULL;
  while (*s != '\0')
    {
      int major = 0, minor = 0;
      // The extensions must be ordered, so we check its position from the last
      // parsed extension.
      const char *e = strcasechr (riscv_extensions, *s);
      if (e == NULL)
	die ("Invalid extension `%c'", *s);
      if (e < ext)
	die ("Out of order extension `%c'", *s);
      else if (e == ext && tolower (*e) != 'x')
	die ("Duplicate extension `%c'", *s);
      else
	ext = e;

      if (tolower (*e) != 'x')
	{
	  // For standard extensions, it is followed by the version number.
	  print_indent ("%c ", *s++);
	  if (parse_riscv_isa_version (&s, &major, &minor) == -1)
	    return -1;
	  Debug_printf ("\n");

	  set_riscv_ext_info (e - riscv_extensions, major, minor);

	}
      else
	{
	  char x_ext[6] = { 0 };
	  // For non-standard extensions, it begins with a 'X' followed by
	  // multiple alphabets and the version.

	  // We speically handle v5- here because it contains a
	  // dash which may confuse the parser.
	  if (strncasecmp (s, "Xv5-", 4) == 0)
	    {
	      print_indent ("%.4s ", s);
	      s += 4;
	      strncpy (x_ext, riscv_x_extensions[X_EXT_V5], 6);
	    }
	  else
	    {
	      const char *end = s;
	      while (isalpha (*end))
		++end;
	      if (s + 1 == end)
		die ("Empty non-standard extension name");
	      print_indent ("%.*s ", (int) (end - s), s);
	      s = end;
	    }
	  if (parse_riscv_isa_version (&s, &major, &minor) == -1)
	    return -1;
	  Debug_printf ("\n");

	  set_riscv_x_ext_info (x_ext, major, minor);
	  // Each non-standard extension must be separated by an underscore ('_').
	  if (*s != '\0')
	    {
	      if (*s == '_')
		++s;
	      else
		die ("Non-standard extensions is not separated by `_'");
	    }
	}

      // Skip over underscores
      while (*s == '_')
	++s;
    }
  --indent;
  return 0;
}

static int
parse_riscv_attributes (const uint8_t * buf, const uint8_t * end)
{
  while (buf < end)
    {
      // Each attribute is a pair of tag and value. The value can be either a
      // null-terminated byte string or an ULEB128 encoded integer depending on
      // the tag.
      uint64_t tag;
      if (parse_uleb128 (&buf, end, &tag) == -1)
	return -1;
      switch (tag)
	{
	case Tag_arch:
	  {
	    // For Tag_arch, parse the arch substring.
	    const char *isa = NULL;
	    if (parse_ntbs (&buf, end, &isa) == -1)
	      return -1;
	    print_indent ("Tag_arch: %s\n", isa);
	    ++indent;
	    if (parse_riscv_isa_string (isa) == -1)
	      return -1;
	    --indent;
	    break;
	  }
#define TAG_CASE(tagname) \
	case tagname: \
	  { \
	    if (parse_uleb128(&buf, end, &tag) == -1) \
	    return -1; \
	    print_indent( #tagname ": %" PRIu64 "\n", tag); \
	    break; \
	  }
	  // For other tags, simply print out the integer.
	  TAG_CASE (Tag_priv_spec)
	  TAG_CASE (Tag_priv_spec_minor)
	  TAG_CASE (Tag_priv_spec_revision)
	  TAG_CASE (Tag_strict_align) TAG_CASE (Tag_stack_align)
#undef TAG_CASE
	default:
	  die ("Unknown RISCV attribute tag %" PRIu64, tag);
	}
    }
  return 0;
}

static int
parse_riscv_subsection (const uint8_t * buf, const uint8_t * end)
{
  while (buf < end)
    {
      // The "riscv" subsection must begin with a Tag_File, indicating that the
      // subcontent of the value is applied to the whole object file.
      const uint8_t *sub_begin = buf;
      uint64_t tag;
      if (parse_uleb128 (&buf, end, &tag) == -1)
	return -1;
      if (tag != Tag_File)
	die ("Non Tag_File in \"riscv\" subsection");
      print_indent ("Tag_File: ");
      ++indent;

      // Followed by a length field including the tag byte.
      uint32_t len;
      if (parse_uint32le (&buf, end, &len) == -1)
	return -1;
      Debug_printf ("Subsection (%#" PRIx32 ") {\n", len);
      if (end - sub_begin < len)
	die ("Truncated \"riscv\" attribute subsection");
      const uint8_t *sub_end = sub_begin + len;

      // Followed by the actual RISC-V attributes.
      parse_riscv_attributes (buf, sub_end);

      --indent;
      print_indent ("}\n");
      buf = sub_end;
    }
  return 0;
}

static int
parse_elf_attribute_section (const uint8_t * buf, const uint8_t * end)
{
  init_elf_info ();

  uint8_t version;
  if (parse_byte (&buf, end, &version) == -1)
    return -1;
  Debug_printf ("ELF Attribute Version: %c\n", version);

  // The first byte indicates the version and must be the ASCII character 'A'
  if ((char) version != 'A')
    die ("Unsupported ELF attribute version");

  // The section is divided into multiple subsections.
  while (buf < end)
    {
      // Each subsection begins with a 32-bit unsigned integer indicating its
      // length (including the length field itself).
      const uint8_t *sub_begin = buf;
      uint32_t len;
      if (parse_uint32le (&buf, end, &len) == -1)
	return -1;

      if (end - sub_begin < len)
	die ("Truncated ELF attribute subsection");
      const uint8_t *sub_end = sub_begin + len;

      // It is followed by the a null-terminated name which determines how to
      // parse the content. We only handle "riscv" here and ignore the rest.
      const char *name = NULL;
      if (parse_ntbs (&buf, sub_end, &name) == -1)
	return -1;
      print_indent ("Subsection \"%s\" (%#" PRIx32 ") {\n", name, len);
      ++indent;
      if (strcmp (name, "riscv") == 0)
	if (parse_riscv_subsection (buf, sub_end) == -1)
	  return -1;
      --indent;
      print_indent ("}\n");

      buf = sub_end;
    }
  return 0;
}

static int
elf_check (void *file_data, unsigned int file_size,
	   CALLBACK_FUNC reg_read_callback, char *buf, unsigned int len)
{
  unsigned int n_error = 0;
  buf_t file = {.data = (uint8_t *) file_data,.size = file_size };

  NEC_buf_init (buf, len);

  if (file.size < EI_NIDENT)
    die ("Not an ELF file.\n");

  const unsigned char *e_ident = (const unsigned char *) file.data;
  if (memcmp (e_ident, ELFMAG, SELFMAG) != 0)
    die ("Not an ELF file.\n");

  const bool is64 = e_ident[EI_CLASS] == ELFCLASS64;
  buf_t sec;
  int ret = is64 ? elf64_get_riscv_attribute_section (file, &sec)
    : elf32_get_riscv_attribute_section (file, &sec);

  if (ret == 1)
    /* No RISCV attribute sections found, exit normally.  */
    return 0;
  else if (ret == -1)
    return -1;

  Debug_printf
    ("RISC-V attribute section at file offset %#tx with size %#zx\n",
     sec.data - file.data, sec.size);

  ret = parse_elf_attribute_section (sec.data, sec.data + sec.size);

  if (ret == -1)
    return -1;

  print_riscv_isa_version ();

  ELF_Fail_Type error_type;

  /* Check BASE ISA. */
  int elf_base_isa = 0, base_isa_major = 0, base_isa_minor = 0;
  get_riscv_base_isa_info (&elf_base_isa, &base_isa_major, &base_isa_minor);

  int mxl = 0;
  reg_t CSR_misa;
  reg_t CSR_mmsc_cfg;
  CSR_misa = reg_read_callback (0x301);
  CSR_mmsc_cfg = reg_read_callback (0xFC2);
  mxl = (CSR_misa >> 30) & 0x3;
  if (mxl == 0)			/* 64bit CPU */
    mxl = (CSR_misa >> 62) & 0x3;

  error_type = EFT_NONE;
  /* BASE_ISA_COUNT has been checked. */
  if (elf_base_isa == BASE_ISA_RV32E)
    {
      /* mxl must be 1 (32bit) for RV32E */
      if (mxl != 1 || !cpu_support_std_ext (CSR_misa, 'E'))
	error_type = EFT_ERROR;
    }
  else
    {
      if (mxl != elf_base_isa || !cpu_support_std_ext (CSR_misa, 'I'))
	error_type = EFT_ERROR;
    }
  if (error_type == EFT_ERROR)
    {
      n_error++;
      NEC_output ("Error: Base ISA does not match\n"
		  "ELF: %s\nCPU : %s\n",
		  base_isas[elf_base_isa], base_isas[mxl]);
    }

  /* Check extensions. */
  NEC_output ("\t   %9s   %9s  \n", "CPU", "ELF");

  char temp[30];
  char ext_str[2] = { 0 };
  int i;
  for (i = 0; i < EXT_COUNT; i++)
    {
      ext_str[0] = riscv_extensions[i];
      NEC_snprintf (temp, sizeof (temp), "Extension '%s'", ext_str);
      if (NEC_check_bool (EFT_ERROR, temp,
			  cpu_support_std_ext (CSR_misa, riscv_extensions[i]),
			  elf_use_ext_i (i, true)))
	n_error++;

    }

  /* Non-standard extension. */
  if (elf_use_ext_i (X_EXT_V5, false))
    {
      if (NEC_check_bool (EFT_ERROR, "V5 Extension",
			  cpu_support_v5_ext (CSR_mmsc_cfg), true))
	n_error++;
    }

  if (n_error)
    {
      NEC_output ("Error: ELF and CPU mismatch\n"
		  "Total Error: %d\n", n_error);

      NEC_output
	("Usage error, Consult Andes Toolchains and their compatible Andes cores for the Toolchain-CPU compatibility.\n");
      NEC_output
	("The Loader Checking can be disabled under Debug Configuration.\n");
    }
  else
    NEC_output ("NDS ELF checking pass\n");

  if (n_error > 0)
    return -1;

  return 0;
}				//end of elf_check

#undef OUTPUT_MSG_MAX_LEN
#undef NEC_output
#undef die

#undef Tag_File
#undef Tag_arch
#undef Tag_priv_spec
#undef Tag_priv_spec_minor
#undef Tag_priv_spec_revision
#undef Tag_strict_align
#undef Tag_stack_align

#ifdef __cplusplus
}
#endif //#ifdef __cplusplus

#endif //end of _NDS_ELF_CHECK
