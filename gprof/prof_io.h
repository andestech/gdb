#ifndef prof_io_h
#define prof_io_h
// ============================================================================
// prof_io.h
// Copyright 2006 Andes Technology Corporation
//
// This file is part of GNU Binutils.
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
// ============================================================================

#define VEP_INIT_PC 0x0
#define IT_MASK     0x40
#define TIMELINE_LIMIT 		0x60000000

extern char prof_temp_file[32];

// blocks to be dumped for timeline based call-graph
typedef struct tcgheaderT
{   unsigned long long insn_cnt;
    unsigned long long cycle_cnt;
    unsigned long long call_cnt;
    unsigned long long to_bb_cnt;
    unsigned long long to_fn_call_tag;				// total function call tag count
    unsigned long long to_fn_rt_tag;				// total function return tag count
    unsigned long long to_br_tag;				// total branch tag count
    unsigned long long to_du_call_tag;				// total dummy function call tag count
    unsigned long long to_du_rt_tag;				// total dummy function return tag count
    unsigned int max_func_level;
    unsigned int min_func_level;
    unsigned int func_cnt;
    unsigned int pool_size;
    unsigned char timeline_level;
    unsigned char version;
    unsigned char fill[6];					// for 32/64 compatibility
} tcgheader;

typedef struct tcgfnameT
{   unsigned long long faddr;
    unsigned int fnoffset;
    unsigned int sfoffset;
    unsigned int lineno;
    unsigned int filler;
} tcgfname;

typedef struct tcgpageT
{   unsigned long long to_insn_cnt;
    unsigned long long to_cycle_cnt;
    unsigned long long rec_start;
    unsigned long long rec_end;
} tcgpage;

// used by timeline temporary file
typedef struct ttcgnodeT1
{   unsigned long long func_addr;
    unsigned long long parent_addr;
    unsigned long long to_insn_cnt;
    unsigned long long to_cycle_cnt;
    unsigned char direction;
    unsigned char fill[7];					// for 32/64 compatibility
} ttcgnode1;

typedef struct ttcgnodeT2
{   unsigned long long func_addr;
    unsigned long long parent_addr;
    unsigned long long to_insn_cnt;
    unsigned long long to_cycle_cnt;
    unsigned char branch_data;
    unsigned char return_data;
    unsigned char direction;
    unsigned char fill[5];					// for 32/64 compatibility
} ttcgnode2;

typedef struct tcgnodeT1
{   unsigned int func_id;
    unsigned int parent_id;
    unsigned long long to_insn_cnt;
    unsigned long long to_cycle_cnt;
    unsigned char direction;
    unsigned char fill[7];					// for 32/64 compatibility
} tcgnode1;

typedef struct tcgnodeT2
{   unsigned int func_id;
    unsigned int parent_id;
    unsigned long long to_insn_cnt;
    unsigned long long to_cycle_cnt;
    unsigned char branch_data;
    unsigned char return_data;
    unsigned char direction;
    unsigned char fill[5];					// for 32/64 compatibility
} tcgnode2;

extern tcgheader tcghdr;  // report header
extern tcgfname *tcgfni;  // report function name index
extern char *tcgfnp;      // report function name pool
extern tcgpage *tcgpages; // report pages
extern tcgnode1 *tcgnodes1; // report call-graph arcs

typedef struct SymListNodeT
{   Sym *sym;
    bfd_vma caller_addr;
    struct SymListNodeT *next;
    unsigned long long self_insn_cnt;
    unsigned long long self_cycle_cnt;
    unsigned long long child_insn_cnt;
    unsigned long long child_cycle_cnt;
    unsigned long long branch_taken_cnt;
    unsigned long long branch_misprediction_cnt;
    unsigned long long BTB_branch_cnt;
    unsigned long long icache_replace_cnt;
    unsigned long long icache_miss_cnt;
    unsigned long long icache_access_cnt;
    unsigned long long dcache_replace_cnt;
    unsigned long long dcache_miss_cnt;
    unsigned long long dcache_access_cnt;
    unsigned long long bb_cnt;
    unsigned long long pos;
    unsigned int calls;
} SymListNode;

extern SymListNode *exec_stack; // program execution stack


extern int gErrorCode;

#define u8  unsigned char
#define u16 unsigned short
#define u32 unsigned long
#define u64 unsigned long long


typedef struct __attribute__ ((packed))_header{
	u8	tag	;
	u16	level;
	u32	func_id;
	u32	parent_id;
}Header_t;

typedef struct {
	int low:4,
		high:4;
}BYTE;


#endif // prof_io_h
