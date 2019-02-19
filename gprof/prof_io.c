// ============================================================================
// prof_io.c - Input and output from/to prof.out files.
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
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
// ============================================================================
#define _LARGEFILE_SOURCE
#define _FILE_OFFSET_BITS 64
#include "gprof.h"
#include "search_list.h"
#include "source.h"
#include "symtab.h"
#include "corefile.h"
#include "call_graph.h"
#include "gmon_io.h"
#include "gmon_out.h"
#include "prof_io.h"
#include "gmon.h"		// Fetch header for old format.
#include "hertz.h"
#include "libiberty.h"
#include "timeline.h"
#include "utils.h"

#undef GMON_VERSION
#define GMON_VERSION 2

extern time_t TotalProfileTime, ReadProfOutDataTime, ParsingProfileDataTime, WriteTemplateFileTime, ReadTemplateFileTime, ProcessDataTime, WriteTimelineBinFileTime, StartTime, EndTime;
enum{
	LOW_NIBBLE,
	HIGH_NIBBLE
}NIBBLE;

SymListNode *exec_stack=NULL;

static FILE *temp_fd=NULL;
char prof_temp_file[32]="prof-xxxxx.tmp";
bfd_vma CurrentPC = 0;

int function_level = 0, min_function_level = 0, max_function_level = 1;	// Due to function level of profile on is offset to 1
static unsigned long long temp_file_pos=0;
static unsigned long long current_insn_cnt=0;
static unsigned long long current_cycle_cnt=0;
static unsigned long long total_BTB_branch_count=0;
static unsigned long long total_branch_taken_count=0;
static unsigned long long total_branch_mispred_count=0;
static unsigned long long total_return_mispred_count=0;

typedef struct prof1dataT
{   bfd_vma pc;
    unsigned int icnt;
    unsigned int ccnt;
} prof1data;

typedef struct prof1data2T
{   prof1data data;
    bfd_vma tpc;
} prof1data2;

typedef struct prof2dataT
{   prof1data2 data;
    unsigned char br;
} prof2data;
/*
typedef struct prof3dataT
{   bfd_vma pc;
    unsigned short ccnt;
} prof3data;

typedef struct prof3data2T
{   prof3data data;
    bfd_vma tpc;
    unsigned char flags[3];
} prof3data2;
*/
typedef struct prof3data1T{
	unsigned short ccnt;
	bfd_vma tpc;
	unsigned char br;
	unsigned short ifetch;
}prof3data1;

typedef struct prof3data2T{
	unsigned short ccnt;
	unsigned char dfetch1;
	unsigned int dfetch2;
	unsigned int dfetch3;
}prof3data2;

typedef struct prof3data3T{
	unsigned short ccnt;
	unsigned short ifetch;
}prof3data3;

typedef struct profmdataT
{   prof1data data;
    unsigned char mode;
} profmdata;

typedef struct prof6dataT{
	bfd_vma pc;
	unsigned int icnt;
	unsigned int ccnt;
	bfd_vma tpc;
	unsigned char length[2];
}prof6data;

typedef struct prof7dataT{
	bfd_vma pc;
	unsigned int icnt;
	unsigned int ccnt;
	bfd_vma tpc;
	unsigned char length[3];
}prof7data;

typedef struct prof8dataT{
	bfd_vma pc;
	unsigned int icnt;
	unsigned int ccnt;
	bfd_vma tpc;
	unsigned char length[5];
}prof8data;

typedef struct prof9dataT{
	bfd_vma pc;
	unsigned int icnt;
	unsigned int ccnt;
	bfd_vma tpc;
	unsigned char br;
	unsigned char length[3];
}prof9data;

// uncomment to debug
//#define TRACE_STACK

#ifdef TRACE_STACK
// ----------------------------------------------------------------------------
// trace_stack
//
// This function displays the instruction and cycle counts of the caller and
// callee pair.
// ----------------------------------------------------------------------------
static void
trace_stack(SymListNode* callee,
            SymListNode* caller)
{   // dump timeline and stack node information first
    fprintf(stderr,"%llu\t%llu\t%s [%s:%u]\t%llu\t%llu\t%llu\t%llu\t",
            current_insn_cnt,current_cycle_cnt,get_name(callee->sym->name),
            (callee->sym->file==NULL)?"":callee->sym->file->name,
            callee->sym->line_num,
            callee->self_insn_cnt,callee->self_cycle_cnt,
            callee->child_insn_cnt,callee->child_cycle_cnt);
    if (caller!=NULL)
        fprintf(stderr,"X%08X@%s [%s:%u]\t%llu\t%llu\t%llu\t%llu\t",
                callee->caller_addr,get_name(caller->sym->name),
                (caller->sym->file==NULL)?"":caller->sym->file->name,
                caller->sym->line_num,
                caller->self_insn_cnt,caller->self_cycle_cnt,
                caller->child_insn_cnt,caller->child_cycle_cnt);
    else
        fprintf(stderr,"\t\t\t\t\t\t\t");

    // dump symbol information next
    fprintf(stderr,"%llu\t%llu\t%llu\t%llu\t",
            callee->sym->hist.total_insn_cnt,callee->sym->hist.total_cycle_cnt,
            callee->sym->cg.child_insn_cnt,callee->sym->cg.child_cycle_cnt);
    if (caller!=NULL)
        fprintf(stderr,"%llu\t%llu\t%llu\t%llu\n",
                caller->sym->hist.total_insn_cnt,caller->sym->hist.total_cycle_cnt,
                caller->sym->cg.child_insn_cnt,caller->sym->cg.child_cycle_cnt);
    else
        fprintf(stderr,"\t\t\t\n");
} // trace_stack
#endif // TRACE_STACK

#define u8  unsigned char
#define u16 unsigned short
#define u32 unsigned long
#define u64 unsigned long long


int PlaceData2Buf(u64 data, char * buf, int sz_idx, int da_idx, int nibble)
{
        unsigned int cnt;

        if((data >> 32)){
                for(cnt = 0; cnt < sizeof(u64); cnt++)
                        buf[da_idx++] = (u8)((data & ((u64)0xFF << (cnt * 8))) >> (cnt * 8));
        }else if((data >> 16)){
                for(cnt = 0; cnt < sizeof(u32); cnt++)
                        buf[da_idx++] = (u8)((data & ((u64)0xFF << (cnt * 8))) >> (cnt * 8));
        }else if((data >> 8)){
                for(cnt = 0; cnt < sizeof(u16); cnt++)
                        buf[da_idx++] = (u8)((data & ((u64)0xFF << (cnt * 8))) >> (cnt * 8));
        }else{
                for(cnt = 0; cnt < sizeof(u8); cnt++)
                        buf[da_idx++] = (u8)((data & ((u64)0xFF << (cnt * 8))) >> (cnt * 8));
        }

        if (nibble == HIGH_NIBBLE) {
                buf[sz_idx] = ((buf[sz_idx] & 0x0F) | (u8)0x10);
                while((cnt >>= 1))
                        buf[sz_idx] = ((buf[sz_idx] & 0x0F) | ((buf[sz_idx] & 0xF0) << 1));
        }
        else {
                buf[sz_idx] = ((buf[sz_idx] & 0xF0) | (u8)0x01);
                while((cnt >>= 1))
                        buf[sz_idx] = ((buf[sz_idx] & 0xF0) | ((buf[sz_idx] & 0x0F) << 1));
        }

        return da_idx;

}

int
lenof(unsigned long long p)
{
	int i;
	for ( i = 2; i >=0; i--)
	{
		if (p >> (1<<i)*8)
			break;
	}
	return (i >= 0)? 1<<(i+1): 1;
}

//place data from cnt/2.... 			: "pbuf"
//place size|size combo from 0 to cnt/2 : "abuf"
int
PackData(char *buf, long long data[], int cnt)
{
	int i;
	unsigned char a, b;
	int t = (cnt+1) >> 1;
	char * abuf = buf;
	char * pbuf = buf + t;

	for (i = 0; i < cnt; i++)
	{
		a = 0;
		b = 0;
		if (data[i] != -1) {
			a = lenof(data[i]);
			memcpy(pbuf, &data[i], a);
			pbuf += a;
		}
		i++;
		if ( i < cnt) {
			if (data[i] != -1) {
				b = lenof(data[i]);
				memcpy(pbuf, &data[i], b);
				pbuf += b;
			}
		}
		*abuf = (a <<4)|b;
		abuf++;
	}

	return pbuf - buf;
}



// ----------------------------------------------------------------------------
// prof_errmsg
//
// This function displays the cause of error and terminates the execution.
// 1 - read error
// 2 - write error
// 3 - memory error
// 4 - bad tag
// ----------------------------------------------------------------------------
static int
prof_errmsg(int errnum,
            const char *filename)
{   if (errnum==1)
        fprintf(stderr, _("%s: corrupted gmon data in file %s?\n"),
                whoami, filename);
    else if (errnum==2)
        fprintf(stderr, _("%s: failed to write temporary data to file %s?\n"),
                whoami, prof_temp_file);
    else if (errnum==3)
        fprintf(stderr, _("%s: memory allocation failed!\n"),whoami);
    else if (errnum==4)
        fprintf(stderr, _("%s: read bad tage from file %s?\n"),
                whoami, filename);

    // no return
    return -1;
} // prof_errmsg

// -----------------------------------------------------------------------------
// update_data_length
//
// This function writes position of summary profile data into function node.
// return code:
// 0 - success
// -----------------------------------------------------------------------------
static int
update_data_length(unsigned long long pos, unsigned int len)
{
	long original_offset = 0;
	unsigned int i = 0;
	unsigned char write_buffer[1] = {0};
	int result = 0;

	original_offset = ftell(temp_fd);
	result = fseek(temp_fd, (long)(-pos), SEEK_CUR);
	for(i = 0; i < sizeof(unsigned int); i++){
		write_buffer[0] = (unsigned char)((len & ((unsigned int)0xFF << (i * 8))) >> (i * 8));
		fwrite(write_buffer, 1, 1, temp_fd);
	}
	fflush(temp_fd);
	result = fseek(temp_fd, original_offset, SEEK_SET);

	return 0;
}

// ----------------------------------------------------------------------------
// write_tl9_temp
//
// This function writes temporary record for level 9 timeline analysis.
// return code:
// 0 - success
// 1 - write error
// ----------------------------------------------------------------------------
static int
write_tl9_temp(bfd_vma child_pc,
		bfd_vma parent_pc,
		unsigned char tag,
		short func_level,
		unsigned int func_length,
		unsigned long long to_insn_cnt,
		unsigned long long to_cycle_cnt,
		unsigned long long self_insn_cnt,
		unsigned long long self_cycle_cnt,
		unsigned long long child_insn_cnt,
		unsigned long long child_cycle_cnt,
		unsigned long long call_cnt,
		unsigned char branch_data,
		unsigned long long bb_cnt,
		unsigned long long br_taken_cnt,
		unsigned long long br_mis_cnt,
		unsigned long long BTB_br_cnt,
		unsigned long long icache_replace_cnt,
		unsigned long long icache_miss_cnt,
		unsigned long long icache_access_cnt,
		unsigned long long dcache_replace_cnt,
		unsigned long long dcache_miss_cnt,
		unsigned long long dcache_access_cnt,
		unsigned long long to_icache_replace_cnt,
		unsigned long long to_icache_miss_cnt,
		unsigned long long to_icache_access_cnt,
		unsigned long long to_dcache_replace_cnt,
		unsigned long long to_dcache_miss_cnt,
		unsigned long long to_dcache_access_cnt,
		unsigned int *len)
{
		int result = 0;
		Header_t H;
		unsigned char packbuf1[64];
		unsigned char packbuf2[128];
		unsigned char packbuf3[128];
		unsigned long long pMsg[32];
		char wbuf[512];
		char * wp;
		int plen1 = 0, plen2 = 0, plen3 = 0;
		char bytes4[4];

		wp = wbuf;

		H.tag = tag;
		H.level = func_level;
		H.func_id = child_pc;

		switch (tag){
			case PROFTYPE_FC9:
			case PROFTYPE_ON9:
				H.parent_id = (exec_stack == NULL)? VEP_INIT_PC: exec_stack->sym->addr;

				pMsg[0] = to_insn_cnt;
				pMsg[1] = to_cycle_cnt;
				plen1 = PackData(packbuf1, pMsg, 2);
				bytes4[0] = packbuf1[0];

				pMsg[0] = icache_replace_cnt;
				pMsg[1] = icache_miss_cnt;

				pMsg[2] = icache_access_cnt;
				pMsg[3] = dcache_replace_cnt;

				pMsg[4] = dcache_miss_cnt;
				pMsg[5] = dcache_access_cnt;

				plen2 = PackData(packbuf2, pMsg, 6);
				memcpy(&bytes4[1], packbuf2, 3);


				wp = mempcpy(wp, &H, sizeof(Header_t));
				wp = mempcpy(wp, &func_length, sizeof(long));
				wp = mempcpy(wp, bytes4, 4);
				wp = mempcpy(wp, packbuf1+1, plen1-1);
				wp = mempcpy(wp, &branch_data, 1);
				wp = mempcpy(wp, packbuf2+3, plen2 - 3);
				break;

			case PROFTYPE_FR9:
			case PROFTYPE_DUMMY_FR9:
			case PROFTYPE_OFF:
				if(exec_stack != NULL){
					H.parent_id = (exec_stack->next == NULL)? VEP_INIT_PC : exec_stack->next->sym->addr;
				}
				else {
					H.parent_id = parent_pc;
				}

				pMsg[0] = to_insn_cnt;
				pMsg[1] = to_cycle_cnt;
				plen1 = PackData(packbuf1, pMsg, 2);
				bytes4[0] = packbuf1[0];

				pMsg[0] = icache_replace_cnt;
				pMsg[1] = icache_miss_cnt;

				pMsg[2] = icache_access_cnt;
				pMsg[3] = dcache_replace_cnt;

				pMsg[4] = dcache_miss_cnt;
				pMsg[5] = dcache_access_cnt;

				plen2 = PackData(packbuf2, pMsg, 6);
				memcpy(&bytes4[1], packbuf2, 3);


				pMsg[0] = self_insn_cnt;
				pMsg[1] = self_cycle_cnt;

				pMsg[2] = child_insn_cnt;
				pMsg[3] = child_cycle_cnt;

				pMsg[4] = call_cnt;
				pMsg[5] = br_taken_cnt;

				pMsg[6] = br_mis_cnt;
				pMsg[7] = BTB_br_cnt;

				pMsg[8] = bb_cnt;
				pMsg[9] = -1;

				pMsg[10] = to_icache_replace_cnt;
				pMsg[11] = to_icache_miss_cnt;

				pMsg[12] = to_icache_access_cnt;
				pMsg[13] = to_dcache_replace_cnt;

				pMsg[14] = to_dcache_miss_cnt;
				pMsg[15] = to_dcache_access_cnt;

				plen3 = PackData(packbuf3, pMsg, 16);

				if(exec_stack != NULL){
	//				update_data_length((temp_file_pos - exec_stack->pos - sizeof(Header_t)),
	//									(temp_file_pos + (sizeof(rec) + plen1) - exec_stack->pos));
	//				update_data_length((temp_file_pos - exec_stack->pos - sizeof(Header_t)),
	//									(temp_file_pos + (sizeof(Header_t) + plen1 + 1 + plen2)) );
					update_data_length((temp_file_pos - exec_stack->pos - sizeof(Header_t)),
										temp_file_pos);
				}

				wp = mempcpy(wp, &H, sizeof(Header_t));
				wp = mempcpy(wp, bytes4, 4);
				wp = mempcpy(wp, packbuf1+1, plen1-1);
				wp = mempcpy(wp, &branch_data, 1);
				wp = mempcpy(wp, packbuf2+3, plen2-3);
				wp = mempcpy(wp, packbuf3, plen3);
				break;

			case PROFTYPE_BR9:
				H.parent_id = parent_pc;
				pMsg[0] = to_insn_cnt;
				pMsg[1] = to_cycle_cnt;
				plen1 = PackData(packbuf1, pMsg, 2);
				bytes4[0] = packbuf1[0];

				pMsg[0] = icache_replace_cnt;
				pMsg[1] = icache_miss_cnt;

				pMsg[2] = icache_access_cnt;
				pMsg[3] = dcache_replace_cnt;

				pMsg[4] = dcache_miss_cnt;
				pMsg[5] = dcache_access_cnt;

				plen2 = PackData(packbuf2, pMsg, 6);
				memcpy(&bytes4[1], packbuf2, 3);

				wp = mempcpy(wp, &H, sizeof(Header_t));
				wp = mempcpy(wp, bytes4, 4);
				wp = mempcpy(wp, packbuf1+1, plen1-1);
				wp = mempcpy(wp, &branch_data, 1);
				wp = mempcpy(wp, &parent_pc, 4);	// reserve 4 bytes
				wp = mempcpy(wp, packbuf2+3, plen2-3);
				break;
		}
		if (!fwrite(wbuf, wp - wbuf, 1, temp_fd))
		{
			gErrorCode = errno;
			return -1;
		}
		else
			*len = wp - wbuf;

		return 0;


}

 // write_tl9_temp

// ----------------------------------------------------------------------------
// write_tl8_temp
//
// This function writes temporary record for level 8 timeline analysis.
// return code:
// 0 - success
// 1 - write error
// ----------------------------------------------------------------------------
static int
write_tl8_temp(bfd_vma child_pc,
		bfd_vma parent_pc,
		unsigned char tag,
		short func_level,
		unsigned int func_length,
		unsigned long long to_insn_cnt,
		unsigned long long to_cycle_cnt,
		unsigned long long self_insn_cnt,
		unsigned long long self_cycle_cnt,
		unsigned long long child_insn_cnt,
		unsigned long long child_cycle_cnt,
		unsigned long long call_cnt,
		unsigned long long br_taken_cnt,
		unsigned long long br_mis_cnt,
		unsigned long long BTB_br_cnt,
		unsigned long long icache_replace_cnt,
		unsigned long long icache_miss_cnt,
		unsigned long long icache_access_cnt,
		unsigned long long dcache_replace_cnt,
		unsigned long long dcache_miss_cnt,
		unsigned long long dcache_access_cnt,
		unsigned int *len)
{
		int result = 0;
		Header_t H;
		unsigned char packbuf1[64];
		unsigned char packbuf2[256];
		unsigned long long pMsg[32];
		char wbuf[512];
		char * wp;
		int plen1 = 0, plen2 = 0;

		wp = wbuf;

		H.tag = tag;
		H.level = func_level;
		H.func_id = child_pc;

		switch (tag){
			case PROFTYPE_FC8:
			case PROFTYPE_ON8:
				H.parent_id = (exec_stack == NULL)? VEP_INIT_PC: exec_stack->sym->addr;

				pMsg[0] = to_insn_cnt;
				pMsg[1] = to_cycle_cnt;
				plen1 = PackData(packbuf1, pMsg, 2);

				wp = mempcpy(wp, &H, sizeof(Header_t));
				wp = mempcpy(wp, &func_length, sizeof(long));
				wp = mempcpy(wp, packbuf1, plen1);
				break;

			case PROFTYPE_FR8:
			case PROFTYPE_DUMMY_FR8:
			case PROFTYPE_OFF:
				if(exec_stack != NULL){
					H.parent_id = (exec_stack->next == NULL)? VEP_INIT_PC : exec_stack->next->sym->addr;
				}
				else {
					H.parent_id = parent_pc;
				}

				pMsg[0] = to_insn_cnt;
				pMsg[1] = to_cycle_cnt;
				plen1 = PackData(packbuf1, pMsg, 2);

				pMsg[0] = self_insn_cnt;
				pMsg[1] = self_cycle_cnt;

				pMsg[2] = child_insn_cnt;
				pMsg[3] = child_cycle_cnt;

				pMsg[4] = call_cnt;
				pMsg[5] = br_taken_cnt;

				pMsg[6] = br_mis_cnt;
				pMsg[7] = BTB_br_cnt;

				pMsg[8] = icache_replace_cnt;
				pMsg[9] = icache_miss_cnt;

				pMsg[10] = icache_access_cnt;
				pMsg[11] = dcache_replace_cnt;

				pMsg[12] = dcache_miss_cnt;
				pMsg[13] = dcache_access_cnt;

				plen2 = PackData(packbuf2, pMsg, 14);


				if(exec_stack != NULL){
	//				update_data_length((temp_file_pos - exec_stack->pos - sizeof(Header_t)),
	//									(temp_file_pos + (sizeof(rec) + plen1) - exec_stack->pos));
					update_data_length((temp_file_pos - exec_stack->pos - sizeof(Header_t)),
										(temp_file_pos + (sizeof(Header_t) + plen1)) );
				}

				wp = mempcpy(wp, &H, sizeof(Header_t));
				wp = mempcpy(wp, packbuf1, plen1);
				wp = mempcpy(wp, packbuf2, plen2);
				break;
		}
		if (!fwrite(wbuf, wp - wbuf, 1, temp_fd))
		{
			gErrorCode = errno;
			return -1;
		}
		else
			*len = wp - wbuf;

		return 0;


} // write_tl8_temp


// ----------------------------------------------------------------------------
// write_tl7_temp
//
// This function writes temporary record for level 7 timeline analysis.
// return code:
// 0 - success
// 1 - write error
// ----------------------------------------------------------------------------
static int
write_tl7_temp(bfd_vma child_pc,
		bfd_vma parent_pc,
		unsigned char tag,
		short func_level,
		unsigned int func_length,
		unsigned long long to_insn_cnt,
		unsigned long long to_cycle_cnt,
		unsigned long long self_insn_cnt,
		unsigned long long self_cycle_cnt,
		unsigned long long child_insn_cnt,
		unsigned long long child_cycle_cnt,
		unsigned long long call_cnt,
		unsigned long long icache_replace_cnt,
		unsigned long long icache_miss_cnt,
		unsigned long long icache_access_cnt,
		unsigned long long dcache_replace_cnt,
		unsigned long long dcache_miss_cnt,
		unsigned long long dcache_access_cnt,
		unsigned int *len)
{
		int result = 0;
		Header_t H;
		unsigned char packbuf1[64];
		unsigned char packbuf2[256];
		unsigned long long pMsg[32];
		char wbuf[512];
		char * wp;
		int plen1 = 0, plen2 = 0;

		wp = wbuf;

		H.tag = tag;
		H.level = func_level;
		H.func_id = child_pc;

		switch (tag){
			case PROFTYPE_FC7:
			case PROFTYPE_ON7:
				H.parent_id = (exec_stack == NULL)? VEP_INIT_PC: exec_stack->sym->addr;

				pMsg[0] = to_insn_cnt;
				pMsg[1] = to_cycle_cnt;
				plen1 = PackData(packbuf1, pMsg, 2);

				wp = mempcpy(wp, &H, sizeof(Header_t));
				wp = mempcpy(wp, &func_length, sizeof(long));
				wp = mempcpy(wp, packbuf1, plen1);
				break;

			case PROFTYPE_FR7:
			case PROFTYPE_DUMMY_FR7:
			case PROFTYPE_OFF:
				if(exec_stack != NULL){
					H.parent_id = (exec_stack->next == NULL)? VEP_INIT_PC : exec_stack->next->sym->addr;
				}
				else {
					H.parent_id = parent_pc;
				}

				pMsg[0] = to_insn_cnt;
				pMsg[1] = to_cycle_cnt;
				plen1 = PackData(packbuf1, pMsg, 2);

				pMsg[0] = self_insn_cnt;
				pMsg[1] = self_cycle_cnt;

				pMsg[2] = child_insn_cnt;
				pMsg[3] = child_cycle_cnt;

				pMsg[4] = call_cnt;
				pMsg[5] = -1;

				pMsg[6] = icache_replace_cnt;
				pMsg[7] = icache_miss_cnt;

				pMsg[8] = icache_access_cnt;
				pMsg[9] = dcache_replace_cnt;

				pMsg[10] = dcache_miss_cnt;
				pMsg[11] = dcache_access_cnt;

				plen2 = PackData(packbuf2, pMsg, 12);


				if(exec_stack != NULL){
	//				update_data_length((temp_file_pos - exec_stack->pos - sizeof(Header_t)),
	//									(temp_file_pos + (sizeof(rec) + plen1) - exec_stack->pos));
					update_data_length((temp_file_pos - exec_stack->pos - sizeof(Header_t)),
										(temp_file_pos + (sizeof(Header_t) + plen1)) );
				}

				wp = mempcpy(wp, &H, sizeof(Header_t));
				wp = mempcpy(wp, packbuf1, plen1);
				wp = mempcpy(wp, packbuf2, plen2);
				break;
		}
		if (!fwrite(wbuf, wp - wbuf, 1, temp_fd))
		{
			gErrorCode = errno;
			return -1;
		}
		else
			*len = wp - wbuf;

		return 0;

} // write_tl7_temp



// ----------------------------------------------------------------------------
// write_tl6_temp
//
// This function writes temporary record for level 6 timeline analysis.
// return code:
// 0 - success
// 1 - write error
// ----------------------------------------------------------------------------
static int
write_tl6_temp(bfd_vma child_pc,
		bfd_vma parent_pc,
		unsigned char tag,
		short func_level,
		unsigned int func_length,
		unsigned long long to_insn_cnt,
		unsigned long long to_cycle_cnt,
		unsigned long long self_insn_cnt,
		unsigned long long self_cycle_cnt,
		unsigned long long child_insn_cnt,
		unsigned long long child_cycle_cnt,
		unsigned long long call_cnt,
		unsigned long long br_taken_cnt,
		unsigned long long br_mis_cnt,
		unsigned long long BTB_br_cnt,
		unsigned int *len)
{
		int result = 0;
		Header_t H;
		unsigned char packbuf1[64];
		unsigned char packbuf2[256];
		unsigned long long pMsg[32];
		char wbuf[512];
		char * wp;
		int plen1 = 0, plen2 = 0;

		wp = wbuf;

		H.tag = tag;
		H.level = func_level;
		H.func_id = child_pc;

		switch (tag){
			case PROFTYPE_FC6:
			case PROFTYPE_ON6:
				H.parent_id = (exec_stack == NULL)? VEP_INIT_PC: exec_stack->sym->addr;

				pMsg[0] = to_insn_cnt;
				pMsg[1] = to_cycle_cnt;
				plen1 = PackData(packbuf1, pMsg, 2);

				wp = mempcpy(wp, &H, sizeof(Header_t));
				wp = mempcpy(wp, &func_length, sizeof(long));
				wp = mempcpy(wp, packbuf1, plen1);
				break;

			case PROFTYPE_FR6:
			case PROFTYPE_DUMMY_FR6:
			case PROFTYPE_OFF:
				if(exec_stack != NULL){
					H.parent_id = (exec_stack->next == NULL)? VEP_INIT_PC : exec_stack->next->sym->addr;
				}
				else {
					H.parent_id = parent_pc;
				}

				pMsg[0] = to_insn_cnt;
				pMsg[1] = to_cycle_cnt;
				plen1 = PackData(packbuf1, pMsg, 2);

				pMsg[0] = self_insn_cnt;
				pMsg[1] = self_cycle_cnt;

				pMsg[2] = child_insn_cnt;
				pMsg[3] = child_cycle_cnt;

				pMsg[4] = call_cnt;
				pMsg[5] = br_taken_cnt;

				pMsg[6] = br_mis_cnt;
				pMsg[7] = BTB_br_cnt;
				plen2 = PackData(packbuf2, pMsg, 8);

				if(exec_stack != NULL){
	//				update_data_length((temp_file_pos - exec_stack->pos - sizeof(Header_t)),
	//									(temp_file_pos + (sizeof(rec) + plen1) - exec_stack->pos));
					update_data_length((temp_file_pos - exec_stack->pos - sizeof(Header_t)),
										(temp_file_pos + (sizeof(Header_t) + plen1)) );
				}

				wp = mempcpy(wp, &H, sizeof(Header_t));
				wp = mempcpy(wp, packbuf1, plen1);
				wp = mempcpy(wp, packbuf2, plen2);
				break;
		}
		if (!fwrite(wbuf, wp - wbuf, 1, temp_fd))
		{
			gErrorCode = errno;
			return -1;
		}
		else
			*len = wp - wbuf;

		return 0;

} // write_tl6_temp


// ----------------------------------------------------------------------------
// write_tl3_temp
//
// This function writes temporary record for level 1 timeline analysis.
// return code:
// 0 - success
// 1 - write error
// ----------------------------------------------------------------------------
static int
write_tl3_temp(bfd_vma child_pc,
		bfd_vma parent_pc,
		unsigned char tag,
		short func_level,
		unsigned int func_length,
		unsigned long long to_insn_cnt,
		unsigned long long to_cycle_cnt,
		unsigned long long self_insn_cnt,
		unsigned long long self_cycle_cnt,
		unsigned long long child_insn_cnt,
		unsigned long long child_cycle_cnt,
		unsigned long long call_cnt,
		unsigned char branch_data,
		unsigned short ifetch_data,
		unsigned char dfetch_data1,
		unsigned int dfetch_data2,
		unsigned int dfetch_data3,
		unsigned int *len)
{
	unsigned char outrec[100] = {0};
	int buf_index = 0, self_cnt_index = 0, child_cnt_index = 0, call_cnt_index = 0;
	unsigned int cnt = 0;
	int result = 0;

	// tag
	outrec[0] = tag;
	// func_level
	outrec[1] = (unsigned char)(func_level & 0x00FF);
	outrec[2] = (unsigned char)((func_level & 0xFF00) >> 8);
	// func_id
	outrec[3] = (unsigned char)(child_pc & 0x000000FF);
	outrec[4] = (unsigned char)((child_pc & 0x0000FF00) >> 8);
	outrec[5] = (unsigned char)((child_pc & 0x00FF0000) >> 16);
	outrec[6] = (unsigned char)((child_pc & 0xFF000000) >> 24);

	if((tag == PROFTYPE_FC3) || (tag == PROFTYPE_ON3)){
		// parent_id
		if(exec_stack == NULL){
			outrec[7] = (unsigned char)(VEP_INIT_PC & 0x000000FF);
			outrec[8] = (unsigned char)((VEP_INIT_PC & 0x0000FF00) >> 8);
			outrec[9] = (unsigned char)((VEP_INIT_PC & 0x00FF0000) >> 16);
			outrec[10] = (unsigned char)((VEP_INIT_PC & 0xFF000000) >> 24);
		}else{
			outrec[7] = (unsigned char)(exec_stack->sym->addr & 0x000000FF);
			outrec[8] = (unsigned char)((exec_stack->sym->addr & 0x0000FF00) >> 8);
			outrec[9] = (unsigned char)((exec_stack->sym->addr & 0x00FF0000) >> 16);
			outrec[10] = (unsigned char)((exec_stack->sym->addr & 0xFF000000) >> 24);
		}
		// length
		outrec[11] = (unsigned char)(func_length & 0x000000FF);
		outrec[12] = (unsigned char)((func_length & 0x0000FF00) >> 8);
		outrec[13] = (unsigned char)((func_length & 0x00FF0000) >> 16);
		outrec[14] = (unsigned char)((func_length & 0xFF000000) >> 24);
		buf_index = 16;

		buf_index = PlaceData2Buf(to_insn_cnt, 		outrec, 15, buf_index, HIGH_NIBBLE);	// to_insn_cnt
		buf_index = PlaceData2Buf(to_cycle_cnt, 		outrec, 15, buf_index, LOW_NIBBLE);	// to_cycle_cnt

		// PC
		for(cnt = 0; cnt < 4; cnt++)
			outrec[buf_index++] = (unsigned char)((((unsigned int)child_pc) & ((unsigned int)0xFF << (cnt * 8))) >> (cnt * 8));
		// branch_data
		outrec[buf_index++] = branch_data;
		// ifetch_data
		for(cnt = 0; cnt < 2; cnt++)
			outrec[buf_index++] = (unsigned char)((ifetch_data & ((unsigned short)0xFF << (cnt * 8))) >> (cnt * 8));
	}else if((tag == PROFTYPE_FR3) || (tag == PROFTYPE_OFF) || (tag == PROFTYPE_DUMMY_FR3)){
		// parent_id
		if(exec_stack != NULL){
			if(exec_stack->next == NULL){
				outrec[7] = (unsigned char)(VEP_INIT_PC & 0x000000FF);
				outrec[8] = (unsigned char)((VEP_INIT_PC & 0x0000FF00) >> 8);
				outrec[9] = (unsigned char)((VEP_INIT_PC & 0x00FF0000) >> 16);
				outrec[10] = (unsigned char)((VEP_INIT_PC & 0xFF000000) >> 24);
			}else{
				outrec[7] = (unsigned char)(exec_stack->next->sym->addr & 0x000000FF);
				outrec[8] = (unsigned char)((exec_stack->next->sym->addr & 0x0000FF00) >> 8);
				outrec[9] = (unsigned char)((exec_stack->next->sym->addr & 0x00FF0000) >> 16);
				outrec[10] = (unsigned char)((exec_stack->next->sym->addr & 0xFF000000) >> 24);
			}
		}else{
				outrec[7] = (unsigned char)(parent_pc & 0x000000FF);
				outrec[8] = (unsigned char)((parent_pc & 0x0000FF00) >> 8);
				outrec[9] = (unsigned char)((parent_pc & 0x00FF0000) >> 16);
				outrec[10] = (unsigned char)((parent_pc & 0xFF000000) >> 24);
		}
		buf_index = 12;
		buf_index = PlaceData2Buf(to_insn_cnt, 		outrec, 11, buf_index, HIGH_NIBBLE);	// to_insn_cnt
		buf_index = PlaceData2Buf(to_cycle_cnt, 		outrec, 11, buf_index, LOW_NIBBLE);	// to_cycle_cnt
		// PC
		for(cnt = 0; cnt < 4; cnt++)
			outrec[buf_index++] = (unsigned char)((((unsigned int)child_pc) & ((unsigned int)0xFF << (cnt * 8))) >> (cnt * 8));
		// branch_data
		outrec[buf_index++] = branch_data;
		// ifetch_data
		for(cnt = 0; cnt < 2; cnt++)
			outrec[buf_index++] = (unsigned char)((ifetch_data & ((unsigned short)0xFF << (cnt * 8))) >> (cnt * 8));

		// push position of summary data into length field
		if(exec_stack != NULL){
			update_data_length((temp_file_pos - exec_stack->pos - 11), (temp_file_pos + buf_index - exec_stack->pos));
		}

		self_cnt_index = buf_index++;
		child_cnt_index = buf_index++;
		call_cnt_index = buf_index++;
		// self_insn_cnt
		buf_index = PlaceData2Buf(self_insn_cnt, 		outrec, self_cnt_index, buf_index, HIGH_NIBBLE);
		// self_cycle_cnt
		buf_index = PlaceData2Buf(self_cycle_cnt, 		outrec, self_cnt_index, buf_index, LOW_NIBBLE);
		// child_insn_cnt
		buf_index = PlaceData2Buf(child_insn_cnt, 		outrec, child_cnt_index, buf_index, HIGH_NIBBLE);
		// child_cycle_cnt
		buf_index = PlaceData2Buf(child_cycle_cnt,		outrec, child_cnt_index, buf_index, LOW_NIBBLE);
		// child_call
		buf_index = PlaceData2Buf(call_cnt,		outrec, call_cnt_index, buf_index, HIGH_NIBBLE);
	}else if(tag == PROFTYPE_BR3){
		// parent_id
		outrec[7] = (unsigned char)(parent_pc & 0x000000FF);
		outrec[8] = (unsigned char)((parent_pc & 0x0000FF00) >> 8);
		outrec[9] = (unsigned char)((parent_pc & 0x00FF0000) >> 16);
		outrec[10] = (unsigned char)((parent_pc & 0xFF000000) >> 24);
		buf_index = 12;
		buf_index = PlaceData2Buf(to_insn_cnt, 		outrec, 11, buf_index, HIGH_NIBBLE);	// to_insn_cnt
		buf_index = PlaceData2Buf(to_cycle_cnt, 		outrec, 11, buf_index, LOW_NIBBLE);	// to_cycle_cnt

		// PC
		for(cnt = 0; cnt < 4; cnt++)
			outrec[buf_index++] = (unsigned char)((((unsigned int)child_pc) & ((unsigned int)0xFF << (cnt * 8))) >> (cnt * 8));
		// branch_data
		outrec[buf_index++] = branch_data;
		// ifetch_data
		for(cnt = 0; cnt < 2; cnt++)
			outrec[buf_index++] = (unsigned char)((ifetch_data & ((unsigned short)0xFF << (cnt * 8))) >> (cnt * 8));
	}else if(tag == PROFTYPE_MA3){
		// parent_id
		outrec[7] = (unsigned char)(parent_pc & 0x000000FF);
		outrec[8] = (unsigned char)((parent_pc & 0x0000FF00) >> 8);
		outrec[9] = (unsigned char)((parent_pc & 0x00FF0000) >> 16);
		outrec[10] = (unsigned char)((parent_pc & 0xFF000000) >> 24);
		buf_index = 12;

		buf_index = PlaceData2Buf(to_insn_cnt, 		outrec, 11, buf_index, HIGH_NIBBLE);	// to_insn_cnt
		buf_index = PlaceData2Buf(to_cycle_cnt, 		outrec, 11, buf_index, LOW_NIBBLE);	// to_cycle_cnt

		// data fetch data 1
		outrec[buf_index++] = dfetch_data1;
		// data fetch data 2
		for(cnt = 0; cnt < 4; cnt++)
			outrec[buf_index++] = (unsigned char)((dfetch_data2 & ((unsigned int)0xFF << (cnt * 8))) >> (cnt * 8));
		// data fetch data 3
		for(cnt = 0; cnt < 4; cnt++)
			outrec[buf_index++] = (unsigned char)((dfetch_data3 & ((unsigned int)0xFF << (cnt * 8))) >> (cnt * 8));
		// PC
		for(cnt = 0; cnt < 4; cnt++)
			outrec[buf_index++] = (unsigned char)((((unsigned int)child_pc) & ((unsigned int)0xFF << (cnt * 8))) >> (cnt * 8));
	}else if(tag == PROFTYPE_OT3){
		// parent_id
		outrec[7] = (unsigned char)(parent_pc & 0x000000FF);
		outrec[8] = (unsigned char)((parent_pc & 0x0000FF00) >> 8);
		outrec[9] = (unsigned char)((parent_pc & 0x00FF0000) >> 16);
		outrec[10] = (unsigned char)((parent_pc & 0xFF000000) >> 24);
		buf_index = 12;
		// to_insn_cnt
		buf_index = PlaceData2Buf(to_insn_cnt, 		outrec, 11, buf_index, HIGH_NIBBLE);	// to_insn_cnt
		buf_index = PlaceData2Buf(to_cycle_cnt, 		outrec, 11, buf_index, LOW_NIBBLE);	// to_cycle_cnt
		// PC
		for(cnt = 0; cnt < 4; cnt++)
			outrec[buf_index++] = (unsigned char)((((unsigned int)child_pc) & ((unsigned int)0xFF << (cnt * 8))) >> (cnt * 8));
		// ifetch_data
		for(cnt = 0; cnt < 2; cnt++)
			outrec[buf_index++] = (unsigned char)((ifetch_data & ((unsigned short)0xFF << (cnt * 8))) >> (cnt * 8));
	}

	*len = buf_index;
	if((tag == PROFTYPE_ON3) || (tag == PROFTYPE_FC3) || (tag == PROFTYPE_FR3) || (tag == PROFTYPE_OFF) || (tag == PROFTYPE_DUMMY_FR3) || (tag == PROFTYPE_BR3) || (tag == PROFTYPE_MA3) || (tag == PROFTYPE_OT3)){

		result = fwrite(outrec, buf_index, 1, temp_fd);

		if(result == 1)
			return 0;
		else
			return 1;
	}else{
		return 0;
	}
} // write_tl3_temp


// ----------------------------------------------------------------------------
// write_tl2_temp
//
// This function writes temporary record for level 1 timeline analysis.
// return code:
// 0 - success
// 1 - write error
// ----------------------------------------------------------------------------


static int
write_tl2_temp(bfd_vma child_pc,
		bfd_vma parent_pc,
		unsigned char tag,
		short func_level,
		unsigned int func_length,
		unsigned long long to_insn_cnt,
		unsigned long long to_cycle_cnt,
		unsigned long long self_insn_cnt,
		unsigned long long self_cycle_cnt,
		unsigned long long child_insn_cnt,
		unsigned long long child_cycle_cnt,
		unsigned long long call_cnt,
		unsigned char branch_data,
		unsigned long long br_taken_cnt,
		unsigned long long br_mis_cnt,
		unsigned long long BTB_br_cnt,
		unsigned long long bb_cnt,
		unsigned int *len)
{
	int result = 0;
	Header_t H;
	unsigned char packbuf1[64];
	unsigned char packbuf2[256];
	unsigned long long pMsg[32];
	char wbuf[512];
	char * wp;
	int plen1 = 0, plen2 = 0;

	wp = wbuf;

	H.tag = tag;
	H.level = func_level;
	H.func_id = child_pc;

	switch (tag){
		case PROFTYPE_FC2:
		case PROFTYPE_ON2:
			H.parent_id = (exec_stack == NULL)? VEP_INIT_PC: exec_stack->sym->addr;

			pMsg[0] = to_insn_cnt;
			pMsg[1] = to_cycle_cnt;
			plen1 = PackData(packbuf1, pMsg, 2);

			wp = mempcpy(wp, &H, sizeof(Header_t));
			wp = mempcpy(wp, &func_length, sizeof(long));
			wp = mempcpy(wp, packbuf1, plen1);
			wp = mempcpy(wp, &branch_data, 1);
			break;

		case PROFTYPE_FR2:
		case PROFTYPE_DUMMY_FR2:
		case PROFTYPE_OFF:
			if(exec_stack != NULL){
				H.parent_id = (exec_stack->next == NULL)? VEP_INIT_PC : exec_stack->next->sym->addr;
			}
			else {
				H.parent_id = parent_pc;
			}

			pMsg[0] = to_insn_cnt;
			pMsg[1] = to_cycle_cnt;
			plen1 = PackData(packbuf1, pMsg, 2);

			pMsg[0] = self_insn_cnt;
			pMsg[1] = self_cycle_cnt;

			pMsg[2] = child_insn_cnt;
			pMsg[3] = child_cycle_cnt;

			pMsg[4] = call_cnt;
			pMsg[5] = br_taken_cnt;

			pMsg[6] = br_mis_cnt;
			pMsg[7] = BTB_br_cnt;

			pMsg[8] = bb_cnt;
			pMsg[9] = -1;
			plen2 = PackData(packbuf2, pMsg, 10);

			if(exec_stack != NULL){
//				update_data_length((temp_file_pos - exec_stack->pos - sizeof(Header_t)),
//									(temp_file_pos + (sizeof(rec) + plen1) - exec_stack->pos));
//				update_data_length((temp_file_pos - exec_stack->pos - sizeof(Header_t)),
//									(temp_file_pos + (sizeof(Header_t) + plen1 + 1)) );

				update_data_length((temp_file_pos - exec_stack->pos - sizeof(Header_t)),
								temp_file_pos);
			}

			wp = mempcpy(wp, &H, sizeof(Header_t));
			wp = mempcpy(wp, packbuf1, plen1);
			wp = mempcpy(wp, &branch_data, 1);
			wp = mempcpy(wp, packbuf2, plen2);
			break;

		case PROFTYPE_BR2:
			H.parent_id = parent_pc;
			pMsg[0] = to_insn_cnt;
			pMsg[1] = to_cycle_cnt;
			plen1 = PackData(packbuf1, pMsg, 2);

			wp = mempcpy(wp, &H, sizeof(Header_t));
			wp = mempcpy(wp, packbuf1, plen1);
			wp = mempcpy(wp, &branch_data, 1);
			wp = mempcpy(wp, &parent_pc, 4);	// reserve 4 bytes
			break;
	}
	if (!fwrite(wbuf, wp - wbuf, 1, temp_fd))
	{
		gErrorCode = errno;
		return -1;
	}
	else
		*len = wp - wbuf;

	return 0;

} // write_tl2_temp
// ----------------------------------------------------------------------------
// write_tl1_temp
//
// This function writes temporary record for level 1 timeline analysis.
// return code:
// 0 - success
// 1 - write error
// ----------------------------------------------------------------------------
static int
write_tl1_temp(bfd_vma child_pc,
		bfd_vma parent_pc,
		unsigned char tag,
		short func_level,
		unsigned int func_length,
		unsigned long long to_insn_cnt,
		unsigned long long to_cycle_cnt,
		unsigned long long self_insn_cnt,
		unsigned long long self_cycle_cnt,
		unsigned long long child_insn_cnt,
		unsigned long long child_cycle_cnt,
		unsigned long long call_cnt,
		unsigned int *len)
{
	int result = 0;
	Header_t H;
	unsigned char packbuf1[64];
	unsigned char packbuf2[256];
	unsigned long long pMsg[32];
	char wbuf[512];
	char * wp;
	int plen1 = 0, plen2 = 0;

	wp = wbuf;

	H.tag = tag;
	H.level = func_level;
	H.func_id = child_pc;

	switch (tag){
		case PROFTYPE_FC1:
		case PROFTYPE_ON1:
			H.parent_id = (exec_stack == NULL)? VEP_INIT_PC: exec_stack->sym->addr;

			pMsg[0] = to_insn_cnt;
			pMsg[1] = to_cycle_cnt;
			plen1 = PackData(packbuf1, pMsg, 2);

			wp = mempcpy(wp, &H, sizeof(Header_t));
			wp = mempcpy(wp, &func_length, sizeof(long));
			wp = mempcpy(wp, packbuf1, plen1);
			break;

		case PROFTYPE_FR1:
		case PROFTYPE_DUMMY_FR1:
		case PROFTYPE_OFF:
			if(exec_stack != NULL){
				H.parent_id = (exec_stack->next == NULL)? VEP_INIT_PC : exec_stack->next->sym->addr;
			}
			else {
				H.parent_id = parent_pc;
			}

			pMsg[0] = to_insn_cnt;
			pMsg[1] = to_cycle_cnt;
			plen1 = PackData(packbuf1, pMsg, 2);

			pMsg[0] = self_insn_cnt;
			pMsg[1] = self_cycle_cnt;
			pMsg[2] = child_insn_cnt;
			pMsg[3] = child_cycle_cnt;
			pMsg[4] = call_cnt;
			plen2 = PackData(packbuf2, pMsg, 5);

			if(exec_stack != NULL){
//				update_data_length((temp_file_pos - exec_stack->pos - sizeof(Header_t)),
//									(temp_file_pos + (sizeof(rec) + plen1) - exec_stack->pos));
				update_data_length((temp_file_pos - exec_stack->pos - sizeof(Header_t)),
									(temp_file_pos + (sizeof(Header_t) + plen1)) );
			}

			wp = mempcpy(wp, &H, sizeof(Header_t));
			wp = mempcpy(wp, packbuf1, plen1);
			wp = mempcpy(wp, packbuf2, plen2);
			break;
	}
	if (!fwrite(wbuf, wp - wbuf, 1, temp_fd))
	{
		gErrorCode = errno;
		return -1;
	}
	else
		*len = wp - wbuf;

	return 0;

} // write_tl1_temp


// ----------------------------------------------------------------------------
// push_callee2
//
// This function handles callee entry.
// return code:
// 0 - success
// 2 - write error
// 3 - memory error
// ----------------------------------------------------------------------------
static int
push_callee2(bfd_vma self,
             bfd_vma parent,
             unsigned int icnt,
             unsigned int ccnt,
	     unsigned long long pos/*,
		unsigned long long branch_taken_cnt,
		unsigned long long branch_misprediction_cnt,
		unsigned long long BTB_branch_cnt*/)
{
	Sym         *func;
    SymListNode *funcnode;

    // is this function monitored?
    func=sym_lookup(&symtab,self);
    if (func==NULL)
    {
	func=symht_lookup(self);
        if (func==NULL)
        {   // new unresolved symbol
            func=(Sym*)malloc(sizeof(Sym));
            if (func==NULL)
                return 3;
            sym_init(func);
            func->is_func=1;
            func->addr=self;

            // add into unresolved list
            if (symht_add(func)!=0)
            {   free(func);
                return 3;
            }
        }
    }

//    // generate a temporary timeline record
//    if (do_timeline&&write_tl_temp(self,icnt,ccnt)!=0)
//        return 2;

    // push callee into execution stack
    funcnode=(SymListNode*)malloc(sizeof(SymListNode));
    if (funcnode==NULL)
        return 3;
    funcnode->sym=func;
    funcnode->caller_addr=parent;
    funcnode->self_insn_cnt=0;
    funcnode->self_cycle_cnt=0;
    funcnode->child_insn_cnt=0;
    funcnode->child_cycle_cnt=0;
    funcnode->calls=0;
    funcnode->branch_taken_cnt = 0;
    funcnode->branch_misprediction_cnt = 0;
    funcnode->BTB_branch_cnt = 0;
    funcnode->icache_replace_cnt = 0;
    funcnode->icache_miss_cnt = 0;
    funcnode->icache_access_cnt = 0;
    funcnode->dcache_replace_cnt = 0;
    funcnode->dcache_miss_cnt = 0;
    funcnode->dcache_access_cnt = 0;
    funcnode->bb_cnt = 0;
    funcnode->pos=pos;
    funcnode->next=exec_stack;
    // add this arc
    tl_cg_tally(parent,self,1,0,0);
    exec_stack=funcnode;
#ifdef TRACE_STACK
    fprintf(stderr,"call\t");
    trace_stack(exec_stack,exec_stack->next);
#endif // TRACE_STACK

    return 0;
} // push_callee2

// ----------------------------------------------------------------------------
// push_callee
//
// This function handles callee entry.
// return code:
// 0 - success
// 2 - write error
// 3 - memory error
// ----------------------------------------------------------------------------
static int
push_callee(bfd_vma self,
        bfd_vma parent,
        unsigned int icnt,
        unsigned int ccnt,
	    unsigned long long pos,
	    unsigned long long br_taken_cnt,
	    unsigned long long br_mis_cnt,
	    unsigned long long BTB_br_cnt,
	    unsigned long long icache_replace_cnt,
	    unsigned long long icache_miss_cnt,
	    unsigned long long icache_access_cnt,
	    unsigned long long dcache_replace_cnt,
	    unsigned long long dcache_miss_cnt,
	    unsigned long long dcache_access_cnt)
{   // if profiling on not started from beginning
    // exec_stack could be NULL
    if (exec_stack!=NULL)
    {
	Sym * caller=exec_stack->sym;

        // update execution time to caller which must exist
        exec_stack->self_insn_cnt 	+= icnt;
        exec_stack->self_cycle_cnt 	+= ccnt;
		exec_stack->calls++;
		exec_stack->bb_cnt++;
		exec_stack->branch_taken_cnt 			+= br_taken_cnt;
		exec_stack->branch_misprediction_cnt	+= br_mis_cnt;
		exec_stack->BTB_branch_cnt 				+= BTB_br_cnt;

		exec_stack->icache_replace_cnt 	+= icache_replace_cnt;
		exec_stack->icache_miss_cnt 	+= icache_miss_cnt;
		exec_stack->icache_access_cnt 	+= icache_access_cnt;

		exec_stack->dcache_replace_cnt	+= dcache_replace_cnt;
		exec_stack->dcache_miss_cnt 	+= dcache_miss_cnt;
		exec_stack->dcache_access_cnt 	+= dcache_access_cnt;

        caller->hist.total_insn_cnt	 += icnt;
        caller->hist.total_cycle_cnt += ccnt;
    }

    return push_callee2(self,parent,icnt,ccnt,pos);
} // push_callee

// ----------------------------------------------------------------------------
// pop_callee2
//
// This function handles callee exit.
// return code:
// 0 - success
// 2 - write error
// ----------------------------------------------------------------------------
static int
pop_callee2(unsigned int icnt,
            unsigned int ccnt,
	    unsigned char tag,
	    bfd_vma tpc,
	    unsigned char branch_data,
	    unsigned short ifetch,
	    unsigned char dfetch1,
	    unsigned int dfetch2,
	    unsigned int dfetch3,
	    unsigned long long br_taken_cnt,
	    unsigned long long br_mis_cnt,
	    unsigned long long BTB_br_cnt,
	    unsigned long long icache_replace_cnt,
	    unsigned long long icache_miss_cnt,
	    unsigned long long icache_access_cnt,
	    unsigned long long dcache_replace_cnt,
	    unsigned long long dcache_miss_cnt,
	    unsigned long long dcache_access_cnt)
{   SymListNode *cexec=exec_stack;
    Sym         *callee=cexec->sym;
    int result = 0;
    unsigned int len = 0;

    // update execution time to callee
    cexec->self_insn_cnt+=icnt;
    cexec->self_cycle_cnt+=ccnt;
    cexec->bb_cnt++;
    cexec->branch_taken_cnt += br_taken_cnt;
    cexec->branch_misprediction_cnt += br_mis_cnt;
    cexec->BTB_branch_cnt += BTB_br_cnt;
    cexec->icache_replace_cnt += icache_replace_cnt;
    cexec->icache_miss_cnt += icache_miss_cnt;
    cexec->icache_access_cnt += icache_access_cnt;
    cexec->dcache_replace_cnt += dcache_replace_cnt;
    cexec->dcache_miss_cnt += dcache_miss_cnt;
    cexec->dcache_access_cnt += dcache_access_cnt;
    callee->hist.total_insn_cnt+=icnt;
    callee->hist.total_cycle_cnt+=ccnt;

	// generate a temporary timeline record
	if (do_timeline){
		if(tcghdr.timeline_level == 1){
			result = write_tl1_temp(callee->addr, tpc, tag, function_level, 0, current_insn_cnt, current_cycle_cnt, cexec->self_insn_cnt, cexec->self_cycle_cnt, cexec->child_insn_cnt, cexec->child_cycle_cnt, cexec->calls, &len);
		}else if(tcghdr.timeline_level == 2){
			result = write_tl2_temp(callee->addr, tpc, tag, function_level, 0, current_insn_cnt, current_cycle_cnt, cexec->self_insn_cnt, cexec->self_cycle_cnt, cexec->child_insn_cnt, cexec->child_cycle_cnt, cexec->calls, branch_data, cexec->branch_taken_cnt, cexec->branch_misprediction_cnt, cexec->BTB_branch_cnt, cexec->bb_cnt, &len);
		}else if(tcghdr.timeline_level == 3){
			result = write_tl3_temp(callee->addr, tpc, tag, function_level, 0, current_insn_cnt, current_cycle_cnt, cexec->self_insn_cnt, cexec->self_cycle_cnt, cexec->child_insn_cnt, cexec->child_cycle_cnt, cexec->calls, branch_data, ifetch, dfetch1, dfetch2, dfetch3, &len);
		}else if(tcghdr.timeline_level == 6){
			result = write_tl6_temp(callee->addr, tpc, tag, function_level, 0, current_insn_cnt, current_cycle_cnt, cexec->self_insn_cnt, cexec->self_cycle_cnt, cexec->child_insn_cnt, cexec->child_cycle_cnt, cexec->calls, cexec->branch_taken_cnt, cexec->branch_misprediction_cnt, cexec->BTB_branch_cnt, &len);
		}else if(tcghdr.timeline_level == 7){
			result = write_tl7_temp(callee->addr, tpc, tag, function_level, 0, current_insn_cnt, current_cycle_cnt, cexec->self_insn_cnt, cexec->self_cycle_cnt, cexec->child_insn_cnt, cexec->child_cycle_cnt, cexec->calls, cexec->icache_replace_cnt, cexec->icache_miss_cnt, cexec->icache_access_cnt, cexec->dcache_replace_cnt, cexec->dcache_miss_cnt, cexec->dcache_access_cnt, &len);
		}else if(tcghdr.timeline_level == 8){
			result = write_tl8_temp(callee->addr, tpc, tag, function_level, 0, current_insn_cnt, current_cycle_cnt, cexec->self_insn_cnt, cexec->self_cycle_cnt, cexec->child_insn_cnt, cexec->child_cycle_cnt, cexec->calls, cexec->branch_taken_cnt, cexec->branch_misprediction_cnt, cexec->BTB_branch_cnt, cexec->icache_replace_cnt, cexec->icache_miss_cnt, cexec->icache_access_cnt, cexec->dcache_replace_cnt, cexec->dcache_miss_cnt, cexec->dcache_access_cnt, &len);
		}else if(tcghdr.timeline_level == 9){
			result = write_tl9_temp(callee->addr, tpc, tag, function_level, 0, current_insn_cnt, current_cycle_cnt, cexec->self_insn_cnt, cexec->self_cycle_cnt, cexec->child_insn_cnt, cexec->child_cycle_cnt, cexec->calls, branch_data, cexec->bb_cnt, cexec->branch_taken_cnt, cexec->branch_misprediction_cnt, cexec->BTB_branch_cnt, icache_replace_cnt, icache_miss_cnt, icache_access_cnt, dcache_replace_cnt, dcache_miss_cnt, dcache_access_cnt, cexec->icache_replace_cnt, cexec->icache_miss_cnt, cexec->icache_access_cnt, cexec->dcache_replace_cnt, cexec->dcache_miss_cnt, cexec->dcache_access_cnt, &len);
		}
		if(result != 0)
			return 2;
		temp_file_pos += len;
	}

    // pop out callee
    exec_stack=cexec->next;

    // update execution time to caller
    tl_cg_tally(cexec->caller_addr,callee->addr,0,cexec->self_insn_cnt,cexec->self_cycle_cnt);
    if (exec_stack!=NULL)
    {   Sym *caller=exec_stack->sym;

        exec_stack->child_insn_cnt+=cexec->self_insn_cnt+cexec->child_insn_cnt;
        exec_stack->child_cycle_cnt+=cexec->self_cycle_cnt+cexec->child_cycle_cnt;
        caller->cg.child_insn_cnt+=cexec->self_insn_cnt;
        caller->cg.child_cycle_cnt+=cexec->self_cycle_cnt;
    }
#ifdef TRACE_STACK
    fprintf(stderr,"ret\t");
    trace_stack(cexec,exec_stack);
#endif // TRACE_STACK
    free(cexec);

    return 0;
} // pop_callee2

// ----------------------------------------------------------------------------
// pop_callee
//
// This function handles callee exit.
// return code:
// 0 - success
// 2 - write error
// ----------------------------------------------------------------------------
static int
pop_callee(bfd_vma self,
           bfd_vma parent,
           unsigned int icnt,
           unsigned int ccnt,
	   unsigned char tag,
	   unsigned char branch_data,
	   unsigned short ifetch,
	   unsigned char dfetch1,
	   unsigned int dfetch2,
	   unsigned int dfetch3,
	   unsigned long long br_taken_cnt,
	   unsigned long long br_mis_cnt,
	   unsigned long long BTB_br_cnt,
	   unsigned long long icache_replace_cnt,
	   unsigned long long icache_miss_cnt,
	   unsigned long long icache_access_cnt,
	   unsigned long long dcache_replace_cnt,
	   unsigned long long dcache_miss_cnt,
	   unsigned long long dcache_access_cnt)
{
	int result = 0;
	unsigned int len = 0;

	if (exec_stack!=NULL)
        return pop_callee2(icnt,ccnt,tag,parent,branch_data,ifetch,dfetch1,dfetch2,dfetch3,br_taken_cnt,br_mis_cnt,BTB_br_cnt, icache_replace_cnt, icache_miss_cnt, icache_access_cnt, dcache_replace_cnt, dcache_miss_cnt, dcache_access_cnt);
    else
    {   Sym *caller=sym_lookup(&symtab,parent);
        Sym *callee=sym_lookup(&symtab,self);

        // off balanced profiling on/off setting
        // update execution time to callee
        callee->hist.total_insn_cnt+=icnt;
        callee->hist.total_cycle_cnt+=ccnt;

//        // generate a temporary timeline record
//        if (do_timeline&&write_tl_temp(callee->addr,icnt,ccnt)!=0)
//            return 2;
	// generate a temporary timeline record
	if (do_timeline){
		if(tcghdr.timeline_level == 1){
			result = write_tl1_temp(callee->addr, caller->addr, tag, function_level, 0, current_insn_cnt, current_cycle_cnt, icnt, ccnt, 0, 0, 0, &len);
		}else if(tcghdr.timeline_level == 2){
			result = write_tl2_temp(callee->addr, caller->addr, tag, function_level, 0, current_insn_cnt, current_cycle_cnt, icnt, ccnt, 0, 0, 0, branch_data, 0, 0, 0, 0, &len);
		}else if(tcghdr.timeline_level == 3){
			result = write_tl3_temp(callee->addr, caller->addr, tag, function_level, 0, current_insn_cnt, current_cycle_cnt, icnt, ccnt, 0, 0, 0, branch_data, ifetch, dfetch1, dfetch2, dfetch3, &len);
		}else if(tcghdr.timeline_level == 6){
			result = write_tl6_temp(callee->addr, caller->addr, tag, function_level, 0, current_insn_cnt, current_cycle_cnt, icnt, ccnt, 0, 0, 0, 0, 0, 0, &len);
		}else if(tcghdr.timeline_level == 7){
			result = write_tl7_temp(callee->addr, caller->addr, tag, function_level, 0, current_insn_cnt, current_cycle_cnt, icnt, ccnt, 0, 0, 0, 0, 0, 0, 0, 0, 0, &len);
		}else if(tcghdr.timeline_level == 8){
			result = write_tl8_temp(callee->addr, caller->addr, tag, function_level, 0, current_insn_cnt, current_cycle_cnt, icnt, ccnt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, &len);
		}else if(tcghdr.timeline_level == 9){
			result = write_tl9_temp(callee->addr, caller->addr, tag, function_level, 0, current_insn_cnt, current_cycle_cnt, icnt, ccnt, 0, 0, 0, branch_data, 0, 0, 0, 0, icache_replace_cnt, icache_miss_cnt, icache_access_cnt, dcache_replace_cnt, dcache_miss_cnt, dcache_access_cnt, 0, 0, 0, 0, 0, 0, &len);
		}
		if(result != 0)
			return 2;
		temp_file_pos += len;
	}

        // add this arc
        tl_cg_tally(parent,callee->addr,1,0,0);
        // update execution time to caller
        tl_cg_tally(parent,callee->addr,0,icnt,ccnt);

        caller->cg.child_insn_cnt+=icnt;
        caller->cg.child_cycle_cnt+=ccnt;
#ifdef TRACE_STACK
        fprintf(stderr,"ret\t\t\t%s\t\t\t\t\t\t\tX%08X@%s\t\t\t\t\t",
                get_name(callee->name),parent,get_name(caller->name));
        fprintf(stderr,"%llu\t%llu\t%llu\t%llu\t",
                callee->hist.total_insn_cnt,callee->hist.total_cycle_cnt,
                callee->cg.child_insn_cnt,callee->cg.child_cycle_cnt);
        if (caller!=NULL)
            fprintf(stderr,"%llu\t%llu\t%llu\t%llu\n",
                    caller->hist.total_insn_cnt,caller->hist.total_cycle_cnt,
                    caller->cg.child_insn_cnt,caller->cg.child_cycle_cnt);
        else
            fprintf(stderr,"\t\t\t\n");
#endif // TRACE_STACK

        return 0;
    }
} // pop_callee

// ----------------------------------------------------------------------------
// read_prof_on
//
// This function reads the profiling on record from prof.out. Currently, we
// only support single on/off block in single prof.out file. We may support
// multiple blocks in future and also possibly different profiling levels in
// single prof.out file.
// return code:
// 0 - success
// 1 - read error
// 2 - write error
// 3 - memory error
// 4 - bad tag
// ----------------------------------------------------------------------------
static int
read_prof_on(FILE *fd,
             unsigned char proftag)
{   int result;
    unsigned char tag;


	// read the tag
	result=(fread(&tag,sizeof(unsigned char),1,fd)!=1);

    if (result==0)
    {   // must be expected tag
        if (tag!=proftag)
            return 4;
        else
        {   profmdata mydata;
            result=(fread(&mydata,PROFDLEN_ON-1,1,fd)!=1);
            if (result==0)
            {
			unsigned int len = 0;
				// generate a temporary timeline record
				if (do_timeline){
					int result = 0;

					CurrentPC = mydata.data.pc;
					current_insn_cnt += mydata.data.icnt;
					current_cycle_cnt += mydata.data.ccnt;
					function_level++;

					if(proftag == PROFTYPE_ON1){
						result = write_tl1_temp(mydata.data.pc, 0, tag, function_level, 0, current_insn_cnt, current_cycle_cnt, 0, 0, 0, 0, 0, &len);
//						fflush(temp_fd);
					}else if(proftag == PROFTYPE_ON2){
						result = write_tl2_temp(mydata.data.pc, 0, tag, function_level, 0, current_insn_cnt, current_cycle_cnt, 0, 0, 0, 0, 0, 0,0,0,0,0, &len);
//						fflush(temp_fd);
					}else if(proftag == PROFTYPE_ON3){
						result = write_tl3_temp(mydata.data.pc, 0, tag, function_level, 0, current_insn_cnt, current_cycle_cnt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, &len);
//						fflush(temp_fd);
					}else if(proftag == PROFTYPE_ON6){
						result = write_tl6_temp(mydata.data.pc, 0, tag, function_level, 0, current_insn_cnt, current_cycle_cnt, 0, 0, 0, 0, 0, 0, 0, 0, &len);
//						fflush(temp_fd);
					}else if(proftag == PROFTYPE_ON7){
						result = write_tl7_temp(mydata.data.pc, 0, tag, function_level, 0, current_insn_cnt, current_cycle_cnt, 0, 0, 0, 0, 0,0,0,0,0,0,0, &len);
//						fflush(temp_fd);
					}else if(proftag == PROFTYPE_ON8){
						result = write_tl8_temp(mydata.data.pc, 0, tag, function_level, 0, current_insn_cnt, current_cycle_cnt, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,0,0, &len);
//						fflush(temp_fd);
					}else if(proftag == PROFTYPE_ON9){
						result = write_tl9_temp(mydata.data.pc, 0, tag, function_level, 0, current_insn_cnt, current_cycle_cnt, 0, 0, 0, 0, 0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, &len);
//						fflush(temp_fd);
					}
					if(result != 0)
						return 2;
				}

                result=push_callee2(mydata.data.pc,VEP_INIT_PC,mydata.data.icnt,mydata.data.ccnt,temp_file_pos);
                if (result==0)
                    current_unmapped=((mydata.mode&IT_MASK)==0);

				temp_file_pos += len;
            }
        }
    }

    return result;
} // read_prof_on

// ----------------------------------------------------------------------------
// read_profm _data
//
// This function reads the pc address, instruction count, cycle count, and
// process mode/endian/IT/DT flag data for all profiling levels from prof.out.
// return code:
// 0 - success
// 1 - read error
// ----------------------------------------------------------------------------
static int
read_profm_data(FILE *fd,
                profmdata *mydata)
{   int result;

    result=fread(&mydata->data,sizeof(prof1data),1,fd);
    if (result)
        result=fread(&mydata->mode,sizeof(unsigned char),1,fd);

    return !result;
} // read_profm_data

// ----------------------------------------------------------------------------
// read_prof1_data
//
// This function reads the function coverage profiling records from prof.out.
// return code:
// 0 - success
// 1 - read error
// 2 - write error
// 3 - memory error
// 4 - bad tag
// ----------------------------------------------------------------------------
static int
read_prof1_data(FILE *fd)
{
	int result;
    unsigned char tag;
    prof1data2 mydata;
    profmdata mydata2;


    while (fread(&tag,sizeof(unsigned char),1,fd)==1)
    {
		if (temp_file_pos > TIMELINE_LIMIT)
		{
			// We have a timeline too big, change the tag to PROF_OFF to finish
			tag = PROFTYPE_OFF;
		}

	switch (tag) {
		case PROFTYPE_FC1:
				tcghdr.to_fn_call_tag++;
				// we have a call here
				result=(fread(&mydata,sizeof(prof1data2),1,fd)!=1);

				if (result==0){
					unsigned int len = 0;
					function_level++;
					if(function_level > max_function_level)
						max_function_level = function_level;

					current_insn_cnt += mydata.data.icnt;
					current_cycle_cnt += mydata.data.ccnt;
					// generate a temporary timeline record
					if (do_timeline){
						result = write_tl1_temp(mydata.tpc, mydata.data.pc, tag, function_level, 0, current_insn_cnt, current_cycle_cnt, 0, 0, 0, 0, 0, &len);
						if(result != 0)
							return 2;
					}
					result=push_callee(mydata.tpc,mydata.data.pc,mydata.data.icnt,mydata.data.ccnt, temp_file_pos, 0, 0, 0, 0,0,0,0,0,0);
					temp_file_pos += len;
				}
				break;

			case PROFTYPE_FR1:
				tcghdr.to_fn_rt_tag++;
	            // we have a return here
	            result=(fread(&mydata,sizeof(prof1data2),1,fd)!=1);

				if (result==0){
					function_level--;
					if(min_function_level > function_level)
						min_function_level = function_level;
					current_insn_cnt += mydata.data.icnt;
					current_cycle_cnt += mydata.data.ccnt;
					result=pop_callee(mydata.data.pc,mydata.tpc,mydata.data.icnt,mydata.data.ccnt,tag,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
				}
				break;
			case PROFTYPE_MOD:
				// mode changed
				result=read_profm_data(fd,&mydata2);

				if (result==0)
				{   // update current time point
					current_insn_cnt+=mydata2.data.icnt;
					current_cycle_cnt+=mydata2.data.ccnt;
					if (exec_stack != NULL) {
						exec_stack->self_insn_cnt+=mydata2.data.icnt;
						exec_stack->self_cycle_cnt+=mydata2.data.ccnt;
						exec_stack->sym->hist.total_insn_cnt+=mydata2.data.icnt;
						exec_stack->sym->hist.total_cycle_cnt+=mydata2.data.ccnt;
					}

					// need to address all possible cause
					// do we need target pc address? for instructions like
					// jral.xton we do not need it again
					current_unmapped=((mydata2.mode&IT_MASK)==0);
				}
				break;
			case PROFTYPE_OFF:
				// done reading profiling data
				result=(fread(&mydata.data,sizeof(prof1data),1,fd)!=1);
				fread(&mydata.tpc, 1, 1, fd);

				if (result==0)
				{
					unsigned int delta_icnt = 0, delta_ccnt = 0;
					int result = 0;

					function_level--;
					if(min_function_level > function_level)
						min_function_level = function_level;
					current_insn_cnt += mydata.data.icnt;
					current_cycle_cnt += mydata.data.ccnt;
					delta_icnt = mydata.data.icnt;
					delta_ccnt = mydata.data.ccnt;
					if (exec_stack != NULL)
					{
						while(exec_stack->next != NULL){
							tag = PROFTYPE_DUMMY_FR1;
							tcghdr.to_du_rt_tag++;
							result = pop_callee(exec_stack->sym->addr, exec_stack->next->sym->addr, delta_icnt, delta_ccnt, tag, 0,0,0,0,0,0,0,0,0,0,0,0,0,0);
							function_level--;
							if(min_function_level > function_level)
							min_function_level = function_level;
							delta_icnt = 0;
							delta_ccnt = 0;
						}
						tag = PROFTYPE_OFF;
						// profiling turned off, so clean up the execution stack
						result=pop_callee2(delta_icnt, delta_ccnt, tag, 0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,0,0);
					}
					else
						result=pop_callee(mydata.data.pc,mydata.tpc,mydata.data.icnt,mydata.data.ccnt,tag,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
				}
				return result;
			default:
				return 4;
        }
        if (result!=0)
            return result;
	}
    // read error - only successful exit is from PROFTYPE_OFF record
    return 1;
} // read_prof1_data

// ----------------------------------------------------------------------------
// tl1_read_rec
//
// This function reads the timeline based profiling data for function coverage.
// The expected sequences are 0xc1, {0x11,0x12,0xcf}*, 0xc0.
// return code: (execution terminated if any error encountered.)
// 0x00 - end-of-file
// tag - next tag
// ----------------------------------------------------------------------------
static unsigned char
tl1_read_rec(FILE *fd,
             const char *filename)
{   unsigned char tag;
    int result;

    result=read_prof_on(fd,PROFTYPE_ON1);
    if (result==0)
        result=read_prof1_data(fd);
    if (result!=0)
        return prof_errmsg(result,filename); // no return

	if (temp_file_pos > TIMELINE_LIMIT)
		return 0x00;

    if (fread(&tag,sizeof(unsigned char),1,fd)!=1)
        tag=0x00; // can only be end-of-file

    return tag;
} // tl1_read_rec

// ----------------------------------------------------------------------------
// read_prof2_data2
//
// This function reads the pc address, instruction count, cycle count, target
// pc, and branch prediction data for profiling level 2 from prof.out.
// return code:
// 0 - success
// 1 - read error
// ----------------------------------------------------------------------------
static int
read_prof2_data2(FILE *fd,
                 prof2data *mydata)
{   int result;

    result=(fread(&mydata->data,sizeof(prof1data2),1,fd)!=1);
    if (result==0)
        result=(fread(&mydata->br,sizeof(unsigned char),1,fd)!=1);

    return result;
} // read_prof2_data2


// ---------------------------------------------------------------------------
// add_branch_count_to_sym
//
// This function adds count of basic block to sym.
// return code:
// 0 - success
// 1 - error
// ----------------------------------------------------------------------------
static int
add_branch_count_to_sym(bfd_vma parent, int count)
{
	Sym *sym;

	if (line_granularity){
		sym = sym_lookup (&symtab, parent);
		if (sym){
			int i;

			for (i = 0; i < NBBS; i++)
			{
				if (! sym->bb_addr[i] || sym->bb_addr[i] == parent)
				{
					sym->bb_addr[i] = parent;
					sym->bb_calls[i] += count;
					break;
				}
			}
		}
	}
}

// ----------------------------------------------------------------------------
// read_prof2_data
//
// This function reads the branch coverage profiling records from prof.out.
// return code:
// 0 - success
// 1 - read error
// 2 - write error
// 3 - memory error
// 4 - bad tag
// ----------------------------------------------------------------------------
static int
read_prof2_data(FILE *fd)
{   int result;
    unsigned char tag;
    prof2data mydata;
    profmdata mydata2;

	while (fread(&tag,sizeof(unsigned char),1,fd)==1)
	{
		if (temp_file_pos > TIMELINE_LIMIT)
		{
			// We have a timeline too big, change the tag to PROF_OFF to finish
			tag = PROFTYPE_OFF;
		}

		switch (tag)
		{
			case PROFTYPE_FC2:
				tcghdr.to_fn_call_tag++;
				result=read_prof2_data2(fd,&mydata);
				if (result==0){
					unsigned int len = 0;

					function_level++;
					if(function_level > max_function_level)
						max_function_level = function_level;
					current_insn_cnt += mydata.data.data.icnt;
					current_cycle_cnt += mydata.data.data.ccnt;
					// generate a temporary timeline record
					if (do_timeline){
						result = write_tl2_temp(mydata.data.tpc, mydata.data.data.pc, tag, function_level, 0, current_insn_cnt, current_cycle_cnt, 0, 0, 0, 0, 0, mydata.br,0,0,0,0, &len);
						if(result != 0)
							return 2;
					}
					result=push_callee(mydata.data.tpc,mydata.data.data.pc,mydata.data.data.icnt,mydata.data.data.ccnt, temp_file_pos, ((mydata.br & 0x02) >> 1), (mydata.br & 0x01), ((mydata.br & 0xFC) >> 2),0,0,0,0,0,0);
					temp_file_pos += len;
				}

				// branch prediction
				total_BTB_branch_count = (unsigned long long)((mydata.br & 0xFC)>>2);
				total_branch_taken_count = (unsigned long long)((mydata.br & 0x02)>>1);
				total_branch_mispred_count = (unsigned long long)(mydata.br & 0x01);
				tcghdr.to_bb_cnt++;
				break;
			case PROFTYPE_FR2:
				tcghdr.to_fn_rt_tag++;
				result=read_prof2_data2(fd,&mydata);
				if (result==0){
					function_level--;
					if(min_function_level > function_level)
						min_function_level = function_level;
					current_insn_cnt += mydata.data.data.icnt;
					current_cycle_cnt += mydata.data.data.ccnt;
					result=pop_callee(mydata.data.data.pc,mydata.data.tpc,mydata.data.data.icnt,mydata.data.data.ccnt,tag,mydata.br,0,0,0,0,0,(mydata.br & 0x01), ((mydata.br & 0xFC) >> 2),0,0,0,0,0,0);
				}
				// branch prediction
				total_BTB_branch_count = (unsigned long long)((mydata.br & 0xFC)>>2);
				total_return_mispred_count = (unsigned long long)(mydata.br & 0x03);
				tcghdr.to_bb_cnt++;
				break;
			case PROFTYPE_BR2:
				tcghdr.to_br_tag++;
				result=read_prof2_data2(fd,&mydata);
				if (result==0){
					unsigned int len = 0;

					current_insn_cnt += mydata.data.data.icnt;
					current_cycle_cnt += mydata.data.data.ccnt;
					if (exec_stack != NULL) {
						exec_stack->self_insn_cnt+=mydata.data.data.icnt;
						exec_stack->self_cycle_cnt+=mydata.data.data.ccnt;
						exec_stack->bb_cnt++;
						exec_stack->branch_taken_cnt += ((mydata.br & 0x02) >> 1);
						exec_stack->branch_misprediction_cnt += (mydata.br & 0x01);
						exec_stack->BTB_branch_cnt += ((mydata.br & 0xFC) >> 2);
						exec_stack->sym->hist.total_insn_cnt+=mydata.data.data.icnt;
						exec_stack->sym->hist.total_cycle_cnt+=mydata.data.data.ccnt;
					}
					// generate a temporary timeline record
					if (do_timeline){
						result = write_tl2_temp(mydata.data.tpc, mydata.data.data.pc, tag, function_level, 0, current_insn_cnt, current_cycle_cnt, 0, 0, 0, 0, 0, mydata.br,0,0,0,0, &len);
						if(result != 0)
							return 2;
					}
					temp_file_pos += len;
				}
				// branch prediction
				add_branch_count_to_sym(mydata.data.data.pc, 1);
				total_BTB_branch_count = (unsigned long long)((mydata.br & 0xFC)>>2);
				total_branch_taken_count = (unsigned long long)((mydata.br & 0x02)>>1);
				total_branch_mispred_count = (unsigned long long)(mydata.br & 0x01);
				tcghdr.to_bb_cnt++;
				break;
			case PROFTYPE_MOD:
				// mode changed
				result=read_profm_data(fd,&mydata2);
				if (result==0){
					// update current time point
					current_insn_cnt+=mydata2.data.icnt;
					current_cycle_cnt+=mydata2.data.ccnt;
					if (exec_stack != NULL ) {
						exec_stack->self_insn_cnt+=mydata2.data.icnt;
						exec_stack->self_cycle_cnt+=mydata2.data.ccnt;
						exec_stack->sym->hist.total_insn_cnt+=mydata2.data.icnt;
						exec_stack->sym->hist.total_cycle_cnt+=mydata2.data.ccnt;
					}
					current_unmapped=((mydata2.mode&IT_MASK)==0);
				}
				break;
			case PROFTYPE_OFF:
				// done reading profiling data
				result=(fread(&mydata.data.data,sizeof(prof1data),1,fd)!=1);
				if (result==0){
					unsigned int delta_icnt = 0, delta_ccnt = 0;
					int result = 0;

					function_level--;
					if(min_function_level > function_level)
						min_function_level = function_level;
					current_insn_cnt += mydata.data.data.icnt;
					current_cycle_cnt += mydata.data.data.ccnt;
					delta_icnt = mydata.data.data.icnt;
					delta_ccnt = mydata.data.data.ccnt;
					if (exec_stack != NULL)
					{
						while(exec_stack->next != NULL){
							tag = PROFTYPE_DUMMY_FR2;
							tcghdr.to_du_rt_tag++;
							result = pop_callee(exec_stack->sym->addr, exec_stack->next->sym->addr, delta_icnt, delta_ccnt, tag, 0,0,0,0,0,0,0,0,0,0,0,0,0,0);
							function_level--;
							if(min_function_level > function_level)
								min_function_level = function_level;
							delta_icnt = 0;
							delta_ccnt = 0;
						}
						tag = PROFTYPE_OFF;
						// profiling turned off, so clean up the execution stack
						result=pop_callee2(mydata.data.data.icnt,mydata.data.data.ccnt, tag, mydata.data.tpc, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,0,0);
					}
					else
						result=pop_callee(mydata.data.data.pc,mydata.data.tpc,mydata.data.data.icnt,mydata.data.data.ccnt,tag,mydata.br,0,0,0,0,0,0, 0,0,0,0,0,0,0);
				}
				tcghdr.to_bb_cnt++;
				return result;
			default:
				return 4;
		}
		if (result!=0)
			return result;
	}

	// read error - only successful exit is from PROFTYPE_OFF record
	return 1;
} // read_prof2_data

// ----------------------------------------------------------------------------
// tl2_read_rec
//
// This function reads the timeline based profiling data for branch coverage.
// The expected sequences are 0xc2, {0x21,0x22,0x23,0xcf}*, 0xc0.
// return code: (execution terminated if any error encountered.)
// 0x00 - end-of-file
// tag - next tag
// ----------------------------------------------------------------------------
static unsigned char
tl2_read_rec(FILE *fd,
             const char *filename)
{   unsigned char tag;
    int result;

    result=read_prof_on(fd,PROFTYPE_ON2);
    if (result==0)
        result=read_prof2_data(fd);
    if (result!=0)
        return prof_errmsg(result,filename); // no return
	if (temp_file_pos > TIMELINE_LIMIT)
		return 0x00;
    if (fread(&tag,sizeof(unsigned char),1,fd)!=1)
        tag=0x00; // can only be end-of-file

    return tag;
} // tl2_read_rec

// ----------------------------------------------------------------------------
// read_prof3_data2
//
// This function reads the cycle count and data fetch data for profiling level
// 3 from prof.out.
// return code:
// 0 - success
// 1 - read error
// ----------------------------------------------------------------------------
static int
read_prof3_data2(FILE *fd,
		prof3data2 *mydata2)
{
	int result;

	result = (fread(&mydata2->ccnt, sizeof(unsigned short), 1, fd) != 1);
	if(result == 0){
		result = (fread(&mydata2->dfetch1, sizeof(unsigned char), 1, fd) != 1);
		if(result == 0){
			result = (fread(&mydata2->dfetch2, sizeof(unsigned int), 1, fd) != 1);
			if(result == 0)
				result = (fread(&mydata2->dfetch3, sizeof(unsigned int), 1, fd) != 1);
		}
	}

	return result;
} // read_prof3_data2

// ----------------------------------------------------------------------------
// read_prof3_data3
//
// This function reads the cycle count and instruction fetch data for profiling
// level 3 from prof.out.
// return code:
// 0 - success
// 1 - read error
// ----------------------------------------------------------------------------
static int
read_prof3_data3(FILE *fd,
		prof3data3 *mydata3)
{
	int result;

	result = (fread(&mydata3->ccnt, sizeof(unsigned short), 1, fd) != 1);
	if(result == 0){
		result = (fread(&mydata3->ifetch, sizeof(unsigned short), 1, fd) != 1);
	}

	return result;
} // read_prof3_data3

// ----------------------------------------------------------------------------
// read_prof3_data1
//
// This function reads the cycle count, target pc, branch data and instruction
// fetch data for profiling level 3 from prof.out.
// return code:
// 0 - success
// 1 - read error
// ----------------------------------------------------------------------------
static int
read_prof3_data1(FILE *fd,
		prof3data1 *mydata1)
{
	int result;
	result = (fread(&mydata1->ccnt, sizeof(unsigned short), 1, fd) != 1);
	if(result == 0){
		result = (fread(&mydata1->tpc, sizeof(bfd_vma), 1, fd) != 1);
		if(result == 0){
			result = (fread(&mydata1->br, sizeof(unsigned char), 1, fd) != 1);
			if(result == 0){
				result = (fread(&mydata1->ifetch, sizeof(unsigned short), 1, fd) != 1);
			}
		}
	}

	return result;
}// read_prof3_data1


// ----------------------------------------------------------------------------
// read_prof3_data
//
// This function reads the instruction coverage profiling records from
// prof.out.
// return code:
// 0 - success
// 1 - read error
// 2 - write error
// 3 - memory error
// 4 - bad tag
// ----------------------------------------------------------------------------
static int
read_prof3_data(FILE *fd)
{
	int result;
	unsigned char tag;
	prof3data1 mydata1;
	prof3data2 mydata2;
	prof3data3 mydata3;
	profmdata mydatam;
	prof1data mydataoff;

	while (fread(&tag,sizeof(unsigned char),1,fd)==1)
	{

		if (temp_file_pos > TIMELINE_LIMIT)
		{
			// We have a timeline too big, change the tag to PROF_OFF to finish
			tag = PROFTYPE_OFF;
		}

		switch (tag){
			case PROFTYPE_FC3:
				tcghdr.to_fn_call_tag++;
				result = read_prof3_data1(fd, &mydata1);
				if (result == 0){
					unsigned int len = 0;

					CurrentPC += sizeof(bfd_vma);
					function_level++;
					if(function_level > max_function_level)
						max_function_level = function_level;
					current_insn_cnt++;
					current_cycle_cnt += mydata1.ccnt;
					if(do_timeline){
						result = write_tl3_temp(mydata1.tpc, CurrentPC, tag, function_level, 0, current_insn_cnt, current_cycle_cnt, 0, 0, 0, 0, 0, mydata1.br, mydata1.ifetch, 0, 0, 0, &len);
						if(result != 0)
							return 2;
					}
					result=push_callee(mydata1.tpc,CurrentPC,1,mydata1.ccnt, temp_file_pos, 0, 0, 0,0,0,0,0,0,0);
					temp_file_pos += len;
					CurrentPC = mydata1.tpc;
					total_BTB_branch_count = (unsigned long long)((mydata1.br & 0xFC) >> 2);
					total_branch_taken_count = (unsigned long long)((mydata1.br & 0x02) >> 1);
					total_branch_mispred_count = (unsigned long long)(mydata1.br & 0x01);
				}
				tcghdr.to_bb_cnt++;
				break;
			case PROFTYPE_FR3:
				tcghdr.to_fn_rt_tag++;
				result = read_prof3_data1(fd, &mydata1);
				if (result == 0){
					CurrentPC += sizeof(bfd_vma);
					function_level--;
					if(min_function_level > function_level)
						min_function_level = function_level;
					current_insn_cnt++;
					current_cycle_cnt += mydata1.ccnt;
					result=pop_callee(CurrentPC, mydata1.tpc, 1, mydata1.ccnt, tag, mydata1.br, mydata1.ifetch, 0, 0, 0,0,0,0,0,0,0,0,0,0);
					CurrentPC = mydata1.tpc;
					total_BTB_branch_count = (unsigned long long)((mydata1.br & 0xFC) >> 2);
					total_return_mispred_count = (unsigned long long)(mydata1.br & 0x03);
				}
				tcghdr.to_bb_cnt++;
				break;
			case PROFTYPE_BR3:
				tcghdr.to_br_tag++;
				result = read_prof3_data1(fd, &mydata1);
				if(result == 0){
					unsigned int len = 0;

					CurrentPC += sizeof(bfd_vma);
					current_insn_cnt++;
					current_cycle_cnt += mydata1.ccnt;
					if (exec_stack != NULL ){
						exec_stack->self_insn_cnt++;
						exec_stack->self_cycle_cnt += mydata1.ccnt;
						exec_stack->sym->hist.total_insn_cnt++;
						exec_stack->sym->hist.total_cycle_cnt += mydata1.ccnt;
					}
					if(do_timeline){
						result = write_tl3_temp(mydata1.tpc, CurrentPC, tag, function_level, 0, current_insn_cnt, current_cycle_cnt, 0, 0, 0, 0, 0, mydata1.br, mydata1.ifetch, 0, 0, 0, &len);
						fflush(temp_fd);
						if(result != 0)
							return 2;
						temp_file_pos += len;
					}
					add_branch_count_to_sym(CurrentPC, 1);
					total_BTB_branch_count = (unsigned long long)((mydata1.br & 0xFC) >> 2);
					total_branch_taken_count = (unsigned long long)((mydata1.br & 0x02) >> 1);
					total_branch_mispred_count = (unsigned long long)(mydata1.br & 0x01);
					tcghdr.to_bb_cnt++;
					CurrentPC = mydata1.tpc;
				}
				break;
			case PROFTYPE_MA3:
				result = read_prof3_data2(fd, &mydata2);
				if(result == 0){
					unsigned int len = 0;

					CurrentPC += sizeof(bfd_vma);
					current_insn_cnt++;
					current_cycle_cnt += mydata2.ccnt;
					if (exec_stack != NULL ){
						exec_stack->self_insn_cnt++;
						exec_stack->self_cycle_cnt += mydata2.ccnt;
						exec_stack->sym->hist.total_insn_cnt++;
						exec_stack->sym->hist.total_cycle_cnt += mydata2.ccnt;
					}
					if(do_timeline){
						result = write_tl3_temp((CurrentPC + sizeof(bfd_vma)), CurrentPC, tag, function_level, 0, current_insn_cnt, current_cycle_cnt, 0, 0, 0, 0, 0, 0, 0, mydata2.dfetch1, mydata2.dfetch2, mydata2.dfetch3, &len);
						fflush(temp_fd);
						if(result != 0)
							return 2;
						temp_file_pos += len;
					}
				}
				break;
			case PROFTYPE_OT3:
				result = read_prof3_data3(fd, &mydata3);
				if(result == 0){
					unsigned int len = 0;

					CurrentPC += sizeof(bfd_vma);
					current_insn_cnt++;
					current_cycle_cnt += mydata3.ccnt;
					if (exec_stack != NULL ) {
						exec_stack->self_insn_cnt++;
						exec_stack->self_cycle_cnt += mydata3.ccnt;
						exec_stack->sym->hist.total_insn_cnt++;
						exec_stack->sym->hist.total_cycle_cnt += mydata3.ccnt;
					}
					if(do_timeline){
						result = write_tl3_temp((CurrentPC + sizeof(bfd_vma)), CurrentPC, tag, function_level, 0, current_insn_cnt, current_cycle_cnt, 0, 0, 0, 0, 0, 0, mydata3.ifetch, 0, 0, 0, &len);
						if(result != 0)
							return 2;
						temp_file_pos += len;
					}
				}
				break;
			case PROFTYPE_MOD:
				result = read_profm_data(fd, &mydatam);
				if(result == 0){
					unsigned int len = 0;

					CurrentPC += sizeof(bfd_vma);
					current_insn_cnt += mydatam.data.icnt;
					current_cycle_cnt += mydatam.data.ccnt;
					if (exec_stack != NULL ) {
						exec_stack->self_insn_cnt += mydatam.data.icnt;
						exec_stack->self_cycle_cnt += mydatam.data.ccnt;
						exec_stack->sym->hist.total_insn_cnt += mydatam.data.icnt;
						exec_stack->sym->hist.total_cycle_cnt += mydatam.data.ccnt;
					}
					if(do_timeline){
						result = write_tl3_temp((CurrentPC + sizeof(bfd_vma)), CurrentPC, tag, function_level, 0, current_insn_cnt, current_cycle_cnt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, &len);
						if(result != 0)
							return 2;
						temp_file_pos += len;
					}
					current_unmapped = ((mydatam.mode & IT_MASK) == 0);
				}
				break;
			case PROFTYPE_OFF:
				result = (fread(&mydataoff, sizeof(prof1data), 1, fd) != 1);
				if(result == 0){
					unsigned int delta_icnt = 0, delta_ccnt = 0;

					CurrentPC = mydataoff.pc;
					function_level--;
					if(min_function_level > function_level)
						min_function_level = function_level;
					current_insn_cnt += mydataoff.icnt;
					current_cycle_cnt += mydataoff.ccnt;
					delta_icnt = mydataoff.icnt;
					delta_ccnt = mydataoff.ccnt;
					if (exec_stack != NULL )
					{
						while(exec_stack->next != NULL){
							tag = PROFTYPE_DUMMY_FR3;
							tcghdr.to_du_rt_tag++;
							result = pop_callee(exec_stack->sym->addr, exec_stack->next->sym->addr, delta_icnt, delta_ccnt, tag, 0, 0, 0, 0, 0,0,0,0,0,0,0,0,0,0);
							function_level--;
							if(min_function_level > function_level)
								min_function_level = function_level;
							delta_icnt = 0;
							delta_ccnt = 0;
						}
						tag = PROFTYPE_OFF;
						result = pop_callee2(mydataoff.icnt, mydataoff.ccnt, tag, 0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,0,0);
					}
					else
						result = pop_callee(CurrentPC, mydata1.tpc, 1, mydata1.ccnt, tag, mydata1.br, mydata1.ifetch, 0, 0, 0,0,0,0,0,0,0,0,0,0);
				}
				tcghdr.to_bb_cnt++;
				return result;
			default:
				return 4;
		} // End of switch
		if (result!=0)
			return result;
	}

	// read error - only successful exit is from PROFTYPE_OFF record
	return 1;
} // read_prof3_data

// ----------------------------------------------------------------------------
// tl3_read_rec
//
// This function reads the timeline based profiling data for instruction
// coverage. The expected sequences are 0xc3, {0x31,0x32,0x33,0x34,0x35,0xcf}*,
// 0xc0.
// return code: (execution terminated if any error encountered.)
// 0x00 - end-of-file
// tag - next tag
// ----------------------------------------------------------------------------
static unsigned char
tl3_read_rec(FILE *fd,
             const char *filename)
{   unsigned char tag;
    int result;

    result=read_prof_on(fd,PROFTYPE_ON3);
    if (result==0)
        result=read_prof3_data(fd);
    if (result!=0)
        return prof_errmsg(result,filename); // no return
	if (temp_file_pos > TIMELINE_LIMIT)
		return 0x00;
    if (fread(&tag,sizeof(unsigned char),1,fd)!=1)
        tag=0x00; // can only be end-of-file

    return tag;
} // tl3_read_rec

// ----------------------------------------------------------------------------
// read_prof6_data
//
// This function reads the function level and branch summary profiling records
// from prof.out.
// return code:
// 0 - success
// 1 - read error
// 2 - write error
// 3 - memory error
// 4 - bad tag
// ----------------------------------------------------------------------------
static int
read_prof6_data(FILE *fd)
{   int result;
    unsigned char tag;
    prof6data mydata;
    profmdata mydata2;



    while (fread(&tag,sizeof(unsigned char),1,fd)==1)
    {
		if (temp_file_pos > TIMELINE_LIMIT)
		{
			// We have a timeline too big, change the tag to PROF_OFF to finish
			tag = PROFTYPE_OFF;
		}

	switch (tag)
        {   case PROFTYPE_FC6:
		tcghdr.to_fn_call_tag++;
        // we have a call here
        result = (fread(&mydata, 18, 1, fd)!=1);
		// Analysis profile time
                if (result==0){
			unsigned int len = 0;
			unsigned long long br_taken_cnt = 0;
			unsigned long long br_mis_cnt = 0;
			unsigned long long BTB_br_cnt = 0;

			function_level++;
			if(function_level > max_function_level)
				max_function_level = function_level;
			current_insn_cnt += mydata.icnt;
			current_cycle_cnt += mydata.ccnt;
			// generate a temporary timeline record
			if (do_timeline){
				result = write_tl6_temp(mydata.tpc, mydata.pc, tag, function_level, 0, current_insn_cnt, current_cycle_cnt, 0, 0, 0, 0, 0, 0, 0, 0, &len);
				if(result != 0)
					return 2;
			}
			result = (fread(&br_taken_cnt, calculate_data_size(mydata.length[0], HIGH_NIBBLE), 1, fd) != 1);
			if(result == 0){
				result = (fread(&br_mis_cnt, calculate_data_size(mydata.length[0], LOW_NIBBLE), 1, fd) != 1);
				if(result == 0)
					result = (fread(&BTB_br_cnt, calculate_data_size(mydata.length[1], HIGH_NIBBLE), 1, fd) != 1);
			}
			if(result != 0){
				// Error
			}
			result=push_callee(mydata.tpc,mydata.pc,mydata.icnt,mydata.ccnt, temp_file_pos, br_taken_cnt, br_mis_cnt, BTB_br_cnt,0,0,0,0,0,0);
			temp_file_pos += len;
		}
                break;
            case PROFTYPE_FR6:
		tcghdr.to_fn_rt_tag++;

                // we have a return here
                result=(fread(&mydata, 18, 1, fd)!=1);

                if (result==0){
			unsigned long long br_taken_cnt = 0;
			unsigned long long br_mis_cnt = 0;
			unsigned long long BTB_br_cnt = 0;
			function_level--;
			if(function_level > max_function_level)
				min_function_level = function_level;
			current_insn_cnt += mydata.icnt;
			current_cycle_cnt += mydata.ccnt;
			result = (fread(&br_taken_cnt, calculate_data_size(mydata.length[0], HIGH_NIBBLE), 1, fd) != 1);
			if(result == 0){
				result = (fread(&br_mis_cnt, calculate_data_size(mydata.length[0], LOW_NIBBLE), 1, fd) != 1);
				if(result == 0)
					result = (fread(&BTB_br_cnt, calculate_data_size(mydata.length[1], HIGH_NIBBLE), 1, fd) != 1);
			}
			if(result != 0){
				// Error
			}
			result=pop_callee(mydata.pc,mydata.tpc,mydata.icnt,mydata.ccnt,tag,0,0,0,0,0, br_taken_cnt, br_mis_cnt, BTB_br_cnt,0,0,0,0,0,0);
		}
                break;
            case PROFTYPE_MOD:
                // mode changed
                result=read_profm_data(fd,&mydata2);

                if (result==0)
                {   // update current time point
                    current_insn_cnt+=mydata2.data.icnt;
                    current_cycle_cnt+=mydata2.data.ccnt;
					if (exec_stack != NULL ){
	                    exec_stack->self_insn_cnt+=mydata2.data.icnt;
	                    exec_stack->self_cycle_cnt+=mydata2.data.ccnt;
	                    exec_stack->sym->hist.total_insn_cnt+=mydata2.data.icnt;
	                    exec_stack->sym->hist.total_cycle_cnt+=mydata2.data.ccnt;
					}
                    // need to address all possible cause
                    // do we need target pc address? for instructions like
                    // jral.xton we do not need it again
                    current_unmapped=((mydata2.mode&IT_MASK)==0);
                }
                break;
            case PROFTYPE_OFF:

                // done reading profiling data
                result=(fread(&mydata, 12, 1, fd)!=1);

                if (result==0)
                {
			unsigned int delta_icnt = 0, delta_ccnt = 0;
			int result = 0;

			function_level--;
			if(min_function_level > function_level)
				min_function_level = function_level;
			current_insn_cnt += mydata.icnt;
			current_cycle_cnt += mydata.ccnt;
			delta_icnt = mydata.icnt;
			delta_ccnt = mydata.ccnt;
			if (exec_stack != NULL )
			{
				while(exec_stack->next != NULL){
					tag = PROFTYPE_DUMMY_FR6;
					tcghdr.to_du_rt_tag++;
					result = pop_callee(exec_stack->sym->addr, exec_stack->next->sym->addr, delta_icnt, delta_ccnt, tag, 0,0,0,0,0,0,0,0,0,0,0,0,0,0);
					function_level--;
					if(min_function_level > function_level)
						min_function_level = function_level;
					delta_icnt = 0;
					delta_ccnt = 0;
				}
				tag = PROFTYPE_OFF;
				// profiling turned off, so clean up the execution stack
				result=pop_callee2(delta_icnt, delta_ccnt, tag, 0, 0, 0, 0, 0, 0,0,0,0,0,0,0,0,0,0);
			}
			else
				result=pop_callee(mydata.pc,mydata.tpc,mydata.icnt,mydata.ccnt,tag,0,0,0,0,0, 0,0,0,0,0,0,0,0,0);
                }
		return result;
            default:
                return 4;
        }
      if (result!=0)
            return result;

    }

    // read error - only successful exit is from PROFTYPE_OFF record
    return 1;
} // read_prof6_data

// ----------------------------------------------------------------------------
// tl6_read_rec
//
// This function reads the timeline based profiling data for function level
// and branch summary. The exceptd sequences are 0xc6, (0x61, 0x62)*, 0xc0.
// return code: (execution terminated if any error encountered.)
// 0x00 - end-of-file
// tag - next tag
// ----------------------------------------------------------------------------
static unsigned char
tl6_read_rec(FILE *fd,
		const char *filename)
{	unsigned char tag = 0;
	int result = 0;

	result = read_prof_on(fd, PROFTYPE_ON6);
	if(result == 0)
		result = read_prof6_data(fd);
	if(result != 0)
		return prof_errmsg(result, filename);

	if (temp_file_pos > TIMELINE_LIMIT)
		return 0x00;

	if(fread(&tag, sizeof(unsigned char), 1, fd) != 1)
		tag = 0x00;

	return tag;
} // tl6_read_rec

// ----------------------------------------------------------------------------
// read_prof7_data
//
// This function reads the function level and branch summary profiling records
// from prof.out.
// return code:
// 0 - success
// 1 - read error
// 2 - write error
// 3 - memory error
// 4 - bad tag
// ----------------------------------------------------------------------------
static int
read_prof7_data(FILE *fd)
{   int result;
    unsigned char tag;
    prof7data mydata;
    profmdata mydata2;

    while (fread(&tag,sizeof(unsigned char),1,fd)==1)
    {
		if (temp_file_pos > TIMELINE_LIMIT)
		{
			// We have a timeline too big, change the tag to PROF_OFF to finish
			tag = PROFTYPE_OFF;
		}


	switch (tag)
        {
	    case PROFTYPE_FC7:
		tcghdr.to_fn_call_tag++;

                // we have a call here
                result = (fread(&mydata, 19, 1, fd)!=1);

                if (result==0){
			unsigned int len = 0;
			unsigned long long icache_replace_cnt = 0;
			unsigned long long icache_miss_cnt = 0;
			unsigned long long icache_access_cnt = 0;
			unsigned long long dcache_replace_cnt = 0;
			unsigned long long dcache_miss_cnt = 0;
			unsigned long long dcache_access_cnt = 0;

			function_level++;
			if(function_level > max_function_level)
				max_function_level = function_level;
			current_insn_cnt += mydata.icnt;
			current_cycle_cnt += mydata.ccnt;
			// generate a temporary timeline record
			if (do_timeline){
				result = write_tl7_temp(mydata.tpc, mydata.pc, tag, function_level, 0, current_insn_cnt, current_cycle_cnt, 0, 0, 0, 0, 0,0,0,0,0,0,0, &len);
				if(result != 0)
					return 2;
			}

			result = (fread(&icache_replace_cnt, calculate_data_size(mydata.length[0], HIGH_NIBBLE), 1, fd) != 1);
			if(result == 0){
				result = (fread(&icache_miss_cnt, calculate_data_size(mydata.length[0], LOW_NIBBLE), 1, fd) != 1);
				if(result == 0){
					result = (fread(&icache_access_cnt, calculate_data_size(mydata.length[1], HIGH_NIBBLE), 1, fd) != 1);
					if(result == 0){
						result = (fread(&dcache_replace_cnt, calculate_data_size(mydata.length[1], LOW_NIBBLE), 1, fd) != 1);
						if(result == 0){
							result = (fread(&dcache_miss_cnt, calculate_data_size(mydata.length[2], HIGH_NIBBLE), 1, fd) != 1);
							if(result == 0)
								result = (fread(&dcache_access_cnt, calculate_data_size(mydata.length[2], LOW_NIBBLE), 1, fd) != 1);
						}
					}
				}
			}

			result=push_callee(mydata.tpc,mydata.pc,mydata.icnt,mydata.ccnt, temp_file_pos, 0,0,0, icache_replace_cnt, icache_miss_cnt, icache_access_cnt, dcache_replace_cnt, dcache_miss_cnt, dcache_access_cnt);
			temp_file_pos += len;
		}
                break;
            case PROFTYPE_FR7:
		tcghdr.to_fn_rt_tag++;

                // we have a return here
                result=(fread(&mydata, 19, 1, fd)!=1);

                if (result==0){
			unsigned long long icache_replace_cnt = 0;
			unsigned long long icache_miss_cnt = 0;
			unsigned long long icache_access_cnt = 0;
			unsigned long long dcache_replace_cnt = 0;
			unsigned long long dcache_miss_cnt = 0;
			unsigned long long dcache_access_cnt = 0;
			function_level--;
			if(function_level > max_function_level)
				min_function_level = function_level;
			current_insn_cnt += mydata.icnt;
			current_cycle_cnt += mydata.ccnt;

			result = (fread(&icache_replace_cnt, calculate_data_size(mydata.length[0], HIGH_NIBBLE), 1, fd) != 1);
			if(result == 0){
				result = (fread(&icache_miss_cnt, calculate_data_size(mydata.length[0], LOW_NIBBLE), 1, fd) != 1);
				if(result == 0){
					result = (fread(&icache_access_cnt, calculate_data_size(mydata.length[1], HIGH_NIBBLE), 1, fd) != 1);
					if(result == 0){
						result = (fread(&dcache_replace_cnt, calculate_data_size(mydata.length[1], LOW_NIBBLE), 1, fd) != 1);
						if(result == 0){
							result = (fread(&dcache_miss_cnt, calculate_data_size(mydata.length[2], HIGH_NIBBLE), 1, fd) != 1);
							if(result == 0)
								result = (fread(&dcache_access_cnt, calculate_data_size(mydata.length[2], LOW_NIBBLE), 1, fd) != 1);
						}
					}
				}
			}

			result=pop_callee(mydata.pc,mydata.tpc,mydata.icnt,mydata.ccnt,tag,0,0,0,0,0, 0,0,0, icache_replace_cnt, icache_miss_cnt, icache_access_cnt, dcache_replace_cnt, dcache_miss_cnt, dcache_access_cnt);
		}
                break;
            case PROFTYPE_MOD:

                // mode changed
                result=read_profm_data(fd,&mydata2);

                if (result==0)
                {   // update current time point
                    current_insn_cnt+=mydata2.data.icnt;
                    current_cycle_cnt+=mydata2.data.ccnt;
					if (exec_stack != NULL ){
	                    exec_stack->self_insn_cnt+=mydata2.data.icnt;
	                    exec_stack->self_cycle_cnt+=mydata2.data.ccnt;
	                    exec_stack->sym->hist.total_insn_cnt+=mydata2.data.icnt;
	                    exec_stack->sym->hist.total_cycle_cnt+=mydata2.data.ccnt;
					}

                    // need to address all possible cause
                    // do we need target pc address? for instructions like
                    // jral.xton we do not need it again
                    current_unmapped=((mydata2.mode&IT_MASK)==0);
                }
                break;
            case PROFTYPE_OFF:

                // done reading profiling data
                result=(fread(&mydata, 12, 1, fd)!=1);

                if (result==0)
                {
			unsigned int delta_icnt = 0, delta_ccnt = 0;
			int result = 0;

			function_level--;
			if(min_function_level > function_level)
				min_function_level = function_level;
			current_insn_cnt += mydata.icnt;
			current_cycle_cnt += mydata.ccnt;
			delta_icnt = mydata.icnt;
			delta_ccnt = mydata.ccnt;
			if (exec_stack != NULL )
			{
				while(exec_stack->next != NULL){
					tag = PROFTYPE_DUMMY_FR7;
					tcghdr.to_du_rt_tag++;
					result = pop_callee(exec_stack->sym->addr, exec_stack->next->sym->addr, delta_icnt, delta_ccnt, tag, 0,0,0,0,0,0,0,0,0,0,0,0,0,0);
					function_level--;
					if(min_function_level > function_level)
						min_function_level = function_level;
					delta_icnt = 0;
					delta_ccnt = 0;
				}
				tag = PROFTYPE_OFF;
				// profiling turned off, so clean up the execution stack
				result=pop_callee2(delta_icnt, delta_ccnt, tag, 0, 0, 0, 0, 0, 0,0,0,0,0,0,0,0,0,0);
			}
			else
				result=pop_callee(mydata.pc,mydata.tpc,mydata.icnt,mydata.ccnt,tag,0,0,0,0,0, 0,0,0, 0, 0, 0, 0, 0, 0);
                }
		return result;
            default:
                return 4;
        }
      if (result!=0)
            return result;


    }

    // read error - only successful exit is from PROFTYPE_OFF record
    return 1;
} // read_prof7_data

// ----------------------------------------------------------------------------
// tl7_read_rec
//
// This function reads the timeline based profiling data for function level
// and cache summary. The exceptd sequences are 0xc7, (0x71, 0x72)*, 0xc0.
// return code: (execution terminated if any error encountered.)
// 0x00 - end-of-file
// tag - next tag
// ----------------------------------------------------------------------------
static unsigned char
tl7_read_rec(FILE *fd,
		const char *filename)
{	unsigned char tag = 0;
	int result = 0;

	result = read_prof_on(fd, PROFTYPE_ON7);
	if(result == 0)
		result = read_prof7_data(fd);
	if(result != 0)
		return prof_errmsg(result, filename);

	if (temp_file_pos > TIMELINE_LIMIT)
		return 0x00;

	if(fread(&tag, sizeof(unsigned char), 1, fd) != 1)
		tag = 0x00;

	return tag;
} // tl7_read_rec

// ----------------------------------------------------------------------------
// read_prof8_data
//
// This function reads the function level and branch & cache summary profiling
// records from prof.out.
// return code:
// 0 - success
// 1 - read error
// 2 - write error
// 3 - memory error
// 4 - bad tag
// ----------------------------------------------------------------------------
static int
read_prof8_data(FILE *fd)
{
	int result;
    unsigned char tag;
    prof8data mydata;
    profmdata mydata2;

    while (fread(&tag,sizeof(unsigned char),1,fd)==1)
    {
		if (temp_file_pos > TIMELINE_LIMIT)
		{
			// We have a timeline too big, change the tag to PROF_OFF to finish
			tag = PROFTYPE_OFF;
		}

	switch (tag)
        {
	    case PROFTYPE_FC8:
		tcghdr.to_fn_call_tag++;


        // we have a call here
        result = (fread(&mydata, 21, 1, fd)!=1);

                if (result==0){
			unsigned int len = 0;
			unsigned long long br_taken_cnt = 0;
			unsigned long long br_mis_cnt = 0;
			unsigned long long BTB_br_cnt = 0;
			unsigned long long icache_replace_cnt = 0;
			unsigned long long icache_miss_cnt = 0;
			unsigned long long icache_access_cnt = 0;
			unsigned long long dcache_replace_cnt = 0;
			unsigned long long dcache_miss_cnt = 0;
			unsigned long long dcache_access_cnt = 0;

			function_level++;
			if(function_level > max_function_level)
				max_function_level = function_level;
			current_insn_cnt += mydata.icnt;
			current_cycle_cnt += mydata.ccnt;
			// generate a temporary timeline record
			if (do_timeline){
				result = write_tl8_temp(mydata.tpc, mydata.pc, tag, function_level, 0, current_insn_cnt, current_cycle_cnt, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,0,0, &len);
				if(result != 0)
					return 2;
			}
			result = (fread(&br_taken_cnt, calculate_data_size(mydata.length[0], HIGH_NIBBLE), 1, fd) != 1);
			if(result == 0){
				result = (fread(&br_mis_cnt, calculate_data_size(mydata.length[0], LOW_NIBBLE), 1, fd) != 1);
				if(result == 0){
					result = (fread(&BTB_br_cnt, calculate_data_size(mydata.length[1], HIGH_NIBBLE), 1, fd) != 1);
					if(result == 0){
						result = (fread(&icache_replace_cnt, calculate_data_size(mydata.length[1], LOW_NIBBLE), 1, fd) != 1);
						if(result == 0){
							result = (fread(&icache_miss_cnt, calculate_data_size(mydata.length[2], HIGH_NIBBLE), 1, fd) != 1);
							if(result == 0){
								result = (fread(&icache_access_cnt, calculate_data_size(mydata.length[2], LOW_NIBBLE), 1, fd) != 1);
								if(result == 0){
									result = (fread(&dcache_replace_cnt, calculate_data_size(mydata.length[3], HIGH_NIBBLE), 1, fd) != 1);
									if(result == 0){
										result = (fread(&dcache_miss_cnt, calculate_data_size(mydata.length[3], LOW_NIBBLE), 1, fd) != 1);
										if(result == 0){
											result = (fread(&dcache_access_cnt, calculate_data_size(mydata.length[4], HIGH_NIBBLE), 1, fd) != 1);
										}
									}
								}
							}
						}
					}
				}
			}
			if(result != 0){
				// Error
			}
			result=push_callee(mydata.tpc,mydata.pc,mydata.icnt,mydata.ccnt, temp_file_pos, br_taken_cnt, br_mis_cnt, BTB_br_cnt, icache_replace_cnt, icache_miss_cnt, icache_access_cnt, dcache_replace_cnt, dcache_miss_cnt, dcache_access_cnt);
			temp_file_pos += len;
		}
                break;


            case PROFTYPE_FR8:
		tcghdr.to_fn_rt_tag++;

                // we have a return here
                result=(fread(&mydata, 21, 1, fd)!=1);

                if (result==0){
			unsigned long long br_taken_cnt = 0;
			unsigned long long br_mis_cnt = 0;
			unsigned long long BTB_br_cnt = 0;
			unsigned long long icache_replace_cnt = 0;
			unsigned long long icache_miss_cnt = 0;
			unsigned long long icache_access_cnt = 0;
			unsigned long long dcache_replace_cnt = 0;
			unsigned long long dcache_miss_cnt = 0;
			unsigned long long dcache_access_cnt = 0;
			function_level--;
			if(function_level > max_function_level)
				min_function_level = function_level;
			current_insn_cnt += mydata.icnt;
			current_cycle_cnt += mydata.ccnt;
			result = (fread(&br_taken_cnt, calculate_data_size(mydata.length[0], HIGH_NIBBLE), 1, fd) != 1);
			if(result == 0){
				result = (fread(&br_mis_cnt, calculate_data_size(mydata.length[0], LOW_NIBBLE), 1, fd) != 1);
				if(result == 0){
					result = (fread(&BTB_br_cnt, calculate_data_size(mydata.length[1], HIGH_NIBBLE), 1, fd) != 1);
					if(result == 0){
						result = (fread(&icache_replace_cnt, calculate_data_size(mydata.length[1], LOW_NIBBLE), 1, fd) != 1);
						if(result == 0){
							result = (fread(&icache_miss_cnt, calculate_data_size(mydata.length[2], HIGH_NIBBLE), 1, fd) != 1);
							if(result == 0){
								result = (fread(&icache_access_cnt, calculate_data_size(mydata.length[2], LOW_NIBBLE), 1, fd) != 1);
								if(result == 0){
									result = (fread(&dcache_replace_cnt, calculate_data_size(mydata.length[3], HIGH_NIBBLE), 1, fd) != 1);
									if(result == 0){
										result = (fread(&dcache_miss_cnt, calculate_data_size(mydata.length[3], LOW_NIBBLE), 1, fd) != 1);
										if(result == 0){
											result = (fread(&dcache_access_cnt, calculate_data_size(mydata.length[4], HIGH_NIBBLE), 1, fd) != 1);
										}
									}
								}
							}
						}
					}
				}
			}
			if(result != 0){
				// Error
			}
			result=pop_callee(mydata.pc,mydata.tpc,mydata.icnt,mydata.ccnt,tag,0,0,0,0,0, br_taken_cnt, br_mis_cnt, BTB_br_cnt, icache_replace_cnt, icache_miss_cnt, icache_access_cnt, dcache_replace_cnt, dcache_miss_cnt, dcache_access_cnt);
		}
                break;

            case PROFTYPE_MOD:

                // mode changed
                result=read_profm_data(fd,&mydata2);

                if (result==0)
                {   // update current time point
                    current_insn_cnt+=mydata2.data.icnt;
                    current_cycle_cnt+=mydata2.data.ccnt;
					if (exec_stack != NULL ) {
	                    exec_stack->self_insn_cnt+=mydata2.data.icnt;
	                    exec_stack->self_cycle_cnt+=mydata2.data.ccnt;
	                    exec_stack->sym->hist.total_insn_cnt+=mydata2.data.icnt;
	                    exec_stack->sym->hist.total_cycle_cnt+=mydata2.data.ccnt;
					}

                    // need to address all possible cause
                    // do we need target pc address? for instructions like
                    // jral.xton we do not need it again
                    current_unmapped=((mydata2.mode&IT_MASK)==0);
                }
                break;

            case PROFTYPE_OFF:
                // done reading profiling data
                result=(fread(&mydata, 12, 1, fd)!=1);

                if (result==0)
		{
			unsigned int delta_icnt = 0, delta_ccnt = 0;
			int result = 0;

			function_level--;
			if(min_function_level > function_level)
				min_function_level = function_level;
			current_insn_cnt += mydata.icnt;
			current_cycle_cnt += mydata.ccnt;
			delta_icnt = mydata.icnt;
			delta_ccnt = mydata.ccnt;
			if (exec_stack != NULL )
			{
				while(exec_stack->next != NULL){
					tag = PROFTYPE_DUMMY_FR8;
					tcghdr.to_du_rt_tag++;
					result = pop_callee(exec_stack->sym->addr, exec_stack->next->sym->addr, delta_icnt, delta_ccnt, tag, 0,0,0,0,0,0,0,0,0,0,0,0,0,0);
					function_level--;
					if(min_function_level > function_level)
						min_function_level = function_level;
					delta_icnt = 0;
					delta_ccnt = 0;
				}
				tag = PROFTYPE_OFF;
				// profiling turned off, so clean up the execution stack
				result=pop_callee2(delta_icnt, delta_ccnt, tag, 0, 0, 0, 0, 0, 0,0,0,0,0,0,0,0,0,0);
			}
			else
				result=pop_callee(mydata.pc,mydata.tpc,mydata.icnt,mydata.ccnt,tag,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	       }
		return result;
            default:
                return 4;
        }
      if (result!=0)
            return result;

    }

    // read error - only successful exit is from PROFTYPE_OFF record
    return 1;
} // read_prof8_data

// ----------------------------------------------------------------------------
// tl8_read_rec
//
// This function reads the timeline based profiling data for function level
// and branch & cache summary. The exceptd sequences are 0xc8, (0x81, 0x82)*,
// 0xc0.
// return code: (execution terminated if any error encountered.)
// 0x00 - end-of-file
// tag - next tag
// ----------------------------------------------------------------------------
static unsigned char
tl8_read_rec(FILE *fd,
		const char *filename)
{	unsigned char tag = 0;
	int result = 0;

	result = read_prof_on(fd, PROFTYPE_ON8);
	if(result == 0)
		result = read_prof8_data(fd);
	if(result != 0)
		return prof_errmsg(result, filename);

	if (temp_file_pos > TIMELINE_LIMIT)
		return 0x00;

	if(fread(&tag, sizeof(unsigned char), 1, fd) != 1)
		tag = 0x00;

	return tag;
} // tl8_read_rec

// ----------------------------------------------------------------------------
// read_prof9_data
//
// This function reads the branch level and cache summary profiling records from
// prof.out.
// return code:
// 0 - success
// 1 - read error
// 2 - write error
// 3 - memory error
// 4 - bad tag
// ----------------------------------------------------------------------------
static int
read_prof9_data(FILE *fd)
{   int result;
    unsigned char tag;
	prof9data mydata;
    profmdata mydata2;

	while (fread(&tag,sizeof(unsigned char),1,fd)==1)
	{
		if (temp_file_pos > TIMELINE_LIMIT)
		{
			// We have a timeline too big, change the tag to PROF_OFF to finish
			tag = PROFTYPE_OFF;
		}

		switch (tag)
		{
			case PROFTYPE_FC9:
				tcghdr.to_fn_call_tag++;
				result = (fread(&mydata, 20, 1, fd)!=1);
				if (result==0){
					unsigned int len = 0;
					unsigned long long icache_replace_cnt = 0;
					unsigned long long icache_miss_cnt = 0;
					unsigned long long icache_access_cnt = 0;
					unsigned long long dcache_replace_cnt = 0;
					unsigned long long dcache_miss_cnt = 0;
					unsigned long long dcache_access_cnt = 0;

					function_level++;
					if(function_level > max_function_level)
						max_function_level = function_level;
					current_insn_cnt += mydata.icnt;
					current_cycle_cnt += mydata.ccnt;

					result = (fread(&icache_replace_cnt, calculate_data_size(mydata.length[0], HIGH_NIBBLE), 1, fd) != 1);
					if(result == 0){
						result = (fread(&icache_miss_cnt, calculate_data_size(mydata.length[0], LOW_NIBBLE), 1, fd) != 1);
						if(result == 0){
							result = (fread(&icache_access_cnt, calculate_data_size(mydata.length[1], HIGH_NIBBLE), 1, fd) != 1);
							if(result == 0){
								result = (fread(&dcache_replace_cnt, calculate_data_size(mydata.length[1], LOW_NIBBLE), 1, fd) != 1);
								if(result == 0){
									result = (fread(&dcache_miss_cnt, calculate_data_size(mydata.length[2], HIGH_NIBBLE), 1, fd) != 1);
									if(result == 0)
										result = (fread(&dcache_access_cnt, calculate_data_size(mydata.length[2], LOW_NIBBLE), 1, fd) != 1);
								}
							}
						}
					}

					// generate a temporary timeline record
					if (do_timeline){
						result = write_tl9_temp(mydata.tpc, mydata.pc, tag, function_level, 0, current_insn_cnt, current_cycle_cnt, 0, 0, 0, 0, 0, mydata.br,0,0,0,0, icache_replace_cnt, icache_miss_cnt, icache_access_cnt, dcache_replace_cnt, dcache_miss_cnt, dcache_access_cnt,0,0,0,0,0,0, &len);
						if(result != 0)
							return 2;
					}
					result=push_callee(mydata.tpc,mydata.pc,mydata.icnt,mydata.ccnt, temp_file_pos, ((mydata.br & 0x02) >> 1), (mydata.br & 0x01), ((mydata.br & 0xFC) >> 2), icache_replace_cnt, icache_miss_cnt, icache_access_cnt, dcache_replace_cnt, dcache_miss_cnt, dcache_access_cnt);
					temp_file_pos += len;
				}

				// branch prediction
				total_BTB_branch_count = (unsigned long long)((mydata.br & 0xFC)>>2);
				total_branch_taken_count = (unsigned long long)((mydata.br & 0x02)>>1);
				total_branch_mispred_count = (unsigned long long)(mydata.br & 0x01);
				tcghdr.to_bb_cnt++;

				break;

			case PROFTYPE_FR9:
				tcghdr.to_fn_rt_tag++;
				result = (fread(&mydata, 20, 1, fd)!=1);
				if (result==0){
					unsigned long long icache_replace_cnt = 0;
					unsigned long long icache_miss_cnt = 0;
					unsigned long long icache_access_cnt = 0;
					unsigned long long dcache_replace_cnt = 0;
					unsigned long long dcache_miss_cnt = 0;
					unsigned long long dcache_access_cnt = 0;
					function_level--;
					if(min_function_level > function_level)
						min_function_level = function_level;
					current_insn_cnt += mydata.icnt;
					current_cycle_cnt += mydata.ccnt;

					result = (fread(&icache_replace_cnt, calculate_data_size(mydata.length[0], HIGH_NIBBLE), 1, fd) != 1);
					if(result == 0){
						result = (fread(&icache_miss_cnt, calculate_data_size(mydata.length[0], LOW_NIBBLE), 1, fd) != 1);
						if(result == 0){
							result = (fread(&icache_access_cnt, calculate_data_size(mydata.length[1], HIGH_NIBBLE), 1, fd) != 1);
							if(result == 0){
								result = (fread(&dcache_replace_cnt, calculate_data_size(mydata.length[1], LOW_NIBBLE), 1, fd) != 1);
								if(result == 0){
									result = (fread(&dcache_miss_cnt, calculate_data_size(mydata.length[2], HIGH_NIBBLE), 1, fd) != 1);
									if(result == 0)
										result = (fread(&dcache_access_cnt, calculate_data_size(mydata.length[2], LOW_NIBBLE), 1, fd) != 1);
								}
							}
						}
					}
					if(result != 0){
						// Error
					}

					result=pop_callee(mydata.pc,mydata.tpc,mydata.icnt,mydata.ccnt,tag,mydata.br,0,0,0,0, ((mydata.br & 0x02) >> 1), (mydata.br & 0x01), ((mydata.br & 0xFC) >> 2), icache_replace_cnt, icache_miss_cnt, icache_access_cnt, dcache_replace_cnt, dcache_miss_cnt, dcache_access_cnt);
				}
				// branch prediction
				total_BTB_branch_count = (unsigned long long)((mydata.br & 0xFC)>>2);
				total_return_mispred_count = (unsigned long long)(mydata.br & 0x03);
				tcghdr.to_bb_cnt++;

				break;

			case PROFTYPE_BR9:
				tcghdr.to_br_tag++;
				result = (fread(&mydata, 20, 1, fd)!=1);
				if (result==0){
					unsigned int len = 0;
					unsigned long long icache_replace_cnt = 0;
					unsigned long long icache_miss_cnt = 0;
					unsigned long long icache_access_cnt = 0;
					unsigned long long dcache_replace_cnt = 0;
					unsigned long long dcache_miss_cnt = 0;
					unsigned long long dcache_access_cnt = 0;

					current_insn_cnt += mydata.icnt;
					current_cycle_cnt += mydata.ccnt;
					if (exec_stack != NULL ) {
						exec_stack->self_insn_cnt += mydata.icnt;
						exec_stack->self_cycle_cnt += mydata.ccnt;
						exec_stack->sym->hist.total_insn_cnt += mydata.icnt;
						exec_stack->sym->hist.total_cycle_cnt += mydata.ccnt;
					}

					result = (fread(&icache_replace_cnt, calculate_data_size(mydata.length[0], HIGH_NIBBLE), 1, fd) != 1);
					if(result == 0){
						result = (fread(&icache_miss_cnt, calculate_data_size(mydata.length[0], LOW_NIBBLE), 1, fd) != 1);
						if(result == 0){
							result = (fread(&icache_access_cnt, calculate_data_size(mydata.length[1], HIGH_NIBBLE), 1, fd) != 1);
							if(result == 0){
								result = (fread(&dcache_replace_cnt, calculate_data_size(mydata.length[1], LOW_NIBBLE), 1, fd) != 1);
								if(result == 0){
									result = (fread(&dcache_miss_cnt, calculate_data_size(mydata.length[2], HIGH_NIBBLE), 1, fd) != 1);
									if(result == 0)
										result = (fread(&dcache_access_cnt, calculate_data_size(mydata.length[2], LOW_NIBBLE), 1, fd) != 1);
								}
							}
						}
					}

					if (exec_stack != NULL ) {
						exec_stack->bb_cnt++;
						exec_stack->branch_taken_cnt += ((mydata.br & 0x02) >> 1);
						exec_stack->branch_misprediction_cnt += (mydata.br & 0x01);
						exec_stack->BTB_branch_cnt += ((mydata.br & 0xFC) >> 2);
						exec_stack->icache_replace_cnt += icache_replace_cnt;
						exec_stack->icache_miss_cnt += icache_miss_cnt;
						exec_stack->icache_access_cnt += icache_access_cnt;
						exec_stack->dcache_replace_cnt += dcache_replace_cnt;
						exec_stack->dcache_miss_cnt += dcache_miss_cnt;
						exec_stack->dcache_access_cnt += dcache_access_cnt;
					}
					// generate a temporary timeline record
					if (do_timeline){
						result = write_tl9_temp(mydata.tpc, mydata.pc, tag, function_level, 0, current_insn_cnt, current_cycle_cnt, 0, 0, 0, 0, 0, mydata.br,0,0,0,0, icache_replace_cnt, icache_miss_cnt, icache_access_cnt, dcache_replace_cnt, dcache_miss_cnt, dcache_access_cnt,0,0,0,0,0,0, &len);
						if(result != 0)
							return 2;
					}
					temp_file_pos += len;
				}

				// branch prediction
				add_branch_count_to_sym(mydata.pc, 1);
				total_BTB_branch_count = (unsigned long long)((mydata.br & 0xFC)>>2);
				total_branch_taken_count = (unsigned long long)((mydata.br & 0x02)>>1);
				total_branch_mispred_count = (unsigned long long)(mydata.br & 0x01);
				tcghdr.to_bb_cnt++;
				break;

			case PROFTYPE_MOD:
				// mode changed
				result=read_profm_data(fd,&mydata2);
				if (result==0){
					// update current time point
					current_insn_cnt+=mydata2.data.icnt;
					current_cycle_cnt+=mydata2.data.ccnt;
					if (exec_stack != NULL ) {
						exec_stack->self_insn_cnt+=mydata2.data.icnt;
						exec_stack->self_cycle_cnt+=mydata2.data.ccnt;
						exec_stack->sym->hist.total_insn_cnt+=mydata2.data.icnt;
						exec_stack->sym->hist.total_cycle_cnt+=mydata2.data.ccnt;
					}
					current_unmapped=((mydata2.mode&IT_MASK)==0);
				}
				break;
			case PROFTYPE_OFF:
				// done reading profiling data
				result = (fread(&mydata, 12, 1, fd) != 1);
				if (result==0){
					unsigned int delta_icnt = 0, delta_ccnt = 0;
					int result = 0;

					function_level--;
					if(min_function_level > function_level)
						min_function_level = function_level;
					current_insn_cnt += mydata.icnt;
					current_cycle_cnt += mydata.ccnt;
					delta_icnt = mydata.icnt;
					delta_ccnt = mydata.ccnt;
					if (exec_stack != NULL )
					{
						while(exec_stack->next != NULL){
							tag = PROFTYPE_DUMMY_FR9;
							tcghdr.to_du_rt_tag++;
							result = pop_callee(exec_stack->sym->addr, exec_stack->next->sym->addr, delta_icnt, delta_ccnt, tag, 0,0,0,0,0,0,0,0,0,0,0,0,0,0);
							function_level--;
							if(min_function_level > function_level)
								min_function_level = function_level;
							delta_icnt = 0;
							delta_ccnt = 0;
						}
						tag = PROFTYPE_OFF;
						// profiling turned off, so clean up the execution stack
						result=pop_callee2(delta_icnt, delta_ccnt, tag, 0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,0,0);
					}
					else
						result=pop_callee(mydata.pc,mydata.tpc,mydata.icnt,mydata.ccnt,tag,mydata.br,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
				}
				tcghdr.to_bb_cnt++;
				return result;
			default:
				return 4;
		}
		if (result!=0)
			return result;
	}

	// read error - only successful exit is from PROFTYPE_OFF record
	return 1;
} // read_prof9_data

// ----------------------------------------------------------------------------
// tl9_read_rec
//
// This function reads the timeline based profiling data for branch level and
// cache summary. The exceptd sequences are 0xc9, (0x91, 0x92, 0x93)*, 0xc0.
// return code: (execution terminated if any error encountered.)
// 0x00 - end-of-file
// tag - next tag
// ----------------------------------------------------------------------------
static unsigned char
tl9_read_rec(FILE *fd,
		const char *filename)
{	unsigned char tag = 0;
	int result = 0;

	result = read_prof_on(fd, PROFTYPE_ON9);
	if(result == 0)
		result = read_prof9_data(fd);
	if(result != 0)
		return prof_errmsg(result, filename);

	if (temp_file_pos > TIMELINE_LIMIT)
		return 0x00;

	if(fread(&tag, sizeof(unsigned char), 1, fd) != 1)
		tag = 0x00;

	return tag;
} // tl9_read_rec

#define SIZE_SF_HASH 64
#define MASK_SF_HASH 0x1f8
static struct sf_hashT
{   const char *name;
    unsigned int offset;
    struct sf_hashT *next;
} *sf_hash[SIZE_SF_HASH]={NULL};

// ----------------------------------------------------------------------------
// get_src_file
// ----------------------------------------------------------------------------
static unsigned int get_src_file(Sym *sym,
                                 tcgfname *fptr,
                                 unsigned int len)
{   int slot=(((unsigned long)sym->file)&MASK_SF_HASH)>>3;
    struct sf_hashT *ptr;

    for (ptr=sf_hash[slot];ptr!=NULL;ptr=ptr->next)
    {   if (ptr->name==sym->file->name)
        {   // already taken cared of
            fptr->sfoffset=ptr->offset;
            return 0;
        }
    }
    fptr->sfoffset=len;

    // create this node
    ptr=(struct sf_hashT*)malloc(sizeof(struct sf_hashT));
    if (ptr==NULL)
        return 0;
    else
    {   // create the hash table entry
        ptr->name=sym->file->name;
        ptr->offset=len;
        ptr->next=sf_hash[slot];
        sf_hash[slot]=ptr;

        return strlen(sym->file->name)+1;
    }
} // get_src_file

// ----------------------------------------------------------------------------
// collect_names
//
// This function gathers function information from symbol tables.
// ----------------------------------------------------------------------------
static void collect_names(void)
{   unsigned int len=0;
    unsigned int indx;

    // first generate function name table
    tcgfni=(tcgfname*)malloc(sizeof(tcgfname)*tcghdr.func_cnt);
    memset(tcgfni, 0x00, (sizeof(tcgfname)*tcghdr.func_cnt));
    if (tcgfni!=NULL)
    {   // resolved names
        for (indx=0;indx<symtab.len;indx++)
        {   tcgfni[indx].faddr=symtab.base[indx].addr;
            tcgfni[indx].fnoffset=len;
//            len+=strlen(symtab.base[indx].name)+1;
            len+=strlen(get_name(symtab.base[indx].name))+1;
            if (symtab.base[indx].file==NULL)
                tcgfni[indx].sfoffset=len-1;
            else
                len+=get_src_file(&symtab.base[indx],&tcgfni[indx],len);
            tcgfni[indx].lineno=symtab.base[indx].line_num;

        }

        // unresolved names
    }

    // then generate function name pool
    tcgfnp=(char*)malloc(len);
    memset(tcgfnp, 0x00, len);
    if (tcgfnp==NULL)
    {   // too bad
        free(tcgfni);
        len=0;
    } else
    {   unsigned int offset=0;

        // copy names
        for (indx=0;indx<symtab.len;indx++)
        {   strcpy(&tcgfnp[offset],get_name(symtab.base[indx].name));
//            offset+=strlen(symtab.base[indx].name)+1;
            offset+=strlen(get_name(symtab.base[indx].name))+1;
            if (offset==tcgfni[indx].sfoffset)
            {   strcpy(&tcgfnp[offset],symtab.base[indx].file->name);
                offset+=strlen(symtab.base[indx].file->name)+1;
            }
        }
    }
    tcghdr.pool_size=len;
} // collect_names

// ============================================================================
// prof_out_read
//
// This function reads the timeline based profiling data file.
// ============================================================================
int
prof_out_read(const char *filename)
{   FILE            *ifp;
    struct gmon_hdr ghdr;
    unsigned char   tag;
    int tl1 = 0, tl2 = 0, tl3 = 0, tl4 = 0, tl5 = 0, tl6 = 0, tl7 = 0, tl8 = 0, tl9 = 0;

    // file_format must be FF_PROF
    if (file_format != FF_PROF)
    {   fprintf(stderr, _("%s: don't know how to deal with file format %d\n"),
                whoami, file_format);
        done(1);
    }

    // open prof.out file
    if (strcmp(filename, "-") == 0)
    {   // it is from stdin
        ifp = stdin;
#ifdef SET_BINARY
        SET_BINARY(fileno(stdin));
#endif // SET_BINARY
    } else
    {   // use the specified file name
        ifp = fopen(filename, FOPEN_RB);
        if (!ifp)
        {   // failed to open it - nothing can be done
            perror(filename);
            done(1);
        }
    }

    // read the header which must exist
    if (fread(&ghdr, sizeof (struct gmon_hdr), 1, ifp) != 1)
    {   fprintf(stderr, _("%s: file too short to be a gmon file\n"),
	        filename);
        done(1);
    }


    // file must contain valid magic
    if (strncmp(&ghdr.cookie[0], GMON_MAGIC, 4))
    {   fprintf(stderr, _("%s: file `%s' has bad magic cookie\n"),
                whoami, filename);
        done(1);
    }

    // right magic, so it's probably really a new prof.out file.
    // make sure it is for this version
    gmon_file_version = *(int*)&ghdr.version;
    if (gmon_file_version != GMON_VERSION && gmon_file_version != 0)
    {   fprintf(stderr,_("%s: file `%s' has unsupported version %d\n"),
                whoami, filename, gmon_file_version);
        done(1);
    }else{
//    	tcghdr.version = gmon_file_version;
	tcghdr.version = 1;			// 2007,12,12, For AndeSight check, Jerry suggestion.
    }


    // read in total counts to intialize header record
    if (fread(&tcghdr.insn_cnt, sizeof(unsigned long long), 3, ifp) != 3)
    {   fprintf(stderr, _("%s: file too short to be a gmon file\n"),
	        filename);
        done(1);
    }


    if (do_timeline)
    {   // timeline also needs a temporary file
        sprintf(&prof_temp_file[5],"%05d",getpid());
        if ((temp_fd=fopen(prof_temp_file,"wb+"))==NULL)
        {   // cannot create temporary file
            fprintf(stderr, _("%s: failed to create temporary file %s?\n"),
                    whoami, prof_temp_file);
            done(1);
        }
	// Initialize tcghdr
	tcghdr.to_bb_cnt = 0;
	tcghdr.max_func_level = 0;
	tcghdr.min_func_level = 0;
	tcghdr.to_fn_call_tag = 0;
	tcghdr.to_fn_rt_tag = 0;
	tcghdr.to_br_tag = 0;
	tcghdr.to_du_call_tag = 0;
	tcghdr.to_du_rt_tag = 0;
    }
    symht_init(); // in case of unresolved


    // read in all the records
    if (fread(&tag, sizeof(tag), 1, ifp) == 1)
    {

	// first one must be valid
        do
        {   switch (tag)
            {   case GMON_TAG_TL_1:
                    ++tl1;
                    tcghdr.timeline_level = (unsigned int)(tag - 2);
                    gmon_input |= INPUT_TIMELINE1;
                    tag=tl1_read_rec(ifp, filename);
                    break;
                case GMON_TAG_TL_2:
                    ++tl2;
                    tcghdr.timeline_level = (unsigned int)(tag - 2);
                    gmon_input |= INPUT_TIMELINE2;
                    tag=tl2_read_rec(ifp, filename);
                    break;
                case GMON_TAG_TL_3:
                    ++tl3;
                    tcghdr.timeline_level = (unsigned int)(tag - 2);
                    gmon_input |= INPUT_TIMELINE3;
                    tag=tl3_read_rec(ifp, filename);
                    break;
		case GMON_TAG_TL_4:
                    fprintf(stderr,_("%s: %s: found unsupport level %d\n"),
                            whoami, filename, (GMON_TAG_TL_4 - 2));
		    break;
		case GMON_TAG_TL_5:
                    fprintf(stderr,_("%s: %s: found unsupport level %d\n"),
                            whoami, filename, (GMON_TAG_TL_5 - 2));
		    break;
		case GMON_TAG_TL_6:
		    ++tl6;
		    tcghdr.timeline_level = (unsigned int)(tag - 2);
		    gmon_input |= INPUT_TIMELINE6;
		    tag = tl6_read_rec(ifp, filename);
		    break;
		case GMON_TAG_TL_7:
		    ++tl7;
		    tcghdr.timeline_level = (unsigned int)(tag - 2);
		    gmon_input |= INPUT_TIMELINE7;
		    tag = tl7_read_rec(ifp, filename);
		    break;
		case GMON_TAG_TL_8:
		    ++tl8;
		    tcghdr.timeline_level = (unsigned int)(tag - 2);
		    gmon_input |= INPUT_TIMELINE8;
		    tag = tl8_read_rec(ifp, filename);
		    break;
		case GMON_TAG_TL_9:
		    ++tl9;
		    tcghdr.timeline_level = (unsigned int)(tag - 2);
		    gmon_input |= INPUT_TIMELINE9;
		    tag = tl9_read_rec(ifp, filename);
		    break;
		case 0xff:
			return -1;
			break;
        default:
            fprintf(stderr,_("%s: %s: found bad tag %d (file corrupted?)\n"),
                    whoami, filename, tag);
            done(1);
            }
        } while (tag>0); // we borrow 0x00 to singal end-of-file
    }

    // timeline also needs a temporary file
    if (do_timeline)
    {   fclose(temp_fd);

//	tcghdr.to_func_level = ((max_function_level - min_function_level) + 1);
		tcghdr.max_func_level = max_function_level;
		tcghdr.min_func_level = min_function_level;
        tcghdr.func_cnt=Sym_HTCount+symtab.len;
        collect_names();
    }

    if (output_style & STYLE_GMON_INFO)
    {   printf(_("File `%s' (version %d) contains:\n"),
               filename, gmon_file_version);
        printf(tl1 > 1 ?
               _("\t%d function coverage records\n") :
               _("\t%d function coverage records\n"), tl1);
        printf(tl2 > 1 ?
               _("\t%d branch coverage records\n") :
               _("\t%d branch coverage record\n"), tl2);
        printf(tl3 > 1 ?
               _("\t%d instruction coverage records\n") :
               _("\t%d instruction coverage record\n"), tl3);
        printf(tl6 > 1 ?
               _("\t%d function level + branch summary records\n") :
               _("\t%d function level + branch summary record\n"), tl6);
        printf(tl7 > 1 ?
               _("\t%d function level + cache summary records\n") :
               _("\t%d function level + cache summary record\n"), tl7);
        printf(tl8 > 1 ?
               _("\t%d function level + branch & cache summary records\n") :
               _("\t%d function level + branch & cache summary record\n"), tl8);
        printf(tl9 > 1 ?
               _("\t%d branch level + cache summary records\n") :
               _("\t%d branch level + cache summary record\n"), tl9);
        first_output = FALSE;
    }
	return 0;
} // prof_out_read
