#include <unordered_map>
#include <vector>
#include <string>
#include <sys/stat.h>
#include <assert.h>
#include <algorithm>

// #define USE_TIMER

// #define ZEROSPY_DEBUG
#define _WERROR

#ifdef TIMING
#include <time.h>
#include <math.h>
uint64_t get_miliseconds() {
    struct timespec spec;
    clock_gettime(CLOCK_REALTIME, &spec);
    return spec.tv_sec*1000 + round(spec.tv_nsec / 1.0e6); // Convert nanoseconds to milliseconds
}
#endif

#include "dr_api.h"
#include "drmgr.h"
#include "drreg.h"
#include "drutil.h"
#include "drcctlib.h"
#include "dr_tools.h"
#include <sys/time.h>
#include "utils.h"
#include "drx.h"

// #define USE_CLEANCALL
#define ENABLE_SAMPLING 1
#ifdef ENABLE_SAMPLING
// different frequency configurations:
//      Rate:   1/10, 1/100, 1/1000, 5/10, 5/100, 5/1000
//      Window: 1e6, 1e7, 1e8, 1e9, 1e10 
// #define WINDOW_ENABLE 1000000
// #define WINDOW_DISABLE 100000000
// #define WINDOW_CLEAN 10
// Enumuration of RATEs
#define RATE_NUM 6
#define RATE_0 (0.5)
#define RATE_1 (0.1)
#define RATE_2 (0.05)
#define RATE_3 (0.01)
#define RATE_4 (0.005)
#define RATE_5 (0.001)
// RATE -> WINDOW_ENABLE mapping
#define RATE_0_WIN(WINDOW) ((int64_t)(RATE_0*WINDOW))
#define RATE_1_WIN(WINDOW) ((int64_t)(RATE_1*WINDOW))
#define RATE_2_WIN(WINDOW) ((int64_t)(RATE_2*WINDOW))
#define RATE_3_WIN(WINDOW) ((int64_t)(RATE_3*WINDOW))
#define RATE_4_WIN(WINDOW) ((int64_t)(RATE_4*WINDOW))
#define RATE_5_WIN(WINDOW) ((int64_t)(RATE_5*WINDOW))
// Enumuration of WINDOWs
#define WINDOW_NUM 5
#define WINDOW_0 (100000)
#define WINDOW_1 (1000000)
#define WINDOW_2 (10000000)
#define WINDOW_3 (100000000)
#define WINDOW_4 (1000000000)

int window_enable;
int window_disable;
#endif

// Client Options
#include "droption.h"
static droption_t<bool> op_enable_sampling
(DROPTION_SCOPE_CLIENT, "enable_sampling", 0, 0, 64, "Enable Bursty Sampling",
 "Enable bursty sampling for lower overhead with less profiling accuracy.");

static droption_t<bool> op_no_flush
(DROPTION_SCOPE_CLIENT, "no_flush", 0, 0, 64, "Disable code flushing of Bursty Sampling",
 "Disable code flushing of Bursty Sampling.");

static droption_t<bool> op_help
(DROPTION_SCOPE_CLIENT, "help", 0, 0, 64, "Show this help",
 "Show this help.");

static droption_t<int> op_rate
(DROPTION_SCOPE_CLIENT, "rate", 3, 0, RATE_NUM, "Sampling rate configuration of bursty sampling",
 "Sampling rate configuration of bursty sampling. Only available when sampling is enabled."
 "Only the following configurations are valid:\n"
 "\t0: 0.5\n"
 "\t1: 0.1\n"
 "\t2: 0.05\n"
 "\t3: 0.01\n"
 "\t4: 0.005\n"
 "\t5: 0.001\n");

const float conf2rate[RATE_NUM] = {
    RATE_0,
    RATE_1,
    RATE_2,
    RATE_3,
    RATE_4,
    RATE_5
};

static droption_t<int> op_window
(DROPTION_SCOPE_CLIENT, "window", 3, 0, WINDOW_NUM, "Window size configuration of sampling",
 "Window size of sampling. Only available when sampling is enabled."
 "Only the following configurations are valid:\n"
 "\t0: 100000\n"
 "\t1: 1000000\n"
 "\t2: 10000000\n"
 "\t3: 100000000\n"
 "\t4: 1000000000\n");

const uint64_t conf2window[WINDOW_NUM] = {
    WINDOW_0,
    WINDOW_1,
    WINDOW_2,
    WINDOW_3,
    WINDOW_4
};

using namespace std;

#define ZEROSPY_PRINTF(format, args...) \
    DRCCTLIB_PRINTF_TEMPLATE("zerospy", format, ##args)
#define ZEROSPY_EXIT_PROCESS(format, args...)                                           \
    DRCCTLIB_CLIENT_EXIT_PROCESS_TEMPLATE("zerospy", format, \
                                          ##args)
#ifdef ARM_CCTLIB
#    define OPND_CREATE_CCT_INT OPND_CREATE_INT
#else
#    define OPND_CREATE_CCT_INT OPND_CREATE_INT32
#endif

#ifdef ARM_CCTLIB
#    define OPND_CREATE_IMMEDIATE_INT OPND_CREATE_INT
#else
#    ifdef CCTLIB_64
#        define OPND_CREATE_IMMEDIATE_INT OPND_CREATE_INT64
#    else
#        define OPND_CREATE_IMMEDIATE_INT OPND_CREATE_INT32
#    endif
#endif

// We only interest in memory loads
bool
zerospy_filter_read_mem_access_instr(instr_t *instr)
{
    return instr_reads_memory(instr);
}

#define ZEROSPY_FILTER_READ_MEM_ACCESS_INSTR zerospy_filter_read_mem_access_instr

static string g_folder_name;
static int tls_idx;

struct INTRedLogs{
    uint64_t tot;
    uint64_t red;
    uint64_t fred; // full redundancy
    uint64_t redByteMap;
};

struct FPRedLogs{
    uint64_t fred; // full redundancy
    uint64_t ftot;
    uint64_t redByteMap;
    uint8_t AccessLen;
    uint8_t size;
};

#define MINSERT instrlist_meta_preinsert
#define DECODE_DEAD(data) static_cast<uint8_t>(((data)  & 0xffffffffffffffff) >> 32 )
#define DECODE_KILL(data) (static_cast<context_handle_t>( (data)  & 0x00000000ffffffff))
#define MAKE_CONTEXT_PAIR(a, b) (((uint64_t)(a) << 32) | ((uint64_t)(b)))
#define delta 0.01
#define MAX_REDUNDANT_CONTEXTS_TO_LOG (1000)
// maximum cct depth to print
#define MAX_DEPTH 10

enum {
    INSTRACE_TLS_OFFS_BUF_PTR,
    INSTRACE_TLS_OFFS_VAL_CACHE_PTR,
    INSTRACE_TLS_COUNT, /* total number of TLS slots allocated */
};

static reg_id_t tls_seg;
static uint tls_offs;
#define TLS_SLOT(tls_base, enum_val) (void **)((byte *)(tls_base) + tls_offs + (enum_val))
#define BUF_PTR(tls_base, type, offs) *(type **)TLS_SLOT(tls_base, offs)

// 1M
#define MAX_CLONE_INS 1048576

// AVX512: 64B
#define MAX_VAL_LEN 64
// maximum cache size: 1M
#define MAX_CACHE_SIZE (1<<20)
struct val_cache_t {
    // isApprox (0,1) <7> | elementSize (1~64) <0-6> | accessLen | ctxt_hndl
    uint32_t ctxt_hndl;
    uint32_t info;
    // uint8_t vals[MAX_VAL_LEN];
    uint64_t redmap;
};

typedef struct _per_thread_t {
    unordered_map<uint64_t, INTRedLogs> *INTRedMap;
    unordered_map<uint64_t, FPRedLogs > *FPRedMap;
    file_t output_file;
#ifdef ENABLE_SAMPLING
    #ifdef USE_CLEANCALL
        long long numIns;
        bool sampleFlag;
    #else
        void* numInsBuff;
        int   cache_val_num;
        val_cache_t* cache_ptr;
    #endif
#endif
    vector<instr_t*> *instr_clones;
} per_thread_t;

#define ENCODE_CACHED_INFO(isApprox, elementSize, accessLen) ((((uint32_t)isApprox)<<31) | (((uint32_t)elementSize)<<24) | ((uint32_t)accessLen))
#define DECODE_ISAPPROX_FROM_CACHED_INFO(info) ((uint32_t)(info)>>31)
#define DECODE_ELEMSIZE_FROM_CACHED_INFO(info) (((uint32_t)(info)>>24)&0x7F)
#define DECODE_ACCESSLEN_FROM_CACHED_INFO(info) (((uint32_t)(info))&0x00FFFFFF)

#define IN
#define INOUT
#define OUT

file_t gFlagF;

// use manual inlined updates
#define RESERVE_AFLAGS(dc, bb, ins) assert(drreg_reserve_aflags (dc, bb, ins)==DRREG_SUCCESS)
#define UNRESERVE_AFLAGS(dc, bb, ins) assert(drreg_unreserve_aflags (dc, bb, ins)==DRREG_SUCCESS)

#define RESERVE_REG(dc, bb, instr, vec, reg) do {\
    if (drreg_reserve_register(dc, bb, instr, vec, &reg) != DRREG_SUCCESS) { \
        ZEROSPY_EXIT_PROCESS("ERROR @ %s:%d: drreg_reserve_register != DRREG_SUCCESS", __FILE__, __LINE__); \
    } } while(0)
#define UNRESERVE_REG(dc, bb, instr, reg) do { \
    if (drreg_unreserve_register(dc, bb, instr, reg) != DRREG_SUCCESS) { \
        ZEROSPY_EXIT_PROCESS("ERROR @ %s:%d: drreg_unreserve_register != DRREG_SUCCESS", __FILE__, __LINE__); \
    } } while(0)

// reg_ctxt and scratch can overlap (same register)
inline
void insertSaveCachedKey(void* drcontext, instrlist_t *ilist, instr_t *where, 
                         uint8_t elementSize, uint8_t accessLen, uint8_t isApprox, 
                         reg_id_t reg_base, reg_id_t reg_ctxt, reg_id_t scratch) {
    if(reg_is_64bit(scratch)) {
        scratch = reg_64_to_32(scratch);
    }
    if(reg_is_64bit(reg_ctxt)) {
        reg_ctxt = reg_64_to_32(reg_ctxt);
    }
    uint32_t info = ENCODE_CACHED_INFO(isApprox, elementSize, accessLen);
    MINSERT(ilist, where, XINST_CREATE_store(drcontext,
                    OPND_CREATE_MEM32(reg_base, offsetof(val_cache_t, ctxt_hndl)),
                    opnd_create_reg(reg_ctxt)));
    MINSERT(ilist, where, XINST_CREATE_load_int(drcontext, opnd_create_reg(scratch), OPND_CREATE_CCT_INT(info)));
    MINSERT(ilist, where, XINST_CREATE_store(drcontext,
                    OPND_CREATE_MEM32(reg_base, offsetof(val_cache_t, info)),
                    opnd_create_reg(scratch)));
}

// reg_addr and reg_val may overlapped
template<bool cache>
void insertLoadAndComputeRedMap_1byte(  void* drcontext, instrlist_t *ilist, instr_t *where, IN reg_id_t reg_base, 
                                        IN reg_id_t reg_addr, int offset, 
                                        OUT reg_id_t reg_val, OUT reg_id_t scratch) 
{
    return ;
    // printf("insertLoadAndComputeRedMap_1byte enter\n"); fflush(stdout);
    assert(reg_is_64bit(reg_val));
    MINSERT(ilist, where, XINST_CREATE_load_1byte(drcontext, 
                    opnd_create_reg(reg_32_to_8(reg_64_to_32(reg_val))),
                    OPND_CREATE_MEM8(reg_addr, offset)));
    // cmp based: 4 operation (cmp + cmov + cmov)
    // or based: 15 operation (7 or + 7 shr + 1 and)
    // So we use CMP based approach for 1 byte case
    // Compare value with 0
    MINSERT(ilist, where, XINST_CREATE_cmp(drcontext, opnd_create_reg(reg_val), OPND_CREATE_CCT_INT(0)));
    // Condition move to set redmap as 1 when ZF is set (val==0)
    MINSERT(ilist, where, XINST_CREATE_load_int(drcontext, opnd_create_reg(scratch), OPND_CREATE_CCT_INT(1)));
    MINSERT(ilist, where, INSTR_PRED(instr_create_1dst_1src((drcontext), (OP_cmovz), (opnd_create_reg(reg_val)), (opnd_create_reg(scratch))), DR_PRED_Z));
    // Condition move to set redmap as 0 when ZF is not set (val!=0)
    MINSERT(ilist, where, XINST_CREATE_load_int(drcontext, opnd_create_reg(scratch), OPND_CREATE_CCT_INT(0)));
    MINSERT(ilist, where, INSTR_PRED(instr_create_1dst_1src((drcontext), (OP_cmovnz), (opnd_create_reg(reg_val)), (opnd_create_reg(scratch))), DR_PRED_NZ));
    // Store back
    if(cache) {
        MINSERT(ilist, where, XINST_CREATE_store(drcontext,
                        OPND_CREATE_MEM64(reg_base, offsetof(val_cache_t, redmap)),
                        opnd_create_reg(reg_val)));
    }
    // printf("insertLoadAndComputeRedMap_1byte exit\n"); fflush(stdout);
}

template<bool cache>
void insertLoadAndComputeRedMap_2bytes( void* drcontext, instrlist_t *ilist, instr_t* where, IN reg_id_t reg_base, 
                                        IN reg_id_t reg_addr, int offset, 
                                        OUT reg_id_t reg_val, OUT reg_id_t scratch) 
{
    return ;
    // printf("insertLoadAndComputeRedMap_2bytes enter\n"); fflush(stdout);
    assert(reg_is_64bit(reg_val));
    MINSERT(ilist, where, XINST_CREATE_load_2bytes(drcontext, 
                    opnd_create_reg(reg_32_to_16(reg_64_to_32(reg_val))),
                    OPND_CREATE_MEM16(reg_addr, offset)));
    // or based: 21 ops = 3 mov + 18 calc (7 or + 7 shr + 1 and + 1 or + 1 shr + 1 and)
    // Merge all bits within a byte in parallel by binary merging: (7 + 7) => (3 * 3)
    // 0x
    MINSERT(ilist, where, XINST_CREATE_move(drcontext, opnd_create_reg(scratch), opnd_create_reg(reg_val)));
    MINSERT(ilist, where, XINST_CREATE_slr_s(drcontext, opnd_create_reg(scratch), OPND_CREATE_INT8(1)));
    MINSERT(ilist, where, INSTR_CREATE_or(drcontext, opnd_create_reg(reg_val), opnd_create_reg(scratch)));
    // 00xx
    MINSERT(ilist, where, XINST_CREATE_move(drcontext, opnd_create_reg(scratch), opnd_create_reg(reg_val)));
    MINSERT(ilist, where, XINST_CREATE_slr_s(drcontext, opnd_create_reg(scratch), OPND_CREATE_INT8(2)));
    MINSERT(ilist, where, INSTR_CREATE_or(drcontext, opnd_create_reg(reg_val), opnd_create_reg(scratch)));
    // 0000xxxx
    MINSERT(ilist, where, XINST_CREATE_move(drcontext, opnd_create_reg(scratch), opnd_create_reg(reg_val)));
    MINSERT(ilist, where, XINST_CREATE_slr_s(drcontext, opnd_create_reg(scratch), OPND_CREATE_INT8(4)));
    MINSERT(ilist, where, INSTR_CREATE_or(drcontext, opnd_create_reg(reg_val), opnd_create_reg(scratch)));
    // 1.1 x = (~x) & 0x101
    MINSERT(ilist, where, XINST_CREATE_load_int(drcontext, opnd_create_reg(scratch), OPND_CREATE_CCT_INT(0x101)));
    MINSERT(ilist, where, INSTR_CREATE_andn(drcontext, opnd_create_reg(reg_val), opnd_create_reg(reg_val), opnd_create_reg(scratch)));
    // x = (x>>7 | x)
    MINSERT(ilist, where, XINST_CREATE_move(drcontext, opnd_create_reg(scratch), opnd_create_reg(reg_val)));
    MINSERT(ilist, where, XINST_CREATE_slr_s(drcontext, opnd_create_reg(scratch), OPND_CREATE_INT8(7)));
    MINSERT(ilist, where, INSTR_CREATE_or(drcontext, opnd_create_reg(reg_val), opnd_create_reg(scratch)));
    // x = x&0x3
    MINSERT(ilist, where, INSTR_CREATE_and(drcontext, opnd_create_reg(reg_val), OPND_CREATE_INT8(0x3)));
    // Store back
    if(cache) {
        MINSERT(ilist, where, XINST_CREATE_store(drcontext,
                        OPND_CREATE_MEM64(reg_base, offsetof(val_cache_t, redmap)),
                        opnd_create_reg(reg_val)));
    }
    // printf("insertLoadAndComputeRedMap_2bytes exit\n"); fflush(stdout);
}
template<bool cache>
void insertLoadAndComputeRedMap_4bytes( void* drcontext, instrlist_t *ilist, instr_t *where, IN reg_id_t reg_base, 
                                        IN reg_id_t reg_addr, int offset, 
                                        OUT reg_id_t reg_val, OUT reg_id_t scratch) 
{
    // printf("insertLoadAndComputeRedMap_4bytes enter\n"); fflush(stdout);
    assert(reg_is_64bit(reg_val));
    MINSERT(ilist, where, XINST_CREATE_load(drcontext, 
                    opnd_create_reg(reg_64_to_32(reg_val)),
                    OPND_CREATE_MEM32(reg_addr, offset)));
    // or based: 22 ops = (7 or + 7 shr + 1 and + 3 or + 3 shr + 1 and)
    // Merge all bits within a byte in parallel by binary merging: (7 + 7) => (3 * 3)
    bool dead; assert(drreg_are_aflags_dead(drcontext, where, &dead)==DRREG_SUCCESS);
    dr_fprintf(gFlagF, "AFLAG is dead: %d (drreg) vs %d (drx): \n", dead, drx_aflags_are_dead(where));
    for(instr_t* inst=where; inst!=NULL; inst=instr_get_next(inst)) {
        dr_fprintf(gFlagF, "\t");
        disassemble(drcontext, instr_get_app_pc(inst), gFlagF);
    }
    dr_fprintf(gFlagF, "======================\n"); 
    assert(dead==drx_aflags_are_dead(where));
    // 0x
    MINSERT(ilist, where, XINST_CREATE_move(drcontext, opnd_create_reg(scratch), opnd_create_reg(reg_val)));
    MINSERT(ilist, where, XINST_CREATE_slr_s(drcontext, opnd_create_reg(scratch), OPND_CREATE_INT8(1)));
    MINSERT(ilist, where, INSTR_CREATE_or(drcontext, opnd_create_reg(reg_val), opnd_create_reg(scratch)));
    // 00xx
    MINSERT(ilist, where, XINST_CREATE_move(drcontext, opnd_create_reg(scratch), opnd_create_reg(reg_val)));
    MINSERT(ilist, where, XINST_CREATE_slr_s(drcontext, opnd_create_reg(scratch), OPND_CREATE_INT8(2)));
    MINSERT(ilist, where, INSTR_CREATE_or(drcontext, opnd_create_reg(reg_val), opnd_create_reg(scratch)));
    // 0000xxxx
    MINSERT(ilist, where, XINST_CREATE_move(drcontext, opnd_create_reg(scratch), opnd_create_reg(reg_val)));
    MINSERT(ilist, where, XINST_CREATE_slr_s(drcontext, opnd_create_reg(scratch), OPND_CREATE_INT8(4)));
    MINSERT(ilist, where, INSTR_CREATE_or(drcontext, opnd_create_reg(reg_val), opnd_create_reg(scratch)));
    // x = (~x) & 0x01010101
    MINSERT(ilist, where, XINST_CREATE_load_int(drcontext, opnd_create_reg(scratch), OPND_CREATE_CCT_INT(0x1010101)));
    MINSERT(ilist, where, INSTR_CREATE_andn(drcontext, opnd_create_reg(reg_val), opnd_create_reg(reg_val), opnd_create_reg(scratch)));
    // x = (x>>7 | x)
    MINSERT(ilist, where, XINST_CREATE_move(drcontext, opnd_create_reg(scratch), opnd_create_reg(reg_val)));
    MINSERT(ilist, where, XINST_CREATE_slr_s(drcontext, opnd_create_reg(scratch), OPND_CREATE_INT8(7)));
    MINSERT(ilist, where, INSTR_CREATE_or(drcontext, opnd_create_reg(reg_val), opnd_create_reg(scratch)));
    // x = (x>>14 | x)
    MINSERT(ilist, where, XINST_CREATE_move(drcontext, opnd_create_reg(scratch), opnd_create_reg(reg_val)));
    MINSERT(ilist, where, XINST_CREATE_slr_s(drcontext, opnd_create_reg(scratch), OPND_CREATE_INT8(14)));
    MINSERT(ilist, where, INSTR_CREATE_or(drcontext, opnd_create_reg(reg_val), opnd_create_reg(scratch)));
    // x = x&0xf
    MINSERT(ilist, where, INSTR_CREATE_and(drcontext, opnd_create_reg(reg_val), OPND_CREATE_INT8(0xf)));
    // Store back
    if(cache) {
        MINSERT(ilist, where, XINST_CREATE_store(drcontext,
                        OPND_CREATE_MEM64(reg_base, offsetof(val_cache_t, redmap)),
                        opnd_create_reg(reg_val)));
    }
    // printf("insertLoadAndComputeRedMap_4bytes exit\n"); fflush(stdout);
}

template<bool cache>
void insertLoadAndComputeRedMap_8bytes( void* drcontext, instrlist_t *ilist, instr_t *where, IN reg_id_t reg_base, 
                                        IN reg_id_t reg_addr, int offset, 
                                        OUT reg_id_t reg_val, OUT reg_id_t scratch) 
{
    return ;
    // printf("insertLoadAndComputeRedMap_8bytes enter\n"); fflush(stdout);
    assert(dr_get_isa_mode(drcontext)==DR_ISA_AMD64 && "insertLoadAndComputeRedMap_8bytes only supports X64 ISA!");
    if(reg_is_32bit(reg_val)) {
        reg_val = reg_32_to_64(reg_val);
        assert(0 && "REGISTER is 32 bit!");
    }
    MINSERT(ilist, where, XINST_CREATE_load(drcontext, 
                    opnd_create_reg(reg_val),
                    OPND_CREATE_MEM64(reg_addr, offset)));
    // or based: 22 ops = (7 or + 7 shr + 1 and + 3 or + 3 shr + 1 and)
    // Merge all bits within a byte in parallel by binary merging: (7 + 7) => (3 * 3)
    // 0x
    MINSERT(ilist, where, XINST_CREATE_move(drcontext, opnd_create_reg(scratch), opnd_create_reg(reg_val)));
    MINSERT(ilist, where, XINST_CREATE_slr_s(drcontext, opnd_create_reg(scratch), OPND_CREATE_INT8(1)));
    MINSERT(ilist, where, INSTR_CREATE_or(drcontext, opnd_create_reg(reg_val), opnd_create_reg(scratch)));
    // 00xx
    MINSERT(ilist, where, XINST_CREATE_move(drcontext, opnd_create_reg(scratch), opnd_create_reg(reg_val)));
    MINSERT(ilist, where, XINST_CREATE_slr_s(drcontext, opnd_create_reg(scratch), OPND_CREATE_INT8(2)));
    MINSERT(ilist, where, INSTR_CREATE_or(drcontext, opnd_create_reg(reg_val), opnd_create_reg(scratch)));
    // 0000xxxx
    MINSERT(ilist, where, XINST_CREATE_move(drcontext, opnd_create_reg(scratch), opnd_create_reg(reg_val)));
    MINSERT(ilist, where, XINST_CREATE_slr_s(drcontext, opnd_create_reg(scratch), OPND_CREATE_INT8(4)));
    MINSERT(ilist, where, INSTR_CREATE_or(drcontext, opnd_create_reg(reg_val), opnd_create_reg(scratch)));
    // x = (~x) & 0x0101010101010101LL
    MINSERT(ilist, where, INSTR_CREATE_mov_imm(drcontext, opnd_create_reg(scratch), OPND_CREATE_INT64(0x101010101010101LL)));
    MINSERT(ilist, where, INSTR_CREATE_andn(drcontext, opnd_create_reg(reg_val), opnd_create_reg(reg_val), opnd_create_reg(scratch)));
    // x = (x>>7 | x)
    MINSERT(ilist, where, XINST_CREATE_move(drcontext, opnd_create_reg(scratch), opnd_create_reg(reg_val)));
    MINSERT(ilist, where, XINST_CREATE_slr_s(drcontext, opnd_create_reg(scratch), OPND_CREATE_INT8(7)));
    MINSERT(ilist, where, INSTR_CREATE_or(drcontext, opnd_create_reg(reg_val), opnd_create_reg(scratch)));
    // x = (x>>14 | x)
    MINSERT(ilist, where, XINST_CREATE_move(drcontext, opnd_create_reg(scratch), opnd_create_reg(reg_val)));
    MINSERT(ilist, where, XINST_CREATE_slr_s(drcontext, opnd_create_reg(scratch), OPND_CREATE_INT8(14)));
    MINSERT(ilist, where, INSTR_CREATE_or(drcontext, opnd_create_reg(reg_val), opnd_create_reg(scratch)));
    // x = (x>>28 | x)
    MINSERT(ilist, where, XINST_CREATE_move(drcontext, opnd_create_reg(scratch), opnd_create_reg(reg_val)));
    MINSERT(ilist, where, XINST_CREATE_slr_s(drcontext, opnd_create_reg(scratch), OPND_CREATE_INT8(28)));
    MINSERT(ilist, where, INSTR_CREATE_or(drcontext, opnd_create_reg(reg_val), opnd_create_reg(scratch)));
    // x = x&0xff
    MINSERT(ilist, where, INSTR_CREATE_and(drcontext, opnd_create_reg(reg_val), OPND_CREATE_INT32(0xff)));
    // Store back
    if(cache) {
        // printf("Caching! 32bit: %d, 64bit: %d\n", reg_is_32bit(reg_val), reg_is_64bit(reg_val));
        MINSERT(ilist, where, XINST_CREATE_store(drcontext,
                        OPND_CREATE_MEM64(reg_base, offsetof(val_cache_t, redmap)),
                        opnd_create_reg(reg_val)));
    }
    // printf("insertLoadAndComputeRedMap_8bytes exit\n"); fflush(stdout);
}

#define SIMD_REG_NUM (DR_REG_XMM31-DR_REG_XMM0+1)

int simdAliveAnalysis(void* drcontext, instrlist_t *ilist, instr_t *where, bool simdIsDead[SIMD_REG_NUM]) {
    int deadNum = 0;
    // conservatively assume that all register is potentially alive
    memset(simdIsDead, 0, sizeof(bool)*SIMD_REG_NUM);
    bool simdUse[SIMD_REG_NUM] = {0};
    // forward to find if there are any dead registers (define before use)
    for(instr_t* instr = where; instr != NULL; instr = instr_get_next(instr)) {
        int num_srcs = instr_num_srcs(instr);
        for(int i=0; i<num_srcs; ++i) {
            opnd_t opnd = instr_get_src(instr, i);
            for(int j=0; j<opnd_num_regs_used(opnd); ++j) {
                reg_id_t regUse = opnd_get_reg_used(opnd, j);
                if(regUse >= DR_REG_XMM0 && regUse <= DR_REG_XMM31) {
                    int index = regUse - DR_REG_XMM0;
                    simdUse[index] = true;
                    // printf("SIMD REG USED: %s\n", get_register_name(regUse)); fflush(stdout);
                } else if(regUse >= DR_REG_YMM0 && regUse <= DR_REG_YMM31) {
                    int index = regUse - DR_REG_YMM0;
                    simdUse[index] = true;
                    // printf("SIMD REG USED: %s\n", get_register_name(regUse)); fflush(stdout);
                } else if(regUse >= DR_REG_ZMM0 && regUse <= DR_REG_ZMM31) {
                    int index = regUse - DR_REG_ZMM0;
                    simdUse[index] = true;
                    // printf("SIMD REG USED: %s\n", get_register_name(regUse)); fflush(stdout);
                }
            }
        }
        int num_dsts = instr_num_dsts(instr);
        for(int i=0; i<num_dsts; ++i) {
            opnd_t opnd = instr_get_dst(instr, i);
            for(int j=0; j<opnd_num_regs_used(opnd); ++j) {
                reg_id_t regDef = opnd_get_reg_used(opnd, j);
                if(regDef >= DR_REG_XMM0 && regDef <= DR_REG_XMM31) {
                    int index = regDef - DR_REG_XMM0;
                    if(!simdUse[index] && !simdIsDead[index]) {
                        simdIsDead[index] = true;
                        ++ deadNum;
                        // printf("SIMD REG DEFINED: %s\n", get_register_name(regDef)); fflush(stdout);
                    }
                } else if(regDef >= DR_REG_YMM0 && regDef <= DR_REG_YMM31) {
                    int index = regDef - DR_REG_YMM0;
                    if(!simdUse[index] && !simdIsDead[index]) {
                        simdIsDead[index] = true;
                        ++ deadNum;
                        // printf("SIMD REG DEFINED: %s\n", get_register_name(regDef)); fflush(stdout);
                    }
                } else if(regDef >= DR_REG_ZMM0 && regDef <= DR_REG_ZMM31) {
                    int index = regDef - DR_REG_ZMM0;
                    if(!simdUse[index] && !simdIsDead[index]) {
                        simdIsDead[index] = true;
                        ++ deadNum;
                        // printf("SIMD REG DEFINED: %s\n", get_register_name(regDef)); fflush(stdout);
                    }
                }
            }
        }
    }
    // Aggressive assumption: all not future-used SIMD is dead
    for(int i=0; i<SIMD_REG_NUM; ++i) {
        if(!simdUse[i] && !simdIsDead[i]) {
            simdIsDead[i] = true;
            ++ deadNum;
        }
    }
    return deadNum;
}

void reserve_simd_reg(void* drcontext, instrlist_t *ilist, instr_t *where, int size, bool simdIsDead[SIMD_REG_NUM], reg_id_t *reg) {
    reg_id_t reg_simd_start;
    switch(size) {
        case 16:
            reg_simd_start = DR_REG_XMM0;
            break;
        case 32:
            reg_simd_start = DR_REG_YMM0;
            break;
        case 64:
            reg_simd_start = DR_REG_ZMM0;
            break;
        default:
            assert(0 && "Unknown SIMD size!\n");
    }
    for(int i=0; i<SIMD_REG_NUM; ++i) {
        if(simdIsDead[i]) {
            *reg = reg_simd_start + i;
            simdIsDead[i] = false;
            // printf("Lazy Reserve Success\n"); fflush(stdout);
            return ;
        }
    }
    assert(0 && "ALL SIMD REG is alive! Spilling has not been implemented!");
}

void unreserve_simd_reg(void* drcontext, instrlist_t *ilist, instr_t* where, reg_id_t reg) {
    // TODO: if we spilled, restore the application value of the spilled SIMD register
}

uint8_t mask[64] __attribute__((aligned(64))) = {   0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 
                                                    0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
                                                    0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 
                                                    0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
                                                    0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 
                                                    0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
                                                    0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 
                                                    0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1 };

uint8_t mask_shuf[32] __attribute__((aligned(64))) = { 
                                         0x00, 0x08, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
                                         0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                         0x00, 0x08, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
                                         0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

// SIMD needed
template<int AccessLen>
void insertLoadAndComputeRedMap_simd(void* drcontext, instrlist_t *ilist, instr_t *where, reg_id_t reg_base, reg_id_t reg_addr, reg_id_t scratch) {
    return ;
    static_assert(AccessLen==16 || AccessLen==32 || AccessLen==64);
    // printf("insertLoadAndComputeRedMap_simd %d enter\n", AccessLen); fflush(stdout);
    // disassemble(drcontext, instr_get_app_pc(where), STDOUT);
    // fflush(stdout);
    // If we do not use SIMD vectorization, we need AccessLen/64 (2/4/8 for 128/256/512) * 22 ops (for 8bytes computation) to obtain redmap
    // Total: 28 Ops + (potentially SIMD register spill/restore ops)
    reg_id_t reg_val, reg_scratch;
    // Reserve 2 SIMD registers
    // Here we simply spill two SIMD registers
    // TODO: 2021/4/22: for better performance, try to track the latest drreg development to enable better lazy SIMD reservation.
    bool simdIsDead[SIMD_REG_NUM];
    int deadNum = simdAliveAnalysis(drcontext, ilist, where, simdIsDead);
    // assert(deadNum >= 2);
    reserve_simd_reg(drcontext, ilist, where, AccessLen, simdIsDead, &reg_val);
    reserve_simd_reg(drcontext, ilist, where, AccessLen, simdIsDead, &reg_scratch);
    // printf("RESERVED SIMD REG: %s\n", get_register_name(reg_val)); fflush(stdout);
    // printf("RESERVED SIMD REG: %s\n", get_register_name(reg_scratch)); fflush(stdout);
    // Load data from memory via SIMD instruction
    // TODO: use aligned memory read if possible
    switch(AccessLen) {
        case 16:
            MINSERT(ilist, where, INSTR_CREATE_vmovdqu(drcontext, opnd_create_reg(reg_val), 
                opnd_create_base_disp(reg_addr, DR_REG_NULL, 0, 0, OPSZ_16)));
            break;
        case 32:
            MINSERT(ilist, where, INSTR_CREATE_vmovdqu(drcontext, opnd_create_reg(reg_val), 
                opnd_create_base_disp(reg_addr, DR_REG_NULL, 0, 0, OPSZ_32)));
            break;
        case 64:
            MINSERT(ilist, where, INSTR_CREATE_vmovdqu(drcontext, opnd_create_reg(reg_val), 
                opnd_create_base_disp(reg_addr, DR_REG_NULL, 0, 0, OPSZ_64)));
            break;
        default:
            assert(0 && "Unknown AccessLen!");
    }
    MINSERT(ilist, where, INSTR_CREATE_vmovdqa(drcontext, opnd_create_reg(reg_scratch), opnd_create_reg(reg_val)));
    // Merge all bits within a byte in parallel
    // 0x
    if(AccessLen==16) {
        MINSERT(ilist, where, INSTR_CREATE_psrlq(drcontext, opnd_create_reg(reg_scratch), OPND_CREATE_INT8(1)));
    } else {
        MINSERT(ilist, where, INSTR_CREATE_vpsrlq(drcontext, opnd_create_reg(reg_scratch), OPND_CREATE_INT8(1), opnd_create_reg(reg_scratch)));
    }
    MINSERT(ilist, where, INSTR_CREATE_vpor(drcontext, opnd_create_reg(reg_val), opnd_create_reg(reg_val), opnd_create_reg(reg_scratch)));
    // 00xx
    if(AccessLen==16) {
        MINSERT(ilist, where, INSTR_CREATE_psrlq(drcontext, opnd_create_reg(reg_scratch), OPND_CREATE_INT8(2)));
    } else {
        MINSERT(ilist, where, INSTR_CREATE_vpsrlq(drcontext, opnd_create_reg(reg_scratch), OPND_CREATE_INT8(2), opnd_create_reg(reg_scratch)));
    }
    MINSERT(ilist, where, INSTR_CREATE_vpor(drcontext, opnd_create_reg(reg_val), opnd_create_reg(reg_val), opnd_create_reg(reg_scratch)));
    // 0000xxxx
    if(AccessLen==16) {
        MINSERT(ilist, where, INSTR_CREATE_psrlq(drcontext, opnd_create_reg(reg_scratch), OPND_CREATE_INT8(4)));
    } else {
        MINSERT(ilist, where, INSTR_CREATE_vpsrlq(drcontext, opnd_create_reg(reg_scratch), OPND_CREATE_INT8(4), opnd_create_reg(reg_scratch)));
    }
    MINSERT(ilist, where, INSTR_CREATE_vpor(drcontext, opnd_create_reg(reg_val), opnd_create_reg(reg_val), opnd_create_reg(reg_scratch)));
    // x = (~x) & broadcast(0x01)
    switch (AccessLen) {
        case 16: {
            // We directly load the simd mask from memory
            MINSERT(ilist, where, INSTR_CREATE_vmovdqa(drcontext, opnd_create_reg(reg_scratch), OPND_CREATE_ABSMEM(mask, OPSZ_16)));
            break ;
        }
        case 32: {
            // We directly load the simd mask from memory
            MINSERT(ilist, where, INSTR_CREATE_vmovdqa(drcontext, opnd_create_reg(reg_scratch), OPND_CREATE_ABSMEM(mask, OPSZ_32)));
            break ;
        }
        case 64: {
            // We directly load the simd mask from memory
            MINSERT(ilist, where, INSTR_CREATE_vmovdqa(drcontext, opnd_create_reg(reg_scratch), OPND_CREATE_ABSMEM(mask, OPSZ_64)));
            break ;
        }
    }
    MINSERT(ilist, where, INSTR_CREATE_vpandn(drcontext, opnd_create_reg(reg_val), opnd_create_reg(reg_val), opnd_create_reg(reg_scratch)));
    // Now SIMD reg_val contains the collected bitmap for each byte, we now narrow them into each 64-bit element in this packed SIMD register
    // x = (x>>7 | x)
    MINSERT(ilist, where, INSTR_CREATE_vmovdqa(drcontext, opnd_create_reg(reg_scratch), opnd_create_reg(reg_val)));
    if(AccessLen==16) {
        MINSERT(ilist, where, INSTR_CREATE_psrlq(drcontext, opnd_create_reg(reg_scratch), OPND_CREATE_INT8(7)));
    } else {
        MINSERT(ilist, where, INSTR_CREATE_vpsrlq(drcontext, opnd_create_reg(reg_scratch), OPND_CREATE_INT8(7), opnd_create_reg(reg_scratch)));
    }
    MINSERT(ilist, where, INSTR_CREATE_vpor(drcontext, opnd_create_reg(reg_val), opnd_create_reg(reg_val), opnd_create_reg(reg_scratch)));
    // x = (x>>14 | x)
    MINSERT(ilist, where, INSTR_CREATE_vmovdqa(drcontext, opnd_create_reg(reg_scratch), opnd_create_reg(reg_val)));
    if(AccessLen==16) {
        MINSERT(ilist, where, INSTR_CREATE_psrlq(drcontext, opnd_create_reg(reg_scratch), OPND_CREATE_INT8(14)));
    } else {
        MINSERT(ilist, where, INSTR_CREATE_vpsrlq(drcontext, opnd_create_reg(reg_scratch), OPND_CREATE_INT8(14), opnd_create_reg(reg_scratch)));
    }
    MINSERT(ilist, where, INSTR_CREATE_vpor(drcontext, opnd_create_reg(reg_val), opnd_create_reg(reg_val), opnd_create_reg(reg_scratch)));
    // x = (x>>28 | x)
    MINSERT(ilist, where, INSTR_CREATE_vmovdqa(drcontext, opnd_create_reg(reg_scratch), opnd_create_reg(reg_val)));
    if(AccessLen==16) {
        MINSERT(ilist, where, INSTR_CREATE_psrlq(drcontext, opnd_create_reg(reg_scratch), OPND_CREATE_INT8(28)));
    } else {
        MINSERT(ilist, where, INSTR_CREATE_vpsrlq(drcontext, opnd_create_reg(reg_scratch), OPND_CREATE_INT8(28), opnd_create_reg(reg_scratch)));
    }
    MINSERT(ilist, where, INSTR_CREATE_vpor(drcontext, opnd_create_reg(reg_val), opnd_create_reg(reg_val), opnd_create_reg(reg_scratch)));
    // After narrowed them by 64-bit elementwise merging, the lowest byte of each element contains the collected redmap, so we can now narrow
    // them by select (bytewise permutation).
    // x = permuteb(x, {0,8,...})
    switch (AccessLen) {
        case 16: {
            // shuffle: [...clear...] [72:64] [8:0]
            // We directly load the simd mask from memory
            MINSERT(ilist, where, INSTR_CREATE_vmovdqa(drcontext, opnd_create_reg(reg_scratch), OPND_CREATE_ABSMEM(mask_shuf, OPSZ_16)));
            MINSERT(ilist, where, INSTR_CREATE_vpshufb(drcontext, opnd_create_reg(reg_val), opnd_create_reg(reg_val), opnd_create_reg(reg_scratch)));
            // now store the lower 32-bits into target (INOUT) reg_addr register
            MINSERT(ilist, where, INSTR_CREATE_vmovq(drcontext, opnd_create_reg(reg_addr), opnd_create_reg(reg_val)));
            break;
        }
        case 32: {
            // shuffle: [...clear...] [200:192] [136:128] | [...clear...] [72:64] [8:0]
            // We directly load the simd mask from memory
            MINSERT(ilist, where, INSTR_CREATE_vmovdqa(drcontext, opnd_create_reg(reg_scratch), OPND_CREATE_ABSMEM(mask_shuf, OPSZ_32)));
            MINSERT(ilist, where, INSTR_CREATE_vpshufb(drcontext, opnd_create_reg(reg_val), opnd_create_reg(reg_val), opnd_create_reg(reg_scratch)));
            // As shuffle is performed per lane, so we need further merging
            // 1. permutation to merge two lanes into the first lane: 8 = (10) (00) -> [...] [192:128] [64:0]
            MINSERT(ilist, where, INSTR_CREATE_vpermpd(drcontext, opnd_create_reg(reg_val), opnd_create_reg(reg_val), OPND_CREATE_INT8(8)));
            // 2. shuffle again for narrowing into lower 64-bit value, here we reuse the previously loaded mask in simd scratch register
            MINSERT(ilist, where, INSTR_CREATE_vpshufb(drcontext, opnd_create_reg(reg_val), opnd_create_reg(reg_val), opnd_create_reg(reg_scratch)));
            // now store the lower 64-bits into target (INOUT) reg_addr register
            MINSERT(ilist, where, INSTR_CREATE_vmovq(drcontext, opnd_create_reg(reg_addr), opnd_create_reg(reg_val-DR_REG_YMM0+DR_REG_XMM0)));
            break;
        }
        case 64:
            assert(0 && "Currently we do not support inlined operation for AVX512! Use cleancall!\n");
            break;
        default:
            assert(0 && "Unknown AccessLen");
            break;
    }
    // Store back
    MINSERT(ilist, where, XINST_CREATE_store(drcontext,
                OPND_CREATE_MEM64(reg_base, offsetof(val_cache_t, redmap)),
                opnd_create_reg(reg_addr)));
    // Unreserve 2 SIMD registers
    unreserve_simd_reg(drcontext, ilist, where, reg_val);
    unreserve_simd_reg(drcontext, ilist, where, reg_scratch);
    // printf("insertLoadAndComputeRedMap_simd %d exit\n", AccessLen); fflush(stdout);
}

void insertComputeAndCacheRedmap_INT(void* drcontext, instrlist_t *ilist, instr_t *where, uint8_t accessLen,
                         INOUT reg_id_t reg_base, INOUT reg_id_t reg_addr, OUT reg_id_t scratch) {
    switch(accessLen) {
        case 1:
            // As we do not need reg_addr after loading values, we overlap the reg_addr and reg_val with the same register
            insertLoadAndComputeRedMap_1byte<true>(drcontext, ilist, where, reg_base, reg_addr, 0, reg_addr, scratch);
            break ;
        case 2:
            // As we do not need reg_addr after loading values, we overlap the reg_addr and reg_val with the same register
            insertLoadAndComputeRedMap_2bytes<true>(drcontext, ilist, where, reg_base, reg_addr, 0, reg_addr, scratch);
            break ;
        case 4:
            // As we do not need reg_addr after loading values, we overlap the reg_addr and reg_val with the same register
            insertLoadAndComputeRedMap_4bytes<true>(drcontext, ilist, where, reg_base, reg_addr, 0, reg_addr, scratch);
            break ;
        case 8:
            // As we do not need reg_addr after loading values, we overlap the reg_addr and reg_val with the same register
            insertLoadAndComputeRedMap_8bytes<true>(drcontext, ilist, where, reg_base, reg_addr, 0, reg_addr, scratch);
            break ;
        case 16:
            insertLoadAndComputeRedMap_simd<16>(drcontext, ilist, where, reg_base, reg_addr, scratch);
            break;
        case 32:
            insertLoadAndComputeRedMap_simd<32>(drcontext, ilist, where, reg_base, reg_addr, scratch);
            break;
        case 64:
            insertLoadAndComputeRedMap_simd<64>(drcontext, ilist, where, reg_base, reg_addr, scratch);
            break;
        default:
            {
                printf("insertComputeAndCacheRedmap_INT default: accessLen=%d enter\n", accessLen); fflush(stdout);
                int i=0;
                reg_id_t reg_val;
                reg_id_t reg_redmap;
                RESERVE_REG(drcontext, ilist, where, NULL, reg_val);
                RESERVE_REG(drcontext, ilist, where, NULL, reg_redmap);
                MINSERT(ilist, where, XINST_CREATE_load_int(drcontext, opnd_create_reg(reg_redmap), OPND_CREATE_CCT_INT(0)));
                while(i+8<=accessLen) {
                    insertLoadAndComputeRedMap_8bytes<false>(drcontext, ilist, where, reg_base, reg_addr, i, reg_val, scratch);
                    if(i!=0) {
                        MINSERT(ilist, where, XINST_CREATE_slr_s(drcontext, opnd_create_reg(reg_val), OPND_CREATE_INT8(i)));
                    }
                    MINSERT(ilist, where, INSTR_CREATE_or(drcontext, opnd_create_reg(reg_redmap), opnd_create_reg(reg_val)));
                    i+=8;
                }
                while(i+4<=accessLen) {
                    insertLoadAndComputeRedMap_4bytes<false>(drcontext, ilist, where, reg_base, reg_addr, i, reg_val, scratch);
                    if(i!=0) {
                        MINSERT(ilist, where, XINST_CREATE_slr_s(drcontext, opnd_create_reg(reg_val), OPND_CREATE_INT8(i)));
                    }
                    MINSERT(ilist, where, INSTR_CREATE_or(drcontext, opnd_create_reg(reg_redmap), opnd_create_reg(reg_val)));
                    i+=4;
                }
                while(i+2<=accessLen) {
                    insertLoadAndComputeRedMap_2bytes<false>(drcontext, ilist, where, reg_base, reg_addr, i, reg_val, scratch);
                    if(i!=0) {
                        MINSERT(ilist, where, XINST_CREATE_slr_s(drcontext, opnd_create_reg(reg_val), OPND_CREATE_INT8(i)));
                    }
                    MINSERT(ilist, where, INSTR_CREATE_or(drcontext, opnd_create_reg(reg_redmap), opnd_create_reg(reg_val)));
                    i+=2;
                }
                while(i+1<=accessLen) {
                    insertLoadAndComputeRedMap_1byte<false>(drcontext, ilist, where, reg_base, reg_addr, i, reg_val, scratch);
                    if(i!=0) {
                        MINSERT(ilist, where, XINST_CREATE_slr_s(drcontext, opnd_create_reg(reg_val), OPND_CREATE_INT8(i)));
                    }
                    MINSERT(ilist, where, INSTR_CREATE_or(drcontext, opnd_create_reg(reg_redmap), opnd_create_reg(reg_val)));
                    i++;
                }
                MINSERT(ilist, where, XINST_CREATE_store(drcontext,
                            OPND_CREATE_MEM64(reg_base, offsetof(val_cache_t, redmap)),
                            opnd_create_reg(reg_redmap)));
                UNRESERVE_REG(drcontext, ilist, where, reg_val);
                UNRESERVE_REG(drcontext, ilist, where, reg_redmap);
                printf("insertComputeAndCacheRedmap_INT default: accessLen=%d exit\n", accessLen); fflush(stdout);
            }
    }
    return ;
}

#ifdef USE_TIMER
bool global_sample_flag = true;
#define IS_SAMPLED(pt, a) (global_sample_flag)
#else
    #ifdef USE_CLEANCALL
    #define IS_SAMPLED(pt, WINDOW_ENABLE) (pt->sampleFlag)
    #else
    #define IS_SAMPLED(pt, WINDOW_ENABLE) ((int64_t)(BUF_PTR(pt->numInsBuff, void, INSTRACE_TLS_OFFS_BUF_PTR))<(int64_t)WINDOW_ENABLE)
    #endif
#endif
file_t gFile;
static void *gLock;
#ifndef _WERROR
file_t fwarn;
bool warned=false;
#endif
#ifdef ZEROSPY_DEBUG
file_t gDebug;
#endif

// global metrics
uint64_t grandTotBytesLoad = 0;
uint64_t grandTotBytesRedLoad = 0;
uint64_t grandTotBytesApproxRedLoad = 0;

/*******************************************************************************************/
static inline void AddToRedTable(uint64_t key,  uint16_t value, uint64_t byteMap, uint16_t total, per_thread_t *pt) __attribute__((always_inline,flatten));
static inline void AddToRedTable(uint64_t key,  uint16_t value, uint64_t byteMap, uint16_t total, per_thread_t *pt) {
    unordered_map<uint64_t, INTRedLogs>::iterator it = pt->INTRedMap->find(key);
    if ( it  == pt->INTRedMap->end()) {
        INTRedLogs log;
        log.red = value;
        log.tot = total;
        log.fred= (value==total);
        log.redByteMap = byteMap;
        (*pt->INTRedMap)[key] = log;
    } else {
        it->second.red += value;
        it->second.tot += total;
        it->second.fred+= (value==total);
        it->second.redByteMap &= byteMap;
    }
}

static inline void AddToApproximateRedTable(uint64_t key, uint64_t byteMap, uint16_t total, uint16_t zeros, uint16_t nums, uint8_t size, per_thread_t *pt) __attribute__((always_inline,flatten));
static inline void AddToApproximateRedTable(uint64_t key, uint64_t byteMap, uint16_t total, uint16_t zeros, uint16_t nums, uint8_t size, per_thread_t *pt) {
    unordered_map<uint64_t, FPRedLogs>::iterator it = pt->FPRedMap->find(key);
    if ( it  == pt->FPRedMap->end()) {
        FPRedLogs log;
        log.fred= zeros;
        log.ftot= nums;
        log.redByteMap = byteMap;
        log.AccessLen = total;
        log.size = size;
        (*pt->FPRedMap)[key] = log;
    } else {
        it->second.fred+= zeros;
        it->second.ftot+= nums;
        it->second.redByteMap &= byteMap;
    }
}
/*******************************************************************************************/
// single floating point zero byte counter
// 32-bit float: |sign|exp|mantissa| = | 1 | 8 | 23 |
// the redmap of single floating point takes up 5 bits (1 bit sign, 1 bit exp, 3 bit mantissa)
#define SP_MAP_SIZE 5
inline __attribute__((always_inline)) uint64_t count_zero_bytemap_fp(void * addr) {
    register uint32_t xx = *((uint32_t*)addr);
    // reduce by bits until byte level
    // | 0 | x0x0 x0x0 | 0x0 x0x0 x0x0 x0x0 x0x0 x0x0 |
    xx = xx | ((xx>>1)&0xffbfffff);
    // | x | 00xx 00xx | 0xx 00xx 00xx 00xx 00xx 00xx |
    xx = xx | ((xx>>2)&0xffdfffff);
    // | x | 0000 xxxx | 000 xxxx 0000 xxxx 0000 xxxx |
    xx = xx | ((xx>>4)&0xfff7ffff);
    // now xx is byte level reduced, check if it is zero and mask the unused bits
    xx = (~xx) & 0x80810101;
    // narrowing
    xx = xx | (xx>>7) | (xx>>14) | (xx>>20) | (xx>>27);
    xx = xx & 0x1f;
    return xx;
}
inline __attribute__((always_inline)) bool hasRedundancy_fp(void * addr) {
    register uint32_t xx = *((uint32_t*)addr);
    return (xx & 0x007f0000)==0;
}
/*******************************************************************************************/
// double floating point zero byte counter
// 64-bit float: |sign|exp|mantissa| = | 1 | 11 | 52 |
// the redmap of single floating point takes up 10 bits (1 bit sign, 2 bit exp, 7 bit mantissa)
#define DP_MAP_SIZE 10
inline __attribute__((always_inline)) uint64_t count_zero_bytemap_dp(void * addr) {
    register uint64_t xx = (static_cast<uint64_t*>(addr))[0];
    // reduce by bits until byte level
    // | 0 | 0x0 x0x0 x0x0 | x0x0 x0x0_x0x0 x0x0_x0x0 x0x0_x0x0 x0x0_x0x0 x0x0_x0x0 x0x0_x0x0 |
    xx = xx | ((xx>>1)&(~0x4008000000000000LL));
    // | x | 0xx 00xx 00xx | 00xx 00xx_00xx 00xx_00xx 00xx_00xx 00xx_00xx 00xx_00xx 00xx_00xx |
    xx = xx | ((xx>>2)&(~0x200c000000000000LL));
    // | x | xxx 0000 xxxx | xxxx 0000_xxxx 0000_xxxx 0000_xxxx 0000_xxxx 0000_xxxx 0000_xxxx |
    xx = xx | ((xx>>4)&(~0x100f000000000000LL));
    // now xx is byte level reduced, check if it is zero and mask the unused bits
    xx = (~xx) & 0x9011010101010101LL;
    // narrowing
    register uint64_t m = xx & 0x1010101010101LL;
    m = m | (m>>7);
    m = m | (m>>14);
    m = m | (m>>28);
    m = m & 0x7f;
    xx = xx | (xx>>9) | (xx>>7);
    xx = (xx >> 45) & 0x380;
    xx = m | xx;
    return xx;
}
inline __attribute__((always_inline)) bool hasRedundancy_dp(void * addr) {
    register uint64_t xx = *((uint64_t*)addr);
    return (xx & 0x000f000000000000LL)==0;
}
/***************************************************************************************/
/*********************** floating point full redundancy functions **********************/
/***************************************************************************************/

#if __BYTE_ORDER == __BIG_ENDIAN
typedef union {
  float f;
  struct {
    uint32_t sign : 1;
    uint32_t exponent : 8;
    uint32_t mantisa : 23;
  } parts;
  struct {
    uint32_t sign : 1;
    uint32_t value : 31;
  } vars;
} float_cast;

typedef union {
  double f;
  struct {
    uint64_t sign : 1;
    uint64_t exponent : 11;
    uint64_t mantisa : 52;
  } parts;
  struct {
    uint64_t sign : 1;
    uint64_t value : 63;
  } vars;
} double_cast;
#elif __BYTE_ORDER == __LITTLE_ENDIAN
typedef union {
  float f;
  struct {
    uint32_t mantisa : 23;
    uint32_t exponent : 8;
    uint32_t sign : 1;
  } parts;
  struct {
    uint32_t value : 31;
    uint32_t sign : 1;
  } vars;
} float_cast;

typedef union {
  double f;
  struct {
    uint64_t mantisa : 52;
    uint64_t exponent : 11;
    uint64_t sign : 1;
  } parts;
  struct {
    uint64_t value : 63;
    uint64_t sign : 1;
  } vars;
} double_cast;
#else
    #error Known Byte Order
#endif

template<int start, int end, int incr>
struct UnrolledConjunctionApprox{
    // if the mantisa is 0, the value of the double/float var must be 0
    static __attribute__((always_inline)) uint64_t BodyZeros(uint8_t* addr){
        if(incr==4)
            return ((*(reinterpret_cast<float_cast*>(&addr[start]))).vars.value==0) + (UnrolledConjunctionApprox<start+incr,end,incr>::BodyZeros(addr));
        else if(incr==8)
            return ((*(reinterpret_cast<double_cast*>(&addr[start]))).vars.value==0) + (UnrolledConjunctionApprox<start+incr,end,incr>::BodyZeros(addr));
        return 0;
    }
    static __attribute__((always_inline)) uint64_t BodyRedMap(uint8_t* addr){
        if(incr==4)
            return count_zero_bytemap_fp((void*)(addr+start)) | (UnrolledConjunctionApprox<start+incr,end,incr>::BodyRedMap(addr)<<SP_MAP_SIZE);
        else if(incr==8)
            return count_zero_bytemap_dp((void*)(addr+start)) | (UnrolledConjunctionApprox<start+incr,end,incr>::BodyRedMap(addr)<<DP_MAP_SIZE);
        else
            assert(0 && "Not Supportted floating size! now only support for FP32 or FP64.");
        return 0;
    }
    static __attribute__((always_inline)) uint64_t BodyHasRedundancy(uint8_t* addr){
        if(incr==4)
            return hasRedundancy_fp((void*)(addr+start)) || (UnrolledConjunctionApprox<start+incr,end,incr>::BodyHasRedundancy(addr));
        else if(incr==8)
            return hasRedundancy_dp((void*)(addr+start)) || (UnrolledConjunctionApprox<start+incr,end,incr>::BodyHasRedundancy(addr));
        else
            assert(0 && "Not Supportted floating size! now only support for FP32 or FP64.");
        return 0;
    }
};

template<int end,  int incr>
struct UnrolledConjunctionApprox<end , end , incr>{
    static __attribute__((always_inline)) uint64_t BodyZeros(uint8_t* addr){
        return 0;
    }
    static __attribute__((always_inline)) uint64_t BodyRedMap(uint8_t* addr){
        return 0;
    }
    static __attribute__((always_inline)) uint64_t BodyHasRedundancy(uint8_t* addr){
        return 0;
    }
};

/****************************************************************************************/
inline __attribute__((always_inline)) uint64_t count_zero_bytemap_int8(uint8_t * addr) {
    register uint8_t xx = *((uint8_t*)addr);
    // reduce by bits until byte level
    xx = xx | (xx>>1) | (xx>>2) | (xx>>3) | (xx>>4) | (xx>>5) | (xx>>6) | (xx>>7);
    // now xx is byte level reduced, check if it is zero and mask the unused bits
    xx = (~xx) & 0x1;
    return xx;
}
inline __attribute__((always_inline)) uint64_t count_zero_bytemap_int16(uint8_t * addr) {
    register uint16_t xx = *((uint16_t*)addr);
    // reduce by bits until byte level
    xx = xx | (xx>>1) | (xx>>2) | (xx>>3) | (xx>>4) | (xx>>5) | (xx>>6) | (xx>>7);
    // now xx is byte level reduced, check if it is zero and mask the unused bits
    xx = (~xx) & 0x101;
    // narrowing
    xx = xx | (xx>>7);
    xx = xx & 0x3;
    return xx;
}
inline __attribute__((always_inline)) uint64_t count_zero_bytemap_int32(uint8_t * addr) {
    register uint32_t xx = *((uint32_t*)addr);
    // reduce by bits until byte level
    xx = xx | (xx>>1) | (xx>>2) | (xx>>3) | (xx>>4) | (xx>>5) | (xx>>6) | (xx>>7);
    // now xx is byte level reduced, check if it is zero and mask the unused bits
    xx = (~xx) & 0x1010101;
    // narrowing
    xx = xx | (xx>>7);
    xx = xx | (xx>>14);
    xx = xx & 0xf;
    return xx;
}
inline __attribute__((always_inline)) uint64_t count_zero_bytemap_int64(uint8_t * addr) {
    register uint64_t xx = *((uint64_t*)addr);
    // reduce by bits until byte level
    xx = xx | (xx>>1) | (xx>>2) | (xx>>3) | (xx>>4) | (xx>>5) | (xx>>6) | (xx>>7);
    // now xx is byte level reduced, check if it is zero and mask the unused bits
    xx = (~xx) & 0x101010101010101LL;
    // narrowing
    xx = xx | (xx>>7);
    xx = xx | (xx>>14);
    xx = xx | (xx>>28);
    xx = xx & 0xff;
    return xx;
}

static const unsigned char BitCountTable4[] __attribute__ ((aligned(64))) = {
    0, 0, 1, 2
};

static const unsigned char BitCountTable8[] __attribute__ ((aligned(64))) = {
    0, 0, 0, 0, 1, 1, 2, 3
};

static const unsigned char BitCountTable16[] __attribute__ ((aligned(64))) = {
    0, 0, 0, 0, 0, 0, 0, 0,
    1, 1, 1, 1, 2, 2, 3, 4
};

static const unsigned char BitCountTable256[] __attribute__ ((aligned(64))) = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
    2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 
    2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 
    3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 
    4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 7, 8
};

// accessLen & eleSize are size in bits
template<uint32_t AccessLen, uint32_t EleSize>
struct RedMapString {
    static __attribute__((always_inline)) string getIntRedMapString(uint64_t redmap) {
        // static_assert(AccessLen % EleSize == 0);
        string buff = 
            RedMapString<AccessLen-EleSize, EleSize>::getIntRedMapString(redmap>>(AccessLen-EleSize)) + 
            " , " + RedMapString<EleSize, EleSize>::getIntRedMapString(redmap);
        return buff;
    }
};

template<uint32_t AccessLen>
struct RedMapString <AccessLen, AccessLen> {
    static __attribute__((always_inline)) string getIntRedMapString(uint64_t redmap) {
        string buff = "";
        buff += ((redmap>>(AccessLen-1))&0x1) ? "00 " : "XX ";
        buff += RedMapString<AccessLen-1, AccessLen-1>::getIntRedMapString(redmap>>1);
        return buff;
    }
};

template<>
struct RedMapString <1, 1> {
    static __attribute__((always_inline)) string getIntRedMapString(uint64_t redmap) {
        return string((redmap&0x1) ? "00" : "XX");
    }
};

template<uint32_t n_exp, uint32_t n_man>
inline __attribute__((always_inline)) string __getFpRedMapString(uint64_t redmap) {
    string buff = "";
    const uint32_t signPos = n_exp + n_man;
    buff += RedMapString<1,1>::getIntRedMapString(redmap>>signPos) + " | ";
    buff += RedMapString<n_exp,n_exp>::getIntRedMapString(redmap>>n_man) + " | ";
    buff += RedMapString<n_man,n_man>::getIntRedMapString(redmap);
    return buff;
}

template<uint32_t n_exp, uint32_t n_man>
string getFpRedMapString(uint64_t redmap, uint64_t accessLen) {
    string buff = "";
    uint64_t newAccessLen = accessLen - (n_exp + n_man + 1);
    if(newAccessLen==0) {
        return __getFpRedMapString<n_exp,n_man>(redmap);
    } else {
        return getFpRedMapString<n_exp,n_man>(redmap>>newAccessLen, newAccessLen) + " , " + __getFpRedMapString<n_exp,n_man>(redmap);
    }
    return buff;
}

#define getFpRedMapString_SP(redmap, num) getFpRedMapString<1,3>(redmap, num*5)
#define getFpRedMapString_DP(redmap, num) getFpRedMapString<2,7>(redmap, num*10)

template<int start, int end, int incr>
struct UnrolledConjunction{
    // if the mantisa is 0, the value of the double/float var must be 0
    static __attribute__((always_inline)) uint64_t BodyRedNum(uint64_t rmap){
        // static_assert(start < end);
        if(incr==1)
            return ((start==0) ? (rmap&0x1) : ((rmap>>start)&0x1)) + (UnrolledConjunction<start+incr,end,incr>::BodyRedNum(rmap));
        else if(incr==2)
            return ((start==0) ? BitCountTable8[rmap&0x3] : BitCountTable8[(rmap>>start)&0x3]) + (UnrolledConjunction<start+incr,end,incr>::BodyRedNum(rmap));
        else if(incr==4)
            return ((start==0) ? BitCountTable16[rmap&0xf] : BitCountTable16[(rmap>>start)&0xf]) + (UnrolledConjunction<start+incr,end,incr>::BodyRedNum(rmap));
        else if(incr==8)
            return ((start==0) ? BitCountTable256[rmap&0xff] : BitCountTable256[(rmap>>start)&0xff]) + (UnrolledConjunction<start+incr,end,incr>::BodyRedNum(rmap));
        return 0;
    }
    static __attribute__((always_inline)) uint64_t BodyRedMap(uint8_t* addr){
        // static_assert(start < end);
        if(incr==1)
            return count_zero_bytemap_int8(addr+start) | (UnrolledConjunction<start+incr,end,incr>::BodyRedMap(addr)<<1);
        else if(incr==2)
            return count_zero_bytemap_int16(addr+start) | (UnrolledConjunction<start+incr,end,incr>::BodyRedMap(addr)<<2);
        else if(incr==4)
            return count_zero_bytemap_int32(addr+start) | (UnrolledConjunction<start+incr,end,incr>::BodyRedMap(addr)<<4);
        else if(incr==8)
            return count_zero_bytemap_int64(addr+start) | (UnrolledConjunction<start+incr,end,incr>::BodyRedMap(addr)<<8);
        else
            assert(0 && "Not Supportted integer size! now only support for INT8, INT16, INT32 or INT64.");
        return 0;
    }
    static __attribute__((always_inline)) bool BodyHasRedundancy(uint8_t* addr){
        if(incr==1)
            return (addr[start]==0) || (UnrolledConjunction<start+incr,end,incr>::BodyHasRedundancy(addr));
        else if(incr==2)
            return (((*((uint16_t*)(&addr[start])))&0xff00)==0) || (UnrolledConjunction<start+incr,end,incr>::BodyRedMap(addr));
        else if(incr==4)
            return (((*((uint32_t*)(&addr[start])))&0xff000000)==0) || (UnrolledConjunction<start+incr,end,incr>::BodyRedMap(addr));
        else if(incr==8)
            return (((*((uint64_t*)(&addr[start])))&0xff00000000000000LL)==0) || (UnrolledConjunction<start+incr,end,incr>::BodyRedMap(addr));
        else
            assert(0 && "Not Supportted integer size! now only support for INT8, INT16, INT32 or INT64.");
        return 0;
    }
};

template<int end,  int incr>
struct UnrolledConjunction<end , end , incr>{
    static __attribute__((always_inline)) uint64_t BodyRedNum(uint64_t rmap){
        return 0;
    }
    static __attribute__((always_inline)) uint64_t BodyRedMap(uint8_t* addr){
        return 0;
    }
    static __attribute__((always_inline)) uint64_t BodyHasRedundancy(uint8_t* addr){
        return 0;
    }
};
/*******************************************************************************************/
#ifdef ENABLE_SAMPLING
#ifdef USE_CLEANCALL
uint64_t global_sample_mask = 0;
template<int64_t WINDOW_ENABLE, int64_t WINDOW_DISABLE, bool sampleFlag>
struct BBSample {
    static void update_per_bb(uint instruction_count, app_pc src)
    {
        void *drcontext = dr_get_current_drcontext();
        per_thread_t *pt = (per_thread_t *)drmgr_get_tls_field(drcontext, tls_idx);
        // if(drcctlib_get_thread_id()!=0)
        // printf("[THREAD %d] BB UPDATE %lld, ins count=%d, %d == %d!\n", drcctlib_get_thread_id(), pt->numIns, instruction_count, pt->sampleFlag, sampleFlag);
        pt->numIns += instruction_count;
        if(sampleFlag) {
            if(pt->numIns > WINDOW_ENABLE) {
                pt->sampleFlag = false;
            }
        } else {
            if(pt->numIns > WINDOW_DISABLE) {
                pt->sampleFlag = true;
                pt->numIns = 0;
            }
        }
        // If the sample flag is changed, flush the region and re-instrument
        if(pt->sampleFlag != sampleFlag) {
            // printf("[THREAD %d] BB FLUSH %d %lld, ins count=%d!\n", drcctlib_get_thread_id(), pt->sampleFlag, pt->numIns, instruction_count);
            // assert(pt->numIns==0 || (pt->numIns > WINDOW_ENABLE && pt->numIns <= WINDOW_ENABLE+instruction_count) );
            dr_mcontext_t mcontext;
            mcontext.size = sizeof(mcontext);
            mcontext.flags = DR_MC_ALL;
            // // flush all fragments in code cache
            // dr_flush_region(0, ~((ptr_uint_t)0));
            dr_flush_region(src, 1);
            dr_get_mcontext(drcontext, &mcontext);
            mcontext.pc = src;
            dr_redirect_execution(&mcontext);
        }
    }

    static void update_per_bb_shared(uint instruction_count, app_pc src)
    {
        void *drcontext = dr_get_current_drcontext();
        per_thread_t *pt = (per_thread_t *)drmgr_get_tls_field(drcontext, tls_idx);
        pt->numIns += instruction_count;
        assert("NOT IMPLEMENTED!");
        // if(sampleFlag) {
        //     if(pt->numIns > WINDOW_ENABLE) {
        //         pt->sampleFlag = false;
        //         uint64_t updateBit = 1 << (drcctlib_get_thread_id());
        //         uint64_t sampled = __sync_and_and_fetch(global_sample_mask, (~updateBit));
        //         if(!sampled) {
        //             assert(dr_unlink_flush_region(0, ~((ptr_uint_t)0)));
        //         }
        //     }
        // } else {
        //     pt->sampleFlag = (pt->numIns > WINDOW_DISABLE);
        // }
        // // If the sample flag is changed, flush the region and re-instrument
        // if(pt->sampleFlag != sampleFlag) {
        //     if(sampleFlag /*pt->sampleFlag==false*/) {
        //         pt->numIns = WINDOW_ENABLE;
        //     } else {
        //         pt->numIns = 0;
        //     }
        //     // assert(dr_unlink_flush_region(0, ~((ptr_uint_t)0)));
            
        // }
    }
};

void (*BBSampleTable[RATE_NUM][WINDOW_NUM][2])(uint, app_pc) = {
    {   {BBSample<RATE_0_WIN(WINDOW_0), WINDOW_0, false>::update_per_bb, BBSample<RATE_0_WIN(WINDOW_0), WINDOW_0, true>::update_per_bb},
        {BBSample<RATE_0_WIN(WINDOW_1), WINDOW_1, false>::update_per_bb, BBSample<RATE_0_WIN(WINDOW_1), WINDOW_1, true>::update_per_bb},
        {BBSample<RATE_0_WIN(WINDOW_2), WINDOW_2, false>::update_per_bb, BBSample<RATE_0_WIN(WINDOW_2), WINDOW_2, true>::update_per_bb},
        {BBSample<RATE_0_WIN(WINDOW_3), WINDOW_3, false>::update_per_bb, BBSample<RATE_0_WIN(WINDOW_3), WINDOW_3, true>::update_per_bb},
        {BBSample<RATE_0_WIN(WINDOW_4), WINDOW_4, false>::update_per_bb, BBSample<RATE_0_WIN(WINDOW_4), WINDOW_4, true>::update_per_bb},
    },
    {   {BBSample<RATE_1_WIN(WINDOW_0), WINDOW_0, false>::update_per_bb, BBSample<RATE_1_WIN(WINDOW_0), WINDOW_0, true>::update_per_bb},
        {BBSample<RATE_1_WIN(WINDOW_1), WINDOW_1, false>::update_per_bb, BBSample<RATE_1_WIN(WINDOW_1), WINDOW_1, true>::update_per_bb},
        {BBSample<RATE_1_WIN(WINDOW_2), WINDOW_2, false>::update_per_bb, BBSample<RATE_1_WIN(WINDOW_2), WINDOW_2, true>::update_per_bb},
        {BBSample<RATE_1_WIN(WINDOW_3), WINDOW_3, false>::update_per_bb, BBSample<RATE_1_WIN(WINDOW_3), WINDOW_3, true>::update_per_bb},
        {BBSample<RATE_1_WIN(WINDOW_4), WINDOW_4, false>::update_per_bb, BBSample<RATE_1_WIN(WINDOW_4), WINDOW_4, true>::update_per_bb},
    },
    {   {BBSample<RATE_2_WIN(WINDOW_0), WINDOW_0, false>::update_per_bb, BBSample<RATE_2_WIN(WINDOW_0), WINDOW_0, true>::update_per_bb},
        {BBSample<RATE_2_WIN(WINDOW_1), WINDOW_1, false>::update_per_bb, BBSample<RATE_2_WIN(WINDOW_1), WINDOW_1, true>::update_per_bb},
        {BBSample<RATE_2_WIN(WINDOW_2), WINDOW_2, false>::update_per_bb, BBSample<RATE_2_WIN(WINDOW_2), WINDOW_2, true>::update_per_bb},
        {BBSample<RATE_2_WIN(WINDOW_3), WINDOW_3, false>::update_per_bb, BBSample<RATE_2_WIN(WINDOW_3), WINDOW_3, true>::update_per_bb},
        {BBSample<RATE_2_WIN(WINDOW_4), WINDOW_4, false>::update_per_bb, BBSample<RATE_2_WIN(WINDOW_4), WINDOW_4, true>::update_per_bb},
    },
    {   {BBSample<RATE_3_WIN(WINDOW_0), WINDOW_0, false>::update_per_bb, BBSample<RATE_3_WIN(WINDOW_0), WINDOW_0, true>::update_per_bb},
        {BBSample<RATE_3_WIN(WINDOW_1), WINDOW_1, false>::update_per_bb, BBSample<RATE_3_WIN(WINDOW_1), WINDOW_1, true>::update_per_bb},
        {BBSample<RATE_3_WIN(WINDOW_2), WINDOW_2, false>::update_per_bb, BBSample<RATE_3_WIN(WINDOW_2), WINDOW_2, true>::update_per_bb},
        {BBSample<RATE_3_WIN(WINDOW_3), WINDOW_3, false>::update_per_bb, BBSample<RATE_3_WIN(WINDOW_3), WINDOW_3, true>::update_per_bb},
        {BBSample<RATE_3_WIN(WINDOW_4), WINDOW_4, false>::update_per_bb, BBSample<RATE_3_WIN(WINDOW_4), WINDOW_4, true>::update_per_bb},
    },
    {   {BBSample<RATE_4_WIN(WINDOW_0), WINDOW_0, false>::update_per_bb, BBSample<RATE_4_WIN(WINDOW_0), WINDOW_0, true>::update_per_bb},
        {BBSample<RATE_4_WIN(WINDOW_1), WINDOW_1, false>::update_per_bb, BBSample<RATE_4_WIN(WINDOW_1), WINDOW_1, true>::update_per_bb},
        {BBSample<RATE_4_WIN(WINDOW_2), WINDOW_2, false>::update_per_bb, BBSample<RATE_4_WIN(WINDOW_2), WINDOW_2, true>::update_per_bb},
        {BBSample<RATE_4_WIN(WINDOW_3), WINDOW_3, false>::update_per_bb, BBSample<RATE_4_WIN(WINDOW_3), WINDOW_3, true>::update_per_bb},
        {BBSample<RATE_4_WIN(WINDOW_4), WINDOW_4, false>::update_per_bb, BBSample<RATE_4_WIN(WINDOW_4), WINDOW_4, true>::update_per_bb},
    },
    {   {BBSample<RATE_5_WIN(WINDOW_0), WINDOW_0, false>::update_per_bb, BBSample<RATE_5_WIN(WINDOW_0), WINDOW_0, true>::update_per_bb},
        {BBSample<RATE_5_WIN(WINDOW_1), WINDOW_1, false>::update_per_bb, BBSample<RATE_5_WIN(WINDOW_1), WINDOW_1, true>::update_per_bb},
        {BBSample<RATE_5_WIN(WINDOW_2), WINDOW_2, false>::update_per_bb, BBSample<RATE_5_WIN(WINDOW_2), WINDOW_2, true>::update_per_bb},
        {BBSample<RATE_5_WIN(WINDOW_3), WINDOW_3, false>::update_per_bb, BBSample<RATE_5_WIN(WINDOW_3), WINDOW_3, true>::update_per_bb},
        {BBSample<RATE_5_WIN(WINDOW_4), WINDOW_4, false>::update_per_bb, BBSample<RATE_5_WIN(WINDOW_4), WINDOW_4, true>::update_per_bb},
    },
};

void (*BBSampleTableShared[RATE_NUM][WINDOW_NUM][2])(uint, app_pc) = {
    {   {BBSample<RATE_0_WIN(WINDOW_0), WINDOW_0, false>::update_per_bb_shared, BBSample<RATE_0_WIN(WINDOW_0), WINDOW_0, true>::update_per_bb_shared},
        {BBSample<RATE_0_WIN(WINDOW_1), WINDOW_1, false>::update_per_bb_shared, BBSample<RATE_0_WIN(WINDOW_1), WINDOW_1, true>::update_per_bb_shared},
        {BBSample<RATE_0_WIN(WINDOW_2), WINDOW_2, false>::update_per_bb_shared, BBSample<RATE_0_WIN(WINDOW_2), WINDOW_2, true>::update_per_bb_shared},
        {BBSample<RATE_0_WIN(WINDOW_3), WINDOW_3, false>::update_per_bb_shared, BBSample<RATE_0_WIN(WINDOW_3), WINDOW_3, true>::update_per_bb_shared},
        {BBSample<RATE_0_WIN(WINDOW_4), WINDOW_4, false>::update_per_bb_shared, BBSample<RATE_0_WIN(WINDOW_4), WINDOW_4, true>::update_per_bb_shared},
    },
    {   {BBSample<RATE_1_WIN(WINDOW_0), WINDOW_0, false>::update_per_bb_shared, BBSample<RATE_1_WIN(WINDOW_0), WINDOW_0, true>::update_per_bb_shared},
        {BBSample<RATE_1_WIN(WINDOW_1), WINDOW_1, false>::update_per_bb_shared, BBSample<RATE_1_WIN(WINDOW_1), WINDOW_1, true>::update_per_bb_shared},
        {BBSample<RATE_1_WIN(WINDOW_2), WINDOW_2, false>::update_per_bb_shared, BBSample<RATE_1_WIN(WINDOW_2), WINDOW_2, true>::update_per_bb_shared},
        {BBSample<RATE_1_WIN(WINDOW_3), WINDOW_3, false>::update_per_bb_shared, BBSample<RATE_1_WIN(WINDOW_3), WINDOW_3, true>::update_per_bb_shared},
        {BBSample<RATE_1_WIN(WINDOW_4), WINDOW_4, false>::update_per_bb_shared, BBSample<RATE_1_WIN(WINDOW_4), WINDOW_4, true>::update_per_bb_shared},
    },
    {   {BBSample<RATE_2_WIN(WINDOW_0), WINDOW_0, false>::update_per_bb_shared, BBSample<RATE_2_WIN(WINDOW_0), WINDOW_0, true>::update_per_bb_shared},
        {BBSample<RATE_2_WIN(WINDOW_1), WINDOW_1, false>::update_per_bb_shared, BBSample<RATE_2_WIN(WINDOW_1), WINDOW_1, true>::update_per_bb_shared},
        {BBSample<RATE_2_WIN(WINDOW_2), WINDOW_2, false>::update_per_bb_shared, BBSample<RATE_2_WIN(WINDOW_2), WINDOW_2, true>::update_per_bb_shared},
        {BBSample<RATE_2_WIN(WINDOW_3), WINDOW_3, false>::update_per_bb_shared, BBSample<RATE_2_WIN(WINDOW_3), WINDOW_3, true>::update_per_bb_shared},
        {BBSample<RATE_2_WIN(WINDOW_4), WINDOW_4, false>::update_per_bb_shared, BBSample<RATE_2_WIN(WINDOW_4), WINDOW_4, true>::update_per_bb_shared},
    },
    {   {BBSample<RATE_3_WIN(WINDOW_0), WINDOW_0, false>::update_per_bb_shared, BBSample<RATE_3_WIN(WINDOW_0), WINDOW_0, true>::update_per_bb_shared},
        {BBSample<RATE_3_WIN(WINDOW_1), WINDOW_1, false>::update_per_bb_shared, BBSample<RATE_3_WIN(WINDOW_1), WINDOW_1, true>::update_per_bb_shared},
        {BBSample<RATE_3_WIN(WINDOW_2), WINDOW_2, false>::update_per_bb_shared, BBSample<RATE_3_WIN(WINDOW_2), WINDOW_2, true>::update_per_bb_shared},
        {BBSample<RATE_3_WIN(WINDOW_3), WINDOW_3, false>::update_per_bb_shared, BBSample<RATE_3_WIN(WINDOW_3), WINDOW_3, true>::update_per_bb_shared},
        {BBSample<RATE_3_WIN(WINDOW_4), WINDOW_4, false>::update_per_bb_shared, BBSample<RATE_3_WIN(WINDOW_4), WINDOW_4, true>::update_per_bb_shared},
    },
    {   {BBSample<RATE_4_WIN(WINDOW_0), WINDOW_0, false>::update_per_bb_shared, BBSample<RATE_4_WIN(WINDOW_0), WINDOW_0, true>::update_per_bb_shared},
        {BBSample<RATE_4_WIN(WINDOW_1), WINDOW_1, false>::update_per_bb_shared, BBSample<RATE_4_WIN(WINDOW_1), WINDOW_1, true>::update_per_bb_shared},
        {BBSample<RATE_4_WIN(WINDOW_2), WINDOW_2, false>::update_per_bb_shared, BBSample<RATE_4_WIN(WINDOW_2), WINDOW_2, true>::update_per_bb_shared},
        {BBSample<RATE_4_WIN(WINDOW_3), WINDOW_3, false>::update_per_bb_shared, BBSample<RATE_4_WIN(WINDOW_3), WINDOW_3, true>::update_per_bb_shared},
        {BBSample<RATE_4_WIN(WINDOW_4), WINDOW_4, false>::update_per_bb_shared, BBSample<RATE_4_WIN(WINDOW_4), WINDOW_4, true>::update_per_bb_shared},
    },
    {   {BBSample<RATE_5_WIN(WINDOW_0), WINDOW_0, false>::update_per_bb_shared, BBSample<RATE_5_WIN(WINDOW_0), WINDOW_0, true>::update_per_bb_shared},
        {BBSample<RATE_5_WIN(WINDOW_1), WINDOW_1, false>::update_per_bb_shared, BBSample<RATE_5_WIN(WINDOW_1), WINDOW_1, true>::update_per_bb_shared},
        {BBSample<RATE_5_WIN(WINDOW_2), WINDOW_2, false>::update_per_bb_shared, BBSample<RATE_5_WIN(WINDOW_2), WINDOW_2, true>::update_per_bb_shared},
        {BBSample<RATE_5_WIN(WINDOW_3), WINDOW_3, false>::update_per_bb_shared, BBSample<RATE_5_WIN(WINDOW_3), WINDOW_3, true>::update_per_bb_shared},
        {BBSample<RATE_5_WIN(WINDOW_4), WINDOW_4, false>::update_per_bb_shared, BBSample<RATE_5_WIN(WINDOW_4), WINDOW_4, true>::update_per_bb_shared},
    },
};

void (*BBSample_target[2])(uint, app_pc);

template<int64_t WINDOW_ENABLE, int64_t WINDOW_DISABLE>
struct BBSampleNoFlush {
    static void update_per_bb(uint instruction_count)
    {
        void *drcontext = dr_get_current_drcontext();
        per_thread_t *pt = (per_thread_t *)drmgr_get_tls_field(drcontext, tls_idx);
        pt->numIns += instruction_count;
        if(pt->sampleFlag) {
            if(pt->numIns > WINDOW_ENABLE) {
                pt->sampleFlag = false;
            }
        } else {
            if(pt->numIns > WINDOW_DISABLE) {
                pt->sampleFlag = true;
                pt->numIns = 0;
            }
        }
    }
};

void (*BBSampleNoFlushTable[RATE_NUM][WINDOW_NUM])(uint) {
    {   BBSampleNoFlush<RATE_0_WIN(WINDOW_0), WINDOW_0>::update_per_bb,
        BBSampleNoFlush<RATE_0_WIN(WINDOW_1), WINDOW_1>::update_per_bb,
        BBSampleNoFlush<RATE_0_WIN(WINDOW_2), WINDOW_2>::update_per_bb,
        BBSampleNoFlush<RATE_0_WIN(WINDOW_3), WINDOW_3>::update_per_bb,
        BBSampleNoFlush<RATE_0_WIN(WINDOW_4), WINDOW_4>::update_per_bb,
    },
    {   BBSampleNoFlush<RATE_1_WIN(WINDOW_0), WINDOW_0>::update_per_bb,
        BBSampleNoFlush<RATE_1_WIN(WINDOW_1), WINDOW_1>::update_per_bb,
        BBSampleNoFlush<RATE_1_WIN(WINDOW_2), WINDOW_2>::update_per_bb,
        BBSampleNoFlush<RATE_1_WIN(WINDOW_3), WINDOW_3>::update_per_bb,
        BBSampleNoFlush<RATE_1_WIN(WINDOW_4), WINDOW_4>::update_per_bb,
    },
    {   BBSampleNoFlush<RATE_2_WIN(WINDOW_0), WINDOW_0>::update_per_bb,
        BBSampleNoFlush<RATE_2_WIN(WINDOW_1), WINDOW_1>::update_per_bb,
        BBSampleNoFlush<RATE_2_WIN(WINDOW_2), WINDOW_2>::update_per_bb,
        BBSampleNoFlush<RATE_2_WIN(WINDOW_3), WINDOW_3>::update_per_bb,
        BBSampleNoFlush<RATE_2_WIN(WINDOW_4), WINDOW_4>::update_per_bb,
    },
    {   BBSampleNoFlush<RATE_3_WIN(WINDOW_0), WINDOW_0>::update_per_bb,
        BBSampleNoFlush<RATE_3_WIN(WINDOW_1), WINDOW_1>::update_per_bb,
        BBSampleNoFlush<RATE_3_WIN(WINDOW_2), WINDOW_2>::update_per_bb,
        BBSampleNoFlush<RATE_3_WIN(WINDOW_3), WINDOW_3>::update_per_bb,
        BBSampleNoFlush<RATE_3_WIN(WINDOW_4), WINDOW_4>::update_per_bb,
    },
    {   BBSampleNoFlush<RATE_4_WIN(WINDOW_0), WINDOW_0>::update_per_bb,
        BBSampleNoFlush<RATE_4_WIN(WINDOW_1), WINDOW_1>::update_per_bb,
        BBSampleNoFlush<RATE_4_WIN(WINDOW_2), WINDOW_2>::update_per_bb,
        BBSampleNoFlush<RATE_4_WIN(WINDOW_3), WINDOW_3>::update_per_bb,
        BBSampleNoFlush<RATE_4_WIN(WINDOW_4), WINDOW_4>::update_per_bb,
    },
    {   BBSampleNoFlush<RATE_5_WIN(WINDOW_0), WINDOW_0>::update_per_bb,
        BBSampleNoFlush<RATE_5_WIN(WINDOW_1), WINDOW_1>::update_per_bb,
        BBSampleNoFlush<RATE_5_WIN(WINDOW_2), WINDOW_2>::update_per_bb,
        BBSampleNoFlush<RATE_5_WIN(WINDOW_3), WINDOW_3>::update_per_bb,
        BBSampleNoFlush<RATE_5_WIN(WINDOW_4), WINDOW_4>::update_per_bb,
    },
};
void (*BBSampleNoFlush_target)(uint);

#else

#    ifdef ARM_CCTLIB
#        define DRCCTLIB_LOAD_IMM32_0(dc, Rt, imm) \
            INSTR_CREATE_movz((dc), (Rt), (imm), OPND_CREATE_INT(0))
#        define DRCCTLIB_LOAD_IMM32_16(dc, Rt, imm) \
            INSTR_CREATE_movk((dc), (Rt), (imm), OPND_CREATE_INT(16))
#        define DRCCTLIB_LOAD_IMM32_32(dc, Rt, imm) \
            INSTR_CREATE_movk((dc), (Rt), (imm), OPND_CREATE_INT(32))
#        define DRCCTLIB_LOAD_IMM32_48(dc, Rt, imm) \
            INSTR_CREATE_movk((dc), (Rt), (imm), OPND_CREATE_INT(48))
static inline void
minstr_load_wint_to_reg(void *drcontext, instrlist_t *ilist, instr_t *where, reg_id_t reg,
                        int32_t wint_num)
{
    MINSERT(ilist, where,
            DRCCTLIB_LOAD_IMM32_0(drcontext, opnd_create_reg(reg),
                                  OPND_CREATE_IMMEDIATE_INT(wint_num & 0xffff)));
    wint_num = (wint_num >> 16) & 0xffff;
    if(wint_num) {
        MINSERT(ilist, where,
                DRCCTLIB_LOAD_IMM32_16(drcontext, opnd_create_reg(reg),
                                    OPND_CREATE_IMMEDIATE_INT(wint_num)));
    }
}

#ifdef ARM64_CCTLIB
static inline void
minstr_load_wwint_to_reg(void *drcontext, instrlist_t *ilist, instr_t *where,
                         reg_id_t reg, uint64_t wwint_num)
{
    MINSERT(ilist, where,
            DRCCTLIB_LOAD_IMM32_0(drcontext, opnd_create_reg(reg),
                                  OPND_CREATE_IMMEDIATE_INT(wwint_num & 0xffff)));
    uint64_t tmp = (wwint_num >> 16) & 0xffff;
    if(tmp) {
        MINSERT(ilist, where,
            DRCCTLIB_LOAD_IMM32_16(drcontext, opnd_create_reg(reg),
                                OPND_CREATE_IMMEDIATE_INT(tmp)));
    }
    tmp = (wwint_num >> 32) & 0xffff;
    if(tmp) {
        MINSERT(ilist, where,
            DRCCTLIB_LOAD_IMM32_32(drcontext, opnd_create_reg(reg),
                                OPND_CREATE_IMMEDIATE_INT(tmp)));
    }
    tmp = (wwint_num >> 48) & 0xffff;
    if(tmp) {
        MINSERT(ilist, where,
            DRCCTLIB_LOAD_IMM32_48(drcontext, opnd_create_reg(reg),
                                OPND_CREATE_IMMEDIATE_INT(tmp)));
    }
}
#endif
#    endif

uint64_t getRedZeroNum(int accessLen, uint64_t redbyteMap) {
    bool counting = true;
    register uint64_t redbytesNum = 0;
    int restLen = accessLen;
    while(counting && restLen>=8) {
        restLen -= 8;
        register uint8_t t = BitCountTable256[(redbyteMap>>restLen)&0xff];
        redbytesNum += t;
        if(t!=8) {
            counting = false;
            break;
        }
    }
    while(counting && restLen>=4) {
        restLen -= 4;
        register uint8_t t = BitCountTable16[(redbyteMap>>restLen)&0xf];
        redbytesNum += t;
        if(t!=4) {
            counting = false;
            break;
        }
    }
    while(counting && restLen>=2) {
        restLen -= 2;
        register uint8_t t = BitCountTable4[(redbyteMap>>restLen)&0x3];
        redbytesNum += t;
        if(t!=8) {
            counting = false;
            break;
        }
    }
    // dont check here as this loop must execute only once
    while(counting && restLen>=1) {
        restLen -= 1;
        register uint8_t t = (redbyteMap>>restLen)&0x1;
        redbytesNum += t;
    }
    return redbytesNum;
}

struct BBSampleInstrument {
    static void bb_flush_code(int memRefCnt) {
        void *drcontext = dr_get_current_drcontext();
        per_thread_t* pt = (per_thread_t *)drmgr_get_tls_field(drcontext, tls_idx);
        // If sampled, merge the metrics from the cached record
        int next_cache_val_num = pt->cache_val_num + memRefCnt;
        // printf("[Flush] memRefCnt=%d, current cache num: %d, next: %d\n", memRefCnt, pt->cache_val_num, next_cache_val_num);
        if(next_cache_val_num > MAX_CACHE_SIZE) {
            // flush_cache(pt);
// #if 0
            for(int i=0; i<pt->cache_val_num; ++i) {
                context_handle_t ctxt = pt->cache_ptr[i].ctxt_hndl;
                //assert(ctxt!=0);
                uint32_t info = pt->cache_ptr[i].info;
                uint64_t redmap = pt->cache_ptr[i].redmap;
                bool isApprox = DECODE_ISAPPROX_FROM_CACHED_INFO(info);
                uint elemSize = DECODE_ELEMSIZE_FROM_CACHED_INFO(info);
                uint accessLen= DECODE_ACCESSLEN_FROM_CACHED_INFO(info);
                assert(elemSize<=8 || elemSize==accessLen);
                int zeros = getRedZeroNum(accessLen, redmap);
                // printf("[Flush] isApprox=%d, elemSize=%d, accessLen=%d, zeros=%d, redmap=%lx\n", isApprox, elemSize, accessLen, zeros, redmap); fflush(stdout);
                if(isApprox) {
                    AddToApproximateRedTable((uint64_t)ctxt, redmap, accessLen, zeros, accessLen/elemSize, elemSize, pt);
                } else {
                    AddToRedTable((uint64_t)MAKE_CONTEXT_PAIR(accessLen, ctxt), zeros, redmap, accessLen, pt);
                }
            }
//             memset(pt->cache_ptr, 0, sizeof(val_cache_t)*MAX_CACHE_SIZE);
// #endif
            pt->cache_val_num = 0;
            BUF_PTR(pt->numInsBuff, void, INSTRACE_TLS_OFFS_VAL_CACHE_PTR) = pt->cache_ptr;
        } else {
            pt->cache_val_num = next_cache_val_num;
        }
    }

#ifdef ARM_CCTLIB
    static void insertBBUpdate(void* drcontext, instrlist_t *ilist, instr_t *where, int insCnt, reg_id_t reg_ptr, reg_id_t reg_val) {
        dr_insert_read_raw_tls(drcontext, ilist, where, tls_seg,
                            tls_offs + INSTRACE_TLS_OFFS_BUF_PTR, reg_ptr);
        MINSERT(ilist, where, XINST_CREATE_add(drcontext, opnd_create_reg(reg_ptr), OPND_CREATE_CCT_INT(insCnt)));
        // Clear insCnt when insCnt > WINDOW_DISABLE
        minstr_load_wint_to_reg(drcontext, ilist, where, reg_val, window_disable);
        MINSERT(ilist, where, XINST_CREATE_cmp(drcontext, opnd_create_reg(reg_ptr), opnd_create_reg(reg_val)));
        instr_t* skipclear = INSTR_CREATE_label(drcontext);
        MINSERT(ilist, where, XINST_CREATE_jump_cond(drcontext, DR_PRED_LE, opnd_create_instr(skipclear)));
        MINSERT(ilist, where, XINST_CREATE_load_int(drcontext, opnd_create_reg(reg_ptr), OPND_CREATE_CCT_INT(0)));
        MINSERT(ilist, where, skipclear);
        dr_insert_write_raw_tls(drcontext, ilist, where, tls_seg,
                            tls_offs + INSTRACE_TLS_OFFS_BUF_PTR, reg_ptr);
    }
    static void insertBBSample(void* drcontext, instrlist_t *ilist, instr_t *where, int insCnt) {
        static_assert(sizeof(void*)==8);
        per_thread_t *pt = (per_thread_t *)drmgr_get_tls_field(drcontext, tls_idx);
        bool af_dead=false, reg_dead=false; 
        reg_id_t reg_ptr;
        RESERVE_AFLAGS(drcontext, ilist, where);
        assert(drreg_reserve_register(drcontext, ilist, where, NULL, &reg_ptr)==DRREG_SUCCESS);
        reg_id_t reg_val;
        bool regVal_dead = false;
        assert(drreg_reserve_register(drcontext, ilist, where, NULL, &reg_val)==DRREG_SUCCESS);
        assert(drreg_is_register_dead(drcontext, reg_val, where, &regVal_dead)==DRREG_SUCCESS);
        assert(drreg_is_register_dead(drcontext, reg_ptr, where, &reg_dead)==DRREG_SUCCESS);

        insertBBUpdate(drcontext, ilist, where, insCnt, reg_ptr, reg_val);

        instr_t* restore = INSTR_CREATE_label(drcontext);
        // printf("%ld: %d\n", (int64_t)(BUF_PTR(pt->numInsBuff, void, INSTRACE_TLS_OFFS_BUF_PTR)), IS_SAMPLED(pt, window_enable));
        if(IS_SAMPLED(pt, window_enable)) {
            // fall to code flush when insCnt > WINDOW_ENABLE
            minstr_load_wint_to_reg(drcontext, ilist, where, reg_val, window_enable);
            MINSERT(ilist, where, XINST_CREATE_cmp(drcontext, opnd_create_reg(reg_ptr), opnd_create_reg(reg_val)));
            MINSERT(ilist, where, XINST_CREATE_jump_cond(drcontext, DR_PRED_LE, opnd_create_instr(restore)));
        } else {
            // fall to code flush when insCnt > WINDOW_DISABLE (check if cleared)
            MINSERT(ilist, where, XINST_CREATE_cmp(drcontext, opnd_create_reg(reg_ptr), OPND_CREATE_IMMEDIATE_INT(0)));
            MINSERT(ilist, where, XINST_CREATE_jump_cond(drcontext, DR_PRED_NE, opnd_create_instr(restore)));
        }
        if(!regVal_dead) assert(drreg_get_app_value(drcontext, ilist, where, reg_val, reg_val)==DRREG_SUCCESS);
        if(!reg_dead) assert(drreg_get_app_value(drcontext, ilist, where, reg_ptr, reg_ptr)==DRREG_SUCCESS);
        assert(drreg_restore_app_aflags(drcontext, ilist, where)==DRREG_SUCCESS);
        dr_insert_clean_call(drcontext, ilist, where, (void *)BBSampleInstrument::bb_flush_code, false, 1, OPND_CREATE_INTPTR(instr_get_app_pc(where)));
        MINSERT(ilist, where, restore);
        assert(drreg_unreserve_register(drcontext, ilist, where, reg_ptr)==DRREG_SUCCESS);
        assert(drreg_unreserve_register(drcontext, ilist, where, reg_val)==DRREG_SUCCESS);
        UNRESERVE_AFLAGS(drcontext, ilist, where);
    }

    static void insertBBSampleNoFlush(void* drcontext, instrlist_t *ilist, instr_t *where, int insCnt) {
        static_assert(sizeof(void*)==8);
        bool dead; 
        reg_id_t reg_ptr; 
        RESERVE_AFLAGS(drcontext, ilist, where);
        reg_id_t reg_val;
        bool regVal_dead = false;
        assert(drreg_reserve_register(drcontext, ilist, where, NULL, &reg_val)==DRREG_SUCCESS);
        assert(drreg_is_register_dead(drcontext, reg_val, where, &regVal_dead)==DRREG_SUCCESS);
        assert(drreg_reserve_register(drcontext, ilist, where, NULL, &reg_ptr)==DRREG_SUCCESS);

        insertBBUpdate(drcontext, ilist, where, insCnt, reg_ptr, reg_val);

        assert(drreg_unreserve_register(drcontext, ilist, where, reg_ptr)==DRREG_SUCCESS);
        assert(drreg_unreserve_register(drcontext, ilist, where, reg_val)==DRREG_SUCCESS);
        UNRESERVE_AFLAGS(drcontext, ilist, where);
    }
#else
    static void insertBBSample(void* drcontext, instrlist_t *ilist, instr_t *where, int insCnt, int memRefCnt) {
        static_assert(sizeof(void*)==8);
        per_thread_t *pt = (per_thread_t *)drmgr_get_tls_field(drcontext, tls_idx);
        reg_id_t reg_ptr;
        RESERVE_AFLAGS(drcontext, ilist, where);
        assert(drreg_reserve_register(drcontext, ilist, where, NULL, &reg_ptr)==DRREG_SUCCESS);
        dr_insert_read_raw_tls(drcontext, ilist, where, tls_seg,
                            tls_offs + INSTRACE_TLS_OFFS_BUF_PTR, reg_ptr);
        MINSERT(ilist, where, XINST_CREATE_add(drcontext, opnd_create_reg(reg_ptr), OPND_CREATE_CCT_INT(insCnt)));
        // Clear insCnt when insCnt > WINDOW_DISABLE
        MINSERT(ilist, where, XINST_CREATE_cmp(drcontext, opnd_create_reg(reg_ptr), OPND_CREATE_CCT_INT(window_disable)));
        instr_t* skipclear = INSTR_CREATE_label(drcontext);
        MINSERT(ilist, where, XINST_CREATE_jump_cond(drcontext, DR_PRED_LE, opnd_create_instr(skipclear)));
        MINSERT(ilist, where, INSTR_CREATE_xor(drcontext, opnd_create_reg(reg_ptr), opnd_create_reg(reg_ptr)));
        MINSERT(ilist, where, skipclear);
        dr_insert_write_raw_tls(drcontext, ilist, where, tls_seg,
                            tls_offs + INSTRACE_TLS_OFFS_BUF_PTR, reg_ptr);
        if(memRefCnt>0) {
            MINSERT(ilist, where, XINST_CREATE_cmp(drcontext, opnd_create_reg(reg_ptr), OPND_CREATE_CCT_INT(window_enable)));
            instr_t* skipflush = INSTR_CREATE_label(drcontext);
            MINSERT(ilist, where, XINST_CREATE_jump_cond(drcontext, DR_PRED_GT, opnd_create_instr(skipflush)));
            dr_insert_clean_call(drcontext, ilist, where, (void *)BBSampleInstrument::bb_flush_code, false, 1, OPND_CREATE_CCT_INT(memRefCnt));
            MINSERT(ilist, where, skipflush);
        }
        assert(drreg_unreserve_register(drcontext, ilist, where, reg_ptr)==DRREG_SUCCESS);
        UNRESERVE_AFLAGS(drcontext, ilist, where);
    }

    static void insertBBSampleNoFlush(void* drcontext, instrlist_t *ilist, instr_t *where, int insCnt) {
        static_assert(sizeof(void*)==8);
        reg_id_t reg_ptr; 
        RESERVE_AFLAGS(drcontext, ilist, where);
        assert(drreg_reserve_register(drcontext, ilist, where, NULL, &reg_ptr)==DRREG_SUCCESS);
        dr_insert_read_raw_tls(drcontext, ilist, where, tls_seg,
                            tls_offs + INSTRACE_TLS_OFFS_BUF_PTR, reg_ptr);
        MINSERT(ilist, where, XINST_CREATE_add(drcontext, opnd_create_reg(reg_ptr), OPND_CREATE_CCT_INT(insCnt)));
        MINSERT(ilist, where, XINST_CREATE_cmp(drcontext, opnd_create_reg(reg_ptr), OPND_CREATE_CCT_INT(window_disable)));
        instr_t* skipclear = INSTR_CREATE_label(drcontext);
        MINSERT(ilist, where, XINST_CREATE_jump_cond(drcontext, DR_PRED_LE, opnd_create_instr(skipclear)));
        MINSERT(ilist, where, INSTR_CREATE_xor(drcontext, opnd_create_reg(reg_ptr), opnd_create_reg(reg_ptr)));
        MINSERT(ilist, where, skipclear);
        dr_insert_write_raw_tls(drcontext, ilist, where, tls_seg,
                            tls_offs + INSTRACE_TLS_OFFS_BUF_PTR, reg_ptr);
        assert(drreg_unreserve_register(drcontext, ilist, where, reg_ptr)==DRREG_SUCCESS);
        UNRESERVE_AFLAGS(drcontext, ilist, where);
    }
#endif
};
// endif USE_CLEANCALL
#endif

bool marked = false;

#ifdef USE_TIMER
void callback(int flush_id) {
    fprintf(stderr, "[Thread %d] Code Cache Flushed!\n", drcctlib_get_thread_id()); fflush(stderr);
}

// static dr_signal_action_t
// code_flush_callback(void *drcontext, dr_siginfo_t *siginfo)
// {
//     if(siginfo->sig == SIGUSR1) {
//         fprintf(stderr, "[Thread %d] SIGUSR1 Detected! Flush Code Cache!\n", drcctlib_get_thread_id()); fflush(stderr);
//         dr_flush_region(0, ~((ptr_uint_t)0));
//         return DR_SIGNAL_REDIRECT;
//     }
//     return DR_SIGNAL_DELIVER;
// }

int num;

void handler(void *drcontext, dr_mcontext_t *mcontext)
{
    global_sample_flag = !global_sample_flag;
    fprintf(stderr,"[Thread %d] Timer triggered! Sample=%d\n", drcctlib_get_thread_id(), global_sample_flag); fflush(stderr);
    if(global_sample_flag) {
        if(!dr_set_itimer(ITIMER_REAL, 1, handler)) {
            ZEROSPY_EXIT_PROCESS("HANDLER: DR_SET_ITIMER  ENABLE FAILED!\n");
        }
    } else {
        if(!dr_set_itimer(ITIMER_REAL, 999, handler)) {
            ZEROSPY_EXIT_PROCESS("HANDLER: DR_SET_ITIMER DISABLE FAILED!\n");
        }
    }
    marked = false;
//    fprintf(stderr,"[Thread %d] Timer triggered: Sending flush region request...\n", drcctlib_get_thread_id()); fflush(stderr);
//    dr_delay_flush_region(0, ~((ptr_uint_t)0), 0, callback);
//    fprintf(stderr,"[Thread %d] Timer triggered: flush region request send!\n", drcctlib_get_thread_id()); fflush(stderr);
}

template<bool ref_sampleFlag>
void bb_flush_code(app_pc src) {
    if(global_sample_flag!=ref_sampleFlag) {
        if(!marked) {
            marked = true;
            dr_unlink_flush_region(0, ~((ptr_uint_t)0));
        }
        // // Only the thread successfully get the lock will flush the code cache to avoid data race
        // bool willFlush = dr_mutex_trylock(gLock);
        // // printf("[THREAD %d] WILL FLUSH? %s(%d)\n", drcctlib_get_thread_id(), willFlush?"YES":"NO", willFlush); fflush(stdout);
        // if(willFlush) {
        //     if(!isFlushing) {
        //         isFlushing = true;
        //         dr_mutex_unlock(gLock);
        //         printf("[THREAD %d] FLUSH!\n", drcctlib_get_thread_id()); fflush(stdout);
        //         // flush all fragments in code cache
        //         dr_flush_region(0, ~((ptr_uint_t)0));
        //         // dr_unlink_flush_region(0, ~((ptr_uint_t)0));
        //         // dr_flush_region(src, 1);
        //         dr_mutex_lock(gLock);
        //         isFlushing = false;
        //         dr_mutex_unlock(gLock);
        //         printf("[THREAD %d] FLUSHED!\n", drcctlib_get_thread_id()); fflush(stdout);
        //         // if(global_sample_flag) {
        //         //     if(!dr_set_itimer(ITIMER_REAL, 10, handler)) {
        //         //         ZEROSPY_EXIT_PROCESS("HANDLER: DR_SET_ITIMER  ENABLE FAILED!\n");
        //         //     }
        //         // } else {
        //         //     if(!dr_set_itimer(ITIMER_REAL, 990, handler)) {
        //         //         ZEROSPY_EXIT_PROCESS("HANDLER: DR_SET_ITIMER DISABLE FAILED!\n");
        //         //     }
        //         // }
        //         // dr_mcontext_t mcontext;
        //         // mcontext.size = sizeof(mcontext);
        //         // mcontext.flags = DR_MC_ALL;
        //         // dr_get_mcontext(drcontext, &mcontext);
        //         // mcontext.pc = src;
        //         // dr_redirect_execution(&mcontext);
        //     } else {
        //         dr_mutex_unlock(gLock);
        //     }
        // }
    }
}
#endif

static dr_emit_flags_t
event_basic_block(void *drcontext, void *tag, instrlist_t *bb, bool for_trace, bool translating, OUT void **user_data)
{
    int num_instructions = 0;
    int memRefCnt = 0;
    instr_t *instr;
    /* count the number of instructions in this block */
    for (instr = instrlist_first(bb); instr != NULL; instr = instr_get_next(instr)) {
        num_instructions++;
        if(    (!op_no_flush.get_value()) 
            &&   instr_reads_memory(instr) 
            && (!instr_is_prefetch(instr)) 
            && (!instr_is_ignorable(instr)) 
            && (!instr_is_gather(instr))) 
        {
            int num_srcs = instr_num_srcs(instr);
            for(int i=0; i<num_srcs; ++i) {
                opnd_t opnd = instr_get_src(instr, i);
                if(opnd_is_memory_reference(opnd)) {
                    // TODO: support floating point caching
                    if(!instr_is_floating(instr)) {
                        memRefCnt++;
                    }
                }
            }
        }
    }
    assert(num_instructions>=0);
    if(num_instructions==0) {
        return DR_EMIT_DEFAULT;
    }
    /* insert clean call */
#ifdef USE_CLEANCALL
    if(op_no_flush.get_value()) {
        dr_insert_clean_call(drcontext, bb, instrlist_first(bb), (void *) BBSampleNoFlush_target, false, 1, 
                            OPND_CREATE_INT32(num_instructions));
    } else {
        per_thread_t *pt = (per_thread_t *)drmgr_get_tls_field(drcontext, tls_idx);
        if(pt->sampleFlag) {
            dr_insert_clean_call(drcontext, bb, instrlist_first(bb), (void *) BBSample_target[1], false, 2, 
                            OPND_CREATE_INT32(num_instructions), 
                            OPND_CREATE_INTPTR(instr_get_app_pc(instrlist_first(bb))));
        } else {
            dr_insert_clean_call(drcontext, bb, instrlist_first(bb), (void *) BBSample_target[0], false, 2, 
                            OPND_CREATE_INT32(num_instructions), 
                            OPND_CREATE_INTPTR(instr_get_app_pc(instrlist_first(bb))));
        }
    }
#else
    if(op_enable_sampling.get_value()) {
        if(op_no_flush.get_value()) {
            BBSampleInstrument::insertBBSampleNoFlush(drcontext, bb, instrlist_first(bb), num_instructions);
        } else {
            BBSampleInstrument::insertBBSample(drcontext, bb, instrlist_first(bb), num_instructions, memRefCnt);
        }
    } else {
        if(memRefCnt) {
            dr_insert_clean_call(drcontext, bb, instrlist_first(bb), (void *)BBSampleInstrument::bb_flush_code, false, 1, OPND_CREATE_CCT_INT(memRefCnt));
        }
    }
    
#endif
    return DR_EMIT_DEFAULT;
}
#endif

template<class T, uint32_t AccessLen, uint32_t ElemLen, bool isApprox, bool enable_sampling>
struct ZeroSpyAnalysis{
#ifdef ZEROSPY_DEBUG
    static __attribute__((always_inline)) void CheckNByteValueAfterRead(int32_t slot, void* addr, instr_t* instr)
#else
    static __attribute__((always_inline)) void CheckNByteValueAfterRead(int32_t slot, void* addr)
#endif
    {
        void *drcontext = dr_get_current_drcontext();
        per_thread_t *pt = (per_thread_t *)drmgr_get_tls_field(drcontext, tls_idx);
#ifdef ENABLE_SAMPLING
#ifdef USE_CLEANCALL
        if(enable_sampling) {
            if(!pt->sampleFlag) {
                return ;
            }
        }
#endif
#endif
// #ifdef ZEROSPY_DEBUG
//         byte* base;
//         size_t size;
//         uint prot; 
//         bool printDisassemble = false;
//         if(!dr_query_memory((byte*)addr, &base, &size, &prot)) {
//             dr_fprintf(STDERR, "\nWARNING: dr_query_memory failed!\n");
//             printDisassemble = true;
//         } else if((prot&DR_MEMPROT_READ)==0) {
//             dr_fprintf(STDERR, "\nWARNING: memory address %p is protected and cannot read!\n", addr);
//             printDisassemble = true;
//         }
//         if(printDisassemble) {
//             dr_fprintf(STDERR, "^^ Disassembled Instruction instr=%p, PC=%p ^^^\n", instr, instr_get_app_pc(instr));
//             dr_fprintf(STDERR, "addr=%p, AccessLen=%d, ElemLen=%d, isApprox=%d\n", addr, AccessLen, ElemLen, isApprox);
//             disassemble(drcontext, instr_get_app_pc(instr), STDERR);
//             dr_fprintf(STDERR, "ADDR[0]: %d\n", *((uint8_t*)addr));
//             dr_fprintf(STDERR, "ADDR[%d]: %d\n", AccessLen-1, *((uint8_t*)addr+AccessLen-1));
//             dr_fprintf(STDERR, "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
//         }
// #endif
        context_handle_t curCtxtHandle = drcctlib_get_context_handle(drcontext, slot);
        uint8_t* bytes = static_cast<uint8_t*>(addr);
        if(isApprox) {
            bool hasRedundancy = UnrolledConjunctionApprox<0,AccessLen,sizeof(T)>::BodyHasRedundancy(bytes);
            if(hasRedundancy) {
                uint64_t map = UnrolledConjunctionApprox<0,AccessLen,sizeof(T)>::BodyRedMap(bytes);
                uint32_t zeros = UnrolledConjunctionApprox<0,AccessLen,sizeof(T)>::BodyZeros(bytes);        
                AddToApproximateRedTable((uint64_t)curCtxtHandle, map, AccessLen, zeros, AccessLen/sizeof(T), sizeof(T), pt);
            } else {
                AddToApproximateRedTable((uint64_t)curCtxtHandle, 0, AccessLen, 0, AccessLen/sizeof(T), sizeof(T), pt);
            }
        } else {
            bool hasRedundancy = UnrolledConjunction<0,AccessLen,sizeof(T)>::BodyHasRedundancy(bytes);
            if(hasRedundancy) {
                uint64_t redbyteMap = UnrolledConjunction<0,AccessLen,sizeof(T)>::BodyRedMap(bytes);
                uint32_t redbyteNum = UnrolledConjunction<0,AccessLen,sizeof(T)>::BodyRedNum(redbyteMap);
                AddToRedTable((uint64_t)MAKE_CONTEXT_PAIR(AccessLen, curCtxtHandle), redbyteNum, redbyteMap, AccessLen, pt);
            } else {
                AddToRedTable((uint64_t)MAKE_CONTEXT_PAIR(AccessLen, curCtxtHandle), 0, 0, AccessLen, pt);
            }
        }
    }
#ifdef X86
    static __attribute__((always_inline)) void CheckNByteValueAfterVGather(int32_t slot, instr_t* instr)
    {
        void *drcontext = dr_get_current_drcontext();
        per_thread_t *pt = (per_thread_t *)drmgr_get_tls_field(drcontext, tls_idx);
#ifdef ENABLE_SAMPLING
#ifdef USE_CLEANCALL
        if(enable_sampling) {
            if(!pt->sampleFlag) {
                return ;
            }
        }
#endif
#endif
        dr_mcontext_t mcontext;
        mcontext.size = sizeof(mcontext);
        mcontext.flags= DR_MC_ALL;
        DR_ASSERT(dr_get_mcontext(drcontext, &mcontext));
        context_handle_t curCtxtHandle = drcctlib_get_context_handle(drcontext, slot);
#ifdef DEBUG_VGATHER
        printf("\n^^ CheckNByteValueAfterVGather: ");
        disassemble(drcontext, instr_get_app_pc(instr), 1/*sdtout file desc*/);
        printf("\n");
#endif
        if(isApprox) {
            uint32_t zeros=0;
            uint64_t map=0;
            app_pc addr;
            bool is_write;
            uint32_t pos;
            for( int index=0; instr_compute_address_ex_pos(instr, &mcontext, index, &addr, &is_write, &pos); ++index ) {
                DR_ASSERT(!is_write);
                uint8_t* bytes = reinterpret_cast<uint8_t*>(addr);
                bool hasRedundancy = (bytes[sizeof(T)-1]==0);
                if(hasRedundancy) {
                    if(sizeof(T)==4) { map = map | (UnrolledConjunctionApprox<0,sizeof(T),sizeof(T)>::BodyRedMap(bytes) << (SP_MAP_SIZE * pos) ); }
                    else if(sizeof(T)==8) { map = map |  (UnrolledConjunctionApprox<0,sizeof(T),sizeof(T)>::BodyRedMap(bytes) << (DP_MAP_SIZE * pos) ); }
                    zeros = zeros + UnrolledConjunctionApprox<0,sizeof(T),sizeof(T)>::BodyZeros(bytes);
                }
            }
            AddToApproximateRedTable((uint64_t)curCtxtHandle, map, AccessLen, zeros, AccessLen/sizeof(T), sizeof(T), pt);
        } else {
            assert(0 && "VGather should be a floating point operation!");
        }
    }
#endif
};

template<bool enable_sampling>
static inline void CheckAfterLargeRead(int32_t slot, void* addr, uint32_t accessLen)
{
    void *drcontext = dr_get_current_drcontext();
    per_thread_t *pt = (per_thread_t *)drmgr_get_tls_field(drcontext, tls_idx);
#ifdef ENABLE_SAMPLING
#ifdef USE_CLEANCALL
    if(enable_sampling) {
        if(!pt->sampleFlag) {
            return ;
        }
    }
#endif
#endif
    context_handle_t curCtxtHandle = drcctlib_get_context_handle(drcontext, slot);
    uint8_t* bytes = static_cast<uint8_t*>(addr);
    // quick check whether the most significant byte of the read memory is redundant zero or not
    bool hasRedundancy = (bytes[accessLen-1]==0);
    if(hasRedundancy) {
        // calculate redmap by binary reduction with bitwise operation
        register uint64_t redbyteMap = 0;
        int restLen = accessLen;
        int index = 0;
        while(restLen>=8) {
            redbyteMap = (redbyteMap<<8) | count_zero_bytemap_int64(bytes+index);
            restLen -= 8;
            index += 8;
        }
        while(restLen>=4) {
            redbyteMap = (redbyteMap<<4) | count_zero_bytemap_int32(bytes+index);
            restLen -= 4;
            index += 4;
        }
        while(restLen>=2) {
            redbyteMap = (redbyteMap<<2) | count_zero_bytemap_int16(bytes+index);
            restLen -= 2;
            index += 2;
        }
        while(restLen>=1) {
            redbyteMap = (redbyteMap<<1) | count_zero_bytemap_int8(bytes+index);
            restLen -= 1;
            index += 1;
        }
        // now redmap is calculated, count for redundancy
        bool counting = true;
        register uint64_t redbytesNum = 0;
        restLen = accessLen;
        while(counting && restLen>=8) {
            restLen -= 8;
            register uint8_t t = BitCountTable256[(redbyteMap>>restLen)&0xff];
            redbytesNum += t;
            if(t!=8) {
                counting = false;
                break;
            }
        }
        while(counting && restLen>=4) {
            restLen -= 4;
            register uint8_t t = BitCountTable16[(redbyteMap>>restLen)&0xf];
            redbytesNum += t;
            if(t!=4) {
                counting = false;
                break;
            }
        }
        while(counting && restLen>=2) {
            restLen -= 2;
            register uint8_t t = BitCountTable4[(redbyteMap>>restLen)&0x3];
            redbytesNum += t;
            if(t!=8) {
                counting = false;
                break;
            }
        }
        // dont check here as this loop must execute only once
        while(counting && restLen>=1) {
            restLen -= 1;
            register uint8_t t = (redbyteMap>>restLen)&0x1;
            redbytesNum += t;
        }
        // report in RedTable
        AddToRedTable((uint64_t)MAKE_CONTEXT_PAIR(accessLen, curCtxtHandle), redbytesNum, redbyteMap, accessLen, pt);
    }
    else {
        AddToRedTable((uint64_t)MAKE_CONTEXT_PAIR(accessLen, curCtxtHandle), 0, 0, accessLen, pt);
    }
}
#ifdef ENABLE_SAMPLING
/* Check if sampling is enabled */
#ifdef ZEROSPY_DEBUG
#define HANDLE_CASE(T, ACCESS_LEN, ELEMENT_LEN, IS_APPROX) \
do { \
if(op_no_flush.get_value()) \
{ dr_insert_clean_call(drcontext, bb, ins, (void *)ZeroSpyAnalysis<T, (ACCESS_LEN), (ELEMENT_LEN), (IS_APPROX), true/*sample*/>::CheckNByteValueAfterRead, false, 3, OPND_CREATE_CCT_INT(slot), opnd_create_reg(addr_reg), OPND_CREATE_INTPTR(ins_clone))); } else \
{ dr_insert_clean_call(drcontext, bb, ins, (void *)ZeroSpyAnalysis<T, (ACCESS_LEN), (ELEMENT_LEN), (IS_APPROX), false/* not */>::CheckNByteValueAfterRead, false, 3, OPND_CREATE_CCT_INT(slot), opnd_create_reg(addr_reg), OPND_CREATE_INTPTR(ins_clone)); } } while(0)
#else
#define HANDLE_CASE(T, ACCESS_LEN, ELEMENT_LEN, IS_APPROX) \
do { \
if(op_no_flush.get_value()) \
{ dr_insert_clean_call(drcontext, bb, ins, (void *)ZeroSpyAnalysis<T, (ACCESS_LEN), (ELEMENT_LEN), (IS_APPROX), true/*sample*/>::CheckNByteValueAfterRead, false, 2, OPND_CREATE_CCT_INT(slot), opnd_create_reg(addr_reg)); } else \
{ dr_insert_clean_call(drcontext, bb, ins, (void *)ZeroSpyAnalysis<T, (ACCESS_LEN), (ELEMENT_LEN), (IS_APPROX), false/* not */>::CheckNByteValueAfterRead, false, 2, OPND_CREATE_CCT_INT(slot), opnd_create_reg(addr_reg)); } } while(0)

#endif

#define HANDLE_LARGE() \
do { \
if(op_no_flush.get_value()) \
{ dr_insert_clean_call(drcontext, bb, ins, (void *)CheckAfterLargeRead<true/*sample*/>, false, 3, OPND_CREATE_CCT_INT(slot), opnd_create_reg(addr_reg), OPND_CREATE_CCT_INT(refSize)); } else \
{ dr_insert_clean_call(drcontext, bb, ins, (void *)CheckAfterLargeRead<false/* not */>, false, 3, OPND_CREATE_CCT_INT(slot), opnd_create_reg(addr_reg), OPND_CREATE_CCT_INT(refSize)); } } while(0)

#ifdef X86
#define HANDLE_VGATHER(T, ACCESS_LEN, ELEMENT_LEN, IS_APPROX, ins) \
do { \
if(op_no_flush.get_value()) \
{ dr_insert_clean_call(drcontext, bb, ins, (void *)ZeroSpyAnalysis<T, (ACCESS_LEN), (ELEMENT_LEN), (IS_APPROX), true/*sample*/>::CheckNByteValueAfterVGather, false, 2, OPND_CREATE_CCT_INT(slot), OPND_CREATE_INTPTR(ins_clone)); } else\
{ dr_insert_clean_call(drcontext, bb, ins, (void *)ZeroSpyAnalysis<T, (ACCESS_LEN), (ELEMENT_LEN), (IS_APPROX), false/* not */>::CheckNByteValueAfterVGather, false, 2, OPND_CREATE_CCT_INT(slot), OPND_CREATE_INTPTR(ins_clone)); } } while (0)
#endif

#else
/* NO SAMPLING */
#ifdef ZEROSPY_DEBUG
#define HANDLE_CASE(T, ACCESS_LEN, ELEMENT_LEN, IS_APPROX) \
dr_insert_clean_call(drcontext, bb, ins, (void *)ZeroSpyAnalysis<T, (ACCESS_LEN), (ELEMENT_LEN), (IS_APPROX)>::CheckNByteValueAfterRead, false, 3, OPND_CREATE_CCT_INT(slot), opnd_create_reg(addr_reg),  OPND_CREATE_INTPTR(instr_get_app_pc(ins)))
#else
#define HANDLE_CASE(T, ACCESS_LEN, ELEMENT_LEN, IS_APPROX) \
dr_insert_clean_call(drcontext, bb, ins, (void *)ZeroSpyAnalysis<T, (ACCESS_LEN), (ELEMENT_LEN), (IS_APPROX)>::CheckNByteValueAfterRead, false, 2, OPND_CREATE_CCT_INT(slot), opnd_create_reg(addr_reg))
#endif

#define HANDLE_LARGE() \
dr_insert_clean_call(drcontext, bb, ins, (void *)CheckAfterLargeRead, false, 3, OPND_CREATE_CCT_INT(slot), opnd_create_reg(addr_reg), OPND_CREATE_CCT_INT(refSize))

#ifdef X86
#define HANDLE_VGATHER(T, ACCESS_LEN, ELEMENT_LEN, IS_APPROX, ins) \
dr_insert_clean_call(drcontext, bb, ins, (void *)ZeroSpyAnalysis<T, (ACCESS_LEN), (ELEMENT_LEN), (IS_APPROX)>::CheckNByteValueAfterVGather, false, 2, OPND_CREATE_CCT_INT(slot), OPND_CREATE_INTPTR(ins_clone))
#endif
#endif /*ENABLE_SAMPLING*/

struct ZerospyInstrument{
    static __attribute__((always_inline)) void InstrumentReadValueBeforeAndAfterLoading(void *drcontext, instrlist_t *bb, instr_t *ins, opnd_t opnd, reg_id_t addr_reg, reg_id_t scratch, int32_t slot)
    {
        uint32_t refSize = opnd_size_in_bytes(opnd_get_size(opnd));
        if (refSize==0) {
            // Something strange happened, so ignore it
            return ;
        }
#ifdef ZEROSPY_DEBUG
        per_thread_t *pt2 = (per_thread_t *)drmgr_get_tls_field(drcontext, tls_idx);
        instr_t* ins_clone = instr_clone(drcontext, ins);
        pt2->instr_clones->push_back(ins_clone);
            dr_mutex_lock(gLock);
            dr_fprintf(gDebug, "^^ INFO: Disassembled Instruction ^^^\n");
            dr_fprintf(gDebug, "ins=%p, ins_clone=%p\n", ins, ins_clone);
            disassemble(drcontext, instr_get_app_pc(ins), gDebug);
            disassemble(drcontext, instr_get_app_pc(ins_clone), gDebug);
            dr_fprintf(gDebug, "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
            dr_mutex_unlock(gLock);
#endif
#ifdef ARM_CCTLIB
        // Currently, we cannot identify the difference between floating point operand 
        // and inter operand from instruction type (both is LD), so just ignore the fp
        // FIXME i#4: to identify the floating point through data flow analysis.
        if (false)
#else
        if (instr_is_floating(ins))
#endif
        {
            uint32_t operSize = FloatOperandSizeTable(ins, opnd);
            if(operSize>0) {
                switch(refSize) {
                    case 1:
                    case 2:
#ifdef _WERROR
                        printf("\nERROR: refSize for floating point instruction is too small: %d!\n", refSize);
                        printf("^^ Disassembled Instruction ^^^\n");
                        disassemble(drcontext, instr_get_app_pc(ins), 1/*sdtout file desc*/);
                        printf("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
                        fflush(stdout);
                        assert(0 && "memory read floating data with unexptected small size");
#else
                        dr_mutex_lock(gLock);
                        dr_fprintf(fwarn, "\nERROR: refSize for floating point instruction is too small: %d!\n", refSize);
                        dr_fprintf(fwarn, "^^ Disassembled Instruction ^^^\n");
                        disassemble(drcontext, instr_get_app_pc(ins), fwarn);
                        dr_fprintf(fwarn, "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
                        warned = true;
                        dr_mutex_unlock(gLock);
                        /* Do nothing, just report warning */
                        break;
#endif
                    case 4: HANDLE_CASE(float, 4, 4, true); break;
                    case 8: HANDLE_CASE(double, 8, 8, true); break;
                    case 10: HANDLE_CASE(uint8_t, 10, 1, true); break;
                    case 16: {
                        switch (operSize) {
                            case 4: HANDLE_CASE(float, 16, 4, true); break;
                            case 8: HANDLE_CASE(double, 16, 8, true); break;
                            default: assert(0 && "handle large mem read with unexpected operand size\n"); break;
                        }
                    }break;
                    case 32: {
                        switch (operSize) {
                            case 4: HANDLE_CASE(float, 32, 4, true); break;
                            case 8: HANDLE_CASE(double, 32, 8, true); break;
                            default: assert(0 && "handle large mem read with unexpected operand size\n"); break;
                        }
                    }break;
                    default: 
#ifdef _WERROR
                        printf("\nERROR: refSize for floating point instruction is too large: %d!\n", refSize);
                        printf("^^ Disassembled Instruction ^^^\n");
                        disassemble(drcontext, instr_get_app_pc(ins), 1/*sdtout file desc*/);
                        printf("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
                        fflush(stdout);
                        assert(0 && "unexpected large memory read\n"); break;
#else
                        dr_mutex_lock(gLock);
                        dr_fprintf(fwarn, "\nERROR: refSize for floating point instruction is too large: %d!\n", refSize);
                        dr_fprintf(fwarn, "^^ Disassembled Instruction ^^^\n");
                        disassemble(drcontext, instr_get_app_pc(ins), fwarn);
                        dr_fprintf(fwarn, "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
                        warned = true;
                        dr_mutex_unlock(gLock);
                        /* Do nothing, just report warning */
                        break;
#endif
                }
            }
        } else {
            if(op_no_flush.get_value()) {
                switch(refSize) {
#ifdef SKIP_SMALLCASE
                    // do nothing when access is small
                    case 1: break;
                    case 2: break;
#else
                    case 1: HANDLE_CASE(uint8_t, 1, 1, false); break;
                    case 2: HANDLE_CASE(uint16_t, 2, 2, false); break;
#endif
                    case 4: HANDLE_CASE(uint32_t, 4, 4, false); break;
                    case 8: HANDLE_CASE(uint64_t, 8, 8, false); break;
                        
                    default: {
                        HANDLE_LARGE();
                    }
                }
            } else {
                reg_id_t reg_ctxt, reg_base;
                // We use the scratch to store the calling context value from drcctlib
                reg_ctxt = scratch;
                bool is_saved = !drx_aflags_are_dead(ins);
                if(is_saved) {
                    MINSERT(bb, ins, INSTR_CREATE_pushf(ins));
                }
                RESERVE_REG(drcontext, bb, ins, NULL, reg_base);
                drcctlib_get_context_handle_in_reg(drcontext, bb, ins, slot, reg_ctxt, reg_base);
                dr_insert_read_raw_tls(drcontext, bb, ins, tls_seg, tls_offs + INSTRACE_TLS_OFFS_VAL_CACHE_PTR, reg_base);
                insertSaveCachedKey(drcontext, bb, ins, /*elemSize=*/refSize, /*accesslen=*/refSize, /*isApprox=*/0, reg_base, reg_ctxt, scratch);
                insertComputeAndCacheRedmap_INT(drcontext, bb, ins, refSize, reg_base, addr_reg, scratch);
                // store the new cache pointer to the TLS field
                MINSERT(bb, ins, XINST_CREATE_add(drcontext, opnd_create_reg(reg_base), OPND_CREATE_CCT_INT(sizeof(val_cache_t))));
                dr_insert_write_raw_tls(drcontext, bb, ins, tls_seg, tls_offs + INSTRACE_TLS_OFFS_VAL_CACHE_PTR, reg_base);
                UNRESERVE_REG(drcontext, bb, ins, reg_base);
                if(is_saved) {
                    MINSERT(bb, ins, INSTR_CREATE_popf(ins));
                }
            }
        }
    }
#ifdef X86
    static __attribute__((always_inline)) void InstrumentReadValueBeforeVGather(void *drcontext, instrlist_t *bb, instr_t *ins, int32_t slot){
        opnd_t opnd = instr_get_src(ins, 0);
        uint32_t operSize = FloatOperandSizeTable(ins, opnd); // VGather's second operand is the memory operand
        uint32_t refSize = opnd_size_in_bytes(opnd_get_size(instr_get_dst(ins, 0)));
        per_thread_t *pt = (per_thread_t *)drmgr_get_tls_field(drcontext, tls_idx);
        instr_t* ins_clone = instr_clone(drcontext, ins);
        pt->instr_clones->push_back(ins_clone);
#ifdef DEBUG_VGATHER
        printf("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
        printf("^^ refSize = %d, operSize = %d\n", refSize, operSize);
        printf("^^ Disassembled Instruction ^^^\n");
        disassemble(drcontext, instr_get_app_pc(ins), STDOUT);
        printf("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
#endif
        switch(refSize) {
            case 1:
            case 2: 
            case 4: 
            case 8: 
            case 10: 
#ifdef _WERROR
                    printf("\nERROR: refSize for floating point instruction is too small: %d!\n", refSize);
                    printf("^^ Disassembled Instruction ^^^\n");
                    disassemble(drcontext, instr_get_app_pc(ins), 1/*sdtout file desc*/);
                    printf("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
                    fflush(stdout);
                    assert(0 && "memory read floating data with unexptected small size");
#else
                    dr_mutex_lock(gLock);
                    dr_fprintf(fwarn, "\nERROR: refSize for floating point instruction is too small: %d!\n", refSize);
                    dr_fprintf(fwarn, "^^ Disassembled Instruction ^^^\n");
                    disassemble(drcontext, instr_get_app_pc(ins), fwarn);
                    dr_fprintf(fwarn, "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
                    warned = true;
                    dr_mutex_unlock(gLock);
                    /* Do nothing, just report warning */
                    break;
#endif
            case 16: {
                switch (operSize) {
                    case 4: HANDLE_VGATHER(float, 16, 4, true, ins); break;
                    case 8: HANDLE_VGATHER(double, 16, 8, true, ins); break;
                    default: assert(0 && "handle large mem read with unexpected operand size\n"); break;
                }
            }break;
            case 32: {
                switch (operSize) {
                    case 4: HANDLE_VGATHER(float, 32, 4, true, ins); break;
                    case 8: HANDLE_VGATHER(double, 32, 8, true, ins); break;
                    default: assert(0 && "handle large mem read with unexpected operand size\n"); break;
                }
            }break;
            default: 
#ifdef _WERROR
                    printf("\nERROR: refSize for floating point instruction is too large: %d!\n", refSize);
                    printf("^^ Disassembled Instruction ^^^\n");
                    disassemble(drcontext, instr_get_app_pc(ins), 1/*sdtout file desc*/);
                    printf("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
                    fflush(stdout);
                    assert(0 && "unexpected large memory read\n"); break;
#else
                    dr_mutex_lock(gLock);
                    dr_fprintf(fwarn, "\nERROR: refSize for floating point instruction is too large: %d!\n", refSize);
                    dr_fprintf(fwarn, "^^ Disassembled Instruction ^^^\n");
                    disassemble(drcontext, instr_get_app_pc(ins), fwarn);
                    dr_fprintf(fwarn, "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
                    warned = true;
                    dr_mutex_unlock(gLock);
                    /* Do nothing, just report warning */
                    break;
#endif
        }
    }
#endif
};

inline void getUnusedRegEntry(drvector_t* vec, instr_t* instr) {
    drreg_init_and_fill_vector(vec, true);
    // the regs used in this instr is not allowd to spill
    for(int i=0; i<instr_num_srcs(instr); ++i) {
        opnd_t opnd = instr_get_src(instr, i);
        for(int j=0; j<opnd_num_regs_used(opnd); ++j) {
            drreg_set_vector_entry(vec, opnd_get_reg_used(opnd, j), false);
        }
    }
}

static void
InstrumentMem(void *drcontext, instrlist_t *ilist, instr_t *where, opnd_t ref, int32_t slot, reg_id_t reg1, reg_id_t reg2)
{
    if (!drutil_insert_get_mem_addr(drcontext, ilist, where, ref, reg1/*addr*/,
                                    reg2/*scratch*/)) {
#ifdef _WERROR
        dr_fprintf(STDERR, "\nERROR: drutil_insert_get_mem_addr failed!\n");
        dr_fprintf(STDERR, "^^ Disassembled Instruction ^^^\n");
        disassemble(drcontext, instr_get_app_pc(where), STDERR);
        dr_fprintf(STDERR, "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
        ZEROSPY_EXIT_PROCESS("InstrumentMem drutil_insert_get_mem_addr failed!");
#else
        dr_mutex_lock(gLock);
        dr_fprintf(fwarn, "\nERROR: drutil_insert_get_mem_addr failed!\n");
        dr_fprintf(fwarn, "^^ Disassembled Instruction ^^^\n");
        disassemble(drcontext, instr_get_app_pc(where), fwarn);
        dr_fprintf(fwarn, "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
        warned = true;
        dr_mutex_unlock(gLock);
#endif
    } else {
        drvector_t vec;
        getUnusedRegEntry(&vec, where);
        instr_t* skipcall = NULL;
        // RESERVE_AFLAGS(drcontext, ilist, where);
        if(op_enable_sampling.get_value()) {
            dr_insert_read_raw_tls(drcontext, ilist, where, tls_seg, tls_offs + INSTRACE_TLS_OFFS_BUF_PTR, reg2);
            // Clear insCnt when insCnt > WINDOW_DISABLE
            MINSERT(ilist, where, XINST_CREATE_cmp(drcontext, opnd_create_reg(reg2), OPND_CREATE_CCT_INT(window_enable)));
            skipcall = INSTR_CREATE_label(drcontext);
            MINSERT(ilist, where, XINST_CREATE_jump_cond(drcontext, DR_PRED_GT, opnd_create_instr(skipcall)));
        }
        ZerospyInstrument::InstrumentReadValueBeforeAndAfterLoading(drcontext, ilist, where, ref, reg1, reg2, slot);
        if(op_enable_sampling.get_value()) {
            assert(skipcall!=NULL);
            MINSERT(ilist, where, skipcall);
        }
        // UNRESERVE_AFLAGS(drcontext, ilist, where);
    }
}

void
InstrumentInsCallback(void *drcontext, instr_instrument_msg_t *instrument_msg)
{
    // early return when code flush is enabled and not sampled
    per_thread_t *pt = (per_thread_t *)drmgr_get_tls_field(drcontext, tls_idx);
    // extract data from drcctprof's given message
    instrlist_t *bb = instrument_msg->bb;
    instr_t *instr = instrument_msg->instr;
    int32_t slot = instrument_msg->slot;
    if(!instr_is_app(instr)) return;
    // check if this instruction is actually our interest
    if(!instr_reads_memory(instr)) return;
    // If this instr is prefetch, the memory address is not guaranteed to be valid, and it may be also ignored by hardware
    // Besides, prefetch instructions are not the actual memory access instructions, so we also ignore them
    if(instr_is_prefetch(instr)) return;

    // Currently, we mark x87 control instructions handling FPU states are ignorable (not insterested)
    if(instr_is_ignorable(instr)) {
        return ;
    }

#ifdef X86
    if(!instr_is_gather(instr))
#endif
    {
        bool ignore=true;
        int num_srcs = instr_num_srcs(instr);
        for(int i=0; i<num_srcs; ++i) {
            opnd_t opnd = instr_get_src(instr, i);
            if(opnd_is_memory_reference(opnd)) {
                uint32_t refSize = opnd_size_in_bytes(opnd_get_size(opnd));
                if (refSize!=0) {
                    ignore = false;
                    break;
                }
#ifdef DEBUG_ZEROSPY
                else {
                    // Something strange happened, so ignore it
                    dr_mutex_lock(gLock);
                    dr_fprintf(STDOUT, "^^ INFO: Disassembled Instruction ^^^\n");
                    dr_fprintf(STDOUT, "ins=%p\n", instr);
                    disassemble(drcontext, instr_get_app_pc(instr), STDOUT);
                    dr_fprintf(STDOUT, "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
                    dr_mutex_unlock(gLock);
                }
#endif
            }
        }
        if(ignore) return ;
    }

    // store cct in tls filed
    // InstrumentIns(drcontext, bb, instr, slot);
    /* We need two scratch registers to get the value */
    reg_id_t reg1, reg2;
#ifdef USE_CLEANCALL
#ifdef X86
    // gather may result in failure, need special care
    if(instr_is_gather(instr)) {
        // We use instr_compute_address_ex_pos to handle gather (with VSIB addressing)
        ZerospyInstrument::InstrumentReadValueBeforeVGather(drcontext, bb, instr, slot);
    } else
#endif
    {
        drvector_t vec;
        getUnusedRegEntry(&vec, instr);
        RESERVE_REG(drcontext, bb, instr, &vec, reg1);
        RESERVE_REG(drcontext, bb, instr, &vec, reg2);
        int num_srcs = instr_num_srcs(instr);
        for(int i=0; i<num_srcs; ++i) {
            opnd_t opnd = instr_get_src(instr, i);
            if(opnd_is_memory_reference(opnd)) {
                InstrumentMem(drcontext, bb, instr, opnd, slot, reg1, reg2);
            }
        }
        UNRESERVE_REG(drcontext, bb, instr, reg1);
        UNRESERVE_REG(drcontext, bb, instr, reg2);
    }
#else
// ELSE USE_CLEANCALL
#ifdef ARM_CCTLIB
    drvector_t vec;
    getUnusedRegEntry(&vec, instr);
    RESERVE_REG(drcontext, bb, instr, &vec, reg1);
    RESERVE_REG(drcontext, bb, instr, &vec, reg2);
    instr_t* skipcall = NULL;
    bool dead = false;
    if(op_no_flush.get_value()) {
        RESERVE_AFLAGS(drcontext, bb, instr);
        dr_insert_read_raw_tls(drcontext, bb, instr, tls_seg, tls_offs + INSTRACE_TLS_OFFS_BUF_PTR, reg1);
        // Clear insCnt when insCnt > WINDOW_DISABLE
        minstr_load_wint_to_reg(drcontext, bb, instr, reg2, window_enable);
        MINSERT(bb, instr, XINST_CREATE_cmp(drcontext, opnd_create_reg(reg1), opnd_create_reg(reg2)));
        skipcall = INSTR_CREATE_label(drcontext);
        MINSERT(bb, instr, XINST_CREATE_jump_cond(drcontext, DR_PRED_GT, opnd_create_instr(skipcall)));
    }
    int num_srcs = instr_num_srcs(instr);
    for(int i=0; i<num_srcs; ++i) {
        opnd_t opnd = instr_get_src(instr, i);
        if(opnd_is_memory_reference(opnd)) {
            InstrumentMem(drcontext, bb, instr, opnd, slot, reg1, reg2);
        }
    }
    if(op_no_flush.get_value()) {
        assert(skipcall!=NULL);
        MINSERT(bb, instr, skipcall);
        UNRESERVE_AFLAGS(drcontext, bb, instr);
    }
    UNRESERVE_REG(drcontext, bb, instr, reg1);
    UNRESERVE_REG(drcontext, bb, instr, reg2);
#else
// X86 architecture
    drvector_t vec;
    getUnusedRegEntry(&vec, instr);
#ifdef X86
    // gather may result in failure, need special care
    if(instr_is_gather(instr)) {
        instr_t* skipcall = NULL;
        if(op_enable_sampling.get_value()) {
            RESERVE_AFLAGS(drcontext, bb, instr);
            RESERVE_REG(drcontext, bb, instr, &vec, reg1);
            dr_insert_read_raw_tls(drcontext, bb, instr, tls_seg, tls_offs + INSTRACE_TLS_OFFS_BUF_PTR, reg1);
            // Clear insCnt when insCnt > WINDOW_DISABLE
            MINSERT(bb, instr, XINST_CREATE_cmp(drcontext, opnd_create_reg(reg1), OPND_CREATE_CCT_INT(window_enable)));
            skipcall = INSTR_CREATE_label(drcontext);
            MINSERT(bb, instr, XINST_CREATE_jump_cond(drcontext, DR_PRED_GT, opnd_create_instr(skipcall)));
        }
        // We use instr_compute_address_ex_pos to handle gather (with VSIB addressing)
        ZerospyInstrument::InstrumentReadValueBeforeVGather(drcontext, bb, instr, slot);
        if(op_enable_sampling.get_value()) {
            assert(skipcall!=NULL);
            MINSERT(bb, instr, skipcall);
            UNRESERVE_REG(drcontext, bb, instr, reg1);
            UNRESERVE_AFLAGS(drcontext, bb, instr);
        }
    } else
#endif
    {
        RESERVE_REG(drcontext, bb, instr, &vec, reg1);
        RESERVE_REG(drcontext, bb, instr, &vec, reg2);
        int num_srcs = instr_num_srcs(instr);
        for(int i=0; i<num_srcs; ++i) {
            opnd_t opnd = instr_get_src(instr, i);
            if(opnd_is_memory_reference(opnd)) {
                InstrumentMem(drcontext, bb, instr, opnd, slot, reg1, reg2);
            }
        }
        UNRESERVE_REG(drcontext, bb, instr, reg1);
        UNRESERVE_REG(drcontext, bb, instr, reg2);
    }
#endif
#endif
}

static void
ThreadOutputFileInit(per_thread_t *pt)
{
    int32_t id = drcctlib_get_thread_id();
    char name[MAXIMUM_PATH] = "";
    sprintf(name + strlen(name), "%s/thread-%d.topn.log", g_folder_name.c_str(), id);
    pt->output_file = dr_open_file(name, DR_FILE_WRITE_OVERWRITE | DR_FILE_ALLOW_LARGE);
    DR_ASSERT(pt->output_file != INVALID_FILE);
    if (op_enable_sampling.get_value()) {
        dr_fprintf(pt->output_file, "[ZEROSPY INFO] Sampling Enabled\n");
    }
}

static void
ClientThreadStart(void *drcontext)
{
    // assert(dr_get_itimer(ITIMER_REAL));
    per_thread_t *pt = (per_thread_t *)dr_thread_alloc(drcontext, sizeof(per_thread_t));
    if (pt == NULL) {
        ZEROSPY_EXIT_PROCESS("pt == NULL");
    }
    pt->INTRedMap = new unordered_map<uint64_t, INTRedLogs>();
    pt->FPRedMap = new unordered_map<uint64_t, FPRedLogs>();
    pt->instr_clones = new vector<instr_t*>();
    pt->INTRedMap->rehash(10000000);
    pt->FPRedMap->rehash(10000000);
#ifdef ENABLE_SAMPLING
#ifdef USE_CLEANCALL
    pt->numIns = 0;
    pt->sampleFlag = true;
#else
    pt->numInsBuff = dr_get_dr_segment_base(tls_seg);
    BUF_PTR(pt->numInsBuff, void, INSTRACE_TLS_OFFS_BUF_PTR) = 0;
    pt->cache_ptr = (val_cache_t*)dr_global_alloc(MAX_CACHE_SIZE*sizeof(val_cache_t));
    pt->cache_val_num = 0;
    BUF_PTR(pt->numInsBuff, void, INSTRACE_TLS_OFFS_VAL_CACHE_PTR) = pt->cache_ptr;
#endif
#endif
    drmgr_set_tls_field(drcontext, tls_idx, (void *)pt);
    ThreadOutputFileInit(pt);
}

/*******************************************************************/
/* Output functions */
struct RedundacyData {
    context_handle_t cntxt;
    uint64_t frequency;
    uint64_t all0freq;
    uint64_t ltot;
    uint64_t byteMap;
    uint8_t accessLen;
};

struct ApproxRedundacyData {
    context_handle_t cntxt;
    uint64_t all0freq;
    uint64_t ftot;
    uint64_t byteMap;
    uint8_t accessLen;
    uint8_t size;
};

static inline bool RedundacyCompare(const struct RedundacyData &first, const struct RedundacyData &second) {
    return first.frequency > second.frequency ? true : false;
}
static inline bool ApproxRedundacyCompare(const struct ApproxRedundacyData &first, const struct ApproxRedundacyData &second) {
    return first.all0freq > second.all0freq ? true : false;
}
//#define SKIP_SMALLACCESS
#ifdef SKIP_SMALLACCESS
#define LOGGING_THRESHOLD 100
#endif

#include <omp.h>

static uint64_t PrintRedundancyPairs(per_thread_t *pt, uint64_t threadBytesLoad, int threadId) {
    vector<RedundacyData> tmpList;
    vector<RedundacyData>::iterator tmpIt;

    file_t gTraceFile = pt->output_file;
    
    uint64_t grandTotalRedundantBytes = 0;
    tmpList.reserve(pt->INTRedMap->size());
    printf("Dumping INTEGER Redundancy Info... Total num : %ld\n",pt->INTRedMap->size());
    fflush(stdout);
    dr_fprintf(gTraceFile, "\n--------------- Dumping INTEGER Redundancy Info ----------------\n");
    dr_fprintf(gTraceFile, "\n*************** Dump Data from Thread %d ****************\n", threadId);
    uint64_t count = 0; uint64_t rep = -1;
    for (unordered_map<uint64_t, INTRedLogs>::iterator it = pt->INTRedMap->begin(); it != pt->INTRedMap->end(); ++it) {
        ++count;
        if(100 * count / pt->INTRedMap->size()!=rep) {
            rep = 100 * count / pt->INTRedMap->size();
            printf("\r%ld%%  Finish",rep);
            fflush(stdout);
        }
        RedundacyData tmp = { DECODE_KILL((*it).first), (*it).second.red,(*it).second.fred,(*it).second.tot,(*it).second.redByteMap,DECODE_DEAD((*it).first)};
        tmpList.push_back(tmp);
        grandTotalRedundantBytes += tmp.frequency;
    }
    printf("\r100%%  Finish\n");
    fflush(stdout);
    
    __sync_fetch_and_add(&grandTotBytesRedLoad,grandTotalRedundantBytes);
    printf("Extracted Raw data, now sorting...\n");
    fflush(stdout);
    
    dr_fprintf(gTraceFile, "\n Total redundant bytes = %f %%\n", grandTotalRedundantBytes * 100.0 / threadBytesLoad);
    dr_fprintf(gTraceFile, "\n INFO : Total redundant bytes = %f %% (%ld / %ld) \n", grandTotalRedundantBytes * 100.0 / threadBytesLoad, grandTotalRedundantBytes, threadBytesLoad);
    
#ifdef ENABLE_FILTER_BEFORE_SORT
#define FILTER_THESHOLD 1000
    dr_fprintf(gTraceFile, "\n Filter out small redundancies according to the predefined threshold: %.2lf %%\n", 100.0/(double)FILTER_THESHOLD);
    vector<RedundacyData> tmpList2;
    tmpList2 = move(tmpList);
    tmpList.clear(); // make sure it is empty
    tmpList.reserve(tmpList2.size());
    for(tmpIt = tmpList2.begin();tmpIt != tmpList2.end(); ++tmpIt) {
        if(tmpIt->frequency * FILTER_THESHOLD > tmpIt->ltot) {
            tmpList.push_back(*tmpIt);
        }
    }
    dr_fprintf(gTraceFile, " Remained Redundancies: %ld (%.2lf %%)\n", tmpList.size(), (double)tmpList.size()/(double)tmpList2.size());
    tmpList2.clear();
#undef FILTER_THRESHOLD
#endif

    sort(tmpList.begin(), tmpList.end(), RedundacyCompare);
    printf("Sorted, Now generating reports...\n");
    fflush(stdout);
    int cntxtNum = 0;
    for (vector<RedundacyData>::iterator listIt = tmpList.begin(); listIt != tmpList.end(); ++listIt) {
        if (cntxtNum < MAX_REDUNDANT_CONTEXTS_TO_LOG) {
            dr_fprintf(gTraceFile, "\n\n======= (%f) %% of total Redundant, with local redundant %f %% (%ld Bytes / %ld Bytes) ======\n", 
                (*listIt).frequency * 100.0 / grandTotalRedundantBytes,
                (*listIt).frequency * 100.0 / (*listIt).ltot,
                (*listIt).frequency,(*listIt).ltot);
            dr_fprintf(gTraceFile, "\n\n======= with All Zero Redundant %f %% (%ld / %ld) ======\n", 
                (*listIt).all0freq * (*listIt).accessLen * 100.0 / (*listIt).ltot,
                (*listIt).all0freq,(*listIt).ltot/(*listIt).accessLen);
            dr_fprintf(gTraceFile, "\n======= Redundant byte map : [0] ");
            for(uint32_t i=0;i<(*listIt).accessLen;++i) {
                if((*listIt).byteMap & (1<<i)) {
                    dr_fprintf(gTraceFile, "00 ");
                }
                else {
                    dr_fprintf(gTraceFile, "XX ");
                }
            }
            dr_fprintf(gTraceFile, " [AccessLen=%d] =======\n", (*listIt).accessLen);
            dr_fprintf(gTraceFile, "\n---------------------Redundant load with---------------------------\n");
            drcctlib_print_full_cct(gTraceFile, (*listIt).cntxt, true, true, MAX_DEPTH);
        }
        else {
            break;
        }
        cntxtNum++;
    }
    dr_fprintf(gTraceFile, "\n------------ Dumping INTEGER Redundancy Info Finish -------------\n");
    printf("INTEGER Report dumped\n");
    fflush(stdout);
    return grandTotalRedundantBytes;
}

static uint64_t PrintApproximationRedundancyPairs(per_thread_t *pt, uint64_t threadBytesLoad, int threadId) {
    vector<ApproxRedundacyData> tmpList;
    vector<ApproxRedundacyData>::iterator tmpIt;
    tmpList.reserve(pt->FPRedMap->size());
    
    file_t gTraceFile = pt->output_file;

    uint64_t grandTotalRedundantBytes = 0;
    uint64_t grandTotalRedundantIns = 0;
    dr_fprintf(gTraceFile, "\n--------------- Dumping Approximation Redundancy Info ----------------\n");
    dr_fprintf(gTraceFile, "\n*************** Dump Data(delta=%.2f%%) from Thread %d ****************\n", delta*100,threadId);

    printf("Dumping INTEGER Redundancy Info... Total num : %ld\n",pt->FPRedMap->size());
    uint64_t count = 0; uint64_t rep = -1;
    for (unordered_map<uint64_t, FPRedLogs>::iterator it = pt->FPRedMap->begin(); it != pt->FPRedMap->end(); ++it) {
        ++count;
        if(100 * count / pt->INTRedMap->size()!=rep) {
            rep = 100 * count / pt->INTRedMap->size();
            printf("\r%ld%%  Finish",rep);
            fflush(stdout);
        }
        ApproxRedundacyData tmp = { static_cast<context_handle_t>((*it).first), (*it).second.fred,(*it).second.ftot, (*it).second.redByteMap,(*it).second.AccessLen, (*it).second.size};
        tmpList.push_back(tmp);
        grandTotalRedundantBytes += (*it).second.fred * (*it).second.AccessLen;
    }
    printf("\r100%%  Finish\n");
    fflush(stdout);
    
    __sync_fetch_and_add(&grandTotBytesApproxRedLoad,grandTotalRedundantBytes);
    
    dr_fprintf(gTraceFile, "\n Total redundant bytes = %f %%\n", grandTotalRedundantBytes * 100.0 / threadBytesLoad);
    dr_fprintf(gTraceFile, "\n INFO : Total redundant bytes = %f %% (%ld / %ld) \n", grandTotalRedundantBytes * 100.0 / threadBytesLoad, grandTotalRedundantBytes, threadBytesLoad);
    
#ifdef ENABLE_FILTER_BEFORE_SORT
#define FILTER_THESHOLD 1000
    dr_fprintf(gTraceFile, "\n Filter out small redundancies according to the predefined threshold: %.2lf %%\n", 100.0/(double)FILTER_THESHOLD);
    // pt->FPRedMap->clear();
    vector<ApproxRedundacyData> tmpList2;
    tmpList2 = move(tmpList);
    tmpList.clear(); // make sure it is empty
    tmpList.reserve(tmpList2.size());
    for(tmpIt = tmpList2.begin();tmpIt != tmpList2.end(); ++tmpIt) {
        if(tmpIt->all0freq * FILTER_THESHOLD > tmpIt->ftot) {
            tmpList.push_back(*tmpIt);
        }
    }
    dr_fprintf(gTraceFile, " Remained Redundancies: %ld (%.2lf %%)\n", tmpList.size(), (double)tmpList.size()/(double)tmpList2.size());
    tmpList2.clear();
#undef FILTER_THRESHOLD
#endif

    sort(tmpList.begin(), tmpList.end(), ApproxRedundacyCompare);
    int cntxtNum = 0;
    for (vector<ApproxRedundacyData>::iterator listIt = tmpList.begin(); listIt != tmpList.end(); ++listIt) {
        if (cntxtNum < MAX_REDUNDANT_CONTEXTS_TO_LOG) {
            dr_fprintf(gTraceFile, "\n======= (%f) %% of total Redundant, with local redundant %f %% (%ld Zeros / %ld Reads) ======\n",
                (*listIt).all0freq * 100.0 / grandTotalRedundantBytes,
                (*listIt).all0freq * 100.0 / (*listIt).ftot,
                (*listIt).all0freq,(*listIt).ftot); 
            dr_fprintf(gTraceFile, "\n======= Redundant byte map : [mantiss | exponent | sign] ========\n");
            if((*listIt).size==4) {
                dr_fprintf(gTraceFile, "%s", getFpRedMapString_SP((*listIt).byteMap, (*listIt).accessLen/4).c_str());
            } else {
                dr_fprintf(gTraceFile, "%s", getFpRedMapString_DP((*listIt).byteMap, (*listIt).accessLen/8).c_str());
            }
            dr_fprintf(gTraceFile, "\n===== [AccessLen=%d, typesize=%d] =======\n", (*listIt).accessLen, (*listIt).size);
            dr_fprintf(gTraceFile, "\n---------------------Redundant load with---------------------------\n");
            drcctlib_print_full_cct(gTraceFile, (*listIt).cntxt, true, true, MAX_DEPTH);
        }
        else {
            break;
        }
        cntxtNum++;
    }
    dr_fprintf(gTraceFile, "\n------------ Dumping Approximation Redundancy Info Finish -------------\n");
    printf("Floating Point Report dumped\n");
    fflush(stdout);
    return grandTotalRedundantBytes;
}
/*******************************************************************/
static uint64_t getThreadByteLoad(per_thread_t *pt) {
    register uint64_t x = 0;
    for (unordered_map<uint64_t, INTRedLogs>::iterator it = pt->INTRedMap->begin(); it != pt->INTRedMap->end(); ++it) {
        x += (*it).second.tot;
    }
    for (unordered_map<uint64_t, FPRedLogs>::iterator it = pt->FPRedMap->begin(); it != pt->FPRedMap->end(); ++it) {
        x += (*it).second.ftot * (*it).second.AccessLen;
    }
    return x;
}
/*******************************************************************/
static void
ClientThreadEnd(void *drcontext)
{
#ifdef TIMING
    uint64_t time = get_miliseconds();
#endif
    BBSampleInstrument::bb_flush_code(MAX_CACHE_SIZE);
    per_thread_t *pt = (per_thread_t *)drmgr_get_tls_field(drcontext, tls_idx);
    if (pt->INTRedMap->empty() && pt->FPRedMap->empty()) return;
    uint64_t threadByteLoad = getThreadByteLoad(pt);
    __sync_fetch_and_add(&grandTotBytesLoad,threadByteLoad);
    int threadId = drcctlib_get_thread_id();
    uint64_t threadRedByteLoadINT = PrintRedundancyPairs(pt, threadByteLoad, threadId);
    uint64_t threadRedByteLoadFP = PrintApproximationRedundancyPairs(pt, threadByteLoad, threadId);
#ifdef TIMING
    time = get_miliseconds() - time;
    printf("Thread %d: Time %ld ms for generating outputs\n", threadId, time);
#endif

    dr_mutex_lock(gLock);
    dr_fprintf(gFile, "\n#THREAD %d Redundant Read:", threadId);
    dr_fprintf(gFile, "\nTotalBytesLoad: %lu ",threadByteLoad);
    dr_fprintf(gFile, "\nRedundantBytesLoad: %lu %.2f",threadRedByteLoadINT, threadRedByteLoadINT * 100.0/threadByteLoad);
    dr_fprintf(gFile, "\nApproxRedundantBytesLoad: %lu %.2f\n",threadRedByteLoadFP, threadRedByteLoadFP * 100.0/threadByteLoad);
    dr_mutex_unlock(gLock);

    dr_close_file(pt->output_file);
    dr_global_free(pt->cache_ptr, MAX_CACHE_SIZE*sizeof(val_cache_t));
    delete pt->INTRedMap;
    delete pt->FPRedMap;
    for(size_t i=0;i<pt->instr_clones->size();++i) {
        instr_destroy(drcontext, (*pt->instr_clones)[i]);
    }
    delete pt->instr_clones;
#ifdef DEBUG_REUSE
    dr_close_file(pt->log_file);
#endif
    dr_thread_free(drcontext, pt, sizeof(per_thread_t));
}

static void
ClientInit(int argc, const char *argv[])
{
    /* Parse options */
    std::string parse_err;
    int last_index;
    if (!droption_parser_t::parse_argv(DROPTION_SCOPE_CLIENT, argc, argv, &parse_err, &last_index)) {
        dr_fprintf(STDERR, "Usage error: %s", parse_err.c_str());
        dr_abort();
    }
    /* Creating result directories */
    pid_t pid = getpid();
#ifdef ARM_CCTLIB
    char name[MAXIMUM_PATH] = "arm-";
#else
    char name[MAXIMUM_PATH] = "x86-";
#endif
    gethostname(name + strlen(name), MAXIMUM_PATH - strlen(name));
    sprintf(name + strlen(name), "-%d-zerospy", pid);
    g_folder_name.assign(name, strlen(name));
    mkdir(g_folder_name.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    dr_fprintf(STDOUT, "[ZEROSPY INFO] Profiling result directory: %s\n", g_folder_name.c_str());

    sprintf(name+strlen(name), "/zerospy.log");
    gFile = dr_open_file(name, DR_FILE_WRITE_OVERWRITE | DR_FILE_ALLOW_LARGE);
    DR_ASSERT(gFile != INVALID_FILE);
    if (op_enable_sampling.get_value()) {
        dr_fprintf(STDOUT, "[ZEROSPY INFO] Sampling Enabled\n");
        dr_fprintf(gFile, "[ZEROSPY INFO] Sampling Enabled\n");
        if(op_no_flush.get_value()) {
            dr_fprintf(STDOUT, "[ZEROSPY INFO] Code flush is disabled.\n");
            dr_fprintf(gFile,  "[ZEROSPY INFO] Code flush is disabled.\n");
        }
        dr_fprintf(STDOUT, "[ZEROSPU INFO] Sampling Rate: %.3f, Window Size: %ld\n", conf2rate[op_rate.get_value()], conf2window[op_window.get_value()]);
        dr_fprintf(gFile,  "[ZEROSPU INFO] Sampling Rate: %.3f, Window Size: %ld\n", conf2rate[op_rate.get_value()], conf2window[op_window.get_value()]);
#ifdef USE_CLEANCALL
        if (dr_using_all_private_caches()) {
            BBSample_target[0] = BBSampleTable[op_rate.get_value()][op_window.get_value()][0];
            BBSample_target[1] = BBSampleTable[op_rate.get_value()][op_window.get_value()][1];
            dr_fprintf(STDOUT, "[ZEROSPY INFO] Thread Private is enabled.\n");
            dr_fprintf(gFile,  "[ZEROSPY INFO] Thread Private is enabled.\n");
        } else {
            BBSample_target[0] = BBSampleTableShared[op_rate.get_value()][op_window.get_value()][0];
            BBSample_target[1] = BBSampleTableShared[op_rate.get_value()][op_window.get_value()][1];
            dr_fprintf(STDOUT, "[ZEROSPY INFO] Thread Private is disabled.\n");
            dr_fprintf(gFile,  "[ZEROSPY INFO] Thread Private is disabled.\n");
        }
        BBSampleNoFlush_target = BBSampleNoFlushTable[op_rate.get_value()][op_window.get_value()];
#else
        window_enable = conf2rate[op_rate.get_value()] * conf2window[op_window.get_value()];
        window_disable= conf2window[op_window.get_value()];
#endif
    }
    if (op_help.get_value()) {
        dr_fprintf(STDOUT, "%s\n", droption_parser_t::usage_long(DROPTION_SCOPE_CLIENT).c_str());
        exit(1);
    }
#ifndef _WERROR
    sprintf(name+strlen(name), ".warn");
    fwarn = dr_open_file(name, DR_FILE_WRITE_OVERWRITE | DR_FILE_ALLOW_LARGE);
    DR_ASSERT(fwarn != INVALID_FILE);
#endif
#ifdef ZEROSPY_DEBUG
    sprintf(name+strlen(name), ".debug");
    gDebug = dr_open_file(name, DR_FILE_WRITE_OVERWRITE | DR_FILE_ALLOW_LARGE);
    DR_ASSERT(gDebug != INVALID_FILE);
#endif
    gFlagF = dr_open_file("debug.log", DR_FILE_WRITE_OVERWRITE | DR_FILE_ALLOW_LARGE);
    DR_ASSERT(gFlagF != INVALID_FILE);
}

static void
ClientExit(void)
{
    dr_fprintf(gFile, "\n#Redundant Read:");
    dr_fprintf(gFile, "\nTotalBytesLoad: %lu \n",grandTotBytesLoad);
    dr_fprintf(gFile, "\nRedundantBytesLoad: %lu %.2f\n",grandTotBytesRedLoad, grandTotBytesRedLoad * 100.0/grandTotBytesLoad);
    dr_fprintf(gFile, "\nApproxRedundantBytesLoad: %lu %.2f\n",grandTotBytesApproxRedLoad, grandTotBytesApproxRedLoad * 100.0/grandTotBytesLoad);
#ifndef _WERROR
    if(warned) {
        dr_fprintf(gFile, "####################################\n");
        dr_fprintf(gFile, "WARNING: some unexpected instructions are ignored. Please check zerospy.log.warn for detail.\n");
        dr_fprintf(gFile, "####################################\n");
    }
#endif
    dr_close_file(gFlagF);
    dr_close_file(gFile);
#ifdef ENABLE_SAMPLING
#ifndef USE_CLEANCALL
    if (!dr_raw_tls_cfree(tls_offs, INSTRACE_TLS_COUNT)) {
        ZEROSPY_EXIT_PROCESS(
            "ERROR: zerospy dr_raw_tls_calloc fail");
    }
#endif
#endif
    dr_mutex_destroy(gLock);
    drcctlib_exit();
    if (!drmgr_unregister_thread_init_event(ClientThreadStart) ||
        !drmgr_unregister_thread_exit_event(ClientThreadEnd) ||
#ifdef USE_CLEANCALL
#ifdef ENABLE_SAMPLING
        // must unregister event after client exit, or it will cause unexpected errors during execution
        ( op_enable_sampling.get_value() && 
#endif
#endif
            !drmgr_unregister_bb_instrumentation_event(event_basic_block) 
#ifdef USE_CLEANCALL
#ifdef ENABLE_SAMPLING
        ) 
#endif
#endif
        || !drmgr_unregister_tls_field(tls_idx)) {
        printf("ERROR: zerospy failed to unregister in ClientExit");
        fflush(stdout);
        exit(-1);
    }
    drutil_exit();
    drreg_exit();
    drmgr_exit();
}

#ifdef __cplusplus
extern "C" {
#endif

DR_EXPORT void
dr_client_main(client_id_t id, int argc, const char *argv[])
{
    dr_set_client_name("DynamoRIO Client 'zerospy'",
                       "http://dynamorio.org/issues");
    ClientInit(argc, argv);

    if (!drmgr_init()) {
        ZEROSPY_EXIT_PROCESS("ERROR: zerospy unable to initialize drmgr");
    }
    drreg_options_t ops = { sizeof(ops), 4 /*max slots needed*/, false };
    if (drreg_init(&ops) != DRREG_SUCCESS) {
        ZEROSPY_EXIT_PROCESS("ERROR: zerospy unable to initialize drreg");
    }
    if (!drutil_init()) {
        ZEROSPY_EXIT_PROCESS("ERROR: zerospy unable to initialize drutil");
    }
    drmgr_priority_t thread_init_pri = { sizeof(thread_init_pri),
                                         "zerospy-thread-init", NULL, NULL,
                                         DRCCTLIB_THREAD_EVENT_PRI + 1 };
    drmgr_priority_t thread_exit_pri = { sizeof(thread_exit_pri),
                                         "zerispy-thread-exit", NULL, NULL,
                                         DRCCTLIB_THREAD_EVENT_PRI + 1 };
    if (
#ifdef USE_CLEANCALL
#ifdef ENABLE_SAMPLING
        ( op_enable_sampling.get_value() && 
#endif
#endif
        // bb instrumentation event must be performed in analysis passes (as we don't change the application codes)
        !drmgr_register_bb_instrumentation_event(event_basic_block, NULL, NULL) 
#ifdef USE_CLEANCALL
#ifdef ENABLE_SAMPLING
        ) 
#endif
#endif 
        || !drmgr_register_thread_init_event_ex(ClientThreadStart, &thread_init_pri) 
        || !drmgr_register_thread_exit_event_ex(ClientThreadEnd, &thread_exit_pri) ) {
        ZEROSPY_EXIT_PROCESS("ERROR: zerospy unable to register events");
    }

    tls_idx = drmgr_register_tls_field();
    if (tls_idx == -1) {
        ZEROSPY_EXIT_PROCESS("ERROR: zerospy drmgr_register_tls_field fail");
    }
#ifdef ENABLE_SAMPLING
#ifndef USE_CLEANCALL
    if (!dr_raw_tls_calloc(&tls_seg, &tls_offs, INSTRACE_TLS_COUNT, 0)) {
        ZEROSPY_EXIT_PROCESS(
            "ERROR: zerospy dr_raw_tls_calloc fail");
    }
#endif
#endif
    gLock = dr_mutex_create();

    // drcctlib_init(ZEROSPY_FILTER_READ_MEM_ACCESS_INSTR, INVALID_FILE, InstrumentInsCallback, false/*do data centric*/);
    drcctlib_init_ex(ZEROSPY_FILTER_READ_MEM_ACCESS_INSTR, INVALID_FILE, InstrumentInsCallback, NULL, NULL, DRCCTLIB_CACHE_MODE);
    dr_register_exit_event(ClientExit);
}

#ifdef __cplusplus
}
#endif