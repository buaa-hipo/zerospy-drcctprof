//#include <unordered_map>
#include <map>
#include <vector>
#include <string>
#include <sys/stat.h>
#include <assert.h>
#include <algorithm>

#ifdef DEBUG
#define IF_DEBUG(stat) stat
#else
#define IF_DEBUG(stat)
#endif
// #define USE_TIMER

//#define ZEROSPY_DEBUG
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
#include "trace.h"
#include "../cl_include/rapidjson/document.h"
#include "../cl_include/rapidjson/filewritestream.h"
#include "../cl_include/rapidjson/prettywriter.h"

// #ifdef X86
//     #define USE_SIMD
//     #define USE_SSE
//     #if defined(USE_SIMD) || defined(USE_SSE)
//         #include <nmmintrin.h>
//         #include <immintrin.h>
//         #include <emmintrin.h>
//     #endif
// #endif

#define WINDOW_ENABLE 1000000
#define WINDOW_DISABLE 100000000
// #define WINDOW_CLEAN 10

int window_enable;
int window_disable;

// Client Options
#include "droption.h"
static droption_t<bool> op_enable_sampling
(DROPTION_SCOPE_CLIENT, "enable_sampling", 0, 0, 64, "Enable Bursty Sampling",
 "Enable bursty sampling for lower overhead with less profiling accuracy.");

static droption_t<bool> op_help
(DROPTION_SCOPE_CLIENT, "help", 0, 0, 64, "Show this help",
 "Show this help.");

static droption_t<int> op_window
(DROPTION_SCOPE_CLIENT, "window", WINDOW_DISABLE, 0, INT32_MAX, "Window size configuration of sampling",
 "Window size of sampling. Only available when sampling is enabled.");

static droption_t<int> op_window_enable
(DROPTION_SCOPE_CLIENT, "window_enable", WINDOW_ENABLE, 0, INT32_MAX, "Window enabled size configuration of sampling",
 "Window enabled size of sampling. Only available when sampling is enabled.");

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

#define MAKE_KEY(ctxt_hndl, elementSize, accessLen) \
    ((((uint64_t)(ctxt_hndl))<<32) | ((((uint64_t)(elementSize))<<8)|(uint64_t)(accessLen)))
#define DECODE_CTXT(key) (context_handle_t)(((key) >> 32) & 0xffffffff)
#define DECODE_ACCESSLEN(key) ((uint8_t)((key) & 0xff))
#define DECODE_ELEMENTSIZE(key) ((uint8_t)(((key)>>8) & 0xff))

#define ENCODE_TO_UPPER(upper, src) ((((uint64_t)upper)<<56) | ((uint64_t)src))
#define DECODE_UPPER(enc) (uint8_t)(((uint64_t)enc>>56) & 0xff)
#define DECODE_SRC(enc) (uint64_t)((uint64_t)enc & 0xffffffffffffffLL)

template<int size>
struct cache_t {
    int32_t ctxt_hndl;
    int8_t val[size];
};
/* Max number of mem_ref a buffer can have. */
#define MAX_NUM_MEM_REFS 4096
/* The maximum size of buffer for holding mem_refs. */
#define MEM_BUF_SIZE(size) (sizeof(cache_t<size>) * MAX_NUM_MEM_REFS)

static trace_buf_t *trace_buffer_i1;
static trace_buf_t *trace_buffer_i2;
static trace_buf_t *trace_buffer_i4;
static trace_buf_t *trace_buffer_i8;
static trace_buf_t *trace_buffer_i16;
static trace_buf_t *trace_buffer_i32;
static trace_buf_t *trace_buffer_sp1;
static trace_buf_t *trace_buffer_dp1;
static trace_buf_t *trace_buffer_sp4;
static trace_buf_t *trace_buffer_dp2;
static trace_buf_t *trace_buffer_sp8;
static trace_buf_t *trace_buffer_dp4;

struct INTRedLog_t{
    uint64_t tot;
    uint64_t red;
    uint64_t fred;
    uint64_t redByteMap;
};

// AVX2
#define MAX_VLEN 4
struct FPRedLog_t{
    uint64_t ftot;
    uint64_t fred;
    //uint64_t redByteMap[MAX_VLEN];
    uint64_t redByteMap;
};

typedef std::map<context_handle_t, INTRedLog_t> INTRedLogMap_t;
typedef std::map<context_handle_t, FPRedLog_t> FPRedLogMap_t;

#define MINSERT instrlist_meta_preinsert
#define delta 0.01
#define MAX_REDUNDANT_CONTEXTS_TO_LOG (1000)
// maximum cct depth to print
#define MAX_DEPTH 10

enum {
    INSTRACE_TLS_OFFS_BUF_PTR,
    INSTRACE_TLS_OFFS_INTLOG_PTR,
    INSTRACE_TLS_OFFS_FPLOG_PTR,
    INSTRACE_TLS_COUNT, /* total number of TLS slots allocated */
};

static reg_id_t tls_seg;
static uint tls_offs;
#define TLS_SLOT(tls_base, enum_val) (void **)((byte *)(tls_base) + tls_offs + (enum_val))
#define BUF_PTR(tls_base, type, offs) *(type **)TLS_SLOT(tls_base, offs)

// 1M
#define MAX_CLONE_INS 1048576

typedef struct _per_thread_t {
    FPRedLogMap_t* FPRedLogMap;
    INTRedLogMap_t* INTRedLogMap;
    file_t output_file;
    void* numInsBuff;
    int32_t threadId;
    vector<instr_t*> *instr_clones;
} per_thread_t;

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

/******************************************************************
 * Cached trace implementation of Zerospy for acceleration:
 *  1. For each memory load, we only cache the cct, loaded value, 
 * and the statically encoded information (size, isApprox, offset).
 * If we use data-centric mode, we also record the target address.
 *  2. Note that the previous tracing (caching) will not include any
 * arithmetic operations with any changes to arithmetic flags, so we
 * don't need to heavily reserve/restore the states. Even though it 
 * may result in better data locality when we analyze the loaded data
 * on the fly, we detect some existing bugs and lack of SIMD register
 * reservation supports of drreg, we still directly cache these values
 * for further offline/buffer-clearing analysis. And we can also 
 * benifit from frequent spilling to reserve arithmetic flags when 
 * register pressure is high.
 *  3. For sampling, we will analyze and store the cached traces when
 * the cache is full or the sampling flag is changed to inactive. We 
 * will discard the cache when the sampling flag is not active.
 * ***************************************************************/

// #include "detect.h"
#define IS_SAMPLED(pt, WINDOW_ENABLE) ((int64_t)(BUF_PTR(pt->numInsBuff, void, INSTRACE_TLS_OFFS_BUF_PTR))<(int64_t)WINDOW_ENABLE)

file_t gFile;
FILE* gJson;
rapidjson::Document gDoc;
rapidjson::Document::AllocatorType &jsonAllocator = gDoc.GetAllocator();
rapidjson::Value metricOverview(rapidjson::kObjectType);
rapidjson::Value totalIntegerRedundantBytes(rapidjson::kObjectType);
rapidjson::Value totalFloatRedundantBytes(rapidjson::kObjectType);
std::map<int32_t, rapidjson::Value> threadDetailedMetricsMap;
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

/****************************************************************************************/
inline __attribute__((always_inline)) uint64_t count_zero_bytemap_int8(uint8_t * addr) {
    return addr[0]==0?1:0;
    // register uint8_t xx = *((uint8_t*)addr);
    // // reduce by bits until byte level
    // // xx = xx | (xx>>1) | (xx>>2) | (xx>>3) | (xx>>4) | (xx>>5) | (xx>>6) | (xx>>7);
    // xx = xx | (xx>>1);
    // xx = xx | (xx>>2);
    // xx = xx | (xx>>4);
    // // now xx is byte level reduced, check if it is zero and mask the unused bits
    // xx = (~xx) & 0x1;
    // return xx;
}
inline __attribute__((always_inline)) uint64_t count_zero_bytemap_int16(uint8_t * addr) {
    register uint16_t xx = *((uint16_t*)addr);
    // reduce by bits until byte level
    // xx = xx | (xx>>1) | (xx>>2) | (xx>>3) | (xx>>4) | (xx>>5) | (xx>>6) | (xx>>7);
    xx = xx | (xx>>1);
    xx = xx | (xx>>2);
    xx = xx | (xx>>4);
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
    // xx = xx | (xx>>1) | (xx>>2) | (xx>>3) | (xx>>4) | (xx>>5) | (xx>>6) | (xx>>7);
    xx = xx | (xx>>1);
    xx = xx | (xx>>2);
    xx = xx | (xx>>4);
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
    // xx = xx | (xx>>1) | (xx>>2) | (xx>>3) | (xx>>4) | (xx>>5) | (xx>>6) | (xx>>7);
    xx = xx | (xx>>1);
    xx = xx | (xx>>2);
    xx = xx | (xx>>4);
    // now xx is byte level reduced, check if it is zero and mask the unused bits
    xx = (~xx) & 0x101010101010101LL;
    // narrowing
    xx = xx | (xx>>7);
    xx = xx | (xx>>14);
    xx = xx | (xx>>28);
    xx = xx & 0xff;
    return xx;
}
#ifdef USE_SIMD
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

inline __attribute__((always_inline)) uint64_t count_zero_bytemap_int128(uint8_t * addr) {
    uint64_t xx;
    __m128i mmx, tmp;
    // load 128-bit val
    mmx = _mm_loadu_si128((__m128i*)addr);
    // Merge all bits within a byte in parallel
    // 0x
    tmp = _mm_srli_epi64 (mmx, 1);
    mmx = _mm_or_si128(mmx, tmp);
    // 00xx
    tmp = _mm_srli_epi64 (mmx, 2);
    mmx = _mm_or_si128(mmx, tmp);
    // 0000xxxx
    tmp = _mm_srli_epi64 (mmx, 4);
    mmx = _mm_or_si128(mmx, tmp);
    // x = (~x) & broadcast(0x01)
    // the mask is already aligned
    tmp = _mm_load_si128((__m128i*)mask);
    mmx = _mm_andnot_si128(mmx, tmp);
    /* Now SIMD reg_val contains the collected bitmap for each byte, we now
       narrow them into each 64-bit element in this packed SIMD register*/
    // x = (x>>7 | x)
    tmp = _mm_srli_epi64 (mmx, 7);
    mmx = _mm_or_si128(mmx, tmp);
    // x = (x>>14 | x)
    tmp = _mm_srli_epi64 (mmx, 14);
    mmx = _mm_or_si128(mmx, tmp);
    // x = (x>>28 | x)
    tmp = _mm_srli_epi64 (mmx, 28);
    mmx = _mm_or_si128(mmx, tmp);
    /* After narrowed them by 64-bit elementwise merging, the lowest byte of
      each element contains the collected redmap, so we can now narrow them
      by select (bytewise permutation).*/
    // x = permuteb(x, {0,8,...})
    // shuffle: [...clear...] [72:64] [8:0]
    // We directly load the simd mask from memory
    tmp = _mm_load_si128((__m128i*)mask_shuf);
    mmx = _mm_shuffle_epi8(mmx, tmp);
    // now store the lower 16-bits into target (INOUT) register
    union U128I {
        __m128i  v;
        uint16_t e[8];
    } cast;
    cast.v = mmx;
    xx = (uint64_t)cast.e[0];
    return xx;
}

inline __attribute__((always_inline)) uint64_t count_zero_bytemap_int256(uint8_t * addr) {
    uint64_t xx;
    __m256i mmx, tmp;
    // Load data from memory via SIMD instruction
    mmx = _mm256_loadu_si256((__m256i*)addr);
    // Merge all bits within a byte in parallel
    // 0x
    tmp = _mm256_srli_epi64(mmx, 1);
    mmx = _mm256_or_si256(mmx, tmp);
    // 00xx
    tmp = _mm256_srli_epi64(mmx, 2);
    mmx = _mm256_or_si256(mmx, tmp);
    // 0000xxxx
    tmp = _mm256_srli_epi64(mmx, 4);
    mmx = _mm256_or_si256(mmx, tmp);
    // x = (~x) & broadcast(0x01)
    tmp = _mm256_load_si256((__m256i*)mask);
    mmx = _mm256_andnot_si256(mmx, tmp);
    /* Now SIMD reg_val contains the collected bitmap for each byte, we now
       narrow them into each 64-bit element in this packed SIMD register*/
    // x = (x>>7 | x)
    tmp = _mm256_srli_epi64 (mmx, 7);
    mmx = _mm256_or_si256(mmx, tmp);
    // x = (x>>14 | x)
    tmp = _mm256_srli_epi64 (mmx, 14);
    mmx = _mm256_or_si256(mmx, tmp);
    // x = (x>>28 | x)
    tmp = _mm256_srli_epi64 (mmx, 28);
    mmx = _mm256_or_si256(mmx, tmp);
    /* After narrowed them by 64-bit elementwise merging, the lowest byte of
      each element contains the collected redmap, so we can now narrow them
      by select (bytewise permutation).*/
    // x = permuteb(x, {0,8,...})
    // shuffle: [...clear...] [200:192] [136:128] | [...clear...] [72:64] [8:0]
    // We directly load the simd mask from memory
    tmp = _mm256_load_si256((__m256i*)mask_shuf);
    mmx = _mm256_shuffle_epi8(mmx, tmp);
    // As shuffle is performed per lane, so we need further merging
    // 1. permutation to merge two lanes into the first lane: 8 = (10) (00) -> [...] [192:128] [64:0]
    mmx = _mm256_permute4x64_epi64(mmx, 8);
    // 2. shuffle again for narrowing into lower 64-bit value, here we reuse the previously loaded mask in simd scratch register
    mmx = _mm256_shuffle_epi8(mmx, tmp);
    // now store the lower 32-bits into target (INOUT) register
    union U256I {
        __m256i  v;
        uint32_t e[8];
    } cast;
    cast.v = mmx;
    xx = (uint64_t)cast.e[0];
    return xx;
}
#endif
/*******************************************************************************************/
inline __attribute__((always_inline)) void AddINTRedLog(uint64_t ctxt_hndl, uint64_t accessLen, uint64_t redZero, uint64_t fred, uint64_t redByteMap, per_thread_t* pt) {
    //uint64_t key = MAKE_KEY(ctxt_hndl, 0/*elementSize, not used*/, accessLen);
    INTRedLogMap_t::iterator it = pt->INTRedLogMap->find(ctxt_hndl);
    if(it==pt->INTRedLogMap->end()) {
        INTRedLog_t log_ptr = { ENCODE_TO_UPPER(accessLen, accessLen), redZero, fred, redByteMap };
        (*pt->INTRedLogMap)[ctxt_hndl] = log_ptr;
    } else {
        it->second.tot += accessLen;
        it->second.red += redZero;
        it->second.fred += fred;
        it->second.redByteMap &= redByteMap;
    }
}

static const unsigned char BitCountTable4[] __attribute__ ((aligned(64))) = {
    0, 0, 1, 2
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

template<int start, int end, int step>
struct UnrolledFunctions {
    static inline __attribute__((always_inline)) int getFullyRedundantZeroFP(void* pval) {
        switch(step) {
            case 4:
                return (((*reinterpret_cast<uint32_t*>(((uint8_t*)pval)+start))&0x7fffffffLL)==0?1:0)
                    + UnrolledFunctions<start+step, end, step>::getFullyRedundantZeroFP(pval);
            case 8:
                return (((*reinterpret_cast<uint64_t*>(((uint8_t*)pval)+start))&0x7fffffffffffffffLL)==0?1:0)
                    + UnrolledFunctions<start+step, end, step>::getFullyRedundantZeroFP(pval);
        }
        assert(0);
        return 0;
    }
    static inline __attribute__((always_inline)) void mergeRedByteMapFP(void* redByteMap, void* pval) {
        switch(step) {
            case 4:
                reinterpret_cast<uint32_t*>(redByteMap)[start] |= reinterpret_cast<uint32_t*>(pval)[start];
                break;
            case 8:
                reinterpret_cast<uint64_t*>(redByteMap)[start] |= reinterpret_cast<uint64_t*>(pval)[start];
                break;
            default:
                assert(0 && "Unknown type size!\n");
        }
        UnrolledFunctions<start+step, end, step>::mergeRedByteMapFP(redByteMap, pval);
    }
    static inline __attribute__((always_inline)) void memcpy(void* dst, void* src) {
        switch(step) {
            case 4:
                reinterpret_cast<uint32_t*>(dst)[start] = reinterpret_cast<uint32_t*>(src)[start];
                break;
            case 8:
                reinterpret_cast<uint64_t*>(dst)[start] = reinterpret_cast<uint64_t*>(src)[start];
                break;
            default:
                assert(0 && "Unknown type size!\n");
        }
        UnrolledFunctions<start+step, end, step>::memcpy(dst, src);
    }
    static __attribute__((always_inline)) uint64_t BodyRedNum(uint64_t rmap){
        static_assert(start < end);
        if(step==1)
            return ((start==0) ? (rmap&0x1) : ((rmap>>start)&0x1)) + (UnrolledFunctions<start+step,end,step>::BodyRedNum(rmap));
        else if(step==2)
            return ((start==0) ? BitCountTable4[rmap&0x3] : BitCountTable4[(rmap>>start)&0x3]) + (UnrolledFunctions<start+step,end,step>::BodyRedNum(rmap));
        else if(step==4)
            return ((start==0) ? BitCountTable16[rmap&0xf] : BitCountTable16[(rmap>>start)&0xf]) + (UnrolledFunctions<start+step,end,step>::BodyRedNum(rmap));
        else if(step==8)
            return ((start==0) ? BitCountTable256[rmap&0xff] : BitCountTable256[(rmap>>start)&0xff]) + (UnrolledFunctions<start+step,end,step>::BodyRedNum(rmap));
        return 0;
    }
    static __attribute__((always_inline)) uint64_t BodyRedMapFP(uint8_t* addr){
        if(step==4)
            return count_zero_bytemap_fp((void*)(addr+start)) | (UnrolledFunctions<start+step,end,step>::BodyRedMapFP(addr)<<SP_MAP_SIZE);
        else if(step==8)
            return count_zero_bytemap_dp((void*)(addr+start)) | (UnrolledFunctions<start+step,end,step>::BodyRedMapFP(addr)<<DP_MAP_SIZE);
        else
            assert(0 && "Not Supportted floating size! now only support for FP32 or FP64.");
        return 0;
    }
    static __attribute__((always_inline)) uint64_t BodyHasRedundancy(uint8_t* addr){
        if(step==4)
            return hasRedundancy_fp((void*)(addr+start)) || (UnrolledFunctions<start+step,end,step>::BodyHasRedundancy(addr));
        else if(step==8)
            return hasRedundancy_dp((void*)(addr+start)) || (UnrolledFunctions<start+step,end,step>::BodyHasRedundancy(addr));
        else
            assert(0 && "Not Supportted floating size! now only support for FP32 or FP64.");
        return 0;
    }
};

template<int end, int step>
struct UnrolledFunctions<end, end, step> {
    static inline __attribute__((always_inline)) int getFullyRedundantZeroFP(void* pval) {
        return 0;
    }
    static inline __attribute__((always_inline)) void mergeRedByteMapFP(void* redByteMap, void* pval) {
        return ;
    }
    static inline __attribute__((always_inline)) void memcpy(void* dst, void* src) {
        return ;
    }
    static inline __attribute__((always_inline)) uint64_t BodyRedNum(uint64_t rmap) {
        return 0;
    }
    static __attribute__((always_inline)) uint64_t BodyRedMapFP(uint8_t* addr){
        return 0;
    }
    static __attribute__((always_inline)) uint64_t BodyHasRedundancy(uint8_t* addr){
        return 0;
    }
};

template<int esize, int size>
inline __attribute__((always_inline)) void AddFPRedLog(uint64_t ctxt_hndl, void* pval, per_thread_t* pt) {
    //uint64_t key = MAKE_KEY(ctxt_hndl, esize, size);
    FPRedLogMap_t::iterator it = pt->FPRedLogMap->find(ctxt_hndl);
    bool hasRedundancy = UnrolledFunctions<0, size, esize>::BodyHasRedundancy((uint8_t*)pval);
    if(it==pt->FPRedLogMap->end()) {
        FPRedLog_t log_ptr;
        log_ptr.ftot = ENCODE_TO_UPPER(size, size/esize);
        if(hasRedundancy) {
            uint64_t fred = UnrolledFunctions<0, size, esize>::getFullyRedundantZeroFP(pval);
            log_ptr.fred = ENCODE_TO_UPPER(esize, fred);
            log_ptr.redByteMap = UnrolledFunctions<0,size,esize>::BodyRedMapFP((uint8_t*)pval);
        } else {
            log_ptr.fred = ENCODE_TO_UPPER(esize, 0);
            log_ptr.redByteMap = 0;
        }
        //UnrolledFunctions<0, size, esize>::memcpy(log_ptr.redByteMap, pval);
        (*pt->FPRedLogMap)[ctxt_hndl] = log_ptr;
    } else {
        it->second.ftot += size/esize;
        if(hasRedundancy) {
            it->second.fred += UnrolledFunctions<0, size, esize>::getFullyRedundantZeroFP(pval);
            if(it->second.redByteMap) {
                it->second.redByteMap &= UnrolledFunctions<0,size,esize>::BodyRedMapFP((uint8_t*)pval);
            }
        }
    }
}

template<int accessLen, int elementSize>
inline __attribute__((always_inline))
void CheckAndInsertIntPage_impl(int32_t ctxt_hndl, void* addr, per_thread_t *pt) {
    // update info
    uint8_t* bytes = reinterpret_cast<uint8_t*>(addr);
    if(bytes[accessLen-1]!=0) {
        // the log have already been clear to 0, so we do nothing here and quick return.
        AddINTRedLog(ctxt_hndl, accessLen, 0, 0, 0, pt);
        return ;
    }
    uint64_t redByteMap;
    switch(accessLen) {
        case 1:
            redByteMap = count_zero_bytemap_int8(bytes);
            break;
        case 2:
            redByteMap = count_zero_bytemap_int16(bytes);
            break;
        case 4:
            redByteMap = count_zero_bytemap_int32(bytes);
            break;
        case 8:
            redByteMap = count_zero_bytemap_int64(bytes);
            break;
        case 16:
#ifdef USE_SIMD
            redByteMap = count_zero_bytemap_int128(bytes);
#else
            redByteMap = count_zero_bytemap_int64(bytes) |
                        (count_zero_bytemap_int64(bytes+8)<<8);
#endif
            break;
        case 32:
#ifdef USE_SIMD
            redByteMap = count_zero_bytemap_int256(bytes);
#else
            redByteMap = count_zero_bytemap_int64(bytes) |
                        (count_zero_bytemap_int64(bytes+8)<<8) |
                        (count_zero_bytemap_int64(bytes+16)<<16) |
                        (count_zero_bytemap_int64(bytes+24)<<24);
#endif
            break;
        default:
            assert(0 && "UNKNOWN ACCESSLEN!\n");
    }
#ifdef USE_SSE
    if(elementSize==1) {
        uint64_t redZero = _mm_popcnt_u64(redByteMap);
        AddINTRedLog(ctxt_hndl, accessLen, redZero, redZero, redByteMap, pt);
    } else {
        // accessLen == elementSize
        uint64_t redByteMap_2 = (~redByteMap) & ((1LL<<accessLen)-1);
        uint64_t redZero = _lzcnt_u64(redByteMap_2) - (64-accessLen);
        AddINTRedLog(ctxt_hndl, accessLen, redZero, redZero==accessLen?1:0, redByteMap, pt);
    }
#else
    uint64_t redZero = UnrolledFunctions<0, accessLen, elementSize>::BodyRedNum(redByteMap);
    if(elementSize==1) {
        AddINTRedLog(ctxt_hndl, accessLen, redZero, redZero, redByteMap, pt);
    } else {
        AddINTRedLog(ctxt_hndl, accessLen, redZero, redZero==accessLen?1:0, redByteMap, pt);
    }
#endif
}

template<int accessLen, int elementSize, bool enable_cache>
void CheckAndInsertIntPage(int slot, void* addr) {
    static_assert(  accessLen==1 || 
                    accessLen==2 || 
                    accessLen==4 || 
                    accessLen==8 || 
                    accessLen==16 || 
                    accessLen==32);
    static_assert(  accessLen==elementSize || elementSize==1  );
    void *drcontext = dr_get_current_drcontext();
    per_thread_t *pt = (per_thread_t *)drmgr_get_tls_field(drcontext, tls_idx);
    context_handle_t ctxt_hndl = drcctlib_get_context_handle(drcontext, slot);
    CheckAndInsertIntPage_impl<accessLen, elementSize>(ctxt_hndl, addr, pt);
}

template<bool enable_cache>
void CheckLargeAndInsertIntPage(int slot, void* addr, int accessLen) {
    void *drcontext = dr_get_current_drcontext();
    per_thread_t *pt = (per_thread_t *)drmgr_get_tls_field(drcontext, tls_idx);
    context_handle_t ctxt_hndl = drcctlib_get_context_handle(drcontext, slot);
    uint64_t redByteMap=0;
    uint8_t* bytes = reinterpret_cast<uint8_t*>(addr);
    int i=0;
    while(i<accessLen) {
        redByteMap |= count_zero_bytemap_int64(&bytes[i])<<i;
        i += 8;
    }
    while(i<accessLen) {
        redByteMap |= count_zero_bytemap_int32(&bytes[i])<<i;
        i += 4;
    }
    while(i<accessLen) {
        redByteMap |= count_zero_bytemap_int16(&bytes[i])<<i;
        i += 2;
    }
    while(i<accessLen) {
        redByteMap |= count_zero_bytemap_int8(&bytes[i])<<i;
        i += 1;
    }
#ifdef USE_SSE
    uint64_t redZero = _mm_popcnt_u64(redByteMap);
#else
    // now redmap is calculated, count for redundancy
    bool counting = true;
    register uint64_t redZero = 0;
    int restLen = accessLen;
    while(counting && restLen>=8) {
        restLen -= 8;
        register uint8_t t = BitCountTable256[(redByteMap>>restLen)&0xff];
        redZero += t;
        if(t!=8) {
            counting = false;
            break;
        }
    }
    while(counting && restLen>=4) {
        restLen -= 4;
        register uint8_t t = BitCountTable16[(redByteMap>>restLen)&0xf];
        redZero += t;
        if(t!=4) {
            counting = false;
            break;
        }
    }
    while(counting && restLen>=2) {
        restLen -= 2;
        register uint8_t t = BitCountTable4[(redByteMap>>restLen)&0x3];
        redZero += t;
        if(t!=8) {
            counting = false;
            break;
        }
    }
    // dont check here as this loop must execute only once
    while(counting && restLen>=1) {
        restLen -= 1;
        register uint8_t t = (redByteMap>>restLen)&0x1;
        redZero += t;
    }
#endif
    AddINTRedLog(ctxt_hndl, accessLen, redZero, redZero, redByteMap, pt);
}

template<int accessLen, int elementSize, bool enable_cache>
void CheckAndInsertFpPage(int slot, void* addr) {
    static_assert(  accessLen==4 || 
                    accessLen==8 || 
                    accessLen==16 || 
                    accessLen==32);
    static_assert(  elementSize==4 || elementSize==8  );
    void *drcontext = dr_get_current_drcontext();
    per_thread_t *pt = (per_thread_t *)drmgr_get_tls_field(drcontext, tls_idx);
    context_handle_t ctxt_hndl = drcctlib_get_context_handle(drcontext, slot);
    AddFPRedLog<elementSize, accessLen>(ctxt_hndl, addr, pt);
}

#ifdef X86
template<uint32_t AccessLen, uint32_t ElemLen, bool isApprox, bool enable_sampling>
void CheckNByteValueAfterVGather(int slot, instr_t* instr)
{
    static_assert(ElemLen==4 || ElemLen==8);
    void *drcontext = dr_get_current_drcontext();
    context_handle_t ctxt_hndl = drcctlib_get_context_handle(drcontext, slot);
    per_thread_t *pt = (per_thread_t *)drmgr_get_tls_field(drcontext, tls_idx);
    if(enable_sampling) {
        if(!IS_SAMPLED(pt, window_enable)) {
            return ;
        }
    }
    dr_mcontext_t mcontext;
    mcontext.size = sizeof(mcontext);
    mcontext.flags= DR_MC_ALL;
    DR_ASSERT(dr_get_mcontext(drcontext, &mcontext));
#ifdef DEBUG_VGATHER
    printf("\n^^ CheckNByteValueAfterVGather: ");
    disassemble(drcontext, instr_get_app_pc(instr), 1/*sdtout file desc*/);
    printf("\n");
#endif
    if(isApprox) {
        app_pc addr;
        bool is_write;
        uint32_t pos;
        for( int index=0; instr_compute_address_ex_pos(instr, &mcontext, index, &addr, &is_write, &pos); ++index ) {
            DR_ASSERT(!is_write);
            AddFPRedLog<ElemLen, ElemLen>(ctxt_hndl, addr, pt);
        }
    } else {
        assert(0 && "VGather should be a floating point operation!");
    }
}
#define HANDLE_VGATHER(T, ACCESS_LEN, ELEMENT_LEN, IS_APPROX) do {\
if(op_enable_sampling.get_value()) { \
dr_insert_clean_call(drcontext, bb, ins, (void *)CheckNByteValueAfterVGather<(ACCESS_LEN), (ELEMENT_LEN), (IS_APPROX), true>, false, 2, OPND_CREATE_CCT_INT(slot), OPND_CREATE_INTPTR(ins_clone)); \
} else { \
dr_insert_clean_call(drcontext, bb, ins, (void *)CheckNByteValueAfterVGather<(ACCESS_LEN), (ELEMENT_LEN), (IS_APPROX), false>, false, 2, OPND_CREATE_CCT_INT(slot), OPND_CREATE_INTPTR(ins_clone)); \
} } while(0)
#endif
/***************************************************************************************/

template<bool enable_cache>
void insertCheckAndUpdateInt(void* drcontext, instrlist_t* ilist, instr_t* where, int accessLen, int elemLen, int32_t slot, reg_id_t reg_addr, reg_id_t scratch) {
    // quick check
    //assert((accessLen<=8 && accessLen==elemLen) || elemLen==1);
    // start check and update
    switch(accessLen) {
        case 1:
            dr_insert_clean_call(drcontext, ilist, where, (void*)CheckAndInsertIntPage<1,1,enable_cache>, false, 2, OPND_CREATE_CCT_INT(slot), opnd_create_reg(reg_addr));
            break;
        case 2:
            dr_insert_clean_call(drcontext, ilist, where, (void*)CheckAndInsertIntPage<2,2,enable_cache>, false, 2, OPND_CREATE_CCT_INT(slot), opnd_create_reg(reg_addr));
            break;
        case 4:
            dr_insert_clean_call(drcontext, ilist, where, (void*)CheckAndInsertIntPage<4,4,enable_cache>, false, 2, OPND_CREATE_CCT_INT(slot), opnd_create_reg(reg_addr));
            break;
        case 8:
            dr_insert_clean_call(drcontext, ilist, where, (void*)CheckAndInsertIntPage<8,8,enable_cache>, false, 2, OPND_CREATE_CCT_INT(slot), opnd_create_reg(reg_addr));
            break;
        case 16:
            dr_insert_clean_call(drcontext, ilist, where, (void*)CheckAndInsertIntPage<16,1,enable_cache>, false, 2, OPND_CREATE_CCT_INT(slot), opnd_create_reg(reg_addr));
            break;
        case 32:
            dr_insert_clean_call(drcontext, ilist, where, (void*)CheckAndInsertIntPage<32,1,enable_cache>, false, 2, OPND_CREATE_CCT_INT(slot), opnd_create_reg(reg_addr));
            break;
        default:
            dr_insert_clean_call(drcontext, ilist, where, (void*)CheckLargeAndInsertIntPage<enable_cache>, false, 3, OPND_CREATE_CCT_INT(slot), opnd_create_reg(reg_addr), OPND_CREATE_CCT_INT(accessLen));
            break;
    }
}

template<bool enable_cache>
void insertCheckAndUpdateFp(void* drcontext, instrlist_t* ilist, instr_t* where, int accessLen, int elemLen, int32_t slot, reg_id_t reg_addr, reg_id_t scratch) {
    // quick check
    //assert(elemLen==4 || elemLen==8);
    switch(accessLen) {
        case 4:
            dr_insert_clean_call(drcontext, ilist, where, (void*)CheckAndInsertFpPage<4,4,enable_cache>, false, 2, OPND_CREATE_CCT_INT(slot), opnd_create_reg(reg_addr));
            break;
        case 8:
            dr_insert_clean_call(drcontext, ilist, where, (void*)CheckAndInsertFpPage<8,8,enable_cache>, false, 2, OPND_CREATE_CCT_INT(slot), opnd_create_reg(reg_addr));
            break;
        case 16:
            if(elemLen==4) {
                dr_insert_clean_call(drcontext, ilist, where, (void*)CheckAndInsertFpPage<16,4,enable_cache>, false, 2, OPND_CREATE_CCT_INT(slot), opnd_create_reg(reg_addr));
            } else {
                dr_insert_clean_call(drcontext, ilist, where, (void*)CheckAndInsertFpPage<16,8,enable_cache>, false, 2, OPND_CREATE_CCT_INT(slot), opnd_create_reg(reg_addr));
            }
            break;
        case 32:
            if(elemLen==4) {
                dr_insert_clean_call(drcontext, ilist, where, (void*)CheckAndInsertFpPage<32,4,enable_cache>, false, 2, OPND_CREATE_CCT_INT(slot), opnd_create_reg(reg_addr));
            } else {
                dr_insert_clean_call(drcontext, ilist, where, (void*)CheckAndInsertFpPage<32,8,enable_cache>, false, 2, OPND_CREATE_CCT_INT(slot), opnd_create_reg(reg_addr));
            }
            break;
        default:
            assert(0 && "Unknown accessLen");
    }
}

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
/*******************************************************************************************/

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

// This clean call will be automatically inlined by -opt_cleancall as it only has one argument
struct BBInstrument {
    static void BBUpdate(int insCnt) {
        void *drcontext = dr_get_current_drcontext();
        per_thread_t *pt = (per_thread_t *)drmgr_get_tls_field(drcontext, tls_idx);
        uint64_t val = reinterpret_cast<uint64_t>(BUF_PTR(pt->numInsBuff, void, INSTRACE_TLS_OFFS_BUF_PTR));
        val += insCnt;
        BUF_PTR(pt->numInsBuff, void, INSTRACE_TLS_OFFS_BUF_PTR) = reinterpret_cast<void*>(val);
    }

    static void BBUpdateAndCheck(int insCnt) {
        void *drcontext = dr_get_current_drcontext();
        per_thread_t *pt = (per_thread_t *)drmgr_get_tls_field(drcontext, tls_idx);
        uint64_t val = reinterpret_cast<uint64_t>(BUF_PTR(pt->numInsBuff, void, INSTRACE_TLS_OFFS_BUF_PTR));
        val += insCnt;
        if(val>=(uint64_t)window_disable) { val=val-(uint64_t)window_disable; }
        BUF_PTR(pt->numInsBuff, void, INSTRACE_TLS_OFFS_BUF_PTR) = reinterpret_cast<void*>(val);
    }
};

template<int sz, int esize>
void trace_update_int() {
    void* drcontext = dr_get_current_drcontext();
    // here we don't need to pass in the trace buffer pointer as we can statically know
    // which buffer will be updated at compile time.
    trace_buf_t* trace_buffer;
    switch (sz) {
        case 1:
            trace_buffer = trace_buffer_i1; break;
        case 2:
            trace_buffer = trace_buffer_i2; break;
        case 4:
            trace_buffer = trace_buffer_i4; break;
        case 8:
            trace_buffer = trace_buffer_i8; break;
        case 16:
            trace_buffer = trace_buffer_i16; break;
        case 32:
            trace_buffer = trace_buffer_i32; break;
    }
    void* buf_base = trace_buf_get_buffer_base(drcontext, trace_buffer);
    void* buf_ptr = trace_buf_get_buffer_ptr(drcontext, trace_buffer);
    per_thread_t* pt = (per_thread_t *)drmgr_get_tls_field(drcontext, tls_idx);
    cache_t<sz> *trace_base = (cache_t<sz> *)(char *)buf_base;
    cache_t<sz> *trace_ptr = (cache_t<sz> *)((char *)buf_ptr);
    cache_t<sz> *cache_ptr;
    IF_DEBUG(dr_fprintf(
        STDOUT,
        "UPDATE INT: trace_ptr=%p, base=%p, end=%p, size=%ld, buf_size=%ld, buf_end=%p\n",
        trace_ptr, trace_base,
        (char *)trace_base + trace_buf_get_buffer_size(drcontext, trace_buffer),
        trace_ptr - trace_base, trace_buf_get_buffer_size(drcontext, trace_buffer),
        trace_buf_get_buffer_end(drcontext, trace_buffer)));
    for(cache_ptr=trace_base; cache_ptr<trace_ptr; ++cache_ptr) {
        CheckAndInsertIntPage_impl<sz, esize>(cache_ptr->ctxt_hndl, (void*)cache_ptr->val, pt);
    }
    // all buffered trace is updated, reset the buffer
    trace_buf_set_buffer_ptr(drcontext, trace_buffer, buf_base);
}

template<int sz, int esize>
void trace_update_fp() {
    void* drcontext = dr_get_current_drcontext();
    // here we don't need to pass in the trace buffer pointer as we can statically know
    // which buffer will be updated at compile time.
    trace_buf_t* trace_buffer;
    switch (sz) {
        case 4:
            trace_buffer = trace_buffer_sp1; break;
        case 8:
            trace_buffer = trace_buffer_dp1; break;
        case 16:
            if(esize==4) {
                trace_buffer = trace_buffer_sp4; break;
            } else if(esize==8) {
                trace_buffer = trace_buffer_dp2; break;
            }
        case 32:
            if(esize==4) {
                trace_buffer = trace_buffer_sp8; break;
            } else if(esize==8) {
                trace_buffer = trace_buffer_dp4; break;
            }
    }
    void* buf_base = trace_buf_get_buffer_base(drcontext, trace_buffer);
    void* buf_ptr = trace_buf_get_buffer_ptr(drcontext, trace_buffer);
    per_thread_t* pt = (per_thread_t *)drmgr_get_tls_field(drcontext, tls_idx);
    cache_t<sz> *trace_base = (cache_t<sz> *)(char *)buf_base;
    cache_t<sz> *trace_ptr = (cache_t<sz> *)((char *)buf_ptr);
    cache_t<sz> *cache_ptr;
    IF_DEBUG(dr_fprintf(STDOUT,
                        "UPDATE FP: trace_ptr=%p, base=%p, size=%ld, buf_size=%ld\n",
                        trace_ptr, trace_base, trace_ptr - trace_base,
                        trace_buf_get_buffer_size(drcontext, trace_buffer)));
    for(cache_ptr=trace_base; cache_ptr<trace_ptr; ++cache_ptr) {
        AddFPRedLog<esize, sz>(cache_ptr->ctxt_hndl, (void*)cache_ptr->val, pt);
    }
    // all buffered trace is updated, reset the buffer
    trace_buf_set_buffer_ptr(drcontext, trace_buffer, buf_base);
}

template<int size, int esize, bool is_float>
void insertBufferCheck_impl(void* drcontext, instrlist_t *bb, instr_t* ins, ushort memRefCnt, trace_buf_t* trace_buffer, reg_id_t reg_ptr, reg_id_t reg_end) {
    // quick return without any instrumentation
    if(memRefCnt<=0) return ;
    instr_t* skip_update = INSTR_CREATE_label(drcontext);
    // when there may be overflow, we check and update the trace buffer if possible
    // the end buffer pointer
    trace_buf_insert_load_buf_end(drcontext, trace_buffer, bb, ins, reg_end);
    // current buffer pointer
    trace_buf_insert_load_buf_ptr(drcontext, trace_buffer, bb, ins, reg_ptr);
    // increament the buffer pointer with memRefCnt
    MINSERT(bb, ins,
            XINST_CREATE_add(drcontext, opnd_create_reg(reg_ptr),
                             OPND_CREATE_INT16(sizeof(cache_t<size>) * memRefCnt)));
    // if buffer will not be full, we will skip the heavy update
    MINSERT(bb, ins, XINST_CREATE_cmp(drcontext, opnd_create_reg(reg_ptr), opnd_create_reg(reg_end)));
    MINSERT(bb, ins, XINST_CREATE_jump_cond(drcontext, DR_PRED_LT, opnd_create_instr(skip_update)));
    // insert cleancall for updating the buffered trace
    if(is_float) {
        dr_insert_clean_call(drcontext, bb, ins, (void*)trace_update_fp<size, esize>, false, 0);
    } else {
        dr_insert_clean_call(drcontext, bb, ins, (void*)trace_update_int<size, esize>, false, 0);
    }
    // skip to here
    MINSERT(bb, ins, skip_update);
}

void insertBufferClear_impl(void* drcontext, instrlist_t *bb, instr_t* ins, ushort memRefCnt, trace_buf_t* trace_buffer, reg_id_t reg_ptr) {
    // quick return without any instrumentation
    if(memRefCnt<=0) return ;
    // current buffer pointer
    trace_buf_insert_clear_buf(drcontext, trace_buffer, bb, ins, reg_ptr);
}

void debug_output_line() {
    dr_fprintf(STDOUT, "--------------\n"); fflush(stdout);
}

void debug_output(void* val) {
    dr_fprintf(STDOUT, "loaded val=%p\n", val); fflush(stdout);
}

void insertBufferCheck(void* drcontext, instrlist_t *bb, instr_t* ins, ushort memRefCnt[12]) {
    bool willInsert = false;
    for(int i=0; i<12; ++i) willInsert = willInsert || (memRefCnt[i]>0);
    if(!willInsert) return ;
    reg_id_t reg_ptr, reg_end;
    // reserve registers if not dead
    RESERVE_AFLAGS(drcontext, bb, ins);
    RESERVE_REG(drcontext, bb, ins, NULL, reg_ptr);
#ifdef ARM_CCTLIB
    // for ARM, we always need two register for the check of bursty sampling.
    RESERVE_REG(drcontext, bb, ins, NULL, reg_end);
#endif
    instr_t* skip_to_end = INSTR_CREATE_label(drcontext);
    instr_t* skip_to_update = INSTR_CREATE_label(drcontext);
    if(op_enable_sampling.get_value()) {
        dr_insert_read_raw_tls(drcontext, bb, ins, tls_seg, tls_offs + INSTRACE_TLS_OFFS_BUF_PTR, reg_ptr);
        // Clear insCnt when insCnt > WINDOW_DISABLE
#ifdef ARM_CCTLIB
    #ifdef ARM64_CCTLIB
        minstr_load_wwint_to_reg(drcontext, bb, ins, reg_end, window_enable);
    #else
        minstr_load_wint_to_reg(drcontext, bb, ins, reg_end, window_enable);
    #endif
        MINSERT(bb, ins, XINST_CREATE_cmp(drcontext, opnd_create_reg(reg_ptr), opnd_create_reg(reg_end)));
#else
        MINSERT(bb, ins, XINST_CREATE_cmp(drcontext, opnd_create_reg(reg_ptr), OPND_CREATE_CCT_INT(window_enable)));
#endif
        MINSERT(bb, ins, XINST_CREATE_jump_cond(drcontext, DR_PRED_LE, opnd_create_instr(skip_to_update)));
        // FP
        insertBufferClear_impl(drcontext, bb, ins, memRefCnt[0], trace_buffer_sp1, reg_ptr);
        insertBufferClear_impl(drcontext, bb, ins, memRefCnt[1], trace_buffer_dp1, reg_ptr);
        insertBufferClear_impl(drcontext, bb, ins, memRefCnt[2], trace_buffer_sp4, reg_ptr);
        insertBufferClear_impl(drcontext, bb, ins, memRefCnt[3], trace_buffer_dp2, reg_ptr);
        insertBufferClear_impl(drcontext, bb, ins, memRefCnt[4], trace_buffer_sp8, reg_ptr);
        insertBufferClear_impl(drcontext, bb, ins, memRefCnt[5], trace_buffer_dp4, reg_ptr);
        // INT
        insertBufferClear_impl(drcontext, bb, ins, memRefCnt[6], trace_buffer_i1, reg_ptr);
        insertBufferClear_impl(drcontext, bb, ins, memRefCnt[7], trace_buffer_i2, reg_ptr);
        insertBufferClear_impl(drcontext, bb, ins, memRefCnt[8], trace_buffer_i4, reg_ptr);
        insertBufferClear_impl(drcontext, bb, ins, memRefCnt[9], trace_buffer_i8, reg_ptr);
        insertBufferClear_impl(drcontext, bb, ins, memRefCnt[10], trace_buffer_i16, reg_ptr);
        insertBufferClear_impl(drcontext, bb, ins, memRefCnt[11], trace_buffer_i32, reg_ptr);
        MINSERT(bb, ins, XINST_CREATE_jump(drcontext, opnd_create_instr(skip_to_end)));
        MINSERT(bb, ins, skip_to_update);
    }
#ifndef ARM_CCTLIB
    // for X86/64, we can lazily reserve the register to avoid unnecessary spilling.
    RESERVE_REG(drcontext, bb, ins, NULL, reg_end);
#endif
    // FP
    insertBufferCheck_impl<4,4, true>(drcontext, bb, ins, memRefCnt[0], trace_buffer_sp1, reg_ptr, reg_end);
    insertBufferCheck_impl<8,8, true>(drcontext, bb, ins, memRefCnt[1], trace_buffer_dp1, reg_ptr, reg_end);
    insertBufferCheck_impl<16,4, true>(drcontext, bb, ins, memRefCnt[2], trace_buffer_sp4, reg_ptr, reg_end);
    insertBufferCheck_impl<16,8, true>(drcontext, bb, ins, memRefCnt[3], trace_buffer_dp2, reg_ptr, reg_end);
    insertBufferCheck_impl<32,4, true>(drcontext, bb, ins, memRefCnt[4], trace_buffer_sp8, reg_ptr, reg_end);
    insertBufferCheck_impl<32,8, true>(drcontext, bb, ins, memRefCnt[5], trace_buffer_dp4, reg_ptr, reg_end);
    // INT
    insertBufferCheck_impl<1,1, false>(drcontext, bb, ins, memRefCnt[6], trace_buffer_i1, reg_ptr, reg_end);
    insertBufferCheck_impl<2,2, false>(drcontext, bb, ins, memRefCnt[7], trace_buffer_i2, reg_ptr, reg_end);
    insertBufferCheck_impl<4,4, false>(drcontext, bb, ins, memRefCnt[8], trace_buffer_i4, reg_ptr, reg_end);
    insertBufferCheck_impl<8,8, false>(drcontext, bb, ins, memRefCnt[9], trace_buffer_i8, reg_ptr, reg_end);
    insertBufferCheck_impl<16,1, false>(drcontext, bb, ins, memRefCnt[10], trace_buffer_i16, reg_ptr, reg_end);
    insertBufferCheck_impl<32,1, false>(drcontext, bb, ins, memRefCnt[11], trace_buffer_i32, reg_ptr, reg_end);
    // restore registers if reserved
#ifdef ARM_CCTLIB
    MINSERT(bb, ins, skip_to_end);
    UNRESERVE_REG(drcontext, bb, ins, reg_end);
#else
    UNRESERVE_REG(drcontext, bb, ins, reg_end);
    MINSERT(bb, ins, skip_to_end);
#endif
    UNRESERVE_REG(drcontext, bb, ins, reg_ptr);
    UNRESERVE_AFLAGS(drcontext, bb, ins);
}

void handleBufferUpdate() {
    // update int
    trace_update_int<1, 1>();
    trace_update_int<2, 2>();
    trace_update_int<4, 4>();
    trace_update_int<8, 8>();
    trace_update_int<16, 1>();
    trace_update_int<32, 1>();
    // update fp
    trace_update_fp<4, 4>();
    trace_update_fp<8, 8>();
    trace_update_fp<16, 4>();
    trace_update_fp<16, 8>();
    trace_update_fp<32, 4>();
    trace_update_fp<32, 8>();
}

int count_bb_info_detailed(void* drcontext, instrlist_t *bb, ushort memRefCnt[12]) {
    instr_t *instr;
    int num_instructions = 0;
    for(int i=0; i<12; ++i) memRefCnt[i] = 0;
    IF_DEBUG(dr_fprintf(gFile, "^^ INFO: Disassembled Instruction ^^^\n"));
    for (instr = instrlist_first(bb); instr != NULL; instr = instr_get_next(instr)) {
        if(!instr_is_app(instr)) continue;
        num_instructions++;
        if(     instr_reads_memory(instr) 
            && (!instr_is_prefetch(instr)) 
#ifdef X86
            && (!instr_is_gather(instr))
#endif
            && (!instr_is_ignorable(instr))) 
        {
            int num_srcs = instr_num_srcs(instr);
            for(int i=0; i<num_srcs; ++i) {
                opnd_t opnd = instr_get_src(instr, i);
                if(opnd_is_memory_reference(opnd)) {
                    uint32_t refSize = opnd_size_in_bytes(opnd_get_size(opnd));
                    if (refSize!=0) {
                        if(instr_is_floating(instr)) {
                            uint32_t operSize = FloatOperandSizeTable(instr, opnd);
                            switch (refSize) {
                                case 4:
                                    ++memRefCnt[0]; break;
                                case 8:
                                    ++memRefCnt[1]; break;
                                case 16:
                                    if(operSize==4) {
                                        ++memRefCnt[2]; break;
                                    } else if(operSize==8) {
                                        ++memRefCnt[3]; break;
                                    }
                                case 32:
                                    if(operSize==4) {
                                        ++memRefCnt[4]; break;
                                    } else if(operSize==8) {
                                        ++memRefCnt[5]; break;
                                    }
                            }
                        } else {
                            switch (refSize) {
                                case 1:
                                    ++memRefCnt[6]; break;
                                case 2:
                                    ++memRefCnt[7]; break;
                                case 4:
                                    ++memRefCnt[8]; break;
                                case 8:
                                    ++memRefCnt[9]; break;
                                case 16:
                                    ++memRefCnt[10]; break;
                                case 32:
                                    ++memRefCnt[11]; break;
                            }
                        }
                    }
                }
            }
            IF_DEBUG(dr_fprintf(gFile, "[memcnt=%d] ", memRefCnt));
        }
        IF_DEBUG(disassemble(drcontext, instr_get_app_pc(instr), gFile));
    }
    IF_DEBUG(dr_fprintf(gFile, "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n"));
    return num_instructions;
}

template<bool is_sampling>
static dr_emit_flags_t
event_basic_block(void *drcontext, void *tag, instrlist_t *bb, bool for_trace, bool translating, OUT void **user_data)
{
    int num_instructions = 0;
    ushort memRefCnt_detail[12];
    /* count the number of instructions in this block */
    num_instructions = count_bb_info_detailed(drcontext, bb, memRefCnt_detail);
    assert(num_instructions>=0);
    if(num_instructions!=0) {
        /* insert clean call */
        if(is_sampling) {
            ushort memRefCnt = memRefCnt_detail[0];
            for(int i=1; i<12; ++i) memRefCnt += memRefCnt_detail[i];
            if(memRefCnt) {
                dr_insert_clean_call(drcontext, bb, instrlist_first(bb), (void *)BBInstrument::BBUpdateAndCheck, false, 1, OPND_CREATE_CCT_INT(num_instructions));
            } else {
                dr_insert_clean_call(drcontext, bb, instrlist_first(bb), (void *)BBInstrument::BBUpdate, false, 1, OPND_CREATE_CCT_INT(num_instructions));
            }
        }
        instr_t* insert_pt = instrlist_first(bb);
        insertBufferCheck(drcontext, bb, insert_pt, memRefCnt_detail);
    }
    return DR_EMIT_DEFAULT;
}

static void
insert_load(void *drcontext, instrlist_t *ilist, instr_t *where, reg_id_t dst,
            reg_id_t src, ushort offset, opnd_size_t opsz)
{
    instr_t* ins_clone = instr_clone(drcontext, where);
    switch (opsz) {
    case OPSZ_1:
        MINSERT(ilist, where,
                XINST_CREATE_load_1byte(
                    drcontext, opnd_create_reg(reg_resize_to_opsz(dst, opsz)),
                    opnd_create_base_disp(src, DR_REG_NULL, 0, offset, opsz)));
        break;
    case OPSZ_2:
        MINSERT(ilist, where,
                XINST_CREATE_load_2bytes(
                    drcontext, opnd_create_reg(reg_resize_to_opsz(dst, opsz)),
                    opnd_create_base_disp(src, DR_REG_NULL, 0, offset, opsz)));
        break;
    case OPSZ_4:
#if defined(X86_64) || defined(AARCH64)
    case OPSZ_8:
#endif
        MINSERT(ilist, where,
                XINST_CREATE_load(drcontext,
                                  opnd_create_reg(reg_resize_to_opsz(dst, opsz)),
                                  opnd_create_base_disp(src, DR_REG_NULL, 0, offset, opsz)));
        break;
    default: DR_ASSERT(false); break;
    }
}

template<int size>
inline __attribute__((always_inline)) void insertStoreTraceBuffer(void *drcontext, instrlist_t *ilist, instr_t *where, int32_t slot, reg_id_t reg_addr, reg_id_t reg_ptr, trace_buf_t* trace_buffer)
{
    trace_buf_insert_load_buf_ptr(drcontext, trace_buffer, ilist, where, reg_ptr);
    switch(size) {
        case 1: {
            reg_id_t reg_val = reg_resize_to_opsz(reg_addr,OPSZ_1);
            insert_load(drcontext, ilist, where, reg_val, reg_addr, 0, OPSZ_1);
            trace_buf_insert_buf_store(drcontext, trace_buffer, ilist, where, reg_ptr, DR_REG_NULL,
                             opnd_create_reg(reg_val), OPSZ_1,
                             offsetof(cache_t<size>, val));
            break;
        }
        case 2: {
            reg_id_t reg_val = reg_resize_to_opsz(reg_addr,OPSZ_2);
            insert_load(drcontext, ilist, where, reg_val, reg_addr, 0, OPSZ_2);
            trace_buf_insert_buf_store(drcontext, trace_buffer, ilist, where, reg_ptr, DR_REG_NULL,
                             opnd_create_reg(reg_val), OPSZ_2,
                             offsetof(cache_t<size>, val));
            break;
        }
        case 4: {
            reg_id_t reg_val = reg_resize_to_opsz(reg_addr,OPSZ_4);
            insert_load(drcontext, ilist, where, reg_val, reg_addr, 0, OPSZ_4);
            trace_buf_insert_buf_store(drcontext, trace_buffer, ilist, where, reg_ptr, DR_REG_NULL,
                             opnd_create_reg(reg_val), OPSZ_4,
                             offsetof(cache_t<size>, val));
            break;
        }
        case 8: {
            insert_load(drcontext, ilist, where, reg_addr, reg_addr, 0, OPSZ_8);
            trace_buf_insert_buf_store(drcontext, trace_buffer, ilist, where, reg_ptr, DR_REG_NULL,
                             opnd_create_reg(reg_addr), OPSZ_8,
                             offsetof(cache_t<size>, val));
            break;
        }
        case 16: {
            reg_id_t scratch;
            RESERVE_REG(drcontext, ilist, where, NULL, scratch);
            // 0-7B
            insert_load(drcontext, ilist, where, scratch, reg_addr, 0, OPSZ_8);
            trace_buf_insert_buf_store(drcontext, trace_buffer, ilist, where, reg_ptr, DR_REG_NULL,
                             opnd_create_reg(scratch), OPSZ_8,
                             offsetof(cache_t<size>, val));
            // 8-15B
            insert_load(drcontext, ilist, where, scratch, reg_addr, 8, OPSZ_8);
            trace_buf_insert_buf_store(drcontext, trace_buffer, ilist, where, reg_ptr, DR_REG_NULL,
                             opnd_create_reg(scratch), OPSZ_8,
                             offsetof(cache_t<size>, val)+8);
            UNRESERVE_REG(drcontext, ilist, where, scratch);
            break;
        }
        case 32: {
            reg_id_t scratch;
            RESERVE_REG(drcontext, ilist, where, NULL, scratch);
            // 0-7B
            insert_load(drcontext, ilist, where, scratch, reg_addr, 0, OPSZ_8);
            trace_buf_insert_buf_store(drcontext, trace_buffer, ilist, where, reg_ptr, DR_REG_NULL,
                             opnd_create_reg(scratch), OPSZ_8,
                             offsetof(cache_t<size>, val));
            // 8-15B
            insert_load(drcontext, ilist, where, scratch, reg_addr, 8, OPSZ_8);
            trace_buf_insert_buf_store(drcontext, trace_buffer, ilist, where, reg_ptr, DR_REG_NULL,
                             opnd_create_reg(scratch), OPSZ_8,
                             offsetof(cache_t<size>, val)+8);
            // 16-23B
            insert_load(drcontext, ilist, where, scratch, reg_addr, 16, OPSZ_8);
            trace_buf_insert_buf_store(drcontext, trace_buffer, ilist, where, reg_ptr, DR_REG_NULL,
                             opnd_create_reg(scratch), OPSZ_8,
                             offsetof(cache_t<size>, val)+16);
            // 24-31B
            insert_load(drcontext, ilist, where, scratch, reg_addr, 24, OPSZ_8);
            trace_buf_insert_buf_store(drcontext, trace_buffer, ilist, where, reg_ptr, DR_REG_NULL,
                             opnd_create_reg(scratch), OPSZ_8,
                             offsetof(cache_t<size>, val)+24);
            UNRESERVE_REG(drcontext, ilist, where, scratch);
            break;
        }
    }
    drcctlib_get_context_handle_in_reg(drcontext, ilist, where, slot, reg_addr, reg_ptr);
    trace_buf_insert_load_buf_ptr(drcontext, trace_buffer, ilist, where, reg_ptr);
    trace_buf_insert_buf_store(drcontext, trace_buffer, ilist, where, reg_ptr, DR_REG_NULL,
                             opnd_create_reg(reg_64_to_32(reg_addr)), OPSZ_4, offsetof(cache_t<size>, ctxt_hndl));
    // update trace buffer
    trace_buf_insert_update_buf_ptr(drcontext, trace_buffer, ilist, where, reg_ptr,
                                  DR_REG_NULL, sizeof(cache_t<size>));
}

struct ZerospyInstrument{
    static __attribute__((always_inline)) void InstrumentForTracing(void *drcontext, instrlist_t *bb, instr_t *ins, opnd_t opnd, int32_t slot, reg_id_t addr_reg, reg_id_t scratch)
    {
        uint32_t refSize = opnd_size_in_bytes(opnd_get_size(opnd));
        if (refSize==0) {
            // Something strange happened, so ignore it
            return ;
        }
        // On ARM, we manually implement the instr_is_floating by estimating the usage of
        // the defined register of the load instruction.
        if (instr_is_floating(ins))
        {
            uint32_t operSize = FloatOperandSizeTable(ins, opnd);
            switch(refSize) {
                case 4:
                    insertStoreTraceBuffer<4>(drcontext, bb, ins, slot, addr_reg, scratch, trace_buffer_sp1);
                    break;
                case 8:
                    insertStoreTraceBuffer<8>(drcontext, bb, ins, slot, addr_reg, scratch, trace_buffer_dp1);
                    break;
                case 16:
                    if(operSize==4) {
                        insertStoreTraceBuffer<16>(drcontext, bb, ins, slot, addr_reg, scratch, trace_buffer_sp4);
                    } else if(operSize==8) {
                        insertStoreTraceBuffer<16>(drcontext, bb, ins, slot, addr_reg, scratch, trace_buffer_dp2);
                    }
                    break;
                case 32:
                    if(operSize==4) {
                        insertStoreTraceBuffer<32>(drcontext, bb, ins, slot, addr_reg, scratch, trace_buffer_sp8);
                    } else if(operSize==8) {
                        insertStoreTraceBuffer<32>(drcontext, bb, ins, slot, addr_reg, scratch, trace_buffer_dp4);
                    }
                    break;
                default:
                    assert(0 && "Unknown refSize\n");
            }
        } else {
            switch(refSize) {
                case 1:
                    insertStoreTraceBuffer<1>(drcontext, bb, ins, slot, addr_reg, scratch, trace_buffer_i1);
                    break;
                case 2:
                    insertStoreTraceBuffer<2>(drcontext, bb, ins, slot, addr_reg, scratch, trace_buffer_i2);
                    break;
                case 4:
                    insertStoreTraceBuffer<4>(drcontext, bb, ins, slot, addr_reg, scratch, trace_buffer_i4);
                    break;
                case 8:
                    insertStoreTraceBuffer<8>(drcontext, bb, ins, slot, addr_reg, scratch, trace_buffer_i8);
                    break;
                case 16:
                    insertStoreTraceBuffer<16>(drcontext, bb, ins, slot, addr_reg, scratch, trace_buffer_i16);
                    break;
                case 32:
                    insertStoreTraceBuffer<32>(drcontext, bb, ins, slot, addr_reg, scratch, trace_buffer_i32);
                    break;
                default:
                    assert(0 && "Unknown refSize\n");
            }
        }
    }

    static __attribute__((always_inline)) void InstrumentReadValueBeforeAndAfterLoading(void *drcontext, instrlist_t *bb, instr_t *ins, opnd_t opnd, int32_t slot, reg_id_t addr_reg, reg_id_t scratch)
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
        if (instr_is_floating(ins))
        {
            uint32_t operSize = FloatOperandSizeTable(ins, opnd);
            if(operSize>0) {
                insertCheckAndUpdateFp<false>(drcontext, bb, ins, refSize, operSize, slot, addr_reg, scratch);
            }
        } else {
            insertCheckAndUpdateInt<false>(drcontext, bb, ins, refSize, refSize>8?1:refSize, slot, addr_reg, scratch);
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
                printf("\nERROR: refSize for floating point instruction is too small: %d!\n", refSize);
                printf("^^ Disassembled Instruction ^^^\n");
                disassemble(drcontext, instr_get_app_pc(ins), 1/*sdtout file desc*/);
                printf("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
                fflush(stdout);
                assert(0 && "memory read floating data with unexptected small size");
            case 16: {
                switch (operSize) {
                    case 4: HANDLE_VGATHER(float, 16, 4, true); break;
                    case 8: HANDLE_VGATHER(double, 16, 8, true); break;
                    default: assert(0 && "handle large mem read with unexpected operand size\n"); break;
                }
            }break;
            case 32: {
                switch (operSize) {
                    case 4: HANDLE_VGATHER(float, 32, 4, true); break;
                    case 8: HANDLE_VGATHER(double, 32, 8, true); break;
                    default: assert(0 && "handle large mem read with unexpected operand size\n"); break;
                }
            }break;
            default: 
                printf("\nERROR: refSize for floating point instruction is too large: %d!\n", refSize);
                printf("^^ Disassembled Instruction ^^^\n");
                disassemble(drcontext, instr_get_app_pc(ins), 1/*sdtout file desc*/);
                printf("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
                fflush(stdout);
                assert(0 && "unexpected large memory read\n"); break;
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
InstrumentMem(void *drcontext, instrlist_t *ilist, instr_t *where, opnd_t ref, int32_t slot, reg_id_t reg_addr, reg_id_t scratch, bool need_restore_0, bool need_restore_1)
{
    if (need_restore_0) {
        if (opnd_uses_reg(ref, reg_addr) &&
            drreg_get_app_value(drcontext, ilist, where, reg_addr, reg_addr) != DRREG_SUCCESS) {
            ZEROSPY_EXIT_PROCESS("InstrumentMemAll drreg_get_app_value reg_addr failed!");
        }
    }
    if (need_restore_1) {
        if (opnd_uses_reg(ref, scratch) &&
            drreg_get_app_value(drcontext, ilist, where, scratch, scratch) != DRREG_SUCCESS) {
            ZEROSPY_EXIT_PROCESS("InstrumentMemAll drreg_get_app_value scratch failed!");
        }
    }
    if (!drutil_insert_get_mem_addr(drcontext, ilist, where, ref, reg_addr/*addr*/,
                                    scratch/*scratch*/)) {
        dr_fprintf(STDERR, "\nERROR: drutil_insert_get_mem_addr failed!\n");
        dr_fprintf(STDERR, "^^ Disassembled Instruction ^^^\n");
        disassemble(drcontext, instr_get_app_pc(where), STDERR);
        dr_fprintf(STDERR, "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
        ZEROSPY_EXIT_PROCESS("InstrumentMem drutil_insert_get_mem_addr failed!");
    } else {
#ifdef USE_CLEANCALL
        ZerospyInstrument::InstrumentReadValueBeforeAndAfterLoading(drcontext, ilist, where, ref, slot, reg_addr, scratch);
#else
        ZerospyInstrument::InstrumentForTracing(drcontext, ilist, where, ref, slot, reg_addr, scratch);
#endif
    }
}

void InstrumentMemAll(void* drcontext, instrlist_t *bb, instr_t* instr, int32_t slot)
{
    reg_id_t reg_addr, scratch, reg_xchg=DR_REG_NULL;
    RESERVE_REG(drcontext, bb, instr, NULL, reg_addr);
    RESERVE_REG(drcontext, bb, instr, NULL, scratch);
    int num_srcs = instr_num_srcs(instr);
    int memop = 0;
    for(int i=0; i<num_srcs; ++i) {
        opnd_t opnd = instr_get_src(instr, i);
        if(opnd_is_memory_reference(opnd)) {
            bool need_restore_1 = memop>0;
            bool need_restore_0 = op_enable_sampling.get_value() || need_restore_1;
            InstrumentMem(drcontext, bb, instr, opnd, slot, reg_addr, scratch, need_restore_0, need_restore_1);
            ++memop;
        }
    }
    UNRESERVE_REG(drcontext, bb, instr, scratch);
    UNRESERVE_REG(drcontext, bb, instr, reg_addr);
}

#ifdef X86
void InstrumentVGather(void* drcontext, instrlist_t *bb, instr_t* instr, int32_t slot)
{
    // We use instr_compute_address_ex_pos to handle gather (with VSIB addressing)
    ZerospyInstrument::InstrumentReadValueBeforeVGather(drcontext, bb, instr, slot);
}
#endif

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

#ifdef X86
    if(instr_is_gather(instr)) {
        InstrumentVGather(drcontext, bb, instr, slot);
    } else
#endif
    {
        InstrumentMemAll(drcontext, bb, instr, slot);
    }
}

static void
ThreadOutputFileInit(per_thread_t *pt)
{
    int32_t id = drcctlib_get_thread_id();
    pt->threadId = id;
    char name[MAXIMUM_PATH] = "";
    sprintf(name + strlen(name), "%s/thread-%d.topn.log", g_folder_name.c_str(), id);
    pt->output_file = dr_open_file(name, DR_FILE_WRITE_OVERWRITE | DR_FILE_ALLOW_LARGE);
    DR_ASSERT(pt->output_file != INVALID_FILE);
    if (op_enable_sampling.get_value()) {
        dr_fprintf(pt->output_file, "[ZEROSPY INFO] Sampling Enabled\n");
    } else {
        dr_fprintf(pt->output_file, "[ZEROSPY INFO] Sampling Disabled\n");
    }
}

#define DEBUG_PRINT_TB_INFO(tb) \
    IF_DEBUG(dr_fprintf(STDOUT, #tb ": buf_ptr=%p, buf_base=%p, buf_end=%p, buf_size=%ld\n", \
        trace_buf_get_buffer_ptr(drcontext, tb), \
        trace_buf_get_buffer_base(drcontext, tb), \
        trace_buf_get_buffer_end(drcontext, tb), \
        trace_buf_get_buffer_size(drcontext, tb)))

static void
ClientThreadStart(void *drcontext)
{
    // assert(dr_get_itimer(ITIMER_REAL));
    per_thread_t *pt = (per_thread_t *)dr_thread_alloc(drcontext, sizeof(per_thread_t));
    if (pt == NULL) {
        ZEROSPY_EXIT_PROCESS("pt == NULL");
    }
    pt->INTRedLogMap = new INTRedLogMap_t();
    pt->FPRedLogMap = new FPRedLogMap_t();
    // pt->FPRedLogMap->rehash(10000000);
    // pt->INTRedLogMap->rehash(10000000);
    pt->instr_clones = new vector<instr_t*>();
    pt->numInsBuff = dr_get_dr_segment_base(tls_seg);
    BUF_PTR(pt->numInsBuff, void, INSTRACE_TLS_OFFS_BUF_PTR) = 0;
    drmgr_set_tls_field(drcontext, tls_idx, (void *)pt);
    // init output files
    ThreadOutputFileInit(pt);
    // check trace
    DEBUG_PRINT_TB_INFO(trace_buffer_i1);
    DEBUG_PRINT_TB_INFO(trace_buffer_i2);
    DEBUG_PRINT_TB_INFO(trace_buffer_i4);
    DEBUG_PRINT_TB_INFO(trace_buffer_i8);
    DEBUG_PRINT_TB_INFO(trace_buffer_i16);
    DEBUG_PRINT_TB_INFO(trace_buffer_i32);
    DEBUG_PRINT_TB_INFO(trace_buffer_sp1);
    DEBUG_PRINT_TB_INFO(trace_buffer_sp4);
    DEBUG_PRINT_TB_INFO(trace_buffer_sp8);
    DEBUG_PRINT_TB_INFO(trace_buffer_dp1);
    DEBUG_PRINT_TB_INFO(trace_buffer_dp2);
    DEBUG_PRINT_TB_INFO(trace_buffer_dp4);
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

static uint64_t PrintRedundancyPairs(per_thread_t *pt, uint64_t threadBytesLoad, int threadId, rapidjson::Value &threadDetailedMetrics, rapidjson::Value &threadDetailedCodeCentricMetrics) 
{
    vector<RedundacyData> tmpList;
    vector<RedundacyData>::iterator tmpIt;
    rapidjson::Value integerRedundantInfo(rapidjson::kArrayType);

    file_t gTraceFile = pt->output_file;
    
    uint64_t grandTotalRedundantBytes = 0;
    tmpList.reserve(pt->INTRedLogMap->size());
    dr_fprintf(STDOUT, "Dumping INTEGER Redundancy Info...\n");
    uint64_t count = 0, rep = -1, num = 0;
    for(auto it=pt->INTRedLogMap->begin(); it!=pt->INTRedLogMap->end(); ++it) {
        ++count;
        if(100 * count / (pt->INTRedLogMap->size())!=rep) {
            rep = 100 * count / (pt->INTRedLogMap->size());
            dr_fprintf(STDOUT, "\r%ld%%  Finish",rep); fflush(stdout);
        }
        uint64_t tot = DECODE_SRC(it->second.tot);
        uint8_t accessLen = DECODE_UPPER(it->second.tot);
        RedundacyData tmp = { it->first, it->second.red,        it->second.fred,
                              tot,       it->second.redByteMap, accessLen };
        tmpList.push_back(tmp);
        grandTotalRedundantBytes += tmp.frequency;
        ++num;
    }
    dr_fprintf(STDOUT, "\r100%%  Finish, Total num = %ld\n", count); fflush(stdout);
    if(count == 0) {
        dr_fprintf(STDOUT, "Warning: No valid profiling data is logged!\n");
        return 0;
    }

    dr_fprintf(gTraceFile, "\n--------------- Dumping INTEGER Redundancy Info ----------------\n");
    dr_fprintf(gTraceFile, "\n*************** Dump Data from Thread %d ****************\n", threadId);
    
    __sync_fetch_and_add(&grandTotBytesRedLoad,grandTotalRedundantBytes);
    dr_fprintf(STDOUT, "Extracted Raw data, now sorting...\n"); fflush(stdout);
    
    dr_fprintf(gTraceFile, "\n Total redundant bytes = %f %%\n", grandTotalRedundantBytes * 100.0 / threadBytesLoad);
    dr_fprintf(gTraceFile, "\n INFO : Total redundant bytes = %f %% (%ld / %ld) \n", grandTotalRedundantBytes * 100.0 / threadBytesLoad, grandTotalRedundantBytes, threadBytesLoad);

    if(grandTotalRedundantBytes==0) {
        dr_fprintf(gTraceFile, "\n------------ Dumping INTEGER Redundancy Info Finish -------------\n");
        dr_fprintf(STDOUT, "INTEGER Report dumped\n"); fflush(stdout);
        return grandTotalRedundantBytes;
    }
    
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
    dr_fprintf(STDOUT, "Sorted, Now generating reports...\n"); fflush(stdout);
    int cntxtNum = 0;
    for (vector<RedundacyData>::iterator listIt = tmpList.begin(); listIt != tmpList.end(); ++listIt) {
        if (cntxtNum < MAX_REDUNDANT_CONTEXTS_TO_LOG) {
            rapidjson::Value integerRedundantInfoItem(rapidjson::kObjectType);
            dr_fprintf(gTraceFile, "\n\n======= (%f) %% of total Redundant, with local redundant %f %% (%ld Bytes / %ld Bytes) ======\n", 
                (*listIt).frequency * 100.0 / grandTotalRedundantBytes,
                (*listIt).frequency * 100.0 / (*listIt).ltot,
                (*listIt).frequency,(*listIt).ltot);
            integerRedundantInfoItem.AddMember("Redundancy", (*listIt).frequency * 100.0 / grandTotalRedundantBytes, jsonAllocator);
            integerRedundantInfoItem.AddMember("local redundancy", rapidjson::Value((std::to_string((*listIt).frequency * 100.0 / (*listIt).ltot) + "% (" + std::to_string((*listIt).frequency) + " Bytes / " + std::to_string((*listIt).ltot) + " Bytes)").c_str(), jsonAllocator), jsonAllocator);

            dr_fprintf(gTraceFile, "\n\n======= with All Zero Redundant %f %% (%ld / %ld) ======\n", 
                (*listIt).all0freq * (*listIt).accessLen * 100.0 / (*listIt).ltot,
                (*listIt).all0freq,(*listIt).ltot/(*listIt).accessLen);
            integerRedundantInfoItem.AddMember("Fully Redundant Zero", rapidjson::Value((std::to_string((*listIt).all0freq * (*listIt).accessLen * 100.0 / (*listIt).ltot) + "% (" + std::to_string((*listIt).all0freq) + " / " + std::to_string((*listIt).ltot/(*listIt).accessLen) + ")").c_str(), jsonAllocator), jsonAllocator);

            dr_fprintf(gTraceFile, "\n======= Redundant byte map : [0] ");
            std::string redMap;
            for(uint32_t i=0;i<(*listIt).accessLen;++i) {
                if((*listIt).byteMap & (1<<i)) {
                    dr_fprintf(gTraceFile, "00 ");
                    redMap.append("00 ");
                }
                else {
                    dr_fprintf(gTraceFile, "XX ");
                    redMap.append("XX ");
                }
            }
            dr_fprintf(gTraceFile, " [AccessLen=%d] =======\n", (*listIt).accessLen);
            integerRedundantInfoItem.AddMember("Redmap", rapidjson::Value(("[0]" + redMap + "[" + std::to_string((*listIt).accessLen) + "]").c_str(), jsonAllocator), jsonAllocator);

            dr_fprintf(gTraceFile, "\n---------------------Redundant load with---------------------------\n");
            drcctlib_print_full_cct(gTraceFile, (*listIt).cntxt, true, true, MAX_DEPTH);
            std::string cctInfo = drcctlib_get_full_cct_string((*listIt).cntxt, true, true, MAX_DEPTH);
            integerRedundantInfoItem.AddMember("CCT Info", rapidjson::Value(cctInfo.c_str(), jsonAllocator), jsonAllocator);

            integerRedundantInfo.PushBack(integerRedundantInfoItem, jsonAllocator);
        }
        else {
            break;
        }
        cntxtNum++;
    }
    dr_fprintf(gTraceFile, "\n------------ Dumping INTEGER Redundancy Info Finish -------------\n");
    dr_fprintf(STDOUT, "INTEGER Report dumped\n"); fflush(stdout);

    threadDetailedCodeCentricMetrics.AddMember("Integer Redundant Info", integerRedundantInfo, jsonAllocator);
    return grandTotalRedundantBytes;
}

static uint64_t PrintApproximationRedundancyPairs(per_thread_t *pt, uint64_t threadBytesLoad, int threadId, rapidjson::Value &threadDetailedMetrics, rapidjson::Value &threadDetailedCodeCentricMetrics) 
{
    vector<ApproxRedundacyData> tmpList;
    vector<ApproxRedundacyData>::iterator tmpIt;
    tmpList.reserve(pt->FPRedLogMap->size());
    rapidjson::Value floatingPointRedundantInfo(rapidjson::kArrayType);
    
    file_t gTraceFile = pt->output_file;
    uint64_t grandTotalRedundantBytes = 0;

    dr_fprintf(STDOUT, "Dumping FLOATING POINT Redundancy Info...\n");
    uint64_t count = 0, rep = -1, num = 0;
    for (auto it=pt->FPRedLogMap->begin(); it!=pt->FPRedLogMap->end(); ++it) {
        ++count;
        if(100 * count / (pt->FPRedLogMap->size())!=rep) {
            rep = 100 * count / (pt->FPRedLogMap->size());
            dr_fprintf(STDOUT, "\r%ld%%  Finish",rep); fflush(stdout);
        }
        uint8_t accessLen = DECODE_UPPER(it->second.ftot);
        uint8_t elementSize = DECODE_UPPER(it->second.fred);
        uint64_t redByteMap = it->second.redByteMap;
        ApproxRedundacyData tmp = { it->first,
                                    DECODE_SRC(it->second.fred),
                                    DECODE_SRC(it->second.ftot),
                                    redByteMap,
                                    accessLen,
                                    elementSize };
        tmpList.push_back(tmp);
        grandTotalRedundantBytes += DECODE_SRC(it->second.fred) * accessLen;
        ++num;
    }
    dr_fprintf(STDOUT, "\r100%%  Finish, Total num = %ld\n", count); fflush(stdout);
    if(count == 0) {
        dr_fprintf(STDOUT, "Warning: No valid profiling data is logged!\n");
        return 0;
    }

    dr_fprintf(gTraceFile, "\n--------------- Dumping Approximation Redundancy Info ----------------\n");
    dr_fprintf(gTraceFile, "\n*************** Dump Data(delta=%.2f%%) from Thread %d ****************\n", delta*100,threadId);
    
    __sync_fetch_and_add(&grandTotBytesApproxRedLoad,grandTotalRedundantBytes);
    
    dr_fprintf(gTraceFile, "\n Total redundant bytes = %f %%\n", grandTotalRedundantBytes * 100.0 / threadBytesLoad);
    dr_fprintf(gTraceFile, "\n INFO : Total redundant bytes = %f %% (%ld / %ld) \n", grandTotalRedundantBytes * 100.0 / threadBytesLoad, grandTotalRedundantBytes, threadBytesLoad);

    if(grandTotalRedundantBytes==0) {
        dr_fprintf(gTraceFile, "\n------------ Dumping Approximation Redundancy Info Finish -------------\n");
        printf("Floating Point Report dumped\n");
        return 0;
    }
    
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
            rapidjson::Value floatRedundantInfoItem(rapidjson::kObjectType);
            dr_fprintf(gTraceFile, "\n======= (%f) %% of total Redundant, with local redundant %f %% (%ld Zeros / %ld Reads) ======\n",
                (*listIt).all0freq * 100.0 / grandTotalRedundantBytes,
                (*listIt).all0freq * 100.0 / (*listIt).ftot,
                (*listIt).all0freq,(*listIt).ftot);
            float totalRedundancy = grandTotalRedundantBytes == 0 ? 0 : (*listIt).all0freq * 100.0 / grandTotalRedundantBytes;
            floatRedundantInfoItem.AddMember("Redundancy", totalRedundancy, jsonAllocator);
            floatRedundantInfoItem.AddMember("local redundancy", rapidjson::Value((std::to_string((*listIt).all0freq * 100.0 / (*listIt).ftot) + "% (" + std::to_string((*listIt).all0freq) + " Zeros / " + std::to_string((*listIt).ftot) + " Reads)").c_str(), jsonAllocator), jsonAllocator);
            dr_fprintf(gTraceFile, "\n======= Redundant byte map : [ sign | exponent | mantissa ] ========\n");
            if((*listIt).size==4) {
                dr_fprintf(gTraceFile, "%s", getFpRedMapString_SP((*listIt).byteMap, (*listIt).accessLen/4).c_str());
                floatRedundantInfoItem.AddMember("Redmap: [mantissa | exponent | sign]", rapidjson::Value(getFpRedMapString_SP((*listIt).byteMap, (*listIt).accessLen/4).c_str(), jsonAllocator), jsonAllocator);
            } else {
                dr_fprintf(gTraceFile, "%s", getFpRedMapString_DP((*listIt).byteMap, (*listIt).accessLen/8).c_str());
                floatRedundantInfoItem.AddMember("Redmap: [mantissa | exponent | sign]", rapidjson::Value(getFpRedMapString_DP((*listIt).byteMap, (*listIt).accessLen/8).c_str(), jsonAllocator), jsonAllocator);
            }
            dr_fprintf(gTraceFile, "\n===== [AccessLen=%d, typesize=%d] =======\n", (*listIt).accessLen, (*listIt).size);
            floatRedundantInfoItem.AddMember("AccessLen", (*listIt).accessLen, jsonAllocator);
            floatRedundantInfoItem.AddMember("typesize", (*listIt).size, jsonAllocator);
            dr_fprintf(gTraceFile, "\n---------------------Redundant load with---------------------------\n");
            drcctlib_print_full_cct(gTraceFile, (*listIt).cntxt, true, true, MAX_DEPTH);
            std::string cctInfo = drcctlib_get_full_cct_string((*listIt).cntxt, true, true, MAX_DEPTH);
            floatRedundantInfoItem.AddMember("CCT Info", rapidjson::Value(cctInfo.c_str(), jsonAllocator), jsonAllocator);

            floatingPointRedundantInfo.PushBack(floatRedundantInfoItem, jsonAllocator);
        }
        else {
            break;
        }
        cntxtNum++;
    }
    dr_fprintf(gTraceFile, "\n------------ Dumping Approximation Redundancy Info Finish -------------\n");
    printf("Floating Point Report dumped\n");

    threadDetailedCodeCentricMetrics.AddMember("Floating Point Redundant Info", floatingPointRedundantInfo, jsonAllocator);
    threadDetailedMetrics.AddMember("Code Centric", threadDetailedCodeCentricMetrics, jsonAllocator);
    threadDetailedMetricsMap[threadId] = threadDetailedMetrics;
    fflush(stdout);
    return grandTotalRedundantBytes;
}
/*******************************************************************/
static uint64_t getThreadByteLoad(per_thread_t *pt) {
    register uint64_t x = 0;
    for (auto it=pt->INTRedLogMap->begin(); it!=pt->INTRedLogMap->end(); ++it) {
        x += DECODE_SRC(it->second.tot);
    }
    for (auto it=pt->FPRedLogMap->begin(); it!=pt->FPRedLogMap->end(); ++it) {
        x += DECODE_SRC(it->second.ftot) * DECODE_UPPER(it->second.ftot);
    }
    return x;
}
/*******************************************************************/
static void
ClientThreadEndFortrace(void *drcontext) {
    handleBufferUpdate();
}

static void
ClientThreadEnd(void *drcontext)
{
#ifdef TIMING
    uint64_t time = get_miliseconds();
#endif
    per_thread_t *pt = (per_thread_t *)drmgr_get_tls_field(drcontext, tls_idx);
    uint64_t threadByteLoad = getThreadByteLoad(pt);
    if(threadByteLoad!=0) 
    {
        __sync_fetch_and_add(&grandTotBytesLoad,threadByteLoad);
        int32_t threadId = pt->threadId;
        // int32_t threadId = dr_get_thread_id(drcontext);
        rapidjson::Value threadDetailedMetrics(rapidjson::kObjectType);
        rapidjson::Value threadDetailedCodeCentricMetrics(rapidjson::kObjectType);
        // uint64_t threadRedByteLoadFP = PrintApproximationRedundancyPairs(pt, threadByteLoad, threadId, threadDetailedMetrics, threadDetailedCodeCentricMetrics);
        uint64_t threadRedByteLoadINT = PrintRedundancyPairs(pt, threadByteLoad, threadId, threadDetailedMetrics, threadDetailedCodeCentricMetrics);
        uint64_t threadRedByteLoadFP = PrintApproximationRedundancyPairs(pt, threadByteLoad, threadId, threadDetailedMetrics, threadDetailedCodeCentricMetrics);
#ifdef TIMING
        time = get_miliseconds() - time;
        printf("Thread %d: Time %ld ms for generating outputs\n", threadId, time);
#endif

        dr_mutex_lock(gLock);
        dr_fprintf(gFile, "\n#THREAD %d Redundant Read:", threadId);
        dr_fprintf(gFile, "\nTotalBytesLoad: %lu ",threadByteLoad);
        dr_fprintf(gFile, "\nRedundantBytesLoad: %lu %.2f",threadRedByteLoadINT, threadRedByteLoadINT * 100.0/threadByteLoad);
        dr_fprintf(gFile, "\nApproxRedundantBytesLoad: %lu %.2f\n",threadRedByteLoadFP, threadRedByteLoadFP * 100.0/threadByteLoad);

        rapidjson::Value threadIntegerTotal(rapidjson::kObjectType);
        rapidjson::Value threadFloatTotal(rapidjson::kObjectType);

        threadIntegerTotal.AddMember("rate", threadRedByteLoadINT * 100.0/threadByteLoad, jsonAllocator);
        threadIntegerTotal.AddMember("fraction", rapidjson::Value((std::to_string(threadRedByteLoadINT) + "/" + std::to_string(threadByteLoad)).c_str(), jsonAllocator), jsonAllocator);
        threadIntegerTotal.AddMember("detail", "file:///abspath/to/thread0CCDetail.md",
                        jsonAllocator);

        threadFloatTotal.AddMember("rate", threadRedByteLoadFP * 100.0/threadByteLoad, jsonAllocator);
        threadFloatTotal.AddMember("fraction", rapidjson::Value((std::to_string(threadRedByteLoadFP) + "/" + std::to_string(threadByteLoad)).c_str(), jsonAllocator), jsonAllocator);
        threadFloatTotal.AddMember("detail", "file:///abspath/to/thread0CCDetail.md",
                        jsonAllocator);

        totalIntegerRedundantBytes.AddMember(rapidjson::Value(("Thread " + std::to_string(threadId)).c_str(), jsonAllocator), threadIntegerTotal, jsonAllocator);
        totalFloatRedundantBytes.AddMember(rapidjson::Value(("Thread " + std::to_string(threadId)).c_str(), jsonAllocator), threadFloatTotal, jsonAllocator);

        dr_mutex_unlock(gLock);
    }
    dr_close_file(pt->output_file);
    for(size_t i=0;i<pt->instr_clones->size();++i) {
        instr_destroy(drcontext, (*pt->instr_clones)[i]);
    }
    delete pt->instr_clones;
    delete pt->INTRedLogMap;
    delete pt->FPRedLogMap;
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

    gDoc.SetObject();

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
    char gJsonName[MAXIMUM_PATH] = "";
    sprintf(gJsonName + strlen(gJsonName), "%s/report.json", g_folder_name.c_str());
    gJson = fopen(gJsonName, "w");
    DR_ASSERT(gFile != INVALID_FILE);
    DR_ASSERT(gJson != NULL);
    if (op_enable_sampling.get_value()) {
        dr_fprintf(STDOUT, "[ZEROSPY INFO] Sampling Enabled\n");
        dr_fprintf(gFile, "[ZEROSPY INFO] Sampling Enabled\n");
        window_enable = op_window_enable.get_value();
        window_disable= op_window.get_value();
        float rate = (float)window_enable / (float)window_disable;
        dr_fprintf(STDOUT, "[ZEROSPY INFO] Sampling Rate: %.3f, Window Size: %ld\n", rate, window_disable);
        dr_fprintf(gFile,  "[ZEROSPY INFO] Sampling Rate: %.3f, Window Size: %ld\n", rate, window_disable);
    } else {
        dr_fprintf(STDOUT, "[ZEROSPY INFO] Sampling Disabled\n");
        dr_fprintf(gFile, "[ZEROSPY INFO] Sampling Disabled\n");
    }
    if (dr_using_all_private_caches()) {
        dr_fprintf(STDOUT, "[ZEROSPY INFO] Thread Private is enabled.\n");
        dr_fprintf(gFile,  "[ZEROSPY INFO] Thread Private is enabled.\n");
    } else {
        dr_fprintf(STDOUT, "[ZEROSPY INFO] Thread Private is disabled.\n");
        dr_fprintf(gFile,  "[ZEROSPY INFO] Thread Private is disabled.\n");
    }
    if (op_help.get_value()) {
        dr_fprintf(STDOUT, "%s\n", droption_parser_t::usage_long(DROPTION_SCOPE_CLIENT).c_str());
        exit(1);
    }
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

    totalIntegerRedundantBytes.AddMember("rate", grandTotBytesRedLoad * 100.0/grandTotBytesLoad, jsonAllocator);
    totalIntegerRedundantBytes.AddMember("fraction", rapidjson::Value((std::to_string(grandTotBytesRedLoad) + "/" + std::to_string(grandTotBytesLoad)).c_str(), jsonAllocator), jsonAllocator);

    totalFloatRedundantBytes.AddMember("rate", grandTotBytesApproxRedLoad * 100.0/grandTotBytesLoad, jsonAllocator);
    totalFloatRedundantBytes.AddMember("fraction", rapidjson::Value((std::to_string(grandTotBytesApproxRedLoad) + "/" + std::to_string(grandTotBytesLoad)).c_str(), jsonAllocator), jsonAllocator);

    metricOverview.AddMember("Total Integer Redundant Bytes", totalIntegerRedundantBytes, jsonAllocator);
    metricOverview.AddMember("Total Floating Point Redundant Bytes", totalFloatRedundantBytes, jsonAllocator);
    metricOverview.AddMember("Thread Num", threadDetailedMetricsMap.size(), jsonAllocator);
    gDoc.AddMember("Metric Overview", metricOverview, jsonAllocator);

    for(auto &threadMetrics : threadDetailedMetricsMap){
        gDoc.AddMember(rapidjson::Value(("Thread " + std::to_string(threadMetrics.first) + " Detailed Metrics").c_str(), jsonAllocator), threadMetrics.second, jsonAllocator);
    }

    char writeBuffer[127];
    rapidjson::FileWriteStream os(gJson, writeBuffer, sizeof(writeBuffer));

    rapidjson::PrettyWriter<rapidjson::FileWriteStream> writer(os);
    gDoc.Accept(writer);
#ifndef _WERROR
    if(warned) {
        dr_fprintf(gFile, "####################################\n");
        dr_fprintf(gFile, "WARNING: some unexpected instructions are ignored. Please check zerospy.log.warn for detail.\n");
        dr_fprintf(gFile, "####################################\n");
    }
#endif
    dr_close_file(gFlagF);
    dr_close_file(gFile);
    fclose(gJson);
    if (!dr_raw_tls_cfree(tls_offs, INSTRACE_TLS_COUNT)) {
        ZEROSPY_EXIT_PROCESS(
            "ERROR: zerospy dr_raw_tls_calloc fail");
    }
    // free trace buffers
    trace_buf_free(trace_buffer_i1);
    trace_buf_free(trace_buffer_i2);
    trace_buf_free(trace_buffer_i4);
    trace_buf_free(trace_buffer_i8);
    trace_buf_free(trace_buffer_i16);
    trace_buf_free(trace_buffer_i32);
    trace_buf_free(trace_buffer_sp1);
    trace_buf_free(trace_buffer_dp1);
    trace_buf_free(trace_buffer_sp4);
    trace_buf_free(trace_buffer_dp2);
    trace_buf_free(trace_buffer_sp8);
    trace_buf_free(trace_buffer_dp4);

    dr_mutex_destroy(gLock);
    drcctlib_exit();
    drmgr_analysis_cb_t event_basic_block_ptr = op_enable_sampling.get_value()
        ? event_basic_block<true>
        : event_basic_block<false>;
    if (!drmgr_unregister_thread_init_event(ClientThreadStart) ||
        !drmgr_unregister_thread_exit_event(ClientThreadEnd) ||
        !drmgr_unregister_thread_exit_event(ClientThreadEndFortrace) ||
        // must unregister event after client exit, or it will cause unexpected errors during execution
        !drmgr_unregister_bb_instrumentation_event(event_basic_block_ptr) ||
        !drmgr_unregister_tls_field(tls_idx)) {
        printf("ERROR: zerospy failed to unregister in ClientExit");
        fflush(stdout);
        exit(-1);
    }
    drutil_exit();
    drreg_exit();
    trace_exit();
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

    if (!drmgr_init() || !trace_init()) {
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
                                         "zerospy-thread-exit", NULL, NULL,
                                         DRCCTLIB_THREAD_EVENT_PRI - 1 };
    drmgr_priority_t thread_exit_high_pri = { sizeof(thread_exit_pri),
                                         "zerospy-thread-exit-trace", NULL, NULL,
                                         DRMGR_PRIORITY_THREAD_EXIT_TRACE_BUF - 1 };
    drmgr_analysis_cb_t event_basic_block_ptr = op_enable_sampling.get_value()
        ? event_basic_block<true>
        : event_basic_block<false>;
    if (   !drmgr_register_bb_instrumentation_event(event_basic_block_ptr, NULL, NULL)
        || !drmgr_register_thread_init_event_ex(ClientThreadStart, &thread_init_pri) 
        || !drmgr_register_thread_exit_event_ex(ClientThreadEnd, &thread_exit_pri)
        || !drmgr_register_thread_exit_event_ex(ClientThreadEndFortrace, &thread_exit_high_pri) ) {
        ZEROSPY_EXIT_PROCESS("ERROR: zerospy unable to register events");
    }

    tls_idx = drmgr_register_tls_field();
    if (tls_idx == -1) {
        ZEROSPY_EXIT_PROCESS("ERROR: zerospy drmgr_register_tls_field fail");
    }
    if (!dr_raw_tls_calloc(&tls_seg, &tls_offs, INSTRACE_TLS_COUNT, 0)) {
        ZEROSPY_EXIT_PROCESS(
            "ERROR: zerospy dr_raw_tls_calloc fail");
    }
    gLock = dr_mutex_create();

    //drcctlib_init(ZEROSPY_FILTER_READ_MEM_ACCESS_INSTR, INVALID_FILE, InstrumentInsCallback, false/*do data centric*/);
    drcctlib_init_ex(ZEROSPY_FILTER_READ_MEM_ACCESS_INSTR, INVALID_FILE, InstrumentInsCallback, NULL, NULL, DRCCTLIB_CACHE_MODE);
    dr_register_exit_event(ClientExit);

    // Tracing Buffer
    // Integer 1 B
    trace_buffer_i1 = trace_buf_create_trace_buffer(MEM_BUF_SIZE(1));
    // Integer 2 B
    trace_buffer_i2 = trace_buf_create_trace_buffer(MEM_BUF_SIZE(2));
    // Integer 4 B
    trace_buffer_i4 = trace_buf_create_trace_buffer(MEM_BUF_SIZE(4));
    // Integer 8 B
    trace_buffer_i8 = trace_buf_create_trace_buffer(MEM_BUF_SIZE(8));
    // Integer 16 B
    trace_buffer_i16 = trace_buf_create_trace_buffer(MEM_BUF_SIZE(16));
    // Integer 32 B
    trace_buffer_i32 = trace_buf_create_trace_buffer(MEM_BUF_SIZE(32));
    // Floating Point Single
    trace_buffer_sp1 = trace_buf_create_trace_buffer(MEM_BUF_SIZE(4));
    // Floating Point Double
    trace_buffer_dp1 = trace_buf_create_trace_buffer(MEM_BUF_SIZE(8));
    // Floating Point 4*Single
    trace_buffer_sp4 = trace_buf_create_trace_buffer(MEM_BUF_SIZE(16));
    // Floating Point 2*Double
    trace_buffer_dp2 = trace_buf_create_trace_buffer(MEM_BUF_SIZE(16));
    // Floating Point 8*Single
    trace_buffer_sp8 = trace_buf_create_trace_buffer(MEM_BUF_SIZE(32));
    // Floating Point 4*Double
    trace_buffer_dp4 = trace_buf_create_trace_buffer(MEM_BUF_SIZE(32));
}

#ifdef __cplusplus
}
#endif