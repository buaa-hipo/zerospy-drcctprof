// #include <unordered_map>
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
#include "drx.h"

#ifdef X86
#include <nmmintrin.h>
#include <immintrin.h>
#include <emmintrin.h>
#endif

// #define WINDOW_ENABLE 1000000
// #define WINDOW_DISABLE 100000000
// #define WINDOW_CLEAN 10

int window_enable;
int window_disable;

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
(DROPTION_SCOPE_CLIENT, "rate", 10, 0, 1000, "Sampling rate configuration of bursty sampling",
 "Sampling rate configuration of bursty sampling. Only available when sampling is enabled."
 "The final sampling rate is specified times of 0.001. Default is 10 (0.010 sampling rate).");

static droption_t<int> op_window
(DROPTION_SCOPE_CLIENT, "window", 100000000, 0, INT32_MAX, "Window size configuration of sampling",
 "Window size of sampling. Only available when sampling is enabled.");

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
// #define DECODE_CTXT(key) (context_handle_t)(((key) >> 32) & 0xffffffff)
// #define DECODE_ACCESSLEN(key) ((uint8_t)((key) & 0xff))
// #define DECODE_ELEMENTSIZE(key) ((uint8_t)(((key)>>8) & 0xff))

// 64K cache
#define MAX_CACHE_NUM 65536
#define ENCODE_CONSTANT(upper, normal) (((uint64_t)(upper)<<56) | ((uint64_t)normal))
#define DECODE_UPPER(val) (uint8_t)(((val)>>56)&0xff)
#define DECODE_NORMAL(val) ((val) & 0xffffffffffffffLL)

struct INTRedLog_t{
    uint64_t tot; // upper 8-bits are accessLen
    uint64_t red;
    uint64_t fred;
    uint64_t redByteMap;
};

// AVX2
#define MAX_VLEN 4
struct FPRedLog_t{
    uint64_t ftot; // upper 8-bits are accessLen
    uint64_t fred; // upper 8-bits are elementSize
    uint64_t redByteMap[MAX_VLEN];
};

// typedef std::unordered_map<uint64_t, INTRedLog_t> INTRedLogMap_t;
// typedef std::unordered_map<uint64_t, FPRedLog_t> FPRedLogMap_t;

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
    FPRedLog_t** FPRedLogList;
    INTRedLog_t** INTRedLogList;
    vector<FPRedLog_t*>* fp_cache_list;
    vector<INTRedLog_t*>* int_cache_list;
    FPRedLog_t* fp_cache_ptr;
    INTRedLog_t* int_cache_ptr;
    int fp_cache_idx;
    int int_cache_idx;
    file_t output_file;
    void* numInsBuff;
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
#define USE_SIMD
#define USE_SSE

#ifdef USE_CLEANCALL
#define IS_SAMPLED(pt, WINDOW_ENABLE) (pt->sampleFlag)
#else
#define IS_SAMPLED(pt, WINDOW_ENABLE) ((int64_t)(BUF_PTR(pt->numInsBuff, void, INSTRACE_TLS_OFFS_BUF_PTR))<(int64_t)WINDOW_ENABLE)
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
    // lookup the preallocated entry list to seek whether there is a valid log entry
    INTRedLog_t* p = pt->INTRedLogList[ctxt_hndl];
    if(p==NULL) {
        // if no valid entry exists, take a log from the preallocated cache
        p = &(pt->int_cache_ptr[pt->int_cache_idx]);
        // set up initial values
        p->tot = ENCODE_CONSTANT(accessLen, accessLen);
        p->red = redZero;
        p->fred = fred;
        p->redByteMap = redByteMap;
        // insert the initialized entry
        pt->INTRedLogList[ctxt_hndl] = p;
        ++(pt->int_cache_idx);
        // if the cache is empty, allocate a new one
        if(pt->int_cache_idx >= MAX_CACHE_NUM) {
            pt->int_cache_idx = 0;
            pt->int_cache_ptr = (INTRedLog_t*)dr_global_alloc(sizeof(INTRedLog_t)*MAX_CACHE_NUM);
            pt->int_cache_list->push_back(pt->int_cache_ptr);
        }
    } else {
        // if exists, accumulate the log
        p->tot += accessLen;
        p->red += redZero;
        p->fred += fred;
        p->redByteMap &= redByteMap;
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
};

template<int esize, int size>
inline __attribute__((always_inline)) void AddFPRedLog(uint64_t ctxt_hndl, void* pval, per_thread_t* pt) {
    FPRedLog_t* p = pt->FPRedLogList[ctxt_hndl];
    if(p==NULL) {
        // if no valid entry exists, take a log from the preallocated cache
        p = &(pt->fp_cache_ptr[pt->fp_cache_idx]);
        // set up initial values
        p->ftot = ENCODE_CONSTANT(size, size/esize);
        uint64_t fred = UnrolledFunctions<0, size, esize>::getFullyRedundantZeroFP(pval);
        p->fred = ENCODE_CONSTANT(esize, fred);
        UnrolledFunctions<0, size, esize>::memcpy(p->redByteMap, pval);
        // insert the initialized entry
        pt->FPRedLogList[ctxt_hndl] = p;
        ++(pt->fp_cache_idx);
        // if the cache is empty, allocate a new one
        if(pt->fp_cache_idx >= MAX_CACHE_NUM) {
            pt->fp_cache_idx = 0;
            pt->fp_cache_ptr = (FPRedLog_t*)dr_global_alloc(sizeof(FPRedLog_t)*MAX_CACHE_NUM);
            pt->fp_cache_list->push_back(pt->fp_cache_ptr);
        }
    } else {
        // if exists, accumulate the log
        p->ftot += size/esize;
        p->fred += UnrolledFunctions<0, size, esize>::getFullyRedundantZeroFP(pval);
        if(esize==4) {
            UnrolledFunctions<0, size, esize>::mergeRedByteMapFP(p->redByteMap, pval);
        } else {
            UnrolledFunctions<0, size, esize>::mergeRedByteMapFP(p->redByteMap, pval);
        }
    }
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
    // update info
    if(accessLen<=8) {
        uint8_t* bytes = reinterpret_cast<uint8_t*>(addr);
        if(bytes[accessLen-1]!=0) {
            // the log have already been clear to 0, so we do nothing here and quick return.
            AddINTRedLog(ctxt_hndl, accessLen, 0, 0, 0, pt);
            return ;
        }
    }
    return ;
    uint64_t redByteMap;
    switch(accessLen) {
        case 1:
            redByteMap = count_zero_bytemap_int8((uint8_t*)addr);
            break;
        case 2:
            redByteMap = count_zero_bytemap_int16((uint8_t*)addr);
            break;
        case 4:
            redByteMap = count_zero_bytemap_int32((uint8_t*)addr);
            break;
        case 8:
            redByteMap = count_zero_bytemap_int64((uint8_t*)addr);
            break;
        case 16:
#ifdef USE_SIMD
            redByteMap = count_zero_bytemap_int128((uint8_t*)addr);
#else
            redByteMap = count_zero_bytemap_int64((uint8_t*)addr) |
                        (count_zero_bytemap_int64((uint8_t*)addr+8)<<8);
#endif
            break;
        case 32:
#ifdef USE_SIMD
            redByteMap = count_zero_bytemap_int256((uint8_t*)addr);
#else
            redByteMap = count_zero_bytemap_int64((uint8_t*)addr) |
                        (count_zero_bytemap_int64((uint8_t*)addr+8)<<8) |
                        (count_zero_bytemap_int64((uint8_t*)addr+16)<<16) |
                        (count_zero_bytemap_int64((uint8_t*)addr+24)<<24);
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
        uint64_t redByteMap_2 = (~redByteMap) & ((1<<accessLen)-1);
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
    uint64_t redZero = _mm_popcnt_u64(redByteMap);
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
template<uint32_t AccessLen, uint32_t ElemLen, bool isApprox>
void CheckNByteValueAfterVGather(int slot, instr_t* instr)
{
    static_assert(ElemLen==4 || ElemLen==8);
    void *drcontext = dr_get_current_drcontext();
    context_handle_t ctxt_hndl = drcctlib_get_context_handle(drcontext, slot);
    per_thread_t *pt = (per_thread_t *)drmgr_get_tls_field(drcontext, tls_idx);
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
#define HANDLE_VGATHER(T, ACCESS_LEN, ELEMENT_LEN, IS_APPROX) \
dr_insert_clean_call(drcontext, bb, ins, (void *)CheckNByteValueAfterVGather<(ACCESS_LEN), (ELEMENT_LEN), (IS_APPROX)>, false, 2, OPND_CREATE_CCT_INT(slot), OPND_CREATE_INTPTR(ins_clone))
#endif
/***************************************************************************************/

static inline int
compute_log2(int value)
{
    int i;
    for (i = 0; i < 31; i++) {
        if (value == 1 << i)
            return i;
    }
    // returns -1 if value is not a power of 2.
    return -1;
}

template<bool enable_cache>
void insertCheckAndUpdateInt(void* drcontext, instrlist_t* ilist, instr_t* where, int accessLen, int elemLen, int32_t slot, reg_id_t reg_addr, reg_id_t scratch) {
    // quick check
    switch(accessLen) {
        case 1:
        case 2:
        case 4:
        case 8:
            assert(accessLen==elemLen);
            break;
        default:
            assert(elemLen==1);
    }
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
    switch(accessLen) {
        case 4:
        case 8:
            assert(accessLen==elemLen);
            break;
        case 16:
        case 32:
            assert(elemLen==4 || elemLen==8);
            break;
        default:
            assert(0 && "Unknown accessLen");
    }
    // start check and update
    if(enable_cache) {
        // TODO: inlined check and updates
    }
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
    }
    if(enable_cache) {
        // TODO: inlined check and updates
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

static dr_emit_flags_t
event_basic_block(void *drcontext, void *tag, instrlist_t *bb, bool for_trace, bool translating, OUT void **user_data)
{
    int num_instructions = 0;
    int memRefCnt = 0;
    instr_t *instr;
    /* count the number of instructions in this block */
    IF_DEBUG(dr_fprintf(gFile, "^^ INFO: Disassembled Instruction ^^^\n"));
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
                    uint32_t refSize = opnd_size_in_bytes(opnd_get_size(opnd));
                    if (refSize!=0) {
                        if(instr_is_floating(instr)) {
                            uint32_t operSize = FloatOperandSizeTable(instr, opnd);
                            if(operSize>0) {
                                memRefCnt++;
                            }
                        } else {
                            memRefCnt++;
                        }
                    }
                }
            }
            IF_DEBUG(dr_fprintf(gFile, "[memcnt=%d] ", memRefCnt));
        }
        IF_DEBUG(disassemble(drcontext, instr_get_app_pc(instr), gFile));
    }
    IF_DEBUG(dr_fprintf(gFile, "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n"));
    assert(num_instructions>=0);
    if(num_instructions!=0) {
        /* insert clean call */
        if(memRefCnt) {
            dr_insert_clean_call(drcontext, bb, instrlist_first(bb), (void *)BBInstrument::BBUpdateAndCheck, false, 1, OPND_CREATE_CCT_INT(num_instructions));
        } else {
            dr_insert_clean_call(drcontext, bb, instrlist_first(bb), (void *)BBInstrument::BBUpdate, false, 1, OPND_CREATE_CCT_INT(num_instructions));
        }
    }
    return DR_EMIT_DEFAULT;
}

struct ZerospyInstrument{
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
InstrumentMem(void *drcontext, instrlist_t *ilist, instr_t *where, opnd_t ref, int32_t slot, reg_id_t reg_addr, reg_id_t scratch)
{
    if (!drutil_insert_get_mem_addr(drcontext, ilist, where, ref, reg_addr/*addr*/,
                                    scratch/*scratch*/)) {
        dr_fprintf(STDERR, "\nERROR: drutil_insert_get_mem_addr failed!\n");
        dr_fprintf(STDERR, "^^ Disassembled Instruction ^^^\n");
        disassemble(drcontext, instr_get_app_pc(where), STDERR);
        dr_fprintf(STDERR, "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
        ZEROSPY_EXIT_PROCESS("InstrumentMem drutil_insert_get_mem_addr failed!");
    } else {
        ZerospyInstrument::InstrumentReadValueBeforeAndAfterLoading(drcontext, ilist, where, ref, slot, reg_addr, scratch);
    }
}

void InstrumentMemAll(void* drcontext, instrlist_t *bb, instr_t* instr, int32_t slot)
{
    drvector_t vec;
    getUnusedRegEntry(&vec, instr);
    reg_id_t reg_base=DR_REG_NULL, reg_addr, scratch;
    // RESERVE_AFLAGS(drcontext, bb, instr);
    RESERVE_REG(drcontext, bb, instr, &vec, reg_addr);
    instr_t* skipcall = INSTR_CREATE_label(drcontext);
    if(op_enable_sampling.get_value()) {
        assert(0);
        dr_insert_read_raw_tls(drcontext, bb, instr, tls_seg, tls_offs + INSTRACE_TLS_OFFS_BUF_PTR, reg_addr);
        // Clear insCnt when insCnt > WINDOW_DISABLE
        MINSERT(bb, instr, XINST_CREATE_cmp(drcontext, opnd_create_reg(reg_addr), OPND_CREATE_CCT_INT(window_enable)));
        MINSERT(bb, instr, XINST_CREATE_jump_cond(drcontext, DR_PRED_GT, opnd_create_instr(skipcall)));
    }
    RESERVE_REG(drcontext, bb, instr, &vec, scratch);
    int num_srcs = instr_num_srcs(instr);
    for(int i=0; i<num_srcs; ++i) {
        opnd_t opnd = instr_get_src(instr, i);
        if(opnd_is_memory_reference(opnd)) {
            InstrumentMem(drcontext, bb, instr, opnd, slot, reg_addr, scratch);
        }
    }
    UNRESERVE_REG(drcontext, bb, instr, scratch);
    MINSERT(bb, instr, skipcall);
    UNRESERVE_REG(drcontext, bb, instr, reg_addr);
    drvector_delete(&vec);
}

#ifdef X86
void InstrumentVGather(void* drcontext, instrlist_t *bb, instr_t* instr, int32_t slot)
{
    if(op_enable_sampling.get_value()) {
        assert(0);
        drvector_t vec;
        reg_id_t reg_ctxt;
        RESERVE_AFLAGS(drcontext, bb, instr);
        instr_t* skipcall = INSTR_CREATE_label(drcontext);
        getUnusedRegEntry(&vec, instr);
        RESERVE_REG(drcontext, bb, instr, &vec, reg_ctxt);
        dr_insert_read_raw_tls(drcontext, bb, instr, tls_seg, tls_offs + INSTRACE_TLS_OFFS_BUF_PTR, reg_ctxt);
        // Clear insCnt when insCnt > WINDOW_DISABLE
        MINSERT(bb, instr, XINST_CREATE_cmp(drcontext, opnd_create_reg(reg_ctxt), OPND_CREATE_CCT_INT(window_enable)));
        MINSERT(bb, instr, XINST_CREATE_jump_cond(drcontext, DR_PRED_GT, opnd_create_instr(skipcall)));
        // We use instr_compute_address_ex_pos to handle gather (with VSIB addressing)
        ZerospyInstrument::InstrumentReadValueBeforeVGather(drcontext, bb, instr, slot);
        MINSERT(bb, instr, skipcall);
        UNRESERVE_REG(drcontext, bb, instr, reg_ctxt);
        UNRESERVE_AFLAGS(drcontext, bb, instr);
        drvector_delete(&vec);
    } else {
        // We use instr_compute_address_ex_pos to handle gather (with VSIB addressing)
        ZerospyInstrument::InstrumentReadValueBeforeVGather(drcontext, bb, instr, slot);
    }
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

    // store cct in tls filed
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

static void
ClientThreadStart(void *drcontext)
{
    // assert(dr_get_itimer(ITIMER_REAL));
    per_thread_t *pt = (per_thread_t *)dr_thread_alloc(drcontext, sizeof(per_thread_t));
    if (pt == NULL) {
        ZEROSPY_EXIT_PROCESS("pt == NULL");
    }
    pt->INTRedLogList = (INTRedLog_t **)dr_raw_mem_alloc(
        (size_t)(sizeof(INTRedLog_t *)) * ((size_t)CONTEXT_HANDLE_MAX),
        DR_MEMPROT_READ | DR_MEMPROT_WRITE, NULL);
    pt->FPRedLogList = (FPRedLog_t **)dr_raw_mem_alloc(
        (size_t)sizeof(FPRedLog_t *) * (size_t)CONTEXT_HANDLE_MAX,
        DR_MEMPROT_READ | DR_MEMPROT_WRITE, NULL);
    assert(pt->INTRedLogList);
    assert(pt->FPRedLogList);
    memset(pt->INTRedLogList, 0, (size_t)sizeof(INTRedLog_t*)*(size_t)CONTEXT_HANDLE_MAX);
    memset(pt->FPRedLogList, 0, (size_t)sizeof(FPRedLog_t*)*(size_t)CONTEXT_HANDLE_MAX);
    pt->int_cache_ptr = (INTRedLog_t*)dr_global_alloc(sizeof(INTRedLog_t)*MAX_CACHE_NUM);
    pt->fp_cache_ptr = (FPRedLog_t*)dr_global_alloc(sizeof(FPRedLog_t)*MAX_CACHE_NUM);
    assert(pt->int_cache_ptr);
    assert(pt->fp_cache_ptr);
    pt->int_cache_idx = 0;
    pt->fp_cache_idx = 0;
    pt->int_cache_list = new vector<INTRedLog_t*>(1, pt->int_cache_ptr);
    pt->fp_cache_list = new vector<FPRedLog_t*>(1, pt->fp_cache_ptr);
    pt->instr_clones = new vector<instr_t*>();
    pt->numInsBuff = dr_get_dr_segment_base(tls_seg);
    BUF_PTR(pt->numInsBuff, void, INSTRACE_TLS_OFFS_BUF_PTR) = 0;
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

static uint64_t PrintRedundancyPairs(per_thread_t *pt, uint64_t threadBytesLoad, int threadId) 
{
    vector<RedundacyData> tmpList;
    vector<RedundacyData>::iterator tmpIt;

    file_t gTraceFile = pt->output_file;
    
    uint64_t grandTotalRedundantBytes = 0;
    // tmpList.reserve(CONTEXT_HANDLE_MAX);
    dr_fprintf(STDOUT, "Dumping INTEGER Redundancy Info...\n");
    uint64_t count = 0, rep = -1, num = 0;
    for(int i=0; i<CONTEXT_HANDLE_MAX; ++i) {
        ++count;
        if(100 * count / (CONTEXT_HANDLE_MAX)!=rep) {
            rep = 100 * count / (CONTEXT_HANDLE_MAX);
            dr_fprintf(STDOUT, "\r%ld%%  Finish",rep); fflush(stdout);
        }
        if(pt->INTRedLogList[i]!=NULL) {
            RedundacyData tmp = { i, pt->INTRedLogList[i]->red,
                                pt->INTRedLogList[i]->fred,        DECODE_NORMAL(pt->INTRedLogList[i]->tot),
                                pt->INTRedLogList[i]->redByteMap,  DECODE_UPPER(pt->INTRedLogList[i]->tot) };
            tmpList.push_back(tmp);
            grandTotalRedundantBytes += tmp.frequency;
            ++num;
        }
    }
    dr_fprintf(STDOUT, "\r100%%  Finish, Total num = %ld\n", num); fflush(stdout);
    if(num == 0) {
        dr_fprintf(STDOUT, "Warning: No valid profiling data is logged!\n");
        return 0;
    }

    dr_fprintf(gTraceFile, "\n--------------- Dumping INTEGER Redundancy Info ----------------\n");
    dr_fprintf(gTraceFile, "\n*************** Dump Data from Thread %d ****************\n", threadId);
    
    __sync_fetch_and_add(&grandTotBytesRedLoad,grandTotalRedundantBytes);
    dr_fprintf(STDOUT, "Extracted Raw data, now sorting...\n"); fflush(stdout);
    
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
    dr_fprintf(STDOUT, "Sorted, Now generating reports...\n"); fflush(stdout);
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
    dr_fprintf(STDOUT, "INTEGER Report dumped\n"); fflush(stdout);
    return grandTotalRedundantBytes;
}

template<int start, int end, int incr>
struct UnrolledConjunctionApprox{
    static __attribute__((always_inline)) uint64_t BodyRedMap(uint8_t* addr){
        if(incr==4)
            return count_zero_bytemap_fp((void*)(addr+start)) | (UnrolledConjunctionApprox<start+incr,end,incr>::BodyRedMap(addr)<<SP_MAP_SIZE);
        else if(incr==8)
            return count_zero_bytemap_dp((void*)(addr+start)) | (UnrolledConjunctionApprox<start+incr,end,incr>::BodyRedMap(addr)<<DP_MAP_SIZE);
        else
            assert(0 && "Not Supportted floating size! now only support for FP32 or FP64.");
        return 0;
    }
};

template<int end,  int incr>
struct UnrolledConjunctionApprox<end , end , incr>{
    static __attribute__((always_inline)) uint64_t BodyRedMap(uint8_t* addr){
        return 0;
    }
};

/****************************************************************************************/

static inline __attribute__((always_inline)) uint64_t getFPRedByteMap(uint8_t* addr, int accessLen, int elementSize) {
    assert(elementSize <= accessLen);
    assert(elementSize==4 || elementSize==8);
    uint64_t map;
    switch(accessLen) {
        case 4:
            map = UnrolledConjunctionApprox<0,4,4>::BodyRedMap(addr); 
            break;
        case 8:
            assert(elementSize==8);
            map = UnrolledConjunctionApprox<0,8,8>::BodyRedMap(addr); 
            break;
        case 16:
            if(elementSize==4) {
                map = UnrolledConjunctionApprox<0,16,4>::BodyRedMap(addr); 
            } else {
                map = UnrolledConjunctionApprox<0,16,8>::BodyRedMap(addr); 
            }
            break;
        case 32:
            if(elementSize==4) {
                map = UnrolledConjunctionApprox<0,32,4>::BodyRedMap(addr); 
            } else {
                map = UnrolledConjunctionApprox<0,32,8>::BodyRedMap(addr); 
            }
            break;
        default:
            assert(0 && "Unknown accessLen!\n");
    }
    return map;
}

static uint64_t PrintApproximationRedundancyPairs(per_thread_t *pt, uint64_t threadBytesLoad, int threadId) 
{
    vector<ApproxRedundacyData> tmpList;
    vector<ApproxRedundacyData>::iterator tmpIt;
    // tmpList.reserve(CONTEXT_HANDLE_MAX);
    
    file_t gTraceFile = pt->output_file;
    uint64_t grandTotalRedundantBytes = 0;

    dr_fprintf(STDOUT, "Dumping FLOATING POINT Redundancy Info...\n");
    uint64_t count = 0, rep = -1, num = 0;
    for(int i=0; i<CONTEXT_HANDLE_MAX; ++i) {
        ++count;
        if(100 * count / (CONTEXT_HANDLE_MAX)!=rep) {
            rep = 100 * count / (CONTEXT_HANDLE_MAX);
            dr_fprintf(STDOUT, "\r%ld%%  Finish",rep); fflush(stdout);
        }
        if(pt->FPRedLogList[i]!=NULL) {
            uint8_t accessLen = DECODE_UPPER(pt->FPRedLogList[i]->ftot);
            uint8_t elementSize = DECODE_UPPER(pt->FPRedLogList[i]->fred);
            uint64_t redByteMap = getFPRedByteMap((uint8_t*)pt->FPRedLogList[i]->redByteMap, accessLen, elementSize);
            ApproxRedundacyData tmp = { i,
                                        DECODE_NORMAL(pt->FPRedLogList[i]->fred),
                                        DECODE_NORMAL(pt->FPRedLogList[i]->ftot),
                                        redByteMap,
                                        accessLen,
                                        elementSize };
            tmpList.push_back(tmp);
            grandTotalRedundantBytes += DECODE_NORMAL(pt->FPRedLogList[i]->fred) * accessLen;
            ++num;
        }
    }
    dr_fprintf(STDOUT, "\r100%%  Finish, Total num = %ld\n", num); fflush(stdout);
    if(num == 0) {
        dr_fprintf(STDOUT, "Warning: No valid profiling data is logged!\n");
        return 0;
    }

    dr_fprintf(gTraceFile, "\n--------------- Dumping Approximation Redundancy Info ----------------\n");
    dr_fprintf(gTraceFile, "\n*************** Dump Data(delta=%.2f%%) from Thread %d ****************\n", delta*100,threadId);
    
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
    for(int i=0; i<CONTEXT_HANDLE_MAX; ++i) {
        if(pt->INTRedLogList[i]!=NULL) {
            x += pt->INTRedLogList[i]->tot;
        }
    }
    for(int i=0; i<CONTEXT_HANDLE_MAX; ++i) {
        if(pt->FPRedLogList[i]!=NULL) {
            x += DECODE_NORMAL(pt->FPRedLogList[i]->ftot) * DECODE_UPPER(pt->FPRedLogList[i]->ftot);
        }
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
    per_thread_t *pt = (per_thread_t *)drmgr_get_tls_field(drcontext, tls_idx);
    uint64_t threadByteLoad = getThreadByteLoad(pt);
    if(threadByteLoad!=0) 
    {
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
    }
    dr_close_file(pt->output_file);
    for(size_t i=0;i<pt->instr_clones->size();++i) {
        instr_destroy(drcontext, (*pt->instr_clones)[i]);
    }
    delete pt->instr_clones;
    dr_global_free(pt->INTRedLogList, sizeof(INTRedLog_t*)*CONTEXT_HANDLE_MAX);
    dr_global_free(pt->FPRedLogList , sizeof(FPRedLog_t *)*CONTEXT_HANDLE_MAX);
    for(size_t i=0; i<pt->int_cache_list->size(); ++i) {
        dr_global_free((*pt->int_cache_list)[i], sizeof(INTRedLog_t)*MAX_CACHE_NUM);
    }
    delete pt->int_cache_list;
    for(size_t i=0; i<pt->fp_cache_list->size(); ++i) {
        dr_global_free((*pt->fp_cache_list)[i], sizeof(FPRedLog_t)*MAX_CACHE_NUM);
    }
    delete pt->fp_cache_list;
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
        dr_fprintf(STDOUT, "[ZEROSPU INFO] Sampling Rate: %.3f, Window Size: %ld\n", op_rate.get_value(), op_window.get_value());
        dr_fprintf(gFile,  "[ZEROSPU INFO] Sampling Rate: %.3f, Window Size: %ld\n", op_rate.get_value(), op_window.get_value());
        window_enable = op_rate.get_value() * op_window.get_value();
        window_disable= op_window.get_value();
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
#ifndef _WERROR
    if(warned) {
        dr_fprintf(gFile, "####################################\n");
        dr_fprintf(gFile, "WARNING: some unexpected instructions are ignored. Please check zerospy.log.warn for detail.\n");
        dr_fprintf(gFile, "####################################\n");
    }
#endif
    dr_close_file(gFlagF);
    dr_close_file(gFile);
    if (!dr_raw_tls_cfree(tls_offs, INSTRACE_TLS_COUNT)) {
        ZEROSPY_EXIT_PROCESS(
            "ERROR: zerospy dr_raw_tls_calloc fail");
    }
    dr_mutex_destroy(gLock);
    drcctlib_exit();
    if (!drmgr_unregister_thread_init_event(ClientThreadStart) ||
        !drmgr_unregister_thread_exit_event(ClientThreadEnd) ||
        // must unregister event after client exit, or it will cause unexpected errors during execution
        ( op_enable_sampling.get_value() && 
            !drmgr_unregister_bb_instrumentation_event(event_basic_block)) 
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
    if (( op_enable_sampling.get_value() && 
           !drmgr_register_bb_instrumentation_event(event_basic_block, NULL, NULL) )
        || !drmgr_register_thread_init_event_ex(ClientThreadStart, &thread_init_pri) 
        || !drmgr_register_thread_exit_event_ex(ClientThreadEnd, &thread_exit_pri) ) {
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

    drcctlib_init(ZEROSPY_FILTER_READ_MEM_ACCESS_INSTR, INVALID_FILE, InstrumentInsCallback, false/*do data centric*/);
    //drcctlib_init_ex(ZEROSPY_FILTER_READ_MEM_ACCESS_INSTR, INVALID_FILE, InstrumentInsCallback, NULL, NULL, DRCCTLIB_CACHE_MODE);
    dr_register_exit_event(ClientExit);
}

#ifdef __cplusplus
}
#endif