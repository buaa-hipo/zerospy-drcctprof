#include <unordered_map>
#include <vector>
#include <list>
#include <string>
#include <sys/stat.h>
#include <assert.h>
#include <algorithm>

#ifdef DEBUG
#define IF_DEBUG(stat) stat
#else
#define IF_DEBUG(stat)
#endif

// #define ZEROSPY_DEBUG
#define DEBUG_CHECK
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
// enable data centric with addr info
#define DRCCTLIB_USE_ADDR
#include "drcctlib.h"
#include "utils.h"
#include "trace.h"
#include "bitvec.h"

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

template<int size>
struct cache_t {
    void* addr;
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

struct RedLogs{
    uint64_t red;  // how many byte zero
    bitvec_t redmap; // bitmap logging if a byte is redundant
    bitvec_t accmap; // bitmap logging if a byte is accessed
};

typedef unordered_map<uint64_t, RedLogs> RedLogSizeMap;
typedef unordered_map<uint64_t, RedLogSizeMap> RedLogMap;

struct FPRedLogs{
    uint64_t red;  // how many byte zero
    bitvec_t redmap; // bitmap logging if a byte is redundant
    bitvec_t accmap; // bitmap logging if a byte is accessed
    uint8_t typesz;
};

typedef unordered_map<uint64_t, FPRedLogs> FPRedLogSizeMap;
typedef unordered_map<uint64_t, FPRedLogSizeMap> FPRedLogMap;

#define MINSERT instrlist_meta_preinsert
#define MAKE_OBJID(a, b) (((uint64_t)(a)<<32) | (b))
#define DECODE_TYPE(a) (((uint64_t)(a)&(0xffffffffffffffff))>>32)
#define DECODE_NAME(b) ((uint64_t)(b)&(0x00000000ffffffff))


#define MAKE_APPROX_OBJID(a, b, ts) (((uint64_t)(a)<<32) | ((b)<<8) | (ts))
#define DECODE_APPROX_TYPE(a) (((uint64_t)(a)&(0xffffffffffffffff))>>32)
#define DECODE_APPROX_NAME(b) (((uint64_t)(b)&(0x00000000ffffff00))>>8)
#define DECODE_APPROX_TYPESZ(c) ((uint64_t)(c)&(0x00000000000000ff))

#define MAKE_CNTXT(a, b, c) (((uint64_t)(a)<<32) | ((uint64_t)(b)<<16) | (uint64_t)(c))
#define DECODE_CNTXT(a) (static_cast<ContextHandle_t>((((a)&(0xffffffffffffffff))>>32)))
#define DECODE_ACCLN(b) (((uint64_t)(b)&(0x00000000ffff0000))>>16)
#define DECODE_TYPSZ(c)  ((uint64_t)(c)&(0x000000000000ffff))

#define MAX_OBJS_TO_LOG 100

#define delta 0.01

#define CACHE_LINE_SIZE (64)
#ifndef PAGE_SIZE
#define PAGE_SIZE (4*1024)
#endif
#define MAX_REDUNDANT_CONTEXTS_TO_LOG (1000)
// maximum cct depth to print
#define MAX_DEPTH 10

enum {
    INSTRACE_TLS_OFFS_BUF_PTR,
    INSTRACE_TLS_COUNT, /* total number of TLS slots allocated */
};

static reg_id_t tls_seg;
static uint tls_offs;
#define TLS_SLOT(tls_base, enum_val) (void **)((byte *)(tls_base) + tls_offs + (enum_val))
#define BUF_PTR(tls_base, type, offs) *(type **)TLS_SLOT(tls_base, offs)

// 1M
#define MAX_CLONE_INS 1048576
typedef struct _per_thread_t {
    RedLogMap *INTRedMap;
    FPRedLogMap *FPRedMap;
    file_t output_file;
    void* numInsBuff;
    int32_t threadId;
#ifdef ZEROSPY_DEBUG
    vector<instr_t*> *instr_clones;
#endif
} per_thread_t;


#define IS_SAMPLED(pt, WINDOW_ENABLE) ((int64_t)(BUF_PTR(pt->numInsBuff, void, INSTRACE_TLS_OFFS_BUF_PTR))<(int64_t)WINDOW_ENABLE)

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
// TODO: May be further optimized by combining size and data hndl to avoid one more mapping
static inline void AddToRedTable(uint64_t addr, data_handle_t data, uint16_t value, uint16_t total, uint32_t redmap, per_thread_t *pt) __attribute__((always_inline,flatten));
static inline void AddToRedTable(uint64_t addr, data_handle_t data, uint16_t value, uint16_t total, uint32_t redmap, per_thread_t *pt) {
    assert(addr<=(uint64_t)data.end_addr);
    size_t offset = addr-(uint64_t)data.beg_addr;
    uint64_t key = MAKE_OBJID(data.object_type,data.sym_name);
    RedLogMap::iterator it2 = pt->INTRedMap->find(key);
    RedLogSizeMap::iterator it;
    size_t size = (uint64_t)data.end_addr - (uint64_t)data.beg_addr;
    // IF_DEBUG(dr_fprintf(
    //         STDOUT,
    //         "AddToRedTable 1: offset=%ld, total=%d, size=%ld\n", offset, total, size));
    if ( it2  == pt->INTRedMap->end() || (it = it2->second.find(size)) == it2->second.end()) {
        RedLogs log;
        log.red = value;
#ifdef DEBUG_CHECK
        if(offset+total>size) {
            printf("AddToRedTable 1: offset=%ld, total=%d, size=%ld\n", offset, total, size);
            if(data.object_type == DYNAMIC_OBJECT) {
                printf("\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Dynamic Object: ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
                drcctlib_print_full_cct(STDOUT, data.sym_name, true, true, MAX_DEPTH);
            } else {
                printf("\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Static Object: %s ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n", drcctlib_get_str_from_strpool((uint32_t)data.sym_name));
            }
        }
#endif
        bitvec_alloc(&log.redmap, size);
        bitvec_and(&log.redmap, redmap, offset, total);
        bitvec_alloc(&log.accmap, size);
        bitvec_and(&log.accmap, 0, offset, total);
        (*pt->INTRedMap)[key][size] = log;
    } else {
        assert(it->second.redmap.size==it->second.accmap.size);
        assert(size == it->second.redmap.size);
#ifdef DEBUG_CHECK
        if(offset+total>size) {
            printf("AddToRedTable 2: offset=%ld, total=%d, size=%ld\n", offset, total, size);
            if(data.object_type == DYNAMIC_OBJECT) {
                printf("\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Dynamic Object: ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
                drcctlib_print_full_cct(STDOUT, data.sym_name, true, true, MAX_DEPTH);
            } else {
                printf("\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Static Object: %s ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n", drcctlib_get_str_from_strpool((uint32_t)data.sym_name));
            }
        }
#endif
        it->second.red += value;
        bitvec_and(&(it->second.redmap), redmap, offset, total);
        bitvec_and(&(it->second.accmap), 0, offset, total);
    }
}

static inline void AddToApproximateRedTable(uint64_t addr, data_handle_t data, uint16_t value, uint16_t total, uint64_t redmap, uint32_t typesz, per_thread_t *pt) __attribute__((always_inline,flatten));
static inline void AddToApproximateRedTable(uint64_t addr, data_handle_t data, uint16_t value, uint16_t total, uint64_t redmap, uint32_t typesz, per_thread_t *pt) {
    // printf("ADDR=%lx, beg_addr=%lx, end_addr=%lx, typesz=%d, index=%ld, size=%ld\n", addr, (uint64_t)data.beg_addr, (uint64_t)data.end_addr, typesz, addr-(uint64_t)data.beg_addr, (uint64_t)data.end_addr - (uint64_t)data.beg_addr);
    assert(addr<=(uint64_t)data.end_addr);
    size_t offset = addr-(uint64_t)data.beg_addr;
    uint64_t key = MAKE_APPROX_OBJID(data.object_type,data.sym_name, typesz);
    FPRedLogMap::iterator it2 = pt->FPRedMap->find(key);
    FPRedLogSizeMap::iterator it;
    // the data size may not aligned with typesz, so use upper bound as the bitvec size
    // Note: not aligned case : struct/class with floating and int.
    size_t size = (uint64_t)data.end_addr - (uint64_t)data.beg_addr;
    if(value > total) {
        dr_fprintf(STDERR, "** Warning AddToApproximateTable : value %d, total %d **\n", value, total);
        assert(0 && "** BUG #0 Detected. Existing **");
    }
    if ( it2  == pt->FPRedMap->end() || (it = it2->second.find(size)) == it2->second.end()) {
        FPRedLogs log;
        log.red = value;
        log.typesz = typesz;
#ifdef DEBUG_CHECK
        if(offset+total>size) {
            printf("AddToApproxRedTable 1: offset=%ld, total=%d, size=%ld\n", offset, total, size);
            if(data.object_type == DYNAMIC_OBJECT) {
                printf("\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Dynamic Object: ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
                drcctlib_print_full_cct(STDOUT, data.sym_name, true, true, MAX_DEPTH);
            } else {
                printf("\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Static Object: %s ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n", drcctlib_get_str_from_strpool((uint32_t)data.sym_name));
            }
        }
#endif
        bitvec_alloc(&log.redmap, size);
        bitvec_and(&log.redmap, redmap, offset, total);
        bitvec_alloc(&log.accmap, size);
        bitvec_and(&log.accmap, 0, offset, total);
        (*pt->FPRedMap)[key][size] = log;
    } else {
        assert(it->second.redmap.size==it->second.accmap.size);
        assert(size == it->second.redmap.size);
        if(it->second.typesz != typesz) {
            printf("it->second.typesz=%d typesz=%d\n", it->second.typesz, typesz);
        }
        assert(it->second.typesz == typesz);
#ifdef DEBUG_CHECK
        if(offset+total>size) {
            printf("AddToApproxRedTable 1: offset=%ld, total=%d, size=%ld\n", offset, total, size);
            if(data.object_type == DYNAMIC_OBJECT) {
                printf("\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Dynamic Object: ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
                drcctlib_print_full_cct(STDOUT, data.sym_name, true, true, MAX_DEPTH);
            } else {
                printf("\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Static Object: %s ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n", drcctlib_get_str_from_strpool((uint32_t)data.sym_name));
            }
        }
#endif
        it->second.red += value;
        bitvec_and(&(it->second.redmap), redmap, offset, total);
        bitvec_and(&(it->second.accmap), 0, offset, total);
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

// sample wrapper for sampling without code flush
#if 0
#define SAMPLED_CLEAN_CALL(insert_clean_call) do { \
    bool dead; \
    reg_id_t reg_ptr; \
    RESERVE_AFLAGS(dead, bb, ins); \
    assert(drreg_reserve_register(drcontext, bb, ins, NULL, &reg_ptr)==DRREG_SUCCESS); 
    dr_insert_read_raw_tls(drcontext, bb, ins, tls_seg, tls_offs + INSTRACE_TLS_OFFS_BUF_PTR, reg_ptr);\
    minstr_load_wint_to_reg(drcontext, ilist, where, reg_val, window_disable);\
    MINSERT(bb, ins, XINST_CREATE_cmp(drcontext, opnd_create_reg(reg_ptr), OPND_CREATE_INT32(window_enable))); \
    instr_t* restore = INSTR_CREATE_label(drcontext); \
    MINSERT(bb, ins, XINST_CREATE_jump_cond(drcontext, DR_PRED_LE, opnd_create_instr(restore))); \
    { insert_clean_call; } \
    MINSERT(bb, ins, restore); \
    assert(drreg_unreserve_register(drcontext, bb, ins, reg_ptr)==DRREG_SUCCESS); 
    UNRESERVE_AFLAGS(dead, bb, ins); \
} while(0)
#endif
#define SAMPLED_CLEAN_CALL(insert_clean_call) insert_clean_call

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

template<int accessLen, int elementSize>
inline __attribute__((always_inline))
void CheckAndInsertIntPage_impl(void* addr, void* pval, per_thread_t *pt) {
    // update info
    uint8_t* bytes = reinterpret_cast<uint8_t*>(pval);
    void* drcontext = dr_get_current_drcontext();
    data_handle_t data_hndl =
                drcctlib_get_data_hndl_ignore_stack_data(drcontext, (app_pc)addr);
    if(data_hndl.object_type!=DYNAMIC_OBJECT && data_hndl.object_type!=STATIC_OBJECT) {
        return ;
    }
    if(bytes[accessLen-1]!=0) {
        // the log have already been clear to 0, so we do nothing here and quick return.
        AddToRedTable((uint64_t) addr, data_hndl, 0, accessLen, 0, pt);
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
    } else {
        // accessLen == elementSize
        uint64_t redByteMap_2 = (~redByteMap) & ((1LL<<accessLen)-1);
        uint64_t redZero = _lzcnt_u64(redByteMap_2) - (64-accessLen);
    }
    AddToRedTable((uint64_t)addr, data_hndl, redZero, accessLen, redByteMap, pt);
#else
    uint64_t redZero = UnrolledConjunction<0, accessLen, elementSize>::BodyRedNum(redByteMap);
    AddToRedTable((uint64_t)addr, data_hndl, redZero, accessLen, redByteMap, pt);
#endif
}

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
        CheckAndInsertIntPage_impl<sz, esize>(cache_ptr->addr, (void*)cache_ptr->val, pt);
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
        data_handle_t data_hndl =
                drcctlib_get_data_hndl_ignore_stack_data(drcontext, (app_pc)cache_ptr->addr);
        if(data_hndl.object_type!=DYNAMIC_OBJECT && data_hndl.object_type!=STATIC_OBJECT) {
            continue ;
        }
        uint8_t* bytes = reinterpret_cast<uint8_t*>(cache_ptr->val);
        bool hasRedundancy = UnrolledConjunctionApprox<0,sz,esize>::BodyHasRedundancy(bytes);
        if(hasRedundancy) {
            uint64_t map = UnrolledConjunctionApprox<0,sz,esize>::BodyRedMap(bytes);
            uint32_t zeros = UnrolledConjunctionApprox<0,sz,esize>::BodyZeros(bytes);        
            AddToApproximateRedTable((uint64_t)cache_ptr->addr,data_hndl, zeros, sz/esize, map, esize, pt);
        } else {
            AddToApproximateRedTable((uint64_t)cache_ptr->addr,data_hndl, 0, sz/esize, 0, esize, pt);
        }
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
    num_instructions = count_bb_info_detailed(drcontext, bb, memRefCnt_detail);
    assert(num_instructions>=0);
    if(num_instructions!=0) {
        /* insert clean call */
        if(is_sampling) {
            ushort memRefCnt = memRefCnt_detail[0];
            for(int i=1; i<12; ++i) memRefCnt += memRefCnt_detail[i];
            if(memRefCnt) {
                // dr_insert_clean_call(drcontext, bb, instrlist_first(bb), (void *)debug_output_line, false, 0);
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
    //dr_insert_clean_call(drcontext, ilist, where, (void *)debug_output_line, false, 0);
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
    trace_buf_insert_buf_store(drcontext, trace_buffer, ilist, where, reg_ptr, DR_REG_NULL,
                             opnd_create_reg(reg_addr), OPSZ_PTR, offsetof(cache_t<size>, addr));
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
    // update trace buffer
    trace_buf_insert_update_buf_ptr(drcontext, trace_buffer, ilist, where, reg_ptr,
                                  DR_REG_NULL, sizeof(cache_t<size>));
}

template<class T, uint32_t AccessLen, uint32_t ElemLen, bool isApprox, bool enable_sampling>
struct ZeroSpyAnalysis{
#ifdef X86
    static __attribute__((always_inline)) void CheckNByteValueAfterVGather(int32_t slot, instr_t* instr)
    {
        void *drcontext = dr_get_current_drcontext();
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
            uint32_t zeros=0;
            uint64_t map=0;
            app_pc addr;
            bool is_write;
            uint32_t pos;
            for( int index=0; instr_compute_address_ex_pos(instr, &mcontext, index, &addr, &is_write, &pos); ++index ) {
                DR_ASSERT(!is_write);
                data_handle_t data_hndl = drcctlib_get_data_hndl_ignore_stack_data(drcontext, addr);
                if(data_hndl.object_type!=DYNAMIC_OBJECT && data_hndl.object_type!=STATIC_OBJECT) {
                    continue ;
                }
                T* bytes = reinterpret_cast<T*>(addr);
                uint64_t val = (bytes[0] == 0) ? 1 : 0;
                AddToApproximateRedTable((uint64_t)addr, data_hndl, val, 1, val, sizeof(T), pt);
            }
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
    data_handle_t data_hndl =
                drcctlib_get_data_hndl_ignore_stack_data(drcontext, (app_pc)addr);
    if(data_hndl.object_type!=DYNAMIC_OBJECT && data_hndl.object_type!=STATIC_OBJECT) {
        return ;
    }
#ifdef DEBUG_CHECK
    {
        size_t size   = (uint64_t)data_hndl.end_addr - (uint64_t)data_hndl.beg_addr;
        size_t offset = (uint64_t)addr - (uint64_t)data_hndl.beg_addr;
        size_t total  = accessLen;
        if(offset + total > size) {
            context_handle_t curCtxtHandle = drcctlib_get_context_handle(drcontext, slot);
            printf("\n\nWARN: overflow detected: offset=%ld, total=%ld, size=%ld\n", offset, total, size);
            drcctlib_print_full_cct(STDOUT, curCtxtHandle, true, true, MAX_DEPTH);
        }
    }
#endif
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
        AddToRedTable((uint64_t)addr, data_hndl, redbytesNum, accessLen, redbyteMap, pt);
    }
    else {
        AddToRedTable((uint64_t)addr, data_hndl, 0, accessLen, 0, pt);
    }
}

#ifdef X86
#define HANDLE_VGATHER(T, ACCESS_LEN, ELEMENT_LEN, IS_APPROX) do {\
if(op_enable_sampling.get_value()) { \
dr_insert_clean_call(drcontext, bb, ins, (void *)ZeroSpyAnalysis<T, (ACCESS_LEN), (ELEMENT_LEN), (IS_APPROX), true>::CheckNByteValueAfterVGather, false, 2, OPND_CREATE_CCT_INT(slot), OPND_CREATE_INTPTR(ins_clone)); \
} else { \
dr_insert_clean_call(drcontext, bb, ins, (void *)ZeroSpyAnalysis<T, (ACCESS_LEN), (ELEMENT_LEN), (IS_APPROX), false>::CheckNByteValueAfterVGather, false, 2, OPND_CREATE_CCT_INT(slot), OPND_CREATE_INTPTR(ins_clone)); \
} } while(0)
#endif

#include <signal.h>

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
                // case 10:
                //     insertStoreTraceBuffer<8>(drcontext, bb, ins, slot, addr_reg, scratch, trace_buffer_dp1);
                //     break;
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

#ifdef X86
    static __attribute__((always_inline)) void InstrumentReadValueBeforeVGather(void *drcontext, instrlist_t *bb, instr_t *ins, int32_t slot){
        opnd_t opnd = instr_get_src(ins, 0);
        uint32_t operSize = FloatOperandSizeTable(ins, opnd); // VGather's second operand is the memory operand
        uint32_t refSize = opnd_size_in_bytes(opnd_get_size(instr_get_dst(ins, 0)));
        per_thread_t *pt = (per_thread_t *)drmgr_get_tls_field(drcontext, tls_idx);
        instr_t* ins_clone = instr_clone(drcontext, ins);
#ifdef ZEROSPY_DEBUG
        pt->instr_clones->push_back(ins_clone);
#endif
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
        ZerospyInstrument::InstrumentForTracing(drcontext, ilist, where, ref, slot, reg_addr, scratch);
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
    // check if this instruction is actually our interest
    if(!instr_is_app(instr)) return;
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
    per_thread_t *pt = (per_thread_t *)dr_thread_alloc(drcontext, sizeof(per_thread_t));
    if (pt == NULL) {
        ZEROSPY_EXIT_PROCESS("pt == NULL");
    }
    pt->INTRedMap = new RedLogMap();
    pt->FPRedMap = new FPRedLogMap();
#ifdef ZEROSPY_DEBUG
    pt->instr_clones = new vector<instr_t*>();
#endif
    pt->INTRedMap->rehash(10000000);
    pt->FPRedMap->rehash(10000000);
    pt->numInsBuff = dr_get_dr_segment_base(tls_seg);
    BUF_PTR(pt->numInsBuff, void, INSTRACE_TLS_OFFS_BUF_PTR) = 0;
    drmgr_set_tls_field(drcontext, tls_idx, (void *)pt);
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
//#define SKIP_SMALLACCESS
#ifdef SKIP_SMALLACCESS
#define LOGGING_THRESHOLD 100
#endif

// redundant data for a object
struct ObjRedundancy {
    uint64_t objID;
    uint64_t dfreq;
    uint64_t bytes;
};

static inline bool ObjRedundancyCompare(const struct ObjRedundancy &first, const struct ObjRedundancy &second) {
    return first.bytes > second.bytes ? true : false;
}

static inline void PrintSize(file_t gTraceFile, uint64_t size, const char* unit="B") {
    if(size >= (1<<20)) {
        dr_fprintf(gTraceFile, "%lf M%s",(double)size/(double)(1<<20),unit);
    } else if(size >= (1<<10)) {
        dr_fprintf(gTraceFile, "%lf K%s",(double)size/(double)(1<<10),unit);
    } else {
        dr_fprintf(gTraceFile, "%ld %s",size,unit);
    }
}

#define MAX_REDMAP_PRINT_SIZE 128
// only print top 5 redundancy with full redmap to file
#define MAX_PRINT_FULL 5

static uint64_t PrintRedundancyPairs(per_thread_t *pt, uint64_t threadBytesLoad, int threadId) {
    printf("[ZEROSPY INFO] PrintRedundancyPairs for INT...\n");
    vector<ObjRedundancy> tmpList;
    file_t gTraceFile = pt->output_file;
    
    uint64_t grandTotalRedundantBytes = 0;
    dr_fprintf(gTraceFile, "\n--------------- Dumping Data Redundancy Info ----------------\n");
    dr_fprintf(gTraceFile, "\n*************** Dump Data from Thread %d ****************\n", threadId);

    int count=0;
    int rep=-1;
    int total = pt->INTRedMap->size();
    tmpList.reserve(total);
    for(RedLogMap::iterator it = pt->INTRedMap->begin(); it != pt->INTRedMap->end(); ++it) {
        ++count;
        if(100 * count / total!=rep) {
            rep = 100 * count / total;
            printf("\r[ZEROSPY INFO] Stage 1 : %3d %% Finish",rep);
            fflush(stdout);
        }
        ObjRedundancy tmp = {(*it).first, 0, 0};
        for(RedLogSizeMap::iterator it2 = it->second.begin(); it2 != it->second.end(); ++it2) {
            tmp.bytes += it2->second.red;
            if(it2->second.red) {
                bitref_t accmap = &(it2->second.accmap);
                bitref_t redmap = &(it2->second.redmap);
                for(size_t i=0;i<accmap->size;++i) {
                    if(!bitvec_at(accmap, i)) {
                        if(bitvec_at(redmap, i)) ++tmp.dfreq;
                    }
                }
            }
        }
        if(tmp.bytes==0) continue;
        
        grandTotalRedundantBytes += tmp.dfreq;
        tmpList.push_back(tmp); 
    }
    printf("\n[ZEROSPY INFO] Stage 1 finish\n");

    __sync_fetch_and_add(&grandTotBytesRedLoad,grandTotalRedundantBytes);
    dr_fprintf(gTraceFile, "\n Total redundant bytes = %f %%\n", grandTotalRedundantBytes * 100.0 / threadBytesLoad);

    if(grandTotalRedundantBytes==0) {
        dr_fprintf(gTraceFile, "\n------------ Dumping Redundancy Info Finish -------------\n");
        return grandTotalRedundantBytes;
    }

    sort(tmpList.begin(), tmpList.end(), ObjRedundancyCompare);

    int objNum = 0;
    rep = -1;
    total = tmpList.size()<MAX_OBJS_TO_LOG?tmpList.size():MAX_OBJS_TO_LOG;
    for(vector<ObjRedundancy>::iterator listIt = tmpList.begin(); listIt != tmpList.end(); ++listIt) {
        if(objNum++ >= MAX_OBJS_TO_LOG) break;
        if(100 * objNum / total!=rep) {
            rep = 100 * objNum / total;
            printf("\r[ZEROSPY INFO] Stage 2 : %3d %% Finish",rep);
            fflush(stdout);
        }
        if((uint8_t)DECODE_TYPE((*listIt).objID) == DYNAMIC_OBJECT) {
            dr_fprintf(gTraceFile, "\n\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Dynamic Object: ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
            drcctlib_print_full_cct(gTraceFile, DECODE_NAME((*listIt).objID), true, true, MAX_DEPTH);
        } else { 
            dr_fprintf(gTraceFile, "\n\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Static Object: %s ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n", drcctlib_get_str_from_strpool((uint32_t)DECODE_NAME((*listIt).objID)));
        }

        dr_fprintf(gTraceFile, "\n==========================================\n");
        dr_fprintf(gTraceFile, "Redundancy Ratio = %f %% (%ld Bytes, %ld Redundant Load Bytes)\n", (*listIt).dfreq * 100.0 / grandTotalRedundantBytes, (*listIt).dfreq, (*listIt).bytes);

        for(RedLogSizeMap::iterator it2 = (*pt->INTRedMap)[(*listIt).objID].begin(); it2 != (*pt->INTRedMap)[(*listIt).objID].end(); ++it2) {
            uint64_t dfreq = 0;
            uint64_t dread = 0;
            uint64_t dsize = it2->first;
            bitref_t accmap = &(it2->second.accmap);
            bitref_t redmap = &(it2->second.redmap);

            assert(accmap->size==dsize);
            assert(accmap->size==redmap->size);

            for(size_t i=0;i<accmap->size;++i) {
                if(!bitvec_at(accmap, i)) {
                    ++dread;
                    if(bitvec_at(redmap, i)) ++dfreq;
                }
            }
                
            dr_fprintf(gTraceFile, "\n\n======= DATA SIZE : ");
            PrintSize(gTraceFile, dsize);
            dr_fprintf(gTraceFile, "( Not Accessed Data %f %% (%ld Bytes), Redundant Data %f %% (%ld Bytes) )", 
                    (dsize-dread) * 100.0 / dsize, dsize-dread, 
                    dfreq * 100.0 / dsize, dfreq);

            dr_fprintf(gTraceFile, "\n======= Redundant byte map : [0] ");
            uint32_t num=0;
            for(size_t i=0;i<accmap->size;++i) {
                if(!bitvec_at(accmap, i)) {
                    if(bitvec_at(redmap, i)) {
                        dr_fprintf(gTraceFile, "00 ");
                    } else {
                        dr_fprintf(gTraceFile, "XX ");
                    }
                } else {
                    dr_fprintf(gTraceFile, "?? ");
                }
                ++num;
                if(num>MAX_REDMAP_PRINT_SIZE) {
                    dr_fprintf(gTraceFile, "... ");
                    break;
                }
            }
        }
#if 0
        if(objNum<=MAX_PRINT_FULL) {
            char fn[50] = {};
            sprintf(fn,"%lx.redmap",(*listIt).objID);
            file_t fp = dr_open(fn,"w");
            if((uint8_t)DECODE_TYPE((*listIt).objID) == DYNAMIC_OBJECT) {
                dr_fprintf(fp, "\n\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Dynamic Object: ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
                PrintFullCallingContext(DECODE_NAME((*listIt).objID)); // segfault might happen if the shadow memory based data centric is used
            } else  
                dr_fprintf(fp, "\n\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Static Object: %s ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n", GetStringFromStringPool((uint32_t)DECODE_NAME((*listIt).objID)));
            for(size_t i=0;i<accmap->size;++i) {
                if(!bitvec_at(accmap, i)) {
                    if(bitvec_at(redmap, i)) {
                        dr_fprintf(fp, "00 ");
                    } else {
                        dr_fprintf(fp, "XX ");
                    }
                } else {
                    dr_fprintf(fp, "?? ");
                }
            }
        }
#endif
    }
    printf("\n[ZEROSPY INFO] Stage 2 Finish\n");
    dr_fprintf(gTraceFile, "\n------------ Dumping Redundancy Info Finish -------------\n");
    return grandTotalRedundantBytes;
}

static uint64_t PrintApproximationRedundancyPairs(per_thread_t *pt, uint64_t threadBytesLoad, int threadId) {
    printf("[ZEROSPY INFO] PrintRedundancyPairs for FP...\n");
    vector<ObjRedundancy> tmpList;
    file_t gTraceFile = pt->output_file;
    
    uint64_t grandTotalRedundantBytes = 0;
    dr_fprintf(gTraceFile, "\n--------------- Dumping Data Approximation Redundancy Info ----------------\n");
    dr_fprintf(gTraceFile, "\n*************** Dump Data(delta=%.2f%%) from Thread %d ****************\n", delta*100,threadId);

    int count=0;
    int rep=-1;
    int total = pt->FPRedMap->size();
    tmpList.reserve(total);
    for(FPRedLogMap::iterator it = pt->FPRedMap->begin(); it != pt->FPRedMap->end(); ++it) {
        ++count;
        if(100 * count / total!=rep) {
            rep = 100 * count / total;
            printf("\r[ZEROSPY INFO] Stage 1 : %3d %% Finish",rep);
            fflush(stdout);
        }
        ObjRedundancy tmp = {(*it).first, 0, 0};
        for(FPRedLogSizeMap::iterator it2 = it->second.begin(); it2 != it->second.end(); ++it2) {
            tmp.bytes += it2->second.red;
            if(it2->second.red) {
                bitref_t accmap = &(it2->second.accmap);
                bitref_t redmap = &(it2->second.redmap);
                for(size_t i=0;i<accmap->size;++i) {
                    if(!bitvec_at(accmap, i)) {
                        if(bitvec_at(redmap, i)) ++tmp.dfreq;
                    }
                }
            }
        }
        if(tmp.bytes==0) continue;
        grandTotalRedundantBytes += tmp.dfreq;
        tmpList.push_back(tmp); 
    }
    printf("\n[ZEROSPY INFO] Stage 1 Finish\n");

    __sync_fetch_and_add(&grandTotBytesApproxRedLoad,grandTotalRedundantBytes);

    dr_fprintf(gTraceFile, "\n Total redundant bytes = %f %%\n", grandTotalRedundantBytes * 100.0 / threadBytesLoad);

    if(grandTotalRedundantBytes==0) {
        dr_fprintf(gTraceFile, "\n------------ Dumping Approx Redundancy Info Finish -------------\n");
        return grandTotalRedundantBytes;
    }
    sort(tmpList.begin(), tmpList.end(), ObjRedundancyCompare);

    int objNum = 0;
    vector<uint8_t> state;
    for(vector<ObjRedundancy>::iterator listIt = tmpList.begin(); listIt != tmpList.end(); ++listIt) {
        if(objNum++ >= MAX_OBJS_TO_LOG) break;
        if(100 * objNum / total!=rep) {
            rep = 100 * objNum / total;
            printf("\r[ZEROSPY INFO] Stage 2 : %3d %% Finish",rep);
            fflush(stdout);
        }
        if((uint8_t)DECODE_APPROX_TYPE((*listIt).objID) == DYNAMIC_OBJECT) {
            dr_fprintf(gTraceFile, "\n\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Dynamic Object: ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
            drcctlib_print_full_cct(gTraceFile, DECODE_APPROX_NAME((*listIt).objID), true, true, MAX_DEPTH);
        } else {
            dr_fprintf(gTraceFile, "\n\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Static Object: %s ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n", drcctlib_get_str_from_strpool((uint32_t)DECODE_APPROX_NAME((*listIt).objID)));
        }
        dr_fprintf(gTraceFile, "\n==========================================\n");
        dr_fprintf(gTraceFile, "Redundancy Ratio = %f %% (%ld Bytes, %ld Redundant Load Bytes)\n", (*listIt).dfreq * 100.0 / grandTotalRedundantBytes, (*listIt).dfreq, (*listIt).bytes);

        for(FPRedLogSizeMap::iterator it2 = (*pt->FPRedMap)[(*listIt).objID].begin(); it2 != (*pt->FPRedMap)[(*listIt).objID].end(); ++it2) {
            uint64_t dfreq = 0;
            uint64_t dread = 0;
            uint64_t dsize = it2->first;
            uint8_t  dtype = it2->second.typesz;
            bitref_t accmap = &(it2->second.accmap);
            bitref_t redmap = &(it2->second.redmap);

            assert(accmap->size==dsize);
            assert(accmap->size==redmap->size);

            for(size_t i=0;i<accmap->size;++i) {
                if(!bitvec_at(accmap, i)) {
                    ++dread;
                    if(bitvec_at(redmap, i)) ++dfreq;
                    i += dtype-1; // each loop will increament 1
                }
            }
                
            dr_fprintf(gTraceFile, "\n\n======= DATA SIZE : ");
            PrintSize(gTraceFile, dsize, " Elements");
            dr_fprintf(gTraceFile, "( Not Accessed Data %f %% (%ld Reads), Redundant Data %f %% (%ld Reads) )", 
                    (dsize-dread) * 100.0 / dsize, dsize-dread, 
                    dfreq * 100.0 / dsize, dfreq);

            dr_fprintf(gTraceFile, "\n======= Redundant byte map : [0] ");
            uint32_t num=0;
            for(size_t i=0;i<accmap->size;++i) {
                if(!bitvec_at(accmap, i)) {
                    if(bitvec_at(redmap, i)) {
                        dr_fprintf(gTraceFile, "00 ");
                    } else {
                        dr_fprintf(gTraceFile, "XX ");
                    }
                } else {
                    dr_fprintf(gTraceFile, "?? ");
                }
                ++num;
                if(num>MAX_REDMAP_PRINT_SIZE) {
                    dr_fprintf(gTraceFile, "... ");
                    break;
                }
            }
        }
#if 0
        if(objNum<=MAX_PRINT_FULL) {
            char fn[50] = {};
            sprintf(fn,"%lx.redmap",(*listIt).objID);
            file_t fp = dr_open(fn,"w");
            if((uint8_t)DECODE_APPROX_TYPE((*listIt).objID) == DYNAMIC_OBJECT) {
                dr_fprintf(fp, "\n\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Dynamic Object: %lx^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n",(*listIt).objID);
            } else  
                dr_fprintf(fp, "\n\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Static Object: %s, %lx ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n", GetStringFromStringPool((uint32_t)DECODE_APPROX_NAME((*listIt).objID)),(*listIt).objID);
            for(size_t i=0;i<accmap->size;++i) {
                if(!bitvec_at(accmap, i)) {
                    if(bitvec_at(redmap, i)) {
                        dr_fprintf(fp, "00 ");
                    } else {
                        dr_fprintf(fp, "XX ");
                    }
                } else {
                    dr_fprintf(fp, "?? ");
                }
            }
        }
#endif
    }
    printf("\n[ZEROSPY INFO] Stage 2 Finish\n");

    dr_fprintf(gTraceFile, "\n------------ Dumping Approx Redundancy Info Finish -------------\n");
    return grandTotalRedundantBytes;
}
/*******************************************************************/
static void
ClientThreadEndFortrace(void *drcontext) {
    printf("ClientThreadEndFortrace\n");
    handleBufferUpdate();
}

uint64_t getThreadByteLoad(per_thread_t* pt) {
    uint64_t threadByteLoad = 0;

    for(RedLogMap::iterator it = pt->INTRedMap->begin(); it != pt->INTRedMap->end(); ++it) {
        ObjRedundancy tmp = {(*it).first, 0};
        for(RedLogSizeMap::iterator it2 = it->second.begin(); it2 != it->second.end(); ++it2) {
            threadByteLoad += it2->first;
        }
    }

    for(FPRedLogMap::iterator it = pt->FPRedMap->begin(); it != pt->FPRedMap->end(); ++it) {
        ObjRedundancy tmp = {(*it).first, 0};
        for(FPRedLogSizeMap::iterator it2 = it->second.begin(); it2 != it->second.end(); ++it2) {
            threadByteLoad += it2->first;
        }
    }

    return threadByteLoad;
}

static void
ClientThreadEnd(void *drcontext)
{
    printf("ClientThreadEnd\n");
#ifdef TIMING
    uint64_t time = get_miliseconds();
#endif
    per_thread_t *pt = (per_thread_t *)drmgr_get_tls_field(drcontext, tls_idx);
    if (pt->INTRedMap->empty() && pt->FPRedMap->empty()) return;
    uint64_t threadByteLoad = getThreadByteLoad(pt);
    __sync_fetch_and_add(&grandTotBytesLoad,threadByteLoad);
    int threadId = pt->threadId;
    uint64_t threadRedByteLoadINT = 0;
    uint64_t threadRedByteLoadFP = 0;
    if(threadByteLoad != 0) {
        threadRedByteLoadINT = PrintRedundancyPairs(pt, threadByteLoad, threadId);
        threadRedByteLoadFP = PrintApproximationRedundancyPairs(pt, threadByteLoad, threadId);
    }
#ifdef TIMING
    time = get_miliseconds() - time;
    printf("Thread %d: Time %ld ms for generating outputs\n", threadId, time);
#endif

    dr_mutex_lock(gLock);
    dr_fprintf(gFile, "\n#THREAD %d Redundant Read:", threadId);
    dr_fprintf(gFile, "\nTotalBytesLoad: %lu ",threadByteLoad);
    if(threadByteLoad != 0) {
        dr_fprintf(gFile, "\nRedundantBytesLoad: %lu %.2f",threadRedByteLoadINT, threadRedByteLoadINT * 100.0/threadByteLoad);
        dr_fprintf(gFile, "\nApproxRedundantBytesLoad: %lu %.2f\n",threadRedByteLoadFP, threadRedByteLoadFP * 100.0/threadByteLoad);
    }
    dr_mutex_unlock(gLock);

    dr_close_file(pt->output_file);
    delete pt->INTRedMap;
    delete pt->FPRedMap;
#ifdef ZEROSPY_DEBUG
    for(size_t i=0;i<pt->instr_clones->size();++i) {
        instr_destroy(drcontext, (*pt->instr_clones)[i]);
    }
    delete pt->instr_clones;
#endif
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
    sprintf(name + strlen(name), "-%d-zerospy-datacentric", pid);
    g_folder_name.assign(name, strlen(name));
    mkdir(g_folder_name.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    dr_fprintf(STDOUT, "[ZEROSPY INFO] Using Data Centric Zerospy\n");
    dr_fprintf(STDOUT, "[ZEROSPY INFO] Profiling result directory: %s\n", g_folder_name.c_str());

    sprintf(name+strlen(name), "/zerospy.log");
    gFile = dr_open_file(name, DR_FILE_WRITE_OVERWRITE | DR_FILE_ALLOW_LARGE);
    DR_ASSERT(gFile != INVALID_FILE);
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
    dr_close_file(gFile);
    dr_mutex_destroy(gLock);

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

    drcctlib_exit();
    drmgr_analysis_cb_t event_basic_block_ptr = op_enable_sampling.get_value()
    ? event_basic_block<true>
    : event_basic_block<false>;
    if (!drmgr_unregister_thread_init_event(ClientThreadStart) ||
        !drmgr_unregister_thread_exit_event(ClientThreadEnd) ||
        !drmgr_unregister_thread_exit_event(ClientThreadEndFortrace) ||
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

    if (!drmgr_init()|| !trace_init()) {
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

    // drcctlib_init(ZEROSPY_FILTER_READ_MEM_ACCESS_INSTR, INVALID_FILE, InstrumentInsCallback, true/*do data centric*/);
    drcctlib_init_ex(ZEROSPY_FILTER_READ_MEM_ACCESS_INSTR, INVALID_FILE, InstrumentInsCallback, NULL, NULL,
                     DRCCTLIB_COLLECT_DATA_CENTRIC_MESSAGE | DRCCTLIB_CACHE_MODE);
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