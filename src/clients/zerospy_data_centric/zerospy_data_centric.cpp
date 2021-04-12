#include <unordered_map>
#include <vector>
#include <list>
#include <string>
#include <sys/stat.h>
#include <assert.h>
#include <algorithm>

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
// enable data centric with addr info
#define DRCCTLIB_USE_ADDR
#include "drcctlib.h"
#include "utils.h"
#include "bitvec.h"

#define ENABLE_SAMPLING 1
//#define USE_CLEANCALL
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
#ifndef USE_CLEANCALL
    int window_enable;
    int window_disable;
#endif
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
    uint64_t bytesLoad;
#ifdef ENABLE_SAMPLING
    #ifdef USE_CLEANCALL
        long long numIns;
        bool sampleFlag;
    #else
        void* numInsBuff;
    #endif
#endif
#ifdef ZEROSPY_DEBUG
    vector<instr_t*> *instr_clones;
#endif
} per_thread_t;

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
    uint64_t key = MAKE_OBJID(data.object_type,data.sym_name);
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
template<int64_t WINDOW_ENABLE, int64_t WINDOW_DISABLE, bool sampleFlag>
struct BBSample {
    static void update_per_bb(uint instruction_count, app_pc src)
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
        // If the sample flag is changed, flush the region and re-instrument
        if(pt->sampleFlag != sampleFlag) {
            dr_mcontext_t mcontext;
            mcontext.size = sizeof(mcontext);
            mcontext.flags = DR_MC_ALL;
            // flush all fragments in code cache
            dr_flush_region(0, ~((ptr_uint_t)0));
            dr_get_mcontext(drcontext, &mcontext);
            mcontext.pc = src;
            dr_redirect_execution(&mcontext);
        }
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

#define SAMPLED_CLEAN_CALL(insert_clean_call) insert_clean_call

#else
// use manual inlined updates

#define RESERVE_AFLAGS(dead, bb, ins) do { \
    assert(drreg_are_aflags_dead(drcontext, ins, &dead)==DRREG_SUCCESS); \
    if(!dead) assert(drreg_reserve_aflags (drcontext, bb, ins)==DRREG_SUCCESS); \
} while(0)
#define UNRESERVE_AFLAGS(dead, bb, ins) do {if(!dead)  assert(drreg_unreserve_aflags (drcontext, bb, ins)==DRREG_SUCCESS);} while(0)

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
    assert(drreg_reserve_register(drcontext, bb, ins, NULL, &reg_ptr)==DRREG_SUCCESS); \
    dr_insert_read_raw_tls(drcontext, bb, ins, tls_seg, tls_offs + INSTRACE_TLS_OFFS_BUF_PTR, reg_ptr);\
    minstr_load_wint_to_reg(drcontext, ilist, where, reg_val, window_disable);\
    MINSERT(bb, ins, XINST_CREATE_cmp(drcontext, opnd_create_reg(reg_ptr), OPND_CREATE_INT32(window_enable))); \
    instr_t* restore = INSTR_CREATE_label(drcontext); \
    MINSERT(bb, ins, XINST_CREATE_jump_cond(drcontext, DR_PRED_LE, opnd_create_instr(restore))); \
    { insert_clean_call; } \
    MINSERT(bb, ins, restore); \
    assert(drreg_unreserve_register(drcontext, bb, ins, reg_ptr)==DRREG_SUCCESS); \
    UNRESERVE_AFLAGS(dead, bb, ins); \
} while(0)
#endif
#define SAMPLED_CLEAN_CALL(insert_clean_call) insert_clean_call

struct BBSampleInstrument {
    static void bb_flush_code(app_pc src) {
        void *drcontext = dr_get_current_drcontext();
        dr_mcontext_t mcontext;
        mcontext.size = sizeof(mcontext);
        mcontext.flags = DR_MC_ALL;
        // flush all fragments in code cache
        dr_flush_region(0, ~((ptr_uint_t)0));
        dr_get_mcontext(drcontext, &mcontext);
        mcontext.pc = src;
        dr_redirect_execution(&mcontext);
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
        RESERVE_AFLAGS(af_dead, ilist, where);
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
        if(!af_dead ) assert(drreg_restore_app_aflags(drcontext, ilist, where)==DRREG_SUCCESS);
        dr_insert_clean_call(drcontext, ilist, where, (void *)BBSampleInstrument::bb_flush_code, false, 1, OPND_CREATE_INTPTR(instr_get_app_pc(where)));
        MINSERT(ilist, where, restore);
        assert(drreg_unreserve_register(drcontext, ilist, where, reg_ptr)==DRREG_SUCCESS);
        assert(drreg_unreserve_register(drcontext, ilist, where, reg_val)==DRREG_SUCCESS);
        UNRESERVE_AFLAGS(af_dead, ilist, where);
    }

    static void insertBBSampleNoFlush(void* drcontext, instrlist_t *ilist, instr_t *where, int insCnt) {
        static_assert(sizeof(void*)==8);
        bool dead; 
        reg_id_t reg_ptr; 
        RESERVE_AFLAGS(dead, ilist, where);
        reg_id_t reg_val;
        bool regVal_dead = false;
        assert(drreg_reserve_register(drcontext, ilist, where, NULL, &reg_val)==DRREG_SUCCESS);
        assert(drreg_is_register_dead(drcontext, reg_val, where, &regVal_dead)==DRREG_SUCCESS);
        assert(drreg_reserve_register(drcontext, ilist, where, NULL, &reg_ptr)==DRREG_SUCCESS);

        insertBBUpdate(drcontext, ilist, where, insCnt, reg_ptr, reg_val);

        assert(drreg_unreserve_register(drcontext, ilist, where, reg_ptr)==DRREG_SUCCESS);
        assert(drreg_unreserve_register(drcontext, ilist, where, reg_val)==DRREG_SUCCESS);
        UNRESERVE_AFLAGS(dead, ilist, where);
    }
#else
    static void insertBBSample(void* drcontext, instrlist_t *ilist, instr_t *where, int insCnt) {
        static_assert(sizeof(void*)==8);
        per_thread_t *pt = (per_thread_t *)drmgr_get_tls_field(drcontext, tls_idx);
        bool af_dead=false, reg_dead=false; 
        reg_id_t reg_ptr;
        RESERVE_AFLAGS(af_dead, ilist, where);
        assert(drreg_reserve_register(drcontext, ilist, where, NULL, &reg_ptr)==DRREG_SUCCESS);
        assert(drreg_is_register_dead(drcontext, reg_ptr, where, &reg_dead)==DRREG_SUCCESS);
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
        instr_t* restore = INSTR_CREATE_label(drcontext);
        // printf("%ld: %d\n", (int64_t)(BUF_PTR(pt->numInsBuff, void, INSTRACE_TLS_OFFS_BUF_PTR)), IS_SAMPLED(pt, window_enable));
        if(IS_SAMPLED(pt, window_enable)) {
            // fall to code flush when insCnt > WINDOW_ENABLE
            MINSERT(ilist, where, XINST_CREATE_cmp(drcontext, opnd_create_reg(reg_ptr), OPND_CREATE_CCT_INT(window_enable)));
            MINSERT(ilist, where, XINST_CREATE_jump_cond(drcontext, DR_PRED_LE, opnd_create_instr(restore)));
        } else {
            // fall to code flush when insCnt > WINDOW_DISABLE (check if cleared)
            MINSERT(ilist, where, XINST_CREATE_cmp(drcontext, opnd_create_reg(reg_ptr), OPND_CREATE_CCT_INT(0)));
            MINSERT(ilist, where, XINST_CREATE_jump_cond(drcontext, DR_PRED_NZ, opnd_create_instr(restore)));
        }
        if(!af_dead ) assert(drreg_restore_app_aflags(drcontext, ilist, where)==DRREG_SUCCESS);
        if(!reg_dead) assert(drreg_get_app_value(drcontext, ilist, where, reg_ptr, reg_ptr)==DRREG_SUCCESS);
        dr_insert_clean_call(drcontext, ilist, where, (void *)BBSampleInstrument::bb_flush_code, false, 1, OPND_CREATE_INTPTR(instr_get_app_pc(where)));
        MINSERT(ilist, where, restore);
        assert(drreg_unreserve_register(drcontext, ilist, where, reg_ptr)==DRREG_SUCCESS);
        UNRESERVE_AFLAGS(af_dead, ilist, where);
    }

    static void insertBBSampleNoFlush(void* drcontext, instrlist_t *ilist, instr_t *where, int insCnt) {
        static_assert(sizeof(void*)==8);
        bool dead; 
        reg_id_t reg_ptr; 
        RESERVE_AFLAGS(dead, ilist, where);
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
        UNRESERVE_AFLAGS(dead, ilist, where);
    }
#endif
};
// endif USE_CLEANCALL
#endif

static dr_emit_flags_t
event_basic_block(void *drcontext, void *tag, instrlist_t *bb, bool for_trace, bool translating, OUT void **user_data)
{
    int num_instructions = 0;
    instr_t *instr;
    /* count the number of instructions in this block */
    for (instr = instrlist_first(bb); instr != NULL; instr = instr_get_next(instr)) {
        num_instructions++;
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
    if(op_no_flush.get_value()) {
        BBSampleInstrument::insertBBSampleNoFlush(drcontext, bb, instrlist_first(bb), num_instructions);
    } else {
        BBSampleInstrument::insertBBSample(drcontext, bb, instrlist_first(bb), num_instructions);
    }
#endif
    return DR_EMIT_DEFAULT;
}
// endif ENABLE_SAMPLING
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
        pt->bytesLoad += AccessLen;
        data_handle_t data_hndl =
                    drcctlib_get_data_hndl_ignore_stack_data(drcontext, (app_pc)addr);
        if(data_hndl.object_type!=DYNAMIC_OBJECT && data_hndl.object_type!=STATIC_OBJECT) {
            return ;
        }
#ifdef DEBUG_CHECK
        {
            size_t size   = (uint64_t)data_hndl.end_addr - (uint64_t)data_hndl.beg_addr;
            size_t offset = (uint64_t)addr - (uint64_t)data_hndl.beg_addr;
            size_t total  = AccessLen;
            if(offset + total > size) {
                context_handle_t curCtxtHandle = drcctlib_get_context_handle(drcontext, slot);
                printf("\n\nWARN: overflow detected: offset=%ld, total=%ld, size=%ld\n", offset, total, size);
                drcctlib_print_full_cct(STDOUT, curCtxtHandle, true, true, MAX_DEPTH);
            }
        }
#endif
        uint8_t* bytes = static_cast<uint8_t*>(addr);
        if(isApprox) {
            bool hasRedundancy = UnrolledConjunctionApprox<0,AccessLen,sizeof(T)>::BodyHasRedundancy(bytes);
            if(hasRedundancy) {
                uint64_t map = UnrolledConjunctionApprox<0,AccessLen,sizeof(T)>::BodyRedMap(bytes);
                uint32_t zeros = UnrolledConjunctionApprox<0,AccessLen,sizeof(T)>::BodyZeros(bytes);        
                AddToApproximateRedTable((uint64_t)addr,data_hndl, zeros, AccessLen/sizeof(T), map, sizeof(T), pt);
            } else {
                AddToApproximateRedTable((uint64_t)addr,data_hndl, 0, AccessLen/sizeof(T), 0, sizeof(T), pt);
            }
        } else {
            bool hasRedundancy = UnrolledConjunction<0,AccessLen,sizeof(T)>::BodyHasRedundancy(bytes);
            if(hasRedundancy) {
                uint64_t redbyteMap = UnrolledConjunction<0,AccessLen,sizeof(T)>::BodyRedMap(bytes);
                uint32_t redbyteNum = UnrolledConjunction<0,AccessLen,sizeof(T)>::BodyRedNum(redbyteMap);
                AddToRedTable((uint64_t)addr, data_hndl, redbyteNum, AccessLen, redbyteMap, pt);
            } else {
                AddToRedTable((uint64_t)addr, data_hndl, 0, AccessLen, 0, pt);
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
                pt->bytesLoad += sizeof(T);
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
#ifdef ENABLE_SAMPLING
#ifdef USE_CLEANCALL
    if(enable_sampling) {
        if(!pt->sampleFlag) {
            return ;
        }
    }
#endif
#endif
    pt->bytesLoad += accessLen;
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
#ifdef ENABLE_SAMPLING
/* Check if sampling is enabled */
#ifdef ZEROSPY_DEBUG
#define HANDLE_CASE(T, ACCESS_LEN, ELEMENT_LEN, IS_APPROX) \
do { \
if(op_no_flush.get_value()) \
{ SAMPLED_CLEAN_CALL(dr_insert_clean_call(drcontext, bb, ins, (void *)ZeroSpyAnalysis<T, (ACCESS_LEN), (ELEMENT_LEN), (IS_APPROX), true/*sample*/>::CheckNByteValueAfterRead, false, 3, OPND_CREATE_CCT_INT(slot), opnd_create_reg(addr_reg), OPND_CREATE_INTPTR(ins_clone))); } else \
{ dr_insert_clean_call(drcontext, bb, ins, (void *)ZeroSpyAnalysis<T, (ACCESS_LEN), (ELEMENT_LEN), (IS_APPROX), false/* not */>::CheckNByteValueAfterRead, false, 3, OPND_CREATE_CCT_INT(slot), opnd_create_reg(addr_reg), OPND_CREATE_INTPTR(ins_clone)); } } while(0)
#else
#define HANDLE_CASE(T, ACCESS_LEN, ELEMENT_LEN, IS_APPROX) \
do { \
if(op_no_flush.get_value()) \
{ SAMPLED_CLEAN_CALL(dr_insert_clean_call(drcontext, bb, ins, (void *)ZeroSpyAnalysis<T, (ACCESS_LEN), (ELEMENT_LEN), (IS_APPROX), true/*sample*/>::CheckNByteValueAfterRead, false, 2, OPND_CREATE_CCT_INT(slot), opnd_create_reg(addr_reg))); } else \
{ dr_insert_clean_call(drcontext, bb, ins, (void *)ZeroSpyAnalysis<T, (ACCESS_LEN), (ELEMENT_LEN), (IS_APPROX), false/* not */>::CheckNByteValueAfterRead, false, 2, OPND_CREATE_CCT_INT(slot), opnd_create_reg(addr_reg)); } } while(0)

#endif

#define HANDLE_LARGE() \
do { \
if(op_no_flush.get_value()) \
{ SAMPLED_CLEAN_CALL(dr_insert_clean_call(drcontext, bb, ins, (void *)CheckAfterLargeRead<true/*sample*/>, false, 3, OPND_CREATE_CCT_INT(slot), opnd_create_reg(addr_reg), OPND_CREATE_CCT_INT(refSize))); } else \
{ dr_insert_clean_call(drcontext, bb, ins, (void *)CheckAfterLargeRead<false/* not */>, false, 3, OPND_CREATE_CCT_INT(slot), opnd_create_reg(addr_reg), OPND_CREATE_CCT_INT(refSize)); } } while(0)

#ifdef X86
#define HANDLE_VGATHER(T, ACCESS_LEN, ELEMENT_LEN, IS_APPROX, ins) \
do { \
if(op_no_flush.get_value()) \
{ SAMPLED_CLEAN_CALL(dr_insert_clean_call(drcontext, bb, ins, (void *)ZeroSpyAnalysis<T, (ACCESS_LEN), (ELEMENT_LEN), (IS_APPROX), true/*sample*/>::CheckNByteValueAfterVGather, false, 2, OPND_CREATE_CCT_INT(slot), OPND_CREATE_INTPTR(ins_clone))); } else\
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

#include <signal.h>

struct ZerospyInstrument{
    static __attribute__((always_inline)) void InstrumentReadValueBeforeAndAfterLoading(void *drcontext, instrlist_t *bb, instr_t *ins, opnd_t opnd, reg_id_t addr_reg, int32_t slot)
    {
        uint32_t refSize = opnd_size_in_bytes(opnd_get_size(opnd));
        if (refSize==0) {
            // Something strange happened, so ignore it
            assert(0 && "Something strange happened! refSize==0.\n");
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
        ZerospyInstrument::InstrumentReadValueBeforeAndAfterLoading(drcontext, ilist, where, ref, reg1, slot);
    }
}

void
InstrumentInsCallback(void *drcontext, instr_instrument_msg_t *instrument_msg)
{
    // early return when code flush is enabled and not sampled
    per_thread_t *pt = (per_thread_t *)drmgr_get_tls_field(drcontext, tls_idx);
    if( op_enable_sampling.get_value() && 
       !op_no_flush.get_value() && 
       !IS_SAMPLED(pt, window_enable)) {
           return;
    }
    // extract data from drcctprof's given message
    instrlist_t *bb = instrument_msg->bb;
    instr_t *instr = instrument_msg->instr;
    int32_t slot = instrument_msg->slot;
    // check if this instruction is actually our interest
    if(!instr_reads_memory(instr)) return;
    // If this instr is prefetch, the memory address is not guaranteed to be valid, and it may be also ignored by hardware
    // Besides, prefetch instructions are not the actual memory access instructions, so we also ignore them
    if(instr_is_prefetch(instr)) return;

    // Currently, we mark x87 control instructions handling FPU states are ignorable (not insterested)
    if(instr_is_ignorable(instr)) {
        return ;
    }

    // store cct in tls filed
    // InstrumentIns(drcontext, bb, instr, slot);
#ifndef USE_CLEANCALL
    /* We need two scratch registers to get the value */
    reg_id_t reg1, reg2;
    if (drreg_reserve_register(drcontext, bb, instr, NULL, &reg1) != DRREG_SUCCESS 
#ifdef ARM_CCTLIB
        || drreg_reserve_register(drcontext, bb, instr, NULL, &reg2) != DRREG_SUCCESS
#endif
        ) {
        ZEROSPY_EXIT_PROCESS("InstrumentMem drreg_reserve_register != DRREG_SUCCESS");
    }
    instr_t* skipcall = NULL;
    bool dead = false;
    if(op_no_flush.get_value()) {
        RESERVE_AFLAGS(dead, bb, instr);
        dr_insert_read_raw_tls(drcontext, bb, instr, tls_seg, tls_offs + INSTRACE_TLS_OFFS_BUF_PTR, reg1);
        // Clear insCnt when insCnt > WINDOW_DISABLE
#ifdef ARM_CCTLIB
        minstr_load_wint_to_reg(drcontext, bb, instr, reg2, window_enable);
        MINSERT(bb, instr, XINST_CREATE_cmp(drcontext, opnd_create_reg(reg1), opnd_create_reg(reg2)));
#else
        MINSERT(bb, instr, XINST_CREATE_cmp(drcontext, opnd_create_reg(reg1), OPND_CREATE_CCT_INT(window_enable)));
#endif
        skipcall = INSTR_CREATE_label(drcontext);
        MINSERT(bb, instr, XINST_CREATE_jump_cond(drcontext, DR_PRED_GT, opnd_create_instr(skipcall)));
    }
#endif

#ifdef X86
    // gather may result in failure, need special care
    if(instr_is_gather(instr)) {
        // We use instr_compute_address_ex_pos to handle gather (with VSIB addressing)
#ifdef DEBUG
        printf("INFO: HANDLE vgather\n");
#endif
        ZerospyInstrument::InstrumentReadValueBeforeVGather(drcontext, bb, instr, slot);
    } else
#endif
    {
#ifndef ARM_CCTLIB
        if (drreg_reserve_register(drcontext, bb, instr, NULL, &reg2) != DRREG_SUCCESS) {
            ZEROSPY_EXIT_PROCESS("InstrumentMem drreg_reserve_register != DRREG_SUCCESS");
        }
#endif
        int num_srcs = instr_num_srcs(instr);
        for(int i=0; i<num_srcs; ++i) {
            opnd_t opnd = instr_get_src(instr, i);
            if(opnd_is_memory_reference(opnd)) {
                InstrumentMem(drcontext, bb, instr, opnd, slot, reg1, reg2);
            }
        }
#ifndef ARM_CCTLIB
        if (drreg_unreserve_register(drcontext, bb, instr, reg2) != DRREG_SUCCESS) {
            ZEROSPY_EXIT_PROCESS("InstrumentMem drreg_unreserve_register != DRREG_SUCCESS");
        }
#endif
    }
#ifndef USE_CLEANCALL
    if(op_no_flush.get_value()) {
        assert(skipcall!=NULL);
        MINSERT(bb, instr, skipcall);
        UNRESERVE_AFLAGS(dead, bb, instr);
    }
    if (drreg_unreserve_register(drcontext, bb, instr, reg1) != DRREG_SUCCESS 
#ifdef ARM_CCTLIB
        || drreg_unreserve_register(drcontext, bb, instr, reg2) != DRREG_SUCCESS
#endif
        ) {
        ZEROSPY_EXIT_PROCESS("InstrumentMem drreg_unreserve_register != DRREG_SUCCESS");
    }
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
#ifdef ENABLE_SAMPLING
#ifdef USE_CLEANCALL
    pt->numIns = 0;
    pt->sampleFlag = true;
#else
    pt->numInsBuff = dr_get_dr_segment_base(tls_seg);
    BUF_PTR(pt->numInsBuff, void, INSTRACE_TLS_OFFS_BUF_PTR) = 0;
#endif
#endif
    pt->bytesLoad = 0;
    drmgr_set_tls_field(drcontext, tls_idx, (void *)pt);
    ThreadOutputFileInit(pt);
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
        ObjRedundancy tmp = {(*it).first, 0};
        for(RedLogSizeMap::iterator it2 = it->second.begin(); it2 != it->second.end(); ++it2) {
            tmp.bytes += it2->second.red;
        }
        if(tmp.bytes==0) continue;
        grandTotalRedundantBytes += tmp.bytes;
        tmpList.push_back(tmp); 
    }
    printf("\n[ZEROSPY INFO] Stage 1 finish\n");

    __sync_fetch_and_add(&grandTotBytesRedLoad,grandTotalRedundantBytes);
    dr_fprintf(gTraceFile, "\n Total redundant bytes = %f %%\n", grandTotalRedundantBytes * 100.0 / threadBytesLoad);

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
        dr_fprintf(gTraceFile, "Redundancy Ratio = %f %% (%ld Bytes)\n", (*listIt).bytes * 100.0 / grandTotalRedundantBytes, (*listIt).bytes);

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
        ObjRedundancy tmp = {(*it).first, 0};
        for(FPRedLogSizeMap::iterator it2 = it->second.begin(); it2 != it->second.end(); ++it2) {
            tmp.bytes += it2->second.red;
        }
        if(tmp.bytes==0) continue;
        grandTotalRedundantBytes += tmp.bytes;
        tmpList.push_back(tmp); 
    }
    printf("\n[ZEROSPY INFO] Stage 1 Finish\n");

    __sync_fetch_and_add(&grandTotBytesApproxRedLoad,grandTotalRedundantBytes);

    dr_fprintf(gTraceFile, "\n Total redundant bytes = %f %%\n", grandTotalRedundantBytes * 100.0 / threadBytesLoad);
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
        if((uint8_t)DECODE_TYPE((*listIt).objID) == DYNAMIC_OBJECT) {
            dr_fprintf(gTraceFile, "\n\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Dynamic Object: ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
            drcctlib_print_full_cct(gTraceFile, DECODE_NAME((*listIt).objID), true, true, MAX_DEPTH);
        } else {
            dr_fprintf(gTraceFile, "\n\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Static Object: %s ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n", drcctlib_get_str_from_strpool((uint32_t)DECODE_NAME((*listIt).objID)));
        }
        dr_fprintf(gTraceFile, "\n==========================================\n");
        dr_fprintf(gTraceFile, "Redundancy Ratio = %f %% (%ld Bytes)\n", (*listIt).bytes * 100.0 / grandTotalRedundantBytes, (*listIt).bytes);

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
            if((uint8_t)DECODE_TYPE((*listIt).objID) == DYNAMIC_OBJECT) {
                dr_fprintf(fp, "\n\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Dynamic Object: %lx^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n",(*listIt).objID);
            } else  
                dr_fprintf(fp, "\n\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Static Object: %s, %lx ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n", GetStringFromStringPool((uint32_t)DECODE_NAME((*listIt).objID)),(*listIt).objID);
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
ClientThreadEnd(void *drcontext)
{
#ifdef TIMING
    uint64_t time = get_miliseconds();
#endif
    per_thread_t *pt = (per_thread_t *)drmgr_get_tls_field(drcontext, tls_idx);
    if (pt->INTRedMap->empty() && pt->FPRedMap->empty()) return;
    uint64_t threadByteLoad = pt->bytesLoad;
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
        if(op_no_flush.get_value()) {
            dr_fprintf(STDOUT, "[ZEROSPY INFO] Code flush is disabled.\n");
            dr_fprintf(gFile,  "[ZEROSPY INFO] Code flush is disabled.\n");
        }
        dr_fprintf(STDOUT, "[ZEROSPU INFO] Sampling Rate: %.3f, Window Size: %ld\n", conf2rate[op_rate.get_value()], conf2window[op_window.get_value()]);
        dr_fprintf(gFile,  "[ZEROSPU INFO] Sampling Rate: %.3f, Window Size: %ld\n", conf2rate[op_rate.get_value()], conf2window[op_window.get_value()]);
#ifdef USE_CLEANCALL
        BBSample_target[0] = BBSampleTable[op_rate.get_value()][op_window.get_value()][0];
        BBSample_target[1] = BBSampleTable[op_rate.get_value()][op_window.get_value()][1];
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
#ifdef ENABLE_SAMPLING
#ifndef USE_CLEANCALL
    if (!dr_raw_tls_cfree(tls_offs, INSTRACE_TLS_COUNT)) {
        ZEROSPY_EXIT_PROCESS(
            "ERROR: zerospy dr_raw_tls_calloc fail");
    }
#endif
#endif
    drcctlib_exit();
    if (!drmgr_unregister_thread_init_event(ClientThreadStart) ||
        !drmgr_unregister_thread_exit_event(ClientThreadEnd) ||
#ifdef ENABLE_SAMPLING
        // must unregister event after client exit, or it will cause unexpected errors during execution
        ( op_enable_sampling.get_value() && !drmgr_unregister_bb_instrumentation_event(event_basic_block) ) ||
#endif
        !drmgr_unregister_tls_field(tls_idx)) {
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
#ifdef ENABLE_SAMPLING
        // bb instrumentation event must be performed in analysis passes (as we don't change the application codes)
        ( op_enable_sampling.get_value() && !drmgr_register_bb_instrumentation_event(event_basic_block, NULL, NULL) ) ||
#endif
        !drmgr_register_thread_init_event_ex(ClientThreadStart, &thread_init_pri) ||
        !drmgr_register_thread_exit_event_ex(ClientThreadEnd, &thread_exit_pri) ) {
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

    // drcctlib_init(ZEROSPY_FILTER_READ_MEM_ACCESS_INSTR, INVALID_FILE, InstrumentInsCallback, true/*do data centric*/);
    drcctlib_init_ex(ZEROSPY_FILTER_READ_MEM_ACCESS_INSTR, INVALID_FILE, InstrumentInsCallback, NULL, NULL,
                     DRCCTLIB_COLLECT_DATA_CENTRIC_MESSAGE | DRCCTLIB_CACHE_MODE);
    dr_register_exit_event(ClientExit);
}

#ifdef __cplusplus
}
#endif