#include <stdint.h>
#include <assert.h>
#include <stdlib.h>
#include "dr_api.h"
#include <unordered_map>
/************************************************/
/****************** Bit Vector ******************/
struct bitvec_t { 
    union {
        uint64_t stat; // static 64 bits small cases
        std::unordered_map<size_t, uint64_t>* dyn; // dynamic allocate memory for bitvec larger than 64
    } data;
    size_t size;
#ifdef DEBUG
    size_t capacity;
#endif
};
typedef bitvec_t* bitref_t;
inline void bitvec_alloc(bitref_t bitref, size_t size) {
    bitref->size = size;
    if(size>64) {
        size_t capacity = size <= dr_page_size() ? size : dr_page_size();
        /* FIXME i#5: Although upper bound of size/64 is usually enough, 
         * the compiler may generate overflowed memory access at the end 
         * of not-aligned data (maybe for better performance?). Here we 
         * just hot-fix the case by allocating one more capacity to avoid 
         * this kind of overflow. */
#ifdef DEBUG
        bitref->capacity = (size+63)/64 + 1;
#endif
        //assert(bitref->capacity > 0);
        // Only Dynamic Malloc for large cases (>64 Bytes)
        bitref->data.dyn = new std::unordered_map<size_t, uint64_t>();
        bitref->data.dyn->rehash(capacity);
        assert(bitref->data.dyn!=NULL);
        // // TODO: may be slow, use avx
        // memset(bitref->data.dyn, -1, sizeof(uint64_t)*(size+63)/64);
    } else {
#ifdef DEBUG
        bitref->capacity = 1;
#endif
        bitref->data.stat = -1; // 0xffffffffffffffffLL;
    }
}

inline void bitvec_free(bitref_t bitref) {
    if(bitref->size>64) {
        free(bitref->data.dyn);
    }
}

inline void bitvec_and(bitref_t bitref, uint64_t val, size_t offset, size_t size) {
    if(bitref->size>64) {
        // if(offset+size>bitref->size) {
        //     printf("%ld %ld %ld %ld %p %lx\n", bitref->size, bitref->capacity, offset, size, bitref->data.dyn, val); fflush(stdout);
        // }
        // assert(offset+size<=bitref->size);
        size_t bytePos = offset / 64;
        size_t bitPos = offset % 64;
        size_t rest = 64-bitPos;
#ifdef DEBUG
        assert(bytePos<bitref->capacity);
#endif
        if(rest<size) {
#ifdef DEBUG
            if(bytePos+1>=bitref->capacity) {
                printf("bitPos=%ld, bytePos=%ld, capacity=%ld, rest=%ld, size=%ld\n", bitPos, bytePos, bitref->capacity, rest, size);
                fflush(stdout);
            }
            assert(bytePos+1<bitref->capacity);
#endif
            register uint64_t mask = (0x1LL << (size-rest)) - 1;
            mask = ~mask;
            mask = ((val>>rest)|mask);
            auto it = bitref->data.dyn->find(bytePos+1);
            if(it==bitref->data.dyn->end()) {
                (*(bitref->data.dyn))[bytePos+1] = mask;
            } else {
                it->second &= mask;
            }
            size = rest;
        }
        register uint64_t mask = (0x1LL << size) - 1;
        mask = mask << bitPos;
        mask = ~mask;
        mask = ((val<<bitPos)|mask);
        auto it = bitref->data.dyn->find(bytePos+1);
        if(it==bitref->data.dyn->end()) {
            (*(bitref->data.dyn))[bytePos] = mask;
        } else {
            it->second &= mask;
        }
    } else {
        assert(offset<64);
        register uint64_t mask = (0x1LL << size) - 1;
        mask = mask << offset;
        mask = ~mask;
        bitref->data.stat &= ((val<<offset)|mask);
    }
}

inline bool bitvec_at(bitref_t bitref, size_t pos) {
    if(bitref->size>64) {
        size_t bytePos = pos / 64;
        size_t bitPos = pos % 64;
#ifdef DEBUG
        assert(bytePos<bitref->capacity);
#endif
        auto it = bitref->data.dyn->find(bytePos);
        if(it==bitref->data.dyn->end()) {
            return true;
        } else {
            if(bitPos!=0) {
                return (it->second & (0x1LL << bitPos))!=0 ? true : false;
            } else {
                return (it->second & (0x1LL))!=0 ? true : false;
            }
        }
    } else {
        return (bitref->data.stat & (0x1LL << pos))!=0 ? true : false;
    }
}
/************************************************/