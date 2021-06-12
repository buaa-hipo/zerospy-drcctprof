#ifndef __DETECT_H__
#define __DETECT_H__
#include "dr_api.h"
#include "drmgr.h"
#include "drreg.h"
#include "drutil.h"
#include "dr_tools.h"
#include <sys/time.h>
#include "utils.h"
#include "drx.h"
// reg_addr and reg_val may overlapped
template<bool cache>
void insertLoadAndComputeRedMap_1byte(  void* drcontext, instrlist_t *ilist, instr_t *where, IN reg_id_t reg_base, 
                                        IN reg_id_t reg_addr, int offset, 
                                        OUT reg_id_t reg_val, OUT reg_id_t scratch) 
{
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
#ifdef USE_CACHE
    if(cache) {
        MINSERT(ilist, where, XINST_CREATE_store(drcontext,
                        OPND_CREATE_MEM64(reg_base, offsetof(val_cache_t, redmap)),
                        opnd_create_reg(reg_val)));
    }
#endif
    // printf("insertLoadAndComputeRedMap_1byte exit\n"); fflush(stdout);
}

template<bool cache>
void insertLoadAndComputeRedMap_2bytes( void* drcontext, instrlist_t *ilist, instr_t* where, IN reg_id_t reg_base, 
                                        IN reg_id_t reg_addr, int offset, 
                                        OUT reg_id_t reg_val, OUT reg_id_t scratch) 
{
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
#ifdef USE_CACHE
    // Store back
    if(cache) {
        MINSERT(ilist, where, XINST_CREATE_store(drcontext,
                        OPND_CREATE_MEM64(reg_base, offsetof(val_cache_t, redmap)),
                        opnd_create_reg(reg_val)));
    }
#endif
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
#ifdef USE_CACHE
    // Store back
    if(cache) {
        MINSERT(ilist, where, XINST_CREATE_store(drcontext,
                        OPND_CREATE_MEM64(reg_base, offsetof(val_cache_t, redmap)),
                        opnd_create_reg(reg_val)));
    }
#endif
    // printf("insertLoadAndComputeRedMap_4bytes exit\n"); fflush(stdout);
}

template<bool cache>
void insertLoadAndComputeRedMap_8bytes( void* drcontext, instrlist_t *ilist, instr_t *where, IN reg_id_t reg_base, 
                                        IN reg_id_t reg_addr, int offset, 
                                        OUT reg_id_t reg_val, OUT reg_id_t scratch) 
{
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
#ifdef USE_CACHE
    // Store back
    if(cache) {
        // printf("Caching! 32bit: %d, 64bit: %d\n", reg_is_32bit(reg_val), reg_is_64bit(reg_val));
        MINSERT(ilist, where, XINST_CREATE_store(drcontext,
                        OPND_CREATE_MEM64(reg_base, offsetof(val_cache_t, redmap)),
                        opnd_create_reg(reg_val)));
    }
#endif
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
#ifdef USE_CACHE
    // Store back
    MINSERT(ilist, where, XINST_CREATE_store(drcontext,
                OPND_CREATE_MEM64(reg_base, offsetof(val_cache_t, redmap)),
                opnd_create_reg(reg_addr)));
#endif
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
            insertLoadAndComputeRedMap_4bytes<false>(drcontext, ilist, where, reg_base, reg_addr, 0, reg_addr, scratch);
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
#ifdef USE_CACHE
                MINSERT(ilist, where, XINST_CREATE_store(drcontext,
                            OPND_CREATE_MEM64(reg_base, offsetof(val_cache_t, redmap)),
                            opnd_create_reg(reg_redmap)));
#endif
                UNRESERVE_REG(drcontext, ilist, where, reg_val);
                UNRESERVE_REG(drcontext, ilist, where, reg_redmap);
                printf("insertComputeAndCacheRedmap_INT default: accessLen=%d exit\n", accessLen); fflush(stdout);
            }
    }
    return ;
}
#endif