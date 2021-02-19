/* 
 *  Copyright (c) 2020-2021 Xuhpclab. All rights reserved.
 *  Licensed under the MIT License.
 *  See LICENSE file for more information.
 */

#include <map>
#include <cstddef>

#include "dr_api.h"
#include "drmgr.h"
#include "drreg.h"
#include "drutil.h"
#include "drcctlib.h"
#include "drcctlib_hpcviewer_format.h"

using namespace std;

#define DRCCTLIB_PRINTF(_FORMAT, _ARGS...) \
    DRCCTLIB_PRINTF_TEMPLATE("reuse_distance_client_cache_hpc_fmt", _FORMAT, ##_ARGS)
#define DRCCTLIB_EXIT_PROCESS(_FORMAT, _ARGS...) \
    DRCCTLIB_CLIENT_EXIT_PROCESS_TEMPLATE("reuse_distance_client_cache_hpc_fmt", _FORMAT, ##_ARGS)

static int tls_idx;

enum {
    INSTRACE_TLS_OFFS_BUF_PTR,
    INSTRACE_TLS_COUNT, /* total number of TLS slots allocated */
};

static reg_id_t tls_seg;
static uint tls_offs;
#define TLS_SLOT(tls_base, enum_val) (void **)((byte *)(tls_base) + tls_offs + (enum_val))
#define BUF_PTR(tls_base, type, offs) *(type **)TLS_SLOT(tls_base, offs)
#define MINSERT instrlist_meta_preinsert

#ifdef ARM_CCTLIB
#    define OPND_CREATE_CCT_INT OPND_CREATE_INT
#else
#    ifdef CCTLIB_64
#        define OPND_CREATE_CCT_INT OPND_CREATE_INT64
#    else
#        define OPND_CREATE_CCT_INT OPND_CREATE_INT32
#    endif
#endif

#ifdef CCTLIB_64
#    define OPND_CREATE_CTXT_HNDL_MEM OPND_CREATE_MEM64
#else
#    define OPND_CREATE_CTXT_HNDL_MEM OPND_CREATE_MEM32
#endif

#define OPND_CREATE_MEM_IDX_MEM OPND_CREATE_MEM64

struct use_node_t {
    context_handle_t create_hndl;
    context_handle_t use_hndl;
    uint64_t last_reuse_mem_idx;

    use_node_t(context_handle_t ch, context_handle_t u, uint64_t m)
        : create_hndl(ch)
        , use_hndl(u)
        , last_reuse_mem_idx(m)
    {
    }
};

struct reuse_node_t {
    context_handle_t create_hndl;
    uint64_t distance;
    uint64_t count;

    reuse_node_t(context_handle_t ch, uint64_t d, uint64_t c)
        : create_hndl(ch)
        , distance(d)
        , count(c)
    {
    }
};

typedef struct _mem_ref_t {
    aligned_ctxt_hndl_t ctxt_hndl;
    app_pc addr;
} mem_ref_t;

typedef struct _output_format_t {
    context_handle_t create_hndl;
    context_handle_t use_hndl;
    context_handle_t reuse_hndl;
    uint64_t count;
    uint64_t distance;
} output_format_t;

typedef struct _per_thread_t {
    uint64_t last_mem_idx;
    uint64_t cur_mem_idx;
    mem_ref_t *cur_buf_list;
    int cur_buf_fill_num;
    void *cur_buf;
    map<uint64_t, use_node_t> *tls_use_map;
    multimap<uint64_t, reuse_node_t> *tls_reuse_map;
    bool sample_mem;
} per_thread_t;

#define TLS_MEM_REF_BUFF_SIZE 100000

// #define SAMPLE_RUN
#ifdef SAMPLE_RUN
#    define UNITE_NUM 1000000000
#    define SAMPLE_NUM 100000000
#endif

#define OUTPUT_SIZE 200
#define REUSED_THRES 8912
#define REUSED_PRINT_MIN_COUNT 1000
#define MAX_CLIENT_CCT_PRINT_DEPTH 10

void
UpdateUseAndReuseMap(void *drcontext, per_thread_t *pt, mem_ref_t *ref, int cur_mem_idx)
{
    map<uint64_t, use_node_t> *use_map = pt->tls_use_map;
    map<uint64_t, use_node_t>::iterator it = (*use_map).find((uint64_t)ref->addr);

    if (it != (*use_map).end()) {
        uint64_t reuse_distance = cur_mem_idx - it->second.last_reuse_mem_idx;
        uint64_t new_pair = (((uint64_t)it->second.use_hndl) << 32) + ref->ctxt_hndl;

        multimap<uint64_t, reuse_node_t> *pair_map = pt->tls_reuse_map;
        multimap<uint64_t, reuse_node_t>::iterator pair_it;
        pair<multimap<uint64_t, reuse_node_t>::iterator,
             multimap<uint64_t, reuse_node_t>::iterator>
            pair_range_it;
        pair_range_it = (*pair_map).equal_range(new_pair);
        for (pair_it = pair_range_it.first; pair_it != pair_range_it.second; ++pair_it) {
            if (pair_it->second.create_hndl == it->second.create_hndl) {
                pair_it->second.count++;
                pair_it->second.distance += reuse_distance;
                break;
            }
        }
        if (pair_it == pair_range_it.second) {
            reuse_node_t val(it->second.create_hndl, reuse_distance, 1);
            (*pair_map).insert(pair<uint64_t, reuse_node_t>(new_pair, val));
        }

        it->second.use_hndl = ref->ctxt_hndl;
        it->second.last_reuse_mem_idx = cur_mem_idx;
    } else {
        data_handle_t data_hndl =
            drcctlib_get_data_hndl_ignore_stack_data(drcontext, ref->addr);
        context_handle_t create_hndl = 0;
        if (data_hndl.object_type == DYNAMIC_OBJECT) {
            create_hndl = data_hndl.path_handle;
        } else if (data_hndl.object_type == STATIC_OBJECT) {
            create_hndl = -data_hndl.sym_name;
        }
        use_node_t new_entry(create_hndl, ref->ctxt_hndl, cur_mem_idx);
        (*use_map).insert(pair<uint64_t, use_node_t>((uint64_t)(ref->addr), new_entry));
    }
}

void
PrintTopN(void *drcontext, per_thread_t *pt, uint32_t print_num)
{
    output_format_t *output_format_list =
        (output_format_t *)dr_global_alloc(print_num * sizeof(output_format_t));
    for (uint32_t i = 0; i < print_num; i++) {
        output_format_list[i].create_hndl = 0;
        output_format_list[i].use_hndl = 0;
        output_format_list[i].reuse_hndl = 0;
        output_format_list[i].count = 0;
        output_format_list[i].distance = 0;
    }
    multimap<uint64_t, reuse_node_t>::iterator it;
    for (it = (*(pt->tls_reuse_map)).begin(); it != (*(pt->tls_reuse_map)).end(); ++it) {
        uint64_t distance = it->second.distance / it->second.count;
        uint64_t count = it->second.count;
        if (distance < REUSED_THRES || count < REUSED_PRINT_MIN_COUNT)
            continue;
        context_handle_t use_hndl = (context_handle_t)(it->first >> 32);
        context_handle_t reuse_hndl = (context_handle_t)(it->first);
        context_handle_t create_hndl = it->second.create_hndl;
        if (create_hndl <= 0) {
            continue;
        }
        if (it->second.count > output_format_list[0].count) {
            uint64_t min_count = output_format_list[1].count;
            uint32_t min_idx = 1;
            for (uint32_t i = 2; i < print_num; i++) {
                if (output_format_list[i].count < min_count) {
                    min_count = output_format_list[i].count;
                    min_idx = i;
                }
            }
            if (it->second.count < min_count) {
                output_format_list[0].count = count;
                output_format_list[0].distance = distance;
                output_format_list[0].reuse_hndl = reuse_hndl;
                output_format_list[0].use_hndl = use_hndl;
                output_format_list[0].create_hndl = create_hndl;
            } else {
                output_format_list[0] = output_format_list[min_idx];
                output_format_list[min_idx].count = count;
                output_format_list[min_idx].distance = distance;
                output_format_list[min_idx].reuse_hndl = reuse_hndl;
                output_format_list[min_idx].use_hndl = use_hndl;
                output_format_list[min_idx].create_hndl = create_hndl;
            }
        }
    }
    output_format_t temp;
    for (uint32_t i = 0; i < print_num; i++) {
        for (uint32_t j = i; j < print_num; j++) {
            if (output_format_list[i].count < output_format_list[j].count) {
                temp = output_format_list[i];
                output_format_list[i] = output_format_list[j];
                output_format_list[j] = temp;
            }
        }
    }

    vector<HPCRunCCT_t *> hpcRunNodes;
    for (uint i = 0; i < print_num; i++) {
        if (output_format_list[i].count <= 0) {
            continue;
        }
        HPCRunCCT_t *hpcRunNode = new HPCRunCCT_t();
        hpcRunNode->ctxt_hndl_list.push_back(output_format_list[i].create_hndl);
        hpcRunNode->ctxt_hndl_list.push_back(output_format_list[i].use_hndl);
        hpcRunNode->ctxt_hndl_list.push_back(output_format_list[i].reuse_hndl);
        hpcRunNode->metric_list.push_back(output_format_list[i].count);
        hpcRunNode->metric_list.push_back(output_format_list[i].distance);
        hpcRunNodes.push_back(hpcRunNode);
    }
    build_thread_custom_cct_hpurun_format(hpcRunNodes, drcontext);
    write_thread_custom_cct_hpurun_format(drcontext);

    dr_global_free(output_format_list, print_num * sizeof(output_format_t));
}

void
ResetPtMap(per_thread_t *pt)
{
    delete pt->tls_use_map;
    pt->tls_use_map = new map<uint64_t, use_node_t>();
}

void
BBStartInsertCleancall(int num)
{
    void *drcontext = dr_get_current_drcontext();
    per_thread_t *pt = (per_thread_t *)drmgr_get_tls_field(drcontext, tls_idx);
    int next_buf_max_idx = pt->cur_buf_fill_num + num;
    if (next_buf_max_idx > TLS_MEM_REF_BUFF_SIZE) {
        pt->cur_mem_idx += pt->cur_buf_fill_num;
#ifdef SAMPLE_RUN
        if (pt->cur_mem_idx % UNITE_NUM <= SAMPLE_NUM) {
            int i = 0;
            if (!pt->sample_mem) {
                i = UNITE_NUM - pt->last_mem_idx % UNITE_NUM;
            }
            for (; i < pt->cur_buf_fill_num; i++) {
                int cur_mem_idx = pt->last_mem_idx + i;
                if (pt->cur_buf_list[i].addr != 0) {
                    UpdateUseAndReuseMap(drcontext, pt, &pt->cur_buf_list[i],
                                         cur_mem_idx);
                }
            }
            pt->sample_mem = true;
        } else if (pt->last_mem_idx % UNITE_NUM <= SAMPLE_NUM) {
            int sample_num = SAMPLE_NUM - pt->last_mem_idx % UNITE_NUM;
            for (int i = 0; i < sample_num; i++) {
                int cur_mem_idx = pt->last_mem_idx + i;
                if (pt->cur_buf_list[i].addr != 0) {
                    UpdateUseAndReuseMap(drcontext, pt, &pt->cur_buf_list[i],
                                         cur_mem_idx);
                }
            }
            pt->sample_mem = true;
        } else if (pt->sample_mem) {
            ResetPtMap(pt);
            pt->sample_mem = false;
        }
#else
        for (int i = 0; i < pt->cur_buf_fill_num; i++) {
            int cur_mem_idx = pt->last_mem_idx + i;
            if (pt->cur_buf_list[i].addr != 0) {
                UpdateUseAndReuseMap(drcontext, pt, &pt->cur_buf_list[i], cur_mem_idx);
            }
        }
#endif
        BUF_PTR(pt->cur_buf, mem_ref_t, INSTRACE_TLS_OFFS_BUF_PTR) = pt->cur_buf_list;
        pt->cur_buf_fill_num = 0;
    }
    pt->last_mem_idx = pt->cur_mem_idx;
    pt->cur_buf_fill_num += num;
}

static void
InstrumentMem(void *drcontext, instrlist_t *ilist, instr_t *where, opnd_t ref,
              reg_id_t reg_ctxt_hndl, reg_id_t free_reg)
{
    /* We need two scratch registers */
    reg_id_t reg_mem_ref_ptr;
    if (drreg_reserve_register(drcontext, ilist, where, NULL, &reg_mem_ref_ptr) !=
        DRREG_SUCCESS) {
        DRCCTLIB_EXIT_PROCESS("InstrumentMem drreg_reserve_register != DRREG_SUCCESS");
    }
    if (!drutil_insert_get_mem_addr(drcontext, ilist, where, ref, free_reg,
                                    reg_mem_ref_ptr)) {
        MINSERT(ilist, where,
                XINST_CREATE_load_int(drcontext, opnd_create_reg(free_reg),
                                      OPND_CREATE_CCT_INT(0)));
    }
    dr_insert_read_raw_tls(drcontext, ilist, where, tls_seg,
                           tls_offs + INSTRACE_TLS_OFFS_BUF_PTR, reg_mem_ref_ptr);
    // store mem_ref_t->addr
    MINSERT(ilist, where,
            XINST_CREATE_store(
                drcontext, OPND_CREATE_MEMPTR(reg_mem_ref_ptr, offsetof(mem_ref_t, addr)),
                opnd_create_reg(free_reg)));
    // store mem_ref_t->ctxt_hndl
    MINSERT(ilist, where,
            XINST_CREATE_store(
                drcontext,
                OPND_CREATE_MEMPTR(reg_mem_ref_ptr, offsetof(mem_ref_t, ctxt_hndl)),
                opnd_create_reg(reg_ctxt_hndl)));

#ifdef ARM_CCTLIB
    MINSERT(ilist, where,
            XINST_CREATE_load_int(drcontext, opnd_create_reg(free_reg),
                                  OPND_CREATE_CCT_INT(sizeof(mem_ref_t))));
    MINSERT(ilist, where,
            XINST_CREATE_add(drcontext, opnd_create_reg(reg_mem_ref_ptr),
                             opnd_create_reg(free_reg)));
#else
    MINSERT(ilist, where,
            XINST_CREATE_add(drcontext, opnd_create_reg(reg_mem_ref_ptr),
                             OPND_CREATE_CCT_INT(sizeof(mem_ref_t))));
#endif
    dr_insert_write_raw_tls(drcontext, ilist, where, tls_seg,
                            tls_offs + INSTRACE_TLS_OFFS_BUF_PTR, reg_mem_ref_ptr);
    /* Restore scratch registers */
    if (drreg_unreserve_register(drcontext, ilist, where, reg_mem_ref_ptr) !=
        DRREG_SUCCESS) {
        DRCCTLIB_EXIT_PROCESS("InstrumentMem drreg_unreserve_register != DRREG_SUCCESS");
    }
}

int
BBMemRefNum(instrlist_t *instrlits)
{
    int num = 0;
    for (instr_t *instr = instrlist_first_app(instrlits); instr != NULL;
         instr = instr_get_next_app(instr)) {
        for (int i = 0; i < instr_num_srcs(instr); i++) {
            if (opnd_is_memory_reference(instr_get_src(instr, i))) {
                num++;
            }
        }
        for (int i = 0; i < instr_num_dsts(instr); i++) {
            if (opnd_is_memory_reference(instr_get_dst(instr, i))) {
                num++;
            }
        }
    }
    return num;
}

void
InstrumentInsCallback(void *drcontext, instr_instrument_msg_t *instrument_msg)
{

    instrlist_t *bb = instrument_msg->bb;
    instr_t *instr = instrument_msg->instr;
    int32_t slot = instrument_msg->slot;
    if (instrument_msg->interest_start) {
        int bb_num = BBMemRefNum(bb);
        dr_insert_clean_call(drcontext, bb, instr, (void *)BBStartInsertCleancall, false,
                             1, OPND_CREATE_CCT_INT(bb_num));
    }

#ifdef x86_CCTLIB
    if (drreg_reserve_aflags(drcontext, bb, instr) != DRREG_SUCCESS) {
        DRCCTLIB_EXIT_PROCESS("instrument_before_every_instr_meta_instr "
                              "drreg_reserve_aflags != DRREG_SUCCESS");
    }
#endif
    reg_id_t reg_ctxt_hndl, reg_temp;
    if (drreg_reserve_register(drcontext, bb, instr, NULL, &reg_ctxt_hndl) !=
            DRREG_SUCCESS ||
        drreg_reserve_register(drcontext, bb, instr, NULL, &reg_temp) != DRREG_SUCCESS) {
        DRCCTLIB_EXIT_PROCESS(
            "InstrumentInsCallback drreg_reserve_register != DRREG_SUCCESS");
    }
    drcctlib_get_context_handle_in_reg(drcontext, bb, instr, slot, reg_ctxt_hndl,
                                       reg_temp);
    for (int i = 0; i < instr_num_srcs(instr); i++) {
        if (opnd_is_memory_reference(instr_get_src(instr, i))) {
            InstrumentMem(drcontext, bb, instr, instr_get_src(instr, i), reg_ctxt_hndl,
                          reg_temp);
        }
    }
    for (int i = 0; i < instr_num_dsts(instr); i++) {
        if (opnd_is_memory_reference(instr_get_dst(instr, i))) {
            InstrumentMem(drcontext, bb, instr, instr_get_dst(instr, i), reg_ctxt_hndl,
                          reg_temp);
        }
    }
    if (drreg_unreserve_register(drcontext, bb, instr, reg_ctxt_hndl) != DRREG_SUCCESS ||
        drreg_unreserve_register(drcontext, bb, instr, reg_temp) != DRREG_SUCCESS) {
        DRCCTLIB_EXIT_PROCESS(
            "InstrumentInsCallback drreg_unreserve_register != DRREG_SUCCESS");
    }
#ifdef x86_CCTLIB
    if (drreg_unreserve_aflags(drcontext, bb, instr) != DRREG_SUCCESS) {
        DRCCTLIB_EXIT_PROCESS("drreg_unreserve_aflags != DRREG_SUCCESS");
    }
#endif
}

static void
ClientThreadStart(void *drcontext)
{
    per_thread_t *pt = (per_thread_t *)dr_thread_alloc(drcontext, sizeof(per_thread_t));
    if (pt == NULL) {
        DRCCTLIB_EXIT_PROCESS("pt == NULL");
    }
    drmgr_set_tls_field(drcontext, tls_idx, (void *)pt);

    pt->cur_buf = dr_get_dr_segment_base(tls_seg);
    pt->cur_buf_list =
        (mem_ref_t *)dr_global_alloc(TLS_MEM_REF_BUFF_SIZE * sizeof(mem_ref_t));
    pt->last_mem_idx = 0;
    pt->cur_mem_idx = 0;
    pt->cur_buf_fill_num = 0;
    BUF_PTR(pt->cur_buf, mem_ref_t, INSTRACE_TLS_OFFS_BUF_PTR) = pt->cur_buf_list;

    pt->tls_use_map = new map<uint64_t, use_node_t>();
    pt->tls_reuse_map = new multimap<uint64_t, reuse_node_t>();
    pt->sample_mem = false;
}

static void
ClientThreadEnd(void *drcontext)
{
    per_thread_t *pt = (per_thread_t *)drmgr_get_tls_field(drcontext, tls_idx);

    BBStartInsertCleancall(TLS_MEM_REF_BUFF_SIZE);
    PrintTopN(drcontext, pt, OUTPUT_SIZE);

    dr_global_free(pt->cur_buf_list, TLS_MEM_REF_BUFF_SIZE * sizeof(mem_ref_t));
    delete pt->tls_use_map;
    delete pt->tls_reuse_map;

    dr_thread_free(drcontext, pt, sizeof(per_thread_t));
}

static void
ClientInit(int argc, const char *argv[])
{
}

static void
ClientExit(void)
{
    if (!dr_raw_tls_cfree(tls_offs, INSTRACE_TLS_COUNT)) {
        DRCCTLIB_EXIT_PROCESS(
            "ERROR: drcctlib_reuse_distance_client_cache_hpc_fmt dr_raw_tls_calloc fail");
    }

    if (!drmgr_unregister_thread_init_event(ClientThreadStart) ||
        !drmgr_unregister_thread_exit_event(ClientThreadEnd) ||
        !drmgr_unregister_tls_field(tls_idx)) {
        DRCCTLIB_PRINTF(
            "ERROR: drcctlib_reuse_distance_client_cache_hpc_fmt failed to unregister in ClientExit");
    }
    drmgr_exit();
    if (drreg_exit() != DRREG_SUCCESS) {
        DRCCTLIB_PRINTF("failed to exit drreg");
    }
    drutil_exit();

    drcctlib_exit();
    hpcrun_format_exit();
}

#ifdef __cplusplus
extern "C" {
#endif

DR_EXPORT void
dr_client_main(client_id_t id, int argc, const char *argv[])
{
    dr_set_client_name("DynamoRIO Client 'drcctlib_reuse_distance_client_cache_hpc_fmt'",
                       "http://dynamorio.org/issues");
    ClientInit(argc, argv);

    if (!drmgr_init()) {
        DRCCTLIB_EXIT_PROCESS(
            "ERROR: drcctlib_reuse_distance_client_cache_hpc_fmt unable to initialize drmgr");
    }
    drreg_options_t ops = { sizeof(ops), 3 /*max slots needed*/, false };
    if (drreg_init(&ops) != DRREG_SUCCESS) {
        DRCCTLIB_EXIT_PROCESS(
            "ERROR: drcctlib_reuse_distance_client_cache_hpc_fmt unable to initialize drreg");
    }
    if (!drutil_init()) {
        DRCCTLIB_EXIT_PROCESS(
            "ERROR: drcctlib_reuse_distance_client_cache_hpc_fmt unable to initialize drutil");
    }
    drmgr_register_thread_init_event(ClientThreadStart);
    drmgr_register_thread_exit_event(ClientThreadEnd);

    tls_idx = drmgr_register_tls_field();
    if (tls_idx == -1) {
        DRCCTLIB_EXIT_PROCESS(
            "ERROR: drcctlib_reuse_distance_client_cache_hpc_fmt drmgr_register_tls_field fail");
    }
    if (!dr_raw_tls_calloc(&tls_seg, &tls_offs, INSTRACE_TLS_COUNT, 0)) {
        DRCCTLIB_EXIT_PROCESS(
            "ERROR: drcctlib_reuse_distance_client_cache_hpc_fmt dr_raw_tls_calloc fail");
    }
    drcctlib_init_ex(DRCCTLIB_FILTER_MEM_ACCESS_INSTR, INVALID_FILE,
                     InstrumentInsCallback, NULL, NULL,
                     DRCCTLIB_COLLECT_DATA_CENTRIC_MESSAGE);
    hpcrun_format_init(dr_get_application_name(), false);
    hpcrun_create_metric("SUM_COUNT");
    hpcrun_create_metric("AVG_DIS");
    dr_register_exit_event(ClientExit);
}

#ifdef __cplusplus
}
#endif