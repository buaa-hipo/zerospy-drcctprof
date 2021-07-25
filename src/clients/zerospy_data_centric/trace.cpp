/* **********************************************************
 * Copyright (c) 2016-2019 Google, Inc.  All rights reserved.
 * **********************************************************/

/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Google, Inc. nor the names of its contributors may be
 *   used to endorse or promote products derived from this software without
 *   specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL GOOGLE, INC. OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

/* DynamoRio eXtension Buffer Filling API */

#include "dr_api.h"
#include "trace.h"
#include "drmgr.h"
#include "drvector.h"
#include <stddef.h> /* for offsetof */
#include <string.h> /* for memcpy */

#define ALIGNED(x, alignment) ((((ptr_uint_t)x) & ((alignment)-1)) == 0)
#define ALIGN_FORWARD(x, alignment) \
    ((((ptr_uint_t)x) + ((alignment)-1)) & (~((alignment)-1)))
#define ALIGN_BACKWARD(x, alignment) (((ptr_uint_t)x) & (~((ptr_uint_t)(alignment)-1)))

#define TLS_SLOT(tls_base, offs) (void **)((byte *)(tls_base) + (offs))
#define BUF_PTR(tls_base, offs) *(byte **)TLS_SLOT(tls_base, offs)

#define MINSERT instrlist_meta_preinsert

/* denotes the possible buffer types */
typedef enum { BUF_TRACE } trace_buf_type_t;

enum {
    TRACE_BUF_TLS_OFFS_BUF_PTR,
    TRACE_BUF_TLS_OFFS_BUF_END,
    TRACE_BUF_TLS_OFFS_BUF_BASE,
    TRACE_BUF_TLS_COUNT
};

typedef struct {
    byte *seg_base;
    byte *cli_base;    /* the base of the buffer from the client's perspective */
    byte *buf_base;    /* the actual base of the buffer */
    size_t total_size; /* the actual size of the buffer */
} per_thread_t;

struct _trace_buf_t {
    trace_buf_type_t buf_type;
    size_t buf_size;
    uint vec_idx; /* index into the clients vector */
    /* tls implementation */
    int tls_idx;
    uint tls_offs;
    reg_id_t tls_seg;
};

/* holds per-client (also per-buf) information */
static drvector_t clients;
/* A flag to avoid work when no buffers were ever created. */
static bool any_bufs_created;

static trace_buf_t *
trace_buf_init(trace_buf_type_t bt, size_t bsz);

static per_thread_t *
per_thread_init_fault(void *drcontext, trace_buf_t *buf);

static void
trace_buf_insert_update_buf_ptr_fault(void *drcontext, trace_buf_t *buf, instrlist_t *ilist,
                                    instr_t *where, reg_id_t buf_ptr, ushort stride);


static void
event_thread_init(void *drcontext);
static void
event_thread_exit(void *drcontext);

bool
trace_init(void)
{
    drmgr_priority_t exit_priority = { sizeof(exit_priority),
                                       DRMGR_PRIORITY_NAME_TRACE_BUF_EXIT, NULL, NULL,
                                       DRMGR_PRIORITY_THREAD_EXIT_TRACE_BUF };
    drmgr_priority_t init_priority = { sizeof(init_priority),
                                       DRMGR_PRIORITY_NAME_TRACE_BUF_INIT, NULL, NULL,
                                       DRMGR_PRIORITY_THREAD_INIT_TRACE_BUF };

    if (!drvector_init(&clients, 1, false /*!synch*/, NULL) ||
        !drmgr_register_thread_init_event_ex(event_thread_init, &init_priority) ||
        !drmgr_register_thread_exit_event_ex(event_thread_exit, &exit_priority))
        return false;

    return true;
}

void
trace_exit(void)
{
    drmgr_unregister_thread_init_event(event_thread_init);
    drmgr_unregister_thread_exit_event(event_thread_exit);
    drvector_delete(&clients);
}

trace_buf_t *
trace_buf_create_trace_buffer(size_t buf_size)
{
    return trace_buf_init(BUF_TRACE, buf_size);
}

static trace_buf_t *
trace_buf_init(trace_buf_type_t bt, size_t bsz)
{
    trace_buf_t *new_client;
    int tls_idx;
    uint tls_offs;
    reg_id_t tls_seg;

    /* allocate raw TLS so we can access it from the code cache */
    if (!dr_raw_tls_calloc(&tls_seg, &tls_offs, TRACE_BUF_TLS_COUNT, 0))
        return NULL;

    tls_idx = drmgr_register_tls_field();
    if (tls_idx == -1)
        return NULL;

    /* init the client struct */
    new_client = (trace_buf_t*)dr_global_alloc(sizeof(*new_client));
    new_client->buf_type = bt;
    new_client->buf_size = bsz;
    new_client->tls_offs = tls_offs;
    new_client->tls_seg = tls_seg;
    new_client->tls_idx = tls_idx;
    /* We don't attempt to re-use NULL entries (presumably which
     * have already been freed), for simplicity.
     */
    new_client->vec_idx = clients.entries;
    drvector_append(&clients, new_client);

    if (!any_bufs_created)
        any_bufs_created = true;

    return new_client;
}

bool
trace_buf_free(trace_buf_t *buf)
{
    if (!(buf != NULL && (trace_buf_t*)drvector_get_entry(&clients, buf->vec_idx) == buf)) {
        return false;
    }
    /* NULL out the entry in the vector */
    ((trace_buf_t **)clients.array)[buf->vec_idx] = NULL;

    if (!drmgr_unregister_tls_field(buf->tls_idx) || !dr_raw_tls_cfree(buf->tls_offs, TRACE_BUF_TLS_COUNT))
        return false;
    dr_global_free(buf, sizeof(*buf));

    return true;
}

void *
trace_buf_get_buffer_ptr(void *drcontext, trace_buf_t *buf)
{
    per_thread_t *data = (per_thread_t*)drmgr_get_tls_field(drcontext, buf->tls_idx);
    return BUF_PTR(data->seg_base, buf->tls_offs);
}

void *
trace_buf_get_buffer_end(void *drcontext, trace_buf_t *buf)
{
    per_thread_t *data = (per_thread_t*)drmgr_get_tls_field(drcontext, buf->tls_idx);
    return data->cli_base + buf->buf_size;
}

void
trace_buf_set_buffer_ptr(void *drcontext, trace_buf_t *buf, void *new_ptr)
{
    per_thread_t *data = (per_thread_t*)drmgr_get_tls_field(drcontext, buf->tls_idx);
    BUF_PTR(data->seg_base, buf->tls_offs) = (byte*)new_ptr;
}

void *
trace_buf_get_buffer_base(void *drcontext, trace_buf_t *buf)
{
    per_thread_t *data = (per_thread_t*)drmgr_get_tls_field(drcontext, buf->tls_idx);
    return data->cli_base;
}

size_t
trace_buf_get_buffer_size(void *drcontext, trace_buf_t *buf)
{
    return buf->buf_size;
}

void
event_thread_init(void *drcontext)
{
    unsigned int i;
    for (i = 0; i < clients.entries; ++i) {
        per_thread_t *data;
        trace_buf_t *buf = (trace_buf_t*)drvector_get_entry(&clients, i);
        if (buf != NULL) {
            data = per_thread_init_fault(drcontext, buf);
            drmgr_set_tls_field(drcontext, buf->tls_idx, data);
            BUF_PTR(data->seg_base, buf->tls_offs) = data->cli_base;
            BUF_PTR(data->seg_base,
                    buf->tls_offs + sizeof(void *) * TRACE_BUF_TLS_OFFS_BUF_END) =
                data->cli_base + buf->buf_size;
            BUF_PTR(data->seg_base,
                    buf->tls_offs + sizeof(void *) * TRACE_BUF_TLS_OFFS_BUF_BASE) =
                data->cli_base;
        }
    }
}

void
event_thread_exit(void *drcontext)
{
    unsigned int i;
    for (i = 0; i < clients.entries; ++i) {
        trace_buf_t *buf = (trace_buf_t*)drvector_get_entry(&clients, i);
        if (buf != NULL) {
            per_thread_t *data = (per_thread_t*)drmgr_get_tls_field(drcontext, buf->tls_idx);
            byte *cli_ptr = BUF_PTR(data->seg_base, buf->tls_offs);
            dr_raw_mem_free(data->buf_base, data->total_size);
            dr_thread_free(drcontext, data, sizeof(per_thread_t));
        }
    }
}

static per_thread_t *
per_thread_init_fault(void *drcontext, trace_buf_t *buf)
{
    size_t page_size = dr_page_size();
    per_thread_t *per_thread = (per_thread_t*)dr_thread_alloc(drcontext, sizeof(per_thread_t));
    byte *ret;
    /* Keep seg_base in a per-thread data structure so we can get the TLS
     * slot and find where the pointer points to in the buffer.
     */
    per_thread->seg_base = (byte*)dr_get_dr_segment_base(buf->tls_seg);
    /* We construct a buffer right before a fault by allocating as
     * many pages as needed to fit the buffer, plus another read-only
     * page. Then, we return an address such that we have exactly
     * buf_size bytes usable before we hit the ro page.
     */
    /* We no longer use the fault for updating the buffer, 
     * so the extra page will not be allocated and protected.
     */
    per_thread->total_size = ALIGN_FORWARD(buf->buf_size, page_size);
    ret = (byte*)dr_raw_mem_alloc(per_thread->total_size, DR_MEMPROT_READ | DR_MEMPROT_WRITE,
                           NULL);
    per_thread->buf_base = ret;
    per_thread->cli_base = ret + ALIGN_FORWARD(buf->buf_size, page_size) - buf->buf_size;
    return per_thread;
}


void
trace_buf_insert_load_buf_ptr(void *drcontext, trace_buf_t *buf, instrlist_t *ilist,
                            instr_t *where, reg_id_t buf_ptr)
{
    dr_insert_read_raw_tls(drcontext, ilist, where, buf->tls_seg, buf->tls_offs, buf_ptr);
}


void
trace_buf_insert_load_buf_end(void *drcontext, trace_buf_t *buf, instrlist_t *ilist,
                            instr_t *where, reg_id_t buf_ptr)
{
    dr_insert_read_raw_tls(drcontext, ilist, where, buf->tls_seg,
                           buf->tls_offs + sizeof(void *) * TRACE_BUF_TLS_OFFS_BUF_END,
                           buf_ptr);
}


void
trace_buf_insert_clear_buf(void *drcontext, trace_buf_t *buf, instrlist_t *ilist,
                            instr_t *where, reg_id_t scratch)
{
    dr_insert_read_raw_tls(drcontext, ilist, where, buf->tls_seg,
                           buf->tls_offs + sizeof(void *) * TRACE_BUF_TLS_OFFS_BUF_BASE,
                           scratch);
    dr_insert_write_raw_tls(drcontext, ilist, where, buf->tls_seg, buf->tls_offs,
                            scratch);
}


void
trace_buf_insert_update_buf_ptr(void *drcontext, trace_buf_t *buf, instrlist_t *ilist,
                              instr_t *where, reg_id_t buf_ptr, reg_id_t scratch,
                              ushort stride)
{
    /* straightforward, just increment buf_ptr */
    MINSERT(
        ilist, where,
        XINST_CREATE_add(drcontext, opnd_create_reg(buf_ptr), OPND_CREATE_INT16(stride)));
    dr_insert_write_raw_tls(drcontext, ilist, where, buf->tls_seg, buf->tls_offs,
                            buf_ptr);
}

static bool
trace_buf_insert_buf_store_1byte(void *drcontext, trace_buf_t *buf, instrlist_t *ilist,
                               instr_t *where, reg_id_t buf_ptr, reg_id_t scratch,
                               opnd_t opnd, short offset)
{
    instr_t *instr;
    if (!opnd_is_reg(opnd) && !opnd_is_immed(opnd))
        return false;
    if (opnd_is_immed(opnd)) {
#ifdef X86
        instr =
            XINST_CREATE_store_1byte(drcontext, OPND_CREATE_MEM8(buf_ptr, offset), opnd);
#elif defined(AARCHXX)
        /* this will certainly not fault, so don't set a translation */
        MINSERT(ilist, where,
                XINST_CREATE_load_int(drcontext, opnd_create_reg(scratch), opnd));
        instr = XINST_CREATE_store_1byte(drcontext, OPND_CREATE_MEM8(buf_ptr, offset),
                                         opnd_create_reg(scratch));
#else
#    error NYI
#endif
    } else {
        instr =
            XINST_CREATE_store_1byte(drcontext, OPND_CREATE_MEM8(buf_ptr, offset), opnd);
    }
    INSTR_XL8(instr, instr_get_app_pc(where));
    MINSERT(ilist, where, instr);
    return true;
}

static bool
trace_buf_insert_buf_store_2bytes(void *drcontext, trace_buf_t *buf, instrlist_t *ilist,
                                instr_t *where, reg_id_t buf_ptr, reg_id_t scratch,
                                opnd_t opnd, short offset)
{
    instr_t *instr;
    if (!opnd_is_reg(opnd) && !opnd_is_immed(opnd))
        return false;
    if (opnd_is_immed(opnd)) {
#ifdef X86
        instr = XINST_CREATE_store_2bytes(drcontext, OPND_CREATE_MEM16(buf_ptr, offset),
                                          opnd);
#elif defined(AARCHXX)
        /* this will certainly not fault, so don't set a translation */
        MINSERT(ilist, where,
                XINST_CREATE_load_int(drcontext, opnd_create_reg(scratch), opnd));
        instr = XINST_CREATE_store_2bytes(drcontext, OPND_CREATE_MEM16(buf_ptr, offset),
                                          opnd_create_reg(scratch));
#else
#    error NYI
#endif
    } else {
        instr = XINST_CREATE_store_2bytes(drcontext, OPND_CREATE_MEM16(buf_ptr, offset),
                                          opnd);
    }
    INSTR_XL8(instr, instr_get_app_pc(where));
    MINSERT(ilist, where, instr);
    return true;
}

#if defined(X86_64) || defined(AARCH64)
/* only valid on platforms where OPSZ_PTR != OPSZ_4 */
static bool
trace_buf_insert_buf_store_4bytes(void *drcontext, trace_buf_t *buf, instrlist_t *ilist,
                                instr_t *where, reg_id_t buf_ptr, reg_id_t scratch,
                                opnd_t opnd, short offset)
{
    instr_t *instr;
    if (!opnd_is_reg(opnd) && !opnd_is_immed(opnd))
        return false;
    if (opnd_is_immed(opnd)) {
#    ifdef X86_64
        instr = XINST_CREATE_store(drcontext, OPND_CREATE_MEM32(buf_ptr, offset), opnd);
#    elif defined(AARCH64)
        /* this will certainly not fault, so don't set a translation */
        instrlist_insert_mov_immed_ptrsz(drcontext, opnd_get_immed_int(opnd),
                                         opnd_create_reg(scratch), ilist, where, NULL,
                                         NULL);
        instr = XINST_CREATE_store(drcontext, OPND_CREATE_MEM32(buf_ptr, offset),
                                   opnd_create_reg(scratch));
#    endif
    } else {
        instr = XINST_CREATE_store(drcontext, OPND_CREATE_MEM32(buf_ptr, offset), opnd);
    }
    INSTR_XL8(instr, instr_get_app_pc(where));
    MINSERT(ilist, where, instr);
    return true;
}
#endif

static bool
trace_buf_insert_buf_store_ptrsz(void *drcontext, trace_buf_t *buf, instrlist_t *ilist,
                               instr_t *where, reg_id_t buf_ptr, reg_id_t scratch,
                               opnd_t opnd, short offset)
{
    if (!opnd_is_reg(opnd) && !opnd_is_immed(opnd))
        return false;
    if (opnd_is_immed(opnd)) {
        instr_t *first, *last;
        ptr_int_t immed = opnd_get_immed_int(opnd);
#ifdef X86
        instrlist_insert_mov_immed_ptrsz(drcontext, immed,
                                         OPND_CREATE_MEMPTR(buf_ptr, offset), ilist,
                                         where, &first, &last);
        for (;; first = instr_get_next(first)) {
            INSTR_XL8(first, instr_get_app_pc(where));
            if (last == NULL || first == last)
                break;
        }
#elif defined(AARCHXX)
        instr_t *instr;
        instrlist_insert_mov_immed_ptrsz(drcontext, immed, opnd_create_reg(scratch),
                                         ilist, where, &first, &last);
        instr = XINST_CREATE_store(drcontext, OPND_CREATE_MEMPTR(buf_ptr, offset),
                                   opnd_create_reg(scratch));
        INSTR_XL8(instr, instr_get_app_pc(where));
        MINSERT(ilist, where, instr);
#else
#    error NYI
#endif
    } else {
        instr_t *instr =
            XINST_CREATE_store(drcontext, OPND_CREATE_MEMPTR(buf_ptr, offset), opnd);
        INSTR_XL8(instr, instr_get_app_pc(where));
        MINSERT(ilist, where, instr);
    }
    return true;
}


bool
trace_buf_insert_buf_store(void *drcontext, trace_buf_t *buf, instrlist_t *ilist,
                         instr_t *where, reg_id_t buf_ptr, reg_id_t scratch, opnd_t opnd,
                         opnd_size_t opsz, short offset)
{
    switch (opsz) {
    case OPSZ_1:
        return trace_buf_insert_buf_store_1byte(drcontext, buf, ilist, where, buf_ptr,
                                              scratch, opnd, offset);
    case OPSZ_2:
        return trace_buf_insert_buf_store_2bytes(drcontext, buf, ilist, where, buf_ptr,
                                               scratch, opnd, offset);
#if defined(X86_64) || defined(AARCH64)
    case OPSZ_4:
        return trace_buf_insert_buf_store_4bytes(drcontext, buf, ilist, where, buf_ptr,
                                               scratch, opnd, offset);
#endif
    case OPSZ_PTR:
        return trace_buf_insert_buf_store_ptrsz(drcontext, buf, ilist, where, buf_ptr,
                                              scratch, opnd, offset);
    default: return false;
    }
}

static void
insert_load(void *drcontext, instrlist_t *ilist, instr_t *where, reg_id_t dst,
            reg_id_t src, opnd_size_t opsz)
{
    switch (opsz) {
    case OPSZ_1:
        MINSERT(ilist, where,
                XINST_CREATE_load_1byte(
                    drcontext, opnd_create_reg(reg_resize_to_opsz(dst, opsz)),
                    opnd_create_base_disp(src, DR_REG_NULL, 0, 0, opsz)));
        break;
    case OPSZ_2:
        MINSERT(ilist, where,
                XINST_CREATE_load_2bytes(
                    drcontext, opnd_create_reg(reg_resize_to_opsz(dst, opsz)),
                    opnd_create_base_disp(src, DR_REG_NULL, 0, 0, opsz)));
        break;
    case OPSZ_4:
#if defined(X86_64) || defined(AARCH64)
    case OPSZ_8:
#endif
        MINSERT(ilist, where,
                XINST_CREATE_load(drcontext,
                                  opnd_create_reg(reg_resize_to_opsz(dst, opsz)),
                                  opnd_create_base_disp(src, DR_REG_NULL, 0, 0, opsz)));
        break;
    default: DR_ASSERT(false); break;
    }
}