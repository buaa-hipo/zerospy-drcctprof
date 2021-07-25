#ifndef _TRACE_H_
#define _TRACE_H_

enum {
    /** Priority of drx_buf thread init event */
    DRMGR_PRIORITY_THREAD_INIT_TRACE_BUF = -7500,
    /** Priority of drx_buf thread exit event */
    DRMGR_PRIORITY_THREAD_EXIT_TRACE_BUF = -7500,
};

#define DRMGR_PRIORITY_NAME_TRACE_BUF_INIT "trace_buf.init"
#define DRMGR_PRIORITY_NAME_TRACE_BUF_EXIT "trace_buf.exit"

bool trace_init(void);
void trace_exit(void);

struct _trace_buf_t;

/** Opaque handle which represents a buffer for use by the trace_buf framework. */
typedef struct _trace_buf_t trace_buf_t;

trace_buf_t *
trace_buf_create_trace_buffer(size_t buffer_size);

bool
trace_buf_free(trace_buf_t *buf);

void
trace_buf_insert_load_buf_ptr(void *drcontext, trace_buf_t *buf, instrlist_t *ilist,
                            instr_t *where, reg_id_t buf_ptr);

void
trace_buf_insert_load_buf_end(void *drcontext, trace_buf_t *buf, instrlist_t *ilist,
                            instr_t *where, reg_id_t buf_ptr);

void
trace_buf_insert_clear_buf(void *drcontext, trace_buf_t *buf, instrlist_t *ilist,
                            instr_t *where, reg_id_t scratch);

void
trace_buf_insert_update_buf_ptr(void *drcontext, trace_buf_t *buf, instrlist_t *ilist,
                              instr_t *where, reg_id_t buf_ptr, reg_id_t scratch,
                              ushort stride);

bool
trace_buf_insert_buf_store(void *drcontext, trace_buf_t *buf, instrlist_t *ilist,
                         instr_t *where, reg_id_t buf_ptr, reg_id_t scratch, opnd_t opnd,
                         opnd_size_t opsz, short offset);
void *
trace_buf_get_buffer_ptr(void *drcontext, trace_buf_t *buf);

void *
trace_buf_get_buffer_end(void *drcontext, trace_buf_t *buf);

void
trace_buf_set_buffer_ptr(void *drcontext, trace_buf_t *buf, void *new_ptr);

void *
trace_buf_get_buffer_base(void *drcontext, trace_buf_t *buf);

size_t
trace_buf_get_buffer_size(void *drcontext, trace_buf_t *buf);

#endif /* _TRACE_H_ */
