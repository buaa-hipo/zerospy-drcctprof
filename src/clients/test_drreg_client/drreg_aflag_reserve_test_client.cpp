#include "dr_api.h"
#include "drmgr.h"
#include "drreg.h"
#include "drutil.h"
#include "drx.h"
#include <assert.h>

#define MINSERT instrlist_meta_preinsert

#define RESERVE_AFLAGS(dc, bb, ins) assert(drreg_reserve_aflags (dc, bb, ins)==DRREG_SUCCESS)
#define UNRESERVE_AFLAGS(dc, bb, ins) assert(drreg_unreserve_aflags (dc, bb, ins)==DRREG_SUCCESS)

#define RESERVE_REG(dc, bb, instr, vec, reg) do {\
    if (drreg_reserve_register(dc, bb, instr, vec, &reg) != DRREG_SUCCESS) { \
        assert(0 && "ERROR: drreg_reserve_register != DRREG_SUCCESS"); \
    } } while(0)
#define UNRESERVE_REG(dc, bb, instr, reg) do { \
    if (drreg_unreserve_register(dc, bb, instr, reg) != DRREG_SUCCESS) { \
        assert(0 && "ERROR: drreg_unreserve_register != DRREG_SUCCESS"); \
    } } while(0)

void ins_print(byte* app_pc) {
    void *drcontext = dr_get_current_drcontext();
    disassemble(drcontext, app_pc, STDOUT);
}

static dr_emit_flags_t
event_basic_block(void *drcontext, void *tag, instrlist_t *bb, bool for_trace, bool translating, OUT void **user_data)
{
    bool early_exit = false;
    for (instr_t* instr = instrlist_first(bb); instr != NULL; instr = instr_get_next(instr)) {
        reg_id_t r;
        bool dead;
        assert(drreg_are_aflags_dead(drcontext, instr, &dead) == DRREG_SUCCESS);
        if(drx_aflags_are_dead(instr)!=dead) {
            dr_fprintf(STDOUT, "AFLAG DEAD ERROR drx(%d), drreg(%d) >> \n\t", drx_aflags_are_dead(instr), dead);
            disassemble(drcontext, instr_get_app_pc(instr), STDOUT);
            early_exit = true;
        }

        RESERVE_AFLAGS(drcontext, bb, instr);
        RESERVE_REG(drcontext, bb, instr, NULL, r);
        // clear the CF flag, it should not effect after aflags unreservation
        MINSERT(bb, instr, INSTR_CREATE_xor(drcontext, opnd_create_reg(reg_64_to_32(r)), opnd_create_reg(reg_64_to_32(r))));
        UNRESERVE_REG(drcontext, bb, instr, r);
        UNRESERVE_AFLAGS(drcontext, bb, instr);
    }
    assert(!early_exit);
    return DR_EMIT_DEFAULT;
}

static void
ClientExit(void)
{
    assert(drmgr_unregister_bb_instrumentation_event(event_basic_block));
    drmgr_exit();
    drx_exit();
    assert(drreg_exit() == DRREG_SUCCESS);
}

#ifdef __cplusplus
extern "C" {
#endif

DR_EXPORT void
dr_client_main(client_id_t id, int argc, const char *argv[])
{
    dr_set_client_name("DynamoRIO Client 'drreg_aflag_reserve_test_client'",
                       "http://dynamorio.org/issues");
    assert(drmgr_init());
    assert(drx_init());
    drreg_options_t ops = { sizeof(ops), 5, false };
    assert(drreg_init(&ops) == DRREG_SUCCESS);
    assert(drmgr_register_bb_instrumentation_event(event_basic_block, NULL, NULL));
    dr_register_exit_event(ClientExit);
}

#ifdef __cplusplus
}
#endif