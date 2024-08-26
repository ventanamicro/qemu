#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "librpmi.h"
#include "target/riscv/cpu.h"

int add_hsm_group(struct rpmi_context *rctx, uint64_t harts_mask,
                  uint32_t soc_xport_type, struct rpmi_hsm **hsm_ctx);
void *create_hsm_context(uint64_t harts_mask, void *ctx);
int get_harts_count(uint64_t harts_mask, rpmi_uint32_t *hart_ids);
enum rpmi_hart_hw_state hart_get_hw_state(void *priv, rpmi_uint32_t hart_index);
/** Prepare a hart to start (optional) */
enum rpmi_error hart_start_prepare(void *priv, rpmi_uint32_t hart_index,
                                      rpmi_uint64_t start_addr);

/** Finalize hart stop (optional) */
void hart_start_finalize(void *priv, rpmi_uint32_t hart_index,
                            rpmi_uint64_t start_addr);

/** Perpare a hart to stop (optional) */
enum rpmi_error hart_stop_prepare(void *priv, rpmi_uint32_t hart_index);

/** Finalize hart stop (optional) */
void hart_stop_finalize(void *priv, rpmi_uint32_t hart_index);
bool execute_rpmi_hsm_stop(void *env);
void *get_soc_hsm_context(void);

int g_hsm_contexts;
void *rpmi_hsms[RPMI_SRVGRP_ID_MAX_COUNT];
void *soc_hsm_cntx;

#define RPMI_SUSP_TYPES_COUNT 1
rpmi_uint32_t g_hart_ids[RPMI_SRVGRP_ID_MAX_COUNT][128];

const struct rpmi_hsm_platform_ops hsm_ops = {
    .hart_get_hw_state = hart_get_hw_state,
    .hart_start_prepare = hart_start_prepare,
    .hart_start_finalize = hart_start_finalize,
    .hart_stop_prepare = hart_stop_prepare,
    .hart_stop_finalize = hart_stop_finalize,
};
#define RPMI_MAX_HARTS 128
uint32_t hart_states[RPMI_MAX_HARTS] = {
    [0 ... (RPMI_MAX_HARTS - 1)] = RPMI_HART_HW_STATE_STARTED} ;

/** Get hart HW state (mandatory) */
enum rpmi_hart_hw_state hart_get_hw_state(void *priv, rpmi_uint32_t hart_index)
{
    return hart_states[hart_index];
}

/** Prepare a hart to start (optional) */
enum rpmi_error hart_start_prepare(void *priv, rpmi_uint32_t hart_index,
                                      rpmi_uint64_t start_addr)
{
    CPUState *cpu = cpu_by_arch_id(hart_index);

    cpu->hold_stop = false;
    cpu_resume(cpu);
    hart_states[hart_index] = RPMI_HART_HW_STATE_STARTED;

    return 0;
}

/** Finalize hart stop (optional) */
void hart_start_finalize(void *priv, rpmi_uint32_t hart_index,
                            rpmi_uint64_t start_addr)
{
}

bool execute_rpmi_hsm_stop(void *env)
{
    CPUState *cs = env_cpu(env);

    riscv_set_wfi_cb(env,  NULL);
    cs->stop = true;
    cs->hold_stop = true;
    qemu_cpu_kick(cs);
    hart_states[cs->cpu_index] = RPMI_HART_HW_STATE_STOPPED;

    return true;
}

/** Perpare a hart to stop (optional) */
enum rpmi_error hart_stop_prepare(void *priv, rpmi_uint32_t hart_index)
{
    CPUState *cpu = cpu_by_arch_id(hart_index);
    CPURISCVState *env = &RISCV_CPU(cpu)->env;

    assert(env);
    riscv_set_wfi_cb(env, execute_rpmi_hsm_stop);
    hart_states[hart_index] = RPMI_HART_HW_STATE_STOPPED;

    return 0;
}

/** Finalize hart stop (optional) */
void hart_stop_finalize(void *priv, rpmi_uint32_t hart_index)
{
}

int get_harts_count(uint64_t harts_mask, rpmi_uint32_t *hart_ids)
{
    int harts_count = 0;
    int pos = 0, idx = 0;

    do {

        if (harts_mask & 1) {
            hart_ids[idx] = harts_count;
            idx++;
            harts_count++;
        }

        harts_mask >>= 1;
        pos++;

    } while (harts_mask);

    return harts_count;
}

int add_hsm_group(struct rpmi_context *rctx, uint64_t harts_mask,
                  uint32_t soc_xport_type, struct rpmi_hsm **hsm_ctx)
{
    int harts_count = 0;
    struct rpmi_service_group *grp;
    qemu_log_mask(LOG_GUEST_ERROR, "g_hsm_contextx value -%d\n", g_hsm_contexts);
    if (harts_mask) {
        harts_count = get_harts_count(harts_mask,
                            (rpmi_uint32_t *)&g_hart_ids[g_hsm_contexts]);
        /* Leaf nodes */
        void *hsm_cntx = rpmi_hsm_create(harts_count,
                            (const rpmi_uint32_t *)&g_hart_ids[g_hsm_contexts],
                            0, NULL, &hsm_ops, rctx);
        if (!hsm_cntx) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: allocation failed, harts_count: %x\n",
                          __func__, harts_count);
            return -1;
        }

        if (!hsm_cntx) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: hsm_context_create failed\n ", __func__);
            return -1;
        }

        /* Create and add HSM service group*/
        grp = rpmi_service_group_hsm_create(hsm_cntx);
        if (!grp) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: non leaf hsm grp create failed\n ", __func__);
            return -1;
        }

        rpmi_context_add_group(rctx, grp);

        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: non leaf hsm grp create, cntx: %p, grp: %p\n ",
                      __func__, hsm_cntx, grp);

        if (soc_xport_type) {
            soc_hsm_cntx = hsm_cntx;
        } else {
            qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: NON SOC HSM CONTEXT, cntx: %p, grp: %p\n ",
                      __func__, hsm_cntx, grp);
            rpmi_hsms[g_hsm_contexts] = hsm_cntx;
            g_hsm_contexts++;
        }
        *hsm_ctx = hsm_cntx;
    } else if (soc_xport_type) {
        /* non-leaf nodes */
        soc_hsm_cntx = rpmi_hsm_nonleaf_create(g_hsm_contexts,
                                               (struct rpmi_hsm **)&rpmi_hsms);
        if (!soc_hsm_cntx) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: SoC hsm_context_create failed\n ", __func__);
            return -1;
        }
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: leaf hsm create, cntx: %p\n ",
                      __func__, soc_hsm_cntx);
    }

    return 0;
}

void *get_soc_hsm_context(void)
{
    if (!soc_hsm_cntx) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: SoC SoC hsm_context NULL error\n ", __func__);
            return NULL;
    }

    return soc_hsm_cntx;
}
