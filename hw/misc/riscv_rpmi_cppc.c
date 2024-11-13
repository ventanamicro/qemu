#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "librpmi.h"
#include "system/runstate.h"
#include "target/riscv/cpu.h"

target_ulong helper_csrr(CPURISCVState *env, int csr);

enum rpmi_error cppc_get_reg(void *priv, rpmi_uint32_t hart_id,
                             rpmi_uint32_t reg_id, rpmi_uint64_t *val);

enum rpmi_error cppc_set_reg(void *priv, rpmi_uint32_t hart_id,
                             rpmi_uint32_t reg_id, rpmi_uint64_t val);

enum rpmi_error cppc_update_perf(void *priv, rpmi_uint32_t desired_perf,
                                 rpmi_uint32_t hart_index);

enum rpmi_error cppc_get_current_freq(void *priv, rpmi_uint32_t hart_index,
                                      rpmi_uint64_t *cur_freq);

int add_cppc_group(struct rpmi_context *rctx,
                   struct rpmi_shmem *shmem,
                   struct rpmi_hsm *hsm,
                   uint64_t harts_mask,
                   uint64_t perf_request_shmem_offset,
                   uint64_t perf_feedback_shmem_offset);

enum rpmi_error cppc_get_reg(void *priv, rpmi_uint32_t reg_id,
                             rpmi_uint32_t hart_id, rpmi_uint64_t *val)
{
    CPUState *cpu = cpu_by_arch_id(hart_id);
    CPURISCVState *env = &RISCV_CPU(cpu)->env;
    uint64_t mcycle = 50;

    mcycle = helper_csrr(env, CSR_TIME);

    switch (reg_id) {
    case RPMI_CPPC_DELIVERED_PERF_COUNTER:
        /* dummy delivered performance */
        *val = mcycle * (4);
        break;

    case RPMI_CPPC_REFERENCE_PERF_COUNTER:
        *val = mcycle;
        break;
    }

    return RPMI_SUCCESS;
}

enum rpmi_error cppc_set_reg(void *priv, rpmi_uint32_t reg_id,
                             rpmi_uint32_t hart_id, rpmi_uint64_t val)
{
    return RPMI_SUCCESS;
}

enum rpmi_error cppc_update_perf(void *priv, rpmi_uint32_t desired_perf,
                                 rpmi_uint32_t hart_index)
{
    return RPMI_SUCCESS;
}

enum rpmi_error cppc_get_current_freq(void *priv, rpmi_uint32_t hart_index,
                                      rpmi_uint64_t *cur_freq)
{
    /* dummy value */
    *cur_freq = 0xdeadbeeffeedbead;
    return 0;
}

struct rpmi_cppc_platform_ops ops = {
    .cppc_get_reg = cppc_get_reg,
    .cppc_set_reg = cppc_set_reg,
    .cppc_update_perf = cppc_update_perf,
    .cppc_get_current_freq = cppc_get_current_freq,
};

int add_cppc_group(struct rpmi_context *rctx,
                   struct rpmi_shmem *shmem,
                   struct rpmi_hsm *hsm,
                   uint64_t harts_mask,
                   uint64_t perf_request_shmem_offset,
                   uint64_t perf_feedback_shmem_offset)

{
    struct rpmi_cppc_regs cppc_regs;

    if (!rctx || !hsm || !harts_mask) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: invalid parameters rctx: %p, hsm: %p, harts_mask: 0x%lx\n",
                      __func__, rctx, hsm, (unsigned long)harts_mask);
        return -1;
    }

    cppc_regs.highest_perf = 5;
    cppc_regs.nominal_perf  = 4;
    cppc_regs.lowest_nonlinear_perf = 2;
    cppc_regs.lowest_perf = 2;
    cppc_regs.reference_perf = 1;
    cppc_regs.lowest_freq = 40;
    cppc_regs.nominal_freq = 80;
    cppc_regs.perf_limited = 0;

    struct rpmi_service_group *group = rpmi_service_group_cppc_create(hsm,
                                                    &cppc_regs,
                                                    RPMI_CPPC_PASSIVE_MODE,
                                                    shmem,
                                                    perf_request_shmem_offset,
                                                    perf_feedback_shmem_offset,
                                                    &ops,
                                                    NULL);
    if (!group) {
        return -1;
    }

    rpmi_context_add_group(rctx, group);

    return 0;
}
