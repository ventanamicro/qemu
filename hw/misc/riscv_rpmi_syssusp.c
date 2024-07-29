#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "librpmi.h"
#include "system/runstate.h"
#include "target/riscv/cpu.h"

void rpmi_do_syssuspend(void *priv, rpmi_uint32_t suspend_type);
int add_syssusp_group(struct rpmi_context *rctx, struct rpmi_hsm *hsm);
bool execute_rpmi_suspend(void *env);

/** RPMI System suspend types */
enum rpmi_sysspnd_suspend_type {
    RPMI_SYSSUSP_SHUTDOWN = 0,
    RPMI_SYSSUSP_COLD_SUSPEND = 1,
    RPMI_SYSSUSP_SUSPEND = 2,
    RPMI_SYSSUSP_MAX_IDN_COUNT,
};

#define RPMI_NUM_SYSSUSP_TYPES 1
const struct rpmi_system_suspend_type syssusp_types[RPMI_NUM_SYSSUSP_TYPES] = {
    {
        .type = RPMI_SYSSUSP_SHUTDOWN
    },
};

/** Prepare for system suspend */
enum rpmi_error system_suspend_prepare(void *priv, rpmi_uint32_t hart_index,
                        const struct rpmi_system_suspend_type *syssusp_type,
                        rpmi_uint64_t resume_addr);
/** Check if the system is ready to suspend */
rpmi_bool_t system_suspend_ready(void *priv, rpmi_uint32_t hart_index);
/** Finalize system suspend */
void system_suspend_finalize(void *priv, rpmi_uint32_t hart_index,
                        const struct rpmi_system_suspend_type *syssusp_type,
                        rpmi_uint64_t resume_addr);
/** Check if the system is ready to resume */
rpmi_bool_t system_suspend_can_resume(void *priv, rpmi_uint32_t hart_index);
/** Resume from system suspend */
enum rpmi_error system_suspend_resume(void *priv, rpmi_uint32_t hart_index,
                        const struct rpmi_system_suspend_type *syssusp_type,
                        rpmi_uint64_t resume_addr);

bool execute_rpmi_suspend(void *env)
{
    riscv_set_wfi_cb(env,  NULL);
    qemu_system_suspend_request();
    return true;
}

struct system_suspend_wakeup_notifier {
    struct rpmi_context *rctx;
    Notifier wakeup;
};

static struct system_suspend_wakeup_notifier *system_suspend_wakeup;

static void system_suspend_wakeup_notify(Notifier *notifier, void *data)
{
    struct rpmi_context *rctx = system_suspend_wakeup->rctx;

    rpmi_context_process_group_events(rctx, RPMI_SRVGRP_SYSTEM_SUSPEND);
}

/** Prepare for system suspend */
enum rpmi_error system_suspend_prepare(void *priv, rpmi_uint32_t hart_index,
                        const struct rpmi_system_suspend_type *syssusp_type,
                        rpmi_uint64_t resume_addr)
{
    CPUState *cpu;
    CPURISCVState *env;

    cpu = cpu_by_arch_id(hart_index);
    env = cpu ? &RISCV_CPU(cpu)->env : NULL;

    if (!system_suspend_wakeup) {
        system_suspend_wakeup = rpmi_env_zalloc(sizeof(*system_suspend_wakeup));
        system_suspend_wakeup->rctx = priv;
        system_suspend_wakeup->wakeup.notify = system_suspend_wakeup_notify;
        qemu_register_wakeup_notifier(&system_suspend_wakeup->wakeup);
    } else {
        g_assert(system_suspend_wakeup->rctx == priv);
    }

    qemu_register_wakeup_support();

    riscv_set_wfi_cb(env,  execute_rpmi_suspend);
    return 0;
}

/** Check if the system is ready to suspend */
rpmi_bool_t system_suspend_ready(void *priv, rpmi_uint32_t hart_index)
{
    return true;
}

/** Finalize system suspend */
void system_suspend_finalize(void *priv, rpmi_uint32_t hart_index,
                        const struct rpmi_system_suspend_type *syssusp_type,
                        rpmi_uint64_t resume_addr)
{
    return;
}

/** Check if the system is ready to resume */
rpmi_bool_t system_suspend_can_resume(void *priv, rpmi_uint32_t hart_index)
{
    return true;
}

/** Resume from system suspend */
enum rpmi_error system_suspend_resume(void *priv, rpmi_uint32_t hart_index,
                        const struct rpmi_system_suspend_type *syssusp_type,
                        rpmi_uint64_t resume_addr)
{
    return 0;
}

struct rpmi_syssusp_platform_ops rpmi_suspend_ops = {
    .system_suspend_prepare = system_suspend_prepare,
    .system_suspend_ready = system_suspend_ready,
    .system_suspend_finalize = system_suspend_finalize,
    .system_suspend_can_resume = system_suspend_can_resume,
    .system_suspend_resume = system_suspend_resume
};

int add_syssusp_group(struct rpmi_context *rctx, struct rpmi_hsm *hsm)
{
    struct rpmi_service_group *grp;

    grp = rpmi_service_group_syssusp_create(hsm,
                                            RPMI_NUM_SYSSUSP_TYPES,
                                            &syssusp_types[0],
                                            &rpmi_suspend_ops, rctx);

    if (!grp) {
        return -1;
    }

    rpmi_context_add_group(rctx, grp);

    return 0;
}
