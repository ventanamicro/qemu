#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "librpmi.h"
#include "system/runstate.h"

void add_sysreset_group(struct rpmi_context *rctx);
void rpmi_do_system_reset(void *priv, rpmi_uint32_t sysreset_type);

const rpmi_uint32_t rpmi_reset_types[2] = {RPMI_SYSRST_TYPE_SHUTDOWN,
    RPMI_SYSRST_TYPE_COLD_REBOOT};

void rpmi_do_system_reset(void *priv, rpmi_uint32_t reset_type)
{
    if (reset_type == RPMI_SYSRST_TYPE_WARM_REBOOT ||
        reset_type == RPMI_SYSRST_TYPE_COLD_REBOOT) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: rebooting..\n",  __func__);
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
    } else if (reset_type == RPMI_SYSRST_TYPE_SHUTDOWN) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Shutting down..\n", __func__);
        exit(0);
    } else {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Invalid reset service Id..\n", __func__);
        return;
    }

    return;
}

struct rpmi_sysreset_platform_ops rpmi_reset_ops = {
    .do_system_reset = rpmi_do_system_reset
};

void add_sysreset_group(struct rpmi_context *rctx)
{
    struct rpmi_service_group *grp;

    grp = rpmi_service_group_sysreset_create(
                                sizeof(rpmi_reset_types) / sizeof(uint32_t),
                                (const rpmi_uint32_t *)&rpmi_reset_types,
                                &rpmi_reset_ops, rctx);

    rpmi_context_add_group(rctx, grp);
}


