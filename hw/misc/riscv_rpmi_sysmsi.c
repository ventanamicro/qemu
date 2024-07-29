#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "hw/misc/riscv_rpmi.h"
#include "librpmi.h"

void add_sysmsi_group(struct rpmi_context *rctx);
struct rpmi_service_group *sysmsi_grp;

static rpmi_bool_t sysmsi_validate_msi_addr(void *priv, rpmi_uint64_t msi_addr)
{
    return true;
}

static struct rpmi_sysmsi_platform_ops sysmsi_ops = {
    .validate_msi_addr = sysmsi_validate_msi_addr,
};

void riscv_rpmi_inject_sysmsi(uint32_t sys_msi_index)
{
    if (!sysmsi_grp)
        return;

    rpmi_service_group_sysmsi_inject(sysmsi_grp, sys_msi_index);
}

void add_sysmsi_group(struct rpmi_context *rctx)
{
    sysmsi_grp = rpmi_service_group_sysmsi_create(RPMI_SYS_NUM_MSI,
                                                  RPMI_SYS_MSI_P2A_DB_INDEX,
                                                  &sysmsi_ops, rctx);

    rpmi_context_add_group(rctx, sysmsi_grp);
}
