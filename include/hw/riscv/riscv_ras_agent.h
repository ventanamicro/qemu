/*
 * RISC-V RAS (Reliability, Availability and Serviceability)
 *
 * Copyright (c) 2023 Ventana Micro Systems, Inc.
 *
 * Author:
 * Himanshu Chauhan <hchauhan@ventanamicro.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#ifndef _RISCV_RAS_AGENT_H
#define _RISCV_RAS_AGENT_H

#include "hw/misc/rpmi_msgprot.h"
#include "riscv_reri_regs.h"

/* FIXME: Encode */
#define RAS_AGENT_VERSION    1

typedef struct RasErrorSource {
    unsigned int sse_vector;
    hwaddr as;
    hwaddr size;
} RasErrorSource;

int riscv_ras_agent_init(RasErrorSource *sources, uint32_t nr_sources);
int ras_get_agent_version(void);
int riscv_ras_agent_synchronize_errors(int hart_id,
                                       struct rpmi_ras_sync_err_resp *resp);
int riscv_ras_agent_synchronize_hart_errors(int hart_id,
                                            struct rpmi_ras_sync_err_resp *resp);

#endif /* _RISCV_RAS_AGENT_H */
