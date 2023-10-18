/*
 * RISC-V RERI HART Component Emulation
 *
 * Copyright (c) 2023 Ventana Micro Systems, Inc.
 *
 * Author: Himanshu Chauhan <hchauhan@ventanamicro.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#ifndef __RISCV_RERI_HART_DEV_H
#define __RISCV_RERI_HART_DEV_H

#include "qom/object.h"
#include "riscv_reri_helper.h"

#define TYPE_RISCV_RERI_HART_DEV "riscv.reri.hart"

#define RISCV_RERI_HART_STATE(obj) \
    OBJECT_CHECK(RiscvReriState, (obj), TYPE_RISCV_RERI_HART_DEV)

#define RISCV_RERI_MAX_HARTS        64
#define RISCV_RERI_IRQS             2
#define RISCV_RERI_DEV_SIZE         4096

typedef struct RiscvReriState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public>*/
    qemu_irq irqs[RISCV_RERI_IRQS];
    Coroutine *co;
    QemuCoSleep w;
    QemuMutex lock;
    MemoryRegion iomem;
    uint32_t nr_err_banks;
    uint32_t hartid;
    RiscvReriErrorBank *err_banks;
} RiscvReriState;

enum {
    IRQ_LOW_PRIORITY = 0,
    IRQ_HIGH_PRIORITY = 1
};

DeviceState *riscv_reri_hart_create(hwaddr addr, uint32_t hartid);

int riscv_reri_inject(void *opaque, int record, hwaddr addr, uint64_t info);

#endif
