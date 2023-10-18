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

#include "qemu/osdep.h"
#include "qemu/coroutine.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/sysbus.h"
#include "target/riscv/cpu.h"
#include "hw/riscv/virt.h"
#include "hw/core/cpu.h"
#include "cpu-qom.h"
#include "hw/irq.h"
#include "hw/riscv/riscv_reri_hart_dev.h"
#include "hw/riscv/riscv_reri_helper.h"

typedef struct RiscvReriState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public>*/
    qemu_irq irqs[2];
    Coroutine *co;
    QemuCoSleep w;
    QemuMutex lock;
    MemoryRegion iomem;
    uint32_t nr_err_banks;
    RiscvReriErrorBank *err_banks;
} RiscvReriState;

enum {
    IRQ_HIGH_PRIORITY = 0,
    IRQ_LOW_PRIORITY = 1
};

DECLARE_INSTANCE_CHECKER(RiscvReriState, RISCV_HART_RERI, TYPE_RISCV_RERI_HART_DEV)

static void coroutine_fn riscv_error_inject(void *opaque);
static RiscvReriState *g_ras_state = NULL;

int riscv_reri_inject(void *opaque, int record, vaddr addr, uint64_t info)
{
    RiscvReriState *s = opaque;
    RiscvReriErrorBank *bank = s->err_banks;
    RiscvReriStatus sts;
    int rc;

    if (record >= s->nr_err_banks)
        return EINVAL;

    sts.value = 0;
    sts.ue = 1;
    sts.at = 3;
    sts.tt = 5;
    sts.iv = 1;

    qemu_mutex_lock(&s->lock);
    rc = riscv_reri_do_inject(&bank->records[record], sts, addr, info);
    qemu_mutex_unlock(&s->lock);
    return rc;
}

static MemTxResult riscv_reri_hart_read_impl(void *opaque, hwaddr addr,
                                             uint64_t *data, unsigned size,
                                             MemTxAttrs attrs)
{
    RiscvReriState *s = opaque;
    int error;

    if (attrs.user) {
        return MEMTX_ERROR;
    }

    qemu_mutex_lock(&s->lock);
    error = riscv_reri_read(s->err_banks, addr, data, size);
    qemu_mutex_unlock(&s->lock);
    if (error != 0) {
        return MEMTX_ERROR;
    }

    return MEMTX_OK;
}

static MemTxResult riscv_reri_hart_write_impl(void *opaque, hwaddr addr,
                                              uint64_t val64, unsigned int size,
                                              MemTxAttrs attrs)
{
    RiscvReriState *s = opaque;
    bool inject = false;
    bool clrsts = false;
    int error;
    CPUState *cpu = cpu_by_arch_id(0);

    if (attrs.user) {
        return MEMTX_ERROR;
    }

    qemu_mutex_lock(&s->lock);
    error = riscv_reri_write(s->err_banks, addr, val64, size, &inject, &clrsts);
    qemu_mutex_unlock(&s->lock);
    if (error != 0) {
        return MEMTX_ERROR;
    }

    if (inject) {
        if (s->w.to_wake != NULL) {
            qemu_co_sleep_wake(&s->w);
        } else {
            qemu_coroutine_enter_if_inactive(s->co);
        }
    }
    if (clrsts)
        riscv_cpu_update_mip(&RISCV_CPU(cpu)->env, MIP_RASHIP, ~(MIP_RASHIP));

    return MEMTX_OK;
}

static void coroutine_fn riscv_error_inject(void *opaque)
{
    RiscvReriState *s = opaque;
    RiscvReriErrorRecord *record;

    CPUState *cpu = cpu_by_arch_id(0);
    int rc;

    record = &s->err_banks->records[0];

    qemu_mutex_lock(&s->lock);
    rc = riscv_error_injection_tick(record);
    while (rc == RISCV_RERI_INJECT_WAIT) {
        qemu_mutex_unlock(&s->lock);
        qemu_co_sleep_ns_wakeable(&s->w, QEMU_CLOCK_VIRTUAL, 1000000);
        qemu_mutex_lock(&s->lock);
        rc = riscv_error_injection_tick(record);
    }
    qemu_mutex_unlock(&s->lock);

    if (rc == RISCV_RERI_INJECT_LOW) {
        qemu_irq_raise(s->irqs[IRQ_LOW_PRIORITY]);
    } else if (rc == RISCV_RERI_INJECT_HIGH) {
        riscv_cpu_update_mip(&RISCV_CPU(cpu)->env, MIP_RASHIP, (MIP_RASHIP));
    }
}

static const MemoryRegionOps riscv_reri_hart_ops = {
    .read_with_attrs = riscv_reri_hart_read_impl,
    .write_with_attrs = riscv_reri_hart_write_impl,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8
    }
};

static void riscv_reri_hart_instance_init(Object *obj)
{
    RiscvReriState *s;
    int i;
    g_ras_state = RISCV_HART_RERI(obj);
    RISCVCPU *cpu = RISCV_CPU(cpu_by_arch_id(0));
    CPUState *cpu_state;

#define NR_RECORDS_PER_HART 1

    if (!g_ras_state)
            return;

    for (i = 0; i < VIRT_CPUS_MAX; i++) {
        cpu_state = cpu_by_arch_id(i);
        if (!cpu_state)
            continue;

        cpu = RISCV_CPU(cpu_by_arch_id(i));
        s = &g_ras_state[i];
        s->err_banks = (RiscvReriErrorBank *)malloc(sizeof(RiscvReriErrorBank));
        memset(s->err_banks, 0, sizeof(RiscvReriErrorBank));
        s->nr_err_banks = 1;

        riscv_reri_register_init(s->err_banks, 0x1af4, 0xabcd, NR_RECORDS_PER_HART);

        memory_region_init_io(&s->iomem, obj, &riscv_reri_hart_ops,
                              s, TYPE_RISCV_RERI_HART_DEV, 0x1000);
        sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);

        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irqs[IRQ_HIGH_PRIORITY]);
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irqs[IRQ_LOW_PRIORITY]);

        if (riscv_cpu_claim_interrupts(cpu, MIP_RASHIP) < 0) {
            fprintf(stderr, "%s: Already claimed MIP_RASHIP\n", __func__);
            exit(1);
        }
        s->co = qemu_coroutine_create(riscv_error_inject, s);
        qemu_mutex_init(&s->lock);
    }
}

static const TypeInfo riscv_reri_hart_info = {
    .name = TYPE_RISCV_RERI_HART_DEV,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RiscvReriState) * VIRT_CPUS_MAX,
    .instance_init = riscv_reri_hart_instance_init,
};

static void riscv_reri_hart_dev_register_types(void)
{
    type_register_static(&riscv_reri_hart_info);
}

type_init(riscv_reri_hart_dev_register_types);

DeviceState *riscv_create_harts_reri_dev(hwaddr addr)
{
    DeviceState *dev = qdev_new(TYPE_RISCV_RERI_HART_DEV);
    SysBusDevice *s = SYS_BUS_DEVICE(dev);

    sysbus_mmio_map(s, 0, addr);
    dev->id = g_strdup(TYPE_RISCV_RERI_HART_DEV);

    sysbus_realize_and_unref(s, &error_fatal);

    return dev;
}
