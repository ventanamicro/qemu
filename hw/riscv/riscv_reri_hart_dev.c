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
#include "hw/core/cpu.h"
#include "cpu-qom.h"
#include "hw/irq.h"
#include "hw/riscv/riscv_reri_hart_dev.h"
#include "hw/riscv/riscv_reri_helper.h"


static void coroutine_fn riscv_error_inject(void *opaque);

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

    if (clrsts) {
	    qemu_irq_lower(s->irqs[IRQ_HIGH_PRIORITY]);
    }

    return MEMTX_OK;
}

static void coroutine_fn riscv_error_inject(void *opaque)
{
    RiscvReriState *s = opaque;
    RiscvReriErrorRecord *record;
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
        qemu_irq_raise(s->irqs[IRQ_HIGH_PRIORITY]);
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

static const Property riscv_reri_hart_properties[] = {
    DEFINE_PROP_UINT32("hartid", RiscvReriState, hartid, 0),
};

static void riscv_reri_hart_realize(DeviceState *dev, Error **errp)
{
    RiscvReriState *s = RISCV_RERI_HART_STATE(dev);
    RISCVCPU *cpu;
    CPUState *cpu_state;

#define NR_RECORDS_PER_HART 1

    qdev_init_gpio_out(dev, s->irqs, RISCV_RERI_IRQS);

    cpu_state = cpu_by_arch_id(s->hartid);
    if (!cpu_state)
        return;

    cpu = RISCV_CPU(cpu_state);

    s->err_banks = (RiscvReriErrorBank *)malloc(sizeof(RiscvReriErrorBank));
    memset(s->err_banks, 0, sizeof(RiscvReriErrorBank));
    s->nr_err_banks = 1;

    riscv_reri_register_init(s->err_banks, 0x1af4, 0xabcd, NR_RECORDS_PER_HART);

    memory_region_init_io(&s->iomem, OBJECT(dev), &riscv_reri_hart_ops,
                          s, TYPE_RISCV_RERI_HART_DEV, RISCV_RERI_DEV_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irqs[IRQ_HIGH_PRIORITY]);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irqs[IRQ_LOW_PRIORITY]);

    if (riscv_cpu_claim_interrupts(cpu, MIP_RASHIP) < 0) {
            fprintf(stderr, "%s: Already claimed MIP_RASHIP\n", __func__);
            exit(1);
    }

    if (riscv_cpu_claim_interrupts(cpu, MIP_RASLOP) < 0) {
            fprintf(stderr, "%s: Already claimed MIP_RASLOP\n", __func__);
            exit(1);
    }

    s->co = qemu_coroutine_create(riscv_error_inject, s);
    qemu_mutex_init(&s->lock);
}

static void riscv_reri_hart_reset(Object *obj, ResetType type)
{
}

static void riscv_reri_hart_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = riscv_reri_hart_realize;
    device_class_set_props(dc, riscv_reri_hart_properties);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    rc->phases.enter = riscv_reri_hart_reset;
}

static const TypeInfo riscv_reri_hart_info = {
    .name = TYPE_RISCV_RERI_HART_DEV,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RiscvReriState),
    .class_init = riscv_reri_hart_class_init,
};

DeviceState *riscv_reri_hart_create(hwaddr addr, uint32_t hartid)
{
    DeviceState *dev = qdev_new(TYPE_RISCV_RERI_HART_DEV);
    CPUState *cpu = cpu_by_arch_id(hartid);
    RISCVCPU *rvcpu = RISCV_CPU(cpu);

    assert(!(addr & 0x3));

    qdev_prop_set_uint32(dev, "hartid", hartid);

    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, addr);

    qdev_connect_gpio_out(dev, 0, qdev_get_gpio_in(DEVICE(rvcpu), IRQ_RAS_LOW));
    qdev_connect_gpio_out(dev, 1, qdev_get_gpio_in(DEVICE(rvcpu), IRQ_RAS_HIGH));

    return dev;
}

static void riscv_reri_hart_dev_register_types(void)
{
    type_register_static(&riscv_reri_hart_info);
}

type_init(riscv_reri_hart_dev_register_types);
