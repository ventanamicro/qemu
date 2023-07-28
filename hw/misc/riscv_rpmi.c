 /*
  * riscv_rpmi.c
  * RPMI transport IO handling routines.
  *
  * Copyright (c) 2024 Ventana Micro Systems Inc.
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation; either version 2 of the License, or
  * (at your option) any later version.

  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.

  * You should have received a copy of the GNU General Public License along
  * with this program; if not, see <http://www.gnu.org/licenses/>.
  */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "hw/misc/riscv_rpmi.h"
#include "hw/boards.h"
#include "system/address-spaces.h"
#include "hw/qdev-properties.h"
#include "system/runstate.h"
#include "librpmi.h"

#define PLAT_INFO      "Ventana Veyron Plat 1.0"
#define PLAT_INFO_LEN  (sizeof(PLAT_INFO))

int g_contexts;
#define MAX_RPMI_XPORTS 16
struct rpmi_context *rpmi_contexts[MAX_RPMI_XPORTS];

int init_rpmi_svc_groups(hwaddr shm_addr, int shm_sz,
                         uint32_t a2preq_qsz, uint32_t p2areq_qsz,
                         uint64_t harts_mask, uint32_t soc_xport_type);
struct rpmi_shmem *rpmi_shmem_qemu_create(const char *name, rpmi_uint64_t base,
                                            rpmi_uint32_t size);

void handle_rpmi_event(void)
{
    int i;

    for (i = 0; i < g_contexts; i++) {
        struct rpmi_context *rpmi_context = rpmi_contexts[i];
        if (rpmi_context) {
            rpmi_context_process_a2p_request(rpmi_context);
            rpmi_context_process_all_events(rpmi_context);
        } else {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: Doorbell event, but context not initialized!\n",
                          __func__);
        }
    }

    return;
}

static uint64_t riscv_rpmi_read(void *opaque, hwaddr offset, unsigned int size)
{
    struct RiscvRpmiState *s = opaque;
    if ((size != 4) || (offset != 0)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "riscv_rpmi_read: Invalid read access to "
                      "addr %" HWADDR_PRIx ", size: %x\n",
                      offset, size);
        return 0;

    } else {
        return s->doorbell;
    }
}

static void riscv_rpmi_write(void *opaque, hwaddr offset,
                uint64_t val64, unsigned int size)
{
    struct RiscvRpmiState *s = opaque;
    if ((size != 4) || (offset != 0)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "riscv_rpmi_write: Invalid write access to "
                      "addr %" HWADDR_PRIx ", size: %x\n",
                      offset, size);
        return;
    }

    s->doorbell = val64;
    if (val64 == 1) {
        handle_rpmi_event();

        /* clear the doorbell register */
        s->doorbell = 0;
    }
}

static const MemoryRegionOps riscv_rpmi_ops = {
    .read = riscv_rpmi_read,
    .write = riscv_rpmi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};

 static const Property riscv_rpmi_properties[] = {
     DEFINE_PROP_UINT64("harts-mask", RiscvRpmiState, harts_mask, 0),
     DEFINE_PROP_UINT32("flags", RiscvRpmiState, flags, false),
 };

 static void riscv_rpmi_realize(DeviceState *dev, Error **errp)
 {
     RiscvRpmiState *rpmi = RISCV_RISCV_RPMI(dev);

     memory_region_init_io(&rpmi->mmio, OBJECT(dev), &riscv_rpmi_ops, rpmi,
                           TYPE_RISCV_RPMI, RPMI_DBREG_SIZE);
     sysbus_init_mmio(SYS_BUS_DEVICE(dev), &rpmi->mmio);
 }

static void riscv_rpmi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = riscv_rpmi_realize;
    device_class_set_props(dc, riscv_rpmi_properties);
}

static const TypeInfo riscv_rpmi_info = {
    .name          = TYPE_RISCV_RPMI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RiscvRpmiState),
    .class_init    = riscv_rpmi_class_init,
};

static enum rpmi_error shmem_qemu_read(void *priv, rpmi_uint64_t addr,
                                       void *in, rpmi_uint32_t len)
{
    cpu_physical_memory_read(addr, in, len);
    return RPMI_SUCCESS;
}

static enum rpmi_error shmem_qemu_write(void *priv, rpmi_uint64_t addr,
                                        const void *out, rpmi_uint32_t len)
{
    cpu_physical_memory_write(addr, out, len);
    return RPMI_SUCCESS;
}

static enum rpmi_error shmem_qemu_fill(void *priv, rpmi_uint64_t addr,
                                       char ch, rpmi_uint32_t len)
{
    while (len > 0) {
        shmem_qemu_write(priv, addr, &ch, 1);
        len--;
        addr++;
    }

    return RPMI_SUCCESS;
}

struct rpmi_shmem_platform_ops rpmi_shmem_qemu_ops = {
    .read = shmem_qemu_read,
    .write = shmem_qemu_write,
    .fill = shmem_qemu_fill,
};

int init_rpmi_svc_groups(hwaddr shm_addr, int shm_sz,
                         uint32_t a2preq_qsz, uint32_t p2areq_qsz,
                         uint64_t harts_mask, uint32_t soc_xport_type)
{
    char name[32];
    struct rpmi_shmem *rpmi_shmem;
    struct rpmi_transport *rpmi_transport_shmem;
    struct rpmi_context *rctx;

    rpmi_shmem = rpmi_shmem_create("rpmi_shmem", shm_addr, shm_sz,
                                   &rpmi_shmem_qemu_ops, NULL);
    if (!rpmi_shmem) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: rpmi_shmem_qemu_create failed\n ", __func__);
        return -1;
    }

    rpmi_transport_shmem = rpmi_transport_shmem_create("rpmi_transport_shmem",
                                                       RPMI_QUEUE_SLOT_SIZE,
                                                       a2preq_qsz, p2areq_qsz,
                                                       rpmi_shmem);
    if (!rpmi_transport_shmem) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: transport_shmem_create failed, slot_sz: %d, shm: %p\n",
                      __func__, RPMI_SLOT_SIZE_MIN, rpmi_shmem);
        return -1;
    }

    sprintf(name, "rpmi_context_%02d", g_contexts);
    rctx = rpmi_context_create(name,
                               rpmi_transport_shmem,
                               RPMI_SRVGRP_ID_MAX_COUNT,
                               RPMI_PRIVILEGE_M_MODE,
                               PLAT_INFO_LEN,
                               PLAT_INFO);
    if (!rctx) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: rpmi_context_create failed\n ", __func__);
        return -1;
    } else {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: rpmi_context created: %p\n",
                      __func__, rctx);
    }

    /* save the context */
    rpmi_contexts[g_contexts] = rctx;
    g_contexts++;

    return 0;
}

/*
 * Create RPMI devices.
 */
DeviceState *riscv_rpmi_create(hwaddr db_addr, hwaddr shm_addr, int shm_sz,
                               uint32_t a2preq_qsz, uint32_t p2areq_qsz,
                               hwaddr fcm_addr, int fcm_sz,
                               uint64_t harts_mask, uint32_t flags,
                               MachineState *ms)
{
    DeviceState *dev = qdev_new(TYPE_RISCV_RPMI);
    MemoryRegion *address_space_mem = get_system_memory();
    MemoryRegion *shm_mr = g_new0(MemoryRegion, 1);
    MemoryRegion *fcm_mr = g_new0(MemoryRegion, 1);
    char name[32];

    assert(!(db_addr & 0x3));
    assert(!(shm_addr & 0x3));
    assert(!(fcm_addr & 0x3));

    qdev_prop_set_uint64(dev, "harts-mask", harts_mask);
    qdev_prop_set_uint32(dev, "flags", flags ? true : false);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, db_addr);

    sprintf(name, "shm@%lx", shm_addr);
    memory_region_init_ram(shm_mr, OBJECT(dev),
                           name, shm_sz, &error_fatal);
    memory_region_add_subregion(address_space_mem,
                                shm_addr, shm_mr);

    if (fcm_sz) {
        sprintf(name, "fcm@%lx", fcm_addr);
        memory_region_init_ram(fcm_mr, OBJECT(dev), name,
                               fcm_sz, &error_fatal);
        memory_region_add_subregion(address_space_mem,
                                    fcm_addr, fcm_mr);
    }

    if (!init_rpmi_svc_groups(shm_addr, shm_sz,
                              a2preq_qsz, p2areq_qsz,
                              harts_mask, flags)) {
        return NULL;
    }

    return dev;
}

static void riscv_rpmi_register_types(void)
{
    type_register_static(&riscv_rpmi_info);
}

type_init(riscv_rpmi_register_types)
