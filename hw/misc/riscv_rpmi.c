 /*
  * riscv_rpmi.c
  * RPMI transport IO handling routines.
  *
  * Copyright (c) 2023 Subrahmanya Lingapa <slingappa@ventanamicro.com>
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
#include "qemu/timer.h"
#include "hw/misc/riscv_rpmi.h"
#include "exec/address-spaces.h"
#include "hw/qdev-properties.h"
#include "hw/misc/riscv_rpmi.h"
#include "hw/misc/riscv_rpmi_transport.h"

static int num_xports;

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
        handle_rpmi_shm(s->id);

        /* clear the doorbell register */
        s->doorbell = 0;
    }
}

void fcm_checkpoint_notify(void *opaque)
{
    struct RiscvRpmiState *s = opaque;
    uint32_t xport;

    for (xport = 0; xport < num_xports; xport++) {
        handle_rpmi_fcm(xport);
    }

    timer_mod(s->fcm_poll_timer,
              qemu_clock_get_us(QEMU_CLOCK_HOST) + FCM_CHECK_TIME);
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

 static Property riscv_rpmi_properties[] = {
     DEFINE_PROP_UINT64("harts-mask", RiscvRpmiState, harts_mask, 0),
     DEFINE_PROP_UINT32("flags", RiscvRpmiState, flags, false),
     DEFINE_PROP_END_OF_LIST(),
 };

 static void riscv_rpmi_realize(DeviceState *dev, Error **errp)
 {
     RiscvRpmiState *rpmi = RISCV_RISCV_RPMI(dev);

     rpmi->id = num_xports;

     memory_region_init_io(&rpmi->mmio, OBJECT(dev), &riscv_rpmi_ops, rpmi,
                           TYPE_RISCV_RPMI, RPMI_DBREG_SIZE);
     sysbus_init_mmio(SYS_BUS_DEVICE(dev), &rpmi->mmio);
     rpmi->fcm_poll_timer =  timer_new_us(QEMU_CLOCK_HOST,
                                       fcm_checkpoint_notify, rpmi);
     timer_mod(rpmi->fcm_poll_timer,
               qemu_clock_get_us(QEMU_CLOCK_HOST) + FCM_CHECK_TIME);
 }

static void riscv_rpmi_class_init(ObjectClass *klass, void *data)
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

 /*
  * Create RPMI devices.
  */
 DeviceState *riscv_rpmi_create(hwaddr db_addr, hwaddr shm_addr, int shm_sz,
                                hwaddr fcm_addr, int fcm_sz,
                                uint64_t harts_mask, uint32_t flags)
 {
     DeviceState *dev = qdev_new(TYPE_RISCV_RPMI);
     MemoryRegion *address_space_mem = get_system_memory();
     MemoryRegion *shm_mr = g_new0(MemoryRegion, 1);
     MemoryRegion *fcm_mr = g_new0(MemoryRegion, 1);
     uint32_t socket_num;
     char name[32];

     assert(!(db_addr & 0x3));
     assert(!(shm_addr & 0x3));
     assert(!(fcm_addr & 0x3));

     qdev_prop_set_uint64(dev, "harts-mask", harts_mask);
     qdev_prop_set_uint32(dev, "flags", flags ? true : false);
     sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
     sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, db_addr);

     sprintf(name, "shm@%lx", shm_addr);
     memory_region_init_ram(shm_mr, OBJECT(dev), name,
                            shm_sz, &error_fatal);
     memory_region_add_subregion(address_space_mem,
                                 shm_addr, shm_mr);

     sprintf(name, "fcm@%lx", fcm_addr);
     memory_region_init_ram(fcm_mr, OBJECT(dev), name,
                            fcm_sz, &error_fatal);
     memory_region_add_subregion(address_space_mem,
                                 fcm_addr, fcm_mr);

     if (flags & (1 << RPMI_XPORT_TYPE_SOC)) {
         /* first transport is for SOC and doesnt have any harts */
         socket_num = -1;
     } else {
         socket_num = num_xports - 1;
     }

     rpmi_init_transport(num_xports, shm_addr, db_addr, fcm_addr, socket_num,
                         harts_mask);

     num_xports++;

     return dev;
 }

static void riscv_rpmi_register_types(void)
{
    type_register_static(&riscv_rpmi_info);
}

type_init(riscv_rpmi_register_types)
