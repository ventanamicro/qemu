 /*
  * riscv_rpmi.h
  * RPMI message IO handling header file
  *
  * Copyright (c) 2024 Ventana Micro Systems Inc.
  *
  * Authors:
  * Subrahmanya Lingappa <slingappa@ventanamicro.com>
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

#ifndef HW_riscv_rpmi_H
#define HW_riscv_rpmi_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_RISCV_RPMI "riscv.riscv.rpmi"

#define RISCV_RISCV_RPMI(obj) \
    OBJECT_CHECK(RiscvRpmiState, (obj), TYPE_RISCV_RPMI)
typedef struct RiscvRpmiState RiscvRpmiState;
DECLARE_INSTANCE_CHECKER(RiscvRpmiState, RISCV_RPMI,
                         TYPE_RISCV_RPMI)
#define __UNUSED__     __attribute__ ((unused))

#define MAX_HARTS 64
#define MAX_XPORTS2 16
#define RPMI_ALL_NUM_QUEUES (4)
#define RPMI_A2P_NUM_QUEUES (2)

#define RPMI_QUEUE_SLOT_SIZE 64
#define RPMI_DBREG_SIZE (0x1000)
#define RPMI_ALL_NUM_REGS (RPMI_ALL_NUM_QUEUES + 1)
#define RPMI_A2P_NUM_REGS (RPMI_A2P_NUM_QUEUES + 1)

struct RiscvRpmiState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    QEMUTimer *fcm_poll_timer;
    MemoryRegion mmio;
    uint32_t doorbell;

    uint64_t harts_mask;
    uint32_t flags;
};

 enum {
     RISCV_RPMI_MAX_HARTS             = 4095,
 };

DeviceState *riscv_rpmi_create(hwaddr db_addr, hwaddr shm_addr, int shm_sz,
                               uint32_t a2preq_qsz, uint32_t p2areq_qsz,
                               hwaddr fcm_addr, int fcm_sz,
                               uint64_t harts_mask, uint32_t flags,
                               MachineState *ms);

void handle_rpmi_event(void);

#endif
