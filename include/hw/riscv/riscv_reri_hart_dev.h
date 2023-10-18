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

#define TYPE_RISCV_RERI_HART_DEV "riscv_reri"

DeviceState *riscv_create_harts_reri_dev(hwaddr addr);

int riscv_reri_inject(void *opaque, int record, hwaddr addr, uint64_t info);

#endif
