/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2024 Ventana Micro Systems Inc.
 */

#include "qemu/osdep.h"
#include "exec/cpu-common.h"
#include "qemu/log.h"
#include "librpmi_env.h"

void *rpmi_env_zalloc(rpmi_size_t size)
{
    return calloc(size, 1);
}

void rpmi_env_free(void *ptr)
{
    free(ptr);
}

void rpmi_env_writel(rpmi_uint64_t addr, rpmi_uint32_t val)
{
    cpu_physical_memory_write(addr, &val, 4);
}
