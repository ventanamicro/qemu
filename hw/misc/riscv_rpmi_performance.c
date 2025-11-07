#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "librpmi.h"
#include "exec/hwaddr.h"
#include "hw/misc/riscv_rpmi.h"
#include "hw/qdev-properties.h"

struct platform_perfs_context {
    uint32_t current_level;
    uint32_t current_min_limit;
    uint32_t current_max_limit;
    uint32_t min_level;
    uint32_t max_level;
};

struct rpmi_perf_shmem {
    uint32_t get_level;
    uint32_t set_level;
    uint32_t get_limit_max;
    uint32_t get_limit_min;
    uint32_t set_limit_max;
    uint32_t set_limit_min;
    uint32_t padding[2];
};

struct rpmi_perf_db {
    uint32_t db_level;
    uint32_t db_limit;
};

int add_performance_group(struct rpmi_context *pctx);

#define RPMI_PERF_DOMAIN_COUNT        6

/*******************************************************
 * Operating performance points (opp) attributes:
 *     index - performance level index
 *     clock_freq - clock frequency (kHZ)
 *     power_cost - power cost (uW)
 *     transition_latency - transition latency (us)
 *******************************************************/

static struct rpmi_perf_level opp_levels_0[] = { /* index      KHz      uW     uS  */
                                                  {    0,    700000, 1075000, 1400 },
                                                  {    1,    800000,  925000,  800 },
                                                  {    2,    900000,  950000,  900 },
                                                  {    3,   1000000,  975000, 1000 },
                                                  {    4,   1100000, 1025000, 1100 },
                                                  {    5,   1200000, 1025000, 1200 },
                                                  {    6,   1300000, 1050000, 1300 },
                                                  {    7,   1500000, 1100000, 1500 },
                                                };

static struct rpmi_perf_fc_attrs fc_attrs_0[] = {
    [RPMI_PERF_FC_GET_LEVEL] = {   /* RPMI_PERF_FC_GET_LEVEL */
        .flags = RPMI_PERF_FST_CHN_DB_REG_32_BITS,
        .offset_phys_addr_low = 0x0,
        .offset_phys_addr_high = 0x0,
        .size = 0x4,
    },
    [RPMI_PERF_FC_SET_LEVEL] = {   /* RPMI_PERF_FC_SET_LEVEL */
        .flags = RPMI_PERF_FST_CHN_DB_REG_32_BITS | RPMI_PERF_FST_CHN_DB_SUPP,
        .offset_phys_addr_low = 0x4,
        .offset_phys_addr_high = 0x0,
        .size = 0x4,
        .db_addr_low = 0x10310000,
        .db_addr_high = 0x0,
        .db_id = 0x1,
    },
    [RPMI_PERF_FC_GET_LIMIT] = {   /* RPMI_PERF_FC_GET_LIMIT */
        .flags = RPMI_PERF_FST_CHN_DB_REG_32_BITS,
        .offset_phys_addr_low = 0x8,
        .offset_phys_addr_high = 0x0,
        .size = 0x8,
    },
    [RPMI_PERF_FC_SET_LIMIT] = {   /* RPMI_PERF_FC_SET_LIMIT */
        .flags = RPMI_PERF_FST_CHN_DB_REG_32_BITS | RPMI_PERF_FST_CHN_DB_SUPP,
        .offset_phys_addr_low = 0x10,
        .offset_phys_addr_high = 0x0,
        .size = 0x8,
        .db_addr_low = 0x10310004,
        .db_addr_high = 0x0,
        .db_id = 0x1,
    }
};

static struct rpmi_perf_level opp_levels_1[] = { /* index      KHz      uW     uS  */
                                                  {    0,    800000,  920000,  800 },
                                                  {    1,    900000,  955000,  900 },
                                                  {    2,   1000000,  970000, 1000 },
                                               };

static struct rpmi_perf_level opp_levels_2[] = { /* index      KHz      uW     uS  */
                                                  {    0,    800000,  945000,  800 },
                                                  {    1,    900000,  960000,  900 },
                                                  {    2,   1000000,  985000, 1000 },
                                                  {    3,   1400000, 1035000, 1100 },
                                                  {    4,   1500000, 1055000, 1200 },
                                                  {    5,   1600000, 1060000, 1300 },
                                               };

static struct rpmi_perf_level opp_levels_3[] = { /* index      KHz      uW     uS  */
                                                  {    0,    600000,  970000,  800 },
                                                  {    1,    800000,  980000,  900 },
                                                  {    2,   1000000,  990000, 1000 },
                                                  {    3,   1100000, 1100000, 1100 },
                                                  {    4,   1200000, 1200000, 1200 },
                                               };

static struct rpmi_perf_level opp_levels_4[] = { /* index      KHz      uW     uS  */
                                                  {    0,    400000,  920000,  800 },
                                                  {    1,   1000000,  955000,  900 },
                                                  {    2,   1050000,  970000, 1000 },
                                               };

static struct rpmi_perf_level opp_levels_5[] = { /* index      KHz      uW     uS  */
                                                  {    0,    300000,   45000,  800 },
                                                  {    1,    400000,   45000,  800 },
                                                  {    2,    450000,   60000,  900 },
                                                  {    3,    500000,   45000,  800 },
                                                  {    4,    700000,   45000,  800 },
                                                  {    5,    800000,   45000,  800 },
                                               };

static struct rpmi_perf_data perf_data[] = {
    [0] = {
         .name = "perf0",
        .trans_latency = 100,
        .perf_capabilities = RPMI_PERF_CAPABILITY_SET_LIMIT |
                             RPMI_PERF_CAPABILITY_SET_LEVEL |
                             RPMI_PERF_CAPABILITY_FAST_CHANNEL_SUPPORT,
        .perf_level_count = ARRAY_SIZE(opp_levels_0),
        .perf_level_array = opp_levels_0,
        .fc_attrs_array = fc_attrs_0,
    },

    [1] = {
        .name = "perf1",
        .trans_latency = 100,
        .perf_capabilities = RPMI_PERF_CAPABILITY_SET_LIMIT |
                             RPMI_PERF_CAPABILITY_SET_LEVEL,
        .perf_level_count = ARRAY_SIZE(opp_levels_1),
        .perf_level_array = opp_levels_1,
    },

    [2] = {
        .name = "perf2",
        .trans_latency = 50,
        .perf_capabilities = RPMI_PERF_CAPABILITY_SET_LIMIT |
                             RPMI_PERF_CAPABILITY_SET_LEVEL,
        .perf_level_count = ARRAY_SIZE(opp_levels_2),
        .perf_level_array = opp_levels_2,
    },

    [3] = {
        .name = "perf3",
        .trans_latency = 50,
        .perf_capabilities = RPMI_PERF_CAPABILITY_SET_LIMIT |
                             RPMI_PERF_CAPABILITY_SET_LEVEL,
        .perf_level_count = ARRAY_SIZE(opp_levels_3),
        .perf_level_array = opp_levels_3,
    },

    [4] = {
        .name = "perf4",
        .trans_latency = 100,
        .perf_capabilities = RPMI_PERF_CAPABILITY_SET_LIMIT |
                             RPMI_PERF_CAPABILITY_SET_LEVEL,
        .perf_level_count = ARRAY_SIZE(opp_levels_4),
        .perf_level_array = opp_levels_4,
    },

    [5] = {
        .name = "perf5",
        .trans_latency = 50,
        .perf_capabilities = RPMI_PERF_CAPABILITY_SET_LIMIT |
                             RPMI_PERF_CAPABILITY_SET_LEVEL,
        .perf_level_count = ARRAY_SIZE(opp_levels_5),
        .perf_level_array = opp_levels_5,
    },
};

static struct rpmi_perf_fc_memory_region perf_memory_region = {
        .addr_low  = 0x10300000,
        .addr_high = 0x0,
        .size_low  = 0x10000,
        .size_high = 0x0,
};

static struct platform_perfs_context platperfs_ctx[RPMI_PERF_DOMAIN_COUNT] = {
    [0] = {.current_level = 2, .current_min_limit = 0, .current_max_limit = 7, .min_level = 0, .max_level = 7},
    [1] = {.current_level = 1, .current_min_limit = 0, .current_max_limit = 2, .min_level = 0, .max_level = 2},
    [2] = {.current_level = 3, .current_min_limit = 0, .current_max_limit = 5, .min_level = 0, .max_level = 5},
    [3] = {.current_level = 2, .current_min_limit = 0, .current_max_limit = 4, .min_level = 0, .max_level = 4},
    [4] = {.current_level = 1, .current_min_limit = 0, .current_max_limit = 2, .min_level = 0, .max_level = 2},
    [5] = {.current_level = 1, .current_min_limit = 0, .current_max_limit = 5, .min_level = 0, .max_level = 5},
};

enum rpmi_error performance_get_level(void *priv, uint32_t perf_id,
                                      uint32_t *perf_level);

enum rpmi_error performance_set_level(void *priv, uint32_t perf_id,
                                      uint32_t perf_level);

enum rpmi_error performance_get_limit(void *priv,
                                      uint32_t perf_id,
                                      uint32_t *max_perf_limit,
                                      uint32_t *min_perf_limit);

enum rpmi_error performance_set_limit(void *priv,
                                      uint32_t perf_id,
                                      uint32_t max_perf_limit,
                                      uint32_t min_perf_limit);

enum rpmi_error performance_get_supp_levels(void *priv,
                                            uint32_t perf_id,
                                            uint32_t max,
                                            uint32_t perf_index,
                                            uint32_t *returned_levels,
                                            struct rpmi_perf_level *level_array);

enum rpmi_error performance_get_level(void *priv,
                                      uint32_t perf_id,
                                      uint32_t *perf_level)
{
    if (perf_id >= RPMI_PERF_DOMAIN_COUNT)
        return RPMI_ERR_INVALID_PARAM;

    *perf_level = platperfs_ctx[perf_id].current_level;

    return RPMI_SUCCESS;
}

enum rpmi_error performance_set_level(void *priv,
                                      uint32_t perf_id,
                                      uint32_t perf_level)
{
    uint32_t max_limit;
    uint32_t min_limit;

    if (perf_id > RPMI_PERF_DOMAIN_COUNT)
        return RPMI_ERR_INVALID_PARAM;

    if (!(perf_data[perf_id].perf_capabilities & RPMI_PERF_CAPABILITY_SET_LEVEL))
        return RPMI_ERR_DENIED;

    max_limit = platperfs_ctx[perf_id].current_max_limit;
    min_limit = platperfs_ctx[perf_id].current_min_limit;

    if (perf_level <= min_limit)
        perf_level = min_limit;
    else if (perf_level >= max_limit)
        perf_level = max_limit;

    platperfs_ctx[perf_id].current_level = perf_level;

    return RPMI_SUCCESS;
}

enum rpmi_error performance_get_limit(void *priv,
                                      uint32_t perf_id,
                                      uint32_t *max_perf_limit,
                                      uint32_t *min_perf_limit)
{
    if (perf_id > RPMI_PERF_DOMAIN_COUNT)
        return RPMI_ERR_INVALID_PARAM;

    *max_perf_limit = platperfs_ctx[perf_id].current_max_limit;
    *min_perf_limit = platperfs_ctx[perf_id].current_min_limit;

    return RPMI_SUCCESS;
}

enum rpmi_error performance_set_limit(void *priv,
                                      uint32_t perf_id,
                                      uint32_t max_perf_limit,
                                      uint32_t min_perf_limit)
{
    uint32_t max_level;
    uint32_t min_level;

    if (perf_id > RPMI_PERF_DOMAIN_COUNT)
        return RPMI_ERR_INVALID_PARAM;

    if (!(perf_data[perf_id].perf_capabilities & RPMI_PERF_CAPABILITY_SET_LIMIT))
        return RPMI_ERR_DENIED;

    max_level = platperfs_ctx[perf_id].max_level;
    min_level = platperfs_ctx[perf_id].min_level;

    if (min_perf_limit < min_level)
        min_perf_limit = min_level;
    else if(max_perf_limit > max_level)
        max_perf_limit = max_level;

    platperfs_ctx[perf_id].current_max_limit = max_perf_limit;
    platperfs_ctx[perf_id].current_min_limit = min_perf_limit;

    return RPMI_SUCCESS;
}

enum rpmi_error performance_get_supp_levels(void *priv,
                                            uint32_t perf_id,
                                            uint32_t max_levels,
                                            uint32_t perf_index,
                                            uint32_t *returned_levels,
                                            struct rpmi_perf_level *level_array)
{
    struct rpmi_perf_level *opp_levels;
    uint32_t i;
    uint32_t remaining, returned;
    uint32_t num;

    if (perf_id >= RPMI_PERF_DOMAIN_COUNT)
        return RPMI_ERR_INVALID_PARAM;

    if (perf_index == perf_data[perf_id].perf_level_count) {
        *returned_levels = 0;
        return RPMI_SUCCESS;
    } else if (perf_index > perf_data[perf_id].perf_level_count) {
        return RPMI_ERR_INVALID_PARAM;
    }

    opp_levels = perf_data[perf_id].perf_level_array;
    remaining = perf_data[perf_id].perf_level_count - perf_index;

    if (remaining > max_levels)
        num = max_levels;
    else
        num = remaining;

    returned = 0;
    for (i = 0; i < num; i++) {
        level_array[i].level_index = opp_levels[i + perf_index].level_index;
        level_array[i].clock_freq = opp_levels[i + perf_index].clock_freq;
        level_array[i].power_cost = opp_levels[i + perf_index].power_cost;
        level_array[i].transition_latency = opp_levels[i + perf_index].transition_latency;
        returned++;
    }

    *returned_levels = returned;

    return RPMI_SUCCESS;
}

const struct rpmi_perf_platform_ops perf_ops = {
    .get_level = performance_get_level,
    .set_level = performance_set_level,
    .set_limit = performance_set_limit,
};

int add_performance_group(struct rpmi_context *pctx)
{
    struct rpmi_service_group *perfgrp;
    uint32_t perf_count = sizeof(perf_data)/sizeof(struct rpmi_perf_data);

    for (int i = 0; i < perf_count; i++) {
        struct rpmi_perf_data data = perf_data[i];

        if (data.perf_capabilities & RPMI_PERF_CAPABILITY_FAST_CHANNEL_SUPPORT) {
            uint64_t address;
            uint32_t perf_level;
            uint32_t perf_max, perf_min;

            struct rpmi_perf_fc_attrs attr;

            attr = data.fc_attrs_array[RPMI_PERF_FC_GET_LEVEL];
            address = attr.offset_phys_addr_low;
            address += ((uint64_t) attr.offset_phys_addr_high) << 32;
            address += perf_memory_region.addr_low;
            address += ((uint64_t) perf_memory_region.addr_high) << 32;
            perf_level = platperfs_ctx[i].current_level;

            if (attr.size == 4)
                cpu_physical_memory_write(address, &perf_level, sizeof(perf_level));

            attr = data.fc_attrs_array[RPMI_PERF_FC_GET_LIMIT];
            address = attr.offset_phys_addr_low;
            address += (((uint64_t) attr.offset_phys_addr_high) << 32);
            address += perf_memory_region.addr_low;
            address += (((uint64_t) perf_memory_region.addr_high) << 32);
            perf_max = platperfs_ctx[i].current_max_limit;
            perf_min = platperfs_ctx[i].current_min_limit;

            if (attr.size == 8) {
                cpu_physical_memory_write(address, &perf_max, sizeof(perf_max));
                cpu_physical_memory_write(address + 4, &perf_min, sizeof(perf_min));
            }
        }
    }

    perfgrp = rpmi_service_group_perf_create(perf_count,
                                             perf_data,
                                             &perf_ops,
                                             &perf_memory_region,
                                             NULL);
    if (!perfgrp) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: perf service group create failed\n",
                      __func__);
        return -1;
    }

    rpmi_context_add_group(pctx, perfgrp);

    return 0;
}

DeviceState *rpmi_perf_init(hwaddr base_shmem, hwaddr base_db, void *rpmi_perf_data)
{
    DeviceState *dev = qdev_new(TYPE_RISCV_RPMI_PERF);

    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);

    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base_shmem);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 1, base_db);

    return dev;
}

static uint64_t riscv_rpmi_perf_read(void *opaque, hwaddr offset, unsigned int size)
{
    return 0;
}

static void riscv_rpmi_perf_write(void *opaque, hwaddr offset,
                uint64_t val64, unsigned int size)
{
    uint32_t perf_domain;
    uint32_t perf_operation; // 0 = level, 1 = limit

    perf_domain = offset / sizeof(struct rpmi_perf_db);
    perf_operation = offset % sizeof(struct rpmi_perf_db);

    if (perf_domain > RPMI_PERF_DOMAIN_COUNT)
        return;

    if ((size != 4) || (val64 != 0x1))
        return;

    if (perf_operation == 0) {
        struct rpmi_perf_fc_attrs attr;
        uint64_t address;
        uint32_t perf_level;
        struct rpmi_perf_data data = perf_data[perf_domain];

        /* Obtain Set Level Information */
        attr = data.fc_attrs_array[RPMI_PERF_FC_SET_LEVEL];
        address = attr.offset_phys_addr_low;
        address += ((uint64_t) attr.offset_phys_addr_high) << 32;
        address += perf_memory_region.addr_low;
        address += ((uint64_t) perf_memory_region.addr_high) << 32;

        /* Write to register */
        cpu_physical_memory_read(address, &perf_level, sizeof(perf_level));
        if (performance_set_level(NULL, perf_domain, perf_level) == RPMI_SUCCESS) {
            /* Copy data to read shared memory */
            attr = data.fc_attrs_array[RPMI_PERF_FC_GET_LEVEL];
            address = attr.offset_phys_addr_low;
            address += ((uint64_t) attr.offset_phys_addr_high) << 32;
            address += perf_memory_region.addr_low;
            address += ((uint64_t) perf_memory_region.addr_high) << 32;
            cpu_physical_memory_write(address, &perf_level, sizeof(perf_level));
        }
    } else if (perf_operation == 1) {
        struct rpmi_perf_fc_attrs attr;
        uint64_t address;
        uint32_t perf_limit_max, perf_limit_min;
        struct rpmi_perf_data data = perf_data[perf_domain];

        /* Obtain Set Level Information */
        attr = data.fc_attrs_array[RPMI_PERF_FC_SET_LIMIT];
        address = attr.offset_phys_addr_low;
        address += ((uint64_t) attr.offset_phys_addr_high) << 32;
        address += perf_memory_region.addr_low;
        address += ((uint64_t) perf_memory_region.addr_high) << 32;

        /* Write to register */
        cpu_physical_memory_read(address, &perf_limit_max, sizeof(perf_limit_max));
        cpu_physical_memory_read(address + 4, &perf_limit_min, sizeof(perf_limit_min));
        if (performance_set_limit(NULL, perf_domain, perf_limit_max, perf_limit_min) == RPMI_SUCCESS) {
            /* Copy data to read shared memory */
            attr = data.fc_attrs_array[RPMI_PERF_FC_GET_LIMIT];
            address = attr.offset_phys_addr_low;
            address += ((uint64_t) attr.offset_phys_addr_high) << 32;
            address += perf_memory_region.addr_low;
            address += ((uint64_t) perf_memory_region.addr_high) << 32;

            cpu_physical_memory_write(address, &perf_limit_max, sizeof(perf_limit_max));
            cpu_physical_memory_write(address + 4, &perf_limit_min, sizeof(perf_limit_min));
        }
    }
}

static const MemoryRegionOps riscv_rpmi_perf_ops = {
    .read = riscv_rpmi_perf_read,
    .write = riscv_rpmi_perf_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};

static void riscv_rpmi_perf_realize(DeviceState *dev, Error **errp)
{
    RiscvRpmiPerfState *rpmi = RISCV_RISCV_RPMI_PERF(dev);

    memory_region_init_ram(&rpmi->ram, OBJECT(dev), TYPE_RISCV_RPMI_PERF_SHMEM, 0x10000, &error_fatal);
    rpmi->ram_ptr = memory_region_get_ram_ptr(&rpmi->ram);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &rpmi->ram);

    memory_region_init_io(&rpmi->mmio, OBJECT(dev), &riscv_rpmi_perf_ops, rpmi,
                        TYPE_RISCV_RPMI_PERF, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &rpmi->mmio);
}

static void riscv_rpmi_perf_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = riscv_rpmi_perf_realize;
}

static const TypeInfo riscv_rpmi_perf_info = {
    .name          = TYPE_RISCV_RPMI_PERF,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RiscvRpmiPerfState),
    .class_init    = riscv_rpmi_perf_class_init,
};

static void riscv_rpmi_perf_register_types(void)
{
    type_register_static(&riscv_rpmi_perf_info);
}

type_init(riscv_rpmi_perf_register_types)
