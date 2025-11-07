#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "librpmi.h"

#define NUM_LEVELS_DISCRETE(a, d)       (sizeof(a) / sizeof(d))

#define NUM_LEVELS_LINEAR(a) \
    ((a)->uvolt_max - (a)->uvolt_min) / ((a)->uvolt_step) + 1;

#define MIN_LEVEL_LINEAR(a)     ((a)->uvolt_min)
#define MAX_LEVEL_LINEAR(a)     ((a)->uvolt_max)
#define LEVEL_STEP_LINEAR(a)    ((a)->uvolt_step)

struct platform_voltage_context {
    uint32_t level;
    uint32_t min_level;
    uint32_t max_level;
    uint32_t config;
};

int add_voltage_group(struct rpmi_context *dctx);

#define RPMI_VOLTAGE_DOMAIN_COUNT               6

int32_t volt_discrete0[3] = { 1200000, 3000000, 5000000 };
int32_t volt_discrete1[3] = { 1100000, 1200000, 1500000 };
int32_t volt_discrete2[3] = { 1300000, 1800000, 3000000 };
int32_t volt_discrete3[3] = { 1200000, 1500000, 3300000 };
int32_t volt_discrete4[3] = { 1200000, 1800000, 3100000 };
int32_t volt_discrete5[3] = { 1000000, 3300000, 5000000 };

int32_t volt_linear[6][3] = {
    { 1000000, 1300000, 100000 },
    { 1500000, 1800000, 100000 },
    { 3000000, 3300000, 100000 },
    { 1500000, 1800000, 100000 },
    { 3000000, 3300000, 100000 },
    { 1500000, 1800000, 100000 },
};

static struct rpmi_voltage_discrete_range volt_discrete_range[6] = {
    [0] = {
        .uvolt = (uint32_t *)&volt_discrete0[0],
    },
    [1] = {
        .uvolt = (uint32_t *)&volt_discrete1[0],
    },
    [2] = {
        .uvolt = (uint32_t *)&volt_discrete2[0],
    },
    [3] = {
        .uvolt = (uint32_t *)&volt_discrete3[0],
    },
    [4] = {
        .uvolt = (uint32_t *)&volt_discrete4[0],
    },
    [5] = {
        .uvolt = (uint32_t *)&volt_discrete5[0],
    },
};

static struct rpmi_voltage_linear_range volt_linear_range[6] = {
    { 1000000, 1300000, 100000 },
    { 1500000, 1800000, 100000 },
    { 3000000, 3300000, 100000 },
    { 1500000, 1800000, 100000 },
    { 3000000, 3300000, 100000 },
    { 1500000, 1800000, 100000 },
};

static struct rpmi_voltage_data voltage_data[] = {
    [0] = {
        .name = "volt0",
        .voltage_type = RPMI_VOLT_TYPE_DISCRETE,
        .control = RPMI_VOLT_CAPABILITY_ALWAYS_ON,
        .config = RPMI_VOLT_CONFIG_NOT_SUPPORTED,
        .num_levels = 3,
        .trans_latency = 90,
        .discrete_range = &volt_discrete_range[0],
        .discrete_levels = &volt_discrete0[0],
        .level_uv = 1100000,
    },

    [1] = {
        .name = "volt1",
        .voltage_type = RPMI_VOLT_TYPE_DISCRETE,
        .control = RPMI_VOLT_CAPABILITY_ALWAYS_ON,
        .config = RPMI_VOLT_CONFIG_NOT_SUPPORTED,
        .num_levels = 3,
        .trans_latency = 120,
        .discrete_range = &volt_discrete_range[1],
        .discrete_levels = &volt_discrete1[0],
        .level_uv = 1600000,
    },

    [2] = {
        .name = "volt2",
        .voltage_type = RPMI_VOLT_TYPE_DISCRETE,
        .control = RPMI_VOLT_CAPABILITY_ENABLED_DISABLED,
        .config = RPMI_VOLT_CONFIG_ENABLED,
        .num_levels = 3,
        .trans_latency = 100,
        .discrete_range = &volt_discrete_range[2],
        .discrete_levels = &volt_discrete2[0],
        .level_uv = 3200000,
    },

    [3] = {
        .name = "volt3",
        .voltage_type = RPMI_VOLT_TYPE_LINEAR,
        .control = RPMI_VOLT_CAPABILITY_ALWAYS_ON,
        .config = RPMI_VOLT_CONFIG_NOT_SUPPORTED,
        .num_levels = 1,
        .trans_latency = 110,
        .linear_range = &volt_linear_range[3],
        .linear_levels = &volt_linear[3][0],
    .level_uv = 1600000,
    },

    [4] = {
        .name = "volt4",
        .voltage_type = RPMI_VOLT_TYPE_DISCRETE,
        .control = RPMI_VOLT_CAPABILITY_ENABLED_DISABLED,
        .config = RPMI_VOLT_CONFIG_ENABLED,
        .num_levels = 3,
        .trans_latency = 130,
        .discrete_range = &volt_discrete_range[4],
        .discrete_levels = &volt_discrete4[0],
    .level_uv = 3200000,
    },

    [5] = {
        .name = "volt5",
        .voltage_type = RPMI_VOLT_TYPE_LINEAR,
        .control = RPMI_VOLT_CAPABILITY_ENABLED_DISABLED,
        .config = RPMI_VOLT_CONFIG_ENABLED,
        .num_levels = 1,
        .trans_latency = 140,
        .linear_range = &volt_linear_range[5],
        .linear_levels = &volt_linear[5][0],
    .level_uv = 1800000,
    },
};

static struct platform_voltage_context platvolts_ctx[RPMI_VOLTAGE_DOMAIN_COUNT] = {
    [0] = {.level = 1200000, .min_level = 0, .max_level = 0, .config = RPMI_VOLT_CONFIG_NOT_SUPPORTED },
    [1] = {.level = 3000000, .min_level = 0, .max_level = 0, .config = RPMI_VOLT_CONFIG_NOT_SUPPORTED },
    [2] = {.level = 5000000, .min_level = 0, .max_level = 0, .config = RPMI_VOLT_CONFIG_ENABLED },
    [3] = {.level = 1100000, .min_level = 1000000, .max_level = 1300000, .config = RPMI_VOLT_CONFIG_NOT_SUPPORTED },
    [4] = {.level = 5000000, .min_level = 0, .max_level = 0, .config = RPMI_VOLT_CONFIG_ENABLED },
    [5] = {.level = 3200000, .min_level = 3000000, .max_level = 3300000, .config = RPMI_VOLT_CONFIG_ENABLED },
};

enum rpmi_error volt_get_config(void *priv, uint32_t volt_id,
                                uint32_t *config);
enum rpmi_error volt_set_config(void *priv, uint32_t volt_id,
                                uint32_t config);
enum rpmi_error volt_get_supp_levels(void *priv, uint32_t volt_id,
                                     uint32_t max, uint32_t volt_index,
                                     uint32_t *returned_levels,
                                     int32_t *level_array);
enum rpmi_error volt_linear_level_change_match(void *priv, uint32_t volt_id,
                                               int32_t level);
enum rpmi_error volt_get_level(void *priv, uint32_t volt_id,
                               int32_t *volt_level);
enum rpmi_error volt_set_level(void *priv, uint32_t volt_id,
                               int32_t *volt_level);

enum rpmi_error volt_get_config(void *priv,
                                uint32_t volt_id,
                                uint32_t *config)
{
    if (volt_id > RPMI_VOLTAGE_DOMAIN_COUNT)
        return RPMI_ERR_INVALID_PARAM;

    if (!config)
        return RPMI_ERR_INVALID_PARAM;

    *config = platvolts_ctx[volt_id].config;

    return RPMI_SUCCESS;
}

enum rpmi_error volt_set_config(void *priv,
                                uint32_t volt_id,
                                uint32_t config)
{
    if (volt_id > RPMI_VOLTAGE_DOMAIN_COUNT)
        return RPMI_ERR_INVALID_PARAM;

    if (platvolts_ctx[volt_id].config == RPMI_VOLT_CONFIG_NOT_SUPPORTED)
        return RPMI_ERR_NOTSUPP;

    if (config != RPMI_VOLT_CONFIG_ENABLED && config != RPMI_VOLT_CONFIG_DISABLED)
        return RPMI_ERR_INVALID_PARAM;

    platvolts_ctx[volt_id].config = config;

    return RPMI_SUCCESS;
}

enum rpmi_error volt_get_supp_levels(void *priv,
                                     uint32_t volt_id,
                                     uint32_t max_levels,
                                     uint32_t volt_index,
                                     uint32_t *returned_levels,
                                     int32_t *level_array)
{
    int32_t *levels;
    uint32_t voltage_type;
    uint32_t i, j;
    uint32_t remaining, returned;
    uint32_t num;

    if (volt_id >= RPMI_VOLTAGE_DOMAIN_COUNT)
        return RPMI_ERR_INVALID_PARAM;

    if (volt_index == voltage_data[volt_id].num_levels) {
        *returned_levels = 0;
        return RPMI_SUCCESS;
    } else if (volt_index > voltage_data[volt_id].num_levels) {
        return RPMI_ERR_INVALID_PARAM;
    }

    voltage_type = voltage_data[volt_id].voltage_type;

    if (voltage_type == RPMI_VOLT_TYPE_DISCRETE)
        levels = voltage_data[volt_id].discrete_levels;
    else if (voltage_type == RPMI_VOLT_TYPE_LINEAR)
        levels = voltage_data[volt_id].linear_levels;

    remaining = voltage_data[volt_id].num_levels - volt_index;

    if (remaining > max_levels)
        num = max_levels;
    else
        num = remaining;

    returned = 0;
    j = 0;
    for (i = 0; i < num; i++) {
        if (voltage_type == RPMI_VOLT_TYPE_DISCRETE) {
            level_array[i] = levels[i + volt_index];
            returned++;
        } else if (voltage_type == RPMI_VOLT_TYPE_LINEAR) {
            for (j = 0; j < 3; j++)
                level_array[j] = levels[j + volt_index];

            returned++;
        }
    }

    *returned_levels = returned;

    return RPMI_SUCCESS;
}

enum rpmi_error volt_linear_level_change_match(void *priv,
                                               uint32_t volt_id,
                                               int32_t level)
{
    uint32_t min_level = MIN_LEVEL_LINEAR(voltage_data[volt_id].linear_range);
    uint32_t max_level = MAX_LEVEL_LINEAR(voltage_data[volt_id].linear_range);
    uint32_t level_step = LEVEL_STEP_LINEAR(voltage_data[volt_id].linear_range);
    uint32_t num_levels = NUM_LEVELS_LINEAR(voltage_data[volt_id].linear_range);
    uint32_t i;

    if (level < min_level || level > max_level)
        return false;

    for (i = 0; i < num_levels; i++) {
        if (level == min_level + level_step * i)
            return true;
    }

    return false;
}

enum rpmi_error volt_get_level(void *priv,
                               uint32_t volt_id,
                               int32_t *volt_level)
{
    if (volt_id > RPMI_VOLTAGE_DOMAIN_COUNT)
        return RPMI_ERR_INVALID_PARAM;

    if (!volt_level)
        return RPMI_ERR_INVALID_PARAM;

    *volt_level = platvolts_ctx[volt_id].level;

    return RPMI_SUCCESS;
}

enum rpmi_error volt_set_level(void *priv,
                               uint32_t volt_id,
                               int32_t *volt_level)
{
    if (volt_id > RPMI_VOLTAGE_DOMAIN_COUNT)
        return RPMI_ERR_INVALID_PARAM;

    if (!volt_level)
        return RPMI_ERR_INVALID_PARAM;

    if (voltage_data[volt_id].voltage_type == RPMI_VOLT_TYPE_DISCRETE)
        platvolts_ctx[volt_id].level = *volt_level;
    else if (voltage_data[volt_id].voltage_type == RPMI_VOLT_TYPE_LINEAR) {
        if (volt_linear_level_change_match(priv, volt_id, *volt_level))
            platvolts_ctx[volt_id].level = *volt_level;
        else
            return RPMI_ERR_INVALID_PARAM;
    }

    return RPMI_SUCCESS;
}

const struct rpmi_voltage_platform_ops voltage_ops = {
    .get_config = volt_get_config,
    .set_config = volt_set_config,
    .get_level  = volt_get_level,
    .set_level  = volt_set_level,
    .get_supp_levels = volt_get_supp_levels,
};

int add_voltage_group(struct rpmi_context *vctx)
{
    struct rpmi_service_group *voltgrp;

    voltgrp = rpmi_service_group_voltage_create(RPMI_VOLTAGE_DOMAIN_COUNT,
                                                voltage_data,
                                                &voltage_ops,
                                                NULL);
    if (!voltgrp) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: voltage service group create failed\n",  __func__);
        return -1;
    }

    rpmi_context_add_group(vctx, voltgrp);

    return 0;
}
