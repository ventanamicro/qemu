#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "librpmi.h"

struct platform_dpwr_context {
    int32_t  current_state;
};

int add_device_power_group(struct rpmi_context *dctx);

#define RPMI_DPWR_DOMAIN_COUNT        6

static struct rpmi_dpwr_data dpwr_data[] = {
    [0] = {
        .name = "dpwr0",
        .trans_latency = 100,
    },

    [1] = {
        .name = "dpwr1",
        .trans_latency = 120,
    },

    [2] = {
        .name = "dpwr2",
        .trans_latency = 90,
    },

    [3] = {
        .name = "dpwr3",
        .trans_latency = 110,
    },

    [4] = {
        .name = "dpwr4",
        .trans_latency = 130,
    },

    [5] = {
        .name = "dpwr5",
        .trans_latency = 140,
    },
};

static struct platform_dpwr_context platdpwrs_ctx[RPMI_DPWR_DOMAIN_COUNT] = {
    [0] = {.current_state = RPMI_DPWR_STATE_ON },
    [1] = {.current_state = RPMI_DPWR_STATE_ON },
    [2] = {.current_state = RPMI_DPWR_STATE_INVALID },
    [3] = {.current_state = RPMI_DPWR_STATE_OFF },
    [4] = {.current_state = RPMI_DPWR_STATE_ON },
    [5] = {.current_state = RPMI_DPWR_STATE_INVALID },
};

enum rpmi_error dpwr_get_state(void *priv, uint32_t dpwr_id,
                               uint32_t *dpwr_state);
enum rpmi_error dpwr_set_state(void *priv, uint32_t dpwr_id,
                               uint32_t dpwr_state);
enum rpmi_error dpwr_get_attrs(void *priv, uint32_t dpwr_id,
                               struct rpmi_dpwr_attrs *dpwr_attrs);

enum rpmi_error dpwr_get_state(void *priv,
                               uint32_t dpwr_id,
                               uint32_t *dpwr_state)
{
    if (dpwr_id > RPMI_DPWR_DOMAIN_COUNT)
        return RPMI_ERR_INVALID_PARAM;

    *dpwr_state = platdpwrs_ctx[dpwr_id].current_state;

    return RPMI_SUCCESS;
}

enum rpmi_error dpwr_set_state(void *priv,
                               uint32_t dpwr_id,
                               uint32_t dpwr_state)
{
    if (dpwr_id > RPMI_DPWR_DOMAIN_COUNT)
        return RPMI_ERR_INVALID_PARAM;

    if (dpwr_state >= RPMI_DPWR_STATE_MAX)
        return RPMI_ERR_INVALID_PARAM;

    platdpwrs_ctx[dpwr_id].current_state = dpwr_state;

    return RPMI_SUCCESS;
}

const struct rpmi_dpwr_platform_ops dpwr_ops = {
    .get_state = dpwr_get_state,
    .set_state = dpwr_set_state,
};

int add_device_power_group(struct rpmi_context *rctx)
{
    struct rpmi_service_group *dpwrgrp;

    dpwrgrp = rpmi_service_group_dpwr_create(RPMI_DPWR_DOMAIN_COUNT,
                                             dpwr_data,
                                             &dpwr_ops,
                                             NULL);
    if (!dpwrgrp) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: dpwr service group create failed\n",  __func__);
        return -1;
    }

    rpmi_context_add_group(rctx, dpwrgrp);

    return 0;
}
