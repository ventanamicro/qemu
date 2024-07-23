#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "librpmi.h"

struct platform_clocks_context {
    uint32_t current_state;
    uint64_t current_rate;
};


int add_clock_group(struct rpmi_context *rctx);

#define RPMI_CLOCK_COUNT        6

static uint64_t rate_linear[3] = {0x1111111122222222, 0xbbbbbbbbcccccccc,
                                    0x2222222222222222};
static uint64_t rate_discrete[6] = {0x1111111122222222, 0x2222222233333333,
                                    0x3333333344444444, 0x4444444455555555,
                                    0x5555555566666666, 0x6666666677777777};

static struct rpmi_clock_data clock_data[] = {
	[0] = {
		.name = "clock0",
		.parent_id = -1U,
        .transition_latency_ms = 100,
        .rate_count = 3,
        .clock_type = RPMI_CLK_TYPE_LINEAR,
        .clock_rate_array = rate_linear,
	},

	[1] = {
		.name = "clock1",
		.parent_id = 0,
        .transition_latency_ms = 100,
        .rate_count = 3,
        .clock_type = RPMI_CLK_TYPE_LINEAR,
        .clock_rate_array = rate_linear,
	},

	[2] = {
		.name = "clock2",
		.parent_id = 0,
        .transition_latency_ms = 50,
        .rate_count = 6,
        .clock_type = RPMI_CLK_TYPE_DISCRETE,
        .clock_rate_array = rate_discrete,
	},

	[3] = {
		.name = "clock3",
		.parent_id = 1,
        .transition_latency_ms = 50,
        .rate_count = 6,
        .clock_type = RPMI_CLK_TYPE_DISCRETE,
        .clock_rate_array = rate_discrete,
	},

	[4] = {
		.name = "clock4",
		.parent_id = 1,
        .transition_latency_ms = 100,
        .rate_count = 3,
        .clock_type = RPMI_CLK_TYPE_LINEAR,
        .clock_rate_array = rate_linear,
	},

	[5] = {
		.name = "clock5",
		.parent_id = 4,
        .transition_latency_ms = 50,
        .rate_count = 6,
        .clock_type = RPMI_CLK_TYPE_DISCRETE,
        .clock_rate_array = rate_discrete,
	},
};

static struct platform_clocks_context platclks_ctx[RPMI_CLOCK_COUNT] = {
    [0] = {.current_rate = 0x1111111122222222, .current_state = RPMI_CLK_STATE_ENABLED},
    [1] = {.current_rate = 0xbbbbbbbbcccccccc, .current_state = RPMI_CLK_STATE_ENABLED},
    [2] = {.current_rate = 0x2222222233333333, .current_state = RPMI_CLK_STATE_ENABLED},
    [3] = {.current_rate = 0x3333333344444444, .current_state = RPMI_CLK_STATE_ENABLED},
    [4] = {.current_rate = 0x1111111122222222, .current_state = RPMI_CLK_STATE_ENABLED},
    [5] = {.current_rate = 0x5555555566666666, .current_state = RPMI_CLK_STATE_ENABLED},
};

enum rpmi_error platform_set_state(void *priv, uint32_t clk_id,
                                   enum rpmi_clock_state state);

enum rpmi_error platform_get_state_and_rate(void *priv, uint32_t clk_id,
                                            enum rpmi_clock_state *state,
                                            uint64_t *rate);

enum rpmi_error platform_rate_change_match(void *priv, uint32_t clk_id,
                                            uint64_t rate);

enum rpmi_error platform_set_rate(void *priv, uint32_t clk_id,
                                  enum rpmi_clock_rate_match match,
                                  uint64_t rate, rpmi_uint64_t *new_rate);

enum rpmi_error platform_set_rate_recalc(void *priv, uint32_t clk_id,
                                   uint64_t parent_rate, uint64_t *new_rate);

enum rpmi_error platform_set_state(void *priv,
                                   uint32_t clk_id,
                                   enum rpmi_clock_state state)
{

    if (clk_id > RPMI_CLOCK_COUNT)
        return RPMI_ERR_INVALID_PARAM;

    if (state >= RPMI_CLK_STATE_MAX_IDX)
        return RPMI_ERR_INVALID_PARAM;

    platclks_ctx[clk_id].current_state = state;

    return RPMI_SUCCESS;
}

enum rpmi_error platform_get_state_and_rate(void *priv,
                                            uint32_t clk_id,
                                            enum rpmi_clock_state *state,
                                            uint64_t *rate)
{
    if (clk_id > RPMI_CLOCK_COUNT)
        return RPMI_ERR_INVALID_PARAM;

    if (!state && !rate)
        return RPMI_ERR_INVALID_PARAM;

    if (state)
        *state = platclks_ctx[clk_id].current_state;

    if (rate)
        *rate = platclks_ctx[clk_id].current_rate;

    return RPMI_SUCCESS;

}

#define CLOCK_DIFF_NEGATIVE      0x100
#define CLOCK_DIFF_POSITIVE      0x100

enum rpmi_error platform_rate_change_match(void *priv,
                                   uint32_t clk_id,
                                   uint64_t rate)
{
    if (clk_id > RPMI_CLOCK_COUNT)
        return false;

    uint64_t current_rate = platclks_ctx[clk_id].current_rate;

    if (rate > current_rate && (rate - current_rate) > CLOCK_DIFF_POSITIVE)
        return true;

    if (rate < current_rate && (current_rate - rate) < CLOCK_DIFF_NEGATIVE)
        return true;

    return false;
}

enum rpmi_error platform_set_rate(void *priv,
                                  uint32_t clk_id,
                                  enum rpmi_clock_rate_match match,
                                  uint64_t rate,
                                  rpmi_uint64_t *new_rate)
{
    if (clk_id > RPMI_CLOCK_COUNT)
        return RPMI_ERR_INVALID_PARAM;

    if (!platform_rate_change_match(NULL, clk_id, rate))
        return RPMI_ERR_ALREADY;

    switch(match) {
        case RPMI_CLK_RATE_MATCH_ROUND_UP:
            platclks_ctx[clk_id].current_rate = rate + 0x100;
            break;
        case RPMI_CLK_RATE_MATCH_ROUND_DOWN:
            platclks_ctx[clk_id].current_rate = rate - 0x100;
            break;
        case RPMI_CLK_RATE_MATCH_PLATFORM:
            platclks_ctx[clk_id].current_rate = rate + 0x200;
            break;
        default:
            return RPMI_ERR_INVALID_PARAM;
    };

    *new_rate = platclks_ctx[clk_id].current_rate;

    return RPMI_SUCCESS;
}

enum rpmi_error platform_set_rate_recalc(void *priv,
                                   uint32_t clk_id,
                                   uint64_t parent_rate,
                                   uint64_t *new_rate)
{

    uint64_t rate = platclks_ctx[clk_id].current_rate;
    rate = (rate/parent_rate) * 1.5;
    platclks_ctx[clk_id].current_rate = rate;
    *new_rate = rate;
    return RPMI_SUCCESS;
}

const struct rpmi_clock_platform_ops clock_ops = {
    .set_rate = platform_set_rate,
    .set_state = platform_set_state,
    .set_rate_recalc = platform_set_rate_recalc,
    .get_state_and_rate = platform_get_state_and_rate,
    .rate_change_match = platform_rate_change_match,
};

int add_clock_group(struct rpmi_context *rctx)
{
    struct rpmi_service_group *clkgrp;

    clkgrp = rpmi_service_group_clock_create(RPMI_CLOCK_COUNT,
                                                  clock_data,
                                                  &clock_ops,
                                                  NULL);
    if (!clkgrp) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: clock service group create failed\n",
                      __func__);
        return -1;
    }

    rpmi_context_add_group(rctx, clkgrp);

    return 0;
}
