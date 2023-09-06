/*
  * rpmi_clock.h
  * RPMI Clock
  *
  *
  * Copyright (c) 2023
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

#ifndef HW_rpmi_clock_H
#define HW_rpmi_clock_H

/* A clock rate representation */
struct rpmi_clk_rate {
    u32 lo;
    u32 hi;
};

/* Number of rates in a GET_SUPPORTED_RATES response */
#define RPMI_CLK_RATES_SUPPORTED_MSG    \
    ((RPMI_MSG_DATA_SIZE - (sizeof(uint32_t) * 4))/sizeof(struct rpmi_clk_rate))
#define RPMI_CLK_NAME_MAX_LEN           16

enum rpmi_clk_rate_match {
    RPMI_CLK_RATE_ROUND_DOWN = 0,
    RPMI_CLK_RATE_ROUND_UP = 1,
    RPMI_CLK_RATE_PLATFORM = 2,
    RPMI_CLK_RATE_MATCH_MAX_IDX,
};

enum rpmi_clk_state {
    RPMI_CLK_STATE_DISABLED,
    RPMI_CLK_STATE_ENABLED,
    RPMI_CLK_STATE_MAX_IDX,
};

enum rpmi_clk_type {
    RPMI_CLK_TYPE_DISCRETE,
    RPMI_CLK_TYPE_LINEAR,
    RPMI_CLK_TYPE_MAX_IDX,
};

/* A clock context and associated data */
struct rpmi_clk {
    u32 num_rates;
    enum rpmi_clk_type type;
    u32 transition_latency_ms;
    enum rpmi_clk_state state;
    u64 current_rate;
    char name[RPMI_CLK_NAME_MAX_LEN];
    struct rpmi_clk_rate *clk_data;
};

#endif
