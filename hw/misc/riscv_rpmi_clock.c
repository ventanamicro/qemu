/*
 * vms_rpmi_transport_base.c
 * RPMI clock service group message handling routines.
 *
 * Copyright (c) 2023
 *
 * Authors:
 * Subrahmanya Lingapa <slingappa@ventanamicro.com>
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
#include "hw/sysbus.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "sysemu/runstate.h"
#include "hw/misc/riscv_rpmi.h"
#include "exec/address-spaces.h"
#include "riscv_rpmi_transport.h"
#include "hw/misc/rpmi_clock.h"

#define RATE_MIN_IDX                    0
#define RATE_MAX_IDX                    1
#define RATE_STEP_IDX                   2

/* Rate round up/down/platform */
#define RPMI_RATE_MATCH_POS             30
#define RPMI_RATE_MATCH_WID             2
#define RPMI_RATE_MATCH_MSK             \
        (((1UL << RPMI_RATE_MATCH_WID) - 1UL) << RPMI_RATE_MATCH_POS)
#define GET_RATE_MATCH(f)   \
        ((f & RPMI_RATE_MATCH_MSK) >> RPMI_RATE_MATCH_POS)

/* Clock rate format */
#define RPMI_RATE_FORMAT_POS            30
#define RPMI_RATE_FORMAT_WID            2
#define RPMI_RATE_FORMAT_MSK            \
        (((1UL << RPMI_RATE_FORMAT_WID) - 1UL) << RPMI_RATE_FORMAT_POS)
#define GET_RATE_FORMAT(f) \
        ((f & RPMI_RATE_FORMAT_MSK) >> RPMI_RATE_FORMAT_POS)
#define SET_RATE_FORMAT(v, f)           \
    ((v & ~RPMI_RATE_FORMAT_MSK) | ((f << RPMI_RATE_FORMAT_POS) & RPMI_RATE_FORMAT_MSK))

#define GET_RATE_U64(hi_u32, lo_u32)  ((u64)hi_u32 << 32 | lo_u32)
#define GET_RATE_HI_U32(rate_u64)     ((u32)(rate_u64 >> 32))
#define GET_RATE_LO_U32(rate_u64)     ((u32)rate_u64)

/* Clock RPMI Response */
union rpmi_clk_resp_data {
    /* 0x01 - ENABLE_NOTIFICATION */
    struct {
        u32 status;
    } notify_enable;

    /* 0x02 - GET_SYSTEM_CLKS */
    struct {
        u32 status;
        u32 num_clocks;
    } get_clocks;

    /* 0x03 - GET_ATTRIBUTES */
    struct {
        u32 status;
        u32 flags;
        u32 num_rates;
        u32 transition_latency_ms;
        u8 name[RPMI_CLK_NAME_MAX_LEN];
    } get_attrs;

    /* 0x04 - GET_SUPPORTED_RATES */
    struct {
        u32 status;
        u32 flags;
        u32 remaining;
        u32 returned;
        struct rpmi_clk_rate clk_rate_array[RPMI_CLK_RATES_SUPPORTED_MSG];
    } get_supp_rates;

    /* 0x05 - SET_CONFIG */
    struct {
        u32 status;
    } set_config;

    /* 0x06 - GET_CONFIG */
    struct {
        u32 status;
        u32 config;
    } get_config;

    /* 0x07 - SET_RATE */
    struct {
        u32 status;
    } set_rate;

    /* 0x08 - GET_RATE */
    struct {
        u32 status;
        struct rpmi_clk_rate clock_rate;
    } get_rate;
};

/* global struct to store the notification subscribers */
struct grp_subscribers clk_subs[RPMI_SRVGRP_ID_MAX_COUNT] = {0};

/* global struct to store the events */
struct grp_events clk_events[RPMI_SRVGRP_ID_MAX_COUNT] = {0};

enum {
    RPMI_CLK_DISABLED,
    RPMI_CLK_ENABLED
};

int handle_rpmi_grp_clock(struct rpmi_message *msg, int xport_id) {
    bool rate_found = false;
    int ret, clk_rate_idx = 0;
    int srvid = GET_SERVICE_ID(msg);
    u32 clk_id, qid = RPMI_QUEUE_IDX_P2A_ACK;
    u32 num_clocks = 0, resp_datalen = 0;
    u32 *req_data;
    u8 queue_data[RPMI_QUEUE_SLOT_SIZE];
    struct rpmi_message *rpmi_msg;
    union rpmi_clk_resp_data resp_data;
    struct rpmi_clk *rpmi_clk, *c;
    enum rpmi_clk_state clk_state;
    enum rpmi_clk_rate_match rate_match;
    u64 rate, min, max, step, temp;
    u32 event_id, remaining, returned;
    size_t idx;
    size_t i = 0, j = 0;

    rpmi_clk = rpmi_get_clock_data(xport_id);
    if (!rpmi_clk) {
        qemu_log_mask(LOG_GUEST_ERROR, "RPMI Clock Data not available\n");
        return -1;
    }

    /* Get the number of clocks from rpmi clock data */
    for (c = rpmi_clk; c->num_rates; c++)
        num_clocks += 1;

    rpmi_msg = (struct rpmi_message *)queue_data;
    req_data = (u32 *)msg->data;

    memset(rpmi_msg, 0, RPMI_QUEUE_SLOT_SIZE);
    memset(&resp_data, 0, sizeof(union rpmi_clk_resp_data));

    switch (srvid) {

    /* Enable Notifications */
    case RPMI_CLK_SRV_ENABLE_NOTIFICATION:
        event_id = le32toh(req_data[0]);
        clk_subs[event_id].state =  RPMI_CLK_ENABLED;
        clk_subs[event_id].subscribers_cnt++;
        resp_data.notify_enable.status = RPMI_SUCCESS;
        resp_datalen = sizeof(resp_data.notify_enable);
        break;

    /* Get Number of System Clocks */
    case RPMI_CLK_SRV_GET_SYSTEM_CLOCKS:
        resp_data.get_clocks.status = RPMI_SUCCESS;
        resp_data.get_clocks.num_clocks = num_clocks;
        resp_datalen = sizeof(resp_data.get_clocks);
        break;

    /* Get Clock Attributes */
    case RPMI_CLK_SRV_GET_ATTRIBUTES:
        clk_id = le32toh(req_data[0]);
        if (clk_id > num_clocks) {
            resp_data.get_attrs.status = RPMI_ERR_NOTFOUND;
            resp_datalen = sizeof(resp_data.get_attrs);
            qemu_log_mask(LOG_GUEST_ERROR, "invalid clock id\n");
            break;
        }

        resp_data.get_attrs.status = RPMI_SUCCESS;
        resp_data.get_attrs.flags = rpmi_clk[clk_id].type << 30;
        resp_data.get_attrs.num_rates = rpmi_clk[clk_id].num_rates;
        resp_data.get_attrs.transition_latency_ms =
                            rpmi_clk[clk_id].transition_latency_ms;
        memcpy(&resp_data.get_attrs.name, &rpmi_clk[clk_id].name,
               RPMI_CLK_NAME_MAX_LEN);
        resp_datalen = sizeof(resp_data.get_attrs);
        break;

    /* Get Clock Supported Rates */
    case RPMI_CLK_SRV_GET_SUPPORTED_RATES:
        clk_id = le32toh(req_data[0]);
        clk_rate_idx = le32toh(req_data[1]);
        if (clk_id > num_clocks) {
            resp_data.get_supp_rates.status = RPMI_ERR_NOTFOUND;
            resp_datalen = resp_data.get_supp_rates.returned *
                          sizeof(struct rpmi_clk_rate) +
                          (4 * sizeof(u32));
            qemu_log_mask(LOG_GUEST_ERROR,
                          "RPMI_CLK: GET_SUPPORTED_RATES: invalid clock id\n");
            break;
        }

        resp_data.get_supp_rates.status = RPMI_SUCCESS;
        resp_data.get_supp_rates.flags = 0;

        if (rpmi_clk[clk_id].type == RPMI_CLK_TYPE_LINEAR) {
            for (i = 0; i < 3; i++) {
                resp_data.get_supp_rates.clk_rate_array[i].lo =
                                rpmi_clk[clk_id].clk_data[i].lo;
                resp_data.get_supp_rates.clk_rate_array[i].hi =
                                rpmi_clk[clk_id].clk_data[i].hi;
            }

            resp_data.get_supp_rates.remaining = 0;
            resp_data.get_supp_rates.returned = 3;
        }
        else { /* RPMI_CLK_TYPE_DISCRETE */
            if (clk_rate_idx >= rpmi_clk[clk_id].num_rates) {
                resp_data.get_supp_rates.status = RPMI_ERR_OUTOFRANGE;
                qemu_log_mask(LOG_GUEST_ERROR, "clock rate index out of range\n");
                resp_datalen = resp_data.get_supp_rates.returned *
                          sizeof(struct rpmi_clk_rate) +
                          (4 * sizeof(u32));
                break;
            }

            remaining = rpmi_clk[clk_id].num_rates - clk_rate_idx;
            if (remaining > RPMI_CLK_RATES_SUPPORTED_MSG)
                returned = RPMI_CLK_RATES_SUPPORTED_MSG;
            else
                returned = remaining;

            for (i = clk_rate_idx; i <= (clk_rate_idx+returned-1); i++, j++) {
                resp_data.get_supp_rates.clk_rate_array[j].lo =
                                rpmi_clk[clk_id].clk_data[i].lo;
                resp_data.get_supp_rates.clk_rate_array[j].hi =
                                rpmi_clk[clk_id].clk_data[i].hi;
            }

            remaining = rpmi_clk[clk_id].num_rates - (clk_rate_idx + returned);
            resp_data.get_supp_rates.returned = returned;
            resp_data.get_supp_rates.remaining = remaining;
            resp_data.get_supp_rates.status = RPMI_SUCCESS;
        }

        resp_datalen = resp_data.get_supp_rates.returned *
                          sizeof(struct rpmi_clk_rate) +
                          (4 * sizeof(u32));
        break;

    /* Set Clock Config */
    case RPMI_CLK_SRV_SET_CONFIG:
        clk_id = le32toh(req_data[0]);
        if (clk_id > num_clocks) {
            resp_data.set_config.status = RPMI_ERR_NOTFOUND;
            resp_datalen = sizeof(resp_data.set_config);
            qemu_log_mask(LOG_GUEST_ERROR,
                          "RPMI_CLK: SET_CONFIG: invalid clock id\n");
            break;
        }

        clk_state = le32toh(req_data[1]);
        if (clk_state > RPMI_CLK_STATE_MAX_IDX) {
            resp_data.set_config.status = RPMI_ERR_INVAL;
            resp_datalen = sizeof(resp_data.set_config);
            qemu_log_mask(LOG_GUEST_ERROR,
                        "RPMI_CLK: SET_CONFIG: invalid clock-%u config\n",
                        clk_id);
            break;
        }

        if (rpmi_clk[clk_id].state == clk_state) {
            resp_data.set_config.status = RPMI_ERR_ALREADY;
            qemu_log_mask(LOG_GUEST_ERROR,
                "RPMI_CLK: SET_CONFIG: clock-%u already in requested state\n",
                clk_id);
            resp_datalen = sizeof(resp_data.set_config);
            break;
        }

        rpmi_clk[clk_id].state = clk_state;
        resp_data.set_config.status = RPMI_SUCCESS;
        resp_datalen = sizeof(resp_data.set_config);
        break;

    /* Get Clock Config */
    case RPMI_CLK_SRV_GET_CONFIG:
        clk_id = le32toh(req_data[0]);
        if (clk_id > num_clocks) {
            resp_data.get_config.status = RPMI_ERR_NOTFOUND;
            resp_datalen = sizeof(resp_data.get_config);
            qemu_log_mask(LOG_GUEST_ERROR,
                          "RPMI_CLK: GET_CONFIG: invalid clock id\n");
            break;
        }

        resp_data.get_config.config = rpmi_clk[clk_id].state;
        resp_data.get_config.status = RPMI_SUCCESS;
        resp_datalen = sizeof(resp_data.get_config);
        break;

    /* Set Clock Rate */
    case RPMI_CLK_SRV_SET_RATE:
        clk_id = le32toh(req_data[0]);
        if (clk_id > num_clocks) {
            resp_data.set_rate.status = RPMI_ERR_NOTFOUND;
            resp_datalen = sizeof(resp_data.set_rate);
            qemu_log_mask(LOG_GUEST_ERROR,
                          "RPMI_CLK: SET_RATE: invalid clock id\n");
            break;
        }

        if (rpmi_clk[clk_id].state == RPMI_CLK_STATE_DISABLED) {
            resp_data.set_rate.status = RPMI_ERR_DENIED;
            resp_datalen = sizeof(resp_data.set_rate);
            qemu_log_mask(LOG_GUEST_ERROR,
                        "RPMI CLK: SET_RATE: clock-%d is disabled\n", clk_id);
            break;
        }

        rate_match = GET_RATE_MATCH(le32toh(req_data[1]));
        if (rate_match > RPMI_CLK_RATE_MATCH_MAX_IDX) {
                qemu_log_mask(LOG_GUEST_ERROR,
                "RPMI CLK: SET_RATE: invalid rate match, fallback to platform\n");
                rate_match = RPMI_CLK_RATE_PLATFORM;
        }

        rate = GET_RATE_U64(le32toh(req_data[3]), le32toh(req_data[2]));
        if (rpmi_clk[clk_id].current_rate == rate) {
            resp_data.set_rate.status = RPMI_SUCCESS;
            resp_datalen = sizeof(resp_data.set_rate);
            break;
        }

        if (rpmi_clk[clk_id].type == RPMI_CLK_TYPE_LINEAR) {
            min = GET_RATE_U64(rpmi_clk[clk_id].clk_data[RATE_MIN_IDX].hi,
                                rpmi_clk[clk_id].clk_data[RATE_MIN_IDX].lo);
            max = GET_RATE_U64(rpmi_clk[clk_id].clk_data[RATE_MAX_IDX].hi,
                                rpmi_clk[clk_id].clk_data[RATE_MAX_IDX].lo);
            step = GET_RATE_U64(rpmi_clk[clk_id].clk_data[RATE_STEP_IDX].hi,
                                rpmi_clk[clk_id].clk_data[RATE_STEP_IDX].lo);

            for (temp = min; temp <= max;  temp+= step) {
                if (rate <= temp) {
                    rate_found = true;
                    break;
                }
            }
            if (rate_found) {
                if (rate_match == RPMI_CLK_RATE_ROUND_UP)
                    rate = temp;
                else if (rate_match == RPMI_CLK_RATE_ROUND_DOWN)
                    rate = temp - step;
                else
                    rate = temp;
            }
        } else {
            for (idx = 0; idx < rpmi_clk[clk_id].num_rates; idx++) {
                temp = GET_RATE_U64(rpmi_clk[clk_id].clk_data[idx].hi,
                                    rpmi_clk[clk_id].clk_data[idx].lo);
                if (rate <= temp) {
                        rate_found = true;
                        break;
                }
            }

            if (rate_found) {
                if (rate_match == RPMI_CLK_RATE_ROUND_UP)
                        rate = temp;
                else if (rate_match == RPMI_CLK_RATE_ROUND_DOWN) {
                    if (idx > 0) {
                        temp = GET_RATE_U64(rpmi_clk[clk_id].clk_data[idx - 1].hi,
                                        rpmi_clk[clk_id].clk_data[idx - 1].lo);
                        rate = temp;
                    } else
                        rate = temp;
                } else
                    rate = temp;
            }
        }

        rpmi_clk[clk_id].current_rate = rate;
        resp_data.set_rate.status = RPMI_SUCCESS;
        resp_datalen = sizeof(resp_data.set_rate);
        break;

    /* Get Clock Rate */
    case RPMI_CLK_SRV_GET_RATE:
        clk_id = le32toh(req_data[0]);
        if (clk_id > num_clocks) {
            resp_data.get_rate.status = RPMI_ERR_NOTFOUND;
            resp_datalen = sizeof(resp_data.get_rate);
            qemu_log_mask(LOG_GUEST_ERROR,
                          "RPMI_CLK: GET_RATE: invalid clock id\n");
            break;
        }

        resp_data.get_rate.status = RPMI_SUCCESS;
        resp_data.get_rate.clock_rate.lo =
                    GET_RATE_LO_U32(rpmi_clk[clk_id].current_rate);
        resp_data.get_rate.clock_rate.hi =
                    GET_RATE_HI_U32(rpmi_clk[clk_id].current_rate);
        resp_datalen = sizeof(resp_data.get_rate);

        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Unhandled Clock Service ID - %x\n",
                      __func__, srvid);
        return -1;
    }

    rpmi_pack_message(RPMI_MSG_ACKNOWLDGEMENT, RPMI_SRVGRP_CLOCK, srvid,
                      GET_TOKEN(msg), resp_datalen, &resp_data, rpmi_msg);

    ret = smq_enqueue(xport_id, qid, rpmi_msg);
    if (ret) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: smq_enqueue failed - %x\n", __func__, ret);
        return -1;
    }

    return 0;
}
