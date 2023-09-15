 /*
  * vms_rpmi_transport_hsm.c
  * RPMI HSM service group message handling routines.
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
#include "target/riscv/cpu.h"
#include "hw/riscv/numa.h"
#include "hw/misc/riscv_rpmi.h"
#include "exec/address-spaces.h"
#include "riscv_rpmi_transport.h"

typedef union rpmi_hsm_response_data_u {
    uint8_t raw[RPMI_MSG_DATA_SIZE];
    struct {
        uint32_t status;
    } start;
    struct {
        uint32_t status;
    } stop;
    struct {
        uint32_t status;
        uint32_t state;
    } state;
    struct {
        uint32_t status;
        uint32_t remaining;
        uint32_t returned;
        uint32_t types[(RPMI_MSG_DATA_SIZE -
                        (sizeof(uint32_t) * 3)) / sizeof(uint32_t)];
    } susp_types;
    struct {
        uint32_t status;
        uint32_t remaining;
        uint32_t returned;
        uint32_t hartid[(RPMI_MSG_DATA_SIZE -
                         (sizeof(uint32_t) * 3)) / sizeof(uint32_t)];
    } hart_list;
    struct {
        uint32_t status;
    } susp_attrs;
   struct {
        uint32_t status;
    } susp;
   struct {
        uint32_t status;
    } notify_enable;

} rpmi_hsm_resp_data_t;

#define RPMI_HSM_NAME_MAX_LEN    32
#define RPMI_HSM_NAME_MAX_SUBS_PER_SVCGRP    32
#define RPMI_HSM_NAME_MAX_EVENTS_PER_SVCGRP    32

enum {
    RPMI_HSM_DISABLED,
    RPMI_HSM_ENABLED
};

/* global struct to store the notification subscribers */
struct grp_subscribers hsm_subs[RPMI_SRVGRP_ID_MAX_COUNT] = {0};

/* global struct to store the events */
struct grp_events hsm_events[RPMI_SRVGRP_ID_MAX_COUNT] = {0};

#define HSM_MAX_CPUS 128
int hart_states[HSM_MAX_CPUS] = {
    [0 ... HSM_MAX_CPUS - 1] = RPMI_HSM_STATE_STARTED
};

bool execute_rpmi_hsm_stop(void *env)
{
    CPUState *cs = env_cpu(env);
    riscv_set_wfi_cb(env,  NULL);
    cs->stop = true;
    cs->hold_stop = true;
    qemu_cpu_kick(cs);
    hart_states[cs->cpu_index] = RPMI_HSM_STATE_STOPPED;
    return true;
}

int handle_rpmi_grp_hsm(struct rpmi_message *msg, int xport_id)
{
    int svc_id = GET_SERVICE_ID(msg);
    uint32_t qid = RPMI_QUEUE_IDX_P2A_ACK, rc = 0;
    uint32_t hart_id = 0, event_id = 0;
    uint8_t queue_data[RPMI_QUEUE_SLOT_SIZE];
    uint32_t *req_data = (uint32_t *)msg->data;
    struct rpmi_message *tmsgbuf = (struct rpmi_message *)queue_data;
    uint32_t resp_dlen = 0;
    rpmi_hsm_resp_data_t resp_data;
    CPUState *cpu;
    CPURISCVState *env;
    uint64_t harts_mask;
    uint32_t skip_count;
    int i, returned, remaining;

    memset(tmsgbuf, 0, sizeof(queue_data));
    memset(&resp_data, 0, sizeof(resp_data));

    qemu_log_mask(LOG_GUEST_ERROR, "%s: HSM service ID: %x\n",
                __func__, svc_id);
    switch (svc_id) {

    case RPMI_HSM_SRV_ENABLE_NOTIFICATION:
         event_id = le32toh(req_data[0]);
         hsm_subs[event_id].state =  RPMI_HSM_ENABLED;
         hsm_subs[event_id].subscribers_cnt++;
         resp_data.notify_enable.status = 0;
         resp_dlen = sizeof(resp_data.notify_enable);
         break;

    case RPMI_HSM_SRV_HART_START:
        hart_id = req_data[0];
        cpu = cpu_by_arch_id(hart_id);

        if (hart_states[cpu->cpu_index] == RPMI_HSM_STATE_STARTED) {
            resp_data.start.status = 0;
        } else if (hart_states[cpu->cpu_index] != RPMI_HSM_STATE_STOPPED) {
            /* hart not running return error */
            resp_data.stop.status = -1;
        } else {
            hart_states[cpu->cpu_index] = RPMI_HSM_STATE_START_PENDING;
            cpu->hold_stop = false;
            cpu_resume(cpu);
            hart_states[cpu->cpu_index] = RPMI_HSM_STATE_STARTED;
            resp_data.start.status = 0;
        }
        resp_dlen = sizeof(resp_data.start);
        break;

    case RPMI_HSM_SRV_HART_STOP:
        hart_id = req_data[0];
        cpu = cpu_by_arch_id(hart_id);

        if (hart_states[cpu->cpu_index] != RPMI_HSM_STATE_STARTED) {
            /* hart not running return error */
            resp_data.stop.status = -1;
        } else {
            hart_states[cpu->cpu_index] = RPMI_HSM_STATE_STOP_PENDING;
            env = &RISCV_CPU(cpu)->env;
            assert(env);
            riscv_set_wfi_cb(env, execute_rpmi_hsm_stop);
            resp_data.stop.status = 0;
        }
        resp_dlen = sizeof(resp_data.stop);
        break;

    case RPMI_HSM_SRV_HART_SUSPEND:
        /* we do not support any supsend types */
        resp_data.susp.status = -1;
        resp_dlen = sizeof(resp_data.susp);
        break;

    case RPMI_HSM_SRV_GET_HART_STATUS:
        hart_id = req_data[0];
        cpu = cpu_by_arch_id(hart_id);
        resp_data.state.status = 0;
        resp_data.state.state = hart_states[cpu->cpu_index];
        resp_dlen = sizeof(resp_data.state);
        break;

    case RPMI_HSM_SRV_GET_SUSPEND_TYPES:
        /* we do not support any supsend types */
        resp_data.susp_types.status = 0;
        resp_data.susp_types.returned = 0;
        resp_data.susp_types.remaining = 0;
        resp_dlen = sizeof(resp_data.susp_types);
        break;

    case RPMI_HSM_SRV_GET_HART_LIST:
        harts_mask =  rpmi_get_harts_mask(xport_id);
        skip_count = req_data[0];
        returned = 0;
        remaining = 0;

        hart_id = 0;
        while (harts_mask && skip_count) {
            if (harts_mask & 0x1)
                skip_count--;
            hart_id++;
            harts_mask >>= 1;
        }

        for (i = 0; i < ARRAY_SIZE(resp_data.hart_list.hartid); i++) {
             while (!(harts_mask & 0x1) && hart_id < 64) {
                 hart_id++;
                 harts_mask >>= 1;
             }
             if (!harts_mask)
                 break;
             resp_data.hart_list.hartid[i] = hart_id;
             hart_id++;
             harts_mask >>= 1;
             returned++;
        }

        while (harts_mask) {
            if (harts_mask & 0x1)
                remaining++;
            harts_mask >>= 1;
        }

        resp_data.hart_list.status = 0;
        resp_data.hart_list.remaining = remaining;
        resp_data.hart_list.returned = returned;
        resp_dlen = sizeof(resp_data.hart_list);
        break;

    case RPMI_HSM_SRV_GET_SUSPEND_INFO:
        /* we do not support any supsend types */
        resp_data.susp_attrs.status = -1;
        resp_dlen = sizeof(resp_data.susp_attrs);
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Unhandled hsm service ID: %x\n",
                __func__, svc_id);
        return -1;
    }

    rpmi_pack_message(RPMI_MSG_ACKNOWLDGEMENT,
            RPMI_SRVGRP_HSM,
            svc_id, GET_TOKEN(msg),
            resp_dlen, &resp_data, tmsgbuf);

    rc = smq_enqueue(xport_id, qid, tmsgbuf);
    if (rc) {
        return -1;
    }

    return 0;
}
