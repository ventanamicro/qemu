 /*
  * riscv_rpmi_transport_suspend.c
  * RPMI suspend service group message handling routines.
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
#include "target/riscv/cpu.h"
#include "sysemu/runstate.h"
#include "hw/misc/riscv_rpmi.h"
#include "exec/address-spaces.h"
#include "riscv_rpmi_transport.h"

typedef union rpmi_suspend_response_data_u {
    uint8_t raw[RPMI_MSG_DATA_SIZE];
    struct {
        uint32_t status;
    } notify_enable;

    struct {
        uint32_t status;
        union {
            uint32_t val;
            struct {
                uint32_t reserved0:30;
                uint32_t cust_res_addr_supported:1;
                uint32_t suspend_supported:1;
            };
        } suspend_flag;
    } get_attrs;
    struct {
        uint32_t status;
    } susp;
} rpmi_suspend_resp_data_t;

#define RPMI_SUSPEND_NAME_MAX_LEN    32
#define RPMI_SUSPEND_NAME_MAX_SUBS_PER_SVCGRP    32
#define RPMI_SUSPEND_NAME_MAX_EVENTS_PER_SVCGRP    32

enum {
    RPMI_SUSPEND_DISABLED,
    RPMI_SUSPEND_ENABLED
};

/* global struct to store the notification subscribers */
struct grp_subscribers suspend_subs[RPMI_SRVGRP_ID_MAX_COUNT] = {0};

/* global struct to store the events */
struct grp_events suspend_events[RPMI_SRVGRP_ID_MAX_COUNT] = {0};

/* global struct to store the suspend data intended to be consumed by PuC FW */
struct suspend_data_s {
    uint32_t sleep_state;
    uint32_t addr_h;
    uint32_t addr_l;
} g_puc_suspend_data = {0};

struct rpmi_srv_data_syssuspend {
    uint32_t hart_id;
    uint32_t suspend_type;
    u32 resume_addr_lo;
    u32 resume_addr_hi;
};

 enum rpmi_sysspnd_suspend_type {
     RPMI_SYSSUSP_SHUTDOWN = 0,
     RPMI_SYSSUSP_COLD_SUSPEND = 1,
     RPMI_SYSSUSP_SUSPEND = 2,
     RPMI_SYSSUSP_MAX_IDN_COUNT,
};

bool execute_rpmi_suspend(void *env)
{
    riscv_set_wfi_cb(env,  NULL);
    qemu_system_suspend_request();
    return false;
}

int handle_rpmi_grp_suspend(struct rpmi_message *msg, int xport_id)
{
    int svc_id = GET_SERVICE_ID(msg);
    uint32_t qid = RPMI_QUEUE_IDX_P2A_ACK, rc, event_id;
    uint32_t *req_data = (uint32_t *)msg->data;
    uint8_t queue_data[RPMI_QUEUE_SLOT_SIZE];
    struct rpmi_message *tmsgbuf = (struct rpmi_message *)queue_data;
    struct rpmi_srv_data_syssuspend *srv_data =
        (struct rpmi_srv_data_syssuspend *)msg->data;
    uint32_t suspend_type;
    uint32_t resp_dlen = 0;
    rpmi_suspend_resp_data_t resp_data;
    CPUState *cpu;
    CPURISCVState *env;


    memset(tmsgbuf, 0, sizeof(queue_data));
    memset(&resp_data, 0, sizeof(resp_data));

    switch (svc_id) {

    case RPMI_SYSSUSP_SRV_ENABLE_NOTIFICATION:
        event_id = le32toh(req_data[0]);
        suspend_subs[event_id].state =  RPMI_SUSPEND_ENABLED;
        suspend_subs[event_id].subscribers_cnt++;
        resp_data.notify_enable.status = 0;
        resp_dlen = sizeof(resp_data.notify_enable);
        break;


    case RPMI_SYSSUSP_SRV_GET_SYSTEM_SUSPEND_ATTRIBUTES:
        resp_data.get_attrs.status = 0;
        resp_data.get_attrs.suspend_flag.suspend_supported = 1;
        resp_dlen = sizeof(resp_data.get_attrs);
        qemu_log_mask(LOG_GUEST_ERROR,
                "%s: Reboot attrs req received, flags: %x ..\n",
                __func__, resp_data.get_attrs.suspend_flag.val);

        break;

    case RPMI_SYSSUSP_SRV_SYSTEM_SUSPEND:
        suspend_type = le32toh(srv_data->suspend_type);
        qemu_log_mask(LOG_GUEST_ERROR,
                "%s: Sys Reset request received, type: %x\n",
                __func__, suspend_type);

        switch (suspend_type) {
        case RPMI_SYSSUSP_SHUTDOWN:
            cpu = cpu_by_arch_id(srv_data->hart_id);
            env = cpu ? &RISCV_CPU(cpu)->env : NULL;

            g_puc_suspend_data.sleep_state = RPMI_SYSSUSP_SHUTDOWN;
            g_puc_suspend_data.addr_l = le32toh(srv_data->resume_addr_lo);
            g_puc_suspend_data.addr_h = le32toh(srv_data->resume_addr_hi);

            qemu_log_mask(LOG_GUEST_ERROR,
                    "%s: Handling suspend service ID: %x\n",
                    __func__, svc_id);
            qemu_register_wakeup_support();

            riscv_set_wfi_cb(env,  execute_rpmi_suspend);

            resp_data.susp.status = 0;
            resp_dlen = sizeof(resp_data.susp);
            break;

        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                    "%s: Invalid suspend service Id..\n",
                    __func__);
            return -1;

        }
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                "%s: Unhandled sys suspend service ID: %x\n",
                __func__, svc_id);
    }

    rpmi_pack_message(RPMI_MSG_ACKNOWLDGEMENT,
                      RPMI_SRVGRP_SYSTEM_SUSPEND,
                      svc_id, GET_TOKEN(msg),
                      resp_dlen, &resp_data, tmsgbuf);

    rc = smq_enqueue(xport_id, qid, tmsgbuf);
    if (rc) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: smq_enqueue failed, rc: %x\n",
                      __func__, rc);
        return -1;
    }

    return 0;
}
