 /*
  * riscv_rpmi_transport_reset.c
  * RPMI reset service group message handling routines.
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

typedef union rpmi_reset_req_data_u {
    uint32_t dw0;
} rpmi_reset_req_data_t;

typedef union rpmi_reset_response_data_u {
        uint8_t raw[RPMI_MSG_DATA_SIZE];
        struct {
                uint32_t status;
                uint32_t warm_res_flag;
        } get_attrs;

        struct {
                uint32_t status;
        } notify_enable;
} rpmi_reset_resp_data_t;

#define RPMI_RESET_NAME_MAX_LEN    32
#define RPMI_RESET_NAME_MAX_SUBS_PER_SVCGRP    32
#define RPMI_RESET_NAME_MAX_EVENTS_PER_SVCGRP    32

enum {
    RPMI_RESET_DISABLED,
    RPMI_RESET_ENABLED
};

/* global struct to store the notification subscribers */
struct grp_subscribers reset_subs[RPMI_SRVGRP_ID_MAX_COUNT] = {0};

/* global struct to store the events */
struct grp_events reset_events[RPMI_SRVGRP_ID_MAX_COUNT] = {0};

struct rpmi_srv_data_sysreset {
        uint32_t reset_type;
};

int handle_rpmi_grp_sys_reset(struct rpmi_message *msg, int xport_id)
{
        int svc_id = GET_SERVICE_ID(msg), event_id;
        uint32_t qid = RPMI_QUEUE_IDX_P2A_ACK, rc;
        uint8_t queue_data[RPMI_QUEUE_SLOT_SIZE];
        uint32_t *req_data = (uint32_t *)msg->data;
        struct rpmi_message *tmsgbuf = (struct rpmi_message *)queue_data;
        struct rpmi_srv_data_sysreset *srv_data =
                (struct rpmi_srv_data_sysreset *)msg->data;
        uint32_t reset_type;
        uint32_t resp_dlen = 0;
        rpmi_reset_resp_data_t resp_data;

        memset(tmsgbuf, 0, sizeof(queue_data));
        memset(&resp_data, 0, sizeof(resp_data));

        switch (svc_id) {

        case RPMI_SYSRST_SRV_ENABLE_NOTIFICATION:
            event_id = le32toh(req_data[0]);
            reset_subs[event_id].state =  RPMI_RESET_ENABLED;
            reset_subs[event_id].subscribers_cnt++;
            resp_data.notify_enable.status = 0;
            resp_dlen = sizeof(resp_data.notify_enable);
            break;

        case RPMI_SYSRST_SRV_GET_SYSTEM_RESET_ATTRIBUTES:
            resp_data.get_attrs.status = 0;
            resp_data.get_attrs.warm_res_flag = 1 << 31;
            resp_dlen = sizeof(resp_data.get_attrs);
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: Reboot attrs req received, flags: %x ..\n",
                          __func__, resp_data.get_attrs.warm_res_flag);
            rpmi_pack_message(RPMI_MSG_ACKNOWLDGEMENT,
                              RPMI_SRVGRP_SYSTEM_RESET,
                              svc_id, GET_TOKEN(msg),
                              resp_dlen, &resp_data, tmsgbuf);

            rc = smq_enqueue(xport_id, qid, tmsgbuf);
            if (rc) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "%s: smq_enqueue failed, rc: %x\n",
                              __func__, rc);
                return -1;
            }

            break;

        case RPMI_SYSRST_SRV_SYSTEM_RESET:
                reset_type = le32toh(srv_data->reset_type);
                qemu_log_mask(LOG_GUEST_ERROR,
                                "%s: Sys Reset request received, type: %x\n",
                                __func__, reset_type);
                if (reset_type == RPMI_SYSRST_WARM_RESET ||
                                reset_type == RPMI_SYSRST_COLD_RESET) {
                        qemu_log_mask(LOG_GUEST_ERROR,
                                        "%s: rebooting..\n",  __func__);
                        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
                } else if (reset_type == RPMI_SYSRST_SHUTDOWN) {
                        qemu_log_mask(LOG_GUEST_ERROR,
                                        "%s: Shutting down..\n",
                                        __func__);
                        exit(0);

                } else {
                        qemu_log_mask(LOG_GUEST_ERROR,
                                        "%s: Invalid reset service Id..\n",
                                        __func__);
                        return -1;
                }
                break;

        default:
                qemu_log_mask(LOG_GUEST_ERROR,
                                "%s: Unhandled sys reset service ID: %x\n",
                                __func__, svc_id);
        }

        return 0;
}
