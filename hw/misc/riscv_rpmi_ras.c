 /*
  * riscv_rpmi_ras.c
  * RPMI RAS service group message handling routines.
  *
  * Copyright (c) 2023 Ventana Micro Systems, Inc.
  *
  * Authors:
  * Himanshu Chauhan <hchauhan@ventanamicro.com>
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
#include "hw/misc/riscv_rpmi.h"
#include "exec/address-spaces.h"
#include "hw/riscv/numa.h"
#include "riscv_rpmi_transport.h"
#include "hw/riscv/riscv_ras_agent.h"

target_ulong helper_csrr(CPURISCVState *env, int csr);

typedef union rpmi_ras_response_data_u {
    uint8_t raw[RPMI_MSG_DATA_SIZE];
    struct rpmi_ras_probe_resp probe;
    struct rpmi_ras_sync_err_resp pend_err_resp;
} rpmi_ras_resp_data_t;

enum {
    RPMI_RAS_DISABLED,
    RPMI_RAS_ENABLED
};

int handle_rpmi_grp_ras_agent(struct rpmi_message *msg, int xport_id)
{
    int svc_id = GET_SERVICE_ID(msg);
    int32_t rc;
    uint32_t qid = RPMI_QUEUE_IDX_P2A_ACK;
    uint32_t hart_id;
    uint8_t queue_data[RPMI_QUEUE_SLOT_SIZE];
    uint32_t *req_data = (uint32_t *)msg->data;
    struct rpmi_message *tmsgbuf = (struct rpmi_message *)queue_data;
    uint32_t resp_dlen = 0;

    rpmi_ras_resp_data_t resp_data;

    memset(tmsgbuf, 0, sizeof(queue_data));
    memset(&resp_data, 0, sizeof(resp_data));

    qemu_log_mask(LOG_GUEST_ERROR, "%s: RAS service ID: %x\n",
                __func__, svc_id);
    switch (svc_id) {
    case RPMI_RAS_SRV_PROBE_REQ:
        resp_data.probe.status = 0;
        resp_data.probe.version = ras_get_agent_version();
        qemu_log_mask(LOG_GUEST_ERROR, "%s: RAS agent version: %d\n",
                     __func__, resp_data.probe.version);
        resp_dlen = sizeof(resp_data.probe);
        break;

    case RPMI_RAS_SRV_SYNC_HART_ERR_REQ:
        hart_id = req_data[0];
        riscv_ras_agent_synchronize_hart_errors(hart_id, &resp_data.pend_err_resp);
        resp_dlen = sizeof(resp_data.pend_err_resp);
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Unhandled RAS service ID: %x\n",
                __func__, svc_id);
        return -1;
    }

    rpmi_pack_message(RPMI_MSG_ACKNOWLDGEMENT,
                      RPMI_SRVGRP_RAS_AGENT,
                      svc_id, GET_TOKEN(msg),
                      resp_dlen, &resp_data, tmsgbuf);

    rc = smq_enqueue(xport_id, qid, tmsgbuf);
    if (rc) {
            qemu_log_mask(LOG_GUEST_ERROR,
                         "%s: smq_enqueue failed, rc: %x\n",
                         __func__, rc);
        return -1;
    }

    qemu_log_mask(LOG_GUEST_ERROR, "%s: Success\n", __func__);

    return 0;
}
