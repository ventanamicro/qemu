 /*
  * riscv_rpmi_transport_base.c
  * RPMI base service group message handling routines.
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

#define RPMI_BASE_NAME_MAX_LEN    32
#define RPMI_BASE_NAME_MAX_SUBS_PER_SVCGRP    32
#define RPMI_BASE_NAME_MAX_EVENTS_PER_SVCGRP    32

enum {
    RPMI_BASE_DISABLED,
    RPMI_BASE_ENABLED
};

/*FIXME: update the real versions */
#define VMS_BASE_IMPL_MAJOR  1
#define VMS_BASE_IMPL_MINOR  1
#define VMS_BASE_SPEC_MINOR  1
#define VMS_BASE_SPEC_MAJOR  1
#define VMS_BASE_IMPL_ID  1
#define VMS_BASE_VENDOR_ID  1

/** union of possible responses for base service group's RPMI messages */
typedef union rpmi_base_response_data_u {
    uint8_t raw[RPMI_MSG_DATA_SIZE];

    struct {
        uint32_t status;
        uint32_t minor:16;
        uint32_t major:16;
    } get_impl_ver;

    struct {
        uint32_t status;
        uint32_t id;
    } get_impl_id;

    struct {
        uint32_t status;
        uint32_t ready;
    } probe_srv_grp;

    struct {
        uint32_t status;
        uint32_t minor:16;
        uint32_t major:16;
    } get_spec_ver;

    struct {
        uint32_t status;
    } set_msi;

    struct {
        uint32_t status;
    } notify_enable;

    struct rpmi_base_get_attributes_resp attrs;

    struct {
        uint32_t status;
        uint32_t vendor_id;
        uint32_t hw_id_len;
        uint32_t hw_id[4];
    } hw_info;

} rpmi_base_resp_data_t;

/* global struct to store the MSI data intended to be consumed by PuC FW */
struct msi_data_s {
    uint32_t addr_l;
    uint32_t addr_h;
    uint32_t data;
} g_puc_msi_data = {0};

/* global struct to store the notification subscribers */
struct grp_subscribers base_subs[RPMI_SRVGRP_ID_MAX_COUNT] = {0};

/* global struct to store the events */
struct grp_events base_events[RPMI_SRVGRP_ID_MAX_COUNT] = {0};

/** Global variable to store the state of PuC FW, if its ready */
bool g_puc_ready = RPMI_BASE_ENABLED;

int handle_rpmi_grp_base(struct rpmi_message *msg, int xport_id)
{
    int svc_id = GET_SERVICE_ID(msg);
    bool rdy_status;
    uint32_t qid = RPMI_QUEUE_IDX_P2A_ACK, rc;
    uint32_t *req_data = (uint32_t *)msg->data;
    uint32_t servicegroup_id = 0;
    uint32_t event_id = 0;
    uint8_t queue_data[RPMI_QUEUE_SLOT_SIZE];
    struct rpmi_message *tmsgbuf = (struct rpmi_message *)queue_data;
    uint32_t resp_dlen = 0;
    rpmi_base_resp_data_t resp_data;

    memset(tmsgbuf, 0, sizeof(queue_data));
    memset(&resp_data, 0, sizeof(resp_data));

    switch (svc_id) {

    case RPMI_BASE_SRV_ENABLE_NOTIFICATION:
        event_id = le32toh(req_data[0]);
        base_subs[event_id].state =  RPMI_BASE_ENABLED;
        base_subs[event_id].subscribers_cnt++;
        resp_data.notify_enable.status = 0;
        resp_dlen = sizeof(resp_data.notify_enable);
        break;

    case RPMI_BASE_SRV_GET_IMPLEMENTATION_VERSION:
        resp_data.get_impl_ver.status = 0;
        resp_data.get_impl_ver.minor = VMS_BASE_IMPL_MINOR;
        resp_data.get_impl_ver.major = VMS_BASE_IMPL_MAJOR;
        resp_dlen = sizeof(resp_data.get_impl_ver);
        break;

    case RPMI_BASE_SRV_GET_IMPLEMENTATION_IDN:
        resp_data.get_impl_id.status = 0;
        resp_data.get_impl_id.id = VMS_BASE_IMPL_ID;
        resp_dlen = sizeof(resp_data.get_impl_id);
        break;

    case RPMI_BASE_SRV_GET_SPEC_VERSION:
        resp_data.get_spec_ver.status = 0;
        resp_data.get_spec_ver.minor = VMS_BASE_SPEC_MINOR;
        resp_data.get_spec_ver.major = VMS_BASE_SPEC_MAJOR;
        resp_dlen = sizeof(resp_data.get_spec_ver);
        break;

    case RPMI_BASE_SRV_GET_HW_INFO:
        resp_data.hw_info.vendor_id = VMS_BASE_VENDOR_ID;
        resp_data.hw_info.hw_id_len = 4;
        resp_data.hw_info.hw_id[0] = VMS_BASE_IMPL_ID;
        resp_data.hw_info.status = 0;
        resp_dlen = sizeof(resp_data.hw_info);
        break;

    case RPMI_BASE_SRV_PROBE_SERVICE_GROUP:
        servicegroup_id = le32toh(req_data[0]);
        rdy_status = (rpmi_get_svc_grps(xport_id) &
                (1 << servicegroup_id)) ? 1 : 0;
        resp_data.probe_srv_grp.status = 0;
        resp_data.probe_srv_grp.ready = rdy_status;
        resp_dlen = sizeof(resp_data.probe_srv_grp);
        break;

    case RPMI_BASE_SRV_GET_ATTRIBUTES:
        resp_data.attrs.f0 = RPMI_BASE_FLAGS_F0_MSI_EN |
            RPMI_BASE_FLAGS_F0_EV_NOTIFY;
        resp_dlen = sizeof(resp_data.attrs);
        break;

    case RPMI_BASE_SRV_SET_MSI:
        g_puc_msi_data.addr_h = le32toh(req_data[0]);
        g_puc_msi_data.addr_l = le32toh(req_data[1]);
        g_puc_msi_data.data = le32toh(req_data[2]);
        resp_data.set_msi.status = 0;
        resp_dlen = sizeof(resp_data.set_msi);
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Unhandled base service ID: %x\n",
                __func__, svc_id);
        return -1;
    }

    rpmi_pack_message(RPMI_MSG_ACKNOWLDGEMENT,
            RPMI_SRVGRP_BASE,
            svc_id, GET_TOKEN(msg),
            resp_dlen, &resp_data, tmsgbuf);

    rc = smq_enqueue(xport_id, qid, tmsgbuf);
    if (rc) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: smq_enqueue failed, rc: %x\n",
                __func__, rc);
        return -1;
    }

    return 0;
}
