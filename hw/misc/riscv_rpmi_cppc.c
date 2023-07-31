 /*
  * vms_rpmi_cppc.c
  * RPMI CPPC service group message handling routines.
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
#include "hw/misc/riscv_rpmi.h"
#include "exec/address-spaces.h"
#include "hw/riscv/numa.h"
#include "riscv_rpmi_transport.h"

target_ulong helper_csrr(CPURISCVState *env, int csr);

enum sbi_cppc_reg_id {
    SBI_CPPC_HIGHEST_PERF           = 0x00000000,
    SBI_CPPC_NOMINAL_PERF           = 0x00000001,
    SBI_CPPC_LOW_NON_LINEAR_PERF    = 0x00000002,
    SBI_CPPC_LOWEST_PERF            = 0x00000003,
    SBI_CPPC_GUARANTEED_PERF        = 0x00000004,
    SBI_CPPC_DESIRED_PERF           = 0x00000005,
    SBI_CPPC_MIN_PERF               = 0x00000006,
    SBI_CPPC_MAX_PERF               = 0x00000007,
    SBI_CPPC_PERF_REDUC_TOLERANCE   = 0x00000008,
    SBI_CPPC_TIME_WINDOW            = 0x00000009,
    SBI_CPPC_CTR_WRAP_TIME          = 0x0000000A,
    SBI_CPPC_REFERENCE_CTR          = 0x0000000B,
    SBI_CPPC_DELIVERED_CTR          = 0x0000000C,
    SBI_CPPC_PERF_LIMITED           = 0x0000000D,
    SBI_CPPC_ENABLE                 = 0x0000000E,
    SBI_CPPC_AUTO_SEL_ENABLE        = 0x0000000F,
    SBI_CPPC_AUTO_ACT_WINDOW        = 0x00000010,
    SBI_CPPC_ENERGY_PERF_PREFERENCE = 0x00000011,
    SBI_CPPC_REFERENCE_PERF         = 0x00000012,
    SBI_CPPC_LOWEST_FREQ            = 0x00000013,
    SBI_CPPC_NOMINAL_FREQ           = 0x00000014,
    SBI_CPPC_ACPI_LAST              = SBI_CPPC_NOMINAL_FREQ,
    SBI_CPPC_TRANSITION_LATENCY     = 0x80000000,
    SBI_CPPC_NON_ACPI_LAST          = SBI_CPPC_TRANSITION_LATENCY,
};

#define SBI_CPPC_ACPI_NUM_REGS (SBI_CPPC_ACPI_LAST - SBI_CPPC_HIGHEST_PERF + 1)
#define SBI_CPPC_NON_ACPI_NUM_REGS (SBI_CPPC_NON_ACPI_LAST - \
                                    SBI_CPPC_TRANSITION_LATENCY + 1)

#define LAST_REG_IDX  (SBI_CPPC_ACPI_NUM_REGS + SBI_CPPC_NON_ACPI_NUM_REGS)
/* for now we support fixed 32bit registers */
uint32_t cppc_reg_lens[LAST_REG_IDX] = {[0 ... LAST_REG_IDX - 1] = 4};
uint64_t cppc_regs_mem[MAX_HARTS][LAST_REG_IDX];
uint64_t cppc_fcm_last_val[MAX_HARTS];

struct fcm_request {
        uint32_t req_data;
        uint8_t pad[RPMI_PER_HART_FCM_SIZE - sizeof(uint32_t)];
};

typedef union rpmi_cppc_response_data_u {
    uint8_t raw[RPMI_MSG_DATA_SIZE];
    struct rpmi_cppc_probe_resp probe;
    struct rpmi_cppc_read_reg_resp read_reg;
    struct rpmi_cppc_write_reg_resp write_reg;
    struct rpmi_cppc_get_fast_channel_addr_resp chan_addr;

    struct {
        uint32_t status;
    } notify_enable;

    struct {
        uint32_t status;
        uint32_t remaining;
        uint32_t returned;
        /* remaining space need to be adjusted for the above 3 uint32_t's */
        uint32_t hartid[(RPMI_MSG_DATA_SIZE -
                         (sizeof(uint32_t) * 3)) / sizeof(uint32_t)];
    } hart_list;

    struct {
        uint32_t status;
    } poke;
} rpmi_cppc_resp_data_t;

#define RPMI_CPPC_NAME_MAX_LEN    32
#define RPMI_CPPC_NAME_MAX_SUBS_PER_SVCGRP    32
#define RPMI_CPPC_NAME_MAX_EVENTS_PER_SVCGRP    32

enum {
    RPMI_CPPC_DISABLED,
    RPMI_CPPC_ENABLED
};

/* global struct to store the notification subscribers */
struct grp_subscribers cppc_subs[RPMI_SRVGRP_ID_MAX_COUNT] = {0};

/* global struct to store the events */
struct grp_events cppc_events[RPMI_SRVGRP_ID_MAX_COUNT] = {0};

/* convert register sparse identifier to register index */
static uint32_t cppc_get_reg_index(uint32_t reg_id)
{
    uint32_t idx = -1;

    if (reg_id <= SBI_CPPC_ACPI_LAST) {
        idx = reg_id;
    } else if (reg_id == SBI_CPPC_TRANSITION_LATENCY) {
        idx = LAST_REG_IDX - 1;
    }

    return idx;
}

static void rw_cppc_reg(uint32_t hart_id, int32_t reg_index, uint64_t *data,
                           bool write)
{
    uint64_t *offset;

    offset = &cppc_regs_mem[hart_id][reg_index];
    if (!write) {
        *data = *offset;
    } else {
        *offset = *data;
    }
}

int handle_rpmi_grp_cppc(struct rpmi_message *msg, int xport_id)
{
    int svc_id = GET_SERVICE_ID(msg);
    int32_t rc, reg_index;
    uint32_t event_id = 0;
    uint32_t qid = RPMI_QUEUE_IDX_P2A_ACK;
    uint32_t hart_id, reg_id = 0;
    uint8_t queue_data[RPMI_QUEUE_SLOT_SIZE];
    uint32_t *req_data = (uint32_t *)msg->data;
    struct rpmi_message *tmsgbuf = (struct rpmi_message *)queue_data;
    uint32_t resp_dlen = 0, data_lo, data_hi;
    uint64_t data = 0;
    uint64_t chan_addr;
    CPUState *cpu;
    uint64_t harts_mask;
    uint32_t skip_count;
    int i, returned, remaining;

    rpmi_cppc_resp_data_t resp_data;

    memset(tmsgbuf, 0, sizeof(queue_data));
    memset(&resp_data, 0, sizeof(resp_data));

    qemu_log_mask(LOG_GUEST_ERROR, "%s: CPPC service ID: %x\n",
                __func__, svc_id);
    switch (svc_id) {
    case RPMI_CPPC_SRV_ENABLE_NOTIFICATION:
        event_id = le32toh(req_data[0]);
        cppc_subs[event_id].state =  RPMI_CPPC_ENABLED;
        cppc_subs[event_id].subscribers_cnt++;
        resp_data.notify_enable.status = 0;
        resp_dlen = sizeof(resp_data.notify_enable);
        break;

    case RPMI_CPPC_SRV_PROBE_REG:
        hart_id = req_data[0];
        reg_id = req_data[1];

        reg_index = cppc_get_reg_index(reg_id);
        if (reg_index < 0) {
            resp_data.probe.status = -1;
        } else {
            resp_data.probe.reg_len = cppc_reg_lens[reg_index];
        }
        resp_dlen = sizeof(resp_data.probe);
        break;

    case RPMI_CPPC_SRV_READ_REG:
        hart_id = req_data[0];
        reg_id = req_data[1];
        cpu = cpu_by_arch_id(hart_id);

        reg_index = cppc_get_reg_index(reg_id);
        if (reg_index < 0) {
            resp_data.read_reg.status = -1;
        } else {
            CPURISCVState *env = &RISCV_CPU(cpu)->env;
            uint64_t mcycle = 50;
            uint64_t desired_perf;

            mcycle = helper_csrr(env, CSR_TIME);
            if (SBI_CPPC_REFERENCE_CTR == reg_id) {
                data = mcycle;
            } else if (SBI_CPPC_DELIVERED_CTR == reg_id) {
                rw_cppc_reg(cpu->cpu_index, SBI_CPPC_DESIRED_PERF,
                            &desired_perf, false);
                /*FIXME: why this (desired_perf + 50) ??*/
                data = mcycle * (desired_perf + 50);
            } else {
                rw_cppc_reg(cpu->cpu_index, reg_index, &data, false);
            }
            resp_data.read_reg.data_lo = (uint32_t)data;
            resp_data.read_reg.data_hi = (uint32_t)(data >> 32);
        }
        resp_dlen = sizeof(resp_data.read_reg);
        break;

    case RPMI_CPPC_SRV_WRITE_REG:
        hart_id = req_data[0];
        reg_id = req_data[1];
        data_lo = req_data[2];
        data_hi = req_data[3];
        cpu = cpu_by_arch_id(hart_id);
        data = ((uint64_t)data_hi << 32) | data_lo;

        reg_index = cppc_get_reg_index(reg_id);
        if (reg_index < 0) {
            resp_data.write_reg.status = -1;
        } else {
            rw_cppc_reg(cpu->cpu_index, reg_index, &data, true);
        }

        resp_dlen = sizeof(resp_data.write_reg);
        break;

    case RPMI_CPPC_SRV_GET_FAST_CHANNEL_ADDR:
        hart_id = req_data[0];
        cpu = cpu_by_arch_id(hart_id);

        chan_addr = (uint64_t)rpmi_get_fcm_base(xport_id) +
            cpu->cpu_index * RPMI_PER_HART_FCM_SIZE;
        resp_data.chan_addr.status = 0;
        resp_data.chan_addr.addr_lo = (uint32_t)chan_addr;
        resp_data.chan_addr.addr_hi = (uint32_t)(chan_addr >> 32);
        resp_dlen = sizeof(resp_data.chan_addr);
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: RPMI_CPPC_SRV_GET_FAST_CHAN_ADDR: hart_id: %x, addr: %x\n",
                      __func__, hart_id, resp_data.chan_addr.addr_lo);
        break;

    case RPMI_CPPC_SRV_POKE_FAST_CHANNEL:
        /*
         * TODO: go through list of fast channel requests and process, if not
         * already processed
         */
        resp_data.poke.status = 0;
        resp_dlen = sizeof(resp_data.poke);
        break;

    case RPMI_CPPC_SRV_GET_HART_LIST:
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

    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Unhandled cppc service ID: %x\n",
                __func__, svc_id);
        return -1;
    }

    rpmi_pack_message(RPMI_MSG_ACKNOWLDGEMENT,
            RPMI_SRVGRP_CPPC,
            svc_id, GET_TOKEN(msg),
            resp_dlen, &resp_data, tmsgbuf);

    rc = smq_enqueue(xport_id, qid, tmsgbuf);
    if (rc) {
        return -1;
    }

    return 0;
}

int handle_rpmi_cppc_fcm(void *req_buf)
{
    struct fcm_request *req;
    uint32_t hart_id;
    uint64_t data;
    CPUState *cpu;
    int modified_harts = 0;

    for (hart_id = 0; hart_id < MAX_HARTS; hart_id++) {

        req = (struct fcm_request *)req_buf;
        data = req->req_data;
        cpu = cpu_by_arch_id(hart_id);

        /* done scanning all active CPUs */
        if (!cpu) {
            return modified_harts;
        }

        /* return if its not a new request */
        if (cppc_fcm_last_val[cpu->cpu_index] == req->req_data) {
            continue;
        } else {
            cppc_fcm_last_val[cpu->cpu_index] = req->req_data;
        }

        qemu_log_mask(LOG_GUEST_ERROR, "%s:%d fcm cpu#%d, data: %lx\n",
                      __func__, __LINE__,
                      cpu->cpu_index, data);

        /* propogate the request data to the desired perf register */
        rw_cppc_reg(cpu->cpu_index, SBI_CPPC_DESIRED_PERF, &data, true);

        modified_harts++;

        req_buf += RPMI_PER_HART_FCM_SIZE;
    }

    return modified_harts;
}
