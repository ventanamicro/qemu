 /*
  * riscv_rpmi_transport.c
  * RPMI sharedmemory and transport queue handling routines.
  *
  * Copyright (c) 2023
  *
  * Authors:
  * Rahul Pathak <rpathak@ventanamicro.com>
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
#include "exec/address-spaces.h"
#include "hw/qdev-core.h"
#include "hw/riscv/numa.h"
#include "riscv_rpmi_transport.h"

/* This shared memory is stored in  lttle endian format */
struct smq_queue {
    volatile int32_t head[RPMI_QUEUE_SLOT_SIZE / sizeof(int32_t)];
    volatile int32_t tail[RPMI_QUEUE_SLOT_SIZE / sizeof(int32_t)];
    /*
     * making this like an array makes it behave like a
     * array name which does not change when you try to
     * change and which does not move when you add or
     * subtract. It does not occupy memory also. Now this
     * is the alias to the buffer address which can now be
     * used as an array with proper typecasting.
     */
     volatile uint8_t buffer[RPMI_QUEUE_SIZE - 2*RPMI_QUEUE_SLOT_SIZE];
};

struct smq_queue_ctx {
    unsigned int queue_id;
    size_t slot_size;
    size_t num_slots;
    /* Type of queue - REQ or ACK */
    enum rpmi_queue_type queue_type;
    /* Is message available in queue */
    bool message_pending;
    /*
     * config of the queue, like memory base addresses,
     * which will come from configuration file statically
     * in the actual code. Keeping the pointer here because
     * we dont know what config attribute may have which
     * we are not initializing with but need later
     */
    struct smq_queue *queue;
};

/* Main structure for the complete shared memory queues data structure */
/* main structure for representing the group of queues and their attributes */
struct rpmi_xport_ctx {
    char name[32];
    hwaddr shm_base;
    hwaddr regs_base;
    hwaddr fcm_base;
    uint64_t harts_mask;
    uint64_t service_grp_mask;
    uint32_t queue_count;
    struct smq_queue_ctx queue_ctx_table[RPMI_QUEUE_IDX_MAX_COUNT];
    struct smq_queue queue_buf[RPMI_NUM_QUEUES];
};

#define RPMI_MAX_TRANSPORTS 16
static struct rpmi_xport_ctx rpmi_xports[RPMI_MAX_TRANSPORTS];
static int num_rpmi_xports;

hwaddr rpmi_get_fcm_base(int xport_id)
{
    struct rpmi_xport_ctx *xport_ctx = &rpmi_xports[xport_id];
    return xport_ctx->fcm_base;
}

int rpmi_get_svc_grps(int xport_id)
{
    struct rpmi_xport_ctx *xport_ctx = &rpmi_xports[xport_id];
    return xport_ctx->service_grp_mask;
}

uint64_t rpmi_get_harts_mask(int xport_id)
{
    struct rpmi_xport_ctx *xport_ctx = &rpmi_xports[xport_id];
    return xport_ctx->harts_mask;
}

void rpmi_init_transport(int xport_id, hwaddr shm_addr, hwaddr reg_addr,
                         hwaddr fcm_addr, int socket_num, uint64_t harts_mask)
{
    struct rpmi_xport_ctx *xport_ctx = &rpmi_xports[xport_id];
    int q;

    sprintf(xport_ctx->name, "rpmi.xport.%02d", xport_id);
    xport_ctx->shm_base = shm_addr;
    xport_ctx->regs_base = reg_addr;
    xport_ctx->fcm_base = fcm_addr;
    xport_ctx->queue_count = RPMI_NUM_QUEUES;

    for (q = 0; q < RPMI_NUM_QUEUES; q++) {
        xport_ctx->queue_ctx_table[q].queue_id = q;
        xport_ctx->queue_ctx_table[q].slot_size = RPMI_QUEUE_SLOT_SIZE;
        xport_ctx->queue_ctx_table[q].num_slots = RPMI_QUEUE_NUM_SLOTS;
        xport_ctx->queue_ctx_table[q].queue = &xport_ctx->queue_buf[q];

    }

    xport_ctx->harts_mask = harts_mask;
    xport_ctx->service_grp_mask = (1 << RPMI_SRVGRP_BASE);
    if (socket_num == -1) {
        /* initialize SOC transport */
        xport_ctx->service_grp_mask |= ((1 << RPMI_SRVGRP_SYSTEM_RESET) |
                                        (1 << RPMI_SRVGRP_SYSTEM_SUSPEND));
    }
    if (harts_mask) {
        /* initialize socket transport */
        xport_ctx->service_grp_mask |= ((1 << RPMI_SRVGRP_HSM) |
                                        (1 << RPMI_SRVGRP_CPPC));
    }

    num_rpmi_xports++;
}

static struct smq_queue_ctx *__get_queue_ctx(unsigned int xport_id,
                                             unsigned int queue_id)
{
    struct rpmi_xport_ctx *xport_ctx = &rpmi_xports[xport_id];

    if (queue_id > xport_ctx->queue_count) {
        return NULL;
    }

    return &xport_ctx->queue_ctx_table[queue_id];
}

static bool __smq_queue_full(struct smq_queue_ctx *qctx)
{
    return ((le32toh(qctx->queue->tail[0]) + 1) % (qctx->num_slots)
            == le32toh(qctx->queue->head[0])) ? true : false;
}

static bool __smq_queue_empty(struct smq_queue_ctx *qctx)
{
    return (le32toh(qctx->queue->head[0]) == le32toh(qctx->queue->tail[0])) ?
        true :  false;
}

static int __smq_dequeue(struct smq_queue_ctx *qctx, void *data)
{
    if (__smq_queue_empty(qctx)) {
        return RPMI_ERR_NOTFOUND;
    }

    memcpy(data, (char *)qctx->queue->buffer +
            (le32toh(qctx->queue->head[0]) * qctx->slot_size),
            qctx->slot_size);

    qctx->queue->head[0] = htole32((le32toh(qctx->queue->head[0]) + 1) %
                        qctx->num_slots);
    /*
     * TODO: Need to make arch generic function
     * which invokes arch specific barrier
     */
    smp_wmb();

    return RPMI_SUCCESS;
}

static int __smq_enqueue(struct smq_queue_ctx *qctx, void *data)
{
    if (__smq_queue_full(qctx)) {
        return RPMI_ERR_OUTOFRES;
    }
    memcpy((char *)qctx->queue->buffer +
            (le32toh(qctx->queue->tail[0]) * qctx->slot_size),
                        data, qctx->slot_size);

    qctx->queue->tail[0] = htole32((le32toh(qctx->queue->tail[0]) + 1) %
                        qctx->num_slots);
    /*
     * TODO: Need to make arch generic function
     * which invokes arch specific barrier
     */
    smp_wmb();

    return RPMI_SUCCESS;
}

__UNUSED__ static int smq_is_empty(unsigned int xport_id, unsigned int queue_id)
{
    struct smq_queue_ctx *qctx;

    qctx = __get_queue_ctx(xport_id, queue_id);
    if (!qctx) {
        return RPMI_ERR_NOTFOUND;
    }

    return RPMI_SUCCESS;
}

int smq_dequeue(unsigned int xport_id, unsigned int queue_id, void *data)
{
    int ret;
    struct smq_queue_ctx *qctx;
    struct rpmi_xport_ctx *xport_ctx = &rpmi_xports[xport_id];

    if ((queue_id > xport_ctx->queue_count) || !data) {
        printf("smq_dequeue(): queue_id or data invalid\n");
        return RPMI_ERR_INVAL;
    }

    qctx = __get_queue_ctx(xport_id, queue_id);
    if (!qctx) {
        printf("smq_dequeue(): error in getting the queue context\n");
        return RPMI_ERR_NOTFOUND;
    }

    ret = __smq_dequeue(qctx, data);
    if (ret) {
        return ret;
    }

    return RPMI_SUCCESS;
}

int smq_enqueue(unsigned int xport_id, unsigned int queue_id, void *data)
{
    int ret;
    struct smq_queue_ctx *qctx;
    struct rpmi_xport_ctx *xport_ctx = &rpmi_xports[xport_id];

    if ((queue_id > xport_ctx->queue_count) || !data) {
        qemu_log_mask(LOG_GUEST_ERROR,
                "smq_enqueue(): queue_id or data invalid, xport)id: %x, qid: %x(max: %d), data: %p\n",
                xport_id, queue_id, xport_ctx->queue_count, data);
        return RPMI_ERR_INVAL;
    }

    qctx = __get_queue_ctx(xport_id, queue_id);
    if (!qctx) {
        qemu_log_mask(LOG_GUEST_ERROR,
                "smq_enqueue(): error in getting the queue context\n");
        return RPMI_ERR_NOTFOUND;
    }

    ret = __smq_enqueue(qctx, data);
    if (ret) {
        qemu_log_mask(LOG_GUEST_ERROR,
                "smq_enqueue(): error in enqueue the data\n");
        return ret ;
    }

    return RPMI_SUCCESS;

}

void rpmi_pack_message(uint32_t type, uint32_t srvgrpid,
        uint32_t srvid, uint32_t token, uint32_t dlen, void *dbuf,
        struct rpmi_message *msgbuf)
{
    uint32_t l_msgidn, l_srvid, l_srvgrpid, l_type;
    /* Pack Message Identifier */
    l_srvgrpid = (srvgrpid << RPMI_MSG_IDN_SERVICEGROUP_ID_POS) &
        RPMI_MSG_IDN_SERVICEGROUP_ID_MASK;
    l_srvid = (srvid << RPMI_MSG_IDN_SERVICE_ID_POS) &
        RPMI_MSG_IDN_SERVICE_ID_MASK;
    l_type = (type << RPMI_MSG_IDN_TYPE_POS) & RPMI_MSG_IDN_TYPE_MASK;
    l_msgidn = l_srvgrpid | l_srvid | l_type;

    SET_MSGIDN(msgbuf, l_msgidn);
    SET_DATALEN(msgbuf, dlen);
    SET_TOKEN(msgbuf, token);

    if (dlen && dbuf) {
        memcpy((int32_t *)msgbuf->data, (int32_t *)dbuf, dlen);
    }

    return;
}

void dump_rpmi_msg(unsigned int xport_id, struct rpmi_message *msg)
{
    qemu_log_mask(LOG_GUEST_ERROR, "%s:  rpmi msg data:\n", __func__);
    qemu_log_mask(LOG_GUEST_ERROR,
            "%s: xport_id: %d, token: %d,  idn: %x, datalen: %x,"
            "type: %x, svc_id: %x, svc_grp_id: %x\n", __func__,
            xport_id,
            GET_TOKEN(msg),
            GET_MSGIDN(msg),
            GET_DATALEN(msg),
            GET_MESSAGE_TYPE(msg),
            GET_SERVICE_ID(msg),
            GET_SERVICEGROUP_ID(msg)
            );
}

int handle_rpmi_msg(struct rpmi_message *msg, int xport_id)
{
    int rc = 0;
    int svc_grp_id;

    svc_grp_id = GET_SERVICEGROUP_ID(msg);
    dump_rpmi_msg(xport_id, msg);
    qemu_log_mask(LOG_GUEST_ERROR, "%s:%d: svc_grp_id: %d, xport_id: %x\n",
            __func__, __LINE__, svc_grp_id, xport_id);

    switch (svc_grp_id) {
    case RPMI_SRVGRP_BASE:
        handle_rpmi_grp_base(msg, xport_id);
        break;

    case RPMI_SRVGRP_CPPC:
        handle_rpmi_grp_cppc(msg, xport_id);
        break;

    case RPMI_SRVGRP_HSM:
    case RPMI_SRVGRP_SYSTEM_RESET:
    case RPMI_SRVGRP_SYSTEM_SUSPEND:
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Unhandled service group id: %x\n",
                __func__, svc_grp_id);
    }

    return rc;
}

void handle_rpmi_shm(int xport_id)
{
    AddressSpace *as = &address_space_memory;
    MemTxResult result = MEMTX_OK;
    struct rpmi_xport_ctx *xport_ctx = &rpmi_xports[xport_id];
    hwaddr addr = (hwaddr)xport_ctx->shm_base;
    uint32_t qid;
    uint32_t *data = (uint32_t *)&xport_ctx->queue_buf;
    uint32_t data_sz = RPMI_QUEUE_SIZE * RPMI_NUM_QUEUES;
    uint8_t queue_data[RPMI_QUEUE_SLOT_SIZE];
    uint32_t rc;

    /*read complete RPMI shared memory into a local copy */
    result = address_space_rw(as, addr, MEMTXATTRS_UNSPECIFIED,
            (void *)data, data_sz, false);
    if (result != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR,
                "%s: Bad address %" HWADDR_PRIx
                " for mem read, rc: %x\n", __func__, addr, result);
        return;
    }

    /* fetch messages, from only a2p request the queue */
    for (qid = 0; qid < RPMI_QUEUE_IDX_P2A_ACK; qid++)   {
        struct rpmi_message *msg = (struct rpmi_message *)&queue_data;

        qemu_log_mask(LOG_GUEST_ERROR, "%s:  qid: %d\n", __func__, qid);

        /* fetch all the messages, from this queue */
        do {
            rc = smq_dequeue(xport_id, qid, queue_data);
            if (rc == RPMI_SUCCESS) {
                if (handle_rpmi_msg(msg, xport_id)) {
                    break;
                }
                memset(&queue_data, 0, sizeof(queue_data));
            }
        } while (rc == RPMI_SUCCESS);
    }

    /*
     * write complete RPMI shared memory from local copy,
     * to reflect the changes
     */
    result = address_space_rw(as, addr, MEMTXATTRS_UNSPECIFIED,
            (void *)data, data_sz, true);
    if (result != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR,
                "%s: Bad address %" HWADDR_PRIx
                " for mem write, rc: %x\n", __func__, addr, result);
        return;
    }
}

void handle_rpmi_fcm(int xport_id)
{
    AddressSpace *as = &address_space_memory;
    MemTxResult result = MEMTX_OK;
    struct rpmi_xport_ctx *xport_ctx = &rpmi_xports[xport_id];
    hwaddr addr = xport_ctx->fcm_base;
    uint8_t data[RPMI_PER_HART_FCM_SIZE * MAX_HARTS] ;
    uint32_t data_sz = RPMI_PER_HART_FCM_SIZE * MAX_HARTS ;
    int modified_harts;

    /*read memory into a local copy */
    result = address_space_rw(as, addr, MEMTXATTRS_UNSPECIFIED,
            (void *)&data, data_sz, false);
    if (result != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR,
                "%s: Bad address %" HWADDR_PRIx
                " for mem read, rc: %x\n", __func__, addr, result);
        return;
    }

    modified_harts = handle_rpmi_cppc_fcm(data);

    if (modified_harts) {
        /*write memory from a local copy, only if its modified */
        result = address_space_rw(as, addr, MEMTXATTRS_UNSPECIFIED,
                                  (void *)&data, data_sz, true);
        if (result != MEMTX_OK) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: Bad address %" HWADDR_PRIx
                          " for mem write, rc: %x\n", __func__, addr, result);
            return;
        }
    }

}
