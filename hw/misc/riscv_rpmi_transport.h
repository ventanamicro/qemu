 /*
  * riscv_rpmi_transport.h
  * header file for RPMI shared memory and transport queue handling routines.
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

#ifndef __TRANSPORT_H__
#define __TRANSPORT_H__


#include <stdint.h>
#include <stddef.h>
#include "hw/misc/riscv_rpmi.h"
#include "hw/misc/rpmi_msgprot.h"
/*
 *              +------------------------------------------+
 *              |                                          |
 *              |  +------------------------------------+  |
 *              |  |     A2P Channel - REQ Queue        |  |
 *    +-------->|  +------------------------------------+  |    get_message()
 *    |         |                                          +-------------+
 *    |         |  +------------------------------------+  |             |
 *    |         |  |     A2P Channel - ACK Queue        |  |             |
 *    |         |  +------------------------------------+  |             |
 *    |         |                                          |             |
 *    |         |       +-------------------------+        |             |
 *    |  +------+------>|        Interrupt        +--------+----------+  |
 *    |  |      |       +-------------------------+        |          |  |
 *    |  |      |                                          |          v  v
 * +--+--+--+   +------------------------------------------+       +--------+
 * |        |                                                      |        |
 * |  A.P   |   +------------------------------------------+       |  PuC   |
 * |        |   |                                          |       |        |
 * +--------+   |  +------------------------------------+  |       +--+--+--+
 *    ^  ^      |  |      P2A Channel - REQ Queue       |  |          |  |
 *    |  |      |  +------------------------------------+  |          |  |
 *    |  +------+                                          |<---------+  |
 *    |         |  +------------------------------------+  | send_message()
 *    |         |  |      P2A Channel - ACK Queue       |  |             |
 *    |         |  +------------------------------------+  |             |
 *    |         |                                          |             |
 *    |         |       +-------------------------+        |             |
 *    +---------+-------+         Interrupt       |<-------+-------------+
 *              |       +-------------------------+        | raise_interrupt()
 *              |                                          |
 *              +------------------------------------------+
 */
#define SIZE_WRITEIDX           4   /* bytes */
#define SIZE_READIDX            4   /* bytes */
#define RPMI_QUEUE_HEADER_SIZE      (SIZE_WRITEIDX + SIZE_READIDX)

#define RPMI_MESSAGE_HEADER_SIZE    16    /* bytes */
#define RPMI_MESSAGE_PAYLOAD_OFFSET 0x10  /* bytes */
#define RPMI_MESSAGE_PAYLOAD_MAX_SIZE   48 /* bytes */
#define RPMI_MESSAGE_SIZE       (RPMI_MESSAGE_HEADER_SIZE + \
                     RPMI_MESSAGE_PAYLOAD_MAX_SIZE)
 /*
  * Poll timer in microseconds to check if there is a new write on FCM,
  * its observed that, OpenSBI seems to be writing with gap of 2-3ms gap,
  * so keeping the poll time to 1ms
  */
#define FCM_CHECK_TIME 1000
#define RPMI_PER_HART_FCM_SIZE (4 * 16)
#define RPMI_FCM_SIZE (MAX_HARTS * RPMI_PER_HART_FCM_SIZE)

enum rpmi_queue_type {
    RPMI_QUEUE_TYPE_REQ = 0,
    RPMI_QUEUE_TYPE_ACK = 1,
};

enum rpmi_queue_idx {
    RPMI_QUEUE_IDX_A2P_REQ = 0,
    RPMI_QUEUE_IDX_P2A_ACK = 1,
    RPMI_QUEUE_IDX_P2A_REQ = 2,
    RPMI_QUEUE_IDX_A2P_ACK = 3,
    RPMI_QUEUE_IDX_MAX_COUNT,
};

enum rpmi_transport_type {
    RPMI_XPORT_TYPE_SOC = 0,
    RPMI_XPORT_TYPE_SOCKET = 1,
    RPMI_XPORT_TYPE_MAX_COUNT,
};

struct rpmi_transport_operations {
    /*
     * get_message() on PuC will be called for -
     * Request Messages sent from AP to PuC on A2P Channel-REQ Queue
     * A2P Channel-ACK Queue for Acknowldgements for Requests
     * sent to chiplet on P2A Channel-REQ Queue.
     * Both types of messages are received from AP in their
     * dedicated queues.
     */
    int (*get_message)(unsigned int queue_id, void *msgbuf);

    /*
     * send_message() on PuC always happen -
     * P2A channel-REQ Queue for Request Messages and
     * P2A channel-ACK Queue for Acknowldgements
     */
    int (*send_message)(unsigned int queue_id, void *msgbuf);
    int (*is_empty)(unsigned int queue_id);
};


struct rpmi_transport_ctx {
    int a2p_req_qid;
    int p2a_ack_qid;
    int p2a_req_qid;
    int a2p_ack_qid;
    struct rpmi_transport_operations *ops;
};

int rpmi_transport_init(struct rpmi_transport_ctx *trans_ctx);
void memcpy_endian(int32_t *dest, int32_t *src, int32_t len,
                bool to_le32);
void rpmi_init_transport(int xport_id, hwaddr shm_addr, hwaddr reg_addr,
                         hwaddr fcm_addr, int socket_num, uint64_t harts_mask);
void handle_rpmi_shm(int xport_id);
void handle_rpmi_fcm(int xport_id);
hwaddr rpmi_get_fcm_base(int xport_id);
int rpmi_get_svc_grps(int xport_id);
uint64_t rpmi_get_harts_mask(int xport_id);

int handle_rpmi_cppc_fcm(void *req_buf);
int handle_rpmi_msg(struct rpmi_message *msg, int xport_id);
void dump_rpmi_msg(unsigned int xport_id, struct rpmi_message *msg);
int smq_enqueue(unsigned int xport_id, unsigned int queue_id, void *data);
int smq_dequeue(unsigned int xport_id, unsigned int queue_id, void *data);

int handle_rpmi_grp_base(struct rpmi_message *msg, int xport_id);
int handle_rpmi_grp_cppc(struct rpmi_message *msg, int xport_id);
int handle_rpmi_grp_hsm(struct rpmi_message *msg, int xport_id);
bool execute_rpmi_hsm_stop(void *env);
int handle_rpmi_grp_sys_reset(struct rpmi_message *msg, int xport_id);
int handle_rpmi_grp_suspend(struct rpmi_message *msg, int xport_id);
bool execute_rpmi_suspend(void *env);

#endif

