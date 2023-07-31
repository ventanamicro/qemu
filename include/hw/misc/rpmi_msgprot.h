 /*
  * rpmi_msgprot.h
  * RPMI message related header file
  *
  *
  * Copyright (c) 2023
  *
  * Authors:
  * Rahul Pathak <rpathak@ventanamicro.com>
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


#ifndef __RPMI_MSGPROT_H__
#define __RPMI_MSGPROT_H__

/*
 * 31                                            0
 * +---------------------------------------------+
 * |                    TOKEN                    |
 * +---------+------------------+----------------+
 * |   FLAGS |  SERVICEGROUP_ID |    SERVICE_ID  |
 * +---------+------------------+----------------+
 * |                 DATA LENGTH                 |
 * +---------------------------------------------+
 * |                 DATA/PAYLOAD                |
 * +---------------------------------------------+
 */

/** Message Header Offset */
#define RPMI_MSG_HDR_OFFSET                     (0x0)
#define RPMI_MSG_HDR_SIZE                       (12)    /* bytes */

/** Token is unique message identifier in the system */
#define RPMI_MSG_TOKEN_OFFSET                   (0x0)
#define RPMI_MSG_TOKEN_SIZE                     (4)     /* bytes */

/** Message Identity = Flags + Service Group ID + Service ID */
#define RPMI_MSG_IDN_OFFSET                     (0x4)
#define RPMI_MSG_IDN_SIZE                       (4)     /* bytes */

#define RPMI_MSG_IDN_SERVICE_ID_POS             (0U)
#define RPMI_MSG_IDN_SERVICE_ID_MASK            \
        ((0xFF) << RPMI_MSG_IDN_SERVICE_ID_POS)

#define RPMI_MSG_IDN_SERVICEGROUP_ID_POS        (8U)
#define RPMI_MSG_IDN_SERVICEGROUP_ID_MASK       \
        ((0xFFFFF) << RPMI_MSG_IDN_SERVICEGROUP_ID_POS)

#define RPMI_MSG_IDN_TYPE_POS                   (28U)
#define RPMI_MSG_IDN_TYPE_MASK                  \
        ((0x3) << RPMI_MSG_IDN_TYPE_POS)

#define RPMI_MSG_IDN_DOORBELL_POS               (30U)
#define RPMI_MSG_IDN_DOORBELL_MASK              \
        ((0x1) << RPMI_MSG_IDN_DOORBELL_POS)

/** Data length field */
#define RPMI_MSG_DATALEN_OFFSET                 (0x8)
#define RPMI_MSG_DATALEN_SIZE                   (4)     /* bytes */

/** Data field */
#define RPMI_MSG_DATA_OFFSET                    (0xc)
#define RPMI_MSG_DATA_SIZE                      (52)    /* bytes */

/** Minimum message size Header + Data */
#define RPMI_MSG_SIZE_MIN                       (RPMI_MSG_HDR_SIZE + \
                                                 RPMI_MSG_DATA_SIZE)

/** Name length of 16 characters */
#define RPMI_NAME_CHARS_MAX                     (16)

#define le32_t  int32_t
#define u64     uint64_t
#define u32     uint32_t
#define u16     uint16_t
#define u8      uint8_t

#define leu32_t  uint32_t
#define s64     int64_t
#define s32     int32_t
#define s16     int16_t
#define s8      int8_t


/** RPMI Message Header */
struct rpmi_message_header {
        le32_t token;
        le32_t msgidn;
        le32_t datalen;
};

/** RPMI Message */
struct rpmi_message {
        struct rpmi_message_header header;
        u8 data[0];
};


/** RPMI Messages Types */
enum rpmi_message_type {
        /* Normal request backed with ack */
        RPMI_MSG_NORMAL_REQUEST = 0x0,
        /* Request without any ack */
        RPMI_MSG_POSTED_REQUEST = 0x1,
        /* Acknowledgment for normal request message */
        RPMI_MSG_ACKNOWLDGEMENT = 0x2,
        /* Notification message */
        RPMI_MSG_NOTIFICATION = 0x3,
};

/** RPMI Error Types */
enum rpmi_error {
     /* Success */
    RPMI_SUCCESS        = 0,
     /* General fail */
    RPMI_ERR_FAILED     = -1,
    /* Service/feature not supported */
    RPMI_ERR_NOTSUPP    = -2,
    /* Invalid Parameter  */
    RPMI_ERR_INVAL      = -3,
    /* Insufficier permissions  */
    RPMI_ERR_DENIED     = -4,
    /* Requested resource not found */
    RPMI_ERR_NOTFOUND   = -5,
    /* Requested resource out of range */
    RPMI_ERR_OUTOFRANGE = -6,
    /* Resource limit reached */
    RPMI_ERR_OUTOFRES   = -7,
    /* Operation failed due to hardware issues  */
    RPMI_ERR_HWFAULT    = -8,
    /* System currently busy, retry later */
    RPMI_ERR_BUSY = -9,
    /* Operation timed out*/
    RPMI_ERR_TIMEOUT = -10,
    /* Error in communication, retry later */
    RPMI_ERR_COMMS = -11,
    /*
     * Operation failed as it was already in progress or the state has changed
     * already for which the operation was carried out.
     */
    RPMI_ERR_ALREADY = -12,
    /* Error in implementsation which violates the specification version */
    RPMI_ERR_IMPL = -13,
    RPMI_ERR_RESERVED_START = -14,
    RPMI_ERR_RESERVED_END = -127,
    RPMI_ERR_VENDOR_START = -128,
};

/** RPMI Message Arguments */
struct rpmi_message_args {
        u32 flags;
#define RPMI_MSG_FLAGS_NO_TX            (1U << 0)
#define RPMI_MSG_FLAGS_NO_RX            (1U << 1)
#define RPMI_MSG_FLAGS_NO_RX_TOKEN      (1U << 2)
        enum rpmi_message_type type;
        u32 service_id;
        u32 tx_endian_words;
        u32 rx_endian_words;
        u32 rx_token;
};
/*
 *      RPMI SERVICEGROUPS AND SERVICES
 */

/** RPMI ServiceGroups IDs */
enum rpmi_servicegroup_id {
        RPMI_SRVGRP_ID_MIN = 0,
        RPMI_SRVGRP_BASE = 0x00001,
        RPMI_SRVGRP_SYSTEM_RESET = 0x00002,
        RPMI_SRVGRP_SYSTEM_SUSPEND = 0x00003,
        RPMI_SRVGRP_HSM = 0x00004,
        RPMI_SRVGRP_CPPC = 0x00005,
        RPMI_SRVGRP_ID_MAX_COUNT,
};

/** RPMI enable notification request */
struct rpmi_enable_notification_req {
        u32 eventid;
};

#define RPMI_EVT_NAME_MAX_LEN    32
#define RPMI_EVT_MAX_SUBS_PER_SVCGRP    32
#define RPMI_EVT_MAX_EVENTS_PER_SVCGRP    32

/** RPMI enable notification response */
struct rpmi_enable_notification_resp {
        s32 status;
};
/* global struct to store the notification subscribers */
struct grp_subscribers {
    uint32_t state;
    uint32_t subscribers_cnt;
    uint32_t sub_entity_list[RPMI_EVT_MAX_SUBS_PER_SVCGRP];
};

/* global struct to store the events */
struct grp_events {
    uint32_t state;
    uint32_t events_cnt;
    struct {
        uint64_t timestamp;
        uint64_t data;
    } event_data[RPMI_EVT_MAX_EVENTS_PER_SVCGRP];
};

/** RPMI Base ServiceGroup Service IDs */
enum rpmi_base_service_id {
        RPMI_BASE_SRV_ENABLE_NOTIFICATION = 0x01,
        RPMI_BASE_SRV_GET_IMPLEMENTATION_VERSION = 0x02,
        RPMI_BASE_SRV_GET_IMPLEMENTATION_IDN = 0x03,
        RPMI_BASE_SRV_GET_SPEC_VERSION = 0x04,
        RPMI_BASE_SRV_GET_HW_INFO = 0x05,
        RPMI_BASE_SRV_PROBE_SERVICE_GROUP = 0x06,
        RPMI_BASE_SRV_GET_ATTRIBUTES = 0x07,
        RPMI_BASE_SRV_SET_MSI = 0x08,
};

struct rpmi_base_get_attributes_resp {
        s32 status_code;
#define RPMI_BASE_FLAGS_F0_EV_NOTIFY            (1U << 31)
#define RPMI_BASE_FLAGS_F0_MSI_EN                       (1U << 30)
        u32 f0;
        u32 f1;
        u32 f2;
        u32 f3;
};

/** RPMI System Reset ServiceGroup Service IDs */
enum rpmi_system_reset_service_id {
        RPMI_SYSRST_SRV_ENABLE_NOTIFICATION = 0x01,
        RPMI_SYSRST_SRV_GET_SYSTEM_RESET_ATTRIBUTES = 0x02,
        RPMI_SYSRST_SRV_SYSTEM_RESET = 0x03,
        RPMI_SYSRST_SRV_ID_MAX_COUNT,
};

/** RPMI System Reset types */
enum rpmi_sysrst_reset_type {
        RPMI_SYSRST_SHUTDOWN = 0,
        RPMI_SYSRST_COLD_RESET = 1,
        RPMI_SYSRST_WARM_RESET = 2,
        RPMI_SYSRST_MAX_IDN_COUNT,
};

/** Response for system reset attributes */
struct rpmi_sysrst_get_reset_attributes_resp {
        s32 status;
#define RPMI_SYSRST_FLAGS_SUPPORTED_POS         (31)
#define RPMI_SYSRST_FLAGS_SUPPORTED_MASK                \
                        (1U << RPMI_SYSRST_FLAGS_SUPPORTED_POS)
        u32 flags;
};

/** RPMI System Suspend ServiceGroup Service IDs */
enum rpmi_system_suspend_service_id {
        RPMI_SYSSUSP_SRV_ENABLE_NOTIFICATION = 0x01,
        RPMI_SYSSUSP_SRV_GET_SYSTEM_SUSPEND_ATTRIBUTES = 0x02,
        RPMI_SYSSUSP_SRV_SYSTEM_SUSPEND = 0x03,
        RPMI_SYSSUSP_SRV_ID_MAX_COUNT,
};

/** Response for system suspend attributes */
struct rpmi_syssusp_get_attr_resp {
        s32 status;
#define RPMI_SYSSUSP_FLAGS_CUSTOM_RESUME_ADDR_SUPPORTED (1U << 31)
#define RPMI_SYSSUSP_FLAGS_SUPPORTED                    (1U << 30)
        u32 flags;
};

struct rpmi_syssusp_req {
        u32 type;
        u32 resume_addr_lo;
        u32 resume_addr_hi;
};

struct rpmi_syssusp_resp {
        s32 status;
};

/** RPMI HSM State Management ServiceGroup Service IDs */
enum rpmi_cpu_hsm_service_id {
        RPMI_HSM_SRV_ENABLE_NOTIFICATION = 0x01,
        RPMI_HSM_SRV_HART_START = 0x02,
        RPMI_HSM_SRV_HART_STOP = 0x03,
        RPMI_HSM_SRV_HART_SUSPEND = 0x04,
        RPMI_HSM_SRV_GET_HART_STATUS = 0x05,
        RPMI_HSM_SRV_GET_HART_LIST = 0x06,
        RPMI_HSM_SRV_GET_SUSPEND_TYPES = 0x07,
        RPMI_HSM_SRV_GET_SUSPEND_INFO = 0x08,
        RPMI_HSM_SRV_ID_MAX_COUNT,
};

/* HSM service group request and response structs */

enum {
    RPMI_HSM_STATE_STARTED         = 0x0,
    RPMI_HSM_STATE_STOPPED         = 0x1,
    RPMI_HSM_STATE_START_PENDING   = 0x2,
    RPMI_HSM_STATE_STOP_PENDING    = 0x3,
    RPMI_HSM_STATE_SUSPENDED       = 0x4,
    RPMI_HSM_STATE_SUSPEND_PENDING = 0x5,
    RPMI_HSM_STATE_RESUME_PENDING  = 0x6,
    RPMI_HSM_MAX_STATES            = 0x7
};

struct rpmi_hsm_hart_start_req {
        u32 hartid;
        u32 start_addr_lo;
        u32 start_addr_hi;
};

struct rpmi_hsm_hart_start_resp {
        s32 status;
};

struct rpmi_hsm_hart_stop_req {
        u32 hartid;
};

struct rpmi_hsm_hart_stop_resp {
        s32 status;
};

struct rpmi_hsm_hart_susp_req {
        u32 hartid;
        u32 suspend_type;
        u32 resume_addr_lo;
        u32 resume_addr_hi;
};

struct rpmi_hsm_hart_susp_resp {
        s32 status;
};

struct rpmi_hsm_get_hart_status_req {
        u32 hartid;
};

struct rpmi_hsm_get_hart_status_resp {
        s32 status;
        u32 hart_status;
};

struct rpmi_hsm_get_hart_list_req {
        u32 start_index;
};

struct rpmi_hsm_get_hart_list_resp {
        s32 status;
        u32 remaining;
        u32 returned;
        /* remaining space need to be adjusted for the above 3 u32's */
        u32 hartid[(RPMI_MSG_DATA_SIZE - (sizeof(u32) * 3)) / sizeof(u32)];
};

struct rpmi_hsm_get_susp_types_req {
        u32 start_index;
};

struct rpmi_hsm_get_susp_types_resp {
        s32 status;
        u32 remaining;
        u32 returned;
        /* remaining space need to be adjusted for the above 3 u32's */
        u32 types[(RPMI_MSG_DATA_SIZE - (sizeof(u32) * 3)) / sizeof(u32)];
};

struct rpmi_hsm_get_susp_info_req {
        u32 suspend_type;
};

struct rpmi_hsm_get_susp_info_resp {
        s32 status;
        u32 flags;
#define RPMI_HSM_FLAGS_LOCAL_TIME_STOP     (1U << 31)
        u32 entry_latency_us;
        u32 exit_latency_us;
        u32 wakeup_latency_us;
        u32 min_residency_us;
};

/** RPMI CPPC ServiceGroup Service IDs */
enum rpmi_cppc_service_id {
        RPMI_CPPC_SRV_ENABLE_NOTIFICATION = 0x01,
        RPMI_CPPC_SRV_PROBE_REG = 0x02,
        RPMI_CPPC_SRV_READ_REG = 0x03,
        RPMI_CPPC_SRV_WRITE_REG = 0x04,
        RPMI_CPPC_SRV_GET_FAST_CHANNEL_ADDR = 0x05,
        RPMI_CPPC_SRV_POKE_FAST_CHANNEL = 0x06,
        RPMI_CPPC_SRV_GET_HART_LIST = 0x07,
        RPMI_CPPC_SRV_MAX_COUNT,
};

struct rpmi_cppc_probe_req {
        u32 hart_id;
        u32 reg_id;
};

struct rpmi_cppc_probe_resp {
        s32 status;
        u32 reg_len;
};

struct rpmi_cppc_read_reg_req {
        u32 hart_id;
        u32 reg_id;
};

struct rpmi_cppc_read_reg_resp {
        s32 status;
        u32 data_lo;
        u32 data_hi;
};

struct rpmi_cppc_write_reg_req {
        u32 hart_id;
        u32 reg_id;
        u32 data_lo;
        u32 data_hi;
};

struct rpmi_cppc_write_reg_resp {
        s32 status;
};

struct rpmi_cppc_get_fast_channel_addr_req {
        u32 hart_id;
};

struct rpmi_cppc_get_fast_channel_addr_resp {
        s32 status;
#define RPMI_CPPC_FAST_CHANNEL_FLAGS_DB_WIDTH_POS       1
#define RPMI_CPPC_FAST_CHANNEL_FLAGS_DB_WIDTH_MASK      \
                        (3U << RPMI_CPPC_FAST_CHANNEL_FLAGS_DB_WIDTH_POS)
#define RPMI_CPPC_FAST_CHANNEL_FLAGS_DB_SUPPORTED       (1U << 0)
        u32 flags;
        u32 addr_lo;
        u32 addr_hi;
        u32 db_addr_lo;
        u32 db_addr_hi;
        u32 db_id_lo;
        u32 db_id_hi;
};

enum rpmi_cppc_fast_channel_db_width {
        RPMI_CPPC_FAST_CHANNEL_DB_WIDTH_8 = 0x0,
        RPMI_CPPC_FAST_CHANNEL_DB_WIDTH_16 = 0x1,
        RPMI_CPPC_FAST_CHANNEL_DB_WIDTH_32 = 0x2,
        RPMI_CPPC_FAST_CHANNEL_DB_WIDTH_64 = 0x3,
};

struct rpmi_cppc_hart_list_req {
        u32 start_index;
};

struct rpmi_cppc_hart_list_resp {
        s32 status;
        u32 remaining;
        u32 returned;
        /* remaining space need to be adjusted for the above 3 u32's */
        u32 hartid[(RPMI_MSG_DATA_SIZE - (sizeof(u32) * 3)) / sizeof(u32)];
};


#define SET_TOKEN(msg, seq)         \
({                      \
    struct rpmi_message *m_mbuf = msg;  \
    m_mbuf->header.token = htole32(seq);     \
})

#define GET_TOKEN(msg)              \
({                      \
    struct rpmi_message *m_mbuf = msg;  \
    le32toh(m_mbuf->header.token);           \
})

#define SET_DATALEN(msg, dlen)              \
({                      \
    struct rpmi_message *m_mbuf = msg;  \
    m_mbuf->header.datalen = htole32(dlen);           \
})

#define GET_DATALEN(msg)              \
({                      \
    struct rpmi_message *m_mbuf = msg;  \
    le32toh(m_mbuf->header.datalen);           \
})

#define GET_MSGIDN(msg)              \
({                      \
    struct rpmi_message *m_mbuf = msg;  \
    le32toh(m_mbuf->header.msgidn);           \
})

#define SET_MSGIDN(msg, l_msgidn)              \
({                      \
    struct rpmi_message *m_mbuf = msg;  \
    m_mbuf->header.msgidn = htole32(l_msgidn);           \
})

#define GET_MESSAGE_TYPE(msg)                           \
({                                      \
    uint32_t m_msgidn = *(uint32_t *)((char *)msg + RPMI_MSG_IDN_OFFSET);   \
    le32toh((uint32_t)((m_msgidn & RPMI_MSG_IDN_TYPE_MASK) >>           \
              RPMI_MSG_IDN_TYPE_POS));               \
})

#define GET_SERVICE_ID(msg)                         \
({                                      \
    uint32_t m_msgidn = *(uint32_t *)((char *)msg + RPMI_MSG_IDN_OFFSET);   \
    le32toh((uint32_t)((m_msgidn & RPMI_MSG_IDN_SERVICE_ID_MASK) >>         \
                    RPMI_MSG_IDN_SERVICE_ID_POS));       \
})

#define GET_SERVICEGROUP_ID(msg)                        \
({                                      \
    uint32_t m_msgidn = *(uint32_t *)((char *)msg + RPMI_MSG_IDN_OFFSET);   \
    le32toh((uint32_t)((m_msgidn & RPMI_MSG_IDN_SERVICEGROUP_ID_MASK) >>    \
                    RPMI_MSG_IDN_SERVICEGROUP_ID_POS));  \
})

/**
 * pack_message() - Pack RPMI Message.
 * @type: message type
 * @srvgrpid: service group id
 * @srvid: service id
 * @token: message token
 * @entityid: entity id
 * @dlen: data/payload size
 * @dbuf: buffer with data
 * @msgbuf: rpmi message buffer
 *
 * Return: none
 */
void rpmi_pack_message(uint32_t type, uint32_t srvgrpid,
               uint32_t srvid, uint32_t token, uint32_t dlen, void *dbuf,
               struct rpmi_message *msgbuf);

#endif /* !__RPMI_MSGPROT_H__ */
