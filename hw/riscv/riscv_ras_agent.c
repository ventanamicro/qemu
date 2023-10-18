/*
 * RISC-V RAS (Reliability, Availability and Serviceability)
 *
 * Copyright (c) 2023 Ventana Micro Systems, Inc.
 *
 * Author: Himanshu Chauhan <hchauhan@ventanamicro.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/coroutine.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/sysbus.h"
#include "target/riscv/cpu.h"
#include "hw/core/cpu.h"
#include "cpu-qom.h"
#include "hw/irq.h"
#include "hw/riscv/riscv_reri_hart_dev.h"
#include "hw/riscv/riscv_reri_helper.h"
#include "hw/riscv/riscv_ras_agent.h"
#include "hw/acpi/ghes.h"
#include "exec/address-spaces.h"

#ifndef MAX_HARTS
#define MAX_HARTS             128
#endif

static RasErrorSource *ras_sources = NULL;
static uint32_t nr_ras_sources = 0;
static int init_done = 0;

int ras_get_agent_version(void)
{
    return RAS_AGENT_VERSION;
}

static int riscv_reri_dev_read_u64(hwaddr dev_addr, u64 *value)
{
    MemTxResult result;
    int rc = 0;

    *value = address_space_ldq_le(&address_space_memory, dev_addr,
                                  MEMTXATTRS_UNSPECIFIED, &result);

    if (result != MEMTX_OK) {
        rc = -1;
        goto err_read_rec;
    }

 err_read_rec:
    return rc;
}

static int riscv_reri_clear_valid_bit(hwaddr control_addr)
{
    MemTxResult result;
    int rc = 0;
    uint64_t control;

    if (riscv_reri_dev_read_u64(control_addr, &control) < 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: error reading control register\n",
                      __func__);
        rc = -1;
        goto err;
    }

    /* set SINV */
    control |= 0x4;

    address_space_stq_le(&address_space_memory, control_addr,
                         control, MEMTXATTRS_UNSPECIFIED, &result);

    if (result != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: error writing contol\n",
                      __func__);
        rc = -1;
        goto err;
    }

 err:
    return rc;
}

static int riscv_reri_get_hart_addr(int hart_id, hwaddr *hart_addr, hwaddr *size)
{
    /* FIXME: only hart is supported */
    if (hart_id > 0)
        return -1;

    *hart_addr = (ras_sources->as);
    *size = 0x1000;

    return 0;
}

static int riscv_reri_get_hart_sse_vector(int hart_id)
{
    if (hart_id > 0)
        return -1;

    return ras_sources->sse_vector;
}

int riscv_ras_agent_synchronize_hart_errors(int hart_id,
                                            struct rpmi_ras_sync_err_resp *resp)
{
    int rc;
    RiscvReriErrorBank *hart_err_bank;
    RiscvReriStatus status, _status;
    hwaddr hart_addr, err_size;
    uint64_t valid_summary, eaddr;
    AcpiGhesErrorInfo einfo;

    if (!init_done) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: RAS agent not initialized.\n",
                      __func__);
        resp->status = -1;
        return -1;
    }

    if (riscv_reri_get_hart_addr(hart_id, &hart_addr, &err_size) != 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Invalid hart: %d\n",
                      __func__, hart_id);
        return -1;
    }

    hart_err_bank = (RiscvReriErrorBank *)hart_addr;

    if (riscv_reri_dev_read_u64((hwaddr)&hart_err_bank->valid_summary,
                                &valid_summary) != 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Failed to read valid summary\n",
                      __func__);
        resp->status = -1;
        return -1;
    }

    if (riscv_reri_dev_read_u64(
                (hwaddr)&hart_err_bank->records[0].status_i.value,
                &status.value) != 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: failed to read status\n",
                      __func__);
        resp->status = -1;
        return -1;
    }

    if (riscv_reri_dev_read_u64((hwaddr)&hart_err_bank->records[0].addr_i,
                                &eaddr) != 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: failed to read eaddr\n",
                      __func__);
        resp->status = -1;
        return -1;
    }

    /* Error is valid process it */
    if (status.v == 1) {
        riscv_reri_clear_valid_bit((hwaddr)&hart_err_bank->records[0].control_i.value);
        if (riscv_reri_dev_read_u64(
                    (hwaddr)&hart_err_bank->records[0].status_i.value,
                    &_status.value) != 0) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: failed to read status\n",
                          __func__);
            resp->status = -1;
            return -1;
        }

        if (status.ce)
            einfo.info.gpe.sev = 2;
        else if (status.de)
            einfo.info.gpe.sev = 0; /* deferred, recoverable? */
        else if (status.ue)
            einfo.info.gpe.sev = 1; /* fatal error */
        else
            einfo.info.gpe.sev = 3; /* Unknown */

        einfo.info.gpe.validation_bits = (GPE_PROC_TYPE_VALID |
                                     GPE_PROC_ISA_VALID |
                                     GPE_PROC_ERR_TYPE_VALID);

        einfo.info.gpe.proc_type = GHES_PROC_TYPE_RISCV;
        einfo.info.gpe.proc_isa = GHES_PROC_ISA_RISCV64;

        if (status.tt &&
            (status.tt >= 4 && status.tt <= 7)) {
                einfo.info.gpe.validation_bits |= GPE_OP_VALID;

            /* Transaction type */
            switch(status.tt) {
            case RERI_TT_IMPLICIT_READ:
                einfo.info.gpe.operation = 3;
                break;
            case RERI_TT_EXPLICIT_READ:
                einfo.info.gpe.operation = 1;
                break;
            case RERI_TT_IMPLICIT_WRITE:
            case RERI_TT_EXPLICIT_WRITE:
                einfo.info.gpe.operation = 2;
                break;
            default:
                einfo.info.gpe.operation = 0;
                break;
            }

            /* Translate error codes from RERI */
            switch(status.ec) {
            case RERI_EC_CBA:
            case RERI_EC_CSD:
            case RERI_EC_CAS:
            case RERI_EC_CUE:
                einfo.info.gpe.proc_err_type = 0x01;
                break;
            case RERI_EC_TPD:
            case RERI_EC_TPA:
            case RERI_EC_TPU:
                einfo.info.gpe.proc_err_type = 0x02;
                break;
            case RERI_EC_SBE:
                einfo.info.gpe.proc_err_type = 0x04;
                break;
            case RERI_EC_HSE:
            case RERI_EC_ITD:
            case RERI_EC_ITO:
            case RERI_EC_IWE:
            case RERI_EC_IDE:
            case RERI_EC_SMU:
            case RERI_EC_SMD:
            case RERI_EC_SMS:
            case RERI_EC_PIO:
            case RERI_EC_PUS:
            case RERI_EC_PTO:
            case RERI_EC_SIC:
                einfo.info.gpe.proc_err_type = 0x08;
                break;
            default:
                einfo.info.gpe.proc_err_type = 0x00;
                break;
            }
        }

        /* Address type */
        if (status.at) {
            einfo.info.gpe.validation_bits |= GPE_TARGET_ADDR_VALID;
            einfo.info.gpe.target_addr = eaddr;
        }

        einfo.etype = ERROR_TYPE_GENERIC_CPU;

        /* Update the CPER record */
        rc = acpi_ghes_record_errors(ACPI_GHES_GENERIC_CPU_ERROR_SOURCE_ID,
                                     &einfo);
        if (rc < 0) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: Failed to log error in APEI\n", __func__);
            resp->status = -1;
            return -1;
        }

        resp->returned = 1;
        resp->remaining = 0;
        resp->status = 0;
        /* Generic CPU error vector is 1 */
        resp->pending_vecs[0] = riscv_reri_get_hart_sse_vector(hart_id);
    }

    return 0;
}

int riscv_ras_agent_init(RasErrorSource *sources, uint32_t nr_sources)
{
    if (sources == NULL)
        return -1;

    ras_sources = sources;
    nr_ras_sources = nr_sources;

    init_done = 1;

    return 0;
}
