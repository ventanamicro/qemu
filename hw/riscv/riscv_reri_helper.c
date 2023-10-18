/*
 * RISC-V RERI Emulation Helper
 *
 * Copyright (c) 2023 Rivos Inc
 * Copyright (c) 2023 Ventana Micro Systems, Inc.
 *
 * Author(s):
 * Dhaval Sharma <dhaval@rivosinc.com>
 * Himanshu Chauhan <hchauhan@ventanamicro.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "hw/riscv/riscv_reri_helper.h"
#include "qemu/log.h"

int riscv_reri_read(RiscvReriErrorBank *err_bank, uintptr_t addr, uint64_t *out,
                    unsigned size)
{
    uint64_t val64 = 0;
    uint8_t *reg;

    if (addr + size > sizeof(RiscvReriErrorBank) ||
       ((addr & 0x7) + size) > 8) {
        return EINVAL;
    }

    reg = ((uint8_t *)err_bank) + addr;

    while (size--) {
        val64 <<= 8;
        val64 |= *(reg + size);
    }

    *out = val64;

    return 0;
}

int riscv_reri_read_error_record(RiscvReriErrorBank *err_bank, uint32_t index,
                                 RiscvReriErrorRecord *record)
{
        RiscvReriErrorRecord *rec;

        if (!err_bank)
                return EINVAL;

        if (index > err_bank->bank_info.n_err_recs)
                return ENOENT;

        rec = &err_bank->records[index];

        memcpy(record, rec, sizeof(RiscvReriErrorRecord));

        return 0;
}

int riscv_reri_write_error_record(RiscvReriErrorBank *err_bank, uint32_t index,
                                  RiscvReriErrorRecord *record)
{
        RiscvReriErrorRecord *rec;

        if (!err_bank)
                return EINVAL;

        if (index > err_bank->bank_info.n_err_recs)
                return ENOENT;

        rec = &err_bank->records[index];

        memcpy(rec, record, sizeof(RiscvReriErrorRecord));

        return 0;
}

static uint64_t riscv_reri_write_status(RiscvReriStatus old, RiscvReriStatus new)
{
    new.value &= RERI_STS_MASK;

    qemu_log_mask(LOG_GUEST_ERROR, "%s: new status 0x%lx\n",
                  __func__, new.value);

    /* Only one error type can be injected. */
    if (new.ce + new.de + new.ue != 1) {
        return old.value;
    }

    if (old.v == 0) {
        return new.value;
    }

    /* Overwrite rules. */
   if (new.ce) {
       if (old.ce) {
           old.mo = 1;
           old.v = 0;
       } else {
           old.ce = 1;
       }
       new.value = old.value;
   } else if (new.de && old.ue) {
       return old.value;
   } else if (new.ue && old.ue) {
       new.mo = 1;
   }

    return new.value;
}

int riscv_reri_write(RiscvReriErrorBank *err_bank, uintptr_t addr, uint64_t val,
                     unsigned size, bool *inject, bool *clrsts)
{
    RiscvReriErrorRecord *record;
    RiscvReriControl ctrl;
    uint64_t reg;

    qemu_log_mask(LOG_GUEST_ERROR, "%s: addr: 0x%lx val: 0x%lx size: %u\n", __func__, addr, val, size);

    if (addr + size > sizeof(RiscvReriErrorBank) ||
       ((addr & 0x7) + size) > 8) {
        return EINVAL;
    }

    if (addr < 32) {
        return 0;
    }

    reg = *(((uint64_t *)err_bank) + (addr / 8));

    /*
     * In order to handle a partial register write merge the new bytes
     * with the current value of the register.
     */
    for (int i = addr & 0x7; i < size + (addr & 0x7); i++) {
        reg &= ~((uint64_t)UINT8_MAX << 8 * i);
        reg |= (val & UINT8_MAX) << 8 * i;
        val >>= 8;
    }

    addr &= ~0x7;
    addr -= 64;
    record = &err_bank->records[addr / 32];
    addr = addr % 64;

    switch (addr) {
    case 0: /* control_i */
        ctrl.value = reg;
        if (ctrl.sinv) {
            record->status_i.v = 0;
            ctrl.sinv = 0;
            *clrsts = true;
        }
        if (ctrl.eid != 0 && record->control_i.eid == 0) {
            *inject = true;
        }
        record->control_i = ctrl;
        break;
    case 8: /* status_i */
        qemu_log_mask(LOG_GUEST_ERROR, "%s (%d): reg: 0x%lx\n", __func__, __LINE__, reg);
        record->status_i.value =
            riscv_reri_write_status(record->status_i, (RiscvReriStatus)reg);
        break;
    /* XXX: Allow modification of addr_i and info_i only if status_i.v==0? */
    case 16: /* addr_i */
        record->addr_i = reg;
        break;
    case 24: /* info_i */
        record->info_i = reg;
        break;
    }

    return 0;
}

int riscv_reri_do_inject(RiscvReriErrorRecord *record, RiscvReriStatus sts,
                         uint64_t addr, uint64_t info)
{
    RiscvReriStatus old;

    old = record->status_i;

    record->status_i.value = riscv_reri_write_status(old, sts);
    record->addr_i = addr;
    record->info_i = info;

    return 0;
}

int riscv_error_injection_tick(RiscvReriErrorRecord *record)
{
    int irq = 0;

    /* Check if the injection was cancelled. */
    if (record->control_i.eid == 0) {
        return RISCV_RERI_INJECT_ABORT;
    }
    if (--record->control_i.eid > 0) {
        return RISCV_RERI_INJECT_WAIT;
    }
    if (record->status_i.v == 1) {
        return RISCV_RERI_INJECT_ABORT;
    }

    record->status_i.v = 1;

    if (record->status_i.ue) {
        irq = record->control_i.uues;
    } else if (record->status_i.ce) {
        irq = record->control_i.ces;
    } else if (record->status_i.de) {
        irq = record->control_i.udes;
    }

    switch (irq) {
    case 1:
        return RISCV_RERI_INJECT_LOW;
    case 2:
        return RISCV_RERI_INJECT_HIGH;
    case 0:
    default:
        return RISCV_RERI_INJECT_ABORT;
    }
}

void riscv_reri_register_init(RiscvReriErrorBank *err_bank, uint16_t vendor_id,
                              uint16_t imp_id, uint16_t nr_records)
{
    err_bank->vendor_n_imp_id.vendor_id = vendor_id;
    err_bank->vendor_n_imp_id.imp_id = imp_id;
    err_bank->bank_info.inst_id = 0;
    err_bank->bank_info.n_err_recs = nr_records;
    err_bank->bank_info.version = 1;
}
