/*
 * Support for generating APEI tables and recording CPER for Guests
 *
 * Copyright (c) 2020 HUAWEI TECHNOLOGIES CO., LTD.
 *
 * Author: Dongjiu Geng <gengdongjiu@huawei.com>
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

#ifndef ACPI_GHES_H
#define ACPI_GHES_H

#include "hw/acpi/bios-linker-loader.h"

/*
 * Values for Hardware Error Notification Type field
 */
enum AcpiGhesNotifyType {
    /* Polled */
    ACPI_GHES_NOTIFY_POLLED = 0,
    /* External Interrupt */
    ACPI_GHES_NOTIFY_EXTERNAL = 1,
    /* Local Interrupt */
    ACPI_GHES_NOTIFY_LOCAL = 2,
    /* SCI */
    ACPI_GHES_NOTIFY_SCI = 3,
    /* NMI */
    ACPI_GHES_NOTIFY_NMI = 4,
    /* CMCI, ACPI 5.0: 18.3.2.7, Table 18-290 */
    ACPI_GHES_NOTIFY_CMCI = 5,
    /* MCE, ACPI 5.0: 18.3.2.7, Table 18-290 */
    ACPI_GHES_NOTIFY_MCE = 6,
    /* GPIO-Signal, ACPI 6.0: 18.3.2.7, Table 18-332 */
    ACPI_GHES_NOTIFY_GPIO = 7,
    /* ARMv8 SEA, ACPI 6.1: 18.3.2.9, Table 18-345 */
    ACPI_GHES_NOTIFY_SEA = 8,
    /* ARMv8 SEI, ACPI 6.1: 18.3.2.9, Table 18-345 */
    ACPI_GHES_NOTIFY_SEI = 9,
    /* External Interrupt - GSIV, ACPI 6.1: 18.3.2.9, Table 18-345 */
    ACPI_GHES_NOTIFY_GSIV = 10,
    /* Software Delegated Exception, ACPI 6.2: 18.3.2.9, Table 18-383 */
    ACPI_GHES_NOTIFY_SDEI = 11,
    /* 12 and greater are reserved */
    ACPI_GHES_NOTIFY_RESERVED = 12
};

/*
 * Error Source IDs for GHES. These are just place holders
 * and each platform can define its own source ID for
 * each error source.
 */
enum {
    ACPI_GHES_DRAM_ERROR_SOURCE_ID,
    ACPI_GHES_GENERIC_CPU_ERROR_SOURCE_ID,
    ACPI_GHES_SOURCE_ID_MAX,
};

typedef struct AcpiGhesState {
    uint64_t ghes_addr_le;
    bool present; /* True if GHES is present at all on this board */
} AcpiGhesState;

enum {
    ERROR_TYPE_MEM,
    ERROR_TYPE_GENERIC_CPU,
    ERROR_TYPE_MAX,
};

enum {
    GPE_PROC_TYPE_VALID_BIT,
    GPE_PROC_ISA_VALID_BIT,
    GPE_PROC_ERR_TYPE_VALID_BIT,
    GPE_OP_VALID_BIT,
    GPE_FLAGS_VALID_BIT,
    GPE_LEVEL_VALID_BIT,
    GPE_CPU_VERSION_VALID_BIT,
    GPE_CPU_BRAND_STRING_VALID_BIT,
    GPE_CPU_ID_VALID_BIT,
    GPE_TARGET_ADDR_VALID_BIT,
    GPE_REQ_IDENT_VALID_BIT,
    GPE_RESP_IDENT_VALID_BIT,
    GPE_IP_VALID_BIT,
    GPE_BIT_RESERVED_BITS,
};

#define GPE_PROC_TYPE_VALID             (1ul << GPE_PROC_TYPE_VALID_BIT)
#define GPE_PROC_ISA_VALID              (1ul << GPE_PROC_ISA_VALID_BIT)
#define GPE_PROC_ERR_TYPE_VALID         (1ul << GPE_PROC_ERR_TYPE_VALID_BIT)
#define GPE_OP_VALID                    (1ul << GPE_OP_VALID_BIT)
#define GPE_FLAGS_VALID                 (1ul << GPE_FLAGS_VALID_BIT)
#define GPE_LEVEL_VALID                 (1ul << GPE_LEVEL_VALID_BIT)
#define GPE_CPU_VERSION_VALID           (1ul << GPE_CPU_VERSION_VALID_BIT)
#define GPE_CPU_BRAND_STRING_VALID      (1ul << GPE_CPU_BRAND_STRING_VALID_BIT)
#define GPE_CPU_ID_VALID                (1ul << GPE_CPU_ID_VALID_BIT)
#define GPE_TARGET_ADDR_VALID           (1ul << GPE_TARGET_ADDR_VALID_BIT)
#define GPE_REQ_IDENT_VALID             (1ul << GPE_REQ_IDENT_VALID_BIT)
#define GPE_RESP_IDENT_VALID            (1ul << GPE_RESP_IDENT_VALID_BIT)
#define GPE_IP_VALID                    (1ul << GPE_IP_VALID_BIT)

enum {
     GHES_PROC_TYPE_IA32X64,
     GHES_PROC_TYPE_IA64,
     GHES_PROC_TYPE_ARM,
     GHES_PROC_TYPE_RISCV,
};

enum {
     GHES_PROC_ISA_IA32,
     GHES_PROC_ISA_IA64,
     GHES_PROC_ISA_X64,
     GHES_PROC_ISA_ARM_A32,
     GHES_PROC_ISA_ARM_A64,
     GHES_PROC_ISA_RISCV32,
     GHES_PROC_ISA_RISCV64,
};

typedef struct AcpiGhesErrorInfo {
    uint32_t etype;
    union {
        struct {
            uint32_t  validation_bits;
            uint32_t  sev;
            uint8_t   proc_type;
            uint8_t   proc_isa;
            uint8_t   proc_err_type;
            uint8_t   operation;
            uint8_t   flags;
            uint8_t   level;
            uint64_t  cpu_version;
            uint8_t   cpu_brand_string[128];
            uint64_t  cpu_id;
            uint64_t  target_addr;
            uint64_t  req_ident;
            uint64_t  resp_ident;
            uint64_t  ip;
        } gpe; /* generic processor error */

        struct {
            uint64_t  physical_address;
        } me; /* DRAM Error */
    } info;
} AcpiGhesErrorInfo;

void build_ghes_error_table(GArray *hardware_errors, BIOSLinker *linker);
void acpi_build_hest(GArray *table_data, BIOSLinker *linker, uint8_t notif_type,
                     const char *oem_id, const char *oem_table_id);
void acpi_ghes_add_fw_cfg(AcpiGhesState *vms, FWCfgState *s,
                          GArray *hardware_errors);
int acpi_ghes_record_errors(uint8_t source_id, AcpiGhesErrorInfo *einfo);

/**
 * acpi_ghes_present: Report whether ACPI GHES table is present
 *
 * Returns: true if the system has an ACPI GHES table and it is
 * safe to call acpi_ghes_record_errors() to record a memory error.
 */
bool acpi_ghes_present(void);
#endif
