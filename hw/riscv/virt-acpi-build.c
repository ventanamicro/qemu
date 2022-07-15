/*
 * Support for generating ACPI tables and passing them to Guests
 *
 * RISC-V virt ACPI generation
 *
 * Copyright (C) 2008-2010  Kevin O'Connor <kevin@koconnor.net>
 * Copyright (C) 2006 Fabrice Bellard
 * Copyright (C) 2013 Red Hat Inc
 * Copyright (C) 2021 Ventana Micro Systems Inc
 *
 * Author: Michael S. Tsirkin <mst@redhat.com>
 *
 * Copyright (c) 2015 HUAWEI TECHNOLOGIES CO.,LTD.
 *
 * Author: Shannon Zhao <zhaoshenglong@huawei.com>
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
#include "hw/acpi/acpi-defs.h"
#include "hw/acpi/acpi.h"
#include "hw/acpi/aml-build.h"
#include "hw/riscv/virt.h"
#include "hw/riscv/numa.h"
#include "hw/acpi/pci.h"
#include "hw/acpi/utils.h"
#include "sysemu/reset.h"
#include "hw/pci-host/gpex.h"
#include "qapi/error.h"
#include "migration/vmstate.h"
#include "hw/intc/riscv_aclint.h"

#define ACPI_BUILD_TABLE_SIZE             0x20000

typedef struct AcpiBuildState {
    /* Copy of table in RAM (for patching). */
    MemoryRegion *table_mr;
    MemoryRegion *rsdp_mr;
    MemoryRegion *linker_mr;
    /* Is table patched? */
    bool patched;
} AcpiBuildState;

static uint32_t
acpi_num_bits(uint32_t count)
{
    uint32_t ret = 0;

    while (BIT(ret) < count) {
        ret++;
    }

    return ret;
}

static void
acpi_align_size(GArray *blob, unsigned align)
{
    /*
     * Align size to multiple of given size. This reduces the chance
     * we need to change size in the future (breaking cross version migration).
     */
    g_array_set_size(blob, ROUND_UP(acpi_data_len(blob), align));
}

static void
acpi_dsdt_add_cpus(Aml *scope, RISCVVirtState *vms)
{
    MachineState *ms = MACHINE(vms);
    uint16_t i;

    /* LPI[0]  - Default WFI */
    Aml *lpi0 = aml_package(10);
    aml_append(lpi0, aml_int(1));    /* Min Residency */
    aml_append(lpi0, aml_int(1));    /* Worst case wakeup latency */
    aml_append(lpi0, aml_int(1));    /* Flags */
    aml_append(lpi0, aml_int(0));    /* Arch Context loss flags */
    aml_append(lpi0, aml_int(100));  /* Residency counter frequency */
    aml_append(lpi0, aml_int(0));    /* Enabled parent state */

    Aml *entry0 = aml_resource_template();
    build_append_int_noprefix(entry0->buf, 0x82, 1);    /* descriptor */
    build_append_int_noprefix(entry0->buf, 0x12, 1);    /* length[7:0] */
    build_append_int_noprefix(entry0->buf, 0x0, 1);     /* length[15:8] */
    build_append_gas(entry0->buf, AML_AS_FFH, 0x40, 0, 4, 0x100000000);
    aml_append(lpi0, entry0);

    Aml *use_res_count = aml_resource_template();
    build_append_int_noprefix(use_res_count->buf, 0x82, 1);  /* descriptor */
    build_append_int_noprefix(use_res_count->buf, 0x12, 1);  /* length[7:0] */
    build_append_int_noprefix(use_res_count->buf, 0x0, 1);   /* length[15:8] */
    build_append_gas(use_res_count->buf, AML_AS_SYSTEM_MEMORY, 0, 0, 0, 0);

    aml_append(lpi0, use_res_count);
    aml_append(lpi0, use_res_count);
    aml_append(lpi0, aml_string("RISC-V WFI"));

    /* LPI[1] */
    Aml *lpi1 = aml_package(10);
    aml_append(lpi1, aml_int(10));  /* Min Residency */
    aml_append(lpi1, aml_int(10));  /* Worst case wakeup latency */
    aml_append(lpi1, aml_int(1));   /* Flags */
    aml_append(lpi1, aml_int(0));   /* Arch Context loss flags */
    aml_append(lpi1, aml_int(100)); /* Residency counter frequency */
    aml_append(lpi1, aml_int(1));   /* Enabled parent state */

    Aml *entry1 = aml_resource_template();
    build_append_int_noprefix(entry1->buf, 0x82, 1);    /* descriptor */
    build_append_int_noprefix(entry1->buf, 0x12, 1);    /* length[7:0] */
    build_append_int_noprefix(entry1->buf, 0x0, 1);     /* length[15:8] */
    build_append_gas(entry1->buf, AML_AS_FFH, 0x20, 0, 3, 0x00000000);
    aml_append(lpi1, entry1);
    aml_append(lpi1, use_res_count);
    aml_append(lpi1, use_res_count);
    aml_append(lpi1, aml_string("RISC-V RET_DEFAULT"));

    /* LPI[2] */
    Aml *lpi2 = aml_package(10);
    aml_append(lpi2, aml_int(3500)); /* Min Residency */
    aml_append(lpi2, aml_int(100));  /* Worst case wakeup latency */
    aml_append(lpi2, aml_int(1));    /* Flags */
    aml_append(lpi2, aml_int(0));    /* Arch Context loss flags */
    aml_append(lpi2, aml_int(100));  /* Residency counter frequency */
    aml_append(lpi2, aml_int(1));    /* Enabled parent state */

    Aml *entry2 = aml_resource_template();
    build_append_int_noprefix(entry2->buf, 0x82, 1);    /* descriptor */
    build_append_int_noprefix(entry2->buf, 0x12, 1);    /* length[7:0] */
    build_append_int_noprefix(entry2->buf, 0x0, 1);     /* length[15:8] */
    build_append_gas(entry2->buf, AML_AS_FFH, 0x20, 0, 3, 0x80000000);
    aml_append(lpi2, entry2);
    aml_append(lpi2, use_res_count);
    aml_append(lpi2, use_res_count);
    aml_append(lpi2, aml_string("RISC-V NONRET_DEFAULT"));

    /*  _LPI */
    Aml *pkg = aml_package(6);
    aml_append(pkg, aml_int(0)); /* Version */
    aml_append(pkg, aml_int(0)); /* Level Index */
    aml_append(pkg, aml_int(3)); /* Count */
    aml_append(pkg, lpi0);
    aml_append(pkg, lpi1);
    aml_append(pkg, lpi2);

    for (i = 0; i < ms->smp.cpus; i++) {
        Aml *dev = aml_device("C%.03X", i);
        aml_append(dev, aml_name_decl("_HID", aml_string("ACPI0007")));
        aml_append(dev, aml_name_decl("_UID", aml_int(i)));
        aml_append(dev, aml_name_decl("_LPI", pkg));
        aml_append(scope, dev);
    }
}

static void
acpi_dsdt_add_fw_cfg(Aml *scope, const MemMapEntry *fw_cfg_memmap)
{
    Aml *dev = aml_device("FWCF");
    aml_append(dev, aml_name_decl("_HID", aml_string("QEMU0002")));
    /* device present, functioning, decoding, not shown in UI */
    aml_append(dev, aml_name_decl("_STA", aml_int(0xB)));
    aml_append(dev, aml_name_decl("_CCA", aml_int(1)));

    Aml *crs = aml_resource_template();
    aml_append(crs, aml_memory32_fixed(fw_cfg_memmap->base,
                                         fw_cfg_memmap->size, AML_READ_WRITE));
    aml_append(dev, aml_name_decl("_CRS", crs));
    aml_append(scope, dev);
}

static void
acpi_dsdt_add_uart(Aml *scope, const MemMapEntry *uart_memmap,
                    uint32_t uart_irq)
{
    Aml *dev = aml_device("COM0");
    aml_append(dev, aml_name_decl("_HID", aml_string("PNP0501")));
    aml_append(dev, aml_name_decl("_UID", aml_int(0)));

    Aml *crs = aml_resource_template();
    aml_append(crs, aml_memory32_fixed(uart_memmap->base,
                                         uart_memmap->size, AML_READ_WRITE));
    aml_append(crs,
                aml_interrupt(AML_CONSUMER, AML_LEVEL, AML_ACTIVE_HIGH,
                               AML_EXCLUSIVE, &uart_irq, 1));
    aml_append(dev, aml_name_decl("_CRS", crs));

    Aml *pkg = aml_package(2);
    aml_append(pkg, aml_string("clock-frequency"));
    aml_append(pkg, aml_int(3686400));

    Aml *UUID = aml_touuid("DAFFD814-6EBA-4D8C-8A91-BC9BBF4AA301");

    Aml *pkg1 = aml_package(1);
    aml_append(pkg1, pkg);

    Aml *package = aml_package(2);
    aml_append(package, UUID);
    aml_append(package, pkg1);

    aml_append(dev, aml_name_decl("_DSD", package));
    aml_append(scope, dev);
}

static void
acpi_dsdt_add_virtio(Aml *scope,
                      const MemMapEntry *virtio_mmio_memmap,
                      uint32_t mmio_irq, int num)
{
    hwaddr base = virtio_mmio_memmap->base;
    hwaddr size = virtio_mmio_memmap->size;
    int i;

    for (i = 0; i < num; i++) {
        uint32_t irq = mmio_irq + i;
        Aml *dev = aml_device("VR%02u", i);
        aml_append(dev, aml_name_decl("_HID", aml_string("LNRO0005")));
        aml_append(dev, aml_name_decl("_UID", aml_int(i)));
        aml_append(dev, aml_name_decl("_CCA", aml_int(1)));

        Aml *crs = aml_resource_template();
        aml_append(crs, aml_memory32_fixed(base, size, AML_READ_WRITE));
        aml_append(crs,
                    aml_interrupt(AML_CONSUMER, AML_LEVEL, AML_ACTIVE_HIGH,
                                   AML_EXCLUSIVE, &irq, 1));
        aml_append(dev, aml_name_decl("_CRS", crs));
        aml_append(scope, dev);
        base += size;
    }
}

static void
acpi_dsdt_add_pci(Aml *scope, const MemMapEntry *memmap,
                   uint32_t irq, RISCVVirtState *vms)
{
    struct GPEXConfig cfg = {
        .mmio32 = memmap[VIRT_PCIE_MMIO],
        .mmio64 = memmap[VIRT_HIGH_PCIE_MMIO],
        .pio = memmap[VIRT_PCIE_PIO],
        .ecam = memmap[VIRT_PCIE_ECAM],
        .irq = irq,
        .bus = vms->bus,
    };

    acpi_dsdt_add_gpex(scope, &cfg);
}

#define RHCT_NODE_ARRAY_OFFSET 56
static void
build_rhct(GArray *table_data, BIOSLinker *linker, RISCVVirtState *vms)
{
    MachineState *ms = MACHINE(vms);
    uint32_t acpi_proc_id = 0;
    int i, socket;
    RISCVCPU *cpu;
    char *isa;
    size_t len, aligned_len;
    uint32_t isa_offset, num_rhct_nodes, cmo_offset;

    AcpiTable table = { .sig = "RHCT", .rev = 1, .oem_id = vms->oem_id,
                        .oem_table_id = vms->oem_table_id };

    acpi_table_begin(&table, table_data);

    build_append_int_noprefix(table_data, 0x0, 4);   /* Reserved */
    build_append_int_noprefix(table_data,
                              RISCV_ACLINT_DEFAULT_TIMEBASE_FREQ, 8);

    /* ISA + N hart info */
    num_rhct_nodes = 2 + ms->smp.cpus;
    build_append_int_noprefix(table_data, num_rhct_nodes, 4);   /* Number of RHCT nodes */
    build_append_int_noprefix(table_data, RHCT_NODE_ARRAY_OFFSET, 4); /* Offset to RHCT node array */

    /* ISA string node */
    isa_offset = table_data->len - table.table_offset;
    build_append_int_noprefix(table_data, 0, 2);   /* Type */

    cpu = &vms->soc[0].harts[0];
    isa = riscv_isa_string(cpu);
    len = 8 + strlen(isa) + 1;
    aligned_len = (len % 2) ? (len + 1) : len;

    build_append_int_noprefix(table_data, aligned_len, 2);   /* Total length */
    build_append_int_noprefix(table_data, 0x1, 2);   /* Revision */
    build_append_int_noprefix(table_data, strlen(isa) + 1, 2);   /* ISA length */
    g_array_append_vals(table_data, isa, strlen(isa) + 1);   /* ISA string */
    if (aligned_len != len)
        build_append_int_noprefix(table_data, 0x0, 1);   /* pad */

    /* CMO node */
    cmo_offset = table_data->len - table.table_offset;
    build_append_int_noprefix(table_data, 1, 2);    /* Type */
    build_append_int_noprefix(table_data, 12, 2);    /* Total length */
    build_append_int_noprefix(table_data, 0x1, 2);   /* Revision */
    build_append_int_noprefix(table_data, 64, 2);    /* CBOM Block Size */
    build_append_int_noprefix(table_data, 64, 2);    /* CBOP Block Size */
    build_append_int_noprefix(table_data, 64, 2);    /* CBOZ Block Size */

    for (socket = 0; socket < riscv_socket_count(ms); socket++) {
        for (i = 0; i < vms->soc[socket].num_harts; i++) {
            build_append_int_noprefix(table_data, 0xFFFF, 2);  /* Type */
            build_append_int_noprefix(table_data, 20, 2);   /* Length */
            build_append_int_noprefix(table_data, 0x1, 2);   /* Revision */
            build_append_int_noprefix(table_data, 2, 2);     /* number of offsets */
            build_append_int_noprefix(table_data, acpi_proc_id, 4); /* ACPI proc ID */
            build_append_int_noprefix(table_data, isa_offset, 4);    /* ISA node offset */
            build_append_int_noprefix(table_data, cmo_offset, 4);    /* CMO node offset */
	    acpi_proc_id++;
        }
    }

    acpi_table_end(linker, &table);
}

/* FADT */
static void
build_fadt_rev5(GArray *table_data, BIOSLinker *linker,
                 RISCVVirtState *vms, unsigned dsdt_tbl_offset)
{
    /* ACPI v5.1 */
    AcpiFadtData fadt = {
        .rev = 5,
        .minor_ver = 1,
        .flags = 1 << ACPI_FADT_F_HW_REDUCED_ACPI,
        .xdsdt_tbl_offset = &dsdt_tbl_offset,
    };

    build_fadt(table_data, linker, &fadt, vms->oem_id, vms->oem_table_id);
}

/* DSDT */
static void
build_dsdt(GArray *table_data, BIOSLinker *linker, RISCVVirtState *vms)
{
    Aml *scope, *dsdt;
    MachineState *ms = MACHINE(vms);
    uint8_t socket_count;
    const MemMapEntry *memmap = vms->memmap;
    AcpiTable table = { .sig = "DSDT", .rev = 2, .oem_id = vms->oem_id,
                        .oem_table_id = vms->oem_table_id };


    acpi_table_begin(&table, table_data);
    dsdt = init_aml_allocator();

    /*
     * When booting the VM with UEFI, UEFI takes ownership of the RTC hardware.
     * While UEFI can use libfdt to disable the RTC device node in the DTB that
     * it passes to the OS, it cannot modify AML. Therefore, we won't generate
     * the RTC ACPI device at all when using UEFI.
     */
    scope = aml_scope("\\_SB");
    acpi_dsdt_add_cpus(scope, vms);

    acpi_dsdt_add_fw_cfg(scope, &memmap[VIRT_FW_CFG]);

    socket_count = riscv_socket_count(ms);

    acpi_dsdt_add_uart(scope, &memmap[VIRT_UART0], (UART0_IRQ));

    if (socket_count == 1) {
        acpi_dsdt_add_virtio(scope, &memmap[VIRT_VIRTIO],
                             (VIRTIO_IRQ), VIRTIO_COUNT);
        acpi_dsdt_add_pci(scope, memmap, PCIE_IRQ, vms);
    } else if (socket_count == 2) {
        acpi_dsdt_add_virtio(scope, &memmap[VIRT_VIRTIO],
                             (VIRTIO_IRQ + VIRT_IRQCHIP_NUM_SOURCES), VIRTIO_COUNT);
        acpi_dsdt_add_pci(scope, memmap, PCIE_IRQ + VIRT_IRQCHIP_NUM_SOURCES, vms);
    } else {
        acpi_dsdt_add_virtio(scope, &memmap[VIRT_VIRTIO],
                             (VIRTIO_IRQ + VIRT_IRQCHIP_NUM_SOURCES), VIRTIO_COUNT);
        acpi_dsdt_add_pci(scope, memmap, PCIE_IRQ + VIRT_IRQCHIP_NUM_SOURCES * 2, vms);
    }

    aml_append(dsdt, scope);

    /* copy AML table into ACPI tables blob and patch header there */
    g_array_append_vals(table_data, dsdt->buf->data, dsdt->buf->len);

    acpi_table_end(linker, &table);
    free_aml_allocator();
}

/* MADT */
static void
build_madt(GArray *table_data, BIOSLinker *linker, RISCVVirtState *vms)
{
    MachineState *mc = MACHINE(vms);
    int socket;
    uint16_t base_hartid = 0;
    uint32_t cpu_id = 0;
    uint64_t imsic_socket_addr, imsic_addr, aplic_addr;
    uint32_t imsic_size, gsi_base;
    uint8_t  hart_index_bits, group_index_bits;
    uint8_t  group_index_shift, guest_index_bits;
    uint16_t imsic_max_hart_per_socket;

    AcpiTable table = { .sig = "APIC", .rev = 3, .oem_id = vms->oem_id,
                        .oem_table_id = vms->oem_table_id };

    acpi_table_begin(&table, table_data);
    /* Local Interrupt Controller Address */
    build_append_int_noprefix(table_data, 0, 4);
    build_append_int_noprefix(table_data, 0, 4);   /* MADT Flags */

    /* RISC-V Local INTC structures per HART */
    if (vms->aia_type == VIRT_AIA_TYPE_NONE) {
        for (socket = 0; socket < riscv_socket_count(mc); socket++) {
            base_hartid = riscv_socket_first_hartid(mc, socket);

            for (int i = 0; i < vms->soc[socket].num_harts; i++) {
                build_append_int_noprefix(table_data, 0x18, 1);    /* Type         */
                build_append_int_noprefix(table_data, 20, 1);      /* Length       */
                build_append_int_noprefix(table_data, 1, 1);       /* Version      */
                build_append_int_noprefix(table_data, 0, 1);       /* Reserved     */
                build_append_int_noprefix(table_data, 5, 4);       /* Flags        */
                build_append_int_noprefix(table_data,
                                          (base_hartid + i), 8);   /* hartid       */
                build_append_int_noprefix(table_data, cpu_id, 4);  /* ACPI Proc ID */
                cpu_id++;
            }
        }
    } else {
        imsic_max_hart_per_socket = 0;
        for (socket = 0; socket < riscv_socket_count(mc); socket++) {
            if (imsic_max_hart_per_socket < vms->soc[socket].num_harts) {
                imsic_max_hart_per_socket = vms->soc[socket].num_harts;
            }
        }

        hart_index_bits = acpi_num_bits(imsic_max_hart_per_socket);
        group_index_bits = acpi_num_bits(riscv_socket_count(mc));
        group_index_shift = IMSIC_MMIO_GROUP_MIN_SHIFT;
        guest_index_bits = acpi_num_bits(vms->aia_guests + 1);

        /* RISC-V Local INTC structures per HART */
        for (socket = 0; socket < riscv_socket_count(mc); socket++) {
            base_hartid = riscv_socket_first_hartid(mc, socket);
            imsic_socket_addr = vms->memmap[VIRT_IMSIC_S].base + (socket *
                                                   VIRT_IMSIC_GROUP_MAX_SIZE);

            for (int i = 0; i < vms->soc[socket].num_harts; i++) {
                imsic_addr = imsic_socket_addr + i * IMSIC_HART_SIZE(guest_index_bits);
                imsic_size = IMSIC_HART_SIZE(guest_index_bits);
                build_append_int_noprefix(table_data, 0x18, 1);    /* Type         */
                build_append_int_noprefix(table_data, 36, 1);      /* Length       */
                build_append_int_noprefix(table_data, 2, 1);       /* Version      */
                build_append_int_noprefix(table_data, 0, 1);       /* Reserved     */
                build_append_int_noprefix(table_data, 5, 4);       /* Flags        */
                build_append_int_noprefix(table_data,
                                          (base_hartid + i), 8);   /* hartid       */
                build_append_int_noprefix(table_data, cpu_id, 4);  /* ACPI Proc ID */
                build_append_int_noprefix(table_data, socket, 4);  /* APLIC ID */
                build_append_int_noprefix(table_data, imsic_addr, 8);
                build_append_int_noprefix(table_data, imsic_size, 4);
                cpu_id++;
            }
        }

        /* IMSIC */
        build_append_int_noprefix(table_data, 0x19, 1);     /* Type */
        build_append_int_noprefix(table_data, 16, 1);       /* Length */
        build_append_int_noprefix(table_data, 1, 1);        /* Version */
        build_append_int_noprefix(table_data, 0, 1);        /* Reserved */
        build_append_int_noprefix(table_data, 0, 4);        /* Flags */
        build_append_int_noprefix(table_data, VIRT_IRQCHIP_NUM_MSIS, 2); /* S-level */
        build_append_int_noprefix(table_data, VIRT_IRQCHIP_NUM_MSIS, 2); /* VS-level */
        build_append_int_noprefix(table_data, guest_index_bits, 1);
        build_append_int_noprefix(table_data, hart_index_bits, 1);
        build_append_int_noprefix(table_data, group_index_bits, 1);
        build_append_int_noprefix(table_data, group_index_shift, 1);
    }

    if (vms->aia_type != VIRT_AIA_TYPE_NONE) {
        /* APLICs */
        for (socket = 0; socket < riscv_socket_count(mc); socket++) {
            aplic_addr = vms->memmap[VIRT_APLIC_S].base + vms->memmap[VIRT_APLIC_S].size * socket;
            gsi_base = VIRT_IRQCHIP_NUM_SOURCES * socket;
            build_append_int_noprefix(table_data, 0x1A, 1);     /* Type */
            build_append_int_noprefix(table_data, 38, 1);       /* Length */
            build_append_int_noprefix(table_data, 1, 1);        /* Version */
            build_append_int_noprefix(table_data, 0, 1);        /* Reserved */
            build_append_int_noprefix(table_data, socket, 4);   /* APLIC ID */
            build_append_int_noprefix(table_data, 0, 8);        /* MFG ID */
            if (vms->aia_type == VIRT_AIA_TYPE_APLIC) {
                build_append_int_noprefix(table_data, vms->soc[socket].num_harts, 4);        /* nr_idcs */
            } else {
                build_append_int_noprefix(table_data, 0, 4);        /* nr_idcs */
            }
            build_append_int_noprefix(table_data, gsi_base, 4);        /* GSI Base */
            build_append_int_noprefix(table_data, aplic_addr, 8);        /* MMIO base */
            build_append_int_noprefix(table_data, vms->memmap[VIRT_APLIC_S].size, 4); /* MMIO size */
            build_append_int_noprefix(table_data, VIRT_IRQCHIP_NUM_SOURCES, 2);        /* nr_irqs */
        }
    }

    acpi_table_end(linker, &table);
}

static void
virt_acpi_build(RISCVVirtState *vms, AcpiBuildTables *tables)
{
    GArray *table_offsets;
    unsigned dsdt, xsdt;
    GArray *tables_blob = tables->table_data;

    table_offsets = g_array_new(false, true,
                                 sizeof(uint32_t));

    bios_linker_loader_alloc(tables->linker,
                              ACPI_BUILD_TABLE_FILE, tables_blob,
                              64, false);

    /* DSDT is pointed to by FADT */
    dsdt = tables_blob->len;
    build_dsdt(tables_blob, tables->linker, vms);

    /* FADT and others pointed to by RSDT */
    acpi_add_table(table_offsets, tables_blob);
    build_fadt_rev5(tables_blob, tables->linker, vms, dsdt);

    acpi_add_table(table_offsets, tables_blob);
    build_madt(tables_blob, tables->linker, vms);

    acpi_add_table(table_offsets, tables_blob);
    build_rhct(tables_blob, tables->linker, vms);

    /* XSDT is pointed to by RSDP */
    xsdt = tables_blob->len;
    build_xsdt(tables_blob, tables->linker, table_offsets, vms->oem_id,
                vms->oem_table_id);

    /* RSDP is in FSEG memory, so allocate it separately */
    {
        AcpiRsdpData rsdp_data = {
            .revision = 2,
            .oem_id = vms->oem_id,
            .xsdt_tbl_offset = &xsdt,
            .rsdt_tbl_offset = NULL,
        };
        build_rsdp(tables->rsdp, tables->linker, &rsdp_data);
    }

    /*
     * The align size is 128, warn if 64k is not enough therefore
     * the align size could be resized.
     */
    if (tables_blob->len > ACPI_BUILD_TABLE_SIZE / 2) {
        warn_report("ACPI table size %u exceeds %d bytes,"
                     " migration may not work",
                     tables_blob->len, ACPI_BUILD_TABLE_SIZE / 2);
        error_printf("Try removing CPUs, NUMA nodes, memory slots"
                      " or PCI bridges.");
    }
    acpi_align_size(tables_blob, ACPI_BUILD_TABLE_SIZE);


    /* Cleanup memory that's no longer used. */
    g_array_free(table_offsets, true);
}

static void
acpi_ram_update(MemoryRegion *mr, GArray *data)
{
    uint32_t size = acpi_data_len(data);

    /*
     * Make sure RAM size is correct - in case it got changed
     * e.g. by migration
     */
    memory_region_ram_resize(mr, size, &error_abort);

    memcpy(memory_region_get_ram_ptr(mr), data->data, size);
    memory_region_set_dirty(mr, 0, size);
}

static void
virt_acpi_build_update(void *build_opaque)
{
    AcpiBuildState *build_state = build_opaque;
    AcpiBuildTables tables;

    /* No state to update or already patched? Nothing to do. */
    if (!build_state || build_state->patched) {
        return;
    }
    build_state->patched = true;

    acpi_build_tables_init(&tables);

    virt_acpi_build(RISCV_VIRT_MACHINE(qdev_get_machine()), &tables);

    acpi_ram_update(build_state->table_mr, tables.table_data);
    acpi_ram_update(build_state->rsdp_mr, tables.rsdp);
    acpi_ram_update(build_state->linker_mr, tables.linker->cmd_blob);

    acpi_build_tables_cleanup(&tables, true);
}

static void
virt_acpi_build_reset(void *build_opaque)
{
    AcpiBuildState *build_state = build_opaque;
    build_state->patched = false;
}

static const VMStateDescription vmstate_virt_acpi_build = {
    .name = "virt_acpi_build",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_BOOL(patched, AcpiBuildState),
        VMSTATE_END_OF_LIST()
    },
};

void
virt_acpi_setup(RISCVVirtState *vms)
{
    AcpiBuildTables tables;
    AcpiBuildState *build_state;

    build_state = g_malloc0(sizeof *build_state);

    acpi_build_tables_init(&tables);
    virt_acpi_build(vms, &tables);

    /* Now expose it all to Guest */
    build_state->table_mr = acpi_add_rom_blob(virt_acpi_build_update,
                                               build_state, tables.table_data,
                                               ACPI_BUILD_TABLE_FILE);
    assert(build_state->table_mr != NULL);

    build_state->linker_mr = acpi_add_rom_blob(virt_acpi_build_update,
                                                build_state,
                                                tables.linker->cmd_blob,
                                                ACPI_BUILD_LOADER_FILE);

    build_state->rsdp_mr = acpi_add_rom_blob(virt_acpi_build_update,
                                              build_state, tables.rsdp,
                                              ACPI_BUILD_RSDP_FILE);

    qemu_register_reset(virt_acpi_build_reset, build_state);
    virt_acpi_build_reset(build_state);
    vmstate_register(NULL, 0, &vmstate_virt_acpi_build, build_state);

    /*
     * Cleanup tables but don't free the memory: we track it
     * in build_state.
     */
    acpi_build_tables_cleanup(&tables, false);
}
