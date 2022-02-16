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
#include "qapi/error.h"
#include "qemu/bitmap.h"
#include "trace.h"
#include "hw/acpi/acpi-defs.h"
#include "hw/acpi/acpi.h"
#include "hw/acpi/bios-linker-loader.h"
#include "hw/acpi/aml-build.h"
#include "hw/acpi/utils.h"
#include "hw/acpi/pci.h"
#include "hw/acpi/memory_hotplug.h"
#include "hw/acpi/generic_event_device.h"
#include "hw/acpi/tpm.h"
#include "hw/pci/pcie_host.h"
#include "hw/pci/pci.h"
#include "hw/pci-host/gpex.h"
#include "hw/acpi/ghes.h"
#include "sysemu/reset.h"
#include "hw/riscv/virt.h"
#include "hw/riscv/numa.h"
#include "hw/intc/riscv_aclint.h"
#include "hw/intc/riscv_imsic.h"
#include "migration/vmstate.h"

#define ACPI_IMSIC_GROUP_MAX_SIZE      (1U << IMSIC_MMIO_GROUP_MIN_SHIFT)

#define ACPI_BUILD_TABLE_SIZE             0x20000

uint16_t imsic_max_hart_per_socket;

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
acpi_dsdt_add_cpus(Aml *scope, RISCVVirtState *vms)
{
    MachineState *ms = MACHINE(vms);
    uint16_t i;


    for (i = 0; i < ms->smp.cpus; i++) {
        Aml *dev = aml_device("C%.03X", i);
        aml_append(dev, aml_name_decl("_HID", aml_string("ACPI0007")));
        aml_append(dev, aml_name_decl("_UID", aml_int(i)));
        aml_append(scope, dev);
    }
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

static int
acpi_madt_aplic(uint16_t socket, RISCVVirtState *vms,
                 GArray *entry, int base_cpu_id)
{
    int cpu_id = base_cpu_id;
    uint64_t aplic_addr;
    uint32_t aplic_size;

    aplic_addr = vms->memmap[VIRT_APLIC_S].base +
        vms->memmap[VIRT_APLIC_S].size * socket;
    aplic_size = vms->memmap[VIRT_APLIC_S].size;

    build_append_int_noprefix(entry, 0x11, 1);     /* Type */
    build_append_int_noprefix(entry, 48, 1);       /* Length */
    build_append_int_noprefix(entry, 1, 1);        /* id */
    build_append_int_noprefix(entry, 1, 1);        /* Version */
    build_append_int_noprefix(entry, 1, 1);        /* mode */
    build_append_int_noprefix(entry, 0, 1);        /* reserved */
    build_append_int_noprefix(entry, 0, 2);       /* Global IRQ base */
    build_append_int_noprefix(entry, VIRT_IRQCHIP_NUM_SOURCES, 2);
    build_append_int_noprefix(entry, 0, 2);
    build_append_int_noprefix(entry, aplic_size, 4);
    build_append_int_noprefix(entry, aplic_addr, 8);
    cpu_id += vms->soc[socket].num_harts;
    return cpu_id;

}

static void
acpi_dsdt_add_aplic(Aml *scope, RISCVVirtState *vms)
{
    MachineState *ms = MACHINE(vms);
    uint16_t i, num, cpu_id = 0;

    const MemMapEntry *memmap = vms->memmap;


    for (i = 0; i < riscv_socket_count(ms); i++) {
        GArray *madt_buf = g_array_new(0, 1, 1);
        Aml *dev = aml_device("PLIC");
        aml_append(dev, aml_name_decl("_HID", aml_string("APLIC001")));
        aml_append(dev, aml_name_decl("_UID", aml_int(i)));
        /* device present, functioning, decoding, not shown in UI */
        aml_append(dev, aml_name_decl("_STA", aml_int(0xB)));
        aml_append(dev, aml_name_decl("_CCA", aml_int(1)));


        /*  _MAT */
        num = acpi_madt_aplic(i, vms, madt_buf, cpu_id);
        cpu_id += num;
        aml_append(dev, aml_name_decl("_MAT",
                                        aml_buffer(madt_buf->len,
                                                    (uint8_t *) madt_buf->
                                                    data)));
        g_array_free(madt_buf, true);

        Aml *crs = aml_resource_template();
        aml_append(crs, aml_memory32_fixed(memmap[VIRT_APLIC_S].base * i,
                                             memmap[VIRT_APLIC_S].size,
                                             AML_READ_WRITE));
        aml_append(dev, aml_name_decl("_CRS", crs));
        aml_append(scope, dev);
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

/* RTDT */
static void
build_rtdt(GArray *table_data, BIOSLinker *linker, RISCVVirtState *vms)
{
    AcpiTable table = { .sig = "RTDT", .rev = 1, .oem_id = vms->oem_id,
                        .oem_table_id = vms->oem_table_id };

    acpi_table_begin(&table, table_data);

    build_append_int_noprefix(table_data, 0, 4); /* Reserved */
    build_append_int_noprefix(table_data,
                              RISCV_ACLINT_DEFAULT_TIMEBASE_FREQ, 8);

    acpi_table_end(linker, &table);
}

/* MADT */
static void
build_madt(GArray *table_data, BIOSLinker *linker, RISCVVirtState *vms)
{
    MachineState *mc = MACHINE(vms);
    int socket;
    AcpiImsicHartIndex imsic_hart_index = { 0 };
    uint16_t node_size = 0, base_hartid = 0;
    uint32_t cpu_id = 0;
    uint32_t num_harts = 0;
    uint32_t num_sockets = riscv_socket_count(mc);
    uint64_t imsic_addr, aplic_addr;
    uint32_t imsic_size, aplic_size;

    AcpiTable table = { .sig = "APIC", .rev = 3, .oem_id = vms->oem_id,
                        .oem_table_id = vms->oem_table_id };

    acpi_table_begin(&table, table_data);
    /* Local Interrupt Controller Address */
    build_append_int_noprefix(table_data, 0, 4);
    build_append_int_noprefix(table_data, 0, 4);   /* MADT Flags */

    /* RISC-V Local INTC structures per HART */
    for (socket = 0; socket < riscv_socket_count(mc); socket++) {
        base_hartid = riscv_socket_first_hartid(mc, socket);

        for (int i = 0; i < vms->soc[socket].num_harts; i++) {
            build_append_int_noprefix(table_data, 0x13, 1);     /* Type */
            build_append_int_noprefix(table_data, 20, 1);       /* Length */
            build_append_int_noprefix(table_data, 1, 1);        /* Version */

            build_append_int_noprefix(table_data, 0, 1);       /* Reserved */

            build_append_int_noprefix(table_data, cpu_id, 4); /* ACPI Proc ID */
            /* mhartid */
            build_append_int_noprefix(table_data, (base_hartid + i), 8);
            build_append_int_noprefix(table_data, 0, 4);   /* Flags */
        }
    }

    /* IMSIC Groups */

    /* S-mode IMSIC Group */
    node_size = 20 + (12 * num_sockets);
    num_harts = mc->smp.cpus;
    imsic_hart_index.lhxw = acpi_num_bits(imsic_max_hart_per_socket);
    imsic_hart_index.hhxw = acpi_num_bits(riscv_socket_count(mc));
    imsic_hart_index.lhxs = 0;
    imsic_hart_index.hhxs = IMSIC_MMIO_GROUP_MIN_SHIFT;

    build_append_int_noprefix(table_data, 0x12, 1);     /* Type */
    build_append_int_noprefix(table_data, node_size, 1);       /* Length */
    build_append_int_noprefix(table_data, 1, 1);        /* Version */
    build_append_int_noprefix(table_data, 1, 1);        /* mode */
    build_append_int_noprefix(table_data, num_sockets, 1);
    build_append_int_noprefix(table_data, 0, 1);        /* Reserved */
    /* Num Interrupt Id */
    build_append_int_noprefix(table_data, VIRT_IRQCHIP_NUM_MSIS, 2);
    build_append_int_noprefix(table_data, VIRT_IRQCHIP_IPI_MSI, 2);
    build_append_int_noprefix(table_data, 0, 2);
    build_append_int_noprefix(table_data, num_harts, 4);
    build_append_int_noprefix(table_data, imsic_hart_index.hart_index, 4);

    for (socket = 0; socket < num_sockets; socket++) {
        imsic_addr = vms->memmap[VIRT_IMSIC_S].base + (socket *
                                               ACPI_IMSIC_GROUP_MAX_SIZE);
        imsic_size = IMSIC_HART_SIZE(0) * vms->soc[socket].num_harts;

        build_append_int_noprefix(table_data, imsic_addr, 4); /* Lo */
        build_append_int_noprefix(table_data, 0, 4);          /* Hi */
        build_append_int_noprefix(table_data, imsic_size, 4);
    }

    /* APLIC */

    for (socket = 0; socket < riscv_socket_count(mc); socket++) {
        aplic_addr = vms->memmap[VIRT_APLIC_S].base +
            vms->memmap[VIRT_APLIC_S].size * socket;
        aplic_size = vms->memmap[VIRT_APLIC_S].size;

        build_append_int_noprefix(table_data, 0x11, 1);     /* Type */
        build_append_int_noprefix(table_data, 24, 1);       /* Length */
        build_append_int_noprefix(table_data, 1, 1);        /* id */
        build_append_int_noprefix(table_data, 1, 1);        /* Version */
        build_append_int_noprefix(table_data, 1, 1);        /* mode */
        build_append_int_noprefix(table_data, 0, 1);        /* reserved */
        build_append_int_noprefix(table_data, 0, 2);       /* Global IRQ base */
        build_append_int_noprefix(table_data, VIRT_IRQCHIP_NUM_SOURCES, 2);
        build_append_int_noprefix(table_data, 0, 2);
        build_append_int_noprefix(table_data, aplic_size, 4);
        build_append_int_noprefix(table_data, aplic_addr, 8);
        cpu_id += vms->soc[socket].num_harts;
    }

    acpi_table_end(linker, &table);
}

static void
build_rhct(GArray *table_data, BIOSLinker *linker, RISCVVirtState *vms)
{
    MachineState *ms = MACHINE(vms);
    int i, socket, acpi_proc_id = 0;
    RISCVCPU *cpu;
    union AcpiRedtHartHwCap hwcap;


    AcpiTable table = { .sig = "RHCT", .rev = 1, .oem_id = vms->oem_id,
                        .oem_table_id = vms->oem_table_id };

    acpi_table_begin(&table, table_data);

    build_append_int_noprefix(table_data, 0x1, 4);   /* Flags */
    for (socket = 0; socket < riscv_socket_count(ms); socket++) {

        for (i = 0; i < vms->soc[socket].num_harts; i++) {
            cpu = &vms->soc[socket].harts[i];
            hwcap.mmu_type = RISCV_HART_CAP_MMU_TYPE_48;

            build_append_int_noprefix(table_data, 16, 2);   /* Length */
            build_append_int_noprefix(table_data, 0, 2);    /* Reserved */
            build_append_int_noprefix(table_data, acpi_proc_id, 4);
            build_append_int_noprefix(table_data, cpu->env.misa_ext, 4);
            build_append_int_noprefix(table_data, hwcap.hw_cap, 4);
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
    acpi_dsdt_add_aplic(scope, vms);
    acpi_dsdt_add_virtio(scope, &memmap[VIRT_VIRTIO],
                          (VIRTIO_IRQ), VIRTIO_COUNT);
    acpi_dsdt_add_pci(scope, memmap, PCIE_IRQ, vms);
    acpi_dsdt_add_uart(scope, &memmap[VIRT_UART0], (UART0_IRQ));

    aml_append(dsdt, scope);

    /* copy AML table into ACPI tables blob and patch header there */
    g_array_append_vals(table_data, dsdt->buf->data, dsdt->buf->len);

    acpi_table_end(linker, &table);
    free_aml_allocator();
}

typedef struct AcpiBuildState {
    /* Copy of table in RAM (for patching). */
    MemoryRegion *table_mr;
    MemoryRegion *rsdp_mr;
    MemoryRegion *linker_mr;
    /* Is table patched? */
    bool patched;
} AcpiBuildState;

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
virt_acpi_build(RISCVVirtState *vms, AcpiBuildTables *tables)
{
    GArray *table_offsets;
    unsigned dsdt, xsdt;
    GArray *tables_blob = tables->table_data;
    int socket;
    MachineState *ms = MACHINE(vms);

    table_offsets = g_array_new(false, true,
                                 sizeof(uint32_t));

    bios_linker_loader_alloc(tables->linker,
                              ACPI_BUILD_TABLE_FILE, tables_blob,
                              64, false);
    for (socket = 0; socket < riscv_socket_count(ms); socket++) {
        if (imsic_max_hart_per_socket < vms->soc[socket].num_harts) {
            imsic_max_hart_per_socket = vms->soc[socket].num_harts;
        }
    }

    /* DSDT is pointed to by FADT */
    dsdt = tables_blob->len;
    build_dsdt(tables_blob, tables->linker, vms);

    /* FADT MADT RTDT MCFG SPCR pointed to by RSDT */
    acpi_add_table(table_offsets, tables_blob);
    build_fadt_rev5(tables_blob, tables->linker, vms, dsdt);

    acpi_add_table(table_offsets, tables_blob);
    build_madt(tables_blob, tables->linker, vms);

    acpi_add_table(table_offsets, tables_blob);
    build_rtdt(tables_blob, tables->linker, vms);

    acpi_add_table(table_offsets, tables_blob);
    build_rhct(tables_blob, tables->linker, vms);

    acpi_add_table(table_offsets, tables_blob);
    {
        AcpiMcfgInfo mcfg = {
            .base = vms->memmap[VIRT_PCIE_ECAM].base,
            .size = vms->memmap[VIRT_PCIE_ECAM].size,
        };
        build_mcfg(tables_blob, tables->linker, &mcfg, vms->oem_id,
                    vms->oem_table_id);
    }

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
