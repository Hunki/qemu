/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "cpu.h"
#include "hw/sysbus.h"
#include "hw/arm/boot.h"
#include "net/net.h"
#include "exec/address-spaces.h"
#include "sysemu/sysemu.h"
#include "hw/boards.h"
#include "hw/block/flash.h"
#include "hw/loader.h"
#include "hw/ssi/ssi.h"
#include "hw/usb/chipidea.h"
#include "qemu/error-report.h"
#include "hw/sd/sdhci.h"
#include "hw/char/cadence_uart.h"
#include "hw/net/cadence_gem.h"
#include "hw/cpu/a9mpcore.h"
#include "hw/qdev-clock.h"
#include "sysemu/reset.h"

#define TYPE_COSMOS_PLUS MACHINE_TYPE_NAME("cosmos-plus")
#define COSMOS_PLUS(obj) \
    OBJECT_CHECK(CosmosPlusMachineState, (obj), TYPE_COSMOS_PLUS)

/* board base frequency: 33.333333 MHz */
#define PS_CLK_FREQUENCY (100 * 1000 * 1000 / 3)

#define NUM_SPI_FLASHES 4
#define NUM_QSPI_FLASHES 2
#define NUM_QSPI_BUSSES 2

#define FLASH_SIZE (64 * 1024 * 1024)
#define FLASH_SECTOR_SIZE (128 * 1024)

#define IRQ_OFFSET 32 /* pic interrupts start from index 32 */

#define MPCORE_PERIPHBASE 0xF8F00000
#define COSMOS_PLUS_BOARD_MIDR 0x413FC090

static const int dma_irqs[8] = {
    46, 47, 48, 49, 72, 73, 74, 75
};

#define BOARD_SETUP_ADDR        0x100

#define SLCR_LOCK_OFFSET        0x004
#define SLCR_UNLOCK_OFFSET      0x008
#define SLCR_ARM_PLL_OFFSET     0x100

#define SLCR_XILINX_UNLOCK_KEY  0xdf0d
#define SLCR_XILINX_LOCK_KEY    0x767b

#define COSMOS_PLUS_SDHCI_CAPABILITIES 0x69ec0080  /* Datasheet: UG585 (v1.12.1) */

#define ARMV7_IMM16(x) (extract32((x),  0, 12) | \
                        extract32((x), 12,  4) << 16)

/* Write immediate val to address r0 + addr. r0 should contain base offset
 * of the SLCR block. Clobbers r1.
 */

#define SLCR_WRITE(addr, val) \
    0xe3001000 + ARMV7_IMM16(extract32((val),  0, 16)), /* movw r1 ... */ \
    0xe3401000 + ARMV7_IMM16(extract32((val), 16, 16)), /* movt r1 ... */ \
    0xe5801000 + (addr)

typedef struct CosmosPlusMachineState {
    MachineState parent;
    Clock *ps_clk;
} CosmosPlusMachineState;

static void cosmos_plus_write_board_setup(ARMCPU *cpu,
                                   const struct arm_boot_info *info)
{
    int n;
    uint32_t board_setup_blob[] = {
        0xe3a004f8, /* mov r0, #0xf8000000 */
        SLCR_WRITE(SLCR_UNLOCK_OFFSET, SLCR_XILINX_UNLOCK_KEY),
        SLCR_WRITE(SLCR_ARM_PLL_OFFSET, 0x00014008),
        SLCR_WRITE(SLCR_LOCK_OFFSET, SLCR_XILINX_LOCK_KEY),
        0xe12fff1e, /* bx lr */
    };
    for (n = 0; n < ARRAY_SIZE(board_setup_blob); n++) {
        board_setup_blob[n] = tswap32(board_setup_blob[n]);
    }
    rom_add_blob_fixed("board-setup", board_setup_blob,
                       sizeof(board_setup_blob), BOARD_SETUP_ADDR);
}

static struct arm_boot_info cosmos_plus_binfo = {};

static void gem_init(NICInfo *nd, uint32_t base, qemu_irq irq)
{
    DeviceState *dev;
    SysBusDevice *s;

    dev = qdev_new(TYPE_CADENCE_GEM);
    if (nd->used) {
        qemu_check_nic_model(nd, TYPE_CADENCE_GEM);
        qdev_set_nic_properties(dev, nd);
    }
    s = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, base);
    sysbus_connect_irq(s, 0, irq);
}

static inline void cosmos_plus_init_spi_flashes(uint32_t base_addr, qemu_irq irq,
                                         bool is_qspi)
{
    DeviceState *dev;
    SysBusDevice *busdev;
    SSIBus *spi;
    DeviceState *flash_dev;
    int i, j;
    int num_busses =  is_qspi ? NUM_QSPI_BUSSES : 1;
    int num_ss = is_qspi ? NUM_QSPI_FLASHES : NUM_SPI_FLASHES;

    dev = qdev_new(is_qspi ? "xlnx.ps7-qspi" : "xlnx.ps7-spi");
    qdev_prop_set_uint8(dev, "num-txrx-bytes", is_qspi ? 4 : 1);
    qdev_prop_set_uint8(dev, "num-ss-bits", num_ss);
    qdev_prop_set_uint8(dev, "num-busses", num_busses);
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(busdev, &error_fatal);
    sysbus_mmio_map(busdev, 0, base_addr);
    if (is_qspi) {
        sysbus_mmio_map(busdev, 1, 0xFC000000);
    }
    sysbus_connect_irq(busdev, 0, irq);

    for (i = 0; i < num_busses; ++i) {
        char bus_name[16];
        qemu_irq cs_line;

        snprintf(bus_name, 16, "spi%d", i);
        spi = (SSIBus *)qdev_get_child_bus(dev, bus_name);

        for (j = 0; j < num_ss; ++j) {
            DriveInfo *dinfo = drive_get_next(IF_MTD);
            flash_dev = qdev_new("n25q128");
            if (dinfo) {
                qdev_prop_set_drive_err(flash_dev, "drive",
                                        blk_by_legacy_dinfo(dinfo),
                                        &error_fatal);
            }
            qdev_realize_and_unref(flash_dev, BUS(spi), &error_fatal);

            cs_line = qdev_get_gpio_in_named(flash_dev, SSI_GPIO_CS, 0);
            sysbus_connect_irq(busdev, i * num_ss + j + 1, cs_line);
        }
    }

}

static void cosmos_plus_init(MachineState *machine)
{
    CosmosPlusMachineState *cosmos_plus_machine = COSMOS_PLUS(machine);
    ARMCPU *cpu;
    MemoryRegion *address_space_mem = get_system_memory();
    MemoryRegion *ocm_ram0 = g_new(MemoryRegion, 1);
    MemoryRegion *ocm_ram1 = g_new(MemoryRegion, 1);
    DeviceState *dev, *slcr;
    SysBusDevice *busdev;
    qemu_irq pic[64];
    int n;

    /* max 2GB ram */
    if (machine->ram_size > 2 * GiB) {
        error_report("RAM size more than 2 GiB is not supported");
        exit(EXIT_FAILURE);
    }

    cpu = ARM_CPU(object_new(machine->cpu_type));

    /* By default A9 CPUs have EL3 enabled.  This board does not
     * currently support EL3 so the CPU EL3 property is disabled before
     * realization.
     */
    if (object_property_find(OBJECT(cpu), "has_el3", NULL)) {
        object_property_set_bool(OBJECT(cpu), "has_el3", false, &error_fatal);
    }

    object_property_set_int(OBJECT(cpu), "midr", COSMOS_PLUS_BOARD_MIDR,
                            &error_fatal);
    object_property_set_int(OBJECT(cpu), "reset-cbar", MPCORE_PERIPHBASE,
                            &error_fatal);
    qdev_realize(DEVICE(cpu), NULL, &error_fatal);

    /* DDR remapped to address zero.  */
    memory_region_add_subregion(address_space_mem, 0x100000, machine->ram);

    /* on-chip memory */
    memory_region_init_ram(ocm_ram0, NULL, "cosmos_plus.ocm_ram0", 0x00030000,
                           &error_fatal);
    memory_region_add_subregion(address_space_mem, 0, ocm_ram0);

    memory_region_init_ram(ocm_ram1, NULL, "cosmos_plus.ocm_ram1", 0x0000FE00,
                           &error_fatal);
    memory_region_add_subregion(address_space_mem, 0xFFFF0000, ocm_ram1);

    DriveInfo *dinfo = drive_get(IF_PFLASH, 0, 0);

    /* AMD */
    pflash_cfi02_register(0xe2000000, "cosmos_plus.pflash", FLASH_SIZE,
                          dinfo ? blk_by_legacy_dinfo(dinfo) : NULL,
                          FLASH_SECTOR_SIZE, 1,
                          1, 0x0066, 0x0022, 0x0000, 0x0000, 0x0555, 0x2aa,
                          0);

    /* Create slcr, keep a pointer to connect clocks */
    slcr = qdev_new("xilinx,zynq_slcr");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(slcr), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(slcr), 0, 0xF8000000);

    /* Create the main clock source, and feed slcr with it */
    cosmos_plus_machine->ps_clk = CLOCK(object_new(TYPE_CLOCK));
    object_property_add_child(OBJECT(cosmos_plus_machine), "ps_clk",
                              OBJECT(cosmos_plus_machine->ps_clk));
    object_unref(OBJECT(cosmos_plus_machine->ps_clk));
    clock_set_hz(cosmos_plus_machine->ps_clk, PS_CLK_FREQUENCY);
    qdev_connect_clock_in(slcr, "ps_clk", cosmos_plus_machine->ps_clk);

    dev = qdev_new(TYPE_A9MPCORE_PRIV);
    qdev_prop_set_uint32(dev, "num-cpu", 1);
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(busdev, &error_fatal);
    sysbus_mmio_map(busdev, 0, MPCORE_PERIPHBASE);
    sysbus_connect_irq(busdev, 0,
                       qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_IRQ));

    for (n = 0; n < 64; n++) {
        pic[n] = qdev_get_gpio_in(dev, n);
    }

    cosmos_plus_init_spi_flashes(0xE0006000, pic[58-IRQ_OFFSET], false);
    cosmos_plus_init_spi_flashes(0xE0007000, pic[81-IRQ_OFFSET], false);
    cosmos_plus_init_spi_flashes(0xE000D000, pic[51-IRQ_OFFSET], true);

    sysbus_create_simple(TYPE_CHIPIDEA, 0xE0002000, pic[53 - IRQ_OFFSET]);
    sysbus_create_simple(TYPE_CHIPIDEA, 0xE0003000, pic[76 - IRQ_OFFSET]);

    // dev = cadence_uart_create(0xE0000000, pic[59 - IRQ_OFFSET], serial_hd(0));
    // qdev_connect_clock_in(dev, "refclk",
    //                       qdev_get_clock_out(slcr, "uart0_ref_clk"));
    dev = cadence_uart_create(0xE0001000, pic[82 - IRQ_OFFSET], serial_hd(0));
    // qdev_connect_clock_in(dev, "refclk",
    //                       qdev_get_clock_out(slcr, "uart1_ref_clk"));

    sysbus_create_varargs("cadence_ttc", 0xF8001000,
            pic[42-IRQ_OFFSET], pic[43-IRQ_OFFSET], pic[44-IRQ_OFFSET], NULL);
    sysbus_create_varargs("cadence_ttc", 0xF8002000,
            pic[69-IRQ_OFFSET], pic[70-IRQ_OFFSET], pic[71-IRQ_OFFSET], NULL);

    gem_init(&nd_table[0], 0xE000B000, pic[54-IRQ_OFFSET]);
    gem_init(&nd_table[1], 0xE000C000, pic[77-IRQ_OFFSET]);

    for (n = 0; n < 2; n++) {
        int hci_irq = n ? 79 : 56;
        hwaddr hci_addr = n ? 0xE0101000 : 0xE0100000;
        DriveInfo *di;
        BlockBackend *blk;
        DeviceState *carddev;

        /* Compatible with:
         * - SD Host Controller Specification Version 2.0 Part A2
         * - SDIO Specification Version 2.0
         * - MMC Specification Version 3.31
         */
        dev = qdev_new(TYPE_SYSBUS_SDHCI);
        qdev_prop_set_uint8(dev, "sd-spec-version", 2);
        qdev_prop_set_uint64(dev, "capareg", COSMOS_PLUS_SDHCI_CAPABILITIES);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, hci_addr);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[hci_irq - IRQ_OFFSET]);

        di = drive_get_next(IF_SD);
        blk = di ? blk_by_legacy_dinfo(di) : NULL;
        carddev = qdev_new(TYPE_SD_CARD);
        qdev_prop_set_drive_err(carddev, "drive", blk, &error_fatal);
        qdev_realize_and_unref(carddev, qdev_get_child_bus(dev, "sd-bus"),
                               &error_fatal);
    }

    dev = qdev_new("pl330");
    qdev_prop_set_uint8(dev, "num_chnls",  8);
    qdev_prop_set_uint8(dev, "num_periph_req",  4);
    qdev_prop_set_uint8(dev, "num_events",  16);

    qdev_prop_set_uint8(dev, "data_width",  64);
    qdev_prop_set_uint8(dev, "wr_cap",  8);
    qdev_prop_set_uint8(dev, "wr_q_dep",  16);
    qdev_prop_set_uint8(dev, "rd_cap",  8);
    qdev_prop_set_uint8(dev, "rd_q_dep",  16);
    qdev_prop_set_uint16(dev, "data_buffer_dep",  256);

    busdev = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(busdev, &error_fatal);
    sysbus_mmio_map(busdev, 0, 0xF8003000);
    sysbus_connect_irq(busdev, 0, pic[45-IRQ_OFFSET]); /* abort irq line */
    for (n = 0; n < ARRAY_SIZE(dma_irqs); ++n) { /* event irqs */
        sysbus_connect_irq(busdev, n + 1, pic[dma_irqs[n] - IRQ_OFFSET]);
    }

    dev = qdev_new("xlnx.ps7-dev-cfg");
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(busdev, &error_fatal);
    sysbus_connect_irq(busdev, 0, pic[40 - IRQ_OFFSET]);
    sysbus_mmio_map(busdev, 0, 0xF8007000);

    cosmos_plus_binfo.ram_size = machine->ram_size;
    cosmos_plus_binfo.nb_cpus = 1;
    cosmos_plus_binfo.board_id = 0xd32;
    cosmos_plus_binfo.loader_start = 0;
    cosmos_plus_binfo.board_setup_addr = BOARD_SETUP_ADDR;
    cosmos_plus_binfo.write_board_setup = cosmos_plus_write_board_setup;

    for (n = 0; n < 8; n++) {   // FIXME: using board info
        DriveInfo *dinfo = drive_get_next(IF_NONE);

        dev = qdev_new("tiger4fmc");
        busdev = SYS_BUS_DEVICE(dev);

        if (dinfo) {
            // qemu_opts_print(dinfo->opts, " ");
            qdev_prop_set_drive_err(dev, "drive", blk_by_legacy_dinfo(dinfo), &error_fatal);
        }

        qdev_prop_set_uint8(dev, "fmc-id", n);

        sysbus_realize_and_unref(busdev, &error_fatal);
        sysbus_mmio_map(busdev, 0, 0x43C00000 + (n * 0x10000));
    }

    dev = qdev_new("tiger4nvme");
    // object_property_set_str(OBJECT(dev),
    //                         "mem0", "hostmem", &error_fatal);
    qdev_prop_set_chr(dev, "chardev", serial_hd(1));
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(busdev, &error_fatal);
    sysbus_mmio_map(busdev, 0, 0x83C00000);
    sysbus_connect_irq(busdev, 0, pic[61 - IRQ_OFFSET]);

    arm_load_kernel(ARM_CPU(first_cpu), machine, &cosmos_plus_binfo);
}

static void cosmos_plus_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    mc->desc = "Cosmos Plus";
    mc->init = cosmos_plus_init;
    mc->max_cpus = 1;
    mc->no_sdcard = 1;
    mc->ignore_memory_transaction_failures = true;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a9");
    mc->default_ram_id = "cosmos_plus.ext_ram";
    mc->no_cdrom = 1;
    mc->no_sdcard = 1;
    mc->no_floppy = 1;
}

static const TypeInfo cosmos_plus_machine_type = {
    .name = TYPE_COSMOS_PLUS,
    .parent = TYPE_MACHINE,
    .class_init = cosmos_plus_machine_class_init,
    .instance_size = sizeof(CosmosPlusMachineState),
};

static void cosmos_plus_machine_register_types(void)
{
    type_register_static(&cosmos_plus_machine_type);
}

type_init(cosmos_plus_machine_register_types)
