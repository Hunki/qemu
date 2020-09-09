/*
 * QEMU NVM Express Controller over Character Device
 *
 * Written by Hunki Kwon <kwonhunki@gmail.com>
 *
 * This code is licensed under the GNU GPL v2 or later.
 */

/**
 * Reference Specs: http://www.nvmexpress.org, 1.4
 *
 *  http://www.nvmexpress.org/resources/
 */

/**
 * Usage: add options:
 *      -drive file=<file>,if=none,id=<drive_id>
 *      -device nvme-oc,hostmem=<memory-backend-file>, \
 *              chardev=<character-device>
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "hw/pci/msix.h"
#include "hw/pci/pci.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "sysemu/sysemu.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "exec/memory.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/cutils.h"
#include "trace.h"
#include "chardev/char-fe.h"
#include "block/nvme.h"
#include "hw/nvme/vpci.h"

#define TYPE_NVME_vPCI "nvme-vpci"
#define NVME_vPCI(obj) \
        OBJECT_CHECK(NvmeCtrl, (obj), TYPE_NVME_vPCI)

typedef struct NvmeCtrl {
    PCIDevice       parent_obj;
    MemoryRegion    iomem;

    NvmeBar         bar;
    CharBackend     chr;
    vPCIObj         *vpci;

    int32_t         bootindex;
    bool            msix_enabled;
} NvmeCtrl;

static uint64_t nvme_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    NvmeCtrl *n = (NvmeCtrl *)opaque;
    uint8_t *ptr = (uint8_t *)&n->bar;
    uint64_t val = 0;

    if (unlikely(addr & (sizeof(uint32_t) - 1))) {
        g_assert(0);
        /* should RAZ, fall through for now */
    } else if (unlikely(size < sizeof(uint32_t))) {
        g_assert(0);
        /* should RAZ, fall through for now */
    }

    if (addr < sizeof(NvmeBar)) {
        memcpy(&val, ptr + addr, size);
    } else {
        g_assert(0);
    }

    // printf("%-24s: 0x%08lx[%d]:0x%08lx\n", __func__, addr, size, val);

    return val;
}

static void nvme_mmio_write(void *opaque, hwaddr addr, uint64_t data,
    unsigned size)
{
    NvmeCtrl *n = (NvmeCtrl *)opaque;

    // printf("%-24s: 0x%08x[%d]:0x%08lx\n", __func__, (uint32_t)addr, size, data);
    vpci_mmio_write_FtoB(n->vpci, addr, data, size, NULL);
}

static const MemoryRegionOps nvme_mmio_ops = {
    .read = nvme_mmio_read,
    .write = nvme_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 2,
        .max_access_size = 8,
    },
};

static void nvme_vpci_mmio_update(void *opaque, vPCIMsg *pmsg)
{
    NvmeCtrl *n = (NvmeCtrl *)opaque;
    uint8_t *ptr = (uint8_t *)&n->bar;
    vPCIMsgMMIO *mmio = &pmsg->body.mmio;

    // printf("%-24s: 0x%08x[%d]:0x%08lx\n", __func__, mmio->addr, mmio->len, mmio->data);

    if (mmio->addr < sizeof(NvmeBar)) {
        memcpy(ptr + mmio->addr, &mmio->data, mmio->len);
    }
    else {
        g_assert(0);
    }
}

static void nvme_vpci_dma_read_req(void *opaque, vPCIMsg *pmsg)
{
    NvmeCtrl *n = (NvmeCtrl *)opaque;
    vPCIMsgDMA *dma = &pmsg->body.dma;
    uint8_t *data = malloc(dma->len);

    pci_dma_read(&n->parent_obj, dma->dma_addr, data, dma->len);
    vpci_dma_read_resp(n->vpci, pmsg, data);
    free(data);
}

static void nvme_vpci_dma_write_req(void *opaque, vPCIMsg *pmsg)
{
    NvmeCtrl *n = (NvmeCtrl *)opaque;
    vPCIMsgDMA *dma = &pmsg->body.dma;

    pci_dma_write(&n->parent_obj, dma->dma_addr, pmsg->data, dma->len);
    vpci_dma_write_resp(n->vpci, pmsg);
}

static void nvme_vpci_irq_req(void *opaque, vPCIMsg *pmsg)
{
    NvmeCtrl *n = (NvmeCtrl *)opaque;
    vPCIMsgIRQ *irq = &pmsg->body.irq;

    if (irq->assert)
        pci_irq_assert(&n->parent_obj);
    else
        pci_irq_deassert(&n->parent_obj);
}

static void nvme_vpci_msix_req(void *opaque, vPCIMsg *pmsg)
{
    NvmeCtrl *n = (NvmeCtrl *)opaque;
    vPCIMsgMSIX *msi = &pmsg->body.msix;

    switch (msi->type) {
        case vPCI_MSI_NOTIFY:
            msix_notify(&n->parent_obj, msi->vector);
            break;
        case vPCI_MSI_USE:
            msix_vector_use(&n->parent_obj, msi->vector);
            break;
        case vPCI_MSI_UNUSE:
            msix_vector_unuse(&n->parent_obj, msi->vector);
            break;
        default:
            assert(0);
            break;
    }
}

static void nvme_vpci_init(void *opaque)
{
    NvmeCtrl *n = (NvmeCtrl *)opaque;
    vpci_msix_enable(n->vpci, n->msix_enabled);
}

static const vPCIOps vpci_ops = {
    .irq_req = nvme_vpci_irq_req,

    .dma_fe_read_req = nvme_vpci_dma_read_req,
    .dma_fe_write_req = nvme_vpci_dma_write_req,

    .msix_req = nvme_vpci_msix_req,
    .mmio_write = nvme_vpci_mmio_update,

    .init = nvme_vpci_init,
};

static void nvme_realize(PCIDevice *pci_dev, Error **errp)
{
    NvmeCtrl *n = NVME_vPCI(pci_dev);
    uint8_t *pci_conf;

    if (!qemu_chr_fe_backend_connected(&n->chr)) {
        error_setg(errp, "NVMe-vPCI requires chardev attribute");
        return;
    }

    pci_conf = pci_dev->config;
    pci_conf[PCI_INTERRUPT_PIN] = 1;
    pci_config_set_prog_interface(pci_dev->config, 0x2);
    pci_config_set_class(pci_dev->config, PCI_CLASS_STORAGE_EXPRESS);
    pcie_endpoint_cap_init(pci_dev, 0x80);

    memory_region_init_io(&n->iomem, OBJECT(n), &nvme_mmio_ops, n,
                          "nvme-vpci", pow2ceil(0x1000 + 2 * (64 + 1) * 4));
    pci_register_bar(pci_dev, 0,
        PCI_BASE_ADDRESS_SPACE_MEMORY | PCI_BASE_ADDRESS_MEM_TYPE_64,
        &n->iomem);

    if (msix_init_exclusive_bar(pci_dev, 64, 4, NULL) == 0)
        n->msix_enabled = true;
    else
        n->msix_enabled = false;

    n->vpci = vPCI(object_new(TYPE_VPCI));
    vpci_init(n->vpci, &n->chr, &vpci_ops, n);
}

static void nvme_exit(PCIDevice *pci_dev)
{
    msix_uninit_exclusive_bar(pci_dev);
}

static Property nvme_props[] = {
    DEFINE_PROP_CHR("chardev", NvmeCtrl, chr),
    DEFINE_PROP_END_OF_LIST(),
};

static const VMStateDescription nvme_vmstate = {
    .name = "nvme-vpci",
    .unmigratable = 1,
};

static void nvme_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PCIDeviceClass *pc = PCI_DEVICE_CLASS(oc);

    pc->realize = nvme_realize;
    pc->exit = nvme_exit;
    pc->class_id = PCI_CLASS_STORAGE_EXPRESS;
    pc->vendor_id = PCI_VENDOR_ID_INTEL;
    pc->device_id = 0x5845;
    pc->revision = 2;

    set_bit(DEVICE_CATEGORY_STORAGE, dc->categories);
    dc->desc = "Non-Volatile Memory Express over Virtual PCI";
    device_class_set_props(dc, nvme_props);
    dc->vmsd = &nvme_vmstate;
}

static void nvme_instance_init(Object *obj)
{
    NvmeCtrl *n = NVME_vPCI(obj);

    device_add_bootindex_property(obj, &n->bootindex,
                                  "bootindex", "/namespace@1,0",
                                  DEVICE(obj));
}

static const TypeInfo nvme_info = {
    .name          = TYPE_NVME_vPCI,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(NvmeCtrl),
    .class_init    = nvme_class_init,
    .instance_init = nvme_instance_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_PCIE_DEVICE },
        { }
    },
};

static void nvme_register_types(void)
{
    type_register_static(&nvme_info);
}

type_init(nvme_register_types)
