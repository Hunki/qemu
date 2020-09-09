
#include "qemu/osdep.h"
#include "chardev/char-fe.h"

#include "vpci.h"

#if 0
static const char *vpci_msg_string[] = {
    [vPCI_MSG_MMIO_WRITE_FTOB  ] = "MMIO_WRITE_FTOB  ",
    [vPCI_MSG_MMIO_WRITE_BTOF  ] = "MMIO_WRITE_BTOF  ",
    [vPCI_MSG_BE_DMA_READ_REQ  ] = "BE_DMA_READ_REQ  ",
    [vPCI_MSG_BE_DMA_READ_RESP ] = "BE_DMA_READ_RESP ",
    [vPCI_MSG_BE_DMA_WRITE_REQ ] = "BE_DMA_WRITE_REQ ",
    [vPCI_MSG_BE_DMA_WRITE_RESP] = "BE_DMA_WRITE_RESP",
    [vPCI_MSG_BE_IRQ           ] = "BE_IRQ           ",
    [vPCI_MSG_MSI              ] = "MSI              ",
};

#define DBG_PRINT(fmt, ...)     printf("[vPCI : %-24s] " fmt, __func__, ##__VA_ARGS__)
#else
#define DBG_PRINT(fmt, ...)     (void)0
#endif

static void vpci_process(vPCIObj *s)
{
    DBG_PRINT("MSG Type %s\n", vpci_msg_string[s->pmsg->hdr.type]);

    switch (s->pmsg->hdr.type) {
        case vPCI_MSG_BE_IRQ:
            s->ops->irq_req(s->opaque, s->pmsg);
            break;
        case vPCI_MSG_BE_DMA_READ_REQ:
            s->ops->dma_fe_read_req(s->opaque, s->pmsg);
            break;
        case vPCI_MSG_BE_DMA_WRITE_REQ:
            s->ops->dma_fe_write_req(s->opaque, s->pmsg);
            break;

        case vPCI_MSG_BE_DMA_READ_RESP:
            s->ops->dma_be_read_resp(s->opaque, s->pmsg);
            break;
        case vPCI_MSG_BE_DMA_WRITE_RESP:
            s->ops->dma_be_write_resp(s->opaque, s->pmsg);
            break;

        case vPCI_MSG_MSI:
            s->ops->msix_req(s->opaque, s->pmsg);
            break;
        case vPCI_MSG_MMIO_WRITE_FTOB:
        case vPCI_MSG_MMIO_WRITE_BTOF:
            DBG_PRINT("%s: 0x%lx 0x%lx\n", vpci_msg_string[s->pmsg->hdr.type],
                s->pmsg->body.mmio.addr, s->pmsg->body.mmio.data);
            s->ops->mmio_write(s->opaque, s->pmsg);
            break;

        default:
            assert(0);
            break;
    }
}

static int can_receive(void *opaque)
{
    vPCIObj *s = (vPCIObj *)opaque;
    return s->msg_recv_size - s->msg_recv_pos;
}

static void receive(void *opaque, const uint8_t *buf, int size)
{
    vPCIObj *s = (vPCIObj *)opaque;
    vPCIMsg *tmp_msg;
    uint8_t *ptr;

    switch (s->msg_recv_state) {
        case vPCI_MSG_RECV_MSG:
            assert(size == sizeof(vPCIMsg));
            tmp_msg = (vPCIMsg *)buf;
            s->pmsg = malloc(tmp_msg->hdr.size);
            ptr = (uint8_t *)s->pmsg;
            memcpy(ptr, buf, sizeof(vPCIMsg));
            s->msg_recv_pos += size;
            if (tmp_msg->hdr.size > sizeof(vPCIMsg)) {
                s->msg_recv_size = tmp_msg->hdr.size;
                s->msg_recv_state = vPCI_MSG_RECV_DATA;
                return;
            }

            break;

        case vPCI_MSG_RECV_DATA:
            ptr = (uint8_t *)s->pmsg;
            memcpy(ptr + s->msg_recv_pos, buf, size);
            s->msg_recv_pos += size;
            if (s->msg_recv_size > s->msg_recv_pos)
                return;

            break;

        default:
            assert(0);
            break;
    }

    assert(s->msg_recv_size == s->msg_recv_pos);
    vpci_process(s);

    free(s->pmsg);
    s->msg_recv_state = vPCI_MSG_RECV_MSG;
    s->msg_recv_size = sizeof(vPCIMsg);
    s->msg_recv_pos = 0;
}

static void vpci_mmio_write(vPCIObj *s, uint32_t addr, uint64_t value, uint8_t size, vPCI_MSG_TYPES type, void *param)
{
    int ret;
    vPCIMsg msg;

    assert(size <= sizeof(uint64_t));

    msg.hdr.type = type;
    msg.hdr.size = sizeof(vPCIMsg);

    msg.body.mmio.addr = addr;
    msg.body.mmio.data = value;
    msg.body.mmio.len = size;

    ret = qemu_chr_fe_write_all((CharBackend *)s->chr, (const uint8_t *)&msg, sizeof(vPCIMsg));
    assert(ret == sizeof(vPCIMsg));
}

void vpci_mmio_write_FtoB(vPCIObj *s, uint32_t addr, uint64_t value, uint8_t size, void *param)
{
    vpci_mmio_write(s, addr, value, size, vPCI_MSG_MMIO_WRITE_FTOB, param);
}

void vpci_mmio_write_BtoF(vPCIObj *s, uint32_t addr, uint64_t value, uint8_t size, void *param)
{
    vpci_mmio_write(s, addr, value, size, vPCI_MSG_MMIO_WRITE_BTOF, param);
}

void vpci_dma_write_req(vPCIObj *s, uint64_t dma_addr, void *buf, uint32_t len, void *param)
{
    int ret;
    vPCIMsg msg;

    msg.hdr.type = vPCI_MSG_BE_DMA_WRITE_REQ;
    msg.hdr.size = sizeof(vPCIMsg) + len;

    msg.body.dma.param = (uint64_t)param;
    msg.body.dma.dma_addr = dma_addr;
    msg.body.dma.buf_addr = 0;
    msg.body.dma.len = len;

    ret = qemu_chr_fe_write_all((CharBackend *)s->chr, (const uint8_t *)&msg, sizeof(vPCIMsg));
    assert(ret == sizeof(vPCIMsg));

    ret = qemu_chr_fe_write_all((CharBackend *)s->chr, (const uint8_t *)buf, len);
    assert(ret == len);
}

void vpci_dma_write_resp(vPCIObj *s, vPCIMsg *pmsg)
{
    int ret;

    pmsg->hdr.type = vPCI_MSG_BE_DMA_WRITE_RESP;
    pmsg->hdr.size = sizeof(vPCIMsg);

    ret = qemu_chr_fe_write_all((CharBackend *)s->chr, (const uint8_t *)pmsg, sizeof(vPCIMsg));
    assert(ret == sizeof(vPCIMsg));
}

void vpci_dma_read_req(vPCIObj *s, uint64_t dma_addr, void *buf_addr, uint32_t len, void *param)
{
    int ret;
    vPCIMsg msg;

    msg.hdr.type = vPCI_MSG_BE_DMA_READ_REQ;
    msg.hdr.size = sizeof(vPCIMsg);

    msg.body.dma.param = (uint64_t)param;
    msg.body.dma.dma_addr = dma_addr;
    msg.body.dma.buf_addr = (uint64_t)buf_addr;
    msg.body.dma.len = len;

    ret = qemu_chr_fe_write_all((CharBackend *)s->chr, (const uint8_t *)&msg, sizeof(vPCIMsg));
    assert(ret == sizeof(vPCIMsg));
}

void vpci_dma_read_resp(vPCIObj *s, vPCIMsg *pmsg, void *buf)
{
    int ret;

    pmsg->hdr.type = vPCI_MSG_BE_DMA_READ_RESP;
    pmsg->hdr.size = sizeof(vPCIMsg) + pmsg->body.dma.len;

    ret = qemu_chr_fe_write_all((CharBackend *)s->chr, (const uint8_t *)pmsg, sizeof(vPCIMsg));
    assert(ret == sizeof(vPCIMsg));

    ret = qemu_chr_fe_write_all((CharBackend *)s->chr, (const uint8_t *)buf, pmsg->body.dma.len);
    assert(ret == pmsg->body.dma.len);
}

static void vpci_msix(vPCIObj *s, vPCI_MSI_TYPES type, uint32_t vector)
{
    int ret;
    vPCIMsg msg;

    msg.hdr.type = vPCI_MSG_MSI;
    msg.hdr.size = sizeof(vPCIMsg);

    msg.body.msix.type = type;
    msg.body.msix.vector = vector;

    ret = qemu_chr_fe_write_all((CharBackend *)s->chr, (const uint8_t *)&msg, sizeof(vPCIMsg));
    assert(ret == sizeof(vPCIMsg));
}

void vpci_msix_enable(vPCIObj *s, bool enable)
{
    vpci_msix(s, enable ? vPCI_MSI_ENABLE : vPCI_MSI_DISABLE, 0xFFFFFFFF);
}

void vpci_msix_notify(vPCIObj *s, uint32_t vector)
{
    vpci_msix(s, vPCI_MSI_NOTIFY, vector);
}

void vpci_msix_use(vPCIObj *s, uint32_t vector)
{
    vpci_msix(s, vPCI_MSI_USE, vector);
}

void vpci_msix_unuse(vPCIObj *s, uint32_t vector)
{
    vpci_msix(s, vPCI_MSI_UNUSE, vector);
}

static void vpci_irq(vPCIObj *s, bool assert)
{
    int ret;
    vPCIMsg msg;

    msg.hdr.type = vPCI_MSG_BE_IRQ;
    msg.hdr.size = sizeof(vPCIMsg);

    msg.body.irq.assert = assert;

    ret = qemu_chr_fe_write_all((CharBackend *)s->chr, (const uint8_t *)&msg, sizeof(vPCIMsg));
    assert(ret == sizeof(vPCIMsg));
}

void vpci_irq_assert(vPCIObj *s)
{
    vpci_irq(s, true);
}

void vpci_irq_deassert(vPCIObj *s)
{
    vpci_irq(s, true);
}

static void chr_event(void *opaque, QEMUChrEvent event)
{
    vPCIObj *s = (vPCIObj *)opaque;

    switch (event) {
    case CHR_EVENT_OPENED:
        s->connected = true;
        s->ops->init(s->opaque);
        break;

    case CHR_EVENT_CLOSED:
        if (!s->connected) {
            return;
        }
        s->connected = false;
        break;

    case CHR_EVENT_BREAK:
    case CHR_EVENT_MUX_IN:
    case CHR_EVENT_MUX_OUT:
        /* Ignore */
        break;
    }
}

void vpci_init(vPCIObj *s, const CharBackend *chr, const vPCIOps *ops, void *opaque)
{
    s->chr = chr;
    s->ops = ops;
    s->opaque = opaque;

    qemu_chr_fe_set_handlers((CharBackend *)s->chr, can_receive, receive,
            chr_event, NULL, s, NULL, true);

    s->msg_recv_state = vPCI_MSG_RECV_MSG;
    s->msg_recv_size = sizeof(vPCIMsg);
    s->msg_recv_pos = 0;
}

static void vpci_initfn(Object *obj)
{
    // vPCIObj *s = vPCI(obj);
}

static void vpci_finalizefn(Object *obj)
{
    // vPCIObj *s = vPCI(obj);
}

static const TypeInfo vpci_info = {
    .name              = TYPE_VPCI,
    .parent            = TYPE_OBJECT,
    .instance_size     = sizeof(vPCIObj),
    .instance_init     = vpci_initfn,
    .instance_finalize = vpci_finalizefn,
};

static void vpci_register_type(void)
{
    type_register_static(&vpci_info);
}

type_init(vpci_register_type)
