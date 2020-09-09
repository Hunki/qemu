#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "qemu/timer.h"
#include "chardev/char-fe.h"
#include "sysemu/hostmem.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "qemu/fifo32.h"

#include "tiger4nvme.h"

#if 0
#define DBG_PRINT(fmt, ...)     printf("[NVME : %-24s] " fmt, __func__, ##__VA_ARGS__)
#else
#define DBG_PRINT(fmt, ...)     (void)0
#endif

#define D2H_TYPE_CQE        1
#define D2H_TYPE_DMA        0

#define H2D_TYPE_NVME       1
#define H2D_TYPE_PRP        0

typedef union vPCI_PARAM {
    uint32_t dw;
    struct {
        uint32_t    type    : 1;
        uint32_t    rsvd0   : 7;

        uint32_t    rsvd1   : 1;
        uint32_t    slot    : 7;

        uint32_t    qid     : 16;
    };
} vPCI_PARAM;

// static uint32_t readl(void *ptr, uint32_t addr)
// {
//     return *(uint32_t *)(((uint8_t *)ptr) + addr);
// }

static void writel(void *ptr, uint32_t addr, uint32_t val)
{
    *(uint32_t *)(((uint8_t *)ptr) + addr) = val;
}

static void irq_check(TIGER4NVMeState *n)
{
    if (n->msix_enabled) {
        return;
    }

    if (~n->bar.intms & n->pci_irq_status)
        vpci_irq_assert(n->vpci);
    else
        vpci_irq_deassert(n->vpci);
}

static void irq_assert(TIGER4NVMeState *n, uint16_t cqid)
{
    NvmeCQueue *cq = &n->cq[cqid];

    if (cq->irq_enabled) {
        if (n->msix_enabled) {
            vpci_msix_notify(n->vpci, cq->vector);
        } else {
            // g_assert(cq->cqid < 32);
            n->pci_irq_status |= 1 << cq->vector;
            irq_check(n);
        }
    }
}

static void irq_deassert(TIGER4NVMeState *n, uint16_t cqid)
{
    NvmeCQueue *cq = &n->cq[cqid];

    if (cq->irq_enabled) {
        if (n->msix_enabled) {
            return;
        } else {
            // assert(cq->vector < 32);
            n->pci_irq_status &= ~(1 << cq->vector);
            irq_check(n);
        }
    }
}

static uint8_t search_invalid_slot(TIGER4NVMeState *n)
{
    uint8_t i;

    for (i = 0; i < T4NVME_MAX_CMD; i++, n->last_slot++) {
        n->last_slot %= T4NVME_MAX_CMD;
        if (n->hwi_cmd[n->last_slot].valid == false)
            return n->last_slot++;
    }

    // DBG_PRINT("no invalid slot\n");
    return 0xFF;
}

static void cmd_fifo_req(void *opaque)
{
    TIGER4NVMeState *n = (TIGER4NVMeState *)opaque;
    uint16_t sqe_size = 1 << NVME_CC_IOSQES(n->bar.cc);

    while (1) {
        uint32_t sq_used = 0;

        for (uint32_t qid = 0; qid < T4NVME_MAX_Q + 1; qid++) {
            NvmeSQueue *sq = &n->sq[qid];

            while (sq->head != sq->tail) {
                uint8_t slot = search_invalid_slot(n);
                if (slot == 0xFF)
                    goto timer_exit;

                uint64_t prp = sq->dma_addr + sq->head * sqe_size;
                sq->head = (sq->head + 1) % sq->size;

                // DBG_PRINT("sqid %d new_head %d:%d\n", qid, sq->head, sq->tail);

                vPCI_PARAM p;
                p.dw = 0;
                p.type = H2D_TYPE_NVME;
                p.qid = qid;
                p.slot = slot;

                NVME_CMD_HW_INFO *hwi = &n->hwi_cmd[slot];
                assert(hwi->valid == false);
                hwi->valid = true;

                vpci_dma_read_req(n->vpci, prp, &n->sram_cmd[p.slot],
                                sizeof(NvmeCmd), (void *)(uint64_t)p.dw);
                sq_used++;
            }
        }

        if (sq_used == 0)
            break;
    }

    return;

timer_exit:
    timer_mod(n->sq_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 500);
}

static void send_cqe(TIGER4NVMeState *n, uint16_t slot, uint16_t status, uint32_t result)
{
    vPCI_PARAM p;
    NvmeCqe cqe;
    uint16_t cqe_size = 1 << NVME_CC_IOCQES(n->bar.cc);
    uint16_t sqid = n->hwi_cmd[slot].qid;

    memset(&cqe, 0, sizeof(NvmeCqe));

    NvmeSQueue *sq = &n->sq[sqid];
    NvmeCQueue *cq = &n->cq[sq->cqid];

    cqe.sq_head = sq->head;
    cqe.sq_id = sqid;
    cqe.cid = n->sram_cmd[slot].cid;
    cqe.status = status | cq->phase ? 1 : 0;
    cqe.result = result;

    uint64_t prp = cq->dma_addr + cq->tail * cqe_size;

    cq->tail++;
    if (cq->tail >= cq->size) {
        cq->tail = 0;
        cq->phase = !cq->phase;
    }

    // DBG_PRINT("cqid %d new_tail %d:%d\n", sq->cqid, cq->head, cq->tail);

    p.dw = 0;
    p.type = D2H_TYPE_CQE;
    p.qid = sq->cqid;
    p.slot = slot;
    vpci_dma_write_req(n->vpci, prp, (void *)&cqe,
                    sizeof(NvmeCqe), (void *)(uint64_t)p.dw);
}

static void process_cpl_fifo(TIGER4NVMeState *n)
{
    switch (n->cpl_fifo.cplType) {
        case ONLY_CPL_TYPE:
            DBG_PRINT("ONLY_CPL_TYPE\n");
            assert(0);
            break;
        case AUTO_CPL_TYPE:
            // DBG_PRINT("AUTO_CPL_TYPE\n");
            break;
        case CMD_SLOT_RELEASE_TYPE:
            DBG_PRINT("CMD_SLOT_RELEASE_TYPE\n");
            assert(0);
            break;
    }

    send_cqe(n, n->cpl_fifo.cmdSlotTag, \
                n->cpl_fifo.statusFieldWord, n->cpl_fifo.specific);
}

static void direct_dma(TIGER4NVMeState *n, HOST_DMA_CMD_FIFO_REG *d)
{
    if (d->dmaDirection == HOST_DMA_TX_DIRECTION) {
        // D2H
        uint8_t *buf;
        uint64_t prp = d->pcieAddrL | (uint64_t)d->pcieAddrH << 32;
        vPCI_PARAM p;

        p.dw = 0;
        p.type = D2H_TYPE_DMA;

        buf = malloc(d->dmaLen);
        cpu_physical_memory_read(d->devAddr, buf, d->dmaLen);
        vpci_dma_write_req(n->vpci, prp, buf,
                    d->dmaLen, (void *)(uint64_t)p.dw);
        free(buf);
    }
    else {
        // H2D
        assert(0);
    }
}

static void process_dma(TIGER4NVMeState *n)
{
    DMA_INFO *dma = &n->dma_info;
    HOST_DMA_CMD_FIFO_REG *d;

    if (dma->state == DS_IDLE) {
        if (fifo8_is_empty(&n->dma_fifo))
            return;

        uint32_t num;
        dma->cmd = (HOST_DMA_CMD_FIFO_REG *)
                    fifo8_pop_buf(&n->dma_fifo, sizeof(HOST_DMA_CMD_FIFO_REG), &num);
        assert(num == sizeof(HOST_DMA_CMD_FIFO_REG));
        dma->state = DS_PRP_START;
    }

    d = dma->cmd;

    if (d->dmaType == HOST_DMA_DIRECT_TYPE) {
        direct_dma(n, d);
        dma->state = DS_DIRECT_WAIT;
        return;
    }

    NVME_CMD_HW_INFO *hwi = &n->hwi_cmd[d->cmdSlotTag];
    NvmeRwCmd *rw = (NvmeRwCmd *)&n->sram_cmd[d->cmdSlotTag];

    if (hwi->prp_list == NULL && rw->nlb > 1) {
        uint32_t nents = (rw->nlb * n->page_size) >> n->page_bits;
        uint32_t prp_trans = nents * sizeof(uint64_t);

        // DBG_PRINT("slot %d PRP2_LIST_GET\n", d->cmdSlotTag);

        hwi->prp_list = malloc(prp_trans);
        vpci_dma_read_req(n->vpci, rw->prp2,
                        hwi->prp_list, prp_trans, (void *)0);
        dma->state = DS_PRP2_LIST_GET;
    }
    else {
        if (d->cmd4KBOffset == 0) {
            if (d->dmaDirection == HOST_DMA_TX_DIRECTION) {
                cpu_physical_memory_read(d->devAddr, dma->buf, n->page_size);
                vpci_dma_write_req(n->vpci, rw->prp1,
                                dma->buf, n->page_size, (void *)0);
            }
            else {
                vpci_dma_read_req(n->vpci, rw->prp1,
                                dma->buf, n->page_size, (void *)0);
            }
        }
        else {
            assert(rw->nlb > 0);
            assert(d->cmd4KBOffset <= rw->nlb);
            if (rw->nlb > 1) {
                uint64_t prp_ent = hwi->prp_list[d->cmd4KBOffset - 1];
                if (d->dmaDirection == HOST_DMA_TX_DIRECTION) {
                    cpu_physical_memory_read(d->devAddr, dma->buf, n->page_size);
                    vpci_dma_write_req(n->vpci, prp_ent,
                                    dma->buf, n->page_size, (void *)0);
                }
                else {
                    vpci_dma_read_req(n->vpci, prp_ent,
                                    dma->buf, n->page_size, (void *)0);
                }
            }
            else {
                if (d->dmaDirection == HOST_DMA_TX_DIRECTION) {
                    cpu_physical_memory_read(d->devAddr, dma->buf, n->page_size);
                    vpci_dma_write_req(n->vpci, rw->prp2,
                                    dma->buf, n->page_size, (void *)0);
                }
                else {
                    vpci_dma_read_req(n->vpci, rw->prp2,
                                    dma->buf, n->page_size, (void *)0);
                }
            }
        }

        assert(hwi->num > 0);
        hwi->num--;
    }

    // DBG_PRINT("slot %d off %d num %d state %d\n",
    //         d->cmdSlotTag, d->cmd4KBOffset, hwi->num, dma->state);
}

static void post_dma(TIGER4NVMeState *n)
{
    DMA_INFO *dma = &n->dma_info;
    HOST_DMA_CMD_FIFO_REG *d = dma->cmd;

    if (d->dmaType == HOST_DMA_AUTO_TYPE) {
        NvmeRwCmd *rw = (NvmeRwCmd *)&n->sram_cmd[d->cmdSlotTag];
        NVME_CMD_HW_INFO *hwi = &n->hwi_cmd[d->cmdSlotTag];

        if (dma->state == DS_PRP2_LIST_GET) {
            dma->state = DS_PRP_START;
        }
        else {
            // DBG_PRINT("[%d:%d] off %d num %d s %d\n",
            //     hwi->qid, d->cmdSlotTag, d->cmd4KBOffset, hwi->num, dma->state);

            if (d->dmaDirection == HOST_DMA_RX_DIRECTION) {
                cpu_physical_memory_write(d->devAddr, dma->buf, n->page_size);
                n->dma_fifo_cnt.autoDmaRx++;
            }
            else
                n->dma_fifo_cnt.autoDmaTx++;

            if (hwi->num == 0) {
                if (rw->nlb > 1) {
                    assert(hwi->prp_list != NULL);
                    free(hwi->prp_list);
                    hwi->prp_list = NULL;
                }

                send_cqe(n, d->cmdSlotTag, 0, 0);
            }

            dma->state = DS_IDLE;
        }
    }
    else { // HOST_DMA_DIRECT_TYPE
        assert(dma->state == DS_DIRECT_WAIT);

        if (d->dmaDirection == HOST_DMA_RX_DIRECTION) {
            assert(0);
            n->dma_fifo_cnt.directDmaRx++;
        }
        else {
            n->dma_fifo_cnt.directDmaTx++;
        }

        dma->state = DS_IDLE;
    }

    process_dma(n);
}

static void dma_cmd_push(TIGER4NVMeState *n)
{
    HOST_DMA_CMD_FIFO_REG *d = &n->dma_fifo_reg;

    // DBG_PRINT("devAddr      0x%x\n", d->devAddr);
    // DBG_PRINT("dmaType      %d\n", d->dmaType);
    // DBG_PRINT("dmaDirection %d\n", d->dmaDirection);
    // DBG_PRINT("cmd4KBOffset %d\n", d->cmd4KBOffset);
    // DBG_PRINT("cmdSlotTag   %d\n", d->cmdSlotTag);

    fifo8_push_all(&n->dma_fifo, (const uint8_t *)d, sizeof(HOST_DMA_CMD_FIFO_REG));

    if (n->dma_info.state == DS_IDLE) {
        // DBG_PRINT("** push and start***\n");
        process_dma(n);
    }
}

static void cpl_d2h(void *opaque, uint32_t param)
{
    TIGER4NVMeState *n = (TIGER4NVMeState *)opaque;
    vPCI_PARAM *p = (vPCI_PARAM *)&param;

    if (p->type == D2H_TYPE_CQE) {
        NvmeCQueue *cq = &n->cq[p->qid];

        if (cq->tail != cq->head)
            irq_assert(n, p->qid);

        NVME_CMD_HW_INFO *hwi = &n->hwi_cmd[p->slot];
        assert(hwi->valid == true);
        assert(hwi->num == 0);
        hwi->valid = false;

        DBG_PRINT("[%d:%d]\n", p->qid, p->slot);

        cmd_fifo_req(n);
    }
    else {
        post_dma(n);
    }
}

static void cpl_h2d(void *opaque, uint32_t param)
{
    TIGER4NVMeState *n = (TIGER4NVMeState *)opaque;
    NVME_CMD_FIFO_REG f;
    vPCI_PARAM *p = (vPCI_PARAM *)&param;

    if (p->type == H2D_TYPE_NVME) {
        f.dw = 0;
        f.qID = p->qid;
        f.cmdSlotTag = p->slot;
        f.cmdValid = 1;

        fifo32_push(&n->cmd_fifo, f.dw);

        NvmeCmd *ncmd = &n->sram_cmd[p->slot];
        NVME_CMD_HW_INFO *hwi = &n->hwi_cmd[p->slot];
        hwi->qid = p->qid;

        if (p->qid == 0) {
            DBG_PRINT("[%"PRIu32":%d] admin: opcode 0x%02x cid %d nsid %d prp1 0x%"PRIx64" prp2 0x%"PRIx64"\n", \
                p->qid, p->slot, ncmd->opcode, ncmd->cid, ncmd->nsid, ncmd->prp1, ncmd->prp2);
        }
        else {
            #if 1
            DBG_PRINT("[%"PRIu32":%d] io   : opcode 0x%02x cid %d nsid %d prp1 0x%"PRIx64" prp2 0x%"PRIx64"\n", \
                p->qid, p->slot, ncmd->opcode, ncmd->cid, ncmd->nsid, ncmd->prp1, ncmd->prp2);
            #endif

            switch (ncmd->opcode) {
                case NVME_CMD_WRITE:
                case NVME_CMD_READ:
                    {
                        NvmeRwCmd *rwcmd = (NvmeRwCmd *)ncmd;
                        assert(hwi->num == 0);
                        hwi->num = rwcmd->nlb + 1;
                        // DBG_PRINT("slot %d SLBA %ld NUM %d\n", p->slot, rwcmd->slba, hwi->num);
                        assert(hwi->prp_list == NULL);
                    }
                    break;
            }
        }
    }
    else {
        post_dma(n);
    }
}

static void process_db(void *opaque, uint32_t addr, uint32_t data)
{
    TIGER4NVMeState *n = (TIGER4NVMeState *)opaque;

    uint32_t qid;

    if (unlikely(addr & ((1 << 2) - 1))) {
        g_assert(0);
        return;
    }

    if (((addr) >> 2) & 1) {
        /* Completion queue doorbell write */
        // DBG_PRINT("Completion queue doorbell write\n");

        uint16_t new_head = data & 0xffff;
        // int start_sqs;

        qid = (addr - (1 << 2)) >> 3;

        NvmeCQueue *cq = &n->cq[qid];
        cq->head = new_head;

        // DBG_PRINT("cqid %d new_head %d:%d\n", qid, cq->head, cq->tail);

        if (cq->tail == cq->head)
            irq_deassert(n, qid);
    } else {
        /* Submission queue doorbell write */
        // DBG_PRINT("Submission queue doorbell write\n");

        uint16_t new_tail = data & 0xffff;

        qid = (addr) >> 3;

        NvmeSQueue *sq = &n->sq[qid];
        sq->tail = new_tail;

        // DBG_PRINT("sqid %d new_tail %d:%d\n", qid, sq->head, sq->tail);

        cmd_fifo_req(n);
        // timer_mod(n->sq_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 500);
    }
}

static void io_sq_set(TIGER4NVMeState *n, int idx)
{
    NVME_IO_SQ_SET_REG *sq_set = &n->io_sq_set[idx];
    uint16_t sq_id = idx + 1;

    NvmeSQueue *sq = &n->sq[sq_id];

    if (sq_set->valid) {
        DBG_PRINT("create: sq_id 0x%x cq_vector 0x%x size %d\n", \
                sq_id, sq_set->cqVector, sq_set->sqSize);

        sq->dma_addr = sq_set->pcieBaseAddrL | (uint64_t)sq_set->pcieBaseAddrH << 32;
        sq->cqid = sq_set->cqVector;
        sq->size = sq_set->sqSize + 1;
        sq->head = sq->tail = 0;
    }
    else {
        DBG_PRINT("delete: sq_id 0x%x\n", sq_id);
        // g_assert(0);
    }
}

static void io_cq_set(TIGER4NVMeState *n, int idx)
{
    NVME_IO_CQ_SET_REG *cq_set = &n->io_cq_set[idx];
    uint16_t cq_id = idx + 1;

    NvmeCQueue *cq = &n->cq[cq_id];

    if (cq_set->valid) {
        DBG_PRINT("create: cq_id 0x%x size %d vector %d\n", \
                    cq_id, cq_set->cqSize, cq_set->irqVector);

        cq->dma_addr = cq_set->pcieBaseAddrL | (uint64_t)cq_set->pcieBaseAddrH << 32;
        cq->vector = cq_set->irqVector;
        cq->irq_enabled = cq_set->irqEn;
        cq->size = cq_set->cqSize + 1;
        cq->head = cq->tail = 0;
        cq->phase = 1;

        vpci_msix_use(n->vpci, cq->vector);
    }
    else {
        vpci_msix_unuse(n->vpci, cq->vector);
        DBG_PRINT("delete: cq_id 0x%x\n", cq_id);
        // g_assert(0);
    }
}

static void admin_q_set(TIGER4NVMeState *n)
{
    DBG_PRINT("cqValid %d sqValid %d cqIrqEn %d\n",
                n->admin_q_set.cqValid, n->admin_q_set.sqValid, n->admin_q_set.cqIrqEn);

    NvmeCQueue *cq = &n->cq[0];
    NvmeSQueue *sq = &n->sq[0];

    if (n->admin_q_set.cqValid) {

        cq->dma_addr = n->bar.acq;
        cq->size = NVME_AQA_ACQS(n->bar.aqa) + 1;
        cq->vector = 0;
        cq->irq_enabled = n->admin_q_set.cqIrqEn;
        cq->head = cq->tail = 0;
        cq->phase = 1;

        vpci_msix_use(n->vpci, cq->vector);
    }
    else {
        vpci_msix_unuse(n->vpci, cq->vector);
    }

    if (n->admin_q_set.sqValid) {
        sq->dma_addr = n->bar.asq;
        sq->cqid = 0;
        sq->size = NVME_AQA_ASQS(n->bar.aqa) + 1;
        sq->head = sq->tail = 0;
    }
    else {

    }

    if (n->admin_q_set.cqValid && n->admin_q_set.sqValid) {
        n->page_bits = NVME_CC_MPS(n->bar.cc) + 12;
        n->page_size = 1 << n->page_bits;
        DBG_PRINT("page_size %d\n", n->page_size);
    }
}

static void process_bar(void *opaque, uint32_t addr, uint32_t data)
{
    TIGER4NVMeState *n = (TIGER4NVMeState *)opaque;

    switch (addr) {
        case 0x14:
            if (NVME_CC_EN(data) && !NVME_CC_EN(n->bar.cc)) {
                // CC.EN 0 => 1
                n->bar.cc = data;
                n->irq_status.nvmeCcEn = 1;
                n->nvme_status.ccEn = 1;
            }
            else if (!NVME_CC_EN(data) && NVME_CC_EN(n->bar.cc)) {
                // CC.EN 1 => 0
                n->bar.cc = 0;
                n->irq_status.nvmeCcEn = 1;
                n->nvme_status.ccEn = 0;
            }
            if (NVME_CC_SHN(data) && !(NVME_CC_SHN(n->bar.cc))) {
                n->irq_status.nvmeCcShn = 1;
                n->nvme_status.ccShn = 1;
                n->bar.cc = data;
            } else if (!NVME_CC_SHN(data) && NVME_CC_SHN(n->bar.cc)) {
                n->irq_status.nvmeCcShn = 1;
                n->nvme_status.ccShn = 0;
                n->bar.cc = data;
            }

            if (n->irq_status.dw)
                qemu_set_irq(n->irq, 1);

            break;
        default:
            DBG_PRINT("addr 0x%"PRIx32" data 0x%"PRIx32"\n", addr, data);
            writel(&n->bar, addr, data);
            // g_assert(0);
            break;
    }

    vpci_mmio_write_BtoF(n->vpci, addr,
            *(uint32_t *)(((uint8_t *)&n->bar) + addr), sizeof(uint32_t), NULL);
}

static void t4nvme_mmio_write(void *opaque, vPCIMsg *pmsg)
{
    vPCIMsgMMIO *mmio = &pmsg->body.mmio;

    if (mmio->addr < 0x1000)
        process_bar(opaque, mmio->addr, mmio->data);
    else
        process_db(opaque, mmio->addr - 0x1000, mmio->data);
}

static void t4nvme_msix_enable(void *opaque, vPCIMsg *pmsg)
{
    TIGER4NVMeState *n = (TIGER4NVMeState *)opaque;
    vPCIMsgMSIX *msix = &pmsg->body.msix;

    switch (msix->type) {
        case vPCI_MSI_ENABLE:
            n->msix_enabled = true;
            break;
        case vPCI_MSI_DISABLE:
            n->msix_enabled = false;
            break;
        default:
            assert(0);
            break;
    }
}

static void t4nvme_dma_read_resp(void *opaque, vPCIMsg *pmsg)
{
    vPCIMsgDMA *dma = &pmsg->body.dma;
    memcpy((void *)dma->buf_addr, pmsg->data, dma->len);
    cpl_h2d(opaque, (uint32_t)dma->param);
}

static void t4nvme_dma_write_resp(void *opaque, vPCIMsg *pmsg)
{
    vPCIMsgDMA *dma = &pmsg->body.dma;
    cpl_d2h(opaque, (uint32_t)dma->param);
}

static void t4nvme_vpci_init(void *opaque)
{
    TIGER4NVMeState *n = (TIGER4NVMeState *)opaque;

    vpci_mmio_write_BtoF(n->vpci, offsetof(NvmeBar, cap),
                        n->bar.cap, sizeof(n->bar.cap), NULL);
    vpci_mmio_write_BtoF(n->vpci, offsetof(NvmeBar, vs),
                        n->bar.vs, sizeof(n->bar.vs), NULL);
    vpci_mmio_write_BtoF(n->vpci, offsetof(NvmeBar, nssrc),
                        n->bar.nssrc, sizeof(n->bar.nssrc), NULL);
}

static const vPCIOps vpci_ops = {
    .dma_be_read_resp = t4nvme_dma_read_resp,
    .dma_be_write_resp = t4nvme_dma_write_resp,
    .msix_req = t4nvme_msix_enable,
    .mmio_write = t4nvme_mmio_write,

    .init = t4nvme_vpci_init,
};

static uint64_t t4nvme_read(void *opaque, hwaddr offset,
                           unsigned size)
{
    TIGER4NVMeState *s = (TIGER4NVMeState *)opaque;

    // DBG_PRINT("0x%"PRIx64"[0x%"PRIx32"]\n", offset, size);

    switch (offset) {
        case 0x004:
            return s->irq_mask.dw;
        case 0x00C:
            return s->irq_status.dw;
        case 0x200:
            return s->nvme_status.dw;
        case 0x204:
            return s->dma_fifo_cnt.dw;
        case 0x21c:
            return s->admin_q_set.dw;
        case 0x300:
            if (fifo32_is_empty(&s->cmd_fifo))
                return 0;
            else
                return fifo32_pop(&s->cmd_fifo);
        case 0x2000 ... (0x2000 + sizeof(NvmeCmd) * T4NVME_MAX_CMD):
            {
                uint32_t off = (offset - 0x2000);
                uint32_t *ptr = (uint32_t *)s->sram_cmd;

                return *(ptr + (off / sizeof(uint32_t)));
            }
            break;

        default:
            DBG_PRINT("0x%"PRIx64"[0x%"PRIx32"]\n", offset, size);
            break;
    }

    assert(0);
    return 0;
}

static void t4nvme_write(void *opaque, hwaddr offset,
                        uint64_t value, unsigned size)
{
    TIGER4NVMeState *s = (TIGER4NVMeState *)opaque;

    // DBG_PRINT("0x%"PRIx64"[0x%"PRIx32"] 0x%"PRIx64"\n", offset, size, value);

    switch (offset) {
        case 0x004:
            s->irq_mask.dw = (uint32_t)value;
            break;
        case 0x008:
            s->irq_status.dw &= ~value;
            if (s->irq_status.dw == 0)
                qemu_set_irq(s->irq, 0);
            break;
        case 0x200:
            {
                NVME_STATUS_REG status;
                status.dw = (uint32_t)value;

                if (!s->nvme_status.cstsRdy && status.cstsRdy) {
                    s->nvme_status.cstsRdy = 1;
                    s->bar.csts = NVME_CSTS_READY;
                }
                else if (s->nvme_status.cstsRdy && !status.cstsRdy) {
                    s->nvme_status.cstsRdy = 0;
                    s->bar.csts &= ~NVME_CSTS_READY;
                }

                if (!s->nvme_status.cstsShst && status.cstsShst) {
                    s->nvme_status.cstsShst = 1;
                    s->bar.csts |= NVME_CSTS_SHST_COMPLETE;
                }
                else if (s->nvme_status.cstsShst && !status.cstsShst) {
                    s->nvme_status.cstsShst = 0;
                    s->bar.csts &= ~NVME_CSTS_SHST_COMPLETE;
                }
                vpci_mmio_write_BtoF(s->vpci, offsetof(NvmeBar, csts),
                                    s->bar.csts, sizeof(s->bar.csts), NULL);
            }
            break;
        case 0x21C:
            s->admin_q_set.dw = (uint32_t)value;
            admin_q_set(s);
            break;
        case 0x220 ... (0x220 + sizeof(NVME_IO_SQ_SET_REG) * T4NVME_MAX_Q - 4):
            {
                uint32_t dw_off = (offset - 0x220) / sizeof(uint32_t);
                int reg_trig = sizeof(NVME_IO_SQ_SET_REG) / sizeof(uint32_t);

                writel(s->io_sq_set, offset - 0x220, value);

                if (dw_off % reg_trig == (reg_trig - 1))
                    io_sq_set(s, dw_off / reg_trig);
            }
            break;
        case 0x260 ... (0x260 + sizeof(NVME_IO_CQ_SET_REG) * T4NVME_MAX_Q - 4):
            {
                uint32_t dw_off = (offset - 0x260) / sizeof(uint32_t);
                int reg_trig = sizeof(NVME_IO_CQ_SET_REG) / sizeof(uint32_t);

                writel(s->io_cq_set, offset - 0x260, value);

                if (dw_off % reg_trig == (reg_trig - 1))
                    io_cq_set(s, dw_off / reg_trig);
            }
            break;
        case 0x304 ... 0x30C:
            s->cpl_fifo.dw[(offset - 0x304) / sizeof(uint32_t)] = (uint32_t)value;
            if (offset == 0x30C)
                process_cpl_fifo(s);
            break;
        case 0x310 ... (0x310 + sizeof(HOST_DMA_CMD_FIFO_REG) - 4):
            {
                uint32_t dw_off = (offset - 0x310) / sizeof(uint32_t);
                int reg_trig = sizeof(HOST_DMA_CMD_FIFO_REG) / sizeof(uint32_t);

                writel(&s->dma_fifo_reg, offset - 0x310, value);

                if (dw_off % reg_trig == (reg_trig - 1))
                    dma_cmd_push(s);
            }
            break;
        default:
            DBG_PRINT("0x%"PRIx64"[0x%"PRIx32"] 0x%"PRIx64"\n", offset, size, value);
            assert(0);
            break;
    }
}

static void t4nvme_init_bar(TIGER4NVMeState *n)
{
    NVME_CAP_SET_MQES(n->bar.cap, 0xff);
    // NVME_CAP_SET_MQES(n->bar.cap, 0x1);
    NVME_CAP_SET_CQR(n->bar.cap, 1);
    NVME_CAP_SET_TO(n->bar.cap, 0x20);
    NVME_CAP_SET_CSS(n->bar.cap, 1);
    NVME_CAP_SET_MPSMAX(n->bar.cap, 0);

    n->bar.vs = 0x00010100;
    n->bar.nssrc = 0x4E564D65;
}

static const MemoryRegionOps t4nvme_ops = {
    .read = t4nvme_read,
    .write = t4nvme_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static Property t4nvme_properties[] = {
    DEFINE_PROP_CHR("chardev", TIGER4NVMeState, chr),
    DEFINE_PROP_END_OF_LIST(),
};

static void t4nvme_init(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    TIGER4NVMeState *s = TIGER4NVME(obj);

    memory_region_init_io(&s->iomem, OBJECT(s), &t4nvme_ops, s, "tiger4nvme", 0x10000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void t4nvme_realize(DeviceState *dev, Error **errp)
{
    TIGER4NVMeState *n = TIGER4NVME(dev);

    if (!qemu_chr_fe_backend_connected(&n->chr)) {
        error_setg(errp, "Tiger4 NVMe Host Controller requires chardev attribute");
        return;
    }

    t4nvme_init_bar(n);

    n->vpci = vPCI(object_new(TYPE_VPCI));
    vpci_init(n->vpci, &n->chr, &vpci_ops, n);

    fifo32_create(&n->cmd_fifo, (sizeof(NVME_CMD_FIFO_REG) / sizeof(uint32_t)) * T4NVME_MAX_CMD);
    // auto 256 * 2(rx,tx), direct 2 (rx,tx)
    fifo8_create(&n->dma_fifo, sizeof(HOST_DMA_CMD_FIFO_REG) * (T4NVME_MAX_DMA * 2 + 2));

    n->dma_info.state = DS_IDLE;
    n->dma_info.buf = malloc(4096);

    n->sq_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, cmd_fifo_req, n);
}

static void t4nvme_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = t4nvme_realize;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    device_class_set_props(dc, t4nvme_properties);

    // assert(sizeof(vPCI_PARAM) == sizeof(uint32_t));
}

static const TypeInfo t4nvme_info = {
    .name          = TYPE_TIGER4NVME,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(TIGER4NVMeState),
    .instance_init = t4nvme_init,
    .class_init    = t4nvme_class_init,
};

static void t4nvme_register_types(void)
{
    type_register_static(&t4nvme_info);
}

type_init(t4nvme_register_types)
