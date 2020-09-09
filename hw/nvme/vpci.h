#ifndef _vPCI_H_
#define _vPCI_H_

#include "vpci_msg.h"

#define TYPE_VPCI "vpci"
#define vPCI(obj) OBJECT_CHECK(vPCIObj, (obj), TYPE_VPCI)

typedef struct vPCIOps {
    // Frontend
    void    (*irq_req)(void *opaque, vPCIMsg *pmsg);
    void    (*dma_fe_read_req)(void *opaque, vPCIMsg *pmsg);
    void    (*dma_fe_write_req)(void *opaque, vPCIMsg *pmsg);

    // Backend
    void    (*dma_be_read_resp)(void *opaque, vPCIMsg *pmsg);
    void    (*dma_be_write_resp)(void *opaque, vPCIMsg *pmsg);

    // Common
    void    (*msix_req)(void *opaque, vPCIMsg *pmsg);
    void    (*mmio_write)(void *opaque, vPCIMsg *pmsg);

    void    (*init)(void *opaque);
} vPCIOps;

typedef enum vPCI_MSG_RECV_STATE {
    vPCI_MSG_RECV_MSG,
    vPCI_MSG_RECV_DATA,
} vPCI_MSG_RECV_STATE;

typedef struct vPCIObj {
    void                    *opaque;
    bool                    connected;
    const CharBackend       *chr;
    const vPCIOps           *ops;

    vPCIMsg                 *pmsg;

    vPCI_MSG_RECV_STATE     msg_recv_state;
    int                     msg_recv_size;
    int                     msg_recv_pos;

} vPCIObj;

typedef enum vPCI_DMA_RW {
    vPCI_DMA_READ,
    vPCI_DMA_WRITE,
} vPCI_DMA_RW;

void vpci_init(vPCIObj *s, const CharBackend *chr, const vPCIOps *ops, void *opaque);
void vpci_mmio_write_FtoB(vPCIObj *s, uint32_t addr, uint64_t value, uint8_t size, void *param);
void vpci_mmio_write_BtoF(vPCIObj *s, uint32_t addr, uint64_t value, uint8_t size, void *param);

void vpci_dma_read_req(vPCIObj *s, uint64_t dma_addr, void *buf_addr, uint32_t len, void *param);
void vpci_dma_read_resp(vPCIObj *s, vPCIMsg *pmsg, void *buf);
void vpci_dma_write_req(vPCIObj *s, uint64_t dma_addr, void *buf, uint32_t len, void *param);
void vpci_dma_write_resp(vPCIObj *s, vPCIMsg *pmsg);

void vpci_irq_assert(vPCIObj *s);
void vpci_irq_deassert(vPCIObj *s);

void vpci_msix_enable(vPCIObj *s, bool enable);
void vpci_msix_notify(vPCIObj *s, uint32_t vector);
void vpci_msix_use(vPCIObj *s, uint32_t vector);
void vpci_msix_unuse(vPCIObj *s, uint32_t vector);

#endif /* _vPCI_H_ */