#ifndef _vPCI_MSG_H_
#define _vPCI_MSG_H_

typedef enum vPCI_MSG_TYPES {
    vPCI_MSG_MMIO_WRITE_FTOB,
    vPCI_MSG_MMIO_WRITE_BTOF,

    vPCI_MSG_BE_DMA_READ_REQ,
    vPCI_MSG_BE_DMA_READ_RESP,
    vPCI_MSG_BE_DMA_WRITE_REQ,
    vPCI_MSG_BE_DMA_WRITE_RESP,
    vPCI_MSG_BE_IRQ,

    vPCI_MSG_MSI,

    vPCI_MSG_MAX,
} vPCI_MSG_TYPES;

typedef enum vPCI_MSI_TYPES {
    vPCI_MSI_ENABLE,
    vPCI_MSI_DISABLE,

    vPCI_MSI_NOTIFY,
    vPCI_MSI_USE,
    vPCI_MSI_UNUSE,
} vPCI_MSI_TYPES;

typedef struct vPCIMsgHeader {
    uint32_t        type;
    uint32_t        size;
} __attribute__ ((packed)) vPCIMsgHeader;

typedef struct vPCIMsgMMIO {
    uint64_t        addr;
    uint64_t        data;
    uint32_t        len;
} __attribute__ ((packed)) vPCIMsgMMIO;

typedef struct vPCIMsgDMA {
    uint64_t        param;
    uint64_t        dma_addr;
    uint64_t        buf_addr;
    uint32_t        len;
} __attribute__ ((packed)) vPCIMsgDMA;

typedef struct vPCIMsgIRQ {
    uint32_t        assert;
} __attribute__ ((packed)) vPCIMsgIRQ;

typedef struct vPCIMsgMSIX {
    uint32_t        type;
    uint32_t        vector;
} __attribute__ ((packed)) vPCIMsgMSIX;

typedef union vPCIMsgBody {
    uint32_t        dw[7];
    vPCIMsgMMIO     mmio;
    vPCIMsgDMA      dma;
    vPCIMsgIRQ      irq;
    vPCIMsgMSIX     msix;
} __attribute__ ((packed)) vPCIMsgBody;

typedef struct vPCIMsg {
    vPCIMsgHeader   hdr;
    vPCIMsgBody     body;
    uint8_t         data[0];
} __attribute__ ((packed)) vPCIMsg;

#endif /* _vPCI_MSG_H_ */