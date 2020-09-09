#ifndef TIGER4NVME_H
#define TIGER4NVME_H

#include "block/nvme.h"
#include "hw/nvme/vpci.h"

#define TYPE_TIGER4NVME "tiger4nvme"
#define TIGER4NVME(obj) \
    OBJECT_CHECK(TIGER4NVMeState, (obj), TYPE_TIGER4NVME)

#define HOST_IP_ADDR						(0)

#define DEV_IRQ_MASK_REG_ADDR				(HOST_IP_ADDR + 0x4)
#define DEV_IRQ_CLEAR_REG_ADDR				(HOST_IP_ADDR + 0x8)
#define DEV_IRQ_STATUS_REG_ADDR				(HOST_IP_ADDR + 0xC)

#define PCIE_STATUS_REG_ADDR				(HOST_IP_ADDR + 0x100)
#define PCIE_FUNC_REG_ADDR					(HOST_IP_ADDR + 0x104)

#define NVME_STATUS_REG_ADDR				(HOST_IP_ADDR + 0x200)
#define HOST_DMA_FIFO_CNT_REG_ADDR			(HOST_IP_ADDR + 0x204)
#define NVME_ADMIN_QUEUE_SET_REG_ADDR		(HOST_IP_ADDR + 0x21C)
#define NVME_IO_SQ_SET_REG_ADDR				(HOST_IP_ADDR + 0x220)
#define NVME_IO_CQ_SET_REG_ADDR				(HOST_IP_ADDR + 0x260)

#define NVME_CMD_FIFO_REG_ADDR				(HOST_IP_ADDR + 0x300)
#define NVME_CPL_FIFO_REG_ADDR				(HOST_IP_ADDR + 0x304)
#define HOST_DMA_CMD_FIFO_REG_ADDR			(HOST_IP_ADDR + 0x310)

#define NVME_CMD_SRAM_ADDR					(HOST_IP_ADDR + 0x2000)


#define HOST_DMA_DIRECT_TYPE				(1)
#define HOST_DMA_AUTO_TYPE					(0)

#define HOST_DMA_TX_DIRECTION				(1)
#define HOST_DMA_RX_DIRECTION				(0)

#define ONLY_CPL_TYPE						(0)
#define AUTO_CPL_TYPE						(1)
#define CMD_SLOT_RELEASE_TYPE				(2)

#pragma pack(push, 1)

typedef struct _DEV_IRQ_REG
{
	union {
		uint32_t dw;
		struct {
			uint32_t pcieLink			:1;
			uint32_t busMaster			:1;
			uint32_t pcieIrq			:1;
			uint32_t pcieMsi			:1;
			uint32_t pcieMsix			:1;
			uint32_t nvmeCcEn			:1;
			uint32_t nvmeCcShn			:1;
			uint32_t mAxiWriteErr		:1;
			uint32_t mAxiReadErr		:1;
			uint32_t pcieMreqErr		:1;
			uint32_t pcieCpldErr		:1;
			uint32_t pcieCpldLenErr		:1;
			uint32_t reserved0			:20;
		};
	};
} DEV_IRQ_REG;

typedef struct _PCIE_STATUS_REG
{
	union {
		uint32_t dw;
		struct {
			uint32_t ltssm				:6;
			uint32_t reserved0			:2;
			uint32_t pcieLinkUp			:1;
			uint32_t reserved1			:23;
		};
	};
} PCIE_STATUS_REG;

typedef struct _PCIE_FUNC_REG
{
	union {
		uint32_t dw;
		struct {
			uint32_t busMaster			:1;
			uint32_t msiEnable			:1;
			uint32_t msixEnable			:1;
			uint32_t irqDisable			:1;
			uint32_t msiVecNum			:3;
			uint32_t reserved0			:25;
		};
	};
} PCIE_FUNC_REG;

//offset: 0x00000200, size: 4
typedef struct _NVME_STATUS_REG
{
	union {
		uint32_t dw;
		struct {
			uint32_t ccEn				:1;
			uint32_t ccShn				:2;
			uint32_t reserved0			:1;
			uint32_t cstsRdy			:1;
			uint32_t cstsShst			:2;
			uint32_t reserved1			:25;
		};
	};
} NVME_STATUS_REG;

//offset: 0x00000300, size: 4
typedef struct _NVME_CMD_FIFO_REG
{
	union {
		uint32_t dw;
		struct {
			uint32_t qID				:4;
			uint32_t reserved0			:4;
			uint32_t cmdSlotTag			:7;
			uint32_t reserved2			:1;
			uint32_t cmdSeqNum			:8;
			uint32_t reserved3			:7;
			uint32_t cmdValid			:1;
		};
	};
} NVME_CMD_FIFO_REG;

//offset: 0x00000304, size: 8
typedef struct _NVME_CPL_FIFO_REG
{
	union {
		uint32_t dw[3];
		struct {
			struct {
				uint32_t cid				:16;
				uint32_t sqId				:4;
				uint32_t reserved0			:12;
			};

			uint32_t specific;

			uint16_t cmdSlotTag			:7;
			uint16_t reserved1			:7;
			uint16_t cplType			:2;

			union {
				uint16_t statusFieldWord;
				struct {
					uint16_t reserved0	:1;
					uint16_t SC			:8;
					uint16_t SCT		:3;
					uint16_t reserved1	:2;
					uint16_t MORE		:1;
					uint16_t DNR		:1;
				} statusField;
			};
		};
	};
} NVME_CPL_FIFO_REG;

//offset: 0x0000021C, size: 4
typedef struct _NVME_ADMIN_QUEUE_SET_REG
{
	union {
		uint32_t dw;
		struct {
			uint32_t cqValid			:1;
			uint32_t sqValid			:1;
			uint32_t cqIrqEn			:1;
			uint32_t reserved0			:29;
		};
	};
} NVME_ADMIN_QUEUE_SET_REG;

//offset: 0x00000220, size: 8
typedef struct _NVME_IO_SQ_SET_REG
{
	union {
		uint32_t dw[2];
		struct {
			uint32_t pcieBaseAddrL;
			uint32_t pcieBaseAddrH		:4;
			uint32_t reserved0			:11;
			uint32_t valid				:1;
			uint32_t cqVector			:4;
			uint32_t reserved1			:4;
			uint32_t sqSize				:8;
		};
	};
} NVME_IO_SQ_SET_REG;


//offset: 0x00000260, size: 8
typedef struct _NVME_IO_CQ_SET_REG
{
	union {
		uint32_t dw[2];
		struct {
			uint32_t pcieBaseAddrL;
			uint32_t pcieBaseAddrH		:4;
			uint32_t reserved0			:11;
			uint32_t valid				:1;
			uint32_t irqVector			:3;
			uint32_t irqEn				:1;
			uint32_t reserved1			:4;
			uint32_t cqSize				:8;
		};
	};
} NVME_IO_CQ_SET_REG;

//offset: 0x00000204, size: 4
typedef struct _HOST_DMA_FIFO_CNT_REG
{
	union {
		uint32_t dw;
		struct {
			uint8_t directDmaRx;
			uint8_t directDmaTx;
			uint8_t autoDmaRx;
			uint8_t autoDmaTx;
		};
	};
} HOST_DMA_FIFO_CNT_REG;

//offset: 0x0000030C, size: 16
typedef struct _HOST_DMA_CMD_FIFO_REG
{
	union {
		uint32_t dw[4];
		struct {
			uint32_t devAddr;
			uint32_t pcieAddrH;
			uint32_t pcieAddrL;
			struct {
				uint32_t dmaLen				:13;
				uint32_t reserved0			:1;
				uint32_t cmd4KBOffset		:9;
				uint32_t cmdSlotTag			:7;
				uint32_t dmaDirection		:1;
				uint32_t dmaType			:1;
			};
		};
	};
} HOST_DMA_CMD_FIFO_REG;

#pragma pack(pop)

#define T4NVME_MAX_CMD	128
//offset: 0x00002000, size: 64 * 128
typedef struct NVME_CMD_FW
{
	// uint32_t dw[128][16];
	NvmeCmd cmd[T4NVME_MAX_CMD];
} NVME_CMD_FW;

typedef struct NVME_CMD_HW_INFO {
	uint16_t	qid;
	uint32_t	num;
	// uint64_t	prp_list[MAX_PRP_ENTRIES];
	uint64_t	*prp_list;
	bool		valid;
} NVME_CMD_HW_INFO;

#define T4NVME_MAX_Q   		8
#define T4NVME_MAX_DMA		256

typedef struct NvmeSQueue {
	uint32_t	head;
	uint32_t	tail;
	uint16_t	cqid;
	uint32_t	size;

	uint64_t	dma_addr;
} NvmeSQueue;

typedef struct NvmeCQueue {
	uint32_t	head;
	uint32_t	tail;
	uint32_t	vector;
	uint32_t	size;

	uint64_t	dma_addr;
	uint8_t		phase;
	uint16_t	irq_enabled;
} NvmeCQueue;

#define MAX_PRP_ENTRIES		512

typedef enum {
	DS_IDLE,
	DS_PRP_START,
	DS_PRP2_LIST_GET,
	DS_PRP_END,
	DS_DIRECT_WAIT,
} DMAState;

typedef struct DMA_INFO {
	DMAState				state;
	uint8_t					*buf;
	HOST_DMA_CMD_FIFO_REG	*cmd;
} DMA_INFO;

typedef struct TIGER4NVMeState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    CharBackend chr;
	vPCIObj *vpci;

    MemoryRegion iomem;
    qemu_irq irq;

    bool msix_enabled;
    NvmeBar bar;

    DEV_IRQ_REG                 irq_mask;       // 0x004
    // DEV_IRQ_REG                 irq_clear;      // 0x008
    DEV_IRQ_REG                 irq_status;     // 0x00C

    PCIE_STATUS_REG             pcie_status;    // 0x100
    PCIE_FUNC_REG               pcie_func;      // 0x104

    NVME_STATUS_REG             nvme_status;    // 0x200
    HOST_DMA_FIFO_CNT_REG       dma_fifo_cnt;   // 0x204
    NVME_ADMIN_QUEUE_SET_REG    admin_q_set;    // 0x21C
    NVME_IO_SQ_SET_REG          io_sq_set[T4NVME_MAX_Q];   // 0x220
    NVME_IO_CQ_SET_REG          io_cq_set[T4NVME_MAX_Q];   // 0x260

	Fifo32						cmd_fifo;		// 0x300
    NVME_CPL_FIFO_REG           cpl_fifo;   	// 0x304
    HOST_DMA_CMD_FIFO_REG       dma_fifo_reg;   // 0x310

	NvmeCmd 					sram_cmd[T4NVME_MAX_CMD];	// 0x2000
	NVME_CMD_HW_INFO			hwi_cmd[T4NVME_MAX_CMD];

	NvmeSQueue					sq[T4NVME_MAX_Q + 1];
	NvmeCQueue					cq[T4NVME_MAX_Q + 1];

	uint8_t						last_slot;
	QEMUTimer					*sq_timer;

	Fifo8						dma_fifo;
	DMA_INFO					dma_info;

	uint32_t					pci_irq_status;
	uint32_t					page_bits;
	uint32_t					page_size;
	uint32_t					max_prp_ents;
} TIGER4NVMeState;

#endif
