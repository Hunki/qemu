#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "qemu/timer.h"
#include "chardev/char-fe.h"
#include "sysemu/block-backend.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "tiger4fmc.h"

#define R_CMD_SELECT        0x00
#define R_ROW_ADDR          0x04
#define R_USER_DATA         0x08
#define R_DATA_ADDR         0x0C
#define R_SPARE_ADDR        0x10
#define R_ERR_CNT_ADDR      0x14
#define R_CPL_ADDR          0x18
#define R_WAY_SEL           0x1C
#define R_CH_BUSY           0x20
#define R_READY_BUSY        0x24

#define V2FCommand_NOP 0
#define V2FCommand_Reset 1
#define V2FCommand_SetFeatures 6
#define V2FCommand_GetFeatures 46
#define V2FCommand_ReadPageTrigger 13
#define V2FCommand_ReadPageTransfer 18
#define V2FCommand_ProgramPage 28
#define V2FCommand_BlockErase 37
#define V2FCommand_StatusCheck 41
#define V2FCommand_ReadPageTransferRaw 55

//LUN
#define LUN_0_BASE_ADDR	0x00000000
#define LUN_1_BASE_ADDR	0x00200000

#define	SECTOR_SIZE_FTL			4096	//4KB

#define	PAGE_SIZE				16384	//16KB
#define SPARE_SIZE				256		//last 8 bytes are CRC bytes
#define BYTE_PER_ROW			(16384 + 1664)

#define	SLC_MODE				1
#define	MLC_MODE				2
#define	BIT_PER_FLASH_CELL		MLC_MODE //select SLC_MODE or MLC_MODE

#define	PAGE_NUM_PER_BLOCK		(128 * BIT_PER_FLASH_CELL)
#define	PAGE_NUM_PER_SLC_BLOCK	128
#define	PAGE_NUM_PER_MLC_BLOCK	256
#define	BLOCK_NUM_PER_LUN		(4096 / BIT_PER_FLASH_CELL) //DRAM size doesn't enough for page mapping when MLC mode uses all blocks. If you want to use all blocks, map cache function should be implemented.
#define	MAX_BLOCK_NUM_PER_LUN	4096
#define LUN_NUM_PER_DIE			2
#define	MAX_LUN_NUM_PER_DIE		2
#define	BLOCK_SIZE_MB			((PAGE_SIZE * PAGE_NUM_PER_BLOCK) / (1024 * 1024))

#define	CHANNEL_NUM				8
#define	MAX_CHANNEL_NUM			8
#define	WAY_NUM					8
#define	MAX_WAY_NUM				8
#define	DIE_NUM					(CHANNEL_NUM * WAY_NUM)

#define	SECTOR_NUM_PER_PAGE		(PAGE_SIZE / SECTOR_SIZE_FTL)

#define	PAGE_NUM_PER_LUN			(PAGE_NUM_PER_BLOCK * BLOCK_NUM_PER_LUN)
#define	MAX_PAGE_NUM_PER_SLC_LUN		(PAGE_NUM_PER_SLC_BLOCK * MAX_BLOCK_NUM_PER_LUN)
#define	PAGE_NUM_PER_DIE			(PAGE_NUM_PER_LUN * LUN_NUM_PER_DIE)
#define	PAGE_NUM_PER_CHANNEL		(PAGE_NUM_PER_DIE * WAY_NUM)
#define	PAGE_NUM_PER_SSD			(PAGE_NUM_PER_CHANNEL * CHANNEL_NUM)

#define	BLOCK_NUM_PER_DIE		(BLOCK_NUM_PER_LUN * LUN_NUM_PER_DIE)
#define	BLOCK_NUM_PER_CHANNEL	(BLOCK_NUM_PER_DIE * WAY_NUM)
#define	BLOCK_NUM_PER_SSD		(BLOCK_NUM_PER_CHANNEL * CHANNEL_NUM)

#define SSD_SIZE				(BLOCK_NUM_PER_SSD * BLOCK_SIZE_MB) //MB
#define FREE_BLOCK_SIZE			(DIE_NUM * BLOCK_SIZE_MB)			//MB
#define METADATA_BLOCK_SIZE		(DIE_NUM * BLOCK_SIZE_MB)			//MB
#define OVER_PROVISION_BLOCK_SIZE		((BLOCK_NUM_PER_SSD / 20) * BLOCK_SIZE_MB)	//MB

#define BAD_BLOCK_MARK_LOCATION1	0 			//first byte of data region
#define BAD_BLOCK_MARK_LOCATION2 	(PAGE_SIZE)	//first byte of spare region

#define CHUNK_NUM				32
#define BIT_ERROR_THRESHOLD		20
#define RETRY_LIMIT				5				//retry the failed request

#define CHANNEL_NUM_PER_HP_PORT	4

///////////////////////////
// cpu_physical_memory_read, cpu_physical_memory_write로 전체 메모리를 접근할 수 있음.
///////////////////////////
#if 0
#define DBG_PRINT(fmt, ...)     printf("[FMC  : %-24s] " fmt, __func__, ##__VA_ARGS__)
#define DEBUG_VERBOSE
#else
#define DBG_PRINT(fmt, ...)     (void)0
#endif

#ifdef DEBUG_VERBOSE
static const char *fmc_op_str[] = {
    [V2FCommand_NOP                ] = "NOP                ",
    [V2FCommand_Reset              ] = "Reset              ",
    [V2FCommand_SetFeatures        ] = "SetFeatures        ",
    [V2FCommand_GetFeatures        ] = "GetFeatures        ",
    [V2FCommand_ReadPageTrigger    ] = "ReadPageTrigger    ",
    [V2FCommand_ReadPageTransfer   ] = "ReadPageTransfer   ",
    [V2FCommand_ProgramPage        ] = "ProgramPage        ",
    [V2FCommand_BlockErase         ] = "BlockErase         ",
    [V2FCommand_StatusCheck        ] = "StatusCheck        ",
    [V2FCommand_ReadPageTransferRaw] = "ReadPageTransferRaw",
};
#endif

static void writel(void *ptr, uint32_t addr, uint32_t val)
{
    *(uint32_t *)(((uint8_t *)ptr) + addr) = val;
}

static void tiger4fmc_do(TIGER4FMCState *s)
{
    uint32_t status = 0;

    // DBG_PRINT("%s\n", fmc_op_str[s->r.cmdSelect]);

    hwaddr len;
    void *membuf;

    switch (s->r.cmdSelect) {
        case V2FCommand_NOP:
            break;
        case V2FCommand_Reset:
            break;
        case V2FCommand_SetFeatures:
            break;
        case V2FCommand_GetFeatures:
            break;
        case V2FCommand_ReadPageTrigger:
            break;
        case V2FCommand_ReadPageTransfer:
            break;
        case V2FCommand_ProgramPage:
            break;
        case V2FCommand_BlockErase:
            break;
        case V2FCommand_StatusCheck:
            break;
        case V2FCommand_ReadPageTransferRaw:
            break;
        default:
            assert(0);
            break;
    }

    if (s->blk) {
        uint64_t way_offset;
        uint64_t daddr;

        switch (s->r.cmdSelect) {
            // case V2FCommand_ReadPageTrigger:
            case V2FCommand_ReadPageTransfer:
            case V2FCommand_ProgramPage:
            case V2FCommand_BlockErase:
                way_offset = s->r.waySelection * 2 * (uint64_t)BYTE_PER_ROW * PAGE_NUM_PER_MLC_BLOCK * BLOCK_NUM_PER_LUN;
                daddr = way_offset + (uint64_t)(s->r.rowAddress & 0xFFFFF) * BYTE_PER_ROW;
                if ((s->r.rowAddress & LUN_1_BASE_ADDR) == LUN_1_BASE_ADDR)
                    daddr += (uint64_t)BYTE_PER_ROW * PAGE_NUM_PER_MLC_BLOCK * BLOCK_NUM_PER_LUN;
#ifdef DEBUG_VERBOSE
                DBG_PRINT("%s:%d - raddr 0x%08x daddr 0x%lx(0x%lx) data 0x%x\n",
                        fmc_op_str[s->r.cmdSelect], s->r.waySelection, s->r.rowAddress,
                        daddr, blk_getlength(s->blk), s->r.dataAddress);
#endif
                assert(blk_getlength(s->blk) > (daddr + PAGE_SIZE + SPARE_SIZE));
                break;
        }

        if (s->r.cmdSelect == V2FCommand_BlockErase) {
            // memset(s->pnand + daddr, 0xFFFFFFFF, BYTE_PER_ROW * PAGE_NUM_PER_MLC_BLOCK);
        }
        else if (s->r.cmdSelect == V2FCommand_ProgramPage) {
            len = PAGE_SIZE;
            membuf = cpu_physical_memory_map(s->r.dataAddress, &len, false);
            assert(len == PAGE_SIZE);
            blk_pwrite(s->blk, daddr, membuf, len, 0);
            cpu_physical_memory_unmap(membuf, len, false, len);

            len = SPARE_SIZE;
            membuf = cpu_physical_memory_map(s->r.spareAddress, &len, false);
            assert(len == SPARE_SIZE);
            blk_pwrite(s->blk, daddr + PAGE_SIZE, membuf, len, 0);
            cpu_physical_memory_unmap(membuf, len, false, len);
        }
        else if (s->r.cmdSelect == V2FCommand_ReadPageTransfer) {
            len = PAGE_SIZE;
            membuf = cpu_physical_memory_map(s->r.dataAddress, &len, true);
            assert(len == PAGE_SIZE);
            blk_pread(s->blk, daddr, membuf, len);
            cpu_physical_memory_unmap(membuf, len, true, len);

            len = SPARE_SIZE;
            membuf = cpu_physical_memory_map(s->r.spareAddress, &len, true);
            assert(len == SPARE_SIZE);
            blk_pread(s->blk, daddr + PAGE_SIZE, membuf, len);
            cpu_physical_memory_unmap(membuf, len, true, len);
        }
    }

    switch (s->r.cmdSelect) {
        case V2FCommand_ReadPageTransfer:
            assert(s->r.errorCountAddress != 0);
            status = 0x11000000;
            cpu_physical_memory_write(s->r.errorCountAddress, &status, sizeof(uint32_t));
            status = 0xFFFFFFFF;
            cpu_physical_memory_write(s->r.errorCountAddress + sizeof(uint32_t), &status, sizeof(uint32_t));
        case V2FCommand_GetFeatures:
        case V2FCommand_ReadPageTransferRaw:
        case V2FCommand_StatusCheck:
            assert(s->r.completionAddress != 0);
            status = 1 | (0x60 << 1);
            cpu_physical_memory_write(s->r.completionAddress, &status, sizeof(uint32_t));
            break;
    }

    s->r.completionAddress = 0;
}

static uint64_t tiger4fmc_read(void *opaque, hwaddr offset,
                           unsigned size)
{
    // TIGER4FMCState *s = (TIGER4FMCState *)opaque;

    switch (offset) {
        case R_CH_BUSY:
            return 0;
        case R_READY_BUSY:
            // return s->r.channelBusy;
            return 0xFFFFFFFF;
    }

    assert(0);
    return 0;
}

static void tiger4fmc_write(void *opaque, hwaddr offset,
                        uint64_t value, unsigned size)
{
    TIGER4FMCState *s = (TIGER4FMCState *)opaque;
    // assert(0);

    writel(&s->r, offset, (uint32_t)value);

    switch (offset) {
        case R_CMD_SELECT:
            tiger4fmc_do(s);
            break;
        default:
            break;
    }
}

static const MemoryRegionOps tiger4fmc_ops = {
    .read = tiger4fmc_read,
    .write = tiger4fmc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static Property tiger4fmc_properties[] = {
    DEFINE_PROP_DRIVE("drive", TIGER4FMCState, blk),
    DEFINE_PROP_UINT8("fmc-id", TIGER4FMCState, fmc_id, 0xFF),
    DEFINE_PROP_END_OF_LIST(),
};

static void tiger4fmc_init(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    TIGER4FMCState *s = TIGER4FMC(obj);

    memory_region_init_io(&s->iomem, OBJECT(s), &tiger4fmc_ops, s, "tiger4fmc", 0x10000);
    sysbus_init_mmio(sbd, &s->iomem);

    memset(&s->r, 0, sizeof(V2FMCRegisters));
}

static void tiger4fmc_realize(DeviceState *dev, Error **errp)
{
    TIGER4FMCState *s = TIGER4FMC(dev);

    if (s->blk) {
        if (blk_is_read_only(s->blk)) {
            error_setg(errp, "Can't use a read-only drive");
            return;
        }
        blk_set_perm(s->blk, BLK_PERM_CONSISTENT_READ | BLK_PERM_WRITE,
                     BLK_PERM_ALL, errp);
    }
    else {
        printf("tiger4fmc %d is dummy\n", s->fmc_id);
    }
}

static void tiger4fmc_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = tiger4fmc_realize;
    device_class_set_props(dc, tiger4fmc_properties);
}

static const TypeInfo tiger4fmc_info = {
    .name          = TYPE_TIGER4FMC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(TIGER4FMCState),
    .instance_init = tiger4fmc_init,
    .class_init    = tiger4fmc_class_init,
};

static void tiger4fmc_register_types(void)
{
    type_register_static(&tiger4fmc_info);
}

type_init(tiger4fmc_register_types)
