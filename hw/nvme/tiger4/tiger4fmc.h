#ifndef TIGER4FMC_H
#define TIGER4FMC_H

#define TYPE_TIGER4FMC "tiger4fmc"
#define TIGER4FMC(obj) \
    OBJECT_CHECK(TIGER4FMCState, (obj), TYPE_TIGER4FMC)

typedef struct {
	uint32_t cmdSelect;
	uint32_t rowAddress;
	uint32_t userData;
	uint32_t dataAddress;
	uint32_t spareAddress;
	uint32_t errorCountAddress;
	uint32_t completionAddress;
	uint32_t waySelection;
	uint32_t channelBusy;
	uint32_t readyBusy;
} V2FMCRegisters;

typedef struct TIGER4FMCState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;

    V2FMCRegisters r;
    uint8_t fmc_id;

    BlockBackend *blk;
} TIGER4FMCState;

#endif
