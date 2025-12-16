/*
 * Copyright 2022, 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "flexspi_ops.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 *****************************************************************************/
flexspi_device_config_t deviceconfig = {
    .flexspiRootClk       = 133000000,
    .flashSize            = FLASH_SIZE,
    .CSIntervalUnit       = kFLEXSPI_CsIntervalUnit1SckCycle,
    .CSInterval           = 2,
    .CSHoldTime           = 3,
    .CSSetupTime          = 3,
    .dataValidTime        = 0,
    .columnspace          = 0,
    .enableWordAddress    = 0,
    .AWRSeqIndex          = NOR_CMD_LUT_SEQ_IDX_WRITE,
    .AWRSeqNumber         = 1,
    .ARDSeqIndex          = NOR_CMD_LUT_SEQ_IDX_READ,
    .ARDSeqNumber         = 1,
    .AHBWriteWaitUnit     = kFLEXSPI_AhbWriteWaitUnit2AhbCycle,
    .AHBWriteWaitInterval = 0,
};

#if defined(OCTAL_FLASH) && (OCTAL_FLASH == 1)

const uint32_t customLUT[CUSTOM_LUT_LENGTH] = {
		    [4 * NOR_CMD_LUT_SEQ_IDX_READ + 0] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR, kFLEXSPI_8PAD, 0xEE, kFLEXSPI_Command_DDR, kFLEXSPI_8PAD, 0x11),
		    [4 * NOR_CMD_LUT_SEQ_IDX_READ + 1] = FLEXSPI_LUT_SEQ( kFLEXSPI_Command_RADDR_DDR, kFLEXSPI_8PAD, 0x20, kFLEXSPI_Command_DUMMY_DDR, kFLEXSPI_8PAD, 20),
		    [4 * NOR_CMD_LUT_SEQ_IDX_READ + 2] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_READ_DDR, kFLEXSPI_8PAD, 0x04, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x0),

		    /* Read ID */
		    [4 * NOR_CMD_LUT_SEQ_IDX_READID_OPI] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR, kFLEXSPI_8PAD, 0x9F, kFLEXSPI_Command_DDR, kFLEXSPI_8PAD, 0x60),
		    [4 * NOR_CMD_LUT_SEQ_IDX_READID_OPI + 1] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR, kFLEXSPI_8PAD, 0x0, kFLEXSPI_Command_DDR, kFLEXSPI_8PAD, 0x0),
		    [4 * NOR_CMD_LUT_SEQ_IDX_READID_OPI + 2] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR, kFLEXSPI_8PAD, 0x0, kFLEXSPI_Command_DDR, kFLEXSPI_8PAD, 0x0),
		    [4 * NOR_CMD_LUT_SEQ_IDX_READID_OPI + 3] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DUMMY_DDR, kFLEXSPI_8PAD, 0x4, kFLEXSPI_Command_READ_SDR, kFLEXSPI_8PAD, 0x04),	//SDR mmode - potential issue here

		    /*  Write Enable */
		    [4 * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE_OPI] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR, kFLEXSPI_8PAD, 0x06, kFLEXSPI_Command_DDR, kFLEXSPI_8PAD, 0xF9),
			[4 * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE_OPI + 1] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_STOP, kFLEXSPI_8PAD, 0x00, kFLEXSPI_Command_STOP, kFLEXSPI_8PAD, 0x00),

		    /*  Erase Sector */
		    [4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR, kFLEXSPI_8PAD, 0x21, kFLEXSPI_Command_DDR, kFLEXSPI_8PAD, 0xDE),
		    [4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 1] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_RADDR_DDR, kFLEXSPI_8PAD, 0x20, kFLEXSPI_Command_STOP, kFLEXSPI_8PAD, 0),

		    /*  Erase Chip */
		    [4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR, kFLEXSPI_8PAD, 0x60, kFLEXSPI_Command_DDR, kFLEXSPI_8PAD, 0x9F),
		    [4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE + 1] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_STOP, kFLEXSPI_8PAD, 0, kFLEXSPI_Command_STOP, kFLEXSPI_8PAD, 0),

		    /*  Program */
		    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM + 0] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR, kFLEXSPI_8PAD, 0x12, kFLEXSPI_Command_DDR, kFLEXSPI_8PAD, 0xED),
		    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM + 1] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_RADDR_DDR, kFLEXSPI_8PAD, 0x20, kFLEXSPI_Command_WRITE_DDR, kFLEXSPI_8PAD, 0x80),

		    /*  Dummy write, do nothing when AHB write command is triggered. */
		    [4 * NOR_CMD_LUT_SEQ_IDX_WRITE] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x0, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x0),

		    /*  Read status register using Octal DDR read */
		    [4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS_OPI + 0] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR, kFLEXSPI_8PAD, 0x05, kFLEXSPI_Command_DDR, kFLEXSPI_8PAD, 0xFA),
		    [4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS_OPI + 1] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR, kFLEXSPI_8PAD, 0x0, kFLEXSPI_Command_DDR, kFLEXSPI_8PAD, 0x0),
		    [4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS_OPI + 2] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR, kFLEXSPI_8PAD, 0x0, kFLEXSPI_Command_DDR, kFLEXSPI_8PAD, 0x0),
		    [4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS_OPI + 3] = FLEXSPI_LUT_SEQ( kFLEXSPI_Command_DUMMY_DDR, kFLEXSPI_8PAD, 0x8, kFLEXSPI_Command_READ_DDR, kFLEXSPI_8PAD, 0x04),

			 /*  Write Enable SPI mode */
			[4 * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE_SPI] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x06, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x0),

			[4 * NOR_CMD_LUT_SEQ_IDX_READ_ID_SPI + 0] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x9F, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x4),
			[4 * NOR_CMD_LUT_SEQ_IDX_READ_ID_SPI + 1] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x0, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x0),
			[4 * NOR_CMD_LUT_SEQ_IDX_READ_ID_OCT + 0] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR, kFLEXSPI_8PAD, 0x9F, kFLEXSPI_Command_DDR, kFLEXSPI_8PAD, 0x60),
			[4 * NOR_CMD_LUT_SEQ_IDX_READ_ID_OCT + 1] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR, kFLEXSPI_8PAD, 0x0, kFLEXSPI_Command_DDR, kFLEXSPI_8PAD, 0x0),
			[4 * NOR_CMD_LUT_SEQ_IDX_READ_ID_OCT + 2] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR, kFLEXSPI_8PAD, 0x0, kFLEXSPI_Command_DDR, kFLEXSPI_8PAD, 0x0),
		    [4 * NOR_CMD_LUT_SEQ_IDX_READ_ID_OCT + 3] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DUMMY_DDR, kFLEXSPI_8PAD, 0x8, kFLEXSPI_Command_READ_DDR, kFLEXSPI_8PAD, 0x02),


			/* Wrine Configuration Register 2 - DTR OPI mode */
			[4 * NOR_CMD_LUT_SEQ_IDX_WRITECONFIG2_SPI + 0] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x72, kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x0),
			[4 * NOR_CMD_LUT_SEQ_IDX_WRITECONFIG2_SPI + 1] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x0, kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x0),
			[4 * NOR_CMD_LUT_SEQ_IDX_WRITECONFIG2_SPI + 2] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x0, kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x2),
			[4 * NOR_CMD_LUT_SEQ_IDX_WRITECONFIG2_SPI + 3] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x0, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x0),

};

#endif

void flexspi_clock_init(void)
{
	EXAMPLE_FLEXSPI->MCR0 |= FLEXSPI_MCR0_MDIS_MASK;

	CLOCK_InitPfd(kCLOCK_PllSys3, kCLOCK_Pfd0, 32);

	clock_root_config_t rootCfg = {0};

	rootCfg.mux = kCLOCK_FLEXSPI1_ClockRoot_MuxSysPll3Pfd0;
	rootCfg.div = 2;
	CLOCK_SetRootClock(kCLOCK_Root_Flexspi1, &rootCfg);

	EXAMPLE_FLEXSPI->MCR0 &= ~FLEXSPI_MCR0_MDIS_MASK;
}

#define CM33_MASTER_ID              0x8
#define CM33_PREFETCH_BUFFER_ID     0x0
#define CM33_PREFETCH_BUFFER_SIZE   32U

void flexspi_config_init(flexspi_config_t *config)
{
    /* Small prefetch buffer for CM33 master */
    config->ahbConfig.buffer[CM33_PREFETCH_BUFFER_ID].enablePrefetch = true;
    config->ahbConfig.buffer[CM33_PREFETCH_BUFFER_ID].masterIndex = CM33_MASTER_ID;
    config->ahbConfig.buffer[CM33_PREFETCH_BUFFER_ID].bufferSize = CM33_PREFETCH_BUFFER_SIZE;
}
