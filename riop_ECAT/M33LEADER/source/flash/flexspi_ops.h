/*
 * Copyright 2020, 2022-2023, 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _FLEXSPI_OPS_H_
#define _FLEXSPI_OPS_H_

#include "fsl_flexspi.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define OCTAL_FLASH 1

#define EXAMPLE_FLEXSPI FLEXSPI1
#define EXAMPLE_FLEXSPI_AMBA_BASE FlexSPI1_AMBA_BASE
#define EXAMPLE_FLEXSPI_PORT kFLEXSPI_PortB1
#define FLASH_SIZE 0x2000 /* 128Mb/KiB (128*1024*1024 / 1024 / 8) */
#define FLASH_PAGE_SIZE 256
#define FLASH_SECTOR_SIZE 0x1000 /* 4KiB */
#define FLASH_SECTOR_TOTAL	((FLASH_SIZE * 1024) / FLASH_SECTOR_SIZE)
#define FLASH_SECTOR_LFS	(FLASH_SECTOR_TOTAL / 4) /* 8MiB */

#if defined(OCTAL_FLASH) && (OCTAL_FLASH == 1)

#define NOR_CMD_LUT_SEQ_IDX_READ     		   0
#define NOR_CMD_LUT_SEQ_IDX_READID_OPI     	   1
#define NOR_CMD_LUT_SEQ_IDX_WRITEENABLE_OPI    2
#define NOR_CMD_LUT_SEQ_IDX_ERASESECTOR    	   3
#define NOR_CMD_LUT_SEQ_IDX_CHIPERASE		   4
#define NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM        5
#define NOR_CMD_LUT_SEQ_IDX_WRITE			   6
#define NOR_CMD_LUT_SEQ_IDX_READSTATUS_OPI     7
#define NOR_CMD_LUT_SEQ_IDX_WRITEENABLE_SPI    8
#define NOR_CMD_LUT_SEQ_IDX_WRITECONFIG2_SPI   9
#define NOR_CMD_LUT_SEQ_IDX_READ_ID_SPI        10
#define NOR_CMD_LUT_SEQ_IDX_READ_ID_OCT 	   12


#endif

#define CUSTOM_LUT_LENGTH        60
#define FLASH_QUAD_ENABLE        0x40
#define FLASH_BUSY_STATUS_POL    1
#define FLASH_BUSY_STATUS_OFFSET 0

extern flexspi_device_config_t deviceconfig;
extern const uint32_t customLUT[CUSTOM_LUT_LENGTH];

int flexspi_init(void);

void flexspi_clock_init(void);

void flexspi_config_init(flexspi_config_t *config);

status_t flexspi_page_program(FLEXSPI_Type *base, uint32_t address, const uint32_t *src);

status_t flexspi_erase_sector(FLEXSPI_Type *base, uint32_t address);

status_t flexspi_nor_enable_octal_mode(FLEXSPI_Type *base);

#endif /* _FLEXSPI_OPS_H_ */
