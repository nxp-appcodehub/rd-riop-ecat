/*
* Copyright 2025 NXP
*
* NXP Proprietary. This software is owned or controlled by NXP and may only be used strictly
* in accordance with the applicable license terms.  By expressly accepting such terms or by downloading,
* installing, activating and/or otherwise using the software, you are agreeing that you have read,
* and that you agree to comply with and are bound by, such license terms.  If you do not agree to be bound by
* the applicable license terms, then you may not retain, install, activate or otherwise use the software.
*/


#include <stdio.h>
#include <string.h>
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "pin_mux.h"
#include "fsl_rgpio.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_lpi2c.h"

#define EXAMPLE_I2C_MASTER_BASE LPI2C3_BASE

/* Get frequency of lpi2c clock */
#define LPI2C_CLOCK_FREQUENCY (CLOCK_GetRootClockFreq(kCLOCK_Root_Lpi2c0304))

#define LPI2C_MASTER_CLOCK_FREQUENCY LPI2C_CLOCK_FREQUENCY
#define WAIT_TIME                    10U

#define EXAMPLE_I2C_MASTER ((LPI2C_Type *)EXAMPLE_I2C_MASTER_BASE)

/* device address write A0 read A1  */
#define LPI2C_MASTER_SLAVE_ADDR_7BIT 0x50U
#define LPI2C_BAUDRATE               100000U
//#define LPI2C_DATA_LENGTH            16U*16

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

#define TAKE_WRITE
//#define TAKE_CLEAN

//#define EEWRITER_GENERIC
#define EEWRITER_3316



#ifdef EEWRITER_GENERIC
// EtherCAT 8 words (16 Bytes) configuration area
static uint8_t eeprom_txbuff[] = 
{
	0x80,0x0C,0x84,0xEE,0x00,0x0A,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x2A,0x00,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,

	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,

	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,

	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
};
#endif

#ifdef EEWRITER_3316
//This is 3316 eeprom image
static uint8_t eeprom_txbuff[] = 
{
	0x80,0x0c,0x84,0xee,0x00,0x0a,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x2a,0x00,
	0xc2,0x0c,0x00,0x00,0x01,0x00,0x00,0x00,
	0x11,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x10,0x80,0x00,0x80,0x10,0x80,0x00,
	0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

	0xff,0xff,0xff,0xff,0xff,0xff,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x01,0x00,0x01,0x00,
};
#endif

#ifdef TAKE_WRITE
#define LPI2C_DATA_LENGTH            sizeof(eeprom_txbuff)
#else
#define LPI2C_DATA_LENGTH            sizeof(eeprom_txbuff)+16
#endif

static uint8_t eeprom_rxbuff[LPI2C_DATA_LENGTH+16] = {}; 

static void EEWRITER_InitPins() {
  CLOCK_EnableClock(kCLOCK_Iomuxc2);          /* Turn on LPCG: LPCG is ON. */

  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AON_08_LPUART1_TX,          /* GPIO_AON_08 is configured as LPUART1_TX */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AON_09_LPUART1_RX,          /* GPIO_AON_09 is configured as LPUART1_RX */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AON_15_LPI2C2_SDA,          /* GPIO_AON_15 is configured as LPI2C2_SDA */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_AON_15 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AON_16_LPI2C2_SCL,          /* GPIO_AON_16 is configured as LPI2C2_SCL */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_AON_16 */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AON_15_LPI2C2_SDA,          /* GPIO_AON_15 PAD functional properties : */
      0x1AU);                                 /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: high driver
                                                 Pull / Keep Select Field: Pull Disable, Highz
                                                 Pull Up / Down Config. Field: Weak pull up
                                                 Open Drain Field: Enabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AON_16_LPI2C2_SCL,          /* GPIO_AON_16 PAD functional properties : */
      0x1AU);                                 /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: high driver
                                                 Pull / Keep Select Field: Pull Disable, Highz
                                                 Pull Up / Down Config. Field: Weak pull up
                                                 Open Drain Field: Enabled */
 
 /* config LPI2C3 */ 
 IOMUXC_SetPinMux(IOMUXC_GPIO_AD_18_LPI2C3_SCL,1);
 IOMUXC_SetPinMux(IOMUXC_GPIO_AD_19_LPI2C3_SDA,1);
 IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_18_LPI2C3_SCL,0x1AU);
 IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_19_LPI2C3_SDA,0x1AU);

}

static uint16_t deviceAddress = 0x0000U;

//returns err
static int read_check()
{
    status_t reVal;
    size_t txCount        = 0xFFU;
    /* Receive blocking data from slave */
    /* subAddress = 0x01, data = g_master_rxBuff - read from slave.
      start + slaveaddress(w) + subAddress + repeated start + slaveaddress(r) + rx data buffer + stop */
    if(kStatus_Success == LPI2C_MasterStart(EXAMPLE_I2C_MASTER, LPI2C_MASTER_SLAVE_ADDR_7BIT, kLPI2C_Write))
    {
        /* Check master tx FIFO empty or not */
        LPI2C_MasterGetFifoCounts(EXAMPLE_I2C_MASTER, NULL, &txCount);
        while (txCount)
        {
            LPI2C_MasterGetFifoCounts(EXAMPLE_I2C_MASTER, NULL, &txCount);
        }
        /* Check communicate with slave successful or not */
        if (LPI2C_MasterGetStatusFlags(EXAMPLE_I2C_MASTER) & kLPI2C_MasterNackDetectFlag)
        {
            return kStatus_LPI2C_Nak;
        }

        deviceAddress = 0x0000;       
        reVal = LPI2C_MasterSend(EXAMPLE_I2C_MASTER, &deviceAddress, 2);
        if(reVal != kStatus_Success)
        {
            if (reVal == kStatus_LPI2C_Nak)
            {
                LPI2C_MasterStop(EXAMPLE_I2C_MASTER);
            }
            return -1;
        }

        reVal = LPI2C_MasterRepeatedStart(EXAMPLE_I2C_MASTER, LPI2C_MASTER_SLAVE_ADDR_7BIT, kLPI2C_Read);
        if (reVal != kStatus_Success)
        {
          return -1;
        }
        
        reVal = LPI2C_MasterReceive(EXAMPLE_I2C_MASTER, eeprom_rxbuff, LPI2C_DATA_LENGTH+16);
        if (reVal != kStatus_Success)
        {
          if (reVal == kStatus_LPI2C_Nak)
          {
            LPI2C_MasterStop(EXAMPLE_I2C_MASTER);
          }
          return -1;
        }
        
        reVal = LPI2C_MasterStop(EXAMPLE_I2C_MASTER);
        if (reVal != kStatus_Success)
        {
          return -1;
        }
        
    }

    for (uint32_t i = 0U; i < LPI2C_DATA_LENGTH+16 ; i++)
    {
        if (i % 8 == 0)
        {
            PRINTF("\r\n");
        }
        PRINTF("0x%02x  ", eeprom_rxbuff[i]);
    }
    PRINTF("\r\n\r\n");

#ifdef TAKE_WRITE //Check
    {
	    //PRINTF("Check the data\r\n");
	    /* Transfer completed. Check the data.*/
	    int err;
	    err=0;
	    for (uint32_t i = 0U; i < LPI2C_DATA_LENGTH; i++)
	    {
		    if (eeprom_rxbuff[i] != eeprom_txbuff[i])
		    {
			    err=1;
			    break;
		    }
	    }
	    if(err)
	    {
		    //PRINTF("\r\nError occurred in the transfer ! \r\n");
	    }
	    else
	    {
		    //PRINTF("\r\n  EEPROM write success!!! .\r\n");
		    return 0;
	    }
    }
#endif
    return 1;
}


static int eeprom_writer(void)
{
	lpi2c_master_config_t masterConfig;
	status_t reVal = kStatus_Fail;


	uint16_t deviceAddress_send = 0x0000U;
	uint8_t deviceAddress_low = 0;
	uint8_t deviceAddress_high = 0;
	uint8_t deviceAddresslength = 2; /* byte */

	size_t txCount        = 0xFFU;
	//uint8_t eeprom_write = 0xa0U;

	uint8_t *txbuff_addr;
	txbuff_addr = eeprom_txbuff;

	BOARD_ConfigMPU();
	EEWRITER_InitPins();
	BOARD_InitBootClocks();
	BOARD_InitDebugConsole();

	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

#ifdef TAKE_WRITE
#ifdef TAKE_CLEAN
	{
		uint8_t *p,*e;
		p=eeprom_txbuff;
		e=eeprom_txbuff+sizeof(eeprom_txbuff);
		for(;;)
		{
			if(p>=e)
				break;
			*p++=0xff;
		}
	}
#endif
#endif
	/*
	 * masterConfig.debugEnable = false;
	 * masterConfig.ignoreAck = false;
	 * masterConfig.pinConfig = kLPI2C_2PinOpenDrain;
	 * masterConfig.baudRate_Hz = 100000U;
	 * masterConfig.busIdleTimeout_ns = 0;
	 * masterConfig.pinLowTimeout_ns = 0;
	 * masterConfig.sdaGlitchFilterWidth_ns = 0;
	 * masterConfig.sclGlitchFilterWidth_ns = 0;
	 */
	LPI2C_MasterGetDefaultConfig(&masterConfig);

	/* Change the default baudrate configuration */
	masterConfig.baudRate_Hz = LPI2C_BAUDRATE;

	/* Initialize the LPI2C master peripheral */
	LPI2C_MasterInit(EXAMPLE_I2C_MASTER, &masterConfig, LPI2C_MASTER_CLOCK_FREQUENCY);
#if 0
	/* delay some time */
	for(volatile int j=0; j< 100000; j++)
	{}
#else
	{
		uint32_t tt= DWT->CYCCNT;
		uint32_t tim=(SystemCoreClock/1000)*1;
		for(;;)
		{
			uint32_t diff = DWT->CYCCNT;
			diff-=tt;
			if(diff>=tim)
				break;
		}
	}
#endif

	PRINTF("Remote IO Platform - ETHERCAT EEPROM WRITER, version 1.0.0.28\r\n");

	if(read_check())
	{
#ifdef TAKE_WRITE
		PRINTF("\r\n  Need for EEPROM update detected.\r\n");
#else
		return 0;
#endif
	}
	else
	{
		PRINTF("\r\n  EEPROM already written, bailing out.\r\n");
		PRINTF("\r\n  ************************************\r\n");
		return 0;
	}


	deviceAddress = 0x0000;       
#if 0
	/* delay some time */
	for(volatile int j=0; j< 100000; j++)
	{}
#else
	{
		uint32_t tt= DWT->CYCCNT;
		uint32_t tim=SystemCoreClock/1000*1;
		for(;;)
		{
			uint32_t diff = DWT->CYCCNT;
			diff-=tt;
			if(diff>=tim)
				break;
		}
	}
#endif

#ifdef TAKE_WRITE //Enable write operation   

	//for(int i=0; i<LPI2C_DATA_LENGTH ;i++)
	for(int i=0;i<4;i++)//4*32byte
	{
		PRINTF(" addr:%4x ",(int)deviceAddress);
		PRINTF(" %d data have been written\r\n ",i);


		/* Send master blocking data to slave */
		if(kStatus_Success == LPI2C_MasterStart(EXAMPLE_I2C_MASTER, LPI2C_MASTER_SLAVE_ADDR_7BIT, kLPI2C_Write))
		{
			/* Check master tx FIFO empty or not */
			LPI2C_MasterGetFifoCounts(EXAMPLE_I2C_MASTER, NULL, &txCount);
			while (txCount)
			{
				LPI2C_MasterGetFifoCounts(EXAMPLE_I2C_MASTER, NULL, &txCount);
			}
			/* Check communicate with slave successful or not */
			if (LPI2C_MasterGetStatusFlags(EXAMPLE_I2C_MASTER) & kLPI2C_MasterNackDetectFlag)
			{
				return kStatus_LPI2C_Nak;
			}       

			deviceAddress_low = deviceAddress&0xff;
			deviceAddress_high = (deviceAddress>>8)&0xff;
			deviceAddress_send = (deviceAddress_low<<8)|deviceAddress_high;

			//PRINTF(" addr:%4x \r\n",(int)deviceAddress_send);
			/* subAddress = 0x01, data = g_master_txBuff - write to slave.
			   start + slaveaddress(w) + subAddress + length of data buffer + data buffer + stop*/
			reVal = LPI2C_MasterSend(EXAMPLE_I2C_MASTER, &deviceAddress_send, deviceAddresslength);
			if (reVal != kStatus_Success)
			{
				if (reVal == kStatus_LPI2C_Nak)
				{
					LPI2C_MasterStop(EXAMPLE_I2C_MASTER);
				}
				return -1;
			}

			/* EEPROM byte write */
			/* max length need <= 32 */
			//reVal = LPI2C_MasterSend(EXAMPLE_I2C_MASTER, txbuff_addr, LPI2C_DATA_LENGTH);
			reVal = LPI2C_MasterSend(EXAMPLE_I2C_MASTER, txbuff_addr, 32);
			if (reVal != kStatus_Success)
			{
				if (reVal == kStatus_LPI2C_Nak)
				{
					LPI2C_MasterStop(EXAMPLE_I2C_MASTER);
				}
				return -1;
			}

			reVal = LPI2C_MasterStop(EXAMPLE_I2C_MASTER);
			if (reVal != kStatus_Success)
			{
				return -1;
			}


		} 
		//txbuff_addr++;
		//deviceAddress++;
		txbuff_addr+=32;
		deviceAddress+=32;
		/* Wait until the slave is ready for transmit, wait time depend on user's case.
		   Slave devices that need some time to process received byte or are not ready yet to
		   send the next byte, can pull the clock low to signal to the master that it should wait.*/
#if 0
		for (uint32_t i = 0U; i < WAIT_TIME; i++)
		{
			__NOP();
		}
#else
		{
			uint32_t tt= DWT->CYCCNT;
			uint32_t tim=SystemCoreClock/(1000*1000*10)*1;
			for(;;)
			{
				uint32_t diff = DWT->CYCCNT;
				diff-=tt;
				if(diff>=tim)
					break;
			}
		}
#endif
	}

#endif

#if 0
	/* Wait until the slave is ready for transmit, wait time depend on user's case.
	   Slave devices that need some time to process received byte or are not ready yet to
	   send the next byte, can pull the clock low to signal to the master that it should wait.*/
	for (uint32_t i = 0U; i < WAIT_TIME; i++)
	{
		__NOP();
	}
#else
		{
			uint32_t tt= DWT->CYCCNT;
			uint32_t tim=SystemCoreClock/(1000*1000*10)*1;
			for(;;)
			{
				uint32_t diff = DWT->CYCCNT;
				diff-=tt;
				if(diff>=tim)
					break;
			}
		}
#endif

	PRINTF("Read the eeprom and verify the content :");

	if(read_check())
	{
		PRINTF("\r\nError occurred in the transfer ! \r\n");
		return 1;
	}
	else
	{
		PRINTF("\r\n  EEPROM write success!!! .\r\n");
	}

	return 0;
}





/*!
 * @brief Main function
 */
int main(void)
{
	int err=eeprom_writer();//Self configured code.
	BOARD_InitBootPins();
	if(err)
	{
		RGPIO_PinWrite(BOARD_USER_LED_RED_GPIO, BOARD_USER_LED_RED_GPIO_PIN, 1);
	}
	else
	{
		RGPIO_PinWrite(BOARD_USER_LED_GREEN_GPIO, BOARD_USER_LED_GREEN_GPIO_PIN, 1);
	}
	for(;;);
}
