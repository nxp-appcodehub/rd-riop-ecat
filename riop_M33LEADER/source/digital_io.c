/*
* Copyright 2025 NXP
*
* NXP Proprietary. This software is owned or controlled by NXP and may only be used strictly
* in accordance with the applicable license terms.  By expressly accepting such terms or by downloading,
* installing, activating and/or otherwise using the software, you are agreeing that you have read,
* and that you agree to comply with and are bound by, such license terms.  If you do not agree to be bound by
* the applicable license terms, then you may not retain, install, activate or otherwise use the software.
*/
/*
* This source file is part of the EtherCAT Slave Stack Code licensed by Beckhoff Automation GmbH & Co KG, 33415 Verl, Germany.
* The corresponding license agreement applies. This hint shall not be removed.
*/

/**
\addtogroup digital_io digital_io
@{
*/

/**
\file digital_io.c
\brief Implementation

\version 1.0.0.11
*/


/*-----------------------------------------------------------------------------------------
------
------    Includes
------
-----------------------------------------------------------------------------------------*/

#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "mcmgr.h"
#include "intercore_communication.h"


#include "gl.h"
#include "ecat_def.h"

#include "api_riop.h"
#include "api_icc.h"
extern riop_board_status_t g_boardStatus;
extern riop_command_t riop_command;
#include "applInterface.h"

#define _DIGITAL_IO_ 1
#include "digital_io.h"
#undef _DIGITAL_IO_

#include "fsl_rgpio.h"
#include "fsl_debug_console.h"

#include "applInterface.h"
/*--------------------------------------------------------------------------------------
------
------    local types and defines
------
--------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------
------
------    local variables and constants
------
-----------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------
------
------    application specific functions
------
-----------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------
------
------    generic functions
------
-----------------------------------------------------------------------------------------*/

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief    The function is called when an error state was acknowledged by the master

*////////////////////////////////////////////////////////////////////////////////////////

void    APPL_AckErrorInd(UINT16 stateTrans)
{

}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return    AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief    The function is called in the state transition from INIT to PREOP when
             all general settings were checked to start the mailbox handler. This function
             informs the application about the state transition, the application can refuse
             the state transition when returning an AL Status error code.
            The return code NOERROR_INWORK can be used, if the application cannot confirm
            the state transition immediately, in that case this function will be called cyclically
            until a value unequal NOERROR_INWORK is returned

*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StartMailboxHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return     0, NOERROR_INWORK

 \brief    The function is called in the state transition from PREEOP to INIT
             to stop the mailbox handler. This functions informs the application
             about the state transition, the application cannot refuse
             the state transition.

*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StopMailboxHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param    pIntMask    pointer to the AL Event Mask which will be written to the AL event Mask
                        register (0x204) when this function is succeeded. The event mask can be adapted
                        in this function
 \return    AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief    The function is called in the state transition from PREOP to SAFEOP when
           all general settings were checked to start the input handler. This function
           informs the application about the state transition, the application can refuse
           the state transition when returning an AL Status error code.
           The return code NOERROR_INWORK can be used, if the application cannot confirm
           the state transition immediately, in that case the application need to be complete 
           the transition by calling ECAT_StateChange.
*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StartInputHandler(UINT16 *pIntMask)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return     0, NOERROR_INWORK

 \brief    The function is called in the state transition from SAFEOP to PREEOP
             to stop the input handler. This functions informs the application
             about the state transition, the application cannot refuse
             the state transition.

*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StopInputHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return    AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief    The function is called in the state transition from SAFEOP to OP when
             all general settings were checked to start the output handler. This function
             informs the application about the state transition, the application can refuse
             the state transition when returning an AL Status error code.
           The return code NOERROR_INWORK can be used, if the application cannot confirm
           the state transition immediately, in that case the application need to be complete 
           the transition by calling ECAT_StateChange.
*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StartOutputHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return     0, NOERROR_INWORK

 \brief    The function is called in the state transition from OP to SAFEOP
             to stop the output handler. This functions informs the application
             about the state transition, the application cannot refuse
             the state transition.

*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StopOutputHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
\return     0(ALSTATUSCODE_NOERROR), NOERROR_INWORK
\param      pInputSize  pointer to save the input process data length
\param      pOutputSize  pointer to save the output process data length

\brief    This function calculates the process data sizes from the actual SM-PDO-Assign
            and PDO mapping
*////////////////////////////////////////////////////////////////////////////////////////
UINT16 APPL_GenerateMapping(UINT16 *pInputSize,UINT16 *pOutputSize)
{
    UINT16 result = ALSTATUSCODE_NOERROR;
    UINT16 InputSize = 0;
    UINT16 OutputSize = 0;

#if COE_SUPPORTED
    UINT16 PDOAssignEntryCnt = 0;
    OBJCONST TOBJECT OBJMEM * pPDO = NULL;
    UINT16 PDOSubindex0 = 0;
    UINT32 *pPDOEntry = NULL;
    UINT16 PDOEntryCnt = 0;
   
#if MAX_PD_OUTPUT_SIZE > 0
    /*Scan object 0x1C12 RXPDO assign*/
    for(PDOAssignEntryCnt = 0; PDOAssignEntryCnt < sRxPDOassign.u16SubIndex0; PDOAssignEntryCnt++)
    {
        pPDO = OBJ_GetObjectHandle(sRxPDOassign.aEntries[PDOAssignEntryCnt]);
        if(pPDO != NULL)
        {
            PDOSubindex0 = *((UINT16 *)pPDO->pVarPtr);
            for(PDOEntryCnt = 0; PDOEntryCnt < PDOSubindex0; PDOEntryCnt++)
            {
                pPDOEntry = (UINT32 *)((UINT16 *)pPDO->pVarPtr + (OBJ_GetEntryOffset((PDOEntryCnt+1),pPDO)>>3)/2);    //goto PDO entry
                // we increment the expected output size depending on the mapped Entry
                OutputSize += (UINT16) ((*pPDOEntry) & 0xFF);
            }
        }
        else
        {
            /*assigned PDO was not found in object dictionary. return invalid mapping*/
            OutputSize = 0;
            result = ALSTATUSCODE_INVALIDOUTPUTMAPPING;
            break;
        }
    }

    OutputSize = (OutputSize + 7) >> 3;
#endif

#if MAX_PD_INPUT_SIZE > 0
    if(result == 0)
    {
        /*Scan Object 0x1C13 TXPDO assign*/
        for(PDOAssignEntryCnt = 0; PDOAssignEntryCnt < sTxPDOassign.u16SubIndex0; PDOAssignEntryCnt++)
        {
            pPDO = OBJ_GetObjectHandle(sTxPDOassign.aEntries[PDOAssignEntryCnt]);
            if(pPDO != NULL)
            {
                PDOSubindex0 = *((UINT16 *)pPDO->pVarPtr);
                for(PDOEntryCnt = 0; PDOEntryCnt < PDOSubindex0; PDOEntryCnt++)
                {
                    pPDOEntry = (UINT32 *)((UINT16 *)pPDO->pVarPtr + (OBJ_GetEntryOffset((PDOEntryCnt+1),pPDO)>>3)/2);    //goto PDO entry
                    // we increment the expected output size depending on the mapped Entry
                    InputSize += (UINT16) ((*pPDOEntry) & 0xFF);
                }
            }
            else
            {
                /*assigned PDO was not found in object dictionary. return invalid mapping*/
                InputSize = 0;
                result = ALSTATUSCODE_INVALIDINPUTMAPPING;
                break;
            }
        }
    }
    InputSize = (InputSize + 7) >> 3;
#endif

#else
#if _WIN32
   #pragma message ("Warning: Define 'InputSize' and 'OutputSize'.")
#else
    #warning "Define 'InputSize' and 'OutputSize'."
#endif
#endif

    *pInputSize = InputSize;
    *pOutputSize = OutputSize;
    return result;
}
uint32_t fsm_mode;
unsigned char LED_status;
unsigned char di;
UINT32 adc_1;
/////////////////////////////////////////////////////////////////////////////////////////
/**
\param      pData  pointer to input process data

\brief      This function will copies the inputs from the local memory to the ESC memory
            to the hardware
*////////////////////////////////////////////////////////////////////////////////////////
#include "digital_ioObjects.h"
void APPL_InputMapping(UINT16* pData)
{
	uint8_t di;
	di=0;
	uint8_t *p;
	p=(uint8_t *)pData;
	uint16_t * pi;
	uint8_t * pii;
	pi=&g_boardStatus.afe_status.afe_gpio_input_pins_status;

#define ATOMIC_OP(XXX) if(*pi&(uint16_t)XXX) *p=1; else *p=0;p++
	ATOMIC_OP(0x0004);
	ATOMIC_OP(0x0008);
	ATOMIC_OP(0x0010);
	ATOMIC_OP(0x0020);

	ATOMIC_OP(0x0040);
	ATOMIC_OP(0x0080);
	ATOMIC_OP(0x0100);
	ATOMIC_OP(0x0200);
#undef ATOMIC_OP

#define ATOMIC_OP(XXX) if(*pii&(uint8_t)XXX) *p=1; else *p=0;p++
	pii=&g_boardStatus.mcu_digital_input_pins_status;
	ATOMIC_OP(0x01);
	ATOMIC_OP(0x02);
	ATOMIC_OP(0x04);
	ATOMIC_OP(0x08);

	ATOMIC_OP(0x10);
	ATOMIC_OP(0x20);
	ATOMIC_OP(0x40);
	MEMCPY(p, &di, SIZEOF(di));p++;

	pii=&g_boardStatus.mcu_digital_output_pins_status;
	ATOMIC_OP(0x01);
	ATOMIC_OP(0x02);
	ATOMIC_OP(0x04);
	ATOMIC_OP(0x08);

	ATOMIC_OP(0x10);
	ATOMIC_OP(0x20);
	ATOMIC_OP(0x40);
	ATOMIC_OP(0x80);

#undef ATOMIC_OP

	p++;
	p++;
	p++;
	p++;

	p++;
	p++;
	p++;
	p++;

	//LED
	MEMCPY(p,&LED_status,SIZEOF(LED_status));p++;
	{
		UINT32 tt;
		tt=(adc_1);
		MEMCPY(p, &tt, SIZEOF(tt));p+=4;
		MEMCPY(p, &adc_1, SIZEOF(adc_1));p+=4;
	}
	{
		INT32 tt;
		tt=(adc_1);
		tt=-tt;
		MEMCPY(p, &tt, SIZEOF(tt));p+=4;
		MEMCPY(p, &tt, SIZEOF(tt));p+=4;
	}
	{
		float * pf;
		float tt;
		tt=(adc_1);
		pf=&(g_boardStatus.afe_status.hvsig_status.channel_value[0]);
		tt=*pf++;
		MEMCPY(p, &tt, SIZEOF(tt));p+=4;
		tt=*pf++;
		MEMCPY(p, &tt, SIZEOF(tt));p+=4;
		tt=*pf++;
		MEMCPY(p, &tt, SIZEOF(tt));p+=4;
		pf=&(g_boardStatus.afe_status.current_status.channel_value);
		tt=*pf++;
		MEMCPY(p, &tt, SIZEOF(tt));p+=4;
		pf=&(g_boardStatus.afe_status.lvsig_status.channel_mean[0]);
		tt=*pf++;
		MEMCPY(p, &tt, SIZEOF(tt));p+=4;
		tt=*pf++;
		MEMCPY(p, &tt, SIZEOF(tt));p+=4;
		tt=*pf++;

		MEMCPY(p, &tt, SIZEOF(tt));p+=4;
		tt=*pf++;
		MEMCPY(p, &tt, SIZEOF(tt));p+=4;
		tt=*pf++;
		MEMCPY(p, &tt, SIZEOF(tt));p+=4; //#13

		pf=&(g_boardStatus.afe_status.temperature_status.channel_value);
		tt=*pf++;
		MEMCPY(p, &tt, SIZEOF(tt));p+=4; //#14
		MEMCPY(p, &tt, SIZEOF(tt));p+=4; //#15
		MEMCPY(p, &tt, SIZEOF(tt));p+=4; //#16
	} 
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
\param      pData  pointer to output process data

\brief    This function will copies the outputs from the ESC memory to the local memory
            to the hardware
*////////////////////////////////////////////////////////////////////////////////////////
void APPL_OutputMapping(UINT16* pData)
{
	//uint8_t di;
	uint8_t *p;
	p=(uint8_t *)pData;
	uint16_t * pi;
	uint8_t * pii;
	pi=&riop_command.afe_command.afe_gpio_command.gpio_output_value;
#define ATOMIC_OP(XXX) if(*p++) *pi|=(uint16_t)XXX; else *pi&=~(uint16_t)XXX
	ATOMIC_OP(0x0004);
	ATOMIC_OP(0x0008);
	ATOMIC_OP(0x0010);
	ATOMIC_OP(0x0020);

	ATOMIC_OP(0x0040);
	ATOMIC_OP(0x0080);
	ATOMIC_OP(0x0100);
	ATOMIC_OP(0x0200);
#undef ATOMIC_OP
	p++;
	p++;
	p++;
	p++;
	p++;
	p++;
	p++;
	p++;

#define ATOMIC_OP(XXX) if(*p++) *pii|=(uint8_t)XXX; else *pii&=~(uint8_t)XXX
	pii=&riop_command.mcu_digital_output_command.mcu_digital_output;
	ATOMIC_OP(0x01);
	ATOMIC_OP(0x02);
	ATOMIC_OP(0x04);
	ATOMIC_OP(0x08);

	ATOMIC_OP(0x10);
	ATOMIC_OP(0x20);
	ATOMIC_OP(0x40);
#if 0
	//Testing i/o switching.
	//One means output functionality. This solves output afe#5.
		if(*p)
			riop_command.afe_command.afe_gpio_command.gpio_pin_direction |=0x0040;// 0x3C0;
		else
			riop_command.afe_command.afe_gpio_command.gpio_pin_direction &=~(uint16_t)0x0040;
#endif
	ATOMIC_OP(0x80);

#undef ATOMIC_OP
#define ATOMIC_OP(XXX) if(*p++) riop_command.afe_command.afe_gpio_command.gpio_pin_direction |=XXX; \
		else riop_command.afe_command.afe_gpio_command.gpio_pin_direction &=~(uint16_t)XXX;

	ATOMIC_OP(0x0004);
	ATOMIC_OP(0x0008);
	ATOMIC_OP(0x0010);
	ATOMIC_OP(0x0020);
	ATOMIC_OP(0x0040);
	ATOMIC_OP(0x0080);
	ATOMIC_OP(0x0100);
	ATOMIC_OP(0x0200);
#undef ATOMIC_OP

	//Mostly unused analog outputs.
	MEMCPY(&LED_status,p,SIZEOF(LED_status));p++;
	{
		UINT32 tt;
		MEMCPY( &fsm_mode, p, SIZEOF(tt));p+=4;
		MEMCPY( &tt, p, SIZEOF(tt));p+=4;
	}
	{
		INT32 tt;
		MEMCPY( &tt, p, SIZEOF(tt));p+=4;
		MEMCPY( &tt, p, SIZEOF(tt));p+=4;
	}
	{
		float tt;
		tt=0.0;
		MEMCPY( &tt, p, SIZEOF(tt));p+=4;
		MEMCPY( &tt, p, SIZEOF(tt));p+=4;
		MEMCPY( &tt, p, SIZEOF(tt));p+=4;
		MEMCPY( &tt, p, SIZEOF(tt));p+=4;
		MEMCPY( &tt, p, SIZEOF(tt));p+=4;
		MEMCPY( &tt, p, SIZEOF(tt));p+=4;

		MEMCPY( &tt, p, SIZEOF(tt));p+=4;
		MEMCPY( &tt, p, SIZEOF(tt));p+=4;
		MEMCPY( &tt, p, SIZEOF(tt));p+=4;
		MEMCPY( &tt, p, SIZEOF(tt));p+=4;
		MEMCPY( &tt, p, SIZEOF(tt));p+=4;
		MEMCPY( &tt, p, SIZEOF(tt));p+=4;
	}
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
\brief    This function will called from the synchronisation ISR 
            or from the mainloop if no synchronisation is supported
*////////////////////////////////////////////////////////////////////////////////////////
void APPL_Application(void)
{
//    RGPIO_PinWrite(RGPIO5, 2, LED_status & 0x01);
    //RGPIO_PinWrite(RGPIO1, 15, LED_status&0x01);//Z desky RIOPKIT: BOARD_USER_LED_GREEN_GPIO_PIN
    led_green=LED_status;
	//RGPIO_PinWrite(BOARD_USER_LED_GREEN_GPIO, BOARD_USER_LED_GREEN_GPIO_PIN, LED_status&0x01);// RED LED, i.e. RGPIO_PinWrite(RGPIO1, 15, 0x01);
}

UINT8 ReadObject0x6020(UINT16 index, UINT8 subindex, UINT32 dataSize, UINT16 MBXMEM * pData, UINT8 bCompleteAccess) {
#if _WIN32
#pragma message ("Warning: Implement CoE read callback")
#else
 #warning "Implement CoE read callback"
#endif
 return 0;
}
UINT8 WriteObject0x7010(UINT16 index, UINT8 subindex, UINT32 dataSize, UINT16 MBXMEM * pData, UINT8 bCompleteAccess) {
#if _WIN32
#pragma message ("Warning: Implement CoE write callback")
#else
 #warning "Implement CoE write callback"
#endif
 return 0;
}

#if EXPLICIT_DEVICE_ID
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return    The Explicit Device ID of the EtherCAT slave

 \brief     Calculate the Explicit Device ID
*////////////////////////////////////////////////////////////////////////////////////////
UINT16 APPL_GetDeviceID()
{
#if _WIN32
   #pragma message ("Warning: Implement explicit Device ID latching")
#else
    #warning "Implement explicit Device ID latching"
#endif
    /* Explicit Device 5 is expected by Explicit Device ID conformance tests*/
    return 0x5;
}
#endif


/////////////////////////////////////////////////////////////////////////////////////////
/**
\param     index               index of the requested object.
\param     subindex            subindex of the requested object.
\param     objSize             size of the requested object data, calculated with OBJ_GetObjectLength
\param     pData               Pointer to the buffer where the data can be copied to
\param     bCompleteAccess     Indicates if a complete read of all subindices of the
                               object shall be done or not

 \return    result of the read operation (0 (success) or an abort code (ABORTIDX_.... defined in
            sdosrv.h))
 *////////////////////////////////////////////////////////////////////////////////////////
UINT8 ReadObject0x6000(UINT16 index, UINT8 subindex, UINT32 dataSize, UINT16 MBXMEM * pData, UINT8 bCompleteAccess) {
    if(bCompleteAccess)
    {
        return ABORTIDX_UNSUPPORTED_ACCESS;
    }

    if(subindex == 0)
    {
        *pData = 1;
    }
    else if(subindex == 1)
    {
        if(dataSize > 0)
        {
            MEMCPY(pData, &LED_status, dataSize);
        }
    }
 return 0;
}
/////////////////////////////////////////////////////////////////////////////////////////
/**
\param     index               index of the requested object.
\param     subindex            subindex of the requested object.
\param     objSize             size of the requested object data, calculated with OBJ_GetObjectLength
\param     pData               Pointer to the buffer where the data can be copied to
\param     bCompleteAccess     Indicates if a complete read of all subindices of the
                               object shall be done or not

 \return    result of the read operation (0 (success) or an abort code (ABORTIDX_.... defined in
            sdosrv.h))
 *////////////////////////////////////////////////////////////////////////////////////////
UINT8 WriteObject0x7000(UINT16 index, UINT8 subindex, UINT32 dataSize, UINT16 MBXMEM * pData, UINT8 bCompleteAccess) {
   if ( bCompleteAccess )
   {
      /* Complete Access is not supported for object 0x1010 */
      return ABORTIDX_UNSUPPORTED_ACCESS;
   }

   if ( subindex == 0 )
   {
      /* Subindex 0 is not writable */
      return ABORTIDX_READ_ONLY_ENTRY;
   }

   else if (subindex == 1)
   {
      /* Save the backup entries */
      MEMCPY(&LED_status, pData, SIZEOF(LED_status));
   }
   else
   {
      return ABORTIDX_VALUE_EXCEEDED;
   }
    //RGPIO_PinWrite(RGPIO4, 27, LED_status & 0x01);
    //led_green=LED_status;
	//RGPIO_PinWrite(BOARD_USER_LED_GREEN_GPIO, BOARD_USER_LED_GREEN_GPIO_PIN, LED_status&0x01);// RED LED, i.e. RGPIO_PinWrite(RGPIO1, 15, 0x01);
 return 0;
}
#if 0 // #if USE_DEFAULT_MAIN
/////////////////////////////////////////////////////////////////////////////////////////
/**

 \brief    This is the main function

*////////////////////////////////////////////////////////////////////////////////////////
//#include "fsl_xbar.h"
//#include "fsl_iomuxc.h"

void main(void)
{


	volatile uint16_t RPMsgRemoteReadyEventData = 0;



    /* Initialize MCMGR, install generic event handlers */
    (void)MCMGR_Init();

    /* Init board hardware.*/
    BOARD_ConfigMPU();
#if 1
    /* initialize the Hardware and the EtherCAT Slave Controller */
    HW_Init();
    MainInit();
#else
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
#endif




    bRunApplication = TRUE;
    do
    {
        MainLoop();
#if 1
        static UINT32 lcnt=0;
        if(!lcnt--)
        {
          lcnt=100000;
          static int lled=0;
          if(lled)
          {
            //RGPIO_PinWrite(RGPIO4, 26, 0x01);
            lled=0;
          }
          else
          {
            //RGPIO_PinWrite(RGPIO4, 26, 0x00);
            lled=1;
          }
	RGPIO_PinWrite(BOARD_USER_LED_RED_GPIO, BOARD_USER_LED_RED_GPIO_PIN, lled&0x01);// RED LED, i.e. RGPIO_PinWrite(RGPIO1, 3, 0x01);
            //RGPIO_PinWrite(RGPIO1, 3, lled&0x01);// RIOPKIT: BOARD_USER_LED_RED_GPIO_PIN
//            RGPIO_PinWrite(RGPIO1, 15, lled&0x01);// RIOPKIT: BOARD_USER_LED_GREEN_GPIO_PIN
//            RGPIO_PinWrite(RGPIO1, 25, lled&0x01);// RIOPKIT: BOARD_USER_LED_BLUE_GPIO_PIN
//            RGPIO_PinWrite(RGPIO1, 15, lled&0x01);// RIOPKIT: BOARD_USER_LED_GREEN_GPIO_PIN
        }
#endif
        
    } while (bRunApplication == TRUE);

    HW_Release();
}
#endif //#if USE_DEFAULT_MAIN
/** @} */


#if 0
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
uint8_t eeprom_txbuff[] = 
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
uint8_t eeprom_txbuff[] = 
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

uint8_t eeprom_rxbuff[LPI2C_DATA_LENGTH+16] = {}; 

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


int eeprom_writer(void)
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

	//PRINTF("\r\nLPI2C read EEPROM example\r\n");
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

	if(read_check())
	{
		PRINTF("\r\n  Need for EEPROM update detected.\r\n");
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

	//PRINTF(" write eeprom data over \r\n");
#endif
	/* Wait until the slave is ready for transmit, wait time depend on user's case.
	   Slave devices that need some time to process received byte or are not ready yet to
	   send the next byte, can pull the clock low to signal to the master that it should wait.*/
	    for (uint32_t i = 0U; i < WAIT_TIME; i++)
	//    {
	//        __NOP();
	//    }

	PRINTF("Read the eeprom and verify the content :");

	if(read_check())
	{
		PRINTF("\r\nError occurred in the transfer ! \r\n");
	}
	else
	{
		PRINTF("\r\n  EEPROM write success!!! .\r\n");
	}

	//    PRINTF("\r\nEnd of LPI2C example .\r\n");
	//    while (1)
	//    {
	//    }
	return 0;
}

#endif


/* Well done: Main function moved here from "flash zone". */
#include "mcmgr.h"

#include "intercore_communication.h"


static void RPMsgRemoteReadyEventHandler(uint16_t eventData, void *context)
{
    uint16_t *data = (uint16_t *)context;

    *data = eventData;
}











uint8_t led_green=0;


extern icc_handle_t g_icc_handle;
extern riop_board_status_t g_boardStatus;
riop_command_t riop_command;

#ifdef DEBUG_PRINT
	DEBUG_PRINT
#endif
void dwt_init(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
//Inline makes a problem under Debug
#define dwt_get() DWT->CYCCNT






/*!
 * @brief Main function
 */
int main(void)
{
	volatile uint16_t RPMsgRemoteReadyEventData = 0;



	/* Initialize MCMGR, install generic event handlers */
	(void)MCMGR_Init();

	/* Init board hardware.*/
#if 1
	HW_Init();
	MainInit();
#else
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitDebugConsole();
#endif
	BOARD_InitDebugConsole();

	dwt_init();

	/* Print the initial banner from Primary core */
	(void)PRINTF("\r\nHello World from the Primary Core!\r\n\n");

#ifdef CORE1_IMAGE_COPY_TO_RAM
	/* This section ensures the secondary core image is copied from flash location to the target RAM memory.
	   It consists of several steps: image size calculation, image copying and cache invalidation (optional for some
	   platforms/cases). These steps are not required on MCUXpresso IDE which copies the secondary core image to the
	   target memory during startup automatically. */
	uint32_t core1_image_size;
	core1_image_size = get_core1_image_size();
	(void)PRINTF("Copy Secondary core image to address: 0x%x, size: %d\r\n", (void *)(char *)CORE1_BOOT_ADDRESS,
			core1_image_size);

	/* Copy Secondary core application from FLASH to the target memory. */
	(void)memcpy((void *)(char *)CORE1_BOOT_ADDRESS, (void *)CORE1_IMAGE_START, core1_image_size);

#ifdef APP_INVALIDATE_CACHE_FOR_SECONDARY_CORE_IMAGE_MEMORY
	/* Invalidate cache for memory range the secondary core image has been copied to. */
	if (LMEM_PSCCR_ENCACHE_MASK == (LMEM_PSCCR_ENCACHE_MASK & LMEM->PSCCR))
	{
		L1CACHE_CleanInvalidateSystemCacheByRange((uint32_t)CORE1_BOOT_ADDRESS, core1_image_size);
	}
#endif /* APP_INVALIDATE_CACHE_FOR_SECONDARY_CORE_IMAGE_MEMORY*/
#endif /* CORE1_IMAGE_COPY_TO_RAM */

	/* Register the application event before starting the secondary core */
	(void)MCMGR_RegisterEvent(kMCMGR_RemoteApplicationEvent, RPMsgRemoteReadyEventHandler,
			(void *)&RPMsgRemoteReadyEventData);

	PRINTF("Remote IO Platform - ETHERCAT DEMO, version 1.0.0.29\r\n");

	/* Boot Secondary core application */
	(void)PRINTF("Starting Secondary core.\r\n");
	(void)MCMGR_StartCore(kMCMGR_Core1, (void *)(char *)CORE1_BOOT_ADDRESS, (uint32_t)rpmsg_lite_base,
			kMCMGR_Start_Synchronous);

	(void)PRINTF("The secondary core application has been started.\r\n");

	/* Wait until the secondary core application signals the rpmsg remote has been initialized and is ready to
	 * communicate. */
	while (APP_RPMSG_READY_EVENT_DATA != RPMsgRemoteReadyEventData)
	{
	};

	RPMSG_CommunicationInit();

	//my_rpmsg = rpmsg_lite_master_init(rpmsg_lite_base, SH_MEM_TOTAL_SIZE, RPMSG_LITE_LINK_ID, RL_NO_FLAGS);

	//my_queue  = rpmsg_queue_create(my_rpmsg);
	//my_ept    = rpmsg_lite_create_ept(my_rpmsg, LOCAL_EPT_ADDR, rpmsg_queue_rx_cb, my_queue);
	//ns_handle = rpmsg_ns_bind(my_rpmsg, app_nameservice_isr_cb, (void *)&remote_addr);

	{
		//uint16_t counter = 0;
		memset(&riop_command, 0, sizeof(riop_command));



		//riop_command.afe_command.afe_gpio_command.gpio_output_value = 4;
		riop_command.afe_command.afe_gpio_command.gpio_output_value = 0;
		//riop_command.afe_command.afe_gpio_command.gpio_pin_direction = 0x3FC;
		riop_command.afe_command.afe_gpio_command.gpio_pin_direction = 0x3C0;//4 outputs and 4 inputs, set via ecat
		riop_command.afe_command.afe_gpio_command.is_dirty = 0;

		riop_command.afe_command.afe_hvsig_command.channel_enable[0] = 1;
		riop_command.afe_command.afe_hvsig_command.channel_enable[1] = 1;
		riop_command.afe_command.afe_hvsig_command.channel_enable[2] = 1;
		riop_command.afe_command.afe_hvsig_command.is_dirty = 0;

#if 1
		riop_command.afe_command.afe_lvsig_command.is_dirty = 0;
		riop_command.afe_command.afe_lvsig_command.channel_enable[0] = 1;
		riop_command.afe_command.afe_lvsig_command.channel_enable[1] = 1;
		riop_command.afe_command.afe_lvsig_command.channel_enable[2] = 1;
		riop_command.afe_command.afe_lvsig_command.channel_enable[3] = 1;
		riop_command.afe_command.afe_lvsig_command.channel_enable[4] = 1;
		riop_command.afe_command.afe_lvsig_command.samples_amt = 500;
#else

		riop_command.siggen_control_command.enable = 1;
		riop_command.siggen_control_command.frequency = 10;
		riop_command.siggen_control_command.is_dirty = 1;
#endif
		riop_command.afe_command.afe_current_command.is_dirty = 0;
		riop_command.afe_command.afe_current_command.shunt_value=kShunt100; 
		riop_command.afe_command.afe_current_command.measurement_enabled=1;

		riop_command.afe_command.afe_temperature_command.is_dirty = 0;
		riop_command.afe_command.afe_temperature_command.measurement_enabled=1;
		riop_command.afe_command.afe_temperature_command.samples_amt=128;//up to 6sec!!

		static riop_status_t last_err=0;
		uint32_t time;
		uint32_t tim;
		uint32_t tim_led;
		int tim_mode=200;
		int tim_first=0;
#ifdef DEBUG_PRINT1
	DEBUG_PRINT1
#endif
		tim=dwt_get();
		tim_led=dwt_get();
//		static int first=1;
		bRunApplication = TRUE;
		do
		{
			MainLoop();
			if(tim_first)
			{
				time=SystemCoreClock/1000*200;
				tim_first=0;
			}
			else
			{
				time=SystemCoreClock/1000*tim_mode;
			}

			static int lled=0;


			uint32_t time_led;
			time_led=SystemCoreClock/1000*100;
			uint32_t diff_led;
			diff_led=dwt_get();
			diff_led-=tim_led;
			//if(!lcnt--)
			if(diff_led>time_led)//max 4 sec
			{
				tim_led+=time_led; //tim=dwt_get();
						   //lcnt=10*1000*1000;
						   //lcnt=100000/4;//Ecat engine delays us

				lled=!lled;
				if(led_green)
				{
					RGPIO_PinWrite(BOARD_USER_LED_RED_GPIO, BOARD_USER_LED_RED_GPIO_PIN, 0);
					RGPIO_PinWrite(BOARD_USER_LED_GREEN_GPIO, BOARD_USER_LED_GREEN_GPIO_PIN, lled&0x01);
				}
				else
				{
					RGPIO_PinWrite(BOARD_USER_LED_RED_GPIO, BOARD_USER_LED_RED_GPIO_PIN, lled&0x01);
					RGPIO_PinWrite(BOARD_USER_LED_GREEN_GPIO, BOARD_USER_LED_GREEN_GPIO_PIN, 0);
				}
#ifdef DEBUG_PRINT2
	DEBUG_PRINT2
#endif
			}


			uint32_t dd;
			dd=dwt_get();
			dd-=tim;
			if(dd>time)//max 4 sec
			{
				tim+=time; //tim=dwt_get();


				riop_command.afe_command.afe_hvsig_command.is_dirty = 0;
				riop_command.afe_command.afe_current_command.is_dirty = 0;
				riop_command.siggen_control_command.is_dirty = 0;
				riop_command.afe_command.afe_lvsig_command.is_dirty = 0;
				riop_command.afe_command.afe_gpio_command.is_dirty = 0;
				riop_command.mcu_digital_output_command.is_dirty = 0;
				riop_command.afe_command.afe_temperature_command.is_dirty = 0;
#if 0
				//gpio toggling
				if(lled)
					riop_command.afe_command.afe_gpio_command.gpio_output_value =0x3F0;
				else
					riop_command.afe_command.afe_gpio_command.gpio_output_value =0x0;

				if(lled)
					riop_command.mcu_digital_output_command.mcu_digital_output =0x00;
				else
					riop_command.mcu_digital_output_command.mcu_digital_output =0xF1;
#endif

				static int fsm=0;
				int32_t * pfsm_mode;
				pfsm_mode=&fsm_mode;
				static int last_mode=0;
				if(last_mode!=*pfsm_mode)
				{
					fsm=0;
					tim_first=1;
					last_mode=*pfsm_mode;
					switch(*pfsm_mode)
					{
						default:
						case 0:
							tim_mode=200;
							break;
						case 1:
							tim_mode=20;
							riop_command.afe_command.afe_hvsig_command.is_dirty = 1;
							break;
						case 2:
							tim_mode=20;
							riop_command.afe_command.afe_current_command.is_dirty = 1;
							break;
						case 3:
							tim_mode=200;
							riop_command.afe_command.afe_lvsig_command.is_dirty = 1;
							break;
						case 4:
							tim_mode=20;
							riop_command.afe_command.afe_gpio_command.is_dirty = 1;
							break;
						case 5:
							tim_mode=20;
							riop_command.mcu_digital_output_command.is_dirty = 1;
							break;
						case 6:
							tim_mode=3000;
							riop_command.afe_command.afe_temperature_command.is_dirty = 1;
							break;
					}
					//!!!!
					//tim_mode*=4;
				}
				switch(*pfsm_mode)
				{
					case 0:
					{
						//Automatic switching modes
						switch(++fsm)
						{
							default:
								fsm=0;
							case 0:
								riop_command.afe_command.afe_gpio_command.is_dirty = 1;
								break;
							case 1:
								break;
							case 2:
								riop_command.afe_command.afe_hvsig_command.is_dirty = 1;
								break;
							case 3:
								riop_command.mcu_digital_output_command.is_dirty = 1;
								break;
							case 4:
								riop_command.afe_command.afe_current_command.is_dirty = 1;
								break;
							case 5:
								{
									static uint32_t pass=0;
									pass++;
									if(!(pass&0x07))
										riop_command.afe_command.afe_lvsig_command.is_dirty = 1;
								}
								break;
								/*
								 * This delays the loop for a couple of seconds.
							case 6:
								{
									riop_command.afe_command.afe_temperature_command.is_dirty = 1;
								}
								break;
							case 7:
								break;
							case 8:
								break;
							case 9:
								break;
							case 10:
								break;
							case 11:
								break;
							case 12:
								break;
							case 13:
								break;
							case 14:
								break;
							case 15:
								break;
							case 16:
								break;
							case 17:
								break;
							case 18:
								break;
							case 19:
								break;
								*/
						}
					}
					break;
					case 1:
					break;
					case 2:
					break;
					case 3:
					{
						static uint32_t pass=0;
						pass++;
						if(!(pass&0x07))
							riop_command.afe_command.afe_lvsig_command.is_dirty = 1;
					}
					break;
					case 4:
					//Change detector with validator.
					static uint16_t afe_last;
					static uint8_t afe_last_val=0;
					if(!afe_last_val)
					{
						afe_last=riop_command.afe_command.afe_gpio_command.gpio_output_value;
						afe_last_val=1;
					}
					if(afe_last!=riop_command.afe_command.afe_gpio_command.gpio_output_value)
					{
						afe_last=riop_command.afe_command.afe_gpio_command.gpio_output_value;
						riop_command.afe_command.afe_gpio_command.is_dirty = 1;
					}
					static uint16_t afe_last_d;
					static uint8_t afe_last_d_val=0;
					//Another change detector
					if(!afe_last_d_val)
					{
						afe_last_d=riop_command.afe_command.afe_gpio_command.gpio_pin_direction;
						afe_last_d_val=1;
					}
					if(afe_last_d!=riop_command.afe_command.afe_gpio_command.gpio_pin_direction)
					{
						afe_last_d=riop_command.afe_command.afe_gpio_command.gpio_pin_direction;
						riop_command.afe_command.afe_gpio_command.is_dirty = 1;
					}
					break;
					case 5:
					//Change detector with validator.
					static uint8_t mcu_last;
					static uint8_t mcu_last_val=0;
					if(!mcu_last_val)
					{
						mcu_last=riop_command.mcu_digital_output_command.mcu_digital_output;
						mcu_last_val=1;
					}
					if(mcu_last!=riop_command.mcu_digital_output_command.mcu_digital_output)
					{
						mcu_last=riop_command.mcu_digital_output_command.mcu_digital_output;
						riop_command.mcu_digital_output_command.is_dirty = 1;
					}
					break;
					case 6:
						riop_command.afe_command.afe_temperature_command.is_dirty = 1;
					break;
				}
				//(void)rpmsg_lite_send(my_rpmsg, my_ept, remote_addr, (char *)&riop_command, sizeof(riop_command), RL_DONT_BLOCK);
				riop_status_t bbb= ICC_WriteData(&g_icc_handle, (void *)&riop_command, sizeof(riop_command), RL_DONT_BLOCK);
				if(bbb)
					last_err=bbb;
			}
			//first=0;
		} while (bRunApplication == TRUE);
		HW_Release();
	}
}
