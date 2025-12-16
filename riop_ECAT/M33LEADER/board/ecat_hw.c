/*
* Copyright 2025 NXP
*
* NXP Proprietary. This software is owned or controlled by NXP and may only be used strictly
* in accordance with the applicable license terms.  By expressly accepting such terms or by downloading,
* installing, activating and/or otherwise using the software, you are agreeing that you have read,
* and that you agree to comply with and are bound by, such license terms.  If you do not agree to be bound by
* the applicable license terms, then you may not retain, install, activate or otherwise use the software.
*/

#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_xbar.h"
#include "fsl_iomuxc.h"
#include "fsl_ecat.h"
#include "fsl_gpt.h"
#include "flexspi_ops.h"
#include "ecat_hw.h"

/* TODO: Review */
#define ESC_AL_EVENTMASK_OFFSET                 0x0204

uint32_t EcatTimerCnt;

static void Ecat_KickOff(void)
{

/* RMII ENABLE */
#if 0  
    BLK_CTRL_WAKEUPMIX->ECAT_MISC_CFG |= BLK_CTRL_WAKEUPMIX_ECAT_MISC_CFG_RMII_REF_CLK_DIR0_MASK |
                                         BLK_CTRL_WAKEUPMIX_ECAT_MISC_CFG_RMII_REF_CLK_DIR1_MASK;    
    BLK_CTRL_WAKEUPMIX->ECAT_MISC_CFG |=
        (BLK_CTRL_WAKEUPMIX_ECAT_MISC_CFG_RMII_SEL0_MASK | BLK_CTRL_WAKEUPMIX_ECAT_MISC_CFG_RMII_SEL1_MASK);
#endif

/* B0 ENABLE */    
#if 1    
    //BLK_CTRL_WAKEUPMIX->MISC_IO_CTRL &= ~(1 << BLK_CTRL_WAKEUPMIX_MISC_IO_CTRL_ECAT_LINK_ACT0_POL_SHIFT);
    //BLK_CTRL_WAKEUPMIX->MISC_IO_CTRL &= ~(1 << BLK_CTRL_WAKEUPMIX_MISC_IO_CTRL_ECAT_LINK_ACT1_POL_SHIFT);
    
    BLK_CTRL_WAKEUPMIX->ECAT_MISC_CFG |= (1 << BLK_CTRL_WAKEUPMIX_ECAT_MISC_CFG_EEPROM_SIZE_OPTION_SHIFT);
    
    //SRC_GENERAL_REG->SRMASK &= ~(0x1 << SRC_GENERAL_SRTMR_ECAT_RSTO_TRIG_MODE_SHIFT);
#endif
    
    BLK_CTRL_WAKEUPMIX->ECAT_MISC_CFG |= (BLK_CTRL_WAKEUPMIX_ECAT_MISC_CFG_PHY_OFFSET_VEC(0));
    BLK_CTRL_WAKEUPMIX->ECAT_MISC_CFG &= ~BLK_CTRL_WAKEUPMIX_ECAT_MISC_CFG_GLB_RST_MASK;
    BLK_CTRL_WAKEUPMIX->ECAT_MISC_CFG |= BLK_CTRL_WAKEUPMIX_ECAT_MISC_CFG_GLB_EN_MASK;
}

void BOARD_ECAT_MDIO_InitPins(void){
  CLOCK_EnableClock(kCLOCK_Iomuxc1);          /* Turn on LPCG: LPCG is ON. */

  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_30_ECAT_MCLK,            /* GPIO_AD_30 is configured as ECAT_MCLK */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_31_ECAT_MDIO,            /* GPIO_AD_31 is configured as ECAT_MDIO */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AD_30_ECAT_MCLK,            /* GPIO_AD_30 PAD functional properties : */
      0x0EU);                                 /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: high driver
                                                 Pull / Keep Select Field: Pull Enable
                                                 Pull Up / Down Config. Field: Weak pull up
                                                 Open Drain Field: Disabled
                                                 Force ibe off Field: Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AD_31_ECAT_MDIO,            /* GPIO_AD_31 PAD functional properties : */
      0x0EU);                                 /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: high driver
                                                 Pull / Keep Select Field: Pull Enable
                                                 Pull Up / Down Config. Field: Weak pull up
                                                 Open Drain Field: Disabled
                                                 Force ibe off Field: Disabled */
}


int BOARD_InitHardware(void)
{
    uint32_t intMask;
    xbar_control_config_t xbaraConfig;
    uint32_t gptFreq;
    gpt_config_t gptConfig;
#if 0
    rgpio_pin_config_t pinConfig = {
      .pinDirection = kRGPIO_DigitalOutput,
      .outputLogic = 0
    };

    UINT16 led_startus = 0;
#endif
    uint16_t Reg_Temp = 0;
    
    /* Init board hardware. */
    //BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_ECAT_MDIO_InitPins();
    BOARD_Init_ECAT_I2C_Pins(); /* TODO: Already invoked in BOARD_InitBootPins() */

    BOARD_BootClockRUN();

    flexspi_init();
    BOARD_InitDebugConsole();
#ifndef BUILD_NAME
#define BUILD_NAME "ETHERCAT"
#endif

    PRINTF("Start the digital_io example. (" BUILD_NAME ") [" __DATE__ " " __TIME__ "]\r\n");
    
    /* Reset ecat PHY */
    RGPIO_PinWrite(RGPIO5, 6, 0);
    SDK_DelayAtLeastUs(15000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);

    Ecat_KickOff();

    RGPIO_PinWrite(RGPIO5, 6, 1);
    SDK_DelayAtLeastUs(45000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
    SDK_DelayAtLeastUs(45000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);

#if 1    
    
    /* bypass control enable auto MDI-X at 10/100  */
    //ECAT_EscMdioWrite(ECAT, 0x00, 0x12, 0x0008);
    //ECAT_EscMdioWrite(ECAT, 0x01, 0x12, 0x0008);
       
    /* init phy to 100M */
#if 1        
    /* enable ActiPHY mode */
    ECAT_EscMdioRead(ECAT, 0x00, 0x1C, &Reg_Temp);
    Reg_Temp = Reg_Temp|0x0040;
    //PRINTF("Reg temp is %0x\r\n",Reg_Temp);
    ECAT_EscMdioWrite(ECAT, 0x00, 0x1C, Reg_Temp);
    
    ECAT_EscMdioRead(ECAT, 0x01, 0x1C, &Reg_Temp);
    Reg_Temp = Reg_Temp|0x0040;
    //PRINTF("Reg temp is %0x\r\n",Reg_Temp);
    ECAT_EscMdioWrite(ECAT, 0x01, 0x1C, Reg_Temp);
#endif

#if 1    
    /* set PHY_AUTONEG_ADVERTISE_REG */
    ECAT_EscMdioRead(ECAT, 0x00, 0x04, &Reg_Temp);
    Reg_Temp = Reg_Temp|0x01E0;
    //PRINTF("Reg temp is %0x\r\n",Reg_Temp);
    //Reg_Temp = Reg_Temp&0xFFFE;
    ECAT_EscMdioWrite(ECAT, 0x00, 0x04, Reg_Temp);
    
    ECAT_EscMdioRead(ECAT, 0x01, 0x04, &Reg_Temp);
    Reg_Temp = Reg_Temp|0x01E0;
    //PRINTF("Reg temp is %0x\r\n",Reg_Temp);
    //Reg_Temp = Reg_Temp&0xFFFE;
    ECAT_EscMdioWrite(ECAT, 0x01, 0x04, Reg_Temp);
#endif

#if 1    
    /* enable auto nego and re nego */
    ECAT_EscMdioRead(ECAT, 0x00, 0x00, &Reg_Temp);
    Reg_Temp = Reg_Temp|0x21;
    //PRINTF("Reg temp is %0x\r\n",Reg_Temp);
    ECAT_EscMdioWrite(ECAT, 0x00, 0x00, Reg_Temp);
    
    ECAT_EscMdioRead(ECAT, 0x01, 0x00, &Reg_Temp);
    Reg_Temp = Reg_Temp|0x21;
    //PRINTF("Reg temp is %0x\r\n",Reg_Temp);
    ECAT_EscMdioWrite(ECAT, 0x01, 0x00, Reg_Temp);
#endif
    
    /* config mode control reg 100Mb and FD */
    ECAT_EscMdioRead(ECAT, 0x00, 0x00, &Reg_Temp);
    //Reg_Temp = Reg_Temp&0xffBf;
    Reg_Temp = Reg_Temp|0x2100;
    //PRINTF("Reg_Temp is %0x\r\n",Reg_Temp);
    ECAT_EscMdioWrite(ECAT, 0x00, 0x00, Reg_Temp);
    
    ECAT_EscMdioRead(ECAT, 0x01, 0x00, &Reg_Temp);
    //Reg_Temp = Reg_Temp&0xffBf;
    Reg_Temp = Reg_Temp|0x2100;
    //PRINTF("Reg temp is %0x\r\n",Reg_Temp);
    ECAT_EscMdioWrite(ECAT, 0x01, 0x00, Reg_Temp);

    /* set PHY_1000BASET_CONTROL_REG to 0 */
    ECAT_EscMdioWrite(ECAT, 0x00, 0x09, 0x0000);
    ECAT_EscMdioWrite(ECAT, 0x01, 0x09, 0x0000);
    
    /* set led0 ACT led1 100tx */
    ECAT_EscMdioWrite(ECAT, 0x00, 0x1D, 0x0020);
    ECAT_EscMdioWrite(ECAT, 0x01, 0x1D, 0x0020);

#if 1     
    /* set led 0 blink led1 pulse stretch and disable led combination */
    ECAT_EscMdioRead(ECAT, 0x00, 0x1E, &Reg_Temp);
    
    Reg_Temp = Reg_Temp|0x0042;
    //PRINTF("Reg temp is %0x\r\n",Reg_Temp);
    ECAT_EscMdioWrite(ECAT, 0x00, 0x1E, Reg_Temp);
    
    ECAT_EscMdioRead(ECAT, 0x01, 0x1E, &Reg_Temp);
    Reg_Temp = Reg_Temp|0x0042;
    //PRINTF("Reg temp is %0x\r\n",Reg_Temp);
    ECAT_EscMdioWrite(ECAT, 0x01, 0x1E, Reg_Temp);
#endif    
    
#endif    

#if 0    
    /*set port0 page register*/
    ECAT_EscMdioWrite(ECAT, 0x00, 31, 0x07);
    
    /*enable prot0 coustomized LED */
    ECAT_EscMdioRead(ECAT, 0x00, 19, &led_startus);
    ECAT_EscMdioWrite(ECAT, 0x00, 19, led_startus | (1 << 3));

    /*Set led1 to LINK100 and set led0 to ACK*/
    ECAT_EscMdioRead(ECAT, 0x00, 17, &led_startus);
    ECAT_EscMdioWrite(ECAT, 0x00, 17, led_startus | (1 << 3) | (1 << 5));
    
    PRINTF("port0 led_startus is %0x\r\n",led_startus);
    
    /*set port1 page register*/
    ECAT_EscMdioWrite(ECAT, 0x01, 31, 0x07);

    /*enable prot1 coustomized LED */
    ECAT_EscMdioRead(ECAT, 0x01, 19, &led_startus);
    ECAT_EscMdioWrite(ECAT, 0x01, 19, led_startus | (1 << 3));

    /*Set led1 to LINK100 and set led0 to ACK*/
    ECAT_EscMdioRead(ECAT, 0x01, 17, &led_startus);
    ECAT_EscMdioWrite(ECAT, 0x01, 17, led_startus | (1 << 3) | (1 << 5));
    
    PRINTF("port1 led_startus is %0x\r\n",led_startus);
    
    PRINTF("\r\n");
    
    /* user led */
    RGPIO_PinInit(RGPIO4, 27, &pinConfig);
    RGPIO_PinInit(RGPIO4, 26, &pinConfig);
#endif
    
    /*config Sync0/1 IRQ*/
    XBAR_Init(kXBAR_DSC1);

    XBAR_SetSignalsConnection(kXBAR1_InputEcatSyncOut0, kXBAR1_OutputDma4MuxReq154);
    BLK_CTRL_WAKEUPMIX->XBAR_TRIG_SYNC_CTRL1 = 0x0;
    xbaraConfig.activeEdge                   = kXBAR_EdgeRising;
    xbaraConfig.requestType                  = kXBAR_RequestInterruptEnable;
    XBAR_SetOutputSignalConfig(kXBAR1_OutputDma4MuxReq154, &xbaraConfig);
    BLK_CTRL_WAKEUPMIX->XBAR_TRIG_SYNC_CTRL1 |= 0xff << 16;
    BLK_CTRL_WAKEUPMIX->XBAR_TRIG_SYNC_CTRL1 |= 0xff << 8;
    BLK_CTRL_WAKEUPMIX->XBAR_TRIG_SYNC_CTRL2 |= 3;
    BLK_CTRL_WAKEUPMIX->XBAR_TRIG_SYNC_CTRL2 |= 3 << 4;

    XBAR_SetSignalsConnection(kXBAR1_InputEcatSyncOut1, kXBAR1_OutputDma4MuxReq155);
    xbaraConfig.activeEdge  = kXBAR_EdgeRising;
    xbaraConfig.requestType = kXBAR_RequestInterruptEnable;
    XBAR_SetOutputSignalConfig(kXBAR1_OutputDma4MuxReq155, &xbaraConfig);

    do
    {
        intMask = 0x93;
        HW_EscWriteDWord(intMask, ESC_AL_EVENTMASK_OFFSET);
        intMask = 0;
        HW_EscReadDWord(intMask, ESC_AL_EVENTMASK_OFFSET);
    } while (intMask != 0x93);

    intMask = 0x00;

    HW_EscWriteDWord(intMask, ESC_AL_EVENTMASK_OFFSET);

    /*Enable GPT1*/
    GPT_GetDefaultConfig(&gptConfig);
    GPT_Init(GPT1, &gptConfig);
    gptFreq = CLOCK_GetRootClockFreq(kCLOCK_Root_Gpt1);
    GPT_SetClockDivider(GPT1, 100);
    GPT_SetOutputCompareValue(GPT1, kGPT_OutputCompare_Channel1, gptFreq / 100000);
    GPT_EnableInterrupts(GPT1, kGPT_OutputCompare1InterruptEnable);
    EnableIRQ(GPT1_IRQn);

#if 0 // DONE by GOAL
    /*Enable PDI IRQ*/
    EnableIRQ(ECAT_INT_IRQn);
    NVIC_EnableIRQ(XBAR1_CH0_CH1_IRQn);
#endif
    GPT_StartTimer(GPT1);
    return 0;
}

#if 0
void HW_Release(void)
{
}
#endif

void GPT1_IRQHandler(void)
{
    /* Clear interrupt flag.*/
    GPT_ClearStatusFlags(GPT1, kGPT_OutputCompare1Flag);
#if ECAT_TIMER_INT
    ECAT_CheckTimer();
#endif
    EcatTimerCnt++;

    SDK_ISR_EXIT_BARRIER;
}

#if 0
uint16_t HW_GetTimer(void)
{
    return EcatTimerCnt;
}

void HW_ClearTimer(void)
{
    EcatTimerCnt = 0;
}

void ENABLE_ESC_INT(void)
{
    NVIC_EnableIRQ(ECAT_INT_IRQn);
    NVIC_EnableIRQ(XBAR1_CH0_CH1_IRQn);
    NVIC_EnableIRQ(GPT1_IRQn);
}

void DISABLE_ESC_INT(void)
{
    NVIC_DisableIRQ(XBAR1_CH0_CH1_IRQn);
    NVIC_DisableIRQ(ECAT_INT_IRQn);
    NVIC_DisableIRQ(GPT1_IRQn);
}

void HW_SetLed(uint8_t RunLed, uint8_t ErrorLed)
{
}
#endif
