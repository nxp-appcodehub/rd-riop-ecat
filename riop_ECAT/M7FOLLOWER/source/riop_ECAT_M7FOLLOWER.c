/*
* Copyright 2025 NXP
*
* NXP Proprietary. This software is owned or controlled by NXP and may only be used strictly
* in accordance with the applicable license terms.  By expressly accepting such terms or by downloading,
* installing, activating and/or otherwise using the software, you are agreeing that you have read,
* and that you agree to comply with and are bound by, such license terms.  If you do not agree to be bound by
* the applicable license terms, then you may not retain, install, activate or otherwise use the software.
*/

#include "icc_task.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "mcmgr.h"
#include "fsl_debug_console.h"

#include "fsl_rgpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "api_afe.h"
#include "gpio_task.h"
#include "api_riop.h"
#include "riop_feature_config.h"
#include "api_riop_common.h"
#include "afe_task.h"
#include "api_siggen.h"
#include "SIGGEN_task.h"
#include "gl.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define GPIO_STATUS_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 256)
#define GPIO_STATUS_PRIORITY   5

#define AFE_TASK_PRIORITY		7
#define AFE_TASK_STACK_SIZE		(configMINIMAL_STACK_SIZE + 2048)

#define SIGGEN_TASK_PRIORITY		6
#define SIGGEN_TASK_STACK_SIZE		(configMINIMAL_STACK_SIZE + 256)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

SemaphoreHandle_t riop_status_mutex = NULL;
QueueHandle_t afe_command_queue = NULL;
QueueHandle_t gpio_command_queue = NULL;
QueueHandle_t siggen_command_queue = NULL;
QueueHandle_t afe_gpio_command_queue = NULL;

QueueHandle_t afe_hvsig_sample_queue = NULL;
QueueHandle_t afe_lvsig_sample_queue = NULL;
QueueHandle_t afe_current_sample_queue = NULL;
QueueHandle_t afe_temp_sample_queue = NULL;
QueueHandle_t afe_gpio_input_status_queue = NULL;


extern NAFE_hdl_t gNafe_Hdl;
extern AFE_ctx_t gAfe_ctx;
extern SIGGEN_config_t gSIGGEN_config;
extern SIGGEN_ctx_t SIGGEN_ctx;

TaskHandle_t GPIO_TaskHandle = NULL;
TaskHandle_t AFE_TaskHandle = NULL;


/*******************************************************************************
 * Code
 ******************************************************************************/

int BOARD_InitHardware(void)
{
    /* Init board hardware.*/
    BOARD_ConfigMPU();

    /* BOARD_InitDebugConsole(); */

    SystemCoreClock = CLOCK_GetRootClockFreq(kCLOCK_Root_M7);
    return 0;
}

/*!
 * @brief Main function
 */
int riop_appl_init(void)
{
    PRINTF("RIOP Application Initialization..\r\n");

    riop_status_mutex = xSemaphoreCreateMutex();

    afe_command_queue = xQueueCreate(10, sizeof(afe_command_t));
    afe_gpio_command_queue = xQueueCreate(10, sizeof(uint16_t));
    siggen_command_queue = xQueueCreate(10, sizeof(siggen_control_command_t));
    gpio_command_queue = xQueueCreate(10, sizeof(uint8_t));

    afe_hvsig_sample_queue = xQueueCreate(10, AFE_MAX_EXTERNAL_CHANNELS*sizeof(float));
    afe_lvsig_sample_queue = xQueueCreate(10, 2*AFE_MAX_INTERNAL_CHANNELS*sizeof(float));
    afe_gpio_input_status_queue = xQueueCreate(10, sizeof(uint16_t));
    afe_current_sample_queue = xQueueCreate(10, AFE_MAX_CURRENT_CHANNELS*sizeof(float));
    afe_temp_sample_queue = xQueueCreate(10, sizeof(float));

    digital_io_init();

    if (xTaskCreate(GPIO_ControlTask, "GPIO status task", GPIO_STATUS_TASK_STACK_SIZE,
            &gAfe_ctx, GPIO_STATUS_PRIORITY, &GPIO_TaskHandle) != pdPASS)
	{
		PRINTF("GPIO_ControlTask() failed\n");
		while(1);
	}

    if (xTaskCreate(AFE_task, "afe task", AFE_TASK_STACK_SIZE,
            &gAfe_ctx, AFE_TASK_PRIORITY, &AFE_TaskHandle) != pdPASS) {
		PRINTF("AFE_task() creation failed\n");
		while(1);
	}

    if (xTaskCreate(SIGGEN_ControlTask, "siggen task", SIGGEN_TASK_STACK_SIZE,
            &SIGGEN_ctx, SIGGEN_TASK_PRIORITY, &AFE_TaskHandle) != pdPASS) {
		PRINTF("SIGGEN_ControlTask() creation failed\n");
		while(1);
	}

    if (AFE_Init(&gNafe_Hdl) != kStatus_RIOP_Ok) {
    	PRINTF("AFE_Init() creation failed\n");
    	while(1);
    }

    if(SIGGEN_Init(&gSIGGEN_config) != kStatus_RIOP_Ok) {
    	PRINTF("SIGGEN_Init() creation failed\n");
    	while(1);
    }

    RIOP_StatusShmInit();
    return 0;
}
