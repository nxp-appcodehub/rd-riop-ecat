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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pin_mux.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "FreeRTOS.h"
#include "task.h"

#include "fsl_common.h"
#include "mcmgr.h"
#include "api_riop.h"
#include "api_riop_common.h"
#include "gl.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define RPMSG_LITE_LINK_ID            (RL_PLATFORM_IMXRT1180_M33_M7_LINK_ID)
#define APP_RPMSG_READY_EVENT_DATA (1U)
#define LOCAL_EPT_ADDR (30U)
#define RPMSG_LITE_NS_ANNOUNCE_STRING "rpmsg-openamp-demo-channel"

#define RPMSG_DATA_HANDLE_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 1024)
#define RPMSG_DATA_HANDLE_TASK_PRIORITY   13

/*******************************************************************************
 * Prototypes
 ******************************************************************************/


/*******************************************************************************
 * Variables
 ******************************************************************************/
#if 0
icc_handle_t g_icc_handle;
#endif
riop_board_status_t g_boardStatus;

extern QueueHandle_t afe_command_queue;
extern QueueHandle_t gpio_command_queue;
extern QueueHandle_t siggen_command_queue;


#if 0
/*******************************************************************************
 * Code
 ******************************************************************************/
static void app_nameservice_isr_cb(uint32_t new_ept, const char *new_ept_name, uint32_t flags, void *user_data)
{
}
#endif

/*TODO: M5 Rel: Change name of the function. It is no more task*/
void ICC_DataHandleTask(void *data)
{
#if 0
	uint32_t len = 0;

	riop_status_t status = kStatus_RIOP_Err;

	riop_command_t riop_command;
	g_boardStatus_t g_boardStatus;

	while(1)
	{
		status = ICC_ReadData(&g_icc_handle, (char *)&riop_command, sizeof(riop_command), &len, portMAX_DELAY);
		if(status == kStatus_RIOP_Ok)
		{
#endif
			if(riop_command.siggen_control_command.is_dirty)
			{
				xQueueSend(siggen_command_queue, &riop_command.siggen_control_command, 0);
			}

			if(riop_command.mcu_digital_output_command.is_dirty)
			{
				xQueueSend(gpio_command_queue, &riop_command.mcu_digital_output_command.mcu_digital_output, 0);
			}

			if(		(riop_command.afe_command.afe_hvsig_command.is_dirty)
				|| 	(riop_command.afe_command.afe_lvsig_command.is_dirty)
				|| 	(riop_command.afe_command.afe_gpio_command.is_dirty)
				||  (riop_command.afe_command.afe_current_command.is_dirty)
				||  (riop_command.afe_command.afe_temperature_command.is_dirty)
				||  (riop_command.afe_command.afe_calibration_command.is_dirty))
			{
				xQueueSend(afe_command_queue, &riop_command.afe_command , 0);
			}
#if 0
		}
#endif
		/*Read riop status from gRiop_status_shm*/
		RIOP_GetBoardStatusForICC(&g_boardStatus);
#if 0
		/*Send riop status to cm33 core*/
		ICC_WriteData(&g_icc_handle, (char *)&g_boardStatus, sizeof(g_boardStatus), 0);
	}
#endif
}

