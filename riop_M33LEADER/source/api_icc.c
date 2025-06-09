/*
* Copyright 2025 NXP
*
* NXP Proprietary. This software is owned or controlled by NXP and may only be used strictly
* in accordance with the applicable license terms.  By expressly accepting such terms or by downloading,
* installing, activating and/or otherwise using the software, you are agreeing that you have read,
* and that you agree to comply with and are bound by, such license terms.  If you do not agree to be bound by
* the applicable license terms, then you may not retain, install, activate or otherwise use the software.
*/

#include "api_icc.h"

#include "api_riop_common.h"

#define REMOTE_EPT_ADDR               (30U)

riop_status_t ICC_WriteData(icc_handle_t * handle, void *data, uint32_t max_length, uint32_t timeout)
{
	uint32_t status = RL_FALSE;

	status = rpmsg_lite_send(handle->rpmsg_instance, handle->ept_instance, REMOTE_EPT_ADDR, data, max_length, timeout);

	if(status != RL_SUCCESS)
	{
		return kStatus_RIOP_Err;
	}

	return kStatus_RIOP_Ok;
}
