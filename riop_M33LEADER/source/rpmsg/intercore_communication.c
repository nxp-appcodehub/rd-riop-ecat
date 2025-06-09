/*
* Copyright 2025 NXP
*
* NXP Proprietary. This software is owned or controlled by NXP and may only be used strictly
* in accordance with the applicable license terms.  By expressly accepting such terms or by downloading,
* installing, activating and/or otherwise using the software, you are agreeing that you have read,
* and that you agree to comply with and are bound by, such license terms.  If you do not agree to be bound by
* the applicable license terms, then you may not retain, install, activate or otherwise use the software.
*/

#include "rpmsg_lite.h"
#include "rpmsg_ns.h"
#include "intercore_communication.h"
#include "api_icc.h"
#include "clock_config.h"
#include "api_riop.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define LOCAL_EPT_ADDR                (40U)
#define REMOTE_EPT_ADDR               (30U)

#define RPMSG_LITE_LINK_ID (RL_PLATFORM_IMXRT1180_M33_M7_LINK_ID)

/*******************************************************************************
 * Variables
 ******************************************************************************/
char rpmsg_lite_base[SH_MEM_TOTAL_SIZE] __attribute__((section(".noinit.$rpmsg_sh_mem")));


icc_handle_t g_icc_handle;
riop_board_status_t g_boardStatus;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/



/*******************************************************************************
 * Code
 ******************************************************************************/

/* This is the read callback, note we are in a task context when this callback
is invoked, so kernel primitives can be used freely */
static int32_t my_ept_read_cb(void *payload, uint32_t payload_len, uint32_t src, void *priv)
{
	memcpy(&g_boardStatus, payload, sizeof(riop_board_status_t));

	return kStatus_RIOP_Ok;
}

void RPMSG_CommunicationInit()
{
	g_icc_handle.rpmsg_instance = rpmsg_lite_master_init(rpmsg_lite_base, SH_MEM_TOTAL_SIZE, RPMSG_LITE_LINK_ID, RL_NO_FLAGS, &g_icc_handle.rpmsg_ctxt);
	g_icc_handle.ept_instance = rpmsg_lite_create_ept(g_icc_handle.rpmsg_instance, LOCAL_EPT_ADDR, my_ept_read_cb, (void *)&g_icc_handle.has_received, &g_icc_handle.ns_ept_context.ept_ctxt);
}
