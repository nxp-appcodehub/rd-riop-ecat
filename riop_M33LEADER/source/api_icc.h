/*
* Copyright 2025 NXP
*
* NXP Proprietary. This software is owned or controlled by NXP and may only be used strictly
* in accordance with the applicable license terms.  By expressly accepting such terms or by downloading,
* installing, activating and/or otherwise using the software, you are agreeing that you have read,
* and that you agree to comply with and are bound by, such license terms.  If you do not agree to be bound by
* the applicable license terms, then you may not retain, install, activate or otherwise use the software.
*/

#ifndef API_ICC_H_
#define API_ICC_H_


#include "fsl_common.h"
#include "intercore_communication.h"
#include "rpmsg_lite.h"
#include "rpmsg_ns.h"
#include "api_riop_common.h"



typedef struct _icc_handle
{
	struct rpmsg_lite_instance *volatile rpmsg_instance;
    struct rpmsg_lite_endpoint *volatile ept_instance;
    rpmsg_ns_static_context ns_ept_context;
    struct rpmsg_lite_instance rpmsg_ctxt;
    volatile uint32_t remote_addr;
    volatile int32_t has_received;
	volatile rpmsg_ns_handle ns_handle;
	struct rpmsg_lite_ept_static_context ept_context;
} icc_handle_t;


riop_status_t ICC_WriteData(icc_handle_t * handle, void *data, uint32_t max_length, uint32_t timeout);


#endif /* API_ICC_H_ */
