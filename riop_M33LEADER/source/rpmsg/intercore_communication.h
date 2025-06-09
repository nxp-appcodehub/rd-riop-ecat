/*
* Copyright 2025 NXP
*
* NXP Proprietary. This software is owned or controlled by NXP and may only be used strictly
* in accordance with the applicable license terms.  By expressly accepting such terms or by downloading,
* installing, activating and/or otherwise using the software, you are agreeing that you have read,
* and that you agree to comply with and are bound by, such license terms.  If you do not agree to be bound by
* the applicable license terms, then you may not retain, install, activate or otherwise use the software.
*/

#ifndef RPMSG_INTERCORE_COMMUNICATION_H_
#define RPMSG_INTERCORE_COMMUNICATION_H_


#define SH_MEM_TOTAL_SIZE (6144U)

/* Well done: Moved from local declaration above main not to give up structure.*/
/* Address of memory, from which the secondary core will boot */
#define CORE1_BOOT_ADDRESS    (void *)0x303C0000
#define CORE1_KICKOFF_ADDRESS 0x0

#define APP_RPMSG_READY_EVENT_DATA    (1U)

extern char rpmsg_lite_base[SH_MEM_TOTAL_SIZE];

void RPMSG_CommunicationInit(void);




#endif /* RPMSG_INTERCORE_COMMUNICATION_H_ */
