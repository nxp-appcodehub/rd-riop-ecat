/*
* Copyright 2025 NXP
*
* NXP Proprietary. This software is owned or controlled by NXP and may only be used strictly
* in accordance with the applicable license terms.  By expressly accepting such terms or by downloading,
* installing, activating and/or otherwise using the software, you are agreeing that you have read,
* and that you agree to comply with and are bound by, such license terms.  If you do not agree to be bound by
* the applicable license terms, then you may not retain, install, activate or otherwise use the software.
*/
#ifndef _GL_H_
#define _GL_H_
//Global includes, global means the embedded viewpoint.

#include "api_riop.h"
#include "icc_task.h"
//Added these lines from ecat_hw.h as part M33 -> M7 migration
#ifndef UINT16
#define UINT16 uint16_t
#define UINT8 uint8_t
#define UINT32 uint32_t
#endif

extern uint8_t led_green;
extern uint32_t fsm_mode;
extern UINT32 adc_1;
extern unsigned char LED_status;
extern unsigned char di;
extern riop_board_status_t g_boardStatus;
extern riop_command_t riop_command;

int ecat_stack_init(void);
void digital_io_init(void);

#endif
