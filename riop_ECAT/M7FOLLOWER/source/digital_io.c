/*
* Copyright 2025 NXP
*
* NXP Proprietary. This software is owned or controlled by NXP and may only be used strictly
* in accordance with the applicable license terms.  By expressly accepting such terms or by downloading,
* installing, activating and/or otherwise using the software, you are agreeing that you have read,
* and that you agree to comply with and are bound by, such license terms.  If you do not agree to be bound by
* the applicable license terms, then you may not retain, install, activate or otherwise use the software.
*/

#include "FreeRTOS.h"
#include "task.h"

#ifdef GOAL
#if 0
#include "goal_ecat.h"
#include "goal_includes.h"
#endif
#include "goal_appl_ecat.h"
#include "goal_appl_ecat_objects.h"
#endif

#include "board.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "mcmgr.h"
#include "pin_mux.h"

#include "gl.h"

#include "api_riop.h"

/* Well done: Main function moved here from "flash zone". */
#include "mcmgr.h"

uint32_t fsm_mode;
unsigned char LED_status;
unsigned char di;
UINT32 adc_1;

uint8_t led_green = 0;
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
// Inline makes a problem under Debug
#define dwt_get() DWT->CYCCNT

// static riop_status_t last_err = 0;
static uint32_t time;
static uint32_t tim;
static uint32_t tim_led;
static int tim_mode = 200;
static int tim_first = 0;
static volatile uint16_t RPMsgRemoteReadyEventData = 0;

void digital_io_init(void)
{
	dwt_init();

	// uint16_t counter = 0;
	memset(&riop_command, 0, sizeof(riop_command));

	// riop_command.afe_command.afe_gpio_command.gpio_output_value = 4;
	riop_command.afe_command.afe_gpio_command.gpio_output_value = 0;
	// riop_command.afe_command.afe_gpio_command.gpio_pin_direction = 0x3FC;
	riop_command.afe_command.afe_gpio_command.gpio_pin_direction =
		0x3C0; // 4 outputs and 4 inputs, set via ecat
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
	riop_command.afe_command.afe_current_command.shunt_value = kShunt100;
	riop_command.afe_command.afe_current_command.measurement_enabled = 1;

	riop_command.afe_command.afe_temperature_command.is_dirty = 0;
	riop_command.afe_command.afe_temperature_command.measurement_enabled = 1;
	riop_command.afe_command.afe_temperature_command.samples_amt =
		128; // up to 6sec!!

	tim = dwt_get();
	tim_led = dwt_get();
}

void appl_loop(void)
{
	if (tim_first) {
		time = SystemCoreClock / 1000 * 200;
		tim_first = 0;
	} else {
		time = SystemCoreClock / 1000 * tim_mode;
	}

	static int lled = 0;

	uint32_t time_led;
	time_led = SystemCoreClock / 1000 * 100;
	uint32_t diff_led;
	diff_led = dwt_get();
	diff_led -= tim_led;
	// if(!lcnt--)
	if (diff_led > time_led) // max 4 sec
	{
		tim_led += time_led; // tim=dwt_get();
				     // lcnt=10*1000*1000;
				     // lcnt=100000/4;//Ecat engine delays us

		lled = !lled;
		if (led_green) {
			RGPIO_PinWrite(BOARD_USER_LED_RED_GPIO, BOARD_USER_LED_RED_GPIO_PIN, 0);
			RGPIO_PinWrite(BOARD_USER_LED_GREEN_GPIO, BOARD_USER_LED_GREEN_GPIO_PIN,
				       lled & 0x01);
		} else {
			RGPIO_PinWrite(BOARD_USER_LED_RED_GPIO, BOARD_USER_LED_RED_GPIO_PIN,
				       lled & 0x01);
			RGPIO_PinWrite(BOARD_USER_LED_GREEN_GPIO, BOARD_USER_LED_GREEN_GPIO_PIN,
				       0);
		}
#ifdef DEBUG_PRINT2
		DEBUG_PRINT2
#endif
	}

	uint32_t dd;
	dd = dwt_get();
	dd -= tim;
	if (dd > time) // max 4 sec
	{
		tim += time; // tim=dwt_get();

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

		static int fsm = 0;
		uint32_t *pfsm_mode;
		pfsm_mode = &fsm_mode;
		static int last_mode = 0;
		if (last_mode != *pfsm_mode) {
			fsm = 0;
			tim_first = 1;
			last_mode = *pfsm_mode;
			switch (*pfsm_mode) {
			default:
			case 0:
				tim_mode = 200;
				break;
			case 1:
				tim_mode = 20;
				riop_command.afe_command.afe_hvsig_command.is_dirty = 1;
				break;
			case 2:
				tim_mode = 20;
				riop_command.afe_command.afe_current_command.is_dirty = 1;
				break;
			case 3:
				tim_mode = 200;
				riop_command.afe_command.afe_lvsig_command.is_dirty = 1;
				break;
			case 4:
				tim_mode = 20;
				riop_command.afe_command.afe_gpio_command.is_dirty = 1;
				break;
			case 5:
				tim_mode = 20;
				riop_command.mcu_digital_output_command.is_dirty = 1;
				break;
			case 6:
				tim_mode = 3000;
				riop_command.afe_command.afe_temperature_command.is_dirty = 1;
				break;
			}
			//!!!!
			// tim_mode*=4;
		}
		switch (*pfsm_mode) {
		case 0:
		{
			// Automatic switching modes
			switch (++fsm) {
			default:
				fsm = 0;
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
				static uint32_t pass = 0;
				pass++;
				if (!(pass & 0x07))
					riop_command.afe_command.afe_lvsig_command.is_dirty = 1;
			} break;
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
		} break;
		case 1:
			break;
		case 2:
			break;
		case 3:
		{
			static uint32_t pass = 0;
			pass++;
			if (!(pass & 0x07))
				riop_command.afe_command.afe_lvsig_command.is_dirty = 1;
		} break;
		case 4:
			// Change detector with validator.
			static uint16_t afe_last;
			static uint8_t afe_last_val = 0;
			if (!afe_last_val) {
				afe_last = riop_command.afe_command.afe_gpio_command.gpio_output_value;
				afe_last_val = 1;
			}
			if (afe_last !=
			    riop_command.afe_command.afe_gpio_command.gpio_output_value) {
				afe_last = riop_command.afe_command.afe_gpio_command.gpio_output_value;
				riop_command.afe_command.afe_gpio_command.is_dirty = 1;
			}
			static uint16_t afe_last_d;
			static uint8_t afe_last_d_val = 0;
			// Another change detector
			if (!afe_last_d_val) {
				afe_last_d =
					riop_command.afe_command.afe_gpio_command.gpio_pin_direction;
				afe_last_d_val = 1;
			}
			if (afe_last_d !=
			    riop_command.afe_command.afe_gpio_command.gpio_pin_direction) {
				afe_last_d =
					riop_command.afe_command.afe_gpio_command.gpio_pin_direction;
				riop_command.afe_command.afe_gpio_command.is_dirty = 1;
			}
			break;
		case 5:
			// Change detector with validator.
			static uint8_t mcu_last;
			static uint8_t mcu_last_val = 0;
			if (!mcu_last_val) {
				mcu_last = riop_command.mcu_digital_output_command.mcu_digital_output;
				mcu_last_val = 1;
			}
			if (mcu_last !=
			    riop_command.mcu_digital_output_command.mcu_digital_output) {
				mcu_last = riop_command.mcu_digital_output_command.mcu_digital_output;
				riop_command.mcu_digital_output_command.is_dirty = 1;
			}
			break;
		case 6:
			riop_command.afe_command.afe_temperature_command.is_dirty = 1;
			break;
		}

		ICC_DataHandleTask(NULL);
	}
}
