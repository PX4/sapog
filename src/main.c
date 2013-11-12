/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Pavel Kirienko <pavel.kirienko@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <ch.h>
#include <hal.h>
#include <assert.h>
#include <unistd.h>
#include "sys/sys.h"
#include "motor/motor.h"
#include "motor/pwm.h"
#include "motor/adc.h"

static void led_set_status(bool state)
{
	palWritePad(GPIO_PORT_LED_STATUS, GPIO_PIN_LED_STATUS, !state);
}

static void led_set_error(bool state)
{
	palWritePad(GPIO_PORT_LED_ERROR, GPIO_PIN_LED_ERROR, !state);
}

void application_halt_hook(void)
{
	motor_emergency();
	led_set_error(true);
	led_set_status(true);
}

int main(void)
{
	halInit();
	chSysInit();
	sdStart(&STDOUT_SD, NULL);

	led_set_status(false);
	led_set_error(false);
	lowsyslog("\nPX4ESC: starting\n");

	usleep(3000000);

	motor_init();
	//assert(0 == motor_test_hardware());
	motor_test_hardware();

	if (motor_test_motor())
		lowsyslog("Motor is not connected or damaged\n");
	else
		lowsyslog("Motor is OK\n");

	lowsyslog("Initialization done\n");
	motor_beep(1000, 150);

	enum motor_pwm_phase_manip manip_cmd[3] = {
		MOTOR_PWM_MANIP_FLOATING,
		MOTOR_PWM_MANIP_FLOATING,
		MOTOR_PWM_MANIP_FLOATING
	};

	bool reverse = false;

	while (1) {
		motor_print_debug_info();

		struct motor_adc_sample sample = motor_adc_get_last_sample();
		lowsyslog("%i %i %i\n",
			sample.raw_phase_values[0], sample.raw_phase_values[1], sample.raw_phase_values[2]);

		const int ch = sdGet(&STDOUT_SD);

		if (ch >= '0' && ch <= ('9' + 1)) {
			const int percent = (ch - '0') * 10;
			const uint16_t duty_cycle = (0xFFFF * percent) / 100;
			lowsyslog("Duty cycle: %i%% (%x)\n", percent, duty_cycle);

			if (motor_get_state() == MOTOR_STATE_IDLE)
				motor_start(duty_cycle, 0xFFFF * 0.2, reverse);  // Engage 20% by default
			else
				motor_set_duty_cycle(duty_cycle);
		} else if (ch >= 'a' && ch <= 'c') {
			motor_stop();
			const int phase_num = ch - 'a';
			if (phase_num >= 0 && phase_num < 3) {
				lowsyslog("Phase %i; enter the command (0 lo, 1 hi, 2 float, 3 half)\n", phase_num);
				const int cmd = sdGet(&STDOUT_SD) - '0';

				if (cmd >= 0 && cmd < 4) {
					lowsyslog("Command %i\n", cmd);
					manip_cmd[phase_num] = (enum motor_pwm_phase_manip)cmd;
					motor_pwm_manip(manip_cmd);
				}
				lowsyslog("New state: %i, %i, %i\n", manip_cmd[0], manip_cmd[1], manip_cmd[2]);
			}
		} else if (ch == '+') {
			motor_beep(1000, 150);
			motor_beep(3000, 150);
			motor_beep(7000, 150);
		} else if (ch == '-') {
			motor_beep(500, 1000);
		} else if (ch == 'r') {
			reverse = !reverse;
			lowsyslog("Reverse %s\n", reverse ? "ON" : "OFF");
		}
	}
	return 0;
}
