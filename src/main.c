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
#include "motor/adc.h"
#include "motor/pwm.h"
#include "motor/timer.h"
#include "motor/selftest.h"

static void led_set_status(bool state)
{
	palWritePad(GPIO_PORT_LED_STATUS, GPIO_PIN_LED_STATUS, !state);
}

static void led_set_error(bool state)
{
	palWritePad(GPIO_PORT_LED_ERROR, GPIO_PIN_LED_ERROR, !state);
}

void motor_timer_callback(void)
{
//	motor_timer_set_relative(5 * HNSEC_PER_USEC);
}

void motor_adc_sample_callback(const struct motor_adc_sample* sample)
{
	motor_timer_set_relative(0);
}

void application_halt_hook(void)
{
	motor_pwm_emergency();
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

	motor_pwm_init();
	motor_timer_init();
	motor_adc_init();
	motor_adc_enable(true);

	assert(0 == motor_selftest());
	lowsyslog("Initialization done\n");
	motor_pwm_beep(1000, 100);

	motor_timer_set_relative(0);

	enum motor_pwm_phase_manip manip_cmd[3] = {
		MOTOR_PWM_MANIP_FLOATING,
		MOTOR_PWM_MANIP_FLOATING,
		MOTOR_PWM_MANIP_FLOATING
	};

	while (1) {
		const int ch = sdGet(&STDOUT_SD);

		const int phase_num = ch - '0';
		if (phase_num >= 0 && phase_num < 3) {
			lowsyslog("Phase %i; enter the command (0 lo, 1 hi, 2 float, 3 half)\n", phase_num);
			const int cmd = sdGet(&STDOUT_SD) - '0';

			if (cmd >= 0 && cmd < 4) {
				lowsyslog("Command %i\n", cmd);
				manip_cmd[phase_num] = (enum motor_pwm_phase_manip)cmd;
				motor_pwm_manip(manip_cmd);
			}
			lowsyslog("New state: %i, %i, %i\n", manip_cmd[0], manip_cmd[1], manip_cmd[2]);
		} else if (ch == '+') {
			motor_pwm_beep(1000, 100);
			motor_pwm_beep(3000, 100);
			motor_pwm_beep(7000, 100);
		} else if (ch == '-') {
			motor_pwm_beep(7000, 1000);
		}

		struct motor_adc_sample sample = motor_adc_get_last_sample();
		lowsyslog("%u %i %i %i\n", (unsigned)(sample.timestamp / HNSEC_PER_MSEC),
		          sample.raw_phase_values[0], sample.raw_phase_values[1], sample.raw_phase_values[2]);
	}
	return 0;
}
