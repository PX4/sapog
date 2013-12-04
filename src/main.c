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
#include <errno.h>
#include <math.h>
#include <unistd.h>
#include "sys/sys.h"
#include "motor_lowlevel/motor.h"
#include "motor_lowlevel/pwm.h"
#include "motor_lowlevel/adc.h"
#include "motor_manager/motormgr.h"
#include <config.h>

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

CONFIG_PARAM_BOOL("bool", false)
CONFIG_PARAM_INT("int", 42, 0, 123)
CONFIG_PARAM_FLOAT("float", 0.5, 0.0, 1.0)

void config_print(const char* name)
{
	struct config_param par;
	const int res = config_get_descr(name, &par);
	if (res)
		lowsyslog("Config param: %s UNKNOWN\n", name);
	else
		lowsyslog("Config param: %s [%f; %f] (%f) %f\n",
			par.name, par.min, par.max, par.default_, config_get(name));
}

void config_list(void)
{
	for (int i = 0;; i++) {
		const char* name = config_name_by_index(i);
		if (!name)
			break;
		config_print(name);
	}
}

void config_test(void)
{
	config_init();

	bool save_later = true;
	if (config_get("bool")) {
		assert(!config_erase());
		lowsyslog("Config erased\n");
		save_later = false;
	}

	config_list();
	lowsyslog("\n");

	assert(-EINVAL == config_set("bool", 123));
	assert(-EINVAL == config_set("int", -2));
	assert(-EINVAL == config_set("int", 4.3));
	assert(-EINVAL == config_set("float", 2));
	assert(-EINVAL == config_set("float", NAN));
	assert(-ENOENT == config_set("nonexistent", 1));

	config_list();
	lowsyslog("\n");

	assert(0 == config_set("bool", true));
	assert(0 == config_set("int", 73));
	assert(0 == config_set("float", 0.75));

	config_list();
	lowsyslog("\n");

	if (save_later) {
		assert(!config_save());
		lowsyslog("Config saved\n");
	}
}

void run_test_serial(void)
{
	enum motor_pwm_phase_manip manip_cmd[3] = {
		MOTOR_PWM_MANIP_FLOATING,
		MOTOR_PWM_MANIP_FLOATING,
		MOTOR_PWM_MANIP_FLOATING
	};

	while (1) {
		motor_print_debug_info();
		float vtg, cur;
		motormgr_get_input_voltage_current(&vtg, &cur);
		lowsyslog("Voltage: %f V, current: %f A, DC: %f, RPM: %u\n",
			vtg, cur, motormgr_get_duty_cycle(), motormgr_get_rpm());

		struct motor_adc_sample sample = motor_adc_get_last_sample();
		lowsyslog("%i %i %i | %i %i\n",
			sample.phase_values[0], sample.phase_values[1], sample.phase_values[2],
			sample.input_voltage, sample.input_current);

		const int ch = sdGet(&STDOUT_SD);

		if (ch >= '0' && ch <= ('9' + 1)) {
			const float duty_cycle = 0.1f * (ch - '0');
			lowsyslog("Duty cycle: %.1f%%\n", duty_cycle * 100);
			motormgr_set_duty_cycle(duty_cycle, 10000);
			//motormgr_set_rpm(duty_cycle * 1000 * 10);
		} else if (ch >= 'a' && ch <= 'c') {
			motormgr_set_duty_cycle(0.0, 0);
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
		} else if (ch == ' ') {
			motormgr_set_duty_cycle(0.0, 0);
			for (int i = 0; i < 3; i++)
				manip_cmd[i] = MOTOR_PWM_MANIP_FLOATING;
		}

		led_set_error(motormgr_get_limit_mask());
	}
}

void run_plot(void)
{
	motor_start(0.2, 0.6, false);
	while (1) {
		usleep(50000);
		const uint32_t per = motor_get_comm_period_hnsec();
		lowsyslog("$ %u\n", (unsigned)per);
	}
}

int main(void)
{
	halInit();
	chSysInit();
	sdStart(&STDOUT_SD, NULL);

	led_set_status(false);
	led_set_error(false);
	lowsyslog("\nPX4ESC: starting\n");

	//config_test();
	lowsyslog("Config test OK\n");

	usleep(3000000);

	assert(0 == motormgr_init());
	assert(0 == motor_test_hardware());
	//motor_test_hardware();

	if (motor_test_motor())
		lowsyslog("Motor is not connected or damaged\n");
	else
		lowsyslog("Motor is OK\n");

	lowsyslog("Initialization done\n");
	motor_beep(1000, 150);

	run_test_serial();
	//run_plot();

	return 0;
}
