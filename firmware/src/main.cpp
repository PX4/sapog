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
#include <config/config.h>
#include <sys.h>
#include <led.hpp>
#include <console.h>
#include <watchdog.h>
#include <pwm_input.h>
#include <motor/motor.h>
#include <uavcan_node/uavcan_node.hpp>

namespace
{

led::Overlay led_ctl;

int init()
{
	int res = 0;

	/*
	 * Indication
	 */
	led::init();
	led_ctl.set(led::Color::PALE_WHITE);

	/*
	 * Config
	 */
	res = config_init();
	if (res) {
		return res;
	}

	/*
	 * Safety
	 */
	watchdog_init();

	/*
	 * Motor control (must be initialized earlier than communicaton interfaces)
	 */
	res = motor_init();
	if (res) {
		return res;
	}

	/*
	 * PWM input
	 */
	pwm_input_init();

	/*
	 * UAVCAN node
	 */
	res = uavcan_node::init();
	if (res) {
		return res;
	}

	/*
	 * Self test
	 */
	res = motor_test_hardware();
	if (res) {
		return res;
	}
	lowsyslog("Hardware OK\n");

	if (motor_test_motor()) {
		lowsyslog("Motor is not connected or damaged\n");
	} else {
		lowsyslog("Motor OK\n");
	}
	return 0;
}

__attribute__((noreturn))
void die(int status)
{
	::usleep(100000);
	lowsyslog("Init failed (%i)\n", status);
	// Really there is nothing left to do; just sit there and beep sadly:
	while (1) {
		motor_beep(100, 400);
		uavcan_node::set_node_status_critical();
		led::emergency_override(led::Color::RED);
		sleep(3);
	}
}

void do_startup_beep()
{
	motor_beep(1000, 100);
	::usleep(200 * 1000);
	motor_beep(1000, 100);
}

void print_banner()
{
	lowsyslog("\n\n\n");
	lowsyslog("\x1b\x5b\x48");      // Home sweet home
	lowsyslog("\x1b\x5b\x32\x4a");  // Clear
	lowsyslog("PX4ESC\n");
	lowsyslog("Git commit hash 0x%08x\n", GIT_HASH);
}

}

void application_halt_hook(void)
{
	motor_emergency();
	led::emergency_override(led::Color::RED);
}

int main()
{
	halInit();
	chSysInit();
	sdStart(&STDOUT_SD, NULL);

	usleep(300000);
	print_banner();

	const int init_status = init();

	const int watchdog_id = watchdog_create(10000);

	console_init();

	if (init_status) {
		die(init_status);
	}

	do_startup_beep();

	motor_confirm_initialization();

	chThdSetPriority(LOWPRIO);

	uavcan_node::set_node_status_ok();

	/*
	 * Here we run some high-level self diagnostics, indicating the system health via UAVCAN and LED.
	 */
	while (1) {
		watchdog_reset(watchdog_id);

		if (motor_is_blocked()) {
			led_ctl.set(led::Color::YELLOW);
			uavcan_node::set_node_status_critical();
		} else {
			led_ctl.set(led::Color::DARK_GREEN);
			uavcan_node::set_node_status_ok();
		}
		::usleep(100 * 1000);
	}

	return 0;
}
