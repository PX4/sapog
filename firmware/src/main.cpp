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
#include <cassert>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <unistd.h>
#include <board/board.hpp>
#include <board/led.hpp>
#include <console.hpp>
#include <pwm_input.h>
#include <motor/motor.h>
#include <uavcan_node/uavcan_node.hpp>


namespace
{

static constexpr unsigned WATCHDOG_TIMEOUT = 10000;

board::LEDOverlay led_ctl;

os::watchdog::Timer init()
{
	auto wdt = board::init(WATCHDOG_TIMEOUT);

	led_ctl.set(board::LEDColor::PALE_WHITE);

	// Motor control (must be initialized earlier than communicaton interfaces)
	int res = motor_init();
	if (res < 0) {
		board::die(res);
	}

	// PWM input
	pwm_input_init();

	// UAVCAN node
	res = uavcan_node::init();
	if (res < 0) {
		board::die(res);
	}

	// Self test
	res = motor_test_hardware();
	if (res != 0) {
		board::die(res);
	}

	if (motor_test_motor()) {
		os::lowsyslog("Motor is not connected or damaged\n");
	}

	// Initializing console after delay to ensure that CLI is flushed
	usleep(300000);
	console_init();

	return wdt;
}

void do_startup_beep()
{
	motor_beep(1000, 100);
	::usleep(200 * 1000);
	motor_beep(1000, 100);
}

}

namespace os
{

void applicationHaltHook()
{
	motor_emergency();
	board::led_emergency_override(board::LEDColor::RED);
}

}

int main()
{
	auto wdt = init();

	chThdSetPriority(LOWPRIO);

	do_startup_beep();

	motor_confirm_initialization();

	uavcan_node::set_node_status_ok();

	/*
	 * Here we run some high-level self diagnostics, indicating the system health via UAVCAN and LED.
	 */
	while (!os::isRebootRequested()) {
		wdt.reset();

		if (motor_is_blocked()) {
			led_ctl.set(board::LEDColor::YELLOW);
			uavcan_node::set_node_status_critical();
		} else {
			led_ctl.set(board::LEDColor::DARK_GREEN);
			uavcan_node::set_node_status_ok();
		}
		::usleep(10 * 1000);
	}

	::usleep(100 * 1000);
	motor_stop();
	board::reboot();

	return 0;
}
