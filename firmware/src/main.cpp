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
#include <zubax_chibios/os.hpp>
#include <led.hpp>
#include <console.hpp>
#include <pwm_input.h>
#include <motor/motor.h>
#include <uavcan_node/uavcan_node.hpp>

// TODO: relocate
static const volatile struct __attribute__((packed))
{
    std::uint8_t signature[8] = {'A','P','D','e','s','c','0','0'};
    std::uint64_t image_crc = 0;
    std::uint32_t image_size = 0;
    std::uint32_t vcs_commit = GIT_HASH;
    std::uint8_t major_version = FW_VERSION_MAJOR;
    std::uint8_t minor_version = FW_VERSION_MINOR;
    std::uint8_t reserved[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
} _app_descriptor __attribute__((section(".app_descriptor")));

namespace
{

static constexpr unsigned WATCHDOG_TIMEOUT = 10000;

led::Overlay led_ctl;

__attribute__((noreturn))
void die(int status)
{
	::usleep(100000);
	os::lowsyslog("Init failed (%i)\n", status);
	// Really there is nothing left to do; just sit there and beep sadly:
	while (1) {
		motor_beep(100, 400);
		uavcan_node::set_node_status_critical();
		led::emergency_override(led::Color::RED);
		sleep(3);
	}
}

os::watchdog::Timer init()
{
	// Watchdog
	os::watchdog::init();
	os::watchdog::Timer wdt;
	wdt.startMSec(WATCHDOG_TIMEOUT);

	// Config
	const int config_init_res = os::config::init();
	if (config_init_res < 0)
	{
		die(config_init_res);
	}

	// Banner
	os::lowsyslog("PX4 Sapog %d.%d.%08x / %d %s\n",
		  FW_VERSION_MAJOR, FW_VERSION_MINOR, GIT_HASH, config_init_res,
		  os::watchdog::wasLastResetTriggeredByWatchdog() ? "WDTRESET" : "OK");

	// Indication
	led::init();
	led_ctl.set(led::Color::PALE_WHITE);

	// Motor control (must be initialized earlier than communicaton interfaces)
	int res = motor_init();
	if (res < 0) {
		die(res);
	}

	// PWM input
	pwm_input_init();

	// UAVCAN node
	res = uavcan_node::init();
	if (res < 0) {
		die(res);
	}

	// Self test
	res = motor_test_hardware();
	if (res != 0) {
		die(res);
	}

	if (motor_test_motor()) {
		os::lowsyslog("Motor is not connected or damaged\n");
	}

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
	led::emergency_override(led::Color::RED);
}

}

int main()
{
	halInit();
	chSysInit();
	sdStart(&STDOUT_SD, NULL);

	auto wdt = init();

	console_init();

	usleep(300000);
	do_startup_beep();

	motor_confirm_initialization();

	chThdSetPriority(LOWPRIO);

	uavcan_node::set_node_status_ok();

	/*
	 * Here we run some high-level self diagnostics, indicating the system health via UAVCAN and LED.
	 */
	while (1) {
		wdt.reset();

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
