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

// TODO: rewrite in C++

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ch.h>
#include <hal.h>
#include <shell.h>
#include <zubax_chibios/os.hpp>
#include <motor/motor.h>
#include "console.hpp"

#pragma GCC diagnostic ignored "-Wunused-parameter"

static void cmd_cfg(BaseSequentialStream *, int argc, char *argv[])
{
	// TODO: refuse to save/erase while the motor is running
	os::config::executeCLICommand(argc, argv);
}

static void cmd_reset(BaseSequentialStream *chp, int argc, char *argv[])
{
	chThdSleep(MS2ST(100)); // Flush the serial buffers
	NVIC_SystemReset();
}

static void cmd_beep(BaseSequentialStream *chp, int argc, char *argv[])
{
	if (argc > 0 && !strcmp(argv[0], "help")) {
		puts("beep [freq_hz [duration_msec]]");
		return;
	}

	int freq = 500;
	if (argc > 0) {
		freq = atoi(argv[0]);
	}

	int duration = 300;
	if (argc > 1) {
		duration = atoi(argv[1]);
	}

	motor_beep(freq, duration);
}

static void cmd_stat(BaseSequentialStream *chp, int argc, char *argv[])
{
	float voltage = 0, current = 0;
	motor_get_input_voltage_current(&voltage, &current);

	std::printf("Power V/A     %-9f %f\n", voltage, current);
	std::printf("RPM/DC        %-9u %f\n", motor_get_rpm(), motor_get_duty_cycle());
	std::printf("Active limits %i\n", motor_get_limit_mask());
	std::printf("ZC failures   %lu\n", (unsigned long)motor_get_zc_failures_since_start());
}

static void cmd_test(BaseSequentialStream *chp, int argc, char *argv[])
{
	puts("Hardware test...");
	int res = motor_test_hardware();
	if (res) {
		std::printf("FAILED %i\n", res);
	} else {
		puts("OK");
	}

	puts("Motor test...");
	res = motor_test_motor();
	puts(res ? "Not connected" : "Connected");
}

static void cmd_dc(BaseSequentialStream *chp, int argc, char *argv[])
{
	static const int TTL_MS = 30000;

	if (argc == 0) {
		motor_stop();
		puts("Usage:\n"
			"  dc <duty cycle>\n"
			"  dc arm");
		return;
	}

	// Safety check
	static bool _armed = false;
	if (!strcmp(argv[0], "arm")) {
		_armed = true;
		puts("OK");
		return;
	}
	if (!_armed) {
		puts("Error: Not armed");
		return;
	}

	const float value = atoff(argv[0]);
	std::printf("Duty cycle %f\n", value);
	motor_set_duty_cycle(value, TTL_MS);
}

static void cmd_rpm(BaseSequentialStream *chp, int argc, char *argv[])
{
	static const int TTL_MS = 30000;

	if (argc == 0) {
		motor_stop();
		puts("Usage:\n"
			"  rpm <RPM>\n"
			"  rpm arm");
		return;
	}

	// Safety check
	static bool _armed = false;
	if (!strcmp(argv[0], "arm")) {
		_armed = true;
		puts("OK");
		return;
	}
	if (!_armed) {
		puts("Error: Not armed");
		return;
	}

	long value = (long)atoff(argv[0]);
	value = (value < 0) ? 0 : value;
	value = (value > 65535) ? 65535 : value;
	std::printf("RPM %li\n", value);
	motor_set_rpm((unsigned)value, TTL_MS);
}

static void cmd_md(BaseSequentialStream *chp, int argc, char *argv[])
{
	motor_print_debug_info();
}

static void cmd_m(BaseSequentialStream *chp, int argc, char *argv[])
{
	motor_execute_cli_command(argc, (const char**)argv);
}

#define COMMAND(cmd)    {#cmd, cmd_##cmd},
static const ShellCommand _commands[] =
{
	COMMAND(cfg)
	COMMAND(reset)
	COMMAND(beep)
	COMMAND(stat)
	COMMAND(test)
	COMMAND(dc)
	COMMAND(rpm)
	COMMAND(md)
	COMMAND(m)
	{NULL, NULL}
};

// --------------------------

static const ShellConfig _config = {(BaseSequentialStream*)&STDOUT_SD, _commands};

static THD_WORKING_AREA(_wa_shell, 1024);

void console_init(void)
{
	shellInit();

	ASSERT_ALWAYS(shellCreateStatic(&_config, _wa_shell, sizeof(_wa_shell), LOWPRIO));
}
