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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ch.h>
#include <hal.h>
#include <shell.h>
#include <config/config.h>
#include "console.h"
#include "motor_lowlevel/motor.h"
#include "motor_lowlevel/pwm.h"
#include "motor_manager/motormgr.h"

static char* getline(const char* prompt);
static void print_status(int err);


static int print_param(const char* name, bool verbose)
{
	static int _max_name_len;
	if (_max_name_len == 0) {
		for (int i = 0;; i++) {
			const char* nm = config_name_by_index(i);
			if (!nm)
				break;
			int len = strlen(nm);
			if (len > _max_name_len)
				_max_name_len = len;
		}
	}

	struct config_param par;
	const int res = config_get_descr(name, &par);
	if (res)
		return res;

	if (par.type == CONFIG_TYPE_FLOAT) {
		lowsyslog("%-*s = %-12f", _max_name_len, name, config_get(name));
		if (verbose)
			lowsyslog("[%f; %f] (%f)", par.min, par.max, par.default_);
	} else {
		lowsyslog("%-*s = %-12i", _max_name_len, name, (int)config_get(name));
		if (verbose)
			lowsyslog("[%i; %i] (%i)", (int)par.min, (int)par.max, (int)par.default_);
	}
	puts("");
	return 0;
}

static void cmd_cfg(BaseSequentialStream *chp, int argc, char *argv[])
{
	const char* const command = (argc < 1) ? "" : argv[0];

	if (!strcmp(command, "list")) {
		for (int i = 0;; i++) {
			const char* name = config_name_by_index(i);
			if (!name)
				break;
			const int res = print_param(name, true);
			if (res) {
				lowsyslog("Internal error %i\n", res);
				assert(0);
			}
		}
	}
	else if (!strcmp(command, "save")) {
		print_status(config_save());
	}
	else if (!strcmp(command, "erase")) {
		print_status(config_erase());
	}
	else if (!strcmp(command, "get")) {
		if (argc < 2) {
			puts("Error: Not enough arguments");
			return;
		}
		const int ret = print_param(argv[1], false);
		if (ret)
			print_status(ret);
	}
	else if (!strcmp(command, "set")) {
		if (argc < 3) {
			puts("Error: Not enough arguments");
			return;
		}
		const char* const name = argv[1];
		const float value = atoff(argv[2]);
		const int res = config_set(name, value);
		if (res == 0)
			print_param(name, false);
		print_status(res);
	}
	else {
		puts("Usage:\n"
			"  cfg list\n"
			"  cfg save\n"
			"  cfg erase\n"
			"  cfg get <name>\n"
			"  cfg set <name> <value>");
	}
}

static void cmd_reset(BaseSequentialStream *chp, int argc, char *argv[])
{
	const char* line = getline("Really? y/(n) ");
	if (line && (line[0] == 'Y' || line[0] == 'y')) {
		puts("RESTART\n\n");
		chThdSleep(MS2ST(100)); // Flush the serial buffers
		// Doesn't work on ESC32 though
		NVIC_SystemReset();
	} else {
		puts("Abort");
	}
}

static void cmd_beep(BaseSequentialStream *chp, int argc, char *argv[])
{
	if (argc > 0 && !strcmp(argv[0], "help")) {
		puts("beep [freq_hz [duration_msec]]");
		return;
	}

	int freq = 500;
	if (argc > 0)
		freq = atoi(argv[0]);

	int duration = 300;
	if (argc > 1)
		duration = atoi(argv[1]);

	motor_beep(freq, duration);
}

static void cmd_pwmmanip(BaseSequentialStream *chp, int argc, char *argv[])
{
	if (argc > 0 && !strcmp(argv[0], "help")) {
		puts("pwmmanip [phase_a [phase_b [phase_c]]]");
		return;
	}

	motormgr_stop();

	enum motor_pwm_phase_manip manip_cmd[3];
	for (int i = 0; i < 3; i++) {
		int cmd = MOTOR_PWM_MANIP_FLOATING;
		if (i < argc) {
			cmd = atoi(argv[i]);
			if (cmd < 0 || cmd >= MOTOR_PWM_MANIP_END_) {
				puts("Error: Invalid command");
				return;
			}
		}
		manip_cmd[i] = cmd;
		lowsyslog("%i ", cmd);
	}
	puts("");
	motor_pwm_manip(manip_cmd);
}

static void cmd_stat(BaseSequentialStream *chp, int argc, char *argv[])
{
	float voltage = 0, current = 0;
	motormgr_get_input_voltage_current(&voltage, &current);

	lowsyslog("Power V/A     %-9f %f\n", voltage, current);
	lowsyslog("RPM/DC        %-9u %f\n", motormgr_get_rpm(), motormgr_get_duty_cycle());
	lowsyslog("Active limits %i\n", motormgr_get_limit_mask());
	lowsyslog("ZC failures   %U\n", (unsigned long)motor_get_zc_failures_since_start());
}

static void cmd_test(BaseSequentialStream *chp, int argc, char *argv[])
{
	puts("Hardware test...");
	int res = motor_test_hardware();
	if (res)
		lowsyslog("FAILED %i\n", res);
	else
		puts("OK");

	puts("Motor test...");
	res = motor_test_motor();
	puts(res ? "Not connected" : "Connected");
}

static void cmd_sp(BaseSequentialStream *chp, int argc, char *argv[])
{
	static const int TTL_MS = 30000;

	if (argc == 0) {
		motormgr_stop();
		puts("Usage:\n"
			"  sp <duty cycle or RPM>\n"
			"  sp arm\n"
			"Value with '.' is duty cycle, RPM otherwise");
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
	const bool mode_rpm = strchr(argv[0], '.') == NULL;  // If the string contains a dot, it's duty cycle

	if (mode_rpm) {
		lowsyslog("RPM %u\n", (unsigned)value);
		motormgr_set_rpm((unsigned)value, TTL_MS);
	} else {
		lowsyslog("DC %f\n", value);
		motormgr_set_duty_cycle(value, TTL_MS);
	}
}

static void cmd_md(BaseSequentialStream *chp, int argc, char *argv[])
{
	motor_print_debug_info();
}

#define COMMAND(cmd)    {#cmd, cmd_##cmd},
static const ShellCommand _commands[] =
{
	COMMAND(cfg)
	COMMAND(reset)
	COMMAND(beep)
	COMMAND(pwmmanip)
	COMMAND(stat)
	COMMAND(test)
	COMMAND(sp)
	COMMAND(md)
	{NULL, NULL}
};

// --------------------------

int puts(const char* str)
{
	const int len = strlen(str);
	if (len)
		sdWrite(&STDOUT_SD, (uint8_t*)str, len); // this fires an assert() if len = 0, haha!
	sdWrite(&STDOUT_SD, (uint8_t*)"\n", 1);
	return len + 1;
}

static char* getline(const char* prompt)
{
	static char _linebuf[32];
	memset(_linebuf, 0, sizeof(_linebuf));
	if (prompt)
		lowsyslog(prompt);
	if (!shellGetLine((BaseSequentialStream*) &STDIN_SD, _linebuf, sizeof(_linebuf)))
		return _linebuf;
	return NULL;
}

static void print_status(int err)
{
	if (err == 0)
		puts("OK");
	else
		lowsyslog("ERROR %d %s\n", err, strerror(abs(err)));
}

static const ShellConfig _config = {(BaseSequentialStream*)&STDOUT_SD, _commands};

static WORKING_AREA(_wa_shell, 1024);

void console_init(void)
{
	if (palReadPad(GPIO_PORT_SERIAL_RX, GPIO_PIN_SERIAL_RX) == 0) {
		lowsyslog("Console: RX pin is low, console will not be inited\n");
		return;
	}

	shellInit();

	assert_always(shellCreateStatic(&_config, _wa_shell, sizeof(_wa_shell), LOWPRIO));
}
