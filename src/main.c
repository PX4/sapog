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
#include "console.h"
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

void run_plot(void)
{
	motor_start(0.2, 0.6, false);
	while (1) {
		usleep(50000);
		const uint32_t per = motor_get_comm_period_hnsec();
		lowsyslog("$ %u\n", (unsigned)per);
	}
}

int init(void)
{
	int res = 0;

	res = config_init();
	if (res)
		return res;

	res = motormgr_init();
	if (res)
		return res;

	res = motor_test_hardware();
	if (res)
		return res;

	if (motor_test_motor())
		lowsyslog("Motor is not connected or damaged\n");
	else
		lowsyslog("Motor is OK\n");
	return 0;
}

__attribute__((noreturn))
void die(int status)
{
	lowsyslog("Now I am dead. %i\n", status);
	motor_beep(500, 1000);
	// Really there is nothing left to do; just sit there and beep sadly:
	while (1) {
		led_set_status(false);
		led_set_error(true);
		sleep(2);
		motor_beep(500, 80);
	}
}

int main(void)
{
	halInit();
	chSysInit();
	sdStart(&STDOUT_SD, NULL);

	usleep(300000);
	lowsyslog("\nPX4ESC: starting\n");

	const int init_status = init();

	console_init();

	if (init_status)
		die(init_status);

	motor_beep(1000, 150);
	led_set_status(false);
	led_set_error(false);

	while (1) {
		// TODO: LED indication
		usleep(10000);
		led_set_error(motormgr_get_limit_mask());
	}

	return 0;
}
