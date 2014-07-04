/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
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

#include "pwm_input.h"
#include <ch.h>
#include <hal.h>
#include <config/config.h>
#include <motor/motor.h>
#include <assert.h>

static const unsigned MIN_VALID_PULSE_WIDTH_USEC = 500;
static const unsigned MAX_VALID_PULSE_WIDTH_USEC = 3000;

static const float STOP_DUTY_CYCLE      = 0.03;
static const float START_MIN_DUTY_CYCLE = 0.06;
static const float START_MAX_DUTY_CYCLE = 0.20;

static const unsigned COMMAND_TTL_MS = 100;


static EVENTSOURCE_DECL(_update_event);

static volatile unsigned _last_pulse_width_usec;

CONFIG_PARAM_BOOL("pwm_enable", true)
CONFIG_PARAM_INT("pwm_min_pulse_width_usec",  1000,  800, 1200)
CONFIG_PARAM_INT("pwm_max_pulse_width_usec",  2000, 1800, 2200)


static void icu_pulse_width_callback(ICUDriver* icup)
{
	const unsigned new_width = icuGetWidth(icup);

	if ((new_width >= MIN_VALID_PULSE_WIDTH_USEC) && (new_width <= MAX_VALID_PULSE_WIDTH_USEC)) {
		_last_pulse_width_usec = new_width;

		chSysLockFromIsr();
		chEvtBroadcastFlagsI(&_update_event, ALL_EVENTS);
		chSysUnlockFromIsr();
	}
}

static msg_t thread(void* arg)
{
	(void)arg;

	EventListener listener;
	chEvtRegisterMask(&_update_event, &listener, ALL_EVENTS);

	const unsigned min_pulse_width_usec = config_get("pwm_min_pulse_width_usec");
	const unsigned max_pulse_width_usec = config_get("pwm_max_pulse_width_usec");

	for (;;) {
		if (chEvtWaitAnyTimeout(ALL_EVENTS, US2ST(65536)) == 0) {
			if (_last_pulse_width_usec > 0) {
				lowsyslog("PWMIN: Timeout\n");
				// We don't stop the motor here - it will be stopped automatically when TTL has expired
			}
			_last_pulse_width_usec = 0;
			continue;
		}

		/*
		 * Scale the input signal into [0, 1]
		 */
		unsigned local_copy = _last_pulse_width_usec;
		if (local_copy < min_pulse_width_usec) {
			local_copy = min_pulse_width_usec;
		}
		if (local_copy > max_pulse_width_usec) {
			local_copy = max_pulse_width_usec;
		}

		float dc = (local_copy - min_pulse_width_usec) / (float)(max_pulse_width_usec - min_pulse_width_usec);
		assert(dc >= 0);
		assert(dc <= 1);

		/*
		 * Handle start/stop corner cases
		 */
		if (motor_is_idle() && ((dc < START_MIN_DUTY_CYCLE) || (dc > START_MAX_DUTY_CYCLE))) {
			dc = 0;
		} else if (dc < STOP_DUTY_CYCLE) {
			dc = 0;
		} else {
			; // Nothing to do
		}
		//lowsyslog("%u\n", (unsigned)(dc * 100));

		/*
		 * Pass the new command into the motor controller
		 */
		if (dc > 0) {
			motor_set_duty_cycle(dc, COMMAND_TTL_MS);
		} else {
			motor_stop();
		}
	}

	assert_always(0);
	return 0;
}

void pwm_input_init(void)
{
	assert(STOP_DUTY_CYCLE < START_MIN_DUTY_CYCLE);
	assert(START_MIN_DUTY_CYCLE < START_MAX_DUTY_CYCLE);

	if (!config_get("pwm_enable")) {
		return;
	}

	chEvtInit(&_update_event);

	static WORKING_AREA(_wa_thread, 1024);
	assert_always(chThdCreateStatic(_wa_thread, sizeof(_wa_thread), NORMALPRIO, thread, NULL));

	static ICUConfig icucfg = {
		ICU_INPUT_ACTIVE_HIGH,
		1000000,
		icu_pulse_width_callback,
		NULL,
		NULL,
		ICU_CHANNEL_1,
		0
	};

	icuStart(&ICUD5, &icucfg);
	icuEnable(&ICUD5);
}
