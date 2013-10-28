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
#include "sys/sys.h"

static void led_set_status(bool state)
{
	if (state)
		palSetPad(GPIO_PORT_LED_STATUS, GPIO_PIN_LED_STATUS);
	else
		palClearPad(GPIO_PORT_LED_STATUS, GPIO_PIN_LED_STATUS);
}

static void led_set_error(bool state)
{
	if (state)
		palSetPad(GPIO_PORT_LED_ERROR, GPIO_PIN_LED_ERROR);
	else
		palClearPad(GPIO_PORT_LED_ERROR, GPIO_PIN_LED_ERROR);
}

int main(void)
{
	halInit();
	chSysInit();
	sdStart(&STDOUT_SD, NULL);

	while (1) {
		lowsyslog("Hello world!\n");
		chThdSleepMilliseconds(500);

		led_set_status(1);
		led_set_error(0);

		chThdSleepMilliseconds(500);

		led_set_status(0);
		led_set_error(1);
	}
	return 0;
}
