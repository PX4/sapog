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

#include <stdint.h>
#include <assert.h>
#include <hal.h>
#include <sys.h>
#include <watchdog.h>

#define KR_KEY_ACCESS   0x5555
#define KR_KEY_RELOAD   0xAAAA
#define KR_KEY_ENABLE   0xCCCC

#define MAX_RELOAD_VAL  0xFFF

#define MAX_NUM_WATCHDOGS 31


static int _wdg_timeout_ms = 0;

static uint32_t _mask __attribute__((section (".noinit")));
static int _num_watchdogs __attribute__((section (".noinit")));


static void set_timeout(int timeout_ms)
{
	return;

	if (timeout_ms <= 0) {
		assert(0);
		timeout_ms = 1;
	}

	int reload_value = timeout_ms / 6;  // For 1/256 prescaler
	if (reload_value > MAX_RELOAD_VAL)
		reload_value = MAX_RELOAD_VAL;

	IWDG->KR = KR_KEY_RELOAD;

	// Wait until the IWDG is ready to accept the new parameters
	while (IWDG->SR != 0) { }

	IWDG->KR = KR_KEY_ACCESS;
	IWDG->PR = 6;             // Div 256 yields 6.4ms per clock period at 40kHz
	IWDG->RLR = reload_value;
	IWDG->KR = KR_KEY_RELOAD;
	IWDG->KR = KR_KEY_ENABLE; // Starts if wasn't running yet
}

void watchdog_init(void)
{
	return;

	assert_always(_wdg_timeout_ms == 0);      // Make sure it wasn't initialized earlier
	assert_always(RCC->CSR & RCC_CSR_LSION);  // Make dure LSI is enabled
	while (!(RCC->CSR & RCC_CSR_LSIRDY)) { }  // Wait for LSI startup

	if (RCC->CSR & RCC_CSR_IWDGRSTF) {
		lowsyslog("Watchdog: RESET WAS CAUSED BY WATCHDOG TIMEOUT\n");
		lowsyslog("Watchdog: RCC_CSR=0x%08x\n", (unsigned)RCC->CSR);
		lowsyslog("Watchdog: LAST STATE: mask=0x%08x, num=%d\n", (unsigned)_mask, _num_watchdogs);
		chSysSuspend();
		RCC->CSR |= RCC_CSR_RMVF;
		chSysEnable();
	}

	_mask = 0;
	_num_watchdogs = 0;

	chSysSuspend();
	DBGMCU->CR |= DBGMCU_CR_DBG_IWDG_STOP;
	chSysEnable();
}

int watchdog_create(int timeout_ms)
{
	return 0;

	if (timeout_ms <= 0) {
		assert(0);
		return -1;
	}

	chSysSuspend();
	if (_num_watchdogs >= MAX_NUM_WATCHDOGS) {
		chSysEnable();
		assert(0);
		return -1;
	}
	const int new_id = _num_watchdogs++;
	_mask |= 1 << new_id;                 // Reset immediately
	chSysEnable();

	if (timeout_ms > _wdg_timeout_ms) {
		set_timeout(timeout_ms);
		_wdg_timeout_ms = timeout_ms;
		lowsyslog("Watchdog: Global timeout set to %i ms\n", _wdg_timeout_ms);
	}
	return new_id;
}

void watchdog_reset(int id)
{
	return;

	assert(id >= 0 && id < _num_watchdogs);

	chSysSuspend();
	_mask |= 1 << id;
	const uint32_t valid_bits_mask = (1 << _num_watchdogs) - 1;
	if ((_mask & valid_bits_mask) == valid_bits_mask) {
		IWDG->KR = KR_KEY_RELOAD;
		_mask = 0;
	}
	chSysEnable();
}
