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

#include "sys.h"
#include <stm32f10x.h>
#include <stdio.h>
#include <unistd.h>
#include <assert.h>
#include <stdarg.h>
#include <chprintf.h>

__attribute__((weak))
void *__dso_handle;

#if !CH_DBG_ENABLED
const char *dbg_panic_msg;
#endif


void system_tick_hook(void)
{
}

static void writepoll(const char* str)
{
	for (const char *p = str; *p; p++) {
		while (!(USART1->SR & USART_SR_TXE)) { }
		USART1->DR = *p;
	}
}

void system_halt_hook(void)
{
	application_halt_hook();

	port_disable();
	writepoll("\nPANIC [");
	const Thread *pthread = chThdSelf();
	if (pthread && pthread->p_name) {
		writepoll(pthread->p_name);
	}
	writepoll("] ");

	if (dbg_panic_msg != NULL) {
		writepoll(dbg_panic_msg);
	}
	writepoll("\n");

#if DEBUG_BUILD
    if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk)
    {
        __asm volatile ("bkpt #0\n"); // Break into the debugger
    }
#endif
}

void lowsyslog(const char* format, ...)
{
	va_list vl;
	va_start(vl, format);
	chvprintf((BaseSequentialStream*)&(STDOUT_SD), format, vl);
	va_end(vl);
}

__attribute__((weak))
void application_halt_hook(void)
{
}

void sys_panic(const char* msg)
{
	dbg_panic_msg = msg;
	chSysHalt();
	while (1) { }
}

void __assert_func(const char* file, int line, const char* func, const char* expr)
{
	port_disable();

	char buf[128]; // We don't care about possible stack overflow because going to die anyway
	snprintf(buf, sizeof(buf), "%s:%i at %s(..): %s", file, line, func, expr);
	dbg_panic_msg = buf;

	chSysHalt();
	while (1) { }
}

void _exit(int status)
{
	(void) status;
	chSysHalt();
	while (1) { }
}

pid_t _getpid(void)
{
	return 1;
}

void _kill(pid_t id)
{
	(void) id;
}

/// From unistd
int usleep(useconds_t useconds)
{
	chThdSleepMicroseconds(useconds);
	return 0;
}

/// From unistd
unsigned sleep(unsigned int seconds)
{
	chThdSleepSeconds(seconds);
	return 0;
}
