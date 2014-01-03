/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Pavel Kirienko (pavel.kirienko@gmail.com)
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

#pragma once

#include <stdint.h>
#include <sys.h>

__BEGIN_DECLS

#define HNSEC_PER_USEC      10
#define HNSEC_PER_MSEC      10000
#define HNSEC_PER_SEC       10000000
#define HNSEC_PER_MINUTE    (HNSEC_PER_SEC * 60)
#define NSEC_PER_HNSEC      100

void motor_timer_init(void);

/**
 * Minimal maintainable RPM depends on this parameter.
 */
uint64_t motor_timer_get_max_delay_hnsec(void);

/**
 * Returns the current timestamp in hectonanoseconds (10^-7).
 */
uint64_t motor_timer_hnsec(void);

void motor_timer_set_relative(int delay_hnsec);
void motor_timer_set_absolute(uint64_t timestamp_hnsec);
void motor_timer_cancel(void);

/**
 * No OS API can be used from this callback!
 */
extern void motor_timer_callback(uint64_t timestamp_hnsec);

/**
 * Busy loop delay
 */
void motor_timer_udelay(int usecs);
void motor_timer_hndelay(int hnsecs);

__END_DECLS
