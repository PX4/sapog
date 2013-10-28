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

#pragma once

#define CH_FREQUENCY                    1000

#define CH_USE_HEAP                     TRUE
#define CH_USE_DYNAMIC                  FALSE
#define CH_USE_MAILBOXES                FALSE
#define CH_USE_MESSAGES                 FALSE
#define CH_USE_CONDVARS                 FALSE

void systemHaltHook(void);
#define SYSTEM_HALT_HOOK                systemHaltHook

void systemTickHook(void);
#define SYSTEM_TICK_EVENT_HOOK          systemTickHook

#if defined(DEBUG) && DEBUG
#   define CH_DBG_SYSTEM_STATE_CHECK       TRUE
#   define CH_DBG_ENABLE_CHECKS            TRUE
#   define CH_DBG_ENABLE_ASSERTS           TRUE
#   define CH_DBG_ENABLE_STACK_CHECK       TRUE
#   define CH_DBG_FILL_THREADS             TRUE
#   define CH_DBG_THREADS_PROFILING        TRUE
#elif defined(RELEASE) && RELEASE
#   define CH_DBG_THREADS_PROFILING        FALSE
#else
#   error "Invalid configuration: Either DEBUG or RELEASE must be true"
#endif

#include "../../chibios/os/kernel/templates/chconf.h"
