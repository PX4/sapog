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

#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include "irq.h"

/**
 * C++ wrappers
 */
#ifdef __cplusplus
#  define __BEGIN_DECLS		extern "C" {
#  define __END_DECLS		}
#else
#  define __BEGIN_DECLS
#  define __END_DECLS
#endif

/**
 * NuttX-like low-level logging
 */
#ifndef STDOUT_SD
#  error "STDOUT_SD must be defined"
#endif
#define lowsyslog(...)     chprintf((BaseSequentialStream*)&(STDOUT_SD), __VA_ARGS__)

/**
 * Unconditional assert
 */
#define STRINGIZE2(x)   #x
#define STRINGIZE(x)    STRINGIZE2(x)
#define MAKE_ASSERT_MSG() __FILE__ ":" STRINGIZE(__LINE__)
#define assert_always(x)                                    \
    do {                                                    \
        if ((x) == 0) {                                     \
            dbg_panic_msg = MAKE_ASSERT_MSG();              \
            chSysHalt();                                    \
        }                                                   \
    } while (0)

/**
 * Application emergency termination hook
 */
extern void application_halt_hook(void);
