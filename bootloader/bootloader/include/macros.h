/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(arraySize)
#define arraySize(a) (sizeof((a))/sizeof((a[0])))
#endif

#if !defined(UNUSED)
#define UNUSED(var) (void)(var)
#endif

#if !defined(CAT)
#if !defined(_CAT)
#define _CAT(a, b) a ## b
#endif
#define CAT(a, b) _CAT(a, b)
#endif

#if !defined(CCASSERT)
#if defined(static_assert)
#       define CCASSERT(predicate) static_assert(predicate)
#   else
#       define CCASSERT(predicate) _x_CCASSERT_LINE(predicate, __LINE__)
#       if !defined(_x_CCASSERT_LINE)
#           define _x_CCASSERT_LINE(predicate, line) typedef char CAT(constraint_violated_on_line_,line)[2*((predicate)!=0)-1] __attribute__ ((unused)) ;
#       endif
#   endif
#endif
