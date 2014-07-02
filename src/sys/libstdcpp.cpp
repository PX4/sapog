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

#include <ch.hpp>
#include <cstdlib>
#include <sys/types.h>
#include "sys.h"

void* operator new(size_t sz)
{
    return chCoreAlloc(sz);
}

void* operator new[](size_t sz)
{
    return chCoreAlloc(sz);
}

void operator delete(void*)
{
    sys_panic("delete");
}

void operator delete[](void*)
{
    sys_panic("delete");
}

/*
 * stdlibc++ workaround.
 * Default implementations will throw, which causes code size explosion.
 * These definitions override the ones defined in the stdlibc+++.
 */
namespace std
{

void __throw_bad_exception() { sys_panic("throw"); }

void __throw_bad_alloc() { sys_panic("throw"); }

void __throw_bad_cast() { sys_panic("throw"); }

void __throw_bad_typeid() { sys_panic("throw"); }

void __throw_logic_error(const char*) { sys_panic("throw"); }

void __throw_domain_error(const char*) { sys_panic("throw"); }

void __throw_invalid_argument(const char*) { sys_panic("throw"); }

void __throw_length_error(const char*) { sys_panic("throw"); }

void __throw_out_of_range(const char*) { sys_panic("throw"); }

void __throw_runtime_error(const char*) { sys_panic("throw"); }

void __throw_range_error(const char*) { sys_panic("throw"); }

void __throw_overflow_error(const char*) { sys_panic("throw"); }

void __throw_underflow_error(const char*) { sys_panic("throw"); }

void __throw_ios_failure(const char*) { sys_panic("throw"); }

void __throw_system_error(int) { sys_panic("throw"); }

void __throw_future_error(int) { sys_panic("throw"); }

void __throw_bad_function_call() { sys_panic("throw"); }

}

namespace __gnu_cxx
{

void __verbose_terminate_handler()
{
    sys_panic("terminate");
}

}

extern "C"
{

int __aeabi_atexit(void*, void(*)(void*), void*)
{
    return 0;
}

__extension__ typedef int __guard __attribute__((mode (__DI__)));

void __cxa_atexit(void(*)(void *), void*, void*)
{
}

int __cxa_guard_acquire(__guard* g)
{
    return !*g;
}

void __cxa_guard_release (__guard* g)
{
    *g = 1;
}

void __cxa_guard_abort (__guard*)
{
}

void __cxa_pure_virtual()
{
    sys_panic("pure virtual");
}

}

