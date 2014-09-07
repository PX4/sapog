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

#include <sys.h>

#ifdef __cplusplus
extern "C" {
#endif

enum config_data_type
{
	CONFIG_TYPE_FLOAT,
	CONFIG_TYPE_INT,
	CONFIG_TYPE_BOOL
};

struct config_param
{
	const char* name;
	float default_;
	float min;
	float max;
	enum config_data_type type;
};


#if !defined(GLUE)
#  define GLUE_(a, b) a##b
#  define GLUE(a, b)  GLUE_(a, b)
#endif

#define CONFIG_PARAM_RAW_(name, default_, min, max, type)                         \
	static const struct config_param GLUE(_config_local_param_, __LINE__) =   \
		{name, default_, min, max, type};                                 \
	__attribute__((constructor))                                              \
	static void GLUE(_config_local_constructor_, __LINE__)(void) {            \
		config_register_param(&GLUE(_config_local_param_, __LINE__));     \
	}

/**
 * Parameter definition macros.
 * Defined parameter can be accessed through config_get("param-name"), config_get_descr(...).
 */
#define CONFIG_PARAM_FLOAT(name, default_, min, max)  CONFIG_PARAM_RAW_(name, default_, min, max, CONFIG_TYPE_FLOAT)
#define CONFIG_PARAM_INT(name, default_, min, max)    CONFIG_PARAM_RAW_(name, default_, min, max, CONFIG_TYPE_INT)
#define CONFIG_PARAM_BOOL(name, default_)             CONFIG_PARAM_RAW_(name, default_, 0,   1,   CONFIG_TYPE_BOOL)


/**
 * Internal logic; shall never be called explicitly.
 */
void config_register_param(const struct config_param* param);

/**
 * Returns 0 if everything is OK, even if the configuration could not be restored (this is not an error).
 * Returns negative errno in case of unrecoverable fault.
 */
int config_init(void);

/**
 * Saves the config into the non-volatile memory.
 * May enter a huge critical section, so it shall never be called concurrently with hard real time processes.
 */
int config_save(void);

/**
 * Erases the configuration from the non-volatile memory; current parameter values will not be affected.
 * Same warning as for @ref config_save()
 */
int config_erase(void);

/**
 * @param [in] index Non-negative parameter index
 * @return Name, or NULL if the index is out of range
 */
const char* config_name_by_index(int index);

/**
 * @param [in] name  Parameter name
 * @param [in] value Parameter value
 * @return 0 if the parameter does exist and the value is valid, negative errno otherwise.
 */
int config_set(const char* name, float value);

/**
 * @param [in]  name Parameter name
 * @param [out] out  Parameter descriptor
 * @return Returns 0 if the parameter does exist, negative errno otherwise.
 */
int config_get_descr(const char* name, struct config_param* out);

/**
 * @param [in] name Parameter name
 * @return The parameter value if it does exist; otherwise fires an assert() in debug builds, returns NAN in release.
 */
float config_get(const char* name);

#ifdef __cplusplus
}
#endif
