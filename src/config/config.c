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
#include <stdlib.h>
#include <assert.h>
#include <errno.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "config.h"

#define MAX_PARAMS     32


int flash_storage_read(unsigned offset, void* data, unsigned len);
int flash_storage_write(unsigned offset, const void* data, unsigned len);
int flash_storage_erase(void);


#define OFFSET_LAYOUT_HASH      0
#define OFFSET_CRC              4
#define OFFSET_VALUES           8

static const struct config_param* _descr_pool[MAX_PARAMS];
static float _value_pool[MAX_PARAMS];

static int _num_params = 0;
static uint32_t _layout_hash = 0;
static bool _frozen = false;

static Mutex _mutex;


static uint32_t crc32_step(uint32_t crc, uint8_t new_byte)
{
	crc = crc ^ (uint32_t)new_byte;
	for (int j = 7; j >= 0; j--)
		crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
	return crc;
}

static uint32_t crc32(const void* data, int len)
{
	assert(data && len >= 0);
	if (!data)
		return 0;

	uint32_t crc = 0;
	for (int i = 0; i < len; i++)
		crc = crc32_step(crc, *(((const uint8_t*)data) + i));
	return crc;
}

static bool is_valid(const struct config_param* descr, float value)
{
	assert(descr);

	if (!isfinite(value))
		return false;

	switch (descr->type) {
	case CONFIG_TYPE_BOOL:
		if (value != 0.0f && value != 1.0f)
			return false;
		break;

	case CONFIG_TYPE_INT: {
		volatile const long truncated = (long)value;
		if (value != (float)truncated)
			return false;
	}
		/* fallthrough */
	case CONFIG_TYPE_FLOAT:
		if (value > descr->max || value < descr->min)
			return false;
		break;

	default:
		assert(0);
		return false;
	}
	return true;
}

static int index_by_name(const char* name)
{
	assert(name);
	if (!name)
		return -1;
	for (int i = 0; i < _num_params; i++) {
		if (!strcmp(_descr_pool[i]->name, name))
			return i;
	}
	return -1;
}

void config_register_param(const struct config_param* param)
{
	// This function can not be executed after the startup initialization is finished
	assert(!_frozen);
	if (_frozen)
		return;

	assert_always(param && param->name);
	assert_always(_num_params < MAX_PARAMS);         // If fails here, increase MAX_PARAMS
	assert_always(is_valid(param, param->default_)); // If fails here, param descriptor is invalid
	assert_always(index_by_name(param->name) < 0);   // If fails here, param name is not unique

	// Register this param
	const int index = _num_params++;
	assert_always(_descr_pool[index] == NULL);
	assert_always(_value_pool[index] == 0);
	_descr_pool[index] = param;
	_value_pool[index] = param->default_;

	// Update the layout identification hash
	for (const char* ch = param->name; *ch; ch++)
		_layout_hash = crc32_step(_layout_hash, *ch);
}

static void reinitialize_defaults(const char* reason)
{
	lowsyslog("Config: Initializing defaults - %s\n", reason);
	for (int i = 0; i < _num_params; i++)
		_value_pool[i] = _descr_pool[i]->default_;
}

int config_init(void)
{
	assert_always(!_frozen);
	_frozen = true;

	chMtxInit(&_mutex);

	assert_always(_num_params < MAX_PARAMS);  // being paranoid

	// Read the layout hash
	uint32_t stored_layout_hash = 0xdeadbeef;
	int flash_res = flash_storage_read(OFFSET_LAYOUT_HASH, &stored_layout_hash, 4);
	if (flash_res)
		goto flash_error;

	// If the layout hash has not changed, we can restore the values safely
	if (stored_layout_hash == _layout_hash) {
		const int pool_len = _num_params * sizeof(_value_pool[0]);

		// Read the data
		flash_res = flash_storage_read(OFFSET_VALUES, _value_pool, pool_len);
		if (flash_res)
			goto flash_error;

		// Check CRC
		const uint32_t true_crc = crc32(_value_pool, pool_len);
		uint32_t stored_crc = 0;
		flash_res = flash_storage_read(OFFSET_CRC, &stored_crc, 4);
		if (flash_res)
			goto flash_error;

		// Reinitialize defaults if restored values are not valid or if CRC does not match
		if (true_crc == stored_crc) {
			lowsyslog("Config: %i params restored\n", _num_params);
			for (int i = 0; i < _num_params; i++) {
				if (!is_valid(_descr_pool[i], _value_pool[i])) {
					lowsyslog("Config: Resetting param [%s]: %f --> %f\n",
						_descr_pool[i]->name, _value_pool[i], _descr_pool[i]->default_);

					_value_pool[i] = _descr_pool[i]->default_;
				}
			}
		} else {
			reinitialize_defaults("CRC mismatch");
		}
	} else {
		reinitialize_defaults("Layout mismatch");
	}

	return 0;

	flash_error:
	assert(flash_res);
	reinitialize_defaults("Flash error");
	return flash_res;
}

int config_save(void)
{
	chMtxLock(&_mutex);

	// Erase
	int flash_res = flash_storage_erase();
	if (flash_res)
		goto flash_error;

	// Write Layout
	flash_res = flash_storage_write(OFFSET_LAYOUT_HASH, &_layout_hash, 4);
	if (flash_res)
		goto flash_error;

	// Write CRC
	const int pool_len = _num_params * sizeof(_value_pool[0]);
	const uint32_t true_crc = crc32(_value_pool, pool_len);
	flash_res = flash_storage_write(OFFSET_CRC, &true_crc, 4);
	if (flash_res)
		goto flash_error;

	// Write Values
	flash_res = flash_storage_write(OFFSET_VALUES, _value_pool, pool_len);
	if (flash_res)
		goto flash_error;

	chMtxUnlock();
	return 0;

	flash_error:
	assert(flash_res);
	chMtxUnlock();
	return flash_res;
}

int config_erase(void)
{
	chMtxLock(&_mutex);
	int res = flash_storage_erase();
	chMtxUnlock();
	return res;
}

const char* config_name_by_index(int index)
{
	assert(index >= 0);
	if (index < 0 || index >= _num_params)
		return NULL;
	return _descr_pool[index]->name;
}

int config_set(const char* name, float value)
{
	int retval = 0;
	chMtxLock(&_mutex);

	const int index = index_by_name(name);
	if (index < 0) {
		retval = -ENOENT;
		goto leave;
	}

	if (!is_valid(_descr_pool[index], value)) {
		retval = -EINVAL;
		goto leave;
	}

	_value_pool[index] = value;

	leave:
	chMtxUnlock();
	return retval;
}

int config_get_descr(const char* name, struct config_param* out)
{
	assert(out);
	if (!out)
		return -EINVAL;

	int retval = 0;
	chMtxLock(&_mutex);

	const int index = index_by_name(name);
	if (index < 0) {
		retval = -ENOENT;
		goto leave;
	}

	*out = *_descr_pool[index];

	leave:
	chMtxUnlock();
	return retval;
}

float config_get(const char* name)
{
	chMtxLock(&_mutex);
	const int index = index_by_name(name);
	const float val = (index < 0) ? nanf("") : _value_pool[index];
	chMtxUnlock();
	return val;
}
