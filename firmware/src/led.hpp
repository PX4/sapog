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

#pragma once

#include <ch.hpp>
#include <sys.h>
#include <stdint.h>

namespace led
{
/**
 * Must be called once before LED can be used.
 */
void init(void);

/**
 * This function can be called from any context.
 * It sets LED to a specified state overriding all existing layers.
 */
void emergency_override_rgb(float red, float green, float blue);

/**
 * This class allows to control the same LED from many sources in a stacked manner.
 * The instance of this class initialized last would be able to control the LED state, while
 * other instances (initialized earlier) would be shadowed until the top one is removed.
 */
class Overlay
{
	static constexpr int MAX_LAYERS = 4;

	static Overlay* layers[MAX_LAYERS];
	static chibios_rt::Mutex mutex;

	float rgb[3] = {};

	static Overlay*& locateLayer(const Overlay* ovr);

	Overlay& operator=(const Overlay&) = delete;
	Overlay(const Overlay&) = delete;

public:
	Overlay() { }
	~Overlay() { unset(); }

	/**
	 * Colors are in the range [0, 1].
	 * This function is thread-safe.
	 */
	void set_rgb(float red, float green, float blue);

	/**
	 * Turns the LED off.
	 * This method has the same effect as setting RGB to zeroes.
	 */
	void set_off() { set_rgb(0.0F, 0.0F, 0.0F); }

	/**
	 * Checks whether the current state is not off.
	 */
	bool is_on() const { return (rgb[0] > 1e-6F) || (rgb[1] > 1e-6F) || (rgb[2] > 1e-6F); }

	/**
	 * Makes the layer inactive.
	 * Next lower-priority layer will become active instead.
	 * This function is thread-safe.
	 */
	void unset();
};

}
