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
#include <cstdint>

namespace led
{
/**
 * Basic color codes
 */
enum class Color : unsigned
{
	OFF    = 0x000000,
	WHITE  = 0xFFFFFF,

	// Basic colors
	RED    = 0xFF0000,
	YELLOW = 0xFFFF00,
	GREEN  = 0x00FF00,
	CYAN   = 0x00FFFF,
	BLUE   = 0x0000FF,
	PURPLE = 0xFF00FF,

	// Shades
	PALE_WHITE  = 0x0F0F0F,
	DARK_RED    = 0x0F0000,
	DARK_GREEN  = 0x000F00,
	DARK_BLUE   = 0x00000F
};

/**
 * Must be called once before LED can be used.
 */
void init(void);

/**
 * This function can be called from any context.
 * It sets LED to a specified state overriding all existing layers.
 * Accepts @ref Color.
 */
void emergency_override(Color color);

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

	std::uint32_t color = 0;

	Overlay& operator=(const Overlay&) = delete;
	Overlay(const Overlay&) = delete;

public:
	Overlay() { }
	~Overlay() { unset(); }

	/**
	 * Accepts standard RGB hex, e.g. 0xFFFFFF for white.
	 * This function is thread-safe.
	 */
	void set_hex_rgb(std::uint32_t hex_rgb);

	/**
	 * Accepts @ref Color.
	 * This function is thread-safe.
	 */
	void set(Color new_color) { set_hex_rgb(unsigned(new_color)); }

	/**
	 * Accepts @ref Color.
	 * Blinks the specified color.
	 */
	void blink(Color new_color) { set_hex_rgb((color > 0) ? 0 : unsigned(new_color)); }

	/**
	 * Returns the current color code.
	 */
	std::uint32_t get_hex_rgb() const { return color; }

	/**
	 * Makes the layer inactive.
	 * Next lower-priority layer will become active instead.
	 * This function is thread-safe.
	 */
	void unset();
};

}
