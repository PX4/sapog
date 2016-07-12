/****************************************************************************
 *
 *   Copyright (C) 2016 PX4 Development Team. All rights reserved.
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

#include <temperature_sensor.hpp>
#include <zubax_chibios/os.hpp>
#include <board/board.hpp>
#include <motor/motor.h>
#include <unistd.h>
#include <limits>
#include <algorithm>

namespace temperature_sensor
{
namespace
{

static constexpr unsigned SENSOR_ADDRESS = 0b1001000;
static constexpr std::int16_t KELVIN_OFFSET = 273;

std::int16_t temperature = -1;
bool functional = false;

std::int16_t convert_lm75b_to_kelvin(const std::array<std::uint8_t, 2>& raw)
{
	auto x = std::int16_t((std::uint16_t(raw[0]) << 8) | raw[1]) >> 5;
	if (x & (1U << 10)) {
		x |= 0xFC00;
	}
	return x / 8 + KELVIN_OFFSET;
}

std::int16_t try_read()
{
	const std::array<std::uint8_t, 1> tx = { 0 };
	std::array<std::uint8_t, 2> rx;
	if (board::i2c_exchange(SENSOR_ADDRESS, tx, rx) < 0) {
		return -1;
	}
	return convert_lm75b_to_kelvin(rx);
}

class : public chibios_rt::BaseStaticThread<128>
{
	void main() override
	{
		os::watchdog::Timer wdt;
		wdt.startMSec(2000);
		setName("tempsens");

		while (!os::isRebootRequested()) {
			wdt.reset();
			::usleep(500 * 1000);

			if (!motor_is_running() && !motor_is_idle()) {
				// When the motor is starting, I2C goes bananas
				continue;
			}

			const std::int16_t new_temp = try_read();
			if (new_temp >= 0) {
				temperature = std::int16_t((std::int32_t(temperature) * 7 + new_temp + 7) / 8);
				functional = true;
			} else {
				functional = false;
			}
		}
	}
} thread_;

}

int init()
{
	temperature = try_read();
	if (temperature < 0) {
		return -1;
	}
	thread_.start(LOWPRIO);
	return 0;
}

bool is_ok()
{
	return functional;
}

std::int16_t get_temperature_K()
{
	return temperature;
}

}
