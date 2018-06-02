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

#pragma once

#include <zubax_chibios/os.hpp>
#include <cstdint>
#include <array>
#include <board/led.hpp>

namespace board
{

os::watchdog::Timer init(unsigned watchdog_timeout_ms);

/**
 * Performs an I2C transaction.
 * This function is thread safe.
 */
int i2c_exchange(std::uint8_t address,
                 const void* tx_data, const std::uint16_t tx_size,
                       void* rx_data, const std::uint16_t rx_size);

/**
 * Safer wrapper over @ref i2c_exchange().
 */
template<unsigned TxSize, unsigned RxSize>
inline int i2c_exchange(std::uint8_t address,
                        const std::array<uint8_t, TxSize>& tx,
                              std::array<uint8_t, RxSize>& rx)
{
	return i2c_exchange(address, tx.data(), TxSize, rx.data(), RxSize);
}

__attribute__((noreturn))
void die(int error);

void reboot();

typedef std::array<std::uint8_t, 12> UniqueID;
UniqueID read_unique_id();

struct HardwareVersion
{
    std::uint8_t major;
    std::uint8_t minor;
};

HardwareVersion detect_hardware_version();

float get_current_shunt_resistance();

typedef std::array<std::uint8_t, 128> DeviceSignature;
bool try_read_device_signature(DeviceSignature& out_sign);
bool try_write_device_signature(const DeviceSignature& sign);

}
