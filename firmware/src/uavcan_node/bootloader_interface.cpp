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

#include <uavcan_stm32/bxcan.hpp>
#include "bootloader_interface.hpp"

namespace uavcan_node
{
/**
 * This is the Brickproof Bootloader's app descriptor.
 * Details: https://github.com/PX4/Firmware/tree/nuttx_next/src/drivers/bootloaders/src/uavcan
 */
static const volatile struct __attribute__((packed))
{
    std::uint8_t signature[8] = {'A','P','D','e','s','c','0','0'};
    std::uint64_t image_crc = 0;
    std::uint32_t image_size = 0;
    std::uint32_t vcs_commit = GIT_HASH;
    std::uint8_t major_version = FW_VERSION_MAJOR;
    std::uint8_t minor_version = FW_VERSION_MINOR;
    std::uint8_t reserved[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
} _app_descriptor __attribute__((section(".app_descriptor")));


static constexpr auto BootloaderSignature = 0xB0A0424CU;
static constexpr auto AppSignature        = 0xB0A04150U;


struct SharedData
{
    std::uint32_t can_bus_bit_rate_bps = 0;
    uavcan::NodeID uavcan_node_id;
} static shared_data;


class CrcComputer : public uavcan::DataTypeSignatureCRC
{
public:
    void add(std::uint32_t value)
    {
        uavcan::DataTypeSignatureCRC::add(reinterpret_cast<const std::uint8_t*>(&value), 4);
    }
};


void init_bootloader_interface()
{
    const auto signature = uavcan_stm32::bxcan::Can[0]->FilterRegister[3].FR1;
    uavcan_stm32::bxcan::Can[0]->FilterRegister[3].FR1 = 0;                     // Invalidate to prevent deja vu

    const auto bus_speed = uavcan_stm32::bxcan::Can[0]->FilterRegister[3].FR2;
    const auto node_id   = uavcan_stm32::bxcan::Can[0]->FilterRegister[4].FR1;

    union
    {
        std::uint64_t u64;
        std::uint32_t u32[2];
    } crc;
    crc.u32[0] = uavcan_stm32::bxcan::Can[0]->FilterRegister[2].FR2;
    crc.u32[1] = uavcan_stm32::bxcan::Can[0]->FilterRegister[2].FR1;

    CrcComputer computer;
    computer.add(signature);
    computer.add(bus_speed);
    computer.add(node_id);

    const auto signature_match = signature == BootloaderSignature;
    const auto crc_match = crc.u64 == computer.get();
    const auto valid_params = (bus_speed > 0) && (node_id <= uavcan::NodeID::Max);

    if (signature_match && crc_match && valid_params)
    {
        shared_data.can_bus_bit_rate_bps = bus_speed;
        shared_data.uavcan_node_id = static_cast<std::uint8_t>(node_id);
    }
}

uavcan::protocol::SoftwareVersion get_uavcan_software_version()
{
    uavcan::protocol::SoftwareVersion x;

    x.major = _app_descriptor.major_version;
    x.minor = _app_descriptor.minor_version;

    x.image_crc = _app_descriptor.image_crc;
    x.vcs_commit = _app_descriptor.vcs_commit;

    x.optional_field_flags = x.OPTIONAL_FIELD_FLAG_VCS_COMMIT | x.OPTIONAL_FIELD_FLAG_IMAGE_CRC;

    return x;
}

std::uint32_t get_inherited_can_bus_bit_rate()
{
    return shared_data.can_bus_bit_rate_bps;
}

uavcan::NodeID get_inherited_node_id()
{
    return shared_data.uavcan_node_id;
}

void pass_parameters_to_bootloader(std::uint32_t can_bus_bit_rate, uavcan::NodeID node_id)
{
    uavcan_stm32::bxcan::Can[0]->FilterRegister[3].FR1 = AppSignature;
    uavcan_stm32::bxcan::Can[0]->FilterRegister[3].FR2 = can_bus_bit_rate;
    uavcan_stm32::bxcan::Can[0]->FilterRegister[4].FR1 = node_id.get();

    CrcComputer computer;
    computer.add(uavcan_stm32::bxcan::Can[0]->FilterRegister[3].FR1);
    computer.add(uavcan_stm32::bxcan::Can[0]->FilterRegister[3].FR2);
    computer.add(uavcan_stm32::bxcan::Can[0]->FilterRegister[4].FR1);

    union
    {
        std::uint64_t u64;
        std::uint32_t u32[2];
    } crc;
    crc.u64 = computer.get();
    uavcan_stm32::bxcan::Can[0]->FilterRegister[2].FR2 = crc.u32[0];
    uavcan_stm32::bxcan::Can[0]->FilterRegister[2].FR1 = crc.u32[1];
}

}
