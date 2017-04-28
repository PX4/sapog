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

#include <board/board.hpp>
#include <cstring>
#include <unistd.h>
#include <zubax_chibios/util/software_i2c.hpp>
#include <zubax_chibios/platform/stm32/flash_writer.hpp>
#include <zubax_chibios/platform/stm32/config_storage.hpp>

// Clock config validation
#if STM32_PREDIV1_VALUE != 2
# error STM32_PREDIV1_VALUE
#endif
#if STM32_SYSCLK != 72000000
# error STM32_SYSCLK
#endif
#if STM32_PCLK2 != 72000000
# error STM32_PCLK2
#endif

// Defines GPIO configuration after boot up; see os_config/board.h
const PALConfig pal_default_config = {
	{ VAL_GPIOAODR, VAL_GPIOACRL, VAL_GPIOACRH },
	{ VAL_GPIOBODR, VAL_GPIOBCRL, VAL_GPIOBCRH },
	{ VAL_GPIOCODR, VAL_GPIOCCRL, VAL_GPIOCCRH },
	{ VAL_GPIODODR, VAL_GPIODCRL, VAL_GPIODCRH },
	{ VAL_GPIOEODR, VAL_GPIOECRL, VAL_GPIOECRH }
};

/// Provided by linker
const extern std::uint8_t DeviceSignatureStorage[];

namespace board
{

// This can't be constexpr because of reinterpret_cast<>
static void* const ConfigStorageAddress = reinterpret_cast<void*>(0x08000000 + (256 * 1024) - 1024);
constexpr unsigned ConfigStorageSize = 1024;

extern void init_led();

os::watchdog::Timer init(unsigned watchdog_timeout_ms)
{
	// OS
	halInit();
	chSysInit();

	// Watchdog - initializing as soon as possible
	os::watchdog::init();
	os::watchdog::Timer wdt;
	wdt.startMSec(watchdog_timeout_ms);

	// CLI
	sdStart(&STDOUT_SD, NULL);

	// LED
	init_led();

	// Config
	static os::stm32::ConfigStorageBackend config_storage_backend(ConfigStorageAddress, ConfigStorageSize);
	const int config_init_res = os::config::init(&config_storage_backend);
	if (config_init_res < 0)
	{
		die(config_init_res);
	}

	// Banner
	const auto hw_version = detect_hardware_version();
	os::lowsyslog("%s %u.%u %u.%u.%08x / %d %s\n",
		NODE_NAME,
		hw_version.major, hw_version.minor,
		FW_VERSION_MAJOR, FW_VERSION_MINOR, GIT_HASH, config_init_res,
		os::watchdog::wasLastResetTriggeredByWatchdog() ? "WDTRESET" : "OK");

	return wdt;
}

int i2c_exchange(std::uint8_t address,
                 const void* tx_data, const std::uint16_t tx_size,
                       void* rx_data, const std::uint16_t rx_size)
{
	static chibios_rt::Mutex mutex;
	os::MutexLocker mlock(mutex);
	return -int(os::software_i2c::Master(GPIO_PORT_I2C_SCL, GPIO_PIN_I2C_SCL,
	                                     GPIO_PORT_I2C_SDA, GPIO_PIN_I2C_SDA)\
		.exchange(address, tx_data, tx_size,
		                   rx_data, rx_size));
}

void die(int error)
{
	os::lowsyslog("FATAL ERROR %d\n", error);
	while (1) {
		led_emergency_override(LEDColor::RED);
		::sleep(1);
	}
}

void reboot()
{
	NVIC_SystemReset();
}

UniqueID read_unique_id()
{
	UniqueID out;
	std::memcpy(out.data(), reinterpret_cast<const void*>(0x1FFFF7E8), std::tuple_size<UniqueID>::value);
	return out;
}

HardwareVersion detect_hardware_version()
{
	auto v = HardwareVersion();

	v.major = HW_VERSION_MAJOR;
	v.minor = std::uint8_t(GPIOC->IDR & 0x0F);

	return v;
}

bool try_read_device_signature(DeviceSignature& out_sign)
{
	std::memcpy(out_sign.data(), &DeviceSignatureStorage[0], std::tuple_size<DeviceSignature>::value);

	for (auto x : out_sign) {
		if (x != 0xFF && x != 0x00) { // All 0xFF/0x00 is not a valid signature, it's empty storage
			return true;
		}
	}
	return false;
}

bool try_write_device_signature(const DeviceSignature& sign)
{
	{
		DeviceSignature dummy;
		if (try_read_device_signature(dummy)) {
			return false;               // Already written
		}
	}

	// Before flash can be written, the source must be aligned.
	alignas(4) std::uint8_t aligned_buffer[std::tuple_size<DeviceSignature>::value];
	std::copy(std::begin(sign), std::end(sign), std::begin(aligned_buffer));

	os::stm32::FlashWriter writer;

	return writer.write(&DeviceSignatureStorage[0], &aligned_buffer[0], sizeof(aligned_buffer));
}

}

extern "C"
{

/// Called from ChibiOS init
void __early_init()
{
	stm32_clock_init();
	// Making sure LSI is up and running
 	while ((RCC->CSR & RCC_CSR_LSIRDY) == 0);
}

/// Called from ChibiOS init
void boardInit()
{
	uint32_t mapr = AFIO->MAPR;
	mapr &= ~AFIO_MAPR_SWJ_CFG; // these bits are write-only

	// Enable SWJ only, JTAG is not needed at all:
	mapr |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;

	// TIM1 - motor control
	mapr |= AFIO_MAPR_TIM1_REMAP_0;

	// Serial CLI
	mapr |= AFIO_MAPR_USART1_REMAP;

	// TIM3 - RGB LED PWM
	mapr |= AFIO_MAPR_TIM3_REMAP_FULLREMAP;

	AFIO->MAPR = mapr;
}

}
