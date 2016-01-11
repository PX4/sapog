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
#include <hal.h>
#include <string.h>

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

const PALConfig pal_default_config = {
	{ VAL_GPIOAODR, VAL_GPIOACRL, VAL_GPIOACRH },
	{ VAL_GPIOBODR, VAL_GPIOBCRL, VAL_GPIOBCRH },
	{ VAL_GPIOCODR, VAL_GPIOCCRL, VAL_GPIOCCRH },
	{ VAL_GPIODODR, VAL_GPIODCRL, VAL_GPIODCRH },
	{ VAL_GPIOEODR, VAL_GPIOECRL, VAL_GPIOECRH }
};

void __early_init(void)
{
	stm32_clock_init();
}

void boardInit(void)
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

uint8_t board_get_hardware_revision(void)
{
	return (uint8_t)(GPIOC->IDR & 0x0F);
}

void board_read_unique_id(uint8_t out_uid[BOARD_UNIQUE_ID_SIZE])
{
	memcpy(out_uid, (const void*)0x1FFFF7E8, BOARD_UNIQUE_ID_SIZE);
}
