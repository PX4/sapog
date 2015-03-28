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
#if STM32_SYSCLK != 120000000
# error STM32_SYSCLK
#endif
#if STM32_PCLK1 != 30000000
# error STM32_PCLK1
#endif
#if STM32_PCLK2 != 60000000
# error STM32_PCLK2
#endif
#if STM32_ADCCLK != 30000000
# error STM32_ADCCLK
#endif

const PALConfig pal_default_config =
{
    {
        VAL_GPIOA_MODER,
        VAL_GPIOA_OTYPER,
        VAL_GPIOA_OSPEEDR,
        VAL_GPIOA_PUPDR,
        VAL_GPIOA_ODR,
        VAL_GPIOA_AFRL,
        VAL_GPIOA_AFRH
    },
    {
        VAL_GPIOB_MODER,
        VAL_GPIOB_OTYPER,
        VAL_GPIOB_OSPEEDR,
        VAL_GPIOB_PUPDR,
        VAL_GPIOB_ODR,
        VAL_GPIOB_AFRL,
        VAL_GPIOB_AFRH
    },
    {
        VAL_GPIOC_MODER,
        VAL_GPIOC_OTYPER,
        VAL_GPIOC_OSPEEDR,
        VAL_GPIOC_PUPDR,
        VAL_GPIOC_ODR,
        VAL_GPIOC_AFRL,
        VAL_GPIOC_AFRH
    }
};

void __early_init(void)
{
    stm32_clock_init();
}

void boardInit(void)
{
}

uint8_t board_get_hardware_revision(void)
{
    uint8_t id = 0;

    id |= palReadPad(GPIO_PORT_HW_ID_0, GPIO_PIN_HW_ID_0) << 0;
    id |= palReadPad(GPIO_PORT_HW_ID_1, GPIO_PIN_HW_ID_1) << 1;
    id |= palReadPad(GPIO_PORT_HW_ID_2, GPIO_PIN_HW_ID_2) << 2;
    id |= palReadPad(GPIO_PORT_HW_ID_3, GPIO_PIN_HW_ID_3) << 3;

    return id;
}

void board_read_unique_id(uint8_t out_uid[BOARD_UNIQUE_ID_SIZE])
{
    memcpy(out_uid, (const void*)0x1FFF7A10, BOARD_UNIQUE_ID_SIZE);
}
