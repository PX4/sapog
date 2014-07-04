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

#include "led.h"
#include <ch.h>
#include <hal.h>
#include <assert.h>
#include <stm32f10x.h>

#undef TIM1
#undef TIM2
#undef TIM4
#undef TIM5
#undef TIM6
#undef TIM7

#define GLUE2_(A, B)     A##B
#define GLUE2(A, B)      GLUE2_(A, B)
#define GLUE3_(A, B, C)  A##B##C
#define GLUE3(A, B, C)   GLUE3_(A, B, C)

/**
 * Timer declaration
 */
#define TIMER_NUMBER   3

#define TIMX            GLUE2(TIM, TIMER_NUMBER)

#if TIMER_NUMBER < 2 || TIMER_NUMBER > 7
#  error "Invalid timer number"
#else
#  define TIMX_RCC_ENR          RCC->APB1ENR
#  define TIMX_RCC_RSTR         RCC->APB1RSTR
#  define TIMX_RCC_ENR_MASK     GLUE3(RCC_APB1ENR_TIM,  TIMER_NUMBER, EN)
#  define TIMX_RCC_RSTR_MASK    GLUE3(RCC_APB1RSTR_TIM, TIMER_NUMBER, RST)
#  define TIMX_INPUT_CLOCK      STM32_TIMCLK1
#endif

#define PWM_TOP   0xFFFF

void led_init(void)
{
	chSysDisable();

	// Power-on and reset
	TIMX_RCC_ENR |= TIMX_RCC_ENR_MASK;
	TIMX_RCC_RSTR |=  TIMX_RCC_RSTR_MASK;
	TIMX_RCC_RSTR &= ~TIMX_RCC_RSTR_MASK;

	chSysEnable();

	TIMX->ARR = PWM_TOP;
	TIMX->CR1 = 0;
	TIMX->CR2 = 0;

	// CC1, CC2, CC3 are R, G, B. Inverted mode.
	TIMX->CCMR1 =
		TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0 |
		TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_0;

	TIMX->CCMR2 =
		TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_0;

	// No inversion, all enabled
	TIMX->CCER = TIM_CCER_CC3E | TIM_CCER_CC2E | TIM_CCER_CC1E;

	// Start
	TIMX->EGR = TIM_EGR_UG | TIM_EGR_COMG;
	TIMX->CR1 |= TIM_CR1_CEN;
}

static float normalize_range(float x)
{
	if (x < 0.0F) { return 0.0F; }
	if (x > 1.0F) { return 1.0F; }
	return x;
}

void led_set_rgb(float red, float green, float blue)
{
	const unsigned pwm_red   = normalize_range(red)   * PWM_TOP;
	const unsigned pwm_green = normalize_range(green) * PWM_TOP;
	const unsigned pwm_blue  = normalize_range(blue)  * PWM_TOP;

	TIMX->CCR1 = pwm_red;
	TIMX->CCR2 = pwm_green;
	TIMX->CCR3 = pwm_blue;
}
