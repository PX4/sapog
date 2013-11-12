/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Pavel Kirienko (pavel.kirienko@gmail.com)
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
#include <assert.h>
#include <unistd.h>
#include <stm32f10x.h>
#include "common.h"
#include "adc.h"
#include "pwm.h"
#include "timer.h"


static uint32_t _adc1_2_dma_buffer[2];
static struct motor_adc_sample _sample;


void ADC1_2_IRQHandler(void)
{
	TESTPAD_SET(GPIO_PORT_TEST_ADC, GPIO_PIN_TEST_ADC);

	// Fix later maybe: Obtain the precise timestamp from PWM driver?
	_sample.timestamp = motor_timer_hnsec() - ((MOTOR_ADC_SAMPLE_DURATION_NANOSEC * 2) / NSEC_PER_HNSEC);

	_sample.raw_phase_values[0] = _adc1_2_dma_buffer[0] & 0xFFFF;
	_sample.raw_phase_values[1] = _adc1_2_dma_buffer[1] & 0xFFFF;
	_sample.raw_phase_values[2] = _adc1_2_dma_buffer[0] >> 16;

	motor_adc_sample_callback(&_sample);

	// TODO: check if the current/voltage/temperature channels need to be sampled

	ADC1->SR = 0;         // Reset the IRQ flags
	TESTPAD_CLEAR(GPIO_PORT_TEST_ADC, GPIO_PIN_TEST_ADC);
}

static void adc_calibrate(ADC_TypeDef* const adc)
{
	// RSTCAL
	assert(!(adc->CR2 & ADC_CR2_RSTCAL));
	adc->CR2 |= ADC_CR2_RSTCAL;
	while (adc->CR2 & ADC_CR2_RSTCAL) { }

	// CAL
	assert(!(adc->CR2 & ADC_CR2_CAL));
	adc->CR2 |= ADC_CR2_CAL;
	while (adc->CR2 & ADC_CR2_CAL) { }
}

static void enable(void)
{
	// DMA
	DMA1_Channel1->CCR = 0;  // Deinitialize
	DMA1_Channel1->CMAR = (uint32_t)_adc1_2_dma_buffer;
	DMA1_Channel1->CNDTR = 2;
	DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
	DMA1_Channel1->CCR =
		DMA_CCR1_PL_0 | DMA_CCR1_PL_1 |  // Max priority
		DMA_CCR1_MSIZE_1 |               // 32 bit
		DMA_CCR1_PSIZE_1 |               // 32 bit
		DMA_CCR1_MINC |
		DMA_CCR1_CIRC |
		DMA_CCR1_EN;

	// ADC enable, reset
	const uint32_t enr_mask = RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN;
	const uint32_t rst_mask = RCC_APB2RSTR_ADC1RST | RCC_APB2RSTR_ADC2RST;
	chSysDisable();
	RCC->APB2ENR |= enr_mask;
	RCC->APB2RSTR |= rst_mask;
	RCC->APB2RSTR &= ~rst_mask;
	chSysEnable();

	usleep(5);  // Sequence: enable ADC, wait 2+ cycles, poweron, calibrate?

	// ADC calibration
	ADC1->CR2 = ADC_CR2_ADON;
	adc_calibrate(ADC1);

	ADC2->CR2 = ADC_CR2_ADON;
	adc_calibrate(ADC2);

	// ADC channels
	ADC1->SQR1 = ADC_SQR1_L_0;                 // 2 channels
	ADC1->SQR3 =
		ADC_SQR3_SQ1_0 |                   // channel 1 (phase A)
		ADC_SQR3_SQ2_1;                    // channel 2 (phase B)

	ADC2->SQR1 = ADC_SQR1_L_0;                 // 2 channels
	ADC2->SQR3 =
		ADC_SQR3_SQ1_0 | ADC_SQR3_SQ1_1 |  // channel 3 (phase C)
		ADC_SQR3_SQ2_0 | ADC_SQR3_SQ2_1;   // channel 3 (phase C)

	// SMPR registers are not configured because they have right values by default

	// ADC initialization
	ADC1->CR1 = ADC_CR1_DUALMOD_1 | ADC_CR1_DUALMOD_2 | ADC_CR1_SCAN | ADC_CR1_EOCIE;
	ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_EXTTRIG | MOTOR_ADC1_2_TRIGGER | ADC_CR2_DMA;

	ADC2->CR1 = ADC_CR1_DUALMOD_1 | ADC_CR1_DUALMOD_2 | ADC_CR1_SCAN;
	ADC2->CR2 = ADC_CR2_ADON | ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL_0 | ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_2;

	// ADC IRQ (only ADC1 IRQ is used because ADC2 is configured in slave mode)
	chSysDisable();
	nvicEnableVector(ADC1_2_IRQn, MOTOR_IRQ_PRIORITY_MASK);
	chSysEnable();
}

void motor_adc_init(void)
{
	chSysDisable();

	RCC->AHBENR |= RCC_AHBENR_DMA1EN;  // Never disabled

	RCC->CFGR &= ~RCC_CFGR_ADCPRE;
#if STM32_PCLK2 == 72000000
	// ADC clock 72 / 6 = 12 MHz
	RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;
#else
#  error "What's wrong with PCLK2?"
#endif
	chSysEnable();

	enable();
}

void motor_adc_enable_from_isr(void)
{
	ADC1->SR = 0;
	ADC1->CR1 |= ADC_CR1_EOCIE;
}

void motor_adc_disable_from_isr(void)
{
	ADC1->CR1 &= ~ADC_CR1_EOCIE;
}

struct motor_adc_sample motor_adc_get_last_sample(void)
{
	struct motor_adc_sample ret;
	chSysDisable();
	ret = _sample;
	chSysEnable();
	return ret;
}
