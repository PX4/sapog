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

#include "adc.h"
#include "internal.h"
#include "pwm.h"
#include "timer.h"
#include <ch.h>
#include <hal.h>
#include <sys.h>
#include <assert.h>
#include <unistd.h>
#include <stm32.h>
#include <config/config.h>


#define ADC_REF_VOLTAGE          3.3f
#define ADC_RESOLUTION           12

#define NUM_SAMPLES_PER_ADC      7

/**
 * One ADC sample at maximum speed takes 14 cycles; max ADC clock at 72 MHz input is 12 MHz, so one ADC sample is:
 *    (1 / 12M) * 14 = 1.17 usec
 */
#define SAMPLE_DURATION_NANOSEC  1170

/**
 * ADC will be triggered at this time before the PWM mid cycle.
 */
//const int MOTOR_ADC_SYNC_ADVANCE_NANOSEC = (SAMPLE_DURATION_NANOSEC * (NUM_SAMPLES_PER_ADC - 1)) / 2;
const int MOTOR_ADC_SYNC_ADVANCE_NANOSEC = 0;

const int MOTOR_ADC_SAMPLE_WINDOW_NANOSEC = SAMPLE_DURATION_NANOSEC * NUM_SAMPLES_PER_ADC;


CONFIG_PARAM_FLOAT("motor_current_shunt_mohm",         5.0,   0.1,   100.0)


static float _shunt_resistance = 0;

static uint32_t _adc1_2_dma_buffer[NUM_SAMPLES_PER_ADC];
static struct motor_adc_sample _sample;


__attribute__((optimize(3)))
CH_FAST_IRQ_HANDLER(ADC1_2_IRQHandler)
{
	TESTPAD_SET(GPIO_PORT_TEST_ADC, GPIO_PIN_TEST_ADC);

	_sample.timestamp = motor_timer_hnsec() -
		((SAMPLE_DURATION_NANOSEC * NUM_SAMPLES_PER_ADC) / 2) / NSEC_PER_HNSEC;

#define SMPLADC1(num)     (_adc1_2_dma_buffer[num] & 0xFFFFU)
#define SMPLADC2(num)     (_adc1_2_dma_buffer[num] >> 16)
	/*
	 * ADC channel sampling:
	 *   A B C A B C VOLT
	 *   C A B C A B CURR
	 */
	_sample.phase_values[0] = (SMPLADC1(0) + SMPLADC2(1) + SMPLADC1(3) + SMPLADC2(4)) / 4;
	_sample.phase_values[1] = (SMPLADC1(1) + SMPLADC2(2) + SMPLADC1(4) + SMPLADC2(5)) / 4;
	_sample.phase_values[2] = (SMPLADC2(0) + SMPLADC1(2) + SMPLADC2(3) + SMPLADC1(5)) / 4;

	_sample.input_voltage = SMPLADC1(6);
	_sample.input_current = SMPLADC2(6);

#undef SMPLADC1
#undef SMPLADC2

	motor_adc_sample_callback(&_sample);

	// TODO: check if the current/voltage/temperature channels need to be sampled

	ADC1->SR = 0;         // Reset the IRQ flags
	TESTPAD_CLEAR(GPIO_PORT_TEST_ADC, GPIO_PIN_TEST_ADC);
}

static void enable(void)
{
#if 0
	// DMA
	DMA1_Channel1->CCR = 0;  // Deinitialize
	DMA1_Channel1->CMAR = (uint32_t)_adc1_2_dma_buffer;
	DMA1_Channel1->CNDTR = sizeof(_adc1_2_dma_buffer) / 4;
	DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
	DMA1_Channel1->CCR =
		DMA_CCR1_PL_0 | DMA_CCR1_PL_1 |  // Max priority
		DMA_CCR1_MSIZE_1 |               // 32 bit
		DMA_CCR1_PSIZE_1 |               // 32 bit
		DMA_CCR1_MINC |
		DMA_CCR1_CIRC |
		DMA_CCR1_EN;
#endif

	// ADC enable, reset
	chSysDisable();
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN;
	RCC->APB2RSTR |= RCC_APB2RSTR_ADCRST;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_ADCRST;
	chSysEnable();

	usleep(5);

	// ADC on
	ADC1->CR2 = ADC_CR2_ADON;
	ADC2->CR2 = ADC_CR2_ADON;

	/*
	 * ADC channel sampling:
	 *   A B C A B C VOLT
	 *   C A B C A B CURR
	 */
	ADC1->SQR1 = ADC_SQR1_L_1 | ADC_SQR1_L_2;
	ADC1->SQR3 =
		ADC_SQR3_SQ1_0 |
		ADC_SQR3_SQ2_1 |
		ADC_SQR3_SQ3_0 | ADC_SQR3_SQ3_1 |
		ADC_SQR3_SQ4_0 |
		ADC_SQR3_SQ5_1 |
		ADC_SQR3_SQ6_0 | ADC_SQR3_SQ6_1;
	ADC1->SQR2 = ADC_SQR2_SQ7_2;

	ADC2->SQR1 = ADC1->SQR1;
	ADC2->SQR3 =
		ADC_SQR3_SQ1_0 | ADC_SQR3_SQ1_1 |
		ADC_SQR3_SQ2_0 |
		ADC_SQR3_SQ3_1 |
		ADC_SQR3_SQ4_0 | ADC_SQR3_SQ4_1 |
		ADC_SQR3_SQ5_0 |
		ADC_SQR3_SQ6_1;
	ADC2->SQR2 = ADC_SQR2_SQ7_2 | ADC_SQR2_SQ7_0;

	// SMPR registers are not configured because they have right values by default

#if 0
	// ADC initialization
	ADC1->CR1 = ADC_CR1_DUALMOD_1 | ADC_CR1_DUALMOD_2 | ADC_CR1_SCAN | ADC_CR1_EOCIE;
	ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_EXTTRIG | MOTOR_ADC1_2_TRIGGER | ADC_CR2_DMA;

	ADC2->CR1 = ADC_CR1_DUALMOD_1 | ADC_CR1_DUALMOD_2 | ADC_CR1_SCAN;
	ADC2->CR2 = ADC_CR2_ADON | ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL_0 | ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_2;

	// ADC IRQ (only ADC1 IRQ is used because ADC2 is configured in slave mode)
	chSysDisable();
	nvicEnableVector(ADC1_2_IRQn, MOTOR_IRQ_PRIORITY_MASK);
	chSysEnable();
#endif
}

int motor_adc_init(void)
{
	_shunt_resistance = config_get("motor_current_shunt_mohm") / 1000.0f;

	chSysDisable();

	// TODO: configure DMA
	// TODO: configure ADC prescaler

	chSysEnable();

	enable();

	return 0;
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
	irq_primask_disable();
	ret = _sample;
	irq_primask_enable();
	return ret;
}

float motor_adc_convert_input_voltage(int raw)
{
	static const float RTOP = 10.0F;
	static const float RBOT = 1.3F;
	static const float SCALE = (RTOP + RBOT) / RBOT;
	const float unscaled = raw * (ADC_REF_VOLTAGE / (float)(1 << ADC_RESOLUTION));
	return unscaled * SCALE;
}

float motor_adc_convert_input_current(int raw)
{
	// http://www.diodes.com/datasheets/ZXCT1051.pdf
	const float vout = raw * (ADC_REF_VOLTAGE / (float)(1 << ADC_RESOLUTION));
	const float vsense = vout / 10;
	const float iload = vsense / _shunt_resistance;
	return iload;
}
