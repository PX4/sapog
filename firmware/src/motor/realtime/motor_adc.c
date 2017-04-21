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
#include "irq.h"
#include <ch.h>
#include <hal.h>
#include <assert.h>
#include <unistd.h>
#include <zubax_chibios/config/config.h>


#define ADC_REF_VOLTAGE          3.3f
#define ADC_RESOLUTION           12

#define NUM_SAMPLES_PER_ADC      4

/**
 * One ADC sample at maximum speed takes 14 cycles; max ADC clock at 72 MHz input is 12 MHz, so one ADC sample is:
 *    (1 / 12M) * 14 = 1.17 usec
 */
#define SAMPLE_DURATION_NANOSEC  1170

/**
 * ADC will be triggered at this time before the PWM mid cycle.
 */
const int MOTOR_ADC_SYNC_ADVANCE_NANOSEC = (SAMPLE_DURATION_NANOSEC * (NUM_SAMPLES_PER_ADC - 1)) / 2;

const int MOTOR_ADC_SAMPLE_WINDOW_NANOSEC = SAMPLE_DURATION_NANOSEC * NUM_SAMPLES_PER_ADC;

/**
 * This parameter is dictated by the phase voltage RC filters.
 * Higher oversampling allows for a lower blanking time, due to stronger averaging.
 */
const int MOTOR_ADC_MIN_BLANKING_TIME_NANOSEC = 5000;


CONFIG_PARAM_FLOAT("mot_i_shunt_mr",         5.0,   0.1,   100.0)


static float _shunt_resistance = 0;

static uint32_t _adc1_2_dma_buffer[NUM_SAMPLES_PER_ADC];
static struct motor_adc_sample _sample;


__attribute__((optimize(3)))
CH_FAST_IRQ_HANDLER(Vector88)	// ADC1 + ADC2 handler
{
	_sample.timestamp = motor_timer_hnsec() -
		((SAMPLE_DURATION_NANOSEC * NUM_SAMPLES_PER_ADC) / 2) / NSEC_PER_HNSEC;

#define SMPLADC1(num)     (_adc1_2_dma_buffer[num] & 0xFFFFU)
#define SMPLADC2(num)     (_adc1_2_dma_buffer[num] >> 16)
	/*
	 * ADC channel sampling:
	 *   VOLT A A C
	 *   CURR C B B
	 */
	_sample.phase_values[0] = (SMPLADC1(1) + SMPLADC1(2)) / 2;
	_sample.phase_values[1] = (SMPLADC2(2) + SMPLADC2(3)) / 2;
	_sample.phase_values[2] = (SMPLADC2(1) + SMPLADC1(3)) / 2;

	_sample.input_voltage = SMPLADC1(0);
	_sample.input_current = SMPLADC2(0);

#undef SMPLADC1
#undef SMPLADC2

	motor_adc_sample_callback(&_sample);

	ADC1->SR = 0;         // Reset the IRQ flags
}

static void adc_calibrate(ADC_TypeDef* const adc)
{
	// RSTCAL
	ASSERT_ALWAYS(!(adc->CR2 & ADC_CR2_RSTCAL));
	adc->CR2 |= ADC_CR2_RSTCAL;
	while (adc->CR2 & ADC_CR2_RSTCAL) { }

	// CAL
	ASSERT_ALWAYS(!(adc->CR2 & ADC_CR2_CAL));
	adc->CR2 |= ADC_CR2_CAL;
	while (adc->CR2 & ADC_CR2_CAL) { }
}

static void enable(void)
{
	// DMA
	DMA1_Channel1->CCR = 0;  // Deinitialize
	DMA1_Channel1->CMAR = (uint32_t)_adc1_2_dma_buffer;
	DMA1_Channel1->CNDTR = sizeof(_adc1_2_dma_buffer) / 4;
	DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
	DMA1_Channel1->CCR =
		DMA_CCR_PL_0 | DMA_CCR_PL_1 |   // Max priority
		DMA_CCR_MSIZE_1 |               // 32 bit
		DMA_CCR_PSIZE_1 |               // 32 bit
		DMA_CCR_MINC |
		DMA_CCR_CIRC |
		DMA_CCR_EN;

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

	/*
	 * ADC channel sampling:
	 *   VOLT A A C
	 *   CURR C B B
	 * BEMF is sampled in the last order because the BEMF signal conditioning circuits need several
	 * microseconds to stabilize. Moving these channels to the end of the sequence allows us to reduce
	 * the overall sampling time.
	 */
	ADC1->SQR1 = ADC_SQR1_L_0 | ADC_SQR1_L_1;
	ADC1->SQR3 =
		ADC_SQR3_SQ1_2 |
		ADC_SQR3_SQ2_0 |
		ADC_SQR3_SQ3_0 |
		ADC_SQR3_SQ4_0 | ADC_SQR3_SQ4_1;

	ADC2->SQR1 = ADC1->SQR1;
	ADC2->SQR3 =
		ADC_SQR3_SQ1_2 | ADC_SQR3_SQ1_0 |
		ADC_SQR3_SQ2_0 | ADC_SQR3_SQ2_1 |
		ADC_SQR3_SQ3_1 |
		ADC_SQR3_SQ4_1;

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

int motor_adc_init(void)
{
	_shunt_resistance = configGet("mot_i_shunt_mr") / 1000.0f;

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
