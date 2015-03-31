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
#include <math.h>
#include <stm32.h>
#include <config/config.h>


#define ADC_REF_VOLTAGE          3.3f
#define ADC_INPUT_SCALE          (ADC_REF_VOLTAGE / (float)(1 << MOTOR_ADC_RESOLUTION))

#define NUM_SAMPLES_PER_ADC      6

/**
 * One ADC sample takes 15 cycles; max ADC clock is 30 MHz, so one ADC sample is:
 *    (1 / 30M) * 15 = 500 nsec
 */
#define SAMPLE_DURATION_NANOSEC  500

/**
 * ADC will be triggered at this time before the PWM mid cycle.
 */
const int MOTOR_ADC_SYNC_ADVANCE_NANOSEC = (SAMPLE_DURATION_NANOSEC * (NUM_SAMPLES_PER_ADC - 1)) / 2;

const int MOTOR_ADC_SAMPLE_WINDOW_NANOSEC = SAMPLE_DURATION_NANOSEC * NUM_SAMPLES_PER_ADC;

/**
 * Gains depend on the hardware configuration.
 * The reference design uses:
 *  - DRV8302's embedded opamps with 2 mOhm shunts and 10x gain.
 *  - Input voltage divider with ratio Rh=4700/Rl=330.
 */
#define DEFAULT_SHUNT_RESISTANCE_OHM            0.002
#define DEFAULT_CURRENT_SENSOR_GAIN             10.0

#define DEFAULT_INPUT_VOLT_DIVIDER_RHIGH        4700.0
#define DEFAULT_INPUT_VOLT_DIVIDER_RLOW         330.0

CONFIG_PARAM_FLOAT(
	"motor_phase_current_gain",
	1.0 / (DEFAULT_CURRENT_SENSOR_GAIN * DEFAULT_SHUNT_RESISTANCE_OHM),
	1.0,
	1000.0)

CONFIG_PARAM_FLOAT(
	"motor_input_voltage_gain",
	(DEFAULT_INPUT_VOLT_DIVIDER_RHIGH + DEFAULT_INPUT_VOLT_DIVIDER_RLOW) / DEFAULT_INPUT_VOLT_DIVIDER_RLOW,
	1.0,
	1000.0)


static float _phase_current_scale = 0;  // Linear
static float _input_voltage_scale = 0;  // Linear

static uint16_t _adc_dma_buffer[NUM_SAMPLES_PER_ADC * 3];
static struct motor_adc_sample _sample;


__attribute__((optimize(3)))
CH_FAST_IRQ_HANDLER(ADC1_2_3_IRQHandler)
{
	TESTPAD_SET(GPIO_PORT_TEST_ADC, GPIO_PIN_TEST_ADC);

	_sample.timestamp = motor_timer_hnsec() -
		((SAMPLE_DURATION_NANOSEC * NUM_SAMPLES_PER_ADC) / 2) / NSEC_PER_HNSEC;

#define SAMPLE(adc_num, sample_index)     (_adc_dma_buffer[3 * sample_index + (adc_num - 1)])

	// Phase voltages, unsigned
	_sample.phase_voltage_raw[0] = (SAMPLE(1, 0) + SAMPLE(1, 2) + SAMPLE(1, 3) + SAMPLE(1, 5)) / 4;
	_sample.phase_voltage_raw[1] = (SAMPLE(2, 0) + SAMPLE(2, 2) + SAMPLE(2, 3) + SAMPLE(2, 5)) / 4;
	_sample.phase_voltage_raw[2] = (SAMPLE(3, 0) + SAMPLE(3, 2) + SAMPLE(3, 3) + SAMPLE(3, 5)) / 4;

	// Phase currents, signed
	_sample.phase_current_raw[0] = ((int)(SAMPLE(1, 1) + SAMPLE(1, 4)) / 2) - MOTOR_ADC_SAMPLE_HALF;
	_sample.phase_current_raw[1] = ((int)(SAMPLE(2, 1) + SAMPLE(2, 4)) / 2) - MOTOR_ADC_SAMPLE_HALF;

	// Voltage and temperature, unsigned
	_sample.input_voltage_raw   = SAMPLE(3, 1);
	_sample.temperature_raw = SAMPLE(3, 4);

#undef SAMPLE

	motor_adc_sample_callback(&_sample);

	ADC1->SR = 0;         // Reset the IRQ flags
	TESTPAD_CLEAR(GPIO_PORT_TEST_ADC, GPIO_PIN_TEST_ADC);
}

int motor_adc_init(void)
{
	_phase_current_scale = config_get("motor_phase_current_gain") * ADC_INPUT_SCALE;
	_input_voltage_scale = config_get("motor_input_voltage_gain") * ADC_INPUT_SCALE;

	lowsyslog("Motor: ADC scales: phase current: %f, input voltage: %f\n",
		_phase_current_scale, _input_voltage_scale);

	/*
	 * DMA configuration
	 */
	chSysDisable();
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN; // No reset is possible because DMA2 is shared with other peripherals
	chSysEnable();

	/*
	 * - The user manual says that in regular simultaneous triple mode, DMA transfer requests are 32-bit:
	 *   "Three 32-bit DMA transfer requests are generated".
	 * - As one should expect from STM, the manual later contradicts itself by saying that in this mode,
	 *   "On each DMA request (one data item is available), a half-word representing an ADC-converted
	 *   data item is transferred."
	 * - The examples provided with SPL use 16-bit transfers too.
	 *
	 * STMicroelectronics, if you're reading this, kindly note that your documentation is not my favorite reading.
	 */
	DMA2_Stream0->CR =
		DMA_SxCR_PL_0 | DMA_SxCR_PL_1 |  // Highest priority
		DMA_SxCR_MSIZE_0 |               // Half-word
		DMA_SxCR_PSIZE_0 |               // Half-word
		DMA_SxCR_MINC |                  // Memory increment enabled
		DMA_SxCR_CIRC;                   // Circular mode

	DMA2_Stream0->NDTR = NUM_SAMPLES_PER_ADC * 3;
	DMA2_Stream0->PAR = (uint32_t)&ADC->CDR;
	DMA2_Stream0->M0AR = (uint32_t)&_adc_dma_buffer;

	DMA2_Stream0->CR |= DMA_SxCR_EN;         // Go go go

	/*
	 * ADC configuration
	 */
	chSysDisable();
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN | RCC_APB2ENR_ADC3EN;
	RCC->APB2RSTR |= RCC_APB2RSTR_ADCRST;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_ADCRST;
	chSysEnable();

	/*
	 * 30MHz clock
	 * Triple regular simultaneous mode
	 * DMA mode 1
	 */
	ADC->CCR =
		ADC_CCR_MULTI_4 | ADC_CCR_MULTI_2 | ADC_CCR_MULTI_1 |
		ADC_CCR_DDS |
		ADC_CCR_DMA_0;

	// ADC on
	ADC3->CR2 = ADC2->CR2 = ADC1->CR2 = ADC_CR2_ADON;

	/*
	 * ADC channel sampling pattern:
	 *         0  1  2  3  4  5
	 *   ADC1: VA IA VA VA IA VA
	 *   ADC2: VB IB VB VB IB VB
	 *   ADC3: VC VS VC VC TP VC
	 * Where:
	 *   VA/VB/VC - phase voltage A/B/C
	 *   IA/IB    - phase current A/B
	 *   VS       - supply voltage
	 *   TP       - thermistor bridge output
	 * ADC channel mapping:
	 *   VA - 0
	 *   VB - 1
	 *   VB - 2
	 *   IA - 13
	 *   IB - 12
	 *   VS - 11
	 *   TP - 10
	 */
	ADC3->SQR1 = ADC2->SQR1 = ADC1->SQR1 = ADC_SQR1_L_0 | ADC_SQR1_L_2; // bin(6-1)

	ADC1->SQR3 =
		(0  << 0)  |
		(13 << 5)  |
		(0  << 10) |
		(0  << 15) |
		(13 << 20) |
		(0  << 25);

	ADC2->SQR3 =
		(1  << 0)  |
		(12 << 5)  |
		(1  << 10) |
		(1  << 15) |
		(12 << 20) |
		(1  << 25);

	ADC3->SQR3 =
		(2  << 0)  |
		(11 << 5)  |
		(2  << 10) |
		(2  << 15) |
		(10 << 20) |
		(2  << 25);

	// SMPR registers are not configured because they have right values by default

	// ADC initialization
	ADC1->CR1 = ADC_CR1_SCAN | ADC_CR1_EOCIE;
	ADC2->CR1 = ADC_CR1_SCAN;
	ADC3->CR1 = ADC_CR1_SCAN;

	ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_EXTEN_0 | MOTOR_ADC_TRIGGER | ADC_CR2_DMA;
	ADC2->CR2 = ADC_CR2_ADON | ADC_CR2_EXTEN_0 | MOTOR_ADC_TRIGGER | ADC_CR2_DMA;
	ADC3->CR2 = ADC_CR2_ADON | ADC_CR2_EXTEN_0 | MOTOR_ADC_TRIGGER | ADC_CR2_DMA;

	// ADC IRQ (only ADC1 IRQ is used)
	chSysDisable();
	nvicEnableVector(ADC_IRQn, MOTOR_IRQ_PRIORITY_MASK);
	chSysEnable();

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
	return _input_voltage_scale * raw;
}

float motor_adc_convert_input_current(int raw)
{
	return _phase_current_scale * raw;
}

float motor_adc_convert_temperature(int raw)
{
	if ((raw <= 0) || (raw > MOTOR_ADC_SAMPLE_MAX)) {
		assert(0);
		return nan("");  // Sorry Mario, your thermistor went bananas.
	}

	// Fixed thermistor parameters - assuming that all hardware revisions use the same thermistor model
	static const float R0 = 10000;
	static const float B  = 3435;  // 25/85 degC

	// Treating the thermistor bridge as a voltage divider circuit to estimate the thermistor's resistance.
	const float Vin         = ADC_REF_VOLTAGE;
	const float Vout        = raw * ADC_INPUT_SCALE;
	const float Rload       = R0;                   // It is a basic assumption of the algorithm that Rload = R0
	const float Rthermistor = ((Vin - Vout) * Rload) / Vout;

	// Applying the Steinhart-Hart equation
	const float T0          = 298.15;              // 25 degC in Kelvin
	const float rinf        = R0 * expf(-B / T0);
	const float temperature = B / logf(Rthermistor / rinf);

	return temperature;
}
