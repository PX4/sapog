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
#include <math.h>
#include <stm32f10x.h>
#include "pwm.h"
#include "adc.h"
#include "timer.h"

#define PWM_TIMER_FREQUENCY     STM32_TIMCLK1

/**
 * Duty cycle is limited to maintain the charge on the high side pump capacitor.
 */
#define PWM_MIN_PULSE_NANOSEC   1000

#define PWM_DEAD_TIME_NANOSEC   500

/**
 * PWM channel mapping
 */
static volatile uint16_t* const PWM_REG_HIGH[MOTOR_NUM_PHASES] = {
	&TIM4->CCR1,
	&TIM4->CCR2,
	&TIM4->CCR3
};
static volatile uint16_t* const PWM_REG_LOW[MOTOR_NUM_PHASES] = {
	&TIM3->CCR2,
	&TIM3->CCR3,
	&TIM3->CCR4
};

static const uint16_t TIM4_HIGH_CCER_POL[MOTOR_NUM_PHASES] = {
	TIM_CCER_CC1P,
	TIM_CCER_CC2P,
	TIM_CCER_CC3P
};
static const uint16_t TIM3_LOW_CCER_POL[MOTOR_NUM_PHASES] = {
	TIM_CCER_CC2P,
	TIM_CCER_CC3P,
	TIM_CCER_CC4P
};
static const uint16_t TIM4_HIGH_CCER_POL_MASK = TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P;
static const uint16_t TIM3_LOW_CCER_POL_MASK  = TIM_CCER_CC2P | TIM_CCER_CC3P | TIM_CCER_CC4P;

static const uint16_t TIM4_HIGH_CCER_EN[MOTOR_NUM_PHASES] = {
	TIM_CCER_CC1E,
	TIM_CCER_CC2E,
	TIM_CCER_CC3E
};
static const uint16_t TIM3_LOW_CCER_EN[MOTOR_NUM_PHASES] = {
	TIM_CCER_CC2E,
	TIM_CCER_CC3E,
	TIM_CCER_CC4E
};
static const uint16_t TIM4_HIGH_CCER_EN_MASK = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
static const uint16_t TIM3_LOW_CCER_EN_MASK  = TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

/**
 * Local constants, initialized once
 */
static uint16_t _pwm_top;
static uint16_t _pwm_half_top;

static uint16_t _pwm_max;
static uint16_t _pwm_min;

static uint16_t _pwm_dead_time_ticks;
static bool _prevent_full_duty_cycle_bump = true;


static int init_constants(unsigned frequency)
{
	assert_always(_pwm_top == 0);   // Make sure it was not initialized already

	/*
	 * PWM top, derived from frequency
	 */
	if (frequency < MOTOR_PWM_MIN_FREQUENCY || frequency > MOTOR_PWM_MAX_FREQUENCY) {
		return -1;
	}

	const int pwm_steps = PWM_TIMER_FREQUENCY / (2 * frequency);
	if (pwm_steps > 65536) {
		assert(0);
		return -1;
	}

	_pwm_top = pwm_steps - 1;
	_pwm_half_top = pwm_steps / 2;

	const int effective_steps = pwm_steps / 2;
	const float true_frequency = PWM_TIMER_FREQUENCY / (float)(pwm_steps * 2);
	const unsigned adc_period_usec = motor_adc_sampling_period_hnsec() / HNSEC_PER_USEC;
	lowsyslog("Motor: PWM freq: %f; Effective steps: %i; ADC period: %u usec\n",
		true_frequency, effective_steps, adc_period_usec);

	/*
	 * PWM max - Limited by PWM_MIN_PULSE_NANOSEC
	 */
	const float pwm_clock_period = 1.f / PWM_TIMER_FREQUENCY;
	const float pwm_min_pulse_len = PWM_MIN_PULSE_NANOSEC / 1e9f;
	const float pwm_min_pulse_ticks_float = pwm_min_pulse_len / pwm_clock_period;
	assert(pwm_min_pulse_ticks_float >= 0);
	assert(pwm_min_pulse_ticks_float < (_pwm_top * 0.3f));

	const uint16_t pwm_min_pulse_ticks = (uint16_t)pwm_min_pulse_ticks_float;

	// We need to divide the min value by 2 since we're using the center-aligned PWM
	_pwm_max = _pwm_top - (pwm_min_pulse_ticks / 2 + 1);

	/*
	 * PWM min - Limited by MOTOR_ADC_SAMPLE_WINDOW_NANOSEC
	 */
	const float pwm_adc_window_len = (MOTOR_ADC_SAMPLE_WINDOW_NANOSEC / 1e9f) * 2;
	const float pwm_adc_window_ticks_float = pwm_adc_window_len / pwm_clock_period;
	assert(pwm_adc_window_ticks_float >= 0);

	// Same here - divide by 2
	uint16_t pwm_half_adc_window_ticks = ((uint16_t)pwm_adc_window_ticks_float) / 2;
	if (pwm_half_adc_window_ticks > _pwm_half_top)
		pwm_half_adc_window_ticks = _pwm_half_top;

	_pwm_min = pwm_half_adc_window_ticks;
	assert(_pwm_min <= _pwm_half_top);

	/*
	 * PWM dead time
	 */
	const float pwm_dead_time = PWM_DEAD_TIME_NANOSEC / 1e9f;
	const float pwm_dead_time_ticks_float = pwm_dead_time / pwm_clock_period;
	assert(pwm_dead_time_ticks_float >= 0);
	assert(pwm_dead_time_ticks_float < (_pwm_top * 0.2f));

	// Dead time shall not be halved
	_pwm_dead_time_ticks = (uint16_t)pwm_dead_time_ticks_float;

	lowsyslog("Motor: PWM top: %u; PWM range: [%u, %u]; Dead time: %u ticks\n",
		(unsigned)_pwm_top, (unsigned)_pwm_min, (unsigned)_pwm_max, (unsigned)_pwm_dead_time_ticks);
	return 0;
}

static void init_timers(void)
{
	assert_always(_pwm_top > 0);   // Make sure it was initialized

	chSysDisable();

	// Enable and reset
	const uint32_t enr_mask = RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;
	const uint32_t rst_mask = RCC_APB1RSTR_TIM3RST | RCC_APB1RSTR_TIM4RST;
	RCC->APB1ENR |= enr_mask;
	RCC->APB1RSTR |= rst_mask;
	RCC->APB1RSTR &= ~rst_mask;

	chSysEnable();

	// Reload value
	TIM3->ARR = TIM4->ARR = _pwm_top;

	// Buffered update, center-aligned PWM
	TIM3->CR1 = TIM4->CR1 = TIM_CR1_ARPE | TIM_CR1_CMS_0;

	// OC channels (all enabled)
	TIM3->CCMR1 = TIM4->CCMR1 =
		TIM_CCMR1_OC1FE | TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 |
		TIM_CCMR1_OC2FE | TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;

	TIM3->CCMR2 = TIM4->CCMR2 =
		TIM_CCMR2_OC3FE | TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 |
		TIM_CCMR2_OC4FE | TIM_CCMR2_OC4PE | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;

	// OC polarity (no inversion by default; all phases are disabled by default except ADC sync)
	TIM3->CCER = 0;
	TIM4->CCER = TIM_CCER_CC4E; // ADC sync

	/*
	 * ADC synchronization
	 *
	 * 100%     /\      /
	 * 75%     /  \    /
	 * 25%    /    \  /
	 * 0%    /      \/
	 *             A  B
	 * Complementary PWM operates in range (50%, 100%], thus a FET commutation will never happen in range
	 * between points A and B on the diagram above (save the braking mode, but it's negligible).
	 * ADC shall be triggered either at PWM top or PWM bottom in order to catch the moment when the instant
	 * winding current matches with average winding current - this helps to eliminate the current ripple
	 * caused by the PWM switching.
	 * Thus we trigger ADC at the bottom minus advance.
	 * Refer to "Synchronizing the On-Chip Analog-to-Digital Converter on 56F80x Devices" for some explanation.
	 */
	const float adc_trigger_advance = MOTOR_ADC_SYNC_ADVANCE_NANOSEC / 1e9f;
	const float adc_trigger_advance_ticks_float = adc_trigger_advance / (1.f / PWM_TIMER_FREQUENCY);
	assert(adc_trigger_advance_ticks_float >= 0);
	assert(adc_trigger_advance_ticks_float < (_pwm_top * 0.4f));
	TIM4->CCR4 = (uint16_t)adc_trigger_advance_ticks_float;
	if (TIM4->CCR4 == 0)
		TIM4->CCR4 = 1;

	// Timers are configured now but not started yet. Starting is tricky because of synchronization, see below.
	TIM3->EGR = TIM_EGR_UG;
	TIM4->EGR = TIM_EGR_UG | TIM_EGR_COMG;
}

static void start_timers(void)
{
	// Make sure the timers are not running
	assert_always(!(TIM3->CR1 & TIM_CR1_CEN));
	assert_always(!(TIM4->CR1 & TIM_CR1_CEN));

	// Start synchronously
	TIM3->CR2 |= TIM_CR2_MMS_0;                                   // TIM3 is master
	TIM4->SMCR = TIM_SMCR_SMS_1 | TIM_SMCR_SMS_2 | TIM_SMCR_MSM | TIM_SMCR_TS_1; // TIM4 is slave

	TIM3->CR1 |= TIM_CR1_CEN;                                     // Start

	// Remove the synchronization
	TIM3->CR2 &= ~TIM_CR2_MMS;
	TIM4->SMCR = 0;

	// Make sure the timers have started now
	assert_always(TIM3->CR1 & TIM_CR1_CEN);
	assert_always(TIM4->CR1 & TIM_CR1_CEN);
}

int motor_pwm_init(unsigned frequency, bool prevent_full_duty_cycle_bump)
{
	const int ret = init_constants(frequency);
	if (ret)
		return ret;

	_prevent_full_duty_cycle_bump = prevent_full_duty_cycle_bump;
	lowsyslog("Motor: PWM bump suppression: %i\n", (int)_prevent_full_duty_cycle_bump);

	init_timers();
	start_timers();

	motor_pwm_set_freewheeling();
	return 0;
}

uint32_t motor_adc_sampling_period_hnsec(void)
{
	return HNSEC_PER_SEC / (PWM_TIMER_FREQUENCY / (((int)_pwm_top + 1) * 2));
}

/**
 * Safely turns off all phases avoiding shoot-through.
 * Assumes:
 *  - motor IRQs are disabled
 */
static void phase_reset_all_i(void)
{
	/*
	 * Outputs must be disabled until the phase setup is done.
	 * Otherwise there might occur an occasional shoot-through while the
	 * phase inversion is configured but PWM register is not, or vice versa.
	 */
	// Disable inversions and outputs:
	TIM3->CCER &= ~(TIM3_LOW_CCER_EN_MASK  | TIM3_LOW_CCER_POL_MASK);
	TIM4->CCER &= ~(TIM4_HIGH_CCER_EN_MASK | TIM4_HIGH_CCER_POL_MASK);
	// Reset PWM registers:
	for (int phase = 0; phase < MOTOR_NUM_PHASES; phase++) {
		*PWM_REG_HIGH[phase] = 0;
		*PWM_REG_LOW[phase] = 0;
	}
}

/**
 * Safely turns off one phase.
 * Assumes:
 *  - motor IRQs are disabled
 */
__attribute__((optimize(3)))
static inline void phase_reset_i(int phase)
{
	assert(phase >= 0 && phase < MOTOR_NUM_PHASES);
	// Disable inversions and outputs:
	TIM3->CCER &= ~(TIM3_LOW_CCER_EN[phase]  | TIM3_LOW_CCER_POL[phase]);
	TIM4->CCER &= ~(TIM4_HIGH_CCER_EN[phase] | TIM4_HIGH_CCER_POL[phase]);
	// Reset PWM registers:
	*PWM_REG_HIGH[phase] = 0;
	*PWM_REG_LOW[phase] = 0;
}

/**
 * Turns the given phase on.
 * Assumes:
 *  - motor IRQs are disabled
 */
__attribute__((optimize(3)))
static inline void phase_enable_i(int phase)
{
	assert(phase >= 0 && phase < MOTOR_NUM_PHASES);
	TIM3->CCER |= TIM3_LOW_CCER_EN[phase];
	TIM4->CCER |= TIM4_HIGH_CCER_EN[phase];
}

/**
 * Assumes:
 *  - motor IRQs are disabled
 *
 * TODO: Current implementation generates unwanted spikes when switching from floating mode to driving mode.
 * This shall be fixed with the new hardware, where the PWM logic is completely different.
 * Scope traces showing this bug:
 *    http://habrastorage.org/storage3/532/7b3/c10/5327b3c1085aa8df7628299e400bc9f6.png
 *    http://habrastorage.org/storage3/50b/348/e6b/50b348e6b903d934fd2a891db336d392.png
 */
__attribute__((optimize(3)))
static inline void phase_set_i(int phase, const int pwm_val, bool inverted)
{
	assert(phase >= 0 && phase < MOTOR_NUM_PHASES);

	uint_fast16_t duty_cycle_high = pwm_val;
	uint_fast16_t duty_cycle_low  = pwm_val;

	if (inverted) {
		// Inverted - high PWM is inverted, low is not
		if (TIM3->CCER & TIM3_LOW_CCER_POL[phase] || !(TIM4->CCER & TIM4_HIGH_CCER_POL[phase])) {
			phase_reset_i(phase);
			TIM4->CCER |= TIM4_HIGH_CCER_POL[phase];
		}

		// Inverted phase shall have greater PWM value than non-inverted one
		// On full PWM there will be no switching at all
		if (pwm_val < _pwm_top) {
			if (pwm_val > _pwm_half_top)
				duty_cycle_low  -= _pwm_dead_time_ticks;
			else
				duty_cycle_high += _pwm_dead_time_ticks;
		}
	} else {
		// Normal - low PWM is inverted, high is not
		if (TIM4->CCER & TIM4_HIGH_CCER_POL[phase] || !(TIM3->CCER & TIM3_LOW_CCER_POL[phase])) {
			phase_reset_i(phase);
			TIM3->CCER |= TIM3_LOW_CCER_POL[phase];
		}

		if (pwm_val < _pwm_top) {
			if (pwm_val > _pwm_half_top)
				duty_cycle_high -= _pwm_dead_time_ticks;
			else
				duty_cycle_low  += _pwm_dead_time_ticks;
		}
	}

	// And as always, thanks for watching.
	*PWM_REG_HIGH[phase] = duty_cycle_high;
	*PWM_REG_LOW[phase] = duty_cycle_low;
}

void motor_pwm_manip(const enum motor_pwm_phase_manip command[MOTOR_NUM_PHASES])
{
	irq_primask_disable();
	phase_reset_all_i();
	irq_primask_enable();

	const uint64_t dead_time_expiration = motor_timer_hnsec() + PWM_DEAD_TIME_NANOSEC / NSEC_PER_HNSEC;

	for (int phase = 0; phase < MOTOR_NUM_PHASES; phase++) {
		switch (command[phase]) {
		case MOTOR_PWM_MANIP_HIGH:
		case MOTOR_PWM_MANIP_HALF: {
			const float duty_cycle = (command[phase] == MOTOR_PWM_MANIP_HIGH) ? 1.f : 0.f;

			const int pwm_val = motor_pwm_compute_pwm_val(duty_cycle);

			irq_primask_disable();
			phase_set_i(phase, pwm_val, false);
			irq_primask_enable();
			break;
		}
		case MOTOR_PWM_MANIP_LOW:
			*PWM_REG_HIGH[phase] = 0;
			*PWM_REG_LOW[phase] = _pwm_top;
			break;
		case MOTOR_PWM_MANIP_FLOATING:
			// Nothing to do
			break;
		default:
			assert(0);
		}
	}

	// This loop should be skipped immediately because all the processing above takes enough time.
	while (motor_timer_hnsec() <= dead_time_expiration) { }

	for (int phase = 0; phase < MOTOR_NUM_PHASES; phase++) {
		if (command[phase] == MOTOR_PWM_MANIP_FLOATING)
			continue;
		irq_primask_disable();
		phase_enable_i(phase);
		irq_primask_enable();
	}
}

void motor_pwm_align(const int polarities[MOTOR_NUM_PHASES], int pwm_val)
{
	irq_primask_disable();
	phase_reset_all_i();
	irq_primask_enable();

	const uint64_t dead_time_expiration = motor_timer_hnsec() + PWM_DEAD_TIME_NANOSEC / NSEC_PER_HNSEC;

	for (int phase = 0; phase < MOTOR_NUM_PHASES; phase++) {
		const int pol = polarities[phase];
		assert(pol == 0 || pol == 1 || pol == -1);

		irq_primask_disable();
		if (pol > 0)
			phase_set_i(phase, pwm_val, false);
		else if (pol == 0)
			phase_set_i(phase, pwm_val, true);
		irq_primask_enable();
	}

	// Just in case
	while (motor_timer_hnsec() <= dead_time_expiration) { }

	irq_primask_disable();
	for (int phase = 0; phase < MOTOR_NUM_PHASES; phase++) {
		if (polarities[phase] >= 0)
			phase_enable_i(phase);
	}
	irq_primask_enable();
}

void motor_pwm_set_freewheeling(void)
{
	irq_primask_disable();
	phase_reset_all_i();
	irq_primask_enable();
}

void motor_pwm_emergency(void)
{
	const irqstate_t irqstate = irq_primask_save();
	phase_reset_all_i();
	irq_primask_restore(irqstate);
}

int motor_pwm_compute_pwm_val(float duty_cycle)
{
	/*
	 * Normalize into [0; PWM_TOP] regardless of sign
	 */
	const float abs_duty_cycle = fabs(duty_cycle);
	uint_fast16_t int_duty_cycle = 0;
	if (abs_duty_cycle > 0.999)
		int_duty_cycle = _pwm_top;
	else
		int_duty_cycle = (uint_fast16_t)(abs_duty_cycle * _pwm_top);

	/*
	 * Compute the complementary duty cycle
	 * Ref. "Influence of PWM Schemes and Commutation Methods for DC and Brushless Motors and Drives", page 4.
	 */
	int output = 0;

	if (duty_cycle >= 0) {
		// Forward mode
		output = _pwm_top - ((_pwm_top - int_duty_cycle) / 2);

		if (output > _pwm_max) {
			output = _pwm_max;

			const uint_fast16_t bump_threshold = (_pwm_top + _pwm_max) / 2;
			if (!_prevent_full_duty_cycle_bump && (int_duty_cycle > bump_threshold))
				output = _pwm_top;
		}

		assert(output >= _pwm_half_top);
		assert(output <= _pwm_top);
	} else {
		// Braking mode
		output = (_pwm_top - int_duty_cycle) / 2;

		if (output < _pwm_min)
			output = _pwm_min;

		assert(output >= 0);
		assert(output <= _pwm_half_top);
	}

	return output;
}

__attribute__((optimize(3)))
void motor_pwm_set_step_from_isr(const struct motor_pwm_commutation_step* step, int pwm_val)
{
	phase_reset_i(step->floating);

	phase_set_i(step->positive, pwm_val, false);
	phase_set_i(step->negative, pwm_val, true);

	phase_enable_i(step->positive);
	phase_enable_i(step->negative);
}

void motor_pwm_beep(int frequency, int duration_msec)
{
	static const float DUTY_CYCLE = 0.008;
	static const int ACTIVE_USEC_MAX = 20;

	motor_pwm_set_freewheeling();

	frequency = (frequency < 100)  ? 100  : frequency;
	frequency = (frequency > 5000) ? 5000 : frequency;

	duration_msec = (duration_msec < 1)    ? 1    : duration_msec;
	duration_msec = (duration_msec > 5000) ? 5000 : duration_msec;

	/*
	 * Timing constants
	 */
	const int half_period_hnsec = (HNSEC_PER_SEC / frequency) / 2;

	int active_hnsec = half_period_hnsec * DUTY_CYCLE;

	if (active_hnsec < PWM_MIN_PULSE_NANOSEC / NSEC_PER_HNSEC)
		active_hnsec = PWM_MIN_PULSE_NANOSEC / NSEC_PER_HNSEC;

	if (active_hnsec > ACTIVE_USEC_MAX * HNSEC_PER_USEC)
		active_hnsec = ACTIVE_USEC_MAX * HNSEC_PER_USEC;

	const int idle_hnsec = half_period_hnsec - active_hnsec;

	const uint64_t end_time = motor_timer_hnsec() + duration_msec * HNSEC_PER_MSEC;

	/*
	 * FET round robin
	 * This way we can beep even if some FETs went bananas
	 */
	static unsigned _phase_sel;
	const int low_phase_first  = _phase_sel++ % MOTOR_NUM_PHASES;
	const int low_phase_second = _phase_sel++ % MOTOR_NUM_PHASES;
	const int high_phase       = _phase_sel++ % MOTOR_NUM_PHASES;
	_phase_sel++;              // We need to increment it not by multiple of 3
	assert(low_phase_first != high_phase && low_phase_second != high_phase);

	/*
	 * Commutations
	 * No high side pumping
	 */
	*PWM_REG_LOW[low_phase_first]  = _pwm_top;
	*PWM_REG_LOW[low_phase_second] = _pwm_top;
	phase_enable_i(low_phase_first);
	phase_enable_i(low_phase_second);
	phase_enable_i(high_phase);

	while (end_time > motor_timer_hnsec()) {
		chSysSuspend();

		*PWM_REG_HIGH[high_phase] = _pwm_top;
		motor_timer_hndelay(active_hnsec);
		*PWM_REG_HIGH[high_phase] = 0;

		chSysEnable();

		motor_timer_hndelay(idle_hnsec);
	}

	motor_pwm_set_freewheeling();
}
