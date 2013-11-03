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
#include <stm32f10x.h>
#include "pwm.h"
#include "adc.h"
#include "timer.h"

#define PWM_TIMER_FREQUENCY     STM32_TIMCLK1

/**
 * Duty cycle is limited to maintain the charge on the high side capacitor.
 */
#define PWM_MIN_PULSE_NANOSEC   300

/**
 * Shoot-through test for IR2301S + IRLR7843:
 *   300ns - average shoot-through current is about 2mA at 35kHz
 *   400ns - less than 1mA at 35kHz
 *   500ns - much less than 1mA
 */
#define PWM_DEAD_TIME_NANOSEC   400

/**
 * PWM is used in center-aligned mode, so the frequency is defined as:
 *      f = pwm_clock / ((pwm_top + 1) * 2)
 *
 * For 72MHz clock, the PWM frequencies are:
 *      70312.5 Hz    @ 9 bit  (this is likely too high for ADC processing)
 *      35156.25 Hz   @ 10 bit
 *      17578.125 Hz  @ 11 bit
 *       8789.0625 Hz @ 12 bit
 * Effective resolution is always one bit less (because of complementary PWM).
 */
#define PWM_TRUE_RESOLUTION 10

#define PWM_TOP        ((1 << PWM_TRUE_RESOLUTION) - 1)
#define PWM_HALF_TOP   ((1 << PWM_TRUE_RESOLUTION) / 2)


/**
 * PWM channel mapping
 */
static volatile uint16_t* const PWM_REG_HIGH[3] = {
	&TIM4->CCR1,
	&TIM4->CCR2,
	&TIM4->CCR3
};
static volatile uint16_t* const PWM_REG_LOW[3] = {
	&TIM3->CCR2,
	&TIM3->CCR3,
	&TIM3->CCR4
};

static const uint16_t TIM4_HIGH_CCER_POL[3] = {
	TIM_CCER_CC1P,
	TIM_CCER_CC2P,
	TIM_CCER_CC3P
};
static const uint16_t TIM3_LOW_CCER_POL[3] = {
	TIM_CCER_CC2P,
	TIM_CCER_CC3P,
	TIM_CCER_CC4P
};
static const uint16_t TIM4_HIGH_CCER_POL_MASK = TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P;
static const uint16_t TIM3_LOW_CCER_POL_MASK  = TIM_CCER_CC2P | TIM_CCER_CC3P | TIM_CCER_CC4P;

static const uint16_t TIM4_HIGH_CCER_EN[3] = {
	TIM_CCER_CC1E,
	TIM_CCER_CC2E,
	TIM_CCER_CC3E
};
static const uint16_t TIM3_LOW_CCER_EN[3] = {
	TIM_CCER_CC2E,
	TIM_CCER_CC3E,
	TIM_CCER_CC4E
};
static const uint16_t TIM4_HIGH_CCER_EN_MASK = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
static const uint16_t TIM3_LOW_CCER_EN_MASK  = TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

static uint16_t _pwm_max;
static uint16_t _pwm_dead_time_ticks;


static void init_timers(void)
{
	chSysDisable();

	// Enable and reset
	const uint32_t enr_mask = RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;
	const uint32_t rst_mask = RCC_APB1RSTR_TIM3RST | RCC_APB1RSTR_TIM4RST;
	RCC->APB1ENR |= enr_mask;
	RCC->APB1RSTR |= rst_mask;
	RCC->APB1RSTR &= ~rst_mask;

	chSysEnable();

	// Reload value
	TIM3->ARR = TIM4->ARR = PWM_TOP;

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

	// ADC synchronization
	const float adc_trigger_advance = MOTOR_ADC_SYNC_ADVANCE_NANOSEC / 1e9f;
	const float adc_trigger_advance_ticks_float = adc_trigger_advance / (1.f / PWM_TIMER_FREQUENCY);
	assert_always(adc_trigger_advance_ticks_float >= 0);
	assert_always(adc_trigger_advance_ticks_float < (PWM_TOP * 0.3f));

	TIM4->CCR4 = PWM_HALF_TOP - (uint16_t)adc_trigger_advance_ticks_float;

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

void motor_pwm_init(void)
{
	init_timers();
	start_timers();

	// PWM limits
	const float pwm_clock_period = 1.f / PWM_TIMER_FREQUENCY;
	const float pwm_min_pulse_len = PWM_MIN_PULSE_NANOSEC / 1e9f;
	const float pwm_min_pulse_ticks_float = pwm_min_pulse_len / pwm_clock_period;
	assert_always(pwm_min_pulse_ticks_float >= 0);
	assert_always(pwm_min_pulse_ticks_float < (PWM_TOP * 0.05f));

	const uint16_t pwm_min_pulse_ticks = (uint16_t)pwm_min_pulse_ticks_float;

	// We need to divide the min value by 2 since we're using the center-aligned PWM
	_pwm_max = PWM_TOP - (pwm_min_pulse_ticks / 2 + 1);

	// PWM dead time
	const float pwm_dead_time = PWM_DEAD_TIME_NANOSEC / 1e9f;
	const float pwm_dead_time_ticks_float = pwm_dead_time / pwm_clock_period;
	assert_always(pwm_dead_time_ticks_float >= 0);
	assert_always(pwm_dead_time_ticks_float < (PWM_TOP * 0.05f));

	// Dead time shall not be halved
	_pwm_dead_time_ticks = (uint16_t)pwm_dead_time_ticks_float;

	lowsyslog("Motor: PWM max: %u; Dead time: %u ticks\n",
	          (unsigned int)_pwm_max, (unsigned int)_pwm_dead_time_ticks);

	// This step is required to complete the initialization
	motor_pwm_set_freewheeling();
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
	for (int phase = 0; phase < 3; phase++) {
		*PWM_REG_HIGH[phase] = 0;
		*PWM_REG_LOW[phase] = 0;
	}
}

/**
 * Turns the given phase on.
 * Assumes:
 *  - motor IRQs are disabled
 */
static void phase_enable_i(int phase)
{
	assert(phase >= 0 && phase < 3);
	TIM3->CCER |= TIM3_LOW_CCER_EN[phase];
	TIM4->CCER |= TIM4_HIGH_CCER_EN[phase];
}

/**
 * Assumes:
 *  - motor IRQs are disabled
 *  - phase resetted and disabled
 */
static void phase_set_i(int phase, const struct motor_pwm_val* pwm_val, bool inverted)
{
	assert(phase >= 0 && phase < 3);
	assert(pwm_val);

	// Make sure the phase is in default state
	assert(!(TIM3->CCER & TIM3_LOW_CCER_POL[phase]));
	assert(!(TIM4->CCER & TIM4_HIGH_CCER_POL[phase]));

	uint_fast16_t duty_cycle_high = pwm_val->normalized_duty_cycle;
	uint_fast16_t duty_cycle_low  = pwm_val->normalized_duty_cycle;

	if (inverted) {
		// Inverted - high PWM is inverted, low is not
		TIM4->CCER |= TIM4_HIGH_CCER_POL[phase];

		// Inverted phase shall have greater PWM value than non-inverted one
		if (pwm_val->normalized_duty_cycle > PWM_HALF_TOP)
			duty_cycle_low  -= _pwm_dead_time_ticks;
		else
			duty_cycle_high += _pwm_dead_time_ticks;
	} else {
		// Normal - low PWM is inverted, high is not
		TIM3->CCER |= TIM3_LOW_CCER_POL[phase];

		if (pwm_val->normalized_duty_cycle > PWM_HALF_TOP)
			duty_cycle_high -= _pwm_dead_time_ticks;
		else
			duty_cycle_low  += _pwm_dead_time_ticks;
	}

	*PWM_REG_HIGH[phase] = duty_cycle_high;
	*PWM_REG_LOW[phase] = duty_cycle_low;
}

void motor_pwm_manip(const enum motor_pwm_phase_manip command[3])
{
	irq_primask_disable();
	phase_reset_all_i();
	irq_primask_enable();

	const uint64_t dead_time_expiration = motor_timer_hnsec() + PWM_DEAD_TIME_NANOSEC / NSEC_PER_HNSEC;

	for (int phase = 0; phase < 3; phase++) {
		switch (command[phase]) {
		case MOTOR_PWM_MANIP_HIGH:
		case MOTOR_PWM_MANIP_HALF: {
			const uint16_t duty_cycle = (command[phase] == MOTOR_PWM_MANIP_HIGH)
			        ? MOTOR_PWM_DUTY_CYCLE_MAX : 0;

			struct motor_pwm_val pwm_val;
			motor_pwm_compute_pwm_val(duty_cycle, &pwm_val);

			irq_primask_disable();
			phase_set_i(phase, &pwm_val, false);
			irq_primask_enable();
			break;
		}
		case MOTOR_PWM_MANIP_LOW:
			*PWM_REG_HIGH[phase] = 0;
			*PWM_REG_LOW[phase] = PWM_TOP;
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

	for (int phase = 0; phase < 3; phase++) {
		if (command[phase] == MOTOR_PWM_MANIP_FLOATING)
			continue;
		irq_primask_disable();
		phase_enable_i(phase);
		irq_primask_enable();
	}
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

void motor_pwm_compute_pwm_val(uint16_t duty_cycle, struct motor_pwm_val* out_val)
{
	assert(out_val);

	// Discard extra least significant bits
	const uint_fast16_t corrected_duty_cycle =
		duty_cycle >> (MOTOR_PWM_DUTY_CYCLE_RESOLUTION - PWM_TRUE_RESOLUTION);

	// Ref. "Influence of PWM Schemes and Commutation Methods for DC and Brushless Motors and Drives", page 4.
	out_val->normalized_duty_cycle = PWM_TOP - ((PWM_TOP - corrected_duty_cycle) / 2);

	// Maintain the proper cycling for the high-side pump capacitor
	if (out_val->normalized_duty_cycle > _pwm_max)
		out_val->normalized_duty_cycle = _pwm_max;

	assert(out_val->normalized_duty_cycle >= PWM_HALF_TOP);
	assert(out_val->normalized_duty_cycle <= PWM_TOP);
}

void motor_pwm_set_step_from_isr(const struct motor_pwm_commutation_step* step, const struct motor_pwm_val* pwm_val)
{
	phase_reset_all_i();

	phase_set_i(step->positive, pwm_val, false);
	phase_set_i(step->negative, pwm_val, true);
	// Floating phase was configured with the reset call above

	phase_enable_i(step->positive);
	phase_enable_i(step->negative);
}

void motor_pwm_beep(int frequency, int duration_msec)
{
	static const int ENERGIZING_DURATION_USEC = 10;

	if (frequency < 200)
		frequency = 200;
	if (duration_msec < 1)
		duration_msec = 1;

	// 1 is always low, 0 and 2 are alternating
	enum motor_pwm_phase_manip manip_cmd[3] = {
		MOTOR_PWM_MANIP_FLOATING,
		MOTOR_PWM_MANIP_LOW,
		MOTOR_PWM_MANIP_FLOATING
	};
	motor_pwm_manip(manip_cmd);

	const int half_period_usec = (1000000 / frequency) / 2;
	const uint64_t end_time = motor_timer_hnsec() + duration_msec * HNSEC_PER_MSEC;

	while (end_time > motor_timer_hnsec()) {
		// 0
		manip_cmd[0] = MOTOR_PWM_MANIP_HIGH;
		motor_pwm_manip(manip_cmd);
		motor_timer_udelay(ENERGIZING_DURATION_USEC);

		manip_cmd[0] = MOTOR_PWM_MANIP_FLOATING;
		motor_pwm_manip(manip_cmd);
		motor_timer_udelay(half_period_usec);

		// 2
		manip_cmd[2] = MOTOR_PWM_MANIP_HIGH;
		motor_pwm_manip(manip_cmd);
		motor_timer_udelay(ENERGIZING_DURATION_USEC);

		manip_cmd[2] = MOTOR_PWM_MANIP_FLOATING;
		motor_pwm_manip(manip_cmd);
		motor_timer_udelay(half_period_usec);
	}

	motor_pwm_set_freewheeling();
}
