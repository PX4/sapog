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

#pragma once

#include <stdint.h>
#include <hal.h>
#include "internal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_ADC1_2_TRIGGER    (ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_0)

struct motor_pwm_commutation_step
{
	int_fast8_t positive;
	int_fast8_t negative;
	int_fast8_t floating;
};

enum motor_pwm_phase_manip
{
	MOTOR_PWM_MANIP_LOW,
	MOTOR_PWM_MANIP_HIGH,
	MOTOR_PWM_MANIP_FLOATING,
	MOTOR_PWM_MANIP_HALF,
	MOTOR_PWM_MANIP_END_
};

/**
 * Sanity constraints
 */
#define MOTOR_PWM_MIN_FREQUENCY   10000
#define MOTOR_PWM_MAX_FREQUENCY   70000

/**
 * Initialize the PWM hardware.
 * @return 0 on success, anything else if the requested frequency is invalid
 */
int motor_pwm_init(void);

/**
 * ADC converstions are triggered by the PWM hardware, so this function is here
 */
uint32_t motor_adc_sampling_period_hnsec(void);

/**
 * Direct phase control - for self-testing
 */
void motor_pwm_manip(const enum motor_pwm_phase_manip command[MOTOR_NUM_PHASES]);

void motor_pwm_set_freewheeling(void);

void motor_pwm_emergency(void);

/**
 * Duty cycle in [-1; 1]
 */
int motor_pwm_compute_pwm_val(float duty_cycle);

void motor_pwm_set_step_from_isr(const struct motor_pwm_commutation_step* step, int pwm_val);

/**
 * Should be called from high priority threads
 */
void motor_pwm_beep(int frequency, int duration_msec);

#ifdef __cplusplus
}
#endif
