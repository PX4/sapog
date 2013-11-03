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

#include <string.h>
#include <assert.h>
#include "motor.h"
#include "adc.h"
#include "pwm.h"
#include "timer.h"
#include "test.h"

static uint32_t erpm_to_comm_period(uint32_t erpm);


#define HNSEC_PER_MINUTE      (HNSEC_PER_SEC * 60)
#define NUM_PHASES            3
#define NUM_COMMUTATION_STEPS 6

/**
 * The neutral voltage will be shifted by this amount to yield the highest ZC threshold.
 * Read the code for details.
 */
#define ZC_THRESHOLD_SHIFT_BASE     4

static const struct motor_pwm_commutation_step COMMUTATION_TABLE[NUM_COMMUTATION_STEPS] = {
    {1, 0, 2}, // Positive, negative, floating
    {1, 2, 0},
    {0, 2, 1},
    {0, 1, 2},
    {2, 1, 0},
    {2, 0, 1}
};


enum control_state_id { CS_IDLE, CS_BEFORE_ZC, CS_PAST_ZC };

static struct control_state
{
	enum control_state_id control_state;

	uint64_t blank_time_deadline;
	uint64_t prev_zc_timestamp;
	uint64_t predicted_zc_timestamp;
	uint64_t prev_adc_sample_timestamp;
	uint32_t comm_period;

	int current_comm_step;
	bool reverse_rotation;

	int zc_threshold_shift;
	int successive_zc_failures;
	uint64_t zc_failures_since_start;

	int prev_adc_normalized_sample;
	int phase_avg_volt[3];

	struct motor_pwm_val pwm_val;
} state;

static struct precomputed_params
{
	int spinup_erpm;
	int spinup_steps;

	int comm_blank_usec;
	int timing_advance_deg;
	int zc_failures_max;

	int comm_period_lowpass_alpha_reciprocal; // Reciprocal of lowpass alpha (0; 1]
	uint32_t max_acceleration_per_step_x64;   // Like percent but in range [0; 64] for faster division

	uint32_t comm_period_zc_threshold_first;  // slow/moderate
	uint32_t comm_period_zc_threshold_second; // moderate/fast

	uint32_t comm_period_min;
	uint32_t comm_period_max;
} params;

static void configure(void) // TODO: obtain the configuration from somewhere else
{
	params.spinup_erpm = 1000;
	params.spinup_steps = 4;

	params.comm_blank_usec = 30;
	params.timing_advance_deg = 10;
	params.zc_failures_max = 50;

	params.comm_period_lowpass_alpha_reciprocal = 10;
	params.max_acceleration_per_step_x64 = 64 / 4;

	params.comm_period_zc_threshold_first  = erpm_to_comm_period(50000);
	params.comm_period_zc_threshold_second = erpm_to_comm_period(80000);

	params.comm_period_min = 50 * HNSEC_PER_USEC;
	params.comm_period_max = motor_timer_get_max_delay_hnsec();
}

static inline uint32_t erpm_to_comm_period(uint32_t erpm)
{
	const uint64_t hnsec_per_rev = HNSEC_PER_MINUTE / erpm;
	return hnsec_per_rev / NUM_COMMUTATION_STEPS;
}

static inline uint32_t comm_period_to_erpm(uint32_t comm_period)
{
	if (comm_period == 0)
		return 0;
	const uint64_t hnsec_per_rev = comm_period * NUM_COMMUTATION_STEPS;
	return HNSEC_PER_MINUTE / hnsec_per_rev;
}

int motor_init(void)
{
	configure();

	motor_pwm_init();
	motor_timer_init();
	motor_adc_init();

	motor_stop();

	return 0;
}

void motor_start(bool reverse)
{
	motor_stop();                          // Just in case

	memset(&state, 0, sizeof(state));      // Mighty reset

	state.control_state = CS_BEFORE_ZC;
	state.reverse_rotation = reverse;

	state.zc_threshold_shift = ZC_THRESHOLD_SHIFT_BASE + 2;
	state.comm_period = erpm_to_comm_period(params.spinup_erpm);
	state.blank_time_deadline = motor_timer_hnsec() + ((uint64_t)state.comm_period) * params.spinup_steps;

	motor_adc_enable(true);
	motor_timer_set_relative(0);           // Go from here
}

void motor_stop(void)
{
	state.control_state = CS_IDLE;
	motor_timer_cancel();
	motor_pwm_set_freewheeling();
	motor_adc_enable(false);
}

uint16_t motor_set_duty_cycle(uint16_t duty_cycle)
{
	struct motor_pwm_val val;
	const uint16_t true_duty_cycle = motor_pwm_compute_pwm_val(duty_cycle, &val);

	irq_primask_disable();
	state.pwm_val = val;
	irq_primask_enable();

	return true_duty_cycle;
}

bool motor_is_started(void)
{
	return state.control_state != CS_IDLE;
}

void motor_beep(int frequency, int duration_msec)
{
	if (state.control_state == CS_IDLE)
		motor_pwm_beep(frequency, duration_msec);
}

uint32_t motor_get_electrical_rpm(void)
{
	irq_primask_disable();
	const uint32_t val = state.comm_period;
	irq_primask_enable();
	return comm_period_to_erpm(val);
}

uint64_t motor_get_zc_failures_since_start(void)
{
	irq_primask_disable();
	const uint64_t ret = state.zc_failures_since_start;
	irq_primask_enable();
	return ret;
}

int motor_test_hardware(void)
{
	if (state.control_state != CS_IDLE)
		return -1;
	motor_adc_enable(true);
	const int res = motor_test_test_power_stage();
	motor_adc_enable(false);
	return res;
}

int motor_test_motor(void)
{
	if (state.control_state != CS_IDLE)
		return -1;
	motor_adc_enable(true);
	const int res = motor_test_test_motor();  // REDRUM
	motor_adc_enable(false);
	return res;
}
