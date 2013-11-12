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

#include <stdlib.h>
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
 * Computes the timing advance in comm_period units
 */
#define TIMING_ADVANCE(comm_period, degrees) \
	(((uint64_t)comm_period * (uint64_t)degrees) / 64/*60*/)

/**
 * Integer low pass filter
 */
#define LOWPASS(old_val, new_val, alpha) \
	(((old_val) * (alpha) + (new_val)) / ((alpha) + 1))


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

	bool spinup_done;

	unsigned int immediate_zc_failures;
	unsigned int immediate_zc_detects;
	uint64_t zc_failures_since_start;
	uint64_t zc_detects_since_start;

	int prev_adc_normalized_sample;
	int phase_avg_volt[3];

	struct motor_pwm_val pwm_val;

	uint64_t started_at;
} state;

static struct precomputed_params
{
	int comm_blank_hnsec;
	int timing_advance_deg;
	unsigned int zc_failures_max;
	unsigned int zc_detects_min;

	int comm_period_shift_to_voltage_lowpass_alpha_reciprocal;
	int comm_period_lowpass_alpha_reciprocal;   // Reciprocal of lowpass alpha (0; 1]
	int max_acceleration_per_step_64;           // Like percent but in range [0; 64] for faster division

	uint32_t comm_period_min;
	uint32_t comm_period_max;
} params;

static void configure(void) // TODO: obtain the configuration from somewhere else
{
	params.comm_blank_hnsec = 30 * HNSEC_PER_USEC;
	params.timing_advance_deg = 10;
	params.zc_failures_max = 50;
	params.zc_detects_min = 50;

	params.comm_period_shift_to_voltage_lowpass_alpha_reciprocal = 11; // 11 --> 2048 hnsec, 204.8 usec
	params.comm_period_lowpass_alpha_reciprocal = 10;
	params.max_acceleration_per_step_64 = 64 / 4;

	params.comm_period_min = 50 * HNSEC_PER_USEC;
	params.comm_period_max = erpm_to_comm_period(1000);//motor_timer_get_max_delay_hnsec();
}

static inline uint32_t erpm_to_comm_period(uint32_t erpm)
{
	// erpm_to_comm_period = lambda erpm: ((10000000 * 60) / erpm) / 6
	const uint64_t hnsec_per_rev = HNSEC_PER_MINUTE / erpm;
	return hnsec_per_rev / NUM_COMMUTATION_STEPS;
}

static inline uint32_t comm_period_to_erpm(uint32_t comm_period)
{
	// comm_period_to_erpm = lambda cp: (10000000 * 60) / (cp * 6)
	if (comm_period == 0)
		return 0;
	const uint64_t hnsec_per_rev = comm_period * NUM_COMMUTATION_STEPS;
	return HNSEC_PER_MINUTE / hnsec_per_rev;
}

static inline void switch_commutation_step(void)
{
	// We need to keep the current step index
	if (state.reverse_rotation) {
		state.current_comm_step--;
		if (state.current_comm_step < 0)
			state.current_comm_step = NUM_COMMUTATION_STEPS - 1;

	} else {
		state.current_comm_step++;
		if (state.current_comm_step >= NUM_COMMUTATION_STEPS)
			state.current_comm_step = 0;
	}
	motor_pwm_set_step_from_isr(COMMUTATION_TABLE + state.current_comm_step, &state.pwm_val);
}

static inline void stop_from_isr(void)
{
	state.control_state = CS_IDLE;
	motor_timer_cancel();
	motor_pwm_set_freewheeling();
}

void motor_timer_callback(void)
{
	const uint64_t timestamp = motor_timer_hnsec();
	if (state.control_state == CS_IDLE)
		return;

	motor_timer_set_relative(state.comm_period);
	switch_commutation_step();
	//From this moment we have at least half of the commutation period before the next time critical event.

	if (state.control_state == CS_BEFORE_ZC) {  // ZC has timed out
		// In this case we need to emulate the ZC detection
		const uint32_t leeway =
			state.comm_period / 2 + TIMING_ADVANCE(state.comm_period, params.timing_advance_deg);
		state.prev_zc_timestamp = timestamp - leeway;

		state.zc_failures_since_start++;

		// Stall detection
		state.immediate_zc_detects = 0;
		state.immediate_zc_failures++;
		if (state.immediate_zc_failures > params.zc_failures_max) {
			// No bounce no play
			stop_from_isr();
			return;
		}

		// Slow down a bit
		if (state.spinup_done) {
			state.comm_period = state.comm_period + state.comm_period / 8; // TODO: make configurable
			if (state.comm_period > params.comm_period_max)
				state.comm_period = params.comm_period_max;
		}
	} else { // On successful ZC
		state.control_state = CS_BEFORE_ZC; // Waiting for the next ZC
		state.zc_detects_since_start++;

		// Stall detection update
		if (state.immediate_zc_failures) {
			state.immediate_zc_detects++;
			if (state.immediate_zc_detects > NUM_COMMUTATION_STEPS)
				state.immediate_zc_failures = 0;
		}
	}

	state.prev_adc_sample_timestamp = 0;        // Discard the previous step sample
	state.blank_time_deadline = timestamp + params.comm_blank_hnsec;
	state.predicted_zc_timestamp =
		timestamp + ((uint64_t)state.comm_period * (uint64_t)params.max_acceleration_per_step_64) / 64;

	// Are we reached the stable operation mode
	if (!state.spinup_done) {
		if (state.immediate_zc_failures == 0 && state.zc_detects_since_start > params.zc_detects_min)
			state.spinup_done = true;
	}
}

static void handle_zero_crossing(uint64_t current_timestamp, uint64_t zc_timestamp)
{
	if (zc_timestamp < state.prev_zc_timestamp)
		zc_timestamp = state.prev_zc_timestamp;
	uint32_t new_comm_period = zc_timestamp - state.prev_zc_timestamp;
	state.prev_zc_timestamp = zc_timestamp;

	if (new_comm_period > params.comm_period_max)
	        new_comm_period = params.comm_period_max;

	state.comm_period = LOWPASS(state.comm_period, new_comm_period, params.comm_period_lowpass_alpha_reciprocal);
	state.control_state = CS_PAST_ZC;

	const uint32_t advance = state.comm_period / 2 - TIMING_ADVANCE(state.comm_period, params.timing_advance_deg);

	// Override the comm period deadline that was set at the last commutation switching
	uint64_t deadline = zc_timestamp + advance;
	if (deadline < current_timestamp)
		deadline = current_timestamp;
	motor_timer_set_relative(deadline - current_timestamp);
}

void motor_adc_sample_callback(const struct motor_adc_sample* sample)
{
	if ((state.control_state != CS_IDLE) && (sample->timestamp > state.blank_time_deadline)) {
		/*
		 * TODO: Derive this from amount of samples per commutation?
		 */
		const int alpha = state.comm_period >> params.comm_period_shift_to_voltage_lowpass_alpha_reciprocal;

		if (alpha) {
			// I hope the compiler would be smart enough to unroll this
			for (int i = 0; i < 3; i++) {
				state.phase_avg_volt[i] =
					LOWPASS(state.phase_avg_volt[i], sample->raw_phase_values[i], alpha);
			}
		} else {
			for (int i = 0; i < 3; i++)
				state.phase_avg_volt[i] = sample->raw_phase_values[i];
		}
	}
	if (state.control_state != CS_BEFORE_ZC)
		return;

	const struct motor_pwm_commutation_step* const step = COMMUTATION_TABLE + state.current_comm_step;

	// Here we compute the floating phase voltage using neutral voltage as a reference
	const int neutral_voltage = (state.phase_avg_volt[step->positive] + state.phase_avg_volt[step->negative]) / 2;
	const int normalized_sample = state.phase_avg_volt[step->floating] - neutral_voltage;

	// Detect our position with respect to zero crossing
	const bool zc_polarity = state.reverse_rotation
		? !(state.current_comm_step & 1) : (state.current_comm_step & 1);
	const bool zc_occurred = (zc_polarity && (normalized_sample > 0)) || (!zc_polarity && (normalized_sample < 0));

	if (zc_occurred) {
		// Sanity check. On low RPM we have a lot of noise on the floating phase.
		if (sample->timestamp < state.predicted_zc_timestamp)
			return;

		uint64_t zc_timestamp = 0;
		if (state.prev_adc_sample_timestamp > 0) {
			/*
			 * Interpolate with previous ADC sample, in order to estimate ZC time precisely.
			 * V1, V2 - voltage readings
			 * t1, t2 - their timestamps
			 * tz     - ZC timestamp
			 *
			 * dt = t2 - t1
			 * dV = V2 - V1
			 * tz = t1 + abs((V1 * dt) / dV)
			 */
			const int dt = sample->timestamp - state.prev_adc_sample_timestamp;
			assert(dt > 0);
			const int dv = normalized_sample - state.prev_adc_normalized_sample;
			const int t_offset = abs((state.prev_adc_normalized_sample * dt) / dv);
			zc_timestamp = state.prev_adc_sample_timestamp + (uint64_t)t_offset;
		} else {
			/*
			 * We have no previous sample to interpolate with,
			 * so we hope that ZC happened exactly between two consequent ADC samples
			 */
			zc_timestamp = sample->timestamp - MOTOR_ADC_SAMPLING_PERIOD_HNSEC / 2;
		}
		handle_zero_crossing(sample->timestamp, zc_timestamp);
	} else {
		state.prev_adc_sample_timestamp = sample->timestamp;
		state.prev_adc_normalized_sample = normalized_sample;
	}
}

int motor_init(void)
{
	motor_pwm_init();
	motor_timer_init();
	motor_adc_init();
	configure();
	motor_stop();
	return 0;
}

void motor_start(uint16_t duty_cycle, bool reverse)
{
	motor_stop();                          // Just in case

	memset(&state, 0, sizeof(state));      // Mighty reset

	motor_set_duty_cycle(duty_cycle);

	state.control_state = CS_BEFORE_ZC;
	state.reverse_rotation = reverse;
	state.spinup_done = false;

	state.comm_period = params.comm_period_max;
	state.blank_time_deadline = motor_timer_hnsec() + params.comm_blank_hnsec;

	state.started_at = motor_timer_hnsec();

	motor_timer_set_relative(0);           // Go from here
}

void motor_stop(void)
{
	irq_primask_disable();
	state.control_state = CS_IDLE;
	irq_primask_enable();
	motor_timer_cancel();
	motor_pwm_set_freewheeling();
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

enum motor_state motor_get_state(void)
{
	irq_primask_disable();
	const enum control_state_id csid = state.control_state;
	const bool spinup_done = state.spinup_done;
	irq_primask_enable();

	if (csid == CS_IDLE)
		return MOTOR_STATE_IDLE;
	return spinup_done ? MOTOR_STATE_RUNNING : MOTOR_STATE_STARTING;
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
	return motor_test_test_power_stage();
}

int motor_test_motor(void)
{
	if (state.control_state != CS_IDLE)
		return -1;
	return motor_test_test_motor();  // REDRUM
}

void motor_emergency(void)
{
	const irqstate_t irqstate = irq_primask_save();
	motor_pwm_emergency();
	state.control_state = CS_IDLE;
	irq_primask_restore(irqstate);
}

void motor_print_debug_info(void)
{
	irq_primask_disable();
	const struct control_state state_copy = state;
	irq_primask_enable();

	lowsyslog("Motor: Debug\n"
		"  comm period        %u usec\n"
		"  erpm               %u RPM\n"
		"  spinup done        %i\n"
		"  zc immed failures  %u\n"
		"  zc immed detects   %u\n"
		"  zc failures        %u\n"
		"  zc detects         %u\n"
		"  phase volt         %i %i %i\n",
		(unsigned)(state_copy.comm_period / HNSEC_PER_USEC),
		(unsigned)(comm_period_to_erpm(state_copy.comm_period)),
		(int)state_copy.spinup_done,
		(unsigned)state_copy.immediate_zc_failures,
		(unsigned)state_copy.immediate_zc_detects,
		(unsigned)state_copy.zc_failures_since_start,
		(unsigned)state_copy.zc_detects_since_start,
		state_copy.phase_avg_volt[0], state_copy.phase_avg_volt[1], state_copy.phase_avg_volt[2]);
}
