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
#include <limits.h>
#include "motor.h"
#include "common.h"
#include "adc.h"
#include "pwm.h"
#include "timer.h"
#include "test.h"

static uint32_t erpm_to_comm_period(uint32_t erpm);

#define HNSEC_PER_MINUTE      (HNSEC_PER_SEC * 60)
#define NUM_PHASES            3
#define NUM_COMMUTATION_STEPS 6


#define INVALID_ADC_SAMPLE_VAL     INT_MIN

/**
 * Computes the timing advance in comm_period units
 */
#define TIMING_ADVANCE(comm_period, degrees) \
	(((uint64_t)comm_period * (uint64_t)degrees) / 64/*60*/)


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
	uint32_t comm_period;

	int current_comm_step;
	bool reverse_rotation;

	bool spinup_done;

	unsigned int immediate_zc_failures;
	unsigned int immediate_zc_detects;
	uint64_t zc_failures_since_start;
	uint64_t zc_detects_since_start;

	int prev_bemf_sample;
	int neutral_voltage;

	struct motor_pwm_val pwm_val;
	struct motor_pwm_val pwm_val_after_spinup;

	uint64_t started_at;
} _state;

static struct precomputed_params
{
	int comm_blank_hnsec;
	int timing_advance_deg;
	unsigned int zc_failures_max;
	unsigned int zc_detects_min;

	uint32_t comm_period_lowpass_base;
//	int comm_period_lowpass_alpha_reciprocal;     // Reciprocal of lowpass alpha (0; 1]
	int neutral_voltage_lowpass_alpha_reciprocal; // Ditto

	uint32_t comm_period_max;
} _params;

static void configure(void) // TODO: obtain the configuration from somewhere else
{
	_params.comm_blank_hnsec = 30 * HNSEC_PER_USEC;
	_params.timing_advance_deg = 10;
	_params.zc_failures_max = 50;
	_params.zc_detects_min = 50;

	_params.comm_period_lowpass_base = 5000 * HNSEC_PER_USEC;
//	_params.comm_period_lowpass_alpha_reciprocal = 10;
	_params.neutral_voltage_lowpass_alpha_reciprocal = 2;

	_params.comm_period_max = erpm_to_comm_period(1000);
	if (_params.comm_period_max > motor_timer_get_max_delay_hnsec())
		_params.comm_period_max = motor_timer_get_max_delay_hnsec();
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

__attribute__((optimize(3), always_inline))
static inline void switch_commutation_step(void)
{
	// We need to keep the current step index
	if (_state.reverse_rotation) {
		_state.current_comm_step--;
		if (_state.current_comm_step < 0)
			_state.current_comm_step = NUM_COMMUTATION_STEPS - 1;

	} else {
		_state.current_comm_step++;
		if (_state.current_comm_step >= NUM_COMMUTATION_STEPS)
			_state.current_comm_step = 0;
	}
	motor_pwm_set_step_from_isr(COMMUTATION_TABLE + _state.current_comm_step, &_state.pwm_val);
}

static inline void stop_from_isr(void)
{
	_state.control_state = CS_IDLE;
	motor_timer_cancel();
	motor_pwm_set_freewheeling();
}

__attribute__((optimize(3)))
void motor_timer_callback(uint64_t timestamp_hnsec)
{
	if (_state.control_state == CS_IDLE)
		return;

	motor_timer_set_relative(_state.comm_period);
	switch_commutation_step();
	//From this moment we have at least half of the commutation period before the next time critical event.

	if (_state.control_state == CS_BEFORE_ZC) {  // ZC has timed out
		// In this case we need to emulate the ZC detection
		const uint32_t leeway =
			_state.comm_period / 2 + TIMING_ADVANCE(_state.comm_period, _params.timing_advance_deg);
		_state.prev_zc_timestamp = timestamp_hnsec - leeway;

		_state.zc_failures_since_start++;

		// Stall detection
		_state.immediate_zc_detects = 0;
		_state.immediate_zc_failures++;
		if (_state.immediate_zc_failures > _params.zc_failures_max) {
			// No bounce no play
			stop_from_isr();
			return;
		}

		// Slow down a bit
		if (_state.spinup_done) {
			_state.comm_period = _state.comm_period + _state.comm_period / 8; // TODO: make configurable
			if (_state.comm_period > _params.comm_period_max)
				_state.comm_period = _params.comm_period_max;
		}
	} else { // On successful ZC
		_state.control_state = CS_BEFORE_ZC; // Waiting for the next ZC
		_state.zc_detects_since_start++;

		// Stall detection update
		if (_state.immediate_zc_failures) {
			_state.immediate_zc_detects++;
			if (_state.immediate_zc_detects > NUM_COMMUTATION_STEPS)
				_state.immediate_zc_failures = 0;
		}
	}

	// Enable ADC sampling
	motor_adc_enable_from_isr();

	_state.prev_bemf_sample = INVALID_ADC_SAMPLE_VAL;    // Discard the previous step sample
	_state.blank_time_deadline = timestamp_hnsec + _params.comm_blank_hnsec;

	// Are we reached the stable operation mode
	if (!_state.spinup_done) {
		if (_state.immediate_zc_failures == 0 && _state.zc_detects_since_start > _params.zc_detects_min) {
			_state.spinup_done = true;
			_state.pwm_val = _state.pwm_val_after_spinup;  // Engage normal duty cycle
		}
	}
}

__attribute__((optimize(3), always_inline))
static void handle_zero_crossing(uint64_t current_timestamp, uint64_t zc_timestamp)
{
	if (zc_timestamp < _state.prev_zc_timestamp)
		zc_timestamp = _state.prev_zc_timestamp;
	uint32_t new_comm_period = zc_timestamp - _state.prev_zc_timestamp;
	_state.prev_zc_timestamp = zc_timestamp;

	if (new_comm_period > _params.comm_period_max)
	        new_comm_period = _params.comm_period_max;

	const unsigned comm_period_base = (new_comm_period + _state.comm_period) / 2;
	unsigned comm_period_lowpass = _params.comm_period_lowpass_base / comm_period_base;
	if (comm_period_lowpass > 200)
		comm_period_lowpass = 200;
	_state.comm_period = (_state.comm_period * comm_period_lowpass + new_comm_period) / (comm_period_lowpass + 1);

	_state.control_state = CS_PAST_ZC;

	const uint32_t advance =
		_state.comm_period / 2 - TIMING_ADVANCE(_state.comm_period, _params.timing_advance_deg);

	// Override the comm period deadline that was set at the last commutation switching
	uint64_t deadline = zc_timestamp + advance;
	if (deadline < current_timestamp)
		deadline = current_timestamp;
	motor_timer_set_relative(deadline - current_timestamp);

	// Disable till next comm period
	motor_adc_disable_from_isr();
}

__attribute__((optimize(3)))
void motor_adc_sample_callback(const struct motor_adc_sample* sample)
{
	if (_state.control_state != CS_BEFORE_ZC || sample->timestamp < _state.blank_time_deadline)
		return;

	/*
	 * Neutral voltage low pass filtering
	 */
	const struct motor_pwm_commutation_step* const step = COMMUTATION_TABLE + _state.current_comm_step;
	const int avg_voltage =
		(sample->raw_phase_values[step->positive] + sample->raw_phase_values[step->negative]) / 2;
	_state.neutral_voltage =
		(_state.neutral_voltage * _params.neutral_voltage_lowpass_alpha_reciprocal + avg_voltage) /
		(_params.neutral_voltage_lowpass_alpha_reciprocal + 1);

	/*
	 * Normalized Back EMF voltage
	 */
	const int current_bemf = sample->raw_phase_values[step->floating] - _state.neutral_voltage;

	if (_state.prev_bemf_sample == INVALID_ADC_SAMPLE_VAL) {
		_state.prev_bemf_sample = current_bemf;
		return;
	}
	const int prev_bemf = _state.prev_bemf_sample;
	_state.prev_bemf_sample = current_bemf;

	/*
	 * Zero cross detection
	 */
	const bool zc_polarity = _state.reverse_rotation
		? !(_state.current_comm_step & 1) : (_state.current_comm_step & 1);
	const bool zc_occurred = (zc_polarity && (current_bemf >= 0)) || (!zc_polarity && (current_bemf <= 0));

	if (!zc_occurred)
		return;

	/*
	 * Zero cross interpolation
	 * References:
	 *  - "3-phase Sensorless BLDC Motor Control Development Kit with MC9S12G128 MCU"
	 *  - "BLDC Sensorless Reference Design Using MC56F8006"
	 *
	 * V1, V2 - voltage readings
	 * t1, t2 - their timestamps
	 * tz     - ZC timestamp
	 *
	 * dt = t2 - t1
	 * dV = V2 - V1
	 * tz = t1 + abs((V1 * dt) / dV)
	 */
	const int dt = MOTOR_ADC_SAMPLING_PERIOD_HNSEC;
	const int dv = current_bemf - prev_bemf;
	uint64_t zc_timestamp = 0;
	if (abs(dv) < 2) {
		zc_timestamp = sample->timestamp;
	} else {
		const int t_offset = abs((prev_bemf * dt) / dv);

		// Invalid offset means that we missed this ZC, and most likely the prev_bemf_sample is invalid
		if (t_offset < 0 || t_offset > dt * 2) {
			_state.prev_bemf_sample = INVALID_ADC_SAMPLE_VAL;
			return;
		}

		zc_timestamp = sample->timestamp - dt + (uint64_t)t_offset;
	}
	if (step->floating == 0)
		TESTPAD_SET(GPIO_PORT_TEST_MZC, GPIO_PIN_TEST_MZC);
	handle_zero_crossing(sample->timestamp, zc_timestamp);
	TESTPAD_CLEAR(GPIO_PORT_TEST_MZC, GPIO_PIN_TEST_MZC);
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

void motor_start(float spinup_duty_cycle, float normal_duty_cycle, bool reverse)
{
	assert(spinup_duty_cycle >= 0);
	assert(normal_duty_cycle >= 0);

	motor_stop();                          // Just in case

	if (spinup_duty_cycle <= 0 || normal_duty_cycle <= 0)
		return;

	memset(&_state, 0, sizeof(_state));    // Mighty reset

	motor_pwm_compute_pwm_val(spinup_duty_cycle, &_state.pwm_val);
	motor_pwm_compute_pwm_val(normal_duty_cycle, &_state.pwm_val_after_spinup);

	_state.reverse_rotation = reverse;
	_state.spinup_done = false;

	_state.comm_period = _params.comm_period_max;
	_state.blank_time_deadline = motor_timer_hnsec() + _params.comm_blank_hnsec;

	_state.started_at = motor_timer_hnsec();

	// TODO: initialize proper average during the initial rotor alignment
	const struct motor_adc_sample adc_sample = motor_adc_get_last_sample();
	_state.neutral_voltage = (adc_sample.raw_phase_values[0] +
		adc_sample.raw_phase_values[1] + adc_sample.raw_phase_values[2]) / 3;

	_state.prev_bemf_sample = INVALID_ADC_SAMPLE_VAL;

	_state.control_state = CS_BEFORE_ZC;
	motor_timer_set_relative(0);           // Go from here
}

void motor_stop(void)
{
	irq_primask_disable();

	_state.control_state = CS_IDLE;
	// ADC should be enabled by default
	motor_adc_enable_from_isr();

	irq_primask_enable();

	motor_timer_cancel();
	motor_pwm_set_freewheeling();
}

void motor_set_duty_cycle(float duty_cycle)
{
	struct motor_pwm_val val;
	motor_pwm_compute_pwm_val(duty_cycle, &val);

	irq_primask_disable();
	_state.pwm_val = val;
	irq_primask_enable();
}

enum motor_state motor_get_state(void)
{
	irq_primask_disable();
	const enum control_state_id csid = _state.control_state;
	const bool spinup_done = _state.spinup_done;
	irq_primask_enable();

	if (csid == CS_IDLE)
		return MOTOR_STATE_IDLE;
	return spinup_done ? MOTOR_STATE_RUNNING : MOTOR_STATE_STARTING;
}

void motor_beep(int frequency, int duration_msec)
{
	if (_state.control_state == CS_IDLE) {
		irq_primask_disable();
		motor_adc_disable_from_isr();
		irq_primask_enable();

		motor_pwm_beep(frequency, duration_msec);

		irq_primask_disable();
		motor_adc_enable_from_isr();
		irq_primask_enable();
	}
}

uint32_t motor_get_electrical_rpm(void)
{
	irq_primask_disable();
	const uint32_t val = _state.comm_period;
	irq_primask_enable();
	return comm_period_to_erpm(val);
}

uint32_t motor_get_comm_period_hnsec(void)
{
	if (motor_get_state() == MOTOR_STATE_IDLE)
		return 0;
	irq_primask_disable();
	const uint32_t val = _state.comm_period;
	irq_primask_enable();
	return val;
}

uint64_t motor_get_zc_failures_since_start(void)
{
	irq_primask_disable();
	const uint64_t ret = _state.zc_failures_since_start;
	irq_primask_enable();
	return ret;
}

int motor_test_hardware(void)
{
	if (_state.control_state != CS_IDLE)
		return -1;
	return motor_test_test_power_stage();
}

int motor_test_motor(void)
{
	if (_state.control_state != CS_IDLE)
		return -1;
	return motor_test_test_motor();  // REDRUM
}

void motor_emergency(void)
{
	const irqstate_t irqstate = irq_primask_save();
	motor_pwm_emergency();
	_state.control_state = CS_IDLE;
	irq_primask_restore(irqstate);
}

void motor_print_debug_info(void)
{
	irq_primask_disable();
	const struct control_state state_copy = _state;
	irq_primask_enable();

	lowsyslog("Motor: Debug\n"
		"  comm period        %u usec\n"
		"  erpm               %u RPM\n"
		"  neutral voltage    %i\n"
		"  spinup done        %i\n"
		"  zc immed failures  %u\n"
		"  zc immed detects   %u\n"
		"  zc failures        %u\n"
		"  zc detects         %u\n",
		(unsigned)(state_copy.comm_period / HNSEC_PER_USEC),
		(unsigned)(comm_period_to_erpm(state_copy.comm_period)),
		(int)state_copy.neutral_voltage,
		(int)state_copy.spinup_done,
		(unsigned)state_copy.immediate_zc_failures,
		(unsigned)state_copy.immediate_zc_detects,
		(unsigned)state_copy.zc_failures_since_start,
		(unsigned)state_copy.zc_detects_since_start);
}
