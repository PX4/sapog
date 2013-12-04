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

#include <unistd.h>
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

#define NUM_PHASES            3
#define NUM_COMMUTATION_STEPS 6

#define COMM_PERIOD_LOWPASS_MAX    100

#define INVALID_ADC_SAMPLE_VAL     INT_MIN

/**
 * Computes the timing advance in comm_period units
 */
#define TIMING_ADVANCE(comm_period, degrees) \
	(((uint64_t)comm_period * (uint64_t)degrees) / 64/*60*/)


/**
 * Commutation tables
 * Phase order: Positive, Negative, Floating
 */
static const struct motor_pwm_commutation_step COMMUTATION_TABLE_FORWARD[NUM_COMMUTATION_STEPS] = {
	{1, 0, 2},
	{1, 2, 0},
	{0, 2, 1},
	{0, 1, 2},
	{2, 1, 0},
	{2, 0, 1}
};
static const struct motor_pwm_commutation_step COMMUTATION_TABLE_REVERSE[NUM_COMMUTATION_STEPS] = {
	{2, 0, 1},
	{2, 1, 0},
	{0, 1, 2},
	{0, 2, 1},
	{1, 2, 0},
	{1, 0, 2}
};


enum control_state_id { CS_IDLE, CS_BEFORE_ZC, CS_PAST_ZC };

static struct control_state
{
	enum control_state_id control_state;

	uint64_t blank_time_deadline;
	uint64_t prev_zc_timestamp;
	uint32_t comm_period;

	int current_comm_step;
	const struct motor_pwm_commutation_step* comm_table;

	bool spinup_done;

	unsigned int immediate_zc_failures;
	unsigned int immediate_zc_detects;
	uint64_t zc_failures_since_start;
	uint64_t zc_detects_since_start;

	int prev_bemf_sample;
	int neutral_voltage;

	int input_voltage;
	int input_current;

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

	int comm_period_shift_on_zc_failure;
	uint32_t comm_period_lowpass_base;
	int neutral_voltage_lowpass_alpha_reciprocal;

	uint32_t spinup_alignment_hnsec;
	uint32_t spinup_comm_period_begin;
	uint32_t spinup_comm_period_end;
	int spinup_num_steps;

	uint32_t comm_period_max;

	uint32_t adc_sampling_period;

	int input_volt_cur_lowpass_alpha_reciprocal;
} _params;

static void configure(void) // TODO: obtain the configuration from somewhere else
{
	_params.comm_blank_hnsec = 30 * HNSEC_PER_USEC;
	_params.timing_advance_deg = 10;
	_params.zc_failures_max = 30;
	_params.zc_detects_min = 50;

	_params.comm_period_shift_on_zc_failure = 1;
	_params.comm_period_lowpass_base = 5000 * HNSEC_PER_USEC;
	_params.neutral_voltage_lowpass_alpha_reciprocal = 2;

	_params.spinup_alignment_hnsec   = 40000 * HNSEC_PER_USEC;
	_params.spinup_comm_period_begin = 20000 * HNSEC_PER_USEC;
	_params.spinup_comm_period_end   = 10000 * HNSEC_PER_USEC;
	_params.spinup_num_steps = 2;

	_params.comm_period_max = motor_timer_get_max_delay_hnsec();
	if (_params.comm_period_max > motor_timer_get_max_delay_hnsec())
		_params.comm_period_max = motor_timer_get_max_delay_hnsec();

	if (_params.spinup_comm_period_end > _params.comm_period_max)
		_params.spinup_comm_period_end = _params.comm_period_max;

	if (_params.spinup_comm_period_begin < _params.spinup_comm_period_end)
		_params.spinup_comm_period_begin = _params.spinup_comm_period_end;

	_params.adc_sampling_period = motor_adc_sampling_period_hnsec();

	_params.input_volt_cur_lowpass_alpha_reciprocal = 100;
}

__attribute__((optimize(3), always_inline))
static inline void switch_commutation_step(void)
{
	// We need to keep the current step index
	_state.current_comm_step++;
	if (_state.current_comm_step >= NUM_COMMUTATION_STEPS)
		_state.current_comm_step = 0;
	assert(_state.comm_table);
	motor_pwm_set_step_from_isr(_state.comm_table + _state.current_comm_step, &_state.pwm_val);
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

	uint32_t comm_period_on_zc_failure =
		_state.comm_period + (_state.comm_period >> _params.comm_period_shift_on_zc_failure);

	if (comm_period_on_zc_failure > _params.comm_period_max)
		comm_period_on_zc_failure = _params.comm_period_max;

	motor_timer_set_relative(comm_period_on_zc_failure);
	switch_commutation_step();
	//From this moment we have at least half of the commutation period before the next time critical event.

	if (_state.control_state == CS_BEFORE_ZC) {  // ZC has timed out
		// In this case we need to emulate the ZC detection
		const uint32_t leeway = comm_period_on_zc_failure / 2 +
			TIMING_ADVANCE(comm_period_on_zc_failure, _params.timing_advance_deg);
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
		if (_state.spinup_done)
			_state.comm_period = comm_period_on_zc_failure;
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
	if (comm_period_lowpass > COMM_PERIOD_LOWPASS_MAX)
		comm_period_lowpass = COMM_PERIOD_LOWPASS_MAX;
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

__attribute__((optimize(3), always_inline))
static void update_input_voltage_current(const struct motor_adc_sample* sample)
{
	const int alpha = _params.input_volt_cur_lowpass_alpha_reciprocal;
	_state.input_voltage = (_state.input_voltage * alpha + sample->input_voltage) / (alpha + 1);
	_state.input_current = (_state.input_current * alpha + sample->input_current) / (alpha + 1);
}

__attribute__((optimize(3)))
void motor_adc_sample_callback(const struct motor_adc_sample* sample)
{
	if (_state.control_state != CS_BEFORE_ZC || sample->timestamp < _state.blank_time_deadline)
		return;

	/*
	 * Neutral voltage low pass filtering
	 */
	assert(_state.comm_table);
	const struct motor_pwm_commutation_step* const step = _state.comm_table + _state.current_comm_step;
	const int avg_voltage = (sample->phase_values[step->positive] + sample->phase_values[step->negative]) / 2;
	_state.neutral_voltage =
		(_state.neutral_voltage * _params.neutral_voltage_lowpass_alpha_reciprocal + avg_voltage) /
		(_params.neutral_voltage_lowpass_alpha_reciprocal + 1);

	/*
	 * Normalized Back EMF voltage
	 */
	const int current_bemf = sample->phase_values[step->floating] - _state.neutral_voltage;

	if (_state.prev_bemf_sample == INVALID_ADC_SAMPLE_VAL) {
		_state.prev_bemf_sample = current_bemf;
		return;
	}
	const int prev_bemf = _state.prev_bemf_sample;
	_state.prev_bemf_sample = current_bemf;

	/*
	 * Zero cross detection
	 */
	const bool zc_polarity = _state.current_comm_step & 1;
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
	const int dt = _params.adc_sampling_period;
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

	/*
	 * Zero cross processing
	 */
	if (step->floating == 0)
		TESTPAD_SET(GPIO_PORT_TEST_MZC, GPIO_PIN_TEST_MZC);

	handle_zero_crossing(sample->timestamp, zc_timestamp);
	update_input_voltage_current(sample);

	TESTPAD_CLEAR(GPIO_PORT_TEST_MZC, GPIO_PIN_TEST_MZC);
}

int motor_init(void)
{
	int ret = motor_pwm_init(30000); // TODO: configuration
	if (ret)
		return ret;

	motor_timer_init();

	ret = motor_adc_init(0.0005);
	if (ret)
		return ret;

	configure();
	motor_stop();
	return 0;
}

static void init_adc_filters(void)
{
	struct motor_adc_sample smpl;
	enum motor_pwm_phase_manip manip_cmd[3];

	// Low phase
	for (int i = 0 ; i < NUM_PHASES; i++)
		manip_cmd[i] = MOTOR_PWM_MANIP_LOW;
	motor_pwm_manip(manip_cmd);
	smpl = motor_adc_get_last_sample();
	const int low = (smpl.phase_values[0] + smpl.phase_values[1] + smpl.phase_values[2]) / 3;

	// High phase
	for (int i = 0 ; i < NUM_PHASES; i++)
		manip_cmd[i] = MOTOR_PWM_MANIP_HIGH;
	motor_pwm_manip(manip_cmd);
	smpl = motor_adc_get_last_sample();
	const int high = (smpl.phase_values[0] + smpl.phase_values[1] + smpl.phase_values[2]) / 3;

	// Phase neutral
	motor_pwm_set_freewheeling();
	_state.neutral_voltage = (low + high) / 2;

	// Supply voltage and current
	_state.input_voltage = smpl.input_voltage;
	_state.input_current = smpl.input_current;
}

static void spinup_align(void)
{
	assert(_state.comm_table);
	const struct motor_pwm_commutation_step* const first_step = _state.comm_table + _state.current_comm_step;

	int polarities[3];
	polarities[first_step->negative] = 0; // 0 - low, 1 - high, -1 - floating
	polarities[first_step->positive] = 1;
	polarities[first_step->floating] = -1;

	motor_pwm_align(polarities, &_state.pwm_val);

	//motor_timer_hndelay(_params.spinup_alignment_hnsec);
	usleep(_params.spinup_alignment_hnsec / HNSEC_PER_USEC);
}

static void spinup_do_blind_comms(void)
{
	if (_params.spinup_num_steps <= 0)
		return;

	assert_always(_params.spinup_comm_period_begin >= _params.spinup_comm_period_end);
	const uint32_t accel =
		(_params.spinup_comm_period_begin - _params.spinup_comm_period_end) / _params.spinup_num_steps;

	uint32_t current_comm_period = _params.spinup_comm_period_begin;
	for (int i = 0; i < _params.spinup_num_steps; i++) {
		switch_commutation_step();
		//motor_timer_hndelay(current_comm_period);
		usleep(current_comm_period / HNSEC_PER_USEC);
		current_comm_period -= accel;
	}
}

void motor_start(float spinup_duty_cycle, float normal_duty_cycle, bool reverse)
{
	assert(spinup_duty_cycle >= 0);
	assert(normal_duty_cycle >= 0);

	motor_stop();                          // Just in case

	if (spinup_duty_cycle <= 0 || normal_duty_cycle <= 0)
		return;

	/*
	 * Initialize the control struct
	 */
	memset(&_state, 0, sizeof(_state));    // Mighty reset

	_state.comm_table = reverse ? COMMUTATION_TABLE_REVERSE : COMMUTATION_TABLE_FORWARD;
	_state.spinup_done = false;
	_state.started_at = motor_timer_hnsec();
	_state.current_comm_step = 0;

	motor_pwm_compute_pwm_val(spinup_duty_cycle, &_state.pwm_val);
	motor_pwm_compute_pwm_val(normal_duty_cycle, &_state.pwm_val_after_spinup);

	init_adc_filters();

	/*
	 * Align the rotor and perform blind commutations
	 */
	irq_primask_disable();
	motor_adc_disable_from_isr();
	irq_primask_enable();

	spinup_align();

	spinup_do_blind_comms();

	irq_primask_disable();
	motor_adc_enable_from_isr();
	irq_primask_enable();

	/*
	 * Engage the closed-loop mode
	 */
	_state.blank_time_deadline = motor_timer_hnsec() + _params.comm_blank_hnsec;
	_state.comm_period = _params.spinup_comm_period_end;
	_state.prev_bemf_sample = INVALID_ADC_SAMPLE_VAL;
	_state.control_state = CS_BEFORE_ZC;

	motor_timer_set_relative(0);
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

void motor_get_input_voltage_current(float* out_voltage, float* out_current)
{
	int volt = 0, curr = 0;

	if (motor_get_state() == MOTOR_STATE_IDLE) {
		const struct motor_adc_sample smpl = motor_adc_get_last_sample();
		volt = smpl.input_voltage;
		curr = smpl.input_current;
	} else {
		irq_primask_disable();
		volt = _state.input_voltage;
		curr = _state.input_current;
		irq_primask_enable();
	}

	if (out_voltage)
		*out_voltage = motor_adc_convert_input_voltage(volt);
	if (out_current)
		*out_current = motor_adc_convert_input_current(curr);
}

uint32_t motor_get_limit_comm_period_hnsec(void)
{
	// Ensure some number of ADC samples per comm period
	return motor_adc_sampling_period_hnsec() * 5;
}

void motor_print_debug_info(void)
{
	irq_primask_disable();
	const struct control_state state_copy = _state;
	irq_primask_enable();

	lowsyslog("Motor: Debug\n"
		"  comm period        %u usec\n"
		"  neutral voltage    %i\n"
		"  spinup done        %i\n"
		"  zc immed failures  %u\n"
		"  zc immed detects   %u\n"
		"  zc failures        %u\n"
		"  zc detects         %u\n"
		"  voltage, current   %i %i\n",
		(unsigned)(state_copy.comm_period / HNSEC_PER_USEC),
		(int)state_copy.neutral_voltage,
		(int)state_copy.spinup_done,
		(unsigned)state_copy.immediate_zc_failures,
		(unsigned)state_copy.immediate_zc_detects,
		(unsigned)state_copy.zc_failures_since_start,
		(unsigned)state_copy.zc_detects_since_start,
		state_copy.input_voltage, state_copy.input_current);
}
