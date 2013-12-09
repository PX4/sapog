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
#include <config/config.h>
#include "motor.h"
#include "common.h"
#include "adc.h"
#include "pwm.h"
#include "timer.h"
#include "test.h"

#define NUM_PHASES                 3
#define NUM_COMMUTATION_STEPS      6

#define COMM_PERIOD_LOWPASS_MAX    50

/**
 * Upper limit is 8 for 72MHz Cortex M3 (limited by the processing power)
 * Least possible value is 2 (logic requirement)
 */
#define MAX_BEMF_SAMPLES           8

/**
 * Computes the timing advance in comm_period units
 */
#define TIMING_ADVANCE64(comm_period, degrees) \
	(((uint64_t)comm_period * (uint64_t)degrees) / 64/*60*/)

#define TESTPAD_ZC_SET()           TESTPAD_SET(GPIO_PORT_TEST_MZC, GPIO_PIN_TEST_MZC)
#define TESTPAD_ZC_CLEAR()         TESTPAD_CLEAR(GPIO_PORT_TEST_MZC, GPIO_PIN_TEST_MZC)

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


enum control_state_id { CS_IDLE, CS_BEFORE_ZC, CS_PAST_ZC, CS_FAILED_ZC };

// TODO: separate the statistics
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
	uint64_t zc_failed_solutions;
	int zc_samples_per_solution;
	int zc_prev_samples[MAX_BEMF_SAMPLES];
	int zc_prev_fitted_samples[MAX_BEMF_SAMPLES];
	int zc_slope;
	uint64_t bemf_samples_out_of_range;
	uint64_t bemf_samples_premature_zc;

	int bemf_samples[MAX_BEMF_SAMPLES];
	uint64_t bemf_timestamps[MAX_BEMF_SAMPLES];
	int optimal_num_bemf_samples;
	int num_bemf_samples_acquired;
	int num_bemf_samples_past_zc;

	int neutral_voltage;
	int integrated_bemf_voltage;

	int input_voltage;
	int input_current;

	struct motor_pwm_val pwm_val;
	struct motor_pwm_val pwm_val_after_spinup;

	uint64_t started_at;
} _state;

static struct precomputed_params
{
	uint32_t comm_period_lowpass_base;
	int comm_period_shift_on_zc_failure;
	int timing_advance_deg64;
	int neutral_voltage_lowpass_alpha_reciprocal;

	uint32_t spinup_alignment_hnsec;
	uint32_t spinup_comm_period_begin;
	uint32_t spinup_comm_period_end;
	int spinup_num_steps;

	int motor_bemf_window_len_denom;
	int bemf_valid_range_pct128;
	int integrated_bemf_threshold_pct128;
	unsigned int zc_failures_max;
	unsigned int zc_detects_min;
	uint32_t comm_period_max;
	int comm_blank_hnsec;
	int input_volt_cur_lowpass_alpha_reciprocal;

	uint32_t adc_sampling_period;
} _params;

static bool _initialization_confirmed = false;


CONFIG_PARAM_INT("motor_pwm_frequency",                30000, MOTOR_PWM_MIN_FREQUENCY, MOTOR_PWM_MAX_FREQUENCY)
CONFIG_PARAM_FLOAT("motor_current_shunt_mohm",         0.5,   0.001, 100.0)
// Most important parameters
CONFIG_PARAM_INT("motor_comm_period_lowpass_base_usec",10000, 0,     50000)
CONFIG_PARAM_INT("motor_deceleration_rate_on_zc_miss", 3,     0,     8)
CONFIG_PARAM_INT("motor_timing_advance_deg",           10,    0,     60)
CONFIG_PARAM_FLOAT("motor_neutral_volt_lowpass_alpha", 1.0,   1e-3,  1.0)
CONFIG_PARAM_INT("motor_comm_blank_usec",              40,    30,    100) // Correct blank time is vital for LSF approx
// Spinup settings
CONFIG_PARAM_INT("motor_spinup_alignment_usec",        300000,0,     900000)
CONFIG_PARAM_INT("motor_spinup_comm_period_begin_usec",20000, 10000, 90000)
CONFIG_PARAM_INT("motor_spinup_comm_period_end_usec",  16000, 10000, 60000)
CONFIG_PARAM_INT("motor_spinup_num_steps",             1,     0,     10)
// Something not so important
CONFIG_PARAM_INT("motor_bemf_window_pct",              25,    10,    70)
CONFIG_PARAM_INT("motor_bemf_valid_range_pct",         70,    10,    100)
CONFIG_PARAM_INT("motor_zc_integral_threshold_pct",    15,    0,     200)
CONFIG_PARAM_INT("motor_zc_failures_to_stop",          40,    1,     500)
CONFIG_PARAM_INT("motor_zc_detects_to_start",          100,   1,     1000)
CONFIG_PARAM_INT("motor_comm_period_max_usec",         20000, 1000,  100000)
CONFIG_PARAM_FLOAT("motor_volt_curr_lowpass_alpha",    0.02,  1e-3,  1.0)


static void configure(void)
{
	_params.comm_period_lowpass_base        = config_get("motor_comm_period_lowpass_base_usec") * HNSEC_PER_USEC;
	_params.comm_period_shift_on_zc_failure = config_get("motor_deceleration_rate_on_zc_miss");
	_params.timing_advance_deg64            = config_get("motor_timing_advance_deg") * 64 / 60;
	_params.neutral_voltage_lowpass_alpha_reciprocal =
		(int)(1.0f / config_get("motor_neutral_volt_lowpass_alpha") + 0.5f) - 1;

	_params.spinup_alignment_hnsec   = config_get("motor_spinup_alignment_usec")         * HNSEC_PER_USEC;
	_params.spinup_comm_period_begin = config_get("motor_spinup_comm_period_begin_usec") * HNSEC_PER_USEC;
	_params.spinup_comm_period_end   = config_get("motor_spinup_comm_period_end_usec")   * HNSEC_PER_USEC;
	_params.spinup_num_steps         = config_get("motor_spinup_num_steps");

	_params.motor_bemf_window_len_denom      = 100.0f / config_get("motor_bemf_window_pct") + 0.5f;
	_params.bemf_valid_range_pct128          = config_get("motor_bemf_valid_range_pct") * 128 / 100;
	_params.integrated_bemf_threshold_pct128 = config_get("motor_zc_integral_threshold_pct") * 128 / 100;
	_params.zc_failures_max  = config_get("motor_zc_failures_to_stop");
	_params.zc_detects_min   = config_get("motor_zc_detects_to_start");
	_params.comm_period_max  = config_get("motor_comm_period_max_usec") * HNSEC_PER_USEC;
	_params.comm_blank_hnsec = config_get("motor_comm_blank_usec") * HNSEC_PER_USEC;
	_params.input_volt_cur_lowpass_alpha_reciprocal =
		(int)(1.0f / config_get("motor_volt_curr_lowpass_alpha")) - 1;

	assert_always(_params.neutral_voltage_lowpass_alpha_reciprocal >= 0);
	assert_always(_params.input_volt_cur_lowpass_alpha_reciprocal >= 0);

	if (_params.comm_period_max > motor_timer_get_max_delay_hnsec())
		_params.comm_period_max = motor_timer_get_max_delay_hnsec();

	if (_params.spinup_comm_period_end > _params.comm_period_max)
		_params.spinup_comm_period_end = _params.comm_period_max;

	if (_params.spinup_comm_period_begin < _params.spinup_comm_period_end)
		_params.spinup_comm_period_begin = _params.spinup_comm_period_end;

	_params.adc_sampling_period = motor_adc_sampling_period_hnsec();

	lowsyslog("Motor: Config: BEMF window denom: %i, Neutral LP: %i\n",
		_params.motor_bemf_window_len_denom, _params.neutral_voltage_lowpass_alpha_reciprocal);
}

// --- Hard real time code below ---
#pragma GCC optimize 3

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

	if (_state.control_state != CS_PAST_ZC) {  // ZC has timed out
		// In this case we need to emulate the ZC detection
		const uint32_t leeway = comm_period_on_zc_failure / 2 +
			TIMING_ADVANCE64(comm_period_on_zc_failure, _params.timing_advance_deg64);
		_state.prev_zc_timestamp = timestamp_hnsec - leeway;

		// There may be some ZC failures during spinup, it's OK but we don't want to count them
		if (_state.spinup_done)
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
		_state.zc_detects_since_start++;

		// Stall detection update
		if (_state.immediate_zc_failures) {
			_state.immediate_zc_detects++;
			if (_state.immediate_zc_detects > NUM_COMMUTATION_STEPS)
				_state.immediate_zc_failures = 0;
		}
	}
	_state.control_state = CS_BEFORE_ZC; // Waiting for the next ZC

	// Enable ADC sampling
	motor_adc_enable_from_isr();

	// Reset and configure the ZC solver
	_state.num_bemf_samples_acquired = 0;
	_state.num_bemf_samples_past_zc = 0;

	_state.optimal_num_bemf_samples =
		(_state.comm_period / _params.adc_sampling_period) / _params.motor_bemf_window_len_denom + 2;
	if (_state.optimal_num_bemf_samples > MAX_BEMF_SAMPLES)
		_state.optimal_num_bemf_samples = MAX_BEMF_SAMPLES;

	_state.integrated_bemf_voltage = 0;
	_state.blank_time_deadline = timestamp_hnsec + _params.comm_blank_hnsec;

	// Have we reached the stable operation mode?
	if (!_state.spinup_done) {
		if (_state.immediate_zc_failures == 0 && _state.zc_detects_since_start > _params.zc_detects_min) {
			_state.spinup_done = true;
			_state.pwm_val = _state.pwm_val_after_spinup;  // Engage normal duty cycle
		}
	}
}

static void handle_zero_cross(uint64_t zc_timestamp)
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
		_state.comm_period / 2 - TIMING_ADVANCE64(_state.comm_period, _params.timing_advance_deg64);

	// Override the comm period deadline that was set at the last commutation switching
	motor_timer_set_absolute(zc_timestamp + advance);

	// Disable till next comm period
	motor_adc_disable_from_isr();
}

static void update_input_voltage_current(const struct motor_adc_sample* sample)
{
	const int alpha = _params.input_volt_cur_lowpass_alpha_reciprocal;
	_state.input_voltage = (_state.input_voltage * alpha + sample->input_voltage) / (alpha + 1);
	_state.input_current = (_state.input_current * alpha + sample->input_current) / (alpha + 1);
}

static void add_bemf_sample(const int bemf, const uint64_t timestamp)
{
	assert(_state.num_bemf_samples_acquired <= _state.optimal_num_bemf_samples);
	assert(_state.optimal_num_bemf_samples > 1);

	int insertion_index = 0;

	if (_state.num_bemf_samples_acquired < _state.optimal_num_bemf_samples) {
		insertion_index = _state.num_bemf_samples_acquired;
		_state.num_bemf_samples_acquired++;
	} else {
		insertion_index = _state.optimal_num_bemf_samples - 1;
		// Move all samples one step to the left (hope the compiler will replace this with memmove())
		for (int i = 0; i < insertion_index; i++) {
			_state.bemf_samples[i] = _state.bemf_samples[i + 1];
			_state.bemf_timestamps[i] = _state.bemf_timestamps[i + 1];
		}
	}

	_state.bemf_samples[insertion_index] = bemf;
	_state.bemf_timestamps[insertion_index] = timestamp;
}

static void update_neutral_voltage(const struct motor_adc_sample* sample)
{
	const struct motor_pwm_commutation_step* const step = _state.comm_table + _state.current_comm_step;
	const int avg_voltage = (sample->phase_values[step->positive] + sample->phase_values[step->negative]) / 2;
	_state.neutral_voltage =
		(_state.neutral_voltage * _params.neutral_voltage_lowpass_alpha_reciprocal + avg_voltage) /
		(_params.neutral_voltage_lowpass_alpha_reciprocal + 1);
}

static bool is_past_zc(const int bemf)
{
	const bool zc_polarity = _state.current_comm_step & 1;
	return (zc_polarity && (bemf >= 0)) || (!zc_polarity && (bemf <= 0));
}

static bool bemf_integrate_and_check(const int bemf)
{
	/*
	 * Ref. "BLDC Sensorless Algorithm Tuning"
	 * BEMF integration is not used to detect ZC, but rather as a mere heuristic
	 * to decide whether an observed ZC is valid or just a noise.
	 */
	const int integrated_bemf_threshold = _state.neutral_voltage * _params.integrated_bemf_threshold_pct128 / 128;

	if (abs(_state.integrated_bemf_voltage) <= integrated_bemf_threshold)
		_state.integrated_bemf_voltage += bemf;

	return abs(_state.integrated_bemf_voltage) > integrated_bemf_threshold;
}

/// Fixed point multiplier
#define LEAST_SQUARES_MULT   65536

static void solve_least_squares(const int n, const int x[], const int y[], int64_t* out_slope, int64_t* out_yintercept)
{
	assert(n > 1 && out_slope && out_yintercept);

	int64_t sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;

	for (int i = 0; i < n; i++) {
		sum_x += x[i];
		sum_y += y[i];
		sum_xy += x[i] * y[i];
		sum_xx += x[i] * x[i];
	}

	const int64_t a = sum_x * sum_y - n * sum_xy;
	const int64_t b = sum_x * sum_x - n * sum_xx;

	if (b == 0)
		return; // Garbage in - garbage out

	const int64_t slope = LEAST_SQUARES_MULT * a / b;

	*out_slope = slope;
	*out_yintercept = (LEAST_SQUARES_MULT * sum_y - slope * sum_x) / n;
}

static uint64_t solve_zero_cross_approximation(void)
{
	// Some stat
	_state.zc_samples_per_solution = _state.num_bemf_samples_acquired;
	memmove(_state.zc_prev_samples, _state.bemf_samples, _state.num_bemf_samples_acquired * 4);

	// Large X numbers may cause overflow, so we subtract the constant (leftmost timestamp)
	int data_x[MAX_BEMF_SAMPLES];
	for (int i = 0; i < _state.num_bemf_samples_acquired; i++) {
		data_x[i] = _state.bemf_timestamps[i] - _state.bemf_timestamps[0];
		assert(data_x[i] >= 0);
	}

	// Linear fitting
	int64_t slope = 0, yintercept = 0;
	solve_least_squares(_state.num_bemf_samples_acquired, data_x, _state.bemf_samples, &slope, &yintercept);

	_state.zc_slope = slope;

	// Linear equation solved for x
	const int x = -yintercept / slope;

	for (int i = 0; i < _state.num_bemf_samples_acquired; i++)
		_state.zc_prev_fitted_samples[i] = (slope * data_x[i] + yintercept) / LEAST_SQUARES_MULT;

	// TODO: check the covariance

	// Noisy input data may break the solution. Implement an outlier detector?
	if (abs(x) > (int)(_params.adc_sampling_period * _state.num_bemf_samples_acquired * 2))
		return 0;

	return _state.bemf_timestamps[0] + x;
}

void motor_adc_sample_callback(const struct motor_adc_sample* sample)
{
	if (_state.control_state != CS_BEFORE_ZC || sample->timestamp < _state.blank_time_deadline)
		return;
	assert(_state.comm_table);

	/*
	 * Normalization against the neutral voltage
	 */
	update_neutral_voltage(sample);
	const struct motor_pwm_commutation_step* const step = _state.comm_table + _state.current_comm_step;
	const int bemf = sample->phase_values[step->floating] - _state.neutral_voltage;

	/*
	 * BEMF range validation
	 * If out of range, the signal will be discarded as noise.
	 */
	if (abs(bemf) > (_state.neutral_voltage * _params.bemf_valid_range_pct128 / 128)) {
		_state.bemf_samples_out_of_range++;
		return;
	}

	/*
	 * ZC event validation
	 * The first valid sample must be before ZC, obviously, otherwise it is noise
	 */
	const bool past_zc = is_past_zc(bemf);

	if (past_zc && (_state.num_bemf_samples_acquired == 0) && _state.spinup_done) {
		_state.bemf_samples_premature_zc++;
		return;
	}

	/*
	 * Here the BEMF sample is considered to be valid, and can be added to the solution.
	 * Store the sample, update the BEMF integral.
	 */
	add_bemf_sample(bemf, sample->timestamp);

	if (!bemf_integrate_and_check(bemf))
		return;

	/*
	 * Is there enough samples collected?
	 */
	bool enough_samples = false;
	if (past_zc) {
		const int optimal_samples_past_zc = _state.optimal_num_bemf_samples / 2;
		_state.num_bemf_samples_past_zc++;

		enough_samples =
			(_state.num_bemf_samples_past_zc >= optimal_samples_past_zc) &&
			(_state.num_bemf_samples_acquired > 1);

		if (step->floating == 0) {
			TESTPAD_ZC_SET();
		}
	}

	if (!enough_samples)
		return;

	/*
	 * Find the exact ZC position using the collected samples
	 */
	const uint64_t zc_timestamp = solve_zero_cross_approximation();

	TESTPAD_ZC_CLEAR();

	if (zc_timestamp == 0) {
		// Sorry Mario
		_state.control_state = CS_FAILED_ZC;
		_state.num_bemf_samples_acquired = 0;
		_state.num_bemf_samples_past_zc = 0;
		if (_state.spinup_done)
			_state.zc_failed_solutions++;
		return;
	}

	handle_zero_cross(zc_timestamp);
	update_input_voltage_current(sample);
}

// --- End of hard real time code ---
#pragma GCC reset_options

int motor_init(void)
{
	int ret = motor_pwm_init(config_get("motor_pwm_frequency"));
	if (ret)
		return ret;

	motor_timer_init();

	ret = motor_adc_init(config_get("motor_current_shunt_mohm") * 1000.0f);
	if (ret)
		return ret;

	configure();
	motor_stop();
	return 0;
}

void motor_confirm_initialization(void)
{
	assert(!_initialization_confirmed);
	_initialization_confirmed = true;
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

	// Strict timing is not required here, so usleep() is OK
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
		// usleep() is not precise enough for the spinup commutations, so we use busyloop:
		motor_timer_hndelay(current_comm_period);
		current_comm_period -= accel;
	}
}

void motor_start(float spinup_duty_cycle, float normal_duty_cycle, bool reverse)
{
	assert(spinup_duty_cycle >= 0);
	assert(normal_duty_cycle >= 0);

	motor_stop();                          // Just in case

	if (!_initialization_confirmed)
		return; // Go home you're drunk

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
	const enum control_state_id csid = _state.control_state;
	const bool spinup_done = _state.spinup_done;

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

	const uint32_t val = _state.comm_period;
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
		volt = _state.input_voltage;
		curr = _state.input_current;
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
		"  zc failures        %u\n"
		"  zc spls in sol     %i\n"
		"  zc sol failures    %u\n"
		"  zc slope           %f\n"
		"  opt bemf spls      %i\n"
		"  bemf spls oor      %u\n"
		"  bemf spls prem     %u\n"
		"  voltage, current   %i %i\n",
		(unsigned)(state_copy.comm_period / HNSEC_PER_USEC),
		(int)state_copy.neutral_voltage,
		(unsigned)state_copy.zc_failures_since_start,
		_state.zc_samples_per_solution,
		(unsigned)_state.zc_failed_solutions,
		_state.zc_slope / (float)LEAST_SQUARES_MULT,
		_state.optimal_num_bemf_samples,
		(unsigned)_state.bemf_samples_out_of_range,
		(unsigned)_state.bemf_samples_premature_zc,
		state_copy.input_voltage, state_copy.input_current);

	if (_state.zc_samples_per_solution > 0) {
		// ZC samples
		lowsyslog("  zc samples  ");
		for (int i = 0; i < _state.zc_samples_per_solution; i++)
			lowsyslog("%i ", _state.zc_prev_samples[i]);
		lowsyslog("\n");

		// ZC fitted samples
		lowsyslog("  zc fitting  ");
		for (int i = 0; i < _state.zc_samples_per_solution; i++)
			lowsyslog("%i ", _state.zc_prev_fitted_samples[i]);
		lowsyslog("\n");
	}
}
