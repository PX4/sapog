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

#define COMM_PERIOD_LOWPASS_MAX    (1 * HNSEC_PER_USEC)

/**
 * Large values affect performance
 * Least possible value is 2
 */
#define MAX_BEMF_SAMPLES           8

/**
 * Computes the timing advance in comm_period units
 */
#define TIMING_ADVANCE64(comm_period, degrees) \
	(((int64_t)comm_period * (int64_t)degrees) / 64LL)

/**
 * Generic lowpass filter with correct rounding
 */
#define LOWPASS(xold, xnew, alpha_rcpr) \
	(((xold) * (alpha_rcpr) + (xnew) + (((alpha_rcpr) + 1) / 2)) / ((alpha_rcpr) + 1))

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

static struct diag_info                /// This data is never used by the control algorithms, it is write only
{
	/// Increment-only counters
	uint64_t zc_failures_since_start;
	uint64_t zc_solution_failures;
	uint64_t bemf_samples_out_of_range;
	uint64_t bemf_samples_premature_zc;

	/// Last ZC solution
	int64_t zc_solution_slope;
	int64_t zc_solution_yintercept;
	int zc_solution_num_samples;
#if DEBUG
	int zc_solution_samples[MAX_BEMF_SAMPLES];
#endif

	uint64_t started_at;
} _diag;

static struct control_state            /// Control state
{
	enum control_state_id control_state;

	uint64_t blank_time_deadline;
	uint64_t prev_zc_timestamp;
	uint32_t comm_period;

	int current_comm_step;
	const struct motor_pwm_commutation_step* comm_table;

	bool spinup_done;

	unsigned immediate_zc_failures;
	unsigned immediate_zc_detects;
	unsigned zc_detects_during_spinup;

	int zc_bemf_samples[MAX_BEMF_SAMPLES];
	uint64_t zc_bemf_timestamps[MAX_BEMF_SAMPLES];
	int zc_bemf_samples_optimal;
	int zc_bemf_samples_acquired;
	int zc_bemf_samples_acquired_past_zc;

	int neutral_voltage;
	int integrated_bemf_voltage;

	int input_voltage;
	int input_current;

	int pwm_val;
	int pwm_val_after_spinup;
} _state;

static struct precomputed_params       /// Parameters are read only
{
	uint32_t comm_period_lowpass_base;
	int comm_period_lowpass_alpha_reciprocal_min;
	int timing_advance_deg64;
	int neutral_voltage_lowpass_alpha_reciprocal;

	int comm_period_shift_on_zc_failure;
	int motor_bemf_window_len_denom;
	int bemf_valid_range_pct128;
	int integrated_bemf_threshold_pct128;
	unsigned zc_failures_max;
	unsigned zc_detects_min;
	uint32_t comm_period_max;
	int comm_blank_hnsec;
	int input_volt_cur_lowpass_alpha_reciprocal;

	uint32_t spinup_end_comm_period;
	uint32_t spinup_timeout;
	uint32_t spinup_vipd_probe_duration;
	uint32_t spinup_vipd_drive_duration;

	uint32_t adc_sampling_period;
} _params;

static bool _initialization_confirmed = false;


CONFIG_PARAM_INT("motor_pwm_frequency",                30000, MOTOR_PWM_MIN_FREQUENCY, MOTOR_PWM_MAX_FREQUENCY)
CONFIG_PARAM_FLOAT("motor_current_shunt_mohm",         0.5,   0.01,  10.0)
// Most important parameters
CONFIG_PARAM_INT("motor_comm_period_lpf_base_usec",    5000,  0,     50000)
CONFIG_PARAM_FLOAT("motor_comm_period_lpf_alpha_max",  0.5,   0.1,   1.0)
CONFIG_PARAM_INT("motor_timing_advance_deg",           0,    -5,     20)
CONFIG_PARAM_FLOAT("motor_neutral_volt_lpf_alpha",     1.0,   0.1,   1.0)
CONFIG_PARAM_INT("motor_comm_blank_usec",              40,    30,    100)
// Something not so important
CONFIG_PARAM_INT("motor_deceleration_rate_on_zc_miss", 3,     0,     8)
CONFIG_PARAM_INT("motor_bemf_window_pct",              25,    10,    70)
CONFIG_PARAM_INT("motor_bemf_valid_range_pct",         70,    10,    100)
CONFIG_PARAM_INT("motor_zc_integral_threshold_pct",    15,    0,     200)
CONFIG_PARAM_INT("motor_zc_failures_to_stop",          50,    6,     300)
CONFIG_PARAM_INT("motor_zc_detects_to_start",          100,   6,     500)
CONFIG_PARAM_INT("motor_comm_period_max_usec",         16000, 1000,  50000)
CONFIG_PARAM_FLOAT("motor_volt_curr_lpf_alpha",        0.2,   0.1,   1.0)
// Spinup settings
CONFIG_PARAM_INT("motor_spinup_end_comm_period_usec",  10000, 8000,  90000)
CONFIG_PARAM_INT("motor_spinup_timeout_ms",            1000,  100,   4000)
CONFIG_PARAM_INT("motor_spinup_vipd_probe_usec",       50,    10,    200)
CONFIG_PARAM_INT("motor_spinup_vipd_drive_usec",       2000,  1000,  4000)


static void configure(void)
{
	_params.comm_period_lowpass_base = config_get("motor_comm_period_lpf_base_usec") * HNSEC_PER_USEC;
	_params.comm_period_lowpass_alpha_reciprocal_min =
		(int)(1.0f / config_get("motor_comm_period_lpf_alpha_max") + 0.5f) - 1;

	_params.timing_advance_deg64 = config_get("motor_timing_advance_deg") * 64 / 60;
	_params.neutral_voltage_lowpass_alpha_reciprocal =
		(int)(1.0f / config_get("motor_neutral_volt_lpf_alpha") + 0.5f) - 1;

	_params.comm_period_shift_on_zc_failure  = config_get("motor_deceleration_rate_on_zc_miss");
	_params.motor_bemf_window_len_denom      = 100.0f / config_get("motor_bemf_window_pct") + 0.5f;
	_params.bemf_valid_range_pct128          = config_get("motor_bemf_valid_range_pct") * 128 / 100;
	_params.integrated_bemf_threshold_pct128 = config_get("motor_zc_integral_threshold_pct") * 128 / 100;
	_params.zc_failures_max  = config_get("motor_zc_failures_to_stop");
	_params.zc_detects_min   = config_get("motor_zc_detects_to_start");
	_params.comm_period_max  = config_get("motor_comm_period_max_usec") * HNSEC_PER_USEC;
	_params.comm_blank_hnsec = config_get("motor_comm_blank_usec") * HNSEC_PER_USEC;
	_params.input_volt_cur_lowpass_alpha_reciprocal =
		(int)(1.0f / config_get("motor_volt_curr_lpf_alpha")) - 1;

	_params.spinup_end_comm_period     = config_get("motor_spinup_end_comm_period_usec") * HNSEC_PER_USEC;
	_params.spinup_timeout             = config_get("motor_spinup_timeout_ms") * HNSEC_PER_MSEC;
	_params.spinup_vipd_probe_duration = config_get("motor_spinup_vipd_probe_usec") * HNSEC_PER_USEC;
	_params.spinup_vipd_drive_duration = config_get("motor_spinup_vipd_drive_usec") * HNSEC_PER_USEC;

	/*
	 * Validation
	 */
	assert_always(_params.neutral_voltage_lowpass_alpha_reciprocal >= 0);
	assert_always(_params.input_volt_cur_lowpass_alpha_reciprocal >= 0);

	if (_params.comm_period_max > motor_timer_get_max_delay_hnsec())
		_params.comm_period_max = motor_timer_get_max_delay_hnsec();

	_params.adc_sampling_period = motor_adc_sampling_period_hnsec();

	lowsyslog("Motor: Config: Max comm period: %u usec, BEMF window denom: %i, Neutral LP: %i\n",
		_params.comm_period_max / HNSEC_PER_USEC,
		_params.motor_bemf_window_len_denom,
		_params.neutral_voltage_lowpass_alpha_reciprocal);
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
	motor_pwm_set_step_from_isr(_state.comm_table + _state.current_comm_step, _state.pwm_val);
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

	if (_state.control_state != CS_PAST_ZC) {
		// ZC has timed out - in this case we need to emulate the ZC detection
		const uint32_t leeway = comm_period_on_zc_failure / 2 +
			TIMING_ADVANCE64(comm_period_on_zc_failure, _params.timing_advance_deg64);
		_state.prev_zc_timestamp = timestamp_hnsec - leeway;

		// There may be some ZC failures during spinup, it's OK but we don't want to count them
		if (_state.spinup_done)
			_diag.zc_failures_since_start++;

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
	} else {
		// On successful ZC
		if (!_state.spinup_done)
			_state.zc_detects_during_spinup++;

		// Stall detection update
		if (_state.immediate_zc_failures) {
			_state.immediate_zc_detects++;
			if (_state.immediate_zc_detects > NUM_COMMUTATION_STEPS)
				_state.immediate_zc_failures = 0;
		}
	}
	_state.control_state = CS_BEFORE_ZC; // Waiting for the next ZC

	motor_adc_enable_from_isr();

	_state.zc_bemf_samples_acquired = 0;
	_state.zc_bemf_samples_acquired_past_zc = 0;

	_state.zc_bemf_samples_optimal =
		(_state.comm_period / _params.adc_sampling_period) / _params.motor_bemf_window_len_denom + 2;
	if (_state.zc_bemf_samples_optimal > MAX_BEMF_SAMPLES)
		_state.zc_bemf_samples_optimal = MAX_BEMF_SAMPLES;

	_state.integrated_bemf_voltage = 0;
	_state.blank_time_deadline = timestamp_hnsec + _params.comm_blank_hnsec;

	// Have we reached the stable operation mode?
	if (!_state.spinup_done) {
		if (_state.immediate_zc_failures == 0 && _state.zc_detects_during_spinup > _params.zc_detects_min) {
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

	if (comm_period_lowpass < (unsigned)_params.comm_period_lowpass_alpha_reciprocal_min)
		comm_period_lowpass = _params.comm_period_lowpass_alpha_reciprocal_min;

	_state.comm_period = LOWPASS(_state.comm_period, new_comm_period, comm_period_lowpass);

	_state.control_state = CS_PAST_ZC;

	const uint32_t advance =
		_state.comm_period / 2 - TIMING_ADVANCE64(_state.comm_period, _params.timing_advance_deg64);

	motor_timer_set_absolute(zc_timestamp + advance);
	motor_adc_disable_from_isr();
}

static void update_input_voltage_current(const struct motor_adc_sample* sample)
{
	const int alpha = _params.input_volt_cur_lowpass_alpha_reciprocal;
	_state.input_voltage = LOWPASS(_state.input_voltage, sample->input_voltage, alpha);
	_state.input_current = LOWPASS(_state.input_current, sample->input_current, alpha);
}

static void add_bemf_sample(const int bemf, const uint64_t timestamp)
{
	assert(_state.zc_bemf_samples_acquired <= _state.zc_bemf_samples_optimal);
	assert(_state.zc_bemf_samples_optimal > 1);

	int insertion_index = 0;

	if (_state.zc_bemf_samples_acquired < _state.zc_bemf_samples_optimal) {
		insertion_index = _state.zc_bemf_samples_acquired;
		_state.zc_bemf_samples_acquired++;
	} else {
		insertion_index = _state.zc_bemf_samples_optimal - 1;
		// Move all samples one step to the left (hope the compiler will replace this with memmove())
		for (int i = 0; i < insertion_index; i++) {
			_state.zc_bemf_samples[i] = _state.zc_bemf_samples[i + 1];
			_state.zc_bemf_timestamps[i] = _state.zc_bemf_timestamps[i + 1];
		}
	}

	_state.zc_bemf_samples[insertion_index] = bemf;
	_state.zc_bemf_timestamps[insertion_index] = timestamp;
}

static void update_neutral_voltage(const struct motor_adc_sample* sample)
{
	const struct motor_pwm_commutation_step* const step = _state.comm_table + _state.current_comm_step;
	const int avg_voltage = (sample->phase_values[step->positive] + sample->phase_values[step->negative]) / 2;
	_state.neutral_voltage =
		LOWPASS(_state.neutral_voltage, avg_voltage, _params.neutral_voltage_lowpass_alpha_reciprocal);
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
#define LEAST_SQUARES_MULT   (1 << 17)

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
	/*
	 * Solution
	 */
	int data_x[MAX_BEMF_SAMPLES];                     // Subtract the leftmost timestamp to avoid overflows
	for (int i = 0; i < _state.zc_bemf_samples_acquired; i++) {
		data_x[i] = _state.zc_bemf_timestamps[i] - _state.zc_bemf_timestamps[0];
		assert(data_x[i] >= 0);
	}

	int64_t slope = 0, yintercept = 0;
	solve_least_squares(_state.zc_bemf_samples_acquired, data_x, _state.zc_bemf_samples, &slope, &yintercept);

	const int x = -yintercept / slope; // Linear equation solved for x

	/*
	 * Diagnostic info
	 */
	_diag.zc_solution_slope = slope;
	_diag.zc_solution_yintercept = yintercept;
	_diag.zc_solution_num_samples = _state.zc_bemf_samples_acquired;
#if DEBUG
	memcpy(_diag.zc_solution_samples, _state.zc_bemf_samples,
		_state.zc_bemf_samples_acquired * sizeof(_state.zc_bemf_samples[0]));
#endif

	/*
	 * Solution validation
	 * Implement an outlier detector? Check the solution covariance?
	 */
	if (abs(x) > (int)(_params.adc_sampling_period * _state.zc_bemf_samples_acquired * 2)) {
		_diag.zc_solution_failures++;
		return 0;
	}
	return _state.zc_bemf_timestamps[0] + x;
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
	 */
	if (abs(bemf) > (_state.neutral_voltage * _params.bemf_valid_range_pct128 / 128)) {
		_diag.bemf_samples_out_of_range++;
		_state.zc_bemf_samples_acquired = 0;
		_state.zc_bemf_samples_acquired_past_zc = 0;
		return;
	}

	/*
	 * ZC event validation
	 */
	const bool past_zc = is_past_zc(bemf);

	if (past_zc && (_state.zc_bemf_samples_acquired == 0) && _state.spinup_done) {
		_diag.bemf_samples_premature_zc++;
		return;
	}

	/*
	 * Here the BEMF sample is considered to be valid, and can be added to the solution.
	 * Input voltage/current updates are synced with ZC - this way the noise can be reduced.
	 */
	add_bemf_sample(bemf, sample->timestamp);

	if (!bemf_integrate_and_check(bemf))
		return;

	if (past_zc)
		update_input_voltage_current(sample);

	/*
	 * Is there enough samples collected?
	 */
	bool enough_samples = false;
	if (past_zc) {
		const int optimal_samples_past_zc = _state.zc_bemf_samples_optimal / 2;
		_state.zc_bemf_samples_acquired_past_zc++;

		enough_samples =
			(_state.zc_bemf_samples_acquired_past_zc >= optimal_samples_past_zc) &&
			(_state.zc_bemf_samples_acquired > 1);

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
		return;
	}
	handle_zero_cross(zc_timestamp);
}

// --- End of hard real time code ---
#pragma GCC reset_options

int motor_init(void)
{
	int ret = motor_pwm_init(config_get("motor_pwm_frequency"));
	if (ret)
		return ret;

	motor_timer_init();

	ret = motor_adc_init(config_get("motor_current_shunt_mohm") / 1000.0f);
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

static void wait_adc_samples(int num)
{
	assert(num > 0);
	while (num --> 0) {
		const volatile uint64_t timestamp = motor_adc_get_last_sample().timestamp;
		while (timestamp == motor_adc_get_last_sample().timestamp)
			__asm volatile ("nop");
	}
}

static int detect_rotor_position_as_step_index(void)
{
	/*
	 * Ref. "Start-up Control Algorithm for Sensorless and Variable Load BLDC Control
	 *       Using Variable Inductance Sensing Method"
	 */
	static const int ENERGIZING_TABLE_FORWARD[3][3] = {
		{1, 0, 0},
		{0, 0, 1},
		{0, 1, 0}
	};
	static const int ENERGIZING_TABLE_REVERSE[3][3] = {
		{0, 1, 1},
		{1, 1, 0},
		{1, 0, 1}
	};
	static const int POS_MAP[6] = {2, 0, 1, 4, 3, 5}; // Step order: 1 2 0 4 3 5

	const int pwm_val = motor_pwm_compute_pwm_val(1.0);

	// With proper rounding
	const int num_samples_energize =
		(_params.spinup_vipd_probe_duration + _params.adc_sampling_period / 2) / _params.adc_sampling_period;
	assert(num_samples_energize > 0 && num_samples_energize < 4);

	const int num_samples_sleep = num_samples_energize * 2;

	chSysSuspend();
	motor_pwm_set_freewheeling();

	int position_code = 0;

	for (int i = 0; i < 3; i++) {
		// We don't care about absolute current values so we don't use offset or scaling
		// Forward
		wait_adc_samples(num_samples_sleep);
		motor_pwm_align(ENERGIZING_TABLE_FORWARD[i], pwm_val);
		wait_adc_samples(num_samples_energize);
		const int forward_current = motor_adc_get_last_sample().input_current;
		motor_pwm_set_freewheeling();

		// Reverse
		wait_adc_samples(num_samples_sleep);
		motor_pwm_align(ENERGIZING_TABLE_REVERSE[i], pwm_val);
		wait_adc_samples(num_samples_energize);
		const int reverse_current = motor_adc_get_last_sample().input_current;
		motor_pwm_set_freewheeling();

		// Position update
		if (forward_current > reverse_current)
			position_code++;
		position_code <<= 1;
	}
	chSysEnable();
	position_code = (position_code >> 1) - 1; // Will be in range [0; 5]
	position_code = POS_MAP[position_code];
	return position_code;
}

static bool do_variable_inductance_spinup(void)
{
	const uint64_t deadline = motor_timer_hnsec() + _params.spinup_timeout;

	uint64_t prev_step_timestamp = motor_timer_hnsec();
	int good_steps = 0;

	_state.comm_period = _params.comm_period_max;
	_state.current_comm_step = detect_rotor_position_as_step_index();

	while (motor_timer_hnsec() < deadline) {
		// We need some arbitrary amount of time to drive the motor between the measurements
		usleep(_params.spinup_vipd_drive_duration / HNSEC_PER_USEC);

		const int this_step = detect_rotor_position_as_step_index();

		// Bring back the power
		irq_primask_disable();
		motor_pwm_set_step_from_isr(_state.comm_table + this_step, _state.pwm_val);
		irq_primask_enable();

		if (_state.current_comm_step == this_step)
			continue;

		const bool valid_sequence = abs(_state.current_comm_step - this_step) <= 1;
		_state.current_comm_step = this_step;
		if (!valid_sequence)
			continue;

		const uint64_t timestamp = motor_timer_hnsec();
		_state.comm_period = (_state.comm_period + (timestamp - prev_step_timestamp)) / 2;
		prev_step_timestamp = timestamp;

		if (_state.comm_period < _params.spinup_end_comm_period) {
			good_steps++;
			if (good_steps > NUM_COMMUTATION_STEPS) // Just in case, do multiple commutations
				break;
		} else {
			good_steps = 0;
		}
	}
	return _state.comm_period < _params.spinup_end_comm_period;
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
	 * Initialize the structs
	 */
	memset(&_diag, 0, sizeof(_diag));
	memset(&_state, 0, sizeof(_state));    // Mighty reset

	_diag.started_at = motor_timer_hnsec();

	_state.comm_table = reverse ? COMMUTATION_TABLE_REVERSE : COMMUTATION_TABLE_FORWARD;
	_state.spinup_done = false;

	_state.pwm_val = motor_pwm_compute_pwm_val(spinup_duty_cycle);
	_state.pwm_val_after_spinup = motor_pwm_compute_pwm_val(normal_duty_cycle);

	init_adc_filters();

	/*
	 * Low speed spin-up based on variable inductance position detection
	 */
	const tprio_t orig_priority = chThdSetPriority(HIGHPRIO);

	const bool started = do_variable_inductance_spinup();

	/*
	 * Engage the closed-loop mode if started
	 */
	if (started) {
		_state.blank_time_deadline = motor_timer_hnsec() + _params.comm_blank_hnsec;
		_state.control_state = CS_BEFORE_ZC;
		motor_timer_set_relative(0);
	} else {
		motor_stop();
	}
	chThdSetPriority(orig_priority);
}

void motor_stop(void)
{
	irq_primask_disable();
	{
		_state.control_state = CS_IDLE;
		// ADC should be enabled by default
		motor_adc_enable_from_isr();
	}
	irq_primask_enable();

	motor_timer_cancel();
	motor_pwm_set_freewheeling();
}

void motor_set_duty_cycle(float duty_cycle)
{
	// We don't need a critical section to write an integer
	_state.pwm_val = motor_pwm_compute_pwm_val(duty_cycle);
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

		const tprio_t orig_priority = chThdSetPriority(HIGHPRIO);
		motor_pwm_beep(frequency, duration_msec);
		chThdSetPriority(orig_priority);

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
	const uint64_t ret = _diag.zc_failures_since_start;
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

uint32_t motor_get_limit_comm_period_hnsec(void) // TODO: rename
{
	// Ensure some number of ADC samples per comm period
	return motor_adc_sampling_period_hnsec() * 5;
}

void motor_print_debug_info(void)
{
	static const int ALIGNMENT = 25;

#define PRINT_INT(name, value) lowsyslog("  %-*s %D\n", ALIGNMENT, (name), (long)(value))
#define PRINT_FLT(name, value) lowsyslog("  %-*s %f\n", ALIGNMENT, (name), (float)(value))

	/*
	 * Instant state
	 */
	irq_primask_disable();
	const struct control_state state_copy = _state;
	irq_primask_enable();

	lowsyslog("Motor state\n");
	PRINT_INT("comm period",     state_copy.comm_period / HNSEC_PER_USEC);
	PRINT_INT("neutral voltage", state_copy.neutral_voltage);
	PRINT_INT("input voltage",   state_copy.input_voltage);
	PRINT_INT("input current",   state_copy.input_current);

	/*
	 * Diagnostics
	 */
	irq_primask_disable();
	const struct diag_info diag_copy = _diag;
	irq_primask_enable();

	lowsyslog("Motor diag\n");
	PRINT_INT("zc failures",       diag_copy.zc_failures_since_start);
	PRINT_INT("bemf out of range", diag_copy.bemf_samples_out_of_range);
	PRINT_INT("bemf premature zc", diag_copy.bemf_samples_premature_zc);
	PRINT_INT("zc sol failures",   diag_copy.zc_solution_failures);
	PRINT_INT("zc sol num samples",diag_copy.zc_solution_num_samples);
	PRINT_FLT("zc sol slope",      diag_copy.zc_solution_slope / (float)LEAST_SQUARES_MULT);
	PRINT_FLT("zc sol yintercept", diag_copy.zc_solution_yintercept / (float)LEAST_SQUARES_MULT);

	/*
	 * ZC fitting
	 */
#if DEBUG
	if (_diag.zc_solution_num_samples > 0) {
		lowsyslog("Motor ZC solution data\n");

		lowsyslog("  zc samples   ");
		for (int i = 0; i < _diag.zc_solution_num_samples; i++)
			lowsyslog("%-5i ", diag_copy.zc_solution_samples[i]);
		lowsyslog("\n");

		lowsyslog("  zc fitted    ");
		for (int i = 0; i < _diag.zc_solution_num_samples; i++) {
			const int x = _params.adc_sampling_period * i;
			const int y = (diag_copy.zc_solution_slope * x + diag_copy.zc_solution_yintercept) /
				LEAST_SQUARES_MULT;
			lowsyslog("%-5i ", y);
		}
		lowsyslog("\n");
	}
#endif

#undef PRINT_INT
#undef PRINT_FLT
}
