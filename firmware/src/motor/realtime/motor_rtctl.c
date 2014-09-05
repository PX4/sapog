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

#include "api.h"
#include "internal.h"
#include "adc.h"
#include "pwm.h"
#include "timer.h"
#include "forced_rotation_detection.h"
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <limits.h>
#include <config/config.h>


#define STEP_SWITCHING_DELAY_HNSEC (1 * HNSEC_PER_USEC)

/**
 * Large values affect performance
 * Least possible value is 2
 */
#define MAX_BEMF_SAMPLES           8

#define ABS_MIN_COMM_PERIOD_USEC   150

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
static const struct motor_pwm_commutation_step COMMUTATION_TABLE_FORWARD[MOTOR_NUM_COMMUTATION_STEPS] = {
	{1, 0, 2},
	{1, 2, 0},
	{0, 2, 1},
	{0, 1, 2},
	{2, 1, 0},
	{2, 0, 1}
};
static const struct motor_pwm_commutation_step COMMUTATION_TABLE_REVERSE[MOTOR_NUM_COMMUTATION_STEPS] = {
	{2, 0, 1},
	{2, 1, 0},
	{0, 1, 2},
	{0, 2, 1},
	{1, 2, 0},
	{1, 0, 2}
};

enum flags
{
	FLAG_ACTIVE        = 1,
	FLAG_SPINUP        = 2,
	FLAG_SYNC_RECOVERY = 4
};

enum zc_detection_result
{
	ZC_NOT_DETECTED,
	ZC_DETECTED,
	ZC_FAILED,
	ZC_DESATURATION
};

static struct diag_info                /// This data is never used by the control algorithms, it is write only
{
	/// Increment-only counters
	uint32_t zc_failures_since_start;
	uint32_t zc_solution_failures;
	uint32_t bemf_samples_out_of_range;
	uint32_t bemf_samples_premature_zc;
	uint32_t desaturations;

	/// Last ZC solution
	int64_t zc_solution_slope;
	int64_t zc_solution_yintercept;
	int zc_solution_num_samples;
#if DEBUG_BUILD
	int zc_solution_samples[MAX_BEMF_SAMPLES];
#endif

	uint64_t started_at;
} _diag;

static struct control_state            /// Control state
{
	unsigned flags;
	enum zc_detection_result zc_detection_result;

	uint64_t blank_time_deadline;
	uint64_t prev_zc_timestamp;
	uint32_t comm_period;

	int current_comm_step;
	const struct motor_pwm_commutation_step* comm_table;

	unsigned immediate_zc_failures;
	unsigned immediate_zc_detects;
	unsigned immediate_desaturations;
	unsigned zc_detects_during_spinup;

	int zc_bemf_samples[MAX_BEMF_SAMPLES];
	uint64_t zc_bemf_timestamps[MAX_BEMF_SAMPLES];
	int zc_bemf_samples_optimal;
	int zc_bemf_samples_optimal_past_zc;
	int zc_bemf_samples_acquired;
	int zc_bemf_samples_acquired_past_zc;

	int neutral_voltage;

	int input_voltage;
	int input_current;

	int pwm_val;
	int pwm_val_after_spinup;
} _state;

static struct precomputed_params       /// Parameters are read only
{
	int timing_advance_deg64;
	int motor_bemf_window_len_denom;
	int bemf_valid_range_pct128;
	unsigned zc_failures_max;
	unsigned zc_detects_min;
	uint32_t comm_period_max;
	int comm_blank_hnsec;

	uint32_t spinup_timeout;
	uint32_t spinup_start_comm_period;
	uint32_t spinup_end_comm_period;
	unsigned spinup_num_good_comms;
	float spinup_duty_cycle_increment;

	uint32_t adc_sampling_period;

	float dc_testpad_threshold;
} _params;

static bool _initialization_confirmed = false;


// Most important parameters
CONFIG_PARAM_INT("motor_timing_advance_deg",           15,    0,     15)
CONFIG_PARAM_INT("motor_comm_blank_usec",              40,    30,    100)
CONFIG_PARAM_INT("motor_bemf_window_len_denom",        4,     3,     8)
CONFIG_PARAM_INT("motor_bemf_valid_range_pct",         70,    10,    100)
CONFIG_PARAM_INT("motor_zc_failures_to_stop",          40,    6,     300)
CONFIG_PARAM_INT("motor_zc_detects_to_start",          40,    6,     1000)
CONFIG_PARAM_INT("motor_comm_period_max_usec",         12000, 1000,  50000)
// Spinup settings
CONFIG_PARAM_INT("motor_spinup_timeout_ms",            1000,  100,   2000)
CONFIG_PARAM_INT("motor_spinup_start_comm_period_usec",50000, 10000, 200000)
CONFIG_PARAM_INT("motor_spinup_end_comm_period_usec",  2000,  1000,  10000)
CONFIG_PARAM_INT("motor_spinup_num_good_comms",        60,    6,     1000)
CONFIG_PARAM_FLOAT("motor_spinup_duty_cycle_inc",      0.02,  0.001, 0.1)
// Debug
CONFIG_PARAM_FLOAT("motor_dc_testpad_threshold",       0.3,   0.0,   1.0)


static void configure(void)
{
	_params.timing_advance_deg64        = config_get("motor_timing_advance_deg") * 64 / 60;
	_params.motor_bemf_window_len_denom = config_get("motor_bemf_window_len_denom");
	_params.bemf_valid_range_pct128     = config_get("motor_bemf_valid_range_pct") * 128 / 100;
	_params.zc_failures_max  = config_get("motor_zc_failures_to_stop");
	_params.zc_detects_min   = config_get("motor_zc_detects_to_start");
	_params.comm_period_max  = config_get("motor_comm_period_max_usec") * HNSEC_PER_USEC;
	_params.comm_blank_hnsec = config_get("motor_comm_blank_usec") * HNSEC_PER_USEC;

	_params.spinup_timeout              = config_get("motor_spinup_timeout_ms") * HNSEC_PER_MSEC;
	_params.spinup_start_comm_period    = config_get("motor_spinup_start_comm_period_usec") * HNSEC_PER_USEC;
	_params.spinup_end_comm_period      = config_get("motor_spinup_end_comm_period_usec") * HNSEC_PER_USEC;
	_params.spinup_num_good_comms       = config_get("motor_spinup_num_good_comms");
	_params.spinup_duty_cycle_increment = config_get("motor_spinup_duty_cycle_inc");

	_params.dc_testpad_threshold = config_get("motor_dc_testpad_threshold");

	/*
	 * Validation
	 */
	if (_params.comm_period_max > motor_timer_get_max_delay_hnsec()) {
		_params.comm_period_max = motor_timer_get_max_delay_hnsec();
	}

	if (_params.spinup_end_comm_period > _params.comm_period_max) {
		_params.spinup_end_comm_period = _params.comm_period_max;
	}

	_params.adc_sampling_period = motor_adc_sampling_period_hnsec();

	lowsyslog("Motor: RTCTL config: Max comm period: %u usec, BEMF window denom: %i\n",
		(unsigned)(_params.comm_period_max / HNSEC_PER_USEC),
		_params.motor_bemf_window_len_denom);
}

// --- Hard real time code below ---
#pragma GCC optimize 3

static void stop_from_isr(void)
{
	_state.flags = 0;
	motor_timer_cancel();
	motor_pwm_set_freewheeling();
}

static void engage_current_comm_step(void)
{
	assert(_state.comm_table);
	motor_pwm_set_step_from_isr(_state.comm_table + _state.current_comm_step, _state.pwm_val);
}

static void register_good_step(void)
{
	if (_state.immediate_zc_failures > 0) {
		_state.immediate_zc_detects++;
		if (_state.immediate_zc_detects > MOTOR_NUM_COMMUTATION_STEPS) {
			_state.immediate_zc_failures = 0;
		}
	}

	if (_state.immediate_desaturations > 0) {
		_state.immediate_desaturations--;
	}

	if (_state.flags & FLAG_SPINUP) {
		_state.zc_detects_during_spinup++;

		const bool is_stable =
			_state.immediate_zc_failures == 0 &&
			_state.zc_detects_during_spinup > _params.zc_detects_min;

		if (is_stable) {
			_state.flags &= ~FLAG_SPINUP;
			_state.pwm_val = _state.pwm_val_after_spinup;
		}
	}
}

static void register_bad_step(bool* need_to_stop)
{
	assert(need_to_stop);

	_state.immediate_zc_detects = 0;
	_state.immediate_zc_failures++;
	*need_to_stop = _state.immediate_zc_failures > _params.zc_failures_max;

	// There may be some ZC failures during spinup, it's OK but we don't want to count them
	if (!(_state.flags & FLAG_SPINUP)) {
		_diag.zc_failures_since_start++;
	}
}

static void fake_missed_zc_detection(uint64_t timestamp_hnsec)
{
	const uint32_t leeway = _state.comm_period / 2 +
		TIMING_ADVANCE64(_state.comm_period, _params.timing_advance_deg64);

	_state.prev_zc_timestamp = timestamp_hnsec - leeway;
}

static void prepare_zc_detector_for_next_step(void)
{
	_state.zc_bemf_samples_acquired = 0;
	_state.zc_bemf_samples_acquired_past_zc = 0;

	/*
	 * Actual length of the BEMF sampling window depends on the advance angle.
	 * E.g. advance 15 deg makes the sampling window twice shorter after ZC,
	 * thus the denom increases by the same amount.
	 */
	const int denom = _params.motor_bemf_window_len_denom +
		_params.motor_bemf_window_len_denom * _params.timing_advance_deg64 / 16;

	_state.zc_bemf_samples_optimal = (_state.comm_period / _params.adc_sampling_period) / denom + 2;

	if (_state.zc_bemf_samples_optimal > MAX_BEMF_SAMPLES) {
		_state.zc_bemf_samples_optimal = MAX_BEMF_SAMPLES;
	}
	_state.zc_bemf_samples_optimal_past_zc = _state.zc_bemf_samples_optimal / 2;
}

void motor_timer_callback(uint64_t timestamp_hnsec)
{
	if (!(_state.flags & FLAG_ACTIVE)) {
		return;
	}

	motor_timer_set_relative(_state.comm_period);

	// Next comm step
	_state.current_comm_step++;
	if (_state.current_comm_step >= MOTOR_NUM_COMMUTATION_STEPS) {
		_state.current_comm_step = 0;
	}

	bool stop_now = false;

	switch (_state.zc_detection_result) {
	case ZC_DETECTED: {
		engage_current_comm_step();
		register_good_step();
		_state.flags &= ~FLAG_SYNC_RECOVERY;
		break;
	}
	case ZC_DESATURATION: {
		engage_current_comm_step();
		fake_missed_zc_detection(timestamp_hnsec);
		_state.flags |= FLAG_SYNC_RECOVERY;
		_state.immediate_desaturations++;
		if (_state.immediate_desaturations >= _params.zc_failures_max) {
			stop_now = true;
		}
		break;
	}
	case ZC_NOT_DETECTED:
	case ZC_FAILED: {
		if ((_state.flags & FLAG_SPINUP) && !(_state.flags & FLAG_SYNC_RECOVERY)) {
			engage_current_comm_step();
		} else {
			motor_pwm_set_freewheeling();
		}
		fake_missed_zc_detection(timestamp_hnsec);
		register_bad_step(&stop_now);
		_state.flags |= FLAG_SYNC_RECOVERY;
		break;
	}
	default: {
		assert(0);
		stop_now = true;
	}
	}

	if (stop_now) {
		stop_from_isr(); // No bounce no play
		return;
	}

	_state.zc_detection_result = ZC_NOT_DETECTED;
	_state.blank_time_deadline = timestamp_hnsec + _params.comm_blank_hnsec;

	prepare_zc_detector_for_next_step();
	motor_adc_enable_from_isr();
}

static void handle_detected_zc(uint64_t zc_timestamp)
{
	assert(zc_timestamp > _state.prev_zc_timestamp);   // Sanity check
	assert(zc_timestamp < _state.prev_zc_timestamp * 10);

	if (zc_timestamp < _state.prev_zc_timestamp)
		zc_timestamp = _state.prev_zc_timestamp;
	uint32_t new_comm_period = zc_timestamp - _state.prev_zc_timestamp;
	_state.prev_zc_timestamp = zc_timestamp;

	if (new_comm_period > _params.comm_period_max) {
	        new_comm_period = _params.comm_period_max;
	}

	if (_state.flags & FLAG_SYNC_RECOVERY) {
		/*
		 * TODO: Proper sync recovery:
		 * - Disable PWM
		 * - Forget current comm step number - we're out of sync, so the step number is likely to be wrong
		 * - Infer the current rotor position from the BEMF signals
		 * - Measure the comm period using two subsequent ZC events
		 * - Resume PWM
		 */
		_state.comm_period = new_comm_period;
		engage_current_comm_step();
	} else {
		// Benchmarking shows that these weights provide lowest RPM ripple
		_state.comm_period = (_state.comm_period * 3 + new_comm_period + 2) / 4;
	}

	_state.zc_detection_result = ZC_DETECTED;

	const uint32_t advance =
		_state.comm_period / 2 - TIMING_ADVANCE64(_state.comm_period, _params.timing_advance_deg64);

	motor_timer_set_absolute(zc_timestamp + advance - STEP_SWITCHING_DELAY_HNSEC);
	motor_adc_disable_from_isr();
}

static void update_input_voltage_current(const struct motor_adc_sample* sample)
{
	static const int ALPHA_RCPR = 7; // A power of two minus one (1, 3, 7)
	_state.input_voltage = LOWPASS(_state.input_voltage, sample->input_voltage, ALPHA_RCPR);
	_state.input_current = LOWPASS(_state.input_current, sample->input_current, ALPHA_RCPR);
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
	_state.neutral_voltage = (sample->phase_values[step->positive] + sample->phase_values[step->negative]) / 2;
}

static bool is_past_zc(const int bemf)
{
	const bool zc_polarity = _state.current_comm_step & 1;
	return (zc_polarity && (bemf >= 0)) || (!zc_polarity && (bemf <= 0));
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

	if (b == 0) {
		return; // Garbage in - garbage out
	}

	const int64_t slope = (LEAST_SQUARES_MULT * a + b / 2) / b;

	*out_slope = slope;
	*out_yintercept = (LEAST_SQUARES_MULT * sum_y - slope * sum_x + n / 2) / n;
}

static uint64_t solve_zc_approximation(void)
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
	if (_state.zc_bemf_samples_acquired == 3) {
		// Special case - 3 samples per comm period - ignore the first sample
		solve_least_squares(_state.zc_bemf_samples_acquired - 1, &data_x[1], &_state.zc_bemf_samples[1],
			&slope, &yintercept);
	} else {
		// General case - 2 or >=4 samples per comm period
		solve_least_squares(_state.zc_bemf_samples_acquired, data_x, _state.zc_bemf_samples,
			&slope, &yintercept);
	}

	const int x = (-yintercept + slope / 2) / slope; // Linear equation solved for x

	/*
	 * Diagnostic info
	 */
	_diag.zc_solution_slope = slope;
	_diag.zc_solution_yintercept = yintercept;
	_diag.zc_solution_num_samples = _state.zc_bemf_samples_acquired;
#if DEBUG_BUILD
	memcpy(_diag.zc_solution_samples, _state.zc_bemf_samples,
		_state.zc_bemf_samples_acquired * sizeof(_state.zc_bemf_samples[0]));
#endif

	const uint64_t zc_timestamp = _state.zc_bemf_timestamps[0] + x;

	/*
	 * Solution validation
	 * Implement an outlier detector? Check the solution covariance?
	 */
	const bool valid =
		(abs(x) <= (int)(_params.adc_sampling_period * _state.zc_bemf_samples_acquired * 2)) &&
		(zc_timestamp > _state.prev_zc_timestamp);

	if (!valid) {
		_diag.zc_solution_failures++;
		return 0;
	}
	return zc_timestamp;
}

void motor_adc_sample_callback(const struct motor_adc_sample* sample)
{
	const bool proceed =
		((_state.flags & FLAG_ACTIVE) != 0) &&
		(_state.zc_detection_result == ZC_NOT_DETECTED) &&
		(sample->timestamp >= _state.blank_time_deadline);

	if (!proceed) {
		if ((_state.flags & FLAG_ACTIVE) == 0) {
			motor_forced_rotation_detector_update_from_adc_callback(COMMUTATION_TABLE_FORWARD, sample);
		}
		return;
	}
	assert(_state.comm_table);

	/*
	 * Normalization against the neutral voltage
	 */
	update_neutral_voltage(sample);
	const struct motor_pwm_commutation_step* const step = _state.comm_table + _state.current_comm_step;
	const int bemf = sample->phase_values[step->floating] - _state.neutral_voltage;

	const bool past_zc = is_past_zc(bemf);

	/*
	 * BEMF/ZC validation
	 */
	const int bemf_threshold = _state.neutral_voltage * _params.bemf_valid_range_pct128 / 128;
	if (!past_zc && (abs(bemf) > bemf_threshold)) {
		_diag.bemf_samples_out_of_range++;
		_state.zc_bemf_samples_acquired = 0;
		_state.zc_bemf_samples_acquired_past_zc = 0;
		return;
	}

	if (past_zc && (_state.zc_bemf_samples_acquired == 0) && !(_state.flags & FLAG_SPINUP)) {
		_diag.bemf_samples_premature_zc++;
		/*
		 * BEMF signal may be affected by extreme saturation, which we can detect
		 * as BEMF readings on the wrong side of neutral voltage past 30 degrees since
		 * the last commutation. In this case we simply turn off the light and freewheel
		 * towards the next scheduled commutation.
		 * Usually, saturation occurs under low RPM with very high duty cycle
		 * (because of large W * sec), as in case of rapid acceleration or high constant load.
		 * In both cases the powerskipping naturally keeps the power within acceptable range for
		 * the given motor.
		 */
		const uint64_t deadline =
			_state.prev_zc_timestamp + _state.comm_period - _params.adc_sampling_period / 2;
		if (sample->timestamp >= deadline) {
			motor_pwm_set_freewheeling();
			_state.zc_detection_result = ZC_DESATURATION;
			_diag.desaturations++;
		}
		return;
	}

	/*
	 * Here the BEMF sample is considered to be valid, and can be added to the solution.
	 * Input voltage/current updates are synced with ZC - this way the noise can be reduced.
	 */
	add_bemf_sample(bemf, sample->timestamp);

	if (past_zc) {
		update_input_voltage_current(sample);
	}

	/*
	 * Is there enough samples collected?
	 */
	bool enough_samples = false;
	if (past_zc) {
		_state.zc_bemf_samples_acquired_past_zc++;

		enough_samples =
			(_state.zc_bemf_samples_acquired_past_zc >= _state.zc_bemf_samples_optimal_past_zc) &&
			(_state.zc_bemf_samples_acquired > 1);

		if (step->floating == 0) {
			TESTPAD_ZC_SET();
		}
	}

	if (!enough_samples) {
		return;
	}

	/*
	 * Find the exact ZC position using the collected samples
	 */
	const uint64_t zc_timestamp = solve_zc_approximation();

	TESTPAD_ZC_CLEAR();

	if (zc_timestamp == 0) {
		// Sorry Mario
		motor_pwm_set_freewheeling();
		_state.zc_detection_result = ZC_FAILED;
		return;
	}
	handle_detected_zc(zc_timestamp);
}

// --- End of hard real time code ---
#pragma GCC reset_options

int motor_rtctl_init(void)
{
	int ret = motor_pwm_init();
	if (ret) {
		return ret;
	}

	motor_timer_init();

	ret = motor_adc_init();
	if (ret) {
		return ret;
	}

	motor_forced_rotation_detector_init();

	configure();
	motor_rtctl_stop();
	return 0;
}

void motor_rtctl_confirm_initialization(void)
{
	assert(!_initialization_confirmed);
	_initialization_confirmed = true;
}

static void init_adc_filters(void)
{
	struct motor_adc_sample smpl;
	enum motor_pwm_phase_manip manip_cmd[3];

	// Low phase
	for (int i = 0 ; i < MOTOR_NUM_PHASES; i++) {
		manip_cmd[i] = MOTOR_PWM_MANIP_LOW;
	}
	motor_pwm_manip(manip_cmd);
	smpl = motor_adc_get_last_sample();
	const int low = (smpl.phase_values[0] + smpl.phase_values[1] + smpl.phase_values[2]) / 3;

	// High phase
	for (int i = 0 ; i < MOTOR_NUM_PHASES; i++) {
		manip_cmd[i] = MOTOR_PWM_MANIP_HIGH;
	}
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

static int spinup_sample_bemf(void)
{
	static uint64_t prev_timestamp = 0;

	struct motor_adc_sample sample;
	while (true) {
		sample = motor_adc_get_last_sample();
		if (sample.timestamp != prev_timestamp) {
			prev_timestamp = sample.timestamp;
			break;
		}
	}

	const struct motor_pwm_commutation_step* const step = _state.comm_table + _state.current_comm_step;
	_state.neutral_voltage = (sample.phase_values[0] + sample.phase_values[1] + sample.phase_values[2]) / 3;
	return sample.phase_values[step->floating] - _state.neutral_voltage;
}

static uint64_t spinup_wait_zc(const uint64_t step_deadline)
{
	const int min_samples_past_zc = (_state.comm_period / _params.adc_sampling_period) / 128 + 1;

	int num_samples_past_zc = 0;
	uint64_t zc_timestamp = 0;

	while (motor_timer_hnsec() <= step_deadline) {
		const bool past_zc = is_past_zc(spinup_sample_bemf());

		if (past_zc) {
			if (zc_timestamp == 0) {
				zc_timestamp = motor_timer_hnsec();
			}

			num_samples_past_zc++;
			if (num_samples_past_zc >= min_samples_past_zc) {
				break;
			}
		} else {
			zc_timestamp = 0;
			if (num_samples_past_zc > 0) {
				num_samples_past_zc--;
			}
		}
	}

	return zc_timestamp;
}

static bool do_bemf_spinup(const float max_duty_cycle)
{
	assert(chThdGetPriority() == HIGHPRIO);  // Mandatory

	// Make sure we're not going to underflow during time calculations
	while (motor_timer_hnsec() < _params.spinup_start_comm_period) {
		;
	}

	const uint64_t deadline = motor_timer_hnsec() + _params.spinup_timeout;
	float dc = _params.spinup_duty_cycle_increment;
	unsigned num_good_comms = 0;

	_state.comm_period = _params.spinup_start_comm_period;
	_state.prev_zc_timestamp = motor_timer_hnsec() - _state.comm_period / 2;
	_state.pwm_val = motor_pwm_compute_pwm_val(dc);

	while (motor_timer_hnsec() <= deadline) {
		// Engage the current comm step
		irq_primask_disable();
		motor_pwm_set_step_from_isr(_state.comm_table + _state.current_comm_step, _state.pwm_val);
		irq_primask_enable();

		uint64_t step_deadline = motor_timer_hnsec() + _state.comm_period;
		motor_timer_hndelay(_params.comm_blank_hnsec);

		// Wait for the next zero crossing
		const uint64_t zc_timestamp = spinup_wait_zc(step_deadline);
		num_good_comms = (zc_timestamp > 0) ? (num_good_comms + 1) : 0;

		// Compute the next duty cycle
		dc += _params.spinup_duty_cycle_increment;
		if (dc > max_duty_cycle) {
			dc = max_duty_cycle;
		}
		_state.pwm_val = motor_pwm_compute_pwm_val(dc);

		if (zc_timestamp > 0) {
			assert(zc_timestamp > _state.prev_zc_timestamp);

			// Update comm period
			const uint32_t new_comm_period = zc_timestamp - _state.prev_zc_timestamp;
			_state.prev_zc_timestamp = zc_timestamp;
			_state.comm_period = (_state.comm_period + new_comm_period) / 2;
			step_deadline = zc_timestamp + _state.comm_period / 2;

			// Check the termination condition
			const bool enough_good_comms = num_good_comms > _params.spinup_num_good_comms;
			const bool fast_enough = _state.comm_period <= _params.spinup_end_comm_period;
			if (enough_good_comms || fast_enough) {
				break;
			}
		} else {
			// If ZC was not detected, we need to fake the previous ZC timestamp now
			_state.prev_zc_timestamp = step_deadline - _state.comm_period / 2;
		}

		// Wait till the end of the current step
		while (motor_timer_hnsec() <= step_deadline) {
			;
		}

		// Next step
		_state.current_comm_step++;
		if (_state.current_comm_step >= MOTOR_NUM_COMMUTATION_STEPS) {
			_state.current_comm_step = 0;
		}
	}

	return _state.comm_period < _params.comm_period_max;
}

void motor_rtctl_start(float duty_cycle, bool reverse)
{
	motor_rtctl_stop();                    // Just in case

	if (!_initialization_confirmed) {
		return; // Go home you're drunk
	}

	if (duty_cycle <= 0) {
		assert(0);
		return;
	}

	/*
	 * Initialize the structs
	 */
	memset(&_diag, 0, sizeof(_diag));
	memset(&_state, 0, sizeof(_state));    // Mighty reset

	motor_forced_rotation_detector_reset();

	_diag.started_at = motor_timer_hnsec();

	_state.comm_table = reverse ? COMMUTATION_TABLE_REVERSE : COMMUTATION_TABLE_FORWARD;

	_state.pwm_val_after_spinup = motor_pwm_compute_pwm_val(duty_cycle);

	init_adc_filters();

	/*
	 * Initial spinup.
	 */
	const tprio_t orig_priority = chThdSetPriority(HIGHPRIO);

	motor_pwm_prepare_to_start();

	const bool started = do_bemf_spinup(duty_cycle);

	/*
	 * Engage the normal mode if started
	 * At this point, if the spinup was OK, we're sutiated RIGHT AFTER THE DETECTED ZERO CROSS, engaged.
	 */
	if (started) {
		_state.blank_time_deadline = motor_timer_hnsec() + _params.comm_blank_hnsec;
		_state.zc_detection_result = ZC_DETECTED;
		_state.flags = FLAG_ACTIVE | FLAG_SPINUP;
		motor_timer_set_relative(_state.comm_period / 3);
		lowsyslog("Motor: Spinup OK, comm period: %u usec\n", (unsigned)(_state.comm_period / HNSEC_PER_USEC));
	} else {
		lowsyslog("Motor: Spinup failed\n");
		motor_rtctl_stop();
	}

	chThdSetPriority(orig_priority);
}

void motor_rtctl_stop(void)
{
	_state.flags = 0;
	motor_timer_cancel();
	_state.flags = 0;

	irq_primask_disable();
	motor_adc_enable_from_isr(); // ADC should be enabled by default
	irq_primask_enable();

	motor_pwm_set_freewheeling();
}

void motor_rtctl_set_duty_cycle(float duty_cycle)
{
	// We don't need a critical section to write an integer
	_state.pwm_val = motor_pwm_compute_pwm_val(duty_cycle);

#if DEBUG_BUILD
	if (duty_cycle > _params.dc_testpad_threshold) {
		TESTPAD_SET(GPIO_PORT_TEST_A, GPIO_PIN_TEST_A);
	} else {
		TESTPAD_CLEAR(GPIO_PORT_TEST_A, GPIO_PIN_TEST_A);
	}
#endif
}

enum motor_rtctl_state motor_rtctl_get_state(void)
{
	volatile const unsigned flags = _state.flags;

	if (flags & FLAG_ACTIVE) {
		return (flags & FLAG_SPINUP) ? MOTOR_RTCTL_STATE_STARTING : MOTOR_RTCTL_STATE_RUNNING;
	} else {
		return MOTOR_RTCTL_STATE_IDLE;
	}
}

void motor_rtctl_beep(int frequency, int duration_msec)
{
	if (_state.flags & FLAG_ACTIVE) {
		return;
	}

	irq_primask_disable();
	motor_adc_disable_from_isr();
	irq_primask_enable();

	const tprio_t orig_priority = chThdSetPriority(HIGHPRIO);
	motor_pwm_beep(frequency, duration_msec);
	chThdSetPriority(orig_priority);

	irq_primask_disable();
	motor_adc_enable_from_isr();
	irq_primask_enable();

	/*
	 * Motor windings may get saturated after beeping, making immediately following spinup unreliable.
	 * This little delay fixes that, not in the best way though.
	 */
	usleep(10000);
}

uint32_t motor_rtctl_get_comm_period_hnsec(void)
{
	if (motor_rtctl_get_state() == MOTOR_RTCTL_STATE_IDLE) {
		return 0;
	}

	const uint32_t val = _state.comm_period;
	return val;
}

uint64_t motor_rtctl_get_zc_failures_since_start(void)
{
	// Atomic
	return _diag.zc_failures_since_start;
}

void motor_rtctl_emergency(void)
{
	const irqstate_t irqstate = irq_primask_save();
	{
		motor_pwm_emergency();
		_state.flags = 0;
		motor_timer_cancel();
	}
	irq_primask_restore(irqstate);
}

void motor_rtctl_get_input_voltage_current(float* out_voltage, float* out_current)
{
	int volt = 0, curr = 0;

	if (motor_rtctl_get_state() == MOTOR_RTCTL_STATE_IDLE) {
		const struct motor_adc_sample smpl = motor_adc_get_last_sample();
		volt = smpl.input_voltage;
		curr = smpl.input_current;
	} else {
		volt = _state.input_voltage;
		curr = _state.input_current;
	}

	if (out_voltage) {
		*out_voltage = motor_adc_convert_input_voltage(volt);
	}
	if (out_current) {
		*out_current = motor_adc_convert_input_current(curr);
	}
}

uint32_t motor_rtctl_get_min_comm_period_hnsec(void)
{
	// Ensure some number of ADC samples per comm period
	uint32_t retval = motor_adc_sampling_period_hnsec() * 5;
	if (retval < ABS_MIN_COMM_PERIOD_USEC * HNSEC_PER_USEC) {
		retval = ABS_MIN_COMM_PERIOD_USEC * HNSEC_PER_USEC;
	}
	return retval;
}

enum motor_rtctl_forced_rotation motor_rtctl_get_forced_rotation_state(void)
{
	if (motor_rtctl_get_state() == MOTOR_RTCTL_STATE_IDLE) {
		return motor_forced_rotation_detector_get_state();
	} else {
		return MOTOR_RTCTL_FORCED_ROT_NONE;
	}
}

void motor_rtctl_print_debug_info(void)
{
	static const int ALIGNMENT = 25;

#define PRINT_INT(name, value) lowsyslog("  %-*s %li\n", ALIGNMENT, (name), (long)(value))
#define PRINT_FLT(name, value) lowsyslog("  %-*s %f\n", ALIGNMENT, (name), (float)(value))

	/*
	 * Instant state
	 */
	irq_primask_disable();
	const struct control_state state_copy = _state;
	irq_primask_enable();

	lowsyslog("Motor RTCTL state\n");
	PRINT_INT("comm period",     state_copy.comm_period / HNSEC_PER_USEC);
	PRINT_INT("flags",           state_copy.flags);
	PRINT_INT("neutral voltage", state_copy.neutral_voltage);
	PRINT_INT("input voltage",   state_copy.input_voltage);
	PRINT_INT("input current",   state_copy.input_current);
	PRINT_INT("pwm val",         state_copy.pwm_val);

	/*
	 * Diagnostics
	 */
	irq_primask_disable();
	const struct diag_info diag_copy = _diag;
	irq_primask_enable();

	lowsyslog("Motor RTCTL diag\n");
	PRINT_INT("zc failures",       diag_copy.zc_failures_since_start);
	PRINT_INT("desaturations",     diag_copy.desaturations);
	PRINT_INT("bemf out of range", diag_copy.bemf_samples_out_of_range);
	PRINT_INT("bemf premature zc", diag_copy.bemf_samples_premature_zc);
	PRINT_INT("zc sol failures",   diag_copy.zc_solution_failures);
	PRINT_INT("zc sol num samples",diag_copy.zc_solution_num_samples);
	PRINT_FLT("zc sol slope",      diag_copy.zc_solution_slope / (float)LEAST_SQUARES_MULT);
	PRINT_FLT("zc sol yintercept", diag_copy.zc_solution_yintercept / (float)LEAST_SQUARES_MULT);

	/*
	 * ZC fitting
	 */
#if DEBUG_BUILD
	if (_diag.zc_solution_num_samples > 0) {
		lowsyslog("Motor ZC solution data\n");

		lowsyslog("  zc samples   ");
		for (int i = 0; i < _diag.zc_solution_num_samples; i++) {
			lowsyslog("%-5i ", diag_copy.zc_solution_samples[i]);
		}
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
