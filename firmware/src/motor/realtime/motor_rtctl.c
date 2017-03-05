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
#include "irq.h"
#include "forced_rotation_detection.h"
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <limits.h>
#include <zubax_chibios/config/config.h>


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

#define TESTPAD_ZC_SET()           TESTPAD_SET  (GPIO_PORT_TEST_ZC, GPIO_PIN_TEST_ZC)
#define TESTPAD_ZC_CLEAR()         TESTPAD_CLEAR(GPIO_PORT_TEST_ZC, GPIO_PIN_TEST_ZC)

#undef MIN
#undef MAX
#define MIN(a, b)                  (((a) < (b)) ? (a) : (b))
#define MAX(a, b)                  (((a) > (b)) ? (a) : (b))

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
	uint32_t zc_solution_extrapolation_discarded;
	uint32_t extra_bemf_samples_past_zc;
	uint32_t bemf_samples_out_of_range;
	uint32_t bemf_samples_premature_zc;
	uint32_t bemf_wrong_slope;
	uint32_t desaturations;
	uint32_t late_commutations;

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
	uint64_t prev_comm_timestamp;
	uint32_t comm_period;

	int current_comm_step;
	const struct motor_pwm_commutation_step* comm_table;

	unsigned immediate_zc_failures;
	unsigned immediate_zc_detects;
	unsigned immediate_desaturations;
	unsigned zc_detects_during_spinup;
	bool spinup_zc_slope_error;

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
	int timing_advance_min_deg64;
	int timing_advance_max_deg64;
	uint32_t max_comm_period_for_max_timing_advance;
	uint32_t max_comm_period_for_min_timing_advance;

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

	uint32_t adc_sampling_period;
} _params;

static bool _initialization_confirmed = false;

// Timing advance settings
CONFIG_PARAM_INT("mot_tim_adv_min",     5,     0,     20)
CONFIG_PARAM_INT("mot_tim_adv_max",     15,    0,     29)
CONFIG_PARAM_INT("mot_tim_cp_max",      300,   100,   50000)
CONFIG_PARAM_INT("mot_tim_cp_min",      600,   100,   50000)
// Most important parameters
CONFIG_PARAM_INT("mot_blank_usec",      40,    10,    100)
CONFIG_PARAM_INT("mot_bemf_win_den",    4,     3,     8)
CONFIG_PARAM_INT("mot_bemf_range",      90,    10,    100)
CONFIG_PARAM_INT("mot_zc_fails_max",    40,    6,     300)
CONFIG_PARAM_INT("mot_zc_dets_min",     200,   6,     1000)
CONFIG_PARAM_INT("mot_comm_per_max",    12000, 1000,  50000)
// Spinup settings
CONFIG_PARAM_INT("mot_spup_to_ms",      3000,  100,   9500)
CONFIG_PARAM_INT("mot_spup_st_cp",      50000, 10000, 200000)
CONFIG_PARAM_INT("mot_spup_en_cp",      1000,  500,   10000)
CONFIG_PARAM_INT("mot_spup_gcomms",     200,   6,     1000)


static void configure(void)
{
	_params.timing_advance_min_deg64               = configGet("mot_tim_adv_min") * 64 / 60;
	_params.timing_advance_max_deg64               = configGet("mot_tim_adv_max") * 64 / 60;
	_params.max_comm_period_for_max_timing_advance = configGet("mot_tim_cp_max") * HNSEC_PER_USEC;
	_params.max_comm_period_for_min_timing_advance = configGet("mot_tim_cp_min") * HNSEC_PER_USEC;

	_params.motor_bemf_window_len_denom = configGet("mot_bemf_win_den");
	_params.bemf_valid_range_pct128     = configGet("mot_bemf_range") * 128 / 100;
	_params.zc_failures_max  = configGet("mot_zc_fails_max");
	_params.zc_detects_min   = configGet("mot_zc_dets_min");
	_params.comm_period_max  = configGet("mot_comm_per_max") * HNSEC_PER_USEC;
	_params.comm_blank_hnsec = configGet("mot_blank_usec") * HNSEC_PER_USEC;

	_params.spinup_timeout              = configGet("mot_spup_to_ms") * HNSEC_PER_MSEC;
	_params.spinup_start_comm_period    = configGet("mot_spup_st_cp") * HNSEC_PER_USEC;
	_params.spinup_end_comm_period      = configGet("mot_spup_en_cp") * HNSEC_PER_USEC;
	_params.spinup_num_good_comms       = configGet("mot_spup_gcomms");

	/*
	 * Validation
	 */
	if (_params.timing_advance_min_deg64 > _params.timing_advance_max_deg64) {
		_params.timing_advance_min_deg64 = _params.timing_advance_max_deg64;  // Minimizing
	}
	assert(_params.timing_advance_min_deg64 <= _params.timing_advance_max_deg64);

	if (_params.max_comm_period_for_max_timing_advance > _params.max_comm_period_for_min_timing_advance) {
		_params.max_comm_period_for_max_timing_advance = _params.max_comm_period_for_min_timing_advance; // Min
	}
	assert(_params.max_comm_period_for_max_timing_advance <= _params.max_comm_period_for_min_timing_advance);

	if (_params.comm_period_max > motor_timer_get_max_delay_hnsec()) {
		_params.comm_period_max = motor_timer_get_max_delay_hnsec();
	}

	if (_params.spinup_end_comm_period > _params.comm_period_max) {
		_params.spinup_end_comm_period = _params.comm_period_max;
	}

	_params.adc_sampling_period = motor_adc_sampling_period_hnsec();

	printf("Motor: RTCTL config: Max comm period: %u usec, BEMF window denom: %i\n",
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

static inline int get_effective_timing_advance_deg64(void)
{
	/*
	 * Handling extremes
	 */
	if (_state.comm_period <= _params.max_comm_period_for_max_timing_advance) {
		return _params.timing_advance_max_deg64;
	}

	if (_state.comm_period >= _params.max_comm_period_for_min_timing_advance) {
		return _params.timing_advance_min_deg64;
	}

	if (_state.flags & (FLAG_SPINUP | FLAG_SYNC_RECOVERY)) {
		return _params.timing_advance_min_deg64;
	}

	/*
	 * Linear interpolation
	 */
	const int tim_delta = _params.timing_advance_max_deg64 - _params.timing_advance_min_deg64;

	assert(_params.max_comm_period_for_min_timing_advance > _params.max_comm_period_for_max_timing_advance);
	assert(tim_delta >= 0);

	const int result = _params.timing_advance_max_deg64 -
	       (tim_delta * (int64_t)(_state.comm_period - _params.max_comm_period_for_max_timing_advance) /
	       (_params.max_comm_period_for_min_timing_advance - _params.max_comm_period_for_max_timing_advance));

	assert(result >= _params.timing_advance_min_deg64);
	assert(result <= _params.timing_advance_max_deg64);
	return result;
}

static void fake_missed_zc_detection(uint64_t timestamp_hnsec)
{
	const uint32_t leeway = _state.comm_period / 2 +
		TIMING_ADVANCE64(_state.comm_period, get_effective_timing_advance_deg64());

	_state.prev_zc_timestamp = timestamp_hnsec - leeway;
}

static void prepare_zc_detector_for_next_step(void)
{
	_state.zc_bemf_samples_acquired = 0;
	_state.zc_bemf_samples_acquired_past_zc = 0;

	const int advance = get_effective_timing_advance_deg64();

	/*
	 * Actual length of the BEMF sampling window depends on the advance angle.
	 * E.g. advance 15 deg makes the sampling window twice shorter after ZC,
	 * thus the denom increases by the same amount.
	 */
	const int denom = _params.motor_bemf_window_len_denom + _params.motor_bemf_window_len_denom * advance / 16;

	_state.zc_bemf_samples_optimal = (_state.comm_period / _params.adc_sampling_period) / denom + 2;

	if (_state.zc_bemf_samples_optimal > MAX_BEMF_SAMPLES) {
		_state.zc_bemf_samples_optimal = MAX_BEMF_SAMPLES;
	}

	// Number of samples past ZC should reduce proportionally to the advance angle.
	_state.zc_bemf_samples_optimal_past_zc = _state.zc_bemf_samples_optimal * (32 - advance) / 64;
}

void motor_timer_callback(uint64_t timestamp_hnsec)
{
	TESTPAD_SET(GPIO_PORT_TEST_A, GPIO_PIN_TEST_A);

	if (!(_state.flags & FLAG_ACTIVE)) {
		return;
	}

	if ((_state.flags & FLAG_SPINUP) == 0) {
		motor_timer_set_relative(_state.comm_period);
	} else {
		// This hack allows the algorithm to automatically lower the comm period when it skips a zero cross
		motor_timer_set_relative((_params.spinup_start_comm_period + _state.comm_period) / 2);
	}

	// Next comm step
	_state.prev_comm_timestamp = timestamp_hnsec;
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
		if (_state.flags & FLAG_SPINUP) {
			engage_current_comm_step();
		} else {
			motor_pwm_set_freewheeling();
			_state.flags |= FLAG_SYNC_RECOVERY;
		}
		fake_missed_zc_detection(timestamp_hnsec);
		register_bad_step(&stop_now);
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

	TESTPAD_CLEAR(GPIO_PORT_TEST_A, GPIO_PIN_TEST_A);
}

static void handle_detected_zc(uint64_t zc_timestamp)
{
	assert(zc_timestamp > _state.prev_zc_timestamp);   // Sanity check
	assert(zc_timestamp < _state.prev_zc_timestamp * 10);

	if (_state.flags & FLAG_SYNC_RECOVERY) {
		/*
		 * TODO: Proper sync recovery:
		 * - Disable PWM
		 * - Forget current comm step number - we're out of sync, so the step number is likely to be wrong
		 * - Infer the current rotor position from the BEMF signals
		 * - Measure the comm period using two subsequent ZC events
		 * - Resume PWM
		 */
		_state.comm_period = zc_timestamp - _state.prev_zc_timestamp;
		engage_current_comm_step();
	} else if (_state.flags & FLAG_SPINUP) {
		_state.comm_period = (zc_timestamp - _state.prev_comm_timestamp) * 2;

		if (_state.spinup_zc_slope_error) {
			_state.spinup_zc_slope_error = false;
			_state.comm_period = _state.comm_period / 2;
		}
	} else {
		const uint64_t predicted_zc_ts = _state.prev_zc_timestamp + _state.comm_period;
		zc_timestamp = (predicted_zc_ts + zc_timestamp + 2ULL) / 2ULL;

		_state.comm_period = zc_timestamp - _state.prev_zc_timestamp;
	}

	_state.prev_zc_timestamp = zc_timestamp;

	if ((_state.flags & FLAG_SPINUP) == 0) {
		_state.comm_period = MIN(_state.comm_period, _params.comm_period_max);
	} else {
		_state.comm_period = MIN(_state.comm_period, _params.spinup_start_comm_period);
	}

	_state.zc_detection_result = ZC_DETECTED;

	const uint32_t advance =
		_state.comm_period / 2 - TIMING_ADVANCE64(_state.comm_period, get_effective_timing_advance_deg64());

	const int64_t delta = motor_timer_set_absolute(zc_timestamp + advance - STEP_SWITCHING_DELAY_HNSEC);
	if (delta <= 0) {
		// The commutation event is already in the past; record an error.
		_diag.late_commutations++;
	}

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
		/*
		 * It is very important that we do not collect more samples than zc_bemf_samples_optimal!
		 * Exceeding this value may cause problems, because first few samples may be distorted by saturation
		 * currents. Limiting the number of samples to the optimum (computed in the function
		 * prepare_zc_detector_for_next_step()) allows us to push the early samples out of the
		 * buffer as newer samples arrive.
		 */
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

// Returns TRUE if the BEMF has POSITIVE slope, otherwise returns FALSE.
static bool is_bemf_slope_positive(void)
{
	return (_state.current_comm_step & 1) != 0;
}

static bool is_past_zc(const int bemf)
{
	const bool bemf_slope_positive = is_bemf_slope_positive();
	return (bemf_slope_positive && (bemf >= 0)) || (!bemf_slope_positive && (bemf <= 0));
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
	solve_least_squares(_state.zc_bemf_samples_acquired, data_x, _state.zc_bemf_samples, &slope, &yintercept);

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
	bool valid = zc_timestamp > _state.prev_zc_timestamp;

	if (valid) {
		if ((_state.flags & FLAG_SPINUP) == 0) {
			if (_state.zc_bemf_samples_acquired_past_zc > 0) {
				// In normal mode, expect the solution close to the first sample
				valid = abs(x) <=
				        (int)(_params.adc_sampling_period * _state.zc_bemf_samples_acquired * 2);
			} else {
				/*
				 * In extrapolation mode, expect the solution strictly ahead of the first sample.
				 * Note that we must not invalidate the solution in extrapolation mode if it is
				 * too far in the future, because it would break a special case when the obtained
				 * samples happen to be on the part of the BEMF curve when it is almost at zero
				 * slope, which happens typically in the beginning of the commutation step (up to
				 * the zero cross under high saturation). In the case of near zero slope, the
				 * calling code will detect that the solution is too far in the future, and it
				 * will just continue collecting more samples, which will naturally resolve the
				 * zero slope problem.
				 */
				valid = x > 0;
			}
		} else {
			// Special case for spinup mode: first and last samples should be on the opposite sides of
			// the neutral
			if (is_bemf_slope_positive()) {
				valid = (slope > 0) &&
				        (_state.zc_bemf_samples[0] < 0) &&
				        (_state.zc_bemf_samples[_state.zc_bemf_samples_acquired - 1] > 0);
			} else {
				valid = (slope < 0) &&
				        (_state.zc_bemf_samples[0] > 0) &&
				        (_state.zc_bemf_samples[_state.zc_bemf_samples_acquired - 1] < 0);
			}
		}
	}

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
	 * Checking if BEMF goes in the right direction.
	 * This check is only performed for the first sample.
	 */
	if (_state.zc_bemf_samples_acquired == 1) {
		bool correct_slope = false;
		if (is_bemf_slope_positive()) {
			correct_slope = bemf > _state.zc_bemf_samples[0];
		} else {
			correct_slope = bemf < _state.zc_bemf_samples[0];
		}

		if (!correct_slope) {
			_diag.bemf_wrong_slope++;
			// Discarding all samples, they are invalid
			_state.zc_bemf_samples_acquired = 0;
			// Continuing regardless - this new sample could be correct
		}
	}

	/*
	 * Here the BEMF sample is considered to be valid, and can be added to the solution.
	 */
	add_bemf_sample(bemf, sample->timestamp);

	if (past_zc) {
		_state.zc_bemf_samples_acquired_past_zc++;

		if (_state.zc_bemf_samples_acquired_past_zc > _state.zc_bemf_samples_optimal_past_zc) {
			// This may indicate that we're trying to cramp too many samples in the commutation period!
			_diag.extra_bemf_samples_past_zc++;
		}
	}

	/*
	 * Deciding if we have enough data to resolve the ZC timestamp.
	 */
	if ((_state.zc_bemf_samples_acquired_past_zc < _state.zc_bemf_samples_optimal_past_zc) ||
	    (_state.zc_bemf_samples_acquired < 2)) {
		// We don't update voltage/current in the same cycle where we solve the ZC approximation,
		// in order to distribute the IRQ load more evenly.
		update_input_voltage_current(sample);
		return;
	}

	/*
	 * Find the exact ZC timestamp using the collected samples
	 */
	//TESTPAD_SET(GPIO_PORT_TEST_A, GPIO_PIN_TEST_A);
	const uint64_t zc_timestamp = solve_zc_approximation();
	//TESTPAD_CLEAR(GPIO_PORT_TEST_A, GPIO_PIN_TEST_A);

	if (zc_timestamp == 0) {
		if ((_state.flags & FLAG_SPINUP) == 0) {
			// Abort only if there's no chance to get more data
			if (_state.zc_bemf_samples_acquired >= _state.zc_bemf_samples_optimal) {
				// Sorry Mario
				motor_pwm_set_freewheeling();
				_state.zc_detection_result = ZC_FAILED;
			}
			// Otherwise just exit and try again with the next sample
		} else {
			_state.spinup_zc_slope_error = true;
			// Just keep going
		}
		return;
	}

	if (zc_timestamp > (sample->timestamp + _params.adc_sampling_period * 2)) {
		/*
		 * We will have at least one more ADC sample before the projected ZC, let's procrastinate one more
		 * cycle. Note that we also add some extra time to ensure that there will be enough time to process
		 * the next sample and arm the step switching timer afterwards.
		 */
		_diag.zc_solution_extrapolation_discarded++;
		return;
	}

	TESTPAD_ZC_SET();
	handle_detected_zc(zc_timestamp);
	TESTPAD_ZC_CLEAR();
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

void motor_rtctl_start(float duty_cycle, bool reverse, unsigned num_prior_attempts)
{
	(void) num_prior_attempts;

	motor_rtctl_stop();                    // Just in case

	if (!_initialization_confirmed) {
		return;
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

	init_adc_filters();

	/*
	 * Start the background IRQ-driven process
	 */
	motor_pwm_prepare_to_start();

	chSysSuspend();

	_diag.started_at = motor_timer_hnsec();

	_state.pwm_val_after_spinup = motor_pwm_compute_pwm_val(duty_cycle);
	_state.pwm_val              = motor_pwm_compute_pwm_val(duty_cycle);  // Same value

	_state.comm_table = reverse ? COMMUTATION_TABLE_REVERSE : COMMUTATION_TABLE_FORWARD;
	_state.comm_period = _params.spinup_start_comm_period;

	_state.prev_comm_timestamp = motor_timer_hnsec();
	_state.prev_zc_timestamp = _state.prev_comm_timestamp - _state.comm_period / 2;
	_state.zc_detection_result = ZC_DETECTED;
	_state.flags = FLAG_ACTIVE | FLAG_SPINUP;

	motor_timer_set_relative(_state.comm_period / 2);

	chSysEnable();
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

#define PRINT_INT(name, value) printf("  %-*s %li\n", ALIGNMENT, (name), (long)(value))
#define PRINT_FLT(name, value) printf("  %-*s %f\n", ALIGNMENT, (name), (float)(value))

	/*
	 * Instant state
	 */
	irq_primask_disable();
	const struct control_state state_copy = _state;
	irq_primask_enable();

	irq_primask_disable();
	const int timing_advance_deg = (get_effective_timing_advance_deg64() + 1) * 60 / 64;  // Rounding
	irq_primask_enable();

	printf("Motor RTCTL state\n");
	PRINT_INT("comm period",     state_copy.comm_period / HNSEC_PER_USEC);
	PRINT_INT("flags",           state_copy.flags);
	PRINT_INT("neutral voltage", state_copy.neutral_voltage);
	PRINT_INT("input voltage",   state_copy.input_voltage);
	PRINT_INT("input current",   state_copy.input_current);
	PRINT_INT("pwm val",         state_copy.pwm_val);
	PRINT_INT("bemf opt",        state_copy.zc_bemf_samples_optimal);
	PRINT_INT("bemf opt past zc",state_copy.zc_bemf_samples_optimal_past_zc);
	PRINT_INT("timing adv deg",  timing_advance_deg);

	/*
	 * Diagnostics
	 */
	irq_primask_disable();
	const struct diag_info diag_copy = _diag;
	irq_primask_enable();

	printf("Motor RTCTL diag\n");
	PRINT_INT("zc failures",       diag_copy.zc_failures_since_start);
	PRINT_INT("desaturations",     diag_copy.desaturations);
	PRINT_INT("late commutations", diag_copy.late_commutations);
	PRINT_INT("bemf out of range", diag_copy.bemf_samples_out_of_range);
	PRINT_INT("bemf premature zc", diag_copy.bemf_samples_premature_zc);
	PRINT_INT("bemf extra past zc",diag_copy.extra_bemf_samples_past_zc);
	PRINT_INT("bemf wrong slope",  diag_copy.bemf_wrong_slope);
	PRINT_INT("zc sol failures",   diag_copy.zc_solution_failures);
	PRINT_INT("zc sol extrpl disc",diag_copy.zc_solution_extrapolation_discarded);
	PRINT_INT("zc sol num samples",diag_copy.zc_solution_num_samples);
	PRINT_FLT("zc sol slope",      diag_copy.zc_solution_slope / (float)LEAST_SQUARES_MULT);
	PRINT_FLT("zc sol yintercept", diag_copy.zc_solution_yintercept / (float)LEAST_SQUARES_MULT);

	/*
	 * ZC fitting
	 */
#if DEBUG_BUILD
	if (_diag.zc_solution_num_samples > 0) {
		printf("Motor ZC solution data\n");

		printf("  zc samples   ");
		for (int i = 0; i < _diag.zc_solution_num_samples; i++) {
			printf("%-5i ", diag_copy.zc_solution_samples[i]);
		}
		printf("\n");

		printf("  zc fitted    ");
		for (int i = 0; i < _diag.zc_solution_num_samples; i++) {
			const int x = _params.adc_sampling_period * i;
			const int y = (diag_copy.zc_solution_slope * x + diag_copy.zc_solution_yintercept) /
				LEAST_SQUARES_MULT;
			printf("%-5i ", y);
		}
		printf("\n");
	}
#endif

#undef PRINT_INT
#undef PRINT_FLT
}
