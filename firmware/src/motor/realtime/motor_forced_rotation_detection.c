/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
 *   Author: Pavel Kirienko <pavel.kirienko@gmail.com>
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

#include "forced_rotation_detection.h"
#include "internal.h"
#include "timer.h"
#include <zubax_chibios/config/config.h>
#include <string.h>


static struct state
{
	int direction_votes;                     ///< + forward, - reverse, 0 none
	int comm_step_index;
	uint64_t last_step_ts;
} _state;

static struct precomputed_params
{
	int bemf_threshold;
	int voting_threshold;
	uint64_t max_step_period;
} _params;


CONFIG_PARAM_INT("enum_bemf",    20,    5,     500)
CONFIG_PARAM_INT("enum_steps",   20,    6,     200)
CONFIG_PARAM_INT("enum_max_step",     50000, 2000,  100000)


void motor_forced_rotation_detector_init(void)
{
	_params.bemf_threshold  = config_get("enum_bemf");
	_params.voting_threshold = config_get("enum_steps");
	_params.max_step_period = config_get("enum_max_step") * HNSEC_PER_USEC;

	motor_forced_rotation_detector_reset();
}

void motor_forced_rotation_detector_reset(void)
{
	memset(&_state, 0, sizeof(_state));

	assert(_params.bemf_threshold > 0);
	assert(_params.voting_threshold > 0);
	assert(_params.max_step_period > 0);
}

static bool does_sample_match_step(const struct motor_adc_sample* sample,
                                   const struct motor_pwm_commutation_step* step)
{
	assert(sample != NULL && step != NULL);
	assert(step->negative >= 0 && step->negative < MOTOR_NUM_PHASES);
	assert(step->floating >= 0 && step->floating < MOTOR_NUM_PHASES);
	assert(step->positive >= 0 && step->positive < MOTOR_NUM_PHASES);

	const int thres = _params.bemf_threshold;
	return (sample->phase_values[step->floating] >= (sample->phase_values[step->negative] + thres)) &&
	       (sample->phase_values[step->positive] >= (sample->phase_values[step->floating] + thres));
}

static int find_matching_step_index(const struct motor_pwm_commutation_step* comm_table,
                                    const struct motor_adc_sample* adc_sample)
{
	int matching_step = -1;
	for (int i = 0; i < MOTOR_NUM_COMMUTATION_STEPS; i++) {
		if (does_sample_match_step(adc_sample, comm_table + i)) {
			matching_step = i;
			break;
		}
	}
	return matching_step;
}

static int normalize_comm_step_index(int step)
{
	while (step < 0) {
		step += MOTOR_NUM_COMMUTATION_STEPS;
	}
	while (step >= MOTOR_NUM_COMMUTATION_STEPS) {
		step -= MOTOR_NUM_COMMUTATION_STEPS;
	}
	return step;
}

static enum motor_rtctl_forced_rotation detect_rotation_direction_from_current_step(int step_index)
{
	if (step_index == normalize_comm_step_index(_state.comm_step_index + 1)) {
		return MOTOR_RTCTL_FORCED_ROT_FORWARD;
	} else if (step_index == normalize_comm_step_index(_state.comm_step_index - 1)) {
		return MOTOR_RTCTL_FORCED_ROT_REVERSE;
	} else {
		return MOTOR_RTCTL_FORCED_ROT_NONE;
	}
}

void motor_forced_rotation_detector_update_from_adc_callback(
	const struct motor_pwm_commutation_step comm_table[MOTOR_NUM_COMMUTATION_STEPS],
        const struct motor_adc_sample* adc_sample)
{
	assert(comm_table != NULL && adc_sample != NULL);

	const bool timed_out = (adc_sample->timestamp - _state.last_step_ts) > _params.max_step_period;
	if (timed_out) {
		_state.direction_votes = 0;
	}

	const int current_step_index = find_matching_step_index(comm_table, adc_sample);
	if (current_step_index < 0) {
		return;
	}

	const enum motor_rtctl_forced_rotation new_assumption =
		detect_rotation_direction_from_current_step(current_step_index);

	_state.comm_step_index = current_step_index;

	if (new_assumption == MOTOR_RTCTL_FORCED_ROT_NONE) {
		return;
	}

	_state.last_step_ts = adc_sample->timestamp;

	if (new_assumption == MOTOR_RTCTL_FORCED_ROT_FORWARD) {
		_state.direction_votes++;
		if (_state.direction_votes > _params.voting_threshold) {
			_state.direction_votes = _params.voting_threshold;
		}
	} else {
		_state.direction_votes--;
		if (_state.direction_votes < -_params.voting_threshold) {
			_state.direction_votes = -_params.voting_threshold;
		}
	}
}

enum motor_rtctl_forced_rotation motor_forced_rotation_detector_get_state(void)
{
	const int votes = _state.direction_votes;

	if (votes >= _params.voting_threshold) {
		return MOTOR_RTCTL_FORCED_ROT_FORWARD;
	} else if (votes <= -_params.voting_threshold) {
		return MOTOR_RTCTL_FORCED_ROT_REVERSE;
	} else {
		return MOTOR_RTCTL_FORCED_ROT_NONE;
	}
}
