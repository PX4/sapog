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

#include <assert.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "pwm.h"
#include "adc.h"
#include "test.h"

#define NUM_PHASES  3

static const int INITIAL_DELAY_MS = 500;
static const int SAMPLE_DELAY_MS  = 10;

static const int ANALOG_TOLERANCE_PERCENT = 5;


static int test_one_phase(int phase, bool level)
{
	assert(phase >= 0 && phase < NUM_PHASES);

	enum motor_pwm_phase_manip manip_cmd[3] = {
		MOTOR_PWM_MANIP_FLOATING,
		MOTOR_PWM_MANIP_FLOATING,
		MOTOR_PWM_MANIP_FLOATING
	};

	manip_cmd[phase] = level ? MOTOR_PWM_MANIP_HIGH : MOTOR_PWM_MANIP_LOW;
	motor_pwm_manip(manip_cmd);
	usleep(SAMPLE_DELAY_MS * 1000);
	const int sample = motor_adc_get_last_sample().raw_phase_values[phase];

	manip_cmd[phase] = MOTOR_PWM_MANIP_FLOATING;
	motor_pwm_manip(manip_cmd);
	return sample;
}

static int compare_samples(const void* p1, const void* p2)
{
    return (*(const int*)p1 - *(const int*)p2);
}

int motor_test_test_power_stage(void)
{
	int result = 0;
	int high_samples[NUM_PHASES];
	memset(high_samples, 0, sizeof(high_samples));

	const int threshold = ((1 << MOTOR_ADC_RESOLUTION) * ANALOG_TOLERANCE_PERCENT) / 100;

	motor_pwm_set_freewheeling();
	usleep(INITIAL_DELAY_MS * 1000);

	/*
	 * Test phases at low level; collect high level readings
	 */
	for (int phase = 0; phase < NUM_PHASES; phase++) {
		// Low level
		const int low = test_one_phase(phase, false);
		if (low > threshold) {
			lowsyslog("Motor: Selftest FAILURE at phase %i: low sample %i is above threshold %i\n",
			          phase, low, threshold);
			result++;
		}

		// High level
		const int high = test_one_phase(phase, true);
		high_samples[phase] = high;
		if (high < threshold) {
			lowsyslog("Motor: Selftest FAILURE at phase %i: high sample %i is below threshold %i\n",
			          phase, high, threshold);
			result++;
		}
		// It is not possible to check against the high threshold directly
		// because its value will depend on the supply voltage

		lowsyslog("Motor: Selftest phase %i: low %i, high %i\n", phase, low, high);
	}

	/*
	 * Make sure that the high level readings are nearly identical
	 */
	int high_samples_sorted[NUM_PHASES];
	memcpy(high_samples_sorted, high_samples, sizeof(high_samples));
	qsort(high_samples_sorted, NUM_PHASES, sizeof(int), compare_samples);
	const int high_median = high_samples_sorted[NUM_PHASES / 2];

	for (int phase = 0; phase < NUM_PHASES; phase++) {
		if (abs(high_samples[phase] - high_median) > threshold) {
			lowsyslog("Motor: Selftest FAILURE at phase %i: sample %i is far from median %i\n",
			          phase, high_samples[phase], high_median);
			result++;
		}
	}

	motor_pwm_set_freewheeling();
	return result;
}

int motor_test_test_motor(void) // What an awkward name.
{
	const int threshold = ((1 << MOTOR_ADC_RESOLUTION) * ANALOG_TOLERANCE_PERCENT) / 100;
	struct motor_adc_sample sample;
	int result = 0;
	enum motor_pwm_phase_manip manip_cmd[3] = {
		MOTOR_PWM_MANIP_LOW,
		MOTOR_PWM_MANIP_FLOATING,
		MOTOR_PWM_MANIP_FLOATING
	};

	motor_pwm_set_freewheeling();
	/*
	 * Test with low level
	 */
	manip_cmd[0] = MOTOR_PWM_MANIP_LOW;
	motor_pwm_manip(manip_cmd);
	usleep(SAMPLE_DELAY_MS * 1000);
	sample = motor_adc_get_last_sample();

	if (sample.raw_phase_values[1] > threshold || sample.raw_phase_values[2] > threshold)
		result++;

	/*
	 * Test with high level
	 */
	manip_cmd[0] = MOTOR_PWM_MANIP_HIGH;
	motor_pwm_manip(manip_cmd);
	usleep(SAMPLE_DELAY_MS * 1000);
	sample = motor_adc_get_last_sample();

	if (abs(sample.raw_phase_values[0] - sample.raw_phase_values[1]) > threshold)
		result++;

	if (abs(sample.raw_phase_values[0] - sample.raw_phase_values[2]) > threshold)
		result++;

	motor_pwm_set_freewheeling();
	return result;
}
