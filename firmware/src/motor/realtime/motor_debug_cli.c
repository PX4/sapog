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

#include "api.h"
#include "adc.h"
#include "pwm.h"
#include "irq.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static enum motor_pwm_phase_manip arg_to_pwm_manip(const char* arg)
{
	if (arg[0] == '+') {
		return MOTOR_PWM_MANIP_HIGH;
	} else if (arg[0] == '-') {
		return MOTOR_PWM_MANIP_LOW;
	} else if (arg[0] == '/') {
		return MOTOR_PWM_MANIP_HALF;
	} else {
		return MOTOR_PWM_MANIP_FLOATING;
	}
}

void motor_rtctl_execute_cli_command(int argc, const char* argv[])
{
	if (argc < 0 || argv == NULL) {
		printf("Invalid args\n");
		return;
	}

	const struct motor_adc_sample adc_sample = motor_adc_get_last_sample();

	printf("ADC raw phases:  %i  %i  %i\n",
		adc_sample.phase_values[0], adc_sample.phase_values[1], adc_sample.phase_values[2]);

	printf("ADC raw vtg/cur: V=%i  I=%i\n", adc_sample.input_voltage, adc_sample.input_current);

	if ((argc > 0) && !strcmp("step", argv[0])) {  // step <dc> <pos> <neg>
		// Step manually with the configured duty cycle
		if (argc != 4) {
			printf("Usage: step <dc> <pos> <neg>\n");
			return;
		}

		const float dc = atof(argv[1]);
		if ((dc < -1.0F) || (dc > 1.0F)) {
			puts("ERROR: Invalid duty cycle");
			return;
		}

		struct motor_pwm_commutation_step step;
		memset(&step, 0, sizeof(step));
		step.positive = atoi(argv[2]);
		step.negative = atoi(argv[3]);

		if ((step.positive < 0) || (step.positive > 2) ||
		    (step.negative < 0) || (step.negative > 2)) {
			puts("ERROR: Invalid phase index");
			return;
		}

		if (step.positive == step.negative) {
			puts("ERROR: Phase polarity collision: pos == neg");
			return;
		}

		// We have verified above that pos != neg and that both pos and neg are in {0, 1, 2}.
		for (int i = 0; i < 3; i++) {
			if ((i != step.positive) && (i != step.negative)) {
				step.floating = i;
				break;
			}
		}

		const int pwm_val = motor_pwm_compute_pwm_val(dc);

		printf("PWM val: %d, pos: %d, neg: %d, flt: %d\n",
		       pwm_val, step.positive, step.negative, step.floating);

		motor_pwm_set_freewheeling();    // Pre-reset is required

		irq_primask_disable();
		motor_pwm_set_step_from_isr(&step, pwm_val);
		irq_primask_enable();

	} else if ((argc > 0) && !strcmp("2q", argv[0])) {
		// Configure 2 quadrant mode
		const bool enable = (argc > 1) ? (atoi(argv[1]) != 0) : false;
		printf("2Q mode: %s\n", enable ? "ON" : "OFF");
		motor_pwm_set_2_quadrant_mode(enable);

	} else if ((argc >= 1) && (argc <= 3)) {
		const enum motor_pwm_phase_manip manip_cmd[MOTOR_NUM_PHASES] = {
			arg_to_pwm_manip(argv[0]),
			(argc > 1) ? arg_to_pwm_manip(argv[1]) : MOTOR_PWM_MANIP_FLOATING,
			(argc > 2) ? arg_to_pwm_manip(argv[2]) : MOTOR_PWM_MANIP_FLOATING
		};
		printf("Manip %i %i %i\n", (int)manip_cmd[0], (int)manip_cmd[1], (int)manip_cmd[2]);
		motor_pwm_manip(manip_cmd);

	} else {
		motor_pwm_set_freewheeling();
		printf("Freewheeling\n");
	}
}
