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

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <sys.h>

__BEGIN_DECLS

enum motor_state
{
	MOTOR_STATE_IDLE,
	MOTOR_STATE_STARTING,
	MOTOR_STATE_RUNNING
};

/**
 * Initialize the hardware and control logic
 * @return 0 on success, negative on error
 */
int motor_init(void);

/**
 * Start the motor.
 * This function returns immediately.
 */
void motor_start(uint16_t duty_cycle, bool reverse);

/**
 * Engage freewheeling.
 */
void motor_stop(void);

/**
 * Configure PWM duty cycle
 * @param [in] duty_cycle PWM duty cycle [0; 65536)
 * @return     True duty cycle, corrected with respect to the true resolution.
 */
uint16_t motor_set_duty_cycle(uint16_t duty_cycle);

/**
 * Returns motor state.
 */
enum motor_state motor_get_state(void);

/**
 * Make noise.
 */
void motor_beep(int frequency, int duration_msec);

/**
 * Magnetic field RPM. Can be used to compute the mechanical RPM.
 * @return Field RPM; 0 if the motor is not running.
 */
uint32_t motor_get_electrical_rpm(void);

/**
 * Number of zero cross detection failures since the motor has started.
 * The value stops incrementing at the maximum value, thus it never overflows.
 */
uint64_t motor_get_zc_failures_since_start(void);

/**
 * Perform the ESC self test.
 * @return 0        - test OK,
 *         negative - unable to run the test at the current state,
 *         positive - test failed.
 */
int motor_test_hardware(void);

/**
 * Test the connected motor (if any).
 * @return 0        - motor appears to be connected,
 *         negative - unable to run the test at the current state,
 *         positive - motor is not connected or went bananas.
 */
int motor_test_motor(void);

/**
 * Emergency deactivation.
 * Can be executed from any context.
 */
void motor_emergency(void);

/**
 * Debug only.
 */
void motor_print_debug_info(void);

__END_DECLS
