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

#include <stdint.h>
#include <stdbool.h>
#include "timer.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * High-level motor control state.
 */
enum motor_rtctl_state
{
	/**
	 * Unpowered.
	 * The motor can be rotated by an external force.
	 * Next state: starting.
	 */
	MOTOR_RTCTL_STATE_IDLE,

	/**
	 * Motor is attempting to start.
	 * Next state: running on success, idle on failure.
	 */
	MOTOR_RTCTL_STATE_STARTING,

	/**
	 * Motor is running in normal mode.
	 * Next state: idle.
	 */
	MOTOR_RTCTL_STATE_RUNNING
};

/**
 * Forced rotation detection.
 */
enum motor_rtctl_forced_rotation
{
	MOTOR_RTCTL_FORCED_ROT_NONE,
	MOTOR_RTCTL_FORCED_ROT_FORWARD,
	MOTOR_RTCTL_FORCED_ROT_REVERSE,
};

/**
 * Initialize the hardware and control logic
 * @return 0 on success, negative on error
 */
int motor_rtctl_init(void);

/**
 * Safety feature.
 * This function must be called when all the required software logic was initialized successfully;
 * otherwise the motor controller will refuse to start.
 */
void motor_rtctl_confirm_initialization(void);

/**
 * Start the motor.
 * @param [in] duty_cycle  PWM duty cycle that will be applied once the motor has started, (0; 1]
 * @param [in] reverse     Spin direction
 */
void motor_rtctl_start(float duty_cycle, bool reverse);

/**
 * Engage freewheeling.
 */
void motor_rtctl_stop(void);

/**
 * Configure PWM duty cycle
 * @param [in] duty_cycle PWM duty cycle [-1; 1], negative - braking, positive - forward
 */
void motor_rtctl_set_duty_cycle(float duty_cycle);

/**
 * Returns motor state.
 */
enum motor_rtctl_state motor_rtctl_get_state(void);

/**
 * Make noise.
 * Will work only if the motor is not running.
 */
void motor_rtctl_beep(int frequency, int duration_msec);

/**
 * Returns the commutation period if running, 0 if not.
 */
uint32_t motor_rtctl_get_comm_period_hnsec(void);

/**
 * Number of zero cross detection failures since the motor has started.
 */
uint64_t motor_rtctl_get_zc_failures_since_start(void);

/**
 * Emergency deactivation.
 * Can be executed from any context.
 */
void motor_rtctl_emergency(void);

/**
 * Returns input voltage and current.
 * If the motor is running, sampling is synchronized with ZC and lowpass filters are applied.
 * If the motor is not running, immediate values are taken.
 * Higher-order low pass filter should be applied to these values anyway.
 * @param [out] out_voltage Volts
 * @param [out] out_current Amperes
 */
void motor_rtctl_get_input_voltage_current(float* out_voltage, float* out_current);

/**
 * Minimum safe comm period. Depends on PWM frequency.
 */
uint32_t motor_rtctl_get_min_comm_period_hnsec(void);

/**
 * When the motor is not running, the control logic continuously monitors the BEMF
 * response in order to detect if the motor is being rotated by an external force.
 * This function allows to check if a forced rotation was detected.
 */
enum motor_rtctl_forced_rotation motor_rtctl_get_forced_rotation_state(void);

/**
 * Prints some debug info.
 * Shall never be called during normal operation because it can disrupt the control timings.
 */
void motor_rtctl_print_debug_info(void);

/**
 * Perform the ESC self test.
 * @return 0        - test OK,
 *         negative - unable to run the test at the current state,
 *         positive - test failed.
 */
int motor_rtctl_test_hardware(void);

/**
 * Test the connected motor (if any).
 * @return 0        - motor appears to be connected,
 *         negative - unable to run the test at the current state,
 *         positive - motor is not connected or went bananas.
 */
int motor_rtctl_test_motor(void);

/**
 * Current timestamp in hectonanoseconds since boot; never overflows.
 */
#define motor_rtctl_timestamp_hnsec() motor_timer_hnsec()

/**
 * Designed for debug purposes. May be removed later.
 */
void motor_rtctl_execute_cli_command(int argc, const char* argv[]);

#ifdef __cplusplus
}
#endif
