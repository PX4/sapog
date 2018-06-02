/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum motor_control_mode
{
	MOTOR_CONTROL_MODE_OPENLOOP,
	MOTOR_CONTROL_MODE_RPM
};

enum motor_limit_mask
{
	MOTOR_LIMIT_RPM = 1,
	MOTOR_LIMIT_CURRENT = 2,
	MOTOR_LIMIT_ACCEL = 4
};

enum motor_forced_rotation_direction
{
	MOTOR_FORCED_ROTATION_NONE,
	MOTOR_FORCED_ROTATION_FORWARD,
	MOTOR_FORCED_ROTATION_REVERSE,
};

/**
 * @param current_shunt_resistance      resistance of the DC current shunt [ohm]
 *
 * @return negative on error, zero otherwise
 */
int motor_init(float current_shunt_resistance);

/**
 * Sets the duty cycle. Control mode will be OPENLOOP.
 * TTL is the amount of time to keep this setpoint before stopping the motor if no new setpoints were set.
 * @param [in] dc     Duty cycle [0.0; 1.0]
 * @param [in] ttl_ms TTL in milliseconds
 */
void motor_set_duty_cycle(float dc, int ttl_ms);

/**
 * Sets the RPM setpoint. Control mode will be RPM.
 * TTL is the amount of time to keep this setpoint before stopping the motor if no new setpoints were set.
 * @param [in] rpm    RPM setpoint
 * @param [in] ttl_ms TTL in milliseconds
 */
void motor_set_rpm(unsigned rpm, int ttl_ms);

/**
 * Returns current duty cycle.
 */
float motor_get_duty_cycle(void);

/**
 * Returns current RPM.
 */
unsigned motor_get_rpm(void);

void motor_stop(void);

enum motor_control_mode motor_get_control_mode(void);

/**
 * Returns the motor state.
 * @return True if the motor is running; false if starting or not running.
 */
bool motor_is_running(void);

/**
 * Returns the motor state.
 * @return True if the motor is idle; false if starting or running.
 */
bool motor_is_idle(void);

/**
 * Returns true if the motor controller has given up trying to start.
 * @return True if the motor controller is locked.
 */
bool motor_is_blocked(void);

/**
 * Returns the bitmask of currently active limits.
 */
int motor_get_limit_mask(void);

/**
 * Returns filtered input voltage and current.
 * @param [out] out_voltage Volts
 * @param [out] out_current Amperes
 */
void motor_get_input_voltage_current(float* out_voltage, float* out_current);

/**
 * Simple wrappers; refer to RTCTL API docs to learn more
 * @{
 */
void motor_confirm_initialization(void);
uint64_t motor_get_zc_failures_since_start(void);
enum motor_forced_rotation_direction motor_get_forced_rotation_direction(void);
int motor_test_hardware(void);
int motor_test_motor(void);
void motor_beep(int frequency, int duration_msec);
void motor_print_debug_info(void);
void motor_emergency(void);
void motor_execute_cli_command(int argc, const char* argv[]);
/**
 * @}
 */

#ifdef __cplusplus
}
#endif
