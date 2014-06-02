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

#include <math.h>
#include <ch.h>
#include <watchdog.h>
#include <config/config.h>
#include "motor.h"
#include "rpmctl.h"
#include "realtime/api.h"

#define IDLE_CONTROL_PERIOD_MSEC  10
#define WATCHDOG_TIMEOUT_MSEC     1000

#define MIN_VALID_INPUT_VOLTAGE 4.0
#define MAX_VALID_INPUT_VOLTAGE 40.0


static unsigned comm_period_to_rpm(uint32_t comm_period);


//static int _watchdog_id;
static Mutex _mutex;
static EVENTSOURCE_DECL(_setpoint_update_event);
static WORKING_AREA(_wa_control_thread, 1024);

/*
 * TODO: Current implementation is a mess.
 * Maybe it should be redesigned from scratch as a nice FSM.
 */
static struct state
{
	enum motor_control_mode mode;
	int limit_mask;

	float dc_actual;
	float dc_openloop_setpoint;

	unsigned rpm_setpoint;

	int setpoint_ttl_ms;
	int num_unexpected_stops;

	float input_voltage;
	float input_current;
	float input_curent_offset;

	enum motor_rtctl_state rtctl_state;
} _state;

static struct params
{
	float dc_min_voltage;
	float dc_step_max;
	float dc_slope;

	int poles;
	bool reverse;

	uint32_t comm_period_limit;
	unsigned rpm_max;
	unsigned rpm_min;

	float current_limit;
	float current_limit_p;

	float voltage_current_lowpass_tau;
	int num_unexpected_stops_to_latch;
} _params;


CONFIG_PARAM_FLOAT("motor_dc_min_voltage",         1.1,    0.5,     10.0)
CONFIG_PARAM_FLOAT("motor_dc_step_max",            0.1,    0.001,   0.5)
CONFIG_PARAM_FLOAT("motor_dc_slope",               5.0,    0.1,     20.0)

CONFIG_PARAM_INT("motor_num_poles",                14,     2,       100)
CONFIG_PARAM_BOOL("motor_reverse",                 false)

CONFIG_PARAM_INT("motor_rpm_min",                  1000,   50,      5000)

CONFIG_PARAM_FLOAT("motor_current_limit",          20.0,   1.0,     60.0)
CONFIG_PARAM_FLOAT("motor_current_limit_p",        0.2,    0.01,    2.0)

CONFIG_PARAM_FLOAT("motor_volt_curr_lowpass_freq", 20.0,   1.0,     200.0)
CONFIG_PARAM_INT("motor_num_halts_to_latch",       7,      1,       100)


static void configure(void)
{
	_params.dc_min_voltage = config_get("motor_dc_min_voltage");
	_params.dc_step_max    = config_get("motor_dc_step_max");
	_params.dc_slope       = config_get("motor_dc_slope");

	_params.poles = config_get("motor_num_poles");
	_params.reverse = config_get("motor_reverse");

	_params.comm_period_limit = motor_rtctl_get_min_comm_period_hnsec();
	_params.rpm_max = comm_period_to_rpm(_params.comm_period_limit);
	_params.rpm_min = config_get("motor_rpm_min");

	_params.current_limit = config_get("motor_current_limit");
	_params.current_limit_p = config_get("motor_current_limit_p");

	_params.voltage_current_lowpass_tau = 1.0f / config_get("motor_volt_curr_lowpass_freq");
	_params.num_unexpected_stops_to_latch = config_get("motor_num_halts_to_latch");

	lowsyslog("Motor: RPM range: [%u, %u]; poles: %i\n", _params.rpm_min, _params.rpm_max, _params.poles);
}

static float lowpass(float xold, float xnew, float tau, float dt)
{
	return (dt * xnew + tau * xold) / (dt + tau);
}

static void init_filters(void)
{
	// Assuming that initial current is zero
	motor_rtctl_get_input_voltage_current(&_state.input_voltage, &_state.input_curent_offset);
	_state.input_current = 0.0f;
}

static void update_filters(float dt)
{
	float voltage = 0, current = 0;
	motor_rtctl_get_input_voltage_current(&voltage, &current);

	if (motor_rtctl_get_state() == MOTOR_RTCTL_STATE_IDLE) {
		// Current sensor offset calibration, corner frequency is much lower.
		const float offset_tau = _params.voltage_current_lowpass_tau * 100;
		_state.input_curent_offset = lowpass(_state.input_curent_offset, current, offset_tau, dt);
	}

	current -= _state.input_curent_offset;

	_state.input_voltage = lowpass(_state.input_voltage, voltage, _params.voltage_current_lowpass_tau, dt);
	_state.input_current = lowpass(_state.input_current, current, _params.voltage_current_lowpass_tau, dt);
}

static void stop(bool expected)
{
	motor_rtctl_stop();
	_state.limit_mask = 0;
	_state.dc_actual = 0.0;
	_state.dc_openloop_setpoint = 0.0;
	_state.rpm_setpoint = 0;
	_state.setpoint_ttl_ms = 0;
	_state.rtctl_state = motor_rtctl_get_state();
	if (expected)
		_state.num_unexpected_stops = 0;
	else
		_state.num_unexpected_stops++;
	rpmctl_reset();
}

static void handle_unexpected_stop(void)
{
	// The motor will not be restarted automatically till the next setpoint update
	stop(false);

	// Usually unexpected stop means that the control is fucked up, so it's good to have some insight
	lowsyslog("Motor: Unexpected stop [%i of %i], below is some debug info\n",
		_state.num_unexpected_stops, _params.num_unexpected_stops_to_latch);
	motor_rtctl_print_debug_info();
}

static void update_control_non_running(void)
{
	// Do not change anything while the motor is starting
	const enum motor_rtctl_state rtctl_state = motor_rtctl_get_state();
	if (rtctl_state == MOTOR_RTCTL_STATE_STARTING)
		return;

	// Start if necessary
	const float spinup_dc = _params.dc_min_voltage / _state.input_voltage;

	const bool need_start =
		(_state.mode == MOTOR_CONTROL_MODE_OPENLOOP && (_state.dc_openloop_setpoint > 0)) ||
		(_state.mode == MOTOR_CONTROL_MODE_RPM && (_state.rpm_setpoint > 0));

	if (need_start && (_state.num_unexpected_stops < _params.num_unexpected_stops_to_latch)) {
		const uint64_t timestamp = motor_rtctl_timestamp_hnsec();

		_state.dc_actual = spinup_dc;
		motor_rtctl_start(spinup_dc, _params.reverse);
		_state.rtctl_state = motor_rtctl_get_state();

		// This HACK prevents the setpoint TTL expiration in case of protracted startup
		const int elapsed_ms = (motor_rtctl_timestamp_hnsec() - timestamp) / HNSEC_PER_MSEC;
		_state.setpoint_ttl_ms += elapsed_ms;

		lowsyslog("Motor: Startup %i ms, DC %f, mode %i\n", elapsed_ms, spinup_dc, _state.mode);

		if (_state.rtctl_state == MOTOR_RTCTL_STATE_IDLE)
			handle_unexpected_stop();
	}
}

static float update_control_open_loop(uint32_t comm_period)
{
	const float min_dc = _params.dc_min_voltage / _state.input_voltage;

	if (_state.dc_openloop_setpoint <= 0)
		return nan("");
	if (_state.dc_openloop_setpoint < min_dc)
		_state.dc_openloop_setpoint = min_dc;

	if (comm_period < _params.comm_period_limit) {
		// Simple P controller
		const float c1 = _params.comm_period_limit;
		const float c0 = _params.comm_period_limit / 2;
		const float dc = (comm_period - c0) / (c1 - c0);

		if (dc < _state.dc_openloop_setpoint) {
			_state.limit_mask |= MOTOR_LIMIT_RPM;
			return dc;
		}
	}
	_state.limit_mask &= ~MOTOR_LIMIT_RPM;
	return _state.dc_openloop_setpoint;
}

static float update_control_rpm(uint32_t comm_period, float dt)
{
	if (_state.rpm_setpoint <= 0)
		return nan("");
	if (_state.rpm_setpoint < _params.rpm_min)
		_state.rpm_setpoint = _params.rpm_min;

	const struct rpmctl_input input = {
		_state.limit_mask,
		dt,
		(float)comm_period_to_rpm(comm_period),
		_state.rpm_setpoint
	};
	return rpmctl_update(&input);
}

static float update_control_current_limit(float new_duty_cycle)
{
	const bool overcurrent = _state.input_current > _params.current_limit;
	const bool braking = _state.dc_actual <= 0.0f || new_duty_cycle <= 0.0f;

	if (overcurrent && !braking) {
		const float error = _state.input_current - _params.current_limit;

		const float comp = error * _params.current_limit_p;
		assert(comp >= 0.0f);

		const float min_dc = _params.dc_min_voltage / _state.input_voltage;

		new_duty_cycle -= comp * _state.dc_actual;
		if (new_duty_cycle < min_dc)
			new_duty_cycle = min_dc;

		_state.limit_mask |= MOTOR_LIMIT_CURRENT;
	} else {
		_state.limit_mask &= ~MOTOR_LIMIT_CURRENT;
	}
	return new_duty_cycle;
}

static float update_control_dc_slope(float new_duty_cycle, float dt)
{
	const float dc_step_max = (fabsf(new_duty_cycle) + fabsf(_state.dc_actual)) * 0.5f * _params.dc_step_max;
	if (fabsf(new_duty_cycle - _state.dc_actual) > dc_step_max) {
		float step = _params.dc_slope * dt;

		if (step > dc_step_max)
			step = dc_step_max;

		if (new_duty_cycle < _state.dc_actual)
			step = -step;

		new_duty_cycle = _state.dc_actual + step;
		_state.limit_mask |= MOTOR_LIMIT_ACCEL;
	} else {
		_state.limit_mask &= ~MOTOR_LIMIT_ACCEL;
	}
	return new_duty_cycle;
}

static void update_control(uint32_t comm_period, float dt)
{
	/*
	 * Start/stop management
	 */
	const enum motor_rtctl_state new_rtctl_state = motor_rtctl_get_state();

	const bool just_stopped =
		new_rtctl_state == MOTOR_RTCTL_STATE_IDLE &&
		_state.rtctl_state != MOTOR_RTCTL_STATE_IDLE;
	if (just_stopped)
		handle_unexpected_stop();

	_state.rtctl_state = new_rtctl_state;
	if (comm_period == 0 || _state.rtctl_state != MOTOR_RTCTL_STATE_RUNNING) {
		update_control_non_running();
		return;
	}

	/*
	 * Primary control logic; can return NAN to stop the motor
	 */
	float new_duty_cycle = nan("");
	if (_state.mode == MOTOR_CONTROL_MODE_OPENLOOP) {
		new_duty_cycle = update_control_open_loop(comm_period);
	}
	else if (_state.mode == MOTOR_CONTROL_MODE_RPM) {
		new_duty_cycle = update_control_rpm(comm_period, dt);
	}
	else assert(0);

	if (!isfinite(new_duty_cycle)) {
		stop(true);
		return;
	}

	/*
	 * Limiters
	 */
	new_duty_cycle = update_control_current_limit(new_duty_cycle);
	new_duty_cycle = update_control_dc_slope(new_duty_cycle, dt);

	/*
	 * Update
	 */
	_state.dc_actual = new_duty_cycle;
	motor_rtctl_set_duty_cycle(_state.dc_actual);
}

static void update_setpoint_ttl(int dt_ms)
{
	const enum motor_rtctl_state rtctl_state = motor_rtctl_get_state();
	if (_state.setpoint_ttl_ms <= 0 || rtctl_state != MOTOR_RTCTL_STATE_RUNNING)
		return;

	_state.setpoint_ttl_ms -= dt_ms;
	if (_state.setpoint_ttl_ms <= 0) {
		stop(true);
		lowsyslog("Motor: Setpoint TTL expired, stop\n");
	}
}

static msg_t control_thread(void* arg)
{
	(void)arg;
	chRegSetThreadName("motor");

	EventListener listener;
	chEvtRegisterMask(&_setpoint_update_event, &listener, ALL_EVENTS);

	uint64_t timestamp_hnsec = motor_rtctl_timestamp_hnsec();

	while (1) {
		/*
		 * Control loop period adapts to comm period.
		 */
		const uint32_t comm_period = motor_rtctl_get_comm_period_hnsec();

		unsigned control_period_ms = IDLE_CONTROL_PERIOD_MSEC;
		if (comm_period > 0)
			control_period_ms = comm_period / HNSEC_PER_MSEC;

		if (control_period_ms < 1)
			control_period_ms = 1;
		else if (control_period_ms > IDLE_CONTROL_PERIOD_MSEC)
			control_period_ms = IDLE_CONTROL_PERIOD_MSEC;

		/*
		 * Thread priority - maximum if the motor is running, normal otherwise
		 */
		const tprio_t desired_thread_priority = (comm_period > 0) ? HIGHPRIO : NORMALPRIO;

		if (desired_thread_priority != chThdGetPriority()) {
			const tprio_t old = chThdSetPriority(desired_thread_priority);
			lowsyslog("Motor: Priority changed: %i --> %i\n", old, desired_thread_priority);
		}

		/*
		 * The event must be set only when the mutex is unlocked.
		 * Otherwise this thread will take control, stumble upon the locked mutex, return the control
		 * to the thread that holds the mutex, unlock the mutex, then proceed.
		 */
		chEvtWaitAnyTimeout(ALL_EVENTS, MS2ST(control_period_ms));

		chMtxLock(&_mutex);

		const uint64_t new_timestamp_hnsec = motor_rtctl_timestamp_hnsec();
		const uint32_t dt_hnsec = new_timestamp_hnsec - timestamp_hnsec;
		const float dt = dt_hnsec / (float)HNSEC_PER_SEC;
		timestamp_hnsec = new_timestamp_hnsec;

		assert(dt > 0);

		update_filters(dt);
		update_setpoint_ttl(dt_hnsec / HNSEC_PER_MSEC);
		update_control(comm_period, dt);

		chMtxUnlock();

		//watchdog_reset(_watchdog_id);
	}

	assert_always(0);
	return 0;
}

int motor_init(void)
{
	// TODO: Watchdog reset logic must be fixed - currently it may timeout while starting the motor
//	_watchdog_id = watchdog_create(WATCHDOG_TIMEOUT_MSEC);
//	if (_watchdog_id < 0)
//		return _watchdog_id;

	int ret = motor_rtctl_init();
	if (ret)
		return ret;

	chMtxInit(&_mutex);
	chEvtInit(&_setpoint_update_event);

	configure();

	init_filters();
	if (_state.input_voltage < MIN_VALID_INPUT_VOLTAGE || _state.input_voltage > MAX_VALID_INPUT_VOLTAGE) {
		lowsyslog("Motor: Invalid input voltage: %f\n", _state.input_voltage);
		return -1;
	}

	ret = rpmctl_init();
	if (ret)
		return ret;

	motor_rtctl_stop();

	assert_always(chThdCreateStatic(_wa_control_thread, sizeof(_wa_control_thread),
		HIGHPRIO, control_thread, NULL));
	return 0;
}

void motor_stop(void)
{
	chMtxLock(&_mutex);
	stop(true);
	chMtxUnlock();
}

void motor_set_duty_cycle(float dc, int ttl_ms)
{
	chMtxLock(&_mutex);

	_state.mode = MOTOR_CONTROL_MODE_OPENLOOP;

	if (dc < 0.0) dc = 0.0;
	if (dc > 1.0) dc = 1.0;
	_state.dc_openloop_setpoint = dc;
	_state.setpoint_ttl_ms = ttl_ms;

	if (dc == 0.0)
		_state.num_unexpected_stops = 0;

	chMtxUnlock();

	// Wake the control thread to process the new setpoint immediately
	chEvtBroadcastFlags(&_setpoint_update_event, ALL_EVENTS);
}

void motor_set_rpm(unsigned rpm, int ttl_ms)
{
	chMtxLock(&_mutex);

	_state.mode = MOTOR_CONTROL_MODE_RPM;

	if (rpm > _params.rpm_max)
		rpm = _params.rpm_max;
	_state.rpm_setpoint = rpm;
	_state.setpoint_ttl_ms = ttl_ms;

	if (rpm == 0)
		_state.num_unexpected_stops = 0;

	chMtxUnlock();

	// Wake the control thread to process the new setpoint immediately
	chEvtBroadcastFlags(&_setpoint_update_event, ALL_EVENTS);
}

float motor_get_duty_cycle(void)
{
	chMtxLock(&_mutex);
	float ret = _state.dc_actual;
	chMtxUnlock();
	return ret;
}

unsigned motor_get_rpm(void)
{
	chMtxLock(&_mutex);
	uint32_t cp = motor_rtctl_get_comm_period_hnsec();
	unsigned rpm = (cp > 0) ? comm_period_to_rpm(cp) : 0;
	chMtxUnlock();
	return rpm;
}

enum motor_control_mode motor_get_control_mode(void)
{
	chMtxLock(&_mutex);
	enum motor_control_mode ret = _state.mode;
	chMtxUnlock();
	return ret;
}

bool motor_is_running(void)
{
	chMtxLock(&_mutex);
	bool ret = motor_rtctl_get_state() == MOTOR_RTCTL_STATE_RUNNING;
	chMtxUnlock();
	return ret;
}

int motor_get_limit_mask(void)
{
	chMtxLock(&_mutex);
	int ret = _state.limit_mask;
	chMtxUnlock();
	return ret;
}

void motor_get_input_voltage_current(float* out_voltage, float* out_current)
{
	chMtxLock(&_mutex);

	if (out_voltage)
		*out_voltage = _state.input_voltage;
	if (out_current)
		*out_current = _state.input_current;

	chMtxUnlock();
}

void motor_confirm_initialization(void)
{
	chMtxLock(&_mutex);
	motor_rtctl_confirm_initialization();
	chMtxUnlock();
}

uint64_t motor_get_zc_failures_since_start(void)
{
	// No lock needed
	return motor_rtctl_get_zc_failures_since_start();
}

int motor_test_hardware(void)
{
	chMtxLock(&_mutex);

	int res = motor_rtctl_test_hardware();
	if (res > 0) { // Try harder in case of failure
		res = motor_rtctl_test_hardware();
	}

	chMtxUnlock();
	return res;
}

int motor_test_motor(void)
{
	chMtxLock(&_mutex);
	const int res = motor_rtctl_test_motor();
	chMtxUnlock();
	return res;
}

void motor_beep(int frequency, int duration_msec)
{
	static const int MAX_DURATION = WATCHDOG_TIMEOUT_MSEC * 3 / 4;

	chMtxLock(&_mutex);

	if (duration_msec > MAX_DURATION)
		duration_msec = MAX_DURATION;

	motor_rtctl_beep(frequency, duration_msec);

	chMtxUnlock();
}

void motor_print_debug_info(void)
{
	chMtxLock(&_mutex);
	motor_rtctl_print_debug_info();
	chMtxUnlock();
}

void motor_emergency(void)
{
	motor_rtctl_emergency();
}

// erpm_to_comm_period = lambda erpm: ((10000000 * 60) / erpm) / 6
// comm_period_to_erpm = lambda cp: (10000000 * 60) / (cp * 6)

static unsigned comm_period_to_rpm(uint32_t comm_period_hnsec)
{
	assert(_params.poles > 0);
	if (comm_period_hnsec == 0)
		return 0;
	const uint32_t x = (120ULL * (uint64_t)HNSEC_PER_SEC) / (_params.poles * 6);
	return x / comm_period_hnsec;
}

void motor_execute_cli_command(int argc, const char* argv[])
{
	if (motor_is_running()) {
		lowsyslog("Unable to execute CLI command now\n");
		return;
	}

	chMtxLock(&_mutex);
	if (argc >= 0 && argv != NULL) {
		motor_rtctl_execute_cli_command(argc, argv);
	} else {
		assert(0);
	}
	chMtxUnlock();
}
