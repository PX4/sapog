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

#include <ch.h>
#include <assert.h>
#include <config/config.h>
#include <watchdog.h>
#include <can_driver.h>
#include <canaerospace/canaerospace.h>
#include <canaerospace/generic_redundancy_resolver.h>
#include <canaerospace/param_id/uav.h>
#include <motor_manager/motormgr.h>
#include "can_iface.h"
#include "can_binding.h"

#define NUM_REDUND_CHANS  3

CONFIG_PARAM_BOOL("canas_interlacing",               true)
CONFIG_PARAM_INT("canas_esc_id",                     1,    1,     8)
CONFIG_PARAM_INT("canas_command_timeout_ms",         100,  10,    500)

static CanasInstance _canas;
static int _self_esc_id;
static int _motor_command_ttl_ms;
static int _watchdog_id;

// --- canaerospace callbacks ---

static void cb_esc_command(CanasInstance* ci, CanasParamCallbackArgs* args)
{
	// Redundancy resolution
	CanasGrrInstance* grr = args->parg;
	const float figure_of_merit = args->message.service_code;

	const int reason = canasGrrUpdate(grr, args->redund_channel_id, figure_of_merit, args->timestamp_usec);
	assert(reason >= 0);
	const int active_chan = canasGrrGetActiveChannel(grr);
#if DEBUG
	if (reason != CANAS_GRR_REASON_NONE)
		lowsyslog("Canas: GRR selected channel %i, FOM %f, reason %i\n", active_chan, figure_of_merit, reason);
#endif
	if (active_chan != args->redund_channel_id)
		return;  // GRR suggests to ignore this message

	float sp = 0.0f;
	switch (args->message.data.type) {
	case CANAS_DATATYPE_USHORT:
		sp = args->message.data.container.USHORT / 65535.0f;
		break;
	case CANAS_DATATYPE_FLOAT:
		sp = args->message.data.container.FLOAT;
		break;
	default:
		return;  // Too bad, ignore
	}

	motormgr_set_duty_cycle(sp, _motor_command_ttl_ms);
}

// ---------

static void publish_rpm(unsigned rpm)
{
	CanasMessageData msgd;
	msgd.type = CANAS_DATATYPE_USHORT;
	msgd.container.USHORT = (rpm > 0xFFFF) ? 0xFFFF : rpm;
	canasParamPublish(&_canas, CANAS_UAV_ROTOR_RPM_1 + _self_esc_id - 1, &msgd, 0);
}

void canif_1hz_callback(void)
{
	if (!motormgr_is_running())
		publish_rpm(0);      // Publish 0 even if the motor is starting
}

void canif_10hz_callback(void)
{
	if (motormgr_is_running())
		publish_rpm(motormgr_get_rpm());

	watchdog_reset(_watchdog_id);
}

#define CHECKERR(x, msg) if ((x) != 0) { lowsyslog("Canas: Init failed (%i): " msg "\n", (int)(x)); return (x); } \

int canif_init(void)
{
	_watchdog_id = watchdog_create(1000);
	if (_watchdog_id < 0) {
		lowsyslog("Canas: Watchdog init failed: %i\n", _watchdog_id);
		return _watchdog_id;
	}

	int res = canif_binding_init(&_canas);
	CHECKERR(res, "Low level");

	const bool enable_interlacing   = config_get("canas_interlacing");
	const uint64_t cmd_timeout_usec = config_get("canas_command_timeout_ms") * 1000;
	_self_esc_id                    = config_get("canas_esc_id");

	// e.g. min cmd freq 30Hz --> timeout 33ms --> TTL 99ms
	_motor_command_ttl_ms = cmd_timeout_usec * 3 / 1000;

	/*
	 * Pub/sub
	 */
	static CanasGrrInstance _grr_command;

	CanasGrrConfig grr_cfg = canasGrrMakeConfig();
	grr_cfg.channel_timeout_usec = cmd_timeout_usec;
	grr_cfg.min_fom_switch_interval_usec = grr_cfg.channel_timeout_usec * 2;
	grr_cfg.fom_hysteresis = 0;
	grr_cfg.num_channels = NUM_REDUND_CHANS;

	res = canasGrrInit(&_grr_command, &grr_cfg, &_canas);
	CHECKERR(res, "GRR: command");

	res = canasParamSubscribe(&_canas, CANAS_UAV_ESC_COMMAND_1 + _self_esc_id - 1, NUM_REDUND_CHANS,
		cb_esc_command, &_grr_command);
	CHECKERR(res, "Sub: command");

	res = canasParamAdvertise(&_canas, CANAS_UAV_ROTOR_RPM_1 + _self_esc_id - 1, enable_interlacing);
	CHECKERR(res, "Pub: RPM");

	return 0;
}
