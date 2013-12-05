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
#include <can_driver.h>
#include <canaerospace/canaerospace.h>
#include <canaerospace/param_id/uav.h>
#include <motor_manager/motormgr.h>
#include "can_iface.h"
#include "can_binding.h"


CONFIG_PARAM_BOOL("canas_interlacing",               true) // Well, this is pointless if there is no iface redundancy
CONFIG_PARAM_INT("canas_num_redund_chans_to_listen", 3,    1,     4)
CONFIG_PARAM_INT("canas_esc_id",                     1,    1,     8)
CONFIG_PARAM_INT("canas_motor_command_ttl_ms",       500,  50,    120000)

static CanasInstance _canas;
static int _self_esc_id;
static int _motor_command_ttl_ms;

// --- canaerospace callbacks ---

static void cb_esc_command(CanasInstance* ci, CanasParamCallbackArgs* args)
{
	// TODO: Redundancy resolver
	float thrust = 0.0f;

	switch (args->message.data.type) {
	case CANAS_DATATYPE_USHORT:
		thrust = args->message.data.container.USHORT / 65535.0f;
		break;
	case CANAS_DATATYPE_FLOAT:
		thrust = args->message.data.container.FLOAT;
		break;
	default:
		return;  // Too bad, ignore
	}

	motormgr_set_duty_cycle(thrust, _motor_command_ttl_ms);
}

// ---------

void canif_1hz_callback(void)
{
}

void canif_10hz_callback(void)
{
	const unsigned rpm = motormgr_get_rpm();

	CanasMessageData msgd;
	msgd.type = CANAS_DATATYPE_USHORT;
	msgd.container.USHORT = (rpm > 0xFFFF) ? 0xFFFF : rpm;

	canasParamPublish(&_canas, CANAS_UAV_ROTOR_RPM_1 + _self_esc_id - 1, &msgd, 0);
}

#define CHECKERR(x, msg) if ((x) != 0) { lowsyslog("Canas: Init failed (%i): " msg "\n", (int)(x)); return (x); } \

int canif_init(void)
{
	int res = canif_binding_init(&_canas);
	CHECKERR(res, "Low level");

	const int redund_chan_count   = config_get("canas_num_redund_chans_to_listen");
	const bool enable_interlacing = config_get("canas_interlacing");
	_self_esc_id                  = config_get("canas_esc_id");
	_motor_command_ttl_ms         = config_get("canas_motor_command_ttl_ms");

	/*
	 * Pub/sub
	 */
	res = canasParamSubscribe(&_canas, CANAS_UAV_ESC_COMMAND_1 + _self_esc_id - 1,
		redund_chan_count, cb_esc_command, NULL);
	CHECKERR(res, "Sub: command");

	res = canasParamAdvertise(&_canas, CANAS_UAV_ROTOR_RPM_1 + _self_esc_id - 1, enable_interlacing);
	CHECKERR(res, "Pub: RPM");

	lowsyslog("Canas: Init OK\n");
	return 0;
}
