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
#include <config.h>
#include <can_driver.h>
#include <canaerospace/canaerospace.h>
#include <canaerospace/param_id/nod_default.h>
#include "can_iface.h"
#include "can_binding.h"


CONFIG_PARAM_INT("canas_num_redund_chans_to_listen", 3,    1,     4)

static CanasInstance _canas;


void canif_1hz_callback(void)
{
	// TEST
	CanasMessageData msgd;
	msgd.type = CANAS_DATATYPE_ULONG;
	msgd.container.ULONG = (uint32_t)(sys_timestamp_usec() / 1000);
	int res = canasParamPublish(&_canas, CANAS_NOD_DEF_RADIO_HEIGHT, &msgd, 0);
	lowsyslog("Published %i 0x%x\n", res, canYieldErrors(0));
}

void canif_10hz_callback(void)
{
}

#define CHECKERR(x, msg) if ((x) != 0) { lowsyslog("Canas: Init failed (%i): " msg "\n", (int)(x)); return (x); } \

int canif_init(void)
{
	int res = canif_binding_init(&_canas);
	CHECKERR(res, "Low level");

	const int redund_chans_to_listen = config_get("canas_num_redund_chans_to_listen");
	(void)redund_chans_to_listen;

	// TEST
	res = canasParamAdvertise(&_canas, CANAS_NOD_DEF_RADIO_HEIGHT, false);
	CHECKERR(res, "Test adv");

	lowsyslog("Canas: Init OK\n");
	return 0;
}
