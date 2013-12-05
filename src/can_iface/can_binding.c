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
#include <canaerospace/services/std_identification.h>
#include "can_binding.h"


#define CAN_BITRATE             1000000
#define CAN_TX_TIMEOUT_USEC     (125 * 200)

#define MY_HARDWARE_REVISION    1
#define MY_SOFTWARE_REVISION    1


CONFIG_PARAM_INT("canas_node_id",                    1,    1,     255)
CONFIG_PARAM_INT("canas_redund_chan",                0,    0,     3)
CONFIG_PARAM_INT("canas_service_chan", CANAS_SERVICE_CHANNEL_HIGH_MAX,
	CANAS_SERVICE_CHANNEL_HIGH_MIN, CANAS_SERVICE_CHANNEL_HIGH_MAX)

static WORKING_AREA(_wa_canas, 2048);
static char _strbuf[CANAS_DUMP_BUF_LEN];

// --- libcanaerospace support ---

static int drv_send(CanasInstance* pi, int iface, const CanasCanFrame* pframe)
{
	assert(pi);
	assert(iface >= 0 && iface < CAN_IFACE_COUNT);
	assert(pframe);
	return canSend(iface, pframe);
}

static int drv_filter(CanasInstance* pi, int iface, const CanasCanFilterConfig* pfilters, int nfilters)
{
	assert(pi);
	assert(iface >= 0 && iface < CAN_IFACE_COUNT);
	assert(pfilters);
	assert(nfilters > 0);
	return canFilterSetup(iface, pfilters, nfilters);
}

static void* impl_malloc(CanasInstance* pi, int size)
{
	assert(pi);
	return chCoreAlloc(size);
}

static uint64_t impl_timestamp(CanasInstance* pi)
{
	assert(pi);
	return sys_timestamp_usec();
}

// ---------

static void blocking_poll_read_update(CanasInstance* ci, int timeout_usec)
{
	CanasCanFrame frm;
	int iface = -1;
	int ret = canReceive(&iface, &frm, timeout_usec);
	if (ret < 0) {
		lowsyslog("Canas: CAN RX failed: %i\n", ret);
		return;
	}

	if (ret == 0) {
		ret = canasUpdate(ci, -1, NULL); // No data received
	} else {
		assert(iface >= 0 && iface < CAN_IFACE_COUNT);
		ret = canasUpdate(ci, iface, &frm);
	}

	if (ret != 0) {
		lowsyslog("Canas: Update failed: %i\n", ret);
		lowsyslog("Canas: Offending frame: %s\n", canasDumpCanFrame(&frm, _strbuf));
	}
}

static msg_t canas_thread(void* arg)
{
	static const int UPDATE_INTERVAL_USEC = 10 * 1000;

	chRegSetThreadName("canas");

	CanasInstance* ci = (CanasInstance*)arg;
	assert(ci);

	uint64_t prev_cb_1hz = sys_timestamp_usec();
	uint64_t prev_cb_10hz = prev_cb_1hz;

	while (1) {
		blocking_poll_read_update(ci, UPDATE_INTERVAL_USEC);

		const uint64_t current_timestamp = sys_timestamp_usec();

		if (current_timestamp - prev_cb_1hz > 1000000) {
			prev_cb_1hz = current_timestamp;
			canif_1hz_callback();
		}

		if (current_timestamp - prev_cb_10hz > 100000) {
			prev_cb_10hz = current_timestamp;
			canif_10hz_callback();
			//watchdog_reset(_watchdog_id);
		}
	}
	return 0;
}

#define CHECKERR(x, msg) if ((x) != 0) { lowsyslog("Canas: Init failed (%i): " msg "\n", (int)(x)); return (x); }

int canif_binding_init(CanasInstance* ci)
{
	CanasConfig cfg = canasMakeConfig();

	cfg.filters_per_iface = CAN_FILTERS_PER_IFACE;
	cfg.fn_filter         = drv_filter;
	cfg.fn_malloc         = impl_malloc;
	cfg.fn_send           = drv_send;
	cfg.fn_timestamp      = impl_timestamp;
	cfg.iface_count       = CAN_IFACE_COUNT;
	cfg.node_id           = config_get("canas_node_id");
	cfg.redund_channel_id = config_get("canas_redund_chan");
	cfg.service_channel   = config_get("canas_service_chan");

	int res = canInit(CAN_BITRATE, CAN_TX_TIMEOUT_USEC);
	CHECKERR(res, "CAN driver");

	res = canasInit(ci, &cfg, NULL);
	CHECKERR(res, "libcanaerospace");

	/*
	 * CANaerospace logic
	 */
	// IDS service
	CanasSrvIdsPayload ids_selfdescr;
	ids_selfdescr.hardware_revision = MY_HARDWARE_REVISION;
	ids_selfdescr.software_revision = MY_SOFTWARE_REVISION;
	ids_selfdescr.id_distribution   = CANAS_SRV_IDS_ID_DISTRIBUTION_STD;  // These two are standard
	ids_selfdescr.header_type       = CANAS_SRV_IDS_HEADER_TYPE_STD;
	res = canasSrvIdsInit(ci, &ids_selfdescr, 0);
	CHECKERR(res, "IDS service");

	/*
	 * Sys
	 */
	assert_always(chThdCreateStatic(_wa_canas, sizeof(_wa_canas), NORMALPRIO, canas_thread, (void*)ci));
	return 0;
}
