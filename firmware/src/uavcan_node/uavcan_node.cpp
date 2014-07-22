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

#include "uavcan_node.hpp"
#include "esc_controller.hpp"
#include "indication_controller.hpp"
#include <ch.hpp>
#include <sys/sys.h>
#include <config/config.h>
#include <uavcan/protocol/param_server.hpp>
#include <unistd.h>
#include <motor/motor.h>
#include <watchdog.h>

namespace uavcan_node
{
namespace
{

typedef uavcan::Node<UAVCAN_MEM_POOL_BLOCK_SIZE * 128> Node;

uavcan_stm32::CanInitHelper<> can;

auto node_status_code = uavcan::protocol::NodeStatus::STATUS_INITIALIZING;

CONFIG_PARAM_INT("can_bitrate",    1000000, 20000, 1000000)
CONFIG_PARAM_INT("uavcan_node_id", 0,       0,     125)      ///< 0 for Passive Mode (default)

Node& get_node()
{
	static Node node(can.driver, uavcan_stm32::SystemClock::instance());
	return node;
}

void configure_node()
{
	Node& node = get_node();

	node.setNodeID(config_get("uavcan_node_id"));
	node.setName("org.pixhawk.px4esc");

	uavcan::protocol::SoftwareVersion swver;
	swver.major = FW_VERSION_MAJOR;
	swver.minor = FW_VERSION_MINOR;
	node.setSoftwareVersion(swver);

	uavcan::protocol::HardwareVersion hwver;
	hwver.major = board_get_hardware_revision();
	node.setHardwareVersion(hwver);
}

uavcan::ParamServer& get_param_server()
{
	static uavcan::ParamServer server(get_node());
	return server;
}

/*
 * Param access server
 */
class ParamManager: public uavcan::IParamManager
{
	void convert(float native_value, config_data_type native_type, uavcan::protocol::param::Value& out_value) const
	{
		if (native_type == CONFIG_TYPE_BOOL) {
			out_value.value_bool.push_back(native_value != 0);
		} else if (native_type == CONFIG_TYPE_INT) {
			out_value.value_int.push_back(native_value);
		} else if (native_type == CONFIG_TYPE_FLOAT) {
			out_value.value_float.push_back(native_value);
		} else {
			; // :(
		}
	}

	void getParamNameByIndex(ParamIndex index, ParamName& out_name) const override
	{
		const char* name = config_name_by_index(index);
		if (name != nullptr) {
			out_name = name;
		}
	}

	void assignParamValue(const ParamName& name, const ParamValue& value) override
	{
		const float native_value = (!value.value_bool.empty()) ? (value.value_bool[0]
		        ? 1 : 0) : (!value.value_int.empty())
		        ? value.value_int[0] : value.value_float[0];
		(void)config_set(name.c_str(), native_value);
	}

	void readParamValue(const ParamName& name, ParamValue& out_value) const override
	{
		config_param descr;
		const int res = config_get_descr(name.c_str(), &descr);
		if (res >= 0) {
			convert(config_get(name.c_str()), descr.type, out_value);
		}
	}

	void readParamDefaultMaxMin(const ParamName& name, ParamValue& out_default,
	                            ParamValue& out_max, ParamValue& out_min) const override
	{
		config_param descr;
		const int res = config_get_descr(name.c_str(), &descr);
		if (res >= 0) {
			convert(descr.default_, descr.type, out_default);
			convert(descr.max, descr.type, out_max);
			convert(descr.min, descr.type, out_min);
		}
	}

	int saveAllParams() override
	{
		// We can't perform flash IO when the motor controller is active
		if (motor_is_idle()) {
			return config_save();
		} else {
			return -1;
		}
	}

	int eraseAllParams() override
	{
		// We can't perform flash IO when the motor controller is active
		if (motor_is_idle()) {
			return config_erase();
		} else {
			return -1;
		}
	}
} param_manager;

/*
 * Restart handler
 */
class RestartRequestHandler: public uavcan::IRestartRequestHandler
{
	bool handleRestartRequest(uavcan::NodeID request_source) override
	{
		::lowsyslog("UAVCAN: Restarting by request from %i\n", int(request_source.get()));
		NVIC_SystemReset();
		return true; // Will never be executed BTW
	}
} restart_request_handler;

/*
 * UAVCAN spin loop
 */
class : public chibios_rt::BaseStaticThread<3000>
{
	int watchdog_id_ = -1;

	void init()
	{
		watchdog_id_ = watchdog_create(10000);

		configure_node();

		get_node().setRestartRequestHandler(&restart_request_handler);

		// Starting the UAVCAN node
		while (true) {
			const int uavcan_start_res = get_node().start();
			if (uavcan_start_res >= 0) {
				break;
			}
			lowsyslog("UAVCAN: Node init failure: %i, will retry\n", uavcan_start_res);
			::sleep(3);
		}
		assert(get_node().isStarted());

		const bool passive_mode = get_node().isPassiveMode();

		if (!passive_mode) {
			while (get_param_server().start(&param_manager) < 0) {
				; // That's impossible to fail
			}

			while (init_esc_controller(get_node()) < 0) {
				lowsyslog("UAVCAN: ESC controller init failed\n");
				::sleep(1);
			}

			while (init_indication_controller(get_node()) < 0) {
				lowsyslog("UAVCAN: Indication controller init failed\n");
				::sleep(1);
			}

			lowsyslog("UAVCAN: Node started, ID %i\n", int(get_node().getNodeID().get()));
		} else {
			lowsyslog("UAVCAN: PASSIVE MODE\n");
		}
	}

public:
	msg_t main() override
	{
		init();

		while (true) {
			get_node().getNodeStatusProvider().setStatusCode(node_status_code);

			const int spin_res = get_node().spin(uavcan::MonotonicDuration::fromMSec(100));
			if (spin_res < 0) {
				lowsyslog("UAVCAN: Spin failure: %i\n", spin_res);
			}

			watchdog_reset(watchdog_id_);
		}
		return msg_t();
	}
} node_thread;

}

void set_node_status_ok()
{
	node_status_code = uavcan::protocol::NodeStatus::STATUS_OK;
}

void set_node_status_warning()
{
	node_status_code = uavcan::protocol::NodeStatus::STATUS_WARNING;
}

void set_node_status_critical()
{
	node_status_code = uavcan::protocol::NodeStatus::STATUS_CRITICAL;
}

int init()
{
	int remained_attempts = 5;

	while (true) {
		int can_res = can.init(config_get("can_bitrate"));
		if (can_res >= 0) {
			lowsyslog("UAVCAN: CAN bitrate %u bps\n", unsigned(config_get("can_bitrate")));
			break;
		}

		lowsyslog("UAVCAN: CAN init failed [%i], trying default bitrate...\n", can_res);

		auto descr = ::config_param();
		(void)config_get_descr("can_bitrate", &descr);

		can_res = can.init(descr.default_);
		if (can_res >= 0) {
			lowsyslog("UAVCAN: CAN bitrate %u bps\n", unsigned(descr.default_));
			break;
		}

		remained_attempts--;
		if (remained_attempts <= 0) {
			lowsyslog("UAVCAN: CAN driver init failure: %i\n", can_res);
			return -1;
		}

		::usleep(100000);
	}

	(void)node_thread.start((HIGHPRIO + NORMALPRIO) / 2);

	return 0;
}

}
