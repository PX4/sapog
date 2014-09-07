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
#include <algorithm>
#include <ch.hpp>
#include <led.hpp>
#include <sys/sys.h>
#include <config/config.h>
#include <uavcan/protocol/param_server.hpp>
#include <uavcan/protocol/EnumerationRequest.hpp>
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

	/*
	 * Software version
	 */
	uavcan::protocol::SoftwareVersion swver;

	swver.major = FW_VERSION_MAJOR;
	swver.minor = FW_VERSION_MINOR;

	swver.vcs_commit = GIT_HASH;
	swver.optional_field_mask |= swver.OPTIONAL_FIELD_MASK_VCS_COMMIT;

	node.setSoftwareVersion(swver);

	/*
	 * Hardware version
	 */
	uavcan::protocol::HardwareVersion hwver;

	hwver.major = board_get_hardware_revision();

	std::uint8_t uid[BOARD_UNIQUE_ID_SIZE] = {};
	board_read_unique_id(uid);
	std::copy(std::begin(uid), std::end(uid), std::begin(hwver.unique_id));

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
		// TODO: delayed restart, add around 100 ms to transmit the response
		return true; // Will never be executed BTW
	}
} restart_request_handler;

/*
 * Enumeration handler
 */
class EnumerationHandler : public uavcan::TimerBase
{
	static constexpr int CONFIRMATION_CHECK_INTERVAL_MSEC = 50;

	typedef uavcan::MethodBinder<EnumerationHandler*,
	                             void (EnumerationHandler::*)
	                             (const uavcan::ReceivedDataStructure<uavcan::protocol::EnumerationRequest>&)>
	        CallbackBinder;

	uavcan::Subscriber<uavcan::protocol::EnumerationRequest, CallbackBinder> sub_;
	uavcan::MonotonicTime confirmation_deadline_;
	uavcan::NodeID received_node_id_;
	mutable led::Overlay led_ctl;

	void finish(bool reverse) const
	{
		::lowsyslog("UAVCAN: Enumeration confirmed: node ID: %d, motor reverse: %d\n",
			(int)received_node_id_.get(), (int)reverse);

		(void)config_set("uavcan_node_id", received_node_id_.get());
		(void)config_set("motor_reverse", reverse);

		motor_stop();  // Shouldn't be running anyway

		const int save_res = config_save();

		::lowsyslog("UAVCAN: Enumeration: Config saved (status: %d), restarting...\n", save_res);

		motor_beep(1000, 500);
		::usleep(500000);
		NVIC_SystemReset();
	}

	void handleTimerEvent(const uavcan::TimerEvent& event) override
	{
		led_ctl.blink(led::Color::CYAN);

		if ((event.real_time >= confirmation_deadline_) || !received_node_id_.isUnicast()) {
			::lowsyslog("UAVCAN: Enumeration request expired\n");
			this->stop();
			led_ctl.unset();
		} else {
			const auto rotation = motor_get_forced_rotation_direction();
			if (rotation != MOTOR_FORCED_ROTATION_NONE) {
				const bool reverse = rotation != MOTOR_FORCED_ROTATION_FORWARD;
				finish(reverse);
				led_ctl.unset();
			}
		}
	}

	void sub_cb(const uavcan::ReceivedDataStructure<uavcan::protocol::EnumerationRequest>& msg)
	{
		::lowsyslog("UAVCAN: Enumeration request from %d, received node ID: %d, timeout: %d sec\n",
			(int)msg.getSrcNodeID().get(), (int)msg.node_id, (int)msg.timeout_sec);

		if (!uavcan::NodeID(msg.node_id).isUnicast()) {
			::lowsyslog("UAVCAN: Enumeration failure - INVALID PARAMS\n");
			return;
		}

		if (!motor_is_idle()) {
			::lowsyslog("UAVCAN: Enumeration failure - MOTOR CONTROLLER IS NOT IDLE\n");
			return;
		}

		received_node_id_ = msg.node_id;

		confirmation_deadline_ = msg.getMonotonicTimestamp() +
					 uavcan::MonotonicDuration::fromMSec(msg.timeout_sec * 1000);

		this->startPeriodic(uavcan::MonotonicDuration::fromMSec(CONFIRMATION_CHECK_INTERVAL_MSEC));
	}

public:
	EnumerationHandler(uavcan::INode& node)
		: uavcan::TimerBase(node)
		, sub_(node)
	{ }

	int start()
	{
		return sub_.start(CallbackBinder(this, &EnumerationHandler::sub_cb));
	}
};

/*
 * UAVCAN spin loop
 */
class : public chibios_rt::BaseStaticThread<3000>
{
	uavcan::LazyConstructor<EnumerationHandler> enumeration_handler_;
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
			::lowsyslog("UAVCAN: Node init failure: %i, will retry\n", uavcan_start_res);
			::sleep(3);
		}
		assert(get_node().isStarted());

		const bool passive_mode = get_node().isPassiveMode();

		if (!passive_mode) {
			while (get_param_server().start(&param_manager) < 0) {
				; // That's impossible to fail
			}

			while (init_esc_controller(get_node()) < 0) {
				::lowsyslog("UAVCAN: ESC controller init failed\n");
				::sleep(1);
			}

			while (init_indication_controller(get_node()) < 0) {
				::lowsyslog("UAVCAN: Indication controller init failed\n");
				::sleep(1);
			}

			::lowsyslog("UAVCAN: Node started, ID %i\n", int(get_node().getNodeID().get()));
		} else {
			::lowsyslog("UAVCAN: PASSIVE MODE\n");

			enumeration_handler_.construct<uavcan::INode&>(get_node());
			while (enumeration_handler_->start() < 0) {
				::lowsyslog("UAVCAN: Enumeration handler start failed\n");
				::sleep(1);
			}
			::lowsyslog("UAVCAN: Enumeration handler started\n");
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
				::lowsyslog("UAVCAN: Spin failure: %i\n", spin_res);
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
			::lowsyslog("UAVCAN: CAN bitrate %u bps\n", unsigned(config_get("can_bitrate")));
			break;
		}

		::lowsyslog("UAVCAN: CAN init failed [%i], trying default bitrate...\n", can_res);

		auto descr = ::config_param();
		(void)config_get_descr("can_bitrate", &descr);

		can_res = can.init(descr.default_);
		if (can_res >= 0) {
			::lowsyslog("UAVCAN: CAN bitrate %u bps\n", unsigned(descr.default_));
			break;
		}

		remained_attempts--;
		if (remained_attempts <= 0) {
			::lowsyslog("UAVCAN: CAN driver init failure: %i\n", can_res);
			return -1;
		}

		::usleep(100000);
	}

	(void)node_thread.start((HIGHPRIO + NORMALPRIO) / 2);

	return 0;
}

}
