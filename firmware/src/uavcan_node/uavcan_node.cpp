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
#include <uavcan/protocol/enumeration/Begin.hpp>
#include <uavcan/protocol/enumeration/Indication.hpp>
#include <uavcan_stm32/bxcan.hpp>
#include <unistd.h>
#include <motor/motor.h>
#include <watchdog.h>

namespace uavcan_node
{
namespace
{

typedef uavcan::Node<UAVCAN_MEM_POOL_BLOCK_SIZE * 128> Node;

uavcan_stm32::CanInitHelper<> can;

auto node_status_mode = uavcan::protocol::NodeStatus::MODE_OPERATIONAL;
auto node_status_health = uavcan::protocol::NodeStatus::HEALTH_OK;
uint8_t node_id = 0;

Node& get_node()
{
	static Node node(can.driver, uavcan_stm32::SystemClock::instance());
	return node;
}

void configure_node()
{
	Node& node = get_node();

	node.setNodeID(node_id);
	node.setName("org.pixhawk.sapog-v1");

	/*
	 * Software version
	 */
	uavcan::protocol::SoftwareVersion swver;

	swver.major = FW_VERSION_MAJOR;
	swver.minor = FW_VERSION_MINOR;

	swver.vcs_commit = GIT_HASH;
	swver.optional_field_flags |= swver.OPTIONAL_FIELD_FLAG_VCS_COMMIT;

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
			out_value.to<uavcan::protocol::param::Value::Tag::boolean_value>() = bool(native_value != 0);
		} else if (native_type == CONFIG_TYPE_INT) {
			out_value.to<uavcan::protocol::param::Value::Tag::integer_value>() = native_value;
		} else if (native_type == CONFIG_TYPE_FLOAT) {
			out_value.to<uavcan::protocol::param::Value::Tag::real_value>() = native_value;
		} else {
			; // :(
		}
	}

	void convertNumeric(float native_value, config_data_type native_type, uavcan::protocol::param::NumericValue& out_value) const
	{
		if (native_type == CONFIG_TYPE_INT) {
			out_value.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = native_value;
		} else if (native_type == CONFIG_TYPE_FLOAT) {
			out_value.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = native_value;
		} else {
			; // :(
		}
	}

	void getParamNameByIndex(Index index, Name& out_name) const override
	{
		const char* name = config_name_by_index(index);
		if (name != nullptr) {
			out_name = name;
		}
	}

	void assignParamValue(const Name& name, const Value& value) override
	{
		Value v = value;
		float native_value = 0.0f;

		if (v.is(uavcan::protocol::param::Value::Tag::boolean_value)) {
			native_value = v.to<uavcan::protocol::param::Value::Tag::boolean_value>() ? 1.0f : 0.0f;
		} else if (v.is(uavcan::protocol::param::Value::Tag::integer_value)) {
			native_value = v.to<uavcan::protocol::param::Value::Tag::integer_value>();
		} else if (value.is(uavcan::protocol::param::Value::Tag::real_value)) {
			native_value = v.to<uavcan::protocol::param::Value::Tag::real_value>();
		}

		(void)config_set(name.c_str(), native_value);
	}

	void readParamValue(const Name& name, Value& out_value) const override
	{
		config_param descr;
		const int res = config_get_descr(name.c_str(), &descr);
		if (res >= 0) {
			convert(config_get(name.c_str()), descr.type, out_value);
		}
	}

	void readParamDefaultMaxMin(const Name& name, Value& out_default,
								NumericValue& out_max, NumericValue& out_min) const override
	{
		config_param descr;
		const int res = config_get_descr(name.c_str(), &descr);
		if (res >= 0) {
			convert(descr.default_, descr.type, out_default);
			convertNumeric(descr.max, descr.type, out_max);
			convertNumeric(descr.min, descr.type, out_min);
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
								 (const uavcan::ReceivedDataStructure<uavcan::protocol::enumeration::Begin::Request>&,
								  uavcan::protocol::enumeration::Begin::Response&)>
			CallbackBinder;

	uavcan::ServiceServer<uavcan::protocol::enumeration::Begin, CallbackBinder> srv_;
	uavcan::Publisher<uavcan::protocol::enumeration::Indication> pub_;
	uavcan::MonotonicTime confirmation_deadline_;
	mutable led::Overlay led_ctl;

	void finish(bool reverse)
	{
		::lowsyslog("UAVCAN: Enumeration confirmed: motor reverse: %d\n", (int)reverse);

		uavcan::protocol::enumeration::Indication msg;
		msg.parameter_name = "esc_index";
		pub_.broadcast(msg);

		(void)config_set("ctl_dir", reverse ? 1 : 0);

		motor_stop();  // Shouldn't be running anyway
	}

	void handleTimerEvent(const uavcan::TimerEvent& event) override
	{
		led_ctl.blink(led::Color::CYAN);

		if (event.real_time >= confirmation_deadline_) {
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

	void handleBegin(const uavcan::ReceivedDataStructure<uavcan::protocol::enumeration::Begin::Request>& req,
					 uavcan::protocol::enumeration::Begin::Response& resp)
	{
		::lowsyslog("UAVCAN: Enumeration request from %d, parameter: %s, timeout: %d sec\n",
			(int)req.getSrcNodeID().get(), (const char*)req.parameter_name.c_str(), (int)req.timeout_sec);

		if (req.parameter_name != "esc_index") {
			::lowsyslog("UAVCAN: Enumeration failure - INVALID PARAM NAME\n");
			resp.error = uavcan::protocol::enumeration::Begin::Response::ERROR_INVALID_PARAMETER;
			return;
		}

		if (!motor_is_idle()) {
			::lowsyslog("UAVCAN: Enumeration failure - MOTOR CONTROLLER IS NOT IDLE\n");
			resp.error = uavcan::protocol::enumeration::Begin::Response::ERROR_INVALID_MODE;
			return;
		}

		confirmation_deadline_ = req.getMonotonicTimestamp() +
					 uavcan::MonotonicDuration::fromMSec(req.timeout_sec * 1000);

		this->startPeriodic(uavcan::MonotonicDuration::fromMSec(CONFIRMATION_CHECK_INTERVAL_MSEC));

		resp.error = uavcan::protocol::enumeration::Begin::Response::ERROR_OK;
	}

public:
	EnumerationHandler(uavcan::INode& node)
		: uavcan::TimerBase(node)
		, srv_(node)
		, pub_(node)
	{ }

	int start()
	{
		return srv_.start(CallbackBinder(this, &EnumerationHandler::handleBegin));
	}
};

/*
 * UAVCAN spin loop
 */
class : public chibios_rt::BaseStaticThread<4000>
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

		enumeration_handler_.construct<uavcan::INode&>(get_node());
		while (enumeration_handler_->start() < 0) {
			::lowsyslog("UAVCAN: Enumeration handler start failed\n");
			::sleep(1);
		}
		::lowsyslog("UAVCAN: Enumeration handler started\n");
	}

public:
	msg_t main() override
	{
		init();

		while (true) {
			get_node().getNodeStatusProvider().setHealth(node_status_health);
			get_node().getNodeStatusProvider().setMode(node_status_mode);

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
	node_status_health = uavcan::protocol::NodeStatus::HEALTH_OK;
}

void set_node_status_warning()
{
	node_status_health = uavcan::protocol::NodeStatus::HEALTH_WARNING;
}

void set_node_status_critical()
{
	node_status_health = uavcan::protocol::NodeStatus::HEALTH_CRITICAL;
}

struct bootloader_app_shared_t {
	union {
		uint64_t ull;
		uint32_t ul[2];
	} crc;
	uint32_t signature;
	uint32_t bus_speed;
	uint32_t node_id;
} __attribute__ ((packed));

static uint64_t bootloader_calculate_signature(
	const bootloader_app_shared_t *pshared
) {
	uavcan::DataTypeSignatureCRC crc;
	crc.add((uint8_t*)(&pshared->signature), sizeof(uint32_t) * 3u);
	return crc.get();
}


/*  CAN_FiRx where (i=0..27|13, x=1, 2)
 *                      STM32_CAN1_FIR(i,x)
 * Using i = 2 does not requier there block
 * to be enabled nor FINIT in CAN_FMR to be set.
 * todo:Validate this claim on F2, F3
 */

#define crc_HiLOC       (uavcan_stm32::bxcan::Can[0]->FilterRegister[2].FR1)
#define crc_LoLOC       (uavcan_stm32::bxcan::Can[0]->FilterRegister[2].FR2)
#define signature_LOC   (uavcan_stm32::bxcan::Can[0]->FilterRegister[3].FR1)
#define bus_speed_LOC   (uavcan_stm32::bxcan::Can[0]->FilterRegister[3].FR2)
#define node_id_LOC     (uavcan_stm32::bxcan::Can[0]->FilterRegister[4].FR1)
#define CRC_H 1
#define CRC_L 0


static bool bootloader_read(bootloader_app_shared_t *shared) {
	shared->signature = signature_LOC;
	shared->bus_speed = bus_speed_LOC;
	shared->node_id = node_id_LOC;
	shared->crc.ul[CRC_L] = crc_LoLOC;
	shared->crc.ul[CRC_H] = crc_HiLOC;

	if (shared->crc.ull == bootloader_calculate_signature(shared)) {
		return true;
	} else {
		return false;
	}
}

int init()
{
	struct bootloader_app_shared_t bl_shared;

	if (bootloader_read(&bl_shared)) {
		node_id = uint8_t(bl_shared.node_id);
		can.init(bl_shared.bus_speed);

		(void)node_thread.start((HIGHPRIO + NORMALPRIO) / 2);
	} else {
		::lowsyslog("UAVCAN: bootloader read failed; not starting node\n");
		::sleep(1);
	}


	return 0;
}

}
