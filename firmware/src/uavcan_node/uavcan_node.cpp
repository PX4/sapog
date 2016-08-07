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
#include "bootloader_interface.hpp"
#include <algorithm>
#include <ch.hpp>
#include <board/board.hpp>
#include <zubax_chibios/os.hpp>
#include <zubax_chibios/config/config.h>	// TODO: remove dependency on the implementation details
#include <uavcan/protocol/param_server.hpp>
#include <uavcan/protocol/enumeration/Begin.hpp>
#include <uavcan/protocol/enumeration/Indication.hpp>
#include <uavcan/protocol/dynamic_node_id_client.hpp>
#include <uavcan/protocol/file/BeginFirmwareUpdate.hpp>
#include <uavcan_stm32/bxcan.hpp>
#include <unistd.h>
#include <motor/motor.h>

namespace uavcan_node
{
namespace
{

typedef uavcan::Node<uavcan::MemPoolBlockSize * 128> Node;

uavcan_stm32::CanInitHelper<254> can;

os::config::Param<unsigned> param_node_id("uavcan_node_id",   0,      0,       125);

auto node_status_mode = uavcan::protocol::NodeStatus::MODE_OPERATIONAL;
auto node_status_health = uavcan::protocol::NodeStatus::HEALTH_OK;

std::uint32_t active_can_bus_bit_rate = 0;

Node& get_node()
{
	static Node node(can.driver, uavcan_stm32::SystemClock::instance());
	return node;
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
	void convert(float native_value, ConfigDataType native_type, uavcan::protocol::param::Value& out_value) const
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

	void convertNumeric(float native_value, ConfigDataType native_type,
		uavcan::protocol::param::NumericValue& out_value) const
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
		const char* name = configNameByIndex(index);
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

		(void)configSet(name.c_str(), native_value);
	}

	void readParamValue(const Name& name, Value& out_value) const override
	{
		ConfigParam descr;
		const int res = configGetDescr(name.c_str(), &descr);
		if (res >= 0) {
			convert(configGet(name.c_str()), descr.type, out_value);
		}
	}

	void readParamDefaultMaxMin(const Name& name, Value& out_default,
		NumericValue& out_max, NumericValue& out_min) const override
	{
		ConfigParam descr;
		const int res = configGetDescr(name.c_str(), &descr);
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
			return configSave();
		} else {
			return -1;
		}
	}

	int eraseAllParams() override
	{
		// We can't perform flash IO when the motor controller is active
		if (motor_is_idle()) {
			return configErase();
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
		os::lowsyslog("UAVCAN: Restarting by request from %i\n", int(request_source.get()));
		os::requestReboot();
		return true;
	}
} restart_request_handler;

/*
 * Firmware update handler
 */
typedef uavcan::ServiceServer<uavcan::protocol::file::BeginFirmwareUpdate,
                void (*)(const uavcan::ReceivedDataStructure<uavcan::protocol::file::BeginFirmwareUpdate::Request>&,
                	 uavcan::protocol::file::BeginFirmwareUpdate::Response&)> BeginFirmwareUpdateServer;

BeginFirmwareUpdateServer& get_begin_firmware_update_server()
{
	static BeginFirmwareUpdateServer srv(get_node());
	return srv;
}

void handle_begin_firmware_update_request(
        const uavcan::ReceivedDataStructure<uavcan::protocol::file::BeginFirmwareUpdate::Request>& request,
        uavcan::protocol::file::BeginFirmwareUpdate::Response& response)
{
	static bool in_progress = false;

	::os::lowsyslog("UAVCAN: BeginFirmwareUpdate request from %d\n", int(request.getSrcNodeID().get()));

	if (in_progress) {
		response.error = response.ERROR_IN_PROGRESS;
	} else {
		in_progress = true;
		pass_parameters_to_bootloader(active_can_bus_bit_rate, get_node().getNodeID());
		os::requestReboot();
	}
}

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
	mutable board::LEDOverlay led_ctl;

	void finish(bool reverse)
	{
		os::lowsyslog("UAVCAN: Enumeration confirmed: motor reverse: %d\n", (int)reverse);

		uavcan::protocol::enumeration::Indication msg;
		msg.parameter_name = "esc_index";
		pub_.broadcast(msg);

		(void)configSet("ctl_dir", reverse ? 1 : 0);

		motor_stop();  // Shouldn't be running anyway
	}

	void handleTimerEvent(const uavcan::TimerEvent& event) override
	{
		led_ctl.blink(board::LEDColor::CYAN);

		if (event.real_time >= confirmation_deadline_) {
			os::lowsyslog("UAVCAN: Enumeration request expired\n");
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
		os::lowsyslog("UAVCAN: Enumeration request from %d, parameter: %s, timeout: %d sec\n",
			(int)req.getSrcNodeID().get(), (const char*)req.parameter_name.c_str(), (int)req.timeout_sec);

		/*
		 * Checking the parameter name.
		 *
		 * Empty means that the node is allowed to autodetect what parameter we're enumerating.
		 * In the case of Sapog we can enumerate only the ESC index, so it's a no-brainer.
		 * If the name is not empty, it must be the name of the ESC index parameter.
		 * Currently this name is "esc_index", but it may be changed in the future.
		 *
		 * However, we'll also need to retain support for "esc_index" even if the parameter gets
		 * renamed in the future, as a workaround for incorrect implementation of the enumeration
		 * procedure in PX4.
		 *
		 * Please find the relevant discussion here: https://github.com/PX4/Firmware/pull/2748
		 * TL;DR:
		 *    There's no such thing as standardized parameter names, except for what's documented here in the
		 *    UAVCAN specification:
		 *    http://uavcan.org/Specification/6._Application_level_functions/#standard-configuration-parameters
		 */
		if (!req.parameter_name.empty() && (req.parameter_name != "esc_index"))
		{
			os::lowsyslog("UAVCAN: Enumeration failure - INVALID PARAM NAME\n");
			resp.error = uavcan::protocol::enumeration::Begin::Response::ERROR_INVALID_PARAMETER;
			return;
		}

		if (!motor_is_idle()) {
			os::lowsyslog("UAVCAN: Enumeration failure - MOTOR CONTROLLER IS NOT IDLE\n");
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
	os::watchdog::Timer wdt_;
	volatile bool need_to_print_status_ = false;

	void handle_background_tasks()
	{
		if (need_to_print_status_) {
			need_to_print_status_ = false;

			std::printf("CAN bitrate: %u\n", unsigned(active_can_bus_bit_rate));
			std::printf("Node ID:     %u\n", get_node().getNodeID().get());
			std::printf("Node mode:   %u\n", node_status_mode);
			std::printf("Node health: %u\n", node_status_health);

			const auto perf = get_node().getDispatcher().getTransferPerfCounter();

			const auto pool_capacity = get_node().getAllocator().getBlockCapacity();
			const auto pool_peak_usage = get_node().getAllocator().getPeakNumUsedBlocks();

			uavcan::CanIfacePerfCounters iface_perf[uavcan::MaxCanIfaces];
			std::uint8_t num_ifaces = 0;
			for (num_ifaces = 0;
			     num_ifaces < get_node().getDispatcher().getCanIOManager().getNumIfaces();
			     num_ifaces++)
			{
				iface_perf[num_ifaces] =
					get_node().getDispatcher().getCanIOManager().getIfacePerfCounters(num_ifaces);
			}

			std::printf("Memory pool capacity:   %u blocks\n", pool_capacity);
			std::printf("Memory pool peak usage: %u blocks\n", pool_peak_usage);

			std::printf("Transfers RX/TX: %u / %u\n",
				    unsigned(perf.getRxTransferCount()),
				    unsigned(perf.getTxTransferCount()));
			std::printf("Transfer errors: %u\n", unsigned(perf.getErrorCount()));

			for (unsigned i = 0; i < num_ifaces; i++)
			{
				std::printf("CAN iface %u:\n", i);
				std::printf("    Frames RX/TX: %u / %u\n",
					    unsigned(iface_perf[i].frames_rx), unsigned(iface_perf[i].frames_tx));
				std::printf("    RX overflows: %u\n",
					    unsigned(can.driver.getIface(i)->getRxQueueOverflowCount()));
				std::printf("    Errors:       %u\n", unsigned(iface_perf[i].errors));
			}
		}
	}

	void init_can()
	{
		int res = 0;
		do {
			wdt_.reset();
			::sleep(1);

			handle_background_tasks();

			std::uint32_t bitrate = get_inherited_can_bus_bit_rate();
			const bool autodetect = bitrate == 0;

			res = can.init([]() {::usleep(can.getRecommendedListeningDelay().toUSec());}, bitrate);

			if (res >= 0) {
				::os::lowsyslog("CAN inited at %u bps\n", unsigned(bitrate));
				active_can_bus_bit_rate = bitrate;
			} else if (autodetect && (res == -uavcan_stm32::ErrBitRateNotDetected)) {
				; // Nothing to do
			} else {
				::os::lowsyslog("Could not init CAN; status: %d, autodetect: %d, bitrate: %u\n",
				        res, int(autodetect), unsigned(bitrate));
			}
		} while (res < 0);

		assert(active_can_bus_bit_rate > 0);
	}

	void init_node()
	{
		get_node().setName(NODE_NAME);

		/*
		 * Software version
		 */
		get_node().setSoftwareVersion(get_uavcan_software_version());

		/*
		 * Hardware version
		 */
		const auto hw_major_minor = board::detect_hardware_version();

		uavcan::protocol::HardwareVersion hwver;

		hwver.major = hw_major_minor.major;
		hwver.minor = hw_major_minor.minor;

		const auto uid = board::read_unique_id();
		std::copy(std::begin(uid), std::end(uid), std::begin(hwver.unique_id));

		{
			board::DeviceSignature signature;
			if (board::try_read_device_signature(signature)) {
				std::copy(std::begin(signature), std::end(signature),
					std::back_inserter(hwver.certificate_of_authenticity));
			}
		}

		get_node().setHardwareVersion(hwver);

		/*
		 * Starting the node
		 */
		while (true) {
			const int uavcan_start_res = get_node().start();
			if (uavcan_start_res >= 0) {
				break;
			}
			os::lowsyslog("UAVCAN: Node init failure: %i, will retry\n", uavcan_start_res);
			::sleep(1);
			handle_background_tasks();
		}
		assert(get_node().isStarted());

		/*
		 * Configuring node ID
		 */
	        if (param_node_id.get() > 0 || get_inherited_node_id().isUnicast())
	        {
	            get_node().setNodeID((param_node_id.get() > 0) ?
	        		    static_cast<std::uint8_t>(param_node_id.get()) : get_inherited_node_id());

	            os::lowsyslog("UAVCAN: Using static node ID %d\n", int(get_node().getNodeID().get()));
	        }
	        else
	        {
	            uavcan::DynamicNodeIDClient dnid_client(get_node());

	            {
	                const int res = dnid_client.start(
	                	get_node().getNodeStatusProvider().getHardwareVersion().unique_id);
	                if (res < 0)
	                {
				board::die(res);
	                }
	            }

	            os::lowsyslog("UAVCAN: Waiting for dynamic node ID allocation...\n");

	            while (!dnid_client.isAllocationComplete())
	            {
	                get_node().spin(uavcan::MonotonicDuration::fromMSec(100));
			wdt_.reset();
			handle_background_tasks();
	            }

	            os::lowsyslog("UAVCAN: Dynamic node ID %d allocated by %d\n",
	                      int(dnid_client.getAllocatedNodeID().get()), int(dnid_client.getAllocatorNodeID().get()));

	            get_node().setNodeID(dnid_client.getAllocatedNodeID());
	        }

		/*
		 * Initializing the logic
		 */
		get_node().setRestartRequestHandler(&restart_request_handler);

		int res = get_param_server().start(&param_manager);
		if (res < 0) {
			board::die(res);
		}

		res = init_esc_controller(get_node());
		if (res < 0) {
			board::die(res);
		}

		res = init_indication_controller(get_node());
		if (res < 0) {
			board::die(res);
		}

	        res = get_begin_firmware_update_server().start(&handle_begin_firmware_update_request);
	        if (res < 0)
	        {
			board::die(res);
	        }

		enumeration_handler_.construct<uavcan::INode&>(get_node());
		res = enumeration_handler_->start();
		if (res < 0) {
			board::die(res);
		}

		os::lowsyslog("UAVCAN: Node started, ID %i\n", int(get_node().getNodeID().get()));
	}

public:
	void main() override
	{
		wdt_.startMSec(10000);
		setName("uavcan");

		init_can();

		wdt_.reset();

		init_node();

		while (!os::isRebootRequested()) {
			wdt_.reset();

			handle_background_tasks();

			get_node().getNodeStatusProvider().setHealth(node_status_health);
			get_node().getNodeStatusProvider().setMode(node_status_mode);

			const int spin_res = get_node().spin(uavcan::MonotonicDuration::fromMSec(100));
			if (spin_res < 0) {
				os::lowsyslog("UAVCAN: Spin failure: %d\n", spin_res);
			}
		}

		os::lowsyslog("UAVCAN: Going down\n");
		(void)get_node().spin(uavcan::MonotonicDuration::fromMSec(10));
	}

	void print_status()
	{
		need_to_print_status_ = true;
		while (need_to_print_status_) {
			::usleep(10000);
		}
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

extern void init_bootloader_interface();

void print_status()
{
	node_thread.print_status();
}

int init()
{
	init_bootloader_interface();

	(void)node_thread.start(HIGHPRIO - 2);

	return 0;
}

}
