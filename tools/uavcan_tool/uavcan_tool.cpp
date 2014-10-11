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

#include <iostream>
#include <iomanip>
#include <thread>
#include <mutex>
#include <algorithm>
#include <iterator>
#include <cassert>
#include <stdexcept>
#include <uavcan_linux/uavcan_linux.hpp>
#include <uavcan/equipment/esc/RawCommand.hpp>
#include <uavcan/equipment/esc/RPMCommand.hpp>
#include <uavcan/equipment/esc/Status.hpp>
#include <uavcan/equipment/indication/LightsCommand.hpp>
#include <uavcan/equipment/indication/BeepCommand.hpp>

namespace
{

class StdinLineReader
{
    mutable std::mutex mutex_;
    std::thread thread_;
    std::queue<std::string> queue_;

    void worker()
    {
        while (true)
        {
            std::string input;
            std::getline(std::cin, input);
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.push(input);
        }
    }

public:
    StdinLineReader()
        : thread_(&StdinLineReader::worker, this)
    { }

    bool hasPendingInput() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return !queue_.empty();
    }

    std::string getLine()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty())
        {
            throw std::runtime_error("Input queue is empty");
        }
        auto ret = queue_.front();
        queue_.pop();
        return ret;
    }

    std::vector<std::string> getSplitLine()
    {
        std::istringstream iss(getLine());
        std::vector<std::string> out;
        std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(),
                  std::back_inserter(out));
        return out;
    }
};

uavcan_linux::NodePtr initNode(const std::vector<std::string>& ifaces, uavcan::NodeID nid, const std::string& name)
{
    auto node = uavcan_linux::makeNode(ifaces);
    node->setNodeID(nid);
    node->setName(name.c_str());

    if (node->start() < 0)
    {
        throw std::runtime_error("Failed to start UAVCAN node");
    }

    uavcan::NetworkCompatibilityCheckResult ncc_result;
    if (node->checkNetworkCompatibility(ncc_result) < 0)
    {
        throw std::runtime_error("Network compatibility check failed");
    }
    if (!ncc_result.isOk())
    {
        throw std::runtime_error("Network conflict with node " + std::to_string(ncc_result.conflicting_node.get()));
    }

    node->setStatusOk();
    return node;
}

template <typename DataType>
std::shared_ptr<uavcan::Subscriber<DataType>> makePrintingSubscriber(const uavcan_linux::NodePtr& node)
{
    return node->makeSubscriber<DataType>([](const uavcan::ReceivedDataStructure<DataType>& msg) {
        std::cout << "[" << DataType::getDataTypeFullName() << "]\n" << msg << "\n---" << std::endl;
    });
}

std::vector<std::string> waitForInput(const uavcan_linux::NodePtr& node, StdinLineReader& stdin_reader)
{
    while (!stdin_reader.hasPendingInput())
    {
        const int res = node->spin(uavcan::MonotonicDuration::fromMSec(50));
        if (res < 0)
        {
            node->logError("spin", "Error %*", res);
        }
    }
    return stdin_reader.getSplitLine();
}

void printCommandReference()
{
    std::cout
        << "Command reference:\n"
        << "    listen                              Print interesting messages to stdout\n"
        << "    dc <index> <duty cycle>             Set duty cycle for ESC with the specified index\n"
        << "    rpm <index> <RPM>                   Set RPM setpoint for ESC with the specified index\n"
        << "    led <index> <red> <green> <blue>    Set RGB LED value\n"
        << "    beep <frequency Hz> <duration ms>   Make noise\n";
}

void executeCommand(const uavcan_linux::NodePtr& node, StdinLineReader& stdin_reader, const std::string& command,
                    const std::vector<std::string>& args)
{
    const auto PeriodicJobPeriod = uavcan::MonotonicDuration::fromMSec(5);

    if (command == "listen")
    {
        std::cout << "\nPRESS ENTER TO STOP" << std::endl;
        auto sub_log        = makePrintingSubscriber<uavcan::protocol::debug::LogMessage>(node);
        auto sub_esc_status = makePrintingSubscriber<uavcan::equipment::esc::Status>(node);

        (void)waitForInput(node, stdin_reader);
    }
    else if (command == "dc")
    {
        static uavcan::equipment::esc::RawCommand msg;
        if (args.size() < 2)
        {
            msg = uavcan::equipment::esc::RawCommand();
        }
        else
        {
            const int index = std::max(std::min(std::stoi(args.at(0)), 14), 0);
            if (msg.cmd.size() <= index)
            {
                msg.cmd.resize(index + 1);
            }
            float val = std::stof(args.at(1));
            if ((val > 1.0F) || (val < -1.0F))
            {
                val = 0.0F;
            }
            msg.cmd[index] = val * uavcan::equipment::esc::RawCommand::FieldTypes::cmd::RawValueType::max();
        }
        std::cout << msg << std::endl;

        static auto pub = node->makePublisher<uavcan::equipment::esc::RawCommand>();
        static auto pub_timer = node->makeTimer(PeriodicJobPeriod, [](const uavcan::TimerEvent&) {
            if (!msg.cmd.empty()) { (void)pub->broadcast(msg); }
        });
    }
    else if (command == "rpm")
    {
        static uavcan::equipment::esc::RPMCommand msg;
        if (args.size() < 2)
        {
            msg = uavcan::equipment::esc::RPMCommand();
        }
        else
        {
            const int index = std::max(std::min(std::stoi(args.at(0)), 14), 0);
            if (msg.rpm.size() <= index)
            {
                msg.rpm.resize(index + 1);
            }
            int val = std::stoi(args.at(1));
            if ((val > 100000) || (val < -100000))
            {
                val = 0;
            }
            msg.rpm[index] = val;
        }
        std::cout << msg << std::endl;

        static auto pub = node->makePublisher<uavcan::equipment::esc::RPMCommand>();
        static auto pub_timer = node->makeTimer(PeriodicJobPeriod, [](const uavcan::TimerEvent&) {
            if (!msg.rpm.empty()) { (void)pub->broadcast(msg); }
        });
    }
    else if (command == "led")
    {
        static auto pub = node->makePublisher<uavcan::equipment::indication::LightsCommand>();
        uavcan::equipment::indication::LightsCommand msg;
        msg.commands.resize(1);
        msg.commands[0].light_id = std::max(std::min(std::stoi(args.at(0)), 31), 0);
        msg.commands[0].color.red   = std::stof(args.at(1)) * 31;
        msg.commands[0].color.green = std::stof(args.at(2)) * 63;
        msg.commands[0].color.blue  = std::stof(args.at(3)) * 31;
        std::cout << msg << std::endl;
        (void)pub->broadcast(msg);
    }
    else if (command == "beep")
    {
        static auto pub = node->makePublisher<uavcan::equipment::indication::BeepCommand>();
        uavcan::equipment::indication::BeepCommand msg;
        msg.frequency = std::stof(args.at(0));
        msg.duration  = std::stof(args.at(1)) / 1000.0F;
        std::cout << msg << std::endl;
        (void)pub->broadcast(msg);
    }
    else
    {
        std::cout << "Unknown command" << std::endl;
        printCommandReference();
    }
}

void runForever(const uavcan_linux::NodePtr& node)
{
    printCommandReference();

    StdinLineReader stdin_reader;

    while (true)
    {
        try
        {
            std::cout << "> " << std::flush;

            const auto input = waitForInput(node, stdin_reader);
            if (input.empty())
            {
                continue;
            }
            const auto command = input.at(0);

            executeCommand(node, stdin_reader, command, std::vector<std::string>(input.begin() + 1, input.end()));
        }
        catch (std::exception& ex)
        {
            std::cout << "FAILURE\n" << ex.what() << std::endl;
        }
    }
}

}

int main(int argc, const char** argv)
{
    if (argc < 3)
    {
        std::cout << "Usage:\n\t" << argv[0] << " <node-id> <can-iface-name-1> [can-iface-name-N...]" << std::endl;
        return 1;
    }
    const int self_node_id = std::stoi(argv[1]);
    std::vector<std::string> iface_names(argv + 2, argv + argc);
    uavcan_linux::NodePtr node = initNode(iface_names, self_node_id, "org.pixhawk.px4esc.uavcan_tool");
    runForever(node);
    return 0;
}
