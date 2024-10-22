// Copyright 2023 Robert Gruberski (Viola Robotics Sp. z o.o. Poland)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "serial_port_protocol.hpp"

namespace hoverboard_hardware_interface {
class SerialPortService {
   public:
    SerialPortService() = default;
    ~SerialPortService() { disconnect(rclcpp::get_logger("SerialPortService")); }

    // Connection Management
    bool connect(const std::string& serial_device, int baud_rate, int timeout, rclcpp::Logger logger);
    bool disconnect(rclcpp::Logger logger);

    // Data Transmission
    void read(rclcpp::Logger logger);
    void asyncRead(rclcpp::Logger logger);

    int write(const char* message, const int& size, rclcpp::Logger logger);

    // Callback Binding
    void BindMotorWheelFeedbackCallback(std::function<void(MotorWheelFeedback)> callback);

   private:
    boost::asio::io_service io_service;
    std::shared_ptr<boost::asio::serial_port> port;
    std::mutex mutex_;

    uint16_t head_frame = 0;
    uint16_t msg_counter = 0;
    uint8_t msg_command = 0;

    char prev_byte = 0;
    char* p = nullptr;

    char read_buf_raw[SERIAL_PORT_READ_BUF_SIZE]{};

    void onReceive(const boost::system::error_code& ec, size_t bytes_transferred, rclcpp::Logger logger);

    std::function<void(MotorWheelFeedback)> motorWheelFeedbackCallback;

    MotorWheelFeedback motorWheelFeedback{};

    std::thread io_thread_;
};
}  // namespace hoverboard_hardware_interface
