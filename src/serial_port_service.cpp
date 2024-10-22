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

#include "serial_port_service.hpp"

namespace hoverboard_hardware_interface {
bool SerialPortService::connect(const std::string& serial_device, int baud_rate, int timeout, rclcpp::Logger logger) {
    boost::system::error_code ec;

    std::lock_guard<std::mutex> lock(mutex_);

    if (port && port->is_open()) {
        RCLCPP_FATAL(logger, "Attempted to open an already opened serial port.");
        return false;
    }

    port = std::make_shared<boost::asio::serial_port>(io_service);
    port->open(serial_device, ec);

    if (ec) {
        RCLCPP_FATAL(logger, "Connection to %s failed. Error: %s",
                     serial_device.c_str(), ec.message().c_str());
        port.reset();
        return false;
    }

    try {
        port->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
        port->set_option(boost::asio::serial_port_base::character_size(8));
        port->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        port->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        port->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    } catch (const boost::system::system_error& e) {
        RCLCPP_FATAL(logger, "Error setting serial port options: %s", e.what());
        port->close();
        port.reset();
        return false;
    }

    // Start the io_service in a separate thread
    io_thread_ = std::thread([this]() {
        io_service.run();
    });

    RCLCPP_INFO(logger, "Successfully connected to serial port: %s", serial_device.c_str());
    return true;
}

bool SerialPortService::disconnect(rclcpp::Logger logger) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (port && port->is_open()) {
        boost::system::error_code ec;

        port->cancel(ec);
        if (ec) {
            RCLCPP_FATAL(logger, "Error cancelling serial port: %s", ec.message().c_str());
            return false;
        }

        port->close(ec);
        if (ec) {
            RCLCPP_FATAL(logger, "Error closing serial port: %s", ec.message().c_str());
            return false;
        }

        port.reset();
        RCLCPP_INFO(logger, "Successfully disconnected serial port.");
    }

    io_service.stop();
    if (io_thread_.joinable()) {
        io_thread_.join();
    }
    io_service.reset();

    return true;
}

void SerialPortService::read(rclcpp::Logger logger) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!port || !port->is_open()) {
        RCLCPP_FATAL(logger, "Attempted to read from a closed serial port.");
        throw std::runtime_error("Serial port is closed.");
    }

    boost::system::error_code ec;
    size_t bytes_transferred = port->read_some(boost::asio::buffer(read_buf_raw, SERIAL_PORT_READ_BUF_SIZE), ec);

    if (ec && ec != boost::asio::error::eof) {
        RCLCPP_FATAL(logger, "Error during serial port read: %s", ec.message().c_str());
        throw std::runtime_error("Serial port read error.");
    }

    for (size_t i = 0; i < bytes_transferred; ++i) {
        head_frame = (static_cast<uint16_t>(read_buf_raw[i]) << 8) | static_cast<uint8_t>(prev_byte);

        if (head_frame == HEAD_FRAME) {
            p = reinterpret_cast<char*>(&motorWheelFeedback);
            *p++ = read_buf_raw[i - 1];
            *p++ = read_buf_raw[i];
            msg_counter = 2;
        } else if (msg_counter >= 2 && msg_counter < sizeof(MotorWheelFeedback)) {
            *p++ = read_buf_raw[i];
            msg_counter++;
        }

        if (msg_counter == sizeof(MotorWheelFeedback)) {
            motorWheelFeedbackCallback(motorWheelFeedback);
            msg_counter = 0;
        }

        prev_byte = read_buf_raw[i];
    }
}

void SerialPortService::asyncRead(rclcpp::Logger logger) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!port || !port->is_open()) {
        RCLCPP_FATAL(logger, "Attempted to start async read on a closed serial port.");
        return;
    }

    port->async_read_some(
        boost::asio::buffer(read_buf_raw, SERIAL_PORT_READ_BUF_SIZE),
        std::bind(&SerialPortService::onReceive, this,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  logger));
}

void SerialPortService::onReceive(const boost::system::error_code& ec, size_t bytes_transferred, rclcpp::Logger logger) {
    if (ec) {
        RCLCPP_FATAL(logger, "Asynchronous read error: %s", ec.message().c_str());
        // Handle error appropriately, possibly by notifying the hardware interface
        return;
    }

    for (size_t i = 0; i < bytes_transferred; ++i) {
        head_frame = (static_cast<uint16_t>(read_buf_raw[i]) << 8) | static_cast<uint8_t>(prev_byte);

        if (head_frame == HEAD_FRAME) {
            p = reinterpret_cast<char*>(&motorWheelFeedback);
            *p++ = read_buf_raw[i - 1];
            *p++ = read_buf_raw[i];
            msg_counter = 2;
        } else if (msg_counter >= 2 && msg_counter < sizeof(MotorWheelFeedback)) {
            *p++ = read_buf_raw[i];
            msg_counter++;
        }

        if (msg_counter == sizeof(MotorWheelFeedback)) {
            motorWheelFeedbackCallback(motorWheelFeedback);
            msg_counter = 0;
        }

        prev_byte = read_buf_raw[i];
    }

    // Continue asynchronous read
    asyncRead(logger);
}

int SerialPortService::write(const char* message, const int& size, rclcpp::Logger logger) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!port || !port->is_open()) {
        RCLCPP_FATAL(logger, "Attempted to write to a closed serial port.");
        return 0;
    }

    if (size == 0) {
        RCLCPP_WARN(logger, "Attempted to write zero bytes to serial port.");
        return 0;
    }

    boost::system::error_code ec;
    size_t bytes_written = port->write_some(boost::asio::buffer(message, size), ec);

    if (ec) {
        RCLCPP_FATAL(logger, "Error writing to serial port: %s", ec.message().c_str());
        return 0;
    }

    return static_cast<int>(bytes_written);
}

void SerialPortService::BindMotorWheelFeedbackCallback(std::function<void(MotorWheelFeedback)> callback) {
    motorWheelFeedbackCallback = callback;
}
}  // namespace hoverboard_hardware_interface
