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

#include "../include/hoverboard_control/serial_port_service.hpp"
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>

using namespace hoverboard_hardware_interface;

bool SerialPortService::connect(const std::string& serial_device, int baud_rate, int timeout) {
    boost::system::error_code ec;

    if (port) {
        RCLCPP_FATAL(rclcpp::get_logger("SerialPortService"), "Port is already opened...");
        return false;
    }

    port = serial_port_ptr(new boost::asio::serial_port(io_service));
    port->open(serial_device, ec);

    if (ec) {
        RCLCPP_FATAL(rclcpp::get_logger("SerialPortService"), "Connection to the %s failed..., error: %s",
                     serial_device.c_str(), ec.message().c_str());
        return false;
    }

    port->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    port->set_option(boost::asio::serial_port_base::character_size(8));
    port->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    port->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    port->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

    const int flags = fcntl(port->native_handle(), F_GETFL);
    if (flags < 0 || fcntl(port->native_handle(), F_SETFL, flags | O_NONBLOCK) < 0) {
        port->close();
        port.reset();
        return false;
    }
    feedback_timeout_ = std::chrono::milliseconds(std::max(1, timeout));
    last_feedback_time_ = std::chrono::steady_clock::now();
    msg_counter = 0;
    prev_byte = 0;
    checksum_errors_ = 0;
    last_checksum_warning_ = std::chrono::steady_clock::time_point{};

    // TODO: try to run it asynchronously
    // boost::thread t([ObjectPtr = &io_service] { return ObjectPtr->run(); });
    // t.detach();

    return true;
}

bool SerialPortService::disconnect() {
    boost::mutex::scoped_lock look(mutex);

    if (port) {
        port->cancel();
        port->close();
        port.reset();
    }

    io_service.stop();
    io_service.reset();

    return true;
}

bool SerialPortService::read() {
    boost::mutex::scoped_lock look(mutex);

    if (!port || !port->is_open()) {
        return false;
    }
    // Use the nonblocking descriptor directly. Boost's synchronous read/write
    // operations can wait for readiness even on a nonblocking descriptor.
    const ssize_t bytes_transferred = ::read(
        port->native_handle(), read_buf_raw, SERIAL_PORT_READ_BUF_SIZE);
    if (bytes_transferred < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        const int read_error = errno;
        RCLCPP_ERROR(rclcpp::get_logger("SerialPortService"),
                     "Hoverboard UART read failed: errno=%d (%s)",
                     read_error, std::strerror(read_error));
        return false;
    }

    for (ssize_t i = 0; i < bytes_transferred; ++i) {
        head_frame = ((uint16_t)(read_buf_raw[i]) << 8) | (uint8_t)prev_byte;

        if (head_frame == HEAD_FRAME) {
            p = (char*)&motorWheelFeedback;
            *p++ = prev_byte;
            *p++ = read_buf_raw[i];
            msg_counter = 2;
        } else if (msg_counter >= 2 && msg_counter < sizeof(MotorWheelFeedback)) {
            *p++ = read_buf_raw[i];
            msg_counter++;
        }

        if (msg_counter == sizeof(MotorWheelFeedback)) {
            const auto now = std::chrono::steady_clock::now();
            if (feedbackChecksum(motorWheelFeedback) == motorWheelFeedback.checksum) {
                motorWheelFeedbackCallback(motorWheelFeedback);
                last_feedback_time_ = now;
            } else {
                ++checksum_errors_;
                if (now - last_checksum_warning_ > std::chrono::seconds(5)) {
                    RCLCPP_WARN(rclcpp::get_logger("SerialPortService"),
                                "Discarding corrupt hoverboard feedback (checksum errors=%llu)",
                                static_cast<unsigned long long>(checksum_errors_));
                    last_checksum_warning_ = now;
                }
            }

            msg_counter = 0;
        }

        prev_byte = read_buf_raw[i];
    }
    if (std::chrono::steady_clock::now() - last_feedback_time_ > feedback_timeout_) {
        RCLCPP_ERROR(rclcpp::get_logger("SerialPortService"), "Hoverboard feedback timed out");
        return false;
    }
    return true;
}

void SerialPortService::asyncRead() {
    if (port.get() == nullptr || !port->is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("SerialPortService"), "Port is already closed...");
        return;
    }

    port->async_read_some(
        boost::asio::buffer(read_buf_raw, SERIAL_PORT_READ_BUF_SIZE),
        boost::bind(&SerialPortService::onReceive,
                    this, boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));
}

void SerialPortService::onReceive(const boost::system::error_code& /*ec*/, size_t /*bytes_transferred*/) {
    RCLCPP_INFO(rclcpp::get_logger("SerialPortService"), "onReceive async event...");

    // boost::mutex::scoped_lock look(mutex);

    // if (port.get() == nullptr || !port->is_open()) return;

    // if (ec) {
    //     asyncRead();
    //     return;
    // }
}

int SerialPortService::write(const char* message, const int& size) {
    if (port.get() == nullptr || !port->is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("SerialPortService"), "Port is already closed...");
        return 0;
    }

    if (size <= 0) {
        return 0;
    }

    const ssize_t written = ::write(port->native_handle(), message, size);
    if (written < 0) {
        const int write_error = errno;
        RCLCPP_ERROR(rclcpp::get_logger("SerialPortService"),
                     "Hoverboard UART write failed: errno=%d (%s), requested=%d bytes",
                     write_error, std::strerror(write_error), size);
        return 0;
    }
    if (written != size) {
        RCLCPP_ERROR(rclcpp::get_logger("SerialPortService"),
                     "Hoverboard UART incomplete write: sent=%zd of %d bytes", written, size);
    }
    return static_cast<int>(written);
}

void SerialPortService::BindMotorWheelFeedbackCallback(std::function<void(MotorWheelFeedback)> fn) {
    motorWheelFeedbackCallback = fn;
}
