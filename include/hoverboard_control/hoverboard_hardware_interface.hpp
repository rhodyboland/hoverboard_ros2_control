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

#include <atomic>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "motor_wheel.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "serial_port_service.hpp"

namespace hoverboard_hardware_interface {
class HoverboardHardwareInterface : public hardware_interface::SystemInterface {
    struct HardwareConfig {
        std::string leftWheelJointName = "left_wheel_joint";
        std::string rightWheelJointName = "right_wheel_joint";

        float loopRate = 30.0;
        int encoderTicksPerRevolution = 1024;
    };

    struct SerialPortConfig {
        std::string device = "/dev/ttyS0";
        int baudRate = 115200;
        int timeout = 1000;
    };

   public:
    RCLCPP_SHARED_PTR_DEFINITIONS(HoverboardHardwareInterface)

    // Lifecycle Callbacks
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    // Read and Write Interfaces
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    // Export Interfaces
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    // Callback for motor wheel feedback
    void motorWheelFeedbackCallback(MotorWheelFeedback feedback);

   private:
    SerialPortService serialPortService;

    HardwareConfig hardwareConfig;
    SerialPortConfig serialPortConfig;

    MotorWheel leftWheel;
    MotorWheel rightWheel;

    // Internal flag to indicate fatal errors
    std::atomic<bool> has_fatal_error_{false};

    // Connection management
    bool connect();
    bool disconnect();
};
}  // namespace hoverboard_hardware_interface

PLUGINLIB_EXPORT_CLASS(hoverboard_hardware_interface::HoverboardHardwareInterface, hardware_interface::SystemInterface)
