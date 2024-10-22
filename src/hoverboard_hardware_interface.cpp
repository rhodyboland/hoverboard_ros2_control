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

#include "hoverboard_hardware_interface.hpp"

namespace hoverboard_hardware_interface {
hardware_interface::CallbackReturn HoverboardHardwareInterface::on_init(const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_FATAL(get_logger(), "SystemInterface::on_init failed.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    try {
        // Parse hardware parameters with error checking
        if (info.hardware_parameters.find("left_wheel_joint_name") == info.hardware_parameters.end()) {
            RCLCPP_FATAL(get_logger(), "Missing 'left_wheel_joint_name' in hardware parameters.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        hardwareConfig.leftWheelJointName = info.hardware_parameters.at("left_wheel_joint_name");

        if (info.hardware_parameters.find("right_wheel_joint_name") == info.hardware_parameters.end()) {
            RCLCPP_FATAL(get_logger(), "Missing 'right_wheel_joint_name' in hardware parameters.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        hardwareConfig.rightWheelJointName = info.hardware_parameters.at("right_wheel_joint_name");

        if (info.hardware_parameters.find("loop_rate") == info.hardware_parameters.end()) {
            RCLCPP_FATAL(get_logger(), "Missing 'loop_rate' in hardware parameters.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        hardwareConfig.loopRate = std::stof(info.hardware_parameters.at("loop_rate"));

        if (info.hardware_parameters.find("encoder_ticks_per_revolution") == info.hardware_parameters.end()) {
            RCLCPP_FATAL(get_logger(), "Missing 'encoder_ticks_per_revolution' in hardware parameters.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        hardwareConfig.encoderTicksPerRevolution = std::stoi(info.hardware_parameters.at("encoder_ticks_per_revolution"));

        if (info.hardware_parameters.find("device") == info.hardware_parameters.end()) {
            RCLCPP_FATAL(get_logger(), "Missing 'device' in hardware parameters.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        serialPortConfig.device = info.hardware_parameters.at("device");

        if (info.hardware_parameters.find("baud_rate") == info.hardware_parameters.end()) {
            RCLCPP_FATAL(get_logger(), "Missing 'baud_rate' in hardware parameters.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        serialPortConfig.baudRate = std::stoi(info.hardware_parameters.at("baud_rate"));

        if (info.hardware_parameters.find("timeout") == info.hardware_parameters.end()) {
            RCLCPP_FATAL(get_logger(), "Missing 'timeout' in hardware parameters.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        serialPortConfig.timeout = std::stoi(info.hardware_parameters.at("timeout"));

        // Initialize motor wheels
        leftWheel = MotorWheel(hardwareConfig.leftWheelJointName, hardwareConfig.encoderTicksPerRevolution);
        rightWheel = MotorWheel(hardwareConfig.rightWheelJointName, hardwareConfig.encoderTicksPerRevolution);
    } catch (const std::out_of_range &e) {
        RCLCPP_FATAL(get_logger(), "Hardware parameter out of range: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    } catch (const std::invalid_argument &e) {
        RCLCPP_FATAL(get_logger(), "Invalid hardware parameter argument: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    } catch (const std::exception &e) {
        RCLCPP_FATAL(get_logger(), "Exception during on_init: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Validate joint interfaces
    for (const hardware_interface::ComponentInfo &joint : info.joints) {
        if (joint.command_interfaces.size() != 1) {
            RCLCPP_FATAL(get_logger(),
                         "Joint '%s' has %zu command interfaces found. 1 expected.",
                         joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(get_logger(),
                         "Joint '%s' has command interface '%s'. '%s' expected.",
                         joint.name.c_str(),
                         joint.command_interfaces[0].name.c_str(),
                         hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 2) {
            RCLCPP_FATAL(get_logger(),
                         "Joint '%s' has %zu state interfaces found. 2 expected.",
                         joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(get_logger(),
                         "Joint '%s' has first state interface '%s'. '%s' expected.",
                         joint.name.c_str(),
                         joint.state_interfaces[0].name.c_str(),
                         hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(get_logger(),
                         "Joint '%s' has second state interface '%s'. '%s' expected.",
                         joint.name.c_str(),
                         joint.state_interfaces[1].name.c_str(),
                         hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HoverboardHardwareInterface::on_configure(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Configuring HoverboardHardwareInterface...");

    // Attempt to connect to the serial port
    if (!serialPortService.connect(serialPortConfig.device, serialPortConfig.baudRate, serialPortConfig.timeout, get_logger())) {
        RCLCPP_FATAL(get_logger(), "Failed to connect to serial port: %s", serialPortConfig.device.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Bind feedback callback
    serialPortService.BindMotorWheelFeedbackCallback(
        std::bind(&HoverboardHardwareInterface::motorWheelFeedbackCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Successfully configured HoverboardHardwareInterface.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HoverboardHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Cleaning up HoverboardHardwareInterface...");

    if (!serialPortService.disconnect(get_logger())) {
        RCLCPP_FATAL(get_logger(), "Failed to disconnect serial port during cleanup.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_logger(), "Successfully cleaned up HoverboardHardwareInterface.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HoverboardHardwareInterface::on_activate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Activating HoverboardHardwareInterface...");

    // Additional activation logic can be added here

    RCLCPP_INFO(get_logger(), "HoverboardHardwareInterface activated.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HoverboardHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Deactivating HoverboardHardwareInterface...");

    // Additional deactivation logic can be added here

    RCLCPP_INFO(get_logger(), "HoverboardHardwareInterface deactivated.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HoverboardHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(
        hardware_interface::StateInterface(
            leftWheel.getName(), hardware_interface::HW_IF_POSITION, &leftWheel.position_));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(
            leftWheel.getName(), hardware_interface::HW_IF_VELOCITY, &leftWheel.velocity_));

    state_interfaces.emplace_back(
        hardware_interface::StateInterface(
            rightWheel.getName(), hardware_interface::HW_IF_POSITION, &rightWheel.position_));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(
            rightWheel.getName(), hardware_interface::HW_IF_VELOCITY, &rightWheel.velocity_));

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HoverboardHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
            leftWheel.getName(), hardware_interface::HW_IF_VELOCITY, &leftWheel.command_));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
            rightWheel.getName(), hardware_interface::HW_IF_VELOCITY, &rightWheel.command_));

    return command_interfaces;
}

hardware_interface::return_type HoverboardHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &period) {
    if (has_fatal_error_) {
        RCLCPP_FATAL(get_logger(), "Fatal error detected. Aborting read operation.");
        return hardware_interface::return_type::ERROR;
    }

    try {
        serialPortService.read(get_logger());
    } catch (const std::exception &e) {
        RCLCPP_FATAL(get_logger(), "Exception during read: %s", e.what());
        has_fatal_error_ = true;
        return hardware_interface::return_type::ERROR;
    }

    // Update wheel positions and velocities
    double lastPositionLeft = leftWheel.getPosition();
    double newPositionLeft = leftWheel.calculateEncoderAngle();
    double newVelocityLeft = (newPositionLeft - lastPositionLeft) / period.seconds();
    leftWheel.updateState(newPositionLeft, newVelocityLeft);

    double lastPositionRight = rightWheel.getPosition();
    double newPositionRight = rightWheel.calculateEncoderAngle();
    double newVelocityRight = (newPositionRight - lastPositionRight) / period.seconds();
    rightWheel.updateState(newPositionRight, newVelocityRight);

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type HoverboardHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &) {
    if (has_fatal_error_) {
        RCLCPP_FATAL(get_logger(), "Fatal error detected. Aborting write operation.");
        return hardware_interface::return_type::ERROR;
    }

    try {
        MotorWheelDriveControl motorWheelDriveControl;

        // Convert command from radians/sec to desired units
        // Assuming 0.10472 rad/s per unit command (as per original code)
        const double speed_cmd = ((leftWheel.getCommand() / 0.10472) + (rightWheel.getCommand() / 0.10472)) / 2.0;
        const double steer_cmd = ((leftWheel.getCommand() / 0.10472) - speed_cmd) * 2.0;

        // Assign commands to drive control structure
        motorWheelDriveControl.steer = static_cast<int16_t>(steer_cmd);
        motorWheelDriveControl.speed = static_cast<int16_t>(speed_cmd);
        motorWheelDriveControl.checksum = static_cast<uint16_t>(
            motorWheelDriveControl.head ^ motorWheelDriveControl.steer ^ motorWheelDriveControl.speed);

        RCLCPP_DEBUG(get_logger(), "Writing to serial port: Speed=%d, Steer=%d",
                     motorWheelDriveControl.speed, motorWheelDriveControl.steer);

        int bytes_written = serialPortService.write(
            reinterpret_cast<const char *>(&motorWheelDriveControl),
            sizeof(MotorWheelDriveControl),
            get_logger());

        if (bytes_written != sizeof(MotorWheelDriveControl)) {
            RCLCPP_WARN(get_logger(), "Partial write to serial port. Expected %zu bytes, wrote %d bytes.",
                        sizeof(MotorWheelDriveControl), bytes_written);
        }
    } catch (const std::exception &e) {
        RCLCPP_FATAL(get_logger(), "Exception during write: %s", e.what());
        has_fatal_error_ = true;
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

void HoverboardHardwareInterface::motorWheelFeedbackCallback(MotorWheelFeedback feedback) {
    // Validate checksum
    uint16_t calculated_checksum = feedback.head ^ feedback.command1 ^ feedback.command2 ^
                                   feedback.rightMotorSpeed ^ feedback.leftMotorSpeed ^
                                   feedback.rightMotorEncoderCumulativeCount ^ feedback.leftMotorEncoderCumulativeCount ^
                                   feedback.batteryVoltage ^ feedback.boardTemperature ^ feedback.commandLed;
    if (feedback.checksum != calculated_checksum) {
        RCLCPP_FATAL(get_logger(), "Invalid checksum in feedback. Expected: 0x%04X, Received: 0x%04X",
                     calculated_checksum, feedback.checksum);
        has_fatal_error_ = true;
        return;
    }

    try {
        leftWheel.updateEncoderTicks(feedback.leftMotorEncoderCumulativeCount);
        rightWheel.updateEncoderTicks(feedback.rightMotorEncoderCumulativeCount);
    } catch (const std::exception &e) {
        RCLCPP_FATAL(get_logger(), "Exception in motorWheelFeedbackCallback: %s", e.what());
        has_fatal_error_ = true;
    }
}
}  // namespace hoverboard_hardware_interface
