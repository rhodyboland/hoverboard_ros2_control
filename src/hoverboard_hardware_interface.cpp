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

#include "../include/hoverboard_control/hoverboard_hardware_interface.hpp"

namespace hoverboard_hardware_interface {
hardware_interface::CallbackReturn HoverboardHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams &params) {
    if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    const auto &info = params.hardware_info;

    hardwareConfig.leftWheelJointName = info.hardware_parameters.at("left_wheel_joint_name");
    hardwareConfig.rightWheelJointName = info.hardware_parameters.at("right_wheel_joint_name");
    hardwareConfig.loopRate = std::stof(info.hardware_parameters.at("loop_rate"));
    // hardwareConfig.encoderTicksPerRevolution = std::stoi(info.hardware_parameters.at("encoder_ticks_per_revolution"));

    serialPortConfig.device = info.hardware_parameters.at("device");
    serialPortConfig.baudRate = std::stoi(info.hardware_parameters.at("baud_rate"));
    serialPortConfig.timeout = std::stoi(info.hardware_parameters.at("timeout"));

    leftWheel = MotorWheel(info.hardware_parameters.at("left_wheel_joint_name"),
                           std::stoi(info.hardware_parameters.at("encoder_ticks_per_revolution")));
    rightWheel = MotorWheel(info.hardware_parameters.at("right_wheel_joint_name"),
                            std::stoi(info.hardware_parameters.at("encoder_ticks_per_revolution")));

    for (const hardware_interface::ComponentInfo &joint : info.joints) {
        if (joint.command_interfaces.size() != 1) {
            RCLCPP_FATAL(
                rclcpp::get_logger("HoverboardHardwareInterface"),
                "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(), joint.command_interfaces.size());

            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(
                rclcpp::get_logger("HoverboardHardwareInterface"),
                "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);

            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 2) {
            RCLCPP_FATAL(
                rclcpp::get_logger("HoverboardHardwareInterface"),
                "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(), joint.state_interfaces.size());

            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(
                rclcpp::get_logger("HoverboardHardwareInterface"),
                "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);

            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(
                rclcpp::get_logger("HoverboardHardwareInterface"),
                "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);

            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HoverboardHardwareInterface::on_configure(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(rclcpp::get_logger("HoverboardHardwareInterface"), "Configuring... please wait a moment...");

    if (!serialPortService.connect(serialPortConfig.device, serialPortConfig.baudRate, serialPortConfig.timeout)) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    serialPortService.BindMotorWheelFeedbackCallback(
        std::bind(&HoverboardHardwareInterface::motorWheelFeedbackCallback, this, std::placeholders::_1));

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HoverboardHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(rclcpp::get_logger("HoverboardHardwareInterface"), "Cleaning up... please wait a moment...");

    if (!serialPortService.disconnect()) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HoverboardHardwareInterface::on_activate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(rclcpp::get_logger("HoverboardHardwareInterface"), "Activating... please wait a moment...");
    for (const auto *wheel : {&leftWheel, &rightWheel}) {
        set_state(wheel->name + "/position", wheel->position);
        set_state(wheel->name + "/velocity", 0.0);
        set_command(wheel->name + "/velocity", 0.0);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HoverboardHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(rclcpp::get_logger("HoverboardHardwareInterface"), "Deactivating... please wait a moment...");
    set_command(leftWheel.name + "/velocity", 0.0);
    set_command(rightWheel.name + "/velocity", 0.0);
    if (write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.0)) != hardware_interface::return_type::OK) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type HoverboardHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &period) {
    if (period.seconds() <= 0.0) {
        return hardware_interface::return_type::ERROR;
    }
    if (!serialPortService.read()) {
        MotorWheelDriveControl stopped{};
        stopped.checksum = stopped.head;
        serialPortService.write(reinterpret_cast<const char *>(&stopped), sizeof(stopped));
        return hardware_interface::return_type::ERROR;
    }

    double lastPosition = leftWheel.position;
    leftWheel.position = leftWheel.calculateEncoderAngle();
    leftWheel.velocity = (leftWheel.position - lastPosition) / period.seconds();

    lastPosition = rightWheel.position;
    rightWheel.position = rightWheel.calculateEncoderAngle();
    rightWheel.velocity = (rightWheel.position - lastPosition) / period.seconds();

    set_state(leftWheel.name + "/position", leftWheel.position);
    set_state(leftWheel.name + "/velocity", leftWheel.velocity);
    set_state(rightWheel.name + "/position", rightWheel.position);
    set_state(rightWheel.name + "/velocity", rightWheel.velocity);

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type HoverboardHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &) {
    MotorWheelDriveControl motorWheelDriveControl;

    const double leftCommand = get_command<double>(leftWheel.name + "/velocity");
    const double rightCommand = get_command<double>(rightWheel.name + "/velocity");
    const bool valid = std::isfinite(leftCommand) && std::isfinite(rightCommand);
    const double speed = valid ? ((leftCommand / 0.10472) + (rightCommand / 0.10472)) / 2.0 : 0.0;
    const double steer = valid ? ((leftCommand / 0.10472) - speed) * 2.0 : 0.0;

    // TODO: radius should be read from the urdf file, check calculations
    motorWheelDriveControl.speed = (int16_t)(speed);
    motorWheelDriveControl.steer = (int16_t)(steer);
    motorWheelDriveControl.checksum = (uint16_t)(motorWheelDriveControl.head ^ motorWheelDriveControl.steer ^ motorWheelDriveControl.speed);

    // RCLCPP_INFO(rclcpp::get_logger("SerialPortService"), "%i %i", motorWheelDriveControl.speed, motorWheelDriveControl.steer);

    const auto sent = serialPortService.write((const char *)&motorWheelDriveControl, sizeof(MotorWheelDriveControl));
    return valid && sent == sizeof(MotorWheelDriveControl)
        ? hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

void HoverboardHardwareInterface::motorWheelFeedbackCallback(MotorWheelFeedback motorWheelFeedback) {
    leftWheel.updateEncoderTicks(motorWheelFeedback.leftMotorEncoderCumulativeCount);
    rightWheel.updateEncoderTicks(motorWheelFeedback.rightMotorEncoderCumulativeCount);
}
}  // namespace hoverboard_hardware_interface

PLUGINLIB_EXPORT_CLASS(hoverboard_hardware_interface::HoverboardHardwareInterface, hardware_interface::SystemInterface)
