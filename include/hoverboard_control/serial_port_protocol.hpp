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

#include <cstdint>

namespace hoverboard_hardware_interface {
constexpr uint16_t HEAD_FRAME = 0xABCD;

// Uncomment and implement if MOTOR_STATES are required
/*
enum class MOTOR_STATES {
    UNOCCUPIED = 0b00,
    RUN = 0b01,
    BRAKE = 0b11,
    LOCK_SHAFT = 0b10,
};
*/

struct MotorWheelFeedback {
    uint16_t head;
    int16_t command1;
    int16_t command2;
    int16_t rightMotorSpeed;
    int16_t leftMotorSpeed;
    int16_t rightMotorEncoderCumulativeCount;
    int16_t leftMotorEncoderCumulativeCount;
    int16_t batteryVoltage;
    int16_t boardTemperature;
    uint16_t commandLed;
    uint16_t checksum;

    // Optional: Constructor to initialize default values
    MotorWheelFeedback()
        : head(0),
          command1(0),
          command2(0),
          rightMotorSpeed(0),
          leftMotorSpeed(0),
          rightMotorEncoderCumulativeCount(0),
          leftMotorEncoderCumulativeCount(0),
          batteryVoltage(0),
          boardTemperature(0),
          commandLed(0),
          checksum(0) {}
};

struct MotorWheelDriveControl {
    uint16_t head = HEAD_FRAME;
    int16_t steer = 0;
    int16_t speed = 0;
    uint16_t checksum = 0;

    // Optional: Constructor to initialize default values
    MotorWheelDriveControl()
        : head(HEAD_FRAME),
          steer(0),
          speed(0),
          checksum(0) {}
};
}  // namespace hoverboard_hardware_interface
