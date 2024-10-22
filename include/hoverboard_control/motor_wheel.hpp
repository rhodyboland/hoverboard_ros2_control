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

#include <cmath>
#include <mutex>
#include <string>

namespace hoverboard_hardware_interface {
constexpr int ENCODER_MIN_VALUE = 0;
constexpr int ENCODER_MAX_VALUE = 9000;
constexpr double ENCODER_LOW_WRAP_FACTOR = 0.3;
constexpr double ENCODER_HIGH_WRAP_FACTOR = 0.7;

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

class MotorWheel {
   public:
    MotorWheel() = default;

    MotorWheel(const std::string &wheelJointName, int encoderTicksPerRevolution)
        : name_(wheelJointName),
          encoderTicksPerRevolution_(encoderTicksPerRevolution),
          encoderLowWrap_(static_cast<int16_t>(ENCODER_LOW_WRAP_FACTOR * (ENCODER_MAX_VALUE - ENCODER_MIN_VALUE) + ENCODER_MIN_VALUE)),
          encoderHighWrap_(static_cast<int16_t>(ENCODER_HIGH_WRAP_FACTOR * (ENCODER_MAX_VALUE - ENCODER_MIN_VALUE) + ENCODER_MIN_VALUE)),
          radiansPerRevolution_((2.0 * M_PI) / encoderTicksPerRevolution) {}

    // Calculate the encoder angle in radians
    double calculateEncoderAngle() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return static_cast<double>(encoderTicks_) * radiansPerRevolution_;
    }

    // Update encoder ticks with thread safety
    void updateEncoderTicks(int16_t newTicks) {
        std::lock_guard<std::mutex> lock(mutex_);

        if (newTicks < encoderLowWrap_ && encoderTicksPrevious_ > encoderHighWrap_) {
            encoderOverflowCount_++;
        }

        if (newTicks > encoderHighWrap_ && encoderTicksPrevious_ < encoderLowWrap_) {
            encoderOverflowCount_--;
        }

        encoderTicks_ = (encoderOverflowCount_ * ENCODER_MAX_VALUE) + newTicks;
        encoderTicksPrevious_ = newTicks;
    }

    // Getters for state interfaces
    std::string getName() const { return name_; }

    double getPosition() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return position_;
    }

    double getVelocity() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return velocity_;
    }

    // Setters for command interfaces
    void setCommand(double cmd) {
        std::lock_guard<std::mutex> lock(mutex_);
        command_ = cmd;
    }

    double getCommand() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return command_;
    }

    // Update position and velocity based on new encoder ticks
    void updateState(double newPosition, double newVelocity) {
        std::lock_guard<std::mutex> lock(mutex_);
        position_ = newPosition;
        velocity_ = newVelocity;
    }

   private:
    std::string name_;
    int encoderTicksPerRevolution_;
    int16_t encoderLowWrap_;
    int16_t encoderHighWrap_;
    double radiansPerRevolution_;

    mutable std::mutex mutex_;

    // Encoder state
    int64_t encoderTicks_ = 0;
    int16_t encoderTicksPrevious_ = 0;
    int16_t encoderOverflowCount_ = 0;

    // Command and state variables
    double command_ = 0.0;
    double position_ = 0.0;
    double velocity_ = 0.0;
};
}  // namespace hoverboard_hardware_interface
