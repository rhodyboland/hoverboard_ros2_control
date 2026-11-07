// Copyright 2026 Rhody Boland
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <poll.h>
#include <pty.h>
#include <unistd.h>

#include <cmath>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <thread>

#include "hardware_interface/component_parser.hpp"
#include "diff_drive_controller_parameters.hpp"
#include "hoverboard_control/hoverboard_hardware_interface.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using hoverboard_hardware_interface::HoverboardHardwareInterface;

class LyricalInterfaceTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    char path[128];
    ASSERT_EQ(openpty(&master_, &slave_, path, nullptr, nullptr), 0);
    const std::string urdf = std::string(R"(
<robot name="test_hoverboard">
  <link name="base_link"/>
  <link name="left_wheel"/>
  <link name="right_wheel"/>
  <joint name="left" type="continuous">
    <parent link="base_link"/><child link="left_wheel"/>
    <axis xyz="0 1 0"/><limit effort="100" velocity="100"/>
  </joint>
  <joint name="right" type="continuous">
    <parent link="base_link"/><child link="right_wheel"/>
    <axis xyz="0 1 0"/><limit effort="100" velocity="100"/>
  </joint>
  <ros2_control name="test_hoverboard" type="system">
    <hardware>
      <plugin>hoverboard_hardware_interface/HoverboardHardwareInterface</plugin>
      <param name="left_wheel_joint_name">left</param>
      <param name="right_wheel_joint_name">right</param>
      <param name="loop_rate">50</param>
      <param name="encoder_ticks_per_revolution">90</param>
      <param name="baud_rate">115200</param>
      <param name="timeout">1000</param>
      <param name="device">)") + path + R"(</param>
    </hardware>
    <joint name="left">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="right">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
</robot>)";
    hardware_interface::HardwareComponentParams params;
    params.hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf).at(0);
    params.clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    ASSERT_EQ(hardware_.init(params), CallbackReturn::SUCCESS);
    ASSERT_EQ(hardware_.on_export_state_interfaces().size(), 4u);
    ASSERT_EQ(hardware_.on_export_command_interfaces().size(), 2u);
    ASSERT_EQ(hardware_.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    configured_ = true;
    ASSERT_EQ(hardware_.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  }

  void TearDown() override
  {
    if (configured_) { hardware_.on_cleanup(rclcpp_lifecycle::State()); }
    if (master_ >= 0) { close(master_); }
    if (slave_ >= 0) { close(slave_); }
  }

  MotorWheelDriveControl receive_command()
  {
    MotorWheelDriveControl command{};
    pollfd descriptor{master_, POLLIN, 0};
    if (poll(&descriptor, 1, 1000) != 1) {
      ADD_FAILURE() << "No command on simulated serial port";
      return command;
    }
    const auto bytes = ::read(master_, &command, sizeof(command));
    EXPECT_EQ(bytes, static_cast<ssize_t>(sizeof(command)));
    EXPECT_EQ(command.head, HEAD_FRAME);
    EXPECT_EQ(command.checksum,
      static_cast<uint16_t>(command.head ^ command.steer ^ command.speed));
    return command;
  }

  HoverboardHardwareInterface hardware_;
  int master_ = -1;
  int slave_ = -1;
  bool configured_ = false;
};

TEST_F(LyricalInterfaceTest, CommandsUseFrameworkStorageAndDeactivateStops)
{
  EXPECT_DOUBLE_EQ(hardware_.get_command<double>("left/velocity"), 0.0);
  EXPECT_DOUBLE_EQ(hardware_.get_command<double>("right/velocity"), 0.0);
  hardware_.set_command("left/velocity", 10.472);
  hardware_.set_command("right/velocity", 10.472);
  ASSERT_EQ(hardware_.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.02)), return_type::OK);
  const auto command = receive_command();
  EXPECT_EQ(command.speed, 100);
  EXPECT_EQ(command.steer, 0);
  ASSERT_EQ(hardware_.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  const auto stopped = receive_command();
  EXPECT_EQ(stopped.speed, 0);
  EXPECT_EQ(stopped.steer, 0);
}

TEST_F(LyricalInterfaceTest, EncoderFeedbackUpdatesFrameworkStates)
{
  MotorWheelFeedback feedback{};
  feedback.head = HEAD_FRAME;
  feedback.leftMotorEncoderCumulativeCount = 45;
  feedback.rightMotorEncoderCumulativeCount = 90;
  ASSERT_EQ(::write(master_, &feedback, sizeof(feedback)), static_cast<ssize_t>(sizeof(feedback)));
  ASSERT_EQ(hardware_.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.5)), return_type::OK);
  EXPECT_NEAR(hardware_.get_state<double>("left/position"), M_PI, 1e-9);
  EXPECT_NEAR(hardware_.get_state<double>("right/position"), 2 * M_PI, 1e-9);
  EXPECT_NEAR(hardware_.get_state<double>("left/velocity"), 2 * M_PI, 1e-9);
  EXPECT_NEAR(hardware_.get_state<double>("right/velocity"), 4 * M_PI, 1e-9);
}

TEST_F(LyricalInterfaceTest, MissingFeedbackDoesNotBlockTheControlLoop)
{
  const auto start = std::chrono::steady_clock::now();
  EXPECT_EQ(hardware_.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.02)), return_type::OK);
  EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(100));
}

TEST_F(LyricalInterfaceTest, FeedbackTimeoutSendsStopAndReportsError)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(1100));
  EXPECT_EQ(hardware_.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.02)), return_type::ERROR);
  const auto stopped = receive_command();
  EXPECT_EQ(stopped.speed, 0);
  EXPECT_EQ(stopped.steer, 0);
}

TEST(SerialPortTimingTest, FullTransmitQueueDoesNotBlock)
{
  int master, slave;
  char path[128];
  ASSERT_EQ(openpty(&master, &slave, path, nullptr, nullptr), 0);
  hoverboard_hardware_interface::SerialPortService serial;
  ASSERT_TRUE(serial.connect(path, 115200, 1000));
  const std::string data(1024, 'x');
  bool full = false;
  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < 1000; ++i) {
    if (serial.write(data.data(), data.size()) == 0) {
      full = true;
      break;
    }
  }
  EXPECT_TRUE(full);
  EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(100));
  serial.disconnect();
  close(master);
  close(slave);
}

TEST_F(LyricalInterfaceTest, NonfiniteCommandsSendZeroAndReportError)
{
  hardware_.set_command("left/velocity", std::numeric_limits<double>::quiet_NaN());
  ASSERT_EQ(hardware_.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.02)), return_type::ERROR);
  const auto stopped = receive_command();
  EXPECT_EQ(stopped.speed, 0);
  EXPECT_EQ(stopped.steer, 0);
}

TEST_F(LyricalInterfaceTest, ControllerYamlPassesLyricalParameterValidation)
{
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", HOVERBOARD_CONTROLLERS_FILE});
  auto node = std::make_shared<rclcpp::Node>("hoverboard_base_controller", options);
  diff_drive_controller::ParamListener listener(node);
  const auto params = listener.get_params();
  EXPECT_EQ(params.left_wheel_names.at(0), "driving_wheel_left_joint");
  EXPECT_EQ(params.right_wheel_names.at(0), "driving_wheel_right_joint");
  EXPECT_DOUBLE_EQ(params.wheel_radius, 0.1);
  EXPECT_DOUBLE_EQ(params.cmd_vel_timeout, 0.5);
  EXPECT_DOUBLE_EQ(params.linear.x.max_acceleration, 1.0);
  EXPECT_DOUBLE_EQ(params.linear.x.max_deceleration, -1.0);
  EXPECT_DOUBLE_EQ(params.linear.x.max_acceleration_reverse, -1.0);
  EXPECT_DOUBLE_EQ(params.linear.x.max_deceleration_reverse, 1.0);
  EXPECT_DOUBLE_EQ(params.angular.z.max_acceleration, 1.0);
  EXPECT_DOUBLE_EQ(params.angular.z.max_deceleration, -1.0);
  EXPECT_DOUBLE_EQ(params.angular.z.max_acceleration_reverse, -1.0);
  EXPECT_DOUBLE_EQ(params.angular.z.max_deceleration_reverse, 1.0);
  EXPECT_TRUE(std::isnan(params.linear.x.max_jerk));
  EXPECT_TRUE(std::isnan(params.angular.z.max_jerk));
}
