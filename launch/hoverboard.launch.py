# Copyright 2023 Robert Gruberski (Viola Robotics Sp. z o.o. Poland)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hoverboard_power_on = LaunchConfiguration("hoverboard_power_on")
    hoverboard_power_off_on_shutdown = LaunchConfiguration(
        "hoverboard_power_off_on_shutdown"
    )
    hoverboard_power_once_per_session = LaunchConfiguration(
        "hoverboard_power_once_per_session"
    )
    hoverboard_sense_active_high = LaunchConfiguration(
        "hoverboard_sense_active_high"
    )
    hoverboard_sense_pin = LaunchConfiguration("hoverboard_sense_pin")
    hoverboard_switch_pin = LaunchConfiguration("hoverboard_switch_pin")
    hoverboard_power_state_file = LaunchConfiguration("hoverboard_power_state_file")
    hoverboard_power_pulse_seconds = LaunchConfiguration(
        "hoverboard_power_pulse_seconds"
    )
    hoverboard_power_settle_seconds = LaunchConfiguration(
        "hoverboard_power_settle_seconds"
    )
    hoverboard_control_start_delay = LaunchConfiguration(
        "hoverboard_control_start_delay"
    )

    # Define robot description
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("hoverboard_control"), "urdf", "hoverboard_description.xacro"]
            ),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("hoverboard_control"), "config", "hoverboard_controllers.yaml"]
    )

    power_on = ExecuteProcess(
        cmd=[
            FindExecutable(name="ros2"),
            "run",
            "hoverboard_control",
            "hoverboard_power.py",
            "on",
            "--sense-pin",
            hoverboard_sense_pin,
            "--switch-pin",
            hoverboard_switch_pin,
            "--pulse-seconds",
            hoverboard_power_pulse_seconds,
            "--settle-seconds",
            hoverboard_power_settle_seconds,
            "--sense-active-high",
            hoverboard_sense_active_high,
            "--once-per-session",
            hoverboard_power_once_per_session,
            "--state-file",
            hoverboard_power_state_file,
        ],
        output="screen",
        condition=IfCondition(hoverboard_power_on),
    )

    power_off = ExecuteProcess(
        cmd=[
            FindExecutable(name="ros2"),
            "run",
            "hoverboard_control",
            "hoverboard_power.py",
            "off",
            "--sense-pin",
            hoverboard_sense_pin,
            "--switch-pin",
            hoverboard_switch_pin,
            "--pulse-seconds",
            hoverboard_power_pulse_seconds,
            "--sense-active-high",
            hoverboard_sense_active_high,
            "--state-file",
            hoverboard_power_state_file,
        ],
        output="screen",
        condition=IfCondition(hoverboard_power_off_on_shutdown),
    )

    # Define nodes
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[('~/robot_description', '/robot_description'),
                    ('/hoverboard_base_controller/cmd_vel', '/cmd_vel')],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        condition=IfCondition(LaunchConfiguration("start_robot_state_publisher")),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager",
                   "--param-file", robot_controllers],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hoverboard_base_controller", "-c", "/controller_manager",
                   "--param-file", robot_controllers,
                   "--controller-ros-args",
                   "--remap /hoverboard_base_controller/cmd_vel:=/cmd_vel"],
    )

    delayed_control_start = TimerAction(
        period=hoverboard_control_start_delay,
        actions=[
            control_node,
            joint_state_broadcaster_spawner,
            robot_controller_spawner,
        ],
    )

    shutdown_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                power_off
            ]
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("start_robot_state_publisher", default_value="true"),
        DeclareLaunchArgument("hoverboard_power_on", default_value="true"),
        DeclareLaunchArgument(
            "hoverboard_power_off_on_shutdown",
            default_value="false",
            description=(
                "Pulse the hoverboard power switch on launch shutdown. "
                "Keep false unless the sense line is known-good."
            ),
        ),
        DeclareLaunchArgument(
            "hoverboard_power_once_per_session",
            default_value="true",
            description=(
                "Only send one startup pulse per state-file session. This avoids "
                "toggle-off relaunches when the sense line is unavailable."
            ),
        ),
        DeclareLaunchArgument("hoverboard_sense_active_high", default_value="true"),
        DeclareLaunchArgument("hoverboard_sense_pin", default_value="7"),
        DeclareLaunchArgument("hoverboard_switch_pin", default_value="11"),
        DeclareLaunchArgument(
            "hoverboard_power_state_file",
            default_value="/tmp/boxbot_hoverboard_power_on",
        ),
        DeclareLaunchArgument("hoverboard_power_pulse_seconds", default_value="0.2"),
        DeclareLaunchArgument("hoverboard_power_settle_seconds", default_value="0.5"),
        DeclareLaunchArgument("hoverboard_control_start_delay", default_value="1.0"),
        power_on,
        shutdown_handler,
        robot_state_pub_node,
        delayed_control_start,
    ])
