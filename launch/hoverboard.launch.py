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
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)

ON_SENSE_PIN = 4
SWITCH_PIN = 17

GPIO.setup(ON_SENSE_PIN, GPIO.IN)
GPIO.setup(SWITCH_PIN, GPIO.OUT)

if not GPIO.input(ON_SENSE_PIN):
    print("Hover was off, turning on")
    GPIO.output(SWITCH_PIN, GPIO.HIGH)
    time.sleep(0.1)
    GPIO.output(SWITCH_PIN, GPIO.LOW)
else:
    print("Hover was on already")
GPIO.cleanup()
def generate_launch_description():

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("hoverboard_control"), "urdf", "hoverboard_description.xacro"
                ]
            ),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("hoverboard_control"), "config", "hoverboard_controllers.yaml",
        ]
    )

    controller_manager_params = PathJoinSubstitution(
        [
            FindPackageShare("hoverboard_control"), "config", "controller_manager_params.yaml",
        ]
    )

    # rviz_config_file = PathJoinSubstitution(
    #     [
    #         FindPackageShare("hoverboard_control"), "rviz", "hoverboard.rviz"
    #     ]
    # )

    teleop_twist_joy_config_file = PathJoinSubstitution(
        [
            FindPackageShare("hoverboard_control"), "config", "ps4.config.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_manager_params],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hoverboard_base_controller", "--param-file", robot_controllers, "-c", "/controller_manager"]
    )

    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     arguments=["-d", rviz_config_file],
    # )

    joy_node = Node(
        package='joy', executable='joy_node', name='joy_node',
        parameters=[{
            'deadzone': 0.1,
            'autorepeat_rate': 30.0
        }],
    )

    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[teleop_twist_joy_config_file],
        remappings=[
            ('/cmd_vel', '/hoverboard_base_controller/cmd_vel_unstamped'),
        ]
    )

    return LaunchDescription([
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        # rviz_node,
        joy_node,
        teleop_twist_joy_node,
    ])
