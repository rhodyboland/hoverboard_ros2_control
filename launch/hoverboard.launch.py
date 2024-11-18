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
from launch.actions import RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# import Jetson.GPIO as GPIO
# import time

# Define GPIO pins
ON_SENSE_PIN = 4
SWITCH_PIN = 17

def gpio_cleanup(context, *args, **kwargs):
    """Cleanup GPIO pins and turn off hoverboard."""
    print("Turning off hoverboard...")
    # Check if the hoverboard is on, then turn it off
    # GPIO.setmode(GPIO.BCM)
    # GPIO.setup(ON_SENSE_PIN, GPIO.IN)
    # GPIO.setup(SWITCH_PIN, GPIO.OUT)
    
    # if GPIO.input(ON_SENSE_PIN):
        # GPIO.output(SWITCH_PIN, GPIO.HIGH)
        # time.sleep(0.1)
        # GPIO.output(SWITCH_PIN, GPIO.LOW)
    #     print("Hoverboard turned off.")
    # else:
    #     print("Hoverboard was already off.")
    
    # Cleanup GPIO pins
    # GPIO.cleanup()
    print("GPIO cleanup completed.")

def generate_launch_description():
    print("Checking hoverboard status...")
    # GPIO.setmode(GPIO.BCM)
    # GPIO.setup(ON_SENSE_PIN, GPIO.IN)
    # GPIO.setup(SWITCH_PIN, GPIO.OUT)

    # Turn on hoverboard if it’s off
    # if not GPIO.input(ON_SENSE_PIN):
        # print("Hoverboard was off, turning on")
        # GPIO.output(SWITCH_PIN, GPIO.HIGH)
        # time.sleep(0.1)
        # GPIO.output(SWITCH_PIN, GPIO.LOW)
    # else:
    #     print("Hoverboard was already on")

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
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("hoverboard_control"), "config", "hoverboard_controllers.yaml"]
    )

    teleop_twist_joy_config_file = PathJoinSubstitution(
        [FindPackageShare("hoverboard_control"), "config", "ps4.config.yaml"]
    )

    # Define nodes
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        remappings=[('/hoverboard_base_controller/cmd_vel_unstamped', '/cmd_vel')],
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
        arguments=["hoverboard_base_controller", "-c", "/controller_manager"],
    )

    joy_node = Node(
        package='joy', executable='joy_node', name='joy_node',
        parameters=[{'deadzone': 0.1, 'autorepeat_rate': 30.0}]
    )

    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[teleop_twist_joy_config_file],
        # remappings=[('/cmd_vel', '/hoverboard_base_controller/cmd_vel_unstamped')],
    )

    # Register a shutdown event handler to turn off hoverboard
    shutdown_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                OpaqueFunction(function=gpio_cleanup)
            ]
        )
    )

    return LaunchDescription([
        shutdown_handler,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        joy_node,
        teleop_twist_joy_node,
    ])
