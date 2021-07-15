# Copyright 2020-2021 the Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
import launch
import launch_ros.actions
import launch.substitutions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import ament_index_python
import os


def get_param(package_name, param_file):
    return os.path.join(ament_index_python.get_package_share_directory(package_name), param_file)


def generate_launch_description():

    # joystick driver node
    joy = launch_ros.actions.Node(
        package='joy',
        node_executable='joy_node',
        output='screen')

    # joystick translator node
    joy_translator = launch_ros.actions.Node(
        package='joystick_vehicle_interface_nodes',
        node_executable='joystick_vehicle_interface_node_exe',
        output='screen',
        remappings=[
            ("gear_cmd", "/joystick/gear_cmd"),
            ("accelerator_cmd", "/joystick/accelerator_cmd"),
            ("steering_cmd", "/vehicle/steering_cmd"),
            ("brake_cmd", "/vehicle/brake_cmd"),
            ("emergency_stop", "/vehicle/emergency_stop"),
        ])

    ld = launch.LaunchDescription([
        joy,
        joy_translator])
    return ld
