# Copyright (c) 2018 Intel Corporation
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        Node(
            package='modulab_motor_odometry',
            executable='modulab_motor_odometry',
            output='screen'),

        Node(package = "tf2_ros", 
            executable = "static_transform_publisher",
            arguments = "0 0 0 0 0 0 wheel_odom base_link_fake".split(' ')),


        # Node(package = "tf2_ros", 
        #     executable = "static_transform_publisher",
        #     arguments = "0 0 0 0 0 0 odom wheel_odom".split(' ')),

        # Node(package = "tf2_ros", 
        #     executable = "static_transform_publisher",
        #     arguments = "0 0 0 0 0 0 odom odom_frame".split(' ')),


        # Node(package = "tf2_ros", 
        #     executable = "static_transform_publisher",
        #     arguments = "0 0 0 0 0 0 odom sense_T_1_odom_frame".split(' ')),


        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node',
        #     output='screen',
        #     parameters=[os.path.join(get_package_share_directory("robot_localization"), 'params', 'ekf.yaml')],
        #    ),

    ])
