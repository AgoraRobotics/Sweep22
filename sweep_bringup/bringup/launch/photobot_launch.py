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
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('modulab_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    description_dir = get_package_share_directory('muv_description')
    description_launch_dir = os.path.join(description_dir, 'launch')

    realsense_dir = get_package_share_directory('realsense2_camera')
    realsense_launch_dir = os.path.join(realsense_dir, 'launch')


    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')


    _file = os.path.join(
        get_package_share_directory('modulab_bringup'),
        'params',
        'muv_params.yaml'
        )

    param_substitutions = {
        'autostart': 'true'}

    params_file = RewrittenYaml(
            source_file=_file,
            param_rewrites=param_substitutions,
            convert_types=True)

    # Specify the actions
    bringup_cmd_group = GroupAction([


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'fitxxx_launch.py'))),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(description_launch_dir, 'muv_description.launch.py'))),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'motor_control_launch.py'))),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'motor_odometry_launch.py'))),


        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(os.path.join(launch_dir, 'modulab_gpio_launch.py'))),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'muv_sonar_launch.py'))),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'muv_bms_launch.py'))),



        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(realsense_launch_dir, 'rs_d400_and_t265_launch.py'))),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'apriltag_launch.py'))),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'muv_velocity_smoother_launch.py'))),



        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(os.path.join(launch_dir, 'modulab_mongo_launch.py'))),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'muv_pointcloud_to_laser_launch.py'))),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'muv_laser_to_pointcloud_launch.py'))),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'muv_rosbridge_launch.py'))),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'modulab_robot_pose_launch.py'))),




        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'localization_launch.py')),
            launch_arguments={'autostart': 'true',
                              'params_file': params_file,
                              'use_lifecycle_mgr': 'false',
                              'map_subscribe_transient_local': 'true'}.items()),



        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'muv_navigation_launch.py')),
            launch_arguments={'autostart': 'true',
                              'params_file': params_file,
                              'use_lifecycle_mgr': 'false',
                              'map_subscribe_transient_local': 'true'}.items()),





        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(os.path.join(launch_dir, 'modulab_current_mode_launch.py'))),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'modulab_rviz_launch.py'))),



    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)


    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld
