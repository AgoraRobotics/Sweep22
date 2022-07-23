
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
    bringup_dir = get_package_share_directory('sweep_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    description_dir = get_package_share_directory('muv_description')
    description_launch_dir = os.path.join(description_dir, 'launch')


    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')


    _file = os.path.join(
        get_package_share_directory('sweep_bringup'),
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
            PythonLaunchDescriptionSource(os.path.join(description_launch_dir, 'muv_description.launch.py'))),


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
