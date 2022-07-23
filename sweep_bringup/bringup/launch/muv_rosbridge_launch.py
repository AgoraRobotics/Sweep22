from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    params = {
        'port': 9090,
        'retry_startup_delay': 5.0,
        'fragment_timeout': 600,
        'delay_between_messages': 0,
        'max_message_size':10000000,
        'unregister_timeout': 10.0,
        'use_compression': False
        }

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[params]
            ),

        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rosapi',
            output='screen'
        )
    ])
