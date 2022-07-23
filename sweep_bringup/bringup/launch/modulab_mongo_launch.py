from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import yaml


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='warehouse_ros_mongo',
            executable='mongo_wrapper_ros.py',
            name='mongo_wrapper'
        ),
        Node(
            package='warehouse_ros_mongo',
            executable='modulab_mongo',
            name='mongo_services',
            output='screen',
            parameters=[
                {'map_topic': 'map_test'}
            ]
        )
    ])
