from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import yaml


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='modulab_robot_pose',
            executable='modulab_robot_pose',
            name='modulab_robot_pose',
            parameters=[
                {"rate": 5}
            ]
        ),
    ])
