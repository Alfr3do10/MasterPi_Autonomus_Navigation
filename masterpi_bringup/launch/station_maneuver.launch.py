#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'masterpi_bringup'
    pkg_share = get_package_share_directory(package_name)

    default_params_file = os.path.join(
        pkg_share,
        'config',
        'station_maneuver_params.yaml'
    )

    params_file = LaunchConfiguration('params_file')
    start_motor = LaunchConfiguration('start_motor')

    motor_node = Node(
        package=package_name,
        executable='motor_node.py',
        name='motor_node',
        output='screen',
        condition=IfCondition(start_motor),
    )

    station_maneuver_node = Node(
        package=package_name,
        executable='station_maneuver_node.py',
        name='station_maneuver_node',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Path to station maneuver parameters YAML file.'
        ),

        DeclareLaunchArgument(
            'start_motor',
            default_value='true',
            description='Start motor_node.py together with station_maneuver_node.py.'
        ),

        motor_node,
        station_maneuver_node,
    ])
