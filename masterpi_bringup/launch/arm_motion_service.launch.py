#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_name = 'masterpi_bringup'

    pkg_share = get_package_share_directory(package_name)

    default_params_file = os.path.join(
        pkg_share,
        'config',
        'arm_motion_service_params.yaml'
    )

    params_file = LaunchConfiguration('params_file')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    use_deviation = LaunchConfiguration('use_deviation')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Path to the arm motion service parameters YAML file.'
        ),

        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='If true, the node logs servo commands but does not move real hardware.'
        ),

        DeclareLaunchArgument(
            'use_deviation',
            default_value='true',
            description='If true, the node applies servo deviation values from the Hiwonder SDK YAML.'
        ),

        Node(
            package=package_name,
            executable='arm_motion_service_node.py',
            name='arm_motion_service_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'use_mock_hardware': use_mock_hardware,
                    'use_deviation': use_deviation,
                }
            ],
        ),
    ])
