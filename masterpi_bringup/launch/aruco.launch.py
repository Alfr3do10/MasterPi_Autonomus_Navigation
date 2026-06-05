#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'masterpi_bringup'
    pkg_share = get_package_share_directory(package_name)

    robot_params = os.path.join(
        pkg_share,
        'config',
        'robot_params.yaml'
    )

    start_camera = LaunchConfiguration('start_camera')
    start_aruco = LaunchConfiguration('start_aruco')

    camera_node = Node(
        package=package_name,
        executable='camera_node_cpp',
        name='camera_node_cpp',
        output='screen',
        parameters=[robot_params],
        condition=IfCondition(start_camera),
    )

    aruco_node = Node(
        package=package_name,
        executable='aruco.py',
        name='aruco_node',
        output='screen',
        parameters=[robot_params],
        condition=IfCondition(start_aruco),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_camera',
            default_value='true',
            description='Start C++ camera node.'
        ),

        DeclareLaunchArgument(
            'start_aruco',
            default_value='true',
            description='Start ArUco node.'
        ),

        camera_node,

        TimerAction(
            period=1.0,
            actions=[aruco_node]
        ),
    ])
