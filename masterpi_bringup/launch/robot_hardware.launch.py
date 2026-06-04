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

    start_motor = LaunchConfiguration('start_motor')
    start_servo = LaunchConfiguration('start_servo')
    start_sonar = LaunchConfiguration('start_sonar')
    start_battery = LaunchConfiguration('start_battery')

    motor_node = Node(
        package=package_name,
        executable='motor_node.py',
        name='motor_node',
        output='screen',
        parameters=[robot_params],
        condition=IfCondition(start_motor),
    )

    servo_node = Node(
        package=package_name,
        executable='servo_node.py',
        name='servo_node',
        output='screen',
        parameters=[robot_params],
        condition=IfCondition(start_servo),
    )

    sonar_node = Node(
        package=package_name,
        executable='sonar_node.py',
        name='sonar_node',
        output='screen',
        parameters=[robot_params],
        condition=IfCondition(start_sonar),
    )

    battery_node = Node(
        package=package_name,
        executable='battery_node.py',
        name='battery_node',
        output='screen',
        parameters=[robot_params],
        condition=IfCondition(start_battery),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_motor',
            default_value='true',
            description='Start motor node.'
        ),

        DeclareLaunchArgument(
            'start_servo',
            default_value='true',
            description='Start servo node.'
        ),

        DeclareLaunchArgument(
            'start_sonar',
            default_value='true',
            description='Start sonar node.'
        ),

        DeclareLaunchArgument(
            'start_battery',
            default_value='false',
            description='Start battery node. On real robot this may require sudo -E.'
        ),

        motor_node,

        TimerAction(
            period=0.5,
            actions=[servo_node]
        ),

        TimerAction(
            period=0.8,
            actions=[sonar_node]
        ),

        TimerAction(
            period=1.0,
            actions=[battery_node]
        ),
    ])
