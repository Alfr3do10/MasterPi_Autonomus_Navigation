#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    package_name = 'masterpi_bringup'
    pkg_share = get_package_share_directory(package_name)

    line_follower_params = os.path.join(
        pkg_share,
        'config',
        'line_follower_params.yaml'
    )

    arm_service_params = os.path.join(
        pkg_share,
        'config',
        'arm_motion_service_params.yaml'
    )

    mission_manager_params = os.path.join(
        pkg_share,
        'config',
        'mission_manager_params.yaml'
    )

    aruco_detector_params = os.path.join(
        pkg_share,
        'config',
        'aruco_detector_params.yaml'
    )

    sonar_safety_params = os.path.join(
        pkg_share,
        'config',
        'sonar_safety_params.yaml'
    )

    start_camera = LaunchConfiguration('start_camera')
    start_motor = LaunchConfiguration('start_motor')
    start_line_follower = LaunchConfiguration('start_line_follower')
    start_arm_service = LaunchConfiguration('start_arm_service')
    start_mission_manager = LaunchConfiguration('start_mission_manager')
    start_sonar = LaunchConfiguration('start_sonar')
    start_sonar_safety = LaunchConfiguration('start_sonar_safety')

    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    use_deviation = LaunchConfiguration('use_deviation')

    camera_aruco_container = ComposableNodeContainer(
        name='camera_aruco_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='masterpi_bringup',
                plugin='masterpi_bringup::CameraComponent',
                name='camera_node',
                parameters=[
                    {'frame_id': 'camera_link'},
                    {'publish_rate': 15.0},
                    {'image_topic': '/camera/image_raw'},
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='masterpi_bringup',
                plugin='masterpi_bringup::ArucoDetectorComponent',
                name='aruco_detector',
                parameters=[
                    aruco_detector_params,
                    {'image_topic': '/camera/image_raw'},
                    {'marker_id_topic': '/aruco/ids'},
                    {'marker_detection_topic': '/aruco/detections'},
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
        condition=IfCondition(start_camera),
    )

    sonar_node = Node(
        package=package_name,
        executable='sonar_node.py',
        name='sonar_node',
        output='screen',
        parameters=[
            {
                'use_mock_hardware': use_mock_hardware,
                'publish_rate': 10.0,
            }
        ],
        condition=IfCondition(start_sonar),
    )

    sonar_safety_node = Node(
        package=package_name,
        executable='sonar_safety_node.py',
        name='sonar_safety_node',
        output='screen',
        parameters=[sonar_safety_params],
        condition=IfCondition(start_sonar_safety),
    )

    line_follower_node = Node(
        package=package_name,
        executable='line_follower_node.py',
        name='line_follower_node',
        output='screen',
        parameters=[line_follower_params],
        condition=IfCondition(start_line_follower),
    )

    motor_node = Node(
        package=package_name,
        executable='motor_node.py',
        name='motor_node',
        output='screen',
        condition=IfCondition(start_motor),
    )

    arm_motion_service_node = Node(
        package=package_name,
        executable='arm_motion_service_node.py',
        name='arm_motion_service_node',
        output='screen',
        parameters=[
            arm_service_params,
            {
                'use_mock_hardware': use_mock_hardware,
                'use_deviation': use_deviation,
            }
        ],
        condition=IfCondition(start_arm_service),
    )

    mission_manager_node = Node(
        package=package_name,
        executable='mission_manager_node.py',
        name='mission_manager_node',
        output='screen',
        emulate_tty=True,
        parameters=[mission_manager_params],
        condition=IfCondition(start_mission_manager),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_camera',
            default_value='true',
            description='Start the C++ camera node and ArUco detector.'
        ),

        DeclareLaunchArgument(
            'start_motor',
            default_value='true',
            description='Start motor_node.py.'
        ),

        DeclareLaunchArgument(
            'start_line_follower',
            default_value='true',
            description='Start line_follower_node.py.'
        ),

        DeclareLaunchArgument(
            'start_arm_service',
            default_value='true',
            description='Start arm_motion_service_node.py.'
        ),

        DeclareLaunchArgument(
            'start_mission_manager',
            default_value='true',
            description='Start mission_manager_node.py.'
        ),

        DeclareLaunchArgument(
            'start_sonar',
            default_value='true',
            description='Start sonar_node.py.'
        ),

        DeclareLaunchArgument(
            'start_sonar_safety',
            default_value='true',
            description='Start sonar_safety_node.py.'
        ),

        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='If true, hardware nodes log commands but do not move hardware.'
        ),

        DeclareLaunchArgument(
            'use_deviation',
            default_value='true',
            description='If true, arm service applies servo deviation values.'
        ),

        camera_aruco_container,

        TimerAction(
            period=0.5,
            actions=[
                sonar_node,
            ]
        ),

        TimerAction(
            period=0.8,
            actions=[
                sonar_safety_node,
            ]
        ),

        TimerAction(
            period=1.0,
            actions=[
                arm_motion_service_node,
            ]
        ),

        TimerAction(
            period=1.5,
            actions=[
                motor_node,
            ]
        ),

        TimerAction(
            period=2.0,
            actions=[
                line_follower_node,
            ]
        ),

        TimerAction(
            period=3.0,
            actions=[
                mission_manager_node,
            ]
        ),
    ])
