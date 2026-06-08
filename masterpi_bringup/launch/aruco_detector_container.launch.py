#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    package_name = 'masterpi_bringup'
    pkg_share = get_package_share_directory(package_name)

    aruco_detector_params = os.path.join(
        pkg_share,
        'config',
        'aruco_detector_params.yaml'
    )

    container = ComposableNodeContainer(
        name='aruco_detector_container',
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
    )

    return LaunchDescription([container])