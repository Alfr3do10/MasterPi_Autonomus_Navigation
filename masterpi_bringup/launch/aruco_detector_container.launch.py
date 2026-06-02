from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node

def generate_launch_description():
    container = ComposableNodeContainer(
        name='aruco_detector_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='masterpi_bringup',
                plugin='masterpi_bringup::ArucoDetectorComponent',
                name='aruco_detector',
                parameters=[
                    {'image_topic': '/camera/image_raw'},
                    {'marker_id_topic': '/aruco/ids'},
                ],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
