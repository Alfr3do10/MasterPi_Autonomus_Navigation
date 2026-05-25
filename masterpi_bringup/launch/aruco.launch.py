from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('masterpi_bringup'),
        'config',
        'robot_params.yaml'
    ])

    camera_node = Node(
        package='masterpi_bringup',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[params_file]
    )
    aruco_node = Node(
        package='masterpi_bringup',
        executable='aruco',
        name='aruco_node',
        output='screen',
        parameters=[params_file]
    )
    return LaunchDescription([
        camera_node,
        aruco_node
    ])