from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    motion_name = LaunchConfiguration('motion_name')

    arm_params = PathJoinSubstitution([
        FindPackageShare('masterpi_bringup'),
        'config',
        'arm_motion_params.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'motion_name',
            default_value='carry_line_follower',
            description='Arm motion to execute: home, carry_line_follower, pickup, drop'
        ),

        Node(
            package='masterpi_bringup',
            executable='arm_motion_node.py',
            name='arm_motion_node',
            output='screen',
            parameters=[
                arm_params,
                {'motion_name': motion_name}
            ]
        )
    ])
