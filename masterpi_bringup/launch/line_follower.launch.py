from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_params = PathJoinSubstitution([
        FindPackageShare('masterpi_bringup'),
        'config',
        'robot_params.yaml'
    ])

    line_params = PathJoinSubstitution([
        FindPackageShare('masterpi_bringup'),
        'config',
        'line_follower_params.yaml'
    ])

    line_follower_pose_node = Node(
        package='masterpi_bringup',
        executable='line_follower_pose_node.py',
        name='line_follower_pose_node',
        output='screen',
        parameters=[line_params]
    )

    camera_node = Node(
        package='masterpi_bringup',
        executable='camera_node_cpp',
        name='camera_node',
        output='screen',
        parameters=[robot_params]
    )

    line_follower_node = Node(
        package='masterpi_bringup',
        executable='line_follower_node.py',
        name='line_follower_node',
        output='screen',
        parameters=[line_params]
    )

    motor_node = Node(
        package='masterpi_bringup',
        executable='motor_node.py',
        name='motor_node',
        output='screen',
        parameters=[robot_params]
    )

    return LaunchDescription([
        line_follower_pose_node,
        TimerAction(period=1.2, actions=[camera_node]),
        TimerAction(period=2.0, actions=[line_follower_node]),
        TimerAction(period=2.5, actions=[motor_node]),
    ])
