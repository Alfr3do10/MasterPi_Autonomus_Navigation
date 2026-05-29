from launch import LaunchDescription
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

    motor_node = Node(
        package='masterpi_bringup',
        executable='motor_node.py',
        name='motor_node',
        output='screen',
        parameters=[robot_params]
    )

    servo_node = Node(
        package='masterpi_bringup',
        executable='servo_node.py',
        name='servo_node',
        output='screen',
        parameters=[robot_params]
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

    return LaunchDescription([
        motor_node,
        servo_node,
        camera_node,
        line_follower_node,
    ])
