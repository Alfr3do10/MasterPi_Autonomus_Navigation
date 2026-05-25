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

    motor_node = Node(
        package='masterpi_bringup',
        executable='motor_node',
        name='motor_node',
        output='screen',
        parameters=[params_file]
    )

    servo_node = Node(
        package='masterpi_bringup',
        executable='servo_node',
        name='servo_node',
        output='screen',
        parameters=[params_file]
    )

    battery_node = Node(
        package='masterpi_bringup',
        executable='battery_node',
        name='battery_node',
        output='screen',
        parameters=[params_file]
    )

    camera_node = Node(
        package='masterpi_bringup',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[params_file]
    )

    odom_node = Node(
        package='masterpi_bringup',
        executable='odom_node',
        name='odom_node',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        motor_node,
        servo_node,
        battery_node,
        camera_node,
        odom_node,
    ])