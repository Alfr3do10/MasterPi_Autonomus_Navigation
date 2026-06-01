from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    motion_name = LaunchConfiguration('motion_name')
    servo_id = LaunchConfiguration('servo_id')
    pulse = LaunchConfiguration('pulse')
    move_time_ms = LaunchConfiguration('move_time_ms')

    arm_params = PathJoinSubstitution([
        FindPackageShare('masterpi_bringup'),
        'config',
        'arm_motion_params.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'motion_name',
            default_value='carry_line_follower',
            description='Arm motion to execute: home, carry_line_follower, pickup, drop, single_servo'
        ),
        DeclareLaunchArgument(
            'servo_id',
            default_value='1',
            description='Servo ID used only when motion_name:=single_servo'
        ),
        DeclareLaunchArgument(
            'pulse',
            default_value='1500',
            description='Servo pulse used only when motion_name:=single_servo'
        ),
        DeclareLaunchArgument(
            'move_time_ms',
            default_value='400',
            description='Move time in milliseconds used only when motion_name:=single_servo'
        ),

        Node(
            package='masterpi_bringup',
            executable='arm_motion_node.py',
            name='arm_motion_node',
            output='screen',
            parameters=[
                arm_params,
                {
                    'motion_name': motion_name,
                    'single_servo_id': ParameterValue(servo_id, value_type=int),
                    'single_servo_pulse': ParameterValue(pulse, value_type=int),
                    'single_servo_move_time_ms': ParameterValue(move_time_ms, value_type=int),
                }
            ]
        )
    ])
