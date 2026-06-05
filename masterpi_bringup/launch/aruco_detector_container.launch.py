from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='aruco_detector_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='masterpi_bringup',
                plugin='masterpi_bringup::CameraComponent', # Ahora sí va a coincidir con el componente C++
                name='camera_node',
                parameters=[
                    {'frame_id': 'camera_link'},
                    {'publish_rate': 15.0},
                    {'image_topic': '/camera/image_raw'},
                ],
                # ¡NUEVO!: Esto fuerza a la cámara a usar punteros compartidos con los vecinos del contenedor
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='masterpi_bringup',
                plugin='masterpi_bringup::ArucoDetectorComponent',
                name='aruco_detector',
                parameters=[
                    {'image_topic': '/camera/image_raw'},
                    {'marker_id_topic': '/aruco/ids'},
                ],
                # ¡NUEVO!: Esto fuerza al detector de ArUco a leer la imagen directo de la memoria compartida
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])