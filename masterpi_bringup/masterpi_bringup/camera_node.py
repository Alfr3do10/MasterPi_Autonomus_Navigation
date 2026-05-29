#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Importación limpia usando el sistema de módulos de ROS 2
from masterpi_bringup.hiwonder_sdk.Camera import Camera

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Declaración de parámetros
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('publish_rate', 15.0) 
        self.declare_parameter('use_mock_hardware', False) 
        self.declare_parameter('publish_grayscale', False)

        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.use_mock_hardware = self.get_parameter('use_mock_hardware').value
        self.publish_grayscale = self.get_parameter('publish_grayscale').value

        self.bridge = CvBridge()
        self.camera_sdk = None

        # Declarar el publicador de ROS 2
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)

        # Inicialización del Hardware según el modo
        if self.use_mock_hardware:
            self.get_logger().info('Nodo de cámara iniciado en MODO SIMULADO (Mock).')
        else:
            self.get_logger().info('Inicializando cámara REAL con el SDK de Hiwonder desde el paquete...')
            self.camera_sdk = Camera()
            self.camera_sdk.camera_open()
            self.get_logger().info('Cámara del SDK abierta con éxito y rectificación activa.')

        # Timer para la publicación periódica
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_image)
        self.get_logger().info('Camera node started.')

    def publish_image(self):
        if self.use_mock_hardware:
            frame = self.create_mock_image()
        else:
            if self.camera_sdk is None or self.camera_sdk.frame is None:
                return
            frame = self.camera_sdk.frame

        # Lógica de color
        if self.publish_grayscale:
            frame_processed = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            encoding_type = 'mono8'
        else:
            frame_processed = frame.copy()
            encoding_type = 'bgr8'

        # Publicar en ROS 2
        try:
            msg = self.bridge.cv2_to_imgmsg(frame_processed, encoding=encoding_type)
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error al publicar la imagen: {e}')

    def create_mock_image(self):
        frame = np.ones((480, 640, 3), dtype=np.uint8) * 255
        cv2.putText(frame, 'MasterPi Camera Mock', (120, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
        return frame

    def destroy_node(self):
        if self.camera_sdk is not None:
            self.get_logger().info('Cerrando la conexión de la cámara del SDK...')
            self.camera_sdk.camera_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()