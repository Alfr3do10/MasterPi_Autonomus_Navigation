#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('use_mock_hardware', True)
        self.declare_parameter('publish_grayscale', True)

        self.camera_index = self.get_parameter('camera_index').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.use_mock_hardware = self.get_parameter('use_mock_hardware').value
        self.publish_grayscale = self.get_parameter('publish_grayscale').value

        self.bridge = CvBridge()

        self.publisher = self.create_publisher(
            Image,
            '/camera/image_raw',
            10
        )

        self.camera = None

        if self.use_mock_hardware:
            self.get_logger().info('Camera node running in mock mode.')
        else:
            self.camera = cv2.VideoCapture(self.camera_index)

            if not self.camera.isOpened():
                self.get_logger().error(f'Could not open camera index {self.camera_index}')
            else:
                self.get_logger().info(f'Camera opened successfully at index {self.camera_index}')

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_image)

        self.get_logger().info('Camera node started.')
        self.get_logger().info(f'use_mock_hardware = {self.use_mock_hardware}')

    def publish_image(self):
        if self.use_mock_hardware:
            frame = self.create_mock_image()
        else:
            if self.camera is None or not self.camera.isOpened():
                return

            ret, frame = self.camera.read()

            if not ret:
                self.get_logger().warn('Failed to read frame from camera.')
                return

        # --- LÓGICA DE CONVERSIÓN DE COLOR ---
        if self.publish_grayscale:
            # Convertimos a escala de grises y usamos codificación 'mono8'
            frame_processed = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            encoding_type = 'mono8'
        else:
            # Mantenemos la imagen original a color y usamos 'bgr8'
            frame_processed = frame
            encoding_type = 'bgr8'

        # Publicamos usando la configuración seleccionada
        msg = self.bridge.cv2_to_imgmsg(frame_processed, encoding=encoding_type)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        self.publisher.publish(msg)

    def create_mock_image(self):
        frame = 255 * self.blank_image(480, 640, 3)
        cv2.putText(
            frame,
            'MasterPi Camera Mock',
            (120, 240),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 0, 0),
            2
        )
        return frame

    def blank_image(self, height, width, channels):
        import numpy as np
        return np.ones((height, width, channels), dtype=np.uint8)

    def destroy_node(self):
        if self.camera is not None:
            self.camera.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()