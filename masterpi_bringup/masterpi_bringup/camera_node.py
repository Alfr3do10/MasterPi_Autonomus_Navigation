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
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('image_width', 320)
        self.declare_parameter('image_height', 240)
        self.declare_parameter('use_mock_hardware', True)
        self.declare_parameter('publish_grayscale', False)

        self.camera_index = int(self.get_parameter('camera_index').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.use_mock_hardware = bool(self.get_parameter('use_mock_hardware').value)
        self.publish_grayscale = bool(self.get_parameter('publish_grayscale').value)

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

            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)

            if not self.camera.isOpened():
                self.get_logger().error(f'Could not open camera index {self.camera_index}')
            else:
                self.get_logger().info(f'Camera opened successfully at index {self.camera_index}')
                self.get_logger().info(
                    f'Requested resolution: {self.image_width}x{self.image_height} @ {self.publish_rate} FPS'
                )

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_image)

        self.get_logger().info('Camera node started.')
        self.get_logger().info(f'use_mock_hardware = {self.use_mock_hardware}')
        self.get_logger().info(f'publish_grayscale = {self.publish_grayscale}')

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

            frame = cv2.resize(frame, (self.image_width, self.image_height))

        if self.publish_grayscale:
            frame_processed = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            encoding_type = 'mono8'
        else:
            frame_processed = frame
            encoding_type = 'bgr8'

        msg = self.bridge.cv2_to_imgmsg(frame_processed, encoding=encoding_type)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        self.publisher.publish(msg)

    def create_mock_image(self):
        import numpy as np

        frame = 255 * np.ones(
            (self.image_height, self.image_width, 3),
            dtype=np.uint8
        )

        cv2.putText(
            frame,
            'MasterPi Camera Mock',
            (20, int(self.image_height / 2)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 0),
            2
        )

        return frame

    def destroy_node(self):
        if self.camera is not None:
            self.camera.release()
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
