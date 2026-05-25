#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range


class SonarNode(Node):
    def __init__(self):
        super().__init__('sonar_node')

        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('frame_id', 'sonar_link')
        self.declare_parameter('min_range', 0.02)
        self.declare_parameter('max_range', 5.0)
        self.declare_parameter('field_of_view', 0.52)
        self.declare_parameter('use_mock_hardware', False)

        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.field_of_view = float(self.get_parameter('field_of_view').value)
        self.use_mock_hardware = bool(self.get_parameter('use_mock_hardware').value)

        self.sonar = None
        if not self.use_mock_hardware:
            try:
                from masterpi_bringup.hiwonder_sdk import Sonar
                self.sonar = Sonar.Sonar()
            except Exception as exc:
                self.get_logger().error(f'Could not initialize real sonar hardware: {exc}')
                raise

        self.publisher = self.create_publisher(Range, '/sonar/range', 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_range)

        self.get_logger().info('Sonar node started.')
        self.get_logger().info(f'use_mock_hardware = {self.use_mock_hardware}')

    def publish_range(self):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self.field_of_view
        msg.min_range = self.min_range
        msg.max_range = self.max_range

        if self.use_mock_hardware:
            distance_m = 0.50
        else:
            distance_m = (self.sonar.getDistance() / 10.0) / 100.0  # raw mm -> cm -> m

        if distance_m < self.min_range or distance_m > self.max_range:
            msg.range = math.inf
        else:
            msg.range = float(distance_m)

        self.publisher.publish(msg)
        self.get_logger().info(f'Sonar | range={msg.range:.3f} m')


def main(args=None):
    rclpy.init(args=args)
    node = SonarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
