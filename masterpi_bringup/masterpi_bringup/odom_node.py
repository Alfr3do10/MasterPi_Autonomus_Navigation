#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')

        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('use_mock_hardware', True)

        self.publish_rate = self.get_parameter('publish_rate').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.use_mock_hardware = self.get_parameter('use_mock_hardware').value

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.last_time = self.get_clock().now()

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_odom)

        self.get_logger().info('Odom node started.')
        self.get_logger().info(f'use_mock_hardware = {self.use_mock_hardware}')
        self.get_logger().info(f'Publishing odom: {self.odom_frame} -> {self.base_frame}')
        self.get_logger().info('Listening to /cmd_vel for mock odometry.')

    def cmd_vel_callback(self, msg):
        if self.use_mock_hardware:
            self.linear_velocity = msg.linear.x
            self.angular_velocity = msg.angular.z
        else:
            # En hardware real, aquí NO conviene usar cmd_vel como odometría.
            # Lo correcto será usar encoders/sensores reales.
            pass

    def publish_odom(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if not self.use_mock_hardware:
            self.linear_velocity = self.read_real_linear_velocity()
            self.angular_velocity = self.read_real_angular_velocity()

        delta_x = self.linear_velocity * math.cos(self.theta) * dt
        delta_y = self.linear_velocity * math.sin(self.theta) * dt
        delta_theta = self.angular_velocity * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        odom_msg.twist.twist.linear.x = self.linear_velocity
        odom_msg.twist.twist.angular.z = self.angular_velocity

        self.odom_pub.publish(odom_msg)

        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(transform)

    def read_real_linear_velocity(self):
        # Aquí después se leerán encoders o datos reales del controlador.
        return 0.0

    def read_real_angular_velocity(self):
        # Aquí después se leerá la velocidad angular real.
        return 0.0


def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
