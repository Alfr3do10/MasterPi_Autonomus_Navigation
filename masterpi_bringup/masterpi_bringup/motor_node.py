#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')

        self.declare_parameter('max_linear_speed', 0.30)          # ROS command limit in m/s
        self.declare_parameter('max_angular_speed', 1.5)          # ROS command limit in rad/s
        self.declare_parameter('max_hardware_velocity', 90.0)     # Hiwonder forward/side speed command
        self.declare_parameter('max_hardware_angular_rate', 0.6)  # Hiwonder angular command
        self.declare_parameter('angular_direction_multiplier', 1.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('use_mock_hardware', False)

        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.max_hardware_velocity = float(self.get_parameter('max_hardware_velocity').value)
        self.max_hardware_angular_rate = float(self.get_parameter('max_hardware_angular_rate').value)
        self.angular_direction_multiplier = float(self.get_parameter('angular_direction_multiplier').value)
        self.cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)
        self.use_mock_hardware = bool(self.get_parameter('use_mock_hardware').value)

        self.chassis = None
        if not self.use_mock_hardware:
            try:
                from masterpi_bringup.hiwonder_sdk import mecanum
                self.chassis = mecanum.MecanumChassis()
                self.get_logger().info('Hiwonder mecanum chassis initialized.')
                self.stop_motors()
            except Exception as exc:
                self.get_logger().error(f'Could not initialize real chassis: {exc}')
                raise

        self.last_cmd_time = self.get_clock().now()
        self.robot_stopped = True

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.safety_timer = self.create_timer(0.1, self.check_cmd_vel_timeout)

        self.get_logger().info('Motor node started.')
        self.get_logger().info(f'use_mock_hardware = {self.use_mock_hardware}')
        self.get_logger().info(f'cmd_vel_timeout = {self.cmd_vel_timeout} s')

    def clamp(self, value, limit):
        return max(min(value, limit), -limit)

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()

        linear_x = self.clamp(float(msg.linear.x), self.max_linear_speed)
        linear_y = self.clamp(float(msg.linear.y), self.max_linear_speed)
        angular_z = self.clamp(float(msg.angular.z), self.max_angular_speed)

        velocity, direction_deg = self.ros_linear_to_hiwonder_polar(linear_x, linear_y)
        angular_rate = self.ros_angular_to_hiwonder(angular_z)

        self.robot_stopped = False

        if self.use_mock_hardware:
            self.get_logger().info(
                f'CMD /cmd_vel | linear_x={linear_x:.2f}, linear_y={linear_y:.2f}, angular_z={angular_z:.2f} '
                f'=> hw velocity={velocity:.1f}, direction={direction_deg:.1f}, angular_rate={angular_rate:.2f}'
            )
        else:
            self.send_to_motors(velocity, direction_deg, angular_rate)

    def ros_linear_to_hiwonder_polar(self, linear_x, linear_y):
        """
        ROS convention:
          linear.x = forward/backward
          linear.y = left/right
        Hiwonder mecanum convention from demos:
          direction 90  = forward
          direction 270 = backward
          direction 180 = left
          direction 0   = right
        """
        if abs(linear_x) < 1e-6 and abs(linear_y) < 1e-6:
            return 0.0, 90.0

        norm = math.hypot(linear_x, linear_y)
        norm = min(norm, self.max_linear_speed)
        velocity = (norm / self.max_linear_speed) * self.max_hardware_velocity

        # Convert ROS x/y into Hiwonder polar axis.
        hw_x = -linear_y  # +ROS y left -> Hiwonder direction 180
        hw_y = linear_x   # +ROS x forward -> Hiwonder direction 90
        direction = math.degrees(math.atan2(hw_y, hw_x))
        if direction < 0:
            direction += 360.0

        return velocity, direction

    def ros_angular_to_hiwonder(self, angular_z):
        if abs(angular_z) < 1e-6:
            return 0.0
        rate = (angular_z / self.max_angular_speed) * self.max_hardware_angular_rate
        rate *= self.angular_direction_multiplier
        return self.clamp(rate, self.max_hardware_angular_rate)

    def check_cmd_vel_timeout(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.cmd_vel_timeout and not self.robot_stopped:
            self.stop_motors()
            self.robot_stopped = True
            self.get_logger().warn('cmd_vel timeout. Motors stopped.')

    def send_to_motors(self, velocity, direction_deg, angular_rate):
        self.chassis.set_velocity(velocity, direction_deg, angular_rate)

    def stop_motors(self):
        if self.use_mock_hardware:
            self.get_logger().warn('Stopping mock motors.')
        else:
            if self.chassis is not None:
                self.chassis.set_velocity(0, 0, 0)

    def destroy_node(self):
        self.stop_motors()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motors()
        node.destroy_node()
        rclpy.shutdown()
