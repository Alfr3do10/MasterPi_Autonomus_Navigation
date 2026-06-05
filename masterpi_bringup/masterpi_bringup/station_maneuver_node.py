#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class StationManeuverNode(Node):
    def __init__(self):
        super().__init__('station_maneuver_node')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.declare_parameter('stop_before_turn_s', 0.50)
        self.declare_parameter('stop_after_turn_s', 0.50)

        self.declare_parameter('turn_angular_speed', 1.0)
        self.declare_parameter('turn_duration_s', 1.70)
        self.declare_parameter('turn_direction', 1.0)

        self.declare_parameter('command_rate_hz', 20.0)

        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        self.stop_before_turn_s = float(
            self.get_parameter('stop_before_turn_s').value
        )
        self.stop_after_turn_s = float(
            self.get_parameter('stop_after_turn_s').value
        )

        self.turn_angular_speed = float(
            self.get_parameter('turn_angular_speed').value
        )
        self.turn_duration_s = float(
            self.get_parameter('turn_duration_s').value
        )
        self.turn_direction = float(
            self.get_parameter('turn_direction').value
        )

        self.command_rate_hz = float(
            self.get_parameter('command_rate_hz').value
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.get_logger().info('Station maneuver node started.')
        self.get_logger().info(f'Publishing cmd_vel: {self.cmd_vel_topic}')
        self.get_logger().info(
            f'Stop before turn: {self.stop_before_turn_s:.2f} s'
        )
        self.get_logger().info(
            f'Turn: speed={self.turn_angular_speed:.2f}, '
            f'duration={self.turn_duration_s:.2f} s, '
            f'direction={self.turn_direction:.2f}'
        )
        self.get_logger().info(
            f'Stop after turn: {self.stop_after_turn_s:.2f} s'
        )

    def publish_cmd_vel(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def stop_robot(self, duration_s):
        self.get_logger().info(f'Stopping robot for {duration_s:.2f} s.')

        start_time = time.time()
        rate_period = 1.0 / max(1.0, self.command_rate_hz)

        while rclpy.ok():
            self.publish_cmd_vel(0.0, 0.0)

            if time.time() - start_time >= duration_s:
                break

            time.sleep(rate_period)

    def turn_180(self):
        angular_z = self.turn_direction * self.turn_angular_speed

        self.get_logger().info(
            f'Turning 180 approx with angular.z={angular_z:.2f} '
            f'for {self.turn_duration_s:.2f} s.'
        )

        start_time = time.time()
        rate_period = 1.0 / max(1.0, self.command_rate_hz)

        while rclpy.ok() and (time.time() - start_time) < self.turn_duration_s:
            self.publish_cmd_vel(0.0, angular_z)
            time.sleep(rate_period)

        self.publish_cmd_vel(0.0, 0.0)

    def run_maneuver(self):
        self.get_logger().info('Running station maneuver: stop -> turn_180 -> stop.')

        self.stop_robot(self.stop_before_turn_s)
        self.turn_180()
        self.stop_robot(self.stop_after_turn_s)

        self.get_logger().info('Station maneuver finished.')


def main(args=None):
    rclpy.init(args=args)
    node = StationManeuverNode()

    try:
        node.run_maneuver()
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_cmd_vel(0.0, 0.0)
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
