#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import Bool


class SonarSafetyNode(Node):
    def __init__(self):
        super().__init__('sonar_safety_node')

        self.declare_parameter('sonar_range_topic', '/sonar/range')
        self.declare_parameter('line_follower_enabled_topic', '/line_follower/enabled')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.declare_parameter('stop_distance_m', 0.10)
        self.declare_parameter('clear_distance_m', 0.15)

        self.declare_parameter('obstacle_confirmations', 3)
        self.declare_parameter('clear_confirmations', 5)

        self.declare_parameter('resume_delay_s', 0.30)
        self.declare_parameter('command_rate_hz', 20.0)
        self.declare_parameter('log_period_s', 1.0)

        self.declare_parameter('assume_clear_on_inf', True)

        self.sonar_range_topic = str(self.get_parameter('sonar_range_topic').value)
        self.line_follower_enabled_topic = str(
            self.get_parameter('line_follower_enabled_topic').value
        )
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        self.stop_distance_m = float(self.get_parameter('stop_distance_m').value)
        self.clear_distance_m = float(self.get_parameter('clear_distance_m').value)

        self.obstacle_confirmations = int(
            self.get_parameter('obstacle_confirmations').value
        )
        self.clear_confirmations = int(
            self.get_parameter('clear_confirmations').value
        )

        self.resume_delay_s = float(self.get_parameter('resume_delay_s').value)
        self.command_rate_hz = float(self.get_parameter('command_rate_hz').value)
        self.log_period_s = float(self.get_parameter('log_period_s').value)

        self.assume_clear_on_inf = bool(self.get_parameter('assume_clear_on_inf').value)

        if self.clear_distance_m <= self.stop_distance_m:
            self.get_logger().warn(
                'clear_distance_m should be greater than stop_distance_m. '
                'Forcing clear_distance_m = stop_distance_m + 0.05.'
            )
            self.clear_distance_m = self.stop_distance_m + 0.05

        self.line_follower_enabled_pub = self.create_publisher(
            Bool,
            self.line_follower_enabled_topic,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )

        self.sonar_sub = self.create_subscription(
            Range,
            self.sonar_range_topic,
            self.sonar_callback,
            10
        )

        self.line_follower_enabled_sub = self.create_subscription(
            Bool,
            self.line_follower_enabled_topic,
            self.line_follower_enabled_callback,
            10
        )

        self.timer = self.create_timer(
            1.0 / max(1.0, self.command_rate_hz),
            self.timer_callback
        )

        self.line_follower_enabled = False

        self.paused_by_sonar = False
        self.obstacle_active = False

        self.obstacle_count = 0
        self.clear_count = 0

        self.last_range_m = math.inf
        self.last_log_time = 0.0
        self.resume_ready_time = 0.0

        self.last_own_enable_command_time = 0.0
        self.last_own_enable_command_value = None

        self.external_pause_detected = False

        self.get_logger().info('Sonar safety node started.')
        self.get_logger().info(
            f'Sonar topic={self.sonar_range_topic} | '
            f'line follower topic={self.line_follower_enabled_topic} | '
            f'stop={self.stop_distance_m:.2f} m | '
            f'clear={self.clear_distance_m:.2f} m'
        )

    def line_follower_enabled_callback(self, msg):
        now = time.time()
        value = bool(msg.data)

        self.line_follower_enabled = value

        own_recent_command = (
            self.last_own_enable_command_value == value
            and now - self.last_own_enable_command_time < 0.30
        )

        if self.paused_by_sonar and not own_recent_command:
            # Otro nodo cambió el estado del line follower mientras sonar pausaba.
            # Normalmente sería mission_manager. En ese caso, sonar no debe reactivar solo.
            self.external_pause_detected = True

    def sonar_callback(self, msg):
        range_m = float(msg.range)

        if not math.isfinite(range_m) or range_m <= 0.0:
            if self.assume_clear_on_inf:
                range_m = math.inf
            else:
                return

        self.last_range_m = range_m

        if range_m <= self.stop_distance_m:
            self.obstacle_count += 1
            self.clear_count = 0
        elif range_m >= self.clear_distance_m:
            self.clear_count += 1
            self.obstacle_count = 0
        else:
            # Zona de histéresis entre 10 cm y 15 cm.
            # No cambia estado para evitar prendido/apagado por ruido.
            return

        if (
            not self.obstacle_active
            and self.obstacle_count >= self.obstacle_confirmations
        ):
            self.activate_pause(range_m)
            return

        if (
            self.obstacle_active
            and self.clear_count >= self.clear_confirmations
        ):
            self.clear_pause(range_m)

    def activate_pause(self, range_m):
        self.obstacle_active = True
        self.paused_by_sonar = True
        self.external_pause_detected = False
        self.resume_ready_time = 0.0

        self.get_logger().warn(
            f'Obstacle detected at {range_m:.3f} m. '
            'Disabling line follower and stopping robot.'
        )

        self.publish_line_follower_enabled(False)
        self.publish_stop()

    def clear_pause(self, range_m):
        self.obstacle_active = False
        self.resume_ready_time = time.time() + self.resume_delay_s

        self.get_logger().info(
            f'Obstacle cleared at {range_m:.3f} m. '
            f'Waiting {self.resume_delay_s:.2f} s before resuming.'
        )

        self.publish_stop()

    def timer_callback(self):
        if not self.paused_by_sonar:
            return

        now = time.time()

        # Mientras sonar tiene el control de seguridad, sigue mandando cero.
        # Así motor_node no conserva el último /cmd_vel.
        self.publish_stop()

        if self.obstacle_active:
            self.log_hold(now)
            return

        if now < self.resume_ready_time:
            return

        if self.external_pause_detected:
            self.log_external_pause(now)
            return

        self.get_logger().info('Resuming line follower after sonar pause.')

        self.publish_line_follower_enabled(True)

        self.paused_by_sonar = False
        self.obstacle_active = False
        self.obstacle_count = 0
        self.clear_count = 0
        self.external_pause_detected = False

    def publish_line_follower_enabled(self, enabled):
        msg = Bool()
        msg.data = bool(enabled)

        self.last_own_enable_command_value = bool(enabled)
        self.last_own_enable_command_time = time.time()

        self.line_follower_enabled_pub.publish(msg)

    def publish_stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

    def log_hold(self, now):
        if now - self.last_log_time < self.log_period_s:
            return

        self.last_log_time = now

        if math.isfinite(self.last_range_m):
            self.get_logger().warn(
                f'Sonar hold active. range={self.last_range_m:.3f} m.'
            )
        else:
            self.get_logger().warn('Sonar hold active. range=inf.')

    def log_external_pause(self, now):
        if now - self.last_log_time < self.log_period_s:
            return

        self.last_log_time = now
        self.get_logger().warn(
            'Sonar pause cleared, but another node changed line follower state. '
            'Waiting instead of resuming automatically.'
        )


def main(args=None):
    rclpy.init(args=args)

    node = SonarSafetyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
