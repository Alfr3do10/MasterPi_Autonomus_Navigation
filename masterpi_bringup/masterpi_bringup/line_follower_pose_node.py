#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

from masterpi_bringup.hiwonder_sdk import Board, yaml_handle


class LineFollowerPoseNode(Node):
    def __init__(self):
        super().__init__('line_follower_pose_node')

        self.declare_parameter('enable_pose_init', True)
        self.declare_parameter('move_time_ms', 1000)
        self.declare_parameter('use_deviation', True)

        self.declare_parameter('servo_1', 1500)
        self.declare_parameter('servo_3', 500)
        self.declare_parameter('servo_4', 2265)
        self.declare_parameter('servo_5', 1080)
        self.declare_parameter('servo_6', 1500)

        self.enable_pose_init = bool(self.get_parameter('enable_pose_init').value)
        self.move_time_ms = int(self.get_parameter('move_time_ms').value)
        self.use_deviation = bool(self.get_parameter('use_deviation').value)

        self.pose = {
            1: int(self.get_parameter('servo_1').value),
            3: int(self.get_parameter('servo_3').value),
            4: int(self.get_parameter('servo_4').value),
            5: int(self.get_parameter('servo_5').value),
            6: int(self.get_parameter('servo_6').value),
        }

    def clamp_pulse(self, pulse):
        return max(500, min(2500, int(pulse)))

    def get_deviation(self):
        if not self.use_deviation:
            return {}

        try:
            return yaml_handle.get_yaml_data(yaml_handle.Deviation_file_path)
        except Exception as exc:
            self.get_logger().warn(f'Could not load servo deviation file: {exc}')
            return {}

    def apply_pose(self):
        if not self.enable_pose_init:
            self.get_logger().info('Pose initialization disabled.')
            return

        deviation = self.get_deviation()

        data = [self.move_time_ms, len(self.pose)]

        self.get_logger().info('Moving arm/camera to line follower pose:')

        for servo_id in sorted(self.pose.keys()):
            raw_pulse = self.pose[servo_id]
            offset = int(deviation.get(str(servo_id), 0))
            final_pulse = self.clamp_pulse(raw_pulse + offset)

            data.extend([servo_id, final_pulse])

            self.get_logger().info(
                f'  servo {servo_id}: raw={raw_pulse}, deviation={offset}, final={final_pulse}'
            )

        Board.setPWMServosPulse(data)

        self.get_logger().info(f'Pose command sent. Waiting {self.move_time_ms} ms.')
        time.sleep(self.move_time_ms / 1000.0)
        self.get_logger().info('Line follower arm/camera pose ready.')


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerPoseNode()

    try:
        node.apply_pose()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
