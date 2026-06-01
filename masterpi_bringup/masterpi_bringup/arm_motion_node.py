#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

try:
    from masterpi_bringup.hiwonder_sdk import Board, yaml_handle
except Exception:
    Board = None
    yaml_handle = None


DEFAULT_POSES = {
    'home': [1, 1500, 3, 500, 4, 2265, 5, 1080, 6, 1500],
    'pickup_ready': [1, 1500, 3, 500, 4, 2265, 5, 1080, 6, 1500],
    'pickup_down': [1, 1500, 3, 500, 4, 2265, 5, 1080, 6, 1500],
    'pickup_grab': [1, 1500, 3, 500, 4, 2265, 5, 1080, 6, 1500],
    'carry_line_follower': [1, 1500, 3, 500, 4, 2265, 5, 1080, 6, 1500],
    'drop_ready': [1, 1500, 3, 500, 4, 2265, 5, 1080, 6, 1500],
    'drop_down': [1, 1500, 3, 500, 4, 2265, 5, 1080, 6, 1500],
    'drop_release': [1, 1500, 3, 500, 4, 2265, 5, 1080, 6, 1500],
}

DEFAULT_SEQUENCES = {
    'home': ['home'],
    'carry_line_follower': ['carry_line_follower'],
    'pickup': ['pickup_ready', 'pickup_down', 'pickup_grab', 'carry_line_follower'],
    'drop': ['drop_ready', 'drop_down', 'drop_release', 'carry_line_follower'],
}


class ArmMotionNode(Node):
    def __init__(self):
        super().__init__('arm_motion_node')

        self.declare_parameter('enable_motion', True)
        self.declare_parameter('motion_name', 'carry_line_follower')

        self.declare_parameter('use_mock_hardware', False)
        self.declare_parameter('use_deviation', True)

        self.declare_parameter('min_pulse', 500)
        self.declare_parameter('max_pulse', 2500)
        self.declare_parameter('default_move_time_ms', 1000)
        self.declare_parameter('wait_after_pose_s', 0.25)

        self.declare_parameter('pose_names', list(DEFAULT_POSES.keys()))
        self.declare_parameter('motion_names', list(DEFAULT_SEQUENCES.keys()))

        self.enable_motion = bool(self.get_parameter('enable_motion').value)
        self.motion_name = str(self.get_parameter('motion_name').value)

        self.use_mock_hardware = bool(self.get_parameter('use_mock_hardware').value)
        self.use_deviation = bool(self.get_parameter('use_deviation').value)

        self.min_pulse = int(self.get_parameter('min_pulse').value)
        self.max_pulse = int(self.get_parameter('max_pulse').value)
        self.default_move_time_ms = int(self.get_parameter('default_move_time_ms').value)
        self.wait_after_pose_s = float(self.get_parameter('wait_after_pose_s').value)

        self.pose_names = list(self.get_parameter('pose_names').value)
        self.motion_names = list(self.get_parameter('motion_names').value)

        self.poses = {}
        for pose_name in self.pose_names:
            default_pose = DEFAULT_POSES.get(pose_name, DEFAULT_POSES['carry_line_follower'])
            self.declare_parameter(f'pose_{pose_name}', default_pose)
            self.poses[pose_name] = list(self.get_parameter(f'pose_{pose_name}').value)

        self.sequences = {}
        for motion_name in self.motion_names:
            default_sequence = DEFAULT_SEQUENCES.get(motion_name, [motion_name])
            self.declare_parameter(f'sequence_{motion_name}', default_sequence)
            self.sequences[motion_name] = list(self.get_parameter(f'sequence_{motion_name}').value)

    def clamp_pulse(self, pulse):
        pulse = int(pulse)
        if pulse < self.min_pulse:
            return self.min_pulse
        if pulse > self.max_pulse:
            return self.max_pulse
        return pulse

    def get_deviation(self):
        if not self.use_deviation:
            return {}

        if yaml_handle is None:
            self.get_logger().warn('yaml_handle import failed. Servo deviations will not be used.')
            return {}

        try:
            deviation = yaml_handle.get_yaml_data(yaml_handle.Deviation_file_path)
            if deviation is None:
                return {}
            return deviation
        except Exception as exc:
            self.get_logger().warn(f'Could not load servo deviation file: {exc}')
            return {}

    def parse_pose(self, pose_name, raw_pose):
        if len(raw_pose) % 2 != 0:
            raise ValueError(
                f'Pose "{pose_name}" has invalid format. Expected pairs: [servo_id, pulse, ...]'
            )

        servo_pairs = []
        for index in range(0, len(raw_pose), 2):
            servo_id = int(raw_pose[index])
            pulse = int(raw_pose[index + 1])
            servo_pairs.append((servo_id, pulse))

        return servo_pairs

    def send_pose(self, pose_name, raw_pose, deviation):
        servo_pairs = self.parse_pose(pose_name, raw_pose)

        data = [self.default_move_time_ms, len(servo_pairs)]

        self.get_logger().info(f'Applying pose: {pose_name}')

        for servo_id, raw_pulse in servo_pairs:
            offset = int(deviation.get(str(servo_id), 0))
            final_pulse = self.clamp_pulse(raw_pulse + offset)

            data.extend([servo_id, final_pulse])

            self.get_logger().info(
                f'  servo {servo_id}: raw={raw_pulse}, deviation={offset}, final={final_pulse}'
            )

        if self.use_mock_hardware:
            self.get_logger().info(f'Mock mode: not sending hardware command. data={data}')
            return

        if Board is None:
            self.get_logger().error('Board import failed. Cannot move servos.')
            return

        Board.setPWMServosPulse(data)
        self.get_logger().info(
            f'Pose "{pose_name}" command sent. Waiting {self.default_move_time_ms} ms.'
        )
        time.sleep(self.default_move_time_ms / 1000.0)

    def run_motion(self):
        if not self.enable_motion:
            self.get_logger().info('Arm motion disabled.')
            return

        if self.motion_name not in self.sequences:
            available = ', '.join(sorted(self.sequences.keys()))
            self.get_logger().error(
                f'Unknown motion "{self.motion_name}". Available motions: {available}'
            )
            return

        deviation = self.get_deviation()
        sequence = self.sequences[self.motion_name]

        self.get_logger().info(f'Running arm motion: {self.motion_name}')
        self.get_logger().info(f'Sequence: {sequence}')

        for pose_name in sequence:
            if pose_name not in self.poses:
                available = ', '.join(sorted(self.poses.keys()))
                self.get_logger().error(
                    f'Unknown pose "{pose_name}". Available poses: {available}'
                )
                return

            self.send_pose(pose_name, self.poses[pose_name], deviation)
            time.sleep(self.wait_after_pose_s)

        self.get_logger().info(f'Arm motion "{self.motion_name}" finished.')


def main(args=None):
    rclpy.init(args=args)
    node = ArmMotionNode()

    try:
        node.run_motion()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
