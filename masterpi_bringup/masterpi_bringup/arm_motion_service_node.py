#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

from masterpi_bringup.srv import RunArmMotion

try:
    from masterpi_bringup.hiwonder_sdk import Board, yaml_handle
except Exception:
    try:
        from hiwonder_sdk import Board, yaml_handle
    except Exception:
        Board = None
        yaml_handle = None


DEFAULT_POSES = {
    'home': [1, 1800, 3, 500, 4, 2265, 5, 800, 6, 1500],
    'carry_line_follower': [1, 1200, 3, 500, 4, 2265, 5, 1150, 6, 1500],
}

DEFAULT_SEQUENCES = {
    'home': ['home'],
    'carry_line_follower': ['carry_line_follower'],
    'pickup': [
        'pickup_start',
        'pickup_rotate',
        'pickup_extend',
        'pickup_open',
        'pickup_lower',
        'pickup_grab',
        'pickup_lift',
        'pickup_contract',
        'pickup_center',
        'carry_line_follower',
    ],
    'drop': [
        'drop_rotate',
        'drop_extend',
        'drop_lower',
        'drop_release',
        'drop_lift',
        'drop_contract',
        'drop_center',
        'drop_close',
        'carry_line_follower',
    ],
}


class ArmMotionServiceNode(Node):
    def __init__(self):
        super().__init__('arm_motion_service_node')

        self.declare_parameter('service_name', '/arm/run_motion')

        self.declare_parameter('enable_motion', True)
        self.declare_parameter('use_mock_hardware', False)
        self.declare_parameter('use_deviation', True)

        self.declare_parameter('min_pulse', 500)
        self.declare_parameter('max_pulse', 2500)

        # Direct pose speed. Also used as fallback.
        self.declare_parameter('default_move_time_ms', 400)
        self.declare_parameter('wait_after_pose_s', 0.05)

        # Smooth interpolation for arm movement.
        self.declare_parameter('use_interpolation', True)
        self.declare_parameter('interpolation_steps', 6)
        self.declare_parameter('interpolation_step_time_ms', 120)

        # Gripper keeps its own speed and is not interpolated.
        self.declare_parameter('gripper_servo_id', 1)
        self.declare_parameter('gripper_move_time_ms', 400)

        # Safety lock for damaged/fixed servo.
        self.declare_parameter('fixed_servo_id', 4)
        self.declare_parameter('fixed_servo_pulse', 2265)

        # Default single-servo calibration values.
        self.declare_parameter('single_servo_id', 1)
        self.declare_parameter('single_servo_pulse', 1500)
        self.declare_parameter('single_servo_move_time_ms', 400)

        self.declare_parameter('pose_names', list(DEFAULT_POSES.keys()))
        self.declare_parameter('motion_names', list(DEFAULT_SEQUENCES.keys()) + ['single_servo'])

        self.service_name = str(self.get_parameter('service_name').value)

        self.enable_motion = bool(self.get_parameter('enable_motion').value)
        self.use_mock_hardware = bool(self.get_parameter('use_mock_hardware').value)
        self.use_deviation = bool(self.get_parameter('use_deviation').value)

        self.min_pulse = int(self.get_parameter('min_pulse').value)
        self.max_pulse = int(self.get_parameter('max_pulse').value)

        self.default_move_time_ms = int(self.get_parameter('default_move_time_ms').value)
        self.wait_after_pose_s = float(self.get_parameter('wait_after_pose_s').value)

        self.use_interpolation = bool(self.get_parameter('use_interpolation').value)
        self.interpolation_steps = int(self.get_parameter('interpolation_steps').value)
        self.interpolation_step_time_ms = int(self.get_parameter('interpolation_step_time_ms').value)

        self.gripper_servo_id = int(self.get_parameter('gripper_servo_id').value)
        self.gripper_move_time_ms = int(self.get_parameter('gripper_move_time_ms').value)

        self.fixed_servo_id = int(self.get_parameter('fixed_servo_id').value)
        self.fixed_servo_pulse = int(self.get_parameter('fixed_servo_pulse').value)

        self.single_servo_id = int(self.get_parameter('single_servo_id').value)
        self.single_servo_pulse = int(self.get_parameter('single_servo_pulse').value)
        self.single_servo_move_time_ms = int(self.get_parameter('single_servo_move_time_ms').value)

        self.pose_names = list(self.get_parameter('pose_names').value)
        self.motion_names = list(self.get_parameter('motion_names').value)

        self.poses = {}

        for pose_name in self.pose_names:
            default_pose = DEFAULT_POSES.get(pose_name, DEFAULT_POSES['carry_line_follower'])
            self.declare_parameter(f'pose_{pose_name}', default_pose)
            self.poses[pose_name] = list(self.get_parameter(f'pose_{pose_name}').value)

        self.sequences = {}

        for motion_name in self.motion_names:
            if motion_name == 'single_servo':
                continue

            default_sequence = DEFAULT_SEQUENCES.get(motion_name, [motion_name])
            self.declare_parameter(f'sequence_{motion_name}', default_sequence)
            self.sequences[motion_name] = list(self.get_parameter(f'sequence_{motion_name}').value)

        self.motion_running = False

        self.service = self.create_service(
            RunArmMotion,
            self.service_name,
            self.handle_run_motion
        )

        self.get_logger().info('Arm motion service node started.')
        self.get_logger().info(f'Service: {self.service_name}')
        self.get_logger().info(f'Motions: {sorted(list(self.sequences.keys()) + ["single_servo"])}')
        self.get_logger().info(f'Poses: {sorted(self.poses.keys())}')
        self.get_logger().info(f'Mock hardware: {self.use_mock_hardware}')
        self.get_logger().info(f'Use deviation: {self.use_deviation}')

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

        pose = {}

        for index in range(0, len(raw_pose), 2):
            servo_id = int(raw_pose[index])
            pulse = int(raw_pose[index + 1])
            pose[servo_id] = pulse

        # Safety: fixed servo must always stay fixed.
        pose[self.fixed_servo_id] = self.fixed_servo_pulse

        return pose

    def apply_deviation(self, servo_id, raw_pulse, deviation):
        # For the damaged/fixed servo, force the final output exactly.
        if servo_id == self.fixed_servo_id:
            return self.fixed_servo_pulse

        offset = int(deviation.get(str(servo_id), 0))

        return self.clamp_pulse(raw_pulse + offset)

    def send_servo_dict(self, servo_dict, move_time_ms, deviation, label='servo command'):
        if not servo_dict:
            return

        data = [int(move_time_ms), len(servo_dict)]

        for servo_id in sorted(servo_dict.keys()):
            raw_pulse = int(servo_dict[servo_id])
            final_pulse = self.apply_deviation(servo_id, raw_pulse, deviation)
            data.extend([servo_id, final_pulse])

        self.get_logger().info(f'Sending {label}: {data}')

        if self.use_mock_hardware:
            self.get_logger().info('Mock mode: not sending hardware command.')
            return

        if Board is None:
            raise RuntimeError('Board import failed. Cannot move servos.')

        Board.setPWMServosPulse(data)
        time.sleep(int(move_time_ms) / 1000.0)

    def send_single_servo(self, servo_id, raw_pulse, move_time_ms, deviation):
        servo_id = int(servo_id)
        raw_pulse = int(raw_pulse)
        move_time_ms = int(move_time_ms)

        if servo_id == self.fixed_servo_id and raw_pulse != self.fixed_servo_pulse:
            self.get_logger().warn(
                f'Servo {servo_id} is fixed. Forcing pulse {self.fixed_servo_pulse}.'
            )
            raw_pulse = self.fixed_servo_pulse

        final_pulse = self.apply_deviation(servo_id, raw_pulse, deviation)
        data = [move_time_ms, 1, servo_id, final_pulse]

        self.get_logger().info('Running single servo calibration command:')
        self.get_logger().info(
            f'  servo {servo_id}: raw={raw_pulse}, final={final_pulse}, time_ms={move_time_ms}'
        )

        if self.use_mock_hardware:
            self.get_logger().info(f'Mock mode: not sending hardware command. data={data}')
            return

        if Board is None:
            raise RuntimeError('Board import failed. Cannot move servo.')

        Board.setPWMServosPulse(data)
        self.get_logger().info('Single servo command sent.')
        time.sleep(move_time_ms / 1000.0)

    def smoothstep(self, value):
        # Ease-in/ease-out curve for smoother motion.
        return value * value * (3.0 - 2.0 * value)

    def interpolate_pose(self, previous_pose, target_pose, deviation, pose_name):
        steps = max(1, self.interpolation_steps)
        step_time_ms = max(20, self.interpolation_step_time_ms)

        arm_servo_ids = sorted(
            servo_id for servo_id in target_pose.keys()
            if servo_id != self.gripper_servo_id
        )

        gripper_changed = (
            self.gripper_servo_id in target_pose
            and previous_pose.get(self.gripper_servo_id) != target_pose.get(self.gripper_servo_id)
        )

        arm_changed = any(
            previous_pose.get(servo_id) != target_pose.get(servo_id)
            for servo_id in arm_servo_ids
        )

        if arm_changed:
            self.get_logger().info(
                f'Interpolating arm movement to pose "{pose_name}" '
                f'with {steps} steps, {step_time_ms} ms each.'
            )

            for step in range(1, steps + 1):
                ratio = self.smoothstep(step / steps)
                interpolated = {}

                for servo_id in arm_servo_ids:
                    start = int(previous_pose.get(servo_id, target_pose[servo_id]))
                    end = int(target_pose[servo_id])

                    if servo_id == self.fixed_servo_id:
                        pulse = self.fixed_servo_pulse
                    else:
                        pulse = int(round(start + (end - start) * ratio))

                    interpolated[servo_id] = pulse

                self.send_servo_dict(
                    interpolated,
                    step_time_ms,
                    deviation,
                    label=f'interpolation step {step}/{steps} for {pose_name}'
                )

        if gripper_changed:
            self.get_logger().info(
                f'Moving gripper for pose "{pose_name}" at fixed speed '
                f'{self.gripper_move_time_ms} ms.'
            )
            self.send_servo_dict(
                {self.gripper_servo_id: target_pose[self.gripper_servo_id]},
                self.gripper_move_time_ms,
                deviation,
                label=f'gripper move for {pose_name}'
            )

        if not arm_changed and not gripper_changed:
            self.get_logger().info(f'Pose "{pose_name}" already matches previous pose.')

    def send_pose_direct(self, pose_name, target_pose, deviation):
        self.get_logger().info(f'Applying first/direct pose: {pose_name}')
        self.send_servo_dict(
            target_pose,
            self.default_move_time_ms,
            deviation,
            label=f'direct pose {pose_name}'
        )

    def execute_motion(self, motion_name, servo_id=0, pulse=0, move_time_ms=0):
        if not self.enable_motion:
            return False, 'Arm motion disabled by parameter enable_motion=false.'

        motion_name = str(motion_name).strip()

        if not motion_name:
            return False, 'motion_name is empty.'

        deviation = self.get_deviation()

        if motion_name == 'single_servo':
            servo_id = int(servo_id) if int(servo_id) > 0 else self.single_servo_id
            pulse = int(pulse) if int(pulse) > 0 else self.single_servo_pulse
            move_time_ms = (
                int(move_time_ms)
                if int(move_time_ms) > 0
                else self.single_servo_move_time_ms
            )

            self.send_single_servo(
                servo_id,
                pulse,
                move_time_ms,
                deviation
            )

            return True, 'Single servo calibration finished.'

        if motion_name not in self.sequences:
            available = ', '.join(sorted(list(self.sequences.keys()) + ['single_servo']))
            return False, f'Unknown motion "{motion_name}". Available motions: {available}'

        sequence = self.sequences[motion_name]

        self.get_logger().info(f'Running arm motion: {motion_name}')
        self.get_logger().info(f'Sequence: {sequence}')
        self.get_logger().info(
            f'Interpolation: {self.use_interpolation}, '
            f'steps={self.interpolation_steps}, '
            f'step_time_ms={self.interpolation_step_time_ms}, '
            f'gripper_time_ms={self.gripper_move_time_ms}'
        )

        previous_pose = None

        for pose_name in sequence:
            if pose_name not in self.poses:
                available = ', '.join(sorted(self.poses.keys()))
                return False, f'Unknown pose "{pose_name}". Available poses: {available}'

            target_pose = self.parse_pose(pose_name, self.poses[pose_name])

            if previous_pose is None or not self.use_interpolation:
                self.send_pose_direct(pose_name, target_pose, deviation)
            else:
                self.interpolate_pose(previous_pose, target_pose, deviation, pose_name)

            previous_pose = target_pose
            time.sleep(self.wait_after_pose_s)

        return True, f'Motion "{motion_name}" finished.'

    def handle_run_motion(self, request, response):
        if self.motion_running:
            response.success = False
            response.message = 'Arm motion service is busy.'
            return response

        self.motion_running = True

        try:
            success, message = self.execute_motion(
                request.motion_name,
                request.servo_id,
                request.pulse,
                request.move_time_ms
            )

            response.success = bool(success)
            response.message = str(message)

            if success:
                self.get_logger().info(message)
            else:
                self.get_logger().error(message)

            return response

        except Exception as exc:
            response.success = False
            response.message = f'Arm motion failed: {exc}'
            self.get_logger().error(response.message)
            return response

        finally:
            self.motion_running = False


def main(args=None):
    rclpy.init(args=args)
    node = ArmMotionServiceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
