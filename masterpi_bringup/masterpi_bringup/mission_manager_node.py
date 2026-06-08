#!/usr/bin/env python3

import sys
import threading
import time
from enum import Enum

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray

from masterpi_bringup.srv import RunArmMotion


class StationState(Enum):
    """Station sequence state machine."""
    SEGUIDOR_LINEA = 1      # State 1: Line following mode (default)
    POSICIONAMIENTO = 2     # State 2: Positioning with ArUco
    ACCION_ESTACION = 3     # State 3: Arm action (pickup/drop based on ID)
    RETORNO = 4             # State 4: 180° turn and return to line following


class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager_node')

        self.declare_parameter('manual_enter_trigger', True)
        self.declare_parameter('station_trigger_topic', '/mission/station_trigger')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('line_follower_enabled_topic', '/line_follower/enabled')
        self.declare_parameter('arm_service_name', '/arm/run_motion')
        self.declare_parameter('set_carry_pose_on_start', True)
        self.declare_parameter('enable_line_follower_on_start', True)
        self.declare_parameter('stop_before_action_s', 0.50)
        self.declare_parameter('stop_after_arm_action_s', 0.50)
        self.declare_parameter('stop_after_turn_s', 0.50)
        self.declare_parameter('turn_angular_speed', 1.2)
        self.declare_parameter('turn_duration_s', 1.8)
        self.declare_parameter('turn_direction', 1.0)
        self.declare_parameter('command_rate_hz', 20.0)
        self.declare_parameter('arm_service_timeout_s', 20.0)

        self.declare_parameter('aruco_trigger_enabled', True)
        self.declare_parameter('aruco_detections_topic', '/aruco/detections')
        self.declare_parameter('valid_aruco_ids', [1, 3])
        self.declare_parameter('aruco_trigger_max_distance_m', 1.20)
        self.declare_parameter('aruco_trigger_confirmations', 3)
        self.declare_parameter('aruco_detection_timeout_s', 0.60)
        self.declare_parameter('station_cooldown_s', 4.0)
        self.declare_parameter('ignore_same_marker_s', 6.0)

        # Expected /aruco/detections default format:
        # [id, yaw_error_deg, distance_error_m]
        # distance_error_m = current_distance_m - approach_target_distance_m
        self.declare_parameter('aruco_detection_group_size', 3)
        self.declare_parameter('aruco_id_index', 0)
        self.declare_parameter('aruco_yaw_error_index', 1)
        self.declare_parameter('aruco_distance_index', 2)
        self.declare_parameter('aruco_distance_mode', 'error')  # error or absolute

        self.declare_parameter('require_aruco_positioning', True)
        self.declare_parameter('approach_target_distance_m', 0.25)
        self.declare_parameter('approach_distance_tolerance_m', 0.015)
        self.declare_parameter('approach_max_duration_s', 8.0)
        self.declare_parameter('approach_allow_reverse', False)
        self.declare_parameter('approach_forward_speed', 0.08)
        self.declare_parameter('approach_min_forward_speed', 0.035)
        self.declare_parameter('approach_distance_kp', 0.60)
        self.declare_parameter('approach_yaw_kp', 0.020)
        self.declare_parameter('approach_max_angular_speed', 0.45)
        self.declare_parameter('align_yaw_tolerance_deg', 3.0)
        self.declare_parameter('align_max_duration_s', 4.0)
        self.declare_parameter('align_yaw_kp', 0.030)
        self.declare_parameter('align_max_angular_speed', 0.45)
        self.declare_parameter('yaw_control_sign', -1.0)

        self.manual_enter_trigger = self.get_bool('manual_enter_trigger')
        self.station_trigger_topic = self.get_str('station_trigger_topic')
        self.cmd_vel_topic = self.get_str('cmd_vel_topic')
        self.line_follower_enabled_topic = self.get_str('line_follower_enabled_topic')
        self.arm_service_name = self.get_str('arm_service_name')
        self.set_carry_pose_on_start = self.get_bool('set_carry_pose_on_start')
        self.enable_line_follower_on_start = self.get_bool('enable_line_follower_on_start')
        self.stop_before_action_s = self.get_float('stop_before_action_s')
        self.stop_after_arm_action_s = self.get_float('stop_after_arm_action_s')
        self.stop_after_turn_s = self.get_float('stop_after_turn_s')
        self.turn_angular_speed = self.get_float('turn_angular_speed')
        self.turn_duration_s = self.get_float('turn_duration_s')
        self.turn_direction = self.get_float('turn_direction')
        self.command_rate_hz = self.get_float('command_rate_hz')
        self.arm_service_timeout_s = self.get_float('arm_service_timeout_s')

        self.aruco_trigger_enabled = self.get_bool('aruco_trigger_enabled')
        self.aruco_detections_topic = self.get_str('aruco_detections_topic')
        self.valid_aruco_ids = [int(x) for x in self.get_parameter('valid_aruco_ids').value]
        self.aruco_trigger_max_distance_m = self.get_float('aruco_trigger_max_distance_m')
        self.aruco_trigger_confirmations = self.get_int('aruco_trigger_confirmations')
        self.aruco_detection_timeout_s = self.get_float('aruco_detection_timeout_s')
        self.station_cooldown_s = self.get_float('station_cooldown_s')
        self.ignore_same_marker_s = self.get_float('ignore_same_marker_s')
        self.aruco_detection_group_size = self.get_int('aruco_detection_group_size')
        self.aruco_id_index = self.get_int('aruco_id_index')
        self.aruco_yaw_error_index = self.get_int('aruco_yaw_error_index')
        self.aruco_distance_index = self.get_int('aruco_distance_index')
        self.aruco_distance_mode = self.get_str('aruco_distance_mode').lower().strip()
        self.require_aruco_positioning = self.get_bool('require_aruco_positioning')
        self.approach_target_distance_m = self.get_float('approach_target_distance_m')
        self.approach_distance_tolerance_m = self.get_float('approach_distance_tolerance_m')
        self.approach_max_duration_s = self.get_float('approach_max_duration_s')
        self.approach_allow_reverse = self.get_bool('approach_allow_reverse')
        self.approach_forward_speed = self.get_float('approach_forward_speed')
        self.approach_min_forward_speed = self.get_float('approach_min_forward_speed')
        self.approach_distance_kp = self.get_float('approach_distance_kp')
        self.approach_yaw_kp = self.get_float('approach_yaw_kp')
        self.approach_max_angular_speed = self.get_float('approach_max_angular_speed')
        self.align_yaw_tolerance_deg = self.get_float('align_yaw_tolerance_deg')
        self.align_max_duration_s = self.get_float('align_max_duration_s')
        self.align_yaw_kp = self.get_float('align_yaw_kp')
        self.align_max_angular_speed = self.get_float('align_max_angular_speed')
        self.yaw_control_sign = self.get_float('yaw_control_sign')

        if self.aruco_distance_mode not in ('error', 'absolute'):
            self.get_logger().warn('Invalid aruco_distance_mode. Using error mode.')
            self.aruco_distance_mode = 'error'

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.line_follower_enabled_pub = self.create_publisher(
            Bool,
            self.line_follower_enabled_topic,
            10
        )

        self.station_trigger_sub = self.create_subscription(
            Bool,
            self.station_trigger_topic,
            self.station_trigger_callback,
            10
        )

        self.aruco_detection_sub = None

        if self.aruco_trigger_enabled:
            self.aruco_detection_sub = self.create_subscription(
                Float32MultiArray,
                self.aruco_detections_topic,
                self.aruco_detections_callback,
                10
            )

        self.arm_client = self.create_client(RunArmMotion, self.arm_service_name)

        self.busy = False
        self.pending_station_trigger = False
        self.pending_station_marker_id = None
        self.active_station_marker_id = None
        self.startup_done = False
        self.latest_aruco_detection = None
        self.last_trigger_marker_id = None
        self.current_confirmation_count = 0
        self.last_station_trigger_time = 0.0
        self.ignored_marker_until = {}
        self.station_state = StationState.SEGUIDOR_LINEA

        self.lock = threading.Lock()

        self.timer = self.create_timer(0.10, self.main_timer_callback)

        if self.manual_enter_trigger:
            self.enter_thread = threading.Thread(
                target=self.manual_enter_loop,
                daemon=True
            )
            self.enter_thread.start()

        self.get_logger().info('Mission manager node started.')
        self.get_logger().info(f'ArUco detections topic: {self.aruco_detections_topic}')
        self.get_logger().info(
            'Expected detections: '
            f'[id_index={self.aruco_id_index}, '
            f'yaw_index={self.aruco_yaw_error_index}, '
            f'distance_index={self.aruco_distance_index}], '
            f'distance_mode={self.aruco_distance_mode}'
        )

    def get_bool(self, name):
        return bool(self.get_parameter(name).value)

    def get_int(self, name):
        return int(self.get_parameter(name).value)

    def get_float(self, name):
        return float(self.get_parameter(name).value)

    def get_str(self, name):
        return str(self.get_parameter(name).value)

    def manual_enter_loop(self):
        self.get_logger().info('Press ENTER to simulate station detection.')

        while rclpy.ok():
            try:
                user_input = sys.stdin.readline()

                if user_input == '':
                    time.sleep(0.2)
                    continue

                self.request_station_trigger(source='ENTER', marker_id=None)

            except Exception as exc:
                self.get_logger().warn(f'Manual Enter thread error: {exc}')
                time.sleep(0.5)

    def station_trigger_callback(self, msg):
        if bool(msg.data):
            self.request_station_trigger(
                source=self.station_trigger_topic,
                marker_id=None
            )

    def aruco_detections_callback(self, msg):
        detections = self.parse_aruco_detections(msg.data)
        detections = [d for d in detections if self.is_valid_marker_id(d['id'])]

        if not detections:
            return

        now = time.time()

        best_detection = min(
            detections,
            key=lambda d: abs(d['distance_error_m'])
        )

        best_detection['stamp'] = now

        with self.lock:
            self.latest_aruco_detection = best_detection

        self.maybe_request_aruco_station_trigger(best_detection, now)

    def parse_aruco_detections(self, data):
        values = list(data)
        detections = []

        group_size = max(3, self.aruco_detection_group_size)

        required_max_index = max(
            self.aruco_id_index,
            self.aruco_yaw_error_index,
            self.aruco_distance_index
        )

        for start in range(0, len(values), group_size):
            group = values[start:start + group_size]

            if len(group) <= required_max_index:
                continue

            try:
                marker_id = int(round(float(group[self.aruco_id_index])))
                yaw_error_deg = float(group[self.aruco_yaw_error_index])
                distance_value = float(group[self.aruco_distance_index])
            except (TypeError, ValueError):
                continue

            if self.aruco_distance_mode == 'absolute':
                distance_m = distance_value
                distance_error_m = distance_m - self.approach_target_distance_m
            else:
                distance_error_m = distance_value
                distance_m = self.approach_target_distance_m + distance_error_m

            detections.append({
                'id': marker_id,
                'yaw_error_deg': yaw_error_deg,
                'distance_m': distance_m,
                'distance_error_m': distance_error_m,
            })

        return detections

    def maybe_request_aruco_station_trigger(self, detection, now):
        marker_id = detection['id']

        with self.lock:
            if self.busy or self.pending_station_trigger:
                return

            if now - self.last_station_trigger_time < self.station_cooldown_s:
                return

            if now < self.ignored_marker_until.get(marker_id, 0.0):
                return

            if detection['distance_m'] > self.aruco_trigger_max_distance_m:
                self.current_confirmation_count = 0
                self.last_trigger_marker_id = None
                return

            if self.last_trigger_marker_id == marker_id:
                self.current_confirmation_count += 1
            else:
                self.last_trigger_marker_id = marker_id
                self.current_confirmation_count = 1

            if self.current_confirmation_count < self.aruco_trigger_confirmations:
                return

        self.request_station_trigger(
            source=(
                f'ArUco id={marker_id}, '
                f'distance={detection["distance_m"]:.3f} m, '
                f'distance_error={detection["distance_error_m"]:.3f} m, '
                f'yaw_error={detection["yaw_error_deg"]:.1f} deg'
            ),
            marker_id=marker_id
        )

    def request_station_trigger(self, source='unknown', marker_id=None):
        now = time.time()

        with self.lock:
            if self.busy:
                self.get_logger().warn(
                    f'Trigger ignored from {source}: mission is busy.'
                )
                return

            if now - self.last_station_trigger_time < self.station_cooldown_s:
                self.get_logger().warn(
                    f'Trigger ignored from {source}: cooldown active.'
                )
                return

            self.pending_station_trigger = True
            self.pending_station_marker_id = marker_id
            self.last_station_trigger_time = now
            self.current_confirmation_count = 0

        self.get_logger().info(f'Station trigger received from {source}.')

    def is_valid_marker_id(self, marker_id):
        return not self.valid_aruco_ids or int(marker_id) in self.valid_aruco_ids

    def main_timer_callback(self):
        if not self.startup_done:
            self.startup_done = True
            threading.Thread(
                target=self.startup_sequence,
                daemon=True
            ).start()
            return

        with self.lock:
            if self.busy or not self.pending_station_trigger:
                return

            self.pending_station_trigger = False
            self.busy = True
            self.active_station_marker_id = self.pending_station_marker_id
            self.pending_station_marker_id = None

        threading.Thread(
            target=self.station_sequence_worker,
            daemon=True
        ).start()

    def startup_sequence(self):
        self.get_logger().info('Running mission startup sequence...')

        if not self.wait_for_arm_service():
            self.get_logger().error('Arm service not available during startup.')

        if self.set_carry_pose_on_start:
            if not self.call_arm_motion('carry_line_follower'):
                self.get_logger().warn('Could not set carry_line_follower pose.')

        if self.enable_line_follower_on_start:
            self.set_line_follower_enabled(True)

        self.stop_robot(duration_s=0.20)

        self.get_logger().info(
            'Startup sequence finished. FOLLOW_LINE mode is active.'
        )

    def station_sequence_worker(self):
        """Station sequence state machine."""
        marker_id_for_ignore = self.active_station_marker_id
        current_state = StationState.POSICIONAMIENTO

        try:
            self.get_logger().info('========================================')
            self.get_logger().info('Station sequence started.')
            self.get_logger().info(
                f'Detected ArUco ID: {marker_id_for_ignore}'
            )

            # State 1: SEGUIDOR_LINEA -> disable line follower for station
            self.set_line_follower_enabled(False)

            # State 2: POSICIONAMIENTO
            current_state = StationState.POSICIONAMIENTO
            self.get_logger().info(
                f'Entering state: {current_state.name}'
            )

            if self.require_aruco_positioning:
                positioning_ok = self.position_with_aruco()

                if not positioning_ok:
                    self.get_logger().warn(
                        'ArUco positioning incomplete. Continuing with arm action.'
                    )
            else:
                self.get_logger().info('ArUco positioning disabled.')

            # State 3: ACCION_ESTACION
            current_state = StationState.ACCION_ESTACION
            self.get_logger().info(
                f'Entering state: {current_state.name}'
            )

            self.stop_robot(self.stop_before_action_s)

            # Determine arm motion based on detected ArUco ID
            arm_motion = self._determine_arm_motion(marker_id_for_ignore)
            self.get_logger().info(
                f'ArUco ID {marker_id_for_ignore} detected -> running {arm_motion.upper()}.'
            )

            success = self.call_arm_motion(arm_motion)

            if success:
                self.get_logger().info(
                    f'Arm action finished successfully. Motion: {arm_motion}'
                )
            else:
                self.get_logger().error(
                    f'Arm action failed. Motion: {arm_motion}'
                )

            # State 4: RETORNO
            current_state = StationState.RETORNO
            self.get_logger().info(
                f'Entering state: {current_state.name}'
            )

            self.stop_robot(self.stop_after_arm_action_s)

            self.turn_180()

            self.stop_robot(self.stop_after_turn_s)

            # Mark this ArUco ID as ignored temporarily
            if marker_id_for_ignore is not None:
                with self.lock:
                    self.ignored_marker_until[int(marker_id_for_ignore)] = (
                        time.time() + self.ignore_same_marker_s
                    )

                self.get_logger().info(
                    f'Ignoring ArUco id={marker_id_for_ignore} for '
                    f'{self.ignore_same_marker_s:.1f} s.'
                )

            # Back to State 1: SEGUIDOR_LINEA
            current_state = StationState.SEGUIDOR_LINEA
            self.set_line_follower_enabled(True)

            self.get_logger().info(
                f'Returning to state: {current_state.name}'
            )
            self.get_logger().info(
                'Station sequence finished. FOLLOW_LINE mode active.'
            )
            self.get_logger().info('========================================')

        except Exception as exc:
            self.get_logger().error(
                f'Station sequence failed in state {current_state.name}: {exc}'
            )
            self.stop_robot(duration_s=0.50)
            self.set_line_follower_enabled(False)

        finally:
            with self.lock:
                self.busy = False
                self.active_station_marker_id = None
                self.station_state = StationState.SEGUIDOR_LINEA

    def _determine_arm_motion(self, marker_id):
        """
        Determine arm motion (pickup or drop) based on ArUco marker ID.
        - ArUco ID 3: pickup
        - ArUco ID 1: drop
        """
        if marker_id == 3:
            return 'pickup'
        elif marker_id == 1:
            return 'drop'
        else:
            self.get_logger().warn(
                f'Unknown ArUco ID {marker_id}. Defaulting to pickup.'
            )
            return 'pickup'

    def position_with_aruco(self):
        self.get_logger().info('Starting ArUco positioning.')

        approach_ok = self.approach_aruco_target_distance()

        self.stop_robot(duration_s=0.10)

        align_ok = self.align_aruco_yaw()

        self.stop_robot(duration_s=0.10)

        if approach_ok and align_ok:
            self.get_logger().info('ArUco positioning finished successfully.')
            return True

        self.get_logger().warn(
            f'ArUco positioning incomplete: approach_ok={approach_ok}, '
            f'align_ok={align_ok}'
        )

        return False

    def approach_aruco_target_distance(self):
        self.get_logger().info(
            f'Approaching target distance: {self.approach_target_distance_m:.2f} m.'
        )

        start_time = time.time()
        rate_period = 1.0 / max(1.0, self.command_rate_hz)

        while rclpy.ok() and time.time() - start_time < self.approach_max_duration_s:
            detection = self.get_fresh_aruco_detection()

            if detection is None:
                self.publish_cmd_vel(0.0, 0.0)
                time.sleep(rate_period)
                continue

            distance_error_m = detection['distance_error_m']
            yaw_error_deg = detection['yaw_error_deg']

            if abs(distance_error_m) <= self.approach_distance_tolerance_m:
                self.get_logger().info(
                    f'Target distance reached. distance_error={distance_error_m:.3f} m.'
                )
                self.publish_cmd_vel(0.0, 0.0)
                return True

            if distance_error_m > 0.0:
                linear_x = self.clamp(
                    self.approach_distance_kp * distance_error_m,
                    self.approach_min_forward_speed,
                    self.approach_forward_speed
                )
            elif self.approach_allow_reverse:
                linear_x = self.clamp(
                    self.approach_distance_kp * distance_error_m,
                    -self.approach_forward_speed,
                    -self.approach_min_forward_speed
                )
            else:
                self.get_logger().warn(
                    f'Already closer than target. distance_error={distance_error_m:.3f} m.'
                )
                self.publish_cmd_vel(0.0, 0.0)
                return True

            angular_z = self.clamp(
                self.yaw_control_sign * self.approach_yaw_kp * yaw_error_deg,
                -self.approach_max_angular_speed,
                self.approach_max_angular_speed
            )

            self.publish_cmd_vel(linear_x, angular_z)

            time.sleep(rate_period)

        self.publish_cmd_vel(0.0, 0.0)
        self.get_logger().warn('ArUco approach timeout.')

        return False

    def align_aruco_yaw(self):
        self.get_logger().info('Aligning ArUco yaw.')

        start_time = time.time()
        rate_period = 1.0 / max(1.0, self.command_rate_hz)

        while rclpy.ok() and time.time() - start_time < self.align_max_duration_s:
            detection = self.get_fresh_aruco_detection()

            if detection is None:
                self.publish_cmd_vel(0.0, 0.0)
                time.sleep(rate_period)
                continue

            yaw_error_deg = detection['yaw_error_deg']

            if abs(yaw_error_deg) <= self.align_yaw_tolerance_deg:
                self.get_logger().info(
                    f'ArUco yaw aligned. yaw_error={yaw_error_deg:.1f} deg.'
                )
                self.publish_cmd_vel(0.0, 0.0)
                return True

            angular_z = self.clamp(
                self.yaw_control_sign * self.align_yaw_kp * yaw_error_deg,
                -self.align_max_angular_speed,
                self.align_max_angular_speed
            )

            self.publish_cmd_vel(0.0, angular_z)

            time.sleep(rate_period)

        self.publish_cmd_vel(0.0, 0.0)
        self.get_logger().warn('ArUco yaw alignment timeout.')

        return False

    def get_fresh_aruco_detection(self):
        now = time.time()

        with self.lock:
            detection = self.latest_aruco_detection
            active_marker_id = self.active_station_marker_id

        if detection is None:
            return None

        if now - detection.get('stamp', 0.0) > self.aruco_detection_timeout_s:
            return None

        if active_marker_id is not None and detection['id'] != active_marker_id:
            return None

        return detection

    def publish_cmd_vel(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def stop_robot(self, duration_s=0.0):
        self.get_logger().info(f'Stopping robot for {duration_s:.2f} s.')

        start_time = time.time()
        rate_period = 1.0 / max(1.0, self.command_rate_hz)

        while rclpy.ok():
            self.publish_cmd_vel(0.0, 0.0)

            if duration_s <= 0.0:
                break

            if time.time() - start_time >= duration_s:
                break

            time.sleep(rate_period)

    def turn_180(self):
        angular_z = self.turn_direction * self.turn_angular_speed

        self.get_logger().info(
            f'Turning 180 approx: angular_z={angular_z:.2f}, '
            f'duration={self.turn_duration_s:.2f} s.'
        )

        start_time = time.time()
        rate_period = 1.0 / max(1.0, self.command_rate_hz)

        while rclpy.ok() and time.time() - start_time < self.turn_duration_s:
            self.publish_cmd_vel(0.0, angular_z)
            time.sleep(rate_period)

        self.publish_cmd_vel(0.0, 0.0)

    def set_line_follower_enabled(self, enabled):
        msg = Bool()
        msg.data = bool(enabled)

        self.line_follower_enabled_pub.publish(msg)

        state = 'enabled' if enabled else 'disabled'

        self.get_logger().info(f'Line follower {state}.')

    @staticmethod
    def clamp(value, min_value, max_value):
        return max(min_value, min(max_value, value))

    def wait_for_arm_service(self):
        self.get_logger().info(f'Waiting for arm service: {self.arm_service_name}')

        available = self.arm_client.wait_for_service(
            timeout_sec=self.arm_service_timeout_s
        )

        if available:
            self.get_logger().info('Arm service is available.')
        else:
            self.get_logger().error('Arm service is NOT available.')

        return available

    def call_arm_motion(self, motion_name):
        if not self.wait_for_arm_service():
            return False

        request = RunArmMotion.Request()
        request.motion_name = str(motion_name)
        request.servo_id = 0
        request.pulse = 0
        request.move_time_ms = 0

        self.get_logger().info(f'Calling arm motion service: {motion_name}')

        future = self.arm_client.call_async(request)

        start_time = time.time()

        while rclpy.ok() and not future.done():
            if time.time() - start_time > self.arm_service_timeout_s:
                self.get_logger().error(
                    f'Arm service timeout while running motion "{motion_name}".'
                )
                return False

            time.sleep(0.05)

        if not future.done():
            return False

        response = future.result()

        if response is None:
            self.get_logger().error('Arm service returned no response.')
            return False

        if response.success:
            self.get_logger().info(f'Arm service success: {response.message}')
            return True

        self.get_logger().error(f'Arm service failed: {response.message}')

        return False


def main(args=None):
    rclpy.init(args=args)

    node = MissionManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot(duration_s=0.20)
        node.set_line_follower_enabled(False)
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
