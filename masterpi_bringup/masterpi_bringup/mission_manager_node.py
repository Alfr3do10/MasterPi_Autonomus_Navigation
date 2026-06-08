#!/usr/bin/env python3

import sys
import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from masterpi_bringup.srv import RunArmMotion


class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager_node')

        # ----------------------------------------------------------------------
        # Parameters
        # ----------------------------------------------------------------------
        self.declare_parameter('manual_enter_trigger', True)
        self.declare_parameter('station_trigger_topic', '/mission/station_trigger')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('line_follower_enabled_topic', '/line_follower/enabled')

        self.declare_parameter('arm_service_name', '/arm/run_motion')

        self.declare_parameter('initial_has_cube', False)

        self.declare_parameter('set_carry_pose_on_start', True)
        self.declare_parameter('enable_line_follower_on_start', True)

        self.declare_parameter('stop_before_action_s', 0.50)
        self.declare_parameter('stop_after_arm_action_s', 0.50)
        self.declare_parameter('stop_after_turn_s', 0.50)

        self.declare_parameter('turn_angular_speed', 1.0)
        self.declare_parameter('turn_duration_s', 1.70)
        self.declare_parameter('turn_direction', 1.0)

        self.declare_parameter('command_rate_hz', 20.0)

        self.declare_parameter('arm_service_timeout_s', 10.0)

        self.manual_enter_trigger = bool(
            self.get_parameter('manual_enter_trigger').value
        )
        self.station_trigger_topic = str(
            self.get_parameter('station_trigger_topic').value
        )

        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.line_follower_enabled_topic = str(
            self.get_parameter('line_follower_enabled_topic').value
        )

        self.arm_service_name = str(self.get_parameter('arm_service_name').value)

        self.has_cube = bool(self.get_parameter('initial_has_cube').value)

        self.set_carry_pose_on_start = bool(
            self.get_parameter('set_carry_pose_on_start').value
        )
        self.enable_line_follower_on_start = bool(
            self.get_parameter('enable_line_follower_on_start').value
        )

        self.stop_before_action_s = float(
            self.get_parameter('stop_before_action_s').value
        )
        self.stop_after_arm_action_s = float(
            self.get_parameter('stop_after_arm_action_s').value
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

        self.arm_service_timeout_s = float(
            self.get_parameter('arm_service_timeout_s').value
        )

        # ----------------------------------------------------------------------
        # ROS interfaces
        # ----------------------------------------------------------------------
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

        self.arm_client = self.create_client(
            RunArmMotion,
            self.arm_service_name
        )

        # ----------------------------------------------------------------------
        # Internal state
        # ----------------------------------------------------------------------
        self.busy = False
        self.pending_station_trigger = False
        self.startup_done = False

        self.lock = threading.Lock()

        self.timer = self.create_timer(0.10, self.main_timer_callback)

        if self.manual_enter_trigger:
            self.enter_thread = threading.Thread(
                target=self.manual_enter_loop,
                daemon=True
            )
            self.enter_thread.start()

        self.get_logger().info('Mission manager node started.')
        self.get_logger().info(f'Manual Enter trigger: {self.manual_enter_trigger}')
        self.get_logger().info(f'Station trigger topic: {self.station_trigger_topic}')
        self.get_logger().info(f'cmd_vel topic: {self.cmd_vel_topic}')
        self.get_logger().info(
            f'Line follower enabled topic: {self.line_follower_enabled_topic}'
        )
        self.get_logger().info(f'Arm service: {self.arm_service_name}')
        self.get_logger().info(f'Initial has_cube: {self.has_cube}')
        self.get_logger().info(
            f'Turn 180 approx: speed={self.turn_angular_speed}, '
            f'duration={self.turn_duration_s}, direction={self.turn_direction}'
        )

    # --------------------------------------------------------------------------
    # Trigger inputs
    # --------------------------------------------------------------------------
    def manual_enter_loop(self):
        self.get_logger().info(
            'Press ENTER in this terminal to simulate ArUco station detection.'
        )

        while rclpy.ok():
            try:
                user_input = sys.stdin.readline()

                if user_input == '':
                    time.sleep(0.2)
                    continue

                self.request_station_trigger(source='ENTER')

            except Exception as exc:
                self.get_logger().warn(f'Manual Enter thread error: {exc}')
                time.sleep(0.5)

    def station_trigger_callback(self, msg):
        if bool(msg.data):
            self.request_station_trigger(source=self.station_trigger_topic)

    def request_station_trigger(self, source='unknown'):
        with self.lock:
            if self.busy:
                self.get_logger().warn(
                    f'Station trigger ignored from {source}: mission is busy.'
                )
                return

            self.pending_station_trigger = True

        self.get_logger().info(f'Station trigger received from {source}.')

    # --------------------------------------------------------------------------
    # Timer / worker orchestration
    # --------------------------------------------------------------------------
    def main_timer_callback(self):
        if not self.startup_done:
            self.startup_done = True
            threading.Thread(target=self.startup_sequence, daemon=True).start()
            return

        with self.lock:
            if self.busy or not self.pending_station_trigger:
                return

            self.pending_station_trigger = False
            self.busy = True

        threading.Thread(target=self.station_sequence_worker, daemon=True).start()

    def startup_sequence(self):
        self.get_logger().info('Running mission startup sequence...')

        if not self.wait_for_arm_service():
            self.get_logger().error(
                'Arm service not available during startup. Mission will still run, '
                'but arm actions will fail until the service is available.'
            )

        if self.set_carry_pose_on_start:
            success = self.call_arm_motion('carry_line_follower')

            if not success:
                self.get_logger().warn(
                    'Could not set carry_line_follower pose during startup.'
                )

        if self.enable_line_follower_on_start:
            self.set_line_follower_enabled(True)

        self.stop_robot(duration_s=0.20)
        self.get_logger().info('Startup sequence finished. FOLLOW_LINE mode is active.')

    # --------------------------------------------------------------------------
    # Mission sequence
    # --------------------------------------------------------------------------
    def station_sequence_worker(self):
        try:
            self.get_logger().info('========================================')
            self.get_logger().info('Station sequence started.')

            # 1. Disable line follower so it stops publishing motion commands.
            self.set_line_follower_enabled(False)

            # 2. Stop the robot before moving the arm.
            self.stop_robot(self.stop_before_action_s)

            # 3. Decide arm action depending on cube state.
            if self.has_cube:
                arm_motion = 'drop'
                self.get_logger().info('Robot has cube -> running DROP.')
            else:
                arm_motion = 'pickup'
                self.get_logger().info('Robot has no cube -> running PICKUP.')

            success = self.call_arm_motion(arm_motion)

            if success:
                self.has_cube = not self.has_cube
                self.get_logger().info(f'Arm action finished. has_cube={self.has_cube}')
            else:
                self.get_logger().error(
                    'Arm action failed. Keeping previous has_cube state.'
                )

            # 4. Stop after arm action.
            self.stop_robot(self.stop_after_arm_action_s)

            # 5. Approximate 180-degree turn.
            self.turn_180()

            # 6. Stop after turn.
            self.stop_robot(self.stop_after_turn_s)

            # 7. Re-enable line follower.
            self.set_line_follower_enabled(True)

            self.get_logger().info('Station sequence finished. FOLLOW_LINE mode active.')
            self.get_logger().info('========================================')

        except Exception as exc:
            self.get_logger().error(f'Station sequence failed: {exc}')
            self.stop_robot(duration_s=0.50)
            self.set_line_follower_enabled(False)

        finally:
            with self.lock:
                self.busy = False

    # --------------------------------------------------------------------------
    # Robot movement helpers
    # --------------------------------------------------------------------------
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

        while rclpy.ok() and (time.time() - start_time) < self.turn_duration_s:
            self.publish_cmd_vel(0.0, angular_z)
            time.sleep(rate_period)

        self.publish_cmd_vel(0.0, 0.0)

    def set_line_follower_enabled(self, enabled):
        msg = Bool()
        msg.data = bool(enabled)

        self.line_follower_enabled_pub.publish(msg)

        state = 'enabled' if enabled else 'disabled'
        self.get_logger().info(f'Line follower {state}.')

    # --------------------------------------------------------------------------
    # Arm service helpers
    # --------------------------------------------------------------------------
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
