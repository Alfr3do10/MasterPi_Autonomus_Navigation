
#!/usr/bin/env python3
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

sys.path.append('/home/pi/MasterPi')

try:
    import HiwonderSDK.Board as Board
except Exception:
    Board = None


class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')

        self.declare_parameter('use_mock_hardware', False)
        self.declare_parameter('pan_channel', 1)
        self.declare_parameter('tilt_channel', 3)
        self.declare_parameter('default_pan', 1500)
        self.declare_parameter('default_tilt', 1500)
        self.declare_parameter('move_time_ms', 500)
        self.declare_parameter('min_pulse', 500)
        self.declare_parameter('max_pulse', 2500)

        self.use_mock_hardware = self.get_parameter('use_mock_hardware').value
        self.pan_channel = int(self.get_parameter('pan_channel').value)
        self.tilt_channel = int(self.get_parameter('tilt_channel').value)
        self.default_pan = int(self.get_parameter('default_pan').value)
        self.default_tilt = int(self.get_parameter('default_tilt').value)
        self.move_time_ms = int(self.get_parameter('move_time_ms').value)
        self.min_pulse = int(self.get_parameter('min_pulse').value)
        self.max_pulse = int(self.get_parameter('max_pulse').value)

        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/servo_cmd',
            self.servo_callback,
            10
        )

        self.get_logger().info('Servo node started.')
        self.get_logger().info(f'use_mock_hardware = {self.use_mock_hardware}')
        self.get_logger().info(f'Channels | pan={self.pan_channel}, tilt={self.tilt_channel}')
        self.get_logger().info('Expected /servo_cmd format: [pan_pulse, tilt_pulse]')

        self.move_servos(self.default_pan, self.default_tilt)

    def clamp(self, value):
        value = int(value)
        if value < self.min_pulse:
            return self.min_pulse
        if value > self.max_pulse:
            return self.max_pulse
        return value

    def servo_callback(self, msg):
        self.get_logger().info(f'Received /servo_cmd: {list(msg.data)}')

        if len(msg.data) < 2:
            self.get_logger().warn('Invalid /servo_cmd. Expected: [pan_pulse, tilt_pulse]')
            return

        pan = self.clamp(msg.data[0])
        tilt = self.clamp(msg.data[1])

        self.move_servos(pan, tilt)

    def move_servos(self, pan, tilt):
        self.get_logger().info(f'Moving real servos | pan={pan}, tilt={tilt}')

        if self.use_mock_hardware:
            self.get_logger().info('Mock mode: not moving real hardware.')
            return

        if Board is None:
            self.get_logger().error('Board import failed. Cannot move servos.')
            return

        try:
            Board.setPWMServoPulse(self.pan_channel, pan, self.move_time_ms)
            Board.setPWMServoPulse(self.tilt_channel, tilt, self.move_time_ms)
        except Exception as e:
            self.get_logger().error(f'Could not move servos: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()

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
