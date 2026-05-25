#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState


class BatteryNode(Node):
    def __init__(self):
        super().__init__('battery_node')

        self.declare_parameter('publish_rate', 0.2)
        self.declare_parameter('use_mock_hardware', False)
        self.declare_parameter('min_voltage', 6.4)
        self.declare_parameter('max_voltage', 8.4)

        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.use_mock_hardware = bool(self.get_parameter('use_mock_hardware').value)
        self.min_voltage = float(self.get_parameter('min_voltage').value)
        self.max_voltage = float(self.get_parameter('max_voltage').value)

        self.board = None
        if not self.use_mock_hardware:
            try:
                from masterpi_bringup.hiwonder_sdk import Board
                self.board = Board
            except Exception as exc:
                self.get_logger().error(f'Could not initialize real battery hardware: {exc}')
                raise

        self.publisher = self.create_publisher(BatteryState, '/battery_state', 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_battery_state)

        self.get_logger().info('Battery node started.')
        self.get_logger().info(f'use_mock_hardware = {self.use_mock_hardware}')

    def publish_battery_state(self):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION

        if self.use_mock_hardware:
            raw = 7400
            voltage = 7.4
        else:
            raw = int(self.board.getBattery())
            voltage = self.raw_to_voltage(raw)

        msg.voltage = float(voltage)
        msg.percentage = self.estimate_percentage(voltage)
        self.publisher.publish(msg)

        self.get_logger().info(
            f'Battery | raw={raw} | voltage={msg.voltage:.2f} V | percentage={msg.percentage * 100:.1f}%'
        )

    def raw_to_voltage(self, raw):
        # Hiwonder images may report mV-like values. On this migration we observed raw≈65419,
        # which is best interpreted as 6.5419 V. Keep a defensive conversion.
        if raw > 20000:
            return raw / 10000.0
        if raw > 1000:
            return raw / 1000.0
        return float(raw)

    def estimate_percentage(self, voltage):
        percentage = (voltage - self.min_voltage) / (self.max_voltage - self.min_voltage)
        return max(0.0, min(1.0, percentage))


def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
