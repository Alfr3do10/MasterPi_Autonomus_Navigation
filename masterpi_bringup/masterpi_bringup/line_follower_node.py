#!/usr/bin/env python3

import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray


class LineFollowerNode(Node):
    """Camera-based yellow line follower for the Hiwonder MasterPi.

    Input:
      /camera/image_raw  sensor_msgs/Image, ideally bgr8 color

    Output:
      /cmd_vel           geometry_msgs/Twist
      /servo_cmd         std_msgs/Int32MultiArray, optional camera pose init
      /line_follower/debug_image sensor_msgs/Image
      /line_follower/mask        sensor_msgs/Image
    """

    def __init__(self):
        super().__init__('line_follower_node')

        # Topics
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('servo_topic', '/servo_cmd')

        # Camera / image processing
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('roi_1', [240.0, 280.0, 0.0, 640.0, 0.10])
        self.declare_parameter('roi_2', [340.0, 380.0, 0.0, 640.0, 0.30])
        self.declare_parameter('roi_3', [420.0, 470.0, 0.0, 640.0, 0.60])
        self.declare_parameter('min_contour_area', 150.0)
        self.declare_parameter('morph_kernel_size', 5)

        # HSV limits for yellow on black floor.
        # Tune these if lighting changes: hue around 20-40 is common for yellow in OpenCV HSV.
        self.declare_parameter('h_min', 18)
        self.declare_parameter('s_min', 70)
        self.declare_parameter('v_min', 70)
        self.declare_parameter('h_max', 45)
        self.declare_parameter('s_max', 255)
        self.declare_parameter('v_max', 255)

        # Control
        self.declare_parameter('base_speed', 0.08)          # m/s, start slow
        self.declare_parameter('kp_angular', 0.90)          # rad/s per normalized error
        self.declare_parameter('max_angular_speed', 0.75)   # rad/s
        self.declare_parameter('angular_sign', -1.0)        # -1: line right -> turn right for ROS convention
        self.declare_parameter('slowdown_on_error', 0.45)   # reduce forward speed on curves
        self.declare_parameter('lost_line_timeout', 0.25)   # seconds
        self.declare_parameter('search_when_lost', False)
        self.declare_parameter('search_angular_speed', 0.25)

        # Servo initial pose to point camera toward the floor.
        self.declare_parameter('set_camera_servo', True)
        self.declare_parameter('servo_pan', 1500)
        self.declare_parameter('servo_tilt', 500)
        self.declare_parameter('servo_init_duration', 3.0)
        self.declare_parameter('servo_publish_period', 0.5)

        # Debug
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('show_debug_window', False)  # keep False on Ubuntu Server/headless
        self.declare_parameter('log_every_n_frames', 10)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.servo_topic = str(self.get_parameter('servo_topic').value)

        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.rois = [
            self._parse_roi(self.get_parameter('roi_1').value),
            self._parse_roi(self.get_parameter('roi_2').value),
            self._parse_roi(self.get_parameter('roi_3').value),
        ]
        self.min_contour_area = float(self.get_parameter('min_contour_area').value)
        self.morph_kernel_size = int(self.get_parameter('morph_kernel_size').value)

        self.lower_yellow = np.array([
            int(self.get_parameter('h_min').value),
            int(self.get_parameter('s_min').value),
            int(self.get_parameter('v_min').value),
        ], dtype=np.uint8)
        self.upper_yellow = np.array([
            int(self.get_parameter('h_max').value),
            int(self.get_parameter('s_max').value),
            int(self.get_parameter('v_max').value),
        ], dtype=np.uint8)

        self.base_speed = float(self.get_parameter('base_speed').value)
        self.kp_angular = float(self.get_parameter('kp_angular').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.angular_sign = float(self.get_parameter('angular_sign').value)
        self.slowdown_on_error = float(self.get_parameter('slowdown_on_error').value)
        self.lost_line_timeout = float(self.get_parameter('lost_line_timeout').value)
        self.search_when_lost = bool(self.get_parameter('search_when_lost').value)
        self.search_angular_speed = float(self.get_parameter('search_angular_speed').value)

        self.set_camera_servo = bool(self.get_parameter('set_camera_servo').value)
        self.servo_pan = int(self.get_parameter('servo_pan').value)
        self.servo_tilt = int(self.get_parameter('servo_tilt').value)
        self.servo_init_duration = float(self.get_parameter('servo_init_duration').value)
        self.servo_publish_period = float(self.get_parameter('servo_publish_period').value)

        self.publish_debug = bool(self.get_parameter('publish_debug').value)
        self.show_debug_window = bool(self.get_parameter('show_debug_window').value)
        self.log_every_n_frames = int(self.get_parameter('log_every_n_frames').value)

        self.bridge = CvBridge()
        self.frame_count = 0
        self.last_seen_time = time.monotonic()
        self.last_error = 0.0
        self.warned_mono = False
        self.start_time = time.monotonic()

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.servo_pub = self.create_publisher(Int32MultiArray, self.servo_topic, 10)
        self.debug_pub = self.create_publisher(Image, '/line_follower/debug_image', 10)
        self.mask_pub = self.create_publisher(Image, '/line_follower/mask', 10)

        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.servo_timer = self.create_timer(self.servo_publish_period, self.publish_initial_servo_pose)

        self.get_logger().info('Yellow line follower started.')
        self.get_logger().info(f'Subscribing: {self.image_topic} | Publishing: {self.cmd_vel_topic}')
        self.get_logger().info(
            f'HSV yellow lower={self.lower_yellow.tolist()} upper={self.upper_yellow.tolist()} | '
            f'base_speed={self.base_speed:.2f} kp={self.kp_angular:.2f}'
        )

    def _parse_roi(self, value):
        data = list(value)
        if len(data) != 5:
            raise ValueError('ROI must be [y1, y2, x1, x2, weight]')
        y1, y2, x1, x2 = [int(v) for v in data[:4]]
        weight = float(data[4])
        return y1, y2, x1, x2, weight

    def publish_initial_servo_pose(self):
        if not self.set_camera_servo:
            self.servo_timer.cancel()
            return

        elapsed = time.monotonic() - self.start_time
        if elapsed > self.servo_init_duration:
            self.servo_timer.cancel()
            return

        msg = Int32MultiArray()
        msg.data = [self.servo_pan, self.servo_tilt]
        self.servo_pub.publish(msg)

    def image_callback(self, msg):
        self.frame_count += 1

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Could not convert camera image to bgr8: {exc}')
            self.publish_stop()
            return

        if msg.encoding.lower() in ('mono8', '8uc1') and not self.warned_mono:
            self.warned_mono = True
            self.get_logger().warn(
                'Camera image is grayscale/mono. Yellow detection needs color. '
                'Set camera_node publish_grayscale=false.'
            )

        frame = cv2.resize(frame, (self.image_width, self.image_height), interpolation=cv2.INTER_NEAREST)
        debug = frame.copy()
        full_mask = np.zeros((self.image_height, self.image_width), dtype=np.uint8)

        center_x, area_total = self.detect_line_center(frame, debug, full_mask)

        if center_x is None:
            if time.monotonic() - self.last_seen_time > self.lost_line_timeout:
                if self.search_when_lost:
                    self.publish_velocity(0.0, self.search_angular_speed * math.copysign(1.0, self.last_error or 1.0))
                else:
                    self.publish_stop()
            self.publish_debug_images(debug, full_mask)
            return

        self.last_seen_time = time.monotonic()
        error_px = center_x - (self.image_width / 2.0)
        error_norm = error_px / (self.image_width / 2.0)
        self.last_error = error_norm

        angular_z = self.angular_sign * self.kp_angular * error_norm
        angular_z = self.clamp(angular_z, self.max_angular_speed)

        speed_scale = 1.0 - self.slowdown_on_error * min(abs(error_norm), 1.0)
        linear_x = self.base_speed * max(0.35, speed_scale)

        self.publish_velocity(linear_x, angular_z)

        cv2.line(debug, (self.image_width // 2, 0), (self.image_width // 2, self.image_height), (255, 0, 0), 2)
        cv2.circle(debug, (int(center_x), self.image_height - 35), 10, (0, 255, 255), -1)
        cv2.putText(
            debug,
            f'cx={center_x:.0f} err={error_norm:+.2f} vx={linear_x:.2f} wz={angular_z:+.2f}',
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
        )

        if self.frame_count % max(1, self.log_every_n_frames) == 0:
            self.get_logger().info(
                f'line_center={center_x:.1f} error={error_norm:+.2f} area={area_total:.0f} '
                f'cmd linear.x={linear_x:.2f} angular.z={angular_z:+.2f}'
            )

        self.publish_debug_images(debug, full_mask)

    def detect_line_center(self, frame, debug, full_mask):
        weighted_sum = 0.0
        weight_sum = 0.0
        area_total = 0.0

        kernel_size = max(1, self.morph_kernel_size)
        kernel = np.ones((kernel_size, kernel_size), np.uint8)

        for y1, y2, x1, x2, weight in self.rois:
            y1 = self.clamp_int(y1, 0, self.image_height - 1)
            y2 = self.clamp_int(y2, y1 + 1, self.image_height)
            x1 = self.clamp_int(x1, 0, self.image_width - 1)
            x2 = self.clamp_int(x2, x1 + 1, self.image_width)

            roi = frame[y1:y2, x1:x2]
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            full_mask[y1:y2, x1:x2] = mask

            contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            if not contours:
                cv2.rectangle(debug, (x1, y1), (x2, y2), (80, 80, 80), 1)
                continue

            contour = max(contours, key=cv2.contourArea)
            area = float(cv2.contourArea(contour))
            if area < self.min_contour_area:
                cv2.rectangle(debug, (x1, y1), (x2, y2), (80, 80, 80), 1)
                continue

            moments = cv2.moments(contour)
            if abs(moments['m00']) < 1e-6:
                continue

            cx_local = moments['m10'] / moments['m00']
            cy_local = moments['m01'] / moments['m00']
            cx = x1 + cx_local
            cy = y1 + cy_local

            weighted_sum += cx * weight
            weight_sum += weight
            area_total += area

            contour_shifted = contour.copy()
            contour_shifted[:, :, 0] += x1
            contour_shifted[:, :, 1] += y1
            cv2.drawContours(debug, [contour_shifted], -1, (0, 0, 255), 2)
            cv2.circle(debug, (int(cx), int(cy)), 5, (0, 255, 255), -1)
            cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 255, 0), 1)

        if weight_sum <= 0.0:
            cv2.putText(debug, 'LINE LOST', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            return None, area_total

        return weighted_sum / weight_sum, area_total

    def publish_velocity(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def publish_stop(self):
        self.publish_velocity(0.0, 0.0)

    def publish_debug_images(self, debug, mask):
        if self.publish_debug:
            stamp = self.get_clock().now().to_msg()

            debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            debug_msg.header.stamp = stamp
            debug_msg.header.frame_id = 'camera_link'
            self.debug_pub.publish(debug_msg)

            mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
            mask_msg.header.stamp = stamp
            mask_msg.header.frame_id = 'camera_link'
            self.mask_pub.publish(mask_msg)

        if self.show_debug_window:
            cv2.imshow('line_follower_debug', debug)
            cv2.imshow('line_follower_mask', mask)
            cv2.waitKey(1)

    @staticmethod
    def clamp(value, limit):
        return max(min(value, limit), -limit)

    @staticmethod
    def clamp_int(value, low, high):
        return max(min(int(value), int(high)), int(low))

    def destroy_node(self):
        self.publish_stop()
        if self.show_debug_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
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
