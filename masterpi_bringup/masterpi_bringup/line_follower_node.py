#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower_node')

        # Topics
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('debug_image_topic', '/line_follower/debug_image')
        self.declare_parameter('mask_topic', '/line_follower/mask')

        # Detection mode: "brightness" or "hsv"
        self.declare_parameter('detection_mode', 'brightness')

        # Brightness binary filter
        self.declare_parameter('brightness_threshold', 180)
        self.declare_parameter('blur_kernel', 5)
        self.declare_parameter('morph_kernel', 5)

        # HSV fallback
        self.declare_parameter('h_min', 18)
        self.declare_parameter('s_min', 70)
        self.declare_parameter('v_min', 70)
        self.declare_parameter('h_max', 45)
        self.declare_parameter('s_max', 255)
        self.declare_parameter('v_max', 255)

        # ROI format: [y1, y2, x1, x2, weight]
        # For 320x240 image, centered region
        self.declare_parameter('roi_1', [105.0, 130.0, 80.0, 240.0, 0.15])
        self.declare_parameter('roi_2', [145.0, 175.0, 80.0, 240.0, 0.30])
        self.declare_parameter('roi_3', [190.0, 235.0, 80.0, 240.0, 0.55])

        # Control parameters
        self.declare_parameter('base_speed', 0.15)
        self.declare_parameter('kp_angular', 0.80)
        self.declare_parameter('max_angular_speed', 1.00)
        self.declare_parameter('angular_sign', -1.0)
        self.declare_parameter('slowdown_on_error', 0.20)

        # Detection safety
        self.declare_parameter('min_contour_area', 40.0)
        self.declare_parameter('search_when_lost', False)
        self.declare_parameter('search_angular_speed', 0.40)

        self.image_topic = self.get_parameter('image_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.debug_image_topic = self.get_parameter('debug_image_topic').value
        self.mask_topic = self.get_parameter('mask_topic').value

        self.detection_mode = str(self.get_parameter('detection_mode').value)

        self.brightness_threshold = int(self.get_parameter('brightness_threshold').value)
        self.blur_kernel = int(self.get_parameter('blur_kernel').value)
        self.morph_kernel = int(self.get_parameter('morph_kernel').value)

        self.h_min = int(self.get_parameter('h_min').value)
        self.s_min = int(self.get_parameter('s_min').value)
        self.v_min = int(self.get_parameter('v_min').value)
        self.h_max = int(self.get_parameter('h_max').value)
        self.s_max = int(self.get_parameter('s_max').value)
        self.v_max = int(self.get_parameter('v_max').value)

        self.rois = [
            self._parse_roi(self.get_parameter('roi_1').value),
            self._parse_roi(self.get_parameter('roi_2').value),
            self._parse_roi(self.get_parameter('roi_3').value),
        ]

        self.base_speed = float(self.get_parameter('base_speed').value)
        self.kp_angular = float(self.get_parameter('kp_angular').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.angular_sign = float(self.get_parameter('angular_sign').value)
        self.slowdown_on_error = float(self.get_parameter('slowdown_on_error').value)

        self.min_contour_area = float(self.get_parameter('min_contour_area').value)
        self.search_when_lost = bool(self.get_parameter('search_when_lost').value)
        self.search_angular_speed = float(self.get_parameter('search_angular_speed').value)

        self.bridge = CvBridge()

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 10)
        self.mask_pub = self.create_publisher(Image, self.mask_topic, 10)

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.get_logger().info('Line follower node started.')
        self.get_logger().info(f'Detection mode: {self.detection_mode}')
        self.get_logger().info(f'Subscribing: {self.image_topic}')
        self.get_logger().info(f'Brightness threshold: {self.brightness_threshold}')
        self.get_logger().info(f'ROIs: {self.rois}')

    def _parse_roi(self, values):
        y1, y2, x1, x2, weight = values
        return int(y1), int(y2), int(x1), int(x2), float(weight)

    def make_mask(self, frame_bgr):
        if self.detection_mode == 'brightness':
            gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

            if self.blur_kernel > 1:
                k = self.blur_kernel
                if k % 2 == 0:
                    k += 1
                gray = cv2.GaussianBlur(gray, (k, k), 0)

            _, mask = cv2.threshold(
                gray,
                self.brightness_threshold,
                255,
                cv2.THRESH_BINARY
            )

        else:
            hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
            lower = np.array([self.h_min, self.s_min, self.v_min], dtype=np.uint8)
            upper = np.array([self.h_max, self.s_max, self.v_max], dtype=np.uint8)
            mask = cv2.inRange(hsv, lower, upper)

        if self.morph_kernel > 1:
            k = self.morph_kernel
            kernel = np.ones((k, k), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        return mask

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'cv_bridge error: {exc}')
            return

        h, w = frame.shape[:2]

        raw_mask = self.make_mask(frame)

        # Only keep the binary mask inside the configured ROIs.
        # This makes /line_follower/mask easier to debug and prevents
        # bright reflections outside the useful region from being considered.
        mask = np.zeros_like(raw_mask)

        for y1, y2, x1, x2, _weight in self.rois:
            y1 = max(0, min(h - 1, int(y1)))
            y2 = max(0, min(h, int(y2)))
            x1 = max(0, min(w - 1, int(x1)))
            x2 = max(0, min(w, int(x2)))

            mask[y1:y2, x1:x2] = raw_mask[y1:y2, x1:x2]

        debug = frame.copy()

        weighted_sum_x = 0.0
        total_weight = 0.0
        total_area = 0.0

        for idx, (y1, y2, x1, x2, weight) in enumerate(self.rois):
            y1 = max(0, min(h - 1, y1))
            y2 = max(0, min(h, y2))
            x1 = max(0, min(w - 1, x1))
            x2 = max(0, min(w, x2))

            roi_mask = mask[y1:y2, x1:x2]

            contours, _ = cv2.findContours(
                roi_mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )

            cv2.rectangle(debug, (x1, y1), (x2, y2), (120, 120, 120), 1)

            if not contours:
                continue

            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area < self.min_contour_area:
                continue

            moments = cv2.moments(largest)
            if moments['m00'] == 0:
                continue

            cx_local = int(moments['m10'] / moments['m00'])
            cy_local = int(moments['m01'] / moments['m00'])

            cx = x1 + cx_local
            cy = y1 + cy_local

            weighted_sum_x += cx * weight
            total_weight += weight
            total_area += area

            cv2.circle(debug, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(
                debug,
                f'ROI{idx + 1} area={int(area)}',
                (x1 + 5, max(15, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.35,
                (0, 0, 255),
                1
            )

        twist = Twist()

        if total_weight > 0.0:
            line_center_x = weighted_sum_x / total_weight
            image_center_x = w / 2.0
            error_px = line_center_x - image_center_x
            error_norm = error_px / image_center_x

            angular_z = self.angular_sign * self.kp_angular * error_norm
            angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))

            speed_factor = 1.0 - min(abs(error_norm), 1.0) * self.slowdown_on_error
            linear_x = self.base_speed * speed_factor

            twist.linear.x = linear_x
            twist.angular.z = angular_z

            cv2.line(debug, (int(image_center_x), 0), (int(image_center_x), h), (255, 0, 0), 1)
            cv2.circle(debug, (int(line_center_x), int(h * 0.85)), 7, (0, 255, 0), -1)

            cv2.putText(
                debug,
                f'CENTER={line_center_x:.1f} ERR={error_norm:.2f}',
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2
            )

            cv2.putText(
                debug,
                f'CMD x={linear_x:.2f} z={angular_z:.2f}',
                (10, 45),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2
            )

        else:
            if self.search_when_lost:
                twist.angular.z = self.search_angular_speed
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            cv2.putText(
                debug,
                'LINE LOST',
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

        self.cmd_pub.publish(twist)

        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)

        debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
