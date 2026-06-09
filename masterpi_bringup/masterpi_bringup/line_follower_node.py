#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray


class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower_node')

        # Topics
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('debug_image_topic', '/line_follower/debug_image')
        self.declare_parameter('mask_topic', '/line_follower/mask')
        self.declare_parameter('status_topic', '/line_follower/status')
        self.declare_parameter('enabled_topic', '/line_follower/enabled')
        self.declare_parameter('start_enabled', True)
        self.declare_parameter('publish_debug', False)
        self.declare_parameter('debug_publish_rate', 3.0)

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
        self.declare_parameter('roi_1', [105.0, 130.0, 80.0, 240.0, 0.15])
        self.declare_parameter('roi_2', [145.0, 175.0, 80.0, 240.0, 0.30])
        self.declare_parameter('roi_3', [190.0, 235.0, 80.0, 240.0, 0.55])

        # Control parameters
        self.declare_parameter('base_speed', 0.15)
        self.declare_parameter('kp_angular', 0.60)
        self.declare_parameter('ki_angular', 0.00)
        self.declare_parameter('kd_angular', 0.25)
        self.declare_parameter('min_linear_speed', 0.15)
        self.declare_parameter('max_linear_speed', 0.18)
        self.declare_parameter('min_angular_speed', 0.35)
        self.declare_parameter('max_angular_speed', 1.00)
        self.declare_parameter('angular_sign', -1.0)
        self.declare_parameter('slowdown_on_error', 0.25)
        self.declare_parameter('deadband_error', 0.03)
        self.declare_parameter('integral_limit', 0.40)
        self.declare_parameter('use_center_filter', True)
        self.declare_parameter('center_smoothing_alpha', 0.30)
        self.declare_parameter('max_center_jump_px', 50.0)
        self.declare_parameter('low_confidence_weight_threshold', 0.95)
        self.declare_parameter('low_confidence_angular_scale', 0.65)

        # Detection safety
        self.declare_parameter('min_contour_area', 40.0)
        self.declare_parameter('search_when_lost', False)
        self.declare_parameter('search_angular_speed', 0.40)

        self.image_topic = self.get_parameter('image_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.debug_image_topic = self.get_parameter('debug_image_topic').value
        self.mask_topic = self.get_parameter('mask_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.enabled_topic = self.get_parameter('enabled_topic').value
        self.enabled = bool(self.get_parameter('start_enabled').value)
        self.publish_debug = bool(self.get_parameter('publish_debug').value)
        self.debug_publish_rate = max(0.0, float(self.get_parameter('debug_publish_rate').value))

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
        self.ki_angular = float(self.get_parameter('ki_angular').value)
        self.kd_angular = float(self.get_parameter('kd_angular').value)
        self.min_linear_speed = float(self.get_parameter('min_linear_speed').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.min_angular_speed = float(self.get_parameter('min_angular_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.angular_sign = float(self.get_parameter('angular_sign').value)
        self.slowdown_on_error = float(self.get_parameter('slowdown_on_error').value)
        self.deadband_error = float(self.get_parameter('deadband_error').value)
        self.integral_limit = float(self.get_parameter('integral_limit').value)
        self.use_center_filter = bool(self.get_parameter('use_center_filter').value)
        self.center_smoothing_alpha = float(self.get_parameter('center_smoothing_alpha').value)
        self.max_center_jump_px = float(self.get_parameter('max_center_jump_px').value)
        self.low_confidence_weight_threshold = float(self.get_parameter('low_confidence_weight_threshold').value)
        self.low_confidence_angular_scale = float(self.get_parameter('low_confidence_angular_scale').value)

        self.min_contour_area = float(self.get_parameter('min_contour_area').value)
        self.search_when_lost = bool(self.get_parameter('search_when_lost').value)
        self.search_angular_speed = float(self.get_parameter('search_angular_speed').value)

        # PID state
        self.integral_error = 0.0
        self.last_error = None
        self.filtered_center_x = None
        self.last_time = None

        self.bridge = CvBridge()
        self.last_debug_pub_time = None

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(Float32MultiArray, self.status_topic, 10)

        self.debug_pub = None
        self.mask_pub = None

        if self.publish_debug:
            self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 10)
        #     self.mask_pub = self.create_publisher(Image, self.mask_topic, 10)

        self.enabled_sub = self.create_subscription(
            Bool,
            self.enabled_topic,
            self.enabled_callback,
            10
        )

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.get_logger().info('Line follower node started.')
        self.get_logger().info(f'Detection mode: {self.detection_mode}')
        self.get_logger().info(f'Subscribing image: {self.image_topic}')
        self.get_logger().info(f'Publishing cmd_vel: {self.cmd_vel_topic}')
        self.get_logger().info(f'Status topic: {self.status_topic}')
        self.get_logger().info(f'Enabled topic: {self.enabled_topic} | start_enabled={self.enabled}')
        self.get_logger().info(
            f'Debug images: {self.publish_debug} @ {self.debug_publish_rate:.1f} Hz'
        )
        self.get_logger().info(f'Brightness threshold: {self.brightness_threshold}')
        self.get_logger().info(f'ROIs: {self.rois}')
        self.get_logger().info(
            f'Control gains | kp={self.kp_angular:.3f}, '
            f'ki={self.ki_angular:.3f}, kd={self.kd_angular:.3f}'
        )
        self.get_logger().info(
            f'Limits | linear=[{self.min_linear_speed:.2f}, {self.max_linear_speed:.2f}], '
            f'angular=[{self.min_angular_speed:.2f}, {self.max_angular_speed:.2f}]'
        )

    def _parse_roi(self, values):
        y1, y2, x1, x2, weight = values
        return int(y1), int(y2), int(x1), int(x2), float(weight)

    def _filter_status_code(self, status):
        codes = {
            'LOST': -1.0,
            'RAW': 0.0,
            'INIT': 1.0,
            'FILT': 2.0,
            'LIMIT': 3.0,
            'LOW_WEIGHT': 4.0,
            'DISABLED': 5.0,
        }
        return codes.get(status, -2.0)

    def _publish_status(
        self,
        detected,
        raw_center_x,
        filtered_center_x,
        error_norm,
        linear_x,
        angular_z,
        total_area,
        roi_areas,
        filter_status,
        total_weight
    ):
        msg = Float32MultiArray()

        padded_roi_areas = list(roi_areas)[:3]

        while len(padded_roi_areas) < 3:
            padded_roi_areas.append(0.0)

        msg.data = [
            1.0 if detected else 0.0,
            float(raw_center_x),
            float(filtered_center_x),
            float(error_norm),
            float(linear_x),
            float(angular_z),
            float(total_area),
            float(padded_roi_areas[0]),
            float(padded_roi_areas[1]),
            float(padded_roi_areas[2]),
            self._filter_status_code(filter_status),
            float(total_weight),
            1.0 if self.enabled else 0.0,
        ]

        self.status_pub.publish(msg)

    def enabled_callback(self, msg):
        previous_enabled = self.enabled
        self.enabled = bool(msg.data)

        if self.enabled != previous_enabled:
            state = 'enabled' if self.enabled else 'disabled'
            self.get_logger().info(f'Line follower {state} from {self.enabled_topic}.')

        if not self.enabled:
            self._reset_controller_state()
            self.cmd_pub.publish(Twist())

    def _reset_controller_state(self):
        self.integral_error = 0.0
        self.last_error = None
        self.last_time = None

    def make_mask(self, frame):
        is_mono = len(frame.shape) == 2 or (
            len(frame.shape) == 3 and frame.shape[2] == 1
        )

        if is_mono:
            mono_frame = frame if len(frame.shape) == 2 else frame[:, :, 0]
        else:
            mono_frame = None

        if self.detection_mode == 'brightness':
            if is_mono:
                gray = mono_frame
            else:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

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
            if is_mono:
                frame_bgr = cv2.cvtColor(mono_frame, cv2.COLOR_GRAY2BGR)
            else:
                frame_bgr = frame

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
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().error(f'cv_bridge error: {exc}')
            return

        h, w = frame.shape[:2]

        raw_mask = self.make_mask(frame)

        # Only keep the binary mask inside the configured ROIs.
        # This prevents reflections outside the useful region from affecting detection.
        mask = np.zeros_like(raw_mask)

        for y1, y2, x1, x2, _weight in self.rois:
            y1 = max(0, min(h - 1, int(y1)))
            y2 = max(0, min(h, int(y2)))
            x1 = max(0, min(w - 1, int(x1)))
            x2 = max(0, min(w, int(x2)))

            mask[y1:y2, x1:x2] = raw_mask[y1:y2, x1:x2]

        if len(frame.shape) == 2:
            debug = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        elif len(frame.shape) == 3 and frame.shape[2] == 1:
            debug = cv2.cvtColor(frame[:, :, 0], cv2.COLOR_GRAY2BGR)
        else:
            debug = frame.copy()

        weighted_sum_x = 0.0
        total_weight = 0.0
        total_area = 0.0
        roi_areas = [0.0 for _ in self.rois]

        image_center_x = w / 2.0
        expected_center_x = (
            self.filtered_center_x
            if self.use_center_filter and self.filtered_center_x is not None
            else image_center_x
        )

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

            contour_candidates = []

            for contour in contours:
                area = cv2.contourArea(contour)

                if area < self.min_contour_area:
                    continue

                moments = cv2.moments(contour)

                if moments['m00'] == 0:
                    continue

                cx_local = int(moments['m10'] / moments['m00'])
                cy_local = int(moments['m01'] / moments['m00'])

                cx = x1 + cx_local
                cy = y1 + cy_local

                distance_to_expected = abs(cx - expected_center_x)

                contour_candidates.append(
                    (distance_to_expected, -area, area, cx, cy)
                )

            if not contour_candidates:
                continue

            contour_candidates.sort(key=lambda item: (item[0], item[1]))
            _distance_to_expected, _negative_area, area, cx, cy = contour_candidates[0]

            weighted_sum_x += cx * weight
            total_weight += weight
            total_area += area
            roi_areas[idx] = area

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
            raw_line_center_x = weighted_sum_x / total_weight
            line_center_x = raw_line_center_x
            center_filter_status = 'RAW'

            if self.use_center_filter:
                if self.filtered_center_x is None:
                    self.filtered_center_x = raw_line_center_x
                    center_filter_status = 'INIT'
                else:
                    delta = raw_line_center_x - self.filtered_center_x
                    limited = False
                    candidate_center_x = raw_line_center_x

                    if (
                        self.max_center_jump_px > 0.0
                        and abs(delta) > self.max_center_jump_px
                    ):
                        candidate_center_x = (
                            self.filtered_center_x + self.max_center_jump_px
                            if delta > 0.0
                            else self.filtered_center_x - self.max_center_jump_px
                        )
                        limited = True

                    alpha = max(0.0, min(1.0, self.center_smoothing_alpha))
                    self.filtered_center_x = (
                        (1.0 - alpha) * self.filtered_center_x
                        + alpha * candidate_center_x
                    )

                    center_filter_status = 'LIMIT' if limited else 'FILT'

                line_center_x = self.filtered_center_x

            error_px = line_center_x - image_center_x
            error_norm = error_px / image_center_x

            confidence_status = center_filter_status

            if abs(error_norm) < self.deadband_error:
                control_error = 0.0
            else:
                control_error = error_norm

            now = self.get_clock().now().nanoseconds / 1e9

            if self.last_time is None:
                dt = 0.0
            else:
                dt = max(1e-3, now - self.last_time)

            if control_error == 0.0:
                self.integral_error = 0.0
                derivative_error = 0.0
            else:
                self.integral_error += control_error * dt
                self.integral_error = max(
                    -self.integral_limit,
                    min(self.integral_limit, self.integral_error)
                )

                if self.last_error is None or dt == 0.0:
                    derivative_error = 0.0
                else:
                    derivative_error = (control_error - self.last_error) / dt

            pid_output = (
                self.kp_angular * control_error
                + self.ki_angular * self.integral_error
                + self.kd_angular * derivative_error
            )

            angular_z = self.angular_sign * pid_output
            angular_z = max(
                -self.max_angular_speed,
                min(self.max_angular_speed, angular_z)
            )

            if (
                control_error != 0.0
                and 0.0 < abs(angular_z) < self.min_angular_speed
            ):
                angular_z = (
                    self.min_angular_speed
                    if angular_z > 0.0
                    else -self.min_angular_speed
                )

            if (
                self.low_confidence_weight_threshold > 0.0
                and 0.0 < total_weight < self.low_confidence_weight_threshold
            ):
                angular_z *= self.low_confidence_angular_scale
                confidence_status = 'LOW_WEIGHT'

            speed_factor = (
                1.0 - min(abs(control_error), 1.0) * self.slowdown_on_error
            )

            linear_x = self.base_speed * speed_factor
            linear_x = max(
                self.min_linear_speed,
                min(self.max_linear_speed, linear_x)
            )

            twist.linear.x = linear_x
            twist.angular.z = angular_z

            self.last_error = control_error
            self.last_time = now

            cv2.line(
                debug,
                (int(image_center_x), 0),
                (int(image_center_x), h),
                (255, 0, 0),
                1
            )
            cv2.circle(
                debug,
                (int(line_center_x), int(h * 0.85)),
                7,
                (0, 255, 0),
                -1
            )

            cv2.putText(
                debug,
                f'RAW={raw_line_center_x:.1f} CENTER={line_center_x:.1f} '
                f'ERR={error_norm:.2f} {confidence_status}',
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

            detected = True
            status_raw_center_x = raw_line_center_x
            status_filtered_center_x = line_center_x
            status_error_norm = error_norm
            status_linear_x = linear_x
            status_angular_z = angular_z
            status_filter = confidence_status
            status_total_weight = total_weight

        else:
            self._reset_controller_state()

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

            detected = False
            status_raw_center_x = -1.0
            status_filtered_center_x = (
                self.filtered_center_x
                if self.filtered_center_x is not None
                else -1.0
            )
            status_error_norm = 0.0
            status_linear_x = twist.linear.x
            status_angular_z = twist.angular.z
            status_filter = 'LOST'
            status_total_weight = 0.0

        if not self.enabled:
            self._reset_controller_state()
            twist = Twist()

            status_linear_x = 0.0
            status_angular_z = 0.0
            status_filter = 'DISABLED'

            cv2.putText(
                debug,
                'LINE FOLLOWER DISABLED',
                (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 165, 255),
                2
            )

        self._publish_status(
            detected,
            status_raw_center_x,
            status_filtered_center_x,
            status_error_norm,
            status_linear_x,
            status_angular_z,
            total_area,
            roi_areas,
            status_filter,
            status_total_weight
        )

        self.cmd_pub.publish(twist)

        if not self.publish_debug:
            return

        now_debug = self.get_clock().now().nanoseconds / 1e9

        if self.debug_publish_rate > 0.0 and self.last_debug_pub_time is not None:
            min_period = 1.0 / self.debug_publish_rate

            if now_debug - self.last_debug_pub_time < min_period:
                return

        self.last_debug_pub_time = now_debug

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
