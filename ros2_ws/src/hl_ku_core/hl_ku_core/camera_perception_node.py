"""Conservative classical baseline for lane, stop-line and traffic-light cues."""

from __future__ import annotations

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, UInt8


class CameraPerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_perception")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("lane_roi_top_ratio", 0.55)
        self.declare_parameter("lane_meters_per_pixel", 0.005)
        self.declare_parameter("minimum_lane_pixels", 500)
        self.declare_parameter("stop_line_projection_ratio", 0.40)
        self.declare_parameter("traffic_roi", [0.35, 0.02, 0.65, 0.45])
        self.declare_parameter("traffic_min_ratio", 0.015)
        self.declare_parameter("publish_traffic_light", True)
        self._bridge = CvBridge()
        self._lane_offset_pub = self.create_publisher(Float32, "/perception/lane_offset_m", 10)
        self._lane_heading_pub = self.create_publisher(Float32, "/perception/lane_heading_rad", 10)
        self._lane_conf_pub = self.create_publisher(Float32, "/perception/lane_confidence", 10)
        self._stop_pub = self.create_publisher(Bool, "/perception/stop_line_detected", 10)
        self._stop_distance_pub = self.create_publisher(Float32, "/perception/stop_line_distance_m", 10)
        self._light_pub = self.create_publisher(UInt8, "/perception/traffic_light", 10)
        self._light_conf_pub = self.create_publisher(Float32, "/perception/traffic_confidence", 10)
        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self._on_image,
            qos_profile_sensor_data,
        )

    @staticmethod
    def _publish(publisher, message_type, value) -> None:
        message = message_type()
        message.data = value
        publisher.publish(message)

    def _on_image(self, message: Image) -> None:
        try:
            image = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as error:  # CvBridge exception type differs by distro
            self.get_logger().warning(f"camera conversion failed: {error}")
            return
        height, width = image.shape[:2]
        roi_top = int(height * float(self.get_parameter("lane_roi_top_ratio").value))
        lane_image = image[roi_top:, :]
        hsv = cv2.cvtColor(lane_image, cv2.COLOR_BGR2HSV)
        white = cv2.inRange(hsv, np.array([0, 0, 165]), np.array([180, 80, 255]))
        yellow = cv2.inRange(hsv, np.array([12, 70, 80]), np.array([42, 255, 255]))
        mask = cv2.morphologyEx(white | yellow, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        ys, xs = np.nonzero(mask)
        minimum_pixels = int(self.get_parameter("minimum_lane_pixels").value)
        confidence = min(1.0, len(xs) / max(minimum_pixels * 4.0, 1.0))
        offset_m, heading_rad = 0.0, 0.0
        if len(xs) >= minimum_pixels:
            lower = ys >= int(mask.shape[0] * 0.55)
            selected_x = xs[lower] if np.any(lower) else xs
            center_x = float(np.median(selected_x))
            offset_m = (center_x - width * 0.5) * float(
                self.get_parameter("lane_meters_per_pixel").value
            )
            points = np.column_stack((xs.astype(np.float32), ys.astype(np.float32)))
            vx, vy, _, _ = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01).flatten()
            heading_rad = math.atan2(float(vx), max(abs(float(vy)), 1.0e-6))
        self._publish(self._lane_offset_pub, Float32, float(offset_m))
        self._publish(self._lane_heading_pub, Float32, float(heading_rad))
        self._publish(self._lane_conf_pub, Float32, float(confidence))

        row_counts = np.count_nonzero(white, axis=1)
        stop_threshold = width * float(self.get_parameter("stop_line_projection_ratio").value)
        stop_rows = np.flatnonzero(row_counts >= stop_threshold)
        stop_detected = len(stop_rows) > 0
        stop_distance = math.inf
        if stop_detected:
            row_from_bottom = mask.shape[0] - int(stop_rows[-1])
            stop_distance = max(0.0, row_from_bottom / max(mask.shape[0], 1) * 6.0)
        self._publish(self._stop_pub, Bool, bool(stop_detected))
        self._publish(self._stop_distance_pub, Float32, float(stop_distance))

        x0r, y0r, x1r, y1r = [float(v) for v in self.get_parameter("traffic_roi").value]
        x0, y0 = int(width * x0r), int(height * y0r)
        x1, y1 = int(width * x1r), int(height * y1r)
        traffic = image[max(0, y0):min(height, y1), max(0, x0):min(width, x1)]
        light, light_confidence = 0, 0.0
        if traffic.size:
            traffic_hsv = cv2.cvtColor(traffic, cv2.COLOR_BGR2HSV)
            red = cv2.inRange(traffic_hsv, np.array([0, 100, 120]), np.array([10, 255, 255]))
            red |= cv2.inRange(traffic_hsv, np.array([170, 100, 120]), np.array([180, 255, 255]))
            green = cv2.inRange(traffic_hsv, np.array([40, 70, 80]), np.array([95, 255, 255]))
            total = float(traffic.shape[0] * traffic.shape[1])
            red_ratio = np.count_nonzero(red) / total
            green_ratio = np.count_nonzero(green) / total
            minimum = float(self.get_parameter("traffic_min_ratio").value)
            if red_ratio >= minimum and red_ratio > green_ratio * 1.2:
                light, light_confidence = 1, min(1.0, red_ratio / minimum)
            elif green_ratio >= minimum and green_ratio > red_ratio * 1.2:
                light, light_confidence = 3, min(1.0, green_ratio / minimum)
        if bool(self.get_parameter("publish_traffic_light").value):
            self._publish(self._light_pub, UInt8, int(light))
            self._publish(self._light_conf_pub, Float32, float(light_confidence))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraPerceptionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
