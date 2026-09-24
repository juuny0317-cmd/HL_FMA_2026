"""Aggregate replaceable perception components into one timestamped contract."""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int8, UInt8

from hl_ku_interfaces.msg import PerceptionState


class PerceptionAggregatorNode(Node):
    _SOURCE_FIELDS = {
        "camera": (
            "lane_offset_m",
            "lane_heading_error_rad",
            "lane_confidence",
            "stop_line_detected",
            "stop_line_distance_m",
            "traffic_light",
            "traffic_confidence",
        ),
        "lidar": (
            "obstacle_in_path",
            "obstacle_distance_m",
            "avoidance_steering_offset_rad",
        ),
        "end": ("end_lane_signal", "allowed_lane", "end_lane_confidence"),
    }

    def __init__(self) -> None:
        super().__init__("perception_aggregator")
        self.declare_parameter("camera_timeout_sec", 0.30)
        self.declare_parameter("lidar_timeout_sec", 0.30)
        self.declare_parameter("end_signal_timeout_sec", 0.50)
        self._state = PerceptionState()
        self._field_times = {}
        self._state.obstacle_distance_m = math.inf
        self._state.stop_line_distance_m = math.inf
        subscriptions = (
            (Float32, "/perception/lane_offset_m", "lane_offset_m", "camera"),
            (Float32, "/perception/lane_heading_rad", "lane_heading_error_rad", "camera"),
            (Float32, "/perception/lane_confidence", "lane_confidence", "camera"),
            (Bool, "/perception/stop_line_detected", "stop_line_detected", "camera"),
            (Float32, "/perception/stop_line_distance_m", "stop_line_distance_m", "camera"),
            (UInt8, "/perception/traffic_light", "traffic_light", "camera"),
            (Float32, "/perception/traffic_confidence", "traffic_confidence", "camera"),
            (Bool, "/perception/obstacle_in_path", "obstacle_in_path", "lidar"),
            (Float32, "/perception/obstacle_distance_m", "obstacle_distance_m", "lidar"),
            (Float32, "/perception/avoidance_steering_rad", "avoidance_steering_offset_rad", "lidar"),
            (UInt8, "/perception/end_lane_signal", "end_lane_signal", "end"),
            (Int8, "/perception/allowed_lane", "allowed_lane", "end"),
            (Float32, "/perception/end_lane_confidence", "end_lane_confidence", "end"),
        )
        for message_type, topic, field, source in subscriptions:
            self.create_subscription(
                message_type,
                topic,
                lambda message, name=field, group=source: self._store(name, group, message.data),
                10,
            )
        self._publisher = self.create_publisher(
            PerceptionState, "/perception/state", 10
        )
        self.create_timer(0.05, self._publish)

    def _store(self, field: str, source: str, value) -> None:
        setattr(self._state, field, value)
        self._field_times[field] = time.monotonic()

    def _fresh(self, source: str, parameter: str) -> bool:
        now_sec = time.monotonic()
        timeout = float(self.get_parameter(parameter).value)
        for field in self._SOURCE_FIELDS[source]:
            stamp = self._field_times.get(field)
            if stamp is None:
                return False
            age = now_sec - stamp
            if not 0.0 <= age <= timeout:
                return False
        return True

    def _publish(self) -> None:
        self._state.header.stamp = self.get_clock().now().to_msg()
        self._state.header.frame_id = "base_link"
        self._state.camera_fresh = self._fresh("camera", "camera_timeout_sec")
        self._state.lidar_fresh = self._fresh("lidar", "lidar_timeout_sec")
        self._state.end_signal_fresh = self._fresh("end", "end_signal_timeout_sec")
        self._publisher.publish(self._state)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionAggregatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
