"""Publish a latched, periodically refreshed autonomous-readiness decision."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from .readiness import (
    check_autonomous_readiness,
    check_global_route_test_readiness,
    check_gps_only_route_test_readiness,
)


class PreflightCheckNode(Node):
    def __init__(self) -> None:
        super().__init__("preflight_check")
        self.declare_parameter("system_config_file", "")
        self.declare_parameter("override_config_file", "")
        self.declare_parameter("ntrip_config_file", "")
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("route_file", "")
        self.declare_parameter("profile", "autonomous")
        self.declare_parameter("publish_period_sec", 1.0)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._ready_pub = self.create_publisher(Bool, "/system/preflight_ok", qos)
        self._report_pub = self.create_publisher(String, "/system/preflight_report", qos)
        self._last_summary = ""
        period = max(0.2, float(self.get_parameter("publish_period_sec").value))
        self.create_timer(period, self._publish)
        self._publish()

    def _publish(self) -> None:
        profile = str(self.get_parameter("profile").value)
        if profile == "gps_only_route_test":
            report = check_gps_only_route_test_readiness(
                str(self.get_parameter("system_config_file").value),
                str(self.get_parameter("override_config_file").value),
                str(self.get_parameter("ntrip_config_file").value),
                str(self.get_parameter("calibration_file").value),
                str(self.get_parameter("route_file").value),
            )
        elif profile == "global_route_test":
            report = check_global_route_test_readiness(
                str(self.get_parameter("system_config_file").value),
                str(self.get_parameter("override_config_file").value),
                str(self.get_parameter("calibration_file").value),
                str(self.get_parameter("route_file").value),
            )
        elif profile == "autonomous":
            report = check_autonomous_readiness(
                str(self.get_parameter("system_config_file").value),
                str(self.get_parameter("calibration_file").value),
                str(self.get_parameter("route_file").value),
            )
        else:
            raise ValueError(f"unknown preflight profile: {profile}")
        ready_message = Bool()
        ready_message.data = report.ok
        self._ready_pub.publish(ready_message)
        report_message = String()
        report_message.data = report.summary()
        self._report_pub.publish(report_message)
        if report_message.data != self._last_summary:
            if report.ok:
                self.get_logger().info(report_message.data)
            else:
                self.get_logger().error(report_message.data)
            self._last_summary = report_message.data


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PreflightCheckNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
