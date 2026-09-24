"""Select a signed speed estimate, favouring LiDAR odometry near zero speed."""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class VelocitySelectorNode(Node):
    def __init__(self) -> None:
        super().__init__("velocity_selector")
        self.declare_parameter("lidar_odometry_topic", "/lidar/odometry")
        self.declare_parameter("lidar_timeout_sec", 0.25)
        self.declare_parameter("gnss_timeout_sec", 0.40)
        self.declare_parameter("gnss_preferred_above_mps", 0.35)
        self.declare_parameter("use_lidar_odometry", True)
        self._gnss: TwistStamped | None = None
        self._lidar: Odometry | None = None
        self._gnss_time = None
        self._lidar_time = None
        self._publisher = self.create_publisher(
            TwistStamped, "/localization/vehicle_velocity", 20
        )
        self.create_subscription(
            TwistStamped, "/localization/gnss_velocity", self._on_gnss, 20
        )
        if bool(self.get_parameter("use_lidar_odometry").value):
            self.create_subscription(
                Odometry,
                str(self.get_parameter("lidar_odometry_topic").value),
                self._on_lidar,
                20,
            )
        self.create_timer(0.02, self._publish)

    def _on_gnss(self, message: TwistStamped) -> None:
        self._gnss = message
        self._gnss_time = time.monotonic()

    def _on_lidar(self, message: Odometry) -> None:
        self._lidar = message
        self._lidar_time = time.monotonic()

    def _fresh(self, stamp, timeout: float) -> bool:
        if stamp is None:
            return False
        age = time.monotonic() - stamp
        return 0.0 <= age <= timeout

    def _publish(self) -> None:
        gnss_fresh = self._fresh(
            self._gnss_time, float(self.get_parameter("gnss_timeout_sec").value)
        )
        lidar_fresh = self._fresh(
            self._lidar_time, float(self.get_parameter("lidar_timeout_sec").value)
        )
        output = TwistStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = "base_link"
        threshold = float(self.get_parameter("gnss_preferred_above_mps").value)
        if gnss_fresh and (
            not lidar_fresh or abs(self._gnss.twist.linear.x) >= threshold
        ):
            output.twist = self._gnss.twist
        elif lidar_fresh:
            output.twist = self._lidar.twist.twist
        elif gnss_fresh:
            output.twist = self._gnss.twist
        else:
            return
        self._publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VelocitySelectorNode()
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
