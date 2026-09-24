"""Publish live pose/track and source state for the Foxglove dashboard."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class LiveStatusVisualizerNode(Node):
    def __init__(self) -> None:
        super().__init__("hl_ku_foxglove_live_status")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("track_minimum_spacing_m", 0.15)
        self.declare_parameter("track_maximum_points", 6000)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._minimum_spacing = max(
            0.01, float(self.get_parameter("track_minimum_spacing_m").value)
        )
        self._maximum_points = max(
            100, int(self.get_parameter("track_maximum_points").value)
        )
        static_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._mode_pub = self.create_publisher(
            String, "/visualization/replay_mode", static_qos
        )
        self._pose_pub = self.create_publisher(
            PoseStamped, "/visualization/current_pose", 10
        )
        self._track_pub = self.create_publisher(Path, "/visualization/actual_path", 2)
        self._track = Path()
        self._track.header.frame_id = self._frame_id
        self._last_xy: tuple[float, float] | None = None
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/localization/gnss_pose",
            self._on_pose,
            20,
        )
        self.create_timer(1.0, self._publish_status)
        self._publish_status()

    def _on_pose(self, message: PoseWithCovarianceStamped) -> None:
        pose = PoseStamped()
        pose.header = message.header
        pose.header.frame_id = self._frame_id
        pose.pose = message.pose.pose
        self._pose_pub.publish(pose)
        position = pose.pose.position
        if self._last_xy is None or math.hypot(
            position.x - self._last_xy[0], position.y - self._last_xy[1]
        ) >= self._minimum_spacing:
            self._track.poses.append(pose)
            self._track.poses = self._track.poses[-self._maximum_points :]
            self._last_xy = (position.x, position.y)

    def _publish_status(self) -> None:
        mode = String()
        mode.data = "LIVE_DRIVE: camera + GNSS"
        self._mode_pub.publish(mode)
        if self._track.poses:
            self._track.header.stamp = self.get_clock().now().to_msg()
            self._track_pub.publish(self._track)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LiveStatusVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
