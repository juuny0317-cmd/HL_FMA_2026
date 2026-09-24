"""Project fresh front LiDAR clusters into map-frame obstacle centers."""

from __future__ import annotations

import math
import time

from geometry_msgs.msg import (
    Pose,
    PoseArray,
    PoseWithCovarianceStamped,
    TransformStamped,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from hl_ku_interfaces.msg import Obstacle2D, ObstacleArray

from .scan_obstacle_mapping import (
    MapperSettings,
    VehiclePose,
    clustered_obstacle_discs_in_map,
)


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _yaw_from_quaternion(orientation) -> float:
    values = (
        float(orientation.x),
        float(orientation.y),
        float(orientation.z),
        float(orientation.w),
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("pose quaternion is not finite")
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 0.5:
        raise ValueError("pose quaternion is invalid")
    x, y, z, w = (value / norm for value in values)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _lidar_transform(
    stamp,
    map_frame: str,
    scan_frame: str,
    vehicle_pose: VehiclePose,
    settings: MapperSettings,
) -> TransformStamped:
    """Describe the measured LiDAR origin in map for live RViz scan display."""

    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = map_frame
    transform.child_frame_id = scan_frame
    cosine = math.cos(vehicle_pose.yaw_rad)
    sine = math.sin(vehicle_pose.yaw_rad)
    transform.transform.translation.x = (
        vehicle_pose.x_m
        + cosine * settings.lidar_x_m
        - sine * settings.lidar_y_m
    )
    transform.transform.translation.y = (
        vehicle_pose.y_m
        + sine * settings.lidar_x_m
        + cosine * settings.lidar_y_m
    )
    yaw = vehicle_pose.yaw_rad + settings.lidar_yaw_rad
    transform.transform.rotation.z = math.sin(yaw * 0.5)
    transform.transform.rotation.w = math.cos(yaw * 0.5)
    return transform


def _obstacle_markers(
    stamp, map_frame: str, discs: tuple[tuple[float, float, float], ...]
) -> MarkerArray:
    """Make detected map-frame clusters visible without changing perception."""

    message = MarkerArray()
    clear = Marker()
    clear.header.stamp = stamp
    clear.header.frame_id = map_frame
    clear.action = Marker.DELETEALL
    message.markers.append(clear)
    for index, (x_m, y_m, radius_m) in enumerate(discs):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = map_frame
        marker.ns = "lidar_clusters"
        marker.id = index
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = x_m
        marker.pose.position.y = y_m
        marker.pose.position.z = 0.15
        marker.pose.orientation.w = 1.0
        marker.scale.x = radius_m * 2.0
        marker.scale.y = radius_m * 2.0
        marker.scale.z = 0.30
        marker.color.r = 1.0
        marker.color.g = 0.12
        marker.color.b = 0.05
        marker.color.a = 0.88
        message.markers.append(marker)
    return message


def _scan_points_in_map(
    scan: LaserScan,
    vehicle_pose: VehiclePose,
    settings: MapperSettings,
) -> tuple[tuple[float, float, float], ...]:
    """Project every finite raw scan return into map for RViz/Foxglove."""

    points: list[tuple[float, float, float]] = []
    vehicle_cosine = math.cos(vehicle_pose.yaw_rad)
    vehicle_sine = math.sin(vehicle_pose.yaw_rad)
    for index, distance in enumerate(scan.ranges):
        if (
            not math.isfinite(distance)
            or distance < float(scan.range_min)
            or distance > float(scan.range_max)
        ):
            continue
        angle = (
            float(scan.angle_min)
            + index * float(scan.angle_increment)
            + settings.lidar_yaw_rad
        )
        base_x = settings.lidar_x_m + distance * math.cos(angle)
        base_y = settings.lidar_y_m + distance * math.sin(angle)
        points.append(
            (
                vehicle_pose.x_m
                + vehicle_cosine * base_x
                - vehicle_sine * base_y,
                vehicle_pose.y_m
                + vehicle_sine * base_x
                + vehicle_cosine * base_y,
                0.0,
            )
        )
    return tuple(points)


class ScanObstacleMapperNode(Node):
    def __init__(self) -> None:
        super().__init__("scan_obstacle_mapper")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("pose_topic", "/localization/gnss_pose")
        self.declare_parameter("obstacle_topic", "/planning/obstacles_map")
        self.declare_parameter("obstacle_discs_topic", "/planning/obstacle_discs_map")
        self.declare_parameter("obstacle_markers_topic", "/planning/obstacle_markers_map")
        self.declare_parameter("scan_points_topic", "/planning/scan_points_map")
        self.declare_parameter("status_topic", "/planning/obstacle_mapper_status")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("expected_scan_frame", "laser")
        self.declare_parameter("geometry_verified", False)
        self.declare_parameter("pose_timeout_sec", 1.50)
        self.declare_parameter("maximum_stamp_offset_sec", 1.50)
        self.declare_parameter("minimum_range_m", 0.50)
        self.declare_parameter("maximum_range_m", 12.0)
        self.declare_parameter("front_half_angle_deg", 70.0)
        self.declare_parameter("minimum_cluster_points", 3)
        self.declare_parameter("maximum_cluster_gap_m", 0.35)
        self.declare_parameter("lidar_x_m", 0.0)
        self.declare_parameter("lidar_y_m", 0.0)
        self.declare_parameter("lidar_yaw_deg", 0.0)
        self.declare_parameter("minimum_obstacle_radius_m", 0.40)
        self.declare_parameter("radius_margin_m", 0.10)

        self._pose: PoseWithCovarianceStamped | None = None
        self._pose_received_at: float | None = None
        self._last_status = ""
        self._obstacle_pub = self.create_publisher(
            PoseArray, str(self.get_parameter("obstacle_topic").value), 10
        )
        self._discs_pub = self.create_publisher(
            ObstacleArray, str(self.get_parameter("obstacle_discs_topic").value), 10
        )
        self._markers_pub = self.create_publisher(
            MarkerArray, str(self.get_parameter("obstacle_markers_topic").value), 10
        )
        self._scan_points_pub = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("scan_points_topic").value),
            qos_profile_sensor_data,
        )
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("pose_topic").value),
            self._on_pose,
            20,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._on_scan,
            qos_profile_sensor_data,
        )

    def _publish_status(self, value: str) -> None:
        message = String()
        message.data = value
        self._status_pub.publish(message)
        if value != self._last_status:
            self.get_logger().info(f"LiDAR obstacle mapper: {value}")
            self._last_status = value

    def _on_pose(self, message: PoseWithCovarianceStamped) -> None:
        if message.header.frame_id != str(self.get_parameter("map_frame").value):
            self._publish_status("POSE_FRAME_INVALID")
            return
        self._pose = message
        self._pose_received_at = time.monotonic()

    def _settings(self) -> MapperSettings:
        return MapperSettings(
            minimum_range_m=float(self.get_parameter("minimum_range_m").value),
            maximum_range_m=float(self.get_parameter("maximum_range_m").value),
            front_half_angle_rad=math.radians(
                float(self.get_parameter("front_half_angle_deg").value)
            ),
            minimum_cluster_points=int(
                self.get_parameter("minimum_cluster_points").value
            ),
            maximum_cluster_gap_m=float(
                self.get_parameter("maximum_cluster_gap_m").value
            ),
            lidar_x_m=float(self.get_parameter("lidar_x_m").value),
            lidar_y_m=float(self.get_parameter("lidar_y_m").value),
            lidar_yaw_rad=math.radians(
                float(self.get_parameter("lidar_yaw_deg").value)
            ),
            minimum_obstacle_radius_m=float(
                self.get_parameter("minimum_obstacle_radius_m").value
            ),
            radius_margin_m=float(self.get_parameter("radius_margin_m").value),
        )

    def _on_scan(self, scan: LaserScan) -> None:
        if not bool(self.get_parameter("geometry_verified").value):
            self._publish_status("GEOMETRY_UNVERIFIED")
            return
        expected_frame = str(self.get_parameter("expected_scan_frame").value)
        if expected_frame and scan.header.frame_id != expected_frame:
            self._publish_status("SCAN_FRAME_INVALID")
            return
        pose = self._pose
        pose_received_at = self._pose_received_at
        timeout = float(self.get_parameter("pose_timeout_sec").value)
        if (
            pose is None
            or pose_received_at is None
            or not 0.0 <= time.monotonic() - pose_received_at <= timeout
        ):
            self._publish_status("POSE_STALE")
            return
        scan_stamp = _stamp_seconds(scan.header.stamp)
        pose_stamp = _stamp_seconds(pose.header.stamp)
        maximum_offset = float(
            self.get_parameter("maximum_stamp_offset_sec").value
        )
        if (
            scan_stamp <= 0.0
            or pose_stamp <= 0.0
            or abs(scan_stamp - pose_stamp) > maximum_offset
        ):
            self._publish_status("STAMP_MISMATCH")
            return
        position = pose.pose.pose.position
        try:
            vehicle_pose = VehiclePose(
                float(position.x),
                float(position.y),
                _yaw_from_quaternion(pose.pose.pose.orientation),
            )
            settings = self._settings()
            discs = clustered_obstacle_discs_in_map(
                scan.ranges,
                float(scan.angle_min),
                float(scan.angle_increment),
                float(scan.range_min),
                float(scan.range_max),
                vehicle_pose,
                settings,
            )
        except ValueError as error:
            self._publish_status("INVALID_INPUT")
            self.get_logger().warning(str(error))
            return

        message = PoseArray()
        message.header.stamp = scan.header.stamp
        message.header.frame_id = str(self.get_parameter("map_frame").value)
        disc_message = ObstacleArray()
        disc_message.header = message.header
        for x_m, y_m, radius_m in discs:
            obstacle = Pose()
            obstacle.position.x = x_m
            obstacle.position.y = y_m
            obstacle.orientation.w = 1.0
            message.poses.append(obstacle)
            disc = Obstacle2D()
            disc.x_m = x_m
            disc.y_m = y_m
            disc.radius_m = radius_m
            disc_message.obstacles.append(disc)
        self._obstacle_pub.publish(message)
        self._discs_pub.publish(disc_message)
        self._markers_pub.publish(
            _obstacle_markers(scan.header.stamp, message.header.frame_id, discs)
        )
        cloud_header = Header()
        cloud_header.stamp = scan.header.stamp
        cloud_header.frame_id = message.header.frame_id
        self._scan_points_pub.publish(
            point_cloud2.create_cloud_xyz32(
                cloud_header,
                _scan_points_in_map(scan, vehicle_pose, settings),
            )
        )
        self._tf_broadcaster.sendTransform(
            _lidar_transform(
                scan.header.stamp,
                message.header.frame_id,
                scan.header.frame_id,
                vehicle_pose,
                settings,
            )
        )
        self._publish_status(f"OK:{len(discs)}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanObstacleMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
