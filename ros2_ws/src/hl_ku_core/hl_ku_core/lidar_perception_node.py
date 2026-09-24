"""2D LiDAR forward-corridor obstacle detection and conservative gap steering."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from visualization_msgs.msg import Marker, MarkerArray

from .geometry import clamp


class LidarPerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_perception")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("vehicle_half_width_m", 0.35)
        self.declare_parameter("corridor_margin_m", 0.15)
        self.declare_parameter("lidar_to_front_bumper_m", 0.15)
        self.declare_parameter("lidar_yaw_deg", 0.0)
        self.declare_parameter("minimum_range_m", 0.50)
        self.declare_parameter("detection_range_m", 12.0)
        self.declare_parameter("obstacle_trigger_m", 8.0)
        self.declare_parameter("minimum_cluster_points", 3)
        self.declare_parameter("gap_max_angle_deg", 55.0)
        self.declare_parameter("avoidance_gain", 0.65)
        self.declare_parameter("maximum_avoidance_steering_rad", 0.30)
        self.declare_parameter("center_obstacle_deadband_m", 0.10)
        self.declare_parameter("center_avoidance_steering_rad", 0.20)
        self.declare_parameter("side_clearance_min_angle_deg", 10.0)
        self.declare_parameter("obstacle_merge_depth_m", 0.50)
        self.declare_parameter("minimum_avoidance_steering_rad", 0.08)
        self.declare_parameter("avoidance_direction_hold_sec", 1.0)
        self.declare_parameter("visualization_frame", "vehicle_heading")
        self._distance_pub = self.create_publisher(Float32, "/perception/obstacle_distance_m", 10)
        self._obstacle_pub = self.create_publisher(Bool, "/perception/obstacle_in_path", 10)
        self._avoid_pub = self.create_publisher(Float32, "/perception/avoidance_steering_rad", 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, "/perception/lidar_markers", 10
        )
        self._last_invalid_scan_warning_sec = 0.0
        self._avoidance_sign = 0.0
        self._last_obstacle_sec: float | None = None
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._on_scan,
            qos_profile_sensor_data,
        )

    @staticmethod
    def _message(message_type, value):
        message = message_type()
        message.data = value
        return message

    def _publish_markers(
        self,
        scan: LaserScan,
        corridor_half_width: float,
        lidar_to_bumper: float,
        obstacle_trigger: float,
        cluster_present: bool,
        center_x: float,
        center_y: float,
        minimum_y: float,
        maximum_y: float,
        obstacle: bool,
        distance: float,
        avoidance: float,
    ) -> None:
        frame = str(self.get_parameter("visualization_frame").value)

        corridor = Marker()
        corridor.header.stamp = scan.header.stamp
        corridor.header.frame_id = frame
        corridor.ns = "lidar_detection"
        corridor.id = 0
        corridor.type = Marker.CUBE
        corridor.action = Marker.ADD
        corridor.pose.position.x = lidar_to_bumper + obstacle_trigger * 0.5
        corridor.pose.orientation.w = 1.0
        corridor.scale.x = max(0.01, obstacle_trigger)
        corridor.scale.y = max(0.01, 2.0 * corridor_half_width)
        corridor.scale.z = 0.02
        corridor.color.r = 1.0 if obstacle else 0.0
        corridor.color.g = 0.15 if obstacle else 0.8
        corridor.color.b = 0.0
        corridor.color.a = 0.12

        target = Marker()
        target.header = corridor.header
        target.ns = "lidar_detection"
        target.id = 1
        target.type = Marker.CUBE
        target.action = Marker.ADD if cluster_present else Marker.DELETE
        target.pose.position.x = center_x
        target.pose.position.y = center_y
        target.pose.position.z = 0.12
        target.pose.orientation.w = 1.0
        target.scale.x = 0.20
        target.scale.y = max(0.10, maximum_y - minimum_y)
        target.scale.z = 0.24
        target.color.r = 1.0
        target.color.g = 0.1 if obstacle else 0.65
        target.color.b = 0.0
        target.color.a = 0.85

        arrow = Marker()
        arrow.header = corridor.header
        arrow.ns = "lidar_detection"
        arrow.id = 2
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD if obstacle else Marker.DELETE
        arrow.scale.x = 0.05
        arrow.scale.y = 0.12
        arrow.scale.z = 0.16
        arrow.color.r = 0.0
        arrow.color.g = 0.65
        arrow.color.b = 1.0
        arrow.color.a = 1.0
        arrow.points = [Point(x=0.0, y=0.0, z=0.08)]
        display_angle = -avoidance
        arrow.points.append(
            Point(
                x=1.5 * math.cos(display_angle),
                y=1.5 * math.sin(display_angle),
                z=0.08,
            )
        )

        label = Marker()
        label.header = corridor.header
        label.ns = "lidar_detection"
        label.id = 3
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD if cluster_present else Marker.DELETE
        label.pose.position.x = center_x
        label.pose.position.y = center_y
        label.pose.position.z = 0.55
        label.pose.orientation.w = 1.0
        label.scale.z = 0.32
        label.color.r = 1.0
        label.color.g = 1.0
        label.color.b = 1.0
        label.color.a = 1.0
        state = "OBSTACLE" if obstacle else "OUTSIDE TRIGGER"
        label.text = f"{state} {distance:.2f} m | steer {avoidance:+.2f} rad"

        message = MarkerArray()
        message.markers = [corridor, target, arrow, label]
        # Clear the overlay if the scan stream stops instead of leaving an
        # apparently current obstacle and steering arrow on the RViz screen.
        for marker in message.markers:
            marker.lifetime.nanosec = 350_000_000
        self._marker_pub.publish(message)

    def _on_scan(self, scan: LaserScan) -> None:
        if (
            not scan.ranges
            or not math.isfinite(scan.angle_increment)
            or scan.angle_increment == 0.0
            or not math.isfinite(scan.range_min)
            or not math.isfinite(scan.range_max)
            or scan.range_max <= scan.range_min
        ):
            now_sec = time.monotonic()
            if now_sec - self._last_invalid_scan_warning_sec >= 1.0:
                self.get_logger().warning("invalid/empty LaserScan ignored")
                self._last_invalid_scan_warning_sec = now_sec
            return
        half_width = float(self.get_parameter("vehicle_half_width_m").value)
        margin = float(self.get_parameter("corridor_margin_m").value)
        lidar_to_bumper = float(self.get_parameter("lidar_to_front_bumper_m").value)
        lidar_yaw = math.radians(
            float(self.get_parameter("lidar_yaw_deg").value)
        )
        maximum_range = float(self.get_parameter("detection_range_m").value)
        minimum_range = max(
            float(self.get_parameter("minimum_range_m").value),
            float(scan.range_min),
        )
        gap_angle = math.radians(float(self.get_parameter("gap_max_angle_deg").value))
        side_min_angle = math.radians(
            float(self.get_parameter("side_clearance_min_angle_deg").value)
        )
        left_clearances: list[float] = []
        right_clearances: list[float] = []
        corridor_clusters: list[list[tuple[float, float]]] = []
        current_cluster: list[tuple[float, float]] = []

        def finish_cluster() -> None:
            if current_cluster:
                corridor_clusters.append(current_cluster.copy())
                current_cluster.clear()

        for index, distance in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment + lidar_yaw
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if side_min_angle <= abs(angle) <= gap_angle:
                clearance = (
                    distance
                    if math.isfinite(distance)
                    and minimum_range <= distance <= min(scan.range_max, maximum_range)
                    else maximum_range
                )
                (left_clearances if angle > 0.0 else right_clearances).append(
                    clearance
                )
            if (
                not math.isfinite(distance)
                or distance < minimum_range
                or distance > scan.range_max
                or distance > maximum_range
            ):
                finish_cluster()
                continue
            if abs(angle) > gap_angle:
                finish_cluster()
                continue
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            if x > lidar_to_bumper and abs(y) <= half_width + margin:
                current_cluster.append((x, y))
            else:
                finish_cluster()
        finish_cluster()
        minimum_points = int(self.get_parameter("minimum_cluster_points").value)
        qualified_clusters = [
            cluster for cluster in corridor_clusters if len(cluster) >= minimum_points
        ]
        nearest_cluster = min(
            qualified_clusters,
            key=lambda cluster: min(point[0] for point in cluster),
            default=None,
        )
        cluster_present = nearest_cluster is not None
        distance = math.inf
        obstacle_angle = 0.0
        obstacle_lateral = 0.0
        obstacle_min_y = 0.0
        obstacle_max_y = 0.0
        obstacle_center_x = 0.0
        if nearest_cluster is not None:
            nearest_x = min(point[0] for point in nearest_cluster)
            merge_depth = max(
                0.0, float(self.get_parameter("obstacle_merge_depth_m").value)
            )
            obstacle_points = [
                point
                for cluster in qualified_clusters
                if min(item[0] for item in cluster) <= nearest_x + merge_depth
                for point in cluster
            ]
            distance = nearest_x - lidar_to_bumper
            center_x = sum(point[0] for point in obstacle_points) / len(
                obstacle_points
            )
            center_y = sum(point[1] for point in obstacle_points) / len(
                obstacle_points
            )
            obstacle_center_x = center_x
            obstacle_angle = math.atan2(center_y, center_x)
            obstacle_lateral = center_y
            obstacle_min_y = min(point[1] for point in obstacle_points)
            obstacle_max_y = max(point[1] for point in obstacle_points)
        obstacle = cluster_present and distance <= float(
            self.get_parameter("obstacle_trigger_m").value
        )
        avoidance = 0.0
        now_sec = time.monotonic()
        if obstacle:
            maximum_steering = float(
                self.get_parameter("maximum_avoidance_steering_rad").value
            )
            center_deadband = float(
                self.get_parameter("center_obstacle_deadband_m").value
            )
            spans_center = (
                obstacle_min_y <= -center_deadband
                and obstacle_max_y >= center_deadband
            )
            if spans_center or abs(obstacle_lateral) <= center_deadband:
                left_clearance = (
                    sorted(left_clearances)[len(left_clearances) // 2]
                    if left_clearances
                    else maximum_range
                )
                right_clearance = (
                    sorted(right_clearances)[len(right_clearances) // 2]
                    if right_clearances
                    else maximum_range
                )
                center_steering = min(
                    abs(
                        float(
                            self.get_parameter(
                                "center_avoidance_steering_rad"
                            ).value
                        )
                    ),
                    maximum_steering,
                )
                # Vehicle convention: negative=left, positive=right.
                candidate_avoidance = (
                    -center_steering
                    if left_clearance > right_clearance
                    else center_steering
                )
            else:
                candidate_avoidance = clamp(
                    float(self.get_parameter("avoidance_gain").value)
                    * obstacle_angle,
                    -maximum_steering,
                    maximum_steering,
                )
            hold_sec = max(
                0.0,
                float(
                    self.get_parameter("avoidance_direction_hold_sec").value
                ),
            )
            if (
                self._avoidance_sign == 0.0
                or self._last_obstacle_sec is None
                or now_sec - self._last_obstacle_sec > hold_sec
            ):
                self._avoidance_sign = (
                    -1.0 if candidate_avoidance < 0.0 else 1.0
                )
            self._last_obstacle_sec = now_sec
            minimum_steering = min(
                maximum_steering,
                max(
                    0.0,
                    float(
                        self.get_parameter(
                            "minimum_avoidance_steering_rad"
                        ).value
                    ),
                ),
            )
            avoidance = self._avoidance_sign * max(
                minimum_steering, abs(candidate_avoidance)
            )
        elif (
            self._last_obstacle_sec is not None
            and now_sec - self._last_obstacle_sec
            > max(
                0.0,
                float(
                    self.get_parameter("avoidance_direction_hold_sec").value
                ),
            )
        ):
            self._avoidance_sign = 0.0
            self._last_obstacle_sec = None
        self._distance_pub.publish(self._message(Float32, float(distance)))
        self._obstacle_pub.publish(self._message(Bool, bool(obstacle)))
        self._avoid_pub.publish(self._message(Float32, float(avoidance)))
        self._publish_markers(
            scan,
            half_width + margin,
            lidar_to_bumper,
            float(self.get_parameter("obstacle_trigger_m").value),
            cluster_present,
            obstacle_center_x,
            obstacle_lateral,
            obstacle_min_y,
            obstacle_max_y,
            obstacle,
            distance,
            avoidance,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
