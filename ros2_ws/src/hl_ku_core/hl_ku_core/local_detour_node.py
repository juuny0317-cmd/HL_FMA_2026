"""Publish a local detour candidate for RViz, never an actuator command."""

from __future__ import annotations

import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import Path as PathMessage
from rclpy.node import Node
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker, MarkerArray
from hl_ku_interfaces.msg import ObstacleArray

from .local_detour import (
    CircleObstacle,
    DetourResult,
    DetourSettings,
    _extend_obstacles_along_route,
    _path_lateral_offset_at_s,
    _path_clear,
    plan_local_detour,
)
from .route import Route


class LocalDetourNode(Node):
    def __init__(self) -> None:
        super().__init__("local_detour")
        self.declare_parameter("route_file", "")
        self.declare_parameter("preview_mode", False)
        self.declare_parameter("preview_progress_s_m", 200.0)
        self.declare_parameter("preview_obstacle_s_m", 210.0)
        self.declare_parameter("preview_obstacle_lateral_m", 0.0)
        self.declare_parameter("preview_obstacle_radius_m", 0.25)
        self.declare_parameter("obstacle_topic", "/planning/obstacle_discs_map")
        self.declare_parameter("live_geometry_verified", False)
        self.declare_parameter("obstacle_timeout_sec", 0.30)
        self.declare_parameter("progress_timeout_sec", 0.30)
        self.declare_parameter("vehicle_half_width_m", 0.35)
        self.declare_parameter("safety_margin_m", 0.20)
        self.declare_parameter("corridor_half_width_m", 1.70)
        self.declare_parameter("preview_distance_m", 18.0)
        self.declare_parameter("entry_length_m", 5.0)
        self.declare_parameter("minimum_entry_length_m", 1.0)
        self.declare_parameter("assumed_obstacle_length_m", 1.3)
        self.declare_parameter("exit_length_m", 5.0)
        self.declare_parameter("maximum_curvature_per_m", 0.75)
        self._preview_mode = bool(self.get_parameter("preview_mode").value)
        self._route: Route | None = None
        route_path = Path(str(self.get_parameter("route_file").value))
        try:
            self._route = Route.load_csv(route_path)
        except (OSError, ValueError, KeyError) as error:
            self.get_logger().error(f"detour route unavailable: {error}")
        self._obstacles: tuple[CircleObstacle, ...] | None = None
        self._obstacles_at: float | None = None
        self._progress_s_m: float | None = None
        self._progress_at: float | None = None
        self._active_candidate: DetourResult | None = None
        self._active_obstacles: tuple[CircleObstacle, ...] = ()
        self._active_settings: DetourSettings | None = None
        # The synthetic preview must never share the steering input topics.
        output_prefix = "/planning" if self._preview_mode else "/planning/live"
        self._path_pub = self.create_publisher(
            PathMessage, f"{output_prefix}/local_detour_candidate", 10
        )
        self._route_pub = self.create_publisher(
            PathMessage, "/planning/reference_path_preview", 10
        )
        self._obstacle_pub = self.create_publisher(
            PoseArray, "/planning/detour_preview_obstacles", 10
        )
        self._marker_pub = self.create_publisher(
            MarkerArray, "/planning/detour_preview_markers", 10
        )
        self._status_pub = self.create_publisher(
            String, f"{output_prefix}/local_detour_status", 10
        )
        self.create_subscription(
            ObstacleArray,
            str(self.get_parameter("obstacle_topic").value),
            self._on_obstacles,
            10,
        )
        self.create_subscription(Float32, "/planning/route_s", self._on_progress, 10)
        self.create_timer(0.5, self._update)

    def _on_obstacles(self, message: ObstacleArray) -> None:
        if message.header.frame_id != "map":
            self.get_logger().warning("obstacle array must use map frame")
            return
        self._obstacles = tuple(
            CircleObstacle(float(p.x_m), float(p.y_m), float(p.radius_m))
            for p in message.obstacles
        )
        self._obstacles_at = time.monotonic()

    def _on_progress(self, message: Float32) -> None:
        self._progress_s_m = float(message.data)
        self._progress_at = time.monotonic()

    def _fresh(self, timestamp: float | None, timeout: float) -> bool:
        return timestamp is not None and 0.0 <= time.monotonic() - timestamp <= timeout

    def _settings(self) -> DetourSettings:
        return DetourSettings(
            vehicle_half_width_m=float(self.get_parameter("vehicle_half_width_m").value),
            safety_margin_m=float(self.get_parameter("safety_margin_m").value),
            corridor_half_width_m=float(self.get_parameter("corridor_half_width_m").value),
            preview_distance_m=float(self.get_parameter("preview_distance_m").value),
            entry_length_m=float(self.get_parameter("entry_length_m").value),
            minimum_entry_length_m=float(
                self.get_parameter("minimum_entry_length_m").value
            ),
            assumed_obstacle_length_m=float(
                self.get_parameter("assumed_obstacle_length_m").value
            ),
            exit_length_m=float(self.get_parameter("exit_length_m").value),
            maximum_curvature_per_m=float(
                self.get_parameter("maximum_curvature_per_m").value
            ),
        )

    def _path(self, points: tuple[tuple[float, float], ...]) -> PathMessage:
        message = PathMessage()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        for index, (x, y) in enumerate(points):
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            other = points[min(index + 1, len(points) - 1)]
            if index == len(points) - 1 and index > 0:
                other = points[index - 1]
                heading = math.atan2(y - other[1], x - other[0])
            elif other == (x, y):
                heading = 0.0
            else:
                heading = math.atan2(other[1] - y, other[0] - x)
            pose.pose.orientation.z = math.sin(heading / 2.0)
            pose.pose.orientation.w = math.cos(heading / 2.0)
            message.poses.append(pose)
        return message

    def _preview_inputs(self) -> tuple[float, tuple[CircleObstacle, ...]]:
        assert self._route is not None
        obstacle_s = float(self.get_parameter("preview_obstacle_s_m").value)
        lateral = float(self.get_parameter("preview_obstacle_lateral_m").value)
        x, y = self._route.position_at_s(obstacle_s)
        heading = self._route.heading_at_s(obstacle_s)
        obstacle = CircleObstacle(
            x - math.sin(heading) * lateral,
            y + math.cos(heading) * lateral,
            float(self.get_parameter("preview_obstacle_radius_m").value),
        )
        return float(self.get_parameter("preview_progress_s_m").value), (obstacle,)

    def _publish_markers(
        self,
        obstacles: tuple[CircleObstacle, ...],
        points: tuple[tuple[float, float], ...],
    ) -> None:
        stamp = self.get_clock().now().to_msg()
        message = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        message.markers.append(clear)
        for index, obstacle in enumerate(obstacles):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = stamp
            marker.ns = "detour_obstacles"
            marker.id = index
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = obstacle.x_m
            marker.pose.position.y = obstacle.y_m
            marker.pose.orientation.w = 1.0
            marker.scale.x = max(0.01, 2.0 * obstacle.radius_m)
            marker.scale.y = max(0.01, 2.0 * obstacle.radius_m)
            marker.scale.z = 0.04
            marker.color.r = 1.0
            marker.color.g = 0.15
            marker.color.b = 0.10
            marker.color.a = 0.85
            message.markers.append(marker)
        if points:
            for index, (label, point, color) in enumerate(
                (
                    ("DETOUR START", points[0], (0.1, 1.0, 0.2)),
                    ("REJOIN", points[-1], (0.1, 0.55, 1.0)),
                )
            ):
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = stamp
                marker.ns = "detour_endpoints"
                marker.id = index
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = point[0]
                marker.pose.position.y = point[1]
                marker.pose.orientation.w = 1.0
                marker.scale.x = marker.scale.y = marker.scale.z = 0.35
                marker.color.r, marker.color.g, marker.color.b = color
                marker.color.a = 1.0
                message.markers.append(marker)
                text = Marker()
                text.header.frame_id = "map"
                text.header.stamp = stamp
                text.ns = "detour_labels"
                text.id = index
                text.type = Marker.TEXT_VIEW_FACING
                text.action = Marker.ADD
                text.pose.position.x = point[0]
                text.pose.position.y = point[1]
                text.pose.position.z = 0.6
                text.pose.orientation.w = 1.0
                text.scale.z = 0.32
                text.color.r = text.color.g = text.color.b = text.color.a = 1.0
                text.text = label
                message.markers.append(text)
        self._marker_pub.publish(message)

    def _update(self) -> None:
        if self._route is None:
            status = "ROUTE_UNAVAILABLE"
            points: tuple[tuple[float, float], ...] = ()
            obstacles: tuple[CircleObstacle, ...] = ()
        else:
            self._route_pub.publish(
                self._path(tuple((p.x_m, p.y_m) for p in self._route.waypoints))
            )
            if self._preview_mode:
                progress, obstacles = self._preview_inputs()
                result = plan_local_detour(self._route, progress, obstacles, self._settings())
                status, points = result.status, result.points
            elif not bool(self.get_parameter("live_geometry_verified").value):
                self._active_candidate = None
                self._active_obstacles = ()
                self._active_settings = None
                status, points, obstacles = "GEOMETRY_UNVERIFIED", (), ()
            elif not self._fresh(
                self._progress_at, float(self.get_parameter("progress_timeout_sec").value)
            ):
                self._active_candidate = None
                self._active_obstacles = ()
                self._active_settings = None
                status, points, obstacles = "PROGRESS_STALE", (), ()
            elif not self._fresh(
                self._obstacles_at, float(self.get_parameter("obstacle_timeout_sec").value)
            ):
                self._active_candidate = None
                self._active_obstacles = ()
                self._active_settings = None
                status, points, obstacles = "OBSTACLES_STALE", (), ()
            else:
                obstacles = self._obstacles or ()
                settings = self._settings()
                active = self._active_candidate
                if any(
                    not all(math.isfinite(value) for value in (
                        obstacle.x_m, obstacle.y_m, obstacle.radius_m
                    )) or obstacle.radius_m < 0.0
                    for obstacle in obstacles
                ):
                    self._active_candidate = None
                    self._active_obstacles = ()
                    self._active_settings = None
                    result = DetourResult("INVALID_OBSTACLE")
                elif (
                    active is not None
                    and self._active_settings == settings
                    and active.end_s_m is not None
                    and self._progress_s_m <= active.end_s_m
                ):
                    # Keep the same path through the maneuver. Replanning from
                    # the current s would otherwise reject it as TOO_CLOSE.
                    known_obstacles = self._active_obstacles + obstacles
                    collision_obstacles = _extend_obstacles_along_route(
                        self._route,
                        known_obstacles,
                        active.end_s_m,
                        settings.assumed_obstacle_length_m,
                        settings.sample_step_m,
                    )
                    if _path_clear(
                        active.points,
                        collision_obstacles,
                        settings.vehicle_half_width_m + settings.safety_margin_m,
                    ):
                        result = active
                    else:
                        self._active_candidate = None
                        self._active_obstacles = ()
                        self._active_settings = None
                        # A new obstacle can appear while returning to the route.
                        # Continue from the current detour offset instead of
                        # jumping back to the global-route centerline.
                        current_offset_m = _path_lateral_offset_at_s(
                            self._route,
                            active,
                            self._progress_s_m,
                            settings.sample_step_m,
                        )
                        result = plan_local_detour(
                            self._route,
                            self._progress_s_m,
                            known_obstacles,
                            settings,
                            initial_lateral_offset_m=current_offset_m,
                        )
                        if result.status == "CANDIDATE":
                            self._active_candidate = result
                            self._active_obstacles = known_obstacles
                            self._active_settings = settings
                else:
                    self._active_candidate = None
                    self._active_obstacles = ()
                    self._active_settings = None
                    result = plan_local_detour(
                        self._route, self._progress_s_m, obstacles, settings
                    )
                    if result.status == "CANDIDATE":
                        self._active_candidate = result
                        self._active_obstacles = obstacles
                        self._active_settings = settings
                status, points = result.status, result.points
        self._path_pub.publish(self._path(points))
        obstacle_message = PoseArray()
        obstacle_message.header.stamp = self.get_clock().now().to_msg()
        obstacle_message.header.frame_id = "map"
        for obstacle in obstacles:
            pose = Pose()
            pose.position.x = obstacle.x_m
            pose.position.y = obstacle.y_m
            pose.orientation.w = 1.0
            obstacle_message.poses.append(pose)
        self._obstacle_pub.publish(obstacle_message)
        self._publish_markers(obstacles, points)
        status_message = String()
        status_message.data = status
        self._status_pub.publish(status_message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalDetourNode()
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
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass
