"""GNSS pose route tracker with bounded camera-lane correction."""

from __future__ import annotations

import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Path as PathMessage
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String

from hl_ku_interfaces.msg import DriveCommand, MissionStatus, PerceptionState

from .control import (
    normalize_angle,
    pure_pursuit_steering,
    stanley_steering,
)
from .geometry import clamp
from .endpoint_selection import blended_final_route
from .parking_selection import ParkingRouteFamily, parse_variant_id, variant_id
from .route import Route
from .route_progress import (
    RouteProgressMatch,
    continuous_nearest_index,
    continuous_search_bounds,
    direction_runs,
    match_route_ahead,
    match_saved_progress,
)


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def effective_target_speed(
    route_speed_mps: float,
    direction: int,
    override_speed_mps: float,
) -> float:
    """Apply a cruise-speed override without defeating a route stop point."""
    if route_speed_mps <= 0.0 or direction == 0:
        return 0.0
    target = override_speed_mps if override_speed_mps > 0.0 else route_speed_mps
    return target * (-1.0 if direction < 0 else 1.0)


def selected_path_steering(
    direction: int,
    stanley_rad: float,
    pure_pursuit_rad: float,
    maximum_steering_rad: float,
) -> tuple[float, float, str]:
    """Select Stanley-only steering for both authored motion directions."""
    if direction > 0:
        return (
            clamp(stanley_rad, -maximum_steering_rad, maximum_steering_rad),
            0.0,
            "STANLEY_100",
        )
    if direction < 0:
        return (
            clamp(stanley_rad, -maximum_steering_rad, maximum_steering_rad),
            0.0,
            "STANLEY_REVERSE_100",
        )
    return 0.0, 0.0, "STOP"


def bounded_route_heading(
    route: Route,
    route_s_m: float,
    run_start_index: int,
    run_end_index: int,
    baseline_m: float,
) -> float:
    """Estimate route tangent without crossing a direction-change cusp."""

    start_s = route.waypoints[run_start_index].s_m
    end_s = route.waypoints[run_end_index].s_m
    baseline = min(max(1.0e-3, baseline_m), max(1.0e-3, end_s - start_s))
    center = max(start_s, min(route_s_m, end_s))
    first_s = max(start_s, center - 0.5 * baseline)
    second_s = min(end_s, first_s + baseline)
    first_s = max(start_s, second_s - baseline)
    first = route.position_at_s(first_s)
    second = route.position_at_s(second_s)
    return math.atan2(second[1] - first[1], second[0] - first[0])


class PathTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("path_tracker")
        self.declare_parameter("route_file", "")
        self.declare_parameter("route_calibrated", False)
        self.declare_parameter("target_speed_override_mps", 0.0)
        self.declare_parameter("wheelbase_m", 0.58)
        self.declare_parameter("lookahead_base_m", 0.65)
        self.declare_parameter("lookahead_speed_gain_sec", 0.45)
        self.declare_parameter("maximum_steering_rad", 0.48)
        self.declare_parameter("stanley_gain", 0.65)
        self.declare_parameter("stanley_softening_mps", 0.65)
        self.declare_parameter("stanley_control_offset_m", 0.58)
        self.declare_parameter("stanley_heading_baseline_m", 1.0)
        # Reverse parking needs a gentler controller than forward travel.  The
        # reverse values are selected only while the active route run has
        # direction=-1; forward Stanley remains unchanged.
        self.declare_parameter("reverse_stanley_gain", 0.45)
        self.declare_parameter("reverse_stanley_softening_mps", 0.90)
        self.declare_parameter("reverse_stanley_control_offset_m", 0.75)
        self.declare_parameter("reverse_stanley_heading_baseline_m", 1.5)
        self.declare_parameter("reverse_stanley_heading_gain", 0.70)
        self.declare_parameter("reverse_use_measured_speed", True)
        self.declare_parameter("curvature_preview_m", 3.0)
        self.declare_parameter("curvature_segment_count", 3)
        self.declare_parameter("finish_tolerance_m", 0.50)
        self.declare_parameter("direction_change_distance_m", 0.35)
        self.declare_parameter("direction_change_hold_sec", 0.75)
        self.declare_parameter("direction_change_stopped_speed_mps", 0.04)
        self.declare_parameter("lane_offset_gain", 0.30)
        self.declare_parameter("lane_heading_gain", 0.45)
        self.declare_parameter("minimum_lane_confidence", 0.55)
        self.declare_parameter("enable_lane_correction", True)
        self.declare_parameter("enable_parking_selection", False)
        self.declare_parameter("enable_endpoint_selection", False)
        self.declare_parameter("endpoint_transition_length_m", 8.0)
        self.declare_parameter("parking_source_route_file", "")
        self.declare_parameter("parking_source_variant_id", "")
        self.declare_parameter("parking_source_start_s_m", 0.0)
        self.declare_parameter("parking_source_length_m", 0.0)
        self.declare_parameter("restart_forward_offset_m", 0.30)
        self.declare_parameter("restart_maximum_heading_error_deg", 60.0)
        self.declare_parameter("restart_heading_weight_m_per_rad", 2.0)
        self.declare_parameter("tracking_forward_search_m", 3.0)
        self.declare_parameter(
            "progress_checkpoint_file",
            "~/.local/state/hl_ku/route_progress.json",
        )
        self.declare_parameter("progress_checkpoint_max_age_sec", 3600.0)
        self.declare_parameter("progress_checkpoint_position_tolerance_m", 2.0)
        self.declare_parameter("progress_checkpoint_write_period_sec", 0.50)
        self._target_speed_override_mps = float(
            self.get_parameter("target_speed_override_mps").value
        )
        if (
            not math.isfinite(self._target_speed_override_mps)
            or self._target_speed_override_mps < 0.0
        ):
            raise ValueError("target_speed_override_mps must be finite and non-negative")
        route_path = Path(str(self.get_parameter("route_file").value))
        self._route_key = str(route_path.expanduser().resolve())
        self._route: Route | None = None
        if route_path.is_file():
            try:
                self._route = Route.load_csv(route_path)
            except (OSError, ValueError, KeyError) as error:
                self.get_logger().error(f"route load failed: {error}")
        else:
            self.get_logger().error(f"route file does not exist: {route_path}")
        self._parking_variant_id = str(
            self.get_parameter("parking_source_variant_id").value
        ).strip().lower()
        self._parking_route_family = self._build_parking_route_family()
        self._pose: PoseWithCovarianceStamped | None = None
        self._perception: PerceptionState | None = None
        self._mission: MissionStatus | None = None
        self._velocity_mps: float | None = None
        self._velocity_monotonic: float | None = None
        self._nearest_index = 0
        self._direction_runs = direction_runs(self._route) if self._route else ()
        self._direction_run_index = 0
        self._progress_initialized = False
        self._restart_context_active = False
        self._last_checkpoint_monotonic: float | None = None
        self._direction_change_started: float | None = None
        self._direction_change_pending = False
        self._command_pub = self.create_publisher(
            DriveCommand, "/planning/path_command", 10
        )
        self._s_pub = self.create_publisher(Float32, "/planning/route_s", 10)
        self._zone_pub = self.create_publisher(String, "/planning/route_zone", 10)
        self._event_pub = self.create_publisher(String, "/planning/route_event", 10)
        self._tracker_mode_pub = self.create_publisher(
            String, "/planning/tracker_mode", 10
        )
        self._pure_pursuit_pub = self.create_publisher(
            Float32, "/planning/pure_pursuit_steering_rad", 10
        )
        self._stanley_pub = self.create_publisher(
            Float32, "/planning/stanley_steering_rad", 10
        )
        self._pure_pursuit_weight_pub = self.create_publisher(
            Float32, "/planning/pure_pursuit_weight", 10
        )
        self._curvature_pub = self.create_publisher(
            Float32, "/planning/path_curvature_per_m", 10
        )
        self._cross_track_pub = self.create_publisher(
            Float32, "/planning/cross_track_error_m", 10
        )
        self._heading_error_pub = self.create_publisher(
            Float32, "/planning/heading_error_rad", 10
        )
        # The reference route is static and can contain thousands of poses.
        # Republishing it every second starves the 20 Hz steering topics when
        # Foxglove and RViz are connected.  Transient-local QoS retains one
        # copy for late subscribers without continuously flooding DDS.
        static_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._path_pub = self.create_publisher(
            PathMessage, "/planning/reference_path", static_qos
        )
        self._start_pose_pub = self.create_publisher(
            PoseStamped, "/planning/route_start_pose", static_qos
        )
        self._finish_pose_pub = self.create_publisher(
            PoseStamped, "/planning/route_finish_pose", static_qos
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/localization/gnss_pose",
            self._on_pose,
            20,
        )
        self.create_subscription(PerceptionState, "/perception/state", self._on_perception, 10)
        self.create_subscription(MissionStatus, "/mission/status", self._on_mission, 10)
        self.create_subscription(
            String,
            "/mission/parking_selection",
            self._on_parking_selection,
            10,
        )
        self.create_subscription(
            String,
            "/mission/end_lane_selection",
            self._on_end_lane_selection,
            10,
        )
        self.create_subscription(
            TwistStamped,
            "/localization/vehicle_velocity",
            self._on_velocity,
            20,
        )
        self.create_timer(0.05, self._update)
        self._publish_reference_path()

    def _on_pose(self, message: PoseWithCovarianceStamped) -> None:
        self._pose = message

    def _on_perception(self, message: PerceptionState) -> None:
        self._perception = message

    def _on_mission(self, message: MissionStatus) -> None:
        self._mission = message

    def _build_parking_route_family(self) -> ParkingRouteFamily | None:
        if not (
            bool(self.get_parameter("enable_parking_selection").value)
            or bool(self.get_parameter("enable_endpoint_selection").value)
        ):
            return None
        if self._route is None:
            self.get_logger().error("parking route switching disabled: active route is unavailable")
            return None
        try:
            family = ParkingRouteFamily.from_placed_route(
                self._route,
                str(self.get_parameter("parking_source_route_file").value),
                self._parking_variant_id,
                float(self.get_parameter("parking_source_start_s_m").value),
                float(self.get_parameter("parking_source_length_m").value),
                max(0.01, self._target_speed_override_mps),
            )
        except (OSError, ValueError, KeyError) as error:
            self.get_logger().error(f"parking route switching disabled: {error}")
            return None
        self.get_logger().info(
            f"parking route switching enabled from {self._parking_variant_id}"
        )
        return family

    def _on_parking_selection(self, message: String) -> None:
        family = self._parking_route_family
        mission = self._mission
        if family is None or mission is None or self._pose is None:
            return
        try:
            value = json.loads(message.data)
            requested = str(value["selected_variant_id"]).strip().lower()
            selection_mission = str(value["mission"]).strip().upper()
            locked = bool(value["locked"])
            current_p, current_t, current_f = parse_variant_id(
                self._parking_variant_id
            )
            requested_p, requested_t, requested_f = parse_variant_id(requested)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warning("invalid parking selection message")
            return
        if not locked or requested == self._parking_variant_id:
            return
        if selection_mission == "PERP_PARK":
            valid = (
                mission.state_name == "PERP_PARK"
                and requested_p == current_p
                and requested_f == current_f
            )
        elif selection_mission == "PARALLEL_PARK":
            valid = (
                mission.state_name == "PARALLEL_PARK"
                and requested_t == current_t
                and requested_f == current_f
            )
        else:
            valid = False
        if not valid or requested not in family.routes:
            return
        self._switch_parking_route(requested, selection_mission)

    def _on_end_lane_selection(self, message: String) -> None:
        family = self._parking_route_family
        mission = self._mission
        if family is None or mission is None or self._pose is None or self._route is None:
            return
        try:
            value = json.loads(message.data)
            active = bool(value["active"])
            locked = bool(value["locked"])
            selected_final = int(value["selected_final"])
            current_p, current_t, current_f = parse_variant_id(
                self._parking_variant_id
            )
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warning("invalid endpoint selection message")
            return
        if (
            not bool(self.get_parameter("enable_endpoint_selection").value)
            or not active
            or not locked
            or selected_final != 2
            or current_f == selected_final
            or mission.state_name != "END_LANE"
        ):
            return
        requested = variant_id(current_p, current_t, selected_final)
        target = family.routes.get(requested)
        if target is None:
            self.get_logger().error(f"endpoint route unavailable: {requested}")
            return
        try:
            current_route_s = self._route.waypoints[self._nearest_index].s_m
            route = blended_final_route(
                self._route,
                target,
                float(self.get_parameter("endpoint_transition_length_m").value),
                reference_transition_s_m=current_route_s,
            )
        except ValueError as error:
            self.get_logger().error(f"endpoint route blend rejected: {error}")
            return
        self._switch_route(route, requested, "END_LANE")

    def _switch_parking_route(self, requested: str, mission: str) -> None:
        assert self._parking_route_family is not None
        assert self._pose is not None
        route = self._parking_route_family.routes[requested]
        self._switch_route(route, requested, mission)

    def _switch_route(self, route: Route, requested: str, mission: str) -> None:
        assert self._pose is not None
        position = self._pose.pose.pose.position
        runs = direction_runs(route)
        eligible: list[tuple[float, int, int]] = []
        for run_index, (start, end, direction) in enumerate(runs):
            if direction <= 0 or not any(
                point.mission == mission for point in route.waypoints[start : end + 1]
            ):
                continue
            nearest = route.nearest_index_between(position.x, position.y, start, end)
            point = route.waypoints[nearest]
            distance_sq = (point.x_m - position.x) ** 2 + (point.y_m - position.y) ** 2
            eligible.append((distance_sq, run_index, nearest))
        if not eligible:
            self.get_logger().error(
                f"parking route switch rejected: no forward {mission} approach"
            )
            return
        _, run_index, nearest = min(eligible)
        self._route = route
        self._direction_runs = runs
        self._direction_run_index = run_index
        self._nearest_index = nearest
        self._direction_change_started = None
        self._direction_change_pending = False
        self._parking_variant_id = requested
        self._publish_reference_path()
        self.get_logger().info(
            f"route branch locked: {requested} ({mission})"
        )

    def _on_velocity(self, message: TwistStamped) -> None:
        self._velocity_mps = float(message.twist.linear.x)
        self._velocity_monotonic = time.monotonic()

    def _initialize_progress(self, x_m: float, y_m: float, yaw_rad: float) -> bool:
        """Acquire route progress once after a fresh node/full-stack start."""

        assert self._route is not None
        route_candidates: list[tuple[str, Route]] = [
            (self._parking_variant_id, self._route)
        ]
        family = self._parking_route_family
        if family is not None:
            route_candidates.extend(
                (route_id, route)
                for route_id, route in family.routes.items()
                if route_id != self._parking_variant_id
            )
        checkpoint = self._checkpoint_match(
            route_candidates,
            x_m,
            y_m,
            yaw_rad,
        )
        if checkpoint is not None:
            route_id, route, match = checkpoint
            return self._apply_initial_match(route_id, route, match, "checkpoint")
        matches = []
        for order, (route_id, route) in enumerate(route_candidates):
            match = match_route_ahead(
                route,
                x_m,
                y_m,
                yaw_rad,
                forward_offset_m=float(
                    self.get_parameter("restart_forward_offset_m").value
                ),
                maximum_heading_error_rad=math.radians(
                    float(
                        self.get_parameter(
                            "restart_maximum_heading_error_deg"
                        ).value
                    )
                ),
                heading_weight_m_per_rad=float(
                    self.get_parameter(
                        "restart_heading_weight_m_per_rad"
                    ).value
                ),
            )
            if match is not None:
                matches.append((match.score, order, route_id, route, match))
        if not matches:
            return False
        _score, _order, route_id, route, match = min(matches)
        return self._apply_initial_match(route_id, route, match, "global")

    def _apply_initial_match(
        self,
        route_id: str,
        route: Route,
        match: RouteProgressMatch,
        source: str,
    ) -> bool:
        route_changed = route is not self._route
        self._route = route
        self._direction_runs = direction_runs(route)
        self._direction_run_index = match.run_index
        self._nearest_index = match.waypoint_index
        self._direction_change_started = None
        self._direction_change_pending = False
        self._parking_variant_id = route_id
        self._progress_initialized = True
        self._restart_context_active = True
        if route_changed:
            self._publish_reference_path()
        waypoint = route.waypoints[self._nearest_index]
        self.get_logger().info(
            f"route progress acquired ({source}): "
            f"s={waypoint.s_m:.2f}m mission={waypoint.mission} "
            f"direction={waypoint.direction:+d} distance={match.distance_m:.2f}m "
            f"heading_error={math.degrees(match.heading_error_rad):.1f}deg "
            f"variant={route_id or 'single'}"
        )
        return True

    def _checkpoint_path(self) -> Path | None:
        value = str(self.get_parameter("progress_checkpoint_file").value).strip()
        return Path(value).expanduser() if value else None

    def _checkpoint_match(
        self,
        route_candidates: list[tuple[str, Route]],
        x_m: float,
        y_m: float,
        yaw_rad: float,
    ) -> tuple[str, Route, RouteProgressMatch] | None:
        path = self._checkpoint_path()
        if path is None or not path.is_file():
            return None
        try:
            value = json.loads(path.read_text(encoding="utf-8"))
            if value.get("route_key") != self._route_key:
                return None
            age = time.time() - float(value["written_at_unix_sec"])
            if age < 0.0 or age > float(
                self.get_parameter("progress_checkpoint_max_age_sec").value
            ):
                return None
            route_id = str(value.get("variant_id", ""))
            route = next(
                candidate_route
                for candidate_id, candidate_route in route_candidates
                if candidate_id == route_id
            )
            run_index = int(value["run_index"])
            match = match_saved_progress(
                route,
                x_m,
                y_m,
                yaw_rad,
                saved_route_s_m=float(value["route_s_m"]),
                saved_run_index=run_index,
                position_tolerance_m=float(
                    self.get_parameter(
                        "progress_checkpoint_position_tolerance_m"
                    ).value
                ),
                maximum_heading_error_rad=math.radians(
                    float(
                        self.get_parameter(
                            "restart_maximum_heading_error_deg"
                        ).value
                    )
                ),
                forward_search_m=float(
                    self.get_parameter("tracking_forward_search_m").value
                ),
            )
            return (route_id, route, match) if match is not None else None
        except (
            OSError,
            KeyError,
            TypeError,
            ValueError,
            StopIteration,
            IndexError,
            json.JSONDecodeError,
        ):
            return None

    def _write_progress_checkpoint(self) -> None:
        path = self._checkpoint_path()
        if path is None or self._route is None or not self._progress_initialized:
            return
        now = time.monotonic()
        period = max(
            0.10,
            float(
                self.get_parameter(
                    "progress_checkpoint_write_period_sec"
                ).value
            ),
        )
        if (
            self._last_checkpoint_monotonic is not None
            and now - self._last_checkpoint_monotonic < period
        ):
            return
        point = self._route.waypoints[self._nearest_index]
        document = {
            "route_key": self._route_key,
            "variant_id": self._parking_variant_id,
            "route_s_m": point.s_m,
            "run_index": self._direction_run_index,
            "direction": point.direction,
            "mission": point.mission,
            "written_at_unix_sec": time.time(),
        }
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            temporary = path.with_suffix(path.suffix + ".tmp")
            temporary.write_text(
                json.dumps(document, separators=(",", ":")),
                encoding="utf-8",
            )
            temporary.replace(path)
            self._last_checkpoint_monotonic = now
        except OSError as error:
            self.get_logger().warning(f"route progress checkpoint failed: {error}")

    def _publish_reference_path(self) -> None:
        if self._route is None:
            return
        path = PathMessage()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"
        for point in self._route.waypoints:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = point.x_m
            pose.pose.position.y = point.y_m
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self._path_pub.publish(path)
        self._start_pose_pub.publish(self._endpoint_pose(path.header, 0))
        self._finish_pose_pub.publish(
            self._endpoint_pose(path.header, len(self._route.waypoints) - 1)
        )

    def _endpoint_pose(self, header, index: int) -> PoseStamped:
        """Build a vehicle-heading arrow for an endpoint of the route."""
        assert self._route is not None
        points = self._route.waypoints
        point = points[index]
        if len(points) < 2:
            yaw = 0.0
        elif index == 0:
            neighbor = points[1]
            yaw = math.atan2(neighbor.y_m - point.y_m, neighbor.x_m - point.x_m)
        else:
            neighbor = points[index - 1]
            yaw = math.atan2(point.y_m - neighbor.y_m, point.x_m - neighbor.x_m)
        if point.direction < 0:
            yaw += math.pi
        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x = point.x_m
        pose.pose.position.y = point.y_m
        pose.pose.orientation.z = math.sin(0.5 * yaw)
        pose.pose.orientation.w = math.cos(0.5 * yaw)
        return pose

    def _update(self) -> None:
        if self._route is None or self._pose is None:
            return
        position = self._pose.pose.pose.position
        orientation = self._pose.pose.pose.orientation
        yaw = quaternion_yaw(orientation.x, orientation.y, orientation.z, orientation.w)
        if not self._progress_initialized:
            if not self._initialize_progress(position.x, position.y, yaw):
                return
        run_start, run_end, run_direction = self._direction_runs[
            self._direction_run_index
        ]
        self._nearest_index = continuous_nearest_index(
            self._route,
            position.x,
            position.y,
            self._nearest_index,
            run_start,
            run_end,
            float(self.get_parameter("tracking_forward_search_m").value),
        )
        direction_hold = False
        if self._direction_run_index < len(self._direction_runs) - 1:
            phase_end = self._route.waypoints[run_end]
            transition_distance = math.hypot(
                position.x - phase_end.x_m,
                position.y - phase_end.y_m,
            )
            # A sampled GNSS pose can move from just before to just beyond the
            # cusp without ever landing inside the distance circle.  Reaching
            # the final waypoint of the active motion run is equivalent and
            # must start the direction-change hold as well.
            phase_end_reached = self._nearest_index >= run_end
            if phase_end_reached or transition_distance <= float(
                self.get_parameter("direction_change_distance_m").value
            ):
                self._direction_change_pending = True
            if self._direction_change_pending:
                direction_hold = True
                now_monotonic = time.monotonic()
                velocity_fresh = (
                    self._velocity_monotonic is not None
                    and now_monotonic - self._velocity_monotonic <= 1.5
                )
                stopped = (
                    velocity_fresh
                    and self._velocity_mps is not None
                    and abs(self._velocity_mps)
                    <= float(
                        self.get_parameter(
                            "direction_change_stopped_speed_mps"
                        ).value
                    )
                )
                if stopped:
                    if self._direction_change_started is None:
                        self._direction_change_started = now_monotonic
                    held = now_monotonic - self._direction_change_started
                    if held >= float(
                        self.get_parameter("direction_change_hold_sec").value
                    ):
                        self._direction_run_index += 1
                        self._direction_change_started = None
                        self._direction_change_pending = False
                        direction_hold = False
                        run_start, run_end, run_direction = self._direction_runs[
                            self._direction_run_index
                        ]
                        self._nearest_index = run_start
                else:
                    self._direction_change_started = None
        nearest = self._route.waypoints[self._nearest_index]
        metadata = self._route.metadata_waypoint(
            self._nearest_index,
            position.x,
            position.y,
            float(self.get_parameter("finish_tolerance_m").value),
        )
        speed = effective_target_speed(
            metadata.target_speed_mps,
            metadata.direction,
            self._target_speed_override_mps,
        )
        lookahead = float(self.get_parameter("lookahead_base_m").value) + abs(speed) * float(
            self.get_parameter("lookahead_speed_gain_sec").value
        )
        target_index = min(
            run_end,
            self._route.lookahead_index(self._nearest_index, lookahead),
        )
        target = self._route.waypoints[target_index]
        lateral_offset = self._mission.lateral_offset_m if self._mission else 0.0
        segment_yaw = yaw
        if target.index > 0:
            previous = self._route.waypoints[target.index - 1]
            segment_yaw = math.atan2(target.y_m - previous.y_m, target.x_m - previous.x_m)
        target_x = target.x_m - math.sin(segment_yaw) * lateral_offset
        target_y = target.y_m + math.cos(segment_yaw) * lateral_offset
        maximum = float(self.get_parameter("maximum_steering_rad").value)
        pure_pursuit = pure_pursuit_steering(
            position.x,
            position.y,
            yaw,
            target_x,
            target_y,
            float(self.get_parameter("wheelbase_m").value),
            maximum,
            target.direction,
        )
        steering = 0.0
        stanley = pure_pursuit
        cross_track_error = 0.0
        heading_error = 0.0
        pure_pursuit_weight = 0.0
        path_curvature = self._route.curvature_ahead(
            self._nearest_index,
            float(self.get_parameter("curvature_preview_m").value),
            int(self.get_parameter("curvature_segment_count").value),
        )
        tracker_mode = "STOP"
        if target.direction in (-1, 1):
            reverse = target.direction < 0
            control_offset = float(
                self.get_parameter(
                    "reverse_stanley_control_offset_m"
                    if reverse
                    else "stanley_control_offset_m"
                ).value
            )
            motion_yaw = (
                yaw if target.direction > 0 else normalize_angle(yaw + math.pi)
            )
            control_x = position.x + control_offset * math.cos(motion_yaw)
            control_y = position.y + control_offset * math.sin(motion_yaw)
            projection_start, projection_end = continuous_search_bounds(
                self._route,
                self._nearest_index,
                run_start,
                run_end,
                float(self.get_parameter("tracking_forward_search_m").value),
            )
            _, projection_s, reference_x, reference_y = (
                self._route.nearest_projection_between(
                    control_x,
                    control_y,
                    projection_start,
                    projection_end,
                )
            )
            stanley_segment_yaw = bounded_route_heading(
                self._route,
                projection_s,
                run_start,
                run_end,
                float(
                    self.get_parameter(
                        "reverse_stanley_heading_baseline_m"
                        if reverse
                        else "stanley_heading_baseline_m"
                    ).value
                ),
            )
            offset_x = -math.sin(stanley_segment_yaw) * lateral_offset
            offset_y = math.cos(stanley_segment_yaw) * lateral_offset
            reference_x += offset_x
            reference_y += offset_y
            stanley_speed_mps = speed
            if reverse and bool(
                self.get_parameter("reverse_use_measured_speed").value
            ):
                velocity_fresh = (
                    self._velocity_monotonic is not None
                    and time.monotonic() - self._velocity_monotonic <= 1.5
                    and self._velocity_mps is not None
                    and math.isfinite(self._velocity_mps)
                )
                if velocity_fresh:
                    # Stanley's speed term should reflect how quickly the car
                    # is actually moving.  Using the much smaller reverse
                    # target speed overreacts to a growing cross-track error.
                    stanley_speed_mps = -abs(self._velocity_mps)
            stanley_terms = stanley_steering(
                position.x,
                position.y,
                motion_yaw,
                reference_x,
                reference_y,
                reference_x + math.cos(stanley_segment_yaw),
                reference_y + math.sin(stanley_segment_yaw),
                stanley_speed_mps,
                float(
                    self.get_parameter(
                        "reverse_stanley_gain" if reverse else "stanley_gain"
                    ).value
                ),
                float(
                    self.get_parameter(
                        "reverse_stanley_softening_mps"
                        if reverse
                        else "stanley_softening_mps"
                    ).value
                ),
                control_offset,
                maximum,
                float(
                    self.get_parameter(
                        "reverse_stanley_heading_gain"
                        if reverse
                        else "stanley_heading_gain"
                    ).value
                )
                if reverse
                else 1.0,
            )
            stanley = (
                stanley_terms.steering_rad
                if target.direction > 0
                else -stanley_terms.steering_rad
            )
            cross_track_error = stanley_terms.cross_track_error_m
            heading_error = stanley_terms.heading_error_rad
        steering, pure_pursuit_weight, tracker_mode = selected_path_steering(
            target.direction,
            stanley,
            pure_pursuit,
            maximum,
        )
        if direction_hold:
            next_direction = self._direction_runs[
                self._direction_run_index + 1
            ][2]
            speed = 0.0
            steering = 0.0
            tracker_mode = (
                f"DIRECTION_HOLD_{run_direction:+d}_TO_{next_direction:+d}"
            )
        perception = self._perception
        if (
            bool(self.get_parameter("enable_lane_correction").value)
            and perception
            and perception.lane_confidence >= float(
                self.get_parameter("minimum_lane_confidence").value
            )
        ):
            correction = -float(self.get_parameter("lane_offset_gain").value) * perception.lane_offset_m
            correction -= float(self.get_parameter("lane_heading_gain").value) * perception.lane_heading_error_rad
            steering = clamp(steering + correction, -maximum, maximum)
        if not bool(self.get_parameter("route_calibrated").value):
            speed = 0.0
        command = DriveCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = "base_link"
        command.speed_mps = speed
        command.steering_angle_rad = steering
        self._command_pub.publish(command)
        s_message = Float32()
        s_message.data = float(nearest.s_m)
        self._s_pub.publish(s_message)
        zone_message = String()
        zone_message.data = metadata.mission
        self._zone_pub.publish(zone_message)
        event = self._route.active_event_waypoint(self._nearest_index)
        event_message = String()
        event_message.data = json.dumps(
            {
                "zone": event.fsm_zone or metadata.mission,
                "current_zone": metadata.mission,
                "event_id": event.fsm_event_ids,
                "action": event.fsm_actions,
                "condition": event.fsm_conditions,
                "hold_sec": event.fsm_hold_sec,
                "has_fsm_metadata": self._route.has_fsm_metadata,
                "relocalized": self._restart_context_active,
                "direction": nearest.direction,
                "variant_id": self._parking_variant_id,
            },
            separators=(",", ":"),
        )
        self._event_pub.publish(event_message)
        self._write_progress_checkpoint()
        if (
            self._mission is not None
            and self._mission.state
            not in (MissionStatus.STATE_INIT, MissionStatus.STATE_READY)
        ):
            self._restart_context_active = False
        mode_message = String()
        mode_message.data = tracker_mode
        self._tracker_mode_pub.publish(mode_message)
        for publisher, value in (
            (self._pure_pursuit_pub, pure_pursuit),
            (self._stanley_pub, stanley),
            (self._pure_pursuit_weight_pub, pure_pursuit_weight),
            (self._curvature_pub, path_curvature),
            (self._cross_track_pub, cross_track_error),
            (self._heading_error_pub, heading_error),
        ):
            diagnostic = Float32()
            diagnostic.data = float(value)
            publisher.publish(diagnostic)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathTrackerNode()
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
