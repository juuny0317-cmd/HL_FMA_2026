"""Turn recorded GNSS and route CSVs into a compact Foxglove scene."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import re

import rclpy
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path as PathMessage
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA, Float32, String
from visualization_msgs.msg import Marker, MarkerArray

from hl_ku_core.route import Route
from hl_ku_interfaces.msg import MissionStatus


MISSION_COLORS = {
    "NORMAL": (0.10, 0.85, 0.55, 0.95),
    "HILL": (0.95, 0.70, 0.15, 1.00),
    "S_OBSTACLE": (0.90, 0.35, 0.25, 1.00),
    "TRAFFIC": (0.75, 0.35, 0.95, 1.00),
    "PERP_PARK": (0.20, 0.65, 1.00, 1.00),
    "DUMMY": (1.00, 0.25, 0.55, 1.00),
    "PARALLEL_PARK": (0.10, 0.80, 1.00, 1.00),
    "END_LANE": (1.00, 0.50, 0.10, 1.00),
    "FINISH": (1.00, 0.15, 0.15, 1.00),
}

# Arc-length boundaries measured on the representative course 07 FSM route.
# Campus slices retain scale 1.0, so local progress can be mapped back to the
# original course by adding the slice's source start s.
COURSE_07_FSM_SECTIONS = (
    (0.0, 21.8, "ROUTE", "출발 일반구간"),
    (21.8, 57.0, "HILL", "언덕"),
    (57.0, 88.6, "ROUTE", "굴절 접근"),
    (88.6, 119.7, "BEND", "굴절"),
    (119.7, 140.6, "ROUTE", "신호1 접근"),
    (140.6, 142.9, "TRAFFIC", "신호1"),
    (142.9, 191.4, "ROUTE", "S자 접근"),
    (191.4, 229.2, "S_CURVE", "S자"),
    (229.2, 260.0, "ROUTE", "신호2 접근"),
    (260.0, 263.3, "TRAFFIC", "신호2"),
    (263.3, 284.9, "ROUTE", "T주차 접근"),
    (284.9, 329.4, "T_PARK", "T자 주차"),
    (329.4, 407.4, "ROUTE", "중간 연결"),
    (407.4, 413.5, "TRAFFIC", "신호3"),
    (413.5, 507.5, "ROUTE", "돌발 접근"),
    (507.5, 572.3, "DUMMY", "돌발"),
    (572.3, 607.7, "ROUTE", "평행주차 접근"),
    (607.7, 637.5, "PARALLEL_PARK", "평행주차"),
    (637.5, 656.4, "ROUTE", "종료차선 접근"),
    (656.4, 679.3, "END_LANE", "종료차선"),
    (679.3, 690.5, "FINISH", "결승"),
)


def course_07_section_at_s(source_s_m: float) -> dict:
    """Return the authored course section nearest to a source arc length."""
    value = max(0.0, float(source_s_m))
    for index, (start, end, zone, label) in enumerate(
        COURSE_07_FSM_SECTIONS, start=1
    ):
        if value < end or index == len(COURSE_07_FSM_SECTIONS):
            return {
                "index": index,
                "start_s_m": start,
                "end_s_m": end,
                "zone": zone,
                "label": label,
            }
    raise AssertionError("course section table is empty")


def source_progress_labels(
    source_s_m: float,
    source_fsm_label: str,
    source_segment_name: str,
) -> tuple[dict, str, str]:
    """Resolve the FSM shown by Foxglove for a source-course position.

    ``ALL`` describes the scope of a full-course scenario; it is not an FSM
    state.  In that case Foxglove must follow the authored section at the
    current source arc length.  A concrete label such as ``T_PARK`` remains a
    valid override for an isolated test slice.
    """

    section = course_07_section_at_s(source_s_m)
    override = source_fsm_label.strip().upper()
    if override and override != "ALL":
        return section, override, source_segment_name
    return section, str(section["zone"]), str(section["label"])


def color_rgba(values: tuple[float, float, float, float]) -> ColorRGBA:
    color = ColorRGBA()
    color.r, color.g, color.b, color.a = values
    return color


def signed_cross_track_error(
    route: Route, segment_index: int, x_m: float, y_m: float
) -> float:
    """Return left-positive cross-track error for a projected route segment."""
    first = route.waypoints[segment_index]
    second = route.waypoints[min(segment_index + 1, len(route.waypoints) - 1)]
    dx = second.x_m - first.x_m
    dy = second.y_m - first.y_m
    length = math.hypot(dx, dy)
    if length <= 1.0e-9:
        return 0.0
    return (dx * (y_m - first.y_m) - dy * (x_m - first.x_m)) / length


def load_variant_mission_metadata(
    route_path: str | Path, slot: int, name: str
) -> dict:
    """Read the waypoint-authored FSM boundaries without changing route behavior."""
    segments: list[dict] = []
    events: list[dict] = []
    point_count = 0
    with Path(route_path).open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        for index, row in enumerate(reader):
            point_count += 1
            mission = (row.get("mission") or "NORMAL").strip().upper()
            zone = (row.get("fsm_zone") or mission or "ROUTE").strip().upper()
            direction = int(float(row.get("direction") or 1))
            s_m = float(row.get("s_m") or 0.0)
            event_ids = [
                token.strip()
                for token in re.split(r"[|;]", row.get("fsm_event_ids") or "")
                if token.strip()
            ]
            x_m = float(row["x_m"])
            y_m = float(row["y_m"])
            events.extend(
                {
                    "event_id": event_id,
                    "index": index,
                    "s_m": s_m,
                    "x_m": x_m,
                    "y_m": y_m,
                }
                for event_id in event_ids
            )
            key = (zone, mission, direction)
            if not segments or segments[-1]["_key"] != key:
                segments.append(
                    {
                        "_key": key,
                        "zone": zone,
                        "mission": mission,
                        "direction": direction,
                        "start_index": index,
                        "end_index": index,
                        "start_s_m": s_m,
                        "end_s_m": s_m,
                        "event_ids": [],
                    }
                )
            segment = segments[-1]
            segment["end_index"] = index
            segment["end_s_m"] = s_m
            for event_id in event_ids:
                if event_id not in segment["event_ids"]:
                    segment["event_ids"].append(event_id)

    for segment in segments:
        del segment["_key"]
    return {
        "slot": slot,
        "name": name,
        "file": Path(route_path).name,
        "point_count": point_count,
        "segment_count": len(segments),
        "segments": segments,
        "events": events,
    }


def mission_segment_at_index(metadata: dict | None, waypoint_index: int) -> dict | None:
    """Return the authored FSM segment containing a zero-based waypoint index."""
    if metadata is None:
        return None
    for segment in metadata.get("segments", []):
        if segment["start_index"] <= waypoint_index <= segment["end_index"]:
            return segment
    return None


class ReplayVisualizerNode(Node):
    def __init__(self) -> None:
        super().__init__("hl_ku_foxglove_visualizer")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("recorded_route_file", "")
        self.declare_parameter("active_route_file", "")
        self.declare_parameter("active_variant_name", "course_07_vehicle")
        self.declare_parameter("source_route_file", "")
        self.declare_parameter("source_route_variant_id", "")
        self.declare_parameter("source_start_s_m", 0.0)
        self.declare_parameter("source_length_m", 0.0)
        self.declare_parameter("source_segment_name", "")
        self.declare_parameter("source_fsm_label", "")
        self.declare_parameter(
            "source_mode", "SAFE_REPLAY: MANUAL BAG / actuator topics excluded"
        )
        self.declare_parameter("track_minimum_spacing_m", 0.15)
        self.declare_parameter("track_maximum_points", 6000)
        for slot in range(1, 9):
            self.declare_parameter(f"variant_{slot:02d}_file", "")
            self.declare_parameter(f"variant_{slot:02d}_name", f"variant_{slot:02d}")

        self._frame_id = str(self.get_parameter("frame_id").value)
        self._track_minimum_spacing_m = max(
            0.01, float(self.get_parameter("track_minimum_spacing_m").value)
        )
        self._track_maximum_points = max(
            100, int(self.get_parameter("track_maximum_points").value)
        )
        self._active_variant_name = str(
            self.get_parameter("active_variant_name").value
        )
        self._source_route_variant_id = str(
            self.get_parameter("source_route_variant_id").value
        ).strip()
        self._source_mode = str(self.get_parameter("source_mode").value)
        self._source_start_s_m = max(
            0.0, float(self.get_parameter("source_start_s_m").value)
        )
        self._source_length_m = max(
            0.0, float(self.get_parameter("source_length_m").value)
        )
        self._source_segment_name = str(
            self.get_parameter("source_segment_name").value
        )
        self._source_fsm_label = str(
            self.get_parameter("source_fsm_label").value
        ).strip().upper()

        static_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._recorded_path_pub = self.create_publisher(
            PathMessage, "/visualization/recorded_route", static_qos
        )
        self._active_path_pub = self.create_publisher(
            PathMessage, "/visualization/active_route", static_qos
        )
        self._source_course_path_pub = self.create_publisher(
            PathMessage, "/visualization/source_course_route", static_qos
        )
        self._route_markers_pub = self.create_publisher(
            MarkerArray, "/visualization/route_mission_markers", static_qos
        )
        self._variant_publishers = [
            self.create_publisher(
                PathMessage,
                f"/visualization/route_variant_{slot:02d}",
                static_qos,
            )
            for slot in range(1, 9)
        ]
        self._variant_metadata_pub = self.create_publisher(
            String, "/visualization/route_variant_metadata", static_qos
        )

        self._current_pose_pub = self.create_publisher(
            PoseStamped, "/visualization/current_pose", 10
        )
        self._actual_path_pub = self.create_publisher(
            PathMessage, "/visualization/actual_path", 2
        )
        self._current_markers_pub = self.create_publisher(
            MarkerArray, "/visualization/current_status_markers", 10
        )
        self._route_s_pub = self.create_publisher(
            Float32, "/visualization/replay_route_s", 10
        )
        self._source_progress_pub = self.create_publisher(
            String, "/visualization/source_course_progress", 10
        )
        self._cross_track_pub = self.create_publisher(
            Float32, "/visualization/cross_track_error_m", 10
        )
        self._target_speed_pub = self.create_publisher(
            Float32, "/visualization/target_speed_mps", 10
        )
        self._zone_pub = self.create_publisher(
            String, "/visualization/replay_zone", 10
        )
        self._mission_pub = self.create_publisher(
            String, "/visualization/display_mission", 10
        )
        self._variant_pub = self.create_publisher(
            String, "/visualization/active_variant", static_qos
        )
        self._mode_pub = self.create_publisher(
            String, "/visualization/replay_mode", static_qos
        )

        self._recorded_route = self._load_route_parameter("recorded_route_file")
        self._active_route = self._load_route_parameter("active_route_file")
        self._source_course_route = self._load_route_parameter(
            "source_route_file",
            optional=True,
            variant_id=self._source_route_variant_id or None,
        )
        self._variant_routes: list[tuple[str, Route] | None] = []
        self._variant_metadata: dict = {"schema_version": 1, "variants": []}
        self._variant_metadata_by_slot: dict[int, dict] = {}
        for slot in range(1, 9):
            parameter_name = f"variant_{slot:02d}_file"
            route = self._load_route_parameter(parameter_name, optional=True)
            name = str(self.get_parameter(f"variant_{slot:02d}_name").value)
            self._variant_routes.append((name, route) if route is not None else None)
            if route is not None:
                route_path = Path(str(self.get_parameter(parameter_name).value))
                try:
                    metadata = load_variant_mission_metadata(route_path, slot, name)
                    self._variant_metadata["variants"].append(metadata)
                    self._variant_metadata_by_slot[slot] = metadata
                    self.get_logger().info(
                        f"loaded variant {slot:02d} waypoint FSM boundaries: "
                        f"{metadata['segment_count']} segments"
                    )
                except (OSError, KeyError, TypeError, ValueError) as error:
                    self.get_logger().error(
                        f"failed to load waypoint FSM metadata from {route_path}: {error}"
                    )

        if self._active_route is None:
            raise RuntimeError("active_route_file must point to a valid route CSV")

        self._nearest_index = 0
        self._variant_nearest_indices = [0] * 8
        self._mission_status: MissionStatus | None = None
        self._track = PathMessage()
        self._track.header.frame_id = self._frame_id
        self._last_track_xy: tuple[float, float] | None = None
        self._last_pose_stamp_sec: float | None = None

        self.create_subscription(
            PoseWithCovarianceStamped,
            "/localization/gnss_pose",
            self._on_pose,
            20,
        )
        self.create_subscription(
            MissionStatus, "/mission/status", self._on_mission_status, 10
        )
        # Static publishers use transient-local QoS, so a single publication
        # remains available to Foxglove clients that connect later.  Rebuilding
        # every 6,995-point route and every route variant once per second can
        # delay live mission and steering topics.
        self.create_timer(1.0, self._publish_track)
        self._publish_static_scene()
        self.get_logger().info(
            f"Foxglove visualization ready ({self._source_mode}); route mission is "
            "inferred until /mission/status is available"
        )

    def _load_route_parameter(
        self,
        parameter_name: str,
        optional: bool = False,
        variant_id: str | None = None,
    ) -> Route | None:
        route_path = Path(str(self.get_parameter(parameter_name).value))
        if not str(route_path) or str(route_path) == ".":
            if not optional:
                self.get_logger().warning(f"{parameter_name} is empty")
            return None
        if not route_path.is_file():
            message = f"{parameter_name} does not exist: {route_path}"
            if optional:
                self.get_logger().info(message)
            else:
                self.get_logger().error(message)
            return None
        try:
            route = Route.load_csv(route_path, variant_id=variant_id)
        except (OSError, KeyError, ValueError) as error:
            self.get_logger().error(f"failed to load {route_path}: {error}")
            return None
        self.get_logger().info(
            f"loaded {parameter_name}: {route_path} ({len(route.waypoints)} points)"
        )
        return route

    def _on_mission_status(self, message: MissionStatus) -> None:
        self._mission_status = message

    def _on_pose(self, message: PoseWithCovarianceStamped) -> None:
        assert self._active_route is not None
        stamp_sec = float(message.header.stamp.sec) + float(
            message.header.stamp.nanosec
        ) * 1.0e-9
        if (
            self._last_pose_stamp_sec is not None
            and stamp_sec < self._last_pose_stamp_sec - 0.5
        ):
            self._nearest_index = 0
            self._variant_nearest_indices = [0] * 8
            self._mission_status = None
            self._track = PathMessage()
            self._track.header.frame_id = self._frame_id
            self._last_track_xy = None
            self.get_logger().info("backward rosbag seek detected; live track reset")
        self._last_pose_stamp_sec = stamp_sec
        position = message.pose.pose.position
        projection = self._active_route.nearest_projection(
            position.x, position.y, self._nearest_index, 120
        )
        segment_index, route_s_m, nearest_x, nearest_y = projection
        distance = math.hypot(position.x - nearest_x, position.y - nearest_y)
        if distance > 5.0:
            segment_index, route_s_m, nearest_x, nearest_y = (
                self._active_route.nearest_projection(
                    position.x,
                    position.y,
                    0,
                    len(self._active_route.waypoints),
                )
            )
        self._nearest_index = segment_index
        metadata_index = min(segment_index, len(self._active_route.waypoints) - 1)
        waypoint = self._active_route.waypoints[metadata_index]
        if route_s_m >= self._active_route.waypoints[-1].s_m - 0.25:
            waypoint = self._active_route.waypoints[-1]
        signed_error = signed_cross_track_error(
            self._active_route, segment_index, position.x, position.y
        )

        pose = PoseStamped()
        pose.header = message.header
        pose.header.frame_id = self._frame_id
        pose.pose = message.pose.pose
        self._current_pose_pub.publish(pose)
        self._append_track_pose(pose)

        route_s = Float32()
        route_s.data = float(route_s_m)
        self._route_s_pub.publish(route_s)
        source_progress_text = ""
        if self._source_course_route is not None:
            source_total_m = self._source_course_route.waypoints[-1].s_m
            source_end_m = min(
                source_total_m,
                self._source_start_s_m
                + (
                    self._source_length_m
                    if self._source_length_m > 0.0
                    else self._active_route.waypoints[-1].s_m
                ),
            )
            source_s_m = min(
                source_end_m, self._source_start_s_m + float(route_s_m)
            )
            section, expected_zone, expected_label = source_progress_labels(
                source_s_m,
                self._source_fsm_label,
                self._source_segment_name,
            )
            source_progress = {
                "course": "course_07",
                "segment_name": self._source_segment_name,
                "local_s_m": float(route_s_m),
                "local_total_m": float(self._active_route.waypoints[-1].s_m),
                "source_s_m": source_s_m,
                "source_start_s_m": self._source_start_s_m,
                "source_end_s_m": source_end_m,
                "source_total_m": source_total_m,
                "section_index": section["index"],
                "section_start_s_m": section["start_s_m"],
                "section_end_s_m": section["end_s_m"],
                "expected_zone": expected_zone,
                "expected_label": expected_label,
            }
            progress = String()
            progress.data = json.dumps(
                source_progress, ensure_ascii=False, separators=(",", ":")
            )
            self._source_progress_pub.publish(progress)
            source_progress_text = (
                f"\n원본 s={source_s_m:.1f}/{source_total_m:.1f} m | "
                f"예상 {expected_zone}"
            )
        cross_track = Float32()
        cross_track.data = float(signed_error)
        self._cross_track_pub.publish(cross_track)
        target_speed = Float32()
        target_speed.data = float(waypoint.target_speed_mps)
        self._target_speed_pub.publish(target_speed)

        inferred = self._infer_waypoint_mission(position.x, position.y)
        zone = String()
        if self._mission_status is not None:
            zone.data = self._mission_status.zone or self._mission_status.state_name
        elif inferred is not None:
            zone.data = str(inferred["zone"])
        else:
            zone.data = waypoint.mission
        self._zone_pub.publish(zone)

        mission = String()
        if self._mission_status is not None:
            state_name = self._mission_status.state_name or str(
                self._mission_status.state
            )
            mission.data = f"ACTUAL: {state_name} / {self._mission_status.zone}"
        elif inferred is not None:
            mission.data = (
                f"INFERRED_FROM_WAYPOINT: {inferred['zone']} / "
                f"{inferred['mission']} · V{inferred['slot']:02d} · "
                f"s={inferred['s_m']:.1f}m"
            )
        else:
            mission.data = f"INFERRED_FROM_ROUTE: {waypoint.mission}"
        self._mission_pub.publish(mission)
        self._publish_current_markers(
            pose,
            nearest_x,
            nearest_y,
            route_s_m,
            signed_error,
            mission.data + source_progress_text,
        )

    def _infer_waypoint_mission(self, x_m: float, y_m: float) -> dict | None:
        """Find the closest progressing route variant and its authored FSM zone."""
        best: dict | None = None
        for list_index, route_entry in enumerate(self._variant_routes):
            if route_entry is None:
                continue
            _, route = route_entry
            hint = self._variant_nearest_indices[list_index]
            segment_index, route_s_m, nearest_x, nearest_y = route.nearest_projection(
                x_m, y_m, hint, 500
            )
            distance = math.hypot(x_m - nearest_x, y_m - nearest_y)
            if distance > 5.0:
                segment_index, route_s_m, nearest_x, nearest_y = (
                    route.nearest_projection(x_m, y_m, 0, len(route.waypoints))
                )
                distance = math.hypot(x_m - nearest_x, y_m - nearest_y)
            self._variant_nearest_indices[list_index] = segment_index

            waypoint_index = min(segment_index, len(route.waypoints) - 1)
            if route_s_m >= route.waypoints[-1].s_m - 0.25:
                waypoint_index = len(route.waypoints) - 1
            slot = list_index + 1
            authored_segment = mission_segment_at_index(
                self._variant_metadata_by_slot.get(slot), waypoint_index
            )
            if authored_segment is None:
                continue
            candidate = {
                "slot": slot,
                "waypoint_index": waypoint_index,
                "s_m": route_s_m,
                "distance_m": distance,
                "zone": authored_segment["zone"],
                "mission": authored_segment["mission"],
            }
            if best is None or distance < best["distance_m"]:
                best = candidate
        return best

    def _append_track_pose(self, pose: PoseStamped) -> None:
        position = pose.pose.position
        if self._last_track_xy is not None:
            distance = math.hypot(
                position.x - self._last_track_xy[0],
                position.y - self._last_track_xy[1],
            )
            if distance < self._track_minimum_spacing_m:
                return
        track_pose = PoseStamped()
        track_pose.header = pose.header
        track_pose.pose = pose.pose
        self._track.poses.append(track_pose)
        if len(self._track.poses) > self._track_maximum_points:
            self._track.poses = self._track.poses[-self._track_maximum_points :]
        self._last_track_xy = (position.x, position.y)

    def _publish_track(self) -> None:
        if not self._track.poses:
            return
        self._track.header.stamp = self.get_clock().now().to_msg()
        self._actual_path_pub.publish(self._track)

    def _publish_static_scene(self) -> None:
        stamp = self.get_clock().now().to_msg()
        if self._recorded_route is not None:
            self._recorded_path_pub.publish(self._route_path(self._recorded_route, stamp))
        assert self._active_route is not None
        self._active_path_pub.publish(self._route_path(self._active_route, stamp))
        if self._source_course_route is not None:
            self._source_course_path_pub.publish(
                self._route_path(self._source_course_route, stamp)
            )
        self._route_markers_pub.publish(self._route_markers(self._active_route, stamp))
        for route_entry, publisher in zip(
            self._variant_routes, self._variant_publishers
        ):
            if route_entry is None:
                continue
            _, route = route_entry
            publisher.publish(self._route_path(route, stamp))

        metadata = String()
        metadata.data = json.dumps(
            self._variant_metadata, ensure_ascii=False, separators=(",", ":")
        )
        self._variant_metadata_pub.publish(metadata)

        variant = String()
        variant.data = self._active_variant_name
        self._variant_pub.publish(variant)
        mode = String()
        mode.data = self._source_mode
        self._mode_pub.publish(mode)

    def _route_path(self, route: Route, stamp) -> PathMessage:
        path = PathMessage()
        path.header.stamp = stamp
        path.header.frame_id = self._frame_id
        for waypoint in route.waypoints:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = waypoint.x_m
            pose.pose.position.y = waypoint.y_m
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    def _route_markers(self, route: Route, stamp) -> MarkerArray:
        markers = MarkerArray()
        groups: list[list] = []
        current: list = []
        current_key: tuple[str, int] | None = None
        for waypoint in route.waypoints:
            key = (waypoint.mission, waypoint.direction)
            if current and key != current_key:
                groups.append(current)
                current = [current[-1]]
            current.append(waypoint)
            current_key = key
        if current:
            groups.append(current)

        marker_id = 0
        for group in groups:
            if len(group) < 2:
                continue
            mission = group[-1].mission
            line = Marker()
            line.header.stamp = stamp
            line.header.frame_id = self._frame_id
            line.ns = "mission_segments"
            line.id = marker_id
            marker_id += 1
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.20
            line.color = color_rgba(MISSION_COLORS.get(mission, MISSION_COLORS["NORMAL"]))
            line.points = [Point(x=point.x_m, y=point.y_m, z=0.04) for point in group]
            markers.markers.append(line)

            if mission != "NORMAL" or group is groups[0]:
                label_point = group[0]
                label = Marker()
                label.header = line.header
                label.ns = "mission_labels"
                label.id = marker_id
                marker_id += 1
                label.type = Marker.TEXT_VIEW_FACING
                label.action = Marker.ADD
                label.pose.position.x = label_point.x_m
                label.pose.position.y = label_point.y_m
                label.pose.position.z = 1.2
                label.pose.orientation.w = 1.0
                label.scale.z = 0.75
                label.color = line.color
                prefix = "START" if group is groups[0] else mission
                direction = "FWD" if group[-1].direction > 0 else "REV"
                label.text = f"{prefix} | s={label_point.s_m:.1f} m | {direction}"
                markers.markers.append(label)
        return markers

    def _publish_current_markers(
        self,
        pose: PoseStamped,
        nearest_x: float,
        nearest_y: float,
        route_s_m: float,
        signed_error_m: float,
        mission_text: str,
    ) -> None:
        markers = MarkerArray()

        vehicle = Marker()
        vehicle.header = pose.header
        vehicle.ns = "vehicle"
        vehicle.id = 0
        vehicle.type = Marker.ARROW
        vehicle.action = Marker.ADD
        vehicle.pose = pose.pose
        vehicle.scale.x = 1.50
        vehicle.scale.y = 0.55
        vehicle.scale.z = 0.40
        vehicle.color = color_rgba((0.05, 0.85, 1.00, 1.00))
        markers.markers.append(vehicle)

        error_line = Marker()
        error_line.header = pose.header
        error_line.ns = "cross_track"
        error_line.id = 1
        error_line.type = Marker.LINE_LIST
        error_line.action = Marker.ADD
        error_line.pose.orientation.w = 1.0
        error_line.scale.x = 0.08
        error_line.color = color_rgba((1.00, 0.25, 0.25, 0.95))
        error_line.points = [
            Point(x=pose.pose.position.x, y=pose.pose.position.y, z=0.10),
            Point(x=nearest_x, y=nearest_y, z=0.10),
        ]
        markers.markers.append(error_line)

        nearest = Marker()
        nearest.header = pose.header
        nearest.ns = "nearest_route_point"
        nearest.id = 2
        nearest.type = Marker.SPHERE
        nearest.action = Marker.ADD
        nearest.pose.position.x = nearest_x
        nearest.pose.position.y = nearest_y
        nearest.pose.position.z = 0.10
        nearest.pose.orientation.w = 1.0
        nearest.scale.x = nearest.scale.y = nearest.scale.z = 0.30
        nearest.color = color_rgba((1.00, 0.90, 0.10, 1.00))
        markers.markers.append(nearest)

        label = Marker()
        label.header = pose.header
        label.ns = "status"
        label.id = 3
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = pose.pose.position.x
        label.pose.position.y = pose.pose.position.y
        label.pose.position.z = 1.8
        label.pose.orientation.w = 1.0
        label.scale.z = 0.65
        label.color = color_rgba((1.00, 1.00, 1.00, 1.00))
        label.text = (
            f"{self._active_variant_name}\n"
            f"s={route_s_m:.1f} m | CTE={signed_error_m:+.2f} m\n"
            f"{mission_text}"
        )
        markers.markers.append(label)
        self._current_markers_pub.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReplayVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
