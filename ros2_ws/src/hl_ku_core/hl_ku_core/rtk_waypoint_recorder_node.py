"""Interactive RTK-fixed waypoint recorder: Enter captures, U undoes, Q exits."""

from __future__ import annotations

import csv
import math
import select
import sys
import termios
import time
import tty
from collections import deque
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path as PathMessage
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from hl_ku_interfaces.msg import GnssStatus

from .geometry import EnuProjector
from .waypoint_capture import (
    CapturedWaypoint,
    FixedPositionSample,
    average_samples,
    build_route_points,
    sample_quality_issue,
    waypoint_spacing_m,
)


FIX_NAMES = {
    0: "NONE",
    1: "SINGLE",
    2: "DGPS",
    4: "RTK_FIXED",
    5: "RTK_FLOAT",
}


class RtkWaypointRecorderNode(Node):
    def __init__(self) -> None:
        super().__init__("rtk_waypoint_recorder")
        self.declare_parameter("output_prefix", "")
        self.declare_parameter("allow_overwrite", False)
        self.declare_parameter("minimum_satellites", 15)
        self.declare_parameter("maximum_hdop", 1.5)
        self.declare_parameter("maximum_correction_age_sec", 2.0)
        self.declare_parameter("maximum_fix_age_sec", 0.5)
        self.declare_parameter("averaging_window_sec", 1.0)
        self.declare_parameter("minimum_samples", 3)
        # Zero or a negative value waits indefinitely for a new good RTK fix.
        self.declare_parameter("capture_wait_timeout_sec", 0.0)
        self.declare_parameter("minimum_waypoint_spacing_m", 0.10)
        self.declare_parameter("target_speed_mps", 0.30)
        self.declare_parameter("live_track_minimum_spacing_m", 0.05)

        if not sys.stdin.isatty():
            raise RuntimeError(
                "rtk_waypoint_recorder needs an interactive terminal; run it with ros2 run"
            )
        prefix_text = str(self.get_parameter("output_prefix").value).strip()
        if not prefix_text:
            raise ValueError("set output_prefix, for example /tmp/course_01")
        prefix = Path(prefix_text).expanduser().resolve()
        self._wgs84_path = Path(str(prefix) + "_wgs84.csv")
        self._route_path = Path(str(prefix) + "_route.csv")
        self._datum_path = Path(str(prefix) + "_datum.yaml")
        outputs = (self._wgs84_path, self._route_path, self._datum_path)
        if not bool(self.get_parameter("allow_overwrite").value):
            existing = [str(path) for path in outputs if path.exists()]
            if existing:
                raise FileExistsError("refusing to overwrite: " + ", ".join(existing))
        prefix.parent.mkdir(parents=True, exist_ok=True)

        self._stdin_fd = sys.stdin.fileno()
        self._terminal_settings = termios.tcgetattr(self._stdin_fd)
        tty.setcbreak(self._stdin_fd)
        self._status: GnssStatus | None = None
        self._status_rx_sec: float | None = None
        self._latest_sample: FixedPositionSample | None = None
        self._samples: deque[FixedPositionSample] = deque(maxlen=500)
        self._waypoints: list[CapturedWaypoint] = []
        self._visual_projector: EnuProjector | None = None
        self._live_track: list[tuple[float, float]] = []
        self._pending_capture_since_sec: float | None = None
        self._pending_capture_deadline_sec: float | None = None
        self._pending_capture_last_log_sec = 0.0
        self._quit_requested = False
        self._captured_path_pub = self.create_publisher(
            PathMessage, "/route_recorder/captured_path", 1
        )
        self._live_track_pub = self.create_publisher(
            PathMessage, "/route_recorder/live_track", 1
        )
        self._current_point_pub = self.create_publisher(
            PointStamped, "/route_recorder/current_point", 10
        )
        self.create_subscription(GnssStatus, "/gnss/status", self._on_status, 20)
        self.create_subscription(NavSatFix, "/gnss/fix", self._on_fix, 20)
        self.create_timer(0.05, self._update_keyboard)
        self.create_timer(0.5, self._publish_visualization)
        self.create_timer(1.0, self._print_status)
        self.get_logger().warning(
            "GNSS ONLY: hold antenna still, ENTER=capture RTK waypoint, "
            "U=undo, P=status, Q=save/quit"
        )
        self.get_logger().info(
            f"outputs: {self._wgs84_path}, {self._route_path}, {self._datum_path}"
        )

    @property
    def quit_requested(self) -> bool:
        return self._quit_requested

    def destroy_node(self):  # type: ignore[override]
        try:
            termios.tcsetattr(
                self._stdin_fd, termios.TCSADRAIN, self._terminal_settings
            )
        finally:
            return super().destroy_node()

    def _on_status(self, message: GnssStatus) -> None:
        self._status = message
        self._status_rx_sec = time.monotonic()

    def _on_fix(self, message: NavSatFix) -> None:
        status = self._status
        if status is None:
            return
        stamp_sec = float(message.header.stamp.sec) + float(
            message.header.stamp.nanosec
        ) * 1.0e-9
        sample = FixedPositionSample(
            received_monotonic_sec=time.monotonic(),
            stamp_sec=stamp_sec,
            latitude_deg=float(message.latitude),
            longitude_deg=float(message.longitude),
            altitude_m=float(message.altitude),
            fix_type=int(status.fix_type),
            position_valid=bool(status.position_valid),
            satellites=int(status.satellites),
            hdop=float(status.hdop),
            correction_age_sec=float(status.correction_age_sec),
            nmea_checksum_valid=bool(status.nmea_checksum_valid),
        )
        self._latest_sample = sample
        self._samples.append(sample)
        self._update_live_visualization(sample)

    def _path_message(self, points: list[tuple[float, float]]) -> PathMessage:
        path = PathMessage()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"
        for x_m, y_m in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x_m
            pose.pose.position.y = y_m
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    def _update_live_visualization(self, sample: FixedPositionSample) -> None:
        if self._visual_projector is None:
            return
        x_m, y_m, _ = self._visual_projector.project(
            sample.latitude_deg,
            sample.longitude_deg,
            sample.altitude_m,
        )
        current = PointStamped()
        current.header.stamp = self.get_clock().now().to_msg()
        current.header.frame_id = "map"
        current.point.x = x_m
        current.point.y = y_m
        self._current_point_pub.publish(current)

        minimum_spacing = float(
            self.get_parameter("live_track_minimum_spacing_m").value
        )
        if (
            not self._live_track
            or math.hypot(
                x_m - self._live_track[-1][0],
                y_m - self._live_track[-1][1],
            )
            >= minimum_spacing
        ):
            self._live_track.append((x_m, y_m))
            self._live_track_pub.publish(self._path_message(self._live_track))

    def _publish_visualization(self) -> None:
        route = build_route_points(
            self._waypoints,
            float(self.get_parameter("target_speed_mps").value),
        )
        captured_points = [(point.x_m, point.y_m) for point in route]
        self._captured_path_pub.publish(self._path_message(captured_points))
        self._live_track_pub.publish(self._path_message(self._live_track))

    def _quality_issue(self, sample: FixedPositionSample) -> str | None:
        return sample_quality_issue(
            sample,
            int(self.get_parameter("minimum_satellites").value),
            float(self.get_parameter("maximum_hdop").value),
            float(self.get_parameter("maximum_correction_age_sec").value),
        )

    def _current_issue(self) -> str | None:
        sample = self._latest_sample
        if sample is None:
            return "no /gnss/fix received"
        age = time.monotonic() - sample.received_monotonic_sec
        maximum_age = float(self.get_parameter("maximum_fix_age_sec").value)
        if age > maximum_age:
            return f"GNSS fix is stale ({age:.2f}s > {maximum_age:.2f}s)"
        return self._quality_issue(sample)

    def _read_keys(self) -> None:
        while select.select([sys.stdin], [], [], 0.0)[0]:
            self._handle_key(sys.stdin.read(1))

    def _handle_key(self, key: str) -> None:
        lower = key.lower()
        if key in ("\r", "\n"):
            self._request_capture()
        elif lower == "u" or key in ("\x7f", "\b"):
            if self._pending_capture_since_sec is not None:
                self._clear_pending_capture()
                self.get_logger().warning("pending waypoint capture cancelled")
            else:
                self._undo()
        elif lower == "p":
            self._print_status()
        elif lower == "q" or key == "\x03":
            self._quit_requested = True

    def _update_keyboard(self) -> None:
        self._read_keys()
        self._service_pending_capture()

    def _capture_candidates(
        self,
        now: float,
        not_before_sec: float | None = None,
    ) -> tuple[list[FixedPositionSample], str | None]:
        current_issue = self._current_issue()
        if current_issue is not None:
            return [], current_issue
        window = float(self.get_parameter("averaging_window_sec").value)
        candidates = [
            sample
            for sample in self._samples
            if now - sample.received_monotonic_sec <= window
            and (
                not_before_sec is None
                or sample.received_monotonic_sec >= not_before_sec
            )
            and self._quality_issue(sample) is None
        ]
        minimum_samples = int(self.get_parameter("minimum_samples").value)
        if len(candidates) < minimum_samples:
            return [], (
                f"only {len(candidates)} new good samples in {window:.1f}s; "
                f"need {minimum_samples}"
            )
        return candidates, None

    def _request_capture(self) -> None:
        if self._pending_capture_since_sec is not None:
            self.get_logger().warning(
                "WAYPOINT WAITING: capture is already pending; hold still"
            )
            return
        now = time.monotonic()
        candidates, issue = self._capture_candidates(now)
        if issue is None:
            self._save_waypoint(candidates)
            return

        timeout = float(self.get_parameter("capture_wait_timeout_sec").value)
        self._pending_capture_since_sec = now
        self._pending_capture_deadline_sec = (
            math.inf if timeout <= 0.0 else now + timeout
        )
        self._pending_capture_last_log_sec = now
        timeout_text = "no timeout" if timeout <= 0.0 else f"timeout {timeout:.1f}s"
        self.get_logger().warning(
            f"WAYPOINT WAITING: {issue}; hold still for a new RTK fix "
            f"({timeout_text})"
        )

    def _service_pending_capture(self) -> None:
        since = self._pending_capture_since_sec
        deadline = self._pending_capture_deadline_sec
        if since is None or deadline is None:
            return
        now = time.monotonic()
        if math.isfinite(deadline) and now >= deadline:
            issue = self._current_issue() or "not enough new good GNSS samples"
            self._clear_pending_capture()
            self.get_logger().error(
                f"WAYPOINT REJECTED: no fresh RTK fix before timeout: {issue}"
            )
            return

        candidates, issue = self._capture_candidates(now, not_before_sec=since)
        if issue is None:
            self._clear_pending_capture()
            self._save_waypoint(candidates)
            return
        if now - self._pending_capture_last_log_sec >= 1.0:
            self._pending_capture_last_log_sec = now
            remaining_text = (
                f"{max(0.0, deadline - now):.1f}s remaining"
                if math.isfinite(deadline)
                else "waiting without timeout"
            )
            self.get_logger().info(
                f"WAYPOINT WAITING: {issue}; {remaining_text}"
            )

    def _clear_pending_capture(self) -> None:
        self._pending_capture_since_sec = None
        self._pending_capture_deadline_sec = None

    def _save_waypoint(self, candidates: list[FixedPositionSample]) -> None:
        waypoint = average_samples(candidates)
        if self._waypoints:
            spacing = waypoint_spacing_m(self._waypoints[-1], waypoint)
            if spacing <= 1.0e-6:
                self.get_logger().error(
                    "WAYPOINT REJECTED: exact duplicate position; move the antenna "
                    "or vehicle before pressing Enter again"
                )
                return
            minimum_spacing = float(
                self.get_parameter("minimum_waypoint_spacing_m").value
            )
            if spacing < minimum_spacing:
                self.get_logger().error(
                    f"WAYPOINT REJECTED: spacing={spacing:.3f}m, "
                    f"need >= {minimum_spacing:.3f}m"
                )
                return
        self._waypoints.append(waypoint)
        if len(self._waypoints) == 1:
            self._visual_projector = EnuProjector(
                waypoint.latitude_deg,
                waypoint.longitude_deg,
                waypoint.altitude_m,
            )
            self._live_track = [(0.0, 0.0)]
        self._write_outputs()
        self._publish_visualization()
        if self._latest_sample is not None:
            self._update_live_visualization(self._latest_sample)
        self.get_logger().info(
            f"WAYPOINT {len(self._waypoints) - 1} SAVED: "
            f"lat={waypoint.latitude_deg:.10f}, lon={waypoint.longitude_deg:.10f}, "
            f"samples={waypoint.sample_count}, spread={waypoint.horizontal_spread_m:.3f}m"
        )

    def _undo(self) -> None:
        if not self._waypoints:
            self.get_logger().warning("nothing to undo")
            return
        removed_index = len(self._waypoints) - 1
        self._waypoints.pop()
        if not self._waypoints:
            self._visual_projector = None
            self._live_track = []
        self._write_outputs()
        self._publish_visualization()
        self.get_logger().warning(f"waypoint {removed_index} removed")

    @staticmethod
    def _atomic_write(path: Path, writer) -> None:
        temporary = Path(str(path) + ".tmp")
        writer(temporary)
        temporary.replace(path)

    def _write_outputs(self) -> None:
        waypoints = list(self._waypoints)

        def write_wgs84(path: Path) -> None:
            with path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(
                    (
                        "index",
                        "stamp_sec",
                        "latitude_deg",
                        "longitude_deg",
                        "altitude_m",
                        "fix_type",
                        "satellites_min",
                        "hdop_max",
                        "correction_age_sec_max",
                        "samples",
                        "horizontal_spread_m",
                    )
                )
                for index, point in enumerate(waypoints):
                    writer.writerow(
                        (
                            index,
                            f"{point.stamp_sec:.9f}",
                            f"{point.latitude_deg:.10f}",
                            f"{point.longitude_deg:.10f}",
                            f"{point.altitude_m:.4f}",
                            4,
                            point.satellites_min,
                            f"{point.hdop_max:.3f}",
                            f"{point.correction_age_sec_max:.3f}",
                            point.sample_count,
                            f"{point.horizontal_spread_m:.4f}",
                        )
                    )

        def write_route(path: Path) -> None:
            route = build_route_points(
                waypoints, float(self.get_parameter("target_speed_mps").value)
            )
            with path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(
                    ("s_m", "x_m", "y_m", "target_speed_mps", "mission", "direction")
                )
                for point in route:
                    writer.writerow(
                        (
                            f"{point.s_m:.4f}",
                            f"{point.x_m:.4f}",
                            f"{point.y_m:.4f}",
                            f"{point.target_speed_mps:.3f}",
                            point.mission,
                            point.direction,
                        )
                    )

        def write_datum(path: Path) -> None:
            if waypoints:
                first = waypoints[0]
                document = {
                    "gnss_localizer": {
                        "ros__parameters": {
                            "datum_configured": True,
                            "datum_latitude_deg": first.latitude_deg,
                            "datum_longitude_deg": first.longitude_deg,
                            "datum_altitude_m": first.altitude_m,
                        }
                    },
                    "rtk_waypoint_capture": {
                        "waypoint_count": len(waypoints),
                        "wgs84_file": str(self._wgs84_path),
                        "route_file": str(self._route_path),
                    },
                }
            else:
                document = {"rtk_waypoint_capture": {"waypoint_count": 0}}
            with path.open("w", encoding="utf-8") as stream:
                yaml.safe_dump(document, stream, sort_keys=False)

        self._atomic_write(self._wgs84_path, write_wgs84)
        self._atomic_write(self._route_path, write_route)
        self._atomic_write(self._datum_path, write_datum)

    def _print_status(self) -> None:
        status = self._status
        if status is None or self._status_rx_sec is None:
            self.get_logger().warning("GNSS STATUS: waiting for /gnss/status")
            return
        age = time.monotonic() - self._status_rx_sec
        fix_name = FIX_NAMES.get(int(status.fix_type), str(int(status.fix_type)))
        correction = float(status.correction_age_sec)
        correction_text = f"{correction:.2f}s" if math.isfinite(correction) else "nan"
        verdict = "READY" if self._current_issue() is None else "NOT_READY"
        capture_text = (
            ", capture=WAITING"
            if self._pending_capture_since_sec is not None
            else ""
        )
        self.get_logger().info(
            f"GNSS {verdict}: fix={fix_name}, sats={status.satellites}, "
            f"HDOP={status.hdop:.2f}, corr_age={correction_text}, "
            f"msg_age={age:.2f}s, saved={len(self._waypoints)}{capture_text}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = RtkWaypointRecorderNode()
        while rclpy.ok() and not node.quit_requested:
            rclpy.spin_once(node, timeout_sec=0.05)
        if node is not None and len(node._waypoints) == 0:
            node.get_logger().warning("no waypoint was captured")
        elif node is not None and len(node._waypoints) == 1:
            node.get_logger().warning(
                "only one waypoint: WGS84 data is saved, route is not drivable"
            )
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
