"""Collect only RTK-fixed samples and print a repeatable ENU datum candidate."""

from __future__ import annotations

import math
import statistics
from datetime import datetime, timezone
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from hl_ku_interfaces.msg import GnssStatus


class GnssSurveyNode(Node):
    def __init__(self) -> None:
        super().__init__("gnss_survey")
        self.declare_parameter("duration_sec", 120.0)
        self.declare_parameter("minimum_samples", 100)
        self.declare_parameter("maximum_hdop", 1.5)
        self.declare_parameter("output_file", "")
        self.declare_parameter("site_name", "datum")
        self._status: GnssStatus | None = None
        self._samples: list[tuple[float, float, float]] = []
        self._start = self.get_clock().now()
        self._finished = False
        result_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._progress_pub = self.create_publisher(
            String, "/survey/progress", 10
        )
        self._result_pub = self.create_publisher(
            String, "/survey/result", result_qos
        )
        self.create_subscription(GnssStatus, "/gnss/status", self._on_status, 20)
        self.create_subscription(NavSatFix, "/gnss/fix", self._on_fix, 20)
        self.create_timer(5.0, self._progress)

    def _on_status(self, message: GnssStatus) -> None:
        self._status = message

    def _on_fix(self, message: NavSatFix) -> None:
        if self._finished or self._status is None:
            return
        if self._status.fix_type != GnssStatus.FIX_RTK_FIXED:
            return
        hdop = float(self._status.hdop)
        if math.isfinite(hdop) and hdop > float(self.get_parameter("maximum_hdop").value):
            return
        if all(math.isfinite(value) for value in (message.latitude, message.longitude, message.altitude)):
            self._samples.append((message.latitude, message.longitude, message.altitude))

    def _progress(self) -> None:
        elapsed = (self.get_clock().now() - self._start).nanoseconds * 1e-9
        duration = float(self.get_parameter("duration_sec").value)
        if elapsed < duration:
            progress = String()
            progress.data = (
                f"{elapsed:.0f}/{duration:.0f}s · "
                f"RTK FIXED samples={len(self._samples)}"
            )
            self._progress_pub.publish(progress)
            self.get_logger().info(
                f"datum survey {elapsed:.0f}/{duration:.0f}s, RTK-fixed samples={len(self._samples)}"
            )
            return
        minimum = int(self.get_parameter("minimum_samples").value)
        if len(self._samples) < minimum:
            self.get_logger().error(
                f"datum rejected: only {len(self._samples)} fixed samples; need {minimum}"
            )
            self._start = self.get_clock().now()
            self._samples.clear()
            return
        latitude = statistics.fmean(sample[0] for sample in self._samples)
        longitude = statistics.fmean(sample[1] for sample in self._samples)
        altitude = statistics.fmean(sample[2] for sample in self._samples)
        lat_spread = statistics.pstdev(sample[0] for sample in self._samples) * 111_320.0
        lon_spread = (
            statistics.pstdev(sample[1] for sample in self._samples)
            * 111_320.0
            * math.cos(math.radians(latitude))
        )
        horizontal_sigma = math.hypot(lat_spread, lon_spread)
        output_path = self._write_result(
            latitude,
            longitude,
            altitude,
            horizontal_sigma,
            len(self._samples),
        )
        result = String()
        result.data = (
            f"{output_path} · lat={latitude:.10f} lon={longitude:.10f} "
            f"alt={altitude:.4f}m sigma={horizontal_sigma:.4f}m "
            f"samples={len(self._samples)}"
        )
        self._result_pub.publish(result)
        self.get_logger().info(
            "DATUM ACCEPTED\n"
            f"datum_latitude_deg: {latitude:.10f}\n"
            f"datum_longitude_deg: {longitude:.10f}\n"
            f"datum_altitude_m: {altitude:.4f}\n"
            f"horizontal_1sigma_m: {horizontal_sigma:.4f}\n"
            f"samples: {len(self._samples)}\n"
            f"output_file: {output_path}"
        )
        self._finished = True

    def _write_result(
        self,
        latitude: float,
        longitude: float,
        altitude: float,
        horizontal_sigma: float,
        sample_count: int,
    ) -> Path:
        output_text = str(self.get_parameter("output_file").value).strip()
        if not output_text:
            raise ValueError("datum survey output_file is empty")
        output_path = Path(output_text).expanduser().resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        document = {
            "site": str(self.get_parameter("site_name").value),
            "survey": {
                "captured_at_utc": datetime.now(timezone.utc).isoformat(),
                "duration_sec": float(self.get_parameter("duration_sec").value),
                "samples": sample_count,
                "maximum_hdop": float(self.get_parameter("maximum_hdop").value),
                "horizontal_1sigma_m": horizontal_sigma,
            },
            "gnss_localizer": {
                "ros__parameters": {
                    "datum_configured": True,
                    "datum_latitude_deg": latitude,
                    "datum_longitude_deg": longitude,
                    "datum_altitude_m": altitude,
                }
            },
        }
        temporary = output_path.with_suffix(output_path.suffix + ".tmp")
        temporary.write_text(
            yaml.safe_dump(document, allow_unicode=True, sort_keys=False),
            encoding="utf-8",
        )
        temporary.replace(output_path)
        return output_path


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GnssSurveyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
