"""Record a calibrated ENU route with mission labels during pre-race mapping."""

from __future__ import annotations

import csv
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import Float32, Int8, String
from std_srvs.srv import Trigger


class RouteRecorderNode(Node):
    def __init__(self) -> None:
        super().__init__("route_recorder")
        self.declare_parameter("output_file", "")
        self.declare_parameter("minimum_spacing_m", 0.10)
        self.declare_parameter("default_speed_mps", 0.6)
        self.declare_parameter("allow_overwrite", False)
        self.declare_parameter("force_finish_tag", True)
        self._recording = False
        self._mission = "NORMAL"
        self._direction = 1
        self._target_speed = float(self.get_parameter("default_speed_mps").value)
        self._points: list[tuple[float, float, float, float, str, int]] = []
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/localization/gnss_pose",
            self._on_pose,
            20,
        )
        self.create_subscription(String, "/route_recorder/mission", self._on_mission, 10)
        self.create_subscription(Int8, "/route_recorder/direction", self._on_direction, 10)
        self.create_subscription(Float32, "/route_recorder/speed", self._on_speed, 10)
        self.create_service(Trigger, "/route_recorder/start", self._start)
        self.create_service(Trigger, "/route_recorder/stop_and_save", self._stop_and_save)

    def _on_mission(self, message: String) -> None:
        self._mission = message.data.strip().upper() or "NORMAL"

    def _on_direction(self, message: Int8) -> None:
        if message.data in (-1, 1):
            self._direction = int(message.data)

    def _on_speed(self, message: Float32) -> None:
        self._target_speed = max(0.0, float(message.data))

    def _start(self, _request, response):
        output = Path(str(self.get_parameter("output_file").value)).expanduser()
        if not output.name:
            response.success = False
            response.message = "set output_file first"
            return response
        if output.exists() and not bool(self.get_parameter("allow_overwrite").value):
            response.success = False
            response.message = f"refusing to overwrite {output}"
            return response
        self._points.clear()
        self._recording = True
        response.success = True
        response.message = f"recording to memory for {output}"
        return response

    def _on_pose(self, message: PoseWithCovarianceStamped) -> None:
        if not self._recording:
            return
        x_m = float(message.pose.pose.position.x)
        y_m = float(message.pose.pose.position.y)
        if self._points:
            spacing = math.hypot(x_m - self._points[-1][1], y_m - self._points[-1][2])
            if spacing < float(self.get_parameter("minimum_spacing_m").value):
                return
            s_m = self._points[-1][0] + spacing
        else:
            s_m = 0.0
        self._points.append(
            (s_m, x_m, y_m, self._target_speed, self._mission, self._direction)
        )

    def _stop_and_save(self, _request, response):
        self._recording = False
        if len(self._points) < 2:
            response.success = False
            response.message = "fewer than two points; nothing saved"
            return response
        output = Path(str(self.get_parameter("output_file").value)).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        if bool(self.get_parameter("force_finish_tag").value):
            final = self._points[-1]
            self._points[-1] = (final[0], final[1], final[2], 0.0, "FINISH", 1)
        with output.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(("s_m", "x_m", "y_m", "target_speed_mps", "mission", "direction"))
            writer.writerows(self._points)
        response.success = True
        response.message = f"saved {len(self._points)} points to {output}"
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RouteRecorderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
