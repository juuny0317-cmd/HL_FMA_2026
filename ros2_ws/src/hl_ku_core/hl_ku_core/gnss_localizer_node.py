"""Project UM982 fixes into a fixed ENU map and remove the ANT1 lever arm."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from hl_ku_interfaces.msg import GnssStatus

from .geometry import (
    EnuProjector,
    antenna_to_base,
    blend_angle,
    heading_deg_to_enu_yaw,
    signed_speed_from_track,
    yaw_to_quaternion,
)


class GnssLocalizerNode(Node):
    def __init__(self) -> None:
        super().__init__("gnss_localizer")
        self.declare_parameter("datum_configured", False)
        self.declare_parameter("datum_latitude_deg", 0.0)
        self.declare_parameter("datum_longitude_deg", 0.0)
        self.declare_parameter("datum_altitude_m", 0.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_to_ant1_x_m", 0.0)
        self.declare_parameter("base_to_ant1_y_m", 0.0)
        self.declare_parameter("heading_mount_offset_deg", 0.0)
        self.declare_parameter("require_rtk_fixed", True)
        self.declare_parameter("fixed_position_sigma_m", 0.02)
        self.declare_parameter("float_position_sigma_m", 0.30)
        self.declare_parameter("fixed_heading_sigma_deg", 0.5)
        self.declare_parameter("allow_single_antenna_heading", False)
        self.declare_parameter("single_antenna_initial_heading_deg", -1.0)
        self.declare_parameter("single_antenna_minimum_track_speed_mps", 0.15)
        self.declare_parameter("single_antenna_track_filter_gain", 0.25)
        self.declare_parameter("single_antenna_heading_sigma_deg", 5.0)
        self._datum_configured = bool(self.get_parameter("datum_configured").value)
        self._projector = EnuProjector(
            float(self.get_parameter("datum_latitude_deg").value),
            float(self.get_parameter("datum_longitude_deg").value),
            float(self.get_parameter("datum_altitude_m").value),
        )
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._status: GnssStatus | None = None
        self._single_antenna_yaw: float | None = None
        self._heading_source = ""
        if bool(self.get_parameter("allow_single_antenna_heading").value):
            initial_heading = float(
                self.get_parameter("single_antenna_initial_heading_deg").value
            )
            if 0.0 <= initial_heading < 360.0:
                self._single_antenna_yaw = heading_deg_to_enu_yaw(initial_heading)
                self.get_logger().warning(
                    "single-antenna mode: align the stationary vehicle to "
                    f"true heading {initial_heading:.2f} deg before arming"
                )
            else:
                self.get_logger().error(
                    "single-antenna mode requires an initial heading in [0, 360)"
                )
        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/localization/gnss_pose", 10
        )
        self._velocity_pub = self.create_publisher(
            TwistStamped, "/localization/gnss_velocity", 10
        )
        self.create_subscription(GnssStatus, "/gnss/status", self._on_status, 20)
        self.create_subscription(NavSatFix, "/gnss/fix", self._on_fix, 20)
        if not self._datum_configured:
            self.get_logger().error(
                "GNSS datum is not configured; localization output is intentionally disabled"
            )

    def _on_status(self, message: GnssStatus) -> None:
        self._status = message

    def _yaw_from_status(self, status: GnssStatus) -> tuple[float, float] | None:
        if status.heading_valid and math.isfinite(status.heading_true_deg):
            yaw = heading_deg_to_enu_yaw(
                status.heading_true_deg,
                math.radians(
                    float(self.get_parameter("heading_mount_offset_deg").value)
                ),
            )
            self._single_antenna_yaw = yaw
            source = "dual-antenna heading"
            sigma_deg = float(self.get_parameter("fixed_heading_sigma_deg").value)
        elif bool(self.get_parameter("allow_single_antenna_heading").value):
            minimum_speed = float(
                self.get_parameter("single_antenna_minimum_track_speed_mps").value
            )
            track_valid = (
                status.velocity_valid
                and math.isfinite(status.ground_speed_mps)
                and status.ground_speed_mps >= minimum_speed
                and math.isfinite(status.track_true_deg)
            )
            if track_valid:
                measured_yaw = heading_deg_to_enu_yaw(status.track_true_deg)
                if self._single_antenna_yaw is None:
                    self._single_antenna_yaw = measured_yaw
                else:
                    self._single_antenna_yaw = blend_angle(
                        self._single_antenna_yaw,
                        measured_yaw,
                        float(
                            self.get_parameter(
                                "single_antenna_track_filter_gain"
                            ).value
                        ),
                    )
                source = "single-antenna GNSS track"
            elif self._single_antenna_yaw is not None:
                source = "single-antenna held/initial heading"
            else:
                return None
            yaw = self._single_antenna_yaw
            sigma_deg = float(
                self.get_parameter("single_antenna_heading_sigma_deg").value
            )
        else:
            return None

        if source != self._heading_source:
            self.get_logger().info(f"localization heading source: {source}")
            self._heading_source = source
        return yaw, sigma_deg

    def _on_fix(self, fix: NavSatFix) -> None:
        status = self._status
        if (
            not self._datum_configured
            or status is None
            or not status.position_valid
        ):
            return
        if bool(self.get_parameter("require_rtk_fixed").value) and (
            status.fix_type != GnssStatus.FIX_RTK_FIXED
        ):
            return
        heading = self._yaw_from_status(status)
        if heading is None:
            return
        yaw, heading_sigma_deg = heading
        east, north, up = self._projector.project(
            fix.latitude, fix.longitude, fix.altitude
        )
        east, north = antenna_to_base(
            east,
            north,
            yaw,
            float(self.get_parameter("base_to_ant1_x_m").value),
            float(self.get_parameter("base_to_ant1_y_m").value),
        )
        pose = PoseWithCovarianceStamped()
        pose.header = fix.header
        pose.header.frame_id = self._map_frame
        pose.pose.pose.position.x = east
        pose.pose.pose.position.y = north
        pose.pose.pose.position.z = up
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        pose.pose.pose.orientation.x = qx
        pose.pose.pose.orientation.y = qy
        pose.pose.pose.orientation.z = qz
        pose.pose.pose.orientation.w = qw
        fixed = status.fix_type == GnssStatus.FIX_RTK_FIXED
        position_sigma = float(
            self.get_parameter(
                "fixed_position_sigma_m" if fixed else "float_position_sigma_m"
            ).value
        )
        yaw_sigma = math.radians(heading_sigma_deg if fixed else 5.0)
        pose.pose.covariance[0] = position_sigma**2
        pose.pose.covariance[7] = position_sigma**2
        pose.pose.covariance[14] = max(position_sigma * 1.8, 0.04) ** 2
        pose.pose.covariance[35] = yaw_sigma**2
        self._pose_pub.publish(pose)

        if status.velocity_valid:
            velocity = TwistStamped()
            velocity.header = pose.header
            velocity.twist.linear.x = signed_speed_from_track(
                status.ground_speed_mps, status.track_true_deg, yaw
            )
            self._velocity_pub.publish(velocity)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GnssLocalizerNode()
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
