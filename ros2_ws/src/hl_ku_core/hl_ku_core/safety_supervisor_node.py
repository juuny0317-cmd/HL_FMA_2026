"""Final fail-closed gate before actuator commands leave the main computer."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String

from hl_ku_interfaces.msg import (
    ActuatorCommand,
    GnssStatus,
    MissionStatus,
    PerceptionState,
    VehicleFeedback,
)


def required_safety_inputs(
    manual_test: bool, relaxed_route_test: bool
) -> tuple[tuple[str, str], ...]:
    if manual_test:
        return (
            ("raw", "raw_command_timeout_sec"),
            ("feedback", "feedback_timeout_sec"),
        )
    if relaxed_route_test:
        return (
            ("raw", "raw_command_timeout_sec"),
            ("pose", "pose_timeout_sec"),
            ("feedback", "feedback_timeout_sec"),
            ("mission", "mission_timeout_sec"),
        )
    return (
        ("raw", "raw_command_timeout_sec"),
        ("gnss", "gnss_timeout_sec"),
        ("pose", "pose_timeout_sec"),
        ("feedback", "feedback_timeout_sec"),
        ("mission", "mission_timeout_sec"),
    )


def lidar_emergency_brake_required(
    obstacle_in_path: bool,
    obstacle_distance_m: float,
    emergency_distance_m: float,
    mission_state: int,
) -> bool:
    """Keep the LiDAR emergency brake out of the active S avoidance zone."""
    return (
        mission_state != MissionStatus.STATE_S_OBSTACLE
        and obstacle_in_path
        and math.isfinite(obstacle_distance_m)
        and obstacle_distance_m <= emergency_distance_m
    )


class SafetySupervisorNode(Node):
    def __init__(self) -> None:
        super().__init__("safety_supervisor")
        self.declare_parameter("drive_enabled", False)
        self.declare_parameter("route_calibrated", False)
        self.declare_parameter("require_rtk_fixed", True)
        self.declare_parameter("require_gnss_heading", True)
        self.declare_parameter("require_perception", True)
        self.declare_parameter("require_camera", True)
        self.declare_parameter("require_lidar", True)
        self.declare_parameter("require_preflight", True)
        self.declare_parameter("manual_test_mode", False)
        self.declare_parameter("relaxed_route_test_mode", False)
        self.declare_parameter("autonomous_maximum_drive_duty", 0.75)
        self.declare_parameter("autonomous_maximum_steering_rad", 0.48)
        self.declare_parameter("manual_maximum_drive_duty", 0.20)
        self.declare_parameter("manual_maximum_steering_rad", 0.35)
        self.declare_parameter("raw_command_timeout_sec", 0.20)
        self.declare_parameter("gnss_timeout_sec", 0.40)
        self.declare_parameter("pose_timeout_sec", 0.40)
        self.declare_parameter("feedback_timeout_sec", 0.30)
        self.declare_parameter("perception_timeout_sec", 0.35)
        self.declare_parameter("mission_timeout_sec", 0.30)
        self.declare_parameter("preflight_timeout_sec", 2.0)
        self.declare_parameter("emergency_distance_m", 0.75)
        self.declare_parameter("maximum_gnss_hdop", 1.5)
        self.declare_parameter("maximum_correction_age_sec", 2.0)
        self.declare_parameter("minimum_gnss_satellites", 8)
        self._raw: ActuatorCommand | None = None
        self._gnss: GnssStatus | None = None
        self._pose: PoseWithCovarianceStamped | None = None
        self._feedback: VehicleFeedback | None = None
        self._perception: PerceptionState | None = None
        self._mission: MissionStatus | None = None
        self._preflight: Bool | None = None
        self._times = {}
        self._last_reason = ""
        self._publisher = self.create_publisher(
            ActuatorCommand, "/vehicle/actuator_command_safe", 10
        )
        self._diagnostic_pub = self.create_publisher(String, "/safety/status", 10)
        self.create_subscription(
            ActuatorCommand,
            "/vehicle/actuator_command_raw",
            lambda message: self._store("raw", message),
            20,
        )
        self.create_subscription(
            GnssStatus, "/gnss/status", lambda message: self._store("gnss", message), 20
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/localization/gnss_pose",
            lambda message: self._store("pose", message),
            20,
        )
        self.create_subscription(
            VehicleFeedback,
            "/vehicle/feedback",
            lambda message: self._store("feedback", message),
            20,
        )
        self.create_subscription(
            PerceptionState,
            "/perception/state",
            lambda message: self._store("perception", message),
            20,
        )
        self.create_subscription(
            MissionStatus,
            "/mission/status",
            lambda message: self._store("mission", message),
            20,
        )
        self.create_subscription(
            Bool,
            "/system/preflight_ok",
            lambda message: self._store("preflight", message),
            10,
        )
        self.create_timer(0.02, self._update)

    def _store(self, name: str, message) -> None:
        setattr(self, f"_{name}", message)
        self._times[name] = time.monotonic()

    def _fresh(self, name: str, timeout_parameter: str) -> bool:
        stamp = self._times.get(name)
        if stamp is None:
            return False
        age = time.monotonic() - stamp
        return 0.0 <= age <= float(self.get_parameter(timeout_parameter).value)

    def _blocking_reasons(self) -> list[str]:
        reasons: list[str] = []
        manual_test = bool(self.get_parameter("manual_test_mode").value)
        relaxed_route_test = bool(
            self.get_parameter("relaxed_route_test_mode").value
        )
        if not bool(self.get_parameter("drive_enabled").value):
            reasons.append("drive_enabled=false")
        required_inputs = required_safety_inputs(manual_test, relaxed_route_test)
        for name, timeout in required_inputs:
            if not self._fresh(name, timeout):
                reasons.append(f"{name}_stale")
        if (
            not manual_test
            and bool(self.get_parameter("require_preflight").value)
        ):
            if not self._fresh("preflight", "preflight_timeout_sec"):
                reasons.append("preflight_stale")
            elif self._preflight is None or not self._preflight.data:
                reasons.append("preflight_failed")
        if not manual_test and not bool(self.get_parameter("route_calibrated").value):
            reasons.append("route_calibrated=false")
        if self._raw is not None:
            drive = float(self._raw.drive_duty)
            steering = float(self._raw.steering_angle_rad)
            if not (math.isfinite(drive) and math.isfinite(steering)):
                reasons.append("raw_command_nonfinite")
            if manual_test and self._raw.mode == ActuatorCommand.MODE_FAULT:
                reasons.append("manual_emergency_latched")
            elif manual_test and self._raw.mode != ActuatorCommand.MODE_MANUAL:
                reasons.append("manual_mode_mismatch")
            elif not manual_test and self._raw.mode not in (
                ActuatorCommand.MODE_AUTONOMOUS,
                ActuatorCommand.MODE_BRAKE,
            ):
                reasons.append("autonomous_mode_mismatch")
            drive_limit_parameter = (
                "manual_maximum_drive_duty"
                if manual_test
                else "autonomous_maximum_drive_duty"
            )
            steering_limit_parameter = (
                "manual_maximum_steering_rad"
                if manual_test
                else "autonomous_maximum_steering_rad"
            )
            drive_limit = float(self.get_parameter(drive_limit_parameter).value)
            if math.isfinite(drive) and abs(drive) > drive_limit + 1.0e-6:
                reasons.append("raw_drive_limit")
            if math.isfinite(steering) and abs(steering) > float(
                self.get_parameter(steering_limit_parameter).value
            ):
                reasons.append("raw_steering_limit")
            active = self._raw.enable and not self._raw.brake
            braking = not self._raw.enable and self._raw.brake
            if not (active or braking):
                reasons.append("raw_command_flags_invalid")
            if braking and abs(drive) > 1.0e-6:
                reasons.append("brake_command_has_drive")
        if (
            not manual_test
            and bool(self.get_parameter("require_perception").value)
            and not self._fresh("perception", "perception_timeout_sec")
        ):
            reasons.append("perception_stale")
        if (
            not manual_test
            and bool(self.get_parameter("require_perception").value)
            and self._perception is not None
        ):
            if (
                bool(self.get_parameter("require_camera").value)
                and not self._perception.camera_fresh
            ):
                reasons.append("camera_source_stale")
            if (
                bool(self.get_parameter("require_lidar").value)
                and not self._perception.lidar_fresh
            ):
                reasons.append("lidar_source_stale")
            if (
                self._mission is not None
                and self._mission.state == MissionStatus.STATE_END_LANE
                and (
                    not self._perception.end_signal_fresh
                    or self._perception.allowed_lane == 0
                )
            ):
                reasons.append("end_lane_signal_invalid")
        if not manual_test and not relaxed_route_test and self._gnss is not None:
            if bool(self.get_parameter("require_rtk_fixed").value) and (
                self._gnss.fix_type != GnssStatus.FIX_RTK_FIXED
            ):
                reasons.append("rtk_not_fixed")
            if (
                bool(self.get_parameter("require_gnss_heading").value)
                and not self._gnss.heading_valid
            ):
                reasons.append("gnss_heading_invalid")
            if not self._gnss.nmea_checksum_valid:
                reasons.append("gnss_checksum_invalid")
            if not self._gnss.position_valid:
                reasons.append("gnss_position_invalid")
            if not self._gnss.velocity_valid:
                reasons.append("gnss_velocity_invalid")
            hdop = float(self._gnss.hdop)
            if not math.isfinite(hdop) or hdop > float(
                self.get_parameter("maximum_gnss_hdop").value
            ):
                reasons.append("gnss_hdop_invalid")
            correction_age = float(self._gnss.correction_age_sec)
            if not math.isfinite(correction_age) or correction_age > float(
                self.get_parameter("maximum_correction_age_sec").value
            ):
                reasons.append("rtk_correction_stale")
            if self._gnss.satellites < int(
                self.get_parameter("minimum_gnss_satellites").value
            ):
                reasons.append("gnss_satellites_low")
        if not manual_test and self._pose is not None:
            position = self._pose.pose.pose.position
            orientation = self._pose.pose.pose.orientation
            if not all(
                math.isfinite(value)
                for value in (
                    position.x,
                    position.y,
                    position.z,
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w,
                )
            ):
                reasons.append("pose_nonfinite")
        if self._feedback is not None and self._feedback.fault_flags:
            reasons.append(f"mcu_fault=0x{self._feedback.fault_flags:08x}")
        if self._feedback is not None and not all(
            math.isfinite(value)
            for value in (
                self._feedback.steering_angle_rad,
                self._feedback.steering_target_rad,
                self._feedback.applied_drive_duty,
                self._feedback.battery_voltage,
            )
        ):
            reasons.append("mcu_feedback_nonfinite")
        if not manual_test and self._mission is not None and self._mission.state in (
            MissionStatus.STATE_INIT,
            MissionStatus.STATE_FAULT,
            MissionStatus.STATE_FINISH,
        ):
            reasons.append(f"mission_{self._mission.state_name.lower()}")
        if self._perception is not None:
            distance = float(self._perception.obstacle_distance_m)
            mission_state = (
                int(self._mission.state)
                if self._mission is not None
                else MissionStatus.STATE_INIT
            )
            if lidar_emergency_brake_required(
                bool(self._perception.obstacle_in_path),
                distance,
                float(self.get_parameter("emergency_distance_m").value),
                mission_state,
            ):
                reasons.append("lidar_emergency_distance")
        return reasons

    def _update(self) -> None:
        reasons = self._blocking_reasons()
        now = self.get_clock().now().to_msg()
        output = ActuatorCommand()
        output.header.stamp = now
        output.header.frame_id = "base_link"
        if not reasons and self._raw is not None:
            if bool(self.get_parameter("manual_test_mode").value):
                drive_limit = float(
                    self.get_parameter("manual_maximum_drive_duty").value
                )
                steering_limit = float(
                    self.get_parameter("manual_maximum_steering_rad").value
                )
                output.drive_duty = max(
                    -drive_limit, min(drive_limit, self._raw.drive_duty)
                )
                output.steering_angle_rad = max(
                    -steering_limit,
                    min(steering_limit, self._raw.steering_angle_rad),
                )
            else:
                output.drive_duty = self._raw.drive_duty
                output.steering_angle_rad = self._raw.steering_angle_rad
            output.brake = self._raw.brake
            output.enable = self._raw.enable
            output.mode = self._raw.mode
            output.sequence = self._raw.sequence
        else:
            output.drive_duty = 0.0
            feedback_steering = (
                float(self._feedback.steering_angle_rad) if self._feedback else 0.0
            )
            output.steering_angle_rad = (
                feedback_steering if math.isfinite(feedback_steering) else 0.0
            )
            output.brake = True
            output.enable = False
            output.mode = (
                ActuatorCommand.MODE_FAULT
                if self._feedback is not None and self._feedback.fault_flags
                else ActuatorCommand.MODE_BRAKE
            )
            output.sequence = self._raw.sequence if self._raw else 0
        self._publisher.publish(output)
        reason_text = "OK" if not reasons else ",".join(reasons)
        if reason_text != self._last_reason:
            if reasons:
                self.get_logger().warning(f"actuation inhibited: {reason_text}")
            else:
                self.get_logger().info("all safety gates passed")
            self._last_reason = reason_text
        diagnostic = String()
        diagnostic.data = reason_text
        self._diagnostic_pub.publish(diagnostic)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetySupervisorNode()
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
