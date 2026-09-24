"""Outer speed controller for a traction motor without an encoder."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

from hl_ku_interfaces.msg import ActuatorCommand, DriveCommand, MissionStatus, VehicleFeedback

from .control import SpeedPiController


@dataclass
class AdaptiveHillHold:
    """Increase forward hold duty while measured motion remains backward."""

    step_duty: float
    step_interval_sec: float
    reverse_speed_threshold_mps: float
    maximum_duty: float
    active: bool = False
    current_duty: float = 0.0
    reverse_since_sec: float | None = None

    def update(
        self,
        *,
        active: bool,
        base_duty: float,
        velocity_mps: float,
        now_sec: float,
    ) -> tuple[float, bool]:
        """Return the hold duty and whether one or more PWM steps were added."""
        if not active:
            self.active = False
            self.current_duty = 0.0
            self.reverse_since_sec = None
            return 0.0, False

        if not self.active:
            self.active = True
            self.current_duty = min(self.maximum_duty, max(0.0, base_duty))
            self.reverse_since_sec = None

        if velocity_mps >= -self.reverse_speed_threshold_mps:
            self.reverse_since_sec = None
            return self.current_duty, False

        if self.reverse_since_sec is None:
            self.reverse_since_sec = now_sec
            return self.current_duty, False

        elapsed = max(0.0, now_sec - self.reverse_since_sec)
        steps = int(elapsed / self.step_interval_sec)
        if steps <= 0:
            return self.current_duty, False

        previous = self.current_duty
        self.current_duty = min(
            self.maximum_duty,
            self.current_duty + steps * self.step_duty,
        )
        self.reverse_since_sec += steps * self.step_interval_sec
        return self.current_duty, self.current_duty > previous


def velocity_input_ready(velocity_fresh: bool, required: bool) -> bool:
    return velocity_fresh or not required


def fixed_pwm_duty(target_speed_mps: float, pwm: int) -> float:
    """Use the route command only for direction and stop points in PWM mode."""
    if not 0 <= pwm <= 100:
        raise ValueError("fixed_drive_pwm must be between 0 and 100")
    if target_speed_mps == 0.0:
        return 0.0
    return math.copysign(pwm / 100.0, target_speed_mps)


def fixed_pwm_for_mission(
    mission_state: int,
    default_pwm: int,
    hill_approach_pwm: int,
    hill_post_stop_pwm: int,
) -> int:
    """Select a direct PWM override for each moving part of the hill zone."""
    if (
        mission_state == MissionStatus.STATE_HILL_APPROACH
        and hill_approach_pwm >= 0
    ):
        return hill_approach_pwm
    if (
        mission_state == MissionStatus.STATE_HILL_CLIMB
        and hill_post_stop_pwm >= 0
    ):
        return hill_post_stop_pwm
    return default_pwm


class VehicleControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("vehicle_controller")
        self.declare_parameter("kp", 0.10)
        self.declare_parameter("ki", 0.04)
        self.declare_parameter("integral_limit", 2.0)
        self.declare_parameter("maximum_duty", 0.75)
        self.declare_parameter("minimum_forward_duty", 0.0)
        self.declare_parameter("fixed_drive_pwm", -1)
        self.declare_parameter("maximum_forward_speed_mps", 1.50)
        self.declare_parameter("maximum_reverse_speed_mps", 0.70)
        self.declare_parameter("deadband_speed_mps", 0.03)
        self.declare_parameter("nominal_voltage", 22.2)
        self.declare_parameter("forward_speeds_mps", [0.25, 0.6, 1.0, 1.5])
        self.declare_parameter("forward_duties", [0.18, 0.27, 0.38, 0.55])
        self.declare_parameter("reverse_speeds_mps", [0.2, 0.4, 0.7])
        self.declare_parameter("reverse_duties", [0.20, 0.29, 0.43])
        self.declare_parameter("command_timeout_sec", 0.25)
        self.declare_parameter("velocity_timeout_sec", 0.40)
        self.declare_parameter("require_velocity_feedback", True)
        self.declare_parameter("mission_timeout_sec", 0.30)
        self.declare_parameter("hill_hold_enabled", False)
        self.declare_parameter("hill_hold_duty", 0.0)
        self.declare_parameter("hill_hold_pwm_step", 3)
        self.declare_parameter("hill_hold_step_interval_sec", 2.0)
        self.declare_parameter("hill_hold_reverse_speed_mps", 0.04)
        self.declare_parameter("hill_approach_pwm", -1)
        self.declare_parameter("hill_post_stop_pwm", -1)
        self._controller = SpeedPiController(
            kp=float(self.get_parameter("kp").value),
            ki=float(self.get_parameter("ki").value),
            integral_limit=float(self.get_parameter("integral_limit").value),
            maximum_duty=float(self.get_parameter("maximum_duty").value),
            deadband_speed_mps=float(self.get_parameter("deadband_speed_mps").value),
            forward_speeds=list(self.get_parameter("forward_speeds_mps").value),
            forward_duties=list(self.get_parameter("forward_duties").value),
            reverse_speeds=list(self.get_parameter("reverse_speeds_mps").value),
            reverse_duties=list(self.get_parameter("reverse_duties").value),
            nominal_voltage=float(self.get_parameter("nominal_voltage").value),
            minimum_forward_duty=float(
                self.get_parameter("minimum_forward_duty").value
            ),
        )
        self._fixed_drive_pwm = int(self.get_parameter("fixed_drive_pwm").value)
        if self._fixed_drive_pwm != -1 and not 0 <= self._fixed_drive_pwm <= 100:
            raise ValueError("fixed_drive_pwm must be -1 (disabled) or 0..100")
        self._hill_approach_pwm = int(self.get_parameter("hill_approach_pwm").value)
        self._hill_post_stop_pwm = int(
            self.get_parameter("hill_post_stop_pwm").value
        )
        hill_hold_pwm_step = int(self.get_parameter("hill_hold_pwm_step").value)
        hill_hold_step_interval_sec = float(
            self.get_parameter("hill_hold_step_interval_sec").value
        )
        hill_hold_reverse_speed_mps = float(
            self.get_parameter("hill_hold_reverse_speed_mps").value
        )
        if not 0 <= hill_hold_pwm_step <= 100:
            raise ValueError("hill_hold_pwm_step must be between 0 and 100")
        if hill_hold_step_interval_sec <= 0.0:
            raise ValueError("hill_hold_step_interval_sec must be positive")
        if hill_hold_reverse_speed_mps < 0.0:
            raise ValueError("hill_hold_reverse_speed_mps must be non-negative")
        self._adaptive_hill_hold = AdaptiveHillHold(
            step_duty=hill_hold_pwm_step / 100.0,
            step_interval_sec=hill_hold_step_interval_sec,
            reverse_speed_threshold_mps=hill_hold_reverse_speed_mps,
            maximum_duty=min(
                1.0, float(self.get_parameter("maximum_duty").value)
            ),
        )
        for name, pwm in (
            ("hill_approach_pwm", self._hill_approach_pwm),
            ("hill_post_stop_pwm", self._hill_post_stop_pwm),
        ):
            if pwm != -1 and not 0 <= pwm <= 100:
                raise ValueError(f"{name} must be -1 (disabled) or 0..100")
        self._command: DriveCommand | None = None
        self._command_time = None
        self._velocity = 0.0
        self._velocity_time = None
        self._mission: MissionStatus | None = None
        self._mission_time = None
        self._battery_voltage = float(self.get_parameter("nominal_voltage").value)
        self._sequence = 0
        self._last_update = time.monotonic()
        self._publisher = self.create_publisher(
            ActuatorCommand, "/vehicle/actuator_command_raw", 10
        )
        self.create_subscription(
            DriveCommand, "/mission/command", self._on_command, 10
        )
        self.create_subscription(
            TwistStamped, "/localization/vehicle_velocity", self._on_velocity, 20
        )
        self.create_subscription(MissionStatus, "/mission/status", self._on_mission, 10)
        self.create_subscription(
            VehicleFeedback, "/vehicle/feedback", self._on_feedback, 20
        )
        self.create_timer(0.02, self._update)

    def _on_command(self, message: DriveCommand) -> None:
        self._command = message
        self._command_time = time.monotonic()

    def _on_velocity(self, message: TwistStamped) -> None:
        self._velocity = float(message.twist.linear.x)
        self._velocity_time = time.monotonic()

    def _on_mission(self, message: MissionStatus) -> None:
        self._mission = message
        self._mission_time = time.monotonic()

    def _on_feedback(self, message: VehicleFeedback) -> None:
        if message.battery_voltage > 1.0:
            self._battery_voltage = float(message.battery_voltage)

    def _is_fresh(self, stamp, timeout: float) -> bool:
        if stamp is None:
            return False
        age = time.monotonic() - stamp
        return 0.0 <= age <= timeout

    def _update(self) -> None:
        now = self.get_clock().now()
        monotonic_now = time.monotonic()
        dt = min(0.1, max(0.0, monotonic_now - self._last_update))
        self._last_update = monotonic_now
        command_fresh = self._is_fresh(
            self._command_time, float(self.get_parameter("command_timeout_sec").value)
        )
        velocity_fresh = self._is_fresh(
            self._velocity_time, float(self.get_parameter("velocity_timeout_sec").value)
        )
        velocity_ready = velocity_input_ready(
            velocity_fresh,
            bool(self.get_parameter("require_velocity_feedback").value),
        )
        mission_fresh = self._is_fresh(
            self._mission_time, float(self.get_parameter("mission_timeout_sec").value)
        )
        command_values_valid = self._command is not None and all(
            math.isfinite(value)
            for value in (
                self._command.speed_mps,
                self._command.steering_angle_rad,
                self._velocity,
            )
        )
        inputs_fresh = command_fresh and velocity_ready and mission_fresh and command_values_valid
        hill_hold_active = bool(self.get_parameter("hill_hold_enabled").value) and (
            self._mission is not None
            and self._mission.state == MissionStatus.STATE_HILL_HOLD
        )
        brake = (
            not command_fresh
            or not velocity_ready
            or not mission_fresh
            or not command_values_valid
            or self._mission.brake_required
        )
        if inputs_fresh and hill_hold_active:
            brake = False
        hill_hold_duty, hill_hold_stepped = self._adaptive_hill_hold.update(
            active=hill_hold_active,
            base_duty=float(self.get_parameter("hill_hold_duty").value),
            velocity_mps=self._velocity if velocity_fresh else 0.0,
            now_sec=monotonic_now,
        )
        if hill_hold_stepped:
            self.get_logger().warning(
                "hill rollback persists: increasing hold command to "
                f"PWM {round(hill_hold_duty * 100.0)}"
            )
        target_speed = float(self._command.speed_mps) if command_values_valid else 0.0
        target_speed = max(
            -float(self.get_parameter("maximum_reverse_speed_mps").value),
            min(
                float(self.get_parameter("maximum_forward_speed_mps").value),
                target_speed,
            ),
        )
        duty = 0.0
        if hill_hold_active and not brake:
            self._controller.reset()
            duty = hill_hold_duty
        elif not brake:
            mission_state = (
                int(self._mission.state)
                if self._mission is not None
                else MissionStatus.STATE_INIT
            )
            selected_pwm = fixed_pwm_for_mission(
                mission_state,
                self._fixed_drive_pwm,
                self._hill_approach_pwm,
                self._hill_post_stop_pwm,
            )
            if selected_pwm >= 0:
                self._controller.reset()
                duty = fixed_pwm_duty(target_speed, selected_pwm)
            else:
                duty = self._controller.update(
                    target_speed,
                    self._velocity,
                    self._battery_voltage,
                    dt,
                )
        else:
            self._controller.reset()
        self._sequence = (self._sequence + 1) & 0xFFFFFFFF
        output = ActuatorCommand()
        output.header.stamp = now.to_msg()
        output.header.frame_id = "base_link"
        output.drive_duty = duty
        output.steering_angle_rad = (
            float(self._command.steering_angle_rad) if self._command else 0.0
        )
        output.brake = brake
        output.enable = not brake
        output.mode = ActuatorCommand.MODE_BRAKE if brake else ActuatorCommand.MODE_AUTONOMOUS
        output.sequence = self._sequence
        self._publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VehicleControllerNode()
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
