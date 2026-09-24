"""Pure controller math used by ROS nodes and unit tests."""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass
from typing import Sequence

from .geometry import clamp, world_to_body


def normalize_angle(angle_rad: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def pure_pursuit_steering(
    vehicle_x_m: float,
    vehicle_y_m: float,
    vehicle_yaw_rad: float,
    target_x_m: float,
    target_y_m: float,
    wheelbase_m: float,
    maximum_steering_rad: float,
    direction: int = 1,
) -> float:
    body_x, body_y = world_to_body(
        target_x_m,
        target_y_m,
        vehicle_x_m,
        vehicle_y_m,
        vehicle_yaw_rad,
    )
    if direction < 0:
        body_x, body_y = -body_x, -body_y
    distance_sq = body_x * body_x + body_y * body_y
    if distance_sq < 1.0e-6:
        return 0.0
    curvature = 2.0 * body_y / distance_sq
    # The installed T870/NUCLEO convention is negative=left, positive=right.
    # Pure-pursuit geometry produces positive curvature for a target on the
    # vehicle's left, so convert that geometric sign at this boundary.
    steering = -math.atan(wheelbase_m * curvature)
    if direction < 0:
        steering = -steering
    return clamp(steering, -maximum_steering_rad, maximum_steering_rad)


@dataclass(frozen=True)
class StanleySteeringTerms:
    steering_rad: float
    cross_track_error_m: float
    heading_error_rad: float


def stanley_steering(
    vehicle_x_m: float,
    vehicle_y_m: float,
    vehicle_yaw_rad: float,
    reference_start_x_m: float,
    reference_start_y_m: float,
    reference_end_x_m: float,
    reference_end_y_m: float,
    speed_mps: float,
    gain: float,
    softening_mps: float,
    control_point_offset_m: float,
    maximum_steering_rad: float,
    heading_gain: float = 1.0,
) -> StanleySteeringTerms:
    """Compute forward Stanley steering in the installed T870 convention.

    The pose is the rear-axle ``base_link`` pose.  Stanley evaluates the
    cross-track error at a configurable point in front of it (normally the
    front axle).  Geometry is calculated with positive-left steering and is
    converted once at the output boundary to the installed negative-left,
    positive-right T870/NUCLEO convention.
    """
    dx = reference_end_x_m - reference_start_x_m
    dy = reference_end_y_m - reference_start_y_m
    if dx * dx + dy * dy < 1.0e-8:
        return StanleySteeringTerms(0.0, 0.0, 0.0)

    path_yaw = math.atan2(dy, dx)
    control_x = vehicle_x_m + control_point_offset_m * math.cos(vehicle_yaw_rad)
    control_y = vehicle_y_m + control_point_offset_m * math.sin(vehicle_yaw_rad)
    reference_to_control_x = reference_start_x_m - control_x
    reference_to_control_y = reference_start_y_m - control_y
    cross_track_error = (
        -math.sin(path_yaw) * reference_to_control_x
        + math.cos(path_yaw) * reference_to_control_y
    )
    heading_error = normalize_angle(path_yaw - vehicle_yaw_rad)
    geometric_steering = max(0.0, heading_gain) * heading_error + math.atan2(
        max(0.0, gain) * cross_track_error,
        abs(speed_mps) + max(1.0e-3, softening_mps),
    )
    steering = clamp(
        -geometric_steering,
        -maximum_steering_rad,
        maximum_steering_rad,
    )
    return StanleySteeringTerms(
        steering_rad=steering,
        cross_track_error_m=cross_track_error,
        heading_error_rad=heading_error,
    )


def pure_pursuit_weight_for_curvature(
    curvature_per_m: float,
    straight_threshold_per_m: float,
    curve_threshold_per_m: float,
    straight_pure_pursuit_weight: float,
    curve_pure_pursuit_weight: float,
) -> float:
    """Continuously increase Pure Pursuit authority as the route bends."""
    straight_threshold = max(0.0, straight_threshold_per_m)
    curve_threshold = max(straight_threshold + 1.0e-6, curve_threshold_per_m)
    ratio = clamp(
        (abs(curvature_per_m) - straight_threshold)
        / (curve_threshold - straight_threshold),
        0.0,
        1.0,
    )
    # Smoothstep avoids a steering step as classification crosses a threshold.
    blend = ratio * ratio * (3.0 - 2.0 * ratio)
    straight_weight = clamp(straight_pure_pursuit_weight, 0.0, 1.0)
    curve_weight = clamp(curve_pure_pursuit_weight, 0.0, 1.0)
    return straight_weight + blend * (curve_weight - straight_weight)


def blend_stanley_pure_pursuit(
    stanley_rad: float,
    pure_pursuit_rad: float,
    pure_pursuit_weight: float,
    maximum_steering_rad: float,
) -> float:
    """Blend two steering requests using a bounded Pure Pursuit weight."""
    weight = clamp(pure_pursuit_weight, 0.0, 1.0)
    return clamp(
        weight * pure_pursuit_rad + (1.0 - weight) * stanley_rad,
        -maximum_steering_rad,
        maximum_steering_rad,
    )


@dataclass
class SpeedPiController:
    kp: float
    ki: float
    integral_limit: float
    maximum_duty: float
    deadband_speed_mps: float
    forward_speeds: Sequence[float]
    forward_duties: Sequence[float]
    reverse_speeds: Sequence[float]
    reverse_duties: Sequence[float]
    nominal_voltage: float
    minimum_forward_duty: float = 0.0
    _integral: float = 0.0

    def reset(self) -> None:
        self._integral = 0.0

    @staticmethod
    def _interpolate(speed: float, speeds: Sequence[float], duties: Sequence[float]) -> float:
        if len(speeds) != len(duties) or not speeds:
            return 0.0
        value = abs(speed)
        if value <= speeds[0]:
            return duties[0] * (value / max(speeds[0], 1.0e-6))
        if value >= speeds[-1]:
            return duties[-1]
        upper = bisect.bisect_right(speeds, value)
        lower = upper - 1
        ratio = (value - speeds[lower]) / (speeds[upper] - speeds[lower])
        return duties[lower] + ratio * (duties[upper] - duties[lower])

    def update(
        self,
        target_speed_mps: float,
        measured_speed_mps: float,
        battery_voltage: float,
        dt_sec: float,
        freeze_integrator: bool = False,
    ) -> float:
        if abs(target_speed_mps) <= self.deadband_speed_mps:
            self.reset()
            return 0.0
        forward = target_speed_mps > 0.0
        feedforward = self._interpolate(
            target_speed_mps,
            self.forward_speeds if forward else self.reverse_speeds,
            self.forward_duties if forward else self.reverse_duties,
        )
        sign = 1.0 if forward else -1.0
        if battery_voltage > 1.0:
            feedforward *= self.nominal_voltage / battery_voltage
        error = target_speed_mps - measured_speed_mps
        if not freeze_integrator and dt_sec > 0.0:
            self._integral = clamp(
                self._integral + error * dt_sec,
                -self.integral_limit,
                self.integral_limit,
            )
        duty = sign * abs(feedforward) + self.kp * error + self.ki * self._integral
        duty = clamp(duty, -self.maximum_duty, self.maximum_duty)
        if forward and duty > 0.0:
            duty = max(
                duty,
                clamp(self.minimum_forward_duty, 0.0, self.maximum_duty),
            )
        return duty
