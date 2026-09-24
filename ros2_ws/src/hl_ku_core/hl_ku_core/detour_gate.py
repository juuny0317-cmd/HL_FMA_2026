"""Choose between route steering and live detour steering without forced stops."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .detour_following import DetourFollowResult


@dataclass(frozen=True)
class DetourGateDecision:
    speed_mps: float
    steering_rad: float
    brake: bool
    engaged: bool
    detail: str = ""


def select_detour_command(
    *,
    enabled: bool,
    in_s_obstacle: bool,
    engaged: bool,
    speed_mps: float,
    route_steering_rad: float,
    legacy_steering_offset_rad: float,
    lidar_fresh: bool,
    obstacle_in_path: bool,
    obstacle_distance_m: float,
    follow: DetourFollowResult,
    approach_speed_limit_mps: float,
) -> DetourGateDecision:
    if not enabled:
        steering = route_steering_rad + (
            legacy_steering_offset_rad if in_s_obstacle else 0.0
        )
        return DetourGateDecision(
            speed_mps,
            steering,
            False,
            False,
            "local detour disabled; route fallback" if engaged else "",
        )
    if not math.isfinite(approach_speed_limit_mps) or approach_speed_limit_mps < 0.0:
        return DetourGateDecision(
            speed_mps,
            route_steering_rad,
            False,
            False,
            "invalid detour limits; route fallback",
        )
    if not in_s_obstacle:
        return DetourGateDecision(
            speed_mps,
            route_steering_rad,
            False,
            False,
            "local detour left S_OBSTACLE; route fallback" if engaged else "",
        )

    if not lidar_fresh:
        return DetourGateDecision(
            speed_mps,
            route_steering_rad,
            False,
            False,
            "local detour lidar stale; route fallback",
        )

    obstacle_ahead = (
        lidar_fresh and obstacle_in_path and math.isfinite(obstacle_distance_m)
    )
    bounded_speed = (
        max(0.0, min(speed_mps, approach_speed_limit_mps))
        if obstacle_ahead or engaged
        else speed_mps
    )
    if follow.state == "TRACK" and (obstacle_ahead or engaged):
        return DetourGateDecision(
            bounded_speed, follow.steering_rad, False, True,
            "local detour tracking",
        )
    if follow.state == "COMPLETE" and engaged:
        return DetourGateDecision(
            speed_mps, route_steering_rad, False, False,
            "local detour complete",
        )
    if follow.state == "APPROACH" and not engaged:
        return DetourGateDecision(
            bounded_speed, route_steering_rad, False, False,
            "local detour approach" if obstacle_ahead else "",
        )
    if engaged or obstacle_ahead:
        steering = route_steering_rad + legacy_steering_offset_rad
        return DetourGateDecision(
            bounded_speed,
            steering,
            False,
            False,
            f"local detour unavailable: {follow.state}; steering fallback",
        )
    return DetourGateDecision(bounded_speed, route_steering_rad, False, False)
