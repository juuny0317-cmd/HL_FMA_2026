"""Pure helpers for manually capturing RTK-fixed geographic waypoints."""

from __future__ import annotations

import math
import statistics
from dataclasses import dataclass
from typing import Iterable, Sequence

from .geometry import EnuProjector


RTK_FIXED_QUALITY = 4


@dataclass(frozen=True)
class FixedPositionSample:
    received_monotonic_sec: float
    stamp_sec: float
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    fix_type: int
    position_valid: bool
    satellites: int
    hdop: float
    correction_age_sec: float
    nmea_checksum_valid: bool


@dataclass(frozen=True)
class CapturedWaypoint:
    stamp_sec: float
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    sample_count: int
    satellites_min: int
    hdop_max: float
    correction_age_sec_max: float
    horizontal_spread_m: float


@dataclass(frozen=True)
class RoutePoint:
    s_m: float
    x_m: float
    y_m: float
    target_speed_mps: float
    mission: str
    direction: int


def waypoint_spacing_m(first: CapturedWaypoint, second: CapturedWaypoint) -> float:
    """Return horizontal spacing without requiring a drivable route segment."""

    projector = EnuProjector(
        first.latitude_deg,
        first.longitude_deg,
        first.altitude_m,
    )
    east, north, _ = projector.project(
        second.latitude_deg,
        second.longitude_deg,
        second.altitude_m,
    )
    return math.hypot(east, north)


def sample_quality_issue(
    sample: FixedPositionSample,
    minimum_satellites: int,
    maximum_hdop: float,
    maximum_correction_age_sec: float,
) -> str | None:
    """Return a concise reason when a sample is not suitable for RTK mapping."""

    if sample.fix_type != RTK_FIXED_QUALITY:
        return f"fix_type={sample.fix_type}, RTK FIX(4) required"
    if not sample.position_valid:
        return "GNSS position is not valid"
    if not sample.nmea_checksum_valid:
        return "NMEA checksum is not valid"
    if not all(
        math.isfinite(value)
        for value in (
            sample.latitude_deg,
            sample.longitude_deg,
            sample.altitude_m,
        )
    ):
        return "latitude/longitude/altitude is not finite"
    if sample.satellites < minimum_satellites:
        return f"satellites={sample.satellites}, need >= {minimum_satellites}"
    if not math.isfinite(sample.hdop) or sample.hdop > maximum_hdop:
        return f"HDOP={sample.hdop:.2f}, need <= {maximum_hdop:.2f}"
    if (
        not math.isfinite(sample.correction_age_sec)
        or sample.correction_age_sec < 0.0
        or sample.correction_age_sec > maximum_correction_age_sec
    ):
        return (
            f"correction_age={sample.correction_age_sec:.2f}s, "
            f"need 0..{maximum_correction_age_sec:.2f}s"
        )
    return None


def average_samples(samples: Sequence[FixedPositionSample]) -> CapturedWaypoint:
    """Average a short stationary window and report its horizontal spread."""

    if not samples:
        raise ValueError("at least one fixed-position sample is required")
    latitude = statistics.fmean(sample.latitude_deg for sample in samples)
    longitude = statistics.fmean(sample.longitude_deg for sample in samples)
    altitude = statistics.fmean(sample.altitude_m for sample in samples)
    projector = EnuProjector(latitude, longitude, altitude)
    horizontal_errors = [
        math.hypot(*projector.project(
            sample.latitude_deg, sample.longitude_deg, sample.altitude_m
        )[:2])
        for sample in samples
    ]
    return CapturedWaypoint(
        stamp_sec=max(sample.stamp_sec for sample in samples),
        latitude_deg=latitude,
        longitude_deg=longitude,
        altitude_m=altitude,
        sample_count=len(samples),
        satellites_min=min(sample.satellites for sample in samples),
        hdop_max=max(sample.hdop for sample in samples),
        correction_age_sec_max=max(
            sample.correction_age_sec for sample in samples
        ),
        horizontal_spread_m=math.sqrt(
            statistics.fmean(error * error for error in horizontal_errors)
        ),
    )


def build_route_points(
    waypoints: Iterable[CapturedWaypoint], target_speed_mps: float
) -> list[RoutePoint]:
    """Project manual WGS84 waypoints into a route referenced to point zero."""

    points = list(waypoints)
    if not points:
        return []
    projector = EnuProjector(
        points[0].latitude_deg,
        points[0].longitude_deg,
        points[0].altitude_m,
    )
    route: list[RoutePoint] = []
    previous_xy: tuple[float, float] | None = None
    s_m = 0.0
    for index, point in enumerate(points):
        x_m, y_m, _ = projector.project(
            point.latitude_deg, point.longitude_deg, point.altitude_m
        )
        if previous_xy is not None:
            spacing = math.hypot(x_m - previous_xy[0], y_m - previous_xy[1])
            if spacing <= 1.0e-6:
                raise ValueError("consecutive waypoints project to the same position")
            s_m += spacing
        route.append(
            RoutePoint(
                s_m=s_m,
                x_m=x_m,
                y_m=y_m,
                target_speed_mps=(0.0 if index == len(points) - 1 else target_speed_mps),
                mission=("FINISH" if index == len(points) - 1 else "NORMAL"),
                direction=1,
            )
        )
        previous_xy = (x_m, y_m)
    return route
