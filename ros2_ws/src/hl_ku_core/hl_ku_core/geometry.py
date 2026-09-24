"""Small dependency-free geometry and geodesy helpers."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple


WGS84_A_M = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_angle(angle_rad: float) -> float:
    """Wrap an angle into [-pi, pi)."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def blend_angle(previous_rad: float, measured_rad: float, gain: float) -> float:
    """Blend across the +/-pi boundary using the shortest angular difference."""
    bounded_gain = clamp(gain, 0.0, 1.0)
    return wrap_angle(
        previous_rad + bounded_gain * wrap_angle(measured_rad - previous_rad)
    )


def heading_deg_to_enu_yaw(
    heading_true_deg: float, mount_offset_rad: float = 0.0
) -> float:
    """Convert clockwise-from-north heading to ROS ENU yaw."""
    return wrap_angle(
        math.pi / 2.0 - math.radians(heading_true_deg) + mount_offset_rad
    )


def yaw_to_quaternion(yaw_rad: float) -> Tuple[float, float, float, float]:
    half = 0.5 * yaw_rad
    return (0.0, 0.0, math.sin(half), math.cos(half))


def geodetic_to_ecef(
    latitude_deg: float, longitude_deg: float, altitude_m: float
) -> Tuple[float, float, float]:
    lat = math.radians(latitude_deg)
    lon = math.radians(longitude_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    radius = WGS84_A_M / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (radius + altitude_m) * cos_lat * math.cos(lon)
    y = (radius + altitude_m) * cos_lat * math.sin(lon)
    z = (radius * (1.0 - WGS84_E2) + altitude_m) * sin_lat
    return x, y, z


@dataclass(frozen=True)
class EnuProjector:
    """WGS84 geodetic to a fixed local ENU frame."""

    latitude_deg: float
    longitude_deg: float
    altitude_m: float

    def __post_init__(self) -> None:
        x, y, z = geodetic_to_ecef(
            self.latitude_deg, self.longitude_deg, self.altitude_m
        )
        object.__setattr__(self, "_origin_ecef", (x, y, z))
        object.__setattr__(self, "_lat_rad", math.radians(self.latitude_deg))
        object.__setattr__(self, "_lon_rad", math.radians(self.longitude_deg))

    def project(
        self, latitude_deg: float, longitude_deg: float, altitude_m: float
    ) -> Tuple[float, float, float]:
        x, y, z = geodetic_to_ecef(latitude_deg, longitude_deg, altitude_m)
        x0, y0, z0 = self._origin_ecef
        dx, dy, dz = x - x0, y - y0, z - z0
        sin_lat, cos_lat = math.sin(self._lat_rad), math.cos(self._lat_rad)
        sin_lon, cos_lon = math.sin(self._lon_rad), math.cos(self._lon_rad)
        east = -sin_lon * dx + cos_lon * dy
        north = (
            -sin_lat * cos_lon * dx
            - sin_lat * sin_lon * dy
            + cos_lat * dz
        )
        up = (
            cos_lat * cos_lon * dx
            + cos_lat * sin_lon * dy
            + sin_lat * dz
        )
        return east, north, up


def antenna_to_base(
    antenna_east_m: float,
    antenna_north_m: float,
    yaw_rad: float,
    base_to_antenna_x_m: float,
    base_to_antenna_y_m: float,
) -> Tuple[float, float]:
    """Remove the planar base_link-to-ANT1 lever arm."""
    cos_yaw, sin_yaw = math.cos(yaw_rad), math.sin(yaw_rad)
    lever_east = cos_yaw * base_to_antenna_x_m - sin_yaw * base_to_antenna_y_m
    lever_north = sin_yaw * base_to_antenna_x_m + cos_yaw * base_to_antenna_y_m
    return antenna_east_m - lever_east, antenna_north_m - lever_north


def world_to_body(
    point_x: float,
    point_y: float,
    vehicle_x: float,
    vehicle_y: float,
    vehicle_yaw: float,
) -> Tuple[float, float]:
    dx, dy = point_x - vehicle_x, point_y - vehicle_y
    cos_yaw, sin_yaw = math.cos(vehicle_yaw), math.sin(vehicle_yaw)
    return cos_yaw * dx + sin_yaw * dy, -sin_yaw * dx + cos_yaw * dy


def signed_speed_from_track(
    ground_speed_mps: float, track_true_deg: float, vehicle_yaw_rad: float
) -> float:
    if not math.isfinite(ground_speed_mps) or ground_speed_mps <= 0.0:
        return 0.0
    track_yaw = heading_deg_to_enu_yaw(track_true_deg)
    return ground_speed_mps * math.cos(wrap_angle(track_yaw - vehicle_yaw_rad))
