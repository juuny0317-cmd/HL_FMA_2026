"""Pure geometry used to project clustered LiDAR returns into the map frame."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class MapperSettings:
    minimum_range_m: float = 0.50
    maximum_range_m: float = 12.0
    front_half_angle_rad: float = math.radians(70.0)
    minimum_cluster_points: int = 3
    maximum_cluster_gap_m: float = 0.35
    lidar_x_m: float = 0.0
    lidar_y_m: float = 0.0
    lidar_yaw_rad: float = 0.0
    minimum_obstacle_radius_m: float = 0.40
    radius_margin_m: float = 0.10

    def validate(self) -> None:
        finite = (
            self.minimum_range_m,
            self.maximum_range_m,
            self.front_half_angle_rad,
            self.maximum_cluster_gap_m,
            self.lidar_x_m,
            self.lidar_y_m,
            self.lidar_yaw_rad,
            self.minimum_obstacle_radius_m,
            self.radius_margin_m,
        )
        if not all(math.isfinite(value) for value in finite):
            raise ValueError("LiDAR mapping parameters must be finite")
        if not 0.0 <= self.minimum_range_m < self.maximum_range_m:
            raise ValueError("LiDAR mapping range is invalid")
        if not 0.0 < self.front_half_angle_rad <= math.pi:
            raise ValueError("front_half_angle_rad must be in (0, pi]")
        if self.minimum_cluster_points < 1:
            raise ValueError("minimum_cluster_points must be positive")
        if self.maximum_cluster_gap_m <= 0.0:
            raise ValueError("maximum_cluster_gap_m must be positive")
        if self.minimum_obstacle_radius_m <= 0.0 or self.radius_margin_m < 0.0:
            raise ValueError("obstacle radius settings are invalid")


@dataclass(frozen=True)
class VehiclePose:
    x_m: float
    y_m: float
    yaw_rad: float


def _transform(
    x_m: float,
    y_m: float,
    translation_x_m: float,
    translation_y_m: float,
    yaw_rad: float,
) -> tuple[float, float]:
    cosine = math.cos(yaw_rad)
    sine = math.sin(yaw_rad)
    return (
        translation_x_m + cosine * x_m - sine * y_m,
        translation_y_m + sine * x_m + cosine * y_m,
    )


def clustered_obstacle_discs_in_map(
    ranges: Sequence[float],
    angle_min_rad: float,
    angle_increment_rad: float,
    scan_range_min_m: float,
    scan_range_max_m: float,
    vehicle_pose: VehiclePose,
    settings: MapperSettings,
) -> tuple[tuple[float, float, float], ...]:
    """Return map-frame center and visible radius for each scan cluster.

    Invalid samples and returns outside the configured front sector terminate a
    cluster.  A large Cartesian gap also splits adjacent scan samples so two
    objects at different depths do not become one obstacle.
    """

    settings.validate()
    if not ranges:
        raise ValueError("LaserScan ranges are empty")
    if not math.isfinite(angle_min_rad):
        raise ValueError("LaserScan angle_min is invalid")
    if not math.isfinite(angle_increment_rad) or angle_increment_rad == 0.0:
        raise ValueError("LaserScan angle_increment is invalid")
    if (
        not math.isfinite(scan_range_min_m)
        or not math.isfinite(scan_range_max_m)
        or scan_range_max_m <= scan_range_min_m
    ):
        raise ValueError("LaserScan range limits are invalid")
    if not all(
        math.isfinite(value)
        for value in (vehicle_pose.x_m, vehicle_pose.y_m, vehicle_pose.yaw_rad)
    ):
        raise ValueError("vehicle pose is invalid")

    effective_minimum = max(settings.minimum_range_m, scan_range_min_m)
    effective_maximum = min(settings.maximum_range_m, scan_range_max_m)
    clusters: list[list[tuple[float, float]]] = []
    current: list[tuple[float, float]] = []

    def finish() -> None:
        if len(current) >= settings.minimum_cluster_points:
            clusters.append(current.copy())
        current.clear()

    for index, distance in enumerate(ranges):
        angle = (
            angle_min_rad
            + index * angle_increment_rad
            + settings.lidar_yaw_rad
        )
        angle = math.atan2(math.sin(angle), math.cos(angle))
        if (
            not math.isfinite(distance)
            or distance < effective_minimum
            or distance > effective_maximum
            or abs(angle) > settings.front_half_angle_rad
        ):
            finish()
            continue
        point = (distance * math.cos(angle), distance * math.sin(angle))
        if point[0] <= 0.0:
            finish()
            continue
        if current and math.hypot(
            point[0] - current[-1][0], point[1] - current[-1][1]
        ) > settings.maximum_cluster_gap_m:
            finish()
        current.append(point)
    finish()

    result: list[tuple[float, float, float]] = []
    for cluster in clusters:
        lidar_x = sum(point[0] for point in cluster) / len(cluster)
        lidar_y = sum(point[1] for point in cluster) / len(cluster)
        visible_radius = max(
            math.hypot(point[0] - lidar_x, point[1] - lidar_y)
            for point in cluster
        )
        radius = max(
            settings.minimum_obstacle_radius_m,
            visible_radius + settings.radius_margin_m,
        )
        base_x = settings.lidar_x_m + lidar_x
        base_y = settings.lidar_y_m + lidar_y
        map_x, map_y = _transform(
            base_x,
            base_y,
            vehicle_pose.x_m,
            vehicle_pose.y_m,
            vehicle_pose.yaw_rad,
        )
        result.append((map_x, map_y, radius))
    return tuple(result)


def clustered_obstacles_in_map(
    ranges: Sequence[float],
    angle_min_rad: float,
    angle_increment_rad: float,
    scan_range_min_m: float,
    scan_range_max_m: float,
    vehicle_pose: VehiclePose,
    settings: MapperSettings,
) -> tuple[tuple[float, float], ...]:
    """Keep the original center-only view for existing map displays."""
    return tuple(
        (x_m, y_m)
        for x_m, y_m, _radius_m in clustered_obstacle_discs_in_map(
            ranges,
            angle_min_rad,
            angle_increment_rad,
            scan_range_min_m,
            scan_range_max_m,
            vehicle_pose,
            settings,
        )
    )
