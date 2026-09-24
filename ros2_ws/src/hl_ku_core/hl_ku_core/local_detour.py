"""Plan short detours around detected circular map obstacles.

The global route remains authoritative. This pure geometry module never
publishes a drive command; the live mission controller validates candidates
before following them.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass

from .route import Route


@dataclass(frozen=True)
class CircleObstacle:
    x_m: float
    y_m: float
    radius_m: float


@dataclass(frozen=True)
class DetourSettings:
    vehicle_half_width_m: float = 0.35
    safety_margin_m: float = 0.20
    corridor_half_width_m: float = 1.70
    preview_distance_m: float = 18.0
    # Start the nominal maneuver this far before the blocking obstacle.
    entry_length_m: float = 5.0
    # If an obstacle is first seen late, shorten the entry instead of
    # rejecting it outright. Geometry/curvature checks still have final say.
    minimum_entry_length_m: float = 1.0
    # A frontal 2D scan sees the obstacle face, not its rear. Treat each
    # detected obstacle as continuing this far along the route.
    assumed_obstacle_length_m: float = 1.3
    exit_length_m: float = 5.0
    minimum_entry_ahead_m: float = 0.0
    sample_step_m: float = 0.20
    maximum_curvature_per_m: float = 0.75


@dataclass(frozen=True)
class DetourResult:
    status: str
    points: tuple[tuple[float, float], ...] = ()
    start_s_m: float | None = None
    end_s_m: float | None = None
    side: str = ""
    obstacle_rear_s_m: float | None = None


def _point_segment_distance(
    px: float, py: float, ax: float, ay: float, bx: float, by: float
) -> tuple[float, float]:
    dx, dy = bx - ax, by - ay
    length_sq = dx * dx + dy * dy
    ratio = 0.0 if length_sq < 1.0e-12 else max(
        0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq)
    )
    return math.hypot(px - ax - ratio * dx, py - ay - ratio * dy), ratio


def _samples(start: float, end: float, step: float) -> tuple[float, ...]:
    count = max(2, math.ceil((end - start) / step))
    return tuple(start + (end - start) * index / count for index in range(count + 1))


def _smoothstep(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return value * value * value * (value * (6.0 * value - 15.0) + 10.0)


def _closest_route_station(
    route: Route,
    obstacle: CircleObstacle,
    start_s_m: float,
    end_s_m: float,
    sample_step_m: float,
) -> tuple[float, float]:
    samples = _samples(start_s_m, end_s_m, sample_step_m)
    centers = tuple(route.position_at_s(value) for value in samples)
    return min(
        (
            (
                distance,
                samples[index] + ratio * (samples[index + 1] - samples[index]),
            )
            for index, (first, second) in enumerate(zip(centers, centers[1:]))
            for distance, ratio in [
                _point_segment_distance(obstacle.x_m, obstacle.y_m, *first, *second)
            ]
        ),
        key=lambda item: item[0],
    )


def _extend_obstacles_along_route(
    route: Route,
    obstacles: tuple[CircleObstacle, ...],
    end_s_m: float,
    length_m: float,
    sample_step_m: float,
) -> tuple[CircleObstacle, ...]:
    """Approximate each visible obstacle face as a route-aligned capsule."""
    route_start_s_m = route.waypoints[0].s_m
    route_end_s_m = route.waypoints[-1].s_m
    # Search from the route start so an obstacle retained while the vehicle
    # passes it is not incorrectly projected forward onto current progress.
    search_start_s_m = route_start_s_m
    search_end_s_m = min(route_end_s_m, end_s_m)
    if search_end_s_m <= search_start_s_m:
        return obstacles

    expanded: list[CircleObstacle] = []
    for obstacle in obstacles:
        _, front_s_m = _closest_route_station(
            route,
            obstacle,
            search_start_s_m,
            search_end_s_m,
            sample_step_m,
        )
        front_x_m, front_y_m = route.position_at_s(front_s_m)
        front_heading_rad = route.heading_at_s(front_s_m, 1.0)
        lateral_m = (
            -math.sin(front_heading_rad) * (obstacle.x_m - front_x_m)
            + math.cos(front_heading_rad) * (obstacle.y_m - front_y_m)
        )
        rear_s_m = min(route_end_s_m, front_s_m + length_m)
        for route_s_m in _samples(front_s_m, rear_s_m, sample_step_m):
            route_x_m, route_y_m = route.position_at_s(route_s_m)
            heading_rad = route.heading_at_s(route_s_m, 1.0)
            expanded.append(
                CircleObstacle(
                    route_x_m - math.sin(heading_rad) * lateral_m,
                    route_y_m + math.cos(heading_rad) * lateral_m,
                    obstacle.radius_m,
                )
            )
    return tuple(expanded)


def _path_clear(
    points: tuple[tuple[float, float], ...],
    obstacles: tuple[CircleObstacle, ...],
    clearance_m: float,
) -> bool:
    for obstacle in obstacles:
        required = obstacle.radius_m + clearance_m
        for first, second in zip(points, points[1:]):
            distance, _ = _point_segment_distance(
                obstacle.x_m, obstacle.y_m, *first, *second
            )
            if distance < required - 1.0e-9:
                return False
    return True


def _maximum_curvature(points: tuple[tuple[float, float], ...]) -> float:
    maximum = 0.0
    for first, middle, last in zip(points, points[1:], points[2:]):
        a = math.dist(first, middle)
        b = math.dist(middle, last)
        c = math.dist(first, last)
        if min(a, b, c) < 1.0e-6:
            return math.inf
        cross = abs(
            (middle[0] - first[0]) * (last[1] - first[1])
            - (middle[1] - first[1]) * (last[0] - first[0])
        )
        maximum = max(maximum, 2.0 * cross / (a * b * c))
    return maximum


def _path_lateral_offset_at_s(
    route: Route,
    result: DetourResult,
    route_s_m: float,
    sample_step_m: float,
) -> float:
    """Interpolate the signed route-relative offset of a detour candidate."""
    if (
        result.start_s_m is None
        or result.end_s_m is None
        or not result.points
    ):
        return 0.0
    stations = _samples(result.start_s_m, result.end_s_m, sample_step_m)
    if len(stations) != len(result.points):
        return 0.0
    station = min(result.end_s_m, max(result.start_s_m, route_s_m))
    right = min(len(stations) - 1, bisect.bisect_right(stations, station))
    left = max(0, right - 1)
    span = stations[right] - stations[left]
    ratio = 0.0 if span <= 1.0e-9 else (station - stations[left]) / span
    point_x_m = result.points[left][0] + ratio * (
        result.points[right][0] - result.points[left][0]
    )
    point_y_m = result.points[left][1] + ratio * (
        result.points[right][1] - result.points[left][1]
    )
    route_x_m, route_y_m = route.position_at_s(station)
    heading_rad = route.heading_at_s(station, 1.0)
    return (
        -math.sin(heading_rad) * (point_x_m - route_x_m)
        + math.cos(heading_rad) * (point_y_m - route_y_m)
    )


def _blocking_obstacle(
    route: Route,
    progress_s_m: float,
    obstacles: tuple[CircleObstacle, ...],
    settings: DetourSettings,
) -> tuple[float, CircleObstacle] | None:
    end = min(route.waypoints[-1].s_m, progress_s_m + settings.preview_distance_m)
    clearance = settings.vehicle_half_width_m + settings.safety_margin_m
    blockers: list[tuple[float, CircleObstacle]] = []
    for obstacle in obstacles:
        closest = _closest_route_station(
            route,
            obstacle,
            progress_s_m,
            end,
            settings.sample_step_m,
        )
        if closest[0] < obstacle.radius_m + clearance:
            blockers.append((closest[1], obstacle))
    return min(blockers, key=lambda item: item[0]) if blockers else None


def _allowed_section(route: Route, start_s_m: float, end_s_m: float) -> bool:
    values = [point.s_m for point in route.waypoints]
    start_index = max(0, bisect.bisect_right(values, start_s_m) - 1)
    end_index = min(len(values) - 1, bisect.bisect_left(values, end_s_m))
    return all(
        point.mission == "S_OBSTACLE"
        and point.direction == 1
        and point.target_speed_mps > 0.0
        for point in route.waypoints[start_index : end_index + 1]
    )


def plan_local_detour(
    route: Route,
    progress_s_m: float,
    obstacles: tuple[CircleObstacle, ...],
    settings: DetourSettings = DetourSettings(),
    initial_lateral_offset_m: float | None = None,
) -> DetourResult:
    """Return a bounded map-frame path, or a reason not to use one.

    Obstacles must already be transformed into the route's ``map`` frame.
    Unknown space is not represented here, so this is *not* a drive approval.
    """
    if not all(
        math.isfinite(value) and value >= 0.0
        for value in (
            settings.vehicle_half_width_m,
            settings.safety_margin_m,
            settings.corridor_half_width_m,
            settings.preview_distance_m,
            settings.entry_length_m,
            settings.minimum_entry_length_m,
            settings.assumed_obstacle_length_m,
            settings.exit_length_m,
            settings.minimum_entry_ahead_m,
            settings.sample_step_m,
            settings.maximum_curvature_per_m,
        )
    ) or min(
        settings.preview_distance_m,
        settings.entry_length_m,
        settings.minimum_entry_length_m,
        settings.assumed_obstacle_length_m,
        settings.exit_length_m,
        settings.sample_step_m,
        settings.maximum_curvature_per_m,
    ) <= 0.0:
        return DetourResult("INVALID_SETTINGS")
    if not math.isfinite(progress_s_m) or not (
        route.waypoints[0].s_m <= progress_s_m < route.waypoints[-1].s_m
    ):
        return DetourResult("INVALID_PROGRESS")
    if initial_lateral_offset_m is not None and not math.isfinite(
        initial_lateral_offset_m
    ):
        return DetourResult("INVALID_SETTINGS")
    if any(
        not all(math.isfinite(value) for value in (item.x_m, item.y_m, item.radius_m))
        or item.radius_m < 0.0
        for item in obstacles
    ):
        return DetourResult("INVALID_OBSTACLE")
    if not obstacles:
        return DetourResult("NO_OBSTACLE")
    blocker = _blocking_obstacle(route, progress_s_m, obstacles, settings)
    if blocker is None:
        return DetourResult("ROUTE_CLEAR")
    obstacle_s, _ = blocker
    available_entry_m = obstacle_s - progress_s_m
    if available_entry_m < settings.minimum_entry_length_m:
        return DetourResult("TOO_CLOSE_OR_ROUTE_END")
    actual_entry_m = min(settings.entry_length_m, available_entry_m)
    continuing = initial_lateral_offset_m is not None
    start_s = progress_s_m if continuing else obstacle_s - actual_entry_m
    start_offset_m = initial_lateral_offset_m if continuing else 0.0
    obstacle_rear_s = obstacle_s + settings.assumed_obstacle_length_m
    end_s = obstacle_rear_s + settings.exit_length_m
    if (
        start_s < progress_s_m + settings.minimum_entry_ahead_m
        or end_s > route.waypoints[-1].s_m
    ):
        return DetourResult("TOO_CLOSE_OR_ROUTE_END")
    if not _allowed_section(route, start_s, end_s):
        return DetourResult("OUTSIDE_S_OBSTACLE")
    maximum_offset = (
        settings.corridor_half_width_m
        - settings.vehicle_half_width_m
        - settings.safety_margin_m
    )
    if maximum_offset <= 0.0:
        return DetourResult("NO_CORRIDOR")
    if abs(start_offset_m) > maximum_offset + 1.0e-9:
        return DetourResult("NO_CORRIDOR")
    sampled_s = _samples(start_s, end_s, settings.sample_step_m)
    clearance = settings.vehicle_half_width_m + settings.safety_margin_m
    collision_obstacles = _extend_obstacles_along_route(
        route,
        obstacles,
        min(route.waypoints[-1].s_m, end_s),
        settings.assumed_obstacle_length_m,
        settings.sample_step_m,
    )
    amplitudes = sorted(
        {maximum_offset, *(0.05 * index for index in range(1, math.ceil(maximum_offset / 0.05)))}
    )
    for amplitude in amplitudes:
        for side, sign in (("LEFT", 1.0), ("RIGHT", -1.0)):
            points = []
            for route_s in sampled_s:
                if route_s <= obstacle_s:
                    fraction = (route_s - start_s) / (obstacle_s - start_s)
                    blend = _smoothstep(fraction)
                    offset = start_offset_m + blend * (
                        sign * amplitude - start_offset_m
                    )
                elif route_s <= obstacle_rear_s:
                    offset = sign * amplitude
                else:
                    fraction = (end_s - route_s) / (end_s - obstacle_rear_s)
                    offset = sign * amplitude * _smoothstep(fraction)
                x, y = route.position_at_s(route_s)
                heading = route.heading_at_s(route_s, 1.0)
                points.append(
                    (x - math.sin(heading) * offset, y + math.cos(heading) * offset)
                )
            candidate = tuple(points)
            if not _path_clear(candidate, collision_obstacles, clearance):
                continue
            if _maximum_curvature(candidate) > settings.maximum_curvature_per_m:
                continue
            return DetourResult(
                "CANDIDATE",
                candidate,
                start_s,
                end_s,
                side,
                obstacle_rear_s,
            )
    return DetourResult("NO_SAFE_DETOUR")
