"""Heading-aware route acquisition and continuity-bounded progress tracking."""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass

from .route import Route


def normalize_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def direction_runs(route: Route) -> tuple[tuple[int, int, int], ...]:
    """Return contiguous waypoint ranges with the same authored direction."""

    runs: list[tuple[int, int, int]] = []
    start = 0
    direction = route.waypoints[0].direction
    for index, waypoint in enumerate(route.waypoints[1:], start=1):
        if waypoint.direction == direction:
            continue
        runs.append((start, index - 1, direction))
        start = index
        direction = waypoint.direction
    runs.append((start, len(route.waypoints) - 1, direction))
    return tuple(runs)


@dataclass(frozen=True)
class RouteProgressMatch:
    run_index: int
    waypoint_index: int
    route_s_m: float
    projection_x_m: float
    projection_y_m: float
    distance_m: float
    heading_error_rad: float
    direction: int
    score: float


def _segment_projection(
    route: Route,
    index: int,
    x_m: float,
    y_m: float,
) -> tuple[float, float, float, float, float]:
    first = route.waypoints[index]
    second = route.waypoints[index + 1]
    dx = second.x_m - first.x_m
    dy = second.y_m - first.y_m
    length_sq = dx * dx + dy * dy
    if length_sq <= 1.0e-12:
        ratio = 0.0
    else:
        ratio = max(
            0.0,
            min(1.0, ((x_m - first.x_m) * dx + (y_m - first.y_m) * dy) / length_sq),
        )
    projection_x = first.x_m + ratio * dx
    projection_y = first.y_m + ratio * dy
    route_s = first.s_m + ratio * (second.s_m - first.s_m)
    distance = math.hypot(x_m - projection_x, y_m - projection_y)
    tangent = math.atan2(dy, dx)
    return route_s, projection_x, projection_y, distance, tangent


def match_route_ahead(
    route: Route,
    x_m: float,
    y_m: float,
    vehicle_yaw_rad: float,
    *,
    forward_offset_m: float = 0.30,
    maximum_heading_error_rad: float = math.radians(60.0),
    heading_weight_m_per_rad: float = 2.0,
) -> RouteProgressMatch | None:
    """Find one forward route point for a fresh start or full-stack restart.

    Every same-direction run is considered once.  Candidates behind the
    vehicle's authored direction of travel are rejected.  Heading is part of
    the score so nearby crossings cannot be confused by GNSS position noise.
    """

    if not all(
        math.isfinite(value)
        for value in (
            x_m,
            y_m,
            vehicle_yaw_rad,
            forward_offset_m,
            maximum_heading_error_rad,
            heading_weight_m_per_rad,
        )
    ):
        return None
    runs = direction_runs(route)
    s_values = [waypoint.s_m for waypoint in route.waypoints]
    candidates: list[RouteProgressMatch] = []
    for run_index, (run_start, run_end, direction) in enumerate(runs):
        if run_end <= run_start:
            continue
        run_end_s = route.waypoints[run_end].s_m
        motion_yaw = (
            vehicle_yaw_rad
            if direction > 0
            else normalize_angle(vehicle_yaw_rad + math.pi)
        )
        for segment_index in range(run_start, run_end):
            (
                projection_s,
                projection_x,
                projection_y,
                distance,
                tangent,
            ) = _segment_projection(route, segment_index, x_m, y_m)
            expected_vehicle_yaw = (
                tangent if direction > 0 else normalize_angle(tangent + math.pi)
            )
            heading_error = abs(
                normalize_angle(vehicle_yaw_rad - expected_vehicle_yaw)
            )
            if heading_error > max(0.0, maximum_heading_error_rad):
                continue
            target_s = min(
                run_end_s,
                max(route.waypoints[run_start].s_m, projection_s)
                + max(0.0, forward_offset_m),
            )
            target_x, target_y = route.position_at_s(target_s)
            forward_distance = (
                (target_x - x_m) * math.cos(motion_yaw)
                + (target_y - y_m) * math.sin(motion_yaw)
            )
            if forward_distance < -1.0e-6:
                continue
            waypoint_index = min(
                run_end,
                max(
                    run_start,
                    bisect.bisect_left(
                        s_values,
                        target_s,
                        run_start,
                        run_end + 1,
                    ),
                ),
            )
            candidates.append(
                RouteProgressMatch(
                    run_index=run_index,
                    waypoint_index=waypoint_index,
                    route_s_m=target_s,
                    projection_x_m=projection_x,
                    projection_y_m=projection_y,
                    distance_m=distance,
                    heading_error_rad=heading_error,
                    direction=direction,
                    score=(
                        distance
                        + max(0.0, heading_weight_m_per_rad) * heading_error
                    ),
                )
            )
    return min(candidates, key=lambda candidate: candidate.score) if candidates else None


def continuous_search_bounds(
    route: Route,
    current_index: int,
    run_start_index: int,
    run_end_index: int,
    forward_search_m: float,
) -> tuple[int, int]:
    """Return a monotonic local search window inside the current motion run."""

    start = max(run_start_index, min(current_index, run_end_index))
    maximum_s = route.waypoints[start].s_m + max(0.10, forward_search_m)
    end = start
    while end < run_end_index and route.waypoints[end + 1].s_m <= maximum_s:
        end += 1
    if end == start and end < run_end_index:
        end += 1
    return start, end


def continuous_nearest_index(
    route: Route,
    x_m: float,
    y_m: float,
    current_index: int,
    run_start_index: int,
    run_end_index: int,
    forward_search_m: float,
) -> int:
    """Advance only within a small window; never jump backward or across the course."""

    start, end = continuous_search_bounds(
        route,
        current_index,
        run_start_index,
        run_end_index,
        forward_search_m,
    )
    return route.nearest_index_between(x_m, y_m, start, end)


def _bounded_route_heading(
    route: Route,
    route_s_m: float,
    run_start_index: int,
    run_end_index: int,
    baseline_m: float = 1.0,
) -> float:
    """Estimate a tangent without crossing a forward/reverse cusp."""

    start_s = route.waypoints[run_start_index].s_m
    end_s = route.waypoints[run_end_index].s_m
    baseline = min(max(1.0e-3, baseline_m), max(1.0e-3, end_s - start_s))
    center = max(start_s, min(route_s_m, end_s))
    first_s = max(start_s, center - 0.5 * baseline)
    second_s = min(end_s, first_s + baseline)
    first_s = max(start_s, second_s - baseline)
    first = route.position_at_s(first_s)
    second = route.position_at_s(second_s)
    return math.atan2(second[1] - first[1], second[0] - first[0])


def match_saved_progress(
    route: Route,
    x_m: float,
    y_m: float,
    vehicle_yaw_rad: float,
    *,
    saved_route_s_m: float,
    saved_run_index: int,
    position_tolerance_m: float = 2.0,
    maximum_heading_error_rad: float = math.radians(60.0),
    forward_search_m: float = 3.0,
) -> RouteProgressMatch | None:
    """Resume one known motion run when the vehicle remains near its checkpoint.

    Position and body heading cannot distinguish two parking manoeuvre stages
    when a reverse-in path overlaps its forward-out path.  The saved run is
    therefore authoritative only while the new pose is still close to the
    checkpoint.  If the vehicle was moved elsewhere, the caller falls back to
    a global heading-aware acquisition.
    """

    if not all(
        math.isfinite(value)
        for value in (
            x_m,
            y_m,
            vehicle_yaw_rad,
            saved_route_s_m,
            position_tolerance_m,
            maximum_heading_error_rad,
            forward_search_m,
        )
    ):
        return None
    runs = direction_runs(route)
    if saved_run_index < 0 or saved_run_index >= len(runs):
        return None
    run_start, run_end, direction = runs[saved_run_index]
    saved_index = min(
        range(run_start, run_end + 1),
        key=lambda index: abs(route.waypoints[index].s_m - saved_route_s_m),
    )
    saved_point = route.waypoints[saved_index]
    distance = math.hypot(x_m - saved_point.x_m, y_m - saved_point.y_m)
    if distance > max(0.0, position_tolerance_m):
        return None
    route_heading = _bounded_route_heading(
        route,
        saved_point.s_m,
        run_start,
        run_end,
    )
    expected_vehicle_yaw = route_heading
    if direction < 0:
        expected_vehicle_yaw = normalize_angle(route_heading + math.pi)
    heading_error = abs(normalize_angle(vehicle_yaw_rad - expected_vehicle_yaw))
    if heading_error > max(0.0, maximum_heading_error_rad):
        return None
    selected = continuous_nearest_index(
        route,
        x_m,
        y_m,
        saved_index,
        run_start,
        run_end,
        forward_search_m,
    )
    point = route.waypoints[selected]
    return RouteProgressMatch(
        run_index=saved_run_index,
        waypoint_index=selected,
        route_s_m=point.s_m,
        projection_x_m=point.x_m,
        projection_y_m=point.y_m,
        distance_m=distance,
        heading_error_rad=heading_error,
        direction=direction,
        score=distance,
    )
