"""Pure, bounded steering selection for an already validated local detour.

This module does not decide whether a candidate is safe to drive.  The caller
must require fresh localization, perception, and a live planner candidate.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .control import normalize_angle, stanley_steering


@dataclass(frozen=True)
class DetourFollowSettings:
    stanley_gain: float = 0.65
    stanley_softening_mps: float = 0.65
    control_point_offset_m: float = 0.58
    heading_baseline_m: float = 1.0
    maximum_steering_rad: float = 0.30
    completion_radius_m: float = 1.0
    entry_activation_m: float = 0.80
    maximum_lateral_error_m: float = 0.70
    maximum_heading_error_rad: float = 1.0
    maximum_segment_m: float = 0.50
    maximum_path_length_m: float = 30.0


@dataclass(frozen=True)
class DetourFollowResult:
    state: str
    steering_rad: float = 0.0


def follow_detour(
    points: tuple[tuple[float, float], ...],
    x_m: float,
    y_m: float,
    yaw_rad: float,
    settings: DetourFollowSettings = DetourFollowSettings(),
    speed_mps: float = 0.0,
) -> DetourFollowResult:
    """Return APPROACH, TRACK, COMPLETE, or INVALID for a map-frame path."""
    values = (
        x_m, y_m, yaw_rad, speed_mps, settings.stanley_gain,
        settings.stanley_softening_mps, settings.control_point_offset_m,
        settings.heading_baseline_m, settings.maximum_steering_rad,
        settings.completion_radius_m, settings.entry_activation_m,
        settings.maximum_lateral_error_m, settings.maximum_heading_error_rad,
        settings.maximum_segment_m, settings.maximum_path_length_m,
    )
    if (
        len(points) < 3
        or not all(math.isfinite(value) for value in values)
        or not all(math.isfinite(value) for point in points for value in point)
        or min(
            settings.stanley_softening_mps, settings.heading_baseline_m,
            settings.maximum_steering_rad, settings.completion_radius_m,
            settings.entry_activation_m,
            settings.maximum_lateral_error_m, settings.maximum_heading_error_rad,
            settings.maximum_segment_m, settings.maximum_path_length_m,
        ) <= 0.0
        or settings.stanley_gain < 0.0
        or settings.control_point_offset_m < 0.0
    ):
        return DetourFollowResult("INVALID")

    lengths = tuple(math.dist(first, second) for first, second in zip(points, points[1:]))
    if (
        any(length < 0.01 or length > settings.maximum_segment_m for length in lengths)
        or sum(lengths) > settings.maximum_path_length_m
    ):
        return DetourFollowResult("INVALID")

    first_dx = points[1][0] - points[0][0]
    first_dy = points[1][1] - points[0][1]
    first_length = lengths[0]
    from_start_x = x_m - points[0][0]
    from_start_y = y_m - points[0][1]
    along_start = (from_start_x * first_dx + from_start_y * first_dy) / first_length
    start_lateral = abs(first_dx * from_start_y - first_dy * from_start_x) / first_length
    if along_start < -settings.entry_activation_m:
        if (
            start_lateral > settings.maximum_lateral_error_m
            or abs(normalize_angle(math.atan2(first_dy, first_dx) - yaw_rad))
            > settings.maximum_heading_error_rad
        ):
            return DetourFollowResult("INVALID")
        return DetourFollowResult("APPROACH")

    last_dx = points[-1][0] - points[-2][0]
    last_dy = points[-1][1] - points[-2][1]
    last_length = lengths[-1]
    past_end_x = x_m - points[-1][0]
    past_end_y = y_m - points[-1][1]
    along_end = (past_end_x * last_dx + past_end_y * last_dy) / last_length
    end_lateral = abs(last_dx * past_end_y - last_dy * past_end_x) / last_length
    best: tuple[float, int, float] | None = None
    control_best: tuple[float, int, float] | None = None
    control_x = x_m + settings.control_point_offset_m * math.cos(yaw_rad)
    control_y = y_m + settings.control_point_offset_m * math.sin(yaw_rad)
    cumulative = 0.0
    distances_along = [0.0]
    for index, ((ax, ay), (bx, by), length) in enumerate(
        zip(points, points[1:], lengths)
    ):
        ux = (bx - ax) / length
        uy = (by - ay) / length
        ratio = max(0.0, min(1.0, ((x_m - ax) * ux + (y_m - ay) * uy) / length))
        px = ax + ratio * (bx - ax)
        py = ay + ratio * (by - ay)
        candidate = (math.hypot(x_m - px, y_m - py), index, ratio)
        if best is None or candidate < best:
            best = candidate
        control_ratio = max(
            0.0, min(1.0, ((control_x - ax) * ux + (control_y - ay) * uy) / length)
        )
        control_px = ax + control_ratio * (bx - ax)
        control_py = ay + control_ratio * (by - ay)
        control_candidate = (
            math.hypot(control_x - control_px, control_y - control_py),
            index,
            control_ratio,
        )
        if control_best is None or control_candidate < control_best:
            control_best = control_candidate
        cumulative += length
        distances_along.append(cumulative)
    assert best is not None and control_best is not None
    if max(best[0], control_best[0]) > settings.maximum_lateral_error_m:
        return DetourFollowResult("INVALID")
    segment_index = best[1]
    if (
        segment_index >= len(lengths) - 3
        and along_end >= 0.0
        and math.hypot(past_end_x, past_end_y) <= settings.completion_radius_m
        and end_lateral <= settings.maximum_lateral_error_m
    ):
        return DetourFollowResult("COMPLETE")
    heading = math.atan2(
        points[segment_index + 1][1] - points[segment_index][1],
        points[segment_index + 1][0] - points[segment_index][0],
    )
    if abs(normalize_angle(heading - yaw_rad)) > settings.maximum_heading_error_rad:
        return DetourFollowResult("INVALID")
    def point_at(distance_m: float) -> tuple[float, float]:
        for index, length in enumerate(lengths):
            if distance_m <= distances_along[index + 1]:
                fraction = max(0.0, min(1.0, (distance_m - distances_along[index]) / length))
                return (
                    points[index][0] + fraction * (points[index + 1][0] - points[index][0]),
                    points[index][1] + fraction * (points[index + 1][1] - points[index][1]),
                )
        return points[-1]

    control_index = control_best[1]
    control_ratio = control_best[2]
    reference_x = points[control_index][0] + control_ratio * (
        points[control_index + 1][0] - points[control_index][0]
    )
    reference_y = points[control_index][1] + control_ratio * (
        points[control_index + 1][1] - points[control_index][1]
    )
    reference_s = distances_along[control_index] + control_ratio * lengths[control_index]
    start_s = max(0.0, reference_s - settings.heading_baseline_m * 0.5)
    end_s = min(cumulative, start_s + settings.heading_baseline_m)
    start_s = max(0.0, end_s - settings.heading_baseline_m)
    first = point_at(start_s)
    last = point_at(end_s)
    path_yaw = math.atan2(last[1] - first[1], last[0] - first[0])
    if abs(normalize_angle(path_yaw - yaw_rad)) > settings.maximum_heading_error_rad:
        return DetourFollowResult("INVALID")
    terms = stanley_steering(
        x_m, y_m, yaw_rad,
        reference_x, reference_y,
        reference_x + math.cos(path_yaw),
        reference_y + math.sin(path_yaw),
        speed_mps,
        settings.stanley_gain,
        settings.stanley_softening_mps,
        settings.control_point_offset_m,
        settings.maximum_steering_rad,
    )
    return DetourFollowResult("TRACK", terms.steering_rad)
