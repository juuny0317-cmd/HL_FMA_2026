#!/usr/bin/env python3
"""Apply the reviewed hill stop and reverse parallel-parking maneuver.

The script is intentionally deterministic and idempotent.  It keeps the
existing fully faired route outside the parallel-parking zone, replaces that
zone with a tangent-constrained forward bypass plus reverse/forward parking
tail, updates the branch-aware masters, and remaps FSM events.

The generated CSV files remain geometry-review previews.  Target speeds stay
at zero until vehicle footprint and field-clearance validation are complete.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from collections import Counter
from pathlib import Path
from typing import Iterable


ROOT = Path(__file__).resolve().parents[1]
ROUTE_DIR = (
    ROOT
    / "ros2_ws"
    / "src"
    / "hl_ku_core"
    / "routes"
    / "course_07_parking_overlay"
)

HILL_HOLD_SEC = 3.0
BYPASS_CONTROL_LENGTH_M = 6.0
PARKING_STRAIGHT_TAIL_M = 1.40
PARKING_APPROACH_BASELINE_M = 1.00
PARKING_CURVE_DEGREE = 7
PARKING_CURVE_DERIVATIVE_SCALE = 0.90
PARKING_CURVE_REGULARIZATION = 3.0
PARKING_MAX_CURVATURE_INV_M = 0.45
P2_TEMPLATE_ALIGNMENT_MAX_ERROR_M = 0.25
P2_EXIT_REJOIN_DISTANCE_M = 4.0
P2_EXIT_REJOIN_START_CONTROL_M = 1.6
P2_EXIT_REJOIN_END_CONTROL_M = 1.4
P2_EXIT_REJOIN_TANGENT_BASELINE_M = 1.0
P2_EXIT_REJOIN_MAX_DEVIATION_M = 0.35
VEHICLE_WHEELBASE_M = 0.58
VEHICLE_MAX_STEERING_RAD = 0.48
FINAL_SPACING_M = 0.10
MASTER_SPACING_M = 0.30

PARKING = {
    1: {
        "entry_source": 106,
        "reverse_start_source": 143,
        "park_complete_source": 131,
        "reverse_depth_m": 9.46,
    },
    2: {
        "entry_source": 146,
        "legacy_reverse_start_source": 191,
        "template_option": 1,
        "park_complete_source": 167,
    },
}


def read_csv(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        return list(reader.fieldnames or []), list(reader)


def write_csv(path: Path, fieldnames: list[str], rows: Iterable[dict[str, object]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=fieldnames,
            extrasaction="ignore",
            lineterminator="\n",
        )
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})


def point(row: dict[str, str]) -> tuple[float, float]:
    return float(row["x_m"]), float(row["y_m"])


def distance(first: tuple[float, float], second: tuple[float, float]) -> float:
    return math.hypot(second[0] - first[0], second[1] - first[1])


def unit(vector: tuple[float, float]) -> tuple[float, float]:
    norm = math.hypot(*vector)
    if norm <= 1e-9:
        raise ValueError("zero-length tangent")
    return vector[0] / norm, vector[1] / norm


def left_normal(vector: tuple[float, float]) -> tuple[float, float]:
    """Return the unit vector 90 degrees left of ``vector``."""

    normalized = unit(vector)
    return -normalized[1], normalized[0]


def trailing_tangent(
    points: list[tuple[float, float]], baseline_m: float
) -> tuple[float, float]:
    """Estimate the final tangent over an exact trailing arc-length baseline."""

    if len(points) < 2:
        raise ValueError("at least two points are required for a trailing tangent")
    end = points[-1]
    remaining = baseline_m
    current = end
    for previous in reversed(points[:-1]):
        segment = distance(previous, current)
        if segment <= 1e-9:
            current = previous
            continue
        if segment >= remaining:
            ratio = remaining / segment
            anchor = (
                current[0] + ratio * (previous[0] - current[0]),
                current[1] + ratio * (previous[1] - current[1]),
            )
            return end[0] - anchor[0], end[1] - anchor[1]
        remaining -= segment
        current = previous
    return end[0] - points[0][0], end[1] - points[0][1]


def split_tokens(value: str) -> list[str]:
    return [token.strip() for token in re.split(r"[|;]", value or "") if token.strip()]


def resample_polyline(
    points: list[tuple[float, float]], spacing_m: float
) -> list[tuple[float, float]]:
    if len(points) < 2:
        return points[:]
    cumulative = [0.0]
    for first, second in zip(points, points[1:]):
        cumulative.append(cumulative[-1] + distance(first, second))
    total = cumulative[-1]
    if total <= 1e-9:
        return [points[0]]
    targets = [index * spacing_m for index in range(int(total / spacing_m) + 1)]
    if total - targets[-1] > 1e-7:
        targets.append(total)
    else:
        targets[-1] = total
    result: list[tuple[float, float]] = []
    segment = 0
    for target in targets:
        while segment + 1 < len(cumulative) and cumulative[segment + 1] < target:
            segment += 1
        if segment + 1 >= len(points):
            result.append(points[-1])
            continue
        span = cumulative[segment + 1] - cumulative[segment]
        ratio = 0.0 if span <= 1e-9 else (target - cumulative[segment]) / span
        first, second = points[segment], points[segment + 1]
        result.append(
            (
                first[0] + ratio * (second[0] - first[0]),
                first[1] + ratio * (second[1] - first[1]),
            )
        )
    return result


def project_to_polyline_prefix(
    points: list[tuple[float, float]], target: tuple[float, float]
) -> tuple[tuple[float, float], list[tuple[float, float]], float]:
    """Project ``target`` onto a polyline and return its exact prefix."""

    if len(points) < 2:
        raise ValueError("at least two points are required for projection")
    best: tuple[float, int, tuple[float, float]] | None = None
    for index, (first, second) in enumerate(zip(points, points[1:])):
        vector = (second[0] - first[0], second[1] - first[1])
        squared_length = vector[0] ** 2 + vector[1] ** 2
        if squared_length <= 1e-12:
            ratio = 0.0
        else:
            ratio = max(
                0.0,
                min(
                    1.0,
                    (
                        (target[0] - first[0]) * vector[0]
                        + (target[1] - first[1]) * vector[1]
                    )
                    / squared_length,
                ),
            )
        projected = (
            first[0] + ratio * vector[0],
            first[1] + ratio * vector[1],
        )
        error = distance(projected, target)
        if best is None or error < best[0]:
            best = (error, index, projected)
    assert best is not None
    error, segment_index, projected = best
    prefix = list(points[: segment_index + 1])
    if distance(prefix[-1], projected) > 1e-9:
        prefix.append(projected)
    else:
        prefix[-1] = projected
    return projected, prefix, error


def project_to_polyline_suffix(
    points: list[tuple[float, float]], target: tuple[float, float]
) -> tuple[tuple[float, float], list[tuple[float, float]], float]:
    """Project ``target`` onto a polyline and return its exact suffix."""

    if len(points) < 2:
        raise ValueError("at least two points are required for projection")
    best: tuple[float, int, tuple[float, float]] | None = None
    for index, (first, second) in enumerate(zip(points, points[1:])):
        vector = (second[0] - first[0], second[1] - first[1])
        squared_length = vector[0] ** 2 + vector[1] ** 2
        ratio = (
            0.0
            if squared_length <= 1e-12
            else max(
                0.0,
                min(
                    1.0,
                    (
                        (target[0] - first[0]) * vector[0]
                        + (target[1] - first[1]) * vector[1]
                    )
                    / squared_length,
                ),
            )
        )
        projected = (
            first[0] + ratio * vector[0],
            first[1] + ratio * vector[1],
        )
        error = distance(projected, target)
        if best is None or error < best[0]:
            best = (error, index, projected)
    assert best is not None
    error, segment_index, projected = best
    suffix = [projected]
    if distance(projected, points[segment_index + 1]) > 1e-9:
        suffix.append(points[segment_index + 1])
    suffix.extend(points[segment_index + 2 :])
    return projected, suffix, error


def polyline_length(points: list[tuple[float, float]]) -> float:
    return sum(distance(first, second) for first, second in zip(points, points[1:]))


def distance_to_polyline(
    target: tuple[float, float], points: list[tuple[float, float]]
) -> float:
    """Return the shortest distance from a point to a polyline."""

    if len(points) < 2:
        raise ValueError("at least two points are required for distance")
    minimum = math.inf
    for first, second in zip(points, points[1:]):
        vector = (second[0] - first[0], second[1] - first[1])
        squared_length = vector[0] ** 2 + vector[1] ** 2
        ratio = (
            0.0
            if squared_length <= 1e-12
            else max(
                0.0,
                min(
                    1.0,
                    (
                        (target[0] - first[0]) * vector[0]
                        + (target[1] - first[1]) * vector[1]
                    )
                    / squared_length,
                ),
            )
        )
        projected = (
            first[0] + ratio * vector[0],
            first[1] + ratio * vector[1],
        )
        minimum = min(minimum, distance(target, projected))
    return minimum


def point_at_polyline_distance(
    points: list[tuple[float, float]], distance_m: float
) -> tuple[float, float]:
    """Return the coordinate at an arc distance from a polyline's start."""

    if not points:
        raise ValueError("polyline cannot be empty")
    remaining = max(0.0, min(polyline_length(points), distance_m))
    for first, second in zip(points, points[1:]):
        segment = distance(first, second)
        if segment >= remaining:
            ratio = 0.0 if segment <= 1e-12 else remaining / segment
            return (
                first[0] + ratio * (second[0] - first[0]),
                first[1] + ratio * (second[1] - first[1]),
            )
        remaining -= segment
    return points[-1]


def split_polyline_at_distance(
    points: list[tuple[float, float]], distance_m: float
) -> tuple[tuple[float, float], list[tuple[float, float]]]:
    """Split a polyline at an arc distance and return the exact suffix."""

    if len(points) < 2:
        raise ValueError("at least two points are required for splitting")
    remaining = max(0.0, min(polyline_length(points), distance_m))
    for index, (first, second) in enumerate(zip(points, points[1:])):
        segment = distance(first, second)
        if segment >= remaining:
            ratio = 0.0 if segment <= 1e-12 else remaining / segment
            split = (
                first[0] + ratio * (second[0] - first[0]),
                first[1] + ratio * (second[1] - first[1]),
            )
            suffix = [split]
            if distance(split, second) > 1e-9:
                suffix.append(second)
            suffix.extend(points[index + 2 :])
            return split, suffix
        remaining -= segment
    return points[-1], [points[-1]]


def tangent_at_polyline_distance(
    points: list[tuple[float, float]],
    distance_m: float,
    baseline_m: float,
) -> tuple[float, float]:
    """Estimate a tangent centered at an arc distance on a polyline."""

    half = baseline_m / 2.0
    before = point_at_polyline_distance(points, distance_m - half)
    after = point_at_polyline_distance(points, distance_m + half)
    return after[0] - before[0], after[1] - before[1]


def cubic_connector(
    start: tuple[float, float],
    end: tuple[float, float],
    start_tangent: tuple[float, float],
    end_tangent: tuple[float, float],
    start_control_m: float,
    end_control_m: float,
    spacing_m: float,
) -> list[tuple[float, float]]:
    """Build a cubic connector with independently sized endpoint controls."""

    start_unit = unit(start_tangent)
    end_unit = unit(end_tangent)
    control_1 = (
        start[0] + start_control_m * start_unit[0],
        start[1] + start_control_m * start_unit[1],
    )
    control_2 = (
        end[0] - end_control_m * end_unit[0],
        end[1] - end_control_m * end_unit[1],
    )
    dense: list[tuple[float, float]] = []
    for index in range(4001):
        parameter = index / 4000.0
        complement = 1.0 - parameter
        dense.append(
            (
                complement**3 * start[0]
                + 3.0 * complement**2 * parameter * control_1[0]
                + 3.0 * complement * parameter**2 * control_2[0]
                + parameter**3 * end[0],
                complement**3 * start[1]
                + 3.0 * complement**2 * parameter * control_1[1]
                + 3.0 * complement * parameter**2 * control_2[1]
                + parameter**3 * end[1],
            )
        )
    return resample_polyline(dense, spacing_m)


def p2_post_exit_path(
    template_bypass: list[tuple[float, float]],
    forward_tail: list[tuple[float, float]],
    spacing_m: float,
) -> list[tuple[float, float]]:
    """Blend onto the P1 first-forward suffix, then follow it exactly."""

    reverse_start = forward_tail[-1]
    _, p1_suffix, projection_error = project_to_polyline_suffix(
        template_bypass, reverse_start
    )
    if projection_error > 1e-4:
        raise RuntimeError(
            f"P2 reverse start is {projection_error:.6f} m off the P1 path"
        )
    suffix_length = polyline_length(p1_suffix)
    if suffix_length <= P2_EXIT_REJOIN_DISTANCE_M:
        raise RuntimeError("P1 first-forward suffix is too short for P2 rejoin")
    merge, exact_suffix = split_polyline_at_distance(
        p1_suffix, P2_EXIT_REJOIN_DISTANCE_M
    )
    exit_tangent = trailing_tangent(
        forward_tail, P2_EXIT_REJOIN_TANGENT_BASELINE_M
    )
    merge_tangent = tangent_at_polyline_distance(
        p1_suffix,
        P2_EXIT_REJOIN_DISTANCE_M,
        P2_EXIT_REJOIN_TANGENT_BASELINE_M,
    )
    connector = cubic_connector(
        reverse_start,
        merge,
        exit_tangent,
        merge_tangent,
        P2_EXIT_REJOIN_START_CONTROL_M,
        P2_EXIT_REJOIN_END_CONTROL_M,
        spacing_m,
    )
    return [*connector, *exact_suffix[1:]]


def cubic_bypass(
    start: tuple[float, float],
    end: tuple[float, float],
    start_tangent: tuple[float, float],
    end_tangent: tuple[float, float],
    spacing_m: float,
) -> list[tuple[float, float]]:
    start_unit = unit(start_tangent)
    end_unit = unit(end_tangent)
    control_1 = (
        start[0] + BYPASS_CONTROL_LENGTH_M * start_unit[0],
        start[1] + BYPASS_CONTROL_LENGTH_M * start_unit[1],
    )
    control_2 = (
        end[0] - BYPASS_CONTROL_LENGTH_M * end_unit[0],
        end[1] - BYPASS_CONTROL_LENGTH_M * end_unit[1],
    )
    dense: list[tuple[float, float]] = []
    for index in range(4001):
        parameter = index / 4000.0
        complement = 1.0 - parameter
        x_m = (
            complement**3 * start[0]
            + 3.0 * complement**2 * parameter * control_1[0]
            + 3.0 * complement * parameter**2 * control_2[0]
            + parameter**3 * end[0]
        )
        y_m = (
            complement**3 * start[1]
            + 3.0 * complement**2 * parameter * control_1[1]
            + 3.0 * complement * parameter**2 * control_2[1]
            + parameter**3 * end[1]
        )
        dense.append((x_m, y_m))
    return resample_polyline(dense, spacing_m)


def degree_seven_parking_curve(
    raw_points: list[tuple[float, float]],
    start_tangent: tuple[float, float],
    end_tangent: tuple[float, float],
) -> list[tuple[float, float]]:
    """Fit a curvature-continuous degree-7 Bezier to the parking path."""

    degree = PARKING_CURVE_DEGREE
    start = raw_points[0]
    end = raw_points[-1]
    start_unit = unit(start_tangent)
    end_unit = unit(end_tangent)
    cumulative = [0.0]
    for first, second in zip(raw_points, raw_points[1:]):
        cumulative.append(cumulative[-1] + distance(first, second))
    total = cumulative[-1]
    if total <= 1e-9:
        raise ValueError("zero-length parking curve")
    derivative_length = total * PARKING_CURVE_DERIVATIVE_SCALE
    controls: list[tuple[float, float] | None] = [None] * (degree + 1)
    controls[0] = start
    controls[1] = (
        start[0] + derivative_length * start_unit[0] / degree,
        start[1] + derivative_length * start_unit[1] / degree,
    )
    controls[2] = (
        2.0 * controls[1][0] - start[0],
        2.0 * controls[1][1] - start[1],
    )
    controls[degree] = end
    controls[degree - 1] = (
        end[0] - derivative_length * end_unit[0] / degree,
        end[1] - derivative_length * end_unit[1] / degree,
    )
    controls[degree - 2] = (
        2.0 * controls[degree - 1][0] - end[0],
        2.0 * controls[degree - 1][1] - end[1],
    )
    fixed_indices = (0, 1, 2, degree - 2, degree - 1, degree)
    normal_00 = PARKING_CURVE_REGULARIZATION
    normal_01 = 0.0
    normal_11 = PARKING_CURVE_REGULARIZATION
    target_3 = (
        start[0] + 3.0 * (end[0] - start[0]) / degree,
        start[1] + 3.0 * (end[1] - start[1]) / degree,
    )
    target_4 = (
        start[0] + 4.0 * (end[0] - start[0]) / degree,
        start[1] + 4.0 * (end[1] - start[1]) / degree,
    )
    rhs_0 = [
        PARKING_CURVE_REGULARIZATION * target_3[0],
        PARKING_CURVE_REGULARIZATION * target_3[1],
    ]
    rhs_1 = [
        PARKING_CURVE_REGULARIZATION * target_4[0],
        PARKING_CURVE_REGULARIZATION * target_4[1],
    ]
    for coordinate, arc_length in zip(raw_points, cumulative):
        parameter = arc_length / total
        complement = 1.0 - parameter
        weights = [
            math.comb(degree, index)
            * complement ** (degree - index)
            * parameter**index
            for index in range(degree + 1)
        ]
        fixed_x = sum(weights[index] * controls[index][0] for index in fixed_indices)
        fixed_y = sum(weights[index] * controls[index][1] for index in fixed_indices)
        weight_3 = weights[3]
        weight_4 = weights[4]
        normal_00 += weight_3 * weight_3
        normal_01 += weight_3 * weight_4
        normal_11 += weight_4 * weight_4
        rhs_0[0] += weight_3 * (coordinate[0] - fixed_x)
        rhs_0[1] += weight_3 * (coordinate[1] - fixed_y)
        rhs_1[0] += weight_4 * (coordinate[0] - fixed_x)
        rhs_1[1] += weight_4 * (coordinate[1] - fixed_y)
    determinant = normal_00 * normal_11 - normal_01 * normal_01
    if abs(determinant) <= 1e-12:
        raise ValueError("singular parking-curve fit")
    controls[3] = (
        (rhs_0[0] * normal_11 - rhs_1[0] * normal_01) / determinant,
        (rhs_0[1] * normal_11 - rhs_1[1] * normal_01) / determinant,
    )
    controls[4] = (
        (normal_00 * rhs_1[0] - normal_01 * rhs_0[0]) / determinant,
        (normal_00 * rhs_1[1] - normal_01 * rhs_0[1]) / determinant,
    )
    dense: list[tuple[float, float]] = []
    for sample in range(4001):
        parameter = sample / 4000.0
        complement = 1.0 - parameter
        weights = [
            math.comb(degree, index)
            * complement ** (degree - index)
            * parameter**index
            for index in range(degree + 1)
        ]
        dense.append(
            (
                sum(weights[index] * controls[index][0] for index in range(degree + 1)),
                sum(weights[index] * controls[index][1] for index in range(degree + 1)),
            )
        )
    return dense


def parking_forward_tail(
    option: int,
    source_points: dict[int, tuple[float, float]],
    end_coordinate: tuple[float, float],
    parking_axis: tuple[float, float],
    spacing_m: float,
    template_tail: list[tuple[float, float]] | None = None,
) -> tuple[list[tuple[float, float]], tuple[float, float]]:
    """Build the parking-exit path from an aligned endpoint to the branch exit.

    Reversing this path makes the vehicle release steering continuously and
    finish on a straight segment parallel to the parking axis.
    """

    settings = PARKING[option]
    if template_tail is not None:
        # Place the completed, aligned endpoint on the original P2 forward-
        # parking reference. The whole P1 tail is then shifted rigidly when its
        # reverse start is projected onto the P1 first-forward path.
        complete = source_points[int(settings["park_complete_source"])]
        translation = (
            complete[0] - template_tail[0][0],
            complete[1] - template_tail[0][1],
        )
        translated = [
            (coordinate[0] + translation[0], coordinate[1] + translation[1])
            for coordinate in template_tail
        ]
        return translated, unit(parking_axis)

    complete_source = int(settings["park_complete_source"])
    complete = source_points[complete_source]
    axis = unit(parking_axis)
    aligned_endpoint = (
        complete[0] - PARKING_STRAIGHT_TAIL_M * axis[0],
        complete[1] - PARKING_STRAIGHT_TAIL_M * axis[1],
    )
    reverse_start_source = int(settings["reverse_start_source"])
    recorded_curve = [
        source_points[source_index]
        for source_index in range(complete_source, reverse_start_source + 1)
    ]
    recorded_curve[-1] = end_coordinate
    end_tangent = (
        recorded_curve[-1][0] - recorded_curve[-2][0],
        recorded_curve[-1][1] - recorded_curve[-2][1],
    )
    smoothed_curve = degree_seven_parking_curve(
        recorded_curve, axis, end_tangent
    )
    tail_raw = [aligned_endpoint, complete, *smoothed_curve[1:]]
    return resample_polyline(tail_raw, spacing_m), axis


def heading_error_deg(
    first: tuple[float, float],
    second: tuple[float, float],
    reference: tuple[float, float],
) -> float:
    first_heading = math.atan2(second[1] - first[1], second[0] - first[0])
    reference_heading = math.atan2(reference[1], reference[0])
    delta = math.atan2(
        math.sin(first_heading - reference_heading),
        math.cos(first_heading - reference_heading),
    )
    return abs(math.degrees(delta))


def vector_heading_deg(vector: tuple[float, float]) -> float:
    return math.degrees(math.atan2(vector[1], vector[0]))


def maximum_curvature(points: list[tuple[float, float]]) -> float:
    maximum = 0.0
    for first, middle, last in zip(points, points[1:], points[2:]):
        a = distance(first, middle)
        b = distance(middle, last)
        c = distance(first, last)
        denominator = a * b * c
        if denominator <= 1e-9:
            continue
        twice_area = abs(
            (middle[0] - first[0]) * (last[1] - first[1])
            - (middle[1] - first[1]) * (last[0] - first[0])
        )
        maximum = max(maximum, 2.0 * twice_area / denominator)
    return maximum


def route_s(rows: list[dict[str, str]]) -> None:
    cumulative = 0.0
    previous: tuple[float, float] | None = None
    for row in rows:
        current = point(row)
        if previous is not None:
            cumulative += distance(previous, current)
        row["s_m"] = f"{cumulative:.4f}"
        previous = current


def row_for_final(
    template: dict[str, str],
    coordinate: tuple[float, float],
    direction: int,
    direction_source: str,
    *,
    event_ids: str = "",
    actions: str = "",
    conditions: str = "",
    hold_sec: str = "",
) -> dict[str, str]:
    row = dict(template)
    row.update(
        {
            "x_m": f"{coordinate[0]:.4f}",
            "y_m": f"{coordinate[1]:.4f}",
            "target_speed_mps": "0.000",
            "mission": "PARALLEL_PARK",
            "direction": str(direction),
            "fsm_zone": "PARALLEL_PARK",
            "fsm_event_ids": event_ids,
            "fsm_actions": actions,
            "fsm_conditions": conditions,
            "fsm_hold_sec": hold_sec,
            "direction_source": direction_source,
        }
    )
    return row


def nearest_index(
    rows: list[dict[str, str]],
    target: tuple[float, float],
    start: int,
    end: int,
) -> int:
    return min(range(start, end + 1), key=lambda index: distance(point(rows[index]), target))


def replace_parallel_zone(
    path: Path,
    option: int,
    source_points: dict[int, tuple[float, float]],
    template_tail: list[tuple[float, float]] | None = None,
    template_bypass: list[tuple[float, float]] | None = None,
    template_post_parallel_rows: list[dict[str, str]] | None = None,
) -> tuple[
    dict[str, object],
    list[tuple[float, float]],
    list[tuple[float, float]],
]:
    fieldnames, rows = read_csv(path)
    start_index = next(
        index
        for index, row in enumerate(rows)
        if "PARALLEL_PARK_START" in split_tokens(row["fsm_event_ids"])
    )
    end_event = f"PARALLEL_{option}_END"
    reverse_event = f"PARALLEL_{option}_FORWARD_TO_REVERSE"
    revised = any(reverse_event in split_tokens(row["fsm_event_ids"]) for row in rows)

    if revised:
        reverse_start_index = next(
            index
            for index, row in enumerate(rows)
            if reverse_event in split_tokens(row["fsm_event_ids"])
        )
        end_index = next(
            index
            for index, row in enumerate(rows)
            if end_event in split_tokens(row["fsm_event_ids"])
        )
        reverse_start_coordinate = point(rows[reverse_start_index])
    else:
        end_event_index = next(
            index
            for index, row in enumerate(rows)
            if end_event in split_tokens(row["fsm_event_ids"])
        )
        end_index = end_event_index
        while (
            end_index + 1 < len(rows)
            and rows[end_index + 1]["fsm_zone"] == "PARALLEL_PARK"
        ):
            end_index += 1
        reverse_start_coordinate = point(rows[end_index])

    prefix = [dict(row) for row in rows[:start_index]]
    if option == 2:
        if template_post_parallel_rows is None:
            raise RuntimeError("P2 requires the matching P1 post-parallel route")
        suffix = [dict(row) for row in template_post_parallel_rows]
    else:
        suffix = [dict(row) for row in rows[end_index + 1 :]]
    start_coordinate = point(rows[start_index])
    start_tangent = (
        start_coordinate[0] - point(prefix[-1])[0],
        start_coordinate[1] - point(prefix[-1])[1],
    )
    approach_tangent = trailing_tangent(
        [point(row) for row in rows[: start_index + 1]],
        PARKING_APPROACH_BASELINE_M,
    )
    parking_axis = left_normal(approach_tangent)
    forward_tail, parking_axis = parking_forward_tail(
        option,
        source_points,
        reverse_start_coordinate,
        parking_axis,
        FINAL_SPACING_M,
        template_tail,
    )
    baseline_reverse_start = forward_tail[-1]
    parking_reference_alignment_error = 0.0
    if template_tail is not None:
        if template_bypass is None:
            raise RuntimeError("P2 requires the matching P1 first-forward path")
        (
            reverse_start_coordinate,
            bypass,
            parking_reference_alignment_error,
        ) = project_to_polyline_prefix(
            template_bypass,
            baseline_reverse_start,
        )
        translation = (
            reverse_start_coordinate[0] - baseline_reverse_start[0],
            reverse_start_coordinate[1] - baseline_reverse_start[1],
        )
        forward_tail = [
            (coordinate[0] + translation[0], coordinate[1] + translation[1])
            for coordinate in forward_tail
        ]
    else:
        reverse_start_coordinate = baseline_reverse_start
        end_tangent = (
            forward_tail[-1][0] - forward_tail[-2][0],
            forward_tail[-1][1] - forward_tail[-2][1],
        )
        bypass = cubic_bypass(
            start_coordinate,
            reverse_start_coordinate,
            start_tangent,
            end_tangent,
            FINAL_SPACING_M,
        )
    post_exit_connector = (
        p2_post_exit_path(template_bypass, forward_tail, FINAL_SPACING_M)
        if option == 2 and template_bypass is not None
        else []
    )
    post_exit_path_reference_deviation = 0.0
    if option == 2 and template_bypass is not None:
        _, p1_exit_reference, _ = project_to_polyline_suffix(
            template_bypass, reverse_start_coordinate
        )
        post_exit_path_reference_deviation = max(
            distance_to_polyline(coordinate, p1_exit_reference)
            for coordinate in post_exit_connector
        )

    template = rows[start_index]
    replacement: list[dict[str, str]] = []
    for index, coordinate in enumerate(bypass[:-1]):
        replacement.append(
            row_for_final(
                template,
                coordinate,
                1,
                f"PARALLEL_{option}_BYPASS_FORWARD",
                event_ids="PARALLEL_PARK_START" if index == 0 else "",
                actions="ENTER_ZONE" if index == 0 else "",
            )
        )

    reverse_tail = list(reversed(forward_tail))
    for index, coordinate in enumerate(reverse_tail[:-1]):
        replacement.append(
            row_for_final(
                template,
                coordinate,
                -1,
                f"PARALLEL_{option}_PARK_REVERSE",
                event_ids=reverse_event if index == 0 else "",
                actions="SET_REVERSE" if index == 0 else "",
            )
        )

    for index, coordinate in enumerate(forward_tail):
        event_ids = ""
        actions = ""
        conditions = ""
        if index == 0:
            event_ids = (
                f"PARALLEL_{option}_PARK_COMPLETE|"
                f"PARALLEL_{option}_REVERSE_TO_FORWARD"
            )
            actions = "PARK_COMPLETE|SET_FORWARD"
            conditions = "ALWAYS_ONCE"
        elif index == len(forward_tail) - 1 and option == 1:
            event_ids = end_event
            actions = "BRANCH_COMPLETE"
        replacement.append(
            row_for_final(
                template,
                coordinate,
                1,
                f"PARALLEL_{option}_PARK_EXIT_FORWARD",
                event_ids=event_ids,
                actions=actions,
                conditions=conditions,
            )
        )

    for index, coordinate in enumerate(post_exit_connector[1:]):
        is_last = index == len(post_exit_connector) - 2
        replacement.append(
            row_for_final(
                template,
                coordinate,
                1,
                "PARALLEL_2_POST_EXIT_FORWARD",
                event_ids=end_event if is_last else "",
                actions="BRANCH_COMPLETE" if is_last else "",
            )
        )

    output = prefix + replacement + suffix
    for row in output:
        if "HILL_STOP" in split_tokens(row.get("fsm_event_ids", "")):
            row["fsm_actions"] = "STOP_THEN_HOLD"
            row["fsm_conditions"] = "ALWAYS_ONCE"
            row["fsm_hold_sec"] = f"{HILL_HOLD_SEC:.1f}"
    route_s(output)
    write_csv(path, fieldnames, output)

    bypass_curvature = maximum_curvature(bypass)
    post_exit_connector_curvature = maximum_curvature(post_exit_connector)
    parking_tail_curvature = maximum_curvature(forward_tail)
    parking_tail_steering_rad = math.atan(
        VEHICLE_WHEELBASE_M * parking_tail_curvature
    )
    post_exit_steering_rad = math.atan(
        VEHICLE_WHEELBASE_M * post_exit_connector_curvature
    )
    complete_source = int(PARKING[option]["park_complete_source"])
    if option == 1:
        reverse_start_source = int(PARKING[option]["reverse_start_source"])
        path_reference = [
            source_points[source_index]
            for source_index in range(complete_source, reverse_start_source + 1)
        ]
        path_reference[-1] = reverse_start_coordinate
        path_reference_kind = (
            f"recorded course_09:{complete_source}-{reverse_start_source}"
        )
        maximum_path_reference_deviation = max(
            min(distance(reference, generated) for generated in forward_tail)
            for reference in path_reference
        )
    else:
        path_reference_kind = (
            "exact P1 parking-tail copy translated to a reverse-start point "
            "on the P1 first-forward path and aligned to recorded C09-167"
        )
        maximum_path_reference_deviation = 0.0
    straight_tail_target = PARKING_STRAIGHT_TAIL_M - 0.35
    straight_tail_index = min(
        range(len(forward_tail)),
        key=lambda index: abs(
            sum(
                distance(first, second)
                for first, second in zip(
                    forward_tail[:index], forward_tail[1 : index + 1]
                )
            )
            - straight_tail_target
        ),
    )
    early_stop_first_index = min(
        range(straight_tail_index + 1),
        key=lambda index: abs(
            sum(
                distance(first, second)
                for first, second in zip(
                    forward_tail[index:straight_tail_index],
                    forward_tail[index + 1 : straight_tail_index + 1],
                )
            )
            - 1.0
        ),
    )
    early_stop_heading_error = heading_error_deg(
        forward_tail[early_stop_first_index],
        forward_tail[straight_tail_index],
        parking_axis,
    )
    maximum_spacing = max(
        distance(point(first), point(second))
        for first, second in zip(output, output[1:])
    )
    event_counts = Counter(
        event
        for row in output
        for event in split_tokens(row.get("fsm_event_ids", ""))
    )
    required = {
        "HILL_STOP",
        "PARALLEL_PARK_START",
        reverse_event,
        f"PARALLEL_{option}_PARK_COMPLETE",
        f"PARALLEL_{option}_REVERSE_TO_FORWARD",
        end_event,
    }
    missing_or_duplicate = {
        event: event_counts[event] for event in required if event_counts[event] != 1
    }
    if missing_or_duplicate:
        raise RuntimeError(f"{path.name}: invalid event counts {missing_or_duplicate}")
    if bypass_curvature > 1.0 / 1.5 + 1e-6:
        raise RuntimeError(
            f"{path.name}: bypass radius below 1.50 m ({1.0 / bypass_curvature:.3f} m)"
        )
    if parking_tail_curvature > PARKING_MAX_CURVATURE_INV_M + 1e-6:
        raise RuntimeError(
            f"{path.name}: parking-tail curvature {parking_tail_curvature:.3f} "
            f"1/m exceeds {PARKING_MAX_CURVATURE_INV_M:.3f} 1/m"
        )
    if post_exit_connector_curvature > PARKING_MAX_CURVATURE_INV_M + 1e-6:
        raise RuntimeError(
            f"{path.name}: post-exit curvature "
            f"{post_exit_connector_curvature:.3f} 1/m exceeds "
            f"{PARKING_MAX_CURVATURE_INV_M:.3f} 1/m"
        )
    if (
        option == 2
        and post_exit_path_reference_deviation
        > P2_EXIT_REJOIN_MAX_DEVIATION_M + 1e-6
    ):
        raise RuntimeError(
            f"{path.name}: P2 post-exit path deviates "
            f"{post_exit_path_reference_deviation:.3f} m from P1, exceeding "
            f"{P2_EXIT_REJOIN_MAX_DEVIATION_M:.3f} m"
        )
    if parking_tail_steering_rad > VEHICLE_MAX_STEERING_RAD:
        raise RuntimeError(
            f"{path.name}: parking-tail steering "
            f"{math.degrees(parking_tail_steering_rad):.2f} deg exceeds vehicle limit"
        )
    if maximum_spacing > 0.111:
        raise RuntimeError(f"{path.name}: spacing {maximum_spacing:.3f} m exceeds 0.11 m")
    if early_stop_heading_error > 1.0:
        raise RuntimeError(
            f"{path.name}: early-stop heading error "
            f"{early_stop_heading_error:.3f} deg exceeds 1.0 deg"
        )
    if (
        option == 2
        and parking_reference_alignment_error
        > P2_TEMPLATE_ALIGNMENT_MAX_ERROR_M + 1e-6
    ):
        raise RuntimeError(
            f"{path.name}: P2 original-route parking reference alignment error "
            f"{parking_reference_alignment_error:.3f} m exceeds "
            f"{P2_TEMPLATE_ALIGNMENT_MAX_ERROR_M:.3f} m"
        )
    return {
        "file": path.name,
        "points": len(output),
        "bypass_points": len(bypass),
        "bypass_length_m": sum(
            distance(first, second) for first, second in zip(bypass, bypass[1:])
        ),
        "reverse_start_x_m": reverse_start_coordinate[0],
        "reverse_start_y_m": reverse_start_coordinate[1],
        "reverse_depth_m": sum(
            distance(first, second)
            for first, second in zip(forward_tail, forward_tail[1:])
        ),
        "straight_alignment_tail_m": PARKING_STRAIGHT_TAIL_M,
        "maximum_parking_tail_curvature_inv_m": parking_tail_curvature,
        "minimum_parking_tail_radius_m": 1.0 / parking_tail_curvature,
        "maximum_parking_tail_steering_deg": math.degrees(
            parking_tail_steering_rad
        ),
        "path_reference": path_reference_kind,
        "maximum_path_reference_deviation_m": maximum_path_reference_deviation,
        "original_forward_parking_reference": (
            f"C09-{complete_source:03d}"
        ),
        "original_forward_parking_alignment_error_m": (
            parking_reference_alignment_error
        ),
        "first_forward_path_reference": (
            "generated P1 cubic bypass"
            if option == 1
            else "exact prefix of the matching P1 first-forward path"
        ),
        "vehicle_maximum_steering_deg": math.degrees(VEHICLE_MAX_STEERING_RAD),
        "approach_heading_deg": vector_heading_deg(approach_tangent),
        "parking_axis_heading_deg": vector_heading_deg(parking_axis),
        "approach_to_parking_axis_deg": 90.0,
        "early_stop_heading_error_deg": early_stop_heading_error,
        "maximum_spacing_m": maximum_spacing,
        "maximum_bypass_curvature_inv_m": bypass_curvature,
        "minimum_bypass_radius_m": 1.0 / bypass_curvature,
        "post_exit_connector_length_m": sum(
            distance(first, second)
            for first, second in zip(
                post_exit_connector, post_exit_connector[1:]
            )
        ),
        "maximum_post_exit_curvature_inv_m": post_exit_connector_curvature,
        "maximum_post_exit_steering_deg": math.degrees(post_exit_steering_rad),
        "post_exit_path_reference": (
            None
            if option == 1
            else (
                f"{P2_EXIT_REJOIN_DISTANCE_M:.2f} m cubic transition, then "
                "the exact P1 first-forward suffix"
            )
        ),
        "maximum_post_exit_deviation_from_p1_first_forward_m": (
            post_exit_path_reference_deviation
        ),
        "minimum_post_exit_radius_m": (
            None
            if post_exit_connector_curvature <= 1e-9
            else 1.0 / post_exit_connector_curvature
        ),
    }, forward_tail, bypass


def master_row(
    fieldnames: list[str],
    segment_id: str,
    option: int,
    phase: str,
    sequence: int,
    coordinate: tuple[float, float],
    direction: int,
    *,
    spacing_kind: str,
) -> dict[str, str]:
    row = {key: "" for key in fieldnames}
    row.update(
        {
            "segment_id": segment_id,
            "branch_group": "PARALLEL",
            "branch_option": str(option),
            "motion_phase": phase,
            "sequence": str(sequence),
            "source": spacing_kind,
            "source_index": "",
            "x_m": f"{coordinate[0]:.4f}",
            "y_m": f"{coordinate[1]:.4f}",
            "mission": "PARALLEL_PARK",
            "direction_status": "FORWARD" if direction > 0 else "REVERSE",
        }
    )
    return row


def parallel_master_segments(
    option: int,
    spacing_m: float,
    fieldnames: list[str],
    source_points: dict[int, tuple[float, float]],
    main_path: list[tuple[float, float]],
    final_common_path: list[tuple[float, float]],
    existing_entry: tuple[float, float],
    template_tail: list[tuple[float, float]] | None = None,
    template_bypass: list[tuple[float, float]] | None = None,
) -> tuple[
    list[dict[str, str]],
    list[tuple[float, float]],
    list[tuple[float, float]],
]:
    settings = PARKING[option]
    start = source_points[106] if option == 1 else existing_entry
    start_tangent = (start[0] - main_path[-1][0], start[1] - main_path[-1][1])
    approach_tangent = trailing_tangent(
        [*main_path, start], PARKING_APPROACH_BASELINE_M
    )
    parking_axis = left_normal(approach_tangent)
    end = (
        source_points[int(settings["reverse_start_source"])]
        if template_tail is None
        else template_tail[-1]
    )
    tail, _ = parking_forward_tail(
        option,
        source_points,
        end,
        parking_axis,
        spacing_m,
        template_tail,
    )
    baseline_end = tail[-1]
    end_tangent = (tail[-1][0] - tail[-2][0], tail[-1][1] - tail[-2][1])
    if template_tail is not None:
        if template_bypass is None:
            raise RuntimeError("P2 master requires the P1 first-forward path")
        end, bypass, alignment_error = project_to_polyline_prefix(
            template_bypass, baseline_end
        )
        if alignment_error > P2_TEMPLATE_ALIGNMENT_MAX_ERROR_M + 1e-6:
            raise RuntimeError(
                "P2 master original-route parking reference alignment error "
                f"{alignment_error:.3f} m exceeds "
                f"{P2_TEMPLATE_ALIGNMENT_MAX_ERROR_M:.3f} m"
            )
        translation = (end[0] - baseline_end[0], end[1] - baseline_end[1])
        tail = [
            (coordinate[0] + translation[0], coordinate[1] + translation[1])
            for coordinate in tail
        ]
    else:
        end = baseline_end
        bypass = cubic_bypass(start, end, start_tangent, end_tangent, spacing_m)
    post_exit: list[tuple[float, float]] = []
    if option == 2:
        if template_bypass is None:
            raise RuntimeError("P2 master requires the P1 first-forward path")
        rejoin = p2_post_exit_path(
            template_bypass,
            tail,
            spacing_m,
        )
        post_exit = [*rejoin[1:], *final_common_path]
    rows: list[dict[str, str]] = []
    definitions = [
        (f"PARALLEL_{option}_BYPASS", "BYPASS_FORWARD", bypass[:-1], 1),
        (
            f"PARALLEL_{option}_REVERSE",
            "PARK_REVERSE",
            list(reversed(tail))[:-1],
            -1,
        ),
        (f"PARALLEL_{option}_EXIT", "PARK_EXIT_FORWARD", tail, 1),
    ]
    if post_exit:
        definitions.append(
            (
                "PARALLEL_2_POST_EXIT",
                "POST_EXIT_FORWARD_TO_P1_PATH",
                post_exit,
                1,
            )
        )
    for segment_id, phase, coordinates, direction in definitions:
        rows.extend(
            master_row(
                fieldnames,
                segment_id,
                option,
                phase,
                sequence,
                coordinate,
                direction,
                spacing_kind="derived_parallel_revision",
            )
            for sequence, coordinate in enumerate(coordinates)
        )
    return rows, tail, bypass


def rebuild_master(
    path: Path,
    spacing_m: float,
    source_points: dict[int, tuple[float, float]],
) -> None:
    fieldnames, rows = read_csv(path)
    parallel_indices = [
        index
        for index, row in enumerate(rows)
        if row["segment_id"].startswith("PARALLEL_")
    ]
    first, last = min(parallel_indices), max(parallel_indices)
    main_rows = [row for row in rows if row["segment_id"] == "MAIN_01"]
    main_path = [point(row) for row in main_rows]
    final_common_path = [
        point(row) for row in rows if row["segment_id"] == "FINAL_COMMON"
    ]
    old_p2 = next(
        row
        for row in rows
        if row["segment_id"].startswith("PARALLEL_2")
        and row.get("sequence") == "0"
    )
    p2_entry = point(old_p2)
    p1_rows, p1_tail, p1_bypass = parallel_master_segments(
        1,
        spacing_m,
        fieldnames,
        source_points,
        main_path,
        final_common_path,
        p2_entry,
    )
    p2_rows, _, _ = parallel_master_segments(
        2,
        spacing_m,
        fieldnames,
        source_points,
        main_path,
        final_common_path,
        p2_entry,
        p1_tail,
        p1_bypass,
    )
    replacement = p1_rows + p2_rows
    write_csv(path, fieldnames, rows[:first] + replacement + rows[last + 1 :])


def annotation_row(
    fieldnames: list[str],
    event_id: str,
    source_index: int | None,
    option: int,
    event_type: str,
    action: str,
    direction_after: str,
    route_phase: str,
    description: str,
    *,
    condition: str = "",
) -> dict[str, str]:
    row = {key: "" for key in fieldnames}
    row.update(
        {
            "event_id": event_id,
            "source_start": "" if source_index is None else str(source_index),
            "source_end": "" if source_index is None else str(source_index),
            "fsm_zone": "PARALLEL_PARK",
            "runtime_mission": "PARALLEL_PARK",
            "branch_group": "PARALLEL",
            "branch_option": str(option),
            "event_type": event_type,
            "action": action,
            "condition": condition,
            "hold_sec": "",
            "direction_after": direction_after,
            "route_phase": route_phase,
            "description_ko": description,
        }
    )
    return row


def update_annotations(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    fieldnames, rows = read_csv(path)
    if "route_phase" not in fieldnames:
        description_index = fieldnames.index("description_ko")
        fieldnames.insert(description_index, "route_phase")
    output: list[dict[str, str]] = []
    generated_parallel_event = re.compile(
        r"PARALLEL_[12]_(?:FORWARD_TO_REVERSE|PARK_COMPLETE|REVERSE_TO_FORWARD|END)"
    )
    for row in rows:
        row = dict(row)
        row.setdefault("route_phase", "")
        if row["event_id"] == "HILL_STOP":
            row["action"] = "STOP_THEN_HOLD"
            row["condition"] = "ALWAYS_ONCE"
            row["hold_sec"] = f"{HILL_HOLD_SEC:.1f}"
            row["route_phase"] = "HILL_STOP_LINE"
            row["description_ko"] = (
                "정지선 직전 C09-002에서 완전 정지 후 3초 유지"
            )
        if generated_parallel_event.fullmatch(row["event_id"]):
            continue
        output.append(row)
        if row["event_id"] != "PARALLEL_PARK_START":
            continue
        output[-1]["route_phase"] = "BYPASS_START"
        output[-1]["description_ko"] = "평행주차 구획을 우회하는 전진 경로 시작"
        for option in (1, 2):
            settings = PARKING[option]
            reverse_start_value = settings.get("reverse_start_source")
            reverse_start = (
                None if reverse_start_value is None else int(reverse_start_value)
            )
            complete = int(settings["park_complete_source"])
            reverse_start_description = (
                f"평행주차 {option}번 출구에서 완전 정지 후 후진 전환"
                if reverse_start is not None
                else (
                    "P1 첫 전진 경로 위에서 원래 P2 전진 주차의 "
                    f"C09-{complete:03d} 기준점에 가장 가깝게 맞춘 지점에서 "
                    "완전 정지 후 후진 전환"
                )
            )
            output.extend(
                [
                    annotation_row(
                        fieldnames,
                        f"PARALLEL_{option}_FORWARD_TO_REVERSE",
                        reverse_start,
                        option,
                        "DIRECTION_CHANGE",
                        "SET_REVERSE",
                        "-1",
                        "REVERSE_START",
                        reverse_start_description,
                    ),
                    annotation_row(
                        fieldnames,
                        f"PARALLEL_{option}_PARK_COMPLETE",
                        complete,
                        option,
                        "STOP_GATE",
                        "PARK_COMPLETE",
                        "",
                        "PARK_COMPLETE",
                        (
                            f"C09-{complete:03d} 이후 평행주차 시작 직전 경로에 "
                            f"수직인 {PARKING_STRAIGHT_TAIL_M:.1f}m 직선 정렬 종점에서 완전 정지"
                            if option == 1
                            else (
                                f"원래 P2 전진 주차 경로 C09-{complete:03d}에 맞춘 "
                                "P1 복사 궤적의 직선 정렬 종점에서 완전 정지"
                            )
                        ),
                        condition="ALWAYS_ONCE",
                    ),
                    annotation_row(
                        fieldnames,
                        f"PARALLEL_{option}_REVERSE_TO_FORWARD",
                        complete,
                        option,
                        "DIRECTION_CHANGE",
                        "SET_FORWARD",
                        "1",
                        "EXIT_START",
                        (
                            f"평행주차 {option}번 직선 정렬 종점에서 "
                            "주차 완료 후 전진 출차"
                        ),
                    ),
                    annotation_row(
                        fieldnames,
                        f"PARALLEL_{option}_END",
                        reverse_start,
                        option,
                        "BRANCH_END",
                        "BRANCH_COMPLETE",
                        "",
                        "EXIT_END",
                        (
                            f"평행주차 {option}번 출구 복귀 완료"
                            if reverse_start is not None
                            else (
                                f"{P2_EXIT_REJOIN_DISTANCE_M:.1f}m 전환곡선 후 "
                                "P1 첫 전진 경로를 따라 공통 경로 복귀 완료"
                            )
                        ),
                    ),
                ]
            )
    write_csv(path, fieldnames, output)
    return fieldnames, output


def direction_legs(rows: list[dict[str, str]]) -> list[dict[str, object]]:
    legs: list[dict[str, object]] = []
    start = 0
    current = int(rows[0]["direction"])
    for index in range(1, len(rows) + 1):
        next_direction = int(rows[index]["direction"]) if index < len(rows) else None
        if next_direction == current:
            continue
        coordinates = [point(row) for row in rows[start:index]]
        curvature = maximum_curvature(coordinates)
        legs.append(
            {
                "start_index": start,
                "end_index": index - 1,
                "start_s_m": float(rows[start]["s_m"]),
                "end_s_m": float(rows[index - 1]["s_m"]),
                "points": index - start,
                "direction": current,
                "maximum_curvature_inv_m": curvature,
                "minimum_radius_m": None if curvature <= 1e-9 else 1.0 / curvature,
            }
        )
        if index < len(rows):
            start = index
            current = int(rows[index]["direction"])
    return legs


def rebuild_event_json(
    path: Path,
    annotations: list[dict[str, str]],
    source_points: dict[int, tuple[float, float]],
    variant_rows: dict[str, list[dict[str, str]]],
) -> None:
    annotation_lookup = {row["event_id"]: row for row in annotations}
    variants: dict[str, object] = {}
    for variant, rows in variant_rows.items():
        events = []
        for index, row in enumerate(rows):
            for event_id in split_tokens(row.get("fsm_event_ids", "")):
                annotation = annotation_lookup.get(event_id, {})
                source_start = annotation.get("source_start", "")
                mapping_distance = None
                if source_start.isdigit() and int(source_start) in source_points:
                    mapping_distance = distance(point(row), source_points[int(source_start)])
                events.append(
                    {
                        "event_id": event_id,
                        "source_start": int(source_start) if source_start.isdigit() else None,
                        "source_end": int(annotation.get("source_end", ""))
                        if annotation.get("source_end", "").isdigit()
                        else None,
                        "route_phase": annotation.get("route_phase", ""),
                        "route_index_start": index,
                        "route_index_end": index,
                        "route_s_start_m": float(row["s_m"]),
                        "route_s_end_m": float(row["s_m"]),
                        "mapping_distance_start_m": mapping_distance,
                        "mapping_distance_end_m": mapping_distance,
                    }
                )
        match = re.fullmatch(r"p([12])_t([12])_f([12])", variant)
        assert match is not None
        variants[variant] = {
            "source_route": f"course_07_{variant}_fair_fsm_preview.csv",
            "annotated_route": f"course_07_{variant}_fair_fsm_preview.csv",
            "branch_options": {
                "PARALLEL": match.group(1),
                "T": match.group(2),
                "FINAL": match.group(3),
            },
            "events": events,
        }
    payload = {
        "schema_version": 2,
        "name": "course_07_fsm_route_events",
        "drive_ready": False,
        "reason": (
            "hill and parallel-parking maneuver events are assigned; target speeds, "
            "vehicle swept-volume clearance, signal perception, and dynamic final-lane "
            "selection remain unvalidated"
        ),
        "annotation_source": "course_09_fsm_annotations.csv",
        "annotations": annotations,
        "variants": variants,
    }
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def update_manifests(
    metrics: dict[str, dict[str, object]],
    variant_rows: dict[str, list[dict[str, str]]],
) -> None:
    overlay_path = ROUTE_DIR / "course_07_parking_overlay_manifest.json"
    overlay = json.loads(overlay_path.read_text(encoding="utf-8"))
    overlay["drive_ready"] = False
    overlay["reason"] = (
        "hill stop and parallel reverse maneuver are authored; speeds and vehicle "
        "swept-volume clearance remain unvalidated"
    )
    overlay["flow"] = [
        "MAIN_00",
        {"choose_one": ["T_1", "T_2"]},
        "MAIN_01",
        {
            "choose_one": [
                [
                    "PARALLEL_1_BYPASS",
                    "PARALLEL_1_REVERSE",
                    "PARALLEL_1_EXIT",
                    "FINAL_COMMON",
                ],
                [
                    "PARALLEL_2_BYPASS",
                    "PARALLEL_2_REVERSE",
                    "PARALLEL_2_EXIT",
                    "PARALLEL_2_POST_EXIT",
                ],
            ]
        },
        {"choose_one": ["FINAL_1", "FINAL_2"]},
    ]
    for option in (1, 2):
        settings = PARKING[option]
        entry = int(settings["entry_source"])
        complete = int(settings["park_complete_source"])
        bypass_start = (
            f"C09-{entry:03d}"
            if option == 1
            else "course_07:704 vicinity"
        )
        if option == 1:
            reverse_start = int(settings["reverse_start_source"])
            ranges = [
                f"derived cubic bypass {bypass_start} to C09-{reverse_start:03d}",
                (
                    f"degree-{PARKING_CURVE_DEGREE} curvature-continuous fit of "
                    f"course_09:{reverse_start}-{complete} reverse plus derived "
                    f"{PARKING_STRAIGHT_TAIL_M:.2f} m approach-normal-aligned tail"
                ),
                (
                    f"derived aligned endpoint through degree-{PARKING_CURVE_DEGREE} "
                    f"fit of course_09:{complete}-{reverse_start} forward exit"
                ),
            ]
        else:
            ranges = [
                (
                    "exact prefix of the P1 cubic bypass from "
                    f"{bypass_start} to the P2 reverse start projected from the "
                    f"C09-{complete:03d} parking reference"
                ),
                (
                    "exact translated copy of the complete P1 reverse-parking "
                    f"path, including its {PARKING_STRAIGHT_TAIL_M:.2f} m "
                    "straight alignment tail"
                ),
                "forward exit by retracing the same copied P1 path",
                (
                    f"{P2_EXIT_REJOIN_DISTANCE_M:.2f} m cubic transition onto "
                    "the P1 first-forward suffix, then the exact P1 suffix and "
                    "FINAL_COMMON"
                ),
            ]
        overlay["source_ranges_inclusive"][f"PARALLEL_{option}"] = ranges
    for variant, segments in list(overlay["variants"].items()):
        option = int(re.search(r"p([12])", variant).group(1))
        replacement = [
            f"PARALLEL_{option}_BYPASS",
            f"PARALLEL_{option}_REVERSE",
            f"PARALLEL_{option}_EXIT",
        ]
        overlay["variants"][variant] = [
            item
            for segment in segments
            for item in (replacement if segment == f"PARALLEL_{option}" else [segment])
        ]
        revised_segments = overlay["variants"][variant]
        if option == 2:
            revised_segments = [
                segment for segment in revised_segments if segment != "FINAL_COMMON"
            ]
            if "PARALLEL_2_POST_EXIT" not in revised_segments:
                exit_index = revised_segments.index("PARALLEL_2_EXIT")
                revised_segments.insert(exit_index + 1, "PARALLEL_2_POST_EXIT")
        overlay["variants"][variant] = revised_segments
    _, master_rows = read_csv(ROUTE_DIR / "course_07_parking_overlay_master.csv")
    segment_paths: dict[str, list[tuple[float, float]]] = {}
    for row in master_rows:
        segment_paths.setdefault(row["segment_id"], []).append(point(row))
    overlay["joins"] = {
        variant: [
            {
                "from": first,
                "to": second,
                "distance_m": round(
                    distance(segment_paths[first][-1], segment_paths[second][0]),
                    4,
                ),
            }
            for first, second in zip(segments, segments[1:])
        ]
        for variant, segments in overlay["variants"].items()
    }
    overlay["parallel_revision"] = {
        "hill_stop_source": "C09-002",
        "hill_hold_sec": HILL_HOLD_SEC,
        "bypass_method": (
            "P1 cubic Bezier with endpoint tangent constraints; P2 exact prefix "
            "of the P1 first-forward path"
        ),
        "bypass_control_length_m": BYPASS_CONTROL_LENGTH_M,
        "parking_alignment_straight_tail_m": PARKING_STRAIGHT_TAIL_M,
        "parking_approach_heading_baseline_m": PARKING_APPROACH_BASELINE_M,
        "parking_alignment_basis": (
            "left normal of the route tangent immediately before "
            "PARALLEL_PARK_START"
        ),
        "parking_alignment_angle_from_approach_deg": 90.0,
        "parking_curve_method": (
            "P1 uses a regularized degree-7 Bezier fit with fixed endpoint "
            "tangents and zero endpoint second derivatives; P2 is an exact "
            "translated copy of the complete P1 parking tail"
        ),
        "parking_curve_derivative_scale": PARKING_CURVE_DERIVATIVE_SCALE,
        "parking_curve_regularization": PARKING_CURVE_REGULARIZATION,
        "parking_maximum_curvature_inv_m": PARKING_MAX_CURVATURE_INV_M,
        "parking_minimum_radius_m": 1.0 / PARKING_MAX_CURVATURE_INV_M,
        "vehicle_wheelbase_m": VEHICLE_WHEELBASE_M,
        "vehicle_maximum_steering_deg": math.degrees(VEHICLE_MAX_STEERING_RAD),
        "P1_park_complete_source": f"C09-{PARKING[1]['park_complete_source']:03d}",
        "P2_park_complete_source": f"C09-{PARKING[2]['park_complete_source']:03d}",
        "P2_original_forward_parking_reference": (
            f"C09-{PARKING[2]['park_complete_source']:03d}"
        ),
        "P2_template_alignment_max_error_m": P2_TEMPLATE_ALIGNMENT_MAX_ERROR_M,
        "P2_reverse_start_method": (
            "project the C09-167-aligned P1 tail start onto the exact P1 "
            "first-forward path"
        ),
        "P2_first_forward_template": "exact prefix of the P1 bypass",
        "P2_path_template": "exact translated copy of the P1 parking tail",
        "P2_exit_rejoin_method": (
            f"{P2_EXIT_REJOIN_DISTANCE_M:.2f} m tangent-constrained cubic "
            "transition followed by the exact P1 first-forward suffix"
        ),
        "P2_exit_rejoin_start_control_m": P2_EXIT_REJOIN_START_CONTROL_M,
        "P2_exit_rejoin_end_control_m": P2_EXIT_REJOIN_END_CONTROL_M,
        "P2_exit_rejoin_max_deviation_m": P2_EXIT_REJOIN_MAX_DEVIATION_M,
        "generator": "operations/rebuild_course_07_parallel_and_hill.py",
    }
    overlay_path.write_text(
        json.dumps(overlay, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )

    smoothing_path = ROUTE_DIR / "course_07_parking_smoothing_manifest.json"
    smoothing = json.loads(smoothing_path.read_text(encoding="utf-8"))
    smoothing["reason"] = (
        "curvature and direction phases are authored; steering slew and vehicle "
        "swept-volume clearance remain unvalidated"
    )
    smoothing["intentional_phase_boundaries"]["PARALLEL_1"] = [
        "C09-143 forward-to-reverse",
        (
            f"derived {PARKING_STRAIGHT_TAIL_M:.2f} m straight-tail endpoint "
            f"anchored at C09-{PARKING[1]['park_complete_source']:03d} reverse-to-forward"
        ),
        "C09-143 exit complete",
    ]
    smoothing["intentional_phase_boundaries"]["PARALLEL_2"] = [
        (
            "P1-copy start projected onto the P1 first-forward path using "
            f"C09-{PARKING[2]['park_complete_source']:03d} forward-to-reverse"
        ),
        (
            f"derived {PARKING_STRAIGHT_TAIL_M:.2f} m straight-tail endpoint "
            f"anchored at C09-{PARKING[2]['park_complete_source']:03d} reverse-to-forward"
        ),
        (
            f"{P2_EXIT_REJOIN_DISTANCE_M:.2f} m transition onto P1 first-forward "
            "suffix and common-path exit complete"
        ),
    ]
    smoothing["parallel_revision_metrics"] = {
        key: value for key, value in metrics.items() if key.endswith("t1_f1")
    }
    smoothing_path.write_text(
        json.dumps(smoothing, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )

    fair_path = ROUTE_DIR / "course_07_full_route_fairing_manifest.json"
    variants = {}
    for variant, rows in variant_rows.items():
        event_counts = Counter(
            event
            for row in rows
            for event in split_tokens(row.get("fsm_event_ids", ""))
        )
        variants[variant] = {
            "source_route": f"course_07_{variant}_fair_fsm_preview.csv",
            "fair_route": f"course_07_{variant}_fair_fsm_preview.csv",
            "output_points": len(rows),
            "output_length_m": float(rows[-1]["s_m"]),
            "direction_legs": direction_legs(rows),
            "fsm_events_preserved": dict(sorted(event_counts.items())),
        }
    fair = {
        "schema_version": 2,
        "name": "course_07_full_route_with_reverse_parallel_parking",
        "method": (
            "existing natural-cubic fair route outside parallel parking; tangent-"
            "constrained P1 cubic bypass, exact P1 first-forward prefix for P2, "
            "regularized degree-7 curvature-continuous P1 parking curve, exact "
            "translated P1 parking-tail copy for P2, and straight alignment "
            "tails; P2 uses a short tangent-constrained transition onto the P1 "
            "first-forward suffix before continuing through FINAL_COMMON"
        ),
        "maximum_spacing_m": FINAL_SPACING_M,
        "minimum_turning_radius_m": 1.5,
        "direction_cusps_split": True,
        "fsm_event_rows_fixed": True,
        "drive_ready": False,
        "reason": (
            "geometry and direction events are authored, but target speeds and "
            "vehicle swept-volume clearance still require field validation"
        ),
        "revision_generator": "operations/rebuild_course_07_parallel_and_hill.py",
        "variants": variants,
    }
    fair_path.write_text(
        json.dumps(fair, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )


def run() -> None:
    _, indexed_rows = read_csv(ROUTE_DIR / "course_09_indexed_points.csv")
    source_points = {
        int(row["source_index"]): (float(row["x_m"]), float(row["y_m"]))
        for row in indexed_rows
    }
    metrics: dict[str, dict[str, object]] = {}
    variant_rows: dict[str, list[dict[str, str]]] = {}
    p1_template_tails: dict[str, list[tuple[float, float]]] = {}
    p1_template_bypasses: dict[str, list[tuple[float, float]]] = {}
    p1_template_post_parallel_rows: dict[str, list[dict[str, str]]] = {}
    for path in sorted(ROUTE_DIR.glob("course_07_p*_t*_f*_fair_fsm_preview.csv")):
        match = re.fullmatch(
            r"course_07_(p([12])_t[12]_f[12])_fair_fsm_preview\.csv", path.name
        )
        if match is None:
            continue
        variant = match.group(1)
        option = int(match.group(2))
        variant_suffix = variant.split("_", 1)[1]
        template_tail = (
            None if option == 1 else p1_template_tails.get(variant_suffix)
        )
        template_bypass = (
            None if option == 1 else p1_template_bypasses.get(variant_suffix)
        )
        template_post_parallel_rows = (
            None
            if option == 1
            else p1_template_post_parallel_rows.get(variant_suffix)
        )
        if option == 2 and (
            template_tail is None
            or template_bypass is None
            or template_post_parallel_rows is None
        ):
            raise RuntimeError(
                f"{variant}: matching P1 parking, first-forward, and post-parallel "
                "templates are unavailable"
            )
        variant_metrics, forward_tail, bypass = replace_parallel_zone(
            path,
            option,
            source_points,
            template_tail,
            template_bypass,
            template_post_parallel_rows,
        )
        metrics[variant] = variant_metrics
        _, generated_rows = read_csv(path)
        if option == 1:
            p1_template_tails[variant_suffix] = forward_tail
            p1_template_bypasses[variant_suffix] = bypass
            end_index = next(
                index
                for index, row in enumerate(generated_rows)
                if "PARALLEL_1_END" in split_tokens(row["fsm_event_ids"])
            )
            p1_template_post_parallel_rows[variant_suffix] = [
                dict(row) for row in generated_rows[end_index + 1 :]
            ]
        variant_rows[variant] = generated_rows

    rebuild_master(
        ROUTE_DIR / "course_07_parking_overlay_master.csv",
        MASTER_SPACING_M,
        source_points,
    )
    rebuild_master(
        ROUTE_DIR / "course_07_parking_smoothed_master.csv",
        FINAL_SPACING_M,
        source_points,
    )
    _, annotations = update_annotations(ROUTE_DIR / "course_09_fsm_annotations.csv")
    rebuild_event_json(
        ROUTE_DIR / "course_07_fsm_route_events.json",
        annotations,
        source_points,
        variant_rows,
    )
    update_manifests(metrics, variant_rows)
    print(json.dumps(metrics, ensure_ascii=False, indent=2))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.parse_args()
    run()


if __name__ == "__main__":
    main()
