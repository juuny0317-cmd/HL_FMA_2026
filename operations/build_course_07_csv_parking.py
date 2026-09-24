#!/usr/bin/env python3
"""Build the active Course 07 branches and move the hill stop upstream.

The recorded source has its own ENU origin.  This builder changes only the
parallel-parking zone, a short, smooth exit join, and the HILL_STOP event.
Other geometry and mission/FSM annotations come from the fair-FSM route files.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
ROUTES = ROOT / "ros2_ws/src/hl_ku_core/routes/course_07_parking_overlay"
SOURCE = ROUTES / "parallel_parking_source.csv"
DATUM = ROOT / "operations/course_07_competition_datum.yaml"
MANIFEST = ROUTES / "course_07_parallel_csv_manifest.json"
VARIANTS = tuple(f"p{p}_t{t}_f{f}" for p in (1, 2) for t in (1, 2) for f in (1, 2))
FIELDS = (
    "s_m", "x_m", "y_m", "target_speed_mps", "mission", "direction",
    "fsm_zone", "fsm_event_ids", "fsm_actions", "fsm_conditions",
    "fsm_hold_sec", "direction_source",
)
ENTRY_BLEND_M = 2.0
EXIT_BLEND_M = 4.0
ENTRY_MAX_JOIN_M = 0.25
EXIT_MAX_JOIN_M = 0.75
HILL_STOP_SHIFT_M = 1.5


def ecef(lat_deg: float, lon_deg: float, height_m: float) -> tuple[float, float, float]:
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    a = 6378137.0
    eccentricity_sq = 6.69437999014e-3
    radius = a / math.sqrt(1 - eccentricity_sq * math.sin(lat) ** 2)
    return (
        (radius + height_m) * math.cos(lat) * math.cos(lon),
        (radius + height_m) * math.cos(lat) * math.sin(lon),
        (radius * (1 - eccentricity_sq) + height_m) * math.sin(lat),
    )


def source_xy(row: dict[str, str], datum: dict[str, float]) -> tuple[float, float]:
    """Rotate and translate one source ENU point into the competition ENU frame."""

    source_lat = math.radians(float(row["origin_latitude_deg"]))
    source_lon = math.radians(float(row["origin_longitude_deg"]))
    east, north, up = float(row["x"]), float(row["y"]), float(row["up_m"] or 0)
    ox, oy, oz = ecef(
        math.degrees(source_lat), math.degrees(source_lon),
        float(row["origin_altitude_m"]),
    )
    x = ox - math.sin(source_lon) * east - math.sin(source_lat) * math.cos(source_lon) * north + math.cos(source_lat) * math.cos(source_lon) * up
    y = oy + math.cos(source_lon) * east - math.sin(source_lat) * math.sin(source_lon) * north + math.cos(source_lat) * math.sin(source_lon) * up
    z = oz + math.cos(source_lat) * north + math.sin(source_lat) * up
    latitude = math.radians(datum["latitude_deg"])
    longitude = math.radians(datum["longitude_deg"])
    dx, dy, dz = (
        value - origin for value, origin in zip(
            (x, y, z),
            ecef(datum["latitude_deg"], datum["longitude_deg"], datum["altitude_m"]),
        )
    )
    return (
        -math.sin(longitude) * dx + math.cos(longitude) * dy,
        -math.sin(latitude) * math.cos(longitude) * dx
        - math.sin(latitude) * math.sin(longitude) * dy
        + math.cos(latitude) * dz,
    )


def distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def xy(row: dict[str, str]) -> tuple[float, float]:
    return float(row["x_m"]), float(row["y_m"])


def smoothstep(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


def _solve_four(matrix: list[list[float]], values: list[float]) -> float:
    """Return the intercept of a small least-squares cubic fit."""

    augmented = [row[:] + [value] for row, value in zip(matrix, values)]
    for column in range(4):
        pivot = max(range(column, 4), key=lambda row: abs(augmented[row][column]))
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        scale = augmented[column][column]
        if abs(scale) < 1e-12:
            raise ValueError("singular parking approach smoothing fit")
        for index in range(column, 5):
            augmented[column][index] /= scale
        for row in range(4):
            if row == column:
                continue
            factor = augmented[row][column]
            for index in range(column, 5):
                augmented[row][index] -= factor * augmented[column][index]
    return augmented[0][4]


def smooth_approach(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Remove centimeter-scale sample kinks while retaining both endpoints."""

    if len(points) < 11:
        raise ValueError("parallel approach has too few samples")
    smoothed: list[tuple[float, float]] = []
    for index in range(len(points)):
        start = max(0, min(index - 5, len(points) - 11))
        sample = [(offset - index, points[offset]) for offset in range(start, start + 11)]
        moments = [sum(t ** order for t, _ in sample) for order in range(7)]
        normal = [[moments[row + column] for column in range(4)] for row in range(4)]
        coords = []
        for axis in (0, 1):
            right = [sum(t ** order * point[axis] for t, point in sample) for order in range(4)]
            coords.append(_solve_four(normal, right))
        smoothed.append((coords[0], coords[1]))
    smoothed[0], smoothed[-1] = points[0], points[-1]
    return smoothed


def read_source() -> tuple[dict[str, list[tuple[float, float]]], dict[str, float]]:
    with DATUM.open(encoding="utf-8") as stream:
        parameters = yaml.safe_load(stream)["gnss_localizer"]["ros__parameters"]
    datum = {
        "latitude_deg": float(parameters["datum_latitude_deg"]),
        "longitude_deg": float(parameters["datum_longitude_deg"]),
        "altitude_m": float(parameters["datum_altitude_m"]),
    }
    with SOURCE.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows or len({(row["origin_latitude_deg"], row["origin_longitude_deg"]) for row in rows}) != 1:
        raise ValueError("parking source must have one consistent geodetic origin")
    sections = {
        name: [source_xy(row, datum) for row in rows if row["segment_id"] == name]
        for name in ("P_entry", "P1_reverse", "P1_exit", "P2_reverse", "P2_exit")
    }
    if any(len(points) < 10 for points in sections.values()):
        raise ValueError("parking source is missing a required branch")
    for option in (1, 2):
        if distance(sections["P_entry"][-1], sections[f"P{option}_reverse"][0]) > 0.005:
            raise ValueError(f"P{option} forward-to-reverse cusp is disconnected")
        if distance(sections[f"P{option}_reverse"][-1], sections[f"P{option}_exit"][0]) > 0.005:
            raise ValueError(f"P{option} reverse-to-forward cusp is disconnected")
    return sections, datum


def _row(point: tuple[float, float], direction: int, source: str) -> dict[str, str]:
    return {
        "s_m": "", "x_m": f"{point[0]:.4f}", "y_m": f"{point[1]:.4f}",
        "target_speed_mps": "0.000", "mission": "PARALLEL_PARK",
        "direction": str(direction), "fsm_zone": "PARALLEL_PARK",
        "fsm_event_ids": "", "fsm_actions": "", "fsm_conditions": "",
        "fsm_hold_sec": "", "direction_source": source,
    }


def _copy_event(destination: dict[str, str], source: dict[str, str]) -> None:
    for name in ("fsm_event_ids", "fsm_actions", "fsm_conditions", "fsm_hold_sec"):
        destination[name] = source[name]


def _curvature(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]) -> float:
    ab, bc, ac = distance(a, b), distance(b, c), distance(a, c)
    if min(ab, bc, ac) < 1e-6:
        return 0.0
    cross = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
    return 2 * cross / (ab * bc * ac)


def _maximum_curvature(points: list[tuple[float, float]]) -> float:
    return max(_curvature(*points[index - 1:index + 2]) for index in range(1, len(points) - 1))


def splice_variant(
    baseline: list[dict[str, str]],
    sections: dict[str, list[tuple[float, float]]],
    option: int,
) -> tuple[list[dict[str, str]], dict[str, float | int]]:
    indices = [index for index, row in enumerate(baseline) if row["fsm_zone"] == "PARALLEL_PARK"]
    if not indices or indices != list(range(indices[0], indices[-1] + 1)):
        raise ValueError("baseline PARALLEL_PARK zone must be contiguous")
    first, last = indices[0], indices[-1]
    events = [baseline[index] for index in indices if baseline[index]["fsm_event_ids"]]
    if len(events) != 4:
        raise ValueError(f"expected four parallel FSM event rows, got {len(events)}")
    expected = [
        "PARALLEL_PARK_START", f"PARALLEL_{option}_FORWARD_TO_REVERSE",
        f"PARALLEL_{option}_PARK_COMPLETE", f"PARALLEL_{option}_END",
    ]
    if any(name not in row["fsm_event_ids"].split("|") for name, row in zip(expected, events)):
        raise ValueError("baseline parallel FSM events do not match source branch")

    approach = sections["P_entry"]
    trim = min(range(len(approach)), key=lambda index: distance(approach[index], xy(baseline[first])))
    entry_gap = distance(approach[trim], xy(baseline[first]))
    if not 35 <= trim <= 60 or entry_gap > ENTRY_MAX_JOIN_M:
        raise ValueError(f"parallel approach trim does not meet old start: index={trim}, gap={entry_gap:.3f} m")
    smoothed_entry = smooth_approach(approach[trim:])
    start_delta = (
        xy(baseline[first])[0] - smoothed_entry[0][0],
        xy(baseline[first])[1] - smoothed_entry[0][1],
    )
    distance_from_start = 0.0
    entry = []
    for index, point in enumerate(smoothed_entry):
        if index:
            distance_from_start += distance(smoothed_entry[index - 1], point)
        weight = 1 - smoothstep(distance_from_start / ENTRY_BLEND_M)
        entry.append((point[0] + weight * start_delta[0], point[1] + weight * start_delta[1]))

    reverse = sections[f"P{option}_reverse"]
    exit_path = sections[f"P{option}_exit"]
    exit_end = exit_path[-1]
    join = min(range(last + 1, min(last + 70, len(baseline))), key=lambda index: distance(exit_end, xy(baseline[index])))
    exit_gap = distance(exit_end, xy(baseline[join]))
    if exit_gap > EXIT_MAX_JOIN_M or any(baseline[index]["fsm_event_ids"] for index in range(last + 1, join + 1)):
        raise ValueError(f"P{option} exit cannot join the original route: gap={exit_gap:.3f} m")
    exit_delta = (exit_end[0] - xy(baseline[join])[0], exit_end[1] - xy(baseline[join])[1])

    prefix = [row.copy() for row in baseline[:first]]
    new_entry = [_row(point, 1, "PARALLEL_CSV_COMMON_FORWARD") for point in entry]
    new_reverse = [_row(point, -1, f"PARALLEL_CSV_P{option}_REVERSE") for point in reverse[1:]]
    new_exit = [_row(point, 1, f"PARALLEL_CSV_P{option}_EXIT") for point in exit_path[1:]]
    _copy_event(new_entry[0], events[0])
    _copy_event(new_reverse[0], events[1])
    _copy_event(new_exit[0], events[2])
    _copy_event(new_exit[-1], events[3])

    suffix = [row.copy() for row in baseline[join + 1:]]
    start_s = float(baseline[join]["s_m"])
    for row in suffix:
        progress = float(row["s_m"]) - start_s
        weight = 1 - smoothstep(progress / EXIT_BLEND_M)
        original = xy(row)
        row["x_m"] = f"{original[0] + weight * exit_delta[0]:.4f}"
        row["y_m"] = f"{original[1] + weight * exit_delta[1]:.4f}"
        if progress < EXIT_BLEND_M:
            row["direction_source"] = "PARALLEL_CSV_EXIT_JOIN"
    output = prefix + new_entry + new_reverse + new_exit + suffix
    previous_xy = xy(output[0])
    accumulated = float(output[0]["s_m"])
    for index, row in enumerate(output[1:], start=1):
        current_xy = xy(row)
        if index >= len(prefix):
            step = distance(previous_xy, current_xy)
            if not 0.00005 < step < 0.35:
                raise ValueError(f"route sample gap {step:.3f} m at index {index}: {previous_xy} -> {current_xy} ({row['direction_source']})")
            accumulated += step
            row["s_m"] = f"{accumulated:.4f}"
        else:
            accumulated = float(row["s_m"])
        previous_xy = current_xy
    if any(float(output[index]["s_m"]) <= float(output[index - 1]["s_m"]) for index in range(1, len(output))):
        raise ValueError("spliced route s_m is not strictly increasing")

    approach_curvature = _maximum_curvature(entry)
    reverse_curvature = _maximum_curvature(reverse)
    exit_curvature = _maximum_curvature(exit_path)
    splice_curvature = max(
        _curvature(xy(output[index - 1]), xy(output[index]), xy(output[index + 1]))
        for index in range(1, len(output) - 1)
        if output[index - 1]["direction"] == output[index]["direction"] == output[index + 1]["direction"]
        and any(
            output[near]["direction_source"].startswith("PARALLEL_CSV")
            for near in (index - 1, index, index + 1)
        )
    )
    if max(approach_curvature, reverse_curvature, exit_curvature, splice_curvature) > 0.80:
        raise ValueError("parallel parking route exceeds 0.80 1/m curvature after splicing")
    report = {
        "source_trim_point_index": trim,
        "entry_gap_before_blending_m": round(entry_gap, 4),
        "source_to_route_exit_join_index": join,
        "exit_gap_before_blending_m": round(exit_gap, 4),
        "entry_max_discrete_curvature_inv_m": round(approach_curvature, 4),
        "reverse_max_discrete_curvature_inv_m": round(reverse_curvature, 4),
        "exit_max_discrete_curvature_inv_m": round(exit_curvature, 4),
        "spliced_max_discrete_curvature_inv_m": round(splice_curvature, 4),
        "length_m": round(float(output[-1]["s_m"]), 4),
        "point_count": len(output),
    }
    return output, report


def shift_hill_stop(rows: list[dict[str, str]]) -> dict[str, float | list[float]]:
    """Move the authored stop exactly 1.5 m upstream along the HILL path."""

    matches = [index for index, row in enumerate(rows) if row["fsm_event_ids"] == "HILL_STOP"]
    if len(matches) != 1:
        raise ValueError(f"expected one HILL_STOP event, got {len(matches)}")
    old_index = matches[0]
    old = rows[old_index]
    if (old["fsm_zone"], old["fsm_actions"], old["fsm_conditions"]) != (
        "HILL", "STOP_THEN_HOLD", "ALWAYS_ONCE"
    ):
        raise ValueError("HILL_STOP metadata does not match the active FSM")
    old_s = float(old["s_m"])
    new_s = round(old_s - HILL_STOP_SHIFT_M, 4)
    insert_index = next(index for index, row in enumerate(rows) if float(row["s_m"]) > new_s)
    if not 0 < insert_index < old_index:
        raise ValueError("new HILL_STOP does not precede the old point")
    before, after = rows[insert_index - 1], rows[insert_index]
    if any(
        row["fsm_zone"] != "HILL" or row["direction"] != "1" or row["fsm_event_ids"]
        for row in rows[insert_index:old_index]
    ):
        raise ValueError("HILL_STOP shift crosses another event or mission zone")
    low_s, high_s = float(before["s_m"]), float(after["s_m"])
    fraction = (new_s - low_s) / (high_s - low_s)
    if not 0 < fraction < 1:
        raise ValueError("new HILL_STOP point is not inside a route segment")
    shifted = before.copy()
    shifted["s_m"] = f"{new_s:.4f}"
    for coordinate in ("x_m", "y_m"):
        low, high = float(before[coordinate]), float(after[coordinate])
        shifted[coordinate] = f"{low + fraction * (high - low):.4f}"
    _copy_event(shifted, old)
    shifted["direction_source"] = "HILL_STOP_SHIFTED_1_5M"
    for field in ("fsm_event_ids", "fsm_actions", "fsm_conditions", "fsm_hold_sec"):
        old[field] = ""
    rows.insert(insert_index, shifted)
    return {
        "old_s_m": old_s,
        "new_s_m": new_s,
        "upstream_shift_m": HILL_STOP_SHIFT_M,
        "old_xy_m": [float(old["x_m"]), float(old["y_m"])],
        "new_xy_m": [float(shifted["x_m"]), float(shifted["y_m"])],
    }


def render_csv(rows: list[dict[str, str]]) -> str:
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=FIELDS, lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
    return stream.getvalue()


def build() -> tuple[dict[Path, str], dict]:
    sections, datum = read_source()
    rendered: dict[Path, str] = {}
    reports: dict[str, dict] = {}
    for name in VARIANTS:
        path = ROUTES / f"course_07_{name}_fair_fsm_preview.csv"
        with path.open(newline="", encoding="utf-8") as stream:
            baseline = list(csv.DictReader(stream))
        option = int(name[1])
        rows, report = splice_variant(baseline, sections, option)
        report["hill_stop"] = shift_hill_stop(rows)
        rendered[ROUTES / f"course_07_{name}_parallel_csv.csv"] = render_csv(rows)
        reports[name] = report
    manifest = {
        "description": "Course 07 CSV parking branches and HILL_STOP moved 1.5 m upstream",
        "source_sha256": hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        "source_file": SOURCE.name,
        "source_segments_used": ["P_entry", "P1_reverse", "P1_exit", "P2_reverse", "P2_exit"],
        "source_segments_trimmed": ["to_forward3", "P_entry before trim index"],
        "coordinate_frame": "competition datum ENU",
        "datum": datum,
        "entry_blend_m": ENTRY_BLEND_M,
        "exit_blend_m": EXIT_BLEND_M,
        "hill_stop_upstream_shift_m": HILL_STOP_SHIFT_M,
        "software_route_generated": True,
        "field_geometry_validated": False,
        "variants": reports,
    }
    rendered[MANIFEST] = json.dumps(manifest, ensure_ascii=False, indent=2) + "\n"
    return rendered, manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="verify generated files are current")
    options = parser.parse_args()
    rendered, manifest = build()
    if options.check:
        stale = [path for path, text in rendered.items() if not path.is_file() or path.read_text(encoding="utf-8") != text]
        if stale:
            raise SystemExit("stale parallel parking files: " + ", ".join(str(path) for path in stale))
        print("current: all Course 07 parallel parking routes")
        return 0
    for path, content in rendered.items():
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(content, encoding="utf-8")
        temporary.replace(path)
    for name, report in manifest["variants"].items():
        print(f"{name}: {report['length_m']:.3f} m, entry trim {report['source_trim_point_index']}, exit join {report['source_to_route_exit_join_index']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
