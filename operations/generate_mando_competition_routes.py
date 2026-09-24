#!/usr/bin/env python3
"""Build fixed Course 07 race and function-check routes.

The full race route comes from the mission bundle.  The bend and S-curve
function-check routes come from the official UTM reference polylines converted
to the same Course 07 ENU datum.  Generated routes keep the measured scale and
orientation. Existing outputs are left untouched unless --force is supplied.
A stale race route is rejected so field corrections are never silently
overwritten and an outdated route cannot be sent to the vehicle.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
CORE = ROOT / "ros2_ws" / "src" / "hl_ku_core"
sys.path.insert(0, str(CORE))

from hl_ku_core.mando_route_slice import calibration_document, slice_and_place
from hl_ku_core.route import Route, Waypoint
from hl_ku_core.route_prepare import densify_route, save_route

SOURCE = CORE / "routes" / "course_07_mission.csv"
GPS = CORE / "config" / "mando_competition_gps.yaml"
ROUTE_DIR = CORE / "routes" / "mando" / "competition"
CAL_DIR = CORE / "config" / "mando" / "competition"

SEGMENTS = (
    ("course_07_full", 0.0, 716.0),
)

REFERENCE_DIR = ROUTE_DIR / "reference"
FUNCTION_CHECK_ROUTES = (
    (
        "course_07_bend",
        REFERENCE_DIR / "course_07_bend_reference.csv",
        "NORMAL",
        "BEND",
        "BEND_START",
    ),
    (
        "course_07_s_curve",
        REFERENCE_DIR / "course_07_s_curve_reference.csv",
        "S_OBSTACLE",
        "S_CURVE",
        "S_CURVE_START",
    ),
)


def build(name: str, start_s: float, length: float, force: bool) -> None:
    output = ROUTE_DIR / f"{name}.csv"
    calibration = CAL_DIR / f"{name}_calibration.yaml"
    source = Route.load_csv(SOURCE, "p1_t1_f1")
    selected = [
        point for point in source.waypoints
        if start_s - 1.0e-6 <= point.s_m <= start_s + length + 1.0e-6
    ]
    if len(selected) < 2:
        raise RuntimeError(f"{name}: fewer than two source waypoints")
    heading = math.degrees(math.atan2(
        selected[1].y_m - selected[0].y_m,
        selected[1].x_m - selected[0].x_m,
    ))
    expected = slice_and_place(
        source, start_s, length, selected[0].x_m, selected[0].y_m,
        heading, 0.30, preserve_source_profile=True,
    )
    if output.exists() and not force:
        route = Route.load_csv(output)
        if len(route.waypoints) != len(expected.waypoints) or any(
            abs(actual.s_m - wanted.s_m) > 0.001
            or abs(actual.x_m - wanted.x_m) > 0.001
            or abs(actual.y_m - wanted.y_m) > 0.001
            or actual.direction != wanted.direction
            or actual.mission != wanted.mission
            or actual.fsm_zone != wanted.fsm_zone
            or actual.fsm_event_ids != wanted.fsm_event_ids
            or actual.fsm_actions != wanted.fsm_actions
            or actual.fsm_conditions != wanted.fsm_conditions
            or abs(actual.fsm_hold_sec - wanted.fsm_hold_sec) > 0.001
            or abs(actual.target_speed_mps - wanted.target_speed_mps) > 0.001
            for actual, wanted in zip(route.waypoints, expected.waypoints)
        ):
            raise RuntimeError(
                f"{output} differs from course_07_mission.csv; review field "
                "corrections or rebuild with --force"
            )
        print(f"keep {output} ({route.waypoints[-1].s_m:.3f} m)")
    else:
        route = expected
        save_route(route, output)
        print(f"write {output} ({route.waypoints[-1].s_m:.3f} m)")
    if not calibration.exists() or force:
        calibration.parent.mkdir(parents=True, exist_ok=True)
        document = calibration_document(GPS, output)
        document["project"]["purpose"] = "HL Mando Course 07 field route"
        document["course"]["source_variant_id"] = "p1_t1_f1"
        document["course"]["source_start_s_m"] = start_s
        document["course"]["source_length_m"] = length
        document["course"]["route_calibrated"] = False
        calibration.write_text(
            yaml.safe_dump(document, allow_unicode=True, sort_keys=False),
            encoding="utf-8",
        )
        print(f"write {calibration}")


def reference_route(
    source_file: Path,
    mission: str,
    fsm_zone: str,
    start_event: str,
) -> Route:
    """Load one ENU function-check polyline and add deterministic FSM metadata."""

    with source_file.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 2:
        raise RuntimeError(f"{source_file}: fewer than two reference points")
    points: list[Waypoint] = []
    accumulated = 0.0
    previous_xy: tuple[float, float] | None = None
    for index, row in enumerate(rows):
        x_m = float(row["x_m"])
        y_m = float(row["y_m"])
        if previous_xy is not None:
            accumulated += math.hypot(x_m - previous_xy[0], y_m - previous_xy[1])
        final = index == len(rows) - 1
        points.append(
            Waypoint(
                index=index,
                s_m=accumulated,
                x_m=x_m,
                y_m=y_m,
                target_speed_mps=0.0 if final else 0.30,
                mission="FINISH" if final else mission,
                direction=1,
                fsm_zone="FINISH" if final else fsm_zone,
                fsm_event_ids=(
                    "TEST_SEGMENT_END" if final else start_event if index == 0 else ""
                ),
                fsm_actions=(
                    "STOP" if final else "ENTER_ZONE" if index == 0 else ""
                ),
                fsm_conditions="ALWAYS_ONCE" if final else "",
                fsm_hold_sec=0.0,
                direction_source="FUNCTION_CHECK_TXT",
            )
        )
        previous_xy = (x_m, y_m)
    return densify_route(Route(points), 0.10)


def build_function_check(
    name: str,
    source_file: Path,
    mission: str,
    fsm_zone: str,
    start_event: str,
    force: bool,
) -> None:
    output = ROUTE_DIR / f"{name}.csv"
    calibration = CAL_DIR / f"{name}_calibration.yaml"
    if output.exists() and not force:
        route = Route.load_csv(output)
        print(f"keep {output} ({route.waypoints[-1].s_m:.3f} m)")
    else:
        route = reference_route(source_file, mission, fsm_zone, start_event)
        save_route(route, output)
        print(f"write {output} ({route.waypoints[-1].s_m:.3f} m)")
    if not calibration.exists() or force:
        calibration.parent.mkdir(parents=True, exist_ok=True)
        document = calibration_document(GPS, output)
        document["project"]["purpose"] = "HL Mando Course 07 function-check route"
        document["course"]["source_reference_file"] = str(
            source_file.relative_to(ROOT)
        )
        document["course"]["source_start_s_m"] = 0.0
        document["course"]["source_length_m"] = route.waypoints[-1].s_m
        document["course"]["route_calibrated"] = False
        calibration.write_text(
            yaml.safe_dump(document, allow_unicode=True, sort_keys=False),
            encoding="utf-8",
        )
        print(f"write {calibration}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--force", action="store_true")
    parser.add_argument(
        "--full-only",
        action="store_true",
        help="prepare only the full race route used by the competition TUI",
    )
    options = parser.parse_args()
    for values in SEGMENTS:
        build(*values, force=options.force)
    if not options.full_only:
        for values in FUNCTION_CHECK_ROUTES:
            build_function_check(*values, force=options.force)


if __name__ == "__main__":
    main()
