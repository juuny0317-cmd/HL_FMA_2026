"""Cut and rigidly place a measured route without changing its scale."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Sequence

import yaml

from .route import Route, Waypoint
from .route_prepare import save_route


def slice_and_place(
    source: Route,
    start_s_m: float,
    length_m: float,
    target_x_m: float,
    target_y_m: float,
    target_heading_deg: float,
    speed_mps: float,
    preserve_source_profile: bool = False,
    placement_source: Route | None = None,
) -> Route:
    if not all(
        math.isfinite(value)
        for value in (
            start_s_m,
            length_m,
            target_x_m,
            target_y_m,
            target_heading_deg,
            speed_mps,
        )
    ):
        raise ValueError("all route placement values must be finite")
    if start_s_m < 0.0 or length_m <= 0.0 or speed_mps <= 0.0:
        raise ValueError("start must be non-negative; length and speed must be positive")
    endpoint_tolerance = 1.0e-6
    selected = [
        point
        for point in source.waypoints
        if (
            start_s_m - endpoint_tolerance
            <= point.s_m
            <= start_s_m + length_m + endpoint_tolerance
        )
    ]
    if len(selected) < 2:
        raise ValueError("selected s range contains fewer than two waypoints")
    if not preserve_source_profile and any(point.direction != 1 for point in selected):
        raise ValueError("initial Mando route slices must use forward direction=1")

    # Parking alternatives live in one common source-map coordinate frame.  A
    # candidate may have a slightly different first sample/tangent even while
    # its shared approach overlaps the active route.  Deriving one placement
    # from every candidate independently rotates those overlapping approaches
    # apart.  When a placement source is supplied, use its origin, progress
    # origin and heading for every candidate so their original relative
    # geometry survives the rigid placement.
    reference = source if placement_source is None else placement_source
    reference_selected = [
        point
        for point in reference.waypoints
        if (
            start_s_m - endpoint_tolerance
            <= point.s_m
            <= start_s_m + length_m + endpoint_tolerance
        )
    ]
    if len(reference_selected) < 2:
        raise ValueError("placement source range contains fewer than two waypoints")
    source_heading = math.atan2(
        reference_selected[1].y_m - reference_selected[0].y_m,
        reference_selected[1].x_m - reference_selected[0].x_m,
    )
    first_direction = (
        reference_selected[0].direction if preserve_source_profile else 1
    )
    # Route points are ordered in the direction of travel.  During reverse the
    # vehicle heading is therefore 180 degrees opposite the route tangent.
    target_motion_heading = math.radians(target_heading_deg)
    if first_direction < 0:
        target_motion_heading += math.pi
    rotation = target_motion_heading - source_heading
    cosine, sine = math.cos(rotation), math.sin(rotation)
    origin_x, origin_y, origin_s = (
        reference_selected[0].x_m,
        reference_selected[0].y_m,
        reference_selected[0].s_m,
    )
    output: list[Waypoint] = []
    for index, point in enumerate(selected):
        local_x = point.x_m - origin_x
        local_y = point.y_m - origin_y
        output.append(
            Waypoint(
                index=index,
                s_m=point.s_m - origin_s,
                x_m=target_x_m + cosine * local_x - sine * local_y,
                y_m=target_y_m + sine * local_x + cosine * local_y,
                target_speed_mps=0.0 if index == len(selected) - 1 else speed_mps,
                mission=(
                    "FINISH"
                    if index == len(selected) - 1
                    else point.mission if preserve_source_profile else "NORMAL"
                ),
                direction=point.direction if preserve_source_profile else 1,
                fsm_zone=(
                    "FINISH"
                    if index == len(selected) - 1 and point.fsm_zone
                    else point.fsm_zone
                ),
                fsm_event_ids=(
                    "TEST_SEGMENT_END"
                    if index == len(selected) - 1 and point.fsm_zone
                    and point.fsm_zone != "FINISH"
                    else point.fsm_event_ids
                ),
                fsm_actions=(
                    "STOP"
                    if index == len(selected) - 1 and point.fsm_zone
                    and point.fsm_zone != "FINISH"
                    else point.fsm_actions
                ),
                fsm_conditions=(
                    "ALWAYS_ONCE"
                    if index == len(selected) - 1 and point.fsm_zone
                    and point.fsm_zone != "FINISH"
                    else point.fsm_conditions
                ),
                fsm_hold_sec=point.fsm_hold_sec,
                direction_source=point.direction_source,
            )
        )
    return Route(output)


def calibration_document(gps_config_file: str | Path, route_file: str | Path) -> dict:
    with Path(gps_config_file).expanduser().open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    localizer = config.get("gnss_localizer", {}).get("ros__parameters", {})
    keys = (
        "datum_latitude_deg",
        "datum_longitude_deg",
        "datum_altitude_m",
        "base_to_ant1_x_m",
        "base_to_ant1_y_m",
        "heading_mount_offset_deg",
    )
    missing = [key for key in keys if not isinstance(localizer.get(key), (int, float))]
    if localizer.get("datum_configured") is not True or missing:
        raise ValueError("gps config has no complete configured datum/antenna transform")
    return {
        "project": {
            "purpose": "HL Mando supervised campus route slice",
            "measured_at": "",
            "operators": [],
        },
        "gnss": {key: localizer[key] for key in keys},
        "course": {
            "route_file": Path(route_file).name,
            "route_calibrated": False,
        },
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Cut a measured vehicle route and rigidly place it in ENU (no scaling)"
    )
    parser.add_argument("source_route", type=Path)
    parser.add_argument("output_route", type=Path)
    parser.add_argument("--variant-id", help="select one variant from a mission route bundle")
    parser.add_argument("--start-s-m", type=float, required=True)
    parser.add_argument("--length-m", type=float, required=True)
    parser.add_argument("--target-x-m", type=float, required=True)
    parser.add_argument("--target-y-m", type=float, required=True)
    parser.add_argument("--target-heading-deg", type=float, required=True)
    parser.add_argument("--speed-mps", type=float, default=0.30)
    parser.add_argument(
        "--preserve-source-profile",
        action="store_true",
        help="preserve mission and forward/reverse direction from the source slice",
    )
    parser.add_argument("--gps-config", type=Path)
    parser.add_argument("--calibration-output", type=Path)
    parser.add_argument("--force", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> None:
    options = _parser().parse_args(argv)
    output = options.output_route.expanduser()
    calibration_output = (
        options.calibration_output.expanduser()
        if options.calibration_output is not None
        else None
    )
    if output.exists() and not options.force:
        raise SystemExit(f"output exists (use --force): {output}")
    if calibration_output is not None and calibration_output.exists() and not options.force:
        raise SystemExit(f"calibration output exists (use --force): {calibration_output}")
    if (options.gps_config is None) != (calibration_output is None):
        raise SystemExit("--gps-config and --calibration-output must be used together")
    source = Route.load_csv(options.source_route, options.variant_id)
    placed = slice_and_place(
        source,
        options.start_s_m,
        options.length_m,
        options.target_x_m,
        options.target_y_m,
        options.target_heading_deg,
        options.speed_mps,
        options.preserve_source_profile,
    )
    save_route(placed, output)
    if calibration_output is not None:
        calibration_output.parent.mkdir(parents=True, exist_ok=True)
        document = calibration_document(options.gps_config, output)
        calibration_output.write_text(
            yaml.safe_dump(document, allow_unicode=True, sort_keys=False),
            encoding="utf-8",
        )
    print(
        f"saved {len(placed.waypoints)} points, "
        f"length={placed.waypoints[-1].s_m:.3f} m, scale=1.0, route={output}"
    )
    if calibration_output is not None:
        print(f"calibration draft={calibration_output} (route_calibrated remains false)")


if __name__ == "__main__":
    main()
