"""Prepare a sparse measured route for deterministic vehicle tracking."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Sequence

from .geometry import antenna_to_base
from .route import Route, Waypoint


def densify_route(route: Route, maximum_spacing_m: float) -> Route:
    """Linearly subdivide route segments without changing the measured polyline."""
    if not math.isfinite(maximum_spacing_m) or maximum_spacing_m <= 0.0:
        raise ValueError("maximum_spacing_m must be finite and positive")

    source = route.waypoints
    dense = [
        Waypoint(
            index=0,
            s_m=0.0,
            x_m=source[0].x_m,
            y_m=source[0].y_m,
            target_speed_mps=source[0].target_speed_mps,
            mission=source[0].mission,
            direction=source[0].direction,
            fsm_zone=source[0].fsm_zone,
            fsm_event_ids=source[0].fsm_event_ids,
            fsm_actions=source[0].fsm_actions,
            fsm_conditions=source[0].fsm_conditions,
            fsm_hold_sec=source[0].fsm_hold_sec,
            direction_source=source[0].direction_source,
        )
    ]
    accumulated = 0.0
    for start, end in zip(source, source[1:]):
        dx = end.x_m - start.x_m
        dy = end.y_m - start.y_m
        length = math.hypot(dx, dy)
        subdivisions = max(1, math.ceil(length / maximum_spacing_m))
        step_length = length / subdivisions
        for step in range(1, subdivisions + 1):
            ratio = step / subdivisions
            accumulated += step_length
            at_endpoint = step == subdivisions
            dense.append(
                Waypoint(
                    index=len(dense),
                    s_m=accumulated,
                    x_m=start.x_m + dx * ratio,
                    y_m=start.y_m + dy * ratio,
                    target_speed_mps=(
                        end.target_speed_mps
                        if at_endpoint
                        else start.target_speed_mps
                    ),
                    mission=end.mission if at_endpoint else start.mission,
                    direction=end.direction if at_endpoint else start.direction,
                    fsm_zone=end.fsm_zone if at_endpoint else start.fsm_zone,
                    fsm_event_ids=end.fsm_event_ids if at_endpoint else start.fsm_event_ids,
                    fsm_actions=end.fsm_actions if at_endpoint else start.fsm_actions,
                    fsm_conditions=end.fsm_conditions if at_endpoint else start.fsm_conditions,
                    fsm_hold_sec=end.fsm_hold_sec if at_endpoint else start.fsm_hold_sec,
                    direction_source=end.direction_source if at_endpoint else start.direction_source,
                )
            )
    return Route(dense)


def antenna_path_to_base_link(
    route: Route,
    base_to_antenna_x_m: float,
    base_to_antenna_y_m: float,
) -> Route:
    """Convert an ANT1 phase-center path into the rear-axle base_link path."""
    if not all(
        math.isfinite(value)
        for value in (base_to_antenna_x_m, base_to_antenna_y_m)
    ):
        raise ValueError("antenna lever arm must be finite")

    source = route.waypoints
    shifted: list[Waypoint] = []
    accumulated = 0.0
    previous_xy: tuple[float, float] | None = None
    for index, waypoint in enumerate(source):
        if index == 0:
            start, end = source[0], source[1]
        elif index == len(source) - 1:
            start, end = source[-2], source[-1]
        else:
            start, end = source[index - 1], source[index + 1]
        yaw = math.atan2(end.y_m - start.y_m, end.x_m - start.x_m)
        x_m, y_m = antenna_to_base(
            waypoint.x_m,
            waypoint.y_m,
            yaw,
            base_to_antenna_x_m,
            base_to_antenna_y_m,
        )
        if previous_xy is not None:
            accumulated += math.hypot(x_m - previous_xy[0], y_m - previous_xy[1])
        shifted.append(
            Waypoint(
                index=index,
                s_m=accumulated,
                x_m=x_m,
                y_m=y_m,
                target_speed_mps=waypoint.target_speed_mps,
                mission=waypoint.mission,
                direction=waypoint.direction,
                fsm_zone=waypoint.fsm_zone,
                fsm_event_ids=waypoint.fsm_event_ids,
                fsm_actions=waypoint.fsm_actions,
                fsm_conditions=waypoint.fsm_conditions,
                fsm_hold_sec=waypoint.fsm_hold_sec,
                direction_source=waypoint.direction_source,
            )
        )
        previous_xy = (x_m, y_m)
    return Route(shifted)


def save_route(route: Route, output_file: str | Path) -> None:
    path = Path(output_file).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(
            (
                "s_m", "x_m", "y_m", "target_speed_mps", "mission", "direction",
                "fsm_zone", "fsm_event_ids", "fsm_actions", "fsm_conditions",
                "fsm_hold_sec", "direction_source",
            )
        )
        for waypoint in route.waypoints:
            writer.writerow(
                (
                    f"{waypoint.s_m:.4f}",
                    f"{waypoint.x_m:.4f}",
                    f"{waypoint.y_m:.4f}",
                    f"{waypoint.target_speed_mps:.3f}",
                    waypoint.mission,
                    waypoint.direction,
                    waypoint.fsm_zone,
                    waypoint.fsm_event_ids,
                    waypoint.fsm_actions,
                    waypoint.fsm_conditions,
                    f"{waypoint.fsm_hold_sec:.3f}",
                    waypoint.direction_source,
                )
            )
    temporary.replace(path)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Densify a measured HL_KU route without smoothing its geometry"
    )
    parser.add_argument("input_file")
    parser.add_argument("output_file")
    parser.add_argument("--variant-id", help="select one variant from a mission route bundle")
    parser.add_argument("--maximum-spacing-m", type=float, default=0.50)
    parser.add_argument("--base-to-antenna-x-m", type=float, default=0.0)
    parser.add_argument("--base-to-antenna-y-m", type=float, default=0.0)
    return parser


def main(argv: Sequence[str] | None = None) -> None:
    arguments = _parser().parse_args(argv)
    route = Route.load_csv(arguments.input_file, arguments.variant_id)
    prepared = densify_route(route, arguments.maximum_spacing_m)
    prepared = antenna_path_to_base_link(
        prepared,
        arguments.base_to_antenna_x_m,
        arguments.base_to_antenna_y_m,
    )
    prepared = densify_route(prepared, arguments.maximum_spacing_m)
    save_route(prepared, arguments.output_file)
    print(
        f"prepared {len(prepared.waypoints)} waypoints, "
        f"length={prepared.waypoints[-1].s_m:.3f} m, "
        f"output={Path(arguments.output_file).expanduser()}"
    )


if __name__ == "__main__":
    main()
