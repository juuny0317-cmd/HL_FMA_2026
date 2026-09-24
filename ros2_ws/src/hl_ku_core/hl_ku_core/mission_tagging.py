"""Project preview FSM zones onto a measured vehicle route.

The course-07 FSM previews contain reviewed mission boundaries, but their
geometry includes parking/end-lane branch alternatives and their speed is
deliberately zero.  This module transfers only the mission meaning to the
measured route.  Coordinates, speed commands and drive direction stay owned by
the measured route.
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence

from .route import ALLOWED_MISSIONS, FSM_ZONE_TO_MISSION


ZONE_TO_MISSION = FSM_ZONE_TO_MISSION


@dataclass(frozen=True)
class MissionBoundary:
    reference_index: int
    mission: str
    x_m: float
    y_m: float
    reference_s_m: float


@dataclass(frozen=True)
class BoundaryMapping:
    mission: str
    reference_index: int
    target_index: int
    reference_s_m: float
    target_s_m: float
    distance_m: float


def _required_columns(
    rows: Sequence[Mapping[str, str]], required: set[str], label: str
) -> None:
    if not rows:
        raise ValueError(f"{label} is empty")
    missing = required.difference(rows[0])
    if missing:
        names = ", ".join(sorted(missing))
        raise ValueError(f"{label} is missing columns: {names}")


def _finite(
    row: Mapping[str, str], field: str, label: str, index: int
) -> float:
    try:
        value = float(row[field])
    except (KeyError, TypeError, ValueError) as error:
        message = f"{label} row {index + 2} has invalid {field}"
        raise ValueError(message) from error
    if not math.isfinite(value):
        raise ValueError(f"{label} row {index + 2} has non-finite {field}")
    return value


def _reference_mission(row: Mapping[str, str], index: int) -> str:
    zone = (row.get("fsm_zone", "") or "").strip().upper()
    if zone:
        try:
            return ZONE_TO_MISSION[zone]
        except KeyError as error:
            raise ValueError(
                f"reference row {index + 2} has unknown fsm_zone: {zone}"
            ) from error
    mission = (row.get("mission", "") or "NORMAL").strip().upper()
    if mission not in ALLOWED_MISSIONS:
        raise ValueError(
            f"reference row {index + 2} has unknown mission: {mission}"
        )
    return mission


def extract_mission_boundaries(
    reference_rows: Sequence[Mapping[str, str]],
) -> list[MissionBoundary]:
    """Convert preview-zone transitions to runtime mission boundaries."""
    _required_columns(
        reference_rows,
        {"x_m", "y_m", "s_m", "mission"},
        "reference route",
    )
    boundaries: list[MissionBoundary] = []
    previous_mission = ""
    for index, row in enumerate(reference_rows):
        mission = _reference_mission(row, index)
        if mission == previous_mission:
            continue
        boundaries.append(
            MissionBoundary(
                reference_index=index,
                mission=mission,
                x_m=_finite(row, "x_m", "reference route", index),
                y_m=_finite(row, "y_m", "reference route", index),
                reference_s_m=_finite(row, "s_m", "reference route", index),
            )
        )
        previous_mission = mission

    # The fairing preview contains a short ROUTE tail after the finish-line
    # marker because it continues to a separate full-stop point.  The measured
    # route has a different selectable final-lane branch.  Keep END_LANE armed
    # through that tail and reserve FINISH for the measured route's final
    # point.
    if (
        len(boundaries) >= 3
        and boundaries[-3].mission == "END_LANE"
        and boundaries[-2].mission == "NORMAL"
        and boundaries[-1].mission == "FINISH"
    ):
        del boundaries[-2]

    if not boundaries or boundaries[0].mission != "NORMAL":
        raise ValueError("reference route must start with NORMAL")
    if boundaries[-1].mission != "FINISH":
        raise ValueError("reference route must end with FINISH")
    return boundaries


def map_boundaries_to_route(
    target_rows: Sequence[Mapping[str, str]],
    boundaries: Sequence[MissionBoundary],
    maximum_mapping_distance_m: float = 1.0,
) -> list[BoundaryMapping]:
    """Map ordered XY boundaries to ordered target indices.

    A monotonically increasing index constraint prevents a later boundary from
    snapping to an earlier pass when the route crosses or revisits an area.
    """
    _required_columns(
        target_rows,
        {"s_m", "x_m", "y_m", "target_speed_mps", "mission", "direction"},
        "target route",
    )
    if len(target_rows) < 2:
        raise ValueError("target route needs at least two points")
    if (
        not math.isfinite(maximum_mapping_distance_m)
        or maximum_mapping_distance_m <= 0
    ):
        raise ValueError(
            "maximum_mapping_distance_m must be finite and positive"
        )

    target_xy = [
        (
            _finite(row, "x_m", "target route", index),
            _finite(row, "y_m", "target route", index),
        )
        for index, row in enumerate(target_rows)
    ]
    target_s = [
        _finite(row, "s_m", "target route", index)
        for index, row in enumerate(target_rows)
    ]
    mappings: list[BoundaryMapping] = []
    previous_index = -1
    for boundary_index, boundary in enumerate(boundaries):
        if boundary_index == 0:
            target_index = 0
        elif boundary.mission == "FINISH":
            target_index = len(target_rows) - 1
        else:
            search_start = previous_index + 1
            # The final point is always reserved for FINISH.
            search_stop = len(target_rows) - 1
            if search_start >= search_stop:
                raise ValueError(
                    "not enough ordered target points for mission boundaries"
                )
            target_index = min(
                range(search_start, search_stop),
                key=lambda index: (
                    (target_xy[index][0] - boundary.x_m) ** 2
                    + (target_xy[index][1] - boundary.y_m) ** 2
                ),
            )
        distance = math.hypot(
            target_xy[target_index][0] - boundary.x_m,
            target_xy[target_index][1] - boundary.y_m,
        )
        if (
            boundary.mission != "FINISH"
            and distance > maximum_mapping_distance_m
        ):
            raise ValueError(
                f"{boundary.mission} boundary at reference "
                f"s={boundary.reference_s_m:.3f} m "
                f"is {distance:.3f} m from the measured route "
                f"(limit {maximum_mapping_distance_m:.3f} m)"
            )
        mappings.append(
            BoundaryMapping(
                mission=boundary.mission,
                reference_index=boundary.reference_index,
                target_index=target_index,
                reference_s_m=boundary.reference_s_m,
                target_s_m=target_s[target_index],
                distance_m=distance,
            )
        )
        previous_index = target_index
    return mappings


def apply_mission_tags(
    target_rows: Sequence[Mapping[str, str]],
    mappings: Sequence[BoundaryMapping],
) -> list[dict[str, str]]:
    """Return target rows with only the mission field replaced."""
    if not mappings or mappings[0].target_index != 0:
        raise ValueError("mission mapping must start at target index 0")
    if mappings[-1].mission != "FINISH":
        raise ValueError("mission mapping must end with FINISH")
    output = [dict(row) for row in target_rows]
    for index, mapping in enumerate(mappings):
        stop = (
            mappings[index + 1].target_index
            if index + 1 < len(mappings)
            else len(output)
        )
        for target_index in range(mapping.target_index, stop):
            output[target_index]["mission"] = mapping.mission
    return output


def read_csv(path: str | Path) -> tuple[list[str], list[dict[str, str]]]:
    with Path(path).expanduser().open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        return list(reader.fieldnames or ()), rows


def write_csv(
    path: str | Path,
    fieldnames: Sequence[str],
    rows: Sequence[Mapping[str, str]],
) -> None:
    destination = Path(path).expanduser()
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    with temporary.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=fieldnames, lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(rows)
    temporary.replace(destination)


def tag_route(
    input_file: str | Path,
    reference_file: str | Path,
    output_file: str | Path,
    maximum_mapping_distance_m: float = 1.0,
) -> list[BoundaryMapping]:
    fieldnames, target_rows = read_csv(input_file)
    _reference_fields, reference_rows = read_csv(reference_file)
    boundaries = extract_mission_boundaries(reference_rows)
    mappings = map_boundaries_to_route(
        target_rows,
        boundaries,
        maximum_mapping_distance_m=maximum_mapping_distance_m,
    )
    output_rows = apply_mission_tags(target_rows, mappings)
    write_csv(output_file, fieldnames, output_rows)
    return mappings


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Transfer reviewed FSM zones from a preview to a measured route; "
            "only the mission column is changed"
        )
    )
    parser.add_argument("input_file", help="measured drive route CSV")
    parser.add_argument("reference_file", help="fair_fsm_preview CSV")
    parser.add_argument(
        "output_file", help="tagged drive route CSV; may equal input"
    )
    parser.add_argument(
        "--maximum-mapping-distance-m", type=float, default=1.0
    )
    return parser


def main(argv: Sequence[str] | None = None) -> None:
    arguments = _parser().parse_args(argv)
    mappings = tag_route(
        arguments.input_file,
        arguments.reference_file,
        arguments.output_file,
        maximum_mapping_distance_m=arguments.maximum_mapping_distance_m,
    )
    for mapping in mappings:
        if mapping.mission == "FINISH":
            mapping_detail = (
                "forced measured-route final point "
                f"(reference separation={mapping.distance_m:.3f} m)"
            )
        else:
            mapping_detail = f"mapping_error={mapping.distance_m:.3f} m"
        print(
            f"{mapping.mission:15s} target index={mapping.target_index:4d} "
            f"s={mapping.target_s_m:8.3f} m "
            f"{mapping_detail}"
        )
    print(f"mission-tagged route: {Path(arguments.output_file).expanduser()}")


if __name__ == "__main__":
    main()
