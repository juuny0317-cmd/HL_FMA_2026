import csv
import math
from pathlib import Path

import pytest

from hl_ku_core.mission_tagging import (
    apply_mission_tags,
    extract_mission_boundaries,
    map_boundaries_to_route,
    tag_route,
)


FIELDS = ["s_m", "x_m", "y_m", "target_speed_mps", "mission", "direction"]


def target_row(s_m: float) -> dict[str, str]:
    return {
        "s_m": f"{s_m:.1f}",
        "x_m": f"{s_m:.1f}",
        "y_m": "0.0",
        "target_speed_mps": "0.3",
        "mission": "NORMAL",
        "direction": "1",
    }


def reference_row(
    s_m: float, zone: str, mission: str = "NORMAL"
) -> dict[str, str]:
    return {
        **target_row(s_m),
        "mission": mission,
        "fsm_zone": zone,
    }


def test_preview_zones_become_runtime_missions_without_changing_drive_data():
    target = [target_row(float(index)) for index in range(12)]
    target[-1]["target_speed_mps"] = "0.0"
    target[-1]["mission"] = "FINISH"
    reference = [
        reference_row(0.0, "ROUTE"),
        reference_row(1.0, "HILL", "HILL"),
        reference_row(2.0, "HILL", "HILL"),
        reference_row(3.0, "S_CURVE"),
        reference_row(4.0, "S_CURVE"),
        reference_row(5.0, "TRAFFIC", "TRAFFIC"),
        reference_row(6.0, "T_PARK", "PERP_PARK"),
        reference_row(7.0, "DUMMY", "DUMMY"),
        reference_row(8.0, "PARALLEL_PARK", "PARALLEL_PARK"),
        reference_row(9.0, "END_LANE", "END_LANE"),
        reference_row(10.0, "ROUTE"),
        reference_row(11.0, "FINISH", "FINISH"),
    ]

    boundaries = extract_mission_boundaries(reference)
    mappings = map_boundaries_to_route(target, boundaries)
    tagged = apply_mission_tags(target, mappings)

    assert [row["mission"] for row in tagged] == [
        "NORMAL",
        "HILL",
        "HILL",
        "S_OBSTACLE",
        "S_OBSTACLE",
        "TRAFFIC",
        "PERP_PARK",
        "DUMMY",
        "PARALLEL_PARK",
        "END_LANE",
        "END_LANE",
        "FINISH",
    ]
    for before, after in zip(target, tagged):
        for field in ("s_m", "x_m", "y_m", "target_speed_mps", "direction"):
            assert after[field] == before[field]


def test_mapping_is_monotonic_at_a_revisited_coordinate():
    target = [target_row(float(index)) for index in range(8)]
    target[5]["x_m"] = "1.0"
    target[-1]["mission"] = "FINISH"
    reference = [
        reference_row(0.0, "ROUTE"),
        reference_row(1.0, "HILL", "HILL"),
        reference_row(1.0, "TRAFFIC", "TRAFFIC"),
        reference_row(7.0, "FINISH", "FINISH"),
    ]

    mappings = map_boundaries_to_route(
        target,
        extract_mission_boundaries(reference),
        maximum_mapping_distance_m=5.0,
    )

    assert [mapping.target_index for mapping in mappings] == [0, 1, 5, 7]


def test_mapping_rejects_a_boundary_far_from_the_measured_route():
    target = [target_row(float(index)) for index in range(4)]
    target[-1]["mission"] = "FINISH"
    reference = [
        reference_row(0.0, "ROUTE"),
        {**reference_row(2.0, "HILL", "HILL"), "y_m": "3.0"},
        reference_row(3.0, "FINISH", "FINISH"),
    ]

    with pytest.raises(ValueError, match="is 3.000 m from the measured route"):
        map_boundaries_to_route(target, extract_mission_boundaries(reference))


def test_tag_route_can_update_a_csv_in_place(tmp_path):
    route_file = tmp_path / "route.csv"
    reference_file = tmp_path / "preview.csv"
    target = [target_row(float(index)) for index in range(5)]
    target[-1]["target_speed_mps"] = "0.0"
    target[-1]["mission"] = "FINISH"
    reference = [
        reference_row(0.0, "ROUTE"),
        reference_row(2.0, "TRAFFIC", "TRAFFIC"),
        reference_row(4.0, "FINISH", "FINISH"),
    ]
    with route_file.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(target)
    with reference_file.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=[*FIELDS, "fsm_zone"])
        writer.writeheader()
        writer.writerows(reference)

    tag_route(route_file, reference_file, route_file)

    with route_file.open(newline="", encoding="utf-8") as stream:
        tagged = list(csv.DictReader(stream))
    assert [row["mission"] for row in tagged] == [
        "NORMAL",
        "NORMAL",
        "TRAFFIC",
        "TRAFFIC",
        "FINISH",
    ]
    assert math.isclose(float(tagged[2]["target_speed_mps"]), 0.3)


def test_committed_course_07_route_has_all_runtime_mission_tags():
    route_file = (
        Path(__file__).parents[1] / "routes" / "course_07_vehicle.csv"
    )
    with route_file.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))

    transitions = []
    previous = ""
    for index, row in enumerate(rows):
        if row["mission"] != previous:
            transitions.append((index, row["s_m"], row["mission"]))
            previous = row["mission"]

    assert transitions == [
        (0, "0.0000", "NORMAL"),
        (64, "22.0475", "HILL"),
        (168, "58.0228", "NORMAL"),
        (371, "141.7181", "TRAFFIC"),
        (378, "144.0837", "NORMAL"),
        (499, "192.7606", "S_OBSTACLE"),
        (595, "230.5307", "NORMAL"),
        (670, "261.3700", "TRAFFIC"),
        (677, "264.5327", "NORMAL"),
        (730, "286.3709", "PERP_PARK"),
        (854, "333.5633", "NORMAL"),
        (1051, "411.7170", "TRAFFIC"),
        (1066, "417.6794", "NORMAL"),
        (1298, "512.0962", "DUMMY"),
        (1460, "576.8564", "NORMAL"),
        (1546, "611.9220", "PARALLEL_PARK"),
        (1625, "644.2341", "NORMAL"),
        (1672, "662.8085", "END_LANE"),
        (1741, "690.3703", "FINISH"),
    ]
