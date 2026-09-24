from hl_ku_core.route import Route, Waypoint
from hl_ku_foxglove.replay_visualizer_node import (
    course_07_section_at_s,
    load_variant_mission_metadata,
    mission_segment_at_index,
    signed_cross_track_error,
    source_progress_labels,
)


def test_course_07_section_lookup_tracks_original_fsm_boundaries() -> None:
    assert course_07_section_at_s(0.0)["zone"] == "ROUTE"
    assert course_07_section_at_s(30.0)["zone"] == "HILL"
    assert course_07_section_at_s(100.0)["zone"] == "BEND"
    assert course_07_section_at_s(200.0)["zone"] == "S_CURVE"
    assert course_07_section_at_s(300.0)["zone"] == "T_PARK"
    assert course_07_section_at_s(540.0)["zone"] == "DUMMY"
    assert course_07_section_at_s(620.0)["zone"] == "PARALLEL_PARK"
    assert course_07_section_at_s(670.0)["zone"] == "END_LANE"
    assert course_07_section_at_s(690.37)["zone"] == "FINISH"


def test_full_course_all_label_uses_current_authored_fsm() -> None:
    section, zone, label = source_progress_labels(0.0, "ALL", "전체코스")
    assert section["zone"] == "ROUTE"
    assert zone == "ROUTE"
    assert label == "출발 일반구간"

    section, zone, label = source_progress_labels(300.0, "ALL", "전체코스")
    assert section["zone"] == "T_PARK"
    assert zone == "T_PARK"
    assert label == "T자 주차"


def test_isolated_scenario_keeps_concrete_fsm_override() -> None:
    _section, zone, label = source_progress_labels(
        100.0, "T_PARK", "T자 주차 단독"
    )
    assert zone == "T_PARK"
    assert label == "T자 주차 단독"


def test_signed_cross_track_error_is_left_positive() -> None:
    route = Route(
        [
            Waypoint(0, 0.0, 0.0, 0.0, 0.3, "NORMAL", 1),
            Waypoint(1, 10.0, 10.0, 0.0, 0.3, "NORMAL", 1),
        ]
    )
    assert signed_cross_track_error(route, 0, 3.0, 2.0) == 2.0
    assert signed_cross_track_error(route, 0, 3.0, -2.0) == -2.0


def test_variant_metadata_preserves_waypoint_fsm_boundaries(tmp_path) -> None:
    route_file = tmp_path / "variant.csv"
    route_file.write_text(
        "s_m,x_m,y_m,target_speed_mps,mission,direction,fsm_zone,fsm_event_ids\n"
        "0.0,0,0,0,NORMAL,1,ROUTE,ROUTE_START\n"
        "0.1,0.1,0,0,NORMAL,1,ROUTE,\n"
        "0.2,0.2,0,0,PERP_PARK,1,T_PARK,T_PARK_START\n"
        "0.3,0.3,0,0,PERP_PARK,-1,T_PARK,T1_FORWARD_TO_REVERSE\n",
        encoding="utf-8",
    )

    metadata = load_variant_mission_metadata(route_file, 1, "test")

    assert metadata["point_count"] == 4
    assert metadata["segment_count"] == 3
    assert metadata["events"][0] == {
        "event_id": "ROUTE_START",
        "index": 0,
        "s_m": 0.0,
        "x_m": 0.0,
        "y_m": 0.0,
    }
    assert metadata["events"][-1]["event_id"] == "T1_FORWARD_TO_REVERSE"
    assert metadata["segments"][0] == {
        "zone": "ROUTE",
        "mission": "NORMAL",
        "direction": 1,
        "start_index": 0,
        "end_index": 1,
        "start_s_m": 0.0,
        "end_s_m": 0.1,
        "event_ids": ["ROUTE_START"],
    }
    assert metadata["segments"][2]["direction"] == -1
    assert metadata["segments"][2]["event_ids"] == ["T1_FORWARD_TO_REVERSE"]
    assert mission_segment_at_index(metadata, 0)["zone"] == "ROUTE"
    assert mission_segment_at_index(metadata, 2)["zone"] == "T_PARK"
    assert mission_segment_at_index(metadata, 3)["direction"] == -1
    assert mission_segment_at_index(metadata, 4) is None
