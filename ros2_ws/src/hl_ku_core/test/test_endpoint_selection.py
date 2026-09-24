import math
from pathlib import Path

from hl_ku_core.endpoint_selection import (
    EndpointDetection,
    EndpointRouteLatch,
    ROUTE_1,
    ROUTE_2,
    UNKNOWN,
    blended_final_route,
    endpoint_route_candidate,
    event_s,
)
from hl_ku_core.route import Route


ROOT = Path(__file__).resolve().parents[4]
MISSION_ROUTE = ROOT / "ros2_ws/src/hl_ku_core/routes/course_07_mission.csv"


def box(name, x, y=100.0, confidence=0.9):
    return EndpointDetection(name, confidence, x - 10, y - 10, x + 10, y + 10)


def test_endpoint_requires_both_classes_in_one_frame():
    assert endpoint_route_candidate([box("green_arrow", 100)], 480).route == UNKNOWN


def test_red_left_of_green_selects_route_2():
    result = endpoint_route_candidate(
        [box("red_signal", 100), box("green_arrow", 200)], 480
    )
    assert result.route == ROUTE_2


def test_green_left_of_red_keeps_route_1():
    result = endpoint_route_candidate(
        [box("green_arrow", 100), box("red_signal", 200)], 480
    )
    assert result.route == ROUTE_1


def test_route_2_latch_never_returns_to_route_1():
    latch = EndpointRouteLatch()
    assert latch.selected_final == 1
    assert latch.update(
        endpoint_route_candidate(
            [box("red_signal", 100), box("green_arrow", 200)], 480
        )
    )
    assert latch.selected_final == 2
    assert latch.locked
    assert not latch.update(
        endpoint_route_candidate(
            [box("green_arrow", 100), box("red_signal", 200)], 480
        )
    )
    assert latch.selected_final == 2


def test_blended_f2_starts_on_f1_and_finishes_on_f2():
    f1 = Route.load_csv(MISSION_ROUTE, "p1_t1_f1")
    f2 = Route.load_csv(MISSION_ROUTE, "p1_t1_f2")
    blended = blended_final_route(f1, f2, 8.0)
    f1_start = event_s(f1, "END_LANE_DECISION_START")
    f2_start = event_s(f2, "END_LANE_DECISION_START")

    at_start = blended.position_at_s(f2_start)
    assert math.dist(at_start, f1.position_at_s(f1_start)) < 0.02

    after_merge = f2_start + 8.5
    assert math.dist(
        blended.position_at_s(after_merge), f2.position_at_s(after_merge)
    ) < 0.02


def test_late_f2_decision_starts_blend_at_current_f1_progress():
    f1 = Route.load_csv(MISSION_ROUTE, "p1_t1_f1")
    f2 = Route.load_csv(MISSION_ROUTE, "p1_t1_f2")
    f1_start = event_s(f1, "END_LANE_DECISION_START")
    f2_start = event_s(f2, "END_LANE_DECISION_START")
    decision_offset = 6.0
    blended = blended_final_route(
        f1,
        f2,
        8.0,
        reference_transition_s_m=f1_start + decision_offset,
    )

    at_decision = blended.position_at_s(f2_start + decision_offset)
    assert math.dist(
        at_decision, f1.position_at_s(f1_start + decision_offset)
    ) < 0.02

    after_merge = f2_start + decision_offset + 8.5
    assert math.dist(
        blended.position_at_s(after_merge), f2.position_at_s(after_merge)
    ) < 0.02


def test_s_obstacle_is_not_a_yolo_runtime_mode():
    from hl_ku_core.yolo_gate import allowed_classes, mode_for_route_zone

    assert mode_for_route_zone("S_OBSTACLE") == "OFF"
    assert mode_for_route_zone("DUMMY") == "PEDESTRIAN"
    assert mode_for_route_zone("NORMAL") == "OFF"
    assert allowed_classes("PEDESTRIAN") == frozenset({"pedestrian"})
    assert "s_obstacle" not in allowed_classes("OFF")
    assert "end_point" not in allowed_classes("TRAFFIC")
