import math

from hl_ku_core.mando_route_slice import slice_and_place
from hl_ku_core.route import Route, Waypoint
from hl_ku_core.route_prepare import save_route


def test_slice_and_place_is_a_rigid_transform_without_scaling():
    source = Route(
        [
            Waypoint(0, 0.0, 10.0, 10.0, 0.3, "NORMAL", 1),
            Waypoint(1, 3.0, 13.0, 10.0, 0.3, "NORMAL", 1),
            Waypoint(2, 7.0, 13.0, 14.0, 0.3, "NORMAL", 1),
            Waypoint(3, 10.0, 16.0, 14.0, 0.0, "FINISH", 1),
        ]
    )

    placed = slice_and_place(source, 0.0, 7.0, 100.0, 200.0, 90.0, 0.25)

    assert len(placed.waypoints) == 3
    assert placed.waypoints[0].x_m == 100.0
    assert placed.waypoints[0].y_m == 200.0
    assert math.isclose(placed.waypoints[1].x_m, 100.0, abs_tol=1.0e-9)
    assert math.isclose(placed.waypoints[1].y_m, 203.0, abs_tol=1.0e-9)
    before = math.hypot(13.0 - 10.0, 14.0 - 10.0)
    after = math.hypot(
        placed.waypoints[2].x_m - placed.waypoints[0].x_m,
        placed.waypoints[2].y_m - placed.waypoints[0].y_m,
    )
    assert math.isclose(before, after)
    assert placed.waypoints[-1].mission == "FINISH"
    assert placed.waypoints[-1].target_speed_mps == 0.0


def test_slice_and_place_preserves_parking_profile_and_reverse_start_heading():
    source = Route(
        [
            Waypoint(0, 10.0, 0.0, 0.0, 0.0, "PERP_PARK", -1),
            Waypoint(1, 12.0, 2.0, 0.0, 0.0, "PERP_PARK", -1),
            Waypoint(2, 14.0, 4.0, 0.0, 0.0, "PERP_PARK", 1),
            Waypoint(3, 16.0, 6.0, 0.0, 0.0, "PERP_PARK", 1),
        ]
    )

    placed = slice_and_place(
        source,
        10.0,
        6.0,
        100.0,
        200.0,
        0.0,
        0.15,
        preserve_source_profile=True,
    )

    assert [point.direction for point in placed.waypoints] == [-1, -1, 1, 1]
    assert placed.waypoints[0].mission == "PERP_PARK"
    assert placed.waypoints[1].x_m < placed.waypoints[0].x_m
    assert math.isclose(placed.waypoints[1].y_m, 200.0, abs_tol=1.0e-9)
    assert placed.waypoints[-1].mission == "FINISH"
    assert placed.waypoints[-1].target_speed_mps == 0.0


def test_slice_carries_fsm_events_and_keeps_a_segment_finish_tag(tmp_path):
    source = Route(
        [
            Waypoint(
                0, 0.0, 0.0, 0.0, 0.0, "S_OBSTACLE", 1,
                "S_CURVE", "S_START", "ENTER_ZONE", "", 0.0,
            ),
            Waypoint(
                1, 1.0, 1.0, 0.0, 0.0, "S_OBSTACLE", 1,
                "S_CURVE", "", "", "", 0.0,
            ),
            Waypoint(
                2, 2.0, 2.0, 0.0, 0.0, "S_OBSTACLE", 1,
                "S_CURVE", "S_END", "EXIT_ZONE", "", 0.0,
            ),
        ]
    )

    placed = slice_and_place(
        source, 0.0, 2.0, 10.0, 20.0, 0.0, 0.25,
        preserve_source_profile=True,
    )
    path = tmp_path / "placed.csv"
    save_route(placed, path)
    loaded = Route.load_csv(path)

    assert loaded.waypoints[0].mission == "S_OBSTACLE"
    assert loaded.active_event_waypoint(1).fsm_event_ids == "S_START"
    assert loaded.waypoints[-1].mission == "FINISH"
    assert loaded.waypoints[-1].fsm_event_ids == "TEST_SEGMENT_END"
