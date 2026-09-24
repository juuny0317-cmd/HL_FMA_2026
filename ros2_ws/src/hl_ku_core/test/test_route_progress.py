import math
from pathlib import Path

import pytest

from hl_ku_core.route import Route, Waypoint
from hl_ku_core.route_progress import (
    continuous_nearest_index,
    direction_runs,
    match_route_ahead,
    match_saved_progress,
)


def crossing_route() -> Route:
    # The route passes almost through the same XY twice, 30 m apart in s, with
    # perpendicular headings.  Heading must select the intended occurrence.
    return Route(
        [
            Waypoint(0, 0.0, 0.0, 0.0, 0.3, "NORMAL", 1),
            Waypoint(1, 10.0, 10.0, 0.0, 0.3, "NORMAL", 1),
            Waypoint(2, 20.0, 10.0, -10.0, 0.3, "NORMAL", 1),
            Waypoint(3, 30.0, 5.0, -10.0, 0.3, "NORMAL", 1),
            Waypoint(4, 40.0, 5.0, 10.0, 0.3, "NORMAL", 1),
        ]
    )


def test_restart_match_uses_heading_to_disambiguate_a_near_crossing():
    route = crossing_route()
    east = match_route_ahead(route, 5.02, 0.01, 0.0)
    north = match_route_ahead(route, 5.02, 0.01, math.pi / 2.0)

    assert east is not None and east.route_s_m < 10.0
    assert north is not None and north.route_s_m > 30.0


def test_restart_match_never_targets_a_waypoint_behind_the_vehicle():
    route = Route(
        [
            Waypoint(0, 0.0, 0.0, 0.0, 0.3, "NORMAL", 1),
            Waypoint(1, 1.0, 1.0, 0.0, 0.3, "NORMAL", 1),
            Waypoint(2, 2.0, 2.0, 0.0, 0.3, "NORMAL", 1),
        ]
    )
    match = match_route_ahead(route, -0.5, 0.0, 0.0, forward_offset_m=0.30)

    assert match is not None
    assert match.waypoint_index == 1
    assert match.route_s_m == pytest.approx(0.30)


def test_continuous_tracking_cannot_jump_to_a_distant_overlapping_leg():
    route = crossing_route()
    # Even though this XY is also on the later northbound leg, a tracker whose
    # current progress is on the eastbound leg may only advance locally.
    selected = continuous_nearest_index(
        route,
        5.0,
        0.01,
        current_index=0,
        run_start_index=0,
        run_end_index=4,
        forward_search_m=12.0,
    )
    assert selected == 0


def test_course_07_crossing_restarts_and_tracks_without_116m_jump():
    path = (
        Path(__file__).resolve().parents[1]
        / "routes/mando/competition/course_07_full.csv"
    )
    route = Route.load_csv(path)
    first_x, first_y = route.position_at_s(153.6)
    first_heading = route.heading_at_s(153.6, 1.0)
    match = match_route_ahead(route, first_x, first_y, first_heading)

    assert match is not None
    assert match.route_s_m == pytest.approx(153.9, abs=0.5)
    current_index = min(
        range(len(route.waypoints)),
        key=lambda index: abs(route.waypoints[index].s_m - 153.6),
    )
    selected = continuous_nearest_index(
        route,
        first_x + 0.04,
        first_y,
        current_index,
        0,
        3047,
        3.0,
    )
    assert route.waypoints[selected].s_m < 157.0


@pytest.mark.parametrize("saved_s", [306.0, 635.0])
def test_checkpoint_preserves_reverse_parking_run_on_overlapping_path(saved_s):
    path = (
        Path(__file__).resolve().parents[1]
        / "routes/mando/competition/course_07_full.csv"
    )
    route = Route.load_csv(path)
    runs = direction_runs(route)
    saved_index = min(
        range(len(route.waypoints)),
        key=lambda index: abs(route.waypoints[index].s_m - saved_s),
    )
    run_index = next(
        index
        for index, (start, end, _direction) in enumerate(runs)
        if start <= saved_index <= end
    )
    assert runs[run_index][2] == -1
    x_m, y_m = route.position_at_s(saved_s)
    vehicle_yaw = normalize_vehicle_heading(route, saved_s, -1)

    match = match_saved_progress(
        route,
        x_m,
        y_m,
        vehicle_yaw,
        saved_route_s_m=saved_s,
        saved_run_index=run_index,
    )

    assert match is not None
    assert match.run_index == run_index
    assert match.direction == -1
    assert abs(match.route_s_m - saved_s) < 0.5


def test_checkpoint_is_ignored_after_vehicle_is_moved_elsewhere():
    route = crossing_route()
    match = match_saved_progress(
        route,
        50.0,
        50.0,
        0.0,
        saved_route_s_m=2.0,
        saved_run_index=0,
        position_tolerance_m=2.0,
    )
    assert match is None


def normalize_vehicle_heading(route: Route, route_s_m: float, direction: int) -> float:
    heading = route.heading_at_s(route_s_m, 0.5)
    return heading if direction > 0 else math.atan2(-math.sin(heading), -math.cos(heading))
