import math

from hl_ku_core.route import Route, Waypoint
from hl_ku_core.route_prepare import antenna_path_to_base_link, densify_route


def test_densify_route_preserves_polyline_and_finish():
    route = Route(
        [
            Waypoint(0, 0.0, 0.0, 0.0, 0.3, "NORMAL", 1),
            Waypoint(1, 2.0, 2.0, 0.0, 0.3, "NORMAL", 1),
            Waypoint(2, 4.0, 2.0, 2.0, 0.0, "FINISH", 1),
        ]
    )

    prepared = densify_route(route, 0.5)

    assert len(prepared.waypoints) == 9
    assert prepared.waypoints[0].x_m == 0.0
    assert prepared.waypoints[-1].x_m == 2.0
    assert prepared.waypoints[-1].y_m == 2.0
    assert prepared.waypoints[-1].mission == "FINISH"
    assert prepared.waypoints[-1].target_speed_mps == 0.0
    assert all(point.mission == "NORMAL" for point in prepared.waypoints[:-1])
    assert math.isclose(prepared.waypoints[-1].s_m, 4.0)
    assert all(
        math.hypot(right.x_m - left.x_m, right.y_m - left.y_m) <= 0.5 + 1e-9
        for left, right in zip(prepared.waypoints, prepared.waypoints[1:])
    )


def test_densify_route_rejects_invalid_spacing():
    route = Route(
        [
            Waypoint(0, 0.0, 0.0, 0.0, 0.3, "NORMAL", 1),
            Waypoint(1, 1.0, 1.0, 0.0, 0.0, "FINISH", 1),
        ]
    )

    for spacing in (0.0, -1.0, math.nan):
        try:
            densify_route(route, spacing)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid spacing must be rejected")


def test_antenna_path_is_shifted_back_to_rear_axle():
    route = Route(
        [
            Waypoint(0, 0.0, 0.0, 0.0, 0.3, "NORMAL", 1),
            Waypoint(1, 1.0, 1.0, 0.0, 0.3, "NORMAL", 1),
            Waypoint(2, 2.0, 2.0, 0.0, 0.0, "FINISH", 1),
        ]
    )

    shifted = antenna_path_to_base_link(route, 0.802, 0.0)

    assert all(math.isclose(point.y_m, 0.0) for point in shifted.waypoints)
    assert math.isclose(shifted.waypoints[0].x_m, -0.802)
    assert math.isclose(shifted.waypoints[-1].x_m, 1.198)
    assert math.isclose(shifted.waypoints[-1].s_m, 2.0)
    assert shifted.waypoints[-1].mission == "FINISH"
