from hl_ku_core.path_tracker_node import (
    bounded_route_heading,
    direction_runs,
    effective_target_speed,
    selected_path_steering,
)
from hl_ku_core.route import Route, Waypoint


def test_speed_override_replaces_csv_cruise_speed():
    assert effective_target_speed(0.30, 1, 0.40) == 0.40
    assert effective_target_speed(0.30, -1, 0.40) == -0.40


def test_speed_override_preserves_zero_speed_finish():
    assert effective_target_speed(0.0, 1, 0.40) == 0.0


def test_zero_override_uses_csv_speed():
    assert effective_target_speed(0.30, 1, 0.0) == 0.30


def test_forward_steering_is_stanley_only_even_when_pursuit_differs():
    steering, pursuit_weight, mode = selected_path_steering(1, -0.12, 0.31, 0.48)

    assert steering == -0.12
    assert pursuit_weight == 0.0
    assert mode == "STANLEY_100"


def test_reverse_steering_is_also_stanley_only():
    steering, pursuit_weight, mode = selected_path_steering(-1, -0.12, 0.31, 0.48)

    assert steering == -0.12
    assert pursuit_weight == 0.0
    assert mode == "STANLEY_REVERSE_100"


def test_direction_runs_and_heading_stop_at_parking_cusps():
    route = Route(
        [
            Waypoint(0, 0.0, 0.0, 0.0, 0.2, "PERP_PARK", 1),
            Waypoint(1, 1.0, 1.0, 0.0, 0.2, "PERP_PARK", 1),
            Waypoint(2, 2.0, 1.0, -1.0, 0.2, "PERP_PARK", -1),
            Waypoint(3, 3.0, 1.0, -2.0, 0.2, "PERP_PARK", -1),
            Waypoint(4, 4.0, 2.0, -2.0, 0.0, "FINISH", 1),
        ]
    )

    assert direction_runs(route) == ((0, 1, 1), (2, 3, -1), (4, 4, 1))
    assert bounded_route_heading(route, 0.9, 0, 1, 1.0) == 0.0
    assert route.nearest_projection_between(2.1, -2.1, 4, 4) == (
        4,
        4.0,
        2.0,
        -2.0,
    )


def test_direction_run_endpoint_is_an_unambiguous_transition_trigger():
    route = Route(
        [
            Waypoint(0, 0.0, 0.0, 0.0, 0.2, "PERP_PARK", 1),
            Waypoint(1, 1.0, 1.0, 0.0, 0.2, "PERP_PARK", 1),
            Waypoint(2, 2.0, 0.9, 0.0, 0.2, "PERP_PARK", -1),
            Waypoint(3, 3.0, 0.0, 0.0, 0.2, "PERP_PARK", -1),
        ]
    )
    runs = direction_runs(route)

    # A pose that has just passed the forward cusp still projects to the final
    # waypoint of that run, even when its Euclidean distance exceeds the old
    # transition radius.
    nearest = route.nearest_index_between(1.8, 0.0, runs[0][0], runs[0][1])
    assert nearest == runs[0][1]
