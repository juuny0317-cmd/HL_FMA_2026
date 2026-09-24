import math
from pathlib import Path

import pytest
import yaml

from hl_ku_core.local_detour import (
    CircleObstacle,
    DetourSettings,
    _maximum_curvature,
    _path_lateral_offset_at_s,
    _path_clear,
    plan_local_detour,
)
from hl_ku_core.route import Route, Waypoint


def straight_route() -> Route:
    return Route(
        Waypoint(
            index=index,
            s_m=float(index),
            x_m=float(index),
            y_m=0.0,
            target_speed_mps=0.3,
            mission="S_OBSTACLE" if 2 <= index <= 28 else "NORMAL",
            direction=1,
        )
        for index in range(31)
    )


def test_center_obstacle_produces_bounded_clear_candidate():
    route = straight_route()
    obstacle = CircleObstacle(15.0, 0.0, 0.25)
    settings = DetourSettings()
    result = plan_local_detour(route, 5.0, (obstacle,), settings)

    assert result.status == "CANDIDATE"
    assert result.start_s_m == 10.0
    assert result.obstacle_rear_s_m == 16.3
    assert result.end_s_m == 21.3
    assert result.points[0] == (10.0, 0.0)
    assert result.points[-1] == (21.3, 0.0)
    plateau = [abs(y) for x, y in result.points if 15.0 <= x <= 16.3]
    assert plateau
    assert min(plateau) == pytest.approx(max(plateau))
    assert _path_clear(
        result.points,
        (obstacle,),
        settings.vehicle_half_width_m + settings.safety_margin_m,
    )
    assert _maximum_curvature(result.points) <= settings.maximum_curvature_per_m
    assert max(abs(y) for _, y in result.points) <= (
        settings.corridor_half_width_m
        - settings.vehicle_half_width_m
        - settings.safety_margin_m
        + 1.0e-6
    )


def test_early_detection_leaves_room_before_detour_entry():
    config = yaml.safe_load(
        (Path(__file__).resolve().parents[1] / "config" / "system.yaml").read_text(
            encoding="utf-8"
        )
    )
    lidar = config["lidar_perception"]["ros__parameters"]
    detour = config["local_detour"]["ros__parameters"]
    settings = DetourSettings()
    assert lidar["detection_range_m"] > lidar["obstacle_trigger_m"]
    assert lidar["obstacle_trigger_m"] == 8.0
    assert settings.entry_length_m == 5.0
    assert settings.assumed_obstacle_length_m == 1.3
    assert detour["assumed_obstacle_length_m"] == 1.3

    route = straight_route()
    result = plan_local_detour(route, 7.0, (CircleObstacle(15.0, 0.0, 0.25),))
    assert result.status == "CANDIDATE"
    assert result.start_s_m == 10.0
    assert result.start_s_m - 7.0 == 3.0


def test_offset_obstacle_chooses_clear_side():
    result = plan_local_detour(
        straight_route(), 5.0, (CircleObstacle(15.0, 0.5, 0.25),)
    )
    assert result.status == "CANDIDATE"
    assert result.side == "RIGHT"


def test_no_candidate_when_both_sides_are_blocked():
    obstacles = (
        CircleObstacle(15.0, 0.0, 0.25),
        CircleObstacle(15.0, 1.1, 0.25),
        CircleObstacle(15.0, -1.1, 0.25),
    )
    result = plan_local_detour(straight_route(), 5.0, obstacles)
    assert result.status == "NO_SAFE_DETOUR"
    assert not result.points


def test_close_obstacle_and_wrong_mission_fail_closed():
    obstacle = CircleObstacle(15.0, 0.0, 0.25)
    at_nominal_entry = plan_local_detour(straight_route(), 10.0, (obstacle,))
    assert at_nominal_entry.status == "CANDIDATE"
    assert at_nominal_entry.start_s_m == 10.0
    assert plan_local_detour(straight_route(), 14.1, (obstacle,)).status == (
        "TOO_CLOSE_OR_ROUTE_END"
    )
    normal = Route(
        Waypoint(i, float(i), float(i), 0.0, 0.3, "NORMAL", 1)
        for i in range(31)
    )
    assert plan_local_detour(normal, 5.0, (obstacle,)).status == "OUTSIDE_S_OBSTACLE"


def test_late_detection_compresses_entry_down_to_one_meter_when_geometry_allows():
    route = straight_route()
    # This small offset obstacle still blocks the swept corridor, but a short
    # right-side correction is feasible from exactly 1 m before it.
    result = plan_local_detour(
        route, 14.0, (CircleObstacle(15.0, 0.55, 0.05),)
    )
    assert result.status == "CANDIDATE"
    assert result.start_s_m == 14.0
    assert result.side == "RIGHT"


def test_late_center_obstacle_keeps_physical_clearance_and_curvature_checks():
    result = plan_local_detour(
        straight_route(), 14.0, (CircleObstacle(15.0, 0.0, 0.25),)
    )
    assert result.status == "NO_SAFE_DETOUR"


def test_clear_or_invalid_input_never_creates_a_candidate():
    route = straight_route()
    assert plan_local_detour(route, 5.0, ()).status == "NO_OBSTACLE"
    assert plan_local_detour(route, 5.0, (CircleObstacle(15.0, 4.0, 0.25),)).status == (
        "ROUTE_CLEAR"
    )
    assert plan_local_detour(
        route, 5.0, (CircleObstacle(math.nan, 0.0, 0.25),)
    ).status == "INVALID_OBSTACLE"
    assert plan_local_detour(
        route,
        5.0,
        (CircleObstacle(15.0, 0.0, 0.25),),
        DetourSettings(assumed_obstacle_length_m=0.0),
    ).status == "INVALID_SETTINGS"


def test_replan_can_continue_from_current_return_offset():
    route = straight_route()
    first = plan_local_detour(route, 5.0, (CircleObstacle(15.0, 0.5, 0.25),))
    assert first.status == "CANDIDATE"
    current_s_m = 17.0
    current_offset_m = _path_lateral_offset_at_s(
        route, first, current_s_m, DetourSettings().sample_step_m
    )
    assert current_offset_m < 0.0

    second = plan_local_detour(
        route,
        current_s_m,
        (CircleObstacle(21.5, -0.5, 0.25),),
        initial_lateral_offset_m=current_offset_m,
    )
    assert second.status == "CANDIDATE"
    assert second.points[0][1] == pytest.approx(current_offset_m)
    assert second.obstacle_rear_s_m == pytest.approx(22.8)


def test_existing_course_07_s_zone_has_a_preview_candidate():
    route_file = (
        Path(__file__).resolve().parents[1] / "routes" / "course_07_vehicle.csv"
    )
    route = Route.load_csv(route_file)
    x, y = route.position_at_s(210.0)
    result = plan_local_detour(route, 200.0, (CircleObstacle(x, y, 0.25),))
    assert result.status == "CANDIDATE"
    assert result.start_s_m is not None and result.start_s_m > 200.0

    early_detection = plan_local_detour(
        route, 202.0, (CircleObstacle(x, y, 0.25),)
    )
    assert early_detection.status == "CANDIDATE"
    assert early_detection.start_s_m is not None
    assert math.isclose(early_detection.start_s_m, 205.0, abs_tol=0.01)
