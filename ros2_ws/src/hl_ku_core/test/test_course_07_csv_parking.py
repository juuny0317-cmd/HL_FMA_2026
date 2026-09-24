"""Integration checks for the CSV parking splice used by Course 07 driving."""

import math
from pathlib import Path

from hl_ku_core.mission import MissionCoordinator, Observation, State
from hl_ku_core.parking_selection import ParkingRouteFamily
from hl_ku_core.route import Route


ROOT = Path(__file__).resolve().parents[4]
ROUTES = ROOT / "ros2_ws/src/hl_ku_core/routes"
BUNDLE = ROUTES / "course_07_mission.csv"
ACTIVE = ROUTES / "mando/competition/course_07_full.csv"
BASELINES = ROUTES / "course_07_parking_overlay"


def event_point(route: Route, name: str):
    return next(
        point for point in route.waypoints
        if name in point.fsm_event_ids.split("|")
    )


def test_all_variants_use_new_parking_phases_and_events():
    for p in (1, 2):
        for t in (1, 2):
            for f in (1, 2):
                route = Route.load_csv(BUNDLE, f"p{p}_t{t}_f{f}")
                parking = [point for point in route.waypoints if point.mission == "PARALLEL_PARK"]
                assert parking
                assert parking[0].direction_source == "PARALLEL_CSV_COMMON_FORWARD"
                assert math.dist((parking[0].x_m, parking[0].y_m), (-12.8436, -42.7671)) < 0.01
                assert parking[0].fsm_event_ids == "PARALLEL_PARK_START"
                reverse = event_point(route, f"PARALLEL_{p}_FORWARD_TO_REVERSE")
                complete = event_point(route, f"PARALLEL_{p}_PARK_COMPLETE")
                exit_point = event_point(route, f"PARALLEL_{p}_END")
                assert reverse.direction == -1
                assert reverse.direction_source == f"PARALLEL_CSV_P{p}_REVERSE"
                assert complete.direction == 1
                assert complete.direction_source == f"PARALLEL_CSV_P{p}_EXIT"
                assert exit_point.direction == 1
                assert parking[0].s_m < reverse.s_m < complete.s_m < exit_point.s_m
                assert route.waypoints[exit_point.index + 1].mission == "NORMAL"


def test_parking_selector_and_fsm_use_replaced_branches():
    p1 = Route.load_csv(BUNDLE, "p1_t1_f1")
    p2 = Route.load_csv(BUNDLE, "p2_t1_f1")
    family = ParkingRouteFamily.from_placed_route(p1, BUNDLE, "p1_t1_f1", 0.0, 716.0, 0.3)
    assert len(family.routes) == 8
    assert family.decision_s_m("PARALLEL_PARK") < event_point(p1, "PARALLEL_1_FORWARD_TO_REVERSE").s_m
    assert math.dist(
        (event_point(p1, "PARALLEL_1_PARK_COMPLETE").x_m,
         event_point(p1, "PARALLEL_1_PARK_COMPLETE").y_m),
        (event_point(p2, "PARALLEL_2_PARK_COMPLETE").x_m,
         event_point(p2, "PARALLEL_2_PARK_COMPLETE").y_m),
    ) > 5.0

    for route in (p1, p2):
        coordinator = MissionCoordinator()
        coordinator.arm()
        start = event_point(route, "PARALLEL_PARK_START")
        decision = coordinator.update(Observation(
            now_sec=1.0, zone=start.mission, route_s_m=start.s_m,
            speed_mps=0.0, route_event_id=start.fsm_event_ids,
            route_has_fsm_metadata=True,
        ))
        assert decision.state == State.PARALLEL_PARK
        assert not decision.brake
        last = next(point for point in route.waypoints if "PARALLEL_" in point.fsm_event_ids and "_END" in point.fsm_event_ids)
        after = route.waypoints[last.index + 1]
        decision = coordinator.update(Observation(
            now_sec=2.0, zone=after.mission, route_s_m=after.s_m,
            speed_mps=0.0, route_has_fsm_metadata=True,
        ))
        assert decision.state == State.ROUTE


def test_active_full_route_matches_p1_mission_bundle():
    source = Route.load_csv(BUNDLE, "p1_t1_f1")
    active = Route.load_csv(ACTIVE)
    assert len(active.waypoints) == len(source.waypoints)
    for actual, expected in zip(active.waypoints, source.waypoints):
        assert math.dist((actual.x_m, actual.y_m), (expected.x_m, expected.y_m)) < 0.002
        assert actual.direction == expected.direction
        assert actual.mission == expected.mission
        assert actual.fsm_event_ids == expected.fsm_event_ids


def test_hill_stop_moves_exactly_1_5_m_upstream_in_every_variant():
    for p in (1, 2):
        for t in (1, 2):
            for f in (1, 2):
                variant = f"p{p}_t{t}_f{f}"
                baseline = Route.load_csv(
                    BASELINES / f"course_07_{variant}_fair_fsm_preview.csv"
                )
                updated = Route.load_csv(BUNDLE, variant)
                before = event_point(baseline, "HILL_STOP")
                after = event_point(updated, "HILL_STOP")
                assert round(before.s_m - after.s_m, 4) == 1.5
                assert after.mission == "HILL"
                assert after.fsm_actions == "STOP_THEN_HOLD"
                assert after.fsm_conditions == "ALWAYS_ONCE"
                assert after.fsm_hold_sec == 3.0
                assert after.direction_source == "HILL_STOP_SHIFTED_1_5M"
                assert len([
                    point for point in updated.waypoints
                    if "HILL_STOP" in point.fsm_event_ids.split("|")
                ]) == 1
                old_location = min(
                    updated.waypoints,
                    key=lambda point: abs(point.s_m - before.s_m),
                )
                assert old_location.fsm_event_ids != "HILL_STOP"


def test_active_route_fsm_stops_at_shifted_hill_point():
    route = Route.load_csv(ACTIVE)
    stop = event_point(route, "HILL_STOP")
    assert stop.s_m == 32.6405
    coordinator = MissionCoordinator()
    coordinator.arm()
    approaching = route.waypoints[stop.index - 1]
    decision = coordinator.update(Observation(
        now_sec=1.0, zone=approaching.mission,
        route_s_m=approaching.s_m, speed_mps=0.0,
        route_event_id=route.active_event_waypoint(approaching.index).fsm_event_ids,
        route_has_fsm_metadata=True,
    ))
    assert decision.state == State.HILL_APPROACH
    decision = coordinator.update(Observation(
        now_sec=2.0, zone=stop.mission, route_s_m=stop.s_m,
        speed_mps=0.0, route_event_id=stop.fsm_event_ids,
        route_event_action=stop.fsm_actions,
        route_event_condition=stop.fsm_conditions,
        route_event_hold_sec=stop.fsm_hold_sec,
        route_has_fsm_metadata=True,
    ))
    assert decision.state == State.HILL_HOLD
    assert decision.brake
