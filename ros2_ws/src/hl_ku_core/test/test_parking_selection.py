import math
from dataclasses import replace
from pathlib import Path

from hl_ku_core.parking_selection import (
    BayState,
    ObstacleDisc,
    ParkingBranchSelector,
    ParkingRouteFamily,
    ParkingSelectionSettings,
    VehiclePose2D,
    choose_option,
)
from hl_ku_core.route import Route


ROOT = Path(__file__).resolve().parents[4]
MISSION_ROUTE = ROOT / "ros2_ws/src/hl_ku_core/routes/course_07_mission.csv"


def route_family() -> ParkingRouteFamily:
    active = Route.load_csv(MISSION_ROUTE, "p1_t1_f1")
    return ParkingRouteFamily.from_placed_route(
        active,
        MISSION_ROUTE,
        "p1_t1_f1",
        0.0,
        700.0,
        0.3,
        ParkingSelectionSettings(
            minimum_visible_fraction=0.0,
            minimum_visible_points=1,
            clear_scans_required=1,
            blocked_scans_required=1,
            lidar_minimum_range_m=0.0,
            lidar_maximum_range_m=100.0,
            lidar_front_half_angle_rad=3.141592653589793,
        ),
    )


def test_user_fallback_table_always_selects_a_route():
    assert choose_option(BayState.CLEAR, BayState.BLOCKED) == 1
    assert choose_option(BayState.BLOCKED, BayState.CLEAR) == 2
    assert choose_option(BayState.CLEAR, BayState.UNKNOWN) == 1
    assert choose_option(BayState.BLOCKED, BayState.UNKNOWN) == 2
    assert choose_option(BayState.UNKNOWN, BayState.CLEAR) == 2
    assert choose_option(BayState.UNKNOWN, BayState.BLOCKED) == 1
    assert choose_option(BayState.UNKNOWN, BayState.UNKNOWN) == 1
    assert choose_option(BayState.BLOCKED, BayState.BLOCKED) == 1
    assert choose_option(BayState.CLEAR, BayState.CLEAR) == 1


def test_t_selection_waits_until_decision_point_when_both_bays_are_resolved():
    family = route_family()
    selector = ParkingBranchSelector(family)
    t1_point = family.bay("PERP_PARK", 1).path[len(family.bay("PERP_PARK", 1).path) // 2]
    obstacle = (ObstacleDisc(t1_point[0], t1_point[1], 0.2),)
    pose = VehiclePose2D(0.0, 0.0, 0.0)

    selector.set_active_mission("NORMAL")
    selector.observe(pose, obstacle)
    assert selector.state(1) == BayState.UNKNOWN
    assert selector.state(2) == BayState.UNKNOWN

    selector.set_active_mission("PERP_PARK")
    selector.observe(pose, obstacle)
    assert selector.state(1) == BayState.BLOCKED
    assert selector.state(2) == BayState.CLEAR
    assert not selector.maybe_lock(0.0)
    assert selector.maybe_lock(family.decision_s_m("PERP_PARK"))
    assert selector.perpendicular_option == 2
    assert selector.selected_variant_id() == "p1_t2_f1"


def test_route_family_contains_all_parking_and_final_lane_variants():
    family = route_family()
    assert set(family.routes) == {
        f"p{p}_t{t}_f{f}"
        for p in (1, 2)
        for t in (1, 2)
        for f in (1, 2)
    }


def test_t_selection_uses_decision_point_as_unknown_fallback_deadline():
    family = route_family()
    selector = ParkingBranchSelector(family)
    selector.set_active_mission("PERP_PARK")
    decision_s = family.decision_s_m("PERP_PARK")

    assert not selector.maybe_lock(
        decision_s - family.settings.decision_lead_m - 0.01
    )
    assert selector.maybe_lock(decision_s)
    assert selector.perpendicular_option == 1


def test_parking_candidates_use_one_shared_rigid_placement():
    family = route_family()
    t1 = family.route(1, 1)
    t2 = family.route(1, 2)
    decision_s = family.decision_s_m("PERP_PARK")

    t1_point = min(t1.waypoints, key=lambda point: abs(point.s_m - decision_s))
    t2_point = min(t2.waypoints, key=lambda point: abs(point.s_m - decision_s))

    # The measured variants share the forward approach.  Their placement must
    # preserve that overlap instead of aligning each candidate's noisy first
    # tangent independently and rotating them apart.
    assert math.hypot(
        t1_point.x_m - t2_point.x_m,
        t1_point.y_m - t2_point.y_m,
    ) < 0.25


def test_blocked_state_requires_repeated_consecutive_hits():
    family = route_family()
    family = ParkingRouteFamily(
        family.routes,
        family.initial_variant_id,
        ParkingSelectionSettings(
            minimum_visible_fraction=0.0,
            minimum_visible_points=1,
            clear_scans_required=1,
            blocked_scans_required=3,
            lidar_minimum_range_m=0.0,
            lidar_maximum_range_m=100.0,
            lidar_front_half_angle_rad=math.pi,
        ),
    )
    selector = ParkingBranchSelector(family)
    selector.set_active_mission("PERP_PARK")
    point = family.bay("PERP_PARK", 1).path[-1]
    obstacle = (ObstacleDisc(point[0], point[1], 0.2),)
    pose = VehiclePose2D(0.0, 0.0, 0.0)

    selector.observe(pose, obstacle)
    selector.observe(pose, obstacle)
    assert selector.state(1) == BayState.UNKNOWN

    # A clear frame breaks the consecutive-hit streak.
    selector.observe(pose, ())
    selector.observe(pose, obstacle)
    selector.observe(pose, obstacle)
    assert selector.state(1) == BayState.UNKNOWN
    selector.observe(pose, obstacle)
    assert selector.state(1) == BayState.BLOCKED


def test_parking_range_limit_ignores_distant_obstacles_only_for_selection():
    base = route_family()
    family = ParkingRouteFamily(
        base.routes,
        base.initial_variant_id,
        replace(
            base.settings,
            lidar_maximum_range_m=5.0,
            lidar_front_half_angle_rad=math.pi,
        ),
    )
    point = family.bay("PERP_PARK", 1).path[-1]
    obstacle = (ObstacleDisc(point[0], point[1], 0.2),)

    distant = ParkingBranchSelector(family)
    distant.set_active_mission("PERP_PARK")
    distant.observe(VehiclePose2D(point[0] - 7.0, point[1], 0.0), obstacle)
    assert distant.state(1) != BayState.BLOCKED

    nearby = ParkingBranchSelector(family)
    nearby.set_active_mission("PERP_PARK")
    nearby.observe(VehiclePose2D(point[0] - 5.0, point[1], 0.0), obstacle)
    assert nearby.state(1) == BayState.BLOCKED


def test_parallel_selection_uses_p2_decision_point_and_preserves_t_choice():
    family = route_family()
    selector = ParkingBranchSelector(family)
    selector.perpendicular_option = 2
    selector.set_active_mission("PARALLEL_PARK")
    p1 = family.bay("PARALLEL_PARK", 1, perpendicular=2)
    point = p1.path[len(p1.path) // 2]
    selector.observe(
        VehiclePose2D(0.0, 0.0, 0.0),
        (ObstacleDisc(point[0], point[1], 0.2),),
    )
    assert selector.state(1) == BayState.BLOCKED
    assert selector.state(2) == BayState.CLEAR
    p2_decision_s = family.decision_s_m("PARALLEL_PARK", perpendicular=2)
    assert selector.maybe_lock(p2_decision_s)
    assert selector.parallel_option == 2
    assert selector.perpendicular_option == 2
    assert selector.selected_variant_id() == "p2_t2_f1"


def test_restart_restores_and_locks_the_active_parking_branch():
    family = route_family()
    selector = ParkingBranchSelector(family)

    selector.restore_variant("p2_t2_f1", "PERP_PARK")

    assert selector.parallel_option == 2
    assert selector.perpendicular_option == 2
    assert selector.active_mission == "PERP_PARK"
    assert not selector.maybe_lock(family.decision_s_m("PERP_PARK"))
    assert selector.selected_variant_id() == "p2_t2_f1"


def test_unknown_or_both_blocked_falls_back_to_first_without_stop():
    family = route_family()
    unknown = ParkingBranchSelector(family)
    unknown.set_active_mission("PERP_PARK")
    assert unknown.maybe_lock(family.decision_s_m("PERP_PARK"))
    assert unknown.perpendicular_option == 1

    blocked = ParkingBranchSelector(family)
    blocked.set_active_mission("PERP_PARK")
    first = family.bay("PERP_PARK", 1).path[-1]
    second = family.bay("PERP_PARK", 2).path[-1]
    blocked.observe(
        VehiclePose2D(0.0, 0.0, 0.0),
        (
            ObstacleDisc(first[0], first[1], 0.2),
            ObstacleDisc(second[0], second[1], 0.2),
        ),
    )
    assert blocked.state(1) == BayState.BLOCKED
    assert blocked.state(2) == BayState.BLOCKED
    assert blocked.maybe_lock(family.decision_s_m("PERP_PARK"))
    assert blocked.perpendicular_option == 1


def test_real_course_parallel_decision_is_before_p1_reverse_point():
    family = route_family()
    p2_decision = family.decision_s_m("PARALLEL_PARK", perpendicular=1)
    p1_route = family.route(1, 1)
    p1_reverse = next(
        point.s_m for point in p1_route.waypoints
        if "PARALLEL_1_FORWARD_TO_REVERSE" in point.fsm_event_ids.split("|")
    )
    assert p2_decision < p1_reverse
