from hl_ku_core.route import Route
import pytest


def test_load_and_query_route(tmp_path):
    path = tmp_path / "route.csv"
    path.write_text(
        "s_m,x_m,y_m,target_speed_mps,mission,direction\n"
        "0,0,0,0.5,NORMAL,1\n"
        "1,1,0,0.4,HILL,1\n"
        "2,2,0,0.2,PERP_PARK,-1\n",
        encoding="utf-8",
    )
    route = Route.load_csv(path)
    assert route.nearest_index(1.1, 0.0) == 1
    assert route.lookahead_index(0, 1.5) == 2
    assert route.waypoints[2].direction == -1


def test_finish_metadata_waits_for_final_point(tmp_path):
    path = tmp_path / "sparse_finish.csv"
    path.write_text(
        "s_m,x_m,y_m,target_speed_mps,mission,direction\n"
        "0,0,0,0.3,NORMAL,1\n"
        "16,16,0,0.0,FINISH,1\n",
        encoding="utf-8",
    )
    route = Route.load_csv(path)

    assert route.nearest_index(9.0, 0.0) == 1
    before_finish = route.metadata_waypoint(1, 9.0, 0.0, 0.5)
    at_finish = route.metadata_waypoint(1, 15.7, 0.0, 0.5)

    assert before_finish.mission == "NORMAL"
    assert before_finish.target_speed_mps == 0.3
    assert at_finish.mission == "FINISH"
    assert at_finish.target_speed_mps == 0.0


def test_route_bundle_selects_one_variant_and_maps_fsm_zone_to_runtime_mission(tmp_path):
    path = tmp_path / "mission.csv"
    path.write_text(
        "variant_id,s_m,x_m,y_m,target_speed_mps,mission,direction,"
        "fsm_zone,fsm_event_ids,fsm_actions,fsm_conditions,fsm_hold_sec\n"
        "p1,0,0,0,0,NORMAL,1,ROUTE,START,START,,0\n"
        "p1,1,1,0,0,NORMAL,1,S_CURVE,S_START,ENTER_ZONE,,0\n"
        "p2,0,100,0,0,NORMAL,1,ROUTE,START,START,,0\n"
        "p2,1,101,0,0,NORMAL,1,T_PARK,PARK,FOLLOW_ROUTE,,0\n",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="explicit variant_id"):
        Route.load_csv(path)
    route = Route.load_csv(path, "p1")

    assert len(route.waypoints) == 2
    assert route.waypoints[0].x_m == 0.0
    assert route.waypoints[1].fsm_zone == "S_CURVE"
    assert route.waypoints[1].mission == "S_OBSTACLE"
    assert route.active_event_waypoint(1).fsm_event_ids == "S_START"
    assert route.has_fsm_metadata
    with pytest.raises(ValueError, match="no waypoints"):
        Route.load_csv(path, "missing")


def test_route_curvature_and_nearest_segment_queries(tmp_path):
    straight_path = tmp_path / "straight.csv"
    straight_path.write_text(
        "s_m,x_m,y_m,target_speed_mps,mission,direction\n"
        "0,0,0,0.3,NORMAL,1\n"
        "1,1,0,0.3,NORMAL,1\n"
        "2,2,0,0.3,NORMAL,1\n"
        "3,3,0,0.3,NORMAL,1\n",
        encoding="utf-8",
    )
    straight = Route.load_csv(straight_path)
    assert straight.nearest_segment_index(1.4, 0.2, 1) == 1
    segment, route_s, projection_x, projection_y = straight.nearest_projection(
        1.4, 0.2, 1
    )
    assert segment == 1
    assert (route_s, projection_x, projection_y) == pytest.approx((1.4, 1.4, 0.0))
    assert straight.position_at_s(1.5) == pytest.approx((1.5, 0.0))
    assert straight.heading_at_s(1.5, 1.0) == pytest.approx(0.0)
    assert straight.curvature_ahead(0, 3.0, 3) == pytest.approx(0.0)

    bend_path = tmp_path / "bend.csv"
    bend_path.write_text(
        "s_m,x_m,y_m,target_speed_mps,mission,direction\n"
        "0,0,0,0.3,NORMAL,1\n"
        "1,1,0,0.3,NORMAL,1\n"
        "2,2,0,0.3,NORMAL,1\n"
        "3,2,1,0.3,NORMAL,1\n",
        encoding="utf-8",
    )
    bend = Route.load_csv(bend_path)
    assert bend.curvature_ahead(0, 3.0, 3) == pytest.approx(1.5707963268)


@pytest.mark.parametrize(
    "row",
    (
        "1,1,0,-0.4,NORMAL,1\n",
        "1,1,0,0.4,TYPO,1\n",
        "0,1,0,0.4,NORMAL,1\n",
    ),
)
def test_rejects_unsafe_route_values(tmp_path, row):
    path = tmp_path / "route.csv"
    path.write_text(
        "s_m,x_m,y_m,target_speed_mps,mission,direction\n"
        "0,0,0,0.5,NORMAL,1\n" + row,
        encoding="utf-8",
    )
    with pytest.raises(ValueError):
        Route.load_csv(path)
