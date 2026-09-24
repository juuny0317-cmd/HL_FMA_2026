from hl_ku_core.mission import (
    LIGHT_GREEN,
    LIGHT_RED,
    MissionCoordinator,
    Observation,
    State,
)
from hl_ku_core.traffic_light import (
    SIGNAL_GREEN_BIT,
    SIGNAL_LEFT_BIT,
    SIGNAL_RED_BIT,
    SIGNAL_YELLOW_BIT,
    TrafficSignalVoter,
)


def observation(now, zone="NORMAL", speed=0.0, **kwargs):
    return Observation(now_sec=now, zone=zone, route_s_m=kwargs.pop("route_s_m", 0.0), speed_mps=speed, **kwargs)


def test_not_armed_is_braked():
    coordinator = MissionCoordinator()
    decision = coordinator.update(observation(0.0))
    assert decision.state == State.INIT
    assert decision.brake


def test_operator_toggle_starts_pauses_and_resumes():
    coordinator = MissionCoordinator(default_speed_mps=0.3)

    success, message = coordinator.toggle_run(1.0)
    assert success
    assert message == "mission started"
    driving = coordinator.update(observation(1.1))
    assert driving.state == State.ROUTE
    assert not driving.brake

    success, message = coordinator.toggle_run(2.0)
    assert success
    assert message == "mission paused"
    paused = coordinator.update(observation(3.0))
    assert paused.state == State.ROUTE
    assert paused.brake
    assert paused.speed_limit_mps == 0.0

    success, message = coordinator.toggle_run(4.0)
    assert success
    assert message == "mission resumed"
    resumed = coordinator.update(observation(4.1))
    assert resumed.state == State.ROUTE
    assert not resumed.brake


def test_fault_rejects_operator_toggle():
    coordinator = MissionCoordinator()
    coordinator.fault()
    success, message = coordinator.toggle_run(1.0)
    assert not success
    assert "fault" in message


def test_restart_context_restores_owning_fsm_and_skips_passed_hill_stop():
    coordinator = MissionCoordinator()
    coordinator.arm()
    coordinator.resume_at_route("S_OBSTACLE", "S_CURVE_START")
    assert coordinator.state == State.S_OBSTACLE

    coordinator.resume_at_route("HILL", "HILL_STOP")
    assert coordinator.state == State.HILL_CLIMB
    resumed = coordinator.update(
        observation(
            1.0,
            "HILL",
            route_event_id="HILL_STOP",
            route_event_action="STOP_THEN_HOLD",
            route_event_condition="ALWAYS_ONCE",
            route_has_fsm_metadata=True,
        )
    )
    assert resumed.state == State.HILL_CLIMB
    assert not resumed.brake


def test_pause_time_does_not_count_toward_hill_hold():
    coordinator = MissionCoordinator(hill_stop_s_m=0.0, hill_hold_sec=3.0)
    coordinator.arm()
    coordinator.update(observation(0.0, "HILL", route_s_m=0.0))
    assert coordinator.state == State.HILL_HOLD

    coordinator.toggle_run(0.0)
    coordinator.toggle_run(10.0)
    still_holding = coordinator.update(observation(10.1, "HILL", route_s_m=0.0))
    assert still_holding.state == State.HILL_HOLD
    launch = coordinator.update(observation(13.1, "HILL", route_s_m=0.0))
    assert launch.state == State.HILL_CLIMB


def test_hill_requires_full_hold_time():
    coordinator = MissionCoordinator(hill_stop_s_m=10.0, hill_hold_sec=3.0)
    coordinator.arm()
    stopped = coordinator.update(observation(1.0, "HILL", route_s_m=10.0))
    assert stopped.state == State.HILL_HOLD
    assert stopped.brake
    holding = coordinator.update(observation(3.99, "HILL", route_s_m=10.0))
    assert holding.state == State.HILL_HOLD
    launch = coordinator.update(observation(4.0, "HILL", route_s_m=10.0))
    assert launch.state == State.HILL_CLIMB
    assert not launch.brake


def test_hill_rollback_enters_powered_hold_before_full_stop():
    coordinator = MissionCoordinator(hill_stop_s_m=10.0, hill_hold_sec=3.0)
    coordinator.arm()

    rollback = coordinator.update(
        observation(1.0, "HILL", speed=-0.08, route_s_m=10.0)
    )

    assert rollback.state == State.HILL_HOLD
    assert rollback.brake

    # The mandatory hold must still be counted only after motion settles.
    still_rolling = coordinator.update(
        observation(3.0, "HILL", speed=-0.08, route_s_m=10.0)
    )
    assert still_rolling.state == State.HILL_HOLD
    settled = coordinator.update(
        observation(3.1, "HILL", speed=0.0, route_s_m=10.0)
    )
    assert settled.state == State.HILL_HOLD
    assert coordinator.update(
        observation(5.99, "HILL", speed=0.0, route_s_m=10.0)
    ).state == State.HILL_HOLD
    assert coordinator.update(
        observation(6.0, "HILL", speed=0.0, route_s_m=10.0)
    ).state == State.HILL_CLIMB


def test_hill_stop_and_hold_are_driven_by_csv_event_metadata():
    coordinator = MissionCoordinator(hill_stop_s_m=999.0, hill_hold_sec=99.0)
    coordinator.arm()
    approach = coordinator.update(
        observation(
            1.0,
            "HILL",
            speed=0.3,
            route_s_m=2.0,
            route_event_id="HILL_START",
            route_has_fsm_metadata=True,
        )
    )
    assert approach.state == State.HILL_APPROACH
    assert not approach.brake

    approaching_stop = coordinator.update(
        observation(
            2.0,
            "HILL",
            speed=0.3,
            route_s_m=4.0,
            route_event_id="HILL_STOP",
            route_event_action="STOP_THEN_HOLD",
            route_event_condition="ALWAYS_ONCE",
            route_event_hold_sec=2.0,
            route_has_fsm_metadata=True,
        )
    )
    assert approaching_stop.state == State.HILL_APPROACH
    assert approaching_stop.brake

    stopped = coordinator.update(
        observation(
            3.0,
            "HILL",
            speed=0.0,
            route_event_id="HILL_STOP",
            route_event_action="STOP_THEN_HOLD",
            route_event_condition="ALWAYS_ONCE",
            route_event_hold_sec=2.0,
            route_has_fsm_metadata=True,
        )
    )
    assert stopped.state == State.HILL_HOLD
    assert stopped.brake
    assert coordinator.update(
        observation(
            4.9,
            "HILL",
            speed=0.0,
            route_event_id="HILL_STOP",
            route_event_action="STOP_THEN_HOLD",
            route_event_condition="ALWAYS_ONCE",
            route_event_hold_sec=2.0,
            route_has_fsm_metadata=True,
        )
    ).state == State.HILL_HOLD
    launch = coordinator.update(
        observation(
            5.0,
            "HILL",
            speed=0.0,
            route_event_id="HILL_STOP",
            route_event_action="STOP_THEN_HOLD",
            route_event_condition="ALWAYS_ONCE",
            route_event_hold_sec=2.0,
            route_has_fsm_metadata=True,
        )
    )
    assert launch.state == State.HILL_CLIMB
    assert not launch.brake


def test_traffic_red_stop_green_debounce():
    coordinator = MissionCoordinator(green_debounce_sec=0.35)
    coordinator.arm()
    red = coordinator.update(
        observation(
            1.0,
            "TRAFFIC",
            stop_line_detected=True,
            stop_line_distance_m=1.0,
            traffic_light=LIGHT_RED,
        )
    )
    assert red.state == State.TRAFFIC_WAIT
    first_green = coordinator.update(observation(2.0, "TRAFFIC", traffic_light=LIGHT_GREEN))
    assert first_green.brake
    go = coordinator.update(observation(2.4, "TRAFFIC", traffic_light=LIGHT_GREEN))
    assert go.state == State.ROUTE
    assert not go.brake


def test_yolo_traffic_votes_stop_and_release_without_stop_line():
    coordinator = MissionCoordinator(default_speed_mps=0.3)
    coordinator.arm()
    stopped = coordinator.update(
        observation(
            1.0,
            "TRAFFIC",
            traffic_votes_active=True,
            traffic_red_confirmed=True,
        )
    )
    assert stopped.state == State.TRAFFIC_WAIT
    assert stopped.brake
    released = coordinator.update(
        observation(
            2.0,
            "TRAFFIC",
            traffic_votes_active=True,
            traffic_left_confirmed=True,
        )
    )
    assert released.state == State.ROUTE
    assert not released.brake


def test_default_traffic_voter_confirms_every_signal_in_one_frame():
    voter = TrafficSignalVoter()
    for signal in (
        SIGNAL_RED_BIT,
        SIGNAL_YELLOW_BIT,
        SIGNAL_GREEN_BIT,
        SIGNAL_LEFT_BIT,
    ):
        assert voter.update(signal).confirmed(signal)


def test_lidar_dummy_stops_at_three_metres_holds_and_rearms_after_clear():
    coordinator = MissionCoordinator(
        default_speed_mps=0.3,
        dummy_trigger_m=3.0,
        dummy_hold_sec=3.2,
    )
    coordinator.arm()

    outside_trigger = coordinator.update(
        observation(
            0.5,
            "DUMMY",
            speed=0.3,
            lidar_dummy_stop_active=True,
            lidar_fresh=True,
            obstacle_in_path=True,
            obstacle_distance_m=3.01,
        )
    )
    assert outside_trigger.state == State.DUMMY_ARMED
    assert not outside_trigger.brake

    braking = coordinator.update(
        observation(
            1.0,
            "DUMMY",
            speed=0.3,
            lidar_dummy_stop_active=True,
            lidar_fresh=True,
            obstacle_in_path=True,
            obstacle_distance_m=3.0,
        )
    )
    assert braking.state == State.DUMMY_BRAKE
    assert braking.brake

    stopped = coordinator.update(
        observation(
            2.0,
            "DUMMY",
            speed=0.0,
            lidar_dummy_stop_active=True,
            lidar_fresh=True,
            pedestrian_speed_fresh=True,
            obstacle_in_path=True,
            obstacle_distance_m=2.0,
        )
    )
    assert stopped.state == State.DUMMY_HOLD

    wait = coordinator.update(
        observation(
            5.2,
            "DUMMY",
            speed=0.0,
            lidar_dummy_stop_active=True,
            lidar_fresh=True,
            pedestrian_speed_fresh=True,
            obstacle_in_path=True,
        )
    )
    assert wait.brake

    stale_clear = coordinator.update(
        observation(
            5.3,
            "DUMMY",
            speed=0.0,
            lidar_dummy_stop_active=True,
            lidar_fresh=False,
            pedestrian_speed_fresh=True,
            obstacle_in_path=False,
        )
    )
    assert stale_clear.state == State.DUMMY_HOLD
    assert stale_clear.brake

    cleared = coordinator.update(
        observation(
            5.4,
            "DUMMY",
            speed=0.0,
            lidar_dummy_stop_active=True,
            lidar_fresh=True,
            pedestrian_speed_fresh=True,
            obstacle_in_path=False,
        )
    )
    assert cleared.state == State.DUMMY_ARMED
    assert not cleared.brake

    second_obstacle = coordinator.update(
        observation(
            5.5,
            "DUMMY",
            speed=0.3,
            lidar_dummy_stop_active=True,
            lidar_fresh=True,
            obstacle_in_path=True,
            obstacle_distance_m=2.5,
        )
    )
    assert second_obstacle.state == State.DUMMY_BRAKE
    assert second_obstacle.brake


def test_lidar_dummy_stays_stopped_when_scan_is_stale():
    coordinator = MissionCoordinator(dummy_trigger_m=3.0)
    coordinator.arm()

    stale = coordinator.update(
        observation(
            1.0,
            "DUMMY",
            speed=0.3,
            lidar_dummy_stop_active=True,
            lidar_fresh=False,
        )
    )

    assert stale.state == State.DUMMY_ARMED
    assert stale.brake
    assert stale.speed_limit_mps == 0.0


def test_pedestrian_yolo_stops_then_releases_on_clear_frames():
    coordinator = MissionCoordinator(
        default_speed_mps=0.3,
        pedestrian_hold_sec=3.5,
    )
    coordinator.arm()
    braking = coordinator.update(
        observation(
            1.0,
            "DUMMY",
            speed=0.3,
            pedestrian_mode_active=True,
            pedestrian_stop_confirmed=True,
        )
    )
    assert braking.state == State.DUMMY_BRAKE
    stopped = coordinator.update(
        observation(
            2.0,
            "DUMMY",
            speed=0.0,
            pedestrian_mode_active=True,
            pedestrian_speed_fresh=True,
        )
    )
    assert stopped.state == State.DUMMY_HOLD
    clear_but_too_early = coordinator.update(
        observation(
            5.4,
            "DUMMY",
            speed=0.0,
            pedestrian_mode_active=True,
            pedestrian_clear_confirmed=True,
        )
    )
    assert clear_but_too_early.state == State.DUMMY_HOLD
    assert clear_but_too_early.brake
    released = coordinator.update(
        observation(
            5.5,
            "DUMMY",
            speed=0.0,
            pedestrian_mode_active=True,
            pedestrian_clear_confirmed=True,
        )
    )
    assert released.state == State.ROUTE
    assert not released.brake


def test_pedestrian_yolo_stays_stopped_after_hold_until_clear():
    coordinator = MissionCoordinator(pedestrian_hold_sec=3.5)
    coordinator.arm()
    coordinator.update(
        observation(
            1.0,
            "DUMMY",
            speed=0.3,
            pedestrian_mode_active=True,
            pedestrian_stop_confirmed=True,
        )
    )
    coordinator.update(
        observation(
            2.0,
            "DUMMY",
            speed=0.0,
            pedestrian_mode_active=True,
            pedestrian_speed_fresh=True,
        )
    )

    still_present = coordinator.update(
        observation(
            6.0,
            "DUMMY",
            speed=0.0,
            pedestrian_mode_active=True,
            pedestrian_clear_confirmed=False,
        )
    )

    assert still_present.state == State.DUMMY_HOLD
    assert still_present.brake
