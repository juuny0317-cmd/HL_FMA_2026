import pytest
from pathlib import Path
import yaml

from hl_ku_core.detour_following import DetourFollowResult
from hl_ku_core.detour_gate import select_detour_command
from hl_ku_core.safety_supervisor_node import lidar_emergency_brake_required
from hl_ku_interfaces.msg import MissionStatus


def decision(**changes):
    arguments = dict(
        enabled=True,
        in_s_obstacle=True,
        engaged=False,
        speed_mps=0.45,
        route_steering_rad=0.05,
        legacy_steering_offset_rad=0.10,
        lidar_fresh=True,
        obstacle_in_path=True,
        obstacle_distance_m=8.0,
        follow=DetourFollowResult("APPROACH"),
        approach_speed_limit_mps=0.30,
    )
    arguments.update(changes)
    return select_detour_command(**arguments)


def test_early_obstacle_slows_before_steering_begins():
    result = decision()
    assert result.speed_mps == 0.30
    assert result.steering_rad == 0.05
    assert not result.brake
    assert not result.engaged


def test_candidate_steering_replaces_legacy_gap_offset():
    result = decision(follow=DetourFollowResult("TRACK", -0.15))
    assert result.steering_rad == -0.15
    assert result.speed_mps == 0.30
    assert result.engaged
    assert not result.brake


def test_missing_or_stale_candidate_continues_without_braking():
    result = decision(
        obstacle_distance_m=5.2,
        follow=DetourFollowResult("UNAVAILABLE"),
    )
    assert not result.brake
    assert result.speed_mps == 0.30
    assert result.steering_rad == pytest.approx(0.15)
    assert not result.engaged

    stale_while_tracking = decision(
        engaged=True,
        lidar_fresh=False,
        follow=DetourFollowResult("UNAVAILABLE"),
    )
    assert not stale_while_tracking.brake
    assert stale_while_tracking.speed_mps == 0.45
    assert stale_while_tracking.steering_rad == 0.05
    assert not stale_while_tracking.engaged
    assert not decision(lidar_fresh=False, obstacle_in_path=False).brake


def test_completion_and_zone_exit_rejoin_without_braking():
    completed = decision(
        engaged=True,
        follow=DetourFollowResult("COMPLETE"),
    )
    assert not completed.brake
    assert not completed.engaged
    assert completed.steering_rad == 0.05

    premature_exit = decision(in_s_obstacle=False, engaged=True)
    assert not premature_exit.brake
    assert not premature_exit.engaged
    assert premature_exit.steering_rad == 0.05


def test_preview_candidate_alone_cannot_engage_or_start_steering():
    no_lidar = decision(
        obstacle_in_path=False,
        follow=DetourFollowResult("TRACK", -0.15),
    )
    assert not no_lidar.engaged
    assert no_lidar.steering_rad == 0.05


def test_disabled_mode_always_continues_with_route_and_gap_steering():
    baseline = decision(enabled=False)
    assert baseline.steering_rad == pytest.approx(0.15)
    assert baseline.speed_mps == 0.45
    assert not baseline.brake
    disabled_mid_detour = decision(enabled=False, engaged=True)
    assert not disabled_mid_detour.brake
    assert not disabled_mid_detour.engaged
    assert disabled_mid_detour.steering_rad == pytest.approx(0.15)


def test_s_obstacle_bypasses_global_lidar_emergency_brake():
    assert not lidar_emergency_brake_required(
        True,
        0.50,
        0.75,
        MissionStatus.STATE_S_OBSTACLE,
    )
    assert lidar_emergency_brake_required(
        True,
        0.50,
        0.75,
        MissionStatus.STATE_ROUTE,
    )


def test_live_detour_is_disabled_until_geometry_is_verified():
    config = yaml.safe_load(
        (Path(__file__).resolve().parents[1] / "config" / "system.yaml").read_text(
            encoding="utf-8"
        )
    )
    assert not config["mission_manager"]["ros__parameters"]["enable_local_detour_steering"]
    assert not config["local_detour"]["ros__parameters"]["live_geometry_verified"]
