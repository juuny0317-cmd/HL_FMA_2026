from hl_ku_core.safety_supervisor_node import required_safety_inputs
from hl_ku_core.nucleo_serial import drive_duty_to_command
from hl_ku_interfaces.msg import MissionStatus
from hl_ku_core.vehicle_controller_node import (
    AdaptiveHillHold,
    fixed_pwm_duty,
    fixed_pwm_for_mission,
    velocity_input_ready,
)


def test_direct_pwm_keeps_requested_magnitude_for_forward_reverse_and_stops():
    assert fixed_pwm_duty(0.25, 30) == 0.30
    assert fixed_pwm_duty(-0.25, 30) == -0.30
    assert fixed_pwm_duty(0.0, 30) == 0.0
    assert fixed_pwm_duty(-0.25, 0) == 0.0
    assert drive_duty_to_command(fixed_pwm_duty(0.25, 100), 100, 100) == 100
    assert drive_duty_to_command(fixed_pwm_duty(-0.25, 30), 100, 100) == -30


def test_hill_states_select_approach_and_post_stop_pwm():
    assert fixed_pwm_for_mission(MissionStatus.STATE_ROUTE, 30, 50, 30) == 30
    assert (
        fixed_pwm_for_mission(MissionStatus.STATE_HILL_APPROACH, 30, 50, 30)
        == 50
    )
    assert fixed_pwm_for_mission(MissionStatus.STATE_HILL_HOLD, 30, 50, 30) == 30
    assert (
        fixed_pwm_for_mission(MissionStatus.STATE_HILL_CLIMB, 30, 50, 30)
        == 30
    )
    assert fixed_pwm_for_mission(MissionStatus.STATE_HILL_APPROACH, 25, -1, -1) == 25


def test_adaptive_hill_hold_steps_only_after_two_seconds_of_rollback():
    hold = AdaptiveHillHold(
        step_duty=0.03,
        step_interval_sec=2.0,
        reverse_speed_threshold_mps=0.04,
        maximum_duty=1.0,
    )

    assert hold.update(
        active=True, base_duty=0.10, velocity_mps=0.0, now_sec=0.0
    ) == (0.10, False)
    assert hold.update(
        active=True, base_duty=0.10, velocity_mps=-0.05, now_sec=0.5
    ) == (0.10, False)
    assert hold.update(
        active=True, base_duty=0.10, velocity_mps=-0.05, now_sec=2.49
    ) == (0.10, False)
    assert hold.update(
        active=True, base_duty=0.10, velocity_mps=-0.05, now_sec=2.5
    ) == (0.13, True)
    assert hold.update(
        active=True, base_duty=0.10, velocity_mps=-0.05, now_sec=4.5
    ) == (0.16, True)

    # Once rollback stops, the balancing command is retained without further steps.
    assert hold.update(
        active=True, base_duty=0.10, velocity_mps=0.0, now_sec=5.0
    ) == (0.16, False)
    assert hold.update(
        active=False, base_duty=0.10, velocity_mps=0.0, now_sec=6.0
    ) == (0.0, False)
    assert hold.update(
        active=True, base_duty=0.10, velocity_mps=0.0, now_sec=7.0
    ) == (0.10, False)


def test_relaxed_route_keeps_runtime_inputs_and_drops_gnss_quality_gate():
    inputs = {name for name, _timeout in required_safety_inputs(False, True)}

    assert inputs == {"raw", "pose", "feedback", "mission"}
    assert "gnss" not in inputs


def test_relaxed_route_can_use_open_loop_duty_without_velocity_message():
    assert velocity_input_ready(False, required=False)
    assert not velocity_input_ready(False, required=True)
    assert velocity_input_ready(True, required=True)
