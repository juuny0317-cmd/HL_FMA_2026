import pytest

from hl_ku_core.teleop import (
    direct_signed_drive,
    fixed_steering_target,
    manual_control_active,
    manual_command_status,
    step_signed_drive,
    step_steering_target,
    three_state_signed_drive,
)


def test_fixed_steering_key_always_returns_the_same_target():
    assert fixed_steering_target(-1, 0.48) == pytest.approx(-0.48)
    assert fixed_steering_target(-1, 0.48) == pytest.approx(-0.48)
    assert fixed_steering_target(1, 0.48) == pytest.approx(0.48)


def test_fixed_steering_stays_active_after_the_key_deadline():
    assert manual_control_active(False, False, True, False)
    assert not manual_control_active(True, False, True, False)


def test_incremental_steering_moves_one_step_and_holds_each_target():
    steering = step_steering_target(0.0, -1, 0.048, 0.48)
    assert steering == pytest.approx(-0.048)
    steering = step_steering_target(steering, -1, 0.048, 0.48)
    assert steering == pytest.approx(-0.096)
    steering = step_steering_target(steering, 1, 0.048, 0.48)
    assert steering == pytest.approx(-0.048)
    assert manual_control_active(False, False, True, False)


def test_incremental_steering_clamps_at_both_ends():
    assert step_steering_target(-0.47, -1, 0.048, 0.48) == pytest.approx(-0.48)
    assert step_steering_target(0.47, 1, 0.048, 0.48) == pytest.approx(0.48)


def test_direct_drive_applies_the_selected_full_range_pwm():
    assert direct_signed_drive(1, 0.60, 1.0) == pytest.approx(0.60)
    assert direct_signed_drive(-1, 1.0, 1.0) == pytest.approx(-1.0)


def test_direct_drive_rejects_values_outside_the_protocol_range():
    with pytest.raises(ValueError):
        direct_signed_drive(1, 1.01, 1.0)


def test_three_state_drive_stops_before_changing_direction():
    drive = three_state_signed_drive(0.0, 1, 0.30, 1.0)
    assert drive == pytest.approx(0.30)

    drive = three_state_signed_drive(drive, -1, 0.30, 1.0)
    assert drive == pytest.approx(0.0)

    drive = three_state_signed_drive(drive, -1, 0.30, 1.0)
    assert drive == pytest.approx(-0.30)

    drive = three_state_signed_drive(drive, 1, 0.30, 1.0)
    assert drive == pytest.approx(0.0)

    drive = three_state_signed_drive(drive, 1, 0.30, 1.0)
    assert drive == pytest.approx(0.30)


def test_three_state_drive_keeps_the_selected_direction():
    assert three_state_signed_drive(0.30, 1, 0.30, 1.0) == pytest.approx(0.30)
    assert three_state_signed_drive(-0.30, -1, 0.30, 1.0) == pytest.approx(-0.30)


def test_w_and_s_change_pwm_by_ten_through_zero_into_reverse():
    assert step_signed_drive(0.0, 1, 0.10, 0.10, 1.0) == pytest.approx(0.10)
    assert step_signed_drive(0.10, 1, 0.10, 0.10, 1.0) == pytest.approx(0.20)
    assert step_signed_drive(0.20, 1, 0.10, 0.10, 1.0) == pytest.approx(0.30)
    assert step_signed_drive(0.30, -1, 0.10, 0.10, 1.0) == pytest.approx(0.20)
    assert step_signed_drive(0.20, -1, 0.10, 0.10, 1.0) == pytest.approx(0.10)
    assert step_signed_drive(0.10, -1, 0.10, 0.10, 1.0) == pytest.approx(0.0)
    assert step_signed_drive(0.0, -1, 0.10, 0.10, 1.0) == pytest.approx(-0.10)


def test_incremental_drive_stays_inside_protocol_range():
    assert step_signed_drive(0.98, 1, 0.10, 0.05, 1.0) == 1.0
    assert step_signed_drive(-0.98, -1, 0.10, 0.05, 1.0) == -1.0


def test_manual_status_shows_the_latest_input_and_actual_command():
    text = manual_command_status("W", 0.15, 0.10, -0.048, False, True, False)

    assert "입력=W" in text
    assert "구동=전진 +15/100" in text
    assert "조향=좌 -0.048 rad" in text
    assert "PWM단위=10/100" in text
    assert "상태=출력 중" in text


def test_manual_status_distinguishes_brake_and_fault():
    brake = manual_command_status("SPACE", 0.0, 0.10, 0.0, True, False, False)
    fault = manual_command_status("X", 0.0, 0.10, 0.0, True, False, True)
    steering_hold = manual_command_status(
        "A", 0.0, 0.30, -0.48, False, True, False
    )

    assert "상태=브레이크" in brake
    assert "상태=FAULT" in fault
    assert "상태=조향만 출력 중 · W/S 구동 가능" in steering_hold
