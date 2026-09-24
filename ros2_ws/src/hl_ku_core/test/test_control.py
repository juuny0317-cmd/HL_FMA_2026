import pytest

from hl_ku_core.control import (
    SpeedPiController,
    blend_stanley_pure_pursuit,
    pure_pursuit_steering,
    pure_pursuit_weight_for_curvature,
    stanley_steering,
)


def controller() -> SpeedPiController:
    return SpeedPiController(
        kp=0.1,
        ki=0.0,
        integral_limit=1.0,
        maximum_duty=0.8,
        deadband_speed_mps=0.03,
        forward_speeds=[0.2, 1.0],
        forward_duties=[0.15, 0.4],
        reverse_speeds=[0.2, 0.6],
        reverse_duties=[0.18, 0.35],
        nominal_voltage=22.2,
    )


def test_speed_controller_forward_reverse_and_stop():
    speed = controller()
    assert speed.update(0.6, 0.0, 22.2, 0.02) > 0.0
    assert speed.update(-0.4, 0.0, 22.2, 0.02) < 0.0
    assert speed.update(0.0, 0.0, 22.2, 0.02) == 0.0


def test_speed_controller_can_apply_a_forward_breakaway_duty():
    speed = controller()
    speed.minimum_forward_duty = 0.30
    assert speed.update(0.10, 0.0, 22.2, 0.02) == 0.30
    assert speed.update(0.0, 0.0, 22.2, 0.02) == 0.0


def test_pure_pursuit_steering_sign_and_limit():
    left = pure_pursuit_steering(0, 0, 0, 2, 1, 0.58, 0.4)
    right = pure_pursuit_steering(0, 0, 0, 2, -1, 0.58, 0.4)
    assert -0.4 <= left < 0.0
    assert 0.0 < right <= 0.4


def test_stanley_uses_t870_negative_left_positive_right_convention():
    path_is_left = stanley_steering(
        0.0, -0.10, 0.0, 0.0, 0.0, 2.0, 0.0, 0.30, 0.65, 0.65, 0.58, 0.48
    )
    path_is_right = stanley_steering(
        0.0, 0.10, 0.0, 0.0, 0.0, 2.0, 0.0, 0.30, 0.65, 0.65, 0.58, 0.48
    )

    assert path_is_left.cross_track_error_m > 0.0
    assert path_is_left.steering_rad < 0.0
    assert path_is_right.cross_track_error_m < 0.0
    assert path_is_right.steering_rad > 0.0


def test_stanley_heading_term_returns_vehicle_to_route_heading():
    terms = stanley_steering(
        0.0, 0.0, 0.20, 0.0, 0.0, 2.0, 0.0, 0.30, 0.65, 0.65, 0.58, 0.48
    )
    assert terms.heading_error_rad == pytest.approx(-0.20)
    assert terms.steering_rad > 0.0


def test_stanley_heading_gain_can_dampen_heading_correction():
    full = stanley_steering(
        0.0, 0.0, 0.20, 0.0, 0.0, 2.0, 0.0, 0.30, 0.65, 0.65, 0.58, 0.48
    )
    damped = stanley_steering(
        0.0, 0.0, 0.20, 0.0, 0.0, 2.0, 0.0, 0.30, 0.65, 0.65, 0.58, 0.48, 0.70
    )

    assert abs(damped.steering_rad) < abs(full.steering_rad)


def test_curvature_continuously_moves_authority_from_stanley_to_pursuit():
    straight = pure_pursuit_weight_for_curvature(0.02, 0.08, 0.25, 0.20, 0.85)
    transition = pure_pursuit_weight_for_curvature(0.165, 0.08, 0.25, 0.20, 0.85)
    curve = pure_pursuit_weight_for_curvature(0.40, 0.08, 0.25, 0.20, 0.85)

    assert straight == pytest.approx(0.20)
    assert transition == pytest.approx(0.525)
    assert curve == pytest.approx(0.85)
    assert blend_stanley_pure_pursuit(0.10, 0.30, straight, 0.48) == pytest.approx(0.14)
    assert blend_stanley_pure_pursuit(0.10, 0.30, curve, 0.48) == pytest.approx(0.27)
