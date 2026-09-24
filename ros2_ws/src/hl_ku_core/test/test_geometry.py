import math

from hl_ku_core.geometry import (
    EnuProjector,
    antenna_to_base,
    blend_angle,
    heading_deg_to_enu_yaw,
    signed_speed_from_track,
)


def test_true_heading_to_ros_yaw():
    assert math.isclose(heading_deg_to_enu_yaw(0.0), math.pi / 2.0)
    assert math.isclose(heading_deg_to_enu_yaw(90.0), 0.0)
    assert math.isclose(heading_deg_to_enu_yaw(180.0), -math.pi / 2.0)
    assert math.isclose(
        heading_deg_to_enu_yaw(0.0, math.pi), -math.pi / 2.0
    )


def test_enu_projection_and_lever_arm():
    projector = EnuProjector(37.0, 127.0, 50.0)
    east, north, up = projector.project(37.0, 127.00001, 50.0)
    assert 0.85 < east < 0.95
    assert abs(north) < 0.01
    assert abs(up) < 0.01
    base_east, base_north = antenna_to_base(10.0, 20.0, 0.0, 0.5, 0.0)
    assert math.isclose(base_east, 9.5)
    assert math.isclose(base_north, 20.0)


def test_signed_doppler_speed_supports_reverse():
    assert math.isclose(signed_speed_from_track(1.0, 90.0, 0.0), 1.0)
    assert math.isclose(signed_speed_from_track(1.0, 270.0, 0.0), -1.0)


def test_angle_blend_uses_short_path_across_wrap():
    blended = blend_angle(math.radians(179.0), math.radians(-179.0), 0.5)
    assert math.isclose(abs(math.degrees(blended)), 180.0, abs_tol=1.0e-9)
