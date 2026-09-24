import pytest

from hl_ku_core.waypoint_capture import (
    FixedPositionSample,
    average_samples,
    build_route_points,
    sample_quality_issue,
    waypoint_spacing_m,
)


def _sample(**overrides):
    values = {
        "received_monotonic_sec": 10.0,
        "stamp_sec": 1000.0,
        "latitude_deg": 37.0,
        "longitude_deg": 127.0,
        "altitude_m": 50.0,
        "fix_type": 4,
        "position_valid": True,
        "satellites": 20,
        "hdop": 0.7,
        "correction_age_sec": 0.8,
        "nmea_checksum_valid": True,
    }
    values.update(overrides)
    return FixedPositionSample(**values)


def test_only_fresh_quality_rtk_fixed_sample_is_accepted():
    assert sample_quality_issue(_sample(), 15, 1.5, 2.0) is None
    assert "RTK FIX" in sample_quality_issue(_sample(fix_type=5), 15, 1.5, 2.0)
    assert "satellites" in sample_quality_issue(
        _sample(satellites=10), 15, 1.5, 2.0
    )
    assert "correction_age" in sample_quality_issue(
        _sample(correction_age_sec=3.0), 15, 1.5, 2.0
    )


def test_average_and_route_are_referenced_to_first_waypoint():
    first = average_samples([_sample(), _sample(stamp_sec=1000.1)])
    second = average_samples(
        [
            _sample(latitude_deg=37.00001, stamp_sec=1001.0),
            _sample(latitude_deg=37.00001, stamp_sec=1001.1),
        ]
    )
    route = build_route_points([first, second], 0.3)
    assert route[0].x_m == pytest.approx(0.0, abs=1.0e-6)
    assert route[0].y_m == pytest.approx(0.0, abs=1.0e-6)
    assert route[0].mission == "NORMAL"
    assert route[1].s_m == pytest.approx(1.11, abs=0.03)
    assert route[1].mission == "FINISH"
    assert route[1].target_speed_mps == 0.0


def test_spacing_helper_handles_duplicate_and_close_waypoints_without_route_error():
    first = average_samples([_sample()])
    duplicate = average_samples([_sample(stamp_sec=1001.0)])
    close = average_samples(
        [_sample(longitude_deg=127.0000001, stamp_sec=1002.0)]
    )

    assert waypoint_spacing_m(first, duplicate) == pytest.approx(0.0)
    assert 0.0 < waypoint_spacing_m(first, close) < 0.10
