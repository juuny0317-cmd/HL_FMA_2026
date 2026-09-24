import argparse
import math

import pytest

from hl_ku_interfaces.msg import GnssStatus

from hl_ku_core.route_drive_session import (
    argument_parser,
    build_launch_command,
    format_rtk_status,
    resolve_options,
)


def options(**overrides):
    values = {
        "supervised": False,
        "route_file": None,
        "mission_speed": 0.30,
        "drive_command": 30,
        "prompt": False,
        "no_rviz": False,
    }
    values.update(overrides)
    return argparse.Namespace(**values)


def test_preview_launch_keeps_actuator_disabled():
    command = build_launch_command(options())
    assert "actuator_bridge_enabled:=false" in command
    assert "route_calibrated:=false" in command
    assert "drive_enabled:=false" in command
    assert "require_preflight:=true" in command


def test_supervised_launch_uses_requested_test_limits(tmp_path):
    route = tmp_path / "route.csv"
    route.write_text("index,x_m,y_m\n0,0,0\n1,1,0\n")
    command = build_launch_command(
        options(supervised=True, route_file=route, mission_speed=0.25)
    )
    assert "actuator_bridge_enabled:=true" in command
    assert "route_calibrated:=true" in command
    assert "drive_enabled:=true" in command
    assert "require_preflight:=false" in command
    assert "mission_speed_mps:=0.250" in command
    assert "maximum_drive_duty:=0.30" in command
    assert "minimum_forward_duty:=0.30" in command
    assert "maximum_drive_command:=30" in command
    assert f"route_file:={route}" in command


def test_cli_accepts_protocol_drive_range_and_rejects_above_it():
    parsed = resolve_options(
        argument_parser().parse_args(["--drive-command", "40"])
    )
    command = build_launch_command(parsed)
    assert "maximum_drive_duty:=0.40" in command
    assert "minimum_forward_duty:=0.40" in command
    assert "maximum_drive_command:=40" in command

    with pytest.raises(SystemExit):
        argument_parser().parse_args(["--drive-command", "101"])


def test_cli_speed_limit_is_one_meter_per_second():
    parsed = resolve_options(
        argument_parser().parse_args(["--mission-speed", "0.80"])
    )
    assert parsed.mission_speed == 0.80
    with pytest.raises(SystemExit):
        argument_parser().parse_args(["--mission-speed", "1.01"])


def test_interactive_prompt_reads_speed_and_drive_command():
    answers = iter(["0.40", "40"])
    parsed = argument_parser().parse_args(["--prompt"])
    resolved = resolve_options(parsed, input_fn=lambda _prompt: next(answers))
    assert resolved.mission_speed == 0.40
    assert resolved.drive_command == 40


def test_interactive_prompt_enter_uses_defaults():
    answers = iter(["", ""])
    parsed = argument_parser().parse_args(["--prompt"])
    resolved = resolve_options(parsed, input_fn=lambda _prompt: next(answers))
    assert resolved.mission_speed == 0.30
    assert resolved.drive_command == 30


def gnss_status(**overrides):
    status = GnssStatus()
    status.fix_type = GnssStatus.FIX_RTK_FIXED
    status.heading_valid = True
    status.position_valid = True
    status.velocity_valid = True
    status.satellites = 35
    status.hdop = 0.5
    status.correction_age_sec = 0.7
    status.heading_age_sec = 0.1
    status.nmea_checksum_valid = True
    for name, value in overrides.items():
        setattr(status, name, value)
    return status


def test_korean_rtk_status_reports_ready_fixed_solution():
    text = format_rtk_status(gnss_status())
    assert "RTK 연결: 정상 · 주행 가능" in text
    assert "측위=RTK 고정(FIXED)" in text
    assert "위성=35개" in text
    assert "보정나이=0.7초" in text


def test_korean_rtk_status_distinguishes_float_and_stale_correction():
    floating = format_rtk_status(
        gnss_status(fix_type=GnssStatus.FIX_RTK_FLOAT)
    )
    assert "RTK 연결: 수렴 중" in floating
    stale = format_rtk_status(gnss_status(correction_age_sec=5.0))
    assert "RTK 연결: 보정정보 지연" in stale


def test_korean_rtk_status_explains_dgps_and_missing_heading():
    text = format_rtk_status(
        gnss_status(
            fix_type=GnssStatus.FIX_DGPS,
            heading_valid=False,
            heading_age_sec=math.inf,
        )
    )
    assert "DGPS(고정 대기)" in text
    assert "헤딩=미수신(ANT2/GNTHS 확인)" in text
    assert "주행대기=RTK 고정, 헤딩" in text


def test_korean_rtk_status_explains_stale_heading_sentence():
    text = format_rtk_status(
        gnss_status(heading_valid=False, heading_age_sec=2.1)
    )
    assert "RTK 고정 · 안전입력 대기" in text
    assert "헤딩=시간초과(2.1초)" in text
    assert "주행대기=헤딩" in text


def test_supervised_quality_display_accepts_relaxed_satellite_and_hdop_limits():
    text = format_rtk_status(gnss_status(satellites=8, hdop=2.5))
    assert "RTK 연결: 정상 · 주행 가능" in text
    assert "주행대기=없음" in text

    blocked = format_rtk_status(gnss_status(satellites=7, hdop=2.6))
    assert "주행대기=위성 수, HDOP" in blocked
