import curses
import math
import subprocess
from pathlib import Path
from types import SimpleNamespace

import yaml
import pytest
from geometry_msgs.msg import PoseWithCovarianceStamped

from hl_ku_interfaces.msg import GnssStatus
from hl_ku_core.route import Route
from hl_ku_core.mando_tui import (
    ConsoleConfig,
    MandoConsole,
    ProcessSlot,
    Sample,
    Scenario,
    Variant,
    alignment_error,
    apply_datum_to_gps_config,
    automatic_placement_reason,
    build_datum_command,
    build_route_command,
    gnss_blockers,
    fit_terminal_text,
    key_input_label,
    load_config,
    parking_status_summary,
    place_variant_at_pose,
    pose_xy_heading_deg,
    relaxed_readiness,
    route_geometry,
    static_readiness,
    status_mark,
    supervised_readiness,
    table_border,
    table_row,
    terminal_text_width,
    variant_compact_summary,
    variant_detail_lines,
    variant_display_name,
)


def test_parking_status_summary_is_operator_readable():
    assert parking_status_summary(
        '{"mission":"PERP_PARK","locked":false,"option_1":"BLOCKED",'
        '"option_2":"CLEAR","selected_t":2}'
    ) == "T1=BLOCKED  T2=CLEAR  → T2  관찰 중"
    assert parking_status_summary(
        '{"mission":"PARALLEL_PARK","locked":true,"option_1":"CLEAR",'
        '"option_2":"BLOCKED","selected_p":1}'
    ) == "P1=CLEAR  P2=BLOCKED  → P1  선택 고정"


def _write_yaml(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(value), encoding="utf-8")


def _ready_fixture(tmp_path: Path) -> tuple[ConsoleConfig, Variant]:
    route = tmp_path / "routes" / "straight.csv"
    route.parent.mkdir()
    route.write_text(
        "s_m,x_m,y_m,target_speed_mps,mission,direction\n"
        "0.0,0.0,0.0,0.30,NORMAL,1\n"
        "5.0,5.0,0.0,0.00,FINISH,1\n",
        encoding="utf-8",
    )
    datum = {
        "datum_latitude_deg": 37.5,
        "datum_longitude_deg": 127.0,
        "datum_altitude_m": 20.0,
        "base_to_ant1_x_m": 0.802,
        "base_to_ant1_y_m": 0.0,
        "heading_mount_offset_deg": 180.0,
    }
    gps = tmp_path / "gps.yaml"
    _write_yaml(
        gps,
        {
            "gnss_localizer": {
                "ros__parameters": {"datum_configured": True, **datum}
            }
        },
    )
    calibration = tmp_path / "calibration.yaml"
    _write_yaml(
        calibration,
        {
            "gnss": datum,
            "course": {
                "route_file": route.name,
                "route_calibrated": True,
            },
        },
    )
    for name in ("system.yaml", "receiver.yaml", "ntrip.yaml"):
        _write_yaml(tmp_path / name, {})
    variant = Variant(
        "straight",
        route,
        calibration,
        preflight="supervised",
        alignment_confirmed=True,
        minimum_length_m=5.0,
        maximum_length_m=10.0,
    )
    config = ConsoleConfig(
        tmp_path / "mando.yaml",
        tmp_path / "system.yaml",
        gps,
        tmp_path / "receiver.yaml",
        tmp_path / "ntrip.yaml",
        tmp_path / "record.sh",
        tmp_path / "finger.sh",
        tmp_path / "logs",
        True,
        (Scenario(1, "straight", "route", 30, (variant,)),),
    )
    return config, variant


def test_config_resolves_relative_paths_and_numbered_variants(tmp_path):
    document = {
        "version": 1,
        "common": {
            "system_config": "config/system.yaml",
            "gps_config": "config/gps.yaml",
            "receiver_config": "config/receiver.yaml",
            "ntrip_config": "config/ntrip.yaml",
            "recording_script": "record.sh",
            "finger_drive_script": "finger.sh",
        },
        "scenarios": [
            {
                "number": 3,
                "name": "sections",
                "kind": "route",
                "default_pwm": 25,
                "variant_defaults": {
                    "source_route": "routes/source.csv",
                    "source_variant_id": "p1_t1_f1",
                    "auto_place_at_current_pose": True,
                    "preflight": "relaxed",
                    "start_position_tolerance_m": 0.5,
                },
                "variants": [
                    {
                        "name": "A",
                        "route_file": "routes/a.csv",
                        "calibration_file": "config/a.yaml",
                        "source_start_s_m": 44.0,
                        "source_length_m": 8.0,
                        "description": "course_07 s=44~52m 직선",
                        "segment_code": "S-FULL",
                        "fsm_label": "S_CURVE",
                        "direction_summary": "전진",
                        "footprint": "21x23m",
                        "run_summary": "전체 형상",
                        "preserve_source_profile": True,
                        "mission_override": "S_OBSTACLE",
                        "allow_reverse": True,
                        "direction_change_hold_sec": 0.8,
                    },
                    {
                        "name": "B",
                        "route_file": "routes/b.csv",
                        "calibration_file": "config/b.yaml",
                    },
                ],
            },
            {
                "number": 4,
                "name": "finger",
                "kind": "finger",
                "default_pwm": 30,
                "maximum_pwm": 100,
            },
            {
                "number": 5,
                "name": "datum",
                "kind": "datum",
                "duration_sec": 120,
                "minimum_samples": 50,
                "maximum_hdop": 2.5,
            },
        ],
    }
    path = tmp_path / "mando.yaml"
    _write_yaml(path, document)

    config = load_config(path)

    assert [scenario.number for scenario in config.scenarios] == [3, 4, 5]
    assert config.scenarios[0].variants[1].route_file == tmp_path / "routes/b.csv"
    assert config.scenarios[0].variants[0].source_route == tmp_path / "routes/source.csv"
    assert config.scenarios[0].variants[0].source_variant_id == "p1_t1_f1"
    assert config.scenarios[0].variants[0].source_start_s_m == 44.0
    assert config.scenarios[0].default_speed == 25
    assert config.scenarios[0].variants[0].source_length_m == 8.0
    assert config.scenarios[0].variants[0].auto_place_at_current_pose
    assert config.scenarios[0].variants[0].preflight == "relaxed"
    assert config.scenarios[0].variants[0].start_position_tolerance_m == 0.5
    assert config.scenarios[0].variants[0].description == "course_07 s=44~52m 직선"
    assert config.scenarios[0].variants[0].segment_code == "S-FULL"
    assert config.scenarios[0].variants[0].fsm_label == "S_CURVE"
    assert config.scenarios[0].variants[0].preserve_source_profile
    assert config.scenarios[0].variants[0].mission_override == "S_OBSTACLE"
    assert config.scenarios[0].variants[0].allow_reverse
    assert config.scenarios[0].variants[0].direction_change_hold_sec == 0.8
    assert config.scenarios[1].maximum_pwm == 100
    assert config.scenarios[2].default_speed == 120
    assert config.scenarios[2].survey_minimum_samples == 50
    assert not config.require_recording


def test_config_accepts_zero_to_disable_start_distance_gate(tmp_path):
    document = {
        "version": 1,
        "common": {
            "system_config": "config/system.yaml",
            "gps_config": "config/gps.yaml",
            "receiver_config": "config/receiver.yaml",
            "ntrip_config": "config/ntrip.yaml",
            "recording_script": "record.sh",
            "finger_drive_script": "finger.sh",
        },
        "scenarios": [
            {
                "number": 1,
                "name": "full course",
                "kind": "route",
                "default_pwm": 30,
                "variants": [
                    {
                        "name": "resume anywhere",
                        "route_file": "routes/full.csv",
                        "calibration_file": "config/full.yaml",
                        "start_position_tolerance_m": 0.0,
                    }
                ],
            }
        ],
    }
    path = tmp_path / "mando.yaml"
    _write_yaml(path, document)

    config = load_config(path)

    assert config.scenarios[0].variants[0].start_position_tolerance_m == 0.0


def test_supervised_readiness_checks_rigid_route_and_calibration(tmp_path):
    config, variant = _ready_fixture(tmp_path)

    assert supervised_readiness(config, variant).ok
    unconfirmed = Variant(
        **{**variant.__dict__, "alignment_confirmed": False}
    )
    assert "alignment_confirmed" in supervised_readiness(config, unconfirmed).summary()


def test_route_launch_uses_selected_pwm_for_both_directions(tmp_path):
    config, variant = _ready_fixture(tmp_path)

    command = build_route_command(config, variant, 25)

    assert command[:4] == [
        "ros2", "launch", "hl_ku_foxglove", "mando_live.launch.py"
    ]
    assert "drive_enabled:=true" in command
    assert "camera_enabled:=true" in command
    assert "active_variant_name:=straight" in command
    assert any(item.startswith("source_route_file:=") for item in command)
    assert not any(item.startswith("source_variant_id:=") for item in command)
    assert "source_start_s_m:=0.000" in command
    assert "source_length_m:=0.000" in command
    assert "source_segment_name:=straight" in command
    assert not any(item.startswith("source_fsm_label:=") for item in command)
    assert "allow_reverse:=false" in command
    assert "direction_change_hold_sec:=0.75" in command
    assert "route_calibrated:=true" in command
    assert "require_preflight:=false" in command
    assert "mission_speed_mps:=0.300" in command
    assert "fixed_drive_pwm:=25" in command
    assert "maximum_drive_duty:=1.00" in command
    assert "minimum_forward_duty:=0.00" in command
    assert "maximum_drive_command:=100" in command
    assert "fixed_drive_pwm:=100" in build_route_command(config, variant, 100)
    assert "fixed_drive_pwm:=0" in build_route_command(config, variant, 0)
    assert "enable_local_detour_steering:=false" in command
    assert "lidar_geometry_verified:=false" in command

    described = Variant(
        **{
            **variant.__dict__,
            "description": "원본 course_07 · s=44.0~52.0m · 약 7.75m 직선",
        }
    )
    assert variant_display_name(described).startswith("straight · 원본 course_07")
    assert any(
        item.startswith("active_variant_name:=straight · 원본 course_07")
        for item in build_route_command(config, described, 25)
    )

    labeled = Variant(
        **{
            **variant.__dict__,
            "name": "S자 전체",
            "segment_code": "S-FULL",
            "fsm_label": "S_CURVE",
            "direction_summary": "전진 37.67m",
            "footprint": "21.14×22.54m",
            "run_summary": "시작부터 종료까지 한 번에 추종",
            "source_route": tmp_path / "course.csv",
            "source_start_s_m": 191.3864,
            "source_length_m": 37.6734,
        }
    )
    assert variant_display_name(labeled).startswith("[S-FULL] S자 전체")
    assert "s 191.4~229.1m" in variant_compact_summary(labeled)
    details = variant_detail_lines(labeled)
    assert any("s=191.386~229.060 m" in line for line in details)
    assert any("FSM=S_CURVE" in line for line in details)

    relaxed = Variant(**{**variant.__dict__, "preflight": "relaxed"})
    relaxed_command = build_route_command(config, relaxed, 25)
    assert "relaxed_route_test_mode:=true" in relaxed_command
    assert "require_velocity_feedback:=false" in relaxed_command


def test_route_launch_includes_nonempty_optional_source_metadata(tmp_path):
    config, variant = _ready_fixture(tmp_path)
    labeled = Variant(
        **{
            **variant.__dict__,
            "source_variant_id": "p1_t1_f1",
            "fsm_label": "T_PARK",
        }
    )

    command = build_route_command(config, labeled, 30)

    assert "source_variant_id:=p1_t1_f1" in command
    assert "source_fsm_label:=T_PARK" in command


def test_s_detour_is_disabled_and_lidar_geometry_is_kept_for_automatic_parking():
    config_file = Path(__file__).resolve().parents[4] / "operations" / "mando_scenarios.yaml"
    config = load_config(config_file)
    for number in (3, 7, 9):
        scenario = next(item for item in config.scenarios if item.number == number)
        for variant in scenario.variants:
            command = build_route_command(config, variant, scenario.default_speed)
            detour_expected = variant.enable_lidar_detour
            geometry_expected = detour_expected or variant.enable_parking_selection
            if variant.mission_override == "S_OBSTACLE":
                assert not detour_expected
            assert f"lidar_geometry_verified:={'true' if geometry_expected else 'false'}" in command
            assert f"enable_local_detour_steering:={'true' if detour_expected else 'false'}" in command
            assert (
                f"enable_parking_selection:="
                f"{'true' if variant.enable_parking_selection else 'false'}"
            ) in command
            assert "use_local_detour_planner:=true" in command
            assert "start_lidar_driver:=true" in command


def test_competition_menu_keeps_full_race_and_campus_style_finger_drive_only():
    config_file = (
        Path(__file__).resolve().parents[4]
        / "operations"
        / "mando_competition_scenarios.yaml"
    )
    config = load_config(config_file)
    assert [item.number for item in config.scenarios] == [1, 2]
    full = next(item for item in config.scenarios if item.number == 1)
    finger = next(item for item in config.scenarios if item.number == 2)

    assert full.variants[0].enable_lidar_detour
    assert full.variants[0].default_hill_approach_pwm == 40
    assert full.variants[0].default_hill_hold_pwm == 10
    assert full.variants[0].default_hill_post_stop_pwm == 30
    assert finger.kind == "finger"
    assert finger.default_speed == 10
    assert finger.finger_initial_pwm == 10
    assert finger.maximum_pwm == 100

    full_command = build_route_command(
        config,
        full.variants[0],
        full.default_speed,
        full.variants[0].default_hill_hold_pwm,
    )
    assert "lidar_geometry_verified:=true" in full_command
    assert "enable_local_detour_steering:=true" in full_command
    assert "hill_hold_enabled:=true" in full_command
    assert "hill_hold_duty:=0.10" in full_command
    assert "hill_approach_pwm:=40" in full_command
    assert "hill_post_stop_pwm:=30" in full_command
def test_campus_parking_groups_default_to_automatic_selection():
    config_file = Path(__file__).resolve().parents[4] / "operations" / "mando_scenarios.yaml"
    config = load_config(config_file)
    t_parking = next(item for item in config.scenarios if item.number == 7)
    parallel = next(item for item in config.scenarios if item.number == 9)

    assert t_parking.variants[0].segment_code == "T-AUTO"
    assert t_parking.variants[0].enable_parking_selection
    assert parallel.variants[0].segment_code == "P-AUTO"
    assert parallel.variants[0].enable_parking_selection


def test_relaxed_readiness_does_not_require_calibration_or_alignment(tmp_path):
    config, variant = _ready_fixture(tmp_path)
    relaxed = Variant(
        **{
            **variant.__dict__,
            "preflight": "relaxed",
            "calibration_file": tmp_path / "missing_calibration.yaml",
            "alignment_confirmed": False,
            "minimum_length_m": 100.0,
            "maximum_length_m": 101.0,
        }
    )

    assert relaxed_readiness(config, relaxed).ok


def test_route_geometry_accepts_authored_fsm_and_enabled_reverse(tmp_path):
    config, base_variant = _ready_fixture(tmp_path)
    route = tmp_path / "reverse_parking.csv"
    route.write_text(
        "s_m,x_m,y_m,target_speed_mps,mission,direction\n"
        "0.0,0.0,0.0,0.20,PERP_PARK,1\n"
        "1.0,1.0,0.0,0.20,PERP_PARK,-1\n"
        "2.0,0.0,0.0,0.00,FINISH,-1\n",
        encoding="utf-8",
    )
    variant = Variant(
        **{
            **base_variant.__dict__,
            "route_file": route,
            "preflight": "relaxed",
            "allow_reverse": True,
            "minimum_length_m": 0.0,
            "maximum_length_m": float("inf"),
        }
    )

    loaded, errors = route_geometry(variant)

    assert loaded is not None
    assert errors == ()
    assert relaxed_readiness(config, variant).ok


def test_route_geometry_reports_reverse_only_when_variant_disables_it(tmp_path):
    _config, base_variant = _ready_fixture(tmp_path)
    route = tmp_path / "reverse.csv"
    route.write_text(
        "s_m,x_m,y_m,target_speed_mps,mission,direction\n"
        "0.0,0.0,0.0,0.20,NORMAL,-1\n"
        "1.0,-1.0,0.0,0.00,FINISH,-1\n",
        encoding="utf-8",
    )
    variant = Variant(
        **{
            **base_variant.__dict__,
            "route_file": route,
            "allow_reverse": False,
            "minimum_length_m": 0.0,
            "maximum_length_m": float("inf"),
        }
    )

    _loaded, errors = route_geometry(variant)

    assert len(errors) == 1
    assert "allow_reverse" in errors[0]


def test_static_readiness_validates_auto_source_before_output_exists(tmp_path):
    config, base_variant = _ready_fixture(tmp_path)
    source = tmp_path / "source_profile.csv"
    source.write_text(
        "s_m,x_m,y_m,target_speed_mps,mission,direction\n"
        "0.0,0.0,0.0,0.20,PERP_PARK,1\n"
        "1.0,1.0,0.0,0.20,PERP_PARK,-1\n"
        "2.0,0.0,0.0,0.20,PERP_PARK,1\n"
        "3.0,1.0,0.0,0.00,FINISH,1\n",
        encoding="utf-8",
    )
    variant = Variant(
        **{
            **base_variant.__dict__,
            "route_file": tmp_path / "not_generated_yet.csv",
            "calibration_file": tmp_path / "not_generated_yet.yaml",
            "source_route": source,
            "source_start_s_m": 0.0,
            "source_length_m": 3.0,
            "auto_place_at_current_pose": True,
            "preflight": "relaxed",
            "preserve_source_profile": True,
            "allow_reverse": True,
            "minimum_length_m": 0.0,
            "maximum_length_m": float("inf"),
        }
    )

    report = static_readiness(config, variant)

    assert report.ok
    assert not variant.route_file.exists()


def test_start_alignment_uses_position_and_vehicle_heading(tmp_path):
    _config, variant = _ready_fixture(tmp_path)
    from hl_ku_core.route import Route

    route = Route.load_csv(variant.route_file)
    pose = PoseWithCovarianceStamped()
    pose.pose.pose.position.x = 0.3
    pose.pose.pose.position.y = 0.4
    pose.pose.pose.orientation.w = math.cos(math.radians(5.0) / 2.0)
    pose.pose.pose.orientation.z = math.sin(math.radians(5.0) / 2.0)

    distance, heading = alignment_error(route, pose)

    assert distance == 0.4
    assert math.isclose(heading, 5.0)


def test_start_alignment_accepts_a_matching_mid_route_position(tmp_path):
    _config, variant = _ready_fixture(tmp_path)
    route = Route.load_csv(variant.route_file)
    pose = PoseWithCovarianceStamped()
    pose.pose.pose.position.x = 3.0
    pose.pose.pose.position.y = 0.2
    pose.pose.pose.orientation.w = 1.0

    distance, heading = alignment_error(route, pose)

    assert distance == pytest.approx(0.2)
    assert heading == pytest.approx(0.0)


def test_missing_straight_route_is_rigidly_placed_at_live_rtk_pose(tmp_path):
    config, base_variant = _ready_fixture(tmp_path)
    source = tmp_path / "routes" / "source.csv"
    source.write_text(
        "s_m,x_m,y_m,target_speed_mps,mission,direction\n"
        "0.0,0.0,0.0,0.30,NORMAL,1\n"
        "44.0,1.0,2.0,0.30,NORMAL,1\n"
        "48.0,1.0,6.0,0.30,NORMAL,1\n"
        "52.0,1.0,10.0,0.30,NORMAL,1\n"
        "53.0,1.0,11.0,0.00,FINISH,1\n",
        encoding="utf-8",
    )
    route_file = tmp_path / "routes" / "mando" / "konkuk_straight.csv"
    calibration_file = tmp_path / "config" / "mando" / "straight.yaml"
    variant = Variant(
        **{
            **base_variant.__dict__,
            "route_file": route_file,
            "calibration_file": calibration_file,
            "source_route": source,
            "source_start_s_m": 44.0,
            "source_length_m": 8.0,
            "auto_place_at_current_pose": True,
            "mission_override": "S_OBSTACLE",
        }
    )
    pose = PoseWithCovarianceStamped()
    pose.pose.pose.position.x = 12.0
    pose.pose.pose.position.y = -3.0
    pose.pose.pose.orientation.z = math.sin(math.radians(30.0) / 2.0)
    pose.pose.pose.orientation.w = math.cos(math.radians(30.0) / 2.0)

    assert automatic_placement_reason(config, variant) == "경로 파일 없음"
    route = place_variant_at_pose(config, variant, pose, 0.42)

    assert route_file.is_file()
    assert calibration_file.is_file()
    assert math.isclose(route.waypoints[0].x_m, 12.0)
    assert math.isclose(route.waypoints[0].y_m, -3.0)
    assert math.isclose(route.waypoints[-1].s_m, 8.0)
    assert math.isclose(route.waypoints[1].target_speed_mps, 0.42)
    assert route.waypoints[-1].mission == "FINISH"
    assert {waypoint.mission for waypoint in route.waypoints[:-1]} == {"S_OBSTACLE"}
    assert route.waypoints[-1].target_speed_mps == 0.0
    x_m, y_m, heading_deg = pose_xy_heading_deg(pose)
    assert (x_m, y_m) == (12.0, -3.0)
    assert math.isclose(heading_deg, 30.0)
    calibration = yaml.safe_load(calibration_file.read_text(encoding="utf-8"))
    assert calibration["course"]["route_calibrated"] is True
    assert calibration["course"]["placement"]["scale"] == 1.0
    assert calibration["course"]["placement"]["mission_override"] == "S_OBSTACLE"
    assert automatic_placement_reason(config, variant) is None


def test_gnss_gate_requires_fixed_fresh_complete_solution():
    status = GnssStatus()
    status.fix_type = GnssStatus.FIX_RTK_FIXED
    status.position_valid = True
    status.heading_valid = True
    status.velocity_valid = True
    status.nmea_checksum_valid = True
    status.satellites = 15
    status.hdop = 1.0
    status.correction_age_sec = 0.2

    assert gnss_blockers(status, True) == []
    status.heading_valid = False
    assert "헤딩 무효" in gnss_blockers(status, True)
    assert gnss_blockers(status, False) == ["GNSS 상태 미수신/시간초과"]


def test_relaxed_rtk_gate_does_not_block_on_quality_metrics():
    status = GnssStatus()
    status.fix_type = GnssStatus.FIX_RTK_FIXED
    status.position_valid = True
    status.heading_valid = True
    status.velocity_valid = False
    status.nmea_checksum_valid = False
    status.satellites = 1
    status.hdop = 99.0
    status.correction_age_sec = 99.0

    assert gnss_blockers(status, True) == []


def test_datum_command_and_apply_preserve_gps_comments(tmp_path):
    config, _variant = _ready_fixture(tmp_path)
    scenario = Scenario(
        5,
        "datum",
        "datum",
        120.0,
        survey_minimum_samples=50,
        survey_maximum_hdop=2.5,
    )
    output = tmp_path / "konkuk_datum.yaml"
    command = build_datum_command(config, scenario, 120.0, output)
    assert "survey_enabled:=true" in command
    assert "survey_duration_sec:=120.0" in command
    assert "survey_minimum_samples:=50" in command
    config.gps_config.write_text(
        "# keep this comment\n"
        "gnss_localizer:\n"
        "  ros__parameters:\n"
        "    datum_configured: false\n"
        "    datum_latitude_deg: 0.0\n"
        "    datum_longitude_deg: 0.0\n"
        "    datum_altitude_m: 0.0\n"
        "path_tracker:\n"
        "  ros__parameters:\n"
        "    wheelbase_m: 0.58\n",
        encoding="utf-8",
    )
    _write_yaml(
        output,
        {
            "gnss_localizer": {
                "ros__parameters": {
                    "datum_configured": True,
                    "datum_latitude_deg": 37.123,
                    "datum_longitude_deg": 127.456,
                    "datum_altitude_m": 25.75,
                }
            }
        },
    )

    backup = apply_datum_to_gps_config(output, config.gps_config)
    updated = config.gps_config.read_text(encoding="utf-8")

    assert backup.is_file()
    assert "# keep this comment" in updated
    assert "datum_configured: true" in updated
    assert "datum_latitude_deg: 37.123" in updated
    assert "wheelbase_m: 0.58" in updated


def test_ready_status_uses_a_visible_check_mark():
    assert status_mark("ok") == "[✓]"
    assert status_mark("wait") == "[대기]"
    assert status_mark("block") == "[차단]"
    assert status_mark("info") == "[정보]"


def test_korean_table_cells_have_stable_terminal_width():
    cell = fit_terminal_text("손가락 주행", 18, "center")
    row = table_row((" 번호", cell), (7, 18))

    assert terminal_text_width(cell) == 18
    assert terminal_text_width(row) == 28
    assert table_border((7, 18)) == "+-------+------------------+"


def test_key_labels_make_number_and_control_keys_visible():
    assert key_input_label(ord("4")) == "4"
    assert key_input_label(ord("v")) == "V"
    assert key_input_label(ord(" ")) == "SPACE"
    assert key_input_label(curses.KEY_LEFT) == "LEFT"


def test_console_remembers_scenario_and_value_numbers(tmp_path):
    config, _variant = _ready_fixture(tmp_path)
    console = MandoConsole(config, object(), auto_rtk=False)

    console.select_number(1)
    assert console.last_numeric_input == "1 (시나리오 번호)"

    console.set_speed(45)
    assert console.last_numeric_input == "45 (경로 구동 PWM)"
    assert console.speeds[1] == 45

    console.set_speed(120)
    assert console.last_numeric_input == "120 (경로 구동 PWM)"
    assert console.message_is_error
    assert console.speeds[1] == 45

    console.set_speed(20.5)
    assert console.message_is_error
    assert console.speeds[1] == 45


def test_draw_centers_the_status_tables_and_shows_latest_input(tmp_path):
    config, _variant = _ready_fixture(tmp_path)

    class Node:
        samples = {"gnss": Sample()}

        @staticmethod
        def node_names():
            return set()

    class Screen:
        def __init__(self):
            self.writes = []

        @staticmethod
        def getmaxyx():
            return 40, 180

        @staticmethod
        def erase():
            pass

        def addnstr(self, row, column, text, maximum, style):
            self.writes.append((row, column, text[:maximum], style))

        @staticmethod
        def refresh():
            pass

    console = MandoConsole(config, Node(), auto_rtk=False)
    console.last_key_input = "4"
    console.last_numeric_input = "60 (손가락 PWM)"
    screen = Screen()

    console.draw(screen)

    assert screen.writes[0][1] == 20
    assert any("HL MANDO FIELD CONSOLE" in item[2] for item in screen.writes)
    assert any("마지막 키     4" in item[2] for item in screen.writes)
    assert any("최근 숫자     60 (손가락 PWM)" in item[2] for item in screen.writes)
    assert any("현재 입력값" in item[2] for item in screen.writes)
    assert any("30/100 PWM" in item[2] for item in screen.writes)


def test_managed_ros_children_cannot_consume_tui_keys(tmp_path, monkeypatch):
    captured = {}

    class Child:
        def poll(self):
            return 0

    def fake_popen(command, **kwargs):
        captured.update(kwargs)
        return Child()

    monkeypatch.setattr(subprocess, "Popen", fake_popen)
    slot = ProcessSlot()
    slot.start(["true"], "test", tmp_path)
    slot.stop()

    assert captured["stdin"] is subprocess.DEVNULL


def test_manual_recording_does_not_duplicate_an_external_recorder(tmp_path):
    config, _variant = _ready_fixture(tmp_path)

    class Node:
        @staticmethod
        def node_names():
            return {"rosbag2_recorder"}

    console = MandoConsole(config, Node(), auto_rtk=False)
    console.start_recording()

    assert console.recorder.child is None
    assert "중복 녹화" in console.message
    assert console.message_is_error


def test_finger_drive_runs_without_gnss_camera_or_mcap_readiness(
    tmp_path, monkeypatch
):
    config, _variant = _ready_fixture(tmp_path)
    finger = Scenario(4, "finger", "finger", 60.0, maximum_pwm=100)
    config = ConsoleConfig(
        **{**config.__dict__, "scenarios": (finger,)}
    )

    class Node:
        @staticmethod
        def node_names():
            return set()

    class Screen:
        @staticmethod
        def refresh():
            pass

    console = MandoConsole(config, Node(), auto_rtk=False)
    def unexpected_recording():
        raise AssertionError("R must not start MCAP")

    monkeypatch.setattr(console, "start_recording", unexpected_recording)
    monkeypatch.setattr("curses.def_prog_mode", lambda: None)
    monkeypatch.setattr("curses.endwin", lambda: None)
    monkeypatch.setattr("curses.reset_prog_mode", lambda: None)
    launched = {}

    def fake_run(command, check):
        launched["command"] = command
        launched["check"] = check
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(subprocess, "run", fake_run)

    console.run_finger(Screen())

    assert launched["command"] == [
        str(config.finger_drive_script),
        "--max-pwm",
        "100",
        "--initial-pwm",
        "60",
        "--step-pwm",
        "60",
    ]
    assert not launched["check"]
