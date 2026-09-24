from pathlib import Path

import yaml

from hl_ku_core.readiness import (
    check_autonomous_readiness,
    check_gps_only_route_test_readiness,
    check_global_route_test_readiness,
)


PACKAGE = Path(__file__).parents[1]


def _write_valid_files(tmp_path):
    system = yaml.safe_load((PACKAGE / "config/system.yaml").read_text(encoding="utf-8"))
    system["gnss_localizer"]["ros__parameters"]["datum_configured"] = True

    record = yaml.safe_load(
        (PACKAGE / "config/calibration_record.yaml").read_text(encoding="utf-8")
    )
    record["project"].update(
        vehicle_id="test_vehicle", measured_at="2026-09-02", operators=["tester"]
    )
    record["power"].update(
        fully_charged_voltage_v=23.0,
        competition_voltage_rule_confirmed=True,
        main_fuse_a=20.0,
        physical_estop_verified=True,
    )
    record["vehicle_geometry"].update(
        wheelbase_m=0.58,
        body_width_m=0.70,
        maximum_left_steering_rad=-0.48,
        maximum_right_steering_rad=0.48,
    )
    record["gnss"].update(
        baseline_m=1.0,
        datum_latitude_deg=0.0,
        datum_longitude_deg=0.0,
        datum_altitude_m=0.0,
        base_to_ant1_x_m=0.0,
        base_to_ant1_y_m=0.0,
        heading_mount_offset_deg=0.0,
        rtk_fixed_ratio=0.98,
        heading_valid_ratio=0.99,
    )
    record["camera"].update(
        intrinsic_calibration_file="camera.yaml",
        ground_homography_file="homography.yaml",
        flat_lane_verified=True,
        uphill_lane_verified=True,
        traffic_light_verified=True,
        end_lane_signal_verified=True,
    )
    record["lidar"].update(
        angle_zero_is_vehicle_forward=True,
        positive_angle_is_vehicle_left=True,
        t870_cluster_verified=True,
        child_dummy_verified=True,
    )
    record["steering"].update(
        adc_left=800,
        adc_center=2000,
        adc_right=3200,
        pwm_limit=0.6,
    )
    controller = system["vehicle_controller"]["ros__parameters"]
    record["traction"].update(
        drive_duty_limit=0.75,
        forward_speeds_mps=controller["forward_speeds_mps"],
        forward_duties=controller["forward_duties"],
        reverse_speeds_mps=controller["reverse_speeds_mps"],
        reverse_duties=controller["reverse_duties"],
        dry_braking_distance_m={"1.0_mps": 0.5},
    )
    record["course"].update(route_file="course_measured.csv", route_calibrated=True)
    for name in (
        "wheels_up_test",
        "estop_test",
        "udp_watchdog_test",
        "steering_limit_test",
        "rtk_dropout_test",
        "camera_dropout_test",
        "lidar_emergency_test",
    ):
        record["acceptance"][name] = True

    system_path = tmp_path / "system.yaml"
    record_path = tmp_path / "calibration.yaml"
    route_path = tmp_path / "course_measured.csv"
    system_path.write_text(yaml.safe_dump(system), encoding="utf-8")
    record_path.write_text(yaml.safe_dump(record), encoding="utf-8")
    route_path.write_text(
        "s_m,x_m,y_m,target_speed_mps,mission,direction\n"
        "0,0,0,0.5,NORMAL,1\n"
        "1,1,0,0.0,FINISH,1\n",
        encoding="utf-8",
    )
    return system_path, record_path, route_path


def test_repository_templates_cannot_authorize_driving():
    report = check_autonomous_readiness(
        PACKAGE / "config/system.yaml",
        PACKAGE / "config/calibration_record.yaml",
        PACKAGE / "routes/course_template.csv",
    )
    assert not report.ok
    assert "course_template.csv" in report.summary()
    assert "project.measured_at is empty" in report.summary()


def test_measured_and_verified_configuration_passes(tmp_path):
    report = check_autonomous_readiness(*_write_valid_files(tmp_path))
    assert report.ok, report.summary()
    assert report.warnings


def _write_valid_route_test_files(tmp_path):
    system = yaml.safe_load((PACKAGE / "config/system.yaml").read_text(encoding="utf-8"))
    system["gnss_localizer"]["ros__parameters"].update(
        datum_configured=True,
        datum_latitude_deg=37.0,
        datum_longitude_deg=127.0,
        datum_altitude_m=50.0,
        base_to_ant1_x_m=0.40,
        base_to_ant1_y_m=0.0,
        heading_mount_offset_deg=0.0,
    )
    record = yaml.safe_load(
        (PACKAGE / "config/calibration_record.yaml").read_text(encoding="utf-8")
    )
    record["project"].update(
        vehicle_id="route_test_vehicle",
        measured_at="2026-09-02",
        operators=["tester"],
    )
    record["power"].update(
        fully_charged_voltage_v=23.0,
        competition_voltage_rule_confirmed=True,
        main_fuse_a=20.0,
        physical_estop_verified=True,
    )
    record["vehicle_geometry"].update(
        wheelbase_m=0.58,
        body_width_m=0.70,
        maximum_left_steering_rad=-0.48,
        maximum_right_steering_rad=0.48,
    )
    record["gnss"].update(
        baseline_m=1.0,
        datum_latitude_deg=37.0,
        datum_longitude_deg=127.0,
        datum_altitude_m=50.0,
        base_to_ant1_x_m=0.40,
        base_to_ant1_y_m=0.0,
        heading_mount_offset_deg=0.0,
    )
    record["lidar"].update(
        angle_zero_is_vehicle_forward=True,
        positive_angle_is_vehicle_left=True,
    )
    record["traction"].update(
        route_test_drive_command_limit=12,
        low_speed_braking_distance_m=0.50,
    )
    record["course"].update(
        route_file="route_test_measured.csv",
        route_calibrated=True,
    )
    for name in (
        "wheels_up_test",
        "estop_test",
        "serial_watchdog_test",
        "steering_limit_test",
        "low_speed_straight_test",
        "rtk_dropout_test",
        "lidar_emergency_test",
    ):
        record["acceptance"][name] = True

    system_path = tmp_path / "system.yaml"
    override_path = tmp_path / "route_test.yaml"
    record_path = tmp_path / "calibration.yaml"
    route_path = tmp_path / "route_test_measured.csv"
    system_path.write_text(yaml.safe_dump(system), encoding="utf-8")
    override_path.write_text(
        (PACKAGE / "config/route_test.yaml").read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    record_path.write_text(yaml.safe_dump(record), encoding="utf-8")
    route_path.write_text(
        "s_m,x_m,y_m,target_speed_mps,mission,direction\n"
        "0,0,0,0.25,NORMAL,1\n"
        "1,1,0,0.0,FINISH,1\n",
        encoding="utf-8",
    )
    return system_path, override_path, record_path, route_path


def test_low_speed_global_route_profile_passes_when_physically_verified(tmp_path):
    report = check_global_route_test_readiness(
        *_write_valid_route_test_files(tmp_path)
    )
    assert report.ok, report.summary()
    assert "camera lane correction is disabled" in report.summary()


def test_low_speed_route_profile_rejects_drive_limit_above_t870_trial_limit(tmp_path):
    system_path, override_path, record_path, route_path = _write_valid_route_test_files(
        tmp_path
    )
    override = yaml.safe_load(override_path.read_text(encoding="utf-8"))
    override["vehicle_controller"]["ros__parameters"]["maximum_duty"] = 0.31
    override_path.write_text(yaml.safe_dump(override), encoding="utf-8")
    report = check_global_route_test_readiness(
        system_path, override_path, record_path, route_path
    )
    assert not report.ok
    assert "maximum_duty must be <= 0.30" in report.summary()


def test_single_antenna_gps_route_accepts_route_seeded_heading(tmp_path):
    system_path, _override_path, record_path, route_path = (
        _write_valid_route_test_files(tmp_path)
    )
    gps_profile = yaml.safe_load(
        (PACKAGE / "config/gps_only_route.yaml").read_text(encoding="utf-8")
    )
    gps_profile["gnss_localizer"]["ros__parameters"].update(
        datum_latitude_deg=37.0,
        datum_longitude_deg=127.0,
        datum_altitude_m=50.0,
        base_to_ant1_x_m=0.40,
        heading_mount_offset_deg=0.0,
        allow_single_antenna_heading=True,
        single_antenna_initial_heading_deg=90.0,
        single_antenna_minimum_track_speed_mps=0.15,
        single_antenna_track_filter_gain=0.25,
    )
    gps_profile["safety_supervisor"]["ros__parameters"][
        "require_gnss_heading"
    ] = False
    gps_path = tmp_path / "gps_only.yaml"
    gps_path.write_text(yaml.safe_dump(gps_profile), encoding="utf-8")

    record = yaml.safe_load(record_path.read_text(encoding="utf-8"))
    record["gnss"]["baseline_m"] = 0.0
    record["traction"]["route_test_drive_command_limit"] = 30
    record["acceptance"]["lidar_emergency_test"] = False
    record_path.write_text(yaml.safe_dump(record), encoding="utf-8")

    ntrip_path = tmp_path / "ntrip.yaml"
    ntrip_path.write_text(
        yaml.safe_dump(
            {
                "ntrip_client": {
                    "ros__parameters": {
                        "enabled": True,
                        "host": "caster.provider.test",
                        "mountpoint": "VRS-RTCM31",
                        "username": "test-user",
                        "password": "test-password",
                    }
                }
            }
        ),
        encoding="utf-8",
    )

    report = check_gps_only_route_test_readiness(
        system_path, gps_path, ntrip_path, record_path, route_path
    )

    assert report.ok, report.summary()
    assert "single-antenna heading" in report.summary()
