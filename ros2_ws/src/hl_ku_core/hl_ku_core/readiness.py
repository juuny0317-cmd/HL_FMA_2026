"""Static commissioning checks used before autonomous actuation is allowed."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import yaml

from .route import Route


ROUTE_TEST_MAXIMUM_DRIVE_DUTY = 0.30
ROUTE_TEST_MAXIMUM_DRIVE_COMMAND = 30
ROUTE_TEST_MAXIMUM_FORWARD_SPEED_MPS = 1.00


@dataclass(frozen=True)
class ReadinessReport:
    errors: tuple[str, ...]
    warnings: tuple[str, ...]

    @property
    def ok(self) -> bool:
        return not self.errors

    def summary(self) -> str:
        if self.ok:
            return "OK" if not self.warnings else "OK; warnings=" + " | ".join(self.warnings)
        return "NOT_READY: " + " | ".join(self.errors)


def _mapping(value: Any) -> Mapping[str, Any]:
    return value if isinstance(value, Mapping) else {}


def _params(system: Mapping[str, Any], node: str) -> Mapping[str, Any]:
    return _mapping(_mapping(system.get(node)).get("ros__parameters"))


def _finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _positive(value: Any) -> bool:
    return _finite(value) and float(value) > 0.0


def _strictly_increasing(values: Sequence[Any]) -> bool:
    return bool(values) and all(
        _finite(value) and float(value) > 0.0 for value in values
    ) and all(float(left) < float(right) for left, right in zip(values, values[1:]))


def _matching_table(speeds: Any, duties: Any) -> bool:
    if not isinstance(speeds, Sequence) or isinstance(speeds, (str, bytes)):
        return False
    if not isinstance(duties, Sequence) or isinstance(duties, (str, bytes)):
        return False
    return (
        len(speeds) == len(duties)
        and _strictly_increasing(speeds)
        and all(_finite(value) and 0.0 < float(value) <= 1.0 for value in duties)
    )


def _close(left: Any, right: Any, tolerance: float) -> bool:
    return _finite(left) and _finite(right) and abs(float(left) - float(right)) <= tolerance


def _sequences_close(left: Any, right: Any, tolerance: float = 1.0e-6) -> bool:
    if not isinstance(left, Sequence) or isinstance(left, (str, bytes)):
        return False
    if not isinstance(right, Sequence) or isinstance(right, (str, bytes)):
        return False
    return len(left) == len(right) and all(
        _close(left_value, right_value, tolerance)
        for left_value, right_value in zip(left, right)
    )


def _load_yaml(path: Path) -> Mapping[str, Any]:
    with path.open(encoding="utf-8") as stream:
        return _mapping(yaml.safe_load(stream))


def _deep_merge(left: Mapping[str, Any], right: Mapping[str, Any]) -> dict[str, Any]:
    result: dict[str, Any] = dict(left)
    for key, value in right.items():
        current = result.get(key)
        if isinstance(current, Mapping) and isinstance(value, Mapping):
            result[key] = _deep_merge(current, value)
        else:
            result[key] = value
    return result


def _require_true(
    errors: list[str], mapping: Mapping[str, Any], names: Iterable[str], prefix: str
) -> None:
    for name in names:
        if mapping.get(name) is not True:
            errors.append(f"{prefix}.{name}=true verification required")


def check_autonomous_readiness(
    system_config_file: str | Path,
    calibration_file: str | Path,
    route_file: str | Path,
) -> ReadinessReport:
    """Validate measured configuration without ever guessing hardware values."""

    errors: list[str] = []
    warnings: list[str] = []
    system_path = Path(system_config_file).expanduser()
    calibration_path = Path(calibration_file).expanduser()
    route_path = Path(route_file).expanduser()

    documents: dict[str, Mapping[str, Any]] = {}
    for name, path in (
        ("system config", system_path),
        ("calibration record", calibration_path),
    ):
        if not path.is_file():
            errors.append(f"{name} missing: {path}")
            continue
        try:
            documents[name] = _load_yaml(path)
        except (OSError, yaml.YAMLError) as error:
            errors.append(f"{name} unreadable: {error}")

    if not route_path.is_file():
        errors.append(f"route file missing: {route_path}")
        route = None
    else:
        try:
            route = Route.load_csv(route_path)
        except (OSError, ValueError, KeyError) as error:
            errors.append(f"route invalid: {error}")
            route = None
    route_missions = {waypoint.mission for waypoint in route.waypoints} if route else set()

    system = documents.get("system config")
    record = documents.get("calibration record")
    if system is None or record is None:
        return ReadinessReport(tuple(errors), tuple(warnings))

    localizer = _params(system, "gnss_localizer")
    tracker = _params(system, "path_tracker")
    controller = _params(system, "vehicle_controller")
    safety = _params(system, "safety_supervisor")
    lidar_config = _params(system, "lidar_perception")

    if localizer.get("datum_configured") is not True:
        errors.append("gnss_localizer.datum_configured=true required")
    latitude = localizer.get("datum_latitude_deg")
    longitude = localizer.get("datum_longitude_deg")
    altitude = localizer.get("datum_altitude_m")
    if not (_finite(latitude) and -90.0 <= float(latitude) <= 90.0):
        errors.append("gnss datum latitude is invalid")
    if not (_finite(longitude) and -180.0 <= float(longitude) <= 180.0):
        errors.append("gnss datum longitude is invalid")
    if not _finite(altitude):
        errors.append("gnss datum altitude is invalid")

    wheelbase = tracker.get("wheelbase_m")
    maximum_steering = tracker.get("maximum_steering_rad")
    maximum_duty = controller.get("maximum_duty")
    if not (_finite(wheelbase) and 0.1 <= float(wheelbase) <= 5.0):
        errors.append("path_tracker.wheelbase_m is not a plausible measured value")
    if not (_finite(maximum_steering) and 0.02 <= float(maximum_steering) <= 1.2):
        errors.append("path_tracker.maximum_steering_rad is invalid")
    if not (_finite(maximum_duty) and 0.0 < float(maximum_duty) <= 1.0):
        errors.append("vehicle_controller.maximum_duty must be in (0, 1]")
    if not _matching_table(
        controller.get("forward_speeds_mps"), controller.get("forward_duties")
    ):
        errors.append("forward speed/duty table is invalid")
    if not _matching_table(
        controller.get("reverse_speeds_mps"), controller.get("reverse_duties")
    ):
        errors.append("reverse speed/duty table is invalid")
    if safety.get("require_preflight") is not True:
        errors.append("safety_supervisor.require_preflight must remain true")
    safety_duty = safety.get("autonomous_maximum_drive_duty")
    safety_steering = safety.get("autonomous_maximum_steering_rad")
    if not (_positive(safety_duty) and _finite(maximum_duty) and float(safety_duty) <= float(maximum_duty)):
        errors.append("autonomous safety duty limit must not exceed controller maximum_duty")
    if not (
        _positive(safety_steering)
        and _finite(maximum_steering)
        and float(safety_steering) <= float(maximum_steering)
    ):
        errors.append("autonomous safety steering limit must not exceed path tracker limit")

    project = _mapping(record.get("project"))
    power = _mapping(record.get("power"))
    geometry = _mapping(record.get("vehicle_geometry"))
    gnss = _mapping(record.get("gnss"))
    camera = _mapping(record.get("camera"))
    lidar = _mapping(record.get("lidar"))
    steering = _mapping(record.get("steering"))
    traction = _mapping(record.get("traction"))
    course = _mapping(record.get("course"))
    acceptance = _mapping(record.get("acceptance"))

    if not str(project.get("vehicle_id", "")).strip():
        errors.append("project.vehicle_id is empty")
    if not str(project.get("measured_at", "")).strip():
        errors.append("project.measured_at is empty")
    operators = project.get("operators")
    if not isinstance(operators, list) or not any(str(value).strip() for value in operators):
        errors.append("project.operators must identify a calibration operator")

    _require_true(
        errors,
        power,
        ("competition_voltage_rule_confirmed", "physical_estop_verified"),
        "power",
    )
    if not _positive(power.get("main_fuse_a")):
        errors.append("power.main_fuse_a must be recorded")
    charged_voltage = power.get("fully_charged_voltage_v")
    if not (_finite(charged_voltage) and 1.0 < float(charged_voltage) <= 24.0):
        errors.append("power.fully_charged_voltage_v must be measured and <= 24 V")

    if not _close(wheelbase, geometry.get("wheelbase_m"), 0.005):
        errors.append("system wheelbase does not match calibration record")
    body_width = geometry.get("body_width_m")
    half_width = lidar_config.get("vehicle_half_width_m")
    if not (_positive(body_width) and _positive(half_width) and float(half_width) >= float(body_width) * 0.5):
        errors.append("LiDAR vehicle_half_width_m is smaller than measured body half-width")
    left_limit = geometry.get("maximum_left_steering_rad")
    right_limit = geometry.get("maximum_right_steering_rad")
    if not (_finite(left_limit) and float(left_limit) < 0.0):
        errors.append("vehicle_geometry.maximum_left_steering_rad must be negative")
    if not (_finite(right_limit) and float(right_limit) > 0.0):
        errors.append("vehicle_geometry.maximum_right_steering_rad must be positive")
    if _finite(maximum_steering) and _finite(left_limit) and _finite(right_limit):
        measured_limit = min(abs(float(left_limit)), abs(float(right_limit)))
        if float(maximum_steering) > measured_limit:
            errors.append("configured steering limit exceeds measured mechanical-safe limit")

    if not _positive(gnss.get("baseline_m")):
        errors.append("gnss.baseline_m must be measured")
    for config_name, record_name in (
        ("datum_latitude_deg", "datum_latitude_deg"),
        ("datum_longitude_deg", "datum_longitude_deg"),
        ("datum_altitude_m", "datum_altitude_m"),
        ("base_to_ant1_x_m", "base_to_ant1_x_m"),
        ("base_to_ant1_y_m", "base_to_ant1_y_m"),
        ("heading_mount_offset_deg", "heading_mount_offset_deg"),
    ):
        tolerance = 1.0e-7 if "latitude" in config_name or "longitude" in config_name else 0.005
        if not _close(localizer.get(config_name), gnss.get(record_name), tolerance):
            errors.append(f"gnss_localizer.{config_name} does not match calibration record")
    if not (_finite(gnss.get("rtk_fixed_ratio")) and float(gnss.get("rtk_fixed_ratio")) >= 0.90):
        errors.append("gnss.rtk_fixed_ratio must be measured at >= 0.90")
    if not (_finite(gnss.get("heading_valid_ratio")) and float(gnss.get("heading_valid_ratio")) >= 0.90):
        errors.append("gnss.heading_valid_ratio must be measured at >= 0.90")

    camera_checks = ["flat_lane_verified"]
    if "HILL" in route_missions:
        camera_checks.append("uphill_lane_verified")
    if "TRAFFIC" in route_missions:
        camera_checks.append("traffic_light_verified")
    if "END_LANE" in route_missions:
        camera_checks.append("end_lane_signal_verified")
    _require_true(errors, camera, camera_checks, "camera")
    if not str(camera.get("intrinsic_calibration_file", "")).strip():
        errors.append("camera.intrinsic_calibration_file is empty")
    if not str(camera.get("ground_homography_file", "")).strip():
        errors.append("camera.ground_homography_file is empty")
    lidar_checks = ["angle_zero_is_vehicle_forward", "positive_angle_is_vehicle_left"]
    if "S_OBSTACLE" in route_missions:
        lidar_checks.append("t870_cluster_verified")
    if "DUMMY" in route_missions:
        lidar_checks.append("child_dummy_verified")
    _require_true(errors, lidar, lidar_checks, "lidar")

    adc_left = steering.get("adc_left")
    adc_center = steering.get("adc_center")
    adc_right = steering.get("adc_right")
    if not all(isinstance(value, int) and not isinstance(value, bool) for value in (adc_left, adc_center, adc_right)):
        errors.append("steering ADC calibration must contain integer left/center/right values")
    elif not (min(adc_left, adc_right) < adc_center < max(adc_left, adc_right)):
        errors.append("steering.adc_center must lie between left and right ADC values")
    if not (_positive(steering.get("pwm_limit")) and float(steering.get("pwm_limit")) <= 1.0):
        errors.append("steering.pwm_limit must be in (0, 1]")

    traction_limit = traction.get("drive_duty_limit")
    if not (_positive(traction_limit) and float(traction_limit) <= 1.0):
        errors.append("traction.drive_duty_limit must be in (0, 1]")
    elif _finite(maximum_duty) and float(maximum_duty) > float(traction_limit):
        errors.append("controller maximum_duty exceeds calibrated traction limit")
    if not _matching_table(traction.get("forward_speeds_mps"), traction.get("forward_duties")):
        errors.append("calibration forward speed/duty table is incomplete")
    if not _matching_table(traction.get("reverse_speeds_mps"), traction.get("reverse_duties")):
        errors.append("calibration reverse speed/duty table is incomplete")
    for system_name, record_name in (
        ("forward_speeds_mps", "forward_speeds_mps"),
        ("forward_duties", "forward_duties"),
        ("reverse_speeds_mps", "reverse_speeds_mps"),
        ("reverse_duties", "reverse_duties"),
    ):
        if not _sequences_close(controller.get(system_name), traction.get(record_name)):
            errors.append(f"vehicle_controller.{system_name} does not match calibration record")
    braking_distances = traction.get("dry_braking_distance_m")
    if not isinstance(braking_distances, Mapping) or not braking_distances:
        errors.append("traction.dry_braking_distance_m must contain measured results")
    else:
        distances = [value for value in braking_distances.values() if _positive(value)]
        if len(distances) != len(braking_distances):
            errors.append("traction.dry_braking_distance_m contains invalid values")
        elif _positive(safety.get("emergency_distance_m")) and max(
            float(value) for value in distances
        ) > float(safety.get("emergency_distance_m")):
            errors.append("safety emergency distance is shorter than measured braking distance")

    if course.get("route_calibrated") is not True:
        errors.append("course.route_calibrated=true verification required")
    recorded_route = str(course.get("route_file", "")).strip()
    if not recorded_route:
        errors.append("course.route_file is empty")
    elif Path(recorded_route).name != route_path.name:
        errors.append("launch route file does not match calibration record")
    if route_path.name == "course_template.csv":
        errors.append("course_template.csv is an example and cannot authorize driving")

    _require_true(
        errors,
        acceptance,
        (
            "wheels_up_test",
            "estop_test",
            "udp_watchdog_test",
            "steering_limit_test",
            "rtk_dropout_test",
            "camera_dropout_test",
            "lidar_emergency_test",
        ),
        "acceptance",
    )
    for optional in ("dry_full_course_test", "wet_test"):
        if acceptance.get(optional) is not True:
            warnings.append(f"acceptance.{optional} not yet verified")

    if route is not None:
        previous_s = -math.inf
        forward_limit = controller.get("maximum_forward_speed_mps")
        reverse_limit = controller.get("maximum_reverse_speed_mps")
        for waypoint in route.waypoints:
            values = (waypoint.s_m, waypoint.x_m, waypoint.y_m, waypoint.target_speed_mps)
            if not all(math.isfinite(value) for value in values):
                errors.append(f"route row {waypoint.index + 2} contains a non-finite value")
                break
            if waypoint.s_m <= previous_s:
                errors.append("route s_m must be strictly increasing")
                break
            previous_s = waypoint.s_m
            speed_limit = forward_limit if waypoint.direction > 0 else reverse_limit
            if _positive(speed_limit) and waypoint.target_speed_mps > float(speed_limit):
                errors.append(f"route row {waypoint.index + 2} exceeds configured direction speed limit")
                break
        if route.waypoints[-1].mission != "FINISH":
            errors.append("final route waypoint must use the FINISH mission tag")

    return ReadinessReport(tuple(dict.fromkeys(errors)), tuple(dict.fromkeys(warnings)))


def check_global_route_test_readiness(
    system_config_file: str | Path,
    override_config_file: str | Path,
    calibration_file: str | Path,
    route_file: str | Path,
    *,
    gps_only: bool = False,
    additional_config_file: str | Path | None = None,
) -> ReadinessReport:
    """Check the deliberately bounded first global-route driving profile.

    This profile does not authorize competition missions or camera correction.
    It does require measured GNSS/geometry, the current NUCLEO steering map,
    low-speed braking, a physical E-stop, watchdog tests and LiDAR stopping.
    """

    errors: list[str] = []
    warnings: list[str] = [
        "global_route_test uses NUCLEO ACKs, not measured battery/steering telemetry",
    ]
    if gps_only:
        warnings.append(
            "GPS-only route mode has no camera/LiDAR obstacle detection; use a closed course"
        )
    else:
        warnings.append("camera lane correction is disabled in the first route test")
    paths = {
        "system config": Path(system_config_file).expanduser(),
        "route-test override": Path(override_config_file).expanduser(),
        "calibration record": Path(calibration_file).expanduser(),
    }
    if additional_config_file is not None:
        paths["additional config"] = Path(additional_config_file).expanduser()
    documents: dict[str, Mapping[str, Any]] = {}
    for name, path in paths.items():
        if not path.is_file():
            errors.append(f"{name} missing: {path}")
            continue
        try:
            documents[name] = _load_yaml(path)
        except (OSError, yaml.YAMLError) as error:
            errors.append(f"{name} unreadable: {error}")

    route_path = Path(route_file).expanduser()
    if not route_path.is_file():
        errors.append(f"route file missing: {route_path}")
        route = None
    else:
        try:
            route = Route.load_csv(route_path)
        except (OSError, ValueError, KeyError) as error:
            errors.append(f"route invalid: {error}")
            route = None

    base = documents.get("system config")
    override = documents.get("route-test override")
    record = documents.get("calibration record")
    if base is None or override is None or record is None:
        return ReadinessReport(tuple(dict.fromkeys(errors)), tuple(warnings))
    system = _deep_merge(base, override)
    additional = documents.get("additional config")
    if additional_config_file is not None:
        if additional is None:
            return ReadinessReport(tuple(dict.fromkeys(errors)), tuple(warnings))
        system = _deep_merge(system, additional)

    localizer = _params(system, "gnss_localizer")
    tracker = _params(system, "path_tracker")
    controller = _params(system, "vehicle_controller")
    safety = _params(system, "safety_supervisor")
    lidar_config = _params(system, "lidar_perception")
    bridge = _params(system, "nucleo_serial_bridge")
    ntrip = _params(system, "ntrip_client")
    single_antenna = localizer.get("allow_single_antenna_heading") is True

    if localizer.get("datum_configured") is not True:
        errors.append("gnss_localizer.datum_configured=true required")
    latitude = localizer.get("datum_latitude_deg")
    longitude = localizer.get("datum_longitude_deg")
    altitude = localizer.get("datum_altitude_m")
    if not (_finite(latitude) and -90.0 <= float(latitude) <= 90.0):
        errors.append("gnss datum latitude is invalid")
    if not (_finite(longitude) and -180.0 <= float(longitude) <= 180.0):
        errors.append("gnss datum longitude is invalid")
    if not _finite(altitude):
        errors.append("gnss datum altitude is invalid")

    wheelbase = tracker.get("wheelbase_m")
    maximum_steering = tracker.get("maximum_steering_rad")
    maximum_duty = controller.get("maximum_duty")
    maximum_speed = controller.get("maximum_forward_speed_mps")
    if not (_finite(wheelbase) and 0.1 <= float(wheelbase) <= 5.0):
        errors.append("path_tracker.wheelbase_m is not a plausible measured value")
    if not (_finite(maximum_steering) and 0.02 <= float(maximum_steering) <= 1.2):
        errors.append("path_tracker.maximum_steering_rad is invalid")
    if not (
        _positive(maximum_duty)
        and float(maximum_duty) <= ROUTE_TEST_MAXIMUM_DRIVE_DUTY
    ):
        errors.append("global route test maximum_duty must be <= 0.30")
    if not (
        _positive(maximum_speed)
        and float(maximum_speed) <= ROUTE_TEST_MAXIMUM_FORWARD_SPEED_MPS
    ):
        errors.append("global route test forward speed must be <= 1.00 m/s")
    if tracker.get("enable_lane_correction") is not False:
        errors.append("first global route test must disable camera lane correction")
    if not _matching_table(
        controller.get("forward_speeds_mps"), controller.get("forward_duties")
    ):
        errors.append("route-test forward speed/duty table is invalid")
    elif _finite(maximum_duty) and max(
        float(value) for value in controller.get("forward_duties", [])
    ) > float(maximum_duty):
        errors.append("route-test forward table exceeds controller maximum_duty")
    if not _close(controller.get("kp"), 0.0, 1.0e-12) or not _close(
        controller.get("ki"), 0.0, 1.0e-12
    ):
        errors.append("first global route test requires kp=ki=0")

    if safety.get("require_preflight") is not True:
        errors.append("safety_supervisor.require_preflight must remain true")
    if safety.get("require_rtk_fixed") is not True:
        errors.append("global route test must require RTK fixed")
    if gps_only:
        if safety.get("require_perception") is not False:
            errors.append("GPS-only route mode must set require_perception=false")
        if safety.get("require_camera") is not False or safety.get("require_lidar") is not False:
            errors.append("GPS-only route mode must disable camera and LiDAR gates")
        if ntrip.get("enabled") is not True:
            errors.append("ntrip_client.enabled=true required for the RTK route profile")
        for name in ("host", "mountpoint", "username", "password"):
            value = str(ntrip.get(name, "")).strip()
            if not value or value in {
                "caster.example.com",
                "MOUNTPOINT",
                "USERNAME",
                "PASSWORD",
            }:
                errors.append(f"ntrip_client.{name} must contain the real provider value")
        if single_antenna:
            if safety.get("require_gnss_heading") is not False:
                errors.append(
                    "single-antenna route mode must set require_gnss_heading=false"
                )
            initial_heading = localizer.get("single_antenna_initial_heading_deg")
            if not (
                _finite(initial_heading)
                and 0.0 <= float(initial_heading) < 360.0
            ):
                errors.append("single-antenna initial heading is invalid")
            minimum_track_speed = localizer.get(
                "single_antenna_minimum_track_speed_mps"
            )
            if not (
                _positive(minimum_track_speed)
                and _finite(maximum_speed)
                and float(minimum_track_speed) < float(maximum_speed)
            ):
                errors.append(
                    "single-antenna track-heading speed must be below route speed"
                )
            filter_gain = localizer.get("single_antenna_track_filter_gain")
            if not (
                _positive(filter_gain) and float(filter_gain) <= 1.0
            ):
                errors.append("single-antenna track-heading filter gain is invalid")
            warnings.append(
                "single-antenna heading is seeded from route direction and needs "
                "correct physical start alignment"
            )
        elif safety.get("require_gnss_heading") is not True:
            errors.append(
                "GNSS heading may only be disabled with explicit single-antenna mode"
            )
    else:
        if single_antenna:
            errors.append("single-antenna heading is permitted only in GPS-only mode")
        if safety.get("require_gnss_heading") is not True:
            errors.append("global route test must require dual-antenna heading")
        if safety.get("require_perception") is not True or safety.get("require_lidar") is not True:
            errors.append("global route test must require fresh LiDAR perception")
        if safety.get("require_camera") is not False:
            errors.append("first global route test must leave camera correction disabled")
    safety_duty = safety.get("autonomous_maximum_drive_duty")
    safety_steering = safety.get("autonomous_maximum_steering_rad")
    if not (
        _positive(safety_duty)
        and _finite(maximum_duty)
        and float(safety_duty) <= float(maximum_duty)
    ):
        errors.append("route-test safety duty limit exceeds controller limit")
    if not (
        _positive(safety_steering)
        and _finite(maximum_steering)
        and float(safety_steering) <= float(maximum_steering)
    ):
        errors.append("route-test safety steering limit exceeds tracker limit")

    bridge_port = str(bridge.get("port", "")).strip()
    if not bridge_port:
        errors.append("nucleo_serial_bridge.port is empty")
    if bridge.get("allow_reverse") is not False:
        errors.append("first global route test must set allow_reverse=false")
    bridge_drive_limit = bridge.get("maximum_drive_command")
    if not (
        isinstance(bridge_drive_limit, int)
        and not isinstance(bridge_drive_limit, bool)
        and 0 < bridge_drive_limit <= ROUTE_TEST_MAXIMUM_DRIVE_COMMAND
    ):
        errors.append("NUCLEO route-test drive command must be an integer <= 30")
    if not _close(bridge.get("drive_command_scale"), 100.0, 1.0e-9):
        errors.append("current NUCLEO profile requires drive_command_scale=100")
    bridge_timeout = bridge.get("command_timeout_sec")
    if not (_positive(bridge_timeout) and float(bridge_timeout) < 0.30):
        errors.append("NUCLEO serial command timeout must be below 0.30 s")
    if not _close(bridge.get("maximum_steering_rad"), maximum_steering, 1.0e-6):
        errors.append("NUCLEO and path-tracker steering ranges do not match")
    current_adc_map = (58744, 31974, 5111)
    configured_adc_map = (
        bridge.get("steer_left_adc"),
        bridge.get("steer_center_adc"),
        bridge.get("steer_right_adc"),
    )
    if configured_adc_map != current_adc_map:
        errors.append("current steering ADC map 58744/31974/5111 must remain unchanged")

    project = _mapping(record.get("project"))
    power = _mapping(record.get("power"))
    geometry = _mapping(record.get("vehicle_geometry"))
    gnss = _mapping(record.get("gnss"))
    lidar = _mapping(record.get("lidar"))
    steering = _mapping(record.get("steering"))
    traction = _mapping(record.get("traction"))
    course = _mapping(record.get("course"))
    acceptance = _mapping(record.get("acceptance"))

    if not str(project.get("vehicle_id", "")).strip():
        errors.append("project.vehicle_id is empty")
    if not str(project.get("measured_at", "")).strip():
        errors.append("project.measured_at is empty")
    operators = project.get("operators")
    if not isinstance(operators, list) or not any(str(value).strip() for value in operators):
        errors.append("project.operators must identify a test operator")
    _require_true(
        errors,
        power,
        ("competition_voltage_rule_confirmed", "physical_estop_verified"),
        "power",
    )
    charged_voltage = power.get("fully_charged_voltage_v")
    if not (_finite(charged_voltage) and 1.0 < float(charged_voltage) <= 24.0):
        errors.append("power.fully_charged_voltage_v must be measured and <= 24 V")
    if not _positive(power.get("main_fuse_a")):
        errors.append("power.main_fuse_a must be recorded")

    if not _close(wheelbase, geometry.get("wheelbase_m"), 0.005):
        errors.append("system wheelbase does not match calibration record")
    if not gps_only:
        body_width = geometry.get("body_width_m")
        half_width = lidar_config.get("vehicle_half_width_m")
        if not (
            _positive(body_width)
            and _positive(half_width)
            and float(half_width) >= float(body_width) * 0.5
        ):
            errors.append("LiDAR corridor is narrower than the measured vehicle")
    left_limit = geometry.get("maximum_left_steering_rad")
    right_limit = geometry.get("maximum_right_steering_rad")
    if not (_finite(left_limit) and float(left_limit) < 0.0):
        errors.append("measured maximum left steering must be negative")
    if not (_finite(right_limit) and float(right_limit) > 0.0):
        errors.append("measured maximum right steering must be positive")
    if _finite(maximum_steering) and _finite(left_limit) and _finite(right_limit):
        if float(maximum_steering) > min(abs(float(left_limit)), abs(float(right_limit))):
            errors.append("configured steering exceeds the measured mechanical-safe range")

    if not single_antenna and not _positive(gnss.get("baseline_m")):
        errors.append("gnss.baseline_m must be measured")
    for name in (
        "datum_latitude_deg",
        "datum_longitude_deg",
        "datum_altitude_m",
        "base_to_ant1_x_m",
        "base_to_ant1_y_m",
        "heading_mount_offset_deg",
    ):
        tolerance = 1.0e-7 if name in ("datum_latitude_deg", "datum_longitude_deg") else 0.005
        if not _close(localizer.get(name), gnss.get(name), tolerance):
            errors.append(f"gnss_localizer.{name} does not match calibration record")

    if not gps_only:
        _require_true(
            errors,
            lidar,
            ("angle_zero_is_vehicle_forward", "positive_angle_is_vehicle_left"),
            "lidar",
        )
    for record_name, config_name in (
        ("adc_left", "steer_left_adc"),
        ("adc_center", "steer_center_adc"),
        ("adc_right", "steer_right_adc"),
    ):
        value = steering.get(record_name)
        if not isinstance(value, int) or isinstance(value, bool):
            errors.append(f"steering.{record_name} must be an integer")
        elif value != bridge.get(config_name):
            errors.append(f"steering.{record_name} does not match NUCLEO UART config")
    adc_left = steering.get("adc_left")
    adc_center = steering.get("adc_center")
    adc_right = steering.get("adc_right")
    if all(
        isinstance(value, int) and not isinstance(value, bool)
        for value in (adc_left, adc_center, adc_right)
    ) and not (adc_right < adc_center < adc_left):
        errors.append("current steering map must remain right < center < left")

    recorded_drive_limit = traction.get("route_test_drive_command_limit")
    if recorded_drive_limit != bridge_drive_limit:
        errors.append("traction.route_test_drive_command_limit must match the tested NUCLEO limit")
    braking_distance = traction.get("low_speed_braking_distance_m")
    emergency_distance = safety.get("emergency_distance_m")
    if not _positive(braking_distance):
        errors.append("traction.low_speed_braking_distance_m must be measured")
    elif _positive(emergency_distance) and float(braking_distance) > float(emergency_distance):
        errors.append("LiDAR emergency distance is shorter than measured low-speed braking distance")

    if course.get("route_calibrated") is not True:
        errors.append("course.route_calibrated=true verification required")
    recorded_route = str(course.get("route_file", "")).strip()
    if not recorded_route:
        errors.append("course.route_file is empty")
    elif Path(recorded_route).name != route_path.name:
        errors.append("launch route file does not match calibration record")
    if route_path.name == "course_template.csv":
        errors.append("course_template.csv is an example and cannot authorize driving")

    acceptance_checks = [
        "wheels_up_test",
        "estop_test",
        "serial_watchdog_test",
        "steering_limit_test",
        "low_speed_straight_test",
        "rtk_dropout_test",
    ]
    if not gps_only:
        acceptance_checks.append("lidar_emergency_test")
    _require_true(
        errors,
        acceptance,
        acceptance_checks,
        "acceptance",
    )

    if route is not None:
        allowed_missions = {"NORMAL", "FINISH"}
        previous_s = -math.inf
        route_speed_limit = float(maximum_speed) if _positive(maximum_speed) else 0.0
        for waypoint in route.waypoints:
            values = (
                waypoint.s_m,
                waypoint.x_m,
                waypoint.y_m,
                waypoint.target_speed_mps,
            )
            if not all(math.isfinite(value) for value in values):
                errors.append(f"route row {waypoint.index + 2} contains a non-finite value")
                break
            if waypoint.s_m <= previous_s:
                errors.append("route s_m must be strictly increasing")
                break
            previous_s = waypoint.s_m
            if waypoint.direction != 1:
                errors.append("first global route test permits forward direction only")
                break
            if waypoint.target_speed_mps > route_speed_limit:
                errors.append(
                    f"route row {waypoint.index + 2} exceeds 1.00 m/s test limit"
                )
                break
            if waypoint.mission not in allowed_missions:
                errors.append("first global route test permits only NORMAL and FINISH tags")
                break
        if route.waypoints[-1].mission != "FINISH":
            errors.append("final route waypoint must use the FINISH mission tag")
        if single_antenna:
            first = route.waypoints[0]
            following = route.waypoints[1]
            dx = following.x_m - first.x_m
            dy = following.y_m - first.y_m
            if math.hypot(dx, dy) <= 1.0e-6:
                errors.append("single-antenna route needs a nonzero first segment")
            else:
                route_heading_deg = (
                    90.0 - math.degrees(math.atan2(dy, dx))
                ) % 360.0
                configured_heading = localizer.get(
                    "single_antenna_initial_heading_deg"
                )
                if _finite(configured_heading):
                    heading_error = abs(
                        (float(configured_heading) - route_heading_deg + 180.0)
                        % 360.0
                        - 180.0
                    )
                    if heading_error > 2.0:
                        errors.append(
                            "single-antenna initial heading does not match route start"
                        )

    return ReadinessReport(tuple(dict.fromkeys(errors)), tuple(dict.fromkeys(warnings)))


def check_gps_only_route_test_readiness(
    system_config_file: str | Path,
    gps_config_file: str | Path,
    ntrip_config_file: str | Path,
    calibration_file: str | Path,
    route_file: str | Path,
) -> ReadinessReport:
    return check_global_route_test_readiness(
        system_config_file,
        gps_config_file,
        calibration_file,
        route_file,
        gps_only=True,
        additional_config_file=ntrip_config_file,
    )
