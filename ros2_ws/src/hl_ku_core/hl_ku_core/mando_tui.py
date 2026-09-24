"""Single-terminal field console for supervised HL Mando route trials."""

from __future__ import annotations

import argparse
import curses
import fcntl
import json
import math
import os
import re
import signal
import subprocess
import sys
import time
import unicodedata
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, LaserScan
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger

from hl_ku_interfaces.msg import DriveCommand, GnssStatus, MissionStatus, VehicleFeedback

from .mando_route_slice import calibration_document, slice_and_place
from .parking_selection import ParkingRouteFamily
from .readiness import ReadinessReport, check_gps_only_route_test_readiness
from .route import ALLOWED_MISSIONS, Route, Waypoint
from .route_progress import RouteProgressMatch, match_route_ahead
from .route_prepare import save_route


DEFAULT_CONFIG = Path("operations/mando_scenarios.yaml")
FRESH_GNSS_SEC = 2.0
FRESH_POSE_SEC = 1.0
FRESH_FAST_SEC = 0.7
FRESH_CAMERA_SEC = 2.0
SAFE_START_SAFETY_REASONS = frozenset(("mission_init",))


def status_mark(state: str) -> str:
    """Return the compact status marker used by the TUI."""
    return {
        "ok": "[✓]",
        "wait": "[대기]",
        "block": "[차단]",
        "optional": "[선택]",
        "info": "[정보]",
    }.get(state, "[대기]")


def terminal_text_width(text: str) -> int:
    """Return the number of terminal columns used by Korean and ASCII text."""

    return sum(
        0
        if unicodedata.combining(character)
        else 2
        if unicodedata.east_asian_width(character) in ("W", "F")
        else 1
        for character in text
    )


def fit_terminal_text(text: str, width: int, align: str = "left") -> str:
    """Clip and pad text to an exact terminal-column width."""

    if width <= 0:
        return ""
    characters: list[str] = []
    used = 0
    for character in str(text):
        character_width = terminal_text_width(character)
        if used + character_width > width:
            break
        characters.append(character)
        used += character_width
    padding = width - used
    if align == "center":
        left = padding // 2
        right = padding - left
    elif align == "right":
        left, right = padding, 0
    else:
        left, right = 0, padding
    return " " * left + "".join(characters) + " " * right


def table_border(widths: tuple[int, ...]) -> str:
    return "+" + "+".join("-" * width for width in widths) + "+"


def table_row(
    cells: tuple[str, ...],
    widths: tuple[int, ...],
    aligns: tuple[str, ...] | None = None,
) -> str:
    if len(cells) != len(widths):
        raise ValueError("table cells and widths must have the same length")
    if aligns is None:
        aligns = tuple("left" for _ in cells)
    if len(aligns) != len(cells):
        raise ValueError("table aligns and cells must have the same length")
    return "|" + "|".join(
        fit_terminal_text(cell, width, align)
        for cell, width, align in zip(cells, widths, aligns)
    ) + "|"


def key_input_label(key: int) -> str:
    """Return a stable label for the latest curses key code."""

    special = {
        ord(" "): "SPACE",
        curses.KEY_LEFT: "LEFT",
        curses.KEY_RIGHT: "RIGHT",
        curses.KEY_UP: "UP",
        curses.KEY_DOWN: "DOWN",
    }
    if key in special:
        return special[key]
    if 32 <= key <= 126:
        return chr(key).upper()
    return str(key)


@dataclass(frozen=True)
class Variant:
    name: str
    route_file: Path
    calibration_file: Path
    preflight: str = "full"
    alignment_confirmed: bool = False
    start_position_tolerance_m: float = 1.0
    start_heading_tolerance_deg: float = 15.0
    minimum_length_m: float = 0.0
    maximum_length_m: float = 1000.0
    source_route: Path | None = None
    source_variant_id: str = ""
    source_start_s_m: float = 0.0
    source_length_m: float = 0.0
    auto_place_at_current_pose: bool = False
    description: str = ""
    segment_code: str = ""
    fsm_label: str = ""
    direction_summary: str = "전진"
    footprint: str = ""
    run_summary: str = ""
    preserve_source_profile: bool = False
    mission_override: str = ""
    enable_lidar_detour: bool = False
    enable_parking_selection: bool = False
    allow_reverse: bool = False
    direction_change_hold_sec: float = 0.75
    enable_camera_perception: bool = False
    enable_yolo_perception: bool = False
    hill_hold_tuning: bool = False
    default_hill_approach_pwm: int = -1
    default_hill_hold_pwm: int = 0
    default_hill_post_stop_pwm: int = -1


@dataclass(frozen=True)
class Scenario:
    number: int
    name: str
    kind: str
    default_speed: float
    variants: tuple[Variant, ...] = ()
    maximum_pwm: int = 100
    survey_minimum_samples: int = 100
    survey_maximum_hdop: float = 2.5
    description: str = ""
    finger_initial_pwm: int = -1


@dataclass(frozen=True)
class ConsoleConfig:
    source_file: Path
    system_config: Path
    gps_config: Path
    receiver_config: Path
    ntrip_config: Path
    recording_script: Path
    finger_drive_script: Path
    log_directory: Path
    require_recording: bool
    scenarios: tuple[Scenario, ...]
    enable_local_detour_steering: bool = False
    camera_device: str = (
        "/dev/v4l/by-id/"
        "usb-046d_C270_HD_WEBCAM_200901010001-video-index0"
    )
    title: str = "HL MANDO FIELD CONSOLE"
    course_datum_file: Path | None = None


@dataclass
class Sample:
    value: Any = None
    received_at: float | None = None

    def fresh(self, maximum_age_sec: float) -> bool:
        return (
            self.received_at is not None
            and 0.0 <= time.monotonic() - self.received_at <= maximum_age_sec
        )


@dataclass
class ProcessSlot:
    child: subprocess.Popen | None = None
    log_stream: Any = None
    log_file: Path | None = None
    label: str = ""

    @property
    def running(self) -> bool:
        return self.child is not None and self.child.poll() is None

    def start(self, command: list[str], label: str, log_directory: Path) -> None:
        self.stop()
        log_directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_label = "".join(c if c.isalnum() or c in "-_" else "_" for c in label)
        self.log_file = log_directory / f"{stamp}_{safe_label}.log"
        self.log_stream = self.log_file.open("a", encoding="utf-8")
        self.child = subprocess.Popen(
            command,
            # The TUI must be the only process reading its terminal.  ROS
            # launch/record children otherwise occasionally consume Q or
            # another control key before curses sees it.
            stdin=subprocess.DEVNULL,
            stdout=self.log_stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        self.label = label

    def stop(self, timeout_sec: float = 5.0) -> None:
        child = self.child
        if child is not None and child.poll() is None:
            for sent_signal, timeout in (
                (signal.SIGINT, timeout_sec),
                (signal.SIGTERM, 2.0),
                (signal.SIGKILL, 1.0),
            ):
                try:
                    os.killpg(child.pid, sent_signal)
                    child.wait(timeout=timeout)
                    break
                except ProcessLookupError:
                    break
                except subprocess.TimeoutExpired:
                    continue
        if self.log_stream is not None:
            self.log_stream.close()
        self.child = None
        self.log_stream = None
        self.label = ""


def _absolute(base: Path, value: Any, field_name: str) -> Path:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{field_name} must be a non-empty path")
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (base / path).resolve()


def _bounded_number(value: Any, minimum: float, maximum: float, field_name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{field_name} must be numeric") from error
    if not math.isfinite(number) or not minimum <= number <= maximum:
        raise ValueError(f"{field_name} must be between {minimum:g} and {maximum:g}")
    return number


def _pwm_number(value: Any, field_name: str) -> int:
    number = _bounded_number(value, 0, 100, field_name)
    if not number.is_integer():
        raise ValueError(f"{field_name} must be an integer PWM value")
    return int(number)


def _optional_pwm_number(value: Any, field_name: str) -> int:
    number = _bounded_number(value, -1, 100, field_name)
    if not number.is_integer():
        raise ValueError(f"{field_name} must be an integer PWM value")
    return int(number)


def load_config(path: str | Path) -> ConsoleConfig:
    source = Path(path).expanduser().resolve()
    with source.open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError("TUI config root must be a mapping")
    if document.get("version") != 1:
        raise ValueError("TUI config version must be 1")
    base = source.parent
    common = document.get("common")
    if not isinstance(common, dict):
        raise ValueError("common must be a mapping")
    scenario_items = document.get("scenarios")
    if not isinstance(scenario_items, list) or not scenario_items:
        raise ValueError("scenarios must contain at least one item")

    scenarios: list[Scenario] = []
    numbers: set[int] = set()
    for index, item in enumerate(scenario_items):
        if not isinstance(item, dict):
            raise ValueError(f"scenarios[{index}] must be a mapping")
        number = int(item.get("number", 0))
        if not 1 <= number <= 9 or number in numbers:
            raise ValueError("scenario numbers must be unique integers from 1 to 9")
        numbers.add(number)
        name = str(item.get("name", "")).strip()
        kind = str(item.get("kind", "route")).strip().lower()
        if not name or kind not in ("route", "finger", "datum"):
            raise ValueError(f"scenario {number} has an invalid name or kind")
        variants: list[Variant] = []
        maximum_pwm = int(_bounded_number(item.get("maximum_pwm", 100), 1, 100, f"scenario {number} maximum_pwm"))
        finger_initial_pwm = 0
        if kind == "route":
            default_speed = _pwm_number(item.get("default_pwm", 30), f"scenario {number} default_pwm")
            maximum_pwm = 100
            variant_items = item.get("variants")
            if not isinstance(variant_items, list) or not variant_items:
                raise ValueError(f"route scenario {number} needs at least one variant")
            variant_defaults = item.get("variant_defaults", {})
            if not isinstance(variant_defaults, dict):
                raise ValueError(
                    f"scenario {number} variant_defaults must be a mapping"
                )
            for variant_index, raw in enumerate(variant_items):
                if not isinstance(raw, dict):
                    raise ValueError(f"scenario {number} variant {variant_index + 1} is invalid")
                def value(key: str, fallback: Any = None) -> Any:
                    return raw.get(key, variant_defaults.get(key, fallback))
                preflight = str(value("preflight", "full")).strip().lower()
                if preflight not in ("full", "supervised", "relaxed", "field"):
                    raise ValueError("preflight must be full, supervised, relaxed, or field")
                source_route_value = value("source_route")
                source_route = (
                    _absolute(base, source_route_value, "source_route")
                    if isinstance(source_route_value, str)
                    and source_route_value.strip()
                    else None
                )
                auto_place = value("auto_place_at_current_pose", False) is True
                if auto_place and source_route is None:
                    raise ValueError(
                        f"scenario {number} variant {variant_index + 1} "
                        "needs source_route for automatic placement"
                    )
                mission_override = str(value("mission_override", "")).strip().upper()
                if mission_override and (
                    mission_override not in ALLOWED_MISSIONS
                    or mission_override == "FINISH"
                ):
                    raise ValueError(
                        f"scenario {number} variant {variant_index + 1} "
                        f"has invalid mission_override: {mission_override}"
                    )
                enable_lidar_detour = value("enable_lidar_detour", False) is True
                if enable_lidar_detour and mission_override != "S_OBSTACLE" and source_route is None:
                    raise ValueError(
                        f"scenario {number} variant {variant_index + 1} "
                        "needs an S_OBSTACLE override or a source route with the S mission"
                    )
                enable_parking_selection = (
                    value("enable_parking_selection", False) is True
                )
                source_variant_id = str(
                    value("source_variant_id", "") or ""
                ).strip()
                if enable_parking_selection and not source_variant_id.lower().startswith(
                    "p1_t1_f"
                ):
                    raise ValueError(
                        f"scenario {number} variant {variant_index + 1} "
                        "must start from p1_t1_f1/f2 for automatic parking selection"
                    )
                variants.append(
                    Variant(
                        name=str(raw.get("name", f"구간 {variant_index + 1}")).strip(),
                        route_file=_absolute(base, raw.get("route_file"), "route_file"),
                        calibration_file=_absolute(base, raw.get("calibration_file"), "calibration_file"),
                        preflight=preflight,
                        alignment_confirmed=value("alignment_confirmed", False) is True,
                        # A zero tolerance explicitly disables the operator-console
                        # distance gate.  The path tracker still acquires the nearest
                        # forward point and the heading/GNSS/feedback checks remain in
                        # force.
                        start_position_tolerance_m=_bounded_number(value("start_position_tolerance_m", 1.0), 0.0, 5000.0, "start position tolerance"),
                        start_heading_tolerance_deg=_bounded_number(value("start_heading_tolerance_deg", 15.0), 1.0, 90.0, "start heading tolerance"),
                        minimum_length_m=_bounded_number(value("minimum_length_m", 0.0), 0.0, 5000.0, "minimum route length"),
                        maximum_length_m=_bounded_number(value("maximum_length_m", 1000.0), 0.1, 5000.0, "maximum route length"),
                        source_route=source_route,
                        source_variant_id=source_variant_id,
                        source_start_s_m=_bounded_number(
                            value("source_start_s_m", 0.0),
                            0.0,
                            5000.0,
                            "source route start",
                        ),
                        source_length_m=_bounded_number(
                            value("source_length_m", 8.0),
                            0.1,
                            5000.0,
                            "source route length",
                        ),
                        auto_place_at_current_pose=auto_place,
                        description=str(value("description", "")).strip(),
                        segment_code=str(value("segment_code", "")).strip(),
                        fsm_label=str(value("fsm_label", "")).strip(),
                        direction_summary=str(
                            value("direction_summary", "전진")
                        ).strip(),
                        footprint=str(value("footprint", "")).strip(),
                        run_summary=str(value("run_summary", "")).strip(),
                        preserve_source_profile=(
                            value("preserve_source_profile", False) is True
                        ),
                        mission_override=mission_override,
                        enable_lidar_detour=enable_lidar_detour,
                        enable_parking_selection=enable_parking_selection,
                        allow_reverse=value("allow_reverse", False) is True,
                        direction_change_hold_sec=_bounded_number(
                            value("direction_change_hold_sec", 0.75),
                            0.0,
                            5.0,
                            "direction change hold",
                        ),
                        enable_camera_perception=value("enable_camera_perception", False) is True,
                        enable_yolo_perception=value("enable_yolo_perception", False) is True,
                        hill_hold_tuning=value("hill_hold_tuning", False) is True,
                        default_hill_approach_pwm=_optional_pwm_number(
                            value("default_hill_approach_pwm", -1),
                            "default hill approach PWM",
                        ),
                        default_hill_hold_pwm=_pwm_number(
                            value("default_hill_hold_pwm", 0), "default hill hold PWM"
                        ),
                        default_hill_post_stop_pwm=_optional_pwm_number(
                            value("default_hill_post_stop_pwm", -1),
                            "default hill post-stop PWM",
                        ),
                    )
                )
        elif kind == "finger":
            default_speed = _bounded_number(item.get("default_pwm", min(30, maximum_pwm)), 1, maximum_pwm, f"scenario {number} default_pwm")
            finger_initial_pwm = _pwm_number(
                item.get("initial_pwm", int(default_speed)),
                f"scenario {number} initial_pwm",
            )
            if finger_initial_pwm > maximum_pwm:
                raise ValueError(
                    f"scenario {number} initial_pwm exceeds maximum_pwm"
                )
        else:
            default_speed = _bounded_number(
                item.get("duration_sec", 120.0),
                10.0,
                3600.0,
                f"scenario {number} datum duration",
            )
            maximum_pwm = 30
        survey_minimum_samples = int(
            _bounded_number(
                item.get("minimum_samples", 100),
                1,
                100000,
                f"scenario {number} survey minimum samples",
            )
        )
        survey_maximum_hdop = _bounded_number(
            item.get("maximum_hdop", 2.5),
            0.1,
            20.0,
            f"scenario {number} survey maximum HDOP",
        )
        scenarios.append(
            Scenario(
                number,
                name,
                kind,
                default_speed,
                tuple(variants),
                maximum_pwm,
                survey_minimum_samples,
                survey_maximum_hdop,
                str(item.get("description", "")).strip(),
                finger_initial_pwm,
            )
        )

    return ConsoleConfig(
        source_file=source,
        system_config=_absolute(base, common.get("system_config"), "system_config"),
        gps_config=_absolute(base, common.get("gps_config"), "gps_config"),
        receiver_config=_absolute(base, common.get("receiver_config"), "receiver_config"),
        ntrip_config=_absolute(base, common.get("ntrip_config"), "ntrip_config"),
        recording_script=_absolute(base, common.get("recording_script"), "recording_script"),
        finger_drive_script=_absolute(base, common.get("finger_drive_script"), "finger_drive_script"),
        log_directory=_absolute(base, common.get("log_directory", "../logs/mando_tui"), "log_directory"),
        require_recording=common.get("require_recording", False) is True,
        scenarios=tuple(sorted(scenarios, key=lambda scenario: scenario.number)),
        enable_local_detour_steering=(
            common.get("enable_local_detour_steering", False) is True
        ),
        camera_device=str(
            common.get(
                "camera_device",
                "/dev/v4l/by-id/"
                "usb-046d_C270_HD_WEBCAM_200901010001-video-index0",
            )
        ).strip(),
        title=str(common.get("title", "HL MANDO FIELD CONSOLE")).strip(),
        course_datum_file=(
            _absolute(base, common["course_datum_file"], "course_datum_file")
            if common.get("course_datum_file") else None
        ),
    )


def route_validation_errors(route: Route, variant: Variant) -> tuple[str, ...]:
    """Validate a runnable route without rejecting authored FSM profiles."""

    errors: list[str] = []
    length = route.waypoints[-1].s_m
    if not variant.minimum_length_m <= length <= variant.maximum_length_m:
        errors.append(
            f"경로 길이 {length:.1f} m가 설정 범위 "
            f"{variant.minimum_length_m:.1f}~{variant.maximum_length_m:.1f} m 밖입니다"
        )
    if (
        any(point.direction < 0 for point in route.waypoints)
        and not variant.allow_reverse
    ):
        errors.append(
            "후진 waypoint가 있지만 이 세부구간의 allow_reverse가 꺼져 있습니다"
        )
    if route.waypoints[-1].mission != "FINISH" or abs(route.waypoints[-1].target_speed_mps) > 1.0e-9:
        errors.append("마지막 waypoint는 FINISH이며 속도 0이어야 합니다")
    return tuple(errors)


def route_geometry(variant: Variant) -> tuple[Route | None, tuple[str, ...]]:
    try:
        route = Route.load_csv(variant.route_file)
    except (OSError, ValueError, KeyError) as error:
        return None, (f"경로 파일: {error}",)
    return route, route_validation_errors(route, variant)


def _yaml_mapping(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"{path} root is not a mapping")
    return value


def supervised_readiness(config: ConsoleConfig, variant: Variant) -> ReadinessReport:
    errors: list[str] = []
    for name, path in (
        ("system config", config.system_config),
        ("receiver config", config.receiver_config),
        ("NTRIP config", config.ntrip_config),
    ):
        if not path.is_file():
            errors.append(f"{name} 파일이 없습니다: {path}")
    route, geometry_errors = route_geometry(variant)
    errors.extend(geometry_errors)
    if not variant.alignment_confirmed:
        errors.append("mando_scenarios.yaml의 alignment_confirmed=true 확인이 필요합니다")
    try:
        gps = _yaml_mapping(config.gps_config)
        localizer = gps.get("gnss_localizer", {}).get("ros__parameters", {})
        if localizer.get("datum_configured") is not True:
            errors.append("GNSS ENU datum이 설정되지 않았습니다")
    except (OSError, yaml.YAMLError, ValueError) as error:
        errors.append(f"GPS config: {error}")
        localizer = {}
    try:
        record = _yaml_mapping(variant.calibration_file)
        course = record.get("course", {})
        if course.get("route_calibrated") is not True:
            errors.append("calibration course.route_calibrated=true 확인이 필요합니다")
        if Path(str(course.get("route_file", ""))).name != variant.route_file.name:
            errors.append("calibration course.route_file과 선택 경로가 다릅니다")
        gnss = record.get("gnss", {})
        for config_key, record_key in (
            ("datum_latitude_deg", "datum_latitude_deg"),
            ("datum_longitude_deg", "datum_longitude_deg"),
            ("datum_altitude_m", "datum_altitude_m"),
            ("base_to_ant1_x_m", "base_to_ant1_x_m"),
            ("base_to_ant1_y_m", "base_to_ant1_y_m"),
            ("heading_mount_offset_deg", "heading_mount_offset_deg"),
        ):
            left, right = localizer.get(config_key), gnss.get(record_key)
            if not isinstance(left, (int, float)) or not isinstance(right, (int, float)) or not math.isclose(float(left), float(right), abs_tol=1.0e-7):
                errors.append(f"GPS config와 calibration의 {config_key} 값이 다릅니다")
    except (OSError, yaml.YAMLError, ValueError) as error:
        errors.append(f"calibration: {error}")
    if route is None and not geometry_errors:
        errors.append("경로를 읽지 못했습니다")
    return ReadinessReport(tuple(dict.fromkeys(errors)), ())


def field_readiness(config: ConsoleConfig, variant: Variant) -> ReadinessReport:
    """Validate a measured race route without pretending it was campus-placed.

    Field mode keeps the live RTK, start-pose, vehicle-feedback and safety gates,
    while avoiding the calibration-record preflight that is intentionally still
    incomplete for the Course 07 preview bundle.
    """

    errors: list[str] = []
    warnings: list[str] = []
    for name, path in (
        ("system config", config.system_config),
        ("GPS config", config.gps_config),
        ("receiver config", config.receiver_config),
        ("NTRIP config", config.ntrip_config),
    ):
        if not path.is_file():
            errors.append(f"{name} 파일이 없습니다: {path}")
    _route, geometry_errors = route_geometry(variant)
    errors.extend(geometry_errors)
    if variant.auto_place_at_current_pose:
        errors.append("본선 field 경로는 현재 위치 자동배치를 사용할 수 없습니다")
    if variant.source_route is not None and not variant.source_route.is_file():
        errors.append(f"원본 경로 파일이 없습니다: {variant.source_route}")

    try:
        gps = _yaml_mapping(config.gps_config)
        localizer = gps.get("gnss_localizer", {}).get("ros__parameters", {})
        if localizer.get("datum_configured") is not True:
            errors.append("본선 GPS datum이 설정되지 않았습니다")
        if config.course_datum_file is None:
            errors.append("본선 course_datum_file이 설정되지 않았습니다")
        else:
            datum = _yaml_mapping(config.course_datum_file)
            expected = datum.get("gnss_localizer", {}).get("ros__parameters", {})
            for key in (
                "datum_latitude_deg",
                "datum_longitude_deg",
                "datum_altitude_m",
            ):
                left, right = localizer.get(key), expected.get(key)
                if (
                    not isinstance(left, (int, float))
                    or not isinstance(right, (int, float))
                    or not math.isclose(float(left), float(right), abs_tol=1.0e-7)
                ):
                    errors.append(f"GPS config와 본선 datum의 {key} 값이 다릅니다")
    except (OSError, yaml.YAMLError, ValueError) as error:
        errors.append(f"본선 datum 검사: {error}")
    warnings.append("출발 전 TUI의 시작점 거리와 방향오차를 현장에서 확인해야 합니다")
    return ReadinessReport(tuple(dict.fromkeys(errors)), tuple(warnings))


def relaxed_readiness(config: ConsoleConfig, variant: Variant) -> ReadinessReport:
    """Check only files and route properties needed to execute the route."""
    errors: list[str] = []
    for name, path in (
        ("system config", config.system_config),
        ("GPS config", config.gps_config),
        ("NTRIP config", config.ntrip_config),
    ):
        if not path.is_file():
            errors.append(f"{name} 파일이 없습니다: {path}")
    try:
        route = Route.load_csv(variant.route_file)
    except (OSError, ValueError, KeyError) as error:
        errors.append(f"경로 파일: {error}")
    else:
        if (
            any(point.direction < 0 for point in route.waypoints)
            and not variant.allow_reverse
        ):
            errors.append(
                "후진 waypoint가 있지만 이 세부구간의 allow_reverse가 꺼져 있습니다"
            )
        if (
            route.waypoints[-1].mission != "FINISH"
            or abs(route.waypoints[-1].target_speed_mps) > 1.0e-9
        ):
            errors.append("마지막 waypoint는 FINISH이며 속도 0이어야 합니다")
    return ReadinessReport(tuple(dict.fromkeys(errors)), ())


def automatic_source_readiness(
    config: ConsoleConfig, variant: Variant
) -> ReadinessReport:
    """Validate an automatic route before its current-pose output exists."""

    errors: list[str] = []
    for name, path in (
        ("system config", config.system_config),
        ("GPS config", config.gps_config),
        ("NTRIP config", config.ntrip_config),
    ):
        if not path.is_file():
            errors.append(f"{name} 파일이 없습니다: {path}")
    if variant.source_route is None:
        errors.append("현재 위치 자동 배치용 source_route가 없습니다")
        return ReadinessReport(tuple(dict.fromkeys(errors)), ())
    try:
        source = Route.load_csv(
            variant.source_route, variant.source_variant_id or None
        )
        preview = slice_and_place(
            source,
            variant.source_start_s_m,
            variant.source_length_m,
            0.0,
            0.0,
            0.0,
            0.1,
            variant.preserve_source_profile,
        )
        errors.extend(route_validation_errors(preview, variant))
        calibration_document(config.gps_config, variant.route_file)
    except (OSError, ValueError, KeyError, yaml.YAMLError) as error:
        errors.append(f"자동 배치 원본: {error}")
    return ReadinessReport(tuple(dict.fromkeys(errors)), ())


def static_readiness(config: ConsoleConfig, variant: Variant) -> ReadinessReport:
    if (
        variant.auto_place_at_current_pose
        and automatic_placement_reason(config, variant) is not None
    ):
        return automatic_source_readiness(config, variant)
    if variant.preflight == "relaxed":
        return relaxed_readiness(config, variant)
    if variant.preflight == "supervised":
        return supervised_readiness(config, variant)
    if variant.preflight == "field":
        return field_readiness(config, variant)
    report = check_gps_only_route_test_readiness(
        config.system_config,
        config.gps_config,
        config.ntrip_config,
        variant.calibration_file,
        variant.route_file,
    )
    _route, geometry_errors = route_geometry(variant)
    errors = tuple(dict.fromkeys((*report.errors, *geometry_errors)))
    return ReadinessReport(errors, report.warnings)


def camera_device_path(config: ConsoleConfig) -> str:
    """Resolve /dev/v4l/by-id links because usb_cam accepts canonical /dev/videoN."""
    path = Path(config.camera_device).expanduser()
    return str(path.resolve()) if path.exists() else str(path)


def apply_datum_to_gps_config(datum_file: Path, gps_config: Path) -> Path:
    """Apply only datum scalars while preserving the operator's YAML comments."""
    datum = _yaml_mapping(datum_file)
    parameters = (
        datum.get("gnss_localizer", {}).get("ros__parameters", {})
        if isinstance(datum.get("gnss_localizer"), dict)
        else {}
    )
    values = {
        "datum_configured": True,
        "datum_latitude_deg": parameters.get("datum_latitude_deg"),
        "datum_longitude_deg": parameters.get("datum_longitude_deg"),
        "datum_altitude_m": parameters.get("datum_altitude_m"),
    }
    if not all(
        isinstance(values[key], (int, float)) and math.isfinite(float(values[key]))
        for key in (
            "datum_latitude_deg",
            "datum_longitude_deg",
            "datum_altitude_m",
        )
    ):
        raise ValueError("datum result has incomplete latitude/longitude/altitude")
    lines = gps_config.read_text(encoding="utf-8").splitlines(keepends=True)
    in_localizer = False
    replaced: set[str] = set()
    for index, line in enumerate(lines):
        if line and not line[0].isspace() and not line.lstrip().startswith("#"):
            in_localizer = line.startswith("gnss_localizer:")
        if not in_localizer:
            continue
        for key, value in values.items():
            match = re.match(rf"^(\s+{re.escape(key)}:\s*).*$", line.rstrip("\n"))
            if match:
                rendered = "true" if value is True else repr(float(value))
                newline = "\n" if line.endswith("\n") else ""
                lines[index] = match.group(1) + rendered + newline
                replaced.add(key)
                break
    missing = set(values) - replaced
    if missing:
        raise ValueError("GPS config datum keys missing: " + ", ".join(sorted(missing)))
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = gps_config.with_suffix(gps_config.suffix + f".bak_{stamp}")
    backup.write_bytes(gps_config.read_bytes())
    temporary = gps_config.with_suffix(gps_config.suffix + ".tmp")
    temporary.write_text("".join(lines), encoding="utf-8")
    temporary.replace(gps_config)
    return backup


def pose_xy_heading_deg(
    pose: PoseWithCovarianceStamped,
) -> tuple[float, float, float]:
    """Extract a finite ENU position and vehicle yaw from a live pose."""

    position = pose.pose.pose.position
    orientation = pose.pose.pose.orientation
    values = (
        position.x,
        position.y,
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("ENU pose contains a non-finite value")
    norm = math.sqrt(
        orientation.x * orientation.x
        + orientation.y * orientation.y
        + orientation.z * orientation.z
        + orientation.w * orientation.w
    )
    if norm < 1.0e-9:
        raise ValueError("ENU pose orientation quaternion is empty")
    x = orientation.x / norm
    y = orientation.y / norm
    z = orientation.z / norm
    w = orientation.w / norm
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return float(position.x), float(position.y), math.degrees(yaw)


def automatic_placement_reason(
    config: ConsoleConfig, variant: Variant
) -> str | None:
    """Return why an automatically placed route must be created again."""

    if not variant.auto_place_at_current_pose:
        return None
    if not variant.route_file.is_file():
        return "경로 파일 없음"
    if not variant.calibration_file.is_file():
        return "경로 보정 파일 없음"
    try:
        expected = calibration_document(config.gps_config, variant.route_file)
        actual = _yaml_mapping(variant.calibration_file)
    except (OSError, ValueError, yaml.YAMLError) as error:
        return f"경로 보정 확인 실패: {error}"
    expected_gnss = expected.get("gnss", {})
    actual_gnss = actual.get("gnss", {})
    for key, expected_value in expected_gnss.items():
        actual_value = actual_gnss.get(key)
        if not isinstance(actual_value, (int, float)) or not math.isclose(
            float(actual_value), float(expected_value), abs_tol=1.0e-7
        ):
            return "datum/안테나 설정 변경"
    actual_route = actual.get("course", {}).get("route_file")
    if Path(str(actual_route)).name != variant.route_file.name:
        return "경로 보정 파일명 불일치"
    return None


def place_variant_at_pose(
    config: ConsoleConfig,
    variant: Variant,
    pose: PoseWithCovarianceStamped,
    speed_mps: float,
) -> Route:
    """Rigidly place a source slice at the current vehicle pose and save it."""

    if not variant.auto_place_at_current_pose or variant.source_route is None:
        raise ValueError("이 경로에는 현재 위치 자동 배치 설정이 없습니다")
    target_x, target_y, target_heading = pose_xy_heading_deg(pose)
    source = Route.load_csv(
        variant.source_route, variant.source_variant_id or None
    )
    placed = slice_and_place(
        source,
        variant.source_start_s_m,
        variant.source_length_m,
        target_x,
        target_y,
        target_heading,
        speed_mps,
        variant.preserve_source_profile,
    )
    if variant.mission_override:
        final_index = len(placed.waypoints) - 1
        placed = Route(
            Waypoint(
                index=waypoint.index,
                s_m=waypoint.s_m,
                x_m=waypoint.x_m,
                y_m=waypoint.y_m,
                target_speed_mps=waypoint.target_speed_mps,
                mission=(
                    "FINISH"
                    if index == final_index
                    else variant.mission_override
                ),
                direction=waypoint.direction,
                fsm_zone=waypoint.fsm_zone,
                fsm_event_ids=waypoint.fsm_event_ids,
                fsm_actions=waypoint.fsm_actions,
                fsm_conditions=waypoint.fsm_conditions,
                fsm_hold_sec=waypoint.fsm_hold_sec,
                direction_source=waypoint.direction_source,
            )
            for index, waypoint in enumerate(placed.waypoints)
        )
    save_route(placed, variant.route_file)
    document = calibration_document(config.gps_config, variant.route_file)
    document["course"]["route_calibrated"] = True
    document["course"]["placement"] = {
        "mode": "current_rtk_pose",
        "source_route": variant.source_route.name,
        "source_variant_id": variant.source_variant_id,
        "source_start_s_m": variant.source_start_s_m,
        "source_length_m": variant.source_length_m,
        "target_x_m": target_x,
        "target_y_m": target_y,
        "target_heading_deg": target_heading,
        "scale": 1.0,
        "preserve_source_profile": variant.preserve_source_profile,
        "mission_override": variant.mission_override,
        "allow_reverse": variant.allow_reverse,
    }
    variant.calibration_file.parent.mkdir(parents=True, exist_ok=True)
    temporary = variant.calibration_file.with_suffix(
        variant.calibration_file.suffix + ".tmp"
    )
    temporary.write_text(
        yaml.safe_dump(document, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    temporary.replace(variant.calibration_file)
    return placed


def build_rtk_command(config: ConsoleConfig) -> list[str]:
    return [
        "ros2", "launch", "hl_ku_foxglove", "mando_rtk_live.launch.py",
        f"receiver_config_file:={config.receiver_config}",
        f"ntrip_config_file:={config.ntrip_config}",
        f"gps_config_file:={config.gps_config}",
        f"camera_device:={camera_device_path(config)}",
    ]


def build_datum_command(
    config: ConsoleConfig, scenario: Scenario, duration_sec: float, output_file: Path
) -> list[str]:
    return [
        *build_rtk_command(config),
        "survey_enabled:=true",
        f"survey_duration_sec:={duration_sec:.1f}",
        f"survey_minimum_samples:={scenario.survey_minimum_samples}",
        f"survey_maximum_hdop:={scenario.survey_maximum_hdop:.2f}",
        f"survey_output_file:={output_file}",
    ]


def variant_display_name(variant: Variant) -> str:
    """Return the route label shared by the TUI and Foxglove dashboard."""

    prefix = f"[{variant.segment_code}] " if variant.segment_code else ""
    source_variant = (
        f" · {variant.source_variant_id.upper().replace('_', '/')}"
        if variant.source_variant_id
        else ""
    )
    return (
        f"{prefix}{variant.name}{source_variant} · {variant.description}"
        if variant.description
        else f"{prefix}{variant.name}{source_variant}"
    )


def variant_compact_summary(variant: Variant) -> str:
    """Return a short, unambiguous scenario-table label."""

    code = f"[{variant.segment_code}] " if variant.segment_code else ""
    source_range = ""
    if variant.source_route is not None and variant.source_length_m > 0.0:
        source_end = variant.source_start_s_m + variant.source_length_m
        source_range = (
            f"s {variant.source_start_s_m:.1f}~{source_end:.1f}m"
        )
    source_variant = (
        f"형상 {variant.source_variant_id.upper().replace('_', '/')}"
        if variant.source_variant_id
        else ""
    )
    details = [
        item for item in (source_range, source_variant, variant.fsm_label) if item
    ]
    suffix = " · " + " · ".join(details) if details else ""
    return f"{code}{variant.name}{suffix}"


def variant_detail_lines(variant: Variant) -> tuple[str, ...]:
    """Return the selected route facts shown below the scenario table."""

    code = variant.segment_code or "--"
    lines = [f"선택 구간  [{code}] {variant.name}"]
    if variant.source_route is not None:
        source_end = variant.source_start_s_m + variant.source_length_m
        lines.append(
            "원본 경로  "
            f"{variant.source_route.name}  |  "
            f"형상={variant.source_variant_id or '단일'}  |  "
            f"s={variant.source_start_s_m:.3f}~{source_end:.3f} m  |  "
            f"길이={variant.source_length_m:.2f} m  |  scale=1.0"
        )
    labels = []
    if variant.fsm_label:
        labels.append(f"FSM={variant.fsm_label}")
    if variant.direction_summary:
        labels.append(f"진행={variant.direction_summary}")
    if variant.footprint:
        labels.append(f"풋프린트={variant.footprint}")
    if labels:
        lines.append("주행 정보  " + "  |  ".join(labels))
    if variant.run_summary:
        lines.append("실행 내용  " + variant.run_summary)
    if variant.enable_lidar_detour:
        lines.append(
            "장애물 회피  실제 /scan 군집 · 후보 실패/끊김 시 전역경로 계속"
        )
    if variant.enable_parking_selection:
        lines.append(
            "주차 선택  T/P 미션에서만 라이다로 1·2번 공간을 판정 · 미확인/둘 다 막힘은 1번"
        )
    if variant.enable_yolo_perception:
        lines.append(
            "카메라 YOLO  신호등/보행자 구간만 활성 · 종료 기본 F1, 빨강이 왼쪽이면 F2 고정"
        )
    return tuple(lines)


def variant_detour_enabled(config: ConsoleConfig, variant: Variant) -> bool:
    return variant.enable_lidar_detour or (
        config.enable_local_detour_steering
        and variant.mission_override == "S_OBSTACLE"
    )


def build_route_command(
    config: ConsoleConfig,
    variant: Variant,
    pwm: int,
    hill_hold_pwm: int = 0,
) -> list[str]:
    preflight = "true" if variant.preflight == "full" else "false"
    relaxed = "true" if variant.preflight == "relaxed" else "false"
    require_velocity = "false" if variant.preflight == "relaxed" else "true"
    pwm = _pwm_number(pwm, "route PWM")
    hill_hold_pwm = _pwm_number(hill_hold_pwm, "hill hold PWM")
    detour_enabled = variant_detour_enabled(config, variant)
    parking_enabled = variant.enable_parking_selection
    command = [
        "ros2", "launch", "hl_ku_foxglove", "mando_live.launch.py",
        f"config_file:={config.system_config}",
        f"gps_config_file:={config.gps_config}",
        f"ntrip_config_file:={config.ntrip_config}",
        f"calibration_file:={variant.calibration_file}",
        f"route_file:={variant.route_file}",
        f"active_variant_name:={variant_display_name(variant)}",
        f"source_route_file:={variant.source_route or variant.route_file}",
        f"source_start_s_m:={variant.source_start_s_m:.3f}",
        f"source_length_m:={variant.source_length_m:.3f}",
        f"source_segment_name:={variant.name}",
        f"allow_reverse:={'true' if variant.allow_reverse else 'false'}",
        f"direction_change_hold_sec:={variant.direction_change_hold_sec:.2f}",
        f"camera_device:={camera_device_path(config)}",
        "rviz:=true", "actuator_bridge_enabled:=true",
        "camera_enabled:=true",
        "use_lidar_pipeline:=true", "start_lidar_driver:=true",
        "use_local_detour_planner:=true",
        "lidar_geometry_verified:="
        f"{'true' if detour_enabled or parking_enabled else 'false'}",
        "enable_local_detour_steering:="
        f"{'true' if detour_enabled else 'false'}",
        "enable_parking_selection:="
        f"{'true' if parking_enabled else 'false'}",
        "use_camera_perception:="
        f"{'true' if variant.enable_camera_perception else 'false'}",
        "use_yolo_missions:="
        f"{'true' if variant.enable_yolo_perception else 'false'}",
        "hill_hold_enabled:="
        f"{'true' if variant.hill_hold_tuning and hill_hold_pwm > 0 else 'false'}",
        f"hill_hold_duty:={hill_hold_pwm / 100.0:.2f}",
        f"hill_approach_pwm:={variant.default_hill_approach_pwm}",
        f"hill_post_stop_pwm:={variant.default_hill_post_stop_pwm}",
        "route_calibrated:=true", "drive_enabled:=true",
        f"require_preflight:={preflight}",
        f"relaxed_route_test_mode:={relaxed}",
        f"require_velocity_feedback:={require_velocity}",
        "mission_speed_mps:=0.300",
        f"fixed_drive_pwm:={pwm}",
        "maximum_drive_duty:=1.00",
        "minimum_forward_duty:=0.00",
        "maximum_drive_command:=100",
    ]
    # ROS 2 launch rejects an argument whose value is completely empty
    # (for example ``source_fsm_label:=``).  Whole-course variants do not
    # have a single FSM label, so leave optional metadata out and let the
    # launch file's empty defaults apply.
    if variant.source_variant_id:
        command.append(f"source_variant_id:={variant.source_variant_id}")
    if variant.fsm_label:
        command.append(f"source_fsm_label:={variant.fsm_label}")
    return command


def alignment_match(
    routes: Iterable[Route], pose: PoseWithCovarianceStamped
) -> RouteProgressMatch | None:
    position = pose.pose.pose.position
    orientation = pose.pose.pose.orientation
    yaw = math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )
    matches = []
    for route in routes:
        match = match_route_ahead(route, position.x, position.y, yaw)
        if match is not None:
            matches.append(match)
    return min(matches, key=lambda match: match.score) if matches else None


def alignment_error(route: Route, pose: PoseWithCovarianceStamped) -> tuple[float, float]:
    match = alignment_match((route,), pose)
    if match is None:
        return math.inf, math.inf
    return match.distance_m, abs(math.degrees(match.heading_error_rad))


def gnss_blockers(status: GnssStatus | None, fresh: bool) -> list[str]:
    if status is None or not fresh:
        return ["GNSS 상태 미수신/시간초과"]
    blockers: list[str] = []
    if status.fix_type != GnssStatus.FIX_RTK_FIXED:
        blockers.append("RTK FIXED 아님")
    if not status.position_valid:
        blockers.append("위치 무효")
    if not status.heading_valid:
        blockers.append("헤딩 무효")
    return blockers


def parking_status_summary(value: str) -> str:
    """Convert the selector JSON into a short operator-facing status line."""

    try:
        status = json.loads(value)
    except (TypeError, json.JSONDecodeError):
        return "판정 데이터 형식 오류"
    mission = str(status.get("mission", ""))
    prefix = "T" if mission == "PERP_PARK" else "P" if mission == "PARALLEL_PARK" else "?"
    selected_key = "selected_t" if prefix == "T" else "selected_p"
    selected = status.get(selected_key, 1)
    lock = "선택 고정" if bool(status.get("locked")) else "관찰 중"
    return (
        f"{prefix}1={status.get('option_1', 'UNKNOWN')}  "
        f"{prefix}2={status.get('option_2', 'UNKNOWN')}  "
        f"→ {prefix}{selected}  {lock}"
    )


class MonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("mando_tui")
        self.samples = {
            name: Sample()
            for name in (
                "gnss", "camera_info", "scan", "obstacle_mapper", "local_detour",
                "parking_selection",
                "survey_progress", "survey_result", "preflight", "preflight_report",
                "pose", "feedback", "mission", "safety", "cte", "heading_error",
                "path_command",
            )
        }
        transient = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # The console only displays the newest value.  Depth 1 prevents old
        # high-rate samples from building a backlog behind the live safety and
        # actuator state that decides whether SPACE may start the mission.
        self._subscribe(GnssStatus, "/gnss/status", "gnss", 1)
        self._subscribe(CameraInfo, "/camera/camera_info", "camera_info", 1)
        self._subscribe(LaserScan, "/scan", "scan", qos_profile_sensor_data)
        self._subscribe(
            String,
            "/planning/obstacle_mapper_status",
            "obstacle_mapper",
            1,
        )
        self._subscribe(
            String,
            "/planning/live/local_detour_status",
            "local_detour",
            1,
        )
        self._subscribe(
            String,
            "/mission/parking_selection",
            "parking_selection",
            1,
        )
        self._subscribe(String, "/survey/progress", "survey_progress", 1)
        self._subscribe(String, "/survey/result", "survey_result", transient)
        self._subscribe(Bool, "/system/preflight_ok", "preflight", transient)
        self._subscribe(String, "/system/preflight_report", "preflight_report", transient)
        self._subscribe(PoseWithCovarianceStamped, "/localization/gnss_pose", "pose", 1)
        self._subscribe(VehicleFeedback, "/vehicle/feedback", "feedback", 1)
        self._subscribe(MissionStatus, "/mission/status", "mission", 1)
        self._subscribe(String, "/safety/status", "safety", 1)
        self._subscribe(Float32, "/planning/cross_track_error_m", "cte", 1)
        self._subscribe(Float32, "/planning/heading_error_rad", "heading_error", 1)
        self._subscribe(DriveCommand, "/planning/path_command", "path_command", 1)
        self.toggle_client = self.create_client(Trigger, "/mission/toggle_run")
        self.fault_client = self.create_client(Trigger, "/mission/fault")
        self._publisher_cache: dict[str, tuple[float, bool]] = {}

    def _subscribe(self, message_type, topic: str, key: str, qos) -> None:
        def callback(message) -> None:
            self.samples[key] = Sample(message, time.monotonic())

        self.create_subscription(message_type, topic, callback, qos)

    def clear_live_route_samples(self) -> None:
        for key in self.samples:
            self.samples[key] = Sample()

    def call(self, client, timeout_sec: float = 1.5) -> tuple[bool, str]:
        if not client.wait_for_service(timeout_sec=0.15):
            return False, "mission service가 준비되지 않았습니다"
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done() or future.result() is None:
            return False, "mission service 응답 시간초과"
        response = future.result()
        return bool(response.success), str(response.message)

    def node_names(self) -> set[str]:
        return {name.rsplit("/", 1)[-1] for name, _namespace in self.get_node_names_and_namespaces()}

    def topic_has_publisher(self, topic: str) -> bool:
        now = time.monotonic()
        cached = self._publisher_cache.get(topic)
        if cached is not None and now - cached[0] < 0.25:
            return cached[1]
        available = bool(self.get_publishers_info_by_topic(topic))
        self._publisher_cache[topic] = (now, available)
        return available

    def topic_is_recorded(self, topic: str) -> bool:
        """Report whether the active rosbag recorder subscribes to a topic."""
        return any(
            "rosbag2_recorder" in endpoint.node_name
            or endpoint.node_name in ("recorder", "rosbag_recorder")
            for endpoint in self.get_subscriptions_info_by_topic(topic)
        )


class MandoConsole:
    def __init__(
        self,
        config: ConsoleConfig,
        node: MonitorNode,
        auto_rtk: bool,
        auto_prepare_scenario: int | None = None,
    ) -> None:
        self.config = config
        self.node = node
        self.stack = ProcessSlot()
        self.recorder = ProcessSlot()
        self.mode = "IDLE"
        self.selected_index = 0
        self.variant_indices = {scenario.number: 0 for scenario in config.scenarios}
        self.speeds = {scenario.number: scenario.default_speed for scenario in config.scenarios}
        self.hill_hold_pwms = {
            scenario.number: (
                scenario.variants[0].default_hill_hold_pwm
                if scenario.variants and scenario.variants[0].hill_hold_tuning
                else 0
            )
            for scenario in config.scenarios
        }
        self.active_variant: Variant | None = None
        self.active_route: Route | None = None
        self.active_routes: tuple[Route, ...] = ()
        self.message = "번호로 시나리오를 선택하세요 · MCAP은 필요할 때 B로 시작"
        self.message_is_error = False
        self.last_key_input = "-"
        self.last_numeric_input = "-"
        self.auto_rtk = auto_rtk
        self.auto_prepare_scenario = auto_prepare_scenario
        self._initial_rtk_attempted = False
        self._static_cache: dict[Variant, ReadinessReport] = {}
        self._fault_sent = False
        self._mission_started_at: float | None = None
        self._created_at = time.monotonic()
        self.recording_directory = (
            Path(os.environ.get("HLKU_DATA", config.source_file.parent.parent / "data"))
            / "bags"
        )
        self.datum_output_file: Path | None = None
        self.datum_applied = False

    @property
    def scenario(self) -> Scenario:
        return self.config.scenarios[self.selected_index]

    @property
    def variant(self) -> Variant | None:
        if not self.scenario.variants:
            return None
        index = self.variant_indices[self.scenario.number]
        return self.scenario.variants[index]

    def _set_message(self, text: str, error: bool = False) -> None:
        self.message = text
        self.message_is_error = error

    def readiness(self, variant: Variant) -> ReadinessReport:
        if variant not in self._static_cache:
            self._static_cache[variant] = static_readiness(self.config, variant)
        return self._static_cache[variant]

    def settle_graph(self, seconds: float = 0.6) -> None:
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def conflicts(self) -> list[str]:
        names = self.node.node_names()
        conflicts = []
        if "rosbag2_player" in names:
            conflicts.append("rosbag 재생기")
        expected = set()
        if self.stack.running and self.mode in ("RTK", "ROUTE", "DATUM"):
            expected.update(("um982_serial", "ntrip_client"))
        if self.stack.running and self.mode == "ROUTE":
            expected.update(("mission_manager", "nucleo_serial_bridge"))
        for name in ("um982_serial", "ntrip_client", "mission_manager", "nucleo_serial_bridge"):
            if name in names and name not in expected:
                conflicts.append(name)
        return conflicts

    def start_rtk(self) -> None:
        conflicts = self.conflicts()
        if conflicts:
            self._set_message("기존 ROS 프로세스 종료 필요: " + ", ".join(conflicts), True)
            return
        self.stack.start(build_rtk_command(self.config), "rtk_check", self.config.log_directory)
        self.mode = "RTK"
        self.active_variant = None
        self.active_route = None
        self.active_routes = ()
        self.node.clear_live_route_samples()
        self._fault_sent = False
        self._mission_started_at = None
        self._set_message("RTK 확인 시작: FIXED/헤딩/보정나이를 확인하세요")

    def enforce_conflicts(self) -> None:
        conflicts = self.conflicts()
        if conflicts and self.stack.running:
            text = "외부 ROS 입력 충돌 감지: " + ", ".join(conflicts)
            self.stop_stack(fault=True)
            self._set_message(text + "; 스택을 정지했습니다", True)

    def stop_stack(self, fault: bool = False) -> None:
        if fault and self.mode == "ROUTE" and self.stack.running:
            self.node.call(self.node.fault_client, timeout_sec=0.7)
        self.stack.stop()
        self.mode = "IDLE"
        self.active_variant = None
        self.active_route = None
        self.active_routes = ()
        self._fault_sent = False
        self._mission_started_at = None

    def start_recording(self) -> None:
        if self.recorder.running:
            return
        if "rosbag2_recorder" in self.node.node_names():
            self._set_message(
                "이미 다른 MCAP 기록기가 실행 중입니다; 중복 녹화를 시작하지 않습니다",
                True,
            )
            return
        name = (
            f"mando_s{self.scenario.number:02d}_"
            + datetime.now().strftime("%Y%m%d_%H%M%S")
        )
        self.recorder.start(
            [str(self.config.recording_script), name],
            name,
            self.config.log_directory,
        )
        self._set_message(f"MCAP 선택 기록 시작: {name} · B를 다시 누르면 종료")

    def toggle_recording(self) -> None:
        if self.recorder.running:
            saved = self.recorder.label
            self.recorder.stop()
            self._set_message(f"MCAP 기록 종료: {saved}")
            return
        self.start_recording()

    def recording_checks(self) -> tuple[bool, bool, bool, bool]:
        """Return MCAP, GNSS, camera and raw LiDAR recording readiness."""
        bag_path = self.recording_directory / self.recorder.label
        mcap_ready = bool(
            self.recorder.running
            and bag_path.is_dir()
            and next(bag_path.glob("*.mcap"), None) is not None
        )
        gnss_ready = bool(
            self.recorder.running
            and all(
                self.node.topic_is_recorded(topic)
                for topic in ("/gnss/status", "/gnss/fix")
            )
        )
        camera_ready = bool(
            self.recorder.running
            and self.node.samples["camera_info"].fresh(FRESH_CAMERA_SEC)
            and all(
                self.node.topic_has_publisher(topic)
                and self.node.topic_is_recorded(topic)
                for topic in ("/camera/image_raw", "/camera/camera_info")
            )
        )
        lidar_ready = bool(
            self.recorder.running
            and self.node.samples["scan"].fresh(FRESH_FAST_SEC)
            and self.node.topic_has_publisher("/scan")
            and self.node.topic_is_recorded("/scan")
        )
        return mcap_ready, gnss_ready, camera_ready, lidar_ready

    def select_number(self, number: int) -> None:
        self.last_numeric_input = f"{number} (시나리오 번호)"
        if self.mode in ("ROUTE", "DATUM"):
            self._set_message("먼저 S로 현재 주행 스택을 종료하세요", True)
            return
        for index, scenario in enumerate(self.config.scenarios):
            if scenario.number == number:
                self.selected_index = index
                self._set_message(f"{number}. {scenario.name} 선택")
                return
        self._set_message(f"시나리오 {number}은 설정되어 있지 않습니다", True)

    def change_variant(self, step: int) -> None:
        if self.mode == "ROUTE" or not self.scenario.variants:
            return
        current = self.variant_indices[self.scenario.number]
        self.variant_indices[self.scenario.number] = (current + step) % len(self.scenario.variants)
        self._set_message(f"구간 선택: {variant_compact_summary(self.variant)}")

    def set_speed(self, value: float) -> None:
        scenario = self.scenario
        if scenario.kind == "finger":
            minimum, maximum, unit = 1.0, float(scenario.maximum_pwm), "PWM/100"
            input_context = "손가락 PWM 증감"
        elif scenario.kind == "datum":
            minimum, maximum, unit = 10.0, 3600.0, "초"
            input_context = "측량시간 sec"
        else:
            minimum, maximum, unit = 0.0, 100.0, "PWM/100"
            input_context = "경로 구동 PWM"
        self.last_numeric_input = f"{value:g} ({input_context})"
        if not math.isfinite(value) or not minimum <= value <= maximum:
            self._set_message(f"허용 범위는 {minimum:g}~{maximum:g} {unit}입니다", True)
            return
        if scenario.kind == "route" and not float(value).is_integer():
            self._set_message("경로 PWM은 0~100의 정수로 입력하세요", True)
            return
        if scenario.kind == "route" and self.mode == "ROUTE":
            self._set_message("주행 중에는 PWM을 바꿀 수 없습니다. S로 스택을 종료한 뒤 V로 설정하세요", True)
            return
        self.speeds[scenario.number] = value
        if scenario.kind == "finger":
            self._set_message(
                f"손가락 키당 PWM 증감: {round(value):d}/100 ({value:.0f}%) · "
                "W는 더하고 S는 뺍니다"
            )
        elif scenario.kind == "datum":
            self._set_message(f"{scenario.name} 설정값: {value:g} {unit}")
        else:
            self._set_message(f"{scenario.name} 구동 PWM: {int(value)}/100 · 전진/후진 공통")

    def ensure_automatic_route(
        self, variant: Variant, force_current_pose: bool = False
    ) -> bool:
        if not variant.auto_place_at_current_pose:
            return True
        reason = automatic_placement_reason(self.config, variant)
        if reason is None and not force_current_pose:
            return True
        if reason is None:
            reason = "R 입력 · 현 위치 재배치"
        route_label = f"{self.scenario.name} · {variant.name}"
        if not self.stack.running or self.mode != "RTK":
            if self.mode == "IDLE":
                self.start_rtk()
                if self.stack.running and self.mode == "RTK":
                    self._set_message(
                        f"{route_label} 자동 배치 대기: RTK FIXED와 ENU 위치·방향이 "
                        "표시되면 R을 다시 누르세요"
                    )
            else:
                self._set_message(
                    f"{route_label} 자동 배치 전에 S로 현재 스택을 끝내고 G로 RTK를 "
                    "시작하세요",
                    True,
                )
            return False
        status_sample = self.node.samples["gnss"]
        blockers = gnss_blockers(
            status_sample.value,
            status_sample.fresh(FRESH_GNSS_SEC),
        )
        pose_sample = self.node.samples["pose"]
        if blockers or pose_sample.value is None or not pose_sample.fresh(FRESH_POSE_SEC):
            waiting = blockers[:2]
            if pose_sample.value is None or not pose_sample.fresh(FRESH_POSE_SEC):
                waiting.append("ENU 위치·방향 대기")
            self._set_message(
                f"{route_label} 자동 배치 대기({reason}): "
                + " / ".join(waiting)
                + "; 수신 후 R을 다시 누르세요"
            )
            return False
        try:
            route = place_variant_at_pose(
                self.config,
                variant,
                pose_sample.value,
                0.30,
            )
        except (OSError, ValueError, KeyError, yaml.YAMLError) as error:
            self._set_message(f"{route_label} 자동 생성 실패: {error}", True)
            return False
        self._static_cache.pop(variant, None)
        x_m, y_m, heading_deg = pose_xy_heading_deg(pose_sample.value)
        self._set_message(
            f"{route_label} 자동 배치 완료: {route.waypoints[-1].s_m:.1f}m, "
            f"시작 x={x_m:.2f} y={y_m:.2f} heading={heading_deg:.1f}deg"
        )
        return True

    def prepare_selected(self, stdscr) -> None:
        if self.scenario.kind == "finger":
            self.run_finger(stdscr)
            return
        if self.scenario.kind == "datum":
            self.start_datum_survey()
            return
        variant = self.variant
        assert variant is not None
        # Every R press places the selected source slice at the vehicle's current
        # RTK position and heading. This lets one campus area host many original
        # course segments and also makes repeated runs independent.
        if not self.ensure_automatic_route(variant, force_current_pose=True):
            return
        report = self.readiness(variant)
        if not report.ok:
            self._set_message("정적 준비 실패: " + report.errors[0], True)
            return
        conflicts = self.conflicts()
        if conflicts:
            self._set_message("기존 ROS 프로세스 종료 필요: " + ", ".join(conflicts), True)
            return
        self.stop_stack(fault=True)
        self.settle_graph()
        conflicts = self.conflicts()
        if conflicts:
            self._set_message("종료되지 않은 ROS 프로세스: " + ", ".join(conflicts), True)
            return
        self.node.clear_live_route_samples()
        self.active_variant = variant
        self.active_route, geometry_errors = route_geometry(variant)
        if geometry_errors or self.active_route is None:
            self._set_message("경로 검사 실패: " + geometry_errors[0], True)
            return
        self.active_routes = (self.active_route,)
        if (
            variant.enable_parking_selection
            and variant.source_route is not None
            and variant.source_variant_id
        ):
            try:
                family = ParkingRouteFamily.from_placed_route(
                    self.active_route,
                    variant.source_route,
                    variant.source_variant_id,
                    variant.source_start_s_m,
                    variant.source_length_m,
                    0.30,
                )
            except (OSError, ValueError, KeyError) as error:
                self._set_message(f"재시작 경로군 검사 실패: {error}", True)
                return
            self.active_routes = (
                self.active_route,
                *family.routes.values(),
            )
        command = build_route_command(
            self.config,
            variant,
            int(self.speeds[self.scenario.number]),
            self.hill_hold_pwms[self.scenario.number],
        )
        self.stack.start(command, f"route_{self.scenario.number}", self.config.log_directory)
        self.mode = "ROUTE"
        self._fault_sent = False
        self._mission_started_at = None
        recording = "MCAP 기록 중" if self.recorder.running else "MCAP 필요 시 B"
        self._set_message(
            f"준비 중: RTK·차량 상태가 ✓이면 Space로 출발 · {recording}"
        )

    def start_datum_survey(self) -> None:
        conflicts = self.conflicts()
        if conflicts:
            self._set_message(
                "기존 ROS 프로세스 종료 필요: " + ", ".join(conflicts), True
            )
            return
        self.stop_stack(fault=True)
        self.settle_graph()
        conflicts = self.conflicts()
        if conflicts:
            self._set_message(
                "종료되지 않은 ROS 프로세스: " + ", ".join(conflicts), True
            )
            return
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.datum_output_file = (
            self.recording_directory.parent
            / "datum"
            / f"konkuk_datum_{stamp}.yaml"
        )
        self.datum_applied = False
        self.node.clear_live_route_samples()
        duration = float(self.speeds[self.scenario.number])
        command = build_datum_command(
            self.config, self.scenario, duration, self.datum_output_file
        )
        self.stack.start(command, "konkuk_datum", self.config.log_directory)
        self.mode = "DATUM"
        self._set_message(
            f"건대 datum 측량 시작: {duration:.0f}초 동안 안테나를 기준점에 고정하세요"
            " · MCAP 필요 시 B"
        )

    def apply_datum(self) -> None:
        if self.datum_output_file is None or not self.datum_output_file.is_file():
            self._set_message("적용할 완료된 건대 datum 파일이 없습니다", True)
            return
        try:
            backup = apply_datum_to_gps_config(
                self.datum_output_file, self.config.gps_config
            )
        except (OSError, ValueError, yaml.YAMLError) as error:
            self._set_message(f"datum 적용 실패: {error}", True)
            return
        self.datum_applied = True
        self._static_cache.clear()
        self._set_message(
            f"건대 datum 적용 완료; 기존 GPS 설정 백업: {backup.name}"
        )

    def run_finger(self, stdscr) -> None:
        names = self.node.node_names()
        if "nucleo_serial_bridge" in names or "keyboard_teleop" in names:
            self._set_message(
                "기존 NUCLEO/손가락 주행 프로세스를 먼저 종료하세요",
                True,
            )
            return

        pwm_step = int(round(self.speeds[self.scenario.number]))
        initial_pwm = (
            self.scenario.finger_initial_pwm
            if self.scenario.finger_initial_pwm >= 0
            else pwm_step
        )
        curses.def_prog_mode()
        curses.endwin()
        print(
            "\n손가락 주행: GNSS 없이 실행 · "
            f"키당 PWM 증감 {pwm_step}/100 · "
            f"MCAP {'기록 중' if self.recorder.running else '꺼짐(B를 먼저 누르면 기록)'} · "
            f"초기 구동 0 PWM · W=+{pwm_step} 누적·유지 S=-{pwm_step} 누적·유지 · "
            "A/D=조향 한 단계씩 누적·고정 C=중앙 SPACE=정지 X=FAULT Q=TUI복귀"
        )
        try:
            result = subprocess.run(
                [
                    str(self.config.finger_drive_script),
                    "--max-pwm",
                    str(self.scenario.maximum_pwm),
                    "--initial-pwm",
                    str(initial_pwm),
                    "--step-pwm",
                    str(pwm_step),
                ],
                check=False,
            )
            self._set_message(
                f"손가락 주행 종료(code={result.returncode}); Q를 다시 누르면 TUI 종료"
            )
        finally:
            curses.reset_prog_mode()
            stdscr.refresh()

    def route_start_blockers(self) -> list[str]:
        blockers: list[str] = []
        if self.mode != "ROUTE" or not self.stack.running or self.active_variant is None or self.active_route is None:
            return ["주행 스택 미준비"]
        variant = self.active_variant
        relaxed = variant.preflight == "relaxed"
        if self.config.require_recording:
            mcap_ready, gnss_recorded, camera_recorded, _lidar_recorded = (
                self.recording_checks()
            )
            if not mcap_ready:
                blockers.append("MCAP 파일 미생성")
            if not gnss_recorded:
                blockers.append("GNSS 기록 미확인")
            if not camera_recorded:
                blockers.append("카메라 기록 미확인")
        blockers.extend("ROS 충돌: " + item for item in self.conflicts())
        blockers.extend(gnss_blockers(self.node.samples["gnss"].value, self.node.samples["gnss"].fresh(FRESH_GNSS_SEC)))
        if variant.preflight == "full":
            sample = self.node.samples["preflight"]
            if not sample.fresh(2.0) or sample.value is None or not sample.value.data:
                blockers.append("정적 preflight 실패/미수신")
        pose_sample = self.node.samples["pose"]
        if not pose_sample.fresh(FRESH_POSE_SEC) or pose_sample.value is None:
            # A live path command proves that path_tracker has already
            # received ENU pose/heading and acquired a forward route point.
            if not self.node.topic_has_publisher("/planning/path_command"):
                blockers.append("ENU pose 미수신")
        elif not relaxed:
            match = alignment_match(self.active_routes, pose_sample.value)
            if match is None:
                blockers.append("차량 전방 재시작 경로 없음")
            else:
                if (
                    variant.start_position_tolerance_m > 0.0
                    and match.distance_m > variant.start_position_tolerance_m
                ):
                    blockers.append(f"경로 거리 {match.distance_m:.2f}m")
                heading = abs(math.degrees(match.heading_error_rad))
                if heading > variant.start_heading_tolerance_deg:
                    blockers.append(f"경로방향 오차 {heading:.1f}deg")
        feedback = self.node.samples["feedback"]
        if not feedback.fresh(FRESH_FAST_SEC) or feedback.value is None:
            if not self.node.topic_has_publisher("/vehicle/feedback"):
                blockers.append("차량 feedback 미수신")
        elif feedback.value.fault_flags:
            blockers.append(f"MCU fault 0x{feedback.value.fault_flags:08x}")
        for key, label, topic in (
            ("mission", "mission", "/mission/status"),
            ("path_command", "조향 명령", "/planning/path_command"),
            ("safety", "safety", "/safety/status"),
        ):
            if (
                not self.node.samples[key].fresh(FRESH_FAST_SEC)
                and not self.node.topic_has_publisher(topic)
            ):
                blockers.append(f"{label} 미수신")
        # safety_supervisor remains the authoritative actuator gate and keeps
        # brake asserted whenever a live input is invalid.  The TUI only
        # requires its publisher above; it must not latch a start rejection
        # from a delayed status string received during ROS graph startup.
        mission = self.node.samples["mission"].value
        if mission is not None and mission.state not in (MissionStatus.STATE_INIT, MissionStatus.STATE_READY):
            blockers.append(f"mission 상태 {mission.state_name}")
        return list(dict.fromkeys(blockers))

    def toggle_run(self) -> None:
        mission = self.node.samples["mission"].value
        starting = mission is None or mission.state == MissionStatus.STATE_INIT
        if starting:
            blockers = self.route_start_blockers()
            if blockers:
                self._set_message("출발 차단: " + " | ".join(blockers[:3]), True)
                return
        ok, message = self.node.call(self.node.toggle_client)
        if ok and starting:
            self._mission_started_at = time.monotonic()
        self._set_message(message, not ok)

    def emergency_stop(self) -> None:
        if self.mode == "ROUTE" and self.stack.running:
            ok, message = self.node.call(self.node.fault_client, timeout_sec=0.7)
            self._fault_sent = ok
            self._set_message("비상정지: " + message, not ok)
        else:
            self._set_message("활성 주행이 없습니다")

    def shutdown(self) -> None:
        self.stop_stack(fault=True)
        self.recorder.stop()

    def runtime_blockers(self) -> list[str]:
        """Return only conditions that justify a permanent mission fault.

        Freshness and transient safety gates are enforced continuously by the
        safety supervisor, which brakes while an input is unavailable and can
        resume after it recovers.  The curses UI can briefly miss callbacks
        while redrawing or while RViz starts, so its local sample age must not
        turn a healthy, recoverable stop into a latched mission fault.
        """
        blockers: list[str] = []
        blockers.extend("ROS 충돌: " + item for item in self.conflicts())
        feedback = self.node.samples["feedback"]
        if (
            feedback.fresh(FRESH_FAST_SEC)
            and feedback.value is not None
            and feedback.value.fault_flags
        ):
            blockers.append(f"MCU fault 0x{feedback.value.fault_flags:08x}")
        return list(dict.fromkeys(blockers))

    def trip_on_runtime_failure(self) -> None:
        if self.mode != "ROUTE" or self._fault_sent:
            return
        if (
            self._mission_started_at is None
            or time.monotonic() - self._mission_started_at < 1.0
        ):
            return
        mission = self.node.samples["mission"].value
        if mission is None or mission.state in (
            MissionStatus.STATE_INIT,
            MissionStatus.STATE_READY,
            MissionStatus.STATE_FAULT,
            MissionStatus.STATE_FINISH,
        ):
            return
        blockers = self.runtime_blockers()
        if blockers:
            ok, response = self.node.call(self.node.fault_client, timeout_sec=0.7)
            self._fault_sent = ok
            suffix = "" if ok else f" ({response})"
            self._set_message(
                "주행 중 안전입력 상실, fault latch: " + blockers[0] + suffix,
                True,
            )

    def check_children(self) -> None:
        if self.stack.child is not None and not self.stack.running:
            code = self.stack.child.returncode
            self.stack.stop()
            self.mode = "IDLE"
            self._set_message(f"ROS launch가 종료되었습니다(code={code}); 로그를 확인하세요", True)
        if self.recorder.child is not None and not self.recorder.running:
            code = self.recorder.child.returncode
            self.recorder.stop()
            self._set_message(f"MCAP 기록기가 종료되었습니다(code={code})", True)

    @staticmethod
    def _put(
        stdscr,
        row: int,
        text: str,
        style: int = 0,
        column: int = 0,
    ) -> int:
        height, width = stdscr.getmaxyx()
        if 0 <= row < height and 0 <= column < width:
            try:
                stdscr.addnstr(
                    row,
                    column,
                    text,
                    max(0, width - column - 1),
                    style,
                )
            except curses.error:
                pass
        return row + 1

    def draw(self, stdscr) -> None:
        stdscr.erase()
        height, width = stdscr.getmaxyx()
        if height < 24 or width < 78:
            self._put(stdscr, 0, "터미널을 78x24 이상으로 늘려주세요", curses.A_BOLD)
            stdscr.refresh()
            return
        finger_selected = self.scenario.kind == "finger"
        stack_text = f"{status_mark('ok' if self.stack.running else 'wait')} {self.mode}"
        mcap_ready, gnss_recorded, camera_recorded, lidar_recorded = (
            self.recording_checks()
        )
        record_text = (
            f"{status_mark('ok' if mcap_ready else 'wait')} {self.recorder.label}"
            if self.recorder.running
            else f"{status_mark('wait' if self.config.require_recording else 'optional')} 꺼짐"
        )
        nodes = self.node.node_names()
        foxglove_text = (
            f"{status_mark('ok')} 연결"
            if "foxglove_bridge" in nodes
            else f"{status_mark('optional')} 미연결"
        )
        conflicts = self.conflicts()
        conflict_state = "optional" if finger_selected else "block"
        conflict_text = (
            f"{status_mark(conflict_state)} 충돌: " + ", ".join(conflicts)
            if conflicts
            else f"{status_mark('ok')} 충돌: 없음"
        )
        if conflicts and finger_selected:
            conflict_text += " (손가락 주행에는 영향 없음)"
        nucleo_connected = (
            Path("/tmp/nucleo-control").exists()
            and Path("/tmp/nucleo-lidar").exists()
        )
        nucleo_text = (
            f"{status_mark('ok')} NUCLEO 연결"
            if nucleo_connected
            else f"{status_mark('wait')} NUCLEO 미연결"
        )

        panel_width = min(width - 2, 140)
        panel_column = max(0, (width - panel_width) // 2)
        half_left = (panel_width - 3) // 2
        half_right = panel_width - 3 - half_left
        half_widths = (half_left, half_right)
        full_widths = (panel_width - 2,)
        selected_value = self.speeds[self.scenario.number]
        if self.scenario.kind == "route":
            current_setting = f"{selected_value:.0f}/100 PWM"
        elif self.scenario.kind == "finger":
            current_setting = (
                f"{selected_value:.0f}/100 ({selected_value:.0f}%)"
            )
        else:
            current_setting = f"{selected_value:.0f} sec"
        if self.recorder.running:
            recording_text = (
                f"MCAP {status_mark('ok' if mcap_ready else 'wait')}  "
                f"GNSS {status_mark('ok' if gnss_recorded else 'wait')}  "
                f"카메라 {status_mark('ok' if camera_recorded else 'wait')}  "
                f"LiDAR /scan {status_mark('ok' if lidar_recorded else 'wait')}"
            )
        else:
            idle_record_state = (
                "wait" if self.config.require_recording else "optional"
            )
            recording_text = (
                f"MCAP {status_mark(idle_record_state)} B로 시작  "
                f"GNSS {status_mark(idle_record_state)}  "
                f"카메라 {status_mark(idle_record_state)}  "
                f"LiDAR /scan {status_mark(idle_record_state)}"
            )

        row = self._put(
            stdscr, 0, table_border(full_widths), column=panel_column
        )
        row = self._put(
            stdscr,
            row,
            table_row(
                (self.config.title,),
                full_widths,
                ("center",),
            ),
            curses.A_BOLD,
            panel_column,
        )
        row = self._put(
            stdscr, row, table_border(half_widths), column=panel_column
        )
        for left, right in (
            (f"  스택       {stack_text}", f"  마지막 키     {self.last_key_input}"),
            (f"  MCAP       {record_text}", f"  최근 숫자     {self.last_numeric_input}"),
            (f"  FOXGLOVE   {foxglove_text}", f"  선택 번호     {self.scenario.number}. {self.scenario.name}"),
            (f"  기록       {recording_text}", f"  현재 설정     {current_setting}"),
        ):
            row = self._put(
                stdscr,
                row,
                table_row((left, right), half_widths),
                column=panel_column,
            )
        row = self._put(
            stdscr, row, table_border(full_widths), column=panel_column
        )
        row = self._put(
            stdscr,
            row,
            table_row((f"  {conflict_text}  |  {nucleo_text}",), full_widths),
            curses.A_BOLD if conflicts else 0,
            panel_column,
        )
        row = self._put(
            stdscr, row, table_border(full_widths), column=panel_column
        )

        number_width = 7
        name_width = 18
        value_width = 25
        detail_width = panel_width - 5 - number_width - name_width - value_width
        scenario_widths = (
            number_width,
            name_width,
            value_width,
            detail_width,
        )
        row = self._put(
            stdscr,
            row,
            table_row(
                ("시나리오 선택 · 숫자키 1~9 · [ / ] 세부구간 변경",),
                full_widths,
                ("center",),
            ),
            curses.A_BOLD,
            panel_column,
        )
        row = self._put(
            stdscr, row, table_border(scenario_widths), column=panel_column
        )
        row = self._put(
            stdscr,
            row,
            table_row(
                (" 번호", " 시나리오", " 현재 입력값", " 구간 / 동작"),
                scenario_widths,
            ),
            curses.A_BOLD,
            panel_column,
        )
        row = self._put(
            stdscr, row, table_border(scenario_widths), column=panel_column
        )
        for scenario in self.config.scenarios:
            selected = scenario.number == self.scenario.number
            if scenario.kind == "route":
                value = f"{self.speeds[scenario.number]:.0f}/100 PWM"
                selected_variant = scenario.variants[
                    self.variant_indices[scenario.number]
                ]
                detail = variant_compact_summary(selected_variant)
                if selected_variant.hill_hold_tuning:
                    detail += (
                        " · 경사 PWM "
                        f"오르막 {selected_variant.default_hill_approach_pwm}"
                        f" / 정지 {self.hill_hold_pwms[scenario.number]}"
                        f" / 정지후 {selected_variant.default_hill_post_stop_pwm}"
                    )
            elif scenario.kind == "finger":
                pwm = self.speeds[scenario.number]
                initial_pwm = (
                    scenario.finger_initial_pwm
                    if scenario.finger_initial_pwm >= 0
                    else int(round(pwm))
                )
                value = f"키당 ±{pwm:.0f}/100"
                detail = (
                    f"시작 {initial_pwm}/100 · "
                    "W/S: PWM 누적 · A/D: 조향 누적·고정"
                )
            else:
                value = f"{self.speeds[scenario.number]:.0f} sec"
                detail = f"min {scenario.survey_minimum_samples} samples"
            row = self._put(
                stdscr,
                row,
                table_row(
                    (
                        f" {'>' if selected else ' '} {scenario.number}",
                        f" {scenario.name}",
                        f" {value}",
                        f" {detail}",
                    ),
                    scenario_widths,
                ),
                curses.A_REVERSE if selected else 0,
                panel_column,
            )
        row = self._put(
            stdscr, row, table_border(scenario_widths), column=panel_column
        )
        variant = self.variant
        if self.scenario.description:
            row = self._put(
                stdscr,
                row,
                table_row((f"  {self.scenario.description}",), full_widths),
                curses.A_BOLD,
                panel_column,
            )
        if variant is not None:
            variant_number = self.variant_indices[self.scenario.number] + 1
            variant_count = len(self.scenario.variants)
            row = self._put(
                stdscr,
                row,
                table_row(
                    (
                        f"현재 선택 · {self.scenario.number}번 {self.scenario.name} · "
                        f"세부구간 {variant_number}/{variant_count}",
                    ),
                    full_widths,
                    ("center",),
                ),
                curses.A_BOLD,
                panel_column,
            )
            row = self._put(
                stdscr, row, table_border(full_widths), column=panel_column
            )
            for detail_line in variant_detail_lines(variant):
                row = self._put(
                    stdscr,
                    row,
                    table_row((f"  {detail_line}",), full_widths),
                    curses.A_BOLD,
                    panel_column,
                )
            row = self._put(
                stdscr, row, table_border(full_widths), column=panel_column
            )
        if finger_selected:
            pwm = self.speeds[self.scenario.number]
            row = self._put(
                stdscr,
                row,
                f"{status_mark('info')} 현재 입력 설정: "
                f"W: -{pwm:.0f}→0→+{pwm:.0f}  "
                f"S: +{pwm:.0f}→0→-{pwm:.0f}  "
                "A=최대좌고정 -0.48rad  D=최대우고정 +0.48rad  C=중앙고정",
                curses.A_BOLD,
            )
        if variant is not None:
            placement_reason = automatic_placement_reason(self.config, variant)
            if placement_reason is not None:
                static_text = (
                    f"{status_mark('wait')} 현재 RTK pose로 자동 생성 예정"
                    f" ({placement_reason})"
                )
            else:
                report = self.readiness(variant)
                static_text = (
                    f"{status_mark('ok')} 완료"
                    if report.ok
                    else f"{status_mark('block')} {report.errors[0]}"
                )
            row = self._put(
                stdscr,
                row,
                table_row(
                    (
                        f"  생성 파일  {variant.route_file.name}  |  "
                        f"preflight={variant.preflight}  |  정적검사={static_text}",
                    ),
                    full_widths,
                ),
                column=panel_column,
            )
        row = self._put(stdscr, row, "-" * (width - 1))
        gnss = self.node.samples["gnss"].value
        blockers = gnss_blockers(gnss, self.node.samples["gnss"].fresh(FRESH_GNSS_SEC))
        if finger_selected:
            gnss_text = f"{status_mark('optional')} RTK: 손가락 주행에는 필요 없음"
        elif gnss is None:
            gnss_text = f"{status_mark('wait')} RTK: 수신 대기"
        else:
            fix = "FIXED" if gnss.fix_type == GnssStatus.FIX_RTK_FIXED else f"type={gnss.fix_type}"
            marker = status_mark("ok" if not blockers else "block")
            gnss_text = f"{marker} RTK: {fix}  sat={gnss.satellites}  HDOP={gnss.hdop:.2f}  RTCM age={gnss.correction_age_sec:.1f}s  heading={'OK' if gnss.heading_valid else 'WAIT'}"
        row = self._put(stdscr, row, gnss_text, curses.A_BOLD if blockers else 0)
        if self.mode == "ROUTE":
            preflight_sample = self.node.samples["preflight"]
            preflight = preflight_sample.value
            if self.active_variant and self.active_variant.preflight == "relaxed":
                preflight_text = f"{status_mark('optional')} 생략"
            elif self.active_variant and self.active_variant.preflight in ("supervised", "field"):
                preflight_text = f"{status_mark('ok')} TUI {self.active_variant.preflight}"
            elif preflight is None or not preflight_sample.fresh(2.0):
                preflight_text = f"{status_mark('wait')} 미수신"
            elif preflight.data:
                preflight_text = f"{status_mark('ok')} 완료"
            else:
                preflight_text = f"{status_mark('block')} 실패"
            feedback_sample = self.node.samples["feedback"]
            mission_sample = self.node.samples["mission"]
            safety_sample = self.node.samples["safety"]
            feedback = feedback_sample.value
            mission = mission_sample.value
            safety = safety_sample.value
            if feedback is None or not feedback_sample.fresh(FRESH_FAST_SEC):
                feedback_text = (
                    f"{status_mark('ok')} 토픽 연결"
                    if self.node.topic_has_publisher("/vehicle/feedback")
                    else f"{status_mark('wait')} 미수신"
                )
            elif feedback.fault_flags:
                feedback_text = f"{status_mark('block')} fault"
            else:
                feedback_text = f"{status_mark('ok')} 정상"
            if safety is None or not safety_sample.fresh(FRESH_FAST_SEC):
                safety_text = (
                    f"{status_mark('ok')} 토픽 연결"
                    if self.node.topic_has_publisher("/safety/status")
                    else f"{status_mark('wait')} 미수신"
                )
            else:
                reasons = {
                    item
                    for item in safety.data.split(",")
                    if item and item != "OK"
                }
                safety_ready = not (reasons - SAFE_START_SAFETY_REASONS)
                safety_text = (
                    f"{status_mark('ok' if safety_ready else 'info')} "
                    f"{safety.data}"
                )
            row = self._put(
                stdscr,
                row,
                f"준비: preflight={preflight_text}  "
                f"feedback={feedback_text}  safety={safety_text}",
            )
            pose_sample = self.node.samples["pose"]
            pose = pose_sample.value
            if (
                pose is not None
                and pose_sample.fresh(FRESH_POSE_SEC)
                and self.active_route is not None
            ):
                match = alignment_match(self.active_routes, pose)
                p = pose.pose.pose.position
                distance = match.distance_m if match is not None else math.inf
                heading = (
                    abs(math.degrees(match.heading_error_rad))
                    if match is not None
                    else math.inf
                )
                distance_allowed = (
                    self.active_variant is not None
                    and (
                        self.active_variant.start_position_tolerance_m <= 0.0
                        or distance
                        <= self.active_variant.start_position_tolerance_m
                    )
                )
                aligned = (
                    self.active_variant is not None
                    and match is not None
                    and distance_allowed
                    and heading
                    <= self.active_variant.start_heading_tolerance_deg
                )
                alignment_state = (
                    "info"
                    if self.active_variant
                    and self.active_variant.preflight == "relaxed"
                    else "ok" if aligned else "block"
                )
                row = self._put(
                    stdscr,
                    row,
                    f"{status_mark(alignment_state)} 정렬: "
                    f"x={p.x:.2f} y={p.y:.2f}  경로거리={distance:.2f}m  "
                    f"방향오차={heading:.1f}deg"
                    + (
                        "  거리제한=없음"
                        if self.active_variant is not None
                        and self.active_variant.start_position_tolerance_m <= 0.0
                        else ""
                    )
                    + (
                        f"  재시작 s={match.route_s_m:.1f}m "
                        f"dir={match.direction:+d}"
                        if match is not None
                        else "  전방 경로 없음"
                    ),
                )
            else:
                tracker_live = self.node.topic_has_publisher(
                    "/planning/path_command"
                )
                row = self._put(
                    stdscr,
                    row,
                    (
                        f"{status_mark('ok')} 정렬: path tracker가 ENU 경로를 획득함"
                        if tracker_live
                        else f"{status_mark('wait')} 정렬: ENU pose 대기"
                    ),
                )
            cte_sample = self.node.samples["cte"]
            heading_sample = self.node.samples["heading_error"]
            command_sample = self.node.samples["path_command"]
            cte = cte_sample.value
            heading_error = heading_sample.value
            command = command_sample.value
            tracking_ready = bool(
                mission
                and mission_sample.fresh(FRESH_FAST_SEC)
                and cte
                and cte_sample.fresh(FRESH_FAST_SEC)
                and heading_error
                and heading_sample.fresh(FRESH_FAST_SEC)
                and command
                and command_sample.fresh(FRESH_FAST_SEC)
            )
            row = self._put(
                stdscr,
                row,
                (
                    f"{status_mark('ok')} 추종: s={mission.route_s_m:.2f}m "
                    f"CTE={cte.data:.3f}m "
                    f"heading_err={math.degrees(heading_error.data):.1f}deg "
                    f"steer_cmd={command.steering_angle_rad:.3f}rad"
                    if tracking_ready
                    else (
                        f"{status_mark('ok')} 추종: 조향 명령 토픽 연결"
                        if self.node.topic_has_publisher("/planning/path_command")
                        else f"{status_mark('wait')} 추종: 데이터 대기"
                    )
                ),
            )
            scan_ready = self.node.samples["scan"].fresh(0.7)
            mapper_sample = self.node.samples["obstacle_mapper"]
            detour_sample = self.node.samples["local_detour"]
            mapper = (
                mapper_sample.value.data
                if mapper_sample.value is not None and mapper_sample.fresh(1.2)
                else "대기"
            )
            detour = (
                detour_sample.value.data
                if detour_sample.value is not None and detour_sample.fresh(1.2)
                else "대기"
            )
            lidar_state = "ok" if scan_ready else "wait"
            if mapper.startswith(("INVALID", "SCAN_FRAME", "STAMP_")):
                lidar_state = "block"
            elif mapper == "GEOMETRY_UNVERIFIED":
                lidar_state = "info"
            row = self._put(
                stdscr,
                row,
                f"{status_mark(lidar_state)} LiDAR: "
                f"/scan={'수신' if scan_ready else '대기'}  "
                f"map={mapper}  detour={detour}  "
                "회피조향="
                f"{'ON' if self.active_variant is not None and variant_detour_enabled(self.config, self.active_variant) else 'OFF'}",
            )
            if (
                self.active_variant is not None
                and self.active_variant.enable_parking_selection
            ):
                parking_sample = self.node.samples["parking_selection"]
                if (
                    parking_sample.value is not None
                    and parking_sample.fresh(1.2)
                ):
                    parking_text = parking_status_summary(
                        parking_sample.value.data
                    )
                    parking_state = "ok"
                else:
                    parking_text = "T/P 공간 판정 대기"
                    parking_state = "wait"
                row = self._put(
                    stdscr,
                    row,
                    f"{status_mark(parking_state)} 주차판정: {parking_text}",
                )
            if feedback is not None and feedback_sample.fresh(FRESH_FAST_SEC):
                vehicle_state = "ok" if not feedback.fault_flags else "block"
                row = self._put(
                    stdscr,
                    row,
                    f"{status_mark(vehicle_state)} 차량: "
                    f"steer={feedback.steering_angle_rad:.3f}rad "
                    f"target={feedback.steering_target_rad:.3f}rad "
                    f"drive={feedback.applied_drive_duty * 100:+.0f}/100 PWM "
                    f"brake={feedback.brake_active} "
                    f"fault=0x{feedback.fault_flags:08x}",
                )
            start_blockers = self.route_start_blockers()
            row = self._put(
                stdscr,
                row,
                f"{status_mark('ok' if not start_blockers else 'block')} "
                "출발 조건: "
                + (
                    "완료"
                    if not start_blockers
                    else " / ".join(start_blockers[:4])
                ),
                curses.A_BOLD,
            )
        elif self.mode == "DATUM":
            complete = bool(
                self.datum_output_file is not None
                and self.datum_output_file.is_file()
            )
            result = self.node.samples["survey_result"].value
            progress = self.node.samples["survey_progress"].value
            if complete:
                survey_text = result.data if result is not None else str(self.datum_output_file)
                row = self._put(
                    stdscr,
                    row,
                    f"{status_mark('ok')} 건대 datum 측량 완료: {survey_text}",
                    curses.A_BOLD,
                )
                row = self._put(
                    stdscr,
                    row,
                    f"{status_mark('ok' if self.datum_applied else 'wait')} GPS 설정 적용: "
                    + ("완료" if self.datum_applied else "A 키를 누르면 gps_only_route.yaml에 적용"),
                )
            else:
                progress_text = progress.data if progress is not None else "측량 노드 준비 중"
                row = self._put(
                    stdscr,
                    row,
                    f"{status_mark('wait')} 건대 datum: {progress_text}",
                    curses.A_BOLD,
                )
                row = self._put(
                    stdscr,
                    row,
                    "안테나 ANT1을 건대 기준 마킹에 움직이지 않게 고정하세요",
                )
        else:
            if self.scenario.kind == "datum":
                preparation = "R을 누르면 datum 측량을 시작합니다."
            elif self.scenario.kind == "finger":
                preparation = (
                    "R을 누르면 GNSS 확인 없이 손가락 주행을 바로 시작합니다."
                )
            else:
                preparation = (
                    "R을 누르면 브레이크 상태로 경로 주행 스택을 준비합니다."
                )
            row = self._put(
                stdscr,
                row,
                f"{status_mark('wait')} 준비: RTK 확인 중. {preparation}",
            )
        while row < height - 4:
            row += 1
        self._put(stdscr, height - 4, "키: 1~9 선택  [/] 구간  V 경로PWM/손가락증감/측량시간  H 경사정지PWM  R 준비  G RTK")
        self._put(stdscr, height - 3, "    SPACE 출발/일시정지  B MCAP 시작/종료  X 비상정지  S 스택종료  Q 종료")
        self._put(stdscr, height - 2, ("오류: " if self.message_is_error else "상태: ") + self.message, curses.A_BOLD)
        self._put(stdscr, height - 1, f"로그: {self.stack.log_file or '-'}")
        stdscr.refresh()

    def prompt_speed(self, stdscr) -> None:
        scenario = self.scenario
        if scenario.kind == "finger":
            unit = "키당 PWM 증감/100 (1~%d)" % scenario.maximum_pwm
        elif scenario.kind == "datum":
            unit = "측량시간 sec(10~3600)"
        else:
            unit = "구동 PWM/100 (0~100, 전진·후진 공통)"
        height, _width = stdscr.getmaxyx()
        prompt = f"{scenario.name} {unit} 입력: "
        curses.echo()
        stdscr.nodelay(False)
        try:
            stdscr.move(height - 2, 0)
            stdscr.clrtoeol()
            stdscr.addstr(height - 2, 0, prompt)
            raw = stdscr.getstr(
                height - 2, terminal_text_width(prompt), 16
            ).decode("utf-8").strip()
            if raw:
                self.last_numeric_input = f"{raw} (입력)"
                self.set_speed(float(raw))
        except (ValueError, UnicodeDecodeError):
            self._set_message("숫자를 입력하세요", True)
        finally:
            curses.noecho()
            stdscr.nodelay(True)

    def prompt_hill_hold_pwm(self, stdscr) -> None:
        variant = self.variant
        if self.scenario.kind != "route" or variant is None or not variant.hill_hold_tuning:
            self._set_message("선택한 시나리오는 경사 정지 PWM 설정을 사용하지 않습니다", True)
            return
        if self.mode == "ROUTE":
            self._set_message("주행 중에는 바꿀 수 없습니다. S로 종료한 뒤 H로 설정하세요", True)
            return
        height, _width = stdscr.getmaxyx()
        prompt = "경사 정지 유지 PWM/100 입력 (0=구동 OFF, 1~100=전진 유지): "
        curses.echo()
        stdscr.nodelay(False)
        try:
            stdscr.move(height - 2, 0)
            stdscr.clrtoeol()
            stdscr.addstr(height - 2, 0, prompt)
            raw = stdscr.getstr(
                height - 2, terminal_text_width(prompt), 8
            ).decode("utf-8").strip()
            if raw:
                value = _pwm_number(raw, "경사 정지 PWM")
                self.hill_hold_pwms[self.scenario.number] = value
                self.last_numeric_input = f"{value} (경사 정지 PWM)"
                self._set_message(
                    f"경사 HILL_STOP 정지 유지값: {value}/100"
                    + (" · 구동 OFF/브레이크" if value == 0 else " · 전진 PWM 유지")
                )
        except (ValueError, UnicodeDecodeError):
            self._set_message("경사 정지 PWM은 0~100 정수로 입력하세요", True)
        finally:
            curses.noecho()
            stdscr.nodelay(True)

    def run(self, stdscr) -> None:
        curses.curs_set(0)
        curses.noecho()
        stdscr.keypad(True)
        stdscr.nodelay(True)
        if self.auto_prepare_scenario is not None:
            self._initial_rtk_attempted = True
            self.select_number(self.auto_prepare_scenario)
            self.last_key_input = f"AUTO-R({self.auto_prepare_scenario})"
            if self.scenario.number == self.auto_prepare_scenario:
                self.prepare_selected(stdscr)
        while rclpy.ok():
            # All monitor subscriptions keep only their newest sample.  Two
            # callbacks per screen cycle rotate through the complete status
            # set within the 0.7 s freshness window without consuming every
            # high-rate telemetry message.
            rclpy.spin_once(self.node, timeout_sec=0.01)
            rclpy.spin_once(self.node, timeout_sec=0.0)
            if (
                self.auto_rtk
                and not self._initial_rtk_attempted
                and time.monotonic() - self._created_at > 1.0
            ):
                self._initial_rtk_attempted = True
                self.start_rtk()
            self.check_children()
            self.enforce_conflicts()
            self.trip_on_runtime_failure()
            # A start attempt made during the first second of ROS graph setup
            # can leave an old "missing command" error on screen even after
            # every live input has recovered.  Clear only that stale start
            # message; current blockers are still rendered on every frame.
            if (
                self.mode == "ROUTE"
                and self.stack.running
                and self.message_is_error
                and self.message.startswith("출발 차단:")
                and not self.route_start_blockers()
            ):
                self._set_message(
                    "출발 준비 완료: SPACE를 누르면 현재 위치의 미션부터 출발합니다"
                )
            self.draw(stdscr)
            key = stdscr.getch()
            if key == -1:
                time.sleep(0.05)
                continue
            self.last_key_input = key_input_label(key)
            if ord("1") <= key <= ord("9"):
                self.select_number(key - ord("0"))
            elif key in (ord("["), curses.KEY_LEFT):
                self.change_variant(-1)
            elif key in (ord("]"), curses.KEY_RIGHT):
                self.change_variant(1)
            elif key in (ord("v"), ord("V")):
                self.prompt_speed(stdscr)
            elif key in (ord("h"), ord("H")):
                self.prompt_hill_hold_pwm(stdscr)
            elif key in (ord("r"), ord("R")):
                self.prepare_selected(stdscr)
            elif key in (ord("a"), ord("A")):
                self.apply_datum()
            elif key in (ord("g"), ord("G")):
                self.stop_stack(fault=True)
                self.settle_graph()
                self.start_rtk()
            elif key in (ord("b"), ord("B")):
                self.toggle_recording()
            elif key == ord(" "):
                self.toggle_run()
            elif key in (ord("x"), ord("X")):
                self.emergency_stop()
            elif key in (ord("s"), ord("S")):
                self.stop_stack(fault=True)
                self._set_message("주행 스택 종료; G로 RTK 확인을 다시 시작할 수 있습니다")
            elif key in (ord("q"), ord("Q")):
                self._set_message("Q 입력 확인: ROS 프로세스와 MCAP을 종료하는 중")
                self.draw(stdscr)
                return


def argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="HL Mando GNSS/scenario field TUI")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--no-auto-rtk", action="store_true")
    parser.add_argument(
        "--auto-prepare-scenario",
        type=int,
        metavar="NUMBER",
        help="open the selected scenario in brake/ready state; SPACE still starts driving",
    )
    parser.add_argument("--check-config", action="store_true", help="validate YAML and print each route's static readiness without opening the TUI")
    parser.add_argument("--print-commands", action="store_true", help="print resolved RTK and route launch commands")
    return parser


def print_config_report(config: ConsoleConfig, print_commands: bool) -> int:
    print(f"config: {config.source_file}")
    if print_commands:
        print("RTK:", " ".join(build_rtk_command(config)))
    all_ready = True
    for scenario in config.scenarios:
        if scenario.kind == "finger":
            print(
                f"{scenario.number}. {scenario.name}: "
                f"step {scenario.default_speed:.0f}/100, allowed 1~{scenario.maximum_pwm}"
            )
            continue
        if scenario.kind == "datum":
            output = Path(os.environ.get("HLKU_DATA", config.source_file.parent.parent / "data")) / "datum" / "konkuk_datum_TIMESTAMP.yaml"
            print(
                f"{scenario.number}. {scenario.name}: {scenario.default_speed:.0f}s, "
                f"minimum {scenario.survey_minimum_samples} samples"
            )
            if print_commands:
                print(
                    "  ",
                    " ".join(
                        build_datum_command(
                            config, scenario, scenario.default_speed, output
                        )
                    ),
                )
            continue
        for variant in scenario.variants:
            report = static_readiness(config, variant)
            all_ready = all_ready and report.ok
            print(
                f"{scenario.number}. {scenario.name}/"
                f"{variant_compact_summary(variant)}: "
                f"{'READY' if report.ok else report.summary()}"
            )
            if print_commands:
                print("  ", " ".join(build_route_command(config, variant, scenario.default_speed)))
    return 0 if all_ready else 1


def main(args: Iterable[str] | None = None) -> None:
    options = argument_parser().parse_args(args)
    try:
        config = load_config(options.config)
    except (OSError, yaml.YAMLError, ValueError) as error:
        print(f"mando TUI config error: {error}", file=sys.stderr)
        raise SystemExit(2)
    if options.check_config or options.print_commands:
        raise SystemExit(print_config_report(config, options.print_commands))
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        print("mando_tui requires an interactive terminal", file=sys.stderr)
        raise SystemExit(2)
    domain = os.environ.get("ROS_DOMAIN_ID", "0")
    lock_path = Path(f"/tmp/hl_ku_mando_tui_domain_{domain}.lock")
    lock = lock_path.open("w", encoding="utf-8")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print(f"another Mando TUI is already running in ROS domain {domain}", file=sys.stderr)
        raise SystemExit(2)
    rclpy.init(args=[])
    node = MonitorNode()
    console = MandoConsole(
        config,
        node,
        not options.no_auto_rtk,
        options.auto_prepare_scenario,
    )
    try:
        curses.wrapper(console.run)
    except KeyboardInterrupt:
        pass
    finally:
        console.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
