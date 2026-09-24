"""Own the terminal while running the complete GNSS route-driving launch."""

from __future__ import annotations

import argparse
import math
import os
import select
import signal
import subprocess
import sys
import termios
import time
from pathlib import Path
import tty

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from hl_ku_interfaces.msg import GnssStatus


DEFAULT_DRIVE_COMMAND = 30
DEFAULT_MISSION_SPEED_MPS = 0.30
MAXIMUM_PROTOCOL_DRIVE_COMMAND = 100
MAXIMUM_CONFIGURED_SPEED_MPS = 1.00
MAXIMUM_STEERING_RAD = 0.48
MAXIMUM_CORRECTION_AGE_SEC = 2.0
MINIMUM_SATELLITES = 8
MAXIMUM_HDOP = 2.5
HEADING_TIMEOUT_SEC = 1.5


def _number_text(value: float, digits: int, suffix: str = "") -> str:
    return f"{value:.{digits}f}{suffix}" if math.isfinite(value) else "없음"


def _heading_text(status: GnssStatus) -> str:
    if status.heading_valid:
        return "정상"
    age = float(status.heading_age_sec)
    if not math.isfinite(age):
        return "미수신(ANT2/GNTHS 확인)"
    if age > HEADING_TIMEOUT_SEC:
        return f"시간초과({_number_text(age, 1, '초')})"
    return "무효(ANT2 기준선 확인)"


def format_rtk_status(status: GnssStatus) -> str:
    fix_labels = {
        GnssStatus.FIX_NONE: "미수신",
        GnssStatus.FIX_SINGLE: "단독측위",
        GnssStatus.FIX_DGPS: "DGPS",
        GnssStatus.FIX_RTK_FIXED: "RTK 고정(FIXED)",
        GnssStatus.FIX_RTK_FLOAT: "RTK 유동(FLOAT)",
    }
    fix = fix_labels.get(int(status.fix_type), f"알 수 없음({status.fix_type})")
    correction_fresh = (
        math.isfinite(float(status.correction_age_sec))
        and 0.0 <= float(status.correction_age_sec) <= MAXIMUM_CORRECTION_AGE_SEC
    )
    rtk_fixed = status.fix_type == GnssStatus.FIX_RTK_FIXED
    quality_ready = (
        rtk_fixed
        and correction_fresh
        and status.position_valid
        and status.heading_valid
        and status.velocity_valid
        and status.nmea_checksum_valid
        and status.satellites >= MINIMUM_SATELLITES
        and math.isfinite(float(status.hdop))
        and float(status.hdop) <= MAXIMUM_HDOP
    )
    blockers = []
    if not rtk_fixed:
        blockers.append("RTK 고정")
    if not correction_fresh:
        blockers.append("보정 데이터")
    if not status.position_valid:
        blockers.append("위치")
    if not status.heading_valid:
        blockers.append("헤딩")
    if not status.velocity_valid:
        blockers.append("속도")
    if status.satellites < MINIMUM_SATELLITES:
        blockers.append("위성 수")
    if not math.isfinite(float(status.hdop)) or float(status.hdop) > MAXIMUM_HDOP:
        blockers.append("HDOP")
    if not status.nmea_checksum_valid:
        blockers.append("NMEA 체크섬")

    if quality_ready:
        connection = "정상 · 주행 가능"
    elif rtk_fixed and not correction_fresh:
        connection = "보정정보 지연"
    elif rtk_fixed:
        connection = "RTK 고정 · 안전입력 대기"
    elif status.fix_type == GnssStatus.FIX_RTK_FLOAT:
        connection = "수렴 중"
    elif status.fix_type == GnssStatus.FIX_DGPS and correction_fresh:
        connection = "보정 수신 중 · DGPS(고정 대기)"
    elif correction_fresh:
        connection = "보정 수신 중 · RTK 미고정"
    else:
        connection = "RTK 미연결"
    blocker_text = ", ".join(blockers) if blockers else "없음"
    return (
        f"RTK 연결: {connection} | 측위={fix} | 위성={status.satellites}개 | "
        f"HDOP={_number_text(float(status.hdop), 2)} | "
        f"보정나이={_number_text(float(status.correction_age_sec), 1, '초')} | "
        f"위치={'정상' if status.position_valid else '대기'} | "
        f"헤딩={_heading_text(status)} | "
        f"속도={'정상' if status.velocity_valid else '대기'} | "
        f"주행대기={blocker_text}"
    )


def _bounded_float(minimum: float, maximum: float):
    def parse(value: str) -> float:
        parsed = float(value)
        if not minimum <= parsed <= maximum:
            raise argparse.ArgumentTypeError(
                f"must be between {minimum:g} and {maximum:g}"
            )
        return parsed

    return parse


def _bounded_int(minimum: int, maximum: int):
    def parse(value: str) -> int:
        parsed = int(value)
        if not minimum <= parsed <= maximum:
            raise argparse.ArgumentTypeError(
                f"must be between {minimum:d} and {maximum:d}"
            )
        return parsed

    return parse


def argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the GNSS waypoint-driving stack with a Space start/pause key."
    )
    parser.add_argument(
        "--supervised",
        action="store_true",
        help="enable the actuator for a supervised closed-course test",
    )
    parser.add_argument(
        "--route-file",
        type=Path,
        help="prepared vehicle-reference route CSV (defaults to course_06_vehicle.csv)",
    )
    parser.add_argument(
        "--mission-speed",
        type=_bounded_float(0.01, MAXIMUM_CONFIGURED_SPEED_MPS),
        default=None,
        metavar="MPS",
        help="mission speed ceiling in m/s (default: 0.30)",
    )
    parser.add_argument(
        "--drive-command",
        type=_bounded_int(1, MAXIMUM_PROTOCOL_DRIVE_COMMAND),
        default=None,
        metavar="PWM",
        help="NUCLEO drive/PWM command from 1 to 100 (default: 30)",
    )
    parser.add_argument(
        "--prompt",
        action="store_true",
        help="ask for speed and PWM in the terminal before launching",
    )
    parser.add_argument(
        "--no-rviz",
        action="store_true",
        help="do not open the route-alignment RViz window",
    )
    return parser


def _prompt_value(label: str, default, parser, input_fn, output_fn):
    while True:
        raw = input_fn(f"{label} [{default}]: ").strip()
        if not raw:
            return default
        try:
            return parser(raw)
        except (TypeError, ValueError, argparse.ArgumentTypeError) as error:
            output_fn(f"입력 오류: {error}. 다시 입력하세요.")


def resolve_options(options: argparse.Namespace, input_fn=input, output_fn=print):
    if options.mission_speed is None:
        if options.prompt:
            options.mission_speed = _prompt_value(
                "목표속도 m/s (0.01~1.00)",
                DEFAULT_MISSION_SPEED_MPS,
                _bounded_float(0.01, MAXIMUM_CONFIGURED_SPEED_MPS),
                input_fn,
                output_fn,
            )
        else:
            options.mission_speed = DEFAULT_MISSION_SPEED_MPS
    if options.drive_command is None:
        if options.prompt:
            options.drive_command = _prompt_value(
                "구동 PWM (1~100)",
                DEFAULT_DRIVE_COMMAND,
                _bounded_int(1, MAXIMUM_PROTOCOL_DRIVE_COMMAND),
                input_fn,
                output_fn,
            )
        else:
            options.drive_command = DEFAULT_DRIVE_COMMAND
    if options.prompt:
        output_fn(
            f"설정 완료: 목표속도={options.mission_speed:.2f} m/s, "
            f"구동 PWM={options.drive_command}/100"
        )
    return options


def build_launch_command(options: argparse.Namespace) -> list[str]:
    duty = options.drive_command / 100.0
    enabled = "true" if options.supervised else "false"
    command = [
        "ros2",
        "launch",
        "hl_ku_core",
        "gps_route_test.launch.py",
        f"rviz:={'false' if options.no_rviz else 'true'}",
        f"actuator_bridge_enabled:={enabled}",
        f"route_calibrated:={enabled}",
        f"drive_enabled:={enabled}",
        f"require_preflight:={'false' if options.supervised else 'true'}",
        f"mission_speed_mps:={options.mission_speed:.3f}",
        f"maximum_drive_duty:={duty:.2f}",
        f"minimum_forward_duty:={duty:.2f}",
        f"maximum_drive_command:={options.drive_command}",
    ]
    if options.route_file is not None:
        command.append(f"route_file:={options.route_file.expanduser().resolve()}")
    return command


class RouteDriveSession(Node):
    def __init__(self) -> None:
        super().__init__("route_drive_session")
        self._toggle_client = self.create_client(Trigger, "/mission/toggle_run")
        self._fault_client = self.create_client(Trigger, "/mission/fault")
        self._gnss_status: GnssStatus | None = None
        self._gnss_received_at: float | None = None
        self._last_rtk_log_at = 0.0
        self.create_subscription(GnssStatus, "/gnss/status", self._on_gnss, 20)
        self.create_timer(0.5, self._report_rtk)

    def _on_gnss(self, message: GnssStatus) -> None:
        self._gnss_status = message
        self._gnss_received_at = time.monotonic()

    def _report_rtk(self) -> None:
        now = time.monotonic()
        if self._gnss_status is None or self._gnss_received_at is None:
            text = "RTK 연결: GNSS 데이터 기다리는 중"
        elif now - self._gnss_received_at > 2.0:
            text = f"RTK 연결: GNSS 메시지 끊김 ({now - self._gnss_received_at:.1f}초)"
        else:
            text = format_rtk_status(self._gnss_status)
        if self._last_rtk_log_at and now - self._last_rtk_log_at < 2.0:
            return
        self.get_logger().info(text)
        self._last_rtk_log_at = now

    def call_trigger(self, client, action: str, timeout_sec: float = 2.0) -> bool:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(f"{action} rejected: mission service is not ready")
            return False
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done() or future.result() is None:
            self.get_logger().error(f"{action} failed: no service response")
            return False
        response = future.result()
        log = self.get_logger().info if response.success else self.get_logger().error
        log(response.message)
        return bool(response.success)

    def toggle_run(self) -> bool:
        return self.call_trigger(self._toggle_client, "start/pause")

    def fault(self) -> bool:
        return self.call_trigger(self._fault_client, "emergency stop", timeout_sec=1.0)


def _stop_launch(child: subprocess.Popen, timeout_sec: float = 8.0) -> None:
    if child.poll() is not None:
        return
    try:
        os.killpg(child.pid, signal.SIGINT)
        child.wait(timeout=timeout_sec)
        return
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass
    try:
        os.killpg(child.pid, signal.SIGTERM)
        child.wait(timeout=2.0)
        return
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass
    try:
        os.killpg(child.pid, signal.SIGKILL)
        child.wait(timeout=1.0)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass


def _print_controls(options: argparse.Namespace) -> None:
    mode = "SUPERVISED ACTUATOR" if options.supervised else "PREVIEW / NO ACTUATOR"
    duty = options.drive_command / 100.0
    print("\n" + "=" * 72, flush=True)
    print(f"GNSS ROUTE SESSION: {mode}", flush=True)
    print(
        "Starts stopped. SPACE=start / pause / resume, X=latched stop, Q=quit",
        flush=True,
    )
    print(
        f"Limits: target={options.mission_speed:.2f} m/s, "
        f"drive={options.drive_command}/100 ({duty:.2f}), "
        f"steering=+/-{MAXIMUM_STEERING_RAD:.2f} rad",
        flush=True,
    )
    print("PC steering slew-rate limiter: none", flush=True)
    if options.drive_command > DEFAULT_DRIVE_COMMAND:
        print(
            "WARNING: drive command is above the previous autonomous baseline 30/100",
            flush=True,
        )
    print("=" * 72 + "\n", flush=True)


def run(options: argparse.Namespace) -> int:
    if not sys.stdin.isatty():
        print("route_drive_session requires an interactive terminal", file=sys.stderr)
        return 2
    if (
        options.route_file is not None
        and not options.route_file.expanduser().is_file()
    ):
        print(f"route file not found: {options.route_file}", file=sys.stderr)
        return 2

    command = build_launch_command(options)
    child = subprocess.Popen(command, start_new_session=True)
    rclpy.init(args=[])
    node = RouteDriveSession()
    old_terminal = termios.tcgetattr(sys.stdin.fileno())
    _print_controls(options)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok() and child.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.02)
            readable, _, _ = select.select([sys.stdin], [], [], 0.03)
            if not readable:
                continue
            key = sys.stdin.read(1)
            if key == " ":
                node.toggle_run()
            elif key in ("x", "X"):
                node.fault()
            elif key in ("q", "Q"):
                node.fault()
                break
        return_code = child.poll()
        if return_code not in (None, 0):
            node.get_logger().error(
                f"launch exited unexpectedly: code {return_code}"
            )
            return int(return_code)
        return 0
    except KeyboardInterrupt:
        node.fault()
        return 130
    finally:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_terminal)
        _stop_launch(child)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None) -> None:
    options = resolve_options(argument_parser().parse_args(args))
    raise SystemExit(run(options))
