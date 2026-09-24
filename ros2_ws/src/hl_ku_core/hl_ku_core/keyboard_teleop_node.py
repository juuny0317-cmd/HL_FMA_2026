"""Momentary, low-duty keyboard control for supervised calibration only."""

from __future__ import annotations

import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node

from hl_ku_interfaces.msg import ActuatorCommand

from .geometry import clamp
from .teleop import (
    direct_signed_drive,
    fixed_steering_target,
    manual_control_active,
    manual_command_status,
    step_signed_drive,
    step_steering_target,
    three_state_signed_drive,
)


class KeyboardTeleopNode(Node):
    def __init__(self) -> None:
        super().__init__("keyboard_teleop")
        self.declare_parameter("initial_drive_duty", 0.10)
        self.declare_parameter("maximum_drive_duty", 0.20)
        self.declare_parameter("drive_duty_step", 0.02)
        self.declare_parameter("maximum_steering_rad", 0.35)
        self.declare_parameter("steering_step_rad", 0.03)
        self.declare_parameter("steering_control_mode", "step")
        self.declare_parameter("deadman_timeout_sec", 0.20)
        self.declare_parameter("steering_hold_sec", 0.30)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("drive_control_mode", "momentary")
        if not sys.stdin.isatty():
            raise RuntimeError(
                "keyboard_teleop needs an interactive terminal; run it with ros2 run"
            )
        self._stdin_fd = sys.stdin.fileno()
        self._terminal_settings = termios.tcgetattr(self._stdin_fd)
        tty.setcbreak(self._stdin_fd)
        maximum_duty = float(self.get_parameter("maximum_drive_duty").value)
        self._duty_magnitude = clamp(
            float(self.get_parameter("initial_drive_duty").value),
            0.0,
            maximum_duty,
        )
        self._drive_control_mode = str(
            self.get_parameter("drive_control_mode").value
        ).strip().lower()
        if self._drive_control_mode not in ("momentary", "incremental", "direct"):
            raise ValueError(
                "drive_control_mode must be momentary, incremental, or direct"
            )
        self._steering_control_mode = str(
            self.get_parameter("steering_control_mode").value
        ).strip().lower()
        if self._steering_control_mode not in ("step", "fixed", "incremental"):
            raise ValueError(
                "steering_control_mode must be step, fixed, or incremental"
            )
        self._incremental_drive_duty = 0.0
        self._direction = 0
        self._steering_rad = 0.0
        self._persistent_steering_active = False
        self._motion_deadline_ns = 0
        self._control_deadline_ns = 0
        self._emergency_latched = False
        self._quit_requested = False
        self._last_key = "-"
        self._last_status = ""
        self._sequence = 0
        self._publisher = self.create_publisher(
            ActuatorCommand, "/vehicle/actuator_command_raw", 10
        )
        rate = max(20.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._update)
        if self._drive_control_mode == "direct":
            drive_help = "W/S +PWM <-> 0 <-> -PWM, A/D fixed full left/right"
        elif self._drive_control_mode == "incremental":
            drive_help = "W/S persistent PWM steps"
        else:
            drive_help = "W/S momentary drive"
        steering_help = (
            "A/D persistent steering steps"
            if self._steering_control_mode == "incremental"
            else "A/D steering"
        )
        self.get_logger().warning(
            f"CALIBRATION ONLY: {drive_help}, {steering_help}, C center, "
            "+/- duty, SPACE brake, X latch stop, Q quit"
        )
        print("현재 입력은 아래 한 줄에 계속 갱신됩니다.", flush=True)

    @property
    def quit_requested(self) -> bool:
        return self._quit_requested

    def destroy_node(self):  # type: ignore[override]
        try:
            if self._last_status:
                sys.stdout.write("\n")
                sys.stdout.flush()
            termios.tcsetattr(
                self._stdin_fd, termios.TCSADRAIN, self._terminal_settings
            )
        finally:
            return super().destroy_node()

    def _read_keys(self) -> None:
        while select.select([sys.stdin], [], [], 0.0)[0]:
            key = sys.stdin.read(1)
            self._handle_key(key)

    def _handle_key(self, key: str) -> None:
        self._last_key = {
            " ": "SPACE",
            "\x03": "CTRL+C",
            "\r": "ENTER",
            "\n": "ENTER",
        }.get(key, key.upper() if key else "-")
        now_ns = time.monotonic_ns()
        deadman_ns = int(
            float(self.get_parameter("deadman_timeout_sec").value) * 1.0e9
        )
        steering_hold_ns = int(
            float(self.get_parameter("steering_hold_sec").value) * 1.0e9
        )
        maximum_steering = float(self.get_parameter("maximum_steering_rad").value)
        steering_step = float(self.get_parameter("steering_step_rad").value)
        maximum_duty = float(self.get_parameter("maximum_drive_duty").value)
        duty_step = float(self.get_parameter("drive_duty_step").value)
        lower_key = key.lower()
        if lower_key == "w" and not self._emergency_latched:
            if self._drive_control_mode == "direct":
                self._incremental_drive_duty = three_state_signed_drive(
                    self._incremental_drive_duty,
                    1,
                    self._duty_magnitude,
                    maximum_duty,
                )
            elif self._drive_control_mode == "incremental":
                self._incremental_drive_duty = step_signed_drive(
                    self._incremental_drive_duty,
                    1,
                    self._duty_magnitude,
                    duty_step,
                    maximum_duty,
                )
            else:
                self._direction = 1
                self._motion_deadline_ns = now_ns + deadman_ns
                self._control_deadline_ns = self._motion_deadline_ns
        elif lower_key == "s" and not self._emergency_latched:
            if self._drive_control_mode == "direct":
                self._incremental_drive_duty = three_state_signed_drive(
                    self._incremental_drive_duty,
                    -1,
                    self._duty_magnitude,
                    maximum_duty,
                )
            elif self._drive_control_mode == "incremental":
                self._incremental_drive_duty = step_signed_drive(
                    self._incremental_drive_duty,
                    -1,
                    self._duty_magnitude,
                    duty_step,
                    maximum_duty,
                )
            else:
                self._direction = -1
                self._motion_deadline_ns = now_ns + deadman_ns
                self._control_deadline_ns = self._motion_deadline_ns
        elif lower_key == "a" and not self._emergency_latched:
            if self._steering_control_mode == "fixed":
                self._steering_rad = fixed_steering_target(-1, maximum_steering)
                self._persistent_steering_active = True
            else:
                self._steering_rad = step_steering_target(
                    self._steering_rad,
                    -1,
                    steering_step,
                    maximum_steering,
                )
                self._persistent_steering_active = (
                    self._steering_control_mode == "incremental"
                )
            self._control_deadline_ns = now_ns + steering_hold_ns
        elif lower_key == "d" and not self._emergency_latched:
            if self._steering_control_mode == "fixed":
                self._steering_rad = fixed_steering_target(1, maximum_steering)
                self._persistent_steering_active = True
            else:
                self._steering_rad = step_steering_target(
                    self._steering_rad,
                    1,
                    steering_step,
                    maximum_steering,
                )
                self._persistent_steering_active = (
                    self._steering_control_mode == "incremental"
                )
            self._control_deadline_ns = now_ns + steering_hold_ns
        elif lower_key == "c" and not self._emergency_latched:
            self._steering_rad = 0.0
            if self._steering_control_mode in ("fixed", "incremental"):
                self._persistent_steering_active = True
            self._control_deadline_ns = now_ns + steering_hold_ns
        elif key in ("+", "="):
            self._duty_magnitude = clamp(
                self._duty_magnitude + duty_step, 0.0, maximum_duty
            )
            if self._drive_control_mode == "direct" and abs(
                self._incremental_drive_duty
            ) > 1.0e-9:
                self._incremental_drive_duty = direct_signed_drive(
                    1 if self._incremental_drive_duty > 0.0 else -1,
                    self._duty_magnitude,
                    maximum_duty,
                )
        elif key in ("-", "_"):
            self._duty_magnitude = clamp(
                self._duty_magnitude - duty_step, 0.0, maximum_duty
            )
            if self._drive_control_mode == "direct" and abs(
                self._incremental_drive_duty
            ) > 1.0e-9:
                self._incremental_drive_duty = direct_signed_drive(
                    1 if self._incremental_drive_duty > 0.0 else -1,
                    self._duty_magnitude,
                    maximum_duty,
                )
        elif key == " ":
            self._direction = 0
            self._incremental_drive_duty = 0.0
            self._persistent_steering_active = False
            self._motion_deadline_ns = 0
            self._control_deadline_ns = 0
        elif lower_key == "x":
            self._emergency_latched = True
            self._persistent_steering_active = False
            self._direction = 0
            self._incremental_drive_duty = 0.0
            self._motion_deadline_ns = 0
            self._control_deadline_ns = 0
            self.get_logger().error("keyboard emergency stop latched; restart to clear")
        elif lower_key == "q" or key == "\x03":
            self._persistent_steering_active = False
            self._direction = 0
            self._incremental_drive_duty = 0.0
            self._motion_deadline_ns = 0
            self._control_deadline_ns = 0
            self._quit_requested = True

    def _update(self) -> None:
        self._read_keys()
        now = self.get_clock().now()
        now_ns = time.monotonic_ns()
        persistent = self._drive_control_mode in ("incremental", "direct")
        moving = not self._emergency_latched and (
            abs(self._incremental_drive_duty) > 1.0e-9
            if persistent
            else self._direction != 0 and now_ns <= self._motion_deadline_ns
        )
        control_active = manual_control_active(
            self._emergency_latched,
            moving,
            self._persistent_steering_active,
            now_ns <= self._control_deadline_ns,
        )
        if not moving and not persistent:
            self._direction = 0
        self._sequence = (self._sequence + 1) & 0xFFFFFFFF
        message = ActuatorCommand()
        message.header.stamp = now.to_msg()
        message.header.frame_id = "base_link"
        message.drive_duty = (
            self._incremental_drive_duty
            if persistent and moving
            else self._direction * self._duty_magnitude if moving else 0.0
        )
        message.steering_angle_rad = self._steering_rad
        message.brake = not control_active
        message.enable = control_active
        message.mode = (
            ActuatorCommand.MODE_FAULT
            if self._emergency_latched
            else ActuatorCommand.MODE_MANUAL
        )
        message.sequence = self._sequence
        self._publisher.publish(message)
        status = manual_command_status(
            self._last_key,
            float(message.drive_duty),
            self._duty_magnitude,
            float(message.steering_angle_rad),
            bool(message.brake),
            bool(message.enable),
            self._emergency_latched,
        )
        if status != self._last_status:
            self._last_status = status
            sys.stdout.write("\r\033[2K" + status)
            sys.stdout.flush()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = KeyboardTeleopNode()
        while rclpy.ok() and not node.quit_requested:
            rclpy.spin_once(node, timeout_sec=0.05)
        if rclpy.ok():
            for _ in range(5):
                node._update()  # publish repeated brake packets before closing
                rclpy.spin_once(node, timeout_sec=0.02)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
