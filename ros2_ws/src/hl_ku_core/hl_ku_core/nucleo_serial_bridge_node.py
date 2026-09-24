"""Bridge safe HL_KU actuator commands to the vehicle's existing NUCLEO UART."""

from __future__ import annotations

import math
import termios
import time

import rclpy
import serial
from rclpy.node import Node

from hl_ku_interfaces.msg import ActuatorCommand, VehicleFeedback

from .nucleo_serial import (
    SteeringAdcCalibration,
    drive_duty_to_command,
    drive_transaction,
    steering_transaction,
)


class NucleoSerialBridgeNode(Node):
    """Use ACK-checked position steering while retaining the current ADC map.

    The installed text protocol returns command acknowledgements, not measured
    telemetry.  Consequently steering feedback published here is the last
    acknowledged target and battery voltage is reported as unavailable (0 V).
    The NUCLEO remains responsible for its local ADC position loop/watchdog.
    """

    def __init__(self) -> None:
        super().__init__("nucleo_serial_bridge")
        self.declare_parameter(
            "port",
            "/tmp/nucleo-control",
        )
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("send_rate_hz", 20.0)
        self.declare_parameter("command_timeout_sec", 0.20)
        self.declare_parameter("serial_timeout_sec", 0.025)
        self.declare_parameter("maximum_consecutive_ack_failures", 3)
        self.declare_parameter("drive_command_scale", 100.0)
        self.declare_parameter("maximum_drive_command", 12)
        self.declare_parameter("allow_reverse", False)
        self.declare_parameter("maximum_steering_rad", 0.48)
        self.declare_parameter("steer_left_adc", 58744)
        self.declare_parameter("steer_center_adc", 31974)
        self.declare_parameter("steer_right_adc", 5111)

        rate = float(self.get_parameter("send_rate_hz").value)
        timeout = float(self.get_parameter("command_timeout_sec").value)
        serial_timeout = float(self.get_parameter("serial_timeout_sec").value)
        if not math.isfinite(rate) or rate < 10.0:
            raise ValueError("send_rate_hz must be finite and at least 10 Hz")
        if not math.isfinite(timeout) or not 0.0 < timeout < 0.30:
            raise ValueError("command_timeout_sec must be below the 0.30 s firmware watchdog")
        if not math.isfinite(serial_timeout) or not 0.0 < serial_timeout < timeout:
            raise ValueError("serial_timeout_sec must be positive and below command timeout")
        self._maximum_consecutive_ack_failures = int(
            self.get_parameter("maximum_consecutive_ack_failures").value
        )
        if self._maximum_consecutive_ack_failures < 1:
            raise ValueError("maximum_consecutive_ack_failures must be positive")

        self._steering = SteeringAdcCalibration(
            maximum_steering_rad=float(
                self.get_parameter("maximum_steering_rad").value
            ),
            left_adc=int(self.get_parameter("steer_left_adc").value),
            center_adc=int(self.get_parameter("steer_center_adc").value),
            right_adc=int(self.get_parameter("steer_right_adc").value),
        )
        self._steering.validate()
        self._drive_scale = float(self.get_parameter("drive_command_scale").value)
        self._maximum_drive_command = int(
            self.get_parameter("maximum_drive_command").value
        )
        drive_duty_to_command(0.0, self._drive_scale, self._maximum_drive_command)
        self._allow_reverse = bool(self.get_parameter("allow_reverse").value)
        self._timeout = timeout
        self._command: ActuatorCommand | None = None
        self._command_time: float | None = None
        self._received_command = False
        self._fault_latched = 0
        self._consecutive_ack_failures = 0
        self._last_acknowledged_steering_rad = 0.0
        self._last_warning_time = 0.0
        self._serial = serial.Serial(
            str(self.get_parameter("port").value),
            int(self.get_parameter("baudrate").value),
            timeout=serial_timeout,
            write_timeout=min(0.05, timeout),
        )
        self._serial.reset_input_buffer()
        self._feedback_pub = self.create_publisher(
            VehicleFeedback, "/vehicle/feedback", 20
        )
        self.create_subscription(
            ActuatorCommand,
            "/vehicle/actuator_command_safe",
            self._on_command,
            20,
        )
        self._send_stop(repetitions=3)
        self.create_timer(1.0 / rate, self._exchange)
        self.get_logger().info(
            "NUCLEO UART ready; steering preserved: "
            f"left={self._steering.left_adc}, center={self._steering.center_adc}, "
            f"right={self._steering.right_adc}, +/-{self._steering.maximum_steering_rad:.3f} rad; "
            f"route-test drive command limited to +/-{self._maximum_drive_command}"
        )

    def destroy_node(self):  # type: ignore[override]
        try:
            self._send_stop(repetitions=3)
            if self._serial.is_open:
                self._serial.close()
        finally:
            return super().destroy_node()

    def _warn(self, text: str) -> None:
        now = time.monotonic()
        if now - self._last_warning_time >= 1.0:
            self.get_logger().warning(text)
            self._last_warning_time = now

    def _on_command(self, message: ActuatorCommand) -> None:
        self._command = message
        self._command_time = time.monotonic()
        self._received_command = True

    def _fresh_command(self) -> bool:
        if self._command_time is None:
            return False
        age = time.monotonic() - self._command_time
        return 0.0 <= age <= self._timeout

    def _transact(self, command: str, expected: str) -> bool:
        self._serial.write((command + "\n").encode("ascii"))
        self._serial.flush()
        reply = self._serial.readline().decode("ascii", "replace").strip()
        if reply == expected:
            return True
        self._warn(f"unexpected NUCLEO reply {reply!r}; expected {expected!r}")
        return False

    def _send_stop(self, repetitions: int = 1) -> bool:
        if not getattr(self, "_serial", None) or not self._serial.is_open:
            return False
        command, expected = drive_transaction(0)
        success = True
        for _ in range(repetitions):
            try:
                success = self._transact(command, expected) and success
            except (OSError, termios.error, serial.SerialException) as error:
                self._warn(f"NUCLEO serial stop failed: {error}")
                return False
            if repetitions > 1:
                time.sleep(0.02)
        return success

    def _latch_protocol_fault(self, reason: str) -> None:
        self._fault_latched |= VehicleFeedback.FAULT_PROTOCOL
        self._warn(reason + "; protocol fault latched until bridge restart")
        self._send_stop()

    def _record_ack_failure(self, reason: str) -> None:
        self._consecutive_ack_failures += 1
        if self._consecutive_ack_failures >= self._maximum_consecutive_ack_failures:
            self._latch_protocol_fault(
                f"{reason} ({self._consecutive_ack_failures} consecutive failures)"
            )
            return
        self._warn(
            f"{reason}; stopped this cycle and will retry "
            f"({self._consecutive_ack_failures}/"
            f"{self._maximum_consecutive_ack_failures})"
        )

    def _record_ack_success(self) -> None:
        self._consecutive_ack_failures = 0

    def _publish_feedback(
        self,
        sequence: int,
        applied_duty: float,
        drive_enabled: bool,
        brake_active: bool,
    ) -> None:
        message = VehicleFeedback()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.steering_angle_rad = self._last_acknowledged_steering_rad
        message.steering_target_rad = self._last_acknowledged_steering_rad
        message.steering_output = math.nan
        message.applied_drive_duty = applied_duty
        message.battery_voltage = 0.0
        message.board_temperature_c = math.nan
        message.fault_flags = self._fault_latched
        message.last_command_sequence = sequence
        message.drive_enabled = drive_enabled and not self._fault_latched
        message.brake_active = brake_active or bool(self._fault_latched)
        self._feedback_pub.publish(message)

    def _exchange(self) -> None:
        command = self._command if self._fresh_command() else None
        if command is None:
            # A short ROS scheduling gap while the last command already asks
            # for braking is harmless: keep sending STOP without permanently
            # latching the bridge.  A gap after an active drive command still
            # latches the watchdog and requires a restart.
            last_command_active = bool(
                self._command is not None
                and self._command.enable
                and not self._command.brake
                and math.isfinite(float(self._command.drive_duty))
                and abs(float(self._command.drive_duty)) > 1.0e-6
            )
            if self._received_command and last_command_active:
                self._fault_latched |= VehicleFeedback.FAULT_WATCHDOG
                self._warn("safe actuator command timed out; watchdog fault latched")
            self._send_stop()
            sequence = self._command.sequence if self._command else 0
            self._publish_feedback(sequence, 0.0, False, True)
            return

        drive = float(command.drive_duty)
        steering = float(command.steering_angle_rad)
        sequence = int(command.sequence)
        values_valid = math.isfinite(drive) and math.isfinite(steering)
        active = command.enable and not command.brake
        braking = not command.enable and command.brake
        mode_valid = command.mode in (
            ActuatorCommand.MODE_AUTONOMOUS,
            ActuatorCommand.MODE_MANUAL,
        ) if active else command.mode in (
            ActuatorCommand.MODE_BRAKE,
            ActuatorCommand.MODE_FAULT,
            ActuatorCommand.MODE_MANUAL,
        )
        if (
            not values_valid
            or not (active or braking)
            or not mode_valid
            or (braking and abs(drive) > 1.0e-6)
            or (not self._allow_reverse and drive < 0.0)
            or abs(steering) > self._steering.maximum_steering_rad + 1.0e-6
        ):
            self._latch_protocol_fault("invalid safe actuator command rejected")
            self._publish_feedback(sequence, 0.0, False, True)
            return

        if self._fault_latched:
            if not self._send_stop():
                self._latch_protocol_fault("NUCLEO stop acknowledgement failed")
            self._publish_feedback(sequence, 0.0, False, True)
            return

        if braking:
            # STOP is transmitted every cycle.  A single delayed/missing ACK
            # on the multiplexed link must not make an otherwise healthy,
            # stationary vehicle unusable for the rest of the run.
            if not self._send_stop():
                self._warn("NUCLEO stop acknowledgement missed; retrying")
            else:
                self._record_ack_success()
            self._publish_feedback(sequence, 0.0, False, True)
            return

        drive_command = 0
        try:
            drive_command = drive_duty_to_command(
                drive, self._drive_scale, self._maximum_drive_command
            )
            target_adc = self._steering.target_adc(steering)
            drive_line, drive_ack = drive_transaction(drive_command)
            steer_line, steer_ack = steering_transaction(target_adc)
            acknowledged = self._transact(drive_line, drive_ack)
            if not acknowledged:
                self._record_ack_failure("NUCLEO drive acknowledgement failed")
            else:
                acknowledged = self._transact(steer_line, steer_ack)
                if not acknowledged:
                    self._record_ack_failure(
                        "NUCLEO steering acknowledgement failed"
                    )
            if not acknowledged:
                # The command may have reached the board even when its ACK was
                # delayed or dropped.  Stop immediately and report this cycle
                # as braked; retry the fresh safe command on the next tick.
                stop_acknowledged = self._send_stop()
                # With zero propulsion, the retry is the exact same CMD 0
                # transaction.  Its ACK proves the link is alive, so sparse
                # delayed replies must not accumulate into a permanent fault
                # while the vehicle is already stopped.
                if stop_acknowledged and drive_command == 0:
                    self._record_ack_success()
                self._publish_feedback(sequence, 0.0, False, True)
                return
            self._record_ack_success()
            self._last_acknowledged_steering_rad = steering
        except (OSError, termios.error, ValueError, serial.SerialException) as error:
            self._latch_protocol_fault(f"NUCLEO command failed: {error}")

        applied = 0.0 if self._fault_latched else drive_command / self._drive_scale
        self._publish_feedback(
            sequence,
            applied,
            not bool(self._fault_latched),
            bool(self._fault_latched),
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NucleoSerialBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
