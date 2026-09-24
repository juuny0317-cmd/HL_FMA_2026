"""Fixed-rate, CRC-protected UDP bridge to the NUCLEO Ethernet endpoint."""

from __future__ import annotations

import math
import secrets
import socket
import time

import rclpy
from rclpy.node import Node

from hl_ku_interfaces.msg import ActuatorCommand, VehicleFeedback

from .protocol import FLAG_BRAKE, FLAG_ENABLE, pack_command, unpack_feedback


class McuUdpBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("mcu_udp_bridge")
        self.declare_parameter("remote_host", "192.168.10.20")
        self.declare_parameter("remote_port", 15000)
        self.declare_parameter("local_host", "0.0.0.0")
        self.declare_parameter("local_port", 15001)
        self.declare_parameter("send_rate_hz", 50.0)
        self.declare_parameter("command_timeout_sec", 0.15)
        self.declare_parameter("accept_feedback_from_remote_only", True)
        self.declare_parameter("feedback_sequence_window", 10)
        self._remote = (
            str(self.get_parameter("remote_host").value),
            int(self.get_parameter("remote_port").value),
        )
        self._remote_ip = socket.gethostbyname(self._remote[0])
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setblocking(False)
        self._socket.bind(
            (
                str(self.get_parameter("local_host").value),
                int(self.get_parameter("local_port").value),
            )
        )
        self._command: ActuatorCommand | None = None
        self._command_time = None
        self._sequence = secrets.randbits(32)
        self._last_error_sec = 0.0
        self._feedback_pub = self.create_publisher(
            VehicleFeedback, "/vehicle/feedback", 20
        )
        self._send_brake_burst()
        self.create_subscription(
            ActuatorCommand,
            "/vehicle/actuator_command_safe",
            self._on_command,
            20,
        )
        rate = max(10.0, float(self.get_parameter("send_rate_hz").value))
        self.create_timer(1.0 / rate, self._exchange)

    def destroy_node(self):  # type: ignore[override]
        try:
            self._send_brake_burst()
            self._socket.close()
        finally:
            return super().destroy_node()

    def _send_brake_burst(self) -> None:
        for _ in range(3):
            self._sequence = (self._sequence + 1) & 0xFFFFFFFF
            try:
                self._socket.sendto(
                    pack_command(self._sequence, 0.0, 0.0, False, True),
                    self._remote,
                )
            except OSError:
                break

    def _on_command(self, message: ActuatorCommand) -> None:
        self._command = message
        self._command_time = time.monotonic()

    def _fresh_command(self) -> bool:
        if self._command_time is None:
            return False
        age = time.monotonic() - self._command_time
        return 0.0 <= age <= float(self.get_parameter("command_timeout_sec").value)

    def _warn(self, text: str) -> None:
        now_sec = time.monotonic()
        if now_sec - self._last_error_sec >= 1.0:
            self.get_logger().warning(text)
            self._last_error_sec = now_sec

    def _feedback_sequence_is_recent(self, sequence: int) -> bool:
        lag = (self._sequence - sequence) & 0xFFFFFFFF
        return lag <= int(self.get_parameter("feedback_sequence_window").value)

    def _exchange(self) -> None:
        self._sequence = (self._sequence + 1) & 0xFFFFFFFF
        command = self._command if self._fresh_command() else None
        packet = pack_command(
            self._sequence,
            command.drive_duty if command else 0.0,
            command.steering_angle_rad if command else 0.0,
            command.enable if command else False,
            command.brake if command else True,
        )
        try:
            self._socket.sendto(packet, self._remote)
        except OSError as error:
            self._warn(f"MCU UDP send failed: {error}")
        while True:
            try:
                data, address = self._socket.recvfrom(256)
            except BlockingIOError:
                break
            except OSError as error:
                self._warn(f"MCU UDP receive failed: {error}")
                break
            if (
                bool(self.get_parameter("accept_feedback_from_remote_only").value)
                and address[0] != self._remote_ip
            ):
                self._warn(f"MCU feedback rejected from unexpected host {address[0]}")
                continue
            try:
                feedback = unpack_feedback(data)
            except ValueError as error:
                self._warn(f"MCU feedback rejected: {error}")
                continue
            if not self._feedback_sequence_is_recent(feedback.sequence):
                self._warn("MCU feedback rejected: stale or unexpected sequence")
                continue
            message = VehicleFeedback()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = "base_link"
            message.steering_angle_rad = feedback.steering_angle_rad
            message.steering_target_rad = feedback.steering_target_rad
            message.steering_output = math.nan
            message.applied_drive_duty = feedback.applied_drive_duty
            message.battery_voltage = feedback.battery_voltage
            message.board_temperature_c = math.nan
            message.fault_flags = feedback.fault_flags
            message.last_command_sequence = feedback.sequence
            message.drive_enabled = bool(feedback.flags & FLAG_ENABLE)
            message.brake_active = bool(feedback.flags & FLAG_BRAKE)
            self._feedback_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = McuUdpBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
