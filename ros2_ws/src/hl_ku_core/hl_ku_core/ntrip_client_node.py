"""Small NTRIP client that publishes caster RTCM bytes for the serial owner."""

from __future__ import annotations

import base64
import socket
import ssl
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import UInt8MultiArray

from .nmea import make_gga
from .ntrip_stream import NtripBodyDecoder, response_is_chunked


class NtripClientNode(Node):
    def __init__(self) -> None:
        super().__init__("ntrip_client")
        self.declare_parameter("enabled", False)
        self.declare_parameter("host", "")
        self.declare_parameter("port", 2101)
        self.declare_parameter("mountpoint", "")
        self.declare_parameter("username", "")
        self.declare_parameter("password", "")
        self.declare_parameter("tls", False)
        self.declare_parameter("gga_period_sec", 5.0)
        self.declare_parameter("reconnect_sec", 3.0)
        self.declare_parameter("rtcm_idle_timeout_sec", 3.0)
        self._enabled = bool(self.get_parameter("enabled").value)
        self._rtcm_pub = self.create_publisher(UInt8MultiArray, "/gnss/rtcm", 20)
        self.create_subscription(NavSatFix, "/gnss/fix", self._on_fix, 10)
        self._latest_fix: tuple[float, float, float] | None = None
        self._stop = threading.Event()
        self._thread = None
        if self._enabled:
            self._validate_parameters()
            self._thread = threading.Thread(target=self._worker, daemon=True)
            self._thread.start()
        else:
            self.get_logger().warning("NTRIP 보정 서버 사용 안 함")

    def destroy_node(self):  # type: ignore[override]
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        return super().destroy_node()

    def _validate_parameters(self) -> None:
        missing = [
            name
            for name in ("host", "mountpoint", "username", "password")
            if not str(self.get_parameter(name).value)
        ]
        if missing:
            raise ValueError("NTRIP enabled but parameters are empty: " + ", ".join(missing))
        idle_timeout = float(self.get_parameter("rtcm_idle_timeout_sec").value)
        if idle_timeout <= 1.0:
            raise ValueError("rtcm_idle_timeout_sec must be greater than 1.0 s")

    def _on_fix(self, message: NavSatFix) -> None:
        self._latest_fix = (message.latitude, message.longitude, message.altitude)

    def _request(self) -> bytes:
        host = str(self.get_parameter("host").value)
        mount = str(self.get_parameter("mountpoint").value).lstrip("/")
        credentials = (
            f"{self.get_parameter('username').value}:"
            f"{self.get_parameter('password').value}"
        ).encode("utf-8")
        authorization = base64.b64encode(credentials).decode("ascii")
        return (
            f"GET /{mount} HTTP/1.1\r\n"
            f"Host: {host}\r\n"
            "Ntrip-Version: Ntrip/2.0\r\n"
            "User-Agent: NTRIP HL_KU/0.1\r\n"
            f"Authorization: Basic {authorization}\r\n"
            "Connection: close\r\n\r\n"
        ).encode("ascii")

    def _connect(self) -> tuple[socket.socket, NtripBodyDecoder]:
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        connection = socket.create_connection((host, port), timeout=10.0)
        if bool(self.get_parameter("tls").value):
            connection = ssl.create_default_context().wrap_socket(
                connection, server_hostname=host
            )
        connection.settimeout(1.0)
        connection.sendall(self._request())
        header = bytearray()
        while b"\r\n\r\n" not in header and len(header) < 16384:
            chunk = connection.recv(1024)
            if not chunk:
                raise ConnectionError("NTRIP caster closed during headers")
            header.extend(chunk)
        header_bytes, remainder = bytes(header).split(b"\r\n\r\n", 1)
        first_line = header_bytes.splitlines()[0] if header_bytes else b""
        if b"200" not in first_line and not first_line.startswith(b"ICY 200"):
            raise ConnectionError(first_line.decode("ascii", errors="replace"))
        decoder = NtripBodyDecoder(response_is_chunked(header_bytes))
        for payload in decoder.feed(remainder):
            self._publish_rtcm(payload)
        return connection, decoder

    def _publish_rtcm(self, payload: bytes) -> None:
        if self._stop.is_set() or not rclpy.ok():
            return
        message = UInt8MultiArray()
        message.data = list(payload)
        try:
            self._rtcm_pub.publish(message)
        except Exception:
            # ROS launch can invalidate the context just before the worker notices
            # the stop event. Suppress only that expected shutdown race.
            if self._stop.is_set() or not rclpy.ok():
                return
            raise

    def _worker(self) -> None:
        reconnect = float(self.get_parameter("reconnect_sec").value)
        gga_period = float(self.get_parameter("gga_period_sec").value)
        idle_timeout = float(self.get_parameter("rtcm_idle_timeout_sec").value)
        while not self._stop.is_set():
            try:
                connection, decoder = self._connect()
                self.get_logger().info("NTRIP 보정 서버 연결 성공")
                next_gga = 0.0
                last_rtcm = time.monotonic()
                with connection:
                    while not self._stop.is_set():
                        now = time.monotonic()
                        if self._latest_fix is not None and now >= next_gga:
                            connection.sendall(make_gga(*self._latest_fix).encode("ascii"))
                            next_gga = now + gga_period
                        try:
                            chunk = connection.recv(4096)
                        except socket.timeout:
                            idle_age = time.monotonic() - last_rtcm
                            if idle_age > idle_timeout:
                                raise ConnectionError(
                                    f"no RTCM payload for {idle_age:.1f}s"
                                )
                            continue
                        if not chunk:
                            raise ConnectionError("NTRIP caster closed the stream")
                        for payload in decoder.feed(chunk):
                            last_rtcm = time.monotonic()
                            self._publish_rtcm(payload)
            except (OSError, ConnectionError) as error:
                self.get_logger().warning(f"NTRIP 연결 실패: {error}")
                self._stop.wait(reconnect)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NtripClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
