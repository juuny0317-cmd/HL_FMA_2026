"""Own the UM982 serial port, parse NMEA, and inject RTCM bytes."""

from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import UInt8MultiArray

from hl_ku_interfaces.msg import GnssStatus

from .nmea import Gga, Hdt, Rmc, parse_sentence, validate_sentence

try:
    import serial
except ImportError:  # pragma: no cover - reported clearly when node starts
    serial = None


class Um982SerialNode(Node):
    def __init__(self) -> None:
        super().__init__("um982_serial")
        self.declare_parameter("port", "/dev/ttyGNSS")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("frame_id", "gnss_main")
        self.declare_parameter("require_checksum", True)
        self.declare_parameter("configure_receiver", False)
        # An empty Python list is inferred as BYTE_ARRAY by ROS 2 Humble.  Use a
        # blank string element so YAML overrides such as ["GPTHS 1"] retain the
        # intended STRING_ARRAY type; blank entries are removed below.
        self.declare_parameter("startup_commands", [""])
        self.declare_parameter("heading_timeout_sec", 0.30)
        self.declare_parameter("position_timeout_sec", 0.30)
        self.declare_parameter("velocity_timeout_sec", 0.30)
        self.declare_parameter("serial_reconnect_sec", 1.0)
        if serial is None:
            raise RuntimeError("pyserial is required: sudo apt install python3-serial")

        self._frame_id = str(self.get_parameter("frame_id").value)
        self._require_checksum = bool(self.get_parameter("require_checksum").value)
        self._port = str(self.get_parameter("port").value)
        self._baudrate = int(self.get_parameter("baudrate").value)
        self._serial_reconnect_sec = max(
            0.1, float(self.get_parameter("serial_reconnect_sec").value)
        )
        self._configure_receiver = bool(
            self.get_parameter("configure_receiver").value
        )
        self._startup_commands = (
            [
                str(command).strip()
                for command in self.get_parameter("startup_commands").value
                if str(command).strip()
            ]
            if self._configure_receiver
            else []
        )
        self._lock = threading.Lock()
        self._serial = None
        self._next_reconnect_sec = 0.0
        self._last_serial_error_log_sec = -math.inf
        self._fix_pub = self.create_publisher(NavSatFix, "/gnss/fix", 10)
        self._status_pub = self.create_publisher(GnssStatus, "/gnss/status", 10)
        self.create_subscription(
            UInt8MultiArray, "/gnss/rtcm", self._on_rtcm, 20
        )

        self._latitude = math.nan
        self._longitude = math.nan
        self._altitude = math.nan
        self._fix_quality = 0
        self._satellites = 0
        self._hdop = math.nan
        self._correction_age = math.nan
        self._speed = 0.0
        self._track = 0.0
        self._heading = math.nan
        self._heading_valid = False
        self._position_valid = False
        self._velocity_valid = False
        self._position_rx_sec: float | None = None
        self._velocity_rx_sec: float | None = None
        self._heading_rx_sec: float | None = None
        self._last_checksum_valid = False
        self._rx_buffer = bytearray()

        if not self._configure_receiver:
            self.get_logger().info("UM982 automatic configuration is disabled")
        self._try_open_serial(force=True)
        self.create_timer(0.005, self._poll_serial)

    def destroy_node(self):  # type: ignore[override]
        with self._lock:
            handle = self._serial
            self._serial = None
        try:
            if handle is not None:
                handle.close()
        finally:
            return super().destroy_node()

    def _try_open_serial(self, force: bool = False) -> bool:
        if self._serial is not None:
            return True

        now_sec = time.monotonic()
        if not force and now_sec < self._next_reconnect_sec:
            return False

        try:
            handle = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=0.0,
                write_timeout=0.2,
            )
        except (serial.SerialException, OSError) as error:
            self._next_reconnect_sec = now_sec + self._serial_reconnect_sec
            self._log_serial_error(
                f"UM982 serial open failed ({self._port}): {error}; "
                f"retrying every {self._serial_reconnect_sec:.1f} s"
            )
            return False

        with self._lock:
            self._serial = handle
        self._rx_buffer.clear()
        self._next_reconnect_sec = 0.0
        self._last_serial_error_log_sec = -math.inf
        self.get_logger().info(
            f"UM982 serial connected: {self._port} @ {self._baudrate} baud"
        )

        if self._configure_receiver:
            for command in self._startup_commands:
                payload = (command + "\r\n").encode("ascii")
                if not self._write(payload):
                    return False
            self.get_logger().warning(
                "UM982 startup commands were sent; verify firmware command responses"
            )
        return True

    def _drop_serial(self, operation: str, error: Exception) -> None:
        with self._lock:
            handle = self._serial
            self._serial = None
        if handle is not None:
            try:
                handle.close()
            except (serial.SerialException, OSError):
                pass

        self._rx_buffer.clear()
        self._fix_quality = 0
        self._satellites = 0
        self._hdop = math.nan
        self._correction_age = math.nan
        self._heading_valid = False
        self._position_valid = False
        self._velocity_valid = False
        self._position_rx_sec = None
        self._velocity_rx_sec = None
        self._heading_rx_sec = None
        self._next_reconnect_sec = time.monotonic() + self._serial_reconnect_sec
        self._publish_status()
        self._log_serial_error(
            f"UM982 serial {operation} failed: {error}; waiting for USB reconnect"
        )

    def _log_serial_error(self, message: str) -> None:
        now_sec = time.monotonic()
        if now_sec - self._last_serial_error_log_sec >= 5.0:
            self.get_logger().error(message)
            self._last_serial_error_log_sec = now_sec

    def _write(self, payload: bytes) -> bool:
        with self._lock:
            handle = self._serial
        if handle is None:
            return False
        try:
            handle.write(payload)
        except (serial.SerialException, OSError) as error:
            self._drop_serial("write", error)
            return False
        return True

    def _on_rtcm(self, message: UInt8MultiArray) -> None:
        if not message.data:
            return
        self._write(bytes(message.data))

    def _poll_serial(self) -> None:
        if not self._try_open_serial():
            return
        with self._lock:
            handle = self._serial
        if handle is None:
            return
        try:
            waiting = handle.in_waiting
            if waiting > 0:
                self._rx_buffer.extend(handle.read(min(waiting, 4096)))
            for _ in range(50):
                if b"\n" not in self._rx_buffer:
                    break
                raw, _, remainder = self._rx_buffer.partition(b"\n")
                self._rx_buffer = bytearray(remainder)
                try:
                    line = raw.decode("ascii", errors="strict").strip()
                except UnicodeDecodeError:
                    continue
                self._last_checksum_valid = validate_sentence(line)
                message = parse_sentence(line, self._require_checksum)
                if message is not None:
                    self._handle_nmea(message)
            if len(self._rx_buffer) > 16384:
                self._rx_buffer.clear()
                self.get_logger().warning("discarded overlong UM982 serial input")
        except (serial.SerialException, OSError) as error:
            self._drop_serial("read", error)

    def _handle_nmea(self, message) -> None:
        if isinstance(message, Gga):
            self._latitude = message.latitude_deg
            self._longitude = message.longitude_deg
            self._altitude = message.altitude_m
            self._fix_quality = message.fix_quality
            self._satellites = message.satellites
            self._hdop = message.hdop
            self._correction_age = message.correction_age_sec
            self._position_valid = (
                message.fix_quality > 0
                and all(
                    math.isfinite(value)
                    for value in (
                        message.latitude_deg,
                        message.longitude_deg,
                        message.altitude_m,
                    )
                )
            )
            self._position_rx_sec = time.monotonic()
            self._publish_status()
            self._publish_fix()
        elif isinstance(message, Rmc):
            if message.valid:
                self._speed = message.ground_speed_mps
                self._track = message.track_true_deg
                self._velocity_valid = math.isfinite(self._speed) and math.isfinite(
                    self._track
                )
                if not math.isfinite(self._latitude):
                    self._latitude = message.latitude_deg
                    self._longitude = message.longitude_deg
            else:
                self._speed = 0.0
                self._velocity_valid = False
            self._velocity_rx_sec = time.monotonic()
            self._publish_status()
        elif isinstance(message, Hdt):
            # Do not discard a recent valid dual-antenna heading because of one
            # transient THS,V sentence.  The normal age check below still makes
            # the status invalid after heading_timeout_sec without a new valid
            # solution, so a persistent ANT2/baseline fault remains fail-closed.
            if message.valid:
                self._heading = message.heading_true_deg
                self._heading_valid = True
                self._heading_rx_sec = time.monotonic()
            elif self._heading_rx_sec is None:
                self._heading = math.nan
                self._heading_valid = False
            self._publish_status()

    def _stamp(self):
        return self.get_clock().now().to_msg()

    def _publish_fix(self) -> None:
        if not (math.isfinite(self._latitude) and math.isfinite(self._longitude)):
            return
        message = NavSatFix()
        message.header.stamp = self._stamp()
        message.header.frame_id = self._frame_id
        message.status.service = NavSatStatus.SERVICE_GPS
        message.status.status = (
            NavSatStatus.STATUS_FIX if self._fix_quality > 0 else NavSatStatus.STATUS_NO_FIX
        )
        message.latitude = self._latitude
        message.longitude = self._longitude
        message.altitude = self._altitude
        horizontal_sigma = {
            4: 0.02,
            5: 0.30,
            2: 0.50,
            1: 1.50,
        }.get(self._fix_quality, 10.0)
        vertical_sigma = max(horizontal_sigma * 1.8, 0.04)
        message.position_covariance[0] = horizontal_sigma**2
        message.position_covariance[4] = horizontal_sigma**2
        message.position_covariance[8] = vertical_sigma**2
        message.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self._fix_pub.publish(message)

    def _publish_status(self, stamp=None) -> None:
        message = GnssStatus()
        message.header.stamp = stamp or self._stamp()
        message.header.frame_id = self._frame_id
        message.fix_type = max(0, min(255, self._fix_quality))
        now_sec = time.monotonic()
        position_age_sec = (
            now_sec - self._position_rx_sec
            if self._position_rx_sec is not None
            else math.inf
        )
        velocity_age_sec = (
            now_sec - self._velocity_rx_sec
            if self._velocity_rx_sec is not None
            else math.inf
        )
        heading_age_sec = (
            now_sec - self._heading_rx_sec
            if self._heading_rx_sec is not None
            else math.inf
        )
        heading_valid = self._heading_valid and heading_age_sec <= float(
            self.get_parameter("heading_timeout_sec").value
        )
        message.heading_valid = heading_valid
        message.position_valid = self._position_valid and position_age_sec <= float(
            self.get_parameter("position_timeout_sec").value
        )
        message.velocity_valid = self._velocity_valid and velocity_age_sec <= float(
            self.get_parameter("velocity_timeout_sec").value
        )
        message.heading_true_deg = self._heading if heading_valid else math.nan
        message.ground_speed_mps = self._speed
        message.track_true_deg = self._track
        message.satellites = self._satellites
        message.hdop = float(self._hdop)
        message.correction_age_sec = float(self._correction_age)
        message.position_age_sec = float(position_age_sec)
        message.velocity_age_sec = float(velocity_age_sec)
        message.heading_age_sec = float(heading_age_sec)
        message.nmea_checksum_valid = self._last_checksum_valid
        self._status_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Um982SerialNode()
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
