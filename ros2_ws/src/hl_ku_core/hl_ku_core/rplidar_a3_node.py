"""Minimal RPLIDAR A3 serial driver publishing sensor_msgs/LaserScan."""

from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.node import Node
import serial
from sensor_msgs.msg import LaserScan


SYNC_BYTE = 0xA5
CMD_STOP = bytes((SYNC_BYTE, 0x25))
CMD_SCAN = bytes((SYNC_BYTE, 0x20))
SCAN_DESCRIPTOR = bytes((0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81))


class RplidarA3Node(Node):
    def __init__(self) -> None:
        super().__init__("rplidar_a3")
        self.declare_parameter("serial_port", "/tmp/nucleo-lidar")
        self.declare_parameter("serial_baudrate", 256000)
        self.declare_parameter("frame_id", "laser")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("range_min_m", 0.15)
        self.declare_parameter("range_max_m", 25.0)
        self.declare_parameter("bins", 720)

        self._publisher = self.create_publisher(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            10,
        )
        self._stop_event = threading.Event()
        self._serial: serial.Serial | None = None
        self._reader = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader.start()

    def _reader_loop(self) -> None:
        port = str(self.get_parameter("serial_port").value)
        baudrate = int(self.get_parameter("serial_baudrate").value)
        while not self._stop_event.is_set():
            try:
                with serial.Serial(
                    port,
                    baudrate,
                    timeout=0.1,
                    write_timeout=0.5,
                ) as device:
                    self._serial = device
                    self._start_scan(device)
                    self.get_logger().info(
                        f"RPLIDAR scan started on {port} at {baudrate} baud"
                    )
                    self._read_scans(device)
            except (serial.SerialException, OSError, RuntimeError) as error:
                if not self._stop_event.is_set():
                    self.get_logger().error(f"RPLIDAR connection failed: {error}")
                    self._stop_event.wait(1.0)
            finally:
                self._serial = None

    def _start_scan(self, device: serial.Serial) -> None:
        device.write(CMD_STOP)
        time.sleep(0.05)
        device.reset_input_buffer()
        device.write(CMD_SCAN)

        descriptor = bytearray()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not self._stop_event.is_set():
            value = device.read(1)
            if not value:
                continue
            descriptor.extend(value)
            sync_index = descriptor.find(b"\xA5\x5A")
            if sync_index < 0:
                if len(descriptor) > 1:
                    del descriptor[:-1]
                continue
            if sync_index:
                del descriptor[:sync_index]
            if len(descriptor) < len(SCAN_DESCRIPTOR):
                continue
            received = bytes(descriptor[: len(SCAN_DESCRIPTOR)])
            if received != SCAN_DESCRIPTOR:
                raise RuntimeError(
                    f"unexpected scan descriptor: {received.hex(' ')}"
                )
            return
        raise RuntimeError("timed out waiting for RPLIDAR scan descriptor")

    def _read_scans(self, device: serial.Serial) -> None:
        bin_count = max(90, int(self.get_parameter("bins").value))
        ranges = [math.inf] * bin_count
        intensities = [0.0] * bin_count
        angle_min = -math.pi
        angle_increment = 2.0 * math.pi / bin_count
        previous_revolution_time = time.monotonic()
        have_revolution = False
        valid_points = 0
        buffer = bytearray()

        while not self._stop_event.is_set():
            chunk = device.read(4096)
            if not chunk:
                continue
            buffer.extend(chunk)
            while len(buffer) >= 5:
                first, second = buffer[0], buffer[1]
                start = first & 0x01
                inverse_start = (first >> 1) & 0x01
                if start == inverse_start or not (second & 0x01):
                    del buffer[0]
                    continue

                node = buffer[:5]
                del buffer[:5]
                quality = node[0] >> 2
                angle_deg = ((node[1] >> 1) | (node[2] << 7)) / 64.0
                distance_m = (node[3] | (node[4] << 8)) / 4000.0

                if start:
                    now = time.monotonic()
                    if have_revolution and valid_points:
                        self._publish_scan(
                            ranges,
                            intensities,
                            angle_min,
                            angle_increment,
                            now - previous_revolution_time,
                        )
                    ranges = [math.inf] * bin_count
                    intensities = [0.0] * bin_count
                    valid_points = 0
                    previous_revolution_time = now
                    have_revolution = True

                if not have_revolution or distance_m <= 0.0:
                    continue

                # RPLIDAR angles increase clockwise. ROS angles increase CCW.
                ros_angle = ((-math.radians(angle_deg) + math.pi) % (2.0 * math.pi)) - math.pi
                index = int((ros_angle - angle_min) / angle_increment) % bin_count
                if distance_m < ranges[index]:
                    ranges[index] = distance_m
                    intensities[index] = float(quality)
                valid_points += 1

    def _publish_scan(
        self,
        ranges: list[float],
        intensities: list[float],
        angle_min: float,
        angle_increment: float,
        scan_time: float,
    ) -> None:
        message = LaserScan()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(self.get_parameter("frame_id").value)
        message.angle_min = angle_min
        message.angle_max = math.pi
        message.angle_increment = angle_increment
        message.scan_time = max(0.0, scan_time)
        message.time_increment = message.scan_time / len(ranges)
        message.range_min = float(self.get_parameter("range_min_m").value)
        message.range_max = float(self.get_parameter("range_max_m").value)
        message.ranges = ranges
        message.intensities = intensities
        self._publisher.publish(message)

    def close(self) -> None:
        if self._stop_event.is_set():
            return
        self._stop_event.set()
        device = self._serial
        if device is not None and device.is_open:
            try:
                device.write(CMD_STOP)
            except (serial.SerialException, OSError):
                pass
        self._reader.join(timeout=1.0)

    def destroy_node(self) -> bool:
        self.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RplidarA3Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
