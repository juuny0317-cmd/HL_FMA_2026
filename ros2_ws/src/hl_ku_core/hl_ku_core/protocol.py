"""Binary UDP protocol shared with the NUCLEO firmware."""

from __future__ import annotations

import dataclasses
import math
import struct
import zlib


MAGIC = 0x554B4C48  # bytes on wire: H L K U
VERSION = 1
TYPE_COMMAND = 1
TYPE_FEEDBACK = 2

FLAG_ENABLE = 1 << 0
FLAG_BRAKE = 1 << 1

COMMAND_WITHOUT_CRC = struct.Struct("<IBBHIff")
COMMAND = struct.Struct("<IBBHIffI")
FEEDBACK_WITHOUT_CRC = struct.Struct("<IBBHIffffI")
FEEDBACK = struct.Struct("<IBBHIffffII")


@dataclasses.dataclass(frozen=True)
class FeedbackPacket:
    flags: int
    sequence: int
    steering_angle_rad: float
    steering_target_rad: float
    applied_drive_duty: float
    battery_voltage: float
    fault_flags: int


def crc32(data: bytes) -> int:
    return zlib.crc32(data) & 0xFFFFFFFF


def pack_command(
    sequence: int,
    drive_duty: float,
    steering_angle_rad: float,
    enable: bool,
    brake: bool,
) -> bytes:
    flags = (FLAG_ENABLE if enable else 0) | (FLAG_BRAKE if brake else 0)
    body = COMMAND_WITHOUT_CRC.pack(
        MAGIC,
        VERSION,
        TYPE_COMMAND,
        flags,
        sequence & 0xFFFFFFFF,
        float(drive_duty),
        float(steering_angle_rad),
    )
    return body + struct.pack("<I", crc32(body))


def unpack_feedback(data: bytes) -> FeedbackPacket:
    if len(data) != FEEDBACK.size:
        raise ValueError(f"feedback packet must be {FEEDBACK.size} bytes")
    body, received_crc = data[:-4], struct.unpack("<I", data[-4:])[0]
    if crc32(body) != received_crc:
        raise ValueError("feedback CRC mismatch")
    (
        magic,
        version,
        packet_type,
        flags,
        sequence,
        steering,
        target,
        duty,
        voltage,
        faults,
        _crc,
    ) = FEEDBACK.unpack(data)
    if magic != MAGIC or version != VERSION or packet_type != TYPE_FEEDBACK:
        raise ValueError("feedback header mismatch")
    if flags & ~(FLAG_ENABLE | FLAG_BRAKE):
        raise ValueError("feedback contains unknown flags")
    if not all(math.isfinite(value) for value in (steering, target, duty, voltage)):
        raise ValueError("feedback contains non-finite values")
    if abs(duty) > 1.0:
        raise ValueError("feedback drive duty is outside [-1, 1]")
    return FeedbackPacket(flags, sequence, steering, target, duty, voltage, faults)
