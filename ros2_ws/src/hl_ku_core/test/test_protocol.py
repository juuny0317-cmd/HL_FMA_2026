import struct

import pytest

from hl_ku_core.protocol import (
    COMMAND,
    FEEDBACK,
    FEEDBACK_WITHOUT_CRC,
    FLAG_BRAKE,
    FLAG_ENABLE,
    MAGIC,
    TYPE_FEEDBACK,
    VERSION,
    crc32,
    pack_command,
    unpack_feedback,
)


def test_command_packet_size_crc_and_flags():
    packet = pack_command(7, 0.25, -0.1, True, False)
    assert len(packet) == COMMAND.size == 24
    assert struct.unpack("<I", packet[-4:])[0] == crc32(packet[:-4])
    values = COMMAND.unpack(packet)
    assert values[0] == MAGIC
    assert values[3] & FLAG_ENABLE
    assert not values[3] & FLAG_BRAKE


def test_feedback_decode():
    body = FEEDBACK_WITHOUT_CRC.pack(
        MAGIC,
        VERSION,
        TYPE_FEEDBACK,
        FLAG_ENABLE,
        99,
        0.12,
        0.15,
        0.31,
        22.4,
        0,
    )
    packet = body + struct.pack("<I", crc32(body))
    assert len(packet) == FEEDBACK.size == 36
    feedback = unpack_feedback(packet)
    assert feedback.sequence == 99
    assert abs(feedback.battery_voltage - 22.4) < 1.0e-5


def test_nonfinite_feedback_is_rejected():
    body = FEEDBACK_WITHOUT_CRC.pack(
        MAGIC,
        VERSION,
        TYPE_FEEDBACK,
        FLAG_ENABLE,
        99,
        float("nan"),
        0.15,
        0.31,
        22.4,
        0,
    )
    with pytest.raises(ValueError, match="non-finite"):
        unpack_feedback(body + struct.pack("<I", crc32(body)))
