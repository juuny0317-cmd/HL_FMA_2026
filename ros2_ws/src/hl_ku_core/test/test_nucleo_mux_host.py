import pytest

from hl_ku_core.nucleo_mux_host import (
    CONTROL,
    END,
    ESC,
    LIDAR,
    Decoder,
    encode_frame,
)


@pytest.mark.parametrize("channel", [CONTROL, LIDAR])
def test_mux_frame_round_trip_handles_escaped_bytes(channel) -> None:
    payload = bytes((0x00, END, 0x55, ESC, 0xFF))
    encoded = encode_frame(channel, payload)
    decoder = Decoder()
    frames = []
    for split in (encoded[:2], encoded[2:5], encoded[5:]):
        frames.extend(decoder.feed(split))
    assert frames == [(channel, payload)]
    assert decoder.invalid_frames == 0


def test_mux_decoder_rejects_corrupt_frame() -> None:
    encoded = bytearray(encode_frame(CONTROL, b"CMD:STOP"))
    encoded[2] ^= 0x01
    decoder = Decoder()
    assert decoder.feed(encoded) == []
    assert decoder.invalid_frames == 1


def test_mux_encoder_rejects_unknown_channel_and_oversized_payload() -> None:
    with pytest.raises(ValueError):
        encode_frame(3, b"data")
    with pytest.raises(ValueError):
        encode_frame(CONTROL, b"x" * 513)
