import pytest

from hl_ku_core.ntrip_stream import NtripBodyDecoder, response_is_chunked


def test_chunked_rtcm_is_unframed_across_socket_reads():
    decoder = NtripBodyDecoder(chunked=True)
    assert decoder.feed(b"4\r\n\xd3\x00") == ()
    assert decoder.feed(b"\x01\x02\r\n3;part=2\r\n\xd3") == (b"\xd3\x00\x01\x02",)
    assert decoder.feed(b"\x03\x04\r\n") == (b"\xd3\x03\x04",)


def test_plain_ntrip_stream_passes_rtcm_unchanged():
    decoder = NtripBodyDecoder(chunked=False)
    assert decoder.feed(b"\xd3\x00\x01") == (b"\xd3\x00\x01",)
    assert decoder.feed(b"") == ()


def test_response_header_selects_chunked_decoder():
    assert response_is_chunked(b"HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n")
    assert not response_is_chunked(b"ICY 200 OK\r\nContent-Type: gnss/data\r\n")


def test_invalid_chunk_framing_is_rejected():
    with pytest.raises(ConnectionError, match="terminator"):
        NtripBodyDecoder(chunked=True).feed(b"2\r\nABXX")
    with pytest.raises(ConnectionError, match="ended"):
        NtripBodyDecoder(chunked=True).feed(b"0\r\n\r\n")
