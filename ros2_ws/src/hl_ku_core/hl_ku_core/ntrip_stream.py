"""Decode the HTTP transfer framing around an NTRIP RTCM stream."""

from __future__ import annotations


class NtripBodyDecoder:
    def __init__(self, chunked: bool) -> None:
        self.chunked = chunked
        self._buffer = bytearray()
        self._remaining: int | None = None

    def feed(self, data: bytes) -> tuple[bytes, ...]:
        if not self.chunked:
            return (data,) if data else ()
        self._buffer.extend(data)
        decoded: list[bytes] = []
        while True:
            if self._remaining is None:
                line_end = self._buffer.find(b"\r\n")
                if line_end < 0:
                    if len(self._buffer) > 128:
                        raise ConnectionError("NTRIP chunk header is too long")
                    break
                line = bytes(self._buffer[:line_end]).split(b";", 1)[0]
                del self._buffer[: line_end + 2]
                try:
                    self._remaining = int(line, 16)
                except ValueError as error:
                    raise ConnectionError("invalid NTRIP chunk length") from error
                if not 0 <= self._remaining <= 1_048_576:
                    raise ConnectionError("NTRIP chunk length is out of range")
                if self._remaining == 0:
                    raise ConnectionError("NTRIP caster ended the correction stream")
            if len(self._buffer) < self._remaining + 2:
                break
            if self._buffer[self._remaining : self._remaining + 2] != b"\r\n":
                raise ConnectionError("invalid NTRIP chunk terminator")
            decoded.append(bytes(self._buffer[: self._remaining]))
            del self._buffer[: self._remaining + 2]
            self._remaining = None
        return tuple(decoded)


def response_is_chunked(header: bytes) -> bool:
    return any(
        line.partition(b":")[0].strip().lower() == b"transfer-encoding"
        and b"chunked" in (
            token.strip().lower()
            for token in line.partition(b":")[2].split(b",")
        )
        for line in header.split(b"\r\n")
    )
