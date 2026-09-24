#!/usr/bin/env python3
"""Expose steering and RPLIDAR virtual serial ports over one Nucleo ST-LINK VCP."""

from __future__ import annotations

import argparse
import errno
import os
import pty
import select
import signal
import sys
import time
import tty
from dataclasses import dataclass, field
from pathlib import Path

import serial

END = 0xC0
ESC = 0xDB
ESC_END = 0xDC
ESC_ESC = 0xDD
CONTROL = 1
LIDAR = 2
MAX_PAYLOAD = 512
MAX_FRAME = MAX_PAYLOAD + 3
MAX_PENDING = 65536
DEFAULT_PORT = (
    "/dev/serial/by-id/"
    "usb-STMicroelectronics_STLINK-V3_004700233434511834313937-if02"
)


def crc16(data: bytes | bytearray) -> int:
    value = 0xFFFF
    for byte in data:
        value ^= byte << 8
        for _ in range(8):
            value = ((value << 1) ^ 0x1021) & 0xFFFF if value & 0x8000 else (value << 1) & 0xFFFF
    return value


def encode_frame(channel: int, payload: bytes) -> bytes:
    if channel not in (CONTROL, LIDAR) or len(payload) > MAX_PAYLOAD:
        raise ValueError("invalid channel or payload length")
    frame = bytes((channel,)) + payload
    frame += crc16(frame).to_bytes(2, "little")
    out = bytearray((END,))
    for byte in frame:
        if byte == END:
            out.extend((ESC, ESC_END))
        elif byte == ESC:
            out.extend((ESC, ESC_ESC))
        else:
            out.append(byte)
    out.append(END)
    return bytes(out)


class Decoder:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.escaped = False
        self.bad = False
        self.invalid_frames = 0

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        frames: list[tuple[int, bytes]] = []
        for byte in data:
            if byte == END:
                if not self.escaped and not self.bad and len(self.buffer) >= 3:
                    received = int.from_bytes(self.buffer[-2:], "little")
                    if crc16(self.buffer[:-2]) == received and self.buffer[0] in (CONTROL, LIDAR):
                        frames.append((self.buffer[0], bytes(self.buffer[1:-2])))
                    else:
                        self.invalid_frames += 1
                self.buffer.clear()
                self.escaped = False
                self.bad = False
                continue
            if self.bad:
                continue
            if self.escaped:
                self.escaped = False
                if byte == ESC_END:
                    byte = END
                elif byte == ESC_ESC:
                    byte = ESC
                else:
                    self.bad = True
                    continue
            elif byte == ESC:
                self.escaped = True
                continue
            if len(self.buffer) < MAX_FRAME:
                self.buffer.append(byte)
            else:
                self.bad = True
        return frames


@dataclass
class VirtualPort:
    link: Path
    master: int
    slave: int
    target: str
    pending: bytearray = field(default_factory=bytearray)
    active: bool = False
    dropped: int = 0

    @classmethod
    def create(cls, link: Path) -> "VirtualPort":
        if link.exists() and not link.is_symlink():
            raise FileExistsError(f"refusing to replace existing path: {link}")
        master, slave = pty.openpty()
        tty.setraw(slave)
        os.set_blocking(master, False)
        target = os.ttyname(slave)
        if link.is_symlink():
            link.unlink()
        link.symlink_to(target)
        return cls(link, master, slave, target)

    def close(self) -> None:
        if self.link.is_symlink() and os.readlink(self.link) == self.target:
            self.link.unlink()
        os.close(self.master)
        os.close(self.slave)

    def queue(self, data: bytes) -> None:
        if not self.active:
            return
        if len(self.pending) + len(data) > MAX_PENDING:
            self.dropped += len(data)
        else:
            self.pending.extend(data)

    def flush(self) -> None:
        if not self.pending:
            return
        try:
            written = os.write(self.master, self.pending)
            del self.pending[:written]
        except BlockingIOError:
            pass
        except OSError as error:
            if error.errno != errno.EIO:
                raise
            self.pending.clear()
            self.active = False


def run(port: str, baud: int, control_link: Path, lidar_link: Path) -> None:
    ports: list[VirtualPort] = []
    wire = None
    stopping = False

    def stop(_signum: int, _frame: object) -> None:
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        wire = serial.Serial(port, baudrate=baud, timeout=0, write_timeout=0.1, exclusive=True)
        wire.reset_input_buffer()
        ports = [VirtualPort.create(control_link), VirtualPort.create(lidar_link)]
        by_channel = {CONTROL: ports[0], LIDAR: ports[1]}
        decoder = Decoder()
        print(f"physical: {port} @ {baud} 8N1", flush=True)
        print(f"control:  {ports[0].link} -> {ports[0].target}", flush=True)
        print(f"lidar:    {ports[1].link} -> {ports[1].target}", flush=True)
        last_report = time.monotonic()
        last_invalid = 0
        while not stopping:
            readable = [wire.fileno(), *(item.master for item in ports)]
            writable = [item.master for item in ports if item.pending]
            ready_read, ready_write, _ = select.select(readable, writable, [], 0.05)
            if wire.fileno() in ready_read:
                data = wire.read(wire.in_waiting or 1)
                for channel, payload in decoder.feed(data):
                    by_channel[channel].queue(payload)
            for channel, item in by_channel.items():
                if item.master in ready_read:
                    try:
                        data = os.read(item.master, 4096)
                    except BlockingIOError:
                        continue
                    if data:
                        item.active = True
                        for start in range(0, len(data), MAX_PAYLOAD):
                            wire.write(encode_frame(channel, data[start:start + MAX_PAYLOAD]))
                if item.master in ready_write or item.pending:
                    item.flush()
            now = time.monotonic()
            if now - last_report >= 5.0:
                if decoder.invalid_frames != last_invalid or any(p.dropped for p in ports):
                    print(
                        f"invalid_frames={decoder.invalid_frames} "
                        f"dropped_control={ports[0].dropped} dropped_lidar={ports[1].dropped}",
                        file=sys.stderr,
                        flush=True,
                    )
                    last_invalid = decoder.invalid_frames
                last_report = now
    finally:
        for item in ports:
            item.close()
        if wire is not None:
            wire.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--control-link", type=Path, default=Path("/tmp/nucleo-control"))
    parser.add_argument("--lidar-link", type=Path, default=Path("/tmp/nucleo-lidar"))
    args = parser.parse_args()
    run(args.port, args.baud, args.control_link, args.lidar_link)


if __name__ == "__main__":
    main()
