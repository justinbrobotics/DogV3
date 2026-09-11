"""ESP32 dual-UART mux framing.

The ESP32 is a transparent bridge: each Feetech packet (built by
:mod:`dogv3.driver.protocol`) is wrapped in a mux frame that selects which
hardware bus it goes out on. The ESP32 unwraps and writes the raw payload to the
chosen UART, and wraps any servo reply back toward the host.

Wire format (matches ``firmware/dogv3_mux``):

    host -> ESP32:  0xFE  BUS  LEN  payload[LEN]  CKSUM
    ESP32 -> host:  0xFD  BUS  LEN  payload[LEN]  CKSUM

    BUS:    0 = bus A (rear, UART1), 1 = bus B (front, UART2)
    LEN:    single byte, length of payload (Feetech packet), 0..255
    CKSUM:  (BUS + LEN + sum(payload)) & 0xFF

ASCII control commands (``VERSION`` / ``PING`` / ``SCAN``) are sent as plain
newline-terminated text — they never begin with 0xFE/0xFD so they cannot collide
with a mux frame. See :mod:`dogv3.setup_program.verify_firmware`.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum

START_HOST_TO_ESP = 0xFE
START_ESP_TO_HOST = 0xFD


class Bus(IntEnum):
    A = 0  # rear  (UART1) — BL, BR, IDs < 30
    B = 1  # front (UART2) — FL, FR, IDs >= 30

    @classmethod
    def from_name(cls, name: str) -> "Bus":
        return cls.A if name.upper() == "A" else cls.B

    @property
    def name_letter(self) -> str:
        return "A" if self is Bus.A else "B"


def _mux_checksum(bus: int, payload: bytes) -> int:
    return (bus + len(payload) + sum(payload)) & 0xFF


def encode(bus: Bus, payload: bytes, *, start: int = START_HOST_TO_ESP) -> bytes:
    """Wrap *payload* (a raw Feetech packet) in a mux frame for *bus*."""
    if len(payload) > 0xFF:
        raise ValueError(f"payload too long for one mux frame: {len(payload)} bytes")
    return bytes([start, int(bus), len(payload)]) + payload + bytes([_mux_checksum(int(bus), payload)])


@dataclass
class MuxFrame:
    bus: Bus
    payload: bytes


class MuxDecodeError(Exception):
    pass


def decode(frame: bytes, *, start: int = START_ESP_TO_HOST) -> MuxFrame:
    """Decode a single complete mux frame (validates checksum)."""
    if len(frame) < 4:
        raise MuxDecodeError("mux frame too short")
    if frame[0] != start:
        raise MuxDecodeError(f"bad mux start byte 0x{frame[0]:02x}")
    bus = frame[1]
    length = frame[2]
    if len(frame) != 4 + length:
        raise MuxDecodeError(f"mux length mismatch: declared {length}, got {len(frame) - 4}")
    payload = frame[3:-1]
    if _mux_checksum(bus, payload) != frame[-1]:
        raise MuxDecodeError("mux checksum mismatch")
    return MuxFrame(bus=Bus(bus), payload=payload)


class MuxStreamParser:
    """Incremental parser for the ESP32 -> host byte stream. Feed raw bytes; it
    yields complete :class:`MuxFrame` objects as they arrive. Bytes that are not
    part of a mux frame (e.g. ASCII command replies) are surfaced via
    :attr:`text_bytes`."""

    def __init__(self, start: int = START_ESP_TO_HOST):
        self._start = start
        self._buf = bytearray()
        self.text_bytes = bytearray()

    def feed(self, data: bytes) -> list[MuxFrame]:
        frames: list[MuxFrame] = []
        for byte in data:
            self._buf.append(byte)
            frames.extend(self._try_extract())
        return frames

    def _try_extract(self) -> list[MuxFrame]:
        out: list[MuxFrame] = []
        while self._buf:
            # Drop leading non-start bytes into the text channel.
            if self._buf[0] != self._start:
                self.text_bytes.append(self._buf.pop(0))
                continue
            if len(self._buf) < 3:
                break  # need at least start, bus, len
            length = self._buf[2]
            total = 4 + length
            if len(self._buf) < total:
                break  # wait for the rest of the frame
            candidate = bytes(self._buf[:total])
            try:
                out.append(decode(candidate, start=self._start))
                del self._buf[:total]
            except MuxDecodeError:
                # Not a valid frame after all — resync past this start byte.
                self.text_bytes.append(self._buf.pop(0))
        return out
