"""Host-side Feetech driver over the ESP32 mux.

High-level operations the Setup and Runtime programs use:
``scan / ping / move / read / calibrate-ofs`` plus the hot-loop
``sync_write_positions``. Each call builds a packet with
:mod:`dogv3.driver.protocol`, wraps it for the target bus with
:mod:`dogv3.driver.mux`, and (for replies) parses the response.

The transport is abstracted so tests can drive a fake loopback without serial
hardware. On hardware, :class:`SerialTransport` wraps ``pyserial``.
"""
from __future__ import annotations

import threading
import time
from typing import Protocol

from . import protocol as P
from .mux import Bus, MuxFrame, MuxStreamParser, encode


class Transport(Protocol):
    """Byte-stream transport to the ESP32 (single USB-CDC serial link)."""

    def write(self, data: bytes) -> None: ...
    def read(self, max_bytes: int = 4096) -> bytes: ...
    def reset_input(self) -> None: ...


class SerialTransport:
    """``pyserial``-backed transport. Imported lazily so the package works on a
    host without ``pyserial`` for pure-logic tests."""

    def __init__(self, port: str, baudrate: int = 1_000_000, timeout: float = 0.05):
        import serial  # type: ignore

        # Deassert DTR/RTS BEFORE opening: dev-board auto-reset wiring ties them
        # to EN/GPIO0, so pyserial's default (both asserted on open) reboots the
        # ESP32 on every connect — a ~1 s bus outage per session/service restart.
        # Constructing with port=None defers the open until the lines are set.
        self._ser = serial.Serial(port=None, baudrate=baudrate, timeout=timeout)
        self._ser.port = port
        self._ser.dtr = False
        self._ser.rts = False
        self._ser.open()

    def write(self, data: bytes) -> None:
        self._ser.write(data)

    def read(self, max_bytes: int = 4096) -> bytes:
        waiting = getattr(self._ser, "in_waiting", 0)
        return self._ser.read(max(1, min(max_bytes, waiting or 1)))

    def reset_input(self) -> None:
        try:
            self._ser.reset_input_buffer()
        except Exception:
            pass

    def close(self) -> None:
        self._ser.close()


class TcpTransport:
    """TCP transport to the ESP32's WiFi SoftAP bridge (the ``esp32-wifi``
    firmware). Wire-identical to the serial link — the ESP32 pipes the same mux
    frames over a raw TCP socket — so the whole driver stack is unchanged; only
    the bytes travel over WiFi instead of USB.

    Default target is the SoftAP gateway ``192.168.4.1:3333``."""

    def __init__(self, host: str = "192.168.4.1", port: int = 3333,
                 connect_timeout: float = 5.0, read_timeout: float = 0.05):
        import socket

        self._socket = socket
        self._read_timeout = read_timeout
        self._sock = socket.create_connection((host, port), timeout=connect_timeout)
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # low-latency, no Nagle
        self._sock.settimeout(read_timeout)

    def write(self, data: bytes) -> None:
        self._sock.sendall(data)

    def read(self, max_bytes: int = 4096) -> bytes:
        try:
            return self._sock.recv(max_bytes)
        except (self._socket.timeout, TimeoutError, BlockingIOError):
            return b""

    def reset_input(self) -> None:
        self._sock.setblocking(False)
        try:
            while True:
                if not self._sock.recv(4096):
                    break
        except (BlockingIOError, OSError):
            pass
        finally:
            self._sock.settimeout(self._read_timeout)

    def close(self) -> None:
        try:
            self._sock.close()
        except Exception:
            pass


def make_transport(port_serial: str | None = None, tcp: str | None = None,
                   read_timeout: float = 0.05) -> "Transport | None":
    """Build a transport from CLI options. ``tcp`` is ``HOST[:PORT]`` (defaults
    to the SoftAP ``192.168.4.1:3333``); ``port_serial`` is a COM/tty path.
    Returns None if neither is given (dry mode)."""
    if tcp:
        host, _, port = tcp.partition(":")
        return TcpTransport(host or "192.168.4.1", int(port) if port else 3333,
                            read_timeout=read_timeout)
    if port_serial:
        return SerialTransport(port_serial, timeout=read_timeout)
    return None


class Feedback:
    """Decoded FeedBack block (PRESENT_POSITION_L .. PRESENT_CURRENT_H)."""

    __slots__ = ("position", "speed", "load", "voltage", "temperature", "moving", "current")

    def __init__(self, block: bytes):
        self.position = P.decode_position(block)
        self.speed = P.decode_speed(block)
        self.load = P.decode_load(block)
        self.voltage = P.decode_voltage(block)
        self.temperature = P.decode_temperature(block)
        self.moving = P.decode_moving(block)
        self.current = P.decode_current(block)

    def as_dict(self) -> dict:
        return {
            "position": self.position,
            "speed": self.speed,
            "load": self.load,
            "voltage": self.voltage,
            "temperature": self.temperature,
            "moving": self.moving,
            "current": self.current,
        }


class FeetechDriver:
    """Thread-safe (single lock) driver. One instance owns the ESP32 link and
    talks to both buses through the mux."""

    def __init__(self, transport: Transport, *, reply_timeout: float = 0.05):
        self._t = transport
        self._lock = threading.Lock()
        self._reply_timeout = reply_timeout
        self._parser = MuxStreamParser()
        self._pending_frames: list[MuxFrame] = []
        self._diag = self._new_diag()

    @staticmethod
    def _new_diag() -> dict[str, int]:
        return {
            "tx": 0,
            "rx_bytes": 0,
            "mux_frames": 0,
            "parse_errors": 0,
            "wrong_ids": 0,
            "timeouts": 0,
        }

    def diagnostics(self) -> dict[str, int]:
        return {
            **self._diag,
            "pending_frames": len(self._pending_frames),
            "text_bytes": len(self._parser.text_bytes),
            # First bytes that arrived OUTSIDE any mux frame. When reads time
            # out with mux_frames at zero this says what the ESP32 actually
            # sent instead of a framed reply, which is the difference between
            # "nothing came back" and "something came back unframed".
            "text_head": repr(bytes(self._parser.text_bytes[:48])),
        }

    # -- low level -------------------------------------------------------
    def _send(self, bus: Bus, packet: bytes) -> None:
        self._t.write(encode(bus, packet))

    def reset_input(self) -> None:
        """Drop stale host input and all partially parsed/pending mux frames.

        The serial buffer and parser are one receive state.  Clearing only the
        OS buffer can leave a partial frame in ``MuxStreamParser`` that poisons
        every later transaction even though ``in_waiting`` reads zero.
        """
        with self._lock:
            self._drop_receive_state()
            self._diag = self._new_diag()

    # Enough newlines to walk any stuck mid-frame state out of the firmware's
    # parser: our payloads are Feetech packets of ~8-15 bytes, so a partial
    # frame completes (with a bad checksum, which the firmware drops) well
    # inside this. In IDLE a newline is explicitly a no-op, so extra costs
    # nothing. A payload of 0x0A bytes is not a valid Feetech packet either,
    # so even an accidental checksum match forwards something every servo
    # ignores.
    RESYNC_NEWLINES = 48
    RESYNC_WAIT_S = 0.4

    def resync(self, expected: str = "DOGV3-MUX") -> bool:
        """Force the ESP32's host-side parser back to IDLE and CONFIRM it.

        ``rxState`` in the firmware is a static global. A truncated or
        mis-aligned write strands it in ASCII_LINE (or mid-MUX), after which
        every mux frame we send is consumed as text and answered with
        ``ERR unknown cmd`` — so every read times out with ``mux_frames`` at
        zero and the link never recovers on its own. Newlines terminate
        ASCII_LINE and are ignored in IDLE, making them a safe resync token;
        VERSION then positively confirms sync instead of leaving us hoping.
        """
        with self._lock:
            self._t.write(b"\n" * self.RESYNC_NEWLINES)
            time.sleep(0.05)
            self._drop_receive_state()
            self._t.write(b"VERSION\n")
            deadline = time.monotonic() + self.RESYNC_WAIT_S
            seen = b""
            while time.monotonic() < deadline:
                chunk = self._t.read(256)
                if chunk:
                    seen += chunk
                    if expected.encode() in seen:
                        time.sleep(0.05)          # let the trailing CRLF land
                        self._drop_receive_state()
                        return True
                else:
                    time.sleep(0.005)
            self._drop_receive_state()
            return False

    def _drop_receive_state(self) -> None:
        """Clear the OS buffer, the frame parser and any pending frames.

        Caller must hold ``self._lock``. These are one receive state; clearing
        them separately leaves a partial frame behind that poisons every later
        transaction while ``in_waiting`` reads zero."""
        self._t.reset_input()
        self._parser = MuxStreamParser()
        self._pending_frames.clear()

    def _pop_pending(self, bus: Bus) -> bytes | None:
        for i, frame in enumerate(self._pending_frames):
            if frame.bus == bus:
                return self._pending_frames.pop(i).payload
        return None

    def _await_frame(self, bus: Bus, timeout: float | None = None) -> bytes | None:
        """Read until a mux frame for *bus* arrives or *timeout* elapses.
        Returns the inner Feetech payload, or None on timeout."""
        deadline = time.monotonic() + (timeout if timeout is not None else self._reply_timeout)
        while time.monotonic() < deadline:
            payload = self._pop_pending(bus)
            if payload is not None:
                return payload
            chunk = self._t.read()
            if chunk:
                self._diag["rx_bytes"] += len(chunk)
                # Preserve every decoded frame.  Returning the first item from
                # parser.feed() used to silently discard later frames from the
                # same USB read, including the response we were waiting for.
                frames = self._parser.feed(chunk)
                self._diag["mux_frames"] += len(frames)
                self._pending_frames.extend(frames)
            else:
                time.sleep(0.0005)
        return None

    def _transact(self, bus: Bus, packet: bytes, timeout: float | None = None) -> P.StatusResponse | None:
        deadline = time.monotonic() + (timeout if timeout is not None else self._reply_timeout)
        expected_id = packet[2] if len(packet) > 2 else None
        with self._lock:
            self._diag["tx"] += 1
            self._send(bus, packet)
            while time.monotonic() < deadline:
                payload = self._await_frame(bus, max(0.0, deadline - time.monotonic()))
                if payload is None:
                    self._diag["timeouts"] += 1
                    return None
                try:
                    response = P.parse_status(payload)
                except P.ProtocolError:
                    self._diag["parse_errors"] += 1
                    continue
                if expected_id is not None and response.servo_id != expected_id:
                    self._diag["wrong_ids"] += 1
                    continue
                return response
        self._diag["timeouts"] += 1
        return None

    # -- ping / scan -----------------------------------------------------
    def ping(self, bus: Bus, servo_id: int, timeout: float | None = None) -> bool:
        resp = self._transact(bus, P.ping(servo_id), timeout)
        return resp is not None and resp.servo_id == servo_id

    def scan(self, buses: list[Bus] | None = None, id_range: range = range(1, 100)) -> dict[Bus, list[int]]:
        """Ping every ID on each bus; return responding IDs per bus."""
        buses = buses or [Bus.A, Bus.B]
        found: dict[Bus, list[int]] = {b: [] for b in buses}
        for bus in buses:
            for sid in id_range:
                if self.ping(bus, sid, timeout=0.02):
                    found[bus].append(sid)
        return found

    # -- motion ----------------------------------------------------------
    def write_pos_ex(self, bus: Bus, servo_id: int, position: int, speed: int, acc: int = 0) -> bool:
        resp = self._transact(bus, P.write_pos_ex(servo_id, position, speed, acc))
        return resp is not None

    def sync_write_positions(self, plan: dict[Bus, list[tuple[int, int, int, int]]]) -> None:
        """Hot-loop multi-servo write. *plan* maps each bus to a list of
        ``(id, position, speed, acc)`` tuples. One broadcast SyncWritePosEx frame
        per bus, no replies (matches the C++ broadcast no-reply behaviour)."""
        with self._lock:
            for bus, entries in plan.items():
                if not entries:
                    continue
                ids = [e[0] for e in entries]
                positions = [e[1] for e in entries]
                speeds = [e[2] for e in entries]
                accs = [e[3] for e in entries]
                self._send(bus, P.sync_write_pos_ex(ids, positions, speeds, accs))

    # -- feedback / reads ------------------------------------------------
    def feedback(self, bus: Bus, servo_id: int) -> Feedback | None:
        resp = self._transact(bus, P.read(servo_id, P.FEEDBACK_ADDR, P.FEEDBACK_LEN))
        if resp is None or len(resp.params) != P.FEEDBACK_LEN:
            return None
        return Feedback(resp.params)

    def read_position(self, bus: Bus, servo_id: int) -> int | None:
        fb = self.feedback(bus, servo_id)
        return fb.position if fb else None

    def _read_word(self, bus: Bus, servo_id: int, addr: int) -> int | None:
        resp = self._transact(bus, P.read(servo_id, addr, 2))
        if resp is None or len(resp.params) != 2:
            return None
        return P._from_lo_hi(resp.params[0], resp.params[1])

    # -- torque / calibration -------------------------------------------
    def enable_torque(self, bus: Bus, servo_id: int, enable: bool) -> bool:
        return self._transact(bus, P.enable_torque(servo_id, enable)) is not None

    def calibration_ofs(self, bus: Bus, servo_id: int) -> bool:
        """Set current position as middle (~2048). Persistent EEPROM write."""
        return self._transact(bus, P.calibration_ofs(servo_id)) is not None

    def read_present_position_raw(self, bus: Bus, servo_id: int) -> int | None:
        return self.read_position(bus, servo_id)

    # -- EEPROM servo ID (one servo on the bus at a time) ---------------
    def write_id(self, bus: Bus, old_id: int, new_id: int) -> bool:
        """Persistently change a servo's ID: unlock EEPROM, write ``SMS_STS_ID``,
        lock, then verify the servo answers on the new ID.

        DANGER: the ID write is a unicast to *old_id*, so every servo currently
        using *old_id* takes *new_id*. Connect exactly ONE servo to the bus."""
        if not (0 <= new_id <= 253):
            raise ValueError("new_id must be 0..253")
        with self._lock:
            self._send(bus, P.unlock_eprom(old_id))
            self._await_frame(bus)
            self._send(bus, P.write_byte(old_id, P.SMS_STS_ID, new_id))
            self._await_frame(bus)
            self._send(bus, P.lock_eprom(new_id))
            self._await_frame(bus)
        return self.ping(bus, new_id)

    # -- EEPROM angle limits (hard backstop) ----------------------------
    def write_angle_limits(self, bus: Bus, servo_id: int, min_raw: int, max_raw: int) -> bool:
        """Unlock EEPROM -> write min/max angle limit (regs 9-12) -> lock ->
        verify by read-back. Returns True only if read-back matches.

        Guards min<=max: an inverted (min>max) angle-limit window breaks the STS
        position clamp so the servo drives continuously, so we always store the
        ordered pair regardless of caller."""
        if min_raw > max_raw:
            min_raw, max_raw = max_raw, min_raw
        with self._lock:
            self._send(bus, P.unlock_eprom(servo_id))
            self._await_frame(bus)
            self._send(bus, P.write_word(servo_id, P.SMS_STS_MIN_ANGLE_LIMIT_L, min_raw))
            self._await_frame(bus)
            self._send(bus, P.write_word(servo_id, P.SMS_STS_MAX_ANGLE_LIMIT_L, max_raw))
            self._await_frame(bus)
            self._send(bus, P.lock_eprom(servo_id))
            self._await_frame(bus)
        got_min = self._read_word(bus, servo_id, P.SMS_STS_MIN_ANGLE_LIMIT_L)
        got_max = self._read_word(bus, servo_id, P.SMS_STS_MAX_ANGLE_LIMIT_L)
        return got_min == min_raw and got_max == max_raw
