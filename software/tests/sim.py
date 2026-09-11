"""In-memory ESP32+servo simulator transport for hardware-free driver tests.

Speaks the same two protocols the real firmware does: mux frames (0xFE in /
0xFD out) wrapping Feetech packets, and ASCII control lines. Each simulated
servo holds a present position and answers PING / READ(feedback) / WRITE.
"""
from __future__ import annotations

from dogv3.driver import protocol as P
from dogv3.driver.mux import Bus


class SimServo:
    def __init__(self, sid: int, position: int = 2048):
        self.id = sid
        self.position = position
        self.goal = position
        self.min_angle = 0
        self.max_angle = 4095
        self.voltage = 120  # 12.0 V
        self.temperature = 30
        self.torque = False


class SimTransport:
    """Implements the Transport protocol; routes mux frames to simulated buses."""

    def __init__(self, bus_ids: dict[Bus, list[int]], firmware_version: str = "DOGV3-MUX v1.0"):
        self.servos: dict[Bus, dict[int, SimServo]] = {
            bus: {sid: SimServo(sid) for sid in ids} for bus, ids in bus_ids.items()
        }
        self.firmware_version = firmware_version
        self._out = bytearray()
        self._in = bytearray()

    # -- Transport interface --------------------------------------------
    def write(self, data: bytes) -> None:
        self._in.extend(data)
        self._process()

    def read(self, max_bytes: int = 4096) -> bytes:
        chunk = bytes(self._out[:max_bytes])
        del self._out[:max_bytes]
        return chunk

    def reset_input(self) -> None:
        # Host receive buffer: bytes emitted by the simulated ESP32 and not yet
        # consumed by the driver.  ``_in`` is host->ESP and is processed
        # synchronously by write().
        self._out.clear()

    # -- internal -------------------------------------------------------
    def _process(self) -> None:
        # Consume complete mux frames (host->ESP, start 0xFE) and ASCII lines.
        while self._in:
            if self._in[0] != 0xFE:
                newline_positions = [i for i in (self._in.find(b"\n"), self._in.find(b"\r")) if i >= 0]
                if not newline_positions:
                    return
                end = min(newline_positions)
                line = bytes(self._in[:end]).decode("ascii", errors="ignore").strip()
                while len(self._in) > end and self._in[end] in (ord("\n"), ord("\r")):
                    end += 1
                del self._in[:end]
                self._handle_ascii(line)
                continue
            if len(self._in) < 3:
                return
            length = self._in[2]
            total = 4 + length
            if len(self._in) < total:
                return
            frame = bytes(self._in[:total])
            del self._in[:total]
            bus = Bus(frame[1])
            payload = frame[3:-1]
            self._handle(bus, payload)

    def _handle_ascii(self, line: str) -> None:
        cmd = line.upper()
        if cmd == "VERSION":
            self._out.extend((self.firmware_version + "\r\n").encode("ascii"))
        elif cmd == "PING":
            self._out.extend(b"PONG\r\n")
        elif cmd == "SCAN":
            for bus in (Bus.A, Bus.B):
                ids = " ".join(str(sid) for sid in sorted(self.servos.get(bus, {})))
                suffix = f" {ids}" if ids else ""
                self._out.extend(f"SCAN {bus.name_letter}:{suffix}\r\n".encode("ascii"))
            self._out.extend(b"SCAN DONE\r\n")
        elif line:
            self._out.extend(f"ERR unknown cmd: {line}\r\n".encode("ascii", errors="replace"))

    def _handle(self, bus: Bus, packet: bytes) -> None:
        if len(packet) < 6 or packet[0] != 0xFF or packet[1] != 0xFF:
            return
        sid = packet[2]
        inst = packet[4]
        servos = self.servos.get(bus, {})

        if sid == P.BROADCAST_ID:
            if inst == P.INST_SYNC_WRITE:
                self._apply_sync_write(servos, packet)
            return  # broadcast: no reply

        servo = servos.get(sid)
        if servo is None:
            return  # nothing answers -> timeout on host side

        if inst == P.INST_PING:
            self._reply(bus, sid, b"")
        elif inst == P.INST_READ:
            addr = packet[5]
            length = packet[6]
            self._reply(bus, sid, self._read_mem(servo, addr, length))
        elif inst == P.INST_WRITE:
            addr = packet[5]
            data = packet[6:-1]
            if addr == P.SMS_STS_ID and data:
                new_id = data[0]
                if new_id != sid:
                    servos[new_id] = servo
                    servos.pop(sid, None)
                    servo.id = new_id
                self._reply(bus, sid, b"")
            else:
                self._apply_write(servo, addr, data)
                self._reply(bus, sid, b"")

    def _read_mem(self, servo: SimServo, addr: int, length: int) -> bytes:
        if addr == P.FEEDBACK_ADDR and length == P.FEEDBACK_LEN:
            block = bytearray(P.FEEDBACK_LEN)
            lo, hi = servo.position & 0xFF, (servo.position >> 8) & 0xFF
            block[P.SMS_STS_PRESENT_POSITION_L - P.FEEDBACK_ADDR] = lo
            block[P.SMS_STS_PRESENT_POSITION_H - P.FEEDBACK_ADDR] = hi
            block[P.SMS_STS_PRESENT_VOLTAGE - P.FEEDBACK_ADDR] = servo.voltage
            block[P.SMS_STS_PRESENT_TEMPERATURE - P.FEEDBACK_ADDR] = servo.temperature
            return bytes(block)
        if addr == P.SMS_STS_PRESENT_POSITION_L and length == 2:
            return bytes([servo.position & 0xFF, (servo.position >> 8) & 0xFF])
        if addr == P.SMS_STS_MIN_ANGLE_LIMIT_L and length == 2:
            return bytes([servo.min_angle & 0xFF, (servo.min_angle >> 8) & 0xFF])
        if addr == P.SMS_STS_MAX_ANGLE_LIMIT_L and length == 2:
            return bytes([servo.max_angle & 0xFF, (servo.max_angle >> 8) & 0xFF])
        return bytes(length)

    def _apply_write(self, servo: SimServo, addr: int, data: bytes) -> None:
        if addr == P.SMS_STS_TORQUE_ENABLE and data:
            if data[0] == P.TORQUE_CALIBRATE_MIDDLE:
                servo.position = 2048  # CalibrationOfs -> reads ~2048
            else:
                servo.torque = bool(data[0])
        elif addr == P.SMS_STS_ACC and len(data) >= 3:
            # WritePosEx payload: [acc, posL, posH, ...]
            pos = P._from_lo_hi(data[1], data[2])
            servo.goal = P.decode_sign_magnitude(pos)
            servo.position = max(0, min(4095, servo.goal))
        elif addr == P.SMS_STS_MIN_ANGLE_LIMIT_L and len(data) >= 2:
            servo.min_angle = P._from_lo_hi(data[0], data[1])
        elif addr == P.SMS_STS_MAX_ANGLE_LIMIT_L and len(data) >= 2:
            servo.max_angle = P._from_lo_hi(data[0], data[1])

    def _apply_sync_write(self, servos: dict[int, SimServo], packet: bytes) -> None:
        data_len = packet[6]
        body = packet[7:-1]
        stride = data_len + 1
        for i in range(0, len(body), stride):
            sid = body[i]
            chunk = body[i + 1 : i + 1 + data_len]
            servo = servos.get(sid)
            if servo and len(chunk) >= 3:
                pos = P._from_lo_hi(chunk[1], chunk[2])
                servo.position = max(0, min(4095, P.decode_sign_magnitude(pos)))

    def _reply(self, bus: Bus, sid: int, params: bytes) -> None:
        body = bytes([sid, len(params) + 2, 0x00]) + params  # ID, LEN, ERR=0, params
        status = b"\xff\xff" + body + bytes([P.checksum(body)])
        cksum = (int(bus) + len(status) + sum(status)) & 0xFF
        self._out.extend(bytes([0xFD, int(bus), len(status)]) + status + bytes([cksum]))
