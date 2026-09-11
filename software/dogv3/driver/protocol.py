"""Feetech STS3215 wire protocol — a faithful Python mirror of the official
``SCServo`` C++ library (`SCS.cpp` / `SMS_STS.cpp`).

Only the *packet byte layout* lives here; transport (serial / ESP32 mux) is
elsewhere. Everything below is checked against the C++ source byte-for-byte in
``tests/test_protocol.py``.

Endianness: the STS application layer constructs ``SMS_STS`` with ``End = 0``
(see `SMS_STS::SMS_STS()`), so 16-bit values are **little-endian** on the wire
(`Host2SCS` with ``End == 0`` puts the low byte first).
"""
from __future__ import annotations

from dataclasses import dataclass

# ---------------------------------------------------------------------------
# Instructions (INST.h)
# ---------------------------------------------------------------------------
INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03
INST_REG_WRITE = 0x04
INST_REG_ACTION = 0x05
INST_SYNC_READ = 0x82
INST_SYNC_WRITE = 0x83

BROADCAST_ID = 0xFE

# ---------------------------------------------------------------------------
# SMS_STS memory table (SMS_STS.h) — authoritative register addresses
# ---------------------------------------------------------------------------
# EPROM (read only)
SMS_STS_MODEL_L = 3
SMS_STS_MODEL_H = 4
# EPROM (read & write)
SMS_STS_ID = 5
SMS_STS_BAUD_RATE = 6
SMS_STS_MIN_ANGLE_LIMIT_L = 9
SMS_STS_MIN_ANGLE_LIMIT_H = 10
SMS_STS_MAX_ANGLE_LIMIT_L = 11
SMS_STS_MAX_ANGLE_LIMIT_H = 12
SMS_STS_CW_DEAD = 26
SMS_STS_CCW_DEAD = 27
SMS_STS_OFS_L = 31
SMS_STS_OFS_H = 32
SMS_STS_MODE = 33
# SRAM (read & write)
SMS_STS_TORQUE_ENABLE = 40
SMS_STS_ACC = 41
SMS_STS_GOAL_POSITION_L = 42
SMS_STS_GOAL_POSITION_H = 43
SMS_STS_GOAL_TIME_L = 44
SMS_STS_GOAL_TIME_H = 45
SMS_STS_GOAL_SPEED_L = 46
SMS_STS_GOAL_SPEED_H = 47
SMS_STS_TORQUE_LIMIT_L = 48
SMS_STS_TORQUE_LIMIT_H = 49
SMS_STS_LOCK = 55
# SRAM (read only)
SMS_STS_PRESENT_POSITION_L = 56
SMS_STS_PRESENT_POSITION_H = 57
SMS_STS_PRESENT_SPEED_L = 58
SMS_STS_PRESENT_SPEED_H = 59
SMS_STS_PRESENT_LOAD_L = 60
SMS_STS_PRESENT_LOAD_H = 61
SMS_STS_PRESENT_VOLTAGE = 62
SMS_STS_PRESENT_TEMPERATURE = 63
SMS_STS_MOVING = 66
SMS_STS_PRESENT_CURRENT_L = 69
SMS_STS_PRESENT_CURRENT_H = 70

# Feedback block read by SMS_STS::FeedBack: PRESENT_POSITION_L .. PRESENT_CURRENT_H
FEEDBACK_ADDR = SMS_STS_PRESENT_POSITION_L
FEEDBACK_LEN = SMS_STS_PRESENT_CURRENT_H - SMS_STS_PRESENT_POSITION_L + 1  # 15

# Special torque-enable values used by SMS_STS
TORQUE_DISABLE = 0
TORQUE_ENABLE = 1
TORQUE_CALIBRATE_MIDDLE = 128  # CalibrationOfs writes this to reg 40


def checksum(body: bytes) -> int:
    """Feetech checksum: ``~(sum(body)) & 0xFF`` where *body* is every byte
    after the two 0xFF header bytes, up to and including the last parameter
    (i.e. ID, Length, Instruction, params...)."""
    return (~sum(body)) & 0xFF


def _lo_hi(value: int) -> tuple[int, int]:
    """Split a 16-bit value little-endian (low byte first) — matches
    ``Host2SCS`` with ``End == 0``."""
    value &= 0xFFFF
    return value & 0xFF, (value >> 8) & 0xFF


def _from_lo_hi(lo: int, hi: int) -> int:
    """Combine two bytes little-endian — matches ``SCS2Host`` with ``End == 0``."""
    return ((hi & 0xFF) << 8) | (lo & 0xFF)


def _sign_magnitude(value: int, bit: int = 15) -> int:
    """Encode a signed int as Feetech sign-magnitude: negative -> magnitude
    with ``1<<bit`` set. Mirrors the `if(Position<0){...|= (1<<15);}` idiom."""
    if value < 0:
        return (-value) | (1 << bit)
    return value


def decode_sign_magnitude(value: int, bit: int = 15) -> int:
    """Inverse of :func:`_sign_magnitude` — used decoding present pos/speed/current."""
    if value & (1 << bit):
        return -(value & ~(1 << bit))
    return value


# ---------------------------------------------------------------------------
# Packet builders — each returns the full frame including 0xFF 0xFF header and
# trailing checksum, exactly as the C++ library would emit on the wire.
# ---------------------------------------------------------------------------

def _frame(servo_id: int, instruction: int, params: bytes = b"") -> bytes:
    """Build a standard instruction packet:
    ``FF FF ID LEN INST PARAMS... CHK`` with ``LEN = len(params) + 2``."""
    length = len(params) + 2
    body = bytes([servo_id & 0xFF, length, instruction]) + params
    return b"\xff\xff" + body + bytes([checksum(body)])


def ping(servo_id: int) -> bytes:
    """PING (INST_PING, no params)."""
    return _frame(servo_id, INST_PING)


def read(servo_id: int, mem_addr: int, length: int) -> bytes:
    """READ command — one param byte = number of bytes to read."""
    return _frame(servo_id, INST_READ, bytes([mem_addr, length]))


def write(servo_id: int, mem_addr: int, data: bytes) -> bytes:
    """Generic WRITE (INST_WRITE) — first param is the memory address."""
    return _frame(servo_id, INST_WRITE, bytes([mem_addr]) + data)


def write_byte(servo_id: int, mem_addr: int, value: int) -> bytes:
    return write(servo_id, mem_addr, bytes([value & 0xFF]))


def write_word(servo_id: int, mem_addr: int, value: int) -> bytes:
    lo, hi = _lo_hi(value)
    return write(servo_id, mem_addr, bytes([lo, hi]))


def reg_write(servo_id: int, mem_addr: int, data: bytes) -> bytes:
    """Async REG_WRITE (INST_REG_WRITE) — same layout as write, deferred until
    REG_ACTION."""
    return _frame(servo_id, INST_REG_WRITE, bytes([mem_addr]) + data)


def reg_action(servo_id: int = BROADCAST_ID) -> bytes:
    return _frame(servo_id, INST_REG_ACTION)


def _pos_payload(position: int, speed: int, acc: int) -> bytes:
    """The 7-byte ACC..GOAL_SPEED payload shared by WritePosEx / RegWritePosEx /
    SyncWritePosEx: ``[ACC, PosL, PosH, TimeL=0, TimeH=0, SpeedL, SpeedH]``."""
    pos = _sign_magnitude(position)
    pos_l, pos_h = _lo_hi(pos)
    spd_l, spd_h = _lo_hi(speed)
    return bytes([acc & 0xFF, pos_l, pos_h, 0, 0, spd_l, spd_h])


def write_pos_ex(servo_id: int, position: int, speed: int, acc: int = 0) -> bytes:
    """Mirror of ``SMS_STS::WritePosEx`` — writes 7 bytes starting at SMS_STS_ACC."""
    return write(servo_id, SMS_STS_ACC, _pos_payload(position, speed, acc))


def reg_write_pos_ex(servo_id: int, position: int, speed: int, acc: int = 0) -> bytes:
    """Mirror of ``SMS_STS::RegWritePosEx``."""
    return reg_write(servo_id, SMS_STS_ACC, _pos_payload(position, speed, acc))


def sync_write_pos_ex(
    ids: list[int],
    positions: list[int],
    speeds: list[int] | None = None,
    accs: list[int] | None = None,
) -> bytes:
    """Mirror of ``SMS_STS::SyncWritePosEx`` -> ``SCS::syncWrite``.

    Frame: ``FF FF FE LEN 0x83 ADDR DLEN [ID, data*DLEN]... CHK`` where
    ``ADDR = SMS_STS_ACC`` (41), ``DLEN = 7``, and
    ``LEN = (DLEN+1)*N + 4``."""
    n = len(ids)
    if speeds is None:
        speeds = [0] * n
    if accs is None:
        accs = [0] * n
    if not (len(positions) == len(speeds) == len(accs) == n):
        raise ValueError("ids/positions/speeds/accs length mismatch")

    mem_addr = SMS_STS_ACC
    data_len = 7
    msg_len = (data_len + 1) * n + 4
    body = bytes([BROADCAST_ID, msg_len, INST_SYNC_WRITE, mem_addr, data_len])
    for sid, pos, spd, acc in zip(ids, positions, speeds, accs):
        body += bytes([sid & 0xFF]) + _pos_payload(pos, spd, acc)
    return b"\xff\xff" + body + bytes([checksum(body)])


def sync_read(ids: list[int], mem_addr: int, length: int) -> bytes:
    """Mirror of ``SCS::syncReadPacketTx``.

    Frame: ``FF FF FE LEN 0x82 ADDR RLEN [IDs...] CHK`` with ``LEN = N + 4``."""
    msg_len = len(ids) + 4
    body = bytes([BROADCAST_ID, msg_len, INST_SYNC_READ, mem_addr, length]) + bytes(
        i & 0xFF for i in ids
    )
    return b"\xff\xff" + body + bytes([checksum(body)])


# Convenience builders for high-level driver semantics ----------------------

def enable_torque(servo_id: int, enable: bool) -> bytes:
    return write_byte(servo_id, SMS_STS_TORQUE_ENABLE, TORQUE_ENABLE if enable else TORQUE_DISABLE)


def calibration_ofs(servo_id: int) -> bytes:
    """``SMS_STS::CalibrationOfs`` — writes 128 to TORQUE_ENABLE; the servo then
    stores its current position into OFS (31/32) so present pos reads ~2048."""
    return write_byte(servo_id, SMS_STS_TORQUE_ENABLE, TORQUE_CALIBRATE_MIDDLE)


def unlock_eprom(servo_id: int) -> bytes:
    return write_byte(servo_id, SMS_STS_LOCK, 0)


def lock_eprom(servo_id: int) -> bytes:
    return write_byte(servo_id, SMS_STS_LOCK, 1)


# ---------------------------------------------------------------------------
# Response parsing
# ---------------------------------------------------------------------------

@dataclass
class StatusResponse:
    """A parsed status/return packet: ``FF FF ID LEN ERR [params...] CHK``."""

    servo_id: int
    error: int
    params: bytes

    @property
    def ok(self) -> bool:
        return True  # presence of a valid-checksum frame; error bits in .error


def parse_status(frame: bytes) -> StatusResponse:
    """Parse a single status return frame, validating header, length and
    checksum. Raises :class:`ProtocolError` on any malformation."""
    if len(frame) < 6:
        raise ProtocolError(f"frame too short: {frame.hex()}")
    if frame[0] != 0xFF or frame[1] != 0xFF:
        raise ProtocolError(f"bad header: {frame[:2].hex()}")
    servo_id = frame[2]
    length = frame[3]
    # length counts: error byte + params + checksum  => total = 4 + length
    if len(frame) != 4 + length:
        raise ProtocolError(f"length mismatch: declared {length}, frame {len(frame)} bytes")
    body = frame[2:-1]  # ID .. last param
    if checksum(body) != frame[-1]:
        raise ProtocolError("checksum mismatch")
    error = frame[4]
    params = frame[5:-1]
    return StatusResponse(servo_id=servo_id, error=error, params=params)


class ProtocolError(Exception):
    """Raised on malformed or unverifiable Feetech frames."""


# Decoders for feedback fields (operate on the 15-byte FeedBack block, indexed
# relative to PRESENT_POSITION_L). Mirror SMS_STS::ReadPos/Speed/Load/etc.

def _fb(block: bytes, addr: int) -> int:
    return block[addr - FEEDBACK_ADDR]


def decode_position(block: bytes) -> int:
    raw = _from_lo_hi(_fb(block, SMS_STS_PRESENT_POSITION_L), _fb(block, SMS_STS_PRESENT_POSITION_H))
    return decode_sign_magnitude(raw, 15)


def decode_speed(block: bytes) -> int:
    raw = _from_lo_hi(_fb(block, SMS_STS_PRESENT_SPEED_L), _fb(block, SMS_STS_PRESENT_SPEED_H))
    return decode_sign_magnitude(raw, 15)


def decode_load(block: bytes) -> int:
    # Load uses bit 10 as the sign (see SMS_STS::ReadLoad).
    raw = _from_lo_hi(_fb(block, SMS_STS_PRESENT_LOAD_L), _fb(block, SMS_STS_PRESENT_LOAD_H))
    return decode_sign_magnitude(raw, 10)


def decode_voltage(block: bytes) -> int:
    return _fb(block, SMS_STS_PRESENT_VOLTAGE)


def decode_temperature(block: bytes) -> int:
    return _fb(block, SMS_STS_PRESENT_TEMPERATURE)


def decode_moving(block: bytes) -> int:
    return _fb(block, SMS_STS_MOVING)


def decode_current(block: bytes) -> int:
    raw = _from_lo_hi(_fb(block, SMS_STS_PRESENT_CURRENT_L), _fb(block, SMS_STS_PRESENT_CURRENT_H))
    return decode_sign_magnitude(raw, 15)
