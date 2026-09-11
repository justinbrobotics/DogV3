"""Packet construction vs. the official SCServo byte layout.

Expected frames are computed by hand from SCS.cpp / SMS_STS.cpp so a regression
in the Python mirror is caught immediately.
"""
import pytest

from dogv3.driver import protocol as P
from dogv3.driver.mux import Bus, encode, decode, MuxStreamParser


def test_ping_frame():
    # FF FF ID LEN INST CHK ; LEN=2 ; CHK = ~(1+2+1) = 0xFB
    assert P.ping(1) == bytes([0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB])


def test_read_feedback_frame():
    # Read(1, 56, 15): params [addr=0x38, len=0x0F], LEN=4, CHK=0xB1
    assert P.read(1, P.FEEDBACK_ADDR, P.FEEDBACK_LEN) == bytes(
        [0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x0F, 0xB1]
    )


def test_write_pos_ex_frame():
    # WritePosEx(1, 2048, 1000, 0) -> writes 7 bytes at ACC (41=0x29)
    # payload [ACC=0, posL=0x00, posH=0x08, 0,0, spdL=0xE8, spdH=0x03]
    expected = bytes(
        [0xFF, 0xFF, 0x01, 0x0A, 0x03, 0x29, 0x00, 0x00, 0x08, 0x00, 0x00, 0xE8, 0x03, 0xD5]
    )
    assert P.write_pos_ex(1, 2048, 1000, 0) == expected


def test_calibration_ofs_frame():
    # CalibrationOfs writes 128 to TORQUE_ENABLE (40=0x28); CHK=0x4F
    assert P.calibration_ofs(1) == bytes([0xFF, 0xFF, 0x01, 0x04, 0x03, 0x28, 0x80, 0x4F])


def test_enable_torque_frames():
    assert P.enable_torque(1, True)[5:7] == bytes([0x28, 0x01])
    assert P.enable_torque(1, False)[5:7] == bytes([0x28, 0x00])


def test_negative_position_sign_magnitude():
    # WritePosEx with negative position sets bit 15 on the magnitude.
    # frame: FF FF ID LEN INST ADDR ACC PosL PosH ... -> PosL=frame[7], PosH=frame[8]
    frame = P.write_pos_ex(1, -100, 0, 0)
    pos_l, pos_h = frame[7], frame[8]
    raw = pos_l | (pos_h << 8)
    assert raw & (1 << 15)
    assert (raw & ~(1 << 15)) == 100


def test_sync_write_header_and_roundtrip():
    frame = P.sync_write_pos_ex([1, 2], [2048, 1000], [1000, 500], [0, 0])
    # FF FF FE LEN 0x83 ADDR DLEN ...
    assert frame[0:3] == bytes([0xFF, 0xFF, 0xFE])
    assert frame[4] == P.INST_SYNC_WRITE
    assert frame[5] == P.SMS_STS_ACC
    assert frame[6] == 7  # data length
    # msg_len = (7+1)*2 + 4 = 20
    assert frame[3] == 20
    # checksum valid over body (everything after FF FF, minus trailing checksum)
    body = frame[2:-1]
    assert P.checksum(body) == frame[-1]


def test_checksum_is_ones_complement_sum():
    body = bytes([0x01, 0x02, 0x01])
    assert P.checksum(body) == (~sum(body)) & 0xFF


def test_parse_status_feedback_roundtrip():
    # Build a synthetic FeedBack reply: position=2048, voltage=120, temp=35.
    block = bytearray(P.FEEDBACK_LEN)
    block[P.SMS_STS_PRESENT_POSITION_L - P.FEEDBACK_ADDR] = 0x00
    block[P.SMS_STS_PRESENT_POSITION_H - P.FEEDBACK_ADDR] = 0x08
    block[P.SMS_STS_PRESENT_VOLTAGE - P.FEEDBACK_ADDR] = 120
    block[P.SMS_STS_PRESENT_TEMPERATURE - P.FEEDBACK_ADDR] = 35
    body = bytes([0x01, P.FEEDBACK_LEN + 2, 0x00]) + bytes(block)
    frame = b"\xff\xff" + body + bytes([P.checksum(body)])
    resp = P.parse_status(frame)
    assert resp.servo_id == 1
    assert resp.error == 0
    assert P.decode_position(resp.params) == 2048
    assert P.decode_voltage(resp.params) == 120
    assert P.decode_temperature(resp.params) == 35


def test_parse_status_rejects_bad_checksum():
    frame = bytearray(P.ping(1))  # not a status frame but valid structure-ish
    frame[-1] ^= 0xFF
    with pytest.raises(P.ProtocolError):
        P.parse_status(bytes(frame))


def test_mux_encode_decode_roundtrip():
    pkt = P.write_pos_ex(5, 2048, 1000, 0)
    wrapped = encode(Bus.A, pkt)
    assert wrapped[0] == 0xFE
    assert wrapped[1] == Bus.A
    # Decode from the ESP->host direction using a matching start byte.
    reply = bytes([0xFD]) + wrapped[1:]
    frame = decode(reply)
    assert frame.bus == Bus.A
    assert frame.payload == pkt


def test_mux_stream_parser_separates_text_and_frames():
    parser = MuxStreamParser()
    pkt = P.ping(7)
    reply = bytes([0xFD, Bus.B, len(pkt)]) + pkt + bytes([(Bus.B + len(pkt) + sum(pkt)) & 0xFF])
    stream = b"PONG\r\n" + reply
    frames = parser.feed(stream)
    assert len(frames) == 1
    assert frames[0].bus == Bus.B
    assert frames[0].payload == pkt
    assert b"PONG" in bytes(parser.text_bytes)
