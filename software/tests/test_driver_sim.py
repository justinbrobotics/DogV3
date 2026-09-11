"""End-to-end driver tests against the in-memory ESP32+servo simulator.

Exercises the full stack: protocol packet -> mux frame -> sim -> reply -> parse.
"""
import math

import pytest

from dogv3.config.loader import default_config
from dogv3.config.schema import ServoSpec
from dogv3.driver.feetech import FeetechDriver
from dogv3.driver.mux import START_ESP_TO_HOST, Bus, encode
from dogv3.kinematics.leg import NEUTRAL_COUNT, angle_to_count
from dogv3.setup_program.state_machine import Commissioner

from sim import SimTransport


def make_driver():
    # Rear bus A: ids 1-6, Front bus B: ids 31-36.
    t = SimTransport({Bus.A: [1, 2, 3, 4, 5, 6], Bus.B: [31, 32, 33, 34, 35, 36]})
    return FeetechDriver(t, reply_timeout=0.2), t


def make_follow_commissioner(monkeypatch):
    """Commissioned mapping with the same non-uniform invert pattern as the
    real robot. Keeps follow tests hardware-free and fast."""
    monkeypatch.setattr("dogv3.setup_program.state_machine.time.sleep", lambda _seconds: None)
    drv, transport = make_driver()
    cfg = default_config()
    layout = {
        "BL": {"hip": 1, "femur": 2, "tibia": 3},
        "BR": {"hip": 4, "femur": 5, "tibia": 6},
        "FL": {"hip": 31, "femur": 32, "tibia": 33},
        "FR": {"hip": 34, "femur": 35, "tibia": 36},
    }
    inverts = {
        ("BL", "hip"): False,
        ("BR", "hip"): True,
        ("FL", "hip"): True,
        ("FR", "hip"): False,
        ("BL", "femur"): True,
        ("BR", "femur"): False,
        ("FL", "femur"): True,
        ("FR", "femur"): False,
        ("BL", "tibia"): True,
        ("BR", "tibia"): False,
        ("FL", "tibia"): True,
        ("FR", "tibia"): False,
    }
    for leg, joints in layout.items():
        for joint, sid in joints.items():
            setattr(cfg.legs[leg].ids, joint, sid)
            cfg.servos[str(sid)] = ServoSpec(
                slot=f"{leg}_{joint}",
                invert=inverts[(leg, joint)],
                ofs_calibrated=True,
                verified={"assigned": True, "center": True, "direction": True, "range": False},
            )
    return Commissioner(cfg, drv), transport


def test_ping_and_scan():
    drv, _ = make_driver()
    assert drv.ping(Bus.A, 1)
    assert not drv.ping(Bus.A, 99)
    found = drv.scan()
    assert found[Bus.A] == [1, 2, 3, 4, 5, 6]
    assert found[Bus.B] == [31, 32, 33, 34, 35, 36]


def test_transaction_skips_malformed_frame_without_dropping_valid_reply():
    drv, transport = make_driver()
    # Simulate stale/corrupt mux traffic already queued ahead of the valid PING
    # status.  Both frames arrive in one read; the valid second frame must not
    # be discarded when the first payload fails Feetech parsing.
    junk = encode(Bus.A, b"not-a-status", start=START_ESP_TO_HOST)
    transport._out.extend(junk)
    assert drv.ping(Bus.A, 1)


def test_write_and_readback_position():
    drv, _ = make_driver()
    assert drv.write_pos_ex(Bus.A, 1, 1500, 800, 0)
    assert drv.read_present_position_raw(Bus.A, 1) == 1500


def test_feedback_fields():
    drv, _ = make_driver()
    fb = drv.feedback(Bus.B, 31)
    assert fb is not None
    assert fb.voltage == 120
    assert fb.temperature == 30


def test_calibration_ofs_centers():
    drv, t = make_driver()
    t.servos[Bus.A][2].position = 1234
    assert drv.calibration_ofs(Bus.A, 2)
    assert drv.read_present_position_raw(Bus.A, 2) == 2048


def test_sync_write_hits_both_buses():
    drv, t = make_driver()
    plan = {
        Bus.A: [(1, 1000, 500, 0), (2, 3000, 500, 0)],
        Bus.B: [(31, 1500, 500, 0)],
    }
    drv.sync_write_positions(plan)
    assert t.servos[Bus.A][1].position == 1000
    assert t.servos[Bus.A][2].position == 3000
    assert t.servos[Bus.B][31].position == 1500


def test_write_angle_limits_orders_inverted_window():
    drv, t = make_driver()

    assert drv.write_angle_limits(Bus.A, 1, 3200, 900)

    servo = t.servos[Bus.A][1]
    assert servo.min_angle == 900
    assert servo.max_angle == 3200


def test_commissioner_discover_and_assign():
    from dogv3.config.loader import default_config

    drv, _ = make_driver()
    cfg = default_config()
    com = Commissioner(cfg, drv)
    result = com.discover()
    assert result.total == 12
    assert not result.duplicates
    # ID 1 is on bus A (rear) -> candidate slots are only BL_/BR_ joints.
    cands = com.candidate_slots(1)
    assert all(s.startswith(("BL_", "BR_")) for s in cands)
    spec = com.assign(1, "BL_hip")
    assert spec.verified.assigned
    assert cfg.legs["BL"].ids.hip == 1


@pytest.mark.parametrize("initial_invert", [False, True])
def test_set_direction_preserves_invert_when_probe_moved_correctly(initial_invert):
    cfg = default_config()
    com = Commissioner(cfg, driver=None)
    spec = com.assign(1, "BL_femur")
    spec.invert = initial_invert

    result = com.set_direction(1, moved_correctly=True)

    assert result.invert is initial_invert
    assert result.verified.direction


@pytest.mark.parametrize("initial_invert", [False, True])
def test_set_direction_toggles_invert_when_probe_moved_wrongly(initial_invert):
    cfg = default_config()
    com = Commissioner(cfg, driver=None)
    spec = com.assign(1, "BL_femur")
    spec.invert = initial_invert

    result = com.set_direction(1, moved_correctly=False)

    assert result.invert is (not initial_invert)
    assert result.verified.direction


def test_commissioner_full_flow_emits():
    from dogv3.config.loader import default_config

    drv, _ = make_driver()
    cfg = default_config()
    com = Commissioner(cfg, drv)
    com.discover()
    # Assign all 12: bus A ids 1-6 -> rear legs, bus B ids 31-36 -> front legs.
    rear_ids = [1, 2, 3, 4, 5, 6]
    front_ids = [31, 32, 33, 34, 35, 36]
    rear_slots = [f"{leg}_{j}" for leg in ("BL", "BR") for j in ("hip", "femur", "tibia")]
    front_slots = [f"{leg}_{j}" for leg in ("FL", "FR") for j in ("hip", "femur", "tibia")]
    for sid, slot in zip(rear_ids, rear_slots):
        com.assign(sid, slot)
    for sid, slot in zip(front_ids, front_slots):
        com.assign(sid, slot)
    # Center, direction, range for each.
    for sid in rear_ids + front_ids:
        assert com.center(sid)
        com.set_direction(sid, moved_correctly=True)
        com.set_range(sid, 200, 2048, 3800)
    ok, warnings = com.ready_to_emit()
    assert ok, warnings
    assert cfg.fully_commissioned()


def test_set_range_insets_canonical_limits_for_inverted_servo():
    from dogv3.config.loader import default_config

    drv, _ = make_driver()
    cfg = default_config()
    com = Commissioner(cfg, drv)
    com.assign(1, "BL_femur")
    cfg.servos["1"].invert = True

    spec = com.set_range(1, 994, NEUTRAL_COUNT, 3219, margin_deg=5.0)
    hard = sorted((com._count_to_deg(spec.min_raw, spec), com._count_to_deg(spec.max_raw, spec)))

    assert spec.soft_min_deg == pytest.approx(hard[0] + 5.0)
    assert spec.soft_max_deg == pytest.approx(hard[1] - 5.0)
    assert hard[0] <= spec.soft_min_deg <= spec.soft_max_deg <= hard[1]


def test_follow_session_ok_reflects_current_capture_not_old_config(monkeypatch):
    com, _ = make_follow_commissioner(monkeypatch)
    for servo in com.config.servos.values():
        servo.verified.range = True

    com.follow_start("BL", ["hip"])
    tick = com.follow_tick()
    stopped = com.follow_stop()

    assert tick["joints"]["hip"]["verified"] is False
    assert stopped["verified"] == []
    assert stopped["pending"] == ["hip"]


def test_follow_set_limit_applies_per_joint_without_stopping_or_eeprom(monkeypatch):
    com, transport = make_follow_commissioner(monkeypatch)
    master_id = com.config.legs["BL"].ids.hip
    assert master_id == 1

    com.follow_start("BL", ["hip"])
    assert com._follow is not None
    assert not transport.servos[Bus.A][master_id].torque
    assert transport.servos[Bus.A][4].torque
    assert transport.servos[Bus.B][31].torque
    assert transport.servos[Bus.B][34].torque

    transport.servos[Bus.A][master_id].position = angle_to_count(math.radians(-25.0), invert=False)
    first = com.follow_set_limit("hip", "min")
    assert first["set"]["hip"]["applied"] is False
    assert com._follow is not None
    assert not transport.servos[Bus.A][master_id].torque

    transport.servos[Bus.A][master_id].position = angle_to_count(math.radians(35.0), invert=False)
    second = com.follow_set_limit("hip", "max")
    assert second["set"]["hip"]["applied"] is True
    assert com._follow is not None

    # Physical-direction mirror: every leg reaches the SAME physical travel as the
    # master (-25 deg .. +35 deg here), each re-signed through its own servo axis,
    # so all legs move outward/inward together instead of mirroring.
    for leg in ("BL", "BR", "FL", "FR"):
        sid = com.config.legs[leg].ids.hip
        spec = com.config.servos[str(sid)]
        phys = sorted((
            com._phys_deg(leg, "hip", spec.min_raw),
            com._phys_deg(leg, "hip", spec.max_raw),
        ))
        assert [round(phys[0]), round(phys[1])] == [-25, 35]
        assert spec.min_raw <= NEUTRAL_COUNT <= spec.max_raw
        assert spec.home_raw == NEUTRAL_COUNT
        assert spec.eeprom_min == 0
        assert spec.eeprom_max == 4095
        assert spec.verified.range

    tick = com.follow_tick()
    assert tick["joints"]["hip"]["verified"] is True

    stopped = com.follow_stop()
    assert stopped == {"verified": ["hip"], "pending": []}
    assert com._follow is None
    assert all(not servo.torque for bus in transport.servos.values() for servo in bus.values())


def test_torque_all_off_stops_active_follow_session(monkeypatch):
    com, transport = make_follow_commissioner(monkeypatch)
    com.follow_start("BL", ["hip"])
    assert com._follow is not None

    results = com.set_all_torque(False)

    assert len(results) == 12
    assert com._follow is None
    assert all(not servo.torque for bus in transport.servos.values() for servo in bus.values())


def test_commissioner_manual_torque_and_read():
    from dogv3.config.loader import default_config

    drv, t = make_driver()
    cfg = default_config()
    com = Commissioner(cfg, drv)
    com.discover()
    # Before assignment, helpers resolve the bus from discovery.
    assert com.set_torque(1, False) is True
    t.servos[Bus.A][1].position = 1777
    assert com.read_raw(1) == 1777
    # After assignment, they resolve the bus from the slot.
    com.assign(31, "FL_hip")
    t.servos[Bus.B][31].position = 2500
    assert com.read_raw(31) == 2500
    assert com.set_torque(31, True) is True


def test_setup_server_exposes_visual_mapping_metadata():
    from dogv3.config.loader import default_config
    from dogv3.setup_program.server import INDEX_HTML, _bus_meta, _slot_meta

    drv, _ = make_driver()
    com = Commissioner(default_config(), drv)

    discovered = com.discover()
    assert discovered.per_bus == {"A": [1, 2, 3, 4, 5, 6], "B": [31, 32, 33, 34, 35, 36]}

    buses = _bus_meta(com)
    slots = _slot_meta(com)
    assert buses["A"]["role"] == "rear"
    assert buses["B"]["role"] == "front"
    assert slots["BL_hip"]["bus"] == "A"
    assert slots["FL_tibia"]["bus"] == "B"
    assert com.config.frame.forward_sign == "+Y"

    assert "DogV3 D1" in INDEX_HTML
    assert "Detected IDs" in INDEX_HTML
    assert "Assign servo IDs" not in INDEX_HTML
    assert "Tune trot" not in INDEX_HTML


def test_resync_recovers_a_firmware_parser_stuck_in_ascii_mode():
    """The ESP32's rxState is a static global. Stranded in ASCII_LINE it answers
    every mux frame with 'ERR unknown cmd' and never recovers on its own, so the
    driver must be able to walk it back to IDLE and confirm."""
    from dogv3.driver.feetech import FeetechDriver

    class StuckEsp:
        """Swallows bytes as text until a newline; only then honours VERSION."""

        def __init__(self):
            self.out = b""
            self.ascii_mode = True     # stranded mid-line, like the real fault

        def write(self, data: bytes) -> None:
            for b in data:
                if self.ascii_mode:
                    if b in (0x0A, 0x0D):
                        self.ascii_mode = False
                        self.out += b"ERR unknown cmd: junk\r\n"
                elif bytes([b]) == b"V":
                    self.pending = True
            if not self.ascii_mode and b"VERSION" in data:
                self.out += b"DOGV3-MUX v1.0\r\n"

        def read(self, n: int = 4096) -> bytes:
            out, self.out = self.out[:n], self.out[n:]
            return out

        def reset_input(self) -> None:
            self.out = b""

    esp = StuckEsp()
    d = FeetechDriver(esp)
    assert d.resync() is True
    assert esp.ascii_mode is False, "parser should be back in IDLE"


def test_resync_reports_failure_when_the_link_never_answers():
    from dogv3.driver.feetech import FeetechDriver

    class Dead:
        def write(self, data: bytes) -> None: ...
        def read(self, n: int = 4096) -> bytes: return b""
        def reset_input(self) -> None: ...

    assert FeetechDriver(Dead()).resync() is False
