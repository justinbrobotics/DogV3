"""Torque writes must report what actually landed, and retry when short.

The failure this guards: a torque write into an unpowered or sagging bus left
the robot ARMED, streaming gait, and completely limp, with nothing in telemetry
saying so.
"""
from dogv3.config.loader import default_config
from dogv3.config.schema import RobotConfig, ServoSpec
from dogv3.driver.mux import Bus
from dogv3.runtime.control import TORQUE_RETRY_S, RobotController


def commissioned_config() -> RobotConfig:
    """default_config() has null servo IDs, so apply_torque would see zero
    servos and the counts would be vacuously right."""
    cfg = default_config()
    sid = 1
    for leg in ("BL", "BR", "FL", "FR"):
        for joint in ("hip", "femur", "tibia"):
            setattr(cfg.legs[leg].ids, joint, sid)
            cfg.servos[str(sid)] = ServoSpec(
                slot=f"{leg}_{joint}", min_raw=200, max_raw=3800, home_raw=2048,
                verified={"assigned": True, "center": True, "direction": True, "range": True},
            )
            sid += 1
    return cfg


class FakeDriver:
    """Counts torque writes and can refuse them, like a dead rail."""

    def __init__(self, acks=True, positions=True):
        self.acks = acks
        self.positions = positions
        self.calls = 0
        self.plans = 0
        self.last_plan = None
        self.resets = 0
        self.resyncs = 0
        self.resync_ok = True

    def resync(self, expected: str = "DOGV3-MUX") -> bool:
        # prepare_arm() resyncs the ESP32's static rxState before reading;
        # stranded in ASCII_LINE it answers every mux frame with an error.
        self.resyncs += 1
        return self.resync_ok

    def enable_torque(self, bus: Bus, servo_id: int, enable: bool) -> bool:
        self.calls += 1
        return self.acks

    def sync_write_positions(self, plan) -> None:
        self.plans += 1
        self.last_plan = plan

    def read_position(self, bus: Bus, servo_id: int) -> int | None:
        return 1000 + servo_id if self.positions else None

    def reset_input(self) -> None:
        self.resets += 1


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def _ctrl(driver, clock=None):
    return RobotController(commissioned_config(), driver, now=clock or Clock())


def test_apply_torque_reports_every_servo_that_acknowledged():
    d = FakeDriver(acks=True)
    assert _ctrl(d).apply_torque(True) == (12, 12)


def test_a_dead_rail_reports_zero_rather_than_looking_fine():
    d = FakeDriver(acks=False)
    c = _ctrl(d)
    assert c.apply_torque(True) == (0, 12)
    t = c.telemetry()["torque"]
    assert t["acked"] == 0 and t["total"] == 12 and t["wanted"] is True


def test_telemetry_carries_the_count_so_the_gui_can_show_it():
    c = _ctrl(FakeDriver(acks=True))
    c.apply_torque(True)
    assert c.telemetry()["torque"] == {"acked": 12, "total": 12, "wanted": True}


def test_short_torque_is_retried_while_armed():
    # The rail was dead at the arm edge; torque is edge-triggered, so without a
    # retry the robot would stay armed and limp forever.
    clock = Clock()
    d = FakeDriver(acks=False)
    c = _ctrl(d, clock)
    c.arm()
    c.apply_torque(True)
    before = d.calls
    clock.t += TORQUE_RETRY_S + 0.01
    c.tick(1 / 60)
    assert d.calls > before


def test_a_late_rail_self_heals():
    clock = Clock()
    d = FakeDriver(acks=False)
    c = _ctrl(d, clock)
    c.arm()
    c.apply_torque(True)
    assert c.torque_acked == 0
    d.acks = True                       # rail comes up
    clock.t += TORQUE_RETRY_S + 0.01
    c.tick(1 / 60)
    assert c.torque_acked == 12


def test_a_healthy_rail_is_not_hammered_with_retries():
    clock = Clock()
    d = FakeDriver(acks=True)
    c = _ctrl(d, clock)
    c.arm()
    c.apply_torque(True)
    before = d.calls
    # Feed intent each pass, or the 10 s long-stale disarm fires and its own
    # torque-off would be miscounted as a retry.
    for i in range(10):
        clock.t += TORQUE_RETRY_S + 0.01
        c.submit_command({"seq": i, "arm": True})
        c.tick(1 / 60)
    assert d.calls == before


def test_no_retry_while_disarmed():
    clock = Clock()
    d = FakeDriver(acks=False)
    c = _ctrl(d, clock)
    c.apply_torque(False)               # disarm path
    before = d.calls
    clock.t += TORQUE_RETRY_S * 5
    c.tick(1 / 60)
    assert d.calls == before


def test_prepare_arm_installs_present_positions_before_torque():
    d = FakeDriver()
    c = _ctrl(d)
    assert c.prepare_arm() == (12, 12)
    assert d.plans == 1
    captured = {
        (bus, sid): count
        for bus, entries in d.last_plan.items()
        for sid, count, _speed, _acc in entries
    }
    assert len(captured) == 12
    assert all(count == 1000 + sid for (_bus, sid), count in captured.items())
    assert c.telemetry()["arm_capture"] == {"acked": 12, "total": 12}


def test_prepare_arm_refuses_partial_position_capture():
    d = FakeDriver(positions=False)
    c = _ctrl(d)
    assert c.prepare_arm() == (0, 12)
    assert d.plans == 0
    assert c.telemetry()["arm_capture"] == {"acked": 0, "total": 12}


def test_arm_start_plan_blends_from_captured_raw_counts():
    d = FakeDriver()
    c = _ctrl(d)
    assert c.prepare_arm() == (12, 12)
    starts = {
        (bus, sid): count
        for bus, entries in d.last_plan.items()
        for sid, count, _speed, _acc in entries
    }
    c.arm()
    c.submit_command({"seq": 1, "arm": True})
    plan = c.tick(1 / 60)
    # The first soft-start tick may move by a few counts, but it must remain
    # much closer to the captured pose than to a full stand target.
    for bus, entries in plan.items():
        for sid, count, _speed, _acc in entries:
            assert abs(count - starts[(bus, sid)]) < 50


def test_prepare_arm_resyncs_the_link_before_reading_positions():
    """The ESP32's parser can be stranded in ASCII_LINE, where it answers every
    mux frame with 'ERR unknown cmd'. Host-side buffer clearing cannot fix that,
    so ARM must resync the firmware parser before trusting any read."""
    d = FakeDriver()
    c = _ctrl(d)
    c.prepare_arm()
    assert d.resyncs == 1
    assert c.link_resynced is True


def test_link_resynced_is_reported_when_the_resync_fails():
    d = FakeDriver()
    d.resync_ok = False
    c = _ctrl(d)
    c.prepare_arm()
    assert c.link_resynced is False
    assert c.telemetry()["link_resynced"] is False
