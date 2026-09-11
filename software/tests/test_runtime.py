"""Runtime controller: ARM gate, soft-start, watchdog, IK->counts plan."""
import math
from dogv3.config.schema import RobotConfig, ServoSpec
from dogv3.config.loader import default_config
from dogv3.driver.feetech import FeetechDriver
from dogv3.driver.mux import Bus
from dogv3.runtime.control import (
    INTENT_STALE_DISARM_TIMEOUT,
    MANUAL_PITCH_SLEW_DEG_S,
    MANUAL_SWAY_SLEW_MM_S,
    RobotController,
    WATCHDOG_TIMEOUT,
)

from sim import SimTransport


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t

    def tick(self, dt):
        self.t += dt


def commissioned_config() -> RobotConfig:
    cfg = default_config()
    sid = 1
    layout = {
        "BL": "A", "BR": "A", "FL": "B", "FR": "B",
    }
    for leg in ("BL", "BR", "FL", "FR"):
        for joint in ("hip", "femur", "tibia"):
            setattr(cfg.legs[leg].ids, joint, sid)
            cfg.servos[str(sid)] = ServoSpec(
                slot=f"{leg}_{joint}", min_raw=200, max_raw=3800, home_raw=2048,
                verified={"assigned": True, "center": True, "direction": True, "range": True},
            )
            sid += 1
    return cfg


def make_controller(with_driver=True):
    cfg = commissioned_config()
    clock = Clock()
    driver = None
    transport = None
    if with_driver:
        # ids 1-6 on bus A (rear), 7-12 on bus B (front).
        transport = SimTransport({Bus.A: [1, 2, 3, 4, 5, 6], Bus.B: [7, 8, 9, 10, 11, 12]})
        driver = FeetechDriver(transport, reply_timeout=0.1)
    return RobotController(cfg, driver, now=clock), clock, transport


def test_disarmed_holds_stand_pose():
    ctrl, clock, _ = make_controller(with_driver=False)
    plan = ctrl.tick(0.02)
    # All 12 joints present, counts within configured range.
    counts = [c for entries in plan.values() for (_id, c, _s, _a) in entries]
    assert len(counts) == 12
    assert all(200 <= c <= 3800 for c in counts)


def test_plan_groups_by_bus():
    ctrl, clock, _ = make_controller(with_driver=False)
    plan = ctrl.compute_plan(ctrl.gait.stand_targets(ctrl.cmd))
    assert len(plan[Bus.A]) == 6  # rear legs
    assert len(plan[Bus.B]) == 6  # front legs


def test_arm_required_for_motion():
    ctrl, clock, _ = make_controller(with_driver=False)
    ctrl.submit_command({"seq": 1, "vy": 1.0, "arm": False})
    p0 = ctrl.gait.clock.phase
    ctrl.tick(0.05)
    # Not armed -> stand pose -> phase clock not advanced by gait.
    assert ctrl.gait.clock.phase == p0


def test_armed_advances_gait():
    ctrl, clock, _ = make_controller(with_driver=False)
    ctrl.submit_command({"seq": 1, "vy": 1.0, "arm": True})
    ctrl.tick(0.05)
    assert ctrl.gait.clock.phase > 0.0
    assert ctrl.state.soft_start > 0.0


def test_watchdog_holds_stand_then_long_stale_disarms():
    ctrl, clock, _ = make_controller(with_driver=False)
    ctrl.submit_command({"seq": 1, "vy": 1.0, "arm": True})
    # Build up soft-start.
    for _ in range(40):
        clock.tick(0.02)
        ctrl.submit_command({"seq": ctrl.state.last_seq + 1, "vy": 1.0, "arm": True})
        ctrl.tick(0.02)
    assert ctrl.state.armed
    # Now starve the command stream past the watchdog timeout.
    clock.tick(WATCHDOG_TIMEOUT + 0.1)
    assert ctrl.check_watchdog()
    phase = ctrl.gait.clock.phase
    plan = ctrl.tick(0.02)
    assert ctrl.state.armed
    assert ctrl.state.watchdog_tripped
    assert ctrl.state.soft_start == 0.0
    assert ctrl.gait.clock.phase == phase
    assert len([c for entries in plan.values() for c in entries]) == 12

    # Long stale intent is the disarm boundary.
    clock.tick(INTENT_STALE_DISARM_TIMEOUT)
    ctrl.tick(0.02)
    assert not ctrl.state.armed


def test_manual_sway_slews_and_drive_waits_for_recentering():
    ctrl, _, _ = make_controller(with_driver=False)
    ctrl.submit_command({"seq": 1, "arm": True, "sway": 1.0})
    ctrl.tick(0.02)
    assert ctrl.cmd.sway_mm == MANUAL_SWAY_SLEW_MM_S * 0.02
    assert ctrl.cmd.vy == 0.0
    for _ in range(39):
        ctrl.tick(0.02)
    assert ctrl.cmd.sway == 1.0
    assert ctrl.cmd.sway_mm == 20.0

    ctrl.submit_command({"seq": 2, "arm": True, "vy": 1.0, "sway": 0.0})
    ctrl.tick(0.02)
    assert ctrl.cmd.sway_mm < 20.0
    assert ctrl.cmd.vy == 0.0, "walking must wait while the body is leaned"
    for _ in range(45):
        ctrl.tick(0.02)
    assert abs(ctrl.cmd.sway_mm) <= 0.5
    assert ctrl.cmd.vy > 0.0


def test_stale_intent_recentres_manual_sway():
    ctrl, clock, _ = make_controller(with_driver=False)
    ctrl.submit_command({"seq": 1, "arm": True, "sway": 0.8, "pitch": 0.6})
    for _ in range(40):
        ctrl.tick(0.02)
    assert ctrl.cmd.sway_mm == 16.0
    assert ctrl.cmd.pitch_deg == 3.0
    clock.tick(WATCHDOG_TIMEOUT + 0.1)
    ctrl.tick(0.02)
    assert ctrl.state.watchdog_tripped
    assert ctrl.cmd_target.sway == 0.0
    assert ctrl.cmd_target.pitch == 0.0
    assert ctrl.cmd.sway_mm < 16.0
    assert ctrl.cmd.pitch_deg < 3.0


def test_manual_sway_amount_is_adjustable_but_clamped_to_70_mm():
    ctrl, _, _ = make_controller(with_driver=False)
    ctrl.submit_command({"seq": 1, "arm": True, "sway": 1.0,
                         "sway_max_mm": 100.0})
    for _ in range(90):
        ctrl.tick(0.02)
    assert ctrl.cmd.sway_max_mm == 70.0
    assert ctrl.cmd.sway_mm == 45.0  # still moving at the fixed 25 mm/s rate
    for _ in range(50):
        ctrl.tick(0.02)
    assert ctrl.cmd.sway_mm == 70.0
    assert ctrl.telemetry()["cmd"]["sway_mm"] == 70.0


def test_manual_pitch_slews_clamps_and_shares_combined_envelope():
    ctrl, _, _ = make_controller(with_driver=False)
    ctrl.submit_command({"seq": 1, "arm": True, "sway": 1.0,
                         "sway_max_mm": 70.0, "pitch": 1.0,
                         "pitch_max_deg": 100.0})
    ctrl.tick(0.02)
    assert ctrl.cmd.sway_mm == MANUAL_SWAY_SLEW_MM_S * 0.02
    assert ctrl.cmd.pitch_deg == MANUAL_PITCH_SLEW_DEG_S * 0.02
    for _ in range(200):
        ctrl.tick(0.02)
    # Full commands on both pairs project to the unit-circle diagonal.
    assert math.isclose(ctrl.cmd.sway_mm, 70.0 / math.sqrt(2.0), abs_tol=1e-6)
    assert math.isclose(ctrl.cmd.pitch_deg, 12.0 / math.sqrt(2.0), abs_tol=1e-6)
    assert ctrl.cmd.pitch_max_deg == 12.0

    ctrl.submit_command({"seq": 2, "arm": True, "vy": 1.0})
    ctrl.tick(0.02)
    assert ctrl.cmd.vy == 0.0, "walking must wait for sway and pitch to recenter"


def test_estop_always_wins():
    ctrl, clock, _ = make_controller(with_driver=False)
    ctrl.submit_command({"seq": 1, "vy": 1.0, "arm": True})
    ctrl.submit_command({"seq": 2, "estop": True})
    assert ctrl.state.estop
    assert not ctrl.state.armed
    ctrl.submit_command({"seq": 3, "arm": True})  # cannot re-arm while estopped
    assert not ctrl.state.armed


def test_tick_sends_syncwrite_to_hardware():
    ctrl, clock, transport = make_controller(with_driver=True)
    ctrl.submit_command({"seq": 1, "vy": 1.0, "arm": True})
    ctrl.tick(0.05)
    # Servo positions should have been updated by the sync write.
    positions = [s.position for s in transport.servos[Bus.A].values()]
    assert any(p != 2048 for p in positions)


# --- host-drive (Pi-less) joystick mapping -------------------------------





