"""D2 operator GUI: ControlLoop safety/intent logic, pose snapshot, gait edits.

Hardware-free — drives the loop's ``step_once`` directly with the in-memory
``SimTransport`` and an injected clock (no thread, deterministic)."""
import pytest

from dogv3.config.loader import default_config
from dogv3.config.schema import RobotConfig, ServoSpec
from dogv3.driver.feetech import FeetechDriver
from dogv3.driver.mux import Bus
from dogv3.runtime.control import RobotController, INTENT_STALE_DISARM_TIMEOUT, WATCHDOG_TIMEOUT
from dogv3.runtime.operator_server import GAIT_PRESETS, GAIT_SLIDERS, ControlLoop

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
    for leg in ("BL", "BR", "FL", "FR"):
        for joint in ("hip", "femur", "tibia"):
            setattr(cfg.legs[leg].ids, joint, sid)
            cfg.servos[str(sid)] = ServoSpec(
                slot=f"{leg}_{joint}", min_raw=200, max_raw=3800, home_raw=2048,
                verified={"assigned": True, "center": True, "direction": True, "range": True},
            )
            sid += 1
    return cfg


def make_loop(with_driver=True):
    cfg = commissioned_config()
    clock = Clock()
    transport = driver = None
    if with_driver:
        transport = SimTransport({Bus.A: [1, 2, 3, 4, 5, 6], Bus.B: [7, 8, 9, 10, 11, 12]})
        driver = FeetechDriver(transport, reply_timeout=0.1)
    loop = ControlLoop(RobotController(cfg, driver, now=clock), hz=60.0)
    return loop, clock, transport


def test_boots_disarmed_torque_off():
    loop, clock, transport = make_loop()
    loop.step_once(0.02)
    assert not loop.controller.state.armed
    assert all(not s.torque for s in transport.servos[Bus.A].values())


def test_press_arm_arms_and_enables_torque():
    loop, clock, transport = make_loop()
    loop.press("arm")
    loop.step_once(0.02)
    assert loop.controller.state.armed
    # Torque was edge-written to every assigned servo on the loop thread.
    torque_a = [s.torque for s in transport.servos[Bus.A].values()]
    assert all(torque_a) and len(torque_a) == 6


def test_disarm_drops_torque():
    loop, clock, transport = make_loop()
    loop.press("arm"); loop.step_once(0.02)
    loop.press("disarm"); loop.step_once(0.02)
    assert not loop.controller.state.armed
    assert all(not s.torque for s in transport.servos[Bus.B].values())


def test_intent_reaches_controller_command():
    loop, clock, _ = make_loop(with_driver=False)
    loop.submit_intent(vx=0.2, vy=0.9, wz=-0.3, body_z=15.0,
                       sway=0.7, sway_max_mm=35.0,
                       pitch=-0.5, pitch_max_deg=8.0)
    loop.step_once(0.02)
    # Raw intent lands in the slew target immediately; the shaped command
    # glides toward it at the seed's accel_limit. Height trim is immediate.
    t = loop.controller.cmd_target
    assert (t.vx, t.vy, t.wz) == (0.2, 0.9, -0.3)
    assert t.sway == 0.7
    assert t.sway_max_mm == 35.0
    assert t.pitch == -0.5 and t.pitch_max_deg == 8.0
    assert loop.controller.cmd.body_z == 15.0
    assert loop.controller.cmd.sway == 0.0  # stand sway is ignored while driving
    assert 0.0 < loop.controller.cmd.vy < 0.9
    for _ in range(60):
        loop.step_once(0.02)
    assert loop.controller.cmd.vy == pytest.approx(0.9, abs=1e-6)


def test_builtin_crawl_period_is_not_silently_clamped():
    loop, _clock, _ = make_loop(with_driver=False)
    crawl = GAIT_PRESETS["crawl"]
    assert crawl["cycle_period"] <= GAIT_SLIDERS["cycle_period"][1]
    out = loop.update_gait(crawl)
    assert out["cycle_period"] == crawl["cycle_period"] == 1.6


class _ImuSnapshot:
    def __init__(self, snapshot):
        self._snapshot = snapshot

    def snapshot(self):
        return dict(self._snapshot)


def test_stale_imu_attitude_is_rejected_before_stabilization():
    loop, _clock, _ = make_loop(with_driver=False)
    loop.controller.config.stabilizer.enabled = True
    loop.imu = _ImuSnapshot({
        "ok": True, "age_s": loop.controller.config.stabilizer.max_imu_age_s + 0.01,
        "roll_deg": 8.0, "pitch_deg": -4.0,
    })
    loop.controller.attitude = (1.0, 1.0)
    loop._update_attitude()
    assert loop.controller.attitude is None


def test_fresh_imu_attitude_applies_level_reference():
    loop, _clock, _ = make_loop(with_driver=False)
    st = loop.controller.config.stabilizer
    st.level_roll_deg, st.level_pitch_deg = 1.5, -2.0
    loop.imu = _ImuSnapshot({
        "ok": True, "age_s": 0.01, "roll_deg": 5.0, "pitch_deg": 1.0,
    })
    loop._update_attitude()
    assert loop.controller.attitude == pytest.approx((3.5, 3.0))


def test_missing_or_invalid_imu_age_fails_safe():
    loop, _clock, _ = make_loop(with_driver=False)
    loop.controller.attitude = (1.0, 1.0)
    loop.imu = _ImuSnapshot({
        "ok": True, "age_s": "not-a-number", "roll_deg": 5.0, "pitch_deg": 1.0,
    })
    loop._update_attitude()
    assert loop.controller.attitude is None


def test_fresh_intent_gate_keeps_watchdog_honest():
    """The loop must NOT manufacture commands on its own — a silent browser must
    let the watchdog go stale and ultimately disarm."""
    loop, clock, _ = make_loop(with_driver=False)
    loop.press("arm")
    # Drive a while with fresh intent each step → stays armed, builds soft-start.
    for _ in range(20):
        loop.submit_intent(vx=0.0, vy=1.0, wz=0.0, body_z=0.0)
        loop.step_once(0.02)
        clock.tick(0.02)
    assert loop.controller.state.armed
    assert loop.controller.state.soft_start > 0.0
    last_seq = loop.controller.state.last_seq

    # Browser goes silent: keep ticking, no new intent.
    clock.tick(WATCHDOG_TIMEOUT + 0.1)
    loop.step_once(0.02)
    assert loop.controller.state.last_seq == last_seq      # nothing new submitted
    assert loop.controller.state.watchdog_tripped          # holding stand
    assert loop.controller.state.armed

    clock.tick(INTENT_STALE_DISARM_TIMEOUT)
    loop.step_once(0.02)
    assert not loop.controller.state.armed                 # long stale -> disarm


def test_estop_latches_and_clear_requires_disarm():
    loop, clock, transport = make_loop()
    loop.press("arm"); loop.step_once(0.02)
    loop.press("estop"); loop.step_once(0.02)
    assert loop.controller.state.estop
    assert not loop.controller.state.armed
    assert all(not s.torque for s in transport.servos[Bus.A].values())

    # Re-arm is refused while E-STOP is latched.
    loop.press("arm"); loop.step_once(0.02)
    assert not loop.controller.state.armed

    # Clear works because we are disarmed.
    loop.press("clear"); loop.step_once(0.02)
    assert not loop.controller.state.estop
    loop.press("arm"); loop.step_once(0.02)
    assert loop.controller.state.armed


def test_estop_via_intent_button_message_path():
    loop, clock, _ = make_loop(with_driver=False)
    loop.press("arm"); loop.step_once(0.02)
    assert loop.controller.state.armed
    loop.press("estop"); loop.step_once(0.02)
    assert loop.controller.state.estop and not loop.controller.state.armed


def test_gait_update_valid_and_live():
    loop, clock, _ = make_loop(with_driver=False)
    out = loop.update_gait({"step_length": 25.0, "cycle_period": 0.8})
    assert out["step_length"] == 25.0
    loop.step_once(0.02)  # pending gait applied on the loop thread
    assert loop.controller.gait.seed.step_length == 25.0
    assert loop.controller.gait.seed.cycle_period == 0.8


def test_gait_update_rejects_bad_field_and_clamps_values():
    loop, clock, _ = make_loop(with_driver=False)
    with pytest.raises(ValueError):
        loop.update_gait({"not_a_field": 1.0})
    with pytest.raises(ValueError):
        loop.update_gait({"gait_type": "gallop"})  # only trot/crawl allowed
    # Out-of-range numeric levers are clamped to GAIT_SLIDERS bounds, not
    # rejected — the API cannot push out-of-range values onto hardware.
    out = loop.update_gait({"duty_factor": 5.0})
    assert out["duty_factor"] == GAIT_SLIDERS["duty_factor"][1]
    out = loop.update_gait({"step_length": -100.0})
    assert out["step_length"] == GAIT_SLIDERS["step_length"][0]


def test_intent_shaping_slews_and_estop_is_instant():
    loop, clock, _ = make_loop(with_driver=False)
    loop.press("arm"); loop.step_once(0.02)
    loop.controller.gait.seed.accel_limit = 2.0  # 0.04 full-scale per 20 ms tick
    loop.submit_intent(0.0, 1.0, 0.0, 0.0)
    loop.step_once(0.02)
    assert loop.controller.cmd.vy == pytest.approx(0.04, abs=1e-6)  # gliding, not stepping
    _tick(loop, clock, 14, vy=1.0)
    assert loop.controller.cmd.vy == pytest.approx(0.60, abs=0.03)  # still ramping
    _tick(loop, clock, 30, vy=1.0)
    assert loop.controller.cmd.vy == pytest.approx(1.0, abs=1e-6)   # reached full stick
    loop.press("estop"); loop.step_once(0.02)
    assert loop.controller.cmd.vy == 0.0  # E-STOP does not glide


def test_stance_dip_shortens_leg_mid_stance():
    from dogv3.config.schema import GaitSeed
    from dogv3.gait.trot import GaitCommand, TrotGait
    seed = GaitSeed(step_length=40, body_height=160, stance_dip=8.0, max_fwd_speed=400)
    gait = TrotGait(seed, None)
    gait.clock.reset(0.3)  # FL mid-stance (duty 0.6)
    z_mid = gait.targets(GaitCommand(vy=1.0))["FL"].z
    seed0 = GaitSeed(step_length=40, body_height=160, stance_dip=0.0, max_fwd_speed=400)
    gait0 = TrotGait(seed0, None)
    gait0.clock.reset(0.3)
    z_rigid = gait0.targets(GaitCommand(vy=1.0))["FL"].z
    assert z_mid == pytest.approx(z_rigid + 8.0, abs=0.5)  # leg 8 mm shorter mid-stance
    # Stance endpoints keep the exact commanded height (no touchdown step).
    gait.clock.reset(0.0)
    assert gait.targets(GaitCommand(vy=1.0))["FL"].z == pytest.approx(-160.0, abs=0.1)


def test_profile_smooth_load_re_dips_soft_start():
    loop, clock, _ = make_loop(with_driver=False)
    loop.press("arm")
    _tick(loop, clock, 80, vy=1.0)  # ramp soft_start to 1.0 while trotting
    assert loop.controller.state.soft_start == 1.0
    loop.update_gait({"step_length": 60.0}, smooth=True)
    loop.step_once(0.02)
    assert loop.controller.state.soft_start <= 0.4  # blend the new gait in
    # Plain slider nudges stay immediate (no dip).
    _tick(loop, clock, 80, vy=1.0)
    loop.update_gait({"step_length": 61.0})
    loop.step_once(0.02)
    assert loop.controller.state.soft_start > 0.9


def test_auto_mode_swaps_profiles_by_stick_magnitude():
    from dogv3.config.schema import GaitSeed
    loop, clock, _ = make_loop(with_driver=False)
    cfg = loop.controller.config
    cfg.gait_profiles["mode-fluid"] = GaitSeed(step_length=30)
    cfg.gait_profiles["mode-fast"] = GaitSeed(step_length=60)
    loop.auto_mode = True
    loop.press("arm")
    loop.submit_intent(0.0, 1.0, 0.0, 0.0)
    loop.step_once(0.02)   # engages mode-fast, stages the seed
    loop.step_once(0.02)   # staged seed applied
    assert loop._mode_current == "mode-fast"
    assert loop.controller.gait.seed.step_length == 60.0
    # Below the low threshold: back to fluid (dwell bypassed for the test).
    loop._mode_last_switch = 0.0
    loop.submit_intent(0.0, 0.3, 0.0, 0.0)
    loop.step_once(0.02); loop.step_once(0.02)
    assert loop._mode_current == "mode-fluid"
    assert loop.controller.gait.seed.step_length == 30.0
    # Mid-band (hysteresis): no switch even with dwell elapsed.
    loop._mode_last_switch = 0.0
    loop.submit_intent(0.0, 0.65, 0.0, 0.0)
    loop.step_once(0.02); loop.step_once(0.02)
    assert loop._mode_current == "mode-fluid"


def test_gait_update_switches_to_crawl_live():
    loop, clock, _ = make_loop(with_driver=False)
    out = loop.update_gait({"gait_type": "crawl", "body_sway_amp": 25.0})
    assert out["gait_type"] == "crawl"
    loop.step_once(0.02)  # pending gait applied on the loop thread
    assert loop.controller.gait.seed.gait_type == "crawl"
    assert loop.controller.gait.seed.body_sway_amp == 25.0


CROUCH = {"FL": [-10.0, 0.0, -120.0], "FR": [10.0, 0.0, -120.0],
          "BL": [-10.0, 0.0, -120.0], "BR": [10.0, 0.0, -120.0]}


def _tick(loop, clock, n, vy=0.0):
    for _ in range(n):
        loop.submit_intent(0.0, vy, 0.0, 0.0)  # keep the watchdog fresh
        loop.step_once(0.02)
        clock.tick(0.02)


def test_trick_runs_stand_hub_then_returns_to_stand():
    loop, clock, _ = make_loop(with_driver=False)
    loop.press("arm"); loop.step_once(0.02)
    loop.stage_trick("run", "dip", [(CROUCH, 0.3, 0.1)])
    loop.step_once(0.02)
    assert loop.controller.trick is not None
    # Timeline: stand-in (0.8+0.2) + step (0.3+0.1) + stand-out (0.8) = 2.2 s.
    _tick(loop, clock, 60)  # 1.2 s in: mid-sequence, somewhere below stand
    assert loop.controller.trick is not None
    _tick(loop, clock, 80)  # well past the end
    assert loop.controller.trick is None
    assert loop.controller.last_targets["FL"].z == pytest.approx(-160.0, abs=1.0)


def test_trick_cancelled_by_drive_intent_and_estop():
    loop, clock, _ = make_loop(with_driver=False)
    loop.press("arm"); loop.step_once(0.02)
    loop.stage_trick("run", "dip", [(CROUCH, 0.5, 0.5)])
    _tick(loop, clock, 10)
    assert loop.controller.trick is not None
    _tick(loop, clock, 3, vy=1.0)   # drive intent cancels
    assert loop.controller.trick is None

    loop.stage_trick("run", "dip", [(CROUCH, 0.5, 0.5)])
    _tick(loop, clock, 10)
    assert loop.controller.trick is not None
    loop.press("estop"); loop.step_once(0.02)
    assert loop.controller.trick is None


def test_snapshot_pose_shape():
    loop, clock, _ = make_loop()
    loop.step_once(0.02)
    snap = loop.snapshot()
    assert set(snap) >= {"telemetry", "pose", "gait_seed", "hz"}
    legs = snap["pose"]["legs"]
    assert set(legs) == {"FL", "FR", "BL", "BR"}
    for leg in legs.values():
        assert len(leg["points"]) == 4            # hip, femur, knee, foot
        assert all(len(p) == 3 for p in leg["points"])
        assert set(leg["counts"]) == {"hip", "femur", "tibia"}
    assert snap["pose"]["body"]["shoulder_lateral"] > 0


def test_dry_loop_runs_without_driver():
    loop, clock, _ = make_loop(with_driver=False)
    loop.press("arm")
    for _ in range(5):
        loop.submit_intent(0.0, 1.0, 0.0, 0.0)
        loop.step_once(0.02)
        clock.tick(0.02)
    # No driver => no crash; arm state still tracked for the UI.
    assert loop.controller.state.armed
    assert loop.snapshot()["pose"]["legs"]["FL"]["points"]


def test_long_stale_disarms_and_drops_torque_on_bus():
    loop, clock, transport = make_loop(with_driver=True)
    loop.press("arm")
    for _ in range(5):
        loop.submit_intent(0.0, 1.0, 0.0, 0.0)
        loop.step_once(0.02)
        clock.tick(0.02)
    assert any(s.torque for s in transport.servos[Bus.A].values())  # armed -> torque on
    # Browser silent past the 10 s disarm boundary.
    clock.tick(INTENT_STALE_DISARM_TIMEOUT + 0.2)
    loop.step_once(0.02)
    assert not loop.controller.state.armed
    assert all(not s.torque for s in transport.servos[Bus.A].values())  # torque dropped


def test_pose_foot_matches_target():
    """The skeleton's foot point must equal the commanded foot target placed at
    the leg's body-frame hip origin (pose math == kinematics)."""
    loop, clock, _ = make_loop(with_driver=False)
    loop.step_once(0.02)
    snap = loop.snapshot()
    ctrl = loop.controller
    hip_sign = {"FL": (-1, 1), "FR": (1, 1), "BL": (-1, -1), "BR": (1, -1)}
    for leg, (sx, sy) in hip_sign.items():
        ft = ctrl.last_targets[leg]
        ox = sx * ctrl.config.body_mm.shoulder_lateral / 2.0
        oy = sy * ctrl.config.body_mm.fore_aft / 2.0
        foot = snap["pose"]["legs"][leg]["points"][3]
        assert abs(foot[0] - (ox + ft.x)) < 0.05
        assert abs(foot[1] - (oy + ft.y)) < 0.05
        assert abs(foot[2] - ft.z) < 0.05


def test_gait_edits_accumulate_before_apply():
    """Two single-lever edits staged before the loop applies them must merge,
    not clobber (guards the lost-update race fix)."""
    loop, clock, _ = make_loop(with_driver=False)
    loop.update_gait({"step_length": 25.0})
    loop.update_gait({"step_height": 12.0})
    loop.step_once(0.02)
    assert loop.controller.gait.seed.step_length == 25.0
    assert loop.controller.gait.seed.step_height == 12.0


def test_post_bodies_are_bound_not_query():
    """Guard the future-annotations + nested-model regression that silently turns
    JSON request bodies into query params (FastAPI 422). Covers D1 and D2."""
    from pathlib import Path
    from fastapi.routing import APIRoute
    from dogv3.setup_program import server as d1
    from dogv3.setup_program.state_machine import Commissioner
    from dogv3.runtime import operator_server as d2

    def bound(app):
        return {
            r.path: [f.name for f in r.dependant.body_params]
            for r in app.routes
            if isinstance(r, APIRoute) and "POST" in r.methods
        }

    d2app = bound(d2.build_app(
        d2.ControlLoop(RobotController(default_config(), None)),
        link=None, dry=True, config_path=Path("x.json"),
    ))
    assert d2app["/api/gait"] and d2app["/api/intent"]

    d1app = bound(d1.build_app(Commissioner(default_config(), None), None, None, Path("c.json")))
    for path in (
        "/api/assign",
        "/api/direction/set",
        "/api/range",
        "/api/gait",
        "/api/set_id",
        "/api/follow/start",
        "/api/follow/set_limit",
    ):
        assert d1app[path], f"{path} body not bound — FastAPI query-param regression"


def test_thread_start_stop_leaves_safe():
    loop, clock, transport = make_loop()
    # Real-time thread; just confirm clean start/stop and safe end state.
    loop.controller._now = __import__("time").monotonic  # thread uses real time
    loop.start()
    loop.press("arm")
    import time as _t; _t.sleep(0.1)
    loop.stop()
    assert not loop.controller.state.armed
    assert all(not s.torque for s in transport.servos[Bus.A].values())


def test_gui_typing_cannot_trigger_drive_or_estop_shortcuts():
    """Typing text/select values must not become WASD motion or Space E-STOP."""
    from dogv3.runtime.operator_server import INDEX_HTML

    assert "function typingTarget(el)" in INDEX_HTML
    assert "addEventListener('focusin',e=>{if(typingTarget(e.target))clearKeyboardDrive();});" in INDEX_HTML
    assert "addEventListener('keydown',e=>{if(typingTarget(e.target))return;" in INDEX_HTML
    assert "addEventListener('keyup',e=>{if(typingTarget(e.target))return;" in INDEX_HTML


def test_gui_offers_audio_voice_selector_and_forwards_selection():
    from dogv3.runtime.operator_server import INDEX_HTML

    assert "id=sayvoice" in INDEX_HTML
    assert "id=sayvolume" in INDEX_HTML and "value=80" in INDEX_HTML
    assert "id=sayrate" in INDEX_HTML and "value=90" in INDEX_HTML
    assert "J('/api/audio/voices')" in INDEX_HTML
    assert "J('/api/audio/say','POST',{text:t,voice,volume,rate_wpm})" in INDEX_HTML


def test_gui_maps_triggers_and_bumpers_to_two_axis_lean_and_neutralizes_on_blur():
    from dogv3.runtime.operator_server import INDEX_HTML

    assert "drive.sway=round(trigger(7)-trigger(6))" in INDEX_HTML
    assert "drive.pitch=(b(5)?1:0)-(b(4)?1:0)" in INDEX_HTML
    assert "id=swayamount" in INDEX_HTML and "max=70" in INDEX_HTML
    assert "id=pitchamount" in INDEX_HTML and "max=12" in INDEX_HTML
    assert "drive.sway_max_mm=n" in INDEX_HTML
    assert "drive.pitch_max_deg=n" in INDEX_HTML
    assert "dogv3-sway-max-mm" in INDEX_HTML
    assert "dogv3-pitch-max-deg" in INDEX_HTML
    assert "drive.vx=drive.vy=drive.wz=drive.body_z=drive.sway=drive.pitch=0" in INDEX_HTML
    assert "recenter before walking" in INDEX_HTML
