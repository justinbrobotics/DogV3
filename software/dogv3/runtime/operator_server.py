"""D2 operator GUI — the primary operator control/tuning surface.

A browser app that drives the powered onboard ESP32: live 3D pose, gait tuning
sliders, ARM / DISARM / E-STOP, telemetry, and the robot camera. It is the GUI
front end for the same reference control stack used by ``dogv3-host-drive``
(trot -> IK -> SyncWritePosEx). In the Pi-onboard deployment this server runs
ON the Pi (systemd unit, ``--host 0.0.0.0``) so the watchdog is enforced one
USB hop from the ESP32; the Home-PC browser is the operator surface.

    dogv3-operate --port-serial COM5 --config robot_config.json        # bench (PC)
    dogv3-operate --port-serial /dev/dogv3-esp32 --host 0.0.0.0 \\
                     --config robot_config.json --no-browser              # onboard Pi
    dogv3-operate --dry --config robot_config.example.json  # UI preview, no hardware

Design / safety:
  - One :class:`ControlLoop` thread owns the controller and is the ONLY thread
    that touches the serial driver (serial is not re-entrant). It ticks the
    reference :class:`RobotController` at a fixed rate.
  - The browser sends *intent* (cmd_vel + arm/estop + gait edits) over the
    WebSocket; the loop submits a fresh command to the controller only when new
    browser intent arrives. If the browser freezes or disconnects, no new intent
    is submitted, so the existing command-stream watchdog holds stand at 300 ms
    and disarms at 10 s — exactly the documented safety behavior.
  - Exactly one WebSocket client is the DRIVER at a time: only its intent and
    ARM reach the controller (E-STOP/DISARM are honored from any client). A
    forgotten viewing tab can neither stomp the driver's sticks nor keep the
    10 s disarm watchdog fed.
  - When bound to a non-loopback host a shared key (``network.token`` in the
    config, or --token, else auto-generated and printed) is required on every
    request — without it any LAN webpage could POST an ARM cross-origin.
  - Boots disarmed, torque off. Torque writes are edge-triggered on arm/disarm/
    estop, on the loop thread. On shutdown the loop disarms and drops torque.
  - Hardware links are verified before the loop starts (VERSION handshake vs
    ``firmware_expected``) so a mis-pinned udev name — e.g. the RPLIDAR's
    CP2102 answering instead of the ESP32's — fails fast instead of streaming
    servo frames at a lidar.
"""
from __future__ import annotations

import argparse
import asyncio
import collections
import hashlib
import itertools
import json
import math
import secrets
import threading
import time
import urllib.error
import urllib.request
from contextlib import contextmanager
from pathlib import Path

from ..config.loader import ConfigError, load_config, save_config
from ..config.schema import GaitSeed, TrickStep
from ..driver.feetech import FeetechDriver, make_transport
from ..gait.trot import DEFAULT_MANUAL_PITCH_DEG, DEFAULT_MANUAL_SWAY_MM, GaitCommand
from ..kinematics.leg import LegGeometry, forward_kinematics, inverse_kinematics
from ..simulation.twin import evaluate_gait, profile_summary
from .control import CONTROL_HZ_DEFAULT, DEADZONE_DEFAULT, RobotController

try:
    from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
    from fastapi.responses import FileResponse, HTMLResponse, JSONResponse
    from pydantic import BaseModel
    import uvicorn
except Exception:  # pragma: no cover - FastAPI optional at import time
    FastAPI = None  # type: ignore
    BaseModel = object  # type: ignore


# Request bodies must live at module scope: with ``from __future__ import
# annotations`` FastAPI resolves these annotations against module globals, so a
# class nested inside build_app would be mistaken for a query param.
class IntentBody(BaseModel):
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    body_z: float = 0.0
    sway: float = 0.0
    sway_max_mm: float = DEFAULT_MANUAL_SWAY_MM
    pitch: float = 0.0
    pitch_max_deg: float = DEFAULT_MANUAL_PITCH_DEG


class GaitBody(BaseModel):
    levers: dict


class PoseBody(BaseModel):
    name: str


class PresetBody(BaseModel):
    name: str


class TrickBody(BaseModel):
    name: str
    steps: list[dict] = []   # [{pose, move_s, hold_s}] — only for /api/trick/save


class AnalyzeBody(BaseModel):
    vx: float = 0.0
    vy: float = 1.0
    wz: float = 0.0
    profile: str | None = None  # analyze a saved profile instead of the live seed


class ProfileImportBody(BaseModel):
    """A full gait profile pushed from another session (the D3 sim GUI's
    "Send to robot") or pasted by hand. Save-only by design: importing NEVER
    loads the profile into the live controller, so it can never move an armed
    robot — the operator loads it deliberately from this GUI."""

    name: str
    seed: dict


class CommsBody(BaseModel):
    on: bool = False

TELEMETRY_HZ = 30.0


def _mtime(path: Path) -> float:
    try:
        return Path(path).stat().st_mtime
    except OSError:
        return 0.0


def _pctl(vals: list[float], q: float) -> float:
    if not vals:
        return 0.0
    s = sorted(vals)
    return s[min(len(s) - 1, int(q * len(s)))]


# The ESP32 keeps emitting its boot banner briefly after the version string
# appears. Wait this long before draining, or the tail lands in the buffer
# immediately after reset_input() and desyncs the decoder anyway.
SETTLE_AFTER_HANDSHAKE_S = 0.25


def verify_firmware_link(transport, expected: str, *, tries: int = 2,
                         wait_s: float = 1.2) -> tuple[bool, str]:
    """VERSION handshake before the control loop ever streams servo frames.
    The ESP32 and the RPLIDAR C1 can both enumerate as CP2102 ttys, so a
    mis-pinned udev symlink must fail fast here, not stream SyncWrites at a
    lidar. Returns (ok, seen_text).

    Leaves the line CLEAN on success. The match fires as soon as the version
    string appears, but the ESP32 is usually still emitting boot chatter after
    it. Anything left in the input buffer desyncs the mux frame decoder for
    every subsequent reply — and because SyncWrites need no reply, the symptom
    is silent and bizarre: goals and torque writes land correctly while no
    status is ever parsed, surfacing much later as a torque count stuck at
    0/12 that no retry can fix."""
    seen = b""
    for _ in range(tries):
        transport.reset_input()
        transport.write(b"VERSION\n")
        deadline = time.time() + wait_s
        while time.time() < deadline:
            chunk = transport.read(256)
            if chunk:
                seen += chunk
                if expected.encode() in seen:
                    time.sleep(SETTLE_AFTER_HANDSHAKE_S)  # let the rest arrive
                    transport.reset_input()
                    return True, seen.decode(errors="replace")
            else:
                time.sleep(0.02)
    return False, seen.decode(errors="replace")


class RobotLink:
    """D3 sim GUI -> onboard robot server client. Deliberately incapable of
    motion: it polls /api/status read-only (the always-on heartbeat while
    comms is toggled on) and pushes profiles via /api/profile/import, which is
    save-only on the robot side. No ARM, no intent — the README safety model
    grants arming authority only to the robot-served operate GUI itself."""

    POLL_S = 1.0

    def __init__(self, base_url: str, token: str = ""):
        self.base_url = base_url.rstrip("/")
        self.token = token
        self._lock = threading.Lock()
        self._state: dict = {"comms_on": False, "reachable": False, "url": self.base_url}
        self._thread: threading.Thread | None = None
        self._on = False

    def _request(self, path: str, body: dict | None = None, timeout: float = 2.0) -> dict:
        url = f"{self.base_url}{path}"
        data = json.dumps(body).encode() if body is not None else None
        req = urllib.request.Request(url, data=data, method="POST" if data else "GET")
        req.add_header("Content-Type", "application/json")
        if self.token:
            req.add_header("X-DogV3-Key", self.token)
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return json.loads(resp.read().decode())

    def _poll(self) -> None:
        while self._on:
            try:
                s = self._request("/api/status", timeout=1.5)
                t = s.get("telemetry", {})
                with self._lock:
                    self._state = {
                        "comms_on": True, "reachable": True, "url": self.base_url,
                        "armed": bool(t.get("armed")), "estop": bool(t.get("estop")),
                        "link_lost": bool(s.get("link_lost")),
                        "profiles": sorted(s.get("profiles", {})),
                        "age_s": 0.0, "error": None,
                    }
            except (urllib.error.URLError, OSError, ValueError) as e:
                with self._lock:
                    self._state = {"comms_on": True, "reachable": False,
                                   "url": self.base_url, "error": str(e)}
            time.sleep(self.POLL_S)

    def set_comms(self, on: bool) -> None:
        if on and not self._on:
            self._on = True
            self._thread = threading.Thread(target=self._poll, name="dogv3-robotlink", daemon=True)
            self._thread.start()
        elif not on and self._on:
            self._on = False
            self._thread = None
            with self._lock:
                self._state = {"comms_on": False, "reachable": False, "url": self.base_url}

    def status(self) -> dict:
        with self._lock:
            return dict(self._state)

    def push_profile(self, name: str, seed: dict) -> dict:
        return self._request("/api/profile/import", {"name": name, "seed": seed})

    def stop(self) -> None:
        self.set_comms(False)
# Numeric gait_seed knobs surfaced as live sliders, with [min, max, step].
GAIT_SLIDERS: dict[str, tuple[float, float, float]] = {
    "body_height": (80.0, 240.0, 1.0),
    "step_length": (0.0, 80.0, 1.0),
    "step_height": (0.0, 60.0, 1.0),
    # Slow crawls need longer cycles; the built-in crawl preset is 1.60 s.
    "cycle_period": (0.3, 3.0, 0.02),
    # duty < 0.5 = flight phases (running); the twin is the place to vet those.
    "duty_factor": (0.35, 0.9, 0.01),
    "swing_retract": (0.0, 1.0, 0.05),
    "stance_dip": (0.0, 15.0, 0.5),
    "accel_limit": (0.5, 12.0, 0.1),
    "stance_width_offset": (-20.0, 40.0, 1.0),
    "turn_gain": (0.0, 1.5, 0.05),
    # CoM management: static body trim (both gaits) + crawl sway amplitude.
    "body_offset_x": (-40.0, 40.0, 1.0),
    "body_offset_y": (-40.0, 40.0, 1.0),
    "body_sway_amp": (0.0, 50.0, 1.0),
    # Speed governors (now enforced in the gait): full-stick body speed and yaw.
    "max_fwd_speed": (20.0, 400.0, 5.0),
    "max_yaw": (0.1, 2.0, 0.05),
}

# Audited gait presets (numerically verified through the repo's own gait + IK
# against the STS3215 velocity budget; see the D1/D2 deliverable audit). Each is
# a full lever set the GUI can apply in one click. "aggressive" is a bench-test
# ceiling, not a default.
GAIT_PRESETS: dict[str, dict] = {
    "conservative": {"gait_type": "trot", "cycle_period": 0.80, "step_length": 40.0, "duty_factor": 0.65,
                     "step_height": 22.0, "body_height": 150.0, "swing_shape": "cycloid",
                     "stance_width_offset": 20.0, "body_sway_amp": 0.0, "max_fwd_speed": 80.0},
    "balanced": {"gait_type": "trot", "cycle_period": 0.60, "step_length": 50.0, "duty_factor": 0.60,
                 "step_height": 25.0, "body_height": 150.0, "swing_shape": "cycloid",
                 "stance_width_offset": 15.0, "body_sway_amp": 0.0, "max_fwd_speed": 150.0},
    "aggressive": {"gait_type": "trot", "cycle_period": 0.45, "step_length": 60.0, "duty_factor": 0.55,
                   "step_height": 28.0, "body_height": 150.0, "swing_shape": "cycloid",
                   "stance_width_offset": 15.0, "body_sway_amp": 0.0, "max_fwd_speed": 250.0},
    "crawl": {"gait_type": "crawl", "cycle_period": 1.60, "step_length": 55.0, "duty_factor": 0.75,
              "step_height": 30.0, "body_height": 145.0, "swing_shape": "cycloid",
              "stance_width_offset": 20.0, "body_sway_amp": 25.0, "max_fwd_speed": 50.0},
}


class ControlLoop:
    """Owns the controller + driver on a single thread. Thread-safe intake of
    intent / buttons / gait edits; thread-safe snapshot out."""

    def __init__(self, controller: RobotController, *, hz: float = CONTROL_HZ_DEFAULT,
                 twin=None, imu=None):
        self.controller = controller
        # Attitude source for the stabilizer. Kept here rather than inside the
        # controller so the hot path never imports hardware.
        self.imu = imu
        # Optional live physics twin (dogv3-operate --sim): stepped on this
        # loop thread right after the controller, from the same foot targets.
        self.twin = twin
        self._sim_reset = False
        self.hz = hz
        self.dt = 1.0 / hz
        self._lock = threading.Lock()
        self._intent = {"vx": 0.0, "vy": 0.0, "wz": 0.0, "body_z": 0.0,
                        "sway": 0.0, "sway_max_mm": DEFAULT_MANUAL_SWAY_MM,
                        "pitch": 0.0, "pitch_max_deg": DEFAULT_MANUAL_PITCH_DEG}
        self._intent_seq = 0      # bumped on every new browser intent frame
        self._consumed_seq = -1   # last intent the loop fed to the controller
        self._btn = {"arm": False, "disarm": False, "estop": False, "clear": False}
        self._pending_gait: tuple[GaitSeed, bool] | None = None  # (seed, smooth-blend)
        self._pending_guided: tuple[str, GaitSeed] | None = None
        self.guided_load: dict = {"status": "idle"}
        self.guided_monitor = None
        self.guided_check_busy = False
        self._arm_in_progress = False
        self.deployment_hold_path: Path | None = None
        # AUTO gait mode: swap between the reserved "mode-fluid"/"mode-fast"
        # profiles by forward-stick magnitude, with hysteresis + dwell.
        self.auto_mode = False
        self._mode_current: str | None = None
        self._mode_last_switch = 0.0
        self._pending_pose: tuple[str, dict | None, str | None] | None = None
        # ("run", name, resolved_steps) or ("stop", None, None)
        self._pending_trick: tuple[str, str | None, list | None] | None = None
        self._cmd_seq = 0         # monotonic seq handed to the controller
        self._snapshot: dict = {}
        self._running = False
        self._thread: threading.Thread | None = None
        self.link_lost = False    # set when the serial driver dies mid-session
        # Tick-duration history (ms, last ~10 s at 60 Hz): the Pi-deployment
        # performance gate — p99 must stay under the tick budget (dt) while
        # camera/lidar services stream, or the 60 Hz loop is oversubscribed.
        self._tick_ms: collections.deque[float] = collections.deque(maxlen=600)
        self.overruns = 0
        self._build_snapshot()

    # -- intake (any thread) --------------------------------------------
    def submit_intent(self, vx: float, vy: float, wz: float, body_z: float,
                      sway: float = 0.0,
                      sway_max_mm: float = DEFAULT_MANUAL_SWAY_MM,
                      pitch: float = 0.0,
                      pitch_max_deg: float = DEFAULT_MANUAL_PITCH_DEG) -> None:
        with self._lock:
            self._intent = {"vx": float(vx), "vy": float(vy), "wz": float(wz),
                            "body_z": float(body_z), "sway": float(sway),
                            "sway_max_mm": float(sway_max_mm),
                            "pitch": float(pitch), "pitch_max_deg": float(pitch_max_deg)}
            self._intent_seq += 1

    def press(self, action: str) -> None:
        if action not in self._btn:
            raise ValueError(f"unknown button {action!r}")
        with self._lock:
            if action == "arm" and self.deployment_held():
                raise ValueError("A Pi update is in progress. ARM is unavailable until it verifies.")
            if action == "arm" and self.guided_check_busy:
                raise ValueError("Wait for the guided trajectory check to finish before arming.")
            self._btn[action] = True
            self._intent_seq += 1  # a button is fresh intent too

    def deployment_held(self) -> bool:
        return self.deployment_hold_path is not None and self.deployment_hold_path.exists()

    def update_gait(self, levers: dict, *, smooth: bool = False) -> dict:
        """Validate + stage a live gait edit. Raises ValueError on a bad value.

        Numeric levers are clamped to the same GAIT_SLIDERS bounds the GUI shows,
        so the API cannot push out-of-range values onto hardware regardless of
        client. Atomic read-merge-store under one lock, merging onto any
        already-staged edit (not just the applied seed) so rapid single-lever
        edits accumulate instead of clobbering each other before the loop
        applies them. ``smooth`` re-dips the soft-start ramp when the edit is
        applied — used for whole-profile/mode swaps so a gait change mid-walk
        blends instead of jerking (slider nudges stay immediate)."""
        clamped = dict(levers)
        for k, v in list(clamped.items()):
            if k in GAIT_SLIDERS and isinstance(v, (int, float)):
                lo, hi, _step = GAIT_SLIDERS[k]
                clamped[k] = min(hi, max(lo, float(v)))
        with self._lock:
            base_seed = self._pending_gait[0] if self._pending_gait else self.controller.gait.seed
            base = base_seed.model_dump()
            unknown = [k for k in clamped if k not in base]
            if unknown:
                raise ValueError(f"unknown gait field(s): {', '.join(sorted(unknown))}")
            base.update(clamped)
            new_seed = GaitSeed.model_validate(base)  # raises on invalid range/type
            prev_smooth = self._pending_gait[1] if self._pending_gait else False
            self._pending_gait = (new_seed, smooth or prev_smooth)
        return new_seed.model_dump()

    @contextmanager
    def guided_check(self):
        """Reserve disarmed time for CPU-heavy screening; never queue an ARM."""
        with self._lock:
            if self.controller.state.armed or self._btn["arm"] or self._arm_in_progress:
                raise ValueError("Disarm before checking guided trajectories.")
            if self.guided_check_busy:
                raise ValueError("A guided trajectory check is already running.")
            self.guided_check_busy = True
        try:
            yield
        finally:
            with self._lock:
                self.guided_check_busy = False

    def stage_guided_gait(self, candidate_id: str, seed: GaitSeed) -> None:
        """Queue a checked profile. The loop rechecks disarmed state on apply.

        No command, ARM or torque authority is created by the guided tuner.
        Regular controls retain their existing behavior.
        """
        with self._lock:
            if self.controller.state.armed or self._btn["arm"] or self._arm_in_progress:
                raise ValueError("Disarm before loading a guided experiment.")
            if self.auto_mode or self._pending_gait or self._pending_guided:
                raise ValueError("Turn AUTO off and wait for pending gait edits before loading.")
            if any(abs(self._intent[k]) > .01 for k in ("vx", "vy", "wz", "body_z", "sway", "pitch")):
                raise ValueError("Center the sticks and reset height/lean trims before loading.")
            self._pending_guided = (candidate_id, seed.model_copy(deep=True))
            self.guided_load = {"status": "pending", "candidate_id": candidate_id}

    def stage_pose(self, action: str, feet: dict | None = None, name: str | None = None) -> None:
        """Stage a pose apply/clear; executed on the loop (driver-owning) thread."""
        with self._lock:
            self._pending_pose = (action, feet, name)
            self._intent_seq += 1  # a pose action is fresh intent too

    def stage_trick(self, action: str, name: str | None = None, steps: list | None = None) -> None:
        """Stage a trick run/stop; executed on the loop thread. ``steps`` is the
        pose-resolved [(feet, move_s, hold_s), ...] list."""
        with self._lock:
            self._pending_trick = (action, name, steps)
            self._intent_seq += 1

    def reset_sim(self) -> None:
        """Stage a physics-twin reset (executed on the loop thread)."""
        with self._lock:
            self._sim_reset = True

    def snapshot(self) -> dict:
        with self._lock:
            return dict(self._snapshot)

    def tick_stats(self) -> dict:
        """Loop-health numbers for telemetry (thread-safe copy; deque appends
        happen on the loop thread)."""
        vals = list(self._tick_ms)
        return {
            "p50_ms": round(_pctl(vals, 0.50), 2),
            "p99_ms": round(_pctl(vals, 0.99), 2),
            "budget_ms": round(self.dt * 1000.0, 2),
            "overruns": self.overruns,
        }

    # -- control step ----------------------------------------------------
    def step_once(self, dt: float) -> None:
        """One control step. Public so tests can drive it without the thread."""
        try:
            self._step_once(dt)
        finally:
            with self._lock:
                self._arm_in_progress = False

    def _step_once(self, dt: float) -> None:
        with self._lock:
            intent = dict(self._intent)
            iseq = self._intent_seq
            btn = dict(self._btn)
            if btn["arm"] and self.deployment_held():
                btn["arm"] = False
            self._arm_in_progress = bool(btn["arm"])
            self._btn = {"arm": False, "disarm": False, "estop": False, "clear": False}
            pending_gait = self._pending_gait
            self._pending_gait = None
            pending_guided = self._pending_guided
            self._pending_guided = None
            pending_pose = self._pending_pose
            self._pending_pose = None
            pending_trick = self._pending_trick
            self._pending_trick = None
            sim_reset = self._sim_reset
            self._sim_reset = False

        if pending_guided is not None:
            cid, seed = pending_guided
            conflict = (self.controller.state.armed or btn["arm"] or self.auto_mode
                        or pending_gait or pending_pose or pending_trick
                        or any(abs(intent[k]) > .01 for k in ("vx", "vy", "wz", "body_z", "sway", "pitch")))
            if conflict:
                self.guided_load = {"status": "refused", "candidate_id": cid,
                    "reason": "Robot state changed before loading. Disarm, center controls and try again."}
            else:
                self.controller.stop_trick()
                self.controller.clear_pose()
                self.controller.gait.seed = seed
                self.controller.gait.clock.reset()
                self.controller.state.soft_start = 0.0
                self.guided_load = {"status": "loaded", "candidate_id": cid}

        if pending_gait is not None:
            seed, smooth = pending_gait
            self.controller.gait.seed = seed
            if smooth:
                # Re-dip the amplitude ramp so the new gait fades in.
                self.controller.state.soft_start = min(self.controller.state.soft_start, 0.35)

        # AUTO mode: high forward stick engages mode-fast, low stick mode-fluid
        # (hysteresis band 0.55..0.75, 1.5 s dwell so it never flaps).
        if self.auto_mode and pending_gait is None:
            profiles = self.controller.config.gait_profiles
            mag = abs(intent["vy"])
            want = self._mode_current
            if mag > 0.75:
                want = "mode-fast"
            elif mag < 0.55:
                want = "mode-fluid"
            if (want and want != self._mode_current and want in profiles
                    and time.time() - self._mode_last_switch > 1.5):
                self._mode_current = want
                self._mode_last_switch = time.time()
                try:
                    self.update_gait(profiles[want].model_dump(), smooth=True)
                except ValueError:
                    pass
        if pending_pose is not None:
            action, feet, name = pending_pose
            if action == "apply" and feet:
                self.controller.hold_pose(feet, name)
            elif action == "clear":
                self.controller.clear_pose()
        if pending_trick is not None:
            action, name, steps = pending_trick
            if action == "run" and steps:
                self.controller.start_trick(name or "?", steps)
            elif action == "stop":
                self.controller.stop_trick()

        # Clear E-STOP only while disarmed (latched until then).
        if btn["clear"] and not self.controller.state.armed:
            self.controller.clear_estop()

        fresh = iseq != self._consumed_seq
        if fresh or btn["estop"] or btn["arm"] or btn["disarm"]:
            self._consumed_seq = iseq
            self._cmd_seq += 1
            cmd = {"seq": self._cmd_seq, "t_host": time.time(), **intent}
            if btn["estop"]:
                cmd["estop"] = True
            elif btn["arm"]:
                cmd["arm"] = True
            elif btn["disarm"]:
                cmd["arm"] = False
            prev_armed = self.controller.state.armed
            # The torque-off loop has already stored stand goals in every
            # servo.  Capture the real collapsed/current pose and replace those
            # goals before ARM, otherwise torque-on can demand the full stand
            # move from all loaded joints instantaneously.  A partial capture
            # is a hard arm refusal; dry/sim has no driver and needs no capture.
            capture_ok = True
            if btn["arm"] and not prev_armed and not self.controller.state.estop:
                captured, total = self.controller.prepare_arm()
                capture_ok = self.controller.driver is None or (total > 0 and captured == total)
                if not capture_ok:
                    cmd["arm"] = False
                if self.deployment_held():
                    cmd["arm"] = False
                    capture_ok = False
            self.controller.submit_command(cmd)
            # Edge-triggered torque, on this (driver-owning) thread.
            if cmd.get("estop"):
                self.controller.apply_torque(False)
            elif (btn["arm"] and capture_ok and self.controller.state.armed
                  and not prev_armed):
                self.controller.apply_torque(True)
            elif btn["disarm"]:
                self.controller.apply_torque(False)

        self._update_attitude()
        self.controller.tick(dt)
        if self.guided_monitor is not None:
            self.guided_monitor.observe(self.controller, dt, self.auto_mode, self.link_lost)
        if self.twin is not None:
            if sim_reset:
                self.twin.reset()
            self.twin.step(self.controller.last_targets)
        with self._lock:
            self._build_snapshot()

    def _update_attitude(self) -> None:
        """Publish level-referenced (roll, pitch) to the controller.

        None whenever there is no sensor or the reading is not trustworthy, so
        the stabilizer stands down rather than acting on stale attitude.
        """
        if self.imu is None:
            self.controller.attitude = None
            return
        snap = self.imu.snapshot()
        st = getattr(self.controller.config, "stabilizer", None)
        try:
            age_s = float(snap.get("age_s"))
        except (TypeError, ValueError):
            age_s = math.inf
        max_age_s = st.max_imu_age_s if st else 0.25
        if (not snap.get("ok") or not math.isfinite(age_s) or age_s > max_age_s):
            self.controller.attitude = None
            return
        zero_r = st.level_roll_deg if st else 0.0
        zero_p = st.level_pitch_deg if st else 0.0
        self.controller.attitude = (snap["roll_deg"] - zero_r,
                                    snap["pitch_deg"] - zero_p)

    def _build_snapshot(self) -> None:
        self._snapshot = {
            "telemetry": self.controller.telemetry(),
            "pose": self.controller.pose_snapshot(),
            "gait_seed": self.controller.gait.seed.model_dump(),
            "hz": self.hz,
            "link_lost": self.link_lost,
            "auto_mode": self.auto_mode,
            "mode": self._mode_current,
            "guided_load": dict(self.guided_load),
            "guided_check_busy": self.guided_check_busy,
        }
        if self.twin is not None:
            self._snapshot["sim"] = self.twin.snapshot()

    # -- thread lifecycle ------------------------------------------------
    def start(self) -> None:
        if self._running:
            return
        # Enforce "boots disarmed, torque off": actively release torque on every
        # servo before the loop starts streaming stand goals. Without this, a robot
        # left torqued (e.g. from a killed D1 session) would be driven to the stand
        # pose the instant operate launches — motion with no ARM. Safe/no-op in dry.
        self.controller.disarm()
        self.controller.apply_torque(False)
        self._running = True
        self._thread = threading.Thread(target=self._run, name="dogv3-control", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        last = time.perf_counter()
        try:
            while self._running:
                now = time.perf_counter()
                step = now - last
                last = now
                self.step_safe(step if step > 0 else self.dt)
                busy = time.perf_counter() - now
                self._tick_ms.append(busy * 1000.0)
                if busy > self.dt:
                    self.overruns += 1
                sleep = self.dt - busy
                if sleep > 0:
                    time.sleep(sleep)
        finally:
            # Always leave the robot safe (best-effort if the link is gone).
            try:
                self.controller.disarm()
                self.controller.apply_torque(False)
            except Exception:
                pass

    def step_safe(self, dt: float) -> None:
        """One step that survives a dying serial link. If the driver throws
        (cable pulled, port re-enumerated), mark link_lost, detach the driver,
        and keep ticking dry — telemetry keeps flowing so the GUI can show LINK
        LOST instead of freezing at a stale ARMED display."""
        try:
            self.step_once(dt)
        except Exception:
            self.link_lost = True
            # Detach the dead driver and keep ticking dry. NOTE: with the link
            # gone we cannot command the servos at all — they hold their last
            # goal until robot power is cut. The GUI banner tells the operator.
            self.controller.driver = None
            self.controller.disarm()
            with self._lock:
                self._build_snapshot()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None


def build_app(loop: ControlLoop, *, link: str | None, dry: bool, config_path: Path,
              token: str = "", imu=None, robot_link: RobotLink | None = None):
    app = FastAPI(title="DogV3 Operate (D2)")
    loop.deployment_hold_path = Path(config_path).parent / ".deployment-hold"
    release_path = Path(config_path).parent / ".robot-release.json"
    release_id = None
    if release_path.exists():
        try:
            release_id = json.loads(release_path.read_text()).get("release_id")
        except (OSError, ValueError):
            pass
    # Track the config file we loaded so the GUI can flag on-disk changes
    # (e.g. D1 re-commissioning while D2 is live) and unsaved gait tuning.
    state = {"loaded_mtime": _mtime(config_path), "gait_dirty": False, "active_profile": None,
             # Single-driver rule: the WS connection id whose intent/ARM the
             # loop accepts. E-STOP/DISARM are honored from every client.
             "driver_id": None}
    ws_ids = itertools.count(1)

    if token:
        @app.middleware("http")
        async def _require_key(request, call_next):
            # Shared-key gate for non-loopback binds: a drive-by page in any
            # LAN browser can POST cross-origin, but it cannot read this key.
            if (request.query_params.get("key") == token
                    or request.headers.get("x-dogv3-key") == token):
                return await call_next(request)
            return JSONResponse({"detail": "missing or bad key (append ?key=... )"}, status_code=401)

    def live_seed() -> GaitSeed:
        """Pending edit if one is staged, else the applied seed."""
        with loop._lock:
            return loop._pending_gait[0] if loop._pending_gait else loop.controller.gait.seed

    def status_payload() -> dict:
        snap = loop.snapshot()
        snap["link"] = link
        snap["dry"] = dry
        snap["release_id"] = release_id
        snap["deployment_hold"] = loop.deployment_held()
        with loop._lock:
            snap["arm_in_progress"] = loop._arm_in_progress or loop._btn["arm"]
        snap["gait_meta"] = {k: {"min": v[0], "max": v[1], "step": v[2]} for k, v in GAIT_SLIDERS.items()}
        snap["config"] = str(config_path)
        snap["gait_dirty"] = state["gait_dirty"]
        snap["active_profile"] = state["active_profile"]
        snap["config_changed_on_disk"] = _mtime(config_path) != state["loaded_mtime"]
        snap["presets"] = list(GAIT_PRESETS)
        snap["poses"] = sorted(loop.controller.config.poses)
        snap["tricks"] = sorted(loop.controller.config.tricks)
        snap["profiles"] = {
            name: profile_summary(seed)
            for name, seed in sorted(loop.controller.config.gait_profiles.items())
        }
        snap["profile_hash"] = hashlib.sha256(json.dumps({name: seed.model_dump()
            for name, seed in loop.controller.config.gait_profiles.items()}, sort_keys=True).encode()).hexdigest()
        snap["driver_id"] = state["driver_id"]
        snap["loop_ms"] = loop.tick_stats()
        cfg = loop.controller.config
        if cfg.peripherals.camera.enabled:
            snap["camera_url"] = cfg.network.camera_stream_url()
        # Lidar and audio ride in the operate page as panels rather than as
        # their own tabs: pad polling is focus-gated, so a second tab taking
        # focus stalls intent into hold-stand. Sim has no real peripherals
        # (snap["sim"] is set by the twin, same test renderCam uses).
        in_sim = bool(snap.get("sim"))
        if cfg.peripherals.lidar.enabled and not in_sim:
            snap["lidar_url"] = cfg.network.lidar_viewer_url()
        snap["audio_enabled"] = bool(cfg.peripherals.audio.enabled and not in_sim)
        if imu is not None:
            snap["imu"] = imu.snapshot()
        if robot_link is not None:
            snap["robot"] = robot_link.status()
        return snap

    from .guided_api import install_guided_routes
    install_guided_routes(app, loop, config_path, dry=dry, live_seed=live_seed,
                          state=state, file_changed=lambda: _mtime(config_path) != state["loaded_mtime"])

    @app.get("/", response_class=HTMLResponse)
    def index() -> str:
        # The page needs the key for its own fetches/WS; injected server-side
        # so it is never guessable from markup alone (the page itself is only
        # reachable with the key when one is set).
        return INDEX_HTML.replace("__KEY__", token)

    @app.get("/static/three.min.js")
    def three_js():
        """Vendored locally so the 3D wireframe — which is how the IMU is
        visualized — survives with no internet (field operation off the Opal
        with no uplink). The page keeps its `typeof THREE` guard, so a missing
        file degrades to "viewer offline" instead of breaking the controls."""
        path = Path(__file__).parent / "static" / "three.min.js"
        if not path.exists():
            raise HTTPException(404, "three.min.js not vendored")
        return FileResponse(path, media_type="application/javascript",
                            headers={"Cache-Control": "max-age=86400"})

    def _audio_request(path: str, body: dict | None = None) -> dict:
        """Proxy one call to the Pi's audio service. Server-side so the browser
        never makes a cross-origin request (no CORS on that service), and so a
        wedged audio box surfaces as an error string instead of a hung fetch."""
        url = f"{loop.controller.config.network.audio_base_url()}{path}"
        data = json.dumps(body).encode() if body is not None else None
        req = urllib.request.Request(url, data=data, method="POST" if data else "GET")
        req.add_header("Content-Type", "application/json")
        try:
            with urllib.request.urlopen(req, timeout=2.0) as resp:
                return json.loads(resp.read().decode())
        except Exception as e:  # service down / not enabled / wrong host
            return {"ok": False, "error": str(e)}

    @app.get("/api/audio/clips")
    def audio_clips() -> dict:
        return _audio_request("/api/clips")

    @app.get("/api/audio/voices")
    def audio_voices() -> dict:
        return _audio_request("/api/voices")

    @app.post("/api/audio/play")
    def audio_play(body: dict) -> dict:
        name = str(body.get("name", ""))
        if not name:
            raise HTTPException(400, "name required")
        return _audio_request("/api/play", {"name": name})

    @app.post("/api/audio/say")
    def audio_say(body: dict) -> dict:
        text = str(body.get("text", "")).strip()
        if not text:
            raise HTTPException(400, "text required")
        voice = str(body.get("voice", "en-us")).strip()
        volume = body.get("volume")
        rate_wpm = body.get("rate_wpm", 90)
        return _audio_request("/api/say", {
            "text": text, "voice": voice, "volume": volume, "rate_wpm": rate_wpm})

    @app.get("/api/status")
    def status() -> dict:
        return status_payload()

    @app.post("/api/intent")
    def intent(body: IntentBody) -> dict:
        # Headless/scripted path. Refused while a browser session holds the
        # driver claim — REST has no session identity, so it may not compete
        # with (or keep the watchdog fed behind) the active driver.
        if state["driver_id"] is not None:
            raise HTTPException(409, "a WebSocket client is driving; REST intent refused")
        loop.submit_intent(body.vx, body.vy, body.wz, body.body_z,
                           body.sway, body.sway_max_mm,
                           body.pitch, body.pitch_max_deg)
        return {"ok": True}

    @app.post("/api/button/{action}")
    def button(action: str) -> dict:
        # Stop-type buttons are honored from anywhere; ARM only when no
        # browser session holds the driver claim.
        if action == "arm" and state["driver_id"] is not None:
            raise HTTPException(409, "a WebSocket client is driving; ARM it from that GUI")
        try:
            loop.press(action)
        except ValueError as e:
            raise HTTPException(400, str(e))
        return {"ok": True, "action": action}

    @app.post("/api/gait")
    def gait(body: GaitBody) -> dict:
        try:
            seed = loop.update_gait(body.levers)
        except ValueError as e:
            raise HTTPException(400, str(e))
        state["gait_dirty"] = True
        return {"ok": True, "gait_seed": seed}

    @app.post("/api/gait/preset")
    def gait_preset(body: PresetBody) -> dict:
        levers = GAIT_PRESETS.get(body.name)
        if levers is None:
            raise HTTPException(400, f"unknown preset {body.name!r}")
        seed = loop.update_gait(dict(levers), smooth=True)
        state["gait_dirty"] = True
        state["active_profile"] = None  # a preset replaces whatever profile was loaded
        return {"ok": True, "preset": body.name, "gait_seed": seed}

    @app.post("/api/gait/save")
    def gait_save() -> dict:
        """Persist the live-tuned gait_seed back into robot_config.json — this
        closes the SSOT loop: D1 commissions the config, D2 tunes it, and the
        tuned values survive a restart / feed the sim tools."""
        cfg = loop.controller.config
        with loop._lock:
            cfg.gait_seed = loop._pending_gait[0] if loop._pending_gait else loop.controller.gait.seed
        save_config(cfg, config_path)
        state["gait_dirty"] = False
        state["loaded_mtime"] = _mtime(config_path)
        return {"saved": str(config_path), "gait_seed": cfg.gait_seed.model_dump()}

    @app.post("/api/profile/save")
    def profile_save(body: PresetBody) -> dict:
        """Snapshot the live (pending-or-applied) seed as a named profile.
        Profiles are the trial-and-error library: one lever rarely improves a
        gait in isolation, so save whole candidates and A/B them by loading."""
        name = body.name.strip()
        if not name:
            raise HTTPException(400, "profile name required")
        cfg = loop.controller.config
        cfg.gait_profiles[name] = live_seed().model_copy(deep=True)
        save_config(cfg, config_path)
        state["active_profile"] = name
        state["gait_dirty"] = False
        state["loaded_mtime"] = _mtime(config_path)
        return {"saved": name, "profiles": sorted(cfg.gait_profiles)}

    @app.post("/api/profile/load")
    def profile_load(body: PresetBody) -> dict:
        seed = loop.controller.config.gait_profiles.get(body.name)
        if seed is None:
            raise HTTPException(400, f"unknown profile {body.name!r}")
        out = loop.update_gait(seed.model_dump(), smooth=True)
        state["active_profile"] = body.name
        state["gait_dirty"] = False
        return {"ok": True, "loaded": body.name, "gait_seed": out}

    @app.post("/api/profile/delete")
    def profile_delete(body: PresetBody) -> dict:
        cfg = loop.controller.config
        if body.name not in cfg.gait_profiles:
            raise HTTPException(400, f"unknown profile {body.name!r}")
        del cfg.gait_profiles[body.name]
        save_config(cfg, config_path)
        if state["active_profile"] == body.name:
            state["active_profile"] = None
        state["loaded_mtime"] = _mtime(config_path)
        return {"deleted": body.name, "profiles": sorted(cfg.gait_profiles)}

    @app.post("/api/profile/import")
    def profile_import(body: ProfileImportBody) -> dict:
        """Receive a full profile from another session (D3 sim push) or a
        paste. Save-only: never touches the live controller, so it cannot move
        an armed robot. The operator loads it deliberately afterwards."""
        name = body.name.strip()
        if not name:
            raise HTTPException(400, "profile name required")
        try:
            seed = GaitSeed.model_validate(body.seed)
        except Exception as e:
            raise HTTPException(400, f"bad gait seed: {e}")
        cfg = loop.controller.config
        cfg.gait_profiles[name] = seed
        save_config(cfg, config_path)
        state["loaded_mtime"] = _mtime(config_path)
        return {"imported": name, "profiles": sorted(cfg.gait_profiles)}

    @app.post("/api/robot/comms")
    def robot_comms(body: CommsBody) -> dict:
        if robot_link is None:
            raise HTTPException(400, "robot link only exists in --sim sessions")
        robot_link.set_comms(body.on)
        return {"ok": True, **robot_link.status()}

    @app.post("/api/robot/push")
    def robot_push(body: PresetBody) -> dict:
        if robot_link is None:
            raise HTTPException(400, "robot link only exists in --sim sessions")
        seed = loop.controller.config.gait_profiles.get(body.name)
        if seed is None:
            raise HTTPException(400, f"unknown profile {body.name!r}")
        if not robot_link.status().get("comms_on"):
            raise HTTPException(409, "robot comms is off — toggle it on first")
        try:
            resp = robot_link.push_profile(body.name, seed.model_dump())
        except (urllib.error.URLError, OSError, ValueError) as e:
            raise HTTPException(502, f"push failed: {e}")
        return {"ok": True, "robot": resp}

    @app.post("/api/mode/auto")
    def mode_auto(body: dict) -> dict:
        on = bool(body.get("on"))
        profiles = loop.controller.config.gait_profiles
        if on and not {"mode-fluid", "mode-fast"} <= set(profiles):
            raise HTTPException(400, "AUTO needs 'mode-fluid' and 'mode-fast' gait profiles")
        loop.auto_mode = on
        loop._mode_current = None  # re-engage on the next intent
        return {"ok": True, "auto_mode": on}

    @app.post("/api/trick/run")
    def trick_run(body: PoseBody) -> dict:
        cfg = loop.controller.config
        steps_def = cfg.tricks.get(body.name)
        if steps_def is None:
            raise HTTPException(400, f"unknown trick {body.name!r}")
        resolved = []
        for st in steps_def:
            feet = cfg.poses.get(st.pose)
            if feet is None:
                raise HTTPException(400, f"trick {body.name!r} references unknown pose {st.pose!r}")
            resolved.append((feet, st.move_s, st.hold_s))
        loop.stage_trick("run", body.name, resolved)
        return {"ok": True, "running": body.name}

    @app.post("/api/trick/stop")
    def trick_stop() -> dict:
        loop.stage_trick("stop")
        return {"ok": True}

    @app.post("/api/trick/save")
    def trick_save(body: TrickBody) -> dict:
        name = body.name.strip()
        if not name:
            raise HTTPException(400, "trick name required")
        if not body.steps:
            raise HTTPException(400, "trick needs at least one step")
        cfg = loop.controller.config
        try:
            steps = [TrickStep.model_validate(s) for s in body.steps]
        except Exception as e:
            raise HTTPException(400, f"bad step: {e}")
        for st in steps:
            if st.pose not in cfg.poses:
                raise HTTPException(400, f"unknown pose {st.pose!r}")
        cfg.tricks[name] = steps
        save_config(cfg, config_path)
        state["loaded_mtime"] = _mtime(config_path)
        return {"saved": name, "tricks": sorted(cfg.tricks)}

    @app.post("/api/trick/delete")
    def trick_delete(body: PoseBody) -> dict:
        cfg = loop.controller.config
        if body.name not in cfg.tricks:
            raise HTTPException(400, f"unknown trick {body.name!r}")
        del cfg.tricks[body.name]
        save_config(cfg, config_path)
        state["loaded_mtime"] = _mtime(config_path)
        return {"deleted": body.name, "tricks": sorted(cfg.tricks)}

    @app.post("/api/sim/reset")
    def sim_reset() -> dict:
        if loop.twin is None:
            raise HTTPException(400, "not running in --sim mode")
        loop.reset_sim()
        return {"ok": True}

    @app.post("/api/gait/analyze")
    def gait_analyze(body: AnalyzeBody) -> dict:
        """Kinematic-twin check (no hardware): roll the seed through the real
        gait + IK and report speed, joint-velocity demand vs the servo budget,
        commissioned-range clamps, and reach violations."""
        cfg = loop.controller.config
        if body.profile is not None:
            seed = cfg.gait_profiles.get(body.profile)
            if seed is None:
                raise HTTPException(400, f"unknown profile {body.profile!r}")
        else:
            seed = live_seed()
        cmd = GaitCommand(vx=body.vx, vy=body.vy, wz=body.wz)
        return {"ok": True, "analysis": evaluate_gait(cfg, seed, cmd).to_dict()}

    def _pose_error(feet: dict) -> str | None:
        """Kinematic validity: every foot target reachable (IK->FK closes)."""
        cfg = loop.controller.config
        for leg, p in feet.items():
            geom = LegGeometry(L1=cfg.links_mm.L1_hip, L2=cfg.links_mm.L2_femur,
                               L3=cfg.links_mm.L3_tibia, side=cfg.legs[leg].side,
                               knee_sign=cfg.legs[leg].knee_sign)
            fk = forward_kinematics(geom, *inverse_kinematics(geom, *p))
            err = sum((a - b) ** 2 for a, b in zip(fk, p)) ** 0.5
            if err > 2.0:
                return f"{leg} target {p} is {err:.1f} mm outside leg reach"
        return None

    @app.post("/api/pose/save")
    def pose_save(body: PoseBody) -> dict:
        name = body.name.strip()
        if not name:
            raise HTTPException(400, "pose name required")
        cfg = loop.controller.config
        feet = loop.controller.capture_pose()
        err = _pose_error(feet)
        if err:
            raise HTTPException(400, f"pose not kinematically valid: {err}")
        cfg.poses[name] = feet
        save_config(cfg, config_path)
        state["loaded_mtime"] = _mtime(config_path)
        return {"saved": name, "poses": sorted(cfg.poses)}

    @app.post("/api/pose/apply")
    def pose_apply(body: PoseBody) -> dict:
        feet = loop.controller.config.poses.get(body.name)
        if feet is None:
            raise HTTPException(400, f"unknown pose {body.name!r}")
        loop.stage_pose("apply", feet, body.name)
        return {"ok": True, "applied": body.name}

    @app.post("/api/pose/clear")
    def pose_clear() -> dict:
        loop.stage_pose("clear")
        return {"ok": True}

    @app.post("/api/pose/delete")
    def pose_delete(body: PoseBody) -> dict:
        cfg = loop.controller.config
        if body.name not in cfg.poses:
            raise HTTPException(400, f"unknown pose {body.name!r}")
        del cfg.poses[body.name]
        save_config(cfg, config_path)
        state["loaded_mtime"] = _mtime(config_path)
        return {"deleted": body.name, "poses": sorted(cfg.poses)}

    @app.websocket("/ws/operate")
    async def ws_operate(ws: WebSocket) -> None:
        """Single-task socket loop: push telemetry at TELEMETRY_HZ, and drain
        any queued client frames between pushes. One task means no cancel/join
        choreography on disconnect — the session ends by returning.

        Driver claim: the first client (or an explicit "claim" frame) becomes
        THE driver; everyone else is a viewer. Only the driver's intent frames
        refresh the watchdog and only the driver may ARM — but E-STOP, DISARM
        and CLEAR are honored from any client, always."""
        if token and ws.query_params.get("key") != token:
            await ws.close(code=4401)
            return
        await ws.accept()
        my_id = next(ws_ids)
        if state["driver_id"] is None:
            state["driver_id"] = my_id
        await ws.send_text(json.dumps({"type": "hello", "you": my_id}))
        period = 1.0 / TELEMETRY_HZ

        def handle(raw: str) -> None:
            try:
                msg = json.loads(raw)
            except (ValueError, TypeError):
                return  # ignore one malformed frame, keep the link alive
            kind = msg.get("type")
            driving = state["driver_id"] == my_id
            if kind == "claim":
                # Explicit takeover (GUI button): last claim wins; the old
                # driver's tab flips to VIEWING on the next status push.
                state["driver_id"] = my_id
            elif kind == "intent":
                if driving:
                    loop.submit_intent(
                        msg.get("vx", 0.0), msg.get("vy", 0.0),
                        msg.get("wz", 0.0), msg.get("body_z", 0.0),
                        msg.get("sway", 0.0), msg.get("sway_max_mm", DEFAULT_MANUAL_SWAY_MM),
                        msg.get("pitch", 0.0), msg.get("pitch_max_deg", DEFAULT_MANUAL_PITCH_DEG),
                    )
            elif kind == "button":
                action = msg.get("action", "")
                if action == "arm" and not driving:
                    return  # arming authority belongs to the driver only
                try:
                    loop.press(action)
                except ValueError:
                    pass
            elif kind == "gait":
                try:
                    loop.update_gait(msg.get("levers", {}))
                    state["gait_dirty"] = True  # slider edits arrive here, not /api/gait
                except ValueError:
                    pass

        try:
            while True:
                await ws.send_text(json.dumps(status_payload()))
                remaining = period
                while True:  # drain frames until the next push is due
                    try:
                        raw = await asyncio.wait_for(ws.receive_text(), timeout=remaining)
                    except asyncio.TimeoutError:
                        break
                    handle(raw)
                    remaining = 0.002  # flush any burst without delaying the push
        except (WebSocketDisconnect, RuntimeError):
            return  # client closed (or send on a closing socket): session over
        finally:
            if state["driver_id"] == my_id:
                # Driver gone: nobody feeds the watchdog until another tab
                # claims — stale intent then holds stand / disarms as documented.
                state["driver_id"] = None

    return app


INDEX_HTML = """<!doctype html><html><head><meta charset=utf-8>
<title>DogV3 Operate</title>
<style>
 :root{color-scheme:dark}
 body{font-family:system-ui,sans-serif;margin:0;display:flex;height:100vh;background:#0e1116;color:#e6e6e6}
 #view{flex:1;position:relative}
 #panel{width:clamp(360px,24vw,500px);padding:14px 16px;overflow:auto;border-left:1px solid #2a2f37;background:#11151b}
 h1{font-size:18px;margin:0 0 8px} h2{font-size:13px;margin:16px 0 6px;color:#9aa4b2;text-transform:uppercase;letter-spacing:.04em}
 .row{display:flex;align-items:center;gap:8px;margin:5px 0}
 .sl label{display:inline-block;width:140px;font-size:13px} .sl input[type=range]{flex:1} .sl .v{width:54px;text-align:right;font-variant-numeric:tabular-nums;font-size:13px}
 button{border:0;border-radius:6px;padding:10px 11px;color:#fff;font-weight:600;cursor:pointer;font-size:14px}
 #arm{background:#1f9d55}#disarm{background:#4b5563}#clear{background:#b45309}
 #estop{background:#dc2626;width:100%;padding:14px;font-size:17px;margin-top:6px;position:sticky;bottom:8px;z-index:5;box-shadow:0 0 0 8px #11151b}
 .btns{display:flex;gap:6px}.btns button{flex:1}
 .pill{display:inline-block;padding:3px 9px;border-radius:999px;font-size:12px;font-weight:700}
 .on{background:#1f9d55}.off{background:#374151}.warn{background:#b45309}.bad{background:#dc2626}
 table{width:100%;border-collapse:collapse;font-size:12.5px} td{padding:2px 4px} td.k{color:#9aa4b2}
 #banner{position:absolute;top:12px;left:12px;padding:8px 14px;border-radius:6px;font-weight:700;font-size:16px}
 select{background:#1a1f27;color:#e6e6e6;border:1px solid #2a2f37;border-radius:5px;padding:4px;font-size:13px}
 .hint{font-size:12px;color:#7e8794;margin:2px 0 0}
 #sticks{display:flex;gap:10px;justify-content:space-between}
 .pad{flex:1;aspect-ratio:1;background:#1a1f27;border:1px solid #2a2f37;border-radius:8px;position:relative;touch-action:none}
 .pad .dot{position:absolute;width:18px;height:18px;border-radius:50%;background:#3b82f6;transform:translate(-50%,-50%);left:50%;top:50%}
 .pad .cap{position:absolute;bottom:3px;width:100%;text-align:center;font-size:10px;color:#7e8794}
</style><link rel="stylesheet" href="/static/guided-tuner.css?v=2&key=__KEY__"></head><body>
<div id=view><div id=banner></div></div>
<div id=panel>
 <div id=operator-controls>
 <h1>DogV3</h1>
 <p id=release-label class=hint></p>
 <div class=row><span id=armpill class="pill off">DISARMED</span><span id=estoppill class="pill off">E-STOP CLEAR</span><span id=wdpill class="pill off">intent</span><span id=drivepill class="pill off">VIEWING</span><button id=takectl style="background:#374151;padding:3px 9px;font-size:12px;display:none">Take control</button></div>
 <div class=btns style="margin-top:6px"><button id=arm>ARM (X)</button><button id=disarm>DISARM (O)</button><button id=clear>CLEAR</button><button id=simreset style="background:#7c3aed;display:none">RESET SIM</button></div>
 <button id=estop>■ E-STOP (btn 9)</button>
 </div>
 <section id="guided-tuner" aria-label="Gait test set"><h2>Test a gait</h2><p>Loading test set…</p></section>
 <p class=hint>Hold W or push the left stick fully forward for a test. Release to stop.</p>
 <div id=sticks>
  <div class=pad id=padL><div class=dot></div><div class=cap>forward / sideways</div></div>
  <div class=pad id=padR><div class=dot></div><div class=cap>turn</div></div>
 </div>
 <details id="advanced-controls"><summary>Advanced controls</summary>
 <details><summary>Controller mapping and stand lean</summary>
  <p class=hint>DualShock 4: L2/R2 shift left/right; L1/R1 tilt nose-down/nose-up. Both are hold-to-run, stand-only, and recenter before walking. Left stick drives, right stick turns, D-pad trims height. Keyboard: WASD move, Q/E turn, Space=E-STOP. Pad input only reaches the robot while this tab is focused AND driving.</p>
  <div class="row sl"><label>L2/R2 sway</label><input id=swayamount type=range min=0 max=70 step=1 value=20 oninput="setSwayMax(this.value)"><span class=v id=swayamountout>20 mm</span></div>
  <div class="row sl"><label>L1/R1 pitch</label><input id=pitchamount type=range min=0 max=12 step=.5 value=5 oninput="setPitchMax(this.value)"><span class=v id=pitchamountout>5.0°</span></div>
  <p class=hint>Trial-1 ceilings: 70 mm lateral and 12° pitch. Start low. These are kinematic software limits, not physically validated no-tip limits.</p>
 </details>
 <details id=camwrap style="display:none;margin:6px 0"><summary style="cursor:pointer;color:#9aa4b2;font-size:13px">Robot vision (camera)</summary><img id=cam style="width:100%;border-radius:6px;margin-top:6px" alt="camera stream"></details>
 <details id=lidarwrap style="display:none;margin:6px 0"><summary style="cursor:pointer;color:#9aa4b2;font-size:13px">Lidar (point cloud)</summary><iframe id=lidarframe style="width:100%;height:360px;border:0;border-radius:6px;margin-top:6px;background:#0d1117" title="lidar point cloud"></iframe></details>
 <details id=sfxwrap style="display:none;margin:6px 0"><summary style="cursor:pointer;color:#9aa4b2;font-size:13px">Sound</summary>
  <div id=sfxbtns class=row style="flex-wrap:wrap;gap:4px;margin-top:6px"></div>
  <div class=row style="margin-top:4px"><select id=sayvoice aria-label="Voice" style="max-width:150px;background:#1a1f27;color:#e6e6e6;border:1px solid #2a2f37;border-radius:5px;padding:5px"></select><input id=saytext placeholder="say something" style="flex:1;background:#1a1f27;color:#e6e6e6;border:1px solid #2a2f37;border-radius:5px;padding:5px"><button onclick=saySpeak() style="background:#2563eb">Say</button></div>
  <div class="row sl"><label>Voice volume</label><input id=sayvolume type=range min=0 max=100 step=1 value=80 oninput="document.getElementById('sayvolout').textContent=this.value+'%'"><span class=v id=sayvolout>80%</span></div>
  <div class="row sl"><label>Speech speed</label><input id=sayrate type=range min=80 max=220 step=5 value=90 oninput="document.getElementById('sayrateout').textContent=this.value+' WPM'"><span class=v id=sayrateout>90 WPM</span></div>
  <div id=sfxerr class=hint style="color:#f59e0b"></div>
 </details>
 <h2>Drive</h2>
 <details><summary>Drive mode shortcuts</summary>
 <div class=row><button id=mfluid style="background:#0e7490;flex:1">FLUID (1)</button><button id=mfast style="background:#7c3aed;flex:1">FAST (2)</button><button id=mauto style="background:#374151;flex:1">AUTO off</button></div>
 <p class=hint>FLUID/FAST load the reserved mode profiles with a smooth blend. AUTO swaps them by stick: &gt;75% forward engages FAST, &lt;55% returns to FLUID (1.5 s dwell).</p>
 </details>
  <details><summary>Live telemetry</summary>
  <h2>Telemetry</h2>
  <table id=tel></table>
  </details>
  <details id="manual-tuning"><summary>Advanced: manual tuning and saved profiles</summary>
 <h2>Gait tuning</h2>
 <div class="row sl"><label>gait</label>
  <select id=gaitsel onchange="setMode(this.value)"><option value=trot>trot (fast, dynamic)</option><option value=crawl>crawl (slow, stable)</option></select></div>
 <p class=hint id=gaithint></p>
 <div id=sliders></div>
 <div class="row sl"><label>swing_shape</label>
  <select id=swing onchange="setSwing(this.value)"><option>parabola</option><option>sine</option><option>cycloid</option><option>flick</option></select></div>
 <div class=row style="flex-wrap:wrap;gap:4px" id=presets></div>
 <div class=row><button id=savegait style="background:#2563eb;flex:1" onclick=saveGait()>Save gait to config</button><span id=dirty class="pill off">saved</span></div>
 <h2>A/B one lever</h2>
 <p class=hint>Collapse on a style: pick one lever and two values, drive, and flip between them. Keep locks in whichever is live; then Save profile when a style feels right.</p>
 <div class=row><select id=ablever style="flex:1"></select><input id=abA style="width:64px;background:#1a1f27;color:#e6e6e6;border:1px solid #2a2f37;border-radius:5px;padding:5px"><input id=abB style="width:64px;background:#1a1f27;color:#e6e6e6;border:1px solid #2a2f37;border-radius:5px;padding:5px"><button id=abgo style="background:#4b5563">Start</button></div>
 <div class=row><button id=abtoggle style="flex:1;background:#0e7490;display:none"></button><button id=abkeep style="background:#1f9d55;display:none">Keep</button></div>
 <h2>Gait profiles</h2>
 <p class=hint>Named full lever sets in robot_config.json. Save every trial, then Load to A/B them — one lever alone rarely makes a better gait. <span id=profpill class="pill off"></span></p>
 <div class=row><input id=profname placeholder="profile name (e.g. trot-a1)" style="flex:1;background:#1a1f27;color:#e6e6e6;border:1px solid #2a2f37;border-radius:5px;padding:5px"><button onclick=saveProfile() style="background:#2563eb">Save profile</button></div>
 <div class=row><input id=proffilter placeholder="filter profiles (glide, sport, ...)" style="flex:1;background:#1a1f27;color:#e6e6e6;border:1px solid #2a2f37;border-radius:5px;padding:5px"></div>
  <div style="max-height:260px;overflow:auto"><table id=proftable></table></div>
  </details>
 <div id=robotlink style="display:none">
  <h2>Robot link (D3)</h2>
  <p class=hint>Heartbeat + save-only profile push. This panel can never arm or move the robot — arming and driving happen only on the robot's own operate page.</p>
  <div class=row><button id=rlcomms style="background:#374151;flex:1">COMMS off</button><span id=rlreach class="pill off">no comms</span><span id=rlarm class="pill off">-</span></div>
  <div class=row><select id=rlprof style="flex:1"></select><button id=rlpush style="background:#2563eb">Send to robot</button></div>
  <div class=row><a id=rlopen href="#" target=_blank style="color:#3b82f6;font-size:13px">Open robot operate page</a></div>
  <details id=rlcamwrap style="margin:6px 0"><summary style="cursor:pointer;color:#9aa4b2;font-size:13px">Robot vision</summary><img id=rlcam style="width:100%;border-radius:6px;margin-top:6px" alt="robot camera"></details>
 </div>
 <h2>Analyze (digital twin)</h2>
 <div class=row><button onclick=analyze() style="background:#4b5563;flex:1">Analyze current gait</button></div>
 <table id=antable></table>
 <div id=anwarn></div>
 <h2>Poses</h2>
 <div class=row><select id=poselist style="flex:1"></select><button onclick=applyPose() style="background:#4b5563">Hold</button><button onclick=clearPose() style="background:#4b5563">Stand</button></div>
 <div class=row><input id=posename placeholder="new pose name" style="flex:1;background:#1a1f27;color:#e6e6e6;border:1px solid #2a2f37;border-radius:5px;padding:5px"><button onclick=savePose() style="background:#2563eb">Save current</button></div>
 <h2>Tricks</h2>
 <p class=hint>A trick blends stand → each pose → stand (safe entry from any posture). Drive input or E-STOP cancels it instantly.</p>
 <div class=row><select id=tricklist style="flex:1"></select><button onclick=runTrick() style="background:#7c3aed">Run</button><button onclick=stopTrick() style="background:#4b5563">Stop</button></div>
 <div id=trsteps></div>
 <div class=row><button onclick=addTrickStep() style="background:#374151;flex:1;font-size:12px;padding:6px">+ add step (pose, move s, hold s)</button></div>
 <div class=row><input id=trickname placeholder="new trick name" style="flex:1;background:#1a1f27;color:#e6e6e6;border:1px solid #2a2f37;border-radius:5px;padding:5px"><button onclick=saveTrick() style="background:#2563eb">Save trick</button></div>
</details></div>
<script src="/static/three.min.js?key=__KEY__"></script>
<script>
const KEY='__KEY__';  // injected server-side when a shared key is required
const J=(u,m,b)=>fetch(u,{method:m||'GET',headers:{'Content-Type':'application/json',...(KEY?{'X-DogV3-Key':KEY}:{})},body:b?JSON.stringify(b):undefined}).then(r=>r.json());
let ws, drive={vx:0,vy:0,wz:0,body_z:0,sway:0,sway_max_mm:20,pitch:0,pitch_max_deg:5}, latest=null, builtSliders=false;
let myId=null, driving=false;  // single-driver claim: only the driver's intent/ARM count
function setSwayMax(v){
 const n=Math.max(0,Math.min(70,+v||0)); drive.sway_max_mm=n;
 document.getElementById('swayamountout').textContent=n.toFixed(0)+' mm';
 try{localStorage.setItem('dogv3-sway-max-mm',String(n));}catch(_e){}
}
function setPitchMax(v){
 const n=Math.max(0,Math.min(12,+v||0)); drive.pitch_max_deg=n;
 document.getElementById('pitchamountout').textContent=n.toFixed(1)+'°';
 try{localStorage.setItem('dogv3-pitch-max-deg',String(n));}catch(_e){}
}
try{const raw=localStorage.getItem('dogv3-sway-max-mm');if(raw!==null){const saved=+raw;if(Number.isFinite(saved))document.getElementById('swayamount').value=Math.max(0,Math.min(70,saved));}}catch(_e){}
try{const raw=localStorage.getItem('dogv3-pitch-max-deg');if(raw!==null){const saved=+raw;if(Number.isFinite(saved))document.getElementById('pitchamount').value=Math.max(0,Math.min(12,saved));}}catch(_e){}
setSwayMax(document.getElementById('swayamount').value);
setPitchMax(document.getElementById('pitchamount').value);

// ---- WebSocket: send intent at 20 Hz, receive telemetry ----
function connect(){
 ws=new WebSocket((location.protocol==='https:'?'wss':'ws')+'://'+location.host+'/ws/operate'+(KEY?'?key='+KEY:''));
 ws.onmessage=e=>{const d=JSON.parse(e.data); if(d.type==='hello'){myId=d.you;return;} latest=d; render(d);};
 ws.onclose=()=>setTimeout(connect,800);
}
// Only the driver heartbeats: a viewing tab must not keep the 10 s disarm
// watchdog fed (the server ignores viewer intent anyway).
function sendIntent(){ if(driving&&ws&&ws.readyState===1) ws.send(JSON.stringify({type:'intent',...drive})); }
setInterval(sendIntent,50);            // 20 Hz heartbeat keeps the watchdog fresh
function press(a){ if(ws&&ws.readyState===1) ws.send(JSON.stringify({type:'button',action:a})); else J('/api/button/'+a,'POST'); }
arm.onclick=()=>press('arm'); disarm.onclick=()=>press('disarm'); estop.onclick=()=>press('estop'); clear.onclick=()=>press('clear');
takectl.onclick=()=>{ if(ws&&ws.readyState===1) ws.send(JSON.stringify({type:'claim'})); };
simreset.onclick=()=>J('/api/sim/reset','POST');
mfluid.onclick=()=>loadProfile('mode-fluid'); mfast.onclick=()=>loadProfile('mode-fast');
mauto.onclick=()=>J('/api/mode/auto','POST',{on:!(latest&&latest.auto_mode)});

// ---- gait sliders ----
// Tuning-first layout: the five levers that define a trot are always visible;
// trim/sway/governors live behind the Advanced toggle so the panel stays sane.
const CORE_LEVERS=['cycle_period','step_length','step_height','duty_factor','body_height'];
let advOpen=false;
function sliderRow(k,m,seed){
 const d=document.createElement('div'); d.className='row sl';
 d.innerHTML=`<label>${k}</label><input type=range min=${m.min} max=${m.max} step=${m.step} value=${seed[k]} oninput="setGait('${k}',this.value)"><span class=v id="v_${k}">${(+seed[k]).toFixed(2)}</span>`;
 return d;
}
function buildSliders(meta,seed){
 const c=document.getElementById('sliders'); c.innerHTML='';
 for(const k of CORE_LEVERS) if(meta[k]) c.appendChild(sliderRow(k,meta[k],seed));
 const tog=document.createElement('button'); tog.id='advtoggle';
 tog.textContent=(advOpen?'Hide':'Show')+' advanced (trim, sway, governors)';
 tog.style.cssText='background:#1a1f27;border:1px solid #2a2f37;color:#9aa4b2;width:100%;padding:5px;font-size:11px;margin:4px 0';
 tog.onclick=()=>{advOpen=!advOpen;builtSliders=false;};
 c.appendChild(tog);
 const adv=document.createElement('div'); adv.style.display=advOpen?'':'none';
 for(const[k,m]of Object.entries(meta)) if(!CORE_LEVERS.includes(k)) adv.appendChild(sliderRow(k,m,seed));
 c.appendChild(adv);
 document.getElementById('swing').value=seed.swing_shape;
 document.getElementById('gaitsel').value=seed.gait_type; updateGaitHint(seed.gait_type);
 builtSliders=true;
}
function setGait(k,v){ document.getElementById('v_'+k).textContent=(+v).toFixed(2);
 if(ws&&ws.readyState===1) ws.send(JSON.stringify({type:'gait',levers:{[k]:parseFloat(v)}})); }
function setSwing(v){ if(ws&&ws.readyState===1) ws.send(JSON.stringify({type:'gait',levers:{swing_shape:v}})); }
function setMode(v){ if(ws&&ws.readyState===1) ws.send(JSON.stringify({type:'gait',levers:{gait_type:v}})); updateGaitHint(v); }
function updateGaitHint(v){ document.getElementById('gaithint').textContent = v==='crawl'
 ? 'Crawl: one foot at a time, three always planted (duty floored to 0.75). Raise body_sway_amp (~20–40) so the body leans over the support triangle — with zero sway it will tip.'
 : 'Trot: diagonal pairs; dynamic, needs speed/balance to stay upright.'; }

// ---- virtual sticks ----
function bindPad(id,onMove){
 const pad=document.getElementById(id), dot=pad.querySelector('.dot'); let active=false;
 const move=ev=>{const r=pad.getBoundingClientRect();const p=(ev.touches?ev.touches[0]:ev);
  let x=(p.clientX-r.left)/r.width*2-1, y=(p.clientY-r.top)/r.height*2-1;
  x=Math.max(-1,Math.min(1,x)); y=Math.max(-1,Math.min(1,y));
  dot.style.left=(50+x*42)+'%'; dot.style.top=(50+y*42)+'%'; onMove(x,y);};
 const end=()=>{active=false;dot.style.left='50%';dot.style.top='50%';onMove(0,0);};
 pad.addEventListener('pointerdown',e=>{active=true;move(e);});
 pad.addEventListener('pointermove',e=>{if(active)move(e);});
 window.addEventListener('pointerup',()=>{if(active)end();});
}
bindPad('padL',(x,y)=>{drive.vx=round(x);drive.vy=round(-y);});   // up = forward
bindPad('padR',(x,_y)=>{drive.wz=round(x);});
const round=v=>Math.abs(v)<0.08?0:Math.round(v*100)/100;

// ---- keyboard ----
const keys={};
function typingTarget(el){const tag=(el&&el.tagName||'').toUpperCase();return !!(el&&(el.isContentEditable||tag==='INPUT'||tag==='TEXTAREA'||tag==='SELECT'));}
function clearKeyboardDrive(){for(const k in keys)keys[k]=0;kbd();}
addEventListener('focusin',e=>{if(typingTarget(e.target))clearKeyboardDrive();});
addEventListener('keydown',e=>{if(typingTarget(e.target))return; keys[e.key.toLowerCase()]=1; if(e.key===' '){press('estop');e.preventDefault();}
 if(e.key==='1'&&latest?.profiles?.['mode-fluid'])loadProfile('mode-fluid'); if(e.key==='2'&&latest?.profiles?.['mode-fast'])loadProfile('mode-fast'); kbd();});
addEventListener('keyup',e=>{if(typingTarget(e.target))return; keys[e.key.toLowerCase()]=0; kbd();});
addEventListener('blur',()=>{clearKeyboardDrive(); drive.vx=drive.vy=drive.wz=drive.body_z=drive.sway=drive.pitch=0;});  // lost keyup/pad focus -> neutral
function kbd(){ drive.vy=(keys['w']?1:0)-(keys['s']?1:0); drive.vx=(keys['d']?1:0)-(keys['a']?1:0);
 drive.wz=(keys['e']?1:0)-(keys['q']?1:0); }

// ---- DualShock 4 via Web Gamepad API ----
// Focus-gated: with a sim tab and a robot tab open, the same physical pad is
// visible to both pages — only the focused, driving tab may act on it, so
// stick motion meant for the sim can never drive the armed real robot.
let prevBtn={},padWasActive=false;
function pollPad(){
  const gp=navigator.getGamepads&&navigator.getGamepads()[0];
  if(gp&&driving&&document.hasFocus()){
   padWasActive=true;
  drive.vx=round(gp.axes[0]||0); drive.vy=round(-(gp.axes[1]||0));
  drive.wz=round(gp.axes[2]||0);
  const b=i=>gp.buttons[i]&&gp.buttons[i].pressed;
  const trigger=i=>gp.buttons[i]?Math.max(0,Math.min(1,+gp.buttons[i].value||0)):0;
  const edge=(i,a)=>{if(b(i)&&!prevBtn[i])press(a); prevBtn[i]=b(i);};
  edge(0,'arm'); edge(1,'disarm'); edge(9,'estop');
   const hatUp=b(12),hatDn=b(13); drive.body_z=(hatUp?20:0)+(hatDn?-20:0);
   drive.sway=round(trigger(7)-trigger(6));  // R2 right, L2 left (standard mapping)
   drive.pitch=(b(5)?1:0)-(b(4)?1:0);       // R1 nose-up, L1 nose-down
  }else if(padWasActive){
   drive.vx=drive.vy=drive.wz=drive.body_z=drive.sway=drive.pitch=0;
   padWasActive=false;
  }
 requestAnimationFrame(pollPad);
}
requestAnimationFrame(pollPad);

// ---- telemetry + render ----
function render(s){
 const t=s.telemetry;
 const ap=document.getElementById('armpill'); ap.textContent=t.armed?'ARMED':'DISARMED'; ap.className='pill '+(t.armed?'on':'off');
 document.getElementById('arm').disabled=!!s.guided_check_busy||!!s.deployment_hold;
 document.getElementById('release-label').textContent=s.deployment_hold?'Update in progress — ARM locked':(s.release_id?'Robot update '+s.release_id.slice(0,10):'');
 const ep=document.getElementById('estoppill'); ep.textContent=t.estop?'E-STOP LATCHED':'E-STOP CLEAR'; ep.className='pill '+(t.estop?'bad':'off');
 const wp=document.getElementById('wdpill'); wp.textContent=t.watchdog_tripped?'STALE→STAND':'intent'; wp.className='pill '+(t.watchdog_tripped?'warn':'off');
 driving=(myId!==null&&s.driver_id===myId);
 const dv=document.getElementById('drivepill'); dv.textContent=driving?'CONTROL HERE':'VIEW ONLY'; dv.className='pill '+(driving?'on':'off');
 document.getElementById('takectl').style.display=driving?'none':'';
 document.getElementById('tel').innerHTML=
  `<tr><td class=k>link</td><td>${s.sim?'SIM (MuJoCo)':(s.dry?'DRY (no hardware)':s.link)}</td><td class=k>phase</td><td>${t.phase}</td></tr>`+
  `<tr><td class=k>soft_start</td><td>${t.soft_start}</td><td class=k>hz</td><td>${s.hz}</td></tr>`+
  `<tr><td class=k>vx</td><td>${t.cmd.vx}</td><td class=k>vy</td><td>${t.cmd.vy}</td></tr>`+
  `<tr><td class=k>wz</td><td>${t.cmd.wz}</td><td class=k>body_z</td><td>${t.cmd.body_z}</td></tr>`+
  `<tr><td class=k>stand sway</td><td>${t.cmd.sway_mm} mm</td><td class=k>sway limit</td><td>${(+t.cmd.sway_max_mm).toFixed(0)} mm</td></tr>`+
  `<tr><td class=k>stand pitch</td><td>${t.cmd.pitch_deg}°</td><td class=k>pitch limit</td><td>${(+t.cmd.pitch_max_deg).toFixed(1)}°</td></tr>`+
  (t.torque&&t.torque.total?`<tr><td class=k>torque</td><td colspan=3 style="color:${t.torque.acked>=t.torque.total?'#4ade80':'#ef4444'}">${t.torque.acked}/${t.torque.total} servos holding${t.torque.acked>=t.torque.total?'':' -- ARMED BUT LIMP: check the servo rail / battery'}</td></tr>`:'')+
  (s.imu?(s.imu.ok?`<tr><td class=k>imu r/p/y</td><td colspan=3>${(+s.imu.roll_deg).toFixed(1)} / ${(+s.imu.pitch_deg).toFixed(1)} / ${(+s.imu.yaw_deg).toFixed(1)} deg</td></tr>`:`<tr><td class=k>imu</td><td colspan=3 style="color:#f59e0b">${s.imu.error||'no data'}</td></tr>`):'')+
  (s.loop_ms?`<tr><td class=k>tick p99</td><td>${s.loop_ms.p99_ms} / ${s.loop_ms.budget_ms} ms</td><td class=k>overruns</td><td>${s.loop_ms.overruns}</td></tr>`:'');
  renderCam(s); renderLidar(s); renderSfx(s); renderRobot(s);
  if(window.GuidedTunerUI) window.GuidedTunerUI.observe(s);
 const bn=document.getElementById('banner');
 document.getElementById('simreset').style.display=s.sim?'':'none';
 if(s.link_lost){bn.textContent='LINK LOST — serial died; cut robot power';bn.style.background='#dc2626';}
 else if(s.guided_check_busy){bn.textContent='Checking guided trajectories — ARM available when finished';bn.style.background='#1d4ed8';}
 else if(s.sim){
  if(s.sim.fell){bn.textContent='SIM FELL — hit RESET SIM';bn.style.background='#dc2626';}
  else if(t.trick){bn.textContent='SIM TRICK: '+t.trick;bn.style.background='#0e7490';}
  else{bn.textContent=t.armed?'SIM DRIVE — physics (WASD to trot)':'SIM — ARM to trot';
   bn.style.background=t.armed?'#7c3aed':'#374151';}
 }
 else{bn.textContent=s.dry?'DRY MODE — no servo motion':(t.estop?'E-STOP LATCHED':(t.armed?(t.trick?'TRICK: '+t.trick:(t.pose?'HOLDING '+t.pose:'ARMED')):'disarmed'));
  bn.style.background=t.estop?'#dc2626':(t.armed?(t.trick?'#0e7490':'#1f9d55'):'#374151');}
 const ma=document.getElementById('mauto');
 document.getElementById('mfluid').disabled=!s.profiles?.['mode-fluid'];
 document.getElementById('mfast').disabled=!s.profiles?.['mode-fast'];
 ma.disabled=!s.auto_mode&&!(s.profiles?.['mode-fluid']&&s.profiles?.['mode-fast']);
 ma.textContent='AUTO '+(s.auto_mode?('on'+(s.mode?' · '+s.mode.replace('mode-',''):'')):'off');
 ma.style.background=s.auto_mode?'#b45309':'#374151';
 document.getElementById('mfluid').style.outline=s.active_profile==='mode-fluid'||s.mode==='mode-fluid'?'2px solid #e6e6e6':'';
 document.getElementById('mfast').style.outline=s.active_profile==='mode-fast'||s.mode==='mode-fast'?'2px solid #e6e6e6':'';
 const dp=document.getElementById('dirty');
 dp.textContent=s.config_changed_on_disk?'config changed on disk':(s.gait_dirty?'unsaved':'saved');
 dp.className='pill '+(s.gait_dirty||s.config_changed_on_disk?'warn':'off');
 if(!builtSliders&&s.gait_meta){buildSliders(s.gait_meta,s.gait_seed);buildPresets(s.presets);abFill(s.gait_meta);}
 fillPoses(s.poses);
 fillTricks(s.tricks);
 fillProfiles(s.profiles,s.active_profile,s.gait_dirty);
 if(s.sim){draw({legs:s.sim.legs});followCam(s.sim);}else draw(s.pose);
 // IMU hand-tilt bench test: the wireframe tilts with the real body. Robot
 // frame X lateral / Y fwd / Z up maps to three (x, z->y, -y->z), so pitch
 // rotates about three-x and roll about three-z. Signs are confirmed against
 // the physical sensor during D4 milestone-2 bring-up.
 if(legGroup){ const im=(!s.sim&&s.imu&&s.imu.ok)?s.imu:null;
  legGroup.rotation.x=im?(+im.pitch_deg)*Math.PI/180:0;
  legGroup.rotation.z=im?-(+im.roll_deg)*Math.PI/180:0; }
}
// ---- camera + D3 robot-link panels ----
// The MJPEG <img> only holds a live stream while its panel is open, so a
// closed panel costs zero WiFi bandwidth on the robot link.
function renderCam(s){
 const w=document.getElementById('camwrap'), im=document.getElementById('cam');
 if(!s.camera_url||s.sim){w.style.display='none';return;}
 w.style.display='';
 if(!w.dataset.bound){w.dataset.bound='1';w.addEventListener('toggle',()=>{im.src=w.open?latest.camera_url:'';});}
 if(w.open&&!im.src)im.src=s.camera_url;
}
// Lidar rides in an iframe here rather than its own tab on purpose: pad
// polling is gated on document.hasFocus(), so a second tab stealing focus
// stalls intent -> hold stand at 300 ms -> disarm at 10 s. Same lazy-load
// rule as the camera: the viewer's WebSocket only runs while the panel is open.
function renderLidar(s){
 const w=document.getElementById('lidarwrap'), fr=document.getElementById('lidarframe');
 if(!s.lidar_url){w.style.display='none';return;}
 w.style.display='';
 if(!w.dataset.bound){w.dataset.bound='1';w.addEventListener('toggle',()=>{fr.src=w.open?latest.lidar_url:'about:blank';});}
 if(w.open&&(!fr.src||fr.src==='about:blank'))fr.src=s.lidar_url;
}
// Sound: proxied through this server (the audio service has no CORS headers).
// Clip list is fetched once, on first open, so a disabled/absent audio box
// costs nothing until you actually look for it.
function sfxErr(m){document.getElementById('sfxerr').textContent=m||'';}
async function sfxPlay(n){const r=await J('/api/audio/play','POST',{name:n}); sfxErr(r&&r.error?('audio: '+r.error):'');}
async function saySpeak(){
  const t=document.getElementById('saytext').value.trim(); if(!t)return;
  const voice=document.getElementById('sayvoice').value||'en-us';
  const volume=+document.getElementById('sayvolume').value;
  const rate_wpm=+document.getElementById('sayrate').value;
  const r=await J('/api/audio/say','POST',{text:t,voice,volume,rate_wpm}); sfxErr(r&&r.error?('audio: '+r.error):'');
}
async function loadClips(){
 const box=document.getElementById('sfxbtns'); box.textContent='';
 const [r,vr]=await Promise.all([J('/api/audio/clips'),J('/api/audio/voices')]);
 if(!r||!r.clips){sfxErr('audio service unreachable'+(r&&r.error?(' — '+r.error):''));return;}
 sfxErr('');
 for(const n of r.clips){
   const b=document.createElement('button'); b.textContent=n; b.style.background='#374151';
   b.style.fontSize='12px'; b.style.padding='4px 8px'; b.onclick=()=>sfxPlay(n); box.appendChild(b);
 }
 const vs=document.getElementById('sayvoice'),selected=vs.value; vs.textContent='';
  const voices=(vr&&vr.voices)||[{id:'en-us',label:'American'}];
  const voiceDefault=(vr&&vr.default)||'en-us';
  for(const v of voices){const o=document.createElement('option');o.value=v.id;o.textContent=v.label;o.selected=v.id===(selected||voiceDefault);vs.appendChild(o);}
  const vol=document.getElementById('sayvolume'),rate=document.getElementById('sayrate');
  if(!vol.dataset.init){
   vol.value=(vr&&vr.default_volume)??80; rate.min=(vr&&vr.min_rate_wpm)??80;
   rate.max=(vr&&vr.max_rate_wpm)??220; rate.value=(vr&&vr.default_rate_wpm)??90;
   document.getElementById('sayvolout').textContent=vol.value+'%';
   document.getElementById('sayrateout').textContent=rate.value+' WPM'; vol.dataset.init='1';
  }
}
function renderSfx(s){
 const w=document.getElementById('sfxwrap');
 if(!s.audio_enabled){w.style.display='none';return;}
 w.style.display='';
 if(!w.dataset.bound){w.dataset.bound='1';w.addEventListener('toggle',()=>{if(w.open)loadClips();});}
}
let rlProfKey='';
function renderRobot(s){
 const box=document.getElementById('robotlink');
 if(!s.robot){box.style.display='none';return;}
 box.style.display=''; const r=s.robot;
 const cb=document.getElementById('rlcomms');
 cb.textContent='COMMS '+(r.comms_on?'on':'off'); cb.style.background=r.comms_on?'#0e7490':'#374151';
 if(!cb.dataset.bound){cb.dataset.bound='1';cb.onclick=()=>J('/api/robot/comms','POST',{on:!(latest.robot&&latest.robot.comms_on)});}
 const rp=document.getElementById('rlreach');
 rp.textContent=!r.comms_on?'no comms':(r.reachable?'ROBOT LINKED':'UNREACHABLE');
 rp.className='pill '+(!r.comms_on?'off':(r.reachable?'on':'bad'));
 const ra=document.getElementById('rlarm');
 ra.textContent=!r.reachable?'-':(r.estop?'ROBOT E-STOP':(r.armed?'ROBOT ARMED':'robot disarmed'));
 ra.className='pill '+(!r.reachable?'off':(r.estop?'bad':(r.armed?'warn':'off')));
 const names=Object.keys(s.profiles||{}); const k=names.join(',');
 if(k!==rlProfKey){rlProfKey=k;document.getElementById('rlprof').innerHTML=names.map(n=>'<option>'+n+'</option>').join('');}
 const pb=document.getElementById('rlpush');
 if(!pb.dataset.bound){pb.dataset.bound='1';pb.onclick=async()=>{const n=document.getElementById('rlprof').value;if(!n)return;
  pb.disabled=true;const resp=await J('/api/robot/push','POST',{name:n});pb.disabled=false;
  pb.textContent=resp.ok?'Sent: '+n:'Failed';setTimeout(()=>pb.textContent='Send to robot',2500);};}
 const oa=document.getElementById('rlopen'); oa.href=r.url+(KEY?'/?key='+KEY:'');
 const rw=document.getElementById('rlcamwrap'), ri=document.getElementById('rlcam');
 if(!rw.dataset.bound){rw.dataset.bound='1';rw.addEventListener('toggle',()=>{ri.src=(rw.open&&latest.robot&&latest.robot.comms_on&&latest.camera_url)?latest.camera_url:'';});}
 if(!r.comms_on&&ri.src)ri.src='';
 if(rw.open&&r.comms_on&&s.camera_url&&!ri.src)ri.src=s.camera_url;
}
let presetsBuilt=false,poseKey='';
function buildPresets(names){ if(presetsBuilt||!names)return; presetsBuilt=true;
 const c=document.getElementById('presets');
 for(const n of names){const b=document.createElement('button');b.textContent=n;b.style.cssText='background:#374151;flex:1;min-width:70px';b.dataset.p=n;b.onclick=()=>applyPreset(b.dataset.p);c.appendChild(b);} }
function fillPoses(poses){ const k=(poses||[]).join(','); if(k===poseKey)return; poseKey=k;
 const s=document.getElementById('poselist'); s.innerHTML=(poses||[]).map(p=>'<option>'+p+'</option>').join('')||'<option value="">(none)</option>'; }
async function applyPreset(n){ const r=await J('/api/gait/preset','POST',{name:n}); builtSliders=false; latest&&(latest.gait_seed=r.gait_seed); }
async function saveGait(){ await J('/api/gait/save','POST',{}); }
async function applyPose(){ const n=document.getElementById('poselist').value; if(n) J('/api/pose/apply','POST',{name:n}); }
function clearPose(){ J('/api/pose/clear','POST',{}); }
async function savePose(){ const n=document.getElementById('posename').value.trim(); if(!n)return; await J('/api/pose/save','POST',{name:n}); document.getElementById('posename').value=''; poseKey=''; }

// ---- tricks: run saved pose sequences + a minimal step builder ----
let trickKey='', tsteps=[];
function fillTricks(tricks){ const k=(tricks||[]).join(','); if(k===trickKey)return; trickKey=k;
 const s=document.getElementById('tricklist');
 s.innerHTML=(tricks||[]).map(t=>'<option>'+t+'</option>').join('')||'<option value="">(none)</option>'; }
function runTrick(){ const n=document.getElementById('tricklist').value; if(n)J('/api/trick/run','POST',{name:n}); }
function stopTrick(){ J('/api/trick/stop','POST'); }
function addTrickStep(){ const p=(latest&&latest.poses&&latest.poses[0])||'';
 tsteps.push({pose:p,move_s:1.0,hold_s:0.5}); renderTrickSteps(); }
function renderTrickSteps(){
 const c=document.getElementById('trsteps'); c.innerHTML='';
 tsteps.forEach((st,i)=>{
  const d=document.createElement('div'); d.className='row';
  const sel=document.createElement('select'); sel.style.flex='1';
  sel.innerHTML=((latest&&latest.poses)||[]).map(p=>'<option'+(p===st.pose?' selected':'')+'>'+p+'</option>').join('');
  sel.onchange=()=>{st.pose=sel.value;};
  const mv=document.createElement('input'); mv.type='number'; mv.step='0.1'; mv.value=st.move_s;
  mv.style.cssText='width:58px;background:#1a1f27;color:#e6e6e6;border:1px solid #2a2f37;border-radius:5px;padding:4px';
  mv.onchange=()=>{st.move_s=parseFloat(mv.value)||1.0;};
  const hd=mv.cloneNode(); hd.value=st.hold_s; hd.onchange=()=>{st.hold_s=parseFloat(hd.value)||0;};
  const rm=document.createElement('button'); rm.textContent='x';
  rm.style.cssText='background:#4b1d1d;padding:4px 8px;font-size:11px'; rm.onclick=()=>{tsteps.splice(i,1);renderTrickSteps();};
  d.append(sel,mv,hd,rm); c.appendChild(d);
 });
}
async function saveTrick(){ const n=document.getElementById('trickname').value.trim();
 if(!n||!tsteps.length||tsteps.some(s=>!s.pose))return;
 await J('/api/trick/save','POST',{name:n,steps:tsteps});
 document.getElementById('trickname').value=''; tsteps=[]; renderTrickSteps(); trickKey=''; }

// ---- A/B one lever: flip a single parameter live and keep the winner ----
let ab={lever:null,A:0,B:0,on:'A'};
function abFill(meta){ const s=document.getElementById('ablever'); if(s.options.length)return;
 s.innerHTML=Object.keys(meta).map(k=>'<option>'+k+'</option>').join(''); s.onchange=abPrefill; abPrefill(); }
function abPrefill(){ if(!latest)return; const k=document.getElementById('ablever').value;
 const v=+latest.gait_seed[k], m=latest.gait_meta[k];
 document.getElementById('abA').value=v;
 document.getElementById('abB').value=Math.min(m.max,+(v+(m.max-m.min)*0.15).toFixed(2)); }
abgo.onclick=()=>{ ab.lever=document.getElementById('ablever').value;
 ab.A=parseFloat(document.getElementById('abA').value); ab.B=parseFloat(document.getElementById('abB').value);
 if(isNaN(ab.A)||isNaN(ab.B))return; ab.on='A'; abApply(); abtoggle.style.display=''; abkeep.style.display=''; };
abtoggle.onclick=()=>{ ab.on=ab.on==='A'?'B':'A'; abApply(); };
abkeep.onclick=()=>{ ab.lever=null; abtoggle.style.display='none'; abkeep.style.display='none'; };
function abApply(){ if(!ab.lever)return; const v=ab.on==='A'?ab.A:ab.B;
 if(ws&&ws.readyState===1) ws.send(JSON.stringify({type:'gait',levers:{[ab.lever]:v}}));
 builtSliders=false;  // re-render sliders so they show the active value
 abtoggle.textContent='Now '+ab.on+': '+ab.lever+' = '+v+'  (click to flip)'; }

// ---- gait profiles + digital-twin analyze ----
let profKey='';
function fillProfiles(profiles,active,dirty){
 const pp=document.getElementById('profpill');
 pp.textContent=active?(active+(dirty?' (edited)':'')):'no profile loaded';
 pp.className='pill '+(active?(dirty?'warn':'on'):'off');
 const f=(document.getElementById('proffilter').value||'').toLowerCase();
 const k=JSON.stringify([profiles,active,f]); if(k===profKey)return; profKey=k;
 const t=document.getElementById('proftable'); t.innerHTML='';
 const names=Object.keys(profiles||{}).filter(n=>!f||n.toLowerCase().includes(f));
 if(!names.length){t.innerHTML='<tr><td class=k>(none'+(f?' matching '+f:' saved yet')+')</td></tr>';return;}
 const hd=t.insertRow(); hd.innerHTML='<td class=k>name</td><td class=k>T</td><td class=k>step</td><td class=k>duty</td><td class=k>mm/s</td><td></td><td></td>';
 for(const n of names){const p=profiles[n]; const r=t.insertRow();
  const c0=r.insertCell(); c0.textContent=(p.gait_type==='crawl'?'c· ':'')+n; if(n===active){c0.style.color='#3b82f6';c0.style.fontWeight='700';}
  r.insertCell().textContent=(+p.cycle_period).toFixed(2); r.insertCell().textContent=(+p.step_length).toFixed(0);
  r.insertCell().textContent=(+p.duty_factor).toFixed(2); r.insertCell().textContent=(+p.speed_mms).toFixed(0);
  const lb=document.createElement('button'); lb.textContent='Load'; lb.style.cssText='background:#374151;padding:2px 7px;font-size:11px'; lb.onclick=()=>loadProfile(n); r.insertCell().appendChild(lb);
  const db=document.createElement('button'); db.textContent='x'; db.style.cssText='background:#4b1d1d;padding:2px 7px;font-size:11px'; db.onclick=()=>deleteProfile(n); r.insertCell().appendChild(db);
 }
}
async function saveProfile(){ const n=document.getElementById('profname').value.trim(); if(!n)return;
 await J('/api/profile/save','POST',{name:n}); document.getElementById('profname').value=''; }
async function loadProfile(n){ const r=await J('/api/profile/load','POST',{name:n}); if(r.gait_seed){builtSliders=false; latest&&(latest.gait_seed=r.gait_seed);} }
async function deleteProfile(n){ if(!confirm('Delete profile '+n+'?'))return; await J('/api/profile/delete','POST',{name:n}); }
async function analyze(){ const r=await J('/api/gait/analyze','POST',{}); if(!r.analysis)return; const a=r.analysis;
 document.getElementById('antable').innerHTML=
  '<tr><td class=k>body speed</td><td>'+a.speed_mms+' mm/s</td><td class=k>stride</td><td>'+a.stride_mm+' mm</td></tr>'+
  '<tr><td class=k>cadence</td><td>'+a.cadence_hz+' Hz</td><td class=k>swing</td><td>'+a.swing_time_s+' s</td></tr>'+
   '<tr><td class=k>peak joint</td><td>'+a.peak_joint_dps+' deg/s</td><td class=k>ceiling</td><td>'+a.cmd_ceiling_dps+' deg/s</td></tr>'+
   '<tr><td class=k>peak acceleration</td><td>'+a.peak_joint_accel_dps2+' deg/s²</td><td class=k>modeled ceiling</td><td>'+a.cmd_accel_ceiling_dps2+' deg/s²</td></tr>'+
  '<tr><td class=k>saturated</td><td>'+a.saturation_pct+'%</td><td class=k>foot peak</td><td>'+a.peak_foot_speed_mms+' mm/s</td></tr>';
 document.getElementById('anwarn').innerHTML=(a.warnings||[]).map(w=>'<p class=hint style="color:#f59e0b">! '+w+'</p>').join('')
   ||'<p class=hint style="color:#1f9d55">No sampled trajectory warnings. Physical tracking and balance remain unverified.</p>';
}

// ---- three.js skeleton ----
let scene,cam,renderer,legGroup;
function init3d(){
 const v=document.getElementById('view');
 if(typeof THREE==='undefined'){v.insertAdjacentHTML('beforeend','<div style="padding:60px 20px;color:#7e8794">3D viewer offline (CDN unreachable) — all controls still work.</div>');return;}
 scene=new THREE.Scene();
 cam=new THREE.PerspectiveCamera(55,v.clientWidth/v.clientHeight,1,12000); cam.position.set(420,330,520); cam.lookAt(0,-120,0);
 renderer=new THREE.WebGLRenderer({antialias:true}); renderer.setSize(v.clientWidth,v.clientHeight); v.appendChild(renderer.domElement);
 scene.add(new THREE.GridHelper(4000,40,0x2a2f37,0x1a1f27));
 scene.add(new THREE.AmbientLight(0xffffff,0.9));
 legGroup=new THREE.Group(); scene.add(legGroup);
 addEventListener('resize',()=>{cam.aspect=v.clientWidth/v.clientHeight;cam.updateProjectionMatrix();renderer.setSize(v.clientWidth,v.clientHeight);});
 (function loop(){requestAnimationFrame(loop);renderer.render(scene,cam);})();
}
// robot frame (X lateral, Y forward, Z up) -> three (x=X, y=Z, z=-Y)
const TP=p=>new THREE.Vector3(p[0],p[2],-p[1]);
function draw(pose){
 if(!pose||!pose.legs||!legGroup)return; while(legGroup.children.length)legGroup.remove(legGroup.children[0]);
 const body=[]; for(const leg in pose.legs){const pts=pose.legs[leg].points; const stance=pose.legs[leg].in_stance;
  body.push(pts[0]);
  const col=stance?0x3b82f6:0xf59e0b;
  const g=new THREE.BufferGeometry().setFromPoints(pts.map(TP));
  legGroup.add(new THREE.Line(g,new THREE.LineBasicMaterial({color:col,linewidth:2})));
  const foot=TP(pts[3]); const fm=new THREE.Mesh(new THREE.SphereGeometry(9),new THREE.MeshBasicMaterial({color:col})); fm.position.copy(foot); legGroup.add(fm);
 }
 if(body.length===4){const order=[0,1,3,2,0].map(i=>TP(body[i]));
  legGroup.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(order),new THREE.LineBasicMaterial({color:0x9aa4b2})));}
}
// ---- sim mode: side-follow camera + obstacle ----
// Treadmill view: camera rides beside the torso looking straight across (-X),
// so the robot stays centered and travels left-to-right through world space.
let obsMesh=null;
function followCam(sim){
 if(!cam||!sim.body_mm)return;
 const b=TP(sim.body_mm);
 cam.position.set(b.x+620,b.y+170,b.z); cam.lookAt(b);
 if(sim.obstacle_mm&&!obsMesh&&typeof THREE!=='undefined'){
  obsMesh=new THREE.Mesh(new THREE.BoxGeometry(800,sim.obstacle_mm,100),
   new THREE.MeshBasicMaterial({color:0x8a5a1a,wireframe:true}));
  obsMesh.position.set(0,sim.obstacle_mm/2,-450); scene.add(obsMesh);
 }
}
init3d(); connect();
</script><script src="/static/guided-tuner.js?v=2&key=__KEY__"></script></body></html>"""


def _open_browser_soon(url: str) -> None:
    import os
    import subprocess
    import threading
    import webbrowser

    def _open() -> None:
        # Prefer a dedicated app-mode window pinned to the primary monitor. A
        # plain webbrowser.open() restores the browser's last window position,
        # which can land the GUI half-off a secondary monitor.
        for exe in (
            r"C:\Program Files\Google\Chrome\Application\chrome.exe",
            r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe",
            os.path.expandvars(r"%LOCALAPPDATA%\Google\Chrome\Application\chrome.exe"),
            r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
            r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
        ):
            if os.path.exists(exe):
                subprocess.Popen([exe, f"--app={url}",
                                  "--window-position=40,40", "--window-size=1520,960"])
                return
        webbrowser.open(url)

    threading.Timer(1.2, _open).start()


def run(config_path: str, *, port_serial: str | None, tcp: str | None,
        host: str, port: int, hz: float, dry: bool, sim: bool = False,
        params_path: str = "simulation_params.json", obstacle_mm: float | None = None,
        open_browser: bool = True, token: str | None = None,
        skip_firmware_check: bool = False) -> int:
    if FastAPI is None:
        print("FastAPI/uvicorn not installed; cannot run the operator GUI.")
        return 1

    transport = None if (dry or sim) else make_transport(port_serial, tcp)
    if not dry and not sim and transport is None:
        print("No link: pass --port-serial COMx, --tcp HOST[:PORT], --sim for physics, or --dry for UI preview.")
        return 1

    # Enforce commissioning exactly when we can actually move servos.
    try:
        config = load_config(config_path, require_commissioned=transport is not None)
    except ConfigError as e:
        print(f"Refusing to operate: {e}")
        return 1

    # Identity check before the loop streams a single servo frame: the ESP32
    # and the RPLIDAR C1 can both be CP2102 ttys, so a swapped udev name must
    # fail fast here rather than SyncWrite at a lidar.
    if transport is not None and not skip_firmware_check:
        ok, seen = verify_firmware_link(transport, config.firmware_expected)
        if not ok:
            print(f"Refusing to operate: no {config.firmware_expected!r} answer on "
                  f"{port_serial or tcp} (saw {seen[:120]!r}). Wrong device or firmware; "
                  f"--skip-firmware-check overrides.")
            return 1

    # Shared key: mandatory off-loopback (an unauthenticated 0.0.0.0 bind
    # would let any LAN webpage POST an ARM cross-origin).
    resolved_token = token if token is not None else config.network.token
    if host not in ("127.0.0.1", "localhost", "::1") and not resolved_token:
        resolved_token = secrets.token_urlsafe(9)
        print(f"  No network.token in config: generated session key {resolved_token}")

    twin = None
    if sim:
        try:
            from ..simulation.dynamics_params import load_dynamics_params
            from ..simulation.live import LiveTwin
            twin = LiveTwin(config, load_dynamics_params(params_path), obstacle_mm=obstacle_mm)
        except (OSError, RuntimeError, ValueError) as e:
            print(f"Cannot start physics sim: {e}")
            return 1

    # BNO085 at body center on the Pi's I2C: read off-loop, surfaced in
    # telemetry. Also started in --dry sessions so the sensor is bench-testable
    # on the Pi with no ESP32 attached (sim stays synthetic-only). A missing
    # sensor/library never crashes: the reader retries and reports its error.
    imu = None
    if not sim and config.peripherals.imu.enabled:
        from ..pi.imu import ImuReader
        spec = config.peripherals.imu
        imu = ImuReader(i2c_bus=spec.i2c_bus, i2c_address=spec.i2c_address,
                        rate_hz=spec.rate_hz)
        imu.start()

    # D3: sim sessions get a read-only/save-only link to the onboard server.
    robot_link = RobotLink(config.network.operate_url(), config.network.token) if sim else None

    # The real ESP32 -> CP2102 -> Pi path occasionally lands just beyond the
    # generic 50 ms desktop default, especially for feedback reads after the
    # 60 Hz stream has been running.  D1's measured-good hardware bench already
    # uses 100 ms; use the same budget for the onboard runtime.  SyncWrites stay
    # no-reply and therefore retain their normal 60 Hz cost.
    driver = FeetechDriver(transport, reply_timeout=0.1) if transport is not None else None
    controller = RobotController(config, driver)
    if sim:
        # No hardware at stake, and a backgrounded browser throttles the 20 Hz
        # intent heartbeat to ~1 Hz — keep the hold-stand watchdog but at a
        # tab-throttle-proof threshold, and never auto-disarm the sim.
        controller.stale_hold_timeout = 2.5
        controller.stale_disarm_timeout = 1e9
    loop = ControlLoop(controller, hz=hz, twin=twin, imu=imu)
    link = "SIM (MuJoCo physics)" if sim else (tcp or port_serial)
    app = build_app(loop, link=link, dry=transport is None and not sim, config_path=Path(config_path),
                    token=resolved_token or "", imu=imu, robot_link=robot_link)

    loop.start()
    view_host = "127.0.0.1" if host in ("0.0.0.0", "::") else host
    url = f"http://{view_host}:{port}" + (f"/?key={resolved_token}" if resolved_token else "")
    print(f"\n  DogV3 Operate (D2)\n  Open {url}\n  link={'DRY (no hardware)' if transport is None else link}\n")
    if open_browser:
        _open_browser_soon(url)
    try:
        uvicorn.run(app, host=host, port=port)
    finally:
        loop.stop()
        if imu is not None:
            imu.stop()
        if robot_link is not None:
            robot_link.stop()
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="DogV3 D2 operator GUI: live pose, gait tuning, ARM/E-STOP, telemetry."
    )
    ap.add_argument("--port-serial", default=None, help="ESP32 serial port, e.g. COM5")
    ap.add_argument("--tcp", default=None, help="ESP32 WiFi bridge HOST[:PORT] (e.g. 192.168.4.1:3333)")
    ap.add_argument("--config", default="robot_config.json")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8001)
    ap.add_argument("--hz", type=float, default=CONTROL_HZ_DEFAULT)
    ap.add_argument("--dry", action="store_true", help="UI preview without hardware (no servo motion)")
    ap.add_argument("--sim", action="store_true",
                    help="drive the MuJoCo physics twin instead of hardware (offline gait tuning)")
    ap.add_argument("--params", default="simulation_params.json",
                    help="dynamics params for --sim (default simulation_params.json)")
    ap.add_argument("--obstacle", type=float, default=None, metavar="MM",
                    help="--sim only: full-width step of this height (mm) across the path")
    ap.add_argument("--no-browser", action="store_true", help="do not auto-open the browser")
    ap.add_argument("--token", default=None,
                    help="shared key for non-loopback binds (default: network.token from the "
                         "config, else auto-generated and printed)")
    ap.add_argument("--skip-firmware-check", action="store_true",
                    help="skip the startup VERSION handshake against firmware_expected")
    args = ap.parse_args(argv)
    if not args.dry and not args.sim and not args.port_serial and not args.tcp:
        ap.error("give --port-serial COMx / --tcp HOST[:PORT], --sim for physics, or --dry for UI preview")
    return run(
        args.config,
        port_serial=args.port_serial,
        tcp=args.tcp,
        host=args.host,
        port=args.port,
        hz=args.hz,
        dry=args.dry,
        sim=args.sim,
        params_path=args.params,
        obstacle_mm=args.obstacle,
        open_browser=not args.no_browser,
        token=args.token,
        skip_firmware_check=args.skip_firmware_check,
    )


if __name__ == "__main__":
    raise SystemExit(main())
