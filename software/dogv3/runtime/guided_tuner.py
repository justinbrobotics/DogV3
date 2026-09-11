"""Bounded, evidence-led trot experiments. No hardware writes or autonomous drive.

Trajectory screening runs outside the control thread. TrialMonitor only observes
the existing loop: it never polls a servo or refreshes the command watchdog.
Sessions live beside the configuration, separately for physical, dry and sim use.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import threading
import time
import uuid
from contextlib import contextmanager
from pathlib import Path
from typing import Literal

from pydantic import BaseModel, Field

from ..config.schema import GaitSeed, RobotConfig
from ..gait.trot import GaitCommand
from ..simulation.twin import CMD_ACCEL_CEILING_DPS2, CMD_CEILING_DPS, evaluate_gait

SCREEN_VERSION = 2
ROUND_LIMIT = 12
FORWARD_INTENT = 0.5


def fingerprint(value: dict) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def context_key(config: RobotConfig, hz: float) -> str:
    data = config.model_dump()
    # Exclude profiles, boot gait, and network credentials from the trial context.
    return fingerprint({"screen_version": SCREEN_VERSION, "hz": hz,
        "command_model": [CMD_CEILING_DPS, CMD_ACCEL_CEILING_DPS2], **{
        k: data[k] for k in ("frame", "links_mm", "body_mm", "legs", "servos", "stabilizer")
    }})


def screen_trajectory(config: RobotConfig, seed: GaitSeed, hz: float, *, sample_hz=0) -> dict:
    """Screen straight, steady commands, including near-idle and both directions.

    Fine sampling reduces acceleration aliasing at swing boundaries. Explicit
    endpoint checks reject non-smooth paths even if finite samples miss the jump.
    This is a software rejection screen, not a loaded-servo or balance model.
    """
    blockers = []
    values = seed.model_dump()
    if any(isinstance(v, (int, float)) and not math.isfinite(v) for v in values.values()):
        return {"passed": False, "blockers": ["Non-finite gait setting."], "metrics": {}}
    crawl = seed.gait_type == "crawl"
    low, high = (.78, .88) if crawl else (.60, .80)
    if not low <= seed.duty_factor <= high:
        blockers.append(f"Stance fraction must be between {low:.0%} and {high:.0%} for this {seed.gait_type} screen.")
    if not 0.3 <= seed.cycle_period <= 3.0 or not 0 < seed.step_length <= 80:
        blockers.append("Cycle time or stride is outside the operator's usable range.")
    if not 12 <= seed.step_height <= 30:
        blockers.append("Use 12–30 mm planned lift for the initial flat-floor search.")
    if seed.swing_shape != "cycloid" or seed.swing_retract != 1 or seed.stance_dip != 0:
        blockers.append("Use cycloid lift, full touchdown retraction and no stance dip for smooth endpoints.")
    if seed.cycle_period * (1 - seed.duty_factor) < 0.28 - 1e-9:
        blockers.append("Less than 0.28 s is available for swing; lengthen the cycle.")
    if not 0.3 <= seed.cycle_period <= 3.0:
        return {"passed": False, "blockers": blockers, "metrics": {}}

    rate = max(240.0, hz * 4, sample_hz)
    commands = [0.0, 0.1, 0.25, 0.5, 0.75, 1.0, -0.25, -0.5, -1.0]
    reports = [evaluate_gait(config, seed, GaitCommand(vy=v), hz=rate, cycles=1.5)
               for v in commands]
    worst_speed = max(r.peak_joint_dps for r in reports)
    worst_accel = max(r.peak_joint_accel_dps2 for r in reports)
    speed_limit = reports[0].cmd_ceiling_dps
    accel_limit = reports[0].cmd_accel_ceiling_dps2
    range_hits = sorted({s for r in reports for s in r.range_hit_pct})
    if worst_speed > speed_limit:
        blockers.append(f"Joint speed demand {worst_speed:.0f} exceeds the command ceiling {speed_limit:.0f} deg/s.")
    if worst_accel > accel_limit:
        blockers.append(f"Joint acceleration demand {worst_accel:.0f} exceeds the modeled ceiling {accel_limit:.0f} deg/s².")
    if range_hits:
        blockers.append("Commissioned joint range exceeded: " + ", ".join(range_hits))
    if max(r.max_reach_err_mm for r in reports) > 1:
        blockers.append("At least one foot target lies outside the configured leg reach.")
    return {
        "version": SCREEN_VERSION, "passed": not blockers, "blockers": blockers,
        "metrics": {"peak_joint_dps": round(worst_speed, 1),
                    "peak_joint_accel_dps2": round(worst_accel, 1),
                    "speed_ceiling_dps": round(speed_limit, 1),
                    "accel_ceiling_dps2": round(accel_limit, 1),
                    "swing_time_s": round(seed.cycle_period * (1 - seed.duty_factor), 3),
                    "four_foot_pct": round(100 * max(0, (4 * seed.duty_factor - 3) if crawl
                                                     else (2 * seed.duty_factor - 1)), 1),
                    "nominal_full_stick_mms": round(min(seed.step_length,
                        seed.max_fwd_speed * seed.duty_factor * seed.cycle_period)
                        / (seed.duty_factor * seed.cycle_period), 1),
                    "nominal_half_stick_mms": round(min(seed.step_length * .5,
                        seed.max_fwd_speed * seed.duty_factor * seed.cycle_period)
                        / (seed.duty_factor * seed.cycle_period), 1)},
        "scope": "Steady straight motion, zero height trim; no turn or transition validation.",
        "limits": "Software command model only. Loaded tracking, foot contact, traction and balance require physical testing.",
        "sample_hz": rate,
    }


class Candidate(BaseModel):
    id: str
    name: str
    seed: GaitSeed
    why: str
    base_id: str = "reference"
    changes: dict = Field(default_factory=dict)
    screen: dict = Field(default_factory=dict)
    status: str = "proposed"
    family: str = ""
    profile_name: str = ""


class Trial(BaseModel):
    id: str
    candidate_id: str
    compared_with: str
    outcome: Literal["better", "same", "worse", "unclear"]
    issues: list[str] = Field(default_factory=list)
    note: str = ""
    distance_m: float | None = None
    travel_time_s: float | None = None
    measured_speed_mms: float | None = None
    observed: dict
    valid: bool
    invalid_reasons: list[str]
    recorded_at: float


class Session(BaseModel):
    id: str
    created_at: float
    context: str
    mode: Literal["physical", "dry", "simulation"]
    surface: str
    reference: Candidate
    best_id: str = "reference"
    candidates: list[Candidate] = Field(default_factory=list)
    trials: list[Trial] = Field(default_factory=list)
    exhausted: bool = False
    pending_recording: dict | None = None
    method: Literal["adaptive", "test_pack"] = "adaptive"
    best_by_gait: dict[str, str] = Field(default_factory=dict)

    def candidate(self, cid: str) -> Candidate:
        for c in [self.reference, *self.candidates]:
            if c.id == cid:
                return c
        raise ValueError("This experiment does not belong to the current session.")


class Journal(BaseModel):
    version: Literal[1] = 1
    sessions: list[Session] = Field(default_factory=list)


class GuidedTuner:
    def __init__(self, path: Path, mode: str):
        self.path, self.mode = path, mode
        self.lock = threading.RLock()
        self.error = None
        self.journal = Journal()
        if path.exists():
            try:
                self.journal = Journal.model_validate_json(path.read_text(encoding="utf-8"))
                if any(s.mode != mode for s in self.journal.sessions):
                    raise ValueError("Journal mode mismatch")
            except (ValueError, OSError):
                self.error = "The tuning journal cannot be read. Preserve it and repair it before recording new trials."

    @property
    def session(self) -> Session:
        if self.error:
            raise ValueError(self.error)
        if not self.journal.sessions:
            raise ValueError("Capture a reference to start a guided session.")
        return self.journal.sessions[-1]

    @contextmanager
    def transaction(self):
        with self.lock:
            if self.error:
                raise ValueError(self.error)
            previous = self.journal.model_copy(deep=True)
            try:
                yield
                self.path.parent.mkdir(parents=True, exist_ok=True)
                tmp = self.path.with_suffix(".tmp")
                tmp.write_text(self.journal.model_dump_json(indent=2), encoding="utf-8")
                tmp.replace(self.path)
            except Exception:
                self.journal = previous
                raise

    def begin(self, config: RobotConfig, seed: GaitSeed, hz: float, surface: str,
              name: str) -> None:
        if seed.gait_type != "trot":
            raise ValueError("Load a trot before capturing a reference.")
        reference = Candidate(id="reference", name=name, seed=seed.model_copy(deep=True),
                              why="Exact settings captured at the start of this session.",
                              screen=screen_trajectory(config, seed, hz), status="reference")
        self.journal.sessions.append(Session(id=uuid.uuid4().hex, created_at=time.time(),
            context=context_key(config, hz), mode=self.mode, surface=surface, reference=reference))

    def check_context(self, config: RobotConfig, hz: float):
        if self.session.context != context_key(config, hz):
            raise ValueError("Robot geometry, calibration or control settings changed. Start a new reference session.")

    def begin_pack(self, config: RobotConfig, seed: GaitSeed, hz: float, surface: str, name: str):
        from .gait_library import build_test_pack
        candidates = build_test_pack(config, seed, hz, use_saved=True)
        self.begin(config, seed, hz, surface, name)
        self.session.method = "test_pack"
        self.session.candidates = candidates
        self.session.best_by_gait = {"trot": "reference", "crawl": "deliberate-low"}

    def propose(self, config: RobotConfig, hz: float) -> Candidate | None:
        s = self.session
        if s.method == "test_pack":
            raise ValueError("Choose a gait from the fixed test set.")
        self.check_context(config, hz)
        pending = next((c for c in reversed(s.candidates) if c.status == "proposed"), None)
        if pending:
            return pending
        if len(s.candidates) >= ROUND_LIMIT:
            s.exhausted = True
            return None
        best = s.candidate(s.best_id)
        base = best.seed.model_dump()
        seen = {fingerprint(c.seed.model_dump()) for c in [s.reference, *s.candidates]}
        options = self._options(best)
        for name, why, updates in options:
            values = {**base, **updates}
            if fingerprint(values) in seen:
                continue
            seed = GaitSeed.model_validate(values)
            if fingerprint(seed.model_dump()) in seen:
                continue
            screen = screen_trajectory(config, seed, hz)
            # A timing repair is computed against the actual commissioned robot.
            if name == "Give the feet time to follow":
                for _ in range(8):
                    if screen["passed"] or seed.cycle_period >= 1.8:
                        break
                    seed.cycle_period = round(min(1.8, seed.cycle_period * 1.12), 3)
                    screen = screen_trajectory(config, seed, hz)
                if fingerprint(seed.model_dump()) in seen:
                    continue
            candidate = Candidate(id=f"trial-{len(s.candidates) + 1:02}", name=name, seed=seed,
                why=why, base_id=best.id, changes={k: {"before": base[k], "after": v}
                    for k, v in seed.model_dump().items() if v != base[k]}, screen=screen,
                status="proposed" if screen["passed"] else "screened_out")
            s.candidates.append(candidate)
            if screen["passed"]:
                return candidate
            if len(s.candidates) >= ROUND_LIMIT:
                break
        s.exhausted = True
        return None

    def _options(self, best: Candidate):
        b = best.seed
        ref = self.session.reference.seed
        options = []
        foundation = {}
        if not best.screen.get("passed"):
            options.append(("Give the feet time to follow",
                "Use smooth lift and touchdown, then lengthen the cycle until the trajectory screen passes. Body height and trim stay fixed.",
                {"swing_shape": "cycloid", "swing_retract": 1., "stance_dip": 0.,
                 "step_height": min(22., max(12., b.step_height)),
                 "step_length": min(40., max(20., b.step_length)),
                 "duty_factor": min(.75, max(.65, b.duty_factor)),
                 "cycle_period": min(1.8, max(1.0, b.cycle_period)),
                 "accel_limit": min(2., b.accel_limit)}))
            repaired = next((c for c in self.session.candidates
                if c.base_id == best.id and c.name == "Give the feet time to follow"
                and c.screen.get("passed")), None)
            if repaired is not None:
                # A failed original may have no passing single-slider neighbors.
                # Continue from its screened timing repair without declaring that
                # unproven experiment the physical best.
                b = repaired.seed
                foundation = {k: v for k, v in b.model_dump().items()
                              if v != best.seed.model_dump()[k]}
        repair_count = len(options)
        options += [
            ("Shorter, quicker steps", "Compare rhythm at the same nominal travel speed. Stride and cycle time decrease together.",
             {"cycle_period": round(max(.8, b.cycle_period * .9), 3),
              "step_length": round(b.step_length * max(.8, b.cycle_period * .9) / b.cycle_period, 3)}),
            ("More time on four feet", "Increase support overlap while preserving swing time. The nominal travel speed decreases.",
             {"duty_factor": min(.78, round(b.duty_factor + .03, 3)),
              "cycle_period": round(b.cycle_period * (1-b.duty_factor) / (1-min(.78, round(b.duty_factor+.03, 3))), 3)}),
            ("A little less foot lift", "Reduce unnecessary vertical travel. Keep it only if the feet still clear the floor.",
             {"step_height": max(12., b.step_height-2)}),
            ("A little more foot clearance", "Test whether clearance, rather than servo lag, is causing dragging.",
             {"step_height": min(26., b.step_height+2)}),
            ("Stand a little taller", "Compare knee posture and rocking with up to 5 mm more height. Check this stance supported first.",
             {"body_height": min(185., ref.body_height+15, b.body_height+5)}),
            ("Stand a little lower", "Compare rocking with up to 5 mm less height. Check this stance supported first.",
             {"body_height": max(130., ref.body_height-15, b.body_height-5)}),
            ("Slightly wider stance", "Move each foot up to 4 mm outward and compare side-to-side rocking.",
             {"stance_width_offset": min(40., ref.stance_width_offset+8, b.stance_width_offset+4)}),
            ("Slightly narrower stance", "Move each foot up to 4 mm inward and compare hip effort and tracking.",
             {"stance_width_offset": max(0., ref.stance_width_offset-8, b.stance_width_offset-4)}),
            ("A little more progress per step", "Add 4 mm of stride at the same rhythm. Keep only a repeatable clean improvement.",
             {"step_length": min(50., ref.step_length+12, b.step_length+4)}),
            ("A little less reach per step", "Reduce stride by 4 mm to compare foot placement and tracking.",
             {"step_length": max(20., b.step_length-4)}),
            ("Gentler changes in drive input", "Slow joystick acceleration without changing the steady gait or servo register settings.",
             {"accel_limit": max(1., round(b.accel_limit * .75, 3))}),
        ]
        if foundation:
            options[repair_count:] = [(name, why + " Includes the earlier trajectory repair; compare against the original reference.",
                                      {**foundation, **updates})
                                     for name, why, updates in options[repair_count:]]
        return options

    def record(self, cid: str, outcome: str, issues: list[str], note: str,
               observed: dict, distance_m=None, travel_time_s=None) -> Trial:
        s = self.session
        c = s.candidate(cid)
        best_id = s.best_by_gait.get(c.seed.gait_type, s.best_id) if s.method == "test_pack" else s.best_id
        if len(s.trials) >= 60:
            raise ValueError("This session has 60 trials. Start a new session; this history remains saved.")
        if any(t.id == observed["id"] for t in s.trials):
            raise ValueError("This recording has already been saved.")
        invalid = list(observed["faults"])
        minimum = max(3., 3*c.seed.cycle_period)
        if observed["drive_seconds"] < minimum or observed.get("longest_steady_seconds", 0) < minimum:
            invalid.append(f"Not enough continuous steady forward drive was observed (at least three cycles and {minimum:g} s required).")
        if observed["off_target_seconds"] > max(.5, .2*observed["drive_seconds"]):
            invalid.append(f"Forward input differed too much from the {observed.get('forward_intent', .5):.0%} comparison target.")
        if s.method != "test_pack" and c.base_id != best_id and c.id != best_id:
            invalid.append("The comparison reference has changed; request a new experiment.")
        trial = Trial(id=observed["id"], candidate_id=cid, compared_with=best_id,
            outcome=outcome, issues=issues, note=note, observed=observed, valid=not invalid,
            invalid_reasons=invalid, recorded_at=time.time(), distance_m=distance_m,
            travel_time_s=travel_time_s,
            measured_speed_mms=round(1000*distance_m/travel_time_s, 1) if distance_m is not None else None)
        s.trials.append(trial)
        clean = lambda t: t.valid and not t.issues and t.outcome in ("better", "same")
        baseline_exists = any(t.candidate_id == best_id and clean(t) for t in s.trials)
        comparisons = [t for t in s.trials if t.candidate_id == cid and t.compared_with == best_id]
        repeated_win = len(comparisons) >= 2 and all(t.outcome == "better" and clean(t) for t in comparisons[-2:])
        if cid != best_id and baseline_exists and repeated_win:
            if s.method == "test_pack":
                s.best_by_gait[c.seed.gait_type] = cid
            if s.method != "test_pack" or c.seed.gait_type == "trot":
                s.best_id = cid
            c.status = "best"
        elif cid != best_id:
            c.status = "tested"
        return trial


class TrialMonitor:
    """Bounded aggregate of actual loop observations, not measured locomotion.

    Position feedback is intentionally not added here: the serial budget must
    be measured before additional blocking servo reads are enabled on the Pi.
    """
    def __init__(self):
        self.lock = threading.Lock()
        self.active = None
        self.completed = None

    def start(self, cid: str, seed: GaitSeed, loop_stats: dict | None = None, *, forward_intent=FORWARD_INTENT):
        with self.lock:
            if self.active or self.completed:
                raise ValueError("Finish or discard the existing recording first.")
            self.active = {"id": uuid.uuid4().hex, "candidate_id": cid,
                "seed": seed.model_dump(), "started_at": time.time(), "elapsed_s": 0.,
                "forward_intent": forward_intent,
                "drive_seconds": 0., "off_target_seconds": 0., "ticks": 0,
                "steady_segment_seconds": 0., "longest_steady_seconds": 0.,
                "faults": [], "imu_samples": 0, "peak_roll_deg": None, "peak_pitch_deg": None,
                "loop_at_start": loop_stats or {}}

    def observe(self, controller, dt: float, auto_mode: bool, link_lost: bool):
        with self.lock:
            r = self.active
            if r is None:
                return
            r["elapsed_s"] += max(0., min(dt, .5))
            r["ticks"] += 1
            fault = lambda text: r["faults"].append(text) if text not in r["faults"] else None
            if controller.gait.seed.model_dump() != r["seed"]:
                fault("Gait settings changed during this recording.")
            if auto_mode or controller.pose_hold or controller.trick:
                fault("AUTO, a pose or a trick interrupted the gait comparison.")
            if link_lost:
                fault("Robot link was lost.")
            if controller.state.estop:
                fault("E-stop occurred during this recording.")
            cmd = controller.cmd
            steady = False
            if abs(cmd.vx) > .03 or abs(cmd.wz) > .03 or abs(cmd.body_z) > .1:
                fault("Turning, sideways motion or height trim changed the straight trial.")
            if controller.state.watchdog_tripped:
                fault("Drive intent went stale during the trial.")
            if controller.state.armed and abs(cmd.vy) > .1:
                if controller.torque_total and controller.torque_acked < controller.torque_total:
                    fault("Not all servos acknowledged torque.")
                elif not controller.state.watchdog_tripped and controller.state.soft_start >= .99:
                    if cmd.vy > .1:
                        r["drive_seconds"] += min(dt, .1)
                        if abs(cmd.vy-r["forward_intent"]) > .15:
                            r["off_target_seconds"] += min(dt, .1)
                        else:
                            steady = True
                    else:
                        fault("Reverse motion changed the forward comparison.")
            r["steady_segment_seconds"] = r["steady_segment_seconds"] + min(dt, .1) if steady else 0.
            r["longest_steady_seconds"] = max(r["longest_steady_seconds"], r["steady_segment_seconds"])
            if controller.attitude is not None:
                roll, pitch = controller.attitude
                if math.isfinite(roll) and math.isfinite(pitch):
                    r["imu_samples"] += 1
                    r["peak_roll_deg"] = max(r["peak_roll_deg"] or 0., abs(roll))
                    r["peak_pitch_deg"] = max(r["peak_pitch_deg"] or 0., abs(pitch))

    def finish(self, *, link_lost=False, loop_stats: dict | None = None):
        with self.lock:
            if self.completed is not None:
                return copy.deepcopy(self.completed)
            if self.active is None:
                raise ValueError("Start a recording before saving a result.")
            self.completed = self.active
            self.active = None
            if link_lost and "Robot link was lost." not in self.completed["faults"]:
                self.completed["faults"].append("Robot link was lost.")
            self.completed["loop_at_end"] = loop_stats or {}
            self.completed["loop_overruns"] = max(0, (loop_stats or {}).get("overruns", 0)
                - self.completed["loop_at_start"].get("overruns", 0))
            return copy.deepcopy(self.completed)

    def clear(self):
        with self.lock:
            self.active = self.completed = None

    def snapshot(self):
        with self.lock:
            r = self.active or self.completed
            return None if r is None else {"id": r["id"], "candidate_id": r["candidate_id"],
                "active": self.active is not None, "elapsed_s": round(r["elapsed_s"], 1),
                "drive_seconds": round(r["drive_seconds"], 1),
                "longest_steady_seconds": math.floor(r["longest_steady_seconds"] * 10) / 10,
                "faults": list(r["faults"])}
