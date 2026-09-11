"""Kinematic digital twin — evaluate a gait seed without touching hardware.

This is the kinematic layer of the D3 digital twin: it runs the exact same
trot/crawl generator and IK the live controller runs (``TrotGait`` →
``inverse_kinematics`` → counts) against the *commissioned* config, and checks
the resulting joint trajectories against the STS3215 servo model. It answers,
before you arm anything: how fast is this profile, will any joint out-run the
servo, and will any joint slam into its commissioned range clamp.

The dynamics layer (masses, contact, MuJoCo) lives in :mod:`.mujoco` /
:mod:`.dynamics_params`; this module needs neither — it is pure geometry, so
D2 can call it live while tuning.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from ..config.schema import GaitSeed, RobotConfig
from ..gait.phase import CRAWL_MIN_DUTY
from ..gait.trot import GaitCommand, TrotGait
from ..kinematics.leg import (
    COUNTS_PER_RAD,
    NEUTRAL_COUNT,
    LegGeometry,
    forward_kinematics,
    inverse_kinematics,
)
from ..runtime.control import CONTROL_HZ_DEFAULT, DEFAULT_ACC, DEFAULT_SPEED

DEG = 180.0 / math.pi
# Commanded velocity ceiling the controller writes with every goal
# (DEFAULT_SPEED counts/s), expressed in deg/s.
CMD_CEILING_DPS = DEFAULT_SPEED / COUNTS_PER_RAD * DEG
CMD_ACCEL_CEILING_DPS2 = DEFAULT_ACC * 100.0 / COUNTS_PER_RAD * DEG
# FK-vs-target mismatch above this means IK clamped: target outside leg reach.
REACH_TOL_MM = 1.0

LEGS = ("FL", "FR", "BL", "BR")
JOINTS = ("hip", "femur", "tibia")


@dataclass
class GaitAnalysis:
    """Twin verdict for one (seed, command) pair. All speeds are demands the
    gait asks for — the servo model says whether they are achievable."""

    speed_mms: float            # commanded body speed at this stick
    stride_mm: float            # post-governor stride actually used
    cadence_hz: float
    swing_time_s: float
    peak_joint_dps: float       # worst joint-speed demand anywhere
    peak_by_joint: dict[str, float] = field(default_factory=dict)
    cmd_ceiling_dps: float = CMD_CEILING_DPS
    saturation_pct: float = 0.0   # % of samples demanding > cmd ceiling
    range_hit_pct: dict[str, float] = field(default_factory=dict)  # slot -> % samples clamped
    max_reach_err_mm: float = 0.0
    peak_foot_speed_mms: float = 0.0
    peak_joint_accel_dps2: float = 0.0
    peak_accel_by_joint: dict[str, float] = field(default_factory=dict)
    cmd_accel_ceiling_dps2: float = CMD_ACCEL_CEILING_DPS2
    warnings: list[str] = field(default_factory=list)
    assumptions: list[str] = field(default_factory=lambda: [
        "command ceiling is a software setting, not measured loaded servo speed",
        "kinematic rollout excludes mass, friction, contact compliance, tracking error, and measured CoM",
        "acceleration ceiling uses the repository ACC register model, not measured loaded acceleration",
        "steady command samples do not validate starting, stopping, or physical balance",
    ])

    def to_dict(self) -> dict:
        return {
            "speed_mms": round(self.speed_mms, 1),
            "stride_mm": round(self.stride_mm, 1),
            "cadence_hz": round(self.cadence_hz, 2),
            "swing_time_s": round(self.swing_time_s, 3),
            "peak_joint_dps": round(self.peak_joint_dps, 1),
            "peak_by_joint": {k: round(v, 1) for k, v in self.peak_by_joint.items()},
            "cmd_ceiling_dps": round(self.cmd_ceiling_dps, 1),
            "saturation_pct": round(self.saturation_pct, 1),
            "range_hit_pct": {k: round(v, 1) for k, v in self.range_hit_pct.items()},
            "max_reach_err_mm": round(self.max_reach_err_mm, 2),
            "peak_foot_speed_mms": round(self.peak_foot_speed_mms, 1),
            "peak_joint_accel_dps2": round(self.peak_joint_accel_dps2, 1),
            "peak_accel_by_joint": {k: round(v, 1) for k, v in self.peak_accel_by_joint.items()},
            "cmd_accel_ceiling_dps2": round(self.cmd_accel_ceiling_dps2, 1),
            "warnings": self.warnings,
            "assumptions": self.assumptions,
        }


def profile_summary(seed: GaitSeed) -> dict:
    """Cheap closed-form numbers for the GUI's profile-compare table (no
    trajectory rollout): full-stick body speed after the governor, cadence."""
    duty = max(seed.duty_factor, CRAWL_MIN_DUTY) if seed.gait_type == "crawl" else seed.duty_factor
    stride = min(seed.step_length, seed.max_fwd_speed * duty * seed.cycle_period)
    speed = stride / (duty * seed.cycle_period) if seed.cycle_period > 0 else 0.0
    return {
        "gait_type": seed.gait_type,
        "cycle_period": seed.cycle_period,
        "step_length": seed.step_length,
        "step_height": seed.step_height,
        "duty_factor": seed.duty_factor,
        "body_height": seed.body_height,
        "swing_shape": seed.swing_shape,
        "speed_mms": round(speed, 1),
    }


def evaluate_gait(
    config: RobotConfig,
    seed: GaitSeed,
    cmd: GaitCommand | None = None,
    *,
    hz: float = CONTROL_HZ_DEFAULT,
    cycles: float = 2.0,
) -> GaitAnalysis:
    """Roll the gait forward ``cycles`` full cycles at the control rate and
    measure what it demands of every joint. Defaults to full-stick forward."""
    cmd = cmd or GaitCommand(vy=1.0)
    gait = TrotGait(seed, config)
    geoms = {
        leg: LegGeometry(
            L1=config.links_mm.L1_hip,
            L2=config.links_mm.L2_femur,
            L3=config.links_mm.L3_tibia,
            side=config.legs[leg].side,
            knee_sign=config.legs[leg].knee_sign,
        )
        for leg in LEGS
    }
    specs = {
        f"{leg}_{joint}": config.servo_by_slot(f"{leg}_{joint}")
        for leg in LEGS
        for joint in JOINTS
    }

    dt = 1.0 / hz
    n = max(2, int(round(cycles * seed.cycle_period * hz)))
    duty = max(seed.duty_factor, CRAWL_MIN_DUTY) if seed.gait_type == "crawl" else seed.duty_factor
    stride = min(seed.step_length * min(1.0, abs(cmd.vy)),
                 seed.max_fwd_speed * duty * seed.cycle_period)
    speed = stride / (duty * seed.cycle_period) if seed.cycle_period > 0 else 0.0

    prev_angles: dict[str, tuple[float, float, float]] = {}
    prev_velocities: dict[str, float] = {}
    peak_accel_by_joint = {slot: 0.0 for slot in specs}
    prev_feet: dict[str, tuple[float, float, float]] = {}
    peak_by_joint = {slot: 0.0 for slot in specs}
    range_hits = {slot: 0 for slot in specs}
    sat_samples = 0
    max_reach_err = 0.0
    peak_foot = 0.0

    for i in range(n):
        gait.clock.advance(dt)
        targets = gait.targets(cmd)
        tick_saturated = False
        for leg in LEGS:
            ft = targets[leg]
            geom = geoms[leg]
            angles = inverse_kinematics(geom, ft.x, ft.y, ft.z)
            fk = forward_kinematics(geom, *angles)
            err = math.dist(fk, (ft.x, ft.y, ft.z))
            max_reach_err = max(max_reach_err, err)
            for j, joint in enumerate(JOINTS):
                slot = f"{leg}_{joint}"
                spec = specs[slot]
                if spec is not None:
                    invert = spec.invert
                    raw = NEUTRAL_COUNT + (-1 if invert else 1) * angles[j] * COUNTS_PER_RAD
                    if raw < spec.min_raw or raw > spec.max_raw:
                        range_hits[slot] += 1
                if i > 0 and leg in prev_angles:
                    velocity = (angles[j] - prev_angles[leg][j]) / dt * DEG
                    dps = abs(velocity)
                    peak_by_joint[slot] = max(peak_by_joint[slot], dps)
                    if slot in prev_velocities:
                        acceleration = abs(velocity - prev_velocities[slot]) / dt
                        peak_accel_by_joint[slot] = max(peak_accel_by_joint[slot], acceleration)
                    prev_velocities[slot] = velocity
                    if dps > CMD_CEILING_DPS:
                        tick_saturated = True
            if i > 0 and leg in prev_feet:
                peak_foot = max(peak_foot, math.dist(prev_feet[leg], (ft.x, ft.y, ft.z)) / dt)
            prev_angles[leg] = angles
            prev_feet[leg] = (ft.x, ft.y, ft.z)
        if tick_saturated:
            sat_samples += 1

    peak = max(peak_by_joint.values()) if peak_by_joint else 0.0
    out = GaitAnalysis(
        speed_mms=speed,
        stride_mm=stride,
        cadence_hz=1.0 / seed.cycle_period if seed.cycle_period > 0 else 0.0,
        swing_time_s=(1.0 - duty) * seed.cycle_period,
        peak_joint_dps=peak,
        peak_by_joint={k: v for k, v in peak_by_joint.items() if v > 0},
        saturation_pct=100.0 * sat_samples / max(1, n - 1),
        range_hit_pct={k: 100.0 * v / n for k, v in range_hits.items() if v},
        max_reach_err_mm=max_reach_err,
        peak_foot_speed_mms=peak_foot,
        peak_joint_accel_dps2=max(peak_accel_by_joint.values(), default=0.0),
        peak_accel_by_joint=peak_accel_by_joint,
    )
    _add_warnings(out, seed)
    return out


def _add_warnings(a: GaitAnalysis, seed: GaitSeed) -> None:
    w = a.warnings
    if a.peak_joint_accel_dps2 > a.cmd_accel_ceiling_dps2:
        w.append(
            f"demands {a.peak_joint_accel_dps2:.0f} deg/s^2 > modeled command acceleration "
            f"{a.cmd_accel_ceiling_dps2:.0f} deg/s^2; allow more swing time or reduce lift/stride"
        )
    if a.peak_joint_dps > a.cmd_ceiling_dps:
        w.append(
            f"demands {a.peak_joint_dps:.0f} deg/s > commanded ceiling "
            f"{a.cmd_ceiling_dps:.0f} deg/s — swing feet will lag and drag; "
            "raise cycle_period or shrink step_length/step_height"
        )
    for slot, pct in sorted(a.range_hit_pct.items(), key=lambda kv: -kv[1]):
        w.append(
            f"{slot} exceeds its commissioned range {pct:.0f}% of the cycle — "
            "the servo clamps there, so the foot will not track the planned path"
        )
    if a.max_reach_err_mm > REACH_TOL_MM:
        w.append(
            f"targets up to {a.max_reach_err_mm:.1f} mm outside leg reach (IK "
            "clamped) — lower body_height or shrink the stride/step_height"
        )
    if seed.gait_type == "trot" and seed.duty_factor < 0.55:
        w.append("duty < 0.55: near-flight trot; needs momentum and clean tracking")
    if seed.gait_type == "crawl" and seed.body_sway_amp < 5.0:
        w.append(
            "crawl has little lateral body sway/shift; check the body-origin support "
            "margin, then validate measured CoM and load transfer on the stand"
        )
