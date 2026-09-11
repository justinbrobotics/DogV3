"""Symmetry + pose validators — is the config kinematically consistent, and does
the REAL robot agree with the model?

Shared, pure, hardware-free. D1 uses it as a commissioning gate and a live
real-vs-model check; D2 references it for pose sanity while tuning gaits/poses.
Feed it a :class:`RobotConfig` and (optionally) a ``{servo_id: raw}`` reading
from wherever — driver, log, or a commanded pose. Returns JSON-friendly dicts so
a server, CLI, or 3D viewer can render them directly.

The single source of truth for the signs used here is
:mod:`dogv3.kinematics.conventions`.
"""
from __future__ import annotations

import math

from .config.schema import RobotConfig
from .gait.trot import GaitCommand, TrotGait
from .kinematics.conventions import JOINTS, NEUTRAL_COUNT, physical_sign
from .kinematics.leg import (
    COUNTS_PER_RAD,
    LegGeometry,
    angle_to_count,
    count_to_angle,
    inverse_kinematics,
    joint_positions,
)

LEGS = ("FL", "FR", "BL", "BR")
# Nominal hip location on the body (lateral, fore-aft sign), for the 3D chain.
_LEG_HIP_SIGN = {"FL": (-1.0, 1.0), "FR": (1.0, 1.0), "BL": (-1.0, -1.0), "BR": (1.0, -1.0)}


def _geom(config: RobotConfig, leg: str) -> LegGeometry:
    return LegGeometry(
        L1=config.links_mm.L1_hip, L2=config.links_mm.L2_femur, L3=config.links_mm.L3_tibia,
        side=config.legs[leg].side, knee_sign=config.legs[leg].knee_sign,
    )


def reconstruct_pose(config: RobotConfig, raws: dict[int, int] | None = None) -> dict:
    """The model's view of a pose from measured (or home) servo raws — exactly the
    data a 3D viewer renders. Per leg: each joint's canonical angle, physical angle
    (outward/forward/flex +), and the body-frame chain ``[hip, femur, knee, foot]``
    in mm. Missing readings fall back to the servo's home."""
    raws = raws or {}
    out: dict[str, dict] = {}
    for leg in LEGS:
        g = _geom(config, leg)
        angles: list[float] = []
        joints: dict[str, dict] = {}
        for j in JOINTS:
            sid = getattr(config.legs[leg].ids, j)
            spec = config.servos.get(str(sid)) if sid is not None else None
            raw = raws.get(sid) if sid is not None else None
            if raw is None:
                raw = spec.home_raw if spec else NEUTRAL_COUNT
            invert = spec.invert if spec else False
            a = count_to_angle(int(raw), invert=invert)
            ps = physical_sign(config.legs[leg].side, invert, j)
            angles.append(a)
            joints[j] = {
                "id": sid,
                "raw": int(raw),
                "angle_deg": round(math.degrees(a), 1),
                "physical_deg": round(math.degrees(ps * (int(raw) - NEUTRAL_COUNT) / COUNTS_PER_RAD), 1),
            }
        sx, sy = _LEG_HIP_SIGN[leg]
        ox = sx * config.body_mm.shoulder_lateral / 2.0
        oy = sy * config.body_mm.fore_aft / 2.0
        points = [[round(ox + p[0], 1), round(oy + p[1], 1), round(p[2], 1)] for p in joint_positions(g, *angles)]
        out[leg] = {"joints": joints, "points": points, "foot": points[3]}
    return out


def validate_config(config: RobotConfig, margin_mm: float = 2.0) -> dict:
    """Static, no-hardware check that the config is a usable, understandable D1
    baseline: fully commissioned, ranges sane, knees-back, the stand pose left/right
    and front/rear symmetric, and every stand angle inside its captured range.
    Returns ``{"ok", "symmetric", "issues": [{level, code, message}]}``."""
    issues: list[dict] = []

    def add(level: str, code: str, message: str) -> None:
        issues.append({"level": level, "code": code, "message": message})

    if not config.fully_commissioned():
        add("error", "not_commissioned",
            "config is not fully commissioned (assign/center/direction/range incomplete)")

    for sid, s in config.servos.items():
        if not (0 <= s.min_raw <= s.max_raw <= 4095):
            add("error", "inverted_range", f"servo {sid} ({s.slot}) has an inverted/out-of-bounds raw range [{s.min_raw}, {s.max_raw}]")
        elif not (s.min_raw <= s.home_raw <= s.max_raw):
            add("error", "home_out_of_range", f"servo {sid} ({s.slot}) home {s.home_raw} is outside its range")

    try:
        stand = TrotGait(config.gait_seed, config).stand_targets(GaitCommand())
        feet, knees = {}, {}
        for leg in LEGS:
            g = _geom(config, leg)
            ft = stand[leg]
            a = inverse_kinematics(g, ft.x, ft.y, ft.z)
            _, _, knee, foot = joint_positions(g, *a)
            feet[leg], knees[leg] = foot, knee
            if knee[1] >= 0:
                add("warn", "knee_not_back", f"{leg} knee points toward the head, not the tail — check knee_sign")
            for j, th in zip(JOINTS, a):
                sid = getattr(config.legs[leg].ids, j)
                spec = config.servos.get(str(sid))
                if spec is None:
                    continue
                cnt = angle_to_count(th, invert=spec.invert)
                if not (spec.min_raw <= cnt <= spec.max_raw):
                    add("warn", "stand_out_of_range",
                        f"{leg}_{j} stand angle {math.degrees(th):.0f}deg -> raw {cnt} is outside its range [{spec.min_raw}, {spec.max_raw}]")
        for lft, rgt in (("FL", "FR"), ("BL", "BR")):
            if abs(feet[lft][0] + feet[rgt][0]) > margin_mm or abs(feet[lft][2] - feet[rgt][2]) > margin_mm:
                add("warn", "left_right_asymmetry", f"{lft}/{rgt} stand feet are not left-right symmetric")
        for fr, bk in (("FL", "BL"), ("FR", "BR")):
            if any(abs(feet[fr][k] - feet[bk][k]) > margin_mm for k in (0, 1, 2)):
                add("warn", "front_rear_asymmetry", f"{fr}/{bk} stand feet differ front-to-rear")
    except Exception as e:  # pragma: no cover - defensive
        add("error", "stand_failed", f"could not compute the stand pose: {e}")

    has_error = any(i["level"] == "error" for i in issues)
    has_warn = any(i["level"] == "warn" for i in issues)
    if not issues:
        add("ok", "valid", "commissioned, symmetric, knees-back, and within range")
    return {"ok": not has_error, "symmetric": not (has_error or has_warn), "issues": issues}


def pose_symmetry(config: RobotConfig, raws: dict[int, int]) -> dict:
    """Per-joint physical-angle spread of a MEASURED pose. For a pose meant to be
    symmetric (e.g. both hips splayed out equally), each spread should be ~0."""
    pose = reconstruct_pose(config, raws)

    def phys(leg: str, j: str) -> float:
        return pose[leg]["joints"][j]["physical_deg"]

    out: dict[str, dict] = {}
    for j in JOINTS:
        out[j] = {
            "front_left_right_deg": round(abs(phys("FL", j) - phys("FR", j)), 1),
            "rear_left_right_deg": round(abs(phys("BL", j) - phys("BR", j)), 1),
            "left_front_rear_deg": round(abs(phys("FL", j) - phys("BL", j)), 1),
        }
    return out


def stand_raws(config: RobotConfig) -> dict[int, int]:
    """The servo raws the model would command for the neutral stand — the
    'expected' side of a real-vs-model comparison."""
    stand = TrotGait(config.gait_seed, config).stand_targets(GaitCommand())
    out: dict[int, int] = {}
    for leg in LEGS:
        g = _geom(config, leg)
        a = inverse_kinematics(g, stand[leg].x, stand[leg].y, stand[leg].z)
        for j, th in zip(JOINTS, a):
            sid = getattr(config.legs[leg].ids, j)
            spec = config.servos.get(str(sid))
            if spec is None:
                continue
            out[sid] = angle_to_count(th, invert=spec.invert, min_raw=spec.min_raw, max_raw=spec.max_raw)
    return out


def pose_matches(config: RobotConfig, measured: dict[int, int], expected: dict[int, int],
                 tol_deg: float = 5.0) -> dict:
    """Does the REAL robot agree with the model? Compares measured raws to an
    expected pose (e.g. :func:`stand_raws`) per joint, in degrees."""
    rows: list[dict] = []
    ok = True
    for leg in LEGS:
        for j in JOINTS:
            sid = getattr(config.legs[leg].ids, j)
            spec = config.servos.get(str(sid))
            if spec is None or sid not in measured or sid not in expected:
                continue
            dev = math.degrees(abs(count_to_angle(int(measured[sid]), invert=spec.invert)
                                   - count_to_angle(int(expected[sid]), invert=spec.invert)))
            within = dev <= tol_deg
            ok = ok and within
            rows.append({"leg": leg, "joint": j, "id": sid, "dev_deg": round(dev, 1), "within": within})
    return {"ok": ok, "tol_deg": tol_deg, "joints": rows}
