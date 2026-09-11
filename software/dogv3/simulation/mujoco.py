"""Generate a MuJoCo MJCF model from the commissioned config + measured params.

The model is built for sim-to-real transfer, not for looks:
  - joint axes and signs are chosen so MJCF qpos == the canonical joint angles
    from :mod:`dogv3.kinematics.leg` (guarded by an FK parity test);
  - joint ranges come from the commissioned ``min_raw``/``max_raw`` counts, so
    the sim hits the same clamps the real servos do;
  - actuators are position servos (kp/kv/forcerange) like the STS3215, not
    ideal torque motors — under load the sim leg lags exactly where the real
    one does;
  - the torso floats (freejoint) on a floor whose friction comes from
    ``simulation_params.json``.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any
from xml.etree import ElementTree as ET

from pydantic import ValidationError

from ..config.loader import ConfigError, load_config
from ..config.schema import RobotConfig
from ..gait.trot import TrotGait
from ..kinematics.leg import LegGeometry, count_to_angle, inverse_kinematics
from .dynamics_params import DynamicsParams, evaluate_readiness, load_dynamics_params

LEGS = ("FL", "FR", "BL", "BR")
JOINTS = ("hip", "femur", "tibia")


class MujocoModelError(Exception):
    """Raised when an MJCF model should not be generated."""


def build_mujoco_model(
    config: RobotConfig,
    params: DynamicsParams,
    *,
    allow_incomplete: bool = False,
    obstacle_mm: float | None = None,
    ramp_rad: float | None = None,
) -> str:
    readiness = evaluate_readiness(params)
    if not readiness.ready and not allow_incomplete:
        preview = ", ".join(readiness.missing[:8])
        more = f" and {len(readiness.missing) - 8} more" if len(readiness.missing) > 8 else ""
        raise MujocoModelError(f"dynamics params incomplete: {preview}{more}")

    incomplete = not readiness.ready
    model = ET.Element("mujoco", model=params.robot_name)
    if incomplete:
        model.append(ET.Comment("ESTIMATED-PARAMS MODEL: calibrate against the real known-good gait before trusting rankings"))
    ET.SubElement(model, "compiler", angle="radian", autolimits="true")
    ET.SubElement(model, "option", timestep=f"{1.0 / params.simulation_rate_hz:.6f}", gravity="0 0 -9.81")

    default = ET.SubElement(model, "default")
    ET.SubElement(default, "joint", damping=_fmt(_joint_value(params, "viscous", 0.02)),
                  armature="0.001", frictionloss=_fmt(_joint_value(params, "static", 0.02)))
    ET.SubElement(
        default,
        "geom",
        friction=_friction(params),
        solref="0.02 1",
        solimp="0.9 0.95 0.001",
    )

    world = ET.SubElement(model, "worldbody")
    ET.SubElement(world, "geom", name="floor", type="plane", size="4 4 0.02",
                  friction=_friction(params), rgba="0.25 0.27 0.28 1")
    if obstacle_mm:
        # A full-width step across the walking path (robot walks +Y), 0.45 m
        # out: the "can it climb over an object" test scene.
        h = _m(obstacle_mm)
        ET.SubElement(world, "geom", name="obstacle", type="box",
                      size=_vec(0.4, 0.05, h / 2), pos=_vec(0, 0.45, h / 2),
                      friction=_friction(params), rgba="0.55 0.35 0.1 1")
    if ramp_rad:
        # A laterally sloped platform under the robot (rotated about +Y):
        # the IMU-leveling test terrain.
        ET.SubElement(world, "geom", name="ramp", type="box",
                      size="1.5 1.5 0.05", pos="0 0 -0.03",
                      euler=_vec(0.0, ramp_rad, 0.0),
                      friction=_friction(params), rgba="0.3 0.32 0.36 1")
    foot_r = _number(params.foot_contact, "radius_m", 0.012)
    stand_z = _m(config.gait_seed.body_height) + foot_r
    torso = ET.SubElement(world, "body", name="body", pos=_vec(0, 0, stand_z))
    ET.SubElement(torso, "freejoint", name="root")
    # Torso mass = frame/electronics + battery. No explicit <inertial>: MuJoCo
    # derives the inertia tensor from the box, which beats a made-up diagonal.
    # Swap to an <inertial> element once real inertias are measured.
    torso_mass = _number(params.masses_kg, "body", 1.0) + _number(params.masses_kg, "battery", 0.0)
    ET.SubElement(
        torso,
        "geom",
        name="body_box",
        type="box",
        size=_vec(_m(config.body_mm.shoulder_lateral) / 2, _m(config.body_mm.fore_aft) / 2, 0.035),
        mass=_fmt(torso_mass),
        rgba="0.1 0.25 0.55 1",
    )

    for leg in LEGS:
        _add_leg(torso, config, params, leg)

    # STS3215-like position servos: stiffness kp, damping kv, torque saturation
    # at stall. ctrlrange mirrors the commissioned joint range.
    actuators = ET.SubElement(model, "actuator")
    stall = _number(params.actuator, "stall_torque_n_m", 2.9)
    kp = _number(params.actuator, "servo_kp_n_m_per_rad", 14.0)
    kv = _number(params.actuator, "servo_kv_n_m_s_per_rad", 0.35)
    for leg in LEGS:
        for joint in JOINTS:
            lo, hi = _joint_range(config, leg, joint)
            ET.SubElement(
                actuators, "position", name=f"{leg}_{joint}", joint=f"{leg}_{joint}",
                kp=_fmt(kp), kv=_fmt(kv),
                forcerange=f"{-stall:.6g} {stall:.6g}", ctrlrange=f"{lo:.6g} {hi:.6g}",
            )

    _add_stand_keyframe(model, config, stand_z)

    ET.indent(ET.ElementTree(model), space="  ")
    return ET.tostring(model, encoding="unicode") + "\n"


def write_mujoco_model(
    config_path: str | Path,
    params_path: str | Path,
    out_path: str | Path,
    *,
    allow_incomplete: bool = False,
) -> str:
    config = load_config(config_path, require_commissioned=False)
    params = load_dynamics_params(params_path)
    text = build_mujoco_model(config, params, allow_incomplete=allow_incomplete)
    Path(out_path).write_text(text, encoding="utf-8")
    return text


def _add_leg(parent: ET.Element, config: RobotConfig, params: DynamicsParams, leg: str) -> None:
    # lx matches LegGeometry.side_sign (left = -1). The hip abduction hinge is
    # +Y for BOTH sides and the femur/tibia pitch hinges are (lx, 0, 0), which
    # makes MJCF qpos equal the canonical angles from kinematics/leg.py on both
    # sides — the FK parity test locks this in.
    lx = -1.0 if config.legs[leg].side == "left" else 1.0
    sy = 1.0 if leg in ("FL", "FR") else -1.0
    pitch_axis = _vec(lx, 0, 0)
    hip = ET.SubElement(
        parent,
        "body",
        name=f"{leg}_hip",
        pos=_vec(lx * _m(config.body_mm.shoulder_lateral) / 2, sy * _m(config.body_mm.fore_aft) / 2, 0),
    )
    lo, hi = _joint_range(config, leg, "hip")
    ET.SubElement(hip, "joint", name=f"{leg}_hip", type="hinge", axis="0 1 0", range=f"{lo:.6g} {hi:.6g}")
    ET.SubElement(
        hip,
        "geom",
        name=f"{leg}_hip_link",
        type="capsule",
        fromto=_fromto(0, 0, 0, lx * _m(config.links_mm.L1_hip), 0, 0),
        size="0.018",
        mass=_fmt(_number(params.masses_kg, "hip_link_each", 0.05)),
        rgba="0.1 0.55 0.45 1",
    )

    femur = ET.SubElement(hip, "body", name=f"{leg}_femur", pos=_vec(lx * _m(config.links_mm.L1_hip), 0, 0))
    lo, hi = _joint_range(config, leg, "femur")
    ET.SubElement(femur, "joint", name=f"{leg}_femur", type="hinge", axis=pitch_axis, range=f"{lo:.6g} {hi:.6g}")
    ET.SubElement(
        femur,
        "geom",
        name=f"{leg}_femur_link",
        type="capsule",
        fromto=_fromto(0, 0, 0, 0, 0, -_m(config.links_mm.L2_femur)),
        size="0.016",
        mass=_fmt(_number(params.masses_kg, "femur_link_each", 0.08)),
        rgba="0.72 0.45 0.12 1",
    )

    tibia = ET.SubElement(femur, "body", name=f"{leg}_tibia", pos=_vec(0, 0, -_m(config.links_mm.L2_femur)))
    lo, hi = _joint_range(config, leg, "tibia")
    ET.SubElement(tibia, "joint", name=f"{leg}_tibia", type="hinge", axis=pitch_axis, range=f"{lo:.6g} {hi:.6g}")
    ET.SubElement(
        tibia,
        "geom",
        name=f"{leg}_tibia_link",
        type="capsule",
        fromto=_fromto(0, 0, 0, 0, 0, -_m(config.links_mm.L3_tibia)),
        size="0.014",
        mass=_fmt(_number(params.masses_kg, "tibia_link_each", 0.07)),
        rgba="0.7 0.18 0.16 1",
    )
    foot_r = _number(params.foot_contact, "radius_m", 0.018)
    ET.SubElement(
        tibia,
        "geom",
        name=f"{leg}_foot",
        type="sphere",
        pos=_vec(0, 0, -_m(config.links_mm.L3_tibia)),
        size=_fmt(foot_r),
        mass=_fmt(_number(params.masses_kg, "foot_each", 0.03)),
        rgba="0.05 0.05 0.05 1",
    )
    # Foot site: slip and clearance are measured here during rollouts.
    ET.SubElement(tibia, "site", name=f"{leg}_foot_site",
                  pos=_vec(0, 0, -_m(config.links_mm.L3_tibia)), size="0.004")


def _joint_range(config: RobotConfig, leg: str, joint: str) -> tuple[float, float]:
    """Commissioned raw-count range -> canonical joint-angle range (rad).

    ``invert`` flips the count axis, so the two endpoints are converted and
    sorted. Falls back to a generous +-pi for uncommissioned configs."""
    spec = config.servo_by_slot(f"{leg}_{joint}")
    if spec is None:
        return (-3.1416, 3.1416)
    a = count_to_angle(spec.min_raw, invert=spec.invert)
    b = count_to_angle(spec.max_raw, invert=spec.invert)
    lo, hi = sorted((a, b))
    if hi - lo < 1e-6:  # degenerate range would make the joint immovable
        return (-3.1416, 3.1416)
    return (lo, hi)


def stand_joint_angles(config: RobotConfig) -> list[float]:
    """Canonical joint angles for the gait-seed stand pose, in MJCF joint
    order (FL, FR, BL, BR x hip, femur, tibia). Shared by the keyframe here
    and the rollout controller's initial servo state."""
    gait = TrotGait(config.gait_seed, config)
    targets = gait.stand_targets()
    out: list[float] = []
    for leg in LEGS:
        geom = LegGeometry(
            L1=config.links_mm.L1_hip, L2=config.links_mm.L2_femur, L3=config.links_mm.L3_tibia,
            side=config.legs[leg].side, knee_sign=config.legs[leg].knee_sign,
        )
        ft = targets[leg]
        out.extend(inverse_kinematics(geom, ft.x, ft.y, ft.z))
    return out


def _add_stand_keyframe(model: ET.Element, config: RobotConfig, stand_z: float) -> None:
    angles = stand_joint_angles(config)
    qpos = [0.0, 0.0, stand_z, 1.0, 0.0, 0.0, 0.0] + angles  # freejoint + 12 hinges
    key = ET.SubElement(model, "keyframe")
    ET.SubElement(key, "key", name="stand",
                  qpos=" ".join(_fmt(v) for v in qpos),
                  ctrl=" ".join(_fmt(v) for v in angles))


def _number(values: dict[str, Any], key: str, fallback: float) -> float:
    value = values.get(key)
    if isinstance(value, (int, float)):
        return float(value)
    return fallback


def _joint_value(params: DynamicsParams, kind: str, fallback: float) -> float:
    values = [
        value
        for key, value in params.joint_friction.items()
        if kind in key and isinstance(value, (int, float))
    ]
    if not values:
        return fallback
    return sum(float(value) for value in values) / len(values)


def _friction(params: DynamicsParams) -> str:
    sliding = _number(params.foot_contact, "static_friction", 0.8)
    torsional = _number(params.foot_contact, "rolling_friction", 0.01)
    rolling = _number(params.foot_contact, "dynamic_friction", 0.6)
    return _vec(sliding, torsional, rolling)


def _m(mm: float) -> float:
    return mm / 1000.0


def _fmt(value: float) -> str:
    return f"{value:.6g}"


def _vec(*values: float) -> str:
    return " ".join(_fmt(value) for value in values)


def _fromto(*values: float) -> str:
    return _vec(*values)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Generate a DogV3 MuJoCo MJCF scaffold from measured parameters")
    ap.add_argument("--config", default="robot_config.example.json")
    ap.add_argument("--params", default="simulation_params.example.json")
    ap.add_argument("--out", required=True)
    ap.add_argument("--allow-incomplete", action="store_true", help="emit a placeholder model even when measurements are missing")
    args = ap.parse_args(argv)

    try:
        write_mujoco_model(args.config, args.params, args.out, allow_incomplete=args.allow_incomplete)
    except (OSError, json.JSONDecodeError, ConfigError, ValidationError, MujocoModelError, ValueError) as e:
        print(f"dogv3-sim-mujoco: {e}", file=sys.stderr)
        return 2
    print(f"wrote MuJoCo MJCF scaffold: {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
