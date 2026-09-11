"""Human-readable robot_config geometry, commissioning, and D2 tuning report."""
from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from pydantic import ValidationError

from ..gait.trot import GaitCommand, TrotGait
from ..kinematics.leg import LegGeometry, angle_to_count, inverse_kinematics
from ..simulation.trot_check import simulate_trot
from .loader import ConfigError, load_config
from .schema import RobotConfig


@dataclass(frozen=True)
class MeasurementRow:
    field: str
    value_mm: float
    meaning: str


@dataclass(frozen=True)
class StandRow:
    leg: str
    foot_mm: dict[str, float]
    angles_deg: dict[str, float]
    servo_ids: dict[str, int | None]
    raw_counts: dict[str, int | None]


@dataclass(frozen=True)
class ConfigReport:
    schema_version: int
    firmware_expected: str
    fully_commissioned: bool
    assigned_slots: int
    servos_defined: int
    unverified_servos: list[str]
    measurements: list[MeasurementRow]
    gait_seed: dict[str, float | str]
    stand: list[StandRow]
    simulation: dict[str, Any]

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "firmware_expected": self.firmware_expected,
            "fully_commissioned": self.fully_commissioned,
            "assigned_slots": self.assigned_slots,
            "servos_defined": self.servos_defined,
            "unverified_servos": self.unverified_servos,
            "measurements": [asdict(row) for row in self.measurements],
            "gait_seed": self.gait_seed,
            "stand": [asdict(row) for row in self.stand],
            "simulation": self.simulation,
        }


def build_config_report(
    config: RobotConfig,
    *,
    command: GaitCommand | None = None,
    samples: int = 64,
) -> ConfigReport:
    command = command or GaitCommand(vy=1.0)
    gait = TrotGait(config.gait_seed, config)
    stand_targets = gait.stand_targets()
    stand_rows: list[StandRow] = []

    for leg in ("FL", "FR", "BL", "BR"):
        leg_spec = config.legs[leg]
        geom = LegGeometry(
            L1=config.links_mm.L1_hip,
            L2=config.links_mm.L2_femur,
            L3=config.links_mm.L3_tibia,
            side=leg_spec.side,
            knee_sign=leg_spec.knee_sign,
        )
        target = stand_targets[leg]
        angles = inverse_kinematics(geom, target.x, target.y, target.z)
        joints = ("hip", "femur", "tibia")
        ids = {joint: config.id_for(leg, joint) for joint in joints}
        counts: dict[str, int | None] = {}
        for joint, theta in zip(joints, angles):
            sid = ids[joint]
            spec = config.servos.get(str(sid)) if sid is not None else None
            counts[joint] = (
                angle_to_count(theta, invert=spec.invert, min_raw=spec.min_raw, max_raw=spec.max_raw)
                if spec is not None
                else None
            )
        stand_rows.append(
            StandRow(
                leg=leg,
                foot_mm={
                    "x": round(target.x, 3),
                    "y": round(target.y, 3),
                    "z": round(target.z, 3),
                },
                angles_deg={joint: round(math.degrees(theta), 3) for joint, theta in zip(joints, angles)},
                servo_ids=ids,
                raw_counts=counts,
            )
        )

    sim = simulate_trot(config, command, samples=samples)
    assigned_slots = sum(1 for leg in ("FL", "FR", "BL", "BR") for joint in ("hip", "femur", "tibia") if config.id_for(leg, joint) is not None)
    return ConfigReport(
        schema_version=config.schema_version,
        firmware_expected=config.firmware_expected,
        fully_commissioned=config.fully_commissioned(),
        assigned_slots=assigned_slots,
        servos_defined=len(config.servos),
        unverified_servos=config.unverified_servos(),
        measurements=_measurement_rows(config),
        gait_seed=config.gait_seed.model_dump(mode="json"),
        stand=stand_rows,
        simulation={
            "status": sim.status,
            "issue_codes": [issue.code for issue in sim.issues],
            "max_ik_error_mm": round(sim.max_ik_error_mm, 6),
            "swing_peak_clearance_mm": round(sim.swing_peak_clearance_mm, 3),
            "dynamic_only_samples": sim.dynamic_only_samples,
            "raw_limit_violations": sim.raw_limit_violations,
            "estimated_forward_speed_mm_s": round(sim.estimated_forward_speed_mm_s, 3),
        },
    )


def _measurement_rows(config: RobotConfig) -> list[MeasurementRow]:
    return [
        MeasurementRow("links_mm.L1_hip", config.links_mm.L1_hip, "hip abduction axis to femur pitch axis lateral offset"),
        MeasurementRow("links_mm.L2_femur", config.links_mm.L2_femur, "femur pitch axis to knee pitch axis"),
        MeasurementRow("links_mm.L3_tibia", config.links_mm.L3_tibia, "knee pitch axis to foot contact point"),
        MeasurementRow("body_mm.shoulder_lateral", config.body_mm.shoulder_lateral, "left hip axis to right hip axis spacing"),
        MeasurementRow("body_mm.fore_aft", config.body_mm.fore_aft, "front hip axis to rear hip axis spacing"),
    ]


def print_report(report: ConfigReport) -> None:
    print("DogV3 config inspect")
    print(f"schema_version: {report.schema_version}")
    print(f"firmware_expected: {report.firmware_expected}")
    print(
        "commissioning: "
        f"{'ready' if report.fully_commissioned else 'incomplete'} "
        f"({report.assigned_slots}/12 slots assigned, {report.servos_defined}/12 servos defined)"
    )
    if report.unverified_servos:
        print(f"unverified servos: {', '.join(report.unverified_servos)}")

    print("\nGeometry measurements (mm):")
    for row in report.measurements:
        print(f"  {row.field:<24} {row.value_mm:>8.3f}  {row.meaning}")

    print("\nD2 gait seed:")
    for key, value in report.gait_seed.items():
        print(f"  {key:<20} {value}")

    print("\nNominal stand targets:")
    print("leg foot_xyz_mm          angles_deg(hip,femur,tibia)       raw_counts")
    for row in report.stand:
        foot = row.foot_mm
        angles = row.angles_deg
        counts = row.raw_counts
        raw = ",".join("-" if counts[j] is None else str(counts[j]) for j in ("hip", "femur", "tibia"))
        print(
            f"{row.leg:<3} "
            f"({foot['x']:>7.2f},{foot['y']:>7.2f},{foot['z']:>7.2f}) "
            f"({angles['hip']:>8.2f},{angles['femur']:>8.2f},{angles['tibia']:>8.2f}) "
            f"{raw}"
        )

    sim = report.simulation
    print("\nD2 kinematic gate:")
    print(
        f"  status={sim['status']} issues={','.join(sim['issue_codes']) or '-'} "
        f"ik_error={sim['max_ik_error_mm']}mm swing_peak={sim['swing_peak_clearance_mm']}mm "
        f"speed={sim['estimated_forward_speed_mm_s']}mm/s"
    )


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Inspect DogV3 robot_config geometry, stand pose, and D2 tuning readiness")
    ap.add_argument("--config", default="robot_config.example.json")
    ap.add_argument("--samples", type=int, default=64)
    ap.add_argument("--vx", type=float, default=0.0)
    ap.add_argument("--vy", type=float, default=1.0)
    ap.add_argument("--wz", type=float, default=0.0)
    ap.add_argument("--body-z", type=float, default=0.0)
    ap.add_argument("--require-commissioned", action="store_true")
    ap.add_argument("--strict-warnings", action="store_true")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args(argv)

    try:
        cfg = load_config(Path(args.config), require_commissioned=args.require_commissioned)
        report = build_config_report(
            cfg,
            command=GaitCommand(vx=args.vx, vy=args.vy, wz=args.wz, body_z=args.body_z),
            samples=args.samples,
        )
    except (ConfigError, ValidationError, ValueError) as e:
        print(f"dogv3-config-inspect: {e}", file=sys.stderr)
        return 2

    if args.json:
        print(json.dumps(report.to_dict(), indent=2))
    else:
        print_report(report)

    status = report.simulation["status"]
    if args.require_commissioned and not report.fully_commissioned:
        return 1
    if status == "fail":
        return 1
    if status == "warn" and args.strict_warnings:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
