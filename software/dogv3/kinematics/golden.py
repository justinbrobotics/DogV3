"""Golden-vector export for the ESP32 C++ kinematics/trot port."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from ..config.loader import default_config
from ..gait.trot import GaitCommand, TrotGait
from .leg import (
    COUNTS_PER_RAD,
    NEUTRAL_COUNT,
    LegGeometry,
    angle_to_count,
    forward_kinematics,
    inverse_kinematics,
)


def build_golden_vectors() -> dict:
    cfg = default_config()
    geom_right = LegGeometry(
        L1=cfg.links_mm.L1_hip,
        L2=cfg.links_mm.L2_femur,
        L3=cfg.links_mm.L3_tibia,
        side="right",
        knee_sign=1,
    )
    geom_left = LegGeometry(
        L1=cfg.links_mm.L1_hip,
        L2=cfg.links_mm.L2_femur,
        L3=cfg.links_mm.L3_tibia,
        side="left",
        knee_sign=1,
    )
    points = [
        ("right_mid", geom_right, (40.0, 30.0, -200.0)),
        ("left_mid", geom_left, (-30.0, -40.0, -180.0)),
        ("right_low", geom_right, (0.0, 0.0, -230.0)),
    ]

    ik_cases = []
    for name, geom, point in points:
        angles = inverse_kinematics(geom, *point)
        fk = forward_kinematics(geom, *angles)
        ik_cases.append(
            {
                "name": name,
                "side": geom.side,
                "knee_sign": geom.knee_sign,
                "foot": _round_list(point),
                "angles_rad": _round_list(angles),
                "fk_mm": _round_list(fk),
            }
        )

    # Goldens exercise the portable kinematic core only. The max_fwd_speed /
    # max_yaw governors are a host-side teleop guard (not in the C++ port), so
    # lift them out of the way here or the caps would bind at the default seed
    # and change every gait vector.
    seed = cfg.gait_seed.model_copy(update={"max_fwd_speed": 1e6, "max_yaw": 1e6})
    gait = TrotGait(seed, cfg)
    gait_cases = []
    for phase, command in (
        (0.0, GaitCommand(vy=1.0)),
        (0.25, GaitCommand(vx=0.5, vy=0.25, wz=0.4, body_z=10.0)),
        (0.75, GaitCommand(vy=1.0)),
    ):
        gait.clock.reset(phase)
        targets = gait.targets(command)
        gait_cases.append(
            {
                "phase": phase,
                "command": {
                    "vx": command.vx,
                    "vy": command.vy,
                    "wz": command.wz,
                    "body_z": command.body_z,
                },
                "targets": {
                    leg: {
                        "x": round(t.x, 6),
                        "y": round(t.y, 6),
                        "z": round(t.z, 6),
                        "phase": round(t.phase, 6),
                        "in_stance": t.in_stance,
                    }
                    for leg, t in targets.items()
                },
            }
        )

    return {
        "schema": "dogv3-golden-v1",
        "constants": {
            "neutral_count": NEUTRAL_COUNT,
            "counts_per_rad": round(COUNTS_PER_RAD, 6),
        },
        "geometry_mm": {
            "L1": cfg.links_mm.L1_hip,
            "L2": cfg.links_mm.L2_femur,
            "L3": cfg.links_mm.L3_tibia,
            "shoulder_lateral": cfg.body_mm.shoulder_lateral,
            "fore_aft": cfg.body_mm.fore_aft,
        },
        "count_cases": [
            {"theta": 0.0, "invert": False, "count": angle_to_count(0.0)},
            {"theta": 0.5, "invert": False, "count": angle_to_count(0.5)},
            {"theta": 0.5, "invert": True, "count": angle_to_count(0.5, invert=True)},
            {"theta": -1.0, "invert": False, "count": angle_to_count(-1.0)},
        ],
        "ik_cases": ik_cases,
        "gait_cases": gait_cases,
    }


def _round_list(values: tuple[float, ...]) -> list[float]:
    return [round(v, 6) for v in values]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Export DogV3 Python-oracle golden vectors")
    ap.add_argument("--out", default=None, help="write JSON to this path instead of stdout")
    args = ap.parse_args(argv)

    text = json.dumps(build_golden_vectors(), indent=2) + "\n"
    if args.out:
        Path(args.out).write_text(text, encoding="utf-8")
    else:
        print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
