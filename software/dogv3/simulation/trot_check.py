"""Kinematic trot checker used before bench or floor tests.

This is not a dynamics simulator. It answers the first questions that must be
true before hardware moves: are the foot targets reachable, are servo counts
inside commissioned limits, how much swing clearance exists, and when does the
gait rely on dynamic two-foot support? Support margin is measured against the
configured body-frame origin, not an inferred centre of mass.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import asdict, dataclass, field
from typing import Literal

from pydantic import ValidationError

from ..config.loader import ConfigError, load_config, save_config
from ..config.schema import GaitSeed, RobotConfig
from ..gait.phase import CRAWL_MIN_DUTY
from ..gait.trot import FootTarget, GaitCommand, TrotGait
from ..kinematics.leg import (
    COUNTS_PER_RAD,
    NEUTRAL_COUNT,
    LegGeometry,
    forward_kinematics,
    inverse_kinematics,
)
from .twin import evaluate_gait

LEG_ORDER = ("FL", "FR", "BL", "BR")
JOINT_ORDER = ("hip", "femur", "tibia")
HIP_SIGN = {
    "FL": (-1.0, +1.0),
    "FR": (+1.0, +1.0),
    "BL": (-1.0, -1.0),
    "BR": (+1.0, -1.0),
}


@dataclass(frozen=True)
class SimulationIssue:
    severity: Literal["fail", "warn"]
    code: str
    message: str


@dataclass
class GaitSimulationResult:
    samples: int
    command: dict[str, float]
    gait: dict[str, float | str]
    estimated_forward_speed_mm_s: float
    stance_time_s: float
    swing_time_s: float
    max_ik_error_mm: float = 0.0
    max_abs_angle_deg: float = 0.0
    raw_count_min: int | None = None
    raw_count_max: int | None = None
    raw_limit_violations: int = 0
    min_stance_feet: int = 4
    dynamic_only_samples: int = 0
    support_margin_min_mm: float | None = None
    swing_peak_clearance_mm: float = 0.0
    peak_joint_dps: float = 0.0
    command_ceiling_dps: float = 0.0
    command_rate_saturation_pct: float = 0.0
    support_reference: str = "body_frame_origin_not_measured_com"
    issues: list[SimulationIssue] = field(default_factory=list)

    @property
    def status(self) -> Literal["pass", "warn", "fail"]:
        if any(issue.severity == "fail" for issue in self.issues):
            return "fail"
        if self.issues:
            return "warn"
        return "pass"

    def to_dict(self) -> dict:
        data = asdict(self)
        data["status"] = self.status
        return data


def simulate_trot(
    config: RobotConfig,
    command: GaitCommand | None = None,
    *,
    samples: int = 64,
    max_ik_error_mm: float = 1.0,
    min_swing_peak_mm: float = 15.0,
) -> GaitSimulationResult:
    """Sample one gait cycle and return reachability/support metrics."""
    if samples < 8:
        raise ValueError("samples must be >= 8")

    cmd = command or GaitCommand(vy=1.0)
    seed = config.gait_seed
    duty = max(seed.duty_factor, CRAWL_MIN_DUTY) if seed.gait_type == "crawl" else seed.duty_factor
    stride_cap = seed.max_fwd_speed * duty * seed.cycle_period
    stride = min(
        seed.step_length * min(1.0, abs(cmd.vy)),
        stride_cap,
    )
    lateral_stride = min(seed.step_length * min(1.0, abs(cmd.vx)), stride_cap)
    yaw = min(
        seed.turn_gain * min(1.0, abs(cmd.wz)),
        seed.max_yaw * duty * seed.cycle_period,
    )
    # Match TrotGait's quiet-idle gate: tiny joystick noise with no lateral or
    # yaw request returns the planted stand instead of advancing a gait.
    if stride < 0.5 and lateral_stride < 0.5 and yaw < 1e-3:
        stride = 0.0
    gait = TrotGait(seed, config)
    result = GaitSimulationResult(
        samples=samples,
        command={"vx": cmd.vx, "vy": cmd.vy, "wz": cmd.wz, "body_z": cmd.body_z},
        gait={
            "body_height": seed.body_height,
            "gait_type": seed.gait_type,
            "step_length": seed.step_length,
            "step_height": seed.step_height,
            "cycle_period": seed.cycle_period,
            "duty_factor": seed.duty_factor,
            "effective_duty_factor": duty,
            "stance_width_offset": seed.stance_width_offset,
            "turn_gain": seed.turn_gain,
            "swing_shape": seed.swing_shape,
        },
        estimated_forward_speed_mm_s=(stride / (duty * seed.cycle_period)),
        stance_time_s=seed.cycle_period * duty,
        swing_time_s=seed.cycle_period * (1.0 - duty),
    )

    if not config.fully_commissioned():
        result.issues.append(
            SimulationIssue(
                "warn",
                "config_not_commissioned",
                "Config is not fully commissioned; missing IDs or servo ranges are not checked.",
            )
        )
    if seed.gait_type == "trot" and duty <= 0.5:
        result.issues.append(
            SimulationIssue(
                "fail",
                "duty_factor_too_low",
                "Trot duty_factor must stay above 0.5 so both diagonal pairs are not airborne.",
            )
        )

    geometries = {
        leg: LegGeometry(
            L1=config.links_mm.L1_hip,
            L2=config.links_mm.L2_femur,
            L3=config.links_mm.L3_tibia,
            side=config.legs[leg].side,
            knee_sign=config.legs[leg].knee_sign,
        )
        for leg in LEG_ORDER
    }

    for i in range(samples):
        gait.clock.reset(i / samples)
        targets = gait.targets(cmd)
        _sample_targets(config, geometries, targets, cmd, result)

    # The geometric sampler cannot see whether the controller's configured
    # joint-speed command can follow the path. Reuse the exact runtime gait/IK
    # rollout to expose command-channel saturation; this still says nothing
    # about the unmeasured physical servo speed under load.
    tracking = evaluate_gait(config, seed, cmd, cycles=1.0)
    result.peak_joint_dps = tracking.peak_joint_dps
    result.command_ceiling_dps = tracking.cmd_ceiling_dps
    result.command_rate_saturation_pct = tracking.saturation_pct

    if result.max_ik_error_mm > max_ik_error_mm:
        result.issues.append(
            SimulationIssue(
                "fail",
                "ik_error",
                f"Max IK/FK error {result.max_ik_error_mm:.3f} mm exceeds {max_ik_error_mm:.3f} mm.",
            )
        )
    if result.raw_limit_violations:
        result.issues.append(
            SimulationIssue(
                "fail",
                "servo_limit",
                f"{result.raw_limit_violations} sampled joint goals exceed commissioned raw limits.",
            )
        )
    if result.command_rate_saturation_pct > 0.0:
        result.issues.append(
            SimulationIssue(
                "warn",
                "command_rate_saturation",
                f"Planned joints exceed the controller's {result.command_ceiling_dps:.0f} deg/s "
                f"command setting in {result.command_rate_saturation_pct:.1f}% of sampled ticks; "
                "tracking lag and foot drag are possible.",
            )
        )
    if seed.gait_type == "crawl":
        result.issues.append(
            SimulationIssue(
                "warn",
                "crawl_transition_unmanaged",
                "Crawl starts and stops at the clock's current phase, so stand-to-sway and "
                "swing-to-stand transitions are not yet load-shift/settle managed.",
            )
        )
    if result.min_stance_feet < 2:
        result.issues.append(
            SimulationIssue("fail", "support_count", "Fewer than two feet are in stance during part of the cycle.")
        )
    if result.dynamic_only_samples:
        result.issues.append(
            SimulationIssue(
                "warn",
                "dynamic_support",
                f"{result.dynamic_only_samples}/{samples} samples have only diagonal two-foot support; bench testing or dynamics simulation must validate balance.",
            )
        )
    if result.swing_peak_clearance_mm < min_swing_peak_mm:
        result.issues.append(
            SimulationIssue(
                "warn",
                "low_swing_clearance",
                f"Peak sampled swing clearance {result.swing_peak_clearance_mm:.1f} mm is below {min_swing_peak_mm:.1f} mm.",
            )
        )
    if result.support_margin_min_mm is not None and result.support_margin_min_mm < 0.0:
        result.issues.append(
            SimulationIssue(
                "fail",
                "static_support_margin",
                f"Configured body-frame origin leaves the stance polygon by "
                f"{-result.support_margin_min_mm:.1f} mm during a >=3-foot support sample; "
                "this is not a measured centre-of-mass verdict.",
            )
        )
    return result


def _sample_targets(
    config: RobotConfig,
    geometries: dict[str, LegGeometry],
    targets: dict[str, FootTarget],
    cmd: GaitCommand,
    result: GaitSimulationResult,
) -> None:
    stance_points: list[tuple[float, float]] = []
    stance_count = 0
    z_down = -(config.gait_seed.body_height + cmd.body_z)

    for leg, ft in targets.items():
        geom = geometries[leg]
        angles = inverse_kinematics(geom, ft.x, ft.y, ft.z)
        fk = forward_kinematics(geom, *angles)
        err = math.sqrt((fk[0] - ft.x) ** 2 + (fk[1] - ft.y) ** 2 + (fk[2] - ft.z) ** 2)
        result.max_ik_error_mm = max(result.max_ik_error_mm, err)
        result.max_abs_angle_deg = max(result.max_abs_angle_deg, max(abs(math.degrees(a)) for a in angles))
        _check_counts(config, leg, angles, result)

        if ft.in_stance:
            stance_count += 1
            stance_points.append(_body_frame_foot_xy(config, leg, ft))
        else:
            result.swing_peak_clearance_mm = max(result.swing_peak_clearance_mm, ft.z - z_down)

    result.min_stance_feet = min(result.min_stance_feet, stance_count)
    if stance_count == 2:
        result.dynamic_only_samples += 1
    elif stance_count >= 3:
        margin = _support_margin_mm(stance_points)
        if margin is not None:
            result.support_margin_min_mm = (
                margin if result.support_margin_min_mm is None else min(result.support_margin_min_mm, margin)
            )


def _check_counts(config: RobotConfig, leg: str, angles: tuple[float, float, float], result: GaitSimulationResult) -> None:
    for joint, theta in zip(JOINT_ORDER, angles):
        sid = config.id_for(leg, joint)
        if sid is None:
            continue
        spec = config.servos.get(str(sid))
        if spec is None:
            continue
        raw = _raw_count(theta, invert=spec.invert)
        result.raw_count_min = raw if result.raw_count_min is None else min(result.raw_count_min, raw)
        result.raw_count_max = raw if result.raw_count_max is None else max(result.raw_count_max, raw)
        if raw < spec.min_raw or raw > spec.max_raw:
            result.raw_limit_violations += 1


def _raw_count(theta: float, *, invert: bool = False) -> int:
    direction = -1 if invert else 1
    return round(NEUTRAL_COUNT + direction * theta * COUNTS_PER_RAD)


def _body_frame_foot_xy(config: RobotConfig, leg: str, ft: FootTarget) -> tuple[float, float]:
    sx, sy = HIP_SIGN[leg]
    hip_x = sx * config.body_mm.shoulder_lateral / 2.0
    hip_y = sy * config.body_mm.fore_aft / 2.0
    return hip_x + ft.x, hip_y + ft.y


def _support_margin_mm(points: list[tuple[float, float]]) -> float | None:
    hull = _convex_hull(points)
    if len(hull) < 3:
        return None
    origin = (0.0, 0.0)
    distances = [
        _point_segment_distance(origin, hull[i], hull[(i + 1) % len(hull)])
        for i in range(len(hull))
    ]
    margin = min(distances)
    return margin if _point_in_polygon(origin, hull) else -margin


def _convex_hull(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    pts = sorted(set(points))
    if len(pts) <= 1:
        return pts

    def cross(o: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: list[tuple[float, float]] = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper: list[tuple[float, float]] = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def _point_in_polygon(point: tuple[float, float], polygon: list[tuple[float, float]]) -> bool:
    x, y = point
    inside = False
    j = len(polygon) - 1
    for i, (xi, yi) in enumerate(polygon):
        xj, yj = polygon[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi):
            inside = not inside
        j = i
    return inside


def _point_segment_distance(
    p: tuple[float, float],
    a: tuple[float, float],
    b: tuple[float, float],
) -> float:
    px, py = p
    ax, ay = a
    bx, by = b
    dx = bx - ax
    dy = by - ay
    denom = dx * dx + dy * dy
    if denom == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def parse_gait_overrides(items: list[str]) -> dict[str, float | str]:
    overrides: dict[str, float | str] = {}
    valid = set(GaitSeed.model_fields)
    for item in items:
        key, sep, value = item.partition("=")
        if not sep:
            raise ValueError(f"expected KEY=VALUE, got {item!r}")
        if key not in valid:
            raise ValueError(f"unknown gait field {key!r}; valid fields: {', '.join(sorted(valid))}")
        if key in {"gait_type", "swing_shape"}:
            overrides[key] = value
        else:
            overrides[key] = float(value)
    return overrides


def config_with_gait_overrides(config: RobotConfig, overrides: dict[str, float | str]) -> RobotConfig:
    if not overrides:
        return config
    data = config.model_dump()
    gait = dict(data["gait_seed"])
    gait.update(overrides)
    data["gait_seed"] = gait
    return RobotConfig.model_validate(data)


def _print_text(result: GaitSimulationResult) -> None:
    print(f"DogV3 trot check: {result.status.upper()}")
    print(
        "command: "
        f"vx={result.command['vx']:.2f} vy={result.command['vy']:.2f} "
        f"wz={result.command['wz']:.2f} body_z={result.command['body_z']:.1f} mm"
    )
    print(
        "gait: "
        f"height={result.gait['body_height']} mm step={result.gait['step_length']}x{result.gait['step_height']} mm "
        f"period={result.gait['cycle_period']} s duty={result.gait['duty_factor']} shape={result.gait['swing_shape']}"
    )
    print(
        "derived: "
        f"forward_speed={result.estimated_forward_speed_mm_s:.1f} mm/s "
        f"stance={result.stance_time_s:.3f} s swing={result.swing_time_s:.3f} s"
    )
    print(
        "metrics: "
        f"ik_error={result.max_ik_error_mm:.3f} mm "
        f"max_angle={result.max_abs_angle_deg:.1f} deg "
        f"swing_peak={result.swing_peak_clearance_mm:.1f} mm "
        f"min_stance_feet={result.min_stance_feet}"
    )
    print(
        "tracking: "
        f"peak_joint={result.peak_joint_dps:.1f} deg/s "
        f"command_ceiling={result.command_ceiling_dps:.1f} deg/s "
        f"saturated={result.command_rate_saturation_pct:.1f}%"
    )
    if result.raw_count_min is not None and result.raw_count_max is not None:
        print(f"servo raw range sampled: {result.raw_count_min}..{result.raw_count_max}")
    if result.support_margin_min_mm is not None:
        print(
            "body-frame-origin support margin when >=3 feet planted: "
            f"{result.support_margin_min_mm:.1f} mm (not measured CoM)"
        )
    for issue in result.issues:
        print(f"[{issue.severity.upper()}] {issue.code}: {issue.message}")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Screen one DogV3 trot/crawl cycle from robot_config.json")
    ap.add_argument("--config", default="robot_config.example.json", help="robot_config.json path")
    ap.add_argument("--samples", type=int, default=64)
    ap.add_argument("--vx", type=float, default=0.0)
    ap.add_argument("--vy", type=float, default=1.0)
    ap.add_argument("--wz", type=float, default=0.0)
    ap.add_argument("--body-z", type=float, default=0.0)
    ap.add_argument(
        "--set",
        dest="sets",
        action="append",
        default=[],
        metavar="FIELD=VALUE",
        help="temporary gait_seed override, e.g. --set step_length=30 --set cycle_period=0.8",
    )
    ap.add_argument("--write-config", default=None, help="write the overridden config to this path")
    ap.add_argument("--json", action="store_true", help="emit JSON instead of text")
    ap.add_argument("--strict-warnings", action="store_true", help="return non-zero when warnings are present")
    args = ap.parse_args(argv)

    try:
        cfg = load_config(args.config, require_commissioned=False)
        overrides = parse_gait_overrides(args.sets)
        cfg = config_with_gait_overrides(cfg, overrides)
        if args.write_config:
            save_config(cfg, args.write_config)
        result = simulate_trot(
            cfg,
            GaitCommand(vx=args.vx, vy=args.vy, wz=args.wz, body_z=args.body_z),
            samples=args.samples,
        )
    except (ConfigError, ValidationError, ValueError) as e:
        print(f"dogv3-sim-trot: {e}", file=sys.stderr)
        return 2

    if args.json:
        print(json.dumps(result.to_dict(), indent=2))
    else:
        _print_text(result)

    if result.status == "fail":
        return 1
    if result.status == "warn" and args.strict_warnings:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
