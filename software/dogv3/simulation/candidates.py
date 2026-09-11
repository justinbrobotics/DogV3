"""Evaluate named D2 trot tuning candidates against the kinematic gate."""
from __future__ import annotations

import argparse
import csv
import json
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Literal

from pydantic import ValidationError

from ..config.loader import ConfigError, load_config, save_config
from ..config.schema import GaitSeed
from ..gait.trot import GaitCommand
from .trot_check import config_with_gait_overrides, parse_gait_overrides, simulate_trot


@dataclass(frozen=True)
class TrotCandidateSpec:
    name: str
    notes: str
    overrides: dict[str, float | str]


@dataclass(frozen=True)
class TrotCandidateResult:
    rank: int
    name: str
    status: Literal["pass", "warn", "fail"]
    notes: str
    overrides: dict[str, float | str]
    issue_codes: list[str]
    estimated_forward_speed_mm_s: float
    max_ik_error_mm: float
    max_abs_angle_deg: float
    swing_peak_clearance_mm: float
    dynamic_only_samples: int
    min_stance_feet: int
    support_margin_min_mm: float | None
    raw_limit_violations: int
    peak_joint_dps: float
    command_rate_saturation_pct: float

    def to_dict(self) -> dict:
        return asdict(self)


def load_trot_candidates(path: str | Path) -> tuple[list[TrotCandidateSpec], dict[str, Any]]:
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    if raw.get("schema_version") != 1:
        raise ValueError(f"candidate schema_version {raw.get('schema_version')} != supported 1")
    items = raw.get("candidates")
    if not isinstance(items, list) or not items:
        raise ValueError("candidate file must contain a non-empty candidates list")

    names: set[str] = set()
    candidates: list[TrotCandidateSpec] = []
    for i, item in enumerate(items):
        if not isinstance(item, dict):
            raise ValueError(f"candidate {i} must be an object")
        name = str(item.get("name", "")).strip()
        if not name:
            raise ValueError(f"candidate {i} missing name")
        if name in names:
            raise ValueError(f"duplicate candidate name {name!r}")
        names.add(name)
        overrides = _validate_json_overrides(item.get("gait_seed", {}))
        candidates.append(TrotCandidateSpec(name=name, notes=str(item.get("notes", "")), overrides=overrides))
    return candidates, raw


def evaluate_trot_candidates(
    config_path: str | Path,
    candidates_path: str | Path,
    *,
    command: GaitCommand | None = None,
    samples: int | None = None,
) -> list[TrotCandidateResult]:
    cfg = load_config(config_path, require_commissioned=False)
    candidates, raw = load_trot_candidates(candidates_path)
    command = command or _command_from_file(raw)
    samples = samples if samples is not None else int(raw.get("samples", 64))

    rows: list[TrotCandidateResult] = []
    for candidate in candidates:
        candidate_cfg = config_with_gait_overrides(cfg, candidate.overrides)
        result = simulate_trot(candidate_cfg, command, samples=samples)
        rows.append(
            TrotCandidateResult(
                rank=0,
                name=candidate.name,
                status=result.status,
                notes=candidate.notes,
                overrides=candidate.overrides,
                issue_codes=[issue.code for issue in result.issues],
                estimated_forward_speed_mm_s=result.estimated_forward_speed_mm_s,
                max_ik_error_mm=result.max_ik_error_mm,
                max_abs_angle_deg=result.max_abs_angle_deg,
                swing_peak_clearance_mm=result.swing_peak_clearance_mm,
                dynamic_only_samples=result.dynamic_only_samples,
                min_stance_feet=result.min_stance_feet,
                support_margin_min_mm=result.support_margin_min_mm,
                raw_limit_violations=result.raw_limit_violations,
                peak_joint_dps=result.peak_joint_dps,
                command_rate_saturation_pct=result.command_rate_saturation_pct,
            )
        )

    ranked = sorted(rows, key=_score)
    return [
        TrotCandidateResult(
            rank=i + 1,
            name=row.name,
            status=row.status,
            notes=row.notes,
            overrides=row.overrides,
            issue_codes=row.issue_codes,
            estimated_forward_speed_mm_s=row.estimated_forward_speed_mm_s,
            max_ik_error_mm=row.max_ik_error_mm,
            max_abs_angle_deg=row.max_abs_angle_deg,
            swing_peak_clearance_mm=row.swing_peak_clearance_mm,
            dynamic_only_samples=row.dynamic_only_samples,
            min_stance_feet=row.min_stance_feet,
            support_margin_min_mm=row.support_margin_min_mm,
            raw_limit_violations=row.raw_limit_violations,
            peak_joint_dps=row.peak_joint_dps,
            command_rate_saturation_pct=row.command_rate_saturation_pct,
        )
        for i, row in enumerate(ranked)
    ]


def write_candidate_config(
    config_path: str | Path,
    rows: list[TrotCandidateResult],
    out_path: str | Path,
    *,
    select: str | None = None,
) -> TrotCandidateResult:
    if not rows:
        raise ValueError("no candidate rows to write")
    chosen = next((row for row in rows if row.name == select), None) if select else rows[0]
    if chosen is None:
        names = ", ".join(row.name for row in rows)
        raise ValueError(f"candidate {select!r} not found; valid names: {names}")
    cfg = load_config(config_path, require_commissioned=False)
    save_config(config_with_gait_overrides(cfg, chosen.overrides), out_path)
    return chosen


def _validate_json_overrides(value: Any) -> dict[str, float | str]:
    if not isinstance(value, dict) or not value:
        raise ValueError("candidate gait_seed must be a non-empty object")
    valid = set(GaitSeed.model_fields)
    bad = sorted(set(value) - valid)
    if bad:
        raise ValueError(f"unknown gait fields: {', '.join(bad)}")
    return parse_gait_overrides([f"{key}={raw}" for key, raw in value.items()])


def _command_from_file(raw: dict[str, Any]) -> GaitCommand:
    data = raw.get("base_command", {}) or {}
    if not isinstance(data, dict):
        raise ValueError("base_command must be an object")
    return GaitCommand(
        vx=float(data.get("vx", 0.0)),
        vy=float(data.get("vy", 1.0)),
        wz=float(data.get("wz", 0.0)),
        body_z=float(data.get("body_z", 0.0)),
    )


def _score(row: TrotCandidateResult) -> tuple:
    status_rank = {"pass": 0, "warn": 1, "fail": 2}[row.status]
    support_margin = row.support_margin_min_mm if row.support_margin_min_mm is not None else -9999.0
    return (
        status_rank,
        row.raw_limit_violations,
        row.dynamic_only_samples,
        -support_margin,
        row.command_rate_saturation_pct,
        row.peak_joint_dps,
        -row.swing_peak_clearance_mm,
        row.max_ik_error_mm,
        -row.estimated_forward_speed_mm_s,
    )


def _print_text(rows: list[TrotCandidateResult], limit: int | None = None) -> None:
    shown = rows[:limit] if limit else rows
    print("rank status name speed_mm_s swing_mm dyn_samples ik_err_mm overrides issues")
    for row in shown:
        overrides = ",".join(f"{k}={v}" for k, v in row.overrides.items())
        issues = ",".join(row.issue_codes) or "-"
        print(
            f"{row.rank:>4} {row.status:<6} {row.name:<18} {row.estimated_forward_speed_mm_s:>10.1f} "
            f"{row.swing_peak_clearance_mm:>8.1f} {row.dynamic_only_samples:>11} "
            f"{row.max_ik_error_mm:>9.3f} {overrides} {issues}"
        )


def _write_csv(rows: list[TrotCandidateResult], path: str) -> None:
    fieldnames = [
        "rank",
        "status",
        "name",
        "estimated_forward_speed_mm_s",
        "max_ik_error_mm",
        "max_abs_angle_deg",
        "swing_peak_clearance_mm",
        "dynamic_only_samples",
        "min_stance_feet",
        "support_margin_min_mm",
        "raw_limit_violations",
        "peak_joint_dps",
        "command_rate_saturation_pct",
        "overrides",
        "issue_codes",
        "notes",
    ]
    with Path(path).open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            data = row.to_dict()
            data["overrides"] = json.dumps(row.overrides, sort_keys=True)
            data["issue_codes"] = ",".join(row.issue_codes)
            writer.writerow(data)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Evaluate named DogV3 trot candidates and optionally write one config")
    ap.add_argument("--config", default="robot_config.example.json")
    ap.add_argument("--candidates", default="trot_candidates.example.json")
    ap.add_argument("--samples", type=int, default=None)
    ap.add_argument("--vx", type=float, default=None)
    ap.add_argument("--vy", type=float, default=None)
    ap.add_argument("--wz", type=float, default=None)
    ap.add_argument("--body-z", type=float, default=None)
    ap.add_argument("--limit", type=int, default=None)
    ap.add_argument("--select", default=None, help="candidate name to write; default writes the top-ranked candidate")
    ap.add_argument("--write-config", default=None, help="write selected/top-ranked gait_seed into this config path")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--csv", default=None)
    ap.add_argument("--strict-warnings", action="store_true")
    args = ap.parse_args(argv)

    try:
        command = None
        if any(v is not None for v in (args.vx, args.vy, args.wz, args.body_z)):
            command = GaitCommand(
                vx=args.vx if args.vx is not None else 0.0,
                vy=args.vy if args.vy is not None else 1.0,
                wz=args.wz if args.wz is not None else 0.0,
                body_z=args.body_z if args.body_z is not None else 0.0,
            )
        rows = evaluate_trot_candidates(args.config, args.candidates, command=command, samples=args.samples)
        chosen = None
        if args.write_config:
            chosen = write_candidate_config(args.config, rows, args.write_config, select=args.select)
    except (OSError, json.JSONDecodeError, ConfigError, ValidationError, ValueError) as e:
        print(f"dogv3-sim-candidates: {e}", file=sys.stderr)
        return 2

    if args.csv:
        _write_csv(rows, args.csv)
    if args.json:
        payload: dict[str, Any] = {"candidates": [row.to_dict() for row in rows]}
        if chosen is not None:
            payload["written"] = {"name": chosen.name, "path": args.write_config}
        print(json.dumps(payload, indent=2))
    else:
        _print_text(rows, args.limit)
        if chosen is not None:
            print(f"wrote {args.write_config} from candidate {chosen.name!r} ({chosen.status})")

    best = rows[0]
    selected = chosen or (next((row for row in rows if row.name == args.select), None) if args.select else best)
    if selected is None:
        return 2
    if selected.status == "fail":
        return 1
    if selected.status == "warn" and args.strict_warnings:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
